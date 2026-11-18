/* Vorbis I decoder, written from the Vorbis I specification (xiph.org).

   A Vorbis stream sets itself up in its third header: codebooks (Huffman
   codes, some with tables of vector values), floors (the rough shape of
   the spectrum: line segments, or the response of an LSP filter),
   residues (the fine detail, cut into partitions, each coded with books
   its class chooses), mappings (which floor and residue each channel uses,
   and which channels are coupled) and modes (a block size and a mapping).
   Each sound packet names a mode. Its spectrum, channel by channel, is
   floor times residue; an inverse MDCT turns it to time, and it is
   windowed and overlapped with the packet before. */
#include "../include/ahvorbis.h"
#include "ahvorbis_internal.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define VORBIS_PI 3.14159265358979323846

enum {
    MAX_ENTRIES = 1 << 20,  /* entries in one codebook, and in all of them
                               together: each costs a byte while the setup
                               is read, and a used one 8 bytes of tree */
    MAX_VALUES = 1 << 22,   /* vector values in all the codebooks together */
    MAX_X = 65,             /* points of a type 1 floor */
    MAX_CLASSIFY = 1 << 15, /* partitions in one residue: 8 channels of 4096 lines */
};

/* ---- reading bits ------------------------------------------------------------- */

/* Vorbis packs bits from the lowest of each byte up. Reading past the end
   of the packet sets `end`, and every read after that gives 0. */
typedef struct {
    const uint8_t *p;
    size_t len, pos; /* pos counts bits */
    bool end;
} bits;

static uint32_t get(bits *b, int n) /* n = 0..32 */
{
    if (b->end) return 0;
    if (!n) return 0;
    if ((uint64_t)n > (uint64_t)b->len * 8 - b->pos) {
        b->end = true;
        b->pos = b->len * 8;
        return 0;
    }
    size_t byte = b->pos >> 3;
    uint64_t w = 0;
    for (int i = 0; i < 5 && byte + (size_t)i < b->len; i++) w |= (uint64_t)b->p[byte + (size_t)i] << (8 * i);
    w >>= b->pos & 7;
    b->pos += (size_t)n;
    return (uint32_t)(w & (((uint64_t)1 << n) - 1));
}

static int get1(bits *b)
{
    if (b->end || b->pos >= b->len * 8) {
        b->end = true;
        return 0;
    }
    int v = b->p[b->pos >> 3] >> (b->pos & 7) & 1;
    b->pos++;
    return v;
}

static int ilog(uint32_t x)
{
    int n = 0;
    while (x) {
        n++;
        x >>= 1;
    }
    return n;
}

static float unpack_float(uint32_t x)
{
    double mantissa = x & 0x1FFFFF;
    int exponent = (int)((x & 0x7FE00000) >> 21);
    if (x & 0x80000000) mantissa = -mantissa;
    return (float)ldexp(mantissa, exponent - 788);
}

/* ---- the setup ---------------------------------------------------------------- */

typedef struct {
    int dims, entries;
    int single;     /* the one used entry of a book with only one, else -1 */
    int32_t *tree;  /* two children per node: 0 none yet, > 0 a node, < 0 entry -1 - value */
    float *values;  /* dims per entry; NULL when the book has no value table */
} book;

typedef struct {
    int type;
    /* type 0 */
    int order, rate, bark_size, amp_bits, amp_offset, nbooks;
    uint8_t booklist[16];
    int *map[2]; /* per block size: a bark-scale place for each line */
    /* type 1 */
    int partitions, mult, values;
    uint8_t pclass[31];
    uint8_t cdim[16], csub[16], cmaster[16];
    int16_t subbooks[16][8];
    uint16_t x[MAX_X];
    uint8_t sorted[MAX_X], low[MAX_X], high[MAX_X];
} floor_setup;

typedef struct {
    int type;
    uint32_t begin, end, psize;
    int classes, classbook;
    int16_t books[64][8];
} residue_setup;

typedef struct {
    int submaps, steps;
    uint8_t mag[256], ang[256], mux[AHVORBIS_MAX_CHANNELS], sfloor[16], sres[16];
} mapping_setup;

typedef struct {
    int blockflag, mapping;
} mode_setup;

struct ahvorbis_decoder {
    const char *error;
    int stage; /* headers taken */
    ahvorbis_format format;
    int channels;
    int bs[2]; /* block sizes */

    book *books;
    int book_count;
    size_t values_total;
    size_t entries_total;
    floor_setup *floors;
    int floor_count;
    residue_setup *residues;
    int residue_count;
    mapping_setup *mappings;
    int mapping_count;
    mode_setup modes[64];
    int mode_count;

    /* working space, sized once the block sizes are known */
    float *saved[AHVORBIS_MAX_CHANNELS]; /* the right half of the last block, windowed */
    float *spec[AHVORBIS_MAX_CHANNELS];  /* residue, then spectrum */
    float *curve[AHVORBIS_MAX_CHANNELS]; /* floor */
    float *flat;                         /* residue type 2, all channels interleaved */
    float *time;                         /* one block from the transform */
    float *slope[2];                     /* rising half of the window, per block size */
    float *twr[2], *twi[2];              /* MDCT twiddles, per block size */
    float *fft_cos, *fft_sin, *re, *im, *u;
    unsigned short *fft_order;
    int fft_bits;
    uint8_t *classes;
    int prev_n;
    bool primed;
};

static const uint8_t channel_place[8][8] = {
    {0},
    {0, 1},
    {0, 2, 1},
    {0, 1, 2, 3},
    {0, 2, 1, 3, 4},
    {0, 2, 1, 4, 5, 3},
    {0, 2, 1, 5, 6, 4, 3},
    {0, 2, 1, 6, 7, 4, 5, 3},
};

static const uint32_t channel_mask[8] = {
    0x4,   /* centre */
    0x3,   /* left, right */
    0x7,   /* left, right, centre */
    0x33,  /* front left and right, back left and right */
    0x37,  /* front left, right and centre, back left and right */
    0x3F,  /* the same and LFE */
    0x70F, /* front left, right and centre, LFE, back centre, side left and right */
    0x63F, /* front left, right and centre, LFE, back left and right, side left and right */
};

ahvorbis_decoder *ahvorbis_decoder_new(void)
{
    return calloc(1, sizeof(ahvorbis_decoder));
}

void ahvorbis_decoder_free(ahvorbis_decoder *d)
{
    if (!d) return;
    for (int i = 0; i < d->book_count; i++) {
        free(d->books[i].tree);
        free(d->books[i].values);
    }
    free(d->books);
    for (int i = 0; i < d->floor_count; i++) {
        free(d->floors[i].map[0]);
        free(d->floors[i].map[1]);
    }
    free(d->floors);
    free(d->residues);
    free(d->mappings);
    for (int c = 0; c < AHVORBIS_MAX_CHANNELS; c++) {
        free(d->saved[c]);
        free(d->spec[c]);
        free(d->curve[c]);
    }
    free(d->flat);
    free(d->time);
    for (int k = 0; k < 2; k++) {
        free(d->slope[k]);
        free(d->twr[k]);
        free(d->twi[k]);
    }
    free(d->fft_cos);
    free(d->fft_sin);
    free(d->re);
    free(d->im);
    free(d->u);
    free(d->fft_order);
    free(d->classes);
    free(d);
}

void ahvorbis_decoder_reset(ahvorbis_decoder *d)
{
    d->primed = false;
}

const char *ahvorbis_decoder_error(const ahvorbis_decoder *d)
{
    return d->error;
}

static bool fail(ahvorbis_decoder *d, const char *why)
{
    if (!d->error) d->error = why;
    return false;
}

/* ---- codebooks ---------------------------------------------------------------- */

/* The depths, 1 to 32, at or below `depth`. */
static uint64_t depths_from(int depth)
{
    return ((((uint64_t)1 << 33) - 1) & ~(((uint64_t)1 << depth) - 1));
}

/* The depths at which a child of a node has a free place below it. */
static uint64_t child_room(int32_t child, int depth, const uint64_t *room)
{
    return child == 0 ? depths_from(depth) : child < 0 ? 0 : room[child];
}

/* Gives the used entries their codewords, as the specification says: in
   entry order, each takes the lowest codeword of its length still free
   (the leftmost free place at that depth of the tree). Each node keeps the
   depths at which its part of the tree has a free place, so finding one is
   one walk down. A complete tree of `used` leaves has used - 1 nodes, so
   more than that means the tree cannot come out complete. False when the
   lengths do not make a complete tree. */
static bool build_tree(book *k, const uint8_t *lengths, int used)
{
    uint64_t *room = malloc((size_t)used * sizeof(uint64_t));
    k->tree = calloc((size_t)used * 2, sizeof(int32_t));
    if (!room || !k->tree) {
        free(room);
        return false;
    }
    int nodes = 1;
    room[0] = depths_from(1);
    bool ok = true;
    for (int e = 0; e < k->entries && ok; e++) {
        int len = lengths[e];
        if (!len) continue;
        uint64_t want = (uint64_t)1 << len;
        if (!(room[0] & want)) {
            ok = false; /* no codeword of that length is left */
            break;
        }
        int path[33], node = 0;
        for (int depth = 0; depth < len; depth++) {
            path[depth] = node;
            int side = (child_room(k->tree[2 * node], depth + 1, room) & want) ? 0 : 1;
            int32_t *slot = &k->tree[2 * node + side];
            if (depth + 1 == len) {
                *slot = -1 - e;
                break;
            }
            if (*slot == 0) {
                if (nodes == used) {
                    ok = false;
                    break;
                }
                *slot = nodes++;
            }
            node = *slot;
        }
        for (int depth = len - 1; depth >= 0 && ok; depth--) {
            int n = path[depth];
            room[n] = child_room(k->tree[2 * n], depth + 1, room) | child_room(k->tree[2 * n + 1], depth + 1, room);
        }
    }
    if (ok && room[0]) ok = false; /* places left over: the tree is incomplete */
    free(room);
    return ok;
}

/* Reads one codeword: the entry number, or -1 at the end of the packet. */
static int decode(const book *k, bits *b)
{
    if (k->single >= 0) {
        get1(b); /* the one codeword is one bit, of either value */
        return b->end ? -1 : k->single;
    }
    if (!k->tree) return -1;
    int32_t node = 0;
    for (;;) {
        int bit = get1(b);
        if (b->end) return -1;
        int32_t c = k->tree[2 * node + bit];
        if (c < 0) return -1 - c;
        if (c == 0) return -1;
        node = c;
    }
}

/* The greatest whole number whose `dims`th power is at most `entries`. */
static uint64_t lookup1_values(uint64_t entries, int dims)
{
    uint64_t r = (uint64_t)floor(pow((double)entries, 1.0 / dims));
    for (;;) {
        /* (r + 1)^dims <= entries means r is too small. */
        uint64_t p = 1;
        int i;
        for (i = 0; i < dims && p <= entries; i++) p *= r + 1;
        if (i == dims && p <= entries) r++;
        else break;
    }
    for (;;) {
        uint64_t p = 1;
        int i;
        for (i = 0; i < dims && p <= entries; i++) p *= r;
        if (r > 1 && (i < dims || p > entries)) r--;
        else break;
    }
    return r;
}

static bool read_book(ahvorbis_decoder *d, bits *b, book *k)
{
    k->single = -1;
    if (get(b, 24) != 0x564342) return fail(d, "damaged Vorbis setup header");
    k->dims = (int)get(b, 16);
    k->entries = (int)get(b, 24);
    if (b->end || k->dims < 1 || k->entries < 1) return fail(d, "damaged Vorbis setup header");
    if (k->entries > MAX_ENTRIES || d->entries_total + (size_t)k->entries > MAX_ENTRIES)
        return fail(d, "Vorbis codebooks too big to decode");
    d->entries_total += (size_t)k->entries;
    uint8_t *lengths = malloc((size_t)k->entries);
    if (!lengths) return fail(d, "out of memory");
    if (!get1(b)) {
        bool sparse = get1(b);
        for (int e = 0; e < k->entries; e++) lengths[e] = sparse && !get1(b) ? 0 : (uint8_t)(get(b, 5) + 1);
    } else {
        int at = 0, length = (int)get(b, 5) + 1;
        while (at < k->entries && !b->end) {
            int count = (int)get(b, ilog((uint32_t)(k->entries - at)));
            if (count > k->entries - at || (count && length > 32)) {
                free(lengths);
                return fail(d, "damaged Vorbis setup header");
            }
            memset(lengths + at, length, (size_t)count);
            at += count;
            length++;
            if (length > 33) break;
        }
        if (at < k->entries) b->end = true;
    }
    int used = 0, last = -1;
    for (int e = 0; e < k->entries; e++)
        if (lengths[e]) {
            used++;
            last = e;
        }
    bool ok = !b->end;
    if (ok && used == 1) {
        /* A book with one entry: its codeword must be stated as one bit. */
        ok = lengths[last] == 1;
        k->single = last;
    } else if (ok && used > 1) {
        ok = build_tree(k, lengths, used);
    }
    free(lengths);
    if (!ok) return fail(d, "damaged Vorbis setup header");

    int type = (int)get(b, 4);
    if (type == 0) return true;
    if (type > 2) return fail(d, "damaged Vorbis setup header");
    float minimum = unpack_float(get(b, 32)), delta = unpack_float(get(b, 32));
    int value_bits = (int)get(b, 4) + 1;
    bool sequence = get1(b);
    uint64_t total = (uint64_t)k->entries * (uint64_t)k->dims;
    uint64_t count = type == 1 ? lookup1_values((uint64_t)k->entries, k->dims) : total;
    if (b->end) return fail(d, "damaged Vorbis setup header");
    if (d->values_total + total > MAX_VALUES) return fail(d, "Vorbis codebooks too big to decode");
    d->values_total += (size_t)total;
    uint32_t *mult = malloc((size_t)count * sizeof(uint32_t));
    k->values = malloc((size_t)total * sizeof(float));
    if (!mult || !k->values) {
        free(mult);
        return fail(d, "out of memory");
    }
    for (uint64_t i = 0; i < count; i++) mult[i] = get(b, value_bits);
    if (b->end) {
        free(mult);
        return fail(d, "damaged Vorbis setup header");
    }
    for (int e = 0; e < k->entries; e++) {
        float last_value = 0;
        uint64_t divisor = 1;
        for (int i = 0; i < k->dims; i++) {
            uint64_t at = type == 1 ? (uint64_t)e / divisor % count : (uint64_t)e * (uint64_t)k->dims + (uint64_t)i;
            float v = (float)mult[at] * delta + minimum + last_value;
            k->values[(size_t)e * (size_t)k->dims + (size_t)i] = v;
            if (sequence) last_value = v;
            divisor *= count;
        }
    }
    free(mult);
    return true;
}

/* ---- floors: setup ------------------------------------------------------------ */

static double bark(double x)
{
    return 13.1 * atan(.00074 * x) + 2.24 * atan(.0000000185 * x * x) + .0001 * x;
}

static bool read_floor(ahvorbis_decoder *d, bits *b, floor_setup *f)
{
    f->type = (int)get(b, 16);
    if (f->type == 0) {
        f->order = (int)get(b, 8);
        f->rate = (int)get(b, 16);
        f->bark_size = (int)get(b, 16);
        f->amp_bits = (int)get(b, 6);
        f->amp_offset = (int)get(b, 8);
        f->nbooks = (int)get(b, 4) + 1;
        for (int i = 0; i < f->nbooks; i++) {
            f->booklist[i] = (uint8_t)get(b, 8);
            if (f->booklist[i] >= d->book_count) return fail(d, "damaged Vorbis setup header");
        }
        if (b->end || f->order < 1 || f->rate < 1 || f->bark_size < 1 || f->amp_bits > 32)
            return fail(d, "damaged Vorbis setup header");
        for (int k = 0; k < 2; k++) {
            int n = d->bs[k] / 2;
            f->map[k] = malloc((size_t)n * sizeof(int));
            if (!f->map[k]) return fail(d, "out of memory");
            double scale = f->bark_size / bark(.5 * f->rate);
            for (int i = 0; i < n; i++) {
                int m = (int)floor(bark((double)f->rate * i / (2.0 * n)) * scale);
                f->map[k][i] = m < f->bark_size - 1 ? m : f->bark_size - 1;
            }
        }
        return true;
    }
    if (f->type != 1) return fail(d, "damaged Vorbis setup header");
    f->partitions = (int)get(b, 5);
    int max_class = -1;
    for (int i = 0; i < f->partitions; i++) {
        f->pclass[i] = (uint8_t)get(b, 4);
        if (f->pclass[i] > max_class) max_class = f->pclass[i];
    }
    for (int c = 0; c <= max_class; c++) {
        f->cdim[c] = (uint8_t)(get(b, 3) + 1);
        f->csub[c] = (uint8_t)get(b, 2);
        if (f->csub[c]) {
            f->cmaster[c] = (uint8_t)get(b, 8);
            if (f->cmaster[c] >= d->book_count) return fail(d, "damaged Vorbis setup header");
        }
        for (int j = 0; j < 1 << f->csub[c]; j++) {
            f->subbooks[c][j] = (int16_t)((int)get(b, 8) - 1);
            if (f->subbooks[c][j] >= d->book_count) return fail(d, "damaged Vorbis setup header");
        }
    }
    f->mult = (int)get(b, 2) + 1;
    int range_bits = (int)get(b, 4);
    f->x[0] = 0;
    f->x[1] = (uint16_t)(1 << range_bits);
    f->values = 2;
    for (int i = 0; i < f->partitions; i++) {
        int c = f->pclass[i];
        for (int j = 0; j < f->cdim[c]; j++) {
            if (f->values == MAX_X) return fail(d, "damaged Vorbis setup header");
            f->x[f->values++] = (uint16_t)get(b, range_bits);
        }
    }
    if (b->end) return fail(d, "damaged Vorbis setup header");
    /* The points in order of place, and each one's neighbours among the
       points before it in the list. */
    for (int i = 0; i < f->values; i++) f->sorted[i] = (uint8_t)i;
    for (int i = 1; i < f->values; i++)
        for (int j = i; j > 0 && f->x[f->sorted[j - 1]] >= f->x[f->sorted[j]]; j--) {
            if (f->x[f->sorted[j - 1]] == f->x[f->sorted[j]]) return fail(d, "damaged Vorbis setup header");
            uint8_t t = f->sorted[j];
            f->sorted[j] = f->sorted[j - 1];
            f->sorted[j - 1] = t;
        }
    for (int i = 2; i < f->values; i++) {
        int lo = 0, hi = 1;
        for (int j = 0; j < i; j++) {
            if (f->x[j] < f->x[i] && f->x[j] > f->x[lo]) lo = j;
            if (f->x[j] > f->x[i] && f->x[j] < f->x[hi]) hi = j;
        }
        f->low[i] = (uint8_t)lo;
        f->high[i] = (uint8_t)hi;
    }
    return true;
}

static bool read_residue(ahvorbis_decoder *d, bits *b, residue_setup *r)
{
    r->type = (int)get(b, 16);
    if (r->type > 2) return fail(d, "damaged Vorbis setup header");
    r->begin = get(b, 24);
    r->end = get(b, 24);
    r->psize = get(b, 24) + 1;
    r->classes = (int)get(b, 6) + 1;
    r->classbook = (int)get(b, 8);
    if (b->end || r->classbook >= d->book_count) return fail(d, "damaged Vorbis setup header");
    /* The classes a codeword names must fit the book's entries. */
    const book *cb = &d->books[r->classbook];
    uint64_t combos = 1;
    for (int i = 0; i < cb->dims; i++) {
        combos *= (uint64_t)r->classes;
        if (combos > (uint64_t)cb->entries) return fail(d, "damaged Vorbis setup header");
    }
    uint8_t cascade[64];
    for (int c = 0; c < r->classes; c++) {
        int low = (int)get(b, 3), high = get1(b) ? (int)get(b, 5) : 0;
        cascade[c] = (uint8_t)(high * 8 + low);
    }
    for (int c = 0; c < r->classes; c++)
        for (int pass = 0; pass < 8; pass++) {
            r->books[c][pass] = -1;
            if (cascade[c] & (1 << pass)) {
                int k = (int)get(b, 8);
                if (k >= d->book_count || !d->books[k].values) return fail(d, "damaged Vorbis setup header");
                r->books[c][pass] = (int16_t)k;
            }
        }
    if (b->end) return fail(d, "damaged Vorbis setup header");
    return true;
}

static bool read_mapping(ahvorbis_decoder *d, bits *b, mapping_setup *m)
{
    if (get(b, 16) != 0) return fail(d, "damaged Vorbis setup header");
    m->submaps = get1(b) ? (int)get(b, 4) + 1 : 1;
    m->steps = get1(b) ? (int)get(b, 8) + 1 : 0;
    int channel_bits = ilog((uint32_t)d->channels - 1);
    for (int i = 0; i < m->steps; i++) {
        m->mag[i] = (uint8_t)get(b, channel_bits);
        m->ang[i] = (uint8_t)get(b, channel_bits);
        if (m->mag[i] == m->ang[i] || m->mag[i] >= d->channels || m->ang[i] >= d->channels)
            return fail(d, "damaged Vorbis setup header");
    }
    if (get(b, 2) != 0) return fail(d, "damaged Vorbis setup header");
    for (int c = 0; c < d->channels; c++) {
        m->mux[c] = m->submaps > 1 ? (uint8_t)get(b, 4) : 0;
        if (m->mux[c] >= m->submaps) return fail(d, "damaged Vorbis setup header");
    }
    for (int s = 0; s < m->submaps; s++) {
        get(b, 8);
        m->sfloor[s] = (uint8_t)get(b, 8);
        m->sres[s] = (uint8_t)get(b, 8);
        if (m->sfloor[s] >= d->floor_count || m->sres[s] >= d->residue_count) return fail(d, "damaged Vorbis setup header");
    }
    if (b->end) return fail(d, "damaged Vorbis setup header");
    return true;
}

/* Sizes the working space and the transform's tables. */
static bool set_up_transform(ahvorbis_decoder *d)
{
    int big = d->bs[1], maxhalf = big / 4;
    for (int c = 0; c < d->channels; c++) {
        d->saved[c] = calloc((size_t)big / 2, sizeof(float));
        d->spec[c] = malloc((size_t)big / 2 * sizeof(float));
        d->curve[c] = malloc((size_t)big / 2 * sizeof(float));
        if (!d->saved[c] || !d->spec[c] || !d->curve[c]) return false;
    }
    d->flat = malloc((size_t)big / 2 * (size_t)d->channels * sizeof(float));
    d->time = malloc((size_t)big * sizeof(float));
    d->re = malloc((size_t)maxhalf * sizeof(float));
    d->im = malloc((size_t)maxhalf * sizeof(float));
    d->u = malloc((size_t)big / 2 * sizeof(float));
    d->fft_cos = malloc((size_t)maxhalf / 2 * sizeof(float));
    d->fft_sin = malloc((size_t)maxhalf / 2 * sizeof(float));
    d->fft_order = malloc((size_t)maxhalf * sizeof(unsigned short));
    d->classes = malloc(MAX_CLASSIFY);
    if (!d->flat || !d->time || !d->re || !d->im || !d->u || !d->fft_cos || !d->fft_sin || !d->fft_order || !d->classes) return false;
    for (int k = 0; k < 2; k++) {
        int half = d->bs[k] / 2, lines = half;
        d->slope[k] = malloc((size_t)half * sizeof(float));
        d->twr[k] = malloc((size_t)lines / 2 * sizeof(float));
        d->twi[k] = malloc((size_t)lines / 2 * sizeof(float));
        if (!d->slope[k] || !d->twr[k] || !d->twi[k]) return false;
        for (int i = 0; i < half; i++) {
            double s = sin((i + 0.5) / half * VORBIS_PI / 2);
            d->slope[k][i] = (float)sin(VORBIS_PI / 2 * s * s);
        }
        for (int j = 0; j < lines / 2; j++) {
            double angle = VORBIS_PI * (8 * j + 1) / (8.0 * lines);
            d->twr[k][j] = (float)cos(angle);
            d->twi[k][j] = (float)-sin(angle);
        }
    }
    d->fft_bits = ilog((uint32_t)maxhalf) - 1;
    for (int k = 0; k < maxhalf / 2; k++) {
        d->fft_cos[k] = (float)cos(2 * VORBIS_PI * k / maxhalf);
        d->fft_sin[k] = (float)sin(2 * VORBIS_PI * k / maxhalf);
    }
    for (int i = 0; i < maxhalf; i++) {
        int r = 0;
        for (int bit = 0; bit < d->fft_bits; bit++)
            if (i & (1 << bit)) r |= (maxhalf >> 1) >> bit;
        d->fft_order[i] = (unsigned short)r;
    }
    return true;
}

static bool read_identification(ahvorbis_decoder *d, bits *b)
{
    uint32_t version = get(b, 32);
    d->channels = (int)get(b, 8);
    uint32_t rate = get(b, 32);
    int32_t max_rate = (int32_t)get(b, 32), nominal = (int32_t)get(b, 32), min_rate = (int32_t)get(b, 32);
    int e0 = (int)get(b, 4), e1 = (int)get(b, 4);
    bool framing = get1(b);
    if (b->end || version != 0 || !framing || d->channels < 1 || rate < 1 || rate > 0x7FFFFFFF || e0 < 6 || e1 > 13 || e0 > e1)
        return fail(d, "damaged Vorbis header");
    if (d->channels > AHVORBIS_MAX_CHANNELS) return fail(d, "Vorbis with more than 8 channels is not supported");
    d->bs[0] = 1 << e0;
    d->bs[1] = 1 << e1;
    d->format.sample_rate = (int)rate;
    d->format.channels = d->channels;
    d->format.channel_mask = channel_mask[d->channels - 1];
    d->format.block_short = d->bs[0];
    d->format.block_long = d->bs[1];
    d->format.bitrate = nominal > 0 ? nominal : max_rate > 0 && min_rate > 0 ? (int)(((int64_t)max_rate + min_rate) / 2) : 0;
    if (!set_up_transform(d)) return fail(d, "out of memory");
    return true;
}

static bool read_setup(ahvorbis_decoder *d, bits *b)
{
    int count = (int)get(b, 8) + 1;
    d->books = calloc((size_t)count, sizeof(book));
    if (!d->books) return fail(d, "out of memory");
    for (int i = 0; i < count; i++) {
        d->book_count = i + 1;
        if (!read_book(d, b, &d->books[i])) return false;
    }
    count = (int)get(b, 6) + 1;
    for (int i = 0; i < count; i++)
        if (get(b, 16) != 0) return fail(d, "damaged Vorbis setup header");
    count = (int)get(b, 6) + 1;
    d->floors = calloc((size_t)count, sizeof(floor_setup));
    if (!d->floors) return fail(d, "out of memory");
    for (int i = 0; i < count; i++) {
        d->floor_count = i + 1;
        if (!read_floor(d, b, &d->floors[i])) return false;
    }
    count = (int)get(b, 6) + 1;
    d->residues = calloc((size_t)count, sizeof(residue_setup));
    if (!d->residues) return fail(d, "out of memory");
    d->residue_count = count;
    for (int i = 0; i < count; i++)
        if (!read_residue(d, b, &d->residues[i])) return false;
    count = (int)get(b, 6) + 1;
    d->mappings = calloc((size_t)count, sizeof(mapping_setup));
    if (!d->mappings) return fail(d, "out of memory");
    d->mapping_count = count;
    for (int i = 0; i < count; i++)
        if (!read_mapping(d, b, &d->mappings[i])) return false;
    d->mode_count = (int)get(b, 6) + 1;
    for (int i = 0; i < d->mode_count; i++) {
        mode_setup *m = &d->modes[i];
        m->blockflag = get1(b);
        uint32_t window = get(b, 16), transform = get(b, 16);
        m->mapping = (int)get(b, 8);
        if (window || transform || m->mapping >= d->mapping_count) return fail(d, "damaged Vorbis setup header");
    }
    if (!get1(b) || b->end) return fail(d, "damaged Vorbis setup header");
    return true;
}

bool ahvorbis_decoder_header(ahvorbis_decoder *d, const uint8_t *packet, size_t len, ahvorbis_format *format)
{
    static const uint8_t types[3] = {1, 3, 5};
    if (d->error) return false;
    if (d->stage >= 3) return fail(d, "a fourth Vorbis header");
    if (len < 7 || packet[0] != types[d->stage] || memcmp(packet + 1, "vorbis", 6))
        return fail(d, d->stage ? "a Vorbis header is missing" : "not a Vorbis stream");
    bits b = {packet + 7, len - 7, 0, false};
    bool ok = d->stage == 0 ? read_identification(d, &b) : d->stage == 1 ? true : read_setup(d, &b);
    if (!ok) return false;
    d->stage++;
    if (format) *format = d->format;
    return true;
}

/* ---- floors: decode ----------------------------------------------------------- */

const float ahvorbis_inverse_db[256] = {
    1.0649863e-07f, 1.1341951e-07f, 1.2079015e-07f, 1.2863978e-07f,
    1.3699951e-07f, 1.4590251e-07f, 1.5538408e-07f, 1.6548181e-07f,
    1.7623575e-07f, 1.8768855e-07f, 1.9988561e-07f, 2.1287530e-07f,
    2.2670913e-07f, 2.4144197e-07f, 2.5713223e-07f, 2.7384213e-07f,
    2.9163793e-07f, 3.1059021e-07f, 3.3077411e-07f, 3.5226968e-07f,
    3.7516214e-07f, 3.9954229e-07f, 4.2550680e-07f, 4.5315863e-07f,
    4.8260743e-07f, 5.1396998e-07f, 5.4737065e-07f, 5.8294187e-07f,
    6.2082472e-07f, 6.6116941e-07f, 7.0413592e-07f, 7.4989464e-07f,
    7.9862701e-07f, 8.5052630e-07f, 9.0579828e-07f, 9.6466216e-07f,
    1.0273513e-06f, 1.0941144e-06f, 1.1652161e-06f, 1.2409384e-06f,
    1.3215816e-06f, 1.4074654e-06f, 1.4989305e-06f, 1.5963394e-06f,
    1.7000785e-06f, 1.8105592e-06f, 1.9282195e-06f, 2.0535261e-06f,
    2.1869758e-06f, 2.3290978e-06f, 2.4804557e-06f, 2.6416497e-06f,
    2.8133190e-06f, 2.9961443e-06f, 3.1908506e-06f, 3.3982101e-06f,
    3.6190449e-06f, 3.8542308e-06f, 4.1047004e-06f, 4.3714470e-06f,
    4.6555282e-06f, 4.9580707e-06f, 5.2802740e-06f, 5.6234160e-06f,
    5.9888572e-06f, 6.3780469e-06f, 6.7925283e-06f, 7.2339451e-06f,
    7.7040476e-06f, 8.2047000e-06f, 8.7378876e-06f, 9.3057248e-06f,
    9.9104632e-06f, 1.0554501e-05f, 1.1240392e-05f, 1.1970856e-05f,
    1.2748789e-05f, 1.3577278e-05f, 1.4459606e-05f, 1.5399272e-05f,
    1.6400004e-05f, 1.7465768e-05f, 1.8600792e-05f, 1.9809576e-05f,
    2.1096914e-05f, 2.2467911e-05f, 2.3928002e-05f, 2.5482978e-05f,
    2.7139006e-05f, 2.8902651e-05f, 3.0780908e-05f, 3.2781225e-05f,
    3.4911534e-05f, 3.7180282e-05f, 3.9596466e-05f, 4.2169667e-05f,
    4.4910090e-05f, 4.7828601e-05f, 5.0936773e-05f, 5.4246931e-05f,
    5.7772202e-05f, 6.1526565e-05f, 6.5524908e-05f, 6.9783085e-05f,
    7.4317983e-05f, 7.9147585e-05f, 8.4291040e-05f, 8.9768747e-05f,
    9.5602426e-05f, 0.00010181521f, 0.00010843174f, 0.00011547824f,
    0.00012298267f, 0.00013097477f, 0.00013948625f, 0.00014855085f,
    0.00015820453f, 0.00016848555f, 0.00017943469f, 0.00019109536f,
    0.00020351382f, 0.00021673929f, 0.00023082423f, 0.00024582449f,
    0.00026179955f, 0.00027881276f, 0.00029693158f, 0.00031622787f,
    0.00033677814f, 0.00035866388f, 0.00038197188f, 0.00040679456f,
    0.00043323036f, 0.00046138411f, 0.00049136745f, 0.00052329927f,
    0.00055730621f, 0.00059352311f, 0.00063209358f, 0.00067317058f,
    0.00071691700f, 0.00076350630f, 0.00081312324f, 0.00086596457f,
    0.00092223983f, 0.00098217216f, 0.0010459992f, 0.0011139742f,
    0.0011863665f, 0.0012634633f, 0.0013455702f, 0.0014330129f,
    0.0015261382f, 0.0016253153f, 0.0017309374f, 0.0018434235f,
    0.0019632195f, 0.0020908006f, 0.0022266726f, 0.0023713743f,
    0.0025254795f, 0.0026895994f, 0.0028643847f, 0.0030505286f,
    0.0032487691f, 0.0034598925f, 0.0036847358f, 0.0039241906f,
    0.0041792066f, 0.0044507950f, 0.0047400328f, 0.0050480668f,
    0.0053761186f, 0.0057254891f, 0.0060975636f, 0.0064938176f,
    0.0069158225f, 0.0073652516f, 0.0078438871f, 0.0083536271f,
    0.0088964928f, 0.009474637f, 0.010090352f, 0.010746080f,
    0.011444421f, 0.012188144f, 0.012980198f, 0.013823725f,
    0.014722068f, 0.015678791f, 0.016697687f, 0.017782797f,
    0.018938423f, 0.020169149f, 0.021479854f, 0.022875735f,
    0.024362330f, 0.025945531f, 0.027631618f, 0.029427276f,
    0.031339626f, 0.033376252f, 0.035545228f, 0.037855157f,
    0.040315199f, 0.042935108f, 0.045725273f, 0.048696758f,
    0.051861348f, 0.055231591f, 0.058820850f, 0.062643361f,
    0.066714279f, 0.071049749f, 0.075666962f, 0.080584227f,
    0.085821044f, 0.091398179f, 0.097337747f, 0.10366330f,
    0.11039993f, 0.11757434f, 0.12521498f, 0.13335215f,
    0.14201813f, 0.15124727f, 0.16107617f, 0.17154380f,
    0.18269168f, 0.19456402f, 0.20720788f, 0.22067342f,
    0.23501402f, 0.25028656f, 0.26655159f, 0.28387361f,
    0.30232132f, 0.32196786f, 0.34289114f, 0.36517414f,
    0.38890521f, 0.41417847f, 0.44109412f, 0.46975890f,
    0.50028648f, 0.53279791f, 0.56742212f, 0.60429640f,
    0.64356699f, 0.68538959f, 0.72993007f, 0.77736504f,
    0.82788260f, 0.88168307f, 0.9389798f, 1.0f,
};

static float db_value(int y)
{
    return ahvorbis_inverse_db[y < 0 ? 0 : y > 255 ? 255 : y];
}

static void render_line(int x0, int y0, int x1, int y1, float *v, int n)
{
    int dy = y1 - y0, adx = x1 - x0, ady = abs(dy), base = dy / adx;
    int sy = dy < 0 ? base - 1 : base + 1, y = y0, err = 0;
    ady -= abs(base) * adx;
    if (x0 < n) v[x0] = db_value(y);
    for (int x = x0 + 1; x < x1 && x < n; x++) {
        err += ady;
        if (err >= adx) {
            err -= adx;
            y += sy;
        } else {
            y += base;
        }
        v[x] = db_value(y);
    }
}

static int render_point(int x0, int y0, int x1, int y1, int x)
{
    /* Wide: damaged heights can be large. */
    int64_t dy = (int64_t)y1 - y0, off = (dy < 0 ? -dy : dy) * (x - x0) / (x1 - x0);
    return (int)(dy < 0 ? y0 - off : y0 + off);
}

/* Decodes a type 1 floor into its curve. False when the channel is unused
   in this packet. */
static bool floor1_decode(const ahvorbis_decoder *d, const floor_setup *f, bits *b, float *out, int n)
{
    static const int ranges[4] = {256, 128, 86, 64};
    if (!get1(b)) return false;
    int range = ranges[f->mult - 1], ybits = ilog((uint32_t)range - 1);
    int y[MAX_X];
    y[0] = (int)get(b, ybits);
    y[1] = (int)get(b, ybits);
    int offset = 2;
    for (int p = 0; p < f->partitions; p++) {
        int c = f->pclass[p], cbits = f->csub[c], csub = (1 << cbits) - 1, cval = 0;
        if (cbits) {
            cval = decode(&d->books[f->cmaster[c]], b);
            if (cval < 0) return false;
        }
        for (int j = 0; j < f->cdim[c]; j++) {
            int k = f->subbooks[c][cval & csub];
            cval >>= cbits;
            y[offset + j] = 0;
            if (k >= 0) {
                y[offset + j] = decode(&d->books[k], b);
                if (y[offset + j] < 0) return false;
            }
        }
        offset += f->cdim[c];
    }
    if (b->end) return false;

    /* Step 1: the values are differences from the line through the points
       before; turn them into heights. */
    int fy[MAX_X];
    bool step2[MAX_X];
    fy[0] = y[0];
    fy[1] = y[1];
    step2[0] = step2[1] = true;
    for (int i = 2; i < f->values; i++) {
        int lo = f->low[i], hi = f->high[i];
        int predicted = render_point(f->x[lo], fy[lo], f->x[hi], fy[hi], f->x[i]);
        int val = y[i], highroom = range - predicted, lowroom = predicted;
        int room = (highroom < lowroom ? highroom : lowroom) * 2;
        if (val) {
            step2[lo] = step2[hi] = step2[i] = true;
            if (val >= room) fy[i] = highroom > lowroom ? val - lowroom + predicted : predicted - val + highroom - 1;
            else fy[i] = (val & 1) ? predicted - (val + 1) / 2 : predicted + val / 2;
        } else {
            step2[i] = false;
            fy[i] = predicted;
        }
    }
    for (int i = 0; i < f->values; i++) fy[i] = fy[i] < 0 ? 0 : fy[i] >= range ? range - 1 : fy[i];

    /* Step 2: lines between the points that count, in order of place. */
    int px[MAX_X], py[MAX_X], count = 0;
    for (int i = 0; i < f->values; i++) {
        int k = f->sorted[i];
        if (step2[k]) {
            px[count] = f->x[k];
            py[count++] = fy[k] * f->mult;
        }
    }
    ahvorbis_floor1_lines(px, py, count, out, n);
    return true;
}

void ahvorbis_floor1_lines(const int *x, const int *y, int count, float *out, int n)
{
    for (int i = 1; i < count; i++) render_line(x[i - 1], y[i - 1], x[i], y[i], out, n);
    if (x[count - 1] < n) render_line(x[count - 1], y[count - 1], n, y[count - 1], out, n);
}

/* Decodes a type 0 floor into its curve: the response of an LSP filter,
   on a bark scale. 1 when the channel is used in this packet, 0 when not,
   -1 when the packet names a book the floor does not have: the
   specification calls the packet undecodable. */
static int floor0_decode(const ahvorbis_decoder *d, const floor_setup *f, bits *b, int blk, float *out, int n)
{
    uint32_t amp = get(b, f->amp_bits);
    if (b->end || !amp) return 0;
    int which = (int)get(b, ilog((uint32_t)f->nbooks));
    if (b->end) return 0;
    if (which >= f->nbooks) return -1;
    const book *k = &d->books[f->booklist[which]];
    if (!k->values) return 0;
    double coef[256];
    int count = 0;
    float last = 0;
    while (count < f->order) {
        int e = decode(k, b);
        if (e < 0) return 0;
        const float *v = k->values + (size_t)e * (size_t)k->dims;
        for (int j = 0; j < k->dims && count + j < f->order; j++) coef[count + j] = cos((double)(v[j] + last));
        last += v[k->dims - 1];
        count += k->dims;
    }
    const int *map = f->map[blk];
    double top = (double)amp * f->amp_offset / (ldexp(1.0, f->amp_bits) - 1);
    int i = 0;
    while (i < n) {
        double w = cos(VORBIS_PI * map[i] / f->bark_size), p, q;
        if (f->order & 1) {
            p = 1 - w * w;
            q = 0.25;
        } else {
            p = (1 - w) / 2;
            q = (1 + w) / 2;
        }
        for (int j = 0; 2 * j + 1 < f->order; j++) p *= 4 * (coef[2 * j + 1] - w) * (coef[2 * j + 1] - w);
        for (int j = 0; 2 * j < f->order; j++) q *= 4 * (coef[2 * j] - w) * (coef[2 * j] - w);
        float value = p + q > 0 ? (float)exp(.11512925 * (top / sqrt(p + q) - f->amp_offset)) : 0.0f;
        int here = map[i];
        do out[i++] = value;
        while (i < n && map[i] == here);
    }
    return 1;
}

/* ---- residues ------------------------------------------------------------------ */

/* Adds one partition's vectors from book k into v[0 .. psize). False at the
   end of the packet. */
static bool vq_partition(const book *k, bits *b, float *v, size_t psize, int type)
{
    size_t dims = (size_t)k->dims;
    if (type == 0) {
        size_t step = psize / dims;
        for (size_t i = 0; i < step; i++) {
            int e = decode(k, b);
            if (e < 0) return false;
            const float *val = k->values + (size_t)e * dims;
            for (size_t j = 0; j < dims; j++) v[i + j * step] += val[j];
        }
    } else {
        for (size_t i = 0; i < psize;) {
            int e = decode(k, b);
            if (e < 0) return false;
            const float *val = k->values + (size_t)e * dims;
            for (size_t j = 0; j < dims && i < psize; j++) v[i++] += val[j];
        }
    }
    return true;
}

/* The passes over `count` vectors of `size` values: classes first, then
   each partition with the book its class names for the pass. An end of
   the packet stops it, keeping what was decoded. */
static void residue_passes(ahvorbis_decoder *d, const residue_setup *r, bits *b, float **v, const bool *skip, int count,
                           size_t size, int type)
{
    size_t begin = r->begin < size ? r->begin : size, end = r->end < size ? r->end : size;
    if (end <= begin) return;
    size_t parts = (end - begin) / r->psize;
    if (!parts) return;
    const book *cb = &d->books[r->classbook];
    int per_word = cb->dims;
    uint8_t *cls = d->classes;
    for (int pass = 0; pass < 8; pass++) {
        size_t p = 0;
        while (p < parts) {
            if (pass == 0) {
                for (int j = 0; j < count; j++) {
                    if (skip[j]) continue;
                    int t = decode(cb, b);
                    if (t < 0) return;
                    for (int i = per_word - 1; i >= 0; i--) {
                        if (p + (size_t)i < parts) cls[(size_t)j * parts + p + (size_t)i] = (uint8_t)(t % r->classes);
                        t /= r->classes;
                    }
                }
            }
            for (int i = 0; i < per_word && p < parts; i++, p++) {
                for (int j = 0; j < count; j++) {
                    if (skip[j]) continue;
                    int k = r->books[cls[(size_t)j * parts + p]][pass];
                    if (k < 0) continue;
                    if (!vq_partition(&d->books[k], b, v[j] + begin + p * r->psize, r->psize, type)) return;
                }
            }
        }
    }
}

static void residue_decode(ahvorbis_decoder *d, const residue_setup *r, bits *b, const int *ids, const bool *skip, int count,
                           int half)
{
    for (int i = 0; i < count; i++) memset(d->spec[ids[i]], 0, (size_t)half * sizeof(float));
    if (r->type == 2) {
        /* All the vectors as one, interleaved; decoded unless all are to be
           left out. */
        bool any = false;
        for (int i = 0; i < count; i++) any |= !skip[i];
        if (!any) return;
        size_t size = (size_t)half * (size_t)count;
        memset(d->flat, 0, size * sizeof(float));
        float *one[1] = {d->flat};
        bool none[1] = {false};
        residue_passes(d, r, b, one, none, 1, size, 1);
        for (size_t i = 0; i < (size_t)half; i++)
            for (int j = 0; j < count; j++) d->spec[ids[j]][i] = d->flat[i * (size_t)count + (size_t)j];
        return;
    }
    float *v[AHVORBIS_MAX_CHANNELS];
    for (int i = 0; i < count; i++) v[i] = d->spec[ids[i]];
    residue_passes(d, r, b, v, skip, count, (size_t)half, r->type);
}

/* ---- the transform ------------------------------------------------------------- */

/* out[n] = sum of X[k] cos(pi/M (n + 1/2 + M/2)(k + 1/2)) over the M = N/2
   lines, for n = 0 .. N-1.

   The N outputs are a type IV cosine transform of the lines, unfolded. That
   transform comes from one complex FFT of M/2 points: pair the lines up as
   X[2j] + i X[M-1-2j], turn each by exp(-i pi (8j + 1) / 8M), transform,
   turn again, and the real and imaginary parts are its outputs. */
static void imdct(const ahvorbis_decoder *d, int blk, const float *X, float *out)
{
    int M = d->bs[blk] / 2, half = M / 2, shift = d->fft_bits - (ilog((uint32_t)half) - 1);
    int maxhalf = d->bs[1] / 4;
    const float *wr = d->twr[blk], *wi = d->twi[blk];
    float *re = d->re, *im = d->im, *u = d->u;
    for (int j = 0; j < half; j++) {
        float a = X[2 * j], b = X[M - 1 - 2 * j];
        int k = d->fft_order[j] >> shift;
        re[k] = a * wr[j] - b * wi[j];
        im[k] = a * wi[j] + b * wr[j];
    }
    for (int len = 2; len <= half; len <<= 1) {
        int h = len / 2, step = maxhalf / len;
        for (int i = 0; i < half; i += len) {
            for (int k = 0; k < h; k++) {
                float c = d->fft_cos[k * step], s = -d->fft_sin[k * step];
                float tr = re[i + k + h] * c - im[i + k + h] * s, ti = re[i + k + h] * s + im[i + k + h] * c;
                re[i + k + h] = re[i + k] - tr;
                im[i + k + h] = im[i + k] - ti;
                re[i + k] += tr;
                im[i + k] += ti;
            }
        }
    }
    for (int n = 0; n < half; n++) {
        u[2 * n] = re[n] * wr[n] - im[n] * wi[n];
        u[M - 1 - 2 * n] = -(re[n] * wi[n] + im[n] * wr[n]);
    }
    for (int n = 0; n < half; n++) out[n] = u[n + half];
    for (int n = half; n < 3 * half; n++) out[n] = -u[3 * half - 1 - n];
    for (int n = 3 * half; n < 4 * half; n++) out[n] = -u[n - 3 * half];
}

/* Multiplies a block by its window: zero, the rising slope, one, the
   falling slope, zero. A long block next to a short one takes the short
   slope on that side. */
static void apply_window(const ahvorbis_decoder *d, int blk, bool prev_long, bool next_long, float *t)
{
    int n = d->bs[blk], short_n = d->bs[0];
    int left_start = 0, left_n = n / 2, right_start = n / 2, right_n = n / 2;
    const float *left = d->slope[blk], *right = d->slope[blk];
    if (blk && !prev_long) {
        left_start = n / 4 - short_n / 4;
        left_n = short_n / 2;
        left = d->slope[0];
    }
    if (blk && !next_long) {
        right_start = n * 3 / 4 - short_n / 4;
        right_n = short_n / 2;
        right = d->slope[0];
    }
    for (int i = 0; i < left_start; i++) t[i] = 0;
    for (int i = 0; i < left_n; i++) t[left_start + i] *= left[i];
    for (int i = 0; i < right_n; i++) t[right_start + i] *= right[right_n - 1 - i];
    for (int i = right_start + right_n; i < n; i++) t[i] = 0;
}

/* ---- sound packets ------------------------------------------------------------- */

bool ahvorbis_decode_packet(ahvorbis_decoder *d, const uint8_t *data, size_t len, float *pcm, int *samples)
{
    *samples = 0;
    if (d->stage < 3) return false;
    bits b = {data, len, 0, false};
    if (get1(&b) || b.end) return false; /* not a sound packet */
    int m = (int)get(&b, ilog((uint32_t)d->mode_count - 1));
    if (b.end || m >= d->mode_count) return false;
    int blk = d->modes[m].blockflag;
    bool prev_long = true, next_long = true;
    if (blk) {
        prev_long = get1(&b);
        next_long = get1(&b);
    }
    if (b.end) return false;
    const mapping_setup *mp = &d->mappings[d->modes[m].mapping];
    int n = d->bs[blk], half = n / 2, channels = d->channels;

    /* Floors, channel by channel. */
    bool unused[AHVORBIS_MAX_CHANNELS], no_residue[AHVORBIS_MAX_CHANNELS];
    for (int c = 0; c < channels; c++) {
        const floor_setup *f = &d->floors[mp->sfloor[mp->mux[c]]];
        int used = f->type == 0 ? floor0_decode(d, f, &b, blk, d->curve[c], half) : floor1_decode(d, f, &b, d->curve[c], half);
        if (used < 0) return false;
        unused[c] = no_residue[c] = !used;
    }
    /* A coupled pair needs both residues if either channel is used. */
    for (int i = 0; i < mp->steps; i++)
        if (!no_residue[mp->mag[i]] || !no_residue[mp->ang[i]]) no_residue[mp->mag[i]] = no_residue[mp->ang[i]] = false;
    /* Residues, submap by submap. */
    for (int s = 0; s < mp->submaps; s++) {
        int ids[AHVORBIS_MAX_CHANNELS], count = 0;
        bool skip[AHVORBIS_MAX_CHANNELS];
        for (int c = 0; c < channels; c++)
            if (mp->mux[c] == s) {
                ids[count] = c;
                skip[count] = no_residue[c];
                count++;
            }
        residue_decode(d, &d->residues[mp->sres[s]], &b, ids, skip, count, half);
    }
    /* Undo the coupling, last step first. */
    for (int i = mp->steps - 1; i >= 0; i--) {
        float *mag = d->spec[mp->mag[i]], *ang = d->spec[mp->ang[i]];
        for (int j = 0; j < half; j++) {
            float M = mag[j], A = ang[j], newm, newa;
            if (M > 0) {
                if (A > 0) {
                    newm = M;
                    newa = M - A;
                } else {
                    newa = M;
                    newm = M + A;
                }
            } else {
                if (A > 0) {
                    newm = M;
                    newa = M + A;
                } else {
                    newa = M;
                    newm = M - A;
                }
            }
            mag[j] = newm;
            ang[j] = newa;
        }
    }
    /* Spectrum = floor times residue; then to time, windowed, and laid
       over the right half of the block before. */
    int prev_n = d->prev_n, out_n = d->primed ? prev_n / 4 + n / 4 : 0;
    const uint8_t *place = channel_place[channels - 1];
    for (int c = 0; c < channels; c++) {
        float *s = d->spec[c], *t = d->time;
        if (unused[c]) {
            memset(s, 0, (size_t)half * sizeof(float));
        } else {
            for (int j = 0; j < half; j++) s[j] *= d->curve[c][j];
        }
        imdct(d, blk, s, t);
        apply_window(d, blk, prev_long, next_long, t);
        float *saved = d->saved[c];
        for (int i = 0; i < out_n; i++) {
            float v = i < prev_n / 2 ? saved[i] : 0;
            int y = i - prev_n / 4 + n / 4;
            if (y >= 0) v += t[y];
            pcm[(size_t)i * (size_t)channels + place[c]] = v;
        }
        memcpy(saved, t + half, (size_t)half * sizeof(float));
    }
    d->prev_n = n;
    d->primed = true;
    *samples = out_n;
    return true;
}
