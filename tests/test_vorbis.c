/* Tests for Vorbis and Ogg. A small Vorbis writer here makes random
   streams that use every part of the format, far beyond what libvorbis
   writes: both floor types, the three residue types, books sparse and
   ordered and with one entry, both kinds of value table, several submaps,
   coupling, many modes. A small Ogg writer cuts the packets into pages at
   random. ffmpeg must decode each stream as this decoder does. Files from
   libvorbis, and FLAC in Ogg, are checked too. */
#include "../include/ahogg.h"
#include "../include/ahreader.h"
#include "../include/ahvorbis.h"
#include "../src/ahvorbis_internal.h"
#include "common.h"

/* ---- writing bits, lowest bit first -------------------------------------------- */

typedef struct {
    bytes b;
    unsigned acc;
    int n;
} bitw;

static void put(bitw *w, uint64_t v, int n)
{
    for (int i = 0; i < n; i++) {
        w->acc |= (unsigned)((v >> i) & 1) << w->n;
        if (++w->n == 8) {
            unsigned char c = (unsigned char)w->acc;
            bytes_put(&w->b, &c, 1);
            w->n = 0;
            w->acc = 0;
        }
    }
}

static void put_text(bitw *w, const char *s, size_t n)
{
    for (size_t i = 0; i < n; i++) put(w, (unsigned char)s[i], 8);
}

static void finish(bitw *w)
{
    if (w->n) put(w, 0, 8 - w->n);
}

static int ilog(uint64_t x)
{
    int n = 0;
    while (x) {
        n++;
        x >>= 1;
    }
    return n;
}

/* ---- codebooks ------------------------------------------------------------------ */

typedef struct {
    int dims, entries, lookup;
    uint8_t *len; /* 0: unused */
    uint32_t *code;
    int *used;
    int nused;
    float *values;
    bool ordered, sparse;
    uint32_t min_packed, delta_packed;
    int value_bits;
    bool seq;
    uint32_t *mult;
    int nmult;
    /* For a book with a value table of type 2 (which ffmpeg does not
       read): the same vectors as a type 1 book, for the twin stream. */
    int twin_entries, twin_nused;
    uint8_t *twin_len;
    uint32_t *twin_code;
    int *twin_of; /* each entry's number in the twin */
    bool scalar;  /* read as a number: a value table it has goes unused, and
                     the twin leaves it out (ffmpeg numbers the entries of
                     such a book as if the unused ones were not there) */
} wbook;

enum { W_BOOKS = 256, W_FLOORS = 8, W_RESIDUES = 40, W_MAPS = 4, W_MODES = 6 };

typedef struct {
    int type;
    int order, rate, bark, amp_bits, amp_offset, nbooks, booklist[16];
    int partitions, pclass[31], maxclass, cdim[16], csub[16], cmaster[16], sub[16][8], mult, rangebits, values, x[65];
} wfloor;

typedef struct {
    int type, begin, end, psize, classes, classbook, cascade[64], books[64][8];
    int twin_begin, twin_end; /* the range limited to the vector, as the
                                 specification limits it */
} wresidue;

typedef struct {
    int submaps, steps, mag[256], ang[256], mux[8], sfloor[16], sres[16];
} wmapping;

typedef struct {
    int channels, rate, e0, e1;
    wbook books[W_BOOKS];
    int nbooks;
    wfloor floors[W_FLOORS];
    int nfloors;
    wresidue res[W_RESIDUES];
    int nres;
    wmapping maps[W_MAPS];
    int map_block[W_MAPS];
    int nmaps;
    int blockflag[W_MODES], mapping[W_MODES];
    int nmodes;
} wsetup;

/* What the random streams used, to check that every part was reached. */
static int g_floor0, g_floor1, g_residue[3], g_lookup[3], g_sparse, g_ordered, g_single, g_submaps, g_coupled, g_unused,
    g_unused_pair, g_switches, g_classes_multi, g_seq, g_no_book, g_cascade_late, g_empty_submap;

/* Writing the twin of a stream: the same sound in only what ffmpeg reads. */
static bool g_twin;
static int g_type2, g_one_bit, g_res_over, g_twins, g_scalar_table, g_chained;
/* Sound packets on the first page of sound (libvorbis puts one there); 0:
   no page end forced. */
static int g_first_page = 1;
/* The samples each sound packet of the last stream built gives. */
static int g_packet_samples[64];
/* Floor 0 packets name a book past the floor's list: the packet cannot be
   decoded. g_packet_bad says which packets of the last stream do. */
static bool g_bad_floor0, g_packet_bad[64];
static int g_bad_written;

static void free_setup(wsetup *s)
{
    for (int i = 0; i < s->nbooks; i++) {
        wbook *k = &s->books[i];
        free(k->len);
        free(k->code);
        free(k->used);
        free(k->values);
        free(k->mult);
        free(k->twin_len);
        free(k->twin_code);
        free(k->twin_of);
    }
    memset(s, 0, sizeof(*s));
}

/* A tree that gives codewords as the specification does: each used entry,
   in order, takes the lowest free codeword of its length. */
typedef struct {
    int child[2]; /* 0 none, -1 a leaf, else a node */
    bool full;
} tnode;

static bool tree_full(const tnode *t, int c) { return c == -1 || (c > 0 && t[c].full); }

static bool place(tnode *t, int *count, int node, int depth, int len, uint32_t prefix, uint32_t *code)
{
    for (int side = 0; side < 2; side++) {
        int c = t[node].child[side];
        if (c == -1) continue;
        if (depth + 1 == len) {
            if (c != 0) continue;
            t[node].child[side] = -1;
            *code = prefix << 1 | (uint32_t)side;
        } else {
            if (c == 0) {
                c = (*count)++;
                t[c].child[0] = t[c].child[1] = 0;
                t[c].full = false;
                t[node].child[side] = c;
            }
            if (t[c].full || !place(t, count, c, depth + 1, len, prefix << 1 | (uint32_t)side, code)) continue;
        }
        t[node].full = tree_full(t, t[node].child[0]) && tree_full(t, t[node].child[1]);
        return true;
    }
    return false;
}

static bool assign_lengths(const uint8_t *len, uint32_t *code, int entries, int nused)
{
    int total = 1;
    for (int e = 0; e < entries; e++) total += len[e];
    tnode *t = calloc((size_t)total + 2, sizeof(tnode));
    int count = 1;
    bool ok = true;
    for (int e = 0; e < entries && ok; e++)
        if (len[e] && nused > 1) ok = place(t, &count, 0, 0, len[e], 0, &code[e]);
    free(t);
    return ok;
}

static bool assign_codes(wbook *k) { return assign_lengths(k->len, k->code, k->entries, k->nused); }

/* The twin of a type 2 book: a type 1 book whose lattice holds every
   combination of multipliers, with only the combinations the type 2 book
   has in use. Its values are worked out by the same sums, so they are the
   same floats. */
static void make_twin(wbook *k)
{
    int levels = 1 << k->value_bits;
    k->twin_entries = 1;
    for (int i = 0; i < k->dims; i++) k->twin_entries *= levels;
    k->twin_len = calloc((size_t)k->twin_entries, 1);
    k->twin_code = calloc((size_t)k->twin_entries, sizeof(uint32_t));
    k->twin_of = calloc((size_t)k->entries, sizeof(int));
    for (int e = 0; e < k->entries; e++) {
        int t = 0, scale = 1;
        for (int i = 0; i < k->dims; i++) {
            t += (int)k->mult[e * k->dims + i] * scale;
            scale *= levels;
        }
        k->twin_of[e] = t;
        if (k->len[e]) k->twin_len[t] = 1; /* in use, length set below */
    }
    int used = 0;
    for (int t = 0; t < k->twin_entries; t++) used += k->twin_len[t];
    k->twin_nused = used;
    /* A complete code: the first ones a bit shorter than the rest. */
    int depth = used > 1 ? ilog((uint64_t)used - 1) : 1, shorter = (1 << depth) - used, n = 0;
    for (int t = 0; t < k->twin_entries; t++)
        if (k->twin_len[t]) k->twin_len[t] = (uint8_t)(used == 1 ? 1 : n++ < shorter ? depth - 1 : depth);
    assign_lengths(k->twin_len, k->twin_code, k->twin_entries, used);
}

static uint32_t pack_float(int mantissa, int exponent)
{
    uint32_t m = (uint32_t)(mantissa < 0 ? -mantissa : mantissa);
    return (mantissa < 0 ? 0x80000000u : 0) | (uint32_t)(exponent + 788) << 21 | m;
}

static float unpack(uint32_t x)
{
    double m = x & 0x1FFFFF;
    if (x & 0x80000000) m = -m;
    return (float)ldexp(m, (int)((x >> 21) & 0x3FF) - 788);
}

static int lookup1(int entries, int dims)
{
    int r = 1;
    for (;;) {
        double p = pow(r + 1, dims);
        if (p > entries) return r;
        r++;
    }
}

/* Makes a book of `used` codewords, `dims` wide. dense: no unused entries
   and entry numbers in order (for books read as numbers). lookup: 0, 1 or
   2. The values are minimum + multiplier * step, multiplier below
   2^value_bits, all in units of 2^unit. */
static int make_book(wsetup *s, int dims, int used, bool dense, int lookup, int minimum, int step, int value_bits, bool seq, int unit)
{
    if (s->nbooks == W_BOOKS) return s->nbooks - 1; /* (never reached: the setups stay well under) */
    wbook *k = &s->books[s->nbooks++];
    memset(k, 0, sizeof(*k));
    k->dims = dims;
    k->lookup = lookup;
    k->sparse = !dense && used > 1 && rnd() % 3 == 0;
    k->entries = used + (k->sparse ? rnd_in(1, used + 2) : 0);
    k->ordered = !k->sparse && rnd() % 3 == 0;
    k->len = calloc((size_t)k->entries, 1);
    k->code = calloc((size_t)k->entries, sizeof(uint32_t));
    k->used = calloc((size_t)used, sizeof(int));
    k->nused = used;
    /* Lengths of a complete code: split leaves of a tree at random. */
    uint8_t lens[1024];
    int n = 1;
    lens[0] = 0;
    if (used == 1) {
        lens[0] = 1;
    } else {
        lens[0] = 1;
        lens[1] = 1;
        n = 2;
        while (n < used) {
            int i = rnd_in(0, n - 1);
            if (lens[i] >= 18) continue;
            lens[i]++;
            lens[n++] = lens[i];
        }
        if (k->ordered) {
            for (int i = 1; i < n; i++)
                for (int j = i; j > 0 && lens[j - 1] > lens[j]; j--) {
                    uint8_t t = lens[j];
                    lens[j] = lens[j - 1];
                    lens[j - 1] = t;
                }
        } else if (!dense || rnd() % 2) {
            for (int i = n - 1; i > 0; i--) {
                int j = rnd_in(0, i);
                uint8_t t = lens[i];
                lens[i] = lens[j];
                lens[j] = t;
            }
        }
    }
    /* Spread the used entries among the unused ones. */
    int at = 0, left_used = used;
    for (int e = 0; e < k->entries; e++) {
        int left = k->entries - e;
        if (left_used && (left == left_used || rnd() % 2)) {
            k->len[e] = lens[at];
            k->used[at++] = e;
            left_used--;
        }
    }
    if (!assign_codes(k)) {
        /* Rare: the order of lengths left no room; sort them, which always fits. */
        int u = 0;
        for (int i = 1; i < used; i++)
            for (int j = i; j > 0 && lens[j - 1] > lens[j]; j--) {
                uint8_t t = lens[j];
                lens[j] = lens[j - 1];
                lens[j - 1] = t;
            }
        for (int e = 0; e < k->entries; e++)
            if (k->len[e]) k->len[e] = lens[u++];
        assign_codes(k);
    }
    if (lookup) {
        k->min_packed = pack_float(minimum, unit);
        k->delta_packed = pack_float(step, unit);
        k->value_bits = value_bits;
        k->seq = seq;
        k->nmult = lookup == 1 ? lookup1(k->entries, dims) : k->entries * dims;
        k->mult = calloc((size_t)k->nmult, sizeof(uint32_t));
        for (int i = 0; i < k->nmult; i++) k->mult[i] = (uint32_t)rnd_in(0, (1 << value_bits) - 1);
        k->values = calloc((size_t)k->entries * (size_t)dims, sizeof(float));
        float mn = unpack(k->min_packed), dl = unpack(k->delta_packed);
        for (int e = 0; e < k->entries; e++) {
            float last = 0;
            int divisor = 1;
            for (int i = 0; i < dims; i++) {
                int off = lookup == 1 ? e / divisor % k->nmult : e * dims + i;
                float v = (float)k->mult[off] * dl + mn + last;
                k->values[e * dims + i] = v;
                if (seq) last = v;
                divisor *= k->nmult;
            }
        }
        g_lookup[lookup]++;
        g_seq += seq;
        if (lookup == 2) make_twin(k);
    } else {
        g_lookup[0]++;
    }
    g_sparse += k->sparse;
    g_ordered += k->ordered;
    g_single += used == 1;
    return s->nbooks - 1;
}

static void put_code(bitw *w, const wbook *k, int e)
{
    bool one = k->nused == 1 && rnd() % 4 == 0; /* either bit is the one codeword */
    if (g_twin && k->lookup == 2) {
        int t = k->twin_of[e];
        if (k->twin_nused == 1) put(w, 0, 1);
        else
            for (int i = k->twin_len[t] - 1; i >= 0; i--) put(w, (k->twin_code[t] >> i) & 1, 1);
        return;
    }
    if (k->nused == 1) {
        /* ffmpeg reads only a 0 here; the specification's errata says
           either bit. */
        g_one_bit += one && !g_twin;
        put(w, one && !g_twin, 1);
        return;
    }
    for (int i = k->len[e] - 1; i >= 0; i--) put(w, (k->code[e] >> i) & 1, 1);
}

static int any_entry(const wbook *k) { return k->used[rnd_in(0, k->nused - 1)]; }

static void put_twin_book(bitw *w, const wbook *k)
{
    put(w, 0x564342, 24);
    put(w, (uint64_t)k->dims, 16);
    put(w, (uint64_t)k->twin_entries, 24);
    put(w, 0, 1);
    put(w, 1, 1);
    for (int t = 0; t < k->twin_entries; t++) {
        put(w, k->twin_len[t] != 0, 1);
        if (k->twin_len[t]) put(w, k->twin_len[t] - 1u, 5);
    }
    put(w, 1, 4);
    put(w, k->min_packed, 32);
    put(w, k->delta_packed, 32);
    put(w, (uint64_t)k->value_bits - 1, 4);
    put(w, k->seq, 1);
    for (int i = 0; i < 1 << k->value_bits; i++) put(w, (uint64_t)i, k->value_bits);
}

static void put_book(bitw *w, const wbook *k)
{
    if (g_twin && k->lookup == 2) {
        put_twin_book(w, k);
        return;
    }
    put(w, 0x564342, 24);
    put(w, (uint64_t)k->dims, 16);
    put(w, (uint64_t)k->entries, 24);
    put(w, k->ordered, 1);
    if (!k->ordered) {
        put(w, k->sparse, 1);
        for (int e = 0; e < k->entries; e++) {
            if (k->sparse) put(w, k->len[e] != 0, 1);
            if (k->len[e]) put(w, k->len[e] - 1u, 5);
        }
    } else {
        int at = 0, length = k->len[0];
        put(w, (uint64_t)length - 1, 5);
        while (at < k->entries) {
            int count = 0;
            while (at + count < k->entries && k->len[at + count] == length) count++;
            put(w, (uint64_t)count, ilog((uint64_t)(k->entries - at)));
            at += count;
            length++;
        }
    }
    int lookup = g_twin && k->scalar ? 0 : k->lookup;
    put(w, (uint64_t)lookup, 4);
    if (lookup) {
        put(w, k->min_packed, 32);
        put(w, k->delta_packed, 32);
        put(w, (uint64_t)k->value_bits - 1, 4);
        put(w, k->seq, 1);
        for (int i = 0; i < k->nmult; i++) put(w, k->mult[i], k->value_bits);
    }
}

/* ---- a random setup ------------------------------------------------------------- */

static const int ranges[4] = {256, 128, 86, 64};

static void make_floor1(wsetup *s, wfloor *f, int half)
{
    f->type = 1;
    f->mult = rnd_in(1, 4);
    int range = ranges[f->mult - 1];
    f->rangebits = ilog((uint64_t)half) - 1;
    int room = (1 << f->rangebits) - 1;
    int classes = rnd_in(1, 5);
    f->maxclass = -1;
    f->partitions = rnd_in(0, 10);
    int total = 0;
    for (int c = 0; c < classes; c++) {
        f->cdim[c] = rnd() % 8 ? rnd_in(1, 4) : rnd_in(5, 8);
        f->csub[c] = rnd_in(0, 3);
        if (f->csub[c]) {
            f->cmaster[c] = make_book(s, rnd_in(1, 3), rnd_in(1, 20), false, rnd_in(0, 1), -3, 1, 3, false, 0);
            s->books[f->cmaster[c]].scalar = true;
        }
        for (int j = 0; j < 1 << f->csub[c]; j++) {
            if (rnd() % 6 == 0) {
                f->sub[c][j] = -1;
                g_no_book++;
            } else {
                int e = rnd_in(1, range < 40 ? range : 40);
                f->sub[c][j] = make_book(s, rnd_in(1, 2), e, true, rnd_in(0, 1), 0, 1, 2, false, 0);
                s->books[f->sub[c][j]].scalar = true;
            }
        }
    }
    for (int i = 0; i < f->partitions; i++) {
        int c = rnd_in(0, classes - 1);
        if (total + f->cdim[c] > 63 || total + f->cdim[c] > room) {
            f->partitions = i;
            break;
        }
        f->pclass[i] = c;
        total += f->cdim[c];
        if (c > f->maxclass) f->maxclass = c;
    }
    f->values = 2 + total;
    f->x[0] = 0;
    f->x[1] = 1 << f->rangebits;
    for (int i = 2; i < f->values; i++) {
        bool fresh;
        do {
            f->x[i] = rnd_in(1, room);
            fresh = true;
            for (int j = 0; j < i; j++) fresh &= f->x[j] != f->x[i];
        } while (!fresh);
    }
    g_floor1++;
}

/* A type 0 floor and books whose values climb in small steps, so that the
   LSP angles come out in order between 0 and pi: a well-behaved filter. */
static void make_floor0(wsetup *s, wfloor *f)
{
    f->type = 0;
    f->order = rnd_in(1, 24);
    f->rate = rnd() % 2 ? 44100 : rnd_in(4000, 50000);
    f->bark = rnd_in(32, 600);
    f->amp_bits = rnd_in(3, 10);
    f->amp_offset = rnd_in(10, 90);
    f->nbooks = rnd_in(1, 3);
    for (int i = 0; i < f->nbooks; i++) {
        int dims = rnd_in(1, 4), bits = rnd_in(1, 3); /* at most 8^4 twin entries */
        /* Steps of (1 + m * 1) / 4096 * scale; the largest, times the order
           and the dims, stays under pi. */
        double room = 3.0 / (f->order + dims);
        int unit = -12, smallest = 1, step = 1;
        int largest = smallest + ((1 << bits) - 1) * step;
        while (largest * ldexp(1, unit) > room) unit--;
        while (largest * ldexp(1, unit + 1) <= room && unit < -2) unit++;
        f->booklist[i] = make_book(s, dims, rnd_in(1, 30), false, rnd_in(1, 2), smallest, step, bits, true, unit);
    }
    g_floor0++;
}

static void make_residue(wsetup *s, wresidue *r, int type, int size, const int *vq, int nvq)
{
    static const int psizes[] = {6, 8, 12, 16, 24, 32, 48, 64};
    r->type = type;
    g_residue[r->type]++;
    r->psize = psizes[rnd_in(0, 7)];
    r->classes = rnd_in(1, 5);
    int dims = rnd_in(1, 3);
    int combos = 1;
    for (int i = 0; i < dims; i++) combos *= r->classes;
    while (combos > 200) {
        dims--;
        combos /= r->classes;
    }
    if (dims > 1 && r->classes > 1) g_classes_multi++;
    r->classbook = make_book(s, dims, combos + rnd_in(0, 2), true, rnd_in(0, 1), 0, 1, 1, false, 0);
    s->books[r->classbook].scalar = true;
    r->begin = rnd() % 3 ? 0 : rnd_in(0, size / 4);
    r->end = rnd() % 3 ? size : rnd_in(r->begin, size);
    if (rnd() % 6 == 0) {
        /* Past the end of the vector, or the wrong way round. */
        if (rnd() % 3) r->end = rnd_in(size + 1, size + size / 2 + 2);
        else r->begin = rnd_in(r->end + 1, r->end + 40);
        g_res_over++;
    }
    r->twin_begin = r->begin < size ? r->begin : size;
    r->twin_end = r->end < size ? r->end : size;
    if (r->twin_begin > r->twin_end) r->twin_begin = r->twin_end = 0;
    for (int c = 0; c < r->classes; c++) {
        r->cascade[c] = 0;
        for (int pass = 0; pass < 8; pass++)
            if (rnd() % (pass < 3 ? 2 : 6) == 0) r->cascade[c] |= 1 << pass;
        if (r->cascade[c] & 0xF8) g_cascade_late++;
        for (int pass = 0; pass < 8; pass++) {
            r->books[c][pass] = -1;
            if (!(r->cascade[c] & (1 << pass))) continue;
            int k;
            do k = vq[rnd_in(0, nvq - 1)];
            while (r->psize % s->books[k].dims);
            r->books[c][pass] = k;
        }
    }
}

static void random_setup(wsetup *s)
{
    static const int rates[] = {8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000};
    static const int chans[] = {1, 1, 2, 2, 2, 2, 3, 4, 5, 6, 7, 8};
    memset(s, 0, sizeof(*s));
    s->channels = chans[rnd_in(0, 11)];
    s->rate = rates[rnd_in(0, 7)];
    s->e0 = rnd_in(6, 9);
    s->e1 = rnd() % 4 ? rnd_in(s->e0, 11) : s->e0;
    /* Value books for residues: dims that divide the partition sizes. A type
       2 table keeps its multipliers few enough for a twin of up to 65536
       entries. */
    static const int vq_dims[] = {1, 2, 3, 4, 6, 8};
    int vq[12], nvq = rnd_in(4, 8);
    for (int i = 0; i < nvq; i++) {
        int dims = i ? vq_dims[rnd_in(0, 5)] : 1, lookup = rnd_in(1, 2), bits = rnd_in(1, 4); /* one fits any partition */
        if (lookup == 2 && bits > 16 / dims) bits = 16 / dims;
        vq[i] = make_book(s, dims, rnd() % 10 ? rnd_in(2, 60) : 1, false, lookup, -((1 << bits) - 1) / 2 - rnd_in(0, 1), 1, bits,
                          rnd() % 5 == 0, -rnd_in(0, 2));
    }
    /* Floors, mappings and residues for each block size, as encoders lay
       them out: a type 1 floor spans the block it is for, and a residue
       codes the vectors of one submap. */
    int floors_of[2][3], nfloors_of[2];
    for (int b = 0; b < 2; b++) {
        nfloors_of[b] = rnd_in(1, 2);
        for (int i = 0; i < nfloors_of[b]; i++) {
            wfloor *f = &s->floors[s->nfloors];
            if (rnd() % 3 == 0) make_floor0(s, f);
            else make_floor1(s, f, (1 << (b ? s->e1 : s->e0)) / 2);
            floors_of[b][i] = s->nfloors++;
        }
    }
    for (int b = 0; b < 2; b++) {
        int count = rnd_in(1, 2);
        for (int i = 0; i < count; i++) {
            wmapping *m = &s->maps[s->nmaps];
            s->map_block[s->nmaps++] = b;
            m->submaps = rnd() % 3 ? 1 : rnd_in(2, 4);
            if (m->submaps > 1) g_submaps++;
            int in[16] = {0};
            for (int c = 0; c < s->channels; c++) {
                m->mux[c] = rnd_in(0, m->submaps - 1);
                in[m->mux[c]]++;
            }
            for (int j = 0; j < m->submaps; j++) {
                m->sfloor[j] = floors_of[b][rnd_in(0, nfloors_of[b] - 1)];
                int type = rnd_in(0, 2), half = (1 << (b ? s->e1 : s->e0)) / 2;
                make_residue(s, &s->res[s->nres], type, half * (type == 2 ? in[j] : 1), vq, nvq);
                m->sres[j] = s->nres++;
                if (!in[j]) g_empty_submap++;
            }
            m->steps = s->channels > 1 && rnd() % 3 ? rnd_in(1, s->channels + 1) : 0;
            for (int j = 0; j < m->steps; j++) {
                m->mag[j] = rnd_in(0, s->channels - 1);
                do m->ang[j] = rnd_in(0, s->channels - 1);
                while (m->ang[j] == m->mag[j]);
            }
            if (m->steps) g_coupled++;
        }
    }
    s->nmodes = rnd_in(1, W_MODES);
    for (int i = 0; i < s->nmodes; i++) {
        s->blockflag[i] = rnd_in(0, 1);
        int pick;
        do pick = rnd_in(0, s->nmaps - 1);
        while (s->map_block[pick] != s->blockflag[i]);
        s->mapping[i] = pick;
    }
}

/* ---- headers -------------------------------------------------------------------- */

static void put_identification(bitw *w, const wsetup *s)
{
    put_text(w, "\x01vorbis", 7);
    put(w, 0, 32);
    put(w, (uint64_t)s->channels, 8);
    put(w, (uint64_t)s->rate, 32);
    put(w, 0, 32);
    put(w, 128000, 32);
    put(w, 0, 32);
    put(w, (uint64_t)s->e0, 4);
    put(w, (uint64_t)s->e1, 4);
    put(w, 1, 1);
    finish(w);
}

static void put_comment(bitw *w)
{
    put_text(w, "\x03vorbis", 7);
    put(w, 4, 32);
    put_text(w, "test", 4);
    put(w, 1, 32);
    put(w, 9, 32);
    put_text(w, "TITLE=one", 9);
    put(w, 1, 1);
    finish(w);
}

static void put_setup(bitw *w, const wsetup *s)
{
    put_text(w, "\x05vorbis", 7);
    put(w, (uint64_t)s->nbooks - 1, 8);
    for (int i = 0; i < s->nbooks; i++) put_book(w, &s->books[i]);
    put(w, 0, 6);
    put(w, 0, 16);
    put(w, (uint64_t)s->nfloors - 1, 6);
    for (int i = 0; i < s->nfloors; i++) {
        const wfloor *f = &s->floors[i];
        put(w, (uint64_t)f->type, 16);
        if (f->type == 0) {
            put(w, (uint64_t)f->order, 8);
            put(w, (uint64_t)f->rate, 16);
            put(w, (uint64_t)f->bark, 16);
            put(w, (uint64_t)f->amp_bits, 6);
            put(w, (uint64_t)f->amp_offset, 8);
            put(w, (uint64_t)f->nbooks - 1, 4);
            for (int j = 0; j < f->nbooks; j++) put(w, (uint64_t)f->booklist[j], 8);
            continue;
        }
        put(w, (uint64_t)f->partitions, 5);
        for (int j = 0; j < f->partitions; j++) put(w, (uint64_t)f->pclass[j], 4);
        for (int c = 0; c <= f->maxclass; c++) {
            put(w, (uint64_t)f->cdim[c] - 1, 3);
            put(w, (uint64_t)f->csub[c], 2);
            if (f->csub[c]) put(w, (uint64_t)f->cmaster[c], 8);
            for (int j = 0; j < 1 << f->csub[c]; j++) put(w, (uint64_t)(f->sub[c][j] + 1), 8);
        }
        put(w, (uint64_t)f->mult - 1, 2);
        put(w, (uint64_t)f->rangebits, 4);
        for (int j = 2; j < f->values; j++) put(w, (uint64_t)f->x[j], f->rangebits);
    }
    put(w, (uint64_t)s->nres - 1, 6);
    for (int i = 0; i < s->nres; i++) {
        const wresidue *r = &s->res[i];
        put(w, (uint64_t)r->type, 16);
        put(w, (uint64_t)(g_twin ? r->twin_begin : r->begin), 24);
        put(w, (uint64_t)(g_twin ? r->twin_end : r->end), 24);
        put(w, (uint64_t)r->psize - 1, 24);
        put(w, (uint64_t)r->classes - 1, 6);
        put(w, (uint64_t)r->classbook, 8);
        for (int c = 0; c < r->classes; c++) {
            put(w, (uint64_t)(r->cascade[c] & 7), 3);
            put(w, r->cascade[c] > 7, 1);
            if (r->cascade[c] > 7) put(w, (uint64_t)(r->cascade[c] >> 3), 5);
        }
        for (int c = 0; c < r->classes; c++)
            for (int pass = 0; pass < 8; pass++)
                if (r->books[c][pass] >= 0) put(w, (uint64_t)r->books[c][pass], 8);
    }
    put(w, (uint64_t)s->nmaps - 1, 6);
    for (int i = 0; i < s->nmaps; i++) {
        const wmapping *m = &s->maps[i];
        put(w, 0, 16);
        put(w, m->submaps > 1, 1);
        if (m->submaps > 1) put(w, (uint64_t)m->submaps - 1, 4);
        put(w, m->steps > 0, 1);
        if (m->steps) {
            put(w, (uint64_t)m->steps - 1, 8);
            for (int j = 0; j < m->steps; j++) {
                put(w, (uint64_t)m->mag[j], ilog((uint64_t)s->channels - 1));
                put(w, (uint64_t)m->ang[j], ilog((uint64_t)s->channels - 1));
            }
        }
        put(w, 0, 2);
        if (m->submaps > 1)
            for (int c = 0; c < s->channels; c++) put(w, (uint64_t)m->mux[c], 4);
        for (int j = 0; j < m->submaps; j++) {
            put(w, 0, 8);
            put(w, (uint64_t)m->sfloor[j], 8);
            put(w, (uint64_t)m->sres[j], 8);
        }
    }
    put(w, (uint64_t)s->nmodes - 1, 6);
    for (int i = 0; i < s->nmodes; i++) {
        put(w, (uint64_t)s->blockflag[i], 1);
        put(w, 0, 16);
        put(w, 0, 16);
        put(w, (uint64_t)s->mapping[i], 8);
    }
    put(w, 1, 1);
    finish(w);
}

/* ---- sound packets -------------------------------------------------------------- */

static bool put_floor(bitw *w, const wsetup *s, const wfloor *f, bool must_use)
{
    if (f->type == 0) {
        int amp = rnd() % 8 || must_use ? rnd_in(1, (1 << f->amp_bits) - 1) : 0;
        put(w, (uint64_t)amp, f->amp_bits);
        if (!amp) return false;
        int which = rnd_in(0, f->nbooks - 1);
        if (g_bad_floor0) {
            which = rnd_in(f->nbooks, (1 << ilog((uint64_t)f->nbooks)) - 1);
            g_bad_written++;
        }
        put(w, (uint64_t)which, ilog((uint64_t)f->nbooks));
        const wbook *k = &s->books[f->booklist[which]];
        for (int count = 0; count < f->order; count += k->dims) put_code(w, k, any_entry(k));
        return true;
    }
    bool used = rnd() % 8 != 0 || must_use;
    put(w, used, 1);
    if (!used) return false;
    int range = ranges[f->mult - 1], ybits = ilog((uint64_t)range - 1);
    put(w, (uint64_t)rnd_in(0, range - 1), ybits);
    put(w, (uint64_t)rnd_in(0, range - 1), ybits);
    for (int p = 0; p < f->partitions; p++) {
        int c = f->pclass[p], cbits = f->csub[c], cval = 0;
        if (cbits) {
            const wbook *m = &s->books[f->cmaster[c]];
            cval = any_entry(m);
            put_code(w, m, cval);
        }
        for (int j = 0; j < f->cdim[c]; j++) {
            int k = f->sub[c][cval & ((1 << cbits) - 1)];
            cval >>= cbits;
            if (k >= 0) put_code(w, &s->books[k], any_entry(&s->books[k]));
        }
    }
    return true;
}

static void put_residue(bitw *w, const wsetup *s, const wresidue *r, const bool *skip_in, int count, int half)
{
    bool skip[8];
    int vectors = count;
    size_t size = (size_t)half;
    if (r->type == 2) {
        bool any = false;
        for (int i = 0; i < count; i++) any |= !skip_in[i];
        if (!any || !count) return;
        vectors = 1;
        skip[0] = false;
        size *= (size_t)count;
    } else {
        memcpy(skip, skip_in, sizeof(bool) * (size_t)count);
    }
    size_t begin = (size_t)r->begin < size ? (size_t)r->begin : size, end = (size_t)r->end < size ? (size_t)r->end : size;
    if (end <= begin) return;
    size_t parts = (end - begin) / (size_t)r->psize;
    if (!parts) return;
    const wbook *cb = &s->books[r->classbook];
    int per = cb->dims;
    uint8_t *cls = malloc((size_t)vectors * (parts + (size_t)per));
    for (size_t i = 0; i < (size_t)vectors * (parts + (size_t)per); i++) cls[i] = (uint8_t)rnd_in(0, r->classes - 1);
    size_t stride = parts + (size_t)per;
    for (int pass = 0; pass < 8; pass++) {
        size_t p = 0;
        while (p < parts) {
            if (pass == 0)
                for (int j = 0; j < vectors; j++) {
                    if (skip[j]) continue;
                    int entry = 0;
                    for (int i = 0; i < per; i++) entry = entry * r->classes + cls[(size_t)j * stride + p + (size_t)i];
                    put_code(w, cb, entry);
                }
            for (int i = 0; i < per && p < parts; i++, p++)
                for (int j = 0; j < vectors; j++) {
                    if (skip[j]) continue;
                    int k = r->books[cls[(size_t)j * stride + p]][pass];
                    if (k < 0) continue;
                    const wbook *b = &s->books[k];
                    for (int v = 0; v < r->psize / b->dims; v++) put_code(w, b, any_entry(b));
                }
        }
    }
    free(cls);
}

static void put_audio(bitw *w, const wsetup *s, int mode, bool prev_long, bool next_long)
{
    put(w, 0, 1);
    put(w, (uint64_t)mode, ilog((uint64_t)s->nmodes - 1));
    int blk = s->blockflag[mode];
    if (blk) {
        put(w, prev_long, 1);
        put(w, next_long, 1);
    }
    const wmapping *m = &s->maps[s->mapping[mode]];
    int half = (1 << (blk ? s->e1 : s->e0)) / 2;
    /* When coupling steps share a channel, ffmpeg passes on which channels
       need their residue in the opposite order to the specification's;
       with every floor in use the order cannot matter. */
    bool no_residue[8], any_unused = false, chained = false;
    for (int i = 0; i < m->steps; i++)
        for (int j = 0; j < i; j++)
            chained |= m->mag[i] == m->mag[j] || m->mag[i] == m->ang[j] || m->ang[i] == m->mag[j] || m->ang[i] == m->ang[j];
    g_chained += chained;
    for (int c = 0; c < s->channels; c++) {
        no_residue[c] = !put_floor(w, s, &s->floors[m->sfloor[m->mux[c]]], chained);
        any_unused |= no_residue[c];
    }
    g_unused += any_unused;
    for (int i = 0; i < m->steps; i++) {
        if (no_residue[m->mag[i]] != no_residue[m->ang[i]]) g_unused_pair++;
        if (!no_residue[m->mag[i]] || !no_residue[m->ang[i]]) no_residue[m->mag[i]] = no_residue[m->ang[i]] = false;
    }
    for (int j = 0; j < m->submaps; j++) {
        bool skip[8];
        int count = 0;
        for (int c = 0; c < s->channels; c++)
            if (m->mux[c] == j) skip[count++] = no_residue[c];
        put_residue(w, s, &s->res[m->sres[j]], skip, count, half);
    }
    finish(w);
}

/* ---- Ogg pages ------------------------------------------------------------------ */

static uint32_t ogg_crc(const uint8_t *p, size_t n)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint32_t)p[i] << 24;
        for (int k = 0; k < 8; k++) crc = crc & 0x80000000u ? crc << 1 ^ 0x04C11DB7u : crc << 1;
    }
    return crc;
}

typedef struct {
    bytes data;
    int64_t end;    /* granule position at the end of the packet */
    bool page_end;  /* a page ends after it */
} opacket;

static void le_put(bytes *b, uint64_t v, int n)
{
    for (int i = 0; i < n; i++) {
        unsigned char c = (unsigned char)(v >> (8 * i));
        bytes_put(b, &c, 1);
    }
}

static void page_out(bytes *out, uint32_t serial, uint32_t *seq, int flags, int64_t granule, const uint8_t *lacing, int segs,
                     const bytes *body)
{
    size_t at = out->len;
    bytes_put(out, "OggS", 4);
    unsigned char z = 0, f = (unsigned char)flags, n = (unsigned char)segs;
    bytes_put(out, &z, 1);
    bytes_put(out, &f, 1);
    le_put(out, (uint64_t)granule, 8);
    le_put(out, serial, 4);
    le_put(out, (*seq)++, 4);
    le_put(out, 0, 4);
    bytes_put(out, &n, 1);
    bytes_put(out, lacing, (size_t)segs);
    bytes_put(out, body->p, body->len);
    uint32_t crc = ogg_crc(out->p + at, out->len - at);
    for (int i = 0; i < 4; i++) out->p[at + 22 + (size_t)i] = (uint8_t)(crc >> (8 * i));
}

/* Cuts the packets into pages: at most max_segs segments a page (a random
   number when 0), and a page end wherever a packet asks for one. */
static void ogg_write(bytes *out, uint32_t serial, const opacket *pk, int count, int max_segs)
{
    uint8_t lacing[255];
    bytes body = {0};
    int segs = 0, limit = max_segs ? max_segs : rnd_in(1, 255);
    uint32_t seq = 0;
    int64_t granule = -1;
    bool first = true, continued = false;
    for (int i = 0; i < count; i++) {
        size_t len = pk[i].data.len, pos = 0;
        for (;;) {
            size_t piece = len - pos > 255 ? 255 : len - pos;
            lacing[segs++] = (uint8_t)piece;
            bytes_put(&body, pk[i].data.p + pos, piece);
            pos += piece;
            bool ends = piece < 255;
            if (ends) granule = pk[i].end;
            bool last = ends && i == count - 1;
            if (segs == limit || (ends && pk[i].page_end) || last) {
                page_out(out, serial, &seq, (continued ? 1 : 0) | (first ? 2 : 0) | (last ? 4 : 0), granule, lacing, segs, &body);
                continued = !ends;
                first = false;
                segs = 0;
                body.len = 0;
                granule = -1;
                limit = max_segs ? max_segs : (rnd() % 4 ? rnd_in(1, 255) : rnd_in(1, 6));
            }
            if (ends) break;
        }
    }
    free(body.p);
}

/* ---- streams against ffmpeg ----------------------------------------------------- */

static size_t read_through(const bytes *file, pcm *out, ahreader_info *info, const char **error)
{
    out->len = 0;
    ahreader *r = ahreader_open(test_memory_read, (void *)file, file->len, error);
    if (!r) return 0;
    *info = *ahreader_get_info(r);
    out->channels = info->channels;
    out->rate = info->sample_rate;
    float buf[8 * 1000];
    size_t n;
    while ((n = ahreader_read(r, buf, 1000)) != 0) pcm_put(out, buf, n * (size_t)info->channels);
    ahreader_close(r);
    return out->len / (size_t)(info->channels ? info->channels : 1);
}

/* Builds a random stream: setup, packets, pages. start: the granule
   position the first sound starts at (negative: that much is cut from the
   start); cut: samples cut from the end. */
static void build_stream(wsetup *s, bytes *file, int packets, int64_t start, int cut, int max_segs, bool twin)
{
    g_twin = twin;
    opacket *pk = calloc((size_t)packets + 3, sizeof(opacket));
    bitw w = {0};
    put_identification(&w, s);
    pk[0].data = w.b;
    pk[0].page_end = true;
    memset(&w, 0, sizeof(w));
    put_comment(&w);
    pk[1].data = w.b;
    memset(&w, 0, sizeof(w));
    put_setup(&w, s);
    pk[2].data = w.b;
    pk[2].page_end = true;
    int *modes = malloc((size_t)packets * sizeof(int));
    for (int i = 0; i < packets; i++) modes[i] = rnd_in(0, s->nmodes - 1);
    int64_t pos = start;
    int prev_n = 0;
    for (int i = 0; i < packets; i++) {
        int blk = s->blockflag[modes[i]], n = 1 << (blk ? s->e1 : s->e0);
        bool prev_long = i ? s->blockflag[modes[i - 1]] : rnd() % 2, next_long = i + 1 < packets ? s->blockflag[modes[i + 1]] : rnd() % 2;
        if (i && s->blockflag[modes[i - 1]] != blk) g_switches++;
        memset(&w, 0, sizeof(w));
        int bad = g_bad_written;
        put_audio(&w, s, modes[i], prev_long, next_long);
        if (i < 64) g_packet_bad[i] = g_bad_written != bad;
        pk[3 + i].data = w.b;
        pk[3 + i].page_end = i + 1 == g_first_page;
        if (i) pos += prev_n / 4 + n / 4;
        if (i < 64) g_packet_samples[i] = i ? prev_n / 4 + n / 4 : 0;
        prev_n = n;
        pk[3 + i].end = pos;
    }
    pk[2 + packets].end -= cut;
    ogg_write(file, (uint32_t)rnd(), pk, packets + 3, max_segs);
    for (int i = 0; i < packets + 3; i++) free(pk[i].data.p);
    free(pk);
    free(modes);
    g_twin = false;
}

static void debug_stream(const bytes *file)
{
    const char *error = NULL;
    ahogg *o = ahogg_open(test_memory_read, (void *)file, file->len, &error);
    if (!o) {
        printf("    ogg: %s\n", error);
        return;
    }
    ahvorbis_decoder *d = ahvorbis_decoder_new();
    for (int i = 0; i < 3; i++) {
        const uint8_t *p;
        size_t n;
        bool ok = ahogg_header(o, i, &p, &n) && ahvorbis_decoder_header(d, p, n, NULL);
        printf("    header %d: %s (%zu bytes) %s\n", i, ok ? "ok" : "refused", n, ahvorbis_decoder_error(d) ? ahvorbis_decoder_error(d) : "");
    }
    const uint8_t *p;
    size_t n;
    int64_t g;
    static float pcm[8 * 4096];
    for (int k = 0; ahogg_packet(o, &p, &n, &g); k++) {
        int got = 0;
        bool ok = ahvorbis_decode_packet(d, p, n, pcm, &got);
        printf("    packet %d: %zu bytes, granule %lld, %s, %d samples\n", k, n, (long long)g, ok ? "ok" : "refused", got);
    }
    ahvorbis_decoder_free(d);
    ahogg_close(o);
}

static bool same_pcm(const pcm *a, const pcm *b)
{
    return a->len == b->len && (!a->len || !memcmp(a->p, b->p, a->len * sizeof(float)));
}

/* Random streams, each with its twin: the same sound in only what ffmpeg
   reads. This decoder must give the two exactly the same sound, and ffmpeg
   must give the twin's. */
static void test_random_streams(void)
{
    int compared = 0, worst_at = -1, twins = 0;
    double worst = 0;
    const char *one = getenv("AHV_SEED");
    for (int round = 0; round < (one ? 1 : getenv("AHV_ROUNDS") ? atoi(getenv("AHV_ROUNDS")) : 60); round++) {
        if (one) g_seed = (unsigned)strtoul(one, NULL, 10);
        unsigned seed = g_seed;
        if (getenv("AHV_VERBOSE")) printf("  round %d seed %u\n", round, seed);
        wsetup s;
        random_setup(&s);
        if (getenv("AHV_VERBOSE"))
            for (int i = 0; i < s.nfloors; i++) printf("    floor %d: type %d order %d amp %d/%d bark %d\n", i, s.floors[i].type, s.floors[i].order, s.floors[i].amp_bits, s.floors[i].amp_offset, s.floors[i].bark);
        int packets = rnd_in(4, 40);
        unsigned packet_seed = rnd();
        int one_bits = g_one_bit;
        bytes file = {0}, twin = {0};
        g_seed = packet_seed;
        build_stream(&s, &file, packets, 0, 0, 0, false);
        g_seed = packet_seed;
        build_stream(&s, &twin, packets, 0, 0, 0, true);
        bool needs_twin = g_one_bit > one_bits;
        for (int i = 0; i < s.nbooks; i++) {
            needs_twin |= s.books[i].lookup == 2 || (s.books[i].scalar && s.books[i].lookup && s.books[i].sparse);
            g_scalar_table += s.books[i].scalar && s.books[i].lookup && s.books[i].sparse;
        }
        for (int i = 0; i < s.nres; i++) needs_twin |= s.res[i].begin != s.res[i].twin_begin || s.res[i].end != s.res[i].twin_end;
        twins += needs_twin;

        char name[64], path[600];
        snprintf(name, sizeof(name), "vorbis_random_%d.ogg", round);
        path_of(path, name);
        write_file(path, twin.p, twin.len);
        pcm mine = {0}, mine_twin = {0};
        ahreader_info info;
        const char *error = NULL;
        size_t got = read_through(&file, &mine, &info, &error);
        CHECK(got > 0, "%s (seed %u): %s", name, seed, error ? error : "no sound");
        read_through(&twin, &mine_twin, &info, &error);
        CHECK(same_pcm(&mine, &mine_twin), "%s (seed %u): the stream and its twin decode differently", name, seed);
        if (one) debug_stream(&file);
        bytes raw = {0};
        if (got && ffmpeg_raw(path, NULL, "f32le", &raw)) {
            pcm ref = {(float *)raw.p, raw.len / sizeof(float), 0, s.channels, s.rate};
            double d = max_diff(&ref, &mine_twin, 0, ref.len);
            CHECK(ref.len == mine_twin.len, "%s (seed %u): ffmpeg gave %zu samples, this decoder %zu", name, seed, ref.len / (size_t)s.channels,
                  mine_twin.len / (size_t)s.channels);
            /* ffmpeg works type 0 floors out in single precision, this
               decoder in double: they may part by a few parts in 10,000. */
            bool lsp = false;
            for (int i = 0; i < s.nfloors; i++) lsp |= s.floors[i].type == 0;
            CHECK(d < (lsp ? 1e-3 : 1e-4), "%s (seed %u): differs from ffmpeg by %.3g", name, seed, d);
            if (d > worst) {
                worst = d;
                worst_at = round;
            }
            compared++;
        } else {
            CHECK(false, "%s (seed %u): ffmpeg could not decode it", name, seed);
        }
        free(raw.p);
        free(mine.p);
        free(mine_twin.p);
        free(file.p);
        free(twin.p);
        free_setup(&s);
        g_seed = seed * 2654435761u + 1013904223u;
    }
    printf("  %d random streams compared, %d through a twin, worst %.2g (stream %d)\n", compared, twins, worst, worst_at);
    CHECK(g_floor0 && g_floor1 && g_residue[0] && g_residue[1] && g_residue[2], "a floor or residue type was never used");
    CHECK(g_lookup[0] && g_lookup[1] && g_lookup[2] && g_sparse && g_ordered && g_single && g_seq, "a kind of codebook was never used");
    CHECK(g_submaps && g_coupled && g_unused && g_unused_pair && g_switches && g_classes_multi && g_no_book && g_cascade_late && g_empty_submap,
          "a part of the format was never used (%d %d %d %d %d %d %d %d %d)", g_submaps, g_coupled, g_unused, g_unused_pair, g_switches,
          g_classes_multi, g_no_book, g_cascade_late, g_empty_submap);
    CHECK(g_chained, "coupling steps never shared a channel");
    CHECK(g_one_bit && g_res_over && g_scalar_table, "a part only the twins cover was never used (%d %d %d)", g_one_bit, g_res_over,
          g_scalar_table);
}

/* Splits an Ogg file into its pages. */
static int pages_of(const bytes *file, size_t *at, size_t *len, int most)
{
    int n = 0;
    for (size_t i = 0; i + 27 <= file->len && n < most;) {
        size_t size = 27 + file->p[i + 26];
        for (int k = 0; k < file->p[i + 26]; k++) size += file->p[i + 27 + (size_t)k];
        at[n] = i;
        len[n++] = size;
        i += size;
    }
    return n;
}

/* Where the sound starts and ends, as the pages state it: the last page's
   granule position cuts the last packet short; a first page that states
   less than its packets give cuts the start (the specification's way to
   say an encoder's run-in); one that states more says the stream starts
   later, and changes nothing that is played. */
static void test_trims(void)
{
    static const struct {
        const char *what;
        int first_page, start; /* start: -1 cut, 0 none, 1 late */
        bool cut;
    } cases[] = {
        {"end cut", 1, 0, true},
        {"start cut", 4, -1, false},
        {"late start", 1, 1, false},
        {"start and end cut", 4, -1, true},
    };
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++)
        for (int round = 0; round < (getenv("AHV_TRIM_SEED") ? 1 : 6); round++) {
            if (getenv("AHV_TRIM_SEED")) g_seed = (unsigned)strtoul(getenv("AHV_TRIM_SEED"), NULL, 10);
            unsigned seed = g_seed;
            wsetup s;
            random_setup(&s);
            int packets = rnd_in(8, 30);
            /* Every packet but the first gives at least 32 samples. */
            int64_t start = cases[c].start < 0 ? -rnd_in(1, 90) : cases[c].start > 0 ? rnd_in(1, 100000) : 0;
            int cut = cases[c].cut ? rnd_in(1, 31) : 0;
            unsigned packet_seed = rnd();
            bytes file = {0}, twin = {0};
            g_first_page = cases[c].first_page;
            g_seed = packet_seed;
            build_stream(&s, &file, packets, start, cut, 0, false);
            g_seed = packet_seed;
            build_stream(&s, &twin, packets, start, cut, 0, true);
            g_first_page = 1;
            int64_t total = 0;
            for (int i = 0; i < packets; i++) total += g_packet_samples[i];
            int64_t want = total - cut + (start < 0 ? start : 0);
            if (cases[c].start < 0 && round < 2) {
                /* Cut off after its first page of sound, which has no end
                   mark: that page's position is still where the sound
                   starts, and the end is where it says. */
                size_t at[512], len[512];
                int n = pages_of(&file, at, len, 512), page = -1;
                int64_t first = start;
                for (int i = 1; i < 4; i++) first += g_packet_samples[i];
                for (int k = 0; k < n && page < 0; k++) {
                    int64_t g = 0;
                    for (int q = 7; q >= 0; q--) g = g << 8 | file.p[at[k] + 6 + (size_t)q];
                    if (g == first) page = k;
                }
                bytes part = {0};
                if (page >= 0) bytes_put(&part, file.p, at[page] + len[page]);
                pcm few = {0};
                ahreader_info fi;
                const char *e2 = NULL;
                size_t got_few = read_through(&part, &few, &fi, &e2);
                CHECK(page >= 0 && got_few == (size_t)first, "%s (seed %u), cut off after its first page: %zu samples, not %lld", cases[c].what, seed,
                      got_few, (long long)first);
                free(few.p);
                free(part.p);
            }
            char path[600];
            path_of(path, "vorbis_trim.ogg");
            write_file(path, twin.p, twin.len);
            pcm mine = {0}, mine_twin = {0}, ref = {0};
            ahreader_info info;
            const char *error = NULL;
            size_t got = read_through(&file, &mine, &info, &error);
            read_through(&twin, &mine_twin, &info, &error);
            CHECK(got == (size_t)want && info.samples == (uint64_t)want, "%s (seed %u): %zu samples, said %llu, not %lld", cases[c].what, seed, got,
                  (unsigned long long)info.samples, (long long)want);
            CHECK(same_pcm(&mine, &mine_twin), "%s (seed %u): the stream and its twin decode differently", cases[c].what, seed);
            bytes raw = {0};
            if (ffmpeg_raw(path, NULL, "f32le", &raw)) {
                /* ffmpeg does not cut the start: it plays the samples the
                   first page marks to be dropped (see the README). */
                size_t lead = start < 0 ? (size_t)-start * (size_t)s.channels : 0;
                ref.p = (float *)raw.p + lead;
                ref.len = raw.len / sizeof(float) > lead ? raw.len / sizeof(float) - lead : 0;
                bool lsp = false;
                for (int i = 0; i < s.nfloors; i++) lsp |= s.floors[i].type == 0;
                /* In some page layouts ffmpeg works the end cut out but does
                   not make it (see the README). */
                bool uncut = cut && ref.len == mine_twin.len + (size_t)cut * (size_t)s.channels;
                CHECK((ref.len == mine_twin.len || uncut) && max_diff(&ref, &mine_twin, 0, mine_twin.len) < (lsp ? 1e-3 : 1e-4),
                      "%s (seed %u): ffmpeg gave %zu samples, this decoder %zu", cases[c].what, seed, ref.len / (size_t)s.channels, got);
                if (ref.len != mine_twin.len && getenv("AHV_VERBOSE")) {
                    char keep[600];
                    path_of(keep, "vorbis_trim_kept.ogg");
                    write_file(keep, twin.p, twin.len);
                }
            }
            free(raw.p);
            free(mine.p);
            free(mine_twin.p);
            free(file.p);
            free(twin.p);
            free_setup(&s);
        }
}

/* A stream wholly on one page: the page states only where the sound ends.
   ffmpeg counts the first packet, which gives no sound, as if it did, and
   cuts that much too much from the end (see the README). */
static void test_one_page(void)
{
    int tried = 0;
    for (int round = 0; round < 200 && tried < 4; round++) {
        wsetup s;
        random_setup(&s);
        if (s.e1 > 8) {
            free_setup(&s);
            continue;
        }
        int packets = rnd_in(3, 6), cut = rnd_in(0, 31);
        bytes file = {0};
        g_first_page = 0;
        build_stream(&s, &file, packets, 0, cut, 255, false);
        g_first_page = 1;
        int pages = 0;
        for (size_t at = 0; at + 4 <= file.len; at++) pages += !memcmp(file.p + at, "OggS", 4);
        if (pages == 3) { /* two of headers, one of sound */
            tried++;
            int64_t total = 0;
            for (int i = 0; i < packets; i++) total += g_packet_samples[i];
            pcm mine = {0};
            ahreader_info info;
            const char *error = NULL;
            size_t got = read_through(&file, &mine, &info, &error);
            CHECK(got == (size_t)(total - cut), "one page: %zu samples, not %lld", got, (long long)(total - cut));
            free(mine.p);
        }
        free(file.p);
        free_setup(&s);
    }
    CHECK(tried == 4, "only %d streams fit on one page", tried);
}

/* Reads a whole file and checks that seeks land on the very sample. */
static void check_seeks(const char *what, const bytes *file)
{
    pcm whole = {0};
    ahreader_info info;
    const char *error = NULL;
    size_t frames = read_through(file, &whole, &info, &error);
    if (!frames) {
        CHECK(false, "%s: %s", what, error ? error : "no sound");
        return;
    }
    ahreader *r = ahreader_open(test_memory_read, (void *)file, file->len, &error);
    float buf[8 * 2000];
    bool ok = true;
    for (int i = 0; i < 30 && r; i++) {
        size_t at = i == 0 ? 0 : i == 1 ? frames - 1 : (size_t)rnd_in(0, (int)frames - 1);
        size_t want = frames - at < 2000 ? frames - at : 2000, got = 0;
        if (!ahreader_seek(r, at)) {
            ok = false;
            break;
        }
        while (got < want) {
            size_t n = ahreader_read(r, buf + got * (size_t)info.channels, want - got);
            if (!n) break;
            got += n;
        }
        if (got != want || memcmp(buf, whole.p + at * (size_t)info.channels, want * (size_t)info.channels * sizeof(float))) {
            ok = false;
            printf("    %s: a seek to %zu of %zu went wrong\n", what, at, frames);
            break;
        }
    }
    CHECK(ok, "%s: a seek did not land on the sample", what);
    ahreader_close(r);
    free(whole.p);
}

/* Random streams cut into pages of every size, some of them a few bytes, so
   that many pages start no packet or end none: a seek must still land on
   the sample. */
static void test_seeks(void)
{
    for (int round = 0; round < 12; round++) {
        wsetup s;
        random_setup(&s);
        bytes file = {0};
        /* A start cut needs a first page whose packets give more than it. */
        bool cut_start = rnd() % 3 == 0;
        g_first_page = cut_start ? 4 : rnd_in(1, 3);
        build_stream(&s, &file, rnd_in(20, 60), cut_start ? -rnd_in(1, 90) : 0, rnd_in(0, 31), round % 2 ? rnd_in(1, 3) : 0, true);
        g_first_page = 1;
        char what[64];
        snprintf(what, sizeof(what), "random stream %d", round);
        check_seeks(what, &file);
        free(file.p);
        free_setup(&s);
    }
}

typedef struct {
    opacket *pk;
    int count;
} packet_list;

/* The packets of an Ogg file's stream, through ahogg, with the place each
   one ends at worked out by decoding it. */
static bool take_packets(const bytes *file, packet_list *out)
{
    const char *error = NULL;
    ahogg *o = ahogg_open(test_memory_read, (void *)file, file->len, &error);
    if (!o) return false;
    const ahogg_info *oi = ahogg_get_info(o);
    out->count = 0;
    out->pk = calloc(100000, sizeof(opacket));
    ahvorbis_decoder *vd = oi->codec == AHOGG_VORBIS ? ahvorbis_decoder_new() : NULL;
    for (int i = 0; i < oi->headers && i < 3; i++) {
        const uint8_t *d;
        size_t n;
        if (ahogg_header(o, i, &d, &n)) {
            bytes_put(&out->pk[out->count].data, d, n);
            if (vd) ahvorbis_decoder_header(vd, d, n, NULL);
            out->pk[out->count++].page_end = i == 0 || i + 1 == oi->headers;
        }
    }
    const uint8_t *d;
    size_t n;
    int64_t g, pos = 0;
    static float pcm[8 * 65536];
    while (out->count < 100000 && ahogg_packet(o, &d, &n, &g)) {
        opacket *k = &out->pk[out->count++];
        bytes_put(&k->data, d, n);
        int got = 0;
        if (vd) {
            ahvorbis_decode_packet(vd, d, n, pcm, &got);
        } else {
            /* A FLAC frame says how many samples it holds: 4 bits in its
               header, and maybe a number after the frame's own number. */
            int code = d[2] >> 4, at = 5;
            while (at < (int)n && (d[at] & 0xC0) == 0x80) at++;
            if (code == 1) got = 192;
            else if (code >= 2 && code <= 5) got = 576 << (code - 2);
            else if (code == 6) got = d[at] + 1;
            else if (code == 7) got = (d[at] << 8 | d[at + 1]) + 1;
            else if (code >= 8) got = 256 << (code - 8);
        }
        pos += got;
        k->end = pos;
        if (k == &out->pk[oi->headers < 3 ? oi->headers : 3]) k->page_end = vd != NULL;
    }
    ahvorbis_decoder_free(vd);
    ahogg_close(o);
    return true;
}

static void free_packets(packet_list *l)
{
    for (int i = 0; i < l->count; i++) free(l->pk[i].data.p);
    free(l->pk);
}

/* Files from ffmpeg (libvorbis, and FLAC in Ogg), their packets cut into
   pages again at random: the same sound must come out, to the sample, and
   ffmpeg must agree. */
static void test_repaged(void)
{
    /* The long one is over 1 MB, so that a seek halves the file several
       times before it walks pages. */
    static const struct {
        const char *name, *in, *args;
    } files[] = {
        {"repaged.ogg", "", "-t 2 -c:a libvorbis -q:a 3"},
        {"repaged_6ch.ogg", "", "-t 2 -ac 6 -c:a libvorbis"},
        {"repaged_long.ogg", "-stream_loop 3", "-c:a libvorbis -q:a 9"},
        {"repaged.oga", "", "-t 2 -c:a flac -frame_size 1152"},
        {"repaged_small.oga", "", "-t 2 -ac 1 -c:a flac -frame_size 192"},
    };
    char src[600], path[600], args[2000];
    path_of(src, "source.wav");
    for (size_t f = 0; f < sizeof(files) / sizeof(files[0]); f++) {
        path_of(path, files[f].name);
        snprintf(args, sizeof(args), "%s -i \"%s\" %s \"%s\"", files[f].in, src, files[f].args, path);
        if (!ffmpeg_run(args)) {
            printf("  %s: ffmpeg cannot make it, skipped\n", files[f].name);
            continue;
        }
        bytes orig = {0};
        read_file(path, &orig);
        packet_list l = {0};
        CHECK(take_packets(&orig, &l), "%s: not read", files[f].name);
        pcm want = {0};
        ahreader_info info;
        const char *error = NULL;
        read_through(&orig, &want, &info, &error);
        for (int round = 0; round < 3; round++) {
            bytes again = {0};
            ogg_write(&again, (uint32_t)rnd(), l.pk, l.count, round == 0 ? 1 : round == 1 ? rnd_in(2, 9) : 0);
            pcm got = {0};
            read_through(&again, &got, &info, &error);
            CHECK(same_pcm(&want, &got), "%s cut into pages anew (%d): the sound changed", files[f].name, round);
            char name[64], p2[600];
            snprintf(name, sizeof(name), "again_%d_%s", round, files[f].name);
            path_of(p2, name);
            write_file(p2, again.p, again.len);
            bytes raw = {0};
            if (ffmpeg_raw(p2, NULL, "f32le", &raw)) {
                pcm ref = {(float *)raw.p, raw.len / sizeof(float), 0, info.channels, info.sample_rate};
                CHECK(ref.len == got.len && max_diff(&ref, &got, 0, ref.len) < (info.bits ? 1e-7 : 1e-4), "%s cut into pages anew (%d): differs from ffmpeg",
                      files[f].name, round);
            }
            if (round != 1) check_seeks(name, &again);
            free(raw.p);
            free(got.p);
            free(again.p);
        }
        check_seeks(files[f].name, &orig);
        if (strstr(files[f].name, ".oga")) {
            /* The first packet says how many headers follow; one that says
               too many, or 0 (not known), must still play: a frame ends
               the headers. */
            for (int k = 0; k < 2; k++) {
                l.pk[0].data.p[7] = 0;
                l.pk[0].data.p[8] = (uint8_t)(k ? 9 : 0);
                bytes again = {0};
                ogg_write(&again, 99, l.pk, l.count, 0);
                pcm got = {0};
                read_through(&again, &got, &info, &error);
                CHECK(same_pcm(&want, &got), "%s saying %d headers: the sound changed", files[f].name, k ? 9 : 0);
                free(got.p);
                free(again.p);
            }
        }
        free(want.p);
        free(orig.p);
        free_packets(&l);
    }
}


/* Several streams in one file: an unknown one first, then a Vorbis stream,
   then FLAC, their pages mixed. The first stream that can be decoded is
   the one read, as it would be alone. */
static void test_multiplexed(void)
{
    char src[600], path[600], args[2000];
    path_of(src, "source.wav");
    path_of(path, "mux_flac.oga");
    snprintf(args, sizeof(args), "-i \"%s\" -t 1 -c:a flac \"%s\"", src, path);
    bytes flac = {0};
    if (!ffmpeg_run(args) || !read_file(path, &flac)) return;
    wsetup s;
    random_setup(&s);
    bytes vorbis = {0};
    build_stream(&s, &vorbis, 30, 0, 0, 0, true);
    /* A stream of a kind not handled: its first packet names no codec here. */
    opacket other[3] = {{{0}}};
    bytes_put(&other[0].data, "fishead\0\3\0\0\0", 12);
    other[0].page_end = true;
    bytes_put(&other[1].data, "something else", 14);
    bytes_put(&other[2].data, "and more", 8);
    bytes junk = {0};
    ogg_write(&junk, 7, other, 3, 0);
    size_t at[3][4096], len[3][4096];
    const bytes *src_files[3] = {&junk, &vorbis, &flac};
    int n[3], next[3] = {0, 0, 0};
    for (int k = 0; k < 3; k++) n[k] = pages_of(src_files[k], at[k], len[k], 4096);
    bytes mixed = {0};
    /* First pages first, then the rest mixed at random. */
    for (int k = 0; k < 3; k++) {
        bytes_put(&mixed, src_files[k]->p + at[k][0], len[k][0]);
        next[k] = 1;
    }
    for (;;) {
        int left = 0;
        for (int k = 0; k < 3; k++) left += next[k] < n[k];
        if (!left) break;
        int k;
        do k = rnd_in(0, 2);
        while (next[k] >= n[k]);
        bytes_put(&mixed, src_files[k]->p + at[k][next[k]], len[k][next[k]]);
        next[k]++;
    }
    pcm alone = {0}, together = {0};
    ahreader_info info;
    const char *error = NULL;
    read_through(&vorbis, &alone, &info, &error);
    size_t got = read_through(&mixed, &together, &info, &error);
    CHECK(got && info.kind == AHREADER_VORBIS && same_pcm(&alone, &together), "the Vorbis stream among others came out otherwise (%s)",
          error ? error : "");
    /* And FLAC after a stream of an unknown kind. */
    bytes flac_mixed = {0};
    bytes_put(&flac_mixed, junk.p + at[0][0], len[0][0]);
    for (int k = 0; k < n[2] || k + 1 < n[0]; k++) {
        if (k < n[2]) bytes_put(&flac_mixed, flac.p + at[2][k], len[2][k]);
        if (k + 1 < n[0]) bytes_put(&flac_mixed, junk.p + at[0][k + 1], len[0][k + 1]);
    }
    pcm flac_alone = {0}, flac_together = {0};
    read_through(&flac, &flac_alone, &info, &error);
    got = read_through(&flac_mixed, &flac_together, &info, &error);
    CHECK(got && info.kind == AHREADER_OGG_FLAC && same_pcm(&flac_alone, &flac_together), "the FLAC stream among others came out otherwise");
    free(flac_alone.p);
    free(flac_together.p);
    free(flac_mixed.p);
    check_seeks("the Vorbis stream among others", &mixed);
    free(alone.p);
    free(together.p);
    free(mixed.p);
    free(junk.p);
    for (int k = 0; k < 3; k++) free(other[k].data.p);
    free(vorbis.p);
    free(flac.p);
    free_setup(&s);
}

/* render_line as the specification writes it, drawn whole. */
static void spec_line(int x0, int y0, int x1, int y1, int *v)
{
    int dy = y1 - y0, adx = x1 - x0, ady = abs(dy), base = dy / adx, x = x0, y = y0, err = 0;
    int sy = dy < 0 ? base - 1 : base + 1;
    ady -= abs(base) * adx;
    v[x] = y;
    for (x = x0 + 1; x < x1; x++) {
        err += ady;
        if (err >= adx) {
            err -= adx;
            y += sy;
        } else {
            y += base;
        }
        v[x] = y;
    }
}

/* A type 1 floor's lines, against the specification's steps followed to
   the letter, chiefly where points lie past the end of the block (a floor
   spanning a long block, used by a short one; or one wider than any
   block). ffmpeg ends such a line at the block's end instead, which bends
   it (see the README); the random streams keep clear of that. */
static void test_floor_lines(void)
{
    static int v[1 << 16];
    static float out[4096], want[4096];
    int beyond = 0;
    for (int round = 0; round < 3000; round++) {
        int n = 1 << rnd_in(5, 12), count = rnd_in(2, 65), reach = rnd() % 2 ? n : 2 * n + 50;
        int x[65], y[65];
        x[0] = 0;
        for (int i = 1; i < count; i++) {
            x[i] = x[i - 1] + rnd_in(1, reach / count + 1);
            y[i] = 0;
        }
        for (int i = 0; i < count; i++) y[i] = rnd() % 8 ? rnd_in(0, 255) : rnd() % 2 ? 0 : 255;
        for (int i = 1; i < count; i++) spec_line(x[i - 1], y[i - 1], x[i], y[i], v);
        if (x[count - 1] < n) spec_line(x[count - 1], y[count - 1], n, y[count - 1], v);
        for (int i = 0; i < n; i++) want[i] = ahvorbis_inverse_db[v[i]];
        ahvorbis_floor1_lines(x, y, count, out, n);
        beyond += x[count - 1] > n;
        if (memcmp(out, want, (size_t)n * sizeof(float))) {
            CHECK(false, "floor lines round %d (n %d, %d points to %d) differ from the specification's", round, n, count, x[count - 1]);
            break;
        }
    }
    CHECK(beyond > 500, "floor lines past the block's end were tried only %d times", beyond);
}

/* Damage: setup headers with bits changed, and sound packets cut short or
   changed. Nothing may crash or run away; a packet cut short is decoded as
   far as it goes, as the specification says. */
static void test_damage(void)
{
    int refused = 0, accepted = 0, cut = 0;
    static float pcm[8 * AHVORBIS_MAX_SAMPLES];
    for (int round = 0; round < 80; round++) {
        wsetup s;
        random_setup(&s);
        bytes file = {0};
        build_stream(&s, &file, rnd_in(4, 20), 0, 0, 0, false);
        packet_list l = {0};
        take_packets(&file, &l);
        for (int k = 0; k < 4; k++) {
            ahvorbis_decoder *d = ahvorbis_decoder_new();
            bool ok = true;
            for (int h = 0; h < 3 && h < l.count && ok; h++) {
                bytes hd = {0};
                bytes_put(&hd, l.pk[h].data.p, l.pk[h].data.len);
                if (h == 2 && k) {
                    if (k == 3) hd.len = (size_t)rnd_in(7, (int)hd.len - 1);
                    else
                        for (int f = 0; f < k * 3; f++) hd.p[rnd_in(7, (int)hd.len - 1)] ^= (unsigned char)(1 << rnd_in(0, 7));
                }
                ok = ahvorbis_decoder_header(d, hd.p, hd.len, NULL);
                if (!ok) CHECK(ahvorbis_decoder_error(d) != NULL, "a refused header gave no reason");
                free(hd.p);
            }
            refused += !ok;
            accepted += ok && k;
            for (int i = 3; i < l.count; i++) {
                bytes pk = {0};
                bytes_put(&pk, l.pk[i].data.p, l.pk[i].data.len);
                if (rnd() % 3 == 0 && pk.len > 1) {
                    pk.len = (size_t)rnd_in(1, (int)pk.len - 1);
                    cut++;
                }
                if (rnd() % 4 == 0 && pk.len) pk.p[rnd_in(0, (int)pk.len - 1)] ^= (unsigned char)rnd();
                int n = -1;
                bool good = ahvorbis_decode_packet(d, pk.p, pk.len, pcm, &n);
                CHECK(n >= 0 && n <= AHVORBIS_MAX_SAMPLES && (good || n == 0), "a damaged packet gave %d samples", n);
                free(pk.p);
            }
            /* Nothing given at all. */
            int n = -1;
            ahvorbis_decode_packet(d, NULL, 0, pcm, &n);
            CHECK(n == 0, "an empty packet gave sound");
            ahvorbis_decoder_free(d);
        }
        free_packets(&l);
        free(file.p);
        free_setup(&s);
    }
    printf("  damaged setups: %d refused, %d taken anyway; %d packets cut short\n", refused, accepted, cut);
    CHECK(refused > 20 && cut > 100, "too little damage was tried");
    /* Headers out of order, or not Vorbis at all. */
    ahvorbis_decoder *d = ahvorbis_decoder_new();
    CHECK(!ahvorbis_decoder_header(d, (const uint8_t *)"\x05vorbis", 7, NULL) && ahvorbis_decoder_error(d), "a setup header was taken first");
    ahvorbis_decoder_free(d);
    d = ahvorbis_decoder_new();
    CHECK(!ahvorbis_decoder_header(d, (const uint8_t *)"OpusHead", 8, NULL), "Opus was taken for Vorbis");
    int n = 0;
    CHECK(!ahvorbis_decode_packet(d, (const uint8_t *)"\0\0\0\0", 4, pcm, &n) && n == 0, "sound was decoded without headers");
    ahvorbis_decoder_free(d);
}

/* Short files from libvorbis, all their sound on one page: they must come
   back exactly as long as what went into the encoder (ffmpeg's are 128
   samples short; see the README). */
static void test_short_files(void)
{
    static const char *lengths[] = {"0.01", "0.1", "0.5", "1"};
    static const int samples[] = {441, 4410, 22050, 44100};
    for (int i = 0; i < 4; i++) {
        char path[600], args[1200];
        path_of(path, "short.ogg");
        snprintf(args, sizeof(args), "-f lavfi -i \"sine=f=440:d=%s\" -ar 44100 -ac %d -c:a libvorbis \"%s\"", lengths[i], 1 + i % 2, path);
        if (!ffmpeg_run(args)) {
            printf("  libvorbis not available in this ffmpeg, skipped\n");
            return;
        }
        bytes file = {0};
        read_file(path, &file);
        pcm got = {0};
        ahreader_info info;
        const char *error = NULL;
        size_t n = read_through(&file, &got, &info, &error);
        CHECK(n == (size_t)samples[i] && info.samples == (uint64_t)samples[i], "%s seconds from libvorbis: %zu samples, not %d", lengths[i], n,
              samples[i]);
        free(got.p);
        free(file.p);
    }
}

/* A setup header of a few bytes that asks for codebooks of a million
   entries each must be refused at once, not built. */
static void test_huge_setup(void)
{
    wsetup s = {0};
    s.channels = 1;
    s.rate = 44100;
    s.e0 = s.e1 = 8;
    bitw id = {0}, setup = {0};
    put_identification(&id, &s);
    put_text(&setup, "\x05vorbis", 7);
    put(&setup, 255, 8);
    for (int b = 0; b < 256; b++) {
        put(&setup, 0x564342, 24);
        put(&setup, 1, 16);
        put(&setup, 1 << 20, 24);
        put(&setup, 1, 1);      /* ordered */
        put(&setup, 19, 5);     /* all of length 20: a complete code */
        put(&setup, 1 << 20, 21);
        put(&setup, 0, 4);
    }
    finish(&setup);
    ahvorbis_decoder *d = ahvorbis_decoder_new();
    bool ok = ahvorbis_decoder_header(d, id.b.p, id.b.len, NULL) && ahvorbis_decoder_header(d, (const uint8_t *)"\x03vorbis\0\0\0\0\0\0\0\0\1", 16, NULL);
    CHECK(ok, "the headers before the setup were refused");
    ok = ahvorbis_decoder_header(d, setup.b.p, setup.b.len, NULL);
    const char *why = ahvorbis_decoder_error(d);
    CHECK(!ok && why && strstr(why, "too big"), "codebooks of 256 million entries were %s", ok ? "taken" : why ? why : "refused without a reason");
    ahvorbis_decoder_free(d);
    free(id.b.p);
    free(setup.b.p);
}

/* A type 0 floor that names a book past its list makes the packet
   undecodable (the specification's words): no sound from it, and the
   packets after it decode as they would have. */
static void test_bad_floor_book(void)
{
    int bad = 0, good = 0;
    static float pcm[8 * AHVORBIS_MAX_SAMPLES];
    for (int round = 0; round < 60 && bad < 20; round++) {
        wsetup s;
        random_setup(&s);
        bytes file = {0};
        int packets = rnd_in(4, 30);
        g_bad_floor0 = true;
        build_stream(&s, &file, packets, 0, 0, 0, false);
        g_bad_floor0 = false;
        packet_list l = {0};
        take_packets(&file, &l);
        ahvorbis_decoder *d = ahvorbis_decoder_new();
        for (int h = 0; h < 3; h++) ahvorbis_decoder_header(d, l.pk[h].data.p, l.pk[h].data.len, NULL);
        for (int i = 0; i < packets && 3 + i < l.count; i++) {
            int n = -1;
            bool ok = ahvorbis_decode_packet(d, l.pk[3 + i].data.p, l.pk[3 + i].data.len, pcm, &n);
            if (g_packet_bad[i]) {
                CHECK(!ok && n == 0, "a packet naming a floor book that is not there was decoded");
                bad++;
            } else {
                CHECK(ok, "a packet after one naming a missing floor book was refused");
                good++;
            }
        }
        ahvorbis_decoder_free(d);
        free_packets(&l);
        free(file.p);
        free_setup(&s);
    }
    CHECK(bad >= 20 && good > 0, "too few packets with a missing floor book (%d)", bad);
}

/* Pages damaged in ways their checksum does not catch (it is worked out
   again after the change): flags, positions, numbers, segment sizes, the
   data. Nothing may crash or run away. And a file of nothing but false
   page starts. */
static void test_page_fuzz(void)
{
    char src[600], path[600], args[2000];
    path_of(src, "source.wav");
    path_of(path, "fuzz.oga");
    snprintf(args, sizeof(args), "-i \"%s\" -t 1 -c:a flac \"%s\"", src, path);
    bytes flac = {0};
    bool have_flac = g_ffmpeg && ffmpeg_run(args) && read_file(path, &flac);
    for (int round = 0; round < 300; round++) {
        bytes base = {0};
        wsetup s = {0};
        if (have_flac && round % 3 == 0) {
            bytes_put(&base, flac.p, flac.len);
        } else {
            random_setup(&s);
            build_stream(&s, &base, rnd_in(4, 20), 0, 0, round % 2 ? 0 : rnd_in(1, 4), false);
            free_setup(&s);
        }
        size_t at[4096], len[4096];
        int n = pages_of(&base, at, len, 4096);
        pcm whole = {0};
        ahreader_info info;
        const char *error = NULL;
        size_t frames = read_through(&base, &whole, &info, &error);
        for (int k = 0; k < 3 && n; k++) {
            int pg = rnd_in(0, n - 1);
            uint8_t *p = base.p + at[pg];
            switch (rnd_in(0, 5)) {
            case 0: p[5] ^= (uint8_t)(1 << rnd_in(0, 2)); break;              /* flags */
            case 1: p[6 + rnd_in(0, 7)] = (uint8_t)rnd(); break;              /* position */
            case 2: p[18 + rnd_in(0, 3)] = (uint8_t)rnd(); break;             /* page number */
            case 3: p[14 + rnd_in(0, 3)] ^= 1; break;                         /* stream number */
            case 4: if (p[26]) p[27 + rnd_in(0, p[26] - 1)] = (uint8_t)rnd(); break; /* a segment size */
            default:
                if (len[pg] > 27u + p[26]) p[27 + p[26] + (size_t)rnd_in(0, (int)(len[pg] - 28 - p[26]))] = (uint8_t)rnd();
            }
            /* The checksum again, over what the page now says it holds. */
            size_t size = 27 + p[26];
            for (int q = 0; q < p[26]; q++) size += p[27 + q];
            if (at[pg] + size <= base.len) {
                memset(p + 22, 0, 4);
                uint32_t crc = ogg_crc(p, size);
                for (int q = 0; q < 4; q++) p[22 + q] = (uint8_t)(crc >> (8 * q));
            }
        }
        pcm junk = {0};
        ahreader_info bad_info;
        if (read_through(&base, &junk, &bad_info, &error)) {
            CHECK(junk.len / (size_t)bad_info.channels <= frames * 2 + 100000, "a damaged Ogg file gave %zu samples", junk.len / (size_t)bad_info.channels);
            ahreader *r = ahreader_open(test_memory_read, &base, base.len, NULL);
            float few[8 * 64];
            for (int k = 0; r && k < 6; k++) {
                ahreader_seek(r, (uint64_t)rnd_in(0, (int)frames + 10));
                ahreader_read(r, few, 64);
            }
            ahreader_close(r);
        }
        free(junk.p);
        free(whole.p);
        free(base.p);
    }
    free(flac.p);
    /* A megabyte of false page starts, each claiming a long page. */
    bytes fake = {0};
    while (fake.len < (1 << 20)) {
        uint8_t head[27 + 255] = {'O', 'g', 'g', 'S', 0, 2};
        head[26] = 255;
        memset(head + 27, 255, 255);
        bytes_put(&fake, head, 27 + (size_t)rnd_in(0, 255));
    }
    const char *error = NULL;
    ahreader *r = ahreader_open(test_memory_read, &fake, fake.len, &error);
    CHECK(!r && error, "false page starts were taken for a file");
    ahreader_close(r);
    free(fake.p);
    /* Opus is named in the refusal. */
    opacket opus[2] = {{{0}}};
    bytes_put(&opus[0].data, "OpusHead\1\2\0\0\x80\xBB\0\0\0\0\0", 19);
    opus[0].page_end = true;
    bytes_put(&opus[1].data, "OpusTags", 8);
    bytes file = {0};
    ogg_write(&file, 5, opus, 2, 0);
    r = ahreader_open(test_memory_read, &file, file.len, &error);
    CHECK(!r && error && strstr(error, "Opus"), "an Ogg Opus file: %s", r ? "opened" : error ? error : "refused without a reason");
    ahreader_close(r);
    free(file.p);
    free(opus[0].data.p);
    free(opus[1].data.p);
}

void test_vorbis(void)
{
    /* These need no ffmpeg. */
    test_floor_lines();
    test_huge_setup();
    test_bad_floor_book();
    test_damage();
    test_one_page();
    test_seeks();
    if (!g_ffmpeg) {
        test_page_fuzz();
        return;
    }
    make_source();
    test_page_fuzz();
    test_random_streams();
    test_trims();
    test_repaged();
    test_multiplexed();
    test_short_files();
}
