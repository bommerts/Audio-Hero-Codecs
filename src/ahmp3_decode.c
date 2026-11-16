/* MPEG audio decoder, following ISO/IEC 11172-3 (MPEG-1) and 13818-3
   (MPEG-2 low sample rates), with the MPEG-2.5 extension. Layer III is here;
   Layers I and II share this file's framing and filter bank. Where the
   standard leaves room, it behaves like the decoders in common use, so files
   that play elsewhere play here the same. */
#include "ahmp3_internal.h"

#define MAX_FREE_FRAME 4096 /* the largest free-format frame looked for */
#define RESERVOIR 5120      /* more than any main_data_begin (511) plus one frame */

typedef struct {
    int part23, big_values, global_gain, scalefac_compress;
    int block_type, mixed, preflag, scalefac_scale, count1_table;
    int table[3], subblock_gain[3];
    int region1, region2; /* first line of Huffman regions 1 and 2 */
    int long_end;         /* long scale factor bands in this granule */
    int short_start;      /* first short scale factor band (13 = none) */
    int scfsi;
    unsigned char sf[40]; /* long bands, then short bands x 3 windows */
} granule;

/* A decoding tree per Huffman table: node pairs, leaves stored as ~value. */
typedef struct {
    short node[2];
} hnode;

typedef struct {
    hnode *nodes;
    int count;
} htree;

struct ahmp3_decoder {
    ahmp3_tables t;
    float synth[64][32]; /* cos((16 + i)(2k + 1) pi / 64) */
    htree trees[32];
    htree quad;
    hnode node_pool[2048];
    int pool_used;

    unsigned char res[RESERVOIR];
    int res_len;
    float overlap[2][32][18];
    float V[2][1024];

    float sub12[2][36][32]; /* subband samples of a Layer I or II frame */

    ahmp3_header last;
    int free_base;          /* frame size of a free-format stream, less padding */
    bool synced;
    bool started; /* the first frame has been looked at for an Info header */
    size_t skip_pending;
    ahmp3_info info;
};

/* ---- Huffman trees --------------------------------------------------------- */

static bool tree_build(ahmp3_decoder *d, htree *tr, const unsigned short *codes16, const unsigned char *codes8,
                       const unsigned char *lens, int count)
{
    tr->nodes = d->node_pool + d->pool_used;
    tr->count = 1;
    tr->nodes[0].node[0] = tr->nodes[0].node[1] = 0;
    for (int v = 0; v < count; v++) {
        unsigned code = codes16 ? codes16[v] : codes8[v];
        int len = lens[v], n = 0;
        for (int b = len - 1; b >= 0; b--) {
            int bit = (code >> b) & 1;
            if (b == 0) {
                tr->nodes[n].node[bit] = (short)~v;
            } else {
                if (tr->nodes[n].node[bit] == 0) {
                    if (d->pool_used + tr->count >= (int)(sizeof(d->node_pool) / sizeof(d->node_pool[0]))) return false;
                    tr->nodes[n].node[bit] = (short)tr->count;
                    tr->nodes[tr->count].node[0] = tr->nodes[tr->count].node[1] = 0;
                    tr->count++;
                }
                n = tr->nodes[n].node[bit];
            }
        }
    }
    d->pool_used += tr->count;
    return true;
}

static int tree_read(const htree *tr, ahmp3_bits *b)
{
    int n = 0;
    for (int depth = 0; depth < 24; depth++) {
        int next = tr->nodes[n].node[ahmp3_get_bits(b, 1)];
        if (next < 0) return ~next;
        if (next == 0) return 0; /* not a code: only in a broken stream */
        n = next;
    }
    return 0;
}

/* ---- setup ----------------------------------------------------------------- */

ahmp3_decoder *ahmp3_decoder_new(void)
{
    ahmp3_decoder *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    ahmp3_tables_init(&d->t);
    for (int i = 0; i < 64; i++)
        for (int k = 0; k < 32; k++)
            d->synth[i][k] = (float)cos((16 + i) * (2 * k + 1) * AHMP3_PI / 64);
    for (int t = 0; t < 32; t++) {
        const ahmp3_huff *h = &ahmp3_huff_tables[t];
        if (!h->size) continue;
        if (t > 16 && t < 24) { d->trees[t] = d->trees[16]; continue; }
        if (t > 24) { d->trees[t] = d->trees[24]; continue; }
        if (!tree_build(d, &d->trees[t], h->codes, NULL, h->lens, h->size * h->size)) abort();
    }
    if (!tree_build(d, &d->quad, NULL, ahmp3_count1_codes[0], ahmp3_count1_lens[0], 16)) abort();
    ahmp3_decoder_reset(d);
    return d;
}

void ahmp3_decoder_free(ahmp3_decoder *d)
{
    free(d);
}

void ahmp3_decoder_reset(ahmp3_decoder *d)
{
    d->res_len = 0;
    memset(d->overlap, 0, sizeof(d->overlap));
    memset(d->V, 0, sizeof(d->V));
    d->synced = false;
    d->free_base = 0;
    d->skip_pending = 0;
}

const ahmp3_info *ahmp3_decoder_info(const ahmp3_decoder *d)
{
    return &d->info;
}

/* ---- side information ------------------------------------------------------ */

static bool read_side_info(ahmp3_bits *b, const ahmp3_header *h, int *main_data_begin, granule gr[2][2])
{
    int nch = h->channels, ngr = h->lsf ? 1 : 2;
    int scfsi[2] = {0, 0};
    if (h->lsf) {
        *main_data_begin = (int)ahmp3_get_bits(b, 8);
        ahmp3_get_bits(b, nch == 1 ? 1 : 2);
    } else {
        *main_data_begin = (int)ahmp3_get_bits(b, 9);
        ahmp3_get_bits(b, nch == 1 ? 5 : 3);
        for (int ch = 0; ch < nch; ch++) scfsi[ch] = (int)ahmp3_get_bits(b, 4);
    }
    for (int g = 0; g < ngr; g++) {
        for (int ch = 0; ch < nch; ch++) {
            granule *G = &gr[g][ch];
            memset(G, 0, sizeof(*G));
            G->scfsi = g ? scfsi[ch] : 0;
            G->part23 = (int)ahmp3_get_bits(b, 12);
            G->big_values = (int)ahmp3_get_bits(b, 9);
            if (G->big_values > 288) return false;
            G->global_gain = (int)ahmp3_get_bits(b, 8);
            G->scalefac_compress = (int)ahmp3_get_bits(b, h->lsf ? 9 : 4);
            int region0_end;
            if (ahmp3_get_bits(b, 1)) { /* window switching */
                G->block_type = (int)ahmp3_get_bits(b, 2);
                if (G->block_type == 0) return false;
                G->mixed = (int)ahmp3_get_bits(b, 1);
                for (int i = 0; i < 2; i++) G->table[i] = (int)ahmp3_get_bits(b, 5);
                for (int i = 0; i < 3; i++) G->subblock_gain[i] = (int)ahmp3_get_bits(b, 3);
                if (G->block_type == 2)
                    region0_end = h->sr_index == 8 ? 72 : 36;
                else
                    region0_end = ahmp3_sfb_long[h->sr_index][8];
                G->region1 = region0_end;
                G->region2 = 576;
            } else {
                for (int i = 0; i < 3; i++) G->table[i] = (int)ahmp3_get_bits(b, 5);
                int r0 = (int)ahmp3_get_bits(b, 4), r1 = (int)ahmp3_get_bits(b, 3);
                G->region1 = ahmp3_sfb_long[h->sr_index][r0 + 1];
                int l = r0 + r1 + 2;
                G->region2 = ahmp3_sfb_long[h->sr_index][l > 22 ? 22 : l];
            }
            if (!h->lsf) G->preflag = (int)ahmp3_get_bits(b, 1);
            G->scalefac_scale = (int)ahmp3_get_bits(b, 1);
            G->count1_table = (int)ahmp3_get_bits(b, 1);

            if (G->block_type == 2) {
                G->long_end = G->mixed ? (h->lsf ? 6 : 8) : 0;
                G->short_start = G->mixed ? 3 : 0;
            } else {
                G->long_end = 22;
                G->short_start = 13;
            }
        }
    }
    return true;
}

/* ---- scale factors --------------------------------------------------------- */

static void read_scalefactors_mpeg1(ahmp3_bits *b, granule *G, const granule *first)
{
    int slen1 = ahmp3_slen[0][G->scalefac_compress], slen2 = ahmp3_slen[1][G->scalefac_compress];
    int j = 0;
    if (G->block_type == 2) {
        int n = G->mixed ? 17 : 18;
        for (int i = 0; i < n; i++) G->sf[j++] = (unsigned char)ahmp3_get_bits(b, slen1);
        for (int i = 0; i < 18; i++) G->sf[j++] = (unsigned char)ahmp3_get_bits(b, slen2);
        for (int i = 0; i < 3; i++) G->sf[j++] = 0;
        return;
    }
    static const int group_start[5] = {0, 6, 11, 16, 21};
    for (int k = 0; k < 4; k++) {
        int slen = k < 2 ? slen1 : slen2;
        for (int i = group_start[k]; i < group_start[k + 1]; i++)
            G->sf[i] = first && (G->scfsi & (8 >> k)) ? first->sf[i] : (unsigned char)ahmp3_get_bits(b, slen);
    }
    G->sf[21] = 0;
}

/* Splits v into four lengths, most significant first; a 0 divisor gives 0. */
static void expand(int v, int n1, int n2, int n3, int slen[4])
{
    slen[3] = n3 ? v % n3 : 0;
    if (n3) v /= n3;
    slen[2] = n2 ? v % n2 : 0;
    if (n2) v /= n2;
    slen[1] = v % n1;
    slen[0] = v / n1;
}

static void read_scalefactors_lsf(ahmp3_bits *b, granule *G, bool intensity_right)
{
    int slen[4], table, sf = G->scalefac_compress;
    if (intensity_right) {
        sf >>= 1;
        if (sf < 180) { expand(sf, 6, 6, 0, slen); table = 3; }
        else if (sf < 244) { expand(sf - 180, 4, 4, 0, slen); table = 4; }
        else { expand(sf - 244, 3, 0, 0, slen); table = 5; }
    } else {
        if (sf < 400) { expand(sf, 5, 4, 4, slen); table = 0; }
        else if (sf < 500) { expand(sf - 400, 5, 4, 0, slen); table = 1; }
        else { expand(sf - 500, 3, 0, 0, slen); table = 2; G->preflag = 1; }
    }
    int kind = G->block_type == 2 ? (G->mixed ? 2 : 1) : 0;
    int j = 0;
    for (int k = 0; k < 4; k++)
        for (int i = 0; i < ahmp3_lsf_nsf[table][kind][k]; i++)
            G->sf[j++] = (unsigned char)ahmp3_get_bits(b, slen[k]);
    while (j < 40) G->sf[j++] = 0;
}

/* ---- Huffman data ---------------------------------------------------------- */

static void read_huffman(ahmp3_decoder *d, ahmp3_bits *b, const granule *G, size_t end, int is[576])
{
    memset(is, 0, 576 * sizeof(int));
    int big = G->big_values * 2;
    int bounds[3] = {G->region1 < big ? G->region1 : big, G->region2 < big ? G->region2 : big, big};
    if (bounds[1] < bounds[0]) bounds[1] = bounds[0];
    int line = 0;
    for (int r = 0; r < 3; r++) {
        int t = G->table[r];
        const ahmp3_huff *h = &ahmp3_huff_tables[t];
        if (!h->size) { /* table 0 (and the unused 4, 14): all zero */
            line = bounds[r];
            continue;
        }
        const htree *tr = &d->trees[t];
        for (; line < bounds[r]; line += 2) {
            int v = tree_read(tr, b);
            int x = v / h->size, y = v % h->size;
            if (x == 15 && h->linbits) x += (int)ahmp3_get_bits(b, h->linbits);
            if (x && ahmp3_get_bits(b, 1)) x = -x;
            if (y == 15 && h->linbits) y += (int)ahmp3_get_bits(b, h->linbits);
            if (y && ahmp3_get_bits(b, 1)) y = -y;
            is[line] = x;
            is[line + 1] = y;
        }
    }
    /* count1: quadruples of -1, 0, 1 until the granule's bits run out. A
       quadruple that runs past the end is dropped. */
    while (line <= 572 && b->pos < end) {
        int v;
        if (G->count1_table) v = 15 - (int)ahmp3_get_bits(b, 4);
        else v = tree_read(&d->quad, b);
        int q[4] = {(v >> 3) & 1, (v >> 2) & 1, (v >> 1) & 1, v & 1};
        for (int i = 0; i < 4; i++)
            if (q[i] && ahmp3_get_bits(b, 1)) q[i] = -1;
        if (b->pos > end) break;
        for (int i = 0; i < 4; i++) is[line + i] = q[i];
        line += 4;
    }
}

/* ---- requantization -------------------------------------------------------- */

static void requantize(const ahmp3_decoder *d, const ahmp3_header *h, const granule *G, const int is[576], float xr[576])
{
    int shift = G->scalefac_scale + 1;
    int gain = G->global_gain - 210;
    const short *sl = ahmp3_sfb_long[h->sr_index], *ss = ahmp3_sfb_short[h->sr_index];
    for (int i = 0; i < G->long_end; i++) {
        int pre = G->preflag ? ahmp3_pretab[i] : 0;
        int e = gain - ((G->sf[i] + pre) << shift);
        float f = (float)pow(2.0, e / 4.0);
        for (int j = sl[i]; j < sl[i + 1]; j++) {
            int v = is[j];
            xr[j] = v == 0 ? 0 : v > 0 ? d->t.pow43[v] * f : -d->t.pow43[-v] * f;
        }
    }
    int k = G->long_end;
    for (int i = G->short_start; i < 13; i++) {
        int len = ss[i + 1] - ss[i];
        for (int w = 0; w < 3; w++) {
            int e = gain - 8 * G->subblock_gain[w] - (G->sf[k++] << shift);
            float f = (float)pow(2.0, e / 4.0);
            int start = 3 * ss[i] + w * len;
            for (int j = start; j < start + len; j++) {
                int v = is[j];
                xr[j] = v == 0 ? 0 : v > 0 ? d->t.pow43[v] * f : -d->t.pow43[-v] * f;
            }
        }
    }
}

/* ---- stereo ---------------------------------------------------------------- */

static void ms_band(float *l, float *r, int n)
{
    const float s = (float)(1 / sqrt(2.0));
    for (int j = 0; j < n; j++) {
        float m = l[j], d = r[j];
        l[j] = (m + d) * s;
        r[j] = (m - d) * s;
    }
}

static bool any_nonzero(const float *v, int n)
{
    for (int j = 0; j < n; j++)
        if (v[j] != 0) return true;
    return false;
}

static void stereo(const ahmp3_header *h, const granule *right, float xr[2][576])
{
    bool ms = (h->mode_ext & 2) != 0;
    if (!(h->mode_ext & 1)) {
        if (ms) ms_band(xr[0], xr[1], 576);
        return;
    }

    /* Intensity stereo: bands above the right channel's last non-zero band
       carry the left channel's signal, placed by the right's scale factor. */
    float ratio[2][16];
    int sf_max;
    if (!h->lsf) {
        sf_max = 7;
        for (int i = 0; i < 7; i++) {
            double v = i == 6 ? 1.0 : tan(i * AHMP3_PI / 12) / (1 + tan(i * AHMP3_PI / 12));
            ratio[0][i] = (float)v;
            ratio[1][6 - i] = (float)v;
        }
    } else {
        sf_max = 16;
        int j = right->scalefac_compress & 1;
        for (int i = 0; i < 16; i++) {
            float f = (float)pow(2.0, -(j + 1) * ((i + 1) >> 1) / 4.0);
            ratio[(i & 1) ^ 1][i] = f;
            ratio[i & 1][i] = 1;
        }
    }

    const short *sl = ahmp3_sfb_long[h->sr_index], *ss = ahmp3_sfb_short[h->sr_index];
    bool found_short[3] = {false, false, false};
    int k = (13 - right->short_start) * 3 + right->long_end - 3;
    for (int i = 12; i >= right->short_start; i--) {
        if (i != 11) k -= 3; /* the last band uses the band before's position */
        int len = ss[i + 1] - ss[i];
        for (int w = 2; w >= 0; w--) {
            int start = 3 * ss[i] + w * len;
            float *l = xr[0] + start, *r = xr[1] + start;
            int sf = right->sf[k + w];
            if (!found_short[w] && any_nonzero(r, len)) found_short[w] = true;
            if (!found_short[w] && sf < sf_max) {
                for (int j = 0; j < len; j++) {
                    float v = l[j];
                    l[j] = v * ratio[0][sf];
                    r[j] = v * ratio[1][sf];
                }
            } else if (ms) {
                ms_band(l, r, len);
            }
        }
    }
    bool found = found_short[0] || found_short[1] || found_short[2];
    for (int i = right->long_end - 1; i >= 0; i--) {
        int len = sl[i + 1] - sl[i];
        float *l = xr[0] + sl[i], *r = xr[1] + sl[i];
        int sf = right->sf[i == 21 ? 20 : i];
        if (!found && any_nonzero(r, len)) found = true;
        if (!found && sf < sf_max) {
            for (int j = 0; j < len; j++) {
                float v = l[j];
                l[j] = v * ratio[0][sf];
                r[j] = v * ratio[1][sf];
            }
        } else if (ms) {
            ms_band(l, r, len);
        }
    }
}

/* ---- reorder, alias reduction, IMDCT, synthesis ---------------------------- */

static void reorder(const ahmp3_header *h, const granule *G, float xr[576])
{
    if (G->block_type != 2) return;
    const short *ss = ahmp3_sfb_short[h->sr_index];
    float tmp[576];
    int start = G->mixed ? (h->sr_index == 8 ? 72 : 36) : 0;
    int pos = start;
    for (int i = G->short_start; i < 13; i++) {
        int len = ss[i + 1] - ss[i];
        for (int j = 0; j < len; j++)
            for (int w = 0; w < 3; w++) tmp[3 * j + w] = xr[pos + w * len + j];
        memcpy(xr + pos, tmp, 3 * len * sizeof(float));
        pos += 3 * len;
    }
}

static void alias_reduce(const ahmp3_decoder *d, const granule *G, float xr[576])
{
    int bands = G->block_type == 2 ? (G->mixed ? 1 : 0) : 31;
    for (int sb = 1; sb <= bands; sb++) {
        for (int i = 0; i < 8; i++) {
            float lo = xr[18 * sb - 1 - i], hi = xr[18 * sb + i];
            xr[18 * sb - 1 - i] = lo * d->t.cs[i] - hi * d->t.ca[i];
            xr[18 * sb + i] = hi * d->t.cs[i] + lo * d->t.ca[i];
        }
    }
}

/* Turns one granule of one channel into 18 time samples for each of the 32
   subbands: out[slot][subband]. */
static void imdct(ahmp3_decoder *d, int ch, const granule *G, const float xr[576], float out[18][32])
{
    for (int sb = 0; sb < 32; sb++) {
        const float *X = xr + 18 * sb;
        float y[36];
        bool long_block = G->block_type != 2 || (G->mixed && sb < 2);
        if (long_block) {
            const float *win = d->t.win[G->block_type == 2 ? 0 : G->block_type];
            for (int i = 0; i < 36; i++) y[i] = 0;
            for (int k = 0; k < 18; k++) {
                float v = X[k];
                if (v == 0) continue;
                const float *c = d->t.imdct_long[k];
                for (int i = 0; i < 36; i++) y[i] += v * c[i];
            }
            for (int i = 0; i < 36; i++) y[i] *= win[i];
        } else {
            for (int i = 0; i < 36; i++) y[i] = 0;
            for (int w = 0; w < 3; w++) {
                float z[12] = {0};
                for (int k = 0; k < 6; k++) {
                    float v = X[3 * k + w];
                    if (v == 0) continue;
                    const float *c = d->t.imdct_short[k];
                    for (int i = 0; i < 12; i++) z[i] += v * c[i];
                }
                for (int i = 0; i < 12; i++) y[6 + 6 * w + i] += z[i] * d->t.win[2][i];
            }
        }
        float *prev = d->overlap[ch][sb];
        for (int i = 0; i < 18; i++) {
            float v = y[i] + prev[i];
            prev[i] = y[18 + i];
            /* frequency inversion: odd time samples of odd subbands */
            out[i][sb] = (sb & 1) && (i & 1) ? -v : v;
        }
    }
}

static void synthesize(ahmp3_decoder *d, int ch, const float (*in)[32], int slots, float *pcm, int stride)
{
    float *V = d->V[ch];
    for (int t = 0; t < slots; t++) {
        memmove(V + 64, V, (1024 - 64) * sizeof(float));
        const float *S = in[t];
        for (int i = 0; i < 64; i++) {
            const float *n = d->synth[i];
            float sum = 0;
            for (int k = 0; k < 32; k++) sum += n[k] * S[k];
            V[i] = sum;
        }
        for (int j = 0; j < 32; j++) {
            float sum = 0;
            for (int i = 0; i < 8; i++) {
                sum += V[128 * i + j] * d->t.D[64 * i + j];
                sum += V[128 * i + 96 + j] * d->t.D[64 * i + 32 + j];
            }
            pcm[(t * 32 + j) * stride] = sum;
        }
    }
}

/* ---- Xing / Info / VBRI ---------------------------------------------------- */

static unsigned be32(const unsigned char *p)
{
    return (unsigned)p[0] << 24 | (unsigned)p[1] << 16 | (unsigned)p[2] << 8 | p[3];
}

static bool read_info_frame(const unsigned char *f, const ahmp3_header *h, ahmp3_info *info)
{
    memset(info, 0, sizeof(*info));
    info->encoder_delay = info->encoder_padding = -1;
    const unsigned char *p = f + 4 + 2 * h->crc + h->side_bytes;
    const unsigned char *end = f + h->frame_bytes;
    if (p + 8 <= end && (!memcmp(p, "Xing", 4) || !memcmp(p, "Info", 4))) {
        unsigned flags = be32(p + 4);
        p += 8;
        if ((flags & 1) && p + 4 <= end) { info->frames = be32(p); p += 4; }
        if ((flags & 2) && p + 4 <= end) { info->bytes = be32(p); p += 4; }
        if (flags & 4) p += 100;
        if (flags & 8) p += 4;
        /* A LAME-style tag follows: 9 characters of encoder name, then the
           delay and padding as two 12-bit numbers 21 bytes in. */
        if (p + 24 <= end) {
            bool name = true;
            for (int i = 0; i < 4; i++)
                if (!((p[i] >= 'A' && p[i] <= 'Z') || (p[i] >= 'a' && p[i] <= 'z') || (p[i] >= '0' && p[i] <= '9')))
                    name = false;
            if (name) {
                info->encoder_delay = (p[21] << 4) | (p[22] >> 4);
                info->encoder_padding = ((p[22] & 15) << 8) | p[23];
            }
        }
        info->present = true;
        return true;
    }
    p = f + 4 + 32;
    if (p + 18 <= end && !memcmp(p, "VBRI", 4)) {
        info->bytes = be32(p + 10);
        info->frames = be32(p + 14);
        info->present = true;
        return true;
    }
    return false;
}

/* ---- frames ---------------------------------------------------------------- */

static bool same_stream(const ahmp3_header *a, const ahmp3_header *b)
{
    return a->lsf == b->lsf && a->mpeg25 == b->mpeg25 && a->sr_index == b->sr_index && a->layer == b->layer &&
           a->free_format == b->free_format;
}

/* Finds the size of the free-format frame at p, whose header does not give
   it: the same for every frame of the stream, so it is the distance to the
   next header like this one, confirmed by the header after that. Returns 1
   with h filled in, 0 when more data is needed, -1 when this is no frame. */
static int free_frame(const ahmp3_decoder *d, const unsigned char *p, size_t len, bool eof, bool trusted, ahmp3_header *h,
                      int *base_out)
{
    int unit = h->layer == 1 ? 4 : 1; /* Layer I frames come in 4-byte slots */
    int pad = h->padding * unit, base = 0;
    int least = 4 + 2 * h->crc + h->side_bytes + 1;
    if (trusted && d->free_base) {
        base = d->free_base;
    } else {
        for (int n = (least + unit - 1) / unit * unit;; n += unit) {
            if (n > MAX_FREE_FRAME) return -1;
            if ((size_t)n + 4 > len) return eof ? -1 : 0;
            ahmp3_header next, third;
            if (p[n] != 0xFF || !ahmp3_read_header(p + n, &next) || !same_stream(h, &next) || next.channels != h->channels) continue;
            if (n - pad < least) continue;
            size_t after = (size_t)n + (size_t)(n - pad + next.padding * unit);
            if (after + 4 > len) {
                if (!eof) return 0;
            } else if (!ahmp3_read_header(p + after, &third) || !same_stream(h, &third)) {
                continue;
            }
            base = n - pad;
            break;
        }
    }
    h->frame_bytes = base + pad;
    long long rate = ahmp3_sample_rates[h->sr_index];
    h->kbps = (int)((base * 8 * rate + h->samples * 500LL) / (h->samples * 1000LL));
    *base_out = base;
    return 1;
}

/* Layers I and II: false when the frame's CRC does not match. */
static bool decode_frame12(ahmp3_decoder *d, const unsigned char *f, const ahmp3_header *h, float *pcm)
{
    bool ok = ahmp3_layer12_decode(h, f, d->sub12);
    for (int ch = 0; ch < h->channels; ch++)
        synthesize(d, ch, (const float (*)[32])d->sub12[ch], h->layer == 1 ? 12 : 36, pcm + ch, h->channels);
    return ok;
}

/* Layer III: false when the frame's CRC does not match. */
static bool decode_frame(ahmp3_decoder *d, const unsigned char *f, const ahmp3_header *h, float *pcm)
{
    ahmp3_bits b = {f + 4 + 2 * h->crc, (size_t)h->side_bytes * 8, 0};
    int main_data_begin;
    granule gr[2][2];
    int nch = h->channels, ngr = h->lsf ? 1 : 2;
    /* The CRC covers the last two header bytes and the side information. A
       frame that fails it is played as silence: wrong side information can
       turn into a loud burst. Its main data still joins the reservoir. */
    bool crc_ok = true;
    if (h->crc) {
        unsigned crc = ahmp3_crc16(0xFFFF, f + 2, 16);
        crc = ahmp3_crc16(crc, f + 6, (size_t)h->side_bytes * 8);
        crc_ok = crc == ((unsigned)f[4] << 8 | f[5]);
    }
    bool ok = read_side_info(&b, h, &main_data_begin, gr) && crc_ok;

    /* Main data: this frame's bytes join the reservoir; ours start
       main_data_begin bytes before them. */
    int md_len = h->frame_bytes - 4 - 2 * h->crc - h->side_bytes;
    if (d->res_len + md_len > RESERVOIR) {
        int keep = RESERVOIR - md_len;
        if (keep > 511) keep = 511;
        if (keep > d->res_len) keep = d->res_len;
        memmove(d->res, d->res + d->res_len - keep, (size_t)keep);
        d->res_len = keep;
    }
    int start = d->res_len - main_data_begin;
    memcpy(d->res + d->res_len, f + 4 + 2 * h->crc + h->side_bytes, (size_t)md_len);
    d->res_len += md_len;
    if (start < 0) ok = false; /* the reservoir bytes it needs are gone (seek) */

    ahmp3_bits m = {d->res, (size_t)d->res_len * 8, ok ? (size_t)start * 8 : 0};
    static const granule silent = {0};
    for (int g = 0; g < ngr; g++) {
        float xr[2][576];
        int is[576];
        for (int ch = 0; ch < nch; ch++) {
            granule *G = &gr[g][ch];
            if (!ok) {
                *G = silent;
                G->long_end = 22;
                G->short_start = 13;
                memset(xr[ch], 0, sizeof(xr[ch]));
                continue;
            }
            size_t begin = m.pos, end = m.pos + (size_t)G->part23;
            if (h->lsf)
                read_scalefactors_lsf(&m, G, ch == 1 && (h->mode_ext & 1) && h->mode == 1);
            else
                read_scalefactors_mpeg1(&m, G, g ? &gr[0][ch] : NULL);
            if (m.pos > end) m.pos = end; /* broken: scale factors overran */
            read_huffman(d, &m, G, end, is);
            m.pos = begin + (size_t)G->part23;
            requantize(d, h, G, is, xr[ch]);
        }
        if (ok && nch == 2 && h->mode == 1) stereo(h, &gr[g][1], xr);
        for (int ch = 0; ch < nch; ch++) {
            float sub[18][32];
            reorder(h, &gr[g][ch], xr[ch]);
            alias_reduce(d, &gr[g][ch], xr[ch]);
            imdct(d, ch, &gr[g][ch], xr[ch], sub);
            synthesize(d, ch, (const float (*)[32])sub, 18, pcm + g * 576 * nch + ch, nch);
        }
    }
    return crc_ok;
}

size_t ahmp3_decode(ahmp3_decoder *d, const uint8_t *data, size_t len, bool eof,
                    float pcm[AHMP3_MAX_SAMPLES * 2], int *samples, ahmp3_frame *frame)
{
    *samples = 0;
    if (d->skip_pending) {
        size_t n = d->skip_pending < len ? d->skip_pending : len;
        d->skip_pending -= n;
        return n;
    }

    size_t pos = 0;
    ahmp3_header h;
    int free_base = 0;
    for (;; pos++) {
        if (pos + 4 > len) {
            /* No frame here. Keep the last bytes: a header may start there. */
            if (eof) return len;
            return len > 3 ? len - 3 : 0;
        }
        if (!d->synced) {
            size_t tag = ahmp3_id3v2_size(data + pos, len - pos);
            if (tag) {
                if (pos) return pos;
                if (tag <= len) return tag;
                d->skip_pending = tag - len;
                return len;
            }
        }
        if (data[pos] != 0xFF || !ahmp3_read_header(data + pos, &h)) continue;
        bool trusted = d->synced && same_stream(&h, &d->last);
        if (h.free_format) {
            int found = free_frame(d, data + pos, len - pos, eof, trusted, &h, &free_base);
            if (found == 0) return pos; /* wait for the frames that follow */
            if (found < 0) continue;
        }
        if (pos + (size_t)h.frame_bytes > len) {
            if (!eof) return pos; /* wait for the whole frame */
            if (!trusted) continue;
            return len;           /* a cut-off last frame */
        }
        if (!trusted && !h.free_format) {
            /* A false sync is unlikely to be followed by a matching header. */
            size_t next = pos + (size_t)h.frame_bytes;
            if (next + 4 <= len) {
                ahmp3_header n;
                if (!ahmp3_read_header(data + next, &n) || !same_stream(&h, &n)) continue;
            } else if (!eof) {
                return pos;
            }
        }
        break;
    }

    const unsigned char *f = data + pos;
    if (d->synced && !same_stream(&h, &d->last)) ahmp3_decoder_reset(d);
    d->synced = true;
    d->last = h;
    d->free_base = free_base;
    if (frame) ahmp3_fill_frame(&h, frame);

    if (!d->started) {
        d->started = true;
        ahmp3_info info;
        if (h.layer == 3 && read_info_frame(f, &h, &info)) {
            d->info = info;
            return pos + (size_t)h.frame_bytes;
        }
    }
    bool crc_ok = h.layer == 3 ? decode_frame(d, f, &h, pcm) : decode_frame12(d, f, &h, pcm);
    if (frame) frame->crc_error = !crc_ok;
    *samples = h.samples;
    return pos + (size_t)h.frame_bytes;
}
