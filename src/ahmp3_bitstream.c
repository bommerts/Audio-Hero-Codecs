#include "ahmp3_bitstream.h"

/* ---- bit writer --------------------------------------------------------------- */

typedef struct {
    unsigned char *p;
    int cap;  /* bytes */
    int bits; /* written so far */
} writer;

static void put(writer *w, unsigned v, int n)
{
    while (n-- > 0) {
        int byte = w->bits >> 3;
        if (byte < w->cap) {
            if (!(w->bits & 7)) w->p[byte] = 0;
            if ((v >> n) & 1) w->p[byte] |= (unsigned char)(0x80 >> (w->bits & 7));
        }
        w->bits++;
    }
}

/* ---- scale factors ------------------------------------------------------------ */

/* MPEG-2 scale factor lengths and table, as the decoder derives them. */
static int lsf_slen(int sfc, bool intensity_right, int slen[4], int *preflag)
{
    int sf = sfc, table, n1, n2, n3;
    *preflag = 0;
    if (intensity_right) {
        sf >>= 1;
        if (sf < 180) { table = 3; n1 = 6; n2 = 6; n3 = 0; }
        else if (sf < 244) { sf -= 180; table = 4; n1 = 4; n2 = 4; n3 = 0; }
        else { sf -= 244; table = 5; n1 = 3; n2 = 0; n3 = 0; }
    } else {
        if (sf < 400) { table = 0; n1 = 5; n2 = 4; n3 = 4; }
        else if (sf < 500) { sf -= 400; table = 1; n1 = 5; n2 = 4; n3 = 0; }
        else { sf -= 500; table = 2; n1 = 3; n2 = 0; n3 = 0; *preflag = 1; }
    }
    slen[3] = n3 ? sf % n3 : 0;
    if (n3) sf /= n3;
    slen[2] = n2 ? sf % n2 : 0;
    if (n2) sf /= n2;
    slen[1] = sf % n1;
    slen[0] = sf / n1;
    return table;
}

static int kind_of(const ahmp3_gr *G)
{
    return G->block_type == 2 ? (G->mixed ? 2 : 1) : 0;
}

static const int mpeg1_groups[5] = {0, 6, 11, 16, 21};

/* Writes (w != NULL) or counts the scale factor bits. */
static int part2(writer *w, const ahmp3_gr *G, int lsf, bool intensity_right, const ahmp3_gr *first)
{
    int bits = 0;
    if (lsf) {
        int slen[4], preflag;
        int table = lsf_slen(G->scalefac_compress, intensity_right, slen, &preflag);
        int j = 0;
        for (int k = 0; k < 4; k++)
            for (int i = 0; i < ahmp3_lsf_nsf[table][kind_of(G)][k]; i++) {
                if (w) put(w, G->sf[j], slen[k]);
                j++;
                bits += slen[k];
            }
        return bits;
    }
    int slen1 = ahmp3_slen[0][G->scalefac_compress], slen2 = ahmp3_slen[1][G->scalefac_compress];
    if (G->block_type == 2) {
        int n1 = G->mixed ? 17 : 18, j = 0;
        for (int i = 0; i < n1; i++, j++) {
            if (w) put(w, G->sf[j], slen1);
            bits += slen1;
        }
        for (int i = 0; i < 18; i++, j++) {
            if (w) put(w, G->sf[j], slen2);
            bits += slen2;
        }
        return bits;
    }
    for (int k = 0; k < 4; k++) {
        if (first && (G->scfsi & (8 >> k))) continue;
        int slen = k < 2 ? slen1 : slen2;
        for (int i = mpeg1_groups[k]; i < mpeg1_groups[k + 1]; i++) {
            if (w) put(w, G->sf[i], slen);
            bits += slen;
        }
    }
    return bits;
}

int ahmp3_part2_bits(const ahmp3_gr *G, int lsf, bool intensity_right, const ahmp3_gr *first)
{
    return part2(NULL, G, lsf, intensity_right, first);
}

int ahmp3_sf_limit(const ahmp3_gr *G, int lsf, int n)
{
    if (lsf) {
        /* Table 0 lengths: 4, 4, 3, 3 bits. */
        int at = 0;
        for (int k = 0; k < 4; k++) {
            at += ahmp3_lsf_nsf[0][kind_of(G)][k];
            if (n < at) return k < 2 ? 15 : 7;
        }
        return 0;
    }
    if (n < G->long_end) return n < 11 ? 15 : 7;
    int b = G->short_start + (n - G->long_end) / 3;
    return b < 6 ? 15 : 7;
}

static int bits_for(int v)
{
    int n = 0;
    while (v >> n) n++;
    return n;
}

bool ahmp3_choose_scalefac_compress(ahmp3_gr *G, int lsf)
{
    int count = G->long_end + (13 - G->short_start) * 3;
    if (lsf) {
        int kind = kind_of(G), need[4] = {0, 0, 0, 0}, j = 0;
        for (int k = 0; k < 4; k++)
            for (int i = 0; i < ahmp3_lsf_nsf[0][kind][k]; i++, j++) {
                int b = bits_for(G->sf[j]);
                if (b > need[k]) need[k] = b;
            }
        if (need[0] > 4 || need[1] > 4 || need[2] > 3 || need[3] > 3) return false;
        G->scalefac_compress = ((need[0] * 5 + need[1]) << 4) + (need[2] << 2) + need[3];
        G->preflag = 0;
        return true;
    }
    int need1 = 0, need2 = 0;
    for (int n = 0; n < count; n++) {
        int b = bits_for(G->sf[n]);
        bool first_part = n < G->long_end ? n < 11 : G->short_start + (n - G->long_end) / 3 < 6;
        if (first_part && b > need1) need1 = b;
        if (!first_part && b > need2) need2 = b;
    }
    int best = -1, best_bits = 1 << 30;
    for (int c = 0; c < 16; c++) {
        if (ahmp3_slen[0][c] < need1 || ahmp3_slen[1][c] < need2) continue;
        int bits = G->block_type == 2 ? (G->mixed ? 17 : 18) * ahmp3_slen[0][c] + 18 * ahmp3_slen[1][c]
                                      : 11 * ahmp3_slen[0][c] + 10 * ahmp3_slen[1][c];
        if (bits < best_bits) {
            best_bits = bits;
            best = c;
        }
    }
    if (best < 0) return false;
    G->scalefac_compress = best;
    return true;
}

/* ---- Huffman ------------------------------------------------------------------- */

static int region1_start(const ahmp3_gr *G, int sr)
{
    if (G->window_switching) return G->block_type == 2 ? (sr == 8 ? 72 : 36) : ahmp3_sfb_long[sr][8];
    return ahmp3_sfb_long[sr][G->region0_count + 1];
}

static int region2_start(const ahmp3_gr *G, int sr)
{
    if (G->window_switching) return 576;
    int l = G->region0_count + G->region1_count + 2;
    return ahmp3_sfb_long[sr][l > 22 ? 22 : l];
}

static void region_bounds(const ahmp3_gr *G, int sr, int b[3])
{
    int big = G->big_values * 2;
    int r1 = region1_start(G, sr), r2 = region2_start(G, sr);
    b[0] = r1 < big ? r1 : big;
    b[1] = r2 < big ? r2 : big;
    if (b[1] < b[0]) b[1] = b[0];
    b[2] = big;
}

#define INVALID (1 << 24)

static int pair_bits(int t, int x, int y)
{
    const ahmp3_huff *h = &ahmp3_huff_tables[t];
    if (x < 0) x = -x;
    if (y < 0) y = -y;
    int bits = (x != 0) + (y != 0);
    if (h->linbits) {
        if (x >= 15) { if (x - 15 >= 1 << h->linbits) return INVALID; bits += h->linbits; x = 15; }
        if (y >= 15) { if (y - 15 >= 1 << h->linbits) return INVALID; bits += h->linbits; y = 15; }
    } else if (x >= h->size || y >= h->size) {
        return INVALID;
    }
    return bits + h->lens[x * h->size + y];
}

static int range_bits(const int *ix, int t, int start, int end)
{
    if (t == 0) {
        for (int j = start; j < end; j++)
            if (ix[j]) return INVALID;
        return 0;
    }
    if (!ahmp3_huff_tables[t].size) return INVALID;
    int bits = 0;
    for (int j = start; j < end; j += 2) {
        bits += pair_bits(t, ix[j], ix[j + 1]);
        if (bits >= INVALID) return INVALID;
    }
    return bits;
}

/* The cheapest table for lines [start, end). */
static int choose_table(const int *ix, int start, int end, int *bits)
{
    int max = 0;
    for (int j = start; j < end; j++) {
        int v = ix[j] < 0 ? -ix[j] : ix[j];
        if (v > max) max = v;
    }
    *bits = 0;
    if (max == 0) return 0;
    static const int classes[16][3] = {
        {0}, {1, 0, 0}, {2, 3, 0}, {5, 6, 0}, {7, 8, 9}, {7, 8, 9}, {10, 11, 12}, {10, 11, 12},
        {13, 15, 0}, {13, 15, 0}, {13, 15, 0}, {13, 15, 0}, {13, 15, 0}, {13, 15, 0}, {13, 15, 0}, {13, 15, 0},
    };
    int best = -1, best_bits = INVALID;
    if (max < 16) {
        for (int i = 0; i < 3 && classes[max][i]; i++) {
            int b = range_bits(ix, classes[max][i], start, end);
            if (b < best_bits) { best_bits = b; best = classes[max][i]; }
        }
    }
    if (max >= 15) {
        for (int base = 16; base <= 24; base += 8) {
            for (int t = base; t < base + 8; t++) {
                if (max - 15 < 1 << ahmp3_huff_tables[t].linbits) {
                    int b = range_bits(ix, t, start, end);
                    if (b < best_bits) { best_bits = b; best = t; }
                    break;
                }
            }
        }
    }
    *bits = best_bits;
    return best;
}

static int count1_bits(const int *ix, int start, int end, int table)
{
    int bits = 0;
    for (int j = start; j < end; j += 4) {
        int idx = 0, signs = 0;
        for (int i = 0; i < 4; i++)
            if (ix[j + i]) { idx |= 8 >> i; signs++; }
        bits += signs + (table ? 4 : ahmp3_count1_lens[0][idx]);
    }
    return bits;
}

int ahmp3_huffman_bits(const ahmp3_gr *G, int sr)
{
    int b[3], start = 0, bits = 0;
    region_bounds(G, sr, b);
    for (int r = 0; r < 3; r++) {
        bits += range_bits(G->ix, G->table[r], start, b[r]);
        start = b[r];
    }
    return bits + count1_bits(G->ix, G->big_values * 2, G->count1_end, G->count1_table);
}

static const unsigned char subdivision[23][2] = {
    {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 1}, {1, 1}, {1, 1}, {1, 2}, {2, 2}, {2, 3}, {2, 3},
    {3, 4}, {3, 4}, {3, 4}, {4, 5}, {4, 5}, {4, 6}, {5, 6}, {5, 6}, {5, 7}, {6, 7}, {6, 7},
};

int ahmp3_huffman_choose(ahmp3_gr *G, int sr, bool best_split)
{
    const int *ix = G->ix;
    int end = 576;
    while (end > 0 && !ix[end - 1] && !ix[end - 2]) end -= 2;
    int c1 = end;
    while (c1 >= 4) {
        bool small = true;
        for (int i = 1; i <= 4; i++)
            if (ix[c1 - i] > 1 || ix[c1 - i] < -1) small = false;
        if (!small) break;
        c1 -= 4;
    }
    G->big_values = c1 / 2;
    G->count1_end = end;
    int a = count1_bits(ix, c1, end, 0), b = count1_bits(ix, c1, end, 1);
    G->count1_table = b < a;
    int bits = b < a ? b : a;

    G->table[0] = G->table[1] = G->table[2] = 0;
    if (G->window_switching) {
        int bound[3], t0, t1;
        G->region0_count = G->region1_count = 0;
        region_bounds(G, sr, bound);
        G->table[0] = choose_table(ix, 0, bound[0], &t0);
        G->table[1] = choose_table(ix, bound[0], bound[2], &t1);
        return bits + t0 + t1;
    }

    /* The usual split for this many bands; the full search below can only
       improve on it. */
    int big = c1;
    int bands = 0;
    while (bands < 22 && ahmp3_sfb_long[sr][bands] < big) bands++;
    G->region0_count = subdivision[bands][0];
    G->region1_count = subdivision[bands][1];
    int bound[3], start = 0, sum = 0;
    region_bounds(G, sr, bound);
    for (int r = 0; r < 3; r++) {
        int rb;
        G->table[r] = choose_table(ix, start, bound[r], &rb);
        sum += rb;
        start = bound[r];
    }
    if (!best_split) return bits + sum;

    int best = sum, best_r0 = G->region0_count, best_r1 = G->region1_count;
    int best_t[3] = {G->table[0], G->table[1], G->table[2]};
    for (int r0 = 0; r0 < 16; r0++) {
        for (int r1 = 0; r1 < 8; r1++) {
            G->region0_count = r0;
            G->region1_count = r1;
            int tb[3], from = 0, total = 0, t[3];
            region_bounds(G, sr, tb);
            for (int r = 0; r < 3 && total < best; r++) {
                int rb;
                t[r] = choose_table(ix, from, tb[r], &rb);
                total += rb;
                from = tb[r];
            }
            if (total < best) {
                best = total;
                best_r0 = r0;
                best_r1 = r1;
                memcpy(best_t, t, sizeof(t));
            }
            if (ahmp3_sfb_long[sr][r0 + r1 + 2 > 22 ? 22 : r0 + r1 + 2] >= big) break;
        }
        if (ahmp3_sfb_long[sr][r0 + 1] >= big) break;
    }
    G->region0_count = best_r0;
    G->region1_count = best_r1;
    memcpy(G->table, best_t, sizeof(best_t));
    return bits + best;
}

static void write_huffman(writer *w, const ahmp3_gr *G, int sr)
{
    int b[3], start = 0;
    region_bounds(G, sr, b);
    for (int r = 0; r < 3; r++) {
        const ahmp3_huff *h = &ahmp3_huff_tables[G->table[r]];
        for (int j = start; j < b[r] && h->size; j += 2) {
            int x = G->ix[j] < 0 ? -G->ix[j] : G->ix[j], y = G->ix[j + 1] < 0 ? -G->ix[j + 1] : G->ix[j + 1];
            int xi = x > 15 ? 15 : x, yi = y > 15 ? 15 : y;
            if (!h->linbits) { xi = x; yi = y; }
            put(w, h->codes[xi * h->size + yi], h->lens[xi * h->size + yi]);
            if (h->linbits && x >= 15) put(w, (unsigned)(x - 15), h->linbits);
            if (x) put(w, G->ix[j] < 0, 1);
            if (h->linbits && y >= 15) put(w, (unsigned)(y - 15), h->linbits);
            if (y) put(w, G->ix[j + 1] < 0, 1);
        }
        start = b[r];
    }
    for (int j = G->big_values * 2; j < G->count1_end; j += 4) {
        int idx = 0;
        for (int i = 0; i < 4; i++)
            if (G->ix[j + i]) idx |= 8 >> i;
        if (G->count1_table) put(w, (unsigned)(15 - idx), 4);
        else put(w, ahmp3_count1_codes[0][idx], ahmp3_count1_lens[0][idx]);
        for (int i = 0; i < 4; i++)
            if (G->ix[j + i]) put(w, G->ix[j + i] < 0, 1);
    }
}

/* ---- frames --------------------------------------------------------------------- */

bool ahmp3_intensity_right(const ahmp3_fr *f, int ch)
{
    return f->lsf && ch == 1 && f->mode == 1 && (f->mode_ext & 1);
}

int ahmp3_write_header(const ahmp3_fr *f, unsigned char *out)
{
    int version = f->lsf ? (f->sr_index >= 6 ? 0 : 2) : 3;
    writer w = {out, 40, 0};
    put(&w, 0x7FF, 11);
    put(&w, (unsigned)version, 2);
    put(&w, 1, 2); /* Layer III */
    put(&w, f->crc ? 0 : 1, 1);
    put(&w, (unsigned)f->bitrate_index, 4);
    put(&w, (unsigned)(f->sr_index % 3), 2);
    put(&w, (unsigned)f->padding, 1);
    put(&w, 0, 1);
    put(&w, (unsigned)f->mode, 2);
    put(&w, (unsigned)f->mode_ext, 2);
    put(&w, 0, 1); /* copyright */
    put(&w, 1, 1); /* original */
    put(&w, 0, 2); /* emphasis */
    if (f->crc) put(&w, 0, 16); /* filled in below */

    int nch = f->channels;
    if (f->lsf) {
        put(&w, (unsigned)f->main_data_begin, 8);
        put(&w, 0, nch == 1 ? 1 : 2);
    } else {
        put(&w, (unsigned)f->main_data_begin, 9);
        put(&w, 0, nch == 1 ? 5 : 3);
        for (int ch = 0; ch < nch; ch++) put(&w, (unsigned)f->gr[1][ch].scfsi, 4);
    }
    for (int g = 0; g < f->granules; g++) {
        for (int ch = 0; ch < nch; ch++) {
            const ahmp3_gr *G = &f->gr[g][ch];
            put(&w, (unsigned)G->part23, 12);
            put(&w, (unsigned)G->big_values, 9);
            put(&w, (unsigned)G->global_gain, 8);
            put(&w, (unsigned)G->scalefac_compress, f->lsf ? 9 : 4);
            put(&w, (unsigned)G->window_switching, 1);
            if (G->window_switching) {
                put(&w, (unsigned)G->block_type, 2);
                put(&w, (unsigned)G->mixed, 1);
                for (int i = 0; i < 2; i++) put(&w, (unsigned)G->table[i], 5);
                for (int i = 0; i < 3; i++) put(&w, (unsigned)G->subblock_gain[i], 3);
            } else {
                for (int i = 0; i < 3; i++) put(&w, (unsigned)G->table[i], 5);
                put(&w, (unsigned)G->region0_count, 4);
                put(&w, (unsigned)G->region1_count, 3);
            }
            if (!f->lsf) put(&w, (unsigned)G->preflag, 1);
            put(&w, (unsigned)G->scalefac_scale, 1);
            put(&w, (unsigned)G->count1_table, 1);
        }
    }
    if (f->crc) {
        unsigned crc = ahmp3_crc16(0xFFFF, out + 2, 16);
        crc = ahmp3_crc16(crc, out + 6, (size_t)w.bits - 48);
        out[4] = (unsigned char)(crc >> 8);
        out[5] = (unsigned char)crc;
    }
    return w.bits / 8;
}

int ahmp3_write_main(const ahmp3_fr *f, unsigned char *out, int cap)
{
    writer w = {out, cap, 0};
    for (int g = 0; g < f->granules; g++) {
        for (int ch = 0; ch < f->channels; ch++) {
            const ahmp3_gr *G = &f->gr[g][ch];
            int start = w.bits;
            part2(&w, G, f->lsf, ahmp3_intensity_right(f, ch), !f->lsf && g ? &f->gr[0][ch] : NULL);
            write_huffman(&w, G, f->sr_index);
            /* Pad to the declared length (only a test frame declares more). */
            while (w.bits - start < G->part23) put(&w, 0, 1);
        }
    }
    return w.bits;
}
