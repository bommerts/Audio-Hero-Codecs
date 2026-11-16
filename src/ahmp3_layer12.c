/* Layers I and II of ISO/IEC 11172-3 and 13818-3: bit allocation, scale
   factors and subband samples. The subband samples then go through the same
   synthesis filter bank as Layer III. */
#include "ahmp3_internal.h"

/* ISO 11172-3 table B.4: levels, bits per code word, three samples in one. */
const ahmp3_l2_class ahmp3_l2_classes[17] = {
    {3, 5, 1},      {5, 7, 1},      {7, 3, 0},      {9, 10, 1},     {15, 4, 0},     {31, 5, 0},
    {63, 6, 0},     {127, 7, 0},    {255, 8, 0},    {511, 9, 0},    {1023, 10, 0},  {2047, 11, 0},
    {4095, 12, 0},  {8191, 13, 0},  {16383, 14, 0}, {32767, 15, 0}, {65535, 16, 0},
};

/* The rows the allocation tables are built from: the class each allocation
   value 1, 2, 3 ... selects. */
static const unsigned char row_a[15] = {0, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static const unsigned char row_b[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16};
static const unsigned char row_c[7] = {0, 1, 2, 3, 4, 5, 16};
static const unsigned char row_d[3] = {0, 1, 16};
static const unsigned char row_e[15] = {0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
static const unsigned char row_f[7] = {0, 1, 3, 4, 5, 6, 7};
static const unsigned char row_g[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
static const unsigned char row_i[3] = {0, 1, 3};

static void rows_put(ahmp3_l2_table *t, const unsigned char *row, int bits, int count)
{
    for (int i = 0; i < count; i++) {
        t->bits[t->sblimit] = (unsigned char)bits;
        t->rows[t->sblimit++] = row;
    }
}

/* ISO 11172-3 tables B.2a to B.2d by sample rate and bit rate per channel,
   and the one table of 13818-3 (B.1) for the low sample rates. */
void ahmp3_l2_select(int lsf, int sample_rate, int kbps, ahmp3_l2_table *t)
{
    t->sblimit = 0;
    if (lsf) {
        rows_put(t, row_g, 4, 4);
        rows_put(t, row_f, 3, 7);
        rows_put(t, row_i, 2, 19);
    } else if ((sample_rate == 48000 && kbps >= 56) || (kbps >= 56 && kbps <= 80)) {
        rows_put(t, row_a, 4, 3);
        rows_put(t, row_b, 4, 8);
        rows_put(t, row_c, 3, 12);
        rows_put(t, row_d, 2, 4);
    } else if (sample_rate != 48000 && kbps >= 96) {
        rows_put(t, row_a, 4, 3);
        rows_put(t, row_b, 4, 8);
        rows_put(t, row_c, 3, 12);
        rows_put(t, row_d, 2, 7);
    } else {
        rows_put(t, row_e, 4, 2);
        rows_put(t, row_f, 3, sample_rate == 32000 ? 10 : 6);
    }
}

bool ahmp3_layer12_decode(const ahmp3_header *h, const unsigned char *frame, float out[2][36][32])
{
    /* Scale factors: 2, 2^(2/3), 2^(1/3), 1, ... (table B.1). */
    static float scale[64];
    if (scale[0] == 0)
        for (int i = 63; i >= 0; i--) scale[i] = (float)pow(2.0, 1 - i / 3.0);

    memset(out, 0, 2 * 36 * 32 * sizeof(float));
    const unsigned char *data = frame + 4 + 2 * h->crc;
    ahmp3_bits b = {data, (size_t)(h->frame_bytes - 4 - 2 * h->crc) * 8, 0};
    int nch = h->channels;
    ahmp3_l2_table table;
    int sblimit = 32;
    if (h->layer == 2) {
        ahmp3_l2_select(h->lsf, ahmp3_sample_rates[h->sr_index], h->kbps / nch, &table);
        sblimit = table.sblimit;
    }
    /* In joint stereo the subbands from `bound` up are coded once for both
       channels (intensity stereo). */
    int bound = h->mode == 1 ? 4 + 4 * h->mode_ext : sblimit;
    if (bound > sblimit) bound = sblimit;

    unsigned char alloc[2][32] = {{0}}, scfsi[2][32] = {{0}}, sf[2][32][3] = {{{0}}};
    for (int sb = 0; sb < sblimit; sb++) {
        int bits = h->layer == 1 ? 4 : table.bits[sb];
        for (int ch = 0; ch < (sb < bound ? nch : 1); ch++) alloc[ch][sb] = (unsigned char)ahmp3_get_bits(&b, bits);
        if (sb >= bound) alloc[1][sb] = alloc[0][sb];
    }
    if (h->layer == 2)
        for (int sb = 0; sb < sblimit; sb++)
            for (int ch = 0; ch < nch; ch++)
                if (alloc[ch][sb]) scfsi[ch][sb] = (unsigned char)ahmp3_get_bits(&b, 2);

    /* The CRC covers the last two header bytes and everything read so far. */
    if (h->crc) {
        unsigned crc = ahmp3_crc16(0xFFFF, frame + 2, 16);
        crc = ahmp3_crc16(crc, data, b.pos);
        if (crc != ((unsigned)frame[4] << 8 | frame[5])) return false;
    }

    for (int sb = 0; sb < sblimit; sb++) {
        for (int ch = 0; ch < nch; ch++) {
            if (!alloc[ch][sb]) continue;
            unsigned char *s = sf[ch][sb];
            if (h->layer == 1) {
                s[0] = (unsigned char)ahmp3_get_bits(&b, 6);
                continue;
            }
            /* How many of the frame's three parts share a scale factor. */
            switch (scfsi[ch][sb]) {
            case 0:
                s[0] = (unsigned char)ahmp3_get_bits(&b, 6);
                s[1] = (unsigned char)ahmp3_get_bits(&b, 6);
                s[2] = (unsigned char)ahmp3_get_bits(&b, 6);
                break;
            case 1:
                s[0] = s[1] = (unsigned char)ahmp3_get_bits(&b, 6);
                s[2] = (unsigned char)ahmp3_get_bits(&b, 6);
                break;
            case 2:
                s[0] = s[1] = s[2] = (unsigned char)ahmp3_get_bits(&b, 6);
                break;
            default:
                s[0] = (unsigned char)ahmp3_get_bits(&b, 6);
                s[1] = s[2] = (unsigned char)ahmp3_get_bits(&b, 6);
                break;
            }
        }
    }

    /* Samples. A code c of a quantizer with L levels stands for
       (2c + 1 - L) / L, which is the standard's formula in one step. */
    if (h->layer == 1) {
        for (int slot = 0; slot < 12; slot++) {
            for (int sb = 0; sb < 32; sb++) {
                for (int ch = 0; ch < (sb < bound ? nch : 1); ch++) {
                    int n = alloc[ch][sb];
                    if (!n) continue;
                    int levels = (2 << n) - 1;
                    float v = (float)(2 * (int)ahmp3_get_bits(&b, n + 1) + 1 - levels) / (float)levels;
                    out[ch][slot][sb] = v * scale[sf[ch][sb][0]];
                    if (sb >= bound && nch == 2) out[1][slot][sb] = v * scale[sf[1][sb][0]];
                }
            }
        }
        return true;
    }
    for (int group = 0; group < 12; group++) {
        int part = group / 4;
        for (int sb = 0; sb < sblimit; sb++) {
            for (int ch = 0; ch < (sb < bound ? nch : 1); ch++) {
                int a = alloc[ch][sb];
                if (!a) continue;
                const ahmp3_l2_class *c = &ahmp3_l2_classes[table.rows[sb][a - 1]];
                int levels = c->levels, code[3];
                if (c->grouped) {
                    int word = (int)ahmp3_get_bits(&b, c->bits);
                    for (int k = 0; k < 3; k++) {
                        code[k] = word % levels;
                        word /= levels;
                    }
                } else {
                    for (int k = 0; k < 3; k++) code[k] = (int)ahmp3_get_bits(&b, c->bits);
                }
                for (int k = 0; k < 3; k++) {
                    float v = (float)(2 * code[k] + 1 - levels) / (float)levels;
                    out[ch][3 * group + k][sb] = v * scale[sf[ch][sb][part]];
                    if (sb >= bound && nch == 2) out[1][3 * group + k][sb] = v * scale[sf[1][sb][part]];
                }
            }
        }
    }
    return true;
}
