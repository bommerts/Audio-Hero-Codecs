/* Tests for the Apple Lossless decoder. A small ALAC writer here codes
   samples the test knows, choosing at random among every part of the
   format, and wraps the packets in a small M4A file. The decoder must give
   back exactly those samples, and ffmpeg must decode the same file to the
   same numbers. Streams that use what ffmpeg reads differently from Apple's
   published decoder (see the README) are checked against their samples
   alone. ffmpeg's own ALAC encoder supplies files too. */
#include "../include/ahalac.h"
#include "../include/ahm4a.h"
#include "../include/ahreader.h"
#include "../src/ahalac_internal.h"
#include "common.h"

/* ---- writing bits --------------------------------------------------------------- */

typedef struct {
    bytes b;
    unsigned acc;
    int n;
} bitw;

static void put(bitw *w, uint64_t v, int n)
{
    for (int i = n - 1; i >= 0; i--) {
        w->acc = w->acc << 1 | (unsigned)((v >> i) & 1);
        if (++w->n == 8) {
            unsigned char c = (unsigned char)w->acc;
            bytes_put(&w->b, &c, 1);
            w->n = 0;
            w->acc = 0;
        }
    }
}

static size_t bit_count(const bitw *w) { return w->b.len * 8 + (size_t)w->n; }

/* Appends the bits of `from` to `w`. */
static void put_bits_of(bitw *w, const bitw *from)
{
    for (size_t i = 0; i < from->b.len; i++) put(w, from->b.p[i], 8);
    put(w, from->acc, from->n);
}

static int32_t sext(uint32_t v, int n)
{
    uint32_t m = n >= 32 ? 0xFFFFFFFFu : (1u << n) - 1, sign = 1u << (n - 1);
    v &= m;
    return (int32_t)(v & sign ? v | ~m : v);
}

static int sgn(int32_t v) { return (v > 0) - (v < 0); }

/* ---- the writer's side of the predictor ----------------------------------------- */

/* The residuals that make the decoder's adaptive predictor give back x. */
static void lpc_residuals(const int32_t *x, int32_t *r, uint32_t n, const int16_t *start, int order, int q, int width)
{
    int16_t c[32];
    memcpy(c, start, sizeof(c));
    if (!n) return;
    r[0] = x[0];
    if (order == 0) {
        memcpy(r, x, n * sizeof(int32_t));
        return;
    }
    uint32_t warm = order == 31 ? n : (uint32_t)order + 1;
    for (uint32_t j = 1; j < n && j < warm; j++) r[j] = sext((uint32_t)x[j] - (uint32_t)x[j - 1], width);
    for (uint32_t j = warm; j < n; j++) {
        int32_t top = x[j - (uint32_t)order - 1];
        uint32_t sum = 0;
        for (int k = 0; k < order; k++) sum += (uint32_t)(int32_t)c[k] * ((uint32_t)x[j - 1 - (uint32_t)k] - (uint32_t)top);
        int32_t guess = (int32_t)(((int64_t)(int32_t)sum + ((int64_t)1 << (q - 1))) >> q);
        int32_t e = sext((uint32_t)x[j] - (uint32_t)top - (uint32_t)guess, width);
        r[j] = e;
        if (e == 0 || e == INT32_MIN) continue;
        /* Apple's adaptation, written as Apple's encoder has it. */
        int32_t left = e;
        for (int k = order - 1; k >= 0; k--) {
            int32_t dd = (int32_t)((uint32_t)top - (uint32_t)x[j - 1 - (uint32_t)k]);
            int s = sgn(dd);
            if (e > 0) {
                c[k] = (int16_t)(c[k] - s);
                left = (int32_t)((uint32_t)left - (uint32_t)(order - k) * (uint32_t)((int32_t)((uint32_t)s * (uint32_t)dd) >> q));
                if (left <= 0) break;
            } else {
                c[k] = (int16_t)(c[k] + s);
                left = (int32_t)((uint32_t)left - (uint32_t)(order - k) * (uint32_t)((int32_t)(0u - (uint32_t)s * (uint32_t)dd) >> q));
                if (left >= 0) break;
            }
        }
    }
}

/* ---- the writer's side of the Golomb code ---------------------------------------- */

static int g_escapes, g_run_escapes, g_runs;

static void put_golomb(bitw *w, uint32_t n, uint32_t m, int k, int width, bool escape)
{
    uint32_t q = n / m;
    if (escape || q >= AHALAC_MAX_PREFIX) {
        put(w, (1u << AHALAC_MAX_PREFIX) - 1, AHALAC_MAX_PREFIX);
        put(w, n, width);
        return;
    }
    uint32_t rem = n - q * m;
    put(w, ((1u << q) - 1) << 1, (int)q + 1);
    if (rem) put(w, rem + 1, k);
    else put(w, 0, k - 1);
}

/* Codes the residuals of one channel. *differs is set when ffmpeg would
   read them otherwise than Apple's decoder does. */
static void put_residuals(bitw *w, const int32_t *r, uint32_t n, const ahalac_setup *s, uint32_t pb, int width, bool *differs)
{
    uint32_t mb = s->mb, zmode = 0;
    for (uint32_t c = 0; c < n;) {
        int k = ahalac_value_k(mb, s->kb);
        uint32_t v = (uint32_t)r[c] << 1 ^ (uint32_t)(r[c] >> 31); /* 0, -1, 1, -2 ... as 0, 1, 2, 3 ... */
        uint32_t x = v - zmode;
        bool escape = rnd_in(0, 60) == 0;
        if (escape || x / ((1u << k) - 1) >= AHALAC_MAX_PREFIX) g_escapes++;
        put_golomb(w, x, (1u << k) - 1, k, width, escape);
        c++;
        if (zmode && x == AHALAC_MEAN_CLAMP) *differs = true; /* ffmpeg clamps the mean on x + 1 */
        mb = ahalac_mean(mb, pb, x, zmode);
        zmode = 0;
        if (ahalac_run_follows(mb) && c < n) {
            int kz = ahalac_run_k(mb);
            if (!mb || kz > s->kb) *differs = true; /* ffmpeg: 7 for a mean of 0, and the parameter capped */
            uint32_t run = 0;
            while (c + run < n && run < AHALAC_RUN_LONG && r[c + run] == 0) run++;
            if (run == AHALAC_RUN_LONG) *differs = true; /* ffmpeg codes the next value one smaller still */
            bool escape_run = rnd_in(0, 20) == 0;
            if (escape_run || run / ahalac_run_m(kz, s->kb) >= AHALAC_MAX_PREFIX) g_run_escapes++;
            if (run) g_runs++;
            put_golomb(w, run, ahalac_run_m(kz, s->kb), kz, AHALAC_RUN_BITS, escape_run);
            c += run;
            zmode = run < AHALAC_RUN_LONG;
            mb = 0;
        }
    }
}

/* ---- elements and packets ------------------------------------------------------- */

typedef struct {
    int rate, channels, bits;
    uint32_t frame;
    bool apple; /* may use what ffmpeg reads differently from Apple */
} alac_kind;

/* What the writer used, to check that every part was. */
static int g_verbatim, g_compressed, g_apart[3], g_mixed, g_modes[16], g_order0, g_order31, g_orders, g_partial, g_lfe, g_dse, g_fil, g_odd_layout;

static void choose_coefs(int16_t *c, int order, int q)
{
    memset(c, 0, 32 * sizeof(int16_t));
    if (order == 0 || order == 31) {
        for (int i = 0; i < order; i++) c[i] = (int16_t)rnd_in(-32768, 32767);
        return;
    }
    switch (rnd_in(0, 4)) {
    case 0: /* anything at all */
        for (int i = 0; i < order; i++) c[i] = (int16_t)rnd_in(-(1 << rnd_in(0, 15)), (1 << rnd_in(0, 15)) - 1);
        break;
    case 1: /* the last sample */
        c[0] = (int16_t)(1 << q);
        break;
    case 2: /* a straight line through the last two */
        if (q <= 13 && order >= 2) {
            c[0] = (int16_t)(2 << q);
            c[1] = (int16_t)-(1 << q);
        } else {
            c[0] = (int16_t)(1 << q);
        }
        break;
    default: /* small, as a fitted predictor's are */
        for (int i = 0; i < order; i++) c[i] = (int16_t)rnd_in(-(1 << q) / (i + 1), (1 << q) / (i + 1));
        break;
    }
}

/* One element of `count` channels, the samples of channel c at src[c][i *
   stride]. */
static void put_element(bitw *w, const alac_kind *k, const ahalac_setup *s, int type, const int32_t *src[2], size_t stride, uint32_t n)
{
    int count = type == AHALAC_CPE ? 2 : 1, depth = k->bits;
    bool partial = n < s->frame_length || rnd_in(0, 7) == 0;
    bool as_is = rnd_in(0, 9) == 0;
    int32_t *x[2], *r[2], *y;
    x[0] = malloc(n * sizeof(int32_t));
    x[1] = malloc(n * sizeof(int32_t));
    r[0] = malloc(n * sizeof(int32_t));
    r[1] = malloc(n * sizeof(int32_t));
    y = malloc(n * sizeof(int32_t));
    bitw body = {{0}, 0, 0};
    for (int attempt = 0; attempt < 2; attempt++) {
        body.b.len = 0;
        body.acc = 0;
        body.n = 0;
        int apart = rnd_in(0, 2);
        while (apart && 8 * apart >= depth + count - 1) apart--;
        bool differs = false;
        if (!as_is) {
            if (depth == 32 && count == 2 && !apart) apart = 1; /* 33 bits will not do */
            int width = depth - 8 * apart + count - 1;
            /* The low bytes go apart; the rest is mixed, predicted and coded. */
            int mix_shift = 0, mix_weight = 0;
            if (count == 2 && rnd_in(0, 2)) {
                mix_shift = rnd_in(0, 3) ? 2 : rnd_in(0, 31);
                mix_weight = rnd_in(0, 3) ? rnd_in(1, 4) : rnd_in(1, k->apple ? 255 : 127);
            }
            for (uint32_t i = 0; i < n; i++) {
                uint32_t a = (uint32_t)(src[0][i * stride] >> (8 * apart)), b = count == 2 ? (uint32_t)(src[1][i * stride] >> (8 * apart)) : 0;
                if (count == 2 && mix_weight) {
                    /* u = r + w (l - r) / 2^shift, v = l - r; Apple's int8 weight. */
                    uint32_t v = a - b;
                    int32_t wt = (int8_t)mix_weight;
                    uint32_t u = b + (uint32_t)((int32_t)((uint32_t)wt * v) >> mix_shift);
                    x[0][i] = sext(u, width);
                    x[1][i] = sext(v, width);
                } else {
                    x[0][i] = sext(a, width);
                    if (count == 2) x[1][i] = sext(b, width);
                }
            }
            put(&body, (unsigned)mix_shift, 8);
            put(&body, (unsigned)mix_weight, 8);
            if (mix_weight) g_mixed++;
            int mode[2], q[2], factor[2], order[2];
            int16_t coef[2][32];
            for (int c = 0; c < count; c++) {
                mode[c] = rnd_in(0, 2) ? 0 : k->apple ? rnd_in(1, 15) : 15;
                q[c] = rnd_in(0, 2) ? 9 : rnd_in(1, 15);
                factor[c] = rnd_in(0, 3) ? 4 : rnd_in(k->apple ? 0 : 1, 7);
                int pick = rnd_in(0, 9);
                order[c] = pick == 0 ? 0 : pick == 1 ? 31 : pick < 5 ? rnd_in(1, 8) : rnd_in(1, 30);
                if ((uint32_t)order[c] >= s->frame_length) order[c] = (int)s->frame_length - 1;
                choose_coefs(coef[c], order[c], q[c]);
                put(&body, (unsigned)mode[c], 4);
                put(&body, (unsigned)q[c], 4);
                put(&body, (unsigned)factor[c], 3);
                put(&body, (unsigned)order[c], 5);
                for (int i = 0; i < order[c]; i++) put(&body, (uint16_t)coef[c][i], 16);
                if (mode[c]) differs |= mode[c] != 15;
            }
            for (uint32_t i = 0; i < n && apart; i++)
                for (int c = 0; c < count; c++) put(&body, (uint32_t)src[c][i * stride] & ((1u << (8 * apart)) - 1), 8 * apart);
            for (int c = 0; c < count; c++) {
                lpc_residuals(x[c], mode[c] ? y : r[c], n, coef[c], order[c], q[c], width);
                if (mode[c]) lpc_residuals(y, r[c], n, coef[c], 31, 1, width);
                put_residuals(&body, r[c], n, s, (uint32_t)s->pb * (uint32_t)factor[c] / 4, width, &differs);
            }
            /* Too big, or read otherwise by ffmpeg: sent as it is instead. */
            if (bit_count(&body) > (size_t)n * (size_t)count * (size_t)depth + 64 || (differs && !k->apple)) {
                as_is = true;
                continue;
            }
            g_compressed++;
            g_apart[apart]++;
            for (int c = 0; c < count; c++) {
                g_modes[mode[c]]++;
                if (order[c] == 0) g_order0++;
                else if (order[c] == 31) g_order31++;
                else g_orders++;
            }
        } else {
            /* As it is: the header's low-byte count is not used; ffmpeg
               still wants it to leave no more than 32 bits. */
            if (!k->apple) apart = depth == 32 && count == 2 ? 1 : 0;
            for (uint32_t i = 0; i < n; i++)
                for (int c = 0; c < count; c++) put(&body, (uint32_t)src[c][i * stride], depth);
            g_verbatim++;
        }
        put(w, (unsigned)type, 3);
        put(w, (unsigned)rnd_in(0, 15), 4);
        put(w, 0, 12);
        put(w, partial, 1);
        put(w, (unsigned)apart, 2);
        put(w, as_is, 1);
        if (partial) {
            put(w, n, 32);
            g_partial++;
        }
        put_bits_of(w, &body);
        break;
    }
    free(body.b.p);
    free(x[0]);
    free(x[1]);
    free(r[0]);
    free(r[1]);
    free(y);
}

/* One packet of `n` samples per channel, interleaved in WAV order in src. */
static void put_packet(bytes *out, const alac_kind *k, const ahalac_setup *s, const int32_t *src, uint32_t n)
{
    bitw w = {{0}, 0, 0};
    int channels = k->channels;
    uint8_t types[8];
    int elements = 0;
    if (k->apple && rnd_in(0, 2) == 0) {
        /* Any run of single channels and pairs: Apple's decoder places them
           in turn. */
        for (int ch = 0; ch < channels;) {
            int t = ch + 1 < channels && rnd_in(0, 1) ? AHALAC_CPE : rnd_in(0, 3) ? AHALAC_SCE : AHALAC_LFE;
            types[elements++] = (uint8_t)t;
            ch += t == AHALAC_CPE ? 2 : 1;
        }
        g_odd_layout++;
    } else {
        for (int e = 0; e < ahalac_element_count[channels - 1]; e++) types[elements++] = ahalac_elements[channels - 1][e];
        /* A bass channel last may say so. */
        if (channels >= 6 && rnd_in(0, 1)) {
            types[elements - 1] = AHALAC_LFE;
            g_lfe++;
        }
    }
    const uint8_t *place = ahalac_channel_place[channels - 1];
    for (int e = 0, ch = 0; e < elements; e++) {
        if (k->apple && rnd_in(0, 5) == 0) {
            /* Things to skip: data and fill. */
            if (rnd_in(0, 1)) {
                int bytes_n = rnd_in(0, 3) ? rnd_in(0, 20) : rnd_in(255, 510);
                bool align = rnd_in(0, 1);
                put(&w, AHALAC_DSE, 3);
                put(&w, (unsigned)rnd_in(0, 15), 4);
                put(&w, align, 1);
                put(&w, (unsigned)(bytes_n < 255 ? bytes_n : 255), 8);
                if (bytes_n >= 255) put(&w, (unsigned)(bytes_n - 255), 8);
                if (align && w.n) put(&w, 0, 8 - w.n);
                for (int i = 0; i < bytes_n; i++) put(&w, rnd(), 8);
                g_dse++;
            } else {
                /* A count of 15 means 14 more than the next byte; 14 itself
                   can be written either way. */
                int bytes_n = rnd_in(0, 3) ? rnd_in(0, 14) : rnd_in(14, 269);
                bool long_form = bytes_n >= 15 || (bytes_n == 14 && rnd_in(0, 1));
                put(&w, AHALAC_FIL, 3);
                put(&w, long_form ? 15u : (unsigned)bytes_n, 4);
                if (long_form) put(&w, (unsigned)(bytes_n - 14), 8);
                for (int i = 0; i < bytes_n; i++) put(&w, rnd(), 8);
                g_fil++;
            }
        }
        const int32_t *chans[2] = {src + place[ch], NULL};
        if (types[e] == AHALAC_CPE) chans[1] = src + place[ch + 1];
        if (types[e] == AHALAC_LFE) g_lfe++;
        put_element(&w, k, s, types[e], chans, (size_t)channels, n);
        ch += types[e] == AHALAC_CPE ? 2 : 1;
    }
    put(&w, AHALAC_END, 3);
    if (w.n) put(&w, 0, 8 - w.n);
    bytes_put(out, w.b.p, w.b.len);
    free(w.b.p);
}

/* ---- an M4A file around the packets ----------------------------------------------- */

static void be(bytes *b, uint64_t v, int n)
{
    for (int i = n - 1; i >= 0; i--) {
        unsigned char c = (unsigned char)(v >> (8 * i));
        bytes_put(b, &c, 1);
    }
}

/* Starts a box; box_end fills in its size. */
static size_t box(bytes *b, const char *type)
{
    size_t at = b->len;
    be(b, 0, 4);
    bytes_put(b, type, 4);
    return at;
}

static void box_end(bytes *b, size_t at)
{
    uint32_t size = (uint32_t)(b->len - at);
    for (int i = 0; i < 4; i++) b->p[at + (size_t)i] = (unsigned char)(size >> (24 - 8 * i));
}

static void zeros(bytes *b, size_t n)
{
    for (size_t i = 0; i < n; i++) be(b, 0, 1);
}

static void matrix(bytes *b)
{
    static const uint32_t m[9] = {0x10000, 0, 0, 0, 0x10000, 0, 0, 0, 0x40000000};
    for (int i = 0; i < 9; i++) be(b, m[i], 4);
}

/* `cut`: the track's length in its header falls this many samples short
   of the packets', inside the last one. */
static void m4a_build(bytes *out, const ahalac_setup *s, const bytes *packets, const uint32_t *sizes, size_t count, uint64_t total, uint64_t cut)
{
    out->len = 0;
    size_t at = box(out, "ftyp");
    bytes_put(out, "M4A ", 4);
    be(out, 0, 4);
    bytes_put(out, "M4A mp42isom", 12);
    box_end(out, at);
    at = box(out, "mdat");
    size_t data = out->len;
    bytes_put(out, packets->p, packets->len);
    box_end(out, at);

    size_t moov = box(out, "moov");
    at = box(out, "mvhd");
    be(out, 0, 4 + 4 + 4);
    be(out, s->sample_rate, 4);
    be(out, total, 4);
    be(out, 0x10000, 4);
    be(out, 0x100, 2);
    zeros(out, 10);
    matrix(out);
    zeros(out, 24);
    be(out, 2, 4);
    box_end(out, at);
    size_t trak = box(out, "trak");
    at = box(out, "tkhd");
    be(out, 7, 4);
    be(out, 0, 8);
    be(out, 1, 4);
    be(out, 0, 4);
    be(out, total, 4);
    zeros(out, 8);
    be(out, 0, 4);
    be(out, 0x100, 2);
    be(out, 0, 2);
    matrix(out);
    be(out, 0, 8);
    box_end(out, at);
    size_t mdia = box(out, "mdia");
    at = box(out, "mdhd");
    be(out, 0, 12);
    be(out, s->sample_rate, 4);
    be(out, total - cut, 4);
    be(out, 0x55C4, 2);
    be(out, 0, 2);
    box_end(out, at);
    at = box(out, "hdlr");
    be(out, 0, 8);
    bytes_put(out, "soun", 4);
    zeros(out, 13);
    box_end(out, at);
    size_t minf = box(out, "minf");
    at = box(out, "smhd");
    be(out, 0, 8);
    box_end(out, at);
    size_t dinf = box(out, "dinf");
    at = box(out, "dref");
    be(out, 0, 4);
    be(out, 1, 4);
    size_t url = box(out, "url ");
    be(out, 1, 4);
    box_end(out, url);
    box_end(out, at);
    box_end(out, dinf);
    size_t stbl = box(out, "stbl");
    at = box(out, "stsd");
    be(out, 0, 4);
    be(out, 1, 4);
    size_t entry = box(out, "alac");
    zeros(out, 6);
    be(out, 1, 2);
    zeros(out, 8);
    be(out, s->channels, 2);
    be(out, s->bits, 2);
    be(out, 0, 4);
    be(out, s->sample_rate < 65536 ? (uint64_t)s->sample_rate << 16 : 0, 4);
    size_t cookie = box(out, "alac");
    be(out, 0, 4);
    uint8_t setup[AHALAC_SETUP_BYTES];
    ahalac_setup_write(s, setup);
    bytes_put(out, setup, sizeof(setup));
    box_end(out, cookie);
    box_end(out, entry);
    box_end(out, at);
    /* Every packet lasts a full frame but the last. */
    at = box(out, "stts");
    uint64_t last = total - (uint64_t)(count - 1) * s->frame_length;
    be(out, 0, 4);
    be(out, last == s->frame_length || count == 1 ? 1 : 2, 4);
    if (count > 1 && last != s->frame_length) {
        be(out, count - 1, 4);
        be(out, s->frame_length, 4);
        be(out, 1, 4);
        be(out, last, 4);
    } else if (count == 1) {
        be(out, 1, 4);
        be(out, last, 4);
    } else {
        be(out, count, 4);
        be(out, s->frame_length, 4);
    }
    box_end(out, at);
    at = box(out, "stsc");
    be(out, 0, 4);
    be(out, 1, 4);
    be(out, 1, 4);
    be(out, count, 4);
    be(out, 1, 4);
    box_end(out, at);
    at = box(out, "stsz");
    be(out, 0, 4);
    be(out, 0, 4);
    be(out, count, 4);
    for (size_t i = 0; i < count; i++) be(out, sizes[i], 4);
    box_end(out, at);
    at = box(out, "stco");
    be(out, 0, 4);
    be(out, 1, 4);
    be(out, data, 4);
    box_end(out, at);
    box_end(out, stbl);
    box_end(out, minf);
    box_end(out, mdia);
    box_end(out, trak);
    box_end(out, moov);
}

/* ---- samples -------------------------------------------------------------------- */

/* Sound of every sort: tones, noise from faint to full scale, flat runs,
   silence (long runs of it when `long_quiet`), sparse clicks, a second
   channel like the first, and stretches whose low bytes are all zero. */
static void make_samples(int32_t *out, size_t frames, int channels, int bits, bool long_quiet)
{
    int64_t top = ((int64_t)1 << (bits - 1)) - 1;
    for (int ch = 0; ch < channels; ch++) {
        size_t at = 0;
        while (at < frames) {
            size_t run = (size_t)rnd_in(50, 6000);
            int kind = rnd_in(0, 11), low = kind == 9 ? 8 * rnd_in(1, (bits - 1) / 8) : 0;
            if (long_quiet && rnd_in(0, 2) == 0) {
                run = (size_t)rnd_in(66000, 140000);
                kind = 3;
            }
            if (run > frames - at) run = frames - at;
            double amp = (double)top * (rnd_in(0, 4) == 0 ? 1.0 : (rnd_in(1, 1000) / 1000.0)), freq = rnd_in(1, 2000) / 10000.0;
            double noise = rnd_in(0, 3) == 0 ? 0 : amp * rnd_in(0, 100) / 2000.0;
            int64_t flat = (int64_t)(((double)rnd() / 8388608.0 - 1.0) * (double)top);
            for (size_t i = 0; i < run; i++) {
                int64_t v;
                if (kind == 0) v = flat;
                else if (kind == 1) v = (int64_t)(((double)((rnd() << 8) ^ rnd()) / 2147483648.0 - 1.0) * (double)top);
                else if (kind == 2) v = rnd_in(0, 1) ? top : -top - 1;
                else if (kind == 3) v = 0;
                else if (kind == 4) v = rnd_in(0, 40) ? 0 : (int64_t)((double)rnd() / 16777216.0 * (double)top) - top / 2;
                else v = (int64_t)(amp * sin(6.283185307 * freq * (double)i) + noise * ((double)rnd() / 8388608.0 - 1.0));
                if (ch > 0 && kind > 5 && kind < 9) v = (v + out[(at + i) * (size_t)channels]) / 2; /* like the first channel */
                v = (v >> low) * ((int64_t)1 << low);
                if (v > top) v = top;
                if (v < -top - 1) v = -top - 1;
                out[(at + i) * (size_t)channels + (size_t)ch] = (int32_t)v;
            }
            at += run;
        }
    }
}

/* ---- decoding ------------------------------------------------------------------- */

typedef struct {
    int32_t *p;
    size_t len, cap;
} ints;

static void ints_put(ints *s, const int32_t *d, size_t n)
{
    if (s->len + n > s->cap) {
        s->cap = (s->len + n) * 2;
        s->p = realloc(s->p, s->cap * sizeof(int32_t));
    }
    if (n) memcpy(s->p + s->len, d, n * sizeof(int32_t));
    s->len += n;
}

/* Decodes the ALAC track of an M4A held in memory. Returns NULL, or why it
   cannot be read; *bad counts packets that did not decode. */
static const char *m4a_decode(const bytes *file, ints *out, ahalac_format *f, int *bad)
{
    const char *error = NULL;
    out->len = 0;
    *bad = 0;
    ahm4a *m = ahm4a_open(test_memory_read, (void *)file, file->len, &error);
    if (!m) return error ? error : "no reason given";
    const ahm4a_info *info = ahm4a_get_info(m);
    ahalac_decoder *d = ahalac_decoder_new();
    if (info->codec != AHM4A_ALAC) error = "not ALAC";
    else if (!ahalac_decoder_configure(d, info->config, info->config_bytes, f)) error = ahalac_decoder_error(d);
    if (!error && (info->sample_rate != f->sample_rate || info->channels != f->channels || info->packet_samples != (uint32_t)f->frame_samples))
        error = "the M4A reader and the decoder disagree on the settings";
    unsigned char *packet = malloc((size_t)info->max_packet_bytes + 1);
    int32_t *pcm = error ? NULL : malloc((size_t)f->frame_samples * (size_t)f->channels * sizeof(int32_t));
    for (uint64_t i = 0; !error && i < info->packets; i++) {
        int n = 0;
        size_t got = ahm4a_read_packet(m, i, packet, (size_t)info->max_packet_bytes);
        if (got && ahalac_decode_packet(d, packet, got, pcm, (size_t)f->frame_samples * (size_t)f->channels, &n) && n) ints_put(out, pcm, (size_t)n * (size_t)f->channels);
        else (*bad)++;
    }
    free(pcm);
    free(packet);
    ahalac_decoder_free(d);
    ahm4a_close(m);
    return error;
}

/* ffmpeg gives 32-bit numbers with the sample in the top bits. It must
   give `numbers` of them (0: all of mine). */
static bool same_as_ffmpeg(const char *path, const ints *mine, int bits, size_t numbers, size_t *ref_len)
{
    bytes ref = {0};
    bool ok = ffmpeg_raw(path, NULL, "s32le", &ref);
    if (!numbers) numbers = mine->len;
    *ref_len = ref.len / 4;
    ok = ok && ref.len == numbers * 4 && numbers <= mine->len;
    for (size_t i = 0; ok && i < numbers; i++) {
        int32_t v;
        memcpy(&v, ref.p + 4 * i, 4);
        if (v != (int32_t)((uint32_t)mine->p[i] << (32 - bits))) ok = false;
    }
    free(ref.p);
    return ok;
}

/* ---- the tests ------------------------------------------------------------------ */

static void alac_build(const alac_kind *k, size_t frames, bytes *file, ints *src, uint64_t cut)
{
    if (src->cap < frames * (size_t)k->channels) {
        src->cap = frames * (size_t)k->channels;
        src->p = realloc(src->p, src->cap * sizeof(int32_t));
    }
    src->len = frames * (size_t)k->channels;
    make_samples(src->p, frames, k->channels, k->bits, k->frame > 65535);
    ahalac_setup s = {k->frame, 0, (uint8_t)k->bits, 40, 10, 14, (uint8_t)k->channels, 255, 0, 0, (uint32_t)k->rate};
    /* The code's own settings, now and then not the usual ones. */
    if (rnd_in(0, 2) == 0) {
        s.pb = (uint8_t)rnd_in(1, 255);
        s.mb = (uint8_t)rnd_in(0, 255);
        s.kb = (uint8_t)(k->apple ? rnd_in(1, 30) : rnd_in(8, 30));
    }
    bytes packets = {0};
    size_t count = (frames + k->frame - 1) / k->frame;
    uint32_t *sizes = malloc(count * sizeof(uint32_t));
    for (size_t p = 0; p < count; p++) {
        size_t at = p * k->frame, n = frames - at < k->frame ? frames - at : k->frame, before = packets.len;
        put_packet(&packets, k, &s, src->p + at * (size_t)k->channels, (uint32_t)n);
        sizes[p] = (uint32_t)(packets.len - before);
        if (sizes[p] > s.max_frame_bytes) s.max_frame_bytes = sizes[p];
    }
    m4a_build(file, &s, &packets, sizes, count, frames, cut);
    free(sizes);
    free(packets.p);
}

static void test_built(void)
{
    static const alac_kind kinds[] = {
        {44100, 2, 16, 4096, false}, {48000, 2, 24, 4096, false}, {96000, 2, 24, 4096, false}, {44100, 1, 16, 4096, false},
        {192000, 2, 32, 4096, false}, {44100, 2, 20, 4096, false}, {48000, 6, 24, 4096, false}, {44100, 8, 16, 4096, false},
        {22050, 3, 16, 1000, false}, {32000, 4, 20, 352, false}, {44100, 5, 32, 2000, false}, {88200, 7, 24, 4096, false},
        {8000, 1, 32, 4096, false}, {44100, 1, 24, 16, false}, {11025, 2, 16, 65535, false}, {48000, 1, 20, 4096, false},
        /* What only Apple's decoder reads this way. */
        {44100, 2, 16, 4096, true}, {48000, 2, 32, 4096, true}, {44100, 6, 24, 4096, true}, {44100, 1, 16, 65536, true},
        {44100, 8, 20, 1024, true}, {96000, 3, 24, 4096, true}, {44100, 2, 24, 65536, true},
    };
    bytes file = {0};
    ints src = {0}, got = {0};
    for (size_t i = 0; i < sizeof(kinds) / sizeof(kinds[0]); i++) {
        const alac_kind *k = &kinds[i];
        size_t frames = k->frame > 4096 ? (size_t)rnd_in(140000, 200000) : (size_t)rnd_in(20000, 90000);
        if (k->frame == 16) frames = (size_t)rnd_in(2000, 5000);
        /* Now and then the track's header says it ends inside the last
           packet: what is played stops there. */
        size_t last = frames - (frames - 1) / k->frame * k->frame, cut = i % 3 == 1 && last > 1 ? (size_t)rnd_in(1, (int)last - 1) : 0;
        alac_build(k, frames, &file, &src, cut);
        ahalac_format f;
        int bad = 0;
        const char *err = m4a_decode(&file, &got, &f, &bad);
        CHECK(!err && !bad && got.len == src.len && !memcmp(got.p, src.p, src.len * sizeof(int32_t)),
              "ALAC kind %zu: decoded samples differ from the source (%s, %d bad packets, %zu of %zu)", i, err ? err : "no error", bad,
              got.len, src.len);
        CHECK(!err && f.sample_rate == k->rate && f.channels == k->channels && f.bits == k->bits && f.frame_samples == (int)k->frame,
              "ALAC kind %zu: wrong format", i);

        /* Through the reader: the same sound, as floats. */
        ahreader *r = ahreader_open(test_memory_read, &file, file.len, &err);
        CHECK(r != NULL, "ALAC kind %zu: the reader refused it (%s)", i, err ? err : "?");
        if (r) {
            const ahreader_info *info = ahreader_get_info(r);
            CHECK(info->kind == AHREADER_ALAC && info->bits == k->bits && info->samples == frames - cut && info->samples_exact
                      && info->channel_mask == ahalac_channel_mask[k->channels - 1],
                  "ALAC kind %zu: the reader describes it wrongly", i);
            float *buf = malloc(src.len * sizeof(float)), *want = malloc(src.len * sizeof(float));
            size_t have = 0, n;
            size_t played = frames - cut;
            for (;;) {
                size_t ask = (size_t)rnd_in(1, 9000);
                if (ask > played - have) ask = played - have;
                if (!ask || !(n = ahreader_read(r, buf + have * (size_t)k->channels, ask))) break;
                have += n;
            }
            float extra[AHALAC_MAX_CHANNELS];
            ahalac_to_float(src.p, want, src.len, k->bits);
            CHECK(have == played && ahreader_read(r, extra, 1) == 0 && !memcmp(buf, want, played * (size_t)k->channels * sizeof(float)),
                  "ALAC kind %zu: the reader gives other sound", i);
            /* A seek lands on the sample. */
            size_t at = (size_t)rnd_in(0, (int)played - 1), len = played - at < 5000 ? played - at : 5000;
            CHECK(ahreader_seek(r, at) && ahreader_read(r, buf, len) == len && !memcmp(buf, want + at * (size_t)k->channels, len * (size_t)k->channels * sizeof(float)),
                  "ALAC kind %zu: a seek to %zu missed", i, at);
            free(buf);
            free(want);
            ahreader_close(r);
        }

        if (g_ffmpeg && !k->apple) {
            char name[64], path[600];
            size_t ref_len = 0;
            snprintf(name, sizeof(name), "built_alac_%zu.m4a", i);
            path_of(path, name);
            write_file(path, file.p, file.len);
            CHECK(same_as_ffmpeg(path, &src, k->bits, (frames - cut) * (size_t)k->channels, &ref_len), "ALAC kind %zu: ffmpeg decodes the test stream differently (%zu of %zu numbers)",
                  i, ref_len, src.len);
        }
    }
    /* Every part of the format was used. */
    CHECK(g_verbatim && g_compressed && g_apart[0] && g_apart[1] && g_apart[2] && g_mixed && g_order0 && g_order31 && g_orders && g_partial,
          "the ALAC writer left parts out (as is %d, coded %d, low bytes %d/%d/%d, mixed %d, orders %d/%d/%d, partial %d)", g_verbatim,
          g_compressed, g_apart[0], g_apart[1], g_apart[2], g_mixed, g_order0, g_order31, g_orders, g_partial);
    bool modes = true;
    for (int m = 0; m < 16; m++) modes = modes && g_modes[m];
    CHECK(modes && g_escapes && g_run_escapes && g_runs && g_lfe && g_dse && g_fil && g_odd_layout,
          "the ALAC writer left parts out (modes %d, escapes %d, run escapes %d, runs %d, LFE %d, data %d, fill %d, other layouts %d)", modes,
          g_escapes, g_run_escapes, g_runs, g_lfe, g_dse, g_fil, g_odd_layout);
    free(file.p);
    free(src.p);
    free(got.p);
}

/* Files from ffmpeg's own ALAC encoder must decode as ffmpeg decodes them. */
static void test_ffmpeg_files(void)
{
    static const struct {
        const char *args;
        int bits, channels;
    } cases[] = {
        {"-c:a alac", 16, 2},
        {"-c:a alac -sample_fmt s32p", 24, 2},
        {"-ac 1 -ar 44100 -c:a alac", 16, 1},
        {"-ac 6 -c:a alac -sample_fmt s32p", 24, 6},
        {"-ac 8 -c:a alac", 16, 8},
        {"-c:a alac -compression_level 0", 16, 2},
        {"-c:a alac -compression_level 2 -min_prediction_order 1 -max_prediction_order 30", 16, 2},
        {"-af volume=0.001 -c:a alac -sample_fmt s16p", 16, 2},
        {"-t 0.37 -ar 22050 -c:a alac -sample_fmt s32p", 24, 2},
        {"-ac 3 -ar 96000 -c:a alac", 16, 3},
    };
    char src[600];
    path_of(src, "source.wav");
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        char name[64], path[600], args[1600];
        snprintf(name, sizeof(name), "ffmpeg_alac_%zu.m4a", c);
        path_of(path, name);
        snprintf(args, sizeof(args), "-i \"%s\" %s \"%s\"", src, cases[c].args, path);
        if (!ffmpeg_run(args)) {
            printf("  %s: this ffmpeg cannot write it, skipped\n", name);
            continue;
        }
        bytes file = {0};
        ints got = {0};
        ahalac_format f = {0};
        int bad = 0;
        size_t ref_len = 0;
        read_file(path, &file);
        const char *err = m4a_decode(&file, &got, &f, &bad);
        CHECK(!err && !bad && f.bits == cases[c].bits && f.channels == cases[c].channels && got.len > 0, "%s: %s, %d bits, %d channels, %d bad packets",
              name, err ? err : "no error", f.bits, f.channels, bad);
        CHECK(!err && same_as_ffmpeg(path, &got, f.bits, 0, &ref_len), "%s: decodes differently from ffmpeg (%zu numbers, ffmpeg %zu)", name, got.len, ref_len);
        free(file.p);
        free(got.p);
    }
}

/* ---- packets worked out by hand ----------------------------------------------------

   The writer above codes with the decoder's own rules for the Golomb code
   (ahalac_internal.h), and ffmpeg checks those rules only where it reads
   ALAC as Apple's decoder does. These packets were worked out by hand from
   Apple's published decoder, each for a part ffmpeg reads otherwise. */

typedef struct {
    uint32_t v;
    int n;
} field;

static void check_vector(const char *what, uint32_t frame, int bits, int channels, int kb, const field *f, size_t count, const int32_t *want, int want_n)
{
    ahalac_setup s = {frame, 0, (uint8_t)bits, 40, 10, (uint8_t)kb, (uint8_t)channels, 255, 0, 0, 44100};
    uint8_t config[AHALAC_SETUP_BYTES];
    ahalac_setup_write(&s, config);
    bitw w = {{0}, 0, 0};
    for (size_t i = 0; i < count; i++) put(&w, f[i].v, f[i].n);
    if (w.n) put(&w, 0, 8 - w.n);
    ahalac_decoder *d = ahalac_decoder_new();
    int32_t pcm[64] = {0};
    int n = 0;
    bool ok = ahalac_decoder_configure(d, config, sizeof(config), NULL) && ahalac_decode_packet(d, w.b.p, w.b.len, pcm, 64, &n);
    ok = ok && n == want_n && !memcmp(pcm, want, (size_t)want_n * (size_t)channels * sizeof(int32_t));
    CHECK(ok, "ALAC by hand, %s: decoded otherwise (%d samples: %d %d %d %d)", what, n, pcm[0], pcm[1], pcm[2], pcm[3]);
    ahalac_decoder_free(d);
    free(w.b.p);
}

#define HEAD(type, as_is) {type, 3}, {0, 4}, {0, 12}, {0, 1}, {0, 2}, {as_is, 1}
#define CHANNEL(mode, factor) {mode, 4}, {9, 4}, {factor, 3}, {0, 5} /* mode, shift 9, history factor, order 0 */

static void test_by_hand(void)
{
    /* A mean of 0 when a run of zeros follows: Apple's parameter is 8
       (ffmpeg's 7). A history factor of 0 keeps the mean from moving. */
    static const field mean0[] = {
        HEAD(AHALAC_SCE, 0), {0, 8}, {0, 8}, CHANNEL(0, 0),
        {0x1FF, 9}, {10, 16}, /* 5, sent as it is: 9 one bits, then 2 * 5 */
        {0, 1}, {3, 4},       /* a run of 2: parameter 4, remainder 2 + 1 */
        {0x3E, 6},            /* 3, one smaller after a run: 2 * 3 - 1 = 5 one bits and a zero */
        {0, 1}, {4, 8},       /* a run of 3: the mean is 0, parameter 8 */
        {6, 3},               /* -2: 2 * 2 - 1 - 1 = 2 */
        {AHALAC_END, 3},
    };
    static const int32_t mean0_want[] = {5, 0, 0, 3, 0, 0, 0, -2};
    check_vector("a run after a mean of 0", 8, 16, 1, 14, mean0, sizeof(mean0) / sizeof(mean0[0]), mean0_want, 8);

    /* A limit of 2 on the parameter: a run's divisor is cut to 3, but all
       of its 4 (or 8) bits are read (ffmpeg reads 2). */
    static const field limit2[] = {
        HEAD(AHALAC_SCE, 0), {0, 8}, {0, 8}, CHANNEL(0, 0),
        {0x1FF, 9}, {10, 16}, /* 5 */
        {2, 2}, {3, 4},       /* a run of 5 = 1 * 3 + 3 - 1 */
        {2, 2},               /* 1, one smaller after a run */
        {0, 1}, {2, 8},       /* a run of 1, parameter 8 */
        {AHALAC_END, 3},
    };
    static const int32_t limit2_want[] = {5, 0, 0, 0, 0, 0, 1, 0};
    check_vector("a parameter limit of 2", 8, 16, 1, 2, limit2, sizeof(limit2) / sizeof(limit2[0]), limit2_want, 8);

    /* 65,535 right after a run: Apple moves the mean by 65,536 times the
       history factor (ffmpeg holds it at 65,535), so the next parameter
       is 12. */
    static const field clamp[] = {
        HEAD(AHALAC_SCE, 0), {0, 8}, {0, 8}, CHANNEL(0, 4),
        {0, 1},                  /* 0 */
        {0, 1}, {2, 4},          /* a run of 1 */
        {0x1FF, 9}, {65535, 24}, /* 32768: 65536, one smaller after a run */
        {0, 1}, {2001, 12},      /* 1000 with parameter 12 */
        {AHALAC_END, 3},
    };
    static const int32_t clamp_want[] = {0, 0, 32768, 1000};
    check_vector("65,535 after a run", 4, 24, 1, 14, clamp, sizeof(clamp) / sizeof(clamp[0]), clamp_want, 4);

    /* Prediction mode 1: Apple runs the running sum first, as for 15
       (ffmpeg treats it as 0). */
    static const field mode1[] = {
        HEAD(AHALAC_SCE, 0), {0, 8}, {0, 8}, CHANNEL(1, 4),
        {6, 3}, {0, 2}, /* 1, then a run of none */
        {2, 2}, {0, 2}, /* 1 (one smaller after a run), a run of none */
        {2, 2}, {0, 2},
        {2, 2},
        {AHALAC_END, 3},
    };
    static const int32_t mode1_want[] = {1, 2, 3, 4};
    check_vector("prediction mode 1", 4, 16, 1, 14, mode1, sizeof(mode1) / sizeof(mode1[0]), mode1_want, 4);

    /* A mix weight of 0xFF is -1 to Apple: left = u + 2v, right = u + v
       (ffmpeg reads 255). */
    static const field weight[] = {
        HEAD(AHALAC_CPE, 0), {0, 8}, {0xFF, 8}, CHANNEL(0, 4), CHANNEL(0, 4),
        {6, 3}, {6, 3}, /* u = 1, v = 1 */
        {AHALAC_END, 3},
    };
    static const int32_t weight_want[] = {3, 2};
    check_vector("a mix weight of -1", 1, 16, 2, 14, weight, sizeof(weight) / sizeof(weight[0]), weight_want, 1);

    /* A 32-bit pair as it is, with no low bytes apart (ffmpeg refuses it). */
    static const field wide[] = {
        HEAD(AHALAC_CPE, 1), {0x80000000u, 32}, {0x7FFFFFFFu, 32},
        {AHALAC_END, 3},
    };
    static const int32_t wide_want[] = {INT32_MIN, INT32_MAX};
    check_vector("a 32-bit pair as it is", 1, 32, 2, 14, wide, sizeof(wide) / sizeof(wide[0]), wide_want, 1);

    /* Fill and data to skip, then three channels as a pair and a single
       one: the pair takes Apple's first two places (centre, left), the
       single one the right. ffmpeg refuses the fill and data, and puts a
       pair at the place of its first channel and the one after. */
    static const field odd[] = {
        {AHALAC_FIL, 3}, {15, 4}, {2, 8}, /* 15 + 2 - 1 = 16 bytes */
        {0x5A5A5A5A, 32}, {0x5A5A5A5A, 32}, {0x5A5A5A5A, 32}, {0x5A5A5A5A, 32},
        {AHALAC_DSE, 3}, {0, 4}, {1, 1}, {3, 8}, {0, 1}, /* 3 bytes, after lining up with a byte */
        {0xABCDEF, 24},
        HEAD(AHALAC_CPE, 1), {100, 16}, {200, 16},
        HEAD(AHALAC_SCE, 1), {300, 16},
        {AHALAC_END, 3},
    };
    static const int32_t odd_want[] = {200, 300, 100};
    check_vector("fill, data and a pair first", 1, 16, 3, 14, odd, sizeof(odd) / sizeof(odd[0]), odd_want, 1);
}

/* Damaged settings and packets: no crash, and never more than a packet. */
static void test_damage(void)
{
    alac_kind k = {44100, 2, 24, 4096, true};
    bytes file = {0};
    ints src = {0}, got = {0};
    alac_build(&k, 30000, &file, &src, 0);
    ahm4a *m = ahm4a_open(test_memory_read, &file, file.len, NULL);
    CHECK(m != NULL, "the ALAC test file did not open");
    if (!m) return;
    const ahm4a_info *info = ahm4a_get_info(m);
    ahalac_decoder *d = ahalac_decoder_new();
    int32_t *pcm = malloc(4096 * 2 * sizeof(int32_t));
    int n;
    CHECK(!ahalac_decode_packet(d, file.p, 100, pcm, 4096 * 2, &n) && n == 0 && ahalac_decoder_error(d), "a decoder not set up decoded");
    CHECK(ahalac_decoder_configure(d, info->config, info->config_bytes, NULL), "the settings were refused");

    /* The settings inside the boxes older files wrap them in. */
    uint8_t wrapped[12 + 12 + 24] = {0, 0, 0, 12, 'f', 'r', 'm', 'a', 'a', 'l', 'a', 'c', 0, 0, 0, 36, 'a', 'l', 'a', 'c'};
    memcpy(wrapped + 24, info->config, 24);
    ahalac_format f;
    CHECK(ahalac_decoder_configure(d, wrapped, sizeof(wrapped), &f) && f.bits == 24 && f.channels == 2 && f.frame_samples == 4096,
          "the settings inside 'frma' and 'alac' boxes were not read");
    CHECK(ahalac_decoder_configure(d, wrapped + 12, sizeof(wrapped) - 12, &f) && f.sample_rate == 44100, "the settings inside an 'alac' box were not read");
    uint8_t setup[24];
    memcpy(setup, info->config, 24);
    setup[5] = 12;
    CHECK(!ahalac_decoder_configure(d, setup, 24, NULL) && ahalac_decoder_error(d), "a bit depth of 12 was taken");
    setup[5] = 24;
    setup[9] = 9;
    CHECK(!ahalac_decoder_configure(d, setup, 24, NULL), "9 channels were taken");
    setup[9] = 2;
    setup[0] = 1;
    CHECK(!ahalac_decoder_configure(d, setup, 24, NULL), "a packet of 16 million samples was taken");
    CHECK(!ahalac_decoder_configure(d, setup, 23, NULL), "settings cut short were taken");
    for (int round = 0; round < 300; round++) {
        static const uint8_t depths[4] = {16, 20, 24, 32};
        for (int i = 0; i < 24; i++) setup[i] = (uint8_t)rnd();
        setup[0] = 0;
        setup[1] = (uint8_t)rnd_in(0, 1);
        if (rnd_in(0, 3)) setup[5] = depths[rnd_in(0, 3)];
        if (rnd_in(0, 3)) setup[9] = (uint8_t)rnd_in(1, 8);
        if (ahalac_decoder_configure(d, setup, 24, &f)) {
            size_t cap = (size_t)f.frame_samples * (size_t)f.channels;
            int32_t *big = malloc(cap * sizeof(int32_t));
            for (int p = 0; p < 4 && (uint64_t)p < info->packets; p++) {
                uint64_t off;
                uint32_t bytes_n;
                ahm4a_packet(m, (uint64_t)p, &off, &bytes_n);
                ahalac_decode_packet(d, file.p + off, bytes_n, big, cap, &n);
                CHECK(n >= 0 && n <= f.frame_samples, "a packet under random settings gave %d samples", n);
            }
            free(big);
        }
    }
    CHECK(ahalac_decoder_configure(d, info->config, info->config_bytes, NULL), "the settings were refused the second time");

    /* Packets with bytes changed or cut short. */
    bytes bad = {0};
    int lost = 0;
    for (int round = 0; round < 2000; round++) {
        uint64_t off;
        uint32_t bytes_n;
        ahm4a_packet(m, (uint64_t)rnd_in(0, (int)info->packets - 1), &off, &bytes_n);
        bad.len = 0;
        bytes_put(&bad, file.p + off, bytes_n);
        int hits = rnd_in(1, 8);
        for (int i = 0; i < hits; i++) bad.p[rnd() % bad.len] = (unsigned char)rnd();
        if (round % 3 == 0) bad.len = rnd() % bad.len;
        if (!ahalac_decode_packet(d, bad.p, bad.len, pcm, 4096 * 2, &n)) lost++;
        if (n < 0 || n > 4096) {
            CHECK(false, "a damaged packet gave %d samples", n);
            break;
        }
    }
    CHECK(lost > 400, "damaged packets were not caught (%d of 2000)", lost);
    /* Too little room is refused, not overrun. */
    uint64_t off;
    uint32_t bytes_n;
    ahm4a_packet(m, 0, &off, &bytes_n);
    CHECK(!ahalac_decode_packet(d, file.p + off, bytes_n, pcm, 4096, &n) && n == 0, "a packet was decoded into too little room");
    CHECK(ahalac_decode_packet(d, file.p + off, bytes_n, pcm, 4096 * 2, &n) && n == 4096 && !memcmp(pcm, src.p, 4096 * 2 * sizeof(int32_t)),
          "the first packet no longer decodes");
    /* Random bytes as packets. */
    for (int round = 0; round < 500; round++) {
        bad.len = 0;
        int len = rnd_in(0, 3000);
        for (int i = 0; i < len; i++) {
            unsigned char c = (unsigned char)rnd();
            bytes_put(&bad, &c, 1);
        }
        ahalac_decode_packet(d, bad.p, bad.len, pcm, 4096 * 2, &n);
    }
    CHECK(true, "random ALAC packets decoded without a crash");
    free(bad.p);
    free(pcm);
    ahalac_decoder_free(d);
    ahm4a_close(m);
    free(file.p);
    free(src.p);
    free(got.p);
}

void test_alac(void)
{
    test_by_hand();
    test_built();
    test_damage();
    if (g_ffmpeg) {
        make_source();
        test_ffmpeg_files();
    }
}
