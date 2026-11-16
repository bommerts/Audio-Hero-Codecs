/* Tests for the FLAC decoder. A small FLAC writer here builds streams that
   use every part of the format, from samples the test knows, so the decoder
   can be checked sample for sample without ffmpeg; ffmpeg then decodes the
   same streams as a check on the writer. */
#include "../include/ahflac.h"
#include "common.h"

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

/* Decodes a whole FLAC held in memory, handing over `step` more bytes each
   time the decoder runs dry and room for `cap` numbers per call. */
static const char *flac_read(const unsigned char *d, size_t len, size_t step, size_t cap, ints *out, ahflac_format *fmt)
{
    ahflac_decoder *dec = ahflac_decoder_new();
    int32_t *buf = malloc((cap ? cap : 1) * sizeof(int32_t));
    size_t have = step < len ? step : len, pos = 0;
    out->len = 0;
    memset(fmt, 0, sizeof(*fmt));
    for (;;) {
        int n;
        bool eof = have == len;
        size_t used = ahflac_decode(dec, d + pos, have - pos, eof, buf, cap, &n, fmt);
        pos += used;
        if (n) ints_put(out, buf, (size_t)n * (size_t)fmt->channels);
        if (!used && !n) {
            if (eof || ahflac_decoder_error(dec)) break;
            have = have + step < len ? have + step : len;
        }
    }
    const char *err = ahflac_decoder_error(dec);
    ahflac_decoder_free(dec);
    free(buf);
    return err;
}

/* ---- a FLAC writer for tests --------------------------------------------------- */

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

static void put_unary(bitw *w, uint32_t q)
{
    for (; q >= 32; q -= 32) put(w, 0, 32);
    put(w, 1, (int)q + 1);
}

static void align(bitw *w)
{
    if (w->n) put(w, 0, 8 - w->n);
}

static unsigned crc(const unsigned char *p, size_t n, unsigned poly, int width)
{
    unsigned c = 0, top = 1u << (width - 1), mask = (top << 1) - 1;
    for (size_t i = 0; i < n; i++) {
        c ^= (unsigned)p[i] << (width - 8);
        for (int k = 0; k < 8; k++) c = (c & top ? (c << 1) ^ poly : c << 1) & mask;
    }
    return c;
}

static int bit_length(uint64_t v)
{
    int n = 0;
    while (v >> n) n++;
    return n;
}

/* The decoder's prediction, on the encoder's side. */
static int64_t predict(const int64_t *s, int i, int order, const int *coef, int shift, bool fixed)
{
    if (fixed) {
        switch (order) {
        case 0: return 0;
        case 1: return s[i - 1];
        case 2: return 2 * s[i - 1] - s[i - 2];
        case 3: return 3 * s[i - 1] - 3 * s[i - 2] + s[i - 3];
        default: return 4 * s[i - 1] - 6 * s[i - 2] + 4 * s[i - 3] - s[i - 4];
        }
    }
    int64_t sum = 0;
    for (int j = 0; j < order; j++) sum += coef[j] * s[i - 1 - j];
    return sum >> shift;
}

static void put_residual(bitw *w, const int64_t *res, int n, int order)
{
    /* Any partition order that divides the block and leaves room for the
       warm-up samples. */
    int max_order = 0;
    while (max_order < 15 && !(n & ((2 << max_order) - 1)) && (n >> (max_order + 1)) >= order && (n >> (max_order + 1)) > 0)
        max_order++;
    int part_order = rnd_in(0, max_order), parts = 1 << part_order, per = n >> part_order;
    static int k[32768], raw[32768];
    bool wide = rnd_in(0, 3) == 0;
    for (int p = 0; p < parts; p++) {
        int from = p ? p * per : order, to = (p + 1) * per;
        uint64_t sum = 0, biggest = 0;
        for (int i = from; i < to; i++) {
            uint64_t u = res[i] < 0 ? (uint64_t)(-res[i]) * 2 - 1 : (uint64_t)res[i] * 2;
            sum += u;
            if (u > biggest) biggest = u;
        }
        int mean_bits = to > from ? bit_length(sum / (uint64_t)(to - from)) : 0;
        k[p] = mean_bits + rnd_in(-1, 1);
        if (k[p] < 0) k[p] = 0;
        if (k[p] > 30) k[p] = 30;
        /* Now and then, and whenever the unary part would run long, the
           partition is stored as plain numbers instead. */
        raw[p] = -1;
        int plain = biggest ? bit_length(biggest / 2 + 1) + 1 : 0;
        if (plain <= 31 && (rnd_in(0, 15) == 0 || (biggest >> k[p]) > 300)) raw[p] = plain;
        if (raw[p] < 0 && k[p] < bit_length(biggest) - 8) k[p] = bit_length(biggest) - 8;
        if (raw[p] < 0 && k[p] > 14) wide = true;
    }
    put(w, wide, 2);
    put(w, (unsigned)part_order, 4);
    for (int p = 0; p < parts; p++) {
        int from = p ? p * per : order, to = (p + 1) * per;
        if (raw[p] >= 0) {
            put(w, wide ? 31 : 15, wide ? 5 : 4);
            put(w, (unsigned)raw[p], 5);
            for (int i = from; i < to && raw[p]; i++) put(w, (uint64_t)res[i], raw[p]);
            continue;
        }
        put(w, (unsigned)k[p], wide ? 5 : 4);
        for (int i = from; i < to; i++) {
            uint32_t u = (uint32_t)(res[i] < 0 ? (uint64_t)(-res[i]) * 2 - 1 : (uint64_t)res[i] * 2);
            put_unary(w, u >> k[p]);
            put(w, u & ((1u << k[p]) - 1), k[p]);
        }
    }
}

/* Writes one channel of a frame, picking among every way the format has. */
static void put_subframe(bitw *w, const int64_t *in, int n, int bps, int64_t *s, int64_t *res)
{
    memcpy(s, in, (size_t)n * sizeof(int64_t));
    int wasted = 0;
    bool constant = true;
    uint64_t any = 0;
    for (int i = 0; i < n; i++) {
        any |= (uint64_t)s[i];
        if (s[i] != s[0]) constant = false;
    }
    if (any && rnd_in(0, 3)) {
        while (!((any >> wasted) & 1)) wasted++;
        if (wasted >= bps) wasted = 0;
        for (int i = 0; i < n; i++) s[i] >>= wasted;
    }
    int width = bps - wasted;

    int kind = rnd_in(0, 19); /* 0-1 stored, 2-10 fixed, 11-19 linear */
    int order = 0, coef[32], precision = 0, shift = 0;
    bool fixed = kind <= 10;
    if (constant && rnd_in(0, 4)) kind = -1;
    else if (kind >= 2 && fixed) order = rnd_in(0, 4);
    else if (kind > 10) {
        order = rnd_in(1, 32);
        precision = rnd_in(2, 15);
        shift = rnd_in(0, 15);
        if (rnd_in(0, 1)) {
            /* A predictor that fits: the second-order fixed one, scaled. */
            order = 2;
            if (precision < 4) precision = 4;
            shift = rnd_in(0, precision - 3);
            coef[0] = 2 << shift;
            coef[1] = -(1 << shift);
        } else {
            for (int j = 0; j < order; j++) coef[j] = rnd_in(-(1 << (precision - 1)), (1 << (precision - 1)) - 1);
        }
    }
    if (order > n) kind = 0;
    if (kind >= 2) {
        /* The residual must fit in 32 bits; else store the block as it is. */
        for (int i = order; i < n && kind >= 2; i++) {
            res[i] = s[i] - predict(s, i, order, coef, shift, fixed);
            if (res[i] > 0x7FFFFFFF || res[i] < -0x7FFFFFFF) kind = 0;
        }
    }

    /* Encoders never write a block bigger than its samples stored as they
       are, and the decoder counts on that: when the choice above comes out
       bigger, it is written again as stored. */
    size_t len0 = w->b.len;
    unsigned acc0 = w->acc;
    int n0 = w->n;
again:
    put(w, 0, 1);
    put(w, kind < 0 ? 0u : kind < 2 ? 1u : fixed ? 8u + (unsigned)order : 31u + (unsigned)order, 6);
    put(w, wasted != 0, 1);
    if (wasted) put_unary(w, (uint32_t)wasted - 1);
    if (kind < 0) {
        put(w, (uint64_t)s[0], width);
    } else if (kind < 2) {
        for (int i = 0; i < n; i++) put(w, (uint64_t)s[i], width);
    } else {
        for (int i = 0; i < order; i++) put(w, (uint64_t)s[i], width);
        if (!fixed) {
            put(w, (unsigned)precision - 1, 4);
            put(w, (unsigned)shift, 5);
            for (int j = 0; j < order; j++) put(w, (uint64_t)(int64_t)coef[j], precision);
        }
        put_residual(w, res, n, order);
        if ((w->b.len - len0) * 8 + (size_t)w->n > (size_t)n0 + 8 + (size_t)wasted + (size_t)n * (size_t)width) {
            w->b.len = len0;
            w->acc = acc0;
            w->n = n0;
            kind = 0;
            goto again;
        }
    }
}

typedef struct {
    int rate, channels, bits;
    uint64_t first_sample; /* the sample number the stream starts at */
    bool fixed_blocks;     /* every frame the same size, numbered by frame */
    bool plain_codes;      /* always spell out block size, rate and width where the format allows */
} flac_kind;

static void put_number(bitw *w, uint64_t v)
{
    if (v < 0x80) {
        put(w, v, 8);
        return;
    }
    /* Like UTF-8: extra + 1 one bits, a zero, the top bits, then six bits
       in each following byte. */
    int extra = 1;
    while (extra < 6 && v >> (5 * extra + 6)) extra++;
    put(w, (1u << (extra + 1)) - 1, extra + 1);
    put(w, 0, 1);
    put(w, v >> (6 * extra), 6 - extra);
    for (int i = extra - 1; i >= 0; i--) put(w, 0x80 | ((v >> (6 * i)) & 0x3F), 8);
}

/* style picks, once per stream, how the rate and width are spelled out:
   ffmpeg's frame finder distrusts streams whose frames do not agree. */
static void put_frame(bitw *w, const flac_kind *k, const int32_t *src, int block, uint64_t number, unsigned style)
{
    static const int rates[12] = {0, 88200, 176400, 192000, 8000, 16000, 22050, 24000, 32000, 44100, 48000, 96000};
    static const int widths[8] = {0, 8, 12, -1, 16, 20, 24, 32};
    size_t start = w->b.len;
    int block_code = block <= 256 ? 6 : 7;
    if (!k->plain_codes || rnd_in(0, 1)) {
        if (block == 192) block_code = 1;
        for (int c = 2; c <= 5; c++)
            if (block == 576 << (c - 2)) block_code = c;
        for (int c = 8; c <= 15; c++)
            if (block == 256 << (c - 8)) block_code = c;
    }
    int rate_code = 0, width_code = 0;
    if (k->plain_codes || (style & 1)) {
        for (int c = 1; c < 12; c++)
            if (rates[c] == k->rate) rate_code = c;
        if (!rate_code || (k->plain_codes && (style & 2))) {
            if (k->rate % 1000 == 0 && k->rate / 1000 < 256 && (style & 4)) rate_code = 12;
            else if (k->rate < 65536) rate_code = 13;
            else if (k->rate % 10 == 0 && k->rate / 10 < 65536) rate_code = 14;
        }
    }
    if (k->plain_codes || (style & 8))
        for (int c = 1; c < 8; c++)
            if (widths[c] == k->bits) width_code = c;
    int assign = k->channels - 1;
    if (k->channels == 2 && rnd_in(0, 3)) assign = rnd_in(8, 10);

    put(w, 0x3FFE, 14);
    put(w, 0, 1);
    put(w, !k->fixed_blocks, 1);
    put(w, (unsigned)block_code, 4);
    put(w, (unsigned)rate_code, 4);
    put(w, (unsigned)assign, 4);
    put(w, (unsigned)width_code, 3);
    put(w, 0, 1);
    put_number(w, number);
    if (block_code == 6) put(w, (unsigned)block - 1, 8);
    if (block_code == 7) put(w, (unsigned)block - 1, 16);
    if (rate_code == 12) put(w, (unsigned)k->rate / 1000, 8);
    if (rate_code == 13) put(w, (unsigned)k->rate, 16);
    if (rate_code == 14) put(w, (unsigned)k->rate / 10, 16);
    put(w, crc(w->b.p + start, w->b.len - start, 0x07, 8), 8);

    int64_t *in = malloc((size_t)block * 3 * sizeof(int64_t)), *work = in + block, *res = work + block;
    for (int ch = 0; ch < k->channels; ch++) {
        int bps = k->bits;
        for (int i = 0; i < block; i++) {
            int64_t a = src[i * k->channels], b = k->channels > 1 ? src[i * k->channels + 1] : 0;
            if (assign < 8) in[i] = src[i * k->channels + ch];
            else if (assign == 8) in[i] = ch ? a - b : a;
            else if (assign == 9) in[i] = ch ? b : a - b;
            else in[i] = ch ? a - b : (a + b) >> 1;
        }
        if ((assign == 8 && ch == 1) || (assign == 9 && ch == 0) || (assign == 10 && ch == 1)) bps++;
        put_subframe(w, in, block, bps, work, res);
    }
    free(in);
    align(w);
    put(w, crc(w->b.p + start, w->b.len - start, 0x8005, 16), 16);
}

/* Sound of every sort for one channel: tones, noise from faint to full
   scale, flat runs, and stretches whose low bits are all zero. */
static void make_samples(int32_t *out, size_t frames, int channels, int bits)
{
    int64_t top = ((int64_t)1 << (bits - 1)) - 1;
    for (int ch = 0; ch < channels; ch++) {
        size_t at = 0;
        while (at < frames) {
            size_t run = (size_t)rnd_in(50, 6000);
            if (run > frames - at) run = frames - at;
            int kind = rnd_in(0, 9), low = kind == 7 ? rnd_in(1, bits - 1) : 0;
            double amp = (double)top * (rnd_in(0, 4) == 0 ? 1.0 : (rnd_in(1, 1000) / 1000.0)), freq = rnd_in(1, 2000) / 10000.0;
            double noise = rnd_in(0, 3) == 0 ? 0 : amp * rnd_in(0, 100) / 2000.0;
            int64_t flat = (int64_t)(((double)rnd() / 8388608.0 - 1.0) * (double)top);
            for (size_t i = 0; i < run; i++) {
                int64_t v;
                if (kind == 0) v = flat;
                else if (kind == 1) v = (int64_t)(((double)((rnd() << 8) ^ rnd()) / 2147483648.0 - 1.0) * (double)top); /* full-scale noise */
                else if (kind == 2) v = rnd_in(0, 1) ? top : -top - 1;
                else if (kind == 3) v = 0;
                else v = (int64_t)(amp * sin(6.283185307 * freq * (double)i) + noise * ((double)rnd() / 8388608.0 - 1.0));
                if (ch == 1 && kind > 4) v = (v + out[(at + i) * (size_t)channels]) / 2; /* like the first channel */
                v = (v >> low) * ((int64_t)1 << low);
                if (v > top) v = top;
                if (v < -top - 1) v = -top - 1;
                out[(at + i) * (size_t)channels + (size_t)ch] = (int32_t)v;
            }
            at += run;
        }
    }
}

static void put_be(bytes *b, uint64_t v, int n)
{
    for (int i = n - 1; i >= 0; i--) {
        unsigned char c = (unsigned char)(v >> (8 * i));
        bytes_put(b, &c, 1);
    }
}

/* A whole FLAC file of `frames` samples; the samples it holds go to src. */
static void flac_build(const flac_kind *k, size_t frames, bytes *out, ints *src)
{
    if (src->cap < frames * (size_t)k->channels) {
        src->cap = frames * (size_t)k->channels;
        src->p = realloc(src->p, src->cap * sizeof(int32_t));
    }
    src->len = frames * (size_t)k->channels;
    make_samples(src->p, frames, k->channels, k->bits);

    static const int common[] = {192, 576, 1152, 2304, 4608, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
    /* ffmpeg's frame finder gives up on frames much over 160,000 bytes (a
       limit of its own, not of the format), so the streams it also decodes
       keep their frames under 100,000. The others go up to the largest. */
    int biggest = k->first_sample ? 65535 : 100000 * 8 / (k->channels * (k->bits + 1));
    int fixed;
    do fixed = common[rnd_in(0, 12)];
    while (fixed > biggest);
    unsigned style = rnd();
    bitw w = {{0}, 0, 0};
    size_t at = 0;
    int min_block = 65535, max_block = 16;
    uint64_t number = k->fixed_blocks ? k->first_sample / (uint64_t)fixed : k->first_sample;
    while (at < frames) {
        int block = fixed;
        if (!k->fixed_blocks) {
            int r = rnd_in(0, 9);
            block = r < 4 ? common[rnd_in(0, 12)] : r < 7 ? rnd_in(16, 300) : r < 9 ? rnd_in(16, 5000) : rnd_in(16, 65535);
        }
        if (block > biggest) block = biggest;
        if ((size_t)block > frames - at) block = (int)(frames - at);
        put_frame(&w, k, src->p + at * (size_t)k->channels, block, number, style);
        number += k->fixed_blocks ? 1 : (uint64_t)block;
        at += (size_t)block;
        if (at < frames || block >= 16) {
            if (block < min_block) min_block = block;
            if (block > max_block) max_block = block;
        }
    }
    if (min_block > max_block) min_block = max_block;
    if (k->fixed_blocks) min_block = max_block = fixed;

    out->len = 0;
    bytes_put(out, "fLaC", 4);
    put_be(out, 0x00, 1); /* stream information, more blocks follow */
    put_be(out, 34, 3);
    put_be(out, (unsigned)min_block, 2);
    put_be(out, (unsigned)max_block, 2);
    put_be(out, 0, 3);
    put_be(out, 0, 3);
    put_be(out, (uint64_t)k->rate << 44 | (uint64_t)(k->channels - 1) << 41 | (uint64_t)(k->bits - 1) << 36 | frames, 8);
    for (int i = 0; i < 16; i++) put_be(out, 0, 1);
    put_be(out, 0x04, 1); /* a comment block */
    put_be(out, 8, 3);
    put_be(out, 0, 8);
    put_be(out, 0x81, 1); /* padding, the last block */
    put_be(out, 333, 3);
    for (int i = 0; i < 333; i++) put_be(out, i % 7 ? 0 : 0xFF, 1);
    bytes_put(out, w.b.p, w.b.len);
    free(w.b.p);
}

static bool same_ints(const ints *a, const ints *b)
{
    return a->len == b->len && (a->len == 0 || !memcmp(a->p, b->p, a->len * sizeof(int32_t)));
}

/* ffmpeg gives 32-bit numbers with the sample in the top bits. */
static bool same_as_ffmpeg(const char *path, const ints *mine, int bits)
{
    bytes ref = {0};
    bool ok = ffmpeg_raw(path, NULL, "s32le", &ref) && ref.len == mine->len * 4;
    for (size_t i = 0; ok && i < mine->len; i++) {
        int32_t v;
        memcpy(&v, ref.p + 4 * i, 4);
        if (v != (int32_t)((uint32_t)mine->p[i] << (32 - bits))) ok = false;
    }
    free(ref.p);
    return ok;
}

static void test_built(void)
{
    static const flac_kind kinds[] = {
        {44100, 2, 16, 0, false, false},      {48000, 2, 24, 0, false, false},       {96000, 2, 24, 0, true, false},
        {8000, 1, 8, 0, false, true},         {192000, 2, 32, 0, false, false},      {44100, 1, 16, 0, true, true},
        {22050, 2, 12, 0, false, true},       {176400, 6, 20, 0, false, false},      {48000, 8, 16, 0, true, false},
        {11025, 2, 4, 0, false, false},       {37800, 2, 17, 0, false, true},        {655350, 2, 13, 0, false, true},
        {1000, 3, 29, 0, false, true},        {32000, 2, 32, 0, true, true},         {44100, 2, 7, 0, false, false},
        {88200, 4, 24, 0, false, true},       {100000, 2, 23, 0, false, true},       {44100, 5, 32, 0, false, false},
        /* Streams that start far along, for every size of sample number. */
        {44100, 2, 16, 100, false, false},    {44100, 2, 16, 3000, false, false},    {48000, 2, 24, 70000, true, false},
        {44100, 1, 16, 3000000, false, false}, {44100, 2, 16, 70000000, false, false},
        {96000, 2, 24, 0x80000000u, false, false}, {44100, 2, 16, 0xFFFFFFFF0ull, false, false},
    };
    bytes flac = {0};
    ints src = {0}, got = {0};
    for (size_t i = 0; i < sizeof(kinds) / sizeof(kinds[0]); i++) {
        const flac_kind *k = &kinds[i];
        size_t frames = (size_t)rnd_in(20000, 90000);
        flac_build(k, frames, &flac, &src);
        ahflac_format f;
        const char *err = flac_read(flac.p, flac.len, flac.len, 1 << 20, &got, &f);
        CHECK(!err && same_ints(&src, &got), "FLAC kind %zu: decoded samples differ from the source (%s, %zu of %zu)", i,
              err ? err : "no error", got.len, src.len);
        CHECK(f.sample_rate == k->rate && f.channels == k->channels && f.bits == k->bits && f.samples == frames,
              "FLAC kind %zu: wrong format", i);
        /* In pieces: fed by the frame size the decoder asks for, and by a
           few bytes with room for a few samples. */
        err = flac_read(flac.p, flac.len, (size_t)f.max_frame_bytes, 1 << 20, &got, &f);
        CHECK(!err && same_ints(&src, &got), "FLAC kind %zu: fed a frame at a time it differs", i);
        if (i % 4 == 0) {
            err = flac_read(flac.p, flac.len, (size_t)rnd_in(1, 4000), (size_t)rnd_in(k->channels, 3000), &got, &f);
            CHECK(!err && same_ints(&src, &got), "FLAC kind %zu: fed in small pieces it differs", i);
        }
        if (g_ffmpeg && !k->first_sample) {
            char name[64], path[600];
            snprintf(name, sizeof(name), "built_%zu.flac", i);
            path_of(path, name);
            write_file(path, flac.p, flac.len);
            CHECK(same_as_ffmpeg(path, &src, k->bits), "FLAC kind %zu: ffmpeg decodes the test stream differently", i);
        }
    }
    free(flac.p);
    free(src.p);
    free(got.p);
}

static void test_against_ffmpeg(void)
{
    static const struct {
        const char *args;
        int bits;
    } cases[] = {
        {"-c:a flac -sample_fmt s16 -compression_level 0", 16},
        {"-c:a flac -sample_fmt s16 -compression_level 5", 16},
        {"-c:a flac -sample_fmt s16 -compression_level 12 -ar 44100", 16},
        {"-c:a flac -sample_fmt s32 -compression_level 8", 24},
        {"-c:a flac -sample_fmt s32 -bits_per_raw_sample 32 -strict experimental", 32},
        {"-c:a flac -sample_fmt s16 -ac 1 -ar 8000", 16},
        {"-c:a flac -sample_fmt s32 -ac 6 -ar 96000 -frame_size 16384", 24},
        {"-c:a flac -sample_fmt s16 -ar 192000 -lpc_type cholesky -lpc_passes 3 -max_prediction_order 32", 16},
        {"-c:a flac -sample_fmt s16 -ch_mode mid_side -frame_size 300", 16},
        {"-c:a flac -sample_fmt s32 -ar 22050 -lpc_type fixed", 24},
        {"-af volume=0.001 -c:a flac -sample_fmt s16", 16},
        {"-af \"volume=8,alimiter=limit=1\" -c:a flac -sample_fmt s32 -ac 8", 24},
    };
    char src[600];
    path_of(src, "source.wav");
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        char name[64], path[600], args[1600];
        snprintf(name, sizeof(name), "ffmpeg_%zu.flac", c);
        path_of(path, name);
        snprintf(args, sizeof(args), "-i \"%s\" %s \"%s\"", src, cases[c].args, path);
        if (!ffmpeg_run(args)) {
            printf("  %s: this ffmpeg cannot write it, skipped\n", name);
            continue;
        }
        bytes flac = {0};
        ints got = {0};
        ahflac_format f;
        read_file(path, &flac);
        const char *err = flac_read(flac.p, flac.len, 65536, 1 << 20, &got, &f);
        CHECK(!err && f.bits == cases[c].bits && got.len > 0 && got.len == f.samples * (uint64_t)f.channels,
              "%s: %s, %d bits, %zu samples of %llu", name, err ? err : "no error", f.bits, got.len / (size_t)(f.channels ? f.channels : 1),
              (unsigned long long)f.samples);
        CHECK(same_as_ffmpeg(path, &got, f.bits), "%s: decodes differently from ffmpeg", name);
        free(flac.p);
        free(got.p);
    }
}

static void test_bad(void)
{
    bytes flac = {0}, bad = {0};
    ints src = {0}, got = {0};
    ahflac_format f;
    flac_kind k = {44100, 2, 16, 0, false, false};
    flac_build(&k, 60000, &flac, &src);

    CHECK(flac_read((const unsigned char *)"RIFF....WAVEfmt not a FLAC file at all", 38, 38, 64, &got, &f) != NULL, "junk read as FLAC");
    CHECK(flac_read(flac.p, 0, 1, 64, &got, &f) != NULL && got.len == 0, "an empty file read as FLAC");
    CHECK(flac_read(flac.p, 20, 20, 64, &got, &f) != NULL && got.len == 0, "a file cut off in the header read as FLAC");

    /* A tag in front and junk between the header and the first frame. */
    unsigned char tag[10 + 77] = {'I', 'D', '3', 4, 0, 0, 0, 0, 0, 77};
    for (int i = 10; i < 87; i++) tag[i] = (unsigned char)(i % 5 ? 0xFF : 0xF8);
    bytes_put(&bad, tag, sizeof(tag));
    bytes_put(&bad, flac.p, 4 + 38 + 12 + 337);
    bytes_put(&bad, tag + 10, 77);
    bytes_put(&bad, flac.p + 4 + 38 + 12 + 337, flac.len - (4 + 38 + 12 + 337));
    const char *err = flac_read(bad.p, bad.len, 5000, 1 << 16, &got, &f);
    CHECK(!err && same_ints(&src, &got), "a tag and junk change the samples");

    /* Cut off mid-frame: whole frames before the cut come out, exact. */
    err = flac_read(flac.p, flac.len * 2 / 3, 4000, 1 << 16, &got, &f);
    CHECK(!err && got.len > 0 && got.len < src.len && !memcmp(got.p, src.p, got.len * 4), "a cut-off file lost its whole frames");

    /* After a seek: decoding from the middle gives the end of the file. */
    ahflac_decoder *d = ahflac_decoder_new();
    int32_t *buf = malloc(sizeof(int32_t) << 18);
    int n;
    size_t pos = 0, used;
    ints tail = {0};
    do {
        used = ahflac_decode(d, flac.p + pos, flac.len - pos, true, buf, 1 << 18, &n, &f);
        pos += used;
    } while (used && !n);
    ahflac_decoder_reset(d);
    pos = flac.len / 2;
    for (;;) {
        used = ahflac_decode(d, flac.p + pos, flac.len - pos, true, buf, 1 << 18, &n, &f);
        pos += used;
        if (n) ints_put(&tail, buf, (size_t)n * 2);
        if (!used && !n) break;
    }
    ahflac_decoder_free(d);
    CHECK(tail.len > 0 && tail.len < src.len && !memcmp(tail.p, src.p + src.len - tail.len, tail.len * 4),
          "decoding from the middle does not give the end of the file");

    /* Random damage: no crash, damaged frames dropped, the rest exact. */
    size_t dropped_runs = 0;
    for (int round = 0; round < 400; round++) {
        bad.len = 0;
        bytes_put(&bad, flac.p, flac.len);
        int hits = rnd_in(1, 30);
        for (int i = 0; i < hits; i++) bad.p[rnd() % bad.len] = (unsigned char)rnd();
        if (round % 4 == 0) bad.len = rnd() % bad.len;
        if (round % 7 == 0)
            for (int i = 0; i < 300; i++) bad.p[rnd() % (bad.len ? bad.len : 1)] = 0xFF;
        flac_read(bad.p, bad.len, (size_t)rnd_in(100, 70000), (size_t)rnd_in(2, 70000), &got, &f);
        if (got.len > src.len) {
            CHECK(false, "a damaged FLAC gave more samples than the file holds");
            break;
        }
        if (got.len < src.len) dropped_runs++;
    }
    CHECK(dropped_runs > 100, "damaged frames were not dropped (%zu of 400 runs lost samples)", dropped_runs);
    for (int round = 0; round < 60; round++) {
        bad.len = 0;
        bytes_put(&bad, flac.p, 42);
        for (int i = 0; i < 30000; i++) {
            unsigned char c = (unsigned char)(rnd_in(0, 5) == 0 ? 0xFF : rnd_in(0, 3) == 0 ? 0xF8 : rnd());
            bytes_put(&bad, &c, 1);
        }
        bad.p[4] |= 0x80; /* the stream information is the only header block */
        flac_read(bad.p, bad.len, 30000, 1 << 16, &got, &f);
    }
    CHECK(true, "noise decoded without a crash");
    free(buf);
    free(tail.p);
    free(bad.p);
    free(flac.p);
    free(src.p);
    free(got.p);
}

void test_flac(void)
{
    test_built();
    test_bad();
    if (g_ffmpeg) {
        make_source();
        test_against_ffmpeg();
    }
}
