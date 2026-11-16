/* FLAC decoder, following RFC 9639. Samples are worked out in 64-bit numbers
   so that the 33-bit side channel of a 32-bit stereo file needs no special
   path. */
#include "../include/ahflac.h"
#include <stdlib.h>
#include <string.h>

enum { ST_MARKER, ST_META, ST_FRAMES, ST_ERROR };

struct ahflac_decoder {
    int state;
    const char *error;
    ahflac_format format;
    bool have_info;
    bool last_meta;
    uint64_t skip_pending;

    int64_t *chan[AHFLAC_MAX_CHANNELS]; /* the frame in hand, one run per channel */
    int chan_cap;
    int block_len, block_pos;           /* its samples, and how many were given out */
    uint64_t block_start;               /* where its first sample lies in the file */
    int fixed_block;                    /* samples per frame of a stream whose frames are all one size */

    uint8_t crc8[256];
    uint16_t crc16[256];
};

/* ---- bits -------------------------------------------------------------------- */

typedef struct {
    const uint8_t *p;
    size_t len, pos; /* pos may run past len; zeros are read there */
    uint64_t cache;  /* next bits at the top */
    int bits;        /* bits in the cache */
} reader;

static void refill(reader *b)
{
    while (b->bits <= 56) {
        uint64_t byte = b->pos < b->len ? b->p[b->pos] : 0;
        b->pos++;
        b->cache |= byte << (56 - b->bits);
        b->bits += 8;
    }
}

/* True once more bits have been read than there are. */
static bool over(const reader *b)
{
    return b->pos * 8 - (size_t)b->bits > b->len * 8;
}

static uint32_t get(reader *b, int n) /* n = 0..32 */
{
    if (n == 0) return 0;
    if (b->bits < n) refill(b);
    uint32_t v = (uint32_t)(b->cache >> (64 - n));
    b->cache <<= n;
    b->bits -= n;
    return v;
}

static int64_t get_signed(reader *b, int n) /* n = 1..33 */
{
    uint64_t v;
    if (n > 32) {
        v = (uint64_t)get(b, n - 32) << 32;
        v |= get(b, 32);
    } else {
        v = get(b, n);
    }
    uint64_t sign = (uint64_t)1 << (n - 1);
    return (int64_t)(v ^ sign) - (int64_t)sign;
}

/* Counts 0 bits up to the next 1 bit, which it also takes. */
static uint32_t unary(reader *b)
{
    uint32_t q = 0;
    for (;;) {
        if (b->bits == 0) refill(b);
        if (b->cache == 0) {
            q += (uint32_t)b->bits;
            b->bits = 0;
            if (b->pos > b->len + 8) return q; /* past the end: stop counting */
            continue;
        }
        int z = 0;
        uint64_t c = b->cache;
        if (!(c >> 32)) { z += 32; c <<= 32; }
        if (!(c >> 48)) { z += 16; c <<= 16; }
        if (!(c >> 56)) { z += 8; c <<= 8; }
        if (!(c >> 60)) { z += 4; c <<= 4; }
        if (!(c >> 62)) { z += 2; c <<= 2; }
        if (!(c >> 63)) { z += 1; }
        b->cache = (b->cache << z) << 1;
        b->bits -= z + 1;
        return q + (uint32_t)z;
    }
}

/* ---- setup -------------------------------------------------------------------- */

ahflac_decoder *ahflac_decoder_new(void)
{
    ahflac_decoder *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    for (int i = 0; i < 256; i++) {
        uint8_t c8 = (uint8_t)i;
        uint16_t c16 = (uint16_t)(i << 8);
        for (int k = 0; k < 8; k++) {
            c8 = (uint8_t)(c8 & 0x80 ? (c8 << 1) ^ 0x07 : c8 << 1);
            c16 = (uint16_t)(c16 & 0x8000 ? (c16 << 1) ^ 0x8005 : c16 << 1);
        }
        d->crc8[i] = c8;
        d->crc16[i] = c16;
    }
    return d;
}

void ahflac_decoder_free(ahflac_decoder *d)
{
    if (!d) return;
    for (int ch = 0; ch < AHFLAC_MAX_CHANNELS; ch++) free(d->chan[ch]);
    free(d);
}

uint64_t ahflac_decoder_position(const ahflac_decoder *d)
{
    return d->block_start + (uint64_t)d->block_pos;
}

void ahflac_decoder_reset(ahflac_decoder *d)
{
    d->block_start += (uint64_t)d->block_len;
    d->block_len = d->block_pos = 0;
    if (d->state == ST_FRAMES) d->skip_pending = 0;
}

const char *ahflac_decoder_error(const ahflac_decoder *d)
{
    return d->error;
}

static void fail(ahflac_decoder *d, const char *why)
{
    d->state = ST_ERROR;
    d->error = why;
}

void ahflac_to_float(const int32_t *in, float *out, size_t count, int bits)
{
    float scale = 1.0f;
    for (int i = 1; i < bits; i++) scale *= 0.5f;
    for (size_t i = 0; i < count; i++) out[i] = (float)in[i] * scale;
}

/* ---- subframes ---------------------------------------------------------------- */

/* Keeps a sample within 33 bits, so that damaged data cannot overflow the
   64-bit sums. Real samples always fit. */
static int64_t wrap(int64_t v)
{
    return (int64_t)((uint64_t)v << 31) >> 31;
}

static bool residual(reader *b, int64_t *s, int n, int order)
{
    uint32_t method = get(b, 2);
    if (method > 1) return false;
    int param_bits = method ? 5 : 4;
    uint32_t escape = method ? 31 : 15;
    int part_order = (int)get(b, 4);
    int per = n >> part_order;
    if (per << part_order != n || per < order) return false;
    int i = order;
    for (int p = 0; p < 1 << part_order; p++) {
        int count = per - (p ? 0 : order);
        uint32_t k = get(b, param_bits);
        if (k == escape) {
            int raw = (int)get(b, 5);
            for (int j = 0; j < count; j++) s[i++] = raw ? get_signed(b, raw) : 0;
        } else {
            for (int j = 0; j < count; j++) {
                uint32_t u = unary(b) << k;
                u |= get(b, (int)k);
                s[i++] = (int32_t)(u >> 1 ^ (0u - (u & 1)));
            }
        }
        if (over(b)) return true; /* the caller sees the overrun */
    }
    return true;
}

/* One channel of one frame: n samples of the given width. */
static bool subframe(reader *b, int64_t *s, int n, int bps)
{
    if (get(b, 1)) return false;
    int type = (int)get(b, 6), wasted = 0;
    if (get(b, 1)) {
        wasted = (int)unary(b) + 1;
        if (wasted >= bps) return false;
        bps -= wasted;
    }
    if (type == 0) { /* one value for the whole block */
        int64_t v = get_signed(b, bps);
        for (int i = 0; i < n; i++) s[i] = v;
    } else if (type == 1) { /* stored as it is */
        for (int i = 0; i < n; i++) s[i] = get_signed(b, bps);
    } else if (type >= 8 && type <= 12) { /* fixed predictor */
        int order = type - 8;
        if (order > n) return false;
        for (int i = 0; i < order; i++) s[i] = get_signed(b, bps);
        if (!residual(b, s, n, order)) return false;
        switch (order) {
        case 1:
            for (int i = 1; i < n; i++) s[i] = wrap(s[i] + s[i - 1]);
            break;
        case 2:
            for (int i = 2; i < n; i++) s[i] = wrap(s[i] + 2 * s[i - 1] - s[i - 2]);
            break;
        case 3:
            for (int i = 3; i < n; i++) s[i] = wrap(s[i] + 3 * s[i - 1] - 3 * s[i - 2] + s[i - 3]);
            break;
        case 4:
            for (int i = 4; i < n; i++) s[i] = wrap(s[i] + 4 * s[i - 1] - 6 * s[i - 2] + 4 * s[i - 3] - s[i - 4]);
            break;
        default:
            break;
        }
    } else if (type >= 32) { /* linear predictor */
        int order = type - 31, coef[32];
        if (order > n) return false;
        for (int i = 0; i < order; i++) s[i] = get_signed(b, bps);
        int precision = (int)get(b, 4) + 1;
        if (precision == 16) return false;
        int shift = (int)get(b, 5);
        if (shift >= 16) return false; /* a negative shift is not allowed */
        for (int i = 0; i < order; i++) coef[i] = (int)get_signed(b, precision);
        if (!residual(b, s, n, order)) return false;
        for (int i = order; i < n; i++) {
            int64_t sum = 0;
            for (int j = 0; j < order; j++) sum += coef[j] * s[i - 1 - j];
            s[i] = wrap(s[i] + (sum >> shift));
        }
    } else {
        return false;
    }
    if (wasted)
        for (int i = 0; i < n; i++) s[i] = wrap((int64_t)((uint64_t)s[i] << wasted));
    return true;
}

/* ---- frames ------------------------------------------------------------------- */

/* Decodes the frame at p into d->chan. Returns 1 and its size when it is
   sound, 0 when the data ends before the frame does, -1 when this is not a
   frame or it is damaged. */
static int decode_frame(ahflac_decoder *d, const uint8_t *p, size_t len, size_t *frame_bytes)
{
    static const int rates[12] = {0, 88200, 176400, 192000, 8000, 16000, 22050, 24000, 32000, 44100, 48000, 96000};
    static const int widths[8] = {0, 8, 12, -1, 16, 20, 24, 32};
    if (len < 5) return 0;
    int block_code = p[2] >> 4, rate_code = p[2] & 15, assign = p[3] >> 4, width_code = (p[3] >> 1) & 7;
    if ((p[1] & 2) || (p[3] & 1) || block_code == 0 || rate_code == 15 || assign > 10 || width_code == 3) return -1;

    /* The frame or sample number, coded like UTF-8. */
    size_t n = 5;
    uint8_t lead = p[4];
    uint64_t number = lead;
    if (lead >= 0x80) {
        if (lead < 0xC0 || lead == 0xFF) return -1;
        uint8_t m = 0x40;
        for (; lead & m; m >>= 1) n++;
        if (len < n) return 0;
        number = lead & (m - 1u);
        for (size_t i = 5; i < n; i++) {
            if ((p[i] & 0xC0) != 0x80) return -1;
            number = number << 6 | (p[i] & 0x3Fu);
        }
    }
    size_t tail = (block_code == 6 ? 1u : block_code == 7 ? 2u : 0u) + (rate_code == 12 ? 1u : rate_code > 12 ? 2u : 0u);
    if (len < n + tail + 1) return 0;

    int block;
    if (block_code == 1) block = 192;
    else if (block_code <= 5) block = 576 << (block_code - 2);
    else if (block_code == 6) block = p[n++] + 1;
    else if (block_code == 7) { block = (p[n] << 8 | p[n + 1]) + 1; n += 2; }
    else block = 256 << (block_code - 8);
    int rate;
    if (rate_code == 0) rate = d->format.sample_rate;
    else if (rate_code < 12) rate = rates[rate_code];
    else if (rate_code == 12) rate = p[n++] * 1000;
    else { rate = (p[n] << 8 | p[n + 1]) * (rate_code == 14 ? 10 : 1); n += 2; }

    uint8_t crc8 = 0;
    for (size_t i = 0; i < n; i++) crc8 = d->crc8[crc8 ^ p[i]];
    if (crc8 != p[n]) return -1;
    n++;

    int channels = assign < 8 ? assign + 1 : 2;
    int bits = width_code ? widths[width_code] : d->format.bits;
    if (channels != d->format.channels || bits != d->format.bits || rate != d->format.sample_rate) return -1;
    if (block > d->format.max_block) return -1;

    if (block > d->chan_cap) {
        for (int ch = 0; ch < channels; ch++) {
            int64_t *grown = realloc(d->chan[ch], (size_t)d->format.max_block * sizeof(int64_t));
            if (!grown) return -1;
            d->chan[ch] = grown;
        }
        d->chan_cap = d->format.max_block;
    }

    reader b = {p + n, len - n, 0, 0, 0};
    bool sound = true;
    for (int ch = 0; ch < channels && sound; ch++) {
        bool side = (assign == 8 && ch == 1) || (assign == 9 && ch == 0) || (assign == 10 && ch == 1);
        sound = subframe(&b, d->chan[ch], block, bits + side);
        if (over(&b)) return 0;
    }
    if (!sound) return -1;

    /* Zero bits to the next whole byte, then a CRC of the whole frame. */
    size_t end = n + b.pos - (size_t)(b.bits / 8);
    if (end + 2 > len) return 0;
    uint16_t crc16 = 0;
    for (size_t i = 0; i < end; i++) crc16 = (uint16_t)(crc16 << 8 ^ d->crc16[crc16 >> 8 ^ p[i]]);
    if (crc16 != (p[end] << 8 | p[end + 1])) return -1;

    int64_t *a = d->chan[0], *c = d->chan[1];
    if (assign == 8) {
        for (int i = 0; i < block; i++) c[i] = a[i] - c[i];
    } else if (assign == 9) {
        for (int i = 0; i < block; i++) a[i] += c[i];
    } else if (assign == 10) {
        for (int i = 0; i < block; i++) {
            int64_t side = c[i], mid = (int64_t)((uint64_t)a[i] << 1) | (side & 1);
            a[i] = (mid + side) >> 1;
            c[i] = (mid - side) >> 1;
        }
    }
    d->block_len = block;
    d->block_pos = 0;
    /* A stream of frames of one size numbers its frames; one of mixed sizes
       says where each frame's first sample lies. */
    d->block_start = (p[1] & 1) ? number : number * (uint64_t)(d->fixed_block ? d->fixed_block : block);
    *frame_bytes = end + 2;
    return 1;
}

static void drain(ahflac_decoder *d, int32_t *pcm, size_t pcm_cap, int *samples)
{
    int channels = d->format.channels;
    size_t room = pcm_cap / (size_t)channels;
    int n = d->block_len - d->block_pos;
    if ((size_t)n > room) n = (int)room;
    for (int ch = 0; ch < channels; ch++) {
        const int64_t *s = d->chan[ch] + d->block_pos;
        for (int i = 0; i < n; i++) pcm[i * channels + ch] = (int32_t)(uint32_t)(uint64_t)s[i];
    }
    d->block_pos += n;
    *samples = n;
}

static size_t id3v2_size(const uint8_t *p, size_t len)
{
    if (len < 10 || memcmp(p, "ID3", 3) || p[3] == 0xFF || p[4] == 0xFF || ((p[6] | p[7] | p[8] | p[9]) & 0x80)) return 0;
    return ((size_t)p[6] << 21 | (size_t)p[7] << 14 | (size_t)p[8] << 7 | p[9]) + 10 + ((p[5] & 0x10) ? 10 : 0);
}

static bool read_stream_info(ahflac_decoder *d, const uint8_t *p)
{
    ahflac_format *f = &d->format;
    int max_block = p[2] << 8 | p[3];
    d->fixed_block = (p[0] << 8 | p[1]) == max_block && max_block >= 16 ? max_block : 0;
    f->sample_rate = p[10] << 12 | p[11] << 4 | p[12] >> 4;
    f->channels = ((p[12] >> 1) & 7) + 1;
    f->bits = ((p[12] & 1) << 4 | p[13] >> 4) + 1;
    f->samples = (uint64_t)(p[13] & 15) << 32 | (uint64_t)p[14] << 24 | (uint64_t)p[15] << 16 | (uint64_t)p[16] << 8 | p[17];
    if (f->sample_rate == 0 || f->bits < 4) return false;
    f->max_block = max_block >= 16 ? max_block : 65535;
    /* A frame is never bigger than its samples stored as they are (encoders
       fall back to that), plus the headers. */
    f->max_frame_bytes = 18 + f->channels * (int)(((int64_t)f->max_block * (f->bits + 1) + 7) / 8 + 6);
    int stated = p[7] << 16 | p[8] << 8 | p[9];
    if (stated > f->max_frame_bytes) f->max_frame_bytes = stated;
    return true;
}

size_t ahflac_decode(ahflac_decoder *d, const uint8_t *data, size_t len, bool eof, int32_t *pcm, size_t pcm_cap,
                     int *samples, ahflac_format *format)
{
    size_t pos = 0;
    *samples = 0;
    if (d->block_pos < d->block_len) {
        drain(d, pcm, pcm_cap, samples);
        goto done;
    }
    for (;;) {
        if (d->skip_pending) {
            size_t n = d->skip_pending < len - pos ? (size_t)d->skip_pending : len - pos;
            d->skip_pending -= n;
            pos += n;
            if (d->skip_pending) break;
        }
        const uint8_t *p = data + pos;
        size_t left = len - pos;
        if (d->state == ST_MARKER) {
            if (left < 10 && !eof) break;
            size_t tag = id3v2_size(p, left);
            if (tag) {
                d->skip_pending = tag;
                continue;
            }
            if (left < 4 || memcmp(p, "fLaC", 4)) {
                fail(d, "not a FLAC file");
                break;
            }
            pos += 4;
            d->state = ST_META;
        } else if (d->state == ST_META) {
            if (d->last_meta) {
                d->state = ST_FRAMES;
                continue;
            }
            size_t need = d->have_info ? 4 : 4 + 34;
            if (left < need) {
                if (eof) fail(d, "damaged FLAC header");
                break;
            }
            size_t size = (size_t)p[1] << 16 | (size_t)p[2] << 8 | p[3];
            if (!d->have_info) {
                /* The first block describes the stream. */
                if ((p[0] & 0x7F) != 0 || size < 34 || !read_stream_info(d, p + 4)) {
                    fail(d, "damaged FLAC header");
                    break;
                }
                d->have_info = true;
            }
            d->last_meta = (p[0] & 0x80) != 0;
            pos += 4;
            d->skip_pending = size;
        } else if (d->state == ST_FRAMES) {
            bool wait = false;
            for (; pos + 2 <= len; pos++) {
                if (data[pos] != 0xFF || (data[pos + 1] & 0xFC) != 0xF8) continue;
                size_t used = 0;
                int r = decode_frame(d, data + pos, len - pos, &used);
                if (r == 1) {
                    pos += used;
                    drain(d, pcm, pcm_cap, samples);
                    goto done;
                }
                if (r == 0 && !eof && len - pos < (size_t)d->format.max_frame_bytes) {
                    wait = true; /* the rest of the frame has yet to arrive */
                    break;
                }
            }
            if (!wait) {
                /* Nothing more here; a frame may start with the last byte. */
                if (eof || (len && data[len - 1] != 0xFF)) pos = len;
                else if (len) pos = len - 1;
            }
            break;
        } else {
            break;
        }
    }
done:
    if (format) {
        if (d->have_info) *format = d->format;
        else memset(format, 0, sizeof(*format));
    }
    return pos;
}
