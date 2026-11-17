/* Apple Lossless (ALAC) decoder. ALAC has no standard of its own beyond
   what Apple published with its reference code in 2011: the magic cookie
   description, and the bit stream that its decoder reads. This follows that
   decoder; where Apple's code has no defined result (signed overflow), it
   does what ffmpeg does.

   A packet is a run of elements, each one or two channels, ended by an end
   element. A channel's samples are coded either as they are, or as the
   residual of an adaptive linear predictor in an adaptive Golomb code. A
   pair of channels may be mixed (one channel minus a part of the other),
   and the low bytes of each sample may be sent apart, as they are. */
#include "../include/ahalac.h"
#include "ahalac_internal.h"
#include <stdlib.h>
#include <string.h>

/* ---- what decoder and encoder share ------------------------------------------- */

const uint8_t ahalac_elements[8][5] = {
    {AHALAC_SCE},
    {AHALAC_CPE},
    {AHALAC_SCE, AHALAC_CPE},
    {AHALAC_SCE, AHALAC_CPE, AHALAC_SCE},
    {AHALAC_SCE, AHALAC_CPE, AHALAC_CPE},
    {AHALAC_SCE, AHALAC_CPE, AHALAC_CPE, AHALAC_SCE},
    {AHALAC_SCE, AHALAC_CPE, AHALAC_CPE, AHALAC_SCE, AHALAC_SCE},
    {AHALAC_SCE, AHALAC_CPE, AHALAC_CPE, AHALAC_CPE, AHALAC_SCE},
};
const uint8_t ahalac_element_count[8] = {1, 1, 2, 3, 3, 4, 5, 5};

const uint8_t ahalac_channel_place[8][8] = {
    {0},
    {0, 1},
    {2, 0, 1},
    {2, 0, 1, 3},
    {2, 0, 1, 3, 4},
    {2, 0, 1, 4, 5, 3},
    {2, 0, 1, 4, 5, 6, 3},
    {2, 6, 7, 0, 1, 4, 5, 3},
};

const uint32_t ahalac_channel_mask[8] = {
    0x4,   /* C */
    0x3,   /* L R */
    0x7,   /* L R C */
    0x107, /* L R C, back centre */
    0x37,  /* L R C, back left and right */
    0x3F,  /* L R C LFE, back left and right */
    0x13F, /* L R C LFE, back left and right, back centre */
    0xFF,  /* L R C LFE, back left and right, left and right of centre */
};

static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }

bool ahalac_setup_read(const uint8_t *p, size_t len, ahalac_setup *s)
{
    if (len >= 12 + AHALAC_SETUP_BYTES && !memcmp(p + 4, "frma", 4)) {
        p += 12;
        len -= 12;
    }
    if (len >= 12 + AHALAC_SETUP_BYTES && !memcmp(p + 4, "alac", 4)) {
        p += 12;
        len -= 12;
    }
    if (len < AHALAC_SETUP_BYTES) return false;
    s->frame_length = be32(p);
    s->version = p[4];
    s->bits = p[5];
    s->pb = p[6];
    s->mb = p[7];
    s->kb = p[8];
    s->channels = p[9];
    s->max_run = (uint16_t)(p[10] << 8 | p[11]);
    s->max_frame_bytes = be32(p + 12);
    s->avg_bit_rate = be32(p + 16);
    s->sample_rate = be32(p + 20);
    return true;
}

void ahalac_setup_write(const ahalac_setup *s, uint8_t out[AHALAC_SETUP_BYTES])
{
    const uint32_t words[3] = {s->max_frame_bytes, s->avg_bit_rate, s->sample_rate};
    for (int i = 0; i < 4; i++) out[i] = (uint8_t)(s->frame_length >> (24 - 8 * i));
    out[4] = s->version;
    out[5] = s->bits;
    out[6] = s->pb;
    out[7] = s->mb;
    out[8] = s->kb;
    out[9] = s->channels;
    out[10] = (uint8_t)(s->max_run >> 8);
    out[11] = (uint8_t)s->max_run;
    for (int w = 0; w < 3; w++)
        for (int i = 0; i < 4; i++) out[12 + 4 * w + i] = (uint8_t)(words[w] >> (24 - 8 * i));
}

/* ---- the decoder ------------------------------------------------------------------ */

struct ahalac_decoder {
    ahalac_setup setup;
    ahalac_format format;
    const char *error;
    int32_t *buf[2]; /* the one or two channels of an element */
    uint32_t *low;   /* the low bits sent apart, interleaved for a pair */
    size_t room;     /* samples per channel the buffers hold */
};

ahalac_decoder *ahalac_decoder_new(void)
{
    ahalac_decoder *d = calloc(1, sizeof(*d));
    if (d) d->error = "the decoder has not been set up";
    return d;
}

void ahalac_decoder_free(ahalac_decoder *d)
{
    if (!d) return;
    free(d->buf[0]);
    free(d->buf[1]);
    free(d->low);
    free(d);
}

const char *ahalac_decoder_error(const ahalac_decoder *d)
{
    return d->error;
}

bool ahalac_decoder_configure(ahalac_decoder *d, const uint8_t *config, size_t len, ahalac_format *format)
{
    ahalac_setup s = {0};
    d->error = NULL;
    if (!config || !ahalac_setup_read(config, len, &s)) d->error = "the Apple Lossless settings are cut short";
    else if (s.bits != 16 && s.bits != 20 && s.bits != 24 && s.bits != 32) d->error = "Apple Lossless of this bit depth is not supported";
    else if (s.channels < 1 || s.channels > AHALAC_MAX_CHANNELS) d->error = "Apple Lossless with this many channels is not supported";
    else if (!s.frame_length || s.frame_length > AHALAC_MAX_FRAME) d->error = "Apple Lossless packets of this length are not supported";
    else if (!s.sample_rate || s.sample_rate > 0x7FFFFFFF) d->error = "the Apple Lossless settings give no sample rate";
    if (d->error) return false;
    if (s.frame_length > d->room) {
        free(d->buf[0]);
        free(d->buf[1]);
        free(d->low);
        d->buf[0] = malloc(s.frame_length * sizeof(int32_t));
        d->buf[1] = malloc(s.frame_length * sizeof(int32_t));
        d->low = malloc(2 * (size_t)s.frame_length * sizeof(uint32_t));
        d->room = s.frame_length;
        if (!d->buf[0] || !d->buf[1] || !d->low) {
            d->room = 0;
            d->error = "out of memory";
            return false;
        }
    }
    d->setup = s;
    d->format.sample_rate = (int)s.sample_rate;
    d->format.channels = s.channels;
    d->format.bits = s.bits;
    d->format.frame_samples = (int)s.frame_length;
    d->format.channel_mask = ahalac_channel_mask[s.channels - 1];
    if (format) *format = d->format;
    return true;
}

/* Reading bits, first bit first. Past the end it reads zeros; the caller
   checks `pos` against `end` where it matters. */
typedef struct {
    const uint8_t *p;
    size_t len;      /* bytes */
    uint64_t pos;    /* bits read */
    uint64_t end;    /* bits there are */
} bits;

static uint32_t peek32(const bits *b)
{
    size_t at = (size_t)(b->pos >> 3);
    uint64_t v = 0;
    if (b->pos < b->end && at + 5 <= b->len) {
        const uint8_t *p = b->p + at;
        v = (uint64_t)p[0] << 32 | (uint64_t)p[1] << 24 | (uint64_t)p[2] << 16 | (uint64_t)p[3] << 8 | p[4];
    } else {
        for (size_t i = 0; i < 5; i++) v = v << 8 | (b->pos < b->end && at + i < b->len ? b->p[at + i] : 0u);
    }
    return (uint32_t)(v >> (8 - (b->pos & 7)));
}

static uint32_t get(bits *b, int n)
{
    if (!n) return 0;
    uint32_t v = peek32(b) >> (32 - n);
    b->pos += (unsigned)n;
    return v;
}

static int32_t wrap32(uint32_t v)
{
    return v < 0x80000000u ? (int32_t)v : -(int32_t)(~v) - 1;
}

/* The low `n` bits of v as a signed number. */
static int32_t sign_extend(uint32_t v, int n)
{
    return wrap32(v << (32 - n)) >> (32 - n);
}

/* One value of the Golomb code (see ahalac_internal.h). */
static uint32_t golomb(bits *b, uint32_t m, int k, int width)
{
    uint32_t look = peek32(b);
    int ones = 0;
    while (ones < AHALAC_MAX_PREFIX && (look & 0x80000000u)) {
        look <<= 1;
        ones++;
    }
    if (ones == AHALAC_MAX_PREFIX) {
        b->pos += AHALAC_MAX_PREFIX;
        return get(b, width);
    }
    b->pos += (unsigned)ones + 1;
    uint32_t v = peek32(b) >> (32 - k);
    if (v < 2) {
        b->pos += (unsigned)k - 1;
        return (uint32_t)ones * m;
    }
    b->pos += (unsigned)k;
    return (uint32_t)ones * m + v - 1;
}

/* The residuals of one channel: `n` values `width` bits wide. */
static bool residuals(bits *b, int32_t *out, uint32_t n, const ahalac_setup *s, uint32_t pb, int width)
{
    uint32_t mb = s->mb, zmode = 0;
    for (uint32_t c = 0; c < n;) {
        if (b->pos >= b->end) return false;
        int k = ahalac_value_k(mb, s->kb);
        uint32_t x = golomb(b, (1u << k) - 1, k, width);
        uint32_t v = x + zmode; /* even: v / 2; odd: -(v + 1) / 2 */
        out[c++] = wrap32((v >> 1) ^ (0u - (v & 1)));
        mb = ahalac_mean(mb, pb, x, zmode);
        zmode = 0;
        if (ahalac_run_follows(mb) && c < n) {
            int kz = ahalac_run_k(mb);
            uint32_t run = golomb(b, ahalac_run_m(kz, s->kb), kz, AHALAC_RUN_BITS);
            if (run > n - c) return false;
            memset(out + c, 0, run * sizeof(int32_t));
            c += run;
            zmode = run < AHALAC_RUN_LONG;
            mb = 0;
        }
    }
    return b->pos <= b->end;
}

static int sign_of(int32_t v) { return (v > 0) - (v < 0); }

static int16_t wrap16(int v)
{
    int w = v & 0xFFFF;
    return (int16_t)(w >= 0x8000 ? w - 0x10000 : w);
}

/* Turns residuals back into samples, in place: an adaptive predictor of
   `order` (1 to 30) whose coefficients move a step after every sample
   toward what would have predicted it better. Order 0 leaves the residuals
   as they are; order 31 is a plain running sum. */
static void predict(int32_t *x, uint32_t n, int16_t *coef, int order, int shift, int width)
{
    if (n < 2 || order == 0) return;
    if (order == 31) {
        for (uint32_t j = 1; j < n; j++) x[j] = sign_extend((uint32_t)x[j] + (uint32_t)x[j - 1], width);
        return;
    }
    uint32_t j = 1;
    for (; j <= (uint32_t)order && j < n; j++) x[j] = sign_extend((uint32_t)x[j] + (uint32_t)x[j - 1], width);
    for (; j < n; j++) {
        const int32_t *past = x + j - 1; /* past[-k]: k + 1 samples back */
        int32_t top = x[j - (uint32_t)order - 1];
        uint32_t sum = 0;
        for (int k = 0; k < order; k++) sum += (uint32_t)coef[k] * ((uint32_t)past[-k] - (uint32_t)top);
        int32_t guess = (int32_t)(((int64_t)wrap32(sum) + ((int64_t)1 << (shift - 1))) >> shift);
        int32_t error = x[j];
        x[j] = sign_extend((uint32_t)error + (uint32_t)top + (uint32_t)guess, width);
        /* Each coefficient, the oldest first, moves one step until the
           error is used up. An error of exactly -2^31 (possible only for
           32-bit mono with no low bytes apart) moves none, as in ffmpeg;
           Apple's sign test negates it, which C leaves undefined. */
        if (error != 0 && error != INT32_MIN) {
            int dir = error > 0 ? 1 : -1;
            uint32_t left = (uint32_t)error;
            for (int k = order - 1; k >= 0; k--) {
                int32_t diff = wrap32((uint32_t)top - (uint32_t)past[-k]);
                int sg = sign_of(diff) * dir;
                coef[k] = wrap16(coef[k] - sg);
                uint32_t step = sg > 0 ? (uint32_t)diff : sg < 0 ? 0u - (uint32_t)diff : 0u;
                left -= (uint32_t)(order - k) * (uint32_t)(wrap32(step) >> shift);
                if (dir > 0 ? wrap32(left) <= 0 : wrap32(left) >= 0) break;
            }
        }
    }
}

/* Decodes one element of `count` channels into pcm, at the places the
   layout gives the channels `first` on. *n is the samples per channel: set
   by the first element, the same for the others. */
static bool element(ahalac_decoder *d, bits *b, int count, int first, uint32_t *n, int32_t *pcm, size_t pcm_cap)
{
    const ahalac_setup *s = &d->setup;
    int channels = s->channels, depth = s->bits;
    get(b, 4);  /* which element of its kind: not needed */
    get(b, 12); /* unused */
    int partial = (int)get(b, 1), bytes_apart = (int)get(b, 2), as_is = (int)get(b, 1);
    if (bytes_apart == 3) return false;
    uint32_t samples = partial ? get(b, 32) : s->frame_length;
    if (!samples || samples > s->frame_length || (*n && samples != *n)) return false;
    if ((size_t)samples * (size_t)channels > pcm_cap) return false;
    *n = samples;

    if (b->pos > b->end) return false;
    int32_t **x = d->buf;
    int mix_shift = 0, mix_weight = 0, apart = 0;
    if (as_is) {
        /* The samples as they are, the channels taking turns. The low bytes
           are not sent apart here, whatever the header says (as ffmpeg
           reads it; encoders write 0 there). */
        if ((uint64_t)samples * (uint64_t)count * (uint64_t)depth > b->end - b->pos) return false;
        for (uint32_t i = 0; i < samples; i++)
            for (int c = 0; c < count; c++) x[c][i] = sign_extend(get(b, depth), depth);
    } else {
        /* A pair carries one bit more: the difference of the channels. */
        apart = 8 * bytes_apart;
        int width = depth - apart + count - 1;
        if (width < 1 || width > 32 || !s->kb) return false;
        mix_shift = (int)get(b, 8);
        mix_weight = (int8_t)get(b, 8);
        if (count == 2 && mix_weight && mix_shift > 31) return false;
        int mode[2], shift[2], order[2];
        uint32_t pb[2];
        int16_t coef[2][32];
        for (int c = 0; c < count; c++) {
            mode[c] = (int)get(b, 4);
            shift[c] = (int)get(b, 4);
            pb[c] = s->pb * get(b, 3) / 4;
            order[c] = (int)get(b, 5);
            for (int i = 0; i < order[c]; i++) coef[c][i] = (int16_t)sign_extend(get(b, 16), 16);
            if (!shift[c] || (uint32_t)order[c] >= s->frame_length) return false;
        }
        if (b->pos > b->end) return false;
        bits low = *b;
        if (apart) {
            uint64_t need = (uint64_t)samples * (uint64_t)count * (uint64_t)apart;
            if (need > b->end - b->pos) return false;
            b->pos += need;
        }
        for (int c = 0; c < count; c++) {
            if (!residuals(b, x[c], samples, s, pb[c], width)) return false;
            /* Apple's decoder runs a plain running sum first for every mode
               but 0; ffmpeg does only for mode 15. */
            if (mode[c]) predict(x[c], samples, NULL, 31, 0, width);
            predict(x[c], samples, coef[c], order[c], shift[c], width);
        }
        if (apart)
            for (uint32_t i = 0; i < (uint32_t)count * samples; i++) d->low[i] = get(&low, apart);
    }

    const uint8_t *place = ahalac_channel_place[channels - 1];
    int32_t *out0 = pcm + place[first], *out1 = count == 2 ? pcm + place[first + 1] : NULL;
    for (uint32_t i = 0; i < samples; i++) {
        uint32_t a = (uint32_t)x[0][i], z;
        if (count == 2) {
            uint32_t v = (uint32_t)x[1][i];
            /* Mixed: a = r + w (l - r) / 2^shift and v = l - r; else as they are. */
            if (mix_weight) {
                a = a + v - (uint32_t)(wrap32((uint32_t)mix_weight * v) >> mix_shift);
                z = a - v;
            } else {
                z = v;
            }
            if (apart) {
                a = a << apart | d->low[2 * i];
                z = z << apart | d->low[2 * i + 1];
            }
            out0[(size_t)i * (size_t)channels] = sign_extend(a, depth);
            out1[(size_t)i * (size_t)channels] = sign_extend(z, depth);
        } else {
            if (apart) a = a << apart | d->low[i];
            out0[(size_t)i * (size_t)channels] = sign_extend(a, depth);
        }
    }
    return true;
}

bool ahalac_decode_packet(ahalac_decoder *d, const uint8_t *data, size_t len, int32_t *pcm, size_t pcm_cap, int *samples)
{
    *samples = 0;
    if (d->error || !data || len > ((size_t)1 << 28)) return false;
    bits b = {data, len, 0, (uint64_t)len * 8};
    int channels = d->setup.channels, done = 0;
    uint32_t n = 0;
    for (;;) {
        if (b.pos + 3 > b.end) return false; /* no end element */
        int kind = (int)get(&b, 3);
        if (kind == AHALAC_END) break;
        if (kind == AHALAC_SCE || kind == AHALAC_LFE || kind == AHALAC_CPE) {
            int count = kind == AHALAC_CPE ? 2 : 1;
            if (done + count > channels || !element(d, &b, count, done, &n, pcm, pcm_cap)) return false;
            done += count;
        } else if (kind == AHALAC_DSE) {
            /* Data that is not sound, to be skipped (ffmpeg refuses the
               packet; Apple's decoder skips it). */
            get(&b, 4);
            bool align = get(&b, 1);
            uint32_t bytes = get(&b, 8);
            if (bytes == 255) bytes += get(&b, 8);
            if (align) b.pos = (b.pos + 7) & ~(uint64_t)7;
            b.pos += 8 * (uint64_t)bytes;
        } else if (kind == AHALAC_FIL) {
            uint32_t bytes = get(&b, 4);
            if (bytes == 15) bytes += get(&b, 8) - 1;
            b.pos += 8 * (uint64_t)bytes;
        } else {
            return false; /* coupling and program elements: not part of ALAC */
        }
        if (b.pos > b.end) return false;
    }
    if (done != channels) return false;
    *samples = (int)n;
    return true;
}

void ahalac_to_float(const int32_t *in, float *out, size_t count, int bits)
{
    float scale = 1.0f / (float)((int64_t)1 << (bits - 1));
    for (size_t i = 0; i < count; i++) out[i] = (float)in[i] * scale;
}
