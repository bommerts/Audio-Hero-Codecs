/* Sample rate conversion with a polyphase windowed-sinc filter.

   An output sample at time t (in input samples) is the input around t
   weighted by a lowpass sinc under a Kaiser window. t = k * in / out, so
   its fraction takes only out / gcd(in, out) different values: the weights
   for each are worked out once ("phases"). Rate pairs with too many phases
   for that share a table of PHASES steps, and each output blends the two
   nearest. */
#include "../include/ahresample.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* The design. Sound below PASS of the lower rate's half is kept; from that
   half upwards it is gone by STOP_DB. The width between the two sets the
   filter's length. */
#define PASS 0.907
#define STOP_DB 100.0
/* Most phases kept one by one; above it PHASES steps are blended. */
#define EXACT_PHASES 1024
#define PHASES 1024
/* Input samples per channel taken in per round. */
#define BLOCK 2048

struct ahresample {
    int channels;
    bool copy;         /* the rates are the same */
    uint32_t step;     /* in / gcd: time moves on by step / den per output */
    uint32_t den;      /* out / gcd */
    uint32_t num;      /* fraction of the time of the next output, over den */
    int taps;          /* input samples one output is made from */
    int phases;        /* rows in coef less one when blending, else den */
    bool blend;
    float *coef;       /* [row][taps] */
    float *buf[AHRESAMPLE_MAX_CHANNELS]; /* input not yet done with, one run per channel */
    size_t cap, fill;  /* room and samples in each buf */
    size_t pos;        /* buf index of the first sample the next output uses */
    uint64_t in_total, out_total;
    bool flushing;
};

static uint32_t gcd(uint32_t a, uint32_t b)
{
    while (b) {
        uint32_t t = a % b;
        a = b;
        b = t;
    }
    return a;
}

/* The modified Bessel function of the first kind, order zero. */
static double bessel_i0(double x)
{
    double sum = 1, term = 1;
    for (int k = 1; k < 200; k++) {
        term *= (x / (2 * k)) * (x / (2 * k));
        sum += term;
        if (term < sum * 1e-17) break;
    }
    return sum;
}

/* One row of weights: the filter sampled at tap distances i - (taps/2 - 1) -
   frac from the output's time, scaled so that they add up to one (a steady
   level then comes out exactly as it went in, at every phase). */
static void make_row(float *row, double *w, int taps, double frac, double cutoff, double beta)
{
    static const double pi = 3.14159265358979323846;
    double half = taps / 2.0, i0_beta = bessel_i0(beta), sum = 0;
    for (int i = 0; i < taps; i++) {
        double x = (double)(i - (taps / 2 - 1)) - frac;
        double r = x / half;
        double window = r * r < 1 ? bessel_i0(beta * sqrt(1 - r * r)) / i0_beta : 0;
        double a = pi * cutoff * x;
        double sinc = fabs(a) < 1e-9 ? 1 : sin(a) / a;
        w[i] = sinc * window;
        sum += w[i];
    }
    for (int i = 0; i < taps; i++) row[i] = (float)(w[i] / sum);
}

static void start(ahresample *r)
{
    /* The first output is the sound at the first input sample, so its
       window reaches taps/2 - 1 samples back: silence. */
    size_t lead = r->copy ? 0 : (size_t)(r->taps / 2 - 1);
    for (int ch = 0; ch < r->channels; ch++) memset(r->buf[ch], 0, lead * sizeof(float));
    r->fill = lead;
    r->pos = 0;
    r->num = 0;
    r->in_total = r->out_total = 0;
    r->flushing = false;
}

ahresample *ahresample_new(int in_rate, int out_rate, int channels)
{
    if (in_rate < AHRESAMPLE_MIN_RATE || in_rate > AHRESAMPLE_MAX_RATE || out_rate < AHRESAMPLE_MIN_RATE
        || out_rate > AHRESAMPLE_MAX_RATE || channels < 1 || channels > AHRESAMPLE_MAX_CHANNELS)
        return NULL;
    ahresample *r = calloc(1, sizeof(*r));
    if (!r) return NULL;
    r->channels = channels;
    uint32_t g = gcd((uint32_t)in_rate, (uint32_t)out_rate);
    r->step = (uint32_t)in_rate / g;
    r->den = (uint32_t)out_rate / g;
    r->copy = in_rate == out_rate;
    if (!r->copy) {
        /* Everything is in units of the input rate. Going down, the filter
           cuts at the output's half rate and is longer by the ratio. */
        double scale = out_rate < in_rate ? (double)out_rate / in_rate : 1.0;
        double width = (1 - PASS) * 0.5 * scale; /* cycles per input sample */
        double beta = 0.1102 * (STOP_DB - 8.7);
        int taps = (int)ceil((STOP_DB - 7.95) / (2.285 * 2 * 3.14159265358979323846 * width));
        r->taps = (taps + 3) & ~3;
        double cutoff = (1 + PASS) * 0.5 * scale; /* the middle of the slope, over the half rate */
        r->blend = r->den > EXACT_PHASES;
        r->phases = r->blend ? PHASES : (int)r->den;
        size_t rows = (size_t)r->phases + (r->blend ? 1 : 0);
        r->coef = malloc(rows * (size_t)r->taps * sizeof(float));
        double *scratch = malloc((size_t)r->taps * sizeof(double));
        if (!r->coef || !scratch) {
            free(scratch);
            ahresample_free(r);
            return NULL;
        }
        for (size_t p = 0; p < rows; p++)
            make_row(r->coef + p * (size_t)r->taps, scratch, r->taps, (double)p / r->phases, cutoff, beta);
        free(scratch);
    }
    r->cap = (size_t)r->taps + BLOCK;
    for (int ch = 0; ch < channels; ch++) {
        r->buf[ch] = malloc(r->cap * sizeof(float));
        if (!r->buf[ch]) {
            ahresample_free(r);
            return NULL;
        }
    }
    start(r);
    return r;
}

void ahresample_free(ahresample *r)
{
    if (!r) return;
    for (int ch = 0; ch < AHRESAMPLE_MAX_CHANNELS; ch++) free(r->buf[ch]);
    free(r->coef);
    free(r);
}

void ahresample_reset(ahresample *r)
{
    start(r);
}

static float dot(const float *x, const float *c, int n) /* n is a multiple of 4 */
{
    float s0 = 0, s1 = 0, s2 = 0, s3 = 0;
    for (int i = 0; i < n; i += 4) {
        s0 += x[i] * c[i];
        s1 += x[i + 1] * c[i + 1];
        s2 += x[i + 2] * c[i + 2];
        s3 += x[i + 3] * c[i + 3];
    }
    return (s0 + s1) + (s2 + s3);
}

/* Makes outputs while the samples each needs are in hand. `limit` is the
   most outputs the input given so far allows in all. */
static size_t produce(ahresample *r, float *out, size_t room, uint64_t limit)
{
    size_t made = 0;
    int taps = r->taps, channels = r->channels;
    while (made < room && r->out_total < limit && r->pos + (size_t)taps <= r->fill) {
        if (r->blend) {
            uint64_t at = (uint64_t)r->num * PHASES;
            size_t row = (size_t)(at / r->den);
            float w = (float)((double)(at % r->den) / r->den);
            const float *c0 = r->coef + row * (size_t)taps, *c1 = c0 + taps;
            for (int ch = 0; ch < channels; ch++) {
                const float *x = r->buf[ch] + r->pos;
                float a = dot(x, c0, taps);
                *out++ = a + (dot(x, c1, taps) - a) * w;
            }
        } else {
            const float *c = r->coef + (size_t)r->num * (size_t)taps;
            for (int ch = 0; ch < channels; ch++) *out++ = dot(r->buf[ch] + r->pos, c, taps);
        }
        uint64_t t = (uint64_t)r->num + r->step;
        r->pos += (size_t)(t / r->den);
        r->num = (uint32_t)(t % r->den);
        r->out_total++;
        made++;
    }
    return made;
}

/* Drops the samples no later output needs. */
static void compact(ahresample *r)
{
    size_t drop = r->pos < r->fill ? r->pos : r->fill;
    if (!drop) return;
    for (int ch = 0; ch < r->channels; ch++) memmove(r->buf[ch], r->buf[ch] + drop, (r->fill - drop) * sizeof(float));
    r->fill -= drop;
    r->pos -= drop;
}

/* ceil(n * den / step): the outputs whose time lies before input sample n. */
static uint64_t outputs_for(const ahresample *r, uint64_t n)
{
    return (n * r->den + r->step - 1) / r->step;
}

size_t ahresample_process(ahresample *r, const float *in, size_t in_frames, size_t *used, float *out,
                          size_t out_room)
{
    size_t taken = 0, made = 0;
    int channels = r->channels;
    /* Once flushing has begun only ahresample_flush may hand out the rest:
       anything here would run past the end. */
    if (r->flushing) {
        if (used) *used = 0;
        return 0;
    }
    if (r->copy) {
        size_t n = in_frames < out_room ? in_frames : out_room;
        memcpy(out, in, n * (size_t)channels * sizeof(float));
        r->in_total += n;
        r->out_total += n;
        if (used) *used = n;
        return n;
    }
    for (;;) {
        /* An output needs taps/2 samples past its time, so it never runs
           ahead of the input: no limit is needed until the end. */
        made += produce(r, out + made * (size_t)channels, out_room - made, UINT64_MAX);
        if (made == out_room || taken == in_frames) break;
        compact(r);
        size_t n = r->cap - r->fill;
        if (n > in_frames - taken) n = in_frames - taken;
        for (int ch = 0; ch < channels; ch++) {
            const float *s = in + taken * (size_t)channels + ch;
            float *d = r->buf[ch] + r->fill;
            for (size_t i = 0; i < n; i++) d[i] = s[i * (size_t)channels];
        }
        r->fill += n;
        r->in_total += n;
        taken += n;
    }
    if (used) *used = taken;
    return made;
}

size_t ahresample_flush(ahresample *r, float *out, size_t out_room)
{
    if (r->copy) return 0;
    r->flushing = true;
    uint64_t limit = outputs_for(r, r->in_total);
    size_t made = 0;
    for (;;) {
        made += produce(r, out + made * (size_t)r->channels, out_room - made, limit);
        if (made == out_room || r->out_total >= limit) break;
        /* The last outputs reach past the end of the input: into silence. */
        compact(r);
        size_t n = r->cap - r->fill;
        if (!n) break;
        for (int ch = 0; ch < r->channels; ch++) memset(r->buf[ch] + r->fill, 0, n * sizeof(float));
        r->fill += n;
    }
    return made;
}
