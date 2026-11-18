/* This file follows the reference decoder in RFC 6716 (Appendix A), the
   normative definition of Opus, step by step, and so is under that
   code's licence:

   Copyright 1994-2011 IETF Trust, Xiph.Org, Skype Limited, Octasic,
                       Jean-Marc Valin, Timothy B. Terriberry,
                       CSIRO, Gregory Maxwell, Mark Borgerding,
                       Erik de Castro Lopo. All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   - Neither the name of Internet Society, IETF or IETF Trust, nor the
   names of specific contributors, may be used to endorse or promote
   products derived from this software without specific prior written
   permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
/* The CELT layer of Opus (RFC 6716 section 4.3): energy envelope, bit
   allocation, the pyramid vector quantizer, the low-overlap inverse MDCT,
   the pitch post-filter and de-emphasis. RFC 6716 defines CELT by its
   reference decoder; this code follows that decoder's arithmetic (the
   floating-point build) with the fixes of RFC 8251. */
#include "ahopus_internal.h"

#define BITRES 3
#define MAX_FINE_BITS 8
#define FINE_OFFSET 21
#define QTHETA_OFFSET 4
#define QTHETA_OFFSET_TWOPHASE 16
#define ALLOC_STEPS 6
#define LOG_MAX_PSEUDO 6
#define SPREAD_NONE 0
#define SPREAD_LIGHT 1
#define SPREAD_NORMAL 2
#define SPREAD_AGGRESSIVE 3
#define MIN_PERIOD 15
#define NB 21 /* bands */

static const uint8_t trim_icdf[11] = {126, 124, 119, 109, 87, 41, 19, 9, 4, 2, 0};
static const uint8_t spread_icdf[4] = {25, 23, 2, 0};
static const uint8_t tapset_icdf[3] = {2, 1, 0};
static const uint8_t small_energy_icdf[3] = {2, 1, 0};
static const signed char tf_select_table[4][8] = {
    {0, -1, 0, -1, 0, -1, 0, -1},
    {0, -1, 0, -2, 1, 0, 1, -1},
    {0, -2, 0, -3, 2, 0, 1, -1},
    {0, -2, 0, -3, 3, 0, 1, -1},
};
static const uint8_t log2_frac_table[24] = {0, 8, 13, 16, 19, 21, 23, 24, 26, 27, 28, 29,
                                            30, 31, 32, 32, 33, 34, 34, 35, 36, 36, 37, 37};
static const float pred_coef[4] = {29440 / 32768.f, 26112 / 32768.f, 21248 / 32768.f, 16384 / 32768.f};
static const float beta_coef[4] = {30147 / 32768.f, 22282 / 32768.f, 12124 / 32768.f, 6554 / 32768.f};
static const float beta_intra = 4915 / 32768.f;

static inline int imin(int a, int b) { return a < b ? a : b; }
static inline int imax(int a, int b) { return a > b ? a : b; }

/* (a * b) in Q15 with rounding, both taken as 16-bit values. */
static inline int frac_mul16(int a, int b)
{
    return (16384 + (int32_t)(int16_t)a * (int16_t)b) >> 15;
}

static inline float exp2f_(float x) { return (float)exp(0.6931471805599453094 * x); }

static uint32_t lcg_rand(uint32_t seed) { return 1664525u * seed + 1013904223u; }

/* ---- the transform -------------------------------------------------------- */

void ahopus_celt_setup_init(ahopus_celt_setup *s)
{
    for (int i = 0; i < AHOPUS_OVERLAP; i++) {
        double t = sin(0.5 * AHOPUS_PI * (i + 0.5) / AHOPUS_OVERLAP);
        s->window[i] = (float)sin(0.5 * AHOPUS_PI * t * t);
    }
    for (int k = 0; k < 480; k++) {
        s->tw_re[k] = (float)cos(2 * AHOPUS_PI * k / 480);
        s->tw_im[k] = (float)sin(2 * AHOPUS_PI * k / 480);
    }
    for (int shift = 0; shift < 4; shift++) {
        int n = 1920 >> shift;
        for (int k = 0; k < n / 4; k++) {
            s->trig_re[shift][k] = (float)cos(2 * AHOPUS_PI * (k + 0.125) / n);
            s->trig_im[shift][k] = (float)sin(2 * AHOPUS_PI * (k + 0.125) / n);
        }
    }
}

/* An inverse complex DFT (e^+i, no scaling) of n = 480 >> shift points by
   mixed radix: factors of 4, then 2, 3 and 5. */
static void fft_rec(const ahopus_celt_setup *s, float *out_re, float *out_im, const float *in_re, const float *in_im,
                    int n, int in_stride, int tw_stride)
{
    int p = n % 4 == 0 ? 4 : n % 2 == 0 ? 2 : n % 3 == 0 ? 3 : 5;
    int m = n / p;
    if (m == 1) {
        for (int q = 0; q < p; q++) {
            out_re[q] = in_re[q * in_stride];
            out_im[q] = in_im[q * in_stride];
        }
    } else {
        for (int q = 0; q < p; q++)
            fft_rec(s, out_re + q * m, out_im + q * m, in_re + q * in_stride, in_im + q * in_stride, m, in_stride * p, tw_stride * p);
    }
    /* Butterflies: out[u + k m] = sum over q of out[u + q m] w^(q u) e^(2 pi i q k / p). */
    for (int u = 0; u < m; u++) {
        float sr[5], si[5];
        for (int q = 0; q < p; q++) {
            float ar = out_re[u + q * m], ai = out_im[u + q * m];
            int t = (q * u * tw_stride) % 480;
            float wr = s->tw_re[t], wi = s->tw_im[t];
            sr[q] = ar * wr - ai * wi;
            si[q] = ar * wi + ai * wr;
        }
        for (int k = 0; k < p; k++) {
            float yr = 0, yi = 0;
            for (int q = 0; q < p; q++) {
                int t = (q * k * m * tw_stride) % 480;
                float wr = s->tw_re[t], wi = s->tw_im[t];
                yr += sr[q] * wr - si[q] * wi;
                yi += sr[q] * wi + si[q] * wr;
            }
            out_re[u + k * m] = yr;
            out_im[u + k * m] = yi;
        }
    }
}

/* One inverse MDCT of n2 coefficients (taken every `stride`) with CELT's low
   overlap window, overlap-added into x[0 .. n2 + overlap): the first
   `overlap` samples are added to, the rest written. */
static void imdct(const ahopus_celt_setup *s, const float *in, int stride, int n2, float *x)
{
    int n4 = n2 / 2, shift = n2 == 960 ? 0 : n2 == 480 ? 1 : n2 == 240 ? 2 : 3;
    const float *cr = s->trig_re[shift], *ci = s->trig_im[shift];
    const int ov = AHOPUS_OVERLAP;
    const float *w = s->window;
    float zr[480], zi[480], fr[480], fi[480], f2[960];
    /* Pre-rotate: -(x2 + i x1) e^(2 pi i (k + 1/8) / N) */
    for (int k = 0; k < n4; k++) {
        float x1 = in[2 * k * stride], x2 = in[(n2 - 1 - 2 * k) * stride];
        zr[k] = -(x2 * cr[k] - x1 * ci[k]);
        zi[k] = -(x2 * ci[k] + x1 * cr[k]);
    }
    fft_rec(s, fr, fi, zr, zi, n4, 1, 480 / n4);
    /* Post-rotate, and unfold into the n2 outputs. */
    for (int k = 0; k < n4; k++) {
        float yr = fr[k] * cr[k] - fi[k] * ci[k];
        float yi = fi[k] * cr[k] + fr[k] * ci[k];
        fr[k] = yr;
        fi[k] = yi;
    }
    for (int k = 0; k < n4; k++) {
        f2[2 * k] = -fr[k];
        f2[2 * k + 1] = fi[n4 - 1 - k];
    }
    /* Time domain aliasing cancellation over the overlap at each end. */
    for (int t = 0; t < ov / 2; t++) {
        float v = f2[t];
        x[ov / 2 - 1 - t] -= w[ov / 2 - 1 - t] * v;
        x[ov / 2 + t] += w[ov / 2 + t] * v;
    }
    for (int i = ov; i < n2; i++) x[i] = f2[i - ov / 2];
    for (int t = 0; t < ov / 2; t++) {
        float v = f2[n2 - ov / 2 + t];
        x[n2 + t] = w[ov - 1 - t] * v;
        x[n2 + ov - 1 - t] = w[t] * v;
    }
}

/* ---- range decoder helpers used only here --------------------------------- */

/* floor(sqrt(v)) */
static unsigned isqrt32(uint32_t v)
{
    unsigned g = 0;
    int bshift = (ahopus_ilog(v) - 1) >> 1;
    unsigned b = 1u << bshift;
    do {
        uint32_t t = (((uint32_t)g << 1) + b) << bshift;
        if (t <= v) {
            g += b;
            v -= t;
        }
        b >>= 1;
        bshift--;
    } while (bshift >= 0);
    return g;
}

/* ---- PVQ codewords (cwrs) ------------------------------------------------- */

/* The row U(n, 0 .. k+1) of the pulse-vector counting recurrence
   U(n, k) = U(n-1, k) + U(n, k-1) + U(n-1, k-1); returns V(n, k), the number
   of vectors of n integers whose magnitudes sum to k. */
static uint32_t pvq_row(int n, int k, uint32_t *u)
{
    int len = k + 2;
    u[0] = 0;
    u[1] = 1;
    for (int j = 2; j < len; j++) u[j] = (uint32_t)(2 * j - 1); /* row n = 2 */
    for (int row = 2; row < n; row++) {
        /* Next row in place, base case U(n, 1) = 1. */
        uint32_t ui0 = 1;
        uint32_t *v = u + 1;
        int j = 1;
        do {
            uint32_t ui1 = v[j] + v[j - 1] + ui0;
            v[j - 1] = ui0;
            ui0 = ui1;
        } while (++j < k + 1);
        v[j - 1] = ui0;
    }
    return u[k] + u[k + 1];
}

static void pvq_decode(int *y, int n, int k, ahopus_range *rd)
{
    uint32_t u[130];
    uint32_t i = ahopus_uint(rd, pvq_row(n, k, u));
    for (int j = 0; j < n; j++) {
        uint32_t p = u[k + 1];
        int s = -(i >= p);
        i -= p & (uint32_t)s;
        int yj = k;
        p = u[k];
        while (p > i) p = u[--k];
        i -= p;
        yj -= k;
        y[j] = (yj + s) ^ s;
        /* The row before, in place. */
        uint32_t ui0 = 0;
        int jj = 1;
        do {
            uint32_t ui1 = u[jj] - u[jj - 1] - ui0;
            u[jj - 1] = ui0;
            ui0 = ui1;
        } while (++jj < k + 2);
        u[jj - 1] = ui0;
    }
}

/* ---- the band shapes ---------------------------------------------------- */

static void exp_rotation1(float *x, int len, int stride, float c, float s)
{
    float *p = x;
    for (int i = 0; i < len - stride; i++) {
        float x1 = p[0], x2 = p[stride];
        p[stride] = c * x2 + s * x1;
        *p++ = c * x1 - s * x2;
    }
    p = &x[len - 2 * stride - 1];
    for (int i = len - 2 * stride - 1; i >= 0; i--) {
        float x1 = p[0], x2 = p[stride];
        p[stride] = c * x2 + s * x1;
        *p-- = c * x1 - s * x2;
    }
}

/* Spreading: rotations that smear the pulses over the band (decoder side). */
static void exp_rotation(float *x, int len, int stride, int k, int spread)
{
    static const int factors[3] = {15, 10, 5};
    if (2 * k >= len || spread == SPREAD_NONE) return;
    int factor = factors[spread - 1];
    float gain = (float)len / (float)(len + factor * k);
    float theta = 0.5f * (gain * gain);
    float c = (float)cos(0.5f * (float)AHOPUS_PI * theta);
    float s = (float)cos(0.5f * (float)AHOPUS_PI * (1.0f - theta));
    int stride2 = 0;
    if (len >= 8 * stride) {
        stride2 = 1;
        while ((stride2 * stride2 + stride2) * stride + (stride >> 2) < len) stride2++;
    }
    len /= stride;
    for (int i = 0; i < stride; i++) {
        if (stride2) exp_rotation1(x + i * len, len, stride2, s, c);
        exp_rotation1(x + i * len, len, 1, c, s);
    }
}

static void renormalise_vector(float *x, int n, float gain)
{
    float e = 1e-15f;
    for (int i = 0; i < n; i++) e += x[i] * x[i];
    float g = (1.f / (float)sqrt(e)) * gain;
    for (int i = 0; i < n; i++) x[i] *= g;
}

static unsigned alg_unquant(float *x, int n, int k, int spread, int b, ahopus_range *rd, float gain)
{
    int iy[176];
    pvq_decode(iy, n, k, rd);
    float ryy = 0;
    for (int i = 0; i < n; i++) ryy += (float)iy[i] * (float)iy[i];
    float g = (1.f / (float)sqrt(ryy)) * gain;
    for (int i = 0; i < n; i++) x[i] = g * (float)iy[i];
    exp_rotation(x, n, b, k, spread);
    if (b <= 1) return 1;
    unsigned mask = 0;
    int n0 = n / b;
    for (int i = 0; i < b; i++)
        for (int j = 0; j < n0; j++) mask |= (unsigned)(iy[i * n0 + j] != 0) << i;
    return mask;
}

static void haar1(float *x, int n0, int stride)
{
    n0 >>= 1;
    for (int i = 0; i < stride; i++)
        for (int j = 0; j < n0; j++) {
            float a = 0.70710678f * x[stride * 2 * j + i];
            float b = 0.70710678f * x[stride * (2 * j + 1) + i];
            x[stride * 2 * j + i] = a + b;
            x[stride * (2 * j + 1) + i] = a - b;
        }
}

/* Natural Hadamard order to "ordery" order, for stride 2, 4, 8 and 16. */
static const int ordery_table[] = {
    1, 0,
    3, 0, 2, 1,
    7, 0, 4, 3, 6, 1, 5, 2,
    15, 0, 8, 7, 12, 3, 11, 4, 14, 1, 9, 6, 13, 2, 10, 5,
};

static void deinterleave_hadamard(float *x, int n0, int stride, int hadamard)
{
    float tmp[176];
    int n = n0 * stride;
    const int *ordery = ordery_table + stride - 2;
    for (int i = 0; i < stride; i++)
        for (int j = 0; j < n0; j++) tmp[(hadamard ? ordery[i] : i) * n0 + j] = x[j * stride + i];
    memcpy(x, tmp, (size_t)n * sizeof(float));
}

static void interleave_hadamard(float *x, int n0, int stride, int hadamard)
{
    float tmp[176];
    int n = n0 * stride;
    const int *ordery = ordery_table + stride - 2;
    for (int i = 0; i < stride; i++)
        for (int j = 0; j < n0; j++) tmp[j * stride + i] = x[(hadamard ? ordery[i] : i) * n0 + j];
    memcpy(x, tmp, (size_t)n * sizeof(float));
}

/* A bit-exact cosine and log2(tan), as the allocation depends on them. */
static int bitexact_cos(int x)
{
    int32_t tmp = (4096 + (int32_t)x * x) >> 13;
    int x2 = (int16_t)tmp;
    x2 = (32767 - x2) + frac_mul16(x2, (-7651 + frac_mul16(x2, (8277 + frac_mul16(-626, x2)))));
    return 1 + x2;
}

static int bitexact_log2tan(int isin, int icos)
{
    int lc = ahopus_ilog((uint32_t)icos), ls = ahopus_ilog((uint32_t)isin);
    icos <<= 15 - lc;
    isin <<= 15 - ls;
    return (ls - lc) * (1 << 11) + frac_mul16(isin, frac_mul16(isin, -2597) + 7932) - frac_mul16(icos, frac_mul16(icos, -2597) + 7932);
}

static int compute_qn(int n, int b, int offset, int pulse_cap, int stereo)
{
    static const int16_t exp2_table8[8] = {16384, 17866, 19483, 21247, 23170, 25267, 27554, 30048};
    int n2 = 2 * n - 1;
    if (stereo && n == 2) n2--;
    int qb = imin(b - pulse_cap - (4 << BITRES), (b + n2 * offset) / n2);
    qb = imin(8 << BITRES, qb);
    if (qb < (1 << BITRES >> 1)) return 1;
    int qn = exp2_table8[qb & 7] >> (14 - (qb >> BITRES));
    return (qn + 1) >> 1 << 1;
}

static const uint8_t *cache_for(int lm, int band)
{
    return ahopus_cache_bits + ahopus_cache_index[(lm + 1) * NB + band];
}

static int bits2pulses(int band, int lm, int bits)
{
    const uint8_t *cache = cache_for(lm, band);
    int lo = 0, hi = cache[0];
    bits--;
    for (int i = 0; i < LOG_MAX_PSEUDO; i++) {
        int mid = (lo + hi + 1) >> 1;
        if (cache[mid] >= bits) hi = mid;
        else lo = mid;
    }
    return bits - (lo == 0 ? -1 : cache[lo]) <= cache[hi] - bits ? lo : hi;
}

static int pulses2bits(int band, int lm, int pulses)
{
    return pulses == 0 ? 0 : cache_for(lm, band)[pulses] + 1;
}

static int get_pulses(int i)
{
    return i < 8 ? i : (8 + (i & 7)) << ((i >> 3) - 1);
}

static void stereo_merge(float *x, float *y, float mid, int n)
{
    float xp = 0, side = 0;
    for (int j = 0; j < n; j++) {
        xp += x[j] * y[j];
        side += y[j] * y[j];
    }
    xp = mid * xp;
    float mid2 = mid;
    float el = mid2 * mid2 + side - 2 * xp;
    float er = mid2 * mid2 + side + 2 * xp;
    if (er < 6e-4f || el < 6e-4f) {
        memcpy(y, x, (size_t)n * sizeof(float));
        return;
    }
    float lgain = 1.f / (float)sqrt(el), rgain = 1.f / (float)sqrt(er);
    for (int j = 0; j < n; j++) {
        float l = mid * x[j], r = y[j];
        x[j] = lgain * (l - r);
        y[j] = rgain * (l + r);
    }
}

typedef struct {
    ahopus_range *rd;
    int band, intensity, spread, tf_change;
    int32_t remaining_bits;
    uint32_t *seed;
    float *lowband_scratch;
} band_ctx;

/* One band, mono or stereo, perhaps split in two (recursively) with the
   energy ratio of the halves sent as an angle. Returns the collapse mask. */
static unsigned quant_band(band_ctx *ctx, float *x, float *y, int n, int b, int blocks, float *lowband, int lm,
                           float *lowband_out, int level, float gain, int fill)
{
    ahopus_range *rd = ctx->rd;
    const int i = ctx->band;
    int n0 = n, n_b = n, b0 = blocks, time_divide = 0, recombine = 0, inv = 0;
    int tf_change = ctx->tf_change;
    float mid = 0, side = 0;
    bool long_blocks = b0 == 1;
    unsigned cm = 0;
    n_b /= blocks;
    int n_b0 = n_b;
    bool split, stereo;
    split = stereo = y != NULL;

    if (n == 1) {
        float *p = x;
        int c = 0;
        do {
            int sign = 0;
            if (ctx->remaining_bits >= 1 << BITRES) {
                sign = (int)ahopus_bits(rd, 1);
                ctx->remaining_bits -= 1 << BITRES;
                b -= 1 << BITRES;
            }
            p[0] = sign ? -1.f : 1.f;
            p = y;
        } while (++c < 1 + stereo);
        if (lowband_out) lowband_out[0] = x[0];
        return 1;
    }

    if (!stereo && level == 0) {
        if (tf_change > 0) recombine = tf_change;
        if (lowband && (recombine || ((n_b & 1) == 0 && tf_change < 0) || b0 > 1)) {
            memcpy(ctx->lowband_scratch, lowband, (size_t)n * sizeof(float));
            lowband = ctx->lowband_scratch;
        }
        for (int k = 0; k < recombine; k++) {
            static const uint8_t bit_interleave_table[16] = {0, 1, 1, 1, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3};
            if (lowband) haar1(lowband, n >> k, 1 << k);
            fill = bit_interleave_table[fill & 0xF] | bit_interleave_table[fill >> 4] << 2;
        }
        blocks >>= recombine;
        n_b <<= recombine;
        while ((n_b & 1) == 0 && tf_change < 0) {
            if (lowband) haar1(lowband, n_b, blocks);
            fill |= fill << blocks;
            blocks <<= 1;
            n_b >>= 1;
            time_divide++;
            tf_change++;
        }
        b0 = blocks;
        n_b0 = n_b;
        if (b0 > 1 && lowband) deinterleave_hadamard(lowband, n_b >> recombine, b0 << recombine, long_blocks);
    }

    /* Split a band that would need more than its cache allows. */
    if (!stereo && lm != -1 && n > 2 && b > cache_for(lm, i)[cache_for(lm, i)[0]] + 12) {
        n >>= 1;
        y = x + n;
        split = true;
        lm -= 1;
        if (blocks == 1) fill = (fill & 1) | (fill << 1);
        blocks = (blocks + 1) >> 1;
    }

    if (split) {
        int itheta = 0, mbits, sbits, delta, imid, iside;
        int pulse_cap = ahopus_logn[i] + lm * (1 << BITRES);
        int offset = (pulse_cap >> 1) - (stereo && n == 2 ? QTHETA_OFFSET_TWOPHASE : QTHETA_OFFSET);
        int qn = compute_qn(n, b, offset, pulse_cap, stereo);
        if (stereo && i >= ctx->intensity) qn = 1;
        uint32_t tell = ahopus_tell_frac(rd);
        if (qn != 1) {
            if (stereo && n > 2) {
                /* A step: probability 3 up to the middle, 1 after. */
                int p0 = 3, x0 = qn / 2, ft = p0 * (x0 + 1) + x0, xv;
                int fs = (int)ahopus_decode(rd, (unsigned)ft);
                xv = fs < (x0 + 1) * p0 ? fs / p0 : x0 + 1 + (fs - (x0 + 1) * p0);
                ahopus_update(rd, (unsigned)(xv <= x0 ? p0 * xv : (xv - 1 - x0) + (x0 + 1) * p0),
                              (unsigned)(xv <= x0 ? p0 * (xv + 1) : (xv - x0) + (x0 + 1) * p0), (unsigned)ft);
                itheta = xv;
            } else if (b0 > 1 || stereo) {
                itheta = (int)ahopus_uint(rd, (uint32_t)qn + 1);
            } else {
                /* Triangular */
                int fs = 1, fl = 0, ft = ((qn >> 1) + 1) * ((qn >> 1) + 1);
                int fm = (int)ahopus_decode(rd, (unsigned)ft);
                if (fm < ((qn >> 1) * ((qn >> 1) + 1) >> 1)) {
                    itheta = ((int)isqrt32(8 * (uint32_t)fm + 1) - 1) >> 1;
                    fs = itheta + 1;
                    fl = itheta * (itheta + 1) >> 1;
                } else {
                    itheta = (2 * (qn + 1) - (int)isqrt32(8 * (uint32_t)(ft - fm - 1) + 1)) >> 1;
                    fs = qn + 1 - itheta;
                    fl = ft - ((qn + 1 - itheta) * (qn + 2 - itheta) >> 1);
                }
                ahopus_update(rd, (unsigned)fl, (unsigned)(fl + fs), (unsigned)ft);
            }
            itheta = (int32_t)itheta * 16384 / qn;
        } else if (stereo) {
            if (b > 2 << BITRES && ctx->remaining_bits > 2 << BITRES) inv = ahopus_bit_logp(rd, 2);
            else inv = 0;
            itheta = 0;
        }
        int qalloc = (int)(ahopus_tell_frac(rd) - tell);
        b -= qalloc;

        int orig_fill = fill;
        if (itheta == 0) {
            imid = 32767;
            iside = 0;
            fill &= (1 << blocks) - 1;
            delta = -16384;
        } else if (itheta == 16384) {
            imid = 0;
            iside = 32767;
            fill &= ((1 << blocks) - 1) << blocks;
            delta = 16384;
        } else {
            imid = bitexact_cos(itheta);
            iside = bitexact_cos(16384 - itheta);
            delta = frac_mul16((n - 1) << 7, bitexact_log2tan(iside, imid));
        }
        mid = (1.f / 32768) * (float)imid;
        side = (1.f / 32768) * (float)iside;

        if (n == 2 && stereo) {
            /* The side of a two-line stereo band is one sign bit. */
            int sign = 0;
            mbits = b;
            sbits = itheta != 0 && itheta != 16384 ? 1 << BITRES : 0;
            mbits -= sbits;
            int c = itheta > 8192;
            ctx->remaining_bits -= qalloc + sbits;
            float *x2 = c ? y : x, *y2 = c ? x : y;
            if (sbits) sign = (int)ahopus_bits(rd, 1);
            sign = 1 - 2 * sign;
            cm = quant_band(ctx, x2, NULL, n, mbits, blocks, lowband, lm, lowband_out, level, gain, orig_fill);
            y2[0] = (float)-sign * x2[1];
            y2[1] = (float)sign * x2[0];
            x[0] *= mid;
            x[1] *= mid;
            y[0] *= side;
            y[1] *= side;
            float tmp = x[0];
            x[0] = tmp - y[0];
            y[0] = tmp + y[0];
            tmp = x[1];
            x[1] = tmp - y[1];
            y[1] = tmp + y[1];
        } else {
            float *next_lowband2 = NULL, *next_lowband_out1 = NULL;
            int next_level = 0;
            if (b0 > 1 && !stereo && (itheta & 0x3fff)) {
                if (itheta > 8192) delta -= delta >> (4 - lm);
                else delta = imin(0, delta + (n << BITRES >> (5 - lm)));
            }
            mbits = imax(0, imin(b, (b - delta) / 2));
            sbits = b - mbits;
            ctx->remaining_bits -= qalloc;
            if (lowband && !stereo) next_lowband2 = lowband + n;
            if (stereo) next_lowband_out1 = lowband_out;
            else next_level = level + 1;
            int32_t rebalance = ctx->remaining_bits;
            int shift_side = (b0 >> 1) & (stereo - 1);
            if (mbits >= sbits) {
                cm = quant_band(ctx, x, NULL, n, mbits, blocks, lowband, lm, next_lowband_out1, next_level, stereo ? 1.f : gain * mid, fill);
                rebalance = mbits - (rebalance - ctx->remaining_bits);
                if (rebalance > 3 << BITRES && itheta != 0) sbits += rebalance - (3 << BITRES);
                cm |= quant_band(ctx, y, NULL, n, sbits, blocks, next_lowband2, lm, NULL, next_level, gain * side, fill >> blocks) << shift_side;
            } else {
                cm = quant_band(ctx, y, NULL, n, sbits, blocks, next_lowband2, lm, NULL, next_level, gain * side, fill >> blocks) << shift_side;
                rebalance = sbits - (rebalance - ctx->remaining_bits);
                if (rebalance > 3 << BITRES && itheta != 16384) mbits += rebalance - (3 << BITRES);
                cm |= quant_band(ctx, x, NULL, n, mbits, blocks, lowband, lm, next_lowband_out1, next_level, stereo ? 1.f : gain * mid, fill);
            }
        }
    } else {
        /* No split: the pulses themselves. */
        int q = bits2pulses(i, lm, b);
        int curr_bits = pulses2bits(i, lm, q);
        ctx->remaining_bits -= curr_bits;
        while (ctx->remaining_bits < 0 && q > 0) {
            ctx->remaining_bits += curr_bits;
            q--;
            curr_bits = pulses2bits(i, lm, q);
            ctx->remaining_bits -= curr_bits;
        }
        if (q != 0) {
            cm = alg_unquant(x, n, get_pulses(q), ctx->spread, blocks, rd, gain);
        } else {
            unsigned cm_mask = (unsigned)(1ul << blocks) - 1;
            fill &= (int)cm_mask;
            if (!fill) {
                memset(x, 0, (size_t)n * sizeof(float));
            } else {
                if (lowband == NULL) {
                    for (int j = 0; j < n; j++) {
                        *ctx->seed = lcg_rand(*ctx->seed);
                        x[j] = (float)((int32_t)*ctx->seed >> 20);
                    }
                    cm = cm_mask;
                } else {
                    for (int j = 0; j < n; j++) {
                        *ctx->seed = lcg_rand(*ctx->seed);
                        float tmp = 1.0f / 256;
                        x[j] = lowband[j] + ((*ctx->seed) & 0x8000 ? tmp : -tmp);
                    }
                    cm = (unsigned)fill;
                }
                renormalise_vector(x, n, gain);
            }
        }
    }

    if (stereo) {
        if (n != 2) stereo_merge(x, y, mid, n);
        if (inv)
            for (int j = 0; j < n; j++) y[j] = -y[j];
    } else if (level == 0) {
        if (b0 > 1) interleave_hadamard(x, n_b >> recombine, b0 << recombine, long_blocks);
        n_b = n_b0;
        blocks = b0;
        for (int k = 0; k < time_divide; k++) {
            blocks >>= 1;
            n_b <<= 1;
            cm |= cm >> blocks;
            haar1(x, n_b, blocks);
        }
        for (int k = 0; k < recombine; k++) {
            static const uint8_t bit_deinterleave_table[16] = {0x00, 0x03, 0x0C, 0x0F, 0x30, 0x33, 0x3C, 0x3F,
                                                               0xC0, 0xC3, 0xCC, 0xCF, 0xF0, 0xF3, 0xFC, 0xFF};
            cm = bit_deinterleave_table[cm];
            haar1(x, n0 >> k, 1 << k);
        }
        blocks <<= recombine;
        if (lowband_out) {
            float nn = (float)sqrt((double)n0);
            for (int j = 0; j < n0; j++) lowband_out[j] = nn * x[j];
        }
        cm &= (1u << blocks) - 1;
    }
    return cm;
}

static void quant_all_bands(ahopus_celt *st, float *x_, float *y_, uint8_t *collapse_masks, const int *pulses, int short_blocks,
                            int spread, int dual_stereo, int intensity, const int *tf_res, int32_t total_bits, int32_t balance,
                            ahopus_range *rd, int lm, int coded_bands)
{
    const short *eb = ahopus_ebands;
    const int start = st->start, end = st->end, m = 1 << lm, c = y_ ? 2 : 1;
    const int blocks = short_blocks ? m : 1;
    float norm[2 * 8 * 100], scratch[8 * 22];
    float *norm2 = norm + m * eb[NB];
    int lowband_offset = 0;
    bool update_lowband = true;
    band_ctx ctx = {rd, 0, intensity, spread, 0, 0, &st->rng, scratch};

    for (int i = start; i < end; i++) {
        float *x = x_ + m * eb[i], *y = y_ ? y_ + m * eb[i] : NULL;
        int n = m * eb[i + 1] - m * eb[i];
        int32_t tell = (int32_t)ahopus_tell_frac(rd);
        int b, effective_lowband = -1;
        unsigned x_cm, y_cm;
        if (i != start) balance -= tell;
        ctx.remaining_bits = total_bits - tell - 1;
        if (i <= coded_bands - 1) {
            int32_t curr_balance = balance / imin(3, coded_bands - i);
            b = imax(0, imin(16383, imin(ctx.remaining_bits + 1, pulses[i] + curr_balance)));
        } else {
            b = 0;
        }
        if ((m * eb[i] - n >= m * eb[start] || i == start + 1) && (update_lowband || lowband_offset == 0)) lowband_offset = i;
        if (i == start + 1) {
            /* RFC 8251: repeat part of the first band so the second can fold. */
            int n1 = m * (eb[start + 1] - eb[start]), n2 = m * (eb[start + 2] - eb[start + 1]), offset = m * eb[start];
            if (n2 > n1) {
                memmove(&norm[offset + n1], &norm[offset + 2 * n1 - n2], (size_t)(n2 - n1) * sizeof(float));
                if (c == 2) memmove(&norm2[offset + n1], &norm2[offset + 2 * n1 - n2], (size_t)(n2 - n1) * sizeof(float));
            }
        }
        ctx.band = i;
        ctx.tf_change = tf_res[i];
        if (lowband_offset != 0 && (spread != SPREAD_AGGRESSIVE || blocks > 1 || ctx.tf_change < 0)) {
            effective_lowband = imax(m * eb[start], m * eb[lowband_offset] - n);
            int fold_start = lowband_offset;
            while (m * eb[--fold_start] > effective_lowband) {}
            int fold_end = lowband_offset - 1;
            while (++fold_end < i && m * eb[fold_end] < effective_lowband + n) {}
            x_cm = y_cm = 0;
            int fold_i = fold_start;
            do {
                x_cm |= collapse_masks[fold_i * c + 0];
                y_cm |= collapse_masks[fold_i * c + c - 1];
            } while (++fold_i < fold_end);
        } else {
            x_cm = y_cm = (1u << blocks) - 1;
        }
        if (dual_stereo && i == intensity) {
            dual_stereo = 0;
            for (int j = m * eb[start]; j < m * eb[i]; j++) norm[j] = 0.5f * (norm[j] + norm2[j]);
        }
        if (dual_stereo) {
            x_cm = quant_band(&ctx, x, NULL, n, b / 2, blocks, effective_lowband != -1 ? norm + effective_lowband : NULL, lm,
                              norm + m * eb[i], 0, 1.f, (int)x_cm);
            y_cm = quant_band(&ctx, y, NULL, n, b / 2, blocks, effective_lowband != -1 ? norm2 + effective_lowband : NULL, lm,
                              norm2 + m * eb[i], 0, 1.f, (int)y_cm);
        } else {
            x_cm = quant_band(&ctx, x, y, n, b, blocks, effective_lowband != -1 ? norm + effective_lowband : NULL, lm,
                              norm + m * eb[i], 0, 1.f, (int)(x_cm | y_cm));
            y_cm = x_cm;
        }
        collapse_masks[i * c + 0] = (uint8_t)x_cm;
        collapse_masks[i * c + c - 1] = (uint8_t)y_cm;
        balance += pulses[i] + tell;
        update_lowband = b > (n << BITRES);
    }
}

/* ---- energy ---------------------------------------------------------------- */

static void unquant_coarse_energy(ahopus_celt *st, int intra, ahopus_range *rd, int c_count, int lm)
{
    const uint8_t *prob = ahopus_e_prob_model[lm][intra];
    float prev[2] = {0, 0};
    float coef = intra ? 0 : pred_coef[lm], beta = intra ? beta_intra : beta_coef[lm];
    int32_t budget = (int32_t)rd->storage * 8;
    for (int i = st->start; i < st->end; i++) {
        for (int c = 0; c < c_count; c++) {
            int qi;
            int32_t tell = ahopus_tell(rd);
            if (budget - tell >= 15) {
                int pi = 2 * imin(i, 20);
                qi = ahopus_laplace(rd, (unsigned)prob[pi] << 7, prob[pi + 1] << 6);
            } else if (budget - tell >= 2) {
                qi = ahopus_icdf(rd, small_energy_icdf, 2);
                qi = (qi >> 1) ^ -(qi & 1);
            } else if (budget - tell >= 1) {
                qi = -ahopus_bit_logp(rd, 1);
            } else {
                qi = -1;
            }
            float q = (float)qi;
            float *e = &st->old_band_e[i + c * NB];
            if (*e < -9.f) *e = -9.f;
            float tmp = coef * *e + prev[c] + q;
            *e = tmp;
            prev[c] = prev[c] + q - beta * q;
        }
    }
}

static void unquant_fine_energy(ahopus_celt *st, const int *fine_quant, ahopus_range *rd, int c_count)
{
    for (int i = st->start; i < st->end; i++) {
        if (fine_quant[i] <= 0) continue;
        for (int c = 0; c < c_count; c++) {
            int q2 = (int)ahopus_bits(rd, (unsigned)fine_quant[i]);
            float offset = ((float)q2 + .5f) * (float)(1 << (14 - fine_quant[i])) * (1.f / 16384) - .5f;
            st->old_band_e[i + c * NB] += offset;
        }
    }
}

static void unquant_energy_finalise(ahopus_celt *st, const int *fine_quant, const int *fine_priority, int bits_left,
                                    ahopus_range *rd, int c_count)
{
    for (int prio = 0; prio < 2; prio++) {
        for (int i = st->start; i < st->end && bits_left >= c_count; i++) {
            if (fine_quant[i] >= MAX_FINE_BITS || fine_priority[i] != prio) continue;
            for (int c = 0; c < c_count; c++) {
                int q2 = (int)ahopus_bits(rd, 1);
                float offset = ((float)q2 - .5f) * (float)(1 << (14 - fine_quant[i] - 1)) * (1.f / 16384);
                st->old_band_e[i + c * NB] += offset;
                bits_left--;
            }
        }
    }
}

/* ---- allocation -------------------------------------------------------------- */

static int interp_bits2pulses(ahopus_celt *st, int skip_start, const int *bits1, const int *bits2, const int *thresh,
                              const int *cap, int32_t total, int32_t *out_balance, int skip_rsv, int *intensity,
                              int intensity_rsv, int *dual_stereo, int dual_stereo_rsv, int *bits, int *ebits,
                              int *fine_priority, int c_count, int lm, ahopus_range *rd)
{
    const short *eb = ahopus_ebands;
    const int start = st->start, end = st->end;
    int alloc_floor = c_count << BITRES, stereo = c_count > 1, log_m = lm << BITRES;
    int lo = 0, hi = 1 << ALLOC_STEPS, coded_bands;
    int32_t psum;
    for (int i = 0; i < ALLOC_STEPS; i++) {
        int mid = (lo + hi) >> 1;
        bool done = false;
        psum = 0;
        for (int j = end; j-- > start;) {
            int tmp = bits1[j] + (mid * (int32_t)bits2[j] >> ALLOC_STEPS);
            if (tmp >= thresh[j] || done) {
                done = true;
                psum += imin(tmp, cap[j]);
            } else if (tmp >= alloc_floor) {
                psum += alloc_floor;
            }
        }
        if (psum > total) hi = mid;
        else lo = mid;
    }
    psum = 0;
    bool done = false;
    for (int j = end; j-- > start;) {
        int tmp = bits1[j] + (lo * bits2[j] >> ALLOC_STEPS);
        if (tmp < thresh[j] && !done) tmp = tmp >= alloc_floor ? alloc_floor : 0;
        else done = true;
        tmp = imin(tmp, cap[j]);
        bits[j] = tmp;
        psum += tmp;
    }
    /* Bands to skip, from the top. */
    for (coded_bands = end;; coded_bands--) {
        int j = coded_bands - 1;
        if (j <= skip_start) {
            total += skip_rsv;
            break;
        }
        int32_t left = total - psum;
        int32_t percoeff = left / (eb[coded_bands] - eb[start]);
        left -= (eb[coded_bands] - eb[start]) * percoeff;
        int rem = imax((int)left - (eb[j] - eb[start]), 0);
        int band_width = eb[coded_bands] - eb[j];
        int band_bits = (int)(bits[j] + percoeff * band_width + rem);
        if (band_bits >= imax(thresh[j], alloc_floor + (1 << BITRES))) {
            if (ahopus_bit_logp(rd, 1)) break;
            psum += 1 << BITRES;
            band_bits -= 1 << BITRES;
        }
        psum -= bits[j] + intensity_rsv;
        if (intensity_rsv > 0) intensity_rsv = log2_frac_table[j - start];
        psum += intensity_rsv;
        if (band_bits >= alloc_floor) {
            psum += alloc_floor;
            bits[j] = alloc_floor;
        } else {
            bits[j] = 0;
        }
    }
    if (intensity_rsv > 0) *intensity = start + (int)ahopus_uint(rd, (uint32_t)(coded_bands + 1 - start));
    else *intensity = 0;
    if (*intensity <= start) {
        total += dual_stereo_rsv;
        dual_stereo_rsv = 0;
    }
    *dual_stereo = dual_stereo_rsv > 0 ? ahopus_bit_logp(rd, 1) : 0;

    int32_t left = total - psum;
    int32_t percoeff = left / (eb[coded_bands] - eb[start]);
    left -= (eb[coded_bands] - eb[start]) * percoeff;
    for (int j = start; j < coded_bands; j++) bits[j] += (int)percoeff * (eb[j + 1] - eb[j]);
    for (int j = start; j < coded_bands; j++) {
        int tmp = (int)(left < eb[j + 1] - eb[j] ? left : eb[j + 1] - eb[j]);
        bits[j] += tmp;
        left -= tmp;
    }

    int balance = 0, j;
    for (j = start; j < coded_bands; j++) {
        int n0 = eb[j + 1] - eb[j], n = n0 << lm, excess;
        bits[j] += balance;
        if (n > 1) {
            excess = imax(bits[j] - cap[j], 0);
            bits[j] -= excess;
            int den = c_count * n + ((c_count == 2 && n > 2 && !*dual_stereo && j < *intensity) ? 1 : 0);
            int nclogn = den * (ahopus_logn[j] + log_m);
            int offset = (nclogn >> 1) - den * FINE_OFFSET;
            if (n == 2) offset += den << BITRES >> 2;
            if (bits[j] + offset < den * 2 << BITRES) offset += nclogn >> 2;
            else if (bits[j] + offset < den * 3 << BITRES) offset += nclogn >> 3;
            ebits[j] = imax(0, (bits[j] + offset + (den << (BITRES - 1))) / (den << BITRES));
            if (c_count * ebits[j] > (bits[j] >> BITRES)) ebits[j] = bits[j] >> stereo >> BITRES;
            ebits[j] = imin(ebits[j], MAX_FINE_BITS);
            fine_priority[j] = ebits[j] * (den << BITRES) >= bits[j] + offset;
            bits[j] -= c_count * ebits[j] << BITRES;
        } else {
            excess = imax(0, bits[j] - (c_count << BITRES));
            bits[j] -= excess;
            ebits[j] = 0;
            fine_priority[j] = 1;
        }
        if (excess > 0) {
            int extra_fine = imin(excess >> (stereo + BITRES), MAX_FINE_BITS - ebits[j]);
            ebits[j] += extra_fine;
            int extra_bits = extra_fine * c_count << BITRES;
            fine_priority[j] = extra_bits >= excess - balance;
            excess -= extra_bits;
        }
        balance = excess;
    }
    *out_balance = balance;
    for (; j < end; j++) {
        ebits[j] = bits[j] >> stereo >> BITRES;
        bits[j] = 0;
        fine_priority[j] = ebits[j] < 1;
    }
    return coded_bands;
}

static int compute_allocation(ahopus_celt *st, const int *offsets, const int *cap, int alloc_trim, int *intensity,
                              int *dual_stereo, int32_t total, int32_t *balance, int *pulses, int *ebits,
                              int *fine_priority, int c_count, int lm, ahopus_range *rd)
{
    const short *eb = ahopus_ebands;
    const int start = st->start, end = st->end;
    int bits1[NB], bits2[NB], thresh[NB], trim_offset[NB];
    int skip_start = start, intensity_rsv = 0, dual_stereo_rsv = 0;
    total = total > 0 ? total : 0;
    int skip_rsv = total >= 1 << BITRES ? 1 << BITRES : 0;
    total -= skip_rsv;
    if (c_count == 2) {
        intensity_rsv = log2_frac_table[end - start];
        if (intensity_rsv > total) {
            intensity_rsv = 0;
        } else {
            total -= intensity_rsv;
            dual_stereo_rsv = total >= 1 << BITRES ? 1 << BITRES : 0;
            total -= dual_stereo_rsv;
        }
    }
    for (int j = start; j < end; j++) {
        thresh[j] = imax(c_count << BITRES, (3 * (eb[j + 1] - eb[j]) << lm << BITRES) >> 4);
        trim_offset[j] = c_count * (eb[j + 1] - eb[j]) * (alloc_trim - 5 - lm) * (end - j - 1) * (1 << (lm + BITRES)) >> 6;
        if ((eb[j + 1] - eb[j]) << lm == 1) trim_offset[j] -= c_count << BITRES;
    }
    int lo = 1, hi = 11 - 1;
    do {
        bool done = false;
        int psum = 0, mid = (lo + hi) >> 1;
        for (int j = end; j-- > start;) {
            int n = eb[j + 1] - eb[j];
            int bitsj = c_count * n * ahopus_band_allocation[mid * NB + j] << lm >> 2;
            if (bitsj > 0) bitsj = imax(0, bitsj + trim_offset[j]);
            bitsj += offsets[j];
            if (bitsj >= thresh[j] || done) {
                done = true;
                psum += imin(bitsj, cap[j]);
            } else if (bitsj >= c_count << BITRES) {
                psum += c_count << BITRES;
            }
        }
        if (psum > total) hi = mid - 1;
        else lo = mid + 1;
    } while (lo <= hi);
    hi = lo--;
    for (int j = start; j < end; j++) {
        int n = eb[j + 1] - eb[j];
        int bits1j = c_count * n * ahopus_band_allocation[lo * NB + j] << lm >> 2;
        int bits2j = hi >= 11 ? cap[j] : c_count * n * ahopus_band_allocation[hi * NB + j] << lm >> 2;
        if (bits1j > 0) bits1j = imax(0, bits1j + trim_offset[j]);
        if (bits2j > 0) bits2j = imax(0, bits2j + trim_offset[j]);
        if (lo > 0) bits1j += offsets[j];
        bits2j += offsets[j];
        if (offsets[j] > 0) skip_start = j;
        bits2j = imax(0, bits2j - bits1j);
        bits1[j] = bits1j;
        bits2[j] = bits2j;
    }
    return interp_bits2pulses(st, skip_start, bits1, bits2, thresh, cap, total, balance, skip_rsv, intensity, intensity_rsv,
                              dual_stereo, dual_stereo_rsv, pulses, ebits, fine_priority, c_count, lm, rd);
}

/* ---- after the bands ------------------------------------------------------- */

static void anti_collapse(ahopus_celt *st, float *x_, const uint8_t *collapse_masks, int lm, int c_count, int size,
                          const int *pulses, uint32_t seed)
{
    const short *eb = ahopus_ebands;
    for (int i = st->start; i < st->end; i++) {
        int n0 = eb[i + 1] - eb[i];
        int depth = (1 + pulses[i]) / (n0 << lm);
        float thresh = .5f * exp2f_(-.125f * (float)depth);
        float sqrt_1 = 1.f / (float)sqrt((double)(n0 << lm));
        for (int c = 0; c < c_count; c++) {
            float prev1 = st->old_log_e[c * NB + i], prev2 = st->old_log_e2[c * NB + i];
            if (c_count == 1) {
                if (st->old_log_e[NB + i] > prev1) prev1 = st->old_log_e[NB + i];
                if (st->old_log_e2[NB + i] > prev2) prev2 = st->old_log_e2[NB + i];
            }
            float ediff = st->old_band_e[c * NB + i] - (prev1 < prev2 ? prev1 : prev2);
            if (ediff < 0) ediff = 0;
            float r = 2.f * exp2f_(-ediff);
            if (lm == 3) r *= 1.41421356f;
            if (thresh < r) r = thresh;
            r = r * sqrt_1;
            float *x = x_ + c * size + (eb[i] << lm);
            bool renormalize = false;
            for (int k = 0; k < 1 << lm; k++) {
                if (!(collapse_masks[i * c_count + c] & 1 << k)) {
                    for (int j = 0; j < n0; j++) {
                        seed = lcg_rand(seed);
                        x[(j << lm) + k] = seed & 0x8000 ? r : -r;
                    }
                    renormalize = true;
                }
            }
            if (renormalize) renormalise_vector(x, n0 << lm, 1.f);
        }
    }
}

/* Band shapes times their energies, the energies kept as log2 amplitudes. */
static void denormalise_bands(const ahopus_celt *st, const float *x, float *freq, int end, int c_count, int m, bool silence)
{
    const short *eb = ahopus_ebands;
    int n = m * 120;
    for (int c = 0; c < c_count; c++) {
        float *f = freq + c * n;
        const float *xx = x + c * n;
        memset(f, 0, (size_t)n * sizeof(float));
        if (silence) continue;
        for (int i = st->start; i < end; i++) {
            float lg = st->old_band_e[i + c * NB] + ahopus_emeans[i];
            if (lg > 32.f) lg = 32.f; /* RFC 8251 */
            float g = exp2f_(lg);
            for (int j = m * eb[i]; j < m * eb[i + 1]; j++) f[j] = xx[j] * g;
        }
    }
}

/* The pitch post-filter, a comb filter run in place over x (which has
   history before it), cross-fading from the old settings to the new over
   the window's overlap. */
static void comb_filter(float *x, int t0, int t1, int n, float g0, float g1, int tapset0, int tapset1, const float *window, int overlap)
{
    static const float gains[3][3] = {{0.3066406250f, 0.2170410156f, 0.1296386719f},
                                      {0.4638671875f, 0.2680664062f, 0.f},
                                      {0.7998046875f, 0.1000976562f, 0.f}};
    if (g0 == 0 && g1 == 0) return;
    float g00 = g0 * gains[tapset0][0], g01 = g0 * gains[tapset0][1], g02 = g0 * gains[tapset0][2];
    float g10 = g1 * gains[tapset1][0], g11 = g1 * gains[tapset1][1], g12 = g1 * gains[tapset1][2];
    int i = 0;
    if (window) {
        for (; i < overlap && i < n; i++) {
            float f = window[i] * window[i];
            x[i] = x[i] + ((1 - f) * g00) * x[i - t0] + ((1 - f) * g01) * x[i - t0 - 1] + ((1 - f) * g01) * x[i - t0 + 1] +
                   ((1 - f) * g02) * x[i - t0 - 2] + ((1 - f) * g02) * x[i - t0 + 2] + (f * g10) * x[i - t1] +
                   (f * g11) * x[i - t1 - 1] + (f * g11) * x[i - t1 + 1] + (f * g12) * x[i - t1 - 2] + (f * g12) * x[i - t1 + 2];
        }
    }
    for (; i < n; i++)
        x[i] = x[i] + g10 * x[i - t1] + g11 * x[i - t1 - 1] + g11 * x[i - t1 + 1] + g12 * x[i - t1 - 2] + g12 * x[i - t1 + 2];
}

static void deemphasis(ahopus_celt *st, float *pcm, int n)
{
    const int cc = st->channels;
    for (int c = 0; c < cc; c++) {
        const float *x = st->decode_mem[c] + AHOPUS_CELT_BUF - n;
        float m = st->preemph_mem[c];
        for (int j = 0; j < n; j++) {
            float tmp = x[j] + m;
            m = 0.8500061035f * tmp;
            pcm[j * cc + c] = tmp * (1.f / 32768);
        }
        st->preemph_mem[c] = m;
    }
}

/* ---- state ------------------------------------------------------------------- */

void ahopus_celt_reset(ahopus_celt *c)
{
    memset(c->decode_mem, 0, sizeof(c->decode_mem));
    memset(c->old_band_e, 0, sizeof(c->old_band_e));
    memset(c->background_log_e, 0, sizeof(c->background_log_e));
    for (int i = 0; i < 2 * NB; i++) c->old_log_e[i] = c->old_log_e2[i] = -28.f;
    c->preemph_mem[0] = c->preemph_mem[1] = 0;
    c->postfilter_period = c->postfilter_period_old = 0;
    c->postfilter_tapset = c->postfilter_tapset_old = 0;
    c->postfilter_gain = c->postfilter_gain_old = 0;
    c->rng = 0;
    c->loss_count = 0;
    c->last_pitch_index = 0;
    c->skip_plc = 1;
    memset(c->lpc, 0, sizeof(c->lpc));
}

void ahopus_celt_init(ahopus_celt *c, const ahopus_celt_setup *setup, int channels)
{
    c->setup = setup;
    c->channels = c->stream_channels = channels;
    c->start = 0;
    c->end = NB;
    ahopus_celt_reset(c);
}

/* Inverse transforms of a frame into decode_mem, which has first been moved
   along by n. */
static void synthesize(ahopus_celt *st, const float *freq, int short_blocks, int lm)
{
    const int n = 120 << lm, cc = st->channels;
    for (int c = 0; c < cc; c++) {
        float *mem = st->decode_mem[c];
        memmove(mem, mem + n, (size_t)(AHOPUS_CELT_BUF - n) * sizeof(float));
        /* The held overlap moves to the start of this frame's output. */
        float *out = mem + AHOPUS_CELT_BUF - n;
        memmove(out, mem + AHOPUS_CELT_BUF, AHOPUS_OVERLAP * sizeof(float));
        int n2 = short_blocks ? 120 : n, blocks = short_blocks ? short_blocks : 1;
        for (int b = 0; b < blocks; b++) imdct(st->setup, freq + c * n + b, blocks, n2, out + n2 * b);
    }
}

static void post_filter(ahopus_celt *st, int lm, int pitch, float gain, int tapset)
{
    const int n = 120 << lm;
    const float *window = st->setup->window;
    if (st->postfilter_period < MIN_PERIOD) st->postfilter_period = MIN_PERIOD;
    if (st->postfilter_period_old < MIN_PERIOD) st->postfilter_period_old = MIN_PERIOD;
    for (int c = 0; c < st->channels; c++) {
        float *out = st->decode_mem[c] + AHOPUS_CELT_BUF - n;
        comb_filter(out, st->postfilter_period_old, st->postfilter_period, 120, st->postfilter_gain_old, st->postfilter_gain,
                    st->postfilter_tapset_old, st->postfilter_tapset, window, AHOPUS_OVERLAP);
        if (lm != 0)
            comb_filter(out + 120, st->postfilter_period, pitch < MIN_PERIOD ? MIN_PERIOD : pitch, n - 120, st->postfilter_gain,
                        gain, st->postfilter_tapset, tapset, window, AHOPUS_OVERLAP);
    }
    st->postfilter_period_old = st->postfilter_period;
    st->postfilter_gain_old = st->postfilter_gain;
    st->postfilter_tapset_old = st->postfilter_tapset;
    st->postfilter_period = pitch;
    st->postfilter_gain = gain;
    st->postfilter_tapset = tapset;
    if (lm != 0) {
        st->postfilter_period_old = st->postfilter_period;
        st->postfilter_gain_old = st->postfilter_gain;
        st->postfilter_tapset_old = st->postfilter_tapset;
    }
}

/* A lost frame: noise shaped by the last band energies, fading by 1.5 dB
   and then 0.5 dB a frame down to the background level. */
static void decode_lost(ahopus_celt *st, float *pcm, int lm)
{
    const int n = 120 << lm, m = 1 << lm, cc = st->channels;
    const short *eb = ahopus_ebands;
    const int eff_end = st->end;
    float x[2 * 960], freq[2 * 960];
    uint32_t seed = st->rng;
    float decay = st->loss_count == 0 ? 1.5f : .5f;
    for (int c = 0; c < cc; c++) {
        for (int i = st->start; i < st->end; i++) {
            float *e = &st->old_band_e[c * NB + i], v = *e - decay, bg = st->background_log_e[c * NB + i];
            *e = bg > v ? bg : v;
        }
        memset(x + c * n, 0, (size_t)n * sizeof(float));
        for (int i = st->start; i < eff_end; i++) {
            int off = c * n + m * eb[i], len = m * (eb[i + 1] - eb[i]);
            for (int j = 0; j < len; j++) {
                seed = lcg_rand(seed);
                x[off + j] = (float)((int32_t)seed >> 20);
            }
            renormalise_vector(x + off, len, 1.f);
        }
    }
    st->rng = seed;
    denormalise_bands(st, x, freq, eff_end, cc, m, false);
    synthesize(st, freq, 0, lm);
    deemphasis(st, pcm, n);
    st->loss_count++;
}

bool ahopus_celt_decode(ahopus_celt *st, const uint8_t *data, int len, float *pcm, int frame_size, ahopus_range *rd)
{
    const short *eb = ahopus_ebands;
    const int cc = st->channels, c_count = st->stream_channels;
    int lm;
    for (lm = 0; lm <= 3; lm++)
        if (120 << lm == frame_size) break;
    if (lm > 3) return false;
    const int m = 1 << lm, n = m * 120;
    int eff_end = st->end > NB ? NB : st->end;

    if (data == NULL || len <= 1) {
        decode_lost(st, pcm, lm);
        return true;
    }
    ahopus_range own;
    if (!rd) {
        ahopus_range_init(&own, data, (uint32_t)len);
        rd = &own;
    }
    if (c_count == 1)
        for (int i = 0; i < NB; i++)
            if (st->old_band_e[NB + i] > st->old_band_e[i]) st->old_band_e[i] = st->old_band_e[NB + i];

    int32_t total_bits = len * 8;
    int32_t tell = ahopus_tell(rd);
    int silence;
    if (tell >= total_bits) silence = 1;
    else if (tell == 1) silence = ahopus_bit_logp(rd, 15);
    else silence = 0;
    if (silence) {
        tell = len * 8;
        rd->nbits_total += tell - ahopus_tell(rd);
    }

    float postfilter_gain = 0;
    int postfilter_pitch = 0, postfilter_tapset = 0;
    if (st->start == 0 && tell + 16 <= total_bits) {
        if (ahopus_bit_logp(rd, 1)) {
            int octave = (int)ahopus_uint(rd, 6);
            postfilter_pitch = (16 << octave) + (int)ahopus_bits(rd, 4 + (unsigned)octave) - 1;
            int qg = (int)ahopus_bits(rd, 3);
            if (ahopus_tell(rd) + 2 <= total_bits) postfilter_tapset = ahopus_icdf(rd, tapset_icdf, 2);
            postfilter_gain = 0.09375f * (float)(qg + 1);
        }
        tell = ahopus_tell(rd);
    }
    int is_transient = 0;
    if (lm > 0 && tell + 3 <= total_bits) {
        is_transient = ahopus_bit_logp(rd, 3);
        tell = ahopus_tell(rd);
    }
    int short_blocks = is_transient ? m : 0;
    int intra_ener = tell + 3 <= total_bits ? ahopus_bit_logp(rd, 3) : 0;
    unquant_coarse_energy(st, intra_ener, rd, c_count, lm);

    /* Time-frequency resolution per band. */
    int tf_res[NB];
    {
        uint32_t budget = rd->storage * 8, tl = (uint32_t)ahopus_tell(rd);
        int logp = is_transient ? 2 : 4, curr = 0, tf_changed = 0;
        int tf_select_rsv = lm > 0 && tl + (uint32_t)logp + 1 <= budget;
        budget -= (uint32_t)tf_select_rsv;
        for (int i = st->start; i < st->end; i++) {
            if (tl + (uint32_t)logp <= budget) {
                curr ^= ahopus_bit_logp(rd, (unsigned)logp);
                tl = (uint32_t)ahopus_tell(rd);
                tf_changed |= curr;
            }
            tf_res[i] = curr;
            logp = is_transient ? 4 : 5;
        }
        int tf_select = 0;
        if (tf_select_rsv && tf_select_table[lm][4 * is_transient + 0 + tf_changed] != tf_select_table[lm][4 * is_transient + 2 + tf_changed])
            tf_select = ahopus_bit_logp(rd, 1);
        for (int i = st->start; i < st->end; i++) tf_res[i] = tf_select_table[lm][4 * is_transient + 2 * tf_select + tf_res[i]];
    }

    tell = ahopus_tell(rd);
    int spread = SPREAD_NORMAL;
    if (tell + 4 <= total_bits) spread = ahopus_icdf(rd, spread_icdf, 5);

    int cap[NB], offsets[NB], pulses[NB], fine_quant[NB], fine_priority[NB];
    for (int i = 0; i < NB; i++) {
        int nn = (eb[i + 1] - eb[i]) << lm;
        cap[i] = (ahopus_cache_caps[NB * (2 * lm + c_count - 1) + i] + 64) * c_count * nn >> 2;
        offsets[i] = 0;
    }
    int dynalloc_logp = 6;
    total_bits <<= BITRES;
    int32_t tellf = (int32_t)ahopus_tell_frac(rd);
    for (int i = st->start; i < st->end; i++) {
        int width = c_count * (eb[i + 1] - eb[i]) << lm;
        int quanta = imin(width << BITRES, imax(6 << BITRES, width));
        int loop_logp = dynalloc_logp, boost = 0;
        while (tellf + (loop_logp << BITRES) < total_bits && boost < cap[i]) {
            int flag = ahopus_bit_logp(rd, (unsigned)loop_logp);
            tellf = (int32_t)ahopus_tell_frac(rd);
            if (!flag) break;
            boost += quanta;
            total_bits -= quanta;
            loop_logp = 1;
        }
        offsets[i] = boost;
        if (boost > 0) dynalloc_logp = imax(2, dynalloc_logp - 1);
    }
    int alloc_trim = tellf + (6 << BITRES) <= total_bits ? ahopus_icdf(rd, trim_icdf, 7) : 5;

    int32_t bits = (((int32_t)len * 8) << BITRES) - (int32_t)ahopus_tell_frac(rd) - 1;
    int anti_collapse_rsv = is_transient && lm >= 2 && bits >= ((lm + 2) << BITRES) ? 1 << BITRES : 0;
    bits -= anti_collapse_rsv;
    int intensity = 0, dual_stereo = 0;
    int32_t balance;
    int coded_bands = compute_allocation(st, offsets, cap, alloc_trim, &intensity, &dual_stereo, bits, &balance, pulses,
                                         fine_quant, fine_priority, c_count, lm, rd);
    unquant_fine_energy(st, fine_quant, rd, c_count);

    float x[2 * 960];
    uint8_t collapse_masks[2 * NB];
    for (int c = 0; c < c_count; c++) {
        memset(x + c * n, 0, (size_t)(m * eb[st->start]) * sizeof(float));
        memset(x + c * n + m * eb[eff_end], 0, (size_t)(n - m * eb[eff_end]) * sizeof(float));
    }
    quant_all_bands(st, x, c_count == 2 ? x + n : NULL, collapse_masks, pulses, short_blocks, spread, dual_stereo, intensity,
                    tf_res, len * (8 << BITRES) - anti_collapse_rsv, balance, rd, lm, coded_bands);

    int anti_collapse_on = anti_collapse_rsv > 0 ? (int)ahopus_bits(rd, 1) : 0;
    unquant_energy_finalise(st, fine_quant, fine_priority, len * 8 - ahopus_tell(rd), rd, c_count);
    if (anti_collapse_on) anti_collapse(st, x, collapse_masks, lm, c_count, n, pulses, st->rng);

    float freq[2 * 960];
    denormalise_bands(st, x, freq, eff_end, c_count, m, silence != 0);
    if (silence)
        for (int i = 0; i < c_count * NB; i++) st->old_band_e[i] = -28.f;
    if (cc == 2 && c_count == 1) memcpy(freq + n, freq, (size_t)n * sizeof(float));
    if (cc == 1 && c_count == 2)
        for (int i = 0; i < n; i++) freq[i] = 0.5f * (freq[i] + freq[n + i]);

    synthesize(st, freq, short_blocks, lm);
    post_filter(st, lm, postfilter_pitch, postfilter_gain, postfilter_tapset);

    if (c_count == 1)
        for (int i = 0; i < NB; i++) st->old_band_e[NB + i] = st->old_band_e[i];
    if (!is_transient) {
        memcpy(st->old_log_e2, st->old_log_e, sizeof(st->old_log_e));
        memcpy(st->old_log_e, st->old_band_e, sizeof(st->old_log_e));
        for (int i = 0; i < 2 * NB; i++) {
            float bg = st->background_log_e[i] + (float)m * 0.001f;
            st->background_log_e[i] = bg < st->old_band_e[i] ? bg : st->old_band_e[i];
        }
    } else {
        for (int i = 0; i < 2 * NB; i++)
            if (st->old_band_e[i] < st->old_log_e[i]) st->old_log_e[i] = st->old_band_e[i];
    }
    for (int c = 0; c < 2; c++) {
        for (int i = 0; i < st->start; i++) {
            st->old_band_e[c * NB + i] = 0;
            st->old_log_e[c * NB + i] = st->old_log_e2[c * NB + i] = -28.f;
        }
        for (int i = st->end; i < NB; i++) {
            st->old_band_e[c * NB + i] = 0;
            st->old_log_e[c * NB + i] = st->old_log_e2[c * NB + i] = -28.f;
        }
    }
    st->rng = rd->rng;
    deemphasis(st, pcm, n);
    st->loss_count = 0;
    st->skip_plc = 0;
    return ahopus_tell(rd) <= 8 * len;
}
