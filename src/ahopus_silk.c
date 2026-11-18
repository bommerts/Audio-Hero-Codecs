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
/* The SILK layer of Opus (RFC 6716 section 4.2): side information, line
   spectral frequencies, pitch and long-term prediction, the excitation and
   its synthesis, stereo unmixing, loss cover and the resampler to 48 kHz.
   SILK is defined bit for bit in whole numbers; this code follows the RFC's
   reference decoder (with the fixes of RFC 8251) in each step. */
#include "ahopus_internal.h"

#define TYPE_NO_VOICE 0
#define TYPE_UNVOICED 1
#define TYPE_VOICED 2
#define CODE_INDEPENDENTLY 0
#define CODE_INDEPENDENTLY_NO_LTP_SCALING 1
#define CODE_CONDITIONALLY 2
#define LTP_ORDER 5
#define MAX_LPC_ORDER 16
#define MAX_NB_SUBFR 4
#define MAX_FRAME 320
#define MAX_SUBFR 80
#define N_LEVELS_QGAIN 64
#define MIN_DELTA_GAIN_QUANT (-4)
#define MAX_DELTA_GAIN_QUANT 36
#define QUANT_LEVEL_ADJUST_Q10 80
#define NLSF_QUANT_MAX_AMPLITUDE 4
#define BWE_AFTER_LOSS_Q16 63570
#define STEREO_INTERP_LEN_MS 8
#define MAX_PULSES 16
#define CNG_BUF_MASK_MAX 255
#define CNG_GAIN_SMTH_Q16 4634
#define CNG_NLSF_SMTH_Q16 16348
#define RAND_BUF_SIZE 128
#define RAND_BUF_MASK (RAND_BUF_SIZE - 1)
#define V_PITCH_GAIN_START_MIN_Q14 11469
#define V_PITCH_GAIN_START_MAX_Q14 15565
#define PITCH_DRIFT_FAC_Q16 655
#define MAX_PITCH_LAG_MS 18
#define LOG2_INV_LPC_GAIN_HIGH_THRES 3
#define LOG2_INV_LPC_GAIN_LOW_THRES 8

/* ---- fixed-point helpers, as SILK defines them --------------------------- */

static inline int32_t lsh(int32_t a, int s) { return (int32_t)((uint32_t)a << s); }
static inline int32_t add_w(int32_t a, int32_t b) { return (int32_t)((uint32_t)a + (uint32_t)b); }
static inline int32_t sub_w(int32_t a, int32_t b) { return (int32_t)((uint32_t)a - (uint32_t)b); }
/* (a * low 16 bits of b, signed) >> 16 */
static inline int32_t smulwb(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * (int16_t)b) >> 16); }
static inline int32_t smlawb(int32_t a, int32_t b, int32_t c) { return add_w(a, smulwb(b, c)); }
static inline int32_t smulww(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 16); }
static inline int32_t smlaww(int32_t a, int32_t b, int32_t c) { return add_w(a, smulww(b, c)); }
static inline int32_t smulbb(int32_t a, int32_t b) { return (int32_t)(int16_t)a * (int16_t)b; }
static inline int32_t smlabb(int32_t a, int32_t b, int32_t c) { return add_w(a, smulbb(b, c)); }
static inline int32_t smmul(int32_t a, int32_t b) { return (int32_t)(((int64_t)a * b) >> 32); }
static inline int32_t rshift_round(int32_t a, int s) { return s == 1 ? (a >> 1) + (a & 1) : ((a >> (s - 1)) + 1) >> 1; }
static inline int64_t rshift_round64(int64_t a, int s) { return s == 1 ? (a >> 1) + (a & 1) : ((a >> (s - 1)) + 1) >> 1; }
static inline int32_t sat16(int32_t a) { return a > 32767 ? 32767 : a < -32768 ? -32768 : a; }
/* Clamps a between two limits given in either order. */
static inline int32_t limit32(int32_t a, int32_t l1, int32_t l2)
{
    if (l1 > l2) return a > l1 ? l1 : a < l2 ? l2 : a;
    return a > l2 ? l2 : a < l1 ? l1 : a;
}
static inline int32_t clz32(int32_t x) { return x == 0 ? 32 : 32 - ahopus_ilog((uint32_t)x); }
static inline int32_t abs32(int32_t x) { return x < 0 ? -x : x; }
static inline int32_t silk_rand(int32_t seed) { return (int32_t)(907633515u + (uint32_t)seed * 196314165u); }
static inline int32_t sub_sat32(int32_t a, int32_t b)
{
    int64_t r = (int64_t)a - b;
    return r > INT32_MAX ? INT32_MAX : r < INT32_MIN ? INT32_MIN : (int32_t)r;
}
static inline int32_t lshift_sat32(int32_t a, int s) { return lsh(limit32(a, INT32_MIN >> s, INT32_MAX >> s), s); }

/* An approximation of (a << q) / b. */
static int32_t div32_varq(int32_t a32, int32_t b32, int qres)
{
    int a_headrm = clz32(abs32(a32)) - 1, b_headrm = clz32(abs32(b32)) - 1;
    int32_t a32_nrm = lsh(a32, a_headrm), b32_nrm = lsh(b32, b_headrm);
    int32_t b32_inv = (INT32_MAX >> 2) / (b32_nrm >> 16);
    int32_t result = smulwb(a32_nrm, b32_inv);
    a32_nrm = sub_w(a32_nrm, lsh(smmul(b32_nrm, result), 3));
    result = smlawb(result, a32_nrm, b32_inv);
    int lshift = 29 + a_headrm - b_headrm - qres;
    if (lshift < 0) return lshift_sat32(result, -lshift);
    return lshift < 32 ? result >> lshift : 0;
}

/* An approximation of (1 << q) / b. */
static int32_t inverse32_varq(int32_t b32, int qres)
{
    int b_headrm = clz32(abs32(b32)) - 1;
    int32_t b32_nrm = lsh(b32, b_headrm);
    int32_t b32_inv = (INT32_MAX >> 2) / (b32_nrm >> 16);
    int32_t result = lsh(b32_inv, 16);
    int32_t err_q32 = lsh((1 << 29) - smulwb(b32_nrm, b32_inv), 3);
    result = smlaww(result, err_q32, b32_inv);
    int lshift = 61 - b_headrm - qres;
    if (lshift <= 0) return lshift_sat32(result, -lshift);
    return lshift < 32 ? result >> lshift : 0;
}

static int32_t sqrt_approx(int32_t x)
{
    if (x <= 0) return 0;
    int32_t lz = clz32(x);
    int r = 24 - lz;
    uint32_t ux = (uint32_t)x;
    int32_t frac_q7 = (int32_t)((r >= 0 ? (ux >> r) | (r ? ux << (32 - r) : 0) : (ux << -r) | (ux >> (32 + r))) & 0x7f);
    int32_t y = (lz & 1) ? 32768 : 46214;
    y >>= lz >> 1;
    return smlawb(y, y, smulbb(213, frac_q7));
}

static int32_t log2lin(int32_t in_log_q7)
{
    if (in_log_q7 < 0) return 0;
    int32_t out = 1 << (in_log_q7 >> 7), frac_q7 = in_log_q7 & 0x7F;
    if (in_log_q7 < 2048) out = out + ((out * smlawb(frac_q7, smulbb(frac_q7, 128 - frac_q7), -174)) >> 7);
    else out = out + (out >> 7) * smlawb(frac_q7, smulbb(frac_q7, 128 - frac_q7), -174);
    return out;
}

static void sum_sqr_shift(int32_t *energy, int *shift, const int16_t *x, int len)
{
    int i, shft = 0;
    int32_t nrg = 0;
    len--;
    for (i = 0; i < len; i += 2) {
        nrg = smlabb(nrg, x[i], x[i]);
        nrg = smlabb(nrg, x[i + 1], x[i + 1]);
        if (nrg < 0) {
            nrg = (int32_t)((uint32_t)nrg >> 2);
            shft = 2;
            break;
        }
    }
    for (; i < len; i += 2) {
        int32_t nrg_tmp = smulbb(x[i], x[i]);
        nrg_tmp = smlabb(nrg_tmp, x[i + 1], x[i + 1]);
        nrg = (int32_t)((uint32_t)nrg + ((uint32_t)nrg_tmp >> shft));
        if (nrg < 0) {
            nrg = (int32_t)((uint32_t)nrg >> 2);
            shft += 2;
        }
    }
    if (i == len) {
        int32_t nrg_tmp = smulbb(x[i], x[i]);
        nrg = (int32_t)((uint32_t)nrg + ((uint32_t)nrg_tmp >> shft));
    }
    if (nrg & 0xC0000000) {
        nrg = (int32_t)((uint32_t)nrg >> 2);
        shft += 2;
    }
    *shift = shft;
    *energy = nrg;
}

/* ---- LPC helpers ----------------------------------------------------------- */

static void bwexpander(int16_t *ar, int d, int32_t chirp_q16)
{
    int32_t chirp_minus_one_q16 = chirp_q16 - 65536;
    for (int i = 0; i < d - 1; i++) {
        ar[i] = (int16_t)rshift_round(chirp_q16 * ar[i], 16);
        chirp_q16 += rshift_round(chirp_q16 * chirp_minus_one_q16, 16);
    }
    ar[d - 1] = (int16_t)rshift_round(chirp_q16 * ar[d - 1], 16);
}

static void bwexpander_32(int32_t *ar, int d, int32_t chirp_q16)
{
    int32_t chirp_minus_one_q16 = chirp_q16 - 65536;
    for (int i = 0; i < d - 1; i++) {
        ar[i] = smulww(chirp_q16, ar[i]);
        chirp_q16 += rshift_round(chirp_q16 * chirp_minus_one_q16, 16);
    }
    ar[d - 1] = smulww(chirp_q16, ar[d - 1]);
}

#define INV_PRED_QA 24
#define A_LIMIT ((int32_t)(0.99975 * (1 << INV_PRED_QA) + 0.5))
#define MIN_INV_GAIN_Q30 ((int32_t)(1.0 / 1e4 * (1 << 30) + 0.5))

/* The inverse prediction gain of an LPC filter in Q30, or 0 when the filter
   is unstable (or nearly so). */
static int32_t lpc_inverse_pred_gain(const int16_t *a_q12, int order)
{
    int32_t a[MAX_LPC_ORDER];
    int32_t dc_resp = 0;
    for (int k = 0; k < order; k++) {
        dc_resp += a_q12[k];
        a[k] = lsh(a_q12[k], INV_PRED_QA - 12);
    }
    if (dc_resp >= 4096) return 0;
    int32_t inv_gain_q30 = 1 << 30;
    for (int k = order - 1; k > 0; k--) {
        if (a[k] > A_LIMIT || a[k] < -A_LIMIT) return 0;
        int32_t rc_q31 = -lsh(a[k], 31 - INV_PRED_QA);
        int32_t rc_mult1_q30 = (1 << 30) - smmul(rc_q31, rc_q31);
        inv_gain_q30 = lsh(smmul(inv_gain_q30, rc_mult1_q30), 2);
        if (inv_gain_q30 < MIN_INV_GAIN_Q30) return 0;
        int mult2q = 32 - clz32(abs32(rc_mult1_q30));
        int32_t rc_mult2 = inverse32_varq(rc_mult1_q30, mult2q + 30);
        for (int n = 0; n < (k + 1) >> 1; n++) {
            int32_t tmp1 = a[n], tmp2 = a[k - n - 1];
            int64_t t = rshift_round64((int64_t)sub_sat32(tmp1, (int32_t)rshift_round64((int64_t)tmp2 * rc_q31, 31)) * rc_mult2, mult2q);
            if (t > INT32_MAX || t < INT32_MIN) return 0;
            a[n] = (int32_t)t;
            t = rshift_round64((int64_t)sub_sat32(tmp2, (int32_t)rshift_round64((int64_t)tmp1 * rc_q31, 31)) * rc_mult2, mult2q);
            if (t > INT32_MAX || t < INT32_MIN) return 0;
            a[k - n - 1] = (int32_t)t;
        }
    }
    if (a[0] > A_LIMIT || a[0] < -A_LIMIT) return 0;
    int32_t rc_q31 = -lsh(a[0], 31 - INV_PRED_QA);
    int32_t rc_mult1_q30 = (1 << 30) - smmul(rc_q31, rc_q31);
    inv_gain_q30 = lsh(smmul(inv_gain_q30, rc_mult1_q30), 2);
    return inv_gain_q30 < MIN_INV_GAIN_Q30 ? 0 : inv_gain_q30;
}

#define NLSF2A_QA 16

static void nlsf2a_find_poly(int32_t *out, const int32_t *c_lsf, int dd)
{
    out[0] = 1 << NLSF2A_QA;
    out[1] = -c_lsf[0];
    for (int k = 1; k < dd; k++) {
        int32_t ftmp = c_lsf[2 * k];
        out[k + 1] = lsh(out[k - 1], 1) - (int32_t)rshift_round64((int64_t)ftmp * out[k], NLSF2A_QA);
        for (int n = k; n > 1; n--) out[n] += out[n - 2] - (int32_t)rshift_round64((int64_t)ftmp * out[n - 1], NLSF2A_QA);
        out[1] -= ftmp;
    }
}

/* Normalized line spectral frequencies to a stable Q12 LPC filter. */
static void nlsf2a(int16_t *a_q12, const int16_t *nlsf, int d)
{
    static const uint8_t ordering16[16] = {0, 15, 8, 7, 4, 11, 12, 3, 2, 13, 10, 5, 6, 9, 14, 1};
    static const uint8_t ordering10[10] = {0, 9, 6, 3, 4, 5, 8, 1, 2, 7};
    const uint8_t *ordering = d == 16 ? ordering16 : ordering10;
    int32_t cos_lsf[MAX_LPC_ORDER], p[MAX_LPC_ORDER / 2 + 1], q[MAX_LPC_ORDER / 2 + 1], a32[MAX_LPC_ORDER];
    for (int k = 0; k < d; k++) {
        int32_t f_int = nlsf[k] >> (15 - 7), f_frac = nlsf[k] - lsh(f_int, 15 - 7);
        int32_t cos_val = ahopus_silk_lsf_cos[f_int], delta = ahopus_silk_lsf_cos[f_int + 1] - cos_val;
        cos_lsf[ordering[k]] = rshift_round(lsh(cos_val, 8) + delta * f_frac, 20 - NLSF2A_QA);
    }
    int dd = d >> 1;
    nlsf2a_find_poly(p, &cos_lsf[0], dd);
    nlsf2a_find_poly(q, &cos_lsf[1], dd);
    for (int k = 0; k < dd; k++) {
        int32_t ptmp = p[k + 1] + p[k], qtmp = q[k + 1] - q[k];
        a32[k] = -qtmp - ptmp;
        a32[d - k - 1] = qtmp - ptmp;
    }
    /* Fit the coefficients into 16 bits. */
    int i, idx = 0;
    for (i = 0; i < 10; i++) {
        int32_t maxabs = 0;
        for (int k = 0; k < d; k++) {
            int32_t absval = abs32(a32[k]);
            if (absval > maxabs) {
                maxabs = absval;
                idx = k;
            }
        }
        maxabs = rshift_round(maxabs, NLSF2A_QA + 1 - 12);
        if (maxabs > 32767) {
            if (maxabs > 163838) maxabs = 163838;
            int32_t sc_q16 = 65470 - lsh(maxabs - 32767, 14) / ((maxabs * (idx + 1)) >> 2);
            bwexpander_32(a32, d, sc_q16);
        } else {
            break;
        }
    }
    if (i == 10) {
        for (int k = 0; k < d; k++) {
            a_q12[k] = (int16_t)sat16(rshift_round(a32[k], NLSF2A_QA + 1 - 12));
            a32[k] = lsh(a_q12[k], NLSF2A_QA + 1 - 12);
        }
    } else {
        for (int k = 0; k < d; k++) a_q12[k] = (int16_t)rshift_round(a32[k], NLSF2A_QA + 1 - 12);
    }
    for (i = 0; i < 16 && lpc_inverse_pred_gain(a_q12, d) == 0; i++) {
        bwexpander_32(a32, d, 65536 - lsh(2, i));
        for (int k = 0; k < d; k++) a_q12[k] = (int16_t)rshift_round(a32[k], NLSF2A_QA + 1 - 12);
    }
}

/* out[ix] = in[ix] - prediction, the first d outputs zero. */
static void lpc_analysis_filter(int16_t *out, const int16_t *in, const int16_t *b, int len, int d)
{
    for (int ix = d; ix < len; ix++) {
        const int16_t *p = &in[ix - 1];
        int32_t acc = smulbb(p[0], b[0]);
        for (int j = 1; j < d; j++) acc = smlabb(acc, p[-j], b[j]);
        acc = sub_w(lsh(p[1], 12), acc);
        out[ix] = (int16_t)sat16(rshift_round(acc, 12));
    }
    memset(out, 0, (size_t)d * sizeof(int16_t));
}

/* ---- line spectral frequencies ------------------------------------------------ */

typedef struct {
    int n_vectors, order;
    int32_t quant_step_size_q16;
    const uint8_t *cb1_nlsf_q8, *cb1_icdf, *pred_q8, *ec_sel, *ec_icdf;
    const short *delta_min_q15;
} nlsf_cb;

static const nlsf_cb cb_nb_mb = {32, 10, 11796, ahopus_silk_nlsf_cb1_nb_mb, ahopus_silk_nlsf_cb1_icdf_nb_mb,
                                 ahopus_silk_nlsf_pred_nb_mb, ahopus_silk_nlsf_cb2_select_nb_mb,
                                 ahopus_silk_nlsf_cb2_icdf_nb_mb, ahopus_silk_nlsf_delta_min_nb_mb};
static const nlsf_cb cb_wb = {32, 16, 9830, ahopus_silk_nlsf_cb1_wb, ahopus_silk_nlsf_cb1_icdf_wb, ahopus_silk_nlsf_pred_wb,
                              ahopus_silk_nlsf_cb2_select_wb, ahopus_silk_nlsf_cb2_icdf_wb, ahopus_silk_nlsf_delta_min_wb};

static void nlsf_unpack(int16_t *ec_ix, uint8_t *pred_q8, const nlsf_cb *cb, int cb1_index)
{
    const uint8_t *sel = &cb->ec_sel[cb1_index * cb->order / 2];
    for (int i = 0; i < cb->order; i += 2) {
        uint8_t entry = *sel++;
        ec_ix[i] = (int16_t)(((entry >> 1) & 7) * (2 * NLSF_QUANT_MAX_AMPLITUDE + 1));
        pred_q8[i] = cb->pred_q8[i + (entry & 1) * (cb->order - 1)];
        ec_ix[i + 1] = (int16_t)(((entry >> 5) & 7) * (2 * NLSF_QUANT_MAX_AMPLITUDE + 1));
        pred_q8[i + 1] = cb->pred_q8[i + ((entry >> 4) & 1) * (cb->order - 1) + 1];
    }
}

static void nlsf_stabilize(int16_t *nlsf, const short *delta_min, int l)
{
    int loops;
    for (loops = 0; loops < 20; loops++) {
        int32_t min_diff = nlsf[0] - delta_min[0];
        int idx = 0;
        for (int i = 1; i <= l - 1; i++) {
            int32_t diff = nlsf[i] - (nlsf[i - 1] + delta_min[i]);
            if (diff < min_diff) {
                min_diff = diff;
                idx = i;
            }
        }
        int32_t diff = (1 << 15) - (nlsf[l - 1] + delta_min[l]);
        if (diff < min_diff) {
            min_diff = diff;
            idx = l;
        }
        if (min_diff >= 0) return;
        if (idx == 0) {
            nlsf[0] = delta_min[0];
        } else if (idx == l) {
            nlsf[l - 1] = (int16_t)((1 << 15) - delta_min[l]);
        } else {
            int32_t min_center = 0, max_center = 1 << 15;
            for (int k = 0; k < idx; k++) min_center += delta_min[k];
            min_center += delta_min[idx] >> 1;
            for (int k = l; k > idx; k--) max_center -= delta_min[k];
            max_center -= delta_min[idx] >> 1;
            int16_t center = (int16_t)limit32(rshift_round((int32_t)nlsf[idx - 1] + nlsf[idx], 1), min_center, max_center);
            nlsf[idx - 1] = (int16_t)(center - (delta_min[idx] >> 1));
            nlsf[idx] = (int16_t)(nlsf[idx - 1] + delta_min[idx]);
        }
    }
    /* The fallback: sort, then push apart. */
    for (int i = 1; i < l; i++) {
        int16_t value = nlsf[i];
        int j;
        for (j = i - 1; j >= 0 && value < nlsf[j]; j--) nlsf[j + 1] = nlsf[j];
        nlsf[j + 1] = value;
    }
    if (nlsf[0] < delta_min[0]) nlsf[0] = delta_min[0];
    for (int i = 1; i < l; i++) {
        int32_t floor_v = sat16(nlsf[i - 1] + delta_min[i]);
        if (nlsf[i] < floor_v) nlsf[i] = (int16_t)floor_v;
    }
    if (nlsf[l - 1] > (1 << 15) - delta_min[l]) nlsf[l - 1] = (int16_t)((1 << 15) - delta_min[l]);
    for (int i = l - 2; i >= 0; i--)
        if (nlsf[i] > nlsf[i + 1] - delta_min[i + 1]) nlsf[i] = (int16_t)(nlsf[i + 1] - delta_min[i + 1]);
}

static void nlsf_decode(int16_t *nlsf_q15, const int8_t *indices, const nlsf_cb *cb)
{
    uint8_t pred_q8[MAX_LPC_ORDER];
    int16_t ec_ix[MAX_LPC_ORDER], res_q10[MAX_LPC_ORDER], w[MAX_LPC_ORDER];
    const uint8_t *cb_element = &cb->cb1_nlsf_q8[indices[0] * cb->order];
    const int order = cb->order;
    for (int i = 0; i < order; i++) nlsf_q15[i] = (int16_t)(cb_element[i] << 7);
    nlsf_unpack(ec_ix, pred_q8, cb, indices[0]);
    /* The residual, predicted backwards. */
    int32_t out_q10 = 0;
    for (int i = order - 1; i >= 0; i--) {
        int32_t pred_q10 = smulbb(out_q10, pred_q8[i]) >> 8;
        out_q10 = lsh(indices[i + 1], 10);
        if (out_q10 > 0) out_q10 -= 102;
        else if (out_q10 < 0) out_q10 += 102;
        out_q10 = smlawb(pred_q10, out_q10, cb->quant_step_size_q16);
        res_q10[i] = (int16_t)out_q10;
    }
    /* Laroia weights of the first stage vector. */
    {
        int32_t tmp1 = (1 << 17) / (nlsf_q15[0] > 1 ? nlsf_q15[0] : 1);
        int32_t d = nlsf_q15[1] - nlsf_q15[0];
        int32_t tmp2 = (1 << 17) / (d > 1 ? d : 1);
        w[0] = (int16_t)(tmp1 + tmp2 < 32767 ? tmp1 + tmp2 : 32767);
        for (int k = 1; k < order - 1; k += 2) {
            d = nlsf_q15[k + 1] - nlsf_q15[k];
            tmp1 = (1 << 17) / (d > 1 ? d : 1);
            w[k] = (int16_t)(tmp1 + tmp2 < 32767 ? tmp1 + tmp2 : 32767);
            d = nlsf_q15[k + 2] - nlsf_q15[k + 1];
            tmp2 = (1 << 17) / (d > 1 ? d : 1);
            w[k + 1] = (int16_t)(tmp1 + tmp2 < 32767 ? tmp1 + tmp2 : 32767);
        }
        d = (1 << 15) - nlsf_q15[order - 1];
        tmp1 = (1 << 17) / (d > 1 ? d : 1);
        w[order - 1] = (int16_t)(tmp1 + tmp2 < 32767 ? tmp1 + tmp2 : 32767);
    }
    for (int i = 0; i < order; i++) {
        int32_t w_q9 = sqrt_approx(lsh(w[i], 18 - 2));
        int32_t v = nlsf_q15[i] + lsh(res_q10[i], 14) / w_q9;
        nlsf_q15[i] = (int16_t)limit32(v, 0, 32767);
    }
    nlsf_stabilize(nlsf_q15, cb->delta_min_q15, order);
}

/* ---- the resampler (8, 12 or 16 kHz to 48 kHz) -------------------------------- */

static void resampler_init(ahopus_resampler *s, int fs_in)
{
    memset(s, 0, sizeof(*s));
    s->fs_in_khz = fs_in / 1000;
    s->input_delay = fs_in == 8000 ? 0 : fs_in == 12000 ? 4 : 7;
    s->batch_size = s->fs_in_khz * 10;
    s->inv_ratio_q16 = lsh(lsh(fs_in, 15) / 48000, 2);
    while (smulww(s->inv_ratio_q16, 48000) < lsh(fs_in, 1)) s->inv_ratio_q16++;
}

static void up2_hq(int32_t *s, int16_t *out, const int16_t *in, int len)
{
    for (int k = 0; k < len; k++) {
        int32_t in32 = lsh(in[k], 10), y, x, out1, out2;
        y = in32 - s[0];
        x = smulwb(y, ahopus_silk_up2_hq_0[0]);
        out1 = s[0] + x;
        s[0] = in32 + x;
        y = out1 - s[1];
        x = smulwb(y, ahopus_silk_up2_hq_0[1]);
        out2 = s[1] + x;
        s[1] = out1 + x;
        y = out2 - s[2];
        x = smlawb(y, y, ahopus_silk_up2_hq_0[2]);
        out1 = s[2] + x;
        s[2] = out2 + x;
        out[2 * k] = (int16_t)sat16(rshift_round(out1, 10));
        y = in32 - s[3];
        x = smulwb(y, ahopus_silk_up2_hq_1[0]);
        out1 = s[3] + x;
        s[3] = in32 + x;
        y = out1 - s[4];
        x = smulwb(y, ahopus_silk_up2_hq_1[1]);
        out2 = s[4] + x;
        s[4] = out1 + x;
        y = out2 - s[5];
        x = smlawb(y, y, ahopus_silk_up2_hq_1[2]);
        out1 = s[5] + x;
        s[5] = out2 + x;
        out[2 * k + 1] = (int16_t)sat16(rshift_round(out1, 10));
    }
}

static int16_t *iir_fir_interpol(int16_t *out, const int16_t *buf, int32_t max_index_q16, int32_t index_increment_q16)
{
    for (int32_t index_q16 = 0; index_q16 < max_index_q16; index_q16 += index_increment_q16) {
        int32_t t = smulwb(index_q16 & 0xFFFF, 12);
        const int16_t *p = &buf[index_q16 >> 16];
        const short *f0 = &ahopus_silk_frac_fir_12[t * 4], *f1 = &ahopus_silk_frac_fir_12[(11 - t) * 4];
        int32_t res = smulbb(p[0], f0[0]);
        res = smlabb(res, p[1], f0[1]);
        res = smlabb(res, p[2], f0[2]);
        res = smlabb(res, p[3], f0[3]);
        res = smlabb(res, p[4], f1[3]);
        res = smlabb(res, p[5], f1[2]);
        res = smlabb(res, p[6], f1[1]);
        res = smlabb(res, p[7], f1[0]);
        *out++ = (int16_t)sat16(rshift_round(res, 15));
    }
    return out;
}

static void iir_fir(ahopus_resampler *s, int16_t *out, const int16_t *in, int in_len)
{
    int16_t buf[2 * 160 + 8];
    int n = 0;
    memcpy(buf, s->s_fir, sizeof(s->s_fir));
    for (;;) {
        n = in_len < s->batch_size ? in_len : s->batch_size;
        up2_hq(s->s_iir, &buf[8], in, n);
        out = iir_fir_interpol(out, buf, lsh(n, 17), s->inv_ratio_q16);
        in += n;
        in_len -= n;
        if (in_len <= 0) break;
        memmove(buf, &buf[n << 1], 8 * sizeof(int16_t));
    }
    memcpy(s->s_fir, &buf[n << 1], 8 * sizeof(int16_t));
}

static void resample(ahopus_resampler *s, int16_t *out, const int16_t *in, int in_len)
{
    int n = s->fs_in_khz - s->input_delay;
    memcpy(&s->delay_buf[s->input_delay], in, (size_t)n * sizeof(int16_t));
    iir_fir(s, out, s->delay_buf, s->fs_in_khz);
    iir_fir(s, &out[48], &in[n], in_len - s->fs_in_khz);
    memcpy(s->delay_buf, &in[in_len - s->input_delay], (size_t)s->input_delay * sizeof(int16_t));
}

/* ---- channel state --------------------------------------------------------------- */

static void plc_reset(ahopus_silk_channel *ch)
{
    ch->plc.pitch_l_q8 = lsh(ch->frame_length, 8 - 1);
    ch->plc.prev_gain_q16[0] = ch->plc.prev_gain_q16[1] = 1 << 16;
    ch->plc.subfr_length = 20;
    ch->plc.nb_subfr = 2;
}

static void cng_reset(ahopus_silk_channel *ch)
{
    int32_t step = 32767 / (ch->lpc_order + 1), acc = 0;
    for (int i = 0; i < ch->lpc_order; i++) {
        acc += step;
        ch->cng.smth_nlsf_q15[i] = (int16_t)acc;
    }
    ch->cng.smth_gain_q16 = 0;
    ch->cng.rand_seed = 3176576;
}

static void channel_init(ahopus_silk_channel *ch)
{
    memset(ch, 0, sizeof(*ch));
    ch->first_frame_after_reset = true;
    ch->prev_gain_q16 = 65536;
    cng_reset(ch);
    plc_reset(ch);
}

void ahopus_silk_init(ahopus_silk *s)
{
    memset(s, 0, sizeof(*s));
    channel_init(&s->ch[0]);
    channel_init(&s->ch[1]);
}

static void set_fs(ahopus_silk_channel *ch, int fs_khz)
{
    ch->subfr_length = 5 * fs_khz;
    int frame_length = ch->nb_subfr * ch->subfr_length;
    if (ch->fs_khz != fs_khz || ch->fs_api_hz != 48000) {
        resampler_init(&ch->resampler, fs_khz * 1000);
        ch->fs_api_hz = 48000;
    }
    if (ch->fs_khz != fs_khz || frame_length != ch->frame_length) {
        if (fs_khz == 8)
            ch->pitch_contour_icdf = ch->nb_subfr == MAX_NB_SUBFR ? ahopus_silk_pitch_contour_nb_icdf : ahopus_silk_pitch_contour_10ms_nb_icdf;
        else
            ch->pitch_contour_icdf = ch->nb_subfr == MAX_NB_SUBFR ? ahopus_silk_pitch_contour_icdf : ahopus_silk_pitch_contour_10ms_icdf;
        if (ch->fs_khz != fs_khz) {
            ch->ltp_mem_length = 20 * fs_khz;
            ch->lpc_order = fs_khz == 16 ? 16 : 10;
            ch->pitch_lag_low_bits_icdf = fs_khz == 16 ? ahopus_silk_uniform8_icdf : fs_khz == 12 ? ahopus_silk_uniform6_icdf : ahopus_silk_uniform4_icdf;
            ch->first_frame_after_reset = true;
            ch->lag_prev = 100;
            ch->last_gain_index = 10;
            ch->prev_signal_type = TYPE_NO_VOICE;
            memset(ch->out_buf, 0, sizeof(ch->out_buf));
            memset(ch->slpc_q14_buf, 0, sizeof(ch->slpc_q14_buf));
        }
        ch->fs_khz = fs_khz;
        ch->frame_length = frame_length;
    }
}

static const nlsf_cb *cb_of(const ahopus_silk_channel *ch)
{
    return ch->lpc_order == 16 ? &cb_wb : &cb_nb_mb;
}

/* ---- side information ------------------------------------------------------- */

static void decode_indices(ahopus_silk_channel *ch, ahopus_range *rd, int frame_index, bool decode_lbrr, int cond_coding)
{
    ahopus_silk_indices *ix = &ch->indices;
    int v;
    if (decode_lbrr || ch->vad_flags[frame_index]) v = ahopus_icdf(rd, ahopus_silk_type_offset_vad_icdf, 8) + 2;
    else v = ahopus_icdf(rd, ahopus_silk_type_offset_no_vad_icdf, 8);
    ix->signal_type = (int8_t)(v >> 1);
    ix->quant_offset_type = (int8_t)(v & 1);

    if (cond_coding == CODE_CONDITIONALLY) {
        ix->gains_indices[0] = (int8_t)ahopus_icdf(rd, ahopus_silk_delta_gain_icdf, 8);
    } else {
        ix->gains_indices[0] = (int8_t)(ahopus_icdf(rd, &ahopus_silk_gain_icdf[ix->signal_type * 8], 8) << 3);
        ix->gains_indices[0] = (int8_t)(ix->gains_indices[0] + ahopus_icdf(rd, ahopus_silk_uniform8_icdf, 8));
    }
    for (int i = 1; i < ch->nb_subfr; i++) ix->gains_indices[i] = (int8_t)ahopus_icdf(rd, ahopus_silk_delta_gain_icdf, 8);

    const nlsf_cb *cb = cb_of(ch);
    int16_t ec_ix[MAX_LPC_ORDER];
    uint8_t pred_q8[MAX_LPC_ORDER];
    ix->nlsf_indices[0] = (int8_t)ahopus_icdf(rd, &cb->cb1_icdf[(ix->signal_type >> 1) * cb->n_vectors], 8);
    nlsf_unpack(ec_ix, pred_q8, cb, ix->nlsf_indices[0]);
    for (int i = 0; i < cb->order; i++) {
        v = ahopus_icdf(rd, &cb->ec_icdf[ec_ix[i]], 8);
        if (v == 0) v -= ahopus_icdf(rd, ahopus_silk_nlsf_ext_icdf, 8);
        else if (v == 2 * NLSF_QUANT_MAX_AMPLITUDE) v += ahopus_icdf(rd, ahopus_silk_nlsf_ext_icdf, 8);
        ix->nlsf_indices[i + 1] = (int8_t)(v - NLSF_QUANT_MAX_AMPLITUDE);
    }
    ix->nlsf_interp_coef_q2 = ch->nb_subfr == MAX_NB_SUBFR ? (int8_t)ahopus_icdf(rd, ahopus_silk_nlsf_interp_icdf, 8) : 4;

    if (ix->signal_type == TYPE_VOICED) {
        bool absolute = true;
        if (cond_coding == CODE_CONDITIONALLY && ch->ec_prev_signal_type == TYPE_VOICED) {
            int delta = ahopus_icdf(rd, ahopus_silk_pitch_delta_icdf, 8);
            if (delta > 0) {
                ix->lag_index = (int16_t)(ch->ec_prev_lag_index + delta - 9);
                absolute = false;
            }
        }
        if (absolute) {
            ix->lag_index = (int16_t)(ahopus_icdf(rd, ahopus_silk_pitch_lag_icdf, 8) * (ch->fs_khz >> 1));
            ix->lag_index = (int16_t)(ix->lag_index + ahopus_icdf(rd, ch->pitch_lag_low_bits_icdf, 8));
        }
        ch->ec_prev_lag_index = ix->lag_index;
        ix->contour_index = (int8_t)ahopus_icdf(rd, ch->pitch_contour_icdf, 8);
        ix->per_index = (int8_t)ahopus_icdf(rd, ahopus_silk_ltp_per_index_icdf, 8);
        static const uint8_t *const gain_icdf[3] = {ahopus_silk_ltp_gain_icdf_0, ahopus_silk_ltp_gain_icdf_1, ahopus_silk_ltp_gain_icdf_2};
        for (int k = 0; k < ch->nb_subfr; k++) ix->ltp_index[k] = (int8_t)ahopus_icdf(rd, gain_icdf[ix->per_index], 8);
        ix->ltp_scale_index = cond_coding == CODE_INDEPENDENTLY ? (int8_t)ahopus_icdf(rd, ahopus_silk_ltp_scale_icdf, 8) : 0;
    }
    ch->ec_prev_signal_type = ix->signal_type;
    ix->seed = (int8_t)ahopus_icdf(rd, ahopus_silk_uniform4_icdf, 8);
}

static void shell_split(int *child1, int *child2, ahopus_range *rd, int p, const uint8_t *table)
{
    if (p > 0) {
        *child1 = ahopus_icdf(rd, &table[ahopus_silk_shell_offsets[p]], 8);
        *child2 = p - *child1;
    } else {
        *child1 = *child2 = 0;
    }
}

static void shell_decoder(int *p0, ahopus_range *rd, int p4)
{
    int p3[2], p2[4], p1[8];
    const uint8_t *t0 = ahopus_silk_shell_table0, *t1 = ahopus_silk_shell_table1, *t2 = ahopus_silk_shell_table2,
                  *t3 = ahopus_silk_shell_table3;
    shell_split(&p3[0], &p3[1], rd, p4, t3);
    shell_split(&p2[0], &p2[1], rd, p3[0], t2);
    shell_split(&p1[0], &p1[1], rd, p2[0], t1);
    shell_split(&p0[0], &p0[1], rd, p1[0], t0);
    shell_split(&p0[2], &p0[3], rd, p1[1], t0);
    shell_split(&p1[2], &p1[3], rd, p2[1], t1);
    shell_split(&p0[4], &p0[5], rd, p1[2], t0);
    shell_split(&p0[6], &p0[7], rd, p1[3], t0);
    shell_split(&p2[2], &p2[3], rd, p3[1], t2);
    shell_split(&p1[4], &p1[5], rd, p2[2], t1);
    shell_split(&p0[8], &p0[9], rd, p1[4], t0);
    shell_split(&p0[10], &p0[11], rd, p1[5], t0);
    shell_split(&p1[6], &p1[7], rd, p2[3], t1);
    shell_split(&p0[12], &p0[13], rd, p1[6], t0);
    shell_split(&p0[14], &p0[15], rd, p1[7], t0);
}

/* The excitation as whole numbers: pulses per block of 16, where they go,
   extra low bits, then signs. */
static void decode_pulses(ahopus_range *rd, int *pulses, int signal_type, int quant_offset_type, int frame_length)
{
    int sum_pulses[20], n_lshifts[20];
    int rate_level = ahopus_icdf(rd, &ahopus_silk_rate_levels_icdf[(signal_type >> 1) * 9], 8);
    int iter = frame_length >> 4;
    if (iter * 16 < frame_length) iter++;
    const uint8_t *cdf = &ahopus_silk_pulses_per_block_icdf[rate_level * 18];
    for (int i = 0; i < iter; i++) {
        n_lshifts[i] = 0;
        sum_pulses[i] = ahopus_icdf(rd, cdf, 8);
        while (sum_pulses[i] == MAX_PULSES + 1) {
            n_lshifts[i]++;
            sum_pulses[i] = ahopus_icdf(rd, &ahopus_silk_pulses_per_block_icdf[9 * 18] + (n_lshifts[i] == 10), 8);
        }
    }
    for (int i = 0; i < iter; i++) {
        if (sum_pulses[i] > 0) shell_decoder(&pulses[i * 16], rd, sum_pulses[i]);
        else memset(&pulses[i * 16], 0, 16 * sizeof(int));
    }
    for (int i = 0; i < iter; i++) {
        if (n_lshifts[i] > 0) {
            int *p = &pulses[i * 16];
            for (int k = 0; k < 16; k++) {
                int abs_q = p[k];
                for (int j = 0; j < n_lshifts[i]; j++) abs_q = (abs_q << 1) + ahopus_icdf(rd, ahopus_silk_lsb_icdf, 8);
                p[k] = abs_q;
            }
            sum_pulses[i] |= n_lshifts[i] << 5;
        }
    }
    /* Signs */
    const uint8_t *icdf_ptr = &ahopus_silk_sign_icdf[7 * (quant_offset_type + (signal_type << 1))];
    int blocks = (frame_length + 8) >> 4;
    int *q = pulses;
    for (int i = 0; i < blocks; i++) {
        int p = sum_pulses[i];
        if (p > 0) {
            uint8_t icdf[2] = {icdf_ptr[(p & 0x1F) < 6 ? (p & 0x1F) : 6], 0};
            for (int j = 0; j < 16; j++)
                if (q[j] > 0) q[j] *= 2 * ahopus_icdf(rd, icdf, 8) - 1;
        }
        q += 16;
    }
}

typedef struct {
    int pitch_l[MAX_NB_SUBFR];
    int32_t gains_q16[MAX_NB_SUBFR];
    int16_t pred_coef_q12[2][MAX_LPC_ORDER];
    int16_t ltp_coef_q14[LTP_ORDER * MAX_NB_SUBFR];
    int ltp_scale_q14;
} frame_ctrl;

static void decode_parameters(ahopus_silk_channel *ch, frame_ctrl *ctrl, int cond_coding)
{
    ahopus_silk_indices *ix = &ch->indices;
    /* Gains: a log scale, the first perhaps coded on its own. */
    const int32_t offset = (2 * 128) / 6 + 16 * 128;
    const int32_t inv_scale_q16 = (65536 * (((88 - 2) * 128) / 6)) / (N_LEVELS_QGAIN - 1);
    for (int k = 0; k < ch->nb_subfr; k++) {
        int prev = ch->last_gain_index;
        if (k == 0 && cond_coding != CODE_CONDITIONALLY) {
            prev = ix->gains_indices[k] > prev - 16 ? ix->gains_indices[k] : prev - 16;
        } else {
            int ind_tmp = ix->gains_indices[k] + MIN_DELTA_GAIN_QUANT;
            int threshold = 2 * MAX_DELTA_GAIN_QUANT - N_LEVELS_QGAIN + prev;
            if (ind_tmp > threshold) prev += (ind_tmp << 1) - threshold;
            else prev += ind_tmp;
        }
        prev = prev < 0 ? 0 : prev > N_LEVELS_QGAIN - 1 ? N_LEVELS_QGAIN - 1 : prev;
        ch->last_gain_index = (int8_t)prev;
        int32_t g = smulwb(inv_scale_q16, prev) + offset;
        ctrl->gains_q16[k] = log2lin(g < 3967 ? g : 3967);
    }

    int16_t nlsf_q15[MAX_LPC_ORDER], nlsf0_q15[MAX_LPC_ORDER];
    nlsf_decode(nlsf_q15, ix->nlsf_indices, cb_of(ch));
    nlsf2a(ctrl->pred_coef_q12[1], nlsf_q15, ch->lpc_order);
    if (ch->first_frame_after_reset) ix->nlsf_interp_coef_q2 = 4;
    if (ix->nlsf_interp_coef_q2 < 4) {
        for (int i = 0; i < ch->lpc_order; i++)
            nlsf0_q15[i] = (int16_t)(ch->prev_nlsf_q15[i] + ((ix->nlsf_interp_coef_q2 * (nlsf_q15[i] - ch->prev_nlsf_q15[i])) >> 2));
        nlsf2a(ctrl->pred_coef_q12[0], nlsf0_q15, ch->lpc_order);
    } else {
        memcpy(ctrl->pred_coef_q12[0], ctrl->pred_coef_q12[1], (size_t)ch->lpc_order * sizeof(int16_t));
    }
    memcpy(ch->prev_nlsf_q15, nlsf_q15, (size_t)ch->lpc_order * sizeof(int16_t));
    if (ch->loss_count) {
        bwexpander(ctrl->pred_coef_q12[0], ch->lpc_order, BWE_AFTER_LOSS_Q16);
        bwexpander(ctrl->pred_coef_q12[1], ch->lpc_order, BWE_AFTER_LOSS_Q16);
    }

    if (ix->signal_type == TYPE_VOICED) {
        /* Pitch lags per subframe from a lag and a contour. */
        const signed char *cb;
        int cbk_size;
        if (ch->fs_khz == 8) {
            cb = ch->nb_subfr == MAX_NB_SUBFR ? ahopus_silk_cb_lags_stage2 : ahopus_silk_cb_lags_stage2_10ms;
            cbk_size = ch->nb_subfr == MAX_NB_SUBFR ? 11 : 3;
        } else {
            cb = ch->nb_subfr == MAX_NB_SUBFR ? ahopus_silk_cb_lags_stage3 : ahopus_silk_cb_lags_stage3_10ms;
            cbk_size = ch->nb_subfr == MAX_NB_SUBFR ? 34 : 12;
        }
        int min_lag = 2 * ch->fs_khz, max_lag = 18 * ch->fs_khz, lag = min_lag + ix->lag_index;
        for (int k = 0; k < ch->nb_subfr; k++)
            ctrl->pitch_l[k] = limit32(lag + cb[k * cbk_size + ix->contour_index], min_lag, max_lag);
        static const signed char *const vq[3] = {ahopus_silk_ltp_vq_0, ahopus_silk_ltp_vq_1, ahopus_silk_ltp_vq_2};
        const signed char *cbk = vq[ix->per_index];
        for (int k = 0; k < ch->nb_subfr; k++)
            for (int i = 0; i < LTP_ORDER; i++) ctrl->ltp_coef_q14[k * LTP_ORDER + i] = (int16_t)(cbk[ix->ltp_index[k] * LTP_ORDER + i] * 128);
        ctrl->ltp_scale_q14 = ahopus_silk_ltp_scales[ix->ltp_scale_index];
    } else {
        memset(ctrl->pitch_l, 0, sizeof(ctrl->pitch_l));
        memset(ctrl->ltp_coef_q14, 0, sizeof(ctrl->ltp_coef_q14));
        ix->per_index = 0;
        ctrl->ltp_scale_q14 = 0;
    }
}

/* ---- synthesis ------------------------------------------------------------------- */

static int32_t lpc_predict(const int32_t *s, const int16_t *a, int order)
{
    int32_t pred = order >> 1;
    for (int j = 0; j < order; j++) pred = smlawb(pred, s[-j - 1], a[j]);
    return pred;
}

static void decode_core(ahopus_silk_channel *ch, frame_ctrl *ctrl, int16_t *xq, const int *pulses)
{
    const ahopus_silk_indices *ix = &ch->indices;
    int16_t s_ltp[MAX_FRAME];
    int32_t s_ltp_q15[2 * MAX_FRAME], res_q14[MAX_SUBFR], slpc_q14[MAX_SUBFR + MAX_LPC_ORDER];
    int32_t offset_q10 = ahopus_silk_quant_offsets[(ix->signal_type >> 1) * 2 + ix->quant_offset_type];
    bool nlsf_interpolation = ix->nlsf_interp_coef_q2 < 4;
    int lag = 0;

    int32_t rand_seed = ix->seed;
    for (int i = 0; i < ch->frame_length; i++) {
        rand_seed = silk_rand(rand_seed);
        int32_t e = lsh(pulses[i], 14);
        if (e > 0) e -= QUANT_LEVEL_ADJUST_Q10 << 4;
        else if (e < 0) e += QUANT_LEVEL_ADJUST_Q10 << 4;
        e += offset_q10 << 4;
        if (rand_seed < 0) e = -e;
        ch->exc_q14[i] = e;
        rand_seed = add_w(rand_seed, pulses[i]);
    }

    memcpy(slpc_q14, ch->slpc_q14_buf, sizeof(ch->slpc_q14_buf));
    const int32_t *pexc = ch->exc_q14;
    int16_t *pxq = xq;
    int s_ltp_buf_idx = ch->ltp_mem_length;
    for (int k = 0; k < ch->nb_subfr; k++) {
        const int32_t *pres;
        const int16_t *a_q12 = ctrl->pred_coef_q12[k >> 1];
        int16_t *b_q14 = &ctrl->ltp_coef_q14[k * LTP_ORDER];
        int signal_type = ix->signal_type;
        int32_t gain_q10 = ctrl->gains_q16[k] >> 6;
        int32_t inv_gain_q31 = inverse32_varq(ctrl->gains_q16[k], 47);
        int32_t gain_adj_q16 = 1 << 16;
        if (ctrl->gains_q16[k] != ch->prev_gain_q16) {
            gain_adj_q16 = div32_varq(ch->prev_gain_q16, ctrl->gains_q16[k], 16);
            for (int i = 0; i < MAX_LPC_ORDER; i++) slpc_q14[i] = smulww(gain_adj_q16, slpc_q14[i]);
        }
        ch->prev_gain_q16 = ctrl->gains_q16[k];

        if (ch->loss_count && ch->prev_signal_type == TYPE_VOICED && ix->signal_type != TYPE_VOICED && k < MAX_NB_SUBFR / 2) {
            memset(b_q14, 0, LTP_ORDER * sizeof(int16_t));
            b_q14[LTP_ORDER / 2] = 1 << 12; /* 0.25 */
            signal_type = TYPE_VOICED;
            ctrl->pitch_l[k] = ch->lag_prev;
        }
        if (signal_type == TYPE_VOICED) {
            lag = ctrl->pitch_l[k];
            if (k == 0 || (k == 2 && nlsf_interpolation)) {
                /* Re-whiten the past output with the new filter. */
                int start_idx = ch->ltp_mem_length - lag - ch->lpc_order - LTP_ORDER / 2;
                if (k == 2) memcpy(&ch->out_buf[ch->ltp_mem_length], xq, (size_t)(2 * ch->subfr_length) * sizeof(int16_t));
                lpc_analysis_filter(&s_ltp[start_idx], &ch->out_buf[start_idx + k * ch->subfr_length], a_q12,
                                    ch->ltp_mem_length - start_idx, ch->lpc_order);
                if (k == 0) inv_gain_q31 = lsh(smulwb(inv_gain_q31, ctrl->ltp_scale_q14), 2);
                for (int i = 0; i < lag + LTP_ORDER / 2; i++)
                    s_ltp_q15[s_ltp_buf_idx - i - 1] = smulwb(inv_gain_q31, s_ltp[ch->ltp_mem_length - i - 1]);
            } else if (gain_adj_q16 != 1 << 16) {
                for (int i = 0; i < lag + LTP_ORDER / 2; i++)
                    s_ltp_q15[s_ltp_buf_idx - i - 1] = smulww(gain_adj_q16, s_ltp_q15[s_ltp_buf_idx - i - 1]);
            }
            const int32_t *pred_lag = &s_ltp_q15[s_ltp_buf_idx - lag + LTP_ORDER / 2];
            for (int i = 0; i < ch->subfr_length; i++) {
                int32_t ltp_pred_q13 = 2;
                ltp_pred_q13 = smlawb(ltp_pred_q13, pred_lag[0], b_q14[0]);
                ltp_pred_q13 = smlawb(ltp_pred_q13, pred_lag[-1], b_q14[1]);
                ltp_pred_q13 = smlawb(ltp_pred_q13, pred_lag[-2], b_q14[2]);
                ltp_pred_q13 = smlawb(ltp_pred_q13, pred_lag[-3], b_q14[3]);
                ltp_pred_q13 = smlawb(ltp_pred_q13, pred_lag[-4], b_q14[4]);
                pred_lag++;
                res_q14[i] = add_w(pexc[i], lsh(ltp_pred_q13, 1));
                s_ltp_q15[s_ltp_buf_idx++] = lsh(res_q14[i], 1);
            }
            pres = res_q14;
        } else {
            pres = pexc;
        }
        for (int i = 0; i < ch->subfr_length; i++) {
            int32_t pred = lpc_predict(&slpc_q14[MAX_LPC_ORDER + i], a_q12, ch->lpc_order);
            slpc_q14[MAX_LPC_ORDER + i] = add_w(pres[i], lsh(pred, 4));
            pxq[i] = (int16_t)sat16(rshift_round(smulww(slpc_q14[MAX_LPC_ORDER + i], gain_q10), 8));
        }
        memmove(slpc_q14, &slpc_q14[ch->subfr_length], MAX_LPC_ORDER * sizeof(int32_t));
        pexc += ch->subfr_length;
        pxq += ch->subfr_length;
    }
    memcpy(ch->slpc_q14_buf, slpc_q14, sizeof(ch->slpc_q14_buf));
}

/* ---- loss cover --------------------------------------------------------------------- */

static void plc_update(ahopus_silk_channel *ch, const frame_ctrl *ctrl)
{
    ahopus_silk_plc *plc = &ch->plc;
    ch->prev_signal_type = ch->indices.signal_type;
    int32_t ltp_gain_q14 = 0;
    if (ch->indices.signal_type == TYPE_VOICED) {
        for (int j = 0; j * ch->subfr_length < ctrl->pitch_l[ch->nb_subfr - 1]; j++) {
            if (j == ch->nb_subfr) break;
            int32_t tmp = 0;
            for (int i = 0; i < LTP_ORDER; i++) tmp += ctrl->ltp_coef_q14[(ch->nb_subfr - 1 - j) * LTP_ORDER + i];
            if (tmp > ltp_gain_q14) {
                ltp_gain_q14 = tmp;
                memcpy(plc->ltp_coef_q14, &ctrl->ltp_coef_q14[(ch->nb_subfr - 1 - j) * LTP_ORDER], LTP_ORDER * sizeof(int16_t));
                plc->pitch_l_q8 = lsh(ctrl->pitch_l[ch->nb_subfr - 1 - j], 8);
            }
        }
        memset(plc->ltp_coef_q14, 0, sizeof(plc->ltp_coef_q14));
        plc->ltp_coef_q14[LTP_ORDER / 2] = (int16_t)ltp_gain_q14;
        if (ltp_gain_q14 < V_PITCH_GAIN_START_MIN_Q14) {
            int32_t scale_q10 = lsh(V_PITCH_GAIN_START_MIN_Q14, 10) / (ltp_gain_q14 > 1 ? ltp_gain_q14 : 1);
            for (int i = 0; i < LTP_ORDER; i++) plc->ltp_coef_q14[i] = (int16_t)(smulbb(plc->ltp_coef_q14[i], scale_q10) >> 10);
        } else if (ltp_gain_q14 > V_PITCH_GAIN_START_MAX_Q14) {
            int32_t scale_q14 = lsh(V_PITCH_GAIN_START_MAX_Q14, 14) / (ltp_gain_q14 > 1 ? ltp_gain_q14 : 1);
            for (int i = 0; i < LTP_ORDER; i++) plc->ltp_coef_q14[i] = (int16_t)(smulbb(plc->ltp_coef_q14[i], scale_q14) >> 14);
        }
    } else {
        plc->pitch_l_q8 = lsh(smulbb(ch->fs_khz, 18), 8);
        memset(plc->ltp_coef_q14, 0, sizeof(plc->ltp_coef_q14));
    }
    memcpy(plc->prev_lpc_q12, ctrl->pred_coef_q12[1], (size_t)ch->lpc_order * sizeof(int16_t));
    plc->prev_ltp_scale_q14 = (int16_t)ctrl->ltp_scale_q14;
    plc->prev_gain_q16[0] = ctrl->gains_q16[ch->nb_subfr - 2];
    plc->prev_gain_q16[1] = ctrl->gains_q16[ch->nb_subfr - 1];
    plc->subfr_length = ch->subfr_length;
    plc->nb_subfr = ch->nb_subfr;
}

static void plc_conceal(ahopus_silk_channel *ch, frame_ctrl *ctrl, int16_t *frame)
{
    static const int16_t harm_att_q15[2] = {32440, 31130};
    static const int16_t rand_att_v_q15[2] = {31130, 26214};
    static const int16_t rand_att_uv_q15[2] = {32440, 29491};
    ahopus_silk_plc *plc = &ch->plc;
    int16_t exc_buf[2 * MAX_SUBFR], a_q12[MAX_LPC_ORDER], s_ltp[MAX_FRAME];
    int32_t s_ltp_q14[2 * MAX_FRAME];
    int32_t prev_gain_q10[2] = {plc->prev_gain_q16[0] >> 6, plc->prev_gain_q16[1] >> 6};
    if (ch->first_frame_after_reset) memset(plc->prev_lpc_q12, 0, sizeof(plc->prev_lpc_q12));

    int16_t *p = exc_buf;
    for (int k = 0; k < 2; k++) {
        for (int i = 0; i < ch->subfr_length; i++)
            p[i] = (int16_t)sat16(smulww(ch->exc_q14[i + (k + ch->nb_subfr - 2) * ch->subfr_length], prev_gain_q10[k]) >> 8);
        p += ch->subfr_length;
    }
    int32_t energy1, energy2;
    int shift1, shift2;
    sum_sqr_shift(&energy1, &shift1, exc_buf, ch->subfr_length);
    sum_sqr_shift(&energy2, &shift2, &exc_buf[ch->subfr_length], ch->subfr_length);
    const int32_t *rand_ptr;
    if ((energy1 >> shift2) < (energy2 >> shift1)) {
        int at = (plc->nb_subfr - 1) * plc->subfr_length - RAND_BUF_SIZE;
        rand_ptr = &ch->exc_q14[at > 0 ? at : 0];
    } else {
        int at = plc->nb_subfr * plc->subfr_length - RAND_BUF_SIZE;
        rand_ptr = &ch->exc_q14[at > 0 ? at : 0];
    }

    int16_t *b_q14 = plc->ltp_coef_q14;
    int16_t rand_scale_q14 = plc->rand_scale_q14;
    int att = ch->loss_count < 1 ? ch->loss_count : 1;
    int32_t harm_gain_q15 = harm_att_q15[att];
    int32_t rand_gain_q15 = ch->prev_signal_type == TYPE_VOICED ? rand_att_v_q15[att] : rand_att_uv_q15[att];
    bwexpander(plc->prev_lpc_q12, ch->lpc_order, 64881); /* 0.99 */
    memcpy(a_q12, plc->prev_lpc_q12, (size_t)ch->lpc_order * sizeof(int16_t));
    if (ch->loss_count == 0) {
        rand_scale_q14 = 1 << 14;
        if (ch->prev_signal_type == TYPE_VOICED) {
            for (int i = 0; i < LTP_ORDER; i++) rand_scale_q14 = (int16_t)(rand_scale_q14 - b_q14[i]);
            if (rand_scale_q14 < 3277) rand_scale_q14 = 3277;
            rand_scale_q14 = (int16_t)(smulbb(rand_scale_q14, plc->prev_ltp_scale_q14) >> 14);
        } else {
            int32_t inv_gain_q30 = lpc_inverse_pred_gain(plc->prev_lpc_q12, ch->lpc_order);
            int32_t down = (1 << 30) >> LOG2_INV_LPC_GAIN_HIGH_THRES;
            if (inv_gain_q30 < down) down = inv_gain_q30;
            if (down < (1 << 30) >> LOG2_INV_LPC_GAIN_LOW_THRES) down = (1 << 30) >> LOG2_INV_LPC_GAIN_LOW_THRES;
            down = lsh(down, LOG2_INV_LPC_GAIN_HIGH_THRES);
            rand_gain_q15 = smulwb(down, rand_gain_q15) >> 14;
        }
    }

    int32_t rand_seed = plc->rand_seed;
    int lag = rshift_round(plc->pitch_l_q8, 8);
    int s_ltp_buf_idx = ch->ltp_mem_length;
    int idx = ch->ltp_mem_length - lag - ch->lpc_order - LTP_ORDER / 2;
    lpc_analysis_filter(&s_ltp[idx], &ch->out_buf[idx], a_q12, ch->ltp_mem_length - idx, ch->lpc_order);
    int32_t inv_gain_q30 = inverse32_varq(plc->prev_gain_q16[1], 46);
    if (inv_gain_q30 > INT32_MAX >> 1) inv_gain_q30 = INT32_MAX >> 1;
    for (int i = idx + ch->lpc_order; i < ch->ltp_mem_length; i++) s_ltp_q14[i] = smulwb(inv_gain_q30, s_ltp[i]);

    for (int k = 0; k < ch->nb_subfr; k++) {
        const int32_t *pred_lag = &s_ltp_q14[s_ltp_buf_idx - lag + LTP_ORDER / 2];
        for (int i = 0; i < ch->subfr_length; i++) {
            int32_t ltp_pred_q12 = 2;
            for (int j = 0; j < LTP_ORDER; j++) ltp_pred_q12 = smlawb(ltp_pred_q12, pred_lag[-j], b_q14[j]);
            pred_lag++;
            rand_seed = silk_rand(rand_seed);
            int r = (rand_seed >> 25) & RAND_BUF_MASK;
            s_ltp_q14[s_ltp_buf_idx++] = lsh(smlawb(ltp_pred_q12, rand_ptr[r], rand_scale_q14), 2);
        }
        for (int j = 0; j < LTP_ORDER; j++) b_q14[j] = (int16_t)(smulbb(harm_gain_q15, b_q14[j]) >> 15);
        rand_scale_q14 = (int16_t)(smulbb(rand_scale_q14, rand_gain_q15) >> 15);
        plc->pitch_l_q8 = smlawb(plc->pitch_l_q8, plc->pitch_l_q8, PITCH_DRIFT_FAC_Q16);
        int32_t max_q8 = lsh(smulbb(MAX_PITCH_LAG_MS, ch->fs_khz), 8);
        if (plc->pitch_l_q8 > max_q8) plc->pitch_l_q8 = max_q8;
        lag = rshift_round(plc->pitch_l_q8, 8);
    }

    int32_t *slpc = &s_ltp_q14[ch->ltp_mem_length - MAX_LPC_ORDER];
    memcpy(slpc, ch->slpc_q14_buf, MAX_LPC_ORDER * sizeof(int32_t));
    for (int i = 0; i < ch->frame_length; i++) {
        int32_t pred = lpc_predict(&slpc[MAX_LPC_ORDER + i], a_q12, ch->lpc_order);
        slpc[MAX_LPC_ORDER + i] = add_w(slpc[MAX_LPC_ORDER + i], lsh(pred, 4));
        frame[i] = (int16_t)sat16(rshift_round(smulww(slpc[MAX_LPC_ORDER + i], prev_gain_q10[1]), 8));
    }
    memcpy(ch->slpc_q14_buf, &slpc[ch->frame_length], MAX_LPC_ORDER * sizeof(int32_t));
    plc->rand_seed = rand_seed;
    plc->rand_scale_q14 = rand_scale_q14;
    for (int i = 0; i < MAX_NB_SUBFR; i++) ctrl->pitch_l[i] = lag;
}

static void plc_glue_frames(ahopus_silk_channel *ch, int16_t *frame, int length)
{
    ahopus_silk_plc *plc = &ch->plc;
    if (ch->loss_count) {
        sum_sqr_shift(&plc->conc_energy, &plc->conc_energy_shift, frame, length);
        plc->last_frame_lost = 1;
        return;
    }
    if (plc->last_frame_lost) {
        int32_t energy;
        int energy_shift;
        sum_sqr_shift(&energy, &energy_shift, frame, length);
        if (energy_shift > plc->conc_energy_shift) plc->conc_energy >>= energy_shift - plc->conc_energy_shift;
        else if (energy_shift < plc->conc_energy_shift) energy >>= plc->conc_energy_shift - energy_shift;
        if (energy > plc->conc_energy) {
            int32_t lz = clz32(plc->conc_energy) - 1;
            plc->conc_energy = lsh(plc->conc_energy, lz);
            energy >>= (24 - lz > 0 ? 24 - lz : 0);
            int32_t frac_q24 = plc->conc_energy / (energy > 1 ? energy : 1);
            int32_t gain_q16 = lsh(sqrt_approx(frac_q24), 4);
            int32_t slope_q16 = ((1 << 16) - gain_q16) / length;
            slope_q16 = lsh(slope_q16, 2);
            for (int i = 0; i < length; i++) {
                frame[i] = (int16_t)smulwb(gain_q16, frame[i]);
                gain_q16 += slope_q16;
                if (gain_q16 > 1 << 16) break;
            }
        }
    }
    plc->last_frame_lost = 0;
}

/* Comfort noise: tracks the background while all is well, and adds it to
   the cover of lost frames. */
static void cng(ahopus_silk_channel *ch, const frame_ctrl *ctrl, int16_t *frame, int length)
{
    ahopus_silk_cng *c = &ch->cng;
    if (ch->fs_khz != c->fs_khz) {
        cng_reset(ch);
        c->fs_khz = ch->fs_khz;
    }
    if (ch->loss_count == 0 && ch->prev_signal_type == TYPE_NO_VOICE) {
        for (int i = 0; i < ch->lpc_order; i++)
            c->smth_nlsf_q15[i] = (int16_t)(c->smth_nlsf_q15[i] + smulwb(ch->prev_nlsf_q15[i] - c->smth_nlsf_q15[i], CNG_NLSF_SMTH_Q16));
        int32_t max_gain = 0;
        int subfr = 0;
        for (int i = 0; i < ch->nb_subfr; i++)
            if (ctrl->gains_q16[i] > max_gain) {
                max_gain = ctrl->gains_q16[i];
                subfr = i;
            }
        memmove(&c->exc_buf_q14[ch->subfr_length], c->exc_buf_q14, (size_t)((ch->nb_subfr - 1) * ch->subfr_length) * sizeof(int32_t));
        memcpy(c->exc_buf_q14, &ch->exc_q14[subfr * ch->subfr_length], (size_t)ch->subfr_length * sizeof(int32_t));
        for (int i = 0; i < ch->nb_subfr; i++) c->smth_gain_q16 += smulwb(ctrl->gains_q16[i] - c->smth_gain_q16, CNG_GAIN_SMTH_Q16);
    }
    if (ch->loss_count) {
        int32_t sig[MAX_FRAME + MAX_LPC_ORDER];
        int16_t a_q12[MAX_LPC_ORDER];
        int exc_mask = CNG_BUF_MASK_MAX;
        while (exc_mask > length) exc_mask >>= 1;
        int32_t seed = c->rand_seed;
        for (int i = 0; i < length; i++) {
            seed = silk_rand(seed);
            int idx = (seed >> 24) & exc_mask;
            sig[MAX_LPC_ORDER + i] = (int16_t)sat16(smulww(c->exc_buf_q14[idx], c->smth_gain_q16 >> 4));
        }
        c->rand_seed = seed;
        nlsf2a(a_q12, c->smth_nlsf_q15, ch->lpc_order);
        memcpy(sig, c->synth_state, sizeof(c->synth_state));
        for (int i = 0; i < length; i++) {
            int32_t sum_q6 = lpc_predict(&sig[MAX_LPC_ORDER + i], a_q12, ch->lpc_order);
            sig[MAX_LPC_ORDER + i] = add_w(sig[MAX_LPC_ORDER + i], lsh(sum_q6, 4));
            frame[i] = (int16_t)sat16(frame[i] + rshift_round(sum_q6, 6));
        }
        memcpy(c->synth_state, &sig[length], sizeof(c->synth_state));
    } else {
        memset(c->synth_state, 0, (size_t)ch->lpc_order * sizeof(int32_t));
    }
}

static int decode_frame(ahopus_silk_channel *ch, ahopus_range *rd, int16_t *out, bool lost, int cond_coding)
{
    frame_ctrl ctrl;
    int pulses[MAX_FRAME];
    memset(&ctrl, 0, sizeof(ctrl));
    int l = ch->frame_length;
    if (!lost) {
        decode_indices(ch, rd, ch->frames_decoded, false, cond_coding);
        decode_pulses(rd, pulses, ch->indices.signal_type, ch->indices.quant_offset_type, ch->frame_length);
        decode_parameters(ch, &ctrl, cond_coding);
        l = ch->frame_length;
        decode_core(ch, &ctrl, out, pulses);
        if (ch->fs_khz != ch->plc.fs_khz) {
            plc_reset(ch);
            ch->plc.fs_khz = ch->fs_khz;
        }
        plc_update(ch, &ctrl);
        ch->loss_count = 0;
        ch->prev_signal_type = ch->indices.signal_type;
        ch->first_frame_after_reset = false;
    } else {
        if (ch->fs_khz != ch->plc.fs_khz) {
            plc_reset(ch);
            ch->plc.fs_khz = ch->fs_khz;
        }
        plc_conceal(ch, &ctrl, out);
        ch->loss_count++;
    }
    int mv_len = ch->ltp_mem_length - ch->frame_length;
    memmove(ch->out_buf, &ch->out_buf[ch->frame_length], (size_t)mv_len * sizeof(int16_t));
    memcpy(&ch->out_buf[mv_len], out, (size_t)ch->frame_length * sizeof(int16_t));
    plc_glue_frames(ch, out, l);
    cng(ch, &ctrl, out, l);
    ch->lag_prev = ctrl.pitch_l[ch->nb_subfr - 1];
    return l;
}

/* ---- the frame decoder ------------------------------------------------------------ */

static void stereo_decode_pred(ahopus_range *rd, int32_t pred_q13[2])
{
    int ix[2][3];
    int n = ahopus_icdf(rd, ahopus_silk_stereo_pred_joint_icdf, 8);
    ix[0][2] = n / 5;
    ix[1][2] = n - 5 * ix[0][2];
    for (n = 0; n < 2; n++) {
        ix[n][0] = ahopus_icdf(rd, ahopus_silk_uniform3_icdf, 8);
        ix[n][1] = ahopus_icdf(rd, ahopus_silk_uniform5_icdf, 8);
    }
    for (n = 0; n < 2; n++) {
        ix[n][0] += 3 * ix[n][2];
        int32_t low_q13 = ahopus_silk_stereo_pred_quant[ix[n][0]];
        int32_t step_q13 = smulwb(ahopus_silk_stereo_pred_quant[ix[n][0] + 1] - low_q13, 6554);
        pred_q13[n] = smlabb(low_q13, step_q13, 2 * ix[n][1] + 1);
    }
    pred_q13[0] -= pred_q13[1];
}

static void ms_to_lr(ahopus_silk *s, int16_t *x1, int16_t *x2, const int32_t pred_q13[2], int fs_khz, int frame_length)
{
    memcpy(x1, s->s_mid, 2 * sizeof(int16_t));
    memcpy(x2, s->s_side, 2 * sizeof(int16_t));
    memcpy(s->s_mid, &x1[frame_length], 2 * sizeof(int16_t));
    memcpy(s->s_side, &x2[frame_length], 2 * sizeof(int16_t));
    int32_t pred0 = s->pred_prev_q13[0], pred1 = s->pred_prev_q13[1];
    int32_t denom_q16 = (1 << 16) / (STEREO_INTERP_LEN_MS * fs_khz);
    int32_t delta0 = rshift_round(smulbb(pred_q13[0] - s->pred_prev_q13[0], denom_q16), 16);
    int32_t delta1 = rshift_round(smulbb(pred_q13[1] - s->pred_prev_q13[1], denom_q16), 16);
    for (int n = 0; n < frame_length; n++) {
        if (n < STEREO_INTERP_LEN_MS * fs_khz) {
            pred0 += delta0;
            pred1 += delta1;
        } else {
            pred0 = pred_q13[0];
            pred1 = pred_q13[1];
        }
        int32_t sum = lsh((x1[n] + x1[n + 2]) + lsh(x1[n + 1], 1), 9);
        sum = smlawb(lsh(x2[n + 1], 8), sum, pred0);
        sum = smlawb(sum, lsh(x1[n + 1], 11), pred1);
        x2[n + 1] = (int16_t)sat16(rshift_round(sum, 8));
    }
    s->pred_prev_q13[0] = (int16_t)pred_q13[0];
    s->pred_prev_q13[1] = (int16_t)pred_q13[1];
    for (int n = 0; n < frame_length; n++) {
        int32_t sum = x1[n + 1] + (int32_t)x2[n + 1], diff = x1[n + 1] - (int32_t)x2[n + 1];
        x1[n + 1] = (int16_t)sat16(sum);
        x2[n + 1] = (int16_t)sat16(diff);
    }
}

int ahopus_silk_decode(ahopus_silk *s, ahopus_range *rd, int channels_api, int channels_internal, int internal_rate,
                       int payload_ms, int lost, bool first_frame, int16_t *out)
{
    ahopus_silk_channel *ch = s->ch;
    int16_t tmp1[2][MAX_FRAME + 2], tmp2[960];
    int32_t ms_pred_q13[2] = {0, 0};
    int decode_only_middle = 0, n_out_dec = 0;

    if (first_frame)
        for (int n = 0; n < channels_internal; n++) ch[n].frames_decoded = 0;
    if (channels_internal > s->channels_internal) channel_init(&ch[1]);
    bool stereo_to_mono = channels_internal == 1 && s->channels_internal == 2 && internal_rate == 1000 * ch[0].fs_khz;

    if (ch[0].frames_decoded == 0) {
        for (int n = 0; n < channels_internal; n++) {
            switch (payload_ms) {
            case 0:
            case 10: ch[n].frames_per_packet = 1; ch[n].nb_subfr = 2; break;
            case 20: ch[n].frames_per_packet = 1; ch[n].nb_subfr = 4; break;
            case 40: ch[n].frames_per_packet = 2; ch[n].nb_subfr = 4; break;
            case 60: ch[n].frames_per_packet = 3; ch[n].nb_subfr = 4; break;
            default: return -1;
            }
            int fs_khz = (internal_rate >> 10) + 1;
            if (fs_khz != 8 && fs_khz != 12 && fs_khz != 16) return -1;
            set_fs(&ch[n], fs_khz);
        }
    }
    if (channels_api == 2 && channels_internal == 2 && (s->channels_api == 1 || s->channels_internal == 1)) {
        s->pred_prev_q13[0] = s->pred_prev_q13[1] = 0;
        s->s_side[0] = s->s_side[1] = 0;
        ch[1].resampler = ch[0].resampler;
    }
    s->channels_api = channels_api;
    s->channels_internal = channels_internal;

    if (!lost && ch[0].frames_decoded == 0) {
        /* Voice activity and redundancy flags, then the redundant (LBRR)
           frames, which are only read past. */
        for (int n = 0; n < channels_internal; n++) {
            for (int i = 0; i < ch[n].frames_per_packet; i++) ch[n].vad_flags[i] = ahopus_bit_logp(rd, 1);
            ch[n].lbrr_flag = ahopus_bit_logp(rd, 1);
        }
        for (int n = 0; n < channels_internal; n++) {
            memset(ch[n].lbrr_flags, 0, sizeof(ch[n].lbrr_flags));
            if (ch[n].lbrr_flag) {
                if (ch[n].frames_per_packet == 1) {
                    ch[n].lbrr_flags[0] = 1;
                } else {
                    int sym = ahopus_icdf(rd, ch[n].frames_per_packet == 2 ? ahopus_silk_lbrr_flags_2_icdf : ahopus_silk_lbrr_flags_3_icdf, 8) + 1;
                    for (int i = 0; i < ch[n].frames_per_packet; i++) ch[n].lbrr_flags[i] = (sym >> i) & 1;
                }
            }
        }
        for (int i = 0; i < ch[0].frames_per_packet; i++) {
            for (int n = 0; n < channels_internal; n++) {
                if (!ch[n].lbrr_flags[i]) continue;
                int pulses[MAX_FRAME];
                if (channels_internal == 2 && n == 0) {
                    stereo_decode_pred(rd, ms_pred_q13);
                    if (ch[1].lbrr_flags[i] == 0) decode_only_middle = ahopus_icdf(rd, ahopus_silk_mid_only_icdf, 8);
                }
                int cond = i > 0 && ch[n].lbrr_flags[i - 1] ? CODE_CONDITIONALLY : CODE_INDEPENDENTLY;
                decode_indices(&ch[n], rd, i, true, cond);
                decode_pulses(rd, pulses, ch[n].indices.signal_type, ch[n].indices.quant_offset_type, ch[n].frame_length);
            }
        }
    }

    if (channels_internal == 2) {
        if (!lost) {
            stereo_decode_pred(rd, ms_pred_q13);
            if (ch[1].vad_flags[ch[0].frames_decoded] == 0) decode_only_middle = ahopus_icdf(rd, ahopus_silk_mid_only_icdf, 8);
            else decode_only_middle = 0;
        } else {
            ms_pred_q13[0] = s->pred_prev_q13[0];
            ms_pred_q13[1] = s->pred_prev_q13[1];
        }
    }
    if (channels_internal == 2 && decode_only_middle == 0 && s->prev_decode_only_middle == 1) {
        memset(ch[1].out_buf, 0, sizeof(ch[1].out_buf));
        memset(ch[1].slpc_q14_buf, 0, sizeof(ch[1].slpc_q14_buf));
        ch[1].lag_prev = 100;
        ch[1].last_gain_index = 10;
        ch[1].prev_signal_type = TYPE_NO_VOICE;
        ch[1].first_frame_after_reset = true;
    }
    bool has_side = !lost ? !decode_only_middle : !s->prev_decode_only_middle;

    for (int n = 0; n < channels_internal; n++) {
        if (n == 0 || has_side) {
            int frame_index = ch[0].frames_decoded - n, cond;
            if (frame_index <= 0) cond = CODE_INDEPENDENTLY;
            else if (n > 0 && s->prev_decode_only_middle) cond = CODE_INDEPENDENTLY_NO_LTP_SCALING;
            else cond = CODE_CONDITIONALLY;
            n_out_dec = decode_frame(&ch[n], rd, &tmp1[n][2], lost != 0, cond);
        } else {
            memset(&tmp1[n][2], 0, (size_t)n_out_dec * sizeof(int16_t));
        }
        ch[n].frames_decoded++;
    }

    if (channels_api == 2 && channels_internal == 2) {
        ms_to_lr(s, tmp1[0], tmp1[1], ms_pred_q13, ch[0].fs_khz, n_out_dec);
    } else {
        memcpy(tmp1[0], s->s_mid, 2 * sizeof(int16_t));
        memcpy(s->s_mid, &tmp1[0][n_out_dec], 2 * sizeof(int16_t));
    }

    int n_out = n_out_dec * 48 / ch[0].fs_khz;
    int16_t *res_out = channels_api == 2 ? tmp2 : out;
    for (int n = 0; n < (channels_api < channels_internal ? channels_api : channels_internal); n++) {
        resample(&ch[n].resampler, res_out, &tmp1[n][1], n_out_dec);
        if (channels_api == 2)
            for (int i = 0; i < n_out; i++) out[n + 2 * i] = res_out[i];
    }
    if (channels_api == 2 && channels_internal == 1) {
        if (stereo_to_mono) {
            resample(&ch[1].resampler, res_out, &tmp1[0][1], n_out_dec);
            for (int i = 0; i < n_out; i++) out[1 + 2 * i] = res_out[i];
        } else {
            for (int i = 0; i < n_out; i++) out[1 + 2 * i] = out[2 * i];
        }
    }
    if (lost) {
        for (int i = 0; i < s->channels_internal; i++) ch[i].last_gain_index = 10;
    } else {
        s->prev_decode_only_middle = decode_only_middle;
    }
    return n_out;
}
