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
/* What the parts of the Opus decoder share: the range decoder, the tables
   and the state of the SILK and CELT layers. */
#ifndef AHOPUS_INTERNAL_H
#define AHOPUS_INTERNAL_H

#include "../include/ahopus.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define AHOPUS_PI 3.14159265358979323846

/* ---- the range decoder (RFC 6716 section 4.1) ----------------------------- */

typedef struct {
    const uint8_t *buf;
    uint32_t storage;    /* bytes; the end may be cut back for redundancy */
    uint32_t offs;       /* next byte read from the front */
    uint32_t end_offs;   /* bytes read from the back, as raw bits */
    uint32_t end_window; /* raw bits not yet used */
    int nend_bits;
    int nbits_total;
    uint32_t rng, val, ext;
    int rem;
    int error;
} ahopus_range;

void ahopus_range_init(ahopus_range *r, const uint8_t *buf, uint32_t storage);
unsigned ahopus_decode(ahopus_range *r, unsigned ft);
unsigned ahopus_decode_bin(ahopus_range *r, unsigned bits);
void ahopus_update(ahopus_range *r, unsigned fl, unsigned fh, unsigned ft);
int ahopus_bit_logp(ahopus_range *r, unsigned logp);
int ahopus_icdf(ahopus_range *r, const uint8_t *icdf, unsigned ftb);
uint32_t ahopus_uint(ahopus_range *r, uint32_t ft);
uint32_t ahopus_bits(ahopus_range *r, unsigned bits);
int ahopus_tell(const ahopus_range *r);
uint32_t ahopus_tell_frac(const ahopus_range *r);
int ahopus_laplace(ahopus_range *r, unsigned fs, int decay);

/* Number of bits in x: 0 for 0, else 1 + floor(log2(x)). */
static inline int ahopus_ilog(uint32_t x)
{
    int n = 0;
    while (x) {
        n++;
        x >>= 1;
    }
    return n;
}

/* ---- CELT tables (ahopus_tables.c) ----------------------------------------- */

#define AHOPUS_BANDS 21
#define AHOPUS_OVERLAP 120

extern const short ahopus_ebands[22];
extern const unsigned char ahopus_band_allocation[11 * 21];
extern const short ahopus_logn[21];
extern const short ahopus_cache_index[105];
extern const unsigned char ahopus_cache_bits[392];
extern const unsigned char ahopus_cache_caps[168];
extern const float ahopus_emeans[25];
extern const unsigned char ahopus_e_prob_model[4][2][42];

/* ---- SILK tables (ahopus_tables.c) ------------------------------------------ */

extern const short ahopus_silk_stereo_pred_quant[16];
extern const unsigned char ahopus_silk_stereo_pred_joint_icdf[25], ahopus_silk_mid_only_icdf[2];
extern const unsigned char ahopus_silk_lbrr_flags_2_icdf[3], ahopus_silk_lbrr_flags_3_icdf[7];
extern const unsigned char ahopus_silk_lsb_icdf[2], ahopus_silk_ltp_scale_icdf[3];
extern const unsigned char ahopus_silk_type_offset_vad_icdf[4], ahopus_silk_type_offset_no_vad_icdf[2];
extern const unsigned char ahopus_silk_nlsf_interp_icdf[5];
extern const short ahopus_silk_quant_offsets[4], ahopus_silk_ltp_scales[3];
extern const unsigned char ahopus_silk_uniform3_icdf[3], ahopus_silk_uniform4_icdf[4], ahopus_silk_uniform5_icdf[5];
extern const unsigned char ahopus_silk_uniform6_icdf[6], ahopus_silk_uniform8_icdf[8], ahopus_silk_nlsf_ext_icdf[7];
extern const unsigned char ahopus_silk_gain_icdf[24], ahopus_silk_delta_gain_icdf[41];
extern const unsigned char ahopus_silk_pitch_lag_icdf[32], ahopus_silk_pitch_delta_icdf[21];
extern const unsigned char ahopus_silk_pitch_contour_icdf[34], ahopus_silk_pitch_contour_nb_icdf[11];
extern const unsigned char ahopus_silk_pitch_contour_10ms_icdf[12], ahopus_silk_pitch_contour_10ms_nb_icdf[3];
extern const unsigned char ahopus_silk_pulses_per_block_icdf[180], ahopus_silk_rate_levels_icdf[18];
extern const unsigned char ahopus_silk_shell_table0[152], ahopus_silk_shell_table1[152];
extern const unsigned char ahopus_silk_shell_table2[152], ahopus_silk_shell_table3[152];
extern const unsigned char ahopus_silk_shell_offsets[17], ahopus_silk_sign_icdf[42];
extern const unsigned char ahopus_silk_ltp_per_index_icdf[3];
extern const unsigned char ahopus_silk_ltp_gain_icdf_0[8], ahopus_silk_ltp_gain_icdf_1[16], ahopus_silk_ltp_gain_icdf_2[32];
extern const signed char ahopus_silk_ltp_vq_0[40], ahopus_silk_ltp_vq_1[80], ahopus_silk_ltp_vq_2[160];
extern const short ahopus_silk_lsf_cos[129];
extern const signed char ahopus_silk_cb_lags_stage2_10ms[6], ahopus_silk_cb_lags_stage3_10ms[24];
extern const signed char ahopus_silk_cb_lags_stage2[44], ahopus_silk_cb_lags_stage3[136];
extern const unsigned char ahopus_silk_nlsf_cb1_nb_mb[320], ahopus_silk_nlsf_cb1_icdf_nb_mb[64];
extern const unsigned char ahopus_silk_nlsf_cb2_select_nb_mb[160], ahopus_silk_nlsf_cb2_icdf_nb_mb[72];
extern const unsigned char ahopus_silk_nlsf_pred_nb_mb[18];
extern const short ahopus_silk_nlsf_delta_min_nb_mb[11];
extern const unsigned char ahopus_silk_nlsf_cb1_wb[512], ahopus_silk_nlsf_cb1_icdf_wb[64];
extern const unsigned char ahopus_silk_nlsf_cb2_select_wb[256], ahopus_silk_nlsf_cb2_icdf_wb[72];
extern const unsigned char ahopus_silk_nlsf_pred_wb[30];
extern const short ahopus_silk_nlsf_delta_min_wb[17];
extern const short ahopus_silk_up2_hq_0[3], ahopus_silk_up2_hq_1[3], ahopus_silk_frac_fir_12[48];

/* ---- SILK ------------------------------------------------------------------ */

#define AHOPUS_SILK_MAX_FRAME 320  /* 20 ms at 16 kHz */
#define AHOPUS_SILK_LTP_MEM 320    /* 20 ms at 16 kHz */

typedef struct {
    int32_t s_iir[6];
    int16_t s_fir[8];
    int16_t delay_buf[16];
    int fs_in_khz, input_delay, batch_size;
    int32_t inv_ratio_q16;
    bool copy; /* same rate in and out */
} ahopus_resampler;

typedef struct {
    int32_t pitch_l_q8;
    int16_t ltp_coef_q14[5];
    int16_t prev_lpc_q12[16];
    int last_frame_lost;
    int32_t rand_seed;
    int16_t rand_scale_q14;
    int32_t conc_energy;
    int conc_energy_shift;
    int16_t prev_ltp_scale_q14;
    int32_t prev_gain_q16[2];
    int fs_khz, nb_subfr, subfr_length;
} ahopus_silk_plc;

typedef struct {
    int32_t exc_buf_q14[AHOPUS_SILK_MAX_FRAME];
    int16_t smth_nlsf_q15[16];
    int32_t synth_state[16];
    int32_t smth_gain_q16;
    int32_t rand_seed;
    int fs_khz;
} ahopus_silk_cng;

/* What one frame's side information says. */
typedef struct {
    int8_t gains_indices[4];
    int8_t ltp_index[4];
    int8_t nlsf_indices[17];
    int16_t lag_index;
    int8_t contour_index, signal_type, quant_offset_type, nlsf_interp_coef_q2, per_index, ltp_scale_index, seed;
} ahopus_silk_indices;

/* One channel of the SILK decoder. */
typedef struct {
    int32_t prev_gain_q16;
    int32_t exc_q14[AHOPUS_SILK_MAX_FRAME];
    int32_t slpc_q14_buf[16];
    int16_t out_buf[AHOPUS_SILK_MAX_FRAME + AHOPUS_SILK_LTP_MEM];
    int lag_prev;
    int8_t last_gain_index;
    int fs_khz, fs_api_hz;
    int nb_subfr, frame_length, subfr_length, ltp_mem_length, lpc_order;
    int16_t prev_nlsf_q15[16];
    bool first_frame_after_reset;
    const uint8_t *pitch_lag_low_bits_icdf, *pitch_contour_icdf;
    int frames_decoded, frames_per_packet;
    int ec_prev_signal_type;
    int16_t ec_prev_lag_index;
    int vad_flags[3];
    int lbrr_flag, lbrr_flags[3];
    ahopus_silk_indices indices;
    ahopus_silk_cng cng;
    int loss_count, prev_signal_type;
    ahopus_silk_plc plc;
    ahopus_resampler resampler;
} ahopus_silk_channel;

typedef struct {
    ahopus_silk_channel ch[2];
    int16_t pred_prev_q13[2];
    int16_t s_mid[2], s_side[2];
    int channels_api, channels_internal;
    int prev_decode_only_middle;
} ahopus_silk;

void ahopus_silk_init(ahopus_silk *s);
/* Decodes one SILK frame (10 or 20 ms) of a packet. internal_rate is 8000,
   12000 or 16000; payload_ms the length of the Opus frame (10, 20, 40, 60).
   lost: 0 to decode, 1 to cover a lost frame. Writes 48 kHz samples,
   interleaved by channels_api, as 16-bit values. Returns the samples per
   channel written, or -1 on bad settings. */
int ahopus_silk_decode(ahopus_silk *s, ahopus_range *rd, int channels_api, int channels_internal, int internal_rate,
                       int payload_ms, int lost, bool first_frame, int16_t *out);

/* ---- CELT ------------------------------------------------------------------ */

#define AHOPUS_CELT_BUF 2048 /* history kept for the post-filter and loss cover */
#define AHOPUS_MAX_PERIOD 1024

/* What every CELT decoder shares and never changes: the window and the
   transform's twiddles. */
typedef struct {
    float window[AHOPUS_OVERLAP];
    float tw_re[480], tw_im[480]; /* e^(+2 pi i k / 480) */
    float trig_re[4][480], trig_im[4][480]; /* e^(2 pi i (k + 1/8) / N), N = 1920 >> shift */
} ahopus_celt_setup;

typedef struct {
    const ahopus_celt_setup *setup;
    int channels;        /* output channels: 1 or 2 */
    int stream_channels; /* channels the packet holds */
    int start, end;      /* bands coded */
    float decode_mem[2][AHOPUS_CELT_BUF + AHOPUS_OVERLAP];
    float old_band_e[2 * AHOPUS_BANDS], old_log_e[2 * AHOPUS_BANDS], old_log_e2[2 * AHOPUS_BANDS];
    float background_log_e[2 * AHOPUS_BANDS];
    float preemph_mem[2];
    int postfilter_period, postfilter_period_old, postfilter_tapset, postfilter_tapset_old;
    float postfilter_gain, postfilter_gain_old;
    uint32_t rng;
    int loss_count, last_pitch_index, skip_plc;
    float lpc[2][24];
} ahopus_celt;

void ahopus_celt_setup_init(ahopus_celt_setup *s);
void ahopus_celt_init(ahopus_celt *c, const ahopus_celt_setup *setup, int channels);
void ahopus_celt_reset(ahopus_celt *c);
/* Decodes frame_size samples (120 << LM) to pcm, interleaved by the output
   channels. data/len NULL/0 (or len <= 1) covers a lost frame. rd may be
   NULL: a decoder of its own is made over data. Returns false on a damaged
   frame (pcm then holds what could be made of it). */
bool ahopus_celt_decode(ahopus_celt *c, const uint8_t *data, int len, float *pcm, int frame_size, ahopus_range *rd);

/* The state of the range decoder after the last packet (for each stream,
   that of its last frame, the streams combined by exclusive or), as RFC
   6716's test vectors record it. For the tests. */
uint32_t ahopus_final_range(const ahopus_decoder *d);

#endif
