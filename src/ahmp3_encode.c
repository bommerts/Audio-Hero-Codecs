/* Layer III encoder: polyphase analysis, MDCT with block switching, a
   masking model, rate/distortion loops, Huffman coding and the bit
   reservoir. Constant and variable bit rate. */
#include "ahmp3_internal.h"
#include "ahmp3_bitstream.h"

#define MAX_PENDING 32
#define FIFO_BYTES 16384
#define STEP_OFF 400
#define STEP_TAB 512

/* One granule of one channel waiting to be coded: its 18 x 32 subband
   samples and what the attack detector saw. */
typedef struct {
    float sub[18][32];
    float energy[18]; /* per time slot, upper subbands */
} granule_in;

typedef struct {
    unsigned char head[40]; /* header and side information */
    int head_bytes;
    int slot;               /* main data bytes this frame carries */
} pending;

struct ahmp3_encoder {
    ahmp3_tables t;
    int lsf, sr_index, bitrate_index, channels_in, channels, mode;
    int granules;           /* per frame: 2 (MPEG-1) or 1 */
    int vbr;                /* 0 = constant bit rate, else the quality 1..10;
                               bitrate_index is then the highest allowed */
    double vbr_noise;       /* how far noise may go over the masking threshold */
    int max_reservoir;      /* bytes main_data_begin can reach back */
    int frame_base, frame_rest, frame_acc; /* frame size bookkeeping */
    float cutoff;           /* lowpass, as a fraction of the band */

    /* input and analysis */
    float X[2][512];        /* polyphase input history ring buffer */
    int X_pos[2];           /* current ring write offset in X */
    float in[2][576];
    int in_len;
    granule_in queue[4][2]; /* previous, frame granules, lookahead */
    int queued;
    int prev_block;         /* block type of the last coded granule */

    /* fast transform tables */
    float inv2cos32[16];
    float inv2cos16[8];
    float inv2cos8[4];
    float inv2cos4[2];
    float mdct18_a[18][9];  /* folded MDCT weights for a[r] = z[8-r] - z[9+r] */
    float mdct18_b[18][9];  /* folded MDCT weights for b[r] = z[26-r] + z[27+r] */
    float mdct_cos_short[6][12];

    /* quantization step lookup tables */
    float pow2_q[STEP_TAB]; /* 2^(-3 * step / 16) */
    float pow2_r[STEP_TAB]; /* 2^(step / 4) */

    /* masking model */
    float ath_long[22], ath_short[13];
    float bark_long[22], bark_short[13];
    float spread_long[22][22], spread_short[13][13];
    float prev_xmin[4][22]; /* left, right, and (variable bit rate) mid, side */
    bool prev_ms;
    float pe_average;
    double full_scale;

    /* output */
    pending frames[MAX_PENDING];
    int frame_count;
    unsigned char fifo[FIFO_BYTES]; /* main data waiting for its frames */
    int fifo_len;
    int slots_pending;      /* main data capacity of the queued frames */
    long long samples_in;
    int flushed;

    /* totals for the info frame */
    uint32_t frames_out;
    uint64_t bytes_out;
    uint16_t music_crc;     /* CRC of every frame written */
    uint64_t seek[256];     /* byte offset of every seek_stride-th frame */
    int seek_count, seek_stride;
};

static inline float fast_log2f(float x)
{
    uint32_t u;
    memcpy(&u, &x, sizeof(u));
    float e = (float)((int32_t)(u >> 23) - 127);
    uint32_t mu = (u & 0x007FFFFFu) | 0x3F800000u;
    float m;
    memcpy(&m, &mu, sizeof(m));
    return e + (-0.34484843f * m + 2.02466578f) * m - 1.67487759f;
}

static inline float fast_pow2f(float x)
{
    if (x < -126.0f) return 0.0f;
    if (x > 126.0f) x = 126.0f;
    int i = (int)floorf(x);
    float f = x - (float)i;
    float p = ((0.0558011f * f + 0.2402265f) * f + 0.6931472f) * f + 1.0f;
    uint32_t u = (uint32_t)(i + 127) << 23;
    float scale;
    memcpy(&scale, &u, sizeof(scale));
    return p * scale;
}

/* CRC-16 with the reflected polynomial 0xA001, as the LAME tag uses. */
static uint16_t crc_a001(uint16_t crc, const unsigned char *p, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int k = 0; k < 8; k++) crc = (uint16_t)(crc & 1 ? (crc >> 1) ^ 0xA001 : crc >> 1);
    }
    return crc;
}

/* ---- setup ----------------------------------------------------------------- */

static int find_index(const int *list, int n, int v)
{
    for (int i = 0; i < n; i++)
        if (list[i] == v) return i;
    return -1;
}

static double bark(double hz)
{
    return 13.0 * atan(0.00076 * hz) + 3.5 * atan((hz / 7500.0) * (hz / 7500.0));
}

/* Threshold in quiet, dB SPL (Terhardt). */
static double ath_db(double hz)
{
    double f = hz / 1000.0;
    if (f < 0.02) f = 0.02;
    if (f > 18) f = 18;
    return 3.64 * pow(f, -0.8) - 6.5 * exp(-0.6 * (f - 3.3) * (f - 3.3)) + 0.001 * f * f * f * f;
}

/* Spreading of masking from a band at bark distance dz (maskee - masker), as
   an energy factor. */
static double spreading(double dz)
{
    double x = dz + 0.474;
    double db = 15.81 + 7.5 * x - 17.5 * sqrt(1 + x * x);
    if (db < -60) return 0;
    return pow(10, db / 10);
}

static void analyze_slot(ahmp3_encoder *e, int ch, const float *x32, float out[32]);
static void mdct_long(const ahmp3_encoder *e, const float block[36], int type, float out[18]);

/* Energy of the MDCT lines of a full-scale tone, so levels can be read as
   dB SPL with a full-scale tone at 96 dB. */
static double calibrate(ahmp3_encoder *e)
{
    float slots[36][32];
    double tone_energy = 0;
    float saved[512];
    int saved_pos = e->X_pos[0];
    memcpy(saved, e->X[0], sizeof(saved));
    memset(e->X[0], 0, sizeof(e->X[0]));
    e->X_pos[0] = 0;
    for (int n = 0; n < 4 * 576; n += 32) {
        float x[32];
        for (int i = 0; i < 32; i++) x[i] = (float)sin(2 * AHMP3_PI * 1000.0 * (n + i) / ahmp3_sample_rates[e->sr_index]);
        float s[32];
        analyze_slot(e, 0, x, s);
        int slot = (n / 32) % 36;
        memcpy(slots[slot], s, sizeof(s));
    }
    for (int sb = 0; sb < 32; sb++) {
        float block[36], out[18];
        for (int i = 0; i < 36; i++) block[i] = slots[i][sb];
        mdct_long(e, block, 0, out);
        for (int k = 0; k < 18; k++) tone_energy += (double)out[k] * out[k];
    }
    memcpy(e->X[0], saved, sizeof(saved));
    e->X_pos[0] = saved_pos;
    return tone_energy;
}

ahmp3_encoder *ahmp3_encoder_new(const ahmp3_encoder_config *c)
{
    if (!c || (c->channels != 1 && c->channels != 2)) return NULL;
    int sr = find_index(ahmp3_sample_rates, 9, c->sample_rate);
    if (sr < 0) return NULL;
    int lsf = sr >= 3;
    if (c->vbr_quality < 0 || c->vbr_quality > 10) return NULL;
    /* With variable bit rate the bit rate is the most a frame may use, and 0
       means the format's highest. */
    int br = c->vbr_quality && !c->bitrate_kbps ? 14 : find_index(ahmp3_bitrates[lsf], 15, c->bitrate_kbps);
    if (br <= 0) return NULL;
    int channels = c->mode == AHMP3_MONO || c->channels == 1 ? 1 : 2;

    ahmp3_encoder *e = calloc(1, sizeof(*e));
    if (!e) return NULL;
    ahmp3_tables_init(&e->t);
    e->lsf = lsf;
    e->sr_index = sr;
    e->bitrate_index = br;
    e->channels_in = c->channels;
    e->channels = channels;
    e->mode = channels == 1 ? 3 : c->mode == AHMP3_STEREO ? 0 : 1;
    e->granules = lsf ? 1 : 2;
    e->max_reservoir = lsf ? 255 : 511;
    int num = (lsf ? 72000 : 144000) * c->bitrate_kbps;
    e->frame_base = num / c->sample_rate;
    e->frame_rest = num % c->sample_rate;
    e->prev_block = 0;
    e->seek_stride = 1;

    for (int k = 0; k < 16; k++) e->inv2cos32[k] = (float)(0.5 / cos((2 * k + 1) * AHMP3_PI / 64.0));
    for (int k = 0; k < 8; k++) e->inv2cos16[k] = (float)(0.5 / cos((2 * k + 1) * AHMP3_PI / 32.0));
    for (int k = 0; k < 4; k++) e->inv2cos8[k] = (float)(0.5 / cos((2 * k + 1) * AHMP3_PI / 16.0));
    for (int k = 0; k < 2; k++) e->inv2cos4[k] = (float)(0.5 / cos((2 * k + 1) * AHMP3_PI / 8.0));

    for (int k = 0; k < 18; k++) {
        for (int r = 0; r < 9; r++) {
            double ang_a = AHMP3_PI / 72.0 * (2 * (8 - r) + 19) * (2 * k + 1);
            double ang_b = AHMP3_PI / 72.0 * (2 * (26 - r) + 19) * (2 * k + 1);
            e->mdct18_a[k][r] = (float)(cos(ang_a) / 9.0);
            e->mdct18_b[k][r] = (float)(cos(ang_b) / 9.0);
        }
    }
    for (int k = 0; k < 6; k++)
        for (int n = 0; n < 12; n++) e->mdct_cos_short[k][n] = (float)(cos(AHMP3_PI / 24.0 * (2 * n + 1 + 6) * (2 * k + 1)) / 3.0);

    for (int i = 0; i < STEP_TAB; i++) {
        int step = i - STEP_OFF;
        e->pow2_q[i] = (float)pow(2.0, -3.0 * step / 16.0);
        e->pow2_r[i] = (float)pow(2.0, step / 4.0);
    }

    /* Lowpass: fewer bits per channel buy less treble, as other encoders do. */
    double rate = c->sample_rate, per_channel = (double)c->bitrate_kbps / channels * (channels == 2 && e->mode == 1 ? 1.25 : 1);
    double hz;
    if (per_channel >= 160) hz = 20500;
    else if (per_channel >= 128) hz = 20000;
    else if (per_channel >= 112) hz = 19500;
    else if (per_channel >= 96) hz = 18500;
    else if (per_channel >= 80) hz = 17500;
    else if (per_channel >= 64) hz = 16500;
    else if (per_channel >= 56) hz = 15000;
    else if (per_channel >= 48) hz = 13500;
    else if (per_channel >= 40) hz = 12000;
    else if (per_channel >= 32) hz = 10000;
    else if (per_channel >= 24) hz = 8000;
    else if (per_channel >= 16) hz = 5500;
    else hz = 3500;
    if (c->vbr_quality) {
        /* The quality sets the treble kept and how far under the masking
           threshold the noise must stay: 3 dB more for each step up. The
           model is a simple one, so the top settings keep a wide margin. */
        static const int treble[11] = {0, 11000, 12500, 14000, 15000, 16000, 17000, 18000, 19000, 19500, 20500};
        hz = treble[c->vbr_quality];
        e->vbr = c->vbr_quality;
        e->vbr_noise = pow(10.0, -0.3 * (c->vbr_quality - 1));
    }
    if (hz > rate * 0.48) hz = rate * 0.48;
    e->cutoff = (float)(hz / (rate / 2));

    e->full_scale = calibrate(e);
    const short *sl = ahmp3_sfb_long[sr], *ss = ahmp3_sfb_short[sr];
    for (int b = 0; b < 22; b++) {
        double lo = sl[b] * rate / 1152, hi = sl[b + 1] * rate / 1152;
        e->bark_long[b] = (float)bark((lo + hi) / 2);
        double m = 1e9;
        for (int j = sl[b]; j < sl[b + 1]; j++) {
            double db = ath_db((j + 0.5) * rate / 1152);
            if (db < m) m = db;
        }
        e->ath_long[b] = (float)(e->full_scale * pow(10, (m - 96) / 10) * (sl[b + 1] - sl[b]));
    }
    for (int b = 0; b < 13; b++) {
        double lo = ss[b] * rate / 384, hi = ss[b + 1] * rate / 384;
        e->bark_short[b] = (float)bark((lo + hi) / 2);
        double m = 1e9;
        for (int j = ss[b]; j < ss[b + 1]; j++) {
            double db = ath_db((j + 0.5) * rate / 384);
            if (db < m) m = db;
        }
        /* A short window holds a third of the energy of a long one. */
        e->ath_short[b] = (float)(e->full_scale / 3 * pow(10, (m - 96) / 10) * (ss[b + 1] - ss[b]));
    }
    for (int a = 0; a < 22; a++)
        for (int b = 0; b < 22; b++) e->spread_long[a][b] = (float)spreading(e->bark_long[a] - e->bark_long[b]);
    for (int a = 0; a < 13; a++)
        for (int b = 0; b < 13; b++) e->spread_short[a][b] = (float)spreading(e->bark_short[a] - e->bark_short[b]);
    for (int ch = 0; ch < 4; ch++)
        for (int b = 0; b < 22; b++) e->prev_xmin[ch][b] = 1e30f;
    e->pe_average = 0;
    return e;
}

void ahmp3_encoder_free(ahmp3_encoder *e)
{
    free(e);
}

int ahmp3_encoder_delay(const ahmp3_encoder *e)
{
    (void)e;
    /* Analysis filter bank (481 - 529 the decoder counts itself) plus one
       granule of MDCT overlap; measured by the tests. */
    return 528;
}

/* ---- analysis filter bank and MDCT --------------------------------------- */

/* Radix-2 fast DCT-III (Lee's algorithm) for the 32-band analysis matrix. */
static inline void dct3_2(const float x[2], float y[2])
{
    float t = x[1] * 0.7071067811865475f;
    y[0] = x[0] + t;
    y[1] = x[0] - t;
}

static inline void dct3_4(const ahmp3_encoder *e, const float x[4], float y[4])
{
    float ev[2] = {x[0], x[2]};
    float od[2] = {x[1], x[1] + x[3]};
    float E[2], O[2];
    dct3_2(ev, E);
    dct3_2(od, O);
    float t0 = O[0] * e->inv2cos4[0];
    float t1 = O[1] * e->inv2cos4[1];
    y[0] = E[0] + t0;
    y[3] = E[0] - t0;
    y[1] = E[1] + t1;
    y[2] = E[1] - t1;
}

static inline void dct3_8(const ahmp3_encoder *e, const float x[8], float y[8])
{
    float ev[4] = {x[0], x[2], x[4], x[6]};
    float od[4] = {x[1], x[1] + x[3], x[3] + x[5], x[5] + x[7]};
    float E[4], O[4];
    dct3_4(e, ev, E);
    dct3_4(e, od, O);
    for (int k = 0; k < 4; k++) {
        float t = O[k] * e->inv2cos8[k];
        y[k] = E[k] + t;
        y[7 - k] = E[k] - t;
    }
}

static inline void dct3_16(const ahmp3_encoder *e, const float x[16], float y[16])
{
    float ev[8], od[8], E[8], O[8];
    for (int m = 0; m < 8; m++) ev[m] = x[2 * m];
    od[0] = x[1];
    for (int m = 1; m < 8; m++) od[m] = x[2 * m - 1] + x[2 * m + 1];
    dct3_8(e, ev, E);
    dct3_8(e, od, O);
    for (int k = 0; k < 8; k++) {
        float t = O[k] * e->inv2cos16[k];
        y[k] = E[k] + t;
        y[15 - k] = E[k] - t;
    }
}

static inline void dct3_32(const ahmp3_encoder *e, const float x[32], float y[32])
{
    float ev[16], od[16], E[16], O[16];
    for (int m = 0; m < 16; m++) ev[m] = x[2 * m];
    od[0] = x[1];
    for (int m = 1; m < 16; m++) od[m] = x[2 * m - 1] + x[2 * m + 1];
    dct3_16(e, ev, E);
    dct3_16(e, od, O);
    for (int k = 0; k < 16; k++) {
        float t = O[k] * e->inv2cos32[k];
        y[k] = E[k] + t;
        y[31 - k] = E[k] - t;
    }
}

/* 32 new input samples -> one sample per subband (ISO 11172-3 C.1.3). */
static void analyze_slot(ahmp3_encoder *e, int ch, const float *x32, float out[32])
{
    float *X = e->X[ch];
    int pos = (e->X_pos[ch] - 32) & 511;
    e->X_pos[ch] = pos;
    for (int i = 0; i < 32; i++) X[pos + 31 - i] = x32[i];

    const float *D = e->t.D;
    float Y[64];
    for (int i = 0; i < 64; i++) {
        float s = D[i] * X[(pos + i) & 511]
                + D[i + 64] * X[(pos + i + 64) & 511]
                + D[i + 128] * X[(pos + i + 128) & 511]
                + D[i + 192] * X[(pos + i + 192) & 511]
                + D[i + 256] * X[(pos + i + 256) & 511]
                + D[i + 320] * X[(pos + i + 320) & 511]
                + D[i + 384] * X[(pos + i + 384) & 511]
                + D[i + 448] * X[(pos + i + 448) & 511];
        Y[i] = s * (1.0f / 32.0f);
    }

    float Z[32];
    Z[0] = Y[16];
    for (int d = 1; d <= 16; d++) Z[d] = Y[16 + d] + Y[16 - d];
    for (int d = 17; d <= 31; d++) Z[d] = Y[16 + d] - Y[80 - d];
    dct3_32(e, Z, out);
}

static void mdct_long(const ahmp3_encoder *e, const float block[36], int type, float out[18])
{
    const float *w = e->t.win[type];
    float z[36];
    for (int n = 0; n < 36; n++) z[n] = block[n] * w[n];
    float a[9], b[9];
    for (int r = 0; r < 9; r++) {
        a[r] = z[8 - r] - z[9 + r];
        b[r] = z[26 - r] + z[27 + r];
    }
    for (int k = 0; k < 18; k++) {
        const float *ca = e->mdct18_a[k];
        const float *cb = e->mdct18_b[k];
        float s = 0;
        for (int r = 0; r < 9; r++) s += a[r] * ca[r] + b[r] * cb[r];
        out[k] = s;
    }
}

/* Three short transforms; out holds window w's coefficient k at 3k + w. */
static void mdct_short(const ahmp3_encoder *e, const float block[36], float out[18])
{
    const float *win = e->t.win[2];
    for (int w = 0; w < 3; w++) {
        float z[12];
        const float *src = block + 6 + 6 * w;
        for (int n = 0; n < 12; n++) z[n] = src[n] * win[n];
        for (int k = 0; k < 6; k++) {
            const float *c = e->mdct_cos_short[k];
            float s = 0;
            for (int n = 0; n < 12; n++) s += z[n] * c[n];
            out[3 * k + w] = s;
        }
    }
}

/* Spectrum of one granule, in the order the decoder reads it. */
static void transform(const ahmp3_encoder *e, const float prev[18][32], const float cur[18][32], int block_type,
                      float xr[576])
{
    if (block_type != 2) {
        for (int sb = 0; sb < 32; sb++) {
            float block[36];
            for (int i = 0; i < 18; i++) {
                block[i] = prev[i][sb];
                block[18 + i] = cur[i][sb];
            }
            mdct_long(e, block, block_type, xr + 18 * sb);
        }
        /* Undo the decoder's alias reduction butterflies. */
        for (int sb = 1; sb < 32; sb++) {
            for (int i = 0; i < 8; i++) {
                float lo = xr[18 * sb - 1 - i], hi = xr[18 * sb + i];
                xr[18 * sb - 1 - i] = lo * e->t.cs[i] + hi * e->t.ca[i];
                xr[18 * sb + i] = hi * e->t.cs[i] - lo * e->t.ca[i];
            }
        }
        return;
    }
    float lines[576]; /* by subband: [18 sb + 3k + w] */
    for (int sb = 0; sb < 32; sb++) {
        float block[36];
        for (int i = 0; i < 18; i++) {
            block[i] = prev[i][sb];
            block[18 + i] = cur[i][sb];
        }
        mdct_short(e, block, lines + 18 * sb);
    }
    /* Short spectrum line f of window w sits at 3f + w; the bitstream wants
       each band's three windows one after another. */
    const short *ss = ahmp3_sfb_short[e->sr_index];
    for (int b = 0; b < 13; b++) {
        int len = ss[b + 1] - ss[b];
        for (int w = 0; w < 3; w++)
            for (int j = 0; j < len; j++) xr[3 * ss[b] + w * len + j] = lines[3 * (ss[b] + j) + w];
    }
}

/* ---- input ------------------------------------------------------------------ */

/* Turns one granule of input (576 samples per channel) into subband samples
   and measures how sharply its loudness rises. */
static void analyze_granule(ahmp3_encoder *e, granule_in *g, int ch, const float *x)
{
    for (int t = 0; t < 18; t++) {
        analyze_slot(e, ch, x + 32 * t, g->sub[t]);
        /* The decoder negates odd samples of odd subbands; match it here. */
        if (t & 1)
            for (int sb = 1; sb < 32; sb += 2) g->sub[t][sb] = -g->sub[t][sb];
        float energy = 0;
        for (int sb = 4; sb < 32; sb++) energy += g->sub[t][sb] * g->sub[t][sb];
        g->energy[t] = energy;
    }
}

/* True when a sharp rise in loudness happens within the granule, which a
   long block would smear into a pre-echo. */
static bool attack(const granule_in *before, const granule_in *g)
{
    float history = 0;
    for (int t = 0; t < 18; t++) history += before->energy[t];
    history *= (1.0f / 18.0f);
    float running = history;
    for (int t = 0; t < 18; t += 3) {
        float now = (g->energy[t] + g->energy[t + 1] + g->energy[t + 2]) * (1.0f / 3.0f);
        if (now > 1e-7f && now > 12.0f * running) return true;
        running = 0.6f * running + 0.4f * now;
    }
    return false;
}

/* ---- masking model ---------------------------------------------------------- */

typedef struct {
    float xmin[22];     /* allowed noise energy per long band */
    float xmin_s[13][3];
    float energy[22];
    float energy_s[13][3];
    float pe;           /* perceptual entropy: how many bits it wants */
} mask;

static void mask_long(ahmp3_encoder *e, int ch, const float xr[576], mask *m)
{
    const short *sl = ahmp3_sfb_long[e->sr_index];
    float tonal[22];
    for (int b = 0; b < 22; b++) {
        float sum = 0.0f, log2sum = 0.0f;
        int n = sl[b + 1] - sl[b];
        for (int j = sl[b]; j < sl[b + 1]; j++) {
            float v = xr[j] * xr[j];
            sum += v;
            log2sum += fast_log2f(v + 1e-20f);
        }
        m->energy[b] = sum;
        /* Spectral flatness: a peaky band is tonal and masks less. */
        float t = 0.0f;
        if (n > 1 && sum > 0.0f) {
            float sfm_db = (log2sum / (float)n - fast_log2f(sum / (float)n + 1e-20f)) * 3.01029996f;
            t = sfm_db * (-1.0f / 25.0f);
            if (t < 0.0f) t = 0.0f;
            else if (t > 1.0f) t = 1.0f;
        }
        tonal[b] = t;
    }
    m->pe = 0.0f;
    for (int b = 0; b < 22; b++) {
        float spread = 0.0f;
        const float *sp = e->spread_long[b];
        for (int k = 0; k < 22; k++) spread += m->energy[k] * sp[k];
        float offset_db = tonal[b] * (14.5f + e->bark_long[b]) + (1.0f - tonal[b]) * 5.5f;
        if (offset_db < 6.0f) offset_db = 6.0f;
        /* 10^(-offset_db/10) = 2^(-offset_db * log2(10) / 10) */
        float thr = spread * fast_pow2f(offset_db * -0.332192809f);
        /* Pre-echo: the threshold may not jump far above the last one. */
        if (thr > 2.0f * e->prev_xmin[ch][b]) thr = 2.0f * e->prev_xmin[ch][b];
        e->prev_xmin[ch][b] = thr;
        if (thr < e->ath_long[b]) thr = e->ath_long[b];
        m->xmin[b] = thr;
        if (m->energy[b] > thr) m->pe += (float)(sl[b + 1] - sl[b]) * fast_log2f(m->energy[b] / thr) * 0.5f;
    }
}

static void mask_short(ahmp3_encoder *e, int ch, const float xr[576], mask *m)
{
    const short *ss = ahmp3_sfb_short[e->sr_index];
    m->pe = 0.0f;
    for (int w = 0; w < 3; w++) {
        for (int b = 0; b < 13; b++) {
            int len = ss[b + 1] - ss[b];
            float sum = 0.0f;
            const float *p = xr + 3 * ss[b] + w * len;
            for (int j = 0; j < len; j++) sum += p[j] * p[j];
            m->energy_s[b][w] = sum;
        }
        for (int b = 0; b < 13; b++) {
            float spread = 0.0f;
            const float *sp = e->spread_short[b];
            for (int k = 0; k < 13; k++) spread += m->energy_s[k][w] * sp[k];
            float thr = spread * 0.19952623f; /* 10^(-0.7) */
            if (thr < e->ath_short[b]) thr = e->ath_short[b];
            m->xmin_s[b][w] = thr;
            if (m->energy_s[b][w] > thr)
                m->pe += (float)(ss[b + 1] - ss[b]) * fast_log2f(m->energy_s[b][w] / thr) * 0.5f;
        }
    }
    /* After a short granule the next long one starts its pre-echo memory fresh. */
    for (int b = 0; b < 22; b++) e->prev_xmin[ch][b] = 1e30f;
}

/* ---- quantization ----------------------------------------------------------- */

typedef struct {
    const float *xr;
    float abs_xr[576];
    float xr34[576];     /* |xr|^(3/4) */
    float xmax34;
    float band_max34[39];
    int cut;             /* lines from here up are zeroed (lowpass) */
    int b_start[39];
    int b_end[39];
    int bands;
} qin;

static void init_band_lines(const ahmp3_encoder *e, const ahmp3_gr *G, qin *q)
{
    const short *sl = ahmp3_sfb_long[e->sr_index], *ss = ahmp3_sfb_short[e->sr_index];
    int n = 0;
    for (int i = 0; i < G->long_end; i++, n++) {
        q->b_start[n] = sl[i];
        q->b_end[n] = sl[i + 1];
    }
    for (int b = G->short_start; b < 13; b++) {
        int len = ss[b + 1] - ss[b];
        for (int w = 0; w < 3; w++, n++) {
            q->b_start[n] = 3 * ss[b] + w * len;
            q->b_end[n] = q->b_start[n] + len;
        }
    }
    q->bands = n;
}

/* Quarter-step exponent of each scale factor band's step size. */
static inline void band_steps(const ahmp3_gr *G, int steps[39], int *bands)
{
    int shift = G->scalefac_scale + 1, gain = G->global_gain - 210;
    int n = 0;
    for (int i = 0; i < G->long_end; i++)
        steps[n++] = gain - ((G->sf[i] + (G->preflag ? ahmp3_pretab[i] : 0)) << shift);
    int k = G->long_end;
    for (int i = G->short_start; i < 13; i++)
        for (int w = 0; w < 3; w++) steps[n++] = gain - 8 * G->subblock_gain[w] - (G->sf[k++] << shift);
    *bands = n;
}

static inline float lookup_q(const ahmp3_encoder *e, int step)
{
    int idx = step + STEP_OFF;
    if ((unsigned)idx < STEP_TAB) return e->pow2_q[idx];
    return (float)pow(2.0, -3.0 * step / 16.0);
}

static inline float lookup_r(const ahmp3_encoder *e, int step)
{
    int idx = step + STEP_OFF;
    if ((unsigned)idx < STEP_TAB) return e->pow2_r[idx];
    return (float)pow(2.0, step / 4.0);
}

/* Quantizes unsigned magnitudes during the rate/distortion search. Overflow is
   rejected upfront per band via band_max34 so the inner loop has no branches.
   Signs are applied once the final gain is chosen. */
static bool quantize_mag(const ahmp3_encoder *e, const qin *q, ahmp3_gr *G)
{
    int steps[39], bands;
    band_steps(G, steps, &bands);
    float fband[39];
    for (int n = 0; n < bands; n++) {
        float f = lookup_q(e, steps[n]);
        fband[n] = f;
        if (q->band_max34[n] * f + 0.4054f > 8206.0f) return false;
    }
    int * restrict ix = G->ix;
    const float * restrict xr34 = q->xr34;
    int cut = q->cut;
    for (int n = 0; n < bands; n++) {
        int s = q->b_start[n], t = q->b_end[n];
        if (s >= cut) {
            memset(ix + s, 0, (size_t)(t - s) * sizeof(int));
            continue;
        }
        int hi = t < cut ? t : cut;
        float f = fband[n];
        if (q->band_max34[n] * f < 0.5946f) {
            memset(ix + s, 0, (size_t)(t - s) * sizeof(int));
            continue;
        }
        for (int j = s; j < hi; j++) ix[j] = (int)(xr34[j] * f + 0.4054f);
        if (hi < t) memset(ix + hi, 0, (size_t)(t - hi) * sizeof(int));
    }
    return true;
}

static void apply_signs(const qin *q, ahmp3_gr *G)
{
    const float *xr = q->xr;
    int *ix = G->ix;
    for (int j = 0; j < 576; j++)
        if (ix[j] && xr[j] < 0.0f) ix[j] = -ix[j];
}

/* Quantization noise energy per band. */
static void noise(const ahmp3_encoder *e, const qin *q, const ahmp3_gr *G, float out[39])
{
    int steps[39], bands;
    band_steps(G, steps, &bands);
    const float *pow43 = e->t.pow43;
    for (int n = 0; n < bands; n++) {
        int s = q->b_start[n], t = q->b_end[n];
        float f = lookup_r(e, steps[n]);
        float sum = 0.0f;
        for (int j = s; j < t; j++) {
            int ix = G->ix[j];
            if (ix < 0) ix = -ix;
            float d = q->abs_xr[j] - pow43[ix] * f;
            sum += d * d;
        }
        out[n] = sum;
    }
}

/* Bits for this granule with its current values: scale factors plus
   Huffman data. Also fills in the Huffman side information. */
static int granule_bits(const ahmp3_encoder *e, ahmp3_gr *G, bool best_split)
{
    if (!ahmp3_choose_scalefac_compress(G, e->lsf)) return 1 << 20;
    return ahmp3_part2_bits(G, e->lsf, false, NULL) + ahmp3_huffman_choose(G, e->sr_index, best_split);
}

/* Inner loop: the finest global gain whose bits fit. When a hint from the
   previous outer iteration is given, an exponential bracket converges around
   it before bisecting. Leaves unsigned magnitudes in G->ix. */
static int fit_gain(const ahmp3_encoder *e, const qin *q, ahmp3_gr *G, int target, int min_lo, int hint)
{
    int lo = min_lo, hi = 255, best = 255, best_bits = 0;
    int last_g = -1;
    if (hint >= min_lo && hint <= 255) {
        G->global_gain = hint;
        last_g = hint;
        int b = quantize_mag(e, q, G) ? granule_bits(e, G, false) : (1 << 20);
        if (b <= target) {
            best = hint;
            best_bits = b;
            hi = hint - 1;
            int step = 1;
            while (best - step >= lo) {
                int p = best - step;
                G->global_gain = p;
                last_g = p;
                b = quantize_mag(e, q, G) ? granule_bits(e, G, false) : (1 << 20);
                if (b <= target) {
                    best = p;
                    best_bits = b;
                    hi = p - 1;
                    step <<= 1;
                } else {
                    lo = p + 1;
                    break;
                }
            }
        } else {
            lo = hint + 1;
            int step = 2;
            while (lo < hi) {
                int p = hint + step;
                if (p >= hi) p = hi;
                G->global_gain = p;
                last_g = p;
                b = quantize_mag(e, q, G) ? granule_bits(e, G, false) : (1 << 20);
                if (b <= target) {
                    best = p;
                    best_bits = b;
                    hi = p - 1;
                    break;
                }
                lo = p + 1;
                if (p == 255) break;
                step <<= 1;
            }
        }
    }
    while (lo <= hi) {
        int mid = (lo + hi) >> 1;
        G->global_gain = mid;
        last_g = mid;
        int b = quantize_mag(e, q, G) ? granule_bits(e, G, false) : (1 << 20);
        if (b <= target) {
            best = mid;
            best_bits = b;
            hi = mid - 1;
        } else {
            lo = mid + 1;
        }
    }
    G->global_gain = best;
    if (last_g != best) {
        quantize_mag(e, q, G);
        best_bits = granule_bits(e, G, false);
    }
    return best_bits;
}

/* Gets a granule ready for the loops: |xr|^(3/4) with the lowpass applied,
   and its side information cleared. */
static void prepare(const ahmp3_encoder *e, const float xr[576], ahmp3_gr *G, qin *q)
{
    q->xr = xr;
    q->xmax34 = 0.0f;
    q->cut = G->block_type == 2 ? 576 : (int)(e->cutoff * 576.0f);
    init_band_lines(e, G, q);
    for (int j = 0; j < 576; j++) {
        float a = fabsf(xr[j]);
        q->abs_xr[j] = a;
        float s = sqrtf(a);
        float v = s * sqrtf(s);
        q->xr34[j] = v;
        if (j < q->cut && v > q->xmax34) q->xmax34 = v;
    }
    if (G->block_type == 2) {
        /* Short blocks: the lowpass applies within each window. */
        const short *ss = ahmp3_sfb_short[e->sr_index];
        int cut = (int)(e->cutoff * 192.0f);
        q->xmax34 = 0.0f;
        for (int b = 0; b < 13; b++) {
            int len = ss[b + 1] - ss[b];
            for (int w = 0; w < 3; w++)
                for (int j = 0; j < len; j++) {
                    int idx = 3 * ss[b] + w * len + j;
                    if (ss[b] + j >= cut) q->xr34[idx] = 0.0f;
                    else if (q->xr34[idx] > q->xmax34) q->xmax34 = q->xr34[idx];
                }
        }
    }
    for (int n = 0; n < q->bands; n++) {
        int s = q->b_start[n], t = q->b_end[n];
        int hi = t < q->cut ? t : q->cut;
        float mx = 0.0f;
        for (int j = s; j < hi; j++)
            if (q->xr34[j] > mx) mx = q->xr34[j];
        q->band_max34[n] = mx;
    }
    memset(G->sf, 0, sizeof(G->sf));
    G->scalefac_scale = 0;
    G->preflag = 0;
}

static void silence(const ahmp3_encoder *e, ahmp3_gr *G)
{
    memset(G->ix, 0, sizeof(G->ix));
    memset(G->sf, 0, sizeof(G->sf));
    G->global_gain = 0;
    G->scalefac_scale = 0;
    G->part23 = granule_bits(e, G, true);
}

/* Variable bit rate: every band gets the coarsest step that keeps its noise
   under the masking threshold, and the granule takes however many bits that
   costs, up to max_bits. */
static void code_granule_vbr(ahmp3_encoder *e, const float xr[576], const mask *m, ahmp3_gr *G, int max_bits)
{
    qin q;
    prepare(e, xr, G, &q);
    if (max_bits > 4095) max_bits = 4095;
    if (q.xmax34 == 0.0f || max_bits < 16) {
        silence(e, G);
        return;
    }
    int bands = q.bands, need[39];
    bool live[39];
    int counted = G->long_end == 22 ? 21 : bands - 3; /* the last band has no scale factor */
    int global = -1;
    for (int n = 0; n < bands; n++) {
        int s = q.b_start[n], t = q.b_end[n];
        float allowed = (float)(e->vbr_noise * (n < G->long_end ? m->xmin[n] : m->xmin_s[G->short_start + (n - G->long_end) / 3][(n - G->long_end) % 3]));
        /* The largest gain (coarsest step) whose noise stays under the
           threshold. A band quieter than its threshold needs nothing; one
           that cannot get under it takes the finest step its largest value
           allows. */
        float top = q.band_max34[n];
        int finest = 0;
        if (top > 0.0f) {
            finest = 210 + (int)ceilf((16.0f / 3.0f) * fast_log2f(top / 8205.0f));
            if (finest < 0) finest = 0;
        }
        int lo = finest, hi = 255, best = finest;
        while (lo <= hi) {
            int mid = (lo + hi) >> 1;
            float fq = lookup_q(e, mid - 210), fr = lookup_r(e, mid - 210);
            float sum = 0.0f;
            for (int j = s; j < t && sum <= allowed; j++) {
                if (q.xr34[j] == 0.0f || j >= q.cut) continue; /* removed by the lowpass */
                float v = q.xr34[j] * fq + 0.4054f;
                if (v > 8206.0f) {
                    sum = 1e30f;
                    break;
                }
                float d = q.abs_xr[j] - e->t.pow43[(int)v] * fr;
                sum += d * d;
            }
            if (sum <= allowed) {
                best = mid;
                lo = mid + 1;
            } else {
                hi = mid - 1;
            }
        }
        live[n] = best < 255;
        need[n] = best;
        if (live[n] && need[n] > global) global = need[n];
    }
    if (global < 0) {
        silence(e, G);
        return;
    }
    for (int n = counted; n < bands; n++)
        if (live[n] && need[n] < global) global = need[n];
    /* Scale factors make up the difference between the global gain and each
       band's own. When they cannot reach, the global gain comes down and the
       other bands are coded finer than they needed. */
    int shift = 1;
    for (;; shift++) {
        bool reach = true;
        for (int n = 0; n < counted; n++)
            if (live[n] && need[n] < global && ((global - need[n] + (1 << shift) - 1) >> shift) > ahmp3_sf_limit(G, e->lsf, n)) reach = false;
        if (reach || shift == 2) break;
    }
    for (int n = 0; n < counted; n++) {
        int cap = need[n] + (ahmp3_sf_limit(G, e->lsf, n) << shift);
        if (live[n] && cap < global) global = cap;
    }
    G->scalefac_scale = shift - 1;
    for (int n = 0; n < counted; n++)
        G->sf[n] = (unsigned char)(live[n] && need[n] < global ? (global - need[n] + (1 << shift) - 1) >> shift : 0);
    G->global_gain = global;
    while (!quantize_mag(e, &q, G) && G->global_gain < 255) G->global_gain++;
    int bits = granule_bits(e, G, true);
    if (bits > max_bits) {
        /* More than the frame can carry: coarsen until it fits. */
        fit_gain(e, &q, G, max_bits, 0, G->global_gain);
        bits = granule_bits(e, G, true);
    }
    apply_signs(&q, G);
    G->part23 = bits;
}

/* Outer loop: shapes the noise under the masking threshold by raising the
   scale factors of bands that are too noisy, keeping the best attempt. */
static void code_granule(ahmp3_encoder *e, const float xr[576], const mask *m, ahmp3_gr *G, int target)
{
    qin q;
    prepare(e, xr, G, &q);
    if (target > 4095) target = 4095;
    if (q.xmax34 == 0.0f || target < 16) {
        memset(G->ix, 0, sizeof(G->ix));
        G->global_gain = 0;
        G->part23 = granule_bits(e, G, true);
        return;
    }

    float xmin[39];
    int bands = q.bands;
    for (int n = 0; n < bands; n++) {
        if (n < G->long_end) {
            xmin[n] = m->xmin[n];
        } else {
            int k = n - G->long_end, b = G->short_start + k / 3;
            xmin[n] = m->xmin_s[b][k % 3];
        }
        if (xmin[n] < 1e-20f) xmin[n] = 1e-20f;
    }
    /* Band 21 (12 for short) has no scale factor; its noise is not counted. */
    int counted = G->long_end == 22 ? 21 : bands - 3;

    /* Minimum global_gain below which xmax34 would exceed 8206 with sf=0. */
    int min_lo = 210 + (int)floorf((16.0f / 3.0f) * fast_log2f(q.xmax34 / 8205.0f)) - 1;
    if (min_lo < 0) min_lo = 0;
    if (min_lo > 255) min_lo = 255;

    ahmp3_gr best = *G;
    int best_over = 1 << 30;
    float best_over_db = 1e30f;
    int hint = -1, stale = 0;
    for (int iter = 0; iter < 16; iter++) {
        fit_gain(e, &q, G, target, min_lo, hint);
        hint = G->global_gain;
        float nz[39];
        noise(e, &q, G, nz);
        int over = 0;
        float over_db = 0.0f;
        for (int n = 0; n < counted; n++) {
            if (nz[n] > xmin[n]) {
                over++;
                over_db += fast_log2f(nz[n] / xmin[n]);
            }
        }
        if (over < best_over || (over == best_over && over_db < best_over_db)) {
            best = *G;
            best_over = over;
            best_over_db = over_db;
            stale = 0;
        } else if (++stale >= 3) {
            break;
        }
        if (!over) break;

        /* Amplify the noisy bands. */
        bool changed = false, all = true;
        int limit = 0;
        for (int n = 0; n < counted; n++) {
            if (nz[n] > xmin[n]) {
                G->sf[n]++;
                changed = true;
            } else {
                all = false;
            }
        }
        for (int n = 0; n < counted; n++)
            if (G->sf[n] > ahmp3_sf_limit(G, e->lsf, n)) limit++;
        if (limit) {
            if (G->scalefac_scale) break;
            /* Coarser scale factor steps reach twice as far. */
            G->scalefac_scale = 1;
            for (int n = 0; n < bands; n++) G->sf[n] = (unsigned char)((G->sf[n] + 1) / 2);
            hint = -1;
            stale = 0;
        }
        if (!changed || all) break;
    }
    *G = best;
    apply_signs(&q, G);
    G->part23 = granule_bits(e, G, true);
}

/* ---- frames ------------------------------------------------------------------ */

static int frame_bytes_next(ahmp3_encoder *e, int *padding)
{
    e->frame_acc += e->frame_rest;
    int sr = ahmp3_sample_rates[e->sr_index];
    *padding = 0;
    if (e->frame_acc >= sr) {
        e->frame_acc -= sr;
        *padding = 1;
    }
    return e->frame_base + *padding;
}

/* Frame size at a bit rate, with no padding: variable bit rate has no
   average to keep. */
static int vbr_frame_bytes(const ahmp3_encoder *e, int bitrate_index)
{
    return (e->lsf ? 72000 : 144000) * ahmp3_bitrates[e->lsf][bitrate_index] / ahmp3_sample_rates[e->sr_index];
}

/* Moves finished frames to out. A frame is finished once the main data
   that fills its slot is known. */
static size_t emit(ahmp3_encoder *e, uint8_t *out, size_t cap, bool all)
{
    size_t written = 0;
    int used = 0;
    int i = 0;
    for (; i < e->frame_count; i++) {
        pending *p = &e->frames[i];
        if (!all && e->fifo_len - used < p->slot) break;
        if (written + (size_t)p->head_bytes + (size_t)p->slot > cap) break;
        size_t frame_start = written;
        if (e->frames_out % (uint32_t)e->seek_stride == 0) {
            if (e->seek_count == 256) {
                /* Full: keep every other entry and note frames half as often. */
                for (int k = 0; k < 128; k++) e->seek[k] = e->seek[2 * k];
                e->seek_count = 128;
                e->seek_stride *= 2;
            }
            if (e->frames_out % (uint32_t)e->seek_stride == 0) e->seek[e->seek_count++] = e->bytes_out;
        }
        memcpy(out + written, p->head, (size_t)p->head_bytes);
        written += (size_t)p->head_bytes;
        int have = e->fifo_len - used;
        int take = have < p->slot ? have : p->slot;
        memcpy(out + written, e->fifo + used, (size_t)take);
        memset(out + written + take, 0, (size_t)(p->slot - take));
        written += (size_t)p->slot;
        used += take;
        e->slots_pending -= p->slot;
        e->frames_out++;
        e->bytes_out += written - frame_start;
        e->music_crc = crc_a001(e->music_crc, out + frame_start, written - frame_start);
    }
    memmove(e->frames, e->frames + i, (size_t)(e->frame_count - i) * sizeof(pending));
    e->frame_count -= i;
    memmove(e->fifo, e->fifo + used, (size_t)(e->fifo_len - used));
    e->fifo_len -= used;
    return written;
}

static void ms_decision(ahmp3_encoder *e, const float xr0[576], const float xr1[576], bool *use_ms)
{
    *use_ms = false;
    if (e->mode != 1) return;
    /* Mid/side pays off when the channels are alike: compare how much
       signal stands above the threshold each way. */
    float lr = 0.0f, ms = 0.0f;
    for (int j = 0; j < 576; j++) {
        float l = xr0[j], r = xr1[j];
        float mid = (l + r) * 0.70710678f, side = (l - r) * 0.70710678f;
        lr += fast_log2f(fabsf(l) * 1000.0f + 1.0f) + fast_log2f(fabsf(r) * 1000.0f + 1.0f);
        ms += fast_log2f(fabsf(mid) * 1000.0f + 1.0f) + fast_log2f(fabsf(side) * 1000.0f + 1.0f);
    }
    *use_ms = ms < lr * 0.95f;
}

static void code_frame(ahmp3_encoder *e, int block_types[2])
{
    ahmp3_fr fr;
    memset(&fr, 0, sizeof(fr));
    int padding = 0;
    int side = e->lsf ? (e->channels == 1 ? 9 : 17) : (e->channels == 1 ? 17 : 32);
    /* Variable bit rate: code with the biggest frame allowed as the limit,
       then pick the smallest frame that holds the result. */
    int bytes = e->vbr ? vbr_frame_bytes(e, e->bitrate_index) : frame_bytes_next(e, &padding);
    fr.lsf = e->lsf;
    fr.sr_index = e->sr_index;
    fr.bitrate_index = e->bitrate_index;
    fr.padding = padding;
    fr.mode = e->mode;
    fr.channels = e->channels;
    fr.granules = e->granules;
    int slot = bytes - 4 - side;

    /* Reservoir: bytes queued for earlier frames' slots that are still free. */
    int free_before = e->slots_pending - e->fifo_len;
    if (free_before > e->max_reservoir) {
        int stuff = free_before - e->max_reservoir;
        memset(e->fifo + e->fifo_len, 0, (size_t)stuff);
        e->fifo_len += stuff;
        free_before = e->max_reservoir;
    }
    fr.main_data_begin = free_before;
    int budget = (free_before + slot) * 8;

    /* Spectra and masking for every granule and channel. */
    float xr[2][2][576];
    mask m[2][2];
    for (int g = 0; g < e->granules; g++) {
        granule_in *cur = e->queue[1 + g];
        for (int ch = 0; ch < e->channels; ch++) {
            transform(e, (const float (*)[32])e->queue[g][ch].sub, (const float (*)[32])cur[ch].sub, block_types[g], xr[g][ch]);
            if (block_types[g] == 2) mask_short(e, ch, xr[g][ch], &m[g][ch]);
            else mask_long(e, ch, xr[g][ch], &m[g][ch]);
        }
    }
    bool ms = false;
    if (e->channels == 2) ms_decision(e, xr[0][0], xr[0][1], &ms);
    fr.mode_ext = ms ? 2 : 0;
    if (ms) {
        for (int g = 0; g < e->granules; g++) {
            for (int j = 0; j < 576; j++) {
                float l = xr[g][0][j], r = xr[g][1][j];
                xr[g][0][j] = (l + r) * 0.70710678f;
                xr[g][1][j] = (l - r) * 0.70710678f;
            }
            for (int b = 0; b < 22; b++) {
                float t = m[g][0].xmin[b] < m[g][1].xmin[b] ? m[g][0].xmin[b] : m[g][1].xmin[b];
                m[g][0].xmin[b] = m[g][1].xmin[b] = t;
            }
            for (int b = 0; b < 13; b++)
                for (int w = 0; w < 3; w++) {
                    float t = m[g][0].xmin_s[b][w] < m[g][1].xmin_s[b][w] ? m[g][0].xmin_s[b][w] : m[g][1].xmin_s[b][w];
                    m[g][0].xmin_s[b][w] = m[g][1].xmin_s[b][w] = t;
                }
            if (!e->vbr) continue;
            /* Variable bit rate codes each band only as finely as its
               threshold asks, so the side channel needs one of its own: under
               the left/right threshold alone it would often be dropped
               whole, and the stereo picture with it. */
            for (int ch = 0; ch < 2; ch++) {
                mask own;
                if (!e->prev_ms)
                    for (int b = 0; b < 22; b++) e->prev_xmin[2 + ch][b] = 1e30f;
                if (block_types[g] == 2) mask_short(e, 2 + ch, xr[g][ch], &own);
                else mask_long(e, 2 + ch, xr[g][ch], &own);
                for (int b = 0; b < 22; b++)
                    if (own.xmin[b] < m[g][ch].xmin[b]) m[g][ch].xmin[b] = own.xmin[b];
                for (int b = 0; b < 13; b++)
                    for (int w = 0; w < 3; w++)
                        if (own.xmin_s[b][w] < m[g][ch].xmin_s[b][w]) m[g][ch].xmin_s[b][w] = own.xmin_s[b][w];
            }
            e->prev_ms = true;
        }
    }
    if (!ms) e->prev_ms = false;

    /* Share the bits: an even split, moved toward the granules and channels
       that need more, while keeping some reservoir for hard passages. */
    int parts = e->granules * e->channels;
    float pe_total = 0.0f;
    for (int g = 0; g < e->granules; g++)
        for (int ch = 0; ch < e->channels; ch++) pe_total += m[g][ch].pe;
    float mean_bits = slot * 8.0f / parts;
    float pe_mean = pe_total / parts;
    e->pe_average = e->pe_average ? 0.9f * e->pe_average + 0.1f * pe_mean : pe_mean;
    int extra = free_before * 8;
    float want_extra = pe_mean > e->pe_average * 1.1f ? (float)extra * 0.6f : (float)extra * 0.15f;
    int remaining = budget;
    for (int g = 0; g < e->granules; g++) {
        for (int ch = 0; ch < e->channels; ch++) {
            float share = pe_total > 0.0f ? m[g][ch].pe / pe_total : 1.0f / parts;
            float evenly = 1.0f / parts;
            int target = (int)(mean_bits + want_extra * (0.5f * share + 0.5f * evenly));
            int left_parts = (e->granules - g) * e->channels - ch;
            int cap = remaining - (left_parts - 1) * 8;
            if (target > cap) target = cap;
            if (target > 4095) target = 4095;
            if (target < 0) target = 0;
            ahmp3_gr *G = &fr.gr[g][ch];
            memset(G, 0, sizeof(*G));
            G->block_type = block_types[g];
            G->window_switching = block_types[g] != 0;
            G->long_end = block_types[g] == 2 ? 0 : 22;
            G->short_start = block_types[g] == 2 ? 0 : 13;
            if (e->vbr) code_granule_vbr(e, xr[g][ch], &m[g][ch], G, 4095);
            else code_granule(e, xr[g][ch], &m[g][ch], G, target);
            remaining -= G->part23;
        }
    }
    if (e->vbr && remaining < 0) {
        /* The frame wants more than the biggest one holds: every part gives
           up the same share. */
        int wanted = budget - remaining;
        remaining = budget;
        for (int i = 0; i < parts; i++) {
            ahmp3_gr *G = &fr.gr[i / e->channels][i % e->channels];
            int limit = (int)((long long)G->part23 * budget / wanted);
            code_granule_vbr(e, xr[i / e->channels][i % e->channels], &m[i / e->channels][i % e->channels], G, limit);
            remaining -= G->part23;
        }
    }
    if (e->vbr) {
        int used_bytes = (budget - remaining + 7) / 8 - free_before;
        int br = 1;
        while (br < e->bitrate_index && vbr_frame_bytes(e, br) - 4 - side < used_bytes) br++;
        fr.bitrate_index = br;
        remaining -= (slot - (vbr_frame_bytes(e, br) - 4 - side)) * 8;
        slot = vbr_frame_bytes(e, br) - 4 - side;
    }
    /* Never more than the frame can carry: silence granules from the end
       until it fits. Only a pathological input could need this. */
    for (int i = parts - 1; i >= 0 && remaining < 0; i--) {
        ahmp3_gr *G = &fr.gr[i / e->channels][i % e->channels];
        remaining += G->part23;
        memset(G->ix, 0, sizeof(G->ix));
        memset(G->sf, 0, sizeof(G->sf));
        G->global_gain = 0;
        G->scalefac_scale = 0;
        G->part23 = granule_bits(e, G, true);
        remaining -= G->part23;
    }

    unsigned char main_buf[4096];
    int main_bits = ahmp3_write_main(&fr, main_buf, sizeof(main_buf));
    int main_bytes = (main_bits + 7) / 8;
    pending *p = &e->frames[e->frame_count++];
    p->head_bytes = ahmp3_write_header(&fr, p->head);
    p->slot = slot;
    e->slots_pending += slot;
    memcpy(e->fifo + e->fifo_len, main_buf, (size_t)main_bytes);
    e->fifo_len += main_bytes;
}

/* Codes the oldest frame of the queue once its lookahead granule is in. */
static size_t step(ahmp3_encoder *e, uint8_t *out, size_t cap)
{
    if (e->queued < e->granules + 1) return 0;

    /* Block types: short around attacks, with start and stop windows to
       switch in and out. Both channels share them so mid/side stays valid. */
    bool shortb[3];
    for (int g = 0; g <= e->granules; g++) {
        shortb[g] = false;
        for (int ch = 0; ch < e->channels; ch++)
            if (attack(&e->queue[g][ch], &e->queue[1 + g][ch])) shortb[g] = true;
    }
    int types[2];
    int prev = e->prev_block;
    for (int g = 0; g < e->granules; g++) {
        bool next_short = shortb[g + 1];
        bool prev_short = prev == 2;
        if (shortb[g] || (prev_short && next_short)) types[g] = 2;
        else if (next_short) types[g] = 1;
        else if (prev_short) types[g] = 3;
        else types[g] = 0;
        /* A start window must lead into a short block. */
        if (prev == 1 && types[g] != 2) types[g] = 2;
        prev = types[g];
    }
    e->prev_block = prev;

    /* A caller that gives no room for output must not overflow the queue:
       the oldest frame is dropped instead. */
    while (e->frame_count >= MAX_PENDING - 1 || e->fifo_len > FIFO_BYTES - 4096) {
        int take = e->frames[0].slot < e->fifo_len ? e->frames[0].slot : e->fifo_len;
        memmove(e->fifo, e->fifo + take, (size_t)(e->fifo_len - take));
        e->fifo_len -= take;
        e->slots_pending -= e->frames[0].slot;
        memmove(e->frames, e->frames + 1, (size_t)(e->frame_count - 1) * sizeof(pending));
        e->frame_count--;
    }
    code_frame(e, types);

    /* Slide the queue: the last granule coded becomes the one before the
       next frame, and the lookahead granule its first. */
    memcpy(e->queue[0], e->queue[e->granules], sizeof(e->queue[0]));
    memcpy(e->queue[1], e->queue[e->granules + 1], sizeof(e->queue[0]));
    e->queued = 1;
    return emit(e, out, cap, false);
}

/* queue[0] is the granule before the next frame, queue[1..granules] the
   frame, queue[granules + 1] the lookahead. */
static size_t push(ahmp3_encoder *e, uint8_t *out, size_t cap)
{
    int slot = ++e->queued;
    for (int ch = 0; ch < e->channels; ch++) analyze_granule(e, &e->queue[slot][ch], ch, e->in[ch]);
    e->in_len = 0;
    return step(e, out, cap);
}

/* Input outside [-2, 2], or not a number, is clipped or silenced. */
static float clean(float v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    if ((bits & 0x7F800000u) == 0x7F800000u) return 0; /* NaN or infinity (checked by bits: fast math) */
    return v > 2 ? 2 : v < -2 ? -2 : v;
}

size_t ahmp3_encode(ahmp3_encoder *e, const float *pcm, size_t samples, uint8_t *out, size_t out_cap)
{
    size_t written = 0;
    for (size_t i = 0; i < samples; i++) {
        if (e->channels == 1) {
            float v = clean(pcm[i * e->channels_in]);
            if (e->channels_in == 2) v = (v + clean(pcm[i * 2 + 1])) * 0.5f;
            e->in[0][e->in_len] = v;
        } else {
            e->in[0][e->in_len] = clean(pcm[i * e->channels_in]);
            e->in[1][e->in_len] = clean(pcm[i * e->channels_in + (e->channels_in == 2 ? 1 : 0)]);
        }
        if (++e->in_len == 576) written += push(e, out + written, out_cap - written);
    }
    e->samples_in += (long long)samples;
    return written;
}

size_t ahmp3_encode_s16(ahmp3_encoder *e, const int16_t *pcm, size_t samples, uint8_t *out, size_t out_cap)
{
    const float inv = 1.0f / 32768.0f;
    size_t written = 0;
    if (e->channels == 2 && e->channels_in == 2) {
        size_t i = 0;
        while (i < samples) {
            size_t take = (size_t)(576 - e->in_len);
            if (take > samples - i) take = samples - i;
            float *l = e->in[0] + e->in_len;
            float *r = e->in[1] + e->in_len;
            const int16_t *s = pcm + i * 2;
            for (size_t k = 0; k < take; k++) {
                l[k] = (float)s[2 * k] * inv;
                r[k] = (float)s[2 * k + 1] * inv;
            }
            e->in_len += (int)take;
            i += take;
            if (e->in_len == 576) written += push(e, out + written, out_cap - written);
        }
        e->samples_in += (long long)samples;
        return written;
    }
    float buf[1024];
    size_t per = 1024 / (size_t)e->channels_in;
    for (size_t i = 0; i < samples; i += per) {
        size_t n = samples - i < per ? samples - i : per;
        for (size_t j = 0; j < n * (size_t)e->channels_in; j++) buf[j] = (float)pcm[i * e->channels_in + j] * inv;
        written += ahmp3_encode(e, buf, n, out + written, out_cap - written);
    }
    return written;
}

size_t ahmp3_encoder_flush(ahmp3_encoder *e, uint8_t *out, size_t out_cap)
{
    size_t written = 0;
    if (!e->flushed) {
        e->flushed = 1;
        /* Enough silence to carry the last input through the filter bank,
           the lookahead granule and the last frame. */
        float zero[2 * 576] = {0};
        int granules = 4 + e->granules;
        long long samples_in = e->samples_in;
        for (int i = 0; i < granules; i++) written += ahmp3_encode(e, zero, 576, out + written, out_cap - written);
        e->samples_in = samples_in; /* the silence is not the caller's input */
    }
    written += emit(e, out + written, out_cap - written, true);
    return written;
}

/* ---- info frame ---------------------------------------------------------------- */

static void put_be32(unsigned char *p, uint32_t v)
{
    p[0] = (unsigned char)(v >> 24);
    p[1] = (unsigned char)(v >> 16);
    p[2] = (unsigned char)(v >> 8);
    p[3] = (unsigned char)v;
}

/* The frame that holds the tag: the stream's own bit rate when the tag fits
   in it, else the lowest one that is big enough. Returns its bytes. */
static int info_frame_layout(const ahmp3_encoder *e, int *bitrate_index, int *side)
{
    *side = e->lsf ? (e->channels == 1 ? 9 : 17) : (e->channels == 1 ? 17 : 32);
    int need = 4 + *side + 120 + 36;
    for (int br = e->vbr ? 1 : e->bitrate_index; br < 15; br++) {
        int bytes = (e->lsf ? 72000 : 144000) * ahmp3_bitrates[e->lsf][br] / ahmp3_sample_rates[e->sr_index];
        if (bytes >= need) {
            *bitrate_index = br;
            return bytes;
        }
    }
    return 0;
}

size_t ahmp3_encoder_info_frame(const ahmp3_encoder *e, uint8_t *out, size_t out_cap)
{
    int br = 0, side = 0;
    int bytes = info_frame_layout(e, &br, &side);
    if (!bytes || out_cap < (size_t)bytes) return 0;
    memset(out, 0, (size_t)bytes);

    ahmp3_fr fr;
    memset(&fr, 0, sizeof(fr));
    fr.lsf = e->lsf;
    fr.sr_index = e->sr_index;
    fr.bitrate_index = br;
    fr.mode = e->mode;
    fr.channels = e->channels;
    fr.granules = e->granules;
    unsigned char head[40];
    ahmp3_write_header(&fr, head);
    memcpy(out, head, 4); /* the side information stays zero: a silent frame */

    unsigned char *p = out + 4 + side;
    uint64_t total = e->bytes_out + (uint64_t)bytes;
    uint32_t total32 = total > 0xFFFFFFFFu ? 0xFFFFFFFFu : (uint32_t)total;
    memcpy(p, e->vbr ? "Xing" : "Info", 4);
    put_be32(p + 4, 0x0F); /* frames, bytes, seek table and quality follow */
    put_be32(p + 8, e->frames_out);
    put_be32(p + 12, total32);
    /* Seek table: where in the file each hundredth of the playing time
       starts, as a fraction of 256. */
    for (int i = 0; i < 100; i++) {
        uint64_t at = 0;
        if (e->seek_count) {
            uint64_t frame = (uint64_t)i * e->frames_out / 100;
            int k = (int)(frame / (uint64_t)e->seek_stride);
            if (k >= e->seek_count) k = e->seek_count - 1;
            at = (e->seek[k] + (uint64_t)bytes) * 256 / total;
        }
        p[16 + i] = (unsigned char)(at > 255 ? 255 : at);
    }
    put_be32(p + 116, (uint32_t)(e->vbr ? e->vbr * 10 : 0));

    /* The LAME tag. Its name starts with "LAME" because players look for
       that before they trust the delay and padding further down. */
    unsigned char *t = p + 120;
    memcpy(t, "LAMEahmp3", 9);
    t[9] = e->vbr ? 3 : 1; /* tag revision 0; variable or constant bit rate */
    double lowpass = e->cutoff * ahmp3_sample_rates[e->sr_index] / 2 / 100;
    t[10] = (unsigned char)(lowpass > 255 ? 255 : lowpass);
    /* 11..18: peak level and replay gain, not measured. 19: flags, none. */
    int kbps = ahmp3_bitrates[e->lsf][e->vbr ? 1 : e->bitrate_index];
    t[20] = (unsigned char)(kbps > 255 ? 255 : kbps);
    int delay = ahmp3_encoder_delay(e);
    long long padding = (long long)e->frames_out * (e->lsf ? 576 : 1152) - delay - e->samples_in;
    if (padding < 0) padding = 0;
    if (padding > 4095) padding = 4095;
    t[21] = (unsigned char)(delay >> 4);
    t[22] = (unsigned char)((delay & 15) << 4 | (int)(padding >> 8));
    t[23] = (unsigned char)padding;
    int rate = ahmp3_sample_rates[e->sr_index];
    int stereo_mode = e->mode == 3 ? 0 : e->mode == 0 ? 1 : 3;
    t[24] = (unsigned char)(stereo_mode << 2 | (rate <= 32000 ? 0 : rate == 44100 ? 1 : 2) << 6);
    put_be32(t + 28, total32);
    t[32] = (unsigned char)(e->music_crc >> 8);
    t[33] = (unsigned char)e->music_crc;
    uint16_t crc = crc_a001(0, out, (size_t)(t + 34 - out));
    t[34] = (unsigned char)(crc >> 8);
    t[35] = (unsigned char)crc;
    return (size_t)bytes;
}
