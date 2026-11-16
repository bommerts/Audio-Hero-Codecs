/* AAC-LC decoder, following ISO/IEC 14496-3 subpart 4 (and 13818-7, the
   same coding in MPEG-2): ADTS framing, the syntactic elements, Huffman
   coded spectra, pulses, mid/side and intensity stereo, perceptual noise
   substitution, temporal noise shaping and the windowed inverse MDCT.
   Where the standard leaves room, it behaves like the decoders in common
   use, so files that play elsewhere play here the same. */
#include "ahaac_internal.h"

#define MAX_SFB 51

/* ---- bits -------------------------------------------------------------------- */

typedef struct {
    const uint8_t *p;
    size_t bits; /* total */
    size_t pos;  /* may run past the end; zeros are read there */
} reader;

static unsigned get1(reader *b)
{
    unsigned bit = 0;
    if (b->pos < b->bits) bit = (b->p[b->pos >> 3] >> (7 - (b->pos & 7))) & 1;
    b->pos++;
    return bit;
}

static unsigned get(reader *b, int n)
{
    unsigned v = 0;
    while (n-- > 0) v = v << 1 | get1(b);
    return v;
}

static bool over(const reader *b)
{
    return b->pos > b->bits;
}

/* ---- Huffman trees ----------------------------------------------------------- */

/* A decoding tree: node pairs, leaves stored as ~value. */
typedef struct {
    short node[2];
} hnode;

typedef struct {
    int window_sequence, window_shape;
    int max_sfb, num_windows, num_groups;
    int group_len[8];
    const ahaac_bands *bands;
} ics_info;

typedef struct {
    ics_info ics;
    unsigned char cb[8][MAX_SFB];  /* code book of each band, by window group */
    float scale[8][MAX_SFB];       /* its gain, intensity factor or noise level */
    bool pulses;
    int pulse_count, pulse_start, pulse_offset[4], pulse_amp[4];
    bool tns;
    int tns_filters[8], tns_length[8][4], tns_order[8][4], tns_backward[8][4];
    float tns_lpc[8][4][13];
    float coef[1024];
} channel;

struct ahaac_decoder {
    hnode pool[1600];
    int pool_used;
    const hnode *book[12], *sf_book;

    float sine_long[1024], sine_short[128], kbd_long[1024], kbd_short[128]; /* rising window halves */
    float pow43[8192 + 16];                    /* n^(4/3) */
    float twist_re[2][512], twist_im[2][512];  /* MDCT twiddles: long, short */
    float fft_cos[256], fft_sin[256];
    unsigned short fft_order[512];             /* bit-reversed positions */

    bool configured;     /* set up from an AudioSpecificConfig, for raw packets */
    int sr_index;
    int channel_config;  /* 0 = whatever the stream's elements say */
    float overlap[2][1024];
    int prev_shape[2];
    unsigned noise_seed;
    bool sbr;

    bool synced;         /* ADTS */
    unsigned char fixed[3];
    size_t skip_pending;
    const char *error;

    channel ch[2];
    int quant[1024];
};

static const hnode *tree_build(ahaac_decoder *d, const unsigned short *codes16, const unsigned int *codes32, const unsigned char *lens,
                               int count)
{
    hnode *nodes = d->pool + d->pool_used;
    int used = 1;
    nodes[0].node[0] = nodes[0].node[1] = 0;
    for (int v = 0; v < count; v++) {
        unsigned code = codes16 ? codes16[v] : codes32[v];
        int n = 0;
        for (int b = lens[v] - 1; b >= 0; b--) {
            int bit = (code >> b) & 1;
            if (b == 0) {
                nodes[n].node[bit] = (short)~v;
            } else {
                if (nodes[n].node[bit] == 0) {
                    nodes[n].node[bit] = (short)used;
                    nodes[used].node[0] = nodes[used].node[1] = 0;
                    used++;
                }
                n = nodes[n].node[bit];
            }
        }
    }
    d->pool_used += used;
    return nodes;
}

/* The value of the next code word, or -1 for bits that are no code. */
static int tree_read(const hnode *nodes, reader *b)
{
    int n = 0;
    for (int depth = 0; depth < 20; depth++) {
        int next = nodes[n].node[get1(b)];
        if (next < 0) return ~next;
        if (next == 0) return -1;
        n = next;
    }
    return -1;
}

/* ---- setup -------------------------------------------------------------------- */

/* Modified Bessel function of the first kind, order 0, by its series. */
static double bessel_i0(double x)
{
    double sum = 1, term = 1;
    for (int k = 1; k < 60; k++) {
        term *= (x / 2 / k) * (x / 2 / k);
        sum += term;
    }
    return sum;
}

/* The rising half of a Kaiser-Bessel derived window of 2n samples. */
static void kbd_window(float *w, int n, double alpha)
{
    double total = 0, running = 0;
    for (int p = 0; p <= n; p++) {
        double r = 2.0 * p / n - 1;
        total += bessel_i0(AHAAC_PI * alpha * sqrt(1 - r * r));
    }
    for (int i = 0; i < n; i++) {
        double r = 2.0 * i / n - 1;
        running += bessel_i0(AHAAC_PI * alpha * sqrt(1 - r * r));
        w[i] = (float)sqrt(running / total);
    }
}

ahaac_decoder *ahaac_decoder_new(void)
{
    ahaac_decoder *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    d->sf_book = tree_build(d, NULL, ahaac_sf_codes, ahaac_sf_bits, 121);
    for (int i = 1; i < 12; i++) d->book[i] = tree_build(d, ahaac_books[i].codes, NULL, ahaac_books[i].bits, ahaac_books[i].count);

    for (int i = 0; i < 1024; i++) d->sine_long[i] = (float)sin(AHAAC_PI / 2048 * (i + 0.5));
    for (int i = 0; i < 128; i++) d->sine_short[i] = (float)sin(AHAAC_PI / 256 * (i + 0.5));
    kbd_window(d->kbd_long, 1024, 4);
    kbd_window(d->kbd_short, 128, 6);
    for (int i = 0; i < 8192 + 16; i++) d->pow43[i] = (float)pow(i, 4.0 / 3.0);

    for (int kind = 0; kind < 2; kind++) {
        int lines = kind ? 128 : 1024;
        for (int j = 0; j < lines / 2; j++) {
            double angle = AHAAC_PI * (8 * j + 1) / (8.0 * lines);
            d->twist_re[kind][j] = (float)cos(angle);
            d->twist_im[kind][j] = (float)-sin(angle);
        }
    }
    for (int k = 0; k < 256; k++) {
        d->fft_cos[k] = (float)cos(2 * AHAAC_PI * k / 512);
        d->fft_sin[k] = (float)sin(2 * AHAAC_PI * k / 512);
    }
    for (int i = 0; i < 512; i++) {
        int r = 0;
        for (int bit = 0; bit < 9; bit++)
            if (i & (1 << bit)) r |= 256 >> bit;
        d->fft_order[i] = (unsigned short)r;
    }
    d->noise_seed = 0x1f2e3d4c;
    d->sr_index = -1;
    return d;
}

void ahaac_decoder_free(ahaac_decoder *d)
{
    free(d);
}

void ahaac_decoder_reset(ahaac_decoder *d)
{
    memset(d->overlap, 0, sizeof(d->overlap));
    d->prev_shape[0] = d->prev_shape[1] = 0;
    d->synced = false;
    d->skip_pending = 0;
}

const char *ahaac_decoder_error(const ahaac_decoder *d)
{
    return d->error;
}

/* ---- the inverse MDCT --------------------------------------------------------- */

/* y[n] = sum of X[k] cos(pi/M (n + 1/2 + M/2)(k + 1/2)) over the M lines, for
   n = 0 .. 2M-1, times 1 / (32768 M).

   The 2M outputs are a type IV cosine transform of the lines, unfolded. That
   transform comes from one complex FFT of M/2 points: pair the lines up as
   X[2j] + i X[M-1-2j], turn each by exp(-i pi (8j + 1) / 8M), transform,
   turn again, and the real and imaginary parts are its outputs. */
void ahaac_imdct(const ahaac_decoder *d, const float *X, float *out, int M)
{
    int half = M / 2, kind = M == 1024 ? 0 : 1, shift = kind ? 3 : 0;
    const float *wr = d->twist_re[kind], *wi = d->twist_im[kind];
    float re[512], im[512], u[1024];
    for (int j = 0; j < half; j++) {
        float a = X[2 * j], b = X[M - 1 - 2 * j];
        int k = d->fft_order[j] >> shift;
        re[k] = a * wr[j] - b * wi[j];
        im[k] = a * wi[j] + b * wr[j];
    }
    for (int len = 2; len <= half; len <<= 1) {
        int h = len / 2, step = 512 / len;
        for (int i = 0; i < half; i += len) {
            for (int k = 0; k < h; k++) {
                float c = d->fft_cos[k * step], s = -d->fft_sin[k * step];
                float tr = re[i + k + h] * c - im[i + k + h] * s, ti = re[i + k + h] * s + im[i + k + h] * c;
                re[i + k + h] = re[i + k] - tr;
                im[i + k + h] = im[i + k] - ti;
                re[i + k] += tr;
                im[i + k] += ti;
            }
        }
    }
    float scale = 1.0f / (32768.0f * (float)M);
    for (int n = 0; n < half; n++) {
        u[2 * n] = (re[n] * wr[n] - im[n] * wi[n]) * scale;
        u[M - 1 - 2 * n] = -(re[n] * wi[n] + im[n] * wr[n]) * scale;
    }
    for (int n = 0; n < half; n++) out[n] = u[n + half];
    for (int n = half; n < 3 * half; n++) out[n] = -u[3 * half - 1 - n];
    for (int n = 3 * half; n < 4 * half; n++) out[n] = -u[n - 3 * half];
}

/* One channel's spectrum to 1024 samples: transform, window by the block
   type, and add the second half of the frame before. */
static void filter_bank(ahaac_decoder *d, int c, const channel *ch, float *pcm, int stride)
{
    const ics_info *ics = &ch->ics;
    const float *prev_long = d->prev_shape[c] ? d->kbd_long : d->sine_long, *prev_short = d->prev_shape[c] ? d->kbd_short : d->sine_short;
    const float *cur_long = ics->window_shape ? d->kbd_long : d->sine_long, *cur_short = ics->window_shape ? d->kbd_short : d->sine_short;
    float y[2048], z[2048];
    if (ics->window_sequence == AHAAC_EIGHT_SHORT) {
        /* Eight short blocks, each overlapping the next by half, in the
           middle of the frame. */
        memset(z, 0, sizeof(z));
        for (int w = 0; w < 8; w++) {
            ahaac_imdct(d, ch->coef + 128 * w, y, 128);
            const float *rise = w ? cur_short : prev_short;
            float *at = z + 448 + 128 * w;
            for (int i = 0; i < 128; i++) {
                at[i] += y[i] * rise[i];
                at[128 + i] += y[128 + i] * cur_short[127 - i];
            }
        }
    } else {
        ahaac_imdct(d, ch->coef, y, 1024);
        if (ics->window_sequence == AHAAC_LONG_STOP) {
            /* Rises like a short block, late. */
            for (int n = 0; n < 448; n++) z[n] = 0;
            for (int n = 0; n < 128; n++) z[448 + n] = y[448 + n] * prev_short[n];
            for (int n = 576; n < 1024; n++) z[n] = y[n];
        } else {
            for (int n = 0; n < 1024; n++) z[n] = y[n] * prev_long[n];
        }
        if (ics->window_sequence == AHAAC_LONG_START) {
            /* Falls like a short block, early. */
            for (int n = 1024; n < 1472; n++) z[n] = y[n];
            for (int n = 0; n < 128; n++) z[1472 + n] = y[1472 + n] * cur_short[127 - n];
            for (int n = 1600; n < 2048; n++) z[n] = 0;
        } else {
            for (int n = 0; n < 1024; n++) z[1024 + n] = y[1024 + n] * cur_long[1023 - n];
        }
    }
    float *held = d->overlap[c];
    for (int n = 0; n < 1024; n++) {
        pcm[n * stride] = z[n] + held[n];
        held[n] = z[1024 + n];
    }
    d->prev_shape[c] = ics->window_shape;
}

/* ---- one channel -------------------------------------------------------------- */

static bool read_ics_info(const ahaac_decoder *d, reader *b, ics_info *ics)
{
    if (get1(b)) return false; /* reserved */
    ics->window_sequence = (int)get(b, 2);
    ics->window_shape = (int)get1(b);
    ics->num_groups = 1;
    ics->group_len[0] = 1;
    if (ics->window_sequence == AHAAC_EIGHT_SHORT) {
        ics->max_sfb = (int)get(b, 4);
        /* A 1 bit joins a window to the group of the one before. */
        unsigned grouping = get(b, 7);
        for (int i = 0; i < 7; i++) {
            if (grouping & (64u >> i)) ics->group_len[ics->num_groups - 1]++;
            else ics->group_len[ics->num_groups++] = 1;
        }
        ics->num_windows = 8;
        ics->bands = &ahaac_bands_short[d->sr_index];
    } else {
        ics->max_sfb = (int)get(b, 6);
        if (get1(b)) return false; /* prediction: not part of AAC-LC */
        ics->num_windows = 1;
        ics->bands = &ahaac_bands_long[d->sr_index];
    }
    return ics->max_sfb <= ics->bands->count;
}

/* Reads one channel: code books, scale factors, pulses, noise shaping
   filters and spectrum, leaving the scaled spectrum in ch->coef. `left` and
   `ms_used` are given for the second channel of a pair, whose noise bands may
   share the first channel's noise. */
static bool read_channel(ahaac_decoder *d, reader *b, channel *ch, bool common_window, const channel *left,
                         const unsigned char ms_used[8][MAX_SFB])
{
    int global_gain = (int)get(b, 8);
    if (!common_window && !read_ics_info(d, b, &ch->ics)) return false;
    const ics_info *ics = &ch->ics;
    bool is_short = ics->window_sequence == AHAAC_EIGHT_SHORT;
    const unsigned short *swb = ics->bands->offset;

    /* Sections: runs of bands that share a code book. */
    int run_bits = is_short ? 3 : 5, run_escape = (1 << run_bits) - 1;
    for (int g = 0; g < ics->num_groups; g++) {
        for (int k = 0; k < ics->max_sfb;) {
            int cb = (int)get(b, 4), len = 0, more;
            do {
                more = (int)get(b, run_bits);
                len += more;
            } while (more == run_escape && !over(b));
            if (cb == 12 || len == 0 || k + len > ics->max_sfb || over(b)) return false;
            for (int i = 0; i < len; i++) ch->cb[g][k++] = (unsigned char)cb;
        }
    }

    /* Scale factors, each coded as the difference from the one before of
       its kind: gains, intensity stereo positions, noise levels. */
    int gain = global_gain, position = 0, noise = global_gain - 90;
    bool first_noise = true;
    for (int g = 0; g < ics->num_groups; g++) {
        for (int k = 0; k < ics->max_sfb; k++) {
            int cb = ch->cb[g][k];
            if (cb == AHAAC_ZERO) {
                ch->scale[g][k] = 0;
                continue;
            }
            int diff = 0;
            if (cb == AHAAC_NOISE && first_noise) {
                first_noise = false;
                diff = (int)get(b, 9) - 256;
            } else {
                diff = tree_read(d->sf_book, b);
                if (diff < 0) return false;
                diff -= 60;
            }
            if (cb == AHAAC_INTENSITY || cb == AHAAC_INTENSITY2) {
                position += diff;
                int p = position < -155 ? -155 : position > 100 ? 100 : position;
                ch->scale[g][k] = (float)pow(2.0, -0.25 * p);
            } else if (cb == AHAAC_NOISE) {
                noise += diff;
                int n = noise < -100 ? -100 : noise > 155 ? 155 : noise;
                ch->scale[g][k] = (float)pow(2.0, 0.25 * n);
            } else {
                gain += diff;
                if (gain < 0 || gain > 255) return false;
                ch->scale[g][k] = (float)pow(2.0, 0.25 * (gain - 100));
            }
        }
    }

    ch->pulses = get1(b) != 0;
    if (ch->pulses) {
        if (is_short) return false;
        ch->pulse_count = (int)get(b, 2) + 1;
        ch->pulse_start = (int)get(b, 6);
        if (ch->pulse_start >= ics->bands->count) return false;
        for (int i = 0; i < ch->pulse_count; i++) {
            ch->pulse_offset[i] = (int)get(b, 5);
            ch->pulse_amp[i] = (int)get(b, 4);
        }
    }

    ch->tns = get1(b) != 0;
    if (ch->tns) {
        for (int w = 0; w < ics->num_windows; w++) {
            ch->tns_filters[w] = (int)get(b, is_short ? 1 : 2);
            if (!ch->tns_filters[w]) continue;
            int resolution = (int)get1(b) + 3;
            for (int f = 0; f < ch->tns_filters[w]; f++) {
                ch->tns_length[w][f] = (int)get(b, is_short ? 4 : 6);
                int order = ch->tns_order[w][f] = (int)get(b, is_short ? 3 : 5);
                if (order > (is_short ? 7 : 12)) return false;
                if (!order) continue;
                ch->tns_backward[w][f] = (int)get1(b);
                int bits = resolution - (int)get1(b);
                /* The coefficients are quantized reflection values; the
                   filter's direct form is built up from them step by step. */
                double fac = ((1 << (resolution - 1)) - 0.5) / (AHAAC_PI / 2), fac_m = ((1 << (resolution - 1)) + 0.5) / (AHAAC_PI / 2);
                double a[13], next[13];
                a[0] = 1;
                for (int m = 1; m <= order; m++) {
                    int v = (int)get(b, bits);
                    if (v & (1 << (bits - 1))) v -= 1 << bits;
                    double r = sin(v / (v >= 0 ? fac : fac_m));
                    for (int i = 1; i < m; i++) next[i] = a[i] + r * a[m - i];
                    for (int i = 1; i < m; i++) a[i] = next[i];
                    a[m] = r;
                }
                for (int i = 1; i <= order; i++) ch->tns_lpc[w][f][i] = (float)a[i];
            }
        }
    }
    if (get1(b)) return false; /* gain control: not part of AAC-LC */

    /* The spectrum, as whole numbers. In a group of short windows each
       band's lines come window after window. */
    int *quant = d->quant;
    memset(quant, 0, 1024 * sizeof(int));
    for (int g = 0, win = 0; g < ics->num_groups; win += ics->group_len[g], g++) {
        for (int k = 0; k < ics->max_sfb; k++) {
            int cb = ch->cb[g][k];
            if (cb == AHAAC_ZERO || cb > AHAAC_ESCAPE) continue;
            const ahaac_book *book = &ahaac_books[cb];
            int width = swb[k + 1] - swb[k];
            for (int w = 0; w < ics->group_len[g]; w++) {
                int *line = quant + (win + w) * 128 + swb[k];
                for (int i = 0; i < width; i += book->dim) {
                    int index = tree_read(d->book[cb], b), v[4];
                    if (index < 0) return false;
                    for (int j = book->dim - 1; j >= 0; j--) {
                        v[j] = index % book->mod + book->low;
                        index /= book->mod;
                    }
                    if (book->sign_bits)
                        for (int j = 0; j < book->dim; j++)
                            if (v[j] && get1(b)) v[j] = -v[j];
                    if (cb == AHAAC_ESCAPE) {
                        /* 16 stands for a bigger number: n one bits, a zero,
                           then n + 4 bits to add to 2^(n + 4). */
                        for (int j = 0; j < 2; j++) {
                            if (v[j] != 16 && v[j] != -16) continue;
                            int n = 0;
                            while (get1(b)) {
                                if (++n > 8) return false;
                            }
                            int big = (1 << (n + 4)) + (int)get(b, n + 4);
                            v[j] = v[j] < 0 ? -big : big;
                        }
                    }
                    for (int j = 0; j < book->dim; j++) line[i + j] = v[j];
                }
            }
        }
        if (over(b)) return false;
    }

    if (ch->pulses) {
        int k = swb[ch->pulse_start], band = ch->pulse_start;
        for (int i = 0; i < ch->pulse_count; i++) {
            k += ch->pulse_offset[i];
            if (k >= 1024) return false;
            while (band + 1 < ics->bands->count && swb[band + 1] <= k) band++;
            int cb = band < ics->max_sfb ? ch->cb[0][band] : AHAAC_ZERO;
            if (cb == AHAAC_ZERO || cb > AHAAC_ESCAPE) continue;
            if (quant[k] > 0) quant[k] += ch->pulse_amp[i];
            else quant[k] -= ch->pulse_amp[i];
        }
    }

    /* Whole numbers to lines: n^(4/3) times the band's gain. A noise band
       gets random lines, scaled to the level the stream gives. */
    memset(ch->coef, 0, sizeof(ch->coef));
    for (int g = 0, win = 0; g < ics->num_groups; win += ics->group_len[g], g++) {
        for (int k = 0; k < ics->max_sfb; k++) {
            int cb = ch->cb[g][k], width = swb[k + 1] - swb[k];
            float scale = ch->scale[g][k];
            if (cb == AHAAC_ZERO || cb == AHAAC_INTENSITY || cb == AHAAC_INTENSITY2) continue;
            for (int w = 0; w < ics->group_len[g]; w++) {
                int at = (win + w) * 128 + swb[k];
                float *line = ch->coef + at;
                if (cb != AHAAC_NOISE) {
                    for (int i = 0; i < width; i++) {
                        int q = quant[at + i];
                        line[i] = q < 0 ? -d->pow43[-q] * scale : d->pow43[q] * scale;
                    }
                } else if (left && ms_used[g][k] && left->cb[g][k] == AHAAC_NOISE && left->scale[g][k] > 0) {
                    /* Marked to share the first channel's noise. */
                    float ratio = scale / left->scale[g][k];
                    for (int i = 0; i < width; i++) line[i] = left->coef[at + i] * ratio;
                } else {
                    float energy = 0;
                    for (int i = 0; i < width; i++) {
                        d->noise_seed = d->noise_seed * 1664525u + 1013904223u;
                        line[i] = (float)(int32_t)d->noise_seed;
                        energy += line[i] * line[i];
                    }
                    float fit = scale / (float)sqrt(energy);
                    for (int i = 0; i < width; i++) line[i] *= fit;
                }
            }
        }
    }
    return true;
}

/* Temporal noise shaping: an all-pole filter run along the spectrum, up or
   down, over the bands the stream names. */
static void apply_tns(const ahaac_decoder *d, channel *ch)
{
    const ics_info *ics = &ch->ics;
    bool is_short = ics->window_sequence == AHAAC_EIGHT_SHORT;
    const unsigned short *swb = ics->bands->offset;
    int limit = is_short ? ahaac_tns_max_short[d->sr_index] : ahaac_tns_max_long[d->sr_index];
    if (limit > ics->max_sfb) limit = ics->max_sfb;
    for (int w = 0; w < ics->num_windows; w++) {
        int bottom = ics->bands->count;
        for (int f = 0; f < ch->tns_filters[w]; f++) {
            int top = bottom, order = ch->tns_order[w][f];
            bottom = top - ch->tns_length[w][f];
            if (bottom < 0) bottom = 0;
            if (!order) continue;
            int start = swb[bottom < limit ? bottom : limit], end = swb[top < limit ? top : limit];
            int size = end - start;
            if (size <= 0) continue;
            int step = ch->tns_backward[w][f] ? -1 : 1;
            float *x = ch->coef + w * 128 + (step < 0 ? end - 1 : start);
            const float *a = ch->tns_lpc[w][f];
            for (int m = 0; m < size; m++) {
                float y = x[m * step];
                for (int i = 1; i <= order && i <= m; i++) y -= a[i] * x[(m - i) * step];
                x[m * step] = y;
            }
        }
    }
}

/* ---- frames ------------------------------------------------------------------- */

/* A program config element: the channels of a stream whose configuration is
   not one of the standard ones. Returns their number, 0 when the bits ran
   out. */
static int read_program_config(reader *b)
{
    get(b, 4 + 2 + 4);
    int count[4], channels = 0;
    for (int i = 0; i < 3; i++) count[i] = (int)get(b, 4);
    count[3] = (int)get(b, 2);
    int assoc = (int)get(b, 3), coupling = (int)get(b, 4);
    if (get1(b)) get(b, 4);
    if (get1(b)) get(b, 4);
    if (get1(b)) get(b, 3);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < count[i]; j++) {
            channels += get1(b) ? 2 : 1;
            get(b, 4);
        }
    channels += count[3];
    get(b, 4 * count[3] + 4 * assoc + 5 * coupling);
    b->pos = (b->pos + 7) & ~(size_t)7;
    int comment = (int)get(b, 8);
    b->pos += (size_t)comment * 8;
    return over(b) ? 0 : channels;
}

static void unsupported(ahaac_decoder *d, const char *why)
{
    d->error = why;
}

/* Decodes one raw data block. Returns the channels written to pcm, 0 when
   the block is damaged or of a kind not handled. */
static int decode_block(ahaac_decoder *d, const uint8_t *data, size_t len, float *pcm)
{
    reader b = {data, len * 8, 0};
    int channels = 0;
    bool common = false;
    unsigned char ms_used[8][MAX_SFB];
    int ms_present = 0;
    for (;;) {
        if (over(&b)) return 0;
        int id = b.pos + 3 > b.bits ? 7 : (int)get(&b, 3);
        if (id == 7) break; /* end */
        if (id == 0 || id == 1) {
            /* A single channel or a pair. */
            if (channels) {
                unsupported(d, "AAC with more than two channels is not supported");
                return 0;
            }
            get(&b, 4);
            if (id == 0) {
                if (!read_channel(d, &b, &d->ch[0], false, NULL, NULL)) return 0;
                channels = 1;
                continue;
            }
            common = get1(&b) != 0;
            ms_present = 0;
            memset(ms_used, 0, sizeof(ms_used));
            if (common) {
                if (!read_ics_info(d, &b, &d->ch[0].ics)) return 0;
                d->ch[1].ics = d->ch[0].ics;
                ms_present = (int)get(&b, 2);
                if (ms_present == 3) return 0;
                for (int g = 0; g < d->ch[0].ics.num_groups; g++)
                    for (int k = 0; k < d->ch[0].ics.max_sfb; k++) ms_used[g][k] = (unsigned char)(ms_present == 2 || (ms_present == 1 && get1(&b)));
            }
            if (!read_channel(d, &b, &d->ch[0], common, NULL, NULL)) return 0;
            if (!read_channel(d, &b, &d->ch[1], common, common ? &d->ch[0] : NULL, ms_used)) return 0;
            channels = 2;
        } else if (id == 4) {
            /* Data that is not sound. */
            get(&b, 4);
            bool align = get1(&b) != 0;
            size_t count = get(&b, 8);
            if (count == 255) count += get(&b, 8);
            if (align) b.pos = (b.pos + 7) & ~(size_t)7;
            b.pos += count * 8;
        } else if (id == 5) {
            int n = read_program_config(&b);
            if (n < 1) return 0;
            if (n > 2) {
                unsupported(d, "AAC with more than two channels is not supported");
                return 0;
            }
        } else if (id == 6) {
            /* Fill, which is also where HE-AAC keeps its extra data. */
            size_t count = get(&b, 4);
            if (count == 15) count += get(&b, 8) - 1;
            if (count) {
                unsigned kind = get(&b, 4);
                if (kind == 13 || kind == 14) d->sbr = true;
                b.pos += count * 8 - 4;
            }
        } else {
            /* A coupling channel or a low-frequency channel. */
            unsupported(d, "AAC with more than two channels is not supported");
            return 0;
        }
    }
    if (!channels || over(&b)) return 0;

    if (channels == 2 && common) {
        channel *l = &d->ch[0], *r = &d->ch[1];
        const ics_info *ics = &l->ics;
        const unsigned short *swb = ics->bands->offset;
        for (int g = 0, win = 0; g < ics->num_groups; win += ics->group_len[g], g++) {
            for (int k = 0; k < ics->max_sfb; k++) {
                int cb = r->cb[g][k], width = swb[k + 1] - swb[k];
                bool intensity = cb == AHAAC_INTENSITY || cb == AHAAC_INTENSITY2;
                if (!intensity && !(ms_used[g][k] && cb < AHAAC_NOISE && l->cb[g][k] < AHAAC_NOISE)) continue;
                /* Intensity: the right channel is the left, scaled, with its
                   sign from the code book and the mid/side flag. */
                float factor = r->scale[g][k];
                if ((cb == AHAAC_INTENSITY2) != (ms_present == 1 && ms_used[g][k])) factor = -factor;
                for (int w = 0; w < ics->group_len[g]; w++) {
                    float *a = l->coef + (win + w) * 128 + swb[k], *c = r->coef + (win + w) * 128 + swb[k];
                    if (intensity) {
                        for (int i = 0; i < width; i++) c[i] = a[i] * factor;
                    } else {
                        for (int i = 0; i < width; i++) {
                            float mid = a[i], side = c[i];
                            a[i] = mid + side;
                            c[i] = mid - side;
                        }
                    }
                }
            }
        }
    }
    for (int c = 0; c < channels; c++) {
        if (d->ch[c].tns) apply_tns(d, &d->ch[c]);
        filter_bank(d, c, &d->ch[c], pcm + c, channels);
    }
    return channels;
}

static void fill_frame(const ahaac_decoder *d, ahaac_frame *frame, int channels, size_t bytes)
{
    if (!frame) return;
    frame->sample_rate = d->sr_index >= 0 ? ahaac_sample_rates[d->sr_index] : 0;
    frame->channels = channels;
    frame->samples = channels ? AHAAC_MAX_SAMPLES : 0;
    frame->frame_bytes = (int)bytes;
    frame->sbr = d->sbr;
}

/* A frame that could not be decoded leaves nothing to overlap with. */
static void drop_frame(ahaac_decoder *d)
{
    memset(d->overlap, 0, sizeof(d->overlap));
}

/* ---- raw packets (MP4) -------------------------------------------------------- */

bool ahaac_decoder_configure(ahaac_decoder *d, const uint8_t *config, size_t len)
{
    reader b = {config, len * 8, 0};
    int object = (int)get(&b, 5);
    if (object == 31) object = 32 + (int)get(&b, 6);
    int sr = (int)get(&b, 4);
    if (sr == 15) get(&b, 24); /* a rate spelled out: not one AAC has bands for */
    int channels = (int)get(&b, 4);
    if (object == 5 || object == 29) {
        /* HE-AAC stated outright: the rate read above is the plain AAC part's
           own; skip the rate of the extension to reach what is inside. */
        d->sbr = true;
        if (get(&b, 4) == 15) get(&b, 24);
        object = (int)get(&b, 5);
        if (object == 31) object = 32 + (int)get(&b, 6);
    }
    d->configured = false;
    if (over(&b) || sr > 12) {
        unsupported(d, "damaged AAC configuration");
        return false;
    }
    if (object != 2) {
        unsupported(d, object == 1 ? "AAC Main is not supported" : object == 4 ? "AAC LTP is not supported" : "only AAC-LC is supported");
        return false;
    }
    if (get1(&b)) {
        unsupported(d, "AAC with 960-sample frames is not supported");
        return false;
    }
    if (get1(&b)) get(&b, 14); /* depends on a core coder */
    get1(&b);
    if (channels == 0) channels = read_program_config(&b);
    if (channels < 1 || channels > 2) {
        unsupported(d, channels ? "AAC with more than two channels is not supported" : "damaged AAC configuration");
        return false;
    }
    if (sr != d->sr_index) ahaac_decoder_reset(d);
    d->sr_index = sr;
    d->channel_config = channels;
    d->configured = true;
    d->error = NULL;
    return true;
}

bool ahaac_decode_packet(ahaac_decoder *d, const uint8_t *data, size_t len, float *pcm, int *samples, ahaac_frame *frame)
{
    *samples = 0;
    int channels = d->configured ? decode_block(d, data, len, pcm) : 0;
    fill_frame(d, frame, channels, len);
    if (!channels) {
        drop_frame(d);
        return false;
    }
    *samples = AHAAC_MAX_SAMPLES;
    return true;
}

/* ---- ADTS --------------------------------------------------------------------- */

static size_t id3v2_size(const uint8_t *p, size_t len)
{
    if (len < 10 || memcmp(p, "ID3", 3) || p[3] == 0xFF || p[4] == 0xFF || ((p[6] | p[7] | p[8] | p[9]) & 0x80)) return 0;
    return ((size_t)p[6] << 21 | (size_t)p[7] << 14 | (size_t)p[8] << 7 | p[9]) + 10 + ((p[5] & 0x10) ? 10 : 0);
}

/* True for the first bytes of an ADTS header; gives the frame's length. */
static bool adts_header(const uint8_t *p, size_t *frame_bytes)
{
    if (p[0] != 0xFF || (p[1] & 0xF6) != 0xF0) return false;
    if (((p[2] >> 2) & 15) > 12) return false;
    *frame_bytes = ((size_t)(p[3] & 3) << 11) | ((size_t)p[4] << 3) | (p[5] >> 5);
    return *frame_bytes >= ((p[1] & 1) ? 7u : 9u);
}

/* The header fields that stay the same from frame to frame. */
static bool same_stream(const uint8_t *p, const unsigned char fixed[3])
{
    return p[1] == fixed[0] && (p[2] & 0xFD) == fixed[1] && (p[3] & 0xC0) == fixed[2];
}

size_t ahaac_decode(ahaac_decoder *d, const uint8_t *data, size_t len, bool eof, float *pcm, int *samples, ahaac_frame *frame)
{
    *samples = 0;
    if (frame) fill_frame(d, frame, 0, 0);
    if (d->skip_pending) {
        size_t n = d->skip_pending < len ? d->skip_pending : len;
        d->skip_pending -= n;
        return n;
    }

    size_t pos = 0, frame_bytes = 0;
    for (;; pos++) {
        if (pos + 7 > len) {
            /* No frame here. Keep the last bytes: a header may start there. */
            if (eof) return len;
            return len > 6 ? len - 6 : 0;
        }
        const uint8_t *p = data + pos;
        if (!d->synced) {
            size_t tag = id3v2_size(p, len - pos);
            if (tag) {
                if (pos) return pos;
                if (tag <= len) return tag;
                d->skip_pending = tag - len;
                return len;
            }
        }
        if (!adts_header(p, &frame_bytes)) continue;
        bool trusted = d->synced && same_stream(p, d->fixed);
        if (pos + frame_bytes > len) {
            if (!eof) return pos; /* wait for the whole frame */
            if (!trusted) continue;
            return len;           /* a cut-off last frame */
        }
        if (!trusted) {
            /* A false sync is unlikely to be followed by a matching header. */
            unsigned char fixed[3] = {p[1], (unsigned char)(p[2] & 0xFD), (unsigned char)(p[3] & 0xC0)};
            size_t next = pos + frame_bytes, next_bytes;
            if (next + 7 <= len) {
                if (!adts_header(data + next, &next_bytes) || !same_stream(data + next, fixed)) continue;
            } else if (!eof) {
                return pos;
            }
        }
        break;
    }

    const uint8_t *p = data + pos;
    int sr = (p[2] >> 2) & 15;
    if (!d->synced || !same_stream(p, d->fixed) || sr != d->sr_index) ahaac_decoder_reset(d);
    d->synced = true;
    d->fixed[0] = p[1];
    d->fixed[1] = p[2] & 0xFD;
    d->fixed[2] = p[3] & 0xC0;
    d->sr_index = sr;
    d->configured = false;

    size_t head = (p[1] & 1) ? 7 : 9;
    int channels = 0;
    if ((p[2] >> 6) != 1) {
        unsupported(d, (p[2] >> 6) == 0 ? "AAC Main is not supported" : "only AAC-LC is supported");
    } else if ((((p[2] & 1) << 2) | (p[3] >> 6)) > 2) {
        unsupported(d, "AAC with more than two channels is not supported");
    } else if ((p[6] & 3) == 0) { /* one block in the frame, as ever */
        channels = decode_block(d, p + head, frame_bytes - head, pcm);
    }
    fill_frame(d, frame, channels, frame_bytes);
    if (channels) *samples = AHAAC_MAX_SAMPLES;
    else drop_frame(d);
    return pos + frame_bytes;
}
