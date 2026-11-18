/* Tests for the AAC decoder and the M4A reader. */
#include "../src/ahaac_internal.h"
#include "../include/ahm4a.h"
#include "common.h"

typedef struct {
    int frames, failed, channels, rate;
    bool sbr;
} aac_run;

/* Decodes a whole ADTS stream held in memory, `step` bytes more at a time. */
static void adts_decode(const unsigned char *d, size_t len, size_t step, pcm *out, aac_run *run)
{
    ahaac_decoder *dec = ahaac_decoder_new();
    size_t have = step < len ? step : len, pos = 0;
    out->len = 0;
    memset(run, 0, sizeof(*run));
    for (;;) {
        float buf[AHAAC_MAX_SAMPLES * AHAAC_MAX_CHANNELS];
        int n;
        ahaac_frame fr;
        bool eof = have == len;
        size_t used = ahaac_decode(dec, d + pos, have - pos, eof, buf, &n, &fr);
        pos += used;
        if (n) {
            run->frames++;
            run->channels = fr.channels;
            run->rate = fr.sample_rate;
            run->sbr = fr.sbr;
            pcm_put(out, buf, (size_t)n * (size_t)fr.channels);
        } else if (used && fr.frame_bytes) {
            run->failed++;
        }
        if (!used) {
            if (eof) break;
            have = have + step < len ? have + step : len;
        }
    }
    out->channels = run->channels;
    ahaac_decoder_free(dec);
}

static bool ffmpeg_decode(const char *path, const char *extra, pcm *out)
{
    bytes b = {0};
    bool ok = ffmpeg_raw(path, extra, "f32le", &b);
    out->len = 0;
    if (ok) pcm_put(out, (const float *)b.p, b.len / 4);
    free(b.p);
    return ok;
}

/* ---- the transform ------------------------------------------------------------ */

static void test_imdct(void)
{
    ahaac_decoder *d = ahaac_decoder_new();
    for (int kind = 0; kind < 2; kind++) {
        int M = kind ? 128 : 1024;
        float X[1024], fast[2048];
        for (int k = 0; k < M; k++) X[k] = (float)((int)(rnd() & 0xFFFF) - 32768);
        ahaac_imdct(d, X, fast, M);
        double worst = 0, peak = 0;
        for (int n = 0; n < 2 * M; n += kind ? 1 : 7) {
            double sum = 0;
            for (int k = 0; k < M; k++) sum += X[k] * cos(AHAAC_PI / M * (n + 0.5 + M / 2.0) * (k + 0.5));
            sum /= 32768.0 * M;
            if (fabs(sum - fast[n]) > worst) worst = fabs(sum - fast[n]);
            if (fabs(sum) > peak) peak = fabs(sum);
        }
        CHECK(worst < 1e-5 * peak, "the fast transform of %d lines is off by %.2g (peak %.2g)", M, worst, peak);
    }
    ahaac_decoder_free(d);
}

/* ---- an AAC frame writer ------------------------------------------------------ */

/* ffmpeg's AAC encoders use only part of the format, so this builds ADTS
   frames at random that use all of it: every code word of every code book,
   escapes, sections, pulses, noise shaping filters of every order, mid/side
   and intensity stereo, noise bands, every window sequence and shape and
   grouping. ffmpeg then decodes the same frames. */

typedef struct {
    unsigned char p[8192];
    int bits;
} bitw;

static void put(bitw *w, unsigned v, int n)
{
    for (int i = n - 1; i >= 0; i--, w->bits++)
        if (w->bits < 8192 * 8 && ((v >> i) & 1)) w->p[w->bits >> 3] |= (unsigned char)(0x80 >> (w->bits & 7));
}

enum { T_PNS = 1, T_INTENSITY = 2, T_MS = 4, T_TNS = 8, T_PULSE = 16, T_SHORT = 32, T_FILL = 64, T_SHARED_NOISE = 128, T_MS_ALL_INTENSITY = 256 };

typedef struct {
    int sr_index, channels;
    int features;
} aac_kind;

typedef struct {
    int seq, shape, max_sfb, groups, group_len[8];
    unsigned grouping;
    const ahaac_bands *bands;
} t_ics;

typedef struct {
    t_ics ics;
    int cb[8][51], sf[8][51], global_gain;
    int q[1024];
    int pulses, pulse_start, pulse_offset[4], pulse_amp[4];
    int tns, tns_filters[8], tns_res[8], tns_length[8][4], tns_order[8][4], tns_backward[8][4], tns_compress[8][4], tns_coef[8][4][12];
} t_chan;

static int g_book_next[12];           /* the next entry of each book to use */
static unsigned char g_book_used[12][289];
static int g_escape_sizes[9];         /* escapes written, by their size */
static int g_prev_seq[2];             /* the last window sequence of each channel */

static void pick_ics(const aac_kind *k, t_ics *ics, int prev_seq)
{
    /* Window sequences follow the standard's order: short blocks sit
       between a start and a stop window. */
    bool in_short = prev_seq == AHAAC_LONG_START || prev_seq == AHAAC_EIGHT_SHORT;
    if (in_short) ics->seq = rnd_in(0, 9) < 6 ? AHAAC_EIGHT_SHORT : AHAAC_LONG_STOP;
    else ics->seq = (k->features & T_SHORT) && rnd_in(0, 9) < 3 ? AHAAC_LONG_START : AHAAC_ONLY_LONG;
    ics->shape = rnd_in(0, 1);
    ics->groups = 1;
    ics->group_len[0] = 1;
    ics->grouping = 0;
    if (ics->seq == AHAAC_EIGHT_SHORT) {
        ics->bands = &ahaac_bands_short[k->sr_index];
        ics->grouping = rnd() & 127;
        if (rnd_in(0, 4) == 0) ics->grouping = rnd_in(0, 1) ? 127 : 0;
        for (int i = 0; i < 7; i++) {
            if (ics->grouping & (64u >> i)) ics->group_len[ics->groups - 1]++;
            else ics->group_len[ics->groups++] = 1;
        }
    } else {
        ics->bands = &ahaac_bands_long[k->sr_index];
    }
    ics->max_sfb = rnd_in(0, 5) == 0 ? ics->bands->count : rnd_in(0, ics->bands->count);
}

/* Fills in one channel at random. `right` is set for the second channel of
   a pair with a common window, which may use intensity stereo. */
static void pick_channel(const aac_kind *k, t_chan *ch, bool right, const t_chan *left, const unsigned char ms_used[8][51], int ms_present)
{
    const t_ics *ics = &ch->ics;
    const unsigned short *swb = ics->bands->offset;
    bool is_short = ics->seq == AHAAC_EIGHT_SHORT;
    memset(ch->q, 0, sizeof(ch->q));
    memset(ch->cb, 0, sizeof(ch->cb));
    int level = rnd_in(0, 3) ? rnd_in(300, 3000) : rnd_in(20, 9000); /* how loud the frame's lines are */

    for (int g = 0, win = 0; g < ics->groups; win += ics->group_len[g], g++) {
        for (int b = 0; b < ics->max_sfb;) {
            int len = rnd_in(0, 3) ? rnd_in(1, 6) : rnd_in(1, ics->max_sfb);
            if (len > ics->max_sfb - b) len = ics->max_sfb - b;
            int cb = rnd_in(1, 11), r = rnd_in(0, 99);
            if (r < 12) cb = AHAAC_ZERO;
            else if (r < 20 && (k->features & T_PNS)) cb = AHAAC_NOISE;
            else if (r < 32 && right && (k->features & T_INTENSITY)) cb = rnd_in(0, 1) ? AHAAC_INTENSITY : AHAAC_INTENSITY2;
            for (int i = 0; i < len; i++, b++) {
                int use = cb;
                /* Two things this stream leaves alone unless asked: noise
                   bands that share their noise across the pair, and
                   intensity stereo when every band is marked mid/side. */
                if (use == AHAAC_NOISE && right && left->cb[g][b] == AHAAC_NOISE && ms_used[g][b] && !(k->features & T_SHARED_NOISE)) use = AHAAC_ZERO;
                if ((use == AHAAC_INTENSITY || use == AHAAC_INTENSITY2) && ms_present == 2 && !(k->features & T_MS_ALL_INTENSITY)) use = 1;
                ch->cb[g][b] = use;
                if (use == AHAAC_ZERO || use > AHAAC_ESCAPE) continue;
                const ahaac_book *book = &ahaac_books[use];
                int width = swb[b + 1] - swb[b];
                for (int w = 0; w < ics->group_len[g]; w++) {
                    int *line = ch->q + (win + w) * 128 + swb[b];
                    for (int j = 0; j < width; j += book->dim) {
                        int index = rnd_in(0, 2) ? rnd_in(0, book->count - 1) : g_book_next[use]++ % book->count;
                        g_book_used[use][index] = 1;
                        for (int d = book->dim - 1; d >= 0; d--) {
                            int v = index % book->mod + book->low;
                            index /= book->mod;
                            if (book->sign_bits && v && (rnd() & 1)) v = -v;
                            if (use == AHAAC_ESCAPE && (v == 16 || v == -16)) {
                                int n = rnd_in(0, 5) ? rnd_in(0, 2) : rnd_in(0, 8);
                                int big = (1 << (n + 4)) + rnd_in(0, (1 << (n + 4)) - 1);
                                g_escape_sizes[n]++;
                                v = v < 0 ? -big : big;
                            }
                            line[j + d] = v;
                        }
                    }
                }
            }
        }
    }

    /* Gains that keep each band near the frame's level, within reach of the
       band before. */
    int gain = -1, first_gain = -1, position = 0, noise = -1000;
    for (int g = 0, win = 0; g < ics->groups; win += ics->group_len[g], g++) {
        for (int b = 0; b < ics->max_sfb; b++) {
            int cb = ch->cb[g][b], width = swb[b + 1] - swb[b];
            if (cb == AHAAC_ZERO) continue;
            if (cb == AHAAC_INTENSITY || cb == AHAAC_INTENSITY2) {
                position += rnd_in(-6, 6);
                if (position > 30) position = 30;
                if (position < -30) position = -30;
                ch->sf[g][b] = position;
            } else if (cb == AHAAC_NOISE) {
                int want = (int)(4 * log2(level * sqrt((double)width))) + rnd_in(-6, 6);
                if (noise == -1000) noise = want;
                if (want > noise + 60) want = noise + 60;
                if (want < noise - 60) want = noise - 60;
                ch->sf[g][b] = noise = want;
            } else {
                int top = 1;
                for (int w = 0; w < ics->group_len[g]; w++)
                    for (int j = 0; j < width; j++) {
                        int v = abs(ch->q[(win + w) * 128 + swb[b] + j]);
                        if (v > top) top = v;
                    }
                int want = 100 + (int)(4 * log2(level / pow(top, 4.0 / 3.0))) + rnd_in(-8, 8);
                if (want < 0) want = 0;
                if (want > 255) want = 255;
                if (gain < 0) gain = first_gain = want;
                if (want > gain + 60) want = gain + 60;
                if (want < gain - 60) want = gain - 60;
                ch->sf[g][b] = gain = want;
            }
        }
    }
    /* The first gain is the global gain; the first noise level is sent as a
       9-bit number against the global gain less 90. */
    ch->global_gain = first_gain < 0 ? rnd_in(100, 200) : first_gain;
    if (noise != -1000) {
        int first = -1000;
        for (int g = 0; g < ics->groups && first == -1000; g++)
            for (int b = 0; b < ics->max_sfb && first == -1000; b++)
                if (ch->cb[g][b] == AHAAC_NOISE) first = ch->sf[g][b];
        int shift = 0;
        if (first - (ch->global_gain - 90) > 255) shift = ch->global_gain - 90 + 255 - first;
        if (first - (ch->global_gain - 90) < -256) shift = ch->global_gain - 90 - 256 - first;
        for (int g = 0; g < ics->groups; g++)
            for (int b = 0; b < ics->max_sfb; b++)
                if (ch->cb[g][b] == AHAAC_NOISE) ch->sf[g][b] += shift;
    }

    /* Pulses: extra amounts added to single lines, within the bands coded. */
    ch->pulses = !is_short && (k->features & T_PULSE) && ics->max_sfb > 0 && rnd_in(0, 3) == 0;
    if (ch->pulses) {
        int count = rnd_in(1, 4), start = rnd_in(0, ics->max_sfb - 1), at = swb[start], end = swb[ics->max_sfb];
        ch->pulses = 0;
        ch->pulse_start = start;
        for (int i = 0; i < count; i++) {
            int offset = rnd_in(0, 31);
            if (at + offset >= end) break;
            at += offset;
            ch->pulse_offset[i] = offset;
            ch->pulse_amp[i] = rnd_in(0, 15);
            ch->pulses = i + 1;
        }
    }

    ch->tns = (k->features & T_TNS) && rnd_in(0, 2) == 0;
    for (int w = 0; ch->tns && w < (is_short ? 8 : 1); w++) {
        ch->tns_filters[w] = is_short ? rnd_in(0, 1) : rnd_in(0, 3);
        ch->tns_res[w] = rnd_in(0, 1);
        for (int f = 0; f < ch->tns_filters[w]; f++) {
            ch->tns_length[w][f] = rnd_in(0, is_short ? 15 : 63);
            ch->tns_order[w][f] = rnd_in(0, 3) ? rnd_in(0, is_short ? 7 : 12) : 0;
            ch->tns_backward[w][f] = rnd_in(0, 1);
            ch->tns_compress[w][f] = rnd_in(0, 1);
            int bits = ch->tns_res[w] + 3 - ch->tns_compress[w][f];
            /* Mostly gentle filters: a steep one of high order rings so
               hard that rounding alone would tell two decoders apart. The
               steepest values go to filters of order 1 and 2. */
            int most = (1 << (bits - 1)) - 1, reach = ch->tns_order[w][f] <= 2 ? most : rnd_in(0, 3) ? 1 : 2;
            if (reach > most) reach = most;
            for (int i = 0; i < ch->tns_order[w][f]; i++) ch->tns_coef[w][f][i] = rnd_in(-reach - (reach == most), reach);
        }
    }
}

static void put_ics_info(bitw *w, const t_ics *ics)
{
    put(w, 0, 1);
    put(w, (unsigned)ics->seq, 2);
    put(w, (unsigned)ics->shape, 1);
    if (ics->seq == AHAAC_EIGHT_SHORT) {
        put(w, (unsigned)ics->max_sfb, 4);
        put(w, ics->grouping, 7);
    } else {
        put(w, (unsigned)ics->max_sfb, 6);
        put(w, 0, 1);
    }
}

static void put_sf(bitw *w, int diff)
{
    put(w, ahaac_sf_codes[diff + 60], ahaac_sf_bits[diff + 60]);
}

static void put_channel(bitw *w, const t_chan *ch, bool common)
{
    const t_ics *ics = &ch->ics;
    const unsigned short *swb = ics->bands->offset;
    bool is_short = ics->seq == AHAAC_EIGHT_SHORT;
    put(w, (unsigned)ch->global_gain, 8);
    if (!common) put_ics_info(w, ics);

    int run_bits = is_short ? 3 : 5, run_escape = (1 << run_bits) - 1;
    for (int g = 0; g < ics->groups; g++) {
        for (int b = 0; b < ics->max_sfb;) {
            int len = 1;
            while (b + len < ics->max_sfb && ch->cb[g][b + len] == ch->cb[g][b]) len++;
            /* Now and then a run is split in two, which says the same. */
            if (len > 1 && rnd_in(0, 5) == 0) len = rnd_in(1, len - 1);
            put(w, (unsigned)ch->cb[g][b], 4);
            b += len;
            for (; len >= run_escape; len -= run_escape) put(w, (unsigned)run_escape, run_bits);
            put(w, (unsigned)len, run_bits);
        }
    }

    int gain = ch->global_gain, position = 0, noise = ch->global_gain - 90;
    bool first_noise = true;
    for (int g = 0; g < ics->groups; g++) {
        for (int b = 0; b < ics->max_sfb; b++) {
            int cb = ch->cb[g][b], v = ch->sf[g][b];
            if (cb == AHAAC_ZERO) continue;
            if (cb == AHAAC_INTENSITY || cb == AHAAC_INTENSITY2) {
                put_sf(w, v - position);
                position = v;
            } else if (cb == AHAAC_NOISE) {
                if (first_noise) put(w, (unsigned)(v - noise + 256), 9);
                else put_sf(w, v - noise);
                first_noise = false;
                noise = v;
            } else {
                put_sf(w, v - gain);
                gain = v;
            }
        }
    }

    put(w, ch->pulses != 0, 1);
    if (ch->pulses) {
        put(w, (unsigned)ch->pulses - 1, 2);
        put(w, (unsigned)ch->pulse_start, 6);
        for (int i = 0; i < ch->pulses; i++) {
            put(w, (unsigned)ch->pulse_offset[i], 5);
            put(w, (unsigned)ch->pulse_amp[i], 4);
        }
    }
    put(w, (unsigned)ch->tns, 1);
    for (int win = 0; ch->tns && win < (is_short ? 8 : 1); win++) {
        put(w, (unsigned)ch->tns_filters[win], is_short ? 1 : 2);
        if (!ch->tns_filters[win]) continue;
        put(w, (unsigned)ch->tns_res[win], 1);
        for (int f = 0; f < ch->tns_filters[win]; f++) {
            put(w, (unsigned)ch->tns_length[win][f], is_short ? 4 : 6);
            put(w, (unsigned)ch->tns_order[win][f], is_short ? 3 : 5);
            if (!ch->tns_order[win][f]) continue;
            put(w, (unsigned)ch->tns_backward[win][f], 1);
            put(w, (unsigned)ch->tns_compress[win][f], 1);
            int bits = ch->tns_res[win] + 3 - ch->tns_compress[win][f];
            for (int i = 0; i < ch->tns_order[win][f]; i++) put(w, (unsigned)ch->tns_coef[win][f][i] & ((1u << bits) - 1), bits);
        }
    }
    put(w, 0, 1); /* no gain control */

    for (int g = 0, win = 0; g < ics->groups; win += ics->group_len[g], g++) {
        for (int b = 0; b < ics->max_sfb; b++) {
            int cb = ch->cb[g][b];
            if (cb == AHAAC_ZERO || cb > AHAAC_ESCAPE) continue;
            const ahaac_book *book = &ahaac_books[cb];
            int width = swb[b + 1] - swb[b];
            for (int win2 = 0; win2 < ics->group_len[g]; win2++) {
                const int *line = ch->q + (win + win2) * 128 + swb[b];
                for (int j = 0; j < width; j += book->dim) {
                    int index = 0;
                    for (int d = 0; d < book->dim; d++) {
                        int v = line[j + d];
                        if (book->sign_bits && v < 0) v = -v;
                        if (cb == AHAAC_ESCAPE && v > 16) v = 16;
                        index = index * book->mod + v - book->low;
                    }
                    put(w, book->codes[index], book->bits[index]);
                    if (book->sign_bits)
                        for (int d = 0; d < book->dim; d++)
                            if (line[j + d]) put(w, line[j + d] < 0, 1);
                    for (int d = 0; cb == AHAAC_ESCAPE && d < 2; d++) {
                        int v = abs(line[j + d]);
                        if (v < 16) continue;
                        int n = 0;
                        while (v >> (n + 5)) n++;
                        put(w, (1u << (n + 1)) - 2, n + 1);
                        put(w, (unsigned)v - (1u << (n + 4)), n + 4);
                    }
                }
            }
        }
    }
}

/* Writes one ADTS frame from the channels given; false when it came out too
   big to be a legal frame. `fill` puts fill and data elements in front:
   1 = plain ones, 2 = one marked as HE-AAC data. */
static bool aac_frame_write(const aac_kind *k, const t_chan *ch, bool common, int ms_present, const unsigned char ms_used[8][51], int fill,
                            bytes *out)
{
    bitw w;
    memset(&w, 0, sizeof(w));
    put(&w, 0, 56); /* the header, filled in below */
    if (fill) {
        /* Fill and data elements, which a decoder steps over. */
        int count = rnd_in(0, 1) ? rnd_in(1, 14) : rnd_in(15, 60);
        put(&w, 6, 3);
        put(&w, count < 15 ? (unsigned)count : 15, 4);
        if (count >= 15) put(&w, (unsigned)count - 14, 8);
        put(&w, fill == 2 ? 13 : 1, 4); /* plain fill, or the kind that holds HE-AAC data */
        for (int i = 0; i < count * 8 - 4; i++) put(&w, rnd() & 1, 1);
        put(&w, 4, 3);
        put(&w, 3, 4);
        put(&w, 1, 1);
        put(&w, 5, 8);
        if (w.bits & 7) put(&w, 0, 8 - (w.bits & 7));
        for (int i = 0; i < 5; i++) put(&w, rnd() & 255, 8);
    }
    if (k->channels == 1) {
        put(&w, 0, 3);
        put(&w, 0, 4);
        put_channel(&w, &ch[0], false);
    } else {
        put(&w, 1, 3);
        put(&w, 0, 4);
        put(&w, common, 1);
        if (common) {
            put_ics_info(&w, &ch[0].ics);
            put(&w, (unsigned)ms_present, 2);
            if (ms_present == 1)
                for (int g = 0; g < ch[0].ics.groups; g++)
                    for (int b = 0; b < ch[0].ics.max_sfb; b++) put(&w, ms_used[g][b], 1);
        }
        put_channel(&w, &ch[0], common);
        put_channel(&w, &ch[1], common);
    }
    put(&w, 7, 3);
    if (w.bits & 7) put(&w, 0, 8 - (w.bits & 7));
    /* The format allows 6144 bits a channel. */
    if (w.bits - 56 > 6144 * k->channels) return false;

    int bytes_total = w.bits / 8;
    bitw h;
    memset(&h, 0, sizeof(h));
    put(&h, 0xFFF, 12);
    put(&h, 0, 1); /* MPEG-4 */
    put(&h, 0, 2);
    put(&h, 1, 1); /* no CRC */
    put(&h, 1, 2); /* low complexity */
    put(&h, (unsigned)k->sr_index, 4);
    put(&h, 0, 1);
    put(&h, (unsigned)k->channels, 3);
    put(&h, 0, 4);
    put(&h, (unsigned)bytes_total, 13);
    put(&h, 0x7FF, 11);
    put(&h, 0, 2);
    memcpy(w.p, h.p, 7);
    bytes_put(out, w.p, (size_t)bytes_total);
    return true;
}

/* One ADTS frame of random content. */
static bool aac_frame_build(const aac_kind *k, bytes *out)
{
    static t_chan ch[2];
    unsigned char ms_used[8][51];
    int ms_present = 0, prev[2] = {g_prev_seq[0], g_prev_seq[1]};
    bool common = k->channels == 2 && rnd_in(0, 4) != 0;
    memset(ms_used, 0, sizeof(ms_used));
    pick_ics(k, &ch[0].ics, prev[0]);
    if (common) {
        /* Both channels must be able to take this window sequence. */
        bool a = prev[0] == AHAAC_LONG_START || prev[0] == AHAAC_EIGHT_SHORT, b = prev[1] == AHAAC_LONG_START || prev[1] == AHAAC_EIGHT_SHORT;
        if (a != b) common = false;
    }
    if (common) {
        ch[1].ics = ch[0].ics;
        if (k->features & T_MS) ms_present = rnd_in(0, 2);
        for (int g = 0; g < ch[0].ics.groups; g++)
            for (int b = 0; b < ch[0].ics.max_sfb; b++) ms_used[g][b] = (unsigned char)(ms_present == 2 || (ms_present == 1 && (rnd() & 1)));
    } else if (k->channels == 2) {
        pick_ics(k, &ch[1].ics, prev[1]);
    }
    pick_channel(k, &ch[0], false, NULL, ms_used, ms_present);
    if (k->channels == 2) pick_channel(k, &ch[1], common, &ch[0], ms_used, ms_present);
    if (!aac_frame_write(k, ch, common, ms_present, ms_used, (k->features & T_FILL) && rnd_in(0, 3) == 0, out)) return false;
    g_prev_seq[0] = ch[0].ics.seq;
    g_prev_seq[1] = k->channels == 2 ? ch[1].ics.seq : ch[0].ics.seq;
    return true;
}

static void aac_stream_build(const aac_kind *k, int frames, bytes *out)
{
    out->len = 0;
    g_prev_seq[0] = g_prev_seq[1] = AHAAC_ONLY_LONG;
    for (int made = 0; made < frames;)
        if (aac_frame_build(k, out)) made++;
}

/* Builds a stream of random frames and compares this decoder with ffmpeg,
   frame by frame. */
static void test_random(const char *name, aac_kind k, int frames)
{
    bytes aac = {0};
    aac_stream_build(&k, frames, &aac);
    pcm mine = {0}, ref = {0};
    aac_run run;
    adts_decode(aac.p, aac.len, 20000, &mine, &run);
    CHECK(run.frames == frames && !run.failed && run.channels == k.channels && run.rate == ahaac_sample_rates[k.sr_index],
          "%s: %d of %d frames decoded, %d failed", name, run.frames, frames, run.failed);
    if (g_ffmpeg) {
        char path[600];
        path_of(path, name);
        write_file(path, aac.p, aac.len);
        CHECK(ffmpeg_decode(path, "-c:a aac", &ref), "%s: ffmpeg could not decode", name);
        CHECK(mine.len == ref.len, "%s: lengths differ: %zu vs %zu", name, mine.len, ref.len);
        size_t frame_samples = (size_t)1024 * (size_t)k.channels;
        int bad = 0;
        for (size_t at = 0; at < ref.len && bad < 5; at += frame_samples) {
            double d = max_diff(&ref, &mine, at, at + frame_samples);
            if (d > 2e-4) {
                bad++;
                CHECK(false, "%s: frame %zu differs from ffmpeg (diff %.2g)", name, at / frame_samples, d);
            }
        }
        printf("  %s: %d frames compared\n", name, frames);
    }
    free(aac.p);
    free(mine.p);
    free(ref.p);
}

/* ---- stereo rules where this decoder follows the standard, not ffmpeg ---------- */

/* A long-window channel whose bands all use one code book. */
static void craft_channel(t_chan *ch, const aac_kind *k, int cb, int value)
{
    memset(ch, 0, sizeof(*ch));
    ch->ics.seq = AHAAC_ONLY_LONG;
    ch->ics.groups = 1;
    ch->ics.group_len[0] = 1;
    ch->ics.bands = &ahaac_bands_long[k->sr_index];
    ch->ics.max_sfb = 30;
    ch->global_gain = cb == AHAAC_NOISE ? value + 90 : value;
    for (int b = 0; b < 30; b++) {
        ch->cb[0][b] = cb;
        ch->sf[0][b] = value;
        if (cb == 5)
            for (int j = ch->ics.bands->offset[b]; j < ch->ics.bands->offset[b + 1]; j++) ch->q[j] = rnd_in(-4, 4);
    }
}

/* Builds four frames of a left channel and a right channel made from it,
   and returns how the right relates to the left in the last one: 1 when it
   is the same, -1 when it is the same upside down, 0 when it is neither. */
static int stereo_relation(int left_cb, int right_cb, int ms_present)
{
    aac_kind k = {4, 2, 0};
    static t_chan ch[2];
    unsigned char ms_used[8][51];
    bytes aac = {0};
    memset(ms_used, ms_present ? 1 : 0, sizeof(ms_used));
    for (int i = 0; i < 4; i++) {
        craft_channel(&ch[0], &k, left_cb, left_cb == AHAAC_NOISE ? 40 : 120);
        craft_channel(&ch[1], &k, right_cb, right_cb == AHAAC_NOISE ? 40 : 0);
        ch[1].global_gain = ch[0].global_gain;
        aac_frame_write(&k, ch, true, ms_present, ms_used, 0, &aac);
    }
    pcm out = {0};
    aac_run run;
    adts_decode(aac.p, aac.len, aac.len, &out, &run);
    int same = 0, flipped = 0, loud = 0;
    for (size_t i = 2 * 2048; i + 1 < out.len && run.frames == 4; i += 2) {
        float l = out.p[i], r = out.p[i + 1];
        if (fabs(l) > 1e-7) loud++;
        if (l == r) same++;
        if (l == -r) flipped++;
    }
    free(aac.p);
    free(out.p);
    if (loud < 1500) return 9; /* the test made no sound */
    return same == 2048 ? 1 : flipped == 2048 ? -1 : 0;
}

static void test_stereo_rules(void)
{
    /* Intensity stereo: the right channel is the left one, upside down for
       the second intensity code book. A mid/side flag turns it over again,
       but only when the flags are sent band by band (ms_mask_present 1);
       "all bands" (2) leaves it alone, as the standard says. ffmpeg turns
       it over in both cases. */
    CHECK(stereo_relation(5, AHAAC_INTENSITY, 0) == 1, "intensity stereo: the right channel is not the left");
    CHECK(stereo_relation(5, AHAAC_INTENSITY2, 0) == -1, "intensity stereo, second book: the right channel is not the left upside down");
    CHECK(stereo_relation(5, AHAAC_INTENSITY, 1) == -1, "intensity stereo with a mid/side flag: not turned over");
    CHECK(stereo_relation(5, AHAAC_INTENSITY2, 1) == 1, "intensity stereo, second book, with a mid/side flag: not turned back");
    CHECK(stereo_relation(5, AHAAC_INTENSITY, 2) == 1, "intensity stereo with all bands mid/side: turned over");
    CHECK(stereo_relation(5, AHAAC_INTENSITY2, 2) == -1, "intensity stereo, second book, with all bands mid/side: turned over");
    /* Noise bands: each channel makes its own noise, unless the mid/side
       flag marks the band, when both get the same. ffmpeg always makes
       separate noise. */
    CHECK(stereo_relation(AHAAC_NOISE, AHAAC_NOISE, 0) == 0, "noise bands: both channels got the same noise");
    CHECK(stereo_relation(AHAAC_NOISE, AHAAC_NOISE, 1) == 1, "noise bands marked to share: the channels differ");
    CHECK(stereo_relation(AHAAC_NOISE, AHAAC_NOISE, 2) == 1, "noise bands, all marked to share: the channels differ");
}

/* ---- M4A files ---------------------------------------------------------------- */

static size_t memory_read(void *user, uint64_t offset, void *buffer, size_t n)
{
    const bytes *file = user;
    if (offset >= file->len) return 0;
    if (n > file->len - offset) n = (size_t)(file->len - offset);
    memcpy(buffer, file->p + offset, n);
    return n;
}

/* Decodes the AAC track of an M4A held in memory, trimmed as the file says.
   Returns NULL, or why it cannot be read. */
static const char *m4a_decode(const bytes *file, pcm *out, ahm4a_info *info_out)
{
    const char *error = NULL;
    out->len = 0;
    ahm4a *m = ahm4a_open(memory_read, (void *)file, file->len, &error);
    if (!m) return error ? error : "no reason given";
    const ahm4a_info *info = ahm4a_get_info(m);
    if (info_out) *info_out = *info;
    ahaac_decoder *d = ahaac_decoder_new();
    if (!ahaac_decoder_configure(d, info->config, info->config_bytes)) error = ahaac_decoder_error(d);
    unsigned char *packet = malloc((size_t)info->max_packet_bytes + 1);
    int channels = 0;
    for (uint64_t i = 0; !error && i < info->packets; i++) {
        float buf[AHAAC_MAX_SAMPLES * AHAAC_MAX_CHANNELS];
        int n;
        ahaac_frame fr;
        size_t bytes_read = ahm4a_read_packet(m, i, packet, (size_t)info->max_packet_bytes);
        if (!bytes_read) continue;
        if (ahaac_decode_packet(d, packet, bytes_read, buf, &n, &fr) && n) {
            channels = fr.channels;
            pcm_put(out, buf, (size_t)n * (size_t)fr.channels);
        }
    }
    /* Drop the run-in and what lies past the end. */
    if (channels) {
        size_t skip = (size_t)info->delay * (size_t)channels, keep = (size_t)info->samples * (size_t)channels;
        if (skip > out->len) skip = out->len;
        if (keep > out->len - skip) keep = out->len - skip;
        memmove(out->p, out->p + skip, keep * sizeof(float));
        out->len = keep;
    }
    out->channels = channels;
    free(packet);
    ahaac_decoder_free(d);
    ahm4a_close(m);
    return error;
}

static void test_m4a_file(const char *name, const char *args, size_t want_frames)
{
    char src[600], path[600], cmd[2000];
    path_of(src, "source.wav");
    path_of(path, name);
    snprintf(cmd, sizeof(cmd), "-i \"%s\" %s \"%s\"", src, args, path);
    if (!ffmpeg_run(cmd)) {
        printf("  %s: this ffmpeg cannot write it, skipped\n", name);
        return;
    }
    bytes file = {0};
    read_file(path, &file);
    pcm mine = {0}, ref = {0};
    ahm4a_info info;
    const char *error = m4a_decode(&file, &mine, &info);
    CHECK(!error, "%s: %s", name, error ? error : "");
    CHECK(ffmpeg_decode(path, NULL, &ref), "%s: ffmpeg could not decode", name);
    CHECK(mine.len == ref.len && ref.len > 0, "%s: %zu samples, ffmpeg plays %zu (delay %u, %llu to play, %llu packets)", name,
          mine.len / (size_t)(mine.channels ? mine.channels : 1), ref.len / (size_t)(mine.channels ? mine.channels : 1), info.delay,
          (unsigned long long)info.samples, (unsigned long long)info.packets);
    if (want_frames) CHECK(mine.len == want_frames * (size_t)mine.channels, "%s: %zu samples, the source had %zu", name, mine.len / (size_t)(mine.channels ? mine.channels : 1), want_frames);
    double diff = max_diff(&ref, &mine, 0, ref.len);
    CHECK(diff < 1e-4, "%s: decodes differently from ffmpeg (%.2g)", name, diff);
    free(file.p);
    free(mine.p);
    free(ref.p);
}

static void put_be(bytes *b, uint64_t v, int n)
{
    for (int i = n - 1; i >= 0; i--) {
        unsigned char c = (unsigned char)(v >> (8 * i));
        bytes_put(b, &c, 1);
    }
}

/* Finds a box by its four-letter name anywhere from `from` on. */
static size_t find_name(const bytes *file, const char *name, size_t from)
{
    for (size_t i = from; i + 4 <= file->len; i++)
        if (!memcmp(file->p + i, name, 4)) return i;
    return 0;
}

static void test_m4a(void)
{
    char src[600], path[600], cmd[2000];
    path_of(src, "source.wav");
    test_m4a_file("m4a_44k.m4a", "-ar 44100 -c:a aac -b:a 128k", (size_t)6 * 44100);
    test_m4a_file("m4a_48k_fast.m4a", "-c:a aac -b:a 192k -movflags +faststart", (size_t)6 * 48000);
    test_m4a_file("m4a_mono.m4a", "-ar 22050 -ac 1 -c:a aac -b:a 48k", (size_t)6 * 22050);
    test_m4a_file("m4a_short.m4a", "-t 0.37 -ar 44100 -c:a aac -b:a 96k", 0);
    test_m4a_file("m4a_mf.m4a", "-ar 44100 -c:a aac_mf -b:a 128k", 0);
    test_m4a_file("m4a_mp4.mp4", "-ar 32000 -c:a aac -b:a 96k -f mp4", (size_t)6 * 32000);

    /* Files of kinds that are not handled say so. */
    static const struct {
        const char *name, *args, *why;
    } refused[] = {
        {"refuse_mp3.mp4", "-c:a libmp3lame -f mp4", "MP3"},
        {"refuse_fragmented.mp4", "-c:a aac -movflags frag_keyframe+empty_moov -f mp4", "fragmented"},
    };
    for (size_t i = 0; i < sizeof(refused) / sizeof(refused[0]); i++) {
        path_of(path, refused[i].name);
        snprintf(cmd, sizeof(cmd), "-i \"%s\" -t 1 %s \"%s\"", src, refused[i].args, path);
        if (!ffmpeg_run(cmd)) continue;
        bytes file = {0};
        pcm out = {0};
        read_file(path, &file);
        const char *error = m4a_decode(&file, &out, NULL);
        CHECK(error && strstr(error, refused[i].why) && out.len == 0, "%s: not refused as %s (%s)", refused[i].name, refused[i].why,
              error ? error : "no error");
        free(file.p);
        free(out.p);
    }

    /* An iTunes gapless tag is used when there is no edit list: the edit
       list of an ffmpeg file is blanked, and a tag saying something else is
       added at the end of the index (the file's last box). */
    bytes file = {0}, tag = {0};
    pcm out = {0};
    ahm4a_info info;
    path_of(path, "m4a_44k.m4a");
    read_file(path, &file);
    size_t moov = find_name(&file, "moov", 0), edts = find_name(&file, "edts", moov), udta = find_name(&file, "udta", moov);
    CHECK(moov && edts && moov + (size_t)(file.p[moov - 4] << 24 | file.p[moov - 3] << 16 | file.p[moov - 2] << 8 | file.p[moov - 1]) - 4 == file.len,
          "the ffmpeg file is not laid out as the tag test expects");
    if (moov && edts) {
        static const char text[] = " 00000000 00000840 000001C0 0000000000012345 00000000 00000000 00000000 00000000 00000000 00000000 00000000 00000000";
        memcpy(file.p + edts, "free", 4);
        if (udta) memcpy(file.p + udta, "free", 4);
        size_t text_len = sizeof(text) - 1, data = 16 + text_len, item = 8 + 28 + 20 + data, ilst = 8 + item, meta = 12 + ilst;
        put_be(&tag, 8 + meta, 4);
        bytes_put(&tag, "udta", 4);
        put_be(&tag, meta, 4);
        bytes_put(&tag, "meta\0\0\0\0", 8);
        put_be(&tag, ilst, 4);
        bytes_put(&tag, "ilst", 4);
        put_be(&tag, item, 4);
        bytes_put(&tag, "----", 4);
        put_be(&tag, 28, 4);
        bytes_put(&tag, "mean\0\0\0\0com.apple.iTunes", 24);
        put_be(&tag, 20, 4);
        bytes_put(&tag, "name\0\0\0\0iTunSMPB", 16);
        put_be(&tag, data, 4);
        bytes_put(&tag, "data\0\0\0\1\0\0\0\0", 12);
        bytes_put(&tag, text, text_len);
        uint32_t moov_size = (uint32_t)(file.len - (moov - 4) + tag.len);
        for (int i = 0; i < 4; i++) file.p[moov - 4 + i] = (unsigned char)(moov_size >> (24 - 8 * i));
        bytes_put(&file, tag.p, tag.len);
        const char *error = m4a_decode(&file, &out, &info);
        CHECK(!error && info.delay == 0x840 && info.samples == 0x12345 && out.len == (size_t)0x12345 * 2, "iTunes gapless tag: %s, delay %u, %llu samples",
              error ? error : "read", info.delay, (unsigned long long)info.samples);
    }

    /* With no edit list, a track whose header says it ends inside the last
       packet stops there, as ffmpeg plays it. */
    read_file(path, &file);
    size_t mdhd = find_name(&file, "mdhd", moov);
    if (moov && edts && mdhd && file.p[mdhd + 4] == 0) {
        char cut_path[600];
        size_t at = mdhd + 4 + 16;
        uint32_t length = (uint32_t)file.p[at] << 24 | (uint32_t)file.p[at + 1] << 16 | (uint32_t)file.p[at + 2] << 8 | file.p[at + 3];
        length -= 50;
        for (int i = 0; i < 4; i++) file.p[at + (size_t)i] = (unsigned char)(length >> (24 - 8 * i));
        memcpy(file.p + edts, "free", 4);
        path_of(cut_path, "m4a_cut.m4a");
        write_file(cut_path, file.p, file.len);
        pcm ref = {0};
        const char *error = m4a_decode(&file, &out, &info);
        CHECK(!error && info.samples == length && out.len == (size_t)length * 2, "a track that ends inside its last packet: %s, %llu samples",
              error ? error : "read", (unsigned long long)info.samples);
        CHECK(ffmpeg_decode(cut_path, NULL, &ref) && ref.len == out.len && max_diff(&ref, &out, 0, ref.len) < 1e-4,
              "a track that ends inside its last packet plays otherwise in ffmpeg (%zu samples, ffmpeg %zu)", out.len / 2, ref.len / 2);
        free(ref.p);
    }

    /* Damage to the index: no crash, and every packet lies inside the file. */
    read_file(path, &file);
    bytes bad = {0};
    for (int round = 0; round < 3000 && moov; round++) {
        bad.len = 0;
        bytes_put(&bad, file.p, file.len);
        int hits = rnd_in(1, 6);
        for (int i = 0; i < hits; i++) bad.p[moov + rnd() % (bad.len - moov)] = (unsigned char)(rnd_in(0, 3) ? rnd() : rnd_in(0, 1) ? 0xFF : 0);
        if (round % 6 == 0) bad.len = moov + rnd() % (bad.len - moov);
        const char *error = NULL;
        ahm4a *m = ahm4a_open(memory_read, &bad, bad.len, &error);
        if (!m) {
            if (!error) CHECK(false, "a damaged M4A was refused without a reason");
            continue;
        }
        const ahm4a_info *in = ahm4a_get_info(m);
        bool inside = true;
        for (uint64_t i = 0; i < in->packets; i += 1 + in->packets / 50) {
            uint64_t offset;
            uint32_t n;
            if (!ahm4a_packet(m, i, &offset, &n) || offset > bad.len || n > bad.len - offset) inside = false;
        }
        if (!inside) CHECK(false, "a damaged M4A gave a packet outside the file");
        if (round % 40 == 0) {
            ahm4a_close(m);
            m4a_decode(&bad, &out, NULL);
            continue;
        }
        ahm4a_close(m);
    }
    CHECK(true, "damaged M4A files read without a crash");
    const char *error = NULL;
    CHECK(!ahm4a_open(memory_read, &tag, tag.len, &error) && error, "junk opened as an M4A");
    bad.len = 0;
    CHECK(!ahm4a_open(memory_read, &bad, 0, &error) && error, "an empty file opened as an M4A");
    free(bad.p);
    free(file.p);
    free(tag.p);
    free(out.p);
}

/* ---- HE-AAC: detected and reported, not decoded ------------------------------- */

/* AAC settings as an MP4 holds them, rate number `sr` inside: `object` 2, 5 or 29;
   `sbr` -1 for no sync word 0x2B7 after the plain settings, else its SBR
   flag; `ps` -1 for no sync word 0x548, else its PS flag; `extension` sets
   the extension flag (and a flag 3 after it). */
static void he_settings(bitw *w, int object, int sr, int channels, bool extension, int sbr, int ps)
{
    memset(w, 0, sizeof(*w));
    put(w, (unsigned)object, 5);
    put(w, (unsigned)sr, 4);
    put(w, (unsigned)channels, 4);
    if (object != 2) {
        put(w, (unsigned)sr - 3, 4); /* twice the rate with the extension */
        put(w, 2, 5);
    }
    put(w, 0, 2); /* 1024-sample frames, no core coder */
    put(w, extension, 1);
    if (extension) put(w, 0, 1);
    if (sbr >= 0) {
        put(w, 0x2B7, 11);
        put(w, 5, 5);
        put(w, (unsigned)sbr, 1);
        if (sbr) {
            put(w, (unsigned)sr - 3, 4);
            if (ps >= 0) {
                put(w, 0x548, 11);
                put(w, (unsigned)ps, 1);
            }
        }
    }
}

/* Sets a decoder up with the settings and reports what a frame says. */
static bool he_report(ahaac_decoder *d, const bitw *w, ahaac_frame *fr)
{
    float buf[AHAAC_MAX_SAMPLES * AHAAC_MAX_CHANNELS];
    int n;
    memset(fr, 0, sizeof(*fr));
    bool ok = ahaac_decoder_configure(d, w->p, (size_t)(w->bits + 7) / 8);
    ahaac_decode_packet(d, w->p, 0, buf, &n, fr);
    return ok;
}

/* The power of one tone in one channel (Goertzel). */
static double tone_power(const pcm *p, int channel, size_t from, size_t to, double freq)
{
    double c = 2 * cos(2 * 3.14159265358979323846 * freq / p->rate), s1 = 0, s2 = 0;
    for (size_t i = from; i < to; i++) {
        double s0 = p->p[i * (size_t)p->channels + (size_t)channel] + c * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return s1 * s1 + s2 * s2 - c * s1 * s2;
}

static void test_he_aac(void)
{
    /* Every way the settings can state HE-AAC, or leave it out. */
    static const struct {
        const char *what;
        int object, channels;
        bool extension;
        int sbr, ps;
        bool want_sbr, want_ps;
    } cases[] = {
        {"plain AAC-LC", 2, 2, false, -1, -1, false, false},
        {"plain AAC-LC with the extension flag", 2, 2, true, -1, -1, false, false},
        {"type 5", 5, 2, false, -1, -1, true, false},
        {"type 29", 29, 1, false, -1, -1, true, true},
        {"sync word 0x2B7, SBR", 2, 2, false, 1, -1, true, false},
        {"sync word 0x2B7 after the extension flag", 2, 2, true, 1, -1, true, false},
        {"sync word 0x2B7, no SBR", 2, 2, false, 0, -1, false, false},
        {"sync words 0x2B7 and 0x548, PS", 2, 1, false, 1, 1, true, true},
        {"sync words 0x2B7 and 0x548, no PS", 2, 1, false, 1, 0, true, false},
        {"type 5, then sync words that are not read", 5, 1, false, 1, 1, true, false},
    };
    static bitw w;
    ahaac_frame fr;
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        he_settings(&w, cases[i].object, 7, cases[i].channels, cases[i].extension, cases[i].sbr, cases[i].ps);
        ahaac_decoder *d = ahaac_decoder_new();
        bool ok = he_report(d, &w, &fr);
        CHECK(ok && fr.sbr == cases[i].want_sbr && fr.ps == cases[i].want_ps, "HE-AAC settings, %s: %s, SBR %d, PS %d", cases[i].what,
              ok ? "taken" : "refused", fr.sbr, fr.ps);
        ahaac_decoder_free(d);
    }
    /* A sync word with a different number after it is not HE-AAC; and new
       settings forget what the old ones said. */
    he_settings(&w, 2, 7, 2, false, -1, -1);
    put(&w, 0x2B6, 11);
    put(&w, 5, 5);
    put(&w, 1, 1);
    ahaac_decoder *d = ahaac_decoder_new();
    CHECK(he_report(d, &w, &fr) && !fr.sbr, "a wrong sync word read as HE-AAC");
    he_settings(&w, 29, 7, 1, false, -1, -1);
    he_report(d, &w, &fr);
    he_settings(&w, 2, 7, 2, false, -1, -1);
    CHECK(he_report(d, &w, &fr) && !fr.sbr && !fr.ps, "plain settings after HE-AAC ones still say HE-AAC");
    ahaac_decoder_free(d);

    /* A packet with SBR data in a fill element: settings that say nothing
       leave it to the packet; settings that say "no SBR" outright win (as in
       ffmpeg); settings cut off before the SBR flag say nothing. */
    aac_kind k = {4, 2, T_FILL};
    static t_chan ch[2];
    unsigned char ms_used[8][51] = {{0}};
    bytes frame = {0};
    craft_channel(&ch[0], &k, 5, 120);
    craft_channel(&ch[1], &k, 5, 120);
    CHECK(aac_frame_write(&k, ch, true, 0, ms_used, 2, &frame), "the frame with SBR data could not be written");
    static const struct {
        const char *what;
        int sbr, cut;
        bool want;
    } stated[] = {{"no sync word", -1, 0, true}, {"SBR", 1, 0, true}, {"no SBR", 0, 0, false}, {"cut off before the SBR flag", 1, 32, true}};
    for (size_t i = 0; i < sizeof(stated) / sizeof(stated[0]) && frame.len > 7; i++) {
        he_settings(&w, 2, 4, 2, false, stated[i].sbr, -1);
        if (stated[i].cut) {
            w.bits = stated[i].cut;
            memset(w.p + stated[i].cut / 8, 0, sizeof(w.p) - (size_t)stated[i].cut / 8);
        }
        ahaac_decoder *pd = ahaac_decoder_new();
        bool ok = ahaac_decoder_configure(pd, w.p, (size_t)(w.bits + 7) / 8);
        float buf[AHAAC_MAX_SAMPLES * AHAAC_MAX_CHANNELS];
        int n = 0;
        ahaac_frame got = {0};
        bool decoded = ahaac_decode_packet(pd, frame.p + 7, frame.len - 7, buf, &n, &got);
        CHECK(ok && decoded && n == AHAAC_MAX_SAMPLES && got.sbr == stated[i].want, "SBR data in a packet, settings %s: %s, %d samples, SBR %d",
              stated[i].what, ok ? "taken" : "refused", n, got.sbr);
        ahaac_decoder_free(pd);
    }
    free(frame.p);

    /* Files made by Windows' AAC encoder (ffmpeg's aac_mf, stereo at 48
       kb/s): one second, 440 Hz on the left and 660 Hz on the right. The
       ADTS file states 22.05 kHz AAC-LC and carries SBR in fill elements;
       the M4A file states type 5. Both play here at 22.05 kHz. */
    static const char *const names[] = {"he_aac.aac", "he_aac.m4a"};
    for (int f = 0; f < 2; f++) {
        char path[1000];
        snprintf(path, sizeof(path), "%s/%s", AHMP3_FIXTURES, names[f]);
        bytes file = {0};
        if (!read_file(path, &file)) {
            CHECK(false, "%s: the file is missing", path);
            continue;
        }
        pcm out = {0};
        ahreader_info info = {0};
        const char *error = NULL;
        size_t n = read_through(&file, &out, &info, &error);
        CHECK(n && info.half_rate && !info.parametric_stereo && info.sample_rate == 22050 && info.channels == 2,
              "%s: %s, %d Hz, %d channels, half rate %d, PS %d", names[f], error ? error : "opened", info.sample_rate, info.channels,
              info.half_rate, info.parametric_stereo);
        CHECK(n >= 22050 - 1024 && n <= 22050 + 3 * 1024, "%s: %zu samples for a second at 22.05 kHz", names[f], n);
        if (n > 20000) {
            out.rate = 22050;
            double l440 = tone_power(&out, 0, 4000, 18000, 440), l660 = tone_power(&out, 0, 4000, 18000, 660);
            double r440 = tone_power(&out, 1, 4000, 18000, 440), r660 = tone_power(&out, 1, 4000, 18000, 660);
            CHECK(l440 > 1000 * l660 && r660 > 1000 * r440, "%s: the tones are not where they belong (%.3g %.3g, %.3g %.3g)", names[f], l440,
                  l660, r440, r660);
        }
        /* ffmpeg decodes the SBR too: twice as many samples, at 44.1 kHz.
           It works on a copy in the scratch folder, since it leaves its
           output beside its input. */
        bytes full = {0};
        size_t ff = 0;
        if (g_ffmpeg) {
            char copy[600];
            path_of(copy, names[f]);
            bool ok = write_file(copy, file.p, file.len) && ffmpeg_raw(copy, NULL, "f32le", &full);
            ff = full.len / 8;
            CHECK(ok && ff + 4096 >= 2 * n && ff <= 2 * n + 4096, "%s: ffmpeg gives %zu samples with SBR, against %zu here", names[f], ff, n);
        }
        printf("  %s: HE-AAC reported, %zu samples at %d Hz (ffmpeg with SBR: %zu)\n", names[f], n, info.sample_rate, ff);
        free(full.p);
        free(out.p);
        free(file.p);
    }
}

/* ---- streams that are odd, damaged or not handled ------------------------------ */

static void test_odd_streams(void)
{
    aac_kind k = {4, 2, T_PNS | T_INTENSITY | T_MS | T_TNS | T_PULSE | T_SHORT | T_FILL};
    bytes aac = {0}, other = {0};
    pcm good = {0}, got = {0};
    aac_run run;
    aac_stream_build(&k, 60, &aac);
    adts_decode(aac.p, aac.len, aac.len, &good, &run);
    CHECK(run.frames == 60 && !run.sbr, "the stream for the odd-stream tests did not decode");

    /* A few bytes at a time; a tag and junk in front. */
    adts_decode(aac.p, aac.len, 61, &got, &run);
    CHECK(got.len == good.len && !memcmp(got.p, good.p, good.len * sizeof(float)), "decoding in small pieces differs");
    unsigned char tag[10 + 200] = {'I', 'D', '3', 4, 0, 0, 0, 0, 1, 72};
    for (int i = 10; i < 210; i++) tag[i] = (unsigned char)(i % 3 ? 0xFF : 0xF1);
    bytes_put(&other, tag, sizeof(tag));
    bytes_put(&other, tag + 10, 99);
    bytes_put(&other, aac.p, aac.len);
    adts_decode(other.p, other.len, 4096, &got, &run);
    CHECK(got.len == good.len && !memcmp(got.p, good.p, good.len * sizeof(float)), "a tag and junk in front change the sound");

    /* HE-AAC data in a fill element is stepped over and reported. */
    static t_chan ch[2];
    unsigned char ms_used[8][51] = {{0}};
    other.len = 0;
    for (int i = 0; i < 6; i++) {
        craft_channel(&ch[0], &k, 5, 120);
        craft_channel(&ch[1], &k, 5, 120);
        aac_frame_write(&k, ch, true, 0, ms_used, 2, &other);
    }
    adts_decode(other.p, other.len, other.len, &got, &run);
    CHECK(run.frames == 6 && run.sbr, "HE-AAC data: %d frames decoded, reported %d", run.frames, run.sbr);

    /* Kinds that are refused: other profiles, more than two channels. */
    static const struct {
        int byte, mask, value;
        const char *why;
    } refused[] = {{2, 0xC0, 0x00, "Main"}, {2, 0xC0, 0x80, "AAC-LC"}, {3, 0xC0, 0xC0, "two channels"}};
    for (size_t r = 0; r < sizeof(refused) / sizeof(refused[0]); r++) {
        other.len = 0;
        bytes_put(&other, aac.p, aac.len);
        for (size_t pos = 0; pos + 7 <= other.len;) {
            size_t n = ((size_t)(other.p[pos + 3] & 3) << 11) | ((size_t)other.p[pos + 4] << 3) | (other.p[pos + 5] >> 5);
            other.p[pos + refused[r].byte] = (unsigned char)((other.p[pos + refused[r].byte] & ~refused[r].mask) | refused[r].value);
            pos += n;
        }
        ahaac_decoder *d = ahaac_decoder_new();
        float buf[AHAAC_MAX_SAMPLES * AHAAC_MAX_CHANNELS];
        int n = 0, sound = 0;
        size_t pos = 0, used;
        while ((used = ahaac_decode(d, other.p + pos, other.len - pos, true, buf, &n, NULL)) != 0) {
            pos += used;
            sound += n;
        }
        const char *error = ahaac_decoder_error(d);
        CHECK(!sound && error && strstr(error, refused[r].why), "a stream that should be refused (%s): %d samples, %s", refused[r].why, sound,
              error ? error : "no reason");
        ahaac_decoder_free(d);
    }

    /* Decoder settings as an MP4 holds them. */
    ahaac_decoder *d = ahaac_decoder_new();
    static const unsigned char lc_44k_stereo[] = {0x12, 0x10}, lc_48k_mono[] = {0x11, 0x88}, main_44k[] = {0x0A, 0x10},
                               he_aac[] = {0x2B, 0x92, 0x08, 0x00}, short_frames[] = {0x12, 0x14}, six_channels[] = {0x12, 0x30};
    CHECK(ahaac_decoder_configure(d, lc_44k_stereo, 2) && !ahaac_decoder_error(d), "AAC-LC 44.1 kHz stereo settings refused");
    CHECK(ahaac_decoder_configure(d, lc_48k_mono, 2), "AAC-LC 48 kHz mono settings refused");
    CHECK(!ahaac_decoder_configure(d, main_44k, 2) && strstr(ahaac_decoder_error(d), "Main"), "AAC Main settings accepted");
    CHECK(!ahaac_decoder_configure(d, short_frames, 2) && strstr(ahaac_decoder_error(d), "960"), "960-sample settings accepted");
    CHECK(!ahaac_decoder_configure(d, six_channels, 2) && strstr(ahaac_decoder_error(d), "two channels"), "5.1 settings accepted");
    CHECK(!ahaac_decoder_configure(d, lc_44k_stereo, 1), "cut-off settings accepted");
    CHECK(!ahaac_decoder_configure(d, lc_44k_stereo, 0), "empty settings accepted");
    /* HE-AAC stated outright: 22.05 kHz AAC-LC inside, 44.1 kHz with the extension. */
    CHECK(ahaac_decoder_configure(d, he_aac, 4), "HE-AAC settings refused: %s", ahaac_decoder_error(d) ? ahaac_decoder_error(d) : "");
    float buf[AHAAC_MAX_SAMPLES * AHAAC_MAX_CHANNELS];
    int n;
    ahaac_frame fr;
    CHECK(!ahaac_decode_packet(d, aac.p, 0, buf, &n, &fr) && n == 0, "an empty packet decoded");
    ahaac_decoder_free(d);

    /* Random damage and noise: no crash, no runaway output. Raw packets too. */
    for (int round = 0; round < 400; round++) {
        other.len = 0;
        bytes_put(&other, aac.p, aac.len);
        int hits = rnd_in(1, 60);
        for (int i = 0; i < hits; i++) other.p[rnd() % other.len] = (unsigned char)rnd();
        if (round % 4 == 0) other.len = rnd() % other.len;
        adts_decode(other.p, other.len, (size_t)rnd_in(1, 9000), &got, &run);
        if (got.len > good.len) {
            CHECK(false, "a damaged AAC stream gave %zu samples from a stream of %zu", got.len, good.len);
            break;
        }
    }
    d = ahaac_decoder_new();
    ahaac_decoder_configure(d, lc_44k_stereo, 2);
    for (int round = 0; round < 3000; round++) {
        unsigned char packet[400];
        size_t len = (size_t)rnd_in(0, 400);
        for (size_t i = 0; i < len; i++) packet[i] = (unsigned char)(rnd_in(0, 4) ? rnd() : 0);
        if (round % 3 == 0 && aac.len > 500) {
            memcpy(packet, aac.p + 7, 400); /* a real frame's start, then damaged */
            len = 400;
            for (int i = 0; i < 4; i++) packet[rnd() % 400] ^= (unsigned char)(1 << rnd_in(0, 7));
        }
        ahaac_decode_packet(d, packet, len, buf, &n, &fr);
        for (int i = 0; i < n * fr.channels; i++) {
            uint32_t bits;
            memcpy(&bits, &buf[i], 4);
            if ((bits & 0x7F800000u) == 0x7F800000u) {
                CHECK(false, "a damaged packet decoded to something that is not a number");
                round = 3000;
                break;
            }
        }
    }
    ahaac_decoder_free(d);
    for (int round = 0; round < 40; round++) {
        other.len = 0;
        for (int i = 0; i < 20000; i++) {
            unsigned char c = (unsigned char)(rnd_in(0, 7) == 0 ? 0xFF : rnd_in(0, 7) == 0 ? 0xF1 : rnd());
            bytes_put(&other, &c, 1);
        }
        adts_decode(other.p, other.len, 20000, &got, &run);
    }
    CHECK(true, "damaged AAC decoded without a crash");
    free(aac.p);
    free(other.p);
    free(good.p);
    free(got.p);
}

/* ---- files from encoders ------------------------------------------------------ */

static void test_encoded(const char *name, const char *args)
{
    char src[600], path[600], cmd[2000];
    path_of(src, "source.wav");
    path_of(path, name);
    snprintf(cmd, sizeof(cmd), "-i \"%s\" %s -f adts \"%s\"", src, args, path);
    if (!ffmpeg_run(cmd)) {
        printf("  %s: encoder not available in this ffmpeg, skipped\n", name);
        return;
    }
    bytes b = {0};
    read_file(path, &b);
    pcm mine = {0}, ref = {0};
    aac_run run;
    adts_decode(b.p, b.len, 4096, &mine, &run);
    CHECK(ffmpeg_decode(path, "-c:a aac", &ref), "%s: ffmpeg could not decode", name);
    CHECK(mine.len == ref.len && ref.len > 0 && !run.failed, "%s: lengths differ: %zu vs %zu (%d frames failed)", name, mine.len, ref.len,
          run.failed);
    double diff = max_diff(&ref, &mine, 0, ref.len);
    CHECK(diff < 1e-4, "%s: decodes differently from ffmpeg (%.2g)", name, diff);
    printf("  %s: %d frames, %d Hz, %d channels, difference %.1g\n", name, run.frames, run.rate, run.channels, diff);
    free(b.p);
    free(mine.p);
    free(ref.p);
}

void test_aac(void)
{
    test_imdct();
    int all = T_PNS | T_INTENSITY | T_MS | T_TNS | T_PULSE | T_SHORT | T_FILL;
    test_random("rnd_aac_44k_stereo.aac", (aac_kind){4, 2, all}, 400);
    test_random("rnd_aac_48k_mono.aac", (aac_kind){3, 1, all}, 200);
    test_random("rnd_aac_96k_stereo.aac", (aac_kind){0, 2, all}, 150);
    test_random("rnd_aac_88k_mono.aac", (aac_kind){1, 1, all}, 80);
    test_random("rnd_aac_64k_stereo.aac", (aac_kind){2, 2, all}, 150);
    test_random("rnd_aac_32k_stereo.aac", (aac_kind){5, 2, all}, 150);
    test_random("rnd_aac_24k_stereo.aac", (aac_kind){6, 2, all}, 150);
    test_random("rnd_aac_22k_mono.aac", (aac_kind){7, 1, all}, 80);
    test_random("rnd_aac_16k_stereo.aac", (aac_kind){8, 2, all}, 150);
    test_random("rnd_aac_12k_stereo.aac", (aac_kind){9, 2, all}, 80);
    test_random("rnd_aac_11k_mono.aac", (aac_kind){10, 1, all}, 80);
    test_random("rnd_aac_8k_stereo.aac", (aac_kind){11, 2, all}, 150);
    test_random("rnd_aac_7k_stereo.aac", (aac_kind){12, 2, all}, 80);
    /* Plain streams, one feature at a time, to tell them apart when one fails. */
    test_random("rnd_aac_plain.aac", (aac_kind){4, 2, 0}, 100);
    test_random("rnd_aac_short.aac", (aac_kind){4, 2, T_SHORT}, 150);
    test_random("rnd_aac_tns.aac", (aac_kind){4, 2, T_TNS | T_SHORT}, 150);
    test_random("rnd_aac_stereo_tools.aac", (aac_kind){4, 2, T_MS | T_INTENSITY | T_SHORT}, 150);
    test_random("rnd_aac_noise.aac", (aac_kind){4, 2, T_PNS | T_MS | T_SHORT}, 150);
    test_random("rnd_aac_pulse.aac", (aac_kind){4, 1, T_PULSE}, 150);
    /* Every code word of every book was written at least once. */
    for (int cb = 1; cb < 12; cb++) {
        int missing = 0;
        for (int i = 0; i < ahaac_books[cb].count; i++) missing += !g_book_used[cb][i];
        CHECK(!missing, "code book %d: %d code words were never used", cb, missing);
    }
    for (int n = 0; n < 9; n++) CHECK(g_escape_sizes[n] > 0, "no escape of size %d was written", n);
    test_stereo_rules();
    test_odd_streams();
    test_he_aac();
    if (g_ffmpeg) {
        make_source();
        test_encoded("aac_128.aac", "-ar 44100 -c:a aac -b:a 128k -aac_pns 0");
        test_encoded("aac_128_pns.aac", "-ar 44100 -c:a aac -b:a 128k");
        test_encoded("aac_48k_256.aac", "-c:a aac -b:a 256k");
        test_encoded("aac_mono_64.aac", "-ar 44100 -ac 1 -c:a aac -b:a 64k");
        test_encoded("aac_low.aac", "-ar 44100 -c:a aac -b:a 48k");
        test_encoded("aac_fast.aac", "-ar 32000 -c:a aac -b:a 96k -aac_coder fast");
        test_encoded("aac_22k.aac", "-ar 22050 -c:a aac -b:a 48k");
        test_encoded("aac_8k.aac", "-ar 8000 -ac 1 -c:a aac -b:a 16k");
        test_encoded("aac_96k.aac", "-ar 96000 -c:a aac -b:a 192k");
        test_encoded("aac_mf_128.aac", "-ar 44100 -c:a aac_mf -b:a 128k");
        test_encoded("aac_mf_48k_192.aac", "-c:a aac_mf -b:a 192k");
        test_encoded("aac_mf_mono.aac", "-ar 44100 -ac 1 -c:a aac_mf -b:a 96k");
        test_m4a();
    }
}
