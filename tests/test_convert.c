/* The sample rate converter and the mix to stereo: measured against the
   mathematics (a tone resampled is the same tone at the new rate) and
   compared with ffmpeg. */
#include "../include/ahmix.h"
#include "../include/ahmp3.h"
#include "../include/ahreader.h"
#include "../include/ahresample.h"
#include "common.h"

static const double PI = 3.14159265358979323846;

/* ---- helpers ------------------------------------------------------------------ */

/* Runs a whole signal through a resampler. With `pieces`, input and room
   come in pieces of random size, down to nothing. */
static void resample_all(const float *in, size_t frames, int channels, int in_rate, int out_rate, bool pieces, pcm *out)
{
    ahresample *r = ahresample_new(in_rate, out_rate, channels);
    out->len = 0;
    out->channels = channels;
    out->rate = out_rate;
    CHECK(r != NULL, "no resampler for %d -> %d", in_rate, out_rate);
    if (!r) return;
    float buf[4096];
    size_t most = sizeof(buf) / sizeof(buf[0]) / (size_t)channels, pos = 0;
    while (pos < frames) {
        size_t n = frames - pos, room = most;
        if (pieces) {
            size_t k = (size_t)rnd_in(0, 700);
            if (k < n) n = k;
            room = (size_t)rnd_in(0, (int)most);
        }
        size_t used = 0, made = ahresample_process(r, in + pos * (size_t)channels, n, &used, buf, room);
        pcm_put(out, buf, made * (size_t)channels);
        pos += used;
    }
    for (;;) {
        size_t made = ahresample_flush(r, buf, pieces ? (size_t)rnd_in(1, (int)most) : most);
        if (!made) break;
        pcm_put(out, buf, made * (size_t)channels);
    }
    ahresample_free(r);
}

static double db(double ratio)
{
    return 20 * log10(ratio > 1e-12 ? ratio : 1e-12);
}

/* A WAV file of floats or 16-bit whole numbers; with a mask, an extensible
   header that names the speakers. */
static void write_wav(const char *path, const float *p, size_t frames, int channels, int rate, unsigned mask, int bits)
{
    bytes b = {0};
    unsigned bytes_per = (unsigned)bits / 8, data = (unsigned)(frames * (size_t)channels * bytes_per);
    unsigned fmt_size = mask ? 40 : 16, tag = bits == 32 ? 3 : 1;
    unsigned char h[80];
    size_t n = 0;
#define PUT32(v) (h[n] = (unsigned char)(v), h[n + 1] = (unsigned char)((v) >> 8), h[n + 2] = (unsigned char)((v) >> 16), h[n + 3] = (unsigned char)((v) >> 24), n += 4)
#define PUT16(v) (h[n] = (unsigned char)(v), h[n + 1] = (unsigned char)((v) >> 8), n += 2)
    memcpy(h, "RIFF", 4);
    n = 4;
    PUT32(4 + 8 + fmt_size + 8 + data);
    memcpy(h + n, "WAVEfmt ", 8);
    n += 8;
    PUT32(fmt_size);
    PUT16(mask ? 0xFFFEu : tag);
    PUT16((unsigned)channels);
    PUT32((unsigned)rate);
    PUT32((unsigned)rate * (unsigned)channels * bytes_per);
    PUT16((unsigned)channels * bytes_per);
    PUT16((unsigned)bits);
    if (mask) {
        static const unsigned char guid_tail[14] = {0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71};
        PUT16(22u);
        PUT16((unsigned)bits);
        PUT32(mask);
        PUT16(tag);
        memcpy(h + n, guid_tail, 14);
        n += 14;
    }
    memcpy(h + n, "data", 4);
    n += 4;
    PUT32(data);
#undef PUT32
#undef PUT16
    bytes_put(&b, h, n);
    if (bits == 32) {
        bytes_put(&b, p, frames * (size_t)channels * sizeof(float));
    } else {
        for (size_t i = 0; i < frames * (size_t)channels; i++) {
            int16_t v;
            ahmp3_to_s16(p + i, &v, 1);
            unsigned char two[2] = {(unsigned char)(v & 255), (unsigned char)((v >> 8) & 255)};
            bytes_put(&b, two, 2);
        }
    }
    CHECK(write_file(path, b.p, b.len), "cannot write %s", path);
    free(b.p);
}

/* ---- the resampler against the mathematics ---------------------------------------- */

/* Resamples a tone and returns how far the result is from the same tone at
   the new rate, relative to the tone, away from the two ends. */
static double tone_error(int in_rate, int out_rate, double hz)
{
    size_t frames = (size_t)in_rate / 2;
    float *in = malloc(frames * sizeof(float));
    for (size_t i = 0; i < frames; i++) in[i] = (float)(0.5 * sin(2 * PI * hz * (double)i / in_rate));
    pcm out = {0};
    resample_all(in, frames, 1, in_rate, out_rate, false, &out);
    size_t edge = (size_t)out_rate / 25; /* 40 ms: the filter's reach at the slowest rate */
    double worst = 0;
    for (size_t k = edge; k + edge < out.len; k++) {
        double d = fabs((double)out.p[k] - 0.5 * sin(2 * PI * hz * (double)k / out_rate));
        if (d > worst) worst = d;
    }
    free(in);
    free(out.p);
    return worst / 0.5;
}

/* Resamples a tone the new rate cannot carry and returns the loudest thing
   that comes out, relative to the tone. */
static double leak(int in_rate, int out_rate, double hz)
{
    size_t frames = (size_t)in_rate / 2;
    float *in = malloc(frames * sizeof(float));
    for (size_t i = 0; i < frames; i++) in[i] = (float)(0.5 * sin(2 * PI * hz * (double)i / in_rate));
    pcm out = {0};
    resample_all(in, frames, 1, in_rate, out_rate, false, &out);
    size_t edge = (size_t)out_rate / 25;
    double worst = 0;
    for (size_t k = edge; k + edge < out.len; k++)
        if (fabs(out.p[k]) > worst) worst = fabs(out.p[k]);
    free(in);
    free(out.p);
    return worst / 0.5;
}

static void test_resample_tones(void)
{
    static const int pairs[][2] = {
        {48000, 44100}, {96000, 44100}, {88200, 44100}, {192000, 44100}, {176400, 44100}, {32000, 44100},
        {22050, 44100}, {11025, 44100}, {8000, 44100},  {44100, 48000},  {44100, 8000},   {44100, 22050},
        {37800, 44100}, {44100, 44099}, {47999, 44100}, {44100, 96000},  {12000, 8000},   {768000, 44100},
    };
    double worst_pass = 0, worst_stop = 0;
    for (size_t i = 0; i < sizeof(pairs) / sizeof(pairs[0]); i++) {
        int in = pairs[i][0], out = pairs[i][1];
        double low = (in < out ? in : out) / 2.0, pass = 0, stop = 0;
        /* Kept: everything up to 90.7% of the lower rate's half. */
        static const double kept[] = {0.002, 0.05, 0.31, 0.62, 0.85, 0.9};
        for (size_t k = 0; k < sizeof(kept) / sizeof(kept[0]); k++) {
            double e = tone_error(in, out, kept[k] * low);
            if (e > pass) pass = e;
        }
        /* Removed: everything from that half up, when the rate goes down. */
        if (out < in) {
            static const double gone[] = {1.0, 1.02, 1.3, 1.9};
            for (size_t k = 0; k < sizeof(gone) / sizeof(gone[0]); k++) {
                double hz = gone[k] * low;
                if (hz >= in / 2.0 * 0.999) continue; /* the input cannot carry it either */
                double e = leak(in, out, hz);
                if (e > stop) stop = e;
            }
        }
        CHECK(pass < 2e-5, "%d -> %d: kept tones come out %.1f dB wrong", in, out, db(pass));
        CHECK(stop < 2e-5, "%d -> %d: removed tones leak at %.1f dB", in, out, db(stop));
        if (pass > worst_pass) worst_pass = pass;
        if (stop > worst_stop) worst_stop = stop;
    }
    printf("  kept tones match the ideal within %.1f dB; removed tones are down %.1f dB\n", db(worst_pass), -db(worst_stop));
}

static void test_resample_shape(void)
{
    /* Same rate: the samples pass through untouched. */
    float in[3000], noise[3000];
    for (int i = 0; i < 3000; i++) noise[i] = in[i] = (float)rnd_in(-30000, 30000) / 32768.0f;
    pcm out = {0}, again = {0};
    resample_all(in, 1500, 2, 44100, 44100, true, &out);
    CHECK(out.len == 3000 && !memcmp(out.p, in, sizeof(in)), "same-rate conversion changed the samples");

    /* n samples in give ceil(n * out / in) out, however they are fed, and
       the same samples however they are fed. */
    static const int pairs[][2] = {{48000, 44100}, {44100, 48000}, {8000, 44100}, {96000, 44100}, {44100, 44099}, {44100, 11025}};
    for (size_t i = 0; i < sizeof(pairs) / sizeof(pairs[0]); i++) {
        int a = pairs[i][0], b = pairs[i][1];
        for (int round = 0; round < 4; round++) {
            size_t n = round == 0 ? 0 : round == 1 ? 1 : (size_t)rnd_in(2, 1500);
            size_t want = (size_t)(((uint64_t)n * (uint64_t)b + (uint64_t)a - 1) / (uint64_t)a);
            resample_all(noise, n, 2, a, b, false, &out);
            resample_all(noise, n, 2, a, b, true, &again);
            CHECK(out.len == want * 2, "%d -> %d: %zu samples gave %zu, not %zu", a, b, n, out.len / 2, want);
            CHECK(again.len == out.len && (!out.len || !memcmp(out.p, again.p, out.len * sizeof(float))),
                  "%d -> %d: feeding in pieces changed the result", a, b);
        }
    }

    /* Each channel is converted on its own: eight at once match one at a time. */
    float eight[8 * 400], one[400];
    for (int i = 0; i < 8 * 400; i++) eight[i] = (float)rnd_in(-30000, 30000) / 32768.0f;
    resample_all(eight, 400, 8, 48000, 44100, true, &out);
    bool same = true;
    for (int ch = 0; ch < 8; ch++) {
        for (int i = 0; i < 400; i++) one[i] = eight[i * 8 + ch];
        resample_all(one, 400, 1, 48000, 44100, false, &again);
        for (size_t k = 0; k < again.len; k++)
            if (out.p[k * 8 + (size_t)ch] != again.p[k]) same = false;
        CHECK(again.len * 8 == out.len, "eight channels: lengths differ");
    }
    CHECK(same, "eight channels at once differ from one at a time");

    /* A steady level comes out as the same steady level. */
    for (int i = 0; i < 3000; i++) in[i] = 0.25f;
    resample_all(in, 3000, 1, 48000, 44100, false, &out);
    double off = 0;
    for (size_t k = 400; k + 400 < out.len; k++)
        if (fabs(out.p[k] - 0.25) > off) off = fabs(out.p[k] - 0.25);
    CHECK(off < 1e-6, "a steady level moved by %.2g", off);

    /* After a reset the resampler behaves as new. */
    ahresample *r = ahresample_new(48000, 44100, 2);
    float buf[4096];
    size_t used;
    ahresample_process(r, noise, 700, &used, buf, 2048);
    ahresample_reset(r);
    resample_all(noise, 1000, 2, 48000, 44100, false, &out);
    again.len = 0;
    size_t made = ahresample_process(r, noise, 1000, &used, buf, 2048);
    pcm_put(&again, buf, made * 2);
    while ((made = ahresample_flush(r, buf, 2048)) != 0) pcm_put(&again, buf, made * 2);
    CHECK(used == 1000 && again.len == out.len && !memcmp(out.p, again.p, out.len * sizeof(float)), "a reset resampler differs from a new one");
    /* Once flushed, more input is not taken and nothing more comes out,
       not even between two flushes. */
    used = 99;
    CHECK(ahresample_process(r, noise, 1000, &used, buf, 2048) == 0 && used == 0 && ahresample_flush(r, buf, 2048) == 0,
          "a flushed resampler gave more");
    ahresample_reset(r);
    made = ahresample_process(r, noise, 1000, &used, buf, 2048);
    size_t first = ahresample_flush(r, buf, 5);
    CHECK(first == 5 && ahresample_process(r, noise, 1000, &used, buf, 2048) == 0 && used == 0, "a resampler part way through its flush took more");
    size_t total = made + first;
    while ((made = ahresample_flush(r, buf, 2048)) != 0) total += made;
    CHECK(total * 2 == out.len, "a resampler interrupted in its flush gave %zu samples, not %zu", total, out.len / 2);
    ahresample_free(r);

    /* Settings that make no sense are refused. */
    CHECK(!ahresample_new(0, 44100, 2) && !ahresample_new(44100, 999, 2) && !ahresample_new(44100, 800000, 2)
              && !ahresample_new(44100, 48000, 0) && !ahresample_new(44100, 48000, 9) && !ahresample_new(-5, -5, 2),
          "bad settings were accepted");
    ahresample_free(NULL);
    free(out.p);
    free(again.p);
}

/* Input that is not sound: nothing may crash, and the damage must not
   outlast the filter's reach. */
static void test_resample_hostile(void)
{
    enum { N = 6000 };
    float *in = malloc(N * sizeof(float));
    unsigned nan_bits = 0x7FC00000u, inf_bits = 0x7F800000u;
    float nan_value, inf_value;
    memcpy(&nan_value, &nan_bits, 4);
    memcpy(&inf_value, &inf_bits, 4);
    for (int i = 0; i < N; i++) {
        int kind = i < 2000 ? rnd_in(0, 5) : 9;
        in[i] = kind == 0 ? nan_value : kind == 1 ? inf_value : kind == 2 ? -inf_value : kind == 3 ? 3.0e38f : kind == 4 ? -1e30f
                                                                                                      : 0.25f;
    }
    pcm out = {0};
    resample_all(in, N, 1, 48000, 44100, true, &out);
    CHECK(out.len == (size_t)(((uint64_t)N * 44100 + 47999) / 48000), "hostile input changed the length");
    /* 2000 input samples of damage end before output sample 1838; the
       filter reaches 80 samples further. */
    bool clean = true;
    for (size_t k = 2000; k + 100 < out.len; k++) {
        unsigned bits;
        memcpy(&bits, out.p + k, 4);
        if ((bits & 0x7F800000u) == 0x7F800000u || fabs(out.p[k] - 0.25) > 1e-5) clean = false;
    }
    CHECK(clean, "damage in the input outlasted the filter");
    free(in);
    free(out.p);
}

/* ---- the resampler against ffmpeg ---------------------------------------------- */

/* Two seconds of stereo with nothing above 80% of the half rate named: tones
   that swell and fade, and a sweep. Faded in and out, so that the two ends
   (where resamplers differ in what they assume lies beyond) are silent. */
static float *band_limited(int rate, double top, size_t *frames)
{
    size_t n = (size_t)rate * 2;
    float *p = malloc(n * 2 * sizeof(float));
    for (size_t i = 0; i < n; i++) {
        double t = (double)i / rate, fade = 1;
        if (t < 0.1) fade = 0.5 - 0.5 * cos(PI * t / 0.1);
        if (t > 1.9) fade = 0.5 - 0.5 * cos(PI * (2 - t) / 0.1);
        double sweep = sin(2 * PI * (100 * t + (top - 100) / 4 * t * t)); /* 100 Hz up to `top` */
        double l = 0.2 * sin(2 * PI * 0.011 * top * t) + 0.2 * sin(2 * PI * 0.37 * top * t) * (1 + 0.5 * sin(2 * PI * 3 * t)) + 0.25 * sweep;
        double r = 0.2 * sin(2 * PI * 0.083 * top * t) + 0.2 * sin(2 * PI * 0.93 * top * t) + 0.2 * sin(2 * PI * 0.59 * top * t + 1);
        p[2 * i] = (float)(l * fade);
        p[2 * i + 1] = (float)(r * fade);
    }
    *frames = n;
    return p;
}

static void against_ffmpeg(int in_rate, int out_rate)
{
    char name[100], path[600], extra[300];
    size_t frames;
    double low = (in_rate < out_rate ? in_rate : out_rate) / 2.0;
    float *in = band_limited(in_rate, 0.8 * low, &frames);
    snprintf(name, sizeof(name), "resample_%d.wav", in_rate);
    path_of(path, name);
    write_wav(path, in, frames, 2, in_rate, 0, 32);
    pcm mine = {0};
    resample_all(in, frames, 2, in_rate, out_rate, true, &mine);

    /* ffmpeg's two resamplers: its own, and soxr at its most exact. */
    static const char *const engines[2] = {"", ":resampler=soxr:precision=28"};
    static const double allowed[2] = {5e-5, 5e-5}; /* 86 dB below the sound */
    for (int e = 0; e < 2; e++) {
        bytes raw = {0};
        snprintf(extra, sizeof(extra), "-af aresample=%d%s", out_rate, engines[e]);
        /* The filter is an output option: it goes after the input. */
        char raw_path[700], args[2000];
        snprintf(raw_path, sizeof(raw_path), "%s.%d.f32", path, e);
        snprintf(args, sizeof(args), "-i \"%s\" %s -f f32le \"%s\"", path, extra, raw_path);
        if (!ffmpeg_run(args) || !read_file(raw_path, &raw)) {
            if (e == 1) printf("  soxr not available in this ffmpeg, skipped\n");
            else CHECK(false, "ffmpeg could not resample %d -> %d", in_rate, out_rate);
            free(raw.p);
            continue;
        }
        pcm ref = {(float *)raw.p, raw.len / sizeof(float), raw.len / sizeof(float), 2, out_rate};
        size_t n = ref.len < mine.len ? ref.len : mine.len;
        long long gap = (long long)ref.len / 2 - (long long)mine.len / 2;
        CHECK(gap >= -2 && gap <= 2, "%d -> %d: ffmpeg gave %zu samples, this %zu", in_rate, out_rate, ref.len / 2, mine.len / 2);
        double d = max_diff(&ref, &mine, 0, n);
        CHECK(d < allowed[e], "%d -> %d: differs from ffmpeg%s by %.2g (%.1f dB)", in_rate, out_rate, engines[e], d, db(d));
        free(raw.p);
    }
    free(in);
    free(mine.p);
}

/* ---- the mix to stereo ------------------------------------------------------------ */

static void test_mix(void)
{
    ahmix m;
    float in[8 * 64], out[2 * 64], copy[8 * 64];
    for (int i = 0; i < 8 * 64; i++) in[i] = (float)rnd_in(-30000, 30000) / 32768.0f;

    /* One channel: to both sides as it is. Two: untouched. */
    CHECK(ahmix_init(&m, 1, 0), "mono refused");
    ahmix_to_stereo(&m, in, 64, out);
    bool same = true;
    for (int i = 0; i < 64; i++)
        if (out[2 * i] != in[i] || out[2 * i + 1] != in[i]) same = false;
    CHECK(same, "mono did not go to both sides unchanged");
    memcpy(copy, in, sizeof(in));
    ahmix_to_stereo(&m, copy, 64, copy); /* in place */
    CHECK(!memcmp(copy, out, sizeof(out)), "mono in place differs");
    CHECK(ahmix_init(&m, 2, 0), "stereo refused");
    ahmix_to_stereo(&m, in, 64, out);
    CHECK(!memcmp(in, out, sizeof(out)), "stereo was changed");

    /* More: full-scale sound in every channel must not clip, the loudest
       side reaches exactly full scale, and the bass effects channel is
       left out. */
    for (int channels = 3; channels <= 8; channels++) {
        CHECK(ahmix_init(&m, channels, 0), "%d channels refused", channels);
        float l = 0, r = 0;
        for (int ch = 0; ch < channels; ch++) {
            l += m.left[ch];
            r += m.right[ch];
        }
        CHECK(l <= 1.00001f && r <= 1.00001f && (l > 0.9999f || r > 0.9999f), "%d channels: sides add up to %.3f and %.3f", channels, l, r);
        if (channels >= 6) CHECK(m.left[3] == 0 && m.right[3] == 0, "%d channels: the LFE channel is mixed in", channels);
        memcpy(copy, in, sizeof(in));
        ahmix_to_stereo(&m, in, 64, out);
        ahmix_to_stereo(&m, copy, 64, copy);
        CHECK(!memcmp(copy, out, sizeof(out)), "%d channels: in place differs", channels);
    }
    /* A layout that does not fit the count falls back to the default. */
    ahmix a, b;
    ahmix_init(&a, 6, 0x3);
    ahmix_init(&b, 6, ahmix_default_layout(6));
    CHECK(!memcmp(&a, &b, sizeof(a)), "a layout that does not fit was used");
    CHECK(!ahmix_init(&m, 0, 0) && !ahmix_init(&m, 9, 0) && ahmix_default_layout(9) == 0, "bad channel counts were accepted");
}

/* The same mix as ffmpeg makes by default, for every usual layout. */
static void mix_against_ffmpeg(int channels, unsigned mask)
{
    enum { FRAMES = 4000 };
    char name[100], path[600];
    float *in = malloc((size_t)FRAMES * (size_t)channels * sizeof(float));
    for (int i = 0; i < FRAMES; i++)
        for (int ch = 0; ch < channels; ch++)
            in[i * channels + ch] = (float)(0.9 * sin(2 * PI * (200.0 + 170.0 * ch) * i / 48000.0 + ch));
    snprintf(name, sizeof(name), "mix_%d_%x.wav", channels, mask);
    path_of(path, name);
    write_wav(path, in, FRAMES, channels, 48000, mask, 16);

    /* Read the file back, so the mix starts from the same 16-bit samples. */
    bytes file = {0}, raw = {0};
    read_file(path, &file);
    pcm mine = {0};
    {
        const char *error = NULL;
        ahreader *r = ahreader_open(test_memory_read, &file, file.len, &error);
        CHECK(r != NULL, "%s: %s", name, error ? error : "?");
        if (r) {
            const ahreader_info *info = ahreader_get_info(r);
            CHECK(info->channels == channels && info->channel_mask == mask, "%s: read as %d channels, mask %x", name, info->channels, info->channel_mask);
            ahmix m;
            ahmix_init(&m, info->channels, info->channel_mask);
            float buf[8 * 512], stereo[2 * 512];
            size_t n;
            while ((n = ahreader_read(r, buf, 512)) != 0) {
                ahmix_to_stereo(&m, buf, n, stereo);
                pcm_put(&mine, stereo, n * 2);
            }
            ahreader_close(r);
        }
    }
    char raw_path[700], args[2000];
    snprintf(raw_path, sizeof(raw_path), "%s.s16", path);
    snprintf(args, sizeof(args), "-i \"%s\" -ac 2 -f s16le \"%s\"", path, raw_path);
    CHECK(ffmpeg_run(args) && read_file(raw_path, &raw), "%s: ffmpeg could not mix it", name);
    size_t n = raw.len / 2;
    CHECK(n == mine.len && n == (size_t)FRAMES * 2, "%s: ffmpeg gave %zu samples, this %zu", name, n, mine.len);
    int worst = 0;
    for (size_t i = 0; i < n && i < mine.len; i++) {
        int ref = (int16_t)(raw.p[2 * i] | raw.p[2 * i + 1] << 8);
        int d = abs(ref - (int)lrintf(mine.p[i] * 32768.0f));
        if (d > worst) worst = d;
    }
    /* ffmpeg mixes 16-bit sound with weights rounded to whole numbers, so
       the two may land a step or two apart. */
    CHECK(worst <= 2, "%s: differs from ffmpeg's mix by %d steps of 16 bits", name, worst);
    free(in);
    free(file.p);
    free(raw.p);
    free(mine.p);
}

void test_convert(void)
{
    printf("sample rate conversion\n");
    test_resample_shape();
    test_resample_tones();
    test_resample_hostile();
    printf("mixing down to stereo\n");
    test_mix();
    if (g_ffmpeg) {
        printf("sample rate conversion against ffmpeg\n");
        against_ffmpeg(48000, 44100);
        against_ffmpeg(96000, 44100);
        against_ffmpeg(22050, 44100);
        against_ffmpeg(44100, 48000);
        against_ffmpeg(32000, 44100);
        printf("mixing down against ffmpeg\n");
        mix_against_ffmpeg(3, 0x7);
        mix_against_ffmpeg(4, 0x33);
        mix_against_ffmpeg(5, 0x37);
        mix_against_ffmpeg(6, 0x3F);
        mix_against_ffmpeg(6, 0x60F);
        mix_against_ffmpeg(7, 0x70F);
        mix_against_ffmpeg(8, 0x63F);
    }
}
