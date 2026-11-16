/* Tests for the WAV reader. */
#include "../include/ahwav.h"
#include "common.h"

/* Reads a whole WAV held in memory, handing the reader `step` bytes more
   each time it runs dry and room for `cap` floats per call. */
static const char *wav_read(const unsigned char *d, size_t len, size_t step, size_t cap, pcm *out, ahwav_format *fmt)
{
    ahwav_decoder *dec = ahwav_decoder_new();
    float *buf = malloc((cap ? cap : 1) * sizeof(float));
    size_t have = step < len ? step : len, pos = 0;
    out->len = 0;
    memset(fmt, 0, sizeof(*fmt));
    for (;;) {
        int n;
        bool eof = have == len;
        size_t used = ahwav_decode(dec, d + pos, have - pos, eof, buf, cap, &n, fmt);
        pos += used;
        if (n) pcm_put(out, buf, (size_t)n * (size_t)fmt->channels);
        if (!used && !n) {
            if (eof || ahwav_decoder_error(dec)) break;
            have = have + step < len ? have + step : len;
        }
    }
    const char *err = ahwav_decoder_error(dec);
    out->channels = fmt->channels;
    out->rate = fmt->sample_rate;
    ahwav_decoder_free(dec);
    free(buf);
    return err;
}

static void put_le(bytes *b, uint64_t v, int n)
{
    for (int i = 0; i < n; i++) {
        unsigned char c = (unsigned char)(v >> (8 * i));
        bytes_put(b, &c, 1);
    }
}

static void put_chunk(bytes *b, const char *id, const void *d, size_t n)
{
    bytes_put(b, id, 4);
    put_le(b, n, 4);
    bytes_put(b, d, n);
    if (n & 1) put_le(b, 0, 1);
}

typedef struct {
    int tag, channels, rate, bytes;
    bool extensible, junk, open_size, rf64;
} wav_kind;

/* Builds a WAV of `frames` random samples and the floats they stand for. */
static void wav_build(const wav_kind *k, size_t frames, bytes *wav, pcm *want)
{
    bytes fmt = {0}, data = {0};
    put_le(&fmt, k->extensible ? 0xFFFE : (unsigned)k->tag, 2);
    put_le(&fmt, (unsigned)k->channels, 2);
    put_le(&fmt, (unsigned)k->rate, 4);
    put_le(&fmt, (uint64_t)k->rate * k->channels * k->bytes, 4);
    put_le(&fmt, (unsigned)(k->channels * k->bytes), 2);
    put_le(&fmt, (unsigned)k->bytes * 8, 2);
    if (k->extensible) {
        static const unsigned char guid_tail[14] = {0, 0, 0, 0, 0x10, 0, 0x80, 0, 0, 0xAA, 0, 0x38, 0x9B, 0x71};
        put_le(&fmt, 22, 2);
        put_le(&fmt, (unsigned)k->bytes * 8, 2);
        put_le(&fmt, 0, 4);
        put_le(&fmt, (unsigned)k->tag, 2);
        bytes_put(&fmt, guid_tail, 14);
    }
    want->len = 0;
    for (size_t i = 0; i < frames * (size_t)k->channels; i++) {
        uint32_t r = (rnd() << 8) ^ rnd();
        /* The ends of the range now and then. */
        int edge = rnd_in(0, 40);
        float f;
        if (k->tag == 3) {
            double v = edge == 0 ? 1.0 : edge == 1 ? -1.0 : (double)(int32_t)r / 2147483648.0 * 1.5;
            if (k->bytes == 4) {
                f = (float)v;
                uint32_t bits;
                memcpy(&bits, &f, 4);
                put_le(&data, bits, 4);
            } else {
                uint64_t bits;
                memcpy(&bits, &v, 8);
                put_le(&data, bits, 8);
                f = (float)v;
            }
        } else {
            int bits = k->bytes * 8;
            int32_t v = (int32_t)r >> (32 - bits);
            if (edge == 0) v = (int32_t)(((int64_t)1 << (bits - 1)) - 1);
            if (edge == 1) v = (int32_t)(-((int64_t)1 << (bits - 1)));
            put_le(&data, k->bytes == 1 ? (uint64_t)(v + 128) : (uint64_t)(uint32_t)v, k->bytes);
            f = (float)((double)v / (double)((int64_t)1 << (bits - 1)));
        }
        pcm_put(want, &f, 1);
    }
    wav->len = 0;
    bytes_put(wav, k->rf64 ? "RF64" : "RIFF", 4);
    put_le(wav, k->rf64 ? 0xFFFFFFFFu : 0, 4); /* patched below */
    bytes_put(wav, "WAVE", 4);
    if (k->rf64) {
        bytes ds = {0};
        put_le(&ds, 0, 8);
        put_le(&ds, data.len, 8);
        put_le(&ds, frames, 8);
        put_le(&ds, 0, 4);
        put_chunk(wav, "ds64", ds.p, ds.len);
        free(ds.p);
    }
    if (k->junk) put_chunk(wav, "JUNK", "odd sized junk before the format", 31);
    put_chunk(wav, "fmt ", fmt.p, fmt.len);
    if (k->junk) put_chunk(wav, "LIST", "INFOISFT\4\0\0\0abc", 16);
    bytes_put(wav, "data", 4);
    put_le(wav, k->rf64 || k->open_size ? 0xFFFFFFFFu : data.len, 4);
    bytes_put(wav, data.p, data.len);
    if (!k->open_size) {
        if (data.len & 1) put_le(wav, 0, 1);
        put_chunk(wav, "LIST", "trailing chunk, not sound!", 26);
    }
    if (!k->rf64) {
        uint32_t riff = (uint32_t)(wav->len - 8);
        memcpy(wav->p + 4, &riff, 4); /* the tests run on little-endian machines */
    }
    free(fmt.p);
    free(data.p);
}

static bool same_floats(const pcm *a, const pcm *b)
{
    return a->len == b->len && (a->len == 0 || !memcmp(a->p, b->p, a->len * sizeof(float)));
}

static void test_built(void)
{
    static const wav_kind kinds[] = {
        {1, 1, 8000, 1, false, false, false, false},  {1, 2, 44100, 2, false, false, false, false},
        {1, 2, 48000, 3, false, true, false, false},  {1, 6, 96000, 4, false, false, false, false},
        {3, 2, 44100, 4, false, false, false, false}, {3, 1, 192000, 8, false, true, false, false},
        {1, 2, 44100, 2, true, false, false, false},  {1, 8, 48000, 3, true, true, false, false},
        {3, 2, 48000, 4, true, false, false, false},  {1, 2, 44100, 2, false, false, true, false},
        {1, 3, 22050, 3, false, true, true, false},   {1, 2, 44100, 2, false, false, false, true},
        {1, 1, 11025, 1, false, true, false, false},  {3, 5, 32000, 8, true, true, true, false},
    };
    bytes wav = {0};
    pcm want = {0}, got = {0};
    for (size_t i = 0; i < sizeof(kinds) / sizeof(kinds[0]); i++) {
        const wav_kind *k = &kinds[i];
        size_t frames = (size_t)rnd_in(1, 3000);
        wav_build(k, frames, &wav, &want);
        ahwav_format f;
        const char *err = wav_read(wav.p, wav.len, wav.len, 1 << 16, &got, &f);
        CHECK(!err && same_floats(&want, &got), "WAV kind %zu: wrong samples (%s)", i, err ? err : "no error");
        CHECK(f.sample_rate == k->rate && f.channels == k->channels && f.bits == k->bytes * 8 && f.is_float == (k->tag == 3),
              "WAV kind %zu: wrong format", i);
        CHECK(f.samples == (k->open_size ? 0 : frames), "WAV kind %zu: says %llu samples, has %zu", i,
              (unsigned long long)f.samples, frames);
        /* A few bytes at a time, and room for only a few samples per call. */
        err = wav_read(wav.p, wav.len, (size_t)rnd_in(1, 13), (size_t)rnd_in(k->channels, 50), &got, &f);
        CHECK(!err && same_floats(&want, &got), "WAV kind %zu: reading in small pieces differs", i);
    }
    free(wav.p);
    free(want.p);
    free(got.p);
}

static void test_against_ffmpeg(void)
{
    static const char *codecs[] = {"pcm_u8", "pcm_s16le", "pcm_s24le", "pcm_s32le", "pcm_f32le", "pcm_f64le"};
    static const char *extra[] = {"", "-ac 1 -ar 22050", "-rf64 always", "-ac 6 -ar 96000"};
    char src[600];
    path_of(src, "source.wav");
    for (int c = 0; c < 6; c++) {
        for (int x = 0; x < 4; x++) {
            char name[64], path[600], args[1600];
            snprintf(name, sizeof(name), "wav_%s_%d.wav", codecs[c], x);
            path_of(path, name);
            snprintf(args, sizeof(args), "-i \"%s\" -c:a %s %s \"%s\"", src, codecs[c], extra[x], path);
            CHECK(ffmpeg_run(args), "%s: ffmpeg could not write it", name);
            bytes wav = {0}, ref = {0};
            pcm got = {0};
            ahwav_format f;
            read_file(path, &wav);
            const char *err = wav_read(wav.p, wav.len, 4096, 4096, &got, &f);
            CHECK(ffmpeg_raw(path, NULL, "f32le", &ref), "%s: ffmpeg could not read it back", name);
            CHECK(!err && got.len * 4 == ref.len && got.len > 0 && !memcmp(got.p, ref.p, ref.len),
                  "%s: differs from ffmpeg (%zu vs %zu samples, %s)", name, got.len, ref.len / 4, err ? err : "no error");
            free(wav.p);
            free(ref.p);
            free(got.p);
        }
    }
}

static void test_bad(void)
{
    bytes wav = {0};
    pcm want = {0}, got = {0};
    ahwav_format f;
    wav_kind k = {1, 2, 44100, 2, false, true, false, false};
    wav_build(&k, 500, &wav, &want);

    CHECK(wav_read((const unsigned char *)"ID3\4\0\0\0\0\0\0junk that is no WAV", 30, 30, 64, &got, &f) != NULL, "junk read as WAV");
    CHECK(wav_read(wav.p, 0, 1, 64, &got, &f) != NULL && got.len == 0, "an empty file read as WAV");
    CHECK(wav_read(wav.p, 30, 30, 64, &got, &f) != NULL && got.len == 0, "a file cut off in the header read as WAV");
    /* Cut off in the sound: what is there comes out. */
    const char *err = wav_read(wav.p, wav.len - 700, 100, 64, &got, &f);
    CHECK(!err && got.len > 0 && got.len < want.len && !memcmp(got.p, want.p, got.len * 4), "a cut-off file lost its sound");

    /* A format this reader does not handle (tag 2, ADPCM). */
    bytes bad = {0};
    bytes_put(&bad, wav.p, wav.len);
    for (size_t i = 12; i + 8 < bad.len; i++)
        if (!memcmp(bad.p + i, "fmt ", 4)) {
            bad.p[i + 8] = 2;
            break;
        }
    CHECK(wav_read(bad.p, bad.len, 64, 64, &got, &f) != NULL && got.len == 0, "ADPCM was not refused");

    /* Random damage to the first 200 bytes, and random bytes: no crash, no
       more sound than the file could hold. */
    for (int round = 0; round < 3000; round++) {
        bad.len = 0;
        bytes_put(&bad, wav.p, wav.len);
        int hits = rnd_in(1, 8);
        for (int i = 0; i < hits; i++) bad.p[rnd() % 200] = (unsigned char)rnd();
        if (round % 5 == 0) bad.len = rnd() % bad.len;
        wav_read(bad.p, bad.len, (size_t)rnd_in(1, 500), (size_t)rnd_in(0, 300), &got, &f);
        if (got.len > bad.len) {
            CHECK(false, "damaged WAV gave %zu samples from %zu bytes", got.len, bad.len);
            break;
        }
    }
    CHECK(true, "damaged WAV files read without a crash");
    free(bad.p);
    free(wav.p);
    free(want.p);
    free(got.p);
}

void test_wav(void)
{
    test_built();
    test_bad();
    if (g_ffmpeg) {
        make_source();
        test_against_ffmpeg();
    }
}
