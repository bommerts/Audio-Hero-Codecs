/* Command line tool.
     ahmp3 decode <in> <out.wav> [--raw-float]
     ahmp3 encode <in> <out.mp3> [--bitrate 128] [--vbr 1..10] [--stereo | --mono] [--rate 44100]
   <in> is an MP3 (or MPEG Layer I or II), WAV, FLAC, AAC (ADTS), M4A (AAC
   or Apple Lossless) or Ogg (Vorbis, FLAC or Opus) file; its kind is told from
   its contents. Encoding
   mixes more than two channels down to stereo and changes the sample rate
   when MP3 has no such rate, or when --rate asks. */
#include "../include/ahmix.h"
#include "../include/ahmp3.h"
#include "../include/ahreader.h"
#include "../include/ahresample.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    float *p;       /* interleaved */
    size_t len, cap;
    int rate, channels;
    int bits;       /* of a lossless source, else 0 */
    unsigned mask;  /* which speaker each channel is for, 0 = not said */
} sound;

static void sound_put(sound *s, const float *d, size_t n)
{
    if (s->len + n > s->cap) {
        s->cap = (s->len + n) * 2;
        s->p = realloc(s->p, s->cap * sizeof(float));
        if (!s->p) {
            fprintf(stderr, "out of memory\n");
            exit(1);
        }
    }
    memcpy(s->p + s->len, d, n * sizeof(float));
    s->len += n;
}

static unsigned char *read_file(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char *p = n < 0 ? NULL : malloc((size_t)n + 1);
    if (p && fread(p, 1, (size_t)n, f) != (size_t)n) {
        free(p);
        p = NULL;
    }
    fclose(f);
    *len = p ? (size_t)n : 0;
    return p;
}

typedef struct {
    const unsigned char *p;
    size_t len;
} memory_file;

static size_t memory_read(void *user, uint64_t offset, void *buffer, size_t bytes)
{
    const memory_file *f = user;
    if (offset >= f->len) return 0;
    if (bytes > f->len - offset) bytes = (size_t)(f->len - offset);
    memcpy(buffer, f->p + offset, bytes);
    return bytes;
}

/* Reads a sound file of any kind handled. Returns NULL, or what went wrong. */
static const char *load(const char *path, sound *s)
{
    size_t len;
    unsigned char *data = read_file(path, &len);
    memset(s, 0, sizeof(*s));
    if (!data) return "cannot read the file";
    memory_file file = {data, len};
    const char *error = NULL;
    ahreader *r = ahreader_open(memory_read, &file, len, &error);
    if (r) {
        const ahreader_info *info = ahreader_get_info(r);
        s->rate = info->sample_rate;
        s->channels = info->channels;
        s->bits = info->bits;
        s->mask = info->channel_mask;
        if (info->half_rate) fprintf(stderr, "%s: an HE-AAC file, played as the plain AAC inside it (half the sample rate)\n", path);
        static float pcm[8 * 4096];
        size_t n;
        while ((n = ahreader_read(r, pcm, 4096)) != 0) sound_put(s, pcm, n * (size_t)s->channels);
        ahreader_close(r);
        if (!s->len) error = "the file holds no sound";
    }
    free(data);
    return error;
}

/* Mixes more than two channels down to stereo. */
static void to_stereo(sound *s)
{
    ahmix m;
    if (s->channels <= 2 || !ahmix_init(&m, s->channels, s->mask)) return;
    size_t frames = s->len / (size_t)s->channels;
    ahmix_to_stereo(&m, s->p, frames, s->p);
    s->len = frames * 2;
    s->channels = 2;
    s->mask = 0;
}

/* Changes the sample rate. */
static bool to_rate(sound *s, int rate)
{
    if (rate == s->rate) return true;
    ahresample *r = ahresample_new(s->rate, rate, s->channels);
    if (!r) return false;
    sound out = *s;
    out.p = NULL;
    out.len = out.cap = 0;
    out.rate = rate;
    static float buf[8 * 4096];
    size_t frames = s->len / (size_t)s->channels, pos = 0, used, made;
    while (pos < frames) {
        made = ahresample_process(r, s->p + pos * (size_t)s->channels, frames - pos, &used, buf, 4096);
        sound_put(&out, buf, made * (size_t)s->channels);
        pos += used;
    }
    while ((made = ahresample_flush(r, buf, 4096)) != 0) sound_put(&out, buf, made * (size_t)s->channels);
    ahresample_free(r);
    free(s->p);
    *s = out;
    return true;
}

/* The MP3 rate to use for sound at `rate`: itself when MP3 has it, else the
   next one up, else the highest. */
static int mp3_rate_for(int rate)
{
    static const int rates[] = {8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000};
    for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++)
        if (rates[i] >= rate) return rates[i];
    return 48000;
}

/* ---- commands ---------------------------------------------------------------- */

static void put32(FILE *f, unsigned v) { fputc(v & 255, f); fputc((v >> 8) & 255, f); fputc((v >> 16) & 255, f); fputc(v >> 24, f); }
static void put16(FILE *f, unsigned v) { fputc(v & 255, f); fputc((v >> 8) & 255, f); }

static int decode(const char *in, const char *out, bool raw_float)
{
    sound s;
    const char *error = load(in, &s);
    if (error) {
        fprintf(stderr, "%s: %s\n", in, error);
        return 1;
    }
    FILE *f = fopen(out, "wb");
    if (!f) {
        fprintf(stderr, "cannot write %s\n", out);
        return 1;
    }
    if (raw_float) {
        fwrite(s.p, sizeof(float), s.len, f);
    } else {
        /* A lossless source keeps its sample size; the rest become 16-bit. */
        int bytes = s.bits > 16 ? (s.bits + 7) / 8 : 2;
        unsigned data_bytes = (unsigned)(s.len * (size_t)bytes);
        fwrite("RIFF", 1, 4, f);
        put32(f, 36 + data_bytes);
        fwrite("WAVEfmt ", 1, 8, f);
        put32(f, 16);
        put16(f, 1);
        put16(f, (unsigned)s.channels);
        put32(f, (unsigned)s.rate);
        put32(f, (unsigned)(s.rate * s.channels * bytes));
        put16(f, (unsigned)(s.channels * bytes));
        put16(f, (unsigned)bytes * 8);
        fwrite("data", 1, 4, f);
        put32(f, data_bytes);
        for (size_t i = 0; i < s.len; i++) {
            if (bytes == 2) {
                int16_t v;
                ahmp3_to_s16(s.p + i, &v, 1);
                put16(f, (unsigned)(uint16_t)v);
            } else {
                /* Up to 24 bits this is exact: the float holds the whole
                   number that was decoded. */
                double scaled = (double)s.p[i] * (bytes == 3 ? 8388608.0 : 2147483648.0);
                int32_t v = scaled >= 2147483647.0 ? 2147483647 : (int32_t)scaled;
                fputc(v & 255, f);
                fputc((v >> 8) & 255, f);
                fputc((v >> 16) & 255, f);
                if (bytes == 4) fputc((v >> 24) & 255, f);
            }
        }
    }
    fclose(f);
    printf("%s: %d Hz, %d channel%s, %.2f seconds\n", in, s.rate, s.channels, s.channels == 1 ? "" : "s",
           (double)(s.len / (size_t)s.channels) / s.rate);
    free(s.p);
    return 0;
}

static int encode(const char *in, const char *out, int kbps, ahmp3_mode mode, int vbr, int rate)
{
    sound s;
    const char *error = load(in, &s);
    if (error) {
        fprintf(stderr, "%s: %s\n", in, error);
        return 1;
    }
    to_stereo(&s);
    if (!rate) rate = mp3_rate_for(s.rate);
    if (!to_rate(&s, rate)) {
        fprintf(stderr, "cannot convert %d Hz to %d Hz\n", s.rate, rate);
        return 1;
    }
    ahmp3_encoder_config cfg = {s.rate, s.channels, kbps, mode, vbr};
    ahmp3_encoder *e = ahmp3_encoder_new(&cfg);
    if (!e) {
        fprintf(stderr, "cannot encode %d Hz, %d channels at %d kbps: MP3 has no such combination\n", s.rate, s.channels, kbps);
        return 1;
    }
    FILE *f = fopen(out, "wb");
    if (!f) {
        fprintf(stderr, "cannot write %s\n", out);
        return 1;
    }
    size_t frames = s.len / (size_t)s.channels;
    unsigned char *buf = malloc(AHMP3_ENCODE_BOUND(4096));
    /* A place for the info frame, filled in once the length is known. */
    fwrite(buf, 1, ahmp3_encoder_info_frame(e, buf, AHMP3_ENCODE_BOUND(4096)), f);
    for (size_t i = 0; i < frames; i += 4096) {
        size_t n = frames - i < 4096 ? frames - i : 4096;
        fwrite(buf, 1, ahmp3_encode(e, s.p + i * (size_t)s.channels, n, buf, AHMP3_ENCODE_BOUND(4096)), f);
    }
    fwrite(buf, 1, ahmp3_encoder_flush(e, buf, AHMP3_ENCODE_BOUND(4096)), f);
    fseek(f, 0, SEEK_SET);
    fwrite(buf, 1, ahmp3_encoder_info_frame(e, buf, AHMP3_ENCODE_BOUND(4096)), f);
    fclose(f);
    free(buf);
    ahmp3_encoder_free(e);
    free(s.p);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 4 && !strcmp(argv[1], "decode")) {
        bool raw = false;
        for (int i = 4; i < argc; i++)
            if (!strcmp(argv[i], "--raw-float")) raw = true;
        return decode(argv[2], argv[3], raw);
    }
    if (argc >= 4 && !strcmp(argv[1], "encode")) {
        int kbps = 128, vbr = 0, rate = 0;
        bool kbps_given = false;
        ahmp3_mode mode = AHMP3_JOINT_STEREO;
        for (int i = 4; i < argc; i++) {
            if (!strcmp(argv[i], "--bitrate") && i + 1 < argc) {
                kbps = atoi(argv[++i]);
                kbps_given = true;
            }
            if (!strcmp(argv[i], "--vbr") && i + 1 < argc) vbr = atoi(argv[++i]);
            if (!strcmp(argv[i], "--rate") && i + 1 < argc) rate = atoi(argv[++i]);
            if (!strcmp(argv[i], "--stereo")) mode = AHMP3_STEREO;
            if (!strcmp(argv[i], "--mono")) mode = AHMP3_MONO;
        }
        return encode(argv[2], argv[3], vbr && !kbps_given ? 0 : kbps, mode, vbr, rate);
    }
    fprintf(stderr, "usage: ahmp3 decode <in> <out.wav> [--raw-float]\n"
                    "       ahmp3 encode <in> <out.mp3> [--bitrate 128] [--vbr 1..10] [--stereo | --mono] [--rate 44100]\n"
                    "<in> is an MP3, WAV, FLAC, AAC, M4A (AAC or Apple Lossless) or Ogg (Vorbis, FLAC or Opus) file.\n");
    return 2;
}
