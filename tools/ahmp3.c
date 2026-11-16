/* Command line tool.
     ahmp3 decode <in> <out.wav> [--raw-float] [--no-trim]
     ahmp3 encode <in> <out.mp3> [--bitrate 128] [--vbr 1..10] [--stereo | --mono]
   <in> is an MP3 (or MPEG Layer I or II), WAV, FLAC, AAC (ADTS) or M4A file;
   its kind is told from its contents. */
#include "../include/ahaac.h"
#include "../include/ahflac.h"
#include "../include/ahm4a.h"
#include "../include/ahmp3.h"
#include "../include/ahwav.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    float *p;       /* interleaved */
    size_t len, cap;
    int rate, channels;
    int bits;       /* of a lossless source, else 0 */
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

/* Keeps samples [first, first + count) per channel. */
static void sound_trim(sound *s, size_t first, size_t count)
{
    size_t frames = s->channels ? s->len / (size_t)s->channels : 0;
    if (first > frames) first = frames;
    if (count > frames - first) count = frames - first;
    memmove(s->p, s->p + first * (size_t)s->channels, count * (size_t)s->channels * sizeof(float));
    s->len = count * (size_t)s->channels;
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

/* ---- one loader per kind of file ------------------------------------------- */

static const char *load_mpeg(const unsigned char *data, size_t len, bool trim, sound *s)
{
    ahmp3_decoder *d = ahmp3_decoder_new();
    size_t pos = 0;
    int layer = 0;
    for (;;) {
        float pcm[AHMP3_MAX_SAMPLES * 2];
        int n;
        ahmp3_frame fr;
        size_t used = ahmp3_decode(d, data + pos, len - pos, true, pcm, &n, &fr);
        if (!used) break;
        pos += used;
        if (!n) continue;
        if (!s->rate) {
            s->rate = fr.sample_rate;
            s->channels = fr.channels;
            layer = fr.layer;
        }
        if (fr.channels == s->channels) sound_put(s, pcm, (size_t)n * (size_t)fr.channels);
    }
    /* Gapless: drop the encoder's and the decoder's delay, and the padding. */
    const ahmp3_info *info = ahmp3_decoder_info(d);
    if (trim && layer == 3 && info->encoder_delay >= 0) {
        size_t frames = s->len / (size_t)s->channels, skip = (size_t)info->encoder_delay + AHMP3_DECODER_DELAY;
        size_t pad = info->encoder_padding > AHMP3_DECODER_DELAY ? (size_t)(info->encoder_padding - AHMP3_DECODER_DELAY) : 0;
        sound_trim(s, skip, frames > skip + pad ? frames - skip - pad : 0);
    }
    ahmp3_decoder_free(d);
    return s->rate ? NULL : "no MPEG audio frames found";
}

static const char *load_wav(const unsigned char *data, size_t len, sound *s)
{
    ahwav_decoder *d = ahwav_decoder_new();
    static float pcm[8192];
    size_t pos = 0;
    for (;;) {
        int n;
        ahwav_format f;
        size_t used = ahwav_decode(d, data + pos, len - pos, true, pcm, 8192, &n, &f);
        pos += used;
        if (n) {
            s->rate = f.sample_rate;
            s->channels = f.channels;
            sound_put(s, pcm, (size_t)n * (size_t)f.channels);
        }
        if (!used && !n) break;
    }
    const char *error = ahwav_decoder_error(d);
    ahwav_decoder_free(d);
    return error ? error : s->rate ? NULL : "the WAV file holds no sound";
}

static const char *load_flac(const unsigned char *data, size_t len, sound *s)
{
    ahflac_decoder *d = ahflac_decoder_new();
    static int32_t pcm[8192];
    static float as_float[8192];
    size_t pos = 0;
    for (;;) {
        int n;
        ahflac_format f;
        size_t used = ahflac_decode(d, data + pos, len - pos, true, pcm, 8192, &n, &f);
        pos += used;
        if (n) {
            s->rate = f.sample_rate;
            s->channels = f.channels;
            s->bits = f.bits;
            ahflac_to_float(pcm, as_float, (size_t)n * (size_t)f.channels, f.bits);
            sound_put(s, as_float, (size_t)n * (size_t)f.channels);
        }
        if (!used && !n) break;
    }
    const char *error = ahflac_decoder_error(d);
    ahflac_decoder_free(d);
    return error ? error : s->rate ? NULL : "the FLAC file holds no sound";
}

static const char *load_adts(const unsigned char *data, size_t len, sound *s)
{
    ahaac_decoder *d = ahaac_decoder_new();
    size_t pos = 0;
    for (;;) {
        float pcm[AHAAC_MAX_SAMPLES * AHAAC_MAX_CHANNELS];
        int n;
        ahaac_frame fr;
        size_t used = ahaac_decode(d, data + pos, len - pos, true, pcm, &n, &fr);
        if (!used) break;
        pos += used;
        if (!n) continue;
        if (!s->rate) {
            s->rate = fr.sample_rate;
            s->channels = fr.channels;
        }
        if (fr.channels == s->channels) sound_put(s, pcm, (size_t)n * (size_t)fr.channels);
    }
    const char *error = ahaac_decoder_error(d);
    ahaac_decoder_free(d);
    return s->rate ? NULL : error ? error : "no AAC frames found";
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

static const char *load_m4a(const unsigned char *data, size_t len, bool trim, sound *s)
{
    memory_file file = {data, len};
    const char *error = NULL;
    ahm4a *m = ahm4a_open(memory_read, &file, len, &error);
    if (!m) return error;
    const ahm4a_info *info = ahm4a_get_info(m);
    ahaac_decoder *d = ahaac_decoder_new();
    if (!ahaac_decoder_configure(d, info->config, info->config_bytes)) error = ahaac_decoder_error(d);
    unsigned char *packet = malloc((size_t)info->max_packet_bytes + 1);
    for (uint64_t i = 0; !error && i < info->packets; i++) {
        float pcm[AHAAC_MAX_SAMPLES * AHAAC_MAX_CHANNELS];
        int n;
        ahaac_frame fr;
        size_t bytes = ahm4a_read_packet(m, i, packet, (size_t)info->max_packet_bytes);
        if (!bytes || !ahaac_decode_packet(d, packet, bytes, pcm, &n, &fr) || !n) continue;
        if (!s->rate) {
            s->rate = fr.sample_rate;
            s->channels = fr.channels;
        }
        if (fr.channels == s->channels) sound_put(s, pcm, (size_t)n * (size_t)fr.channels);
    }
    if (!error && !s->rate) error = ahaac_decoder_error(d) ? ahaac_decoder_error(d) : "the M4A file holds no sound that could be decoded";
    if (!error && trim) sound_trim(s, info->delay, (size_t)info->samples);
    free(packet);
    ahaac_decoder_free(d);
    ahm4a_close(m);
    return error;
}

/* Reads a sound file of any kind handled. Returns NULL, or what went wrong. */
static const char *load(const char *path, bool trim, sound *s)
{
    size_t len;
    unsigned char *data = read_file(path, &len);
    memset(s, 0, sizeof(*s));
    if (!data) return "cannot read the file";
    /* Past a tag at the front, the first bytes tell the kinds apart. */
    size_t at = ahmp3_id3v2_size(data, len);
    if (at > len) at = 0;
    const unsigned char *p = data + at;
    size_t left = len - at;
    const char *error;
    if (left >= 12 && (!memcmp(p, "RIFF", 4) || !memcmp(p, "RF64", 4))) error = load_wav(p, left, s);
    else if (left >= 4 && !memcmp(p, "fLaC", 4)) error = load_flac(p, left, s);
    else if (left >= 12 && !memcmp(p + 4, "ftyp", 4)) error = load_m4a(p, left, trim, s);
    else {
        /* MPEG audio and ADTS both start with eleven 1 bits; ADTS has two
           0 bits where MPEG audio names its layer. */
        size_t i = 0;
        while (i + 2 <= left && !(p[i] == 0xFF && (p[i + 1] & 0xE0) == 0xE0)) i++;
        if (i + 2 <= left && (p[i + 1] & 0xF6) == 0xF0) error = load_adts(p, left, s);
        else error = load_mpeg(data, len, trim, s);
    }
    free(data);
    return error;
}

/* ---- commands ---------------------------------------------------------------- */

static void put32(FILE *f, unsigned v) { fputc(v & 255, f); fputc((v >> 8) & 255, f); fputc((v >> 16) & 255, f); fputc(v >> 24, f); }
static void put16(FILE *f, unsigned v) { fputc(v & 255, f); fputc((v >> 8) & 255, f); }

static int decode(const char *in, const char *out, bool raw_float, bool trim)
{
    sound s;
    const char *error = load(in, trim, &s);
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

static int encode(const char *in, const char *out, int kbps, ahmp3_mode mode, int vbr)
{
    sound s;
    const char *error = load(in, true, &s);
    if (error) {
        fprintf(stderr, "%s: %s\n", in, error);
        return 1;
    }
    if (s.channels > 2) {
        fprintf(stderr, "%s: more than two channels\n", in);
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
        bool raw = false, trim = true;
        for (int i = 4; i < argc; i++) {
            if (!strcmp(argv[i], "--raw-float")) raw = true;
            if (!strcmp(argv[i], "--no-trim")) trim = false;
        }
        return decode(argv[2], argv[3], raw, trim);
    }
    if (argc >= 4 && !strcmp(argv[1], "encode")) {
        int kbps = 128, vbr = 0;
        bool kbps_given = false;
        ahmp3_mode mode = AHMP3_JOINT_STEREO;
        for (int i = 4; i < argc; i++) {
            if (!strcmp(argv[i], "--bitrate") && i + 1 < argc) {
                kbps = atoi(argv[++i]);
                kbps_given = true;
            }
            if (!strcmp(argv[i], "--vbr") && i + 1 < argc) vbr = atoi(argv[++i]);
            if (!strcmp(argv[i], "--stereo")) mode = AHMP3_STEREO;
            if (!strcmp(argv[i], "--mono")) mode = AHMP3_MONO;
        }
        return encode(argv[2], argv[3], vbr && !kbps_given ? 0 : kbps, mode, vbr);
    }
    fprintf(stderr, "usage: ahmp3 decode <in> <out.wav> [--raw-float] [--no-trim]\n"
                    "       ahmp3 encode <in> <out.mp3> [--bitrate 128] [--vbr 1..10] [--stereo | --mono]\n"
                    "<in> is an MP3, WAV, FLAC, AAC or M4A file.\n");
    return 2;
}
