/* Command line tool.
     ahmp3 decode in.mp3 out.wav [--raw-float] [--no-trim]
     ahmp3 encode in.wav out.mp3 [--bitrate 128] [--stereo | --mono]
   WAV input must be 16-bit PCM or 32-bit float. */
#include "../include/ahmp3.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned char *read_file(const char *path, size_t *len)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char *p = malloc((size_t)n + 1);
    if (p && fread(p, 1, (size_t)n, f) != (size_t)n) { free(p); p = NULL; }
    fclose(f);
    *len = (size_t)n;
    return p;
}

static void put32(FILE *f, unsigned v) { fputc(v & 255, f); fputc((v >> 8) & 255, f); fputc((v >> 16) & 255, f); fputc(v >> 24, f); }
static void put16(FILE *f, unsigned v) { fputc(v & 255, f); fputc((v >> 8) & 255, f); }

static void wav_header(FILE *f, int rate, int channels, bool fl, unsigned data_bytes)
{
    int bytes = fl ? 4 : 2;
    fwrite("RIFF", 1, 4, f); put32(f, 36 + data_bytes); fwrite("WAVEfmt ", 1, 8, f);
    put32(f, 16); put16(f, fl ? 3 : 1); put16(f, (unsigned)channels); put32(f, (unsigned)rate);
    put32(f, (unsigned)(rate * channels * bytes)); put16(f, (unsigned)(channels * bytes)); put16(f, (unsigned)(bytes * 8));
    fwrite("data", 1, 4, f); put32(f, data_bytes);
}

static int decode(const char *in, const char *out, bool raw_float, bool trim)
{
    size_t len;
    unsigned char *data = read_file(in, &len);
    if (!data) { fprintf(stderr, "cannot read %s\n", in); return 1; }
    FILE *f = fopen(out, "wb");
    if (!f) { fprintf(stderr, "cannot write %s\n", out); return 1; }
    ahmp3_decoder *d = ahmp3_decoder_new();
    float *all = NULL;
    size_t total = 0, cap = 0;
    int rate = 0, channels = 0;
    size_t pos = 0;
    for (;;) {
        float pcm[AHMP3_MAX_SAMPLES * 2];
        int n;
        ahmp3_frame fr;
        size_t used = ahmp3_decode(d, data + pos, len - pos, true, pcm, &n, &fr);
        if (!used) break;
        pos += used;
        if (!n) continue;
        if (!rate) { rate = fr.sample_rate; channels = fr.channels; }
        if (fr.channels != channels) continue;
        if (total + (size_t)n * channels > cap) {
            cap = (total + (size_t)n * channels) * 2;
            all = realloc(all, cap * sizeof(float));
        }
        memcpy(all + total, pcm, (size_t)n * channels * sizeof(float));
        total += (size_t)n * channels;
    }
    size_t first = 0, frames = channels ? total / channels : 0;
    const ahmp3_info *info = ahmp3_decoder_info(d);
    if (trim && info->encoder_delay >= 0) {
        size_t skip = (size_t)info->encoder_delay + AHMP3_DECODER_DELAY;
        size_t pad = info->encoder_padding > AHMP3_DECODER_DELAY ? (size_t)(info->encoder_padding - AHMP3_DECODER_DELAY) : 0;
        first = skip < frames ? skip : frames;
        frames = frames - first > pad ? frames - first - pad : 0;
    } else {
        frames -= first;
    }
    if (!raw_float) wav_header(f, rate, channels, false, (unsigned)(frames * channels * 2));
    for (size_t i = 0; i < frames * channels; i += 4096) {
        size_t n = frames * channels - i < 4096 ? frames * channels - i : 4096;
        if (raw_float) {
            fwrite(all + first * channels + i, sizeof(float), n, f);
        } else {
            int16_t s[4096];
            ahmp3_to_s16(all + first * channels + i, s, n);
            fwrite(s, 2, n, f);
        }
    }
    fclose(f);
    ahmp3_decoder_free(d);
    free(all);
    free(data);
    return rate ? 0 : 1;
}

static unsigned get32(const unsigned char *p) { return p[0] | p[1] << 8 | p[2] << 16 | (unsigned)p[3] << 24; }

static int encode(const char *in, const char *out, int kbps, ahmp3_mode mode)
{
    size_t len;
    unsigned char *wav = read_file(in, &len);
    if (!wav || len < 44 || memcmp(wav, "RIFF", 4) || memcmp(wav + 8, "WAVE", 4)) { fprintf(stderr, "not a WAV file\n"); return 1; }
    int format = 0, channels = 0, rate = 0, bits = 0;
    const unsigned char *pcm = NULL;
    size_t pcm_len = 0;
    for (size_t p = 12; p + 8 <= len;) {
        unsigned size = get32(wav + p + 4);
        if (!memcmp(wav + p, "fmt ", 4) && p + 24 <= len) {
            format = wav[p + 8] | wav[p + 9] << 8;
            channels = wav[p + 10] | wav[p + 11] << 8;
            rate = (int)get32(wav + p + 12);
            bits = wav[p + 22] | wav[p + 23] << 8;
            if (format == 0xFFFE && p + 32 <= len) format = wav[p + 32] | wav[p + 33] << 8;
        } else if (!memcmp(wav + p, "data", 4)) {
            pcm = wav + p + 8;
            pcm_len = size < len - p - 8 ? size : len - p - 8;
        }
        p += 8 + size + (size & 1);
    }
    if (!pcm || !(format == 1 && bits == 16) && !(format == 3 && bits == 32)) { fprintf(stderr, "need 16-bit or float WAV\n"); return 1; }
    ahmp3_encoder_config cfg = {rate, channels, kbps, mode};
    ahmp3_encoder *e = ahmp3_encoder_new(&cfg);
    if (!e) { fprintf(stderr, "unsupported: %d Hz, %d channels, %d kbps\n", rate, channels, kbps); return 1; }
    FILE *f = fopen(out, "wb");
    size_t frames = pcm_len / (size_t)(channels * bits / 8);
    unsigned char *buf = malloc(AHMP3_ENCODE_BOUND(4096));
    for (size_t i = 0; i < frames; i += 4096) {
        size_t n = frames - i < 4096 ? frames - i : 4096;
        size_t w = format == 3 ? ahmp3_encode(e, (const float *)pcm + i * channels, n, buf, AHMP3_ENCODE_BOUND(4096))
                               : ahmp3_encode_s16(e, (const int16_t *)pcm + i * channels, n, buf, AHMP3_ENCODE_BOUND(4096));
        fwrite(buf, 1, w, f);
    }
    fwrite(buf, 1, ahmp3_encoder_flush(e, buf, AHMP3_ENCODE_BOUND(4096)), f);
    fclose(f);
    free(buf);
    ahmp3_encoder_free(e);
    free(wav);
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
        int kbps = 128;
        ahmp3_mode mode = AHMP3_JOINT_STEREO;
        for (int i = 4; i < argc; i++) {
            if (!strcmp(argv[i], "--bitrate") && i + 1 < argc) kbps = atoi(argv[++i]);
            if (!strcmp(argv[i], "--stereo")) mode = AHMP3_STEREO;
            if (!strcmp(argv[i], "--mono")) mode = AHMP3_MONO;
        }
        return encode(argv[2], argv[3], kbps, mode);
    }
    fprintf(stderr, "usage: ahmp3 decode in.mp3 out.wav [--raw-float] [--no-trim]\n"
                    "       ahmp3 encode in.wav out.mp3 [--bitrate 128] [--stereo | --mono]\n");
    return 2;
}
