/* Tests for Layers I and II. ffmpeg has no Layer I encoder and its Layer II
   encoders use only part of the format, so a small frame writer here builds
   frames with every allocation, scale factor pattern and stereo bound, and
   ffmpeg decodes them for comparison. */
#include "../src/ahmp3_internal.h"
#include "common.h"

typedef struct {
    int errors, frames, layer, samples;
} run_info;

static void decode_all(const unsigned char *d, size_t len, pcm *out, run_info *info)
{
    ahmp3_decoder *dec = ahmp3_decoder_new();
    size_t pos = 0;
    out->len = 0;
    memset(info, 0, sizeof(*info));
    for (;;) {
        float buf[AHMP3_MAX_SAMPLES * 2];
        int n;
        ahmp3_frame fr;
        size_t used = ahmp3_decode(dec, d + pos, len - pos, true, buf, &n, &fr);
        if (!used) break;
        pos += used;
        if (!n) continue;
        info->frames++;
        info->errors += fr.crc_error;
        info->layer = fr.layer;
        info->samples = fr.samples;
        out->channels = fr.channels;
        pcm_put(out, buf, (size_t)n * (size_t)fr.channels);
    }
    ahmp3_decoder_free(dec);
}

static bool ffmpeg_decode(const char *path, int layer, pcm *out)
{
    bytes b = {0};
    bool ok = ffmpeg_raw(path, layer == 1 ? "-c:a mp1float" : "-c:a mp2float", "f32le", &b);
    out->len = 0;
    if (ok) pcm_put(out, (const float *)b.p, b.len / 4);
    free(b.p);
    return ok;
}

/* Largest difference in samples [from, to) of b against a[shift + ...],
   relative to the louder signal. ffmpeg works Layers I and II out in whole
   numbers of 2^-23, so a very quiet frame is held to that step instead. */
static double layer_diff(const pcm *a, const pcm *b, size_t shift, size_t from, size_t to)
{
    double diff = 0, peak = 0;
    for (size_t i = from; i < to && i + shift < a->len && i < b->len; i++) {
        double d = fabs((double)a->p[i + shift] - b->p[i]);
        if (d > diff) diff = d;
        if (fabs(b->p[i]) > peak) peak = fabs(b->p[i]);
    }
    return diff / (peak > 0.03 ? peak : 0.03);
}

/* ---- a frame writer ----------------------------------------------------------- */

typedef struct {
    unsigned char p[8192];
    int bits;
} bitw;

static void put(bitw *w, unsigned v, int n)
{
    for (int i = n - 1; i >= 0; i--, w->bits++)
        if ((v >> i) & 1) w->p[w->bits >> 3] |= (unsigned char)(0x80 >> (w->bits & 7));
}

typedef struct {
    int layer, lsf, sr_index, bitrate_index, mode; /* mode: 0 stereo, 1 joint, 2 dual, 3 mono */
    bool crc;
    int free_bytes; /* > 0: free format frames of this size */
} layer_kind;

static int stated_bytes(const layer_kind *k)
{
    int rate = ahmp3_sample_rates[k->sr_index], kbps = ahmp3_bitrates_12[k->lsf][k->layer - 1][k->bitrate_index];
    return k->layer == 1 ? 12000 * kbps / rate * 4 : 144000 * kbps / rate;
}

/* One frame of random sound. */
static void frame_build(const layer_kind *k, bytes *out)
{
    int nch = k->mode == 3 ? 1 : 2;
    int rate = ahmp3_sample_rates[k->sr_index], kbps = ahmp3_bitrates_12[k->lsf][k->layer - 1][k->bitrate_index];
    int bytes_in_frame = k->free_bytes ? k->free_bytes : stated_bytes(k);
    int budget = (stated_bytes(k) - 4) * 8 - 16; /* what fits with or without a CRC, stated or free */
    ahmp3_l2_table table;
    int sblimit = 32;
    if (k->layer == 2) {
        ahmp3_l2_select(k->lsf, rate, kbps / nch, &table);
        sblimit = table.sblimit;
    }
    int mode_ext = rnd_in(0, 3);
    int bound = k->mode == 1 ? 4 + 4 * mode_ext : sblimit;
    if (bound > sblimit) bound = sblimit;

    int alloc[2][32], scfsi[2][32], sf[2][32][3];
    for (int empty = 1;; empty++) {
        /* Allocations at random; emptier and emptier until the frame fits. */
        int bits = 0;
        for (int sb = 0; sb < sblimit; sb++) {
            int nb = k->layer == 1 ? 4 : table.bits[sb];
            for (int ch = 0; ch < nch; ch++) {
                if (sb >= bound && ch) {
                    alloc[1][sb] = alloc[0][sb];
                } else {
                    alloc[ch][sb] = rnd_in(0, empty) > 1 ? 0 : rnd_in(1, (1 << nb) - 1 - (k->layer == 1));
                    bits += nb;
                    if (alloc[ch][sb]) {
                        const ahmp3_l2_class *c = k->layer == 2 ? &ahmp3_l2_classes[table.rows[sb][alloc[ch][sb] - 1]] : NULL;
                        bits += k->layer == 1 ? 12 * (alloc[ch][sb] + 1) : 12 * c->bits * (c->grouped ? 1 : 3);
                    }
                }
                scfsi[ch][sb] = rnd_in(0, 3);
                static const int counts[4] = {3, 2, 1, 2};
                if (alloc[ch][sb]) bits += k->layer == 1 ? 6 : 2 + 6 * counts[scfsi[ch][sb]];
            }
        }
        if (bits <= budget) break;
    }
    /* Scale factors from loud to very quiet; the frame as a whole stays
       near full scale. */
    int loudest = rnd_in(0, 3) ? rnd_in(6, 20) : rnd_in(0, 62);
    for (int sb = 0; sb < sblimit; sb++)
        for (int ch = 0; ch < nch; ch++) {
            int a = rnd_in(loudest, 62), b = rnd_in(loudest, 62), c = rnd_in(loudest, 62);
            int *s = sf[ch][sb];
            switch (k->layer == 1 ? 2 : scfsi[ch][sb]) {
            case 0: s[0] = a; s[1] = b; s[2] = c; break;
            case 1: s[0] = s[1] = a; s[2] = b; break;
            case 2: s[0] = s[1] = s[2] = a; break;
            default: s[0] = a; s[1] = s[2] = b; break;
            }
        }

    bitw w;
    memset(&w, 0, sizeof(w));
    int version = k->lsf ? (k->sr_index >= 6 ? 0 : 2) : 3;
    put(&w, 0x7FF, 11);
    put(&w, (unsigned)version, 2);
    put(&w, (unsigned)(4 - k->layer), 2);
    put(&w, !k->crc, 1);
    put(&w, k->free_bytes ? 0 : (unsigned)k->bitrate_index, 4);
    put(&w, (unsigned)(k->sr_index % 3), 2);
    put(&w, 0, 1); /* padding */
    put(&w, 0, 1);
    put(&w, (unsigned)k->mode, 2);
    put(&w, k->mode == 1 ? (unsigned)mode_ext : 0, 2);
    put(&w, 0, 4);
    if (k->crc) put(&w, 0, 16);
    int protected_from = w.bits;
    for (int sb = 0; sb < sblimit; sb++)
        for (int ch = 0; ch < (sb < bound ? nch : 1); ch++) put(&w, (unsigned)alloc[ch][sb], k->layer == 1 ? 4 : table.bits[sb]);
    if (k->layer == 2)
        for (int sb = 0; sb < sblimit; sb++)
            for (int ch = 0; ch < nch; ch++)
                if (alloc[ch][sb]) put(&w, (unsigned)scfsi[ch][sb], 2);
    if (k->crc) {
        unsigned crc = ahmp3_crc16(0xFFFF, w.p + 2, 16);
        crc = ahmp3_crc16(crc, w.p + 6, (size_t)(w.bits - protected_from));
        w.p[4] = (unsigned char)(crc >> 8);
        w.p[5] = (unsigned char)crc;
    }
    for (int sb = 0; sb < sblimit; sb++)
        for (int ch = 0; ch < nch; ch++) {
            if (!alloc[ch][sb]) continue;
            const int *s = sf[ch][sb];
            if (k->layer == 1) put(&w, (unsigned)s[0], 6);
            else if (scfsi[ch][sb] == 0) { put(&w, (unsigned)s[0], 6); put(&w, (unsigned)s[1], 6); put(&w, (unsigned)s[2], 6); }
            else if (scfsi[ch][sb] == 2) put(&w, (unsigned)s[0], 6);
            else { put(&w, (unsigned)s[0], 6); put(&w, (unsigned)s[2], 6); }
        }
    for (int group = 0; group < 12; group++)
        for (int sb = 0; sb < sblimit; sb++)
            for (int ch = 0; ch < (sb < bound ? nch : 1); ch++) {
                int a = alloc[ch][sb];
                if (!a) continue;
                if (k->layer == 1) {
                    put(&w, (unsigned)rnd_in(0, (2 << a) - 2), a + 1);
                    continue;
                }
                const ahmp3_l2_class *c = &ahmp3_l2_classes[table.rows[sb][a - 1]];
                int v[3];
                for (int i = 0; i < 3; i++) v[i] = rnd_in(0, c->levels - 1);
                if (c->grouped) put(&w, (unsigned)(v[0] + c->levels * (v[1] + c->levels * v[2])), c->bits);
                else for (int i = 0; i < 3; i++) put(&w, (unsigned)v[i], c->bits);
            }
    bytes_put(out, w.p, (size_t)bytes_in_frame);
}

static void stream_build(const layer_kind *k, int frames, bytes *out)
{
    out->len = 0;
    for (int i = 0; i < frames; i++) frame_build(k, out);
}

/* ---- tests -------------------------------------------------------------------- */

static void test_random(const char *name, layer_kind k, int frames)
{
    bytes mp = {0};
    stream_build(&k, frames, &mp);
    pcm mine = {0}, ref = {0};
    run_info info;
    decode_all(mp.p, mp.len, &mine, &info);
    int per = k.layer == 1 ? 384 : 1152, nch = k.mode == 3 ? 1 : 2;
    CHECK(info.frames == frames && info.layer == k.layer && info.samples == per && !info.errors && mine.len == (size_t)frames * (size_t)per * (size_t)nch,
          "%s: %d frames of Layer %d, %d samples each, %d CRC errors", name, info.frames, info.layer, info.samples, info.errors);
    if (g_ffmpeg) {
        char path[600];
        path_of(path, name);
        write_file(path, mp.p, mp.len);
        CHECK(ffmpeg_decode(path, k.layer, &ref), "%s: ffmpeg could not decode", name);
        CHECK(mine.len == ref.len, "%s: lengths differ: %zu vs %zu", name, mine.len, ref.len);
        size_t frame_samples = (size_t)per * (size_t)nch;
        int bad = 0;
        for (size_t at = 0; at < ref.len && bad < 5; at += frame_samples) {
            double d = layer_diff(&mine, &ref, 0, at, at + frame_samples);
            if (d > 1e-4) {
                bad++;
                CHECK(false, "%s: frame %zu differs from ffmpeg (diff %.2g)", name, at / frame_samples, d);
            }
        }
        printf("  %s: %d frames compared\n", name, frames);
    }
    free(mp.p);
    free(mine.p);
    free(ref.p);
}

static void test_encoded(const char *name, const char *args)
{
    char src[600], path[600], cmd[2000];
    path_of(src, "source.wav");
    path_of(path, name);
    snprintf(cmd, sizeof(cmd), "-i \"%s\" %s \"%s\"", src, args, path);
    if (!ffmpeg_run(cmd)) {
        printf("  %s: encoder not available in this ffmpeg, skipped\n", name);
        return;
    }
    bytes b = {0};
    read_file(path, &b);
    pcm mine = {0}, ref = {0};
    run_info info;
    decode_all(b.p, b.len, &mine, &info);
    CHECK(ffmpeg_decode(path, 2, &ref), "%s: ffmpeg could not decode", name);
    /* twolame's first joint stereo frame says "joint" and the rest "stereo";
       ffmpeg's file reader drops a first frame that differs from the second,
       so there the comparison starts one frame in. */
    size_t lead = mine.len == ref.len + (size_t)1152 * (size_t)mine.channels ? (size_t)1152 * (size_t)mine.channels : 0;
    CHECK(mine.len == ref.len + lead && ref.len > 0 && info.layer == 2 && !info.errors, "%s: lengths differ: %zu vs %zu (%d CRC errors)", name,
          mine.len, ref.len, info.errors);
    /* ...and one more, past what the dropped frame left in the filter. */
    CHECK(layer_diff(&mine, &ref, lead, lead, ref.len) < 1e-4, "%s: decodes differently from ffmpeg (%.2g)", name,
          layer_diff(&mine, &ref, lead, lead, ref.len));
    free(b.p);
    free(mine.p);
    free(ref.p);
}

/* Free format: the same frames with a stated bit rate and as bigger
   free-format frames decode to the same sound. */
static void test_free(layer_kind k, int free_bytes)
{
    bytes stated = {0}, loose = {0};
    pcm a = {0}, b = {0};
    run_info ia, ib;
    unsigned seed = g_seed;
    stream_build(&k, 40, &stated);
    g_seed = seed; /* the same frames again, each in a bigger frame without a bit rate */
    k.free_bytes = free_bytes;
    stream_build(&k, 40, &loose);
    decode_all(stated.p, stated.len, &a, &ia);
    decode_all(loose.p, loose.len, &b, &ib);
    CHECK(ia.frames == 40 && ib.frames == 40 && !ia.errors && !ib.errors && a.len == b.len && a.len > 0 && !memcmp(a.p, b.p, a.len * sizeof(float)),
          "Layer %d free format (%d bytes): %d frames, %d CRC errors, differs from the same frames with a bit rate", k.layer, free_bytes, ib.frames,
          ib.errors);
    free(stated.p);
    free(loose.p);
    free(a.p);
    free(b.p);
}

static void test_crc_and_damage(void)
{
    /* A damaged protected bit: that frame is reported and silent. */
    for (int layer = 1; layer <= 2; layer++) {
        layer_kind k = {layer, 0, 0, 10, 0, true, 0};
        bytes mp = {0};
        stream_build(&k, 30, &mp);
        int size = stated_bytes(&k);
        for (int i = 0; i < 30; i += 3) mp.p[i * size + 6 + rnd_in(0, 3)] ^= (unsigned char)(1 << rnd_in(0, 7));
        ahmp3_decoder *d = ahmp3_decoder_new();
        size_t pos = 0;
        int frames = 0, wrong = 0;
        for (;;) {
            float buf[AHMP3_MAX_SAMPLES * 2];
            int n;
            ahmp3_frame fr;
            size_t used = ahmp3_decode(d, mp.p + pos, mp.len - pos, true, buf, &n, &fr);
            if (!used) break;
            pos += used;
            if (!n) continue;
            double energy = 0;
            for (int i = 2 * (n - 30); i < 2 * n; i++) energy += (double)buf[i] * buf[i];
            /* A Layer II frame is longer than the filter's memory: the
               end of a failed one is silent. */
            if (fr.crc_error != (frames % 3 == 0) || (layer == 2 && fr.crc_error != (energy < 1e-12))) wrong++;
            frames++;
        }
        ahmp3_decoder_free(d);
        CHECK(frames == 30 && !wrong, "Layer %d CRC: %d frames, %d of them wrong", layer, frames, wrong);

        /* Random damage and cut-off streams: no crash, no runaway output. */
        bytes bad = {0};
        pcm out = {0};
        run_info info;
        for (int round = 0; round < 300; round++) {
            bad.len = 0;
            bytes_put(&bad, mp.p, mp.len);
            int hits = rnd_in(1, 100);
            for (int i = 0; i < hits; i++) bad.p[rnd() % bad.len] = (unsigned char)rnd();
            if (round % 4 == 0) bad.len = rnd() % bad.len;
            decode_all(bad.p, bad.len, &out, &info);
            if (out.len > (size_t)31 * 1152 * 2) {
                CHECK(false, "a damaged Layer %d stream gave %zu samples", layer, out.len);
                break;
            }
        }
        CHECK(true, "damaged Layer %d streams decoded without a crash", layer);
        free(bad.p);
        free(out.p);
        free(mp.p);
    }
}

void test_layers(void)
{
    /* Layer II: every allocation table (four for MPEG-1, one for the low
       sample rates), each stereo mode, with and without a CRC. */
    test_random("rnd_l2_48k_192.mp2", (layer_kind){2, 0, 1, 10, 0, false, 0}, 120);
    test_random("rnd_l2_44k_384_joint.mp2", (layer_kind){2, 0, 0, 14, 1, true, 0}, 120);
    test_random("rnd_l2_44k_128.mp2", (layer_kind){2, 0, 0, 8, 2, false, 0}, 80);
    test_random("rnd_l2_44k_48_mono.mp2", (layer_kind){2, 0, 0, 2, 3, true, 0}, 80);
    test_random("rnd_l2_32k_32_mono.mp2", (layer_kind){2, 0, 2, 1, 3, false, 0}, 80);
    test_random("rnd_l2_32k_96_joint.mp2", (layer_kind){2, 0, 2, 6, 1, false, 0}, 80);
    test_random("rnd_l2_48k_64_mono.mp2", (layer_kind){2, 0, 1, 4, 3, false, 0}, 80);
    test_random("rnd_l2_22k_160_joint.mp2", (layer_kind){2, 1, 3, 14, 1, true, 0}, 120);
    test_random("rnd_l2_16k_32_mono.mp2", (layer_kind){2, 1, 5, 4, 3, false, 0}, 80);
    test_random("rnd_l2_24k_64.mp2", (layer_kind){2, 1, 4, 8, 0, false, 0}, 80);
    /* Layer I. */
    test_random("rnd_l1_44k_448.mp1", (layer_kind){1, 0, 0, 14, 0, false, 0}, 200);
    test_random("rnd_l1_48k_256_joint.mp1", (layer_kind){1, 0, 1, 8, 1, true, 0}, 200);
    test_random("rnd_l1_32k_96_mono.mp1", (layer_kind){1, 0, 2, 3, 3, true, 0}, 150);
    test_random("rnd_l1_22k_256_joint.mp1", (layer_kind){1, 1, 3, 14, 1, false, 0}, 200);
    test_random("rnd_l1_16k_64_mono.mp1", (layer_kind){1, 1, 5, 4, 3, false, 0}, 150);

    test_free((layer_kind){2, 0, 1, 10, 0, false, 0}, 700);
    test_free((layer_kind){2, 0, 0, 10, 1, true, 0}, 800);
    test_free((layer_kind){2, 1, 3, 14, 1, false, 0}, 3000);
    test_free((layer_kind){1, 0, 0, 14, 0, true, 0}, 600);
    test_free((layer_kind){1, 1, 5, 4, 3, false, 0}, 400);
    test_crc_and_damage();

    if (g_ffmpeg) {
        make_source();
        test_encoded("mp2_192.mp2", "-c:a mp2 -b:a 192k");
        test_encoded("mp2_44k_128.mp2", "-ar 44100 -c:a mp2 -b:a 128k");
        test_encoded("mp2_32k_mono_48.mp2", "-ar 32000 -ac 1 -c:a mp2 -b:a 48k");
        test_encoded("mp2_16k_32.mp2", "-ar 16000 -ac 1 -c:a mp2 -b:a 32k");
        test_encoded("twolame_44k_384.mp2", "-ar 44100 -c:a libtwolame -b:a 384k -mode stereo");
        test_encoded("twolame_joint_crc.mp2", "-ar 44100 -c:a libtwolame -b:a 96k -mode joint_stereo -error_protection 1");
        test_encoded("twolame_dual.mp2", "-c:a libtwolame -b:a 160k -mode dual_channel");
        test_encoded("twolame_48k_mono_crc.mp2", "-ac 1 -c:a libtwolame -b:a 32k -error_protection 1");
        test_encoded("twolame_22k_joint.mp2", "-ar 22050 -c:a libtwolame -b:a 64k -mode joint_stereo -error_protection 1");
        test_encoded("twolame_24k.mp2", "-ar 24000 -c:a libtwolame -b:a 160k");
    }
}
