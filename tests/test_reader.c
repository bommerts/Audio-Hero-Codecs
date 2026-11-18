/* The reader that handles every kind of file: what it gives must be what
   ffmpeg gives, a seek must land on the sample, and damage must not hurt. */
#include "../include/ahreader.h"
#include "common.h"

size_t test_memory_read(void *user, uint64_t offset, void *buffer, size_t n)
{
    const bytes *b = user;
    if (offset >= b->len) return 0;
    if (n > b->len - offset) n = (size_t)(b->len - offset);
    memcpy(buffer, b->p + offset, n);
    return n;
}

/* A read function that hands over a little at a time, as a slow network
   drive might. */
static size_t stingy_read(void *user, uint64_t offset, void *buffer, size_t n)
{
    return test_memory_read(user, offset, buffer, n > 777 ? 777 : n);
}

/* Reads a whole file through the reader, in pieces of random size. */
static bool read_all(const bytes *file, ahreader_read_fn read, pcm *out, ahreader_info *info, const char **error)
{
    out->len = 0;
    ahreader *r = ahreader_open(read, (void *)file, file->len, error);
    if (!r) return false;
    *info = *ahreader_get_info(r);
    out->channels = info->channels;
    out->rate = info->sample_rate;
    float buf[8 * 700];
    for (;;) {
        size_t n = ahreader_read(r, buf, (size_t)rnd_in(1, 700));
        if (!n) break;
        pcm_put(out, buf, n * (size_t)info->channels);
        if (out->len > ((size_t)1 << 28)) break; /* a reader that never ends */
    }
    CHECK(ahreader_read(r, buf, 700) == 0, "the reader gave more after its end");
    ahreader_close(r);
    return true;
}

/* Makes a file from the test signal with ffmpeg and checks the reader on
   it. `exact` files (lossless) must seek to the very number. */
static void check_file(const char *name, const char *encode_args, const char *decode_args, ahreader_kind kind, bool exact,
                       bool length_stated)
{
    char src[600], path[600], args[2000];
    path_of(src, "source.wav");
    path_of(path, name);
    snprintf(args, sizeof(args), "-i \"%s\" %s \"%s\"", src, encode_args, path);
    if (!ffmpeg_run(args)) {
        printf("  %s: ffmpeg cannot make it, skipped\n", name);
        return;
    }
    bytes file = {0}, raw = {0};
    read_file(path, &file);
    pcm mine = {0}, slow = {0};
    ahreader_info info, info2;
    const char *error = NULL;
    CHECK(read_all(&file, test_memory_read, &mine, &info, &error), "%s: %s", name, error ? error : "?");
    if (!mine.len) {
        free(file.p);
        return;
    }
    CHECK(info.kind == kind, "%s: taken for %s", name, info.name);
    size_t frames = mine.len / (size_t)info.channels;

    /* The same sound as ffmpeg gives, and as much of it. */
    CHECK(ffmpeg_raw(path, decode_args, "f32le", &raw), "%s: ffmpeg could not decode", name);
    pcm ref = {(float *)raw.p, raw.len / sizeof(float), 0, info.channels, info.sample_rate};
    CHECK(ref.len == mine.len, "%s: ffmpeg gave %zu samples, the reader %zu", name, ref.len / (size_t)info.channels, frames);
    double d = max_diff(&ref, &mine, 0, ref.len);
    CHECK(d < (exact ? 1e-7 : 1e-4), "%s: differs from ffmpeg by %.2g", name, d);

    /* The length it states is the length it gives. */
    if (length_stated) {
        CHECK(info.samples_exact && info.samples == frames, "%s: says %llu samples, gives %zu", name, (unsigned long long)info.samples, frames);
    } else {
        double off = fabs((double)info.samples - (double)frames) / (double)frames;
        CHECK(!info.samples_exact && off < 0.01, "%s: guesses %llu samples, gives %zu", name, (unsigned long long)info.samples, frames);
    }

    /* A read function that gives a little at a time changes nothing. */
    CHECK(read_all(&file, stingy_read, &slow, &info2, &error) && slow.len == mine.len && !memcmp(slow.p, mine.p, mine.len * sizeof(float)),
          "%s: reading the file in small pieces changed the sound", name);

    /* Seeks: to the start, the end, and places in between, back and forth. */
    ahreader *r = ahreader_open(test_memory_read, &file, file.len, &error);
    if (r) {
        float buf[8 * 3000];
        double worst = 0;
        bool lengths = true;
        for (int i = 0; i < 24; i++) {
            size_t at = i == 0 ? 0 : i == 1 ? frames - 1 : i == 2 ? 1 : i == 3 ? frames / 2 : (size_t)rnd_in(0, (int)frames - 1);
            if (!ahreader_seek(r, at)) {
                lengths = false;
                continue;
            }
            size_t want = frames - at < 3000 ? frames - at : 3000, got = 0;
            while (got < want) {
                size_t n = ahreader_read(r, buf + got * (size_t)info.channels, want - got);
                if (!n) break;
                got += n;
            }
            if (got != want) lengths = false;
            pcm part = {buf, got * (size_t)info.channels, 0, info.channels, info.sample_rate};
            pcm whole = {mine.p + at * (size_t)info.channels, mine.len - at * (size_t)info.channels, 0, info.channels, info.sample_rate};
            double e = max_diff(&whole, &part, 0, part.len);
            if (e > worst) worst = e;
        }
        CHECK(lengths, "%s: a seek gave too few samples", name);
        CHECK(worst <= (exact ? 0 : 1e-4), "%s: after a seek the sound differs by %.2g", name, worst);
        /* The end, and past it. */
        if (length_stated) CHECK(!ahreader_seek(r, frames) && ahreader_read(r, buf, 100) == 0, "%s: a seek to the end gave sound", name);
        ahreader_seek(r, (uint64_t)frames + 1000000);
        CHECK(ahreader_read(r, buf, 100) == 0, "%s: a seek past the end gave sound", name);
        /* And it still works afterwards. */
        CHECK(ahreader_seek(r, 0) && ahreader_read(r, buf, 100) == 100 && !memcmp(buf, mine.p, 100 * (size_t)info.channels * sizeof(float)),
              "%s: a seek back from the end failed", name);
        ahreader_close(r);
    }

    /* Damage: cut short, and with bytes changed. Nothing may crash, and no
       file may give much more sound than the whole one does. */
    for (int round = 0; round < 12; round++) {
        bytes bad = {0};
        bytes_put(&bad, file.p, file.len);
        if (round < 6) {
            bad.len = (size_t)rnd_in(0, (int)file.len - 1);
        } else {
            for (int k = 0; k < 40; k++) bad.p[rnd_in(0, (int)bad.len - 1)] = (unsigned char)rnd();
        }
        pcm junk = {0};
        ahreader_info bad_info;
        if (read_all(&bad, test_memory_read, &junk, &bad_info, &error)) {
            CHECK(junk.len / (size_t)bad_info.channels <= frames * 2 + 100000, "%s: a damaged copy gave %zu samples", name, junk.len / (size_t)bad_info.channels);
            ahreader *again = ahreader_open(test_memory_read, &bad, bad.len, NULL);
            if (again) {
                float few[8 * 64];
                for (int k = 0; k < 6; k++) {
                    ahreader_seek(again, (uint64_t)rnd_in(0, (int)frames));
                    ahreader_read(again, few, 64);
                }
                ahreader_close(again);
            }
        } else {
            CHECK(error != NULL, "%s: a damaged copy was refused without a reason", name);
        }
        free(junk.p);
        free(bad.p);
    }
    free(file.p);
    free(raw.p);
    free(mine.p);
    free(slow.p);
}

/* Files the reader must refuse, saying why. */
static void check_refused(const char *name, const char *encode_args, const char *reason_part)
{
    char src[600], path[600], args[2000];
    path_of(src, "source.wav");
    path_of(path, name);
    snprintf(args, sizeof(args), "-i \"%s\" %s \"%s\"", src, encode_args, path);
    if (!ffmpeg_run(args)) {
        printf("  %s: ffmpeg cannot make it, skipped\n", name);
        return;
    }
    bytes file = {0};
    read_file(path, &file);
    const char *error = NULL;
    ahreader *r = ahreader_open(test_memory_read, &file, file.len, &error);
    CHECK(!r && error && strstr(error, reason_part), "%s: %s", name, r ? "was opened" : error ? error : "refused without a reason");
    ahreader_close(r);
    free(file.p);
}

void test_reader(void)
{
    /* Things that are not sound files. */
    bytes junk = {0};
    const char *error = NULL;
    CHECK(!ahreader_open(test_memory_read, &junk, 0, &error) && error, "an empty file was opened");
    for (int round = 0; round < 30; round++) {
        junk.len = 0;
        int n = rnd_in(1, 20000);
        for (int i = 0; i < n; i++) {
            unsigned char c = round < 10 ? 0 : (unsigned char)rnd();
            bytes_put(&junk, &c, 1);
        }
        if (round % 5 == 1 && junk.len > 12) memcpy(junk.p, "RIFFxxxxWAVE", 12);
        if (round % 5 == 2 && junk.len > 4) memcpy(junk.p, "fLaC", 4);
        if (round % 5 == 3 && junk.len > 12) memcpy(junk.p, "\0\0\0\x20" "ftypM4A ", 12);
        pcm out = {0};
        ahreader_info info;
        if (!read_all(&junk, test_memory_read, &out, &info, &error)) CHECK(error != NULL, "random bytes were refused without a reason");
        free(out.p);
    }
    free(junk.p);

    if (!g_ffmpeg) return;
    make_source();
    check_file("reader_lame.mp3", "-ar 44100 -c:a libmp3lame -b:a 128k", NULL, AHREADER_MP3, false, true);
    check_file("reader_lame_vbr.mp3", "-c:a libmp3lame -q:a 2", NULL, AHREADER_MP3, false, true);
    check_file("reader_lame_mono.mp3", "-ar 22050 -ac 1 -c:a libmp3lame -b:a 48k", NULL, AHREADER_MP3, false, true);
    check_file("reader_plain.mp3", "-ar 44100 -c:a libmp3lame -b:a 128k -write_xing 0", NULL, AHREADER_MP3, false, false);
    /* ffmpeg's usual Layer II decoder works in whole numbers; its other one
       is the like-for-like reference. */
    check_file("reader.mp2", "-c:a mp2 -b:a 192k", "-c:a mp2float", AHREADER_MP2, false, false);
    check_file("reader.flac", "-c:a flac", NULL, AHREADER_FLAC, true, true);
    check_file("reader_24.flac", "-ar 96000 -c:a flac -sample_fmt s32 -bits_per_raw_sample 24", NULL, AHREADER_FLAC, true, true);
    check_file("reader_small_blocks.flac", "-c:a flac -frame_size 256", NULL, AHREADER_FLAC, true, true);
    /* AAC without noise bands: their noise is made up afresh by each
       decoder run, so only without them can a seek be held to the sample. */
    check_file("reader.m4a", "-c:a aac -b:a 128k -aac_pns 0", NULL, AHREADER_M4A, false, true);
    check_file("reader_mono.m4a", "-ar 32000 -ac 1 -c:a aac -b:a 64k -aac_pns 0", NULL, AHREADER_M4A, false, true);
    /* A QuickTime file at a rate too high for the short sample entry: the
       rate and the channel count are only in the long one. */
    check_file("reader_96k.mov", "-ar 96000 -c:a aac -b:a 192k -aac_pns 0 -f mov", NULL, AHREADER_M4A, false, true);
    check_file("reader.aac", "-c:a aac -b:a 128k -aac_pns 0 -f adts", NULL, AHREADER_AAC, false, true);
    check_file("reader.wav", "-c:a pcm_s16le", NULL, AHREADER_WAV, true, true);
    check_file("reader_24.wav", "-c:a pcm_s24le", NULL, AHREADER_WAV, true, true);
    check_file("reader_float.wav", "-ar 8000 -c:a pcm_f32le", NULL, AHREADER_WAV, true, true);
    check_file("reader_6ch.wav", "-ac 6 -c:a pcm_s16le", NULL, AHREADER_WAV, true, true);
    check_file("reader_6ch.flac", "-ac 6 -c:a flac", NULL, AHREADER_FLAC, true, true);
    check_file("reader_alac.m4a", "-c:a alac", NULL, AHREADER_ALAC, true, true);
    check_file("reader_alac_24_6ch.m4a", "-ac 6 -c:a alac -sample_fmt s32p", NULL, AHREADER_ALAC, true, true);
    check_file("reader.ogg", "-c:a libvorbis -q:a 4", NULL, AHREADER_VORBIS, false, true);
    check_file("reader_mono.ogg", "-ar 22050 -ac 1 -c:a libvorbis -q:a 0", NULL, AHREADER_VORBIS, false, true);
    check_file("reader_6ch.ogg", "-ac 6 -c:a libvorbis", NULL, AHREADER_VORBIS, false, true);
    check_file("reader_flac.oga", "-c:a flac", NULL, AHREADER_OGG_FLAC, true, true);
    check_file("reader_flac_24_6ch.oga", "-ac 6 -c:a flac -sample_fmt s32 -bits_per_raw_sample 24", NULL, AHREADER_OGG_FLAC, true, true);
    check_refused("reader_fragmented.mp4", "-c:a aac -movflags frag_keyframe+empty_moov -f mp4", "fragmented");
}
