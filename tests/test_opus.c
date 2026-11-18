/* Opus: files from libopus and random streams against ffmpeg's libopus
   decoder (asked for floats: it gives 16-bit samples otherwise), the RFC's
   own test vectors when they are at hand, the start and end the pages
   state, seeks, and damaged packets. */
#include "common.h"
#include "../include/ahogg.h"
#include "../include/ahopus.h"
#include "../src/ahopus_internal.h"

static const char *LIBOPUS = "-c:a libopus -request_sample_fmt flt";

static int g_opus_rounds = 30;
static double g_seek_worst = 0; /* the most a seek has differed, relative to full scale */

/* ---- writing Ogg Opus ----------------------------------------------------------- */

static void le16(bytes *b, int v)
{
    unsigned char c[2] = {(unsigned char)v, (unsigned char)(v >> 8)};
    bytes_put(b, c, 2);
}

static void le32(bytes *b, uint32_t v)
{
    unsigned char c[4] = {(unsigned char)v, (unsigned char)(v >> 8), (unsigned char)(v >> 16), (unsigned char)(v >> 24)};
    bytes_put(b, c, 4);
}

typedef struct {
    int channels, pre_skip, gain, family, streams, coupled;
    uint8_t map[8];
} head_spec;

static void put_head(bytes *b, const head_spec *h)
{
    unsigned char v[2] = {1, (unsigned char)h->channels};
    bytes_put(b, "OpusHead", 8);
    bytes_put(b, v, 2);
    le16(b, h->pre_skip);
    le32(b, 48000);
    le16(b, h->gain);
    unsigned char f = (unsigned char)h->family;
    bytes_put(b, &f, 1);
    if (h->family) {
        unsigned char sc[2] = {(unsigned char)h->streams, (unsigned char)h->coupled};
        bytes_put(b, sc, 2);
        bytes_put(b, h->map, (size_t)h->channels);
    }
}

static void put_tags(bytes *b)
{
    bytes_put(b, "OpusTags", 8);
    le32(b, 4);
    bytes_put(b, "test", 4);
    le32(b, 0);
}

/* The two header packets, each on a page of its own, then the sound. */
static void start_packets(opacket *pk, const head_spec *h)
{
    memset(pk, 0, 2 * sizeof(*pk));
    put_head(&pk[0].data, h);
    put_tags(&pk[1].data);
    pk[0].page_end = pk[1].page_end = true;
}

static void free_list(opacket *pk, int count)
{
    for (int i = 0; i < count; i++) free(pk[i].data.p);
}

/* ---- random packets ------------------------------------------------------------ */

static void put_size(bytes *b, int n)
{
    unsigned char c[2];
    if (n < 252) {
        c[0] = (unsigned char)n;
        bytes_put(b, c, 1);
    } else {
        c[0] = (unsigned char)(252 + (n & 3));
        c[1] = (unsigned char)((n - c[0]) / 4);
        bytes_put(b, c, 2);
    }
}

static void put_random(bytes *b, int n)
{
    for (int i = 0; i < n; i++) {
        unsigned char c = (unsigned char)rnd();
        bytes_put(b, &c, 1);
    }
}

static int frame_bytes(void)
{
    int k = rnd_in(0, 7);
    return k < 4 ? rnd_in(10, 150) : k < 6 ? rnd_in(2, 20) : rnd_in(150, 600);
}

/* One stream's part of a packet: `count` frames of TOC config `config`,
   random contents, each framing code the count allows, self-delimited for
   all streams but the last. */
static void put_stream_packet(bytes *b, int config, bool stereo, int count, bool self_delimited)
{
    int sizes[48], code;
    if (count == 1) code = rnd_in(0, 3) ? 0 : 3;
    else if (count == 2) code = rnd_in(1, 3);
    else code = 3;
    bool cbr = code == 1 || (code == 3 && rnd_in(0, 1));
    int common = frame_bytes();
    for (int i = 0; i < count; i++) sizes[i] = cbr ? common : frame_bytes();
    unsigned char toc = (unsigned char)(config << 3 | (stereo ? 4 : 0) | code);
    bytes_put(b, &toc, 1);
    int pad = 0;
    if (code == 2) {
        put_size(b, sizes[0]);
    } else if (code == 3) {
        bool padded = rnd_in(0, 3) == 0;
        unsigned char ch = (unsigned char)((cbr ? 0 : 0x80) | (padded ? 0x40 : 0) | count);
        bytes_put(b, &ch, 1);
        if (padded) {
            pad = rnd_in(0, 2) ? rnd_in(0, 20) : rnd_in(254, 600);
            int left = pad;
            while (left >= 255) {
                unsigned char p = 255;
                bytes_put(b, &p, 1);
                left -= 254;
            }
            unsigned char p = (unsigned char)left;
            bytes_put(b, &p, 1);
        }
        if (!cbr)
            for (int i = 0; i < count - 1; i++) put_size(b, sizes[i]);
    }
    if (self_delimited) put_size(b, sizes[count - 1]);
    for (int i = 0; i < count; i++) put_random(b, sizes[i]);
    put_random(b, pad);
}

/* A TOC config for the class (CELT, or SILK and hybrid) and frame size. */
static int config_for(bool celt, int frame)
{
    if (celt) {
        int sz = frame == 120 ? 0 : frame == 240 ? 1 : frame == 480 ? 2 : 3;
        return 16 + rnd_in(0, 3) * 4 + sz;
    }
    if (frame >= 1920 || rnd_in(0, 1)) {
        int sz = frame == 480 ? 0 : frame == 960 ? 1 : frame == 1920 ? 2 : 3;
        return rnd_in(0, 2) * 4 + sz;
    }
    return 12 + rnd_in(0, 1) * 2 + (frame == 960);
}

typedef struct {
    head_spec head;
    bool celt;
    int switch_at; /* SILK and hybrid until this packet, CELT from it; -1: no switch */
    int64_t switch_pos; /* where that packet's sound starts, as decoded */
    int64_t total, end; /* samples decoded, and the last page's granule */
} stream_desc;

/* A random stream: the headers, then packets of random frames. */
static int random_stream(opacket *pk, int most, stream_desc *sd)
{
    head_spec *h = &sd->head;
    memset(h, 0, sizeof(*h));
    sd->celt = rnd_in(0, 2) == 0;
    /* A switch to CELT (one way: CELT to SILK without the redundant frame
       an encoder adds is covered by libopus with a concealment of its own) */
    sd->switch_at = !sd->celt && rnd_in(0, 1) ? rnd_in(3, 15) : -1;
    sd->switch_pos = -1;
    if (rnd_in(0, 2)) {
        h->family = 0;
        h->channels = rnd_in(1, 2);
        h->streams = 1;
        h->coupled = h->channels - 1;
    } else {
        h->family = 1;
        h->channels = rnd_in(1, 8);
        h->streams = rnd_in(1, 4);
        h->coupled = rnd_in(0, h->streams);
        for (int c = 0; c < h->channels; c++)
            h->map[c] = (uint8_t)(rnd_in(0, 9) ? rnd_in(0, h->streams + h->coupled - 1) : 255);
    }
    h->pre_skip = rnd_in(0, 1) ? 312 : rnd_in(0, 4000);
    h->gain = rnd_in(0, 2) ? 0 : rnd_in(-3000, 1500);
    start_packets(pk, h);
    int count = 2, packets = rnd_in(20, 60);
    int64_t pos = 0;
    static const int celt_frames[4] = {120, 240, 480, 960}, silk_frames[4] = {480, 960, 1920, 2880};
    int frame = sd->celt ? celt_frames[rnd_in(0, 3)] : silk_frames[rnd_in(0, 3)];
    for (int p = 0; p < packets && count < most; p++) {
        bool celt = sd->celt || (sd->switch_at >= 0 && p >= sd->switch_at);
        if (p == sd->switch_at) sd->switch_pos = pos;
        if (rnd_in(0, 4) == 0 || p == sd->switch_at) frame = celt ? celt_frames[rnd_in(0, 3)] : silk_frames[rnd_in(0, 3)];
        int max_count = 5760 / frame;
        int n = rnd_in(0, 3) ? 1 : rnd_in(1, max_count < 6 ? max_count : 6);
        opacket *k = &pk[count++];
        memset(k, 0, sizeof(*k));
        for (int s = 0; s < h->streams; s++)
            put_stream_packet(&k->data, config_for(celt, frame), rnd_in(0, 1), n, s != h->streams - 1);
        pos += (int64_t)frame * n;
        k->end = pos;
        k->page_end = rnd_in(0, 3) == 0;
    }
    sd->total = pos;
    /* An end cut within the last packet, at times. */
    int64_t last = count > 3 ? pos - pk[count - 2].end : pos;
    sd->end = rnd_in(0, 1) ? pos : pos - rnd_in(0, (int)last - 1);
    if (sd->end < h->pre_skip) sd->end = pos;
    pk[count - 1].end = sd->end;
    return count;
}

static void describe(const stream_desc *sd)
{
    printf("    %s%s, family %d, %d channels, %d streams (%d coupled), pre-skip %d, gain %d\n", sd->celt ? "CELT" : "SILK and hybrid", sd->switch_at >= 0 ? " then CELT" : "",
           sd->head.family, sd->head.channels, sd->head.streams, sd->head.coupled, sd->head.pre_skip, sd->head.gain);
}

/* The reader's sound and ffmpeg's libopus decoder's: same length, and the
   same to float rounding (relative to the louder), apart from `skip_len`
   samples from `skip_from` (a join libopus conceals its own way). */
static bool against_ffmpeg_skip(const char *what, const bytes *file, double tolerance, double *worst_out, int64_t skip_from, int skip_len)
{
    char path[600], name[100];
    snprintf(name, sizeof(name), "%s.opus", what);
    path_of(path, name);
    write_file(path, file->p, file->len);
    pcm mine = {0};
    ahreader_info info;
    const char *error = NULL;
    size_t frames = read_through(file, &mine, &info, &error);
    bytes raw = {0};
    bool ok = false;
    if (!frames) {
        printf("    %s: %s\n", what, error ? error : "no sound");
    } else if (ffmpeg_raw(path, LIBOPUS, "f32le", &raw)) {
        pcm ref = {(float *)raw.p, raw.len / sizeof(float), 0, info.channels, 48000};
        for (int64_t i = skip_from < 0 ? 0 : skip_from; skip_from >= 0 && i < skip_from + skip_len && (size_t)i < frames; i++)
            for (int c = 0; c < info.channels; c++)
                if ((size_t)i * info.channels + c < ref.len) ref.p[i * info.channels + c] = mine.p[i * info.channels + c];
        double worst = ref.len == mine.len ? max_diff(&ref, &mine, 0, ref.len) : 1;
        if (worst_out) *worst_out = worst;
        ok = ref.len == mine.len && worst <= tolerance;
        if (!ok)
            printf("    %s: ffmpeg gave %zu samples, this decoder %zu; worst difference %.3g\n", what, ref.len / (size_t)info.channels,
                   frames, worst);
    } else {
        printf("    %s: ffmpeg could not decode it\n", what);
    }
    free(raw.p);
    free(mine.p);
    return ok;
}

static bool against_ffmpeg(const char *what, const bytes *file, double tolerance, double *worst_out)
{
    return against_ffmpeg_skip(what, file, tolerance, worst_out, -1, 0);
}

/* ---- files from libopus -------------------------------------------------------- */

static void test_libopus_files(void)
{
    static const struct {
        const char *name, *args;
    } files[] = {
        {"silk_nb", "-ac 1 -b:a 6k -application voip"},
        {"silk_mono", "-ac 1 -b:a 12k -application voip"},
        {"silk_stereo", "-b:a 20k -application voip"},
        {"silk_fec", "-ac 1 -b:a 24k -application voip -packet_loss 20 -fec 1"},
        {"silk_40ms", "-ac 1 -b:a 16k -application voip -frame_duration 40"},
        {"silk_60ms", "-b:a 24k -application voip -frame_duration 60"},
        {"hybrid", "-b:a 32k -application voip"},
        {"hybrid_mono", "-ac 1 -b:a 28k -application audio"},
        {"celt_48k", "-b:a 48k -application audio"},
        {"celt_96k_10ms", "-b:a 96k -frame_duration 10"},
        {"celt_5ms", "-b:a 64k -application lowdelay -frame_duration 5"},
        {"celt_2_5ms", "-b:a 128k -application lowdelay -frame_duration 2.5"},
        {"celt_cbr", "-b:a 64k -vbr off"},
        {"celt_max", "-b:a 510k"},
        {"long_packets", "-b:a 64k -frame_duration 120"},
        {"surround_3", "-af aformat=channel_layouts=3.0 -b:a 96k"},
        {"surround_4", "-af aformat=channel_layouts=quad -b:a 128k"},
        {"surround_6", "-ac 6 -b:a 192k"},
        {"surround_8", "-ac 8 -b:a 256k"},
    };
    char src[600], path[600], args[2000];
    path_of(src, "source.wav");
    double worst = 0;
    for (size_t f = 0; f < sizeof(files) / sizeof(files[0]); f++) {
        char name[100];
        snprintf(name, sizeof(name), "libopus_%s.opus", files[f].name);
        path_of(path, name);
        snprintf(args, sizeof(args), "-i \"%s\" -c:a libopus %s \"%s\"", src, files[f].args, path);
        if (!ffmpeg_run(args)) {
            printf("  %s: ffmpeg cannot make it, skipped\n", name);
            continue;
        }
        bytes file = {0};
        read_file(path, &file);
        double w = 0;
        CHECK(against_ffmpeg(name, &file, 1e-5, &w), "%s: differs from ffmpeg", name);
        if (w > worst) worst = w;
        free(file.p);
    }
    printf("  libopus files: worst difference from ffmpeg %.3g\n", worst);
}

/* ---- random streams -------------------------------------------------------------- */

static void test_random_streams(void)
{
    const char *seed_env = getenv("AHO_SEED"), *rounds_env = getenv("AHO_ROUNDS");
    unsigned base = seed_env ? (unsigned)strtoul(seed_env, NULL, 10) : g_seed;
    int rounds = rounds_env ? atoi(rounds_env) : g_opus_rounds;
    static opacket pk[200];
    double worst = 0;
    for (int round = 0; round < rounds; round++) {
        unsigned seed = base + (unsigned)round * 7919u;
        g_seed = seed;
        stream_desc sd;
        int count = random_stream(pk, 200, &sd);
        bytes file = {0};
        ogg_write(&file, (uint32_t)rnd(), pk, count, 0);
        char what[64];
        snprintf(what, sizeof(what), "random_opus_%u", seed);
        double w = 0;
        /* A switch to CELT not covered by a redundant frame is joined by
           concealment, which RFC 6716 leaves open: the 5 ms there are not
           compared. */
        int64_t at = sd.switch_pos >= 0 ? sd.switch_pos - sd.head.pre_skip : -1;
        bool ok = against_ffmpeg_skip(what, &file, 1e-4, &w, at, 240);
        if (!ok) describe(&sd);
        CHECK(ok, "random stream (seed %u) differs from ffmpeg", seed);
        if (w > worst) worst = w;
        free(file.p);
        free_list(pk, count);
    }
    printf("  random streams: worst difference from ffmpeg %.3g\n", worst);
}

/* ---- the RFC's test vectors --------------------------------------------------------- */

static uint32_t be32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

/* RFC 8251's test vectors, when AHOPUS_VECTORS names the folder holding
   them: the range decoder must end each packet in the state the vector
   records, and the sound must match the reference to a 16-bit step. */
static void test_vectors(void)
{
    const char *dir = getenv("AHOPUS_VECTORS");
    if (!dir) {
        printf("  the RFC test vectors were not run (set AHOPUS_VECTORS to their folder)\n");
        return;
    }
    for (int v = 1; v <= 12; v++) {
        char bit_path[700], dec_path[700];
        snprintf(bit_path, sizeof(bit_path), "%s/testvector%02d.bit", dir, v);
        snprintf(dec_path, sizeof(dec_path), "%s/testvector%02d.dec", dir, v);
        bytes bit = {0}, dec = {0};
        if (!read_file(bit_path, &bit) || !read_file(dec_path, &dec)) {
            CHECK(false, "test vector %d is missing", v);
            continue;
        }
        head_spec h = {2, 0, 0, 0, 1, 1, {0}};
        bytes head = {0};
        put_head(&head, &h);
        ahopus_decoder *d = ahopus_decoder_new();
        ahopus_decoder_header(d, head.p, head.len, NULL);
        static float out[AHOPUS_MAX_SAMPLES * 2];
        size_t pos = 0, at = 0;
        int ranges_wrong = 0, worst = 0;
        while (pos + 8 <= bit.len) {
            uint32_t len = be32(bit.p + pos), rng = be32(bit.p + pos + 4);
            pos += 8;
            if (len > bit.len - pos) break;
            int n = 0;
            ahopus_decode_packet(d, len ? bit.p + pos : NULL, len, out, &n);
            if (len && ahopus_final_range(d) != rng) ranges_wrong++;
            pos += len;
            for (int i = 0; i < 2 * n && at + 2 <= dec.len; i++, at += 2) {
                float x = out[i] * 32768.f;
                x = x > 32767 ? 32767 : x < -32768 ? -32768 : x;
                int ref = (int16_t)(dec.p[at] | dec.p[at + 1] << 8), e = abs((int)floorf(x + .5f) - ref);
                if (e > worst) worst = e;
            }
        }
        CHECK(ranges_wrong == 0 && worst <= 1 && at == dec.len, "test vector %d: %d packets end in another state, worst difference %d",
              v, ranges_wrong, worst);
        ahopus_decoder_free(d);
        free(head.p);
        free(bit.p);
        free(dec.p);
    }
}

/* ---- start, end and seeks ------------------------------------------------------------ */

/* The packets of an Ogg Opus file, each with the place it ends at. */
static int take_opus_packets(const bytes *file, opacket *pk, int most)
{
    const char *error = NULL;
    ahogg *o = ahogg_open(test_memory_read, (void *)file, file->len, &error);
    if (!o) return 0;
    int count = 0;
    for (int i = 0; i < 2; i++) {
        const uint8_t *d;
        size_t n;
        memset(&pk[count], 0, sizeof(pk[count]));
        if (ahogg_header(o, i, &d, &n)) bytes_put(&pk[count].data, d, n);
        pk[count++].page_end = true;
    }
    ahopus_decoder *dec = ahopus_decoder_new();
    ahopus_decoder_header(dec, pk[0].data.p, pk[0].data.len, NULL);
    static float out[AHOPUS_MAX_SAMPLES * 8];
    const uint8_t *d;
    size_t n;
    int64_t g, pos = 0;
    while (count < most && ahogg_packet(o, &d, &n, &g)) {
        opacket *k = &pk[count++];
        memset(k, 0, sizeof(*k));
        bytes_put(&k->data, d, n);
        int got = 0;
        ahopus_decode_packet(dec, d, n, out, &got);
        pos += got;
        k->end = pos;
    }
    ahopus_decoder_free(dec);
    ahogg_close(o);
    return count;
}

/* Seeks land on the sample. Opus decodes from 200 ms before the target
   (RFC 7845 asks for at least 80); the decoder has settled by then but not
   to the last bit,
   so the sound after a seek is held to `bound` (relative to the loudest)
   and not to an exact match. Seeks near the start decode from the start
   and must match exactly. */
static void check_opus_seeks(const char *what, const bytes *file, double bound)
{
    pcm whole = {0};
    ahreader_info info;
    const char *error = NULL;
    size_t frames = read_through(file, &whole, &info, &error);
    if (!frames) {
        CHECK(false, "%s: %s", what, error ? error : "no sound");
        return;
    }
    float peak = 1e-9f;
    for (size_t i = 0; i < whole.len; i++) peak = fabsf(whole.p[i]) > peak ? fabsf(whole.p[i]) : peak;
    ahreader *r = ahreader_open(test_memory_read, (void *)file, file->len, &error);
    static float buf[8 * 2000];
    double worst = 0;
    bool ok = r != NULL;
    for (int i = 0; i < 25 && ok; i++) {
        size_t at = i == 0 ? 0 : i == 1 ? frames - 1 : i == 2 ? 1000 : (size_t)rnd_in(0, (int)frames - 1);
        size_t want = frames - at < 2000 ? frames - at : 2000, got = 0;
        if (!ahreader_seek(r, at)) {
            ok = false;
            break;
        }
        while (got < want) {
            size_t n = ahreader_read(r, buf + got * (size_t)info.channels, want - got);
            if (!n) break;
            got += n;
        }
        if (got != want) {
            ok = false;
            printf("    %s: a seek to %zu of %zu gave %zu of %zu samples\n", what, at, frames, got, want);
            break;
        }
        double err = 0;
        for (size_t k = 0; k < want * (size_t)info.channels; k++) {
            double e = fabs((double)buf[k] - whole.p[at * (size_t)info.channels + k]) / peak;
            if (e > err) err = e;
        }
        /* On the sample: lined up one or two samples either way, it would
           differ by more. */
        if (want == 2000 && at >= 2 && at + 2002 < frames) {
            double sq[5] = {0};
            for (int shift = -2; shift <= 2; shift++)
                for (size_t k = 0; k < want * (size_t)info.channels; k++) {
                    double e = (double)buf[k] - whole.p[(at + (size_t)shift) * (size_t)info.channels + k];
                    sq[shift + 2] += e * e;
                }
            for (int shift = 0; shift < 5; shift++)
                if (shift != 2 && sq[shift] <= sq[2]) {
                    ok = false;
                    printf("    %s: a seek to %zu lines up better %d samples off\n", what, at, shift - 2);
                }
        }
        if (at < 9600 && err != 0) {
            ok = false;
            printf("    %s: a seek to %zu (decoded from the start) is not exact\n", what, at);
        }
        if (err > worst) worst = err;
    }
    CHECK(ok && worst <= bound, "%s: seeks went wrong (worst difference %.3g)", what, worst);
    if (worst > g_seek_worst) g_seek_worst = worst;
    if (r) ahreader_close(r);
    free(whole.p);
}

/* Files from libopus with their pages and headers redone: the pre-skip,
   a start cropped (a first page stating more than its packets hold), an
   end cut, and pages of every size. ffmpeg must agree, and seeks land. */
static void test_trims(void)
{
    char src[600], path[600], args[2000];
    path_of(src, "source.wav");
    path_of(path, "trims_base.opus");
    snprintf(args, sizeof(args), "-i \"%s\" -t 3 -c:a libopus -b:a 64k \"%s\"", src, path);
    if (!ffmpeg_run(args)) return;
    bytes orig = {0};
    read_file(path, &orig);
    static opacket pk[1000];
    int count = take_opus_packets(&orig, pk, 1000);
    CHECK(count > 10, "the libopus file could not be read");
    if (count <= 10) {
        free(orig.p);
        return;
    }
    int64_t total = pk[count - 1].end;
    static const struct {
        const char *name;
        int pre_skip, crop, cut, segs;
    } cases[] = {
        {"preskip_0", 0, 0, 0, 0},
        {"preskip_long", 5000, 0, 0, 0},
        {"cropped", 312, 48000, 0, 0},
        {"end_cut", 312, 0, 700, 0},
        {"small_pages", 312, 0, 333, 3},
        {"one_packet_pages", 312, 0, 0, 1},
    };
    /* A stream on one page that is also its last: a granule position below
       the samples it holds is the end, counted from 0; one above them is
       worked back from (RFC 7845 section 4.5). */
    for (int k = 0; k < 2; k++) {
        opacket few[12];
        memcpy(few, pk, sizeof(few));
        int64_t held = few[11].end, g = k ? held + 5000 : held - 700;
        few[11].end = g;
        bytes file = {0};
        ogg_write(&file, 78, few, 12, 255);
        pcm got = {0};
        ahreader_info info;
        const char *error = NULL;
        size_t frames = read_through(&file, &got, &info, &error);
        int64_t want = k ? held - 312 : g - 312;
        CHECK(frames == (size_t)want, "one-page stream stating %s: %zu samples, not %lld", k ? "more" : "less", frames, (long long)want);
        free(got.p);
        free(file.p);
    }
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        opacket again[1000];
        memcpy(again, pk, (size_t)count * sizeof(opacket));
        bytes head = {0};
        head_spec h = {2, cases[c].pre_skip, 0, 0, 1, 1, {0}};
        put_head(&head, &h);
        again[0].data = head;
        for (int i = 2; i < count; i++) again[i].end = pk[i].end + cases[c].crop;
        again[count - 1].end -= cases[c].cut;
        bytes file = {0};
        ogg_write(&file, 77, again, count, cases[c].segs);
        char what[64];
        snprintf(what, sizeof(what), "trim_%s", cases[c].name);
        pcm got = {0};
        ahreader_info info;
        const char *error = NULL;
        size_t frames = read_through(&file, &got, &info, &error);
        CHECK(frames == (size_t)(total - cases[c].cut - cases[c].pre_skip), "%s: %zu samples, not %lld", what, frames,
              (long long)(total - cases[c].cut - cases[c].pre_skip));
        CHECK(against_ffmpeg(what, &file, 1e-5, NULL), "%s: differs from ffmpeg", what);
        check_opus_seeks(what, &file, 0.02);
        free(got.p);
        free(file.p);
        free(head.p);
    }
    free_list(pk, count);
    free(orig.p);
}

/* ---- damage --------------------------------------------------------------------------- */

/* Damaged packets are covered over for the length they state; random bytes
   and broken framing must not upset the decoder. Headers it cannot use are
   refused with a reason. */
static void test_damage(void)
{
    ahopus_decoder *d = ahopus_decoder_new();
    static float out[AHOPUS_MAX_SAMPLES * 8];
    int n;
    CHECK(!ahopus_decode_packet(d, (const uint8_t *)"\x80\x01", 2, out, &n), "a packet before the header was taken");
    static const struct {
        const char *bytes;
        int len;
        const char *why;
    } heads[] = {
        {"OpusHead\x01\x00\x00\x00\x80\xBB\x00\x00\x00\x00\x00", 19, "no channels"},
        {"OpusHead\x10\x02\x00\x00\x80\xBB\x00\x00\x00\x00\x00", 19, "version 16"},
        {"OpusHead\x01\x03\x00\x00\x80\xBB\x00\x00\x00\x00\x00", 19, "three channels in family 0"},
        {"OpusHead\x01\x02\x00\x00\x80\xBB\x00\x00\x00\x00\x01\x01\x01\x00\x02", 23, "a stream that is not there"},
        {"OpusHead\x01\x02\x00\x00\x80\xBB\x00\x00\x00\x00\x01\x01\x02\x00\x01", 23, "more coupled streams than streams"},
        {"OpusHead\x01\x02\x00\x00\x80\xBB\x00\x00\x00\x00\x01\x01", 20, "a mapping table cut short"},
        {"OpusHead\x01\x09\x00\x00\x80\xBB\x00\x00\x00\x00\x01", 19, "nine channels"},
        {"OpusHead\x01\x02\x00\x00\x80\xBB\x00\x00\x00\x00\x02", 19, "family 2"},
        {"OpusHeax\x01\x02\x00\x00\x80\xBB\x00\x00\x00\x00\x00", 19, "a wrong name"},
    };
    for (size_t i = 0; i < sizeof(heads) / sizeof(heads[0]); i++) {
        bool taken = ahopus_decoder_header(d, (const uint8_t *)heads[i].bytes, (size_t)heads[i].len, NULL);
        CHECK(!taken && ahopus_decoder_error(d), "a header with %s was taken", heads[i].why);
    }

    head_spec h = {2, 0, 0, 0, 1, 1, {0}};
    bytes head = {0};
    put_head(&head, &h);
    CHECK(ahopus_decoder_header(d, head.p, head.len, NULL), "a good header was refused");
    /* Lost before any sound: silence, a frame long. */
    CHECK(ahopus_decode_packet(d, NULL, 0, out, &n) && n == 960 && out[0] == 0, "a loss before any packet gave %d samples", n);
    /* Broken framing: covered for the length the first byte states. */
    static const struct {
        const uint8_t bytes[8];
        int len, samples;
    } bad[] = {
        {{0x81, 0x01}, 2, 240},            /* code 1 with an odd length */
        {{0x82, 0xFF}, 2, 240},            /* code 2 whose first size runs past the end */
        {{0x83, 0x00}, 2, 0},              /* code 3 with no frames: no length can be told */
        {{0x83, 0x31}, 2, 0},              /* code 3 with 49 frames, over 120 ms */
        {{0x83, 0x42, 0xFF}, 3, 240},      /* padding past the end */
        {{0xFB, 0x03, 0x01, 0x02}, 4, 2880}, /* code 3 CBR not dividing evenly */
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        bool ok = ahopus_decode_packet(d, bad[i].bytes, (size_t)bad[i].len, out, &n);
        CHECK(!ok && n == bad[i].samples, "broken framing %zu: %s, %d samples", i, ok ? "taken" : "refused", n);
    }
    /* Random bytes as packets, and lost packets between: no harm, and the
       sound stays a number. */
    bool finite = true;
    unsigned char junk[1500];
    for (int i = 0; i < 3000; i++) {
        int len = rnd_in(0, 3) ? rnd_in(1, 100) : rnd_in(0, 1500);
        for (int k = 0; k < len; k++) junk[k] = (unsigned char)rnd();
        ahopus_decode_packet(d, rnd_in(0, 20) ? junk : NULL, (size_t)len, out, &n);
        for (int k = 0; k < 2 * n; k++) {
            uint32_t bits;
            memcpy(&bits, &out[k], 4);
            if ((bits & 0x7F800000u) == 0x7F800000u) finite = false;
        }
    }
    CHECK(finite, "random packets gave infinity or not-a-number");

    /* A lost packet is covered for the length of the one before, even when
       that is no single frame's length (three frames of 2.5 ms). */
    {
        static const uint8_t three[] = {0x83, 0x03, 0x10, 0x20, 0x30};
        ahopus_decode_packet(d, three, sizeof(three), out, &n);
        int good = n;
        ahopus_decode_packet(d, NULL, 0, out, &n);
        CHECK(good == 360 && n == 360, "a loss after a packet of 360 samples was covered with %d (the packet gave %d)", n, good);
    }

    /* Multistream: the last frame of a self-delimited stream may not run
       past the packet (2 streams; the first says frames of 10 and 8 bytes
       but 17 follow), nor may the streams differ in length. */
    {
        head_spec m = {2, 0, 0, 1, 2, 0, {0, 1}};
        bytes mh = {0};
        put_head(&mh, &m);
        CHECK(ahopus_decoder_header(d, mh.p, mh.len, NULL), "a two-stream header was refused");
        uint8_t over[20] = {0xFA, 0x0A, 0x08};
        for (int k = 3; k < 20; k++) over[k] = (uint8_t)rnd();
        bool ok = ahopus_decode_packet(d, over, sizeof(over), out, &n);
        CHECK(!ok && n == 1920, "a stream running past the packet: %s, %d samples", ok ? "taken" : "refused", n);
        static const uint8_t mixed[8] = {0x80, 0x02, 0x11, 0x22, 0x88, 0x33, 0x44, 0x55}; /* 2.5 ms, then 5 ms */
        ok = ahopus_decode_packet(d, mixed, sizeof(mixed), out, &n);
        CHECK(!ok, "streams of different lengths were taken");
        free(mh.p);
    }

    /* Random headers of up to 255 streams and random multistream packets,
       well formed and not: no harm, and no infinity or not-a-number. */
    finite = true;
    for (int round = 0; round < 300; round++) {
        uint8_t hb[21 + 8];
        int ch = rnd_in(1, 8), streams = rnd_in(1, rnd_in(0, 9) ? 6 : 255), coupled = rnd_in(0, streams);
        memcpy(hb, "OpusHead\x01", 9);
        hb[9] = (uint8_t)ch;
        memset(hb + 10, 0, 8);
        hb[18] = 1;
        hb[19] = (uint8_t)streams;
        hb[20] = (uint8_t)(streams + coupled > 255 ? 255 - streams : coupled);
        for (int c = 0; c < ch; c++) hb[21 + c] = (uint8_t)(rnd_in(0, 7) ? rnd_in(0, streams + hb[20] - 1) : 255);
        if (!ahopus_decoder_header(d, hb, 21 + (size_t)ch, NULL)) continue;
        for (int i = 0; i < 6; i++) {
            bytes pk = {0};
            int config = rnd_in(0, 31), count = rnd_in(1, 2);
            for (int st = 0; st < streams; st++) {
                if (rnd_in(0, 30) == 0) {
                    put_random(&pk, rnd_in(0, 5)); /* broken framing */
                    continue;
                }
                put_stream_packet(&pk, config, rnd_in(0, 1), count, st != streams - 1);
            }
            if (rnd_in(0, 5) == 0) pk.len = (size_t)rnd_in(0, (int)pk.len); /* cut short */
            ahopus_decode_packet(d, rnd_in(0, 15) ? pk.p : NULL, pk.len, out, &n);
            for (int k = 0; k < n * ch; k++) {
                uint32_t bits;
                memcpy(&bits, &out[k], 4);
                if ((bits & 0x7F800000u) == 0x7F800000u) finite = false;
            }
            free(pk.p);
        }
    }
    CHECK(finite, "random multistream packets gave infinity or not-a-number");
    ahopus_decoder_free(d);
    free(head.p);

    /* A stream with a damaged packet keeps its length: the packet is covered
       over, and the sound after it is where the pages say. */
    char src[600], path[600], args[2000];
    path_of(src, "source.wav");
    path_of(path, "damage_base.opus");
    snprintf(args, sizeof(args), "-i \"%s\" -t 2 -c:a libopus -b:a 64k \"%s\"", src, path);
    if (g_ffmpeg && ffmpeg_run(args)) {
        bytes orig = {0};
        read_file(path, &orig);
        static opacket pk[500];
        int count = take_opus_packets(&orig, pk, 500);
        pcm want = {0}, got = {0};
        ahreader_info info;
        const char *error = NULL;
        bytes plain = {0};
        ogg_write(&plain, 5, pk, count, 0);
        size_t frames = read_through(&plain, &want, &info, &error);
        free(plain.p);
        /* One packet cut to its first byte (no frame: covered over as lost),
           one a single frame longer than a frame may be (refused). */
        pk[20].data.len = 1;
        pk[30].data.p[0] &= 0xFC;
        while (pk[30].data.len < 1400) bytes_put(&pk[30].data, "\x55", 1);
        bytes file = {0};
        ogg_write(&file, 5, pk, count, 0);
        size_t got_frames = read_through(&file, &got, &info, &error);
        CHECK(got_frames == frames, "with damaged packets: %zu samples, not %zu", got_frames, frames);
        free_list(pk, count);
        free(want.p);
        free(got.p);
        free(file.p);
        free(orig.p);
    }
}

void test_opus(void)
{
    if (getenv("AHO_ROUNDS")) g_opus_rounds = atoi(getenv("AHO_ROUNDS"));
    test_damage();
    test_vectors();
    if (!g_ffmpeg) {
        printf("  ffmpeg not found: the Opus checks against it were skipped\n");
        return;
    }
    make_source();
    test_libopus_files();
    test_random_streams();
    test_trims();
    printf("  seeks: worst difference from the sound read through %.3g of full scale\n", g_seek_worst);
}
