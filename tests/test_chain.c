/* Chained Ogg: one stream after another, as internet radio sends a track at
   a time and as files joined end to end are. Each stream of a chain must
   play exactly as its file does alone, and that file as ffmpeg plays it.
   (ffmpeg's own reading of a chain is no reference: it stops where the
   sample rate or channels change, and plays the first block of each later
   Vorbis stream, which the specification says to drop; see the README.)
   Chains are read at random (ahreader_open) and in order, as radio comes
   (ahreader_open_stream), through a read function that hands out small,
   uneven pieces and checks that it is asked for bytes in order. */
#include "common.h"
#include "../include/ahogg.h"

#define LIBOPUS "-c:a libopus -request_sample_fmt flt"
#define MOST_STREAMS 8

typedef struct {
    const char *name, *args, *title;
} part_def;

/* One file of a chain, and its sound read alone. */
typedef struct {
    bytes file;
    pcm alone;
    size_t frames;
    ahreader_info info;
} part;

/* What a reader gave, cut where the stream it came from changes. */
typedef struct {
    pcm s[MOST_STREAMS];
    ahreader_info info[MOST_STREAMS];
    char title[MOST_STREAMS][32];
    int count;
} by_stream;

/* ---- reading in order --------------------------------------------------------------- */

/* A read function that gives a file in order, a few bytes at a time, from
   `from` on (a listener that joined part way). */
typedef struct {
    const bytes *file;
    size_t from;
    uint64_t next;   /* the offset it must be asked for next */
    bool out_of_order;
    int style;       /* 0: 1 to 37 bytes; 1: one byte; 2: up to 5000 */
} drip;

static size_t drip_read(void *user, uint64_t offset, void *buffer, size_t n)
{
    drip *d = user;
    if (offset != d->next) d->out_of_order = true;
    size_t left = d->file->len - d->from;
    if (offset >= left || !n) return 0;
    size_t k = d->style == 1 ? 1 : (size_t)rnd_in(1, d->style == 2 ? 5000 : 37);
    if (k > n) k = n;
    if (k > left - offset) k = (size_t)(left - offset);
    memcpy(buffer, d->file->p + d->from + offset, k);
    d->next = offset + k;
    return k;
}

static void free_streams(by_stream *o)
{
    for (int i = 0; i < MOST_STREAMS; i++) free(o->s[i].p);
    memset(o, 0, sizeof(*o));
}

/* The value of the TITLE comment, or "". */
static void title_of(const ahreader *r, char *out)
{
    const char *t;
    size_t n;
    out[0] = 0;
    for (int i = 0; ahreader_comment(r, i, &t, &n); i++)
        if (n > 6 && (t[0] == 't' || t[0] == 'T') && !memcmp(t + 1, (t[0] == 't' ? "itle=" : "ITLE="), 5)) {
            size_t len = n - 6 < 31 ? n - 6 : 31;
            memcpy(out, t + 6, len);
            out[len] = 0;
        }
}

/* Reads all a reader gives, `step` samples at a time at most. */
static void read_all(ahreader *r, by_stream *o, size_t step)
{
    static float buf[AHREADER_MAX_CHANNELS * 1000];
    memset(o, 0, sizeof(*o));
    if (step > 1000) step = 1000;
    size_t n;
    while ((n = ahreader_read(r, buf, step)) != 0) {
        const ahreader_info *in = ahreader_get_info(r);
        if (!o->count || in->stream != o->info[o->count - 1].stream) {
            if (o->count == MOST_STREAMS) break;
            o->info[o->count] = *in;
            o->s[o->count].channels = in->channels;
            o->s[o->count].rate = in->sample_rate;
            title_of(r, o->title[o->count]);
            o->count++;
        }
        pcm *s = &o->s[o->count - 1];
        if (in->channels != s->channels || in->sample_rate != s->rate) break; /* not one stream's */
        pcm_put(s, buf, n * (size_t)in->channels);
    }
}

static ahreader *open_drip(drip *d, const bytes *file, size_t from, int style, bool follow, const char **error)
{
    memset(d, 0, sizeof(*d));
    d->file = file;
    d->from = from;
    d->style = style;
    ahreader *r = ahreader_open_stream(drip_read, d, error);
    if (r) ahreader_follow_changes(r, follow);
    return r;
}

static ahreader *open_whole(const bytes *file, bool follow)
{
    ahreader *r = ahreader_open(test_memory_read, (void *)file, file->len, NULL);
    if (r) ahreader_follow_changes(r, follow);
    return r;
}

/* The streams read are the parts given (`which`, in order), each exactly as
   it plays alone, with its title. */
static bool same_parts(const char *what, const by_stream *o, part *parts, const int *which, int count)
{
    bool ok = o->count == count;
    for (int i = 0; ok && i < count; i++) {
        const part *p = &parts[which[i]];
        ok = same_pcm(&o->s[i], &p->alone) && o->info[i].sample_rate == p->info.sample_rate && o->info[i].channels == p->info.channels &&
             o->info[i].bits == p->info.bits && o->info[i].kind == p->info.kind && o->info[i].stream == (i ? o->info[i - 1].stream + 1 : o->info[0].stream);
        if (!ok)
            printf("    %s: stream %d gave %zu samples at %d Hz, %d channels; alone %zu at %d Hz, %d channels\n", what, i,
                   o->s[i].len / (size_t)(o->s[i].channels ? o->s[i].channels : 1), o->s[i].rate, o->s[i].channels, p->frames,
                   p->info.sample_rate, p->info.channels);
    }
    if (o->count != count) printf("    %s: %d streams, not %d\n", what, o->count, count);
    return ok;
}

static void join(bytes *out, const bytes *a, const bytes *b, const bytes *c)
{
    out->len = 0;
    bytes_put(out, a->p, a->len);
    if (b) bytes_put(out, b->p, b->len);
    if (c) bytes_put(out, c->p, c->len);
}

/* The pages of an Ogg file: where each starts, and how long it is. */
static int page_list(const bytes *f, size_t *at, size_t *len, int most)
{
    int n = 0;
    for (size_t off = 0; off + 27 <= f->len && n < most;) {
        const uint8_t *p = f->p + off;
        if (memcmp(p, "OggS", 4)) break;
        size_t size = 27 + p[26];
        if (off + size > f->len) break;
        for (int i = 0; i < p[26]; i++) size += p[27 + i];
        at[n] = off;
        len[n++] = size;
        off += size;
    }
    return n;
}

static int64_t page_granule(const uint8_t *p)
{
    uint64_t g = 0;
    for (int i = 7; i >= 0; i--) g = g << 8 | p[6 + i];
    return (int64_t)g;
}

static void redo_checksums(bytes *f)
{
    static size_t at[20000], len[20000];
    int n = page_list(f, at, len, 20000);
    for (int i = 0; i < n; i++) {
        uint8_t *p = f->p + at[i];
        memset(p + 22, 0, 4);
        uint32_t crc = ogg_crc(p, len[i]);
        for (int k = 0; k < 4; k++) p[22 + k] = (uint8_t)(crc >> (8 * k));
    }
}

/* ---- the parts ------------------------------------------------------------------------- */

static bool make_part(const part_def *d, part *out)
{
    char src[600], path[600], args[2000];
    memset(out, 0, sizeof(*out));
    path_of(src, "source.wav");
    path_of(path, d->name);
    snprintf(args, sizeof(args), "-i \"%s\" %s -metadata title=%s -fflags +bitexact \"%s\"", src, d->args, d->title, path);
    if (!ffmpeg_run(args) || !read_file(path, &out->file)) {
        printf("  %s: ffmpeg cannot make it, skipped\n", d->name);
        return false;
    }
    const char *error = NULL;
    out->frames = read_through(&out->file, &out->alone, &out->info, &error);
    CHECK(out->frames, "%s: %s", d->name, error ? error : "no sound");
    /* Alone, as ffmpeg plays it. */
    bytes raw = {0};
    bool opus = out->info.kind == AHREADER_OPUS;
    if (out->frames && ffmpeg_raw(path, opus ? LIBOPUS : NULL, "f32le", &raw)) {
        pcm ref = {(float *)raw.p, raw.len / sizeof(float), 0, out->info.channels, out->info.sample_rate};
        double worst = ref.len == out->alone.len ? max_diff(&ref, &out->alone, 0, ref.len) : 1;
        CHECK(worst < (out->info.bits ? 1e-7 : opus ? 1e-5 : 1e-4), "%s: differs from ffmpeg (%zu samples, ffmpeg %zu; %.3g)", d->name,
              out->alone.len, ref.len, worst);
    }
    free(raw.p);
    return out->frames != 0;
}

static void free_part(part *p)
{
    free(p->file.p);
    free(p->alone.p);
}

/* ---- chains read at random ----------------------------------------------------------- */

/* Seeks in a chain of the given parts land where they should: on the
   sample for Vorbis and FLAC; for Opus, whose decoder settles only after a
   seek, close to it (as in test_opus.c). */
static void check_chain_seeks(const char *what, const bytes *chain, part *parts, const int *which, int count, bool follow)
{
    ahreader *r = open_whole(chain, follow);
    if (!r) {
        CHECK(false, "%s: cannot be opened", what);
        return;
    }
    uint64_t total = 0, starts[MOST_STREAMS + 1];
    for (int i = 0; i < count; i++) {
        starts[i] = total;
        total += parts[which[i]].frames;
    }
    starts[count] = total;
    CHECK(ahreader_get_info(r)->samples == total && ahreader_get_info(r)->samples_exact, "%s: says %llu samples, not %llu", what,
          (unsigned long long)ahreader_get_info(r)->samples, (unsigned long long)total);
    bool opus = parts[which[0]].info.kind == AHREADER_OPUS;
    static float buf[AHREADER_MAX_CHANNELS * 600];
    bool ok = true;
    double worst = 0;
    for (int t = 0; t < 40 && ok; t++) {
        /* The first and last sample of each stream, and anywhere. */
        int k = t % count;
        uint64_t at = t < 2 * count ? (t < count ? starts[k] : starts[k + 1] - 1) : (uint64_t)rnd() % total;
        while (at >= starts[k + 1]) k++;
        while (at < starts[k]) k--;
        int shown = ahreader_get_info(r)->stream;
        if (!ahreader_seek(r, at)) {
            printf("    %s: a seek to %llu failed\n", what, (unsigned long long)at);
            ok = false;
            break;
        }
        /* The info changes with the first read, not with the seek. */
        ok = ahreader_get_info(r)->stream == shown;
        const part *p = &parts[which[k]];
        size_t in = (size_t)(at - starts[k]), want = p->frames - in < 600 ? p->frames - in : 600;
        size_t got = ahreader_read(r, buf, 600);
        const ahreader_info *info = ahreader_get_info(r);
        ok = ok && got == want && info->stream == k && info->channels == p->info.channels && info->sample_rate == p->info.sample_rate;
        if (ok) {
            pcm a = {buf, got * (size_t)info->channels, 0, info->channels, info->sample_rate};
            pcm b = {p->alone.p + in * (size_t)info->channels, got * (size_t)info->channels, 0, info->channels, info->sample_rate};
            double d = opus ? max_diff(&a, &b, 0, a.len) : (memcmp(a.p, b.p, a.len * sizeof(float)) ? 1 : 0);
            if (d > worst) worst = d;
            ok = d <= (opus ? 0.02 : 0);
        }
        if (!ok)
            printf("    %s: a seek to %llu (stream %d, %zu in) gave %zu of %zu samples from stream %d\n", what, (unsigned long long)at, k, in,
                   got, want, info->stream);
    }
    CHECK(ok, "%s: a seek went wrong (worst difference %.3g)", what, worst);
    /* Past the end: false, and nothing more is read, not even the next
       stream's. */
    CHECK(!ahreader_seek(r, total) && ahreader_read(r, buf, 600) == 0, "%s: a seek past the end did not end the sound", what);
    CHECK(ahreader_seek(r, 0) && ahreader_read(r, buf, 600) == 600, "%s: no sound after seeking back to the start", what);
    ahreader_close(r);
}

/* The streams of a chain are the parts in order: read at random and in
   order, and a stream of another shape ending the sound unless changes are
   followed. */
static void test_chain_of(const char *kind, part *parts)
{
    char what[128];
    bytes chain = {0};
    by_stream o;
    static const int abc[] = {0, 1, 2}, ac[] = {0, 2}, a[] = {0};

    /* A, B, C: B has another rate (or, for Opus, channels). */
    join(&chain, &parts[0].file, &parts[1].file, &parts[2].file);
    ahreader *r = open_whole(&chain, false);
    CHECK(r && ahreader_get_info(r)->samples == parts[0].frames, "%s chain: not following changes, the length is not the first stream's", kind);
    if (r) {
        read_all(r, &o, 1000);
        snprintf(what, sizeof(what), "%s chain, not following changes", kind);
        CHECK(same_parts(what, &o, parts, a, 1), "%s: the sound did not end where the shape changed", what);
        free_streams(&o);
        ahreader_close(r);
    }
    r = open_whole(&chain, true);
    if (r) {
        read_all(r, &o, (size_t)rnd_in(1, 1000));
        snprintf(what, sizeof(what), "%s chain", kind);
        CHECK(same_parts(what, &o, parts, abc, 3), "%s: the streams did not play as they do alone", what);
        CHECK(o.count == 3 && !strcmp(o.title[0], "One") && !strcmp(o.title[1], "Two") && !strcmp(o.title[2], "Three"),
              "%s: the titles were %s, %s, %s", what, o.title[0], o.title[1], o.title[2]);
        free_streams(&o);
        ahreader_close(r);
    }
    snprintf(what, sizeof(what), "%s chain seeks", kind);
    check_chain_seeks(what, &chain, parts, abc, 3, true);

    /* In order, through small pieces. */
    for (int style = 0; style < 3; style++) {
        drip d;
        const char *error = NULL;
        r = open_drip(&d, &chain, 0, style, true, &error);
        snprintf(what, sizeof(what), "%s chain in order (pieces %d)", kind, style);
        CHECK(r, "%s: %s", what, error ? error : "not opened");
        if (!r) continue;
        CHECK(d.next < parts[0].file.len / 2 && d.next < 70000, "%s: %llu bytes read before the headers were in", what, (unsigned long long)d.next);
        CHECK(ahreader_get_info(r)->samples == 0 && !ahreader_get_info(r)->samples_exact, "%s: a length was given", what);
        static float first[AHREADER_MAX_CHANNELS * 64];
        size_t n = ahreader_read(r, first, 64);
        CHECK(n == 64 && d.next < parts[0].file.len / 2, "%s: %llu bytes read before the first samples", what, (unsigned long long)d.next);
        CHECK(!ahreader_seek(r, 0), "%s: a seek did not fail", what);
        read_all(r, &o, (size_t)rnd_in(1, 1000));
        /* The first 64 samples were read before. */
        if (o.count && n == 64) {
            pcm whole = {0};
            whole.channels = o.s[0].channels;
            pcm_put(&whole, first, 64 * (size_t)whole.channels);
            pcm_put(&whole, o.s[0].p, o.s[0].len);
            free(o.s[0].p);
            o.s[0] = whole;
            o.info[0].stream = 0;
        }
        CHECK(same_parts(what, &o, parts, abc, 3) && !d.out_of_order, "%s: the streams did not play as they do alone%s", what,
              d.out_of_order ? " (bytes were asked for out of order)" : "");
        free_streams(&o);
        ahreader_close(r);
    }
    /* In order, not following: the sound ends at B. */
    {
        drip d;
        r = open_drip(&d, &chain, 0, 2, false, NULL);
        if (r) {
            read_all(r, &o, 1000);
            snprintf(what, sizeof(what), "%s chain in order, not following changes", kind);
            CHECK(same_parts(what, &o, parts, a, 1), "%s: the sound did not end where the shape changed", what);
            free_streams(&o);
            ahreader_close(r);
        }
    }

    /* A cut off without its last page's mark (a radio source that stopped):
       B's first page ends it. Read at random, the walk over the chain still
       finds where its sound ends; read in order nothing says so, and its
       last packet plays whole. */
    {
        bytes cut = {0};
        join(&cut, &parts[0].file, &parts[1].file, &parts[2].file);
        static size_t at[20000], len[20000];
        int n = page_list(&parts[0].file, at, len, 20000);
        cut.p[at[n - 1] + 5] &= (uint8_t)~4;
        redo_checksums(&cut);
        snprintf(what, sizeof(what), "%s chain with A unmarked at its end", kind);
        r = open_whole(&cut, true);
        if (r) {
            read_all(r, &o, 1000);
            CHECK(same_parts(what, &o, parts, abc, 3), "%s: the streams did not play as they do alone", what);
            free_streams(&o);
            ahreader_close(r);
        }
        drip d;
        r = open_drip(&d, &cut, 0, 0, true, NULL);
        if (r) {
            read_all(r, &o, 1000);
            const pcm *a0 = &parts[0].alone;
            bool ok = o.count == 3 && same_pcm(&o.s[1], &parts[1].alone) && same_pcm(&o.s[2], &parts[2].alone) && o.s[0].len >= a0->len &&
                      o.s[0].len <= a0->len + 8192 * (size_t)a0->channels && !memcmp(o.s[0].p, a0->p, a0->len * sizeof(float));
            CHECK(ok, "%s, in order: did not play A whole, then B and C", what);
            free_streams(&o);
            ahreader_close(r);
        }
        free(cut.p);
    }

    /* A and C have the same shape: they follow on without asking. */
    join(&chain, &parts[0].file, &parts[2].file, NULL);
    r = open_whole(&chain, false);
    if (r) {
        CHECK(ahreader_get_info(r)->samples == parts[0].frames + parts[2].frames, "%s A+C: the length is not both streams'", kind);
        read_all(r, &o, 1000);
        snprintf(what, sizeof(what), "%s A+C", kind);
        CHECK(same_parts(what, &o, parts, ac, 2), "%s: the streams did not play as they do alone", what);
        free_streams(&o);
        ahreader_close(r);
    }
    snprintf(what, sizeof(what), "%s A+C seeks", kind);
    check_chain_seeks(what, &chain, parts, ac, 2, true);
    snprintf(what, sizeof(what), "%s A+C seeks, not following changes", kind);
    check_chain_seeks(what, &chain, parts, ac, 2, false);

    /* Joined part way, as a radio listener joins: in the middle of A's
       pages, or one byte in. The stream whose start was missed is passed
       over; B and C play. Joined in C, nothing starts. */
    join(&chain, &parts[0].file, &parts[1].file, &parts[2].file);
    static const int bc[] = {1, 2};
    size_t froms[3] = {1, parts[0].file.len / 2 + 7, (size_t)rnd_in(28, (int)parts[0].file.len - 1)};
    for (int j = 0; j < 3; j++) {
        drip d;
        const char *error = NULL;
        r = open_drip(&d, &chain, froms[j], j, true, &error);
        snprintf(what, sizeof(what), "%s chain joined %zu bytes in", kind, froms[j]);
        CHECK(r, "%s: %s", what, error ? error : "not opened");
        if (!r) continue;
        read_all(r, &o, 1000);
        CHECK(same_parts(what, &o, parts, bc, 2) && o.info[0].stream == 0, "%s: did not play B and C", what);
        free_streams(&o);
        ahreader_close(r);
    }
    {
        drip d;
        const char *error = NULL;
        r = open_drip(&d, &chain, parts[0].file.len + parts[1].file.len + 100, 2, true, &error);
        CHECK(!r && error, "%s chain joined in its last stream: opened", kind);
        ahreader_close(r);
    }
    /* Junk before the first page. */
    {
        bytes junk = {0};
        for (int i = 0; i < 5000; i++) {
            uint8_t c = (uint8_t)rnd();
            bytes_put(&junk, &c, 1);
        }
        bytes_put(&junk, chain.p, chain.len);
        drip d;
        r = open_drip(&d, &junk, 0, 2, true, NULL);
        snprintf(what, sizeof(what), "%s chain after junk", kind);
        CHECK(r, "%s: not opened", what);
        if (r) {
            read_all(r, &o, 1000);
            CHECK(same_parts(what, &o, parts, abc, 3), "%s: the streams did not play as they do alone", what);
            free_streams(&o);
            ahreader_close(r);
        }
        free(junk.p);
    }
    free(chain.p);
}

/* As Icecast serves a listener who joins: the stream's header pages, then
   its pages from where the broadcast has got to, then on. The sound from
   there is the end of the stream's sound alone (exactly, for Vorbis and
   FLAC; Opus's decoder settles within 200 ms). */
static void test_icecast_join(const char *kind, part *parts)
{
    static size_t at[20000], len[20000];
    const bytes *a = &parts[0].file;
    int n = page_list(a, at, len, 20000), headers = 0;
    while (headers < n && page_granule(a->p + at[headers]) == 0) headers++;
    bytes s = {0};
    for (int i = 0; i < headers; i++) bytes_put(&s, a->p + at[i], len[i]);
    int from = headers + (n - headers) / 2;
    /* Where the sound joined lies: after the last page before it that
       states a place; it ends where the stream does. */
    int64_t joined = -1, end = -1;
    for (int i = headers; i < n; i++) {
        int64_t g = page_granule(a->p + at[i]);
        if (g < 0) continue;
        if (i < from) joined = g;
        end = g;
    }
    bytes_put(&s, a->p + at[from], a->len - at[from]);
    bytes_put(&s, parts[1].file.p, parts[1].file.len);
    drip d;
    by_stream o;
    char what[128];
    snprintf(what, sizeof(what), "%s joined as Icecast serves it", kind);
    ahreader *r = open_drip(&d, &s, 0, 0, true, NULL);
    CHECK(r, "%s: not opened", what);
    if (r) {
        read_all(r, &o, 1000);
        bool ok = o.count == 2 && same_pcm(&o.s[1], &parts[1].alone);
        const pcm *alone = &parts[0].alone;
        if (ok) {
            size_t got = o.s[0].len, ch = (size_t)alone->channels;
            ok = got > 0 && got < alone->len;
            /* The first packet joined gives no sound (Vorbis), or a
               packet may be cut off at the join, and Opus drops its
               pre-skip: no more than about a packet goes. */
            int64_t frames = (int64_t)(got / ch), most = end - joined;
            if (ok && (frames > most || frames < most - 9600)) {
                printf("    %s: %lld samples from the join, not about %lld\n", what, (long long)frames, (long long)most);
                ok = false;
            }
            if (ok && parts[0].info.kind == AHREADER_OPUS) {
                size_t skip = 9600 * ch < got ? 9600 * ch : got;
                pcm x = {o.s[0].p + skip, got - skip, 0, alone->channels, 48000};
                pcm y = {alone->p + alone->len - got + skip, got - skip, 0, alone->channels, 48000};
                ok = max_diff(&x, &y, 0, x.len) < 0.02;
            } else if (ok) {
                ok = !memcmp(o.s[0].p, alone->p + alone->len - got, got * sizeof(float));
            }
        }
        CHECK(ok, "%s: the sound is not the end of the stream's", what);
        free_streams(&o);
        ahreader_close(r);
    }
    free(s.p);
}

/* A last page whose position cuts off more than its last packet (all but
   a little of what the page holds): read in order, the cut must be the
   same as read at random. Alone, and before another stream. */
static void test_long_end_cut(const char *kind, part *parts)
{
    static size_t at[20000], len[20000];
    bytes cut = {0}, chain = {0};
    bytes_put(&cut, parts[0].file.p, parts[0].file.len);
    int n = page_list(&cut, at, len, 20000);
    int64_t last = page_granule(cut.p + at[n - 1]), before = -1;
    for (int i = n - 2; i >= 0 && before < 0; i--) before = page_granule(cut.p + at[i]);
    int64_t g = before + (last - before) / 8 + 1;
    for (int i = 0; i < 8; i++) cut.p[at[n - 1] + 6 + (size_t)i] = (uint8_t)((uint64_t)g >> (8 * i));
    redo_checksums(&cut);
    join(&chain, &cut, &parts[1].file, NULL);
    const bytes *files[2] = {&cut, &chain};
    for (int f = 0; f < 2; f++) {
        by_stream a, b;
        ahreader *r = open_whole(files[f], true);
        drip d;
        ahreader *s = open_drip(&d, files[f], 0, 0, true, NULL);
        bool ok = r && s;
        if (ok) {
            read_all(r, &a, 1000);
            read_all(s, &b, 1000);
            ok = a.count == f + 1 && b.count == f + 1 && a.s[0].len + (size_t)(last - g) * (size_t)a.s[0].channels == parts[0].alone.len;
            for (int i = 0; ok && i <= f; i++) ok = same_pcm(&a.s[i], &b.s[i]);
            free_streams(&a);
            free_streams(&b);
        }
        CHECK(ok, "%s: a cut of %lld samples at the end, %s, differs read in order", kind, (long long)(last - g), f ? "before another stream" : "alone");
        ahreader_close(r);
        ahreader_close(s);
    }
    free(cut.p);
    free(chain.p);
}

/* Changes followed only once the sound has ended at one: it goes on. And
   a seek into a stream not followed fails until changes are. */
static void test_follow_later(const char *kind, part *parts)
{
    bytes chain = {0};
    join(&chain, &parts[0].file, &parts[1].file, &parts[2].file);
    static const int bc[] = {1, 2};
    static float buf[AHREADER_MAX_CHANNELS * 1000];
    for (int mode = 0; mode < 2; mode++) {
        drip d;
        ahreader *r = mode ? open_drip(&d, &chain, 0, 0, false, NULL) : open_whole(&chain, false);
        if (!r) {
            CHECK(false, "%s: not opened", kind);
            continue;
        }
        size_t got = 0, n;
        while ((n = ahreader_read(r, buf, 1000)) != 0) got += n;
        ahreader_follow_changes(r, true);
        by_stream o;
        read_all(r, &o, 1000);
        char what[128];
        snprintf(what, sizeof(what), "%s, following changes once the sound ended (%s)", kind, mode ? "in order" : "at random");
        CHECK(got == parts[0].frames && same_parts(what, &o, parts, bc, 2), "%s: did not go on", what);
        free_streams(&o);
        ahreader_close(r);
    }
    ahreader *r = open_whole(&chain, false);
    if (r) {
        uint64_t into_b = parts[0].frames + 10;
        bool refused = !ahreader_seek(r, into_b) && ahreader_read(r, buf, 1000) == 0;
        ahreader_follow_changes(r, true);
        bool done = ahreader_seek(r, into_b) && ahreader_read(r, buf, 100) == 100 && ahreader_get_info(r)->stream == 1;
        /* Opus seeks come near, not to the bit. */
        if (done && parts[1].info.kind != AHREADER_OPUS)
            done = !memcmp(buf, parts[1].alone.p + 10 * (size_t)parts[1].info.channels, 100 * (size_t)parts[1].info.channels * sizeof(float));
        CHECK(refused && done, "%s: a seek into the second stream, not followed then followed, went wrong", kind);
        ahreader_close(r);
    }
    free(chain.p);
}

/* ---- the Ogg reader --------------------------------------------------------------------- */

/* Read at random and in order, ahogg hands over the same headers and
   packets, stream after stream, and can go to any stream. */
static void test_ogg_level(const char *kind, part *parts)
{
    bytes chain = {0};
    join(&chain, &parts[0].file, &parts[1].file, &parts[2].file);
    const char *error = NULL;
    ahogg *o = ahogg_open(test_memory_read, &chain, chain.len, &error);
    drip d = {&chain, 0, 0, false, 0};
    ahogg *s = ahogg_open_stream(drip_read, &d, &error);
    bool ok = o && s && ahogg_get_info(o)->streams == 3 && ahogg_get_info(s)->streams == 0 && ahogg_get_info(s)->last_granule == -1;
    int streams = 0;
    long packets = 0;
    while (ok) {
        const ahogg_info *a = ahogg_get_info(o), *b = ahogg_get_info(s);
        ok = a->stream == streams && b->stream == streams && a->codec == b->codec && a->serial == b->serial && a->headers == b->headers;
        for (int i = 0; ok && i < 3; i++) {
            const uint8_t *x, *y;
            size_t nx, ny;
            bool hx = ahogg_header(o, i, &x, &nx), hy = ahogg_header(s, i, &y, &ny);
            ok = hx == hy && (!hx || (nx == ny && !memcmp(x, y, nx)));
        }
        const uint8_t *x, *y;
        size_t nx, ny;
        int64_t gx, gy;
        for (;;) {
            bool px = ahogg_packet(o, &x, &nx, &gx), py = ahogg_packet(s, &y, &ny, &gy);
            if (px != py || (px && (nx != ny || gx != gy || memcmp(x, y, nx) || ahogg_last_page(o) != ahogg_last_page(s)))) ok = false;
            if (!px || !ok) break;
            packets++;
        }
        streams++;
        bool nx_ = ahogg_next_stream(o), ny_ = ahogg_next_stream(s);
        if (nx_ != ny_) ok = false;
        if (!nx_) break;
    }
    CHECK(ok && streams == 3 && packets > 100, "%s: ahogg read in order differs from read at random (stream %d, %ld packets)", kind, streams,
          packets);
    CHECK(!d.out_of_order, "%s: ahogg asked for bytes out of order", kind);
    if (s) CHECK(!ahogg_seek(s, 0) && !ahogg_select_stream(s, 0), "%s: a stream read in order seeked", kind);
    /* Any stream at random: its headers are the part's own. */
    for (int k = 2; o && k >= 0; k--) {
        ahogg *alone = ahogg_open(test_memory_read, &parts[k].file, parts[k].file.len, NULL);
        bool same = alone && ahogg_select_stream(o, k) && ahogg_get_info(o)->stream == k &&
                    ahogg_get_info(o)->last_granule == ahogg_get_info(alone)->last_granule;
        for (int i = 0; same && i < 3; i++) {
            const uint8_t *x, *y;
            size_t nx, ny;
            bool hx = ahogg_header(o, i, &x, &nx), hy = ahogg_header(alone, i, &y, &ny);
            same = hx == hy && (!hx || (nx == ny && !memcmp(x, y, nx)));
        }
        const uint8_t *x, *y;
        size_t nx, ny;
        int64_t gx, gy;
        same = same && ahogg_packet(o, &x, &nx, &gx) && ahogg_packet(alone, &y, &ny, &gy) && nx == ny && !memcmp(x, y, nx);
        CHECK(same, "%s: stream %d chosen at random is not the part's", kind, k);
        ahogg_close(alone);
    }
    if (o) CHECK(!ahogg_select_stream(o, 3) && ahogg_get_info(o)->stream == 0, "%s: a stream past the last was chosen", kind);
    ahogg_close(o);
    ahogg_close(s);
    free(chain.p);
}

/* ---- odd chains --------------------------------------------------------------------- */

/* A file joined to itself: both copies have the same serial number, which
   RFC 3533 forbids in a chain. Read at random, the streams are told apart
   by number, so the file plays once; read in order, a stream's first page
   after others starts a new one, so it plays twice. */
static void test_same_serial(const char *kind, part *parts)
{
    bytes twice = {0};
    join(&twice, &parts[0].file, &parts[0].file, NULL);
    by_stream o;
    char what[128];
    static const int aa[] = {0, 0}, a[] = {0};
    ahreader *r = open_whole(&twice, true);
    if (r) {
        read_all(r, &o, 1000);
        snprintf(what, sizeof(what), "%s joined to itself, read at random", kind);
        CHECK(same_parts(what, &o, parts, a, 1), "%s: did not play once", what);
        free_streams(&o);
        ahreader_close(r);
    }
    drip d;
    r = open_drip(&d, &twice, 0, 0, true, NULL);
    if (r) {
        read_all(r, &o, 1000);
        snprintf(what, sizeof(what), "%s joined to itself, read in order", kind);
        CHECK(same_parts(what, &o, parts, aa, 2), "%s: did not play twice", what);
        free_streams(&o);
        ahreader_close(r);
    }
    free(twice.p);
}

/* A stream of another kind between two: it is passed over. */
static void test_other_kind_between(part *vorbis, part *opus)
{
    bytes chain = {0};
    join(&chain, &vorbis[0].file, &opus[0].file, &vorbis[2].file);
    by_stream o;
    static const int ac[] = {0, 2};
    ahreader *r = open_whole(&chain, false);
    if (r) {
        read_all(r, &o, 1000);
        CHECK(same_parts("Vorbis, Opus, Vorbis", &o, vorbis, ac, 2) && o.info[1].stream == 1, "Vorbis, Opus, Vorbis: the Opus stream was not passed over");
        free_streams(&o);
        ahreader_close(r);
    }
    drip d;
    r = open_drip(&d, &chain, 0, 0, false, NULL);
    if (r) {
        read_all(r, &o, 1000);
        CHECK(same_parts("Vorbis, Opus, Vorbis in order", &o, vorbis, ac, 2), "Vorbis, Opus, Vorbis in order: the Opus stream was not passed over");
        free_streams(&o);
        ahreader_close(r);
    }
    free(chain.p);
}

/* Damage, with and without the checksums made good again: no crash, no
   runaway sound, read at random or in order. */
static void test_chain_damage(const char *kind, part *parts)
{
    bytes chain = {0}, broken = {0};
    join(&chain, &parts[0].file, &parts[1].file, &parts[2].file);
    size_t total = 0, worst = 0;
    for (int i = 0; i < 3; i++) total += parts[i].alone.len;
    static float buf[AHREADER_MAX_CHANNELS * 1000];
    bool ended = true;
    for (int round = 0; round < 40; round++) {
        broken.len = 0;
        bytes_put(&broken, chain.p, chain.len);
        int hits = rnd_in(1, 60);
        for (int i = 0; i < hits; i++) broken.p[rnd() % broken.len] ^= (uint8_t)rnd_in(1, 255);
        if (round % 2) redo_checksums(&broken);
        size_t got = 0, n;
        ahreader *r = round % 4 < 2 ? open_whole(&broken, true) : NULL;
        drip d;
        if (!r) r = open_drip(&d, &broken, (size_t)(rnd() % 64), 0, true, NULL);
        while (r && (n = ahreader_read(r, buf, 1000)) != 0 && got < 64 * total) got += n * (size_t)ahreader_get_info(r)->channels;
        ended &= got < 64 * total;
        if (r) {
            /* And seeks, read at random. */
            for (int t = 0; t < 5; t++) {
                ahreader_seek(r, (uint64_t)rnd() % (total + 1));
                ahreader_read(r, buf, 1000);
            }
        }
        ahreader_close(r);
        if (got > worst) worst = got;
    }
    /* Damaged headers can make blocks longer, so only a runaway counts. */
    CHECK(ended, "%s: a damaged chain gave %zu samples and went on, the whole one %zu", kind, worst, total);
    free(broken.p);
    free(chain.p);
}

void test_chain(void)
{
    if (!g_ffmpeg) return;
    make_source();
    static const part_def defs[3][3] = {
        {{"chain_v1.ogg", "-t 2.5 -ar 44100 -c:a libvorbis -q:a 3 -serial_offset 101", "One"},
         {"chain_v2.ogg", "-ss 1 -t 2 -ac 1 -c:a libvorbis -q:a 3 -serial_offset 202", "Two"},
         {"chain_v3.ogg", "-ss 2 -t 2.2 -ar 44100 -c:a libvorbis -q:a 6 -serial_offset 303", "Three"}},
        {{"chain_o1.opus", "-t 2.5 -c:a libopus -b:a 64k -serial_offset 104", "One"},
         {"chain_o2.opus", "-ss 1 -t 2 -ac 1 -c:a libopus -b:a 32k -serial_offset 205", "Two"},
         {"chain_o3.opus", "-ss 2 -t 2.2 -c:a libopus -b:a 96k -serial_offset 306", "Three"}},
        {{"chain_f1.oga", "-t 2.5 -ar 44100 -c:a flac -serial_offset 107", "One"},
         {"chain_f2.oga", "-ss 1 -t 2 -ac 1 -sample_fmt s32 -c:a flac -serial_offset 208", "Two"},
         {"chain_f3.oga", "-ss 2 -t 2.2 -ar 44100 -c:a flac -serial_offset 309", "Three"}},
    };
    static const char *kinds[3] = {"Vorbis", "Opus", "FLAC"};
    static part parts[3][3];
    bool made[3];
    for (int k = 0; k < 3; k++) {
        made[k] = true;
        for (int i = 0; i < 3; i++) made[k] &= make_part(&defs[k][i], &parts[k][i]);
    }
    for (int k = 0; k < 3; k++) {
        if (!made[k]) continue;
        printf("  %s\n", kinds[k]);
        test_chain_of(kinds[k], parts[k]);
        test_icecast_join(kinds[k], parts[k]);
        test_long_end_cut(kinds[k], parts[k]);
        test_follow_later(kinds[k], parts[k]);
        test_ogg_level(kinds[k], parts[k]);
        test_same_serial(kinds[k], parts[k]);
        test_chain_damage(kinds[k], parts[k]);
    }
    if (made[0] && made[1]) test_other_kind_between(parts[0], parts[1]);
    for (int k = 0; k < 3; k++)
        for (int i = 0; i < 3; i++) free_part(&parts[k][i]);
}
