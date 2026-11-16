/* Tests for the decoder and the encoder.
     ahmp3_tests <scratch folder> [--no-ffmpeg]
   Without ffmpeg on the PATH the checks against it are skipped. ffmpeg is
   used only here, as an independent reference decoder and as a source of
   files from other encoders. */
#include "../src/ahmp3_bitstream.h"
#include "common.h"

int g_checks, g_failures;
bool g_ffmpeg;
static char g_dir[512];
enum { F_SWITCH = 1, F_MIXED = 2, F_MS = 4, F_IS = 8, F_SCFSI = 16, F_PREFLAG = 32, F_SUBGAIN = 64, F_SFSCALE = 128, F_TABLES = 256, F_DUAL = 512, F_SF = 1024, F_ALL = 2047 };
static int g_features = F_ALL;
static int g_force_type = -1;
static int g_crc;        /* test frames carry a CRC */
static int g_free_bytes; /* test frames are free format, of this size */
static pcm *g_keep;      /* test_random_frames leaves its decoded sound here */

/* ---- helpers --------------------------------------------------------------- */

unsigned g_seed = 12345;
unsigned rnd(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return (g_seed >> 8) & 0xFFFFFF;
}
int rnd_in(int lo, int hi) { return lo + (int)(rnd() % (unsigned)(hi - lo + 1)); }

void bytes_put(bytes *b, const void *d, size_t n)
{
    if (b->len + n > b->cap) {
        b->cap = (b->len + n) * 2;
        b->p = realloc(b->p, b->cap);
    }
    if (n) memcpy(b->p + b->len, d, n);
    b->len += n;
}

void pcm_put(pcm *s, const float *d, size_t n)
{
    if (s->len + n > s->cap) {
        s->cap = (s->len + n) * 2;
        s->p = realloc(s->p, s->cap * sizeof(float));
    }
    if (n) memcpy(s->p + s->len, d, n * sizeof(float));
    s->len += n;
}

void path_of(char *out, const char *name)
{
    snprintf(out, 600, "%s/%s", g_dir, name);
}

bool write_file(const char *path, const void *d, size_t n)
{
    FILE *f = fopen(path, "wb");
    if (!f) return false;
    bool ok = fwrite(d, 1, n, f) == n;
    fclose(f);
    return ok;
}

bool read_file(const char *path, bytes *out)
{
    FILE *f = fopen(path, "rb");
    if (!f) return false;
    unsigned char buf[65536];
    size_t n;
    out->len = 0;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) bytes_put(out, buf, n);
    fclose(f);
    return true;
}

bool ffmpeg_run(const char *args)
{
    char cmd[4000];
    snprintf(cmd, sizeof(cmd), "ffmpeg -v error -nostdin -y %s", args);
    return system(cmd) == 0;
}

bool ffmpeg_raw(const char *in_path, const char *extra, const char *format, bytes *out)
{
    char raw[700], args[2000];
    snprintf(raw, sizeof(raw), "%s.%s", in_path, format);
    snprintf(args, sizeof(args), "%s -i \"%s\" -f %s \"%s\"", extra ? extra : "", in_path, format, raw);
    out->len = 0;
    return ffmpeg_run(args) && read_file(raw, out);
}

static void decode_all(const unsigned char *d, size_t len, pcm *out, ahmp3_info *info)
{
    ahmp3_decoder *dec = ahmp3_decoder_new();
    size_t pos = 0;
    out->len = 0;
    out->channels = 0;
    for (;;) {
        float buf[AHMP3_MAX_SAMPLES * 2];
        int n;
        ahmp3_frame fr;
        size_t used = ahmp3_decode(dec, d + pos, len - pos, true, buf, &n, &fr);
        if (!used) break;
        pos += used;
        if (!n) continue;
        out->channels = fr.channels;
        out->rate = fr.sample_rate;
        pcm_put(out, buf, (size_t)n * fr.channels);
    }
    if (info) *info = *ahmp3_decoder_info(dec);
    ahmp3_decoder_free(dec);
}

/* ffmpeg's own floating point decoder, as raw interleaved floats. */
static bool ffmpeg_decode(const char *mp3_path, pcm *out)
{
    bytes b = {0};
    if (!ffmpeg_raw(mp3_path, "-c:a mp3float", "f32le", &b)) return false;
    out->len = 0;
    pcm_put(out, (const float *)b.p, b.len / 4);
    free(b.p);
    return true;
}

double max_diff(const pcm *a, const pcm *b, size_t from, size_t to)
{
    double diff = 0, peak = 1e-9;
    for (size_t i = from; i < to && i < a->len && i < b->len; i++) {
        double d = fabs((double)a->p[i] - b->p[i]);
        if (d > diff) diff = d;
        if (fabs(a->p[i]) > peak) peak = fabs(a->p[i]);
    }
    return diff / (peak > 1e-3 ? peak : 1e-3);
}

/* ---- Huffman tables -------------------------------------------------------- */

static void test_tables(void)
{
    /* Every table must be a complete prefix code: one code per value, none
       the start of another, and no unused bit patterns. */
    for (int t = 1; t < 34; t++) {
        const unsigned short *codes16 = NULL;
        const unsigned char *codes8 = NULL, *lens;
        int count;
        if (t < 32) {
            const ahmp3_huff *h = &ahmp3_huff_tables[t];
            if (!h->size || (t > 16 && t < 24) || t > 24) continue;
            codes16 = h->codes;
            lens = h->lens;
            count = h->size * h->size;
        } else {
            codes8 = ahmp3_count1_codes[t - 32];
            lens = ahmp3_count1_lens[t - 32];
            count = 16;
        }
        double kraft = 0;
        bool prefix_free = true;
        for (int i = 0; i < count; i++) {
            unsigned ci = codes16 ? codes16[i] : codes8[i];
            kraft += ldexp(1, -lens[i]);
            for (int j = 0; j < count; j++) {
                if (i == j || lens[j] < lens[i]) continue;
                unsigned cj = codes16 ? codes16[j] : codes8[j];
                if ((cj >> (lens[j] - lens[i])) == ci) prefix_free = false;
            }
        }
        CHECK(kraft == 1.0 && prefix_free, "table %d is not a complete prefix code", t);
    }
}

/* ---- building test frames --------------------------------------------------- */

typedef struct {
    int lsf, sr_index, bitrate_index, channels, mode;
} stream_kind;

static void frame_start(ahmp3_fr *f, const stream_kind *k)
{
    memset(f, 0, sizeof(*f));
    f->lsf = k->lsf;
    f->sr_index = k->sr_index;
    f->bitrate_index = k->bitrate_index;
    f->channels = k->channels;
    f->mode = k->mode;
    f->granules = k->lsf ? 1 : 2;
    for (int g = 0; g < 2; g++)
        for (int ch = 0; ch < 2; ch++) {
            f->gr[g][ch].long_end = 22;
            f->gr[g][ch].short_start = 13;
        }
}

static int frame_size(const ahmp3_fr *f)
{
    if (!f->bitrate_index) return g_free_bytes + f->padding;
    int rate = ahmp3_sample_rates[f->sr_index], kbps = ahmp3_bitrates[f->lsf][f->bitrate_index];
    return (f->lsf ? 72000 : 144000) * kbps / rate + f->padding;
}

/* Writes the frame with main_data_begin 0. False when the data does not fit. */
static bool frame_write(ahmp3_fr *f, bytes *out)
{
    for (int g = 0; g < f->granules; g++)
        for (int ch = 0; ch < f->channels; ch++) {
            ahmp3_gr *G = &f->gr[g][ch];
            G->part23 = ahmp3_part2_bits(G, f->lsf, ahmp3_intensity_right(f, ch), !f->lsf && g ? &f->gr[0][ch] : NULL) +
                        ahmp3_huffman_bits(G, f->sr_index);
            if (G->part23 > 4095) return false;
        }
    unsigned char head[40], main[4096];
    int size = frame_size(f);
    int head_bytes = ahmp3_write_header(f, head);
    int main_bytes = (ahmp3_write_main(f, main, sizeof(main)) + 7) / 8;
    if (head_bytes + main_bytes > size) return false;
    if (g_free_bytes) {
        /* Keep only what also fits the frame with a stated bit rate, so
           both streams hold the same frames. */
        int stated = (f->lsf ? 72000 : 144000) * ahmp3_bitrates[f->lsf][14] / ahmp3_sample_rates[f->sr_index];
        if (head_bytes - 2 * f->crc + main_bytes > stated) return false;
    }
    unsigned char frame[5000] = {0};
    memcpy(frame, head, (size_t)head_bytes);
    memcpy(frame + head_bytes, main, (size_t)main_bytes);
    bytes_put(out, frame, (size_t)size);
    return true;
}

/* ---- every Huffman code against ffmpeg -------------------------------------- */

typedef struct {
    int table, x, y; /* table 32/33 = count1 A/B, x = the quadruple */
} code_case;

static void test_every_code(void)
{
    stream_kind k = {0, 0, 14, 1, 3}; /* MPEG-1 44.1 kHz 320 kbps mono */
    bytes mp3 = {0};
    code_case *cases = malloc(4096 * sizeof(*cases));
    int count = 0;
    static const int tables[] = {1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 24};
    for (size_t i = 0; i < sizeof(tables) / sizeof(tables[0]); i++) {
        int t = tables[i], size = ahmp3_huff_tables[t].size;
        for (int x = 0; x < size; x++)
            for (int y = 0; y < size; y++) cases[count++] = (code_case){t, x, y};
    }
    /* Escapes of every linbits table, near both ends of their range. */
    for (int t = 16; t < 32; t++) {
        int lb = ahmp3_huff_tables[t].linbits;
        int top = 15 + (1 << lb) - 1;
        if (top > 8206) top = 8206;
        cases[count++] = (code_case){t, top, 15};
        cases[count++] = (code_case){t, 15, top};
        cases[count++] = (code_case){t, 16, 3};
        cases[count++] = (code_case){t, rnd_in(15, top), rnd_in(15, top)};
    }
    for (int t = 32; t < 34; t++)
        for (int q = 0; q < 16; q++) cases[count++] = (code_case){t, q, 0};

    for (int c = 0; c < count; c++) {
        ahmp3_fr f;
        frame_start(&f, &k);
        ahmp3_gr *G = &f.gr[0][0];
        G->global_gain = 140;
        if (cases[c].table < 32) {
            G->ix[0] = rnd() & 1 ? -cases[c].x : cases[c].x;
            G->ix[1] = rnd() & 1 ? -cases[c].y : cases[c].y;
            G->big_values = 1;
            G->table[0] = cases[c].table;
            G->count1_end = 2;
        } else {
            for (int i = 0; i < 4; i++)
                G->ix[i] = (cases[c].x & (8 >> i)) ? (rnd() & 1 ? -1 : 1) : 0;
            G->count1_table = cases[c].table - 32;
            G->count1_end = 4;
        }
        CHECK(frame_write(&f, &mp3), "code case %d does not fit", c);
        frame_start(&f, &k); /* a silent frame lets the filters settle */
        frame_write(&f, &mp3);
    }

    char path[600];
    path_of(path, "codes.mp3");
    write_file(path, mp3.p, mp3.len);
    pcm mine = {0}, ref = {0};
    decode_all(mp3.p, mp3.len, &mine, NULL);
    CHECK(ffmpeg_decode(path, &ref), "ffmpeg could not decode the code test");
    CHECK(mine.len == ref.len, "code test lengths differ: %zu vs %zu", mine.len, ref.len);
    int bad = 0;
    for (int c = 0; c < count && bad < 10; c++) {
        double d = max_diff(&ref, &mine, (size_t)c * 2304, (size_t)c * 2304 + 2304);
        if (d > 1e-4) {
            bad++;
            CHECK(false, "table %d value (%d, %d) decodes differently from ffmpeg (diff %.2g)", cases[c].table,
                  cases[c].x, cases[c].y, d);
        }
    }
    printf("  %d codes compared\n", count);
    free(cases);
    free(mp3.p);
    free(mine.p);
    free(ref.p);
}

/* ---- random frames using every feature, against ffmpeg ---------------------- */

static int random_table(const int *ix, int start, int end)
{
    int max = 0;
    for (int j = start; j < end; j++) {
        int v = ix[j] < 0 ? -ix[j] : ix[j];
        if (v > max) max = v;
    }
    if (max == 0) return rnd_in(0, 3) ? 0 : 1;
    for (int tries = 0; tries < 200; tries++) {
        int t = rnd_in(1, 31);
        const ahmp3_huff *h = &ahmp3_huff_tables[t];
        if (!h->size) continue;
        if (h->linbits ? max - 15 < (1 << h->linbits) : max < h->size) return t;
    }
    return max <= 15 ? 15 : 31;
}

static void random_values(ahmp3_gr *G, int sr, int lines, int loud)
{
    memset(G->ix, 0, sizeof(G->ix));
    for (int j = 0; j < lines; j++) {
        int r = rnd_in(0, 99), v;
        if (r < 45) v = 0;
        else if (r < 75) v = 1;
        else if (r < 92) v = rnd_in(2, 6);
        else if (r < 98 || !loud) v = rnd_in(7, 15);
        else v = rnd_in(16, rnd_in(0, 3) ? 60 : 8000);
        G->ix[j] = rnd() & 1 ? -v : v;
    }
    /* A tail of -1/0/1 for the count1 region. */
    int tail = rnd_in(0, 40) * 4;
    for (int j = lines; j < lines + tail && j < 576; j++) G->ix[j] = rnd_in(-1, 1);
    ahmp3_huffman_choose(G, sr, rnd() & 1);
    G->count1_table = (int)(rnd() & 1);
    if (!G->window_switching) {
        G->region0_count = rnd_in(0, 15);
        G->region1_count = rnd_in(0, 7);
    }
    /* Any table the values fit. */
    int b[3];
    int big = G->big_values * 2;
    int r1, r2;
    if (G->window_switching) {
        r1 = G->block_type == 2 ? (sr == 8 ? 72 : 36) : ahmp3_sfb_long[sr][8];
        r2 = 576;
    } else {
        r1 = ahmp3_sfb_long[sr][G->region0_count + 1];
        int l = G->region0_count + G->region1_count + 2;
        r2 = ahmp3_sfb_long[sr][l > 22 ? 22 : l];
    }
    b[0] = r1 < big ? r1 : big;
    b[1] = r2 < big ? r2 : big;
    if (b[1] < b[0]) b[1] = b[0];
    b[2] = big;
    int start = 0;
    for (int r = 0; r < 3 && (g_features & F_TABLES); r++) {
        G->table[r] = random_table(G->ix, start, b[r]);
        start = b[r];
    }
    if (G->window_switching) G->table[2] = 0;
}

static void random_scalefactors(ahmp3_fr *f, int g, int ch)
{
    ahmp3_gr *G = &f->gr[g][ch];
    int count = G->long_end + (13 - G->short_start) * 3;
    memset(G->sf, 0, sizeof(G->sf));
    if (!f->lsf) {
        G->scalefac_compress = rnd_in(0, 15);
        int s1 = ahmp3_slen[0][G->scalefac_compress], s2 = ahmp3_slen[1][G->scalefac_compress];
        for (int n = 0; n < count; n++) {
            bool first = n < G->long_end ? n < 11 : G->short_start + (n - G->long_end) / 3 < 6;
            int b = n < G->long_end ? n : G->short_start + (n - G->long_end) / 3;
            bool last = n < G->long_end ? b == 21 : b == 12;
            G->sf[n] = last ? 0 : (unsigned char)rnd_in(0, (1 << (first ? s1 : s2)) - 1);
        }
        if (!(g_features & F_SF)) memset(G->sf, 0, sizeof(G->sf));
        if (g == 1 && G->block_type != 2 && (g_features & F_SCFSI)) {
            G->scfsi = rnd_in(0, 15);
            static const int groups[5] = {0, 6, 11, 16, 21};
            for (int k = 0; k < 4; k++)
                if (G->scfsi & (8 >> k))
                    for (int i = groups[k]; i < groups[k + 1]; i++) G->sf[i] = f->gr[0][ch].sf[i];
        }
        return;
    }
    /* MPEG-2: a random 9-bit scalefac_compress, then values its lengths hold. */
    bool is_right = ahmp3_intensity_right(f, ch);
    int sfc = rnd_in(0, 511), slen[4], table;
    int sf = sfc;
    if (is_right) {
        sf >>= 1;
        int n1, n2;
        if (sf < 180) { table = 3; n1 = 6; n2 = 6; }
        else if (sf < 244) { sf -= 180; table = 4; n1 = 4; n2 = 4; }
        else { sf -= 244; table = 5; n1 = 3; n2 = 0; }
        slen[3] = 0;
        slen[2] = n2 ? sf % n2 : 0;
        if (n2) sf /= n2;
        slen[1] = sf % n1;
        slen[0] = sf / n1;
    } else {
        if (sfc < 400) { table = 0; slen[0] = (sf >> 4) / 5; slen[1] = (sf >> 4) % 5; slen[2] = (sf & 15) >> 2; slen[3] = sf & 3; }
        else if (sfc < 500) { sf -= 400; table = 1; slen[0] = (sf >> 2) / 5; slen[1] = (sf >> 2) % 5; slen[2] = sf & 3; slen[3] = 0; }
        else { sf -= 500; table = 2; slen[0] = sf / 3; slen[1] = sf % 3; slen[2] = slen[3] = 0; }
    }
    G->scalefac_compress = sfc;
    int kind = G->block_type == 2 ? (G->mixed ? 2 : 1) : 0, j = 0;
    for (int k = 0; k < 4; k++)
        for (int i = 0; i < ahmp3_lsf_nsf[table][kind][k]; i++) G->sf[j++] = (unsigned char)rnd_in(0, (1 << slen[k]) - 1);
}

static void test_random_frames(const char *name, stream_kind k, int frames)
{
    bytes mp3 = {0};
    int made = 0;
    /* Block types follow the standard's order: a short block comes after a
       start or short block. ffmpeg relies on that and drops overlap when it
       is broken; this decoder overlaps exactly either way. */
    int last_type[2] = {0, 0}, last_mixed[2] = {0, 0};
    for (int n = 0; n < frames * 4 && made < frames; n++) {
        int prev_type[2] = {last_type[0], last_type[1]}, prev_mixed[2] = {last_mixed[0], last_mixed[1]};
        ahmp3_fr f;
        frame_start(&f, &k);
        f.crc = g_crc;
        if (g_free_bytes) f.bitrate_index = 0;
        if (k.channels == 2) {
            /* The mode stays the same through a stream, as in real files. */
            if (f.mode == 1) f.mode_ext = ((g_features & F_MS) ? rnd_in(0, 1) * 2 : 0) | ((g_features & F_IS) ? rnd_in(0, 1) : 0);
        }
        bool joint = f.mode == 1 && f.mode_ext;
        for (int g = 0; g < f.granules; g++) {
            int shared_type = rnd_in(0, 3), shared_mixed = rnd_in(0, 2) == 0;
            for (int ch = 0; ch < f.channels; ch++) {
                ahmp3_gr *G = &f.gr[g][ch];
                G->block_type = g_force_type >= 100 ? (g ? g_force_type % 10 : (g_force_type / 10) % 10) : g_force_type >= 0 ? g_force_type : !(g_features & F_SWITCH) ? 0 : joint ? shared_type : rnd_in(0, 3);
                G->window_switching = G->block_type != 0;
                if (G->block_type == 2 && (prev_type[ch] == 0 || prev_type[ch] == 3)) G->block_type = 1;
                G->window_switching = G->block_type != 0;
                prev_type[ch] = G->block_type;
                G->mixed = (g_features & F_MIXED) && G->block_type == 2 && k.sr_index != 8 && (joint ? shared_mixed : rnd_in(0, 2) == 0);
                /* Same reason: after a mixed block ffmpeg needs another mixed one. */
                if (G->block_type == 2 && prev_mixed[ch] && k.sr_index != 8) G->mixed = 1;
                prev_mixed[ch] = G->mixed;
                if (G->block_type == 2) {
                    G->long_end = G->mixed ? (f.lsf ? 6 : 8) : 0;
                    G->short_start = G->mixed ? 3 : 0;
                }
                G->global_gain = rnd_in(120, 190);
                G->scalefac_scale = (g_features & F_SFSCALE) ? (int)(rnd() & 1) : 0;
                G->preflag = f.lsf || !(g_features & F_PREFLAG) ? 0 : (int)(rnd() & 1);
                for (int i = 0; i < 3; i++) G->subblock_gain[i] = G->window_switching && (g_features & F_SUBGAIN) ? rnd_in(0, 7) : 0;
                int lines = rnd_in(0, 4) == 0 ? 0 : rnd_in(2, 200) & ~1;
                /* For intensity stereo the right channel stops early. */
                if (ch == 1 && (f.mode_ext & 1) && f.mode == 1) lines = rnd_in(0, 60) & ~1;
                random_values(G, k.sr_index, lines, rnd_in(0, 3) == 0);
                /* Keep it near full scale, as real music is. */
                int peak = 1;
                for (int j = 0; j < 576; j++) peak = abs(G->ix[j]) > peak ? abs(G->ix[j]) : peak;
                int top = 210 - (int)ceil(4 * log2(pow(peak, 4.0 / 3.0)));
                if (G->global_gain > top) G->global_gain = top < 0 ? 0 : top;
                random_scalefactors(&f, g, ch);
            }
        }
        f.padding = g_free_bytes ? (n * 7 + made) % 3 == 0 : 0;
        if (frame_write(&f, &mp3)) {
            made++;
            last_type[0] = prev_type[0];
            last_type[1] = prev_type[1];
            last_mixed[0] = prev_mixed[0];
            last_mixed[1] = prev_mixed[1];
        }
    }

    char path[600];
    path_of(path, name);
    write_file(path, mp3.p, mp3.len);
    pcm mine = {0}, ref = {0};
    decode_all(mp3.p, mp3.len, &mine, NULL);
    if (g_keep) {
        g_keep->len = 0;
        pcm_put(g_keep, mine.p, mine.len);
        CHECK(mine.len == (size_t)made * (size_t)(k.lsf ? 576 : 1152) * (size_t)k.channels, "%s: %zu samples from %d frames", name, mine.len, made);
    }
    if (g_free_bytes) {
        /* ffmpeg does not decode free format. */
        free(mp3.p);
        free(mine.p);
        return;
    }
    CHECK(ffmpeg_decode(path, &ref), "%s: ffmpeg could not decode", name);
    CHECK(mine.len == ref.len, "%s: lengths differ: %zu vs %zu", name, mine.len, ref.len);
    size_t frame_samples = (size_t)(k.lsf ? 576 : 1152) * k.channels;
    int bad = 0;
    for (size_t at = 0; at < ref.len && bad < 5; at += frame_samples) {
        double d = max_diff(&ref, &mine, at, at + frame_samples);
        if (d > 2e-4) {
            bad++;
            CHECK(false, "%s: frame %zu differs from ffmpeg (diff %.2g)", name, at / frame_samples, d);
        }
    }
    printf("  %s: %d frames compared\n", name, made);
    free(mp3.p);
    free(mine.p);
    free(ref.p);
}

/* ---- CRC and free format ---------------------------------------------------- */

/* Frames with a CRC decode as ffmpeg decodes them; one whose protected bytes
   are damaged is reported and played as silence. */
static void test_crc(void)
{
    g_crc = 1;
    if (g_ffmpeg) {
        test_random_frames("crc_mpeg1_stereo.mp3", (stream_kind){0, 0, 14, 2, 1}, 120);
        test_random_frames("crc_mpeg2_mono.mp3", (stream_kind){1, 3, 14, 1, 3}, 120);
    }
    /* Loud frames, then the same with the side information of every third
       one damaged. */
    stream_kind k = {0, 0, 14, 2, 0};
    bytes mp3 = {0};
    int count = 30, sizes = 0;
    for (int n = 0; n < count; n++) {
        ahmp3_fr f;
        frame_start(&f, &k);
        f.crc = 1;
        for (int g = 0; g < 2; g++)
            for (int ch = 0; ch < 2; ch++) {
                ahmp3_gr *G = &f.gr[g][ch];
                G->global_gain = 180;
                for (int j = 0; j < 40; j++) G->ix[j] = rnd_in(-6, 6);
                ahmp3_huffman_choose(G, k.sr_index, false);
            }
        CHECK(frame_write(&f, &mp3), "CRC test frame does not fit");
        sizes = (int)mp3.len / (n + 1);
    }
    for (int pass = 0; pass < 2; pass++) {
        if (pass)
            for (int n = 0; n < count; n += 3) mp3.p[n * sizes + 6 + rnd_in(0, 31)] ^= (unsigned char)(1 << rnd_in(0, 7));
        ahmp3_decoder *d = ahmp3_decoder_new();
        size_t pos = 0;
        int errors = 0, frames = 0, wrong = 0;
        for (;;) {
            float buf[AHMP3_MAX_SAMPLES * 2];
            int n;
            ahmp3_frame fr;
            size_t used = ahmp3_decode(d, mp3.p + pos, mp3.len - pos, true, buf, &n, &fr);
            if (!used) break;
            pos += used;
            if (!n) continue;
            double energy = 0;
            for (int i = 2 * 1100; i < 2 * 1152; i++) energy += (double)buf[i] * buf[i];
            if (fr.crc_error) errors++;
            /* Past the filter's memory of the frame before, a failed frame
               is silent and a good one is not. */
            if (fr.crc_error != (energy < 1e-12)) wrong++;
            if (fr.crc_error != (pass && frames % 3 == 0)) wrong++;
            frames++;
        }
        ahmp3_decoder_free(d);
        CHECK(frames == count && errors == (pass ? count / 3 : 0) && !wrong, "CRC pass %d: %d frames, %d CRC errors, %d wrong", pass, frames,
              errors, wrong);
    }
    free(mp3.p);
    g_crc = 0;
}

/* Free format: the same frames written with a stated bit rate and as
   free-format frames of other sizes decode to the same sound. */
static void test_free_format(void)
{
    static const struct {
        stream_kind kind;
        int bytes;
    } cases[] = {
        {{0, 0, 14, 2, 1}, 1500}, {{0, 0, 14, 2, 1}, 2880}, {{0, 2, 14, 1, 3}, 1441}, {{0, 1, 14, 2, 1}, 4000},
        {{1, 3, 14, 2, 1}, 700},  {{1, 8, 14, 2, 1}, 1700}, {{1, 4, 14, 1, 3}, 577},
    };
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        pcm stated = {0}, free_format = {0};
        unsigned seed = g_seed;
        g_keep = &stated;
        test_random_frames("free_stated.mp3", cases[c].kind, 60);
        g_seed = seed;
        g_keep = &free_format;
        g_free_bytes = cases[c].bytes;
        g_crc = (int)(c & 1);
        test_random_frames("free_format.mp3", cases[c].kind, 60);
        g_free_bytes = 0;
        g_crc = 0;
        g_keep = NULL;
        CHECK(stated.len == free_format.len && stated.len > 0 && !memcmp(stated.p, free_format.p, stated.len * sizeof(float)),
              "free format case %zu: decodes differently from the same frames with a bit rate (%zu vs %zu samples)", c, free_format.len,
              stated.len);
        free(stated.p);
        free(free_format.p);
    }
}

/* ---- files from other encoders --------------------------------------------- */

void make_source(void)
{
    char path[600], cmd[2000];
    path_of(path, "source.wav");
    FILE *f = fopen(path, "rb");
    if (f) { fclose(f); return; }
    snprintf(cmd, sizeof(cmd),
             "ffmpeg -v error -nostdin -y -f lavfi -i \"aevalsrc=exprs='"
             "0.3*sin(2*PI*220*t)*(1+0.5*sin(2*PI*0.5*t))+0.2*sin(2*PI*1330*t+3*sin(2*PI*5*t))"
             "+0.15*(random(0)-0.5)*gt(mod(t\\,1.3)\\,1.1)+0.4*exp(-40*mod(t\\,0.5))*sin(2*PI*3000*t)|"
             "0.3*sin(2*PI*330*t)+0.2*sin(2*PI*2210*t)*sin(2*PI*0.3*t)+0.1*(random(1)-0.5)"
             "+0.4*exp(-60*mod(t+0.25\\,0.5))*sin(2*PI*6000*t)':s=48000:d=6\" \"%s\"",
             path);
    CHECK(system(cmd) == 0, "ffmpeg could not make the source signal");
}

static void test_other_encoder(const char *name, const char *args)
{
    char src[600], mp3[600], cmd[2000];
    path_of(src, "source.wav");
    path_of(mp3, name);
    snprintf(cmd, sizeof(cmd), "ffmpeg -v error -nostdin -y -i \"%s\" %s -write_xing 0 \"%s\"", src, args, mp3);
    if (system(cmd) != 0) {
        printf("  %s: encoder not available in this ffmpeg, skipped\n", name);
        return;
    }
    bytes b = {0};
    read_file(mp3, &b);
    pcm mine = {0}, ref = {0};
    decode_all(b.p, b.len, &mine, NULL);
    CHECK(ffmpeg_decode(mp3, &ref), "%s: ffmpeg could not decode", name);
    CHECK(mine.len == ref.len && ref.len > 0, "%s: lengths differ: %zu vs %zu", name, mine.len, ref.len);
    CHECK(max_diff(&ref, &mine, 0, ref.len) < 1e-4, "%s: decodes differently from ffmpeg (%.2g)", name,
          max_diff(&ref, &mine, 0, ref.len));
    free(b.p);
    free(mine.p);
    free(ref.p);
}

/* A file with a LAME tag: trimming its delay and padding gives exactly what
   ffmpeg's gapless playback gives. */
static void test_gapless(void)
{
    char src[600], mp3[600], cmd[2000];
    path_of(src, "source.wav");
    path_of(mp3, "gapless.mp3");
    snprintf(cmd, sizeof(cmd), "ffmpeg -v error -nostdin -y -i \"%s\" -ar 44100 -c:a libmp3lame -b:a 128k \"%s\"", src, mp3);
    if (system(cmd) != 0) {
        printf("  LAME not available in this ffmpeg, skipped\n");
        return;
    }
    bytes b = {0};
    read_file(mp3, &b);
    pcm mine = {0}, ref = {0};
    ahmp3_info info;
    decode_all(b.p, b.len, &mine, &info);
    char raw[600];
    snprintf(raw, sizeof(raw), "%s.f32", mp3);
    snprintf(cmd, sizeof(cmd), "ffmpeg -v error -nostdin -y -i \"%s\" -f f32le \"%s\"", mp3, raw);
    CHECK(system(cmd) == 0, "ffmpeg could not decode the gapless file");
    bytes r = {0};
    read_file(raw, &r);
    pcm_put(&ref, (const float *)r.p, r.len / 4);
    CHECK(info.present && info.encoder_delay > 0 && info.encoder_padding > 0, "LAME tag not read (delay %d, padding %d)",
          info.encoder_delay, info.encoder_padding);
    size_t ch = 2, skip = (size_t)(info.encoder_delay + AHMP3_DECODER_DELAY) * ch;
    size_t cut = (size_t)(info.encoder_padding - AHMP3_DECODER_DELAY) * ch;
    size_t len = mine.len > skip + cut ? mine.len - skip - cut : 0;
    CHECK(len == ref.len, "gapless length %zu, ffmpeg %zu", len / ch, ref.len / ch);
    CHECK(len == (size_t)6 * 44100 * ch, "gapless length %zu is not the source's %d", len / ch, 6 * 44100);
    pcm trimmed = {0};
    pcm_put(&trimmed, mine.p + skip, len);
    CHECK(max_diff(&ref, &trimmed, 0, ref.len) < 1e-4, "trimmed audio differs from ffmpeg's gapless output");
    free(trimmed.p);
    free(r.p);
    free(b.p);
    free(mine.p);
    free(ref.p);
}

/* ---- the encoder ------------------------------------------------------------ */

typedef struct {
    double snr_db, seg_min_db;
    size_t samples;
    int lag;
} quality;

static void signal(float *x, size_t frames, int channels, int rate, int kind)
{
    for (size_t i = 0; i < frames; i++) {
        double t = (double)i / rate;
        for (int ch = 0; ch < channels; ch++) {
            double v;
            switch (kind) {
            case 0: v = 0.5 * sin(2 * AHMP3_PI * (440 + 220 * ch) * t); break; /* tones */
            case 1: v = ((rnd() & 0xFFFF) / 65536.0 - 0.5) * 0.6; break;       /* noise */
            case 2: /* clicks every 0.25 s over quiet */
                v = 0.01 * sin(2 * AHMP3_PI * 300 * t) + (fmod(t, 0.25) < 0.004 ? 0.8 * sin(2 * AHMP3_PI * 2500 * t) : 0);
                break;
            case 3: v = 0; break; /* silence */
            case 5: /* a mix that changes: tones that come and go, noise bursts, clicks, a quiet stretch */
                v = 0.3 * sin(2 * AHMP3_PI * (300 + 200 * ch) * t) * (fmod(t, 0.9) < 0.6) + 0.2 * sin(2 * AHMP3_PI * 3100 * t) * (fmod(t, 0.37) < 0.2) +
                    (fmod(t, 0.7) > 0.55 ? ((rnd() & 0xFFFF) / 65536.0 - 0.5) * 0.3 : 0) + (fmod(t, 0.31) < 0.003 ? 0.5 * sin(2 * AHMP3_PI * 5000 * t) : 0);
                if (t > 1.9 && t < 2.2) v *= 0.01;
                break;
            default: v = ((i / 40) & 1) ? 0.99 : -0.99; break; /* full-scale square */
            }
            x[i * channels + ch] = (float)v;
        }
    }
}

static int g_round_trip_vbr; /* the variable bit rate quality round_trip uses, 0 = constant */

/* Encodes, then decodes with this decoder; quality against the input. */
static quality round_trip(int rate, int channels, int kbps, ahmp3_mode mode, int kind, bytes *mp3)
{
    quality q = {0, 0, 0, 0};
    size_t frames = (size_t)rate * 3;
    float *x = malloc(frames * channels * sizeof(float));
    signal(x, frames, channels, rate, kind);
    ahmp3_encoder_config cfg = {rate, channels, kbps, mode, g_round_trip_vbr};
    ahmp3_encoder *e = ahmp3_encoder_new(&cfg);
    CHECK(e != NULL, "encoder refused %d Hz %d kbps", rate, kbps);
    if (!e) { free(x); return q; }
    unsigned char *buf = malloc(AHMP3_ENCODE_BOUND(1000));
    mp3->len = 0;
    for (size_t i = 0; i < frames; i += 1000) {
        size_t n = frames - i < 1000 ? frames - i : 1000;
        bytes_put(mp3, buf, ahmp3_encode(e, x + i * channels, n, buf, AHMP3_ENCODE_BOUND(1000)));
    }
    bytes_put(mp3, buf, ahmp3_encoder_flush(e, buf, AHMP3_ENCODE_BOUND(1000)));
    int delay = ahmp3_encoder_delay(e) + AHMP3_DECODER_DELAY;
    ahmp3_encoder_free(e);

    pcm out = {0};
    decode_all(mp3->p, mp3->len, &out, NULL);
    int och = out.channels ? out.channels : 1;
    q.samples = out.len / och;
    q.lag = delay;
    double sig = 0, noise = 0, seg_min = 1e9;
    double seg_sig = 0, seg_noise = 0;
    for (size_t i = 0; i < frames; i++) {
        for (int ch = 0; ch < och; ch++) {
            double a = channels == 2 && och == 1 ? (x[i * 2] + x[i * 2 + 1]) / 2 : x[i * channels + ch];
            size_t at = (i + (size_t)delay) * och + ch;
            double b = at < out.len ? out.p[at] : 0;
            sig += a * a;
            noise += (a - b) * (a - b);
            seg_sig += a * a;
            seg_noise += (a - b) * (a - b);
        }
        if (i % 1152 == 1151) {
            if (seg_sig > 1e-6) {
                double s = 10 * log10(seg_sig / (seg_noise + 1e-30));
                if (s < seg_min) seg_min = s;
            }
            seg_sig = seg_noise = 0;
        }
    }
    q.snr_db = sig > 0 ? 10 * log10(sig / (noise + 1e-30)) : (noise < 1e-12 ? 200 : -200);
    q.seg_min_db = seg_min;
    free(out.p);
    free(x);
    free(buf);
    return q;
}

static void test_encoder(void)
{
    bytes mp3 = {0};
    static const struct {
        int rate, channels, kbps;
        ahmp3_mode mode;
    } cases[] = {
        {44100, 2, 128, AHMP3_JOINT_STEREO}, {44100, 2, 320, AHMP3_STEREO}, {48000, 2, 192, AHMP3_JOINT_STEREO},
        {32000, 1, 64, AHMP3_MONO},          {44100, 2, 64, AHMP3_MONO},    {22050, 2, 64, AHMP3_JOINT_STEREO},
        {24000, 1, 32, AHMP3_MONO},          {16000, 2, 48, AHMP3_STEREO},  {11025, 1, 16, AHMP3_MONO},
        {8000, 1, 8, AHMP3_MONO},            {12000, 2, 32, AHMP3_JOINT_STEREO},
    };
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        quality q = round_trip(cases[c].rate, cases[c].channels, cases[c].kbps, cases[c].mode, 0, &mp3);
        printf("  %5d Hz %d ch %3d kbps tones: SNR %.1f dB\n", cases[c].rate, cases[c].channels, cases[c].kbps, q.snr_db);
        CHECK(q.snr_db > 12, "%d Hz %d kbps: tones come back at only %.1f dB", cases[c].rate, cases[c].kbps, q.snr_db);
        /* Every frame has the size the bit rate says. */
        size_t pos = 0, frames = 0;
        bool sizes_ok = true;
        while (pos + 4 <= mp3.len) {
            ahmp3_frame fr;
            if (!ahmp3_parse_header(mp3.p + pos, mp3.len - pos, &fr) || fr.bitrate_kbps != cases[c].kbps) {
                sizes_ok = false;
                break;
            }
            pos += (size_t)fr.frame_bytes;
            frames++;
        }
        CHECK(sizes_ok && pos == mp3.len, "%d Hz %d kbps: the stream is not a clean run of frames", cases[c].rate, cases[c].kbps);

        if (g_ffmpeg) {
            /* ffmpeg decodes our files exactly as we do. */
            char path[600], name[64];
            snprintf(name, sizeof(name), "enc_%d_%d_%d.mp3", cases[c].rate, cases[c].channels, cases[c].kbps);
            path_of(path, name);
            write_file(path, mp3.p, mp3.len);
            pcm mine = {0}, ref = {0};
            decode_all(mp3.p, mp3.len, &mine, NULL);
            CHECK(ffmpeg_decode(path, &ref), "%s: ffmpeg could not decode it", name);
            CHECK(mine.len == ref.len && max_diff(&ref, &mine, 0, ref.len) < 1e-4,
                  "%s: ffmpeg decodes it differently (%zu vs %zu samples, diff %.2g)", name, ref.len, mine.len,
                  max_diff(&ref, &mine, 0, ref.len));
            free(mine.p);
            free(ref.p);
        }
    }

    /* Hard material: noise at a low rate, clicks, silence, a full-scale square. */
    static const char *kinds[] = {"tones", "noise", "clicks", "silence", "square"};
    for (int kind = 1; kind < 5; kind++) {
        quality q = round_trip(44100, 2, kind == 1 ? 64 : 128, AHMP3_JOINT_STEREO, kind, &mp3);
        printf("  44100 Hz %-7s: SNR %.1f dB, worst frame %.1f dB\n", kinds[kind], q.snr_db, q.seg_min_db);
        if (kind == 3) CHECK(q.snr_db >= 200 || q.snr_db == -200 || q.snr_db > 100, "silence is not silent");
        /* White noise at 64 kbps loses its treble to the lowpass, as with LAME (2.4 dB). */
        else CHECK(q.snr_db > (kind == 1 ? 1.0 : 10.0), "%s comes back at only %.1f dB", kinds[kind], q.snr_db);
        if (g_ffmpeg) {
            char path[600];
            path_of(path, "enc_hard.mp3");
            write_file(path, mp3.p, mp3.len);
            pcm mine = {0}, ref = {0};
            decode_all(mp3.p, mp3.len, &mine, NULL);
            CHECK(ffmpeg_decode(path, &ref) && mine.len == ref.len && max_diff(&ref, &mine, 0, ref.len) < 1e-4,
                  "%s: ffmpeg decodes it differently", kinds[kind]);
            free(mine.p);
            free(ref.p);
        }
    }
    free(mp3.p);
}

/* Walks a stream frame by frame: false when it is not a clean run of frames.
   Gives the number of frames, the lowest and highest bit rate, and how many
   different ones there were. */
static bool walk_frames(const bytes *mp3, size_t *frames, int *low, int *high, int *kinds)
{
    bool seen[400] = {false};
    size_t pos = 0;
    *frames = 0;
    *low = 1000;
    *high = 0;
    *kinds = 0;
    while (pos + 4 <= mp3->len) {
        ahmp3_frame fr;
        if (!ahmp3_parse_header(mp3->p + pos, mp3->len - pos, &fr) || !fr.frame_bytes) return false;
        pos += (size_t)fr.frame_bytes;
        (*frames)++;
        if (fr.bitrate_kbps < *low) *low = fr.bitrate_kbps;
        if (fr.bitrate_kbps > *high) *high = fr.bitrate_kbps;
        if (!seen[fr.bitrate_kbps]) (*kinds)++;
        seen[fr.bitrate_kbps] = true;
    }
    return pos == mp3->len;
}

static void test_vbr(void)
{
    bytes mp3 = {0};
    size_t frames;
    int low, high, kinds;

    /* More quality costs more bits and gives less noise; frames differ in
       size; ffmpeg decodes the stream exactly as this decoder does. */
    double last_snr = 0, last_kbps = 0;
    for (int level = 1; level <= 10; level += 3) {
        g_round_trip_vbr = level;
        quality q = round_trip(44100, 2, 0, AHMP3_JOINT_STEREO, 5, &mp3);
        bool clean = walk_frames(&mp3, &frames, &low, &high, &kinds);
        double kbps = (double)mp3.len * 8 / 3 / 1000;
        printf("  quality %2d: %.0f kbps (frames from %d to %d), SNR %.1f dB\n", level, kbps, low, high, q.snr_db);
        CHECK(clean && frames > 100, "VBR quality %d: the stream is not a clean run of frames", level);
        CHECK(kinds >= 3, "VBR quality %d: only %d different frame sizes", level, kinds);
        CHECK(kbps > last_kbps * 1.1 && q.snr_db > last_snr + 0.2, "VBR quality %d: %.0f kbps and %.1f dB, no better than the step below (%.0f, %.1f)",
              level, kbps, q.snr_db, last_kbps, last_snr);
        last_kbps = kbps;
        last_snr = q.snr_db;
        if (g_ffmpeg) {
            char path[600], name[64], cmd[1500], log_path[620];
            snprintf(name, sizeof(name), "vbr_%d.mp3", level);
            path_of(path, name);
            write_file(path, mp3.p, mp3.len);
            pcm mine = {0}, ref = {0};
            decode_all(mp3.p, mp3.len, &mine, NULL);
            CHECK(ffmpeg_decode(path, &ref) && mine.len == ref.len && max_diff(&ref, &mine, 0, ref.len) < 1e-4,
                  "%s: ffmpeg decodes it differently", name);
            snprintf(log_path, sizeof(log_path), "%s.log", path);
            snprintf(cmd, sizeof(cmd), "ffmpeg -v error -xerror -nostdin -y -i \"%s\" -f null - 2> \"%s\"", path, log_path);
            int rc = system(cmd);
            bytes log = {0};
            read_file(log_path, &log);
            CHECK(rc == 0 && log.len == 0, "%s: ffmpeg reports errors in it", name);
            free(log.p);
            free(mine.p);
            free(ref.p);
        }
    }

    /* A limit on the bit rate is kept; silence takes the smallest frames. */
    g_round_trip_vbr = 10;
    round_trip(44100, 2, 128, AHMP3_JOINT_STEREO, 5, &mp3);
    CHECK(walk_frames(&mp3, &frames, &low, &high, &kinds) && high <= 128, "VBR with a 128 kbps limit used %d kbps", high);
    quality q = round_trip(44100, 2, 0, AHMP3_JOINT_STEREO, 3, &mp3);
    CHECK(walk_frames(&mp3, &frames, &low, &high, &kinds) && high == 32 && q.snr_db > 100, "VBR silence used %d kbps frames", high);

    /* Other sample rates, mono, and hard material. */
    static const struct {
        int rate, channels, kind;
    } cases[] = {{48000, 2, 1}, {32000, 1, 2}, {22050, 2, 5}, {16000, 1, 4}, {8000, 2, 5}, {44100, 2, 4}, {11025, 1, 1}};
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        g_round_trip_vbr = 2 + (int)c;
        q = round_trip(cases[c].rate, cases[c].channels, 0, cases[c].channels == 1 ? AHMP3_MONO : AHMP3_JOINT_STEREO, cases[c].kind, &mp3);
        CHECK(walk_frames(&mp3, &frames, &low, &high, &kinds) && frames > 30, "VBR %d Hz: the stream is not a clean run of frames", cases[c].rate);
        CHECK(q.snr_db > (cases[c].kind == 1 ? 1.0 : 8.0), "VBR %d Hz kind %d comes back at only %.1f dB", cases[c].rate, cases[c].kind, q.snr_db);
        if (g_ffmpeg) {
            char path[600];
            path_of(path, "vbr_case.mp3");
            write_file(path, mp3.p, mp3.len);
            pcm mine = {0}, ref = {0};
            decode_all(mp3.p, mp3.len, &mine, NULL);
            CHECK(ffmpeg_decode(path, &ref) && mine.len == ref.len && max_diff(&ref, &mine, 0, ref.len) < 1e-4,
                  "VBR %d Hz: ffmpeg decodes it differently", cases[c].rate);
            free(mine.p);
            free(ref.p);
        }
    }
    g_round_trip_vbr = 0;

    /* Settings that make no sense are refused. */
    ahmp3_encoder_config bad1 = {44100, 2, 0, AHMP3_JOINT_STEREO, 0}, bad2 = {44100, 2, 128, AHMP3_JOINT_STEREO, 11},
                         bad3 = {44100, 2, 100, AHMP3_JOINT_STEREO, 5};
    CHECK(!ahmp3_encoder_new(&bad1) && !ahmp3_encoder_new(&bad2) && !ahmp3_encoder_new(&bad3), "bad encoder settings were accepted");
    free(mp3.p);
}

/* The info frame the encoder writes: players that read it (ffmpeg here) give
   back exactly as many samples as went in, lined up with the input. */
static void test_info_frame(void)
{
    static const struct {
        int rate, channels, kbps, vbr;
        size_t frames;
    } cases[] = {
        {44100, 2, 128, 0, 100000}, {48000, 2, 320, 0, 48000 * 2 + 1}, {44100, 1, 64, 0, 1},  {32000, 2, 32, 0, 77777},
        {22050, 2, 64, 0, 50001},   {8000, 1, 8, 0, 20000},            {11025, 2, 16, 0, 1152}, {44100, 2, 128, 0, 1151},
        {44100, 2, 0, 10, 100003},  {44100, 2, 0, 5, 99999},           {16000, 1, 0, 7, 40000}, {48000, 2, 0, 1, 96000},
    };
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        int rate = cases[c].rate, ch = cases[c].channels;
        size_t frames = cases[c].frames;
        float *x = malloc(frames * (size_t)ch * sizeof(float));
        signal(x, frames, ch, rate, 0);
        ahmp3_encoder_config cfg = {rate, ch, cases[c].kbps, ch == 1 ? AHMP3_MONO : AHMP3_JOINT_STEREO, cases[c].vbr};
        ahmp3_encoder *e = ahmp3_encoder_new(&cfg);
        CHECK(e != NULL, "info frame case %zu: encoder refused the settings", c);
        if (!e) { free(x); continue; }
        unsigned char first[AHMP3_ENCODE_BOUND(0)], last[AHMP3_ENCODE_BOUND(0)];
        unsigned char *buf = malloc(AHMP3_ENCODE_BOUND(frames));
        size_t first_len = ahmp3_encoder_info_frame(e, first, sizeof(first));
        size_t n = ahmp3_encode(e, x, frames, buf, AHMP3_ENCODE_BOUND(frames));
        n += ahmp3_encoder_flush(e, buf + n, AHMP3_ENCODE_BOUND(frames) - n);
        size_t last_len = ahmp3_encoder_info_frame(e, last, sizeof(last));
        CHECK(first_len > 0 && first_len == last_len, "info frame case %zu: its size changed (%zu, %zu)", c, first_len, last_len);
        CHECK(ahmp3_encoder_info_frame(e, last, last_len - 1) == 0, "info frame case %zu: written into too little room", c);
        ahmp3_encoder_free(e);

        bytes mp3 = {0};
        bytes_put(&mp3, last, last_len);
        bytes_put(&mp3, buf, n);
        pcm mine = {0};
        ahmp3_info info;
        decode_all(mp3.p, mp3.len, &mine, &info);
        size_t count = 0, at = last_len;
        while (at + 4 <= mp3.len) {
            ahmp3_frame fr;
            if (!ahmp3_parse_header(mp3.p + at, mp3.len - at, &fr) || !fr.frame_bytes) break;
            at += (size_t)fr.frame_bytes;
            count++;
        }
        CHECK(info.present && info.frames == count && info.bytes == mp3.len && info.encoder_delay == 528,
              "info frame case %zu: says %u frames, %u bytes, delay %d; the file has %zu frames, %zu bytes", c, info.frames,
              info.bytes, info.encoder_delay, count, mp3.len);
        /* Trimmed by the tag, the sound is the input's length and lines up
           with it sample for sample. */
        size_t skip = (size_t)(info.encoder_delay + AHMP3_DECODER_DELAY) * (size_t)ch;
        size_t cut = info.encoder_padding >= AHMP3_DECODER_DELAY ? (size_t)(info.encoder_padding - AHMP3_DECODER_DELAY) * (size_t)ch : 0;
        size_t len = mine.len > skip + cut ? mine.len - skip - cut : 0;
        CHECK(info.encoder_padding >= AHMP3_DECODER_DELAY && len == frames * (size_t)ch,
              "info frame case %zu: trimmed length %zu, input %zu (padding %d)", c, len / (size_t)ch, frames, info.encoder_padding);
        if (len == frames * (size_t)ch && frames > 5000) {
            double sig = 0, noise = 0;
            for (size_t i = 0; i < len; i++) {
                double a = x[i], b = mine.p[skip + i];
                sig += a * a;
                noise += (a - b) * (a - b);
            }
            CHECK(10 * log10(sig / (noise + 1e-30)) > 9, "info frame case %zu: trimmed sound is not lined up with the input (%.1f dB)", c,
                  10 * log10(sig / (noise + 1e-30)));
        }
        if (g_ffmpeg) {
            char name[64], path[600];
            snprintf(name, sizeof(name), "info_%zu.mp3", c);
            path_of(path, name);
            write_file(path, mp3.p, mp3.len);
            bytes ref = {0};
            CHECK(ffmpeg_raw(path, NULL, "f32le", &ref), "%s: ffmpeg could not decode it", name);
            pcm r = {0}, trimmed = {0};
            pcm_put(&r, (const float *)ref.p, ref.len / 4);
            pcm_put(&trimmed, mine.p + (skip < mine.len ? skip : mine.len), len);
            CHECK(r.len == frames * (size_t)ch, "%s: ffmpeg plays %zu samples, the input had %zu", name, r.len / (size_t)ch, frames);
            CHECK(r.len == trimmed.len && max_diff(&r, &trimmed, 0, r.len) < 1e-4, "%s: ffmpeg's gapless output differs from the tag-trimmed sound", name);
            free(ref.p);
            free(r.p);
            free(trimmed.p);
        }
        free(mp3.p);
        free(mine.p);
        free(buf);
        free(x);
    }
}

/* Bad input and a caller with no room for output: no crash, a clean stream. */
static void test_encoder_hostile(void)
{
    ahmp3_encoder_config cfg = {44100, 2, 128, AHMP3_JOINT_STEREO};
    ahmp3_encoder *e = ahmp3_encoder_new(&cfg);
    float x[2 * 4096];
    unsigned char *buf = malloc(AHMP3_ENCODE_BOUND(4096));
    bytes mp3 = {0};
    float inf, nan;
    uint32_t inf_bits = 0x7F800000u, nan_bits = 0x7FC00000u;
    memcpy(&inf, &inf_bits, sizeof(inf));
    memcpy(&nan, &nan_bits, sizeof(nan));
    for (int round = 0; round < 120; round++) {
        for (int i = 0; i < 2 * 4096; i++) {
            int r = rnd_in(0, 99);
            x[i] = r < 2 ? nan : r < 4 ? inf : r < 6 ? -inf : r < 20 ? (float)rnd_in(-100000, 100000) : (float)((rnd() & 0xFFFF) / 32768.0 - 1);
        }
        /* Every third call gets no room at all. */
        size_t cap = round % 3 == 1 ? 0 : AHMP3_ENCODE_BOUND(4096);
        bytes_put(&mp3, buf, ahmp3_encode(e, x, 4096, buf, cap));
    }
    bytes_put(&mp3, buf, ahmp3_encoder_flush(e, buf, AHMP3_ENCODE_BOUND(4096)));
    ahmp3_encoder_free(e);
    size_t pos = 0;
    bool clean = true;
    while (pos + 4 <= mp3.len) {
        ahmp3_frame fr;
        if (!ahmp3_parse_header(mp3.p + pos, mp3.len - pos, &fr)) { clean = false; break; }
        pos += (size_t)fr.frame_bytes;
    }
    CHECK(clean && pos == mp3.len && mp3.len > 0, "hostile input broke the stream");
    if (g_ffmpeg) {
        char path[600], cmd[1400];
        path_of(path, "hostile.mp3");
        write_file(path, mp3.p, mp3.len);
        snprintf(cmd, sizeof(cmd), "ffmpeg -v error -xerror -nostdin -y -i \"%s\" -f null - 2> \"%s.log\"", path, path);
        int rc = system(cmd);
        bytes log = {0};
        char log_path[620];
        snprintf(log_path, sizeof(log_path), "%s.log", path);
        read_file(log_path, &log);
        CHECK(rc == 0 && log.len == 0, "ffmpeg reports errors in the hostile-input stream");
        free(log.p);
    }
    free(mp3.p);
    free(buf);
}

/* ---- damaged input ----------------------------------------------------------- */

static void test_damage(void)
{
    bytes mp3 = {0};
    round_trip(44100, 2, 128, AHMP3_JOINT_STEREO, 0, &mp3);
    pcm good = {0};
    decode_all(mp3.p, mp3.len, &good, NULL);

    /* Junk and an ID3v2 tag in front, junk between frames: the same sound. */
    bytes messy = {0};
    unsigned char tag[10 + 300] = {'I', 'D', '3', 4, 0, 0, 0, 0, 2, 44};
    for (int i = 10; i < 310; i++) tag[i] = (unsigned char)(i * 7);
    tag[100] = 0xFF;
    tag[101] = 0xFB;
    bytes_put(&messy, tag, sizeof(tag));
    unsigned char junk[37];
    for (int i = 0; i < 37; i++) junk[i] = (unsigned char)(0xFF - i);
    bytes_put(&messy, junk, sizeof(junk));
    bytes_put(&messy, mp3.p, mp3.len);
    CHECK(ahmp3_id3v2_size(tag, sizeof(tag)) == 310, "ID3v2 size");
    pcm got = {0};
    decode_all(messy.p, messy.len, &got, NULL);
    CHECK(got.len == good.len && max_diff(&good, &got, 0, good.len) == 0, "a tag and junk in front change the sound");

    /* Feeding the decoder a few bytes at a time gives the same result. */
    ahmp3_decoder *d = ahmp3_decoder_new();
    pcm drip = {0};
    size_t have = 0, pos = 0;
    for (;;) {
        float buf[AHMP3_MAX_SAMPLES * 2];
        int n;
        bool eof = have == mp3.len;
        size_t used = ahmp3_decode(d, mp3.p + pos, have - pos, eof, buf, &n, NULL);
        pos += used;
        if (n) pcm_put(&drip, buf, (size_t)n * 2);
        if (!used) {
            if (eof) break;
            have += 97;
            if (have > mp3.len) have = mp3.len;
        }
    }
    ahmp3_decoder_free(d);
    CHECK(drip.len == good.len && max_diff(&good, &drip, 0, good.len) == 0, "decoding in small pieces differs");

    /* Random damage: no crash, no runaway output. */
    bytes broken = {0};
    for (int round = 0; round < 300; round++) {
        broken.len = 0;
        bytes_put(&broken, mp3.p, mp3.len);
        int hits = rnd_in(1, 200);
        for (int i = 0; i < hits; i++) broken.p[rnd() % broken.len] = (unsigned char)rnd();
        if (round % 3 == 0)
            for (size_t i = 0; i < broken.len; i++) broken.p[i] ^= (unsigned char)((rnd() & 31) == 0 ? rnd() : 0);
        pcm out = {0};
        decode_all(broken.p, broken.len, &out, NULL);
        CHECK(out.len <= good.len * 3, "damaged stream produced %zu samples", out.len);
        free(out.p);
    }
    /* Pure noise. */
    for (int round = 0; round < 50; round++) {
        broken.len = 0;
        for (int i = 0; i < 20000; i++) {
            unsigned char c = (unsigned char)(rnd_in(0, 9) == 0 ? 0xFF : rnd());
            bytes_put(&broken, &c, 1);
        }
        pcm out = {0};
        decode_all(broken.p, broken.len, &out, NULL);
        free(out.p);
    }
    CHECK(true, "noise decoded without a crash");
    free(broken.p);
    free(messy.p);
    free(got.p);
    free(drip.p);
    free(good.p);
    free(mp3.p);
}

int main(int argc, char **argv)
{
    snprintf(g_dir, sizeof(g_dir), "%s", argc > 1 ? argv[1] : ".");
    g_ffmpeg = system("ffmpeg -version > "
#ifdef _WIN32
                      "NUL"
#else
                      "/dev/null"
#endif
                      " 2>&1") == 0;
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "--no-ffmpeg")) g_ffmpeg = false;
        if (!strcmp(argv[i], "--type") && i + 1 < argc) g_force_type = atoi(argv[++i]);
        if (!strcmp(argv[i], "--features") && i + 1 < argc) {
            g_features = atoi(argv[++i]);
            printf("features %d\n", g_features);
            test_random_frames("probe.mp3", (stream_kind){0, 0, 14, 2, 1}, 200);
            test_random_frames("probe2.mp3", (stream_kind){1, 3, 14, 2, 1}, 200);
            return g_failures ? 1 : 0;
        }
    }
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("tables\n");
    test_tables();
    printf("encoder\n");
    test_encoder();
    printf("CRC and free format\n");
    test_crc();
    test_free_format();
    printf("variable bit rate\n");
    test_vbr();
    printf("info frame and gapless playback of the encoder's own files\n");
    test_info_frame();
    printf("damaged input\n");
    test_damage();
    test_encoder_hostile();
    if (g_ffmpeg) {
        printf("every Huffman code against ffmpeg\n");
        test_every_code();
        printf("random frames against ffmpeg\n");
        test_random_frames("rnd_mpeg1_stereo.mp3", (stream_kind){0, 0, 14, 2, 1}, 300);
        test_random_frames("rnd_mpeg1_32k_mono.mp3", (stream_kind){0, 2, 14, 1, 3}, 150);
        test_random_frames("rnd_mpeg1_48k_stereo.mp3", (stream_kind){0, 1, 14, 2, 1}, 150);
        test_random_frames("rnd_mpeg2_stereo.mp3", (stream_kind){1, 3, 14, 2, 1}, 300);
        test_random_frames("rnd_mpeg2_24k_mono.mp3", (stream_kind){1, 4, 14, 1, 3}, 150);
        test_random_frames("rnd_mpeg2_16k_stereo.mp3", (stream_kind){1, 5, 14, 2, 1}, 150);
        test_random_frames("rnd_mpeg25_stereo.mp3", (stream_kind){1, 6, 14, 2, 1}, 200);
        test_random_frames("rnd_mpeg25_12k_stereo.mp3", (stream_kind){1, 7, 14, 2, 1}, 150);
        test_random_frames("rnd_mpeg25_8k_stereo.mp3", (stream_kind){1, 8, 14, 2, 1}, 150);
        printf("files from other encoders\n");
        make_source();
        test_other_encoder("lame_128.mp3", "-c:a libmp3lame -b:a 128k");
        test_other_encoder("lame_vbr.mp3", "-c:a libmp3lame -q:a 0");
        test_other_encoder("lame_320_48k.mp3", "-ar 48000 -c:a libmp3lame -b:a 320k");
        test_other_encoder("lame_22k.mp3", "-ar 22050 -c:a libmp3lame -b:a 64k");
        test_other_encoder("lame_8k_mono.mp3", "-ar 8000 -ac 1 -c:a libmp3lame -b:a 16k");
        test_other_encoder("lame_11k.mp3", "-ar 11025 -c:a libmp3lame -b:a 32k");
        test_other_encoder("shine.mp3", "-ar 44100 -c:a libshine -b:a 128k");
        test_other_encoder("mf.mp3", "-ar 44100 -c:a mp3_mf -b:a 128k");
        printf("gapless\n");
        test_gapless();
    } else {
        printf("ffmpeg not found: the checks against it were skipped\n");
    }
    printf("Layers I and II\n");
    test_layers();
    printf("WAV\n");
    test_wav();
    printf("FLAC\n");
    test_flac();
    printf("AAC\n");
    test_aac();
    printf("%d checks, %d failed\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
