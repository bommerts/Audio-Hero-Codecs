/* MP4 reader for AAC and Apple Lossless sound tracks, following ISO/IEC
   14496-12 (the file format's boxes and sample tables) and 14496-14 (the
   esds box that holds AAC's decoder settings). The moov box, which is the file's index, is read
   whole; the sound itself is left where it is. */
#include "../include/ahm4a.h"
#include <stdlib.h>
#include <string.h>

#define MAX_INDEX_BYTES ((uint64_t)256 << 20) /* the most index (moov box) read into memory */

struct ahm4a {
    ahm4a_read_fn read;
    void *user;
    uint64_t size;
    ahm4a_info info;
    uint8_t config[64];
    uint64_t *offsets; /* of every packet */
    uint32_t *sizes;
};

/* A run of bytes in memory. */
typedef struct {
    const uint8_t *p;
    size_t len;
} span;

static uint32_t be16(const uint8_t *p) { return (uint32_t)p[0] << 8 | p[1]; }
static uint32_t be32(const uint8_t *p) { return be16(p) << 16 | be16(p + 2); }
static uint64_t be64(const uint8_t *p) { return (uint64_t)be32(p) << 32 | be32(p + 4); }

/* Takes the next box off the front of `rest`: its four-letter type and its
   contents. False when no whole box is left. */
static bool next_box(span *rest, const uint8_t **type, span *body)
{
    if (rest->len < 8) return false;
    uint64_t size = be32(rest->p);
    size_t head = 8;
    if (size == 1) {
        if (rest->len < 16) return false;
        size = be64(rest->p + 8);
        head = 16;
    } else if (size == 0) {
        size = rest->len; /* runs to the end */
    }
    if (size < head || size > rest->len) return false;
    *type = rest->p + 4;
    body->p = rest->p + head;
    body->len = (size_t)size - head;
    rest->p += (size_t)size;
    rest->len -= (size_t)size;
    return true;
}

/* The first box of the given type directly inside `in`. */
static bool find_box(span in, const char *want, span *out)
{
    const uint8_t *type;
    span body;
    while (next_box(&in, &type, &body))
        if (!memcmp(type, want, 4)) {
            *out = body;
            return true;
        }
    return false;
}

/* A descriptor of the esds box: a tag byte, then a length of 7 bits a byte. */
static bool next_descriptor(span *rest, int *tag, span *body)
{
    if (rest->len < 2) return false;
    *tag = rest->p[0];
    size_t at = 1, len = 0;
    for (int i = 0; i < 4; i++) {
        if (at >= rest->len) return false;
        uint8_t c = rest->p[at++];
        len = len << 7 | (c & 0x7F);
        if (!(c & 0x80)) break;
    }
    if (len > rest->len - at) return false;
    body->p = rest->p + at;
    body->len = len;
    rest->p += at + len;
    rest->len -= at + len;
    return true;
}

/* The AudioSpecificConfig inside an esds box, when the stream is AAC. */
static const char *read_esds(span esds, ahm4a *m)
{
    int tag;
    span es, dec, specific, rest;
    if (esds.len < 4) return "damaged MP4 sound description";
    esds.p += 4;
    esds.len -= 4;
    if (!next_descriptor(&esds, &tag, &es) || tag != 3 || es.len < 3) return "damaged MP4 sound description";
    /* Stream id, then flags that say which optional fields follow. */
    size_t skip = 3;
    if (es.p[2] & 0x80) skip += 2;
    if (es.p[2] & 0x40) skip += es.len > skip ? 1u + es.p[skip] : 1u;
    if (es.p[2] & 0x20) skip += 2;
    if (skip > es.len) return "damaged MP4 sound description";
    es.p += skip;
    es.len -= skip;
    if (!next_descriptor(&es, &tag, &dec) || tag != 4 || dec.len < 13) return "damaged MP4 sound description";
    int object = dec.p[0];
    if (object == 0x69 || object == 0x6B) return "the MP4 file holds MP3 sound, not AAC";
    if (object != 0x40 && object != 0x66 && object != 0x67 && object != 0x68) return "the MP4 file's sound is not AAC";
    rest.p = dec.p + 13;
    rest.len = dec.len - 13;
    if (!next_descriptor(&rest, &tag, &specific) || tag != 5 || specific.len < 2 || specific.len > sizeof(m->config))
        return "damaged MP4 sound description";
    memcpy(m->config, specific.p, specific.len);
    m->info.config = m->config;
    m->info.config_bytes = specific.len;
    return NULL;
}

/* Apple Lossless settings: an 'alac' box holding a version and the 24-byte
   ALACSpecificConfig, which gives the channels and the sample rate. */
static const char *read_alac(span box, ahm4a *m)
{
    if (box.len < 4 + 24) return "damaged Apple Lossless settings";
    const uint8_t *c = box.p + 4;
    uint32_t frame = be32(c), rate = be32(c + 20);
    if (!frame || !c[9] || !rate || rate > 0x7FFFFFFF) return "damaged Apple Lossless settings";
    memcpy(m->config, c, 24);
    m->info.config = m->config;
    m->info.config_bytes = 24;
    m->info.codec = AHM4A_ALAC;
    m->info.packet_samples = frame;
    m->info.channels = c[9];
    m->info.sample_rate = (int)rate;
    return NULL;
}

/* The sound description: an mp4a entry whose esds box describes AAC, or an
   alac entry. */
static const char *read_description(span stsd, ahm4a *m)
{
    if (stsd.len < 8) return "damaged MP4 sound description";
    span entries = {stsd.p + 8, stsd.len - 8}, entry, esds, wave, alac;
    const uint8_t *type;
    if (!next_box(&entries, &type, &entry)) return "damaged MP4 sound description";
    bool lossless = !memcmp(type, "alac", 4);
    if ((!lossless && memcmp(type, "mp4a", 4)) || entry.len < 28) return "the MP4 file's sound is neither AAC nor Apple Lossless";
    m->info.channels = (int)be16(entry.p + 16);
    m->info.sample_rate = (int)be16(entry.p + 24);
    /* QuickTime's later versions of the entry are longer. */
    uint32_t version = be16(entry.p + 8);
    size_t fixed = version == 1 ? 28 + 16 : version == 2 ? 28 + 36 : 28;
    if (entry.len < fixed) return "damaged MP4 sound description";
    if (version == 2) {
        /* The short fields are placeholders: the rate is a 64-bit float
           and the channel count a 32-bit number further on. */
        uint64_t bits = be64(entry.p + 32);
        double rate;
        memcpy(&rate, &bits, sizeof(rate));
        uint32_t channels = be32(entry.p + 40);
        m->info.sample_rate = rate >= 1 && rate <= 1e6 ? (int)(rate + 0.5) : 0;
        m->info.channels = channels <= 64 ? (int)channels : 0;
    }
    span inside = {entry.p + fixed, entry.len - fixed};
    if (lossless) {
        if (find_box(inside, "alac", &alac)) return read_alac(alac, m);
        if (find_box(inside, "wave", &wave) && find_box(wave, "alac", &alac)) return read_alac(alac, m);
        return "damaged Apple Lossless settings";
    }
    m->info.codec = AHM4A_AAC;
    m->info.packet_samples = 1024;
    if (find_box(inside, "esds", &esds)) return read_esds(esds, m);
    if (find_box(inside, "wave", &wave) && find_box(wave, "esds", &esds)) return read_esds(esds, m);
    return "damaged MP4 sound description";
}

/* Works out where every packet lies from the sample tables: the size of
   each (stsz), how many each chunk holds (stsc) and where chunks start
   (stco or co64). */
static const char *read_tables(span stbl, ahm4a *m, uint64_t *duration, uint64_t *last)
{
    span stsz, stsc, stco, stts;
    bool wide = false;
    if (!find_box(stbl, "stsz", &stsz) || !find_box(stbl, "stsc", &stsc) || stsz.len < 12 || stsc.len < 8) return "damaged MP4 index";
    if (!find_box(stbl, "stco", &stco)) {
        if (!find_box(stbl, "co64", &stco)) return "damaged MP4 index";
        wide = true;
    }
    if (stco.len < 8) return "damaged MP4 index";
    uint32_t fixed_size = be32(stsz.p + 4), count = be32(stsz.p + 8);
    uint32_t chunks = be32(stco.p + 4), runs = be32(stsc.p + 4);
    if (!fixed_size && (stsz.len - 12) / 4 < count) return "damaged MP4 index";
    if ((stco.len - 8) / (wide ? 8 : 4) < chunks || (stsc.len - 8) / 12 < runs) return "damaged MP4 index";
    if (!count || !chunks || !runs) return "the MP4 file holds no sound";
    /* With one size for all, the count is not tied to the box's length. */
    if (fixed_size && count > m->size / fixed_size) return "damaged MP4 index";

    m->offsets = malloc((size_t)count * sizeof(uint64_t));
    m->sizes = malloc((size_t)count * sizeof(uint32_t));
    if (!m->offsets || !m->sizes) return "out of memory";
    uint32_t packet = 0, run = 0, biggest = 0;
    for (uint32_t chunk = 0; chunk < chunks && packet < count; chunk++) {
        /* Each run of the table says how many packets every chunk from
           its first chunk on holds, until the next run takes over. */
        while (run + 1 < runs && be32(stsc.p + 8 + 12 * (run + 1)) <= chunk + 1) run++;
        uint32_t per_chunk = be32(stsc.p + 8 + 12 * run + 4);
        uint64_t at = wide ? be64(stco.p + 8 + 8 * (size_t)chunk) : be32(stco.p + 8 + 4 * (size_t)chunk);
        for (uint32_t i = 0; i < per_chunk && packet < count; i++, packet++) {
            uint32_t bytes = fixed_size ? fixed_size : be32(stsz.p + 12 + 4 * (size_t)packet);
            if (at > m->size || bytes > m->size - at) return "damaged MP4 index";
            m->offsets[packet] = at;
            m->sizes[packet] = bytes;
            at += bytes;
            if (bytes > biggest) biggest = bytes;
        }
    }
    if (packet < count) return "damaged MP4 index";
    m->info.packets = count;
    m->info.max_packet_bytes = (int)(biggest > 0x7FFFFFFF ? 0x7FFFFFFF : biggest);

    /* The playing time of the packets together, in samples. */
    *duration = *last = 0;
    if (find_box(stbl, "stts", &stts) && stts.len >= 8) {
        uint32_t entries = be32(stts.p + 4);
        if ((stts.len - 8) / 8 < entries) entries = (uint32_t)((stts.len - 8) / 8);
        for (uint32_t i = 0; i < entries; i++) {
            uint32_t times = be32(stts.p + 8 + 8 * (size_t)i), each = be32(stts.p + 12 + 8 * (size_t)i);
            *duration += (uint64_t)times * each;
            if (times) *last = each;
        }
    }
    return NULL;
}

static uint32_t hex_field(const uint8_t *p, size_t len, int field, uint64_t *value)
{
    /* Fields of hex digits, a space before each. */
    size_t at = 0;
    for (int f = 0;; f++) {
        while (at < len && p[at] == ' ') at++;
        size_t start = at;
        uint64_t v = 0;
        while (at < len && p[at] != ' ') {
            uint8_t c = p[at++];
            int digit = c >= '0' && c <= '9' ? c - '0' : c >= 'A' && c <= 'F' ? c - 'A' + 10 : c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
            if (digit < 0) return 0;
            v = v << 4 | (uint64_t)digit;
        }
        if (at == start) return 0;
        if (f == field) {
            *value = v;
            return 1;
        }
    }
}

/* An iTunes gapless tag: text in a '----' box named iTunSMPB, under
   moov/udta/meta/ilst. Its fields are the delay, the padding and the
   length in samples. */
static void read_itunes_gapless(span moov, ahm4a *m)
{
    span udta, meta, ilst, item, name, data;
    const uint8_t *type;
    if (!find_box(moov, "udta", &udta) || !find_box(udta, "meta", &meta) || meta.len < 4) return;
    meta.p += 4;
    meta.len -= 4;
    if (!find_box(meta, "ilst", &ilst)) return;
    while (next_box(&ilst, &type, &item)) {
        if (memcmp(type, "----", 4) || !find_box(item, "name", &name) || !find_box(item, "data", &data)) continue;
        if (name.len != 4 + 8 || memcmp(name.p + 4, "iTunSMPB", 8) || data.len < 8) continue;
        uint64_t delay, samples;
        if (hex_field(data.p + 8, data.len - 8, 1, &delay) && hex_field(data.p + 8, data.len - 8, 3, &samples) && delay < 100000) {
            m->info.delay = (uint32_t)delay;
            m->info.samples = samples;
        }
        return;
    }
}

/* Reads the index. Returns NULL when all is well, else what is wrong. */
static const char *read_index(span moov, ahm4a *m)
{
    span rest = moov, trak, mdia, hdlr, mdhd, minf, stbl, stsd, mvhd, edts, elst;
    const uint8_t *type;
    const char *problem = "the MP4 file has no sound track";
    uint32_t movie_scale = 0;
    if (find_box(moov, "mvhd", &mvhd) && mvhd.len >= 24) movie_scale = be32(mvhd.p + (mvhd.p[0] == 1 ? 20 : 12));
    if (find_box(moov, "mvex", &mvhd)) return "fragmented MP4 files are not supported";
    while (next_box(&rest, &type, &trak)) {
        if (memcmp(type, "trak", 4) || !find_box(trak, "mdia", &mdia)) continue;
        if (!find_box(mdia, "hdlr", &hdlr) || hdlr.len < 12 || memcmp(hdlr.p + 8, "soun", 4)) continue;
        if (!find_box(mdia, "minf", &minf) || !find_box(minf, "stbl", &stbl) || !find_box(stbl, "stsd", &stsd)) continue;
        problem = read_description(stsd, m);
        if (problem) continue; /* another track may be the one */
        uint64_t duration = 0, last = 0;
        problem = read_tables(stbl, m, &duration, &last);
        if (problem) return problem;

        uint32_t media_scale = 0;
        uint64_t media_length = 0;
        if (find_box(mdia, "mdhd", &mdhd) && mdhd.len >= 24) {
            bool wide = mdhd.p[0] == 1;
            media_scale = be32(mdhd.p + (wide ? 20 : 12));
            media_length = wide ? (mdhd.len >= 32 ? be64(mdhd.p + 24) : 0) : be32(mdhd.p + 16);
        }
        m->info.timescale = media_scale ? media_scale : m->info.sample_rate > 0 ? (uint32_t)m->info.sample_rate : 0;
        /* The edit list: the first entry that plays something says where
           in the track the music starts and how long it runs. */
        if (find_box(trak, "edts", &edts) && find_box(edts, "elst", &elst) && elst.len >= 8 && media_scale) {
            bool wide = elst.p[0] == 1;
            size_t each = wide ? 20 : 12;
            uint32_t entries = be32(elst.p + 4);
            for (uint32_t i = 0; i < entries && 8 + each * ((size_t)i + 1) <= elst.len; i++) {
                const uint8_t *e = elst.p + 8 + each * i;
                uint64_t length = wide ? be64(e) : be32(e);
                int64_t start = wide ? (int64_t)be64(e + 8) : (int32_t)be32(e + 4);
                if (start < 0) continue; /* an empty stretch */
                if ((uint64_t)start < duration && start < 100000) {
                    m->info.delay = (uint32_t)start;
                    m->info.samples = duration - (uint64_t)start;
                    /* The entry's length is in the file's time units, which
                       are often too coarse to count samples in; it only cuts
                       the end when it is clearly shorter. */
                    if (movie_scale) {
                        uint64_t limit = length / movie_scale * media_scale + (length % movie_scale + 1) * media_scale / movie_scale;
                        if (limit < m->info.samples) m->info.samples = limit;
                    }
                }
                break;
            }
        }
        if (!m->info.delay && !m->info.samples) read_itunes_gapless(moov, m);
        if (!m->info.samples) {
            /* Neither says how long: the track's own length does, when it
               ends inside the last packet (some encoders round the last
               packet up to a whole one, and ffmpeg cuts it there too). */
            if (media_length < duration && media_length > duration - last) duration = media_length;
            m->info.samples = duration > m->info.delay ? duration - m->info.delay : 0;
        }
        return NULL;
    }
    return problem;
}

ahm4a *ahm4a_open(ahm4a_read_fn read, void *user, uint64_t size, const char **error)
{
    const char *problem = "not an MP4 file";
    uint8_t *moov = NULL;
    bool found = false;
    ahm4a *m = calloc(1, sizeof(*m));
    if (error) *error = NULL;
    if (m) {
        m->read = read;
        m->user = user;
        m->size = size;
        /* Step from box to box at the top of the file until the index. */
        uint64_t at = 0;
        bool known = false;
        while (at + 8 <= size) {
            uint8_t head[16];
            if (read(user, at, head, 8) != 8) break;
            uint64_t box = be32(head);
            uint64_t header = 8;
            if (box == 1) {
                if (read(user, at + 8, head + 8, 8) != 8) break;
                box = be64(head + 8);
                header = 16;
            } else if (box == 0) {
                box = size - at;
            }
            if (box < header || box > size - at) break;
            if (!memcmp(head + 4, "ftyp", 4) || !memcmp(head + 4, "mdat", 4) || !memcmp(head + 4, "free", 4)) known = true;
            if (!memcmp(head + 4, "moov", 4)) {
                size_t body = (size_t)(box - header);
                found = true;
                problem = "damaged MP4 index";
                if (box - header > MAX_INDEX_BYTES) break;
                moov = malloc(body ? body : 1);
                if (!moov) {
                    problem = "out of memory";
                    break;
                }
                if (read(user, at + header, moov, body) != body) break;
                span s = {moov, body};
                problem = read_index(s, m);
                break;
            }
            at += box;
        }
        if (!found && known) problem = "the MP4 file has no index";
    } else {
        problem = "out of memory";
    }
    free(moov);
    if (problem) {
        if (error) *error = problem;
        ahm4a_close(m);
        return NULL;
    }
    return m;
}

void ahm4a_close(ahm4a *m)
{
    if (!m) return;
    free(m->offsets);
    free(m->sizes);
    free(m);
}

const ahm4a_info *ahm4a_get_info(const ahm4a *m)
{
    return &m->info;
}

bool ahm4a_packet(const ahm4a *m, uint64_t index, uint64_t *offset, uint32_t *bytes)
{
    if (index >= m->info.packets) return false;
    *offset = m->offsets[index];
    *bytes = m->sizes[index];
    return true;
}

size_t ahm4a_read_packet(const ahm4a *m, uint64_t index, uint8_t *buffer, size_t room)
{
    uint64_t offset;
    uint32_t bytes;
    if (!ahm4a_packet(m, index, &offset, &bytes) || bytes > room || !bytes) return 0;
    return m->read(m->user, offset, buffer, bytes) == bytes ? bytes : 0;
}
