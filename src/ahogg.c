/* Ogg reader, written from RFC 3533 (the Ogg page format), the Vorbis I
   specification's appendix on Ogg, and the FLAC mapping onto Ogg.

   An Ogg file is a run of pages. Each page belongs to one logical stream
   (its serial number), carries a granule position (for sound, the place
   of the end of the last packet that ends on it; -1 when none does) and a
   list of segment sizes: a size under 255 ends a packet, and a page that
   ends on 255 leaves its last packet to go on in the next page. A checksum
   covers the whole page.

   Streams that play together start with their first pages side by side (a
   group); a chain is one group after another. A stream's first page that
   comes after other pages therefore starts a new link of the chain. */
#include "../include/ahogg.h"
#include <stdlib.h>
#include <string.h>

enum {
    WINDOW = 1 << 17,          /* bytes of the file held at a time; two pages fit */
    PAGE_MAX = 27 + 255 + 255 * 255,
    FIRST_PAGE_WITHIN = 1 << 16, /* the first page must start this near the front */
    PACKET_MAX = 1 << 24,      /* a bigger packet is taken for damage and dropped */
    SEEK_CLOSE = 1 << 16,      /* a seek stops halving at this many bytes */
};

#define NO_PLACE UINT64_MAX

typedef struct {
    uint64_t offset;
    size_t size;
    int segs;
    uint8_t flags; /* 1: goes on from the page before; 2: first page; 4: last page */
    int64_t granule;
    uint32_t serial, seq;
} page_info;

/* One stream of a chain, in a file read at random. */
typedef struct {
    uint64_t begin; /* its first page */
    uint64_t end;   /* where the next link of the chain starts */
    uint32_t serial;
    int64_t last_granule;
} link_info;

struct ahogg {
    ahogg_read_fn read;
    void *user;
    uint64_t size; /* in order: UINT64_MAX until the stream ends */
    bool in_order;
    ahogg_info info;
    uint8_t *header[3];
    size_t header_bytes[3];
    uint64_t first_page; /* the stream's first page */
    uint64_t audio_from; /* pages from here on hold only sound packets */
    uint64_t limit;      /* the stream's pages lie before this */
    link_info *links;    /* read at random: the chain's streams of this kind */
    int nlinks, links_cap;

    uint8_t *window; /* window[0] lies window_off bytes in */
    uint64_t window_off;
    size_t window_len;

    uint8_t *page; /* the page packets are being taken from */
    page_info cur;
    int seg_at;
    size_t body_at;
    int last_end; /* the last segment that ends a packet on this page, or -1 */
    uint64_t next_page;
    uint32_t next_seq;
    bool have_seq;
    bool stream_ended;
    bool past_first; /* a page that is no stream's first has come since this
                        stream's first page: another first page now starts
                        a new link */

    uint8_t *packet;
    size_t packet_len, packet_cap;
    bool partial; /* a packet goes on from the page before */
    bool at_last; /* the packet handed over ended on the stream's last page */
    bool broken;  /* the packet being gathered has lost a piece */
    bool held;    /* the packet in hand is to be handed over again */
    int64_t held_granule;
};

static uint32_t crc_table[256];

static void make_crc_table(void)
{
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t r = i << 24;
        for (int k = 0; k < 8; k++) r = r & 0x80000000 ? r << 1 ^ 0x04C11DB7 : r << 1;
        crc_table[i] = r;
    }
}

static uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

/* ---- reading the file ---------------------------------------------------------- */

/* In order: the window holds the bytes from the earliest still wanted, and
   grows by exactly what is asked for, so that nothing waits on bytes not
   yet needed. Offsets never go back before the window or past its end. */
static const uint8_t *fetch_in_order(ahogg *o, uint64_t off, size_t n)
{
    uint64_t have_end = o->window_off + o->window_len;
    if (off < o->window_off || off > have_end) return NULL;
    if (off + n <= have_end) return o->window + (off - o->window_off);
    if (off > o->size || n > o->size - off) return NULL; /* past the end */
    if (off - o->window_off + n > WINDOW) {
        size_t drop = (size_t)(off - o->window_off);
        memmove(o->window, o->window + drop, o->window_len - drop);
        o->window_off = off;
        o->window_len -= drop;
    }
    while (o->window_len < (size_t)(off - o->window_off) + n) {
        size_t want = (size_t)(off - o->window_off) + n - o->window_len;
        size_t k = o->read(o->user, o->window_off + o->window_len, o->window + o->window_len, want);
        if (!k || k > want) {
            o->size = o->window_off + o->window_len; /* the stream has ended */
            return NULL;
        }
        o->window_len += k;
    }
    return o->window + (off - o->window_off);
}

/* n bytes of the file at off, or NULL past its end. */
static const uint8_t *fetch(ahogg *o, uint64_t off, size_t n)
{
    if (n > WINDOW) return NULL;
    if (o->in_order) return fetch_in_order(o, off, n);
    if (off > o->size || n > o->size - off) return NULL;
    if (off < o->window_off || off + n > o->window_off + o->window_len) {
        o->window_off = off;
        o->window_len = 0;
        size_t want = o->size - off < WINDOW ? (size_t)(o->size - off) : WINDOW;
        while (o->window_len < want) {
            size_t k = o->read(o->user, off + o->window_len, o->window + o->window_len, want - o->window_len);
            if (!k || k > want - o->window_len) break;
            o->window_len += k;
        }
        if (o->window_len < want) o->size = off + o->window_len; /* the file is shorter than said */
        if (o->window_len < n) return NULL;
    }
    return o->window + (off - o->window_off);
}

/* Reads the page at off, if a whole and sound one starts there. */
static const uint8_t *page_at(ahogg *o, uint64_t off, page_info *pg)
{
    const uint8_t *p = fetch(o, off, 27);
    if (!p || memcmp(p, "OggS", 4) || p[4] != 0) return NULL;
    int segs = p[26];
    p = fetch(o, off, 27 + (size_t)segs);
    if (!p) return NULL;
    size_t size = 27 + (size_t)segs;
    for (int i = 0; i < segs; i++) size += p[27 + i];
    p = fetch(o, off, size);
    if (!p) return NULL;
    uint32_t crc = 0;
    for (size_t i = 0; i < size; i++) crc = crc << 8 ^ crc_table[(crc >> 24) ^ (i >= 22 && i < 26 ? 0 : p[i])];
    if (crc != le32(p + 22)) return NULL;
    pg->offset = off;
    pg->size = size;
    pg->segs = segs;
    pg->flags = p[5];
    pg->granule = (int64_t)((uint64_t)le32(p + 6) | (uint64_t)le32(p + 10) << 32);
    if (pg->granule < 0) pg->granule = -1; /* only -1 has a meaning: none */
    pg->serial = le32(p + 14);
    pg->seq = le32(p + 18);
    return p;
}

/* In order: the bytes are looked at one by one, so that none is asked for
   before it is needed. */
static const uint8_t *find_page_in_order(ahogg *o, uint64_t from, uint64_t limit, page_info *pg)
{
    for (uint64_t off = from; off < limit; off++) {
        const uint8_t *w = fetch(o, off, 4);
        if (!w) return NULL;
        if (w[0] != 'O' || memcmp(w, "OggS", 4)) continue;
        const uint8_t *p = page_at(o, off, pg);
        if (p) return p;
    }
    return NULL;
}

/* Finds the first whole page that starts at or after `from` and before
   `limit`. A page that may run past the window is looked at with the
   window moved to start at it, so that checking it moves nothing. */
static const uint8_t *find_page(ahogg *o, uint64_t from, uint64_t limit, page_info *pg)
{
    if (o->in_order) return find_page_in_order(o, from, limit, pg);
    uint64_t off = from;
    while (off < limit && off < o->size) {
        size_t n = o->size - off < WINDOW ? (size_t)(o->size - off) : WINDOW;
        const uint8_t *w = fetch(o, off, n);
        if (!w) return NULL;
        size_t i;
        bool moved = false;
        for (i = 0; i + 4 <= n && off + i < limit; i++) {
            if (w[i] != 'O' || memcmp(w + i, "OggS", 4)) continue;
            if (i && off + n < o->size) {
                size_t need = PAGE_MAX;
                if (i + 27 <= n && i + 27 + (size_t)w[i + 26] <= n) {
                    need = 27 + (size_t)w[i + 26];
                    for (int k = 0; k < w[i + 26]; k++) need += w[i + 27 + (size_t)k];
                }
                if (i + need > n) {
                    off += i;
                    moved = true;
                    break;
                }
            }
            const uint8_t *p = page_at(o, off + i, pg);
            if (p) return p;
        }
        if (moved) continue;
        if (off + n >= o->size || !i) return NULL; /* the end of the file */
        off += i; /* the last few bytes may start a page */
    }
    return NULL;
}

/* The kind of stream a first page starts, or -1 for one not handled. */
static int codec_of(const uint8_t *p, const page_info *pg)
{
    const uint8_t *body = p + 27 + pg->segs;
    size_t first = pg->segs ? p[27] : 0;
    if (first >= 30 && !memcmp(body, "\x01vorbis", 7)) return AHOGG_VORBIS;
    if (first >= 51 && !memcmp(body, "\x7F" "FLAC", 5)) return AHOGG_FLAC;
    if (first >= 19 && !memcmp(body, "OpusHead", 8)) return AHOGG_OPUS;
    return -1;
}

/* ---- packets --------------------------------------------------------------------- */

static void start_over(ahogg *o, uint64_t off)
{
    o->next_page = off;
    o->seg_at = o->cur.segs = 0;
    o->packet_len = 0;
    o->partial = o->broken = false;
    o->have_seq = false;
    o->stream_ended = false;
    o->past_first = false;
    o->held = false;
}

/* Makes the page just found (at p) the one packets are taken from. */
static void take_page(ahogg *o, const uint8_t *p, const page_info *pg)
{
    /* A page lost on the way breaks the packet that was going on. */
    if (o->have_seq && pg->seq != o->next_seq) o->broken = o->partial;
    o->next_seq = pg->seq + 1;
    o->have_seq = true;
    if (pg->flags & 1) {
        if (!o->partial) o->broken = true; /* the start of this piece is not here */
    } else if (o->partial) {
        o->packet_len = 0; /* the packet before never ended */
        o->partial = o->broken = false;
    }
    memcpy(o->page, p, pg->size);
    o->cur = *pg;
    o->seg_at = 0;
    o->body_at = 27 + (size_t)pg->segs;
    o->last_end = -1;
    for (int i = 0; i < pg->segs; i++)
        if (p[27 + i] < 255) o->last_end = i;
    if (pg->flags & 4) o->stream_ended = true;
}

/* Moves on to the stream's next page. False at the end of the stream. */
static bool next_page(ahogg *o)
{
    for (;;) {
        if (o->stream_ended) return false;
        page_info pg;
        const uint8_t *p = find_page(o, o->next_page, o->limit, &pg);
        if (!p) return false;
        if ((pg.flags & 2) && o->past_first) {
            /* The next link of a chain: this stream has ended, without a
               last page. Its first page is left to be found again. */
            o->stream_ended = true;
            return false;
        }
        o->next_page = pg.offset + pg.size;
        if (!(pg.flags & 2)) o->past_first = true;
        if (pg.serial != o->info.serial) continue;
        if (pg.offset < o->first_page) continue;
        take_page(o, p, &pg);
        return true;
    }
}

/* The next packet of the stream, headers included. */
static bool any_packet(ahogg *o, const uint8_t **data, size_t *bytes, int64_t *granule)
{
    if (o->held) {
        o->held = false;
        *data = o->packet;
        *bytes = o->packet_len;
        *granule = o->held_granule;
        return true;
    }
    for (;;) {
        if (o->seg_at >= o->cur.segs) {
            if (!next_page(o)) return false;
            continue;
        }
        if (!o->partial) {
            o->packet_len = 0;
            o->broken = o->broken && (o->cur.flags & 1) && o->seg_at == 0;
        }
        bool done = false;
        while (o->seg_at < o->cur.segs && !done) {
            size_t n = o->page[27 + o->seg_at];
            if (!o->broken) {
                if (o->packet_len + n > PACKET_MAX) {
                    o->broken = true;
                } else {
                    if (o->packet_len + n > o->packet_cap) {
                        size_t cap = o->packet_cap ? o->packet_cap : 1 << 16;
                        while (cap < o->packet_len + n) cap *= 2;
                        uint8_t *grown = realloc(o->packet, cap);
                        if (!grown) {
                            o->broken = true;
                        } else {
                            o->packet = grown;
                            o->packet_cap = cap;
                        }
                    }
                    if (!o->broken) {
                        memcpy(o->packet + o->packet_len, o->page + o->body_at, n);
                        o->packet_len += n;
                    }
                }
            }
            o->body_at += n;
            done = n < 255;
            o->seg_at++;
        }
        if (!done) {
            o->partial = true;
            continue;
        }
        bool keep = !o->broken;
        int64_t g = o->seg_at - 1 == o->last_end ? o->cur.granule : -1;
        o->partial = o->broken = false;
        if (!keep) continue;
        *data = o->packet;
        *bytes = o->packet_len;
        *granule = g;
        o->at_last = (o->cur.flags & 4) != 0;
        return true;
    }
}

bool ahogg_packet(ahogg *o, const uint8_t **data, size_t *bytes, int64_t *granule)
{
    return any_packet(o, data, bytes, granule);
}

bool ahogg_last_page(const ahogg *o)
{
    return o->at_last;
}

/* ---- headers ----------------------------------------------------------------------- */

static void keep_header(ahogg *o, int index, const uint8_t *p, size_t n)
{
    if (index >= 3) return;
    o->header[index] = malloc(n ? n : 1);
    if (o->header[index]) {
        memcpy(o->header[index], p, n);
        o->header_bytes[index] = n;
    }
}

static void drop_headers(ahogg *o)
{
    for (int i = 0; i < 3; i++) {
        free(o->header[i]);
        o->header[i] = NULL;
        o->header_bytes[i] = 0;
    }
    o->info.headers = 0;
}

/* Reads the header packets of the stream whose first page is in hand:
   three for Vorbis, two for Opus; for FLAC the first packet says how many
   follow it (0: until the one marked last). Returns what went wrong, or
   NULL; reading then goes on with the first sound packet. */
static const char *read_headers(ahogg *o)
{
    drop_headers(o);
    const uint8_t *data;
    size_t bytes;
    int64_t granule;
    int want = o->info.codec == AHOGG_OPUS ? 2 : 3;
    uint64_t after = 0; /* the end of the page the last header ends on */
    for (int i = 0; i < want; i++) {
        if (!any_packet(o, &data, &bytes, &granule)) return "the Ogg file's headers are damaged";
        if (o->info.codec == AHOGG_FLAC && i && bytes && data[0] == 0xFF) {
            /* A frame: fewer headers than the first packet said (or it
               did not say). They end here, and the frame is sound. */
            o->held = true;
            o->held_granule = granule;
            break;
        }
        keep_header(o, i, data, bytes);
        o->info.headers = i + 1;
        after = o->cur.offset + o->cur.size;
        if (o->info.codec == AHOGG_FLAC) {
            if (i == 0) {
                if (bytes < 51 || data[5] != 1 || memcmp(data + 9, "fLaC", 4)) return "an Ogg FLAC stream of an unknown kind";
                int count = data[7] << 8 | data[8];
                want = count ? 1 + count : 1 << 20;
                if (data[13] & 0x80) want = 1; /* the stream information was the only block */
            } else if (want == 1 << 20 && bytes && (data[0] & 0x80)) {
                want = i + 1;
            }
        }
    }
    /* Pages after the one the last header ends on hold only sound. */
    o->audio_from = after;
    if (o->info.codec == AHOGG_VORBIS && (!o->header[0] || !o->header[2])) return "out of memory";
    if (!o->header[0]) return "out of memory";
    return NULL;
}

/* ---- opening --------------------------------------------------------------------- */

/* The stream's last granule position: walk the pages near the end of the
   file, going further back until one of the stream's is found. */
static int64_t find_last_granule(ahogg *o)
{
    uint64_t end = o->size;
    for (uint64_t step = 1 << 16; end > o->first_page; step *= 2) {
        uint64_t from = end - o->first_page > step ? end - step : o->first_page;
        int64_t found = -1;
        page_info pg;
        for (uint64_t off = from; find_page(o, off, end, &pg); off = pg.offset + pg.size)
            if (pg.serial == o->info.serial && pg.granule != -1) found = pg.granule;
        if (found != -1) return found;
        end = from;
    }
    return -1;
}

/* The serial number of the file's last page; false when none is found. */
static bool last_serial(ahogg *o, uint32_t *serial)
{
    uint64_t end = o->size;
    for (uint64_t step = 1 << 16; end > o->first_page; step *= 2) {
        uint64_t from = end - o->first_page > step ? end - step : o->first_page;
        bool found = false;
        page_info pg;
        for (uint64_t off = from; find_page(o, off, end, &pg); off = pg.offset + pg.size) {
            *serial = pg.serial;
            found = true;
        }
        if (found) return true;
        end = from;
    }
    return false;
}

static bool add_link(ahogg *o, const link_info *l)
{
    if (o->nlinks == o->links_cap) {
        int cap = o->links_cap ? o->links_cap * 2 : 4;
        link_info *grown = realloc(o->links, (size_t)cap * sizeof(link_info));
        if (!grown) return false;
        o->links = grown;
        o->links_cap = cap;
    }
    o->links[o->nlinks++] = *l;
    return true;
}

/* A chained file: walk its pages from `from` (where the first group
   starts) and note each link that holds a stream of this kind. */
static bool walk_links(ahogg *o, uint64_t from)
{
    link_info cur = {0};
    bool open = false, chosen = false, in_group = false;
    page_info pg;
    const uint8_t *p;
    for (uint64_t off = from; (p = find_page(o, off, o->size, &pg)) != NULL; off = pg.offset + pg.size) {
        if (pg.flags & 2) {
            if (!in_group) {
                /* A new group: the link before ends here. */
                if (chosen) {
                    cur.end = pg.offset;
                    if (!add_link(o, &cur)) return false;
                }
                open = true;
                chosen = false;
                in_group = true;
            }
            if (!chosen && codec_of(p, &pg) == (int)o->info.codec) {
                chosen = true;
                cur.begin = pg.offset;
                cur.serial = pg.serial;
                cur.last_granule = -1;
            }
        } else {
            in_group = false;
            if (chosen && pg.serial == cur.serial && pg.granule != -1) cur.last_granule = pg.granule;
        }
    }
    if (open && chosen) {
        cur.end = o->size;
        if (!add_link(o, &cur)) return false;
    }
    return true;
}

static ahogg *new_ogg(ahogg_read_fn read, void *user, uint64_t size, bool in_order)
{
    if (!crc_table[1]) make_crc_table();
    ahogg *o = calloc(1, sizeof(*o));
    if (o) {
        o->window = malloc(WINDOW);
        o->page = malloc(PAGE_MAX);
    }
    if (!o || !o->window || !o->page) {
        ahogg_close(o);
        return NULL;
    }
    o->read = read;
    o->user = user;
    o->size = size;
    o->limit = size;
    o->in_order = in_order;
    o->info.last_granule = -1;
    return o;
}

static ahogg *failed(ahogg *o, const char *why, const char **error)
{
    if (error) *error = why;
    ahogg_close(o);
    return NULL;
}

ahogg *ahogg_open(ahogg_read_fn read, void *user, uint64_t size, const char **error)
{
    ahogg *o = new_ogg(read, user, size, false);
    if (!o) return failed(o, "out of memory", error);

    /* The streams' first pages come first; take the first one whose sound
       can be decoded. */
    page_info pg;
    const uint8_t *p = find_page(o, 0, FIRST_PAGE_WITHIN, &pg);
    uint64_t group = p ? pg.offset : 0;
    bool found = false;
    while (p && (pg.flags & 2) && !found) {
        int codec = codec_of(p, &pg);
        if (codec >= 0) {
            o->info.codec = (ahogg_codec)codec;
            o->info.serial = pg.serial;
            o->first_page = pg.offset;
            found = true;
        } else {
            p = find_page(o, pg.offset + pg.size, o->size, &pg);
        }
    }
    if (!found) return failed(o, !p ? "not an Ogg file" : "the Ogg file holds no Vorbis, FLAC or Opus sound", error);
    start_over(o, o->first_page);
    const char *why = read_headers(o);
    if (why) return failed(o, why, error);

    /* The group's streams. When the file's last page belongs to none of
       them, the file is chained: walk it for its links. */
    uint32_t group_serials[64];
    int nserials = 0;
    for (uint64_t off = group; nserials < 64 && find_page(o, off, o->size, &pg) && (pg.flags & 2); off = pg.offset + pg.size)
        group_serials[nserials++] = pg.serial;
    uint32_t last;
    bool chained = last_serial(o, &last);
    for (int i = 0; i < nserials && chained; i++) chained = group_serials[i] != last;
    if (chained) {
        if (!walk_links(o, group)) return failed(o, "out of memory", error);
        /* The walk finds the stream taken above first. */
        chained = o->nlinks > 1 && o->links[0].begin == o->first_page;
    }
    if (!chained) {
        link_info only = {o->first_page, o->size, o->info.serial, find_last_granule(o)};
        o->nlinks = 0;
        if (!add_link(o, &only)) return failed(o, "out of memory", error);
    }
    o->limit = o->links[0].end;
    o->info.last_granule = o->links[0].last_granule;
    o->info.streams = o->nlinks;
    ahogg_seek(o, -1); /* back to the first sound packet */
    if (error) *error = NULL;
    return o;
}

/* In order: passes over pages until the first page of a stream of kind
   `want` (any handled when negative), and reads that stream's headers.
   With `need_sound`, a first page counts only after a page that is no
   stream's first: a new group, not another stream of the group in hand. */
static bool next_in_order(ahogg *o, int want, bool need_sound, const char **why)
{
    page_info pg;
    const uint8_t *p;
    *why = NULL;
    while ((p = find_page(o, o->next_page, o->limit, &pg)) != NULL) {
        o->next_page = pg.offset + pg.size;
        if (!(pg.flags & 2)) {
            need_sound = false;
            continue;
        }
        int codec = codec_of(p, &pg);
        if (need_sound || codec < 0 || (want >= 0 && codec != want)) continue;
        o->info.codec = (ahogg_codec)codec;
        o->info.serial = pg.serial;
        o->first_page = pg.offset;
        start_over(o, o->next_page);
        take_page(o, p, &pg);
        *why = read_headers(o);
        if (!*why) return true;
        /* Damaged headers: look on from here (bytes are not read twice). */
        start_over(o, o->next_page);
        need_sound = true;
    }
    return false;
}

ahogg *ahogg_open_stream(ahogg_read_fn read, void *user, const char **error)
{
    ahogg *o = new_ogg(read, user, UINT64_MAX, true);
    if (!o) return failed(o, "out of memory", error);
    /* A stream joined part way starts in the middle of a page. */
    page_info pg;
    if (!find_page(o, 0, FIRST_PAGE_WITHIN + PAGE_MAX, &pg)) return failed(o, "not an Ogg stream", error);
    o->next_page = pg.offset;
    const char *why = NULL;
    if (!next_in_order(o, -1, false, &why))
        return failed(o, why ? why : "the Ogg stream ended before a Vorbis, FLAC or Opus stream started", error);
    if (error) *error = NULL;
    return o;
}

void ahogg_close(ahogg *o)
{
    if (!o) return;
    for (int i = 0; i < 3; i++) free(o->header[i]);
    free(o->links);
    free(o->window);
    free(o->page);
    free(o->packet);
    free(o);
}

const ahogg_info *ahogg_get_info(const ahogg *o)
{
    return &o->info;
}

bool ahogg_header(const ahogg *o, int index, const uint8_t **data, size_t *bytes)
{
    if (index < 0 || index >= 3 || index >= o->info.headers || !o->header[index]) return false;
    *data = o->header[index];
    *bytes = o->header_bytes[index];
    return true;
}

/* ---- chains ------------------------------------------------------------------------- */

/* Read at random: moves to link `index`. */
static bool select_link(ahogg *o, int index)
{
    const link_info *l = &o->links[index];
    o->info.serial = l->serial;
    o->info.stream = index;
    o->info.last_granule = l->last_granule;
    o->first_page = l->begin;
    o->limit = l->end;
    start_over(o, l->begin);
    return !read_headers(o); /* reading goes on with the first sound packet */
}

bool ahogg_select_stream(ahogg *o, int index)
{
    if (o->in_order || index < 0 || index >= o->nlinks) return false;
    int was = o->info.stream;
    if (select_link(o, index)) return true;
    select_link(o, was); /* it was read before */
    return false;
}

bool ahogg_next_stream(ahogg *o)
{
    if (!o->in_order) {
        for (int k = o->info.stream + 1; k < o->nlinks; k++)
            if (ahogg_select_stream(o, k)) return true;
        /* None: reading stays at the end, not back at this stream's start. */
        o->held = false;
        o->stream_ended = true;
        o->seg_at = o->cur.segs;
        o->partial = false;
        return false;
    }
    /* What is left of this stream goes, up to the next group. */
    const char *why;
    if (!next_in_order(o, (int)o->info.codec, !o->past_first, &why)) return false;
    o->info.stream++;
    return true;
}

/* ---- seeking ----------------------------------------------------------------------- */

/* Walks the pages from `from` while their granule positions stay at or
   below `target`, and gives the best place to start reading: a page on
   which a packet starts and ends, or else the page on which the packet
   that ends on such a page started. The first granule position handed
   over from there is then that page's. NO_PLACE when there is none. */
static uint64_t seek_scan(ahogg *o, uint64_t from, int64_t target)
{
    uint64_t best = NO_PLACE, last_start = NO_PLACE;
    page_info pg;
    for (uint64_t off = from; find_page(o, off, o->limit, &pg); off = pg.offset + pg.size) {
        if (pg.serial != o->info.serial) continue;
        if (pg.granule > target) break;
        const uint8_t *p = fetch(o, pg.offset, pg.size);
        if (!p) break;
        bool continued = pg.flags & 1, starts = !continued && pg.segs > 0;
        int ends = 0;
        for (int i = 0; i < pg.segs; i++)
            if (p[27 + i] < 255) {
                ends++;
                if (i + 1 < pg.segs) starts = true;
            }
        if (pg.granule != -1) {
            if (ends > (continued ? 1 : 0)) best = pg.offset;
            else if (last_start != NO_PLACE) best = last_start;
        }
        if (starts) last_start = pg.offset;
        if (pg.flags & 4) break;
    }
    return best;
}

/* The granule position of the first of the stream's pages that has one,
   from `from` on and starting before `limit`; -2 when there is none. */
static int64_t granule_after(ahogg *o, uint64_t from, uint64_t limit)
{
    page_info pg;
    for (uint64_t off = from; find_page(o, off, limit, &pg); off = pg.offset + pg.size)
        if (pg.serial == o->info.serial && pg.granule != -1) return pg.granule;
    return -2;
}

bool ahogg_seek(ahogg *o, int64_t granule)
{
    if (o->in_order) return false;
    uint64_t place = NO_PLACE, end = o->limit < o->size ? o->limit : o->size;
    if (granule >= 0 && o->audio_from && o->audio_from < end) {
        /* Halve the stretch of file the place lies in, then walk. */
        uint64_t lo = o->audio_from, hi = end;
        while (hi - lo > SEEK_CLOSE) {
            uint64_t mid = lo + (hi - lo) / 2;
            int64_t g = granule_after(o, mid, hi);
            if (g >= 0 && g <= granule) lo = mid;
            else hi = mid;
        }
        place = seek_scan(o, lo, granule);
        /* No place to start from there: look further back. */
        for (uint64_t back = SEEK_CLOSE * 4; place == NO_PLACE && lo > o->audio_from; back *= 4) {
            lo = lo - o->audio_from > back ? lo - back : o->audio_from;
            place = seek_scan(o, lo, granule);
        }
    }
    if (place != NO_PLACE) {
        start_over(o, place);
        o->past_first = true;
        return true;
    }
    /* From the start: past the headers again. */
    start_over(o, o->first_page);
    const uint8_t *data;
    size_t bytes;
    int64_t g;
    for (int i = 0; i < o->info.headers; i++)
        if (!any_packet(o, &data, &bytes, &g)) break;
    return false;
}
