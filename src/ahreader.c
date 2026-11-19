/* One reader over every decoder: tells the kind of file, decodes it a piece
   at a time, trims gapless files, and seeks. */
#include "../include/ahreader.h"
#include "../include/ahaac.h"
#include "../include/ahalac.h"
#include "../include/ahflac.h"
#include "../include/ahm4a.h"
#include "../include/ahmp3.h"
#include "../include/ahogg.h"
#include "../include/ahvorbis.h"
#include "../include/ahopus.h"
#include "../include/ahwav.h"
#include <stdlib.h>
#include <string.h>

enum {
    IN_START = 1 << 16,   /* the window on the file, to begin with */
    IN_MAX = 1 << 26,     /* and the most it may grow to hold one frame */
    PCM_FRAMES = 4096,    /* samples per channel decoded per round */
    HOLD_MAX = 4096,      /* most samples kept back for an MP3 file's padding */
    SCAN = 1 << 16,       /* bytes read at a time when walking frame headers */
    JUNK_MAX = 1 << 22,   /* bytes that may come before the first frame */
    M4A_PACKET_MAX = 1 << 14, /* bytes of the biggest M4A packet read (AAC allows 768 per channel) */
    OGG_HOLD_MAX = 1 << 18,   /* samples held, read in order, until a page says where they lie */
};

/* The shape of the sound of one Ogg stream. */
typedef struct {
    int rate, channels, bits;
    uint32_t mask;
} ogg_shape;

/* One stream of a chained Ogg file read at random. */
typedef struct {
    bool ok;         /* it can be played */
    ogg_shape shape;
    int64_t first;   /* where the first sound decoded from its start lies */
    int64_t start;   /* where the sound played starts */
    uint64_t play;   /* samples it plays; UINT64_MAX when not known */
    uint64_t before; /* samples of the streams played before it */
} ogg_stream;

struct ahreader {
    ahreader_read_fn read;
    void *user;
    uint64_t size;
    uint64_t base; /* where the file proper starts, past a tag in front */
    ahreader_info info;

    uint8_t *in; /* a window on the file: in[0] lies in_off bytes in */
    size_t in_cap, in_len, in_pos;
    uint64_t in_off;
    uint64_t junk_left; /* bytes that may still pass without sound */

    float *pcm; /* decoded, not yet given out */
    size_t pcm_cap, pcm_len, pcm_pos; /* in samples per channel */
    uint64_t skip; /* samples to drop before the next one given out */
    uint64_t left; /* samples still to give, when the file says */
    size_t hold;   /* samples kept back until the end is known: they are
                      dropped there (the padding of an MP3 file) */
    bool ended;

    ahmp3_decoder *mp3;
    ahaac_decoder *aac;
    ahflac_decoder *flac;
    ahwav_decoder *wav;
    ahalac_decoder *alac;
    ahvorbis_decoder *vorbis;
    ahopus_decoder *opus;
    ahm4a *m4a;
    ahogg *ogg;
    bool ogg_lost;      /* Ogg: after a seek, where the sound decoded lies is
                           not known until a page states it */
    int64_t ogg_first;  /* where the first sound decoded from the start lies */
    int64_t ogg_start;  /* where the sound played starts */
    int64_t ogg_target; /* where a seek is to land */
    int64_t opus_pre_skip; /* Opus: samples the header says to drop at the start */
    bool in_order;      /* opened with ahreader_open_stream */
    bool follow;        /* ahreader_follow_changes */
    bool ogg_changed;   /* the decoder is at a stream the info does not
                           describe yet: it does from the next read on */
    bool ogg_over;      /* no further stream of a chain is to be played */
    bool ogg_waiting;   /* the sound ended at a stream of another shape;
                           ahreader_follow_changes lets it go on */
    ogg_shape dec;      /* the sound the decoder gives */
    size_t ogg_packet_max; /* samples per channel in the biggest packet */
    int64_t ogg_place;  /* in order: the granule position at the end of the
                           sound decoded; -1 until a page states one */
    size_t ogg_held;    /* in order: samples decoded before then, kept in
                           pcm after pcm_len */
    ogg_stream *streams; /* read at random: the chain's streams */
    int nstreams, nfollowed, ogg_cur;
    uint8_t *comments;  /* the comment header of the stream shown */
    size_t comments_bytes;
    int32_t *ints;       /* FLAC's and ALAC's whole numbers */
    uint8_t *packet;     /* one M4A packet */
    size_t packet_cap;
    uint64_t next_packet;
    uint64_t packet_samples; /* M4A: per channel in a full packet */
    uint64_t play_total; /* M4A: samples to play in all */

    uint64_t first_frame; /* where the first frame of sound starts */
    int frame_samples;    /* samples per frame (MPEG audio, ADTS) */
    uint64_t lead;        /* samples the decoder gives before the music starts */
    bool no_walk;         /* frames do not state their size (free format) */

    uint8_t *scan; /* a second window, for walking frame headers */
    uint64_t scan_off;
    size_t scan_len;
};

/* ---- reading the file ---------------------------------------------------------- */

static size_t read_at(ahreader *r, uint64_t off, void *buf, size_t n)
{
    size_t got = 0;
    while (got < n && off + got < r->size) {
        size_t want = n - got;
        if (want > r->size - off - got) want = (size_t)(r->size - off - got);
        size_t k = r->read(r->user, off + got, (uint8_t *)buf + got, want);
        if (!k || k > want) break;
        got += k;
    }
    return got;
}

static void set_window(ahreader *r, uint64_t off)
{
    r->in_off = off;
    r->in_len = r->in_pos = 0;
}

/* Brings more of the file into the window. False when the window cannot
   take a frame that big. */
static bool refill(ahreader *r)
{
    if (r->in_pos) {
        memmove(r->in, r->in + r->in_pos, r->in_len - r->in_pos);
        r->in_off += r->in_pos;
        r->in_len -= r->in_pos;
        r->in_pos = 0;
    }
    if (r->in_len == r->in_cap) {
        if (r->in_cap >= IN_MAX) return false;
        uint8_t *grown = realloc(r->in, r->in_cap * 2);
        if (!grown) return false;
        r->in = grown;
        r->in_cap *= 2;
    }
    size_t n = read_at(r, r->in_off + r->in_len, r->in + r->in_len, r->in_cap - r->in_len);
    /* A file shorter than was said ends where reading stops. */
    if (!n) r->size = r->in_off + r->in_len;
    r->in_len += n;
    return true;
}

/* A few bytes of the file at any place, for walking frame headers. */
static const uint8_t *peek(ahreader *r, uint64_t off, size_t n)
{
    if (off > r->size || n > r->size - off) return NULL;
    if (off < r->scan_off || off + n > r->scan_off + r->scan_len) {
        if (!r->scan) r->scan = malloc(SCAN);
        if (!r->scan) return NULL;
        r->scan_off = off;
        r->scan_len = read_at(r, off, r->scan, SCAN);
        if (r->scan_len < n) return NULL;
    }
    return r->scan + (off - r->scan_off);
}

/* ---- decoding -------------------------------------------------------------------- */

/* Takes in `n` samples per channel just decoded to the end of pcm. */
static void append(ahreader *r, size_t n)
{
    size_t channels = (size_t)r->info.channels;
    float *at = r->pcm + r->pcm_len * channels;
    if (r->skip) {
        size_t drop = r->skip < n ? (size_t)r->skip : n;
        memmove(at, at + drop * channels, (n - drop) * channels * sizeof(float));
        r->skip -= drop;
        n -= drop;
    }
    r->pcm_len += n;
}

/* Decodes the next packet of an M4A file. A packet that cannot be read or
   decoded plays as silence, so that the sound after it keeps its place. */
static bool decode_packet(ahreader *r)
{
    const ahm4a_info *info = ahm4a_get_info(r->m4a);
    if (r->next_packet >= info->packets) return false;
    size_t channels = (size_t)r->info.channels, frame = (size_t)r->packet_samples;
    float *at = r->pcm + r->pcm_len * channels;
    int n = 0;
    size_t bytes = ahm4a_read_packet(r->m4a, r->next_packet++, r->packet, r->packet_cap);
    if (r->alac) {
        if (bytes && ahalac_decode_packet(r->alac, r->packet, bytes, r->ints, frame * channels, &n) && n) {
            ahalac_to_float(r->ints, at, (size_t)n * channels, r->info.bits);
            frame = (size_t)n;
        } else {
            memset(at, 0, frame * channels * sizeof(float));
        }
        append(r, frame);
        return true;
    }
    ahaac_frame fr;
    if (!bytes || !ahaac_decode_packet(r->aac, r->packet, bytes, at, &n, &fr) || !n || fr.channels != r->info.channels
        || fr.sample_rate != r->info.sample_rate)
        memset(at, 0, (size_t)AHAAC_MAX_SAMPLES * channels * sizeof(float));
    append(r, AHAAC_MAX_SAMPLES);
    return true;
}

/* Decodes one FLAC frame that came as an Ogg packet. Returns the samples
   per channel it gave, as floats at `at`, which has room for `room`. */
static size_t ogg_flac_frame(ahreader *r, const uint8_t *data, size_t bytes, float *at, size_t room)
{
    size_t channels = (size_t)r->dec.channels, total = 0, pos = 0;
    if (room > r->ogg_packet_max) room = r->ogg_packet_max; /* what ints holds */
    for (;;) {
        int n = 0;
        size_t used = ahflac_decode(r->flac, data + pos, bytes - pos, true, r->ints + total * channels, (room - total) * channels, &n, NULL);
        pos += used;
        total += (size_t)n;
        if ((!used && !n) || total == room) break;
    }
    ahflac_to_float(r->ints, at, total * channels, r->dec.bits);
    return total;
}

/* Where the first sound decoded from an Ogg stream's start lies: what is
   decoded up to the first page that states a place (`granule`, from a page
   that `last` says ends the stream), `got` samples, ends there. A stream
   whose first such page is also its last states only where it ends; Opus
   then still works back from it when it states more than it holds (RFC
   7845 section 4.5). No such page: 0. */
static int64_t ogg_first_place(bool opus, int64_t granule, int64_t got, bool last)
{
    if (granule < 0) return 0;
    if (!last || (opus && granule > got)) return granule - got;
    return 0;
}

/* Where the sound played starts: not before 0, and for Opus past the
   header's pre-skip. */
static int64_t ogg_start_place(int64_t first, bool opus, int64_t pre_skip)
{
    int64_t start = first > 0 ? first : 0;
    if (opus && first + pre_skip > start) start = first + pre_skip;
    return start;
}

/* In order: room in pcm for one more packet after those held. False when
   no more may be held. */
static bool ogg_room(ahreader *r)
{
    size_t need = r->pcm_len + r->ogg_held + r->ogg_packet_max;
    if (need <= r->pcm_cap) return true;
    if (r->ogg_held >= OGG_HOLD_MAX) return false;
    size_t cap = r->pcm_cap * 2 > need ? r->pcm_cap * 2 : need;
    float *grown = realloc(r->pcm, cap * (size_t)r->dec.channels * sizeof(float));
    if (!grown) return false;
    r->pcm = grown;
    r->pcm_cap = cap;
    return true;
}

/* In order: where the sound held lies is now known from a page (granule;
   `last`: the stream's last page), or never will be (-1). What lies before
   the start goes, and after the end; the rest is let out. */
static void ogg_settle(ahreader *r, int64_t granule, bool last)
{
    bool opus = r->opus != NULL;
    int64_t got = (int64_t)r->ogg_held;
    int64_t first = ogg_first_place(opus, granule, got, last);
    int64_t start = ogg_start_place(first, opus, r->opus_pre_skip);
    int64_t end = first + got;
    if (granule >= 0 && last && granule < end) end = granule;
    int64_t from = start - first, to = end - first;
    size_t channels = (size_t)r->dec.channels;
    if (from < to) {
        size_t kept = (size_t)(to - from);
        memmove(r->pcm + r->pcm_len * channels, r->pcm + (r->pcm_len + (size_t)from) * channels, kept * channels * sizeof(float));
        r->pcm_len += kept;
    }
    if (from > got) r->skip = (uint64_t)(from - got); /* the start lies further on */
    r->ogg_held = 0;
    r->ogg_place = granule >= 0 ? granule : first + got;
}

/* In order: lets out the first `keep` of the samples held from a stream's
   last page; the rest go. */
static void ogg_let_out(ahreader *r, size_t keep)
{
    size_t held = r->ogg_held;
    r->ogg_held = 0;
    append(r, keep < held ? keep : held);
}

/* Decodes the next packet of an Ogg file. After a seek the place of the
   sound is not known until a page states it; what comes before goes. Read
   in order, the sound at a stream's start is held until a page says where
   it lies, and the stream's last page says where it ends. */
static bool decode_ogg(ahreader *r)
{
    const uint8_t *data;
    size_t bytes;
    int64_t granule;
    size_t channels = (size_t)r->dec.channels;
    bool holding = r->in_order && r->ogg_place < 0;
    if (r->in_order && r->ogg_held && !ogg_room(r)) {
        if (holding) ogg_settle(r, -1, false);
        else ogg_let_out(r, r->ogg_held);
        return true;
    }
    if (!ahogg_packet(r->ogg, &data, &bytes, &granule)) {
        /* A stream that ends before a page says where its sound lies or
           ends plays what was held. */
        if (r->in_order && r->ogg_held) {
            if (holding) ogg_settle(r, -1, false);
            else ogg_let_out(r, r->ogg_held);
            return true;
        }
        return false;
    }
    float *at = r->pcm + (r->pcm_len + r->ogg_held) * channels;
    int n = 0;
    if (r->vorbis) ahvorbis_decode_packet(r->vorbis, data, bytes, at, &n);
    else if (r->opus) ahopus_decode_packet(r->opus, data, bytes, at, &n);
    else n = (int)ogg_flac_frame(r, data, bytes, at, r->pcm_cap - r->pcm_len - r->ogg_held);
    if (r->ogg_lost) {
        if (granule >= 0) {
            r->ogg_lost = false;
            r->skip = r->ogg_target > granule ? (uint64_t)(r->ogg_target - granule) : 0;
        }
        return true;
    }
    if (!r->in_order) {
        append(r, (size_t)n);
        return true;
    }
    bool last = ahogg_last_page(r->ogg);
    if (holding) {
        r->ogg_held += (size_t)n;
        if (granule >= 0) ogg_settle(r, granule, last);
        return true;
    }
    if (last) {
        /* The stream's last page: its packets are held until the one that
           says where the sound ends, which may cut more than one. */
        r->ogg_held += (size_t)n;
        if (granule < 0) return true;
        int64_t keep = granule - r->ogg_place;
        r->ogg_place = granule;
        ogg_let_out(r, keep > 0 ? (size_t)keep : 0);
        return true;
    }
    r->ogg_place = granule >= 0 ? granule : r->ogg_place + n;
    append(r, (size_t)n);
    return true;
}

/* Decodes a little more into pcm. False at the end of the file. `fr_mp3`
   and `fr_aac` (optional) get the frame that gave sound. */
static bool decode_more(ahreader *r, ahmp3_frame *fr_mp3, ahaac_frame *fr_aac)
{
    if (r->pcm_pos) {
        size_t channels = (size_t)(r->info.channels ? r->info.channels : AHFLAC_MAX_CHANNELS);
        memmove(r->pcm, r->pcm + r->pcm_pos * channels, (r->pcm_len - r->pcm_pos) * channels * sizeof(float));
        r->pcm_len -= r->pcm_pos;
        r->pcm_pos = 0;
    }
    if (r->m4a) return decode_packet(r);
    if (r->ogg) return decode_ogg(r);

    for (;;) {
        const uint8_t *data = r->in + r->in_pos;
        size_t have = r->in_len - r->in_pos, used = 0, room = r->pcm_cap - r->pcm_len;
        bool eof = r->in_off + r->in_len >= r->size;
        /* Before the first frame the channel count is not known: the buffer
           has room for the most there can be. */
        size_t channels = (size_t)(r->info.channels ? r->info.channels : AHFLAC_MAX_CHANNELS);
        float *at = r->pcm + r->pcm_len * channels;
        int n = 0;
        if (r->mp3) {
            ahmp3_frame fr;
            used = ahmp3_decode(r->mp3, data, have, eof, at, &n, &fr);
            /* A frame of another shape than the file's first is left out. */
            if (n && r->info.channels && (fr.channels != r->info.channels || fr.sample_rate != r->info.sample_rate)) n = 0;
            if (n && fr_mp3) *fr_mp3 = fr;
        } else if (r->aac) {
            ahaac_frame fr;
            used = ahaac_decode(r->aac, data, have, eof, at, &n, &fr);
            if (n && r->info.channels && (fr.channels != r->info.channels || fr.sample_rate != r->info.sample_rate)) n = 0;
            if (n && fr_aac) *fr_aac = fr;
        } else if (r->flac) {
            ahflac_format f;
            if (room > PCM_FRAMES) room = PCM_FRAMES;
            used = ahflac_decode(r->flac, data, have, eof, r->ints, room * channels, &n, &f);
            if (n) ahflac_to_float(r->ints, at, (size_t)n * (size_t)f.channels, f.bits);
            if (ahflac_decoder_error(r->flac)) return false;
        } else {
            used = ahwav_decode(r->wav, data, have, eof, at, room * channels, &n, NULL);
            if (ahwav_decoder_error(r->wav)) return false;
        }
        r->in_pos += used;
        if (n) {
            append(r, (size_t)n);
            return true;
        }
        if (used) {
            if (used > r->junk_left) return false;
            r->junk_left -= used;
            continue;
        }
        if (eof || !refill(r)) return false;
    }
}

/* ---- walking frames -------------------------------------------------------------- */

/* Steps over the frames of an MPEG audio or ADTS file from the first one,
   by the size each header states. True when frame `want` was reached:
   *offset is where it starts. False at the end of the file: *count is the
   number of frames there are. */
static bool walk_frames(ahreader *r, uint64_t want, uint64_t *offset, uint64_t *count)
{
    uint64_t off = r->first_frame, n = 0, junk = 0;
    bool adts = r->info.kind == AHREADER_AAC;
    while (n < want) {
        const uint8_t *p = peek(r, off, adts ? 7 : 4);
        if (!p) break;
        size_t bytes = 0;
        if (adts) {
            if (p[0] == 0xFF && (p[1] & 0xF6) == 0xF0) bytes = (size_t)(p[3] & 3) << 11 | (size_t)p[4] << 3 | p[5] >> 5;
            if (bytes < 7) bytes = 0;
        } else {
            ahmp3_frame fr;
            if (ahmp3_parse_header(p, 4, &fr) && fr.frame_bytes > 0) bytes = (size_t)fr.frame_bytes;
        }
        if (!bytes) {
            /* Not a frame: a tag or damage. Look for the next one. */
            off++;
            if (++junk > IN_MAX) break;
            continue;
        }
        off += bytes;
        n++;
    }
    if (count) *count = n;
    if (n < want) return false;
    *offset = off;
    return true;
}

/* ---- opening ---------------------------------------------------------------------- */

static size_t m4a_read(void *user, uint64_t offset, void *buffer, size_t bytes)
{
    ahreader *r = user;
    return read_at(r, r->base + offset, buffer, bytes);
}

/* Apple Lossless in an M4A file: every packet stands alone, and the
   settings say all there is to know. */
static const char *open_alac(ahreader *r)
{
    const ahm4a_info *info = ahm4a_get_info(r->m4a);
    ahalac_format f;
    r->alac = ahalac_decoder_new();
    if (!r->alac) return "out of memory";
    if (!ahalac_decoder_configure(r->alac, info->config, info->config_bytes, &f)) return ahalac_decoder_error(r->alac);
    size_t frame = (size_t)f.frame_samples, channels = (size_t)f.channels;
    /* The biggest packet an encoder makes is the samples as they are, plus
       a few bytes; a bigger one, which only a damaged index claims, plays
       as silence. */
    uint64_t biggest = (uint64_t)frame * channels * 5 + 64 * channels + 64;
    r->packet_cap = (size_t)((uint64_t)info->max_packet_bytes < biggest ? (uint64_t)info->max_packet_bytes : biggest) + 1;
    r->packet = malloc(r->packet_cap);
    r->ints = malloc(frame * channels * sizeof(int32_t));
    r->pcm_cap = 2 * frame;
    r->pcm = malloc(r->pcm_cap * channels * sizeof(float));
    if (!r->packet || !r->ints || !r->pcm) return "out of memory";
    /* A file whose first packets do not decode holds no sound to speak of. */
    int n = 0;
    for (uint64_t i = 0; i < info->packets && i < 16 && !n; i++) {
        size_t bytes = ahm4a_read_packet(r->m4a, i, r->packet, r->packet_cap);
        if (bytes) ahalac_decode_packet(r->alac, r->packet, bytes, r->ints, frame * channels, &n);
    }
    if (!n) return "the M4A file holds no sound that could be decoded";
    r->info.kind = AHREADER_ALAC;
    r->info.name = "ALAC";
    r->info.sample_rate = f.sample_rate;
    r->info.channels = f.channels;
    r->info.channel_mask = f.channel_mask;
    r->info.bits = f.bits;
    r->packet_samples = frame;
    uint64_t delay = info->delay, samples = info->samples, decoded = info->packets * frame;
    if (info->timescale > 0 && info->timescale != (uint32_t)f.sample_rate) {
        delay = delay * (uint64_t)f.sample_rate / info->timescale;
        samples = samples * (uint64_t)f.sample_rate / info->timescale;
    }
    if (delay > decoded) delay = decoded;
    if (!samples || samples > decoded - delay) samples = decoded - delay;
    r->lead = r->skip = delay;
    r->play_total = r->left = samples;
    r->info.samples = samples;
    r->info.samples_exact = true;
    return NULL;
}

static const char *open_m4a(ahreader *r)
{
    const char *error = NULL;
    r->m4a = ahm4a_open(m4a_read, r, r->size - r->base, &error);
    if (!r->m4a) return error ? error : "not an M4A file that can be read";
    const ahm4a_info *info = ahm4a_get_info(r->m4a);
    if (info->codec == AHM4A_ALAC) return open_alac(r);
    r->aac = ahaac_decoder_new();
    /* No real AAC packet comes near this; a bigger one, which only a
       damaged index claims, plays as silence. */
    r->packet_cap = (size_t)(info->max_packet_bytes < M4A_PACKET_MAX ? info->max_packet_bytes : M4A_PACKET_MAX) + 1;
    r->packet = malloc(r->packet_cap);
    r->pcm_cap = 2 * AHAAC_MAX_SAMPLES;
    r->pcm = malloc(r->pcm_cap * AHAAC_MAX_CHANNELS * sizeof(float));
    if (!r->aac || !r->packet || !r->pcm) return "out of memory";
    if (!ahaac_decoder_configure(r->aac, info->config, info->config_bytes)) {
        const char *why = ahaac_decoder_error(r->aac);
        return why ? why : "an M4A file of a kind that is not supported";
    }
    /* The first packet that decodes tells the shape of the sound (an
       HE-AAC file states the full rate but plays at half of it). */
    ahaac_frame fr = {0};
    int n = 0;
    for (uint64_t i = 0; i < info->packets && i < 16 && !n; i++) {
        size_t bytes = ahm4a_read_packet(r->m4a, i, r->packet, r->packet_cap);
        if (bytes) ahaac_decode_packet(r->aac, r->packet, bytes, r->pcm, &n, &fr);
    }
    if (!n) {
        const char *why = ahaac_decoder_error(r->aac);
        return why ? why : "the M4A file holds no sound that could be decoded";
    }
    /* Start over with a new decoder: the noise that fills AAC's noise bands
       then begins as it does for anyone decoding the file from its start. */
    ahaac_decoder_free(r->aac);
    r->aac = ahaac_decoder_new();
    if (!r->aac || !ahaac_decoder_configure(r->aac, info->config, info->config_bytes)) return "out of memory";
    r->info.kind = AHREADER_M4A;
    r->info.name = "M4A";
    r->packet_samples = AHAAC_MAX_SAMPLES;
    r->info.sample_rate = fr.sample_rate;
    r->info.channels = fr.channels;
    r->info.half_rate = fr.sbr;
    r->info.parametric_stereo = fr.ps;
    /* The file counts in its own time units, which are not always samples
       at the rate the sound decodes at. */
    uint64_t delay = info->delay, samples = info->samples;
    if (info->timescale > 0 && info->timescale != (uint32_t)fr.sample_rate) {
        delay = delay * (uint64_t)fr.sample_rate / info->timescale;
        samples = samples * (uint64_t)fr.sample_rate / info->timescale;
    }
    uint64_t decoded = info->packets * AHAAC_MAX_SAMPLES;
    if (delay > decoded) delay = decoded;
    if (!samples || samples > decoded - delay) samples = decoded - delay;
    r->lead = r->skip = delay;
    r->play_total = r->left = samples;
    r->info.samples = samples;
    r->info.samples_exact = true;
    return NULL;
}

/* Sets up a new decoder for the stream the Ogg reader is at, from its
   headers, and room for its sound. Returns what went wrong, or NULL. */
static const char *ogg_decoder(ahreader *r, ogg_shape *s)
{
    ahvorbis_decoder_free(r->vorbis);
    ahopus_decoder_free(r->opus);
    ahflac_decoder_free(r->flac);
    r->vorbis = NULL;
    r->opus = NULL;
    r->flac = NULL;
    memset(s, 0, sizeof(*s));
    const ahogg_info *oi = ahogg_get_info(r->ogg);
    const uint8_t *data;
    size_t bytes, most;
    if (oi->codec == AHOGG_VORBIS) {
        r->vorbis = ahvorbis_decoder_new();
        if (!r->vorbis) return "out of memory";
        ahvorbis_format f = {0};
        for (int i = 0; i < 3; i++)
            if (!ahogg_header(r->ogg, i, &data, &bytes) || !ahvorbis_decoder_header(r->vorbis, data, bytes, &f)) {
                const char *why = ahvorbis_decoder_error(r->vorbis);
                return why ? why : "the Vorbis headers are damaged";
            }
        *s = (ogg_shape){f.sample_rate, f.channels, 0, f.channel_mask};
        most = AHVORBIS_MAX_SAMPLES;
    } else if (oi->codec == AHOGG_OPUS) {
        r->opus = ahopus_decoder_new();
        if (!r->opus) return "out of memory";
        ahopus_format f = {0};
        if (!ahogg_header(r->ogg, 0, &data, &bytes) || !ahopus_decoder_header(r->opus, data, bytes, &f)) {
            const char *why = ahopus_decoder_error(r->opus);
            return why ? why : "the Opus header is damaged";
        }
        *s = (ogg_shape){f.sample_rate, f.channels, 0, f.channel_mask};
        r->opus_pre_skip = f.pre_skip;
        most = AHOPUS_MAX_SAMPLES;
    } else {
        /* The first packet carries FLAC's own stream information block: hand
           the decoder that, as a FLAC file would start. */
        if (!ahogg_header(r->ogg, 0, &data, &bytes) || bytes < 51) return "the Ogg FLAC headers are damaged";
        uint8_t head[4 + 4 + 34] = {'f', 'L', 'a', 'C', 0x80, 0, 0, 34};
        memcpy(head + 8, data + 17, 34);
        r->flac = ahflac_decoder_new();
        if (!r->flac) return "out of memory";
        ahflac_format f = {0};
        size_t pos = 0, used;
        int n;
        do {
            used = ahflac_decode(r->flac, head + pos, sizeof(head) - pos, false, NULL, 0, &n, &f);
            pos += used;
        } while (used && pos < sizeof(head));
        if (ahflac_decoder_error(r->flac) || !f.sample_rate || f.max_block <= 0) return "the Ogg FLAC headers are damaged";
        *s = (ogg_shape){f.sample_rate, f.channels, f.bits, 0};
        most = (size_t)f.max_block;
    }
    if (s->rate <= 0 || s->channels <= 0) return "the Ogg file holds no sound";
    size_t channels = (size_t)s->channels;
    float *pcm = realloc(r->pcm, 2 * most * channels * sizeof(float));
    if (!pcm) return "out of memory";
    r->pcm = pcm;
    r->pcm_cap = 2 * most;
    if (r->flac) {
        int32_t *ints = realloc(r->ints, most * channels * sizeof(int32_t));
        if (!ints) return "out of memory";
        r->ints = ints;
    }
    r->ogg_packet_max = most;
    r->dec = *s;
    return NULL;
}

static void ogg_reset_decoder(ahreader *r)
{
    if (r->vorbis) ahvorbis_decoder_reset(r->vorbis);
    else if (r->opus) ahopus_decoder_reset(r->opus);
    else if (r->flac) ahflac_decoder_reset(r->flac);
}

/* Read at random: where the sound of the stream the Ogg reader is at
   starts, and how much it plays. Leaves the reader at its first sound
   packet. False when it holds no sound that can be decoded. */
static bool ogg_measure(ahreader *r, ogg_stream *st)
{
    const ahogg_info *oi = ahogg_get_info(r->ogg);
    const uint8_t *data;
    size_t bytes;
    int64_t got = 0, granule = -1;
    bool any = false, last = false;
    for (int packets = 0; packets < 100000 && granule < 0; packets++) {
        if (!ahogg_packet(r->ogg, &data, &bytes, &granule)) break;
        last = ahogg_last_page(r->ogg);
        int n = 0;
        if (r->vorbis) any |= ahvorbis_decode_packet(r->vorbis, data, bytes, r->pcm, &n);
        else if (r->opus) any |= ahopus_decode_packet(r->opus, data, bytes, r->pcm, &n);
        else n = (int)ogg_flac_frame(r, data, bytes, r->pcm, r->pcm_cap);
        got += n;
        any |= n > 0;
    }
    if (!any) return false;
    bool opus = r->opus != NULL;
    st->first = ogg_first_place(opus, granule, got, last);
    st->start = ogg_start_place(st->first, opus, r->opus_pre_skip);
    st->play = UINT64_MAX;
    if (oi->last_granule >= 0) st->play = oi->last_granule > st->start ? (uint64_t)(oi->last_granule - st->start) : 0;
    /* And back to the start. */
    ahogg_seek(r->ogg, -1);
    ogg_reset_decoder(r);
    return true;
}

/* Read at random: the streams played, and the length of them all. Without
   ahreader_follow_changes the sound ends where a stream of another shape
   starts. */
static void ogg_totals(ahreader *r)
{
    const ogg_shape *a = &r->streams[0].shape;
    uint64_t total = 0;
    bool known = true;
    r->nfollowed = r->nstreams;
    for (int k = 0; k < r->nstreams; k++) {
        ogg_stream *st = &r->streams[k];
        st->before = total;
        if (!st->ok) continue;
        const ogg_shape *b = &st->shape;
        if (!r->follow && (a->rate != b->rate || a->channels != b->channels || a->bits != b->bits || a->mask != b->mask)) {
            r->nfollowed = k;
            break;
        }
        if (st->play == UINT64_MAX) known = false;
        else total += st->play;
    }
    r->info.samples = known ? total : 0;
    r->info.samples_exact = known;
}

/* Read at random: plays stream k from its start. */
static void ogg_begin(ahreader *r, int k)
{
    const ogg_stream *st = &r->streams[k];
    r->ogg_cur = k;
    r->ogg_first = st->first;
    r->ogg_start = st->start;
    r->play_total = st->play;
    r->left = st->play;
    r->skip = (uint64_t)(st->start - st->first);
    r->ogg_lost = false;
}

/* The info describes the stream the decoder is at, from now on. */
static void ogg_show(ahreader *r)
{
    const ahogg_info *oi = ahogg_get_info(r->ogg);
    r->info.kind = oi->codec == AHOGG_VORBIS ? AHREADER_VORBIS : oi->codec == AHOGG_OPUS ? AHREADER_OPUS : AHREADER_OGG_FLAC;
    r->info.name = oi->codec == AHOGG_VORBIS ? "Vorbis" : oi->codec == AHOGG_OPUS ? "Opus" : "Ogg FLAC";
    r->info.sample_rate = r->dec.rate;
    r->info.channels = r->dec.channels;
    r->info.channel_mask = r->dec.mask;
    r->info.bits = r->dec.bits;
    r->info.stream = oi->stream;
    /* The comments: Vorbis's and Opus's second header; for FLAC the block
       of them, which the mapping puts right after the first packet. */
    free(r->comments);
    r->comments = NULL;
    r->comments_bytes = 0;
    const uint8_t *data;
    size_t bytes;
    for (int i = 1; i < 3; i++) {
        if (!ahogg_header(r->ogg, i, &data, &bytes)) break;
        if (oi->codec == AHOGG_FLAC && (!bytes || (data[0] & 0x7F) != 4)) continue;
        r->comments = malloc(bytes ? bytes : 1);
        if (r->comments) {
            memcpy(r->comments, data, bytes);
            r->comments_bytes = bytes;
        }
        break;
    }
    r->ogg_changed = false;
}

/* The stream played has ended: sets up the next one of the chain to play.
   False when there is none. */
static bool ogg_next(ahreader *r)
{
    if (r->ogg_over) return false;
    bool moved = false;
    if (r->in_order) {
        while (!moved) {
            /* Waiting, the decoder is at that stream already. */
            if (!r->ogg_waiting) {
                if (!ahogg_next_stream(r->ogg)) break;
                ogg_shape s;
                if (ogg_decoder(r, &s)) continue; /* damaged: on to the next */
            }
            const ogg_shape *s = &r->dec;
            r->ogg_waiting = !r->follow && (s->rate != r->info.sample_rate || s->channels != r->info.channels ||
                                            s->bits != r->info.bits || s->mask != r->info.channel_mask);
            if (r->ogg_waiting) break;
            r->ogg_place = -1;
            r->ogg_held = 0;
            r->skip = 0;
            r->left = UINT64_MAX;
            moved = true;
        }
    } else {
        for (int k = r->ogg_cur + 1; k < r->nfollowed && !moved; k++) {
            ogg_shape s;
            if (!r->streams[k].ok || !ahogg_select_stream(r->ogg, k)) continue;
            if (ogg_decoder(r, &s)) {
                r->ogg_cur = -1; /* no stream's decoder is in hand */
                continue;
            }
            ogg_begin(r, k);
            moved = true;
        }
        r->ogg_waiting = !moved && r->nfollowed < r->nstreams;
    }
    if (!moved) {
        r->ogg_over = true;
        return false;
    }
    r->pcm_len = r->pcm_pos = 0;
    r->ended = false;
    r->ogg_changed = true;
    return true;
}

/* Vorbis, FLAC or Opus in an Ogg file. The pages state where the sound
   ends; the first page that states a place also tells where the sound
   starts: the sound decoded up to it ends there. Opus drops the header's
   pre-skip as well (RFC 7845). Each stream of a chain is measured so. */
static const char *open_ogg(ahreader *r)
{
    const char *error = NULL;
    r->ogg = ahogg_open(m4a_read, r, r->size - r->base, &error);
    if (!r->ogg) return error ? error : "not an Ogg file that can be read";
    int n = ahogg_get_info(r->ogg)->streams;
    r->streams = calloc((size_t)n, sizeof(ogg_stream));
    if (!r->streams) return "out of memory";
    r->nstreams = n;
    for (int k = 0; k < n; k++) {
        ogg_stream *st = &r->streams[k];
        if (k && !ahogg_select_stream(r->ogg, k)) continue;
        const char *why = ogg_decoder(r, &st->shape);
        if (!why && !ogg_measure(r, st)) why = "the Ogg file holds no sound that could be decoded";
        if (why && !k) return why;
        st->ok = !why;
    }
    ogg_shape s;
    if (n > 1 && (!ahogg_select_stream(r->ogg, 0) || ogg_decoder(r, &s))) return "the Ogg file cannot be read again";
    ogg_show(r);
    ogg_totals(r);
    ogg_begin(r, 0);
    return NULL;
}

static const char *open_mpeg(ahreader *r, uint64_t tag)
{
    r->mp3 = ahmp3_decoder_new();
    r->pcm_cap = HOLD_MAX + 2 * AHMP3_MAX_SAMPLES;
    r->pcm = malloc(r->pcm_cap * 2 * sizeof(float));
    if (!r->mp3 || !r->pcm) return "out of memory";
    r->junk_left = tag + JUNK_MAX;
    ahmp3_frame fr;
    if (!decode_more(r, &fr, NULL)) return "no MPEG audio frames found";
    r->info.kind = fr.layer == 3 ? AHREADER_MP3 : fr.layer == 2 ? AHREADER_MP2 : AHREADER_MP1;
    r->info.name = fr.layer == 3 ? "MP3" : fr.layer == 2 ? "MP2" : "MP1";
    r->info.sample_rate = fr.sample_rate;
    r->info.channels = fr.channels;
    r->frame_samples = fr.samples;
    r->first_frame = r->in_off + r->in_pos - (uint64_t)fr.frame_bytes;
    const uint8_t *head = peek(r, r->first_frame, 4);
    ahmp3_frame parsed;
    r->no_walk = !head || !ahmp3_parse_header(head, 4, &parsed) || parsed.frame_bytes <= 0;

    /* Gapless: the encoder's and the decoder's run-in go, and the padding. */
    const ahmp3_info *info = ahmp3_decoder_info(r->mp3);
    if (fr.layer == 3 && info->present && info->encoder_delay >= 0) {
        r->lead = (uint64_t)info->encoder_delay + AHMP3_DECODER_DELAY;
        r->hold = info->encoder_padding > AHMP3_DECODER_DELAY ? (size_t)(info->encoder_padding - AHMP3_DECODER_DELAY) : 0;
        if (r->hold > HOLD_MAX) r->hold = HOLD_MAX;
    }
    /* The first frame is in pcm already, untrimmed. */
    size_t got = r->pcm_len;
    r->pcm_len = 0;
    r->skip = r->lead;
    append(r, got);

    if (info->present && info->frames) {
        uint64_t all = (uint64_t)info->frames * (uint64_t)fr.samples, cut = r->lead + r->hold;
        r->info.samples = all > cut ? all - cut : 0;
        r->info.samples_exact = true;
    } else if (fr.bitrate_kbps > 0 && r->size > r->first_frame) {
        r->info.samples = (r->size - r->first_frame) * 8 / 1000 * (uint64_t)fr.sample_rate / (uint64_t)fr.bitrate_kbps;
    }
    return NULL;
}

static const char *open_adts(ahreader *r, uint64_t tag)
{
    r->aac = ahaac_decoder_new();
    r->pcm_cap = 2 * AHAAC_MAX_SAMPLES;
    r->pcm = malloc(r->pcm_cap * AHAAC_MAX_CHANNELS * sizeof(float));
    if (!r->aac || !r->pcm) return "out of memory";
    r->junk_left = tag + JUNK_MAX;
    ahaac_frame fr;
    if (!decode_more(r, NULL, &fr)) {
        const char *why = ahaac_decoder_error(r->aac);
        return why ? why : "no AAC frames found";
    }
    r->info.kind = AHREADER_AAC;
    r->info.name = "AAC";
    r->info.sample_rate = fr.sample_rate;
    r->info.channels = fr.channels;
    r->info.half_rate = fr.sbr;
    r->info.parametric_stereo = fr.ps;
    r->frame_samples = AHAAC_MAX_SAMPLES;
    r->first_frame = r->in_off + r->in_pos - (uint64_t)fr.frame_bytes;
    /* An ADTS file does not say how long it is: count its frames. */
    uint64_t frames = 0, end;
    walk_frames(r, UINT64_MAX, &end, &frames);
    r->info.samples = frames * AHAAC_MAX_SAMPLES;
    r->info.samples_exact = true;
    return NULL;
}

/* Where a FLAC file's frames start: past the marker and the blocks of
   information that follow it. */
static bool flac_first_frame(ahreader *r, uint64_t tag, uint64_t *offset)
{
    uint64_t off = tag + 4;
    for (int blocks = 0; blocks < 100000; blocks++) {
        const uint8_t *p = peek(r, off, 4);
        if (!p) return false;
        off += 4 + ((uint64_t)p[1] << 16 | (uint64_t)p[2] << 8 | p[3]);
        if (p[0] & 0x80) {
            *offset = off;
            return off < r->size;
        }
    }
    return false;
}

static const char *open_flac(ahreader *r, uint64_t tag)
{
    r->flac = ahflac_decoder_new();
    r->pcm_cap = PCM_FRAMES;
    r->pcm = malloc(r->pcm_cap * AHFLAC_MAX_CHANNELS * sizeof(float));
    r->ints = malloc((size_t)PCM_FRAMES * AHFLAC_MAX_CHANNELS * sizeof(int32_t));
    if (!r->flac || !r->pcm || !r->ints) return "out of memory";
    /* Decode one sample to learn the shape of the sound, then start over
       from the first frame. */
    ahflac_format f = {0};
    for (;;) {
        bool eof = r->in_off + r->in_len >= r->size;
        int n = 0;
        uint64_t before = r->in_off + r->in_pos;
        size_t used = ahflac_decode(r->flac, r->in + r->in_pos, r->in_len - r->in_pos, eof, r->ints, AHFLAC_MAX_CHANNELS, &n, &f);
        r->in_pos += used;
        if (ahflac_decoder_error(r->flac)) return ahflac_decoder_error(r->flac);
        if (n) {
            if (!flac_first_frame(r, tag, &r->first_frame)) r->first_frame = before;
            break;
        }
        if (used) continue;
        if (eof || !refill(r)) return "the FLAC file holds no sound";
    }
    r->info.kind = AHREADER_FLAC;
    r->info.name = "FLAC";
    r->info.sample_rate = f.sample_rate;
    r->info.channels = f.channels;
    r->info.bits = f.bits;
    r->info.samples = f.samples;
    r->info.samples_exact = f.samples != 0;
    ahflac_decoder_reset(r->flac);
    set_window(r, r->first_frame);
    return NULL;
}

static const char *open_wav(ahreader *r)
{
    r->wav = ahwav_decoder_new();
    r->pcm_cap = PCM_FRAMES;
    r->pcm = malloc(r->pcm_cap * AHWAV_MAX_CHANNELS * sizeof(float));
    if (!r->wav || !r->pcm) return "out of memory";
    set_window(r, r->base);
    /* Read the header alone: no room for samples yet. */
    ahwav_format f = {0};
    for (;;) {
        bool eof = r->in_off + r->in_len >= r->size;
        int n = 0;
        size_t used = ahwav_decode(r->wav, r->in + r->in_pos, r->in_len - r->in_pos, eof, r->pcm, 0, &n, &f);
        r->in_pos += used;
        if (ahwav_decoder_error(r->wav)) return ahwav_decoder_error(r->wav);
        uint64_t at;
        if (f.sample_rate && ahwav_decoder_seek(r->wav, 0, &at)) {
            set_window(r, r->base + at);
            break;
        }
        if (used) continue;
        if (eof || !refill(r)) return "the WAV file holds no sound";
    }
    r->info.kind = AHREADER_WAV;
    r->info.name = "WAV";
    r->info.sample_rate = f.sample_rate;
    r->info.channels = f.channels;
    r->info.channel_mask = f.channel_mask;
    r->info.bits = f.is_float ? 0 : f.bits;
    r->info.samples = f.samples;
    r->info.samples_exact = f.samples != 0;
    if (f.samples) r->left = f.samples; /* what follows the sound is not read */
    return NULL;
}

ahreader *ahreader_open(ahreader_read_fn read, void *user, uint64_t size, const char **error)
{
    const char *why = NULL;
    ahreader *r = calloc(1, sizeof(*r));
    if (error) *error = NULL;
    if (r) r->in = malloc(IN_START);
    if (!r || !r->in) {
        if (error) *error = "out of memory";
        ahreader_close(r);
        return NULL;
    }
    r->read = read;
    r->user = user;
    r->size = size;
    r->in_cap = IN_START;
    r->left = UINT64_MAX;
    r->junk_left = UINT64_MAX;

    /* Past a tag at the front, the first bytes tell the kinds apart. */
    uint8_t head[16];
    size_t got = read_at(r, 0, head, sizeof(head));
    uint64_t tag = ahmp3_id3v2_size(head, got);
    if (tag >= size) tag = 0;
    if (tag) got = read_at(r, tag, head, sizeof(head));
    if (got >= 12 && (!memcmp(head, "RIFF", 4) || !memcmp(head, "RF64", 4))) {
        r->base = tag;
        why = open_wav(r);
    } else if (got >= 4 && !memcmp(head, "fLaC", 4)) {
        why = open_flac(r, tag);
    } else if (got >= 12 && !memcmp(head + 4, "ftyp", 4)) {
        r->base = tag;
        why = open_m4a(r);
    } else if (got >= 4 && !memcmp(head, "OggS", 4)) {
        r->base = tag;
        why = open_ogg(r);
    } else {
        /* MPEG audio and ADTS both start with eleven 1 bits; ADTS has two
           0 bits where MPEG audio names its layer. */
        bool adts = false;
        const uint8_t *p = peek(r, tag, 2);
        for (size_t i = 0; p && i + 2 <= r->scan_len; i++) {
            if (p[i] == 0xFF && (p[i + 1] & 0xE0) == 0xE0) {
                adts = (p[i + 1] & 0xF6) == 0xF0;
                break;
            }
        }
        why = adts ? open_adts(r, tag) : open_mpeg(r, tag);
    }
    if (!why && (r->info.sample_rate <= 0 || r->info.channels <= 0)) why = "the file holds no sound";
    if (why) {
        if (error) *error = why;
        ahreader_close(r);
        return NULL;
    }
    r->junk_left = UINT64_MAX;
    return r;
}

ahreader *ahreader_open_stream(ahreader_read_fn read, void *user, const char **error)
{
    const char *why = NULL;
    ahreader *r = calloc(1, sizeof(*r));
    if (error) *error = NULL;
    if (!r) {
        if (error) *error = "out of memory";
        return NULL;
    }
    r->read = read;
    r->user = user;
    r->size = UINT64_MAX;
    r->left = UINT64_MAX;
    r->junk_left = UINT64_MAX;
    r->in_order = true;
    r->ogg_place = -1;
    r->ogg = ahogg_open_stream(m4a_read, r, &why);
    if (!r->ogg) {
        if (!why) why = "not an Ogg stream that can be read";
    } else {
        /* A stream whose headers the decoder refuses is passed over. */
        ogg_shape s;
        while ((why = ogg_decoder(r, &s)) != NULL && ahogg_next_stream(r->ogg)) {}
        if (!why) ogg_show(r);
    }
    if (why) {
        if (error) *error = why;
        ahreader_close(r);
        return NULL;
    }
    return r;
}

void ahreader_follow_changes(ahreader *r, bool follow)
{
    r->follow = follow;
    if (r->ogg && !r->in_order) ogg_totals(r);
    if (follow && r->ogg_waiting) r->ogg_over = false; /* it goes on */
}

static uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

bool ahreader_comment(const ahreader *r, int index, const char **text, size_t *bytes)
{
    const uint8_t *p = r->comments;
    size_t n = r->comments_bytes, at;
    if (!p || index < 0) return false;
    if (n >= 7 && !memcmp(p, "\x03vorbis", 7)) at = 7;
    else if (n >= 8 && !memcmp(p, "OpusTags", 8)) at = 8;
    else if (n >= 4 && (p[0] & 0x7F) == 4) at = 4; /* FLAC's block of comments */
    else return false;
    /* The vendor's name, then the comments, each with its length first. */
    if (n - at < 4 || le32(p + at) > n - at - 4) return false;
    at += 4 + le32(p + at);
    if (n - at < 4) return false;
    uint32_t count = le32(p + at);
    at += 4;
    for (uint32_t i = 0; i < count; i++) {
        if (n - at < 4) return false;
        uint32_t len = le32(p + at);
        at += 4;
        if (len > n - at) return false;
        if (i == (uint32_t)index) {
            *text = (const char *)(p + at);
            *bytes = len;
            return true;
        }
        at += len;
    }
    return false;
}

void ahreader_close(ahreader *r)
{
    if (!r) return;
    ahmp3_decoder_free(r->mp3);
    ahaac_decoder_free(r->aac);
    ahflac_decoder_free(r->flac);
    ahwav_decoder_free(r->wav);
    ahalac_decoder_free(r->alac);
    ahvorbis_decoder_free(r->vorbis);
    ahopus_decoder_free(r->opus);
    ahm4a_close(r->m4a);
    ahogg_close(r->ogg);
    free(r->in);
    free(r->pcm);
    free(r->ints);
    free(r->packet);
    free(r->scan);
    free(r->streams);
    free(r->comments);
    free(r);
}

const ahreader_info *ahreader_get_info(const ahreader *r)
{
    return &r->info;
}

/* ---- reading ---------------------------------------------------------------------- */

/* Gives what the stream in hand has, as ahreader_read does. */
static size_t read_stream(ahreader *r, float *pcm, size_t max_frames)
{
    size_t given = 0, channels = (size_t)r->info.channels;
    while (given < max_frames && r->left) {
        size_t have = r->pcm_len - r->pcm_pos;
        if (!r->ended) have = have > r->hold ? have - r->hold : 0;
        if (!have) {
            if (r->ended) break;
            if (!decode_more(r, NULL, NULL)) {
                /* The end: what was kept back is the padding. */
                r->ended = true;
                size_t kept = r->pcm_len - r->pcm_pos;
                r->pcm_len -= kept < r->hold ? kept : r->hold;
            }
            continue;
        }
        size_t n = max_frames - given;
        if (n > have) n = have;
        if (n > r->left) n = (size_t)r->left;
        memcpy(pcm + given * channels, r->pcm + r->pcm_pos * channels, n * channels * sizeof(float));
        r->pcm_pos += n;
        given += n;
        if (r->left != UINT64_MAX) r->left -= n;
    }
    return given;
}

size_t ahreader_read(ahreader *r, float *pcm, size_t max_frames)
{
    if (!max_frames) return 0;
    for (;;) {
        /* A read gives the sound of one stream of a chain only. */
        if (r->ogg_changed) ogg_show(r);
        size_t given = read_stream(r, pcm, max_frames);
        if (given || !r->ogg || !ogg_next(r)) return given;
    }
}

/* ---- seeking ---------------------------------------------------------------------- */

/* Finds the first frame at or after file offset `off` and says where its
   first sample lies. */
static bool flac_probe(ahreader *r, uint64_t off, uint64_t *sample)
{
    ahflac_decoder_reset(r->flac);
    set_window(r, off);
    for (;;) {
        bool eof = r->in_off + r->in_len >= r->size;
        int n = 0;
        size_t used = ahflac_decode(r->flac, r->in + r->in_pos, r->in_len - r->in_pos, eof, r->ints, (size_t)r->info.channels, &n, NULL);
        r->in_pos += used;
        if (n) {
            *sample = ahflac_decoder_position(r->flac) - (uint64_t)n;
            return true;
        }
        if (used) continue;
        if (eof || !refill(r)) return false;
    }
}

/* A FLAC frame says where it lies in the sound, so the place is found by
   trying offsets: each try narrows the stretch of file the target is in. */
static void flac_seek(ahreader *r, uint64_t sample)
{
    uint64_t lo_off = r->first_frame, lo_at = 0, hi_off = r->size, hi_at = r->info.samples;
    uint64_t near = (uint64_t)r->info.sample_rate / 2;
    for (int tries = 0; tries < 40 && hi_at > lo_at && sample - lo_at > 2 * near; tries++) {
        /* Aim a little before the target, to land on a frame at or before it. */
        uint64_t aim = sample - near;
        double part = (double)(aim - lo_at) / (double)(hi_at - lo_at);
        uint64_t guess = lo_off + (uint64_t)((double)(hi_off - lo_off) * (part < 1 ? part : 1));
        if (guess <= lo_off || guess >= hi_off) break;
        uint64_t at = 0;
        if (!flac_probe(r, guess, &at)) {
            hi_off = guess; /* no frame from there on */
            continue;
        }
        if (at > sample) {
            hi_off = guess;
            if (at < hi_at) hi_at = at;
            continue;
        }
        if (at <= lo_at) break;
        lo_off = guess;
        lo_at = at;
    }
    ahflac_decoder_reset(r->flac);
    set_window(r, lo_off);
    r->skip = sample - lo_at;
}

bool ahreader_seek(ahreader *r, uint64_t sample)
{
    if (r->in_order) return false; /* reading goes on */
    r->pcm_len = r->pcm_pos = 0;
    r->skip = 0;
    r->ended = false;
    r->ogg_over = r->ogg_waiting = false;
    if (r->info.samples_exact && sample >= r->info.samples) {
        r->ended = r->ogg_over = true;
        return false;
    }
    if (r->wav) {
        uint64_t at;
        if (!ahwav_decoder_seek(r->wav, sample, &at)) {
            r->ended = true;
            return false;
        }
        set_window(r, r->base + at);
        if (r->info.samples) r->left = r->info.samples - sample;
    } else if (r->m4a) {
        /* An AAC packet needs the one before it; start two back to be
           sure. An ALAC packet stands alone. */
        uint64_t at = sample + r->lead, packet = at / r->packet_samples, back = r->aac ? 2 : 0;
        uint64_t first = packet > back ? packet - back : 0;
        if (r->aac) ahaac_decoder_reset(r->aac);
        r->next_packet = first;
        r->skip = at - first * r->packet_samples;
        r->left = r->play_total - sample;
    } else if (r->ogg) {
        /* Back to a page a little before; the place is known once a page
           states it, and the sound up to the target goes. Opus starts 200 ms
           before the target, for its decoder to settle (RFC 7845 asks for at
           least 80; CELT's band energies settle by half each 20 ms frame),
           or from the very start when that is nearer. */
        /* First the stream of the chain it lies in. */
        int k = -1;
        for (int i = 0; i < r->nfollowed && k < 0 && sample <= (uint64_t)INT64_MAX / 2; i++) {
            const ogg_stream *st = &r->streams[i];
            if (st->ok && (st->play == UINT64_MAX || sample < st->before + st->play)) k = i;
        }
        ogg_shape s;
        bool moved = k >= 0 && k != r->ogg_cur && ahogg_select_stream(r->ogg, k);
        if (moved && ogg_decoder(r, &s)) {
            r->ogg_cur = -1; /* no stream's decoder is in hand */
            moved = false;
        }
        if (k < 0 || (k != r->ogg_cur && !moved)) {
            r->ended = r->ogg_over = true; /* no stream runs that long */
            return false;
        }
        ogg_begin(r, k);
        r->ogg_changed = ahogg_get_info(r->ogg)->stream != r->info.stream;
        sample -= r->streams[k].before;
        int64_t target = r->ogg_start + (int64_t)sample, aim = target;
        ogg_reset_decoder(r);
        if (r->opus) aim = target - 9600 < r->ogg_start ? -1 : target - 9600;
        if (aim >= 0 && ahogg_seek(r->ogg, aim)) {
            r->ogg_lost = true;
            r->ogg_target = target;
        } else {
            if (aim < 0) ahogg_seek(r->ogg, -1);
            r->ogg_lost = false;
            r->skip = (uint64_t)(target - r->ogg_first);
        }
        if (r->play_total != UINT64_MAX) r->left = r->play_total - sample;
    } else if (r->flac) {
        flac_seek(r, sample);
    } else {
        /* MPEG audio and ADTS: a frame leans on those before it (MP3's bit
           reservoir reaches back several), so decoding starts early and
           the run-up is dropped. */
        uint64_t at = sample + r->lead, per = (uint64_t)r->frame_samples, frame = at / per;
        uint64_t back = r->mp3 ? 10 : 2, first = frame > back ? frame - back : 0, off = r->first_frame;
        if (first && r->no_walk) first = 0; /* no frame sizes to step by: from the start */
        if (first && !walk_frames(r, first, &off, NULL)) {
            r->ended = true;
            return false;
        }
        if (r->mp3) ahmp3_decoder_reset(r->mp3);
        else ahaac_decoder_reset(r->aac);
        set_window(r, off);
        r->skip = at - first * per;
    }
    return true;
}
