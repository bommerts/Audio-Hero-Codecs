/* One reader over every decoder: tells the kind of file, decodes it a piece
   at a time, trims gapless files, and seeks. */
#include "../include/ahreader.h"
#include "../include/ahaac.h"
#include "../include/ahalac.h"
#include "../include/ahflac.h"
#include "../include/ahm4a.h"
#include "../include/ahmp3.h"
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
};

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
    ahm4a *m4a;
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

void ahreader_close(ahreader *r)
{
    if (!r) return;
    ahmp3_decoder_free(r->mp3);
    ahaac_decoder_free(r->aac);
    ahflac_decoder_free(r->flac);
    ahwav_decoder_free(r->wav);
    ahalac_decoder_free(r->alac);
    ahm4a_close(r->m4a);
    free(r->in);
    free(r->pcm);
    free(r->ints);
    free(r->packet);
    free(r->scan);
    free(r);
}

const ahreader_info *ahreader_get_info(const ahreader *r)
{
    return &r->info;
}

/* ---- reading ---------------------------------------------------------------------- */

size_t ahreader_read(ahreader *r, float *pcm, size_t max_frames)
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
    r->pcm_len = r->pcm_pos = 0;
    r->skip = 0;
    r->ended = false;
    if (r->info.samples_exact && sample >= r->info.samples) {
        r->ended = true;
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
