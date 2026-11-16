/* WAV reader: RIFF/WAVE and RF64, PCM and IEEE float, plain or extensible
   format chunks. */
#include "../include/ahwav.h"
#include <stdlib.h>
#include <string.h>

enum { ST_RIFF, ST_CHUNK, ST_DATA, ST_DONE, ST_ERROR };

struct ahwav_decoder {
    int state;
    const char *error;
    ahwav_format format;
    bool have_format;
    int sample_bytes;      /* bytes one sample of one channel takes */
    bool rf64;
    uint64_t ds64_data;    /* data size from an RF64 ds64 chunk */
    bool have_ds64;
    uint64_t skip_pending; /* bytes of a chunk still to pass over */
    uint64_t data_left;    /* bytes of sound left in the data chunk */
    bool data_open;        /* the data chunk runs to the end of the file */
    uint64_t consumed;     /* bytes used by earlier calls */
    uint64_t data_start;   /* where the sound starts, from the first byte fed */
    uint64_t data_size;    /* bytes of sound in all (not when data_open) */
    bool in_data;          /* the data chunk has been reached */
};

static uint32_t le16(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8; }
static uint32_t le32(const uint8_t *p) { return le16(p) | le16(p + 2) << 16; }
static uint64_t le64(const uint8_t *p) { return le32(p) | (uint64_t)le32(p + 4) << 32; }

ahwav_decoder *ahwav_decoder_new(void)
{
    return calloc(1, sizeof(ahwav_decoder));
}

void ahwav_decoder_free(ahwav_decoder *d)
{
    free(d);
}

const char *ahwav_decoder_error(const ahwav_decoder *d)
{
    return d->error;
}

static size_t fail(ahwav_decoder *d, const char *why)
{
    d->state = ST_ERROR;
    d->error = why;
    return 0;
}

/* The fmt chunk: 16 bytes, or 40 for the extensible form whose real format
   tag is the start of the sub-format GUID. */
static bool read_format(ahwav_decoder *d, const uint8_t *p, size_t size)
{
    uint32_t tag = le16(p), channels = le16(p + 2), rate = le32(p + 4), align = le16(p + 12), bits = le16(p + 14);
    if (tag == 0xFFFE) {
        if (size < 40) return false;
        tag = le16(p + 24);
        d->format.channel_mask = le32(p + 20);
    }
    if (tag != 1 && tag != 3) return false;
    if (channels < 1 || channels > AHWAV_MAX_CHANNELS || rate < 1 || rate > 768000) return false;
    /* The block size says how many bytes a sample takes; fall back on the
       bit count when it makes no sense. */
    uint32_t bytes = (bits + 7) / 8;
    if (align % channels == 0 && align / channels >= bytes) bytes = align / channels;
    if (tag == 1 && (bytes < 1 || bytes > 4)) return false;
    if (tag == 3 && bytes != 4 && bytes != 8) return false;
    d->format.sample_rate = (int)rate;
    d->format.channels = (int)channels;
    d->format.bits = (int)bytes * 8;
    d->format.is_float = tag == 3;
    d->sample_bytes = (int)bytes;
    d->have_format = true;
    return true;
}

static void convert(const ahwav_decoder *d, const uint8_t *p, float *out, size_t count)
{
    switch (d->sample_bytes) {
    case 1:
        for (size_t i = 0; i < count; i++) out[i] = (float)((int)p[i] - 128) * (1.0f / 128);
        break;
    case 2:
        for (size_t i = 0; i < count; i++) out[i] = (float)(int16_t)le16(p + 2 * i) * (1.0f / 32768);
        break;
    case 3:
        for (size_t i = 0; i < count; i++) {
            int32_t v = (int32_t)((uint32_t)p[3 * i] << 8 | (uint32_t)p[3 * i + 1] << 16 | (uint32_t)p[3 * i + 2] << 24) >> 8;
            out[i] = (float)v * (1.0f / 8388608);
        }
        break;
    case 4:
        if (d->format.is_float) {
            for (size_t i = 0; i < count; i++) {
                uint32_t bits = le32(p + 4 * i);
                memcpy(out + i, &bits, 4);
            }
        } else {
            for (size_t i = 0; i < count; i++) out[i] = (float)(int32_t)le32(p + 4 * i) * (1.0f / 2147483648.0f);
        }
        break;
    default:
        for (size_t i = 0; i < count; i++) {
            uint64_t bits = le64(p + 8 * i);
            double v;
            memcpy(&v, &bits, 8);
            out[i] = (float)v;
        }
        break;
    }
}

size_t ahwav_decode(ahwav_decoder *d, const uint8_t *data, size_t len, bool eof, float *pcm, size_t pcm_cap,
                    int *samples, ahwav_format *format)
{
    size_t pos = 0;
    *samples = 0;
    for (;;) {
        if (d->skip_pending) {
            size_t n = d->skip_pending < len - pos ? (size_t)d->skip_pending : len - pos;
            d->skip_pending -= n;
            pos += n;
            if (d->skip_pending) {
                if (eof && d->state == ST_CHUNK) fail(d, "the WAV file holds no sound");
                break;
            }
        }
        if (d->state == ST_RIFF) {
            if (len - pos < 12) {
                if (eof) fail(d, "not a WAV file");
                break;
            }
            const uint8_t *p = data + pos;
            d->rf64 = !memcmp(p, "RF64", 4);
            if ((memcmp(p, "RIFF", 4) && !d->rf64) || memcmp(p + 8, "WAVE", 4)) {
                fail(d, "not a WAV file");
                break;
            }
            pos += 12;
            d->state = ST_CHUNK;
        } else if (d->state == ST_CHUNK) {
            if (len - pos < 8) {
                if (eof) fail(d, "the WAV file holds no sound"); /* the data chunk never came */
                break;
            }
            const uint8_t *p = data + pos;
            uint64_t size = le32(p + 4);
            if (!memcmp(p, "fmt ", 4) || !memcmp(p, "ds64", 4)) {
                /* Small chunks that are read whole. */
                bool fmt = p[0] == 'f';
                if (size < (fmt ? 16u : 24u) || size > 4096) {
                    fail(d, "damaged WAV header");
                    break;
                }
                if (len - pos - 8 < size) {
                    if (eof) fail(d, "damaged WAV header");
                    break;
                }
                if (fmt) {
                    if (!read_format(d, p + 8, (size_t)size)) {
                        fail(d, "a kind of WAV file that is not supported");
                        break;
                    }
                } else {
                    d->ds64_data = le64(p + 16);
                    d->have_ds64 = true;
                }
                pos += 8;
                d->skip_pending = size + (size & 1);
            } else if (!memcmp(p, "data", 4)) {
                if (!d->have_format) {
                    fail(d, "damaged WAV header");
                    break;
                }
                pos += 8;
                /* A size of all ones means "see ds64" in RF64 and "unknown"
                   in a stream; 0 is also written by tools that could not
                   seek back to fill it in. */
                if (size == 0xFFFFFFFFu && d->rf64 && d->have_ds64) size = d->ds64_data;
                else if (size == 0xFFFFFFFFu || size == 0) d->data_open = true;
                d->data_left = size;
                size_t block = (size_t)d->sample_bytes * (size_t)d->format.channels;
                d->format.samples = d->data_open ? 0 : size / block;
                d->data_start = d->consumed + pos;
                d->data_size = size;
                d->in_data = true;
                d->state = ST_DATA;
            } else {
                pos += 8;
                d->skip_pending = size + (size & 1);
            }
        } else if (d->state == ST_DATA) {
            size_t block = (size_t)d->sample_bytes * (size_t)d->format.channels;
            size_t frames = (len - pos) / block;
            if (!d->data_open && frames > d->data_left / block) frames = (size_t)(d->data_left / block);
            size_t room = pcm_cap / (size_t)d->format.channels;
            if (room > 0x40000000u / AHWAV_MAX_CHANNELS) room = 0x40000000u / AHWAV_MAX_CHANNELS;
            if (frames > room) frames = room;
            if (frames) {
                convert(d, data + pos, pcm, frames * (size_t)d->format.channels);
                pos += frames * block;
                if (!d->data_open) d->data_left -= frames * block;
                *samples = (int)frames;
                break;
            }
            if (!room) break;
            if (!d->data_open && d->data_left < block) {
                /* The sound has ended; whatever follows is not needed. */
                d->state = ST_DONE;
            } else {
                if (eof) {
                    pos = len; /* a cut-off last sample */
                    d->state = ST_DONE;
                }
                break;
            }
        } else if (d->state == ST_DONE) {
            pos = len;
            break;
        } else {
            break;
        }
    }
    if (format) {
        if (d->have_format) *format = d->format;
        else memset(format, 0, sizeof(*format));
    }
    d->consumed += pos;
    return pos;
}

bool ahwav_decoder_seek(ahwav_decoder *d, uint64_t sample, uint64_t *offset)
{
    if (!d->in_data || d->state == ST_ERROR) return false;
    uint64_t block = (uint64_t)d->sample_bytes * (uint64_t)d->format.channels;
    if (sample > UINT64_MAX / block) return false;
    uint64_t at = sample * block;
    if (!d->data_open && at > d->data_size) return false;
    d->data_left = d->data_open ? 0 : d->data_size - at;
    d->skip_pending = 0;
    d->state = ST_DATA;
    d->consumed = d->data_start + at;
    *offset = d->data_start + at;
    return true;
}
