/* Audio Hero Ogg: a reader for the sound stream of an Ogg file (.ogg,
   .oga, .opus), in plain C, written from RFC 3533 and the Vorbis, FLAC and
   Opus (RFC 7845) mappings onto Ogg. It finds the first Vorbis, FLAC or
   Opus stream in the file,
   collects its header packets, and hands over its sound packets one at a
   time, with the positions its pages give. It says how far the stream
   runs and finds where to start reading for a seek; ahvorbis.h, ahflac.h
   or ahopus.h decodes the packets. No dependencies beyond the C standard library, and
   no file access of its own: the caller supplies a function that reads.

   Not handled: a chained file (one stream after another, as recordings of
   internet radio often are) gives only its first stream. */
#ifndef AHOGG_H
#define AHOGG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reads up to `bytes` bytes of the file, starting `offset` bytes in, into
   buffer; returns how many it read (fewer at the end of the file). */
typedef size_t (*ahogg_read_fn)(void *user, uint64_t offset, void *buffer, size_t bytes);

/* What the stream holds. */
typedef enum {
    AHOGG_VORBIS = 0,
    AHOGG_FLAC,
    AHOGG_OPUS,
} ahogg_codec;

typedef struct {
    ahogg_codec codec;
    uint32_t serial;      /* the stream's number in the file */
    int headers;          /* header packets at the start of the stream: 3 for
                             Vorbis, 2 for Opus; for FLAC the first packet
                             and one per block of information after it */
    int64_t last_granule; /* the granule position of the stream's last page
                             that has one: where its sound ends, in samples
                             per channel from the stream's start (Vorbis and
                             FLAC count so; Opus counts at 48 kHz, its
                             pre-skip included); -1 when no page has one */
} ahogg_info;

typedef struct ahogg ahogg;

/* Opens a file of `size` bytes through `read`. Returns NULL when it is not
   an Ogg file with a Vorbis, FLAC or Opus stream, or its headers are
   damaged; then
   *error (optional) says why. */
ahogg *ahogg_open(ahogg_read_fn read, void *user, uint64_t size, const char **error);
void ahogg_close(ahogg *o);
const ahogg_info *ahogg_get_info(const ahogg *o);

/* Header packet `index` (0 for the first). The first three are kept;
   others are counted but not kept. */
bool ahogg_header(const ahogg *o, int index, const uint8_t **data, size_t *bytes);

/* The next sound packet. *data and *bytes stay good until the next call.
   *granule is the granule position the page states for the end of this
   packet; only the last packet to end on a page has one, and the others get
   -1. False at the end of the stream. A page whose checksum fails is left
   out, with the packets on it; a packet with a piece missing is dropped. */
bool ahogg_packet(ahogg *o, const uint8_t **data, size_t *bytes, int64_t *granule);

/* True when the packet ahogg_packet last handed over ended on the page
   marked as the stream's last. That page's granule position says where
   the sound ends, where another page's may say where it starts. */
bool ahogg_last_page(const ahogg *o);

/* Gets ready to seek to granule position `granule`. True: reading goes on
   from a place a little before it, at the start of a packet; where those
   packets lie is not known until the first granule position handed over
   after this, which is at or below `granule` (unless the packet that would
   carry it is damaged and left out). False: no such place comes before,
   and reading goes back to the stream's first sound packet. For a codec
   that needs packets before the target to settle (Opus), aim that much
   earlier. */
bool ahogg_seek(ahogg *o, int64_t granule);

#ifdef __cplusplus
}
#endif

#endif
