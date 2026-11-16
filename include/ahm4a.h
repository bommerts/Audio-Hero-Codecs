/* Audio Hero M4A: a reader for the AAC sound track of an MP4 file (.m4a,
   .mp4), in plain C, written from ISO/IEC 14496-12 and 14496-14. It finds
   the track, its decoder settings and where each packet lies; ahaac.h
   decodes the packets. No dependencies beyond the C standard library, and
   no file access of its own: the caller supplies a function that reads.

   Not handled: Apple Lossless (ALAC) and other codecs in the same kind of
   file, and fragmented files (the kind streaming services send in pieces). */
#ifndef AHM4A_H
#define AHM4A_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reads up to `bytes` bytes of the file, starting `offset` bytes in, into
   buffer; returns how many it read (fewer at the end of the file). */
typedef size_t (*ahm4a_read_fn)(void *user, uint64_t offset, void *buffer, size_t bytes);

typedef struct {
    int sample_rate;         /* Hz, as the track states it */
    int channels;
    uint64_t packets;        /* AAC frames in the track, 1024 samples each */
    int max_packet_bytes;    /* the biggest of them */
    const uint8_t *config;   /* the AudioSpecificConfig, for ahaac_decoder_configure */
    size_t config_bytes;
    /* For gapless playback: the decoded sound starts with `delay` samples
       that are not part of the music (the encoder's run-in), and `samples`
       are to be played after them (0 when the file does not say). Taken
       from the file's edit list, or else from an iTunes gapless tag. */
    uint32_t delay;
    uint64_t samples;
} ahm4a_info;

typedef struct ahm4a ahm4a;

/* Opens a file of `size` bytes through `read`. Returns NULL when it is not
   an MP4 file with an AAC track, and then *error (optional) says why. */
ahm4a *ahm4a_open(ahm4a_read_fn read, void *user, uint64_t size, const char **error);
void ahm4a_close(ahm4a *m);
const ahm4a_info *ahm4a_get_info(const ahm4a *m);

/* Where packet `index` (0 to packets - 1) lies in the file. False when
   there is no such packet. */
bool ahm4a_packet(const ahm4a *m, uint64_t index, uint64_t *offset, uint32_t *bytes);
/* Reads packet `index` into buffer; returns its size, or 0 when there is no
   such packet, it does not fit, or it could not be read. */
size_t ahm4a_read_packet(const ahm4a *m, uint64_t index, uint8_t *buffer, size_t room);

#ifdef __cplusplus
}
#endif

#endif
