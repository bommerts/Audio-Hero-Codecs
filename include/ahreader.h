/* Audio Hero reader: one way to read every kind of sound file the decoders
   here handle. It tells the kind from the file's contents, decodes it a
   piece at a time, drops the run-in and padding of gapless files, says how
   long the file is, and seeks. Plain C, no dependencies beyond the C
   standard library, and no file access of its own: the caller supplies a
   function that reads.

   Kinds: MP3 and MPEG Layers I and II, FLAC, AAC as ADTS (.aac) or in MP4
   (.m4a), Apple Lossless (in .m4a), WAV, and Vorbis, FLAC or Opus in Ogg
   (.ogg, .oga, .opus). See each decoder's header for what it does not
   handle. */
#ifndef AHREADER_H
#define AHREADER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reads up to `bytes` bytes of the file, starting `offset` bytes in, into
   buffer; returns how many it read (fewer only at the end of the file). */
typedef size_t (*ahreader_read_fn)(void *user, uint64_t offset, void *buffer, size_t bytes);

typedef enum {
    AHREADER_MP3 = 1,
    AHREADER_MP2,
    AHREADER_MP1,
    AHREADER_FLAC,
    AHREADER_AAC, /* ADTS */
    AHREADER_M4A, /* AAC in MP4 */
    AHREADER_WAV,
    AHREADER_ALAC, /* Apple Lossless, in MP4 */
    AHREADER_VORBIS,   /* Vorbis, in Ogg */
    AHREADER_OGG_FLAC, /* FLAC, in Ogg */
    AHREADER_OPUS,     /* Opus, in Ogg; always 48 kHz */
} ahreader_kind;

typedef struct {
    ahreader_kind kind;
    const char *name;      /* "MP3", "FLAC" ... */
    int sample_rate;       /* Hz, of the sound ahreader_read gives */
    int channels;          /* 1 to 8 */
    uint32_t channel_mask; /* which speaker each channel is for, as ahmix.h
                              numbers them; 0 when the file does not say */
    int bits;              /* per sample of a FLAC (native or in Ogg), ALAC or
                              whole-number WAV file, else 0 */
    uint64_t samples;      /* per channel in the file; 0 when it cannot be told */
    bool samples_exact;    /* false: samples is worked out from the file's
                              size and bit rate (an MP3 file without a
                              length header) and may be a little off */
    bool half_rate;        /* an HE-AAC file, played as the plain AAC inside
                              it: half the sample rate, no top treble. A
                              caller that wants the whole sound hands such
                              a file to the system's decoder. */
    bool parametric_stereo; /* an HE-AAC v2 file that says it uses
                              parametric stereo: it plays here in mono. An
                              ADTS file cannot say so; one with half_rate
                              set and one channel may use it too. */
} ahreader_info;

typedef struct ahreader ahreader;

/* Opens a file of `size` bytes through `read`. Returns NULL when it is not
   a sound file of a kind handled, and then *error (optional) says why. */
ahreader *ahreader_open(ahreader_read_fn read, void *user, uint64_t size, const char **error);
void ahreader_close(ahreader *r);
const ahreader_info *ahreader_get_info(const ahreader *r);

/* Gives the next samples as interleaved floats around [-1, 1]: up to
   max_frames per channel. Returns how many per channel it gave; 0 means
   the file has ended. */
size_t ahreader_read(ahreader *r, float *pcm, size_t max_frames);

/* Moves to sample `sample` (per channel, from 0): the next ahreader_read
   starts there, to the sample. False when the file has no such place; the
   reader is then at its end. */
bool ahreader_seek(ahreader *r, uint64_t sample);

#ifdef __cplusplus
}
#endif

#endif
