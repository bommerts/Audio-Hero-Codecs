/* Audio Hero reader: one way to read every kind of sound file the decoders
   here handle. It tells the kind from the file's contents, decodes it a
   piece at a time, drops the run-in and padding of gapless files, says how
   long the file is, and seeks. Plain C, no dependencies beyond the C
   standard library, and no file access of its own: the caller supplies a
   function that reads.

   Kinds: MP3 and MPEG Layers I and II, FLAC, AAC as ADTS (.aac) or in MP4
   (.m4a), Apple Lossless (in .m4a), WAV, and Vorbis, FLAC or Opus in Ogg
   (.ogg, .oga, .opus). See each decoder's header for what it does not
   handle.

   Chained Ogg (one stream after another, as internet radio sends a track
   at a time, or files joined end to end) plays every stream of the first
   one's kind in turn; `stream` in the info says which one a read came
   from. A stream whose sample rate, channels or bits differ ends the sound
   unless the caller asks for ahreader_follow_changes. Internet radio and
   other sources that can only be read in order open with
   ahreader_open_stream. */
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

#define AHREADER_MAX_CHANNELS 8

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
    int stream;            /* chained Ogg: which stream of the chain the
                              samples the last ahreader_read gave come from,
                              0 for the first (a stream read in order counts
                              from where it was joined). Each read gives the
                              samples of one stream only, and the info
                              describes them: when stream changes, the
                              comments are the new stream's, and with
                              ahreader_follow_changes sample_rate, channels,
                              channel_mask and bits may change too. 0 for
                              every other kind. */
} ahreader_info;

typedef struct ahreader ahreader;

/* Opens a file of `size` bytes through `read`. Returns NULL when it is not
   a sound file of a kind handled, and then *error (optional) says why. */
ahreader *ahreader_open(ahreader_read_fn read, void *user, uint64_t size, const char **error);

/* Opens a source that can only be read in order, of no known length, such
   as internet radio. Ogg only (Vorbis, FLAC or Opus). `read` is asked for
   bytes in order, each offset following on from the bytes it gave before,
   and never for more than the next page needs; it may give fewer than
   asked, as they come, and gives 0 only where the source ends. It may be
   joined anywhere: what comes before the first stream's start (a new
   track) is passed over. samples is 0; ahreader_seek gives false and
   reading goes on. Chains are followed, with each new stream's headers;
   radio that may change its sample rate or channels between tracks wants
   ahreader_follow_changes. */
ahreader *ahreader_open_stream(ahreader_read_fn read, void *user, const char **error);
void ahreader_close(ahreader *r);
const ahreader_info *ahreader_get_info(const ahreader *r);

/* Gives the next samples as interleaved floats around [-1, 1]: up to
   max_frames per channel. Returns how many per channel it gave; 0 means
   the file has ended. pcm has room for max_frames * channels floats, or
   for max_frames * AHREADER_MAX_CHANNELS with ahreader_follow_changes. */
size_t ahreader_read(ahreader *r, float *pcm, size_t max_frames);

/* Chained Ogg: lets the sound go on into a stream whose sample rate,
   channels or bits differ from those the info gave so far; the info
   describes each read's samples. pcm must then have room for max_frames *
   AHREADER_MAX_CHANNELS floats. Off (false) by default, so that a caller
   that made room for the channels at opening is safe: the sound then ends
   where such a stream starts (as in ffmpeg), and samples counts only the
   streams before it. With it on, samples counts every stream, each at its
   own rate. */
void ahreader_follow_changes(ahreader *r, bool follow);

/* Ogg: comment `index` (0 for the first) of the stream the samples the
   last read gave come from (before any read, the first stream's), as
   "NAME=value" in UTF-8, the way Vorbis comments are; not ended by a 0
   byte. False past the last, and for other kinds of file. */
bool ahreader_comment(const ahreader *r, int index, const char **text, size_t *bytes);

/* Moves to sample `sample` (per channel, from 0; in a chain, counting the
   samples of every stream played before): the next ahreader_read starts
   there, to the sample. False when the file has no such place; the reader
   is then at its end. */
bool ahreader_seek(ahreader *r, uint64_t sample);

#ifdef __cplusplus
}
#endif

#endif
