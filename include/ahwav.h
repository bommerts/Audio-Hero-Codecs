/* Audio Hero WAV: a reader for WAV files (RIFF/WAVE and RF64) in plain C.
   Whole-number samples of 8, 16, 24 and 32 bits and floats of 32 and 64 bits.
   No dependencies beyond the C standard library. */
#ifndef AHWAV_H
#define AHWAV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AHWAV_MAX_CHANNELS 8

typedef struct {
    int sample_rate;  /* Hz; 0 until the header has been read */
    int channels;     /* 1 to AHWAV_MAX_CHANNELS */
    int bits;         /* per sample as stored: 8, 16, 24, 32 or 64 */
    bool is_float;
    uint64_t samples; /* per channel in the file; 0 when the file does not say */
    uint32_t channel_mask; /* which speaker each channel is for, from an
                              extensible header; 0 when the file does not say */
} ahwav_format;

typedef struct ahwav_decoder ahwav_decoder;

ahwav_decoder *ahwav_decoder_new(void); /* NULL only when out of memory */
void ahwav_decoder_free(ahwav_decoder *d);

/* Reads more of data[0..len).

   Returns the bytes used. *samples is the number of samples per channel
   written to pcm as interleaved floats around [-1, 1]; never more than
   pcm_cap floats in all. Call again with what is left, adding more data as it
   arrives, and with eof set once the file has ended. When a call uses no
   bytes and gives no samples, it needs more data, or (with eof) the file is
   done.

   format (optional) is filled in once the header has been read. */
size_t ahwav_decode(ahwav_decoder *d, const uint8_t *data, size_t len, bool eof, float *pcm, size_t pcm_cap,
                    int *samples, ahwav_format *format);

/* For a seek, once the header has been read: tells where in the file sample
   `sample` (per channel, from 0) lies, and from then on expects the data
   from that place. False before the header is read, or when the file is
   known to be shorter. The offset counts from the first byte ever handed
   to ahwav_decode, so hand it the file from its start. */
bool ahwav_decoder_seek(ahwav_decoder *d, uint64_t sample, uint64_t *offset);

/* NULL while all is well; otherwise why the file cannot be read (not a WAV
   file, or a kind of WAV this reader does not handle). */
const char *ahwav_decoder_error(const ahwav_decoder *d);

#ifdef __cplusplus
}
#endif

#endif
