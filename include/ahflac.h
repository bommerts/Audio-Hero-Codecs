/* Audio Hero FLAC: a FLAC decoder in plain C, written from the format
   specification (RFC 9639). Every bit depth from 4 to 32, up to 8 channels.
   No dependencies beyond the C standard library. */
#ifndef AHFLAC_H
#define AHFLAC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AHFLAC_MAX_CHANNELS 8

typedef struct {
    int sample_rate;     /* Hz; 0 until the file's header has been read */
    int channels;        /* 1 to AHFLAC_MAX_CHANNELS */
    int bits;            /* per sample: 4 to 32 */
    int max_block;       /* most samples per channel one frame holds */
    int max_frame_bytes; /* most bytes one frame takes: hand ahflac_decode at
                            least this much at a time, or all that is left */
    uint64_t samples;    /* per channel in the file; 0 when the file does not say */
} ahflac_format;

typedef struct ahflac_decoder ahflac_decoder;

ahflac_decoder *ahflac_decoder_new(void); /* NULL only when out of memory */
void ahflac_decoder_free(ahflac_decoder *d);
/* Forgets the frame in hand after a seek; the next call looks for the next
   frame in whatever data it is given. */
void ahflac_decoder_reset(ahflac_decoder *d);

/* Decodes more of data[0..len).

   Returns the bytes used: the header, any junk skipped, or one frame.
   *samples is the number of samples per channel written to pcm, interleaved,
   as whole numbers of format->bits bits (so a 16-bit file gives -32768 to
   32767); never more than pcm_cap numbers in all. A frame bigger than the
   room given comes out over several calls. Call again with what is left,
   adding more data as it arrives, and with eof set once the file has ended.
   When a call uses no bytes and gives no samples, it needs more data, or
   (with eof) the file is done.

   A damaged frame is skipped. format (optional) is filled in once the header
   has been read. */
size_t ahflac_decode(ahflac_decoder *d, const uint8_t *data, size_t len, bool eof, int32_t *pcm, size_t pcm_cap,
                     int *samples, ahflac_format *format);

/* NULL while all is well; otherwise why the file cannot be read. */
const char *ahflac_decoder_error(const ahflac_decoder *d);

/* Converts samples of the given bit depth to floats in [-1, 1). */
void ahflac_to_float(const int32_t *in, float *out, size_t count, int bits);

#ifdef __cplusplus
}
#endif

#endif
