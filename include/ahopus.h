/* Audio Hero Opus: an Opus decoder in plain C, after RFC 6716 (with the
   updates of RFC 8251) and RFC 7845, the way Ogg files hold Opus. It decodes
   every mode: SILK (speech), CELT (music) and the hybrid of the two, at every
   frame size, mono or stereo, and up to 8 channels in the streams of mapping
   family 0 or 1. Output is always 48 kHz. No dependencies beyond the C
   standard library.

   The decoder is fed the Ogg packets: first the OpusHead header, then the
   sound packets one at a time. Ogg's page framing is left to the caller
   (see ahogg.h); so is the start and end trim the pages state, apart from
   the header's pre-skip, which is the caller's to drop too. */
#ifndef AHOPUS_H
#define AHOPUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AHOPUS_MAX_CHANNELS 8
#define AHOPUS_MAX_SAMPLES 5760 /* per channel in one packet: 120 ms */

typedef struct {
    int sample_rate;       /* always 48000 */
    int channels;          /* 1 to 8 */
    uint32_t channel_mask; /* WAVE_FORMAT_EXTENSIBLE speaker bits, in output order */
    int pre_skip;          /* samples at the start to drop */
    int input_rate;        /* the rate the encoder was fed, for information only; 0 if not stated */
    int gain;              /* the header's output gain in 1/256 dB; the decoder applies it */
    int mapping_family;    /* 0 or 1 */
    int streams, coupled;  /* Opus streams in each packet, and how many of them are stereo */
} ahopus_format;

typedef struct ahopus_decoder ahopus_decoder;

ahopus_decoder *ahopus_decoder_new(void); /* NULL only when out of memory */
void ahopus_decoder_free(ahopus_decoder *d);

/* The first Ogg packet, "OpusHead". False when it is not one, or describes
   a stream this decoder does not handle (see ahopus_decoder_error). */
bool ahopus_decoder_header(ahopus_decoder *d, const uint8_t *head, size_t len, ahopus_format *format);

/* One sound packet. On return *samples is the samples per channel written
   to pcm as interleaved floats around [-1, 1], in WAV channel order. data
   NULL (or len 0) stands for a lost packet: it is covered over, for the
   length of the packet before. A damaged packet returns false and is covered
   over the same way, for the length its framing states (if it can be told,
   else *samples is 0). */
bool ahopus_decode_packet(ahopus_decoder *d, const uint8_t *data, size_t len,
                          float pcm[AHOPUS_MAX_SAMPLES * AHOPUS_MAX_CHANNELS], int *samples);

/* Forgets the sound so far, as after a seek. RFC 7845 asks for at least
   80 ms of sound to be decoded and dropped after a seek before the output
   is used; ahreader decodes 200 ms. CELT has settled by then; SILK's
   long-term predictor can carry steady tones on for longer. */
void ahopus_decoder_reset(ahopus_decoder *d);

/* NULL while all is well; otherwise why the stream cannot be decoded. */
const char *ahopus_decoder_error(const ahopus_decoder *d);

#ifdef __cplusplus
}
#endif

#endif
