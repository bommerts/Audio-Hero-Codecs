/* Audio Hero ALAC: an Apple Lossless decoder in plain C, written from the
   format Apple published with its reference code (the ALAC magic cookie
   and bit stream descriptions). 16, 20, 24 and 32 bits, 1 to 8 channels.
   No dependencies beyond the C standard library.

   ALAC comes in MP4/M4A files: ahm4a.h finds the settings (the "magic
   cookie") and the packets, and this decodes one packet at a time. Every
   packet stands alone, so a seek needs nothing more than starting at
   another packet. */
#ifndef AHALAC_H
#define AHALAC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AHALAC_MAX_CHANNELS 8
/* The most samples per channel a packet may hold. Encoders write 4096;
   settings that ask for more than this are refused. */
#define AHALAC_MAX_FRAME 65536

typedef struct {
    int sample_rate;       /* Hz */
    int channels;          /* 1 to AHALAC_MAX_CHANNELS */
    int bits;              /* per sample: 16, 20, 24 or 32 */
    int frame_samples;     /* per channel in a full packet; the last packet
                              of a file may hold fewer */
    uint32_t channel_mask; /* the speakers, as a WAV file's channel mask
                              numbers them (and ahmix.h); the channels come
                              out in that order */
} ahalac_format;

typedef struct ahalac_decoder ahalac_decoder;

ahalac_decoder *ahalac_decoder_new(void); /* NULL only when out of memory */
void ahalac_decoder_free(ahalac_decoder *d);

/* Sets the decoder up from the file's settings: the 24-byte
   ALACSpecificConfig, alone or inside the 'alac' (and 'frma') boxes that
   some files wrap it in. False when they cannot be used; then
   ahalac_decoder_error says why. format (optional) is filled in. */
bool ahalac_decoder_configure(ahalac_decoder *d, const uint8_t *config, size_t len, ahalac_format *format);

/* Decodes one packet. pcm gets the samples interleaved, as whole numbers of
   format.bits bits (a 16-bit file gives -32768 to 32767), in the order
   channel_mask names; it must have room for pcm_cap numbers, which must be
   at least frame_samples * channels. *samples is the number per channel.
   False when the packet is damaged or uses a part of the format this
   decoder does not handle (*samples is then 0). */
bool ahalac_decode_packet(ahalac_decoder *d, const uint8_t *data, size_t len, int32_t *pcm, size_t pcm_cap, int *samples);

/* NULL while all is well; otherwise why the settings cannot be used. */
const char *ahalac_decoder_error(const ahalac_decoder *d);

/* Converts samples of the given bit depth to floats in [-1, 1). */
void ahalac_to_float(const int32_t *in, float *out, size_t count, int bits);

#ifdef __cplusplus
}
#endif

#endif
