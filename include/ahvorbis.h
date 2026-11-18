/* Audio Hero Vorbis: a Vorbis decoder in plain C, written from the Vorbis I
   specification. All of the format: both floor types, the three residue
   types, both kinds of codebook value table, any number of modes, mappings
   and coupling steps, 1 to 8 channels. No dependencies beyond the C
   standard library.

   Vorbis comes in Ogg files: ahogg.h finds the stream and hands over its
   packets, and this decodes one packet at a time. The first three packets
   are headers; every packet after them is sound. */
#ifndef AHVORBIS_H
#define AHVORBIS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AHVORBIS_MAX_CHANNELS 8
/* The most samples per channel one packet gives. */
#define AHVORBIS_MAX_SAMPLES 4096

typedef struct {
    int sample_rate;       /* Hz */
    int channels;          /* 1 to AHVORBIS_MAX_CHANNELS */
    uint32_t channel_mask; /* the speakers, as a WAV file's channel mask
                              numbers them (and ahmix.h); the channels come
                              out in that order */
    int block_short;       /* the two transform sizes the stream uses: */
    int block_long;        /* 64 to 8192 samples */
    int bitrate;           /* bits per second the encoder aimed at; 0 when
                              it does not say */
} ahvorbis_format;

typedef struct ahvorbis_decoder ahvorbis_decoder;

ahvorbis_decoder *ahvorbis_decoder_new(void); /* NULL only when out of memory */
void ahvorbis_decoder_free(ahvorbis_decoder *d);

/* Takes the three header packets, in order: identification, comment and
   setup. False when a packet is not the header expected or the stream
   cannot be decoded; then ahvorbis_decoder_error says why. format
   (optional) is filled in once the first has been taken. */
bool ahvorbis_decoder_header(ahvorbis_decoder *d, const uint8_t *packet, size_t len, ahvorbis_format *format);

/* Decodes one sound packet. pcm gets the samples as interleaved floats
   around [-1, 1], in the order channel_mask names; it must have room for
   AHVORBIS_MAX_SAMPLES * channels. *samples is the number per channel.

   Each packet's sound overlaps the next one's, so the first packet (and
   the first after ahvorbis_decoder_reset) gives no samples: it only
   starts the overlap. False when the packet is damaged or not a sound
   packet (*samples is then 0); the packet is then left out, as the
   specification says. */
bool ahvorbis_decode_packet(ahvorbis_decoder *d, const uint8_t *data, size_t len, float *pcm, int *samples);

/* Forgets the overlap after a seek: the next packet starts afresh. */
void ahvorbis_decoder_reset(ahvorbis_decoder *d);

/* NULL while all is well; otherwise why the headers cannot be used. */
const char *ahvorbis_decoder_error(const ahvorbis_decoder *d);

#ifdef __cplusplus
}
#endif

#endif
