/* Audio Hero mix: turns sound of 1 to 8 channels into stereo, in plain C.
   No dependencies beyond the C standard library.

   One channel goes to both sides as it is. Two pass through. More are
   mixed down by where each speaker stands: the centre and the surrounds go
   in 3 dB down, a back centre 6 dB down to each side, and the bass effects
   channel (LFE) is left out. The whole mix is then turned down just far
   enough that full-scale sound in every channel cannot clip. */
#ifndef AHMIX_H
#define AHMIX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AHMIX_MAX_CHANNELS 8

/* Speakers, as the channel mask of a WAV file numbers them. A layout is
   these added together; the channels of the sound come in this order. */
#define AHMIX_FRONT_LEFT 0x1u
#define AHMIX_FRONT_RIGHT 0x2u
#define AHMIX_FRONT_CENTER 0x4u
#define AHMIX_LFE 0x8u
#define AHMIX_BACK_LEFT 0x10u
#define AHMIX_BACK_RIGHT 0x20u
#define AHMIX_FRONT_LEFT_OF_CENTER 0x40u
#define AHMIX_FRONT_RIGHT_OF_CENTER 0x80u
#define AHMIX_BACK_CENTER 0x100u
#define AHMIX_SIDE_LEFT 0x200u
#define AHMIX_SIDE_RIGHT 0x400u

/* The layout a file of this many channels has when it does not say: the
   one FLAC lays down for each count, which is also the usual one for WAV.
   0 when the count is out of range. */
uint32_t ahmix_default_layout(int channels);

typedef struct {
    int channels;
    float left[AHMIX_MAX_CHANNELS]; /* how much of each channel goes to the left */
    float right[AHMIX_MAX_CHANNELS];
} ahmix;

/* Works out the mix for sound of `channels` channels laid out as `layout`
   says. A layout of 0, or one that does not name exactly that many
   speakers, means the default for the count. False when the count is not 1
   to AHMIX_MAX_CHANNELS. */
bool ahmix_init(ahmix *m, int channels, uint32_t layout);

/* Mixes `frames` samples per channel of interleaved floats down to
   interleaved stereo. out may be the same buffer as in, given room for the
   stereo. */
void ahmix_to_stereo(const ahmix *m, const float *in, size_t frames, float *out);

#ifdef __cplusplus
}
#endif

#endif
