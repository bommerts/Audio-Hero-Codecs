/* Audio Hero resampler: changes the sample rate of sound, in plain C. Any
   rate to any rate, 1 to 8 channels, fed a piece at a time. No dependencies
   beyond the C standard library.

   It is a windowed-sinc filter worked out for the two rates when the
   resampler is made. Sound up to 90.7% of the lower rate's half (20 kHz
   when either rate is 44.1 kHz) passes unchanged; everything from that half
   upwards, which the lower rate cannot carry, is removed. The output lines
   up with the input: its first sample is the sound at the input's first
   sample, and n input samples give ceil(n * out_rate / in_rate) samples. */
#ifndef AHRESAMPLE_H
#define AHRESAMPLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AHRESAMPLE_MAX_CHANNELS 8
#define AHRESAMPLE_MIN_RATE 1000
#define AHRESAMPLE_MAX_RATE 768000

typedef struct ahresample ahresample;

/* NULL when a rate or the channel count is out of range, or out of memory. */
ahresample *ahresample_new(int in_rate, int out_rate, int channels);
void ahresample_free(ahresample *r);
/* Forgets the sound so far, as after a seek: the next input starts afresh. */
void ahresample_reset(ahresample *r);

/* Converts interleaved floats. Takes up to in_frames samples per channel
   from in and writes up to out_room samples per channel to out. Returns the
   samples per channel written; *used is how many were taken from in. Call
   again with what is left, adding more as it arrives. The output runs a
   little behind the input: ahresample_flush gives the rest. */
size_t ahresample_process(ahresample *r, const float *in, size_t in_frames, size_t *used, float *out,
                          size_t out_room);

/* After the last input: writes the sound still inside. Call until it
   returns 0. More input may follow only after ahresample_reset. */
size_t ahresample_flush(ahresample *r, float *out, size_t out_room);

#ifdef __cplusplus
}
#endif

#endif
