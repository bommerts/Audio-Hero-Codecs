/* Tables shared by the AAC decoder and its tests. */
#ifndef AHAAC_INTERNAL_H
#define AHAAC_INTERNAL_H

#include "../include/ahaac.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define AHAAC_PI 3.14159265358979323846

/* Sample rate by index 0..12. */
extern const int ahaac_sample_rates[13];

/* Scale factor differences: entry i codes the difference i - 60. */
extern const unsigned int ahaac_sf_codes[121];
extern const unsigned char ahaac_sf_bits[121];

/* A spectral code book. Entry i holds `dim` values; taking i as a number
   in base `mod`, its digits (most significant first) plus `low` are the
   values. With sign bits, a bit follows for every value that is not 0. */
typedef struct {
    const unsigned short *codes;
    const unsigned char *bits;
    short count;
    unsigned char dim, mod;
    signed char low;
    unsigned char sign_bits;
} ahaac_book;

extern const ahaac_book ahaac_books[12]; /* 1..11; book 11's value 16 is an escape */

/* Scale factor band boundaries for one sample rate and window size. */
typedef struct {
    const unsigned short *offset; /* count + 1 entries */
    int count;
} ahaac_bands;

extern const ahaac_bands ahaac_bands_long[13], ahaac_bands_short[13];
extern const unsigned char ahaac_tns_max_long[13], ahaac_tns_max_short[13];

/* Code book numbers with a meaning of their own. */
enum { AHAAC_ZERO = 0, AHAAC_ESCAPE = 11, AHAAC_NOISE = 13, AHAAC_INTENSITY2 = 14, AHAAC_INTENSITY = 15 };

/* Window sequences. */
enum { AHAAC_ONLY_LONG, AHAAC_LONG_START, AHAAC_EIGHT_SHORT, AHAAC_LONG_STOP };

/* The inverse transform, for testing: n spectral lines (1024 or 128) in,
   2n time samples out, scaled as the standard's formula times 1 / 32768. */
void ahaac_imdct(const ahaac_decoder *d, const float *spectrum, float *out, int n);

#endif
