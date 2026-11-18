/* Parts of the Vorbis decoder shared with its tests. */
#ifndef AHVORBIS_INTERNAL_H
#define AHVORBIS_INTERNAL_H

#include "../include/ahvorbis.h"

/* The specification's table of floor values, from about -140 dB to 0 dB. */
extern const float ahvorbis_inverse_db[256];

/* Draws a type 1 floor's curve into out[0 .. n): straight lines between
   the `count` points that count, in order of place (x[0] is 0), each line
   drawn whole and then cut at n, and the last height carried on to n.
   Heights are already times the floor's multiplier. */
void ahvorbis_floor1_lines(const int *x, const int *y, int count, float *out, int n);

#endif
