/* Shared tables and helpers for the decoder and the encoder. */
#ifndef AHMP3_INTERNAL_H
#define AHMP3_INTERNAL_H

#include "../include/ahmp3.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define AHMP3_PI 3.14159265358979323846

/* ---- stream constants (ISO/IEC 11172-3 and 13818-3) ---------------------- */

/* Sample rate index 0..8: 44100, 48000, 32000 (MPEG-1), 22050, 24000, 16000
   (MPEG-2), 11025, 12000, 8000 (MPEG-2.5). */
extern const int ahmp3_sample_rates[9];
/* Layer III: [lsf][bitrate index], kbps. lsf = 1 for MPEG-2 and 2.5. */
extern const int ahmp3_bitrates[2][15];
/* Layers I and II: [lsf][layer - 1][bitrate index], kbps. */
extern const int ahmp3_bitrates_12[2][2][15];

/* Scale factor band boundaries in lines, per sample rate index. */
extern const short ahmp3_sfb_long[9][23];
extern const short ahmp3_sfb_short[9][14];
extern const unsigned char ahmp3_pretab[22];
/* MPEG-1 scalefac_compress -> slen1, slen2. */
extern const unsigned char ahmp3_slen[2][16];
/* MPEG-2 scale factor counts: [table][long, short, mixed][partition]. */
extern const unsigned char ahmp3_lsf_nsf[6][3][4];

/* ISO synthesis window D[0..256] scaled by 65536; the rest follows by symmetry. */
extern const int ahmp3_window_half[257];

/* ---- Huffman tables (ahmp3_huff.c) ---------------------------------------- */

typedef struct {
    unsigned char size;    /* values per dimension; 0 = table not used */
    unsigned char linbits; /* extra bits after the escape value 15 */
    const unsigned short *codes;
    const unsigned char *lens;
} ahmp3_huff;

extern const ahmp3_huff ahmp3_huff_tables[32];
extern const unsigned char ahmp3_count1_codes[2][16];
extern const unsigned char ahmp3_count1_lens[2][16];

/* ---- derived float tables -------------------------------------------------- */

typedef struct {
    float D[512];           /* synthesis window */
    float imdct_long[18][36]; /* [k][i] = cos(pi/72 (2i + 1 + 18)(2k + 1)) */
    float imdct_short[6][12];
    float win[4][36];       /* by block type; [2] is the 12-point short window */
    float cs[8], ca[8];     /* alias reduction butterflies */
    float pow43[8207];      /* n^(4/3) */
} ahmp3_tables;

/* Fills the tables shared by the decoder and the encoder. */
void ahmp3_tables_init(ahmp3_tables *t);

/* ---- frame header ---------------------------------------------------------- */

typedef struct {
    int lsf;       /* 0 = MPEG-1, 1 = MPEG-2 or 2.5 */
    int mpeg25;
    int layer;     /* 1, 2 or 3 */
    int sr_index;  /* 0..8 */
    int bitrate_index;
    int free_format; /* bit rate index 0: the frame size is not in the header */
    int kbps;      /* 0 for free format until the decoder has worked it out */
    int padding;
    int crc;       /* a 16-bit CRC follows the header */
    int mode;      /* 0 stereo, 1 joint, 2 dual, 3 mono */
    int mode_ext;
    int channels;
    int samples;   /* per channel in the frame */
    int frame_bytes; /* 0 for free format until the decoder has worked it out */
    int side_bytes;  /* Layer III side information */
} ahmp3_header;

bool ahmp3_read_header(const unsigned char *p, ahmp3_header *h);
void ahmp3_fill_frame(const ahmp3_header *h, ahmp3_frame *frame);

/* The CRC-16 of MPEG audio frames (polynomial 0x8005), over the first
   `bits` bits of p. Starts from 0xFFFF. */
unsigned ahmp3_crc16(unsigned crc, const unsigned char *p, size_t bits);

/* ---- Layers I and II (ahmp3_layer12.c) ------------------------------------- */

/* A Layer II quantization class: how many levels, the bits of one code word,
   and whether a code word holds three samples at once. */
typedef struct {
    unsigned short levels;
    unsigned char bits, grouped;
} ahmp3_l2_class;

extern const ahmp3_l2_class ahmp3_l2_classes[17];

/* The Layer II bit allocation table for a stream: the number of subbands
   coded, and for each the bits of its allocation code (bits[sb]) and the
   class each code value selects (rows[sb][value - 1]). */
typedef struct {
    int sblimit;
    unsigned char bits[32];
    const unsigned char *rows[32];
} ahmp3_l2_table;

void ahmp3_l2_select(int lsf, int sample_rate, int kbps_per_channel, ahmp3_l2_table *t);

/* Turns a Layer I or II frame into subband samples out[ch][slot][subband]:
   12 slots for Layer I, 36 for Layer II. False when the frame's CRC does not
   match; out is then silence. */
bool ahmp3_layer12_decode(const ahmp3_header *h, const unsigned char *frame, float out[2][36][32]);

/* ---- bits ------------------------------------------------------------------- */

typedef struct {
    const unsigned char *p;
    size_t bits; /* total */
    size_t pos;
} ahmp3_bits;

static inline unsigned ahmp3_get_bits(ahmp3_bits *b, int n)
{
    unsigned v = 0;
    while (n-- > 0) {
        unsigned bit = 0;
        if (b->pos < b->bits) bit = (b->p[b->pos >> 3] >> (7 - (b->pos & 7))) & 1;
        b->pos++;
        v = (v << 1) | bit;
    }
    return v;
}

#endif
