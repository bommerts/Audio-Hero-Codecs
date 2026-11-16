/* Writing Layer III frames: header, side information, scale factors and
   Huffman data. Used by the encoder, and by the tests to build frames with
   every feature of the format. */
#ifndef AHMP3_BITSTREAM_H
#define AHMP3_BITSTREAM_H

#include "ahmp3_internal.h"

/* One granule of one channel, ready to be written. */
typedef struct {
    int part23;            /* bits of scale factors and Huffman data */
    int big_values, global_gain, scalefac_compress;
    int window_switching, block_type, mixed;
    int table[3], subblock_gain[3];
    int region0_count, region1_count;
    int preflag, scalefac_scale, count1_table;
    int scfsi;             /* MPEG-1 second granule: groups reused from the first */
    int long_end, short_start;
    int count1_end;        /* the coded lines are [0, count1_end) */
    unsigned char sf[40];  /* long bands, then short bands x 3 windows */
    int ix[576];           /* quantized values, in bitstream order */
} ahmp3_gr;

typedef struct {
    int lsf, sr_index, bitrate_index, padding, mode, mode_ext, channels, granules;
    int crc;               /* write a CRC after the header */
    int main_data_begin;
    ahmp3_gr gr[2][2];
} ahmp3_fr;

/* Header (with its CRC when asked for) plus side information; returns its
   bytes. */
int ahmp3_write_header(const ahmp3_fr *f, unsigned char *out);
/* Scale factors and Huffman data of every granule; returns bits written. */
int ahmp3_write_main(const ahmp3_fr *f, unsigned char *out, int cap);

/* True for the right channel of an MPEG-2 intensity stereo frame, whose
   scale factors are coded differently. */
bool ahmp3_intensity_right(const ahmp3_fr *f, int ch);

/* Scale factor bits. first is the frame's first granule of the same channel
   when this is an MPEG-1 second granule (for scfsi), else NULL. */
int ahmp3_part2_bits(const ahmp3_gr *G, int lsf, bool intensity_right, const ahmp3_gr *first);
/* Picks the cheapest scalefac_compress that holds the scale factors. False
   when none does. */
bool ahmp3_choose_scalefac_compress(ahmp3_gr *G, int lsf);
/* Largest value scale factor n (flat index) can take. */
int ahmp3_sf_limit(const ahmp3_gr *G, int lsf, int n);

/* Fills big_values, count1_end, the tables and the region split for the
   values in G->ix; returns the Huffman bits. best_split tries every region
   split (slower, fewer bits). */
int ahmp3_huffman_choose(ahmp3_gr *G, int sr_index, bool best_split);
/* Huffman bits for the side information already in G. */
int ahmp3_huffman_bits(const ahmp3_gr *G, int sr_index);

#endif
