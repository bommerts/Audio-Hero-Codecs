/* What an ALAC decoder and an encoder share: the settings block, the
   elements a packet is made of, the rules of the adaptive Golomb code, and
   the channel layouts. Also used by the tests, whose small ALAC writer is
   built on it. */
#ifndef AHALAC_INTERNAL_H
#define AHALAC_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The ALACSpecificConfig ("magic cookie"): 24 bytes, big-endian. */
#define AHALAC_SETUP_BYTES 24

typedef struct {
    uint32_t frame_length;    /* samples per channel in a full packet; usually 4096 */
    uint8_t version;          /* compatible version: 0 */
    uint8_t bits;             /* per sample */
    uint8_t pb;               /* how fast the code's running mean follows the values; usually 40 */
    uint8_t mb;               /* the running mean's start; usually 10 */
    uint8_t kb;               /* the largest Golomb parameter; usually 14 */
    uint8_t channels;
    uint16_t max_run;         /* usually 255; not used in decoding */
    uint32_t max_frame_bytes; /* 0 when not known */
    uint32_t avg_bit_rate;    /* 0 when not known */
    uint32_t sample_rate;
} ahalac_setup;

/* Reads the settings: the 24 bytes alone, or after an 'frma' box header
   and an 'alac' box header (12 bytes each), as older files have them.
   False when there are too few bytes. Nothing is checked. */
bool ahalac_setup_read(const uint8_t *p, size_t len, ahalac_setup *s);
void ahalac_setup_write(const ahalac_setup *s, uint8_t out[AHALAC_SETUP_BYTES]);

/* The elements of a packet, each named by 3 bits. */
enum {
    AHALAC_SCE = 0, /* one channel */
    AHALAC_CPE = 1, /* a pair */
    AHALAC_CCE = 2, /* not used by ALAC */
    AHALAC_LFE = 3, /* one channel, the bass effects */
    AHALAC_DSE = 4, /* data to skip */
    AHALAC_PCE = 5, /* not used by ALAC */
    AHALAC_FIL = 6, /* fill to skip */
    AHALAC_END = 7,
};

/* ---- the adaptive Golomb code -----------------------------------------------------

   Each value is coded with a parameter k worked out from a running mean
   (mb) of the values before it. A value n is n / (2^k - 1) one bits, a
   zero, and then n % (2^k - 1) + 1 in k bits, or just k - 1 zero bits when
   the remainder is 0. Nine one bits instead mean the value follows as it
   is (in the sample's width; 16 bits for a run of zeros). When the mean
   falls low, a count of zero values follows each value. */
#define AHALAC_QB_SHIFT 9    /* the mean is kept with 9 bits of fraction */
#define AHALAC_MAX_PREFIX 9  /* one bits that mean "the value follows as it is" */
#define AHALAC_RUN_BITS 16   /* the width of a run of zeros sent as it is */
#define AHALAC_RUN_LONG 65535 /* a run this long is not followed by a value one smaller */
#define AHALAC_MEAN_CLAMP 0xFFFFu

static inline int ahalac_log2(uint32_t v)
{
    int n = 0;
    while (v >>= 1) n++;
    return n;
}

/* The parameter for the next value. */
static inline int ahalac_value_k(uint32_t mb, int kb)
{
    int k = ahalac_log2((mb >> AHALAC_QB_SHIFT) + 3);
    return k < kb ? k : kb;
}

/* True when a run of zeros follows: the mean has fallen under 0.25. */
static inline bool ahalac_run_follows(uint32_t mb)
{
    return mb < (1u << AHALAC_QB_SHIFT) / 4;
}

/* The parameter for a run of zeros: Apple's 32 - 24 when the mean is 0. */
static inline int ahalac_run_k(uint32_t mb)
{
    return (mb ? 7 - ahalac_log2(mb) : 8) + (int)((mb + 16) >> 6);
}

/* The divisor that goes with a run's parameter: 2^k - 1, cut to the
   largest parameter allowed. */
static inline uint32_t ahalac_run_m(int k, int kb)
{
    uint32_t wb = kb >= 32 ? 0xFFFFFFFFu : (1u << kb) - 1;
    return ((1u << k) - 1) & wb;
}

/* The running mean after a value: `n` as coded, `zmode` 1 when it follows a
   run of zeros (and so was coded one smaller). Unsigned arithmetic, wrapping
   as Apple's does. */
static inline uint32_t ahalac_mean(uint32_t mb, uint32_t pb, uint32_t n, uint32_t zmode)
{
    if (n > AHALAC_MEAN_CLAMP) return AHALAC_MEAN_CLAMP;
    return pb * (n + zmode) + mb - ((pb * mb) >> AHALAC_QB_SHIFT);
}

/* ---- channels ------------------------------------------------------------------- */

/* For each channel count: the elements a packet holds, in order (Apple's
   layouts: C; L R; C L R; C L R Cs; C L R Ls Rs; C L R Ls Rs LFE;
   C L R Ls Rs Cs LFE; C Lc Rc L R Ls Rs LFE). */
extern const uint8_t ahalac_elements[8][5];
extern const uint8_t ahalac_element_count[8];
/* Where the channel the packet carries at place i goes in WAV order. */
extern const uint8_t ahalac_channel_place[8][8];
/* The WAV channel mask of each count. */
extern const uint32_t ahalac_channel_mask[8];

#endif
