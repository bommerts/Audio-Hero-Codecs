/* Audio Hero MP3: an MPEG audio decoder (Layers I, II and III of MPEG-1, 2
   and 2.5) and an MP3 encoder in plain C. No dependencies beyond the C
   standard library. */
#ifndef AHMP3_H
#define AHMP3_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Most samples one frame holds, per channel. */
#define AHMP3_MAX_SAMPLES 1152

/* Samples every Layer III decoder outputs before the encoder's first sample
   (filter bank delay). Gapless players drop this plus the encoder delay.
   For Layers I and II it is 481. */
#define AHMP3_DECODER_DELAY 529

/* ---- decoding ------------------------------------------------------------- */

typedef struct {
    int version;      /* 1 = MPEG-1, 2 = MPEG-2, 25 = MPEG-2.5 */
    int layer;        /* 1, 2 or 3 (3 is MP3) */
    int sample_rate;  /* Hz */
    int channels;     /* 1 or 2 */
    int bitrate_kbps; /* a free-format frame: worked out from its size by
                         ahmp3_decode; 0 from ahmp3_parse_header */
    int frame_bytes;  /* the whole frame, header included (free format: as
                         for bitrate_kbps) */
    int samples;      /* per channel: 1152, 576 (MPEG-2 Layer III) or 384 (Layer I) */
    bool crc_error;   /* the frame carries a CRC and it does not match: its
                         sound was replaced by silence */
} ahmp3_frame;

/* What an Xing, Info or VBRI header in a file's first frame says. */
typedef struct {
    bool present;
    uint32_t frames;     /* audio frames in the file, 0 when not given */
    uint32_t bytes;      /* audio bytes in the file, 0 when not given */
    int encoder_delay;   /* samples the encoder added at the start, -1 when not given */
    int encoder_padding; /* samples the encoder added at the end, -1 when not given */
} ahmp3_info;

typedef struct ahmp3_decoder ahmp3_decoder;

ahmp3_decoder *ahmp3_decoder_new(void); /* NULL only when out of memory */
void ahmp3_decoder_free(ahmp3_decoder *d);
/* Forgets the stream after a seek: bit reservoir, filter history, sync. */
void ahmp3_decoder_reset(ahmp3_decoder *d);

/* Decodes the next frame of data[0..len).

   Returns the bytes used: any junk or tags skipped, plus the frame. Returns 0
   when a whole frame is not there yet; call again with more data, or with
   eof set once the stream has ended (0 then means nothing is left).

   On return *samples is the number of samples per channel written to pcm, as
   interleaved floats around [-1, 1]. It is 0 when the bytes used held no
   sound: skipped junk, or a file's Xing/Info header frame. A frame whose bit
   reservoir is missing (just after a seek), or whose CRC does not match,
   gives silence of its length. frame (optional) describes the frame decoded.

   A free-format stream (one whose frames do not state their bit rate) needs
   the start of the next two frames to find its frame size, so hand over at
   least 8200 bytes at a time, or all that is left. */
size_t ahmp3_decode(ahmp3_decoder *d, const uint8_t *data, size_t len, bool eof,
                    float pcm[AHMP3_MAX_SAMPLES * 2], int *samples, ahmp3_frame *frame);

/* The Xing/Info/VBRI header the decoder found, if any. */
const ahmp3_info *ahmp3_decoder_info(const ahmp3_decoder *d);

/* Size of an ID3v2 tag at the start of data, or 0. Needs 10 bytes. */
size_t ahmp3_id3v2_size(const uint8_t *data, size_t len);

/* Reads the header of the frame at data. False when it is not a valid
   frame header. */
bool ahmp3_parse_header(const uint8_t *data, size_t len, ahmp3_frame *frame);

/* Converts floats to 16-bit with rounding and clipping. */
void ahmp3_to_s16(const float *in, int16_t *out, size_t count);

/* ---- encoding ------------------------------------------------------------- */

typedef enum {
    AHMP3_JOINT_STEREO, /* left/right or mid/side, chosen per frame */
    AHMP3_STEREO,       /* always left/right */
    AHMP3_MONO,         /* one channel; stereo input is mixed down */
} ahmp3_mode;

typedef struct {
    int sample_rate;  /* 32000, 44100, 48000; 16000, 22050, 24000; 8000, 11025, 12000 */
    int channels;     /* channels in the input: 1 or 2 */
    int bitrate_kbps; /* constant bit rate, one the sample rate allows */
    ahmp3_mode mode;
    int vbr_quality;  /* 0 = constant bit rate. 1 (smallest files) to 10 (best
                         sound) = variable bit rate: each frame takes the bits
                         its sound needs. bitrate_kbps is then the most a
                         frame may use, or 0 for no limit. */
} ahmp3_encoder_config;

typedef struct ahmp3_encoder ahmp3_encoder;

/* NULL when the settings are not a valid MP3 combination. */
ahmp3_encoder *ahmp3_encoder_new(const ahmp3_encoder_config *config);
void ahmp3_encoder_free(ahmp3_encoder *e);

/* Room ahmp3_encode or ahmp3_encoder_flush may need for a call with this
   many samples per channel. */
#define AHMP3_ENCODE_BOUND(samples) ((((size_t)(samples) / 576) + 8) * 1441)

/* Encodes interleaved samples (floats around [-1, 1], or 16-bit) and writes
   every finished frame to out. Returns the bytes written. Frames come out a
   little behind the input; ahmp3_encoder_flush writes the rest. */
size_t ahmp3_encode(ahmp3_encoder *e, const float *pcm, size_t samples, uint8_t *out, size_t out_cap);
size_t ahmp3_encode_s16(ahmp3_encoder *e, const int16_t *pcm, size_t samples, uint8_t *out, size_t out_cap);
/* Pads the last frame with silence and writes everything left. */
size_t ahmp3_encoder_flush(ahmp3_encoder *e, uint8_t *out, size_t out_cap);

/* Samples of silence a decoder puts out before the first input sample, not
   counting AHMP3_DECODER_DELAY. */
int ahmp3_encoder_delay(const ahmp3_encoder *e);

/* Writes an info frame for the start of the file: an Xing header (Info for
   constant bit rate) with a LAME-style tag. It tells players the length, a
   seek table, and the delay and padding to drop so that the sound starts and
   ends exactly where the input did (gapless playback).

   The frame describes what has been encoded so far, and its size never
   changes. So a file starts with one made before any sound is encoded, and
   once ahmp3_encoder_flush is done the final one is written over it. A
   stream that cannot go back keeps the first, which still carries the delay.
   Room of AHMP3_ENCODE_BOUND(0) is always enough; returns 0 when out_cap is
   too small. */
size_t ahmp3_encoder_info_frame(const ahmp3_encoder *e, uint8_t *out, size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif
