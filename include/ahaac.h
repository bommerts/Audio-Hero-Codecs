/* Audio Hero AAC: an AAC decoder in plain C, written from ISO/IEC 14496-3.
   It decodes AAC-LC (low complexity, the kind music files use), mono or
   stereo, from ADTS streams (.aac files) or as the raw packets an MP4/M4A
   file holds (see ahm4a.h). No dependencies beyond the C standard library.

   Not decoded: HE-AAC's spectral band replication and parametric stereo.
   Such a stream still plays, as the plain AAC inside it: at half the sample
   rate, without the top of the treble (and in mono where parametric stereo
   was used). The frame says so in its sbr field. The Main, SSR and LTP
   kinds of AAC, 960-sample frames and more than two channels are refused. */
#ifndef AHAAC_H
#define AHAAC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Samples one frame holds, per channel, and the most channels. */
#define AHAAC_MAX_SAMPLES 1024
#define AHAAC_MAX_CHANNELS 2

typedef struct {
    int sample_rate; /* Hz */
    int channels;    /* 1 or 2 */
    int samples;     /* per channel: 1024, or 0 when the frame gave no sound */
    int frame_bytes; /* the whole ADTS frame, or the packet */
    bool sbr;        /* the stream carries HE-AAC data that is not decoded */
} ahaac_frame;

typedef struct ahaac_decoder ahaac_decoder;

ahaac_decoder *ahaac_decoder_new(void); /* NULL only when out of memory */
void ahaac_decoder_free(ahaac_decoder *d);
/* Forgets the stream after a seek: the overlap of the last frame and sync. */
void ahaac_decoder_reset(ahaac_decoder *d);

/* An ADTS stream: decodes the next frame of data[0..len).

   Returns the bytes used: any junk or tags skipped, plus the frame. Returns 0
   when a whole frame is not there yet; call again with more data, or with
   eof set once the stream has ended (0 then means nothing is left).

   On return *samples is the number of samples per channel written to pcm, as
   interleaved floats around [-1, 1]. It is 0 when the bytes used held no
   sound: skipped junk, or a frame that is damaged or of a kind this decoder
   does not handle. frame (optional) describes the frame decoded. */
size_t ahaac_decode(ahaac_decoder *d, const uint8_t *data, size_t len, bool eof,
                    float pcm[AHAAC_MAX_SAMPLES * AHAAC_MAX_CHANNELS], int *samples, ahaac_frame *frame);

/* Raw AAC packets, as an MP4/M4A file holds them. Set the decoder up once
   with the file's AudioSpecificConfig, then hand it one packet at a time.
   Both return false when the stream is of a kind this decoder does not
   handle, or the packet is damaged (*samples is then 0). */
bool ahaac_decoder_configure(ahaac_decoder *d, const uint8_t *config, size_t len);
bool ahaac_decode_packet(ahaac_decoder *d, const uint8_t *data, size_t len,
                         float pcm[AHAAC_MAX_SAMPLES * AHAAC_MAX_CHANNELS], int *samples, ahaac_frame *frame);

/* NULL while all is well; otherwise why the stream cannot be decoded (for
   example "AAC Main is not supported"). */
const char *ahaac_decoder_error(const ahaac_decoder *d);

#ifdef __cplusplus
}
#endif

#endif
