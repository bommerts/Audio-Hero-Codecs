/* This file follows the reference decoder in RFC 6716 (Appendix A), the
   normative definition of Opus, step by step, and so is under that
   code's licence:

   Copyright 1994-2011 IETF Trust, Xiph.Org, Skype Limited, Octasic,
                       Jean-Marc Valin, Timothy B. Terriberry,
                       CSIRO, Gregory Maxwell, Mark Borgerding,
                       Erik de Castro Lopo. All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   - Neither the name of Internet Society, IETF or IETF Trust, nor the
   names of specific contributors, may be used to endorse or promote
   products derived from this software without specific prior written
   permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
/* Opus (RFC 6716) as Ogg holds it (RFC 7845): the range decoder, packet
   framing, the switching between SILK, hybrid and CELT frames, the streams
   of a multichannel packet and the identification header. */
#include "ahopus_internal.h"

/* ---- the range decoder ---------------------------------------------------------- */

#define CODE_BITS 32
#define SYM_BITS 8
#define CODE_TOP (1u << 31)
#define CODE_BOT (CODE_TOP >> SYM_BITS)
#define CODE_EXTRA ((CODE_BITS - 2) % SYM_BITS + 1)
#define SYM_MAX 255u
#define WINDOW_BITS 32

static int read_byte(ahopus_range *r)
{
    return r->offs < r->storage ? r->buf[r->offs++] : 0;
}

static int read_byte_from_end(ahopus_range *r)
{
    return r->end_offs < r->storage ? r->buf[r->storage - ++r->end_offs] : 0;
}

static void normalize(ahopus_range *r)
{
    while (r->rng <= CODE_BOT) {
        r->nbits_total += SYM_BITS;
        r->rng <<= SYM_BITS;
        int sym = r->rem;
        r->rem = read_byte(r);
        sym = (sym << SYM_BITS | r->rem) >> (SYM_BITS - CODE_EXTRA);
        r->val = ((r->val << SYM_BITS) + (SYM_MAX & ~(uint32_t)sym)) & (CODE_TOP - 1);
    }
}

void ahopus_range_init(ahopus_range *r, const uint8_t *buf, uint32_t storage)
{
    r->buf = buf;
    r->storage = storage;
    r->end_offs = 0;
    r->end_window = 0;
    r->nend_bits = 0;
    r->nbits_total = CODE_BITS + 1 - ((CODE_BITS - CODE_EXTRA) / SYM_BITS) * SYM_BITS;
    r->offs = 0;
    r->rng = 1u << CODE_EXTRA;
    r->rem = read_byte(r);
    r->val = r->rng - 1 - (uint32_t)(r->rem >> (SYM_BITS - CODE_EXTRA));
    r->error = 0;
    normalize(r);
}

unsigned ahopus_decode(ahopus_range *r, unsigned ft)
{
    r->ext = r->rng / ft;
    unsigned s = (unsigned)(r->val / r->ext);
    return ft - (s + 1 < ft ? s + 1 : ft);
}

unsigned ahopus_decode_bin(ahopus_range *r, unsigned bits)
{
    r->ext = r->rng >> bits;
    unsigned s = (unsigned)(r->val / r->ext);
    return (1u << bits) - (s + 1u < 1u << bits ? s + 1u : 1u << bits);
}

void ahopus_update(ahopus_range *r, unsigned fl, unsigned fh, unsigned ft)
{
    uint32_t s = r->ext * (ft - fh);
    r->val -= s;
    r->rng = fl > 0 ? r->ext * (fh - fl) : r->rng - s;
    normalize(r);
}

int ahopus_bit_logp(ahopus_range *r, unsigned logp)
{
    uint32_t s = r->rng >> logp;
    int ret = r->val < s;
    if (!ret) r->val -= s;
    r->rng = ret ? s : r->rng - s;
    normalize(r);
    return ret;
}

int ahopus_icdf(ahopus_range *r, const uint8_t *icdf, unsigned ftb)
{
    uint32_t s = r->rng, d = r->val, rr = s >> ftb, t;
    int ret = -1;
    do {
        t = s;
        s = rr * icdf[++ret];
    } while (d < s);
    r->val = d - s;
    r->rng = t - s;
    normalize(r);
    return ret;
}

uint32_t ahopus_bits(ahopus_range *r, unsigned bits)
{
    uint32_t window = r->end_window;
    int available = r->nend_bits;
    if ((unsigned)available < bits) {
        do {
            window |= (uint32_t)read_byte_from_end(r) << available;
            available += SYM_BITS;
        } while (available <= WINDOW_BITS - SYM_BITS);
    }
    uint32_t ret = window & (bits >= 32 ? 0xFFFFFFFFu : (1u << bits) - 1u);
    window = bits >= 32 ? 0 : window >> bits;
    available -= (int)bits;
    r->end_window = window;
    r->nend_bits = available;
    r->nbits_total += (int)bits;
    return ret;
}

uint32_t ahopus_uint(ahopus_range *r, uint32_t ft)
{
    ft--;
    int ftb = ahopus_ilog(ft);
    if (ftb > 8) {
        ftb -= 8;
        unsigned f = (unsigned)(ft >> ftb) + 1;
        unsigned s = ahopus_decode(r, f);
        ahopus_update(r, s, s + 1, f);
        uint32_t t = (uint32_t)s << ftb | ahopus_bits(r, (unsigned)ftb);
        if (t <= ft) return t;
        r->error = 1;
        return ft;
    }
    ft++;
    unsigned s = ahopus_decode(r, (unsigned)ft);
    ahopus_update(r, s, s + 1, (unsigned)ft);
    return s;
}

int ahopus_tell(const ahopus_range *r)
{
    return r->nbits_total - ahopus_ilog(r->rng);
}

uint32_t ahopus_tell_frac(const ahopus_range *r)
{
    uint32_t nbits = (uint32_t)r->nbits_total << 3;
    int l = ahopus_ilog(r->rng);
    uint32_t rr = r->rng >> (l - 16);
    for (int i = 3; i-- > 0;) {
        rr = rr * rr >> 15;
        int b = (int)(rr >> 16);
        l = l << 1 | b;
        rr >>= b;
    }
    return nbits - (uint32_t)l;
}

/* The Laplace-like distribution of CELT's coarse energy. */
int ahopus_laplace(ahopus_range *r, unsigned fs, int decay)
{
    int val = 0;
    unsigned fl = 0, fm = ahopus_decode_bin(r, 15);
    if (fm >= fs) {
        val++;
        fl = fs;
        fs = ((32768 - 2 * 16 - fs) * (unsigned)(16384 - decay) >> 15) + 1;
        while (fs > 1 && fm >= fl + 2 * fs) {
            fs *= 2;
            fl += fs;
            fs = ((fs - 2) * (unsigned)decay) >> 15;
            fs += 1;
            val++;
        }
        if (fs <= 1) {
            int di = (int)((fm - fl) >> 1);
            val += di;
            fl += 2 * (unsigned)di;
        }
        if (fm < fl + fs) val = -val;
        else fl += fs;
    }
    ahopus_update(r, fl, fl + fs < 32768 ? fl + fs : 32768, 32768);
    return val;
}

/* ---- one Opus stream ------------------------------------------------------------- */

enum { MODE_SILK = 1000, MODE_HYBRID = 1001, MODE_CELT = 1002 };
enum { BW_NB, BW_MB, BW_WB, BW_SWB, BW_FB };

typedef struct {
    int channels;        /* 1 or 2, as the stream is set up */
    int stream_channels; /* as the current packet says */
    int mode, prev_mode, bandwidth, frame_size;
    bool prev_redundancy;
    int silk_rate, silk_channels; /* the SILK settings of the last packet, for loss cover */
    uint32_t range_final;         /* the range decoder's state after the last frame */
    ahopus_silk silk;
    ahopus_celt celt;
} stream_decoder;

#define F20 960
#define F10 480
#define F5 240
#define F2_5 120

static void stream_reset(stream_decoder *st)
{
    ahopus_silk_init(&st->silk);
    ahopus_celt_reset(&st->celt);
    st->stream_channels = st->channels;
    st->mode = st->prev_mode = 0;
    st->bandwidth = 0;
    st->frame_size = 960 / 8;
    st->prev_redundancy = false;
    st->silk_rate = 16000;
    st->silk_channels = st->channels;
}

static void stream_init(stream_decoder *st, const ahopus_celt_setup *setup, int channels)
{
    st->channels = channels;
    ahopus_celt_init(&st->celt, setup, channels);
    stream_reset(st);
}

static void smooth_fade(const float *in1, const float *in2, float *out, int overlap, int channels, const float *window)
{
    for (int c = 0; c < channels; c++)
        for (int i = 0; i < overlap; i++) {
            float w = window[i] * window[i];
            out[i * channels + c] = w * in2[i * channels + c] + (1.f - w) * in1[i * channels + c];
        }
}

/* One Opus frame (or, with data NULL, a lost one) into pcm. Returns the
   samples per channel, or -1 when the frame is damaged. */
static int decode_frame(stream_decoder *st, const uint8_t *data, int len, float *pcm, int frame_size)
{
    const int cc = st->channels;
    float pcm_transition[F5 * 2], redundant_audio[F5 * 2];
    int16_t pcm_silk[AHOPUS_MAX_SAMPLES * 2];
    ahopus_range dec = {0};
    uint32_t redundant_rng = 0;
    int audiosize, mode, transition = 0, redundancy = 0, redundancy_bytes = 0, celt_to_silk = 0;
    const bool ended_in_celt = st->prev_mode != MODE_SILK || st->prev_redundancy;
    bool celt_ok = true;

    if (len <= 1) {
        data = NULL;
        frame_size = frame_size < st->frame_size ? frame_size : st->frame_size;
    }
    if (data) {
        audiosize = st->frame_size;
        mode = st->mode;
        ahopus_range_init(&dec, data, (uint32_t)len);
    } else {
        audiosize = frame_size;
        if (st->prev_mode == 0) {
            memset(pcm, 0, (size_t)(audiosize * cc) * sizeof(float));
            return audiosize;
        }
        /* After a SILK frame that led into CELT, the cover is CELT's. */
        mode = st->prev_redundancy ? MODE_CELT : st->prev_mode;
        /* Cover in pieces of a size a frame can have, as libopus does. */
        if (audiosize > F20) {
            int n = 0;
            while (n < frame_size) {
                int ret = decode_frame(st, NULL, 0, pcm + n * cc, frame_size - n < F20 ? frame_size - n : F20);
                if (ret <= 0) return -1;
                n += ret;
            }
            return frame_size;
        }
        if (audiosize < F20) {
            if (audiosize > F10) audiosize = F10;
            else if (mode != MODE_SILK && audiosize > F5 && audiosize < F10) audiosize = F5;
        }
        frame_size = audiosize;
    }
    if (data && st->prev_mode > 0 &&
        ((mode == MODE_CELT && st->prev_mode != MODE_CELT && !st->prev_redundancy) || (mode != MODE_CELT && st->prev_mode == MODE_CELT))) {
        transition = 1;
        if (mode == MODE_CELT) decode_frame(st, NULL, 0, pcm_transition, F5 < audiosize ? F5 : audiosize);
    }
    if (audiosize > frame_size) return -1;
    frame_size = audiosize;

    if (mode != MODE_CELT) {
        if (st->prev_mode == MODE_CELT) ahopus_silk_init(&st->silk);
        int payload_ms = 1000 * audiosize / 48000;
        if (payload_ms < 10) payload_ms = 10;
        if (data) {
            st->silk_channels = st->stream_channels;
            st->silk_rate = mode == MODE_SILK ? (st->bandwidth == BW_NB ? 8000 : st->bandwidth == BW_MB ? 12000 : 16000) : 16000;
        }
        int done = 0;
        do {
            int n = ahopus_silk_decode(&st->silk, data ? &dec : NULL, cc, st->silk_channels, st->silk_rate, payload_ms, data == NULL,
                                       done == 0, pcm_silk + done * cc);
            if (n <= 0) {
                if (data) return -1;
                n = frame_size;
                memset(pcm_silk + done * cc, 0, (size_t)(n * cc) * sizeof(int16_t));
            }
            done += n;
        } while (done < frame_size);
    }

    int start_band = 0;
    if (mode != MODE_CELT && data && ahopus_tell(&dec) + 17 + 20 * (mode == MODE_HYBRID) <= 8 * len) {
        redundancy = mode == MODE_HYBRID ? ahopus_bit_logp(&dec, 12) : 1;
        if (redundancy) {
            celt_to_silk = ahopus_bit_logp(&dec, 1);
            redundancy_bytes = mode == MODE_HYBRID ? (int)ahopus_uint(&dec, 256) + 2 : len - ((ahopus_tell(&dec) + 7) >> 3);
            len -= redundancy_bytes;
            if (len * 8 < ahopus_tell(&dec)) {
                len = 0;
                redundancy_bytes = 0;
                redundancy = 0;
            }
            dec.storage -= (uint32_t)redundancy_bytes;
        }
    }
    if (mode != MODE_CELT) start_band = 17;
    static const int end_bands[5] = {13, 17, 17, 19, 21};
    st->celt.end = end_bands[st->bandwidth];
    st->celt.stream_channels = st->stream_channels;
    if (redundancy) transition = 0;
    if (transition && mode != MODE_CELT) decode_frame(st, NULL, 0, pcm_transition, F5 < audiosize ? F5 : audiosize);
    if (redundancy && celt_to_silk) {
        st->celt.start = 0;
        ahopus_celt_decode(&st->celt, data + len, redundancy_bytes, redundant_audio, F5, NULL);
        redundant_rng = st->celt.rng;
    }
    st->celt.start = start_band;

    if (mode != MODE_SILK) {
        int celt_frame_size = F20 < frame_size ? F20 : frame_size;
        if (mode != st->prev_mode && st->prev_mode > 0 && !st->prev_redundancy) ahopus_celt_reset(&st->celt);
        celt_ok = ahopus_celt_decode(&st->celt, data, len, pcm, celt_frame_size, data ? &dec : NULL);
    } else {
        static const uint8_t silence[2] = {0xFF, 0xFF};
        memset(pcm, 0, (size_t)(frame_size * cc) * sizeof(float));
        if (st->prev_mode == MODE_HYBRID && !(redundancy && celt_to_silk && st->prev_redundancy)) {
            st->celt.start = 0;
            ahopus_celt_decode(&st->celt, silence, 2, pcm, F2_5, NULL);
        }
    }
    if (mode != MODE_CELT)
        for (int i = 0; i < frame_size * cc; i++) pcm[i] = pcm[i] + (1.f / 32768.f) * (float)pcm_silk[i];

    const float *window = st->celt.setup->window;
    if (redundancy && !celt_to_silk) {
        ahopus_celt_reset(&st->celt);
        st->celt.start = 0;
        ahopus_celt_decode(&st->celt, data + len, redundancy_bytes, redundant_audio, F5, NULL);
        redundant_rng = st->celt.rng;
        smooth_fade(pcm + cc * (frame_size - F2_5), redundant_audio + cc * F2_5, pcm + cc * (frame_size - F2_5), F2_5, cc, window);
    }
    /* The CELT to SILK redundant frame leads in only where the frame before
       was not plain SILK, as libopus does (RFC 6716 leaves redundancy
       outside a switch of mode to the decoder). */
    if (redundancy && celt_to_silk && ended_in_celt) {
        memcpy(pcm, redundant_audio, (size_t)(F2_5 * cc) * sizeof(float));
        smooth_fade(redundant_audio + cc * F2_5, pcm + cc * F2_5, pcm + cc * F2_5, F2_5, cc, window);
    }
    if (transition) {
        if (audiosize >= F5) {
            memcpy(pcm, pcm_transition, (size_t)(F2_5 * cc) * sizeof(float));
            smooth_fade(pcm_transition + cc * F2_5, pcm + cc * F2_5, pcm + cc * F2_5, F2_5, cc, window);
        } else {
            smooth_fade(pcm_transition, pcm, pcm, F2_5, cc, window);
        }
    }
    st->range_final = len <= 1 ? 0 : dec.rng ^ redundant_rng;
    st->prev_mode = mode;
    st->prev_redundancy = redundancy && !celt_to_silk;
    return celt_ok ? audiosize : -1;
}

/* The frames of one stream's packet. self_delimited: the last frame's size
   is coded too (every stream but the last of a multistream packet). */
static int parse_size(const uint8_t *data, int len, int *size)
{
    if (len < 1) return -1;
    if (data[0] < 252) {
        *size = data[0];
        return 1;
    }
    if (len < 2) return -1;
    *size = 4 * data[1] + data[0];
    return 2;
}

static int samples_per_frame(uint8_t toc)
{
    if (toc & 0x80) return (48000 << ((toc >> 3) & 3)) / 400;
    if ((toc & 0x60) == 0x60) return (toc & 0x08) ? 960 : 480;
    int s = (toc >> 3) & 3;
    return s == 3 ? 2880 : (48000 << s) / 100;
}

/* Returns the number of frames, or -1 for a malformed packet. */
static int parse_packet(const uint8_t *data, int len, bool self_delimited, const uint8_t *frames[48], int size[48], int *packet_bytes)
{
    const uint8_t *start = data;
    if (len < 1) return -1;
    int framesize = samples_per_frame(data[0]);
    uint8_t toc = *data++;
    len--;
    int last_size = len, count, bytes, pad = 0;
    bool cbr = false;
    switch (toc & 3) {
    case 0:
        count = 1;
        break;
    case 1:
        count = 2;
        cbr = true;
        if (!self_delimited) {
            if (len & 1) return -1;
            last_size = len / 2;
            size[0] = last_size;
        }
        break;
    case 2:
        count = 2;
        bytes = parse_size(data, len, &size[0]);
        len -= bytes;
        if (bytes < 0 || size[0] > len) return -1;
        data += bytes;
        last_size = len - size[0];
        break;
    default: {
        if (len < 1) return -1;
        uint8_t ch = *data++;
        count = ch & 0x3F;
        if (count <= 0 || framesize * count > 5760) return -1;
        len--;
        if (ch & 0x40) {
            int p;
            do {
                if (len <= 0) return -1;
                p = *data++;
                len--;
                len -= p == 255 ? 254 : p;
                pad += p == 255 ? 254 : p;
            } while (p == 255);
        }
        if (len < 0) return -1;
        cbr = !(ch & 0x80);
        if (!cbr) {
            last_size = len;
            for (int i = 0; i < count - 1; i++) {
                bytes = parse_size(data, len, &size[i]);
                len -= bytes;
                if (bytes < 0 || size[i] > len) return -1;
                data += bytes;
                last_size -= bytes + size[i];
            }
            if (last_size < 0) return -1;
        } else if (!self_delimited) {
            last_size = len / count;
            if (last_size * count != len) return -1;
            for (int i = 0; i < count - 1; i++) size[i] = last_size;
        }
        break;
    }
    }
    if (self_delimited) {
        bytes = parse_size(data, len, &size[count - 1]);
        len -= bytes;
        if (bytes < 0 || size[count - 1] > len) return -1;
        data += bytes;
        if (cbr) {
            if (size[count - 1] * count > len) return -1;
            for (int i = 0; i < count - 1; i++) size[i] = size[count - 1];
        } else if (bytes + size[count - 1] > last_size) {
            return -1;
        }
    } else {
        if (last_size > 1275) return -1;
        size[count - 1] = last_size;
    }
    for (int i = 0; i < count; i++) {
        frames[i] = data;
        data += size[i];
    }
    /* A stream's padding follows its frames, before the next stream. */
    *packet_bytes = (int)(data - start) + pad;
    return count;
}

/* One stream's packet into pcm (interleaved by the stream's channels).
   Returns the samples per channel or -1. */
static int decode_stream(stream_decoder *st, const uint8_t *data, int len, float *pcm, int max_samples, bool self_delimited,
                         int *packet_bytes)
{
    if (!data || len == 0) {
        /* A lost packet: cover it frame by frame. */
        *packet_bytes = 0;
        int n = 0;
        while (n < max_samples) {
            int ret = decode_frame(st, NULL, 0, pcm + n * st->channels, max_samples - n);
            if (ret <= 0) return -1;
            n += ret;
        }
        return n;
    }
    uint8_t toc = data[0];
    st->mode = (toc & 0x80) ? MODE_CELT : (toc & 0x60) == 0x60 ? MODE_HYBRID : MODE_SILK;
    if (toc & 0x80) {
        int bw = 1 + ((toc >> 5) & 3);
        st->bandwidth = bw == 1 ? BW_NB : bw;
    } else if ((toc & 0x60) == 0x60) {
        st->bandwidth = (toc & 0x10) ? BW_FB : BW_SWB;
    } else {
        st->bandwidth = (toc >> 5) & 3;
    }
    st->frame_size = samples_per_frame(toc);
    st->stream_channels = (toc & 0x4) ? 2 : 1;
    const uint8_t *frames[48];
    int size[48];
    int count = parse_packet(data, len, self_delimited, frames, size, packet_bytes);
    if (count < 0) return -1;
    if (count * st->frame_size > max_samples) return -1;
    int n = 0;
    for (int i = 0; i < count; i++) {
        int ret = decode_frame(st, frames[i], size[i], pcm + n * st->channels, max_samples - n);
        if (ret < 0) return -1;
        n += ret;
    }
    return n;
}

/* ---- the multistream decoder and Ogg header ---------------------------------------- */

struct ahopus_decoder {
    ahopus_celt_setup setup;
    stream_decoder *streams;
    int nb_streams, nb_coupled, channels;
    uint8_t mapping[AHOPUS_MAX_CHANNELS]; /* by output (WAV order) channel */
    float gain;
    int last_samples;
    bool ready;
    const char *error;
    float buf[AHOPUS_MAX_SAMPLES * 2];
};

ahopus_decoder *ahopus_decoder_new(void)
{
    ahopus_decoder *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    ahopus_celt_setup_init(&d->setup);
    d->gain = 1.f;
    return d;
}

void ahopus_decoder_free(ahopus_decoder *d)
{
    if (!d) return;
    free(d->streams);
    free(d);
}

void ahopus_decoder_reset(ahopus_decoder *d)
{
    for (int s = 0; s < d->nb_streams; s++) stream_reset(&d->streams[s]);
    d->last_samples = 0;
}

uint32_t ahopus_final_range(const ahopus_decoder *d)
{
    uint32_t r = 0;
    for (int s = 0; s < d->nb_streams; s++) r ^= d->streams[s].range_final;
    return r;
}

const char *ahopus_decoder_error(const ahopus_decoder *d)
{
    return d->error;
}

/* Vorbis channel order to WAV order, and the speakers, by channel count. */
static const uint8_t wav_place[8][8] = {
    {0}, {0, 1}, {0, 2, 1}, {0, 1, 2, 3}, {0, 2, 1, 3, 4}, {0, 2, 1, 4, 5, 3}, {0, 2, 1, 5, 6, 4, 3}, {0, 2, 1, 6, 7, 4, 5, 3},
};
static const uint32_t wav_mask[8] = {0x4, 0x3, 0x7, 0x33, 0x37, 0x3F, 0x70F, 0x63F};

bool ahopus_decoder_header(ahopus_decoder *d, const uint8_t *head, size_t len, ahopus_format *format)
{
    d->ready = false;
    if (len < 19 || memcmp(head, "OpusHead", 8) != 0) {
        d->error = "not an Opus stream";
        return false;
    }
    if (head[8] >= 16) {
        d->error = "Opus header of an unknown version";
        return false;
    }
    int channels = head[9], family = head[18], streams, coupled;
    uint8_t map[255];
    if (channels == 0) {
        d->error = "damaged Opus header";
        return false;
    }
    if (family == 0) {
        if (channels > 2) {
            d->error = "damaged Opus header";
            return false;
        }
        streams = 1;
        coupled = channels - 1;
        for (int c = 0; c < channels; c++) map[c] = (uint8_t)c;
    } else if (family == 1) {
        if (channels > 8 || len < 21u + (size_t)channels) {
            d->error = channels > 8 ? "Opus with more than 8 channels is not supported" : "damaged Opus header";
            return false;
        }
        streams = head[19];
        coupled = head[20];
        if (streams < 1 || coupled > streams || streams + coupled > 255) {
            d->error = "damaged Opus header";
            return false;
        }
        for (int c = 0; c < channels; c++) {
            map[c] = head[21 + c];
            if (map[c] != 255 && map[c] >= streams + coupled) {
                d->error = "damaged Opus header";
                return false;
            }
        }
    } else {
        d->error = "Opus channel mapping family not supported";
        return false;
    }
    stream_decoder *st = calloc((size_t)streams, sizeof(*st));
    if (!st) {
        d->error = "out of memory";
        return false;
    }
    free(d->streams);
    d->streams = st;
    d->nb_streams = streams;
    d->nb_coupled = coupled;
    d->channels = channels;
    for (int s = 0; s < streams; s++) stream_init(&st[s], &d->setup, s < coupled ? 2 : 1);
    for (int c = 0; c < channels; c++) d->mapping[wav_place[channels - 1][c]] = map[c];
    int16_t gain = (int16_t)(head[16] | head[17] << 8);
    d->gain = gain ? (float)exp(0.6931471805599453094 * (6.48814081e-4f * (float)gain)) : 1.f;
    d->last_samples = 0;
    d->error = NULL;
    d->ready = true;
    if (format) {
        memset(format, 0, sizeof(*format));
        format->sample_rate = 48000;
        format->channels = channels;
        format->channel_mask = wav_mask[channels - 1];
        format->pre_skip = head[10] | head[11] << 8;
        format->input_rate = (int)((uint32_t)head[12] | (uint32_t)head[13] << 8 | (uint32_t)head[14] << 16 | (uint32_t)head[15] << 24);
        format->gain = gain;
        format->mapping_family = family;
        format->streams = streams;
        format->coupled = coupled;
    }
    return true;
}

/* Every stream of one packet (data NULL: a lost one, `frame_size` long)
   into pcm. Returns the samples per channel, or -1 when the packet is
   damaged. */
static int decode_streams(ahopus_decoder *d, const uint8_t *data, int len, float *pcm, int frame_size)
{
    const int nc = d->channels;
    bool lost = data == NULL;
    if (!lost) {
        /* Every stream's framing must hold, all of the same length, before
           any is decoded (libopus checks the same). */
        if (len < 2 * d->nb_streams - 1) return -1;
        const uint8_t *p = data, *frames[48];
        int left = len, size[48], duration = 0;
        for (int s = 0; s < d->nb_streams; s++) {
            int used = 0;
            if (left <= 0) return -1;
            int count = parse_packet(p, left, s != d->nb_streams - 1, frames, size, &used);
            if (count < 0) return -1;
            int n = count * samples_per_frame(p[0]);
            if (s > 0 && n != duration) return -1;
            duration = n;
            p += used;
            left -= used;
        }
    }
    int got = 0;
    for (int s = 0; s < d->nb_streams; s++) {
        stream_decoder *st = &d->streams[s];
        int used = 0;
        if (!lost && len <= 0) return -1;
        int n = decode_stream(st, data, len, d->buf, frame_size, s != d->nb_streams - 1, &used);
        if (n <= 0 || (s > 0 && n != got)) return -1;
        if (!lost) {
            data += used;
            len -= used;
        }
        got = frame_size = n;
        /* To the output channels that take this stream's channels. */
        for (int c = 0; c < nc; c++) {
            int m = d->mapping[c], src;
            if (m == 255) continue;
            if (s < d->nb_coupled) {
                if (m >> 1 != s) continue;
                src = m & 1;
            } else {
                if (m != d->nb_coupled * 2 + (s - d->nb_coupled)) continue;
                src = 0;
            }
            for (int i = 0; i < n; i++) pcm[i * nc + c] = d->buf[i * st->channels + src] * d->gain;
        }
    }
    for (int c = 0; c < nc; c++)
        if (d->mapping[c] == 255)
            for (int i = 0; i < got; i++) pcm[i * nc + c] = 0;
    return got;
}

/* The length a packet states in its first byte (and its frame count), or
   0 when that says nothing that can be. */
static int packet_duration(const uint8_t *data, size_t len)
{
    int count = (data[0] & 3) == 0 ? 1 : (data[0] & 3) < 3 ? 2 : len >= 2 ? data[1] & 0x3F : 0;
    int samples = count * samples_per_frame(data[0]);
    return samples <= AHOPUS_MAX_SAMPLES ? samples : 0;
}

bool ahopus_decode_packet(ahopus_decoder *d, const uint8_t *data, size_t len, float *pcm, int *samples)
{
    *samples = 0;
    if (!d->ready) return false;
    bool lost = data == NULL || len == 0;
    int n = lost ? -1 : decode_streams(d, data, (int)(len > 0x7FFFFFFF ? 0x7FFFFFFF : len), pcm, AHOPUS_MAX_SAMPLES);
    if (n > 0) {
        d->last_samples = n;
        *samples = n;
        return true;
    }
    /* Lost, or damaged (libopus refuses such a packet): covered for the
       length it states, or that of the packet before. */
    int cover = lost ? (d->last_samples ? d->last_samples : 960) : packet_duration(data, len);
    if (cover > 0) {
        n = decode_streams(d, NULL, 0, pcm, cover);
        if (n > 0) *samples = n;
    }
    return lost && n > 0;
}
