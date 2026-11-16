# Audio Hero MP3

An MP3 (MPEG-1/2/2.5 Layer III) decoder and encoder in plain C, written from
the ISO standards with no dependencies beyond the C standard library. It was
built for the Audio Hero Companion, which streams music to network audio
devices, and is published here for anyone to use.

## What it does

**Decoder**
- MPEG-1, MPEG-2 and MPEG-2.5 Layer III: every sample rate from 8 to 48 kHz,
  mono, stereo, joint stereo (mid/side and intensity), dual channel
- Long, short and mixed blocks; the bit reservoir; CBR and VBR
- Skips ID3v2 tags and junk, resyncs after damage, can be fed any number of
  bytes at a time
- Reads Xing, Info, VBRI and LAME headers, including the encoder delay and
  padding for gapless playback
- Output as floats, with a helper for 16-bit

**Encoder**
- Constant bit rate at every MPEG-1/2/2.5 sample rate and bit rate
- Mono, stereo, or joint stereo choosing left/right or mid/side per frame
- Polyphase filter bank and MDCT with block switching for transients
- A masking model (spreading, tonality, threshold in quiet, pre-echo
  control) driving the noise-shaping and bit-rate loops
- The bit reservoir, and a lowpass that follows the bit rate

## How it is checked

`tests/tests.c` runs about 1,900 checks. With `ffmpeg` on the PATH it also
uses ffmpeg as an independent reference:

- Every Huffman code of every table is written into a frame and decoded by
  both decoders; the output must match.
- Thousands of randomly built frames use every feature of the format (block
  types, mixed blocks, mid/side and intensity stereo, scfsi, preflag,
  subblock gains, both scale factor scales, every table, all sample rates);
  both decoders must agree to floating point precision.
- Files from LAME, shine and the Windows Media Foundation encoder must decode
  exactly as ffmpeg decodes them.
- This encoder's output must decode the same in ffmpeg as here, with no
  errors, including for input full of NaN, infinity and clipping.
- Gapless trimming must match ffmpeg's to the sample.
- Damaged and random input must never crash the decoder. The whole suite also
  runs under AddressSanitizer.

One deliberate difference: when a stream puts a short block straight after a
long one (encoders are meant to put a start block between them), or a short
block after a mixed one, ffmpeg drops part of the overlap. This decoder
overlaps exactly, as the standard describes. Common encoders never write
either sequence.

## Quality and speed

On mixed test material at 128 kbps, the encoder's signal-to-noise ratio is
within a few tenths of a dB of LAME's at the same bit rate, and above it at
some others. Signal-to-noise is not how music sounds, so judge by listening.
On one core of a desktop PC it encodes about 50 times faster than real time
and decodes about 500 times faster.

## Using it

```c
#include "ahmp3.h"

/* Decoding */
ahmp3_decoder *d = ahmp3_decoder_new();
float pcm[AHMP3_MAX_SAMPLES * 2];
int samples;
ahmp3_frame frame;
size_t used = ahmp3_decode(d, data, len, eof, pcm, &samples, &frame);
/* used = bytes consumed; samples per channel in pcm (interleaved);
   0 used means more data is needed, or the end. */
ahmp3_decoder_free(d);

/* Encoding */
ahmp3_encoder_config cfg = {44100, 2, 128, AHMP3_JOINT_STEREO};
ahmp3_encoder *e = ahmp3_encoder_new(&cfg);
size_t n = ahmp3_encode(e, interleaved, frames, out, AHMP3_ENCODE_BOUND(frames));
n = ahmp3_encoder_flush(e, out, AHMP3_ENCODE_BOUND(0));
ahmp3_encoder_free(e);
```

The whole interface is in [include/ahmp3.h](include/ahmp3.h).

There is also a small command line tool:

```
ahmp3 decode in.mp3 out.wav
ahmp3 encode in.wav out.mp3 --bitrate 192
```

## Building

CMake and a C17 compiler. Built and tested with Visual Studio on Windows;
the code is portable C but has not yet been built on Linux or macOS.

```powershell
.\test.ps1          # build and run the tests
.\test.ps1 -Asan    # the same under AddressSanitizer
```

## Not done yet

- Variable bit rate encoding, and writing a Xing/LAME header from the encoder
- Layers I and II, free-format streams, CRC checking

## License

MIT, see [LICENSE](LICENSE). The MP3 patents have expired.
