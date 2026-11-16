# Audio Hero MP3

Audio decoders and an MP3 encoder in plain C, written from the published
standards with no dependencies beyond the C standard library. They were built
for the Audio Hero Companion, which streams music to network audio devices,
and are published here for anyone to use.

| Format | Decode | Encode | Header |
| --- | --- | --- | --- |
| MP3 (MPEG-1/2/2.5 Layer III) | yes | yes | `ahmp3.h` |
| MPEG Layers I and II (.mp1, .mp2) | yes | | `ahmp3.h` |
| FLAC | yes | | `ahflac.h` |
| AAC-LC, as ADTS (.aac) or in MP4 (.m4a) | yes | | `ahaac.h`, `ahm4a.h` |
| WAV | yes | | `ahwav.h` |

The repository keeps the name it started with, when it held MP3 alone.

## What it does

**MP3 and MPEG audio decoder**
- MPEG-1, MPEG-2 and MPEG-2.5, Layers I, II and III: every sample rate from
  8 to 48 kHz, mono, stereo, joint stereo (mid/side and intensity), dual
  channel
- Layer III: long, short and mixed blocks; the bit reservoir; CBR and VBR
- Free-format streams (frames that do not state their bit rate)
- CRC checking: a frame whose CRC does not match is played as silence and
  reported
- Skips ID3v2 tags and junk, resyncs after damage, can be fed any number of
  bytes at a time
- Reads Xing, Info, VBRI and LAME headers, including the encoder delay and
  padding for gapless playback
- Output as floats, with a helper for 16-bit

**MP3 encoder**
- Constant bit rate at every MPEG-1/2/2.5 sample rate and bit rate, or
  variable bit rate at ten quality levels
- Mono, stereo, or joint stereo choosing left/right or mid/side per frame
- Polyphase filter bank and MDCT with block switching for transients
- A masking model (spreading, tonality, threshold in quiet, pre-echo
  control) driving the noise-shaping and bit-rate loops
- The bit reservoir, and a lowpass that follows the bit rate
- Writes an Xing/Info header with a LAME-style tag: length, seek table, and
  the delay and padding that make its files play gapless

**FLAC decoder**
- Every bit depth from 4 to 32, 1 to 8 channels, every sample rate
- All of the format: fixed and linear predictors, both residual codings,
  escaped partitions, wasted bits, the three stereo modes, variable block
  sizes
- Checks both CRCs of every frame and drops damaged frames; resyncs; can
  start in the middle of a file after a seek
- Output as whole numbers, exactly as encoded, with a helper for floats

**AAC decoder**
- AAC-LC (low complexity, the kind music files use), mono and stereo, all 13
  sample rates from 7.35 to 96 kHz
- ADTS streams, and MP4/M4A files through a small reader that finds the
  track, its settings, each packet, and the gapless trim (edit list or
  iTunes tag)
- All of AAC-LC: every code book, pulses, mid/side and intensity stereo,
  perceptual noise substitution, temporal noise shaping, long and short
  windows of both shapes
- Not decoded: HE-AAC's spectral band replication and parametric stereo.
  Such a file still plays, as the plain AAC inside it: at half the sample
  rate, without the top of the treble. The decoder says when that happens.
- Refused with a reason: AAC Main, SSR and LTP, 960-sample frames, more than
  two channels, Apple Lossless (ALAC), fragmented MP4

**WAV reader**
- RIFF/WAVE and RF64, plain and extensible headers
- Whole-number samples of 8, 16, 24 and 32 bits; floats of 32 and 64 bits;
  up to 8 channels
- Files whose length was never filled in (streams) are read to their end
- Not read: compressed kinds of WAV (ADPCM, A-law, mu-law)

## How it is checked

`tests/` runs about 2,700 checks. With `ffmpeg` on the PATH it also uses
ffmpeg as an independent reference:

- **Every code of every table.** Each Huffman code of MP3 and of AAC is
  written into a frame and decoded by both decoders; the output must match.
- **Random frames using every feature.** Small frame writers in the tests
  build thousands of MP3, Layer I, Layer II, AAC and FLAC frames at random
  that use every part of each format, well beyond what encoders choose to
  write. This decoder and ffmpeg must agree: to floating point precision for
  the lossy formats, bit for bit for FLAC. For FLAC the test also knows the
  samples it encoded, so the decoder is checked without ffmpeg too.
- **Files from other encoders.** LAME, shine, twolame, ffmpeg's own MP2, AAC
  and FLAC encoders and the Windows Media Foundation MP3 and AAC encoders:
  every file must decode exactly as ffmpeg decodes it. WAV files of every
  sample format must read bit for bit as ffmpeg reads them.
- **This encoder's output** must decode the same in ffmpeg as here, with no
  errors, for constant and variable bit rate, including input full of NaN,
  infinity and clipping.
- **Gapless.** Trimming by the header must match ffmpeg's to the sample, for
  LAME's files, for this encoder's files (which must come back exactly as
  long as the input, down to a file of one sample), and for M4A files.
- **Damage.** Damaged, cut-off and random input must never crash a decoder
  or make it produce more than the file could hold; frames with a failed CRC
  must be caught. The whole suite also runs under AddressSanitizer.

Formats ffmpeg cannot help with are checked against themselves: free-format
streams must decode to the same sound as the same frames with a stated bit
rate.

### Where this differs from ffmpeg, on purpose

- **MP3 block order.** When a stream puts a short block straight after a
  long one (encoders are meant to put a start block between them), or a
  short block after a mixed one, ffmpeg drops part of the overlap. This
  decoder overlaps exactly, as the standard describes. Common encoders never
  write either sequence.
- **MP3 CRC.** ffmpeg ignores a frame's CRC unless asked to check it. This
  decoder checks it and plays a failed frame as silence, since wrong side
  information can turn into a loud burst.
- **Free format.** ffmpeg does not decode free-format streams. This decoder
  does.
- **AAC intensity stereo with "all bands mid/side".** The standard turns the
  right channel over only when the mid/side flags are sent band by band;
  ffmpeg also does it when the stream says "all bands". This decoder follows
  the standard.
- **AAC noise shared by both channels.** The standard gives both channels of
  a pair the same noise in a noise band that is flagged mid/side; ffmpeg
  always makes separate noise. This decoder follows the standard.
- **AAC after a damaged frame.** ffmpeg carries on with what the frame
  before the bad one left behind; this decoder starts the next frame from
  silence.

Encoders in common use write none of these, so files play the same in both.

One thing is the same on purpose: the noise that fills AAC's noise bands
comes from the same simple generator and starting value that ffmpeg uses, so
that the two can be compared sample for sample. The standard leaves the
generator open.

## Quality and speed

On mixed test material at 128 kbps, the encoder's signal-to-noise ratio is
within a few tenths of a dB of LAME's at the same bit rate, and above it at
some others. Variable bit rate quality 10 lands near LAME's `-V 0` in bit
rate and signal-to-noise ratio on the same material, and quality 1 at under
a third of that bit rate. Signal-to-noise is not how music sounds, so judge
by listening.

On one core of a desktop PC:

| | Times faster than real time |
| --- | --- |
| MP3 decode | 500 |
| Layer II decode | 800 |
| AAC decode | 650 |
| FLAC decode (16-bit, 44.1 kHz) | 1000 |
| MP3 encode | 50 to 100 |

## Using it

Every decoder is fed the same way: hand it whatever bytes you have, it tells
you how many it used and gives you the samples of at most one frame, and you
call again. Nothing here opens files or allocates more than its own state.

```c
#include "ahmp3.h"

/* Decoding MP3 (ahaac_decode works the same way for .aac) */
ahmp3_decoder *d = ahmp3_decoder_new();
float pcm[AHMP3_MAX_SAMPLES * 2];
int samples;
ahmp3_frame frame;
size_t used = ahmp3_decode(d, data, len, eof, pcm, &samples, &frame);
/* used = bytes consumed; samples per channel in pcm (interleaved);
   0 used means more data is needed, or the end. */
ahmp3_decoder_free(d);

/* Encoding MP3: constant 128 kbps here; set the last field to 1..10 for
   variable bit rate. */
ahmp3_encoder_config cfg = {44100, 2, 128, AHMP3_JOINT_STEREO, 0};
ahmp3_encoder *e = ahmp3_encoder_new(&cfg);
size_t n = ahmp3_encode(e, interleaved, frames, out, AHMP3_ENCODE_BOUND(frames));
n = ahmp3_encoder_flush(e, out, AHMP3_ENCODE_BOUND(0));
n = ahmp3_encoder_info_frame(e, out, AHMP3_ENCODE_BOUND(0)); /* goes first in the file */
ahmp3_encoder_free(e);
```

```c
#include "ahflac.h"

/* FLAC (ahwav_decode works the same way, giving floats) */
ahflac_decoder *d = ahflac_decoder_new();
int32_t pcm[8192];
int samples;
ahflac_format format;
size_t used = ahflac_decode(d, data, len, eof, pcm, 8192, &samples, &format);
/* Call again until a call uses no bytes and gives no samples. */
ahflac_decoder_free(d);
```

```c
#include "ahaac.h"
#include "ahm4a.h"

/* M4A: the reader finds the packets, the AAC decoder decodes them. */
ahm4a *m = ahm4a_open(my_read_function, my_file, file_size, &error);
const ahm4a_info *info = ahm4a_get_info(m);
ahaac_decoder *d = ahaac_decoder_new();
ahaac_decoder_configure(d, info->config, info->config_bytes);
for (uint64_t i = 0; i < info->packets; i++) {
    size_t bytes = ahm4a_read_packet(m, i, packet, sizeof(packet));
    ahaac_decode_packet(d, packet, bytes, pcm, &samples, &frame);
}
/* info->delay and info->samples say what to keep for gapless playback. */
```

Each header documents its whole interface: [ahmp3.h](include/ahmp3.h),
[ahflac.h](include/ahflac.h), [ahaac.h](include/ahaac.h),
[ahm4a.h](include/ahm4a.h), [ahwav.h](include/ahwav.h).

There is also a small command line tool. It tells the kind of file from its
contents:

```
ahmp3 decode in.flac out.wav
ahmp3 encode in.m4a out.mp3 --bitrate 192
ahmp3 encode in.wav out.mp3 --vbr 8
```

## Building

CMake and a C17 compiler. Built and tested with Visual Studio on Windows;
the code is portable C but has not yet been built on Linux or macOS.

```powershell
.\test.ps1          # build and run the tests
.\test.ps1 -Asan    # the same under AddressSanitizer
```

The library target is `ahcodecs`. Each decoder stands alone, so you can also
take just the files you need:

| For | Files in `src/` |
| --- | --- |
| MP3 and Layers I/II decoding | `ahmp3_decode.c`, `ahmp3_layer12.c`, `ahmp3_tables.c`, `ahmp3_huff.c` |
| MP3 encoding | `ahmp3_encode.c`, `ahmp3_bitstream.c`, `ahmp3_tables.c`, `ahmp3_huff.c` |
| FLAC | `ahflac.c` |
| AAC | `ahaac_decode.c`, `ahaac_tables.c` |
| M4A | `ahm4a.c` |
| WAV | `ahwav.c` |

## Not done

- No sample rate conversion: a decoder gives the file's own rate, and the
  MP3 encoder takes only the rates MP3 has (8 to 48 kHz).
- HE-AAC (SBR and parametric stereo), AAC with more than two channels, and
  Apple Lossless.
- FLAC inside Ogg; the MD5 check of a whole FLAC file.
- The CRC that an ADTS frame may carry is not checked.

## License

MIT, see [LICENSE](LICENSE).

On patents: the MP3 patents have expired, and FLAC and WAV were never
covered by any. AAC-LC dates from 1997 and its own patents are widely held
to have run out, but a patent pool still offers an AAC license that bundles
it with the later HE-AAC additions, which this code leaves out. This is not
legal advice; if you ship a product, check for yourself.
