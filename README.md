# Audio Hero Codecs

Audio decoders, an MP3 encoder, a sample rate converter and a mix to stereo
in plain C, written from the published standards with no dependencies beyond
the C standard library. They were built for the Audio Hero Companion, which
streams music to network audio devices, and are published here for anyone to
use. [EXCEPTIONS.md](EXCEPTIONS.md) lists the formats with no published
standard to follow, and the one (Opus) whose standard is code.

| Format | Decode | Encode | Header |
| --- | --- | --- | --- |
| MP3 (MPEG-1/2/2.5 Layer III) | yes | yes | `ahmp3.h` |
| MPEG Layers I and II (.mp1, .mp2) | yes | | `ahmp3.h` |
| FLAC, native (.flac) or in Ogg (.oga) | yes | | `ahflac.h`, `ahogg.h` |
| AAC-LC, as ADTS (.aac) or in MP4 (.m4a); HE-AAC detected | yes | | `ahaac.h`, `ahm4a.h` |
| Apple Lossless (ALAC), in MP4 (.m4a) | yes | | `ahalac.h`, `ahm4a.h` |
| Vorbis, in Ogg (.ogg) | yes | | `ahvorbis.h`, `ahogg.h` |
| Opus, in Ogg (.opus) | yes | | `ahopus.h`, `ahogg.h` |
| WAV | yes | | `ahwav.h` |

Around them:

| | Header |
| --- | --- |
| One reader for all of the above: tells the kind, decodes, trims, seeks | `ahreader.h` |
| Sample rate conversion, any rate to any rate | `ahresample.h` |
| Mixing 1 to 8 channels to stereo | `ahmix.h` |

The repository was first named Audio-Hero-MP3, when it held MP3 alone. Links
to the old name still work.

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
- Native FLAC files, and FLAC in Ogg through the same Ogg reader as Vorbis
- Output as whole numbers, exactly as encoded, with a helper for floats

**AAC decoder**
- AAC-LC (low complexity, the kind music files use), mono and stereo, all 13
  sample rates from 7.35 to 96 kHz
- ADTS streams, and MP4/M4A files through a small reader that finds the
  track, its settings, each packet, and the gapless trim (edit list,
  iTunes tag, or the track's own stated length)
- All of AAC-LC: every code book, pulses, mid/side and intensity stereo,
  perceptual noise substitution, temporal noise shaping, long and short
  windows of both shapes
- AAC stays AAC-LC. HE-AAC is detected and reported, so that a caller can
  hand such a file to the system's decoder (Windows plays it): by the SBR
  data in the fill elements of its frames (ADTS or MP4), and in MP4 also by
  object type 5 or 29, or by the sync word 0x2B7 after the plain settings.
  MP4 settings that say "no SBR" outright win over SBR data in the frames,
  as in ffmpeg. Parametric stereo (HE-AAC v2) is
  reported when the MP4 settings state it (type 29, or sync word 0x548); an
  ADTS stream cannot state it. Its spectral band replication and parametric
  stereo are not decoded: played here, such a file gives the plain AAC
  inside it, at half the sample rate, without the top of the treble (and in
  mono where parametric stereo was used).
- Refused with a reason: AAC Main, SSR and LTP, 960-sample frames, more than
  two channels, fragmented MP4

**Apple Lossless (ALAC) decoder**
- 16, 20, 24 and 32 bits, 1 to 8 channels, every sample rate, from M4A
  files through the same MP4 reader as AAC
- All of the format: samples sent as they are or predicted, the adaptive
  predictor in both of its modes, the adaptive Golomb code with its escapes
  and runs of zeros, mixed stereo pairs, low bytes sent apart, a short last
  packet, and the data and fill elements a packet may carry
- Apple's channel layout for each count, given out in WAV order with a
  channel mask
- Every packet stands alone, so a seek is exact and costs nothing
- Output as whole numbers, exactly as encoded, with a helper for floats

**Vorbis decoder**
- 1 to 8 channels, every sample rate, both block sizes from 64 to 8192
- All of the format: both floor types (line segments, and the LSP filter
  of encoders from before Vorbis 1.0), the three residue types, codebooks sparse,
  ordered and of a single entry, both kinds of value table, any number of
  modes, mappings, submaps and coupling steps
- The channels given out in WAV order with a channel mask
- A packet cut short is decoded as far as it goes, as the specification
  says

**Opus decoder**
- Every mode: SILK (speech), CELT (music) and the hybrid of the two, every
  bandwidth and frame size (2.5 to 60 ms, packets up to 120 ms), mono and
  stereo, and up to 8 channels in the streams of mapping family 0 or 1
- The switches between modes as RFC 6716 defines them, with the redundant
  frames encoders send there; the header's output gain applied
- Always 48 kHz; the channels given out in WAV order with a channel mask
- A lost or damaged packet is covered over for its length, the way libopus
  does it (see below), so the sound after it keeps its place

**Ogg reader**
- Finds the first Vorbis, FLAC or Opus stream in an Ogg file, past any other
  streams mixed in with it
- Gathers packets across pages, checks every page's checksum, and leaves
  out damaged pages and the packets they break
- Says where the stream ends and finds where to start reading for a seek,
  by halving the file and then walking its pages
- Applies what the pages state: the start of the sound (an encoder's run-in
  to drop, and Opus's pre-skip) and its end; an Opus seek starts 200 ms
  early so the decoder settles (RFC 7845 asks for at least 80 ms)

**WAV reader**
- RIFF/WAVE and RF64, plain and extensible headers
- Whole-number samples of 8, 16, 24 and 32 bits; floats of 32 and 64 bits;
  up to 8 channels
- Files whose length was never filled in (streams) are read to their end
- Not read: compressed kinds of WAV (ADPCM, A-law, mu-law)

**One reader for every format**
- Tells the kind of file from its contents, not its name
- Decodes a piece at a time through a read function you supply, so a file
  of any size plays in a small, fixed amount of memory
- Drops the run-in and padding of gapless MP3, M4A and Ogg files, so each
  file gives exactly its own sound
- Says how long the file is, and seeks to the sample

**Sample rate converter**
- Any rate from 1 to 768 kHz to any other, 1 to 8 channels, fed a piece at
  a time
- A windowed-sinc filter worked out for the two rates: sound up to 90.7% of
  the lower rate's half (20 kHz when either rate is 44.1 kHz) passes
  unchanged, and everything the lower rate cannot carry is removed
- The output lines up with the input to the sample, with no delay to
  account for

**Mix to stereo**
- One channel goes to both sides as it is; more than two are mixed down by
  where each speaker stands, with the bass effects channel left out, and
  turned down just far enough that the mix cannot clip

## How it is checked

`tests/` runs about 9,300 checks. With `ffmpeg` on the PATH it also uses
ffmpeg as an independent reference:

- **Every code of every table.** Each Huffman code of MP3 and of AAC is
  written into a frame and decoded by both decoders; the output must match.
- **Random frames using every feature.** Small frame writers in the tests
  build thousands of MP3, Layer I, Layer II, AAC, FLAC and ALAC frames at
  random that use every part of each format, well beyond what encoders
  choose to write. This decoder and ffmpeg must agree: to floating point
  precision for the lossy formats, bit for bit for FLAC and ALAC. For FLAC
  and ALAC the test also knows the samples it encoded, so the decoder is
  checked without ffmpeg too, including the parts of ALAC that ffmpeg reads
  differently (below).
- **Random Vorbis streams.** A Vorbis writer in the tests makes random
  setups and packets that use every part of the format, cut into Ogg pages
  of every size. Where a stream uses something ffmpeg does not read (below),
  the writer also makes its twin: the same sound written with only what
  ffmpeg reads (a type 2 value table as an equivalent type 1 table, for
  instance). This decoder must give a stream and its twin exactly the same
  sound, and ffmpeg must give the twin's to floating point precision. Floor
  lines that run past the end of a block are checked against the
  specification's steps followed to the letter.
- **Random Opus streams.** Packets of random frames in every mode, size and
  framing (all four packet codes, padding, the self-delimited framing of
  multistream packets), mono and stereo switching freely, up to 8 channels
  in up to 4 streams with silent channels, a random pre-skip, output gain and
  end cut. ffmpeg's libopus decoder must give the same sound to floating
  point precision. Files from libopus in 19 settings (SILK, hybrid and CELT,
  2.5 to 120 ms, constant bit rate, forward error correction, 3 to 8
  channels) must too.
- **The Opus test vectors.** With `AHOPUS_VECTORS` set to the folder of RFC
  8251's test vectors, the range decoder must end every packet in the state
  each vector records, and the sound must match the reference within one
  16-bit step. (The vectors are not part of this repository.)
- **HE-AAC detection.** MP4 settings that state HE-AAC in each way the
  standard allows, or leave it out, must be reported as they say. Two
  one-second HE-AAC files from the Windows Media Foundation encoder (ADTS
  and M4A, in `tests/fixtures/`) must be reported as HE-AAC, play their
  tones at 22.05 kHz, and last half as many samples as ffmpeg gives with
  SBR decoded.
- **Files from other encoders.** LAME, shine, twolame, libvorbis, ffmpeg's
  own MP2, AAC, FLAC and ALAC encoders, libopus, and the Windows Media Foundation MP3
  and AAC encoders: every file must decode exactly as ffmpeg decodes it.
  Ogg files from ffmpeg are also cut into pages anew at random, and must
  give the same sound. WAV files
  of every sample format must read bit for bit as ffmpeg reads them.
- **This encoder's output** must decode the same in ffmpeg as here, with no
  errors, for constant and variable bit rate, including input full of NaN,
  infinity and clipping.
- **Gapless.** Trimming by the header must match ffmpeg's to the sample, for
  LAME's files, for this encoder's files (which must come back exactly as
  long as the input, down to a file of one sample), and for M4A files.
- **The reader.** Files of every kind must come out as ffmpeg decodes them,
  to the same length; the length the reader states must be the length it
  gives; and a seek to any sample must give what a read from the start
  gives there (exactly for FLAC, ALAC, Vorbis and WAV, to floating point
  precision for the rest but Opus). An Opus decoder that starts mid-stream
  settles but not to the last bit: the reader decodes 200 ms before the
  target (RFC 7845 asks for at least 80). CELT has settled by then (80 ms
  left up to 7% of full scale). SILK's long-term predictor carries a steady
  tone from frame to frame, so on the tests' steady tones a SILK decoder
  started mid-stream stays about 1% of full scale off for seconds (libopus
  does the same). An Opus seek must line up to the sample and come within 2%
  of full scale; near the start, where it decodes from the start, it must
  be exact. Ogg files are also checked with their pages cut
  every way, down to a few bytes, and with other streams mixed in.
- **Sample rate conversion against the mathematics.** A tone resampled is
  the same tone at the new rate. For 18 pairs of rates, tones across the
  kept band must come out within 94 dB of that ideal (a level error under
  0.001 dB), and tones the new rate cannot carry must come out at least
  94 dB down. Feeding the sound in pieces of any size must not change a
  single sample.
- **Sample rate conversion against ffmpeg.** On band-limited test sound the
  output must match both of ffmpeg's resamplers (its own, and soxr at its
  most exact) within 86 dB of the sound, sample for sample.
- **Mixing down.** For each usual layout from 3 to 8 channels the mix must
  match ffmpeg's default mix within two steps of 16 bits.
- **Damage.** Damaged, cut-off and random input must never crash a decoder
  or make it produce more than the file could hold; frames with a failed CRC
  must be caught. Sound that is not sound (NaN, infinity) must leave the
  sample rate converter as soon as its filter has passed it. The whole suite
  also runs under AddressSanitizer.

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
- **Where AAC settings state HE-AAC.** The standard puts the sync word
  0x2B7 right after the plain AAC settings, and that is the only place this
  decoder looks; ffmpeg searches for it bit by bit. Settings with the sync
  word anywhere else are HE-AAC to ffmpeg and plain AAC-LC here. ffmpeg also
  leaves SBR open when the stated full rate equals the plain rate
  (downsampled SBR); this decoder reports such a file as HE-AAC, since it
  is one, though it plays at the rate stated rather than half of it. Only
  the report differs: the sound decoded here is the same.
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
- **ALAC corners no encoder uses.** ALAC has no standard but the decoder
  Apple published, and ffmpeg reads a few rare things otherwise: prediction
  modes 1 to 14 (Apple's decoder runs its extra first stage for every mode
  but 0, ffmpeg only for 15), mix weights over 127 (Apple reads the byte as
  signed), data and fill elements (Apple skips them, ffmpeg refuses the
  packet), a 32-bit pair sent as it is with no low bytes apart (ffmpeg
  refuses it), single channels and pairs in an order other than the usual
  one for the channel count (Apple places them in turn; ffmpeg puts a pair
  at its first channel's place and the next), and three cases in the
  Golomb code (a value of 65,535 right after a run of zeros, a running
  mean of 0 when a run follows, and a limit on the code's parameter below
  8). This decoder follows Apple's (see [EXCEPTIONS.md](EXCEPTIONS.md)).
  Damage is also met as Apple's decoder
  meets it: a run of zeros longer than the packet, or 24 low bits sent
  apart, makes the packet play as silence, where ffmpeg plays on.
- **Vorbis corners no encoder uses.** ffmpeg refuses a stream that uses a
  type 2 value table, a type 1 floor wider than half a long block, or a
  residue range that runs past the end of the vector or the wrong way
  round (the specification limits the range to the vector); it refuses a
  packet in which a book of one entry sends its one-bit codeword as 1 (the
  specification's errata allow either bit). It numbers the entries of a
  sparse book that has a value table as if the unused ones were not there,
  which changes what such a book gives when it is read as a number. When
  coupling steps share a channel, it passes on which channels need a
  residue in the opposite order to the specification's. And where a floor
  line runs past the end of a short block, it ends the line there, which
  bends it, instead of drawing it whole and cutting it. A type 0 floor
  that names a book it does not have makes the packet undecodable, the
  specification says; ffmpeg uses the floor's first book. This decoder
  follows the specification in all of these.
- **Vorbis type 0 floors.** The specification gives the LSP floor as a
  formula; ffmpeg works it out in single precision, this decoder in double,
  so the two can part by up to about one part in a thousand of the loudest
  sample. Only encoders from before Vorbis 1.0 (2002) wrote this floor type.
- **Ogg Vorbis start.** When the first page of sound states a position
  lower than its packets give, the specification says the samples before
  it are to be dropped (an encoder's way to mark its run-in). ffmpeg plays
  them; this decoder drops them.
- **Ogg Vorbis end.** The last page's position cuts the last packet short.
  ffmpeg works the cut out, but with some ways of laying the packets out
  in pages it does not make it, and plays the whole packet. This decoder
  always cuts where the page says. Files from libvorbis, laid out in pages
  every way the tests try, were cut the same by both.
- **Short Ogg Vorbis files.** When all of a stream's sound fits on one page,
  as it does in a file of a second or so, ffmpeg counts the first packet,
  which gives no sound, as if it did, and cuts that much too much from the
  end: 128 samples from libvorbis's files. This decoder ends where the page
  says, which gives back exactly as many samples as went into the encoder.
- **Opus loss cover.** A lost packet (or one libopus refuses as damaged,
  in whose place ffmpeg plays nothing) is covered over for the length it
  states, so the sound after it stays where the pages put it. RFC 6716
  leaves the cover to the decoder. This decoder follows libopus where its
  way is known: SILK's cover from the RFC's reference decoder, and noise in
  the shape of the last band energies for CELT bands. libopus covers a lost
  CELT-only frame with a pitch-based method of its own, so the sound in its
  place differs. The same cover joins a switch from SILK or hybrid to CELT
  that the encoder sent without a redundant frame: libopus's own files match
  there, but in random streams the 5 ms join can differ, and the tests leave
  those 5 ms out. A page whose checksum fails is left out with its
  packets (as for Vorbis), so its sound is missing, not covered.
- **Opus start before the first page.** A stream whose first page of sound
  states fewer samples than its packets hold (and is not its last page) is
  invalid by RFC 7845. It is played as an Ogg Vorbis start would be, with
  the samples before position 0 dropped.
- **Mono to stereo.** ffmpeg plays a one-channel file 3 dB down on each
  side. The mix here puts it on both sides as it is, so a mono file is no
  quieter than a stereo one.

Apart from short Ogg Vorbis files and Opus switches into CELT without a
redundant frame, encoders in common use write none of these, so files play
the same in both.

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
| ALAC decode (16-bit, 44.1 kHz) | 500 |
| Vorbis decode (44.1 kHz stereo, quality 5) | 600 |
| Opus decode, 128 kbps stereo music (CELT) | 250 |
| Opus decode, 16 kbps mono speech (SILK) | 600 |
| MP3 encode | 50 to 100 |
| Sample rate conversion, 48 to 44.1 kHz stereo | 500 |

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
/* info->delay and info->samples say what to keep for gapless playback.
   When info->codec is AHM4A_ALAC the packets are Apple Lossless:
   ahalac_decoder_configure and ahalac_decode_packet (ahalac.h) take the
   same places. */
```

```c
#include "ahogg.h"
#include "ahvorbis.h"

/* Ogg Vorbis: the Ogg reader finds the packets, the Vorbis decoder decodes
   them. */
ahogg *o = ahogg_open(my_read_function, my_file, file_size, &error);
ahvorbis_decoder *d = ahvorbis_decoder_new();
for (int i = 0; i < 3; i++) {
    ahogg_header(o, i, &data, &bytes);
    ahvorbis_decoder_header(d, data, bytes, &format);
}
float pcm[AHVORBIS_MAX_SAMPLES * AHVORBIS_MAX_CHANNELS];
while (ahogg_packet(o, &data, &bytes, &granule))
    ahvorbis_decode_packet(d, data, bytes, pcm, &samples);
/* The pages' granule positions say where the sound starts and ends; the
   reader below applies them, and seeks. Opus is the same with ahopus.h:
   ahopus_decoder_header() takes the first header (OpusHead), and the
   header's pre-skip is dropped from the start. */
```

Or let the reader pick the decoder, and bring whatever the file holds to
44.1 kHz stereo:

```c
#include "ahreader.h"
#include "ahmix.h"
#include "ahresample.h"

ahreader *r = ahreader_open(my_read_function, my_file, file_size, &error);
const ahreader_info *info = ahreader_get_info(r); /* kind, rate, channels, length */
ahmix mix;
ahmix_init(&mix, info->channels, info->channel_mask);
ahresample *rs = ahresample_new(info->sample_rate, 44100, 2);

float pcm[8 * 1024], out[2 * 1024];
size_t n, used, made;
while ((n = ahreader_read(r, pcm, 1024)) != 0) {
    ahmix_to_stereo(&mix, pcm, n, pcm);
    for (size_t pos = 0; pos < n; pos += used) {
        made = ahresample_process(rs, pcm + 2 * pos, n - pos, &used, out, 1024);
        /* `made` stereo samples at 44.1 kHz are in out */
    }
}
while ((made = ahresample_flush(rs, out, 1024)) != 0) { /* the last of it */ }
ahreader_seek(r, 30 * (uint64_t)info->sample_rate); /* to 0:30 */
```

Each header documents its whole interface: [ahmp3.h](include/ahmp3.h),
[ahflac.h](include/ahflac.h), [ahaac.h](include/ahaac.h),
[ahalac.h](include/ahalac.h), [ahm4a.h](include/ahm4a.h),
[ahvorbis.h](include/ahvorbis.h), [ahopus.h](include/ahopus.h), [ahogg.h](include/ahogg.h),
[ahwav.h](include/ahwav.h),
[ahreader.h](include/ahreader.h), [ahresample.h](include/ahresample.h),
[ahmix.h](include/ahmix.h).

There is also a small command line tool. It tells the kind of file from its
contents:

```
ahmp3 decode in.flac out.wav
ahmp3 encode in.m4a out.mp3 --bitrate 192
ahmp3 decode in.ogg out.wav
ahmp3 encode in.wav out.mp3 --vbr 8
ahmp3 encode surround96k.flac out.mp3 --rate 44100
```

Encoding mixes more than two channels down to stereo, and changes the sample
rate when MP3 has no such rate or `--rate` asks for one.

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
| Apple Lossless | `ahalac.c` |
| Vorbis | `ahvorbis.c` |
| Opus | `ahopus.c`, `ahopus_silk.c`, `ahopus_celt.c`, `ahopus_tables.c` |
| M4A | `ahm4a.c` |
| Ogg | `ahogg.c` |
| WAV | `ahwav.c` |
| The reader | `ahreader.c` and all of the decoders |
| Sample rate conversion | `ahresample.c` |
| Mixing to stereo | `ahmix.c` |

## Not done

- The length of an MP3 file that carries no length header is worked out
  from its size and bit rate, which is only right when the bit rate is
  constant. A seek in such a file is still exact.
- Decoding HE-AAC's spectral band replication and parametric stereo (it is
  detected and reported instead), and AAC with more than two channels.
- Parametric stereo in an ADTS stream, or in MP4 settings that do not state
  it, is not detected: finding it means decoding the SBR data it sits in.
- Apple Lossless packets of more than 65,536 samples. Encoders write 4,096;
  ffmpeg reads up to about 16 million, but a decoder holding that much
  would not be small.
- The MD5 check of a whole FLAC file.
- Chained Ogg files (one stream after another, as recordings of internet
  radio often are) give only their first stream.
- The other codecs an Ogg file may hold (Speex, Theora video).
- Opus with more than 8 channels, and Opus mapping families other than 0
  and 1 (family 255, channels with no stated speaker, and the ambisonics
  families 2 and 3).
- Vorbis with more than 8 channels, and Vorbis setups with more than a
  million codebook entries or 4 million vector values in all (libvorbis's
  setups use far fewer).
- The CRC that an ADTS frame may carry is not checked.

## License

MIT, see [LICENSE](LICENSE), except the Opus files (`src/ahopus.c`,
`src/ahopus_silk.c`, `src/ahopus_celt.c`, `src/ahopus_internal.h` and
`src/ahopus_tables.c`): they follow the reference decoder in RFC 6716 step
by step, and their tables are copied from it, so they are under that code's
BSD licence, whose notice is at the top of each file and must go along with
the files or a program built from them.

On patents: the MP3 patents have expired, FLAC and WAV were never covered
by any, Xiph.Org published Ogg and Vorbis for anyone to use free of
royalties, the companies that developed Opus (an IETF standard) license
their patents on it free of royalties (a pool formed in 2022 says other
patents read on it), and Apple published Apple Lossless as open source, free of
royalties, in 2011. AAC-LC dates from 1997 and its own patents are widely
held to have run out, but a patent pool still offers an AAC license that
bundles it with the later HE-AAC additions, which this code detects but
does not decode.
This is not legal advice; if you ship a product, check for yourself.
