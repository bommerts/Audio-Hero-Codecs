# Exceptions

Exceptions to the rule that every decoder is written from a published
standard.

| Format | Why | What was done | Approved |
| --- | --- | --- | --- |
| Opus | RFC 6716 defines Opus by the reference decoder printed in it (BSD licence); its text describes, the code decides, and for CELT the text sends the reader to the code for most details. | The Opus files (`src/ahopus.c`, `src/ahopus_silk.c`, `src/ahopus_celt.c`, `src/ahopus_internal.h`, `src/ahopus_tables.c`) follow that reference decoder step by step (with the fixes of RFC 8251), and the tables are copied from it by a script, so each of these files carries its BSD notice. It differs from ffmpeg (libopus) only in the loss cover the [README](README.md#where-this-differs-from-ffmpeg-on-purpose) describes. | By the owner, 2026-11-18 |
| Apple Lossless (ALAC) | It has no standard of its own, only the reference decoder Apple published in 2011 (Apache 2.0). | This decoder is our own code that follows that decoder's behavior and includes none of Apple's code. It differs from ffmpeg in the corners the [README](README.md#where-this-differs-from-ffmpeg-on-purpose) lists. | By the owner, 2026-11-17 |
