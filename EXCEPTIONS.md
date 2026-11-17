# Exceptions

Exceptions to the rule that every decoder is written from a published
standard.

| Format | Why | What was done | Approved |
| --- | --- | --- | --- |
| Apple Lossless (ALAC) | It has no standard of its own, only the reference decoder Apple published in 2011 (Apache 2.0). | This decoder is our own code that follows that decoder's behavior and includes none of Apple's code. It differs from ffmpeg in the corners the [README](README.md#where-this-differs-from-ffmpeg-on-purpose) lists. | By the owner, 2026-11-17 |
