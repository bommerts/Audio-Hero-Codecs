/* Shared by the test files: the check counter, growable buffers, files, and
   running ffmpeg. */
#ifndef AH_TESTS_COMMON_H
#define AH_TESTS_COMMON_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/ahreader.h"

extern int g_checks, g_failures;
extern bool g_ffmpeg; /* ffmpeg is on the PATH: the checks against it run */

#define CHECK(cond, ...)                                     \
    do {                                                     \
        g_checks++;                                          \
        if (!(cond)) {                                       \
            g_failures++;                                    \
            printf("FAIL %s:%d  ", __FILE__, __LINE__);      \
            printf(__VA_ARGS__);                             \
            printf("\n");                                    \
        }                                                    \
    } while (0)

extern unsigned g_seed; /* set it back to replay the same random choices */
unsigned rnd(void);
int rnd_in(int lo, int hi);

typedef struct {
    unsigned char *p;
    size_t len, cap;
} bytes;

void bytes_put(bytes *b, const void *d, size_t n);

typedef struct {
    float *p;
    size_t len, cap; /* floats */
    int channels, rate;
} pcm;

void pcm_put(pcm *s, const float *d, size_t n);

/* The path of a file in the scratch folder; out holds 600 characters. */
void path_of(char *out, const char *name);
bool write_file(const char *path, const void *d, size_t n);
bool read_file(const char *path, bytes *out);

/* Runs "ffmpeg -v error -nostdin -y <args>"; true when it succeeded. */
bool ffmpeg_run(const char *args);
/* Decodes a file with ffmpeg to raw samples of the given format (f32le,
   s16le, s32le ...); extra holds further input options or is NULL. */
bool ffmpeg_raw(const char *in_path, const char *extra, const char *format, bytes *out);

/* Largest difference, relative to the louder signal, in samples [from, to). */
double max_diff(const pcm *a, const pcm *b, size_t from, size_t to);

/* A six second stereo test signal at 48 kHz: tones, sweeps, noise bursts and
   clicks. Made once per run as source.wav in the scratch folder. */
void make_source(void);

void test_wav(void);
void test_flac(void);
void test_layers(void);
void test_aac(void);
void test_alac(void);
void test_vorbis(void);
void test_opus(void);
void test_convert(void);
void test_reader(void);
void test_chain(void);

/* Ogg pages, for the Ogg tests (in test_vorbis.c). */
typedef struct {
    bytes data;
    int64_t end;   /* granule position at the end of the packet */
    bool page_end; /* a page ends after it */
} opacket;
uint32_t ogg_crc(const uint8_t *p, size_t n);
/* Cuts the packets into pages: at most max_segs segments a page (a random
   number when 0), and a page end wherever a packet asks for one. */
void ogg_write(bytes *out, uint32_t serial, const opacket *pk, int count, int max_segs);

/* All the sound of a file in memory, through ahreader. Returns the samples
   per channel (0 when it cannot be opened; then *error says why). */
size_t read_through(const bytes *file, pcm *out, ahreader_info *info, const char **error);
bool same_pcm(const pcm *a, const pcm *b);

/* A read function over a file held in memory: user is its `bytes`. */
size_t test_memory_read(void *user, uint64_t offset, void *buffer, size_t n);

#endif
