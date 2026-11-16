/* Mixing down to stereo. */
#include "../include/ahmix.h"
#include <string.h>

uint32_t ahmix_default_layout(int channels)
{
    static const uint32_t layouts[AHMIX_MAX_CHANNELS + 1] = {
        0,
        AHMIX_FRONT_CENTER,
        AHMIX_FRONT_LEFT | AHMIX_FRONT_RIGHT,
        AHMIX_FRONT_LEFT | AHMIX_FRONT_RIGHT | AHMIX_FRONT_CENTER,
        AHMIX_FRONT_LEFT | AHMIX_FRONT_RIGHT | AHMIX_BACK_LEFT | AHMIX_BACK_RIGHT,
        AHMIX_FRONT_LEFT | AHMIX_FRONT_RIGHT | AHMIX_FRONT_CENTER | AHMIX_BACK_LEFT | AHMIX_BACK_RIGHT,
        AHMIX_FRONT_LEFT | AHMIX_FRONT_RIGHT | AHMIX_FRONT_CENTER | AHMIX_LFE | AHMIX_BACK_LEFT | AHMIX_BACK_RIGHT,
        AHMIX_FRONT_LEFT | AHMIX_FRONT_RIGHT | AHMIX_FRONT_CENTER | AHMIX_LFE | AHMIX_BACK_CENTER | AHMIX_SIDE_LEFT
            | AHMIX_SIDE_RIGHT,
        AHMIX_FRONT_LEFT | AHMIX_FRONT_RIGHT | AHMIX_FRONT_CENTER | AHMIX_LFE | AHMIX_BACK_LEFT | AHMIX_BACK_RIGHT
            | AHMIX_SIDE_LEFT | AHMIX_SIDE_RIGHT,
    };
    return channels >= 1 && channels <= AHMIX_MAX_CHANNELS ? layouts[channels] : 0;
}

bool ahmix_init(ahmix *m, int channels, uint32_t layout)
{
    static const float minus3 = 0.70710678f;
    memset(m, 0, sizeof(*m));
    if (channels < 1 || channels > AHMIX_MAX_CHANNELS) return false;
    m->channels = channels;
    if (channels == 1) {
        /* As loud on each side as it was: a mono file plays no quieter
           than a stereo one. */
        m->left[0] = m->right[0] = 1;
        return true;
    }
    if (channels == 2) {
        m->left[0] = m->right[1] = 1;
        return true;
    }

    int named = 0;
    for (uint32_t bits = layout; bits; bits &= bits - 1) named++;
    if (named != channels) layout = ahmix_default_layout(channels);

    int ch = 0;
    for (uint32_t speaker = 1; speaker && ch < channels; speaker <<= 1) {
        if (!(layout & speaker)) continue;
        float l = 0, r = 0;
        switch (speaker) {
        case AHMIX_FRONT_LEFT:
        case AHMIX_FRONT_LEFT_OF_CENTER: l = 1; break;
        case AHMIX_FRONT_RIGHT:
        case AHMIX_FRONT_RIGHT_OF_CENTER: r = 1; break;
        case AHMIX_FRONT_CENTER: l = r = minus3; break;
        case AHMIX_LFE: break;
        case AHMIX_BACK_LEFT:
        case AHMIX_SIDE_LEFT: l = minus3; break;
        case AHMIX_BACK_RIGHT:
        case AHMIX_SIDE_RIGHT: r = minus3; break;
        case AHMIX_BACK_CENTER: l = r = 0.5f; break;
        default: l = r = 0.5f; break; /* overhead and other speakers: a little to each side */
        }
        m->left[ch] = l;
        m->right[ch] = r;
        ch++;
    }

    /* Turn it down until neither side can go past full scale. */
    float sum_left = 0, sum_right = 0;
    for (int i = 0; i < channels; i++) {
        sum_left += m->left[i];
        sum_right += m->right[i];
    }
    float most = sum_left > sum_right ? sum_left : sum_right;
    if (most > 1) {
        for (int i = 0; i < channels; i++) {
            m->left[i] /= most;
            m->right[i] /= most;
        }
    }
    return true;
}

void ahmix_to_stereo(const ahmix *m, const float *in, size_t frames, float *out)
{
    int channels = m->channels;
    if (channels == 1) {
        /* Backwards, so that it also works in place. */
        for (size_t i = frames; i-- > 0;) out[2 * i] = out[2 * i + 1] = in[i];
        return;
    }
    if (channels == 2) {
        if (out != in) memmove(out, in, frames * 2 * sizeof(float));
        return;
    }
    for (size_t i = 0; i < frames; i++) {
        const float *s = in + i * (size_t)channels;
        float l = 0, r = 0;
        for (int ch = 0; ch < channels; ch++) {
            l += s[ch] * m->left[ch];
            r += s[ch] * m->right[ch];
        }
        out[2 * i] = l;
        out[2 * i + 1] = r;
    }
}
