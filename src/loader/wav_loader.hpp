#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "../core/dtype.hpp"
#include "asset_types.hpp"

// Minimal RIFF/WAVE reader: PCM 8/16/24/32-bit and 32-bit IEEE float, mono or
// stereo. Everything is converted to interleaved 16-bit signed, which is what
// the mixer works in. No third-party code.
//
// Not handled: compressed formats (ADPCM, mp3-in-wav), >2 channels.

static bool wav_load_file(mem_arena& arena, const char* path, sound_file_data* out);

// ---------------- implementation ----------------

static uint32_t wav__u32(const unsigned char* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t wav__u16(const unsigned char* p) {
    return (uint16_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8));
}

static bool wav_load_file(mem_arena& arena, const char* path, sound_file_data* out) {
    *out = sound_file_data();

    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    // the raw file goes through malloc so only decoded pcm lands in the arena
    unsigned char* file = (unsigned char*)malloc((size_t)(size > 0 ? size : 1));
    if (!file) { fclose(f); return false; }
    fread(file, 1, (size_t)size, f);
    fclose(f);

    if (size < 12 || memcmp(file, "RIFF", 4) || memcmp(file + 8, "WAVE", 4)) { free(file); return false; }

    int format = 0, channels = 0, rate = 0, bits = 0;
    const unsigned char* data = 0;
    size_t data_size = 0;

    for (size_t p = 12; p + 8 <= (size_t)size; ) {
        const unsigned char* id = file + p;
        uint32_t len = wav__u32(file + p + 4);
        const unsigned char* body = file + p + 8;
        if (p + 8 + (size_t)len > (size_t)size) len = (uint32_t)((size_t)size - p - 8);

        if (!memcmp(id, "fmt ", 4) && len >= 16) {
            format   = wav__u16(body);
            channels = wav__u16(body + 2);
            rate     = (int)wav__u32(body + 4);
            bits     = wav__u16(body + 14);
        } else if (!memcmp(id, "data", 4)) {
            data = body;
            data_size = len;
        }
        p += 8 + len + (len & 1);          // chunks are word aligned
    }

    size_t bytes = (size_t)(bits / 8);
    if (!data || channels < 1 || channels > 2 || rate <= 0 || !bytes) { free(file); return false; }
    size_t frames = data_size / (bytes * (size_t)channels);
    if (!frames) { free(file); return false; }

    size_t total = frames * (size_t)channels;
    int16_t* pcm = allocate<int16_t>(arena, total);
    if (!pcm) { free(file); return false; }

    for (size_t i = 0; i < total; i++) {
        const unsigned char* s = data + i * bytes;
        int32_t v = 0;
        if (format == 3 && bits == 32) {                     // IEEE float, -1..1
            float fv; memcpy(&fv, s, 4);
            fv = fv < -1.0f ? -1.0f : (fv > 1.0f ? 1.0f : fv);
            v = (int32_t)(fv * 32767.0f);
        } else if (bits == 8) {                              // unsigned, midpoint 128
            v = ((int32_t)s[0] - 128) << 8;
        } else if (bits == 16) {
            v = (int16_t)wav__u16(s);
        } else if (bits == 24) {                             // sign-extend through the top byte
            v = (int32_t)((uint32_t)s[0] << 8 | (uint32_t)s[1] << 16 | (uint32_t)s[2] << 24) >> 16;
        } else if (bits == 32) {
            v = (int32_t)wav__u32(s) >> 16;
        } else { free(file); return false; }
        pcm[i] = (int16_t)(v < -32768 ? -32768 : (v > 32767 ? 32767 : v));
    }
    free(file);

    out->samples     = pcm;
    out->frame_count = frames;
    out->channels    = channels;
    out->rate        = rate;
    return true;
}
