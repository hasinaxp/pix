#ifndef PIX_HDR_LOADER_HPP
#define PIX_HDR_LOADER_HPP
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "../core/dtype.hpp"

// Radiance .hdr (RGBE) decoder: the format HDR environment maps ship in.
// Each pixel is a shared exponent and three 8-bit mantissas; scanlines are
// usually run-length encoded one channel at a time ("new" RLE), sometimes
// stored flat. Only the standard -Y H +X W orientation (rows top to bottom).
// No third-party code.

// Linear RGB floats, width * height * 3, rows top to bottom; free with pix_free
// (or nothing, for PIX_MEM_SCOPED). Null on failure.
PIXDEF float* hdr_load_file(const char* path, int* out_width, int* out_height, unsigned mem = PIX_MEM_GLOBAL);
PIXDEF float* hdr_load_memory(const unsigned char* file, size_t size, int* out_width, int* out_height,
                              unsigned mem = PIX_MEM_GLOBAL);

#endif // PIX_HDR_LOADER_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_HDR_LOADER_IMPL)
#define PIX_HDR_LOADER_IMPL

static void hdr__rgbe_to_float(const unsigned char* rgbe, float* out) {
    if (!rgbe[3]) { out[0] = out[1] = out[2] = 0.0f; return; }
    float f = ldexpf(1.0f, (int)rgbe[3] - (128 + 8));
    out[0] = rgbe[0] * f;
    out[1] = rgbe[1] * f;
    out[2] = rgbe[2] * f;
}

// One scanline of RGBE into `line` (width * 4 bytes). Returns false on bad data.
static bool hdr__read_line(const unsigned char*& p, const unsigned char* end, int width, unsigned char* line) {
    if (end - p < 4) return false;
    bool rle = width >= 8 && width < 32768 && p[0] == 2 && p[1] == 2 && !(p[2] & 0x80)
            && ((p[2] << 8) | p[3]) == width;
    if (!rle) {   // flat
        if (end - p < (ptrdiff_t)width * 4) return false;
        memcpy(line, p, (size_t)width * 4);
        p += (size_t)width * 4;
        return true;
    }
    p += 4;
    // the four channels one after another, each as runs and literal spans
    for (int c = 0; c < 4; c++) {
        int x = 0;
        while (x < width) {
            if (p >= end) return false;
            int n = *p++;
            if (n > 128) {
                n -= 128;
                if (p >= end || x + n > width) return false;
                unsigned char v = *p++;
                for (int i = 0; i < n; i++) line[(x++) * 4 + c] = v;
            } else {
                if (n == 0 || x + n > width || end - p < n) return false;
                for (int i = 0; i < n; i++) line[(x++) * 4 + c] = *p++;
            }
        }
    }
    return true;
}

float* hdr_load_memory(const unsigned char* file, size_t size, int* out_w, int* out_h, unsigned mem) {
    const unsigned char* p = file;
    const unsigned char* end = file + size;
    if (size < 11 || (memcmp(p, "#?RADIANCE", 10) != 0 && memcmp(p, "#?RGBE", 6) != 0)) return 0;
    // header lines up to the blank one; only FORMAT matters
    bool blank = false;
    while (p < end && !blank) {
        const unsigned char* nl = (const unsigned char*)memchr(p, '\n', (size_t)(end - p));
        if (!nl) return 0;
        if (nl == p) blank = true;
        else if (nl - p > 7 && memcmp(p, "FORMAT=", 7) == 0 && memcmp(p + 7, "32-bit_rle_rgbe", 15) != 0) return 0;
        p = nl + 1;
    }
    char res[64] = {};
    const unsigned char* nl = (const unsigned char*)memchr(p, '\n', (size_t)(end - p));
    if (!nl || nl - p >= (ptrdiff_t)sizeof(res)) return 0;
    memcpy(res, p, (size_t)(nl - p));
    p = nl + 1;
    int w = 0, h = 0;
    if (sscanf(res, "-Y %d +X %d", &h, &w) != 2 || w <= 0 || h <= 0 || w > 65536 || h > 65536) return 0;

    float* out = pix_new<float>((size_t)w * h * 3, mem);
    unsigned char* line = pix_new<unsigned char>((size_t)w * 4);
    if (!out || !line) { pix_free(line); if (!(mem & PIX_MEM_SCOPED)) pix_free(out); return 0; }
    for (int y = 0; y < h; y++) {
        if (!hdr__read_line(p, end, w, line)) {
            pix_free(line);
            if (!(mem & PIX_MEM_SCOPED)) pix_free(out);
            return 0;
        }
        for (int x = 0; x < w; x++) hdr__rgbe_to_float(line + x * 4, out + ((size_t)y * w + x) * 3);
    }
    pix_free(line);
    *out_w = w;
    *out_h = h;
    return out;
}

float* hdr_load_file(const char* path, int* out_w, int* out_h, unsigned mem) {
    size_t size = 0;
    unsigned char* file = pix_read_file(path, &size);
    if (!file) return 0;
    float* px = hdr_load_memory(file, size, out_w, out_h, mem);
    pix_free(file);
    return px;
}

#endif // PIX_HDR_LOADER_IMPL
