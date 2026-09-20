#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../core/dtype.hpp"

// Minimal PNG encoder: 8-bit RGB, no dependencies and no third-party code.
//
// The one part of a PNG that looks like it needs a library is the IDAT stream,
// because it has to be zlib. It does not: deflate defines a *stored* block type
// that carries its payload verbatim, so a valid zlib stream is a two byte
// header, the rows chopped into stored blocks of at most 65535 bytes, and an
// Adler-32 of the uncompressed data. The file comes out about as large as the
// raw pixels, which for a map that is written once and looked at is a better
// trade than carrying a compressor.
//
// Rows are filtered with filter type 0 (none), which is what makes them
// literal; every other filter exists to help the compressor we are not running.

static uint32_t png__crc_table[256];
static bool     png__crc_ready = false;

static void png__init_crc(void) {
    for (uint32_t n = 0; n < 256; n++) {
        uint32_t c = n;
        for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        png__crc_table[n] = c;
    }
    png__crc_ready = true;
}

static uint32_t png__crc(const unsigned char* data, size_t len, uint32_t crc) {
    if (!png__crc_ready) png__init_crc();
    for (size_t i = 0; i < len; i++) crc = png__crc_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc;
}

static void png__u32(unsigned char* p, uint32_t v) {
    p[0] = (unsigned char)(v >> 24); p[1] = (unsigned char)(v >> 16);
    p[2] = (unsigned char)(v >> 8);  p[3] = (unsigned char)v;
}

// length, type, data, then the CRC of type+data - the same four steps for
// every chunk in the file
static void png__chunk(FILE* f, const char* type, const unsigned char* data, size_t len) {
    unsigned char head[4];
    png__u32(head, (uint32_t)len);
    fwrite(head, 1, 4, f);
    fwrite(type, 1, 4, f);
    if (len) fwrite(data, 1, len, f);

    uint32_t crc = png__crc((const unsigned char*)type, 4, 0xFFFFFFFFu);
    if (len) crc = png__crc(data, len, crc);
    png__u32(head, crc ^ 0xFFFFFFFFu);
    fwrite(head, 1, 4, f);
}

// `rgb` is width*height*3 bytes, top row first
static bool png_write_rgb(const char* path, const unsigned char* rgb, int width, int height) {
    FILE* f = fopen(path, "wb");
    if (!f) return false;

    static const unsigned char SIG[8] = { 137, 'P', 'N', 'G', 13, 10, 26, 10 };
    fwrite(SIG, 1, 8, f);

    unsigned char ihdr[13];
    png__u32(ihdr + 0, (uint32_t)width);
    png__u32(ihdr + 4, (uint32_t)height);
    ihdr[8] = 8;    // bit depth
    ihdr[9] = 2;    // colour type 2 = truecolour RGB
    ihdr[10] = 0;   // deflate
    ihdr[11] = 0;   // adaptive filtering
    ihdr[12] = 0;   // no interlace
    png__chunk(f, "IHDR", ihdr, sizeof(ihdr));

    // the raw stream: one filter byte per row, then the row's pixels
    size_t stride = (size_t)width * 3 + 1;
    size_t raw_size = stride * (size_t)height;
    unsigned char* raw = (unsigned char*)malloc(raw_size);
    if (!raw) { fclose(f); return false; }
    for (int y = 0; y < height; y++) {
        raw[y * stride] = 0;
        memcpy(raw + y * stride + 1, rgb + (size_t)y * width * 3, (size_t)width * 3);
    }

    // zlib: 0x78 0x01 is deflate, 32K window, no preset dictionary
    size_t blocks = (raw_size + 65534) / 65535;
    size_t idat_size = 2 + blocks * 5 + raw_size + 4;
    unsigned char* idat = (unsigned char*)malloc(idat_size);
    if (!idat) { free(raw); fclose(f); return false; }

    size_t o = 0;
    idat[o++] = 0x78;
    idat[o++] = 0x01;
    size_t left = raw_size, at = 0;
    while (left) {
        size_t n = left > 65535 ? 65535 : left;
        idat[o++] = (unsigned char)((n == left) ? 1 : 0);      // BFINAL on the last one
        idat[o++] = (unsigned char)(n & 0xFF);
        idat[o++] = (unsigned char)(n >> 8);
        idat[o++] = (unsigned char)(~n & 0xFF);
        idat[o++] = (unsigned char)((~n >> 8) & 0xFF);
        memcpy(idat + o, raw + at, n);
        o += n; at += n; left -= n;
    }

    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < raw_size; i++) {
        a = (a + raw[i]) % 65521;
        b = (b + a) % 65521;
    }
    png__u32(idat + o, (b << 16) | a);
    o += 4;

    png__chunk(f, "IDAT", idat, o);
    png__chunk(f, "IEND", 0, 0);

    free(idat);
    free(raw);
    fclose(f);
    return true;
}
