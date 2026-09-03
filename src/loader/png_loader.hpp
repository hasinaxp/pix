#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "../core/dtype.hpp"

// Minimal PNG decoder: 8-bit, non-interlaced, colour types 0/2/3/4/6.
// Includes a small DEFLATE (RFC 1951) / zlib (RFC 1950) inflate. Output is RGBA8.
// No third-party code.

struct png_result {
    int width;
    int height;
    int channels;          // always 4 (RGBA)
    unsigned char* pixels; // width * height * 4, allocated from the passed arena
};

static bool png_load_file(mem_arena& arena, const char* path, png_result* out);
// same decoder over an already-resident buffer (glTF embeds textures in its binary chunk)
static bool png_load_memory(mem_arena& arena, const unsigned char* file, size_t size, png_result* out);


// ---------------- inflate ----------------

struct inf__bits {
    const unsigned char* data;
    size_t len;
    size_t pos;
    unsigned int buf;
    int count;
};

static int inf__bit(inf__bits* b) {
    if (b->count == 0) {
        b->buf = (b->pos < b->len) ? b->data[b->pos++] : 0;
        b->count = 8;
    }
    int r = b->buf & 1;
    b->buf >>= 1;
    b->count--;
    return r;
}

static int inf__bits_n(inf__bits* b, int n) {
    int v = 0;
    for (int i = 0; i < n; i++) v |= inf__bit(b) << i;
    return v;
}

struct inf__huff {
    unsigned short counts[16];
    unsigned short symbols[288];
};

static void inf__huff_build(inf__huff* h, const unsigned char* lengths, int n) {
    for (int i = 0; i < 16; i++) h->counts[i] = 0;
    for (int i = 0; i < n; i++) h->counts[lengths[i]]++;
    h->counts[0] = 0;
    unsigned short offs[16];
    offs[0] = 0;
    for (int i = 1; i < 16; i++) offs[i] = (unsigned short)(offs[i - 1] + h->counts[i - 1]);
    for (int i = 0; i < n; i++)
        if (lengths[i]) h->symbols[offs[lengths[i]]++] = (unsigned short)i;
}

static int inf__decode(inf__bits* b, inf__huff* h) {
    int code = 0, first = 0, index = 0;
    for (int len = 1; len < 16; len++) {
        code |= inf__bit(b);
        int count = h->counts[len];
        if (code - first < count) return h->symbols[index + (code - first)];
        index += count;
        first = (first + count) << 1;
        code <<= 1;
    }
    return -1;
}

static bool inf__run(inf__bits* b, unsigned char* out, size_t cap, size_t* produced) {
    static const unsigned short lbase[] = { 3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,35,43,51,59,67,83,99,115,131,163,195,227,258 };
    static const unsigned char  lext[]  = { 0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,0 };
    static const unsigned short dbase[] = { 1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,257,385,513,769,1025,1537,2049,3073,4097,6145,8193,12289,16385,24577 };
    static const unsigned char  dext[]  = { 0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13 };

    size_t o = 0;
    int last = 0;
    while (!last) {
        last = inf__bit(b);
        int type = inf__bits_n(b, 2);

        if (type == 0) {                          // stored
            b->count = 0;                         // align to byte boundary
            if (b->pos + 4 > b->len) return false;
            int len = b->data[b->pos] | (b->data[b->pos + 1] << 8);
            b->pos += 4;
            for (int i = 0; i < len; i++) {
                if (o >= cap || b->pos >= b->len) return false;
                out[o++] = b->data[b->pos++];
            }
            continue;
        }
        if (type != 1 && type != 2) return false;

        inf__huff lh, dh;
        if (type == 1) {                          // fixed Huffman
            unsigned char ll[288], dl[30];
            for (int i = 0;   i < 144; i++) ll[i] = 8;
            for (int i = 144; i < 256; i++) ll[i] = 9;
            for (int i = 256; i < 280; i++) ll[i] = 7;
            for (int i = 280; i < 288; i++) ll[i] = 8;
            for (int i = 0;   i < 30;  i++) dl[i] = 5;
            inf__huff_build(&lh, ll, 288);
            inf__huff_build(&dh, dl, 30);
        } else {                                  // dynamic Huffman
            int hlit  = inf__bits_n(b, 5) + 257;
            int hdist = inf__bits_n(b, 5) + 1;
            int hclen = inf__bits_n(b, 4) + 4;
            static const unsigned char ord[19] = { 16,17,18,0,8,7,9,6,10,5,11,4,12,3,13,2,14,1,15 };
            unsigned char cl[19] = {};
            for (int i = 0; i < hclen; i++) cl[ord[i]] = (unsigned char)inf__bits_n(b, 3);
            inf__huff clh;
            inf__huff_build(&clh, cl, 19);

            unsigned char lens[320] = {};
            int n = 0;
            while (n < hlit + hdist) {
                int sym = inf__decode(b, &clh);
                if (sym < 0) return false;
                if (sym < 16) { lens[n++] = (unsigned char)sym; }
                else if (sym == 16) { int r = inf__bits_n(b, 2) + 3; unsigned char pv = n ? lens[n - 1] : 0; while (r-- && n < 320) lens[n++] = pv; }
                else if (sym == 17) { int r = inf__bits_n(b, 3) + 3; while (r-- && n < 320) lens[n++] = 0; }
                else                { int r = inf__bits_n(b, 7) + 11; while (r-- && n < 320) lens[n++] = 0; }
            }
            inf__huff_build(&lh, lens, hlit);
            inf__huff_build(&dh, lens + hlit, hdist);
        }

        for (;;) {
            int sym = inf__decode(b, &lh);
            if (sym < 0) return false;
            if (sym == 256) break;
            if (sym < 256) {
                if (o >= cap) return false;
                out[o++] = (unsigned char)sym;
                continue;
            }
            sym -= 257;
            if (sym > 28) return false;
            int length = lbase[sym] + inf__bits_n(b, lext[sym]);
            int dsym = inf__decode(b, &dh);
            if (dsym < 0 || dsym > 29) return false;
            int dist = dbase[dsym] + inf__bits_n(b, dext[dsym]);
            if ((size_t)dist > o) return false;
            for (int i = 0; i < length; i++) {
                if (o >= cap) return false;
                out[o] = out[o - dist];
                o++;
            }
        }
    }
    *produced = o;
    return true;
}

static bool zlib__inflate(const unsigned char* src, size_t len, unsigned char* out, size_t cap, size_t* produced) {
    if (len < 2) return false;
    if ((src[1] & 0x20) != 0) return false; // preset dictionary unsupported
    inf__bits b = {};
    b.data = src + 2;
    b.len = len - 2;
    return inf__run(&b, out, cap, produced);
}

// ---------------- PNG ----------------

static uint32_t png__be32(const unsigned char* p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static int png__paeth(int a, int b, int c) {
    int p = a + b - c;
    int pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);
    if (pa <= pb && pa <= pc) return a;
    return (pb <= pc) ? b : c;
}

bool png_load_file(mem_arena& arena, const char* path, png_result* out) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char* file = allocate<unsigned char>(arena, (size_t)size);
    if (!file) { fclose(f); return false; }
    fread(file, 1, (size_t)size, f);
    fclose(f);
    return png_load_memory(arena, file, (size_t)size, out);
}

bool png_load_memory(mem_arena& arena, const unsigned char* file, size_t size, png_result* out) {
    static const unsigned char sig[8] = { 137,80,78,71,13,10,26,10 };
    if (size < 8 || memcmp(file, sig, 8) != 0) return false;

    int width = 0, height = 0, bitdepth = 0, colortype = 0, interlace = 0;
    unsigned char palette[256 * 3] = {};
    unsigned char palette_a[256];
    memset(palette_a, 255, sizeof(palette_a));

    // pass 1: header + total compressed size
    size_t idat_total = 0;
    for (size_t p = 8; p + 8 <= (size_t)size; ) {
        uint32_t clen = png__be32(file + p);
        const unsigned char* type = file + p + 4;
        const unsigned char* cdat = file + p + 8;
        if (p + 12 + clen > (size_t)size) break;
        if      (!memcmp(type, "IHDR", 4)) {
            width     = (int)png__be32(cdat);
            height    = (int)png__be32(cdat + 4);
            bitdepth  = cdat[8];
            colortype = cdat[9];
            interlace = cdat[12];
        }
        else if (!memcmp(type, "IDAT", 4)) idat_total += clen;
        else if (!memcmp(type, "IEND", 4)) break;
        p += 12 + clen;
    }
    if (width <= 0 || height <= 0 || bitdepth != 8 || interlace != 0) return false;

    int src_ch;
    switch (colortype) {
        case 0: src_ch = 1; break; // grey
        case 2: src_ch = 3; break; // rgb
        case 3: src_ch = 1; break; // palette
        case 4: src_ch = 2; break; // grey + alpha
        case 6: src_ch = 4; break; // rgba
        default: return false;
    }

    unsigned char* comp = allocate<unsigned char>(arena, idat_total ? idat_total : 1);
    size_t comp_len = 0;

    // pass 2: gather palette + IDAT bytes
    for (size_t p = 8; p + 8 <= (size_t)size; ) {
        uint32_t clen = png__be32(file + p);
        const unsigned char* type = file + p + 4;
        const unsigned char* cdat = file + p + 8;
        if (p + 12 + clen > (size_t)size) break;
        if      (!memcmp(type, "PLTE", 4)) memcpy(palette, cdat, clen < sizeof(palette) ? clen : sizeof(palette));
        else if (!memcmp(type, "tRNS", 4) && colortype == 3) memcpy(palette_a, cdat, clen < sizeof(palette_a) ? clen : sizeof(palette_a));
        else if (!memcmp(type, "IDAT", 4)) { memcpy(comp + comp_len, cdat, clen); comp_len += clen; }
        else if (!memcmp(type, "IEND", 4)) break;
        p += 12 + clen;
    }

    size_t stride = (size_t)width * src_ch;
    size_t raw_len = (stride + 1) * (size_t)height;
    unsigned char* raw = allocate<unsigned char>(arena, raw_len);
    size_t got = 0;
    if (!zlib__inflate(comp, comp_len, raw, raw_len, &got) || got != raw_len) return false;

    // unfilter in place into a tightly packed buffer
    unsigned char* lines = allocate<unsigned char>(arena, stride * (size_t)height);
    for (int y = 0; y < height; y++) {
        int ft = raw[y * (stride + 1)];
        const unsigned char* in = raw + y * (stride + 1) + 1;
        unsigned char* cur = lines + (size_t)y * stride;
        unsigned char* prev = lines + (size_t)(y - 1) * stride;
        for (size_t x = 0; x < stride; x++) {
            int a = (x >= (size_t)src_ch) ? cur[x - src_ch] : 0;
            int b = (y > 0) ? prev[x] : 0;
            int c = (y > 0 && x >= (size_t)src_ch) ? prev[x - src_ch] : 0;
            int v = in[x];
            switch (ft) {
                case 1: v += a; break;
                case 2: v += b; break;
                case 3: v += (a + b) / 2; break;
                case 4: v += png__paeth(a, b, c); break;
                default: break;
            }
            cur[x] = (unsigned char)v;
        }
    }

    // expand to RGBA8
    unsigned char* rgba = allocate<unsigned char>(arena, (size_t)width * height * 4);
    for (int i = 0; i < width * height; i++) {
        const unsigned char* s = lines + (size_t)i * src_ch;
        unsigned char* d = rgba + (size_t)i * 4;
        if (colortype == 0)      { d[0] = d[1] = d[2] = s[0]; d[3] = 255; }
        else if (colortype == 2) { d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; d[3] = 255; }
        else if (colortype == 3) { int e = s[0]; d[0] = palette[e*3]; d[1] = palette[e*3+1]; d[2] = palette[e*3+2]; d[3] = palette_a[e]; }
        else if (colortype == 4) { d[0] = d[1] = d[2] = s[0]; d[3] = s[1]; }
        else                     { d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; d[3] = s[3]; }
    }

    out->width = width;
    out->height = height;
    out->channels = 4;
    out->pixels = rgba;
    return true;
}
