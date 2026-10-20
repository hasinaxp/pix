// pix - base types, memory and logging. Every other module includes this one.
//
// ---- how the library is built ----
//
// pix is header only, in the stb style. Every module declares its API at the
// top and keeps its implementation in a block guarded by PIX_IMPLEMENTATION.
// Exactly one .cpp in a program defines it before including pix:
//
//     #define PIX_IMPLEMENTATION
//     #include "pix.h"
//
// and every other file just includes pix.h. With a single-file game, that one
// file is the whole program.
#ifndef PIX_DTYPE_HPP
#define PIX_DTYPE_HPP

// must come before the first CRT header anywhere in the file that includes pix
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifndef PIXDEF
#define PIXDEF extern
#endif

// Keep <windows.h> from defining min/max and pulling in winsock, OLE and the
// rest of the kitchen sink into every file that includes pix.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

// arenas, the global heap, scopes and stack buffers - everything below
// allocates through it
#include "memory.hpp"

#define KB (1024)
#define MB (1024 * 1024)
#define GB (1024ull * 1024ull * 1024ull)

#define PIX_INVALID ((idx)-1)
#define PIX_ARRAY_COUNT(a) (sizeof(a) / sizeof((a)[0]))

typedef uint32_t idx;

struct vec2 { float x; float y; };
struct vec3 { float x; float y; float z; };
struct vec4 { float x; float y; float z; float w; };

struct quat { float x; float y; float z; float w; }; // rotation, w last (glTF order)

struct mat4 { float data[16]; }; // column-major

// A screen or atlas rectangle: top-left corner plus size, in pixels.
struct pix_rect { float x; float y; float w; float h; };

// 8-bit sRGB colour, straight (not premultiplied) alpha. What the 2D API and
// every "tint" parameter take.
struct pix_color { uint8_t r; uint8_t g; uint8_t b; uint8_t a; };

static inline pix_color pix_rgba(int r, int g, int b, int a = 255) {
    pix_color c = { (uint8_t)r, (uint8_t)g, (uint8_t)b, (uint8_t)a };
    return c;
}
// 0xRRGGBBAA, the way colours are written in design tools
static inline pix_color pix_hex(uint32_t rgba) {
    return pix_rgba((int)(rgba >> 24) & 255, (int)(rgba >> 16) & 255, (int)(rgba >> 8) & 255, (int)rgba & 255);
}
static inline pix_color pix_fade(pix_color c, float alpha) {
    float a = alpha < 0.0f ? 0.0f : (alpha > 1.0f ? 1.0f : alpha);
    c.a = (uint8_t)((float)c.a * a + 0.5f);
    return c;
}
static inline vec4 pix_color_to_vec4(pix_color c) {
    vec4 v = { c.r / 255.0f, c.g / 255.0f, c.b / 255.0f, c.a / 255.0f };
    return v;
}

static const pix_color PIX_WHITE      = { 255, 255, 255, 255 };
static const pix_color PIX_BLACK      = {   0,   0,   0, 255 };
static const pix_color PIX_BLANK      = {   0,   0,   0,   0 };
static const pix_color PIX_GRAY       = { 128, 128, 128, 255 };
static const pix_color PIX_LIGHTGRAY  = { 200, 200, 200, 255 };
static const pix_color PIX_DARKGRAY   = {  64,  64,  64, 255 };
static const pix_color PIX_RED        = { 230,  41,  55, 255 };
static const pix_color PIX_GREEN      = {   0, 200,  80, 255 };
static const pix_color PIX_BLUE       = {   0, 121, 241, 255 };
static const pix_color PIX_YELLOW     = { 253, 219,   0, 255 };
static const pix_color PIX_ORANGE     = { 255, 161,   0, 255 };
static const pix_color PIX_PURPLE     = { 160,  90, 220, 255 };
static const pix_color PIX_SKYBLUE    = { 102, 191, 255, 255 };

// ---------------- logging ----------------
//
// To stderr and to the debugger's output window, so a message is visible
// whether the game was started from a console or from Visual Studio.
PIXDEF void pix_log(const char* fmt, ...);

// Whole file into a zero-terminated buffer, global (free with pix_free) or
// PIX_MEM_SCOPED. Returns null when the file cannot be read.
PIXDEF unsigned char* pix_read_file(const char* path, size_t* out_size, unsigned mem = PIX_MEM_GLOBAL);

#endif // PIX_DTYPE_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_DTYPE_IMPL)
#define PIX_DTYPE_IMPL
#include <stdio.h>
#include <stdarg.h>
#include <windows.h>

void pix_log(const char* fmt, ...) {
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf) - 2, fmt, args);
    va_end(args);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 2) n = (int)sizeof(buf) - 2;
    buf[n] = '\n'; buf[n + 1] = 0;
    fputs(buf, stderr);
    OutputDebugStringA(buf);
}

unsigned char* pix_read_file(const char* path, size_t* out_size, unsigned mem) {
    if (out_size) *out_size = 0;
    FILE* f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size < 0) { fclose(f); return 0; }
    unsigned char* data = pix_new<unsigned char>((size_t)size + 1, mem & PIX_MEM_SCOPED);
    if (!data) { fclose(f); return 0; }
    size_t got = fread(data, 1, (size_t)size, f);
    fclose(f);
    data[got] = 0;
    if (out_size) *out_size = got;
    return data;
}

#endif // PIX_DTYPE_IMPL
