// pix - textures from files or pixels. Shared by the 3D renderer (material
// textures) and the 2D API (sprites).
#ifndef PIX_TEXTURE_HPP
#define PIX_TEXTURE_HPP
#include "dtype.hpp"
#include "opengl_utils.hpp"
#include "../loader/png_loader.hpp"

struct pix_texture {
    idx id;       // GL texture name; 0 = not loaded
    int width;
    int height;
};

// Loads a PNG. `flags` is a TEXTURE_* sampling mode (TEXTURE_BILINEAR for
// anything drawn in 3D, TEXTURE_PIXELATED for pixel art), optionally or'd with
// TEXTURE_CLAMP. On failure the texture has id 0 and a message is logged.
PIXDEF pix_texture pix_load_texture(const char* path, idx flags = TEXTURE_BILINEAR);
// From tightly packed 8-bit pixels (channels 1..4), rows top to bottom.
PIXDEF pix_texture pix_texture_from_pixels(const void* pixels, int width, int height, int channels,
                                           idx flags = TEXTURE_BILINEAR);
PIXDEF void pix_unload_texture(pix_texture& texture);

// Decodes a PNG to RGBA8 without touching the GPU - for CPU-side use such as
// heightmaps and collision masks. Global (free with pix_free) or PIX_MEM_SCOPED.
PIXDEF unsigned char* pix_load_image_rgba(const char* path, int* width, int* height,
                                          unsigned mem = PIX_MEM_GLOBAL);

#endif // PIX_TEXTURE_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_TEXTURE_IMPL)
#define PIX_TEXTURE_IMPL

// One scratch arena for every decode, reserved once and rewound after each
// image: the decoded pixels live exactly as long as the upload needs them.
static mem_arena pix__image_scratch;

static bool pix__decode_png(const char* path, png_result* out) {
    if (!pix__image_scratch.data) pix__image_scratch = pix_arena_create(1ull * GB);
    arena_reset(pix__image_scratch);
    return png_load_file(pix__image_scratch, path, out);
}

pix_texture pix_texture_from_pixels(const void* pixels, int width, int height, int channels, idx flags) {
    pix_texture t = {};
    t.id = opengl_create_texture2d(width, height, channels, pixels, flags);
    t.width = width;
    t.height = height;
    return t;
}

pix_texture pix_load_texture(const char* path, idx flags) {
    pix_texture t = {};
    png_result png = {};
    if (!pix__decode_png(path, &png)) {
        pix_log("pix: could not load texture '%s' (only 8-bit PNG is supported)", path);
        return t;
    }
    t = pix_texture_from_pixels(png.pixels, png.width, png.height, 4, flags);
    arena_reset(pix__image_scratch);
    pix_arena_trim(pix__image_scratch);   // one huge image should not pin its memory forever
    return t;
}

unsigned char* pix_load_image_rgba(const char* path, int* width, int* height, unsigned mem) {
    png_result png = {};
    if (!pix__decode_png(path, &png)) return 0;
    size_t bytes = (size_t)png.width * (size_t)png.height * 4;
    unsigned char* out = pix_new<unsigned char>(bytes, mem & PIX_MEM_SCOPED);
    if (out) memcpy(out, png.pixels, bytes);
    if (width) *width = png.width;
    if (height) *height = png.height;
    arena_reset(pix__image_scratch);
    return out;
}

void pix_unload_texture(pix_texture& texture) {
    opengl_destroy_texture(texture.id);
    texture = pix_texture();
}

#endif // PIX_TEXTURE_IMPL
