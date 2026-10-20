// pix - shader and texture helpers over the raw GL loader.
#ifndef PIX_OPENGL_UTILS_HPP
#define PIX_OPENGL_UTILS_HPP
#include "dtype.hpp"
#include "opengl_api.hpp"

// sampling modes for opengl_create_texture2d
#define TEXTURE_PIXELATED 1 // nearest: pixel art, palettes, lookup tables
#define TEXTURE_LINEAR    2 // bilinear, no mipmaps: UI, atlases with tight packing
#define TEXTURE_BILINEAR  3 // trilinear + mipmaps + anisotropy: anything on a 3D surface
// or'd in: clamp at the edges instead of repeating
#define TEXTURE_CLAMP     0x100

// Compiles and links a program. On failure the log goes to stderr, to
// shader_error.txt and to a message box, and 0 is returned.
PIXDEF idx opengl_create_shader(const char* vsrc, const char* fsrc);
PIXDEF void opengl_destroy_shader(idx program);
// Shader errors always go to stderr and shader_error.txt; this controls the
// message box as well (on by default - turn it off for tools and tests, where
// nobody is there to click it and it looks exactly like a hang).
PIXDEF void opengl_set_error_dialogs(bool enabled);

// `channel` is 1 (grey, reads as rrr1), 2 (grey + alpha), 3 (rgb) or 4 (rgba);
// data is tightly packed 8 bits per channel, rows top to bottom. `data` may be
// null to allocate an empty texture to fill later with opengl_update_texture2d.
PIXDEF idx opengl_create_texture2d(int width, int height, int channel, const void* data,
                                   idx flag = TEXTURE_PIXELATED);
PIXDEF void opengl_update_texture2d(idx texture, int x, int y, int width, int height,
                                    int channel, const void* data);
PIXDEF void opengl_destroy_texture(idx texture);

// ---- your own uniform buffers ----
// A std140 block shared by every shader that declares
//     layout(std140, binding = N) uniform MyBlock { ... };
// Bindings 0 and 1 belong to pix (scene, lights); use 2 and up. Updates orphan
// the old storage, so writing every frame never stalls on the GPU.
struct pix_uniform_buffer { GLuint id; size_t size; GLuint binding; };
PIXDEF pix_uniform_buffer pix_create_uniform_buffer(size_t size, GLuint binding, const void* data = nullptr);
PIXDEF void pix_update_uniform_buffer(pix_uniform_buffer& buffer, const void* data, size_t size = 0, size_t offset = 0);
PIXDEF void pix_destroy_uniform_buffer(pix_uniform_buffer& buffer);

#endif // PIX_OPENGL_UTILS_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_OPENGL_UTILS_IMPL)
#define PIX_OPENGL_UTILS_IMPL
#include <stdio.h>

// A shader that will not build is fatal to the frame, so it is reported three
// ways: to stderr, which is all an unattended run can see; to a file, which
// survives the process; and in a message box for somebody sitting in front of
// it.
//
// The box alone was actively misleading. An unattended run would sit on it for
// as long as it was given, looking exactly like a hang, with the reason waiting
// behind a window nobody was there to click. Printing first means the log says
// what happened either way.
static bool opengl__dialogs = true;
void opengl_set_error_dialogs(bool enabled) { opengl__dialogs = enabled; }

static void opengl__shader_failed(const char* what, const char* log) {
    pix_log("%s\n%s", what, log);
    FILE* f = fopen("shader_error.txt", "a");
    if (f) { fprintf(f, "%s\n%s\n\n", what, log); fclose(f); }
    if (opengl__dialogs) MessageBoxA(0, log, what, MB_OK);
}

static GLuint opengl__compile(GLenum type, const char* src, bool* ok_out) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, 0);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048] = {};
        glGetShaderInfoLog(s, sizeof(log), 0, log);
        opengl__shader_failed(type == GL_VERTEX_SHADER ? "vertex shader compile error"
                                                       : "fragment shader compile error", log);
        *ok_out = false;
    }
    return s;
}

idx opengl_create_shader(const char* vsrc, const char* fsrc) {
    bool ok = true;
    GLuint v = opengl__compile(GL_VERTEX_SHADER, vsrc, &ok);
    GLuint f = opengl__compile(GL_FRAGMENT_SHADER, fsrc, &ok);
    if (!ok) { glDeleteShader(v); glDeleteShader(f); return 0; }
    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    GLint linked = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &linked);
    glDeleteShader(v);
    glDeleteShader(f);
    if (!linked) {
        char log[2048] = {};
        glGetProgramInfoLog(p, sizeof(log), 0, log);
        opengl__shader_failed("shader link error", log);
        glDeleteProgram(p);
        return 0;
    }
    return p;
}

void opengl_destroy_shader(idx program) {
    if (program) glDeleteProgram(program);
}

static void opengl__formats(int channel, GLint* internal, GLenum* format) {
    switch (channel) {
        case 1:  *internal = GL_R8;    *format = GL_RED;  break;
        case 2:  *internal = GL_RG8;   *format = GL_RG;   break;
        case 3:  *internal = GL_RGB8;  *format = GL_RGB;  break;
        default: *internal = GL_RGBA8; *format = GL_RGBA; break;
    }
}

idx opengl_create_texture2d(int width, int height, int channel, const void* data, idx flag) {
    GLint internal; GLenum format;
    opengl__formats(channel, &internal, &format);

    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);   // rgb and grey rows are not 4-byte multiples
    glTexImage2D(GL_TEXTURE_2D, 0, internal, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

    // one and two channel textures read the way an image viewer would show them
    if (channel == 1) {
        GLint swz[4] = { GL_RED, GL_RED, GL_RED, GL_ONE };
        glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swz);
    } else if (channel == 2) {
        GLint swz[4] = { GL_RED, GL_RED, GL_RED, GL_GREEN };
        glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swz);
    }

    idx mode = flag & 0xFF;
    if (mode == TEXTURE_BILINEAR) {
        if (data) glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // Near-universal since 2004 and core since 4.6; the single cheapest
        // win for any texture seen at a grazing angle (floors, roads, terrain).
        GLfloat max_aniso = 0.0f;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY, &max_aniso);
        if (max_aniso > 1.0f) glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY,
                                              max_aniso < 8.0f ? max_aniso : 8.0f);
        glGetError();   // swallow the enum error on a driver without the extension
    } else if (mode == TEXTURE_LINEAR) {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    GLint wrap = (flag & TEXTURE_CLAMP) ? GL_CLAMP_TO_EDGE : GL_REPEAT;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
    return tex;
}

void opengl_update_texture2d(idx texture, int x, int y, int width, int height,
                             int channel, const void* data) {
    GLint internal; GLenum format;
    opengl__formats(channel, &internal, &format);
    glBindTexture(GL_TEXTURE_2D, texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_2D, 0, x, y, width, height, format, GL_UNSIGNED_BYTE, data);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
}

void opengl_destroy_texture(idx texture) {
    if (texture) { GLuint t = texture; glDeleteTextures(1, &t); }
}

pix_uniform_buffer pix_create_uniform_buffer(size_t size, GLuint binding, const void* data) {
    pix_uniform_buffer b = {};
    b.size = (size + 15) & ~(size_t)15;
    b.binding = binding;
    glGenBuffers(1, &b.id);
    glBindBuffer(GL_UNIFORM_BUFFER, b.id);
    glBufferData(GL_UNIFORM_BUFFER, (GLsizeiptr)b.size, 0, GL_DYNAMIC_DRAW);
    if (data) glBufferSubData(GL_UNIFORM_BUFFER, 0, (GLsizeiptr)size, data);
    glBindBufferBase(GL_UNIFORM_BUFFER, binding, b.id);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    return b;
}

void pix_update_uniform_buffer(pix_uniform_buffer& b, const void* data, size_t size, size_t offset) {
    if (!b.id || !data) return;
    if (!size || offset + size > b.size) size = b.size - offset;
    glBindBuffer(GL_UNIFORM_BUFFER, b.id);
    // a whole-buffer write can hand the old storage back to the driver instead of waiting on it
    if (offset == 0 && size == b.size) glBufferData(GL_UNIFORM_BUFFER, (GLsizeiptr)b.size, 0, GL_DYNAMIC_DRAW);
    glBufferSubData(GL_UNIFORM_BUFFER, (GLintptr)offset, (GLsizeiptr)size, data);
    glBindBufferBase(GL_UNIFORM_BUFFER, b.binding, b.id);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void pix_destroy_uniform_buffer(pix_uniform_buffer& b) {
    if (b.id) glDeleteBuffers(1, &b.id);
    b = pix_uniform_buffer();
}

#endif // PIX_OPENGL_UTILS_IMPL
