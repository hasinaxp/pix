#pragma once
#include <windows.h>
#include "dtype.hpp"
#include "opengl_api.hpp"
#include "opengl_utils.hpp"

// Offscreen render targets.
//
// Two shapes cover nearly everything an engine does with them:
//
//   colour + depth   the scene buffer that post processing reads back. Its
//                    depth is a texture too, because fog and any later depth
//                    effect need to sample it.
//   depth only       a shadow map. No colour attachment at all, and the depth
//                    texture is set up for hardware comparison sampling so the
//                    shader gets a filtered in/out shadow term rather than a
//                    raw distance it has to compare itself.
//
// A target owns its textures and hands back their GL ids; binding one also
// sets the viewport, since forgetting that is the classic way to spend an hour
// wondering why half the screen is black.

struct framebuffer {
    idx fbo;
    idx color;       // 0 when depth only
    idx depth;       // always a texture here, never a renderbuffer
    int width;
    int height;
    bool hdr;
};

// `hdr` picks RGBA16F over RGBA8, which is what lets a bloom or tonemap pass
// work with values above 1 instead of clipping them at the source
static framebuffer pix_create_render_target(int width, int height, bool hdr = true,
                                            bool mipmapped = false);
static framebuffer pix_create_shadow_map(int width, int height);
static void pix_destroy_framebuffer(framebuffer& target);

static void pix_bind_framebuffer(const framebuffer& target);
static void pix_bind_backbuffer(int width, int height);

// a full-screen triangle, drawn with no vertex buffer at all - the vertex
// shader builds it from gl_VertexID
static void pix_draw_fullscreen(void);

// ---------------- implementation ----------------

// `mipmapped` is for a target something else samples at a heavy, uneven
// minification - a planar reflection, say, projected across a wide range of
// distances and grazing angles in one draw. Mip selection there is what turns
// a crisp, doubled copy of the scene (which reads as a sheet of chrome) into
// the softened, broken-up reflection a rippled surface actually shows; a
// single full-res sample cannot do that no matter how the normal distorts
// its UV.
static idx fb__make_color(int width, int height, bool hdr, bool mipmapped = false) {
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, hdr ? GL_RGBA16F : GL_RGBA8, width, height, 0,
                 GL_RGBA, hdr ? GL_HALF_FLOAT : GL_UNSIGNED_BYTE, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                    mipmapped ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    if (mipmapped) glGenerateMipmap(GL_TEXTURE_2D);   // valid to call before any data too
    return tex;
}

static idx fb__make_depth(int width, int height, bool comparison) {
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    if (comparison) {
        // sampler2DShadow: the hardware does the depth compare and, with LINEAR
        // filtering, blends four taps - free 2x2 percentage-closer filtering
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
        // anything outside the light's view is lit, not shadowed
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        float border[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    return tex;
}

static bool fb__complete(const char* what) {
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status == GL_FRAMEBUFFER_COMPLETE) return true;
    char message[128];
    wsprintfA(message, "incomplete framebuffer (%s): 0x%04X", what, (unsigned)status);
    MessageBoxA(0, message, "pix", MB_OK);
    return false;
}

static framebuffer pix_create_render_target(int width, int height, bool hdr, bool mipmapped) {
    framebuffer t = {};
    t.width = width;
    t.height = height;
    t.hdr = hdr;

    glGenFramebuffers(1, &t.fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, t.fbo);

    t.color = fb__make_color(width, height, hdr, mipmapped);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t.color, 0);

    t.depth = fb__make_depth(width, height, false);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, t.depth, 0);

    fb__complete("scene");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return t;
}

// `width` may be several tiles wide: cascaded shadows keep every cascade in one
// texture side by side, so the whole set needs one sampler and one bind, and a
// cascade is selected by offsetting the lookup rather than by swapping textures.
static framebuffer pix_create_shadow_map(int width, int height) {
    framebuffer t = {};
    t.width = width;
    t.height = height;

    glGenFramebuffers(1, &t.fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, t.fbo);

    t.depth = fb__make_depth(width, height, true);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, t.depth, 0);

    // no colour attachment: say so explicitly or the framebuffer is incomplete
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    fb__complete("shadow");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return t;
}

static void pix_destroy_framebuffer(framebuffer& t) {
    if (t.color) glDeleteTextures(1, &t.color);
    if (t.depth) glDeleteTextures(1, &t.depth);
    if (t.fbo)   glDeleteFramebuffers(1, &t.fbo);
    t = framebuffer();
}

static void pix_bind_framebuffer(const framebuffer& t) {
    glBindFramebuffer(GL_FRAMEBUFFER, t.fbo);
    glViewport(0, 0, t.width, t.height);
}

static void pix_bind_backbuffer(int width, int height) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, width, height);
}

// One triangle big enough to cover the screen beats two that meet down the
// middle: no seam, and every pixel is shaded exactly once.
static void pix_draw_fullscreen(void) {
    static GLuint vao = 0;
    if (!vao) glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
}
