// pix - immediate-mode 2D: shapes, textures, text. Everything drawn between
// flushes goes into one CPU vertex array and out in as few draw calls as the
// texture changes allow (one, for a game that uses a sprite atlas).
//
//   pix_batch2d b;
//   pix_batch2d_create(b);
//   pix_batch2d_begin(b, width, height);
//   pix_batch_rect(b, { 10, 10, 100, 50 }, PIX_RED);
//   pix_batch_text(b, font, "hello", 10, 80, 24, PIX_WHITE);
//   pix_batch2d_flush(b);
//
// The app layer (app.hpp) owns one of these and wraps every call as
// pix_draw_*, so most games never touch the batch directly.
#ifndef PIX_DRAW2D_HPP
#define PIX_DRAW2D_HPP
#include "dtype.hpp"
#include "math.hpp"
#include "opengl_utils.hpp"
#include "texture.hpp"
#include "font.hpp"

// A 2D camera, raylib style: `target` is the world point that appears at
// `offset` on screen, rotated by `rotation` degrees and scaled by `zoom`.
struct pix_camera2d {
    vec2  offset;
    vec2  target;
    float rotation;
    float zoom;       // 0 is treated as 1
};

struct pix__vertex2d {
    float x, y, u, v;
    pix_color color;
    pix_color outline;
    uint8_t mode;        // 0 texture * colour, 255 SDF glyph
    uint8_t weight;      // SDF edge bias, 128 = normal
    uint8_t outline_w;   // SDF outline width, 0..255 -> 0..0.5
    uint8_t pad;
};

struct pix_text_style2d {
    float size;               // pixel height of a line; 0 = font's baked size
    float spacing;            // extra pixels after each glyph
    float weight;             // -0.25 thin .. +0.25 bold
    float outline;            // 0 none, ~0.1..0.3 visible
    pix_color outline_color;
};

struct pix_batch2d {
    GLuint vao, vbo, program;
    GLint  u_proj, u_tex;
    pix__vertex2d* verts;
    size_t count, capacity;   // vertices
    idx    texture;           // bound to the vertices pending
    idx    white;
    mat4   projection;        // screen space, y down
    mat4   transform;         // projection * camera
    int    width, height;
    int    draws;             // draw calls since begin
    bool   scissor;
};

PIXDEF void pix_batch2d_create(pix_batch2d& b, size_t capacity = 65536);
PIXDEF void pix_batch2d_destroy(pix_batch2d& b);
// Starts screen-space drawing into a target of this size (the window, or a
// render target). Pixel (0,0) is the top-left corner.
PIXDEF void pix_batch2d_begin(pix_batch2d& b, int width, int height);
// World-space drawing through a camera; null returns to screen space.
PIXDEF void pix_batch2d_set_camera(pix_batch2d& b, const pix_camera2d* cam);
// Clip everything after this to a screen rectangle; w <= 0 turns clipping off.
PIXDEF void pix_batch2d_scissor(pix_batch2d& b, pix_rect rect);
PIXDEF void pix_batch2d_flush(pix_batch2d& b);

// ---- shapes ----
PIXDEF void pix_batch_rect(pix_batch2d& b, pix_rect r, pix_color c);
PIXDEF void pix_batch_rect_gradient(pix_batch2d& b, pix_rect r, pix_color top, pix_color bottom);
PIXDEF void pix_batch_rect_lines(pix_batch2d& b, pix_rect r, float thickness, pix_color c);
PIXDEF void pix_batch_rect_rounded(pix_batch2d& b, pix_rect r, float radius, pix_color c);
PIXDEF void pix_batch_rect_pro(pix_batch2d& b, pix_rect r, vec2 origin, float rotation_deg, pix_color c);
PIXDEF void pix_batch_line(pix_batch2d& b, vec2 a, vec2 c, float thickness, pix_color color);
PIXDEF void pix_batch_circle(pix_batch2d& b, vec2 center, float radius, pix_color c);
PIXDEF void pix_batch_circle_lines(pix_batch2d& b, vec2 center, float radius, float thickness, pix_color c);
PIXDEF void pix_batch_triangle(pix_batch2d& b, vec2 p0, vec2 p1, vec2 p2, pix_color c);
// convex polygon, points in order
PIXDEF void pix_batch_polygon(pix_batch2d& b, const vec2* points, int count, pix_color c);

// ---- textures ----
PIXDEF void pix_batch_texture(pix_batch2d& b, const pix_texture& t, float x, float y, pix_color tint);
// `src` is a pixel rectangle in the texture (w/h negative to flip); `dst` is
// where it lands, rotated `rotation_deg` about `origin` (relative to dst's corner).
PIXDEF void pix_batch_texture_pro(pix_batch2d& b, const pix_texture& t, pix_rect src, pix_rect dst,
                                  vec2 origin, float rotation_deg, pix_color tint);

// ---- text (SDF: sharp at any size) ----
PIXDEF pix_text_style2d pix_text_style(float size);
PIXDEF void pix_batch_text(pix_batch2d& b, font_data& font, const char* text, float x, float y,
                           float size, pix_color color);
PIXDEF void pix_batch_text_ex(pix_batch2d& b, font_data& font, const char* text, vec2 pos,
                              const pix_text_style2d& style, pix_color color);
// width and height the text would take, multi-line aware
PIXDEF vec2 pix_measure_text(const font_data& font, const char* text, float size, float spacing = 0.0f);

#endif // PIX_DRAW2D_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_DRAW2D_IMPL)
#define PIX_DRAW2D_IMPL
#include <math.h>

static const char* const VSHDER_2D = R"(#version 440 core
layout(location=0) in vec2 aPos;
layout(location=1) in vec2 aUV;
layout(location=2) in vec4 aColor;
layout(location=3) in vec4 aOutline;
layout(location=4) in vec4 aParams;
uniform mat4 uProj;
out vec2 vUV;
out vec4 vColor;
out vec4 vOutline;
out vec4 vParams;
void main() {
    vUV = aUV; vColor = aColor; vOutline = aOutline; vParams = aParams;
    gl_Position = uProj * vec4(aPos, 0.0, 1.0);
}
)";

static const char* const FSHDER_2D = R"(#version 440 core
in vec2 vUV;
in vec4 vColor;
in vec4 vOutline;
in vec4 vParams;
out vec4 frag;
uniform sampler2D uTex;
void main() {
    if (vParams.x < 0.5) { frag = texture(uTex, vUV) * vColor; return; }
    // SDF glyph: 0.5 is the outline; fwidth keeps the edge one pixel wide at any scale
    float sdf = texture(uTex, vUV).r;
    float w = max(fwidth(sdf), 1e-4);
    float fill_edge = 0.5 - (vParams.y - 0.5);
    float outline_edge = fill_edge - vParams.z * 0.5;
    float fill = smoothstep(fill_edge - w, fill_edge + w, sdf);
    float outer = smoothstep(outline_edge - w, outline_edge + w, sdf);
    float t = outer > 1e-4 ? clamp(fill / outer, 0.0, 1.0) : 0.0;
    vec4 c = mix(vOutline, vColor, t);
    frag = vec4(c.rgb, c.a * outer);
}
)";

void pix_batch2d_create(pix_batch2d& b, size_t capacity) {
    b = pix_batch2d();
    b.capacity = capacity < 6 ? 6 : capacity - capacity % 6;
    b.verts = pix_alloc<pix__vertex2d>(b.capacity);
    b.program = opengl_create_shader(VSHDER_2D, FSHDER_2D);
    b.u_proj = glGetUniformLocation(b.program, "uProj");
    b.u_tex = glGetUniformLocation(b.program, "uTex");
    unsigned char white[4] = { 255, 255, 255, 255 };
    b.white = opengl_create_texture2d(1, 1, 4, white, TEXTURE_PIXELATED);
    b.texture = b.white;

    glGenVertexArrays(1, &b.vao);
    glGenBuffers(1, &b.vbo);
    glBindVertexArray(b.vao);
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(b.capacity * sizeof(pix__vertex2d)), 0, GL_STREAM_DRAW);
    GLsizei s = sizeof(pix__vertex2d);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(pix__vertex2d, x));
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(pix__vertex2d, u));
    glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, s, (void*)offsetof(pix__vertex2d, color));
    glVertexAttribPointer(3, 4, GL_UNSIGNED_BYTE, GL_TRUE, s, (void*)offsetof(pix__vertex2d, outline));
    glVertexAttribPointer(4, 4, GL_UNSIGNED_BYTE, GL_TRUE, s, (void*)offsetof(pix__vertex2d, mode));
    for (int i = 0; i < 5; i++) glEnableVertexAttribArray(i);
    glBindVertexArray(0);
    b.projection = b.transform = mat4_identity();
}

void pix_batch2d_destroy(pix_batch2d& b) {
    if (b.vbo) glDeleteBuffers(1, &b.vbo);
    if (b.vao) glDeleteVertexArrays(1, &b.vao);
    opengl_destroy_shader(b.program);
    opengl_destroy_texture(b.white);
    pix_free(b.verts);
    b = pix_batch2d();
}

void pix_batch2d_flush(pix_batch2d& b) {
    if (!b.count) return;
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    glUseProgram(b.program);
    glUniformMatrix4fv(b.u_proj, 1, GL_FALSE, b.transform.data);
    glUniform1i(b.u_tex, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, b.texture);
    glBindVertexArray(b.vao);
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    // orphan, then fill: the driver never waits for last flush's draw
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(b.capacity * sizeof(pix__vertex2d)), 0, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(b.count * sizeof(pix__vertex2d)), b.verts);
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)b.count);
    glBindVertexArray(0);
    b.count = 0;
    b.draws++;
}

void pix_batch2d_begin(pix_batch2d& b, int width, int height) {
    pix_batch2d_flush(b);
    b.width = width;
    b.height = height;
    b.projection = mat4_ortho(0.0f, (float)width, (float)height, 0.0f, -1.0f, 1.0f);
    b.transform = b.projection;
    b.draws = 0;
    if (b.scissor) { glDisable(GL_SCISSOR_TEST); b.scissor = false; }
}

void pix_batch2d_set_camera(pix_batch2d& b, const pix_camera2d* cam) {
    pix_batch2d_flush(b);
    if (!cam) { b.transform = b.projection; return; }
    float zoom = cam->zoom != 0.0f ? cam->zoom : 1.0f;
    // screen = offset + rotate(zoom * (world - target))
    mat4 m = mat4_translate(cam->offset.x, cam->offset.y, 0.0f);
    m = mat4_mul(m, mat4_rotate_z(cam->rotation * PIX_DEG2RAD));
    m = mat4_mul(m, mat4_scale(zoom, zoom, 1.0f));
    m = mat4_mul(m, mat4_translate(-cam->target.x, -cam->target.y, 0.0f));
    b.transform = mat4_mul(b.projection, m);
}

void pix_batch2d_scissor(pix_batch2d& b, pix_rect r) {
    pix_batch2d_flush(b);
    if (r.w <= 0.0f || r.h <= 0.0f) {
        glDisable(GL_SCISSOR_TEST);
        b.scissor = false;
        return;
    }
    glEnable(GL_SCISSOR_TEST);
    // GL's scissor origin is bottom-left
    glScissor((GLint)r.x, (GLint)((float)b.height - r.y - r.h), (GLsizei)r.w, (GLsizei)r.h);
    b.scissor = true;
}

// Room for `n` more vertices with `texture` bound, flushing if either changes.
static pix__vertex2d* pix__2d_reserve(pix_batch2d& b, size_t n, idx texture) {
    if (!texture) texture = b.white;
    if (texture != b.texture) { pix_batch2d_flush(b); b.texture = texture; }
    if (b.count + n > b.capacity) pix_batch2d_flush(b);
    if (n > b.capacity) return 0;
    pix__vertex2d* v = b.verts + b.count;
    b.count += n;
    memset(v, 0, n * sizeof(pix__vertex2d));
    return v;
}

static inline void pix__2d_set(pix__vertex2d& v, float x, float y, float u, float t, pix_color c) {
    v.x = x; v.y = y; v.u = u; v.v = t; v.color = c;
}

// two triangles from four corners in order (tl, tr, br, bl)
static void pix__2d_quad(pix_batch2d& b, idx tex, const vec2* p, const vec2* uv, const pix_color* c) {
    pix__vertex2d* v = pix__2d_reserve(b, 6, tex);
    if (!v) return;
    static const int order[6] = { 0, 1, 2, 0, 2, 3 };
    for (int i = 0; i < 6; i++) {
        int k = order[i];
        pix__2d_set(v[i], p[k].x, p[k].y, uv[k].x, uv[k].y, c[k]);
    }
}

void pix_batch_rect(pix_batch2d& b, pix_rect r, pix_color c) {
    vec2 p[4] = { {r.x, r.y}, {r.x + r.w, r.y}, {r.x + r.w, r.y + r.h}, {r.x, r.y + r.h} };
    vec2 uv[4] = { {0,0}, {1,0}, {1,1}, {0,1} };
    pix_color cs[4] = { c, c, c, c };
    pix__2d_quad(b, 0, p, uv, cs);
}

void pix_batch_rect_gradient(pix_batch2d& b, pix_rect r, pix_color top, pix_color bottom) {
    vec2 p[4] = { {r.x, r.y}, {r.x + r.w, r.y}, {r.x + r.w, r.y + r.h}, {r.x, r.y + r.h} };
    vec2 uv[4] = { {0,0}, {1,0}, {1,1}, {0,1} };
    pix_color cs[4] = { top, top, bottom, bottom };
    pix__2d_quad(b, 0, p, uv, cs);
}

void pix_batch_rect_pro(pix_batch2d& b, pix_rect r, vec2 origin, float rotation_deg, pix_color c) {
    float a = rotation_deg * PIX_DEG2RAD, cs = cosf(a), sn = sinf(a);
    vec2 local[4] = { {-origin.x, -origin.y}, {r.w - origin.x, -origin.y},
                      {r.w - origin.x, r.h - origin.y}, {-origin.x, r.h - origin.y} };
    vec2 p[4];
    for (int i = 0; i < 4; i++)
        p[i] = v2(r.x + local[i].x * cs - local[i].y * sn, r.y + local[i].x * sn + local[i].y * cs);
    vec2 uv[4] = { {0,0}, {1,0}, {1,1}, {0,1} };
    pix_color col[4] = { c, c, c, c };
    pix__2d_quad(b, 0, p, uv, col);
}

void pix_batch_line(pix_batch2d& b, vec2 a, vec2 e, float thickness, pix_color c) {
    vec2 d = v2sub(e, a);
    float len = v2len(d);
    if (len < 1e-6f) return;
    vec2 n = v2scale(v2(-d.y / len, d.x / len), thickness * 0.5f);
    vec2 p[4] = { v2add(a, n), v2add(e, n), v2sub(e, n), v2sub(a, n) };
    vec2 uv[4] = { {0,0}, {1,0}, {1,1}, {0,1} };
    pix_color cs[4] = { c, c, c, c };
    pix__2d_quad(b, 0, p, uv, cs);
}

void pix_batch_rect_lines(pix_batch2d& b, pix_rect r, float t, pix_color c) {
    pix_batch_rect(b, { r.x, r.y, r.w, t }, c);
    pix_batch_rect(b, { r.x, r.y + r.h - t, r.w, t }, c);
    pix_batch_rect(b, { r.x, r.y + t, t, r.h - 2 * t }, c);
    pix_batch_rect(b, { r.x + r.w - t, r.y + t, t, r.h - 2 * t }, c);
}

static int pix__circle_segments(float radius) {
    int n = (int)(radius * 0.75f) + 12;   // enough that the edge never looks faceted
    return n > 128 ? 128 : n;
}

void pix_batch_triangle(pix_batch2d& b, vec2 p0, vec2 p1, vec2 p2, pix_color c) {
    pix__vertex2d* v = pix__2d_reserve(b, 3, 0);
    if (!v) return;
    pix__2d_set(v[0], p0.x, p0.y, 0, 0, c);
    pix__2d_set(v[1], p1.x, p1.y, 0, 0, c);
    pix__2d_set(v[2], p2.x, p2.y, 0, 0, c);
}

void pix_batch_polygon(pix_batch2d& b, const vec2* pts, int count, pix_color c) {
    for (int i = 1; i + 1 < count; i++) pix_batch_triangle(b, pts[0], pts[i], pts[i + 1], c);
}

void pix_batch_circle(pix_batch2d& b, vec2 ctr, float radius, pix_color c) {
    int n = pix__circle_segments(radius);
    float step = PIX_TAU / (float)n;
    vec2 prev = v2(ctr.x + radius, ctr.y);
    for (int i = 1; i <= n; i++) {
        vec2 cur = v2(ctr.x + cosf(step * i) * radius, ctr.y + sinf(step * i) * radius);
        pix_batch_triangle(b, ctr, prev, cur, c);
        prev = cur;
    }
}

void pix_batch_circle_lines(pix_batch2d& b, vec2 ctr, float radius, float t, pix_color c) {
    int n = pix__circle_segments(radius);
    float step = PIX_TAU / (float)n, ro = radius + t * 0.5f, ri = radius - t * 0.5f;
    if (ri < 0.0f) ri = 0.0f;
    for (int i = 0; i < n; i++) {
        float a0 = step * i, a1 = step * (i + 1);
        vec2 p[4] = { v2(ctr.x + cosf(a0) * ro, ctr.y + sinf(a0) * ro), v2(ctr.x + cosf(a1) * ro, ctr.y + sinf(a1) * ro),
                      v2(ctr.x + cosf(a1) * ri, ctr.y + sinf(a1) * ri), v2(ctr.x + cosf(a0) * ri, ctr.y + sinf(a0) * ri) };
        vec2 uv[4] = { {0,0}, {1,0}, {1,1}, {0,1} };
        pix_color cs[4] = { c, c, c, c };
        pix__2d_quad(b, 0, p, uv, cs);
    }
}

void pix_batch_rect_rounded(pix_batch2d& b, pix_rect r, float radius, pix_color c) {
    float rad = minf(radius, minf(r.w, r.h) * 0.5f);
    if (rad <= 0.5f) { pix_batch_rect(b, r, c); return; }
    pix_batch_rect(b, { r.x + rad, r.y, r.w - 2 * rad, r.h }, c);
    pix_batch_rect(b, { r.x, r.y + rad, rad, r.h - 2 * rad }, c);
    pix_batch_rect(b, { r.x + r.w - rad, r.y + rad, rad, r.h - 2 * rad }, c);
    vec2 corners[4] = { {r.x + rad, r.y + rad}, {r.x + r.w - rad, r.y + rad},
                        {r.x + r.w - rad, r.y + r.h - rad}, {r.x + rad, r.y + r.h - rad} };
    int n = pix__circle_segments(rad) / 4 + 2;
    for (int k = 0; k < 4; k++) {
        float a0 = PIX_PI + (float)k * PIX_PI * 0.5f;
        for (int i = 0; i < n; i++) {
            float t0 = a0 + (PIX_PI * 0.5f) * (float)i / (float)n;
            float t1 = a0 + (PIX_PI * 0.5f) * (float)(i + 1) / (float)n;
            pix_batch_triangle(b, corners[k],
                               v2(corners[k].x + cosf(t0) * rad, corners[k].y + sinf(t0) * rad),
                               v2(corners[k].x + cosf(t1) * rad, corners[k].y + sinf(t1) * rad), c);
        }
    }
}

void pix_batch_texture_pro(pix_batch2d& b, const pix_texture& t, pix_rect src, pix_rect dst,
                           vec2 origin, float rotation_deg, pix_color tint) {
    if (!t.id || t.width <= 0 || t.height <= 0) return;
    float u0 = src.x / (float)t.width, v0 = src.y / (float)t.height;
    float u1 = (src.x + src.w) / (float)t.width, v1 = (src.y + src.h) / (float)t.height;
    float w = fabsf(dst.w), h = fabsf(dst.h);
    vec2 local[4] = { {-origin.x, -origin.y}, {w - origin.x, -origin.y},
                      {w - origin.x, h - origin.y}, {-origin.x, h - origin.y} };
    vec2 p[4];
    if (rotation_deg == 0.0f) {
        for (int i = 0; i < 4; i++) p[i] = v2(dst.x + local[i].x, dst.y + local[i].y);
    } else {
        float a = rotation_deg * PIX_DEG2RAD, cs = cosf(a), sn = sinf(a);
        for (int i = 0; i < 4; i++)
            p[i] = v2(dst.x + local[i].x * cs - local[i].y * sn, dst.y + local[i].x * sn + local[i].y * cs);
    }
    vec2 uv[4] = { {u0, v0}, {u1, v0}, {u1, v1}, {u0, v1} };
    pix_color cs[4] = { tint, tint, tint, tint };
    pix__2d_quad(b, t.id, p, uv, cs);
}

void pix_batch_texture(pix_batch2d& b, const pix_texture& t, float x, float y, pix_color tint) {
    pix_rect src = { 0, 0, (float)t.width, (float)t.height };
    pix_rect dst = { x, y, (float)t.width, (float)t.height };
    pix_batch_texture_pro(b, t, src, dst, v2(0, 0), 0.0f, tint);
}

// ---- text ----

static unsigned int pix__utf8_next(const char** p) {
    unsigned char c = (unsigned char)**p;
    if (c < 0x80) { (*p)++; return c; }
    int extra = (c & 0xE0) == 0xC0 ? 1 : (c & 0xF0) == 0xE0 ? 2 : (c & 0xF8) == 0xF0 ? 3 : 0;
    unsigned int cp = extra == 1 ? c & 0x1F : extra == 2 ? c & 0x0F : extra == 3 ? c & 0x07 : c;
    (*p)++;
    for (int i = 0; i < extra && ((unsigned char)**p & 0xC0) == 0x80; i++) {
        cp = (cp << 6) | ((unsigned char)**p & 0x3F);
        (*p)++;
    }
    return cp;
}

static float pix__font_scale(const font_data& f, float size) {
    float natural = f.ascent - f.descent;
    return (size > 0.0f && natural > 0.0f) ? size / natural : 1.0f;
}

pix_text_style2d pix_text_style(float size) {
    pix_text_style2d s = {};
    s.size = size;
    s.outline_color = PIX_BLACK;
    return s;
}

void pix_batch_text_ex(pix_batch2d& b, font_data& font, const char* text, vec2 pos,
                       const pix_text_style2d& st, pix_color color) {
    if (!text || !font.bitmap) return;
    if (!font.atlas_texture)
        font.atlas_texture = opengl_create_texture2d((int)font.width, (int)font.height, 1, font.bitmap,
                                                     TEXTURE_LINEAR | TEXTURE_CLAMP);
    float scale = pix__font_scale(font, st.size);
    float line = font.line_height * scale;
    float iw = 1.0f / (float)font.width, ih = 1.0f / (float)font.height;
    uint8_t weight = (uint8_t)clampf((st.weight + 0.5f) * 255.0f, 0.0f, 255.0f);
    uint8_t outline = (uint8_t)clampf(st.outline * 2.0f * 255.0f, 0.0f, 255.0f);
    // pos is the top-left of the first line; glyph offsets are from the baseline
    float pen_x = pos.x, pen_y = pos.y + font.ascent * scale;
    const char* p = text;
    while (*p) {
        unsigned int cp = pix__utf8_next(&p);
        if (cp == '\n') { pen_x = pos.x; pen_y += line; continue; }
        const glyph& g = font.glyphs[pix_font_slot(cp)];
        if (g.crop.z > 0.0f) {
            float x0 = pen_x + g.offset.x * scale, y0 = pen_y + g.offset.y * scale;
            float x1 = x0 + g.crop.z * scale, y1 = y0 + g.crop.w * scale;
            float u0 = g.crop.x * iw, v0 = g.crop.y * ih;
            float u1 = (g.crop.x + g.crop.z) * iw, v1 = (g.crop.y + g.crop.w) * ih;
            pix__vertex2d* v = pix__2d_reserve(b, 6, font.atlas_texture);
            if (!v) return;
            float xs[6] = { x0, x1, x1, x0, x1, x0 }, ys[6] = { y0, y0, y1, y0, y1, y1 };
            float us[6] = { u0, u1, u1, u0, u1, u0 }, vs[6] = { v0, v0, v1, v0, v1, v1 };
            for (int i = 0; i < 6; i++) {
                pix__2d_set(v[i], xs[i], ys[i], us[i], vs[i], color);
                v[i].outline = st.outline_color;
                v[i].mode = 255;
                v[i].weight = weight;
                v[i].outline_w = outline;
            }
        }
        pen_x += g.advance * scale + st.spacing;
    }
}

void pix_batch_text(pix_batch2d& b, font_data& font, const char* text, float x, float y,
                    float size, pix_color color) {
    pix_batch_text_ex(b, font, text, v2(x, y), pix_text_style(size), color);
}

vec2 pix_measure_text(const font_data& font, const char* text, float size, float spacing) {
    float scale = pix__font_scale(font, size);
    float w = 0.0f, max_w = 0.0f;
    int lines = 1;
    const char* p = text;
    while (p && *p) {
        unsigned int cp = pix__utf8_next(&p);
        if (cp == '\n') { max_w = maxf(max_w, w); w = 0.0f; lines++; continue; }
        w += font.glyphs[pix_font_slot(cp)].advance * scale + spacing;
    }
    return v2(maxf(max_w, w), (float)lines * font.line_height * scale);
}

#endif // PIX_DRAW2D_IMPL
