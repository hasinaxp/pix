// pix - immediate-mode 2D: shapes, textures, text. Everything drawn between
// flushes goes into one CPU vertex array and out in as few draw calls as
// possible:
//
//   * up to PIX_BATCH2D_SLOTS (16) textures share one draw - each vertex
//     names its slot - so sprites from several sheets, text and shapes
//     interleave freely; a flush happens only when a 17th texture turns up,
//     the blend mode, the camera or the clip rectangle changes, or the
//     buffer fills
//   * each flush uploads exactly the bytes it used, so even a frame forced
//     into many flushes stays cheap
//   * shapes sample a white texture that always sits in slot 0, so they never
//     cost a texture change at all
//   * quads are indexed (4 vertices, not 6), and every shape reserves its
//     vertices once - a circle is one reservation and no trigonometry per
//     segment
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
//
// Angles are degrees, 0 pointing right (+x), growing clockwise on screen
// (y points down).
#ifndef PIX_DRAW2D_HPP
#define PIX_DRAW2D_HPP
#include "dtype.hpp"
#include "math.hpp"
#include "random.hpp"
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

// Textures one draw can mix; slot 0 is always white. 16 is what every GL 4.x
// driver guarantees a fragment shader can sample.
#define PIX_BATCH2D_SLOTS 16

struct pix__vertex2d {
    float x, y, u, v;
    pix_color color;
    pix_color outline;
    uint8_t mode;        // 0 texture * colour, 255 SDF glyph
    uint8_t weight;      // SDF edge bias, 128 = normal
    uint8_t outline_w;   // SDF outline width, 0..255 -> 0..0.5
    uint8_t slot;        // which of the batch's textures to sample
};

struct pix_text_style2d {
    float size;               // pixel height of a line; 0 = font's baked size
    float spacing;            // extra pixels after each glyph
    float weight;             // -0.25 thin .. +0.25 bold
    float outline;            // 0 none, ~0.1..0.3 visible
    pix_color outline_color;
};

struct pix_batch2d {
    GLuint vao, vbo, ebo, program;
    GLint  u_proj;
    pix__vertex2d* verts;
    uint16_t*      indices;
    size_t count, capacity;               // vertices
    size_t index_count, index_capacity;
    idx    slots[PIX_BATCH2D_SLOTS];      // textures the pending vertices use
    int    slot_count;
    idx    white;
    int    blend;                         // PIX_BLEND_*
    mat4   projection;                    // screen space, y down
    mat4   transform;                     // projection * camera
    pix_camera2d camera;                  // the camera in use (parallax reads its target)
    bool   has_camera;
    int    width, height;
    bool   top_down;                      // the target stores its rows top first (2D canvases)
    int    draws;                         // draw calls since begin
    bool   scissor;
};

// `capacity` in vertices, at most 65536 (indices are 16-bit).
PIXDEF void pix_batch2d_create(pix_batch2d& b, size_t capacity = 65536);
PIXDEF void pix_batch2d_destroy(pix_batch2d& b);
// Starts screen-space drawing into a target of this size (the window, or a
// render target). Pixel (0,0) is the top-left corner. `top_down` is for
// targets read back as textures with their first row at the top (canvases);
// the window and 3D targets keep GL's bottom-up rows.
PIXDEF void pix_batch2d_begin(pix_batch2d& b, int width, int height, bool top_down = false);
// World-space drawing through a camera; null returns to screen space.
PIXDEF void pix_batch2d_set_camera(pix_batch2d& b, const pix_camera2d* cam);
// The world rectangle the batch shows right now, through its camera if one is
// set (the bounding box when rotated): what culling tests against.
PIXDEF pix_rect pix_batch2d_visible(const pix_batch2d& b);
// Clip everything after this to a screen rectangle; w <= 0 turns clipping off.
PIXDEF void pix_batch2d_scissor(pix_batch2d& b, pix_rect rect);
// PIX_BLEND_ALPHA (the default), _ADDITIVE, _MULTIPLY or _PREMULTIPLIED.
PIXDEF void pix_batch2d_set_blend(pix_batch2d& b, int blend);
PIXDEF void pix_batch2d_flush(pix_batch2d& b);

// ---- shapes ----
PIXDEF void pix_batch_rect(pix_batch2d& b, pix_rect r, pix_color c);
PIXDEF void pix_batch_rect_gradient(pix_batch2d& b, pix_rect r, pix_color top, pix_color bottom);
// one colour per corner: top-left, top-right, bottom-right, bottom-left
PIXDEF void pix_batch_rect_colors(pix_batch2d& b, pix_rect r, pix_color tl, pix_color tr,
                                  pix_color br, pix_color bl);
PIXDEF void pix_batch_rect_lines(pix_batch2d& b, pix_rect r, float thickness, pix_color c);
PIXDEF void pix_batch_rect_rounded(pix_batch2d& b, pix_rect r, float radius, pix_color c);
PIXDEF void pix_batch_rect_rounded_lines(pix_batch2d& b, pix_rect r, float radius, float thickness, pix_color c);
PIXDEF void pix_batch_rect_pro(pix_batch2d& b, pix_rect r, vec2 origin, float rotation_deg, pix_color c);
PIXDEF void pix_batch_line(pix_batch2d& b, vec2 a, vec2 c, float thickness, pix_color color);
// Connected segments with mitred joins; on very sharp corners the mitre is
// capped at twice the thickness instead of shooting off to a spike. `closed`
// joins the last point back to the first.
PIXDEF void pix_batch_polyline(pix_batch2d& b, const vec2* points, int count, float thickness, pix_color c,
                               bool closed = false);
PIXDEF void pix_batch_circle(pix_batch2d& b, vec2 center, float radius, pix_color c);
PIXDEF void pix_batch_circle_lines(pix_batch2d& b, vec2 center, float radius, float thickness, pix_color c);
PIXDEF void pix_batch_ellipse(pix_batch2d& b, vec2 center, float rx, float ry, pix_color c);
PIXDEF void pix_batch_ellipse_lines(pix_batch2d& b, vec2 center, float rx, float ry, float thickness, pix_color c);
// The band between two radii from one angle to another: arcs, gauges, cooldown
// rings. inner = 0 gives a pie slice.
PIXDEF void pix_batch_ring(pix_batch2d& b, vec2 center, float inner, float outer, float start_deg,
                           float end_deg, pix_color c);
PIXDEF void pix_batch_triangle(pix_batch2d& b, vec2 p0, vec2 p1, vec2 p2, pix_color c);
// convex polygon, points in order
PIXDEF void pix_batch_polygon(pix_batch2d& b, const vec2* points, int count, pix_color c);

// ---- textures ----
PIXDEF void pix_batch_texture(pix_batch2d& b, const pix_texture& t, float x, float y, pix_color tint);
// `src` is a pixel rectangle in the texture (w/h negative to flip); `dst` is
// where it lands, rotated `rotation_deg` about `origin` (relative to dst's corner).
PIXDEF void pix_batch_texture_pro(pix_batch2d& b, const pix_texture& t, pix_rect src, pix_rect dst,
                                  vec2 origin, float rotation_deg, pix_color tint);
// Any four corners with their texture coordinates (0..1), in order around
// the quad: the building block for sprites, skewed cards and custom meshes.
// `gl_texture` 0 draws untextured.
PIXDEF void pix_batch_quad(pix_batch2d& b, idx gl_texture, const vec2 corners[4], const vec2 uvs[4],
                           pix_color tint);
// A panel that stretches without distorting its frame: `border` (left, top,
// right, bottom, in source pixels) keeps the corners at their size, stretches
// the edges along one axis and the middle along both. If `dst` is smaller
// than the borders, they shrink together.
PIXDEF void pix_batch_nine_slice(pix_batch2d& b, const pix_texture& t, pix_rect src, vec4 border,
                                 pix_rect dst, pix_color tint);

// ---- parallax ----
// A texture as a background layer that scrolls at `factor` of the camera's
// movement: (0, 0) stays fixed like a distant sky, (0.5, 0.5) moves at half
// speed, (1, 1) moves with the world. `offset` is where its top-left sits in
// the world (at camera target 0), `scale` its size. `repeat_x` / `repeat_y`
// tile it endlessly across the view - one quad, the texture repeats on the
// GPU, so it needs its own texture (not one packed in an atlas).
PIXDEF void pix_batch_parallax(pix_batch2d& b, const pix_texture& t, vec2 factor, vec2 offset, float scale,
                               bool repeat_x, bool repeat_y, pix_color tint);

// ---- text (SDF: sharp at any size) ----
PIXDEF pix_text_style2d pix_text_style(float size);
PIXDEF void pix_batch_text(pix_batch2d& b, font_data& font, const char* text, float x, float y,
                           float size, pix_color color);
PIXDEF void pix_batch_text_ex(pix_batch2d& b, font_data& font, const char* text, vec2 pos,
                              const pix_text_style2d& style, pix_color color);
// width and height the text would take, multi-line aware
PIXDEF vec2 pix_measure_text(const font_data& font, const char* text, float size, float spacing = 0.0f);

// ---- cameras and rectangles ----

// Where a screen pixel lands in the world, and the reverse.
static inline vec2 pix_screen_to_world_2d(const pix_camera2d& cam, vec2 screen) {
    float zoom = cam.zoom != 0.0f ? cam.zoom : 1.0f;
    vec2 d = v2scale(v2sub(screen, cam.offset), 1.0f / zoom);
    return v2add(cam.target, v2rotate(d, -cam.rotation * PIX_DEG2RAD));
}
static inline vec2 pix_world_to_screen_2d(const pix_camera2d& cam, vec2 world) {
    float zoom = cam.zoom != 0.0f ? cam.zoom : 1.0f;
    return v2add(cam.offset, v2rotate(v2scale(v2sub(world, cam.target), zoom), cam.rotation * PIX_DEG2RAD));
}
// The world rectangle a `width` x `height` screen shows through the camera
// (its bounding box when rotated): skip drawing whatever lies outside it.
static inline pix_rect pix_camera2d_visible(const pix_camera2d& cam, float width, float height) {
    vec2 c[4] = { pix_screen_to_world_2d(cam, v2(0, 0)), pix_screen_to_world_2d(cam, v2(width, 0)),
                  pix_screen_to_world_2d(cam, v2(width, height)), pix_screen_to_world_2d(cam, v2(0, height)) };
    vec2 lo = c[0], hi = c[0];
    for (int i = 1; i < 4; i++) {
        lo = v2(minf(lo.x, c[i].x), minf(lo.y, c[i].y));
        hi = v2(maxf(hi.x, c[i].x), maxf(hi.y, c[i].y));
    }
    pix_rect r = { lo.x, lo.y, hi.x - lo.x, hi.y - lo.y };
    return r;
}
static inline bool pix_rect_overlaps(pix_rect a, pix_rect b) {
    return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}
static inline bool pix_rect_contains(pix_rect r, vec2 p) {
    return p.x >= r.x && p.x < r.x + r.w && p.y >= r.y && p.y < r.y + r.h;
}

// ---- screen shake ----
//
// Trauma-based shake (Squirrel Eiserloh, "Math for Game Programmers: Juicing
// Your Cameras with Math", GDC 2016): hits add trauma 0..1, it drains over
// time, and the shake is trauma squared - so small hits barely nudge and big
// ones slam - driven by smooth noise rather than random jumps.
//
//   static pix_shake shake;                       // zero = sensible defaults
//   if (hit) pix_shake_add(shake, 0.4f);
//   pix_begin_2d(pix_shake_camera(shake, cam, dt));
struct pix_shake {
    float trauma;          // 0..1, what pix_shake_add raises
    float max_offset;      // pixels at full trauma (0 = 20)
    float max_angle;       // degrees at full trauma (0 = 4)
    float decay;           // trauma lost per second (0 = 1.2)
    float frequency;       // how fast it wobbles (0 = 18)
    float time;
    vec2  offset;          // this frame's shake, also usable for a 3D camera
    float angle;           // degrees
};
static inline void pix_shake_add(pix_shake& s, float trauma) { s.trauma = saturatef(s.trauma + trauma); }
// Advances the shake by dt and fills s.offset / s.angle.
static inline void pix_shake_update(pix_shake& s, float dt) {
    float decay = s.decay > 0.0f ? s.decay : 1.2f;
    float freq = s.frequency > 0.0f ? s.frequency : 18.0f;
    float max_offset = s.max_offset > 0.0f ? s.max_offset : 20.0f;
    float max_angle = s.max_angle > 0.0f ? s.max_angle : 4.0f;
    s.time += dt * freq;
    float k = s.trauma * s.trauma;
    // three independent noise streams, recentred from 0..1 to -1..1
    s.offset = v2(max_offset * k * (value_noise2(s.time, 0.5f, 101) * 2.0f - 1.0f),
                  max_offset * k * (value_noise2(s.time, 0.5f, 202) * 2.0f - 1.0f));
    s.angle = max_angle * k * (value_noise2(s.time, 0.5f, 303) * 2.0f - 1.0f);
    s.trauma = maxf(s.trauma - decay * dt, 0.0f);
}
// `cam` shaken by the current trauma (advancing it by dt).
static inline pix_camera2d pix_shake_camera(pix_shake& s, pix_camera2d cam, float dt) {
    pix_shake_update(s, dt);
    cam.offset = v2add(cam.offset, s.offset);
    cam.rotation += s.angle;
    return cam;
}

#endif // PIX_DRAW2D_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_DRAW2D_IMPL)
#define PIX_DRAW2D_IMPL
#include <math.h>
#include <stdio.h>

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
out vec3 vParams;
flat out int vSlot;
void main() {
    vUV = aUV; vColor = aColor; vOutline = aOutline; vParams = aParams.xyz;
    vSlot = int(aParams.w * 255.0 + 0.5);
    gl_Position = uProj * vec4(aPos, 0.0, 1.0);
}
)";

// The slot is flat per triangle, so a 2x2 pixel quad never straddles two
// slots in practice; gradients are still taken outside the switch, where
// control flow is uniform, so mipmapped textures sample correctly.
static const char* const FSHDER_2D = R"(#version 440 core
in vec2 vUV;
in vec4 vColor;
in vec4 vOutline;
in vec3 vParams;
flat in int vSlot;
out vec4 frag;
uniform sampler2D uTex[16];
vec4 slot_sample(vec2 uv, vec2 dx, vec2 dy) {
    switch (vSlot) {
        case 0:  return textureGrad(uTex[0], uv, dx, dy);
        case 1:  return textureGrad(uTex[1], uv, dx, dy);
        case 2:  return textureGrad(uTex[2], uv, dx, dy);
        case 3:  return textureGrad(uTex[3], uv, dx, dy);
        case 4:  return textureGrad(uTex[4], uv, dx, dy);
        case 5:  return textureGrad(uTex[5], uv, dx, dy);
        case 6:  return textureGrad(uTex[6], uv, dx, dy);
        case 7:  return textureGrad(uTex[7], uv, dx, dy);
        case 8:  return textureGrad(uTex[8], uv, dx, dy);
        case 9:  return textureGrad(uTex[9], uv, dx, dy);
        case 10: return textureGrad(uTex[10], uv, dx, dy);
        case 11: return textureGrad(uTex[11], uv, dx, dy);
        case 12: return textureGrad(uTex[12], uv, dx, dy);
        case 13: return textureGrad(uTex[13], uv, dx, dy);
        case 14: return textureGrad(uTex[14], uv, dx, dy);
        default: return textureGrad(uTex[15], uv, dx, dy);
    }
}
void main() {
    vec4 t = slot_sample(vUV, dFdx(vUV), dFdy(vUV));
    float w = max(fwidth(t.r), 1e-4);
    if (vParams.x < 0.5) { frag = t * vColor; return; }
    // SDF glyph: 0.5 is the outline; fwidth keeps the edge one pixel wide at any scale
    float sdf = t.r;
    float fill_edge = 0.5 - (vParams.y - 0.5);
    float outline_edge = fill_edge - vParams.z * 0.5;
    float fill = smoothstep(fill_edge - w, fill_edge + w, sdf);
    float outer = smoothstep(outline_edge - w, outline_edge + w, sdf);
    float k = outer > 1e-4 ? clamp(fill / outer, 0.0, 1.0) : 0.0;
    vec4 c = mix(vOutline, vColor, k);
    frag = vec4(c.rgb, c.a * outer);
}
)";

void pix_batch2d_create(pix_batch2d& b, size_t capacity) {
    b = pix_batch2d();
    if (capacity > 65536) capacity = 65536;
    b.capacity = capacity < 64 ? 64 : capacity;
    b.index_capacity = b.capacity * 3;        // a fan or strip needs up to 3 indices per vertex
    b.verts = pix_new<pix__vertex2d>(b.capacity);
    b.indices = pix_new<uint16_t>(b.index_capacity);
    b.program = opengl_create_shader(VSHDER_2D, FSHDER_2D);
    b.u_proj = glGetUniformLocation(b.program, "uProj");
    glUseProgram(b.program);
    for (int i = 0; i < PIX_BATCH2D_SLOTS; i++) {
        char name[16];
        snprintf(name, sizeof(name), "uTex[%d]", i);
        glUniform1i(glGetUniformLocation(b.program, name), i);
    }
    glUseProgram(0);
    unsigned char white[4] = { 255, 255, 255, 255 };
    b.white = opengl_create_texture2d(1, 1, 4, white, TEXTURE_PIXELATED);
    b.slots[0] = b.white;
    b.slot_count = 1;
    b.blend = PIX_BLEND_ALPHA;

    glGenVertexArrays(1, &b.vao);
    glGenBuffers(1, &b.vbo);
    glGenBuffers(1, &b.ebo);
    glBindVertexArray(b.vao);
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(b.capacity * sizeof(pix__vertex2d)), 0, GL_STREAM_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, b.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(b.index_capacity * sizeof(uint16_t)), 0, GL_STREAM_DRAW);
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
    if (b.ebo) glDeleteBuffers(1, &b.ebo);
    if (b.vao) glDeleteVertexArrays(1, &b.vao);
    opengl_destroy_shader(b.program);
    opengl_destroy_texture(b.white);
    pix_free(b.verts);
    pix_free(b.indices);
    b = pix_batch2d();
}

static void pix__2d_apply_blend(int blend) {
    switch (blend) {
        case PIX_BLEND_ADDITIVE:      glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE, GL_ZERO, GL_ONE); break;
        case PIX_BLEND_MULTIPLY:      glBlendFuncSeparate(GL_DST_COLOR, GL_ONE_MINUS_SRC_ALPHA, GL_ZERO, GL_ONE); break;
        case PIX_BLEND_PREMULTIPLIED: glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); break;
        default: glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA); break;
    }
}

void pix_batch2d_flush(pix_batch2d& b) {
    if (b.index_count) {
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glEnable(GL_BLEND);
        pix__2d_apply_blend(b.blend);
        glUseProgram(b.program);
        glUniformMatrix4fv(b.u_proj, 1, GL_FALSE, b.transform.data);
        for (int i = 0; i < b.slot_count; i++) {
            glActiveTexture(GL_TEXTURE0 + i);
            glBindTexture(GL_TEXTURE_2D, b.slots[i]);
        }
        glActiveTexture(GL_TEXTURE0);
        glBindVertexArray(b.vao);
        // fresh storage of exactly the size used, filled in the same call: the
        // driver never waits for the previous flush's draw, and a frame with
        // many small flushes does not re-allocate the whole buffer each time
        glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(b.count * sizeof(pix__vertex2d)), b.verts, GL_STREAM_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, b.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(b.index_count * sizeof(uint16_t)), b.indices, GL_STREAM_DRAW);
        glDrawElements(GL_TRIANGLES, (GLsizei)b.index_count, GL_UNSIGNED_SHORT, 0);
        glBindVertexArray(0);
        b.draws++;
    }
    b.count = 0;
    b.index_count = 0;
    b.slot_count = 1;    // white stays in slot 0
}

void pix_batch2d_begin(pix_batch2d& b, int width, int height, bool top_down) {
    pix_batch2d_flush(b);
    b.width = width;
    b.height = height;
    b.top_down = top_down;
    // a top-down target gets its first row at the bottom of clip space, where GL stores row 0
    b.projection = top_down ? mat4_ortho(0.0f, (float)width, 0.0f, (float)height, -1.0f, 1.0f)
                            : mat4_ortho(0.0f, (float)width, (float)height, 0.0f, -1.0f, 1.0f);
    b.transform = b.projection;
    b.has_camera = false;
    b.draws = 0;
    b.blend = PIX_BLEND_ALPHA;
    if (b.scissor) { glDisable(GL_SCISSOR_TEST); b.scissor = false; }
}

void pix_batch2d_set_camera(pix_batch2d& b, const pix_camera2d* cam) {
    pix_batch2d_flush(b);
    b.has_camera = cam != 0;
    if (!cam) { b.transform = b.projection; return; }
    b.camera = *cam;
    float zoom = cam->zoom != 0.0f ? cam->zoom : 1.0f;
    // screen = offset + rotate(zoom * (world - target))
    mat4 m = mat4_translate(cam->offset.x, cam->offset.y, 0.0f);
    m = mat4_mul(m, mat4_rotate_z(cam->rotation * PIX_DEG2RAD));
    m = mat4_mul(m, mat4_scale(zoom, zoom, 1.0f));
    m = mat4_mul(m, mat4_translate(-cam->target.x, -cam->target.y, 0.0f));
    b.transform = mat4_mul(b.projection, m);
}

pix_rect pix_batch2d_visible(const pix_batch2d& b) {
    // clip space -1..1 back through the inverse transform
    mat4 inv = mat4_inverse(b.transform);
    vec3 c[4] = { mat4_mul_point(inv, v3(-1, -1, 0)), mat4_mul_point(inv, v3(1, -1, 0)),
                  mat4_mul_point(inv, v3(1, 1, 0)), mat4_mul_point(inv, v3(-1, 1, 0)) };
    float x0 = c[0].x, x1 = c[0].x, y0 = c[0].y, y1 = c[0].y;
    for (int i = 1; i < 4; i++) {
        x0 = minf(x0, c[i].x); x1 = maxf(x1, c[i].x);
        y0 = minf(y0, c[i].y); y1 = maxf(y1, c[i].y);
    }
    pix_rect r = { x0, y0, x1 - x0, y1 - y0 };
    return r;
}

void pix_batch2d_scissor(pix_batch2d& b, pix_rect r) {
    pix_batch2d_flush(b);
    if (r.w <= 0.0f || r.h <= 0.0f) {
        glDisable(GL_SCISSOR_TEST);
        b.scissor = false;
        return;
    }
    glEnable(GL_SCISSOR_TEST);
    // GL's scissor origin is the target's first stored row
    float y = b.top_down ? r.y : (float)b.height - r.y - r.h;
    glScissor((GLint)r.x, (GLint)y, (GLsizei)r.w, (GLsizei)r.h);
    b.scissor = true;
}

void pix_batch2d_set_blend(pix_batch2d& b, int blend) {
    if (blend == b.blend) return;
    pix_batch2d_flush(b);
    b.blend = blend;
}

// ---- reserving geometry ----

struct pix__2d_span {
    pix__vertex2d* v;      // nv vertices to fill
    uint16_t*      i;      // ni indices to fill, relative to base already
    uint16_t       base;   // index of v[0]
    uint8_t        slot;
};

// The slot `texture` (0 = white) occupies, binding it if new; flushes when
// all slots are taken by others.
static uint8_t pix__2d_slot(pix_batch2d& b, idx texture) {
    if (!texture || texture == b.white) return 0;
    for (int s = 1; s < b.slot_count; s++) if (b.slots[s] == texture) return (uint8_t)s;
    if (b.slot_count == PIX_BATCH2D_SLOTS) pix_batch2d_flush(b);
    b.slots[b.slot_count] = texture;
    return (uint8_t)b.slot_count++;
}

// Room for nv vertices and ni indices; flushes first if they would not fit.
static bool pix__2d_reserve(pix_batch2d& b, size_t nv, size_t ni, idx texture, pix__2d_span* out) {
    if (nv > b.capacity || ni > b.index_capacity) return false;
    if (b.count + nv > b.capacity || b.index_count + ni > b.index_capacity) pix_batch2d_flush(b);
    out->slot = pix__2d_slot(b, texture);    // after the flush above, so the slot survives
    out->base = (uint16_t)b.count;
    out->v = b.verts + b.count;
    out->i = b.indices + b.index_count;
    b.count += nv;
    b.index_count += ni;
    return true;
}

static inline void pix__2d_vert(pix__vertex2d& v, float x, float y, float u, float t, pix_color c, uint8_t slot) {
    v.x = x; v.y = y; v.u = u; v.v = t;
    v.color = c;
    v.outline = PIX_BLANK;
    v.mode = 0; v.weight = 0; v.outline_w = 0;
    v.slot = slot;
}

// two triangles over corners in order (tl, tr, br, bl)
static void pix__2d_quad(pix_batch2d& b, idx tex, const vec2* p, const vec2* uv, const pix_color* c) {
    pix__2d_span s;
    if (!pix__2d_reserve(b, 4, 6, tex, &s)) return;
    for (int k = 0; k < 4; k++) pix__2d_vert(s.v[k], p[k].x, p[k].y, uv[k].x, uv[k].y, c[k], s.slot);
    uint16_t q = s.base;
    s.i[0] = q; s.i[1] = (uint16_t)(q + 1); s.i[2] = (uint16_t)(q + 2);
    s.i[3] = q; s.i[4] = (uint16_t)(q + 2); s.i[5] = (uint16_t)(q + 3);
}

static int pix__circle_segments(float radius) {
    int n = (int)(radius * 0.75f) + 12;   // enough that the edge never looks faceted
    return n > 128 ? 128 : n;
}

// A filled ellipse sector as a triangle fan, or the band between two
// ellipses as a strip. The rim is walked by rotating one vector by a fixed
// step, so the loop has no sin/cos in it.
static void pix__2d_arc(pix_batch2d& b, vec2 c, float rx_in, float ry_in, float rx_out, float ry_out,
                        float a0, float a1, pix_color col) {
    float sweep = a1 - a0;
    if (sweep == 0.0f || rx_out <= 0.0f || ry_out <= 0.0f) return;
    if (fabsf(sweep) > PIX_TAU) sweep = sweep > 0.0f ? PIX_TAU : -PIX_TAU;
    int n = (int)((float)pix__circle_segments(maxf(rx_out, ry_out)) * fabsf(sweep) / PIX_TAU + 0.999f);
    if (n < 2) n = 2;
    float step = sweep / (float)n, cs = cosf(step), sn = sinf(step);
    float dx = cosf(a0), dy = sinf(a0);
    bool fill = rx_in <= 0.0f && ry_in <= 0.0f;
    pix__2d_span s;
    if (fill) {
        if (!pix__2d_reserve(b, (size_t)n + 2, (size_t)n * 3, 0, &s)) return;
        pix__2d_vert(s.v[0], c.x, c.y, 0, 0, col, 0);
        for (int k = 0; k <= n; k++) {
            pix__2d_vert(s.v[k + 1], c.x + dx * rx_out, c.y + dy * ry_out, 0, 0, col, 0);
            float nx = dx * cs - dy * sn; dy = dx * sn + dy * cs; dx = nx;
        }
        for (int k = 0; k < n; k++) {
            s.i[k * 3] = s.base;
            s.i[k * 3 + 1] = (uint16_t)(s.base + 1 + k);
            s.i[k * 3 + 2] = (uint16_t)(s.base + 2 + k);
        }
    } else {
        if (!pix__2d_reserve(b, (size_t)(n + 1) * 2, (size_t)n * 6, 0, &s)) return;
        for (int k = 0; k <= n; k++) {
            pix__2d_vert(s.v[k * 2], c.x + dx * rx_out, c.y + dy * ry_out, 0, 0, col, 0);
            pix__2d_vert(s.v[k * 2 + 1], c.x + dx * rx_in, c.y + dy * ry_in, 0, 0, col, 0);
            float nx = dx * cs - dy * sn; dy = dx * sn + dy * cs; dx = nx;
        }
        for (int k = 0; k < n; k++) {
            uint16_t o = (uint16_t)(s.base + k * 2);
            uint16_t* i = s.i + k * 6;
            i[0] = o; i[1] = (uint16_t)(o + 2); i[2] = (uint16_t)(o + 1);
            i[3] = (uint16_t)(o + 1); i[4] = (uint16_t)(o + 2); i[5] = (uint16_t)(o + 3);
        }
    }
}

// The outline of a rounded rectangle inset by `inset`, as `per_corner + 1`
// points per corner, clockwise from the top-left corner's left end.
static int pix__rounded_outline(pix_rect r, float rad, float inset, int per_corner, vec2* out) {
    vec2 centers[4] = { {r.x + rad, r.y + rad}, {r.x + r.w - rad, r.y + rad},
                        {r.x + r.w - rad, r.y + r.h - rad}, {r.x + rad, r.y + r.h - rad} };
    float rr = maxf(rad - inset, 0.0f);
    float step = (PIX_PI * 0.5f) / (float)per_corner, cs = cosf(step), sn = sinf(step);
    int n = 0;
    for (int k = 0; k < 4; k++) {
        float a = PIX_PI + (float)k * PIX_PI * 0.5f;
        float dx = cosf(a), dy = sinf(a);
        for (int i = 0; i <= per_corner; i++) {
            out[n++] = v2(centers[k].x + dx * rr, centers[k].y + dy * rr);
            float nx = dx * cs - dy * sn; dy = dx * sn + dy * cs; dx = nx;
        }
    }
    return n;
}

// ---- shapes ----

void pix_batch_rect(pix_batch2d& b, pix_rect r, pix_color c) {
    pix_batch_rect_colors(b, r, c, c, c, c);
}

void pix_batch_rect_gradient(pix_batch2d& b, pix_rect r, pix_color top, pix_color bottom) {
    pix_batch_rect_colors(b, r, top, top, bottom, bottom);
}

void pix_batch_rect_colors(pix_batch2d& b, pix_rect r, pix_color tl, pix_color tr, pix_color br, pix_color bl) {
    pix__2d_span s;
    if (!pix__2d_reserve(b, 4, 6, 0, &s)) return;
    pix__2d_vert(s.v[0], r.x, r.y, 0, 0, tl, 0);
    pix__2d_vert(s.v[1], r.x + r.w, r.y, 0, 0, tr, 0);
    pix__2d_vert(s.v[2], r.x + r.w, r.y + r.h, 0, 0, br, 0);
    pix__2d_vert(s.v[3], r.x, r.y + r.h, 0, 0, bl, 0);
    uint16_t q = s.base;
    s.i[0] = q; s.i[1] = (uint16_t)(q + 1); s.i[2] = (uint16_t)(q + 2);
    s.i[3] = q; s.i[4] = (uint16_t)(q + 2); s.i[5] = (uint16_t)(q + 3);
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

// a frame as one strip of eight corners: outer then inner
void pix_batch_rect_lines(pix_batch2d& b, pix_rect r, float t, pix_color c) {
    t = minf(t, minf(r.w, r.h) * 0.5f);
    pix__2d_span s;
    if (!pix__2d_reserve(b, 8, 24, 0, &s)) return;
    float x0 = r.x, y0 = r.y, x1 = r.x + r.w, y1 = r.y + r.h;
    vec2 p[8] = { {x0, y0}, {x1, y0}, {x1, y1}, {x0, y1},
                  {x0 + t, y0 + t}, {x1 - t, y0 + t}, {x1 - t, y1 - t}, {x0 + t, y1 - t} };
    for (int k = 0; k < 8; k++) pix__2d_vert(s.v[k], p[k].x, p[k].y, 0, 0, c, 0);
    for (int k = 0; k < 4; k++) {
        uint16_t o0 = (uint16_t)(s.base + k), o1 = (uint16_t)(s.base + (k + 1) % 4);
        uint16_t i0 = (uint16_t)(o0 + 4), i1 = (uint16_t)(o1 + 4);
        uint16_t* i = s.i + k * 6;
        i[0] = o0; i[1] = o1; i[2] = i1; i[3] = o0; i[4] = i1; i[5] = i0;
    }
}

void pix_batch_triangle(pix_batch2d& b, vec2 p0, vec2 p1, vec2 p2, pix_color c) {
    pix__2d_span s;
    if (!pix__2d_reserve(b, 3, 3, 0, &s)) return;
    pix__2d_vert(s.v[0], p0.x, p0.y, 0, 0, c, 0);
    pix__2d_vert(s.v[1], p1.x, p1.y, 0, 0, c, 0);
    pix__2d_vert(s.v[2], p2.x, p2.y, 0, 0, c, 0);
    s.i[0] = s.base; s.i[1] = (uint16_t)(s.base + 1); s.i[2] = (uint16_t)(s.base + 2);
}

void pix_batch_polygon(pix_batch2d& b, const vec2* pts, int count, pix_color c) {
    if (count < 3) return;
    pix__2d_span s;
    if (!pix__2d_reserve(b, (size_t)count, (size_t)(count - 2) * 3, 0, &s)) return;
    for (int k = 0; k < count; k++) pix__2d_vert(s.v[k], pts[k].x, pts[k].y, 0, 0, c, 0);
    for (int k = 0; k < count - 2; k++) {
        s.i[k * 3] = s.base;
        s.i[k * 3 + 1] = (uint16_t)(s.base + k + 1);
        s.i[k * 3 + 2] = (uint16_t)(s.base + k + 2);
    }
}

void pix_batch_polyline(pix_batch2d& b, const vec2* pts, int count, float thickness, pix_color c, bool closed) {
    if (count < 2) return;
    if (count == 2 && !closed) { pix_batch_line(b, pts[0], pts[1], thickness, c); return; }
    int segs = closed ? count : count - 1;
    pix__2d_span s;
    if (!pix__2d_reserve(b, (size_t)count * 2 + (closed ? 2 : 0), (size_t)segs * 6, 0, &s)) return;
    float h = thickness * 0.5f;
    for (int k = 0; k < count; k++) {
        bool has_prev = closed || k > 0, has_next = closed || k < count - 1;
        vec2 p = pts[k];
        vec2 dp = has_prev ? v2norm(v2sub(p, pts[(k + count - 1) % count])) : v2(0, 0);
        vec2 dn = has_next ? v2norm(v2sub(pts[(k + 1) % count], p)) : v2(0, 0);
        if (!has_prev) dp = dn;
        if (!has_next) dn = dp;
        // the miter: the average of the two segment normals, lengthened so
        // both edges keep their thickness - capped at 4x the half thickness,
        // where a sharp corner would send it off towards infinity
        vec2 n0 = v2(-dp.y, dp.x), n1 = v2(-dn.y, dn.x);
        vec2 m = v2norm(v2add(n0, n1));
        if (v2dot(m, m) < 1e-12f) m = n0;
        float d = v2dot(m, n1);
        float len = h / maxf(d, 0.25f);
        vec2 o = v2scale(m, len);
        pix__2d_vert(s.v[k * 2], p.x + o.x, p.y + o.y, 0, 0, c, 0);
        pix__2d_vert(s.v[k * 2 + 1], p.x - o.x, p.y - o.y, 0, 0, c, 0);
    }
    if (closed) { s.v[count * 2] = s.v[0]; s.v[count * 2 + 1] = s.v[1]; }
    for (int k = 0; k < segs; k++) {
        uint16_t o = (uint16_t)(s.base + k * 2);
        uint16_t* i = s.i + k * 6;
        i[0] = o; i[1] = (uint16_t)(o + 2); i[2] = (uint16_t)(o + 1);
        i[3] = (uint16_t)(o + 1); i[4] = (uint16_t)(o + 2); i[5] = (uint16_t)(o + 3);
    }
}

void pix_batch_circle(pix_batch2d& b, vec2 ctr, float radius, pix_color c) {
    pix__2d_arc(b, ctr, 0, 0, radius, radius, 0.0f, PIX_TAU, c);
}

void pix_batch_circle_lines(pix_batch2d& b, vec2 ctr, float radius, float t, pix_color c) {
    float ri = maxf(radius - t * 0.5f, 0.0f), ro = radius + t * 0.5f;
    pix__2d_arc(b, ctr, ri > 0.0f ? ri : 1e-4f, ri > 0.0f ? ri : 1e-4f, ro, ro, 0.0f, PIX_TAU, c);
}

void pix_batch_ellipse(pix_batch2d& b, vec2 ctr, float rx, float ry, pix_color c) {
    pix__2d_arc(b, ctr, 0, 0, rx, ry, 0.0f, PIX_TAU, c);
}

void pix_batch_ellipse_lines(pix_batch2d& b, vec2 ctr, float rx, float ry, float t, pix_color c) {
    float h = t * 0.5f;
    pix__2d_arc(b, ctr, maxf(rx - h, 1e-4f), maxf(ry - h, 1e-4f), rx + h, ry + h, 0.0f, PIX_TAU, c);
}

void pix_batch_ring(pix_batch2d& b, vec2 ctr, float inner, float outer, float start_deg, float end_deg, pix_color c) {
    pix__2d_arc(b, ctr, inner, inner, outer, outer, start_deg * PIX_DEG2RAD, end_deg * PIX_DEG2RAD, c);
}

// one convex fan around the centre, the four corner arcs as its rim
void pix_batch_rect_rounded(pix_batch2d& b, pix_rect r, float radius, pix_color c) {
    float rad = minf(radius, minf(r.w, r.h) * 0.5f);
    if (rad <= 0.5f) { pix_batch_rect(b, r, c); return; }
    int per = pix__circle_segments(rad) / 4 + 2;
    vec2 rim[4 * 36];
    if (per > 35) per = 35;
    int n = pix__rounded_outline(r, rad, 0.0f, per, rim);
    pix__2d_span s;
    if (!pix__2d_reserve(b, (size_t)n + 1, (size_t)n * 3, 0, &s)) return;
    pix__2d_vert(s.v[0], r.x + r.w * 0.5f, r.y + r.h * 0.5f, 0, 0, c, 0);
    for (int k = 0; k < n; k++) pix__2d_vert(s.v[k + 1], rim[k].x, rim[k].y, 0, 0, c, 0);
    for (int k = 0; k < n; k++) {
        s.i[k * 3] = s.base;
        s.i[k * 3 + 1] = (uint16_t)(s.base + 1 + k);
        s.i[k * 3 + 2] = (uint16_t)(s.base + 1 + (k + 1) % n);
    }
}

void pix_batch_rect_rounded_lines(pix_batch2d& b, pix_rect r, float radius, float t, pix_color c) {
    float rad = minf(radius, minf(r.w, r.h) * 0.5f);
    if (rad <= 0.5f) { pix_batch_rect_lines(b, r, t, c); return; }
    t = minf(t, minf(r.w, r.h) * 0.5f);
    int per = pix__circle_segments(rad) / 4 + 2;
    if (per > 35) per = 35;
    vec2 outer[4 * 36], inner[4 * 36];
    int n = pix__rounded_outline(r, rad, 0.0f, per, outer);
    // the inner edge: the same corner centres, the radius shrunk by t; where
    // t passes the radius the inner corner goes square, which is correct
    pix_rect in = { r.x + t, r.y + t, r.w - 2 * t, r.h - 2 * t };
    float rin = maxf(rad - t, 0.0f);
    if (rin > 0.0f) pix__rounded_outline(in, rin, 0.0f, per, inner);
    else for (int k = 0; k < 4; k++) {
        vec2 corner = k == 0 ? v2(in.x, in.y) : k == 1 ? v2(in.x + in.w, in.y)
                    : k == 2 ? v2(in.x + in.w, in.y + in.h) : v2(in.x, in.y + in.h);
        for (int i = 0; i <= per; i++) inner[k * (per + 1) + i] = corner;
    }
    pix__2d_span s;
    if (!pix__2d_reserve(b, (size_t)n * 2, (size_t)n * 6, 0, &s)) return;
    for (int k = 0; k < n; k++) {
        pix__2d_vert(s.v[k * 2], outer[k].x, outer[k].y, 0, 0, c, 0);
        pix__2d_vert(s.v[k * 2 + 1], inner[k].x, inner[k].y, 0, 0, c, 0);
    }
    for (int k = 0; k < n; k++) {
        uint16_t o = (uint16_t)(s.base + k * 2), p = (uint16_t)(s.base + ((k + 1) % n) * 2);
        uint16_t* i = s.i + k * 6;
        i[0] = o; i[1] = p; i[2] = (uint16_t)(o + 1);
        i[3] = (uint16_t)(o + 1); i[4] = p; i[5] = (uint16_t)(p + 1);
    }
}

// ---- textures ----

void pix_batch_texture_pro(pix_batch2d& b, const pix_texture& t, pix_rect src, pix_rect dst,
                           vec2 origin, float rotation_deg, pix_color tint) {
    if (!t.id || t.width <= 0 || t.height <= 0) return;
    float iw = 1.0f / (float)t.width, ih = 1.0f / (float)t.height;
    float u0 = src.x * iw, v0 = src.y * ih;
    float u1 = (src.x + src.w) * iw, v1 = (src.y + src.h) * ih;
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

void pix_batch_quad(pix_batch2d& b, idx tex, const vec2 corners[4], const vec2 uvs[4], pix_color tint) {
    pix_color cs[4] = { tint, tint, tint, tint };
    pix__2d_quad(b, tex, corners, uvs, cs);
}

// a 4x4 grid of vertices, nine quads, one reservation
void pix_batch_nine_slice(pix_batch2d& b, const pix_texture& t, pix_rect src, vec4 border, pix_rect dst, pix_color tint) {
    if (!t.id || t.width <= 0 || t.height <= 0) return;
    float l = border.x, tp = border.y, r = border.z, bt = border.w;
    // borders keep their size unless the panel is too small for them
    float kx = (l + r) > dst.w && (l + r) > 0.0f ? dst.w / (l + r) : 1.0f;
    float ky = (tp + bt) > dst.h && (tp + bt) > 0.0f ? dst.h / (tp + bt) : 1.0f;
    float xs[4] = { dst.x, dst.x + l * kx, dst.x + dst.w - r * kx, dst.x + dst.w };
    float ys[4] = { dst.y, dst.y + tp * ky, dst.y + dst.h - bt * ky, dst.y + dst.h };
    float iw = 1.0f / (float)t.width, ih = 1.0f / (float)t.height;
    float us[4] = { src.x * iw, (src.x + l) * iw, (src.x + src.w - r) * iw, (src.x + src.w) * iw };
    float vs[4] = { src.y * ih, (src.y + tp) * ih, (src.y + src.h - bt) * ih, (src.y + src.h) * ih };
    pix__2d_span s;
    if (!pix__2d_reserve(b, 16, 54, t.id, &s)) return;
    for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++) pix__2d_vert(s.v[y * 4 + x], xs[x], ys[y], us[x], vs[y], tint, s.slot);
    int n = 0;
    for (int y = 0; y < 3; y++)
        for (int x = 0; x < 3; x++) {
            uint16_t q = (uint16_t)(s.base + y * 4 + x);
            s.i[n++] = q; s.i[n++] = (uint16_t)(q + 1); s.i[n++] = (uint16_t)(q + 5);
            s.i[n++] = q; s.i[n++] = (uint16_t)(q + 5); s.i[n++] = (uint16_t)(q + 4);
        }
}

// ---- parallax ----

void pix_batch_parallax(pix_batch2d& b, const pix_texture& t, vec2 factor, vec2 offset, float scale,
                        bool repeat_x, bool repeat_y, pix_color tint) {
    if (!t.id || t.width <= 0 || t.height <= 0 || scale <= 0.0f) return;
    // the layer's origin trails the camera: at factor 1 it is fixed in the
    // world, at factor 0 it rides along with the camera
    vec2 target = b.has_camera ? b.camera.target : v2(0, 0);
    vec2 origin = v2(offset.x + target.x * (1.0f - factor.x), offset.y + target.y * (1.0f - factor.y));
    float sw = (float)t.width * scale, sh = (float)t.height * scale;
    pix_rect view = pix_batch2d_visible(b);
    float x0 = repeat_x ? view.x : origin.x, x1 = repeat_x ? view.x + view.w : origin.x + sw;
    float y0 = repeat_y ? view.y : origin.y, y1 = repeat_y ? view.y + view.h : origin.y + sh;
    if (x1 <= x0 || y1 <= y0) return;
    float u0 = (x0 - origin.x) / sw, u1 = (x1 - origin.x) / sw;
    float v0 = (y0 - origin.y) / sh, v1 = (y1 - origin.y) / sh;
    vec2 p[4] = { {x0, y0}, {x1, y0}, {x1, y1}, {x0, y1} };
    vec2 uv[4] = { {u0, v0}, {u1, v0}, {u1, v1}, {u0, v1} };
    pix_batch_quad(b, t.id, p, uv, tint);
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
            pix__2d_span s;
            if (!pix__2d_reserve(b, 4, 6, font.atlas_texture, &s)) return;
            float xs[4] = { x0, x1, x1, x0 }, ys[4] = { y0, y0, y1, y1 };
            float us[4] = { u0, u1, u1, u0 }, vs[4] = { v0, v0, v1, v1 };
            for (int i = 0; i < 4; i++) {
                pix__2d_vert(s.v[i], xs[i], ys[i], us[i], vs[i], color, s.slot);
                s.v[i].outline = st.outline_color;
                s.v[i].mode = 255;
                s.v[i].weight = weight;
                s.v[i].outline_w = outline;
            }
            uint16_t q = s.base;
            s.i[0] = q; s.i[1] = (uint16_t)(q + 1); s.i[2] = (uint16_t)(q + 2);
            s.i[3] = q; s.i[4] = (uint16_t)(q + 2); s.i[5] = (uint16_t)(q + 3);
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
