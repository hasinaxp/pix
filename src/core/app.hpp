// pix - the easy layer. One window, one global context, short names:
//
//   #define PIX_IMPLEMENTATION
//   #include "pix.h"
//
//   int main() {
//       pix_init("my game", 1280, 720);
//       while (pix_frame()) {
//           pix_clear(PIX_SKYBLUE);
//           pix_draw_rect(100, 100, 200, 80, PIX_RED);
//           pix_draw_text("hello pix", 110, 120, 32, PIX_WHITE);
//       }
//       pix_shutdown();
//   }
//
// Everything here is a thin wrapper over the modules underneath (platform,
// renderer, draw2d, sound, loaders), which stay fully usable directly -
// pix_get_window(), pix_get_renderer() and friends hand them out.
//
// Frame order: 3D first (pix_begin_3d .. pix_end_3d renders the whole screen),
// then 2D on top (HUD, menus). 2D drawn before pix_end_3d is covered by it.
#ifndef PIX_APP_HPP
#define PIX_APP_HPP
#include "dtype.hpp"
#include "math.hpp"
#include "platform.hpp"
#include "opengl_api.hpp"
#include "texture.hpp"
#include "renderer.hpp"
#include "draw2d.hpp"
#include "font.hpp"
#include "sound.hpp"
#include "animation.hpp"
#include "jobs.hpp"
#include "../loader/data_loader.hpp"

// extra pix_init flags, alongside the PIX_WINDOW_* ones
#define PIX_NO_3D_EFFECTS 0x100    // skip shadows + bloom when the 3D renderer starts
#define PIX_NO_AUDIO      0x200    // never open the audio device

#define PIX_MOUSE_LEFT   MOUSE_BUTTON_LEFT
#define PIX_MOUSE_RIGHT  MOUSE_BUTTON_RIGHT
#define PIX_MOUSE_MIDDLE MOUSE_BUTTON_MID

// A loaded model: its GPU mesh, a material made from its texture, and for
// rigged models the skeleton and clips an animator needs.
struct pix_model {
    idx  mesh;
    idx  material;
    bool skinned;
    skeleton_file_data*       skeleton;    // skinned only
    animation_clip_file_data* clips;       // skinned only, clip_count long
    size_t                    clip_count;
    vec3 bounds_min;
    vec3 bounds_max;
};

// ---- lifetime and frame ----
PIXDEF bool pix_init(const char* title, int width, int height,
                     unsigned int flags = PIX_WINDOW_RESIZABLE | PIX_WINDOW_VSYNC);
PIXDEF void pix_shutdown(void);
// Presents the previous frame, pumps input, advances the clock. Returns false
// once the window was closed or pix_quit was called.
PIXDEF bool pix_frame(void);
PIXDEF void pix_quit(void);

PIXDEF float  pix_dt(void);          // seconds since the previous frame (capped at 0.1)
PIXDEF double pix_time(void);        // seconds since pix_init
PIXDEF int    pix_fps(void);         // smoothed
PIXDEF int    pix_width(void);       // window client size in pixels
PIXDEF int    pix_height(void);

PIXDEF void pix_set_title(const char* title);
PIXDEF void pix_set_vsync_enabled(bool on);
PIXDEF void pix_toggle_fullscreen(void);
// Hides the cursor and keeps it centred; mouse_delta keeps reporting motion.
PIXDEF void pix_set_mouse_locked(bool locked);

// ---- input ----
PIXDEF bool pix_key_down(int key);        // held
PIXDEF bool pix_key_pressed(int key);     // went down this frame
PIXDEF bool pix_key_released(int key);    // went up this frame
PIXDEF bool pix_mouse_down(int button);
PIXDEF bool pix_mouse_pressed(int button);
PIXDEF bool pix_mouse_released(int button);
PIXDEF vec2 pix_mouse_position(void);
PIXDEF vec2 pix_mouse_delta(void);
PIXDEF int  pix_mouse_wheel(void);
PIXDEF const pix_gamepad& pix_get_gamepad(int index);
// codepoints typed this frame (for text fields); *count receives how many
PIXDEF const unsigned int* pix_text_input(int* count);
// WASD/arrows as a -1..1 vector, falling back to gamepad 0's left stick
PIXDEF vec2 pix_input_axis(void);

// ---- 2D ----
PIXDEF void pix_clear(pix_color color);
PIXDEF void pix_begin_2d(const pix_camera2d& camera);
PIXDEF void pix_end_2d(void);
PIXDEF void pix_draw_rect(float x, float y, float w, float h, pix_color color);
PIXDEF void pix_draw_rect(pix_rect r, pix_color color);
PIXDEF void pix_draw_rect_lines(pix_rect r, float thickness, pix_color color);
PIXDEF void pix_draw_rect_rounded(pix_rect r, float radius, pix_color color);
PIXDEF void pix_draw_rect_gradient(pix_rect r, pix_color top, pix_color bottom);
PIXDEF void pix_draw_line(vec2 a, vec2 b, float thickness, pix_color color);
PIXDEF void pix_draw_circle(vec2 center, float radius, pix_color color);
PIXDEF void pix_draw_circle_lines(vec2 center, float radius, float thickness, pix_color color);
PIXDEF void pix_draw_triangle(vec2 a, vec2 b, vec2 c, pix_color color);
PIXDEF void pix_draw_texture(const pix_texture& t, float x, float y, pix_color tint = PIX_WHITE);
PIXDEF void pix_draw_texture_pro(const pix_texture& t, pix_rect src, pix_rect dst, vec2 origin,
                                 float rotation_deg, pix_color tint = PIX_WHITE);
// default font (a system TrueType face) unless one is passed
PIXDEF void pix_draw_text(const char* text, float x, float y, float size, pix_color color);
PIXDEF void pix_draw_text_font(font_data& font, const char* text, float x, float y, float size, pix_color color);
PIXDEF void pix_draw_textf(float x, float y, float size, pix_color color, const char* fmt, ...);
PIXDEF vec2 pix_measure_text(const char* text, float size);
PIXDEF void pix_draw_fps(float x, float y);
// Clip 2D drawing to a rectangle; pix_end_clip turns it off.
PIXDEF void pix_begin_clip(pix_rect r);
PIXDEF void pix_end_clip(void);

// ---- 3D ----
PIXDEF void pix_begin_3d(camera& cam);
// Renders the default pipeline (shadows, reflections, main view, post) to the window.
PIXDEF void pix_end_3d(void);
// For your own composition instead: after pix_begin_3d and the draws, call the
// renderer's stages (pix_render_shadows / _reflections / _view, pix_post_process,
// pix_fullscreen_pass - see renderer.hpp) and then this, instead of pix_end_3d.
PIXDEF void pix_finish_3d(void);
// Draws a render target's colour in 2D (e.g. a minimap or a security camera).
PIXDEF void pix_draw_target(const framebuffer& target, pix_rect dst, pix_color tint = PIX_WHITE);
PIXDEF void pix_draw_mesh(idx mesh, idx material, const mat4& transform, pix_color tint = PIX_WHITE);
PIXDEF void pix_draw_model(const pix_model& model, const mat4& transform, pix_color tint = PIX_WHITE);
// `pose` (animator.pose) must stay alive until pix_end_3d
PIXDEF void pix_draw_model_animated(const pix_model& model, const animation& pose, const mat4& transform,
                                    pix_color tint = PIX_WHITE);
PIXDEF void pix_draw_line3d(vec3 a, vec3 b, pix_color color);
PIXDEF void pix_draw_grid(float size, int cells, pix_color color = PIX_GRAY);
PIXDEF void pix_draw_bounds(vec3 min, vec3 max, pix_color color);
PIXDEF void pix_add_light(const pix_light& light);

// ---- look: shadows, post processing, time of day ----
// Plain settings you edit in place; changes apply at the next frame and only
// reallocate what they have to. e.g.
//   pix_shadows().resolution = 4096;   pix_shadows().softness = 2;
//   pix_post().bloom_strength = 0.6f;  pix_post().tonemap = PIX_TONEMAP_REINHARD;
PIXDEF pix_shadow_settings& pix_shadows(void);
PIXDEF pix_post_settings&   pix_post(void);
PIXDEF void pix_set_quality(int quality);            // PIX_QUALITY_LOW .. PIX_QUALITY_ULTRA
// A fullscreen effect after tonemapping (PIX_EFFECT_* or your own main()).
// Returns a handle for pix_effect_enable / pix_effect_params, -1 on failure.
PIXDEF int  pix_add_effect(const char* body, vec4 params = { 1.0f, 0.0f, 0.0f, 0.0f });
PIXDEF void pix_effect_enable(int effect, bool enabled);
PIXDEF void pix_effect_params(int effect, vec4 params);
PIXDEF void pix_time_of_day(float hour);

// ---- assets ----
// .obj (with its .mtl texture/colours) or .gltf/.glb (static or rigged)
PIXDEF pix_model pix_load_model(const char* path);
// an animator with every clip the model shipped with, playing clip 0
PIXDEF animator pix_model_animator(const pix_model& model);
PIXDEF idx pix_make_material(pix_color color, float metallic = 0.0f, float roughness = 0.6f);
PIXDEF idx pix_make_material_texture(const pix_texture& texture, pix_color color = PIX_WHITE,
                                     float metallic = 0.0f, float roughness = 0.6f);
PIXDEF idx pix_gen_cube(vec3 size);
PIXDEF idx pix_gen_sphere(float radius, int rings = 16, int segments = 32);
PIXDEF idx pix_gen_plane(float width, float depth, float uv_scale = 1.0f);
PIXDEF idx pix_gen_cylinder(float radius, float height);
// heap allocated; free with pix_unload_font
PIXDEF font_data* pix_load_font(const char* ttf_path);
PIXDEF void pix_unload_font(font_data* font);
PIXDEF font_data& pix_default_font(void);

// ---- audio ----
PIXDEF idx  pix_load_sound(const char* wav_path);          // PIX_INVALID on failure
PIXDEF idx  pix_play_sound(idx sound, float volume = 1.0f, float pitch = 1.0f, bool loop = false,
                           float pan = 0.0f);                // returns a voice handle
PIXDEF void pix_stop_sound(idx voice);
PIXDEF void pix_set_volume(float master);

// ---- the modules underneath ----
PIXDEF pix_window&      pix_get_window(void);
PIXDEF pix_renderer&    pix_get_renderer(void);   // created on first use
PIXDEF pix_batch2d&     pix_get_batch2d(void);
PIXDEF pix_audio&       pix_get_audio(void);      // opened on first use
PIXDEF pix_data_loader& pix_get_loader(void);
// a worker pool sized to the machine, started on first use
PIXDEF pix_jobs&        pix_get_jobs(void);
// a crowd animation system (4096 actors) on that pool; you call pix_anim_update
PIXDEF pix_anim_system& pix_get_animations(void);
// draw a rigged model with any bone source (e.g. pix_anim_bones)
PIXDEF void pix_draw_model_bones(const pix_model& model, const mat4* bones, int bone_count,
                                 const mat4& transform, pix_color tint = PIX_WHITE);

#endif // PIX_APP_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_APP_IMPL)
#define PIX_APP_IMPL
#include <stdio.h>
#include <stdarg.h>

struct pix__context {
    pix_window      window;
    unsigned int    flags;
    pix_renderer    renderer;
    bool            renderer_ready;
    pix_batch2d     batch;
    pix_audio       audio;       // lives inside the heap context: its thread holds its address
    bool            audio_ready;
    pix_data_loader loader;
    bool            loader_ready;
    font_data       font;
    bool            font_ready;
    obj_palette     palette;
    pix_jobs*       jobs;          // constructed with new: it owns threads and mutexes
    pix_anim_system animations;
    bool            animations_ready;

    double start, now, last;
    float  dt;
    float  fps;
    bool   quit;
    bool   first_frame;
    bool   in_3d;
};

static pix__context* pix__ctx;

bool pix_init(const char* title, int width, int height, unsigned int flags) {
    if (pix__ctx) return true;
    pix__ctx = pix_alloc<pix__context>(1);   // heap: the context is large and must not move
    if (!pix__ctx) return false;
    pix__context& c = *pix__ctx;
    c.flags = flags;
    c.window = pix_create_window(title, width, height, flags & 0xFF);
    if (c.window.should_close || !c.window.gl_context) { pix_log("pix: window creation failed"); return false; }
    if (!opengl_load_functions()) {
        MessageBoxA(0, "Failed to load the OpenGL 4.4 functions pix needs.", "pix", MB_OK | MB_ICONERROR);
        return false;
    }
    if (flags & PIX_WINDOW_DEBUG_GL) opengl_enable_debug_output();
    pix_batch2d_create(c.batch);
    c.start = c.now = c.last = pix_time_seconds();
    c.dt = 1.0f / 60.0f;
    c.fps = 60.0f;
    c.first_frame = true;
    return true;
}

void pix_shutdown(void) {
    if (!pix__ctx) return;
    pix__context& c = *pix__ctx;
    pix_stop_gamepad_rumble(c.window);
    if (c.animations_ready) pix_anim_system_destroy(c.animations);
    if (c.jobs) { pix_jobs_stop(*c.jobs); delete c.jobs; }
    if (c.audio_ready) pix_destroy_audio(c.audio);
    if (c.renderer_ready) pix_destroy_renderer(c.renderer);
    if (c.font_ready) { opengl_destroy_texture(c.font.atlas_texture); pix_free_font(c.font); }
    if (c.loader_ready) pix_destroy_data_loader(c.loader);
    pix_batch2d_destroy(c.batch);
    pix_destroy_window(c.window);
    pix_free(pix__ctx);
    pix__ctx = 0;
}

bool pix_frame(void) {
    if (!pix__ctx) return false;
    pix__context& c = *pix__ctx;
    pix_batch2d_flush(c.batch);
    pix_update_window(c.window);     // presents the finished frame, then pumps input
    if (c.window.resized && c.renderer_ready) pix_renderer_resize(c.renderer, c.window.width, c.window.height);

    c.now = pix_time_seconds();
    float dt = c.first_frame ? 1.0f / 60.0f : (float)(c.now - c.last);
    c.last = c.now;
    c.first_frame = false;
    c.dt = dt > 0.1f ? 0.1f : dt;     // a breakpoint or a drag must not launch the physics
    if (dt > 0.0f) c.fps += (1.0f / dt - c.fps) * 0.05f;

    pix_batch2d_begin(c.batch, c.window.width, c.window.height);
    glViewport(0, 0, c.window.width, c.window.height);
    if (c.renderer_ready) pix_set_time(c.renderer, (float)(c.now - c.start));
    return !(c.window.should_close || c.quit);
}

void pix_quit(void) { if (pix__ctx) pix__ctx->quit = true; }

float  pix_dt(void)     { return pix__ctx ? pix__ctx->dt : 0.0f; }
double pix_time(void)   { return pix__ctx ? pix__ctx->now - pix__ctx->start : 0.0; }
int    pix_fps(void)    { return pix__ctx ? (int)(pix__ctx->fps + 0.5f) : 0; }
int    pix_width(void)  { return pix__ctx ? pix__ctx->window.width : 0; }
int    pix_height(void) { return pix__ctx ? pix__ctx->window.height : 0; }

void pix_set_title(const char* title) { pix_set_window_title(pix__ctx->window, title); }
void pix_set_vsync_enabled(bool on)   { pix_set_vsync(pix__ctx->window, on); }
void pix_toggle_fullscreen(void)      { pix_set_fullscreen(pix__ctx->window, !pix__ctx->window.fullscreen); }
void pix_set_mouse_locked(bool locked) { pix_set_mouse_capture(pix__ctx->window, locked); }

// ---- input ----

static const pix_key_state& pix__key(int k) {
    static pix_key_state none;
    return (k >= 0 && k < 256) ? pix__ctx->window.keystates[k] : none;
}
bool pix_key_down(int key)          { return pix__key(key).held; }
bool pix_key_pressed(int key)       { return pix__key(key).pressed; }
bool pix_key_released(int key)      { return pix__key(key).released; }
bool pix_mouse_down(int button)     { return pix__key(button).held; }
bool pix_mouse_pressed(int button)  { return pix__key(button).pressed; }
bool pix_mouse_released(int button) { return pix__key(button).released; }
vec2 pix_mouse_position(void) { return v2((float)pix__ctx->window.mouse_x, (float)pix__ctx->window.mouse_y); }
vec2 pix_mouse_delta(void)    { return v2((float)pix__ctx->window.mouse_rel_x, (float)pix__ctx->window.mouse_rel_y); }
int  pix_mouse_wheel(void)    { return pix__ctx->window.mouse_wheel; }

const pix_gamepad& pix_get_gamepad(int index) {
    static pix_gamepad none;
    return (index >= 0 && index < PIX_MAX_GAMEPADS) ? pix__ctx->window.gamepads[index] : none;
}

const unsigned int* pix_text_input(int* count) {
    if (count) *count = pix__ctx->window.text_input_count;
    return pix__ctx->window.text_input;
}

vec2 pix_input_axis(void) {
    vec2 a = v2(0.0f, 0.0f);
    if (pix_key_down('A') || pix_key_down(KEY_LEFT))  a.x -= 1.0f;
    if (pix_key_down('D') || pix_key_down(KEY_RIGHT)) a.x += 1.0f;
    if (pix_key_down('W') || pix_key_down(KEY_UP))    a.y += 1.0f;
    if (pix_key_down('S') || pix_key_down(KEY_DOWN))  a.y -= 1.0f;
    if (a.x == 0.0f && a.y == 0.0f) {
        const pix_gamepad& g = pix_get_gamepad(0);
        if (g.connected) a = v2(g.left_x, g.left_y);
    } else {
        a = v2norm(a);
    }
    return a;
}

// ---- 2D ----

void pix_clear(pix_color c) {
    pix_batch2d_flush(pix__ctx->batch);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, pix__ctx->window.width, pix__ctx->window.height);
    glDisable(GL_SCISSOR_TEST);
    glDepthMask(GL_TRUE);
    glClearColor(c.r / 255.0f, c.g / 255.0f, c.b / 255.0f, c.a / 255.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void pix_begin_2d(const pix_camera2d& cam) { pix_batch2d_set_camera(pix__ctx->batch, &cam); }
void pix_end_2d(void)                      { pix_batch2d_set_camera(pix__ctx->batch, 0); }

void pix_draw_rect(float x, float y, float w, float h, pix_color c) { pix_rect r = { x, y, w, h }; pix_batch_rect(pix__ctx->batch, r, c); }
void pix_draw_rect(pix_rect r, pix_color c)                        { pix_batch_rect(pix__ctx->batch, r, c); }
void pix_draw_rect_lines(pix_rect r, float t, pix_color c)         { pix_batch_rect_lines(pix__ctx->batch, r, t, c); }
void pix_draw_rect_rounded(pix_rect r, float radius, pix_color c)  { pix_batch_rect_rounded(pix__ctx->batch, r, radius, c); }
void pix_draw_rect_gradient(pix_rect r, pix_color top, pix_color bottom) { pix_batch_rect_gradient(pix__ctx->batch, r, top, bottom); }
void pix_draw_line(vec2 a, vec2 b, float t, pix_color c)           { pix_batch_line(pix__ctx->batch, a, b, t, c); }
void pix_draw_circle(vec2 ctr, float radius, pix_color c)          { pix_batch_circle(pix__ctx->batch, ctr, radius, c); }
void pix_draw_circle_lines(vec2 ctr, float radius, float t, pix_color c) { pix_batch_circle_lines(pix__ctx->batch, ctr, radius, t, c); }
void pix_draw_triangle(vec2 a, vec2 b, vec2 c, pix_color col)      { pix_batch_triangle(pix__ctx->batch, a, b, c, col); }
void pix_draw_texture(const pix_texture& t, float x, float y, pix_color tint) { pix_batch_texture(pix__ctx->batch, t, x, y, tint); }
void pix_draw_texture_pro(const pix_texture& t, pix_rect src, pix_rect dst, vec2 origin, float rot, pix_color tint) {
    pix_batch_texture_pro(pix__ctx->batch, t, src, dst, origin, rot, tint);
}
void pix_begin_clip(pix_rect r) { pix_batch2d_scissor(pix__ctx->batch, r); }
void pix_end_clip(void)         { pix_rect none = { 0, 0, 0, 0 }; pix_batch2d_scissor(pix__ctx->batch, none); }

font_data& pix_default_font(void) {
    pix__context& c = *pix__ctx;
    if (!c.font_ready) {
        // every Windows install has at least one of these
        const char* faces[] = { "C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/arial.ttf",
                                "C:/Windows/Fonts/tahoma.ttf", "C:/Windows/Fonts/consola.ttf" };
        for (size_t i = 0; i < PIX_ARRAY_COUNT(faces) && !c.font.line_height; i++) {
            if (c.font.bitmap) pix_free_font(c.font);
            c.font = pix_load_font_ttf(faces[i]);
        }
        c.font_ready = true;
    }
    return c.font;
}

void pix_draw_text_font(font_data& f, const char* text, float x, float y, float size, pix_color c) {
    pix_batch_text(pix__ctx->batch, f, text, x, y, size, c);
}
void pix_draw_text(const char* text, float x, float y, float size, pix_color c) {
    pix_batch_text(pix__ctx->batch, pix_default_font(), text, x, y, size, c);
}
void pix_draw_textf(float x, float y, float size, pix_color c, const char* fmt, ...) {
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    pix_draw_text(buf, x, y, size, c);
}
vec2 pix_measure_text(const char* text, float size) { return pix_measure_text(pix_default_font(), text, size); }
void pix_draw_fps(float x, float y) {
    int fps = pix_fps();
    pix_color c = fps >= 55 ? PIX_GREEN : (fps >= 30 ? PIX_ORANGE : PIX_RED);
    pix_draw_textf(x, y, 20.0f, c, "%d fps", fps);
}

// ---- 3D ----

pix_renderer& pix_get_renderer(void) {
    pix__context& c = *pix__ctx;
    if (!c.renderer_ready) {
        c.renderer = pix_create_renderer(c.window.width, c.window.height);
        if (!(c.flags & PIX_NO_3D_EFFECTS)) pix_set_quality(c.renderer, PIX_QUALITY_HIGH);
        c.renderer_ready = true;
    }
    return c.renderer;
}

void pix_begin_3d(camera& cam) {
    pix_batch2d_flush(pix__ctx->batch);
    pix_renderer& r = pix_get_renderer();
    pix_renderer_resize(r, pix__ctx->window.width, pix__ctx->window.height);
    begin_frame(r, cam);
    pix__ctx->in_3d = true;
}

void pix_finish_3d(void) {
    if (!pix__ctx->in_3d) return;
    pix_finish_frame(pix__ctx->renderer);
    pix__ctx->in_3d = false;
    pix_batch2d_begin(pix__ctx->batch, pix__ctx->window.width, pix__ctx->window.height);
}

void pix_draw_target(const framebuffer& t, pix_rect dst, pix_color tint) {
    pix_texture tex = { t.color, t.width, t.height };
    // render targets are stored bottom-up: a negative source height flips them
    pix_rect src = { 0.0f, (float)t.height, (float)t.width, -(float)t.height };
    pix_batch_texture_pro(pix__ctx->batch, tex, src, dst, v2(0, 0), 0.0f, tint);
}

void pix_end_3d(void) {
    if (!pix__ctx->in_3d) return;
    end_frame(pix__ctx->renderer);
    pix__ctx->in_3d = false;
    // end_frame leaves the backbuffer bound; 2D continues on top of it
    pix_batch2d_begin(pix__ctx->batch, pix__ctx->window.width, pix__ctx->window.height);
}

static vec4 pix__tint(pix_color c) { return pix_color_to_vec4(c); }

void pix_draw_mesh(idx mesh, idx material, const mat4& transform, pix_color tint) {
    push_instance(pix_get_renderer(), mesh, material, transform, pix__tint(tint));
}

void pix_draw_model(const pix_model& m, const mat4& transform, pix_color tint) {
    if (m.mesh == PIX_INVALID) return;
    if (m.skinned) {
        // a rigged model drawn without a pose: its rest pose, cached per model skeleton
        static animation rest;
        static const skeleton_file_data* rest_of = 0;
        if (rest_of != m.skeleton && m.skeleton) { animation_rest_pose(*m.skeleton, &rest); rest_of = m.skeleton; }
        pix_draw_model_animated(m, rest, transform, tint);
        return;
    }
    push_instance(pix_get_renderer(), m.mesh, m.material, transform, pix__tint(tint));
}

void pix_draw_model_animated(const pix_model& m, const animation& pose, const mat4& transform, pix_color tint) {
    if (m.mesh == PIX_INVALID || !m.skinned) { pix_draw_model(m, transform, tint); return; }
    pix_render_instance inst;
    inst.mesh = m.mesh;
    inst.material = m.material;
    inst.transform = transform;
    inst.tint = pix__tint(tint);
    push_animated_instance(pix_get_renderer(), inst, pose);
}

void pix_draw_line3d(vec3 a, vec3 b, pix_color c)      { push_line(pix_get_renderer(), a, b, c); }
void pix_draw_grid(float size, int cells, pix_color c) { push_grid_lines(pix_get_renderer(), v3(0, 0, 0), size, cells, c); }
void pix_draw_bounds(vec3 lo, vec3 hi, pix_color c)    { push_aabb_lines(pix_get_renderer(), lo, hi, c); }
void pix_add_light(const pix_light& l)                 { push_light(pix_get_renderer(), l); }

pix_shadow_settings& pix_shadows(void)  { return pix_get_renderer().shadows; }
pix_post_settings&   pix_post(void)     { return pix_get_renderer().post; }
void pix_set_quality(int quality)        { pix_set_quality(pix_get_renderer(), quality); }
void pix_time_of_day(float hour)         { pix_set_time_of_day(pix_get_renderer(), hour); }
int  pix_add_effect(const char* body, vec4 params) { return pix_add_post_effect(pix_get_renderer(), body, params); }
void pix_effect_enable(int e, bool on) {
    pix_renderer& r = pix_get_renderer();
    if (e >= 0 && e < r.post_effect_count) r.post_effects[e].enabled = on;
}
void pix_effect_params(int e, vec4 params) {
    pix_renderer& r = pix_get_renderer();
    if (e >= 0 && e < r.post_effect_count) r.post_effects[e].params = params;
}

// ---- assets ----

pix_data_loader& pix_get_loader(void) {
    pix__context& c = *pix__ctx;
    if (!c.loader_ready) { c.loader = pix_create_data_loader(); c.loader_ready = true; }
    return c.loader;
}

static bool pix__ends_with(const char* s, const char* suffix) {
    size_t n = strlen(s), m = strlen(suffix);
    return n >= m && _stricmp(s + n - m, suffix) == 0;
}

static vec3 pix__srgb3(pix_color c) { return v3(c.r / 255.0f, c.g / 255.0f, c.b / 255.0f); }

pix_model pix_load_model(const char* path) {
    pix_model model = {};
    model.mesh = model.material = PIX_INVALID;
    pix_renderer& r = pix_get_renderer();
    pix_data_loader& l = pix_get_loader();

    if (pix__ends_with(path, ".obj")) {
        // the parsed file is only needed until it is on the GPU
        pix_loader_mark mark = pix_loader_mark_now(l);
        obj_palette palette = {};
        mesh_file_data md = {};
        if (!obj_load_file(l.arena, path, &md, &palette)) {
            pix_log("pix: could not load model '%s'", path);
            pix_loader_rewind(l, mark);
            return model;
        }
        model.mesh = load_mesh(r, md);
        model.bounds_min = md.bounds_min;
        model.bounds_max = md.bounds_max;
        material m = pix_material_default();
        if (md.texture_path[0]) {
            m.texture = pix_load_texture(md.texture_path, TEXTURE_BILINEAR).id;
        } else if (palette.count) {
            unsigned char px[OBJ_PALETTE_MAX * 4];
            obj_palette_pixels(palette, px);
            m.texture = opengl_create_texture2d(OBJ_PALETTE_DIM, OBJ_PALETTE_DIM, 4, px, TEXTURE_PIXELATED);
        }
        model.material = pix_add_material(r, m);
        pix_loader_rewind(l, mark);
        return model;
    }

    if (pix__ends_with(path, ".gltf") || pix__ends_with(path, ".glb")) {
        pix_loader_mark mark = pix_loader_mark_now(l);
        idx id = load_model_gltf_file(l, path);
        if (id == PIX_INVALID) { pix_log("pix: could not load model '%s'", path); return model; }
        model_file_data& mf = l.model_files[id];
        skinned_mesh_file_data* md = get_model_mesh(l, id);
        skeleton_file_data* sk = get_model_skeleton(l, id);
        if (!md) { pix_loader_rewind(l, mark); return model; }
        model.material = load_material_image(r, mf.image >= 0 ? &l.image_files[mf.image] : 0,
                                             v3(1, 1, 1), 0.0f, 0.6f, mf.image_is_palette);
        mesh tmp = {};
        pix__mesh_bounds(tmp, md->vertex_data, md->vertex_count, sizeof(vertex_rigged));
        model.bounds_min = tmp.bounds_min;
        model.bounds_max = tmp.bounds_max;
        if (sk && sk->bone_count) {
            // rigged: skeleton and clips stay resident, the animator reads them every frame
            model.skinned = true;
            model.mesh = load_skinned_mesh(r, *md);
            model.skeleton = sk;
            model.clip_count = mf.animation_count;
            model.clips = mf.first_animation >= 0 ? &l.animation_clips[mf.first_animation] : 0;
        } else {
            model.mesh = load_mesh_from_skinned(r, *md);
            pix_loader_rewind(l, mark);   // a static prop needs nothing CPU side
        }
        return model;
    }

    pix_log("pix: unsupported model format '%s' (use .obj, .gltf or .glb)", path);
    return model;
}

animator pix_model_animator(const pix_model& m) {
    animator a = pix_create_animator(m.skeleton);
    for (size_t i = 0; i < m.clip_count; i++) animator_add_clip(a, &m.clips[i]);
    if (a.clip_count) animator_play(a, 0);
    return a;
}

idx pix_make_material(pix_color color, float metallic, float roughness) {
    material m = pix_material_default();
    m.color = pix__srgb3(color);
    m.alpha = color.a / 255.0f;
    m.metallic = metallic;
    m.roughness = roughness;
    if (color.a < 255) m.blend = PIX_BLEND_ALPHA;
    return pix_add_material(pix_get_renderer(), m);
}

idx pix_make_material_texture(const pix_texture& t, pix_color color, float metallic, float roughness) {
    material m = pix_material_default();
    m.texture = t.id;
    m.color = pix__srgb3(color);
    m.alpha = color.a / 255.0f;
    m.metallic = metallic;
    m.roughness = roughness;
    if (color.a < 255) m.blend = PIX_BLEND_ALPHA;
    return pix_add_material(pix_get_renderer(), m);
}

idx pix_gen_cube(vec3 size)                        { return pix_mesh_cube(pix_get_renderer(), size); }
idx pix_gen_sphere(float radius, int rings, int segs) { return pix_mesh_sphere(pix_get_renderer(), radius, rings, segs); }
idx pix_gen_plane(float w, float d, float uv)      { return pix_mesh_plane(pix_get_renderer(), w, d, uv); }
idx pix_gen_cylinder(float radius, float height)   { return pix_mesh_cylinder(pix_get_renderer(), radius, height); }

font_data* pix_load_font(const char* ttf_path) {
    font_data* f = pix_alloc<font_data>(1);
    if (!f) return 0;
    *f = pix_load_font_ttf(ttf_path);
    if (!f->line_height) pix_log("pix: could not load font '%s'", ttf_path);
    return f;
}

void pix_unload_font(font_data* f) {
    if (!f) return;
    opengl_destroy_texture(f->atlas_texture);
    pix_free_font(*f);
    pix_free(f);
}

// ---- audio ----

pix_audio& pix_get_audio(void) {
    pix__context& c = *pix__ctx;
    if (!c.audio_ready) {
        if (!(c.flags & PIX_NO_AUDIO)) pix_create_audio(c.audio);
        c.audio_ready = true;
    }
    return c.audio;
}

idx pix_load_sound(const char* wav_path) {
    pix_data_loader& l = pix_get_loader();
    pix_loader_mark mark = pix_loader_mark_now(l);
    idx wav = load_sound_wav_file(l, wav_path);
    if (wav == PIX_INVALID) { pix_log("pix: could not load sound '%s'", wav_path); return PIX_INVALID; }
    idx s = load_sound(pix_get_audio(), l.sound_files[wav]);   // copied into the audio arena
    l.sound_file_count--;          // the decoded file is not needed any more
    l.arena.size = mark.arena;
    return s;
}

idx  pix_play_sound(idx s, float volume, float pitch, bool loop, float pan) {
    return play_sound(pix_get_audio(), s, volume, pitch, loop, pan);
}
void pix_stop_sound(idx voice) { stop_sound(pix_get_audio(), voice); }
void pix_set_volume(float v)   { set_master_volume(pix_get_audio(), v); }

pix_jobs& pix_get_jobs(void) {
    if (!pix__ctx->jobs) { pix__ctx->jobs = new pix_jobs(); pix_jobs_start(*pix__ctx->jobs); }
    return *pix__ctx->jobs;
}

pix_anim_system& pix_get_animations(void) {
    pix__context& c = *pix__ctx;
    if (!c.animations_ready) { pix_anim_system_create(c.animations, 4096, &pix_get_jobs()); c.animations_ready = true; }
    return c.animations;
}

void pix_draw_model_bones(const pix_model& m, const mat4* bones, int count, const mat4& transform, pix_color tint) {
    if (m.mesh == PIX_INVALID || !m.skinned) { pix_draw_model(m, transform, tint); return; }
    push_skinned(pix_get_renderer(), m.mesh, m.material, transform, bones, count, pix__tint(tint));
}

pix_window&  pix_get_window(void)  { return pix__ctx->window; }
pix_batch2d& pix_get_batch2d(void) { return pix__ctx->batch; }

#endif // PIX_APP_IMPL
