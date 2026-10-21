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
#include "atlas.hpp"
#include "tilemap.hpp"
#include "physics_debug.hpp"
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
// the size you draw in, in pixels: the window's, or the one pix_set_resolution set
PIXDEF int    pix_width(void);
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
PIXDEF void pix_draw_rect_rounded_lines(pix_rect r, float radius, float thickness, pix_color color);
PIXDEF void pix_draw_rect_gradient_h(pix_rect r, pix_color left, pix_color right);
PIXDEF void pix_draw_ellipse(vec2 center, float rx, float ry, pix_color color);
PIXDEF void pix_draw_ellipse_lines(vec2 center, float rx, float ry, float thickness, pix_color color);
// The band between two radii, from one angle to another (degrees, 0 = right,
// clockwise): arcs, gauges, cooldown rings.
PIXDEF void pix_draw_ring(vec2 center, float inner, float outer, float start_deg, float end_deg, pix_color color);
// a pie slice
PIXDEF void pix_draw_circle_sector(vec2 center, float radius, float start_deg, float end_deg, pix_color color);
// connected line segments with clean corners; `closed` joins the ends
PIXDEF void pix_draw_polyline(const vec2* points, int count, float thickness, pix_color color, bool closed = false);
// a filled convex polygon, points in order
PIXDEF void pix_draw_polygon(const vec2* points, int count, pix_color color);
// A UI panel that stretches without distorting its frame. `border` is the
// left, top, right and bottom frame width in source pixels.
PIXDEF void pix_draw_nine_slice(const pix_texture& t, pix_rect src, vec4 border, pix_rect dst,
                                pix_color tint = PIX_WHITE);
// How 2D colours combine with what is drawn: PIX_BLEND_ADDITIVE for glows and
// sparks, PIX_BLEND_MULTIPLY for shadows and darkening. pix_end_blend goes
// back to normal alpha blending.
PIXDEF void pix_begin_blend(int mode);
PIXDEF void pix_end_blend(void);
// The world rectangle a 2D camera shows on screen: skip drawing what is
// outside it. (The size is the drawing area's, see pix_set_resolution.)
PIXDEF pix_rect pix_camera2d_visible(const pix_camera2d& camera);
// the mouse position in world coordinates, through a 2D camera
PIXDEF vec2 pix_mouse_world_2d(const pix_camera2d& camera);

// ---- sprites from an atlas (see atlas.hpp) ----
// A frame drawn with its pivot at `pos`. `flags`: PIX_FLIP_X, PIX_FLIP_Y.
PIXDEF void pix_draw_sprite(const pix_atlas& atlas, int frame, vec2 pos, float scale = 1.0f,
                            float rotation_deg = 0.0f, pix_color tint = PIX_WHITE, int flags = 0);
// by name (a lookup per call: keep pix_atlas_find's index for hot loops)
PIXDEF void pix_draw_sprite(const pix_atlas& atlas, const char* frame_name, vec2 pos, float scale = 1.0f,
                            float rotation_deg = 0.0f, pix_color tint = PIX_WHITE, int flags = 0);
// whatever frame the player is on
PIXDEF void pix_draw_sprite_anim(const pix_atlas& atlas, const pix_sprite_player& player, vec2 pos,
                                 float scale = 1.0f, float rotation_deg = 0.0f, pix_color tint = PIX_WHITE,
                                 int flags = 0);

// ---- 2D physics debug view (see physics2d.hpp) ----
// Shapes, bounds, contacts and grid cells, through the current 2D camera.
PIXDEF void pix_draw_physics2d(const phys2d_world& world, unsigned flags = PHYS2D_DRAW_DEFAULT,
                               float thickness = 1.0f);

// ---- tilemaps and parallax (see tilemap.hpp) ----
// Through the current 2D camera, culled to the view; animated tiles run on pix_time.
PIXDEF void pix_draw_tilemap(pix_tilemap& map);
// one layer, so sprites can go between layers (ground, then characters, then roofs)
PIXDEF void pix_draw_tilemap_layer(pix_tilemap& map, int layer);
// A background that scrolls at `factor` of the camera: (0, 0) a fixed sky,
// (0.3, 0.3) distant hills, (1, 1) the world itself. Repeats endlessly across
// x (and y if asked). Use a texture of its own, not one from an atlas.
PIXDEF void pix_draw_parallax(const pix_texture& t, vec2 factor, vec2 offset = { 0.0f, 0.0f }, float scale = 1.0f,
                              bool repeat_x = true, bool repeat_y = false, pix_color tint = PIX_WHITE);

// ---- canvases: drawing into a texture ----
// A canvas is an image you draw into with the normal 2D calls, then draw like
// a texture: a minimap, a cached background, a trail that fades, a
// render-to-texture effect. `filter` is TEXTURE_LINEAR or TEXTURE_PIXELATED.
PIXDEF framebuffer pix_create_canvas(int width, int height, idx filter = TEXTURE_LINEAR);
PIXDEF void        pix_destroy_canvas(framebuffer& canvas);
// Everything drawn between these (and pix_clear) goes into the canvas.
PIXDEF void        pix_begin_canvas(const framebuffer& canvas);
PIXDEF void        pix_end_canvas(void);
// the canvas as a texture, for pix_draw_texture_pro and friends (its colours
// are premultiplied by alpha; pix_draw_target accounts for that by itself)
PIXDEF pix_texture pix_canvas_texture(const framebuffer& canvas);

// ---- resolution ----
// Draw at a fixed resolution and let pix scale it to the window: the whole
// game (2D, 3D, the mouse, pix_width / pix_height) then works in those
// pixels. The classic pixel-art setup is pix_set_resolution(320, 180).
#define PIX_SCALE_INTEGER 0x0    // whole multiples only: every pixel the same size, bars around
#define PIX_SCALE_FIT     0x1    // as large as fits, keeping the shape
#define PIX_SCALE_SMOOTH  0x10   // or'd in: smooth filtering instead of crisp pixels
// width or height 0 turns it off
PIXDEF void pix_set_resolution(int width, int height, int mode = PIX_SCALE_INTEGER);

// ---- screen effects ----
// Effects added with pix_add_effect run once a frame: inside pix_end_3d for
// 3D games, otherwise over the finished frame. Call this to run them earlier,
// e.g. after the world and before the HUD so the HUD stays clean.
PIXDEF void pix_apply_effects(void);

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
// the 3D physics world's shapes, contacts and joints as lines (see physics3d.hpp)
PIXDEF void pix_draw_physics3d(const phys3d_world& world, unsigned flags = PHYS3D_DRAW_DEFAULT);
PIXDEF void pix_draw_physics3d_flat(const phys3d_flat_world& world);
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

    framebuffer canvas;          // being drawn into, between pix_begin_canvas / pix_end_canvas
    bool        in_canvas;
    framebuffer virt;            // pix_set_resolution's target; fbo 0 = off
    int         virt_mode;
    pix_rect    virt_dst;        // where it lands in the window
    float       virt_scale;
    bool        effects_done;    // the screen effects already ran this frame
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
    if (c.virt.fbo) pix_destroy_framebuffer(c.virt);
    pix_batch2d_destroy(c.batch);
    pix_destroy_window(c.window);
    pix_free(pix__ctx);
    pix__ctx = 0;
}

// ---- where a frame is drawn ----
//
// The frame target is the window, or the fixed-resolution target when
// pix_set_resolution is on; a canvas temporarily replaces it.

static int pix__frame_w(const pix__context& c) { return c.virt.fbo ? c.virt.width : c.window.width; }
static int pix__frame_h(const pix__context& c) { return c.virt.fbo ? c.virt.height : c.window.height; }
static const framebuffer* pix__frame_fb(const pix__context& c) { return c.virt.fbo ? &c.virt : 0; }

// binds whatever 2D draws into right now and restarts the batch for it
static void pix__bind_draw_target(pix__context& c) {
    if (c.in_canvas) {
        pix_bind_framebuffer(c.canvas);
        pix_batch2d_begin(c.batch, c.canvas.width, c.canvas.height, c.canvas.top_down);
    } else if (c.virt.fbo) {
        pix_bind_framebuffer(c.virt);
        pix_batch2d_begin(c.batch, c.virt.width, c.virt.height);
    } else {
        pix_bind_backbuffer(c.window.width, c.window.height);
        pix_batch2d_begin(c.batch, c.window.width, c.window.height);
    }
}

// the renderer always matches the frame target, so 3D and the effects line up
static void pix__fit_renderer(pix__context& c) {
    if (c.renderer_ready) pix_renderer_resize(c.renderer, pix__frame_w(c), pix__frame_h(c));
}

// where the fixed-resolution image lands in the window, and at what scale
static void pix__place_virtual(pix__context& c) {
    float ww = (float)c.window.width, wh = (float)c.window.height;
    float vw = (float)c.virt.width, vh = (float)c.virt.height;
    float s = minf(ww / vw, wh / vh);
    // whole multiples keep every pixel the same size; a window smaller than
    // the resolution itself still gets the fractional fit rather than nothing
    if (!(c.virt_mode & PIX_SCALE_FIT) && s >= 1.0f) s = floorf(s);
    c.virt_scale = s;
    c.virt_dst.w = floorf(vw * s);
    c.virt_dst.h = floorf(vh * s);
    c.virt_dst.x = floorf((ww - c.virt_dst.w) * 0.5f);
    c.virt_dst.y = floorf((wh - c.virt_dst.h) * 0.5f);
}

// The fixed-resolution image onto the window: one blit, no shader, black bars.
static void pix__present_virtual(pix__context& c) {
    if (!c.virt.fbo) return;
    pix_batch2d_flush(c.batch);
    pix__place_virtual(c);
    pix_bind_backbuffer(c.window.width, c.window.height);
    glDisable(GL_SCISSOR_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    // both are stored bottom-up, so the rectangle only flips into GL's y
    GLint x0 = (GLint)c.virt_dst.x, y0 = (GLint)(c.window.height - c.virt_dst.y - c.virt_dst.h);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, c.virt.fbo);
    glBlitFramebuffer(0, 0, c.virt.width, c.virt.height, x0, y0, x0 + (GLint)c.virt_dst.w, y0 + (GLint)c.virt_dst.h,
                      GL_COLOR_BUFFER_BIT, (c.virt_mode & PIX_SCALE_SMOOTH) ? GL_LINEAR : GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// the screen effects over the frame target, if they have not run yet
static void pix__run_effects(pix__context& c) {
    if (c.effects_done || !c.renderer_ready) return;
    c.effects_done = true;
    pix_batch2d_flush(c.batch);
    pix__fit_renderer(c);
    pix_apply_effects(c.renderer, pix__frame_fb(c));
}

bool pix_frame(void) {
    if (!pix__ctx) return false;
    pix__context& c = *pix__ctx;
    if (c.in_canvas) { pix_log("pix: pix_begin_canvas without pix_end_canvas"); pix_end_canvas(); }
    pix__run_effects(c);
    pix_batch2d_flush(c.batch);
    pix__present_virtual(c);
    pix_update_window(c.window);     // presents the finished frame, then pumps input
    if (c.window.resized) pix__fit_renderer(c);

    c.now = pix_time_seconds();
    float dt = c.first_frame ? 1.0f / 60.0f : (float)(c.now - c.last);
    c.last = c.now;
    c.first_frame = false;
    c.dt = dt > 0.1f ? 0.1f : dt;     // a breakpoint or a drag must not launch the physics
    if (dt > 0.0f) c.fps += (1.0f / dt - c.fps) * 0.05f;

    c.effects_done = false;
    pix__bind_draw_target(c);
    if (c.renderer_ready) pix_set_time(c.renderer, (float)(c.now - c.start));
    return !(c.window.should_close || c.quit);
}

void pix_set_resolution(int width, int height, int mode) {
    pix__context& c = *pix__ctx;
    pix_batch2d_flush(c.batch);
    if (c.virt.fbo && (width != c.virt.width || height != c.virt.height || width <= 0 || height <= 0))
        pix_destroy_framebuffer(c.virt);
    c.virt_mode = mode;
    if (width > 0 && height > 0 && !c.virt.fbo) {
        c.virt = pix_create_render_target(width, height, false, false, false);
        pix_bind_framebuffer(c.virt);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
    }
    if (c.virt.fbo) pix__place_virtual(c);
    pix__fit_renderer(c);
    if (!c.in_canvas) pix__bind_draw_target(c);
}

void pix_apply_effects(void) {
    pix__context& c = *pix__ctx;
    if (c.in_canvas) return;            // effects run over the frame, not a canvas
    pix__run_effects(c);
    // the effect passes moved the GL state; carry on drawing where we were
    mat4 transform = c.batch.transform;
    pix__bind_draw_target(c);
    c.batch.transform = transform;
}

void pix_quit(void) { if (pix__ctx) pix__ctx->quit = true; }

float  pix_dt(void)     { return pix__ctx ? pix__ctx->dt : 0.0f; }
double pix_time(void)   { return pix__ctx ? pix__ctx->now - pix__ctx->start : 0.0; }
int    pix_fps(void)    { return pix__ctx ? (int)(pix__ctx->fps + 0.5f) : 0; }
int    pix_width(void)  { return pix__ctx ? pix__frame_w(*pix__ctx) : 0; }
int    pix_height(void) { return pix__ctx ? pix__frame_h(*pix__ctx) : 0; }

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
// in the drawing area's pixels: with a fixed resolution, through its scale
vec2 pix_mouse_position(void) {
    const pix__context& c = *pix__ctx;
    vec2 m = v2((float)c.window.mouse_x, (float)c.window.mouse_y);
    if (!c.virt.fbo || c.virt_scale <= 0.0f) return m;
    return v2((m.x - c.virt_dst.x) / c.virt_scale, (m.y - c.virt_dst.y) / c.virt_scale);
}
vec2 pix_mouse_delta(void) {
    const pix__context& c = *pix__ctx;
    vec2 d = v2((float)c.window.mouse_rel_x, (float)c.window.mouse_rel_y);
    return (c.virt.fbo && c.virt_scale > 0.0f) ? v2scale(d, 1.0f / c.virt_scale) : d;
}
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

// clears whatever 2D draws into right now: a canvas, the fixed-resolution
// target or the window
void pix_clear(pix_color c) {
    pix__context& x = *pix__ctx;
    pix_batch2d_flush(x.batch);
    if (x.in_canvas)       pix_bind_framebuffer(x.canvas);
    else if (x.virt.fbo)   pix_bind_framebuffer(x.virt);
    else                   pix_bind_backbuffer(x.window.width, x.window.height);
    glDisable(GL_SCISSOR_TEST);
    x.batch.scissor = false;
    glDepthMask(GL_TRUE);
    glClearColor(c.r / 255.0f, c.g / 255.0f, c.b / 255.0f, c.a / 255.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void pix_begin_2d(const pix_camera2d& cam) { pix_batch2d_set_camera(pix__ctx->batch, &cam); }
void pix_end_2d(void)                      { pix_batch2d_set_camera(pix__ctx->batch, 0); }

void pix_draw_rect_rounded_lines(pix_rect r, float radius, float t, pix_color c) {
    pix_batch_rect_rounded_lines(pix__ctx->batch, r, radius, t, c);
}
void pix_draw_rect_gradient_h(pix_rect r, pix_color left, pix_color right) {
    pix_batch_rect_colors(pix__ctx->batch, r, left, right, right, left);
}
void pix_draw_ellipse(vec2 ctr, float rx, float ry, pix_color c) { pix_batch_ellipse(pix__ctx->batch, ctr, rx, ry, c); }
void pix_draw_ellipse_lines(vec2 ctr, float rx, float ry, float t, pix_color c) {
    pix_batch_ellipse_lines(pix__ctx->batch, ctr, rx, ry, t, c);
}
void pix_draw_ring(vec2 ctr, float inner, float outer, float a0, float a1, pix_color c) {
    pix_batch_ring(pix__ctx->batch, ctr, inner, outer, a0, a1, c);
}
void pix_draw_circle_sector(vec2 ctr, float radius, float a0, float a1, pix_color c) {
    pix_batch_ring(pix__ctx->batch, ctr, 0.0f, radius, a0, a1, c);
}
void pix_draw_polyline(const vec2* pts, int n, float t, pix_color c, bool closed) {
    pix_batch_polyline(pix__ctx->batch, pts, n, t, c, closed);
}
void pix_draw_polygon(const vec2* pts, int n, pix_color c) { pix_batch_polygon(pix__ctx->batch, pts, n, c); }
void pix_draw_nine_slice(const pix_texture& t, pix_rect src, vec4 border, pix_rect dst, pix_color tint) {
    pix_batch_nine_slice(pix__ctx->batch, t, src, border, dst, tint);
}
void pix_begin_blend(int mode) { pix_batch2d_set_blend(pix__ctx->batch, mode); }
void pix_end_blend(void)       { pix_batch2d_set_blend(pix__ctx->batch, PIX_BLEND_ALPHA); }

pix_rect pix_camera2d_visible(const pix_camera2d& cam) {
    return pix_camera2d_visible(cam, (float)pix_width(), (float)pix_height());
}
vec2 pix_mouse_world_2d(const pix_camera2d& cam) { return pix_screen_to_world_2d(cam, pix_mouse_position()); }

// ---- sprites ----

void pix_draw_sprite(const pix_atlas& a, int frame, vec2 pos, float scale, float rot, pix_color tint, int flags) {
    pix_batch_sprite(pix__ctx->batch, a, frame, pos, v2(scale, scale), rot, tint, flags);
}
void pix_draw_sprite(const pix_atlas& a, const char* name, vec2 pos, float scale, float rot, pix_color tint, int flags) {
    pix_batch_sprite(pix__ctx->batch, a, pix_atlas_find(a, name), pos, v2(scale, scale), rot, tint, flags);
}
void pix_draw_sprite_anim(const pix_atlas& a, const pix_sprite_player& p, vec2 pos, float scale, float rot,
                          pix_color tint, int flags) {
    pix_batch_sprite(pix__ctx->batch, a, pix_sprite_frame(p, a), pos, v2(scale, scale), rot, tint, flags);
}

// ---- tilemaps ----

void pix_draw_tilemap(pix_tilemap& map) { pix_batch_tilemap(pix__ctx->batch, map, (float)pix_time()); }
void pix_draw_physics2d(const phys2d_world& world, unsigned flags, float thickness) {
    phys2d_debug_draw(pix__ctx->batch, world, flags, thickness);
}
void pix_draw_tilemap_layer(pix_tilemap& map, int layer) {
    pix_batch_tilemap_layer(pix__ctx->batch, map, layer, (float)pix_time());
}
void pix_draw_parallax(const pix_texture& t, vec2 factor, vec2 offset, float scale, bool rx, bool ry, pix_color tint) {
    pix_batch_parallax(pix__ctx->batch, t, factor, offset, scale, rx, ry, tint);
}

// ---- canvases ----

framebuffer pix_create_canvas(int width, int height, idx filter) {
    pix_batch2d_flush(pix__ctx->batch);
    framebuffer f = pix_create_render_target(width, height, false, false, false);
    f.top_down = true;
    GLint mode = (filter & 0xFF) == TEXTURE_PIXELATED ? GL_NEAREST : GL_LINEAR;
    glBindTexture(GL_TEXTURE_2D, f.color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, mode);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mode);
    // a fresh canvas is transparent, not whatever the driver left there
    pix_bind_framebuffer(f);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    pix__context& c = *pix__ctx;
    if (c.in_canvas)     pix_bind_framebuffer(c.canvas);
    else if (c.virt.fbo) pix_bind_framebuffer(c.virt);
    else                 pix_bind_backbuffer(c.window.width, c.window.height);
    return f;
}

void pix_destroy_canvas(framebuffer& canvas) { pix_destroy_framebuffer(canvas); }

void pix_begin_canvas(const framebuffer& canvas) {
    pix__context& c = *pix__ctx;
    if (c.in_canvas) { pix_log("pix: canvases do not nest; end the first one before beginning another"); return; }
    pix_batch2d_flush(c.batch);
    c.canvas = canvas;
    c.in_canvas = true;
    pix__bind_draw_target(c);
}

void pix_end_canvas(void) {
    pix__context& c = *pix__ctx;
    if (!c.in_canvas) return;
    pix_batch2d_flush(c.batch);
    c.in_canvas = false;
    pix__bind_draw_target(c);
}

pix_texture pix_canvas_texture(const framebuffer& canvas) {
    pix_texture t = { canvas.color, canvas.width, canvas.height };
    return t;
}

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
        c.renderer = pix_create_renderer(pix__frame_w(c), pix__frame_h(c));
        if (!(c.flags & PIX_NO_3D_EFFECTS)) pix_set_quality(c.renderer, PIX_QUALITY_HIGH);
        c.renderer_ready = true;
    }
    return c.renderer;
}

void pix_begin_3d(camera& cam) {
    pix__context& c = *pix__ctx;
    pix_batch2d_flush(c.batch);
    pix_renderer& r = pix_get_renderer();
    pix__fit_renderer(c);
    begin_frame(r, cam);
    c.in_3d = true;
}

void pix_finish_3d(void) {
    pix__context& c = *pix__ctx;
    if (!c.in_3d) return;
    pix_finish_frame(c.renderer);
    c.in_3d = false;
    pix__bind_draw_target(c);
}

void pix_draw_target(const framebuffer& t, pix_rect dst, pix_color tint) {
    pix_batch2d& b = pix__ctx->batch;
    pix_texture tex = { t.color, t.width, t.height };
    if (!t.top_down) {
        // 3D targets are stored bottom-up: a negative source height flips them
        pix_rect src = { 0.0f, (float)t.height, (float)t.width, -(float)t.height };
        pix_batch_texture_pro(b, tex, src, dst, v2(0, 0), 0.0f, tint);
        return;
    }
    // A canvas holds colour already multiplied by its alpha (that is what
    // alpha blending into a transparent target produces), so it goes back
    // with premultiplied blending, and the tint is premultiplied to match.
    int prev = b.blend;
    pix_batch2d_set_blend(b, PIX_BLEND_PREMULTIPLIED);
    float a = tint.a / 255.0f;
    pix_color pre = pix_rgba((int)(tint.r * a + 0.5f), (int)(tint.g * a + 0.5f), (int)(tint.b * a + 0.5f), tint.a);
    pix_rect src = { 0.0f, 0.0f, (float)t.width, (float)t.height };
    pix_batch_texture_pro(b, tex, src, dst, v2(0, 0), 0.0f, pre);
    pix_batch2d_set_blend(b, prev);
}

void pix_end_3d(void) {
    pix__context& c = *pix__ctx;
    if (!c.in_3d) return;
    if (c.virt.fbo) {
        // end_frame's stages, with the fixed-resolution target as the output
        pix_renderer& r = c.renderer;
        pix_render_shadows(r);
        pix_render_reflections(r);
        pix_render_view(r, pix_view_default(r, r.frame_camera));
        pix_post_process(r, r.scene_target, &c.virt);
        pix_finish_frame(r);
    } else {
        end_frame(c.renderer);
    }
    c.in_3d = false;
    c.effects_done = true;     // the post pass ran them, before any HUD
    // 2D continues on top of the 3D image
    pix__bind_draw_target(c);
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
void pix_draw_physics3d(const phys3d_world& w, unsigned flags) { phys3d_debug_draw(pix_get_renderer(), w, flags); }
void pix_draw_physics3d_flat(const phys3d_flat_world& w)         { phys3d_flat_debug_draw(pix_get_renderer(), w); }
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
