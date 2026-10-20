// pix - sprite atlases and sprite animation.
//
// An atlas is a texture plus the rectangles of the sprites in it: frames,
// each with a pivot, and animations, each a sequence of frames with a
// timing. Everything drawn from one atlas shares one texture, so it batches
// into a single draw call.
//
//   pix_texture tex = pix_load_texture("hero.png", TEXTURE_PIXELATED);
//   pix_rect rects[32];
//   int n = pix_rects_grid(rects, 32, tex.width, tex.height, 32, 32);   // or list them by hand
//   pix_atlas hero = pix_create_atlas(tex, rects, n);
//   pix_atlas_add_anim(hero, "run", 0, 6, 12.0f);
//   pix_sprite_player p = {};
//   // each frame:
//   pix_sprite_play(p, hero, "run");
//   pix_sprite_update(p, hero, dt);
//   pix_draw_sprite_anim(hero, p, position);
//
// The rectangles are yours to make: by hand, with the pix_rects_* helpers
// (grids, rows, columns), or read from an Aseprite / TexturePacker export
// with pix_rects_from_json - whose extras (names, trimming, durations, tags)
// pix_atlas_apply_json adds afterwards. pix_atlas_build is the other way in:
// it packs loose images into a texture of its own (skyline packing, edges
// extruded so neighbouring sprites never bleed into each other).
#ifndef PIX_ATLAS_HPP
#define PIX_ATLAS_HPP
#include "dtype.hpp"
#include "math.hpp"
#include "texture.hpp"
#include "draw2d.hpp"
#include "../loader/json.hpp"

struct pix_atlas_frame {
    pix_rect src;        // the pixels in the atlas texture
    vec2  offset;        // where src sits inside the untrimmed frame (trimmed exports cut empty borders)
    vec2  size;          // the untrimmed frame's size
    vec2  pivot;         // pixels from the untrimmed frame's top-left: placed at the draw position, turned about
    float duration;      // seconds this frame shows when animated; 0 = the animation's fps decides
    uint32_t name_hash;
    uint32_t name;       // offset into pix_atlas::names, or 0xFFFFFFFF when unnamed
};

#define PIX_ANIM_LOOP     0   // round and round
#define PIX_ANIM_ONCE     1   // stops on the last frame (pix_sprite_player::finished)
#define PIX_ANIM_PINGPONG 2   // forward, then back, then forward...

struct pix_sprite_anim {
    char     name[32];
    uint32_t name_hash;
    uint32_t first;      // into pix_atlas::sequence
    uint32_t count;      // frames in the sequence
    float    fps;        // for frames without a duration of their own (0 = 10)
    uint8_t  mode;       // PIX_ANIM_*
};

struct pix_atlas {
    pix_texture texture;
    bool owns_texture;   // pix_unload_atlas frees the texture too
    pix_atlas_frame*  frames;  int frame_count, frame_cap;
    pix_sprite_anim*  anims;   int anim_count, anim_cap;
    uint16_t* sequence;        int sequence_count, sequence_cap;   // frame indices, in play order
    char*     names;           uint32_t names_used, names_cap;
};

// ---- making atlases ----

// One frame per rectangle (pixels in `texture`), in order: frame i is
// rects[i]. Pivots start at each frame's centre. The texture stays yours -
// pix_unload_atlas does not free it.
PIXDEF pix_atlas pix_create_atlas(pix_texture texture, const pix_rect* rects, int count);
// Frees the frames and animations (and the texture only if pix_atlas_build made it).
PIXDEF void      pix_unload_atlas(pix_atlas& atlas);

// ---- rectangles ----
//
// Each fills `out` (room for `max`) and returns how many it wrote.

// Equal cells of an image, left to right then top to bottom, after a
// `margin` around the image and `spacing` between cells: a sprite sheet.
PIXDEF int pix_rects_grid(pix_rect* out, int max, int image_w, int image_h, int cell_w, int cell_h,
                          int margin = 0, int spacing = 0);
// `count` cells side by side from (x, y): one animation strip of a sheet
PIXDEF int pix_rects_row(pix_rect* out, int max, float x, float y, float cell_w, float cell_h, int count,
                         float spacing = 0.0f);
// `count` cells stacked downwards from (x, y)
PIXDEF int pix_rects_column(pix_rect* out, int max, float x, float y, float cell_w, float cell_h, int count,
                            float spacing = 0.0f);
// The frame rectangles of an Aseprite or TexturePacker JSON export (hash or
// array format), in the file's order. `image_out`, if given, receives the
// image's path, relative to the JSON's folder like the file says it. 0 if
// the file cannot be read (logged).
PIXDEF int pix_rects_from_json(const char* json_path, pix_rect* out, int max, char* image_out = nullptr,
                               size_t image_cap = 0);
// The rest of such an export, onto an atlas created from its rectangles:
// frame names, trimming, pivots and per-frame durations, Aseprite tags and
// TexturePacker animation lists as animations. False if the file cannot be read.
PIXDEF bool pix_atlas_apply_json(pix_atlas& atlas, const char* json_path);

// frame index by name, -1 if there is none (look it up once, keep the index)
PIXDEF int  pix_atlas_find(const pix_atlas& atlas, const char* frame_name);
// Sets every frame's pivot as a fraction of its size: (0.5, 0.5) the centre
// (the default), (0.5, 1) the feet, (0, 0) the top-left corner.
PIXDEF void pix_atlas_set_pivot(pix_atlas& atlas, vec2 normalized);

// ---- animations ----

// `count` frames from `first_frame` on, at `fps`. Returns the animation's index.
PIXDEF int pix_atlas_add_anim(pix_atlas& atlas, const char* name, int first_frame, int count, float fps,
                              int mode = PIX_ANIM_LOOP);
// any frames, in any order (repeats allowed)
PIXDEF int pix_atlas_add_anim_frames(pix_atlas& atlas, const char* name, const int* frames, int count, float fps,
                                     int mode = PIX_ANIM_LOOP);
PIXDEF int pix_atlas_find_anim(const pix_atlas& atlas, const char* name);

// A playhead: one per animated thing, zero-initialised is ready to use.
struct pix_sprite_player {
    int      anim;       // which animation (-1 or 0 before the first play: see pix_sprite_play)
    int      step;       // position in the animation, 0..count-1
    float    time;       // seconds into the current step
    float    speed;      // playback rate; 0 counts as 1
    bool     paused;
    bool     finished;   // a PIX_ANIM_ONCE animation has reached its last frame
    bool     playing;    // pix_sprite_play has been called
    int      dir;        // +1 forward, -1 backward (ping-pong)
    uint32_t loops;      // times it has wrapped round
    uint64_t entered;    // steps entered during the last update, one bit per step (first 64)
};

// Starts an animation, unless it is the one already playing (pass `restart`
// to start it over anyway) - so calling it every frame with the animation you
// want is the normal way to use it.
PIXDEF void pix_sprite_play(pix_sprite_player& p, const pix_atlas& atlas, const char* anim, bool restart = false);
PIXDEF void pix_sprite_play(pix_sprite_player& p, int anim, bool restart = false);
PIXDEF void pix_sprite_update(pix_sprite_player& p, const pix_atlas& atlas, float dt);
// the atlas frame to draw now, -1 if nothing is playing
PIXDEF int  pix_sprite_frame(const pix_sprite_player& p, const pix_atlas& atlas);
// True if the last update stepped onto `step` (a footstep frame, the frame a
// sword connects). Counts every step passed, even when a long update skips
// several at once.
static inline bool pix_sprite_entered(const pix_sprite_player& p, int step) {
    return step >= 0 && step < 64 && ((p.entered >> step) & 1u) != 0;
}

// ---- packing atlases at load time ----
//
// Many small images cost a texture each; packed into one they draw as one.
//
//   pix_atlas_builder b = {};
//   pix_atlas_builder_add_file(b, "coin", "coin.png");
//   pix_atlas_builder_add_sheet(b, "run", "hero_run.png", 32, 32, 12);   // frames run_0.. + animation "run"
//   pix_atlas atlas = pix_atlas_build(b);                                 // frees the builder's images
struct pix__atlas_item {
    unsigned char* pixels;   // RGBA, owned by the builder until the build
    int w, h;
    int x, y;                // where the build placed it (inside the padding)
    uint32_t name;           // into the builder's names
};
struct pix__atlas_pending_anim { char name[32]; int first, count; float fps; int mode; };
struct pix_atlas_builder {
    pix__atlas_item* items;  int count, cap;
    pix__atlas_pending_anim* anims; int anim_count, anim_cap;
    char* names; uint32_t names_used, names_cap;
};
// one image from RGBA pixels (copied)
PIXDEF void pix_atlas_builder_add(pix_atlas_builder& b, const char* name, const void* rgba, int w, int h);
PIXDEF bool pix_atlas_builder_add_file(pix_atlas_builder& b, const char* name, const char* png_path);
// A sheet of equal cells (after a `margin` around the sheet, `spacing` between
// cells): frames named name_0, name_1, ... and, when fps > 0, an animation
// called `name` over all of them. Returns how many frames it added.
PIXDEF int  pix_atlas_builder_add_sheet(pix_atlas_builder& b, const char* name, const char* png_path,
                                        int frame_w, int frame_h, float fps = 10.0f, int mode = PIX_ANIM_LOOP,
                                        int margin = 0, int spacing = 0);
// Packs everything into one texture no larger than max_size on a side,
// `padding` pixels apart with the edges extruded into the gap. Frees the
// builder's images either way; frame_count is 0 if it did not fit.
PIXDEF pix_atlas pix_atlas_build(pix_atlas_builder& b, idx texture_flags = TEXTURE_PIXELATED,
                                 int max_size = 4096, int padding = 1);
PIXDEF void pix_atlas_builder_free(pix_atlas_builder& b);

// ---- drawing ----

#define PIX_FLIP_X 1   // mirrored left-right about the pivot
#define PIX_FLIP_Y 2   // upside down about the pivot

// Frame `frame` with its pivot at `pos`, scaled, turned `rotation_deg`
// about the pivot. Trimmed frames land exactly where the untrimmed ones would.
PIXDEF void pix_batch_sprite(pix_batch2d& b, const pix_atlas& atlas, int frame, vec2 pos, vec2 scale,
                             float rotation_deg, pix_color tint, int flags);

#endif // PIX_ATLAS_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_ATLAS_IMPL)
#define PIX_ATLAS_IMPL
#include <stdio.h>

static uint32_t pix__name_hash(const char* s) {              // FNV-1a
    uint32_t h = 2166136261u;
    for (; s && *s; s++) { h ^= (uint8_t)*s; h *= 16777619u; }
    return h;
}

// grows a pix_mem array to hold `need` elements
template<typename T>
static bool pix__grow(T*& data, int& cap, int need) {
    if (need <= cap) return true;
    int n = cap ? cap : 16;
    while (n < need) n *= 2;
    T* grown = (T*)pix_mem_realloc(data, (size_t)n * sizeof(T));
    if (!grown) return false;
    data = grown;
    cap = n;
    return true;
}

static uint32_t pix__pool_add(char*& pool, uint32_t& used, uint32_t& cap, const char* s) {
    if (!s || !*s) return 0xFFFFFFFFu;
    uint32_t n = (uint32_t)strlen(s) + 1;
    if (used + n > cap) {
        uint32_t c = cap ? cap : 256;
        while (c < used + n) c *= 2;
        char* grown = (char*)pix_mem_realloc(pool, c);
        if (!grown) return 0xFFFFFFFFu;
        pool = grown;
        cap = c;
    }
    memcpy(pool + used, s, n);
    uint32_t at = used;
    used += n;
    return at;
}

static int pix__atlas_add_frame(pix_atlas& a, const char* name, pix_rect src, vec2 offset, vec2 size, float duration) {
    if (!pix__grow(a.frames, a.frame_cap, a.frame_count + 1)) return -1;
    pix_atlas_frame& f = a.frames[a.frame_count];
    f.src = src;
    f.offset = offset;
    f.size = size;
    f.pivot = v2(size.x * 0.5f, size.y * 0.5f);
    f.duration = duration;
    f.name_hash = name ? pix__name_hash(name) : 0;
    f.name = pix__pool_add(a.names, a.names_used, a.names_cap, name);
    return a.frame_count++;
}

pix_atlas pix_create_atlas(pix_texture t, const pix_rect* rects, int count) {
    pix_atlas a = {};
    a.texture = t;
    if (count > 65535) { pix_log("pix: an atlas holds at most 65535 frames"); count = 65535; }
    if (count > 0 && !pix__grow(a.frames, a.frame_cap, count)) return a;
    for (int i = 0; i < count; i++)
        pix__atlas_add_frame(a, 0, rects[i], v2(0, 0), v2(rects[i].w, rects[i].h), 0.0f);
    return a;
}

int pix_rects_grid(pix_rect* out, int max, int iw, int ih, int cw, int ch, int margin, int spacing) {
    int n = 0;
    if (cw <= 0 || ch <= 0) return 0;
    for (int y = margin; y + ch <= ih - margin; y += ch + spacing)
        for (int x = margin; x + cw <= iw - margin && n < max; x += cw + spacing) {
            pix_rect r = { (float)x, (float)y, (float)cw, (float)ch };
            out[n++] = r;
        }
    return n;
}

int pix_rects_row(pix_rect* out, int max, float x, float y, float cw, float ch, int count, float spacing) {
    int n = count < max ? count : max;
    for (int i = 0; i < n; i++) { pix_rect r = { x + i * (cw + spacing), y, cw, ch }; out[i] = r; }
    return n > 0 ? n : 0;
}

int pix_rects_column(pix_rect* out, int max, float x, float y, float cw, float ch, int count, float spacing) {
    int n = count < max ? count : max;
    for (int i = 0; i < n; i++) { pix_rect r = { x, y + i * (ch + spacing), cw, ch }; out[i] = r; }
    return n > 0 ? n : 0;
}

void pix_unload_atlas(pix_atlas& a) {
    if (a.owns_texture) pix_unload_texture(a.texture);
    pix_free(a.frames);
    pix_free(a.anims);
    pix_free(a.sequence);
    pix_free(a.names);
    a = pix_atlas();
}

int pix_atlas_find(const pix_atlas& a, const char* name) {
    uint32_t h = pix__name_hash(name);
    for (int i = 0; i < a.frame_count; i++)
        if (a.frames[i].name_hash == h && a.frames[i].name != 0xFFFFFFFFu && strcmp(a.names + a.frames[i].name, name) == 0)
            return i;
    return -1;
}

void pix_atlas_set_pivot(pix_atlas& a, vec2 n) {
    for (int i = 0; i < a.frame_count; i++) a.frames[i].pivot = v2(a.frames[i].size.x * n.x, a.frames[i].size.y * n.y);
}

// a new animation with room for `count` frame indices; the caller fills them
static uint16_t* pix__atlas_new_anim(pix_atlas& a, const char* name, int count, float fps, int mode) {
    if (count <= 0 || !pix__grow(a.anims, a.anim_cap, a.anim_count + 1) ||
        !pix__grow(a.sequence, a.sequence_cap, a.sequence_count + count)) return 0;
    pix_sprite_anim& an = a.anims[a.anim_count++];
    memset(&an, 0, sizeof(an));
    strncpy(an.name, name ? name : "", sizeof(an.name) - 1);
    an.name_hash = pix__name_hash(an.name);
    an.first = (uint32_t)a.sequence_count;
    an.count = (uint32_t)count;
    an.fps = fps;
    an.mode = (uint8_t)mode;
    uint16_t* slots = a.sequence + a.sequence_count;
    a.sequence_count += count;
    return slots;
}

int pix_atlas_add_anim_frames(pix_atlas& a, const char* name, const int* frames, int count, float fps, int mode) {
    for (int i = 0; i < count; i++)
        if (frames[i] < 0 || frames[i] >= a.frame_count) {
            pix_log("pix: animation '%s' uses frame %d, the atlas has %d", name ? name : "", frames[i], a.frame_count);
            return -1;
        }
    uint16_t* s = pix__atlas_new_anim(a, name, count, fps, mode);
    if (!s) return -1;
    for (int i = 0; i < count; i++) s[i] = (uint16_t)frames[i];
    return a.anim_count - 1;
}

int pix_atlas_add_anim(pix_atlas& a, const char* name, int first, int count, float fps, int mode) {
    if (count <= 0 || first < 0 || first + count > a.frame_count) {
        pix_log("pix: animation '%s' wants frames %d..%d, the atlas has %d", name ? name : "", first, first + count - 1, a.frame_count);
        return -1;
    }
    uint16_t* s = pix__atlas_new_anim(a, name, count, fps, mode);
    if (!s) return -1;
    for (int i = 0; i < count; i++) s[i] = (uint16_t)(first + i);
    return a.anim_count - 1;
}

int pix_atlas_find_anim(const pix_atlas& a, const char* name) {
    uint32_t h = pix__name_hash(name);
    for (int i = 0; i < a.anim_count; i++)
        if (a.anims[i].name_hash == h && strcmp(a.anims[i].name, name) == 0) return i;
    return -1;
}

// ---- JSON (Aseprite / TexturePacker) ----

// Both JSON helpers walk the export's frames the same way: an object keyed by
// name (hash format) or an array whose entries carry "filename". `fn` gets
// the frame's index, its name and its token.
template<typename F>
static void pix__json_frames(const js_doc& d, int root, F fn) {
    char name[128];
    int frames = js_get(d, root, "frames"), i = 0;
    if (js_type(d, frames) == JS_OBJ) {
        for (int k = js_first(d, frames); k >= 0; k = js_next(d, frames, js_value_of(d, k)), i++) {
            js_str(d, k, name, sizeof(name));
            fn(i, name, js_value_of(d, k));
        }
    } else {
        for (int e = js_first(d, frames); e >= 0; e = js_next(d, frames, e), i++) {
            js_str(d, js_get(d, e, "filename"), name, sizeof(name));
            fn(i, name, e);
        }
    }
}

static int pix__json_open(const char* path, js_doc& d) {
    size_t size = 0;
    char* text = (char*)pix_read_file(path, &size, PIX_MEM_SCOPED);
    int root = text ? js_parse(d, text, size, PIX_MEM_SCOPED) : -1;
    if (root < 0) pix_log("pix: could not read '%s'", path);
    return root;
}

int pix_rects_from_json(const char* json_path, pix_rect* out, int max, char* image_out, size_t image_cap) {
    pix_mem_scope scope = pix_scope_begin();
    js_doc d;
    int root = pix__json_open(json_path, d), n = 0;
    if (root >= 0) {
        pix__json_frames(d, root, [&](int i, const char* name, int fr) {
            if (i >= max) return;
            int r = js_get(d, fr, "frame");
            pix_rect src = { js_float(d, js_get(d, r, "x"), 0), js_float(d, js_get(d, r, "y"), 0),
                             js_float(d, js_get(d, r, "w"), 0), js_float(d, js_get(d, r, "h"), 0) };
            if (js_bool(d, js_get(d, fr, "rotated"), false))
                pix_log("pix: frame '%s' is stored rotated; export without rotation", name);
            out[n++] = src;
        });
        if (image_out && image_cap) js_str(d, js_get(d, js_get(d, root, "meta"), "image"), image_out, image_cap);
    }
    pix_scope_end(scope);
    return n;
}

bool pix_atlas_apply_json(pix_atlas& a, const char* json_path) {
    pix_mem_scope scope = pix_scope_begin();
    js_doc d;
    int root = pix__json_open(json_path, d);
    if (root < 0) { pix_scope_end(scope); return false; }
    char name[128];

    // per frame: name, trimming, pivot, duration
    pix__json_frames(d, root, [&](int i, const char* fname, int fr) {
        if (i >= a.frame_count) return;
        pix_atlas_frame& f = a.frames[i];
        int sss = js_get(d, fr, "spriteSourceSize"), ss = js_get(d, fr, "sourceSize");
        f.offset = v2(js_float(d, js_get(d, sss, "x"), 0), js_float(d, js_get(d, sss, "y"), 0));
        f.size = v2(js_float(d, js_get(d, ss, "w"), f.src.w), js_float(d, js_get(d, ss, "h"), f.src.h));
        f.duration = js_float(d, js_get(d, fr, "duration"), 0.0f) * 0.001f;   // Aseprite: milliseconds
        int pv = js_get(d, fr, "pivot");                                      // TexturePacker: normalised
        f.pivot = v2(f.size.x * js_float(d, js_get(d, pv, "x"), 0.5f), f.size.y * js_float(d, js_get(d, pv, "y"), 0.5f));
        f.name_hash = pix__name_hash(fname);
        f.name = pix__pool_add(a.names, a.names_used, a.names_cap, fname);
    });
    int meta = js_get(d, root, "meta");

    // Aseprite tags: frame ranges with a direction
    int tags = js_get(d, meta, "frameTags");
    for (int t = js_first(d, tags); t >= 0; t = js_next(d, tags, t)) {
        js_str(d, js_get(d, t, "name"), name, sizeof(name));
        int from = js_int(d, js_get(d, t, "from"), 0), to = js_int(d, js_get(d, t, "to"), 0);
        if (from < 0 || to >= a.frame_count || to < from) continue;
        bool reverse = js_is(d, js_get(d, t, "direction"), "reverse") ||
                       js_is(d, js_get(d, t, "direction"), "pingpong_reverse");
        bool ping = js_is(d, js_get(d, t, "direction"), "pingpong") ||
                    js_is(d, js_get(d, t, "direction"), "pingpong_reverse");
        int n = to - from + 1;
        int* list = pix_new<int>((size_t)n, PIX_MEM_SCOPED);
        for (int i = 0; i < n; i++) list[i] = reverse ? to - i : from + i;
        pix_atlas_add_anim_frames(a, name, list, n, 0.0f, ping ? PIX_ANIM_PINGPONG : PIX_ANIM_LOOP);
    }

    // TexturePacker: "animations": { "walk": ["walk_0.png", ...] }
    int anims = js_get(d, root, "animations");
    if (js_type(d, anims) == JS_OBJ) {
        for (int k = js_first(d, anims); k >= 0; k = js_next(d, anims, js_value_of(d, k))) {
            js_str(d, k, name, sizeof(name));
            int arr = js_value_of(d, k), n = js_len(d, arr), got = 0;
            int* list = pix_new<int>((size_t)(n > 0 ? n : 1), PIX_MEM_SCOPED);
            char fname[128];
            for (int e = js_first(d, arr); e >= 0; e = js_next(d, arr, e)) {
                js_str(d, e, fname, sizeof(fname));
                int f = pix_atlas_find(a, fname);
                if (f >= 0) list[got++] = f;
            }
            if (got) pix_atlas_add_anim_frames(a, name, list, got, 10.0f, PIX_ANIM_LOOP);
        }
    }
    pix_scope_end(scope);
    return true;
}

// ---- playback ----

void pix_sprite_play(pix_sprite_player& p, int anim, bool restart) {
    if (p.playing && p.anim == anim && !restart) return;
    p.anim = anim;
    p.step = 0;
    p.time = 0.0f;
    p.dir = 1;
    p.finished = false;
    p.loops = 0;
    p.entered = 1;            // the first step counts as entered
    p.playing = anim >= 0;
}

void pix_sprite_play(pix_sprite_player& p, const pix_atlas& a, const char* anim, bool restart) {
    int i = pix_atlas_find_anim(a, anim);
    if (i < 0 && p.anim != -2) {
        pix_log("pix: no animation '%s' in this atlas", anim);
        p.anim = -2;          // logged once, not every frame
        return;
    }
    if (i >= 0) pix_sprite_play(p, i, restart);
}

static float pix__step_seconds(const pix_atlas& a, const pix_sprite_anim& an, int step) {
    float d = a.frames[a.sequence[an.first + step]].duration;
    if (d > 0.0f) return d;
    return 1.0f / (an.fps > 0.0f ? an.fps : 10.0f);
}

void pix_sprite_update(pix_sprite_player& p, const pix_atlas& a, float dt) {
    p.entered = 0;
    if (!p.playing || p.paused || p.finished || p.anim < 0 || p.anim >= a.anim_count) return;
    const pix_sprite_anim& an = a.anims[p.anim];
    int count = (int)an.count;
    if (count <= 0) return;
    p.time += dt * (p.speed > 0.0f ? p.speed : 1.0f);
    float dur = pix__step_seconds(a, an, p.step);
    for (int guard = 0; p.time >= dur && guard < 4096; guard++) {
        p.time -= dur;
        int next = p.step + p.dir;
        if (an.mode == PIX_ANIM_ONCE) {
            if (next >= count) { p.finished = true; p.time = 0.0f; break; }
        } else if (an.mode == PIX_ANIM_PINGPONG) {
            if (next >= count)  { p.dir = -1; next = count >= 2 ? count - 2 : 0; }
            else if (next < 0)  { p.dir = 1;  next = count >= 2 ? 1 : 0; p.loops++; }
        } else if (next >= count) {
            next = 0;
            p.loops++;
        }
        p.step = next;
        if (p.step < 64) p.entered |= 1ull << p.step;
        dur = pix__step_seconds(a, an, p.step);
    }
}

int pix_sprite_frame(const pix_sprite_player& p, const pix_atlas& a) {
    if (!p.playing || p.anim < 0 || p.anim >= a.anim_count) return -1;
    const pix_sprite_anim& an = a.anims[p.anim];
    int step = p.step < (int)an.count ? p.step : (int)an.count - 1;
    return step >= 0 ? a.sequence[an.first + step] : -1;
}

// ---- building ----

void pix_atlas_builder_add(pix_atlas_builder& b, const char* name, const void* rgba, int w, int h) {
    if (!rgba || w <= 0 || h <= 0 || !pix__grow(b.items, b.cap, b.count + 1)) return;
    pix__atlas_item& it = b.items[b.count];
    it.pixels = pix_new<unsigned char>((size_t)w * h * 4);
    if (!it.pixels) return;
    memcpy(it.pixels, rgba, (size_t)w * h * 4);
    it.w = w; it.h = h; it.x = it.y = 0;
    it.name = pix__pool_add(b.names, b.names_used, b.names_cap, name);
    b.count++;
}

bool pix_atlas_builder_add_file(pix_atlas_builder& b, const char* name, const char* path) {
    pix_mem_scope s = pix_scope_begin();
    int w, h;
    unsigned char* px = pix_load_image_rgba(path, &w, &h, PIX_MEM_SCOPED);
    if (px) pix_atlas_builder_add(b, name, px, w, h);
    else    pix_log("pix: atlas builder could not load '%s'", path);
    pix_scope_end(s);
    return px != 0;
}

int pix_atlas_builder_add_sheet(pix_atlas_builder& b, const char* name, const char* path, int fw, int fh,
                                float fps, int mode, int margin, int spacing) {
    if (fw <= 0 || fh <= 0) return 0;
    pix_mem_scope s = pix_scope_begin();
    int w, h;
    unsigned char* px = pix_load_image_rgba(path, &w, &h, PIX_MEM_SCOPED);
    if (!px) { pix_log("pix: atlas builder could not load '%s'", path); pix_scope_end(s); return 0; }
    unsigned char* cell = pix_new<unsigned char>((size_t)fw * fh * 4, PIX_MEM_SCOPED);
    int first = b.count, n = 0;
    char fname[96];
    for (int y = margin; y + fh <= h - margin; y += fh + spacing)
        for (int x = margin; x + fw <= w - margin; x += fw + spacing) {
            for (int r = 0; r < fh; r++) memcpy(cell + (size_t)r * fw * 4, px + ((size_t)(y + r) * w + x) * 4, (size_t)fw * 4);
            snprintf(fname, sizeof(fname), "%s_%d", name, n++);
            pix_atlas_builder_add(b, fname, cell, fw, fh);
        }
    pix_scope_end(s);
    if (fps > 0.0f && n > 0 && pix__grow(b.anims, b.anim_cap, b.anim_count + 1)) {
        pix__atlas_pending_anim& an = b.anims[b.anim_count++];
        memset(&an, 0, sizeof(an));
        strncpy(an.name, name, sizeof(an.name) - 1);
        an.first = first; an.count = n; an.fps = fps; an.mode = mode;
    }
    return n;
}

void pix_atlas_builder_free(pix_atlas_builder& b) {
    for (int i = 0; i < b.count; i++) pix_free(b.items[i].pixels);
    pix_free(b.items);
    pix_free(b.anims);
    pix_free(b.names);
    b = pix_atlas_builder();
}

// Skyline bottom-left packing: the packed area's top edge is a list of
// horizontal segments; each rectangle goes where its top would end lowest,
// then becomes part of the skyline. Tall-first ordering keeps it tight.
struct pix__skyline { int x, y, w; };

static bool pix__skyline_place(pix__skyline* sky, int& n, int width, int max_h, int w, int h, int* ox, int* oy) {
    int best = -1, best_y = 0, best_top = 0x7FFFFFFF, best_waste = 0x7FFFFFFF;
    for (int i = 0; i < n; i++) {
        if (sky[i].x + w > width) break;
        int y = 0, left = w, j = i;
        while (left > 0 && j < n) { if (sky[j].y > y) y = sky[j].y; left -= sky[j].w; j++; }
        if (left > 0 || y + h > max_h) continue;
        int waste = 0;            // the area left under the rectangle, for ties
        left = w;
        for (int k = i; k < j; k++) { int span = sky[k].w < left ? sky[k].w : left; waste += (y - sky[k].y) * span; left -= span; }
        if (y + h < best_top || (y + h == best_top && waste < best_waste)) {
            best = i; best_y = y; best_top = y + h; best_waste = waste;
        }
    }
    if (best < 0) return false;
    int x = sky[best].x;
    *ox = x; *oy = best_y;
    // the new segment, then trim what it covers
    pix__skyline seg = { x, best_y + h, w };
    int i = best;
    memmove(sky + i + 1, sky + i, (size_t)(n - i) * sizeof(pix__skyline));
    sky[i] = seg;
    n++;
    for (int k = i + 1; k < n;) {
        int over = sky[i].x + sky[i].w - sky[k].x;
        if (over <= 0) break;
        if (over >= sky[k].w) { memmove(sky + k, sky + k + 1, (size_t)(n - k - 1) * sizeof(pix__skyline)); n--; continue; }
        sky[k].x += over; sky[k].w -= over;
        break;
    }
    for (int k = 0; k + 1 < n;) {          // merge neighbours at the same height
        if (sky[k].y == sky[k + 1].y) {
            sky[k].w += sky[k + 1].w;
            memmove(sky + k + 1, sky + k + 2, (size_t)(n - k - 2) * sizeof(pix__skyline));
            n--;
        } else k++;
    }
    return true;
}

pix_atlas pix_atlas_build(pix_atlas_builder& b, idx flags, int max_size, int pad) {
    pix_atlas a = {};
    if (b.count == 0) { pix_atlas_builder_free(b); return a; }
    pix_mem_scope scope = pix_scope_begin();
    // tallest first
    int* order = pix_new<int>((size_t)b.count, PIX_MEM_SCOPED);
    long long area = 0;
    int widest = 0;
    for (int i = 0; i < b.count; i++) {
        order[i] = i;
        area += (long long)(b.items[i].w + 2 * pad) * (b.items[i].h + 2 * pad);
        if (b.items[i].w + 2 * pad > widest) widest = b.items[i].w + 2 * pad;
    }
    for (int i = 1; i < b.count; i++) {
        int v = order[i], j = i;
        while (j > 0 && b.items[order[j - 1]].h < b.items[v].h) { order[j] = order[j - 1]; j--; }
        order[j] = v;
    }
    // the narrowest power-of-two width that holds it all within max_size
    int width = 64, height = 0;
    while (width < widest || (long long)width * width < area) width *= 2;
    pix__skyline* sky = pix_new<pix__skyline>((size_t)b.count * 2 + 2, PIX_MEM_SCOPED);
    bool packed = false;
    for (; width <= max_size && !packed; width *= 2) {
        int n = 1;
        sky[0].x = 0; sky[0].y = 0; sky[0].w = width;
        packed = true;
        height = 0;
        for (int k = 0; k < b.count && packed; k++) {
            pix__atlas_item& it = b.items[order[k]];
            int x, y;
            if (!pix__skyline_place(sky, n, width, max_size, it.w + 2 * pad, it.h + 2 * pad, &x, &y)) { packed = false; break; }
            it.x = x + pad; it.y = y + pad;
            if (y + it.h + 2 * pad > height) height = y + it.h + 2 * pad;
        }
        if (packed) break;
    }
    if (!packed) {
        pix_log("pix: atlas does not fit in %dx%d", max_size, max_size);
        pix_scope_end(scope);
        pix_atlas_builder_free(b);
        return a;
    }
    height = (height + 3) & ~3;
    unsigned char* px = pix_new<unsigned char>((size_t)width * height * 4, PIX_MEM_SCOPED | PIX_MEM_ZERO);
    for (int i = 0; i < b.count; i++) {
        const pix__atlas_item& it = b.items[i];
        // the image plus its padding, the padding repeating the nearest edge
        // pixel: filtering at a frame's border then samples its own colour
        for (int y = -pad; y < it.h + pad; y++) {
            int sy = y < 0 ? 0 : (y >= it.h ? it.h - 1 : y);
            unsigned char* dst = px + ((size_t)(it.y + y) * width + (it.x - pad)) * 4;
            const unsigned char* row = it.pixels + (size_t)sy * it.w * 4;
            for (int x = -pad; x < it.w + pad; x++, dst += 4) {
                int sx = x < 0 ? 0 : (x >= it.w ? it.w - 1 : x);
                memcpy(dst, row + sx * 4, 4);
            }
        }
    }
    a.texture = pix_texture_from_pixels(px, width, height, 4, flags);
    a.owns_texture = a.texture.id != 0;
    pix_scope_end(scope);
    for (int i = 0; i < b.count; i++) {
        const pix__atlas_item& it = b.items[i];
        pix_rect src = { (float)it.x, (float)it.y, (float)it.w, (float)it.h };
        pix__atlas_add_frame(a, it.name != 0xFFFFFFFFu ? b.names + it.name : 0, src, v2(0, 0),
                             v2((float)it.w, (float)it.h), 0.0f);
    }
    for (int i = 0; i < b.anim_count; i++)
        pix_atlas_add_anim(a, b.anims[i].name, b.anims[i].first, b.anims[i].count, b.anims[i].fps, b.anims[i].mode);
    pix_atlas_builder_free(b);
    return a;
}

// ---- drawing ----

void pix_batch_sprite(pix_batch2d& b, const pix_atlas& a, int frame, vec2 pos, vec2 scale, float rot, pix_color tint,
                      int flags) {
    if (frame < 0 || frame >= a.frame_count || !a.texture.id) return;
    const pix_atlas_frame& f = a.frames[frame];
    // the trimmed rectangle relative to the pivot, in untrimmed frame space
    float x0 = f.offset.x - f.pivot.x, y0 = f.offset.y - f.pivot.y;
    float x1 = x0 + f.src.w, y1 = y0 + f.src.h;
    float iw = 1.0f / (float)a.texture.width, ih = 1.0f / (float)a.texture.height;
    float u0 = f.src.x * iw, v0 = f.src.y * ih, u1 = (f.src.x + f.src.w) * iw, v1 = (f.src.y + f.src.h) * ih;
    if (flags & PIX_FLIP_X) { float t = -x1; x1 = -x0; x0 = t; float u = u0; u0 = u1; u1 = u; }
    if (flags & PIX_FLIP_Y) { float t = -y1; y1 = -y0; y0 = t; float v = v0; v0 = v1; v1 = v; }
    vec2 c[4] = { {x0 * scale.x, y0 * scale.y}, {x1 * scale.x, y0 * scale.y},
                  {x1 * scale.x, y1 * scale.y}, {x0 * scale.x, y1 * scale.y} };
    if (rot != 0.0f) {
        float r = rot * PIX_DEG2RAD, cs = cosf(r), sn = sinf(r);
        for (int i = 0; i < 4; i++) c[i] = v2(c[i].x * cs - c[i].y * sn, c[i].x * sn + c[i].y * cs);
    }
    for (int i = 0; i < 4; i++) c[i] = v2add(c[i], pos);
    vec2 uv[4] = { {u0, v0}, {u1, v0}, {u1, v1}, {u0, v1} };
    pix_batch_quad(b, a.texture.id, c, uv, tint);
}

#endif // PIX_ATLAS_IMPL
