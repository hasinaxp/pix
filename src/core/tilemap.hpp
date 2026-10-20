// pix - tilemaps: grids of tiles from an atlas, drawn in layers, with
// autotiling, parallax and object layers, loaded from Tiled or built in code.
//
//   pix_tilemap level = pix_load_tilemap("level1.tmj");      // a Tiled map (JSON)
//   // each frame, inside pix_begin_2d(cam) ... pix_end_2d():
//   pix_draw_tilemap(level);
//
// How it stays fast. A tile is 8 bytes on the GPU - its cell, its tile index
// and its flips - and the vertex shader turns it into a quad, looking its
// texture rectangle up in a small table. So:
//   * static tiles cost no CPU work at all per frame; editing a tile
//     re-uploads only its 32x32 chunk, and only once that chunk is on screen
//   * only chunks overlapping the view are drawn, one instanced draw each
//   * animated tiles change one table entry, never the tiles that use them
//
// Tile indices are frames of the map's tileset atlas (atlas.hpp). The map
// owns the atlas when it loaded it (pix_load_tilemap); a map created with
// pix_create_tilemap only borrows yours, which must outlive it.
#ifndef PIX_TILEMAP_HPP
#define PIX_TILEMAP_HPP
#include "dtype.hpp"
#include "math.hpp"
#include "opengl_api.hpp"
#include "texture.hpp"
#include "draw2d.hpp"
#include "atlas.hpp"
#include "../loader/json.hpp"
#include "../loader/png_loader.hpp"

#define PIX_TILE_CHUNK 32                  // chunks are 32 x 32 tiles

// flips, as Tiled stores them; combine with |
#define PIX_TILE_FLIP_X 0x80000000u        // mirrored left-right
#define PIX_TILE_FLIP_Y 0x40000000u        // mirrored top-bottom
#define PIX_TILE_FLIP_D 0x20000000u        // mirrored across the diagonal (with X or Y: a 90 degree turn)

#define PIX_LAYER_TILES   0
#define PIX_LAYER_IMAGE   1                // a parallax background image
#define PIX_LAYER_OBJECTS 2                // Tiled objects: spawn points, triggers, ...

#define PIX_MAX_TERRAINS 16                // autotile terrains 1..15 (0 = none)

// Autotiling: which tile each arrangement of same-terrain neighbours gets.
#define PIX_AUTOTILE_16 16                 // edges only: 16 tiles
#define PIX_AUTOTILE_47 47                 // edges and corners ("blob"): 47 tiles
struct pix_autotile {
    int      mode;                         // PIX_AUTOTILE_16 or _47; 0 = unset
    uint16_t table[256];                   // neighbour mask -> tile index
};

struct pix_tile_layer {
    int   kind;                            // PIX_LAYER_*
    char  name[32];
    bool  visible;
    vec2  offset;                          // world units, added to everything in the layer
    vec2  parallax;                        // scroll factor against the camera: (1, 1) moves with the world
    pix_color tint;
    // tiles: width * height cells, row by row; low 16 bits tile + 1 (0 = empty),
    // the top bits PIX_TILE_FLIP_*
    uint32_t* cells;
    uint8_t*  terrain;                     // autotile paint per cell, made on first paint
    // image layers
    pix_texture image;
    bool owns_image;                       // loaded by pix_load_tilemap (yours otherwise)
    bool repeat_x, repeat_y;
    // GPU (tile layers): each 32x32 chunk has a fixed slot in one buffer
    GLuint    buffer;
    int32_t*  chunk_slot;                  // -1 = no slot yet
    uint16_t* chunk_count;                 // tiles in it last upload
    uint8_t*  chunk_dirty;
    int       slots_used, slots_cap;
};

struct pix_map_property { uint32_t key, value; };   // offsets into pix_tilemap::strings

struct pix_map_object {
    char  name[48];
    char  type[48];                        // Tiled's "type" / "class"
    float x, y, w, h;                      // world units; Tiled puts tile objects' y at their bottom
    float rotation;                        // degrees, clockwise
    int   tile;                            // tile objects: the tile, else -1
    int   layer;                           // the object layer it came from
    int   first_property, property_count;
};

struct pix_tile_anim { int tile; int first, count; float total; int shown; };

struct pix_tilemap {
    int   width, height;                   // in tiles
    float tile_w, tile_h;                  // world units per tile
    pix_atlas tileset;
    bool  owns_tileset;
    pix_color background;                  // Tiled's background colour, if any (alpha 0 = none)
    pix_tile_layer* layers;  int layer_count, layer_cap;
    pix_map_object* objects; int object_count, object_cap;
    pix_map_property* properties; int property_count, property_cap;
    char* strings; uint32_t strings_used, strings_cap;
    pix_autotile terrains[PIX_MAX_TERRAINS];
    // animated tiles: frames and durations live in anim_frames / anim_times
    pix_tile_anim* anims; int anim_count, anim_cap;
    int*   anim_frames; float* anim_times; int anim_frame_count, anim_frame_cap;
    // GPU: the texture rectangle of every tile, as drawn right now
    GLuint rect_buffer;
    vec4*  rects;                          // live (animated entries swapped)
    int    rect_count;
    bool   rects_dirty;
};

// ---- making maps ----

// A map from a Tiled JSON export (.tmj / .json, orthogonal, not infinite).
// Every tileset it uses - image sheets or image collections, embedded or
// external .tsj - is packed into one atlas; tile animations, flips, layer
// offsets, parallax, opacity and tint, image layers and object layers with
// their properties all come across. width is 0 on failure (logged).
PIXDEF pix_tilemap pix_load_tilemap(const char* tiled_json_path, idx texture_flags = TEXTURE_PIXELATED);
// An empty map in code, `tile_w` x `tile_h` world units per tile, drawing
// from `tileset` (borrowed: keep it alive).
PIXDEF pix_tilemap pix_create_tilemap(int width, int height, float tile_w, float tile_h, const pix_atlas& tileset);
PIXDEF void        pix_unload_tilemap(pix_tilemap& map);
PIXDEF int         pix_tilemap_add_layer(pix_tilemap& map, const char* name);
// a background image (see pix_batch_parallax); the texture stays yours
PIXDEF int         pix_tilemap_add_image_layer(pix_tilemap& map, const char* name, pix_texture image,
                                               vec2 parallax, bool repeat_x, bool repeat_y);
PIXDEF int         pix_tilemap_find_layer(const pix_tilemap& map, const char* name);

// ---- tiles ----

// `tile` -1 clears the cell; `flags` PIX_TILE_FLIP_*
PIXDEF void pix_tilemap_set(pix_tilemap& map, int layer, int x, int y, int tile, uint32_t flags = 0);
PIXDEF void pix_tilemap_fill(pix_tilemap& map, int layer, int x, int y, int w, int h, int tile);
// the tile there, -1 for empty or outside the map
PIXDEF int  pix_tilemap_get(const pix_tilemap& map, int layer, int x, int y);
// The cell under a world point (ignoring layer offsets); false outside the map.
PIXDEF bool pix_tilemap_tile_at(const pix_tilemap& map, vec2 world, int* tx, int* ty);
// a cell's world rectangle
PIXDEF pix_rect pix_tilemap_tile_rect(const pix_tilemap& map, int tx, int ty);
// Makes `tile` cycle through `frames`, each shown for its duration (seconds),
// everywhere it is used. Free per tile: it changes one table entry.
PIXDEF void pix_tilemap_animate_tile(pix_tilemap& map, int tile, const int* frames, const float* seconds, int count);

// ---- autotiling ----
//
// Paint terrain instead of placing tiles: pix_tilemap_paint marks a cell as
// terrain t, and that cell and its neighbours pick the tile that fits their
// neighbours (inner corners, edges, ends...) from the rules for t.
//
// Neighbour bits: N 1, NE 2, E 4, SE 8, S 16, SW 32, W 64, NW 128. For
// PIX_AUTOTILE_16 only the edges count. For PIX_AUTOTILE_47 a corner counts
// only when both edges beside it are set - the 47 arrangements that leaves.
// pix_autotile_make lays the tiles out in ascending order of those masks from
// `first_tile` on; pix_autotile_masks lists that order, so art can be drawn to
// match (or fill pix_autotile::table yourself for any other layout).
PIXDEF pix_autotile pix_autotile_make(int mode, int first_tile);
PIXDEF int  pix_autotile_masks(int mode, uint8_t* out);         // fills up to 47 masks, returns the count
PIXDEF void pix_tilemap_set_terrain(pix_tilemap& map, int terrain, const pix_autotile& rules);
PIXDEF void pix_tilemap_paint(pix_tilemap& map, int layer, int x, int y, int terrain);   // 0 erases
PIXDEF int  pix_tilemap_terrain(const pix_tilemap& map, int layer, int x, int y);

// ---- objects ----

PIXDEF int         pix_tilemap_find_object(const pix_tilemap& map, const char* name);   // -1 if none
// a custom property of an object as text (Tiled's value, whatever its type), or null
PIXDEF const char* pix_tilemap_object_property(const pix_tilemap& map, const pix_map_object& object, const char* key);

// ---- drawing ----
// Draws through the batch's current camera, culled to what it shows; `time`
// (seconds) drives animated tiles.
PIXDEF void pix_batch_tilemap_layer(pix_batch2d& b, pix_tilemap& map, int layer, float time);
PIXDEF void pix_batch_tilemap(pix_batch2d& b, pix_tilemap& map, float time);

#endif // PIX_TILEMAP_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_TILEMAP_IMPL)
#define PIX_TILEMAP_IMPL
#include <stdio.h>
#include <math.h>

#define PIX__TILE_SSBO_BINDING 11

static const char* const PIX__TILE_VS = R"(#version 440 core
layout(location = 0) in uvec2 aTile;       // cell x | y << 16, tile | flips << 16
layout(std430, binding = 11) readonly buffer PixTileRects { vec4 rects[]; };
uniform mat4 uProj;
uniform vec2 uTileSize;
uniform vec2 uOffset;
out vec2 vUV;
void main() {
    vec2 c = vec2(float(gl_VertexID & 1), float(gl_VertexID >> 1));   // strip corners
    vec2 cell = vec2(float(aTile.x & 0xFFFFu), float(aTile.x >> 16));
    uint flips = aTile.y >> 16;
    vec4 r = rects[aTile.y & 0xFFFFu];
    // Tiled applies the diagonal flip first, then horizontal, then vertical;
    // looking a corner's texel up undoes them in reverse
    vec2 t = c;
    if ((flips & 2u) != 0u) t.y = 1.0 - t.y;
    if ((flips & 1u) != 0u) t.x = 1.0 - t.x;
    if ((flips & 4u) != 0u) t = t.yx;
    vUV = mix(r.xy, r.zw, t);
    gl_Position = uProj * vec4((cell + c) * uTileSize + uOffset, 0.0, 1.0);
}
)";

static const char* const PIX__TILE_FS = R"(#version 440 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uTex;
uniform vec4 uTint;
void main() { frag = texture(uTex, vUV) * uTint; }
)";

// one program and vertex layout for every tilemap, made on first draw
static GLuint pix__tile_prog, pix__tile_vao;
static GLint  pix__tile_u_proj, pix__tile_u_size, pix__tile_u_offset, pix__tile_u_tint, pix__tile_u_tex;

static bool pix__tile_gl(void) {
    if (pix__tile_prog) return true;
    pix__tile_prog = opengl_create_shader(PIX__TILE_VS, PIX__TILE_FS);
    if (!pix__tile_prog) return false;
    pix__tile_u_proj   = glGetUniformLocation(pix__tile_prog, "uProj");
    pix__tile_u_size   = glGetUniformLocation(pix__tile_prog, "uTileSize");
    pix__tile_u_offset = glGetUniformLocation(pix__tile_prog, "uOffset");
    pix__tile_u_tint   = glGetUniformLocation(pix__tile_prog, "uTint");
    pix__tile_u_tex    = glGetUniformLocation(pix__tile_prog, "uTex");
    glGenVertexArrays(1, &pix__tile_vao);
    glBindVertexArray(pix__tile_vao);
    glEnableVertexAttribArray(0);
    glVertexAttribDivisor(0, 1);            // one record per tile, the quad comes from gl_VertexID
    glBindVertexArray(0);
    return true;
}

static uint32_t pix__map_string(pix_tilemap& m, const char* s) {
    return pix__pool_add(m.strings, m.strings_used, m.strings_cap, s ? s : "");
}

static int pix__chunks_x(const pix_tilemap& m) { return (m.width + PIX_TILE_CHUNK - 1) / PIX_TILE_CHUNK; }
static int pix__chunks_y(const pix_tilemap& m) { return (m.height + PIX_TILE_CHUNK - 1) / PIX_TILE_CHUNK; }

static void pix__mark_dirty(pix_tilemap& m, pix_tile_layer& l, int x, int y) {
    if (l.chunk_dirty) l.chunk_dirty[(y / PIX_TILE_CHUNK) * pix__chunks_x(m) + x / PIX_TILE_CHUNK] = 1;
}

static pix_tile_layer* pix__new_layer(pix_tilemap& m, int kind, const char* name) {
    if (!pix__grow(m.layers, m.layer_cap, m.layer_count + 1)) return 0;
    pix_tile_layer& l = m.layers[m.layer_count++];
    memset(&l, 0, sizeof(l));
    l.kind = kind;
    strncpy(l.name, name ? name : "", sizeof(l.name) - 1);
    l.visible = true;
    l.parallax = v2(1, 1);
    l.tint = PIX_WHITE;
    if (kind == PIX_LAYER_TILES) {
        int chunks = pix__chunks_x(m) * pix__chunks_y(m);
        l.cells = pix_new<uint32_t>((size_t)m.width * m.height, PIX_MEM_ZERO);
        l.chunk_slot = pix_new<int32_t>((size_t)chunks);
        l.chunk_count = pix_new<uint16_t>((size_t)chunks, PIX_MEM_ZERO);
        l.chunk_dirty = pix_new<uint8_t>((size_t)chunks);
        for (int i = 0; i < chunks; i++) { l.chunk_slot[i] = -1; l.chunk_dirty[i] = 1; }
    }
    return &l;
}

// the tileset's rectangles, normalised, as the GPU table starts out
static void pix__map_rects(pix_tilemap& m) {
    pix_free(m.rects);
    m.rect_count = m.tileset.frame_count;
    m.rects = pix_new<vec4>((size_t)(m.rect_count > 0 ? m.rect_count : 1));
    float iw = m.tileset.texture.width > 0 ? 1.0f / (float)m.tileset.texture.width : 0.0f;
    float ih = m.tileset.texture.height > 0 ? 1.0f / (float)m.tileset.texture.height : 0.0f;
    for (int i = 0; i < m.rect_count; i++) {
        pix_rect s = m.tileset.frames[i].src;
        m.rects[i] = v4(s.x * iw, s.y * ih, (s.x + s.w) * iw, (s.y + s.h) * ih);
    }
    m.rects_dirty = true;
}

pix_tilemap pix_create_tilemap(int width, int height, float tile_w, float tile_h, const pix_atlas& tileset) {
    pix_tilemap m = {};
    if (width <= 0 || height <= 0 || width > 65535 || height > 65535) {
        pix_log("pix: tilemap size %dx%d (1..65535 per side)", width, height);
        return m;
    }
    m.width = width; m.height = height;
    m.tile_w = tile_w; m.tile_h = tile_h;
    m.tileset = tileset;
    m.owns_tileset = false;
    pix__map_rects(m);
    return m;
}

void pix_unload_tilemap(pix_tilemap& m) {
    for (int i = 0; i < m.layer_count; i++) {
        pix_tile_layer& l = m.layers[i];
        if (l.buffer) glDeleteBuffers(1, &l.buffer);
        if (l.owns_image) pix_unload_texture(l.image);
        pix_free(l.cells); pix_free(l.terrain);
        pix_free(l.chunk_slot); pix_free(l.chunk_count); pix_free(l.chunk_dirty);
    }
    if (m.rect_buffer) glDeleteBuffers(1, &m.rect_buffer);
    if (m.owns_tileset) pix_unload_atlas(m.tileset);
    pix_free(m.layers); pix_free(m.objects); pix_free(m.properties); pix_free(m.strings);
    pix_free(m.anims); pix_free(m.anim_frames); pix_free(m.anim_times); pix_free(m.rects);
    m = pix_tilemap();
}

int pix_tilemap_add_layer(pix_tilemap& m, const char* name) {
    return pix__new_layer(m, PIX_LAYER_TILES, name) ? m.layer_count - 1 : -1;
}

int pix_tilemap_add_image_layer(pix_tilemap& m, const char* name, pix_texture image, vec2 parallax, bool rx, bool ry) {
    pix_tile_layer* l = pix__new_layer(m, PIX_LAYER_IMAGE, name);
    if (!l) return -1;
    l->image = image;
    l->parallax = parallax;
    l->repeat_x = rx; l->repeat_y = ry;
    return m.layer_count - 1;
}

int pix_tilemap_find_layer(const pix_tilemap& m, const char* name) {
    for (int i = 0; i < m.layer_count; i++) if (strcmp(m.layers[i].name, name) == 0) return i;
    return -1;
}

static pix_tile_layer* pix__tile_layer(pix_tilemap& m, int layer) {
    if (layer < 0 || layer >= m.layer_count || m.layers[layer].kind != PIX_LAYER_TILES) return 0;
    return &m.layers[layer];
}

void pix_tilemap_set(pix_tilemap& m, int layer, int x, int y, int tile, uint32_t flags) {
    pix_tile_layer* l = pix__tile_layer(m, layer);
    if (!l || x < 0 || y < 0 || x >= m.width || y >= m.height) return;
    uint32_t v = (tile < 0 || tile > 65534) ? 0u : ((uint32_t)(tile + 1) | (flags & 0xE0000000u));
    uint32_t& cell = l->cells[(size_t)y * m.width + x];
    if (cell == v) return;
    cell = v;
    pix__mark_dirty(m, *l, x, y);
}

void pix_tilemap_fill(pix_tilemap& m, int layer, int x, int y, int w, int h, int tile) {
    for (int j = y; j < y + h; j++) for (int i = x; i < x + w; i++) pix_tilemap_set(m, layer, i, j, tile);
}

int pix_tilemap_get(const pix_tilemap& m, int layer, int x, int y) {
    if (layer < 0 || layer >= m.layer_count || m.layers[layer].kind != PIX_LAYER_TILES) return -1;
    if (x < 0 || y < 0 || x >= m.width || y >= m.height) return -1;
    return (int)(m.layers[layer].cells[(size_t)y * m.width + x] & 0xFFFFu) - 1;
}

bool pix_tilemap_tile_at(const pix_tilemap& m, vec2 w, int* tx, int* ty) {
    int x = (int)floorf(w.x / m.tile_w), y = (int)floorf(w.y / m.tile_h);
    if (tx) *tx = x;
    if (ty) *ty = y;
    return x >= 0 && y >= 0 && x < m.width && y < m.height;
}

pix_rect pix_tilemap_tile_rect(const pix_tilemap& m, int tx, int ty) {
    pix_rect r = { tx * m.tile_w, ty * m.tile_h, m.tile_w, m.tile_h };
    return r;
}

void pix_tilemap_animate_tile(pix_tilemap& m, int tile, const int* frames, const float* seconds, int count) {
    if (tile < 0 || tile >= m.rect_count || count <= 0) return;
    if (!pix__grow(m.anims, m.anim_cap, m.anim_count + 1)) return;
    int need = m.anim_frame_count + count;
    int cap_f = m.anim_frame_cap, cap_t = m.anim_frame_cap;
    if (!pix__grow(m.anim_frames, cap_f, need) || !pix__grow(m.anim_times, cap_t, need)) return;
    m.anim_frame_cap = cap_f < cap_t ? cap_f : cap_t;
    pix_tile_anim& a = m.anims[m.anim_count++];
    a.tile = tile; a.first = m.anim_frame_count; a.count = count; a.total = 0.0f; a.shown = -1;
    for (int i = 0; i < count; i++) {
        m.anim_frames[m.anim_frame_count] = frames[i];
        m.anim_times[m.anim_frame_count] = seconds[i] > 0.0f ? seconds[i] : 0.1f;
        a.total += m.anim_times[m.anim_frame_count++];
    }
}

// ---- autotiling ----

static uint8_t pix__autotile_reduce(uint8_t m) {
    // a corner only counts when both edges next to it are there
    if ((m & 1) == 0 || (m & 4) == 0)  m &= (uint8_t)~2;
    if ((m & 4) == 0 || (m & 16) == 0) m &= (uint8_t)~8;
    if ((m & 16) == 0 || (m & 64) == 0) m &= (uint8_t)~32;
    if ((m & 64) == 0 || (m & 1) == 0)  m &= (uint8_t)~128;
    return m;
}

int pix_autotile_masks(int mode, uint8_t* out) {
    int n = 0;
    if (mode == PIX_AUTOTILE_16) {
        for (int m = 0; m < 16; m++) out[n++] = (uint8_t)m;
        return n;
    }
    for (int m = 0; m < 256; m++) if (pix__autotile_reduce((uint8_t)m) == m) out[n++] = (uint8_t)m;
    return n;   // 47
}

pix_autotile pix_autotile_make(int mode, int first) {
    pix_autotile r = {};
    r.mode = mode;
    uint8_t masks[256];
    int n = pix_autotile_masks(mode, masks);
    uint16_t index_of[256] = {};
    for (int i = 0; i < n; i++) index_of[masks[i]] = (uint16_t)(first + i);
    for (int m = 0; m < 256; m++) {
        if (mode == PIX_AUTOTILE_16) r.table[m] = (uint16_t)(first + (m & 15));
        else                         r.table[m] = index_of[pix__autotile_reduce((uint8_t)m)];
    }
    return r;
}

void pix_tilemap_set_terrain(pix_tilemap& m, int terrain, const pix_autotile& rules) {
    if (terrain > 0 && terrain < PIX_MAX_TERRAINS) m.terrains[terrain] = rules;
}

int pix_tilemap_terrain(const pix_tilemap& m, int layer, int x, int y) {
    if (layer < 0 || layer >= m.layer_count || !m.layers[layer].terrain) return 0;
    if (x < 0 || y < 0 || x >= m.width || y >= m.height) return 0;
    return m.layers[layer].terrain[(size_t)y * m.width + x];
}

// re-picks one cell's tile from its painted neighbours; outside the map counts
// as the same terrain, so terrain runs cleanly off the edges
static void pix__autotile_cell(pix_tilemap& m, int layer, int x, int y) {
    pix_tile_layer& l = m.layers[layer];
    if (x < 0 || y < 0 || x >= m.width || y >= m.height) return;
    int t = l.terrain[(size_t)y * m.width + x];
    if (t == 0) return;
    const pix_autotile& rules = m.terrains[t];
    if (!rules.mode) return;
    static const int dx[8] = { 0, 1, 1, 1, 0, -1, -1, -1 }, dy[8] = { -1, -1, 0, 1, 1, 1, 0, -1 };
    uint8_t mask = 0;
    for (int k = 0; k < 8; k++) {
        int nx = x + dx[k], ny = y + dy[k];
        bool same = nx < 0 || ny < 0 || nx >= m.width || ny >= m.height || l.terrain[(size_t)ny * m.width + nx] == t;
        if (same) mask |= (uint8_t)(1u << k);
    }
    if (rules.mode == PIX_AUTOTILE_16)   // N E S W -> 1 2 4 8
        mask = (uint8_t)(((mask & 1) ? 1 : 0) | ((mask & 4) ? 2 : 0) | ((mask & 16) ? 4 : 0) | ((mask & 64) ? 8 : 0));
    pix_tilemap_set(m, layer, x, y, rules.table[mask]);
}

void pix_tilemap_paint(pix_tilemap& m, int layer, int x, int y, int terrain) {
    pix_tile_layer* l = pix__tile_layer(m, layer);
    if (!l || x < 0 || y < 0 || x >= m.width || y >= m.height || terrain < 0 || terrain >= PIX_MAX_TERRAINS) return;
    if (!l->terrain) { l->terrain = pix_new<uint8_t>((size_t)m.width * m.height, PIX_MEM_ZERO); if (!l->terrain) return; }
    l->terrain[(size_t)y * m.width + x] = (uint8_t)terrain;
    if (terrain == 0) pix_tilemap_set(m, layer, x, y, -1);
    for (int j = -1; j <= 1; j++) for (int i = -1; i <= 1; i++) pix__autotile_cell(m, layer, x + i, y + j);
}

// ---- objects ----

int pix_tilemap_find_object(const pix_tilemap& m, const char* name) {
    for (int i = 0; i < m.object_count; i++) if (strcmp(m.objects[i].name, name) == 0) return i;
    return -1;
}

const char* pix_tilemap_object_property(const pix_tilemap& m, const pix_map_object& o, const char* key) {
    for (int i = 0; i < o.property_count; i++) {
        const pix_map_property& p = m.properties[o.first_property + i];
        if (strcmp(m.strings + p.key, key) == 0) return m.strings + p.value;
    }
    return 0;
}

// ---- drawing ----

// Packs a chunk's non-empty tiles into its slot (growing the buffer when a
// new slot is needed) - the only time tiles travel to the GPU.
static void pix__upload_chunk(pix_tilemap& m, pix_tile_layer& l, int cx, int cy) {
    int ci = cy * pix__chunks_x(m) + cx;
    l.chunk_dirty[ci] = 0;
    PIX_STACK_BUFFER(uint32_t, recs, PIX_TILE_CHUNK * PIX_TILE_CHUNK * 2, PIX_TILE_CHUNK * PIX_TILE_CHUNK * 2);
    int n = 0;
    int x1 = (cx + 1) * PIX_TILE_CHUNK, y1 = (cy + 1) * PIX_TILE_CHUNK;
    if (x1 > m.width) x1 = m.width;
    if (y1 > m.height) y1 = m.height;
    for (int y = cy * PIX_TILE_CHUNK; y < y1; y++) {
        const uint32_t* row = l.cells + (size_t)y * m.width;
        for (int x = cx * PIX_TILE_CHUNK; x < x1; x++) {
            uint32_t c = row[x];
            if (!(c & 0xFFFFu)) continue;
            uint32_t flips = ((c & PIX_TILE_FLIP_X) ? 1u : 0u) | ((c & PIX_TILE_FLIP_Y) ? 2u : 0u) | ((c & PIX_TILE_FLIP_D) ? 4u : 0u);
            recs[n * 2] = (uint32_t)x | ((uint32_t)y << 16);
            recs[n * 2 + 1] = ((c & 0xFFFFu) - 1u) | (flips << 16);
            n++;
        }
    }
    l.chunk_count[ci] = (uint16_t)n;
    if (!n) return;
    const GLsizeiptr slot_bytes = PIX_TILE_CHUNK * PIX_TILE_CHUNK * 8;
    if (l.chunk_slot[ci] < 0) {
        if (l.slots_used == l.slots_cap) {
            // grow by doubling, copying what is there on the GPU
            int cap = l.slots_cap ? l.slots_cap * 2 : 8;
            GLuint grown;
            glGenBuffers(1, &grown);
            glBindBuffer(GL_COPY_WRITE_BUFFER, grown);
            glBufferData(GL_COPY_WRITE_BUFFER, slot_bytes * cap, 0, GL_STATIC_DRAW);
            if (l.buffer) {
                glBindBuffer(GL_COPY_READ_BUFFER, l.buffer);
                glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0, slot_bytes * l.slots_used);
                glDeleteBuffers(1, &l.buffer);
            }
            l.buffer = grown;
            l.slots_cap = cap;
        }
        l.chunk_slot[ci] = l.slots_used++;
    }
    glBindBuffer(GL_ARRAY_BUFFER, l.buffer);
    glBufferSubData(GL_ARRAY_BUFFER, slot_bytes * l.chunk_slot[ci], (GLsizeiptr)n * 8, recs);
}

// where the animated tiles are right now; only changed entries reach the GPU
static void pix__animate_tiles(pix_tilemap& m, float time) {
    for (int i = 0; i < m.anim_count; i++) {
        pix_tile_anim& a = m.anims[i];
        float t = fmodf(time, a.total);
        int k = 0;
        while (k < a.count - 1 && t >= m.anim_times[a.first + k]) { t -= m.anim_times[a.first + k]; k++; }
        int frame = m.anim_frames[a.first + k];
        if (frame == a.shown || frame < 0 || frame >= m.tileset.frame_count) continue;
        a.shown = frame;
        pix_rect s = m.tileset.frames[frame].src;
        float iw = 1.0f / (float)m.tileset.texture.width, ih = 1.0f / (float)m.tileset.texture.height;
        m.rects[a.tile] = v4(s.x * iw, s.y * ih, (s.x + s.w) * iw, (s.y + s.h) * ih);
        if (m.rect_buffer && !m.rects_dirty) {
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, m.rect_buffer);
            glBufferSubData(GL_SHADER_STORAGE_BUFFER, (GLintptr)a.tile * 16, 16, &m.rects[a.tile]);
        }
    }
}

void pix_batch_tilemap_layer(pix_batch2d& b, pix_tilemap& m, int layer, float time) {
    if (layer < 0 || layer >= m.layer_count) return;
    pix_tile_layer& l = m.layers[layer];
    if (!l.visible || l.kind == PIX_LAYER_OBJECTS) return;
    if (l.kind == PIX_LAYER_IMAGE) {
        pix_batch_parallax(b, l.image, l.parallax, l.offset, 1.0f, l.repeat_x, l.repeat_y, l.tint);
        return;
    }
    if (!m.tileset.texture.id || !m.rect_count || !pix__tile_gl()) return;
    pix_batch2d_flush(b);     // everything before the layer draws under it

    // the layer trails the camera by its parallax, then its own offset
    vec2 target = b.has_camera ? b.camera.target : v2(0, 0);
    vec2 shift = v2(l.offset.x + target.x * (1.0f - l.parallax.x), l.offset.y + target.y * (1.0f - l.parallax.y));
    pix_rect view = pix_batch2d_visible(b);
    int tx0 = (int)floorf((view.x - shift.x) / m.tile_w), ty0 = (int)floorf((view.y - shift.y) / m.tile_h);
    int tx1 = (int)floorf((view.x + view.w - shift.x) / m.tile_w), ty1 = (int)floorf((view.y + view.h - shift.y) / m.tile_h);
    if (tx1 < 0 || ty1 < 0 || tx0 >= m.width || ty0 >= m.height) return;
    int cx0 = tx0 < 0 ? 0 : tx0 / PIX_TILE_CHUNK, cy0 = ty0 < 0 ? 0 : ty0 / PIX_TILE_CHUNK;
    int cx1 = (tx1 >= m.width ? m.width - 1 : tx1) / PIX_TILE_CHUNK;
    int cy1 = (ty1 >= m.height ? m.height - 1 : ty1) / PIX_TILE_CHUNK;
    int chunks_x = pix__chunks_x(m);
    for (int cy = cy0; cy <= cy1; cy++)
        for (int cx = cx0; cx <= cx1; cx++)
            if (l.chunk_dirty[cy * chunks_x + cx]) pix__upload_chunk(m, l, cx, cy);
    if (!l.buffer) return;

    if (!m.rect_buffer) glGenBuffers(1, &m.rect_buffer);
    pix__animate_tiles(m, time);
    if (m.rects_dirty) {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, m.rect_buffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)m.rect_count * 16, m.rects, GL_DYNAMIC_DRAW);
        m.rects_dirty = false;
    }

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    pix__2d_apply_blend(b.blend);
    glUseProgram(pix__tile_prog);
    glUniformMatrix4fv(pix__tile_u_proj, 1, GL_FALSE, b.transform.data);
    glUniform2f(pix__tile_u_size, m.tile_w, m.tile_h);
    glUniform2f(pix__tile_u_offset, shift.x, shift.y);
    vec4 tint = pix_color_to_vec4(l.tint);
    glUniform4f(pix__tile_u_tint, tint.x, tint.y, tint.z, tint.w);
    glUniform1i(pix__tile_u_tex, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m.tileset.texture.id);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, PIX__TILE_SSBO_BINDING, m.rect_buffer);
    glBindVertexArray(pix__tile_vao);
    glBindBuffer(GL_ARRAY_BUFFER, l.buffer);
    glVertexAttribIPointer(0, 2, GL_UNSIGNED_INT, 8, 0);
    for (int cy = cy0; cy <= cy1; cy++)
        for (int cx = cx0; cx <= cx1; cx++) {
            int ci = cy * chunks_x + cx;
            if (!l.chunk_count[ci] || l.chunk_slot[ci] < 0) continue;
            glDrawArraysInstancedBaseInstance(GL_TRIANGLE_STRIP, 0, 4, l.chunk_count[ci],
                                              (GLuint)(l.chunk_slot[ci] * PIX_TILE_CHUNK * PIX_TILE_CHUNK));
            b.draws++;
        }
    glBindVertexArray(0);
}

void pix_batch_tilemap(pix_batch2d& b, pix_tilemap& m, float time) {
    for (int i = 0; i < m.layer_count; i++) pix_batch_tilemap_layer(b, m, i, time);
}

// ---- Tiled ----

// "#RRGGBB" or "#AARRGGBB"
static pix_color pix__tiled_color(const js_doc& d, int tok, pix_color def) {
    char s[16];
    js_str(d, tok, s, sizeof(s));
    if (s[0] != '#') return def;
    uint32_t v = (uint32_t)strtoul(s + 1, 0, 16);
    if (strlen(s) == 7) return pix_rgba((int)(v >> 16) & 255, (int)(v >> 8) & 255, (int)v & 255);
    if (strlen(s) == 9) return pix_rgba((int)(v >> 16) & 255, (int)(v >> 8) & 255, (int)v & 255, (int)(v >> 24) & 255);
    return def;
}

static void pix__path_join(char* out, size_t cap, const char* base_file, const char* rel) {
    const char* slash = strrchr(base_file, '/');
    const char* back = strrchr(base_file, '\\');
    if (back > slash) slash = back;
    int dir = slash ? (int)(slash - base_file) + 1 : 0;
    snprintf(out, cap, "%.*s%s", dir, base_file, rel);
}

static int pix__b64(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

// A tile layer's gids, from a plain array or base64 (raw, zlib or gzip).
static bool pix__tiled_gids(const js_doc& d, int layer, uint32_t* out, size_t count) {
    int data = js_get(d, layer, "data");
    if (js_type(d, data) == JS_ARR) {
        size_t i = 0;
        for (int e = js_first(d, data); e >= 0 && i < count; e = js_next(d, data, e))
            out[i++] = (uint32_t)strtoul(d.text + d.toks[e].start, 0, 10);
        return true;
    }
    if (js_type(d, data) != JS_STR) return false;
    // base64 straight from the JSON text, then maybe inflate
    const char* s = d.text + d.toks[data].start;
    size_t len = (size_t)(d.toks[data].end - d.toks[data].start);
    unsigned char* raw = pix_new<unsigned char>(len, PIX_MEM_SCOPED);
    size_t n = 0;
    uint32_t acc = 0;
    int bits = 0;
    for (size_t i = 0; i < len; i++) {
        int v = pix__b64(s[i]);
        if (v < 0) continue;
        acc = (acc << 6) | (uint32_t)v;
        bits += 6;
        if (bits >= 8) { bits -= 8; raw[n++] = (unsigned char)(acc >> bits); }
    }
    size_t want = count * 4;
    unsigned char* bytes = raw;
    if (js_is(d, js_get(d, layer, "compression"), "zlib") || js_is(d, js_get(d, layer, "compression"), "gzip")) {
        bytes = pix_new<unsigned char>(want, PIX_MEM_SCOPED);
        size_t got = 0;
        bool ok;
        if (js_is(d, js_get(d, layer, "compression"), "zlib")) {
            ok = zlib__inflate(raw, n, bytes, want, &got);
        } else {
            // gzip: a 10-byte header, optional fields, then raw deflate
            size_t at = 10;
            if (n < 18 || raw[0] != 0x1F || raw[1] != 0x8B) return false;
            uint8_t flg = raw[3];
            if (flg & 4) { at += 2 + (size_t)(raw[at] | (raw[at + 1] << 8)); }
            if (flg & 8) { while (at < n && raw[at]) at++; at++; }
            if (flg & 16) { while (at < n && raw[at]) at++; at++; }
            if (flg & 2) at += 2;
            inf__bits bb = {};
            bb.data = raw + at;
            bb.len = n - at;
            ok = inf__run(&bb, bytes, want, &got);
        }
        if (!ok || got != want) return false;
    } else if (js_get(d, layer, "compression") >= 0 && !js_is(d, js_get(d, layer, "compression"), "")) {
        pix_log("pix: Tiled layer compression not supported (use CSV, base64, zlib or gzip)");
        return false;
    } else if (n < want) {
        return false;
    }
    for (size_t i = 0; i < count; i++)
        out[i] = (uint32_t)bytes[i * 4] | ((uint32_t)bytes[i * 4 + 1] << 8) | ((uint32_t)bytes[i * 4 + 2] << 16) |
                 ((uint32_t)bytes[i * 4 + 3] << 24);
    return true;
}

struct pix__tiled_ctx {
    pix_tilemap* map;
    const char*  path;
    int*  gid_tile;          // gid -> tile index
    uint32_t gid_count;
    idx   flags;
};

static void pix__tiled_props(pix_tilemap& m, const js_doc& d, int props, pix_map_object& o) {
    o.first_property = m.property_count;
    o.property_count = 0;
    char key[64], value[256];
    for (int p = js_first(d, props); p >= 0; p = js_next(d, props, p)) {
        js_str(d, js_get(d, p, "name"), key, sizeof(key));
        int v = js_get(d, p, "value");
        if (js_type(d, v) == JS_STR) js_str(d, v, value, sizeof(value));
        else if (v >= 0) {
            size_t n = (size_t)(d.toks[v].end - d.toks[v].start);
            if (n >= sizeof(value)) n = sizeof(value) - 1;
            memcpy(value, d.text + d.toks[v].start, n);
            value[n] = 0;
        } else value[0] = 0;
        if (!pix__grow(m.properties, m.property_cap, m.property_count + 1)) return;
        pix_map_property& pr = m.properties[m.property_count++];
        pr.key = pix__map_string(m, key);
        pr.value = pix__map_string(m, value);
        o.property_count++;
    }
}

static void pix__tiled_layers(pix__tiled_ctx& c, const js_doc& d, int layers, vec2 offset, vec2 parallax,
                              float opacity, bool visible) {
    pix_tilemap& m = *c.map;
    char name[64], type[32], image[256], full[512];
    for (int L = js_first(d, layers); L >= 0; L = js_next(d, layers, L)) {
        js_str(d, js_get(d, L, "type"), type, sizeof(type));
        js_str(d, js_get(d, L, "name"), name, sizeof(name));
        vec2 off = v2(offset.x + js_float(d, js_get(d, L, "offsetx"), 0), offset.y + js_float(d, js_get(d, L, "offsety"), 0));
        vec2 par = v2(parallax.x * js_float(d, js_get(d, L, "parallaxx"), 1), parallax.y * js_float(d, js_get(d, L, "parallaxy"), 1));
        float op = opacity * js_float(d, js_get(d, L, "opacity"), 1.0f);
        bool vis = visible && js_bool(d, js_get(d, L, "visible"), true);
        pix_color tint = pix__tiled_color(d, js_get(d, L, "tintcolor"), PIX_WHITE);
        tint.a = (uint8_t)(tint.a * op + 0.5f);
        if (strcmp(type, "group") == 0) {
            pix__tiled_layers(c, d, js_get(d, L, "layers"), off, par, op, vis);
            continue;
        }
        pix_tile_layer* l = 0;
        if (strcmp(type, "tilelayer") == 0) {
            l = pix__new_layer(m, PIX_LAYER_TILES, name);
            if (!l) return;
            pix_mem_scope s = pix_scope_begin();
            size_t count = (size_t)m.width * m.height;
            uint32_t* gids = pix_new<uint32_t>(count, PIX_MEM_SCOPED | PIX_MEM_ZERO);
            if (!pix__tiled_gids(d, L, gids, count)) pix_log("pix: could not read the tiles of layer '%s'", name);
            for (size_t i = 0; i < count; i++) {
                uint32_t g = gids[i], id = g & 0x0FFFFFFFu;
                if (!id || id >= c.gid_count || c.gid_tile[id] < 0) continue;
                l->cells[i] = (uint32_t)(c.gid_tile[id] + 1) | (g & 0xE0000000u);
            }
            pix_scope_end(s);
        } else if (strcmp(type, "imagelayer") == 0) {
            js_str(d, js_get(d, L, "image"), image, sizeof(image));
            pix__path_join(full, sizeof(full), c.path, image);
            pix_texture t = pix_load_texture(full, c.flags);
            if (!t.id) continue;
            l = pix__new_layer(m, PIX_LAYER_IMAGE, name);
            if (!l) return;
            l->image = t;
            l->owns_image = true;
            l->repeat_x = js_bool(d, js_get(d, L, "repeatx"), false);
            l->repeat_y = js_bool(d, js_get(d, L, "repeaty"), false);
        } else if (strcmp(type, "objectgroup") == 0) {
            l = pix__new_layer(m, PIX_LAYER_OBJECTS, name);
            if (!l) return;
            int objs = js_get(d, L, "objects");
            for (int o = js_first(d, objs); o >= 0; o = js_next(d, objs, o)) {
                if (!pix__grow(m.objects, m.object_cap, m.object_count + 1)) break;
                pix_map_object& ob = m.objects[m.object_count++];
                memset(&ob, 0, sizeof(ob));
                js_str(d, js_get(d, o, "name"), ob.name, sizeof(ob.name));
                int t = js_get(d, o, "type");
                if (t < 0) t = js_get(d, o, "class");
                js_str(d, t, ob.type, sizeof(ob.type));
                ob.x = js_float(d, js_get(d, o, "x"), 0) + off.x;
                ob.y = js_float(d, js_get(d, o, "y"), 0) + off.y;
                ob.w = js_float(d, js_get(d, o, "width"), 0);
                ob.h = js_float(d, js_get(d, o, "height"), 0);
                ob.rotation = js_float(d, js_get(d, o, "rotation"), 0);
                uint32_t gid = (uint32_t)strtoul(js_get(d, o, "gid") >= 0 ? d.text + d.toks[js_get(d, o, "gid")].start : "0", 0, 10) & 0x0FFFFFFFu;
                ob.tile = gid && gid < c.gid_count ? c.gid_tile[gid] : -1;
                ob.layer = m.layer_count - 1;
                pix__tiled_props(m, d, js_get(d, o, "properties"), ob);
            }
        } else {
            continue;
        }
        l->offset = off;
        l->parallax = par;
        l->tint = tint;
        l->visible = vis;
    }
}

// a tileset (embedded, or the external .tsj it points at) into the builder
struct pix__tiled_pending { int tile_gid; int frame_gids[64]; float seconds[64]; int count; };

static void pix__tiled_tileset(pix__tiled_ctx& c, pix_atlas_builder& b, const js_doc& d, int ts, int firstgid,
                               const char* ts_path, pix__tiled_pending*& pending, int& pend_n, int& pend_cap) {
    char image[256], full[512], name[64];
    js_str(d, js_get(d, ts, "name"), name, sizeof(name));
    int tw = js_int(d, js_get(d, ts, "tilewidth"), 0), th = js_int(d, js_get(d, ts, "tileheight"), 0);
    int tilecount = js_int(d, js_get(d, ts, "tilecount"), 0);
    int tiles = js_get(d, ts, "tiles");
    if (js_get(d, ts, "image") >= 0) {
        js_str(d, js_get(d, ts, "image"), image, sizeof(image));
        pix__path_join(full, sizeof(full), ts_path, image);
        int base = b.count;
        int n = pix_atlas_builder_add_sheet(b, name, full, tw, th, 0.0f, PIX_ANIM_LOOP,
                                            js_int(d, js_get(d, ts, "margin"), 0), js_int(d, js_get(d, ts, "spacing"), 0));
        if (tilecount <= 0 || tilecount > n) tilecount = n;
        for (int i = 0; i < tilecount && (uint32_t)(firstgid + i) < c.gid_count; i++) c.gid_tile[firstgid + i] = base + i;
    } else {
        // an image collection: one picture per tile
        for (int t = js_first(d, tiles); t >= 0; t = js_next(d, tiles, t)) {
            js_str(d, js_get(d, t, "image"), image, sizeof(image));
            pix__path_join(full, sizeof(full), ts_path, image);
            int id = js_int(d, js_get(d, t, "id"), 0), at = b.count;
            if (pix_atlas_builder_add_file(b, image, full) && (uint32_t)(firstgid + id) < c.gid_count)
                c.gid_tile[firstgid + id] = at;
        }
    }
    // tile animations, resolved once every tileset is in
    for (int t = js_first(d, tiles); t >= 0; t = js_next(d, tiles, t)) {
        int anim = js_get(d, t, "animation");
        if (js_len(d, anim) == 0 || !pix__grow(pending, pend_cap, pend_n + 1)) continue;
        pix__tiled_pending& p = pending[pend_n++];
        p.tile_gid = firstgid + js_int(d, js_get(d, t, "id"), 0);
        p.count = 0;
        for (int f = js_first(d, anim); f >= 0 && p.count < 64; f = js_next(d, anim, f)) {
            p.frame_gids[p.count] = firstgid + js_int(d, js_get(d, f, "tileid"), 0);
            p.seconds[p.count++] = js_float(d, js_get(d, f, "duration"), 100.0f) * 0.001f;
        }
    }
}

pix_tilemap pix_load_tilemap(const char* path, idx flags) {
    pix_tilemap m = {};
    pix_mem_scope scope = pix_scope_begin();
    size_t size = 0;
    char* text = (char*)pix_read_file(path, &size, PIX_MEM_SCOPED);
    js_doc d;
    int root = text ? js_parse(d, text, size, PIX_MEM_SCOPED) : -1;
    if (root < 0) { pix_log("pix: could not read tilemap '%s'", path); pix_scope_end(scope); return m; }
    if (js_bool(d, js_get(d, root, "infinite"), false) ||
        (js_get(d, root, "orientation") >= 0 && !js_is(d, js_get(d, root, "orientation"), "orthogonal"))) {
        pix_log("pix: '%s' is infinite or not orthogonal; pix reads fixed-size orthogonal Tiled maps", path);
        pix_scope_end(scope);
        return m;
    }
    m.width = js_int(d, js_get(d, root, "width"), 0);
    m.height = js_int(d, js_get(d, root, "height"), 0);
    m.tile_w = js_float(d, js_get(d, root, "tilewidth"), 16);
    m.tile_h = js_float(d, js_get(d, root, "tileheight"), 16);
    m.background = pix__tiled_color(d, js_get(d, root, "backgroundcolor"), PIX_BLANK);
    if (m.width <= 0 || m.height <= 0 || m.width > 65535 || m.height > 65535) {
        pix_log("pix: tilemap '%s' has an unusable size", path);
        m = pix_tilemap();
        pix_scope_end(scope);
        return m;
    }

    // gid range: the last tileset's firstgid plus its tile count
    pix__tiled_ctx c = {};
    c.map = &m; c.path = path; c.flags = flags;
    int tilesets = js_get(d, root, "tilesets");
    uint32_t max_gid = 1;
    js_doc* ext = pix_new<js_doc>((size_t)(js_len(d, tilesets) + 1), PIX_MEM_SCOPED | PIX_MEM_ZERO);
    int* ext_root = pix_new<int>((size_t)(js_len(d, tilesets) + 1), PIX_MEM_SCOPED);
    char** ext_path = pix_new<char*>((size_t)(js_len(d, tilesets) + 1), PIX_MEM_SCOPED);
    int k = 0;
    for (int ts = js_first(d, tilesets); ts >= 0; ts = js_next(d, tilesets, ts), k++) {
        ext_root[k] = -1;
        ext_path[k] = 0;
        int firstgid = js_int(d, js_get(d, ts, "firstgid"), 1);
        int src = js_get(d, ts, "source");
        int count_tok = js_get(d, ts, "tilecount");
        if (src >= 0) {
            char rel[256];
            js_str(d, src, rel, sizeof(rel));
            ext_path[k] = pix_new<char>(512, PIX_MEM_SCOPED);
            pix__path_join(ext_path[k], 512, path, rel);
            size_t es = 0;
            char* et = (char*)pix_read_file(ext_path[k], &es, PIX_MEM_SCOPED);
            ext_root[k] = et ? js_parse(ext[k], et, es, PIX_MEM_SCOPED) : -1;
            if (ext_root[k] < 0) { pix_log("pix: could not read tileset '%s'", ext_path[k]); continue; }
            count_tok = js_get(ext[k], ext_root[k], "tilecount");
            uint32_t end = (uint32_t)(firstgid + js_int(ext[k], count_tok, 0));
            if (end > max_gid) max_gid = end;
        } else {
            uint32_t end = (uint32_t)(firstgid + js_int(d, count_tok, 0));
            int tiles = js_get(d, ts, "tiles");          // collections may skip ids
            for (int t = js_first(d, tiles); t >= 0; t = js_next(d, tiles, t)) {
                uint32_t e = (uint32_t)(firstgid + js_int(d, js_get(d, t, "id"), 0) + 1);
                if (e > end) end = e;
            }
            if (end > max_gid) max_gid = end;
        }
    }
    c.gid_count = max_gid + 1;
    c.gid_tile = pix_new<int>(c.gid_count, PIX_MEM_SCOPED);
    for (uint32_t i = 0; i < c.gid_count; i++) c.gid_tile[i] = -1;

    pix_atlas_builder b = {};
    pix__tiled_pending* pending = 0;
    int pend_n = 0, pend_cap = 0;
    k = 0;
    for (int ts = js_first(d, tilesets); ts >= 0; ts = js_next(d, tilesets, ts), k++) {
        int firstgid = js_int(d, js_get(d, ts, "firstgid"), 1);
        if (ext_path[k]) {
            if (ext_root[k] >= 0) pix__tiled_tileset(c, b, ext[k], ext_root[k], firstgid, ext_path[k], pending, pend_n, pend_cap);
        } else {
            pix__tiled_tileset(c, b, d, ts, firstgid, path, pending, pend_n, pend_cap);
        }
    }
    m.tileset = pix_atlas_build(b, flags, 8192, 1);
    m.owns_tileset = true;
    if (!m.tileset.texture.id) pix_log("pix: tilemap '%s' has no usable tileset images", path);
    pix__map_rects(m);
    for (int i = 0; i < pend_n; i++) {
        pix__tiled_pending& p = pending[i];
        if (p.tile_gid <= 0 || (uint32_t)p.tile_gid >= c.gid_count || c.gid_tile[p.tile_gid] < 0) continue;
        int frames[64];
        for (int f = 0; f < p.count; f++)
            frames[f] = (uint32_t)p.frame_gids[f] < c.gid_count ? c.gid_tile[p.frame_gids[f]] : -1;
        pix_tilemap_animate_tile(m, c.gid_tile[p.tile_gid], frames, p.seconds, p.count);
    }
    pix_free(pending);

    pix__tiled_layers(c, d, js_get(d, root, "layers"), v2(0, 0), v2(1, 1), 1.0f, true);
    pix_scope_end(scope);
    return m;
}

#endif // PIX_TILEMAP_IMPL
