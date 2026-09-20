#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../core/dtype.hpp"
#include "../core/math.hpp"
#include "asset_types.hpp"

// Minimal Wavefront OBJ parser: positions/uvs/normals + polygon faces, fan
// triangulated. Vertices are emitted unshared (one index per corner) so no
// hashing is needed. Missing normals are generated from face geometry.
// No third-party code.
//
// Two things beyond a bare parser, both of which the art in this project needs:
//
// * .mtl colours. Kits split into two families - some ship a texture atlas and
//   UVs that index it (map_Kd), others ship no texture at all and carry their
//   colour purely as `Kd` per material. Rather than grow the vertex format or
//   split those models into one draw per material, an untextured material is
//   assigned a slot in a shared palette and every vertex using it gets its UV
//   rewritten to that slot's cell. One tiny generated texture then colours
//   every such model, and they all still batch together.
//
// * groups. `g name` / `o name` runs are recorded as vertex ranges, so a model
//   authored as one file with named parts (a body plus four wheels) can be
//   split into independently transformable meshes at load time.

#define OBJ_PALETTE_DIM  16                                 // cells per axis
#define OBJ_PALETTE_MAX  (OBJ_PALETTE_DIM * OBJ_PALETTE_DIM)

// Shared across every untextured model loaded into one world, so they can all
// be drawn with a single material. Build the texture with obj_palette_pixels.
struct obj_palette {
    vec3   colors[OBJ_PALETTE_MAX];
    size_t count;
};

static idx  obj_palette_slot(obj_palette& palette, vec3 color);   // dedupes
// the uv a vertex needs to sample one palette slot; generated geometry that
// wants to share the palette material asks for it directly
static vec2 obj_palette_uv(idx slot);
// RGBA8, OBJ_PALETTE_DIM x OBJ_PALETTE_DIM, ready for opengl_create_texture2d
static void obj_palette_pixels(const obj_palette& palette, unsigned char* out_rgba);

// `palette` may be null, in which case untextured materials fall back to uv 0
static bool obj_load_file(mem_arena& arena, const char* path, mesh_file_data* out,
                          obj_palette* palette = 0);

// Carves one named group out of a loaded model into a mesh of its own, with the
// vertices re-centred on the group's own centre so it can be rotated in place
// (wheels). *out_pivot receives that centre in the parent's model space.
static bool obj_extract_group(mem_arena& arena, const mesh_file_data& src, size_t group,
                              mesh_file_data* out, vec3* out_pivot);
static int  obj_find_group(const mesh_file_data& src, const char* name);

// ---------------- implementation ----------------

static const char* obj__skip_ws(const char* p) {
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

static const char* obj__next_line(const char* p) {
    while (*p && *p != '\n') p++;
    if (*p == '\n') p++;
    return p;
}

static const char* obj__read_floats(const char* p, float* out, int n) {
    for (int i = 0; i < n; i++) out[i] = strtof(p, (char**)&p);
    return p;
}

// one face vertex: "v", "v/t", "v//n" or "v/t/n"; 1-based, negatives allowed
static const char* obj__read_corner(const char* p, int* v, int* t, int* n) {
    *v = *t = *n = 0;
    p = obj__skip_ws(p);
    *v = (int)strtol(p, (char**)&p, 10);
    if (*p == '/') {
        p++;
        if (*p != '/') *t = (int)strtol(p, (char**)&p, 10);
        if (*p == '/') { p++; *n = (int)strtol(p, (char**)&p, 10); }
    }
    return p;
}

static int obj__resolve(int i, int count) { return i > 0 ? i - 1 : count + i; }

// copies the rest of the line into `dst`, trimmed
static void obj__read_token(const char* p, char* dst, size_t cap) {
    p = obj__skip_ws(p);
    size_t n = 0;
    while (*p && *p != '\n' && *p != '\r' && n + 1 < cap) dst[n++] = *p++;
    while (n && (dst[n - 1] == ' ' || dst[n - 1] == '\t')) n--;
    dst[n] = 0;
}

// ---- palette ----

static idx obj_palette_slot(obj_palette& palette, vec3 color) {
    for (size_t i = 0; i < palette.count; i++) {
        vec3 c = palette.colors[i];
        if (fabsf(c.x - color.x) < 0.002f &&
            fabsf(c.y - color.y) < 0.002f &&
            fabsf(c.z - color.z) < 0.002f) return (idx)i;
    }
    if (palette.count >= OBJ_PALETTE_MAX) return 0;
    idx id = (idx)palette.count;
    palette.colors[palette.count++] = color;
    return id;
}

static void obj_palette_pixels(const obj_palette& palette, unsigned char* out_rgba) {
    for (int i = 0; i < OBJ_PALETTE_MAX; i++) {
        vec3 c = (size_t)i < palette.count ? palette.colors[i] : v3(1.0f, 0.0f, 1.0f);
        out_rgba[i * 4 + 0] = (unsigned char)(clampf(c.x, 0.0f, 1.0f) * 255.0f + 0.5f);
        out_rgba[i * 4 + 1] = (unsigned char)(clampf(c.y, 0.0f, 1.0f) * 255.0f + 0.5f);
        out_rgba[i * 4 + 2] = (unsigned char)(clampf(c.z, 0.0f, 1.0f) * 255.0f + 0.5f);
        out_rgba[i * 4 + 3] = 255;
    }
}

// the uv that lands in the middle of a palette cell. VSHDER_BASIC flips V
// (OBJ's origin is bottom-left), so the stored V is pre-flipped to cancel it
// and the sampled texel is row `slot / DIM` of the uploaded image.
static vec2 obj_palette_uv(idx slot) {
    float col = (float)(slot % OBJ_PALETTE_DIM);
    float row = (float)(slot / OBJ_PALETTE_DIM);
    vec2 uv;
    uv.x = (col + 0.5f) / (float)OBJ_PALETTE_DIM;
    uv.y = 1.0f - (row + 0.5f) / (float)OBJ_PALETTE_DIM;
    return uv;
}

// ---- .mtl ----

#define OBJ_MAX_MATERIALS 64

struct obj__material {
    char  name[48];
    vec3  color;
    bool  textured;   // has a map_Kd, so the file's own uvs are meaningful
};

// The source text and the index tables it parses into are scratch: they are
// several times the size of the geometry that comes out, and the arena is a
// bump allocator with no way to give an interior block back. Keeping them on
// the heap means a whole asset kit costs the arena only its final vertices.
static char* obj__read_whole_file(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long bytes = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* src = (char*)malloc((size_t)bytes + 1);
    if (!src) { fclose(f); return 0; }
    size_t read = fread(src, 1, (size_t)bytes, f);
    src[read] = 0;
    fclose(f);
    return src;
}

// resolves `name` against the directory `obj_path` lives in
static void obj__sibling_path(const char* obj_path, const char* name, char* dst, size_t cap) {
    size_t cut = 0;
    for (size_t i = 0; obj_path[i]; i++)
        if (obj_path[i] == '/' || obj_path[i] == '\\') cut = i + 1;
    if (cut >= cap) cut = 0;
    memcpy(dst, obj_path, cut);
    snprintf(dst + cut, cap - cut, "%s", name);
}

static size_t obj__load_mtl(const char* obj_path, const char* mtl_name,
                            obj__material* out, size_t capacity) {
    char path[512];
    obj__sibling_path(obj_path, mtl_name, path, sizeof(path));

    char* src = obj__read_whole_file(path);
    if (!src) return 0;

    size_t count = 0;
    for (const char* p = src; *p; p = obj__next_line(p)) {
        if (strncmp(p, "newmtl", 6) == 0 && count < capacity) {
            obj__material& m = out[count++];
            obj__read_token(p + 6, m.name, sizeof(m.name));
            m.color = v3(1.0f, 1.0f, 1.0f);
            m.textured = false;
        } else if (count && p[0] == 'K' && p[1] == 'd' && p[2] == ' ') {
            float c[3];
            obj__read_floats(p + 3, c, 3);
            out[count - 1].color = v3(c[0], c[1], c[2]);
        } else if (count && strncmp(p, "map_Kd", 6) == 0) {
            out[count - 1].textured = true;
        }
    }

    free(src);
    return count;
}

static bool obj_load_file(mem_arena& arena, const char* path, mesh_file_data* out,
                          obj_palette* palette) {
    char* src = obj__read_whole_file(path);
    if (!src) return false;

    // pass 0: materials. Parsed up front so a face knows its palette uv the
    // moment it is emitted, rather than needing a fixup pass afterwards.
    obj__material materials[OBJ_MAX_MATERIALS];
    size_t material_count = 0;
    for (const char* p = src; *p; p = obj__next_line(p)) {
        if (strncmp(p, "mtllib", 6) != 0) continue;
        char name[128];
        obj__read_token(p + 6, name, sizeof(name));
        material_count = obj__load_mtl(path, name, materials, OBJ_MAX_MATERIALS);
        break;
    }

    // pass 1: count
    size_t n_pos = 0, n_uv = 0, n_nrm = 0, n_out = 0, n_groups = 0;
    for (const char* p = src; *p; p = obj__next_line(p)) {
        if (p[0] == 'v' && p[1] == ' ')      n_pos++;
        else if (p[0] == 'v' && p[1] == 't') n_uv++;
        else if (p[0] == 'v' && p[1] == 'n') n_nrm++;
        else if ((p[0] == 'g' || p[0] == 'o') && p[1] == ' ') n_groups++;
        else if (p[0] == 'f' && p[1] == ' ') {
            int corners = 0;
            const char* q = p + 2;
            while (*q && *q != '\n' && *q != '\r') {
                q = obj__skip_ws(q);
                if (!*q || *q == '\n' || *q == '\r') break;
                while (*q && *q != ' ' && *q != '\t' && *q != '\n' && *q != '\r') q++;
                corners++;
            }
            if (corners >= 3) n_out += (size_t)(corners - 2) * 3;
        }
    }

    vec3* pos = (vec3*)malloc((n_pos ? n_pos : 1) * sizeof(vec3));
    vec2* uv  = (vec2*)malloc((n_uv  ? n_uv  : 1) * sizeof(vec2));
    vec3* nrm = (vec3*)malloc((n_nrm ? n_nrm : 1) * sizeof(vec3));
    vertex*   verts = allocate<vertex>(arena, n_out ? n_out : 1);
    uint16_t* inds  = allocate<uint16_t>(arena, n_out ? n_out : 1);
    mesh_group* groups = allocate<mesh_group>(arena, n_groups ? n_groups : 1);
    if (!pos || !uv || !nrm || !verts || !inds || !groups) {
        free(src); free(pos); free(uv); free(nrm);
        return false;
    }

    // pass 2: fill
    size_t ip = 0, iu = 0, in = 0, iv = 0, ig = 0;
    bool  use_palette = false;
    vec2  palette_uv = { 0.0f, 0.0f };

    for (const char* p = src; *p; p = obj__next_line(p)) {
        if (p[0] == 'v' && p[1] == ' ') {
            float t[3]; obj__read_floats(p + 2, t, 3);
            pos[ip++] = v3(t[0], t[1], t[2]);
        } else if (p[0] == 'v' && p[1] == 't') {
            float t[2]; obj__read_floats(p + 3, t, 2);
            vec2 c = { t[0], t[1] }; uv[iu++] = c;
        } else if (p[0] == 'v' && p[1] == 'n') {
            float t[3]; obj__read_floats(p + 3, t, 3);
            nrm[in++] = v3(t[0], t[1], t[2]);
        } else if ((p[0] == 'g' || p[0] == 'o') && p[1] == ' ') {
            if (ig) groups[ig - 1].vertex_count = iv - groups[ig - 1].vertex_offset;
            mesh_group& g = groups[ig++];
            obj__read_token(p + 2, g.name, sizeof(g.name));
            g.vertex_offset = iv;
            g.vertex_count = 0;
            g.pivot = v3(0.0f, 0.0f, 0.0f);
        } else if (strncmp(p, "usemtl", 6) == 0) {
            char name[48];
            obj__read_token(p + 6, name, sizeof(name));
            use_palette = false;
            for (size_t m = 0; m < material_count; m++) {
                if (strcmp(materials[m].name, name) != 0) continue;
                if (!materials[m].textured && palette) {
                    use_palette = true;
                    palette_uv = obj_palette_uv(obj_palette_slot(*palette, materials[m].color));
                }
                break;
            }
        } else if (p[0] == 'f' && p[1] == ' ') {
            int cv[16], ct[16], cn[16], nc = 0;
            const char* q = p + 2;
            while (*q && *q != '\n' && *q != '\r' && nc < 16) {
                q = obj__skip_ws(q);
                if (!*q || *q == '\n' || *q == '\r') break;
                q = obj__read_corner(q, &cv[nc], &ct[nc], &cn[nc]);
                nc++;
            }
            for (int k = 2; k < nc; k++) {
                int tri[3] = { 0, k - 1, k };
                for (int e = 0; e < 3; e++) {
                    int c = tri[e];
                    vertex vout = {};
                    vout.position = pos[obj__resolve(cv[c], (int)n_pos)];
                    if (cn[c] != 0 && n_nrm) vout.normal = nrm[obj__resolve(cn[c], (int)n_nrm)];
                    if (use_palette)              vout.uv = palette_uv;
                    else if (ct[c] != 0 && n_uv)  vout.uv = uv[obj__resolve(ct[c], (int)n_uv)];
                    inds[iv] = (uint16_t)iv;
                    verts[iv] = vout;
                    iv++;
                }
            }
        }
    }
    if (ig) groups[ig - 1].vertex_count = iv - groups[ig - 1].vertex_offset;

    // generate flat normals for any triangle the file did not supply them for
    if (n_nrm == 0) {
        for (size_t t = 0; t + 2 < iv; t += 3) {
            vec3 n = v3norm(v3cross(v3sub(verts[t + 1].position, verts[t].position),
                                    v3sub(verts[t + 2].position, verts[t].position)));
            verts[t].normal = verts[t + 1].normal = verts[t + 2].normal = n;
        }
    }

    // each group's pivot: the centre of its bounding box, which is where a
    // part's true axis sits far more reliably than its vertex average would
    for (size_t g = 0; g < ig; g++) {
        mesh_group& grp = groups[g];
        if (!grp.vertex_count) continue;
        vec3 lo = verts[grp.vertex_offset].position, hi = lo;
        for (size_t i = 1; i < grp.vertex_count; i++) {
            vec3 p = verts[grp.vertex_offset + i].position;
            if (p.x < lo.x) lo.x = p.x; if (p.x > hi.x) hi.x = p.x;
            if (p.y < lo.y) lo.y = p.y; if (p.y > hi.y) hi.y = p.y;
            if (p.z < lo.z) lo.z = p.z; if (p.z > hi.z) hi.z = p.z;
        }
        grp.pivot = v3scale(v3add(lo, hi), 0.5f);
    }

    out->vertex_count = iv;
    out->vertex_data  = verts;
    out->index_count  = iv;
    out->index_data   = inds;
    out->group_count  = ig;
    out->groups       = groups;

    // bounds, so callers can fit a model to a lot or size a collider
    if (iv) {
        vec3 lo = verts[0].position, hi = lo;
        for (size_t i = 1; i < iv; i++) {
            vec3 p = verts[i].position;
            if (p.x < lo.x) lo.x = p.x; if (p.x > hi.x) hi.x = p.x;
            if (p.y < lo.y) lo.y = p.y; if (p.y > hi.y) hi.y = p.y;
            if (p.z < lo.z) lo.z = p.z; if (p.z > hi.z) hi.z = p.z;
        }
        out->bounds_min = lo;
        out->bounds_max = hi;
    }

    free(src); free(pos); free(uv); free(nrm);
    return true;
}

static int obj_find_group(const mesh_file_data& src, const char* name) {
    for (size_t i = 0; i < src.group_count; i++)
        if (strcmp(src.groups[i].name, name) == 0) return (int)i;
    return -1;
}

static bool obj_extract_group(mem_arena& arena, const mesh_file_data& src, size_t group,
                              mesh_file_data* out, vec3* out_pivot) {
    if (group >= src.group_count) return false;
    const mesh_group& g = src.groups[group];
    if (!g.vertex_count) return false;

    vertex*   verts = allocate<vertex>(arena, g.vertex_count);
    uint16_t* inds  = allocate<uint16_t>(arena, g.vertex_count);
    if (!verts || !inds) return false;

    vec3 lo = v3(1e30f, 1e30f, 1e30f), hi = v3(-1e30f, -1e30f, -1e30f);
    for (size_t i = 0; i < g.vertex_count; i++) {
        verts[i] = src.vertex_data[g.vertex_offset + i];
        verts[i].position = v3sub(verts[i].position, g.pivot);
        inds[i] = (uint16_t)i;
        vec3 p = verts[i].position;
        if (p.x < lo.x) lo.x = p.x; if (p.x > hi.x) hi.x = p.x;
        if (p.y < lo.y) lo.y = p.y; if (p.y > hi.y) hi.y = p.y;
        if (p.z < lo.z) lo.z = p.z; if (p.z > hi.z) hi.z = p.z;
    }

    *out = mesh_file_data();
    out->vertex_count = g.vertex_count;
    out->vertex_data  = verts;
    out->index_count  = g.vertex_count;
    out->index_data   = inds;
    out->bounds_min   = lo;
    out->bounds_max   = hi;
    if (out_pivot) *out_pivot = g.pivot;
    return true;
}
