#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "../core/dtype.hpp"
#include "../core/math.hpp"
#include "asset_types.hpp"
#include "png_loader.hpp"

// Minimal glTF 2.0 reader: .glb (binary) and .gltf with an external .bin,
// covering skinned meshes, skeletons and skeletal animation. Includes its own
// JSON tokenizer. No third-party code.
//
// Anything the file leaves out is generated: missing indices become a
// sequential list, missing normals are derived from the triangles, missing
// uvs/joints/weights get sane defaults, and weights are re-normalized.
//
// Not handled: sparse accessors, base64 `data:` uris, morph targets, and
// non-PNG embedded textures (the decoder here is PNG-only).

struct gltf_result {
    skinned_mesh_file_data    mesh;
    skeleton_file_data        skeleton;
    animation_clip_file_data* clips;
    size_t                    clip_count;
    image_file_data           image; // .data == 0 when absent or not decodable
};

static bool gltf_load_file(mem_arena& arena, const char* path, gltf_result* out);

// ---------------- json ----------------

enum { JS_OBJ = 1, JS_ARR, JS_STR, JS_NUM, JS_BOOL, JS_NULL };

struct js_tok {
    uint8_t type;
    int32_t start;  // for strings, the first char inside the quotes
    int32_t end;    // exclusive
    int32_t count;  // OBJ: member pairs, ARR: elements
    int32_t next;   // token index just past this whole subtree
};

struct js_doc {
    const char* text;  // NUL terminated
    js_tok* toks;
    int count;
    int cap;
};

static void js__ws(const char* s, int* p) {
    while (s[*p] == ' ' || s[*p] == '\t' || s[*p] == '\n' || s[*p] == '\r') (*p)++;
}

static int js__push(js_doc* d, uint8_t type, int start) {
    if (d->count >= d->cap) return -1;
    int i = d->count++;
    js_tok* t = &d->toks[i];
    t->type = type; t->start = start; t->end = start; t->count = 0; t->next = 0;
    return i;
}

// recursive descent; tokens land in document order so `next` can be filled on the way out
static int js__value(js_doc* d, int* p) {
    const char* s = d->text;
    js__ws(s, p);
    char c = s[*p];
    int me;

    if (c == '{') {
        me = js__push(d, JS_OBJ, *p);
        if (me < 0) return -1;
        (*p)++;
        js__ws(s, p);
        if (s[*p] == '}') (*p)++;
        else for (;;) {
            js__ws(s, p);
            if (s[*p] != '"' || js__value(d, p) < 0) return -1; // key
            js__ws(s, p);
            if (s[*p] != ':') return -1;
            (*p)++;
            if (js__value(d, p) < 0) return -1;                 // value
            d->toks[me].count++;
            js__ws(s, p);
            if (s[*p] == ',') { (*p)++; continue; }
            if (s[*p] == '}') { (*p)++; break; }
            return -1;
        }
    } else if (c == '[') {
        me = js__push(d, JS_ARR, *p);
        if (me < 0) return -1;
        (*p)++;
        js__ws(s, p);
        if (s[*p] == ']') (*p)++;
        else for (;;) {
            if (js__value(d, p) < 0) return -1;
            d->toks[me].count++;
            js__ws(s, p);
            if (s[*p] == ',') { (*p)++; continue; }
            if (s[*p] == ']') { (*p)++; break; }
            return -1;
        }
    } else if (c == '"') {
        (*p)++;
        int st = *p;
        while (s[*p] && s[*p] != '"') { if (s[*p] == '\\') (*p)++; (*p)++; }
        me = js__push(d, JS_STR, st);
        if (me < 0) return -1;
        d->toks[me].end = *p;
        if (s[*p] == '"') (*p)++;
    } else if (c == 't' || c == 'f' || c == 'n') {
        int st = *p;
        while (s[*p] >= 'a' && s[*p] <= 'z') (*p)++;
        me = js__push(d, c == 'n' ? JS_NULL : JS_BOOL, st);
        if (me < 0) return -1;
        d->toks[me].end = *p;
    } else {
        int st = *p;
        while (s[*p] && (s[*p] == '-' || s[*p] == '+' || s[*p] == '.' ||
                         s[*p] == 'e' || s[*p] == 'E' || (s[*p] >= '0' && s[*p] <= '9'))) (*p)++;
        if (*p == st) return -1;
        me = js__push(d, JS_NUM, st);
        if (me < 0) return -1;
        d->toks[me].end = *p;
    }

    d->toks[me].next = d->count;
    return me;
}

static bool js_is(const js_doc& d, int tok, const char* lit) {
    if (tok < 0 || d.toks[tok].type != JS_STR) return false;
    size_t n = (size_t)(d.toks[tok].end - d.toks[tok].start);
    return strlen(lit) == n && memcmp(d.text + d.toks[tok].start, lit, n) == 0;
}

static int js_get(const js_doc& d, int obj, const char* key) {
    if (obj < 0 || d.toks[obj].type != JS_OBJ) return -1;
    int k = obj + 1;
    for (int i = 0; i < d.toks[obj].count; i++) {
        int v = d.toks[k].next;
        if (js_is(d, k, key)) return v;
        k = d.toks[v].next;
    }
    return -1;
}

static int js_len(const js_doc& d, int arr) {
    return (arr >= 0 && d.toks[arr].type == JS_ARR) ? d.toks[arr].count : 0;
}

static int js_at(const js_doc& d, int arr, int i) {
    if (arr < 0 || d.toks[arr].type != JS_ARR || i < 0 || i >= d.toks[arr].count) return -1;
    int e = arr + 1;
    while (i-- > 0) e = d.toks[e].next;
    return e;
}

static float js_float(const js_doc& d, int tok, float def) {
    if (tok < 0 || d.toks[tok].type != JS_NUM) return def;
    return strtof(d.text + d.toks[tok].start, 0);
}

static int js_int(const js_doc& d, int tok, int def) {
    if (tok < 0 || d.toks[tok].type != JS_NUM) return def;
    return (int)strtol(d.text + d.toks[tok].start, 0, 10);
}

static void js_str(const js_doc& d, int tok, char* out, size_t cap) {
    out[0] = 0;
    if (tok < 0 || d.toks[tok].type != JS_STR || cap == 0) return;
    size_t n = (size_t)(d.toks[tok].end - d.toks[tok].start);
    if (n >= cap) n = cap - 1;
    memcpy(out, d.text + d.toks[tok].start, n);
    out[n] = 0;
}

// ---------------- container ----------------

struct gltf_file {
    js_doc json;
    int    root;
    const unsigned char* bin;
    size_t bin_size;
};

static unsigned char* gltf__read_whole(mem_arena& arena, const char* path, size_t* size) {
    FILE* f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char* buf = allocate<unsigned char>(arena, (size_t)n + 1);
    if (!buf) { fclose(f); return 0; }
    fread(buf, 1, (size_t)n, f);
    buf[n] = 0;
    fclose(f);
    *size = (size_t)n;
    return buf;
}

static bool gltf__open(mem_arena& arena, const char* path, gltf_file* g) {
    size_t size = 0;
    unsigned char* file = gltf__read_whole(arena, path, &size);
    if (!file || size < 4) return false;

    const char* json_text = 0;
    size_t json_len = 0;
    *g = gltf_file();

    if (memcmp(file, "glTF", 4) == 0) {           // .glb: header + JSON chunk + BIN chunk
        if (size < 20) return false;
        size_t p = 12;
        while (p + 8 <= size) {
            uint32_t clen, ctype;
            memcpy(&clen, file + p, 4);
            memcpy(&ctype, file + p + 4, 4);
            const unsigned char* cdat = file + p + 8;
            if (p + 8 + clen > size) break;
            if (ctype == 0x4E4F534A) {            // 'JSON'
                char* copy = allocate<char>(arena, clen + 1); // NUL terminate for strtof
                if (!copy) return false;
                memcpy(copy, cdat, clen);
                copy[clen] = 0;
                json_text = copy;
                json_len = clen;
            } else if (ctype == 0x004E4942) {     // 'BIN'
                g->bin = cdat;
                g->bin_size = clen;
            }
            p += 8 + clen + ((4 - (clen & 3)) & 3);
        }
    } else {                                       // .gltf text + sibling .bin
        json_text = (const char*)file;
        json_len = size;
    }
    if (!json_text) return false;

    g->json.text = json_text;
    g->json.cap  = (int)(json_len / 3) + 64;      // ~3.5 chars per token in practice
    g->json.toks = allocate<js_tok>(arena, (size_t)g->json.cap);
    if (!g->json.toks) return false;
    int pos = 0;
    g->root = js__value(&g->json, &pos);
    if (g->root < 0) return false;

    if (!g->bin) { // resolve buffers[0].uri next to the .gltf
        int buffers = js_get(g->json, g->root, "buffers");
        int uri = js_get(g->json, js_at(g->json, buffers, 0), "uri");
        if (uri < 0) return false;
        char name[260];
        js_str(g->json, uri, name, sizeof(name));
        if (!name[0] || strncmp(name, "data:", 5) == 0) return false; // base64 not supported
        char dir[260];
        strncpy(dir, path, sizeof(dir) - 1);
        dir[sizeof(dir) - 1] = 0;
        char* slash = strrchr(dir, '/');
        char* back = strrchr(dir, '\\');
        if (back > slash) slash = back;
        if (slash) slash[1] = 0; else dir[0] = 0;
        char full[520];
        snprintf(full, sizeof(full), "%s%s", dir, name);
        size_t bsize = 0;
        g->bin = gltf__read_whole(arena, full, &bsize);
        g->bin_size = bsize;
        if (!g->bin) return false;
    }
    return true;
}

// ---------------- accessors ----------------

static int gltf__comp_size(int ct) {
    switch (ct) {
        case 5120: case 5121: return 1;
        case 5122: case 5123: return 2;
        case 5125: case 5126: return 4;
    }
    return 0;
}

static int gltf__type_comps(const js_doc& d, int tok) {
    if (js_is(d, tok, "SCALAR")) return 1;
    if (js_is(d, tok, "VEC2"))   return 2;
    if (js_is(d, tok, "VEC3"))   return 3;
    if (js_is(d, tok, "VEC4"))   return 4;
    if (js_is(d, tok, "MAT2"))   return 4;
    if (js_is(d, tok, "MAT3"))   return 9;
    if (js_is(d, tok, "MAT4"))   return 16;
    return 0;
}

struct gltf__view {
    const unsigned char* base;
    size_t stride;      // bytes between elements
    size_t count;
    int    comp_type;
    int    comps;
    bool   normalized;
};

static bool gltf__accessor(const gltf_file& g, int index, gltf__view* v) {
    const js_doc& d = g.json;
    int acc = js_at(d, js_get(d, g.root, "accessors"), index);
    if (acc < 0) return false;

    int ct = js_int(d, js_get(d, acc, "componentType"), 0);
    int cs = gltf__comp_size(ct);
    int comps = gltf__type_comps(d, js_get(d, acc, "type"));
    if (!cs || !comps) return false;

    int norm = js_get(d, acc, "normalized");
    v->comp_type  = ct;
    v->comps      = comps;
    v->count      = (size_t)js_int(d, js_get(d, acc, "count"), 0);
    v->normalized = norm >= 0 && d.toks[norm].type == JS_BOOL && d.text[d.toks[norm].start] == 't';

    size_t elem = (size_t)cs * (size_t)comps;
    size_t offset = (size_t)js_int(d, js_get(d, acc, "byteOffset"), 0);

    int bv_index = js_int(d, js_get(d, acc, "bufferView"), -1);
    if (bv_index < 0) { // zero-filled accessor
        v->base = 0;
        v->stride = elem;
        return true;
    }
    int bv = js_at(d, js_get(d, g.root, "bufferViews"), bv_index);
    if (bv < 0) return false;
    if (js_int(d, js_get(d, bv, "buffer"), 0) != 0) return false; // single-buffer only

    size_t bv_off = (size_t)js_int(d, js_get(d, bv, "byteOffset"), 0);
    size_t stride = (size_t)js_int(d, js_get(d, bv, "byteStride"), 0);
    v->stride = stride ? stride : elem;           // 0/absent means tightly packed
    if (bv_off + offset > g.bin_size) return false;
    v->base = g.bin + bv_off + offset;

    size_t span = v->count ? (v->count - 1) * v->stride + elem : 0;
    if (bv_off + offset + span > g.bin_size) return false;
    return true;
}

static float gltf__comp_to_float(const unsigned char* p, int ct, bool norm) {
    switch (ct) {
        case 5126: { float f; memcpy(&f, p, 4); return f; }
        case 5121: return norm ? (float)*p / 255.0f : (float)*p;
        case 5120: { int8_t s; memcpy(&s, p, 1); float f = (float)s / 127.0f; return norm ? (f < -1.0f ? -1.0f : f) : (float)s; }
        case 5123: { uint16_t s; memcpy(&s, p, 2); return norm ? (float)s / 65535.0f : (float)s; }
        case 5122: { int16_t s; memcpy(&s, p, 2); float f = (float)s / 32767.0f; return norm ? (f < -1.0f ? -1.0f : f) : (float)s; }
        case 5125: { uint32_t s; memcpy(&s, p, 4); return (float)s; }
    }
    return 0.0f;
}

// reads `want` components per element into a tightly packed float array
static size_t gltf_read_floats(const gltf_file& g, int accessor, float* out, int want) {
    gltf__view v;
    if (!gltf__accessor(g, accessor, &v)) return 0;
    int cs = gltf__comp_size(v.comp_type);
    for (size_t i = 0; i < v.count; i++) {
        const unsigned char* e = v.base ? v.base + i * v.stride : 0;
        for (int c = 0; c < want; c++)
            out[i * want + c] = (e && c < v.comps) ? gltf__comp_to_float(e + c * cs, v.comp_type, v.normalized) : 0.0f;
    }
    return v.count;
}

static uint32_t gltf__comp_to_uint(const unsigned char* p, int ct) {
    switch (ct) {
        case 5121: return *p;
        case 5120: return (uint32_t)*(const int8_t*)p;
        case 5123: { uint16_t s; memcpy(&s, p, 2); return s; }
        case 5122: { int16_t s; memcpy(&s, p, 2); return (uint32_t)s; }
        case 5125: { uint32_t s; memcpy(&s, p, 4); return s; }
        case 5126: { float f; memcpy(&f, p, 4); return (uint32_t)f; }
    }
    return 0;
}

static size_t gltf_read_uints(const gltf_file& g, int accessor, uint32_t* out, int want) {
    gltf__view v;
    if (!gltf__accessor(g, accessor, &v)) return 0;
    int cs = gltf__comp_size(v.comp_type);
    for (size_t i = 0; i < v.count; i++) {
        const unsigned char* e = v.base ? v.base + i * v.stride : 0;
        for (int c = 0; c < want; c++)
            out[i * want + c] = (e && c < v.comps) ? gltf__comp_to_uint(e + c * cs, v.comp_type) : 0u;
    }
    return v.count;
}

static size_t gltf__accessor_count(const gltf_file& g, int accessor) {
    gltf__view v;
    return gltf__accessor(g, accessor, &v) ? v.count : 0;
}

// ---------------- nodes ----------------

static int gltf__node(const gltf_file& g, int i) {
    return js_at(g.json, js_get(g.json, g.root, "nodes"), i);
}

static void gltf__node_trs(const gltf_file& g, int node, vec3* t, quat* r, vec3* s) {
    *t = v3(0.0f, 0.0f, 0.0f);
    *r = quat_identity();
    *s = v3(1.0f, 1.0f, 1.0f);
    const js_doc& d = g.json;

    int m = js_get(d, node, "matrix");
    if (m >= 0) {
        mat4 mm;
        for (int i = 0; i < 16; i++) mm.data[i] = js_float(d, js_at(d, m, i), 0.0f);
        mat4_decompose(mm, t, r, s);
        return;
    }
    int tt = js_get(d, node, "translation");
    if (tt >= 0) *t = v3(js_float(d, js_at(d, tt, 0), 0.0f),
                          js_float(d, js_at(d, tt, 1), 0.0f),
                          js_float(d, js_at(d, tt, 2), 0.0f));
    int rr = js_get(d, node, "rotation");
    if (rr >= 0) { r->x = js_float(d, js_at(d, rr, 0), 0.0f);
                   r->y = js_float(d, js_at(d, rr, 1), 0.0f);
                   r->z = js_float(d, js_at(d, rr, 2), 0.0f);
                   r->w = js_float(d, js_at(d, rr, 3), 1.0f); }
    int ss = js_get(d, node, "scale");
    if (ss >= 0) *s = v3(js_float(d, js_at(d, ss, 0), 1.0f),
                          js_float(d, js_at(d, ss, 1), 1.0f),
                          js_float(d, js_at(d, ss, 2), 1.0f));
}

static mat4 gltf__node_matrix(const gltf_file& g, int node) {
    int m = js_get(g.json, node, "matrix");
    if (m >= 0) { // use it verbatim rather than round-tripping through TRS
        mat4 mm;
        for (int i = 0; i < 16; i++) mm.data[i] = js_float(g.json, js_at(g.json, m, i), 0.0f);
        return mm;
    }
    vec3 t, s; quat r;
    gltf__node_trs(g, node, &t, &r, &s);
    return mat4_from_trs(t, r, s);
}

// ---------------- animation sampling ----------------

struct gltf__chan {
    const float* times;
    size_t       key_count;
    const float* values;
    int          comps;
    int          val_stride;  // floats between keys (3x comps for CUBICSPLINE)
    int          val_offset;  // floats into a key to reach the value itself
    bool         step;
};

static size_t gltf__find_key(const float* times, size_t n, float t) {
    if (n < 2 || t <= times[0]) return 0;
    if (t >= times[n - 1]) return n - 1;
    size_t lo = 0, hi = n - 1;
    while (lo + 1 < hi) {
        size_t mid = (lo + hi) >> 1;
        if (times[mid] <= t) lo = mid; else hi = mid;
    }
    return lo;
}

// fraction along the segment starting at key i
static float gltf__key_blend(const gltf__chan& c, size_t i, float t) {
    if (c.step || i + 1 >= c.key_count) return 0.0f;
    float t0 = c.times[i], t1 = c.times[i + 1];
    if (t1 <= t0) return 0.0f;
    float u = (t - t0) / (t1 - t0);
    return u < 0.0f ? 0.0f : (u > 1.0f ? 1.0f : u);
}

static vec3 gltf__sample_vec3(const gltf__chan& c, float t, vec3 fallback) {
    if (!c.times || c.key_count == 0) return fallback;
    size_t i = gltf__find_key(c.times, c.key_count, t);
    const float* a = c.values + i * c.val_stride + c.val_offset;
    vec3 va = v3(a[0], a[1], a[2]);
    float u = gltf__key_blend(c, i, t);
    if (u <= 0.0f) return va;
    const float* b = c.values + (i + 1) * c.val_stride + c.val_offset;
    return v3lerp(va, v3(b[0], b[1], b[2]), u);
}

static quat gltf__sample_quat(const gltf__chan& c, float t, quat fallback) {
    if (!c.times || c.key_count == 0) return fallback;
    size_t i = gltf__find_key(c.times, c.key_count, t);
    const float* a = c.values + i * c.val_stride + c.val_offset;
    quat qa = { a[0], a[1], a[2], a[3] };
    float u = gltf__key_blend(c, i, t);
    if (u <= 0.0f) return quat_norm(qa);
    const float* b = c.values + (i + 1) * c.val_stride + c.val_offset;
    quat qb = { b[0], b[1], b[2], b[3] };
    return quat_slerp(quat_norm(qa), quat_norm(qb), u);
}

static int gltf__cmp_float(const void* a, const void* b) {
    float x = *(const float*)a, y = *(const float*)b;
    return (x > y) - (x < y);
}

// ---------------- load ----------------

static bool gltf_load_file(mem_arena& arena, const char* path, gltf_result* out) {
    *out = gltf_result();
    out->skeleton.root_transform = mat4_identity();

    gltf_file g;
    if (!gltf__open(arena, path, &g)) return false;
    const js_doc& d = g.json;

    int nodes = js_get(d, g.root, "nodes");
    int node_count = js_len(d, nodes);
    if (node_count <= 0) return false;

    // parent of every node, for walking ancestor transforms
    int32_t* parent = allocate<int32_t>(arena, (size_t)node_count);
    if (!parent) return false;
    for (int i = 0; i < node_count; i++) parent[i] = -1;
    for (int i = 0; i < node_count; i++) {
        int kids = js_get(d, gltf__node(g, i), "children");
        for (int k = 0, n = js_len(d, kids); k < n; k++) {
            int c = js_int(d, js_at(d, kids, k), -1);
            if (c >= 0 && c < node_count) parent[c] = (int32_t)i;
        }
    }

    // the node carrying the mesh; prefer one that is also skinned
    int mesh_node = -1, skin_index = -1;
    for (int i = 0; i < node_count; i++) {
        int n = gltf__node(g, i);
        if (js_get(d, n, "mesh") < 0) continue;
        int s = js_int(d, js_get(d, n, "skin"), -1);
        if (mesh_node < 0 || s >= 0) { mesh_node = i; skin_index = s; }
        if (s >= 0) break;
    }
    if (mesh_node < 0) return false;
    int mesh = js_at(d, js_get(d, g.root, "meshes"),
                     js_int(d, js_get(d, gltf__node(g, mesh_node), "mesh"), -1));
    if (mesh < 0) return false;

    // ---- skeleton ----
    int32_t* node_to_bone = allocate<int32_t>(arena, (size_t)node_count);
    if (!node_to_bone) return false;
    for (int i = 0; i < node_count; i++) node_to_bone[i] = -1;

    size_t bone_count = 0;
    int32_t* joint_to_bone = 0;   // JOINTS_0 value -> bone index, kept for the vertex remap
    int joint_count = 0;
    if (skin_index >= 0) {
        int skin = js_at(d, js_get(d, g.root, "skins"), skin_index);
        int joints = js_get(d, skin, "joints");
        joint_count = js_len(d, joints);
        if (joint_count > 0) {
            int32_t* joint_node = allocate<int32_t>(arena, (size_t)joint_count);
            for (int j = 0; j < joint_count; j++)
                joint_node[j] = (int32_t)js_int(d, js_at(d, joints, j), -1);

            // topologically sort so a bone's parent always precedes it: one linear
            // pass can then resolve a pose. `order` maps new bone -> joint slot.
            int32_t* order = allocate<int32_t>(arena, (size_t)joint_count);
            joint_to_bone = allocate<int32_t>(arena, (size_t)joint_count);
            if (!joint_node || !order || !joint_to_bone) return false;
            for (int j = 0; j < joint_count; j++) joint_to_bone[j] = -1;

            size_t emitted = 0;
            for (int guard = 0; guard <= joint_count && (int)emitted < joint_count; guard++) {
                for (int j = 0; j < joint_count; j++) {
                    if (joint_to_bone[j] >= 0) continue;
                    int pn = (joint_node[j] >= 0) ? parent[joint_node[j]] : -1;
                    int pj = -1;
                    for (int k = 0; k < joint_count; k++) if (joint_node[k] == pn) { pj = k; break; }
                    if (pj >= 0 && joint_to_bone[pj] < 0) continue; // parent not emitted yet
                    joint_to_bone[j] = (int32_t)emitted;
                    order[emitted++] = (int32_t)j;
                }
            }
            for (int j = 0; j < joint_count; j++) // cycles shouldn't happen, but never drop a joint
                if (joint_to_bone[j] < 0) { joint_to_bone[j] = (int32_t)emitted; order[emitted++] = (int32_t)j; }

            bone_count = emitted;
            bone* bones = allocate<bone>(arena, bone_count);
            if (!bones) return false;

            float* ibm = allocate<float>(arena, bone_count * 16);
            int ibm_acc = js_int(d, js_get(d, skin, "inverseBindMatrices"), -1);
            bool have_ibm = ibm && ibm_acc >= 0 && gltf_read_floats(g, ibm_acc, ibm, 16) >= bone_count;

            for (size_t b = 0; b < bone_count; b++) {
                int j = order[b];
                int nidx = joint_node[j];
                bone* bo = &bones[b];
                *bo = bone();
                bo->inverse_bind = mat4_identity();
                if (have_ibm) memcpy(bo->inverse_bind.data, ibm + (size_t)j * 16, 16 * sizeof(float));

                int pn = (nidx >= 0) ? parent[nidx] : -1;
                bo->parent = (pn >= 0) ? node_to_bone[pn] : -1;   // parents are emitted first

                if (nidx >= 0) {
                    int n = gltf__node(g, nidx);
                    gltf__node_trs(g, n, &bo->local_position, &bo->local_rotation, &bo->local_scale);
                    js_str(d, js_get(d, n, "name"), bo->name, sizeof(bo->name));
                    node_to_bone[nidx] = (int32_t)b;
                }
            }

            // everything sitting above the first root bone (unit scale, z-up fixups, ...)
            mat4 root_xf = mat4_identity();
            for (size_t b = 0; b < bone_count; b++) {
                if (bones[b].parent != -1) continue;
                for (int n = parent[joint_node[order[b]]]; n >= 0; n = parent[n])
                    root_xf = mat4_mul(gltf__node_matrix(g, gltf__node(g, n)), root_xf);
                break;
            }

            out->skeleton.bone_count = bone_count;
            out->skeleton.bones = bones;
            out->skeleton.root_transform = root_xf;
        }
    }

    // ---- skinned mesh (all primitives merged into one buffer) ----
    int prims = js_get(d, mesh, "primitives");
    int prim_count = js_len(d, prims);
    size_t total_verts = 0, total_inds = 0;
    for (int p = 0; p < prim_count; p++) {
        int prim = js_at(d, prims, p);
        if (js_int(d, js_get(d, prim, "mode"), 4) != 4) continue; // triangles only
        int pos_acc = js_int(d, js_get(d, js_get(d, prim, "attributes"), "POSITION"), -1);
        size_t vc = gltf__accessor_count(g, pos_acc);
        if (!vc) continue;
        int idx_acc = js_int(d, js_get(d, prim, "indices"), -1);
        total_verts += vc;
        total_inds  += (idx_acc >= 0) ? gltf__accessor_count(g, idx_acc) : vc;
    }
    if (total_verts == 0) return false;
    if (total_verts > 65535) return false; // renderer draws with GL_UNSIGNED_SHORT

    vertex_rigged* verts = allocate<vertex_rigged>(arena, total_verts);
    uint16_t* inds = allocate<uint16_t>(arena, total_inds ? total_inds : 1);
    if (!verts || !inds) return false;

    size_t vbase = 0, iat = 0;
    for (int p = 0; p < prim_count; p++) {
        int prim = js_at(d, prims, p);
        if (js_int(d, js_get(d, prim, "mode"), 4) != 4) continue;
        int attrs = js_get(d, prim, "attributes");
        int pos_acc = js_int(d, js_get(d, attrs, "POSITION"), -1);
        size_t vc = gltf__accessor_count(g, pos_acc);
        if (!vc) continue;

        float* tmp3 = allocate<float>(arena, vc * 4);
        uint32_t* tmpu = allocate<uint32_t>(arena, vc * 4);
        if (!tmp3 || !tmpu) return false;

        gltf_read_floats(g, pos_acc, tmp3, 3);
        for (size_t i = 0; i < vc; i++) {
            verts[vbase + i] = vertex_rigged();
            verts[vbase + i].position = v3(tmp3[i * 3], tmp3[i * 3 + 1], tmp3[i * 3 + 2]);
        }

        int nrm_acc = js_int(d, js_get(d, attrs, "NORMAL"), -1);
        bool have_normals = nrm_acc >= 0 && gltf_read_floats(g, nrm_acc, tmp3, 3) == vc;
        if (have_normals)
            for (size_t i = 0; i < vc; i++)
                verts[vbase + i].normal = v3(tmp3[i * 3], tmp3[i * 3 + 1], tmp3[i * 3 + 2]);

        int uv_acc = js_int(d, js_get(d, attrs, "TEXCOORD_0"), -1);
        if (uv_acc >= 0 && gltf_read_floats(g, uv_acc, tmp3, 2) == vc)
            for (size_t i = 0; i < vc; i++) {
                vec2 uv = { tmp3[i * 2], tmp3[i * 2 + 1] };
                verts[vbase + i].uv = uv;
            }

        int jnt_acc = js_int(d, js_get(d, attrs, "JOINTS_0"), -1);
        int wgt_acc = js_int(d, js_get(d, attrs, "WEIGHTS_0"), -1);
        bool have_skin = jnt_acc >= 0 && wgt_acc >= 0 && bone_count > 0;
        if (have_skin && gltf_read_uints(g, jnt_acc, tmpu, 4) == vc) {
            for (size_t i = 0; i < vc; i++) {
                // JOINTS_0 indexes skin.joints; the topo sort re-ordered the bones
                float r[4];
                for (int c = 0; c < 4; c++) {
                    uint32_t j = tmpu[i * 4 + c];
                    int b = (j < (uint32_t)joint_count) ? joint_to_bone[j] : -1;
                    r[c] = (float)(b >= 0 ? b : 0);
                }
                verts[vbase + i].bone_ids = v4(r[0], r[1], r[2], r[3]);
            }
        }
        if (have_skin && gltf_read_floats(g, wgt_acc, tmp3, 4) == vc) {
            for (size_t i = 0; i < vc; i++) {
                float w0 = tmp3[i * 4], w1 = tmp3[i * 4 + 1], w2 = tmp3[i * 4 + 2], w3 = tmp3[i * 4 + 3];
                float sum = w0 + w1 + w2 + w3;
                if (sum > 1e-6f) { float k = 1.0f / sum; w0 *= k; w1 *= k; w2 *= k; w3 *= k; }
                else { w0 = 1.0f; w1 = w2 = w3 = 0.0f; }   // unweighted vertex: pin it to bone 0
                verts[vbase + i].bone_weights = v4(w0, w1, w2, w3);
            }
        } else {
            for (size_t i = 0; i < vc; i++) verts[vbase + i].bone_weights = v4(1.0f, 0.0f, 0.0f, 0.0f);
        }

        int idx_acc = js_int(d, js_get(d, prim, "indices"), -1);
        size_t ic = (idx_acc >= 0) ? gltf__accessor_count(g, idx_acc) : 0;
        if (idx_acc >= 0 && ic) {
            uint32_t* tmpi = allocate<uint32_t>(arena, ic);
            if (!tmpi) return false;
            gltf_read_uints(g, idx_acc, tmpi, 1);
            for (size_t i = 0; i < ic; i++) inds[iat++] = (uint16_t)(tmpi[i] + vbase);
        } else {
            for (size_t i = 0; i < vc; i++) inds[iat++] = (uint16_t)(vbase + i); // non-indexed
        }

        // derive normals the file didn't ship, area-weighted across shared vertices
        if (!have_normals) {
            size_t first = iat - (ic ? ic : vc);
            for (size_t t = first; t + 2 < iat; t += 3) {
                vertex_rigged* a = &verts[inds[t]];
                vertex_rigged* b = &verts[inds[t + 1]];
                vertex_rigged* c = &verts[inds[t + 2]];
                vec3 n = v3cross(v3sub(b->position, a->position), v3sub(c->position, a->position));
                a->normal = v3add(a->normal, n);
                b->normal = v3add(b->normal, n);
                c->normal = v3add(c->normal, n);
            }
            for (size_t i = 0; i < vc; i++) verts[vbase + i].normal = v3norm(verts[vbase + i].normal);
        }
        vbase += vc;
    }

    out->mesh.vertex_count = vbase;
    out->mesh.vertex_data  = verts;
    out->mesh.index_count  = iat;
    out->mesh.index_data   = inds;
    out->mesh.skeleton     = bone_count ? 0 : -1;

    // ---- animation clips ----
    int anims = js_get(d, g.root, "animations");
    int anim_count = js_len(d, anims);
    if (anim_count > 0 && bone_count > 0) {
        out->clips = allocate<animation_clip_file_data>(arena, (size_t)anim_count);
        if (!out->clips) return false;

        for (int a = 0; a < anim_count; a++) {
            int anim = js_at(d, anims, a);
            int channels = js_get(d, anim, "channels");
            int samplers = js_get(d, anim, "samplers");
            int chan_count = js_len(d, channels);

            animation_clip_file_data* clip = &out->clips[out->clip_count];
            *clip = animation_clip_file_data();
            js_str(d, js_get(d, anim, "name"), clip->name, sizeof(clip->name));
            if (!clip->name[0]) snprintf(clip->name, sizeof(clip->name), "clip_%d", a);
            clip->skeleton = 0;
            clip->track_count = bone_count;
            clip->tracks = allocate<bone_animation_track>(arena, bone_count);
            if (!clip->tracks) return false;
            for (size_t b = 0; b < bone_count; b++) {
                clip->tracks[b] = bone_animation_track();
                clip->tracks[b].bone = (int32_t)b;
            }

            // gather this clip's T/R/S channel per bone
            gltf__chan* ch_t = allocate<gltf__chan>(arena, bone_count);
            gltf__chan* ch_r = allocate<gltf__chan>(arena, bone_count);
            gltf__chan* ch_s = allocate<gltf__chan>(arena, bone_count);
            if (!ch_t || !ch_r || !ch_s) return false;
            for (size_t b = 0; b < bone_count; b++) { ch_t[b] = gltf__chan(); ch_r[b] = gltf__chan(); ch_s[b] = gltf__chan(); }

            for (int c = 0; c < chan_count; c++) {
                int chan = js_at(d, channels, c);
                int target = js_get(d, chan, "target");
                int tn = js_int(d, js_get(d, target, "node"), -1);
                if (tn < 0 || tn >= node_count) continue;
                int b = node_to_bone[tn];
                if (b < 0) continue;                                   // not a joint of this skin

                int path = js_get(d, target, "path");
                int comps = js_is(d, path, "rotation") ? 4 : (js_is(d, path, "translation") || js_is(d, path, "scale")) ? 3 : 0;
                if (!comps) continue;                                  // weights/morph targets

                int smp = js_at(d, samplers, js_int(d, js_get(d, chan, "sampler"), -1));
                if (smp < 0) continue;
                int in_acc = js_int(d, js_get(d, smp, "input"), -1);
                int out_acc = js_int(d, js_get(d, smp, "output"), -1);
                size_t keys = gltf__accessor_count(g, in_acc);
                if (!keys) continue;

                int interp = js_get(d, smp, "interpolation");
                bool cubic = js_is(d, interp, "CUBICSPLINE");

                gltf__chan cc = {};
                float* times = allocate<float>(arena, keys);
                size_t vals_n = gltf__accessor_count(g, out_acc);
                float* vals = allocate<float>(arena, vals_n * (size_t)comps);
                if (!times || !vals) return false;
                gltf_read_floats(g, in_acc, times, 1);
                gltf_read_floats(g, out_acc, vals, comps);

                cc.times = times;
                cc.key_count = keys;
                cc.values = vals;
                cc.comps = comps;
                cc.val_stride = cubic ? comps * 3 : comps;  // in-tangent, value, out-tangent
                cc.val_offset = cubic ? comps : 0;          // sampled linearly off the middle value
                cc.step = js_is(d, interp, "STEP");

                if (comps == 4) ch_r[b] = cc;
                else if (js_is(d, path, "translation")) ch_t[b] = cc;
                else ch_s[b] = cc;

                float last = times[keys - 1];
                if (last > clip->duration) clip->duration = last;
            }

            // resample T/R/S onto one merged timeline per bone
            for (size_t b = 0; b < bone_count; b++) {
                size_t n = ch_t[b].key_count + ch_r[b].key_count + ch_s[b].key_count;
                if (n == 0) continue;

                float* merged = allocate<float>(arena, n);
                if (!merged) return false;
                size_t m = 0;
                for (size_t i = 0; i < ch_t[b].key_count; i++) merged[m++] = ch_t[b].times[i];
                for (size_t i = 0; i < ch_r[b].key_count; i++) merged[m++] = ch_r[b].times[i];
                for (size_t i = 0; i < ch_s[b].key_count; i++) merged[m++] = ch_s[b].times[i];
                qsort(merged, m, sizeof(float), gltf__cmp_float);

                size_t uniq = 0;
                for (size_t i = 0; i < m; i++)
                    if (i == 0 || merged[i] - merged[uniq - 1] > 1e-6f) merged[uniq++] = merged[i];

                bone_keyframe* keys = allocate<bone_keyframe>(arena, uniq);
                if (!keys) return false;
                const bone& rest = out->skeleton.bones[b];
                for (size_t i = 0; i < uniq; i++) {
                    float t = merged[i];
                    keys[i].time     = t;
                    keys[i].position = gltf__sample_vec3(ch_t[b], t, rest.local_position);
                    keys[i].rotation = gltf__sample_quat(ch_r[b], t, rest.local_rotation);
                    keys[i].scale    = gltf__sample_vec3(ch_s[b], t, rest.local_scale);
                }
                clip->tracks[b].keyframe_count = uniq;
                clip->tracks[b].keyframes = keys;
            }
            out->clip_count++;
        }
    }

    // ---- base colour texture, if it is a png we can decode ----
    int prim0 = js_at(d, prims, 0);
    int mat_index = js_int(d, js_get(d, prim0, "material"), -1);
    int mat = js_at(d, js_get(d, g.root, "materials"), mat_index);
    int tex_index = js_int(d, js_get(d, js_get(d, js_get(d, mat, "pbrMetallicRoughness"), "baseColorTexture"), "index"), -1);
    int tex = js_at(d, js_get(d, g.root, "textures"), tex_index);
    int img_index = js_int(d, js_get(d, tex, "source"), -1);
    int img = js_at(d, js_get(d, g.root, "images"), img_index);
    if (img >= 0) {
        const unsigned char* bytes = 0;
        size_t nbytes = 0;
        int bv_index = js_int(d, js_get(d, img, "bufferView"), -1);
        if (bv_index >= 0) {
            int bv = js_at(d, js_get(d, g.root, "bufferViews"), bv_index);
            size_t off = (size_t)js_int(d, js_get(d, bv, "byteOffset"), 0);
            size_t len = (size_t)js_int(d, js_get(d, bv, "byteLength"), 0);
            if (off + len <= g.bin_size) { bytes = g.bin + off; nbytes = len; }
        } else {
            int uri = js_get(d, img, "uri");
            char name[260];
            js_str(d, uri, name, sizeof(name));
            if (name[0] && strncmp(name, "data:", 5) != 0) {
                char dir[260];
                strncpy(dir, path, sizeof(dir) - 1);
                dir[sizeof(dir) - 1] = 0;
                char* slash = strrchr(dir, '/');
                char* back = strrchr(dir, '\\');
                if (back > slash) slash = back;
                if (slash) slash[1] = 0; else dir[0] = 0;
                char full[520];
                snprintf(full, sizeof(full), "%s%s", dir, name);
                size_t n = 0;
                bytes = gltf__read_whole(arena, full, &n);
                nbytes = n;
            }
        }
        // sniff rather than trust mimeType; jpeg and friends are simply skipped
        static const unsigned char png_sig[8] = { 137,80,78,71,13,10,26,10 };
        if (bytes && nbytes > 8 && memcmp(bytes, png_sig, 8) == 0) {
            png_result png = {};
            if (png_load_memory(arena, bytes, nbytes, &png)) {
                out->image.width   = png.width;
                out->image.height  = png.height;
                out->image.channel = png.channels;
                out->image.data    = (char*)png.pixels;
            }
        }
    }
    return true;
}
