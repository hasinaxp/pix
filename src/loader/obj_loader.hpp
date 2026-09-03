#pragma once
#include <stdlib.h>
#include <stdio.h>
#include "../core/dtype.hpp"
#include "../core/math.hpp"
#include "asset_types.hpp"

// Minimal Wavefront OBJ parser: positions/uvs/normals + polygon faces, fan
// triangulated. Vertices are emitted unshared (one index per corner) so no
// hashing is needed. Missing normals are generated from face geometry.
// No third-party code.

static bool obj_load_file(mem_arena& arena, const char* path, mesh_file_data* out);

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

static bool obj_load_file(mem_arena& arena, const char* path, mesh_file_data* out) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long bytes = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* src = allocate<char>(arena, (size_t)bytes + 1);
    if (!src) { fclose(f); return false; }
    fread(src, 1, (size_t)bytes, f);
    src[bytes] = 0;
    fclose(f);

    // pass 1: count
    size_t n_pos = 0, n_uv = 0, n_nrm = 0, n_out = 0;
    for (const char* p = src; *p; p = obj__next_line(p)) {
        if (p[0] == 'v' && p[1] == ' ')      n_pos++;
        else if (p[0] == 'v' && p[1] == 't') n_uv++;
        else if (p[0] == 'v' && p[1] == 'n') n_nrm++;
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

    vec3* pos = allocate<vec3>(arena, n_pos ? n_pos : 1);
    vec2* uv  = allocate<vec2>(arena, n_uv  ? n_uv  : 1);
    vec3* nrm = allocate<vec3>(arena, n_nrm ? n_nrm : 1);
    vertex*   verts = allocate<vertex>(arena, n_out ? n_out : 1);
    uint16_t* inds  = allocate<uint16_t>(arena, n_out ? n_out : 1);
    if (!verts || !inds) return false;

    // pass 2: fill
    size_t ip = 0, iu = 0, in = 0, iv = 0;
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
                    if (ct[c] != 0 && n_uv)  vout.uv     = uv[obj__resolve(ct[c], (int)n_uv)];
                    inds[iv] = (uint16_t)iv;
                    verts[iv] = vout;
                    iv++;
                }
            }
        }
    }

    // generate flat normals for any triangle the file did not supply them for
    if (n_nrm == 0) {
        for (size_t t = 0; t + 2 < iv; t += 3) {
            vec3 n = v3norm(v3cross(v3sub(verts[t + 1].position, verts[t].position),
                                    v3sub(verts[t + 2].position, verts[t].position)));
            verts[t].normal = verts[t + 1].normal = verts[t + 2].normal = n;
        }
    }

    out->vertex_count = iv;
    out->vertex_data  = verts;
    out->index_count  = iv;
    out->index_data   = inds;
    return true;
}
