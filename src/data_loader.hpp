#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "dtype.hpp"
#include "png_loader.hpp"

struct vertex {
    vec3 position;
    vec3 normal;
    vec2 uv;
};

struct mesh_file_data {
    size_t   vertex_count;
    vertex*  vertex_data;
    size_t   index_count;
    uint16_t* index_data;
};

struct image_file_data {
    int   width;
    int   height;
    int   channel;
    char* data;
};

#define MAX_MESH_FILES  256
#define MAX_IMAGE_FILES 64

struct pix_data_loader {
    mem_arena arena;
    mesh_file_data*  mesh_files;
    size_t mesh_file_count;
    image_file_data* image_files;
    size_t image_file_count;
};

static pix_data_loader pix_create_data_loader(size_t capacity = 128 * MB) {
    pix_data_loader loader = {};
    loader.arena.capacity = capacity;
    loader.arena.data = (char*)malloc(capacity);
    loader.mesh_files  = allocate<mesh_file_data>(loader.arena, MAX_MESH_FILES);
    loader.image_files = allocate<image_file_data>(loader.arena, MAX_IMAGE_FILES);
    return loader;
}

static idx load_mesh_obj_file(pix_data_loader& loader, const char* filepath);
static idx load_image_file(pix_data_loader& loader, const char* filepath, size_t channels);


// ----------- implementation --------------------

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

static idx load_mesh_obj_file(pix_data_loader& loader, const char* filepath) {
    FILE* f = fopen(filepath, "rb");
    if (!f) { return (idx)-1; }
    fseek(f, 0, SEEK_END);
    long bytes = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* src = allocate<char>(loader.arena, (size_t)bytes + 1);
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

    vec3* pos = allocate<vec3>(loader.arena, n_pos ? n_pos : 1);
    vec2* uv  = allocate<vec2>(loader.arena, n_uv  ? n_uv  : 1);
    vec3* nrm = allocate<vec3>(loader.arena, n_nrm ? n_nrm : 1);
    vertex*   verts = allocate<vertex>(loader.arena, n_out ? n_out : 1);
    uint16_t* inds  = allocate<uint16_t>(loader.arena, n_out ? n_out : 1);

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

    idx id = (idx)loader.mesh_file_count++;
    mesh_file_data* m = &loader.mesh_files[id];
    m->vertex_count = iv;
    m->vertex_data  = verts;
    m->index_count  = iv;
    m->index_data   = inds;
    return id;
}

static idx load_image_file(pix_data_loader& loader, const char* filepath, size_t channels) {
    (void)channels; // png_loader always returns RGBA8
    idx id = (idx)loader.image_file_count++;
    image_file_data* img = &loader.image_files[id];

    png_result png = {};
    if (png_load_file(loader.arena, filepath, &png)) {
        img->width   = png.width;
        img->height  = png.height;
        img->channel = png.channels;
        img->data    = (char*)png.pixels;
        return id;
    }

    // fallback: 2x2 white so the caller still gets a usable texture
    img->width = 2;
    img->height = 2;
    img->channel = 4;
    img->data = allocate<char>(loader.arena, 2 * 2 * 4);
    memset(img->data, 0xFF, 2 * 2 * 4);
    return id;
}
