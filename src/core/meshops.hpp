// pix - CPU mesh processing: weld, normals, simplification / LODs, subdivide,
// merge, transform. Works on pix_mesh_data (arrays you own), which
// load_mesh_data uploads. The arrays live in global memory unless a copy or
// merge is asked for PIX_MEM_SCOPED; every operation that replaces them later
// allocates in the same place, and pix_mesh_free does nothing for scoped ones.
//
//   pix_mesh_data m = pix_mesh_copy(loader.mesh_files[id]);
//   pix_mesh_weld(m);                               // share identical vertices
//   pix_mesh_recompute_normals(m, 50.0f);           // smooth below 50 degrees, crisp above
//   pix_mesh_data lod1 = pix_mesh_copy(m);
//   pix_mesh_simplify(lod1, 0.25f);                 // a quarter of the triangles
//   idx base = load_mesh_data(r, m.vertices, m.vertex_count, m.indices, m.index_count);
//   idx low  = load_mesh_data(r, lod1.vertices, ...);    // pick base / low by distance when drawing
#ifndef PIX_MESHOPS_HPP
#define PIX_MESHOPS_HPP
#include "dtype.hpp"
#include "math.hpp"
#include "../loader/asset_types.hpp"

struct pix_mesh_data {
    vertex*   vertices;
    uint32_t* indices;       // triangles
    size_t    vertex_count;
    size_t    index_count;
    unsigned  mem;           // PIX_MEM_* the arrays were allocated with
};

PIXDEF pix_mesh_data pix_mesh_copy(const mesh_file_data& file, unsigned mem = PIX_MEM_GLOBAL);
PIXDEF pix_mesh_data pix_mesh_copy_data(const vertex* vertices, size_t vertex_count,
                                        const uint32_t* indices, size_t index_count,
                                        unsigned mem = PIX_MEM_GLOBAL);
PIXDEF void pix_mesh_free(pix_mesh_data& mesh);

// Merges vertices that agree within the tolerances. OBJ files and most
// generators emit one vertex per triangle corner; welding typically cuts the
// vertex count by 4-6x and is what makes smooth normals and simplification
// possible at all.
PIXDEF void pix_mesh_weld(pix_mesh_data& mesh, float position_eps = 1e-5f,
                          float normal_dot = 0.999f, float uv_eps = 1e-5f);
// Normals from the geometry. Faces meeting at less than `crease_deg` share a
// smooth normal; sharper edges get split so they stay crisp. 0 = flat shading.
PIXDEF void pix_mesh_recompute_normals(pix_mesh_data& mesh, float crease_deg = 60.0f);
// Quadric-error simplification down to `ratio` of the triangles (0..1), or
// until the next collapse would move the surface more than `max_error`
// (world units). UV seams and open borders are kept intact, and collapses
// that would flip a triangle are refused. Returns the triangles left.
PIXDEF size_t pix_mesh_simplify(pix_mesh_data& mesh, float ratio, float max_error = 1e30f);
// Splits every triangle into four (midpoint subdivision), iterations times.
PIXDEF void pix_mesh_subdivide(pix_mesh_data& mesh, int iterations = 1);
PIXDEF void pix_mesh_transform(pix_mesh_data& mesh, const mat4& m);
// Tangents for normal mapping, from the uvs. load_mesh_data does this by itself
// for any vertex whose tangent is zero, so this is only for a mesh you keep on
// the CPU or edit after loading.
PIXDEF void pix_mesh_compute_tangents(pix_mesh_data& mesh);
// The same on any vertex layout that starts position, normal, uv (vertex and
// vertex_rigged both do). only_missing leaves tangents that are already set.
PIXDEF void pix_compute_tangents(void* vertices, size_t vertex_count, size_t stride, size_t tangent_offset,
                                 const uint32_t* indices, size_t index_count, bool only_missing = false);
// true if any vertex's tangent is still zero
PIXDEF bool pix_tangents_missing(const void* vertices, size_t vertex_count, size_t stride, size_t tangent_offset);
// Several meshes into one (each with its own transform, or null for identity).
PIXDEF pix_mesh_data pix_mesh_merge(const pix_mesh_data* meshes, const mat4* transforms, int count,
                                    unsigned mem = PIX_MEM_GLOBAL);
PIXDEF void pix_mesh_bounds(const pix_mesh_data& mesh, vec3* out_min, vec3* out_max);
// Drops triangles with repeated or collinear corners and vertices nothing uses.
PIXDEF void pix_mesh_cleanup(pix_mesh_data& mesh);

#endif // PIX_MESHOPS_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_MESHOPS_IMPL)
#define PIX_MESHOPS_IMPL
#include <math.h>

pix_mesh_data pix_mesh_copy_data(const vertex* v, size_t vc, const uint32_t* i, size_t ic, unsigned mem) {
    pix_mesh_data m = {};
    m.mem = mem & PIX_MEM_SCOPED;
    m.vertices = pix_new<vertex>(vc, m.mem);
    m.indices = pix_new<uint32_t>(ic, m.mem);
    if (!m.vertices || !m.indices) { pix_mesh_free(m); return m; }
    if (vc) memcpy(m.vertices, v, vc * sizeof(vertex));
    if (ic) memcpy(m.indices, i, ic * sizeof(uint32_t));
    m.vertex_count = vc;
    m.index_count = ic;
    return m;
}

pix_mesh_data pix_mesh_copy(const mesh_file_data& f, unsigned mem) {
    return pix_mesh_copy_data(f.vertex_data, f.vertex_count, f.index_data, f.index_count, mem);
}

void pix_mesh_free(pix_mesh_data& m) {
    pix_mem_free(m.vertices);   // no-op for scoped arrays
    pix_mem_free(m.indices);
    m = pix_mesh_data();
}

// ---- spatial hashing of positions ----

static uint32_t mesh__hash_cell(int x, int y, int z) {
    uint32_t h = (uint32_t)x * 73856093u ^ (uint32_t)y * 19349663u ^ (uint32_t)z * 83492791u;
    return h;
}

// A table from quantised position to a chain of vertex ids, for finding every
// vertex within `eps` of a point in O(1) instead of O(n).
struct mesh__grid {
    uint32_t* head;     // bucket -> first vertex, or ~0
    uint32_t* next;     // vertex -> next in its bucket
    uint32_t  mask;
    float     inv_cell;
};

static void mesh__grid_build(mesh__grid& g, const vertex* v, size_t n, float cell) {
    size_t buckets = 1;
    while (buckets < n * 2) buckets <<= 1;
    g.mask = (uint32_t)buckets - 1;
    g.head = (uint32_t*)malloc(buckets * sizeof(uint32_t));
    g.next = (uint32_t*)malloc((n ? n : 1) * sizeof(uint32_t));
    g.inv_cell = 1.0f / cell;
    memset(g.head, 0xFF, buckets * sizeof(uint32_t));
    for (size_t i = 0; i < n; i++) {
        const vec3& p = v[i].position;
        uint32_t b = mesh__hash_cell((int)floorf(p.x * g.inv_cell), (int)floorf(p.y * g.inv_cell),
                                     (int)floorf(p.z * g.inv_cell)) & g.mask;
        g.next[i] = g.head[b];
        g.head[b] = (uint32_t)i;
    }
}

static void mesh__grid_free(mesh__grid& g) { free(g.head); free(g.next); }

// ---- weld ----

static bool mesh__same(const vertex& a, const vertex& b, float peps, float ndot, float uveps) {
    return fabsf(a.position.x - b.position.x) <= peps && fabsf(a.position.y - b.position.y) <= peps &&
           fabsf(a.position.z - b.position.z) <= peps && v3dot(a.normal, b.normal) >= ndot &&
           fabsf(a.uv.x - b.uv.x) <= uveps && fabsf(a.uv.y - b.uv.y) <= uveps;
}

// For every vertex, the first earlier vertex it matches (or itself). Neighbours
// are searched in the 27 cells around it so points straddling a cell edge
// still find each other.
static uint32_t* mesh__match(const vertex* v, size_t n, float peps, float ndot, float uveps) {
    mesh__grid g;
    float cell = peps * 4.0f > 1e-6f ? peps * 4.0f : 1e-6f;
    mesh__grid_build(g, v, n, cell);
    uint32_t* rep = (uint32_t*)malloc((n ? n : 1) * sizeof(uint32_t));
    for (size_t i = 0; i < n; i++) {
        rep[i] = (uint32_t)i;
        const vec3& p = v[i].position;
        int cx = (int)floorf(p.x * g.inv_cell), cy = (int)floorf(p.y * g.inv_cell), cz = (int)floorf(p.z * g.inv_cell);
        uint32_t best = (uint32_t)i;
        for (int dz = -1; dz <= 1; dz++) for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) {
            uint32_t b = mesh__hash_cell(cx + dx, cy + dy, cz + dz) & g.mask;
            for (uint32_t k = g.head[b]; k != 0xFFFFFFFFu; k = g.next[k])
                if (k < best && mesh__same(v[k], v[i], peps, ndot, uveps)) best = k;
        }
        rep[i] = rep[best];   // chains collapse to the earliest representative
    }
    mesh__grid_free(g);
    return rep;
}

// Keeps the vertices `rep` maps to themselves, renumbers the indices.
static void mesh__compact(pix_mesh_data& m, const uint32_t* rep) {
    uint32_t* newid = (uint32_t*)malloc(m.vertex_count * sizeof(uint32_t));
    size_t kept = 0;
    for (size_t i = 0; i < m.vertex_count; i++) {
        if (rep[i] == i) { newid[i] = (uint32_t)kept; m.vertices[kept++] = m.vertices[i]; }
    }
    for (size_t i = 0; i < m.vertex_count; i++) if (rep[i] != i) newid[i] = newid[rep[i]];
    for (size_t i = 0; i < m.index_count; i++) m.indices[i] = newid[m.indices[i]];
    m.vertex_count = kept;
    free(newid);
}

void pix_mesh_weld(pix_mesh_data& m, float peps, float ndot, float uveps) {
    if (!m.vertex_count) return;
    uint32_t* rep = mesh__match(m.vertices, m.vertex_count, peps, ndot, uveps);
    mesh__compact(m, rep);
    free(rep);
}

void pix_mesh_cleanup(pix_mesh_data& m) {
    size_t out = 0;
    for (size_t t = 0; t + 2 < m.index_count; t += 3) {
        uint32_t a = m.indices[t], b = m.indices[t + 1], c = m.indices[t + 2];
        if (a == b || b == c || a == c) continue;
        vec3 n = v3cross(v3sub(m.vertices[b].position, m.vertices[a].position),
                         v3sub(m.vertices[c].position, m.vertices[a].position));
        if (v3dot(n, n) < 1e-20f) continue;
        m.indices[out++] = a; m.indices[out++] = b; m.indices[out++] = c;
    }
    m.index_count = out;
    uint32_t* rep = (uint32_t*)malloc((m.vertex_count ? m.vertex_count : 1) * sizeof(uint32_t));
    bool* used = (bool*)calloc(m.vertex_count ? m.vertex_count : 1, 1);
    for (size_t i = 0; i < m.index_count; i++) used[m.indices[i]] = true;
    // unused vertices map to vertex 0 of the kept set; nothing references them
    size_t first = 0;
    while (first < m.vertex_count && !used[first]) first++;
    for (size_t i = 0; i < m.vertex_count; i++) rep[i] = used[i] ? (uint32_t)i : (uint32_t)first;
    if (first < m.vertex_count) {
        // compact without the unused ones
        uint32_t* newid = (uint32_t*)malloc(m.vertex_count * sizeof(uint32_t));
        size_t kept = 0;
        for (size_t i = 0; i < m.vertex_count; i++) if (used[i]) { newid[i] = (uint32_t)kept; m.vertices[kept++] = m.vertices[i]; }
        for (size_t i = 0; i < m.index_count; i++) m.indices[i] = newid[m.indices[i]];
        m.vertex_count = kept;
        free(newid);
    }
    free(rep);
    free(used);
}

// ---- normals ----

void pix_mesh_recompute_normals(pix_mesh_data& m, float crease_deg) {
    size_t tris = m.index_count / 3;
    if (!tris) return;
    // face normals (area weighted: the raw cross product)
    vec3* fn = (vec3*)malloc(tris * sizeof(vec3));
    for (size_t t = 0; t < tris; t++) {
        const vec3& a = m.vertices[m.indices[t * 3]].position;
        const vec3& b = m.vertices[m.indices[t * 3 + 1]].position;
        const vec3& c = m.vertices[m.indices[t * 3 + 2]].position;
        fn[t] = v3cross(v3sub(b, a), v3sub(c, a));
    }
    // corners grouped by position, so faces on either side of a UV seam still smooth together
    uint32_t* pos_rep = mesh__match(m.vertices, m.vertex_count, 1e-5f, -2.0f, 1e30f);
    // position -> incident triangles (CSR)
    uint32_t* start = (uint32_t*)calloc(m.vertex_count + 1, sizeof(uint32_t));
    for (size_t i = 0; i < m.index_count; i++) start[pos_rep[m.indices[i]] + 1]++;
    for (size_t i = 0; i < m.vertex_count; i++) start[i + 1] += start[i];
    uint32_t* fill = (uint32_t*)malloc((m.vertex_count + 1) * sizeof(uint32_t));
    memcpy(fill, start, (m.vertex_count + 1) * sizeof(uint32_t));
    uint32_t* inc = (uint32_t*)malloc(m.index_count * sizeof(uint32_t));
    for (size_t i = 0; i < m.index_count; i++) inc[fill[pos_rep[m.indices[i]]]++] = (uint32_t)(i / 3);

    // every corner becomes its own vertex with the normal of the faces it may
    // smooth with; welding afterwards re-shares the ones that agree
    float cos_crease = crease_deg > 0.0f ? cosf(crease_deg * PIX_DEG2RAD) : 2.0f;
    vertex* out = pix_new<vertex>(m.index_count, m.mem);
    for (size_t i = 0; i < m.index_count; i++) {
        uint32_t v = m.indices[i], p = pos_rep[v];
        size_t t = i / 3;
        vec3 own = v3norm(fn[t]);
        vec3 sum = v3(0, 0, 0);
        for (uint32_t k = start[p]; k < start[p + 1]; k++) {
            vec3 other = fn[inc[k]];
            if (v3dot(v3norm(other), own) >= cos_crease || inc[k] == t) sum = v3add(sum, other);
        }
        out[i] = m.vertices[v];
        out[i].normal = v3norm(sum);
        m.indices[i] = (uint32_t)i;
    }
    pix_mem_free(m.vertices);
    m.vertices = out;
    m.vertex_count = m.index_count;
    free(fn); free(pos_rep); free(start); free(fill); free(inc);
    pix_mesh_weld(m, 1e-6f, 0.9999f, 1e-6f);
}

// ---- simplification ----

struct mesh__quadric { double q[10]; };   // symmetric 4x4: aa ab ac ad bb bc bd cc cd dd

static void mesh__q_add_plane(mesh__quadric& Q, double a, double b, double c, double d, double w) {
    Q.q[0] += w * a * a; Q.q[1] += w * a * b; Q.q[2] += w * a * c; Q.q[3] += w * a * d;
    Q.q[4] += w * b * b; Q.q[5] += w * b * c; Q.q[6] += w * b * d;
    Q.q[7] += w * c * c; Q.q[8] += w * c * d; Q.q[9] += w * d * d;
}

static double mesh__q_error(const mesh__quadric& Q, vec3 p) {
    double x = p.x, y = p.y, z = p.z;
    return Q.q[0] * x * x + 2 * Q.q[1] * x * y + 2 * Q.q[2] * x * z + 2 * Q.q[3] * x
         + Q.q[4] * y * y + 2 * Q.q[5] * y * z + 2 * Q.q[6] * y
         + Q.q[7] * z * z + 2 * Q.q[8] * z + Q.q[9];
}

struct mesh__collapse { double cost; uint32_t from, to; };

static int mesh__cmp_collapse(const void* a, const void* b) {
    double x = ((const mesh__collapse*)a)->cost, y = ((const mesh__collapse*)b)->cost;
    return x < y ? -1 : (x > y ? 1 : 0);
}

size_t pix_mesh_simplify(pix_mesh_data& m, float ratio, float max_error) {
    size_t tris = m.index_count / 3;
    size_t target = (size_t)((double)tris * (ratio < 0.0f ? 0.0f : (ratio > 1.0f ? 1.0f : ratio)));
    if (target < 1) target = 1;
    if (tris <= target) return tris;
    size_t nv = m.vertex_count;

    // topology lives at the position level: every attribute vertex at one
    // position moves together, and positions carrying several (UV seams) or
    // on an open border are locked in place - they may be collapsed onto,
    // never moved, which is what keeps texture seams and silhouettes intact
    uint32_t* pos = mesh__match(m.vertices, nv, 1e-6f, -2.0f, 1e30f);
    uint8_t* locked = (uint8_t*)calloc(nv, 1);
    {
        uint32_t* attrs = (uint32_t*)calloc(nv, sizeof(uint32_t));
        for (size_t i = 0; i < nv; i++) attrs[pos[i]]++;
        for (size_t i = 0; i < nv; i++) if (attrs[pos[i]] > 1) locked[pos[i]] = 1;
        free(attrs);
    }
    // open borders: an edge (in position space) used by exactly one triangle
    {
        size_t ne = m.index_count;
        uint64_t* edges = (uint64_t*)malloc(ne * sizeof(uint64_t));
        for (size_t t = 0; t < tris; t++)
            for (int e = 0; e < 3; e++) {
                uint32_t a = pos[m.indices[t * 3 + e]], b = pos[m.indices[t * 3 + (e + 1) % 3]];
                uint32_t lo = a < b ? a : b, hi = a < b ? b : a;
                edges[t * 3 + e] = ((uint64_t)lo << 32) | hi;
            }
        // sort and count duplicates
        // (simple shell sort: n log^2 n, no allocation)
        for (size_t gap = ne / 2; gap > 0; gap /= 2)
            for (size_t i = gap; i < ne; i++) {
                uint64_t v = edges[i]; size_t j = i;
                while (j >= gap && edges[j - gap] > v) { edges[j] = edges[j - gap]; j -= gap; }
                edges[j] = v;
            }
        for (size_t i = 0; i < ne; ) {
            size_t j = i + 1;
            while (j < ne && edges[j] == edges[i]) j++;
            if (j - i == 1) { locked[(uint32_t)(edges[i] >> 32)] = 1; locked[(uint32_t)edges[i]] = 1; }
            i = j;
        }
        free(edges);
    }

    mesh__quadric* Q = (mesh__quadric*)calloc(nv, sizeof(mesh__quadric));
    for (size_t t = 0; t < tris; t++) {
        vec3 a = m.vertices[m.indices[t * 3]].position, b = m.vertices[m.indices[t * 3 + 1]].position,
             c = m.vertices[m.indices[t * 3 + 2]].position;
        vec3 n = v3cross(v3sub(b, a), v3sub(c, a));
        float area = v3len(n);
        if (area < 1e-12f) continue;
        n = v3scale(n, 1.0f / area);
        double d = -(double)v3dot(n, a);
        for (int k = 0; k < 3; k++) mesh__q_add_plane(Q[pos[m.indices[t * 3 + k]]], n.x, n.y, n.z, d, area);
    }

    uint32_t* remap = (uint32_t*)malloc(nv * sizeof(uint32_t));      // attribute vertex -> attribute vertex
    uint32_t* posmap = (uint32_t*)malloc(nv * sizeof(uint32_t));     // position -> position it collapsed into
    for (size_t i = 0; i < nv; i++) { remap[i] = (uint32_t)i; posmap[i] = (uint32_t)i; }
    uint8_t* touched = (uint8_t*)malloc(nv);
    mesh__collapse* cands = (mesh__collapse*)malloc(m.index_count * sizeof(mesh__collapse));
    uint32_t* start = (uint32_t*)malloc((nv + 1) * sizeof(uint32_t));
    uint32_t* inc = (uint32_t*)malloc(m.index_count * sizeof(uint32_t));
    size_t live = tris;

    for (int pass = 0; pass < 64 && live > target; pass++) {
        // position -> incident live triangles
        memset(start, 0, (nv + 1) * sizeof(uint32_t));
        for (size_t i = 0; i < m.index_count; i++) start[pos[m.indices[i]] + 1]++;
        for (size_t i = 0; i < nv; i++) start[i + 1] += start[i];
        for (size_t i = 0; i < m.index_count; i++) inc[start[pos[m.indices[i]]]++] = (uint32_t)(i / 3);
        for (size_t i = nv; i > 0; i--) start[i] = start[i - 1];
        start[0] = 0;

        // every edge, both directions, costed
        size_t nc = 0;
        for (size_t t = 0; t < m.index_count / 3; t++)
            for (int e = 0; e < 3; e++) {
                uint32_t va = m.indices[t * 3 + e], vb = m.indices[t * 3 + (e + 1) % 3];
                uint32_t a = pos[va], b = pos[vb];
                if (a == b) continue;
                if (!locked[a]) {
                    mesh__quadric s = Q[a];
                    for (int k = 0; k < 10; k++) s.q[k] += Q[b].q[k];
                    cands[nc].cost = mesh__q_error(s, m.vertices[vb].position);
                    cands[nc].from = va; cands[nc].to = vb; nc++;
                }
            }
        qsort(cands, nc, sizeof(mesh__collapse), mesh__cmp_collapse);
        memset(touched, 0, nv);
        size_t done = 0;
        for (size_t c = 0; c < nc && live > target; c++) {
            if (cands[c].cost > (double)max_error * max_error) break;
            uint32_t va = cands[c].from, vb = cands[c].to, a = pos[va], b = pos[vb];
            if (touched[a] || touched[b] || locked[a]) continue;
            // refuse a collapse that would flip any surviving triangle around a
            vec3 target_p = m.vertices[vb].position;
            bool flips = false;
            int removed = 0;
            for (uint32_t k = start[a]; k < start[a + 1] && !flips; k++) {
                uint32_t t = inc[k];
                uint32_t* tri = m.indices + t * 3;
                uint32_t p0 = pos[tri[0]], p1 = pos[tri[1]], p2 = pos[tri[2]];
                if (p0 == p1 || p1 == p2 || p0 == p2) continue;          // already dead
                if (p0 == b || p1 == b || p2 == b) { removed++; continue; }
                vec3 v0 = m.vertices[tri[0]].position, v1 = m.vertices[tri[1]].position, v2 = m.vertices[tri[2]].position;
                vec3 before = v3cross(v3sub(v1, v0), v3sub(v2, v0));
                if (p0 == a) v0 = target_p;
                if (p1 == a) v1 = target_p;
                if (p2 == a) v2 = target_p;
                vec3 after = v3cross(v3sub(v1, v0), v3sub(v2, v0));
                if (v3dot(before, after) <= 0.2f * v3len(before) * v3len(after)) flips = true;
            }
            if (flips) continue;
            // commit: every attribute vertex at position a now uses vb's
            for (uint32_t k = start[a]; k < start[a + 1]; k++) {
                uint32_t* tri = m.indices + inc[k] * 3;
                for (int j = 0; j < 3; j++) if (pos[tri[j]] == a) tri[j] = vb;
                for (int j = 0; j < 3; j++) touched[pos[tri[j]]] = 1;
            }
            for (int k = 0; k < 10; k++) Q[b].q[k] += Q[a].q[k];
            touched[a] = touched[b] = 1;
            live -= (size_t)removed;
            done++;
        }
        // drop dead triangles
        size_t out = 0;
        for (size_t t = 0; t + 2 < m.index_count; t += 3) {
            uint32_t x = m.indices[t], y = m.indices[t + 1], z = m.indices[t + 2];
            if (pos[x] == pos[y] || pos[y] == pos[z] || pos[x] == pos[z]) continue;
            m.indices[out++] = x; m.indices[out++] = y; m.indices[out++] = z;
        }
        m.index_count = out;
        live = out / 3;
        if (!done) break;
    }
    free(pos); free(locked); free(Q); free(remap); free(posmap); free(touched); free(cands); free(start); free(inc);
    pix_mesh_cleanup(m);
    return m.index_count / 3;
}

// ---- subdivide / transform / merge / bounds ----

void pix_mesh_subdivide(pix_mesh_data& m, int iterations) {
    for (int it = 0; it < iterations; it++) {
        size_t tris = m.index_count / 3;
        vertex* nv = pix_new<vertex>(m.vertex_count + m.index_count, m.mem);
        uint32_t* ni = pix_new<uint32_t>(tris * 12, m.mem);
        if (!nv || !ni) { pix_mem_free(nv); pix_mem_free(ni); return; }
        memcpy(nv, m.vertices, m.vertex_count * sizeof(vertex));
        size_t vc = m.vertex_count, ic = 0;
        for (size_t t = 0; t < tris; t++) {
            uint32_t c[3] = { m.indices[t * 3], m.indices[t * 3 + 1], m.indices[t * 3 + 2] };
            uint32_t mid[3];
            for (int e = 0; e < 3; e++) {
                const vertex& a = m.vertices[c[e]];
                const vertex& b = m.vertices[c[(e + 1) % 3]];
                vertex v;
                v.position = v3scale(v3add(a.position, b.position), 0.5f);
                v.normal = v3norm(v3add(a.normal, b.normal));
                v.uv = v2scale(v2add(a.uv, b.uv), 0.5f);
                vec3 t = v3add(v3(a.tangent.x, a.tangent.y, a.tangent.z), v3(b.tangent.x, b.tangent.y, b.tangent.z));
                t = v3dot(t, t) > 1e-12f ? v3norm(t) : t;   // zero stays zero: derived again at load
                v.tangent = v4(t.x, t.y, t.z, a.tangent.w);
                mid[e] = (uint32_t)vc;
                nv[vc++] = v;
            }
            uint32_t q[12] = { c[0], mid[0], mid[2],  mid[0], c[1], mid[1],  mid[2], mid[1], c[2],  mid[0], mid[1], mid[2] };
            memcpy(ni + ic, q, sizeof(q));
            ic += 12;
        }
        pix_mem_free(m.vertices); pix_mem_free(m.indices);
        m.vertices = nv; m.indices = ni;
        m.vertex_count = vc; m.index_count = ic;
        pix_mesh_weld(m);   // midpoints shared by two triangles become one vertex again
    }
}

void pix_mesh_transform(pix_mesh_data& m, const mat4& t) {
    // normals go through the inverse transpose, so non-uniform scale keeps them perpendicular
    mat4 nt = mat4_transpose(mat4_inverse(t));
    // tangents follow the surface (the plain transform); a mirroring transform
    // flips the bitangent's side, so the handedness sign flips with it
    const float* d = t.data;
    float det = d[0] * (d[5] * d[10] - d[9] * d[6]) - d[4] * (d[1] * d[10] - d[9] * d[2])
              + d[8] * (d[1] * d[6] - d[5] * d[2]);
    float flip = det < 0.0f ? -1.0f : 1.0f;
    for (size_t i = 0; i < m.vertex_count; i++) {
        vertex& v = m.vertices[i];
        v.position = mat4_mul_point(t, v.position);
        v.normal = v3norm(mat4_mul_dir(nt, v.normal));
        vec3 tg = mat4_mul_dir(t, v3(v.tangent.x, v.tangent.y, v.tangent.z));
        if (v3dot(tg, tg) > 1e-12f) tg = v3norm(tg);
        v.tangent = v4(tg.x, tg.y, tg.z, v.tangent.w * flip);
    }
}

// Lengyel's method: each triangle's direction of increasing u (and v) in
// space, summed per vertex, then made perpendicular to the normal.
void pix_compute_tangents(void* vertices, size_t vertex_count, size_t stride, size_t tangent_offset,
                          const uint32_t* indices, size_t index_count, bool only_missing) {
    if (!vertex_count) return;
    vec3* acc = pix_alloc<vec3>(vertex_count * 2);   // sum of u directions, then of v directions
    if (!acc) return;
    vec3* sdir = acc;
    vec3* tdir = acc + vertex_count;
    char* base = (char*)vertices;
    #define PIX__VPOS(i) (*(vec3*)(base + (i) * stride))
    #define PIX__VNRM(i) (*(vec3*)(base + (i) * stride + 12))
    #define PIX__VUV(i)  (*(vec2*)(base + (i) * stride + 24))
    #define PIX__VTAN(i) (*(vec4*)(base + (i) * stride + tangent_offset))
    for (size_t t = 0; t + 2 < index_count; t += 3) {
        uint32_t i0 = indices[t], i1 = indices[t + 1], i2 = indices[t + 2];
        if (i0 >= vertex_count || i1 >= vertex_count || i2 >= vertex_count) continue;
        vec3 e1 = v3sub(PIX__VPOS(i1), PIX__VPOS(i0)), e2 = v3sub(PIX__VPOS(i2), PIX__VPOS(i0));
        vec2 w1 = v2sub(PIX__VUV(i1), PIX__VUV(i0)), w2 = v2sub(PIX__VUV(i2), PIX__VUV(i0));
        float det = w1.x * w2.y - w2.x * w1.y;
        if (fabsf(det) < 1e-12f) continue;   // no uv area: says nothing about direction
        float k = 1.0f / det;
        // not normalised: bigger triangles (in uv) weigh more, as they should
        vec3 s = v3scale(v3sub(v3scale(e1, w2.y), v3scale(e2, w1.y)), k);
        vec3 u = v3scale(v3sub(v3scale(e2, w1.x), v3scale(e1, w2.x)), k);
        sdir[i0] = v3add(sdir[i0], s); sdir[i1] = v3add(sdir[i1], s); sdir[i2] = v3add(sdir[i2], s);
        tdir[i0] = v3add(tdir[i0], u); tdir[i1] = v3add(tdir[i1], u); tdir[i2] = v3add(tdir[i2], u);
    }
    for (size_t i = 0; i < vertex_count; i++) {
        vec4& out = PIX__VTAN(i);
        if (only_missing && (out.x != 0.0f || out.y != 0.0f || out.z != 0.0f)) continue;
        vec3 n = PIX__VNRM(i);
        vec3 t = v3sub(sdir[i], v3scale(n, v3dot(n, sdir[i])));   // Gram-Schmidt
        if (v3dot(t, t) < 1e-12f) {
            // no usable uvs here: any direction along the surface beats none
            t = fabsf(n.y) < 0.99f ? v3cross(v3(0, 1, 0), n) : v3cross(n, v3(1, 0, 0));
            if (v3dot(t, t) < 1e-12f) t = v3(1, 0, 0);
        }
        t = v3norm(t);
        // pix's uvs start top-left (glTF's convention; OBJ is flipped on load), so
        // +v runs down the image while a normal map's green points up it: the
        // bitangent is the one against tdir. glTF TANGENT data means the same.
        float w = v3dot(v3cross(n, t), tdir[i]) < 0.0f ? 1.0f : -1.0f;
        out = v4(t.x, t.y, t.z, w);
    }
    #undef PIX__VPOS
    #undef PIX__VNRM
    #undef PIX__VUV
    #undef PIX__VTAN
    pix_free(acc);
}

bool pix_tangents_missing(const void* vertices, size_t vertex_count, size_t stride, size_t tangent_offset) {
    const char* base = (const char*)vertices;
    for (size_t i = 0; i < vertex_count; i++) {
        const vec4& t = *(const vec4*)(base + i * stride + tangent_offset);
        if (t.x == 0.0f && t.y == 0.0f && t.z == 0.0f) return true;
    }
    return false;
}

void pix_mesh_compute_tangents(pix_mesh_data& m) {
    pix_compute_tangents(m.vertices, m.vertex_count, sizeof(vertex), offsetof(vertex, tangent),
                         m.indices, m.index_count, false);
}

pix_mesh_data pix_mesh_merge(const pix_mesh_data* meshes, const mat4* transforms, int count, unsigned mem) {
    size_t vc = 0, ic = 0;
    for (int i = 0; i < count; i++) { vc += meshes[i].vertex_count; ic += meshes[i].index_count; }
    pix_mesh_data out = {};
    out.mem = mem & PIX_MEM_SCOPED;
    out.vertices = pix_new<vertex>(vc, out.mem);
    out.indices = pix_new<uint32_t>(ic, out.mem);
    if (!out.vertices || !out.indices) { pix_mesh_free(out); return out; }
    for (int i = 0; i < count; i++) {
        pix_mesh_data part = { out.vertices + out.vertex_count, 0, meshes[i].vertex_count, 0 };
        memcpy(part.vertices, meshes[i].vertices, meshes[i].vertex_count * sizeof(vertex));
        if (transforms) pix_mesh_transform(part, transforms[i]);
        for (size_t k = 0; k < meshes[i].index_count; k++)
            out.indices[out.index_count + k] = meshes[i].indices[k] + (uint32_t)out.vertex_count;
        out.vertex_count += meshes[i].vertex_count;
        out.index_count += meshes[i].index_count;
    }
    return out;
}

void pix_mesh_bounds(const pix_mesh_data& m, vec3* lo, vec3* hi) {
    vec3 a = v3(0, 0, 0), b = v3(0, 0, 0);
    for (size_t i = 0; i < m.vertex_count; i++) {
        if (i == 0) { a = b = m.vertices[i].position; continue; }
        a = v3min(a, m.vertices[i].position);
        b = v3max(b, m.vertices[i].position);
    }
    if (lo) *lo = a;
    if (hi) *hi = b;
}

#endif // PIX_MESHOPS_IMPL
