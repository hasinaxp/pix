// Compile + behaviour check for the library's CPU-side modules. Not part of the library.
//   build test
#define PIX_IMPLEMENTATION
#include "../src/pix.h"
#include <stdio.h>

static int failures = 0;
static void check(bool ok, const char* what) {
    if (!ok) { printf("FAIL  %s\n", what); failures++; }
}
static void near_eq(float got, float want, float tol, const char* what) {
    if (fabsf(got - want) > tol) {
        printf("FAIL  %s: got %.5f want %.5f\n", what, got, want);
        failures++;
    }
}

static int marks[10000];
static void mark_job(void* user, size_t begin, size_t end, int worker) {
    (void)user; (void)worker;
    for (size_t i = begin; i < end; i++) marks[i]++;
}

// every worker fills scoped memory with its own pattern and checks nothing
// else wrote over it: each thread must get a stack of its own
static std::atomic<int> scoped_errors;
static void scoped_job(void* user, size_t begin, size_t end, int worker) {
    (void)user;
    pix_mem_scope s = pix_scope_begin();
    size_t n = (end - begin) * 64;
    uint32_t* a = pix_new<uint32_t>(n, PIX_MEM_SCOPED);
    if (!a) { scoped_errors++; pix_scope_end(s); return; }
    for (size_t i = 0; i < n; i++) a[i] = (uint32_t)(begin * 31 + i + worker);
    for (size_t i = 0; i < n; i++) if (a[i] != (uint32_t)(begin * 31 + i + worker)) scoped_errors++;
    pix_scope_end(s);
}

int main() {
    contact3 c;

    // ---- sphere / sphere ----
    check(collide_sphere_sphere(sphere_make(v3(0,0,0),1), sphere_make(v3(1.5f,0,0),1), &c),
          "spheres 1.5 apart, radii 1+1, overlap");
    near_eq(c.depth, 0.5f, 1e-5f, "sphere/sphere depth");
    near_eq(c.normal.x, 1.0f, 1e-5f, "sphere/sphere normal points a to b");
    check(!collide_sphere_sphere(sphere_make(v3(0,0,0),1), sphere_make(v3(3,0,0),1), &c),
          "spheres 3 apart do not overlap");

    // ---- sphere / plane: a ball sunk 0.4 into a floor ----
    plane floor_p = plane_through(v3(0,1,0), v3(0,0,0));
    check(collide_sphere_plane(sphere_make(v3(0,0.6f,0),1), floor_p, &c), "sphere sunk into floor");
    near_eq(c.depth, 0.4f, 1e-5f, "sphere/plane depth");
    near_eq(c.normal.y, -1.0f, 1e-5f, "sphere/plane normal points into the floor");
    check(!collide_sphere_plane(sphere_make(v3(0,2,0),1), floor_p, &c), "sphere clear of floor");

    // ---- sphere / aabb, outside and inside ----
    aabb box = aabb_make(v3(-1,-1,-1), v3(1,1,1));
    check(collide_sphere_aabb(sphere_make(v3(1.5f,0,0),1), box, &c), "sphere clipping box face");
    near_eq(c.depth, 0.5f, 1e-5f, "sphere/aabb depth");
    near_eq(c.normal.x, -1.0f, 1e-5f, "sphere/aabb normal points sphere to box");
    check(collide_sphere_aabb(sphere_make(v3(0.9f,0,0),0.2f), box, &c), "sphere centre inside box");
    near_eq(c.normal.x, -1.0f, 1e-5f, "inside-box normal leaves by nearest face");
    check(!collide_sphere_aabb(sphere_make(v3(3,0,0),1), box, &c), "sphere clear of box");

    // ---- capsule_upright shape ----
    capsule k = capsule_upright(v3(0,0,0), 1.8f, 0.3f);
    near_eq(k.a.y, 0.3f, 1e-5f, "upright capsule bottom cap centre");
    near_eq(k.b.y, 1.5f, 1e-5f, "upright capsule top cap centre");
    capsule squat = capsule_upright(v3(0,0,0), 0.4f, 5.0f);   // radius wider than half height
    near_eq(squat.radius, 0.2f, 1e-5f, "over-fat capsule collapses to a sphere");
    aabb kb = capsule_bounds(k);
    near_eq(kb.min.y, 0.0f, 1e-5f, "capsule bounds reach the feet");
    near_eq(kb.max.y, 1.8f, 1e-5f, "capsule bounds reach the head");

    // ---- capsule / capsule ----
    capsule k2 = capsule_upright(v3(0.5f,0,0), 1.8f, 0.3f);
    check(collide_capsule_capsule(k, k2, &c), "capsules 0.5 apart, radii 0.3+0.3, overlap");
    near_eq(c.depth, 0.1f, 1e-5f, "capsule/capsule depth");
    near_eq(c.normal.x, 1.0f, 1e-5f, "capsule/capsule normal");
    check(!collide_capsule_capsule(k, capsule_upright(v3(2,0,0), 1.8f, 0.3f), &c),
          "capsules 2 apart do not overlap");
    check(!collide_capsule_capsule(k, capsule_make(v3(-2,3,0), v3(2,3,0), 0.2f), &c),
          "crossing bar passes clear above the capsule");
    check(collide_capsule_capsule(k, capsule_make(v3(-2,1.0f,0), v3(2,1.0f,0), 0.2f), &c),
          "crossing bar at chest height hits the capsule");

    // ---- capsule / plane ----
    check(collide_capsule_plane(capsule_upright(v3(0,-0.1f,0), 1.8f, 0.3f), floor_p, &c),
          "capsule sunk into floor");
    check(!collide_capsule_plane(capsule_upright(v3(0,1,0), 1.8f, 0.3f), floor_p, &c),
          "capsule standing clear of floor");

    // ---- capsule / aabb ----
    check(collide_capsule_aabb(capsule_upright(v3(1.2f,0,0), 1.8f, 0.3f), box, &c),
          "capsule clipping box");
    check(!collide_capsule_aabb(capsule_upright(v3(4,0,0), 1.8f, 0.3f), box, &c),
          "capsule clear of box");

    // ---- rays ----
    float t;
    check(ray_vs_sphere(v3(-5,0,0), v3(1,0,0), sphere_make(v3(0,0,0),1), 100, &t), "ray hits sphere");
    near_eq(t, 4.0f, 1e-4f, "ray/sphere distance");
    check(!ray_vs_sphere(v3(-5,5,0), v3(1,0,0), sphere_make(v3(0,0,0),1), 100, &t), "ray misses sphere");
    check(!ray_vs_sphere(v3(5,0,0), v3(1,0,0), sphere_make(v3(0,0,0),1), 100, &t),
          "ray pointing away from sphere misses");
    check(ray_vs_sphere(v3(0,0,0), v3(1,0,0), sphere_make(v3(0,0,0),1), 100, &t),
          "ray starting inside sphere hits");
    near_eq(t, 0.0f, 1e-5f, "ray inside sphere reports 0");

    check(ray_vs_plane(v3(0,5,0), v3(0,-1,0), floor_p, 100, false, &t), "ray down onto floor");
    near_eq(t, 5.0f, 1e-4f, "ray/plane distance");
    check(!ray_vs_plane(v3(0,-5,0), v3(0,1,0), floor_p, 100, false, &t),
          "one-sided floor ignores its underside");
    check(ray_vs_plane(v3(0,-5,0), v3(0,1,0), floor_p, 100, true, &t),
          "two-sided floor takes its underside");
    check(!ray_vs_plane(v3(0,5,0), v3(1,0,0), floor_p, 100, true, &t), "ray parallel to plane misses");

    check(ray_vs_capsule(v3(-5,0.9f,0), v3(1,0,0), k, 100, &t), "ray into capsule barrel");
    near_eq(t, 4.7f, 1e-4f, "ray/capsule barrel distance");
    check(ray_vs_capsule(v3(0,5,0), v3(0,-1,0), k, 100, &t), "ray down onto capsule top cap");
    near_eq(t, 3.2f, 1e-4f, "ray/capsule top cap distance");
    check(!ray_vs_capsule(v3(-5,0.9f,2), v3(1,0,0), k, 100, &t), "ray misses capsule sideways");
    check(!ray_vs_capsule(v3(-5,3,0), v3(1,0,0), k, 100, &t), "ray passes above capsule");

    // ---- closest points on segments ----
    vec3 p1, p2;
    closest_points_segments(v3(0,0,0), v3(1,0,0), v3(0,1,0), v3(1,1,0), &p1, &p2);
    near_eq(v3len(v3sub(p2,p1)), 1.0f, 1e-5f, "parallel segments 1 apart");
    closest_points_segments(v3(-1,0,0), v3(1,0,0), v3(0,2,-1), v3(0,2,1), &p1, &p2);
    near_eq(v3len(v3sub(p2,p1)), 2.0f, 1e-5f, "crossed segments 2 apart");

    // ---- memory: a virtual arena commits as it grows and never moves ----
    mem_arena a = pix_arena_create(64ull * MB);
    check(a.data && a.committed == 0, "arena reserves without committing");
    int* first = allocate<int>(a, 4);
    first[0] = 42;
    check(a.committed >= 16, "first allocation commits pages");
    char* big = allocate<char>(a, 5 * MB);
    check(big && a.committed >= 5 * MB, "large allocation commits as needed");
    big[5 * MB - 1] = 1;                                     // touching the last byte must not fault
    check(first[0] == 42, "earlier allocations do not move");
    arena_reset(a);
    pix_arena_trim(a);
    check(a.committed <= 1 * MB, "trim returns pages to the OS");
    check(allocate<char>(a, 65 * MB) == nullptr, "allocations past the reservation fail cleanly");
    pix_arena_destroy(a);

    // ---- quaternions ----
    quat q = quat_from_axis_angle(v3(0, 1, 0), PIX_PI * 0.5f);
    vec3 rx = quat_rotate(q, v3(1, 0, 0));
    near_eq(rx.z, -1.0f, 1e-5f, "90 degrees about +Y takes +X to -Z");
    mat4 qm = mat4_from_quat(q);
    near_eq(mat4_mul_dir(qm, v3(1, 0, 0)).z, -1.0f, 1e-5f, "quat matrix agrees with quat_rotate");
    vec3 sum = v3(1, 2, 3) + v3(1, 1, 1) * 2.0f;
    near_eq(sum.z, 5.0f, 1e-6f, "vector operators");
    // ---- mesh processing: an unshared cube (24 corners per face set -> 36 verts) ----
    {
        vertex cv[36]; uint32_t ci[36];
        const float P[8][3] = { {-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},{-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1} };
        const int F[12][3] = { {0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},{3,7,6},{3,6,2},{0,4,7},{0,7,3},{1,2,6},{1,6,5} };
        for (int t = 0; t < 12; t++) for (int k = 0; k < 3; k++) {
            vertex& v = cv[t * 3 + k];
            v.position = v3(P[F[t][k]][0], P[F[t][k]][1], P[F[t][k]][2]);
            v.normal = v3(0, 1, 0); v.uv = v2(0, 0);
            ci[t * 3 + k] = (uint32_t)(t * 3 + k);
        }
        pix_mesh_data m = pix_mesh_copy_data(cv, 36, ci, 36);
        pix_mesh_recompute_normals(m, 30.0f);
        check(m.vertex_count == 24, "crisp cube: 4 corners per face after normal recompute");
        near_eq(fabsf(m.vertices[0].normal.x) + fabsf(m.vertices[0].normal.y) + fabsf(m.vertices[0].normal.z), 1.0f, 1e-4f,
                "crisp cube normals are axis aligned");
        pix_mesh_recompute_normals(m, 180.0f);
        check(m.vertex_count == 8, "smooth cube welds to 8 vertices");
        pix_mesh_free(m);
    }
    {
        // a finely tessellated sphere-ish grid: simplify to a quarter and keep its size
        pix_mesh_data g = {};
        const int N = 40;
        g.vertices = pix_new<vertex>((N + 1) * (N + 1));
        g.indices = pix_new<uint32_t>(N * N * 6);
        for (int y = 0; y <= N; y++) for (int x = 0; x <= N; x++) {
            float th = (float)x / N * PIX_TAU, ph = (float)y / N * PIX_PI;
            vertex& v = g.vertices[g.vertex_count++];
            v.normal = v3(sinf(ph) * cosf(th), cosf(ph), sinf(ph) * sinf(th));
            v.position = v.normal;
            v.uv = v2(0.5f, 0.5f);      // uniform uv: no seams to lock
        }
        for (int y = 0; y < N; y++) for (int x = 0; x < N; x++) {
            uint32_t a = y * (N + 1) + x, b = a + 1, c = a + N + 1, d = c + 1;
            uint32_t q[6] = { a, c, b, b, c, d };
            memcpy(g.indices + g.index_count, q, sizeof(q)); g.index_count += 6;
        }
        pix_mesh_weld(g, 1e-4f, -2.0f, 1.0f);   // the sphere's pole and seam rows collapse
        pix_mesh_cleanup(g);
        size_t before = g.index_count / 3;
        size_t after = pix_mesh_simplify(g, 0.25f);
        check(after <= before / 3 && after > 20, "simplify reaches roughly a quarter");
        vec3 lo, hi;
        pix_mesh_bounds(g, &lo, &hi);
        near_eq(hi.y - lo.y, 2.0f, 0.2f, "simplified sphere keeps its size");
        pix_mesh_subdivide(g, 1);
        check(g.index_count / 3 == after * 4, "subdivide quadruples the triangles");
        pix_mesh_free(g);
    }
    // ---- math ----
    mat4 m = mat4_trs_y3(v3(1,2,3), 0.7f, 1.5f, 2.0f, 0.5f);
    mat4 id = mat4_mul(m, mat4_inverse(m));
    for (int i = 0; i < 16; i++)
        near_eq(id.data[i], mat4_identity().data[i], 1e-4f, "m times inverse(m) is identity");
    near_eq(angle_delta(3.0f, -3.0f), 0.28318f, 1e-4f, "angle_delta takes the short way round");
    near_eq(damp(0.0f, 1.0f, 1e9f, 1.0f), 1.0f, 1e-5f, "damp converges");
    near_eq(mat4_mul_point(mat4_translate(1,2,3), v3(0,0,0)).y, 2.0f, 1e-5f,
            "mat4_mul_point applies translation");

    // ---- frustum: a 90 degree perspective looking down -Z ----
    mat4 vp = mat4_mul(mat4_perspective(1.5708f, 1.0f, 0.1f, 100.0f),
                       mat4_lookat(v3(0,0,0), v3(0,0,-1), v3(0,1,0)));
    frustum fr = frustum_from_viewproj(vp);
    check(frustum_test_sphere(fr, v3(0,0,-10), 1.0f), "sphere in front is visible");
    check(!frustum_test_sphere(fr, v3(0,0,10), 1.0f), "sphere behind is culled");
    check(!frustum_test_sphere(fr, v3(0,0,-200), 1.0f), "sphere past the far plane is culled");
    check(frustum_test_aabb(fr, v3(-1,-1,-11), v3(1,1,-9)), "box in front is visible");
    check(!frustum_test_aabb(fr, v3(-1,-1,9), v3(1,1,11)), "box behind is culled");

    // ---- random ----
    rng ra = rng_seed(7), rb = rng_seed(7);
    check(rng_u32(ra) == rng_u32(rb), "same seed gives the same stream");
    check(hash2_float(3,4,9) == hash2_float(3,4,9), "hash2 is stateless and stable");
    for (int i = 0; i < 1000; i++) {
        float f = rng_float(ra);
        check(f >= 0.0f && f < 1.0f, "rng_float stays in [0,1)");
        int n = rng_int(ra, 2, 5);
        check(n >= 2 && n <= 5, "rng_int stays in range");
    }

    // ---- flat 3D world ----
    static phys3d_flat_world w;
    phys3d_flat_create_world(w);
    phys3d_flat_add_static_box(w, v3(-20, -1, -20), v3(20, 0, 20));   // floor slab
    phys3d_flat_add_static_box(w, v3(2, 0, -20), v3(3, 4, 20));       // wall across x = 2..3
    phys3d_flat_build_statics(w);

    phys3d_flat_body body = phys3d_flat_make_cylinder(v3(0, 5, 0), 0.3f, 1.8f);
    body.layer = PHYS_LAYER_PLAYER;
    idx id_body = phys3d_flat_add_body(w, body);

    for (int i = 0; i < 240; i++) phys3d_flat_step(w, 1.0f / 60.0f);
    phys3d_flat_body* bp = phys3d_flat_get_body(w, id_body);
    check(bp->on_ground, "body came to rest on the ground");
    near_eq(bp->position.y, 0.0f, 1e-3f, "body rests at ground height");

    for (int i = 0; i < 240; i++) {                 // drive it into the wall
        phys3d_flat_get_body(w, id_body)->velocity.x = 6.0f;
        phys3d_flat_step(w, 1.0f / 60.0f);
    }
    bp = phys3d_flat_get_body(w, id_body);
    check(bp->position.x < 2.0f, "body stopped by the wall");
    check(bp->position.x > 1.4f, "body stopped against the wall, not short of it");

    phys3d_flat_ray_hit fh;
    check(phys3d_flat_raycast(w, v3(0, 1, 0), v3(1, 0, 0), 50.0f, &fh, PHYS_LAYER_ALL, id_body), "raycast finds the wall");
    check(fh.body == PIX_INVALID, "flat raycast reports static geometry as PIX_INVALID");
    near_eq(fh.distance, 2.0f, 0.05f, "raycast distance to the wall");
    near_eq(fh.normal.x, -1.0f, 1e-4f, "flat raycast normal faces the ray");
    check(phys3d_flat_raycast(w, v3(-5, 1, 0), v3(1, 0, 0), 50.0f, &fh) && fh.body == id_body,
          "flat raycast hits the body before the wall");
    near_eq(fh.normal.x, -1.0f, 0.05f, "flat raycast body normal");
    check(!phys3d_flat_raycast(w, v3(-5, 1, 0), v3(1, 0, 0), 50.0f, &fh, PHYS_LAYER_1) || fh.body == PIX_INVALID,
          "flat raycast mask skips the body");

    // layers under the shared rule: both must accept
    phys3d_flat_body crate = phys3d_flat_make_box(v3(-5, 0, 0), v2(0.5f, 0.5f), 1.0f, 0.3f);
    crate.layer = PHYS_LAYER_3;
    crate.mask = PHYS_LAYER_ALL & ~PHYS_LAYER_PLAYER;      // the player walks through it
    idx id_crate = phys3d_flat_add_body(w, crate);
    phys3d_flat_get_body(w, id_body)->position = v3(-7, 0, 0);
    for (int i = 0; i < 90; i++) {
        phys3d_flat_get_body(w, id_body)->velocity.x = 4.0f;
        phys3d_flat_step(w, 1.0f / 60.0f);
    }
    check(phys3d_flat_get_body(w, id_body)->position.x > -3.5f, "flat layers: one side refusing is enough to pass");
    check(phys_layers_collide(PHYS_LAYER_0, PHYS_LAYER_ALL, PHYS_LAYER_1, PHYS_LAYER_0) &&
          !phys_layers_collide(PHYS_LAYER_0, PHYS_LAYER_ALL, PHYS_LAYER_1, PHYS_LAYER_2), "shared layer rule");

    idx near_ids[4];
    check(phys3d_flat_overlap(w, v3(-5, 0, 0), 0.2f, near_ids, 4) == 1 && near_ids[0] == id_crate, "flat overlap finds the crate");
    check(phys3d_flat_overlap(w, v3(-5, 0, 0), 0.2f, near_ids, 4, PHYS_LAYER_PLAYER) == 0, "flat overlap mask");
    check(phys3d_flat_overlap(w, v3(0, 0, 0), 30.0f, near_ids, 1) == 2, "flat overlap counts past its capacity");

    w.accumulator = 0;
    check(phys3d_flat_update(w, 0.04f) == 2 && w.alpha > 0.35f && w.alpha < 0.45f, "flat update runs fixed steps");
    phys3d_flat_destroy_world(w);

    // ---- jobs ----
    pix_jobs jobs;
    pix_jobs_start(jobs);
    pix_parallel_for(jobs, 10000, mark_job, 0, 64);
    int wrong = 0;
    for (int i = 0; i < 10000; i++) if (marks[i] != 1) wrong++;
    check(wrong == 0, "parallel_for visited every item exactly once");
    pix_jobs_stop(jobs);

    // ---- memory: global heap ----
    {
        pix_mem_stats before = pix_mem_get_stats();
        uint32_t* g = pix_new<uint32_t>(1000, PIX_MEM_GLOBAL | PIX_MEM_ZERO);
        check(g != 0, "global alloc");
        bool zero = true;
        for (int i = 0; i < 1000; i++) if (g[i]) zero = false;
        check(zero, "PIX_MEM_ZERO gives zeroed global memory");
        check(pix_mem_get_stats().global_blocks == before.global_blocks + 1, "global block counted");
        check(pix_mem_get_stats().global_bytes >= before.global_bytes + 4000, "global bytes counted");
        g[999] = 7;
        g = (uint32_t*)pix_mem_realloc(g, 4000 * sizeof(uint32_t));
        check(g && g[999] == 7, "global realloc keeps contents");
        check(!pix_mem_is_scoped(g), "global block is not scoped");
        pix_mem_free(g);
        check(pix_mem_get_stats().global_blocks == before.global_blocks, "global free uncounts the block");
        check(pix_mem_get_stats().global_bytes == before.global_bytes, "global free uncounts the bytes");
        pix_mem_free(0);
    }

    // ---- memory: scopes ----
    {
        check(pix_mem_alloc(16, PIX_MEM_SCOPED) == 0, "scoped alloc with no scope open is refused");
        pix_mem_scope outer = pix_scope_begin();
        char* a = pix_new<char>(100, PIX_MEM_SCOPED);
        check(a && pix_mem_is_scoped(a), "scoped alloc lands in the scope stack");
        check(((uintptr_t)a & 15) == 0, "scoped alloc is 16-byte aligned");
        memset(a, 0xAB, 100);
        pix_mem_scope inner = pix_scope_begin();
        char* b = pix_new<char>(100, PIX_MEM_SCOPED);
        check(b > a, "nested scope allocates above the outer one");
        pix_scope_end(inner);
        char* b2 = pix_new<char>(100, PIX_MEM_SCOPED);
        check(b2 == b, "ending a scope gives its memory back");
        char* grown = (char*)pix_mem_realloc(b2, 5000);
        check(grown == b2, "newest scoped block grows in place");
        char* moved = (char*)pix_mem_realloc(a, 300);
        check(moved != a && moved && (unsigned char)moved[99] == 0xAB, "older scoped block is copied when grown");
        uint8_t* z = pix_new<uint8_t>(64, PIX_MEM_SCOPED | PIX_MEM_ZERO);
        bool zero = true;
        for (int i = 0; i < 64; i++) if (z[i]) zero = false;
        check(zero, "PIX_MEM_ZERO zeroes reused scoped memory");
        long long blocks = pix_mem_get_stats().global_blocks;
        pix_mem_free(a);   // must be ignored
        check(pix_mem_get_stats().global_blocks == blocks, "pix_mem_free ignores scoped blocks");
        // a big one, past the trim threshold, then make sure the stack shrinks back
        char* big = pix_new<char>(64u * 1024 * 1024, PIX_MEM_SCOPED);
        check(big != 0, "64 MB scoped alloc");
        if (big) big[64u * 1024 * 1024 - 1] = 1;
        pix_scope_end(outer);
        check(pix_mem_get_stats().scoped_committed <= 8 * 1024 * 1024 * (long long)pix_mem_get_stats().scoped_threads,
              "outermost scope end trims the stack back");
    }

    // ---- memory: stack buffers ----
    {
        pix_mem_scope s = pix_scope_begin();
        PIX_STACK_BUFFER(int, small, 10, 64);
        check(small == small__stack, "small request uses the call stack");
        PIX_STACK_BUFFER(int, large, 1000, 64);
        check(large && large != large__stack && pix_mem_is_scoped(large), "large request falls back to scoped");
        large[999] = 3;
        small[9] = 4;
        pix_scope_end(s);
    }

    // ---- memory: arenas as commit-on-demand tables ----
    {
        long long before = pix_mem_get_stats().arena_committed;
        mem_arena t = pix_arena_create(64u * 1024 * 1024);
        check(t.committed == 0, "reserving commits nothing");
        check(pix_arena_commit(t, 100), "commit a small prefix");
        check(t.committed == 64 * 1024, "a small table commits one 64 KB step");
        check(pix_mem_get_stats().arena_committed == before + 64 * 1024, "arena commit is counted");
        check(((uint8_t*)t.data)[100] == 0, "committed pages read as zero");
        check(pix_arena_commit(t, 1000000) && t.committed >= 1000000, "commit grows to cover the request");
        check(!pix_arena_commit(t, 65u * 1024 * 1024), "commit past the reservation fails");
        uint32_t* x = allocate<uint32_t>(t, 10);
        check(x == (uint32_t*)t.data, "allocate still bumps from the start");
        pix_arena_trim(t);
        check(t.committed == 64 * 1024, "trim keeps one step around the used part");
        pix_arena_destroy(t);
        check(pix_mem_get_stats().arena_committed == before, "destroy uncounts the arena");
    }

    // ---- memory: functions that hand memory back take a flag ----
    {
        long long blocks = pix_mem_get_stats().global_blocks;
        pix_mem_scope s = pix_scope_begin();
        size_t size = 0;
        unsigned char* text = pix_read_file("tests/build_check.cpp", &size, PIX_MEM_SCOPED);
        check(text && size > 100 && pix_mem_is_scoped(text) && text[size] == 0, "pix_read_file into scoped memory");
        unsigned char* text_g = pix_read_file("tests/build_check.cpp", &size);
        check(text_g && !pix_mem_is_scoped(text_g), "pix_read_file defaults to global");
        pix_free(text_g);

        vertex tri[3] = {};
        tri[1].position = v3(1, 0, 0); tri[2].position = v3(0, 1, 0);
        uint32_t idx3[3] = { 0, 1, 2 };
        pix_mesh_data m = pix_mesh_copy_data(tri, 3, idx3, 3, PIX_MEM_SCOPED);
        check(pix_mem_is_scoped(m.vertices) && pix_mem_is_scoped(m.indices), "scoped mesh copy");
        pix_mesh_subdivide(m, 2);
        check(m.index_count == 48 && pix_mem_is_scoped(m.vertices) && pix_mem_is_scoped(m.indices),
              "operations on a scoped mesh keep it scoped");
        pix_mesh_recompute_normals(m, 0.0f);
        check(pix_mem_is_scoped(m.vertices), "recomputed normals stay scoped");
        pix_mesh_data both[2] = { m, m };
        pix_mesh_data merged = pix_mesh_merge(both, 0, 2, PIX_MEM_SCOPED);
        check(merged.index_count == 96 && pix_mem_is_scoped(merged.vertices), "scoped merge");
        pix_mesh_free(m);       // harmless for scoped arrays
        pix_mesh_free(merged);
        pix_scope_end(s);
        check(pix_mem_get_stats().global_blocks == blocks, "nothing global left behind");
    }

    // ---- memory: one scope stack per thread ----
    {
        pix_jobs mj;
        pix_jobs_start(mj);
        pix_parallel_for(mj, 4096, scoped_job, 0, 16);
        check(scoped_errors == 0, "workers' scoped memory never overlaps");
        int threads_during = pix_mem_get_stats().scoped_threads;
        pix_jobs_stop(mj);
        check(pix_mem_get_stats().scoped_threads < threads_during || threads_during <= 1,
              "stopped workers give their scope stacks back");
    }

    // ---- lighting ----
    for (float hour = 0.0f; hour < 24.0f; hour += 0.25f) {
        pix_sun sun; pix_sky sky; float exposure;
        pix_daylight_at(hour, &sun, &sky, &exposure);
        check(exposure > 0.0f && exposure < 100.0f, "exposure stays sane across the day");
        near_eq(v3len(sun.direction), 1.0f, 1e-3f, "sun direction stays normalised");
    }

    // ---- math: scalar and vector helpers ----
    near_eq(remapf(5, 0, 10, 100, 200), 150, 1e-4f, "remapf");
    near_eq(inverse_lerpf(2, 4, 3), 0.5f, 1e-6f, "inverse_lerpf");
    near_eq(fractf(-0.25f), 0.75f, 1e-6f, "fractf of a negative");
    near_eq(wrapf(370, 0, 360), 10, 1e-3f, "wrapf from above");
    near_eq(wrapf(-30, 0, 360), 330, 1e-3f, "wrapf from below");
    near_eq(saturatef(1.5f) + saturatef(-1) + signf(-3), 0.0f, 0.0f, "saturatef / signf");
    check(approx_equalf(1e6f, 1e6f + 1.0f), "approx_equalf is relative for large values");
    check(!approx_equalf(1.0f, 1.001f), "approx_equalf sees a real difference");
    check(v3approx_equal(v3reflect(v3(1, -1, 0), v3(0, 1, 0)), v3(1, 1, 0)), "v3reflect");
    check(v3approx_equal(v3project(v3(2, 3, 4), v3(0, 5, 0)), v3(0, 3, 0)), "v3project");
    check(v3approx_equal(v3reject(v3(2, 3, 4), v3(0, 5, 0)), v3(2, 0, 4)), "v3reject");

    // ---- math: quaternion conversions ----
    {
        float p = 0.3f, y = -1.2f, r = 0.7f, p2, y2, r2;
        quat q = quat_from_euler(p, y, r);
        quat_to_euler(q, &p2, &y2, &r2);
        near_eq(p2, p, 1e-4f, "quat_to_euler pitch");
        near_eq(y2, y, 1e-4f, "quat_to_euler yaw");
        near_eq(r2, r, 1e-4f, "quat_to_euler roll");
        quat g = quat_from_euler(PIX_PI * 0.5f, 0.4f, 0.3f);           // looking straight up
        quat_to_euler(g, &p2, &y2, &r2);
        quat g2 = quat_from_euler(p2, y2, r2);
        vec3 t = v3(0.3f, 0.5f, 0.8f);
        check(v3approx_equal(quat_rotate(g, t), quat_rotate(g2, t), 1e-3f), "euler at the pole still gives the same rotation");
        quat m = quat_from_mat4(mat4_from_trs(v3(1, 2, 3), q, v3(2, 3, 4)));
        check(fabsf(m.x * q.x + m.y * q.y + m.z * q.z + m.w * q.w) > 0.9999f, "quat_from_mat4 sees through scale");
        vec3 axis; float angle;
        quat_to_axis_angle(quat_from_axis_angle(v3(0, 0, 2), 1.1f), &axis, &angle);
        near_eq(angle, 1.1f, 1e-5f, "axis-angle round trip: angle");
        near_eq(axis.z, 1.0f, 1e-5f, "axis-angle round trip: axis");
    }

    // ---- math: mat3 ----
    {
        mat4 m4 = mat4_from_trs(v3(1, 2, 3), quat_from_euler(0.2f, 0.5f, -0.3f), v3(2, 0.5f, 3));
        mat3 m = mat3_from_mat4(m4);
        mat3 id = mat3_mul(mat3_inverse(m), m);
        float off = 0.0f;
        for (int i = 0; i < 9; i++) off = maxf(off, fabsf(id.data[i] - ((i % 4) == 0 ? 1.0f : 0.0f)));
        check(off < 1e-5f, "mat3_inverse * m is the identity");
        near_eq(mat3_determinant(m), 3.0f, 1e-4f, "mat3_determinant is the volume scale");
        vec3 tangent = v3(1, 1, 0), normal = v3norm(v3(1, -1, 0));
        float dotp = v3dot(mat4_mul_dir(m4, tangent), mat3_mul_vec3(mat3_normal_matrix(m4), normal));
        near_eq(dotp, 0.0f, 1e-5f, "normal matrix keeps normals perpendicular under uneven scale");
        quat q = quat_from_euler(0.1f, 0.9f, 0.4f);
        check(v3approx_equal(mat3_mul_vec3(mat3_from_quat(q), v3(1, 2, 3)), quat_rotate(q, v3(1, 2, 3)), 1e-5f),
              "mat3_from_quat matches quat_rotate");
    }

    // ---- math: colour ----
    {
        bool round_trip = true;
        for (int i = 0; i < 256; i++) {
            pix_color c = pix_rgba(i, 255 - i, i / 2, i);
            pix_color back = pix_color_from_linear(pix_color_to_linear(c));
            if (back.r != c.r || back.g != c.g || back.b != c.b || back.a != c.a) round_trip = false;
        }
        check(round_trip, "sRGB -> linear -> sRGB is exact on every 8-bit value");
        near_eq(pix_srgb_to_linear(0.5f), 0.21404f, 1e-4f, "sRGB 0.5 is about 21% linear light");
        pix_color red = pix_hsv(0, 1, 1), green = pix_hsv(120, 1, 1), blue = pix_hsv(-120, 1, 1);
        check(red.r == 255 && red.g == 0 && green.g == 255 && green.r == 0 && blue.b == 255 && blue.g == 0,
              "pix_hsv primaries, with wrapping hue");
        vec4 hsv = pix_color_to_hsv(pix_rgba(255, 128, 0));
        near_eq(hsv.x, 30.1f, 0.2f, "orange hue");
        vec4 hsl = pix_color_to_hsl(pix_hsl(200, 0.6f, 0.4f));
        near_eq(hsl.x, 200, 1.0f, "hsl round trip hue");
        near_eq(hsl.y, 0.6f, 0.01f, "hsl round trip saturation");
        near_eq(hsl.z, 0.4f, 0.01f, "hsl round trip lightness");
    }

    // ---- math: curves, easing, trajectories ----
    {
        float (*eases[])(float) = {
            ease_in_quad, ease_out_quad, ease_in_out_quad, ease_in_cubic, ease_out_cubic, ease_in_out_cubic,
            ease_in_quart, ease_out_quart, ease_in_out_quart, ease_in_sine, ease_out_sine, ease_in_out_sine,
            ease_in_expo, ease_out_expo, ease_in_out_expo, ease_in_circ, ease_out_circ, ease_in_out_circ,
            ease_in_back, ease_out_back, ease_in_out_back, ease_in_elastic, ease_out_elastic, ease_in_out_elastic,
            ease_in_bounce, ease_out_bounce, ease_in_out_bounce };
        bool ends = true, mids = true;
        for (size_t i = 0; i < PIX_ARRAY_COUNT(eases); i++) {
            if (fabsf(eases[i](0.0f)) > 1e-4f || fabsf(eases[i](1.0f) - 1.0f) > 1e-4f) ends = false;
            // the in_out ones pass through the middle
            if (i % 3 == 2 && fabsf(eases[i](0.5f) - 0.5f) > 1e-4f) mids = false;
        }
        check(ends, "every easing runs 0 -> 1");
        check(mids, "every in_out easing passes through 0.5");
        near_eq(catmull_rom(0, 1, 2, 3, 0.5f), 1.5f, 1e-6f, "catmull_rom on a straight line");
        near_eq(catmull_rom(5, 1, 2, -7, 0.0f) + catmull_rom(5, 1, 2, -7, 1.0f), 3.0f, 1e-6f, "catmull_rom passes its middle points");
        vec3 b0 = bezier_cubic(v3(0, 0, 0), v3(1, 2, 0), v3(3, 2, 0), v3(4, 0, 0), 0.0f);
        vec3 b1 = bezier_cubic(v3(0, 0, 0), v3(1, 2, 0), v3(3, 2, 0), v3(4, 0, 0), 1.0f);
        check(v3approx_equal(b0, v3(0, 0, 0)) && v3approx_equal(b1, v3(4, 0, 0)), "bezier_cubic ends on its end points");
        near_eq(bezier_quadratic(0, 2, 0, 0.5f), 1.0f, 1e-6f, "bezier_quadratic peak");
        near_eq(hermite(0, 0, 1, 0, 0.5f), 0.5f, 1e-6f, "hermite with flat tangents is smoothstep");

        vec3 from = v3(0, 1, 0), to = v3(20, 4, 5), vel;
        for (int high = 0; high < 2; high++) {
            check(ballistic_launch(from, to, 20.0f, 9.8f, high != 0, &vel), "ballistic_launch finds an arc");
            near_eq(v3len(vel), 20.0f, 1e-3f, "launch velocity has the asked speed");
            float flight = v2len(v2(to.x - from.x, to.z - from.z)) / v2len(v2(vel.x, vel.z));
            vec3 land = ballistic_position(from, vel, v3(0, -9.8f, 0), flight);
            check(v3approx_equal(land, to, 1e-3f), high ? "the lob lands on the target" : "the flat shot lands on the target");
        }
        check(!ballistic_launch(from, v3(1000, 0, 0), 10.0f, 9.8f, false, &vel), "out of range is refused");
    }

    // ---- colliders: oriented boxes ----
    {
        contact3 c;
        obb a = obb_make(v3(0, 0, 0), v3(1, 1, 1), quat_identity());
        obb b = obb_make(v3(1.5f, 0, 0), v3(1, 1, 1), quat_identity());
        check(collide_obb_obb(a, b, &c), "touching boxes overlap");
        near_eq(c.depth, 0.5f, 1e-5f, "obb/obb depth");
        near_eq(c.normal.x, 1.0f, 1e-5f, "obb/obb normal points a to b");
        quat spin = quat_from_axis_angle(v3(0, 0, 1), PIX_PI * 0.25f);   // a diamond in xy
        obb d = obb_make(v3(2.2f, 2.2f, 0), v3(1, 1, 1), spin);
        check(aabb_overlap(obb_bounds(a), obb_bounds(d)) && !collide_obb_obb(a, d, &c),
              "rotated boxes can be apart while their bounds overlap");
        d.centre = v3(1.6f, 1.6f, 0);
        check(collide_obb_obb(a, d, &c), "rotated boxes overlapping");
        near_eq(c.depth, 1.0f + sqrtf(2.0f) - 1.6f * sqrtf(2.0f), 1e-4f, "rotated overlap is along the diagonal");
        near_eq(c.normal.x, c.normal.y, 1e-4f, "rotated overlap normal is diagonal");
        obb diamond = obb_make(v3(0, 0, 0), v3(1, 1, 1), spin);
        check(collide_sphere_obb(sphere_make(v3(0, sqrtf(2.0f) + 0.3f, 0), 0.5f), diamond, &c), "sphere on a box corner");
        near_eq(c.depth, 0.2f, 1e-4f, "sphere/obb depth");
        near_eq(c.normal.y, -1.0f, 1e-4f, "sphere/obb normal points sphere to box");
        float t;
        check(ray_vs_obb(v3(-5, 0, 0), v3(1, 0, 0), diamond, 100.0f, &t), "ray hits a rotated box");
        near_eq(t, 5.0f - sqrtf(2.0f), 1e-4f, "ray/obb distance to the corner");
        aabb local = aabb_make(v3(-1, 0, -1), v3(1, 2, 1));
        obb moved = obb_from_aabb(local, mat4_from_trs(v3(10, 0, 0), spin, v3(2, 2, 2)));
        near_eq(moved.half.x, 2.0f, 1e-5f, "obb_from_aabb puts scale into half");
        check(v3approx_equal(moved.centre, v3(10, 0, 0) + quat_rotate(spin, v3(0, 2, 0)), 1e-4f), "obb_from_aabb centre");
    }

    // ---- colliders: triangles ----
    {
        vec3 ta = v3(0, 0, 0), tb = v3(1, 0, 0), tc = v3(0, 0, -1);   // front faces +Y
        check(v3approx_equal(triangle_normal(ta, tb, tc), v3(0, 1, 0)), "triangle_normal");
        float t; vec3 bary;
        check(ray_vs_triangle(v3(0.25f, 5, -0.25f), v3(0, -1, 0), ta, tb, tc, 100.0f, false, &t, &bary), "ray hits the front");
        near_eq(t, 5.0f, 1e-5f, "ray/triangle distance");
        check(v3approx_equal(bary, v3(0.5f, 0.25f, 0.25f)), "ray/triangle barycentrics");
        check(!ray_vs_triangle(v3(0.25f, -5, -0.25f), v3(0, 1, 0), ta, tb, tc, 100.0f, false, &t), "one-sided ignores the back");
        check(ray_vs_triangle(v3(0.25f, -5, -0.25f), v3(0, 1, 0), ta, tb, tc, 100.0f, true, &t), "two-sided hits the back");
        check(!ray_vs_triangle(v3(2, 5, 0), v3(0, -1, 0), ta, tb, tc, 100.0f, true, &t), "ray beside the triangle misses");
        check(v3approx_equal(barycentric(v3(0.25f, 0, -0.25f), ta, tb, tc), v3(0.5f, 0.25f, 0.25f)), "barycentric");
        check(v3approx_equal(closest_point_triangle(v3(2, 3, 0), ta, tb, tc), tb), "closest point: corner");
        check(v3approx_equal(closest_point_triangle(v3(0.5f, 3, 0.5f), ta, tb, tc), v3(0.5f, 0, 0)), "closest point: edge");
        check(v3approx_equal(closest_point_triangle(v3(0.2f, 3, -0.2f), ta, tb, tc), v3(0.2f, 0, -0.2f)), "closest point: face");
        contact3 c;
        check(collide_sphere_triangle(sphere_make(v3(0.2f, 0.3f, -0.2f), 0.5f), ta, tb, tc, &c), "sphere resting on a triangle");
        near_eq(c.depth, 0.2f, 1e-5f, "sphere/triangle depth");
        near_eq(c.normal.y, -1.0f, 1e-5f, "sphere/triangle normal points sphere to triangle");
    }

    // ---- 2D: cameras, rectangles, shake ----
    {
        pix_camera2d cam = {};
        cam.offset = v2(640, 360); cam.target = v2(100, 50); cam.rotation = 30.0f; cam.zoom = 2.0f;
        vec2 w = v2(123.0f, -45.0f);
        vec2 back = pix_screen_to_world_2d(cam, pix_world_to_screen_2d(cam, w));
        check(fabsf(back.x - w.x) < 1e-3f && fabsf(back.y - w.y) < 1e-3f, "2D screen/world round trip");
        vec2 centre = pix_screen_to_world_2d(cam, cam.offset);
        check(fabsf(centre.x - 100.0f) < 1e-4f && fabsf(centre.y - 50.0f) < 1e-4f, "the offset shows the target");
        cam.rotation = 0.0f;
        pix_rect vis = pix_camera2d_visible(cam, 1280, 720);
        near_eq(vis.w, 640.0f, 1e-3f, "visible width at zoom 2");
        near_eq(vis.x, 100.0f - 320.0f, 1e-3f, "visible rect is centred on the target");
        cam.rotation = 90.0f;
        vis = pix_camera2d_visible(cam, 1280, 720);
        near_eq(vis.w, 360.0f, 1e-2f, "rotated view swaps the visible extents");
        pix_rect a = { 0, 0, 10, 10 }, b = { 9, 9, 5, 5 }, d = { 10, 0, 5, 5 };
        check(pix_rect_overlaps(a, b) && !pix_rect_overlaps(a, d), "pix_rect_overlaps, touching edges do not count");
        check(pix_rect_contains(a, v2(0, 0)) && !pix_rect_contains(a, v2(10, 5)), "pix_rect_contains is half open");

        pix_shake sh = {};
        pix_shake_update(sh, 1.0f / 60.0f);
        check(sh.offset.x == 0.0f && sh.angle == 0.0f, "no trauma, no shake");
        pix_shake_add(sh, 2.0f);
        near_eq(sh.trauma, 1.0f, 0.0f, "trauma clamps at 1");
        float biggest = 0.0f;
        for (int i = 0; i < 30; i++) { pix_shake_update(sh, 1.0f / 60.0f); biggest = maxf(biggest, v2len(sh.offset)); }
        check(biggest > 2.0f && biggest <= 20.0f * 1.5f, "full trauma shakes within max_offset");
        for (int i = 0; i < 120; i++) pix_shake_update(sh, 1.0f / 60.0f);
        check(sh.trauma == 0.0f && v2len(sh.offset) == 0.0f, "trauma drains to a standstill");
    }

    // ---- JSON reader ----
    {
        const char* text = "{\"name\": \"hero\", \"hp\": 42, \"speed\": 3.5, \"alive\": true,"
                           " \"tags\": [\"a\", \"b\", \"c\"], \"pos\": {\"x\": -1.25, \"y\": 2e2}, \"none\": null}";
        js_doc d;
        int root = js_parse(d, text, strlen(text));
        check(root == 0, "js_parse returns the root");
        char s[16];
        js_str(d, js_get(d, root, "name"), s, sizeof(s));
        check(strcmp(s, "hero") == 0, "js_str");
        check(js_int(d, js_get(d, root, "hp"), 0) == 42, "js_int");
        near_eq(js_float(d, js_get(d, js_get(d, root, "pos"), "y"), 0), 200.0f, 1e-3f, "js_float with exponent");
        check(js_bool(d, js_get(d, root, "alive"), false), "js_bool");
        check(js_type(d, js_get(d, root, "none")) == JS_NULL && js_get(d, root, "missing") < 0, "null and missing keys");
        int tags = js_get(d, root, "tags"), n = 0;
        for (int e = js_first(d, tags); e >= 0; e = js_next(d, tags, e)) n++;
        check(n == 3 && js_len(d, tags) == 3, "js_first / js_next walk an array");
        int keys = 0;
        for (int k = js_first(d, root); k >= 0; k = js_next(d, root, js_value_of(d, k))) keys++;
        check(keys == 7, "js_first / js_next walk an object's keys");
        js_free(d);
        js_doc bad;
        check(js_parse(bad, "{\"a\": [1, 2}", 12) < 0, "malformed JSON is refused");
        js_free(bad);
    }

    // ---- sprite animation (no GL: a grid over a pretend texture) ----
    {
        pix_texture fake = { 1, 64, 32 };
        pix_rect rects[16];
        int nr = pix_rects_grid(rects, 16, fake.width, fake.height, 16, 16);
        check(nr == 8 && rects[5].x == 16 && rects[5].y == 16, "pix_rects_grid: 4 x 2 cells, row by row");
        check(pix_rects_grid(rects, 3, 64, 32, 16, 16) == 3, "pix_rects_grid stops at max");
        // 74 x 40, margin 2, spacing 2: columns at 2 20 38 56 (56 + 16 = 72 = 74 - 2), rows at 2 20
        check(pix_rects_grid(rects, 16, 74, 40, 16, 16, 2, 2) == 8 && rects[1].x == 20 && rects[4].y == 20,
              "pix_rects_grid with margin and spacing");
        check(pix_rects_grid(rects, 16, 73, 40, 16, 16, 2, 2) == 6, "a cell that would cross the margin is left out");
        pix_rect strip[4];
        check(pix_rects_row(strip, 4, 8, 40, 16, 24, 4, 2) == 4 && strip[3].x == 8 + 3 * 18 && strip[3].y == 40,
              "pix_rects_row");
        check(pix_rects_column(strip, 4, 8, 40, 16, 24, 3) == 3 && strip[2].y == 40 + 48, "pix_rects_column");
        nr = pix_rects_grid(rects, 16, fake.width, fake.height, 16, 16);
        pix_atlas a = pix_create_atlas(fake, rects, nr);
        check(a.frame_count == 8 && !a.owns_texture, "pix_create_atlas: one frame per rect, texture stays yours");
        near_eq(a.frames[5].src.x, 16.0f, 0.0f, "grid order is row by row");
        near_eq(a.frames[0].pivot.x, 8.0f, 0.0f, "default pivot is the centre");
        int run = pix_atlas_add_anim(a, "run", 0, 4, 10.0f);         // 0.1 s per frame
        int once = pix_atlas_add_anim(a, "die", 4, 3, 10.0f, PIX_ANIM_ONCE);
        int ping = pix_atlas_add_anim(a, "bob", 4, 3, 10.0f, PIX_ANIM_PINGPONG);
        int custom[3] = { 7, 0, 7 };
        int seq = pix_atlas_add_anim_frames(a, "blink", custom, 3, 10.0f);
        check(run == 0 && once == 1 && ping == 2 && seq == 3 && pix_atlas_find_anim(a, "die") == 1, "animations by index and name");
        check(pix_atlas_add_anim(a, "bad", 6, 5, 10.0f) < 0, "an animation past the last frame is refused");

        pix_sprite_player p = {};
        check(pix_sprite_frame(p, a) == -1, "nothing plays before pix_sprite_play");
        pix_sprite_play(p, a, "run");
        check(pix_sprite_frame(p, a) == 0 && pix_sprite_entered(p, 0), "play starts on the first frame");
        pix_sprite_update(p, a, 0.25f);                              // 2.5 frames
        check(p.step == 2 && pix_sprite_entered(p, 1) && pix_sprite_entered(p, 2), "steps entered in one long update");
        pix_sprite_play(p, a, "run");                                // same animation: keeps going
        check(p.step == 2, "playing the current animation again does not restart it");
        pix_sprite_update(p, a, 0.2f);
        check(p.step == 0 && p.loops == 1, "loops wrap and count");

        pix_sprite_play(p, a, "die");
        pix_sprite_update(p, a, 1.0f);
        check(p.finished && pix_sprite_frame(p, a) == 6, "once stops on its last frame");
        pix_sprite_play(p, a, "die");
        check(p.finished, "finished stays until restart");
        pix_sprite_play(p, a, "die", true);
        check(!p.finished && p.step == 0, "restart starts over");

        pix_sprite_play(p, a, "bob");
        int seen[8] = {}, k = 0;
        for (int i = 0; i < 8; i++) { seen[k++] = p.step; pix_sprite_update(p, a, 0.1f); }
        check(seen[0] == 0 && seen[1] == 1 && seen[2] == 2 && seen[3] == 1 && seen[4] == 0 && seen[5] == 1,
              "ping-pong goes 0 1 2 1 0 1");

        pix_sprite_play(p, a, "blink");
        pix_sprite_update(p, a, 0.1f);
        check(pix_sprite_frame(p, a) == 0, "frame lists play in their own order");

        a.frames[0].duration = 0.5f;                                 // per-frame timing beats fps
        pix_sprite_play(p, a, "run", true);
        pix_sprite_update(p, a, 0.3f);
        check(p.step == 0, "a long frame holds");
        pix_sprite_update(p, a, 0.25f);
        check(p.step == 1, "and then moves on");

        p.speed = 2.0f;
        pix_sprite_update(p, a, 0.05f);
        check(p.step == 2, "speed scales time");
        p.paused = true;
        pix_sprite_update(p, a, 5.0f);
        check(p.step == 2 && p.entered == 0, "paused holds still");
        a.texture.id = 0;                                            // nothing to free on the GPU
        pix_unload_atlas(a);
    }

    // ---- tilemaps (no GL until drawn) ----
    {
        pix_texture fake = { 1, 256, 256 };
        pix_rect cells[256];
        pix_atlas tiles = pix_create_atlas(fake, cells, pix_rects_grid(cells, 256, 256, 256, 16, 16));   // 256 tiles
        pix_tilemap m = pix_create_tilemap(40, 30, 16, 16, tiles);
        int ground = pix_tilemap_add_layer(m, "ground");
        check(m.width == 40 && ground == 0 && pix_tilemap_find_layer(m, "ground") == 0, "tilemap and layer");
        pix_tilemap_set(m, ground, 3, 4, 7, PIX_TILE_FLIP_X);
        check(pix_tilemap_get(m, ground, 3, 4) == 7, "set / get");
        check((m.layers[0].cells[4 * 40 + 3] & PIX_TILE_FLIP_X) != 0, "flip flag stored with the tile");
        check(pix_tilemap_get(m, ground, -1, 0) == -1 && pix_tilemap_get(m, ground, 3, 5) == -1, "outside and empty read -1");
        pix_tilemap_fill(m, ground, 0, 0, 5, 2, 1);
        check(pix_tilemap_get(m, ground, 4, 1) == 1 && pix_tilemap_get(m, ground, 5, 1) == -1, "fill");
        pix_tilemap_set(m, ground, 0, 0, -1);
        check(pix_tilemap_get(m, ground, 0, 0) == -1, "set -1 clears");
        int tx, ty;
        check(pix_tilemap_tile_at(m, v2(33.0f, 70.0f), &tx, &ty) && tx == 2 && ty == 4, "world point to cell");
        check(!pix_tilemap_tile_at(m, v2(-1.0f, 5.0f), &tx, &ty), "outside the map");
        pix_rect tr = pix_tilemap_tile_rect(m, 2, 4);
        check(tr.x == 32 && tr.y == 64 && tr.w == 16, "cell to world rect");

        uint8_t masks[256];
        check(pix_autotile_masks(PIX_AUTOTILE_47, masks) == 47 && pix_autotile_masks(PIX_AUTOTILE_16, masks) == 16,
              "47 blob arrangements, 16 edge arrangements");
        pix_autotile blob = pix_autotile_make(PIX_AUTOTILE_47, 100);
        check(blob.table[0] == 100 && blob.table[255] == 146, "blob: alone is the first tile, surrounded the last");
        check(blob.table[2] == 100, "a lone corner without its edges does not count");

        int deco = pix_tilemap_add_layer(m, "deco");
        pix_tilemap_set_terrain(m, 1, pix_autotile_make(PIX_AUTOTILE_16, 0));
        pix_tilemap_paint(m, deco, 10, 10, 1);
        check(pix_tilemap_get(m, deco, 10, 10) == 0, "a lone painted cell: no neighbours");
        pix_tilemap_paint(m, deco, 11, 10, 1);
        check(pix_tilemap_get(m, deco, 10, 10) == 2 && pix_tilemap_get(m, deco, 11, 10) == 8,
              "painting a neighbour re-tiles both (E = 2, W = 8)");
        pix_tilemap_paint(m, deco, 11, 10, 0);
        check(pix_tilemap_get(m, deco, 11, 10) == -1 && pix_tilemap_get(m, deco, 10, 10) == 0, "erasing re-tiles too");

        pix_tilemap_set_terrain(m, 2, blob);
        for (int y = 20; y < 23; y++) for (int x = 20; x < 23; x++) pix_tilemap_paint(m, deco, x, y, 2);
        check(pix_tilemap_get(m, deco, 21, 21) == 146, "blob centre is the full tile");
        int corner = -1;                                          // E, SE, S = 4 | 8 | 16 = 28
        pix_autotile_masks(PIX_AUTOTILE_47, masks);
        for (int i = 0; i < 47; i++) if (masks[i] == 28) corner = 100 + i;
        check(pix_tilemap_get(m, deco, 20, 20) == corner, "blob top-left corner");
        check(pix_tilemap_terrain(m, deco, 22, 22) == 2, "terrain is kept per cell");
        check(m.layers[1].chunk_dirty[0], "edits mark their chunk for upload");
        pix_unload_tilemap(m);
    }

    // ---- math: fast approximations, worst error over their ranges ----
    {
        float e_quake = 0, e_rsqrt = 0, e_sin = 0, e_sin_far = 0, e_atan = 0, e_atan2 = 0;
        const int N = 400000;
        for (int i = 0; i < N; i++) {
            float u = (float)i / (float)(N - 1);
            float x = expf(-40.0f + 80.0f * u);                         // 4e-18 .. 2e17
            float exact = 1.0f / sqrtf(x);
            e_quake = maxf(e_quake, fabsf(fast_inv_sqrt(x) - exact) / exact);
            e_rsqrt = maxf(e_rsqrt, fabsf(fast_rsqrt(x) - exact) / exact);
            float a = -PIX_TAU + 2.0f * PIX_TAU * u;
            e_sin   = maxf(e_sin, fabsf(fast_sin(a) - sinf(a)));
            float wide = -1000.0f + 2000.0f * u;
            e_sin_far = maxf(e_sin_far, fabsf(fast_sin(wide) - sinf(wide)));
            float z = -100.0f + 200.0f * u;
            e_atan  = maxf(e_atan, fabsf(fast_atan(z) - atanf(z)));
            float ang = -PIX_PI + 2.0f * PIX_PI * u, rad = 0.001f + 1000.0f * u;
            e_atan2 = maxf(e_atan2, fabsf(fast_atan2(sinf(ang) * rad, cosf(ang) * rad) - atan2f(sinf(ang) * rad, cosf(ang) * rad)));
        }
        printf("info  fast math worst error: inv_sqrt %.2e  rsqrt %.2e  sin %.2e (|x|<1000: %.2e)  atan %.2e  atan2 %.2e\n",
               e_quake, e_rsqrt, e_sin, e_sin_far, e_atan, e_atan2);
        check(e_quake < 1.8e-3f, "fast_inv_sqrt relative error under 0.18%");
        check(e_rsqrt < 5e-7f,   "fast_rsqrt relative error under 5e-7");
        check(e_sin < 1e-6f,     "fast_sin error under 1e-6 within a turn");
        check(e_sin_far < 1e-4f, "fast_sin error under 1e-4 for |x| < 1000");
        check(e_atan < 2.5e-6f,  "fast_atan error under 2.5e-6");
        check(e_atan2 < 2.5e-6f, "fast_atan2 error under 2.5e-6");
        vec3 nv = v3norm_fast(v3(3, 4, 12));
        near_eq(v3len(nv), 1.0f, 1e-5f, "v3norm_fast gives unit length");
    }

    // ---- 2D physics world ----
    {
        // narrowphase, every shape pair the one routine handles
        phys2d__poly pa, pb;
        phys2d_contact m;
        phys2d__make_poly(phys2d_circle(10), v2(0, 0), 0, pa);
        phys2d__make_poly(phys2d_circle(10), v2(15, 0), 0, pb);
        check(phys2d__collide(pa, pb, 0.5f, 0, m) == 1, "p2d circle/circle one point");
        near_eq(m.points[0].separation, -5.0f, 1e-4f, "p2d circle/circle depth");
        near_eq(m.normal.x, 1.0f, 1e-4f, "p2d circle/circle normal a to b");

        phys2d__make_poly(phys2d_box(40, 20), v2(0, 0), 0, pa);           // resting box on box
        phys2d__make_poly(phys2d_box(20, 20), v2(5, -19), 0, pb);
        check(phys2d__collide(pa, pb, 0.5f, 0, m) == 2, "p2d box/box face contact has two points");
        near_eq(m.normal.y, -1.0f, 1e-4f, "p2d box/box normal up toward the top box");
        near_eq(m.points[0].separation, -1.0f, 1e-4f, "p2d box/box depth");

        phys2d__make_poly(phys2d_box(40, 20), v2(0, 0), 0.5f, pa);        // rotated corner hit
        phys2d__make_poly(phys2d_circle(5), v2(0, -30), 0, pb);
        check(phys2d__collide(pa, pb, 0.5f, 0, m) == 0, "p2d circle clear of rotated box");

        phys2d__make_poly(phys2d_capsule(10, 60), v2(0, 0), PIX_PI * 0.5f, pa);   // capsule lying down
        phys2d__make_poly(phys2d_box(200, 20), v2(0, 19), 0, pb);
        check(phys2d__collide(pa, pb, 0.5f, 0, m) == 2, "p2d lying capsule on box has two points");
        near_eq(m.points[0].separation, -1.0f, 1e-3f, "p2d capsule/box depth");
        near_eq(m.normal.y, 1.0f, 1e-4f, "p2d capsule/box normal down to the box");

        phys2d__make_poly(phys2d_capsule(5, 40), v2(0, 0), 0, pa);       // end to end, collinear
        phys2d__make_poly(phys2d_capsule(5, 40), v2(0, 38), 0, pb);
        check(phys2d__collide(pa, pb, 0.5f, 0, m) == 1, "p2d collinear capsules touch at the caps");
        near_eq(m.points[0].separation, -2.0f, 1e-3f, "p2d capsule cap depth");

        phys2d__make_poly(phys2d_aabb(20, 20), v2(0, 0), 1.0f, pa);       // aabb ignores angle
        check(fabsf(pa.v[0].x + 10) < 1e-5f && fabsf(pa.v[0].y + 10) < 1e-5f, "p2d aabb never rotates");

        // a stack of boxes settles and stays put
        phys2d_world w;
        phys2d_create_world(w);
        phys2d_add(w, PHYS2D_STATIC, phys2d_box(3000, 40), v2(0, 20));
        idx stack[8];
        for (int i = 0; i < 8; i++) stack[i] = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(30, 30), v2(0, -15.0f - i * 30.5f));
        for (int i = 0; i < 600; i++) phys2d_step(w, 1.0f / 60.0f);
        phys2d_body* top = phys2d_get_body(w, stack[7]);
        near_eq(top->position.x, 0.0f, 0.5f, "p2d stack of 8 stays upright");
        near_eq(top->position.y, -15.0f - 7 * 30.0f, 3.0f, "p2d stack rests at its height");
        near_eq(top->angle, 0.0f, 0.02f, "p2d stack top did not tip");
        check(fabsf(top->velocity.y) < 2.0f, "p2d stack is at rest");
        check(phys2d_get_body(w, stack[0])->on_ground, "p2d bottom box is on the ground");

        // a staggered pyramid (each box resting on two) holds its shape
        {
            phys2d_world pw;
            phys2d_create_world(pw);
            phys2d_add(pw, PHYS2D_STATIC, phys2d_box(2000, 40), v2(0, 20));
            idx apex = PIX_INVALID;
            for (int row = 0; row < 10; row++)
                for (int i = 0; i < 10 - row; i++)
                    apex = phys2d_add(pw, PHYS2D_DYNAMIC, phys2d_box(30, 30),
                                      v2((i - (9 - row) * 0.5f) * 31.0f, -15.0f - row * 30.0f));
            for (int i = 0; i < 600; i++) phys2d_step(pw, 1.0f / 60.0f);
            phys2d_body* ap = phys2d_get_body(pw, apex);
            near_eq(ap->position.x, 0.0f, 1.0f, "p2d pyramid apex stays centred");
            near_eq(ap->position.y, -15.0f - 9 * 30.0f, 4.0f, "p2d pyramid keeps its height");
            check(v2len(ap->velocity) < 2.0f, "p2d pyramid is at rest");
            phys2d_destroy_world(pw);
        }

        // a fast ball does not pass through a thin wall
        idx wall = phys2d_add(w, PHYS2D_STATIC, phys2d_box(4, 400), v2(400, -200));
        phys2d_body bullet = phys2d_make_body(PHYS2D_DYNAMIC, phys2d_circle(3), v2(300, -300));
        bullet.velocity = v2(6000, 0);
        bullet.gravity_scale = 0;
        idx bid = phys2d_add_body(w, bullet);
        for (int i = 0; i < 30; i++) phys2d_step(w, 1.0f / 60.0f);
        check(phys2d_get_body(w, bid)->position.x < 400, "p2d speculative contacts stop tunnelling");
        phys2d_remove_body(w, bid);

        // restitution: a bouncy ball comes back up, a dead one does not
        phys2d_body ball = phys2d_make_body(PHYS2D_DYNAMIC, phys2d_circle(10), v2(-300, -200));
        ball.restitution = 0.8f;
        idx bouncy = phys2d_add_body(w, ball);
        ball.restitution = 0.0f; ball.position.x = -200;
        idx dead = phys2d_add_body(w, ball);
        float peak_after = -1e9f, lowest = -1e9f;
        bool bounced = false;
        for (int i = 0; i < 120; i++) {
            phys2d_step(w, 1.0f / 60.0f);
            for (size_t e = 0; e < w.event_count; e++)
                if (w.events[e].type == PHYS2D_CONTACT_BEGIN && (w.events[e].a == bouncy || w.events[e].b == bouncy))
                    bounced = true;
            phys2d_body* b = phys2d_get_body(w, bouncy);
            if (bounced && b->velocity.y < 0) peak_after = maxf(peak_after, -b->position.y);
            lowest = maxf(lowest, phys2d_get_body(w, dead)->velocity.y < -20 ? 1.0f : -1e9f);
        }
        check(bounced, "p2d contact begin event fires on landing");
        check(peak_after > 100.0f, "p2d restitution bounces the ball back up");
        check(lowest < 0.0f, "p2d restitution 0 does not bounce");

        // triggers report enter/exit and never push
        phys2d_body zone = phys2d_make_body(PHYS2D_STATIC, phys2d_aabb(100, 100), v2(600, -50));
        zone.trigger = true;
        idx zid = phys2d_add_body(w, zone);
        phys2d_body walker = phys2d_make_body(PHYS2D_DYNAMIC, phys2d_capsule(10, 40), v2(500, -21));
        walker.fixed_rotation = true;
        walker.friction = 0;
        walker.layer = PHYS_LAYER_1;
        idx wid = phys2d_add_body(w, walker);
        int enters = 0, exits = 0;
        for (int i = 0; i < 120; i++) {
            phys2d_get_body(w, wid)->velocity.x = 200;
            phys2d_step(w, 1.0f / 60.0f);
            for (size_t e = 0; e < w.event_count; e++) {
                if (w.events[e].type == PHYS2D_TRIGGER_ENTER && w.events[e].a == zid && w.events[e].b == wid) enters++;
                if (w.events[e].type == PHYS2D_TRIGGER_EXIT  && w.events[e].a == zid && w.events[e].b == wid) exits++;
            }
        }
        check(enters == 1 && exits == 1, "p2d trigger enter and exit fire once each");
        check(phys2d_get_body(w, wid)->position.x > 800, "p2d trigger does not block");
        check(phys2d_get_body(w, wid)->on_ground, "p2d capsule walker is grounded");
        near_eq(phys2d_get_body(w, wid)->angle, 0.0f, 1e-6f, "p2d fixed rotation stays upright");

        // layers: a body that masks out layer 0 falls through the floor
        phys2d_body ghost = phys2d_make_body(PHYS2D_DYNAMIC, phys2d_circle(10), v2(-100, -100));
        ghost.mask = PHYS_LAYER_ALL & ~PHYS_LAYER_0;
        idx gid = phys2d_add_body(w, ghost);
        for (int i = 0; i < 60; i++) phys2d_step(w, 1.0f / 60.0f);
        check(phys2d_get_body(w, gid)->position.y > 100, "p2d layer mask lets a ghost through");

        // queries
        phys2d_ray_hit hit;
        check(phys2d_raycast(w, v2(0, -1000), v2(0, 1), 2000, &hit), "p2d raycast down hits the stack");
        check(hit.body == stack[7], "p2d raycast hits the top box first");
        near_eq(hit.point.y, top->position.y - 15, 1.0f, "p2d raycast hit point on the top face");
        near_eq(hit.normal.y, -1.0f, 1e-3f, "p2d raycast normal faces the ray");
        check(phys2d_raycast(w, v2(0, -1000), v2(0, 1), 2000, &hit, PHYS_LAYER_ALL, stack[7]) && hit.body == stack[6],
              "p2d raycast ignore skips a body");
        check(!phys2d_raycast(w, v2(-450, -1000), v2(0, 1), 500, &hit), "p2d raycast miss");

        check(phys2d_shape_cast(w, phys2d_circle(10), v2(-400, -100), 0, v2(0, 1), 500, &hit),
              "p2d circle cast hits the floor");
        near_eq(hit.distance, 90.0f, 0.5f, "p2d shape cast distance to touch");
        near_eq(hit.normal.y, -1.0f, 1e-2f, "p2d shape cast normal");

        idx found[16];
        size_t n = phys2d_overlap(w, phys2d_aabb(40, 100), v2(0, -60), 0, found, 16);
        check(n == 4, "p2d overlap aabb finds the four bottom boxes");
        check(phys2d_overlap_point(w, v2(600, -50), found, 16) == 1 && found[0] == zid, "p2d overlap point finds the trigger");
        check(phys2d_overlap(w, phys2d_circle(5), v2(0, -5), 0, found, 16, PHYS_LAYER_1) == 0, "p2d overlap mask");

        // fixed step bookkeeping
        w.accumulator = 0;
        int steps = phys2d_update(w, 0.04f);
        check(steps == 2, "p2d update runs whole fixed steps");
        check(w.alpha > 0.35f && w.alpha < 0.45f, "p2d update leaves the remainder as alpha");
        check(phys2d_update(w, 10.0f) == w.max_steps, "p2d update caps steps after a hitch");

        // removal: slot reuse waits a step and the end event fires
        phys2d_remove_body(w, stack[0]);
        check(phys2d_get_body(w, stack[0]) == 0, "p2d removed body is gone");
        phys2d_step(w, 1.0f / 60.0f);
        bool ended = false;
        for (size_t e = 0; e < w.event_count; e++)
            ended |= w.events[e].type == PHYS2D_CONTACT_END && (w.events[e].a == stack[0] || w.events[e].b == stack[0]);
        check(ended, "p2d removing a body ends its contacts");
        phys2d_destroy_world(w);
    }

    // ---- 2D joints and sleeping ----
    {
        const float dt = 1.0f / 60.0f;
        phys2d_world w;
        phys2d_create_world(w);
        w.allow_sleep = false;

        // pendulum: a revolute pin to the world keeps the bob at its radius
        idx bob = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_circle(10), v2(100, 0));
        idx pin = phys2d_add_revolute(w, PIX_INVALID, bob, v2(0, 0));
        float worst = 0, lowest = -1e9f;
        for (int i = 0; i < 300; i++) {
            phys2d_step(w, dt);
            phys2d_body* b = phys2d_get_body(w, bob);
            worst = maxf(worst, fabsf(v2len(b->position) - 100.0f));
            lowest = maxf(lowest, b->position.y);
        }
        check(worst < 1.5f, "p2d revolute pendulum keeps its length");
        check(lowest > 95.0f, "p2d pendulum swings down through the bottom");
        check(phys2d_get_joint(w, pin)->reaction > 0.0f, "p2d joint reports its reaction force");

        // revolute limit: a hinged bar may only swing 0.5 rad either way
        idx bar = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(100, 10), v2(550, 0));
        idx hinge = phys2d_add_revolute(w, PIX_INVALID, bar, v2(500, 0));
        phys2d_get_joint(w, hinge)->enable_limit = true;
        phys2d_get_joint(w, hinge)->lower = -0.5f;
        phys2d_get_joint(w, hinge)->upper = 0.5f;
        float max_angle = 0;
        for (int i = 0; i < 240; i++) { phys2d_step(w, dt); max_angle = maxf(max_angle, phys2d_get_body(w, bar)->angle); }
        check(max_angle < 0.56f && max_angle > 0.45f, "p2d revolute limit stops the swing at the upper angle");

        // revolute motor spins a wheel at its set speed
        idx wheel = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_circle(20), v2(-500, 0));
        idx axle = phys2d_add_revolute(w, PIX_INVALID, wheel, v2(-500, 0));
        phys2d_joint* aj = phys2d_get_joint(w, axle);
        aj->enable_motor = true; aj->motor_speed = 5.0f; aj->max_motor_force = 1e9f;
        for (int i = 0; i < 60; i++) phys2d_step(w, dt);
        near_eq(phys2d_get_body(w, wheel)->angular_velocity, 5.0f, 0.05f, "p2d revolute motor reaches its speed");
        near_eq(phys2d_get_body(w, wheel)->position.x, -500.0f, 0.5f, "p2d motor wheel stays on its axle");

        // prismatic: a block on a vertical rail drops to the lower stop (y down = +)
        idx car = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(20, 20), v2(1000, 0));
        idx rail = phys2d_add_prismatic(w, PIX_INVALID, car, v2(1000, 0), v2(0, 1));
        phys2d_joint* rj = phys2d_get_joint(w, rail);
        rj->enable_limit = true; rj->lower = -20.0f; rj->upper = 50.0f;
        for (int i = 0; i < 120; i++) phys2d_step(w, dt);
        phys2d_body* cb = phys2d_get_body(w, car);
        near_eq(cb->position.y, 50.0f, 1.0f, "p2d prismatic limit holds the block at its stop");
        near_eq(cb->position.x, 1000.0f, 0.1f, "p2d prismatic keeps the block on its axis");
        near_eq(cb->angle, 0.0f, 0.01f, "p2d prismatic keeps the block from rotating");
        rj->enable_motor = true; rj->motor_speed = -100.0f; rj->max_motor_force = 1e9f;
        for (int i = 0; i < 20; i++) phys2d_step(w, dt);
        near_eq(cb->velocity.y, -100.0f, 2.0f, "p2d prismatic motor drives the block");

        // a hanging chain of rigid rods keeps every link length
        idx prev = PIX_INVALID;
        vec2 top = v2(2000, -200);
        idx links[10];
        for (int i = 0; i < 10; i++) {
            phys2d_body link = phys2d_make_body(PHYS2D_DYNAMIC, phys2d_circle(4), top + v2(20.0f * (i + 1), 0));
            link.linear_damping = 1.5f;     // air drag, so the swing settles
            links[i] = phys2d_add_body(w, link);
            phys2d_add_distance(w, prev, links[i], i ? top + v2(20.0f * i, 0) : top, top + v2(20.0f * (i + 1), 0));
            prev = links[i];
        }
        float chain_err = 0;
        for (int i = 0; i < 300; i++) {
            phys2d_step(w, dt);
            for (int k = 0; k < 10; k++) {
                vec2 a = k ? phys2d_get_body(w, links[k - 1])->position : top;
                chain_err = maxf(chain_err, fabsf(v2dist(a, phys2d_get_body(w, links[k])->position) - 20.0f));
            }
        }
        printf("info  chain worst link error %.3f\n", chain_err);
        check(chain_err < 2.0f, "p2d distance chain holds its link lengths");
        for (int i = 0; i < 600; i++) phys2d_step(w, dt);
        check(phys2d_get_body(w, links[9])->position.y > top.y + 190.0f, "p2d chain settles hanging straight down");

        // spring: settles stretched below its rest length, without exploding
        idx weight = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_circle(10), v2(3000, 100));
        idx spring = phys2d_add_distance(w, PIX_INVALID, weight, v2(3000, 0), v2(3000, 100));
        phys2d_joint* sj = phys2d_get_joint(w, spring);
        sj->enable_spring = true; sj->hertz = 2.0f; sj->damping_ratio = 0.5f;
        for (int i = 0; i < 600; i++) phys2d_step(w, dt);
        float stretch = phys2d_get_body(w, weight)->position.y - 100.0f;
        check(stretch > 5.0f && stretch < 200.0f, "p2d spring sags under the weight and settles");
        check(fabsf(phys2d_get_body(w, weight)->velocity.y) < 5.0f, "p2d spring comes to rest");

        // rope: slack inside its length, taut at it
        idx load = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_circle(10), v2(4000, 30));
        idx rope = phys2d_add_distance(w, PIX_INVALID, load, v2(4000, 0), v2(4000, 30));
        phys2d_joint* ro = phys2d_get_joint(w, rope);
        ro->enable_spring = true; ro->hertz = 0; ro->enable_limit = true; ro->lower = 0; ro->upper = 120;
        for (int i = 0; i < 10; i++) phys2d_step(w, dt);
        check(phys2d_get_body(w, load)->velocity.y > 100.0f, "p2d rope is slack inside its length");
        for (int i = 0; i < 300; i++) phys2d_step(w, dt);
        near_eq(phys2d_get_body(w, load)->position.y, 120.0f, 2.0f, "p2d rope holds at its length");

        // weld: a cantilever glued to the world barely sags
        idx beam = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(80, 10), v2(5040, 0));
        phys2d_add_weld(w, PIX_INVALID, beam, v2(5000, 0));
        for (int i = 0; i < 120; i++) phys2d_step(w, dt);
        check(fabsf(phys2d_get_body(w, beam)->angle) < 0.05f && fabsf(phys2d_get_body(w, beam)->position.y) < 2.0f,
              "p2d weld holds a cantilever");

        // mouse joint drags a body to the target
        idx toy = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(30, 30), v2(6000, 0));
        idx grab = phys2d_add_mouse(w, toy, v2(6000, 0));
        phys2d_get_joint(w, grab)->target = v2(6100, -100);
        for (int i = 0; i < 180; i++) phys2d_step(w, dt);
        check(v2dist(phys2d_get_body(w, toy)->position, v2(6100, -100)) < 5.0f, "p2d mouse joint drags to the target");
        phys2d_remove_joint(w, grab);
        check(phys2d_get_joint(w, grab) == 0, "p2d removed joint is gone");

        // a weak joint snaps under load and reports it
        idx heavy = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(40, 40), v2(7000, 40));
        idx weak = phys2d_add_revolute(w, PIX_INVALID, heavy, v2(7000, 0));
        phys2d_get_joint(w, weak)->break_force = 1000.0f;
        bool snapped = false;
        for (int i = 0; i < 30; i++) {
            phys2d_step(w, dt);
            for (size_t e = 0; e < w.event_count; e++) snapped |= w.events[e].type == PHYS2D_JOINT_BREAK && w.events[e].joint == weak;
        }
        check(snapped && phys2d_get_joint(w, weak) == 0, "p2d break_force snaps the joint");
        check(phys2d_get_body(w, heavy)->position.y > 100.0f, "p2d broken joint lets go");

        // jointed bodies do not collide unless asked to
        idx ja = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(40, 40), v2(8000, 0));
        idx jb = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(40, 40), v2(8010, 0));
        phys2d_add_weld(w, ja, jb, v2(8005, 0));
        phys2d_step(w, dt);
        bool pair = false;
        for (size_t i = 0; i < w.contact_count; i++) pair |= (w.contacts[i].a == ja && w.contacts[i].b == jb);
        check(!pair, "p2d jointed bodies skip their contact");
        phys2d_destroy_world(w);

        // ---- sleeping ----
        phys2d_create_world(w);
        phys2d_add(w, PHYS2D_STATIC, phys2d_box(2000, 40), v2(0, 20));
        idx s0 = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(30, 30), v2(0, -15));
        idx s1 = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(30, 30), v2(0, -45));
        bool ended_while_asleep = false;
        for (int i = 0; i < 120; i++) {
            phys2d_step(w, dt);
            for (size_t e = 0; e < w.event_count; e++) ended_while_asleep |= w.events[e].type == PHYS2D_CONTACT_END;
        }
        check(phys2d_get_body(w, s0)->sleeping && phys2d_get_body(w, s1)->sleeping, "p2d resting stack falls asleep");
        check(!ended_while_asleep, "p2d sleeping keeps its contacts (no end events)");
        check(phys2d_get_body(w, s1)->on_ground, "p2d sleeping body stays grounded");
        vec2 rest = phys2d_get_body(w, s1)->position;
        for (int i = 0; i < 60; i++) phys2d_step(w, dt);
        check(phys2d_get_body(w, s1)->position.x == rest.x && phys2d_get_body(w, s1)->position.y == rest.y, "p2d sleeping body does not drift");

        idx dropper = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(30, 30), v2(0, -200));
        bool woke = false;
        for (int i = 0; i < 60 && !woke; i++) { phys2d_step(w, dt); woke = !phys2d_get_body(w, s0)->sleeping; }
        check(woke, "p2d a box landing on a sleeping stack wakes the whole island");
        for (int i = 0; i < 120; i++) phys2d_step(w, dt);
        check(phys2d_get_body(w, dropper)->sleeping, "p2d the stack sleeps again");
        phys2d_get_body(w, s1)->velocity = v2(0, -300);
        phys2d_step(w, dt);
        check(!phys2d_get_body(w, s1)->sleeping, "p2d setting a velocity wakes a body");
        phys2d_get_body(w, s1)->velocity = v2(0, 0);
        for (int i = 0; i < 180; i++) phys2d_step(w, dt);
        phys2d_remove_body(w, s0);
        for (int i = 0; i < 30; i++) phys2d_step(w, dt);
        check(phys2d_get_body(w, s1)->position.y > -40.0f, "p2d removing the support wakes what rested on it");
        phys2d_destroy_world(w);
    }

    // ---- 3D physics world ----
    {
        const float dt = 1.0f / 60.0f;
        quat qi = quat_identity();
        phys3d__core ca, cb;
        phys3d_contact m;

        // narrowphase, shape pair by shape pair
        phys3d__make_core(phys3d_sphere(1), v3(0, 0, 0), qi, ca);
        phys3d__make_core(phys3d_sphere(1), v3(1.5f, 0, 0), qi, cb);
        check(phys3d__collide(ca, cb, 0.005f, 0, m) == 1, "p3d sphere/sphere one point");
        near_eq(m.points[0].separation, -0.5f, 1e-4f, "p3d sphere/sphere depth");
        near_eq(m.normal.x, 1.0f, 1e-4f, "p3d sphere/sphere normal a to b");

        phys3d__make_core(phys3d_box(v3(2, 2, 2)), v3(0, 0.99f, 0), qi, ca);
        phys3d__make_core(phys3d_plane(v3(0, 1, 0)), v3(0, 0, 0), qi, cb);
        check(phys3d__collide(ca, cb, 0.005f, 0, m) == 4, "p3d box on plane has four points");
        near_eq(m.points[0].separation, -0.01f, 1e-4f, "p3d box/plane depth");
        near_eq(m.normal.y, -1.0f, 1e-4f, "p3d box/plane normal points into the plane");

        phys3d__make_core(phys3d_box(v3(4, 1, 4)), v3(0, 0, 0), qi, ca);
        phys3d__make_core(phys3d_box(v3(1, 1, 1)), v3(0.3f, 0.99f, -0.2f), quat_from_axis_angle(v3(0, 1, 0), 0.4f), cb);
        check(phys3d__collide(ca, cb, 0.005f, 0, m) == 4, "p3d box resting on box face has four points");
        near_eq(m.normal.y, 1.0f, 1e-4f, "p3d box/box face normal up");
        near_eq(m.points[0].separation, -0.01f, 1e-3f, "p3d box/box depth");

        // two boxes meeting edge to edge
        quat ea = quat_from_axis_angle(v3(0, 0, 1), PIX_PI * 0.25f);
        quat eb = quat_from_axis_angle(v3(1, 0, 0), PIX_PI * 0.25f);
        phys3d__make_core(phys3d_box(v3(1, 1, 1)), v3(0, 0, 0), ea, ca);
        phys3d__make_core(phys3d_box(v3(1, 1, 1)), v3(0, 1.40f, 0), eb, cb);
        check(phys3d__collide(ca, cb, 0.005f, 0, m) == 1, "p3d crossed box edges give one point");
        near_eq(m.normal.y, 1.0f, 1e-3f, "p3d edge/edge normal");
        near_eq(m.points[0].separation, 1.40f - 2 * 0.70710678f, 1e-3f, "p3d edge/edge depth");

        phys3d__make_core(phys3d_capsule(0.25f, 2.0f), v3(0, 1.24f, 0), quat_from_axis_angle(v3(0, 0, 1), PIX_PI * 0.5f), ca);
        phys3d__make_core(phys3d_box(v3(4, 2, 4)), v3(0, 0, 0), qi, cb);
        check(phys3d__collide(ca, cb, 0.005f, 0, m) == 2, "p3d capsule lying on a box has two points");
        near_eq(m.points[0].separation, -0.01f, 1e-3f, "p3d capsule/box depth");
        near_eq(m.normal.y, -1.0f, 1e-3f, "p3d capsule/box normal from capsule to box");

        quat lie = quat_from_axis_angle(v3(0, 0, 1), PIX_PI * 0.5f);
        phys3d__make_core(phys3d_capsule(0.25f, 2.0f), v3(0, 0, 0), lie, ca);
        phys3d__make_core(phys3d_capsule(0.25f, 2.0f), v3(0.5f, 0.45f, 0), lie, cb);
        check(phys3d__collide(ca, cb, 0.005f, 0, m) == 2, "p3d parallel capsules have two points");
        near_eq(m.points[0].separation, -0.05f, 1e-3f, "p3d capsule/capsule depth");

        phys3d__make_core(phys3d_sphere(0.2f), v3(0.8f, 0.1f, 0), qi, ca);
        phys3d__make_core(phys3d_box(v3(2, 2, 2)), v3(0, 0, 0), qi, cb);
        check(phys3d__collide(ca, cb, 0.005f, 0, m) == 1, "p3d sphere centre inside a box");
        near_eq(m.normal.x, -1.0f, 1e-3f, "p3d deep sphere leaves through the nearest face");
        near_eq(m.points[0].separation, -0.4f, 1e-3f, "p3d deep sphere depth");

        phys3d__core tri;
        phys3d__triangle_core(v3(-5, 0, -5), v3(-5, 0, 5), v3(5, 0, -5), tri);
        check(tri.axis[0].y > 0.99f, "p3d triangle normal faces up (counter-clockwise from above)");
        phys3d__make_core(phys3d_box(v3(1, 1, 1)), v3(-1, 0.49f, -1), qi, ca);
        check(phys3d__collide(ca, tri, 0.005f, 0, m) == 4, "p3d box on a triangle has four points");
        near_eq(m.normal.y, -1.0f, 1e-3f, "p3d box/triangle normal");
        phys3d__make_core(phys3d_sphere(0.5f), v3(-1, 0.45f, -1), qi, ca);
        check(phys3d__collide(ca, tri, 0.005f, 0, m) == 1, "p3d sphere on a triangle");
        near_eq(m.points[0].separation, -0.05f, 1e-3f, "p3d sphere/triangle depth");

        vec3 gp, gq;
        phys3d__make_core(phys3d_box(v3(2, 2, 2)), v3(0, 0, 0), quat_from_axis_angle(v3(0, 1, 0), 0.3f), ca);
        phys3d__make_core(phys3d_sphere(0.1f), v3(0, 4, 0), qi, cb);
        near_eq(phys3d__gjk(ca, cb, &gp, &gq), 3.0f, 1e-3f, "p3d gjk box to point distance");
        phys3d__make_core(phys3d_capsule(0.3f, 2.0f), v3(3, 0, 0), qi, cb);
        // the segment at (3, 0) seen from the box's frame sits at (3 cos 0.3, 3 sin 0.3): in front of the +x face
        near_eq(phys3d__gjk(ca, cb, &gp, &gq), 3.0f * cosf(0.3f) - 1.0f, 2e-3f, "p3d gjk rotated box to segment distance");

        // ---- simulation ----
        phys3d_world w;
        phys3d_create_world(w);
        w.allow_sleep = false;
        phys3d_add(w, PHYS3D_STATIC, phys3d_plane(v3(0, 1, 0)), v3(0, 0, 0));

        // a stack of six boxes stands and settles
        idx stack[6];
        for (int i = 0; i < 6; i++) stack[i] = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(1, 1, 1)), v3(0, 0.5f + i * 1.01f, 0));
        for (int i = 0; i < 360; i++) phys3d_step(w, dt);
        phys3d_body* top = phys3d_get_body(w, stack[5]);
        printf("info  3d stack top at %.4f %.4f %.4f\n", top->position.x, top->position.y, top->position.z);
        near_eq(top->position.x, 0.0f, 0.02f, "p3d stack of 6 stays upright (x)");
        near_eq(top->position.z, 0.0f, 0.02f, "p3d stack of 6 stays upright (z)");
        near_eq(top->position.y, 5.5f, 0.06f, "p3d stack rests at its height");
        check(v3len(top->velocity) < 0.02f && v3len(top->angular_velocity) < 0.05f, "p3d stack is at rest");
        check(phys3d_get_body(w, stack[0])->on_ground, "p3d bottom box is grounded");

        // a pyramid (each box on two) holds its shape
        idx apex = PIX_INVALID;
        for (int row = 0; row < 5; row++)
            for (int i = 0; i < 5 - row; i++)
                apex = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(1, 1, 1)), v3(10 + (i - (4 - row) * 0.5f) * 1.05f, 0.5f + row * 1.0f, 0));
        for (int i = 0; i < 360; i++) phys3d_step(w, dt);
        phys3d_body* ap = phys3d_get_body(w, apex);
        near_eq(ap->position.x, 10.0f, 0.05f, "p3d pyramid apex stays centred");
        near_eq(ap->position.y, 4.5f, 0.08f, "p3d pyramid keeps its height");

        // a ball rolls down a ramp: rolling, not sliding, so v ~ w * r
        phys3d_body ramp = phys3d_make_body(PHYS3D_STATIC, phys3d_box(v3(10, 0.4f, 4)), v3(-20, 2, 0));
        ramp.rotation = quat_from_axis_angle(v3(0, 0, 1), -0.3f);    // falls toward +x
        phys3d_add_body(w, ramp);
        idx ball = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_sphere(0.5f), v3(-23, 3.6f, 0));
        for (int i = 0; i < 60; i++) phys3d_step(w, dt);
        phys3d_body* bb = phys3d_get_body(w, ball);
        float roll = v3len(bb->angular_velocity) * 0.5f, speed = v3len(bb->velocity);
        printf("info  3d rolling ball: speed %.3f, spin * r %.3f\n", speed, roll);
        check(speed > 1.0f && fabsf(roll - speed) < 0.1f * speed, "p3d ball rolls without slipping down a ramp");
        check(bb->velocity.x > 0.5f, "p3d ball rolls downhill");

        // restitution and tunnelling
        phys3d_body bouncy = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_sphere(0.3f), v3(30, 5, 0));
        bouncy.restitution = 0.8f;
        idx bid = phys3d_add_body(w, bouncy);
        float peak = 0; bool landed = false;
        for (int i = 0; i < 150; i++) {
            phys3d_step(w, dt);
            for (size_t e = 0; e < w.event_count; e++)
                landed |= w.events[e].type == PHYS3D_CONTACT_BEGIN && (w.events[e].a == bid || w.events[e].b == bid) && w.events[e].speed > 5.0f;
            phys3d_body* b = phys3d_get_body(w, bid);
            if (landed && b->velocity.y < 0) peak = maxf(peak, b->position.y);
        }
        check(landed, "p3d contact begin reports the impact speed");
        check(peak > 2.5f, "p3d restitution bounces back up");

        phys3d_add(w, PHYS3D_STATIC, phys3d_box(v3(0.05f, 4, 4)), v3(45, 2, 0));
        phys3d_body bullet = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_sphere(0.05f), v3(40, 2, 0));
        bullet.velocity = v3(300, 0, 0);
        bullet.gravity_scale = 0;
        idx bul = phys3d_add_body(w, bullet);
        for (int i = 0; i < 30; i++) phys3d_step(w, dt);
        check(phys3d_get_body(w, bul)->position.x < 45, "p3d speculative contacts stop a 300 m/s bullet at a thin wall");
        phys3d_remove_body(w, bul);

        // friction on a slope: grippy stays, icy slides
        phys3d_body slope = phys3d_make_body(PHYS3D_STATIC, phys3d_box(v3(20, 0.4f, 4)), v3(60, 2, 0));
        slope.rotation = quat_from_axis_angle(v3(0, 0, 1), -0.35f);
        phys3d_add_body(w, slope);
        phys3d_body grip = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_box(v3(0.6f, 0.6f, 0.6f)), v3(60, 0, -1));
        grip.rotation = slope.rotation;
        grip.position = v3(60, 2, -1) + quat_rotate(slope.rotation, v3(0, 0.5f, 0));
        grip.friction = 1.0f;
        idx gid = phys3d_add_body(w, grip);
        grip.friction = 0.0f; grip.position.z = 1;
        idx iid = phys3d_add_body(w, grip);
        vec3 g0 = phys3d_get_body(w, gid)->position, i0 = phys3d_get_body(w, iid)->position;
        for (int i = 0; i < 90; i++) phys3d_step(w, dt);
        check(v3dist(phys3d_get_body(w, gid)->position, g0) < 0.05f, "p3d high friction holds a box on a slope");
        check(v3dist(phys3d_get_body(w, iid)->position, i0) > 1.0f, "p3d zero friction lets it slide");
        phys3d_destroy_world(w);

        // ---- meshes ----
        phys3d_create_world(w);
        w.allow_sleep = false;
        float heights[11 * 11] = {};
        phys3d_mesh floor_mesh;
        check(phys3d_create_heightfield(floor_mesh, heights, 11, 11, 1.0f), "p3d heightfield builds");
        check(floor_mesh.triangle_count == 200, "p3d heightfield triangle count");
        idx ground = phys3d_add(w, PHYS3D_STATIC, phys3d_mesh_shape(floor_mesh), v3(0, 0, 0));
        idx mbox = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(1, 1, 1)), v3(3.3f, 2, 3.7f));
        phys3d_body rolling = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_sphere(0.3f), v3(1, 0.3f, 5.5f));
        rolling.velocity = v3(4, 0, 0);
        idx rid = phys3d_add_body(w, rolling);
        float lowest = 1e9f, highest = -1e9f;
        for (int i = 0; i < 120; i++) {
            phys3d_step(w, dt);
            if (i > 10 && phys3d_get_body(w, rid)->position.x < 9.5f) {
                lowest = minf(lowest, phys3d_get_body(w, rid)->position.y);
                highest = maxf(highest, phys3d_get_body(w, rid)->position.y);
            }
        }
        near_eq(phys3d_get_body(w, mbox)->position.y, 0.5f, 0.02f, "p3d box rests on a mesh");
        check(highest - lowest < 0.02f, "p3d ball rolls across mesh triangles without bumping");
        phys3d_ray_hit hit;
        check(phys3d_raycast(w, v3(7.2f, 5, 2.3f), v3(0, -1, 0), 10, &hit) && hit.body == ground, "p3d raycast hits the mesh");
        near_eq(hit.point.y, 0.0f, 1e-4f, "p3d mesh hit point");
        near_eq(hit.normal.y, 1.0f, 1e-4f, "p3d mesh hit normal");
        check(!phys3d_raycast(w, v3(7.2f, -5, 2.3f), v3(0, 1, 0), 10, &hit), "p3d one-sided mesh: rays from below pass");
        phys3d_destroy_world(w);
        phys3d_destroy_mesh(floor_mesh);

        // ---- triggers, layers, queries ----
        phys3d_create_world(w);
        w.allow_sleep = false;
        idx floor_id = phys3d_add(w, PHYS3D_STATIC, phys3d_plane(v3(0, 1, 0)), v3(0, 0, 0));
        phys3d_body zone = phys3d_make_body(PHYS3D_STATIC, phys3d_box(v3(2, 2, 2)), v3(5, 1, 0));
        zone.trigger = true;
        idx zid = phys3d_add_body(w, zone);
        phys3d_body walker = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_capsule(0.3f, 1.8f), v3(0, 0.9f, 0));
        walker.fixed_rotation = true;
        walker.friction = 0;
        idx wid = phys3d_add_body(w, walker);
        int enters = 0, exits = 0;
        for (int i = 0; i < 180; i++) {
            phys3d_get_body(w, wid)->velocity.x = 4;
            phys3d_step(w, dt);
            for (size_t e = 0; e < w.event_count; e++) {
                if (w.events[e].type == PHYS3D_TRIGGER_ENTER && w.events[e].a == zid && w.events[e].b == wid) enters++;
                if (w.events[e].type == PHYS3D_TRIGGER_EXIT && w.events[e].a == zid && w.events[e].b == wid) exits++;
            }
        }
        check(enters == 1 && exits == 1, "p3d trigger enter and exit fire once each");
        check(phys3d_get_body(w, wid)->position.x > 10, "p3d trigger does not block");
        check(phys3d_get_body(w, wid)->on_ground, "p3d capsule walker is grounded");
        check(v3len(phys3d_get_body(w, wid)->angular_velocity) == 0.0f, "p3d fixed rotation never spins");

        phys3d_body ghost = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_sphere(0.3f), v3(-5, 1, 0));
        ghost.mask = PHYS_LAYER_ALL & ~PHYS_LAYER_0;
        idx ghost_id = phys3d_add_body(w, ghost);
        for (int i = 0; i < 60; i++) phys3d_step(w, dt);
        check(phys3d_get_body(w, ghost_id)->position.y < -1, "p3d layer mask lets a ghost fall through");

        idx crate = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(1, 1, 1)), v3(-10, 0.5f, 0));
        for (int i = 0; i < 30; i++) phys3d_step(w, dt);
        check(phys3d_raycast(w, v3(-10, 5, 0), v3(0, -1, 0), 10, &hit) && hit.body == crate, "p3d raycast hits the crate");
        near_eq(hit.point.y, 1.0f, 0.01f, "p3d raycast point on the crate top");
        near_eq(hit.normal.y, 1.0f, 1e-3f, "p3d raycast normal faces the ray");
        check(phys3d_raycast(w, v3(-10, 5, 0), v3(0, -1, 0), 10, &hit, PHYS_LAYER_ALL, crate) && hit.body == floor_id,
              "p3d raycast ignore skips the crate");
        check(phys3d_raycast(w, v3(-30, 0.5f, 0.2f), v3(1, 0, 0), 100, &hit) && hit.body == crate, "p3d long raycast walks the grid");
        near_eq(hit.point.x, -10.5f, 0.01f, "p3d long raycast hits the near face");
        check(phys3d_shape_cast(w, phys3d_sphere(0.5f), v3(-10, 5, 0), qi, v3(0, -1, 0), 10, &hit) && hit.body == crate,
              "p3d sphere cast hits the crate");
        near_eq(hit.distance, 3.5f, 0.01f, "p3d sphere cast distance to touch");
        check(phys3d_shape_cast(w, phys3d_box(v3(1, 1, 1)), v3(-20, 5, 0), qi, v3(0, -1, 0), 10, &hit) && hit.body == floor_id,
              "p3d box cast hits the floor");
        near_eq(hit.distance, 4.5f, 0.01f, "p3d box cast distance");
        idx found[16];
        size_t n = phys3d_overlap_sphere(w, v3(-10, 0.5f, 0), 1.0f, found, 16);
        check(n == 2, "p3d overlap sphere finds the crate and the floor");
        check(phys3d_overlap_sphere(w, v3(5, 1, 0), 0.5f, found, 16) == 1 && found[0] == zid, "p3d overlap finds the trigger");

        phys3d_remove_body(w, crate);
        phys3d_step(w, dt);
        bool ended = false;
        for (size_t e = 0; e < w.event_count; e++) ended |= w.events[e].type == PHYS3D_CONTACT_END && (w.events[e].a == crate || w.events[e].b == crate);
        check(ended, "p3d removing a body ends its contacts");
        phys3d_destroy_world(w);

        // ---- joints ----
        phys3d_create_world(w);
        w.allow_sleep = false;
        // pendulum on a ball joint
        idx bob = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_sphere(0.2f), v3(2, 10, 0));
        phys3d_add_ball(w, PIX_INVALID, bob, v3(0, 10, 0));
        float worst = 0;
        for (int i = 0; i < 240; i++) { phys3d_step(w, dt); worst = maxf(worst, fabsf(v3dist(phys3d_get_body(w, bob)->position, v3(0, 10, 0)) - 2.0f)); }
        check(worst < 0.01f, "p3d ball joint pendulum keeps its length");

        // hinge door: stays on its axis, the limit stops it, a motor drives it
        idx door = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(1, 2, 0.1f)), v3(10.5f, 1.1f, 0));
        idx hinge = phys3d_add_hinge(w, PIX_INVALID, door, v3(10, 1.1f, 0), v3(0, 1, 0));
        phys3d_joint* hj = phys3d_get_joint(w, hinge);
        hj->enable_motor = true; hj->motor_speed = 1.0f; hj->max_motor_force = 1000;
        hj->enable_limit = true; hj->lower = -0.5f; hj->upper = 1.2f;
        for (int i = 0; i < 30; i++) phys3d_step(w, dt);
        near_eq(phys3d_get_body(w, door)->angular_velocity.y, 1.0f, 0.02f, "p3d hinge motor speed");
        for (int i = 0; i < 120; i++) phys3d_step(w, dt);
        near_eq(phys3d_hinge_angle(w, *hj), 1.2f, 0.02f, "p3d hinge limit stops the door");
        phys3d_body* db = phys3d_get_body(w, door);
        vec3 da, dbp;
        phys3d_joint_anchors(w, *hj, &da, &dbp);
        check(v3dist(da, dbp) < 0.005f, "p3d hinge keeps its pivot");
        vec3 door_up = quat_rotate(db->rotation, v3(0, 1, 0));
        check(door_up.y > 0.999f, "p3d hinge keeps the door upright (axis held)");

        // a chain of ball joints hangs straight
        idx prev = PIX_INVALID;
        vec3 top_p = v3(20, 10, 0);
        idx links[8];
        for (int i = 0; i < 8; i++) {
            phys3d_body l = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_capsule(0.05f, 0.5f), top_p + v3(0.5f * i + 0.25f, 0, 0));
            l.rotation = quat_from_axis_angle(v3(0, 0, 1), PIX_PI * 0.5f);
            l.linear_damping = 0.5f;
            links[i] = phys3d_add_body(w, l);
            phys3d_add_ball(w, prev, links[i], top_p + v3(0.5f * i, 0, 0));
            prev = links[i];
        }
        float chain = 0;
        for (int i = 0; i < 600; i++) {
            phys3d_step(w, dt);
            for (int k = 1; k < 8; k++) {
                const phys3d_joint& j = w.joints[k + 2];
                vec3 a, b2; phys3d_joint_anchors(w, j, &a, &b2);
                chain = maxf(chain, v3dist(a, b2));
            }
        }
        printf("info  3d chain worst joint gap %.4f\n", chain);
        check(chain < 0.02f, "p3d ball chain holds together");
        check(phys3d_get_body(w, links[7])->position.y < 6.5f, "p3d chain hangs down");

        // swing cone: a rod on a ball joint may not swing past 30 degrees
        idx rod = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_capsule(0.05f, 1.0f), v3(30, 9.5f, 0));
        idx cone = phys3d_add_ball(w, PIX_INVALID, rod, v3(30, 10, 0), v3(0, -1, 0));
        phys3d_get_joint(w, cone)->swing_limit = PIX_PI / 6.0f;
        phys3d_get_body(w, rod)->velocity = v3(5, 0, 0);
        float max_swing = 0;
        for (int i = 0; i < 120; i++) {
            phys3d_step(w, dt);
            vec3 down = quat_rotate(phys3d_get_body(w, rod)->rotation, v3(0, -1, 0));
            max_swing = maxf(max_swing, acosf(clampf(-down.y, -1, 1)));
        }
        check(max_swing < PIX_PI / 6.0f + 0.05f, "p3d swing cone limits the rod");

        // slider: a block on a vertical rail drops to its lower stop without turning
        idx car = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(0.5f, 0.5f, 0.5f)), v3(40, 10, 0));
        idx rail = phys3d_add_slider(w, PIX_INVALID, car, v3(40, 10, 0), v3(0, 1, 0));
        phys3d_joint* sj = phys3d_get_joint(w, rail);
        sj->enable_limit = true; sj->lower = -2.0f; sj->upper = 1.0f;
        phys3d_get_body(w, car)->angular_velocity = v3(3, 2, 1);
        for (int i = 0; i < 120; i++) phys3d_step(w, dt);
        phys3d_body* cb2 = phys3d_get_body(w, car);
        near_eq(cb2->position.y, 8.0f, 0.02f, "p3d slider limit holds the block");
        check(fabsf(cb2->position.x - 40) < 0.005f && fabsf(cb2->position.z) < 0.005f, "p3d slider keeps the block on its axis");
        check(fabsf(cb2->rotation.w) > 0.9999f, "p3d slider stops rotation");

        // fixed: a cantilever glued to the world barely sags
        idx beam = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(2, 0.2f, 0.2f)), v3(51, 10, 0));
        phys3d_add_fixed(w, PIX_INVALID, beam, v3(50, 10, 0));
        for (int i = 0; i < 120; i++) phys3d_step(w, dt);
        check(fabsf(phys3d_get_body(w, beam)->position.y - 10) < 0.02f, "p3d fixed joint holds a cantilever");

        // rope: slack inside its length, taut at it
        idx load = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_sphere(0.2f), v3(60, 9.5f, 0));
        idx rope = phys3d_add_distance(w, PIX_INVALID, load, v3(60, 10, 0), v3(60, 9.5f, 0));
        phys3d_joint* rj = phys3d_get_joint(w, rope);
        rj->enable_spring = true; rj->hertz = 0; rj->enable_limit = true; rj->lower = 0; rj->upper = 3;
        for (int i = 0; i < 15; i++) phys3d_step(w, dt);
        check(phys3d_get_body(w, load)->velocity.y < -1.0f, "p3d rope is slack inside its length");
        for (int i = 0; i < 300; i++) phys3d_step(w, dt);
        near_eq(phys3d_get_body(w, load)->position.y, 7.0f, 0.03f, "p3d rope holds at its length");

        // mouse joint drags a body to the target
        idx toy = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(0.5f, 0.5f, 0.5f)), v3(70, 10, 0));
        idx grab = phys3d_add_mouse(w, toy, v3(70, 10, 0));
        phys3d_get_joint(w, grab)->target = v3(71, 11, 1);
        for (int i = 0; i < 180; i++) phys3d_step(w, dt);
        check(v3dist(phys3d_get_body(w, toy)->position, v3(71, 11, 1)) < 0.05f, "p3d mouse joint drags to the target");

        // a weak joint snaps
        idx heavy = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(1, 1, 1)), v3(80, 9, 0));
        idx weak = phys3d_add_ball(w, PIX_INVALID, heavy, v3(80, 10, 0));
        phys3d_get_joint(w, weak)->break_force = 1000;
        bool snapped = false;
        for (int i = 0; i < 30; i++) {
            phys3d_step(w, dt);
            for (size_t e = 0; e < w.event_count; e++) snapped |= w.events[e].type == PHYS3D_JOINT_BREAK && w.events[e].joint == weak;
        }
        check(snapped && !phys3d_get_joint(w, weak), "p3d break_force snaps the joint");
        phys3d_destroy_world(w);

        // ---- sleeping ----
        phys3d_create_world(w);
        phys3d_add(w, PHYS3D_STATIC, phys3d_plane(v3(0, 1, 0)), v3(0, 0, 0));
        idx s0 = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(1, 1, 1)), v3(0, 0.5f, 0));
        idx s1 = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(1, 1, 1)), v3(0, 1.5f, 0));
        bool ended_asleep = false;
        for (int i = 0; i < 120; i++) {
            phys3d_step(w, dt);
            for (size_t e = 0; e < w.event_count; e++) ended_asleep |= w.events[e].type == PHYS3D_CONTACT_END;
        }
        check(phys3d_get_body(w, s0)->sleeping && phys3d_get_body(w, s1)->sleeping, "p3d resting stack falls asleep");
        check(!ended_asleep, "p3d sleeping keeps its contacts");
        vec3 rest = phys3d_get_body(w, s1)->position;
        for (int i = 0; i < 60; i++) phys3d_step(w, dt);
        check(v3dist(phys3d_get_body(w, s1)->position, rest) == 0.0f, "p3d sleeping body does not drift");
        phys3d_add(w, PHYS3D_DYNAMIC, phys3d_sphere(0.3f), v3(0.2f, 5, 0));
        bool woke = false;
        for (int i = 0; i < 90 && !woke; i++) { phys3d_step(w, dt); woke = !phys3d_get_body(w, s0)->sleeping; }
        check(woke, "p3d a ball landing on a sleeping stack wakes the island");

        // a tall stack settles completely: this is what one solver sweep per
        // manifold failed (it rocked corner to corner and crept sideways)
        idx tall[12];
        for (int i = 0; i < 12; i++) tall[i] = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(1, 1, 1)), v3(20, 0.5f + i * 1.0f, 0));
        for (int i = 0; i < 600; i++) phys3d_step(w, dt);
        phys3d_body* tt = phys3d_get_body(w, tall[11]);
        check(tt->sleeping, "p3d a 12-box stack settles and sleeps");
        check(fabsf(tt->position.x - 20) < 0.02f && fabsf(tt->position.z) < 0.02f, "p3d the 12-box stack stays straight");
        phys3d_destroy_world(w);
    }

    if (failures == 0) printf("all checks passed\n");
    else               printf("%d check(s) failed\n", failures);
    return failures;
}
