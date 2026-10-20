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

    // ---- physics ----
    static phys_world w;
    phys_create_world(w);
    phys_add_static_box(w, v3(-20, -1, -20), v3(20, 0, 20));   // floor slab
    phys_add_static_box(w, v3(2, 0, -20), v3(3, 4, 20));       // wall across x = 2..3
    phys_build_statics(w);

    phys_body body = {};
    body.position = v3(0, 5, 0);
    body.shape = PHYS_CYLINDER; body.radius = 0.3f; body.height = 1.8f;
    body.inv_mass = 1.0f; body.gravity = true; body.active = true;
    body.friction = 0.4f;
    body.group = PHYS_LAYER_PLAYER; body.collides = PHYS_LAYER_ALL;
    idx id_body = phys_add_body(w, body);

    for (int i = 0; i < 240; i++) phys_step(w, 1.0f / 60.0f);
    phys_body* bp = phys_get_body(w, id_body);
    check(bp->on_ground, "body came to rest on the ground");
    near_eq(bp->position.y, 0.0f, 1e-3f, "body rests at ground height");

    for (int i = 0; i < 240; i++) {                 // drive it into the wall
        phys_get_body(w, id_body)->velocity.x = 6.0f;
        phys_step(w, 1.0f / 60.0f);
    }
    bp = phys_get_body(w, id_body);
    check(bp->position.x < 2.0f, "body stopped by the wall");
    check(bp->position.x > 1.4f, "body stopped against the wall, not short of it");

    check(phys_raycast(w, v3(0, 1, 0), v3(1, 0, 0), 50.0f, &t), "raycast finds the wall");
    near_eq(t, 2.0f, 0.05f, "raycast distance to the wall");
    phys_destroy_world(w);

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

    if (failures == 0) printf("all checks passed\n");
    else               printf("%d check(s) failed\n", failures);
    return failures;
}
