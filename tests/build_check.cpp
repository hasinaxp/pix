// Compile + behaviour check for the engine headers. Not part of the library.
//   cl /nologo /std:c++14 /EHsc /O2 /W3 /Fe:build\check.exe /Fo:build\ build_check.cpp
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include "src/core/platform.hpp"
#include "src/core/opengl_utils.hpp"
#include "src/core/math.hpp"
#include "src/core/random.hpp"
#include "src/core/colliders.hpp"
#include "src/core/jobs.hpp"
#include "src/core/physics.hpp"
#include "src/core/profile.hpp"
#include "src/core/lighting.hpp"
#include "src/core/framebuffer.hpp"
#include "src/core/renderer.hpp"
#include "src/core/animation.hpp"
#include "src/core/sprite.hpp"
#include "src/core/text.hpp"
#include "src/core/sound.hpp"
#include "src/loader/data_loader.hpp"
#include "src/loader/png_writer.hpp"

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

    // ---- lighting ----
    for (float hour = 0.0f; hour < 24.0f; hour += 0.25f) {
        pix_sun sun; pix_sky sky; float exposure;
        pix_daylight_at(hour, &sun, &sky, &exposure);
        check(exposure > 0.0f && exposure < 100.0f, "exposure stays sane across the day");
        near_eq(v3len(sun.direction), 1.0f, 1e-3f, "sun direction stays normalised");
        float n = pix_night_factor(sun);
        check(n >= 0.0f && n <= 1.0f, "night factor stays in [0,1]");
    }

    if (failures == 0) printf("all checks passed\n");
    else               printf("%d check(s) failed\n", failures);
    return failures;
}
