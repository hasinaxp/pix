#ifndef PIX_PHYSICS3D_HPP
#define PIX_PHYSICS3D_HPP
// dtype.hpp first: it sets up the CRT before any C header
#include "dtype.hpp"
#include <string.h>
#include <math.h>
#include "math.hpp"
#include "memory.hpp"
#include "colliders.hpp"
#include "physics_common.hpp"

// A 3D rigid-body world: boxes, spheres and capsules that tumble, roll,
// stack and rest, on static planes, boxes and triangle meshes, with joints
// for doors, chains, ragdolls and vehicles. Units are metres, y is up,
// gravity is 9.81 down.
//
//   phys3d_world w;
//   phys3d_create_world(w);
//   phys3d_add(w, PHYS3D_STATIC,  phys3d_plane(v3(0, 1, 0)), v3(0, 0, 0));   // the ground
//   idx crate = phys3d_add(w, PHYS3D_DYNAMIC, phys3d_box(v3(1, 1, 1)), v3(0, 5, 0));
//   while (pix_frame()) {
//       phys3d_update(w, pix_dt());
//       phys3d_body* b = phys3d_get_body(w, crate);   // b->position, b->rotation
//       pix_draw_physics3d(w);
//   }
//
// It is the 2D world (physics2d.hpp) lifted to 3D, so the same ideas run
// through both:
//
//   broadphase   hashed uniform grid, statics in their own grid rebuilt only
//                when one changes. Bodies too big for the grid (planes, level
//                meshes) go in an "oversize" list tested against everything.
//   narrowphase  every shape is a convex core plus a radius: a point
//                (sphere), a segment (capsule), a box, or a mesh triangle.
//                Rounded cores use GJK for closest points (with a face clip
//                for a capsule lying flat); box-box and box-triangle use SAT
//                over face and edge axes with reference-face clipping, the
//                method of Gregorius' GDC talks and qu3e - up to four points
//                per manifold, which is what lets boxes stack.
//   contacts     persistent manifolds: points are matched to last step's by
//                where they sit on body a, and inherit their impulses (warm
//                starting). Speculative contacts stop fast bodies tunnelling.
//   solver       sequential impulses with Coulomb friction in two tangent
//                directions, substepped with a relax pass (Box2D v3's soft
//                step); restitution after the solve.
//   joints       ball (with swing cone and twist limits: ragdolls), hinge
//                (limits, motor), slider (limits, motor), fixed, distance
//                (rod, spring, rope) and mouse - Bullet's basic set, as soft
//                constraints.
//   sleeping     islands by union-find; an island that stays still for
//                `time_to_sleep` stops simulating until touched.
//
// Layers work as in every pix world: phys_layers_collide (physics_common.hpp).
//
// Not here yet: convex hulls, compound bodies, gyroscopic torque, threading,
// continuous collision for spinning bodies, internal-edge smoothing on meshes.

#ifndef PHYS3D_MAX_BODIES
#define PHYS3D_MAX_BODIES 4096
#endif
#ifndef PHYS3D_MAX_JOINTS
#define PHYS3D_MAX_JOINTS 4096
#endif
#define PHYS3D_GRID_BUCKETS 8192     // hashed cells per grid (power of two)
#define PHYS3D_OVERSIZE_CELLS 512    // bodies covering more cells than this skip the grid

// body types
#define PHYS3D_STATIC    0
#define PHYS3D_DYNAMIC   1
#define PHYS3D_KINEMATIC 2

// shapes
#define PHYS3D_SPHERE  0
#define PHYS3D_BOX     1
#define PHYS3D_CAPSULE 2     // upright along the body's local y
#define PHYS3D_PLANE   3     // infinite; static or kinematic only
#define PHYS3D_MESH    4     // triangles; static or kinematic only, one-sided (front = counter-clockwise)

// events
#define PHYS3D_CONTACT_BEGIN 0
#define PHYS3D_CONTACT_END   1
#define PHYS3D_TRIGGER_ENTER 2
#define PHYS3D_TRIGGER_EXIT  3
#define PHYS3D_JOINT_BREAK   4

// joints
#define PHYS3D_JOINT_BALL     0   // a shared pivot, free to rotate: chains, ragdoll shoulders (swing / twist limits)
#define PHYS3D_JOINT_HINGE    1   // rotates about one axis: doors, wheels, elbows, knees
#define PHYS3D_JOINT_SLIDER   2   // slides along one axis without turning: pistons, drawers, lifts
#define PHYS3D_JOINT_FIXED    3   // glues two bodies (springy if hertz > 0); breakable things
#define PHYS3D_JOINT_DISTANCE 4   // rod, spring or rope between two points
#define PHYS3D_JOINT_MOUSE    5   // drags a point of a body toward `target`

// debug draw flags
#define PHYS3D_DRAW_SHAPES   1
#define PHYS3D_DRAW_AABBS    2
#define PHYS3D_DRAW_CONTACTS 4
#define PHYS3D_DRAW_JOINTS   8
#define PHYS3D_DRAW_MESHES   16   // every mesh triangle: can be a lot of lines
#define PHYS3D_DRAW_DEFAULT  (PHYS3D_DRAW_SHAPES | PHYS3D_DRAW_CONTACTS | PHYS3D_DRAW_JOINTS)

struct phys3d__bvh_node {
    aabb     bounds;
    uint32_t first;   // leaf: first entry in `order`; inner: the left child (right is first + 1)
    uint32_t count;   // leaf: triangle count; 0 for an inner node
};

// Triangles for static collision: a level, a terrain. Build once, keep it
// alive while bodies use it; the mesh owns copies of the data.
struct phys3d_mesh {
    vec3*     vertices;
    uint32_t* indices;          // three per triangle
    size_t    vertex_count, triangle_count;
    phys3d__bvh_node* nodes;    // bounding volume hierarchy over the triangles
    uint32_t* order;            // triangle indices, grouped by leaf
    size_t    node_count;
    aabb      bounds;
};

struct phys3d_shape {
    uint8_t type;
    float   radius;             // sphere, capsule
    vec3    half;               // box half extents
    float   half_length;        // capsule: half the straight part
    vec3    normal;             // plane, in the body's frame
    const phys3d_mesh* mesh;
};

static inline phys3d_shape phys3d_sphere(float radius) {
    phys3d_shape s = {}; s.type = PHYS3D_SPHERE; s.radius = radius; return s;
}
// full size, not half
static inline phys3d_shape phys3d_box(vec3 size) {
    phys3d_shape s = {}; s.type = PHYS3D_BOX; s.half = size * 0.5f; return s;
}
// `height` overall, caps included
static inline phys3d_shape phys3d_capsule(float radius, float height) {
    phys3d_shape s = {}; s.type = PHYS3D_CAPSULE; s.radius = radius;
    s.half_length = maxf(0.0f, height * 0.5f - radius);
    return s;
}
// the plane through the body's position, facing `normal`
static inline phys3d_shape phys3d_plane(vec3 normal) {
    phys3d_shape s = {}; s.type = PHYS3D_PLANE; s.normal = v3norm(normal); return s;
}
static inline phys3d_shape phys3d_mesh_shape(const phys3d_mesh& mesh) {
    phys3d_shape s = {}; s.type = PHYS3D_MESH; s.mesh = &mesh; return s;
}

struct phys3d_body {
    phys3d_shape shape;
    vec3  position;
    quat  rotation;
    vec3  velocity;
    vec3  angular_velocity;      // radians per second about each world axis

    uint8_t type;
    bool  trigger;
    bool  fixed_rotation;        // never rotates (characters)
    float density;               // kg per cubic metre; water is 1000
    float friction, restitution;
    // Resists rolling and spinning in place, which friction at a single
    // contact point cannot. 0 = a ball rolls forever; the default 0.01 brings
    // a 5 m/s ball to rest in under a minute and stops piles twirling awake.
    float rolling_friction;
    float linear_damping, angular_damping;
    float gravity_scale;
    uint16_t layer, mask;
    void*    user;
    bool     never_sleep;

    // ---- written by the world ----
    bool  active;
    bool  sleeping;
    float sleep_time;
    bool  on_ground;
    vec3  ground_normal;
    vec3  force, torque;         // accumulated, cleared each step
    float inv_mass;
    vec3  inv_inertia_local;     // diagonal, in the body's frame
    mat3  inv_inertia;           // world space, kept current by the solver
    vec3  prev_position;
    quat  prev_rotation;
    quat  delta_rot;             // how far it turned this step (solver)
    aabb  bounds;
    idx   joint_list;
    uint8_t oversize;            // in the oversize list instead of a grid
};

struct phys3d_contact_point {
    vec3  point;
    float separation;
    float normal_impulse;
    float tangent_impulse[2];
    float max_normal_impulse;
    float approach_speed;
    vec3  local_a;               // the point in a's frame: matches it to last step's
    vec3  ra, rb;
    float normal_mass, tangent_mass[2], bias;
};

struct phys3d_contact {
    idx      a, b;               // a < b; normal points from a to b
    uint32_t sub;                // which mesh triangle, for mesh pairs
    vec3     normal;
    vec3     tangent[2];
    int      count;              // 0..4
    phys3d_contact_point points[4];
    float    friction, restitution;
    float    rolling;            // rolling friction times a contact radius
    vec3     rolling_impulse;
    mat3     rolling_mass;       // inverse of the summed inverse inertias
    bool     trigger, touching, idle, persisted;
    uint32_t prev;               // first contact of this pair last step, or 0xFFFFFFFF
};

struct phys3d_event {
    uint8_t type;
    idx     a, b;                // triggers: a is the trigger. PIX_INVALID = the world
    idx     joint;
    vec3    point, normal;
    float   speed;
};

struct phys3d_ray_hit {
    idx      body;
    uint32_t triangle;           // mesh hits: which triangle
    vec3     point, normal;
    float    distance;
};

struct phys3d_joint {
    uint8_t type;
    bool    active;
    bool    collide_connected;
    idx     a, b;                // PIX_INVALID = the world
    vec3    local_a, local_b;    // anchors in each body's frame
    vec3    local_axis_a, local_axis_b;   // hinge / slider / twist axis in each frame
    vec3    local_ref_a, local_ref_b;     // perpendicular to it: where angle 0 points
    quat    relative_rotation;   // fixed / slider: b's rotation in a's frame when joined

    // limits: hinge angle and ball twist (radians), slider travel, distance length
    bool  enable_limit;
    float lower, upper;
    float swing_limit;           // ball: the cone the twist axis may swing in, radians (0 = free)
    bool  enable_motor;
    float motor_speed, max_motor_force;   // hinge: rad/s and N*m, slider: m/s and N
    bool  enable_spring;         // distance: spring at `length` (hertz 0 = rope held by the limits)
    float length;
    float hertz, damping_ratio;  // springs, soft fixed joints, mouse
    vec3  target;                // mouse
    float max_force;             // mouse
    float break_force;           // 0 = unbreakable

    // ---- written by the world ----
    float reaction;
    idx   next_a, next_b;
    bool  idle;
    vec3  ra, rb, d, axis, perp1, perp2;
    vec3  linear_impulse, angular_impulse;
    float axial_impulse, motor_impulse, lower_impulse, upper_impulse, swing_impulse;
};

struct phys3d__cell_item { int32_t cx, cy, cz; idx body; };
struct phys3d__grid {
    uint32_t*          start;
    phys3d__cell_item* items;
    size_t             count, capacity;
};

struct phys3d_world {
    // ---- settings ----
    vec3  gravity;               // (0, -9.81, 0)
    float fixed_dt;              // 1/60
    int   max_steps;             // 4
    int   substeps;              // 4
    int   iterations;            // per substep, 1
    int   relax_iterations;      // 1
    float cell_size;             // broadphase cell, 2 m
    float slop;                  // 0.005 m of allowed overlap
    float speculative;           // 0.02 m contact margin
    float bounce_threshold;      // 1 m/s
    float correction;            // 0.2
    float joint_hertz, joint_damping;   // 60, 2
    bool  allow_sleep;
    float sleep_linear;          // 0.05 m/s
    float sleep_angular;         // 0.05 rad/s
    float time_to_sleep;         // 0.5 s

    // ---- state ----
    float accumulator, alpha;
    phys3d_body* bodies;
    size_t       body_count;
    idx*         free_ids;
    size_t       free_count;
    idx*         pending_free;
    size_t       pending_count;

    phys3d_joint* joints;
    size_t        joint_count;
    idx*          free_joints;
    size_t        free_joint_count;
    phys3d_body   ground;

    uint32_t* island;
    float*    island_sleep;
    uint8_t*  island_awake;

    phys3d__grid static_grid, moving_grid;
    idx*         oversize;        // bodies too big for a grid
    size_t       oversize_count;
    bool         statics_dirty;

    phys3d_contact* contacts;
    size_t          contact_count, contact_capacity;
    phys3d_contact* prev_contacts;
    size_t          prev_count, prev_capacity;
    uint32_t*       prev_table;   // pair -> first prev contact of that pair
    size_t          table_size;

    phys3d_event* events;
    size_t        event_count, event_capacity;
};

// ---- meshes ----
PIXDEF bool phys3d_create_mesh(phys3d_mesh& m, const vec3* vertices, size_t vertex_count,
                               const uint32_t* indices, size_t index_count);
// a terrain grid: heights[row * columns + column], `spacing` metres apart, the
// first sample at (0, h, 0), columns along +x and rows along +z
PIXDEF bool phys3d_create_heightfield(phys3d_mesh& m, const float* heights, int columns, int rows, float spacing);
PIXDEF void phys3d_destroy_mesh(phys3d_mesh& m);

// ---- world ----
PIXDEF void phys3d_create_world(phys3d_world& w);
PIXDEF void phys3d_destroy_world(phys3d_world& w);
PIXDEF int  phys3d_update(phys3d_world& w, float frame_dt);
PIXDEF void phys3d_step(phys3d_world& w, float dt);

// ---- bodies ----
PIXDEF phys3d_body phys3d_make_body(uint8_t type, phys3d_shape shape, vec3 position);
PIXDEF idx  phys3d_add_body(phys3d_world& w, const phys3d_body& body);
PIXDEF idx  phys3d_add(phys3d_world& w, uint8_t type, phys3d_shape shape, vec3 position);
PIXDEF phys3d_body* phys3d_get_body(phys3d_world& w, idx id);
PIXDEF void phys3d_remove_body(phys3d_world& w, idx id);
PIXDEF void phys3d_update_mass(phys3d_world& w, idx id);
PIXDEF void phys3d_teleport(phys3d_world& w, idx id, vec3 position, quat rotation);
PIXDEF void phys3d_wake(phys3d_world& w, idx id);

PIXDEF void phys3d_apply_force(phys3d_body& b, vec3 force, vec3 world_point);
PIXDEF void phys3d_apply_impulse(phys3d_body& b, vec3 impulse, vec3 world_point);
PIXDEF void phys3d_apply_torque(phys3d_body& b, vec3 torque);
PIXDEF void phys3d_apply_angular_impulse(phys3d_body& b, vec3 impulse);
PIXDEF void phys3d_apply_force_centre(phys3d_body& b, vec3 force);
PIXDEF void phys3d_apply_impulse_centre(phys3d_body& b, vec3 impulse);

static inline vec3 phys3d_lerp_position(const phys3d_world& w, const phys3d_body& b) {
    return v3lerp(b.prev_position, b.position, w.alpha);
}
static inline quat phys3d_lerp_rotation(const phys3d_world& w, const phys3d_body& b) {
    return quat_slerp(b.prev_rotation, b.rotation, w.alpha);
}
// the body's transform for drawing, interpolated
static inline mat4 phys3d_body_transform(const phys3d_world& w, const phys3d_body& b) {
    return mat4_from_trs(phys3d_lerp_position(w, b), phys3d_lerp_rotation(w, b), v3(1, 1, 1));
}

// ---- joints ----
// Anchors and axes in world space, at the bodies' current poses; a or b may be
// PIX_INVALID for the world. `twist_axis` is where a ball joint's cone points.
PIXDEF idx phys3d_add_ball(phys3d_world& w, idx a, idx b, vec3 anchor, vec3 twist_axis = v3(0, 1, 0));
PIXDEF idx phys3d_add_hinge(phys3d_world& w, idx a, idx b, vec3 anchor, vec3 axis);
PIXDEF idx phys3d_add_slider(phys3d_world& w, idx a, idx b, vec3 anchor, vec3 axis);
PIXDEF idx phys3d_add_fixed(phys3d_world& w, idx a, idx b, vec3 anchor);
PIXDEF idx phys3d_add_distance(phys3d_world& w, idx a, idx b, vec3 anchor_a, vec3 anchor_b);
PIXDEF idx phys3d_add_mouse(phys3d_world& w, idx body, vec3 grab, float max_force = 0.0f);
PIXDEF idx phys3d_add_joint(phys3d_world& w, const phys3d_joint& joint);
PIXDEF phys3d_joint* phys3d_get_joint(phys3d_world& w, idx id);
PIXDEF void phys3d_remove_joint(phys3d_world& w, idx id);
PIXDEF void phys3d_joint_anchors(const phys3d_world& w, const phys3d_joint& j, vec3* a, vec3* b);
// hinge: the current angle, radians, 0 where it was joined
PIXDEF float phys3d_hinge_angle(const phys3d_world& w, const phys3d_joint& j);

// ---- queries ----
PIXDEF bool phys3d_raycast(phys3d_world& w, vec3 origin, vec3 direction, float max_distance,
                           phys3d_ray_hit* hit, uint16_t mask = PHYS_LAYER_ALL, idx ignore = PIX_INVALID);
PIXDEF bool phys3d_shape_cast(phys3d_world& w, phys3d_shape shape, vec3 position, quat rotation,
                              vec3 direction, float max_distance, phys3d_ray_hit* hit,
                              uint16_t mask = PHYS_LAYER_ALL, idx ignore = PIX_INVALID);
PIXDEF size_t phys3d_overlap(phys3d_world& w, phys3d_shape shape, vec3 position, quat rotation,
                             idx* out, size_t capacity, uint16_t mask = PHYS_LAYER_ALL);
static inline size_t phys3d_overlap_sphere(phys3d_world& w, vec3 centre, float radius, idx* out, size_t capacity,
                                           uint16_t mask = PHYS_LAYER_ALL) {
    return phys3d_overlap(w, phys3d_sphere(radius), centre, quat_identity(), out, capacity, mask);
}

// debug draw: phys3d_debug_draw in physics_debug.hpp (it needs the renderer)

#endif // PIX_PHYSICS3D_HPP


#if defined(PIX_IMPLEMENTATION) && !defined(PIX_PHYSICS3D_IMPL)
#define PIX_PHYSICS3D_IMPL

// ---- small helpers ----

static inline float phys3d__m(const mat3& m, int row, int col) { return m.data[col * 3 + row]; }

// R * diag(d) * R^T: a body's inverse inertia turned into world space
static mat3 phys3d__rotated_inertia(quat q, vec3 d) {
    mat3 r = mat3_from_quat(q), out;
    float dv[3] = { d.x, d.y, d.z };
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += phys3d__m(r, i, k) * dv[k] * phys3d__m(r, j, k);
            out.data[j * 3 + i] = s;
        }
    return out;
}

static inline mat3 phys3d__add(const mat3& a, const mat3& b) {
    mat3 r; for (int i = 0; i < 9; i++) r.data[i] = a.data[i] + b.data[i]; return r;
}

// a stable pair of unit vectors perpendicular to n
static inline void phys3d__basis(vec3 n, vec3* t1, vec3* t2) {
    vec3 t = fabsf(n.x) < 0.57735f ? v3(1, 0, 0) : (fabsf(n.y) < 0.57735f ? v3(0, 1, 0) : v3(0, 0, 1));
    *t1 = v3norm(v3cross(n, t));
    *t2 = v3cross(n, *t1);
}

static inline vec3 phys3d__cross(vec3 a, vec3 b) { return v3cross(a, b); }

// ---- meshes: a bounding volume hierarchy over triangles ----

static aabb phys3d__tri_bounds(const phys3d_mesh& m, uint32_t t) {
    vec3 a = m.vertices[m.indices[t * 3]], b = m.vertices[m.indices[t * 3 + 1]], c = m.vertices[m.indices[t * 3 + 2]];
    return aabb_make(v3min(a, v3min(b, c)), v3max(a, v3max(b, c)));
}

// Top-down build: split each node's triangles at the mean centroid along its
// longest axis, falling back to halves when that leaves one side empty. An
// explicit stack instead of recursion; each inner node's two children sit
// next to each other, so a node only needs the left child's index.
static void phys3d__bvh_build_all(phys3d_mesh& m, vec3* centroids) {
    struct task { uint32_t node, first, count; };
    task* stack = pix_alloc<task>(m.triangle_count * 2 + 64);
    size_t top = 0;
    m.node_count = 1;
    stack[top++] = { 0, 0, (uint32_t)m.triangle_count };
    while (top) {
        task t = stack[--top];
        phys3d__bvh_node& node = m.nodes[t.node];
        aabb b = phys3d__tri_bounds(m, m.order[t.first]);
        for (uint32_t i = 1; i < t.count; i++) {
            aabb tb = phys3d__tri_bounds(m, m.order[t.first + i]);
            b.min = v3min(b.min, tb.min); b.max = v3max(b.max, tb.max);
        }
        node.bounds = b;
        if (t.count <= 4) { node.first = t.first; node.count = t.count; continue; }
        vec3 size = b.max - b.min;
        int axis = size.x > size.y ? (size.x > size.z ? 0 : 2) : (size.y > size.z ? 1 : 2);
        float mean = 0.0f;
        for (uint32_t i = 0; i < t.count; i++) mean += (&centroids[m.order[t.first + i]].x)[axis];
        mean /= (float)t.count;
        uint32_t mid = t.first;
        for (uint32_t i = t.first; i < t.first + t.count; i++)
            if ((&centroids[m.order[i]].x)[axis] < mean) { uint32_t s = m.order[i]; m.order[i] = m.order[mid]; m.order[mid] = s; mid++; }
        if (mid == t.first || mid == t.first + t.count) mid = t.first + t.count / 2;
        uint32_t left = (uint32_t)m.node_count;
        m.node_count += 2;
        node.first = left;
        node.count = 0;
        stack[top++] = { left, t.first, mid - t.first };
        stack[top++] = { left + 1, mid, t.first + t.count - mid };
    }
    pix_free(stack);
}

bool phys3d_create_mesh(phys3d_mesh& m, const vec3* vertices, size_t vertex_count,
                        const uint32_t* indices, size_t index_count) {
    memset(&m, 0, sizeof(m));
    if (!vertices || !indices || index_count < 3 || index_count % 3) {
        pix_log("phys3d: a mesh needs vertices and a multiple of 3 indices\n");
        return false;
    }
    for (size_t i = 0; i < index_count; i++)
        if (indices[i] >= vertex_count) { pix_log("phys3d: mesh index %u out of range\n", indices[i]); return false; }
    m.vertex_count = vertex_count;
    m.triangle_count = index_count / 3;
    m.vertices = pix_alloc<vec3>(vertex_count);
    m.indices = pix_alloc<uint32_t>(index_count);
    memcpy(m.vertices, vertices, vertex_count * sizeof(vec3));
    memcpy(m.indices, indices, index_count * sizeof(uint32_t));
    m.order = pix_alloc<uint32_t>(m.triangle_count);
    m.nodes = pix_alloc<phys3d__bvh_node>(m.triangle_count * 2 + 1);
    vec3* centroids = pix_alloc<vec3>(m.triangle_count);
    for (uint32_t t = 0; t < m.triangle_count; t++) {
        m.order[t] = t;
        centroids[t] = (m.vertices[m.indices[t * 3]] + m.vertices[m.indices[t * 3 + 1]] + m.vertices[m.indices[t * 3 + 2]]) * (1.0f / 3.0f);
    }
    phys3d__bvh_build_all(m, centroids);
    pix_free(centroids);
    m.bounds = m.nodes[0].bounds;
    return true;
}

bool phys3d_create_heightfield(phys3d_mesh& m, const float* heights, int columns, int rows, float spacing) {
    memset(&m, 0, sizeof(m));
    if (!heights || columns < 2 || rows < 2) { pix_log("phys3d: a heightfield needs at least 2 x 2 samples\n"); return false; }
    size_t vc = (size_t)columns * rows, tc = (size_t)(columns - 1) * (rows - 1) * 2;
    vec3* v = pix_alloc<vec3>(vc);
    uint32_t* ix = pix_alloc<uint32_t>(tc * 3);
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < columns; c++) v[r * columns + c] = v3(c * spacing, heights[r * columns + c], r * spacing);
    size_t k = 0;
    for (int r = 0; r + 1 < rows; r++)
        for (int c = 0; c + 1 < columns; c++) {
            uint32_t i0 = r * columns + c, i1 = i0 + 1, i2 = i0 + columns, i3 = i2 + 1;
            // counter-clockwise seen from above, so the front faces up
            ix[k++] = i0; ix[k++] = i2; ix[k++] = i1;
            ix[k++] = i1; ix[k++] = i2; ix[k++] = i3;
        }
    bool ok = phys3d_create_mesh(m, v, vc, ix, tc * 3);
    pix_free(v);
    pix_free(ix);
    return ok;
}

void phys3d_destroy_mesh(phys3d_mesh& m) {
    pix_free(m.vertices);
    pix_free(m.indices);
    pix_free(m.nodes);
    pix_free(m.order);
    memset(&m, 0, sizeof(m));
}

// every triangle whose bounds overlap `box` (in the mesh's own frame)
template <typename F>
static void phys3d__mesh_query(const phys3d_mesh& m, const aabb& box, F&& visit) {
    if (!m.node_count) return;
    uint32_t stack[64];
    int top = 0;
    stack[top++] = 0;
    while (top) {
        const phys3d__bvh_node& n = m.nodes[stack[--top]];
        if (!aabb_overlap(n.bounds, box)) continue;
        if (n.count) {
            for (uint32_t i = 0; i < n.count; i++) {
                uint32_t t = m.order[n.first + i];
                if (aabb_overlap(phys3d__tri_bounds(m, t), box)) visit(t);
            }
        } else if (top < 62) {
            stack[top++] = n.first;
            stack[top++] = n.first + 1;
        }
    }
}

// ---- cores: every shape as a convex core plus a radius ----

#define PHYS3D__POINT    0
#define PHYS3D__SEGMENT  1
#define PHYS3D__BOX      2
#define PHYS3D__TRIANGLE 3
#define PHYS3D__PLANE    4

struct phys3d__core {
    uint8_t kind;
    int     count;        // vertices in v
    vec3    v[8];
    vec3    centre;
    vec3    axis[3];      // box: its axes; triangle: axis[0] is the face normal
    vec3    half;
    float   radius;
    float   offset;       // plane: dot(normal, p) == offset; normal in axis[0]
};

static void phys3d__make_core(const phys3d_shape& s, vec3 p, quat q, phys3d__core& c) {
    c.radius = 0.0f;
    c.centre = p;
    if (s.type == PHYS3D_SPHERE || (s.type == PHYS3D_CAPSULE && s.half_length <= 0.0f)) {
        c.kind = PHYS3D__POINT; c.count = 1; c.v[0] = p; c.radius = s.radius;
    } else if (s.type == PHYS3D_CAPSULE) {
        vec3 up = quat_rotate(q, v3(0, s.half_length, 0));
        c.kind = PHYS3D__SEGMENT; c.count = 2; c.radius = s.radius;
        c.v[0] = p - up; c.v[1] = p + up;
    } else if (s.type == PHYS3D_BOX) {
        c.kind = PHYS3D__BOX; c.count = 8; c.half = s.half;
        c.axis[0] = quat_rotate(q, v3(1, 0, 0));
        c.axis[1] = quat_rotate(q, v3(0, 1, 0));
        c.axis[2] = quat_rotate(q, v3(0, 0, 1));
        for (int i = 0; i < 8; i++)
            c.v[i] = p + c.axis[0] * ((i & 1) ? s.half.x : -s.half.x) + c.axis[1] * ((i & 2) ? s.half.y : -s.half.y)
                       + c.axis[2] * ((i & 4) ? s.half.z : -s.half.z);
    } else if (s.type == PHYS3D_PLANE) {
        c.kind = PHYS3D__PLANE; c.count = 0;
        c.axis[0] = quat_rotate(q, s.normal);
        c.offset = v3dot(c.axis[0], p);
    }
}

static void phys3d__triangle_core(vec3 a, vec3 b, vec3 c, phys3d__core& out) {
    out.kind = PHYS3D__TRIANGLE; out.count = 3; out.radius = 0.0f;
    out.v[0] = a; out.v[1] = b; out.v[2] = c;
    out.centre = (a + b + c) * (1.0f / 3.0f);
    out.axis[0] = triangle_normal(a, b, c);
}

static aabb phys3d__core_bounds(const phys3d__core& c) {
    if (c.kind == PHYS3D__PLANE) return aabb_make(v3(-1e30f, -1e30f, -1e30f), v3(1e30f, 1e30f, 1e30f));
    aabb b = aabb_make(c.v[0], c.v[0]);
    for (int i = 1; i < c.count; i++) { b.min = v3min(b.min, c.v[i]); b.max = v3max(b.max, c.v[i]); }
    vec3 r = v3(c.radius, c.radius, c.radius);
    b.min = b.min - r; b.max = b.max + r;
    return b;
}

static vec3 phys3d__support(const phys3d__core& c, vec3 d) {
    if (c.kind == PHYS3D__BOX)
        return c.centre + c.axis[0] * (v3dot(d, c.axis[0]) >= 0.0f ? c.half.x : -c.half.x)
                        + c.axis[1] * (v3dot(d, c.axis[1]) >= 0.0f ? c.half.y : -c.half.y)
                        + c.axis[2] * (v3dot(d, c.axis[2]) >= 0.0f ? c.half.z : -c.half.z);
    int best = 0;
    float bd = v3dot(c.v[0], d);
    for (int i = 1; i < c.count; i++) { float x = v3dot(c.v[i], d); if (x > bd) { bd = x; best = i; } }
    return c.v[best];
}

// ---- GJK: the distance between two cores ----
//
// The Minkowski difference A - B is convex; its point closest to the origin
// gives the closest pair of points. GJK walks a simplex (up to a
// tetrahedron) toward the origin, using only the shapes' support functions.
// Returns 0 when the cores overlap.

struct phys3d__sv { vec3 w, a, b; };

// closest point of triangle abc to the origin (Ericson, Real-Time Collision
// Detection 5.1.5), with barycentric weights
static vec3 phys3d__closest_tri(vec3 a, vec3 b, vec3 c, float* la, float* lb, float* lc) {
    vec3 ab = b - a, ac = c - a, ap = -a;
    float d1 = v3dot(ab, ap), d2 = v3dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) { *la = 1; *lb = 0; *lc = 0; return a; }
    vec3 bp = -b;
    float d3 = v3dot(ab, bp), d4 = v3dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) { *la = 0; *lb = 1; *lc = 0; return b; }
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        *la = 1 - v; *lb = v; *lc = 0;
        return a + ab * v;
    }
    vec3 cp = -c;
    float d5 = v3dot(ab, cp), d6 = v3dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) { *la = 0; *lb = 0; *lc = 1; return c; }
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        *la = 1 - w; *lb = 0; *lc = w;
        return a + ac * w;
    }
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        *la = 0; *lb = 1 - w; *lc = w;
        return b + (c - b) * w;
    }
    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom, w = vc * denom;
    *la = 1 - v - w; *lb = v; *lc = w;
    return a + ab * v + ac * w;
}

// Reduces the simplex to the smallest part holding its closest point to the
// origin; returns that point. Sets *inside when the origin is in a tetrahedron.
static vec3 phys3d__simplex_closest(phys3d__sv* s, int* n, float* lambda, bool* inside) {
    *inside = false;
    if (*n == 1) { lambda[0] = 1; return s[0].w; }
    if (*n == 2) {
        vec3 e = s[1].w - s[0].w;
        float t = -v3dot(s[0].w, e), l2 = v3dot(e, e);
        if (t <= 0.0f || l2 <= 1e-20f) { *n = 1; lambda[0] = 1; return s[0].w; }
        if (t >= l2) { s[0] = s[1]; *n = 1; lambda[0] = 1; return s[0].w; }
        t /= l2;
        lambda[0] = 1 - t; lambda[1] = t;
        return s[0].w + e * t;
    }
    if (*n == 3) {
        float l[3];
        vec3 p = phys3d__closest_tri(s[0].w, s[1].w, s[2].w, &l[0], &l[1], &l[2]);
        int k = 0;
        phys3d__sv keep[3];
        for (int i = 0; i < 3; i++) if (l[i] > 0.0f) { keep[k] = s[i]; lambda[k] = l[i]; k++; }
        for (int i = 0; i < k; i++) s[i] = keep[i];
        *n = k;
        return p;
    }
    // tetrahedron: the origin is inside unless it is beyond one of the faces
    static const int faces[4][4] = { { 0, 1, 2, 3 }, { 0, 1, 3, 2 }, { 0, 2, 3, 1 }, { 1, 2, 3, 0 } };
    float best = 1e30f;
    vec3 bp = v3(0, 0, 0);
    int bf = -1;
    float bl[3] = { 0, 0, 0 };
    for (int f = 0; f < 4; f++) {
        vec3 a = s[faces[f][0]].w, b = s[faces[f][1]].w, c = s[faces[f][2]].w, d = s[faces[f][3]].w;
        vec3 nrm = v3cross(b - a, c - a);
        float side_o = -v3dot(nrm, a), side_d = v3dot(nrm, d - a);
        if (side_o * side_d > 0.0f && fabsf(side_d) > 1e-12f) continue;   // origin on the inner side of this face
        float l[3];
        vec3 p = phys3d__closest_tri(a, b, c, &l[0], &l[1], &l[2]);
        float d2 = v3dot(p, p);
        if (d2 < best) { best = d2; bp = p; bf = f; bl[0] = l[0]; bl[1] = l[1]; bl[2] = l[2]; }
    }
    if (bf < 0) { *inside = true; return v3(0, 0, 0); }
    phys3d__sv keep[3];
    int k = 0;
    for (int i = 0; i < 3; i++) if (bl[i] > 0.0f) { keep[k] = s[faces[bf][i]]; lambda[k] = bl[i]; k++; }
    for (int i = 0; i < k; i++) s[i] = keep[i];
    *n = k;
    return bp;
}

static float phys3d__gjk(const phys3d__core& A, const phys3d__core& B, vec3* pa, vec3* pb) {
    phys3d__sv s[4];
    float lambda[4] = { 1, 0, 0, 0 };
    int n = 1;
    vec3 d = A.centre - B.centre;
    if (v3dot(d, d) < 1e-12f) d = v3(1, 0, 0);
    s[0].a = phys3d__support(A, -d); s[0].b = phys3d__support(B, d); s[0].w = s[0].a - s[0].b;
    vec3 v = s[0].w;
    for (int iter = 0; iter < 32; iter++) {
        bool inside;
        v = phys3d__simplex_closest(s, &n, lambda, &inside);
        float vv = v3dot(v, v);
        if (inside || vv < 1e-14f) return 0.0f;
        phys3d__sv nw;
        nw.a = phys3d__support(A, -v);
        nw.b = phys3d__support(B, v);
        nw.w = nw.a - nw.b;
        if (vv - v3dot(v, nw.w) <= 1e-6f * vv + 1e-12f) break;   // no closer point exists
        bool dup = false;
        for (int i = 0; i < n; i++) dup |= v3dot(s[i].w - nw.w, s[i].w - nw.w) < 1e-14f;
        if (dup) break;
        s[n++] = nw;
    }
    vec3 a = v3(0, 0, 0), b = v3(0, 0, 0);
    for (int i = 0; i < n; i++) { a += s[i].a * lambda[i]; b += s[i].b * lambda[i]; }
    *pa = a; *pb = b;
    return v3len(v);
}

// ---- manifold building ----

static void phys3d__add_point(phys3d_contact& m, vec3 p, float sep) {
    if (m.count >= 4) return;
    phys3d_contact_point& cp = m.points[m.count++];
    memset(&cp, 0, sizeof(cp));
    cp.point = p;
    cp.separation = sep;
}

// keeps the four points that cover the largest area: the deepest, the one
// farthest from it, the one farthest off that line, and the one farthest from
// those three
static int phys3d__reduce(const vec3* pts, const float* seps, int n, int* keep) {
    if (n <= 4) { for (int i = 0; i < n; i++) keep[i] = i; return n; }
    int a = 0;
    for (int i = 1; i < n; i++) if (seps[i] < seps[a]) a = i;
    int b = a; float best = -1;
    for (int i = 0; i < n; i++) { float d = v3dot(pts[i] - pts[a], pts[i] - pts[a]); if (d > best) { best = d; b = i; } }
    int c = a; best = -1;
    for (int i = 0; i < n; i++) {
        vec3 x = v3cross(pts[b] - pts[a], pts[i] - pts[a]);
        float d = v3dot(x, x);
        if (d > best) { best = d; c = i; }
    }
    int d = a; best = -1;
    for (int i = 0; i < n; i++) {
        if (i == a || i == b || i == c) continue;
        float m = minf(v3dot(pts[i] - pts[a], pts[i] - pts[a]), minf(v3dot(pts[i] - pts[b], pts[i] - pts[b]), v3dot(pts[i] - pts[c], pts[i] - pts[c])));
        if (m > best) { best = m; d = i; }
    }
    keep[0] = a; keep[1] = b; keep[2] = c; keep[3] = d;
    return 4;
}

// A face of a box (6) or a triangle (1): its outward normal, a point on it and
// its corners in order.
static int phys3d__face(const phys3d__core& c, int f, vec3* n, vec3* poly) {
    if (c.kind == PHYS3D__TRIANGLE) {
        *n = c.axis[0];
        poly[0] = c.v[0]; poly[1] = c.v[1]; poly[2] = c.v[2];
        return 3;
    }
    int i = f >> 1, j = (i + 1) % 3, k = (i + 2) % 3;
    float s = (f & 1) ? -1.0f : 1.0f;
    float h[3] = { c.half.x, c.half.y, c.half.z };
    *n = c.axis[i] * s;
    vec3 fc = c.centre + c.axis[i] * (s * h[i]);
    vec3 u = c.axis[j] * h[j], v = c.axis[k] * h[k];
    poly[0] = fc + u + v; poly[1] = fc - u + v; poly[2] = fc - u - v; poly[3] = fc + u - v;
    return 4;
}
static inline int phys3d__face_count(const phys3d__core& c) { return c.kind == PHYS3D__TRIANGLE ? 1 : 6; }

// Clips polygon `in` (n points) to the reference face's side planes, then
// keeps the points within `margin` of the face. `ref_n` points out of the
// reference shape; `flip` means the reference is b, so the a->b normal is -ref_n.
static int phys3d__clip_to_face(const vec3* ref_poly, int ref_n_count, vec3 ref_n, const vec3* in, int n,
                                float r_ref, float r_inc, bool flip, float margin, phys3d_contact& m) {
    vec3 buf_a[16], buf_b[16];
    for (int i = 0; i < n; i++) buf_a[i] = in[i];
    vec3* cur = buf_a; vec3* next = buf_b;
    vec3 centre = v3(0, 0, 0);
    for (int i = 0; i < ref_n_count; i++) centre += ref_poly[i];
    centre = centre * (1.0f / ref_n_count);
    for (int e = 0; e < ref_n_count && n > 0; e++) {
        vec3 p0 = ref_poly[e], p1 = ref_poly[(e + 1) % ref_n_count];
        vec3 side = v3norm(v3cross(p1 - p0, ref_n));
        if (v3dot(side, centre - p0) > 0.0f) side = -side;    // outward, whatever the winding
        float off = v3dot(side, p0);
        int k = 0;
        if (n <= 2) {
            // a point or a segment: clip it as such (a polygon pass would duplicate points)
            vec3 a = cur[0], b = cur[n - 1];
            float da = v3dot(side, a) - off, db = v3dot(side, b) - off;
            if (da > 0.0f && db > 0.0f) { n = 0; break; }
            if (da > 0.0f)      a = a + (b - a) * (da / (da - db));
            else if (db > 0.0f) b = b + (a - b) * (db / (db - da));
            next[k++] = a;
            if (n == 2) next[k++] = b;
        } else {
            for (int i = 0; i < n; i++) {
                vec3 a = cur[i], b = cur[(i + 1) % n];
                float da = v3dot(side, a) - off, db = v3dot(side, b) - off;
                if (da <= 0.0f) next[k++] = a;
                if ((da < 0.0f) != (db < 0.0f) && k < 16) next[k++] = a + (b - a) * (da / (da - db));
                if (k >= 16) break;
            }
        }
        vec3* t = cur; cur = next; next = t;
        n = k;
    }
    vec3 pts[16]; float seps[16];
    int count = 0;
    float plane = v3dot(ref_n, ref_poly[0]);
    for (int i = 0; i < n; i++) {
        float sep = v3dot(ref_n, cur[i]) - plane - r_ref - r_inc;
        if (sep > margin) continue;
        pts[count] = cur[i] - ref_n * (r_inc + 0.5f * sep);
        seps[count++] = sep;
    }
    if (!count) return 0;
    int keep[4];
    int k = phys3d__reduce(pts, seps, count, keep);
    m.normal = flip ? -ref_n : ref_n;
    m.count = 0;
    for (int i = 0; i < k; i++) phys3d__add_point(m, pts[keep[i]], seps[keep[i]]);
    return m.count;
}

// a point or segment (with radius) against a point or segment
static int phys3d__collide_rounded(const phys3d__core& a, const phys3d__core& b, float margin, phys3d_contact& m) {
    vec3 a0 = a.v[0], a1 = a.v[a.count - 1], b0 = b.v[0], b1 = b.v[b.count - 1];
    vec3 ca, cb;
    closest_points_segments(a0, a1, b0, b1, &ca, &cb);
    float r = a.radius + b.radius, d = v3dist(ca, cb);
    if (d - r > margin) return 0;
    vec3 n = d > 1e-6f ? (cb - ca) / d : v3(0, 1, 0);
    m.normal = n;
    m.count = 0;
    // two capsules lying side by side: a line contact, so two points
    if (a.count == 2 && b.count == 2) {
        vec3 ua = a1 - a0, ub = b1 - b0;
        float la = v3len(ua), lb = v3len(ub);
        if (la > 1e-6f && lb > 1e-6f) {
            ua = ua / la; ub = ub / lb;
            if (fabsf(v3dot(ua, ub)) > 0.995f) {
                float t0 = v3dot(b0 - a0, ua), t1 = v3dot(b1 - a0, ua);
                float lo = maxf(0.0f, minf(t0, t1)), hi = minf(la, maxf(t0, t1));
                if (hi - lo > 1e-4f) {
                    float ts[2] = { lo, hi };
                    for (int k = 0; k < 2; k++) {
                        vec3 pa = a0 + ua * ts[k];
                        vec3 pb = closest_point_segment(b0, b1, pa);
                        float sep = v3dot(pb - pa, n) - r;
                        if (sep <= margin) phys3d__add_point(m, pa + n * (a.radius + 0.5f * sep), sep);
                    }
                    if (m.count) return m.count;
                }
            }
        }
    }
    phys3d__add_point(m, (ca + n * a.radius + cb - n * b.radius) * 0.5f, d - r);
    return 1;
}

// a point or segment (with radius) against a box or triangle
static int phys3d__collide_rounded_poly(const phys3d__core& a, const phys3d__core& b, float margin, phys3d_contact& m) {
    vec3 pa, pb;
    float d = phys3d__gjk(a, b, &pa, &pb);
    vec3 poly[4], fn;
    m.count = 0;
    if (d > 1e-5f) {
        if (d - a.radius > margin) return 0;
        vec3 n = (pb - pa) / d;
        if (a.count == 2) {
            // a capsule lying on a face: clip its segment to that face for two points
            int best = -1; float align = 0.995f;
            for (int f = 0; f < phys3d__face_count(b); f++) {
                phys3d__face(b, f, &fn, poly);
                float x = -v3dot(fn, n);
                if (x > align) { align = x; best = f; }
            }
            if (best >= 0) {
                int pc = phys3d__face(b, best, &fn, poly);
                if (phys3d__clip_to_face(poly, pc, fn, a.v, 2, 0.0f, a.radius, true, margin, m) && m.count) return m.count;
            }
        }
        m.normal = n;
        phys3d__add_point(m, (pa + n * a.radius + pb) * 0.5f, d - a.radius);
        return 1;
    }
    // The core is inside: leave through the face it is least deep behind. A
    // triangle only has its front.
    int best = 0;
    float best_sep = -1e30f;
    for (int f = 0; f < phys3d__face_count(b); f++) {
        phys3d__face(b, f, &fn, poly);
        float plane = v3dot(fn, poly[0]);
        float s = 1e30f;
        for (int i = 0; i < a.count; i++) s = minf(s, v3dot(fn, a.v[i]) - plane);
        if (s > best_sep) { best_sep = s; best = f; }
    }
    int pc = phys3d__face(b, best, &fn, poly);
    if (phys3d__clip_to_face(poly, pc, fn, a.v, a.count, 0.0f, a.radius, true, margin, m)) return m.count;
    // clipped away entirely (the core pokes out past the face's edges): use it unclipped
    float plane = v3dot(fn, poly[0]);
    m.normal = -fn;
    for (int i = 0; i < a.count; i++) {
        float sep = v3dot(fn, a.v[i]) - plane - a.radius;
        phys3d__add_point(m, a.v[i] - fn * (a.radius + 0.5f * sep), sep);
    }
    return m.count;
}

// the edge directions of a box (3) or triangle (3)
static int phys3d__edges(const phys3d__core& c, vec3* dirs) {
    if (c.kind == PHYS3D__TRIANGLE) {
        dirs[0] = c.v[1] - c.v[0]; dirs[1] = c.v[2] - c.v[1]; dirs[2] = c.v[0] - c.v[2];
    } else {
        dirs[0] = c.axis[0]; dirs[1] = c.axis[1]; dirs[2] = c.axis[2];
    }
    return 3;
}

// the edge of c running along direction index e that reaches farthest along `toward`
static void phys3d__support_edge(const phys3d__core& c, int e, vec3 toward, vec3* p0, vec3* p1) {
    if (c.kind == PHYS3D__TRIANGLE) { *p0 = c.v[e]; *p1 = c.v[(e + 1) % 3]; return; }
    int j = (e + 1) % 3, k = (e + 2) % 3;
    float h[3] = { c.half.x, c.half.y, c.half.z };
    vec3 base = c.centre + c.axis[j] * (v3dot(c.axis[j], toward) >= 0.0f ? h[j] : -h[j])
                         + c.axis[k] * (v3dot(c.axis[k], toward) >= 0.0f ? h[k] : -h[k]);
    *p0 = base - c.axis[e] * h[e];
    *p1 = base + c.axis[e] * h[e];
}

// Box against box or triangle: SAT over both shapes' faces and every pair of
// edges. The axis of least penetration decides: a face axis clips the other
// shape's most opposed face against it (up to four points), an edge axis
// gives one point between the two edges.
static int phys3d__collide_polys(const phys3d__core& a, const phys3d__core& b, float slop, float margin, phys3d_contact& m) {
    m.count = 0;
    vec3 poly[4], fn;
    float sep_a = -1e30f, sep_b = -1e30f;
    int face_a = 0, face_b = 0;
    for (int f = 0; f < phys3d__face_count(a); f++) {
        phys3d__face(a, f, &fn, poly);
        float plane = v3dot(fn, poly[0]), s = 1e30f;
        for (int i = 0; i < b.count; i++) s = minf(s, v3dot(fn, b.v[i]) - plane);
        if (s > sep_a) { sep_a = s; face_a = f; }
    }
    if (sep_a > margin) return 0;
    for (int f = 0; f < phys3d__face_count(b); f++) {
        phys3d__face(b, f, &fn, poly);
        float plane = v3dot(fn, poly[0]), s = 1e30f;
        for (int i = 0; i < a.count; i++) s = minf(s, v3dot(fn, a.v[i]) - plane);
        if (s > sep_b) { sep_b = s; face_b = f; }
    }
    if (sep_b > margin) return 0;
    vec3 ea[3], eb[3];
    phys3d__edges(a, ea); phys3d__edges(b, eb);
    float sep_e = -1e30f;
    int edge_i = 0, edge_j = 0;
    vec3 edge_axis = v3(0, 1, 0);
    vec3 dcent = b.centre - a.centre;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            vec3 ax = v3cross(ea[i], eb[j]);
            float l2 = v3dot(ax, ax);
            if (l2 < 1e-8f * v3dot(ea[i], ea[i]) * v3dot(eb[j], eb[j])) continue;   // parallel edges
            ax = ax / sqrtf(l2);
            if (v3dot(ax, dcent) < 0.0f) ax = -ax;
            float amax = -1e30f, bmin = 1e30f;
            for (int k = 0; k < a.count; k++) amax = maxf(amax, v3dot(ax, a.v[k]));
            for (int k = 0; k < b.count; k++) bmin = minf(bmin, v3dot(ax, b.v[k]));
            float s = bmin - amax;
            if (s > margin) return 0;
            if (s > sep_e) { sep_e = s; edge_i = i; edge_j = j; edge_axis = ax; }
        }
    // faces win near-ties: an edge contact only when clearly shallower, so a
    // box resting on a face never flickers onto an edge normal
    float face_best = maxf(sep_a, sep_b);
    if (sep_e > face_best + 0.5f * slop) {
        vec3 a0, a1, b0, b1, ca, cb;
        phys3d__support_edge(a, edge_i, edge_axis, &a0, &a1);
        phys3d__support_edge(b, edge_j, -edge_axis, &b0, &b1);
        closest_points_segments(a0, a1, b0, b1, &ca, &cb);
        m.normal = edge_axis;
        phys3d__add_point(m, (ca + cb) * 0.5f, sep_e);
        return 1;
    }
    bool flip = sep_b > sep_a + 0.1f * slop;
    const phys3d__core& ref = flip ? b : a;
    const phys3d__core& inc = flip ? a : b;
    int ref_face = flip ? face_b : face_a;
    int rc = phys3d__face(ref, ref_face, &fn, poly);
    // the incident face: the other shape's face most opposed to the reference
    vec3 inc_poly[4];
    int ic = 0;
    float lowest = 2.0f;
    for (int f = 0; f < phys3d__face_count(inc); f++) {
        vec3 tp[4], tn;
        int c = phys3d__face(inc, f, &tn, tp);
        float x = v3dot(tn, fn);
        if (x < lowest) { lowest = x; ic = c; for (int i = 0; i < c; i++) inc_poly[i] = tp[i]; }
    }
    return phys3d__clip_to_face(poly, rc, fn, inc_poly, ic, 0.0f, 0.0f, flip, margin, m);
}

// any core against an infinite plane (b)
static int phys3d__collide_plane(const phys3d__core& a, const phys3d__core& p, float margin, phys3d_contact& m) {
    vec3 n = p.axis[0];
    vec3 pts[8]; float seps[8];
    int count = 0;
    for (int i = 0; i < a.count; i++) {
        float sep = v3dot(n, a.v[i]) - p.offset - a.radius;
        if (sep > margin) continue;
        pts[count] = a.v[i] - n * (a.radius + 0.5f * sep);
        seps[count++] = sep;
    }
    if (!count) return 0;
    int keep[4];
    int k = phys3d__reduce(pts, seps, count, keep);
    m.normal = -n;
    m.count = 0;
    for (int i = 0; i < k; i++) phys3d__add_point(m, pts[keep[i]], seps[keep[i]]);
    return m.count;
}

// The one narrowphase entry: normal from a to b, points within `margin`.
static int phys3d__collide(const phys3d__core& a, const phys3d__core& b, float slop, float margin, phys3d_contact& m) {
    m.count = 0;
    if (a.kind > b.kind) {    // handle each pair once, in kind order
        int n = phys3d__collide(b, a, slop, margin, m);
        m.normal = -m.normal;
        return n;
    }
    if (b.kind == PHYS3D__PLANE) return a.kind == PHYS3D__PLANE ? 0 : phys3d__collide_plane(a, b, margin, m);
    if (b.kind <= PHYS3D__SEGMENT) return phys3d__collide_rounded(a, b, margin, m);
    if (a.kind <= PHYS3D__SEGMENT) return phys3d__collide_rounded_poly(a, b, margin, m);
    return phys3d__collide_polys(a, b, slop, margin, m);
}

// ---- mass ----

static void phys3d__compute_mass(phys3d_body& b) {
    b.inv_mass = 0.0f;
    b.inv_inertia_local = v3(0, 0, 0);
    if (b.type != PHYS3D_DYNAMIC) return;
    float d = b.density > 0.0f ? b.density : 1.0f;
    const phys3d_shape& s = b.shape;
    float mass;
    vec3 inertia;
    if (s.type == PHYS3D_SPHERE) {
        mass = d * (4.0f / 3.0f) * PIX_PI * s.radius * s.radius * s.radius;
        float i = 0.4f * mass * s.radius * s.radius;
        inertia = v3(i, i, i);
    } else if (s.type == PHYS3D_CAPSULE) {
        // a cylinder plus a sphere split into two caps pushed out to the ends
        float r = s.radius, h = 2.0f * s.half_length, rr = r * r;
        float mc = d * PIX_PI * rr * h, ms = d * (4.0f / 3.0f) * PIX_PI * rr * r;
        mass = mc + ms;
        float iy = mc * rr * 0.5f + ms * rr * 0.4f;
        float ix = mc * (h * h / 12.0f + rr * 0.25f) + ms * (rr * 0.4f + h * h * 0.25f + 0.375f * h * r);
        inertia = v3(ix, iy, ix);
    } else {
        vec3 h = s.half;
        mass = d * 8.0f * h.x * h.y * h.z;
        inertia = v3(h.y * h.y + h.z * h.z, h.x * h.x + h.z * h.z, h.x * h.x + h.y * h.y) * (mass / 3.0f);
    }
    if (mass > 0.0f) b.inv_mass = 1.0f / mass;
    if (!b.fixed_rotation && inertia.x > 0.0f && inertia.y > 0.0f && inertia.z > 0.0f)
        b.inv_inertia_local = v3(1.0f / inertia.x, 1.0f / inertia.y, 1.0f / inertia.z);
}

static inline void phys3d__update_inertia(phys3d_body& b) {
    b.inv_inertia = phys3d__rotated_inertia(b.rotation, b.inv_inertia_local);
}

// ---- world ----

void phys3d_create_world(phys3d_world& w) {
    memset(&w, 0, sizeof(w));
    w.gravity = v3(0.0f, -9.81f, 0.0f);
    w.fixed_dt = 1.0f / 60.0f;
    w.max_steps = 4;
    w.substeps = 4;
    w.iterations = 1;
    w.relax_iterations = 1;
    w.cell_size = 2.0f;
    w.slop = 0.005f;
    w.speculative = 0.02f;
    w.bounce_threshold = 1.0f;
    w.correction = 0.2f;
    w.joint_hertz = 60.0f;
    w.joint_damping = 2.0f;
    w.allow_sleep = true;
    w.sleep_linear = 0.05f;
    w.sleep_angular = 0.05f;
    w.time_to_sleep = 0.5f;
    w.ground.type = PHYS3D_STATIC;
    w.ground.rotation = quat_identity();
    w.ground.delta_rot = quat_identity();
    w.ground.joint_list = PIX_INVALID;
    w.bodies       = pix_alloc<phys3d_body>(PHYS3D_MAX_BODIES);
    w.free_ids     = pix_alloc<idx>(PHYS3D_MAX_BODIES);
    w.pending_free = pix_alloc<idx>(PHYS3D_MAX_BODIES);
    w.oversize     = pix_alloc<idx>(PHYS3D_MAX_BODIES);
    w.joints       = pix_alloc<phys3d_joint>(PHYS3D_MAX_JOINTS);
    w.free_joints  = pix_alloc<idx>(PHYS3D_MAX_JOINTS);
    w.island       = pix_alloc<uint32_t>(PHYS3D_MAX_BODIES);
    w.island_sleep = pix_alloc<float>(PHYS3D_MAX_BODIES);
    w.island_awake = pix_alloc<uint8_t>(PHYS3D_MAX_BODIES);
    w.static_grid.start = pix_alloc<uint32_t>(PHYS3D_GRID_BUCKETS + 1);
    w.moving_grid.start = pix_alloc<uint32_t>(PHYS3D_GRID_BUCKETS + 1);
}

void phys3d_destroy_world(phys3d_world& w) {
    pix_free(w.bodies);
    pix_free(w.free_ids);
    pix_free(w.pending_free);
    pix_free(w.oversize);
    pix_free(w.joints);
    pix_free(w.free_joints);
    pix_free(w.island);
    pix_free(w.island_sleep);
    pix_free(w.island_awake);
    pix_free(w.static_grid.start);
    pix_free(w.static_grid.items);
    pix_free(w.moving_grid.start);
    pix_free(w.moving_grid.items);
    pix_free(w.contacts);
    pix_free(w.prev_contacts);
    pix_free(w.prev_table);
    pix_free(w.events);
    memset(&w, 0, sizeof(w));
}

template <typename T>
static void phys3d__reserve(T*& p, size_t& capacity, size_t need) {
    if (need <= capacity) return;
    size_t cap = capacity ? capacity : 64;
    while (cap < need) cap *= 2;
    p = (T*)pix_mem_realloc(p, cap * sizeof(T));
    capacity = cap;
}

phys3d_body phys3d_make_body(uint8_t type, phys3d_shape shape, vec3 position) {
    phys3d_body b;
    memset(&b, 0, sizeof(b));
    b.type = type;
    b.shape = shape;
    b.position = position;
    b.rotation = quat_identity();
    b.density = 1000.0f;
    b.friction = 0.6f;
    b.rolling_friction = 0.01f;
    b.gravity_scale = 1.0f;
    b.layer = PHYS_LAYER_0;
    b.mask = PHYS_LAYER_ALL;
    return b;
}

static inline void phys3d__wake_body(phys3d_body& b) { b.sleeping = false; b.sleep_time = 0.0f; }

idx phys3d_add_body(phys3d_world& w, const phys3d_body& body) {
    idx id;
    if (w.free_count) id = w.free_ids[--w.free_count];
    else if (w.body_count < PHYS3D_MAX_BODIES) id = (idx)w.body_count++;
    else { pix_log("phys3d: body limit (%d) reached\n", PHYS3D_MAX_BODIES); return PIX_INVALID; }
    phys3d_body& b = w.bodies[id];
    b = body;
    if ((b.shape.type == PHYS3D_PLANE || b.shape.type == PHYS3D_MESH) && b.type == PHYS3D_DYNAMIC) {
        pix_log("phys3d: planes and meshes cannot be dynamic; made static\n");
        b.type = PHYS3D_STATIC;
    }
    if (b.shape.type == PHYS3D_MESH && !b.shape.mesh) { pix_log("phys3d: mesh shape without a mesh\n"); w.free_ids[w.free_count++] = id; return PIX_INVALID; }
    b.rotation = quat_norm(b.rotation);
    b.active = true;
    b.sleeping = false;
    b.sleep_time = 0.0f;
    b.joint_list = PIX_INVALID;
    b.prev_position = b.position;
    b.prev_rotation = b.rotation;
    b.delta_rot = quat_identity();
    phys3d__compute_mass(b);
    phys3d__update_inertia(b);
    w.statics_dirty = true;     // the oversize list may change too
    return id;
}

idx phys3d_add(phys3d_world& w, uint8_t type, phys3d_shape shape, vec3 position) {
    return phys3d_add_body(w, phys3d_make_body(type, shape, position));
}

phys3d_body* phys3d_get_body(phys3d_world& w, idx id) {
    return id < w.body_count && w.bodies[id].active ? &w.bodies[id] : 0;
}

void phys3d_remove_body(phys3d_world& w, idx id) {
    phys3d_body* b = phys3d_get_body(w, id);
    if (!b) return;
    while (b->joint_list != PIX_INVALID) phys3d_remove_joint(w, b->joint_list);
    for (size_t i = 0; i < w.contact_count; i++) {
        const phys3d_contact& c = w.contacts[i];
        if (c.a == id) phys3d__wake_body(w.bodies[c.b]);
        if (c.b == id) phys3d__wake_body(w.bodies[c.a]);
    }
    b->active = false;
    w.statics_dirty = true;
    w.pending_free[w.pending_count++] = id;
}

void phys3d_update_mass(phys3d_world& w, idx id) {
    phys3d_body* b = phys3d_get_body(w, id);
    if (!b) return;
    phys3d__compute_mass(*b);
    phys3d__update_inertia(*b);
    w.statics_dirty = true;
}

void phys3d_teleport(phys3d_world& w, idx id, vec3 position, quat rotation) {
    phys3d_body* b = phys3d_get_body(w, id);
    if (!b) return;
    b->position = b->prev_position = position;
    b->rotation = b->prev_rotation = quat_norm(rotation);
    phys3d__update_inertia(*b);
    phys3d__wake_body(*b);
    if (b->type == PHYS3D_STATIC) w.statics_dirty = true;
}

void phys3d_wake(phys3d_world& w, idx id) {
    phys3d_body* b = phys3d_get_body(w, id);
    if (b) phys3d__wake_body(*b);
}

void phys3d_apply_force(phys3d_body& b, vec3 force, vec3 world_point) {
    phys3d__wake_body(b);
    b.force += force;
    b.torque += v3cross(world_point - b.position, force);
}
void phys3d_apply_impulse(phys3d_body& b, vec3 impulse, vec3 world_point) {
    phys3d__wake_body(b);
    b.velocity += impulse * b.inv_mass;
    b.angular_velocity += b.inv_inertia * v3cross(world_point - b.position, impulse);
}
void phys3d_apply_torque(phys3d_body& b, vec3 torque) { phys3d__wake_body(b); b.torque += torque; }
void phys3d_apply_angular_impulse(phys3d_body& b, vec3 impulse) {
    phys3d__wake_body(b);
    b.angular_velocity += b.inv_inertia * impulse;
}
void phys3d_apply_force_centre(phys3d_body& b, vec3 force) { phys3d__wake_body(b); b.force += force; }
void phys3d_apply_impulse_centre(phys3d_body& b, vec3 impulse) { phys3d__wake_body(b); b.velocity += impulse * b.inv_mass; }

// ---- joints ----

static inline phys3d_body& phys3d__jbody(phys3d_world& w, idx id) { return id == PIX_INVALID ? w.ground : w.bodies[id]; }
static inline const phys3d_body& phys3d__jbody(const phys3d_world& w, idx id) { return id == PIX_INVALID ? w.ground : w.bodies[id]; }

static inline bool phys3d__idle(const phys3d_body& b) {
    return b.type == PHYS3D_STATIC || b.sleeping ||
           (b.type == PHYS3D_KINEMATIC && v3dot(b.velocity, b.velocity) == 0.0f && v3dot(b.angular_velocity, b.angular_velocity) == 0.0f);
}

static inline vec3 phys3d__to_local(const phys3d_body& b, vec3 world) { return quat_rotate(quat_conjugate(b.rotation), world - b.position); }
static inline vec3 phys3d__dir_to_local(const phys3d_body& b, vec3 world) { return quat_rotate(quat_conjugate(b.rotation), world); }

idx phys3d_add_joint(phys3d_world& w, const phys3d_joint& joint) {
    if (joint.a == joint.b) { pix_log("phys3d: a joint needs two different bodies\n"); return PIX_INVALID; }
    if ((joint.a != PIX_INVALID && !phys3d_get_body(w, joint.a)) || (joint.b != PIX_INVALID && !phys3d_get_body(w, joint.b))) {
        pix_log("phys3d: joint on a removed body\n");
        return PIX_INVALID;
    }
    idx id;
    if (w.free_joint_count) id = w.free_joints[--w.free_joint_count];
    else if (w.joint_count < PHYS3D_MAX_JOINTS) id = (idx)w.joint_count++;
    else { pix_log("phys3d: joint limit (%d) reached\n", PHYS3D_MAX_JOINTS); return PIX_INVALID; }
    phys3d_joint& j = w.joints[id];
    j = joint;
    j.active = true;
    j.linear_impulse = j.angular_impulse = v3(0, 0, 0);
    j.axial_impulse = j.motor_impulse = j.lower_impulse = j.upper_impulse = j.swing_impulse = 0.0f;
    j.next_a = j.next_b = PIX_INVALID;
    if (j.a != PIX_INVALID) { j.next_a = w.bodies[j.a].joint_list; w.bodies[j.a].joint_list = id; phys3d__wake_body(w.bodies[j.a]); }
    if (j.b != PIX_INVALID) { j.next_b = w.bodies[j.b].joint_list; w.bodies[j.b].joint_list = id; phys3d__wake_body(w.bodies[j.b]); }
    return id;
}

static phys3d_joint phys3d__joint_base(phys3d_world& w, uint8_t type, idx a, idx b, vec3 anchor_a, vec3 anchor_b, vec3 axis) {
    phys3d_joint j;
    memset(&j, 0, sizeof(j));
    j.type = type;
    j.a = a; j.b = b;
    const phys3d_body& A = phys3d__jbody(w, a);
    const phys3d_body& B = phys3d__jbody(w, b);
    j.local_a = phys3d__to_local(A, anchor_a);
    j.local_b = phys3d__to_local(B, anchor_b);
    axis = v3dot(axis, axis) > 1e-12f ? v3norm(axis) : v3(0, 1, 0);
    vec3 ref, unused;
    phys3d__basis(axis, &ref, &unused);
    j.local_axis_a = phys3d__dir_to_local(A, axis);
    j.local_axis_b = phys3d__dir_to_local(B, axis);
    j.local_ref_a = phys3d__dir_to_local(A, ref);
    j.local_ref_b = phys3d__dir_to_local(B, ref);
    j.relative_rotation = quat_mul(quat_conjugate(A.rotation), B.rotation);
    j.damping_ratio = 1.0f;
    return j;
}

idx phys3d_add_ball(phys3d_world& w, idx a, idx b, vec3 anchor, vec3 twist_axis) {
    return phys3d_add_joint(w, phys3d__joint_base(w, PHYS3D_JOINT_BALL, a, b, anchor, anchor, twist_axis));
}
idx phys3d_add_hinge(phys3d_world& w, idx a, idx b, vec3 anchor, vec3 axis) {
    return phys3d_add_joint(w, phys3d__joint_base(w, PHYS3D_JOINT_HINGE, a, b, anchor, anchor, axis));
}
idx phys3d_add_slider(phys3d_world& w, idx a, idx b, vec3 anchor, vec3 axis) {
    return phys3d_add_joint(w, phys3d__joint_base(w, PHYS3D_JOINT_SLIDER, a, b, anchor, anchor, axis));
}
idx phys3d_add_fixed(phys3d_world& w, idx a, idx b, vec3 anchor) {
    return phys3d_add_joint(w, phys3d__joint_base(w, PHYS3D_JOINT_FIXED, a, b, anchor, anchor, v3(0, 1, 0)));
}
idx phys3d_add_distance(phys3d_world& w, idx a, idx b, vec3 anchor_a, vec3 anchor_b) {
    phys3d_joint j = phys3d__joint_base(w, PHYS3D_JOINT_DISTANCE, a, b, anchor_a, anchor_b, v3(0, 1, 0));
    j.length = maxf(v3dist(anchor_a, anchor_b), 1e-3f);
    j.lower = 0.0f;
    j.upper = j.length;
    j.damping_ratio = 0.5f;
    return phys3d_add_joint(w, j);
}
idx phys3d_add_mouse(phys3d_world& w, idx body, vec3 grab, float max_force) {
    phys3d_joint j = phys3d__joint_base(w, PHYS3D_JOINT_MOUSE, PIX_INVALID, body, grab, grab, v3(0, 1, 0));
    j.target = grab;
    j.hertz = 5.0f;
    j.damping_ratio = 0.7f;
    const phys3d_body* b = phys3d_get_body(w, body);
    float mass = b && b->inv_mass > 0.0f ? 1.0f / b->inv_mass : 1.0f;
    j.max_force = max_force > 0.0f ? max_force : 50.0f * mass * maxf(v3len(w.gravity), 1.0f);
    return phys3d_add_joint(w, j);
}

phys3d_joint* phys3d_get_joint(phys3d_world& w, idx id) {
    return id < w.joint_count && w.joints[id].active ? &w.joints[id] : 0;
}

static void phys3d__unlink_joint(phys3d_world& w, idx body, idx id) {
    idx* link = &w.bodies[body].joint_list;
    while (*link != PIX_INVALID) {
        phys3d_joint& j = w.joints[*link];
        idx* next = j.a == body ? &j.next_a : &j.next_b;
        if (*link == id) { *link = *next; return; }
        link = next;
    }
}

void phys3d_remove_joint(phys3d_world& w, idx id) {
    phys3d_joint* j = phys3d_get_joint(w, id);
    if (!j) return;
    if (j->a != PIX_INVALID) { phys3d__unlink_joint(w, j->a, id); phys3d__wake_body(w.bodies[j->a]); }
    if (j->b != PIX_INVALID) { phys3d__unlink_joint(w, j->b, id); phys3d__wake_body(w.bodies[j->b]); }
    j->active = false;
    w.free_joints[w.free_joint_count++] = id;
}

void phys3d_joint_anchors(const phys3d_world& w, const phys3d_joint& j, vec3* a, vec3* b) {
    const phys3d_body& A = phys3d__jbody(w, j.a);
    const phys3d_body& B = phys3d__jbody(w, j.b);
    *a = j.type == PHYS3D_JOINT_MOUSE ? j.target : A.position + quat_rotate(A.rotation, j.local_a);
    *b = B.position + quat_rotate(B.rotation, j.local_b);
}

// the signed angle from reference `ra` to `rb` about `axis`
static inline float phys3d__angle_about(vec3 ra, vec3 rb, vec3 axis) {
    return atan2f(v3dot(v3cross(ra, rb), axis), v3dot(ra, rb));
}

float phys3d_hinge_angle(const phys3d_world& w, const phys3d_joint& j) {
    const phys3d_body& A = phys3d__jbody(w, j.a);
    const phys3d_body& B = phys3d__jbody(w, j.b);
    return phys3d__angle_about(quat_rotate(A.rotation, j.local_ref_a), quat_rotate(B.rotation, j.local_ref_b),
                               quat_rotate(A.rotation, j.local_axis_a));
}

static bool phys3d__joined(const phys3d_world& w, idx a, idx b) {
    for (idx id = w.bodies[a].joint_list; id != PIX_INVALID;) {
        const phys3d_joint& j = w.joints[id];
        idx other = j.a == a ? j.b : j.a;
        if (other == b && !j.collide_connected) return true;
        id = j.a == a ? j.next_a : j.next_b;
    }
    return false;
}

// soft constraints, exactly as in physics2d.hpp
struct phys3d__soft { float bias_rate, mass_scale, impulse_scale; };
static phys3d__soft phys3d__make_soft(float hertz, float zeta, float h) {
    phys3d__soft s = { 0.0f, 1.0f, 0.0f };
    if (hertz <= 0.0f) return s;
    float omega = 2.0f * PIX_PI * hertz;
    float a1 = 2.0f * zeta + h * omega;
    float a2 = h * omega * a1;
    float a3 = 1.0f / (1.0f + a2);
    s.bias_rate = omega / a1;
    s.mass_scale = a2 * a3;
    s.impulse_scale = a3;
    return s;
}

static inline void phys3d__apply_at(phys3d_body& A, phys3d_body& B, vec3 ra, vec3 rb, vec3 P) {
    A.velocity -= P * A.inv_mass;
    A.angular_velocity -= A.inv_inertia * v3cross(ra, P);
    B.velocity += P * B.inv_mass;
    B.angular_velocity += B.inv_inertia * v3cross(rb, P);
}
static inline void phys3d__apply_angular(phys3d_body& A, phys3d_body& B, vec3 L) {
    A.angular_velocity -= A.inv_inertia * L;
    B.angular_velocity += B.inv_inertia * L;
}
// one constraint row: linear part `lin`, angular parts angA (on a) and angB (on b)
static inline void phys3d__apply_row(phys3d_body& A, phys3d_body& B, vec3 lin, vec3 angA, vec3 angB, float imp) {
    A.velocity -= lin * (imp * A.inv_mass);
    A.angular_velocity -= A.inv_inertia * (angA * imp);
    B.velocity += lin * (imp * B.inv_mass);
    B.angular_velocity += B.inv_inertia * (angB * imp);
}
static inline float phys3d__row_speed(const phys3d_body& A, const phys3d_body& B, vec3 lin, vec3 angA, vec3 angB) {
    return v3dot(lin, B.velocity - A.velocity) + v3dot(angB, B.angular_velocity) - v3dot(angA, A.angular_velocity);
}
static inline float phys3d__row_mass(const phys3d_body& A, const phys3d_body& B, vec3 lin, vec3 angA, vec3 angB) {
    float k = (A.inv_mass + B.inv_mass) * v3dot(lin, lin) + v3dot(angA, A.inv_inertia * angA) + v3dot(angB, B.inv_inertia * angB);
    return k > 0.0f ? 1.0f / k : 0.0f;
}
static inline float phys3d__angular_mass(const phys3d_body& A, const phys3d_body& B, vec3 axis) {
    float k = v3dot(axis, A.inv_inertia * axis) + v3dot(axis, B.inv_inertia * axis);
    return k > 0.0f ? 1.0f / k : 0.0f;
}

// -[r]x M [r]x: the angular part of a point constraint's effective mass
static mat3 phys3d__skew_sandwich(const mat3& M, vec3 r) {
    mat3 s;
    s.data[0] = 0;    s.data[1] = r.z;  s.data[2] = -r.y;
    s.data[3] = -r.z; s.data[4] = 0;    s.data[5] = r.x;
    s.data[6] = r.y;  s.data[7] = -r.x; s.data[8] = 0;
    mat3 t = mat3_mul(mat3_mul(s, M), s);
    for (int i = 0; i < 9; i++) t.data[i] = -t.data[i];
    return t;
}

static vec3 phys3d__point_impulse(const phys3d_body& A, const phys3d_body& B, vec3 ra, vec3 rb, vec3 C,
                                  phys3d__soft s, vec3 accumulated) {
    vec3 cdot = B.velocity + v3cross(B.angular_velocity, rb) - A.velocity - v3cross(A.angular_velocity, ra);
    mat3 k = phys3d__add(phys3d__skew_sandwich(A.inv_inertia, ra), phys3d__skew_sandwich(B.inv_inertia, rb));
    float m = A.inv_mass + B.inv_mass;
    k.data[0] += m; k.data[4] += m; k.data[8] += m;
    if (fabsf(mat3_determinant(k)) < 1e-20f) return v3(0, 0, 0);
    vec3 x = mat3_inverse(k) * (cdot + C * s.bias_rate);
    return -x * s.mass_scale - accumulated * s.impulse_scale;
}

// lock all relative rotation: b keeps the orientation it had in a's frame
static vec3 phys3d__lock_impulse(const phys3d_body& A, const phys3d_body& B, quat relative, phys3d__soft s, vec3 accumulated) {
    quat target = quat_mul(A.rotation, relative);
    quat e = quat_mul(B.rotation, quat_conjugate(target));
    if (e.w < 0.0f) { e.x = -e.x; e.y = -e.y; e.z = -e.z; e.w = -e.w; }
    vec3 C = v3(e.x, e.y, e.z) * 2.0f;
    vec3 cdot = B.angular_velocity - A.angular_velocity;
    mat3 k = phys3d__add(A.inv_inertia, B.inv_inertia);
    if (fabsf(mat3_determinant(k)) < 1e-20f) return v3(0, 0, 0);
    vec3 x = mat3_inverse(k) * (cdot + C * s.bias_rate);
    return -x * s.mass_scale - accumulated * s.impulse_scale;
}

static float phys3d__limit(float C, float cdot, float mass, float inv_h, phys3d__soft rigid, float* accumulated) {
    float bias = 0.0f, ms = 1.0f, is = 0.0f;
    if (C > 0.0f) bias = C * inv_h;
    else { bias = rigid.bias_rate * C; ms = rigid.mass_scale; is = rigid.impulse_scale; }
    float imp = -mass * ms * (cdot + bias) - is * *accumulated;
    float total = maxf(*accumulated + imp, 0.0f);
    imp = total - *accumulated;
    *accumulated = total;
    return imp;
}

static void phys3d__joints_prepare(phys3d_world& w) {
    for (size_t i = 0; i < w.joint_count; i++) {
        phys3d_joint& j = w.joints[i];
        if (!j.active) continue;
        phys3d_body& A = phys3d__jbody(w, j.a);
        phys3d_body& B = phys3d__jbody(w, j.b);
        j.idle = phys3d__idle(A) && phys3d__idle(B);
        if (j.idle) continue;
        j.ra = quat_rotate(A.rotation, j.local_a);
        j.rb = quat_rotate(B.rotation, j.local_b);
        j.d = (B.position + j.rb) - (A.position + j.ra);
        if (j.type == PHYS3D_JOINT_DISTANCE) {
            float len = v3len(j.d);
            j.axis = len > 1e-6f ? j.d / len : v3(0, 1, 0);
        } else {
            j.axis = quat_rotate(A.rotation, j.local_axis_a);
        }
        phys3d__basis(j.axis, &j.perp1, &j.perp2);
    }
}

// hinge: the two rows that keep b's hinge axis on a's
static inline void phys3d__hinge_rows(const phys3d_joint& j, const phys3d_body& B, vec3* r1, vec3* r2, float* c1, float* c2) {
    vec3 ab = quat_rotate(B.rotation, j.local_axis_b);
    *r1 = v3cross(ab, j.perp1);
    *r2 = v3cross(ab, j.perp2);
    *c1 = v3dot(ab, j.perp1);
    *c2 = v3dot(ab, j.perp2);
}

static void phys3d__joints_warm(phys3d_world& w) {
    for (size_t i = 0; i < w.joint_count; i++) {
        phys3d_joint& j = w.joints[i];
        if (!j.active || j.idle) continue;
        phys3d_body& A = phys3d__jbody(w, j.a);
        phys3d_body& B = phys3d__jbody(w, j.b);
        float axial = j.axial_impulse + j.motor_impulse + j.lower_impulse - j.upper_impulse;
        switch (j.type) {
        case PHYS3D_JOINT_BALL: {
            phys3d__apply_at(A, B, j.ra, j.rb, j.linear_impulse);
            vec3 ab = quat_rotate(B.rotation, j.local_axis_b);
            phys3d__apply_angular(A, B, ab * (j.lower_impulse - j.upper_impulse) + j.angular_impulse);
        } break;
        case PHYS3D_JOINT_HINGE: {
            phys3d__apply_at(A, B, j.ra, j.rb, j.linear_impulse);
            vec3 r1, r2; float c1, c2;
            phys3d__hinge_rows(j, B, &r1, &r2, &c1, &c2);
            phys3d__apply_angular(A, B, r1 * j.angular_impulse.x + r2 * j.angular_impulse.y + j.axis * axial);
        } break;
        case PHYS3D_JOINT_FIXED:
        case PHYS3D_JOINT_MOUSE:
            phys3d__apply_at(A, B, j.ra, j.rb, j.linear_impulse);
            if (j.type == PHYS3D_JOINT_FIXED) phys3d__apply_angular(A, B, j.angular_impulse);
            break;
        case PHYS3D_JOINT_SLIDER: {
            vec3 dra = j.d + j.ra;
            phys3d__apply_row(A, B, j.perp1, v3cross(dra, j.perp1), v3cross(j.rb, j.perp1), j.linear_impulse.x);
            phys3d__apply_row(A, B, j.perp2, v3cross(dra, j.perp2), v3cross(j.rb, j.perp2), j.linear_impulse.y);
            phys3d__apply_row(A, B, j.axis, v3cross(dra, j.axis), v3cross(j.rb, j.axis), axial);
            phys3d__apply_angular(A, B, j.angular_impulse);
        } break;
        case PHYS3D_JOINT_DISTANCE:
            phys3d__apply_row(A, B, j.axis, v3cross(j.ra, j.axis), v3cross(j.rb, j.axis), axial);
            break;
        }
    }
}

static void phys3d__joints_solve(phys3d_world& w, float h, bool use_bias) {
    float inv_h = 1.0f / h;
    phys3d__soft rigid = { 0.0f, 1.0f, 0.0f };
    if (use_bias) rigid = phys3d__make_soft(w.joint_hertz, w.joint_damping, h);
    for (size_t i = 0; i < w.joint_count; i++) {
        phys3d_joint& j = w.joints[i];
        if (!j.active || j.idle) continue;
        phys3d_body& A = phys3d__jbody(w, j.a);
        phys3d_body& B = phys3d__jbody(w, j.b);

        if (j.type == PHYS3D_JOINT_HINGE) {
            float am = phys3d__angular_mass(A, B, j.axis);
            float rel = v3dot(B.angular_velocity - A.angular_velocity, j.axis);
            if (j.enable_motor && am > 0.0f) {
                float old = j.motor_impulse, max = j.max_motor_force * h;
                j.motor_impulse = clampf(old - am * (rel - j.motor_speed), -max, max);
                phys3d__apply_angular(A, B, j.axis * (j.motor_impulse - old));
            }
            if (j.enable_limit && am > 0.0f) {
                float angle = phys3d__angle_about(quat_rotate(A.rotation, j.local_ref_a), quat_rotate(B.rotation, j.local_ref_b), j.axis);
                rel = v3dot(B.angular_velocity - A.angular_velocity, j.axis);
                float imp = phys3d__limit(angle - j.lower, rel, am, inv_h, rigid, &j.lower_impulse);
                phys3d__apply_angular(A, B, j.axis * imp);
                rel = v3dot(B.angular_velocity - A.angular_velocity, j.axis);
                imp = phys3d__limit(j.upper - angle, -rel, am, inv_h, rigid, &j.upper_impulse);
                phys3d__apply_angular(A, B, j.axis * -imp);
            }
            // keep the axes aligned: two angular rows, solved together
            vec3 r1, r2; float c1, c2;
            phys3d__hinge_rows(j, B, &r1, &r2, &c1, &c2);
            mat3 isum = phys3d__add(A.inv_inertia, B.inv_inertia);
            float k11 = v3dot(r1, isum * r1), k12 = v3dot(r1, isum * r2), k22 = v3dot(r2, isum * r2);
            float det = k11 * k22 - k12 * k12;
            if (det > 1e-20f) {
                vec3 dw = B.angular_velocity - A.angular_velocity;
                float b1 = v3dot(r1, dw) + rigid.bias_rate * c1, b2 = v3dot(r2, dw) + rigid.bias_rate * c2;
                float inv = 1.0f / det;
                float x1 = inv * (k22 * b1 - k12 * b2), x2 = inv * (k11 * b2 - k12 * b1);
                float i1 = -x1 * rigid.mass_scale - rigid.impulse_scale * j.angular_impulse.x;
                float i2 = -x2 * rigid.mass_scale - rigid.impulse_scale * j.angular_impulse.y;
                j.angular_impulse.x += i1; j.angular_impulse.y += i2;
                phys3d__apply_angular(A, B, r1 * i1 + r2 * i2);
            }
            vec3 imp = phys3d__point_impulse(A, B, j.ra, j.rb, j.d, rigid, j.linear_impulse);
            j.linear_impulse += imp;
            phys3d__apply_at(A, B, j.ra, j.rb, imp);
        } else if (j.type == PHYS3D_JOINT_BALL) {
            vec3 aa = quat_rotate(A.rotation, j.local_axis_a), ab = quat_rotate(B.rotation, j.local_axis_b);
            if (j.swing_limit > 0.0f) {
                // the swing cone: the angle between the two twist axes
                vec3 n = v3cross(aa, ab);
                float s = v3len(n);
                float angle = atan2f(s, v3dot(aa, ab));
                if (s > 1e-6f) {
                    n = n / s;
                    float am = phys3d__angular_mass(A, B, n);
                    float rel = v3dot(B.angular_velocity - A.angular_velocity, n);
                    float imp = phys3d__limit(j.swing_limit - angle, -rel, am, inv_h, rigid, &j.swing_impulse);
                    phys3d__apply_angular(A, B, n * -imp);
                    j.angular_impulse = n * -j.swing_impulse;
                }
            }
            if (j.enable_limit) {
                // twist about b's axis, measured from a's reference laid into the same plane
                vec3 ra = quat_rotate(A.rotation, j.local_ref_a), rb = quat_rotate(B.rotation, j.local_ref_b);
                ra = ra - ab * v3dot(ra, ab);
                if (v3dot(ra, ra) > 1e-10f) {
                    ra = v3norm(ra);
                    float twist = phys3d__angle_about(ra, rb, ab);
                    float am = phys3d__angular_mass(A, B, ab);
                    float rel = v3dot(B.angular_velocity - A.angular_velocity, ab);
                    float imp = phys3d__limit(twist - j.lower, rel, am, inv_h, rigid, &j.lower_impulse);
                    phys3d__apply_angular(A, B, ab * imp);
                    rel = v3dot(B.angular_velocity - A.angular_velocity, ab);
                    imp = phys3d__limit(j.upper - twist, -rel, am, inv_h, rigid, &j.upper_impulse);
                    phys3d__apply_angular(A, B, ab * -imp);
                }
            }
            vec3 imp = phys3d__point_impulse(A, B, j.ra, j.rb, j.d, rigid, j.linear_impulse);
            j.linear_impulse += imp;
            phys3d__apply_at(A, B, j.ra, j.rb, imp);
        } else if (j.type == PHYS3D_JOINT_FIXED) {
            phys3d__soft soft = j.hertz > 0.0f ? phys3d__make_soft(j.hertz, j.damping_ratio, h) : rigid;
            vec3 L = phys3d__lock_impulse(A, B, j.relative_rotation, soft, j.angular_impulse);
            j.angular_impulse += L;
            phys3d__apply_angular(A, B, L);
            vec3 imp = phys3d__point_impulse(A, B, j.ra, j.rb, j.d, soft, j.linear_impulse);
            j.linear_impulse += imp;
            phys3d__apply_at(A, B, j.ra, j.rb, imp);
        } else if (j.type == PHYS3D_JOINT_MOUSE) {
            phys3d__soft soft = phys3d__make_soft(j.hertz, j.damping_ratio, h);
            vec3 C = B.position + j.rb - j.target;
            vec3 imp = phys3d__point_impulse(A, B, j.ra, j.rb, C, soft, j.linear_impulse);
            vec3 old = j.linear_impulse;
            j.linear_impulse += imp;
            float max = j.max_force * h, len = v3len(j.linear_impulse);
            if (len > max) j.linear_impulse *= max / len;
            phys3d__apply_at(A, B, j.ra, j.rb, j.linear_impulse - old);
            // a little spin damping so a grabbed body does not twirl forever
            B.angular_velocity *= 1.0f / (1.0f + h * 2.0f);
        } else if (j.type == PHYS3D_JOINT_SLIDER) {
            vec3 dra = j.d + j.ra;
            vec3 axA = v3cross(dra, j.axis), axB = v3cross(j.rb, j.axis);
            float am = phys3d__row_mass(A, B, j.axis, axA, axB);
            if (j.enable_motor) {
                float old = j.motor_impulse, max = j.max_motor_force * h;
                j.motor_impulse = clampf(old + am * (j.motor_speed - phys3d__row_speed(A, B, j.axis, axA, axB)), -max, max);
                phys3d__apply_row(A, B, j.axis, axA, axB, j.motor_impulse - old);
            }
            if (j.enable_limit) {
                float travel = v3dot(j.axis, j.d);
                float imp = phys3d__limit(travel - j.lower, phys3d__row_speed(A, B, j.axis, axA, axB), am, inv_h, rigid, &j.lower_impulse);
                phys3d__apply_row(A, B, j.axis, axA, axB, imp);
                imp = phys3d__limit(j.upper - travel, -phys3d__row_speed(A, B, j.axis, axA, axB), am, inv_h, rigid, &j.upper_impulse);
                phys3d__apply_row(A, B, j.axis, axA, axB, -imp);
            }
            vec3 L = phys3d__lock_impulse(A, B, j.relative_rotation, rigid, j.angular_impulse);
            j.angular_impulse += L;
            phys3d__apply_angular(A, B, L);
            vec3 perps[2] = { j.perp1, j.perp2 };
            for (int k = 0; k < 2; k++) {
                vec3 p = perps[k], pa = v3cross(dra, p), pb = v3cross(j.rb, p);
                float m = phys3d__row_mass(A, B, p, pa, pb);
                float acc = k ? j.linear_impulse.y : j.linear_impulse.x;
                float imp = -m * rigid.mass_scale * (phys3d__row_speed(A, B, p, pa, pb) + rigid.bias_rate * v3dot(j.d, p))
                          - rigid.impulse_scale * acc;
                if (k) j.linear_impulse.y += imp; else j.linear_impulse.x += imp;
                phys3d__apply_row(A, B, p, pa, pb, imp);
            }
        } else if (j.type == PHYS3D_JOINT_DISTANCE) {
            vec3 axA = v3cross(j.ra, j.axis), axB = v3cross(j.rb, j.axis);
            float am = phys3d__row_mass(A, B, j.axis, axA, axB);
            float len = v3dot(j.d, j.axis);
            if (!j.enable_spring || j.hertz > 0.0f) {
                phys3d__soft soft = j.enable_spring ? phys3d__make_soft(j.hertz, j.damping_ratio, h) : rigid;
                float imp = -am * soft.mass_scale * (phys3d__row_speed(A, B, j.axis, axA, axB) + soft.bias_rate * (len - j.length))
                          - soft.impulse_scale * j.axial_impulse;
                j.axial_impulse += imp;
                phys3d__apply_row(A, B, j.axis, axA, axB, imp);
            }
            if (j.enable_limit) {
                float imp = phys3d__limit(len - j.lower, phys3d__row_speed(A, B, j.axis, axA, axB), am, inv_h, rigid, &j.lower_impulse);
                phys3d__apply_row(A, B, j.axis, axA, axB, imp);
                imp = phys3d__limit(j.upper - len, -phys3d__row_speed(A, B, j.axis, axA, axB), am, inv_h, rigid, &j.upper_impulse);
                phys3d__apply_row(A, B, j.axis, axA, axB, -imp);
            }
        }
    }
}

static void phys3d__joints_finish(phys3d_world& w, float h) {
    for (size_t i = 0; i < w.joint_count; i++) {
        phys3d_joint& j = w.joints[i];
        if (!j.active || j.idle) continue;
        j.reaction = (v3len(j.linear_impulse) + fabsf(j.axial_impulse + j.lower_impulse - j.upper_impulse)) / h;
        if (j.break_force > 0.0f && j.reaction > j.break_force) {
            phys3d__reserve(w.events, w.event_capacity, w.event_count + 1);
            phys3d_event& e = w.events[w.event_count++];
            memset(&e, 0, sizeof(e));
            e.type = PHYS3D_JOINT_BREAK; e.a = j.a; e.b = j.b; e.joint = (idx)i;
            e.speed = j.reaction;
            phys3d_remove_joint(w, (idx)i);
        }
    }
}

// ---- islands and sleeping (as in 2D) ----

static uint32_t phys3d__find(uint32_t* parent, uint32_t i) {
    while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
    return i;
}

static void phys3d__islands_wake(phys3d_world& w) {
    uint32_t* p = w.island;
    for (size_t i = 0; i < w.body_count; i++) { p[i] = (uint32_t)i; w.island_awake[i] = 0; w.island_sleep[i] = 1e30f; }
    for (size_t i = 0; i < w.contact_count; i++) {
        const phys3d_contact& c = w.contacts[i];
        if (c.trigger || !c.touching) continue;
        phys3d_body& a = w.bodies[c.a];
        phys3d_body& b = w.bodies[c.b];
        if (a.type == PHYS3D_DYNAMIC && b.type == PHYS3D_DYNAMIC) p[phys3d__find(p, c.a)] = phys3d__find(p, c.b);
        else if (a.type == PHYS3D_KINEMATIC && !phys3d__idle(a)) phys3d__wake_body(b);
        else if (b.type == PHYS3D_KINEMATIC && !phys3d__idle(b)) phys3d__wake_body(a);
    }
    for (size_t i = 0; i < w.joint_count; i++) {
        const phys3d_joint& j = w.joints[i];
        if (!j.active || j.a == PIX_INVALID || j.b == PIX_INVALID) continue;
        phys3d_body& a = w.bodies[j.a];
        phys3d_body& b = w.bodies[j.b];
        if (a.type == PHYS3D_DYNAMIC && b.type == PHYS3D_DYNAMIC) p[phys3d__find(p, j.a)] = phys3d__find(p, j.b);
        else if (a.type == PHYS3D_KINEMATIC && !phys3d__idle(a)) phys3d__wake_body(b);
        else if (b.type == PHYS3D_KINEMATIC && !phys3d__idle(b)) phys3d__wake_body(a);
    }
    for (size_t i = 0; i < w.body_count; i++) {
        const phys3d_body& b = w.bodies[i];
        if (b.active && b.type == PHYS3D_DYNAMIC && !b.sleeping) w.island_awake[phys3d__find(p, (uint32_t)i)] = 1;
    }
    for (size_t i = 0; i < w.body_count; i++) {
        phys3d_body& b = w.bodies[i];
        if (b.active && b.type == PHYS3D_DYNAMIC && b.sleeping && w.island_awake[phys3d__find(p, (uint32_t)i)])
            phys3d__wake_body(b);
    }
}

static void phys3d__islands_sleep(phys3d_world& w, float dt) {
    if (!w.allow_sleep) return;
    float lin2 = w.sleep_linear * w.sleep_linear, ang2 = w.sleep_angular * w.sleep_angular;
    for (size_t i = 0; i < w.body_count; i++) {
        phys3d_body& b = w.bodies[i];
        if (!b.active || b.type != PHYS3D_DYNAMIC || b.sleeping) continue;
        bool still = !b.never_sleep && v3dot(b.velocity, b.velocity) <= lin2 && v3dot(b.angular_velocity, b.angular_velocity) <= ang2;
        b.sleep_time = still ? b.sleep_time + dt : 0.0f;
        uint32_t r = phys3d__find(w.island, (uint32_t)i);
        w.island_sleep[r] = minf(w.island_sleep[r], b.sleep_time);
    }
    for (size_t i = 0; i < w.body_count; i++) {
        phys3d_body& b = w.bodies[i];
        if (!b.active || b.type != PHYS3D_DYNAMIC || b.sleeping) continue;
        if (w.island_sleep[phys3d__find(w.island, (uint32_t)i)] < w.time_to_sleep) continue;
        b.sleeping = true;
        b.velocity = b.angular_velocity = v3(0, 0, 0);
        b.prev_position = b.position;
        b.prev_rotation = b.rotation;
    }
}

// ---- broadphase ----

static inline int32_t phys3d__cell(float x, float inv) { return (int32_t)floorf(x * inv); }
static inline uint32_t phys3d__bucket(int32_t x, int32_t y, int32_t z) {
    return ((uint32_t)x * 73856093u ^ (uint32_t)y * 19349663u ^ (uint32_t)z * 83492791u) & (PHYS3D_GRID_BUCKETS - 1);
}

static bool phys3d__is_oversize(const phys3d_world& w, const phys3d_body& b) {
    if (b.shape.type == PHYS3D_PLANE) return true;
    float inv = 1.0f / w.cell_size;
    vec3 e = (b.bounds.max - b.bounds.min) * inv;
    return (e.x + 1.0f) * (e.y + 1.0f) * (e.z + 1.0f) > (float)PHYS3D_OVERSIZE_CELLS;
}

// every body for which statics == (type is static), and not oversize
static void phys3d__build_grid(phys3d_world& w, phys3d__grid& g, bool statics) {
    float inv = 1.0f / w.cell_size;
    uint32_t* start = g.start;
    memset(start, 0, (PHYS3D_GRID_BUCKETS + 1) * sizeof(uint32_t));
    size_t total = 0;
    for (int pass = 0; pass < 2; pass++) {
        if (pass == 1) {
            phys3d__reserve(g.items, g.capacity, total);
            g.count = total;
            uint32_t sum = 0;
            for (int i = 0; i < PHYS3D_GRID_BUCKETS; i++) { uint32_t c = start[i]; start[i] = sum; sum += c; }
        }
        for (size_t i = 0; i < w.body_count; i++) {
            const phys3d_body& b = w.bodies[i];
            if (!b.active || b.oversize || (b.type == PHYS3D_STATIC) != statics) continue;
            int32_t x0 = phys3d__cell(b.bounds.min.x, inv), x1 = phys3d__cell(b.bounds.max.x, inv);
            int32_t y0 = phys3d__cell(b.bounds.min.y, inv), y1 = phys3d__cell(b.bounds.max.y, inv);
            int32_t z0 = phys3d__cell(b.bounds.min.z, inv), z1 = phys3d__cell(b.bounds.max.z, inv);
            for (int32_t z = z0; z <= z1; z++)
                for (int32_t y = y0; y <= y1; y++)
                    for (int32_t x = x0; x <= x1; x++) {
                        uint32_t bk = phys3d__bucket(x, y, z);
                        if (pass == 0) { start[bk]++; total++; continue; }
                        phys3d__cell_item& it = g.items[start[bk]++];
                        it.cx = x; it.cy = y; it.cz = z; it.body = (idx)i;
                    }
        }
    }
    memmove(start + 1, start, PHYS3D_GRID_BUCKETS * sizeof(uint32_t));
    start[0] = 0;
}

static inline bool phys3d__owns_pair(const aabb& a, const aabb& b, int32_t x, int32_t y, int32_t z, float inv) {
    return phys3d__cell(maxf(a.min.x, b.min.x), inv) == x && phys3d__cell(maxf(a.min.y, b.min.y), inv) == y &&
           phys3d__cell(maxf(a.min.z, b.min.z), inv) == z;
}

static inline bool phys3d__layers_ok(const phys3d_body& a, const phys3d_body& b) {
    return phys_layers_collide(a.layer, a.mask, b.layer, b.mask);
}

// ---- contacts ----

static inline uint64_t phys3d__pair_key(idx a, idx b) { return ((uint64_t)a << 32) | b; }
static inline uint32_t phys3d__hash64(uint64_t k) {
    k ^= k >> 33; k *= 0xff51afd7ed558ccdull; k ^= k >> 33;
    return (uint32_t)k;
}

// the first of last step's contacts for this pair (a pair's contacts are consecutive)
static uint32_t phys3d__find_prev(const phys3d_world& w, idx a, idx b) {
    if (!w.table_size) return 0xFFFFFFFFu;
    size_t mask = w.table_size - 1;
    for (size_t h = phys3d__hash64(phys3d__pair_key(a, b)) & mask;; h = (h + 1) & mask) {
        uint32_t i = w.prev_table[h];
        if (i == 0xFFFFFFFFu) return i;
        if (w.prev_contacts[i].a == a && w.prev_contacts[i].b == b) return i;
    }
}

static void phys3d__push_event(phys3d_world& w, uint8_t type, idx a, idx b, vec3 point, vec3 normal, float speed) {
    phys3d__reserve(w.events, w.event_capacity, w.event_count + 1);
    phys3d_event& e = w.events[w.event_count++];
    memset(&e, 0, sizeof(e));
    e.type = type; e.a = a; e.b = b; e.joint = PIX_INVALID; e.point = point; e.normal = normal; e.speed = speed;
}

// Fills in a freshly collided contact and warm starts it from last step's
// points that sit (in a's frame) within reach of the new ones.
static void phys3d__finish_contact(phys3d_world& w, phys3d_contact& c, idx ia, idx ib, uint32_t sub, bool trigger, uint32_t prev) {
    const phys3d_body& a = w.bodies[ia];
    const phys3d_body& b = w.bodies[ib];
    c.a = ia; c.b = ib; c.sub = sub;
    c.trigger = trigger;
    c.friction = sqrtf(a.friction * b.friction);
    c.restitution = maxf(a.restitution, b.restitution);
    // a resisting torque of rolling_friction * normal force * contact radius;
    // the radius is the rounder body's (the one that can actually roll)
    float ra = a.shape.type == PHYS3D_BOX ? minf(a.shape.half.x, minf(a.shape.half.y, a.shape.half.z)) : a.shape.radius;
    float rb = b.shape.type == PHYS3D_BOX ? minf(b.shape.half.x, minf(b.shape.half.y, b.shape.half.z)) : b.shape.radius;
    if (a.type != PHYS3D_DYNAMIC) ra = rb;
    if (b.type != PHYS3D_DYNAMIC) rb = ra;
    c.rolling = maxf(a.rolling_friction, b.rolling_friction) * minf(ra, rb);
    c.rolling_impulse = v3(0, 0, 0);
    phys3d__basis(c.normal, &c.tangent[0], &c.tangent[1]);
    float min_sep = 1e30f;
    for (int i = 0; i < c.count; i++) {
        min_sep = minf(min_sep, c.points[i].separation);
        c.points[i].local_a = phys3d__to_local(a, c.points[i].point);
    }
    c.touching = trigger || min_sep <= w.slop;
    c.persisted = false;
    c.prev = prev;
    if (trigger || prev == 0xFFFFFFFFu) return;
    float reach2 = 16.0f * w.slop * w.slop + 4.0f * w.speculative * w.speculative;
    for (uint32_t k = prev; k < w.prev_count && w.prev_contacts[k].a == ia && w.prev_contacts[k].b == ib; k++) {
        const phys3d_contact& old = w.prev_contacts[k];
        if (old.sub != sub || v3dot(old.normal, c.normal) < 0.95f) continue;
        for (int i = 0; i < c.count; i++)
            for (int j = 0; j < old.count; j++) {
                vec3 d = old.points[j].local_a - c.points[i].local_a;
                if (v3dot(d, d) > reach2) continue;
                c.points[i].normal_impulse = old.points[j].normal_impulse;
                c.points[i].tangent_impulse[0] = old.points[j].tangent_impulse[0];
                c.points[i].tangent_impulse[1] = old.points[j].tangent_impulse[1];
                break;
            }
    }
}

static void phys3d__narrowphase(phys3d_world& w, idx ia, idx ib, float dt) {
    if (ia > ib) { idx t = ia; ia = ib; ib = t; }
    phys3d_body& a = w.bodies[ia];
    phys3d_body& b = w.bodies[ib];
    if (!phys3d__layers_ok(a, b)) return;
    bool trigger = a.trigger || b.trigger;
    if (a.trigger && b.trigger) return;
    if (!trigger && a.type != PHYS3D_DYNAMIC && b.type != PHYS3D_DYNAMIC) return;
    if (!aabb_overlap(a.bounds, b.bounds)) return;
    if (a.joint_list != PIX_INVALID && b.joint_list != PIX_INVALID && phys3d__joined(w, ia, ib)) return;
    uint32_t prev = phys3d__find_prev(w, ia, ib);
    if (phys3d__idle(a) && phys3d__idle(b)) {
        // nothing moved: keep last step's contacts for the pair as they were
        for (uint32_t k = prev; k != 0xFFFFFFFFu && k < w.prev_count && w.prev_contacts[k].a == ia && w.prev_contacts[k].b == ib; k++) {
            phys3d__reserve(w.contacts, w.contact_capacity, w.contact_count + 1);
            phys3d_contact& c = w.contacts[w.contact_count++];
            c = w.prev_contacts[k];
            c.prev = k == prev ? prev : 0xFFFFFFFFu;
            c.persisted = false;
        }
        return;
    }
    float margin = trigger ? 0.0f : w.speculative + v3len(b.velocity - a.velocity) * dt;
    // one of them a mesh: a contact per nearby triangle
    const phys3d_body* mesh_body = a.shape.type == PHYS3D_MESH ? &a : (b.shape.type == PHYS3D_MESH ? &b : 0);
    if (mesh_body) {
        if (a.shape.type == PHYS3D_MESH && b.shape.type == PHYS3D_MESH) return;
        const phys3d_body& other = mesh_body == &a ? b : a;
        bool mesh_is_a = mesh_body == &a;
        phys3d__core oc;
        phys3d__make_core(other.shape, other.position, other.rotation, oc);
        const phys3d_mesh& mesh = *mesh_body->shape.mesh;
        quat inv = quat_conjugate(mesh_body->rotation);
        // the other body's bounds, in the mesh's frame
        aabb box = aabb_make(v3(1e30f, 1e30f, 1e30f), v3(-1e30f, -1e30f, -1e30f));
        for (int i = 0; i < 8; i++) {
            vec3 corner = v3((i & 1) ? other.bounds.max.x : other.bounds.min.x, (i & 2) ? other.bounds.max.y : other.bounds.min.y,
                             (i & 4) ? other.bounds.max.z : other.bounds.min.z);
            vec3 l = quat_rotate(inv, corner - mesh_body->position);
            box.min = v3min(box.min, l); box.max = v3max(box.max, l);
        }
        bool first = true;
        phys3d__mesh_query(mesh, box, [&](uint32_t t) {
            vec3 v0 = mesh_body->position + quat_rotate(mesh_body->rotation, mesh.vertices[mesh.indices[t * 3]]);
            vec3 v1 = mesh_body->position + quat_rotate(mesh_body->rotation, mesh.vertices[mesh.indices[t * 3 + 1]]);
            vec3 v2 = mesh_body->position + quat_rotate(mesh_body->rotation, mesh.vertices[mesh.indices[t * 3 + 2]]);
            phys3d__core tri;
            phys3d__triangle_core(v0, v1, v2, tri);
            // one-sided: a body behind the surface passes through rather than snagging
            if (v3dot(oc.centre - v0, tri.axis[0]) < 0.0f) return;
            phys3d__reserve(w.contacts, w.contact_capacity, w.contact_count + 1);
            phys3d_contact& c = w.contacts[w.contact_count];
            bool hit = mesh_is_a ? phys3d__collide(tri, oc, w.slop, margin, c) : phys3d__collide(oc, tri, w.slop, margin, c);
            if (!hit) return;
            float min_sep = 1e30f;
            for (int i = 0; i < c.count; i++) min_sep = minf(min_sep, c.points[i].separation);
            if (trigger && min_sep >= 0.0f) return;
            phys3d__finish_contact(w, c, ia, ib, t, trigger, prev);
            if (!first) c.prev = 0xFFFFFFFFu;   // only a pair's first contact carries the link for events
            first = false;
            w.contact_count++;
        });
        return;
    }
    phys3d__core ca, cb;
    phys3d__make_core(a.shape, a.position, a.rotation, ca);
    phys3d__make_core(b.shape, b.position, b.rotation, cb);
    phys3d__reserve(w.contacts, w.contact_capacity, w.contact_count + 1);
    phys3d_contact& c = w.contacts[w.contact_count];
    if (!phys3d__collide(ca, cb, w.slop, margin, c)) return;
    float min_sep = 1e30f;
    for (int i = 0; i < c.count; i++) min_sep = minf(min_sep, c.points[i].separation);
    if (trigger && min_sep >= 0.0f) return;
    phys3d__finish_contact(w, c, ia, ib, 0, trigger, prev);
    w.contact_count++;
}

static void phys3d__find_contacts(phys3d_world& w, float dt) {
    float inv = 1.0f / w.cell_size;
    const phys3d__grid& mg = w.moving_grid;
    const phys3d__grid& sg = w.static_grid;
    for (int bk = 0; bk < PHYS3D_GRID_BUCKETS; bk++)
        for (uint32_t i = mg.start[bk]; i < mg.start[bk + 1]; i++)
            for (uint32_t j = i + 1; j < mg.start[bk + 1]; j++) {
                const phys3d__cell_item& p = mg.items[i];
                const phys3d__cell_item& q = mg.items[j];
                if (p.cx != q.cx || p.cy != q.cy || p.cz != q.cz) continue;
                const aabb& ra = w.bodies[p.body].bounds;
                const aabb& rb = w.bodies[q.body].bounds;
                if (!aabb_overlap(ra, rb) || !phys3d__owns_pair(ra, rb, p.cx, p.cy, p.cz, inv)) continue;
                phys3d__narrowphase(w, p.body, q.body, dt);
            }
    for (size_t i = 0; i < w.body_count; i++) {
        const phys3d_body& b = w.bodies[i];
        if (!b.active || b.type == PHYS3D_STATIC || b.oversize) continue;
        int32_t x0 = phys3d__cell(b.bounds.min.x, inv), x1 = phys3d__cell(b.bounds.max.x, inv);
        int32_t y0 = phys3d__cell(b.bounds.min.y, inv), y1 = phys3d__cell(b.bounds.max.y, inv);
        int32_t z0 = phys3d__cell(b.bounds.min.z, inv), z1 = phys3d__cell(b.bounds.max.z, inv);
        for (int32_t z = z0; z <= z1; z++)
            for (int32_t y = y0; y <= y1; y++)
                for (int32_t x = x0; x <= x1; x++) {
                    uint32_t bk = phys3d__bucket(x, y, z);
                    for (uint32_t k = sg.start[bk]; k < sg.start[bk + 1]; k++) {
                        const phys3d__cell_item& it = sg.items[k];
                        if (it.cx != x || it.cy != y || it.cz != z) continue;
                        const aabb& rs = w.bodies[it.body].bounds;
                        if (!aabb_overlap(b.bounds, rs) || !phys3d__owns_pair(b.bounds, rs, x, y, z, inv)) continue;
                        phys3d__narrowphase(w, (idx)i, it.body, dt);
                    }
                }
    }
    // oversize bodies (planes, level meshes) against everything that could touch them
    for (size_t o = 0; o < w.oversize_count; o++) {
        idx big = w.oversize[o];
        const phys3d_body& g = w.bodies[big];
        for (size_t i = 0; i < w.body_count; i++) {
            const phys3d_body& b = w.bodies[i];
            if (!b.active || (idx)i == big) continue;
            if (b.oversize && (idx)i < big) continue;       // that pair is handled from the other side
            if (g.type != PHYS3D_DYNAMIC && b.type == PHYS3D_STATIC) continue;
            phys3d__narrowphase(w, big, (idx)i, dt);
        }
    }
}

// Begin / end events per body pair, however many triangles it touched.
static void phys3d__report(phys3d_world& w) {
    size_t i = 0;
    while (i < w.contact_count) {
        size_t end = i;
        bool touching = false, was = false;
        uint32_t prev = w.contacts[i].prev;
        while (end < w.contact_count && w.contacts[end].a == w.contacts[i].a && w.contacts[end].b == w.contacts[i].b) {
            phys3d_contact& c = w.contacts[end];
            for (int k = 0; k < c.count && !c.trigger; k++) c.touching |= c.points[k].max_normal_impulse > 0.0f;
            touching |= c.touching;
            end++;
        }
        if (prev != 0xFFFFFFFFu) {
            for (uint32_t k = prev; k < w.prev_count && w.prev_contacts[k].a == w.contacts[i].a && w.prev_contacts[k].b == w.contacts[i].b; k++)
                was |= w.prev_contacts[k].touching;
            w.prev_contacts[prev].persisted = touching;
        }
        const phys3d_contact& c = w.contacts[i];
        if (touching && !was) {
            if (c.trigger) {
                bool at = w.bodies[c.a].trigger;
                phys3d__push_event(w, PHYS3D_TRIGGER_ENTER, at ? c.a : c.b, at ? c.b : c.a, c.count ? c.points[0].point : v3(0, 0, 0), v3(0, 0, 0), 0.0f);
            } else {
                float speed = 0.0f;
                vec3 point = v3(0, 0, 0), normal = c.normal;
                for (size_t k = i; k < end; k++)
                    for (int p = 0; p < w.contacts[k].count; p++)
                        if (w.contacts[k].points[p].approach_speed >= speed) {
                            speed = w.contacts[k].points[p].approach_speed;
                            point = w.contacts[k].points[p].point;
                            normal = w.contacts[k].normal;
                        }
                phys3d__push_event(w, PHYS3D_CONTACT_BEGIN, c.a, c.b, point, normal, speed);
            }
        }
        i = end;
    }
    // pairs that touched last step and no longer do (or whose body is gone)
    size_t k = 0;
    while (k < w.prev_count) {
        const phys3d_contact& o = w.prev_contacts[k];
        size_t end = k;
        bool was = false;
        while (end < w.prev_count && w.prev_contacts[end].a == o.a && w.prev_contacts[end].b == o.b) was |= w.prev_contacts[end++].touching;
        if (was && !o.persisted) {
            bool at = w.bodies[o.a].trigger;
            uint8_t type = o.trigger ? PHYS3D_TRIGGER_EXIT : PHYS3D_CONTACT_END;
            idx ea = o.trigger && !at ? o.b : o.a, eb = o.trigger && !at ? o.a : o.b;
            phys3d__push_event(w, type, ea, eb, v3(0, 0, 0), v3(0, 0, 0), 0.0f);
        }
        k = end;
    }
}

// ---- solver (substepped, as in 2D) ----

static void phys3d__prepare_contacts(phys3d_world& w) {
    for (size_t i = 0; i < w.contact_count; i++) {
        phys3d_contact& c = w.contacts[i];
        phys3d_body& a = w.bodies[c.a];
        phys3d_body& b = w.bodies[c.b];
        c.idle = c.trigger || (phys3d__idle(a) && phys3d__idle(b));
        if (c.idle) continue;
        for (int k = 0; k < c.count; k++) {
            phys3d_contact_point& p = c.points[k];
            p.ra = p.point - a.position;
            p.rb = p.point - b.position;
            p.normal_mass = phys3d__row_mass(a, b, c.normal, v3cross(p.ra, c.normal), v3cross(p.rb, c.normal));
            for (int t = 0; t < 2; t++)
                p.tangent_mass[t] = phys3d__row_mass(a, b, c.tangent[t], v3cross(p.ra, c.tangent[t]), v3cross(p.rb, c.tangent[t]));
            vec3 dv = b.velocity + v3cross(b.angular_velocity, p.rb) - a.velocity - v3cross(a.angular_velocity, p.ra);
            p.approach_speed = -v3dot(dv, c.normal);
            p.max_normal_impulse = 0.0f;
        }
        if (c.rolling > 0.0f) {
            mat3 k = phys3d__add(a.inv_inertia, b.inv_inertia);
            float det = mat3_determinant(k);
            if (fabsf(det) > 1e-20f) c.rolling_mass = mat3_inverse(k);
            else c.rolling = 0.0f;
        }
    }
}

static inline vec3 phys3d__point_speed(const phys3d_body& a, const phys3d_body& b, const phys3d_contact_point& p) {
    return b.velocity + v3cross(b.angular_velocity, p.rb) - a.velocity - v3cross(a.angular_velocity, p.ra);
}

static void phys3d__warm_contacts(phys3d_world& w) {
    for (size_t i = 0; i < w.contact_count; i++) {
        phys3d_contact& c = w.contacts[i];
        if (c.idle) continue;
        for (int k = 0; k < c.count; k++) {
            const phys3d_contact_point& p = c.points[k];
            vec3 P = c.normal * p.normal_impulse + c.tangent[0] * p.tangent_impulse[0] + c.tangent[1] * p.tangent_impulse[1];
            phys3d__apply_at(w.bodies[c.a], w.bodies[c.b], p.ra, p.rb, P);
        }
    }
}

static inline float phys3d__separation(const phys3d_body& a, const phys3d_body& b, const phys3d_contact_point& p, vec3 n) {
    vec3 ra = quat_rotate(a.delta_rot, p.ra), rb = quat_rotate(b.delta_rot, p.rb);
    vec3 moved = (b.position - b.prev_position + rb - p.rb) - (a.position - a.prev_position + ra - p.ra);
    return p.separation + v3dot(moved, n);
}

static void phys3d__solve_contacts(phys3d_world& w, float inv_h, float push_rate) {
    for (size_t i = 0; i < w.contact_count; i++) {
        phys3d_contact& c = w.contacts[i];
        if (c.idle) continue;
        phys3d_body& a = w.bodies[c.a];
        phys3d_body& b = w.bodies[c.b];
        if (c.rolling > 0.0f) {
            // stop the relative turning, up to what the contact's grip allows
            float total = 0.0f;
            for (int k = 0; k < c.count; k++) total += c.points[k].normal_impulse;
            float limit = c.rolling * total;
            vec3 old = c.rolling_impulse;
            vec3 acc = old - c.rolling_mass * (b.angular_velocity - a.angular_velocity);
            float len = v3len(acc);
            if (len > limit) acc = acc * (limit / len);
            c.rolling_impulse = acc;
            phys3d__apply_angular(a, b, acc - old);
        }
        // friction first, bounded by the normal impulse from the last pass, as a
        // circle (the true friction cone) rather than two separate limits
        for (int k = 0; k < c.count; k++) {
            phys3d_contact_point& p = c.points[k];
            vec3 dv = phys3d__point_speed(a, b, p);
            float limit = c.friction * p.normal_impulse;
            float t0 = p.tangent_impulse[0] - p.tangent_mass[0] * v3dot(dv, c.tangent[0]);
            float t1 = p.tangent_impulse[1] - p.tangent_mass[1] * v3dot(dv, c.tangent[1]);
            float len = sqrtf(t0 * t0 + t1 * t1);
            if (len > limit) { float s = limit / len; t0 *= s; t1 *= s; }
            vec3 P = c.tangent[0] * (t0 - p.tangent_impulse[0]) + c.tangent[1] * (t1 - p.tangent_impulse[1]);
            p.tangent_impulse[0] = t0; p.tangent_impulse[1] = t1;
            phys3d__apply_at(a, b, p.ra, p.rb, P);
        }
        for (int k = 0; k < c.count; k++) {
            phys3d_contact_point& p = c.points[k];
            float s = phys3d__separation(a, b, p, c.normal);
            p.bias = s > 0.0f ? -s * inv_h : push_rate * maxf(0.0f, -s - w.slop);
        }
        // Two sweeps over the manifold's points. One sweep leaves the first
        // point solved carrying more than its share; the second evens the
        // load out between the corners. Without it a tall stack rocks from
        // corner to corner and creeps sideways (measured: a 12-box stack
        // keeps moving at 0.27 m/s with one sweep, 0.005 m/s with two; more
        // sweeps add nothing). It is a cheap stand-in for solving the points
        // together, which is what the 2D world's block solver does.
        for (int sweep = 0; sweep < 2; sweep++)
        for (int k = 0; k < c.count; k++) {
            phys3d_contact_point& p = c.points[k];
            float vn = v3dot(phys3d__point_speed(a, b, p), c.normal);
            float pn = maxf(p.normal_impulse + p.normal_mass * (p.bias - vn), 0.0f);
            phys3d__apply_at(a, b, p.ra, p.rb, c.normal * (pn - p.normal_impulse));
            p.normal_impulse = pn;
            p.max_normal_impulse = maxf(p.max_normal_impulse, pn);
        }
    }
}

static void phys3d__restitution(phys3d_world& w) {
    for (size_t i = 0; i < w.contact_count; i++) {
        phys3d_contact& c = w.contacts[i];
        if (c.idle || c.restitution <= 0.0f) continue;
        phys3d_body& a = w.bodies[c.a];
        phys3d_body& b = w.bodies[c.b];
        for (int k = 0; k < c.count; k++) {
            phys3d_contact_point& p = c.points[k];
            if (p.approach_speed < w.bounce_threshold || p.max_normal_impulse <= 0.0f) continue;
            float vn = v3dot(phys3d__point_speed(a, b, p), c.normal);
            float pn = maxf(p.normal_impulse - p.normal_mass * (vn - c.restitution * p.approach_speed), 0.0f);
            phys3d__apply_at(a, b, p.ra, p.rb, c.normal * (pn - p.normal_impulse));
            p.normal_impulse = pn;
        }
    }
}

static void phys3d__rebuild_statics(phys3d_world& w) {
    w.oversize_count = 0;
    for (size_t i = 0; i < w.body_count; i++) {
        phys3d_body& b = w.bodies[i];
        if (!b.active) continue;
        b.oversize = phys3d__is_oversize(w, b);
        if (b.oversize) w.oversize[w.oversize_count++] = (idx)i;
    }
    phys3d__build_grid(w, w.static_grid, true);
    w.statics_dirty = false;
}

static aabb phys3d__body_bounds(const phys3d_body& b) {
    if (b.shape.type == PHYS3D_MESH) {
        const aabb& mb = b.shape.mesh->bounds;
        aabb r = aabb_make(v3(1e30f, 1e30f, 1e30f), v3(-1e30f, -1e30f, -1e30f));
        for (int i = 0; i < 8; i++) {
            vec3 c = v3((i & 1) ? mb.max.x : mb.min.x, (i & 2) ? mb.max.y : mb.min.y, (i & 4) ? mb.max.z : mb.min.z);
            vec3 p = b.position + quat_rotate(b.rotation, c);
            r.min = v3min(r.min, p); r.max = v3max(r.max, p);
        }
        return r;
    }
    phys3d__core c;
    phys3d__make_core(b.shape, b.position, b.rotation, c);
    return phys3d__core_bounds(c);
}

static void phys3d__step(phys3d_world& w, float dt) {
    if (dt <= 0.0f) return;
    float glen = v3len(w.gravity);
    vec3 gdir = glen > 0.0f ? w.gravity / glen : v3(0, 0, 0);

    for (size_t i = 0; i < w.body_count; i++) {
        phys3d_body& b = w.bodies[i];
        if (!b.active) continue;
        if (b.sleeping) {
            bool moved = v3dot(b.position - b.prev_position, b.position - b.prev_position) > 0.0f ||
                         b.rotation.x != b.prev_rotation.x || b.rotation.y != b.prev_rotation.y ||
                         b.rotation.z != b.prev_rotation.z || b.rotation.w != b.prev_rotation.w;
            bool pushed = v3dot(b.velocity, b.velocity) > 0.0f || v3dot(b.angular_velocity, b.angular_velocity) > 0.0f ||
                          v3dot(b.force, b.force) > 0.0f || v3dot(b.torque, b.torque) > 0.0f;
            if (moved || pushed) phys3d__wake_body(b);
        }
        b.prev_position = b.position;
        b.prev_rotation = b.rotation;
        b.delta_rot = quat_identity();
        if (!b.sleeping) b.on_ground = false;
        aabb r = phys3d__body_bounds(b);
        if (b.type == PHYS3D_STATIC) {
            if (r.min.x != b.bounds.min.x || r.min.y != b.bounds.min.y || r.min.z != b.bounds.min.z ||
                r.max.x != b.bounds.max.x || r.max.y != b.bounds.max.y || r.max.z != b.bounds.max.z) w.statics_dirty = true;
            b.bounds = r;
            continue;
        }
        vec3 v = b.velocity;
        if (b.type == PHYS3D_DYNAMIC && !b.sleeping) v += w.gravity * (b.gravity_scale * dt);
        vec3 move = v * dt, pad = v3(w.speculative, w.speculative, w.speculative);
        r.min = r.min + v3min(move, v3(0, 0, 0)) - pad;
        r.max = r.max + v3max(move, v3(0, 0, 0)) + pad;
        b.bounds = r;
    }
    if (w.statics_dirty) phys3d__rebuild_statics(w);
    phys3d__build_grid(w, w.moving_grid, false);

    phys3d_contact* t = w.prev_contacts; w.prev_contacts = w.contacts; w.contacts = t;
    size_t tc = w.prev_capacity; w.prev_capacity = w.contact_capacity; w.contact_capacity = tc;
    w.prev_count = w.contact_count;
    w.contact_count = 0;
    size_t need = 16;
    while (need < w.prev_count * 2) need *= 2;
    if (need > w.table_size) {
        pix_free(w.prev_table);
        w.prev_table = pix_alloc<uint32_t>(need);
        w.table_size = need;
    }
    memset(w.prev_table, 0xFF, w.table_size * sizeof(uint32_t));
    for (size_t i = 0; i < w.prev_count; i++) {
        const phys3d_contact& c = w.prev_contacts[i];
        if (i > 0 && w.prev_contacts[i - 1].a == c.a && w.prev_contacts[i - 1].b == c.b) continue;   // first of the pair only
        size_t h = phys3d__hash64(phys3d__pair_key(c.a, c.b)) & (w.table_size - 1);
        while (w.prev_table[h] != 0xFFFFFFFFu) h = (h + 1) & (w.table_size - 1);
        w.prev_table[h] = (uint32_t)i;
    }

    phys3d__find_contacts(w, dt);
    phys3d__islands_wake(w);
    phys3d__prepare_contacts(w);

    int substeps = w.substeps > 0 ? w.substeps : 1;
    float h = dt / substeps, inv_h = 1.0f / h;
    float push_rate = w.correction / dt;
    float max_spin = 0.25f * PIX_PI * inv_h;     // Box2D's cap: a quarter turn per substep
    for (int sub = 0; sub < substeps; sub++) {
        for (size_t i = 0; i < w.body_count; i++) {
            phys3d_body& b = w.bodies[i];
            if (!b.active || b.type != PHYS3D_DYNAMIC || b.sleeping) continue;
            b.velocity += (w.gravity * b.gravity_scale + b.force * b.inv_mass) * h;
            b.angular_velocity += (b.inv_inertia * b.torque) * h;
            b.velocity *= 1.0f / (1.0f + h * b.linear_damping);
            b.angular_velocity *= 1.0f / (1.0f + h * b.angular_damping);
        }
        phys3d__joints_prepare(w);
        phys3d__joints_warm(w);
        phys3d__warm_contacts(w);
        for (int i = 0; i < w.iterations; i++) {
            phys3d__joints_solve(w, h, true);
            phys3d__solve_contacts(w, inv_h, push_rate);
        }
        for (size_t i = 0; i < w.body_count; i++) {
            phys3d_body& b = w.bodies[i];
            if (!b.active || b.type == PHYS3D_STATIC || b.sleeping) continue;
            float spin = v3len(b.angular_velocity);
            if (spin > max_spin) b.angular_velocity *= max_spin / spin;
            b.position += b.velocity * h;
            // q += 0.5 * (w, 0) * q * h
            vec3 wv = b.angular_velocity * (0.5f * h);
            quat dq = { wv.x, wv.y, wv.z, 0.0f };
            quat q = quat_mul(dq, b.rotation);
            b.rotation = quat_norm(quat{ b.rotation.x + q.x, b.rotation.y + q.y, b.rotation.z + q.z, b.rotation.w + q.w });
            b.delta_rot = quat_mul(b.rotation, quat_conjugate(b.prev_rotation));
            phys3d__update_inertia(b);
        }
        for (int i = 0; i < w.relax_iterations; i++) {
            phys3d__joints_solve(w, h, false);
            phys3d__solve_contacts(w, inv_h, 0.0f);
        }
    }
    phys3d__restitution(w);
    phys3d__report(w);
    phys3d__joints_finish(w, h);

    for (size_t i = 0; i < w.contact_count; i++) {
        const phys3d_contact& c = w.contacts[i];
        if (c.trigger || !c.touching || glen == 0.0f) continue;
        float d = v3dot(c.normal, gdir);
        if (d > 0.7f)  { w.bodies[c.a].on_ground = true; w.bodies[c.a].ground_normal = -c.normal; }
        if (d < -0.7f) { w.bodies[c.b].on_ground = true; w.bodies[c.b].ground_normal = c.normal; }
    }
    for (size_t i = 0; i < w.body_count; i++) {
        w.bodies[i].force = v3(0, 0, 0);
        w.bodies[i].torque = v3(0, 0, 0);
    }
    phys3d__islands_sleep(w, dt);

    for (size_t i = 0; i < w.pending_count; i++) w.free_ids[w.free_count++] = w.pending_free[i];
    w.pending_count = 0;
}

void phys3d_step(phys3d_world& w, float dt) {
    w.event_count = 0;
    phys3d__step(w, dt);
    w.alpha = 1.0f;
}

int phys3d_update(phys3d_world& w, float frame_dt) {
    w.event_count = 0;
    w.accumulator += frame_dt;
    int steps = 0;
    while (w.accumulator >= w.fixed_dt && steps < w.max_steps) {
        phys3d__step(w, w.fixed_dt);
        w.accumulator -= w.fixed_dt;
        steps++;
    }
    if (w.accumulator > w.fixed_dt) w.accumulator = w.fixed_dt;
    w.alpha = w.accumulator / w.fixed_dt;
    return steps;
}

// ---- queries ----

static void phys3d__refresh_for_query(phys3d_world& w) {
    if (!w.statics_dirty) return;
    for (size_t i = 0; i < w.body_count; i++) {
        phys3d_body& b = w.bodies[i];
        if (b.active && b.type == PHYS3D_STATIC) b.bounds = phys3d__body_bounds(b);
    }
    phys3d__rebuild_statics(w);
}

// every body whose bounds overlap `r`, each once
template <typename F>
static void phys3d__query_box(phys3d_world& w, const aabb& r, F&& visit) {
    phys3d__refresh_for_query(w);
    float inv = 1.0f / w.cell_size;
    int32_t x0 = phys3d__cell(r.min.x, inv), x1 = phys3d__cell(r.max.x, inv);
    int32_t y0 = phys3d__cell(r.min.y, inv), y1 = phys3d__cell(r.max.y, inv);
    int32_t z0 = phys3d__cell(r.min.z, inv), z1 = phys3d__cell(r.max.z, inv);
    const phys3d__grid* grids[2] = { &w.static_grid, &w.moving_grid };
    for (int g = 0; g < 2; g++) {
        if (!grids[g]->items) continue;
        for (int32_t z = z0; z <= z1; z++)
            for (int32_t y = y0; y <= y1; y++)
                for (int32_t x = x0; x <= x1; x++) {
                    uint32_t bk = phys3d__bucket(x, y, z);
                    for (uint32_t k = grids[g]->start[bk]; k < grids[g]->start[bk + 1]; k++) {
                        const phys3d__cell_item& it = grids[g]->items[k];
                        if (it.cx != x || it.cy != y || it.cz != z) continue;
                        const phys3d_body& b = w.bodies[it.body];
                        if (!b.active || b.oversize || !aabb_overlap(r, b.bounds) || !phys3d__owns_pair(r, b.bounds, x, y, z, inv)) continue;
                        visit(it.body, b);
                    }
                }
    }
    for (size_t o = 0; o < w.oversize_count; o++) {
        const phys3d_body& b = w.bodies[w.oversize[o]];
        if (b.active && aabb_overlap(r, b.bounds)) visit(w.oversize[o], b);
    }
}

// ray against one body; false when it misses or starts inside
static bool phys3d__ray_body(const phys3d_body& b, vec3 o, vec3 d, float max_distance, float* out_t, vec3* out_n, uint32_t* out_tri) {
    const phys3d_shape& s = b.shape;
    *out_tri = 0;
    if (s.type == PHYS3D_SPHERE) {
        if (v3dot(o - b.position, o - b.position) <= s.radius * s.radius) return false;
        if (!ray_vs_sphere(o, d, sphere_make(b.position, s.radius), max_distance, out_t)) return false;
        *out_n = v3norm(o + d * *out_t - b.position);
        return true;
    }
    if (s.type == PHYS3D_CAPSULE) {
        vec3 up = quat_rotate(b.rotation, v3(0, s.half_length, 0));
        capsule c = capsule_make(b.position - up, b.position + up, s.radius);
        vec3 q = closest_point_segment(c.a, c.b, o);
        if (v3dot(o - q, o - q) <= s.radius * s.radius) return false;
        if (!ray_vs_capsule(o, d, c, max_distance, out_t)) return false;
        vec3 p = o + d * *out_t;
        *out_n = v3norm(p - closest_point_segment(c.a, c.b, p));
        return true;
    }
    if (s.type == PHYS3D_BOX) {
        obb box;
        box.centre = b.position; box.half = s.half; box.axes = mat3_from_quat(b.rotation);
        vec3 l = obb_to_local(box, o);
        if (fabsf(l.x) <= s.half.x && fabsf(l.y) <= s.half.y && fabsf(l.z) <= s.half.z) return false;
        if (!ray_vs_obb(o, d, box, max_distance, out_t)) return false;
        vec3 lp = obb_to_local(box, o + d * *out_t);
        vec3 r = v3(fabsf(lp.x) / s.half.x, fabsf(lp.y) / s.half.y, fabsf(lp.z) / s.half.z);
        int ax = r.x > r.y ? (r.x > r.z ? 0 : 2) : (r.y > r.z ? 1 : 2);
        vec3 ln = v3(0, 0, 0);
        (&ln.x)[ax] = (&lp.x)[ax] >= 0.0f ? 1.0f : -1.0f;
        *out_n = mat3_col(box.axes, 0) * ln.x + mat3_col(box.axes, 1) * ln.y + mat3_col(box.axes, 2) * ln.z;
        return true;
    }
    if (s.type == PHYS3D_PLANE) {
        vec3 n = quat_rotate(b.rotation, s.normal);
        if (!ray_vs_plane(o, d, plane_through(n, b.position), max_distance, false, out_t)) return false;
        if (v3dot(o - b.position, n) < 0.0f) return false;
        *out_n = n;
        return true;
    }
    if (s.type == PHYS3D_MESH) {
        // into the mesh's frame, where its tree lives
        const phys3d_mesh& m = *s.mesh;
        quat inv = quat_conjugate(b.rotation);
        vec3 lo = quat_rotate(inv, o - b.position), ld = quat_rotate(inv, d);
        vec3 inv_d = v3(ld.x != 0.0f ? 1.0f / ld.x : 1e30f, ld.y != 0.0f ? 1.0f / ld.y : 1e30f, ld.z != 0.0f ? 1.0f / ld.z : 1e30f);
        float best = max_distance;
        bool hit = false;
        uint32_t stack[64];
        int top = 0;
        stack[top++] = 0;
        while (top) {
            const phys3d__bvh_node& n = m.nodes[stack[--top]];
            float tn;
            if (!ray_vs_aabb(lo, inv_d, n.bounds, best, &tn)) continue;
            if (n.count) {
                for (uint32_t i = 0; i < n.count; i++) {
                    uint32_t t = m.order[n.first + i];
                    vec3 a = m.vertices[m.indices[t * 3]], bb = m.vertices[m.indices[t * 3 + 1]], c = m.vertices[m.indices[t * 3 + 2]];
                    float tt;
                    if (ray_vs_triangle(lo, ld, a, bb, c, best, false, &tt)) {
                        best = tt; hit = true; *out_tri = t;
                        *out_n = quat_rotate(b.rotation, triangle_normal(a, bb, c));
                    }
                }
            } else if (top < 62) {
                stack[top++] = n.first;
                stack[top++] = n.first + 1;
            }
        }
        *out_t = best;
        return hit;
    }
    return false;
}

// Walks the grid cells along the ray in order (Amanatides & Woo), testing the
// bodies in each, and stops once a hit is nearer than the next cell.
bool phys3d_raycast(phys3d_world& w, vec3 origin, vec3 direction, float max_distance,
                    phys3d_ray_hit* hit, uint16_t mask, idx ignore) {
    float len = v3len(direction);
    if (len <= 0.0f || max_distance <= 0.0f) return false;
    phys3d__refresh_for_query(w);
    vec3 d = direction / len;
    float best = max_distance;
    bool found = false;
    auto test = [&](idx id) {
        const phys3d_body& b = w.bodies[id];
        if (!b.active || id == ignore || b.trigger || !(b.layer & mask)) return;
        float t; vec3 n; uint32_t tri;
        if (!phys3d__ray_body(b, origin, d, best, &t, &n, &tri) || t > best) return;
        best = t;
        found = true;
        if (hit) { hit->body = id; hit->distance = t; hit->point = origin + d * t; hit->normal = n; hit->triangle = tri; }
    };
    for (size_t o = 0; o < w.oversize_count; o++) test(w.oversize[o]);

    float cs = w.cell_size, inv = 1.0f / cs;
    int32_t c[3] = { phys3d__cell(origin.x, inv), phys3d__cell(origin.y, inv), phys3d__cell(origin.z, inv) };
    float o3[3] = { origin.x, origin.y, origin.z }, d3[3] = { d.x, d.y, d.z };
    int32_t step[3];
    float t_max[3], t_delta[3];
    for (int k = 0; k < 3; k++) {
        if (d3[k] > 0.0f)      { step[k] = 1;  t_max[k] = ((c[k] + 1) * cs - o3[k]) / d3[k]; t_delta[k] = cs / d3[k]; }
        else if (d3[k] < 0.0f) { step[k] = -1; t_max[k] = (c[k] * cs - o3[k]) / d3[k];       t_delta[k] = -cs / d3[k]; }
        else                   { step[k] = 0;  t_max[k] = 1e30f; t_delta[k] = 1e30f; }
    }
    float t_enter = 0.0f;
    // a body in many cells is tested many times; harmless, and cheaper than tracking
    for (int guard = 0; guard < 100000 && t_enter <= best; guard++) {
        const phys3d__grid* grids[2] = { &w.static_grid, &w.moving_grid };
        uint32_t bk = phys3d__bucket(c[0], c[1], c[2]);
        for (int g = 0; g < 2; g++) {
            if (!grids[g]->items) continue;
            for (uint32_t k = grids[g]->start[bk]; k < grids[g]->start[bk + 1]; k++) {
                const phys3d__cell_item& it = grids[g]->items[k];
                if (it.cx == c[0] && it.cy == c[1] && it.cz == c[2]) test(it.body);
            }
        }
        int ax = t_max[0] < t_max[1] ? (t_max[0] < t_max[2] ? 0 : 2) : (t_max[1] < t_max[2] ? 1 : 2);
        t_enter = t_max[ax];
        if (t_enter > max_distance) break;
        c[ax] += step[ax];
        t_max[ax] += t_delta[ax];
    }
    return found;
}

// Conservative advancement on GJK distances, as the 2D shape cast does: a
// translating shape cannot close a gap faster than it moves, so stepping by
// the gap never passes the first touch.
bool phys3d_shape_cast(phys3d_world& w, phys3d_shape shape, vec3 position, quat rotation,
                       vec3 direction, float max_distance, phys3d_ray_hit* hit, uint16_t mask, idx ignore) {
    float len = v3len(direction);
    if (len <= 0.0f || shape.type == PHYS3D_PLANE || shape.type == PHYS3D_MESH) return false;
    vec3 d = direction / len;
    phys3d__core start;
    phys3d__make_core(shape, position, quat_norm(rotation), start);
    aabb r0 = phys3d__core_bounds(start);
    vec3 move = d * max_distance;
    aabb r = aabb_make(r0.min + v3min(move, v3(0, 0, 0)), r0.max + v3max(move, v3(0, 0, 0)));
    float best = max_distance, tolerance = 0.25f * w.slop;
    bool found = false;
    auto cast_core = [&](const phys3d__core& target, idx id, uint32_t tri) {
        float t = 0.0f;
        for (int iter = 0; iter < 32; iter++) {
            phys3d__core moved = start;
            moved.centre += d * t;
            for (int k = 0; k < moved.count; k++) moved.v[k] += d * t;
            vec3 pa, pb, n = -d;
            float gap;
            if (target.kind == PHYS3D__PLANE) {
                float lowest = 1e30f;
                for (int k = 0; k < moved.count; k++) lowest = minf(lowest, v3dot(target.axis[0], moved.v[k]) - target.offset);
                gap = lowest - moved.radius;
                n = target.axis[0];
                pb = moved.centre - n * (gap + moved.radius);
            } else {
                float cd = phys3d__gjk(moved, target, &pa, &pb);
                gap = cd - moved.radius - target.radius;
                if (cd > 1e-6f) n = (pa - pb) / cd;
            }
            if (gap <= tolerance) {
                if (t <= best) {
                    best = t; found = true;
                    if (hit) { hit->body = id; hit->distance = t; hit->normal = n; hit->point = pb + n * target.radius; hit->triangle = tri; }
                }
                return;
            }
            t += gap;
            if (t > best) return;
        }
    };
    phys3d__query_box(w, r, [&](idx id, const phys3d_body& b) {
        if (id == ignore || b.trigger || !(b.layer & mask)) return;
        if (b.shape.type == PHYS3D_MESH) {
            const phys3d_mesh& m = *b.shape.mesh;
            quat inv = quat_conjugate(b.rotation);
            aabb box = aabb_make(v3(1e30f, 1e30f, 1e30f), v3(-1e30f, -1e30f, -1e30f));
            for (int i = 0; i < 8; i++) {
                vec3 corner = v3((i & 1) ? r.max.x : r.min.x, (i & 2) ? r.max.y : r.min.y, (i & 4) ? r.max.z : r.min.z);
                vec3 l = quat_rotate(inv, corner - b.position);
                box.min = v3min(box.min, l); box.max = v3max(box.max, l);
            }
            phys3d__mesh_query(m, box, [&](uint32_t t) {
                phys3d__core tri;
                phys3d__triangle_core(b.position + quat_rotate(b.rotation, m.vertices[m.indices[t * 3]]),
                                      b.position + quat_rotate(b.rotation, m.vertices[m.indices[t * 3 + 1]]),
                                      b.position + quat_rotate(b.rotation, m.vertices[m.indices[t * 3 + 2]]), tri);
                if (v3dot(start.centre - tri.v[0], tri.axis[0]) < 0.0f) return;   // one-sided
                cast_core(tri, id, t);
            });
            return;
        }
        phys3d__core target;
        phys3d__make_core(b.shape, b.position, b.rotation, target);
        cast_core(target, id, 0);
    });
    return found;
}

size_t phys3d_overlap(phys3d_world& w, phys3d_shape shape, vec3 position, quat rotation,
                      idx* out, size_t capacity, uint16_t mask) {
    if (shape.type == PHYS3D_PLANE || shape.type == PHYS3D_MESH) return 0;
    phys3d__core q;
    phys3d__make_core(shape, position, quat_norm(rotation), q);
    size_t count = 0;
    phys3d_contact m;
    auto inside = [&]() {
        for (int k = 0; k < m.count; k++) if (m.points[k].separation < 0.0f) return true;
        return false;
    };
    phys3d__query_box(w, phys3d__core_bounds(q), [&](idx id, const phys3d_body& b) {
        if (!(b.layer & mask)) return;
        bool hit = false;
        if (b.shape.type == PHYS3D_MESH) {
            const phys3d_mesh& mesh = *b.shape.mesh;
            quat inv = quat_conjugate(b.rotation);
            aabb qb = phys3d__core_bounds(q);
            aabb box = aabb_make(v3(1e30f, 1e30f, 1e30f), v3(-1e30f, -1e30f, -1e30f));
            for (int i = 0; i < 8; i++) {
                vec3 corner = v3((i & 1) ? qb.max.x : qb.min.x, (i & 2) ? qb.max.y : qb.min.y, (i & 4) ? qb.max.z : qb.min.z);
                vec3 l = quat_rotate(inv, corner - b.position);
                box.min = v3min(box.min, l); box.max = v3max(box.max, l);
            }
            phys3d__mesh_query(mesh, box, [&](uint32_t t) {
                if (hit) return;
                phys3d__core tri;
                phys3d__triangle_core(b.position + quat_rotate(b.rotation, mesh.vertices[mesh.indices[t * 3]]),
                                      b.position + quat_rotate(b.rotation, mesh.vertices[mesh.indices[t * 3 + 1]]),
                                      b.position + quat_rotate(b.rotation, mesh.vertices[mesh.indices[t * 3 + 2]]), tri);
                hit = phys3d__collide(q, tri, w.slop, 0.0f, m) && inside();
            });
        } else {
            phys3d__core c;
            phys3d__make_core(b.shape, b.position, b.rotation, c);
            hit = phys3d__collide(q, c, w.slop, 0.0f, m) && inside();
        }
        if (!hit) return;
        if (count < capacity) out[count] = id;
        count++;
    });
    return count;
}

#endif // PIX_PHYSICS3D_IMPL
