#ifndef PIX_PHYSICS2D_HPP
#define PIX_PHYSICS2D_HPP
// dtype.hpp first: it sets up the CRT before any C header
#include "dtype.hpp"
#include <string.h>
#include <math.h>
#include "math.hpp"
#include "memory.hpp"
#include "colliders.hpp"
#include "physics_common.hpp"

// A 2D rigid-body world for side-on and top-down games, in the coordinates the
// 2D drawing uses: x right, y down, units are whatever you draw in (the
// defaults assume pixels). Bodies are circles, axis-aligned boxes, rotated
// boxes and capsules; they fall, slide, bounce, spin, stack and rest.
//
//   phys2d_world w;
//   phys2d_create_world(w);
//   phys2d_add(w, PHYS2D_STATIC,  phys2d_box(800, 40), v2(400, 600));   // floor
//   idx crate = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(32, 32), v2(400, 100));
//   while (pix_frame()) {
//       phys2d_update(w, pix_dt());          // fixed 1/60 s steps under the hood
//       phys2d_body* b = phys2d_get_body(w, crate);
//       pix_draw_physics2d(w);               // see every shape and contact
//   }
//
// How it works - the same design as Box2D, so its literature applies:
//
//   broadphase   a hashed uniform grid. Statics live in their own grid that is
//                rebuilt only when one of them changes; moving bodies are
//                re-bucketed every step. A pair is reported by exactly one
//                cell - the one holding the min corner of the two boxes'
//                overlap - so no duplicate filtering is needed.
//   narrowphase  every shape is a convex "core" (a point, a segment or a box)
//                plus a radius. One routine collides any two of them: SAT
//                when the cores overlap, closest points when they do not, and
//                reference-face clipping for a two-point manifold whenever two
//                flat sides meet - which is what lets boxes stack.
//   solver       sequential impulses with accumulated, clamped impulses and
//                warm starting (impulses carried over from the last step by
//                contact feature id), Coulomb friction, Baumgarte position
//                correction and restitution applied after the solve. Each
//                step runs as `substeps` short substeps with a relax pass
//                (Box2D v3), and the two points of a face contact are solved
//                together (Box2D v2's block solver).
//   tunnelling   speculative contacts: a pair is collided while its gap is
//                smaller than how far the two close in one step, and the
//                solver only lets them close that gap. A fast ball stops on a
//                thin wall instead of passing through it.
//   time         phys2d_update runs fixed steps and leaves `alpha` for drawing
//                between the last two (phys2d_lerp_position / _angle).
//   joints       distance (rod, spring, rope), revolute (hinge, with limits
//                and a motor), prismatic (slider), weld and mouse, as soft
//                constraints in the same impulse loop (Box2D v3's soft step:
//                a constraint is a damped spring of chosen stiffness, and a
//                "rigid" one is just a very stiff spring).
//   sleeping     bodies joined by touching contacts or joints form islands
//                (union-find, rebuilt each step). An island whose bodies have
//                all been nearly still for `time_to_sleep` stops simulating
//                until something moving touches it.
//
// Layers: a pair collides only when each accepts the other -
// phys_layers_collide in physics_common.hpp, the same rule in every world.
//
// Not here yet: threading, polygon shapes beyond boxes.

#ifndef PHYS2D_MAX_BODIES
#define PHYS2D_MAX_BODIES 4096      // capacity; the array is heap allocated
#endif
#ifndef PHYS2D_MAX_JOINTS
#define PHYS2D_MAX_JOINTS 4096
#endif
#define PHYS2D_GRID_BUCKETS 4096    // hashed cells per grid (power of two)

// body types
#define PHYS2D_STATIC    0   // never moves on its own: floors, walls
#define PHYS2D_DYNAMIC   1   // moved by gravity, forces and collisions
#define PHYS2D_KINEMATIC 2   // moved only by the velocity you give it; pushes dynamics, nothing pushes it

// shapes
#define PHYS2D_CIRCLE  0
#define PHYS2D_AABB    1     // a box that never rotates; `angle` is ignored
#define PHYS2D_BOX     2     // a box that rotates with the body
#define PHYS2D_CAPSULE 3     // upright: a vertical segment with round ends

// event types
#define PHYS2D_CONTACT_BEGIN 0   // two solid bodies started touching
#define PHYS2D_CONTACT_END   1
#define PHYS2D_TRIGGER_ENTER 2   // a body entered a trigger (a = trigger, b = visitor)
#define PHYS2D_TRIGGER_EXIT  3
#define PHYS2D_JOINT_BREAK   4   // a joint pulled harder than its break_force and was removed (see `joint`)

// joint types
#define PHYS2D_JOINT_DISTANCE  0   // keeps two points a set distance apart: rod, spring or rope
#define PHYS2D_JOINT_REVOLUTE  1   // a shared pin the bodies rotate about: hinges, wheels, ragdoll joints
#define PHYS2D_JOINT_PRISMATIC 2   // b slides along an axis fixed in a: pistons, elevators, sliding doors
#define PHYS2D_JOINT_WELD      3   // glues two bodies together (springy if hertz > 0)
#define PHYS2D_JOINT_MOUSE     4   // drags a point of a body toward `target`: picking things up with the mouse

// debug draw flags
#define PHYS2D_DRAW_SHAPES   1
#define PHYS2D_DRAW_AABBS    2
#define PHYS2D_DRAW_CONTACTS 4
#define PHYS2D_DRAW_GRID     8
#define PHYS2D_DRAW_JOINTS   16
#define PHYS2D_DRAW_DEFAULT  (PHYS2D_DRAW_SHAPES | PHYS2D_DRAW_CONTACTS | PHYS2D_DRAW_JOINTS)

struct phys2d_shape {
    uint8_t type;          // PHYS2D_CIRCLE ...
    float   radius;        // circle, capsule
    vec2    half;          // box / aabb half extents
    float   half_length;   // capsule: half the straight part, along local y
};

// shape constructors: sizes are full widths and heights
static inline phys2d_shape phys2d_circle(float radius) {
    phys2d_shape s = {}; s.type = PHYS2D_CIRCLE; s.radius = radius; return s;
}
static inline phys2d_shape phys2d_aabb(float width, float height) {
    phys2d_shape s = {}; s.type = PHYS2D_AABB; s.half = v2(width * 0.5f, height * 0.5f); return s;
}
static inline phys2d_shape phys2d_box(float width, float height) {
    phys2d_shape s = {}; s.type = PHYS2D_BOX; s.half = v2(width * 0.5f, height * 0.5f); return s;
}
// `height` overall, caps included; under 2 * radius it is just a circle
static inline phys2d_shape phys2d_capsule(float radius, float height) {
    phys2d_shape s = {}; s.type = PHYS2D_CAPSULE; s.radius = radius;
    s.half_length = maxf(0.0f, height * 0.5f - radius);
    return s;
}

struct phys2d_body {
    phys2d_shape shape;
    vec2  position;           // centre
    float angle;              // radians, clockwise on screen (y down); * PIX_RAD2DEG to draw
    vec2  velocity;
    float angular_velocity;   // radians per second

    uint8_t type;             // PHYS2D_STATIC / DYNAMIC / KINEMATIC
    bool  trigger;            // reports overlaps, never pushes or gets pushed
    bool  fixed_rotation;     // characters: collide like a capsule, never tip over
    float density;            // mass = density * area
    float friction;           // 0 ice .. 1 rubber; a pair uses sqrt(a * b)
    float restitution;        // 0 dead stop .. 1 perfect bounce; a pair uses the larger
    float linear_damping;     // per second, like air drag
    float angular_damping;
    float gravity_scale;      // 1 normal, 0 floats, -1 falls up

    uint16_t layer;           // which PHYS_LAYER_* bits this body is on
    uint16_t mask;            // which layers it collides with
    void*    user;            // your own pointer, handed back in events
    bool     never_sleep;     // keep simulating even when still

    // ---- written by the world ----
    bool  active;
    bool  sleeping;           // still for a while, not simulated; touching it or setting velocity wakes it
    float sleep_time;         // how long it has been nearly still
    idx   joint_list;         // first joint attached to this body
    bool  on_ground;          // touching something below it (relative to gravity) this step
    vec2  ground_normal;      // that surface's normal, pointing up out of it
    vec2  force;              // accumulated by phys2d_apply_force, cleared each step
    float torque;
    float inv_mass, inv_inertia;
    vec2  prev_position;      // where the body was one step ago, for interpolation
    float prev_angle;
    rect2 bounds;             // broadphase box (fattened by motion for moving bodies)
    vec2  delta_rot;          // cos / sin of how far it has turned this step (solver)
};

struct phys2d_contact_point {
    vec2     point;               // world, halfway between the two surfaces
    float    separation;          // negative = overlapping
    float    normal_impulse;      // accumulated this step; warm-starts the next
    float    tangent_impulse;
    float    max_normal_impulse;
    float    approach_speed;      // closing speed before the solve
    uint32_t id;                  // which features made this point (for warm starting)
    vec2     ra, rb;              // solver: anchors relative to each centre
    float    normal_mass, tangent_mass, bias;
};

struct phys2d_contact {
    idx   a, b;                   // a < b; normal points from a to b
    vec2  normal;
    int   count;                  // 0..2 points
    phys2d_contact_point points[2];
    float friction, restitution;
    float k11, k12, k22;          // solver: two-point effective mass, and its inverse
    float i11, i12, i22;
    bool  block;                  // solve both points together
    bool  idle;                   // both bodies asleep or fixed: kept, not solved
    bool  trigger;
    bool  touching;               // overlapping, or took an impulse: not just a near miss
    bool  persisted;              // set on last step's copy when the pair still touches (end events)
    uint32_t prev;                // this pair in last step's contacts, or 0xFFFFFFFF
};

struct phys2d_event {
    uint8_t type;                 // PHYS2D_CONTACT_BEGIN ...
    idx     a, b;                 // for triggers, a is the trigger; PIX_INVALID = the world
    idx     joint;                // PHYS2D_JOINT_BREAK: the joint (already removed)
    vec2    point, normal;        // normal from a to b; zero for triggers and ends
    float   speed;                // closing speed at begin: use it to pick an impact sound
};

// One struct for every joint type; each type reads the fields it needs.
// Build one with phys2d_add_revolute and friends, then edit it through
// phys2d_get_joint - limits, motors and springs are all off until turned on.
struct phys2d_joint {
    uint8_t type;                 // PHYS2D_JOINT_*
    bool    active;
    bool    collide_connected;    // false: the two bodies pass through each other
    idx     a, b;                 // PIX_INVALID = fixed to the world
    vec2    local_a, local_b;     // the anchor in each body's own frame
    vec2    local_axis;           // prismatic: the slide direction, in a's frame
    float   reference_angle;      // b's angle minus a's when joined

    // limits: revolute angle (radians), prismatic travel, distance min / max length
    bool  enable_limit;
    float lower, upper;
    // motors: revolute speed (rad/s) and torque, prismatic speed and force
    bool  enable_motor;
    float motor_speed, max_motor_force;
    // distance: `length`; with enable_spring it is a spring of `hertz`
    // (hertz 0 = slack: a rope held only by the limits)
    bool  enable_spring;
    float length;
    // stiffness for springs, weld and mouse: oscillations per second, and how
    // quickly they die out (1 = no overshoot). hertz 0 on a weld = rigid
    float hertz, damping_ratio;
    vec2  target;                 // mouse: where to drag the anchor
    float max_force;              // mouse: how hard it may pull
    float break_force;            // 0 = unbreakable; more force than this and the joint snaps

    // ---- written by the world ----
    float reaction;               // force the joint applied last step
    idx   next_a, next_b;         // next joint in a's / b's list
    vec2  ra, rb, d, axis;        // solver scratch
    vec2  linear_impulse;
    float angular_impulse, axial_impulse, motor_impulse, lower_impulse, upper_impulse;
    float axial_mass, angular_mass, a1, a2, s1, s2;
    bool  idle;
};

struct phys2d_ray_hit {
    idx   body;
    vec2  point;
    vec2  normal;                 // surface normal at the hit, facing the ray
    float distance;
};

// a shape in world space: a convex core (point, segment or box) plus a radius
struct phys2d__poly {
    vec2  v[4];
    vec2  n[4];      // n[i] is the outward normal of edge v[i] -> v[i+1]
    int   count;     // 1 point, 2 segment, 4 box
    float radius;
};

struct phys2d__cell_item { int32_t cx, cy; idx body; };
struct phys2d__grid {
    uint32_t*          start;     // PHYS2D_GRID_BUCKETS + 1 prefix offsets
    phys2d__cell_item* items;
    size_t             count, capacity;
};

struct phys2d_world {
    // ---- settings, edit freely ----
    vec2  gravity;                // (0, 980): pixels per second squared, down
    float fixed_dt;               // 1/60
    int   max_steps;              // per phys2d_update; any more time is dropped (no spiral of death)
    int   substeps;               // solver substeps per step, 4; more = stiffer chains and stacks
    int   iterations;             // solver passes per substep, 1
    int   relax_iterations;       // passes without position correction per substep, 1
    float cell_size;              // broadphase cell, 64; about the size of a typical body
    float slop;                   // allowed overlap that is not corrected, 0.5
    float speculative;            // contact margin beyond motion, 4
    float bounce_threshold;       // slower impacts than this do not bounce, 30
    float correction;             // fraction of overlap removed per step, 0.2
    float joint_hertz;            // stiffness of "rigid" joints, 60, with joint_damping 2
    float joint_damping;
    bool  allow_sleep;            // true
    float sleep_linear;           // slower than this (units/s) counts as still, 8
    float sleep_angular;          // radians/s, 0.05
    float time_to_sleep;          // seconds still before an island sleeps, 0.5

    // ---- state ----
    float  accumulator;
    float  alpha;                 // after phys2d_update: 0..1 between the last two steps

    phys2d_body* bodies;          // PHYS2D_MAX_BODIES
    phys2d__poly* polys;          // each body's world shape, built once per step
    size_t       body_count;      // high-water mark, not the live count
    idx*         free_ids;
    size_t       free_count;
    idx*         pending_free;    // removed this frame, reusable after the next step
    size_t       pending_count;

    phys2d_joint* joints;         // PHYS2D_MAX_JOINTS
    size_t        joint_count;    // high-water mark
    idx*          free_joints;
    size_t        free_joint_count;
    phys2d_body   ground;         // what PIX_INVALID means in a joint: a static body at the origin

    uint32_t* island;             // union-find parent per body
    float*    island_sleep;       // per island root: the shortest sleep_time in it
    uint8_t*  island_awake;

    phys2d__grid static_grid, moving_grid;
    bool         statics_dirty;

    phys2d_contact* contacts;     // this step's pairs
    size_t          contact_count, contact_capacity;
    phys2d_contact* prev_contacts;
    size_t          prev_count, prev_capacity;
    uint32_t*       prev_table;   // open-addressed pair -> prev index
    size_t          table_size;

    phys2d_event* events;         // since the last phys2d_update / phys2d_step
    size_t        event_count, event_capacity;
};

// ---- world ----
PIXDEF void phys2d_create_world(phys2d_world& w);
PIXDEF void phys2d_destroy_world(phys2d_world& w);
// Runs as many fixed steps as `frame_dt` covers and returns how many ran.
// Events from all of them are in w.events afterwards.
PIXDEF int  phys2d_update(phys2d_world& w, float frame_dt);
// One step of exactly `dt`, for code that runs its own clock. Clears events.
PIXDEF void phys2d_step(phys2d_world& w, float dt);

// ---- bodies ----
// A body with sensible defaults, to edit before phys2d_add_body.
PIXDEF phys2d_body phys2d_make_body(uint8_t type, phys2d_shape shape, vec2 position);
PIXDEF idx  phys2d_add_body(phys2d_world& w, const phys2d_body& body);    // PIX_INVALID when full
PIXDEF idx  phys2d_add(phys2d_world& w, uint8_t type, phys2d_shape shape, vec2 position);
PIXDEF phys2d_body* phys2d_get_body(phys2d_world& w, idx id);             // nullptr once removed
PIXDEF void phys2d_remove_body(phys2d_world& w, idx id);
// after changing a body's shape, density, type or fixed_rotation
PIXDEF void phys2d_update_mass(phys2d_world& w, idx id);
// move without the interpolation smearing it across the screen
PIXDEF void phys2d_teleport(phys2d_world& w, idx id, vec2 position, float angle);

// wake a sleeping body (setting its velocity or moving it does this too)
PIXDEF void phys2d_wake(phys2d_world& w, idx id);

PIXDEF void phys2d_apply_force(phys2d_body& b, vec2 force, vec2 world_point);
PIXDEF void phys2d_apply_impulse(phys2d_body& b, vec2 impulse, vec2 world_point);
static inline void phys2d_apply_force_centre(phys2d_body& b, vec2 force) { b.force += force; }
static inline void phys2d_apply_impulse_centre(phys2d_body& b, vec2 impulse) { b.velocity += impulse * b.inv_mass; }

// ---- joints ----
// `a` or `b` may be PIX_INVALID to pin a body to the world. Anchors and axes
// are given in world space, where the bodies are right now.
PIXDEF idx phys2d_add_revolute(phys2d_world& w, idx a, idx b, vec2 anchor);
PIXDEF idx phys2d_add_weld(phys2d_world& w, idx a, idx b, vec2 anchor);
PIXDEF idx phys2d_add_prismatic(phys2d_world& w, idx a, idx b, vec2 anchor, vec2 axis);
// a rigid rod at the current distance; set enable_spring (+ hertz) for a
// spring, or enable_spring + hertz 0 + enable_limit for a rope
PIXDEF idx phys2d_add_distance(phys2d_world& w, idx a, idx b, vec2 anchor_a, vec2 anchor_b);
// drags `body`'s point under `grab` toward j->target; max_force 0 picks a
// pull strong enough to lift the body easily
PIXDEF idx phys2d_add_mouse(phys2d_world& w, idx body, vec2 grab, float max_force = 0.0f);
// any joint filled in by hand (anchors in local space); PIX_INVALID when full
PIXDEF idx phys2d_add_joint(phys2d_world& w, const phys2d_joint& joint);
PIXDEF phys2d_joint* phys2d_get_joint(phys2d_world& w, idx id);      // nullptr once removed
PIXDEF void phys2d_remove_joint(phys2d_world& w, idx id);
// world positions of a joint's two anchors
PIXDEF void phys2d_joint_anchors(const phys2d_world& w, const phys2d_joint& j, vec2* a, vec2* b);

// where to draw a body this frame: between its last two steps by w.alpha
static inline vec2  phys2d_lerp_position(const phys2d_world& w, const phys2d_body& b) {
    return v2lerp(b.prev_position, b.position, w.alpha);
}
static inline float phys2d_lerp_angle(const phys2d_world& w, const phys2d_body& b) {
    return b.prev_angle + (b.angle - b.prev_angle) * w.alpha;
}

// ---- queries ----
// Closest body along a ray. Skips triggers and bodies the ray starts inside.
PIXDEF bool phys2d_raycast(phys2d_world& w, vec2 origin, vec2 direction, float max_distance,
                           phys2d_ray_hit* hit, uint16_t mask = PHYS_LAYER_ALL, idx ignore = PIX_INVALID);
// Sweeps `shape` from `position` along `direction` (no rotation) and reports
// the first solid body it would touch; distance 0 if it already overlaps one.
PIXDEF bool phys2d_shape_cast(phys2d_world& w, phys2d_shape shape, vec2 position, float angle,
                              vec2 direction, float max_distance, phys2d_ray_hit* hit,
                              uint16_t mask = PHYS_LAYER_ALL, idx ignore = PIX_INVALID);
// Every body (triggers included) overlapping `shape`; returns how many, writes up to `capacity`.
PIXDEF size_t phys2d_overlap(phys2d_world& w, phys2d_shape shape, vec2 position, float angle,
                             idx* out, size_t capacity, uint16_t mask = PHYS_LAYER_ALL);
static inline size_t phys2d_overlap_point(phys2d_world& w, vec2 point, idx* out, size_t capacity,
                                          uint16_t mask = PHYS_LAYER_ALL) {
    return phys2d_overlap(w, phys2d_circle(0.0f), point, 0.0f, out, capacity, mask);
}

// debug draw: phys2d_debug_draw in physics_debug.hpp (it needs the 2D batch)

#endif // PIX_PHYSICS2D_HPP


#if defined(PIX_IMPLEMENTATION) && !defined(PIX_PHYSICS2D_IMPL)
#define PIX_PHYSICS2D_IMPL

static inline float phys2d__cross(vec2 a, vec2 b) { return a.x * b.y - a.y * b.x; }
static inline vec2  phys2d__cross_sv(float s, vec2 v) { return v2(-s * v.y, s * v.x); }
static inline vec2  phys2d__perp(vec2 e) { return v2(e.y, -e.x); }   // outward for our winding

// ---- shapes as core + radius ----

static void phys2d__make_poly(const phys2d_shape& s, vec2 p, float angle, phys2d__poly& out) {
    if (s.type == PHYS2D_AABB) angle = 0.0f;
    float c = cosf(angle), sn = sinf(angle);
    out.radius = 0.0f;
    if (s.type == PHYS2D_CIRCLE || (s.type == PHYS2D_CAPSULE && s.half_length <= 0.0f)) {
        out.count = 1; out.v[0] = p; out.radius = s.radius;
        return;
    }
    if (s.type == PHYS2D_CAPSULE) {
        vec2 axis = v2(-sn * s.half_length, c * s.half_length);   // local (0, L) rotated
        out.count = 2; out.radius = s.radius;
        out.v[0] = p - axis; out.v[1] = p + axis;
        out.n[0] = v2norm(phys2d__perp(out.v[1] - out.v[0]));
        out.n[1] = -out.n[0];
        return;
    }
    static const float sx[4] = { -1, 1, 1, -1 }, sy[4] = { -1, -1, 1, 1 };
    static const float nx[4] = { 0, 1, 0, -1 },  ny[4] = { -1, 0, 1, 0 };
    out.count = 4;
    for (int i = 0; i < 4; i++) {
        float lx = sx[i] * s.half.x, ly = sy[i] * s.half.y;
        out.v[i] = v2(p.x + c * lx - sn * ly, p.y + sn * lx + c * ly);
        out.n[i] = v2(c * nx[i] - sn * ny[i], sn * nx[i] + c * ny[i]);
    }
}

static rect2 phys2d__poly_bounds(const phys2d__poly& p) {
    rect2 r = { p.v[0], p.v[0] };
    for (int i = 1; i < p.count; i++) {
        r.min = v2(minf(r.min.x, p.v[i].x), minf(r.min.y, p.v[i].y));
        r.max = v2(maxf(r.max.x, p.v[i].x), maxf(r.max.y, p.v[i].y));
    }
    r.min = r.min - v2(p.radius, p.radius);
    r.max = r.max + v2(p.radius, p.radius);
    return r;
}

static inline bool phys2d__rects_overlap(const rect2& a, const rect2& b) {
    return a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y && b.min.y <= a.max.y;
}

// gap between the two cores' projections on `axis`; positive = separated
static float phys2d__axis_gap(const phys2d__poly& a, const phys2d__poly& b, vec2 axis) {
    float a0 = 1e30f, a1 = -1e30f, b0 = 1e30f, b1 = -1e30f;
    for (int i = 0; i < a.count; i++) { float d = v2dot(a.v[i], axis); a0 = minf(a0, d); a1 = maxf(a1, d); }
    for (int i = 0; i < b.count; i++) { float d = v2dot(b.v[i], axis); b0 = minf(b0, d); b1 = maxf(b1, d); }
    return maxf(b0 - a1, a0 - b1);
}

// Do the cores not touch? Face normals of both, plus each segment's own
// direction - a segment is a degenerate polygon, and two collinear segments
// end to end are only separated along that line.
static bool phys2d__cores_disjoint(const phys2d__poly& a, const phys2d__poly& b) {
    if (a.count == 1 && b.count == 1) return a.v[0].x != b.v[0].x || a.v[0].y != b.v[0].y;
    const phys2d__poly* ps[2] = { &a, &b };
    for (int k = 0; k < 2; k++) {
        const phys2d__poly& p = *ps[k];
        if (p.count == 2) {
            if (phys2d__axis_gap(a, b, p.n[0]) > 0.0f) return true;
            if (phys2d__axis_gap(a, b, v2norm(p.v[1] - p.v[0])) > 0.0f) return true;
        } else if (p.count == 4) {
            if (phys2d__axis_gap(a, b, p.n[0]) > 0.0f) return true;
            if (phys2d__axis_gap(a, b, p.n[1]) > 0.0f) return true;
        }
    }
    return false;
}

static inline vec2 phys2d__closest_on_segment(vec2 a, vec2 b, vec2 p) {
    vec2 ab = b - a;
    float len2 = v2dot(ab, ab);
    if (len2 <= 1e-12f) return a;
    return a + ab * clampf(v2dot(p - a, ab) / len2, 0.0f, 1.0f);
}

// Closest points between two disjoint cores. With at most four vertices a
// side, testing every vertex against every edge is cheaper than GJK.
static float phys2d__core_distance(const phys2d__poly& a, const phys2d__poly& b, vec2* pa, vec2* pb) {
    float best = 1e30f;
    for (int pass = 0; pass < 2; pass++) {
        const phys2d__poly& p = pass ? b : a;    // vertices from p
        const phys2d__poly& q = pass ? a : b;    // edges from q
        int edges = q.count == 4 ? 4 : 1;
        for (int i = 0; i < p.count; i++)
            for (int e = 0; e < edges; e++) {
                vec2 c = phys2d__closest_on_segment(q.v[e], q.v[q.count == 1 ? 0 : (e + 1) % q.count], p.v[i]);
                vec2 d = c - p.v[i];
                float d2 = v2dot(d, d);
                if (d2 < best) {
                    best = d2;
                    if (pass) { *pa = c; *pb = p.v[i]; } else { *pa = p.v[i]; *pb = c; }
                }
            }
    }
    return sqrtf(best);
}

// Keeps the part of segment p1..p2 where dot(dir, p) <= offset.
static bool phys2d__clip_segment(vec2& p1, vec2& p2, vec2 dir, float offset) {
    float d1 = v2dot(dir, p1) - offset, d2 = v2dot(dir, p2) - offset;
    if (d1 > 0.0f && d2 > 0.0f) return false;
    if (d1 > 0.0f)      p1 = p1 + (p2 - p1) * (d1 / (d1 - d2));
    else if (d2 > 0.0f) p2 = p2 + (p1 - p2) * (d2 / (d2 - d1));
    return true;
}

// Reference-face clipping: the incident shape's most opposing edge is clipped
// to the side planes of reference edge `edge`, and every clipped end within
// `margin` of the face becomes a contact point. `flip` means the reference is
// shape b, so the A->B normal is the face normal negated.
static int phys2d__clip(const phys2d__poly& ref, int edge, const phys2d__poly& inc, bool flip,
                        float margin, phys2d_contact& m) {
    vec2 v1 = ref.v[edge], v2p = ref.v[(edge + 1) % ref.count], n = ref.n[edge];
    int j = 0;
    if (inc.count >= 2) {
        float lowest = 1e30f;
        for (int k = 0; k < inc.count; k++) {
            float d = v2dot(inc.n[k], n);
            if (d < lowest) { lowest = d; j = k; }
        }
    }
    vec2 w1 = inc.v[j], w2 = inc.count >= 2 ? inc.v[(j + 1) % inc.count] : w1;
    vec2 t = v2norm(v2p - v1);
    if (!phys2d__clip_segment(w1, w2, -t, -v2dot(t, v1))) return 0;
    if (!phys2d__clip_segment(w1, w2, t, v2dot(t, v2p))) return 0;

    vec2 pts[2] = { w1, w2 };
    int np = inc.count >= 2 ? 2 : 1;
    m.normal = flip ? -n : n;
    m.count = 0;
    for (int k = 0; k < np; k++) {
        float sep = v2dot(pts[k] - v1, n) - ref.radius - inc.radius;
        if (sep > margin) continue;
        phys2d_contact_point& cp = m.points[m.count++];
        memset(&cp, 0, sizeof(cp));
        cp.point = pts[k] - n * (inc.radius + 0.5f * sep);
        cp.separation = sep;
        cp.id = (uint32_t)edge | ((uint32_t)j << 4) | ((uint32_t)k << 8) | ((uint32_t)flip << 12);
    }
    return m.count;
}

// The one narrowphase routine. Fills m.normal (A to B) and up to two points
// closer than `margin`; returns the point count.
static int phys2d__collide(const phys2d__poly& a, const phys2d__poly& b, float slop, float margin,
                           phys2d_contact& m) {
    m.count = 0;
    float radius = a.radius + b.radius;

    // a single point when the closest features are a corner or an end cap
    auto closest_point = [&](vec2 pa, vec2 pb, float d) {
        phys2d_contact_point& cp = m.points[0];
        memset(&cp, 0, sizeof(cp));
        m.normal = d > 1e-6f ? (pb - pa) / d : v2(0.0f, -1.0f);
        m.count = 1;
        cp.separation = d - radius;
        cp.point = (pa + m.normal * a.radius + pb - m.normal * b.radius) * 0.5f;
        cp.id = 0xFFFF;
        return 1;
    };
    if (a.count == 1 && b.count == 1) {
        float d = v2dist(a.v[0], b.v[0]);
        return d - radius > margin ? 0 : closest_point(a.v[0], b.v[0], d);
    }

    // SAT over both shapes' faces: sep_* is the largest gap along one
    // shape's face normals (negative when overlapping).
    float sep_a = -1e30f, sep_b = -1e30f;
    int edge_a = -1, edge_b = -1;
    for (int i = 0; a.count >= 2 && i < a.count; i++) {
        float s = 1e30f;
        for (int j = 0; j < b.count; j++) s = minf(s, v2dot(a.n[i], b.v[j] - a.v[i]));
        if (s > sep_a) { sep_a = s; edge_a = i; }
    }
    for (int i = 0; b.count >= 2 && i < b.count; i++) {
        float s = 1e30f;
        for (int j = 0; j < a.count; j++) s = minf(s, v2dot(b.n[i], a.v[j] - b.v[i]));
        if (s > sep_b) { sep_b = s; edge_b = i; }
    }
    // The reference face. a's wins unless b's is clearly better: without that
    // bias two resting boxes trade the role every step, the feature ids change
    // and warm starting is lost - a stack would sway instead of settling.
    bool flip = edge_b >= 0 && (edge_a < 0 || sep_b > sep_a + 0.1f * slop);
    float face_sep = flip ? sep_b : sep_a;
    if (face_sep - radius > margin) return 0;   // a face gap never exceeds the true distance

    if (phys2d__cores_disjoint(a, b)) {
        vec2 pa, pb;
        float d = phys2d__core_distance(a, b, &pa, &pb);
        if (d - radius > margin) return 0;
        // the face is the closest feature only if its gap is the distance;
        // otherwise this is a corner or end cap and gets one point
        if (face_sep < d - 0.1f * slop) return closest_point(pa, pb, d);
        if (flip ? phys2d__clip(b, edge_b, a, true, margin, m) : phys2d__clip(a, edge_a, b, false, margin, m))
            return m.count;
        return closest_point(pa, pb, d);
    }
    return flip ? phys2d__clip(b, edge_b, a, true, margin, m) : phys2d__clip(a, edge_a, b, false, margin, m);
}

// ---- mass ----

static void phys2d__compute_mass(phys2d_body& b) {
    b.inv_mass = 0.0f;
    b.inv_inertia = 0.0f;
    if (b.type != PHYS2D_DYNAMIC) return;
    float d = b.density > 0.0f ? b.density : 1.0f;
    float mass, inertia;
    const phys2d_shape& s = b.shape;
    if (s.type == PHYS2D_CIRCLE) {
        mass = d * PIX_PI * s.radius * s.radius;
        inertia = 0.5f * mass * s.radius * s.radius;
    } else if (s.type == PHYS2D_CAPSULE) {
        // Box2D's formula: the rectangle plus two half discs pushed out to the ends
        float r = s.radius, h = s.half_length, rr = r * r;
        float box_mass = d * 2.0f * r * 2.0f * h, circle_mass = d * PIX_PI * rr;
        float lc = 4.0f * r / (3.0f * PIX_PI);
        mass = box_mass + circle_mass;
        inertia = circle_mass * (0.5f * rr + h * h + 2.0f * h * lc) + box_mass * (4.0f * rr + 4.0f * h * h) / 12.0f;
    } else {
        mass = d * 4.0f * s.half.x * s.half.y;
        inertia = mass * (s.half.x * s.half.x + s.half.y * s.half.y) / 3.0f;
    }
    if (mass > 0.0f) b.inv_mass = 1.0f / mass;
    if (inertia > 0.0f && !b.fixed_rotation && s.type != PHYS2D_AABB) b.inv_inertia = 1.0f / inertia;
}

// ---- world ----

void phys2d_create_world(phys2d_world& w) {
    memset(&w, 0, sizeof(w));
    w.gravity = v2(0.0f, 980.0f);
    w.fixed_dt = 1.0f / 60.0f;
    w.max_steps = 4;
    w.substeps = 4;
    w.iterations = 1;
    w.relax_iterations = 1;
    w.cell_size = 64.0f;
    w.slop = 0.5f;
    w.speculative = 4.0f;
    w.bounce_threshold = 30.0f;
    w.correction = 0.2f;
    w.joint_hertz = 60.0f;
    w.joint_damping = 2.0f;
    w.allow_sleep = true;
    w.sleep_linear = 8.0f;
    w.sleep_angular = 0.05f;
    w.time_to_sleep = 0.5f;
    w.ground.type = PHYS2D_STATIC;
    w.ground.joint_list = PIX_INVALID;
    w.joints       = pix_alloc<phys2d_joint>(PHYS2D_MAX_JOINTS);
    w.free_joints  = pix_alloc<idx>(PHYS2D_MAX_JOINTS);
    w.island       = pix_alloc<uint32_t>(PHYS2D_MAX_BODIES);
    w.island_sleep = pix_alloc<float>(PHYS2D_MAX_BODIES);
    w.island_awake = pix_alloc<uint8_t>(PHYS2D_MAX_BODIES);
    w.bodies       = pix_alloc<phys2d_body>(PHYS2D_MAX_BODIES);
    w.polys        = pix_alloc<phys2d__poly>(PHYS2D_MAX_BODIES);
    w.free_ids     = pix_alloc<idx>(PHYS2D_MAX_BODIES);
    w.pending_free = pix_alloc<idx>(PHYS2D_MAX_BODIES);
    w.static_grid.start = pix_alloc<uint32_t>(PHYS2D_GRID_BUCKETS + 1);
    w.moving_grid.start = pix_alloc<uint32_t>(PHYS2D_GRID_BUCKETS + 1);
}

void phys2d_destroy_world(phys2d_world& w) {
    pix_free(w.joints);
    pix_free(w.free_joints);
    pix_free(w.island);
    pix_free(w.island_sleep);
    pix_free(w.island_awake);
    pix_free(w.bodies);
    pix_free(w.polys);
    pix_free(w.free_ids);
    pix_free(w.pending_free);
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

// grows a heap array to at least `need` elements, keeping its contents
template <typename T>
static void phys2d__reserve(T*& p, size_t& capacity, size_t need) {
    if (need <= capacity) return;
    size_t cap = capacity ? capacity : 64;
    while (cap < need) cap *= 2;
    p = (T*)pix_mem_realloc(p, cap * sizeof(T));
    capacity = cap;
}

phys2d_body phys2d_make_body(uint8_t type, phys2d_shape shape, vec2 position) {
    phys2d_body b;
    memset(&b, 0, sizeof(b));
    b.type = type;
    b.shape = shape;
    b.position = position;
    b.density = 1.0f;
    b.friction = 0.6f;
    b.gravity_scale = 1.0f;
    b.layer = PHYS_LAYER_0;
    b.mask = PHYS_LAYER_ALL;
    return b;
}

idx phys2d_add_body(phys2d_world& w, const phys2d_body& body) {
    idx id;
    if (w.free_count) id = w.free_ids[--w.free_count];
    else if (w.body_count < PHYS2D_MAX_BODIES) id = (idx)w.body_count++;
    else { pix_log("phys2d: body limit (%d) reached\n", PHYS2D_MAX_BODIES); return PIX_INVALID; }
    phys2d_body& b = w.bodies[id];
    b = body;
    b.active = true;
    b.sleeping = false;
    b.sleep_time = 0.0f;
    b.joint_list = PIX_INVALID;
    b.prev_position = b.position;
    b.prev_angle = b.angle;
    phys2d__compute_mass(b);
    if (b.type == PHYS2D_STATIC) w.statics_dirty = true;
    return id;
}

idx phys2d_add(phys2d_world& w, uint8_t type, phys2d_shape shape, vec2 position) {
    return phys2d_add_body(w, phys2d_make_body(type, shape, position));
}

phys2d_body* phys2d_get_body(phys2d_world& w, idx id) {
    return id < w.body_count && w.bodies[id].active ? &w.bodies[id] : 0;
}

// The slot is only reused after the next step, so that step can still report
// the end of every contact the body had.
static inline void phys2d__wake_body(phys2d_body& b) { b.sleeping = false; b.sleep_time = 0.0f; }

void phys2d_remove_body(phys2d_world& w, idx id) {
    phys2d_body* b = phys2d_get_body(w, id);
    if (!b) return;
    if (b->type == PHYS2D_STATIC) w.statics_dirty = true;
    while (b->joint_list != PIX_INVALID) phys2d_remove_joint(w, b->joint_list);
    // whatever was asleep on top of it has to notice that it is gone
    for (size_t i = 0; i < w.contact_count; i++) {
        const phys2d_contact& c = w.contacts[i];
        if (c.a == id) phys2d__wake_body(w.bodies[c.b]);
        if (c.b == id) phys2d__wake_body(w.bodies[c.a]);
    }
    b->active = false;
    w.pending_free[w.pending_count++] = id;
}

void phys2d_wake(phys2d_world& w, idx id) {
    phys2d_body* b = phys2d_get_body(w, id);
    if (b) phys2d__wake_body(*b);
}

void phys2d_update_mass(phys2d_world& w, idx id) {
    phys2d_body* b = phys2d_get_body(w, id);
    if (!b) return;
    phys2d__compute_mass(*b);
    w.statics_dirty = true;    // the type may have moved in or out of the static grid
}

void phys2d_teleport(phys2d_world& w, idx id, vec2 position, float angle) {
    phys2d_body* b = phys2d_get_body(w, id);
    if (!b) return;
    b->position = b->prev_position = position;
    b->angle = b->prev_angle = angle;
    phys2d__wake_body(*b);
    if (b->type == PHYS2D_STATIC) w.statics_dirty = true;
}

void phys2d_apply_force(phys2d_body& b, vec2 force, vec2 world_point) {
    phys2d__wake_body(b);
    b.force += force;
    b.torque += phys2d__cross(world_point - b.position, force);
}

void phys2d_apply_impulse(phys2d_body& b, vec2 impulse, vec2 world_point) {
    phys2d__wake_body(b);
    b.velocity += impulse * b.inv_mass;
    b.angular_velocity += b.inv_inertia * phys2d__cross(world_point - b.position, impulse);
}

// ---- joints ----

static inline vec2 phys2d__rot(vec2 v, float angle) {
    float c = cosf(angle), sn = sinf(angle);
    return v2(c * v.x - sn * v.y, sn * v.x + c * v.y);
}
static inline vec2 phys2d__unrot(vec2 v, float angle) {
    float c = cosf(angle), sn = sinf(angle);
    return v2(c * v.x + sn * v.y, -sn * v.x + c * v.y);
}
static inline phys2d_body& phys2d__jbody(phys2d_world& w, idx id) { return id == PIX_INVALID ? w.ground : w.bodies[id]; }
static inline const phys2d_body& phys2d__jbody(const phys2d_world& w, idx id) { return id == PIX_INVALID ? w.ground : w.bodies[id]; }

static inline bool phys2d__idle(const phys2d_body& b) {
    return b.type == PHYS2D_STATIC || b.sleeping ||
           (b.type == PHYS2D_KINEMATIC && b.velocity.x == 0.0f && b.velocity.y == 0.0f && b.angular_velocity == 0.0f);
}

idx phys2d_add_joint(phys2d_world& w, const phys2d_joint& joint) {
    if (joint.a == joint.b) { pix_log("phys2d: a joint needs two different bodies\n"); return PIX_INVALID; }
    if ((joint.a != PIX_INVALID && !phys2d_get_body(w, joint.a)) || (joint.b != PIX_INVALID && !phys2d_get_body(w, joint.b))) {
        pix_log("phys2d: joint on a removed body\n");
        return PIX_INVALID;
    }
    idx id;
    if (w.free_joint_count) id = w.free_joints[--w.free_joint_count];
    else if (w.joint_count < PHYS2D_MAX_JOINTS) id = (idx)w.joint_count++;
    else { pix_log("phys2d: joint limit (%d) reached\n", PHYS2D_MAX_JOINTS); return PIX_INVALID; }
    phys2d_joint& j = w.joints[id];
    j = joint;
    j.active = true;
    j.linear_impulse = v2(0.0f, 0.0f);
    j.angular_impulse = j.axial_impulse = j.motor_impulse = j.lower_impulse = j.upper_impulse = 0.0f;
    j.next_a = j.next_b = PIX_INVALID;
    // each body keeps a list of its joints, threaded through the joints themselves
    if (j.a != PIX_INVALID) { j.next_a = w.bodies[j.a].joint_list; w.bodies[j.a].joint_list = id; phys2d__wake_body(w.bodies[j.a]); }
    if (j.b != PIX_INVALID) { j.next_b = w.bodies[j.b].joint_list; w.bodies[j.b].joint_list = id; phys2d__wake_body(w.bodies[j.b]); }
    return id;
}

static phys2d_joint phys2d__joint_base(phys2d_world& w, uint8_t type, idx a, idx b, vec2 anchor_a, vec2 anchor_b) {
    phys2d_joint j;
    memset(&j, 0, sizeof(j));
    j.type = type;
    j.a = a; j.b = b;
    const phys2d_body& A = phys2d__jbody(w, a);
    const phys2d_body& B = phys2d__jbody(w, b);
    j.local_a = phys2d__unrot(anchor_a - A.position, A.angle);
    j.local_b = phys2d__unrot(anchor_b - B.position, B.angle);
    j.reference_angle = B.angle - A.angle;
    j.damping_ratio = 1.0f;
    return j;
}

idx phys2d_add_revolute(phys2d_world& w, idx a, idx b, vec2 anchor) {
    return phys2d_add_joint(w, phys2d__joint_base(w, PHYS2D_JOINT_REVOLUTE, a, b, anchor, anchor));
}

idx phys2d_add_weld(phys2d_world& w, idx a, idx b, vec2 anchor) {
    return phys2d_add_joint(w, phys2d__joint_base(w, PHYS2D_JOINT_WELD, a, b, anchor, anchor));
}

idx phys2d_add_prismatic(phys2d_world& w, idx a, idx b, vec2 anchor, vec2 axis) {
    phys2d_joint j = phys2d__joint_base(w, PHYS2D_JOINT_PRISMATIC, a, b, anchor, anchor);
    j.local_axis = phys2d__unrot(v2norm(axis), phys2d__jbody(w, a).angle);
    return phys2d_add_joint(w, j);
}

idx phys2d_add_distance(phys2d_world& w, idx a, idx b, vec2 anchor_a, vec2 anchor_b) {
    phys2d_joint j = phys2d__joint_base(w, PHYS2D_JOINT_DISTANCE, a, b, anchor_a, anchor_b);
    j.length = maxf(v2dist(anchor_a, anchor_b), 1e-3f);
    j.lower = 0.0f;
    j.upper = j.length;
    j.damping_ratio = 0.5f;
    return phys2d_add_joint(w, j);
}

idx phys2d_add_mouse(phys2d_world& w, idx body, vec2 grab, float max_force) {
    phys2d_joint j = phys2d__joint_base(w, PHYS2D_JOINT_MOUSE, PIX_INVALID, body, grab, grab);
    j.target = grab;
    j.hertz = 5.0f;
    j.damping_ratio = 0.7f;
    const phys2d_body* b = phys2d_get_body(w, body);
    // strong enough to lift the body briskly, whatever the units
    float mass = b && b->inv_mass > 0.0f ? 1.0f / b->inv_mass : 1.0f;
    j.max_force = max_force > 0.0f ? max_force : 50.0f * mass * maxf(v2len(w.gravity), 100.0f);
    return phys2d_add_joint(w, j);
}

phys2d_joint* phys2d_get_joint(phys2d_world& w, idx id) {
    return id < w.joint_count && w.joints[id].active ? &w.joints[id] : 0;
}

static void phys2d__unlink_joint(phys2d_world& w, idx body, idx id) {
    idx* link = &w.bodies[body].joint_list;
    while (*link != PIX_INVALID) {
        phys2d_joint& j = w.joints[*link];
        idx* next = j.a == body ? &j.next_a : &j.next_b;
        if (*link == id) { *link = *next; return; }
        link = next;
    }
}

void phys2d_remove_joint(phys2d_world& w, idx id) {
    phys2d_joint* j = phys2d_get_joint(w, id);
    if (!j) return;
    if (j->a != PIX_INVALID) { phys2d__unlink_joint(w, j->a, id); phys2d__wake_body(w.bodies[j->a]); }
    if (j->b != PIX_INVALID) { phys2d__unlink_joint(w, j->b, id); phys2d__wake_body(w.bodies[j->b]); }
    j->active = false;
    w.free_joints[w.free_joint_count++] = id;
}

void phys2d_joint_anchors(const phys2d_world& w, const phys2d_joint& j, vec2* a, vec2* b) {
    const phys2d_body& A = phys2d__jbody(w, j.a);
    const phys2d_body& B = phys2d__jbody(w, j.b);
    *a = j.type == PHYS2D_JOINT_MOUSE ? j.target : A.position + phys2d__rot(j.local_a, A.angle);
    *b = B.position + phys2d__rot(j.local_b, B.angle);
}

// do a and b share a joint that turns their collision off?
static bool phys2d__joined(const phys2d_world& w, idx a, idx b) {
    for (idx id = w.bodies[a].joint_list; id != PIX_INVALID;) {
        const phys2d_joint& j = w.joints[id];
        idx other = j.a == a ? j.b : j.a;
        if (other == b && !j.collide_connected) return true;
        id = j.a == a ? j.next_a : j.next_b;
    }
    return false;
}

// Soft constraints (Catto, "Solver2D" / Box2D v3): a constraint behaves as a
// spring of `hertz` with damping ratio `zeta`, solved implicitly so it stays
// stable at any stiffness. bias_rate turns position error into a target
// velocity; mass_scale and impulse_scale make the impulse spring-like. hertz 0
// means "no spring at all" - no error correction, full impulse.
struct phys2d__soft { float bias_rate, mass_scale, impulse_scale; };

static phys2d__soft phys2d__make_soft(float hertz, float zeta, float h) {
    phys2d__soft s = { 0.0f, 1.0f, 0.0f };
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

static inline void phys2d__japply(phys2d_body& A, phys2d_body& B, vec2 ra, vec2 rb, vec2 P) {
    A.velocity -= P * A.inv_mass;
    A.angular_velocity -= A.inv_inertia * phys2d__cross(ra, P);
    B.velocity += P * B.inv_mass;
    B.angular_velocity += B.inv_inertia * phys2d__cross(rb, P);
}
static inline void phys2d__japply_angular(phys2d_body& A, phys2d_body& B, float L) {
    A.angular_velocity -= A.inv_inertia * L;
    B.angular_velocity += B.inv_inertia * L;
}
// an impulse along `axis` whose lever arms are a1 (on A) and a2 (on B)
static inline void phys2d__japply_axial(phys2d_body& A, phys2d_body& B, vec2 axis, float a1, float a2, float imp) {
    A.velocity -= axis * (imp * A.inv_mass);
    A.angular_velocity -= A.inv_inertia * imp * a1;
    B.velocity += axis * (imp * B.inv_mass);
    B.angular_velocity += B.inv_inertia * imp * a2;
}

// the 2D point-to-point constraint: anchors ra / rb together, error C
static vec2 phys2d__point_impulse(const phys2d_body& A, const phys2d_body& B, vec2 ra, vec2 rb, vec2 C,
                                  phys2d__soft s, vec2 accumulated) {
    vec2 cdot = B.velocity + phys2d__cross_sv(B.angular_velocity, rb) - A.velocity - phys2d__cross_sv(A.angular_velocity, ra);
    float mA = A.inv_mass, mB = B.inv_mass, iA = A.inv_inertia, iB = B.inv_inertia;
    float k11 = mA + mB + iA * ra.y * ra.y + iB * rb.y * rb.y;
    float k12 = -iA * ra.x * ra.y - iB * rb.x * rb.y;
    float k22 = mA + mB + iA * ra.x * ra.x + iB * rb.x * rb.x;
    float det = k11 * k22 - k12 * k12;
    det = det != 0.0f ? 1.0f / det : 0.0f;
    vec2 rhs = cdot + C * s.bias_rate;
    vec2 x = v2(det * (k22 * rhs.x - k12 * rhs.y), det * (k11 * rhs.y - k12 * rhs.x));
    return -x * s.mass_scale - accumulated * s.impulse_scale;
}

// one side of a limit: C >= 0 is allowed. Ahead of the limit it is
// speculative (may close the gap, no more); past it, a stiff spring pushes back.
static float phys2d__limit(float C, float cdot, float mass, float inv_dt, phys2d__soft rigid, float* accumulated) {
    float bias = 0.0f, ms = 1.0f, is = 0.0f;
    if (C > 0.0f) bias = C * inv_dt;
    else { bias = rigid.bias_rate * C; ms = rigid.mass_scale; is = rigid.impulse_scale; }
    float imp = -mass * ms * (cdot + bias) - is * *accumulated;
    float total = maxf(*accumulated + imp, 0.0f);
    imp = total - *accumulated;
    *accumulated = total;
    return imp;
}

// Joint geometry from where the bodies are now: anchors, axes, effective
// masses. Runs at the start of every substep, so the constraints always see
// the current pose.
static void phys2d__joints_prepare(phys2d_world& w) {
    for (size_t i = 0; i < w.joint_count; i++) {
        phys2d_joint& j = w.joints[i];
        if (!j.active) continue;
        phys2d_body& A = phys2d__jbody(w, j.a);
        phys2d_body& B = phys2d__jbody(w, j.b);
        j.idle = phys2d__idle(A) && phys2d__idle(B);
        if (j.idle) continue;
        float mA = A.inv_mass, mB = B.inv_mass, iA = A.inv_inertia, iB = B.inv_inertia;
        j.ra = phys2d__rot(j.local_a, A.angle);
        j.rb = phys2d__rot(j.local_b, B.angle);
        j.d = (B.position + j.rb) - (A.position + j.ra);
        j.angular_mass = iA + iB > 0.0f ? 1.0f / (iA + iB) : 0.0f;
        if (j.type == PHYS2D_JOINT_PRISMATIC) {
            j.axis = phys2d__rot(j.local_axis, A.angle);
            vec2 perp = v2(-j.axis.y, j.axis.x);
            j.a1 = phys2d__cross(j.d + j.ra, j.axis); j.a2 = phys2d__cross(j.rb, j.axis);
            j.s1 = phys2d__cross(j.d + j.ra, perp);   j.s2 = phys2d__cross(j.rb, perp);
        } else if (j.type == PHYS2D_JOINT_DISTANCE) {
            float len = v2len(j.d);
            j.axis = len > 1e-6f ? j.d / len : v2(1.0f, 0.0f);
            j.a1 = phys2d__cross(j.ra, j.axis); j.a2 = phys2d__cross(j.rb, j.axis);
        }
        if (j.type == PHYS2D_JOINT_PRISMATIC || j.type == PHYS2D_JOINT_DISTANCE) {
            float k = mA + mB + iA * j.a1 * j.a1 + iB * j.a2 * j.a2;
            j.axial_mass = k > 0.0f ? 1.0f / k : 0.0f;
        }
    }
}

// re-apply last substep's impulses before solving this one
static void phys2d__joints_warm(phys2d_world& w) {
    for (size_t i = 0; i < w.joint_count; i++) {
        phys2d_joint& j = w.joints[i];
        if (!j.active || j.idle) continue;
        phys2d_body& A = phys2d__jbody(w, j.a);
        phys2d_body& B = phys2d__jbody(w, j.b);
        float axial = j.axial_impulse + j.motor_impulse + j.lower_impulse - j.upper_impulse;
        if (j.type == PHYS2D_JOINT_PRISMATIC) {
            vec2 perp = v2(-j.axis.y, j.axis.x);
            phys2d__japply_axial(A, B, j.axis, j.a1, j.a2, axial);
            phys2d__japply_axial(A, B, perp, j.s1, j.s2, j.linear_impulse.x);
            phys2d__japply_angular(A, B, j.linear_impulse.y);
        } else if (j.type == PHYS2D_JOINT_DISTANCE) {
            phys2d__japply_axial(A, B, j.axis, j.a1, j.a2, axial);
        } else {
            phys2d__japply(A, B, j.ra, j.rb, j.linear_impulse);
            phys2d__japply_angular(A, B, j.angular_impulse + j.motor_impulse + j.lower_impulse - j.upper_impulse);
        }
    }
}

// One pass over the joints for a substep of length h. With use_bias the
// rigid parts also pull position error back out; without it (the relax pass)
// they only hold velocities, so the correction never becomes a bounce.
// User springs (distance spring, soft weld, mouse) act in both passes.
static void phys2d__joints_solve(phys2d_world& w, float h, bool use_bias) {
    float inv_h = 1.0f / h;
    phys2d__soft rigid = { 0.0f, 1.0f, 0.0f };
    if (use_bias) rigid = phys2d__make_soft(w.joint_hertz, w.joint_damping, h);
    for (size_t i = 0; i < w.joint_count; i++) {
        phys2d_joint& j = w.joints[i];
        if (!j.active || j.idle) continue;
        phys2d_body& A = phys2d__jbody(w, j.a);
        phys2d_body& B = phys2d__jbody(w, j.b);
        float iA = A.inv_inertia, iB = B.inv_inertia;
        float angle = B.angle - A.angle - j.reference_angle;

        if (j.type == PHYS2D_JOINT_REVOLUTE) {
            if (j.enable_motor && j.angular_mass > 0.0f) {
                float cdot = B.angular_velocity - A.angular_velocity - j.motor_speed;
                float old = j.motor_impulse, max = j.max_motor_force * h;
                j.motor_impulse = clampf(old - j.angular_mass * cdot, -max, max);
                phys2d__japply_angular(A, B, j.motor_impulse - old);
            }
            if (j.enable_limit && j.angular_mass > 0.0f) {
                float imp = phys2d__limit(angle - j.lower, B.angular_velocity - A.angular_velocity, j.angular_mass, inv_h, rigid, &j.lower_impulse);
                phys2d__japply_angular(A, B, imp);
                imp = phys2d__limit(j.upper - angle, A.angular_velocity - B.angular_velocity, j.angular_mass, inv_h, rigid, &j.upper_impulse);
                phys2d__japply_angular(A, B, -imp);
            }
            vec2 imp = phys2d__point_impulse(A, B, j.ra, j.rb, j.d, rigid, j.linear_impulse);
            j.linear_impulse += imp;
            phys2d__japply(A, B, j.ra, j.rb, imp);
        } else if (j.type == PHYS2D_JOINT_WELD) {
            phys2d__soft soft = j.hertz > 0.0f ? phys2d__make_soft(j.hertz, j.damping_ratio, h) : rigid;
            float cdot = B.angular_velocity - A.angular_velocity;
            float imp = -j.angular_mass * soft.mass_scale * (cdot + soft.bias_rate * angle) - soft.impulse_scale * j.angular_impulse;
            j.angular_impulse += imp;
            phys2d__japply_angular(A, B, imp);
            vec2 li = phys2d__point_impulse(A, B, j.ra, j.rb, j.d, soft, j.linear_impulse);
            j.linear_impulse += li;
            phys2d__japply(A, B, j.ra, j.rb, li);
        } else if (j.type == PHYS2D_JOINT_MOUSE) {
            phys2d__soft soft = phys2d__make_soft(j.hertz, j.damping_ratio, h);
            vec2 C = B.position + j.rb - j.target;
            vec2 imp = phys2d__point_impulse(A, B, j.ra, j.rb, C, soft, j.linear_impulse);
            vec2 old = j.linear_impulse;
            j.linear_impulse += imp;
            float max = j.max_force * h, len = v2len(j.linear_impulse);
            if (len > max) j.linear_impulse *= max / len;
            phys2d__japply(A, B, j.ra, j.rb, j.linear_impulse - old);
        } else if (j.type == PHYS2D_JOINT_PRISMATIC) {
            vec2 axis = j.axis, perp = v2(-axis.y, axis.x);
            auto axial_speed = [&]() {
                return v2dot(axis, B.velocity - A.velocity) + j.a2 * B.angular_velocity - j.a1 * A.angular_velocity;
            };
            if (j.enable_motor) {
                float old = j.motor_impulse, max = j.max_motor_force * h;
                j.motor_impulse = clampf(old + j.axial_mass * (j.motor_speed - axial_speed()), -max, max);
                phys2d__japply_axial(A, B, axis, j.a1, j.a2, j.motor_impulse - old);
            }
            if (j.enable_limit) {
                float travel = v2dot(axis, j.d);
                float imp = phys2d__limit(travel - j.lower, axial_speed(), j.axial_mass, inv_h, rigid, &j.lower_impulse);
                phys2d__japply_axial(A, B, axis, j.a1, j.a2, imp);
                imp = phys2d__limit(j.upper - travel, -axial_speed(), j.axial_mass, inv_h, rigid, &j.upper_impulse);
                phys2d__japply_axial(A, B, axis, j.a1, j.a2, -imp);
            }
            // across the axis and in rotation: solved together as a 2x2
            float mA = A.inv_mass, mB = B.inv_mass;
            float cdot1 = v2dot(perp, B.velocity - A.velocity) + j.s2 * B.angular_velocity - j.s1 * A.angular_velocity;
            float cdot2 = B.angular_velocity - A.angular_velocity;
            float k11 = mA + mB + iA * j.s1 * j.s1 + iB * j.s2 * j.s2;
            float k12 = iA * j.s1 + iB * j.s2;
            float k22 = iA + iB;
            if (k22 == 0.0f) k22 = 1.0f;    // neither can rotate: the angular row is free
            float det = k11 * k22 - k12 * k12;
            det = det != 0.0f ? 1.0f / det : 0.0f;
            vec2 rhs = v2(cdot1 + rigid.bias_rate * v2dot(perp, j.d), cdot2 + rigid.bias_rate * angle);
            vec2 x = v2(det * (k22 * rhs.x - k12 * rhs.y), det * (k11 * rhs.y - k12 * rhs.x));
            vec2 imp = -x * rigid.mass_scale - j.linear_impulse * rigid.impulse_scale;
            j.linear_impulse += imp;
            phys2d__japply_axial(A, B, perp, j.s1, j.s2, imp.x);
            phys2d__japply_angular(A, B, imp.y);
        } else if (j.type == PHYS2D_JOINT_DISTANCE) {
            vec2 axis = j.axis;
            auto axial_speed = [&]() {
                vec2 dv = B.velocity + phys2d__cross_sv(B.angular_velocity, j.rb) - A.velocity - phys2d__cross_sv(A.angular_velocity, j.ra);
                return v2dot(axis, dv);
            };
            float len = v2dot(j.d, axis);
            if (!j.enable_spring || j.hertz > 0.0f) {
                phys2d__soft soft = j.enable_spring ? phys2d__make_soft(j.hertz, j.damping_ratio, h) : rigid;
                float imp = -j.axial_mass * soft.mass_scale * (axial_speed() + soft.bias_rate * (len - j.length))
                          - soft.impulse_scale * j.axial_impulse;
                j.axial_impulse += imp;
                phys2d__japply_axial(A, B, axis, j.a1, j.a2, imp);
            }
            if (j.enable_limit) {
                float imp = phys2d__limit(len - j.lower, axial_speed(), j.axial_mass, inv_h, rigid, &j.lower_impulse);
                phys2d__japply_axial(A, B, axis, j.a1, j.a2, imp);
                imp = phys2d__limit(j.upper - len, -axial_speed(), j.axial_mass, inv_h, rigid, &j.upper_impulse);
                phys2d__japply_axial(A, B, axis, j.a1, j.a2, -imp);
            }
        }
    }
}

// how hard each joint pulled, and the ones that pulled too hard snap
static void phys2d__joints_finish(phys2d_world& w, float dt) {
    for (size_t i = 0; i < w.joint_count; i++) {
        phys2d_joint& j = w.joints[i];
        if (!j.active || j.idle) continue;
        float linear = j.type == PHYS2D_JOINT_PRISMATIC ? fabsf(j.linear_impulse.x) : v2len(j.linear_impulse);
        j.reaction = (linear + fabsf(j.axial_impulse + j.lower_impulse - j.upper_impulse)) / dt;
        if (j.break_force > 0.0f && j.reaction > j.break_force) {
            phys2d__reserve(w.events, w.event_capacity, w.event_count + 1);
            phys2d_event& e = w.events[w.event_count++];
            memset(&e, 0, sizeof(e));
            e.type = PHYS2D_JOINT_BREAK; e.a = j.a; e.b = j.b; e.joint = (idx)i;
            e.speed = j.reaction;
            phys2d_remove_joint(w, (idx)i);
        }
    }
}

// ---- islands and sleeping ----

static uint32_t phys2d__find(uint32_t* parent, uint32_t i) {
    while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
    return i;
}

// Groups dynamic bodies that touch or are jointed, then wakes every island
// with at least one awake body in it - so a crate landing on a sleeping pile
// wakes the whole pile, not just the box it hit.
static void phys2d__islands_wake(phys2d_world& w) {
    uint32_t* p = w.island;
    for (size_t i = 0; i < w.body_count; i++) { p[i] = (uint32_t)i; w.island_awake[i] = 0; w.island_sleep[i] = 1e30f; }
    for (size_t i = 0; i < w.contact_count; i++) {
        const phys2d_contact& c = w.contacts[i];
        if (c.trigger || !c.touching) continue;
        phys2d_body& a = w.bodies[c.a];
        phys2d_body& b = w.bodies[c.b];
        if (a.type == PHYS2D_DYNAMIC && b.type == PHYS2D_DYNAMIC) p[phys2d__find(p, c.a)] = phys2d__find(p, c.b);
        else if (a.type == PHYS2D_KINEMATIC && !phys2d__idle(a)) phys2d__wake_body(b);   // a moving platform
        else if (b.type == PHYS2D_KINEMATIC && !phys2d__idle(b)) phys2d__wake_body(a);
    }
    for (size_t i = 0; i < w.joint_count; i++) {
        const phys2d_joint& j = w.joints[i];
        if (!j.active || j.a == PIX_INVALID || j.b == PIX_INVALID) continue;
        phys2d_body& a = w.bodies[j.a];
        phys2d_body& b = w.bodies[j.b];
        if (a.type == PHYS2D_DYNAMIC && b.type == PHYS2D_DYNAMIC) p[phys2d__find(p, j.a)] = phys2d__find(p, j.b);
        else if (!phys2d__idle(a) && a.type == PHYS2D_KINEMATIC) phys2d__wake_body(b);
        else if (!phys2d__idle(b) && b.type == PHYS2D_KINEMATIC) phys2d__wake_body(a);
    }
    for (size_t i = 0; i < w.body_count; i++) {
        const phys2d_body& b = w.bodies[i];
        if (b.active && b.type == PHYS2D_DYNAMIC && !b.sleeping) w.island_awake[phys2d__find(p, (uint32_t)i)] = 1;
    }
    for (size_t i = 0; i < w.body_count; i++) {
        phys2d_body& b = w.bodies[i];
        if (b.active && b.type == PHYS2D_DYNAMIC && b.sleeping && w.island_awake[phys2d__find(p, (uint32_t)i)])
            phys2d__wake_body(b);
    }
}

// An island sleeps only when every body in it has been still long enough.
static void phys2d__islands_sleep(phys2d_world& w, float dt) {
    if (!w.allow_sleep) return;
    float lin2 = w.sleep_linear * w.sleep_linear;
    for (size_t i = 0; i < w.body_count; i++) {
        phys2d_body& b = w.bodies[i];
        if (!b.active || b.type != PHYS2D_DYNAMIC || b.sleeping) continue;
        bool still = !b.never_sleep && v2dot(b.velocity, b.velocity) <= lin2 && fabsf(b.angular_velocity) <= w.sleep_angular;
        b.sleep_time = still ? b.sleep_time + dt : 0.0f;
        uint32_t r = phys2d__find(w.island, (uint32_t)i);
        w.island_sleep[r] = minf(w.island_sleep[r], b.sleep_time);
    }
    for (size_t i = 0; i < w.body_count; i++) {
        phys2d_body& b = w.bodies[i];
        if (!b.active || b.type != PHYS2D_DYNAMIC || b.sleeping) continue;
        if (w.island_sleep[phys2d__find(w.island, (uint32_t)i)] < w.time_to_sleep) continue;
        b.sleeping = true;
        b.velocity = v2(0.0f, 0.0f);
        b.angular_velocity = 0.0f;
        // so next step does not read this step's motion as the game moving it
        b.prev_position = b.position;
        b.prev_angle = b.angle;
    }
}

// ---- broadphase grid ----

static inline int32_t phys2d__cell(float x, float inv_cell) { return (int32_t)floorf(x * inv_cell); }
static inline uint32_t phys2d__bucket(int32_t cx, int32_t cy) {
    return ((uint32_t)cx * 73856093u ^ (uint32_t)cy * 19349663u) & (PHYS2D_GRID_BUCKETS - 1);
}

// Buckets every body for which `want(type)` holds into every cell its bounds
// cover: a counting pass, a prefix sum, then a fill that turns each bucket's
// start into its end - shifted back by one at the end.
static void phys2d__build_grid(phys2d_world& w, phys2d__grid& g, bool statics) {
    float inv = 1.0f / w.cell_size;
    uint32_t* start = g.start;
    memset(start, 0, (PHYS2D_GRID_BUCKETS + 1) * sizeof(uint32_t));
    size_t total = 0;
    for (size_t i = 0; i < w.body_count; i++) {
        const phys2d_body& b = w.bodies[i];
        if (!b.active || (b.type == PHYS2D_STATIC) != statics) continue;
        int32_t x0 = phys2d__cell(b.bounds.min.x, inv), x1 = phys2d__cell(b.bounds.max.x, inv);
        int32_t y0 = phys2d__cell(b.bounds.min.y, inv), y1 = phys2d__cell(b.bounds.max.y, inv);
        for (int32_t y = y0; y <= y1; y++)
            for (int32_t x = x0; x <= x1; x++) { start[phys2d__bucket(x, y)]++; total++; }
    }
    phys2d__reserve(g.items, g.capacity, total);
    g.count = total;
    uint32_t sum = 0;
    for (int i = 0; i < PHYS2D_GRID_BUCKETS; i++) { uint32_t c = start[i]; start[i] = sum; sum += c; }
    for (size_t i = 0; i < w.body_count; i++) {
        const phys2d_body& b = w.bodies[i];
        if (!b.active || (b.type == PHYS2D_STATIC) != statics) continue;
        int32_t x0 = phys2d__cell(b.bounds.min.x, inv), x1 = phys2d__cell(b.bounds.max.x, inv);
        int32_t y0 = phys2d__cell(b.bounds.min.y, inv), y1 = phys2d__cell(b.bounds.max.y, inv);
        for (int32_t y = y0; y <= y1; y++)
            for (int32_t x = x0; x <= x1; x++) {
                phys2d__cell_item& it = g.items[start[phys2d__bucket(x, y)]++];
                it.cx = x; it.cy = y; it.body = (idx)i;
            }
    }
    memmove(start + 1, start, PHYS2D_GRID_BUCKETS * sizeof(uint32_t));
    start[0] = 0;
}

// a pair seen in several cells is owned by the cell holding the min corner of
// the two bounds' overlap; both bodies are always bucketed there
static inline bool phys2d__owns_pair(const rect2& a, const rect2& b, int32_t cx, int32_t cy, float inv) {
    return phys2d__cell(maxf(a.min.x, b.min.x), inv) == cx && phys2d__cell(maxf(a.min.y, b.min.y), inv) == cy;
}

static inline bool phys2d__layers_ok(const phys2d_body& a, const phys2d_body& b) {
    return phys_layers_collide(a.layer, a.mask, b.layer, b.mask);
}

// ---- contacts ----

static inline uint64_t phys2d__pair_key(idx a, idx b) { return ((uint64_t)a << 32) | b; }
static inline uint32_t phys2d__hash64(uint64_t k) {
    k ^= k >> 33; k *= 0xff51afd7ed558ccdull; k ^= k >> 33;
    return (uint32_t)k;
}

static const phys2d_contact* phys2d__find_prev(const phys2d_world& w, idx a, idx b) {
    if (!w.table_size) return 0;
    uint64_t key = phys2d__pair_key(a, b);
    size_t mask = w.table_size - 1;
    for (size_t h = phys2d__hash64(key) & mask;; h = (h + 1) & mask) {
        uint32_t i = w.prev_table[h];
        if (i == 0xFFFFFFFFu) return 0;
        if (w.prev_contacts[i].a == a && w.prev_contacts[i].b == b) return &w.prev_contacts[i];
    }
}

static void phys2d__push_event(phys2d_world& w, uint8_t type, idx a, idx b, vec2 point, vec2 normal, float speed) {
    phys2d__reserve(w.events, w.event_capacity, w.event_count + 1);
    phys2d_event& e = w.events[w.event_count++];
    e.type = type; e.a = a; e.b = b; e.point = point; e.normal = normal; e.speed = speed;
}

static void phys2d__narrowphase(phys2d_world& w, idx ia, idx ib, float dt) {
    if (ia > ib) { idx t = ia; ia = ib; ib = t; }
    phys2d_body& a = w.bodies[ia];
    phys2d_body& b = w.bodies[ib];
    if (!phys2d__layers_ok(a, b)) return;
    bool trigger = a.trigger || b.trigger;
    if (a.trigger && b.trigger) return;
    if (!trigger && a.type != PHYS2D_DYNAMIC && b.type != PHYS2D_DYNAMIC) return;
    if (!phys2d__rects_overlap(a.bounds, b.bounds)) return;
    if (a.joint_list != PIX_INVALID && b.joint_list != PIX_INVALID && phys2d__joined(w, ia, ib)) return;
    if (phys2d__idle(a) && phys2d__idle(b)) {
        // nothing moved: keep last step's contact as it was, events and all
        const phys2d_contact* old = phys2d__find_prev(w, ia, ib);
        if (!old) return;
        phys2d__reserve(w.contacts, w.contact_capacity, w.contact_count + 1);
        phys2d_contact& c = w.contacts[w.contact_count++];
        c = *old;
        c.prev = (uint32_t)(old - w.prev_contacts);
        c.persisted = false;
        return;
    }

    const phys2d__poly& pa = w.polys[ia];
    const phys2d__poly& pb = w.polys[ib];
    // speculative margin: how far the pair can close this step
    float margin = trigger ? 0.0f : w.speculative + v2len(b.velocity - a.velocity) * dt;

    phys2d__reserve(w.contacts, w.contact_capacity, w.contact_count + 1);
    phys2d_contact& c = w.contacts[w.contact_count];
    if (!phys2d__collide(pa, pb, w.slop, margin, c)) return;
    c.a = ia; c.b = ib;
    c.trigger = trigger;
    c.friction = sqrtf(a.friction * b.friction);
    c.restitution = maxf(a.restitution, b.restitution);
    float min_sep = 1e30f;
    for (int i = 0; i < c.count; i++) min_sep = minf(min_sep, c.points[i].separation);
    if (trigger && min_sep >= 0.0f) return;
    c.touching = trigger || min_sep <= w.slop;    // phys2d__report adds "took an impulse"
    c.persisted = false;

    // warm start from the same features last step
    const phys2d_contact* old = phys2d__find_prev(w, ia, ib);
    c.prev = old ? (uint32_t)(old - w.prev_contacts) : 0xFFFFFFFFu;
    if (old && !trigger)
        for (int i = 0; i < c.count; i++)
            for (int j = 0; j < old->count; j++)
                if (old->points[j].id == c.points[i].id) {
                    c.points[i].normal_impulse = old->points[j].normal_impulse;
                    c.points[i].tangent_impulse = old->points[j].tangent_impulse;
                }
    w.contact_count++;
}

static void phys2d__find_contacts(phys2d_world& w, float dt) {
    float inv = 1.0f / w.cell_size;
    const phys2d__grid& mg = w.moving_grid;
    const phys2d__grid& sg = w.static_grid;
    // moving vs moving: pairs within each bucket that share a cell
    for (int bk = 0; bk < PHYS2D_GRID_BUCKETS; bk++)
        for (uint32_t i = mg.start[bk]; i < mg.start[bk + 1]; i++)
            for (uint32_t j = i + 1; j < mg.start[bk + 1]; j++) {
                const phys2d__cell_item& p = mg.items[i];
                const phys2d__cell_item& q = mg.items[j];
                if (p.cx != q.cx || p.cy != q.cy) continue;
                const rect2& ra = w.bodies[p.body].bounds;
                const rect2& rb = w.bodies[q.body].bounds;
                if (!phys2d__rects_overlap(ra, rb) || !phys2d__owns_pair(ra, rb, p.cx, p.cy, inv)) continue;
                phys2d__narrowphase(w, p.body, q.body, dt);
            }
    // moving vs static: look each moving body up in the static grid
    for (size_t i = 0; i < w.body_count; i++) {
        const phys2d_body& b = w.bodies[i];
        if (!b.active || b.type == PHYS2D_STATIC) continue;
        int32_t x0 = phys2d__cell(b.bounds.min.x, inv), x1 = phys2d__cell(b.bounds.max.x, inv);
        int32_t y0 = phys2d__cell(b.bounds.min.y, inv), y1 = phys2d__cell(b.bounds.max.y, inv);
        for (int32_t y = y0; y <= y1; y++)
            for (int32_t x = x0; x <= x1; x++) {
                uint32_t bk = phys2d__bucket(x, y);
                for (uint32_t k = sg.start[bk]; k < sg.start[bk + 1]; k++) {
                    const phys2d__cell_item& it = sg.items[k];
                    if (it.cx != x || it.cy != y) continue;
                    const rect2& rs = w.bodies[it.body].bounds;
                    if (!phys2d__rects_overlap(b.bounds, rs) || !phys2d__owns_pair(b.bounds, rs, x, y, inv)) continue;
                    phys2d__narrowphase(w, (idx)i, it.body, dt);
                }
            }
    }
}

// Begin/end events by comparing this step's touching pairs with the last's.
// Runs after the solve: a speculative point that stopped a body counts as a
// touch (as in Box2D), or a ball bouncing off from a few pixels away would
// never report hitting the floor.
static void phys2d__report(phys2d_world& w) {
    for (size_t i = 0; i < w.contact_count; i++) {
        phys2d_contact& c = w.contacts[i];
        for (int k = 0; k < c.count && !c.trigger; k++) c.touching |= c.points[k].max_normal_impulse > 0.0f;
        bool was_touching = false;
        if (c.prev != 0xFFFFFFFFu) {
            was_touching = w.prev_contacts[c.prev].touching;
            w.prev_contacts[c.prev].persisted = c.touching;
        }
        if (!c.touching || was_touching) continue;
        if (c.trigger) {
            bool a_is_trigger = w.bodies[c.a].trigger;
            phys2d__push_event(w, PHYS2D_TRIGGER_ENTER, a_is_trigger ? c.a : c.b, a_is_trigger ? c.b : c.a,
                               c.points[0].point, v2(0.0f, 0.0f), 0.0f);
        } else {
            float speed = 0.0f;
            for (int k = 0; k < c.count; k++) speed = maxf(speed, c.points[k].approach_speed);
            phys2d__push_event(w, PHYS2D_CONTACT_BEGIN, c.a, c.b, c.points[0].point, c.normal, speed);
        }
    }
    // a pair that was touching and is no longer, or whose body was removed
    for (size_t i = 0; i < w.prev_count; i++) {
        const phys2d_contact& o = w.prev_contacts[i];
        if (!o.touching || o.persisted) continue;
        bool a_is_trigger = w.bodies[o.a].trigger;
        uint8_t type = o.trigger ? PHYS2D_TRIGGER_EXIT : PHYS2D_CONTACT_END;
        idx ea = o.trigger && !a_is_trigger ? o.b : o.a, eb = o.trigger && !a_is_trigger ? o.a : o.b;
        phys2d__push_event(w, type, ea, eb, v2(0.0f, 0.0f), v2(0.0f, 0.0f), 0.0f);
    }
}

// ---- solver ----
//
// Each step is cut into `substeps` (Box2D v3's soft step). Contacts are
// found once per step; then every substep integrates velocities, warm
// starts, solves with position correction, moves the bodies, and relaxes -
// solves again without correction, so pushing an overlap apart never turns
// into a bounce. Short substeps are what keep a long chain or a heavy box on
// a light one from stretching: far better than more iterations of one long
// step, for the same cost.

static void phys2d__prepare_contacts(phys2d_world& w) {
    for (size_t i = 0; i < w.contact_count; i++) {
        phys2d_contact& c = w.contacts[i];
        phys2d_body& a = w.bodies[c.a];
        phys2d_body& b = w.bodies[c.b];
        c.idle = c.trigger || (phys2d__idle(a) && phys2d__idle(b));
        if (c.idle) continue;
        vec2 n = c.normal, t = phys2d__perp(n);
        for (int k = 0; k < c.count; k++) {
            phys2d_contact_point& p = c.points[k];
            // anchors stay fixed for the whole step; the separation is tracked
            // from how far the bodies have moved (phys2d__separation)
            p.ra = p.point - a.position;
            p.rb = p.point - b.position;
            float rna = phys2d__cross(p.ra, n), rnb = phys2d__cross(p.rb, n);
            float rta = phys2d__cross(p.ra, t), rtb = phys2d__cross(p.rb, t);
            float kn = a.inv_mass + b.inv_mass + a.inv_inertia * rna * rna + b.inv_inertia * rnb * rnb;
            float kt = a.inv_mass + b.inv_mass + a.inv_inertia * rta * rta + b.inv_inertia * rtb * rtb;
            p.normal_mass = kn > 0.0f ? 1.0f / kn : 0.0f;
            p.tangent_mass = kt > 0.0f ? 1.0f / kt : 0.0f;
            vec2 dv = b.velocity + phys2d__cross_sv(b.angular_velocity, p.rb)
                    - a.velocity - phys2d__cross_sv(a.angular_velocity, p.ra);
            p.approach_speed = -v2dot(dv, n);
            p.max_normal_impulse = 0.0f;
        }
        // The 2x2 effective mass of a two-point manifold, for the block
        // solver. Skipped when nearly singular (the two points almost on top
        // of each other, or an infinitely heavy pair).
        c.block = false;
        if (c.count == 2) {
            const phys2d_contact_point& p1 = c.points[0];
            const phys2d_contact_point& p2 = c.points[1];
            float r1a = phys2d__cross(p1.ra, n), r1b = phys2d__cross(p1.rb, n);
            float r2a = phys2d__cross(p2.ra, n), r2b = phys2d__cross(p2.rb, n);
            float m = a.inv_mass + b.inv_mass;
            c.k11 = m + a.inv_inertia * r1a * r1a + b.inv_inertia * r1b * r1b;
            c.k22 = m + a.inv_inertia * r2a * r2a + b.inv_inertia * r2b * r2b;
            c.k12 = m + a.inv_inertia * r1a * r2a + b.inv_inertia * r1b * r2b;
            float det = c.k11 * c.k22 - c.k12 * c.k12;
            if (c.k11 * c.k11 < 1000.0f * det) {
                float inv = 1.0f / det;
                c.i11 = c.k22 * inv; c.i22 = c.k11 * inv; c.i12 = -c.k12 * inv;
                c.block = true;
            }
        }
    }
}

static inline void phys2d__apply(phys2d_body& a, phys2d_body& b, const phys2d_contact_point& p, vec2 P) {
    a.velocity -= P * a.inv_mass;
    a.angular_velocity -= a.inv_inertia * phys2d__cross(p.ra, P);
    b.velocity += P * b.inv_mass;
    b.angular_velocity += b.inv_inertia * phys2d__cross(p.rb, P);
}

static void phys2d__warm_contacts(phys2d_world& w) {
    for (size_t i = 0; i < w.contact_count; i++) {
        phys2d_contact& c = w.contacts[i];
        if (c.idle) continue;
        vec2 n = c.normal, t = phys2d__perp(n);
        for (int k = 0; k < c.count; k++)
            phys2d__apply(w.bodies[c.a], w.bodies[c.b], c.points[k], n * c.points[k].normal_impulse + t * c.points[k].tangent_impulse);
    }
}

// where a contact point's gap is now: its separation at the start of the
// step, plus how far the two anchors have moved along the normal since
static inline float phys2d__separation(const phys2d_body& a, const phys2d_body& b, const phys2d_contact_point& p, vec2 n) {
    vec2 ra = v2(a.delta_rot.x * p.ra.x - a.delta_rot.y * p.ra.y, a.delta_rot.y * p.ra.x + a.delta_rot.x * p.ra.y);
    vec2 rb = v2(b.delta_rot.x * p.rb.x - b.delta_rot.y * p.rb.y, b.delta_rot.y * p.rb.x + b.delta_rot.x * p.rb.y);
    vec2 moved = (b.position - b.prev_position + rb - p.rb) - (a.position - a.prev_position + ra - p.ra);
    return p.separation + v2dot(moved, n);
}

static inline float phys2d__normal_speed(const phys2d_body& a, const phys2d_body& b,
                                         const phys2d_contact_point& p, vec2 n) {
    vec2 dv = b.velocity + phys2d__cross_sv(b.angular_velocity, p.rb)
            - a.velocity - phys2d__cross_sv(a.angular_velocity, p.ra);
    return v2dot(dv, n);
}

// Both points of a face contact at once, as Box2D v2's block solver does: find
// the accumulated impulses x >= 0 with the closing speeds at their targets,
// trying "both push", "only 1", "only 2", "neither" until one is consistent.
// Solving the two one after the other instead biases whichever goes first,
// and a stack of boxes rocks on that bias.
static void phys2d__solve_block(phys2d_contact& c, phys2d_body& a, phys2d_body& b) {
    phys2d_contact_point& p1 = c.points[0];
    phys2d_contact_point& p2 = c.points[1];
    vec2 n = c.normal;
    float a1 = p1.normal_impulse, a2 = p2.normal_impulse;
    // b = vn - target - K * a, so that vn_new = K * x + b
    float b1 = phys2d__normal_speed(a, b, p1, n) - p1.bias - (c.k11 * a1 + c.k12 * a2);
    float b2 = phys2d__normal_speed(a, b, p2, n) - p2.bias - (c.k12 * a1 + c.k22 * a2);
    float x1, x2;
    for (;;) {
        x1 = -(c.i11 * b1 + c.i12 * b2);
        x2 = -(c.i12 * b1 + c.i22 * b2);
        if (x1 >= 0.0f && x2 >= 0.0f) break;
        x1 = -p1.normal_mass * b1; x2 = 0.0f;
        if (x1 >= 0.0f && c.k12 * x1 + b2 >= 0.0f) break;
        x1 = 0.0f; x2 = -p2.normal_mass * b2;
        if (x2 >= 0.0f && c.k12 * x2 + b1 >= 0.0f) break;
        x1 = 0.0f; x2 = 0.0f;
        if (b1 >= 0.0f && b2 >= 0.0f) break;
        return;   // no consistent case (rare, degenerate): leave the impulses as they are
    }
    phys2d__apply(a, b, p1, n * (x1 - a1));
    phys2d__apply(a, b, p2, n * (x2 - a2));
    p1.normal_impulse = x1; p2.normal_impulse = x2;
    p1.max_normal_impulse = maxf(p1.max_normal_impulse, x1);
    p2.max_normal_impulse = maxf(p2.max_normal_impulse, x2);
}

// `push_rate` is how much of an overlap (beyond slop) is removed per second;
// 0 in the relax pass. A point still apart may close its gap and no more.
static void phys2d__solve_contacts(phys2d_world& w, float inv_h, float push_rate) {
    for (size_t i = 0; i < w.contact_count; i++) {
        phys2d_contact& c = w.contacts[i];
        if (c.idle) continue;
        phys2d_body& a = w.bodies[c.a];
        phys2d_body& b = w.bodies[c.b];
        vec2 n = c.normal, t = phys2d__perp(n);
        for (int k = 0; k < c.count; k++) {
            phys2d_contact_point& p = c.points[k];
            float s = phys2d__separation(a, b, p, n);
            p.bias = s > 0.0f ? -s * inv_h : push_rate * maxf(0.0f, -s - w.slop);
        }
        // friction first, bounded by the normal impulse from the last pass
        for (int k = 0; k < c.count; k++) {
            phys2d_contact_point& p = c.points[k];
            float limit = c.friction * p.normal_impulse;
            float pt = clampf(p.tangent_impulse - p.tangent_mass * phys2d__normal_speed(a, b, p, t), -limit, limit);
            phys2d__apply(a, b, p, t * (pt - p.tangent_impulse));
            p.tangent_impulse = pt;
        }
        if (c.block) { phys2d__solve_block(c, a, b); continue; }
        for (int k = 0; k < c.count; k++) {
            phys2d_contact_point& p = c.points[k];
            float pn = maxf(p.normal_impulse + p.normal_mass * (p.bias - phys2d__normal_speed(a, b, p, n)), 0.0f);
            phys2d__apply(a, b, p, n * (pn - p.normal_impulse));
            p.normal_impulse = pn;
            p.max_normal_impulse = maxf(p.max_normal_impulse, pn);
        }
    }
}

// After the solve, as Box2D does: a point that really took an impulse and was
// closing faster than the threshold leaves at `restitution` times that speed.
// Done here rather than as a bias so a speculative point never bounces a body
// off a surface it was not going to reach.
static void phys2d__restitution(phys2d_world& w) {
    for (size_t i = 0; i < w.contact_count; i++) {
        phys2d_contact& c = w.contacts[i];
        if (c.idle || c.restitution <= 0.0f) continue;
        phys2d_body& a = w.bodies[c.a];
        phys2d_body& b = w.bodies[c.b];
        for (int k = 0; k < c.count; k++) {
            phys2d_contact_point& p = c.points[k];
            if (p.approach_speed < w.bounce_threshold || p.max_normal_impulse <= 0.0f) continue;
            float vn = phys2d__normal_speed(a, b, p, c.normal);
            float pn = maxf(p.normal_impulse - p.normal_mass * (vn - c.restitution * p.approach_speed), 0.0f);
            phys2d__apply(a, b, p, c.normal * (pn - p.normal_impulse));
            p.normal_impulse = pn;
        }
    }
}

static void phys2d__step(phys2d_world& w, float dt) {
    if (dt <= 0.0f) return;
    float glen = v2len(w.gravity);
    vec2 gdir = glen > 0.0f ? w.gravity / glen : v2(0.0f, 0.0f);

    // wake what the game touched, then bounds swept along the motion for the broadphase
    for (size_t i = 0; i < w.body_count; i++) {
        phys2d_body& b = w.bodies[i];
        if (!b.active) continue;
        if (b.sleeping) {
            // the game moved it, gave it a velocity or pushed it: wake up
            bool moved = b.position.x != b.prev_position.x || b.position.y != b.prev_position.y || b.angle != b.prev_angle;
            bool pushed = b.velocity.x != 0.0f || b.velocity.y != 0.0f || b.angular_velocity != 0.0f ||
                          b.force.x != 0.0f || b.force.y != 0.0f || b.torque != 0.0f;
            if (moved || pushed) phys2d__wake_body(b);
        }
        b.prev_position = b.position;
        b.prev_angle = b.angle;
        b.delta_rot = v2(1.0f, 0.0f);
        if (!b.sleeping) b.on_ground = false;
        phys2d__poly& p = w.polys[i];
        phys2d__make_poly(b.shape, b.position, b.angle, p);
        rect2 r = phys2d__poly_bounds(p);
        if (b.type == PHYS2D_STATIC) {
            if (r.min.x != b.bounds.min.x || r.min.y != b.bounds.min.y ||
                r.max.x != b.bounds.max.x || r.max.y != b.bounds.max.y) w.statics_dirty = true;
            b.bounds = r;
            continue;
        }
        vec2 v = b.velocity;
        if (b.type == PHYS2D_DYNAMIC && !b.sleeping) v += w.gravity * (b.gravity_scale * dt);
        vec2 move = v * dt, pad = v2(w.speculative, w.speculative);
        r.min = v2(r.min.x + minf(move.x, 0.0f), r.min.y + minf(move.y, 0.0f)) - pad;
        r.max = v2(r.max.x + maxf(move.x, 0.0f), r.max.y + maxf(move.y, 0.0f)) + pad;
        b.bounds = r;
    }
    if (w.statics_dirty) { phys2d__build_grid(w, w.static_grid, true); w.statics_dirty = false; }
    phys2d__build_grid(w, w.moving_grid, false);

    // last step's contacts become the warm-start / event reference
    phys2d_contact* t = w.prev_contacts; w.prev_contacts = w.contacts; w.contacts = t;
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
        size_t h = phys2d__hash64(phys2d__pair_key(w.prev_contacts[i].a, w.prev_contacts[i].b)) & (w.table_size - 1);
        while (w.prev_table[h] != 0xFFFFFFFFu) h = (h + 1) & (w.table_size - 1);
        w.prev_table[h] = (uint32_t)i;
    }

    phys2d__find_contacts(w, dt);
    phys2d__islands_wake(w);
    phys2d__prepare_contacts(w);

    int substeps = w.substeps > 0 ? w.substeps : 1;
    float h = dt / substeps, inv_h = 1.0f / h;
    float push_rate = w.correction / dt;     // the same correction per second at any substep count
    for (int sub = 0; sub < substeps; sub++) {
        for (size_t i = 0; i < w.body_count; i++) {
            phys2d_body& b = w.bodies[i];
            if (!b.active || b.type != PHYS2D_DYNAMIC || b.sleeping) continue;
            b.velocity += (w.gravity * b.gravity_scale + b.force * b.inv_mass) * h;
            b.angular_velocity += b.torque * b.inv_inertia * h;
            b.velocity *= 1.0f / (1.0f + h * b.linear_damping);
            b.angular_velocity *= 1.0f / (1.0f + h * b.angular_damping);
            if (b.inv_inertia == 0.0f) b.angular_velocity = 0.0f;
        }
        phys2d__joints_prepare(w);
        phys2d__joints_warm(w);
        phys2d__warm_contacts(w);
        for (int i = 0; i < w.iterations; i++) {
            phys2d__joints_solve(w, h, true);
            phys2d__solve_contacts(w, inv_h, push_rate);
        }
        for (size_t i = 0; i < w.body_count; i++) {
            phys2d_body& b = w.bodies[i];
            if (!b.active || b.type == PHYS2D_STATIC || b.sleeping) continue;
            b.position += b.velocity * h;
            b.angle += b.angular_velocity * h;
            float turned = b.angle - b.prev_angle;
            b.delta_rot = v2(cosf(turned), sinf(turned));
        }
        for (int i = 0; i < w.relax_iterations; i++) {
            phys2d__joints_solve(w, h, false);
            phys2d__solve_contacts(w, inv_h, 0.0f);
        }
    }
    phys2d__restitution(w);
    phys2d__report(w);
    phys2d__joints_finish(w, h);

    for (size_t i = 0; i < w.contact_count; i++) {
        const phys2d_contact& c = w.contacts[i];
        if (c.trigger || !c.touching || glen == 0.0f) continue;
        // normal points a -> b: b is below a when it runs along gravity
        float d = v2dot(c.normal, gdir);
        if (d > 0.7f)  { w.bodies[c.a].on_ground = true; w.bodies[c.a].ground_normal = -c.normal; }
        if (d < -0.7f) { w.bodies[c.b].on_ground = true; w.bodies[c.b].ground_normal = c.normal; }
    }
    for (size_t i = 0; i < w.body_count; i++) {
        w.bodies[i].force = v2(0.0f, 0.0f);
        w.bodies[i].torque = 0.0f;
    }
    phys2d__islands_sleep(w, dt);

    for (size_t i = 0; i < w.pending_count; i++) w.free_ids[w.free_count++] = w.pending_free[i];
    w.pending_count = 0;
}

void phys2d_step(phys2d_world& w, float dt) {
    w.event_count = 0;
    phys2d__step(w, dt);
    w.alpha = 1.0f;
}

int phys2d_update(phys2d_world& w, float frame_dt) {
    w.event_count = 0;
    w.accumulator += frame_dt;
    int steps = 0;
    while (w.accumulator >= w.fixed_dt && steps < w.max_steps) {
        phys2d__step(w, w.fixed_dt);
        w.accumulator -= w.fixed_dt;
        steps++;
    }
    if (w.accumulator > w.fixed_dt) w.accumulator = w.fixed_dt;   // fell behind: drop time
    w.alpha = w.accumulator / w.fixed_dt;
    return steps;
}

// ---- queries ----

// every body whose bounds overlap `r`, from both grids, each reported once
template <typename F>
static void phys2d__query_rect(phys2d_world& w, rect2 r, F&& visit) {
    float inv = 1.0f / w.cell_size;
    if (w.statics_dirty) {
        // a query between adding statics and the first step still sees them
        for (size_t i = 0; i < w.body_count; i++) {
            phys2d_body& b = w.bodies[i];
            if (!b.active || b.type != PHYS2D_STATIC) continue;
            phys2d__poly p;
            phys2d__make_poly(b.shape, b.position, b.angle, p);
            b.bounds = phys2d__poly_bounds(p);
        }
        phys2d__build_grid(w, w.static_grid, true);
        w.statics_dirty = false;
    }
    int32_t x0 = phys2d__cell(r.min.x, inv), x1 = phys2d__cell(r.max.x, inv);
    int32_t y0 = phys2d__cell(r.min.y, inv), y1 = phys2d__cell(r.max.y, inv);
    const phys2d__grid* grids[2] = { &w.static_grid, &w.moving_grid };
    for (int g = 0; g < 2; g++) {
        if (!grids[g]->items) continue;
        for (int32_t y = y0; y <= y1; y++)
            for (int32_t x = x0; x <= x1; x++) {
                uint32_t bk = phys2d__bucket(x, y);
                for (uint32_t k = grids[g]->start[bk]; k < grids[g]->start[bk + 1]; k++) {
                    const phys2d__cell_item& it = grids[g]->items[k];
                    if (it.cx != x || it.cy != y) continue;
                    const phys2d_body& b = w.bodies[it.body];
                    if (!b.active || !phys2d__rects_overlap(r, b.bounds)) continue;
                    // the same min-corner rule as pairs, so a body in many cells visits once
                    if (!phys2d__owns_pair(r, b.bounds, x, y, inv)) continue;
                    visit(it.body, b);
                }
            }
    }
}

// Ray against one core + radius in the shape's terms: a point is a circle, a
// segment is a capsule (two circles and two side lines), a box is a slab test
// in its own frame. Returns false when the ray starts inside.
static bool phys2d__ray_circle(vec2 o, vec2 d, vec2 c, float r, float* t, vec2* n) {
    vec2 m = o - c;
    float b = v2dot(m, d), cc = v2dot(m, m) - r * r;
    if (cc <= 0.0f) return false;
    float disc = b * b - cc;
    if (b > 0.0f || disc < 0.0f) return false;
    *t = -b - sqrtf(disc);
    *n = v2norm(m + d * *t);
    return true;
}

static bool phys2d__ray_poly(const phys2d__poly& p, vec2 o, vec2 d, float* out_t, vec2* out_n) {
    if (p.count == 1) return phys2d__ray_circle(o, d, p.v[0], p.radius, out_t, out_n);
    if (p.count == 2) {
        vec2 axis = p.v[1] - p.v[0];
        float len = v2len(axis);
        vec2 u = axis / len;
        // starts inside the capsule?
        vec2 q = phys2d__closest_on_segment(p.v[0], p.v[1], o);
        if (v2dot(o - q, o - q) <= p.radius * p.radius) return false;
        float best = 1e30f; vec2 bn = v2(0, 0); float t; vec2 n;
        for (int e = 0; e < 2; e++)
            if (phys2d__ray_circle(o, d, p.v[e], p.radius, &t, &n) && t < best) { best = t; bn = n; }
        for (int s = 0; s < 2; s++) {
            vec2 sn = s ? -p.n[0] : p.n[0];
            float denom = v2dot(d, sn);
            if (denom >= 0.0f) continue;
            t = (p.radius - v2dot(o - p.v[0], sn)) / denom;
            float along = v2dot(o + d * t - p.v[0], u);
            if (t >= 0.0f && along >= 0.0f && along <= len && t < best) { best = t; bn = sn; }
        }
        if (best == 1e30f) return false;
        *out_t = best; *out_n = bn;
        return true;
    }
    // box: slabs along its two face normals
    vec2 c = (p.v[0] + p.v[2]) * 0.5f;
    float t0 = -1e30f, t1 = 1e30f; vec2 n0 = v2(0, 0);
    for (int k = 0; k < 2; k++) {
        vec2 ax = p.n[k + 1];   // n[1] = +x, n[2] = +y in local terms
        float h = v2dot(p.v[2] - c, ax);
        float oo = v2dot(o - c, ax), dd = v2dot(d, ax);
        if (fabsf(dd) < 1e-9f) { if (oo < -h || oo > h) return false; continue; }
        float ta = (-h - oo) / dd, tb = (h - oo) / dd;
        vec2 na = -ax;
        if (ta > tb) { float s = ta; ta = tb; tb = s; na = ax; }
        if (ta > t0) { t0 = ta; n0 = na; }
        t1 = minf(t1, tb);
        if (t0 > t1) return false;
    }
    if (t0 < 0.0f) return false;   // inside, or behind
    *out_t = t0; *out_n = n0;
    return true;
}

bool phys2d_raycast(phys2d_world& w, vec2 origin, vec2 direction, float max_distance,
                    phys2d_ray_hit* hit, uint16_t mask, idx ignore) {
    float len = v2len(direction);
    if (len <= 0.0f || max_distance <= 0.0f) return false;
    vec2 d = direction / len, end = origin + d * max_distance;
    rect2 r = { v2(minf(origin.x, end.x), minf(origin.y, end.y)), v2(maxf(origin.x, end.x), maxf(origin.y, end.y)) };
    float best = max_distance;
    bool found = false;
    // shortcut: tests every body in the ray's bounding box rather than walking
    // cells in order (DDA); fine for game-length rays, slow for map-long ones
    phys2d__query_rect(w, r, [&](idx id, const phys2d_body& b) {
        if (id == ignore || b.trigger || !(b.layer & mask)) return;
        phys2d__poly p;
        phys2d__make_poly(b.shape, b.position, b.angle, p);
        float t; vec2 n;
        if (!phys2d__ray_poly(p, origin, d, &t, &n) || t > best) return;
        best = t;
        found = true;
        if (hit) { hit->body = id; hit->distance = t; hit->point = origin + d * t; hit->normal = n; }
    });
    return found;
}

bool phys2d_shape_cast(phys2d_world& w, phys2d_shape shape, vec2 position, float angle,
                       vec2 direction, float max_distance, phys2d_ray_hit* hit, uint16_t mask, idx ignore) {
    float len = v2len(direction);
    if (len <= 0.0f) return false;
    vec2 d = direction / len;
    phys2d__poly start;
    phys2d__make_poly(shape, position, angle, start);
    rect2 r0 = phys2d__poly_bounds(start);
    vec2 move = d * max_distance;
    rect2 r = { v2(r0.min.x + minf(move.x, 0.0f), r0.min.y + minf(move.y, 0.0f)),
                v2(r0.max.x + maxf(move.x, 0.0f), r0.max.y + maxf(move.y, 0.0f)) };
    float best = max_distance;
    bool found = false;
    float tolerance = 0.25f * w.slop;
    phys2d__query_rect(w, r, [&](idx id, const phys2d_body& b) {
        if (id == ignore || b.trigger || !(b.layer & mask)) return;
        phys2d__poly target;
        phys2d__make_poly(b.shape, b.position, b.angle, target);
        // Conservative advancement: when only translating at unit speed, the
        // gap cannot shrink faster than the distance travelled, so stepping
        // by the current gap never jumps past the first touch.
        float t = 0.0f;
        for (int iter = 0; iter < 32; iter++) {
            phys2d__poly moved = start;
            for (int k = 0; k < moved.count; k++) moved.v[k] += d * t;
            vec2 pa, pb;
            float gap = 0.0f;
            vec2 n = -d;
            if (phys2d__cores_disjoint(moved, target)) {
                float cd = phys2d__core_distance(moved, target, &pa, &pb);
                gap = cd - moved.radius - target.radius;
                if (cd > 1e-6f) n = (pa - pb) / cd;
            }
            if (gap <= tolerance) {
                if (t <= best) {
                    best = t;
                    found = true;
                    if (hit) {
                        hit->body = id; hit->distance = t; hit->normal = n;
                        hit->point = gap > 0.0f || t > 0.0f ? pb + n * target.radius : position;
                    }
                }
                return;
            }
            t += gap;
            if (t > best) return;
        }
    });
    return found;
}

size_t phys2d_overlap(phys2d_world& w, phys2d_shape shape, vec2 position, float angle,
                      idx* out, size_t capacity, uint16_t mask) {
    phys2d__poly q;
    phys2d__make_poly(shape, position, angle, q);
    size_t count = 0;
    phys2d__query_rect(w, phys2d__poly_bounds(q), [&](idx id, const phys2d_body& b) {
        if (!(b.layer & mask)) return;
        phys2d__poly p;
        phys2d__make_poly(b.shape, b.position, b.angle, p);
        phys2d_contact m;
        if (!phys2d__collide(q, p, w.slop, 0.0f, m)) return;
        bool inside = false;
        for (int k = 0; k < m.count; k++) inside |= m.points[k].separation < 0.0f;
        if (!inside) return;
        if (count < capacity) out[count] = id;
        count++;
    });
    return count;
}

#endif // PIX_PHYSICS2D_IMPL
