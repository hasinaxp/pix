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

// A 3D rigid-body world: boxes, spheres, capsules, convex hulls and compounds
// that tumble, roll, stack and rest, on static planes, boxes and triangle
// meshes, with joints for doors, chains, ragdolls and machines, a raycast
// vehicle and a character controller. Units are metres, y is up, gravity is
// 9.81 down.
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
//                (rod, spring, rope), mouse, wheel, motor, gear, pulley and
//                a generic 6-DOF - Bullet's set, as soft constraints.
//   sleeping     islands by union-find; an island that stays still for
//                `time_to_sleep` stops simulating until touched.
//   continuous   conservative advancement (Mirtich) for fast and spinning
//                bodies against static and kinematic ones.
//   hulls        quickhull; SAT with Gauss-map pruning of edge pairs
//                (Gregorius, GDC 2013) and reference-face clipping.
//   vehicles     Bullet's raycast vehicle; the character controller is
//                collide-and-slide with step-up and snap-down.
//
// Layers work as in every pix world: phys_layers_collide (physics_common.hpp).
//
// Not here yet: gyroscopic torque, threading, continuous collision between two
// moving bodies, internal-edge smoothing on meshes.

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
#define PHYS3D_HULL    5     // any convex shape, from points (phys3d_create_hull)
#define PHYS3D_COMPOUND 6    // several shapes glued into one body (phys3d_create_compound)

#ifndef PHYS3D_MAX_HULL_VERTICES
#define PHYS3D_MAX_HULL_VERTICES 256   // a hull keeps at most this many corners
#endif
#define PHYS3D_MAX_FACE_VERTICES 64    // corners on one hull face

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
#define PHYS3D_JOINT_WHEEL    6   // a car wheel: suspension spring, steering, spin with a motor
#define PHYS3D_JOINT_MOTOR    7   // drives b toward a pose relative to a, within a force and torque
#define PHYS3D_JOINT_GEAR     8   // two spins locked in a ratio: gears, belts, differentials
#define PHYS3D_JOINT_PULLEY   9   // two ropes over fixed points: len_a + ratio * len_b stays the same
#define PHYS3D_JOINT_GENERIC  10  // six axes, each free, locked or limited (Bullet's 6-DOF): anything else

// generic joint axes: linear x, y, z then angular x, y, z of its frame
#define PHYS3D_AXIS_LOCKED  0
#define PHYS3D_AXIS_FREE    1
#define PHYS3D_AXIS_LIMITED 2

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

// A convex shape built from any points (the corners of a rock, a crate's
// model, a car body): their convex hull. Build once, keep it alive while
// bodies use it; it owns copies of its data. Its corners are moved so the
// centre of mass is at the origin - `offset` is where that centre was in the
// points you gave, which drawing needs (phys3d_body_draw_transform does it).
struct phys3d_hull_edge { uint16_t v0, v1, f0, f1; };   // corners, and the faces either side
struct phys3d_hull {
    vec3*  vertices;  int vertex_count;     // centred on the centre of mass
    vec4*  planes;    int face_count;       // xyz outward normal, w: dot(normal, p) on the face
    int*   face_start;                      // face f's corners: face_verts[face_start[f] .. face_start[f + 1])
    uint16_t* face_verts;                   // counter-clockwise seen from outside
    phys3d_hull_edge* edges; int edge_count;
    vec3   offset;                          // the centre of mass in your points' frame
    float  volume;
    mat3   inertia;                         // per unit density, about the centre of mass
    aabb   bounds;                          // local
};

struct phys3d_compound;

struct phys3d_shape {
    uint8_t type;
    float   radius;             // sphere, capsule
    vec3    half;               // box half extents
    float   half_length;        // capsule: half the straight part
    vec3    normal;             // plane, in the body's frame
    const phys3d_mesh* mesh;
    const phys3d_hull* hull;
    const phys3d_compound* compound;
};

// One part of a compound: a sphere, box, capsule or hull, placed in the
// compound's frame.
struct phys3d_compound_child {
    phys3d_shape shape;
    vec3 position;
    quat rotation;
};
// Several shapes moving as one rigid body: a table (top and legs), a hammer, a
// boat. Like a hull it is re-centred on its centre of mass: `offset` is where
// that sat in your frame. Build once, keep alive while bodies use it.
struct phys3d_compound {
    phys3d_compound_child* children;
    int   count;
    vec3  offset;
    float volume;
    mat3  inertia;              // per unit density, about the centre of mass
    aabb  bounds;               // local
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
static inline phys3d_shape phys3d_hull_shape(const phys3d_hull& hull) {
    phys3d_shape s = {}; s.type = PHYS3D_HULL; s.hull = &hull; return s;
}
static inline phys3d_shape phys3d_compound_shape(const phys3d_compound& compound) {
    phys3d_shape s = {}; s.type = PHYS3D_COMPOUND; s.compound = &compound; return s;
}
static inline phys3d_compound_child phys3d_child(phys3d_shape shape, vec3 position, quat rotation = quat_identity()) {
    phys3d_compound_child c; c.shape = shape; c.position = position; c.rotation = rotation; return c;
}
// where a shape's own origin sits relative to the body: hulls and compounds
// are centred on their centre of mass, so this is minus their offset
static inline vec3 phys3d_shape_offset(const phys3d_shape& s) {
    if (s.type == PHYS3D_HULL && s.hull) return s.hull->offset;
    if (s.type == PHYS3D_COMPOUND && s.compound) return s.compound->offset;
    return v3(0, 0, 0);
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
    mat3  inv_inertia_local;     // in the body's frame (full: hulls and compounds are not symmetric)
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
    vec3  target;                // mouse: where to drag; motor: b's offset in a's frame
    float max_force;             // mouse, motor
    float break_force;           // 0 = unbreakable
    float steer;                 // wheel: steering angle about the suspension axis, radians
    float ratio;                 // gear, pulley
    vec3  ground_a, ground_b;    // pulley: the fixed points the ropes run over (world)
    // generic: per axis (linear x y z, angular x y z of the joint frame) a
    // PHYS3D_AXIS_* mode, limits (metres / radians) and an optional motor
    uint8_t axis_mode[6];
    float   axis_lower[6], axis_upper[6];
    float   axis_motor_speed[6], axis_motor_force[6];   // force 0 = no motor
    quat    frame_a, frame_b;    // generic: the joint frame in each body

    // ---- written by the world ----
    float reaction;
    idx   next_a, next_b;
    bool  idle;
    vec3  ra, rb, d, axis, perp1, perp2;
    vec3  linear_impulse, angular_impulse;
    float axial_impulse, motor_impulse, lower_impulse, upper_impulse, swing_impulse;
    float axis_impulse[6];       // generic: each axis's lock / limit impulse (lower - upper), for warm starting
    float axis_motor_impulse[6]; // generic: each axis's motor
};

struct phys3d_world;
// Code that runs inside every fixed step, before the solver: vehicles, custom
// forces, buoyancy. `dt` is the step's.
typedef void (*phys3d_action_fn)(phys3d_world& w, float dt, void* user);
#define PHYS3D_MAX_ACTIONS 32

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
    // Fast or fast-spinning bodies are swept against static and kinematic
    // bodies each step and stopped where they would first touch, so a bullet
    // or a whirling blade cannot pass through a thin wall.
    bool  continuous;            // true
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

    struct { phys3d_action_fn fn; void* user; } actions[PHYS3D_MAX_ACTIONS];
    int action_count;
};

// ---- meshes ----
PIXDEF bool phys3d_create_mesh(phys3d_mesh& m, const vec3* vertices, size_t vertex_count,
                               const uint32_t* indices, size_t index_count);
// a terrain grid: heights[row * columns + column], `spacing` metres apart, the
// first sample at (0, h, 0), columns along +x and rows along +z
PIXDEF bool phys3d_create_heightfield(phys3d_mesh& m, const float* heights, int columns, int rows, float spacing);
PIXDEF void phys3d_destroy_mesh(phys3d_mesh& m);

// ---- hulls and compounds ----
// The convex hull of `points` (quickhull; coplanar triangles merged into one
// face). False when the points are flat or fewer than four.
PIXDEF bool phys3d_create_hull(phys3d_hull& h, const vec3* points, size_t count);
PIXDEF void phys3d_destroy_hull(phys3d_hull& h);
// Children: spheres, boxes, capsules, hulls (they reference the hull, which must
// outlive the compound). False for none or an unsupported child.
PIXDEF bool phys3d_create_compound(phys3d_compound& c, const phys3d_compound_child* children, int count);
PIXDEF void phys3d_destroy_compound(phys3d_compound& c);

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
// The same for a model made in the shape's own frame: for a hull or compound
// this undoes its centring, so the model you built it from lines up.
static inline mat4 phys3d_body_draw_transform(const phys3d_world& w, const phys3d_body& b) {
    vec3 o = phys3d_shape_offset(b.shape);
    return phys3d_body_transform(w, b) * mat4_translate(-o.x, -o.y, -o.z);
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
// A wheel on a chassis: `centre` the wheel's centre, `suspension` the way the
// spring works (usually up), `spin` the axle. Springy (hertz 5, damping 0.7),
// steer with j->steer, drive with enable_motor / motor_speed / max_motor_force.
PIXDEF idx phys3d_add_wheel(phys3d_world& w, idx chassis, idx wheel, vec3 centre, vec3 suspension, vec3 spin);
// Holds b where it is relative to a (target / relative_rotation) with at most
// `max_force` and `max_torque`: moving platforms carrying things, a character's
// root following its animation, a soft-follow camera rig.
PIXDEF idx phys3d_add_motor_joint(phys3d_world& w, idx a, idx b, float max_force, float max_torque);
// a's spin about axis_a and b's about axis_b (world) kept as ratio * a + b = 0.
PIXDEF idx phys3d_add_gear(phys3d_world& w, idx a, idx b, vec3 axis_a, vec3 axis_b, float ratio = 1.0f);
// Ropes from anchor_a over ground_a and from anchor_b over ground_b:
// length_a + ratio * length_b stays what it is now.
PIXDEF idx phys3d_add_pulley(phys3d_world& w, idx a, idx b, vec3 ground_a, vec3 ground_b, vec3 anchor_a, vec3 anchor_b,
                             float ratio = 1.0f);
// Every axis locked to start with (a weld); open the ones you want with
// axis_mode / axis_lower / axis_upper. `frame` turns the joint's axes in the world.
PIXDEF idx phys3d_add_generic(phys3d_world& w, idx a, idx b, vec3 anchor, quat frame = quat_identity());
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

// ---- actions ----
// Runs `fn` every fixed step; returns a handle for phys3d_remove_action (-1 when full).
PIXDEF int  phys3d_add_action(phys3d_world& w, phys3d_action_fn fn, void* user);
PIXDEF void phys3d_remove_action(phys3d_world& w, int handle);

// ---- raycast vehicle ----
//
// A car the way most games make one (Bullet's btRaycastVehicle): one rigid
// body for the chassis, and each wheel a ray down from the chassis with a
// spring and damper. Tyres grip sideways and drive forward through impulses
// at the contact, limited together by a friction circle, so it skids,
// drifts and spins out. Cheap and stable at any speed.
//
//   phys3d_vehicle car = phys3d_make_vehicle(chassis_body);
//   for each corner: phys3d_vehicle_add_wheel(car, v3(+-0.8f, 0, +-1.3f), 0.35f, 0.3f);
//   phys3d_add_vehicle(world, car);           // keep `car` alive while added
//   each frame: car.wheels[i].engine_force / brake / steer, then phys3d_update as usual
//   draw wheel i with phys3d_vehicle_wheel_transform(world, car, i)
#define PHYS3D_MAX_VEHICLE_WHEELS 8
struct phys3d_vehicle_wheel {
    vec3  attach;            // where the suspension hangs from, in the chassis frame
    float radius;
    float rest_length;       // suspension length with no load
    float max_travel;        // how far it compresses or extends from rest
    float stiffness;         // spring, N/m (default: holds a quarter of the chassis 10 cm compressed)
    float damping;           // N*s/m
    float friction;          // tyre grip (1.0 dry tarmac)
    float roll_influence;    // 0..1: side forces applied this high up the body (lower = harder to roll over)
    float steer;             // radians, about the chassis up
    float engine_force;      // newtons, along where the wheel points
    float brake;             // newtons of braking
    // ---- written by the vehicle ----
    bool  contact;
    vec3  contact_point, contact_normal;
    idx   ground;
    float suspension_length;
    float suspension_force;
    float spin, spin_speed;  // the wheel's own rotation, for drawing
    float skid;              // 0 gripping .. 1 sliding (for skid marks, tyre squeal)
};
struct phys3d_vehicle {
    idx   chassis;
    vec3  up_local, forward_local;   // the chassis's own up and forward ((0,1,0), (0,0,1))
    int   wheel_count;
    phys3d_vehicle_wheel wheels[PHYS3D_MAX_VEHICLE_WHEELS];
    int   action;                    // its handle while added
};
PIXDEF phys3d_vehicle phys3d_make_vehicle(idx chassis);
PIXDEF int  phys3d_vehicle_add_wheel(phys3d_vehicle& v, vec3 attach, float radius, float rest_length);
PIXDEF void phys3d_add_vehicle(phys3d_world& w, phys3d_vehicle& v);
PIXDEF void phys3d_remove_vehicle(phys3d_world& w, phys3d_vehicle& v);
// the step itself (phys3d_add_vehicle runs it for you every fixed step)
PIXDEF void phys3d_vehicle_step(phys3d_world& w, phys3d_vehicle& v, float dt);
// where to draw wheel i: on its suspension, steered and spinning (interpolated with the chassis)
PIXDEF mat4 phys3d_vehicle_wheel_transform(const phys3d_world& w, const phys3d_vehicle& v, int i);
// forward speed, m/s (negative reversing)
PIXDEF float phys3d_vehicle_speed(phys3d_world& w, const phys3d_vehicle& v);

// ---- character controller ----
//
// A walking capsule that is not simulated: you say where it wants to go, it
// goes as far as the world lets it and slides along whatever it meets (the
// collide-and-slide method). It climbs steps up to step_height, walks up
// slopes up to max_slope and is stopped by anything steeper, stays on the
// ground going down slopes and stairs, rides moving platforms and shoves
// dynamic bodies it walks into.
//
//   phys3d_character hero = phys3d_make_character(v3(0, 0, 0));   // feet position
//   phys3d_add_character_body(world, hero);                      // optional: others bump into it
//   ...
//   phys3d_character_update(world, hero, walk_velocity, pix_key_pressed(KEY_SPACE), dt);
//   draw at hero.position (the feet)
struct phys3d_character {
    vec3  position;          // the feet: the bottom of the capsule
    float radius, height;    // the capsule, height overall
    float max_slope;         // radians: steeper ground is a wall (default 50 degrees)
    float step_height;       // ledges it walks up without jumping (0.35)
    float snap_distance;     // how far down it follows the ground when walking off a slope or step (0.35)
    float skin;              // gap kept to surfaces (0.01)
    float jump_speed;        // phys3d_character_update: upward speed of a jump (5)
    float push_force;        // newtons it shoves dynamic bodies with (600: a person leaning in)
    float frame_dt;          // the time a move covers, for pushing (update sets it; 1/60)
    uint16_t mask;           // what stops it (PHYS_LAYER_ALL)
    idx   body;              // its kinematic body, when added (PIX_INVALID)

    // ---- written by the controller ----
    vec3  velocity;          // what it really moved in the last update, per second
    float vertical_speed;    // along up: jumps and falls, kept between updates
    vec3  last_move;         // what the last phys3d_character_move really moved it
    bool  on_ground;
    vec3  ground_normal;
    idx   ground_body;       // what it stands on (PIX_INVALID when airborne)
};

PIXDEF phys3d_character phys3d_make_character(vec3 feet, float radius = 0.35f, float height = 1.8f);
// Moves by `displacement` (world units, this frame), sliding and stepping.
PIXDEF void phys3d_character_move(phys3d_world& w, phys3d_character& c, vec3 displacement);
// The usual loop in one call: gravity (the world's), a jump when `jump` and on
// the ground, walking at `walk_velocity` (its vertical part is ignored), riding
// whatever it stands on.
PIXDEF void phys3d_character_update(phys3d_world& w, phys3d_character& c, vec3 walk_velocity, bool jump, float dt);
// A kinematic capsule that follows the character, so dynamic bodies collide
// with it and raycasts see it. The controller ignores it when moving.
PIXDEF idx phys3d_add_character_body(phys3d_world& w, phys3d_character& c);
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

// ---- convex hulls ----
//
// Quickhull, incrementally: start from four extreme points, then for every
// point outside the hull, find the faces it can see (a flood fill from the
// one it sees best, so the region stays connected), cut them out and close
// the hole with a fan of triangles from the point to the region's border.
// Triangles that turn out coplanar are merged into polygon faces afterwards,
// so a box comes out with 6 faces, not 12 - which is what the contact
// clipping wants.

struct phys3d__qh_face { int v[3]; vec3 n; float d; bool alive; };

static void phys3d__qh_plane(phys3d__qh_face& f, const vec3* p) {
    f.n = v3norm(v3cross(p[f.v[1]] - p[f.v[0]], p[f.v[2]] - p[f.v[0]]));
    f.d = v3dot(f.n, p[f.v[0]]);
}

// the live face holding directed edge a -> b
static int phys3d__qh_owner(const phys3d__qh_face* f, int count, int a, int b) {
    for (int i = 0; i < count; i++) {
        if (!f[i].alive) continue;
        for (int e = 0; e < 3; e++) if (f[i].v[e] == a && f[i].v[(e + 1) % 3] == b) return i;
    }
    return -1;
}

static uint32_t phys3d__qh_root(uint32_t* parent, uint32_t i) {
    while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
    return i;
}

bool phys3d_create_hull(phys3d_hull& h, const vec3* pts, size_t n) {
    memset(&h, 0, sizeof(h));
    if (!pts || n < 4) return false;
    // ---- the starting tetrahedron ----
    int ext[6] = { 0, 0, 0, 0, 0, 0 };
    for (size_t i = 1; i < n; i++)
        for (int a = 0; a < 3; a++) {
            const float* p = &pts[i].x;
            if (p[a] < (&pts[ext[a * 2]].x)[a]) ext[a * 2] = (int)i;
            if (p[a] > (&pts[ext[a * 2 + 1]].x)[a]) ext[a * 2 + 1] = (int)i;
        }
    int i0 = ext[0], i1 = ext[1];
    float spread = v3dist(pts[i0], pts[i1]);
    for (int a = 1; a < 3; a++)
        if (v3dist(pts[ext[a * 2]], pts[ext[a * 2 + 1]]) > spread) {
            i0 = ext[a * 2]; i1 = ext[a * 2 + 1]; spread = v3dist(pts[i0], pts[i1]);
        }
    float eps = 1e-5f * spread;
    if (spread < 1e-6f) return false;
    int i2 = -1; float best = eps;
    vec3 line = v3norm(pts[i1] - pts[i0]);
    for (size_t i = 0; i < n; i++) {
        vec3 d = pts[i] - pts[i0];
        float dist = v3len(d - line * v3dot(d, line));
        if (dist > best) { best = dist; i2 = (int)i; }
    }
    if (i2 < 0) return false;
    vec3 pn = v3norm(v3cross(pts[i1] - pts[i0], pts[i2] - pts[i0]));
    int i3 = -1; best = eps;
    for (size_t i = 0; i < n; i++) {
        float dist = fabsf(v3dot(pn, pts[i] - pts[i0]));
        if (dist > best) { best = dist; i3 = (int)i; }
    }
    if (i3 < 0) return false;   // flat

    size_t cap = 8 + n * 4;
    phys3d__qh_face* f = pix_alloc<phys3d__qh_face>(cap);
    int* stack = pix_alloc<int>(cap);
    int* visible = pix_alloc<int>(cap);
    uint8_t* seen = pix_alloc<uint8_t>(cap);
    int* horizon = pix_alloc<int>(cap * 2);
    if (!f || !stack || !visible || !seen || !horizon) {
        pix_free(f); pix_free(stack); pix_free(visible); pix_free(seen); pix_free(horizon);
        return false;
    }
    int fc = 0;
    vec3 inside = (pts[i0] + pts[i1] + pts[i2] + pts[i3]) * 0.25f;
    int tet[4][3] = { { i0, i1, i2 }, { i0, i3, i1 }, { i1, i3, i2 }, { i2, i3, i0 } };
    for (int t = 0; t < 4; t++) {
        phys3d__qh_face& nf = f[fc++];
        nf.v[0] = tet[t][0]; nf.v[1] = tet[t][1]; nf.v[2] = tet[t][2];
        nf.alive = true;
        phys3d__qh_plane(nf, pts);
        if (v3dot(nf.n, inside) > nf.d) {   // facing in: flip
            int s = nf.v[1]; nf.v[1] = nf.v[2]; nf.v[2] = s;
            phys3d__qh_plane(nf, pts);
        }
    }

    // ---- add points, one at a time ----
    bool ok = true;
    for (size_t pi = 0; pi < n && ok; pi++) {
        int p = (int)pi;
        if (p == i0 || p == i1 || p == i2 || p == i3) continue;
        int start = -1; float furthest = eps;
        for (int i = 0; i < fc; i++) {
            if (!f[i].alive) continue;
            float dist = v3dot(f[i].n, pts[p]) - f[i].d;
            if (dist > furthest) { furthest = dist; start = i; }
        }
        if (start < 0) continue;   // inside
        // flood fill the faces the point sees, through shared edges
        int sp = 0, vc = 0, hc = 0;
        memset(seen, 0, (size_t)fc);
        stack[sp++] = start;
        seen[start] = 1;
        while (sp) {
            int cur = stack[--sp];
            visible[vc++] = cur;
            for (int e = 0; e < 3; e++) {
                int a = f[cur].v[e], b = f[cur].v[(e + 1) % 3];
                int nb = phys3d__qh_owner(f, fc, b, a);
                if (nb < 0) { ok = false; break; }
                if (seen[nb] == 1) continue;
                if (seen[nb] == 2 || v3dot(f[nb].n, pts[p]) - f[nb].d <= eps) {
                    seen[nb] = 2;              // not visible: a -> b is on the border
                    horizon[hc * 2] = a; horizon[hc * 2 + 1] = b; hc++;
                } else {
                    seen[nb] = 1;
                    stack[sp++] = nb;
                }
            }
            if (!ok) break;
        }
        if (!ok || (size_t)(fc + hc) > cap) { ok = false; break; }
        for (int i = 0; i < vc; i++) f[visible[i]].alive = false;
        for (int e = 0; e < hc; e++) {
            phys3d__qh_face& nf = f[fc++];
            nf.v[0] = horizon[e * 2]; nf.v[1] = horizon[e * 2 + 1]; nf.v[2] = p;
            nf.alive = true;
            phys3d__qh_plane(nf, pts);
        }
        // compact now and then, so the scans stay proportional to the live hull
        if (fc > (int)cap / 2) {
            int k = 0;
            for (int i = 0; i < fc; i++) if (f[i].alive) f[k++] = f[i];
            fc = k;
        }
    }
    pix_free(stack); pix_free(visible); pix_free(horizon);
    if (!ok) { pix_free(f); pix_free(seen); pix_log("phys3d: hull construction failed (degenerate points?)\n"); return false; }
    int k = 0;
    for (int i = 0; i < fc; i++) if (f[i].alive) f[k++] = f[i];
    fc = k;

    // ---- merge coplanar neighbours into polygon faces ----
    uint32_t* group = pix_alloc<uint32_t>((size_t)fc);
    int* remap = pix_alloc<int>(n);
    int* tri_twin = pix_alloc<int>((size_t)fc * 3);
    if (!group || !remap || !tri_twin) { pix_free(f); pix_free(seen); pix_free(group); pix_free(remap); pix_free(tri_twin); return false; }
    for (int i = 0; i < fc; i++) group[i] = (uint32_t)i;
    for (int i = 0; i < fc; i++)
        for (int e = 0; e < 3; e++) {
            int t = phys3d__qh_owner(f, fc, f[i].v[(e + 1) % 3], f[i].v[e]);
            tri_twin[i * 3 + e] = t;
            if (t > i && v3dot(f[i].n, f[t].n) > 0.9999f) {
                uint32_t ra = phys3d__qh_root(group, (uint32_t)i), rb = phys3d__qh_root(group, (uint32_t)t);
                if (ra != rb) group[rb] = ra;
            }
        }
    for (int i = 0; i < fc; i++) group[i] = phys3d__qh_root(group, (uint32_t)i);
    // used vertices, compacted
    for (size_t i = 0; i < n; i++) remap[i] = -1;
    int vcount = 0;
    for (int i = 0; i < fc; i++)
        for (int e = 0; e < 3; e++) if (remap[f[i].v[e]] < 0) remap[f[i].v[e]] = vcount++;
    if (vcount > PHYS3D_MAX_HULL_VERTICES) {
        pix_log("phys3d: hull has %d corners, more than PHYS3D_MAX_HULL_VERTICES (%d)\n", vcount, PHYS3D_MAX_HULL_VERTICES);
        pix_free(f); pix_free(seen); pix_free(group); pix_free(remap); pix_free(tri_twin);
        return false;
    }
    int faces = 0;
    for (int i = 0; i < fc; i++) if (group[i] == (uint32_t)i) faces++;
    h.vertices = pix_alloc<vec3>((size_t)vcount);
    h.planes = pix_alloc<vec4>((size_t)faces);
    h.face_start = pix_alloc<int>((size_t)faces + 1);
    h.face_verts = pix_alloc<uint16_t>((size_t)fc * 3);
    h.edges = pix_alloc<phys3d_hull_edge>((size_t)fc * 3);
    if (!h.vertices || !h.planes || !h.face_start || !h.face_verts || !h.edges) {
        pix_free(f); pix_free(seen); pix_free(group); pix_free(remap); pix_free(tri_twin);
        phys3d_destroy_hull(h);
        return false;
    }
    for (size_t i = 0; i < n; i++) if (remap[i] >= 0) h.vertices[remap[i]] = pts[i];
    h.vertex_count = vcount;
    // each group's border, walked edge to edge into one loop
    int fv = 0;
    for (int g = 0; g < fc; g++) {
        if (group[g] != (uint32_t)g) continue;
        int face = h.face_count++;
        h.face_start[face] = fv;
        vec3 nsum = v3(0, 0, 0);
        int first_tri = -1, first_e = -1;
        for (int i = 0; i < fc; i++) {
            if (group[i] != (uint32_t)g) continue;
            nsum += f[i].n * v3len(v3cross(pts[f[i].v[1]] - pts[f[i].v[0]], pts[f[i].v[2]] - pts[f[i].v[0]]));
            for (int e = 0; e < 3 && first_tri < 0; e++) {
                int t = tri_twin[i * 3 + e];
                if (t < 0 || group[t] != (uint32_t)g) { first_tri = i; first_e = e; }
            }
        }
        vec3 nrm = v3norm(nsum);
        int start_v = f[first_tri].v[first_e];
        int cur_v = start_v;
        int guard = 0;
        do {
            // the border edge leaving cur_v
            int next_v = -1;
            for (int i = 0; i < fc && next_v < 0; i++) {
                if (group[i] != (uint32_t)g) continue;
                for (int e = 0; e < 3; e++) {
                    int t = tri_twin[i * 3 + e];
                    if (f[i].v[e] == cur_v && (t < 0 || group[t] != (uint32_t)g)) { next_v = f[i].v[(e + 1) % 3]; break; }
                }
            }
            if (next_v < 0 || fv >= fc * 3) break;
            h.face_verts[fv++] = (uint16_t)remap[cur_v];
            cur_v = next_v;
        } while (cur_v != start_v && ++guard < PHYS3D_MAX_FACE_VERTICES);
        if (fv - h.face_start[face] > PHYS3D_MAX_FACE_VERTICES) fv = h.face_start[face] + PHYS3D_MAX_FACE_VERTICES;
        // the plane through the face's furthest corner: conservative if not exactly flat
        float d = -1e30f;
        for (int i = h.face_start[face]; i < fv; i++) d = maxf(d, v3dot(nrm, h.vertices[h.face_verts[i]]));
        h.planes[face] = v4(nrm.x, nrm.y, nrm.z, d);
    }
    h.face_start[h.face_count] = fv;
    // edges: each border edge with the face on its other side, once
    for (int a = 0; a < h.face_count; a++)
        for (int i = h.face_start[a]; i < h.face_start[a + 1]; i++) {
            int u = h.face_verts[i];
            int v = h.face_verts[i + 1 < h.face_start[a + 1] ? i + 1 : h.face_start[a]];
            if (u > v) continue;   // the twin direction records it
            for (int b = 0; b < h.face_count; b++) {
                if (b == a) continue;
                bool found = false;
                for (int j = h.face_start[b]; j < h.face_start[b + 1] && !found; j++) {
                    int u2 = h.face_verts[j];
                    int v2 = h.face_verts[j + 1 < h.face_start[b + 1] ? j + 1 : h.face_start[b]];
                    if (u2 == v && v2 == u) found = true;
                }
                if (found) {
                    phys3d_hull_edge& e = h.edges[h.edge_count++];
                    e.v0 = (uint16_t)u; e.v1 = (uint16_t)v; e.f0 = (uint16_t)a; e.f1 = (uint16_t)b;
                    break;
                }
            }
        }
    pix_free(f); pix_free(seen); pix_free(group); pix_free(remap); pix_free(tri_twin);

    // ---- mass properties: tetrahedra from a point inside to every face ----
    vec3 ref = v3(0, 0, 0);
    for (int i = 0; i < h.vertex_count; i++) ref += h.vertices[i];
    ref = ref * (1.0f / h.vertex_count);
    double vol = 0.0, cx = 0, cy = 0, cz = 0, C[3][3] = {};
    for (int a = 0; a < h.face_count; a++) {
        int s = h.face_start[a], e = h.face_start[a + 1];
        for (int i = s + 1; i + 1 < e; i++) {
            vec3 p0 = h.vertices[h.face_verts[s]] - ref, p1 = h.vertices[h.face_verts[i]] - ref, p2 = h.vertices[h.face_verts[i + 1]] - ref;
            double det = v3dot(p0, v3cross(p1, p2));    // six times the tetrahedron's signed volume
            vol += det / 6.0;
            cx += det / 24.0 * (p0.x + p1.x + p2.x);
            cy += det / 24.0 * (p0.y + p1.y + p2.y);
            cz += det / 24.0 * (p0.z + p1.z + p2.z);
            // covariance of the tetrahedron (Blow & Binstock): det/120 * sum over vertex pairs
            const float* q[3] = { &p0.x, &p1.x, &p2.x };
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++) {
                    double sum = 0.0;
                    for (int x = 0; x < 3; x++) {
                        sum += 2.0 * q[x][r] * q[x][c];
                        for (int y = 0; y < 3; y++) if (y != x) sum += q[x][r] * q[y][c];
                    }
                    C[r][c] += det / 120.0 * sum;
                }
        }
    }
    if (vol <= 1e-12) { phys3d_destroy_hull(h); return false; }
    vec3 com = v3((float)(cx / vol), (float)(cy / vol), (float)(cz / vol));
    // move the covariance to the centre of mass, then I = trace(C) * 1 - C
    double cm[3] = { com.x, com.y, com.z };
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) C[r][c] -= vol * cm[r] * cm[c];
    double tr = C[0][0] + C[1][1] + C[2][2];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) h.inertia.data[c * 3 + r] = (float)((r == c ? tr : 0.0) - C[r][c]);
    h.volume = (float)vol;
    h.offset = ref + com;
    h.bounds = aabb_make(v3(1e30f, 1e30f, 1e30f), v3(-1e30f, -1e30f, -1e30f));
    for (int i = 0; i < h.vertex_count; i++) {
        h.vertices[i] = h.vertices[i] - h.offset;
        h.bounds.min = v3min(h.bounds.min, h.vertices[i]);
        h.bounds.max = v3max(h.bounds.max, h.vertices[i]);
    }
    for (int a = 0; a < h.face_count; a++) {
        vec4& pl = h.planes[a];
        pl.w -= pl.x * h.offset.x + pl.y * h.offset.y + pl.z * h.offset.z;
    }
    return true;
}

void phys3d_destroy_hull(phys3d_hull& h) {
    pix_free(h.vertices);
    pix_free(h.planes);
    pix_free(h.face_start);
    pix_free(h.face_verts);
    pix_free(h.edges);
    memset(&h, 0, sizeof(h));
}

// ---- compounds ----

static void phys3d__shape_mass(const phys3d_shape& s, float d, float* out_mass, mat3* out_inertia);

// a radius around the shape's centre that holds all of it
static float phys3d__shape_reach(const phys3d_shape& s) {
    if (s.type == PHYS3D_SPHERE) return s.radius;
    if (s.type == PHYS3D_CAPSULE) return s.radius + s.half_length;
    if (s.type == PHYS3D_BOX) return v3len(s.half);
    if (s.type == PHYS3D_HULL && s.hull) {
        float r = 0.0f;
        for (int i = 0; i < s.hull->vertex_count; i++) r = maxf(r, v3len(s.hull->vertices[i]));
        return r;
    }
    return 0.0f;
}

bool phys3d_create_compound(phys3d_compound& c, const phys3d_compound_child* children, int count) {
    memset(&c, 0, sizeof(c));
    if (!children || count <= 0) return false;
    for (int i = 0; i < count; i++) {
        uint8_t t = children[i].shape.type;
        if (t != PHYS3D_SPHERE && t != PHYS3D_BOX && t != PHYS3D_CAPSULE && !(t == PHYS3D_HULL && children[i].shape.hull)) {
            pix_log("phys3d: compound child %d must be a sphere, box, capsule or hull\n", i);
            return false;
        }
    }
    c.children = pix_alloc<phys3d_compound_child>((size_t)count);
    if (!c.children) return false;
    c.count = count;
    // the parts' masses at unit density: the centre of mass, then every part's
    // inertia turned into the compound's frame and moved there (parallel axes)
    float total = 0.0f;
    vec3 com = v3(0, 0, 0);
    for (int i = 0; i < count; i++) {
        c.children[i] = children[i];
        c.children[i].rotation = quat_norm(children[i].rotation);
        float m; mat3 I;
        phys3d__shape_mass(children[i].shape, 1.0f, &m, &I);
        // a hull part's own centre is its centre of mass, `offset` from its points' origin
        vec3 at = children[i].position + quat_rotate(c.children[i].rotation, phys3d_shape_offset(children[i].shape));
        c.children[i].position = at;
        total += m;
        com += at * m;
    }
    if (total <= 0.0f) { phys3d_destroy_compound(c); return false; }
    com = com * (1.0f / total);
    mat3 I = {};
    for (int i = 0; i < count; i++) {
        float m; mat3 Ic;
        phys3d__shape_mass(c.children[i].shape, 1.0f, &m, &Ic);
        mat3 r = mat3_from_quat(c.children[i].rotation);
        mat3 rot = r * Ic * mat3_transpose(r);
        vec3 d = c.children[i].position - com;
        float dd = v3dot(d, d);
        const float dv[3] = { d.x, d.y, d.z };
        for (int row = 0; row < 3; row++)
            for (int col = 0; col < 3; col++)
                I.data[col * 3 + row] += rot.data[col * 3 + row] + m * ((row == col ? dd : 0.0f) - dv[row] * dv[col]);
        c.children[i].position = d;   // re-centred
    }
    c.bounds = aabb_make(v3(1e30f, 1e30f, 1e30f), v3(-1e30f, -1e30f, -1e30f));
    for (int i = 0; i < count; i++) {
        float reach = phys3d__shape_reach(c.children[i].shape);
        vec3 r = v3(reach, reach, reach);
        c.bounds.min = v3min(c.bounds.min, c.children[i].position - r);
        c.bounds.max = v3max(c.bounds.max, c.children[i].position + r);
    }
    c.volume = total;
    c.inertia = I;
    c.offset = com;
    return true;
}

void phys3d_destroy_compound(phys3d_compound& c) {
    pix_free(c.children);
    memset(&c, 0, sizeof(c));
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
#define PHYS3D__HULL     4
#define PHYS3D__PLANE    5

struct phys3d__core {
    uint8_t kind;
    int     count;        // vertices in v (a hull's: hull->vertex_count, read through phys3d__vert)
    vec3    v[8];
    vec3    centre;
    vec3    axis[3];      // box: its axes; triangle: axis[0] is the face normal
    vec3    half;
    float   radius;
    float   offset;       // plane: dot(normal, p) == offset; normal in axis[0]
    quat    q;            // hull: its rotation (centre is its position)
    const phys3d_hull* hull;
};

static inline vec3 phys3d__vert(const phys3d__core& c, int i) {
    return c.kind == PHYS3D__HULL ? c.centre + quat_rotate(c.q, c.hull->vertices[i]) : c.v[i];
}

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
    } else if (s.type == PHYS3D_HULL) {
        c.kind = PHYS3D__HULL; c.hull = s.hull; c.q = q;
        c.count = s.hull->vertex_count;
    }
}

// Every convex piece of a body as a core, with its part number: one for a
// plain shape, one per child for a compound.
template <typename F>
static void phys3d__for_each_core(const phys3d_shape& s, vec3 p, quat q, F&& visit) {
    phys3d__core c;
    if (s.type != PHYS3D_COMPOUND) { phys3d__make_core(s, p, q, c); visit(c, 0u); return; }
    for (int i = 0; i < s.compound->count; i++) {
        const phys3d_compound_child& ch = s.compound->children[i];
        phys3d__make_core(ch.shape, p + quat_rotate(q, ch.position), quat_mul(q, ch.rotation), c);
        visit(c, (uint32_t)i);
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
    if (c.kind == PHYS3D__HULL) {
        // the local box turned with it: a little loose, but eight points instead of all of them
        const aabb& lb = c.hull->bounds;
        aabb b = aabb_make(v3(1e30f, 1e30f, 1e30f), v3(-1e30f, -1e30f, -1e30f));
        for (int i = 0; i < 8; i++) {
            vec3 p = c.centre + quat_rotate(c.q, v3((i & 1) ? lb.max.x : lb.min.x, (i & 2) ? lb.max.y : lb.min.y,
                                                   (i & 4) ? lb.max.z : lb.min.z));
            b.min = v3min(b.min, p); b.max = v3max(b.max, p);
        }
        return b;
    }
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
    if (c.kind == PHYS3D__HULL) {
        vec3 ld = quat_rotate(quat_conjugate(c.q), d);
        const vec3* v = c.hull->vertices;
        int best = 0;
        float bd = v3dot(v[0], ld);
        for (int i = 1; i < c.count; i++) { float x = v3dot(v[i], ld); if (x > bd) { bd = x; best = i; } }
        return c.centre + quat_rotate(c.q, v[best]);
    }
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
    if (c.kind == PHYS3D__HULL) {
        const phys3d_hull& h = *c.hull;
        *n = quat_rotate(c.q, v3(h.planes[f].x, h.planes[f].y, h.planes[f].z));
        int k = 0;
        for (int i = h.face_start[f]; i < h.face_start[f + 1]; i++) poly[k++] = c.centre + quat_rotate(c.q, h.vertices[h.face_verts[i]]);
        return k;
    }
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
static inline int phys3d__face_count(const phys3d__core& c) {
    return c.kind == PHYS3D__HULL ? c.hull->face_count : (c.kind == PHYS3D__TRIANGLE ? 1 : 6);
}

// Clips polygon `in` (n points) to the reference face's side planes, then
// keeps the points within `margin` of the face. `ref_n` points out of the
// reference shape; `flip` means the reference is b, so the a->b normal is -ref_n.
#define PHYS3D__CLIP_MAX (2 * PHYS3D_MAX_FACE_VERTICES + 8)   // each clipping plane adds at most one corner
static int phys3d__clip_to_face(const vec3* ref_poly, int ref_n_count, vec3 ref_n, const vec3* in, int n,
                                float r_ref, float r_inc, bool flip, float margin, phys3d_contact& m) {
    vec3 buf_a[PHYS3D__CLIP_MAX], buf_b[PHYS3D__CLIP_MAX];
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
                if ((da < 0.0f) != (db < 0.0f) && k < PHYS3D__CLIP_MAX) next[k++] = a + (b - a) * (da / (da - db));
                if (k >= PHYS3D__CLIP_MAX) break;
            }
        }
        vec3* t = cur; cur = next; next = t;
        n = k;
    }
    vec3 pts[PHYS3D__CLIP_MAX]; float seps[PHYS3D__CLIP_MAX];
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
    vec3 poly[PHYS3D_MAX_FACE_VERTICES], fn;
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

// ---- hulls against boxes, triangles and hulls ----
//
// Each shape seen the same way, in world space: corners, face planes and
// polygons, and edges with the face either side. Candidate separating axes are
// every face normal of both, and the cross product of every edge pair - but an
// edge pair can only separate the shapes when its two arcs cross on the Gauss
// map (it is then a face of the Minkowski difference), which rules out nearly
// all of them cheaply (Gregorius, "The Separating Axis Test between Convex
// Polyhedra", GDC 2013). A triangle skips that test: its two sides face
// opposite ways, which the arc test cannot handle, and it has three edges.

struct phys3d__poly_view {
    const vec3* v; int nv;
    const vec3* n; const float* d; int nf;
    const int* fstart; const uint16_t* fverts;
    const phys3d_hull_edge* e; int ne;
    bool prune;
    vec3 centre;
};

// world-space copies of the two shapes in a test (thread-local: not on the stack)
static thread_local vec3  phys3d__pv_verts[2][PHYS3D_MAX_HULL_VERTICES];
static thread_local vec3  phys3d__pv_norms[2][2 * PHYS3D_MAX_HULL_VERTICES];
static thread_local float phys3d__pv_offs[2][2 * PHYS3D_MAX_HULL_VERTICES];

// a box's faces in phys3d__face's order (+x, -x, +y, -y, +z, -z) and its edges
static int phys3d__box_fstart[7];
static uint16_t phys3d__box_fverts[24];
static phys3d_hull_edge phys3d__box_edges[12];
static const phys3d_hull_edge phys3d__tri_edges[3] = { { 0, 1, 0, 0 }, { 1, 2, 0, 0 }, { 2, 0, 0, 0 } };
static const int phys3d__tri_fstart[2] = { 0, 3 };
static const uint16_t phys3d__tri_fverts[3] = { 0, 1, 2 };

static void phys3d__box_tables() {
    static bool done = false;
    if (done) return;
    int fv = 0, ec = 0;
    for (int f = 0; f < 6; f++) {
        int i = f >> 1, j = (i + 1) % 3, k = (i + 2) % 3;
        int side = (f & 1) ? 0 : 1;
        static const int cycle[4][2] = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };
        phys3d__box_fstart[f] = fv;
        for (int c = 0; c < 4; c++)
            phys3d__box_fverts[fv++] = (uint16_t)((side << i) | (cycle[c][0] << j) | (cycle[c][1] << k));
    }
    phys3d__box_fstart[6] = fv;
    for (int e = 0; e < 3; e++) {
        int j = (e + 1) % 3, k = (e + 2) % 3;
        for (int bj = 0; bj < 2; bj++)
            for (int bk = 0; bk < 2; bk++) {
                phys3d_hull_edge& ed = phys3d__box_edges[ec++];
                ed.v0 = (uint16_t)((bj << j) | (bk << k));
                ed.v1 = (uint16_t)(ed.v0 | (1 << e));
                ed.f0 = (uint16_t)(2 * j + (bj ? 0 : 1));
                ed.f1 = (uint16_t)(2 * k + (bk ? 0 : 1));
            }
    }
    done = true;
}

static phys3d__poly_view phys3d__make_view(const phys3d__core& c, int slot) {
    phys3d__poly_view v = {};
    v.centre = c.centre;
    if (c.kind == PHYS3D__HULL) {
        const phys3d_hull& h = *c.hull;
        for (int i = 0; i < h.vertex_count; i++) phys3d__pv_verts[slot][i] = c.centre + quat_rotate(c.q, h.vertices[i]);
        for (int f = 0; f < h.face_count; f++) {
            vec3 n = quat_rotate(c.q, v3(h.planes[f].x, h.planes[f].y, h.planes[f].z));
            phys3d__pv_norms[slot][f] = n;
            phys3d__pv_offs[slot][f] = h.planes[f].w + v3dot(n, c.centre);
        }
        v.v = phys3d__pv_verts[slot]; v.nv = h.vertex_count;
        v.n = phys3d__pv_norms[slot]; v.d = phys3d__pv_offs[slot]; v.nf = h.face_count;
        v.fstart = h.face_start; v.fverts = h.face_verts;
        v.e = h.edges; v.ne = h.edge_count;
        v.prune = true;
    } else if (c.kind == PHYS3D__BOX) {
        phys3d__box_tables();
        const float hh[3] = { c.half.x, c.half.y, c.half.z };
        for (int f = 0; f < 6; f++) {
            vec3 n = c.axis[f >> 1] * ((f & 1) ? -1.0f : 1.0f);
            phys3d__pv_norms[slot][f] = n;
            phys3d__pv_offs[slot][f] = v3dot(n, c.centre) + hh[f >> 1];
        }
        v.v = c.v; v.nv = 8;
        v.n = phys3d__pv_norms[slot]; v.d = phys3d__pv_offs[slot]; v.nf = 6;
        v.fstart = phys3d__box_fstart; v.fverts = phys3d__box_fverts;
        v.e = phys3d__box_edges; v.ne = 12;
        v.prune = true;
    } else {   // triangle: its front only
        phys3d__pv_norms[slot][0] = c.axis[0];
        phys3d__pv_offs[slot][0] = v3dot(c.axis[0], c.v[0]);
        v.v = c.v; v.nv = 3;
        v.n = phys3d__pv_norms[slot]; v.d = phys3d__pv_offs[slot]; v.nf = 1;
        v.fstart = phys3d__tri_fstart; v.fverts = phys3d__tri_fverts;
        v.e = phys3d__tri_edges; v.ne = 3;
        v.prune = false;
    }
    return v;
}

// do edge (a, b normals) of A and edge (c, d normals) of B form a face of the
// Minkowski difference? (B's normals are negated by the caller)
static inline bool phys3d__minkowski_face(vec3 a, vec3 b, vec3 c, vec3 d) {
    vec3 bxa = v3cross(b, a), dxc = v3cross(d, c);
    float cba = v3dot(c, bxa), dba = v3dot(d, bxa), adc = v3dot(a, dxc), bdc = v3dot(b, dxc);
    return cba * dba < 0.0f && adc * bdc < 0.0f && cba * bdc > 0.0f;
}

static int phys3d__view_poly(const phys3d__poly_view& v, int f, vec3* out) {
    int k = 0;
    for (int i = v.fstart[f]; i < v.fstart[f + 1]; i++) out[k++] = v.v[v.fverts[i]];
    return k;
}

static int phys3d__collide_views(const phys3d__poly_view& A, const phys3d__poly_view& B, float slop, float margin,
                                 phys3d_contact& m) {
    m.count = 0;
    float sep_a = -1e30f, sep_b = -1e30f;
    int face_a = 0, face_b = 0;
    for (int f = 0; f < A.nf; f++) {
        float s = 1e30f;
        for (int i = 0; i < B.nv; i++) s = minf(s, v3dot(A.n[f], B.v[i]));
        s -= A.d[f];
        if (s > margin) return 0;
        if (s > sep_a) { sep_a = s; face_a = f; }
    }
    for (int f = 0; f < B.nf; f++) {
        float s = 1e30f;
        for (int i = 0; i < A.nv; i++) s = minf(s, v3dot(B.n[f], A.v[i]));
        s -= B.d[f];
        if (s > margin) return 0;
        if (s > sep_b) { sep_b = s; face_b = f; }
    }
    float sep_e = -1e30f;
    int edge_i = -1, edge_j = -1;
    vec3 edge_axis = v3(0, 1, 0);
    bool prune = A.prune && B.prune;
    vec3 dcent = B.centre - A.centre;
    for (int i = 0; i < A.ne; i++) {
        const phys3d_hull_edge& ea = A.e[i];
        vec3 pa = A.v[ea.v0], da = A.v[ea.v1] - pa;
        for (int j = 0; j < B.ne; j++) {
            const phys3d_hull_edge& eb = B.e[j];
            if (prune && !phys3d__minkowski_face(A.n[ea.f0], A.n[ea.f1], -B.n[eb.f0], -B.n[eb.f1])) continue;
            vec3 pb = B.v[eb.v0], db = B.v[eb.v1] - pb;
            vec3 ax = v3cross(da, db);
            float l2 = v3dot(ax, ax);
            if (l2 < 1e-8f * v3dot(da, da) * v3dot(db, db)) continue;   // parallel edges
            ax = ax / sqrtf(l2);
            float s;
            if (prune) {
                // a Minkowski face: the distance between the two edges' lines along it
                if (v3dot(ax, pa - A.centre) < 0.0f) ax = -ax;
                s = v3dot(ax, pb - pa);
            } else {
                if (v3dot(ax, dcent) < 0.0f) ax = -ax;
                float amax = -1e30f, bmin = 1e30f;
                for (int k = 0; k < A.nv; k++) amax = maxf(amax, v3dot(ax, A.v[k]));
                for (int k = 0; k < B.nv; k++) bmin = minf(bmin, v3dot(ax, B.v[k]));
                s = bmin - amax;
            }
            if (s > margin) return 0;
            if (s > sep_e) { sep_e = s; edge_i = i; edge_j = j; edge_axis = ax; }
        }
    }
    // faces win near-ties, as for boxes: no flicker between a face and an edge normal
    float face_best = maxf(sep_a, sep_b);
    if (edge_i >= 0 && sep_e > face_best + 0.5f * slop) {
        vec3 ca, cb;
        closest_points_segments(A.v[A.e[edge_i].v0], A.v[A.e[edge_i].v1], B.v[B.e[edge_j].v0], B.v[B.e[edge_j].v1], &ca, &cb);
        m.normal = edge_axis;
        phys3d__add_point(m, (ca + cb) * 0.5f, sep_e);
        return 1;
    }
    bool flip = sep_b > sep_a + 0.1f * slop;
    const phys3d__poly_view& ref = flip ? B : A;
    const phys3d__poly_view& inc = flip ? A : B;
    int rf = flip ? face_b : face_a;
    vec3 rn = ref.n[rf];
    vec3 ref_poly[PHYS3D_MAX_FACE_VERTICES], inc_poly[PHYS3D_MAX_FACE_VERTICES];
    int rc = phys3d__view_poly(ref, rf, ref_poly);
    int best = 0;
    float lowest = 2.0f;
    for (int f = 0; f < inc.nf; f++) {
        float x = v3dot(inc.n[f], rn);
        if (x < lowest) { lowest = x; best = f; }
    }
    int ic = phys3d__view_poly(inc, best, inc_poly);
    return phys3d__clip_to_face(ref_poly, rc, rn, inc_poly, ic, 0.0f, 0.0f, flip, margin, m);
}

// any core against an infinite plane (b)
static int phys3d__collide_plane(const phys3d__core& a, const phys3d__core& p, float margin, phys3d_contact& m) {
    vec3 n = p.axis[0];
    vec3 pts[PHYS3D_MAX_HULL_VERTICES]; float seps[PHYS3D_MAX_HULL_VERTICES];
    int count = 0;
    for (int i = 0; i < a.count; i++) {
        vec3 v = phys3d__vert(a, i);
        float sep = v3dot(n, v) - p.offset - a.radius;
        if (sep > margin) continue;
        pts[count] = v - n * (a.radius + 0.5f * sep);
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
    if (b.kind == PHYS3D__HULL)   // with a box, triangle or hull
        return phys3d__collide_views(phys3d__make_view(a, 0), phys3d__make_view(b, 1), slop, margin, m);
    return phys3d__collide_polys(a, b, slop, margin, m);
}

// ---- mass ----

// A shape's mass and inertia tensor about its own centre, in its own frame.
static void phys3d__shape_mass(const phys3d_shape& s, float d, float* out_mass, mat3* out_inertia) {
    float mass = 0.0f;
    mat3 I = {};
    if (s.type == PHYS3D_SPHERE) {
        mass = d * (4.0f / 3.0f) * PIX_PI * s.radius * s.radius * s.radius;
        float i = 0.4f * mass * s.radius * s.radius;
        I = mat3_scale(i, i, i);
    } else if (s.type == PHYS3D_CAPSULE) {
        // a cylinder plus a sphere split into two caps pushed out to the ends
        float r = s.radius, h = 2.0f * s.half_length, rr = r * r;
        float mc = d * PIX_PI * rr * h, ms = d * (4.0f / 3.0f) * PIX_PI * rr * r;
        mass = mc + ms;
        float iy = mc * rr * 0.5f + ms * rr * 0.4f;
        float ix = mc * (h * h / 12.0f + rr * 0.25f) + ms * (rr * 0.4f + h * h * 0.25f + 0.375f * h * r);
        I = mat3_scale(ix, iy, ix);
    } else if (s.type == PHYS3D_BOX) {
        vec3 h = s.half;
        mass = d * 8.0f * h.x * h.y * h.z;
        I = mat3_scale((h.y * h.y + h.z * h.z) * mass / 3.0f, (h.x * h.x + h.z * h.z) * mass / 3.0f,
                       (h.x * h.x + h.y * h.y) * mass / 3.0f);
    } else if (s.type == PHYS3D_HULL && s.hull) {
        mass = d * s.hull->volume;
        for (int k = 0; k < 9; k++) I.data[k] = s.hull->inertia.data[k] * d;
    } else if (s.type == PHYS3D_COMPOUND && s.compound) {
        mass = d * s.compound->volume;
        for (int k = 0; k < 9; k++) I.data[k] = s.compound->inertia.data[k] * d;
    }
    *out_mass = mass;
    *out_inertia = I;
}

static void phys3d__compute_mass(phys3d_body& b) {
    b.inv_mass = 0.0f;
    b.inv_inertia_local = mat3();
    if (b.type != PHYS3D_DYNAMIC) return;
    float mass;
    mat3 inertia;
    phys3d__shape_mass(b.shape, b.density > 0.0f ? b.density : 1.0f, &mass, &inertia);
    if (mass > 0.0f) b.inv_mass = 1.0f / mass;
    if (!b.fixed_rotation && mat3_determinant(inertia) > 1e-12f * mass * mass * mass)
        b.inv_inertia_local = mat3_inverse(inertia);
}

// R * M * R^T: a body's inverse inertia turned into world space
static inline void phys3d__update_inertia(phys3d_body& b) {
    mat3 r = mat3_from_quat(b.rotation);
    b.inv_inertia = r * b.inv_inertia_local * mat3_transpose(r);
}

// ---- world ----

void phys3d_create_world(phys3d_world& w) {
    memset(&w, 0, sizeof(w));
    w.continuous = true;
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
    for (int k = 0; k < 6; k++) j.axis_impulse[k] = j.axis_motor_impulse[k] = 0.0f;
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

idx phys3d_add_wheel(phys3d_world& w, idx chassis, idx wheel, vec3 centre, vec3 suspension, vec3 spin) {
    phys3d_joint j = phys3d__joint_base(w, PHYS3D_JOINT_WHEEL, chassis, wheel, centre, centre, suspension);
    spin = v3dot(spin, spin) > 1e-12f ? v3norm(spin) : v3(1, 0, 0);
    j.local_ref_a = phys3d__dir_to_local(phys3d__jbody(w, chassis), spin);   // the axle in the chassis
    j.local_axis_b = phys3d__dir_to_local(phys3d__jbody(w, wheel), spin);    // and in the wheel
    j.enable_spring = true;
    j.hertz = 5.0f;
    j.damping_ratio = 0.7f;
    return phys3d_add_joint(w, j);
}
idx phys3d_add_motor_joint(phys3d_world& w, idx a, idx b, float max_force, float max_torque) {
    const phys3d_body& A = phys3d__jbody(w, a);
    const phys3d_body& B = phys3d__jbody(w, b);
    phys3d_joint j = phys3d__joint_base(w, PHYS3D_JOINT_MOTOR, a, b, B.position, B.position, v3(0, 1, 0));
    j.local_a = j.local_b = v3(0, 0, 0);
    j.target = phys3d__to_local(A, B.position);
    j.max_force = max_force;
    j.max_motor_force = max_torque;
    return phys3d_add_joint(w, j);
}
idx phys3d_add_gear(phys3d_world& w, idx a, idx b, vec3 axis_a, vec3 axis_b, float ratio) {
    phys3d_joint j = phys3d__joint_base(w, PHYS3D_JOINT_GEAR, a, b, phys3d__jbody(w, a).position, phys3d__jbody(w, b).position, axis_a);
    j.local_axis_b = phys3d__dir_to_local(phys3d__jbody(w, b), v3norm(axis_b));
    j.ratio = ratio;
    j.collide_connected = true;   // geared parts are separate things that may touch
    return phys3d_add_joint(w, j);
}
idx phys3d_add_pulley(phys3d_world& w, idx a, idx b, vec3 ground_a, vec3 ground_b, vec3 anchor_a, vec3 anchor_b, float ratio) {
    phys3d_joint j = phys3d__joint_base(w, PHYS3D_JOINT_PULLEY, a, b, anchor_a, anchor_b, v3(0, 1, 0));
    j.ground_a = ground_a;
    j.ground_b = ground_b;
    j.ratio = ratio > 1e-3f ? ratio : 1e-3f;
    j.length = v3dist(anchor_a, ground_a) + j.ratio * v3dist(anchor_b, ground_b);
    j.collide_connected = true;
    return phys3d_add_joint(w, j);
}
idx phys3d_add_generic(phys3d_world& w, idx a, idx b, vec3 anchor, quat frame) {
    phys3d_joint j = phys3d__joint_base(w, PHYS3D_JOINT_GENERIC, a, b, anchor, anchor, v3(0, 1, 0));
    frame = quat_norm(frame);
    j.frame_a = quat_mul(quat_conjugate(phys3d__jbody(w, a).rotation), frame);
    j.frame_b = quat_mul(quat_conjugate(phys3d__jbody(w, b).rotation), frame);
    for (int i = 0; i < 6; i++) j.axis_mode[i] = PHYS3D_AXIS_LOCKED;
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

// one equality row, soft: drives C to 0 and the row's speed to 0
static float phys3d__row_solve(phys3d_body& A, phys3d_body& B, vec3 lin, vec3 angA, vec3 angB, float C, phys3d__soft s, float* acc) {
    float m = phys3d__row_mass(A, B, lin, angA, angB);
    if (m <= 0.0f) return 0.0f;
    float imp = -m * s.mass_scale * (phys3d__row_speed(A, B, lin, angA, angB) + s.bias_rate * C) - s.impulse_scale * *acc;
    *acc += imp;
    phys3d__apply_row(A, B, lin, angA, angB, imp);
    return imp;
}
// a motor on one row: toward `speed`, within +-max_impulse in total
static void phys3d__row_motor(phys3d_body& A, phys3d_body& B, vec3 lin, vec3 angA, vec3 angB, float speed, float max_impulse, float* acc) {
    float m = phys3d__row_mass(A, B, lin, angA, angB);
    if (m <= 0.0f) return;
    float old = *acc;
    *acc = clampf(old + m * (speed - phys3d__row_speed(A, B, lin, angA, angB)), -max_impulse, max_impulse);
    phys3d__apply_row(A, B, lin, angA, angB, *acc - old);
}
// both limits of one row; `acc` collects lower - upper
static void phys3d__row_limits(phys3d_body& A, phys3d_body& B, vec3 lin, vec3 angA, vec3 angB, float value, float lower, float upper,
                               float inv_h, phys3d__soft rigid, float* lower_acc, float* upper_acc) {
    float m = phys3d__row_mass(A, B, lin, angA, angB);
    if (m <= 0.0f) return;
    float imp = phys3d__limit(value - lower, phys3d__row_speed(A, B, lin, angA, angB), m, inv_h, rigid, lower_acc);
    phys3d__apply_row(A, B, lin, angA, angB, imp);
    imp = phys3d__limit(upper - value, -phys3d__row_speed(A, B, lin, angA, angB), m, inv_h, rigid, upper_acc);
    phys3d__apply_row(A, B, lin, angA, angB, -imp);
}

// The generic joint's angles: b's frame seen from a's as X-Y-Z Euler angles,
// and the three axes those angles turn about (Bullet's btGeneric6DofConstraint).
static void phys3d__generic_angles(const phys3d_body& A, const phys3d_body& B, const phys3d_joint& j, vec3 axes[3], float angles[3]) {
    mat3 fa = mat3_from_quat(quat_mul(A.rotation, j.frame_a)), fb = mat3_from_quat(quat_mul(B.rotation, j.frame_b));
    mat3 rel = mat3_transpose(fa) * fb;
    float m02 = phys3d__m(rel, 0, 2);
    if (m02 < 1.0f) {
        if (m02 > -1.0f) {
            angles[0] = atan2f(-phys3d__m(rel, 1, 2), phys3d__m(rel, 2, 2));
            angles[1] = asinf(m02);
            angles[2] = atan2f(-phys3d__m(rel, 0, 1), phys3d__m(rel, 0, 0));
        } else {   // gimbal lock
            angles[0] = -atan2f(phys3d__m(rel, 1, 0), phys3d__m(rel, 1, 1));
            angles[1] = -0.5f * PIX_PI;
            angles[2] = 0.0f;
        }
    } else {
        angles[0] = atan2f(phys3d__m(rel, 1, 0), phys3d__m(rel, 1, 1));
        angles[1] = 0.5f * PIX_PI;
        angles[2] = 0.0f;
    }
    vec3 axis0 = mat3_col(fb, 0), axis2 = mat3_col(fa, 2);
    axes[1] = v3cross(axis2, axis0);
    axes[0] = v3cross(axes[1], axis2);
    axes[2] = v3cross(axis0, axes[1]);
    for (int i = 0; i < 3; i++) axes[i] = v3dot(axes[i], axes[i]) > 1e-12f ? v3norm(axes[i]) : v3(0, 0, 0);
}

// the wheel's axle as the chassis wants it: its spin axis turned by the steering
static inline vec3 phys3d__wheel_axle(const phys3d_body& A, const phys3d_joint& j) {
    vec3 axle = quat_rotate(quat_from_axis_angle(j.local_axis_a, j.steer), j.local_ref_a);
    return quat_rotate(A.rotation, axle);
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
        case PHYS3D_JOINT_WHEEL: {
            vec3 dra = j.d + j.ra;
            phys3d__apply_row(A, B, j.perp1, v3cross(dra, j.perp1), v3cross(j.rb, j.perp1), j.linear_impulse.x);
            phys3d__apply_row(A, B, j.perp2, v3cross(dra, j.perp2), v3cross(j.rb, j.perp2), j.linear_impulse.y);
            phys3d__apply_row(A, B, j.axis, v3cross(dra, j.axis), v3cross(j.rb, j.axis), j.axial_impulse + j.lower_impulse - j.upper_impulse);
            vec3 axle = phys3d__wheel_axle(A, j), q1, q2;
            phys3d__basis(axle, &q1, &q2);
            vec3 spin = quat_rotate(B.rotation, j.local_axis_b);
            phys3d__apply_angular(A, B, v3cross(spin, q1) * j.angular_impulse.x + v3cross(spin, q2) * j.angular_impulse.y
                                        + spin * j.motor_impulse);
        } break;
        case PHYS3D_JOINT_MOTOR:
            phys3d__apply_at(A, B, v3(0, 0, 0), v3(0, 0, 0), j.linear_impulse);
            phys3d__apply_angular(A, B, j.angular_impulse);
            break;
        case PHYS3D_JOINT_GEAR: {
            vec3 axa = quat_rotate(A.rotation, j.local_axis_a), axb = quat_rotate(B.rotation, j.local_axis_b);
            phys3d__apply_row(A, B, v3(0, 0, 0), axa * -j.ratio, axb, j.axial_impulse);
        } break;
        case PHYS3D_JOINT_PULLEY: {
            vec3 pa = A.position + j.ra, pb = B.position + j.rb;
            vec3 ua = v3norm(pa - j.ground_a), ub = v3norm(pb - j.ground_b);
            float imp = j.axial_impulse;
            A.velocity += ua * (imp * A.inv_mass);
            A.angular_velocity += A.inv_inertia * (v3cross(j.ra, ua) * imp);
            B.velocity += ub * (imp * j.ratio * B.inv_mass);
            B.angular_velocity += B.inv_inertia * (v3cross(j.rb, ub) * (imp * j.ratio));
        } break;
        case PHYS3D_JOINT_GENERIC: {
            vec3 dra = j.d + j.ra, axes[3];
            float angles[3];
            mat3 fa = mat3_from_quat(quat_mul(A.rotation, j.frame_a));
            for (int k = 0; k < 3; k++) {
                vec3 ax = mat3_col(fa, k);
                phys3d__apply_row(A, B, ax, v3cross(dra, ax), v3cross(j.rb, ax), j.axis_impulse[k] + j.axis_motor_impulse[k]);
            }
            phys3d__generic_angles(A, B, j, axes, angles);
            for (int k = 0; k < 3; k++) phys3d__apply_angular(A, B, axes[k] * (j.axis_impulse[3 + k] + j.axis_motor_impulse[3 + k]));
        } break;
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
        } else if (j.type == PHYS3D_JOINT_WHEEL) {
            vec3 dra = j.d + j.ra;
            vec3 axle = phys3d__wheel_axle(A, j), q1, q2;
            phys3d__basis(axle, &q1, &q2);
            vec3 spin = quat_rotate(B.rotation, j.local_axis_b);
            // drive: about the wheel's own axle
            if (j.enable_motor)
                phys3d__row_motor(A, B, v3(0, 0, 0), spin, spin, j.motor_speed, j.max_motor_force * h, &j.motor_impulse);
            // suspension: a spring along its axis (rigid without one), and its travel
            vec3 sA = v3cross(dra, j.axis), sB = v3cross(j.rb, j.axis);
            phys3d__soft spring = j.enable_spring && j.hertz > 0.0f ? phys3d__make_soft(j.hertz, j.damping_ratio, h) : rigid;
            phys3d__row_solve(A, B, j.axis, sA, sB, v3dot(j.d, j.axis), spring, &j.axial_impulse);
            if (j.enable_limit)
                phys3d__row_limits(A, B, j.axis, sA, sB, v3dot(j.d, j.axis), j.lower, j.upper, inv_h, rigid, &j.lower_impulse, &j.upper_impulse);
            // the axle stays where the steering puts it: two angular rows, free to spin
            phys3d__row_solve(A, B, v3(0, 0, 0), v3cross(spin, q1), v3cross(spin, q1), v3dot(spin, q1), rigid, &j.angular_impulse.x);
            phys3d__row_solve(A, B, v3(0, 0, 0), v3cross(spin, q2), v3cross(spin, q2), v3dot(spin, q2), rigid, &j.angular_impulse.y);
            // and the wheel's centre stays on the suspension line
            phys3d__row_solve(A, B, j.perp1, v3cross(dra, j.perp1), v3cross(j.rb, j.perp1), v3dot(j.d, j.perp1), rigid, &j.linear_impulse.x);
            phys3d__row_solve(A, B, j.perp2, v3cross(dra, j.perp2), v3cross(j.rb, j.perp2), v3dot(j.d, j.perp2), rigid, &j.linear_impulse.y);
        } else if (j.type == PHYS3D_JOINT_MOTOR) {
            // toward the target pose, the total held within the force and torque caps
            phys3d__soft soft = phys3d__make_soft(j.hertz > 0.0f ? j.hertz : 5.0f, j.damping_ratio, h);
            vec3 L = phys3d__lock_impulse(A, B, j.relative_rotation, soft, j.angular_impulse);
            vec3 oldL = j.angular_impulse;
            j.angular_impulse += L;
            float maxL = j.max_motor_force * h, lenL = v3len(j.angular_impulse);
            if (lenL > maxL) j.angular_impulse *= maxL / lenL;
            phys3d__apply_angular(A, B, j.angular_impulse - oldL);
            vec3 C = B.position - (A.position + quat_rotate(A.rotation, j.target));
            vec3 P = phys3d__point_impulse(A, B, v3(0, 0, 0), v3(0, 0, 0), C, soft, j.linear_impulse);
            vec3 oldP = j.linear_impulse;
            j.linear_impulse += P;
            float maxP = j.max_force * h, lenP = v3len(j.linear_impulse);
            if (lenP > maxP) j.linear_impulse *= maxP / lenP;
            phys3d__apply_at(A, B, v3(0, 0, 0), v3(0, 0, 0), j.linear_impulse - oldP);
        } else if (j.type == PHYS3D_JOINT_GEAR) {
            // velocity only, like a real gear train: no drift correction to fight
            vec3 axa = quat_rotate(A.rotation, j.local_axis_a), axb = quat_rotate(B.rotation, j.local_axis_b);
            phys3d__soft none = { 0.0f, 1.0f, 0.0f };
            phys3d__row_solve(A, B, v3(0, 0, 0), axa * -j.ratio, axb, 0.0f, none, &j.axial_impulse);
        } else if (j.type == PHYS3D_JOINT_PULLEY) {
            vec3 pa = A.position + j.ra, pb = B.position + j.rb;
            vec3 ua = pa - j.ground_a, ub = pb - j.ground_b;
            float la = v3len(ua), lb = v3len(ub);
            if (la > 1e-6f && lb > 1e-6f) {
                ua = ua / la; ub = ub / lb;
                vec3 ca = v3cross(j.ra, ua), cb = v3cross(j.rb, ub);
                float k = A.inv_mass + v3dot(ca, A.inv_inertia * ca) + j.ratio * j.ratio * (B.inv_mass + v3dot(cb, B.inv_inertia * cb));
                if (k > 0.0f) {
                    float cdot = v3dot(ua, A.velocity + v3cross(A.angular_velocity, j.ra))
                               + j.ratio * v3dot(ub, B.velocity + v3cross(B.angular_velocity, j.rb));
                    float C = la + j.ratio * lb - j.length;
                    float imp = -(1.0f / k) * rigid.mass_scale * (cdot + rigid.bias_rate * C) - rigid.impulse_scale * j.axial_impulse;
                    j.axial_impulse += imp;
                    A.velocity += ua * (imp * A.inv_mass);
                    A.angular_velocity += A.inv_inertia * (ca * imp);
                    B.velocity += ub * (imp * j.ratio * B.inv_mass);
                    B.angular_velocity += B.inv_inertia * (cb * (imp * j.ratio));
                }
            }
        } else if (j.type == PHYS3D_JOINT_GENERIC) {
            vec3 dra = j.d + j.ra, axes[3];
            float angles[3];
            mat3 fa = mat3_from_quat(quat_mul(A.rotation, j.frame_a));
            phys3d__generic_angles(A, B, j, axes, angles);
            for (int k = 0; k < 6; k++) {
                bool linear = k < 3;
                vec3 ax = linear ? mat3_col(fa, k) : axes[k - 3];
                if (v3dot(ax, ax) < 0.5f) continue;   // gimbal lock: that axis is not defined
                vec3 lin = linear ? ax : v3(0, 0, 0);
                vec3 angA = linear ? v3cross(dra, ax) : ax, angB = linear ? v3cross(j.rb, ax) : ax;
                float value = linear ? v3dot(j.d, ax) : angles[k - 3];
                if (j.axis_motor_force[k] > 0.0f)
                    phys3d__row_motor(A, B, lin, angA, angB, j.axis_motor_speed[k], j.axis_motor_force[k] * h, &j.axis_motor_impulse[k]);
                if (j.axis_mode[k] == PHYS3D_AXIS_LOCKED) {
                    phys3d__row_solve(A, B, lin, angA, angB, value, rigid, &j.axis_impulse[k]);
                } else if (j.axis_mode[k] == PHYS3D_AXIS_LIMITED) {
                    // one accumulator holds both sides: only one limit can push at a time
                    float lo = maxf(j.axis_impulse[k], 0.0f), hi = maxf(-j.axis_impulse[k], 0.0f);
                    phys3d__row_limits(A, B, lin, angA, angB, value, j.axis_lower[k], j.axis_upper[k], inv_h, rigid, &lo, &hi);
                    j.axis_impulse[k] = lo - hi;
                }
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
// how far any part of a shape reaches from the body's centre: how far a
// turning body can sweep, for speculative margins and continuous collision
static float phys3d__reach(const phys3d_shape& s) {
    if (s.type == PHYS3D_HULL || s.type == PHYS3D_COMPOUND) {
        const aabb& b = s.type == PHYS3D_HULL ? s.hull->bounds : s.compound->bounds;
        return v3len(v3max(v3(fabsf(b.min.x), fabsf(b.min.y), fabsf(b.min.z)), v3(fabsf(b.max.x), fabsf(b.max.y), fabsf(b.max.z))));
    }
    return phys3d__shape_reach(s);
}

// how round a shape is, for rolling friction: its smallest half-extent
static float phys3d__roll_radius(const phys3d_shape& s) {
    if (s.type == PHYS3D_BOX) return minf(s.half.x, minf(s.half.y, s.half.z));
    if (s.type == PHYS3D_HULL) { vec3 e = aabb_half(s.hull->bounds); return minf(e.x, minf(e.y, e.z)); }
    if (s.type == PHYS3D_COMPOUND) { vec3 e = aabb_half(s.compound->bounds); return minf(e.x, minf(e.y, e.z)); }
    return s.radius;
}

static void phys3d__finish_contact(phys3d_world& w, phys3d_contact& c, idx ia, idx ib, uint32_t sub, bool trigger, uint32_t prev) {
    const phys3d_body& a = w.bodies[ia];
    const phys3d_body& b = w.bodies[ib];
    c.a = ia; c.b = ib; c.sub = sub;
    c.trigger = trigger;
    c.friction = sqrtf(a.friction * b.friction);
    c.restitution = maxf(a.restitution, b.restitution);
    // a resisting torque of rolling_friction * normal force * contact radius;
    // the radius is the rounder body's (the one that can actually roll)
    float ra = phys3d__roll_radius(a.shape), rb = phys3d__roll_radius(b.shape);
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
    // linear only: a turning body is caught by the continuous pass (a speculative
    // margin wide enough for rotation makes contact points far from the real touch, and those fling)
    float margin = trigger ? 0.0f : w.speculative + v3len(b.velocity - a.velocity) * dt;
    // one of them a mesh: a contact per nearby triangle
    const phys3d_body* mesh_body = a.shape.type == PHYS3D_MESH ? &a : (b.shape.type == PHYS3D_MESH ? &b : 0);
    if (mesh_body) {
        if (a.shape.type == PHYS3D_MESH && b.shape.type == PHYS3D_MESH) return;
        const phys3d_body& other = mesh_body == &a ? b : a;
        bool mesh_is_a = mesh_body == &a;
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
            phys3d__for_each_core(other.shape, other.position, other.rotation, [&](const phys3d__core& oc, uint32_t part) {
                // one-sided: a body behind the surface passes through rather than snagging
                if (v3dot(oc.centre - v0, tri.axis[0]) < 0.0f) return;
                phys3d__reserve(w.contacts, w.contact_capacity, w.contact_count + 1);
                phys3d_contact& c = w.contacts[w.contact_count];
                bool hit = mesh_is_a ? phys3d__collide(tri, oc, w.slop, margin, c) : phys3d__collide(oc, tri, w.slop, margin, c);
                if (!hit) return;
                float min_sep = 1e30f;
                for (int i = 0; i < c.count; i++) min_sep = minf(min_sep, c.points[i].separation);
                if (trigger && min_sep >= 0.0f) return;
                // which triangle, and in the top byte which part of a compound
                phys3d__finish_contact(w, c, ia, ib, t | (part << 24), trigger, prev);
                if (!first) c.prev = 0xFFFFFFFFu;   // only a pair's first contact carries the link for events
                first = false;
                w.contact_count++;
            });
        });
        return;
    }
    // a manifold per pair of parts (one pair unless a compound is involved)
    bool first = true;
    phys3d__for_each_core(a.shape, a.position, a.rotation, [&](const phys3d__core& ca, uint32_t pa) {
        phys3d__for_each_core(b.shape, b.position, b.rotation, [&](const phys3d__core& cb, uint32_t pb) {
            phys3d__reserve(w.contacts, w.contact_capacity, w.contact_count + 1);
            phys3d_contact& c = w.contacts[w.contact_count];
            if (!phys3d__collide(ca, cb, w.slop, margin, c)) return;
            float min_sep = 1e30f;
            for (int i = 0; i < c.count; i++) min_sep = minf(min_sep, c.points[i].separation);
            if (trigger && min_sep >= 0.0f) return;
            phys3d__finish_contact(w, c, ia, ib, (pa << 12) | pb, trigger, prev);
            if (!first) c.prev = 0xFFFFFFFFu;
            first = false;
            w.contact_count++;
        });
    });
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
    aabb r = aabb_make(v3(1e30f, 1e30f, 1e30f), v3(-1e30f, -1e30f, -1e30f));
    phys3d__for_each_core(b.shape, b.position, b.rotation, [&](const phys3d__core& c, uint32_t) {
        aabb cb = phys3d__core_bounds(c);
        r.min = v3min(r.min, cb.min); r.max = v3max(r.max, cb.max);
    });
    return r;
}

static void phys3d__continuous(phys3d_world& w);   // after the queries, which it uses

// Joint settings are plain fields, so nothing wakes an island when a motor is
// switched on: a motor that wants to turn (or a motor joint away from its
// target) wakes its bodies here instead.
static void phys3d__wake_motors(phys3d_world& w) {
    for (size_t i = 0; i < w.joint_count; i++) {
        const phys3d_joint& j = w.joints[i];
        if (!j.active) continue;
        bool wants = j.enable_motor && j.motor_speed != 0.0f;
        if (j.type == PHYS3D_JOINT_GENERIC)
            for (int k = 0; k < 6; k++) wants |= j.axis_motor_force[k] > 0.0f && j.axis_motor_speed[k] != 0.0f;
        if (j.type == PHYS3D_JOINT_MOTOR) {
            const phys3d_body& A = phys3d__jbody(w, j.a);
            const phys3d_body& B = phys3d__jbody(w, j.b);
            wants = v3dist(B.position, A.position + quat_rotate(A.rotation, j.target)) > w.slop;
        }
        if (!wants) continue;
        if (j.a != PIX_INVALID && w.bodies[j.a].sleeping) phys3d__wake_body(w.bodies[j.a]);
        if (j.b != PIX_INVALID && w.bodies[j.b].sleeping) phys3d__wake_body(w.bodies[j.b]);
    }
}

static void phys3d__step(phys3d_world& w, float dt) {
    if (dt <= 0.0f) return;
    float glen = v3len(w.gravity);
    vec3 gdir = glen > 0.0f ? w.gravity / glen : v3(0, 0, 0);
    phys3d__wake_motors(w);

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
        float spin_pad = w.speculative + v3len(b.angular_velocity) * phys3d__reach(b.shape) * dt;
        vec3 move = v * dt, pad = v3(spin_pad, spin_pad, spin_pad);
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
    for (int i = 0; i < w.action_count; i++) if (w.actions[i].fn) w.actions[i].fn(w, dt, w.actions[i].user);
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
    if (w.continuous) phys3d__continuous(w);
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

// ---- continuous collision ----
//
// Conservative advancement (Mirtich): along the step's motion from the old
// pose to the new, measure the gap to the obstacle (GJK), and advance by the
// longest time in which nothing could close it - the gap divided by the
// fastest any point of the body can approach (its linear speed toward the
// obstacle plus its turn rate times its reach). Repeat until touching or
// through the step. A body that would pass through is left at the first
// touch; the next step's contacts take it from there.

static float phys3d__toi_core(const phys3d_body& b, const phys3d__core& target, vec3 p0, quat q0, vec3 p1, quat q1,
                              float reach, float target_gap) {
    float angle = 2.0f * acosf(clampf(fabsf(q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w), 0.0f, 1.0f));   // how far it turned this step
    vec3 dp = p1 - p0;
    float t = 0.0f;
    for (int iter = 0; iter < 24; iter++) {
        vec3 p = v3lerp(p0, p1, t);
        quat q = quat_slerp(q0, q1, t);
        float gap = 1e30f;
        vec3 n = v3(0, 1, 0);
        phys3d__for_each_core(b.shape, p, q, [&](const phys3d__core& c, uint32_t) {
            float g;
            vec3 nn;
            if (target.kind == PHYS3D__PLANE) {
                float lowest = 1e30f;
                for (int k = 0; k < c.count; k++) lowest = minf(lowest, v3dot(target.axis[0], phys3d__vert(c, k)) - target.offset);
                g = lowest - c.radius;
                nn = target.axis[0];
            } else {
                vec3 pa, pb;
                float d = phys3d__gjk(c, target, &pa, &pb);
                g = d - c.radius - target.radius;
                nn = d > 1e-6f ? (pa - pb) / d : v3(0, 1, 0);
            }
            if (g < gap) { gap = g; n = nn; }
        });
        if (gap <= target_gap) return iter == 0 ? 1.0f : t;   // touching from the start: contacts handle that
        float approach = maxf(0.0f, -v3dot(dp, n)) + angle * reach;
        if (approach <= 1e-9f) return 1.0f;
        t += (gap - 0.5f * target_gap) / approach;
        if (t >= 1.0f) return 1.0f;
    }
    return 1.0f;
}

static void phys3d__continuous(phys3d_world& w) {
    for (size_t i = 0; i < w.body_count; i++) {
        phys3d_body& b = w.bodies[i];
        if (!b.active || b.type != PHYS3D_DYNAMIC || b.sleeping || b.trigger) continue;
        float reach = phys3d__reach(b.shape);
        float angle = 2.0f * acosf(clampf(fabsf(b.prev_rotation.x * b.rotation.x + b.prev_rotation.y * b.rotation.y + b.prev_rotation.z * b.rotation.z + b.prev_rotation.w * b.rotation.w), 0.0f, 1.0f));
        float sweep = v3len(b.position - b.prev_position) + angle * reach;
        // slow bodies are caught by speculative contacts; only sweeps longer
        // than the body is thin can pass through something
        if (sweep < 0.5f * phys3d__roll_radius(b.shape) + w.speculative) continue;
        aabb a0 = b.bounds;   // already covers the old pose, the new one and the margin
        float best = 1.0f;
        phys3d__query_box(w, a0, [&](idx id, const phys3d_body& o) {
            if (id == (idx)i || o.type == PHYS3D_DYNAMIC || o.trigger || !phys3d__layers_ok(b, o)) return;
            if (o.shape.type == PHYS3D_MESH) {
                const phys3d_mesh& m = *o.shape.mesh;
                quat inv = quat_conjugate(o.rotation);
                aabb box = aabb_make(v3(1e30f, 1e30f, 1e30f), v3(-1e30f, -1e30f, -1e30f));
                for (int k = 0; k < 8; k++) {
                    vec3 corner = v3((k & 1) ? a0.max.x : a0.min.x, (k & 2) ? a0.max.y : a0.min.y, (k & 4) ? a0.max.z : a0.min.z);
                    vec3 l = quat_rotate(inv, corner - o.position);
                    box.min = v3min(box.min, l); box.max = v3max(box.max, l);
                }
                phys3d__mesh_query(m, box, [&](uint32_t t) {
                    phys3d__core tri;
                    phys3d__triangle_core(o.position + quat_rotate(o.rotation, m.vertices[m.indices[t * 3]]),
                                          o.position + quat_rotate(o.rotation, m.vertices[m.indices[t * 3 + 1]]),
                                          o.position + quat_rotate(o.rotation, m.vertices[m.indices[t * 3 + 2]]), tri);
                    if (v3dot(b.prev_position - tri.v[0], tri.axis[0]) < 0.0f) return;   // one-sided
                    best = minf(best, phys3d__toi_core(b, tri, b.prev_position, b.prev_rotation, b.position, b.rotation, reach, w.slop));
                });
                return;
            }
            phys3d__for_each_core(o.shape, o.position, o.rotation, [&](const phys3d__core& target, uint32_t) {
                best = minf(best, phys3d__toi_core(b, target, b.prev_position, b.prev_rotation, b.position, b.rotation, reach, w.slop));
            });
        });
        if (best < 1.0f) {
            b.position = v3lerp(b.prev_position, b.position, best);
            b.rotation = quat_norm(quat_slerp(b.prev_rotation, b.rotation, best));
            phys3d__update_inertia(b);
        }
    }
}

// ray against one body; false when it misses or starts inside
static bool phys3d__ray_shape(const phys3d_shape& s, vec3 pos, quat rot, vec3 o, vec3 d, float max_distance,
                              float* out_t, vec3* out_n, uint32_t* out_tri) {
    *out_tri = 0;
    if (s.type == PHYS3D_SPHERE) {
        if (v3dot(o - pos, o - pos) <= s.radius * s.radius) return false;
        if (!ray_vs_sphere(o, d, sphere_make(pos, s.radius), max_distance, out_t)) return false;
        *out_n = v3norm(o + d * *out_t - pos);
        return true;
    }
    if (s.type == PHYS3D_CAPSULE) {
        vec3 up = quat_rotate(rot, v3(0, s.half_length, 0));
        capsule c = capsule_make(pos - up, pos + up, s.radius);
        vec3 q = closest_point_segment(c.a, c.b, o);
        if (v3dot(o - q, o - q) <= s.radius * s.radius) return false;
        if (!ray_vs_capsule(o, d, c, max_distance, out_t)) return false;
        vec3 p = o + d * *out_t;
        *out_n = v3norm(p - closest_point_segment(c.a, c.b, p));
        return true;
    }
    if (s.type == PHYS3D_BOX) {
        obb box;
        box.centre = pos; box.half = s.half; box.axes = mat3_from_quat(rot);
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
        vec3 n = quat_rotate(rot, s.normal);
        if (!ray_vs_plane(o, d, plane_through(n, pos), max_distance, false, out_t)) return false;
        if (v3dot(o - pos, n) < 0.0f) return false;
        *out_n = n;
        return true;
    }
    if (s.type == PHYS3D_MESH) {
        // into the mesh's frame, where its tree lives
        const phys3d_mesh& m = *s.mesh;
        quat inv = quat_conjugate(rot);
        vec3 lo = quat_rotate(inv, o - pos), ld = quat_rotate(inv, d);
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
                        *out_n = quat_rotate(rot, triangle_normal(a, bb, c));
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
    if (s.type == PHYS3D_HULL) {
        // clip the ray by every face plane (Cyrus-Beck): it is inside all of them between enter and leave
        const phys3d_hull& h = *s.hull;
        quat inv = quat_conjugate(rot);
        vec3 lo = quat_rotate(inv, o - pos), ld = quat_rotate(inv, d);
        float enter = 0.0f, leave = max_distance;
        int enter_face = -1;
        for (int f = 0; f < h.face_count; f++) {
            vec3 n = v3(h.planes[f].x, h.planes[f].y, h.planes[f].z);
            float dist = h.planes[f].w - v3dot(n, lo), den = v3dot(n, ld);
            if (fabsf(den) < 1e-12f) { if (dist < 0.0f) return false; continue; }
            float t = dist / den;
            if (den < 0.0f) { if (t > enter) { enter = t; enter_face = f; } }
            else if (t < leave) leave = t;
            if (enter > leave) return false;
        }
        if (enter_face < 0) return false;   // starts inside
        *out_t = enter;
        *out_n = quat_rotate(rot, v3(h.planes[enter_face].x, h.planes[enter_face].y, h.planes[enter_face].z));
        return true;
    }
    if (s.type == PHYS3D_COMPOUND) {
        const phys3d_compound& c = *s.compound;
        bool hit = false;
        float best = max_distance;
        for (int i = 0; i < c.count; i++) {
            const phys3d_compound_child& ch = c.children[i];
            float t; vec3 n; uint32_t tri;
            if (phys3d__ray_shape(ch.shape, pos + quat_rotate(rot, ch.position), quat_mul(rot, ch.rotation), o, d, best, &t, &n, &tri)
                && t <= best) {
                best = t; *out_n = n; *out_tri = (uint32_t)i; hit = true;
            }
        }
        *out_t = best;
        return hit;
    }
    return false;
}

static bool phys3d__ray_body(const phys3d_body& b, vec3 o, vec3 d, float max_distance, float* out_t, vec3* out_n, uint32_t* out_tri) {
    return phys3d__ray_shape(b.shape, b.position, b.rotation, o, d, max_distance, out_t, out_n, out_tri);
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
    if (len <= 0.0f || shape.type == PHYS3D_PLANE || shape.type == PHYS3D_MESH || shape.type == PHYS3D_COMPOUND) return false;
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
            if (moved.kind != PHYS3D__HULL) for (int k = 0; k < moved.count; k++) moved.v[k] += d * t;
            vec3 pa, pb, n = -d;
            float gap;
            if (target.kind == PHYS3D__PLANE) {
                float lowest = 1e30f;
                for (int k = 0; k < moved.count; k++) lowest = minf(lowest, v3dot(target.axis[0], phys3d__vert(moved, k)) - target.offset);
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
        phys3d__for_each_core(b.shape, b.position, b.rotation, [&](const phys3d__core& target, uint32_t part) {
            cast_core(target, id, part);
        });
    });
    return found;
}

size_t phys3d_overlap(phys3d_world& w, phys3d_shape shape, vec3 position, quat rotation,
                      idx* out, size_t capacity, uint16_t mask) {
    if (shape.type == PHYS3D_PLANE || shape.type == PHYS3D_MESH || shape.type == PHYS3D_COMPOUND) return 0;
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
            phys3d__for_each_core(b.shape, b.position, b.rotation, [&](const phys3d__core& c, uint32_t) {
                if (!hit) hit = phys3d__collide(q, c, w.slop, 0.0f, m) && inside();
            });
        }
        if (!hit) return;
        if (count < capacity) out[count] = id;
        count++;
    });
    return count;
}

// ---- actions ----

int phys3d_add_action(phys3d_world& w, phys3d_action_fn fn, void* user) {
    for (int i = 0; i < w.action_count; i++)
        if (!w.actions[i].fn) { w.actions[i].fn = fn; w.actions[i].user = user; return i; }
    if (w.action_count >= PHYS3D_MAX_ACTIONS) { pix_log("phys3d: action limit (%d) reached\n", PHYS3D_MAX_ACTIONS); return -1; }
    w.actions[w.action_count].fn = fn;
    w.actions[w.action_count].user = user;
    return w.action_count++;
}

void phys3d_remove_action(phys3d_world& w, int handle) {
    if (handle >= 0 && handle < w.action_count) { w.actions[handle].fn = 0; w.actions[handle].user = 0; }
}

// ---- raycast vehicle ----

phys3d_vehicle phys3d_make_vehicle(idx chassis) {
    phys3d_vehicle v;
    memset(&v, 0, sizeof(v));
    v.chassis = chassis;
    v.up_local = v3(0, 1, 0);
    v.forward_local = v3(0, 0, 1);
    v.action = -1;
    return v;
}

int phys3d_vehicle_add_wheel(phys3d_vehicle& v, vec3 attach, float radius, float rest_length) {
    if (v.wheel_count >= PHYS3D_MAX_VEHICLE_WHEELS) return -1;
    phys3d_vehicle_wheel& wh = v.wheels[v.wheel_count];
    memset(&wh, 0, sizeof(wh));
    wh.attach = attach;
    wh.radius = radius;
    wh.rest_length = rest_length;
    wh.max_travel = rest_length * 0.8f;
    wh.friction = 1.0f;
    wh.roll_influence = 0.1f;
    wh.suspension_length = rest_length;
    wh.ground = PIX_INVALID;
    return v.wheel_count++;
}

static void phys3d__vehicle_action(phys3d_world& w, float dt, void* user) { phys3d_vehicle_step(w, *(phys3d_vehicle*)user, dt); }

void phys3d_add_vehicle(phys3d_world& w, phys3d_vehicle& v) {
    if (v.action < 0) v.action = phys3d_add_action(w, phys3d__vehicle_action, &v);
}
void phys3d_remove_vehicle(phys3d_world& w, phys3d_vehicle& v) {
    phys3d_remove_action(w, v.action);
    v.action = -1;
}

static inline vec3 phys3d__vehicle_axle(const phys3d_vehicle& v, const phys3d_vehicle_wheel& wh) {
    vec3 axle = v3cross(v.up_local, v.forward_local);
    return quat_rotate(quat_from_axis_angle(v3norm(v.up_local), wh.steer), axle);
}

void phys3d_vehicle_step(phys3d_world& w, phys3d_vehicle& v, float dt) {
    phys3d_body* cb = phys3d_get_body(w, v.chassis);
    if (!cb || cb->type != PHYS3D_DYNAMIC || dt <= 0.0f) return;
    phys3d_body& c = *cb;
    float mass = c.inv_mass > 0.0f ? 1.0f / c.inv_mass : 1.0f;
    vec3 up = v3norm(quat_rotate(c.rotation, v.up_local));
    float g = v3len(w.gravity);
    bool awake = false;
    for (int i = 0; i < v.wheel_count; i++) {
        phys3d_vehicle_wheel& wh = v.wheels[i];
        if (wh.engine_force != 0.0f || wh.steer != 0.0f) awake = true;
        // a spring that holds its share of the chassis 10 cm compressed, damped near critically
        if (wh.stiffness <= 0.0f) wh.stiffness = mass * maxf(g, 1.0f) / (float)v.wheel_count / 0.1f;
        if (wh.damping <= 0.0f) wh.damping = 2.0f * 0.4f * sqrtf(wh.stiffness * mass / (float)v.wheel_count);
    }
    if (awake && c.sleeping) phys3d_wake(w, v.chassis);
    if (c.sleeping) return;

    // ---- suspension: a ray per wheel, a spring and damper where it lands ----
    for (int i = 0; i < v.wheel_count; i++) {
        phys3d_vehicle_wheel& wh = v.wheels[i];
        vec3 hard = c.position + quat_rotate(c.rotation, wh.attach);
        float reach = wh.rest_length + wh.max_travel + wh.radius;
        phys3d_ray_hit h;
        wh.contact = phys3d_raycast(w, hard, -up, reach, &h, c.mask, v.chassis);
        wh.suspension_force = 0.0f;
        if (!wh.contact) { wh.suspension_length = wh.rest_length + wh.max_travel; wh.ground = PIX_INVALID; continue; }
        wh.contact_point = h.point;
        wh.contact_normal = h.normal;
        wh.ground = h.body;
        wh.suspension_length = clampf(h.distance - wh.radius, wh.rest_length - wh.max_travel, wh.rest_length + wh.max_travel);
        float facing = -v3dot(h.normal, -up);   // how square the ground is to the suspension
        if (facing < 0.1f) { wh.contact = false; continue; }
        vec3 rel = h.point - c.position;
        vec3 vel = c.velocity + v3cross(c.angular_velocity, rel);
        float closing = v3dot(h.normal, vel) / facing;   // the suspension's own speed (negative compressing)
        float force = wh.stiffness * (wh.rest_length - wh.suspension_length) - wh.damping * closing;
        wh.suspension_force = maxf(force, 0.0f);         // a spring cannot pull the car down
        phys3d_apply_impulse(c, h.normal * (wh.suspension_force * dt), h.point);
    }

    // ---- tyres: sideways grip and forward drive, sharing a friction circle ----
    // Solved together over a few passes (sequential impulses, accumulated and
    // clamped per tyre) so no wheel's order biases the car to one side.
    struct tyre { vec3 axle, forward, ra, rb, side_at; phys3d_body* ground; bool moves; float grip, side, drive; };
    tyre t[PHYS3D_MAX_VEHICLE_WHEELS];
    for (int i = 0; i < v.wheel_count; i++) {
        phys3d_vehicle_wheel& wh = v.wheels[i];
        tyre& T = t[i];
        T.grip = 0.0f;
        wh.skid = 0.0f;
        if (!wh.contact) continue;
        vec3 n = wh.contact_normal;
        vec3 axle = quat_rotate(c.rotation, phys3d__vehicle_axle(v, wh));
        axle = axle - n * v3dot(axle, n);
        if (v3dot(axle, axle) < 1e-8f) continue;
        T.axle = v3norm(axle);
        T.forward = v3norm(v3cross(n, T.axle));
        if (v3dot(T.forward, quat_rotate(c.rotation, v.forward_local)) < 0.0f) T.forward = -T.forward;
        T.ground = wh.ground != PIX_INVALID ? phys3d_get_body(w, wh.ground) : 0;
        T.moves = T.ground && T.ground->type == PHYS3D_DYNAMIC;
        T.ra = wh.contact_point - c.position;
        T.rb = T.ground ? wh.contact_point - T.ground->position : v3(0, 0, 0);
        // side forces act lower down (roll_influence) so cornering does not tip it over
        T.side_at = T.ra - up * (v3dot(T.ra, up) * (1.0f - wh.roll_influence));
        T.grip = wh.suspension_force * dt * wh.friction;
        T.side = 0.0f;
        T.drive = 0.0f;
    }
    auto rel_vel = [&](const tyre& T) {
        vec3 vel = c.velocity + v3cross(c.angular_velocity, T.ra);
        if (T.ground) vel -= T.ground->velocity + v3cross(T.ground->angular_velocity, T.rb);
        return vel;
    };
    auto inv_mass_along = [&](const tyre& T, vec3 at, vec3 d) {
        float k = c.inv_mass + v3dot(v3cross(at, d), c.inv_inertia * v3cross(at, d));
        if (T.moves) k += T.ground->inv_mass + v3dot(v3cross(T.rb, d), T.ground->inv_inertia * v3cross(T.rb, d));
        return maxf(k, 1e-9f);
    };
    auto push = [&](tyre& T, const phys3d_vehicle_wheel& wh, float dside, float ddrive) {
        vec3 J = T.axle * dside + T.forward * ddrive;
        c.velocity += J * c.inv_mass;
        c.angular_velocity += c.inv_inertia * (v3cross(T.side_at, T.axle * dside) + v3cross(T.ra, T.forward * ddrive));
        if (T.moves) phys3d_apply_impulse(*T.ground, -J, wh.contact_point);
    };
    for (int pass = 0; pass < 4; pass++)
        for (int i = 0; i < v.wheel_count; i++) {
            phys3d_vehicle_wheel& wh = v.wheels[i];
            tyre& T = t[i];
            if (T.grip <= 0.0f) continue;
            vec3 vel = rel_vel(T);
            float side = T.side - v3dot(vel, T.axle) / inv_mass_along(T, T.side_at, T.axle);
            float drive;
            if (wh.engine_force != 0.0f) drive = wh.engine_force * dt;
            else {
                // coasting rolls freely; a brake works against the rolling speed
                float limit = wh.brake * dt;
                drive = clampf(T.drive - v3dot(vel, T.forward) / inv_mass_along(T, T.ra, T.forward), -limit, limit);
            }
            float total = sqrtf(side * side + drive * drive);
            if (total > T.grip) {
                float scale = T.grip / total;
                side *= scale;
                drive *= scale;
                wh.skid = 1.0f - scale;
            } else {
                wh.skid = 0.0f;
            }
            push(T, wh, side - T.side, drive - T.drive);
            T.side = side;
            T.drive = drive;
        }
    for (int i = 0; i < v.wheel_count; i++) {
        phys3d_vehicle_wheel& wh = v.wheels[i];
        if (t[i].grip <= 0.0f) { wh.spin_speed *= 0.99f; wh.spin += wh.spin_speed * dt; continue; }
        wh.spin_speed = v3dot(rel_vel(t[i]), t[i].forward) / maxf(wh.radius, 1e-3f);
        wh.spin += wh.spin_speed * dt;
    }
}

mat4 phys3d_vehicle_wheel_transform(const phys3d_world& w, const phys3d_vehicle& v, int i) {
    if (i < 0 || i >= v.wheel_count || v.chassis >= w.body_count) return mat4_identity();
    const phys3d_body& c = w.bodies[v.chassis];
    const phys3d_vehicle_wheel& wh = v.wheels[i];
    vec3 pos = phys3d_lerp_position(w, c);
    quat rot = phys3d_lerp_rotation(w, c);
    vec3 up = v3norm(v.up_local);
    vec3 centre = wh.attach - up * wh.suspension_length;
    quat q = quat_mul(rot, quat_mul(quat_from_axis_angle(up, wh.steer), quat_from_axis_angle(v3norm(v3cross(up, v.forward_local)), wh.spin)));
    return mat4_from_trs(pos + quat_rotate(rot, centre), q, v3(1, 1, 1));
}

float phys3d_vehicle_speed(phys3d_world& w, const phys3d_vehicle& v) {
    phys3d_body* c = phys3d_get_body(w, v.chassis);
    return c ? v3dot(c->velocity, quat_rotate(c->rotation, v.forward_local)) : 0.0f;
}

// ---- character controller ----

phys3d_character phys3d_make_character(vec3 feet, float radius, float height) {
    phys3d_character c;
    memset(&c, 0, sizeof(c));
    c.position = feet;
    c.radius = radius > 0.01f ? radius : 0.01f;
    c.height = maxf(height, 2.0f * c.radius);
    c.max_slope = 50.0f * PIX_DEG2RAD;
    c.step_height = 0.35f;
    c.snap_distance = 0.35f;
    c.skin = 0.01f;
    c.jump_speed = 5.0f;
    c.push_force = 600.0f;
    c.frame_dt = 1.0f / 60.0f;
    c.mask = PHYS_LAYER_ALL;
    c.body = PIX_INVALID;
    c.ground_body = PIX_INVALID;
    c.ground_normal = v3(0, 1, 0);
    return c;
}

static inline vec3 phys3d__char_up(const phys3d_world& w) {
    float g = v3len(w.gravity);
    return g > 1e-6f ? -w.gravity / g : v3(0, 1, 0);
}
// the capsule's rotation: its local y along `up`
static quat phys3d__char_rot(vec3 up) {
    vec3 y = v3(0, 1, 0);
    float d = v3dot(y, up);
    if (d > 0.9999f) return quat_identity();
    if (d < -0.9999f) return quat_from_axis_angle(v3(1, 0, 0), PIX_PI);
    return quat_from_axis_angle(v3norm(v3cross(y, up)), acosf(clampf(d, -1.0f, 1.0f)));
}
static inline vec3 phys3d__char_centre(const phys3d_character& c, vec3 up) { return c.position + up * (c.height * 0.5f); }

// Pushes the capsule out of anything it overlaps (a platform rose into it, it
// was placed inside something).
static void phys3d__char_depenetrate(phys3d_world& w, phys3d_character& c, vec3 up) {
    phys3d_shape cap = phys3d_capsule(c.radius, c.height);
    quat q = phys3d__char_rot(up);
    for (int iter = 0; iter < 4; iter++) {
        phys3d__core me;
        phys3d__make_core(cap, phys3d__char_centre(c, up), q, me);
        vec3 push = v3(0, 0, 0);
        phys3d_contact m;
        phys3d__query_box(w, phys3d__core_bounds(me), [&](idx id, const phys3d_body& b) {
            if (id == c.body || b.trigger || !(b.layer & c.mask) || b.type == PHYS3D_DYNAMIC) return;
            // out to the skin, not just to touching: a capsule resting exactly
            // on the floor would see the floor in every sweep, even sideways
            auto test = [&](const phys3d__core& o) {
                if (!phys3d__collide(me, o, 0.0f, c.skin, m)) return;
                float deepest = c.skin;
                for (int k = 0; k < m.count; k++) deepest = minf(deepest, m.points[k].separation);
                if (deepest < c.skin * 0.99f) push += m.normal * (deepest - c.skin);   // normal points away from the capsule
            };
            if (b.shape.type == PHYS3D_MESH) {
                const phys3d_mesh& mesh = *b.shape.mesh;
                aabb mb = phys3d__core_bounds(me);
                quat inv = quat_conjugate(b.rotation);
                aabb box = aabb_make(v3(1e30f, 1e30f, 1e30f), v3(-1e30f, -1e30f, -1e30f));
                for (int k = 0; k < 8; k++) {
                    vec3 corner = v3((k & 1) ? mb.max.x : mb.min.x, (k & 2) ? mb.max.y : mb.min.y, (k & 4) ? mb.max.z : mb.min.z);
                    vec3 l = quat_rotate(inv, corner - b.position);
                    box.min = v3min(box.min, l); box.max = v3max(box.max, l);
                }
                phys3d__mesh_query(mesh, box, [&](uint32_t t) {
                    phys3d__core tri;
                    phys3d__triangle_core(b.position + quat_rotate(b.rotation, mesh.vertices[mesh.indices[t * 3]]),
                                          b.position + quat_rotate(b.rotation, mesh.vertices[mesh.indices[t * 3 + 1]]),
                                          b.position + quat_rotate(b.rotation, mesh.vertices[mesh.indices[t * 3 + 2]]), tri);
                    test(tri);
                });
            } else {
                phys3d__for_each_core(b.shape, b.position, b.rotation, [&](const phys3d__core& o, uint32_t) { test(o); });
            }
        });
        if (v3dot(push, push) < 1e-10f) return;
        c.position += push;
    }
}

// One sweep along `d`: moves as far as it can (keeping the skin), and reports
// the surface it stopped at. Returns the fraction of `d` it travelled.
static float phys3d__char_sweep(phys3d_world& w, phys3d_character& c, vec3 up, vec3 d, phys3d_ray_hit* hit, bool* blocked) {
    *blocked = false;
    float len = v3len(d);
    if (len < 1e-7f) return 1.0f;
    phys3d_ray_hit h;
    if (!phys3d_shape_cast(w, phys3d_capsule(c.radius, c.height), phys3d__char_centre(c, up), phys3d__char_rot(up), d,
                           len + c.skin, &h, c.mask, c.body)) {
        c.position += d;
        return 1.0f;
    }
    // stop the skin short of the surface measured along its normal, so an
    // oblique approach keeps the same gap as a head-on one
    float approach = maxf(-v3dot(d / len, h.normal), 0.1f);
    float travel = maxf(0.0f, h.distance - c.skin / approach);
    c.position += d * (travel / len);
    *hit = h;
    *blocked = true;
    return travel / len;
}

// Slides along everything it meets, up to a few times per move. `walkable`
// surfaces only redirect; walls also refuse to be climbed (no gain along up).
static void phys3d__char_slide(phys3d_world& w, phys3d_character& c, vec3 up, vec3 d, float cos_slope, bool horizontal) {
    vec3 first_wall = v3(0, 0, 0);
    for (int iter = 0; iter < 4 && v3dot(d, d) > 1e-12f; iter++) {
        phys3d_ray_hit h;
        bool blocked;
        float done = phys3d__char_sweep(w, c, up, d, &h, &blocked);
        if (!blocked) return;
        vec3 n = h.normal;
        // shove what it walked into, with its push force over this frame
        if (c.push_force > 0.0f && h.body != PIX_INVALID && v3dot(d, n) < 0.0f) {
            phys3d_body* b = phys3d_get_body(w, h.body);
            if (b && b->type == PHYS3D_DYNAMIC) phys3d_apply_impulse(*b, -n * (c.push_force * c.frame_dt), h.point);
        }
        vec3 rest = d * (1.0f - done);
        bool walkable = v3dot(n, up) >= cos_slope;
        if (horizontal && !walkable) {
            // a wall: slide along it, sideways only
            n = n - up * v3dot(n, up);
            if (v3dot(n, n) < 1e-10f) return;
            n = v3norm(n);
        }
        rest = rest - n * v3dot(rest, n);
        // in a corner between two walls: along their crease
        if (iter > 0 && v3dot(first_wall, first_wall) > 0.0f) {
            vec3 crease = v3cross(first_wall, n);
            if (v3dot(crease, crease) > 1e-8f) {
                crease = v3norm(crease);
                rest = crease * v3dot(rest, crease);
            }
        }
        if (iter == 0) first_wall = n;
        if (horizontal) rest = rest - up * v3dot(rest, up);   // walking never lifts it
        d = rest;
    }
}

void phys3d_character_move(phys3d_world& w, phys3d_character& c, vec3 displacement) {
    vec3 up = phys3d__char_up(w);
    float cos_slope = cosf(c.max_slope);
    vec3 start = c.position;
    phys3d__char_depenetrate(w, c, up);
    float vertical = v3dot(displacement, up);
    vec3 side = displacement - up * vertical;
    bool was_on_ground = c.on_ground;
    c.on_ground = false;
    c.ground_body = PIX_INVALID;

    // 1. up by the step height (only when walking on the ground), 2. across,
    // 3. down by as much as it went up plus the fall: a ledge lower than the
    // step is walked onto, a higher one is a wall
    float lifted = 0.0f;
    if (was_on_ground && v3dot(side, side) > 1e-12f && c.step_height > 0.0f) {
        vec3 before = c.position;
        phys3d_ray_hit h; bool blocked;
        phys3d__char_sweep(w, c, up, up * c.step_height, &h, &blocked);
        lifted = v3dot(c.position - before, up);
    }
    phys3d__char_slide(w, c, up, side, cos_slope, true);
    // down: the lift back, the fall, and on the ground a snap to follow slopes and steps
    float down = lifted - minf(vertical, 0.0f);
    bool rising = vertical > 0.0f;
    if (rising) phys3d__char_slide(w, c, up, up * vertical, cos_slope, false);
    float snap = (was_on_ground && !rising) ? c.snap_distance : 0.0f;
    if (down + snap > 0.0f) {
        vec3 before = c.position;
        phys3d_ray_hit h; bool blocked;
        phys3d__char_sweep(w, c, up, -up * (down + snap), &h, &blocked);
        float fallen = v3dot(before - c.position, up);
        vec3 ground_n = blocked ? h.normal : up;
        if (blocked && v3dot(ground_n, up) < cos_slope) {
            // On an edge (the top of a step) the touch normal leans over though
            // the surface is flat: ask the surface itself, just past the edge
            // (away from the capsule's axis, onto the thing it touches).
            vec3 outward = (h.point - c.position) - up * v3dot(h.point - c.position, up);
            float l = v3len(outward);
            vec3 probe = h.point + up * 0.05f + (l > 1e-5f ? outward / l * 0.02f : v3(0, 0, 0));
            phys3d_ray_hit g;
            if (phys3d_raycast(w, probe, -up, 0.1f, &g, c.mask, c.body) && v3dot(g.normal, up) >= cos_slope) ground_n = g.normal;
        }
        if (blocked && v3dot(ground_n, up) >= cos_slope) {
            c.on_ground = true;
            c.ground_normal = ground_n;
            c.ground_body = h.body;
        } else if (blocked) {
            // too steep to stand on: the rest of the real fall slides down it
            if (down > fallen) phys3d__char_slide(w, c, up, -up * (down - fallen), cos_slope, false);
        } else if (snap > 0.0f) {
            // no ground within the snap: it walked off an edge, so only the real fall applies
            c.position = before - up * down;
        }
    }
    c.last_move = c.position - start;
    if (c.body != PIX_INVALID) {
        phys3d_body* b = phys3d_get_body(w, c.body);
        if (b) { b->position = phys3d__char_centre(c, up); b->rotation = phys3d__char_rot(up); }
    }
}

void phys3d_character_update(phys3d_world& w, phys3d_character& c, vec3 walk_velocity, bool jump, float dt) {
    if (dt <= 0.0f) return;
    c.frame_dt = dt;
    vec3 up = phys3d__char_up(w);
    if (c.on_ground) {
        if (c.vertical_speed < 0.0f) c.vertical_speed = 0.0f;
        if (jump) c.vertical_speed = c.jump_speed;
    }
    c.vertical_speed += v3dot(w.gravity, up) * dt;
    vec3 walk = walk_velocity - up * v3dot(walk_velocity, up);
    vec3 move = walk * dt + up * (c.vertical_speed * dt);
    // standing on something that moves: go with it
    if (c.on_ground && c.ground_body != PIX_INVALID && c.ground_body != c.body) {
        const phys3d_body* g = phys3d_get_body(w, c.ground_body);
        if (g && g->type != PHYS3D_STATIC)
            move += (g->velocity + v3cross(g->angular_velocity, c.position - g->position)) * dt;
    }
    float wanted_up = v3dot(move, up);
    phys3d_character_move(w, c, move);
    float got_up = v3dot(c.last_move, up);
    if (c.on_ground && c.vertical_speed < 0.0f) c.vertical_speed = 0.0f;
    if (wanted_up > 0.0f && got_up < wanted_up * 0.5f && !c.on_ground) c.vertical_speed = 0.0f;   // hit its head
    c.velocity = c.last_move / dt;
}

idx phys3d_add_character_body(phys3d_world& w, phys3d_character& c) {
    vec3 up = phys3d__char_up(w);
    phys3d_body b = phys3d_make_body(PHYS3D_KINEMATIC, phys3d_capsule(c.radius, c.height), phys3d__char_centre(c, up));
    b.rotation = phys3d__char_rot(up);
    b.layer = PHYS_LAYER_0;
    c.body = phys3d_add_body(w, b);
    return c.body;
}

#endif // PIX_PHYSICS3D_IMPL
