#ifndef PIX_PHYSICS3D_FLAT_HPP
#define PIX_PHYSICS3D_FLAT_HPP
// dtype.hpp first: it sets up the CRT before any C header
#include "dtype.hpp"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "math.hpp"
#include "colliders.hpp"
#include "physics_common.hpp"
#include "jobs.hpp"

// The flat 3D world: a small, allocation-light physics world for a
// character-scale game. An immovable set of boxes (level geometry, walls,
// props) plus a population of upright cylinders (characters) and upright,
// yawed boxes (vehicles, crates) that push against them and each other.
//
// It is deliberately 2.5D. For a game played on broadly flat ground every
// collision that matters resolves in the XZ plane, and Y is only gravity plus
// a ground clamp. That buys a solver that is a couple of hundred lines, runs a
// thousand bodies in well under a millisecond, and never tunnels at the speeds
// a vehicle reaches. Bodies do not tip, roll or stack - for that, physics3d.hpp.
//
// Two broadphases, because the two sets behave differently:
//   statics  - built once after world generation, bucketed into a uniform grid
//   dynamics - rebuilt every step into a hash grid, since they all move
//
// The shape maths lives in colliders.hpp; this file is only the simulation
// built on top of it - integration, broadphase, and turning contacts into
// motion. No third-party code.
//
// ---- what is threaded, and what is not ----
//
// Hand the world a pix_jobs pool and two of the three phases split across it.
// Which two is decided by what each phase writes, not by what it costs:
//
//   integration       body i writes body i. Independent, so parallel.
//   static contacts   body i writes body i and *reads* the static grid, which
//                     nothing mutates during a step. Independent, so parallel,
//                     and it is also the expensive phase - a body in dense
//                     geometry tests against every wall and prop near it.
//   dynamic contacts  resolving a pair writes to *both* bodies, so two
//                     threads holding overlapping pairs would race. This one
//                     stays on the calling thread. It is also the cheaper of
//                     the two: a typical world has a few hundred dynamic
//                     bodies against tens of thousands of statics.
//
// Splitting only what is provably independent is why there is not a lock
// anywhere in the step.

// Capacities, not memory in the struct: the arrays behind them are heap
// allocated by phys3d_flat_create_world. Define before including pix to change them.
#ifndef PHYS3D_FLAT_MAX_BODIES
#define PHYS3D_FLAT_MAX_BODIES    2048
#endif
#ifndef PHYS3D_FLAT_MAX_STATICS
#define PHYS3D_FLAT_MAX_STATICS   32768
#endif
#define PHYS3D_FLAT_CELL          10.0f   // metres per broadphase cell
#define PHYS3D_FLAT_SOLVER_PASSES 2

// shapes
#define PHYS3D_FLAT_CYLINDER 0   // upright, `radius` wide, `height` tall, origin at the feet
#define PHYS3D_FLAT_BOX      1   // upright box, `half` extents in local XZ, rotated by `yaw`

struct phys3d_flat_body {
    vec3  position;      // XZ centre, Y at the feet
    vec3  velocity;
    float yaw;           // PHYS3D_FLAT_BOX only; rotation about +Y
    float radius;        // PHYS3D_FLAT_CYLINDER
    vec2  half;          // PHYS3D_FLAT_BOX: x across, y along the facing axis
    float height;
    float inv_mass;      // 1 / mass; 0 = immovable
    float restitution;   // 0 = dead stop, 1 = perfect bounce
    float linear_damping;   // per second, like air drag; XZ only (Y is gravity and the ground)
    // Coulomb friction at a contact, as a fraction of the normal impulse.
    // Without it two bodies in contact slide across each other freely - a
    // character resting against a moving vehicle glides off its flank instead
    // of being dragged along with it.
    float friction;
    float gravity_scale; // 1 normal, 0 floats (ghosts, flying things)

    uint8_t shape;
    uint16_t layer;      // which PHYS_LAYER_* bits this body is on
    uint16_t mask;       // which layers it collides with
    void*    user;       // the game-side owner, so a hit can be traced back

    // ---- written by the world ----
    bool    active;
    bool    on_ground;
    bool    touched;     // set by the solver whenever anything pushed this body this step
    // Largest normal impulse this body took this step, in metres per second
    // of velocity change. Game code reads it to tell a shove from a crash
    // without having to subscribe to the collisions themselves.
    float   impact;
    vec3    prev_position;   // one step ago, for interpolation
};

// The true footprint of a static, kept beside its axis-aligned bound.
//
// The broadphase grid and the raycaster want an AABB and always will. The
// narrowphase does not: a prop dropped at 40 degrees has an AABB a third
// wider than the thing itself in both directions, and walking into that is
// being stopped by air a foot short of the prop. So the AABB stays as the
// coarse bound and this carries what the collider actually is.
struct phys3d_flat_static_shape {
    vec2  half;      // local half extents in XZ
    float yaw;
    bool  rotated;   // false for a box added axis-aligned; skips the OBB path
};

struct phys3d_flat_ray_hit {
    idx   body;          // the body hit, or PIX_INVALID for static geometry
    vec3  point, normal; // normal faces the ray
    float distance;
};

struct phys3d_flat_world {
    // ---- settings ----
    vec3  gravity;        // (0, -22, 0): snappier than real gravity at character scale
    float ground_y;
    float step_height;    // statics shorter than this are stepped over, not blocked
    float fixed_dt;       // phys3d_flat_update's step, 1/60
    int   max_steps;      // per update, 4

    // Where the floor is, if it is not flat.
    //
    // A single `ground_y` is right for one flat plane and wrong for anything
    // else - a ledge, a slope, a bridge deck over water. Set this and bodies
    // land on whatever height it returns for (x, z) instead. It is called from
    // the integration phase, which is threaded, so it must only read: no
    // lazily built caches, no allocation, no writing back into the terrain.
    float (*ground_at)(void* user, float x, float z);
    void*  ground_user;

    // Optional; without one every phase runs on the calling thread.
    pix_jobs* jobs;

    // ---- state ----
    float accumulator, alpha;

    aabb*  statics;                        // PHYS3D_FLAT_MAX_STATICS, heap
    phys3d_flat_static_shape* static_shape;
    size_t      static_count;

    // static uniform grid, built by phys3d_flat_build_statics
    int       grid_x0, grid_z0, grid_w, grid_h;
    uint32_t* cell_start;     // grid_w * grid_h + 1 prefix offsets
    uint32_t* cell_items;     // static indices, bucketed
    size_t    cell_item_count;

    phys3d_flat_body* bodies;              // PHYS3D_FLAT_MAX_BODIES, heap
    size_t    body_count;

    // dynamic hash grid, rebuilt every step
    uint32_t* dyn_start;
    uint32_t* dyn_items;
    uint32_t* dyn_cursor;
};

// ---- world ----
PIXDEF void phys3d_flat_create_world(phys3d_flat_world& w);
PIXDEF void phys3d_flat_destroy_world(phys3d_flat_world& w);
// fixed steps of w.fixed_dt, as many as `frame_dt` covers; returns how many ran
PIXDEF int  phys3d_flat_update(phys3d_flat_world& w, float frame_dt);
PIXDEF void phys3d_flat_step(phys3d_flat_world& w, float dt);

// ---- static geometry ----
PIXDEF void phys3d_flat_add_static_box(phys3d_flat_world& w, vec3 min, vec3 max);
// a yawed box standing on `position` (its base centre)
PIXDEF void phys3d_flat_add_static_prop(phys3d_flat_world& w, vec3 position, float yaw, vec2 half, float height);
PIXDEF void phys3d_flat_build_statics(phys3d_flat_world& w);
// drops every static collider (not the bodies) so the world can be
// regenerated in place - see the definition for why this has to exist
PIXDEF void phys3d_flat_reset_statics(phys3d_flat_world& w);

// ---- bodies ----
// bodies with sensible defaults: mass 1, gravity on, layer 0, colliding with all
PIXDEF phys3d_flat_body phys3d_flat_make_cylinder(vec3 feet, float radius, float height);
PIXDEF phys3d_flat_body phys3d_flat_make_box(vec3 base, vec2 half, float height, float yaw);
PIXDEF idx  phys3d_flat_add_body(phys3d_flat_world& w, const phys3d_flat_body& body);   // PIX_INVALID when full
PIXDEF phys3d_flat_body* phys3d_flat_get_body(phys3d_flat_world& w, idx id);            // nullptr once removed
PIXDEF void phys3d_flat_remove_body(phys3d_flat_world& w, idx id);
static inline vec3 phys3d_flat_lerp_position(const phys3d_flat_world& w, const phys3d_flat_body& b) {
    return v3lerp(b.prev_position, b.position, w.alpha);
}

// ---- queries ----
// The nearest thing a ray meets, static geometry or body; bodies on layers
// outside `mask`, and `ignore`, are skipped (static geometry always counts).
PIXDEF bool phys3d_flat_raycast(const phys3d_flat_world& w, vec3 origin, vec3 direction, float max_distance,
                                phys3d_flat_ray_hit* hit, uint16_t mask = PHYS_LAYER_ALL, idx ignore = PIX_INVALID);
// Bodies whose footprint centre lies within `radius` (in XZ) of `centre`.
// Returns how many; writes up to `capacity`. Exact at any time.
PIXDEF size_t phys3d_flat_overlap(const phys3d_flat_world& w, vec3 centre, float radius,
                                  idx* out, size_t capacity, uint16_t mask = PHYS_LAYER_ALL);
// The same question through the hash grid the last step built - much faster
// for crowds asking for their neighbours every frame, but blind to bodies
// added or moved by hand since that step.
PIXDEF size_t phys3d_flat_query_neighbours(const phys3d_flat_world& w, vec3 centre, float radius,
                                           idx* out, size_t capacity, uint16_t mask = PHYS_LAYER_ALL);
// would an upright cylinder placed here intersect static geometry?
PIXDEF bool phys3d_flat_point_blocked(const phys3d_flat_world& w, vec3 position, float radius, float height);

// debug draw: phys3d_flat_debug_draw in physics_debug.hpp (it needs the renderer)

#endif // PIX_PHYSICS3D_FLAT_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_PHYSICS3D_FLAT_IMPL)
#define PIX_PHYSICS3D_FLAT_IMPL

#define PHYS3D_FLAT_DYN_BUCKETS 4096

void phys3d_flat_create_world(phys3d_flat_world& w) {
    memset(&w, 0, sizeof(w));
    w.statics      = pix_alloc<aabb>(PHYS3D_FLAT_MAX_STATICS);
    w.static_shape = pix_alloc<phys3d_flat_static_shape>(PHYS3D_FLAT_MAX_STATICS);
    w.bodies       = pix_alloc<phys3d_flat_body>(PHYS3D_FLAT_MAX_BODIES);
    w.dyn_start    = pix_alloc<uint32_t>(PHYS3D_FLAT_DYN_BUCKETS + 1);
    w.dyn_cursor   = pix_alloc<uint32_t>(PHYS3D_FLAT_DYN_BUCKETS + 1);
    w.dyn_items    = pix_alloc<uint32_t>(PHYS3D_FLAT_MAX_BODIES);
    // Snappier than 9.8: a character falling at real gravity reads as floaty
    // at game scale. Overwrite it for a simulation.
    w.gravity = v3(0.0f, -22.0f, 0.0f);
    w.ground_y = 0.0f;
    w.step_height = 0.45f;
    w.fixed_dt = 1.0f / 60.0f;
    w.max_steps = 4;
}

void phys3d_flat_destroy_world(phys3d_flat_world& w) {
    free(w.cell_start);
    free(w.cell_items);
    pix_free(w.statics);
    pix_free(w.static_shape);
    pix_free(w.bodies);
    pix_free(w.dyn_start);
    pix_free(w.dyn_cursor);
    pix_free(w.dyn_items);
    memset(&w, 0, sizeof(w));
}

// Drops every static collider and the grid built over them, leaving every
// dynamic body exactly where it was.
//
// This is what regenerating a level in place needs. A generator that runs a
// second time replaces its geometry, but without this the colliders from the
// first run are still standing - invisible walls in the middle of whatever got
// built over them. Generators should call it before they repopulate.
void phys3d_flat_reset_statics(phys3d_flat_world& w) {
    free(w.cell_start);
    free(w.cell_items);
    w.cell_start = 0;
    w.cell_items = 0;
    w.cell_item_count = 0;
    w.static_count = 0;
}

void phys3d_flat_add_static_box(phys3d_flat_world& w, vec3 min, vec3 max) {
    if (w.static_count >= PHYS3D_FLAT_MAX_STATICS) return;
    size_t i = w.static_count++;
    w.statics[i] = aabb_make(min, max);
    w.static_shape[i].half = v2((max.x - min.x) * 0.5f, (max.z - min.z) * 0.5f);
    w.static_shape[i].yaw = 0.0f;
    w.static_shape[i].rotated = false;
}

void phys3d_flat_add_static_prop(phys3d_flat_world& w, vec3 position, float yaw, vec2 half, float height) {
    if (w.static_count >= PHYS3D_FLAT_MAX_STATICS) return;
    obb2 box = { v2(position.x, position.z), half, yaw };
    size_t i = w.static_count++;
    w.statics[i] = aabb_from_obb(box, position.y, height);
    w.static_shape[i].half = half;
    w.static_shape[i].yaw = yaw;
    // A quarter turn maps the box onto itself with its extents swapped, which
    // aabb_from_obb has already done - so those stay on the cheaper rect path.
    // Only genuinely diagonal props need the oriented test.
    float turns = yaw * (2.0f / 3.14159265f);
    float snapped = floorf(turns + 0.5f);
    w.static_shape[i].rotated = fabsf(turns - snapped) > 0.02f;
}

void phys3d_flat_build_statics(phys3d_flat_world& w) {
    free(w.cell_start);
    free(w.cell_items);
    w.cell_start = 0;
    w.cell_items = 0;
    w.cell_item_count = 0;
    if (!w.static_count) return;

    // bounds of everything, in cells
    float x0 = w.statics[0].min.x, x1 = w.statics[0].max.x;
    float z0 = w.statics[0].min.z, z1 = w.statics[0].max.z;
    for (size_t i = 1; i < w.static_count; i++) {
        const aabb& s = w.statics[i];
        if (s.min.x < x0) x0 = s.min.x;
        if (s.max.x > x1) x1 = s.max.x;
        if (s.min.z < z0) z0 = s.min.z;
        if (s.max.z > z1) z1 = s.max.z;
    }
    w.grid_x0 = (int)floorf(x0 / PHYS3D_FLAT_CELL);
    w.grid_z0 = (int)floorf(z0 / PHYS3D_FLAT_CELL);
    w.grid_w  = (int)floorf(x1 / PHYS3D_FLAT_CELL) - w.grid_x0 + 1;
    w.grid_h  = (int)floorf(z1 / PHYS3D_FLAT_CELL) - w.grid_z0 + 1;
    if (w.grid_w < 1) w.grid_w = 1;
    if (w.grid_h < 1) w.grid_h = 1;

    size_t cells = (size_t)w.grid_w * (size_t)w.grid_h;
    w.cell_start = (uint32_t*)calloc(cells + 1, sizeof(uint32_t));
    if (!w.cell_start) return;

    // counting sort: pass one tallies how many boxes touch each cell
    for (size_t i = 0; i < w.static_count; i++) {
        const aabb& s = w.statics[i];
        int cx0 = (int)floorf(s.min.x / PHYS3D_FLAT_CELL) - w.grid_x0;
        int cx1 = (int)floorf(s.max.x / PHYS3D_FLAT_CELL) - w.grid_x0;
        int cz0 = (int)floorf(s.min.z / PHYS3D_FLAT_CELL) - w.grid_z0;
        int cz1 = (int)floorf(s.max.z / PHYS3D_FLAT_CELL) - w.grid_z0;
        for (int cz = cz0; cz <= cz1; cz++)
            for (int cx = cx0; cx <= cx1; cx++)
                if (cx >= 0 && cz >= 0 && cx < w.grid_w && cz < w.grid_h)
                    w.cell_start[(size_t)cz * w.grid_w + cx + 1]++;
    }
    for (size_t i = 0; i < cells; i++) w.cell_start[i + 1] += w.cell_start[i];
    w.cell_item_count = w.cell_start[cells];

    w.cell_items = (uint32_t*)malloc(w.cell_item_count * sizeof(uint32_t));
    if (!w.cell_items) return;

    uint32_t* cursor = (uint32_t*)malloc((cells + 1) * sizeof(uint32_t));
    if (!cursor) return;
    memcpy(cursor, w.cell_start, (cells + 1) * sizeof(uint32_t));

    for (size_t i = 0; i < w.static_count; i++) {
        const aabb& s = w.statics[i];
        int cx0 = (int)floorf(s.min.x / PHYS3D_FLAT_CELL) - w.grid_x0;
        int cx1 = (int)floorf(s.max.x / PHYS3D_FLAT_CELL) - w.grid_x0;
        int cz0 = (int)floorf(s.min.z / PHYS3D_FLAT_CELL) - w.grid_z0;
        int cz1 = (int)floorf(s.max.z / PHYS3D_FLAT_CELL) - w.grid_z0;
        for (int cz = cz0; cz <= cz1; cz++)
            for (int cx = cx0; cx <= cx1; cx++)
                if (cx >= 0 && cz >= 0 && cx < w.grid_w && cz < w.grid_h)
                    w.cell_items[cursor[(size_t)cz * w.grid_w + cx]++] = (uint32_t)i;
    }
    free(cursor);
}

phys3d_flat_body phys3d_flat_make_cylinder(vec3 feet, float radius, float height) {
    phys3d_flat_body b;
    memset(&b, 0, sizeof(b));
    b.shape = PHYS3D_FLAT_CYLINDER;
    b.position = feet;
    b.radius = radius;
    b.height = height;
    b.inv_mass = 1.0f;
    b.gravity_scale = 1.0f;
    b.friction = 0.4f;
    b.layer = PHYS_LAYER_0;
    b.mask = PHYS_LAYER_ALL;
    return b;
}

phys3d_flat_body phys3d_flat_make_box(vec3 base, vec2 half, float height, float yaw) {
    phys3d_flat_body b = phys3d_flat_make_cylinder(base, 0.0f, height);
    b.shape = PHYS3D_FLAT_BOX;
    b.half = half;
    b.yaw = yaw;
    return b;
}

idx phys3d_flat_add_body(phys3d_flat_world& w, const phys3d_flat_body& body) {
    idx id = PIX_INVALID;
    for (size_t i = 0; i < w.body_count && id == PIX_INVALID; i++)
        if (!w.bodies[i].active) id = (idx)i;
    if (id == PIX_INVALID) {
        if (w.body_count >= PHYS3D_FLAT_MAX_BODIES) { pix_log("phys3d_flat: body limit (%d) reached\n", PHYS3D_FLAT_MAX_BODIES); return PIX_INVALID; }
        id = (idx)w.body_count++;
    }
    w.bodies[id] = body;
    w.bodies[id].active = true;
    w.bodies[id].prev_position = body.position;
    return id;
}

phys3d_flat_body* phys3d_flat_get_body(phys3d_flat_world& w, idx id) {
    return (id < w.body_count && w.bodies[id].active) ? &w.bodies[id] : 0;
}

void phys3d_flat_remove_body(phys3d_flat_world& w, idx id) {
    if (id < w.body_count) w.bodies[id].active = false;
}

// the horizontal footprint radius a body sweeps, used for broadphase sizing
static float phys3d_flat__body_radius(const phys3d_flat_body& b) {
    return b.shape == PHYS3D_FLAT_BOX ? sqrtf(b.half.x * b.half.x + b.half.y * b.half.y) : b.radius;
}

static bool phys3d_flat__layers_interact(const phys3d_flat_body& a, const phys3d_flat_body& b) {
    return phys_layers_collide(a.layer, a.mask, b.layer, b.mask);
}

// ---- static resolution ----

static void phys3d_flat__resolve_statics(phys3d_flat_world& w, phys3d_flat_body& b) {
    if (!w.cell_start || !w.cell_items) return;

    float r = phys3d_flat__body_radius(b);
    int cx0 = (int)floorf((b.position.x - r) / PHYS3D_FLAT_CELL) - w.grid_x0;
    int cx1 = (int)floorf((b.position.x + r) / PHYS3D_FLAT_CELL) - w.grid_x0;
    int cz0 = (int)floorf((b.position.z - r) / PHYS3D_FLAT_CELL) - w.grid_z0;
    int cz1 = (int)floorf((b.position.z + r) / PHYS3D_FLAT_CELL) - w.grid_z0;

    obb2    self_box    = { v2xz(b.position), b.half, b.yaw };
    circle2 self_circle = { v2xz(b.position), b.radius };
    // the body's square in XZ: a static whose bounds miss it cannot touch it
    // (a rotated prop sits inside its bounds), so no shape test is needed
    float sx0 = b.position.x - r, sx1 = b.position.x + r, sz0 = b.position.z - r, sz1 = b.position.z + r;

    for (int cz = cz0; cz <= cz1; cz++) {
        if (cz < 0 || cz >= w.grid_h) continue;
        for (int cx = cx0; cx <= cx1; cx++) {
            if (cx < 0 || cx >= w.grid_w) continue;
            size_t cell = (size_t)cz * w.grid_w + cx;
            for (uint32_t k = w.cell_start[cell]; k < w.cell_start[cell + 1]; k++) {
                const aabb& s = w.statics[w.cell_items[k]];
                if (s.max.x < sx0 || s.min.x > sx1 || s.max.z < sz0 || s.min.z > sz1) continue;

                // ledges, steps and doorsteps: low enough to walk straight over
                if (aabb_height(s) <= w.step_height) continue;
                // standing on the roof, or passing under an overhang
                if (b.position.y >= s.max.y - 0.05f) continue;
                if (!span_overlap(b.position.y, b.height, s.min.y, aabb_height(s))) continue;

                const phys3d_flat_static_shape& shape = w.static_shape[w.cell_items[k]];
                contact2 hit;
                bool touching;
                if (shape.rotated) {
                    // Diagonal prop: test the footprint it really has. Its
                    // centre is the AABB's, since aabb_from_obb bounds the box
                    // symmetrically about it.
                    vec3 mid = aabb_centre(s);
                    obb2 target = { v2(mid.x, mid.z), shape.half, shape.yaw };
                    if (b.shape == PHYS3D_FLAT_BOX) {
                        self_box.centre = v2xz(b.position);
                        touching = collide_obb_obb(self_box, target, &hit);
                    } else {
                        self_circle.centre = v2xz(b.position);
                        touching = collide_circle_obb(self_circle, target, &hit);
                    }
                } else {
                    rect2 foot = aabb_footprint(s);
                    if (b.shape == PHYS3D_FLAT_BOX) {
                        self_box.centre = v2xz(b.position);
                        touching = collide_obb_rect(self_box, foot, &hit);
                    } else {
                        self_circle.centre = v2xz(b.position);
                        touching = collide_circle_rect(self_circle, foot, &hit);
                    }
                }
                if (!touching) continue;

                // both tests report body -> wall; the body has to go the other way
                vec2 push = v2scale(hit.normal, -hit.depth);
                b.position.x += push.x;
                b.position.z += push.y;
                b.touched = true;

                vec2 n = v2scale(hit.normal, -1.0f);
                float vn = b.velocity.x * n.x + b.velocity.z * n.y;
                if (vn < 0.0f) {                       // kill only the inbound component
                    float j = -(1.0f + b.restitution) * vn;
                    b.velocity.x += n.x * j;
                    b.velocity.z += n.y * j;
                    if (j > b.impact) b.impact = j;

                    // Sliding along the face costs something. A body scraping
                    // a wall with no friction keeps every bit of the speed it
                    // had along that wall, which is how a body brushing a
                    // corner came out of the contact as fast as it went in.
                    if (b.friction > 0.0f) {
                        vec2 t = v2(-n.y, n.x);
                        float vt = b.velocity.x * t.x + b.velocity.z * t.y;
                        float cap = j * b.friction;
                        float take = vt > cap ? cap : (vt < -cap ? -cap : vt);
                        b.velocity.x -= t.x * take;
                        b.velocity.z -= t.y * take;
                    }
                }
            }
        }
    }
}

// ---- dynamic broadphase ----

static inline int phys3d_flat__dyn_cell(float x) { return (int)floorf(x / PHYS3D_FLAT_CELL); }
static inline uint32_t phys3d_flat__cell_bucket(int cx, int cz) {
    return (uint32_t)((cx * 73856093) ^ (cz * 19349663)) & (PHYS3D_FLAT_DYN_BUCKETS - 1);
}
static uint32_t phys3d_flat__dyn_bucket(float x, float z) {
    return phys3d_flat__cell_bucket(phys3d_flat__dyn_cell(x), phys3d_flat__dyn_cell(z));
}

static void phys3d_flat__build_dynamic_grid(phys3d_flat_world& w) {
    memset(w.dyn_start, 0, sizeof(uint32_t) * (PHYS3D_FLAT_DYN_BUCKETS + 1));
    for (size_t i = 0; i < w.body_count; i++) {
        if (!w.bodies[i].active) continue;
        w.dyn_start[phys3d_flat__dyn_bucket(w.bodies[i].position.x, w.bodies[i].position.z) + 1]++;
    }
    for (int i = 0; i < PHYS3D_FLAT_DYN_BUCKETS; i++) w.dyn_start[i + 1] += w.dyn_start[i];

    uint32_t* cursor = w.dyn_cursor;
    memcpy(cursor, w.dyn_start, sizeof(uint32_t) * (PHYS3D_FLAT_DYN_BUCKETS + 1));
    for (size_t i = 0; i < w.body_count; i++) {
        if (!w.bodies[i].active) continue;
        w.dyn_items[cursor[phys3d_flat__dyn_bucket(w.bodies[i].position.x, w.bodies[i].position.z)]++] = (uint32_t)i;
    }
}

// ---- pair resolution ----

static void phys3d_flat__resolve_pair(phys3d_flat_body& a, phys3d_flat_body& b) {
    if (a.inv_mass == 0.0f && b.inv_mass == 0.0f) return;
    // most pairs from the nine cells around a body are metres apart: their
    // bounding circles settle it before any shape is built
    float dx = b.position.x - a.position.x, dz = b.position.z - a.position.z;
    float reach = (a.shape == PHYS3D_FLAT_BOX ? phys3d_flat__body_radius(a) : a.radius)
                + (b.shape == PHYS3D_FLAT_BOX ? phys3d_flat__body_radius(b) : b.radius);
    if (dx * dx + dz * dz > reach * reach) return;
    if (!phys3d_flat__layers_interact(a, b)) return;

    // vertical reject first - it is the cheapest test there is, and in a world
    // where most pairs are far apart in height it throws out the majority
    if (!span_overlap(a.position.y, a.height, b.position.y, b.height)) return;

    obb2    abox  = { v2xz(a.position), a.half, a.yaw };
    obb2    bbox  = { v2xz(b.position), b.half, b.yaw };
    circle2 acirc = { v2xz(a.position), a.radius };
    circle2 bcirc = { v2xz(b.position), b.radius };

    contact2 hit;
    bool touching;
    if (a.shape == PHYS3D_FLAT_BOX && b.shape == PHYS3D_FLAT_BOX) {
        touching = collide_obb_obb(abox, bbox, &hit);
    } else if (a.shape == PHYS3D_FLAT_BOX) {
        touching = collide_circle_obb(bcirc, abox, &hit);   // reports b -> a
        if (touching) hit.normal = v2scale(hit.normal, -1.0f);
    } else if (b.shape == PHYS3D_FLAT_BOX) {
        touching = collide_circle_obb(acirc, bbox, &hit);   // already a -> b
    } else {
        touching = collide_circle_circle(acirc, bcirc, &hit);
    }
    if (!touching) return;

    a.touched = b.touched = true;

    float inv_sum = a.inv_mass + b.inv_mass;
    if (inv_sum <= 0.0f) return;

    // positional correction, split by mass
    float ca = hit.depth * (a.inv_mass / inv_sum);
    float cb = hit.depth * (b.inv_mass / inv_sum);
    a.position.x -= hit.normal.x * ca; a.position.z -= hit.normal.y * ca;
    b.position.x += hit.normal.x * cb; b.position.z += hit.normal.y * cb;

    // normal impulse
    float rvx = b.velocity.x - a.velocity.x;
    float rvz = b.velocity.z - a.velocity.z;
    float vn = rvx * hit.normal.x + rvz * hit.normal.y;
    if (vn > 0.0f) return;                             // already separating

    float e = a.restitution < b.restitution ? a.restitution : b.restitution;
    float j = -(1.0f + e) * vn / inv_sum;
    a.velocity.x -= hit.normal.x * j * a.inv_mass;
    a.velocity.z -= hit.normal.y * j * a.inv_mass;
    b.velocity.x += hit.normal.x * j * b.inv_mass;
    b.velocity.z += hit.normal.y * j * b.inv_mass;

    float felt_a = -j * a.inv_mass, felt_b = -j * b.inv_mass;
    if (felt_a > a.impact) a.impact = felt_a;
    if (felt_b > b.impact) b.impact = felt_b;

    // Tangential impulse, clamped to the Coulomb cone. This is what makes a
    // body clipped by a moving flank get dragged along with it rather than
    // squirting out sideways with all of its own speed intact.
    float mu = a.friction < b.friction ? a.friction : b.friction;
    if (mu > 0.0f) {
        vec2 t = v2(-hit.normal.y, hit.normal.x);
        float vt = (b.velocity.x - a.velocity.x) * t.x + (b.velocity.z - a.velocity.z) * t.y;
        float jt = -vt / inv_sum;
        float cap = -j * mu;                  // j is negative here; the cone is not
        if (jt > cap) jt = cap; else if (jt < -cap) jt = -cap;
        a.velocity.x -= t.x * jt * a.inv_mass;
        a.velocity.z -= t.y * jt * a.inv_mass;
        b.velocity.x += t.x * jt * b.inv_mass;
        b.velocity.z += t.y * jt * b.inv_mass;
    }
}

static void phys3d_flat__resolve_dynamics(phys3d_flat_world& w) {
    // every body checks its own bucket and the eight around it; each pair is
    // visited once by keeping only i < j
    for (size_t i = 0; i < w.body_count; i++) {
        phys3d_flat_body& a = w.bodies[i];
        if (!a.active || a.inv_mass == 0.0f) continue;

        int cx = phys3d_flat__dyn_cell(a.position.x), cz = phys3d_flat__dyn_cell(a.position.z);
        for (int oz = -1; oz <= 1; oz++)
            for (int ox = -1; ox <= 1; ox++) {
                uint32_t bucket = phys3d_flat__cell_bucket(cx + ox, cz + oz);
                for (uint32_t k = w.dyn_start[bucket]; k < w.dyn_start[bucket + 1]; k++) {
                    uint32_t j = w.dyn_items[k];
                    if (j <= i) continue;
                    phys3d_flat_body& b = w.bodies[j];
                    if (!b.active) continue;
                    phys3d_flat__resolve_pair(a, b);
                }
            }
    }
}

// ---- the phases, as job bodies ----
//
// Each takes the world plus this step's dt as its payload and a slice of the
// body array as its range, so the same code runs whether it is called once on
// the calling thread or in pieces across the pool - which is what makes
// "is a pool attached?" the only difference between the two builds.
struct phys3d_flat__phase {
    phys3d_flat_world* world;
    float       dt;
};

static void phys3d_flat__job_integrate(void* user, size_t begin, size_t end, int) {
    phys3d_flat__phase& ph = *(phys3d_flat__phase*)user;
    phys3d_flat_world& w = *ph.world;
    float dt = ph.dt;

    for (size_t i = begin; i < end; i++) {
        phys3d_flat_body& b = w.bodies[i];
        if (!b.active) continue;
        b.prev_position = b.position;
        b.touched = false;
        b.impact = 0.0f;
        if (b.inv_mass == 0.0f) continue;

        b.velocity = v3add(b.velocity, v3scale(w.gravity, b.gravity_scale * dt));
        if (b.linear_damping > 0.0f) {
            float k = 1.0f / (1.0f + b.linear_damping * dt);
            b.velocity.x *= k;
            b.velocity.z *= k;
        }

        b.position = v3add(b.position, v3scale(b.velocity, dt));

        // Where the floor is here, which need not be the same everywhere.
        // Stepping *up* onto a ledge is allowed outright rather than resolved
        // as a contact, the same allowance phys3d_flat__resolve_statics makes for
        // any static shorter than step_height.
        float floor_y = w.ground_at ? w.ground_at(w.ground_user, b.position.x, b.position.z)
                                    : w.ground_y;
        if (b.position.y <= floor_y) {
            b.position.y = floor_y;
            if (b.velocity.y < 0.0f) b.velocity.y = 0.0f;
            b.on_ground = true;
        } else {
            b.on_ground = false;
        }
    }
}

static void phys3d_flat__job_statics(void* user, size_t begin, size_t end, int) {
    phys3d_flat__phase& ph = *(phys3d_flat__phase*)user;
    phys3d_flat_world& w = *ph.world;
    for (size_t i = begin; i < end; i++) {
        phys3d_flat_body& b = w.bodies[i];
        if (b.active && b.inv_mass != 0.0f) phys3d_flat__resolve_statics(w, b);
    }
}

void phys3d_flat_step(phys3d_flat_world& w, float dt) {
    if (dt <= 0.0f) return;

    phys3d_flat__phase phase;
    phase.world = &w;
    phase.dt = dt;

    if (w.jobs) pix_parallel_for(*w.jobs, w.body_count, phys3d_flat__job_integrate, &phase, 64);
    else        phys3d_flat__job_integrate(&phase, 0, w.body_count, 0);

    phys3d_flat__build_dynamic_grid(w);

    // a couple of relaxation passes: one is enough for a lone contact, two
    // stop a body squeezed between a wall and a mover from being pushed through
    for (int pass = 0; pass < PHYS3D_FLAT_SOLVER_PASSES; pass++) {
        phys3d_flat__resolve_dynamics(w);          // writes both halves of a pair; stays serial
        if (w.jobs) pix_parallel_for(*w.jobs, w.body_count, phys3d_flat__job_statics, &phase, 32);
        else        phys3d_flat__job_statics(&phase, 0, w.body_count, 0);
    }
    w.alpha = 1.0f;
}

int phys3d_flat_update(phys3d_flat_world& w, float frame_dt) {
    w.accumulator += frame_dt;
    int steps = 0;
    while (w.accumulator >= w.fixed_dt && steps < w.max_steps) {
        phys3d_flat_step(w, w.fixed_dt);
        w.accumulator -= w.fixed_dt;
        steps++;
    }
    if (w.accumulator > w.fixed_dt) w.accumulator = w.fixed_dt;   // fell behind: drop time
    w.alpha = w.accumulator / w.fixed_dt;
    return steps;
}

// ---- queries ----

// static geometry only; also reports which box was hit
static bool phys3d_flat__raycast_statics(const phys3d_flat_world& w, vec3 origin, vec3 d,
                                         float max_distance, float* out_distance, size_t* out_static) {
    if (!w.cell_start || !w.cell_items) return false;
    vec3 inv = v3(d.x != 0.0f ? 1.0f / d.x : 1e30f,
                  d.y != 0.0f ? 1.0f / d.y : 1e30f,
                  d.z != 0.0f ? 1.0f / d.z : 1e30f);

    // walk the ray's cells in fixed steps rather than a full DDA: at the
    // distances a third person camera cares about this is a handful of cells
    float best = max_distance;
    bool found = false;
    float step = PHYS3D_FLAT_CELL * 0.5f;
    for (float t = 0.0f; t <= max_distance + step; t += step) {
        vec3 p = v3add(origin, v3scale(d, t < max_distance ? t : max_distance));
        int cx = (int)floorf(p.x / PHYS3D_FLAT_CELL) - w.grid_x0;
        int cz = (int)floorf(p.z / PHYS3D_FLAT_CELL) - w.grid_z0;
        for (int oz = -1; oz <= 1; oz++)
            for (int ox = -1; ox <= 1; ox++) {
                int gx = cx + ox, gz = cz + oz;
                if (gx < 0 || gz < 0 || gx >= w.grid_w || gz >= w.grid_h) continue;
                size_t cell = (size_t)gz * w.grid_w + gx;
                for (uint32_t k = w.cell_start[cell]; k < w.cell_start[cell + 1]; k++) {
                    float hit;
                    if (ray_vs_aabb(origin, inv, w.statics[w.cell_items[k]], best, &hit)
                        && hit < best) {
                        best = hit;
                        found = true;
                        *out_static = w.cell_items[k];
                    }
                }
            }
        if (found) break;   // the cells are walked front to back, so this is close enough
    }
    if (found && out_distance) *out_distance = best;
    return found;
}

size_t phys3d_flat_overlap(const phys3d_flat_world& w, vec3 centre, float radius,
                           idx* out, size_t capacity, uint16_t mask) {
    size_t n = 0;
    float r2 = radius * radius;
    for (size_t i = 0; i < w.body_count; i++) {
        const phys3d_flat_body& b = w.bodies[i];
        if (!b.active || !(b.layer & mask)) continue;
        float dx = b.position.x - centre.x, dz = b.position.z - centre.z;
        if (dx * dx + dz * dz > r2) continue;
        if (n < capacity) out[n] = (idx)i;
        n++;
    }
    return n;
}

size_t phys3d_flat_query_neighbours(const phys3d_flat_world& w, vec3 centre, float radius,
                                    idx* out, size_t capacity, uint16_t mask) {
    size_t n = 0;
    int reach = (int)(radius / PHYS3D_FLAT_CELL) + 1;
    float r2 = radius * radius;
    // shortcut: neighbouring cells can hash to the same bucket, so a body may
    // be counted twice in a tiny world; harmless for steering, which is the use
    int cx = phys3d_flat__dyn_cell(centre.x), cz = phys3d_flat__dyn_cell(centre.z);
    for (int oz = -reach; oz <= reach; oz++)
        for (int ox = -reach; ox <= reach; ox++) {
            uint32_t bucket = phys3d_flat__cell_bucket(cx + ox, cz + oz);
            for (uint32_t k = w.dyn_start[bucket]; k < w.dyn_start[bucket + 1]; k++) {
                uint32_t i = w.dyn_items[k];
                const phys3d_flat_body& b = w.bodies[i];
                if (!b.active || !(b.layer & mask)) continue;
                float dx = b.position.x - centre.x, dz = b.position.z - centre.z;
                if (dx * dx + dz * dz > r2) continue;
                if (n < capacity) out[n] = (idx)i;
                n++;
            }
        }
    return n;
}

bool phys3d_flat_point_blocked(const phys3d_flat_world& w, vec3 position, float radius, float height) {
    if (!w.cell_start || !w.cell_items) return false;
    int cx0 = (int)floorf((position.x - radius) / PHYS3D_FLAT_CELL) - w.grid_x0;
    int cx1 = (int)floorf((position.x + radius) / PHYS3D_FLAT_CELL) - w.grid_x0;
    int cz0 = (int)floorf((position.z - radius) / PHYS3D_FLAT_CELL) - w.grid_z0;
    int cz1 = (int)floorf((position.z + radius) / PHYS3D_FLAT_CELL) - w.grid_z0;
    for (int cz = cz0; cz <= cz1; cz++) {
        if (cz < 0 || cz >= w.grid_h) continue;
        for (int cx = cx0; cx <= cx1; cx++) {
            if (cx < 0 || cx >= w.grid_w) continue;
            size_t cell = (size_t)cz * w.grid_w + cx;
            for (uint32_t k = w.cell_start[cell]; k < w.cell_start[cell + 1]; k++) {
                const aabb& s = w.statics[w.cell_items[k]];
                if (aabb_height(s) <= w.step_height) continue;
                if (position.y >= s.max.y - 0.05f) continue;
                if (!span_overlap(position.y, height, s.min.y, aabb_height(s))) continue;
                circle2 c = { v2(position.x, position.z), radius };
                contact2 hit;
                if (collide_circle_rect(c, aabb_footprint(s), &hit)) return true;
            }
        }
    }
    return false;
}


// ---- ray against one body ----
//
// Both shapes are prisms: a footprint extruded between the feet and the head.
// So each one is a 2D problem plus a height check, which is the same
// simplification the whole solver is built on.
static bool phys3d_flat__ray_prism(vec2 o, vec2 d, float t_near, float t_far,
                            float oy, float dy, float base, float top, float* out_t) {
    // vertical slab first - it is one divide and it throws out the shots
    // that pass entirely over or under the body
    if (dy > 1e-6f || dy < -1e-6f) {
        float t0 = (base - oy) / dy, t1 = (top - oy) / dy;
        if (t0 > t1) { float k = t0; t0 = t1; t1 = k; }
        if (t0 > t_near) t_near = t0;
        if (t1 < t_far) t_far = t1;
    } else if (oy < base || oy > top) {
        return false;
    }
    (void)o; (void)d;
    if (t_far < t_near || t_far < 0.0f) return false;
    *out_t = t_near > 0.0f ? t_near : 0.0f;
    return true;
}

static bool phys3d_flat__ray_body(const phys3d_flat_body& b, vec3 origin, vec3 dir, float max_distance,
                           float* out_t) {
    vec2 o = v2(origin.x - b.position.x, origin.z - b.position.z);
    vec2 d = v2(dir.x, dir.z);
    float t_near, t_far;

    if (b.shape == PHYS3D_FLAT_CYLINDER) {
        float a = d.x * d.x + d.y * d.y;
        if (a < 1e-8f) {                       // straight up or down through the axis
            if (o.x * o.x + o.y * o.y > b.radius * b.radius) return false;
            t_near = 0.0f; t_far = max_distance;
        } else {
            float half_b = o.x * d.x + o.y * d.y;
            float c = o.x * o.x + o.y * o.y - b.radius * b.radius;
            float disc = half_b * half_b - a * c;
            if (disc < 0.0f) return false;
            float root = sqrtf(disc);
            t_near = (-half_b - root) / a;
            t_far  = (-half_b + root) / a;
        }
    } else {
        // into the box's own frame, then a plain slab test
        float cs = cosf(-b.yaw), sn = sinf(-b.yaw);
        vec2 lo = v2(o.x * cs - o.y * sn, o.x * sn + o.y * cs);
        vec2 ld = v2(d.x * cs - d.y * sn, d.x * sn + d.y * cs);
        t_near = -1e30f; t_far = 1e30f;
        const float half[2] = { b.half.x, b.half.y };
        const float po[2] = { lo.x, lo.y };
        const float pd[2] = { ld.x, ld.y };
        for (int k = 0; k < 2; k++) {
            if (pd[k] > -1e-6f && pd[k] < 1e-6f) {
                if (po[k] < -half[k] || po[k] > half[k]) return false;
                continue;
            }
            float t0 = (-half[k] - po[k]) / pd[k];
            float t1 = ( half[k] - po[k]) / pd[k];
            if (t0 > t1) { float sw = t0; t0 = t1; t1 = sw; }
            if (t0 > t_near) t_near = t0;
            if (t1 < t_far) t_far = t1;
            if (t_far < t_near) return false;
        }
    }

    if (t_far < 0.0f || t_near > max_distance) return false;
    float t;
    if (!phys3d_flat__ray_prism(o, d, t_near, t_far, origin.y, dir.y,
                         b.position.y, b.position.y + b.height, &t)) return false;
    if (t > max_distance) return false;
    *out_t = t;
    return true;
}

static idx phys3d_flat__raycast_bodies(const phys3d_flat_world& w, vec3 origin, vec3 d,
                                      float max_distance, uint16_t mask, idx ignore,
                                      float* out_distance) {
    idx best = (idx)-1;
    float best_t = max_distance;

    // A linear scan over a few hundred bodies, on purpose. Walking the hash
    // grid along the ray would mean visiting every cell the ray crosses and
    // re-testing whatever straddles two of them, which at this body count is
    // more work than the scan it replaces.
    for (size_t i = 0; i < w.body_count; i++) {
        const phys3d_flat_body& b = w.bodies[i];
        if (!b.active || (idx)i == ignore) continue;
        if (!(b.layer & mask)) continue;
        float t;
        if (!phys3d_flat__ray_body(b, origin, d, best_t, &t)) continue;
        if (t < best_t) { best_t = t; best = (idx)i; }
    }
    if (best != (idx)-1 && out_distance) *out_distance = best_t;
    return best;
}

// the face a point on a box's surface lies on, as an outward normal
static vec3 phys3d_flat__aabb_normal(const aabb& s, vec3 p) {
    float d[6] = { fabsf(p.x - s.min.x), fabsf(p.x - s.max.x), fabsf(p.y - s.min.y),
                   fabsf(p.y - s.max.y), fabsf(p.z - s.min.z), fabsf(p.z - s.max.z) };
    int best = 0;
    for (int i = 1; i < 6; i++) if (d[i] < d[best]) best = i;
    vec3 n = v3(0, 0, 0);
    (&n.x)[best / 2] = (best & 1) ? 1.0f : -1.0f;
    return n;
}

static vec3 phys3d_flat__body_normal(const phys3d_flat_body& b, vec3 p) {
    if (p.y <= b.position.y + 1e-4f) return v3(0, -1, 0);
    if (p.y >= b.position.y + b.height - 1e-4f) return v3(0, 1, 0);
    vec2 o = v2(p.x - b.position.x, p.z - b.position.z);
    if (b.shape == PHYS3D_FLAT_CYLINDER) {
        vec2 n = v2norm(o);
        return v3(n.x, 0, n.y);
    }
    // into the box's frame (local +X is (cos yaw, -sin yaw)), pick the face, back out
    float c = cosf(b.yaw), s = sinf(b.yaw);
    float lx = o.x * c - o.y * s, lz = o.x * s + o.y * c;
    vec2 ax = fabsf(lx) / b.half.x > fabsf(lz) / b.half.y ? v2(c, -s) * (lx >= 0 ? 1.0f : -1.0f)
                                                         : v2(s, c) * (lz >= 0 ? 1.0f : -1.0f);
    return v3(ax.x, 0, ax.y);
}

bool phys3d_flat_raycast(const phys3d_flat_world& w, vec3 origin, vec3 direction, float max_distance,
                         phys3d_flat_ray_hit* hit, uint16_t mask, idx ignore) {
    float len = v3len(direction);
    if (len <= 0.0f || max_distance <= 0.0f) return false;
    vec3 d = direction / len;
    float ts = max_distance, tb = max_distance;
    size_t which = 0;
    bool hit_static = phys3d_flat__raycast_statics(w, origin, d, max_distance, &ts, &which);
    idx body = phys3d_flat__raycast_bodies(w, origin, d, hit_static ? ts : max_distance, mask, ignore, &tb);
    if (!hit_static && body == PIX_INVALID) return false;
    if (hit) {
        bool is_body = body != PIX_INVALID && (!hit_static || tb < ts);
        hit->body = is_body ? body : PIX_INVALID;
        hit->distance = is_body ? tb : ts;
        hit->point = origin + d * hit->distance;
        hit->normal = is_body ? phys3d_flat__body_normal(w.bodies[body], hit->point)
                              : phys3d_flat__aabb_normal(w.statics[which], hit->point);
    }
    return true;
}

#endif // PIX_PHYSICS3D_FLAT_IMPL
