#ifndef PIX_COLLIDERS_HPP
#define PIX_COLLIDERS_HPP
#include <math.h>
#include "dtype.hpp"
#include "math.hpp"

// Collision shapes and the tests between them. Pure geometry: nothing here
// knows about bodies, mass, velocity or time - it answers "do these two shapes
// overlap, and by how much, in which direction". physics.hpp is the layer that
// turns those answers into motion, and anything else that needs a shape query
// (placement checks, camera probes, triggers, editor tools) can use this header
// without pulling the simulation in with it.
//
// Two families live here. The 2D tests in the XZ plane, plus a vertical band
// check, are what a broadly flat game resolves its motion with - they are
// cheaper and they never let a character drift off a floor it is standing on.
// The 3D shapes after them - sphere, capsule, plane - are for the queries that
// a flattened test gets wrong rather than merely approximates.
//
// Every test that reports an overlap returns the *minimum translation*: a unit
// normal pointing from the first shape toward the second, and the depth to push
// them apart along it.

struct aabb {
    vec3 min;
    vec3 max;
};

// axis aligned rectangle in XZ
struct rect2 {
    vec2 min;
    vec2 max;
};

struct circle2 {
    vec2  centre;
    float radius;
};

// rotated rectangle in XZ; `yaw` matches mat4_rotate_y, so local +X points
// along (cos yaw, -sin yaw) in world XZ
struct obb2 {
    vec2  centre;
    vec2  half;
    float yaw;
};

struct contact2 {
    vec2  normal;   // unit, from shape a toward shape b
    float depth;    // overlap along the normal; always > 0 on a hit
};

// ---- constructors ----

static inline aabb   aabb_make(vec3 min, vec3 max);
static inline aabb   aabb_from_centre(vec3 centre, vec3 half_extents);
static inline vec3   aabb_centre(const aabb& b);
static inline vec3   aabb_half(const aabb& b);
static inline float  aabb_height(const aabb& b);
static inline rect2  aabb_footprint(const aabb& b);
// the axis aligned bound of a rotated rectangle extruded to `height`
static inline aabb   aabb_from_obb(const obb2& box, float base_y, float height);

static inline bool   aabb_overlap(const aabb& a, const aabb& b);
static inline bool   aabb_contains_xz(const aabb& b, vec2 p);
// do two vertical spans [a0, a0+ah) and [b0, b0+bh) share any height at all
static inline bool   span_overlap(float a0, float ah, float b0, float bh);

// ---- 2D queries ----

static inline vec2 closest_point_rect(const rect2& r, vec2 p);
static inline vec2 closest_point_obb(const obb2& b, vec2 p);

static inline bool collide_circle_circle(const circle2& a, const circle2& b, contact2* out);
static inline bool collide_circle_rect(const circle2& c, const rect2& r, contact2* out);
static inline bool collide_circle_obb(const circle2& c, const obb2& b, contact2* out);
static inline bool collide_obb_obb(const obb2& a, const obb2& b, contact2* out);
static inline bool collide_obb_rect(const obb2& a, const rect2& r, contact2* out);

// ---- rays ----

// slab test; `inv_dir` is the componentwise reciprocal of a normalised
// direction, precomputed because a raycast tests the same ray against many
// boxes. *out_t is the near hit distance, clamped to 0 when the ray starts
// inside.
static inline bool ray_vs_aabb(vec3 origin, vec3 inv_dir, const aabb& box, float max_distance,
                        float* out_t);

// ---------------- implementation ----------------

static inline aabb aabb_make(vec3 min, vec3 max) { aabb b; b.min = min; b.max = max; return b; }

static inline aabb aabb_from_centre(vec3 centre, vec3 half_extents) {
    return aabb_make(v3sub(centre, half_extents), v3add(centre, half_extents));
}

static inline vec3 aabb_centre(const aabb& b) { return v3scale(v3add(b.min, b.max), 0.5f); }
static inline vec3 aabb_half(const aabb& b)   { return v3scale(v3sub(b.max, b.min), 0.5f); }
static inline float aabb_height(const aabb& b) { return b.max.y - b.min.y; }

static inline rect2 aabb_footprint(const aabb& b) {
    rect2 r;
    r.min = v2(b.min.x, b.min.z);
    r.max = v2(b.max.x, b.max.z);
    return r;
}

static inline aabb aabb_from_obb(const obb2& box, float base_y, float height) {
    // project both local axes onto world X and Z
    float c = fabsf(cosf(box.yaw)), s = fabsf(sinf(box.yaw));
    float ex = box.half.x * c + box.half.y * s;
    float ez = box.half.x * s + box.half.y * c;
    return aabb_make(v3(box.centre.x - ex, base_y, box.centre.y - ez),
                     v3(box.centre.x + ex, base_y + height, box.centre.y + ez));
}

static inline bool aabb_overlap(const aabb& a, const aabb& b) {
    return a.min.x <= b.max.x && a.max.x >= b.min.x
        && a.min.y <= b.max.y && a.max.y >= b.min.y
        && a.min.z <= b.max.z && a.max.z >= b.min.z;
}

static inline bool aabb_contains_xz(const aabb& b, vec2 p) {
    return p.x >= b.min.x && p.x <= b.max.x && p.y >= b.min.z && p.y <= b.max.z;
}

static inline bool span_overlap(float a0, float ah, float b0, float bh) {
    return a0 < b0 + bh && b0 < a0 + ah;
}

// ---- 2D ----

static inline vec2 closest_point_rect(const rect2& r, vec2 p) {
    return v2(p.x < r.min.x ? r.min.x : (p.x > r.max.x ? r.max.x : p.x),
              p.y < r.min.y ? r.min.y : (p.y > r.max.y ? r.max.y : p.y));
}

// rotates a world offset into a box's local frame. Local +X is (cos, -sin), so
// the inverse rotation is this pair of dot products.
static inline vec2 obb__to_local(const obb2& b, vec2 world) {
    float cs = cosf(b.yaw), sn = sinf(b.yaw);
    float dx = world.x - b.centre.x, dz = world.y - b.centre.y;
    return v2(dx * cs - dz * sn, dx * sn + dz * cs);
}

static inline vec2 obb__to_world_dir(const obb2& b, vec2 local) {
    float cs = cosf(b.yaw), sn = sinf(b.yaw);
    return v2(local.x * cs + local.y * sn, -local.x * sn + local.y * cs);
}

static inline vec2 closest_point_obb(const obb2& b, vec2 p) {
    rect2 local = { v2(-b.half.x, -b.half.y), v2(b.half.x, b.half.y) };
    vec2 c = closest_point_rect(local, obb__to_local(b, p));
    return v2add(b.centre, obb__to_world_dir(b, c));
}

static inline bool collide_circle_circle(const circle2& a, const circle2& b, contact2* out) {
    vec2 d = v2sub(b.centre, a.centre);
    float rr = a.radius + b.radius;
    float d2 = d.x * d.x + d.y * d.y;
    if (d2 >= rr * rr) return false;
    float len = sqrtf(d2);
    // exactly coincident centres have no meaningful normal; any axis will do
    out->normal = len > 1e-6f ? v2scale(d, 1.0f / len) : v2(1.0f, 0.0f);
    out->depth = rr - len;
    return true;
}

static inline bool collide_circle_rect(const circle2& c, const rect2& r, contact2* out) {
    vec2 closest = closest_point_rect(r, c.centre);
    vec2 d = v2sub(c.centre, closest);
    float d2 = d.x * d.x + d.y * d.y;
    if (d2 > c.radius * c.radius) return false;

    if (d2 > 1e-8f) {
        // the usual case: the centre is outside, push along the gap. Normal
        // points rect -> circle here, and is flipped below to match the
        // documented a -> b convention (a is the circle).
        float len = sqrtf(d2);
        out->normal = v2scale(d, -1.0f / len);
        out->depth = c.radius - len;
        return true;
    }

    // the centre is inside the rectangle: leave by the nearest face
    float to_min_x = c.centre.x - r.min.x, to_max_x = r.max.x - c.centre.x;
    float to_min_y = c.centre.y - r.min.y, to_max_y = r.max.y - c.centre.y;
    float best = to_min_x;
    vec2 n = v2(1.0f, 0.0f);                   // push circle toward -x => normal circle->rect is +x
    if (to_max_x < best) { best = to_max_x; n = v2(-1.0f, 0.0f); }
    if (to_min_y < best) { best = to_min_y; n = v2(0.0f, 1.0f); }
    if (to_max_y < best) { best = to_max_y; n = v2(0.0f, -1.0f); }
    out->normal = n;
    out->depth = best + c.radius;
    return true;
}

static inline bool collide_circle_obb(const circle2& c, const obb2& b, contact2* out) {
    // obb__to_local and obb__to_world_dir inlined, sharing one cos / sin
    float cs = cosf(b.yaw), sn = sinf(b.yaw);
    float dx = c.centre.x - b.centre.x, dz = c.centre.y - b.centre.y;
    circle2 local = { v2(dx * cs - dz * sn, dx * sn + dz * cs), c.radius };
    rect2 lr = { v2(-b.half.x, -b.half.y), v2(b.half.x, b.half.y) };
    contact2 hit;
    if (!collide_circle_rect(local, lr, &hit)) return false;
    out->normal = v2(hit.normal.x * cs + hit.normal.y * sn, -hit.normal.x * sn + hit.normal.y * cs);
    out->depth = hit.depth;
    return true;
}

// Separating axis test. Two rectangles cannot overlap if any of their four edge
// normals separates them, so testing all four and keeping the smallest overlap
// gives both the answer and the cheapest way out.
static inline bool collide_obb_obb(const obb2& a, const obb2& b, contact2* out) {
    vec2 axis[4];
    axis[0] = v2(cosf(a.yaw), -sinf(a.yaw));   // a's local x
    axis[1] = v2(sinf(a.yaw),  cosf(a.yaw));   // a's local z
    axis[2] = v2(cosf(b.yaw), -sinf(b.yaw));
    axis[3] = v2(sinf(b.yaw),  cosf(b.yaw));

    vec2 delta = v2sub(b.centre, a.centre);
    float best = 1e30f;
    vec2  best_axis = v2(1.0f, 0.0f);

    for (int i = 0; i < 4; i++) {
        vec2 ax = axis[i];
        float ra = a.half.x * fabsf(v2dot(ax, axis[0])) + a.half.y * fabsf(v2dot(ax, axis[1]));
        float rb = b.half.x * fabsf(v2dot(ax, axis[2])) + b.half.y * fabsf(v2dot(ax, axis[3]));
        float dist = v2dot(delta, ax);
        float overlap = ra + rb - fabsf(dist);
        if (overlap <= 0.0f) return false;
        if (overlap < best) {
            best = overlap;
            best_axis = dist < 0.0f ? v2scale(ax, -1.0f) : ax;   // always a -> b
        }
    }
    out->normal = best_axis;
    out->depth = best;
    return true;
}

static inline bool collide_obb_rect(const obb2& a, const rect2& r, contact2* out) {
    obb2 b;
    b.centre = v2scale(v2add(r.min, r.max), 0.5f);
    b.half   = v2scale(v2sub(r.max, r.min), 0.5f);
    b.yaw    = 0.0f;
    return collide_obb_obb(a, b, out);
}

// ---- rays ----

static inline bool ray_vs_aabb(vec3 origin, vec3 inv_dir, const aabb& box, float max_distance,
                        float* out_t) {
    float t0 = 0.0f, t1 = max_distance;
    const float* bmin = &box.min.x;
    const float* bmax = &box.max.x;
    const float* o    = &origin.x;
    const float* inv  = &inv_dir.x;
    for (int a = 0; a < 3; a++) {
        float ta = (bmin[a] - o[a]) * inv[a];
        float tb = (bmax[a] - o[a]) * inv[a];
        if (ta > tb) { float tmp = ta; ta = tb; tb = tmp; }
        if (ta > t0) t0 = ta;
        if (tb < t1) t1 = tb;
        if (t0 > t1) return false;
    }
    if (out_t) *out_t = t0;
    return true;
}

// ---------------- 3D shapes ----------------
//
// The 2D tests above are what a flat-ground game resolves its motion with.
// These are the other half: the queries that are genuinely three dimensional
// and that a flattened test gets wrong rather than merely approximates - a
// shot arcing over a wall, a character standing on a sloped roof, a trigger
// volume that a player can jump out of the top of.
//
// Same contract as the 2D side: a test that reports an overlap fills in the
// minimum translation - a unit normal pointing from `a` toward `b`, and the
// depth to push them apart along it - so a caller resolves any of them with
// the same two lines.

struct sphere {
    vec3  centre;
    float radius;
};

// A capsule is the set of points within `radius` of the segment a..b: a
// cylinder with a hemisphere on each end.
//
// It is stored as a segment rather than as centre/height/radius because the
// segment form is what every test below actually wants, and because it costs
// nothing to let one lie on its side or lean. An upright character capsule is
// the common case and capsule_upright builds it.
struct capsule {
    vec3  a;
    vec3  b;
    float radius;
};

// An infinite plane: the points p where dot(normal, p) == distance. `normal`
// is unit length and the positive side is the front - for a floor built with
// normal +Y, "in front" is above it.
struct plane {
    vec3  normal;
    float distance;
};

struct contact3 {
    vec3  normal;   // unit, from shape a toward shape b
    float depth;    // overlap along the normal; always > 0 on a hit
};

// ---- constructors ----

static inline sphere  sphere_make(vec3 centre, float radius);
static inline capsule capsule_make(vec3 a, vec3 b, float radius);
// the character case: a capsule standing on `base`, `height` tall overall, so
// the segment runs from one radius up to `height - radius`. A `height` under
// two radii cannot be a capsule at all and collapses to a sphere.
static inline capsule capsule_upright(vec3 base, float height, float radius);
static inline plane   plane_make(vec3 normal, float distance);
static inline plane   plane_through(vec3 normal, vec3 point);

static inline aabb sphere_bounds(const sphere& s);
static inline aabb capsule_bounds(const capsule& c);

// signed: positive in front of the plane, negative behind, zero on it
static inline float plane_distance_to(const plane& p, vec3 point);

// ---- closest points ----

static inline vec3 closest_point_segment(vec3 a, vec3 b, vec3 p);
static inline vec3 closest_point_aabb(const aabb& box, vec3 p);
static inline vec3 closest_point_plane(const plane& p, vec3 point);
// The pair of closest points between two segments, which is the one primitive
// capsule-vs-capsule needs and the only one here worth writing out. Degenerate
// cases (either segment a point, the two parallel) are handled by clamping
// rather than by branching into special cases.
static inline void closest_points_segments(vec3 p1, vec3 q1, vec3 p2, vec3 q2,
                                    vec3* out_c1, vec3* out_c2);

// ---- overlaps ----

static inline bool collide_sphere_sphere(const sphere& a, const sphere& b, contact3* out);
static inline bool collide_sphere_plane(const sphere& s, const plane& p, contact3* out);
static inline bool collide_sphere_aabb(const sphere& s, const aabb& box, contact3* out);
static inline bool collide_sphere_capsule(const sphere& s, const capsule& c, contact3* out);
static inline bool collide_capsule_capsule(const capsule& a, const capsule& b, contact3* out);
static inline bool collide_capsule_plane(const capsule& c, const plane& p, contact3* out);
// Capsule against a box, resolved as a sphere at the point on the capsule's
// axis nearest the box. That is exact whenever the contact is on a face or an
// end cap, which is nearly every contact a character makes, and slightly
// generous on an edge hit at a steep angle. It costs one iteration instead of
// the full segment-vs-box minimisation, and for a character controller the
// difference never shows.
static inline bool collide_capsule_aabb(const capsule& c, const aabb& box, contact3* out);

// ---- rays ----
//
// Each returns the near hit distance along a *normalised* direction, clamped
// to 0 when the ray starts inside the shape. `max_distance` bounds the search
// so a miss costs no more than a hit.

static inline bool ray_vs_sphere(vec3 origin, vec3 dir, const sphere& s, float max_distance,
                          float* out_t);
// one-sided is the useful default for a floor: a ray travelling with the
// normal (from behind) passes through rather than hitting the underside
static inline bool ray_vs_plane(vec3 origin, vec3 dir, const plane& p, float max_distance,
                         bool two_sided, float* out_t);
static inline bool ray_vs_capsule(vec3 origin, vec3 dir, const capsule& c, float max_distance,
                           float* out_t);

// ---------------- implementation ----------------

static inline sphere sphere_make(vec3 centre, float radius) {
    sphere s; s.centre = centre; s.radius = radius; return s;
}

static inline capsule capsule_make(vec3 a, vec3 b, float radius) {
    capsule c; c.a = a; c.b = b; c.radius = radius; return c;
}

static inline capsule capsule_upright(vec3 base, float height, float radius) {
    float half = height * 0.5f;
    if (radius > half) radius = half;           // a sphere is as tall as it gets
    float lo = base.y + radius;
    float hi = base.y + height - radius;
    if (hi < lo) hi = lo;
    return capsule_make(v3(base.x, lo, base.z), v3(base.x, hi, base.z), radius);
}

static inline plane plane_make(vec3 normal, float distance) {
    plane p; p.normal = v3norm(normal); p.distance = distance; return p;
}

static inline plane plane_through(vec3 normal, vec3 point) {
    vec3 n = v3norm(normal);
    plane p; p.normal = n; p.distance = v3dot(n, point); return p;
}

static inline aabb sphere_bounds(const sphere& s) {
    vec3 r = v3(s.radius, s.radius, s.radius);
    return aabb_make(v3sub(s.centre, r), v3add(s.centre, r));
}

static inline aabb capsule_bounds(const capsule& c) {
    vec3 lo = v3(c.a.x < c.b.x ? c.a.x : c.b.x,
                 c.a.y < c.b.y ? c.a.y : c.b.y,
                 c.a.z < c.b.z ? c.a.z : c.b.z);
    vec3 hi = v3(c.a.x > c.b.x ? c.a.x : c.b.x,
                 c.a.y > c.b.y ? c.a.y : c.b.y,
                 c.a.z > c.b.z ? c.a.z : c.b.z);
    vec3 r = v3(c.radius, c.radius, c.radius);
    return aabb_make(v3sub(lo, r), v3add(hi, r));
}

static inline float plane_distance_to(const plane& p, vec3 point) {
    return v3dot(p.normal, point) - p.distance;
}

// ---- closest points ----

static inline vec3 closest_point_segment(vec3 a, vec3 b, vec3 p) {
    vec3 ab = v3sub(b, a);
    float denom = v3dot(ab, ab);
    if (denom < 1e-12f) return a;               // the segment is a point
    float t = clampf(v3dot(v3sub(p, a), ab) / denom, 0.0f, 1.0f);
    return v3add(a, v3scale(ab, t));
}

static inline vec3 closest_point_aabb(const aabb& box, vec3 p) {
    return v3(clampf(p.x, box.min.x, box.max.x),
              clampf(p.y, box.min.y, box.max.y),
              clampf(p.z, box.min.z, box.max.z));
}

static inline vec3 closest_point_plane(const plane& p, vec3 point) {
    return v3sub(point, v3scale(p.normal, plane_distance_to(p, point)));
}

static inline void closest_points_segments(vec3 p1, vec3 q1, vec3 p2, vec3 q2,
                                    vec3* out_c1, vec3* out_c2) {
    vec3 d1 = v3sub(q1, p1);        // direction of segment 1
    vec3 d2 = v3sub(q2, p2);
    vec3 r  = v3sub(p1, p2);
    float a = v3dot(d1, d1);
    float e = v3dot(d2, d2);
    float f = v3dot(d2, r);

    float s, t;
    if (a < 1e-12f && e < 1e-12f) {             // both degenerate
        s = t = 0.0f;
    } else if (a < 1e-12f) {                    // segment 1 is a point
        s = 0.0f;
        t = clampf(f / e, 0.0f, 1.0f);
    } else {
        float c = v3dot(d1, r);
        if (e < 1e-12f) {                       // segment 2 is a point
            t = 0.0f;
            s = clampf(-c / a, 0.0f, 1.0f);
        } else {
            float b = v3dot(d1, d2);
            float denom = a * e - b * b;        // zero exactly when parallel
            // Parallel segments have a whole interval of equally close pairs;
            // any point on segment 1 will do, so take its start and let the
            // clamp below pick the matching point on segment 2.
            s = (denom > 1e-12f) ? clampf((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
            t = (b * s + f) / e;
            // t may have left [0,1]; re-clamp it and recompute s against the
            // clamped end, which is what keeps the pair on both segments
            if (t < 0.0f)      { t = 0.0f; s = clampf(-c / a, 0.0f, 1.0f); }
            else if (t > 1.0f) { t = 1.0f; s = clampf((b - c) / a, 0.0f, 1.0f); }
        }
    }
    *out_c1 = v3add(p1, v3scale(d1, s));
    *out_c2 = v3add(p2, v3scale(d2, t));
}

// ---- overlaps ----

static inline bool collide_sphere_sphere(const sphere& a, const sphere& b, contact3* out) {
    vec3 d = v3sub(b.centre, a.centre);
    float rr = a.radius + b.radius;
    float d2 = v3dot(d, d);
    if (d2 >= rr * rr) return false;
    float len = sqrtf(d2);
    // exactly coincident centres have no meaningful normal; any axis will do
    out->normal = len > 1e-6f ? v3scale(d, 1.0f / len) : v3(0.0f, 1.0f, 0.0f);
    out->depth = rr - len;
    return true;
}

static inline bool collide_sphere_plane(const sphere& s, const plane& p, contact3* out) {
    float d = plane_distance_to(p, s.centre);
    if (d > s.radius || d < -s.radius) return false;
    // normal points sphere -> plane, so it is the plane normal reversed when
    // the sphere is in front of it (the usual case: something resting on a floor)
    out->normal = d >= 0.0f ? v3scale(p.normal, -1.0f) : p.normal;
    out->depth = s.radius - (d >= 0.0f ? d : -d);
    return true;
}

static inline bool collide_sphere_aabb(const sphere& s, const aabb& box, contact3* out) {
    vec3 closest = closest_point_aabb(box, s.centre);
    vec3 d = v3sub(s.centre, closest);
    float d2 = v3dot(d, d);

    if (d2 > 1e-10f) {
        if (d2 > s.radius * s.radius) return false;
        float len = sqrtf(d2);
        out->normal = v3scale(d, -1.0f / len);  // sphere -> box
        out->depth = s.radius - len;
        return true;
    }

    // The centre is inside the box, where there is no gap to measure and the
    // shortest way out is through the nearest face.
    float dx0 = s.centre.x - box.min.x, dx1 = box.max.x - s.centre.x;
    float dy0 = s.centre.y - box.min.y, dy1 = box.max.y - s.centre.y;
    float dz0 = s.centre.z - box.min.z, dz1 = box.max.z - s.centre.z;
    float best = dx0;
    vec3 n = v3(1.0f, 0.0f, 0.0f);              // push sphere to -x => sphere->box is +x
    if (dx1 < best) { best = dx1; n = v3(-1.0f, 0.0f, 0.0f); }
    if (dy0 < best) { best = dy0; n = v3(0.0f, 1.0f, 0.0f); }
    if (dy1 < best) { best = dy1; n = v3(0.0f, -1.0f, 0.0f); }
    if (dz0 < best) { best = dz0; n = v3(0.0f, 0.0f, 1.0f); }
    if (dz1 < best) { best = dz1; n = v3(0.0f, 0.0f, -1.0f); }
    out->normal = n;
    out->depth = best + s.radius;
    return true;
}

static inline bool collide_sphere_capsule(const sphere& s, const capsule& c, contact3* out) {
    vec3 on_axis = closest_point_segment(c.a, c.b, s.centre);
    sphere as = sphere_make(on_axis, c.radius);
    return collide_sphere_sphere(s, as, out);
}

static inline bool collide_capsule_capsule(const capsule& a, const capsule& b, contact3* out) {
    vec3 ca, cb;
    closest_points_segments(a.a, a.b, b.a, b.b, &ca, &cb);
    return collide_sphere_sphere(sphere_make(ca, a.radius), sphere_make(cb, b.radius), out);
}

static inline bool collide_capsule_plane(const capsule& c, const plane& p, contact3* out) {
    // the deeper end decides it, and for a capsule lying flat on a floor both
    // ends are equally deep, which is the same answer either way
    float da = plane_distance_to(p, c.a);
    float db = plane_distance_to(p, c.b);
    vec3 deepest = (da < db) ? c.a : c.b;
    return collide_sphere_plane(sphere_make(deepest, c.radius), p, out);
}

static inline bool collide_capsule_aabb(const capsule& c, const aabb& box, contact3* out) {
    vec3 on_axis = closest_point_segment(c.a, c.b, aabb_centre(box));
    return collide_sphere_aabb(sphere_make(on_axis, c.radius), box, out);
}

// ---- rays ----

static inline bool ray_vs_sphere(vec3 origin, vec3 dir, const sphere& s, float max_distance,
                          float* out_t) {
    vec3 m = v3sub(origin, s.centre);
    float b = v3dot(m, dir);
    float c = v3dot(m, m) - s.radius * s.radius;
    // pointing away from a sphere it is already outside of
    if (c > 0.0f && b > 0.0f) return false;

    float disc = b * b - c;
    if (disc < 0.0f) return false;
    float t = -b - sqrtf(disc);
    if (t < 0.0f) t = 0.0f;                     // started inside
    if (t > max_distance) return false;
    if (out_t) *out_t = t;
    return true;
}

static inline bool ray_vs_plane(vec3 origin, vec3 dir, const plane& p, float max_distance,
                         bool two_sided, float* out_t) {
    float denom = v3dot(p.normal, dir);
    // parallel, or approaching the back face when only the front counts
    if (denom > -1e-6f && (!two_sided || denom < 1e-6f)) return false;

    float t = (p.distance - v3dot(p.normal, origin)) / denom;
    if (t < 0.0f || t > max_distance) return false;
    if (out_t) *out_t = t;
    return true;
}

static inline bool ray_vs_capsule(vec3 origin, vec3 dir, const capsule& c, float max_distance,
                           float* out_t) {
    // The capsule is an infinite cylinder clipped to the segment's span, with a
    // sphere at each end filling in what the clip cuts off. Solve the cylinder
    // first; if the hit lands outside the span, or the ray runs parallel to the
    // axis, fall back to whichever end sphere it actually meets.
    vec3 ab = v3sub(c.b, c.a);
    vec3 ao = v3sub(origin, c.a);
    float ab_ab = v3dot(ab, ab);
    if (ab_ab < 1e-12f) return ray_vs_sphere(origin, dir, sphere_make(c.a, c.radius),
                                             max_distance, out_t);

    float ab_d  = v3dot(ab, dir);
    float ab_ao = v3dot(ab, ao);

    // the ray and the axis, each with their component along the other removed
    float m = ab_d / ab_ab;
    float n = ab_ao / ab_ab;
    vec3 q = v3sub(dir, v3scale(ab, m));
    vec3 r = v3sub(ao,  v3scale(ab, n));

    float qa = v3dot(q, q);
    float qb = 2.0f * v3dot(q, r);
    float qc = v3dot(r, r) - c.radius * c.radius;

    float best = max_distance;
    bool  hit  = false;

    if (qa > 1e-12f) {                          // not parallel to the axis
        float disc = qb * qb - 4.0f * qa * qc;
        if (disc >= 0.0f) {
            float t = (-qb - sqrtf(disc)) / (2.0f * qa);
            if (t < 0.0f) t = 0.0f;
            if (t <= best) {
                // only counts if it landed between the two caps
                float along = n + t * m;
                if (along >= 0.0f && along <= 1.0f) { best = t; hit = true; }
            }
        }
    } else if (qc <= 0.0f) {
        // travelling straight along the axis from inside the cylinder's radius:
        // the end spheres below are the whole answer
        hit = false;
    }

    float t_cap;
    if (ray_vs_sphere(origin, dir, sphere_make(c.a, c.radius), best, &t_cap)) {
        best = t_cap; hit = true;
    }
    if (ray_vs_sphere(origin, dir, sphere_make(c.b, c.radius), best, &t_cap)) {
        best = t_cap; hit = true;
    }

    if (hit && out_t) *out_t = best;
    return hit;
}

// ---------------- oriented boxes ----------------
//
// A box turned to any orientation: a crate knocked over, a car on a slope, a
// trigger volume along a rotated corridor. Same contract as the shapes above:
// overlaps report the minimum translation, normal from the first shape toward
// the second.

struct obb {
    vec3 centre;
    vec3 half;   // half size along the box's own axes
    mat3 axes;   // columns: the box's local x, y, z in world space, unit length
};

static inline obb obb_make(vec3 centre, vec3 half, quat rotation) {
    obb b;
    b.centre = centre;
    b.half = half;
    b.axes = mat3_from_quat(quat_norm(rotation));
    return b;
}

// A model-space box (a mesh's bounds) carried through its transform; scale
// ends up in `half`, so the axes stay unit length.
static inline obb obb_from_aabb(const aabb& local, const mat4& transform) {
    obb b;
    b.centre = mat4_mul_point(transform, aabb_centre(local));
    vec3 h = aabb_half(local);
    vec3 cx = v3(transform.data[0], transform.data[1], transform.data[2]);
    vec3 cy = v3(transform.data[4], transform.data[5], transform.data[6]);
    vec3 cz = v3(transform.data[8], transform.data[9], transform.data[10]);
    float sx = v3len(cx), sy = v3len(cy), sz = v3len(cz);
    b.half = v3(h.x * sx, h.y * sy, h.z * sz);
    b.axes = mat3_from_cols(sx > 0.0f ? v3scale(cx, 1.0f / sx) : v3(1, 0, 0),
                            sy > 0.0f ? v3scale(cy, 1.0f / sy) : v3(0, 1, 0),
                            sz > 0.0f ? v3scale(cz, 1.0f / sz) : v3(0, 0, 1));
    return b;
}

// a world point in the box's own frame (centre at the origin, axes x, y, z)
static inline vec3 obb_to_local(const obb& b, vec3 p) {
    vec3 d = v3sub(p, b.centre);
    return v3(v3dot(d, mat3_col(b.axes, 0)), v3dot(d, mat3_col(b.axes, 1)), v3dot(d, mat3_col(b.axes, 2)));
}

// the axis aligned box around it, for broadphase grids and culling
static inline aabb obb_bounds(const obb& b) {
    // each world axis reaches as far as the three box axes lean along it
    vec3 e;
    float* ep = &e.x;
    for (int k = 0; k < 3; k++)
        ep[k] = fabsf(b.axes.data[0 + k]) * b.half.x + fabsf(b.axes.data[3 + k]) * b.half.y
              + fabsf(b.axes.data[6 + k]) * b.half.z;
    return aabb_from_centre(b.centre, e);
}

static inline vec3 closest_point_obb(const obb& b, vec3 p) {
    vec3 l = obb_to_local(b, p);
    l = v3(clampf(l.x, -b.half.x, b.half.x), clampf(l.y, -b.half.y, b.half.y), clampf(l.z, -b.half.z, b.half.z));
    return v3add(b.centre, mat3_mul_vec3(b.axes, l));
}

// Solved in the box's frame, where it is an AABB, then turned back.
static inline bool collide_sphere_obb(const sphere& s, const obb& b, contact3* out) {
    sphere local = sphere_make(obb_to_local(b, s.centre), s.radius);
    if (!collide_sphere_aabb(local, aabb_from_centre(v3(0, 0, 0), b.half), out)) return false;
    out->normal = mat3_mul_vec3(b.axes, out->normal);
    return true;
}

// how far the box reaches along a unit axis from its centre
static inline float obb__reach(const obb& b, vec3 axis) {
    return b.half.x * fabsf(v3dot(mat3_col(b.axes, 0), axis))
         + b.half.y * fabsf(v3dot(mat3_col(b.axes, 1), axis))
         + b.half.z * fabsf(v3dot(mat3_col(b.axes, 2), axis));
}

// Separating axis test over the 15 axes that can separate two boxes: each
// box's three face normals and the nine edge-edge cross products. The axis
// with the least overlap is the cheapest way out.
static inline bool collide_obb_obb(const obb& a, const obb& b, contact3* out) {
    vec3 axes[15];
    int n = 0;
    for (int i = 0; i < 3; i++) axes[n++] = mat3_col(a.axes, i);
    for (int i = 0; i < 3; i++) axes[n++] = mat3_col(b.axes, i);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            vec3 c = v3cross(mat3_col(a.axes, i), mat3_col(b.axes, j));
            float l2 = v3dot(c, c);
            if (l2 > 1e-6f) axes[n++] = v3scale(c, 1.0f / sqrtf(l2));   // parallel edges add nothing
        }
    vec3 d = v3sub(b.centre, a.centre);
    float best = 1e30f;
    vec3 best_axis = v3(0, 1, 0);
    for (int k = 0; k < n; k++) {
        float dist = v3dot(d, axes[k]);
        float overlap = obb__reach(a, axes[k]) + obb__reach(b, axes[k]) - fabsf(dist);
        if (overlap < 0.0f) return false;
        // face axes win near-ties: an edge axis only takes over when it is
        // clearly better, so resting boxes do not flicker between normals
        bool better = k < 6 ? overlap < best : overlap < best * 0.95f - 1e-4f;
        if (better) { best = overlap; best_axis = dist < 0.0f ? v3neg(axes[k]) : axes[k]; }
    }
    out->normal = best_axis;
    out->depth = best;
    return true;
}

// `dir` normalised, like the other ray tests; 0 when the ray starts inside
static inline bool ray_vs_obb(vec3 origin, vec3 dir, const obb& b, float max_distance, float* out_t) {
    vec3 o = obb_to_local(b, origin);
    vec3 ld = v3(v3dot(dir, mat3_col(b.axes, 0)), v3dot(dir, mat3_col(b.axes, 1)), v3dot(dir, mat3_col(b.axes, 2)));
    vec3 inv = v3(ld.x != 0.0f ? 1.0f / ld.x : 1e30f, ld.y != 0.0f ? 1.0f / ld.y : 1e30f,
                  ld.z != 0.0f ? 1.0f / ld.z : 1e30f);
    return ray_vs_aabb(o, inv, aabb_from_centre(v3(0, 0, 0), b.half), max_distance, out_t);
}

// ---------------- triangles ----------------
//
// For picking against real mesh geometry, standing on sloped terrain, and
// interpolating anything stored per vertex.

// unit normal of a triangle wound counter-clockwise when seen from its front
static inline vec3 triangle_normal(vec3 a, vec3 b, vec3 c) { return v3norm(v3cross(v3sub(b, a), v3sub(c, a))); }

// Barycentric weights (u, v, w) of p for triangle abc: p = a*u + b*v + c*w
// with u + v + w = 1, p taken onto the triangle's plane. All three in 0..1
// means p is inside. Interpolate uvs, normals or colours with them:
//   uv = a_uv * w.x + b_uv * w.y + c_uv * w.z
// A degenerate (zero area) triangle gives (1, 0, 0).
static inline vec3 barycentric(vec3 p, vec3 a, vec3 b, vec3 c) {
    vec3 e0 = v3sub(b, a), e1 = v3sub(c, a), e2 = v3sub(p, a);
    float d00 = v3dot(e0, e0), d01 = v3dot(e0, e1), d11 = v3dot(e1, e1);
    float d20 = v3dot(e2, e0), d21 = v3dot(e2, e1);
    float denom = d00 * d11 - d01 * d01;
    if (fabsf(denom) < 1e-20f) return v3(1.0f, 0.0f, 0.0f);
    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    return v3(1.0f - v - w, v, w);
}

// The point of triangle abc nearest p: inside it, on an edge, or a corner
// (Ericson, Real-Time Collision Detection 5.1.5).
static inline vec3 closest_point_triangle(vec3 p, vec3 a, vec3 b, vec3 c) {
    vec3 ab = v3sub(b, a), ac = v3sub(c, a), ap = v3sub(p, a);
    float d1 = v3dot(ab, ap), d2 = v3dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;                              // corner a
    vec3 bp = v3sub(p, b);
    float d3 = v3dot(ab, bp), d4 = v3dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;                                // corner b
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)                          // edge ab
        return v3add(a, v3scale(ab, d1 / (d1 - d3)));
    vec3 cp = v3sub(p, c);
    float d5 = v3dot(ab, cp), d6 = v3dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;                                // corner c
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)                          // edge ac
        return v3add(a, v3scale(ac, d2 / (d2 - d6)));
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)            // edge bc
        return v3add(b, v3scale(v3sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
    float k = 1.0f / (va + vb + vc);                                     // inside the face
    return v3add(a, v3add(v3scale(ab, vb * k), v3scale(ac, vc * k)));
}

// normal from the sphere toward the triangle, like collide_sphere_aabb
static inline bool collide_sphere_triangle(const sphere& s, vec3 a, vec3 b, vec3 c, contact3* out) {
    vec3 q = closest_point_triangle(s.centre, a, b, c);
    vec3 d = v3sub(q, s.centre);
    float d2 = v3dot(d, d);
    if (d2 > s.radius * s.radius) return false;
    if (d2 > 1e-12f) {
        float len = sqrtf(d2);
        out->normal = v3scale(d, 1.0f / len);
        out->depth = s.radius - len;
    } else {                                    // centre on the surface: push out the front
        out->normal = v3neg(triangle_normal(a, b, c));
        out->depth = s.radius;
    }
    return true;
}

// Moller-Trumbore. `dir` normalised. The front face is the side the
// triangle's corners run counter-clockwise from; one-sided tests skip rays
// that come from behind. `out_bary`, if given, receives the hit's barycentric
// weights for interpolating per-vertex data (uv under the mouse, say).
static inline bool ray_vs_triangle(vec3 origin, vec3 dir, vec3 a, vec3 b, vec3 c, float max_distance,
                                   bool two_sided, float* out_t, vec3* out_bary = nullptr) {
    vec3 e1 = v3sub(b, a), e2 = v3sub(c, a);
    vec3 pv = v3cross(dir, e2);
    float det = v3dot(e1, pv);                  // > 0 when the ray meets the front
    if (two_sided ? fabsf(det) < 1e-12f : det < 1e-12f) return false;
    float inv = 1.0f / det;
    vec3 tv = v3sub(origin, a);
    float u = v3dot(tv, pv) * inv;
    if (u < 0.0f || u > 1.0f) return false;
    vec3 qv = v3cross(tv, e1);
    float v = v3dot(dir, qv) * inv;
    if (v < 0.0f || u + v > 1.0f) return false;
    float t = v3dot(e2, qv) * inv;
    if (t < 0.0f || t > max_distance) return false;
    if (out_t) *out_t = t;
    if (out_bary) *out_bary = v3(1.0f - u - v, u, v);
    return true;
}

#endif // PIX_COLLIDERS_HPP
