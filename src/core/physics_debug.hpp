#ifndef PIX_PHYSICS_DEBUG_HPP
#define PIX_PHYSICS_DEBUG_HPP
#include "physics.hpp"
#include "draw2d.hpp"
#include "renderer.hpp"

// Debug drawing for the physics worlds: the one place physics meets
// rendering, kept apart so physics.hpp itself needs no graphics at all (a
// server or a tool can simulate without linking OpenGL). The app layer wraps
// these as pix_draw_physics2d / pix_draw_physics3d / pix_draw_physics3d_flat.

PIXDEF void phys2d_debug_draw(pix_batch2d& batch, const phys2d_world& w,
                              unsigned flags = PHYS2D_DRAW_DEFAULT, float thickness = 1.0f);
PIXDEF void phys3d_debug_draw(pix_renderer& r, const phys3d_world& w, unsigned flags = PHYS3D_DRAW_DEFAULT);
PIXDEF void phys3d_flat_debug_draw(pix_renderer& r, const phys3d_flat_world& w);

#endif // PIX_PHYSICS_DEBUG_HPP


#if defined(PIX_IMPLEMENTATION) && !defined(PIX_PHYSICS_DEBUG_IMPL)
#define PIX_PHYSICS_DEBUG_IMPL

// ======== 2D ========

// ---- debug draw ----

void phys2d_debug_draw(pix_batch2d& batch, const phys2d_world& w, unsigned flags, float thickness) {
    const pix_color static_c = pix_rgba(140, 150, 165), dynamic_c = pix_rgba(110, 220, 120),
                    ground_c = pix_rgba(240, 200, 80), kinematic_c = pix_rgba(110, 170, 255),
                    trigger_c = pix_rgba(80, 220, 230, 160), sleeping_c = pix_rgba(70, 120, 85),
                    joint_c = pix_rgba(230, 120, 255);
    if (flags & PHYS2D_DRAW_JOINTS)
        for (size_t i = 0; i < w.joint_count; i++) {
            const phys2d_joint& j = w.joints[i];
            if (!j.active) continue;
            vec2 pa, pb;
            phys2d_joint_anchors(w, j, &pa, &pb);
            if (j.a != PIX_INVALID) pix_batch_line(batch, w.bodies[j.a].position, pa, thickness, pix_fade(joint_c, 0.4f));
            if (j.b != PIX_INVALID) pix_batch_line(batch, w.bodies[j.b].position, pb, thickness, pix_fade(joint_c, 0.4f));
            pix_batch_line(batch, pa, pb, thickness, joint_c);
            pix_batch_circle(batch, pa, thickness * 2.5f, joint_c);
            pix_batch_circle(batch, pb, thickness * 2.5f, joint_c);
        }
    if (flags & PHYS2D_DRAW_GRID) {
        const phys2d__grid& g = w.moving_grid;
        for (size_t i = 0; i < g.count; i++) {
            pix_rect cell = { g.items[i].cx * w.cell_size, g.items[i].cy * w.cell_size, w.cell_size, w.cell_size };
            pix_batch_rect_lines(batch, cell, thickness, pix_rgba(255, 255, 255, 30));
        }
    }
    for (size_t i = 0; i < w.body_count; i++) {
        const phys2d_body& b = w.bodies[i];
        if (!b.active) continue;
        if (flags & PHYS2D_DRAW_AABBS) {
            pix_rect r = { b.bounds.min.x, b.bounds.min.y, b.bounds.max.x - b.bounds.min.x, b.bounds.max.y - b.bounds.min.y };
            pix_batch_rect_lines(batch, r, thickness, pix_rgba(255, 120, 200, 120));
        }
        if (!(flags & PHYS2D_DRAW_SHAPES)) continue;
        pix_color c = b.trigger ? trigger_c : b.type == PHYS2D_STATIC ? static_c
                    : b.type == PHYS2D_KINEMATIC ? kinematic_c : b.sleeping ? sleeping_c
                    : b.on_ground ? ground_c : dynamic_c;
        phys2d__poly p;
        phys2d__make_poly(b.shape, b.position, b.angle, p);
        float a = b.shape.type == PHYS2D_AABB ? 0.0f : b.angle;
        vec2 facing = v2(cosf(a), sinf(a));
        if (p.count == 1) {
            pix_batch_circle_lines(batch, p.v[0], p.radius, thickness, c);
            pix_batch_line(batch, p.v[0], p.v[0] + facing * p.radius, thickness, c);
        } else if (p.count == 2) {
            // the capsule outline: a half circle at each end, joined
            vec2 pts[34];
            vec2 u = v2norm(p.v[1] - p.v[0]);
            float base = atan2f(u.y, u.x);
            for (int k = 0; k <= 16; k++) {
                float t = base - PIX_PI * 0.5f + PIX_PI * k / 16.0f;
                pts[k] = p.v[1] + v2(cosf(t), sinf(t)) * p.radius;
                pts[17 + k] = p.v[0] - v2(cosf(t), sinf(t)) * p.radius;
            }
            pix_batch_polyline(batch, pts, 34, thickness, c, true);
            vec2 mid = (p.v[0] + p.v[1]) * 0.5f;
            pix_batch_line(batch, mid, mid + facing * p.radius, thickness, c);
        } else {
            pix_batch_polyline(batch, p.v, p.count, thickness, c, true);
            float reach = b.shape.type == PHYS2D_POLYGON ? 0.5f * v2len(p.v[0] - b.position) : minf(b.shape.half.x, b.shape.half.y);
            pix_batch_line(batch, b.position, b.position + facing * reach, thickness, c);
        }
    }
    if (flags & PHYS2D_DRAW_CONTACTS)
        for (size_t i = 0; i < w.contact_count; i++) {
            const phys2d_contact& c = w.contacts[i];
            if (c.trigger) continue;
            for (int k = 0; k < c.count; k++) {
                const phys2d_contact_point& p = c.points[k];
                bool real = p.separation <= w.slop;
                pix_batch_circle(batch, p.point, thickness * 2.5f, real ? pix_rgba(255, 70, 60) : pix_rgba(255, 220, 60, 140));
                if (real) pix_batch_line(batch, p.point, p.point + c.normal * 12.0f * thickness, thickness, pix_rgba(255, 70, 60));
            }
        }
}


// ======== 3D ========

// ---- debug draw ----

static void phys3d__draw_circle(pix_renderer& r, vec3 c, vec3 u, vec3 v, float radius, pix_color col, int segs = 16) {
    vec3 prev = c + u * radius;
    for (int i = 1; i <= segs; i++) {
        float a = 2.0f * PIX_PI * i / segs;
        vec3 p = c + (u * cosf(a) + v * sinf(a)) * radius;
        push_line(r, prev, p, col);
        prev = p;
    }
}

// one shape's outline; compounds draw each part
static void phys3d__draw_shape(pix_renderer& r, const phys3d_shape& s, vec3 pos, quat rot, pix_color c) {
    vec3 ax = quat_rotate(rot, v3(1, 0, 0)), ay = quat_rotate(rot, v3(0, 1, 0)), az = quat_rotate(rot, v3(0, 0, 1));
    if (s.type == PHYS3D_SPHERE) {
        phys3d__draw_circle(r, pos, ax, ay, s.radius, c);
        phys3d__draw_circle(r, pos, ay, az, s.radius, c);
        phys3d__draw_circle(r, pos, az, ax, s.radius, c);
    } else if (s.type == PHYS3D_CAPSULE) {
        vec3 top = pos + ay * s.half_length, bot = pos - ay * s.half_length;
        phys3d__draw_circle(r, top, ax, az, s.radius, c);
        phys3d__draw_circle(r, bot, ax, az, s.radius, c);
        phys3d__draw_circle(r, top, ax, ay, s.radius, c);
        phys3d__draw_circle(r, bot, ax, ay, s.radius, c);
        phys3d__draw_circle(r, top, az, ay, s.radius, c);
        phys3d__draw_circle(r, bot, az, ay, s.radius, c);
        vec3 sides[4] = { ax, -ax, az, -az };
        for (int k = 0; k < 4; k++) push_line(r, bot + sides[k] * s.radius, top + sides[k] * s.radius, c);
    } else if (s.type == PHYS3D_BOX) {
        phys3d__core core;
        phys3d__make_core(s, pos, rot, core);
        for (int a = 0; a < 8; a++)
            for (int k = 0; k < 3; k++) {
                int bb = a | (1 << k);
                if (bb != a) push_line(r, core.v[a], core.v[bb], c);
            }
    } else if (s.type == PHYS3D_PLANE) {
        vec3 n = quat_rotate(rot, s.normal), u, v;
        phys3d__basis(n, &u, &v);
        for (int k = -5; k <= 5; k++) {
            push_line(r, pos + u * (k * 2.0f) - v * 10.0f, pos + u * (k * 2.0f) + v * 10.0f, c);
            push_line(r, pos + v * (k * 2.0f) - u * 10.0f, pos + v * (k * 2.0f) + u * 10.0f, c);
        }
    } else if (s.type == PHYS3D_HULL) {
        const phys3d_hull& h = *s.hull;
        for (int e = 0; e < h.edge_count; e++)
            push_line(r, pos + quat_rotate(rot, h.vertices[h.edges[e].v0]), pos + quat_rotate(rot, h.vertices[h.edges[e].v1]), c);
    } else if (s.type == PHYS3D_COMPOUND) {
        for (int i = 0; i < s.compound->count; i++) {
            const phys3d_compound_child& ch = s.compound->children[i];
            phys3d__draw_shape(r, ch.shape, pos + quat_rotate(rot, ch.position), quat_mul(rot, ch.rotation), c);
        }
    }
}

void phys3d_debug_draw(pix_renderer& r, const phys3d_world& w, unsigned flags) {
    const pix_color static_c = pix_rgba(140, 150, 165), dynamic_c = pix_rgba(110, 220, 120), ground_c = pix_rgba(240, 200, 80),
                    kinematic_c = pix_rgba(110, 170, 255), trigger_c = pix_rgba(80, 220, 230), sleeping_c = pix_rgba(70, 120, 85),
                    joint_c = pix_rgba(230, 120, 255);
    for (size_t i = 0; i < w.body_count; i++) {
        const phys3d_body& b = w.bodies[i];
        if (!b.active) continue;
        if ((flags & PHYS3D_DRAW_AABBS) && !b.oversize) push_aabb_lines(r, b.bounds.min, b.bounds.max, pix_rgba(255, 120, 200));
        pix_color c = b.trigger ? trigger_c : b.type == PHYS3D_STATIC ? static_c : b.type == PHYS3D_KINEMATIC ? kinematic_c
                    : b.sleeping ? sleeping_c : b.on_ground ? ground_c : dynamic_c;
        const phys3d_shape& s = b.shape;
        if (s.type == PHYS3D_MESH) {
            if (!(flags & PHYS3D_DRAW_MESHES)) continue;
            const phys3d_mesh& m = *s.mesh;
            for (size_t t = 0; t < m.triangle_count; t++) {
                vec3 p[3];
                for (int k = 0; k < 3; k++) p[k] = b.position + quat_rotate(b.rotation, m.vertices[m.indices[t * 3 + k]]);
                push_line(r, p[0], p[1], c); push_line(r, p[1], p[2], c); push_line(r, p[2], p[0], c);
            }
            continue;
        }
        if (!(flags & PHYS3D_DRAW_SHAPES)) continue;
        phys3d__draw_shape(r, s, b.position, b.rotation, c);
    }
    if (flags & PHYS3D_DRAW_CONTACTS)
        for (size_t i = 0; i < w.contact_count; i++) {
            const phys3d_contact& c = w.contacts[i];
            if (c.trigger) continue;
            for (int k = 0; k < c.count; k++) {
                const phys3d_contact_point& p = c.points[k];
                bool real = p.separation <= w.slop;
                pix_color col = real ? pix_rgba(255, 70, 60) : pix_rgba(255, 220, 60);
                push_line(r, p.point, p.point + c.normal * (real ? 0.25f : 0.08f), col);
                push_line(r, p.point - c.tangent[0] * 0.04f, p.point + c.tangent[0] * 0.04f, col);
                push_line(r, p.point - c.tangent[1] * 0.04f, p.point + c.tangent[1] * 0.04f, col);
            }
        }
    if (flags & PHYS3D_DRAW_JOINTS)
        for (size_t i = 0; i < w.joint_count; i++) {
            const phys3d_joint& j = w.joints[i];
            if (!j.active) continue;
            vec3 pa, pb;
            phys3d_joint_anchors(w, j, &pa, &pb);
            if (j.a != PIX_INVALID) push_line(r, w.bodies[j.a].position, pa, joint_c);
            if (j.b != PIX_INVALID) push_line(r, w.bodies[j.b].position, pb, joint_c);
            push_line(r, pa, pb, joint_c);
            if (j.type == PHYS3D_JOINT_HINGE || j.type == PHYS3D_JOINT_SLIDER) {
                vec3 ax = quat_rotate(phys3d__jbody(w, j.a).rotation, j.local_axis_a);
                push_line(r, pa - ax * 0.3f, pa + ax * 0.3f, pix_rgba(255, 255, 255));
            }
        }
}


// ======== flat 3D ========

// ---- debug draw ----

void phys3d_flat_debug_draw(pix_renderer& r, const phys3d_flat_world& w) {
    const pix_color static_c = pix_rgba(140, 150, 165), body_c = pix_rgba(110, 220, 120),
                    ground_c = pix_rgba(240, 200, 80), fixed_c = pix_rgba(110, 170, 255);
    for (size_t i = 0; i < w.static_count; i++) {
        const phys3d_flat_static_shape& sh = w.static_shape[i];
        const aabb& s = w.statics[i];
        if (!sh.rotated) { push_aabb_lines(r, s.min, s.max, static_c); continue; }
        // the true yawed footprint, extruded
        vec3 c = aabb_centre(s);
        float cs = cosf(sh.yaw), sn = sinf(sh.yaw);
        vec2 ax = v2(cs, -sn) * sh.half.x, az = v2(sn, cs) * sh.half.y;
        vec2 f[4] = { v2(c.x, c.z) + ax + az, v2(c.x, c.z) - ax + az, v2(c.x, c.z) - ax - az, v2(c.x, c.z) + ax - az };
        for (int k = 0; k < 4; k++) {
            vec2 a = f[k], b = f[(k + 1) % 4];
            push_line(r, v3(a.x, s.min.y, a.y), v3(b.x, s.min.y, b.y), static_c);
            push_line(r, v3(a.x, s.max.y, a.y), v3(b.x, s.max.y, b.y), static_c);
            push_line(r, v3(a.x, s.min.y, a.y), v3(a.x, s.max.y, a.y), static_c);
        }
    }
    for (size_t i = 0; i < w.body_count; i++) {
        const phys3d_flat_body& b = w.bodies[i];
        if (!b.active) continue;
        pix_color c = b.inv_mass == 0.0f ? fixed_c : b.on_ground ? ground_c : body_c;
        vec3 p = b.position, top = v3(0, b.height, 0);
        if (b.shape == PHYS3D_FLAT_CYLINDER) {
            const int segs = 16;
            for (int k = 0; k < segs; k++) {
                float a0 = 2.0f * PIX_PI * k / segs, a1 = 2.0f * PIX_PI * (k + 1) / segs;
                vec3 e0 = v3(cosf(a0), 0, sinf(a0)) * b.radius, e1 = v3(cosf(a1), 0, sinf(a1)) * b.radius;
                push_line(r, p + e0, p + e1, c);
                push_line(r, p + e0 + top, p + e1 + top, c);
                if (k % 4 == 0) push_line(r, p + e0, p + e0 + top, c);
            }
        } else {
            float cs = cosf(b.yaw), sn = sinf(b.yaw);
            vec3 ax = v3(cs, 0, -sn) * b.half.x, az = v3(sn, 0, cs) * b.half.y;
            vec3 f[4] = { p + ax + az, p - ax + az, p - ax - az, p + ax - az };
            for (int k = 0; k < 4; k++) {
                push_line(r, f[k], f[(k + 1) % 4], c);
                push_line(r, f[k] + top, f[(k + 1) % 4] + top, c);
                push_line(r, f[k], f[k] + top, c);
            }
        }
    }
}


#endif // PIX_PHYSICS_DEBUG_IMPL
