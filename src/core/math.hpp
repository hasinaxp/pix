#pragma once
#include <math.h>
#include "dtype.hpp"

static vec3 v3(float x, float y, float z) { vec3 r = { x, y, z }; return r; }
static vec3 v3add(vec3 a, vec3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static vec3 v3sub(vec3 a, vec3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static vec3 v3scale(vec3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static float v3dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static vec3 v3cross(vec3 a, vec3 b) {
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static vec3 v3norm(vec3 a) {
    float l = sqrtf(v3dot(a, a));
    if (l < 1e-6f) l = 1.0f;
    return v3(a.x / l, a.y / l, a.z / l);
}

static vec4 v4(float x, float y, float z, float w) { vec4 r = { x, y, z, w }; return r; }

static vec3 v3lerp(vec3 a, vec3 b, float t) {
    return v3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
}

// ---------------- quaternions ----------------

static quat quat_identity() { quat q = { 0.0f, 0.0f, 0.0f, 1.0f }; return q; }

static quat quat_norm(quat q) {
    float l = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (l < 1e-8f) return quat_identity();
    float inv = 1.0f / l;
    quat r = { q.x * inv, q.y * inv, q.z * inv, q.w * inv };
    return r;
}

static quat quat_mul(quat a, quat b) {
    quat r;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    return r;
}

// shortest-arc interpolation; falls back to nlerp when the arc is tiny
static quat quat_slerp(quat a, quat b, float t) {
    float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0.0f) { b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w; d = -d; }
    float ka = 1.0f - t, kb = t;
    if (d < 0.9995f) {
        float theta = acosf(d < -1.0f ? -1.0f : (d > 1.0f ? 1.0f : d));
        float s = sinf(theta);
        if (s > 1e-6f) { ka = sinf(theta * (1.0f - t)) / s; kb = sinf(theta * t) / s; }
    }
    quat r = { a.x * ka + b.x * kb, a.y * ka + b.y * kb, a.z * ka + b.z * kb, a.w * ka + b.w * kb };
    return quat_norm(r);
}

static mat4 mat4_identity() {
    mat4 m = {};
    m.data[0] = m.data[5] = m.data[10] = m.data[15] = 1.0f;
    return m;
}

// r = a * b  (column-major, index = col*4 + row)
static mat4 mat4_mul(mat4 a, mat4 b) {
    mat4 r = {};
    for (int c = 0; c < 4; c++)
        for (int row = 0; row < 4; row++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) s += a.data[k * 4 + row] * b.data[c * 4 + k];
            r.data[c * 4 + row] = s;
        }
    return r;
}

static mat4 mat4_translate(float x, float y, float z) {
    mat4 m = mat4_identity();
    m.data[12] = x; m.data[13] = y; m.data[14] = z;
    return m;
}

static mat4 mat4_scale(float x, float y, float z) {
    mat4 m = {};
    m.data[0] = x; m.data[5] = y; m.data[10] = z; m.data[15] = 1.0f;
    return m;
}

static mat4 mat4_rotate_y(float radians) {
    mat4 m = mat4_identity();
    float c = cosf(radians), s = sinf(radians);
    m.data[0] = c; m.data[8] = s;
    m.data[2] = -s; m.data[10] = c;
    return m;
}

static mat4 mat4_perspective(float fovy, float aspect, float n, float f) {
    mat4 m = {};
    float t = 1.0f / tanf(fovy * 0.5f);
    m.data[0]  = t / aspect;
    m.data[5]  = t;
    m.data[10] = (f + n) / (n - f);
    m.data[11] = -1.0f;
    m.data[14] = (2.0f * f * n) / (n - f);
    return m;
}

// maps [left,right]x[bottom,top] to NDC; pass top=0,bottom=height for y-down screen space
static mat4 mat4_ortho(float left, float right, float bottom, float top, float n, float f) {
    mat4 m = {};
    m.data[0]  = 2.0f / (right - left);
    m.data[5]  = 2.0f / (top - bottom);
    m.data[10] = -2.0f / (f - n);
    m.data[12] = -(right + left) / (right - left);
    m.data[13] = -(top + bottom) / (top - bottom);
    m.data[14] = -(f + n) / (f - n);
    m.data[15] = 1.0f;
    return m;
}

static mat4 mat4_lookat(vec3 eye, vec3 center, vec3 up) {
    vec3 fwd = v3norm(v3sub(center, eye));
    vec3 s   = v3norm(v3cross(fwd, up));
    vec3 u   = v3cross(s, fwd);
    mat4 m = {};
    m.data[0] = s.x;    m.data[4] = s.y;    m.data[8]  = s.z;
    m.data[1] = u.x;    m.data[5] = u.y;    m.data[9]  = u.z;
    m.data[2] = -fwd.x; m.data[6] = -fwd.y; m.data[10] = -fwd.z;
    m.data[12] = -v3dot(s, eye);
    m.data[13] = -v3dot(u, eye);
    m.data[14] =  v3dot(fwd, eye);
    m.data[15] = 1.0f;
    return m;
}

static mat4 mat4_from_quat(quat q) {
    mat4 m = mat4_identity();
    float x = q.x, y = q.y, z = q.z, w = q.w;
    m.data[0] = 1.0f - 2.0f * (y * y + z * z);
    m.data[1] = 2.0f * (x * y + z * w);
    m.data[2] = 2.0f * (x * z - y * w);
    m.data[4] = 2.0f * (x * y - z * w);
    m.data[5] = 1.0f - 2.0f * (x * x + z * z);
    m.data[6] = 2.0f * (y * z + x * w);
    m.data[8] = 2.0f * (x * z + y * w);
    m.data[9] = 2.0f * (y * z - x * w);
    m.data[10] = 1.0f - 2.0f * (x * x + y * y);
    return m;
}

// T * R * S, built directly (cheaper than three mat4_mul calls)
static mat4 mat4_from_trs(vec3 t, quat r, vec3 s) {
    mat4 m = mat4_from_quat(r);
    m.data[0] *= s.x; m.data[1] *= s.x; m.data[2] *= s.x;
    m.data[4] *= s.y; m.data[5] *= s.y; m.data[6] *= s.y;
    m.data[8] *= s.z; m.data[9] *= s.z; m.data[10] *= s.z;
    m.data[12] = t.x; m.data[13] = t.y; m.data[14] = t.z;
    return m;
}

// splits an affine matrix back into TRS; assumes no shear (true for node matrices in practice)
static void mat4_decompose(const mat4& m, vec3* t, quat* r, vec3* s) {
    t->x = m.data[12]; t->y = m.data[13]; t->z = m.data[14];

    vec3 cx = v3(m.data[0], m.data[1], m.data[2]);
    vec3 cy = v3(m.data[4], m.data[5], m.data[6]);
    vec3 cz = v3(m.data[8], m.data[9], m.data[10]);
    float sx = sqrtf(v3dot(cx, cx)), sy = sqrtf(v3dot(cy, cy)), sz = sqrtf(v3dot(cz, cz));

    // a negative determinant means one axis is mirrored; fold it into x by convention
    if (v3dot(v3cross(cx, cy), cz) < 0.0f) sx = -sx;
    *s = v3(sx, sy, sz);

    float ix = (sx != 0.0f) ? 1.0f / sx : 0.0f;
    float iy = (sy != 0.0f) ? 1.0f / sy : 0.0f;
    float iz = (sz != 0.0f) ? 1.0f / sz : 0.0f;
    float m00 = m.data[0] * ix, m01 = m.data[1] * ix, m02 = m.data[2] * ix;
    float m10 = m.data[4] * iy, m11 = m.data[5] * iy, m12 = m.data[6] * iy;
    float m20 = m.data[8] * iz, m21 = m.data[9] * iz, m22 = m.data[10] * iz;

    quat q;
    float tr = m00 + m11 + m22;
    if (tr > 0.0f) {
        float k = sqrtf(tr + 1.0f) * 2.0f;
        q.w = 0.25f * k; q.x = (m12 - m21) / k; q.y = (m20 - m02) / k; q.z = (m01 - m10) / k;
    } else if (m00 > m11 && m00 > m22) {
        float k = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
        q.w = (m12 - m21) / k; q.x = 0.25f * k; q.y = (m10 + m01) / k; q.z = (m20 + m02) / k;
    } else if (m11 > m22) {
        float k = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
        q.w = (m20 - m02) / k; q.x = (m10 + m01) / k; q.y = 0.25f * k; q.z = (m21 + m12) / k;
    } else {
        float k = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
        q.w = (m01 - m10) / k; q.x = (m20 + m02) / k; q.y = (m21 + m12) / k; q.z = 0.25f * k;
    }
    *r = quat_norm(q);
}
