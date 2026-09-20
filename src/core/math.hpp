#pragma once
#include <math.h>
#include "dtype.hpp"

// ---- SSE, where the compiler offers it ----
//
// Only one thing in this file is worth hand-vectorising, and it is
// mat4_mul: a character rig is sixty-odd bones deep in matrix products, a
// frame poses a crowd of them, and the scalar version is sixty-four scalar
// multiplies where SSE does the same work in sixteen. Everything else here is
// three or four floats at a time, where the shuffling costs more than it
// saves - a vec3 in four lanes wastes a quarter of every instruction and the
// compiler's own auto-vectorisation already does better with the plain code.
//
// Loads are unaligned on purpose. A mat4 is a bare float[16] that lives inside
// larger structs, in arrays, and in the middle of vertex buffers, so nothing
// guarantees it is on a sixteen-byte boundary; on anything since Nehalem an
// unaligned load off an aligned address costs nothing, and off an unaligned
// one it is the only thing that works at all.
#if defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
  #if defined(__has_include)
    #if __has_include(<immintrin.h>)
      #include <immintrin.h>
      #define PIX_MATH_SSE 1
    #endif
  #else
    #include <immintrin.h>
    #define PIX_MATH_SSE 1
  #endif
#endif
#ifndef PIX_MATH_SSE
  #define PIX_MATH_SSE 0
#endif

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

static vec2 v2(float x, float y) { vec2 r = { x, y }; return r; }
// a vec3's horizontal part - the plane collision and steering queries live in
static vec2 v2xz(vec3 a) { vec2 r = { a.x, a.z }; return r; }
static vec2 v2add(vec2 a, vec2 b) { return v2(a.x + b.x, a.y + b.y); }
static vec2 v2sub(vec2 a, vec2 b) { return v2(a.x - b.x, a.y - b.y); }
static vec2 v2scale(vec2 a, float s) { return v2(a.x * s, a.y * s); }
static float v2dot(vec2 a, vec2 b) { return a.x * b.x + a.y * b.y; }
static float v2len(vec2 a) { return sqrtf(a.x * a.x + a.y * a.y); }

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
//
// Column-major is what makes the SSE form so direct: a column of the result is
// a linear combination of a's four columns, weighted by the four scalars in
// the matching column of b. So each output column is four broadcasts and four
// multiply-adds over whole columns, with no transposing and no horizontal adds
// - the shape the scalar loop below spells out one element at a time.
#if PIX_MATH_SSE
static mat4 mat4_mul(mat4 a, mat4 b) {
    mat4 r;
    __m128 a0 = _mm_loadu_ps(a.data + 0);
    __m128 a1 = _mm_loadu_ps(a.data + 4);
    __m128 a2 = _mm_loadu_ps(a.data + 8);
    __m128 a3 = _mm_loadu_ps(a.data + 12);

    for (int c = 0; c < 4; c++) {
        __m128 bc = _mm_loadu_ps(b.data + c * 4);
        __m128 out =                _mm_mul_ps(a0, _mm_shuffle_ps(bc, bc, _MM_SHUFFLE(0, 0, 0, 0)));
        out = _mm_add_ps(out,       _mm_mul_ps(a1, _mm_shuffle_ps(bc, bc, _MM_SHUFFLE(1, 1, 1, 1))));
        out = _mm_add_ps(out,       _mm_mul_ps(a2, _mm_shuffle_ps(bc, bc, _MM_SHUFFLE(2, 2, 2, 2))));
        out = _mm_add_ps(out,       _mm_mul_ps(a3, _mm_shuffle_ps(bc, bc, _MM_SHUFFLE(3, 3, 3, 3))));
        _mm_storeu_ps(r.data + c * 4, out);
    }
    return r;
}
#else
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
#endif

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

// ---------------- extra rotations ----------------

static mat4 mat4_rotate_x(float radians) {
    mat4 m = mat4_identity();
    float c = cosf(radians), s = sinf(radians);
    m.data[5] = c; m.data[9] = -s;
    m.data[6] = s; m.data[10] = c;
    return m;
}

static mat4 mat4_rotate_z(float radians) {
    mat4 m = mat4_identity();
    float c = cosf(radians), s = sinf(radians);
    m.data[0] = c; m.data[4] = -s;
    m.data[1] = s; m.data[5] = c;
    return m;
}

// the common "place a prop" transform, built without three mat4_mul calls
static mat4 mat4_trs_y(vec3 position, float yaw, float scale) {
    mat4 m = {};
    float c = cosf(yaw) * scale, s = sinf(yaw) * scale;
    m.data[0] = c;  m.data[8]  = s;
    m.data[5] = scale;
    m.data[2] = -s; m.data[10] = c;
    m.data[12] = position.x; m.data[13] = position.y; m.data[14] = position.z;
    m.data[15] = 1.0f;
    return m;
}

// same, with an independent vertical scale (squash/stretch a prop without a second mul)
static mat4 mat4_trs_y2(vec3 position, float yaw, float scale_xz, float scale_y) {
    mat4 m = mat4_trs_y(position, yaw, scale_xz);
    m.data[5] = scale_y;
    return m;
}

// same again, with all three axes independent.
//
// What this is for is fitting a model to a footprint it was not authored at.
// Scaling uniformly to fill a slot means anything shallower than the slot
// comes out narrow, leaving a gap beside it; stretching along one local axis
// closes that gap without changing how deep the model sits on the others. A
// yaw and three scales is the whole of it, still as one matrix rather than
// three multiplied together.
static mat4 mat4_trs_y3(vec3 position, float yaw, float scale_x, float scale_y, float scale_z) {
    mat4 m = {};
    float c = cosf(yaw), s = sinf(yaw);
    m.data[0] = c * scale_x;  m.data[2]  = -s * scale_x;
    m.data[5] = scale_y;
    m.data[8] = s * scale_z;  m.data[10] = c * scale_z;
    m.data[12] = position.x; m.data[13] = position.y; m.data[14] = position.z;
    m.data[15] = 1.0f;
    return m;
}

// Full 4x4 inverse (cofactor expansion). Wanted for turning a view-projection
// back into camera rays, and for undoing a bind pose; not on any hot path, so
// clarity over speed.
static mat4 mat4_inverse(const mat4& m) {
    const float* a = m.data;
    mat4 out;
    float* o = out.data;

    o[0]  =  a[5]*a[10]*a[15] - a[5]*a[11]*a[14] - a[9]*a[6]*a[15] + a[9]*a[7]*a[14] + a[13]*a[6]*a[11] - a[13]*a[7]*a[10];
    o[4]  = -a[4]*a[10]*a[15] + a[4]*a[11]*a[14] + a[8]*a[6]*a[15] - a[8]*a[7]*a[14] - a[12]*a[6]*a[11] + a[12]*a[7]*a[10];
    o[8]  =  a[4]*a[9]*a[15]  - a[4]*a[11]*a[13] - a[8]*a[5]*a[15] + a[8]*a[7]*a[13] + a[12]*a[5]*a[11] - a[12]*a[7]*a[9];
    o[12] = -a[4]*a[9]*a[14]  + a[4]*a[10]*a[13] + a[8]*a[5]*a[14] - a[8]*a[6]*a[13] - a[12]*a[5]*a[10] + a[12]*a[6]*a[9];

    o[1]  = -a[1]*a[10]*a[15] + a[1]*a[11]*a[14] + a[9]*a[2]*a[15] - a[9]*a[3]*a[14] - a[13]*a[2]*a[11] + a[13]*a[3]*a[10];
    o[5]  =  a[0]*a[10]*a[15] - a[0]*a[11]*a[14] - a[8]*a[2]*a[15] + a[8]*a[3]*a[14] + a[12]*a[2]*a[11] - a[12]*a[3]*a[10];
    o[9]  = -a[0]*a[9]*a[15]  + a[0]*a[11]*a[13] + a[8]*a[1]*a[15] - a[8]*a[3]*a[13] - a[12]*a[1]*a[11] + a[12]*a[3]*a[9];
    o[13] =  a[0]*a[9]*a[14]  - a[0]*a[10]*a[13] - a[8]*a[1]*a[14] + a[8]*a[2]*a[13] + a[12]*a[1]*a[10] - a[12]*a[2]*a[9];

    o[2]  =  a[1]*a[6]*a[15]  - a[1]*a[7]*a[14]  - a[5]*a[2]*a[15] + a[5]*a[3]*a[14] + a[13]*a[2]*a[7]  - a[13]*a[3]*a[6];
    o[6]  = -a[0]*a[6]*a[15]  + a[0]*a[7]*a[14]  + a[4]*a[2]*a[15] - a[4]*a[3]*a[14] - a[12]*a[2]*a[7]  + a[12]*a[3]*a[6];
    o[10] =  a[0]*a[5]*a[15]  - a[0]*a[7]*a[13]  - a[4]*a[1]*a[15] + a[4]*a[3]*a[13] + a[12]*a[1]*a[7]  - a[12]*a[3]*a[5];
    o[14] = -a[0]*a[5]*a[14]  + a[0]*a[6]*a[13]  + a[4]*a[1]*a[14] - a[4]*a[2]*a[13] - a[12]*a[1]*a[6]  + a[12]*a[2]*a[5];

    o[3]  = -a[1]*a[6]*a[11]  + a[1]*a[7]*a[10]  + a[5]*a[2]*a[11] - a[5]*a[3]*a[10] - a[9]*a[2]*a[7]   + a[9]*a[3]*a[6];
    o[7]  =  a[0]*a[6]*a[11]  - a[0]*a[7]*a[10]  - a[4]*a[2]*a[11] + a[4]*a[3]*a[10] + a[8]*a[2]*a[7]   - a[8]*a[3]*a[6];
    o[11] = -a[0]*a[5]*a[11]  + a[0]*a[7]*a[9]   + a[4]*a[1]*a[11] - a[4]*a[3]*a[9]  - a[8]*a[1]*a[7]   + a[8]*a[3]*a[5];
    o[15] =  a[0]*a[5]*a[10]  - a[0]*a[6]*a[9]   - a[4]*a[1]*a[10] + a[4]*a[2]*a[9]  + a[8]*a[1]*a[6]   - a[8]*a[2]*a[5];

    float det = a[0]*o[0] + a[1]*o[4] + a[2]*o[8] + a[3]*o[12];
    if (det > -1e-12f && det < 1e-12f) return mat4_identity();
    float inv = 1.0f / det;
    for (int i = 0; i < 16; i++) o[i] *= inv;
    return out;
}

static vec3 mat4_mul_point(const mat4& m, vec3 p) {
    return v3(m.data[0] * p.x + m.data[4] * p.y + m.data[8]  * p.z + m.data[12],
              m.data[1] * p.x + m.data[5] * p.y + m.data[9]  * p.z + m.data[13],
              m.data[2] * p.x + m.data[6] * p.y + m.data[10] * p.z + m.data[14]);
}

static float v3len(vec3 a) { return sqrtf(v3dot(a, a)); }
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float lerpf(float a, float b, float t) { return a + (b - a) * t; }

// shortest signed difference between two headings, wrapped to (-pi, pi]
static float angle_delta(float from, float to) {
    float d = to - from;
    while (d >  3.14159265f) d -= 6.28318531f;
    while (d < -3.14159265f) d += 6.28318531f;
    return d;
}

// exponential smoothing that behaves the same at any framerate
static float damp(float current, float target, float rate, float dt) {
    return lerpf(current, target, 1.0f - expf(-rate * dt));
}

// the same, but taking the short way round the circle so a heading never spins
// the long way to get somewhere a few degrees away
static float damp_angle(float current, float target, float rate, float dt) {
    return current + angle_delta(current, target) * (1.0f - expf(-rate * dt));
}

// ---------------- frustum ----------------

// six half-spaces in world space, each stored as (nx, ny, nz, d) with the
// interior on the positive side. Extracted from a view-projection with the
// standard Gribb/Hartmann row combinations.
struct frustum { vec4 planes[6]; };

static frustum frustum_from_viewproj(const mat4& m) {
    frustum f;
    // rows of a column-major matrix: row r = (m[r], m[4+r], m[8+r], m[12+r])
    float row[4][4];
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) row[r][c] = m.data[c * 4 + r];

    static const int SRC[6] = { 0, 0, 1, 1, 2, 2 };
    static const float SIGN[6] = { 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f };
    for (int i = 0; i < 6; i++) {
        int s = SRC[i];
        float sg = SIGN[i];
        vec4 p = v4(row[3][0] + sg * row[s][0], row[3][1] + sg * row[s][1],
                    row[3][2] + sg * row[s][2], row[3][3] + sg * row[s][3]);
        float len = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
        if (len > 1e-8f) { p.x /= len; p.y /= len; p.z /= len; p.w /= len; }
        f.planes[i] = p;
    }
    return f;
}

static bool frustum_test_sphere(const frustum& f, vec3 c, float radius) {
    for (int i = 0; i < 6; i++) {
        const vec4& p = f.planes[i];
        if (p.x * c.x + p.y * c.y + p.z * c.z + p.w < -radius) return false;
    }
    return true;
}

// Positive-vertex test: for each plane, only the box corner furthest along the
// plane normal can keep the box inside, so one corner per plane decides it.
static bool frustum_test_aabb(const frustum& f, vec3 min_corner, vec3 max_corner) {
    for (int i = 0; i < 6; i++) {
        const vec4& p = f.planes[i];
        vec3 positive = v3(p.x >= 0.0f ? max_corner.x : min_corner.x,
                           p.y >= 0.0f ? max_corner.y : min_corner.y,
                           p.z >= 0.0f ? max_corner.z : min_corner.z);
        if (p.x * positive.x + p.y * positive.y + p.z * positive.z + p.w < 0.0f)
            return false;
    }
    return true;
}
