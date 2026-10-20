#ifndef PIX_MATH_HPP
#define PIX_MATH_HPP
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

static inline vec3 v3(float x, float y, float z) { vec3 r = { x, y, z }; return r; }
static inline vec3 v3add(vec3 a, vec3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline vec3 v3sub(vec3 a, vec3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline vec3 v3scale(vec3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float v3dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline vec3 v3cross(vec3 a, vec3 b) {
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static inline vec3 v3norm(vec3 a) {
    float l = sqrtf(v3dot(a, a));
    if (l < 1e-6f) l = 1.0f;
    return v3(a.x / l, a.y / l, a.z / l);
}

static inline vec4 v4(float x, float y, float z, float w) { vec4 r = { x, y, z, w }; return r; }

static inline vec2 v2(float x, float y) { vec2 r = { x, y }; return r; }
// a vec3's horizontal part - the plane collision and steering queries live in
static inline vec2 v2xz(vec3 a) { vec2 r = { a.x, a.z }; return r; }
static inline vec2 v2add(vec2 a, vec2 b) { return v2(a.x + b.x, a.y + b.y); }
static inline vec2 v2sub(vec2 a, vec2 b) { return v2(a.x - b.x, a.y - b.y); }
static inline vec2 v2scale(vec2 a, float s) { return v2(a.x * s, a.y * s); }
static inline float v2dot(vec2 a, vec2 b) { return a.x * b.x + a.y * b.y; }
static inline float v2len(vec2 a) { return sqrtf(a.x * a.x + a.y * a.y); }

static inline vec3 v3lerp(vec3 a, vec3 b, float t) {
    return v3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
}

// ---------------- quaternions ----------------

static inline quat quat_identity() { quat q = { 0.0f, 0.0f, 0.0f, 1.0f }; return q; }

static inline quat quat_norm(quat q) {
    float l = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (l < 1e-8f) return quat_identity();
    float inv = 1.0f / l;
    quat r = { q.x * inv, q.y * inv, q.z * inv, q.w * inv };
    return r;
}

static inline quat quat_mul(quat a, quat b) {
    quat r;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    return r;
}

// shortest-arc interpolation; falls back to nlerp when the arc is tiny
static inline quat quat_slerp(quat a, quat b, float t) {
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

static inline mat4 mat4_identity() {
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
static inline mat4 mat4_mul(mat4 a, mat4 b) {
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
static inline mat4 mat4_mul(mat4 a, mat4 b) {
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

static inline mat4 mat4_translate(float x, float y, float z) {
    mat4 m = mat4_identity();
    m.data[12] = x; m.data[13] = y; m.data[14] = z;
    return m;
}

static inline mat4 mat4_scale(float x, float y, float z) {
    mat4 m = {};
    m.data[0] = x; m.data[5] = y; m.data[10] = z; m.data[15] = 1.0f;
    return m;
}

static inline mat4 mat4_rotate_y(float radians) {
    mat4 m = mat4_identity();
    float c = cosf(radians), s = sinf(radians);
    m.data[0] = c; m.data[8] = s;
    m.data[2] = -s; m.data[10] = c;
    return m;
}

static inline mat4 mat4_perspective(float fovy, float aspect, float n, float f) {
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
static inline mat4 mat4_ortho(float left, float right, float bottom, float top, float n, float f) {
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

static inline mat4 mat4_lookat(vec3 eye, vec3 center, vec3 up) {
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

static inline mat4 mat4_from_quat(quat q) {
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
static inline mat4 mat4_from_trs(vec3 t, quat r, vec3 s) {
    mat4 m = mat4_from_quat(r);
    m.data[0] *= s.x; m.data[1] *= s.x; m.data[2] *= s.x;
    m.data[4] *= s.y; m.data[5] *= s.y; m.data[6] *= s.y;
    m.data[8] *= s.z; m.data[9] *= s.z; m.data[10] *= s.z;
    m.data[12] = t.x; m.data[13] = t.y; m.data[14] = t.z;
    return m;
}

// splits an affine matrix back into TRS; assumes no shear (true for node matrices in practice)
static inline void mat4_decompose(const mat4& m, vec3* t, quat* r, vec3* s) {
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

static inline mat4 mat4_rotate_x(float radians) {
    mat4 m = mat4_identity();
    float c = cosf(radians), s = sinf(radians);
    m.data[5] = c; m.data[9] = -s;
    m.data[6] = s; m.data[10] = c;
    return m;
}

static inline mat4 mat4_rotate_z(float radians) {
    mat4 m = mat4_identity();
    float c = cosf(radians), s = sinf(radians);
    m.data[0] = c; m.data[4] = -s;
    m.data[1] = s; m.data[5] = c;
    return m;
}

// the common "place a prop" transform, built without three mat4_mul calls
static inline mat4 mat4_trs_y(vec3 position, float yaw, float scale) {
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
static inline mat4 mat4_trs_y2(vec3 position, float yaw, float scale_xz, float scale_y) {
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
static inline mat4 mat4_trs_y3(vec3 position, float yaw, float scale_x, float scale_y, float scale_z) {
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
static inline mat4 mat4_inverse(const mat4& m) {
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

static inline vec3 mat4_mul_point(const mat4& m, vec3 p) {
    return v3(m.data[0] * p.x + m.data[4] * p.y + m.data[8]  * p.z + m.data[12],
              m.data[1] * p.x + m.data[5] * p.y + m.data[9]  * p.z + m.data[13],
              m.data[2] * p.x + m.data[6] * p.y + m.data[10] * p.z + m.data[14]);
}

static inline float v3len(vec3 a) { return sqrtf(v3dot(a, a)); }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

// shortest signed difference between two headings, wrapped to (-pi, pi]
static inline float angle_delta(float from, float to) {
    float d = to - from;
    while (d >  3.14159265f) d -= 6.28318531f;
    while (d < -3.14159265f) d += 6.28318531f;
    return d;
}

// exponential smoothing that behaves the same at any framerate
static inline float damp(float current, float target, float rate, float dt) {
    return lerpf(current, target, 1.0f - expf(-rate * dt));
}

// the same, but taking the short way round the circle so a heading never spins
// the long way to get somewhere a few degrees away
static inline float damp_angle(float current, float target, float rate, float dt) {
    return current + angle_delta(current, target) * (1.0f - expf(-rate * dt));
}

// ---------------- frustum ----------------

// six half-spaces in world space, each stored as (nx, ny, nz, d) with the
// interior on the positive side. Extracted from a view-projection with the
// standard Gribb/Hartmann row combinations.
struct frustum { vec4 planes[6]; };

static inline frustum frustum_from_viewproj(const mat4& m) {
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

static inline bool frustum_test_sphere(const frustum& f, vec3 c, float radius) {
    for (int i = 0; i < 6; i++) {
        const vec4& p = f.planes[i];
        if (p.x * c.x + p.y * c.y + p.z * c.z + p.w < -radius) return false;
    }
    return true;
}

// Positive-vertex test: for each plane, only the box corner furthest along the
// plane normal can keep the box inside, so one corner per plane decides it.
static inline bool frustum_test_aabb(const frustum& f, vec3 min_corner, vec3 max_corner) {
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

// ---------------- more vector helpers ----------------

static inline vec3 v3neg(vec3 a) { return v3(-a.x, -a.y, -a.z); }
static inline vec3 v3mul(vec3 a, vec3 b) { return v3(a.x * b.x, a.y * b.y, a.z * b.z); }
static inline vec3 v3min(vec3 a, vec3 b) { return v3(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z); }
static inline vec3 v3max(vec3 a, vec3 b) { return v3(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z); }
static inline float v3dist(vec3 a, vec3 b) { return v3len(v3sub(a, b)); }
static inline vec2 v2norm(vec2 a) {
    float l = v2len(a);
    return l < 1e-6f ? v2(0.0f, 0.0f) : v2(a.x / l, a.y / l);
}
static inline vec2 v2rotate(vec2 a, float radians) {
    float c = cosf(radians), s = sinf(radians);
    return v2(a.x * c - a.y * s, a.x * s + a.y * c);
}
static inline vec2 v2lerp(vec2 a, vec2 b, float t) { return v2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t); }
static inline float v2dist(vec2 a, vec2 b) { return v2len(v2sub(a, b)); }
static inline vec4 v4lerp(vec4 a, vec4 b, float t) {
    return v4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
}

#define PIX_PI      3.14159265358979f
#define PIX_TAU     6.28318530717959f
#define PIX_DEG2RAD (PIX_PI / 180.0f)
#define PIX_RAD2DEG (180.0f / PIX_PI)

static inline float minf(float a, float b) { return a < b ? a : b; }
static inline float maxf(float a, float b) { return a > b ? a : b; }
static inline float smoothstepf(float e0, float e1, float x) {
    float t = clampf((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}
// moves `current` toward `target` by at most `max_step`, never overshooting
static inline float approachf(float current, float target, float max_step) {
    if (current < target) return current + max_step < target ? current + max_step : target;
    return current - max_step > target ? current - max_step : target;
}

// ---------------- more quaternion / matrix helpers ----------------

static inline quat quat_from_axis_angle(vec3 axis, float radians) {
    vec3 a = v3norm(axis);
    float s = sinf(radians * 0.5f);
    quat q = { a.x * s, a.y * s, a.z * s, cosf(radians * 0.5f) };
    return q;
}

// yaw about +Y, then pitch about +X, then roll about +Z (the usual game order)
static inline quat quat_from_euler(float pitch, float yaw, float roll) {
    quat qy = quat_from_axis_angle(v3(0.0f, 1.0f, 0.0f), yaw);
    quat qx = quat_from_axis_angle(v3(1.0f, 0.0f, 0.0f), pitch);
    quat qz = quat_from_axis_angle(v3(0.0f, 0.0f, 1.0f), roll);
    return quat_mul(quat_mul(qy, qx), qz);
}

static inline quat quat_conjugate(quat q) { quat r = { -q.x, -q.y, -q.z, q.w }; return r; }

static inline vec3 quat_rotate(quat q, vec3 v) {
    vec3 u = v3(q.x, q.y, q.z);
    vec3 t = v3scale(v3cross(u, v), 2.0f);
    return v3add(v3add(v, v3scale(t, q.w)), v3cross(u, t));
}

static inline mat4 mat4_rotate(vec3 axis, float radians) {
    return mat4_from_quat(quat_from_axis_angle(axis, radians));
}

static inline mat4 mat4_transpose(const mat4& m) {
    mat4 r;
    for (int c = 0; c < 4; c++)
        for (int row = 0; row < 4; row++) r.data[row * 4 + c] = m.data[c * 4 + row];
    return r;
}

// a direction, ignoring translation
static inline vec3 mat4_mul_dir(const mat4& m, vec3 d) {
    return v3(m.data[0] * d.x + m.data[4] * d.y + m.data[8]  * d.z,
              m.data[1] * d.x + m.data[5] * d.y + m.data[9]  * d.z,
              m.data[2] * d.x + m.data[6] * d.y + m.data[10] * d.z);
}

static inline vec4 mat4_mul_vec4(const mat4& m, vec4 p) {
    return v4(m.data[0] * p.x + m.data[4] * p.y + m.data[8]  * p.z + m.data[12] * p.w,
              m.data[1] * p.x + m.data[5] * p.y + m.data[9]  * p.z + m.data[13] * p.w,
              m.data[2] * p.x + m.data[6] * p.y + m.data[10] * p.z + m.data[14] * p.w,
              m.data[3] * p.x + m.data[7] * p.y + m.data[11] * p.z + m.data[15] * p.w);
}

static inline vec3 mat4_get_translation(const mat4& m) { return v3(m.data[12], m.data[13], m.data[14]); }

// largest axis scale, for growing a bounding sphere through a transform
static inline float mat4_max_scale(const mat4& m) {
    float sx = m.data[0] * m.data[0] + m.data[1] * m.data[1] + m.data[2]  * m.data[2];
    float sy = m.data[4] * m.data[4] + m.data[5] * m.data[5] + m.data[6]  * m.data[6];
    float sz = m.data[8] * m.data[8] + m.data[9] * m.data[9] + m.data[10] * m.data[10];
    float s = sx > sy ? sx : sy;
    return sqrtf(s > sz ? s : sz);
}

// ---------------- more scalar helpers ----------------

static inline float saturatef(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }
// -1, 0 or +1
static inline float signf(float x) { return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f); }
// where `x` sits between a and b as 0..1 (unclamped): the inverse of lerpf
static inline float inverse_lerpf(float a, float b, float x) { return b != a ? (x - a) / (b - a) : 0.0f; }
// maps x from the range a0..a1 onto b0..b1 (unclamped), e.g. a health value onto a bar width
static inline float remapf(float x, float a0, float a1, float b0, float b1) {
    return lerpf(b0, b1, inverse_lerpf(a0, a1, x));
}
// the part after the decimal point, always 0..1, also for negatives: fractf(-0.25f) == 0.75f
static inline float fractf(float x) { return x - floorf(x); }
// wraps x into lo..hi (hi excluded): looping timers, angles, tiled worlds
static inline float wrapf(float x, float lo, float hi) {
    float w = hi - lo;
    return w > 0.0f ? x - w * floorf((x - lo) / w) : lo;
}
// equal within `eps` near zero, and within a relative `eps` for large values,
// where a fixed tolerance would be too strict
static inline bool approx_equalf(float a, float b, float eps = 1e-5f) {
    float d = fabsf(a - b);
    if (d <= eps) return true;
    float m = fabsf(a) > fabsf(b) ? fabsf(a) : fabsf(b);
    return d <= eps * m;
}

// ---------------- more vector helpers ----------------

// v bounced off a surface with unit normal n (a ball off a wall, a ray off a mirror)
static inline vec3 v3reflect(vec3 v, vec3 n) { return v3sub(v, v3scale(n, 2.0f * v3dot(v, n))); }
static inline vec2 v2reflect(vec2 v, vec2 n) { return v2sub(v, v2scale(n, 2.0f * v2dot(v, n))); }
// the part of v along `onto` (which need not be unit length; zero gives zero)
static inline vec3 v3project(vec3 v, vec3 onto) {
    float d = v3dot(onto, onto);
    return d > 1e-12f ? v3scale(onto, v3dot(v, onto) / d) : v3(0.0f, 0.0f, 0.0f);
}
static inline vec2 v2project(vec2 v, vec2 onto) {
    float d = v2dot(onto, onto);
    return d > 1e-12f ? v2scale(onto, v2dot(v, onto) / d) : v2(0.0f, 0.0f);
}
// the part of v across `onto`: what is left of a velocity after removing the
// part going into a wall
static inline vec3 v3reject(vec3 v, vec3 onto) { return v3sub(v, v3project(v, onto)); }
static inline vec2 v2reject(vec2 v, vec2 onto) { return v2sub(v, v2project(v, onto)); }
static inline bool v3approx_equal(vec3 a, vec3 b, float eps = 1e-5f) {
    return approx_equalf(a.x, b.x, eps) && approx_equalf(a.y, b.y, eps) && approx_equalf(a.z, b.z, eps);
}

// ---------------- more quaternion conversions ----------------

// the rotation in a matrix, with any scale divided out
static inline quat quat_from_mat4(const mat4& m) {
    vec3 t, s;
    quat q;
    mat4_decompose(m, &t, &q, &s);
    return q;
}

// The inverse of quat_from_euler, in the same order (yaw about +Y, then pitch
// about +X, then roll about +Z), radians. Pitch comes back in -pi/2..pi/2.
// Looking straight up or down, yaw and roll turn about the same axis and
// cannot be told apart: roll is reported as 0 and the whole turn as yaw.
static inline void quat_to_euler(quat q, float* pitch, float* yaw, float* roll) {
    mat4 m = mat4_from_quat(quat_norm(q));
    // R = Ry * Rx * Rz; with M[row][col] = m.data[col * 4 + row]:
    //   M12 = -sin(pitch), M02 / M22 give yaw, M10 / M11 give roll
    float sp = clampf(-m.data[9], -1.0f, 1.0f);
    *pitch = asinf(sp);
    if (fabsf(sp) < 0.99999f) {
        *yaw  = atan2f(m.data[8], m.data[10]);
        *roll = atan2f(m.data[1], m.data[5]);
    } else {
        *yaw  = atan2f(-m.data[2], m.data[0]);
        *roll = 0.0f;
    }
}

// the axis (unit) and angle (radians, 0..pi) of a rotation; the identity gives axis +X, angle 0
static inline void quat_to_axis_angle(quat q, vec3* axis, float* radians) {
    q = quat_norm(q);
    if (q.w < 0.0f) { q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w; }   // the short way round
    float s = sqrtf(maxf(1.0f - q.w * q.w, 0.0f));
    *radians = 2.0f * acosf(clampf(q.w, -1.0f, 1.0f));
    *axis = s > 1e-6f ? v3(q.x / s, q.y / s, q.z / s) : v3(1.0f, 0.0f, 0.0f);
}

// ---------------- 3x3 matrices ----------------
//
// Rotation and scale without translation: transforming normals, oriented
// box axes, inertia. Column-major like mat4: data[col * 3 + row].

struct mat3 { float data[9]; };

static inline mat3 mat3_identity() {
    mat3 m = {};
    m.data[0] = m.data[4] = m.data[8] = 1.0f;
    return m;
}
// from three columns, e.g. a basis: right, up, back
static inline mat3 mat3_from_cols(vec3 x, vec3 y, vec3 z) {
    mat3 m = { { x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z } };
    return m;
}
static inline vec3 mat3_col(const mat3& m, int i) { return v3(m.data[i * 3], m.data[i * 3 + 1], m.data[i * 3 + 2]); }
static inline mat3 mat3_mul(const mat3& a, const mat3& b) {
    mat3 r;
    for (int c = 0; c < 3; c++)
        for (int row = 0; row < 3; row++)
            r.data[c * 3 + row] = a.data[row] * b.data[c * 3] + a.data[3 + row] * b.data[c * 3 + 1]
                                + a.data[6 + row] * b.data[c * 3 + 2];
    return r;
}
static inline vec3 mat3_mul_vec3(const mat3& m, vec3 v) {
    return v3(m.data[0] * v.x + m.data[3] * v.y + m.data[6] * v.z,
              m.data[1] * v.x + m.data[4] * v.y + m.data[7] * v.z,
              m.data[2] * v.x + m.data[5] * v.y + m.data[8] * v.z);
}
static inline mat3 mat3_transpose(const mat3& m) {
    mat3 r;
    for (int c = 0; c < 3; c++)
        for (int row = 0; row < 3; row++) r.data[row * 3 + c] = m.data[c * 3 + row];
    return r;
}
static inline float mat3_determinant(const mat3& m) {
    return v3dot(mat3_col(m, 0), v3cross(mat3_col(m, 1), mat3_col(m, 2)));
}
// the identity when the matrix cannot be inverted (a zero scale squashed it flat)
static inline mat3 mat3_inverse(const mat3& m) {
    vec3 a = mat3_col(m, 0), b = mat3_col(m, 1), c = mat3_col(m, 2);
    vec3 r0 = v3cross(b, c), r1 = v3cross(c, a), r2 = v3cross(a, b);
    float det = v3dot(a, r0);
    if (fabsf(det) < 1e-12f) return mat3_identity();
    float k = 1.0f / det;
    // the rows of the inverse are the cross products above
    mat3 r = { { r0.x * k, r1.x * k, r2.x * k, r0.y * k, r1.y * k, r2.y * k, r0.z * k, r1.z * k, r2.z * k } };
    return r;
}
static inline mat3 mat3_scale(float x, float y, float z) {
    mat3 m = {};
    m.data[0] = x; m.data[4] = y; m.data[8] = z;
    return m;
}
// the rotation and scale part of a mat4 (its upper-left 3x3)
static inline mat3 mat3_from_mat4(const mat4& m) {
    mat3 r = { { m.data[0], m.data[1], m.data[2], m.data[4], m.data[5], m.data[6], m.data[8], m.data[9], m.data[10] } };
    return r;
}
static inline mat4 mat4_from_mat3(const mat3& m) {
    mat4 r = mat4_identity();
    for (int c = 0; c < 3; c++)
        for (int row = 0; row < 3; row++) r.data[c * 4 + row] = m.data[c * 3 + row];
    return r;
}
static inline mat3 mat3_from_quat(quat q) { return mat3_from_mat4(mat4_from_quat(q)); }
static inline mat3 mat3_rotate(vec3 axis, float radians) { return mat3_from_quat(quat_from_axis_angle(axis, radians)); }
// What transforms normals correctly through `model` even when it scales
// unevenly (a squashed sphere): the inverse transpose of its 3x3 part.
static inline mat3 mat3_normal_matrix(const mat4& model) { return mat3_transpose(mat3_inverse(mat3_from_mat4(model))); }

// ---------------- colour ----------------
//
// pix_color, and every colour you pick in an image editor, is sRGB: its steps
// are spaced for the eye, not for light. Lighting and blending maths wants
// linear values, where twice the number is twice the light. These convert
// both ways; HSV and HSL are friendlier ways to pick or animate a colour.

static inline float pix_srgb_to_linear(float c) {
    return c <= 0.04045f ? c / 12.92f : powf((c + 0.055f) / 1.055f, 2.4f);
}
static inline float pix_linear_to_srgb(float c) {
    if (c <= 0.0f) return 0.0f;
    return c <= 0.0031308f ? c * 12.92f : 1.055f * powf(c, 1.0f / 2.4f) - 0.055f;
}
// rgb in linear light 0..1, alpha unchanged (alpha is never gamma encoded)
static inline vec4 pix_color_to_linear(pix_color c) {
    return v4(pix_srgb_to_linear(c.r / 255.0f), pix_srgb_to_linear(c.g / 255.0f),
              pix_srgb_to_linear(c.b / 255.0f), c.a / 255.0f);
}
static inline pix_color pix_color_from_linear(vec4 c) {
    return pix_rgba((int)(saturatef(pix_linear_to_srgb(c.x)) * 255.0f + 0.5f),
                    (int)(saturatef(pix_linear_to_srgb(c.y)) * 255.0f + 0.5f),
                    (int)(saturatef(pix_linear_to_srgb(c.z)) * 255.0f + 0.5f),
                    (int)(saturatef(c.w) * 255.0f + 0.5f));
}

// Hue in degrees (0 red, 120 green, 240 blue, wrapping), saturation and value
// 0..1. Sweeping the hue walks the rainbow; value 0 is black.
static inline pix_color pix_hsv(float hue, float saturation, float value, float alpha = 1.0f) {
    float h = wrapf(hue, 0.0f, 360.0f) / 60.0f;
    float s = saturatef(saturation), v = saturatef(value);
    float c = v * s, x = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f)), m = v - c;
    float r = 0, g = 0, b = 0;
    switch ((int)h) {
        case 0:  r = c; g = x; break;
        case 1:  r = x; g = c; break;
        case 2:  g = c; b = x; break;
        case 3:  g = x; b = c; break;
        case 4:  r = x; b = c; break;
        default: r = c; b = x; break;
    }
    return pix_rgba((int)((r + m) * 255.0f + 0.5f), (int)((g + m) * 255.0f + 0.5f),
                    (int)((b + m) * 255.0f + 0.5f), (int)(saturatef(alpha) * 255.0f + 0.5f));
}
// x hue in degrees, y saturation, z value, w alpha
static inline vec4 pix_color_to_hsv(pix_color c) {
    float r = c.r / 255.0f, g = c.g / 255.0f, b = c.b / 255.0f;
    float mx = maxf(r, maxf(g, b)), mn = minf(r, minf(g, b)), d = mx - mn;
    float h = 0.0f;
    if (d > 0.0f) {
        if (mx == r)      h = 60.0f * fmodf((g - b) / d, 6.0f);
        else if (mx == g) h = 60.0f * ((b - r) / d + 2.0f);
        else              h = 60.0f * ((r - g) / d + 4.0f);
        if (h < 0.0f) h += 360.0f;
    }
    return v4(h, mx > 0.0f ? d / mx : 0.0f, mx, c.a / 255.0f);
}
// Hue in degrees, saturation and lightness 0..1. Lightness 0.5 is the pure
// colour, 0 black, 1 white - easier than HSV for "a lighter shade of this".
static inline pix_color pix_hsl(float hue, float saturation, float lightness, float alpha = 1.0f) {
    float l = saturatef(lightness), s = saturatef(saturation);
    float v = l + s * minf(l, 1.0f - l);
    return pix_hsv(hue, v > 0.0f ? 2.0f * (1.0f - l / v) : 0.0f, v, alpha);
}
// x hue in degrees, y saturation, z lightness, w alpha
static inline vec4 pix_color_to_hsl(pix_color c) {
    vec4 hsv = pix_color_to_hsv(c);
    float l = hsv.z * (1.0f - hsv.y * 0.5f);
    float m = minf(l, 1.0f - l);
    return v4(hsv.x, m > 0.0f ? (hsv.z - l) / m : 0.0f, l, hsv.w);
}

// ---------------- curves ----------------
//
// Each works on float, vec2 and vec3, with t running 0..1.

// Cubic Hermite: from p0 to p1, leaving p0 along tangent m0 and arriving at p1
// along m1. The building block of smooth keyframe animation.
static inline float hermite(float p0, float m0, float p1, float m1, float t) {
    float t2 = t * t, t3 = t2 * t;
    return (2.0f * t3 - 3.0f * t2 + 1.0f) * p0 + (t3 - 2.0f * t2 + t) * m0
         + (-2.0f * t3 + 3.0f * t2) * p1 + (t3 - t2) * m1;
}
// Catmull-Rom: a smooth curve through p1 (t = 0) and p2 (t = 1), shaped by the
// points either side. Step along a list of points four at a time and the path
// passes through every one of them - camera rails, patrol routes.
static inline float catmull_rom(float p0, float p1, float p2, float p3, float t) {
    return hermite(p1, 0.5f * (p2 - p0), p2, 0.5f * (p3 - p1), t);
}
// Bezier: starts at the first point, ends at the last, pulled toward the ones
// between without passing through them - the curves of drawing programs.
static inline float bezier_quadratic(float p0, float p1, float p2, float t) {
    float u = 1.0f - t;
    return u * u * p0 + 2.0f * u * t * p1 + t * t * p2;
}
static inline float bezier_cubic(float p0, float p1, float p2, float p3, float t) {
    float u = 1.0f - t;
    return u * u * u * p0 + 3.0f * u * u * t * p1 + 3.0f * u * t * t * p2 + t * t * t * p3;
}
// the direction a cubic Bezier is heading at t (not normalised): to face along the path
static inline float bezier_cubic_tangent(float p0, float p1, float p2, float p3, float t) {
    float u = 1.0f - t;
    return 3.0f * u * u * (p1 - p0) + 6.0f * u * t * (p2 - p1) + 3.0f * t * t * (p3 - p2);
}

// the vec2 / vec3 versions apply the float one to each component
#define PIX__CURVE3(name)                                                                     \
    static inline vec2 name(vec2 a, vec2 b, vec2 c, float t) {                                \
        return v2(name(a.x, b.x, c.x, t), name(a.y, b.y, c.y, t)); }                          \
    static inline vec3 name(vec3 a, vec3 b, vec3 c, float t) {                                \
        return v3(name(a.x, b.x, c.x, t), name(a.y, b.y, c.y, t), name(a.z, b.z, c.z, t)); }
#define PIX__CURVE4(name)                                                                     \
    static inline vec2 name(vec2 a, vec2 b, vec2 c, vec2 d, float t) {                        \
        return v2(name(a.x, b.x, c.x, d.x, t), name(a.y, b.y, c.y, d.y, t)); }                \
    static inline vec3 name(vec3 a, vec3 b, vec3 c, vec3 d, float t) {                        \
        return v3(name(a.x, b.x, c.x, d.x, t), name(a.y, b.y, c.y, d.y, t),                   \
                  name(a.z, b.z, c.z, d.z, t)); }
PIX__CURVE4(hermite)
PIX__CURVE4(catmull_rom)
PIX__CURVE3(bezier_quadratic)
PIX__CURVE4(bezier_cubic)
PIX__CURVE4(bezier_cubic_tangent)
#undef PIX__CURVE3
#undef PIX__CURVE4

// ---------------- easing ----------------
//
// Shapes for animating a value over time: feed in t from 0 to 1, get back a
// reshaped 0 to 1 to lerp with. "in" starts slow, "out" ends slow, "in_out"
// does both. back overshoots a little, elastic springs, bounce bounces.
//   float t = ease_out_back(clampf(timer / duration, 0, 1));
//   button_scale = lerpf(0.5f, 1.0f, t);

static inline float ease_in_quad(float t)      { return t * t; }
static inline float ease_out_quad(float t)     { return t * (2.0f - t); }
static inline float ease_in_out_quad(float t)  { return t < 0.5f ? 2.0f * t * t : 1.0f - 2.0f * (1.0f - t) * (1.0f - t); }
static inline float ease_in_cubic(float t)     { return t * t * t; }
static inline float ease_out_cubic(float t)    { float u = 1.0f - t; return 1.0f - u * u * u; }
static inline float ease_in_out_cubic(float t) {
    float u = 1.0f - t;
    return t < 0.5f ? 4.0f * t * t * t : 1.0f - 4.0f * u * u * u;
}
static inline float ease_in_quart(float t)     { return t * t * t * t; }
static inline float ease_out_quart(float t)    { float u = 1.0f - t; return 1.0f - u * u * u * u; }
static inline float ease_in_out_quart(float t) {
    float u = 1.0f - t;
    return t < 0.5f ? 8.0f * t * t * t * t : 1.0f - 8.0f * u * u * u * u;
}
static inline float ease_in_sine(float t)      { return 1.0f - cosf(t * PIX_PI * 0.5f); }
static inline float ease_out_sine(float t)     { return sinf(t * PIX_PI * 0.5f); }
static inline float ease_in_out_sine(float t)  { return 0.5f - 0.5f * cosf(t * PIX_PI); }
static inline float ease_in_expo(float t)      { return t <= 0.0f ? 0.0f : powf(2.0f, 10.0f * t - 10.0f); }
static inline float ease_out_expo(float t)     { return t >= 1.0f ? 1.0f : 1.0f - powf(2.0f, -10.0f * t); }
static inline float ease_in_out_expo(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t < 0.5f ? 0.5f * powf(2.0f, 20.0f * t - 10.0f) : 1.0f - 0.5f * powf(2.0f, 10.0f - 20.0f * t);
}
static inline float ease_in_circ(float t)      { return 1.0f - sqrtf(maxf(1.0f - t * t, 0.0f)); }
static inline float ease_out_circ(float t)     { float u = t - 1.0f; return sqrtf(maxf(1.0f - u * u, 0.0f)); }
static inline float ease_in_out_circ(float t) {
    return t < 0.5f ? 0.5f * ease_in_circ(2.0f * t) : 0.5f + 0.5f * ease_out_circ(2.0f * t - 1.0f);
}
static inline float ease_in_back(float t)      { const float c = 1.70158f; return t * t * ((c + 1.0f) * t - c); }
static inline float ease_out_back(float t)     { return 1.0f - ease_in_back(1.0f - t); }
static inline float ease_in_out_back(float t) {
    const float c = 1.70158f * 1.525f;
    float u = 2.0f * t;
    if (t < 0.5f) return 0.5f * u * u * ((c + 1.0f) * u - c);
    u -= 2.0f;
    return 0.5f * (u * u * ((c + 1.0f) * u + c) + 2.0f);
}
static inline float ease_in_elastic(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return -powf(2.0f, 10.0f * t - 10.0f) * sinf((10.0f * t - 10.75f) * (PIX_TAU / 3.0f));
}
static inline float ease_out_elastic(float t)  { return 1.0f - ease_in_elastic(1.0f - t); }
static inline float ease_in_out_elastic(float t) {
    return t < 0.5f ? 0.5f * ease_in_elastic(2.0f * t) : 0.5f + 0.5f * ease_out_elastic(2.0f * t - 1.0f);
}
static inline float ease_out_bounce(float t) {
    const float n = 7.5625f, d = 2.75f;
    if (t < 1.0f / d)   return n * t * t;
    if (t < 2.0f / d)   { t -= 1.5f / d;   return n * t * t + 0.75f; }
    if (t < 2.5f / d)   { t -= 2.25f / d;  return n * t * t + 0.9375f; }
    t -= 2.625f / d;
    return n * t * t + 0.984375f;
}
static inline float ease_in_bounce(float t)    { return 1.0f - ease_out_bounce(1.0f - t); }
static inline float ease_in_out_bounce(float t) {
    return t < 0.5f ? 0.5f * ease_in_bounce(2.0f * t) : 0.5f + 0.5f * ease_out_bounce(2.0f * t - 1.0f);
}

// ---------------- trajectories ----------------

// Where something thrown from p0 with velocity v0 is after t seconds, with
// constant acceleration `gravity` (e.g. (0, -9.8, 0)).
static inline vec3 ballistic_position(vec3 p0, vec3 v0, vec3 gravity, float t) {
    return v3add(v3add(p0, v3scale(v0, t)), v3scale(gravity, 0.5f * t * t));
}
// The launch velocity, at `speed`, that lands a projectile thrown from `from`
// on `to`, with gravity of strength `gravity` (> 0) pulling down -Y. Two arcs
// reach any target in range: `high` picks the lob, otherwise the flat shot.
// False when the target is out of reach at that speed.
static inline bool ballistic_launch(vec3 from, vec3 to, float speed, float gravity, bool high, vec3* out_velocity) {
    vec3 d = v3sub(to, from);
    float h = d.y;
    vec2 flat = v2(d.x, d.z);
    float x = v2len(flat), v2s = speed * speed;
    if (gravity <= 0.0f || speed <= 0.0f) return false;
    if (x < 1e-6f) {                                   // straight up or down
        if (h > 0.0f && v2s < 2.0f * gravity * h) return false;
        *out_velocity = v3(0.0f, h >= 0.0f ? speed : -speed, 0.0f);
        return true;
    }
    float disc = v2s * v2s - gravity * (gravity * x * x + 2.0f * h * v2s);
    if (disc < 0.0f) return false;
    float root = sqrtf(disc);
    float tan_a = (v2s + (high ? root : -root)) / (gravity * x);
    float cos_a = 1.0f / sqrtf(1.0f + tan_a * tan_a), sin_a = tan_a * cos_a;
    vec2 dir = v2scale(flat, 1.0f / x);
    *out_velocity = v3(dir.x * cos_a * speed, sin_a * speed, dir.y * cos_a * speed);
    return true;
}

// ---------------- fast approximations ----------------
//
// Cheaper stand-ins for the C library functions, for code that runs millions
// of times a frame (particles, noise, per-vertex work) and can trade a little
// accuracy for speed. The error bound on each is checked by the test suite
// (tests/build_check.cpp) over the range given.
//
// Only the ones that measurably beat the C library are here (MSVC /O2, x64:
// fast_inv_sqrt 2x, fast_rsqrt 1.2x, fast_sin/cos 2x, fast_atan2 2.7x).
// sqrtf, 1/x, exp2f, log2f and powf are already as fast or faster than any
// polynomial stand-in, so use them as they are.

static inline uint32_t pix__float_bits(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }
static inline float pix__bits_float(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }

// The Quake III inverse square root: the 0x5f3759df bit trick for a first
// guess, then one Newton-Raphson step. x > 0. Relative error under 0.18%.
// Kept for what it is; fast_rsqrt below is both faster and closer on any CPU
// with SSE.
static inline float fast_inv_sqrt(float x) {
    float y = pix__bits_float(0x5f3759dfu - (pix__float_bits(x) >> 1));
    return y * (1.5f - 0.5f * x * y * y);
}
// 1 / sqrt(x), x > 0: the CPU's reciprocal square root estimate refined by one
// Newton step. Relative error under 5e-7 with SSE (fast_inv_sqrt without).
static inline float fast_rsqrt(float x) {
#if PIX_MATH_SSE
    float y = _mm_cvtss_f32(_mm_rsqrt_ss(_mm_set_ss(x)));
    return y * (1.5f - 0.5f * x * y * y);
#else
    return fast_inv_sqrt(x);
#endif
}
// normalise through fast_rsqrt; zero stays zero
static inline vec3 v3norm_fast(vec3 a) {
    float d = v3dot(a, a);
    return d > 1e-20f ? v3scale(a, fast_rsqrt(d)) : v3(0.0f, 0.0f, 0.0f);
}
static inline vec2 v2norm_fast(vec2 a) {
    float d = v2dot(a, a);
    return d > 1e-20f ? v2scale(a, fast_rsqrt(d)) : v2(0.0f, 0.0f);
}

// Sine and cosine of any angle (radians): folded into -pi/2..pi/2 and
// evaluated as a 7th-order minimax polynomial. Absolute error under 1e-6 for
// |x| < 2 pi; larger angles lose float precision in the folding itself (under
// 1e-4 at |x| < 1000), so keep angles wrapped if you need the accuracy.
static inline float fast_sin(float x) {
    x -= PIX_TAU * floorf(x * (1.0f / PIX_TAU) + 0.5f);        // -pi..pi
    if (x > PIX_PI * 0.5f)       x = PIX_PI - x;               // sin is symmetric about pi/2
    else if (x < -PIX_PI * 0.5f) x = -PIX_PI - x;
    float x2 = x * x;
    // minimax coefficients (Lawson fit on 0..pi/2), not Taylor: the error is
    // spread evenly instead of piling up at the ends
    return x * (0.9999966159f + x2 * (-0.1666482836f + x2 * (0.008306324991f + x2 * -0.0001836364733f)));
}
static inline float fast_cos(float x) { return fast_sin(x + PIX_PI * 0.5f); }

// atan on 0..1 as an 11th-order odd polynomial; the others fold onto it
static inline float pix__atan01(float z) {
    float z2 = z * z;
    return z * (0.99997726f + z2 * (-0.33262347f + z2 * (0.19354346f + z2 * (-0.11643287f
           + z2 * (0.05265332f + z2 * -0.01172120f)))));
}
// Absolute error under 2.5e-6 radians, any input.
static inline float fast_atan(float x) {
    float a = fabsf(x);
    float r = a > 1.0f ? PIX_PI * 0.5f - pix__atan01(1.0f / a) : pix__atan01(a);
    return x < 0.0f ? -r : r;
}
// Same quadrants and range as atan2f (-pi..pi); 0 for (0, 0). Absolute error under 2.5e-6 radians.
static inline float fast_atan2(float y, float x) {
    float ax = fabsf(x), ay = fabsf(y);
    if (ax == 0.0f && ay == 0.0f) return 0.0f;
    float r = ay > ax ? PIX_PI * 0.5f - pix__atan01(ax / ay) : pix__atan01(ay / ax);
    if (x < 0.0f) r = PIX_PI - r;
    return y < 0.0f ? -r : r;
}

// ---------------- operators ----------------
//
// Plain arithmetic on the plain structs above, so game code can write
// `pos + vel * dt` instead of v3add(pos, v3scale(vel, dt)). Opt out with
// PIX_NO_OPERATORS if they clash with another maths library.
#ifndef PIX_NO_OPERATORS
static inline vec2  operator+(vec2 a, vec2 b)   { return v2add(a, b); }
static inline vec2  operator-(vec2 a, vec2 b)   { return v2sub(a, b); }
static inline vec2  operator-(vec2 a)           { return v2(-a.x, -a.y); }
static inline vec2  operator*(vec2 a, float s)  { return v2scale(a, s); }
static inline vec2  operator*(float s, vec2 a)  { return v2scale(a, s); }
static inline vec2  operator/(vec2 a, float s)  { return v2scale(a, 1.0f / s); }
static inline vec2& operator+=(vec2& a, vec2 b) { a = a + b; return a; }
static inline vec2& operator-=(vec2& a, vec2 b) { a = a - b; return a; }
static inline vec2& operator*=(vec2& a, float s) { a = a * s; return a; }

static inline vec3  operator+(vec3 a, vec3 b)   { return v3add(a, b); }
static inline vec3  operator-(vec3 a, vec3 b)   { return v3sub(a, b); }
static inline vec3  operator-(vec3 a)           { return v3neg(a); }
static inline vec3  operator*(vec3 a, float s)  { return v3scale(a, s); }
static inline vec3  operator*(float s, vec3 a)  { return v3scale(a, s); }
static inline vec3  operator*(vec3 a, vec3 b)   { return v3mul(a, b); }
static inline vec3  operator/(vec3 a, float s)  { return v3scale(a, 1.0f / s); }
static inline vec3& operator+=(vec3& a, vec3 b) { a = a + b; return a; }
static inline vec3& operator-=(vec3& a, vec3 b) { a = a - b; return a; }
static inline vec3& operator*=(vec3& a, float s) { a = a * s; return a; }

static inline mat4  operator*(const mat4& a, const mat4& b) { return mat4_mul(a, b); }
static inline mat3  operator*(const mat3& a, const mat3& b) { return mat3_mul(a, b); }
static inline vec3  operator*(const mat3& m, vec3 v)        { return mat3_mul_vec3(m, v); }
static inline quat  operator*(quat a, quat b)   { return quat_mul(a, b); }
#endif

#endif // PIX_MATH_HPP
