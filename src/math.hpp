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
