#ifndef PIX_RANDOM_HPP
#define PIX_RANDOM_HPP
#include <stdint.h>
#include "math.hpp"

// Deterministic, seekable noise. Procedural generation needs the same world
// every run from the same seed, and it needs to ask "what belongs at cell
// (x, z)?" out of order - so alongside the streaming xorshift generator there
// is a stateless hash that turns coordinates straight into values.

struct rng {
    uint32_t state;
};

static inline rng rng_seed(uint32_t seed) {
    rng r;
    r.state = seed ? seed : 0x9E3779B9u;   // 0 is a fixed point of xorshift
    return r;
}

static inline uint32_t rng_u32(rng& r) {
    uint32_t x = r.state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    r.state = x;
    return x;
}

// [0, 1)
static inline float rng_float(rng& r) { return (float)(rng_u32(r) >> 8) * (1.0f / 16777216.0f); }
static inline float rng_range(rng& r, float lo, float hi) { return lo + (hi - lo) * rng_float(r); }
// [lo, hi] inclusive
static inline int rng_int(rng& r, int lo, int hi) {
    if (hi <= lo) return lo;
    return lo + (int)(rng_u32(r) % (uint32_t)(hi - lo + 1));
}
static inline bool rng_chance(rng& r, float probability) { return rng_float(r) < probability; }

// ---------------- stateless hashes ----------------

static inline uint32_t hash_u32(uint32_t x) {
    x ^= x >> 16; x *= 0x7FEB352Du;
    x ^= x >> 15; x *= 0x846CA68Bu;
    x ^= x >> 16;
    return x;
}

static inline uint32_t hash2(int x, int y, uint32_t seed) {
    return hash_u32((uint32_t)x * 0x8DA6B343u ^ (uint32_t)y * 0xD8163841u ^ seed);
}

// [0, 1) from a coordinate pair - the workhorse for per-cell decisions
static inline float hash2_float(int x, int y, uint32_t seed) {
    return (float)(hash2(x, y, seed) >> 8) * (1.0f / 16777216.0f);
}

// an rng deterministically seeded from a coordinate, for when one cell needs
// a whole sequence of decisions rather than a single value
static inline rng rng_at(int x, int y, uint32_t seed) { return rng_seed(hash2(x, y, seed)); }

// ---- value noise ----
// Smooth pseudo-random fields for terrain, placement density, variation.
// Stateless: the same (x, y, seed) always gives the same value in 0..1.
static inline float value_noise2(float x, float y, uint32_t seed) {
    float fx = floorf(x), fy = floorf(y);
    int ix = (int)fx, iy = (int)fy;
    float tx = x - fx, ty = y - fy;
    tx = tx * tx * (3.0f - 2.0f * tx);
    ty = ty * ty * (3.0f - 2.0f * ty);
    float a = hash2_float(ix, iy, seed),     b = hash2_float(ix + 1, iy, seed);
    float c = hash2_float(ix, iy + 1, seed), d = hash2_float(ix + 1, iy + 1, seed);
    return lerpf(lerpf(a, b, tx), lerpf(c, d, tx), ty);
}

// fractal sum of `octaves` layers, each twice the frequency and half the amplitude; 0..1
static inline float fbm2(float x, float y, int octaves, uint32_t seed) {
    float sum = 0.0f, amp = 0.5f, norm = 0.0f;
    for (int i = 0; i < octaves; i++) {
        sum += value_noise2(x, y, seed + (uint32_t)i * 101u) * amp;
        norm += amp;
        x *= 2.03f; y *= 2.03f;
        amp *= 0.5f;
    }
    return norm > 0.0f ? sum / norm : 0.0f;
}
#endif // PIX_RANDOM_HPP
