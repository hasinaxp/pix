#pragma once
#include <stdint.h>
#include "math.hpp"

// Deterministic, seekable noise. Procedural generation needs the same world
// every run from the same seed, and it needs to ask "what belongs at cell
// (x, z)?" out of order - so alongside the streaming xorshift generator there
// is a stateless hash that turns coordinates straight into values.

struct rng {
    uint32_t state;
};

static rng rng_seed(uint32_t seed) {
    rng r;
    r.state = seed ? seed : 0x9E3779B9u;   // 0 is a fixed point of xorshift
    return r;
}

static uint32_t rng_u32(rng& r) {
    uint32_t x = r.state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    r.state = x;
    return x;
}

// [0, 1)
static float rng_float(rng& r) { return (float)(rng_u32(r) >> 8) * (1.0f / 16777216.0f); }
static float rng_range(rng& r, float lo, float hi) { return lo + (hi - lo) * rng_float(r); }
// [lo, hi] inclusive
static int rng_int(rng& r, int lo, int hi) {
    if (hi <= lo) return lo;
    return lo + (int)(rng_u32(r) % (uint32_t)(hi - lo + 1));
}
static bool rng_chance(rng& r, float probability) { return rng_float(r) < probability; }

// ---------------- stateless hashes ----------------

static uint32_t hash_u32(uint32_t x) {
    x ^= x >> 16; x *= 0x7FEB352Du;
    x ^= x >> 15; x *= 0x846CA68Bu;
    x ^= x >> 16;
    return x;
}

static uint32_t hash2(int x, int y, uint32_t seed) {
    return hash_u32((uint32_t)x * 0x8DA6B343u ^ (uint32_t)y * 0xD8163841u ^ seed);
}

// [0, 1) from a coordinate pair - the workhorse for per-cell decisions
static float hash2_float(int x, int y, uint32_t seed) {
    return (float)(hash2(x, y, seed) >> 8) * (1.0f / 16777216.0f);
}

// an rng deterministically seeded from a coordinate, for when one cell needs
// a whole sequence of decisions rather than a single value
static rng rng_at(int x, int y, uint32_t seed) { return rng_seed(hash2(x, y, seed)); }
