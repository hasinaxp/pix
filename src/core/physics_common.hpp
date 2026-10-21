#ifndef PIX_PHYSICS_COMMON_HPP
#define PIX_PHYSICS_COMMON_HPP
#include "dtype.hpp"

// What every physics world shares, so the three never disagree.
//
// Collision layers. Each body is on some layers (`layer`) and collides with
// some layers (`mask`); a pair collides only when each accepts the other.
// "Both" rather than "either" is what makes a one-sided filter work: a
// trigger whose mask is the player's layer reports the player and nothing
// else, however the other bodies' masks are set.
//
// The engine gives the bits no meaning; a game names the ones it uses
// (#define LAYER_ENEMY PHYS_LAYER_4). The first four carry names nearly every
// game ends up wanting, so the common case needs no boilerplate.
#define PHYS_LAYER_0         0x0001u
#define PHYS_LAYER_1         0x0002u
#define PHYS_LAYER_2         0x0004u
#define PHYS_LAYER_3         0x0008u
#define PHYS_LAYER_4         0x0010u
#define PHYS_LAYER_5         0x0020u
#define PHYS_LAYER_6         0x0040u
#define PHYS_LAYER_7         0x0080u
#define PHYS_LAYER_8         0x0100u
#define PHYS_LAYER_9         0x0200u
#define PHYS_LAYER_10        0x0400u
#define PHYS_LAYER_11        0x0800u
#define PHYS_LAYER_12        0x1000u
#define PHYS_LAYER_13        0x2000u
#define PHYS_LAYER_14        0x4000u
#define PHYS_LAYER_15        0x8000u
#define PHYS_LAYER_PLAYER    PHYS_LAYER_0
#define PHYS_LAYER_CHARACTER PHYS_LAYER_1
#define PHYS_LAYER_VEHICLE   PHYS_LAYER_2
#define PHYS_LAYER_PROP      PHYS_LAYER_3
#define PHYS_LAYER_ALL       0xFFFFu

// the one rule all three worlds use
static inline bool phys_layers_collide(uint16_t layer_a, uint16_t mask_a, uint16_t layer_b, uint16_t mask_b) {
    return (mask_a & layer_b) != 0 && (mask_b & layer_a) != 0;
}

#endif // PIX_PHYSICS_COMMON_HPP
