#ifndef PIX_PHYSICS_HPP
#define PIX_PHYSICS_HPP

// pix physics: include this one header for all of it.
//
// Pick the world that fits the game; they share layers, units and naming, so
// moving between them is a change of prefix, not of habits:
//
//   physics2d.hpp       phys2d_*       2D games: rigid bodies that rotate, stack,
//                                      bounce, hang from joints. x right, y down,
//                                      units are your drawing units (pixels)
//   physics3d.hpp       phys3d_*       3D rigid bodies: boxes, balls, capsules on
//                                      planes and triangle meshes, joints, ragdolls.
//                                      Metres, y up
//   physics3d_flat.hpp  phys3d_flat_*  3D games on the ground: upright characters
//                                      and vehicles pushing each other and walls,
//                                      no tipping or stacking - the cheapest world
//                                      for crowds. Metres, y up
//   colliders.hpp       collide_*,     shape tests with no world at all: does this
//                       ray_vs_*       overlap that, where does this ray hit
//   physics_common.hpp  PHYS_LAYER_*   what they share: layer bits and the one
//                                      layer rule, phys_layers_collide
//
// Every world has the same shape of API: <prefix>_create_world /
// _destroy_world, _update(w, pix_dt()) for fixed steps or _step(w, dt) for
// one, _add_body / _get_body / _remove_body with `idx` handles, and
// _raycast / _overlap queries that take an optional layer mask. Bodies carry
// layer, mask, friction, restitution, linear_damping, gravity_scale and a
// `user` pointer under the same names everywhere.
//
// No graphics in here: debug drawing lives in physics_debug.hpp (the app
// layer's pix_draw_physics2d / 3d / 3d_flat), so a server or tool can
// simulate without linking OpenGL.

// dtype.hpp first: it sets up the CRT before any C header
#include "dtype.hpp"
#include "colliders.hpp"
#include "physics_common.hpp"
#include "physics2d.hpp"
#include "physics3d.hpp"
#include "physics3d_flat.hpp"

#endif // PIX_PHYSICS_HPP
