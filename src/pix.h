// pix - a header-only game library for Windows / OpenGL 4.4.
//
//   #define PIX_IMPLEMENTATION      // in exactly ONE .cpp
//   #include "pix.h"
//
// Build: cl /std:c++14 /O2 game.cpp        (links itself: opengl32, gdi32, user32, winmm)
//
// No dependencies beyond the Windows SDK and the C/C++ standard library.
// compile.py flattens this file and every module below into dist/pix.h.
//
// Modules, low level to high level:
//   core/dtype.hpp          types, colours, logging
//   core/memory.hpp         global heap, scoped stacks, stack buffers, arenas
//   core/math.hpp           vectors, matrices, quaternions, frustum
//   core/random.hpp         rng and hashes
//   core/platform.hpp       window, GL context, keyboard/mouse/gamepad, time
//   core/opengl_api.hpp     GL 4.4 function loader
//   core/opengl_utils.hpp   shaders, textures
//   core/jobs.hpp           thread pool: parallel_for and a side task
//   core/profile.hpp        CPU section profiler
//   core/colliders.hpp      2D/3D shape tests and contacts
//   core/physics.hpp        character-scale physics world with broadphase
//   loader/*                PNG, OBJ, glTF, WAV decoders, JSON reader, asset tables
//   core/texture.hpp        textures from files
//   core/font.hpp           TrueType -> SDF atlas
//   core/framebuffer.hpp    render targets
//   core/lighting.hpp       sun, sky, time of day, point/spot lights
//   core/shader_sources.hpp built-in GLSL
//   core/compute.hpp        compute shaders, GPU buffers, indirect draws
//   core/animation.hpp      skeletal animation: animators and the crowd system
//   core/meshops.hpp        weld, normals, simplify/LODs, subdivide, merge
//   core/renderer.hpp       instanced PBR 3D renderer: shadows, bloom, FXAA
//   core/sprite.hpp         retained sprite batches
//   core/text.hpp           retained text
//   core/draw2d.hpp         immediate 2D: shapes, textures, SDF text, cameras, shake
//   core/atlas.hpp          sprite atlases, sprite animation, atlas packing
//   core/tilemap.hpp        tilemaps, Tiled loading, autotiling, parallax layers
//   core/platform_audio.hpp + core/sound.hpp   mixer and playback
//   core/app.hpp            pix_init / pix_frame / pix_draw_* - start here
// No guard of its own on purpose: every module carries an API guard and a
// separate implementation guard, so including pix.h again after defining
// PIX_IMPLEMENTATION in the same file still emits the implementation.

#include "core/dtype.hpp"
#include "core/math.hpp"
#include "core/random.hpp"
#include "core/platform.hpp"
#include "core/opengl_api.hpp"
#include "core/opengl_utils.hpp"
#include "core/jobs.hpp"
#include "core/profile.hpp"
#include "core/colliders.hpp"
#include "core/physics.hpp"
#include "loader/asset_types.hpp"
#include "loader/png_loader.hpp"
#include "loader/png_writer.hpp"
#include "loader/obj_loader.hpp"
#include "loader/json.hpp"
#include "loader/gltf_loader.hpp"
#include "loader/wav_loader.hpp"
#include "loader/data_loader.hpp"
#include "core/texture.hpp"
#include "core/font.hpp"
#include "core/framebuffer.hpp"
#include "core/lighting.hpp"
#include "core/shader_sources.hpp"
#include "core/compute.hpp"
#include "core/animation.hpp"
#include "core/meshops.hpp"
#include "core/renderer.hpp"
#include "core/sprite.hpp"
#include "core/text.hpp"
#include "core/draw2d.hpp"
#include "core/atlas.hpp"
#include "core/tilemap.hpp"
#include "core/platform_audio.hpp"
#include "core/sound.hpp"
#include "core/app.hpp"

