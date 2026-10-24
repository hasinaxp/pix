# pix

A header-only C++ game library for Windows and OpenGL 4.4, for making any kind
of game, from a 2D arcade game to a shadowed, reflective 3D world. It has no
dependencies beyond the Windows SDK and the C/C++ standard library.
Everything is hand-written: window, GL loader, PNG/OBJ/glTF/WAV/TTF decoding,
the renderer and the audio mixer.

```cpp
#define PIX_IMPLEMENTATION
#include "pix.h"

int main() {
    pix_init("my game", 1280, 720);
    while (pix_frame()) {
        pix_clear(PIX_SKYBLUE);
        pix_draw_rect(100, 100, 200, 80, PIX_RED);
        pix_draw_text("hello pix", 110, 120, 32, PIX_WHITE);
    }
    pix_shutdown();
}
```

```
cl /std:c++14 /O2 /I dist game.cpp        # single header
cl /std:c++14 /O2 /I src  game.cpp        # or the modular sources
```

pix links its own system libraries (`opengl32`, `gdi32`, `user32`, `winmm`)
through `#pragma comment`, so the command line needs nothing else.

## Documentation

New to pix? Start with **[Getting started](docs/getting-started.md)**. The
full set is in [`docs/`](docs/index.md):

| page | covers |
|---|---|
| [Getting started](docs/getting-started.md) | setup, compiling, the frame loop, coordinates, types, errors |
| [Input](docs/inputs.md) | keyboard, mouse, gamepads, text entry |
| [2D rendering](docs/2d-rendering.md) | shapes, images, sprites, text, 2D cameras |
| [Tilemaps](docs/tilemaps.md) | Tiled maps, autotiling, animated tiles, objects, parallax |
| [3D rendering](docs/3d-rendering.md) | cameras, meshes, materials, models, lights, shadows, post-processing |
| [3D physics](docs/physics-3d.md) | 3D rigid bodies, meshes, joints, ragdolls, sleeping, queries |
| [2D physics](docs/physics-2d.md) | 2D rigid bodies, joints, sleeping, triggers, events, queries |
| [Physics](docs/physics.md) | which world to use, what they share, shape tests |
| [Flat 3D physics](docs/physics-3d-flat.md) | characters and crowds on the ground |
| [Math](docs/math.md) | conventions, helpers, rotations, colour, easing and curves, fast approximations |
| [Memory](docs/memory.md) | optional: scoped memory, stack buffers, arenas, limits, a custom heap |
| [Cheatsheet](docs/cheatsheet.md) | the common calls on one page |

## Getting it

| | |
|---|---|
| `dist/pix.h` | the whole library in one file; generate it with `python compile.py` |
| `src/pix.h` | the same library as ~30 modules, for reading and working on it |

Define `PIX_IMPLEMENTATION` in **exactly one** `.cpp` before including pix. Every
other file just includes it. That's the stb convention, so a game can have as many
source files as it likes.

## Building the examples

```
build                 every example into build\
build hello_3d run    one example, then run it
build test            the CPU-side test suite (tests\build_check.cpp)
build single          regenerate dist\pix.h and compile every example against it
build debug city      unoptimised, with debug info
```

`build.bat` locates Visual Studio through `vswhere`, so a normal command prompt
works. Run examples from the repository root so `assets/` resolves.

| example | shows |
|---|---|
| `hello_2d` | shapes, SDF text, a 2D camera, input |
| `hello_3d` | primitives, an OBJ and a rigged glTF, shadows, lights, particles, post effects, quality presets |
| `reflections` | reflection probe, planar water, glass refraction |
| `pipeline` | composing a frame by hand: minimap view, layers, a custom shader with named uniforms, a PBR → toon blend, a custom fullscreen pass |
| `animation` | two rigs, clip cross-fades, footstep audio, gamepad rumble |
| `crowd` | 1,000 characters on `pix_anim_system`: gait cross-fades, masked overlay, events, LOD, instanced skinning |
| `grassland` | 500,000 grass blades simulated and culled with compute shaders, drawn with one indirect call via a render hook |
| `city` | the lower-level loader/renderer API: thousands of instances in a few draws |

## The layers

pix has two levels, and you can mix them freely:

* **The app layer** (`core/app.hpp`) is one global context with short raylib-style names:
  `pix_init`, `pix_frame`, `pix_draw_*`, `pix_load_model`, `pix_play_sound`.
* **The modules** under it are plain structs and functions: `pix_window`,
  `pix_renderer`, `pix_batch2d`, `pix_audio`, the loaders. `pix_get_window()`,
  `pix_get_renderer()` and friends hand out the app's instances.

The code is C-style: structs with free functions, no inheritance, no hidden
allocation, and no exceptions.

## A frame

```cpp
while (pix_frame()) {                      // present last frame, pump input, tick the clock
    float dt = pix_dt();

    camera cam = {};
    cam.position = v3(0, 5, 10);
    cam.direction = v3(0, -0.4f, -1);

    pix_begin_3d(cam);                     // 3D first...
    pix_draw_model(hero, mat4_translate(x, 0, z));
    pix_draw_mesh(cube_mesh, crate_material, transform);
    pix_add_light(pix_point_light(torch_pos, 8.0f, v3(4, 2.5f, 1)));
    pix_end_3d();                          // ...renders the whole screen

    pix_draw_textf(16, 16, 24, PIX_WHITE, "score %d", score);   // ...then 2D on top
}
```

## 3D

### Meshes, materials, models

```cpp
idx mesh = pix_gen_cube(v3(1, 1, 1));                     // also sphere, plane, cylinder
idx mat  = pix_make_material(pix_rgba(200, 60, 50), /*metallic*/ 0.0f, /*roughness*/ 0.4f);
pix_model knight = pix_load_model("knight.glb");          // .obj (+.mtl) or .gltf/.glb, static or rigged
animator  anim   = pix_model_animator(knight);            // every clip it shipped with
```

`material` is a plain struct (`pix_material_default()` gives the defaults). Its fields:

| field | meaning |
|---|---|
| `texture`, `color`, `metallic`, `roughness`, `emissive` | the usual metal/rough PBR inputs; emissive over 1 blooms |
| `normal_map`, `orm_map`, `emissive_map` | glTF's maps: bumps, occlusion/roughness/metallic, glow (`pix_load_model` fills them) |
| `alpha`, `blend` | `PIX_BLEND_ALPHA` / `PIX_BLEND_ADDITIVE`; blended materials are sorted back to front |
| `alpha_cutoff`, `double_sided` | cut-out foliage, fences, cloth |
| `unlit`, `cast_shadows` | |
| `toon`, `toon_bands`, `rim`, `toon_specular` | cel shading, **blended** with PBR by `toon` (0 = PBR, 1 = toon) |
| `planar_reflection`, `refraction` | see [Reflections](#reflections-and-refraction) |
| `shader`, `textures[4]`, `params0/1`, named uniforms | see [Your own shaders](#your-own-shaders) |

### How it stays fast

* Every static mesh lives in **one shared vertex buffer and one index buffer**.
  Both grow on the GPU as meshes load.
* Instances are submitted per frame. Each view frustum-culls them, radix-sorts
  them by (material, mesh), streams their transforms in one upload and draws
  **one instanced call per run**. The city example draws 2,600 instances in about 40 draws.
* The instance stream is a per-frame bump allocator over an orphaned buffer, so no
  upload ever waits on the GPU.
* Uniform locations are cached per program. Material state is sent only when the
  material changes, and the frame's shared data is in two uniform buffers.
* Everything big is heap-allocated or in virtual-memory arenas. No structure
  needs a large stack, and memory grows with what the game actually uses.

### Look settings: shadows, post-processing

These are plain data you edit in place. A change applies at the next frame and
reallocates only what it has to.

```cpp
pix_set_quality(PIX_QUALITY_HIGH);        // LOW..ULTRA: shadow res + softness, bloom, temporal AA, reflection probe

pix_shadows().distance   = 80.0f;         // world units the cascades cover
pix_shadows().resolution = 4096;          // per cascade
pix_shadows().softness   = 2;             // PCF: 0 hard .. 2 soft
pix_shadows().enabled    = false;         // frees the shadow atlas

pix_post().bloom_strength = 0.5f;
pix_post().tonemap        = PIX_TONEMAP_ACES;   // or REINHARD, NONE
pix_post().exposure       = 1.2f;
pix_post().saturation     = 1.1f;
pix_post().vignette       = 0.3f;
pix_post().color_filter   = v3(1.0f, 0.95f, 0.85f);
pix_post().fxaa           = true;

int fx = pix_add_effect(PIX_EFFECT_OUTLINE, v4(1, 0.04f, 0.9f, 0));   // or CRT, PIXELATE, CHROMATIC,
pix_effect_enable(fx, false);                                          //    GRAYSCALE, SEPIA, DEPTH_BLUR
```

### Lighting

```cpp
pix_time_of_day(17.5f);                              // sun, sky, fog, exposure from keyframes
pix_set_sun(r, dir_to_sun, v3(3, 2.8f, 2.5f));       // or set everything yourself
pix_set_sky_colors(r, zenith, horizon, ground);
pix_set_ambient(r, sky_irradiance, ground_irradiance);
pix_set_fog(r, v3(0.7f, 0.75f, 0.8f), 0.004f);
r.sky_enabled = false;                               // flat r.clear_color instead of the sky
pix_add_light(pix_spot_light(pos, dir, 20.0f, 0.3f, 0.5f, v3(10, 9, 8)));   // per frame, nearest 32 shaded
```

### Reflections and refraction

```cpp
pix_reflections().probe = true;                      // the scene as a cubemap: every glossy surface
                                                     // reflects the real world (amortised, 1 face/frame)
pix_set_reflection_plane(r, v3(0, 1, 0), v3(0, water_level, 0));   // exact planar mirror

idx water = pix_add_material(r, pix_material_water(r));        // waves, depth colour, foam, refraction
idx toony = pix_add_material(r, pix_material_water_toon(r));   // banded colours, crisp shore foam
idx glass = pix_add_material(r, pix_material_glass(v3(0.9f, 1, 1), 0.1f, 0.05f));
idx mirror = pix_add_material(r, pix_material_mirror());
```

Water reads the scene depth behind every pixel, so it's clear in the shallows,
deep-coloured in the middle, and foamy where it meets things. Tune it through
`params0` / `params1` (documented at `GLSL_WATER_COMMON` in `shader_sources.hpp`).
The refraction copy is made only when a refractive surface is on screen.

### Your own shaders

**A custom surface.** Write a fragment shader. The whole lighting library is
spliced in: `shade_surface`, `shade_toon`, `sky_env`, `apply_fog`,
`planar_reflection`, `refraction_color`, `scene_depth_at`, `value_noise`/`fbm`,
and scene values such as `uTime`, `uCameraPos` and `uSunDir`.

```cpp
material m = pix_material_default();
m.shader = load_surface_shader(r, MY_FRAGMENT_SHADER /*, optional custom vertex shader */);
idx mat = pix_add_material(r, m);
pix_material_set_vec3 (r, mat, "uGlow", v3(0.2f, 0.9f, 1.6f));   // named uniforms, sent only on material change
pix_material_set_float(r, mat, "uSpeed", 2.0f);
pix_material_set_texture(r, mat, 1, noise_texture.id);            // uTex1..uTex4
```

**Your own uniform buffers**, shared by any shaders that declare them:

```cpp
pix_uniform_buffer ub = pix_create_uniform_buffer(sizeof(my_block), /*binding*/ 2);
pix_update_uniform_buffer(ub, &my_block);            // GLSL: layout(std140, binding = 2) uniform ...
```

**Post effects and fullscreen passes** take just a `main()`:

```cpp
int fx = pix_add_effect(R"( void main() { frag = vec4(1.0 - texture(uScene, vUV).rgb, 1.0); } )");
idx pass = pix_load_fullscreen_shader(r, body);      // uTex0..3, uTexel, uParams
```

### Composing your own frame

The renderer is a set of stages, and `pix_end_3d` just runs the default composition:

```cpp
pix_render_shadows(r);                               // sun cascades around the frame camera
pix_render_reflections(r);                           // probe faces + mirror
pix_render_view(r, pix_view_default(r, cam));        // into r.scene_target (HDR)
pix_post_process(r, r.scene_target, nullptr);        // -> the window
pix_finish_3d();                                     // instead of pix_end_3d
```

Call them yourself to build anything else. A `pix_view` takes a camera
(perspective or `ortho_height`), a target and viewport, a **layer mask**, and
switches for sky, opaque, skinned, transparent, particles and lines, shadows and
reflections. It also takes a clip plane, a distance cap and a shader override.
Split-screen, minimaps, portals, security cameras, ID/outline buffers and
render-to-texture are all a few views. See `examples/pipeline.cpp`.

```cpp
push_instance(r, mesh, mat, transform, tint, /*layers*/ 0x2);   // e.g. minimap-only
pix_draw_target(minimap_ldr, { x, y, 256, 256 });               // show a render target in 2D
```

## Compute shaders and GPU-driven drawing

```cpp
pix_gpu_buffer particles = pix_create_gpu_buffer(sizeof(p) * N, initial);
idx sim = pix_create_compute_shader(R"(
    layout(local_size_x = 256) in;
    layout(std430, binding = 0) buffer P { vec4 pos[]; };
    void main() { pos[gl_GlobalInvocationID.x].y += sin(uTime) * 0.01; }   // scene block + noise available
)");
pix_bind_storage(particles, 0);
pix_dispatch_for(sim, N, 256);
pix_compute_barrier(PIX_BARRIER_VERTEX | PIX_BARRIER_STORAGE);
```

`pix_dispatch_indirect`, storage images (`pix_create_storage_texture`,
`pix_bind_image`), read-back and the indirect-argument structs are all there.

To draw your own GPU work *inside* the pipeline, lit and shadowed like
everything else, register a **render hook**. It runs in every view (and
optionally each shadow cascade), with that view's framebuffer, matrices, scene
block and shadow map already bound:

```cpp
static void my_hook(pix_renderer& r, const pix_hook_context& ctx, void* user) {
    if (ctx.pass != PIX_HOOK_OPAQUE) return;
    // cull with a compute shader against ctx.frustum, then glDrawArraysIndirect(...)
}
pix_add_render_hook(r, my_hook, &my_data);
idx prog = pix_create_scene_shader(r, vs, fs);   // gets shade_surface, shadows, fog...
```

`examples/grassland.cpp` is the full pattern.

## Animation at scale

`animator` handles a single character. `pix_anim_system` handles crowds:

```cpp
pix_anim_system& anims = pix_get_animations();                 // on the app's worker threads
idx a = pix_anim_add(anims, model.skeleton, model.clips, model.clip_count);
pix_anim_play(anims, a, run_clip, /*fade*/ 0.3f);               // cross-fade
pix_anim_overlay(anims, a, wave_clip, 1.0f, pix_anim_bone_mask(*model.skeleton, "Spine2"));
pix_anim_add_event(anims, &model.clips[run_clip], 0.2f, FOOTSTEP);
pix_anim_update(anims, dt, camera_pos);                         // parallel, distance LOD
int n; const mat4* bones = pix_anim_bones(anims, a, &n);
pix_draw_model_bones(model, bones, n, transform);
```

All actors share one packed bone palette. The renderer uploads it once per
frame and skins with instanced draws that read it from storage buffers, so
**a crowd of one model is one draw call** (plus one per shadow cascade).
There's no per-draw bone limit.

## Mesh processing

```cpp
pix_mesh_data m = pix_mesh_copy(loader.mesh_files[id]);
pix_mesh_weld(m);                        // share identical vertices
pix_mesh_recompute_normals(m, 45.0f);    // smooth under 45 degrees, crisp edges above
pix_mesh_data lod = pix_mesh_copy_data(m.vertices, m.vertex_count, m.indices, m.index_count);
pix_mesh_simplify(lod, 0.25f);           // quadric error: keeps UV seams, borders, no flips
pix_mesh_subdivide(m, 1);  pix_mesh_transform(m, t);  pix_mesh_merge(parts, transforms, n);
idx mesh = load_mesh_data(r, m.vertices, m.vertex_count, m.indices, m.index_count);
```
## 2D

```cpp
pix_draw_rect_rounded({ x, y, w, h }, 8, color);
pix_draw_circle(center, r, color);   pix_draw_line(a, b, 3, color);   pix_draw_triangle(a, b, c, color);
pix_texture hero = pix_load_texture("hero.png", TEXTURE_PIXELATED);
pix_draw_texture_pro(hero, src_rect, dst_rect, origin, rotation_deg, tint);
pix_draw_text("any size, crisp", x, y, 48, PIX_WHITE);        // SDF text
pix_begin_2d(camera2d); ... pix_end_2d();                     // world space: offset, target, rotation, zoom
pix_begin_clip(rect); ... pix_end_clip();
```

Everything goes into one streamed vertex batch and is flushed only when the
texture changes, so a sprite atlas draws in one call.

## Input, audio, animation

```cpp
pix_key_pressed('E'); pix_key_down(KEY_SHIFT); pix_mouse_pressed(PIX_MOUSE_LEFT);
vec2 move = pix_input_axis();                         // WASD / arrows / left stick
const pix_gamepad& pad = pix_get_gamepad(0);          // XInput, radial deadzones, rumble
pix_text_input(&count);                               // typed characters, for text fields

idx boom = pix_load_sound("boom.wav");
pix_play_sound(boom, /*volume*/ 1, /*pitch*/ 1.2f, /*loop*/ false, /*pan*/ -0.5f);

animator_update(anim, dt);
animator_sample_blend(anim, from, t0, to, t1, weight, &pose);   // cross-fades on bone TRS, not matrices
pix_draw_model_animated(knight, anim.pose, transform);
```

## Modules

| module | |
|---|---|
| `dtype` | types, colours, logging, file reading |
| `memory` | global heap, per-thread scoped memory, stack buffers, arenas, statistics |
| `math` | vectors, matrices, quaternions, frustums, operators |
| `random` | seeded RNG and position hashes |
| `platform` | window (resizable, fullscreen, DPI aware), GL 4.4 context, keyboard/mouse/gamepad, time |
| `opengl_api`, `opengl_utils` | GL loader, shaders, textures, uniform buffers |
| `texture`, `font`, `draw2d`, `sprite`, `text` | 2D |
| `renderer`, `shader_sources`, `lighting`, `framebuffer`, `animation` | 3D: PBR + toon, shadows, reflections, HDR environments, LODs, billboards, TAA / MSAA |
| `platform_audio`, `sound` | waveOut device, mixer, voices with pitch and pan |
| `loader/*` | PNG, Radiance HDR, OBJ/MTL, glTF 2.0 (skins, animation, PBR maps), WAV |
| `physics` | all physics in one include: `colliders` (shape tests), `physics_common` (layers), `physics2d` (polygons, joints), `physics3d` (convex hulls, compounds, joints, vehicles, a character controller, continuous collision), `physics3d_flat` (characters and crowds on the ground); `physics_debug` draws them |
| `jobs`, `profile` | thread pool / parallel-for, section profiler |
| `compute` | compute shaders, GPU buffers, indirect draws, storage images |
| `meshops` | weld, normals, simplify, subdivide, merge, transform |
| `app` | the easy layer |

## Current limits

* **Physics** runs on one thread. 2D has no compound bodies, character
  controller or wheel/gear/pulley joints yet (3D has them).
* Images are PNG (and HDR environments); JPEG is not supported, so glTF files
  with JPEG textures load untextured. Audio is WAV only.
* Skinned meshes have no automatic LODs, and under temporal AA they follow
  only the camera's motion.
* Windows + OpenGL 4.4 only, by design.
