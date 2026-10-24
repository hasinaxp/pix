# 3D rendering

This page explains how to put a 3D scene on screen: a camera, objects, lights
and the look of the final image. You don't need to know OpenGL. pix handles
the GPU work, including shadows, lighting and post-processing.

## The ideas

Six things make up every 3D scene in pix:

| thing | what it is | how you get one |
|---|---|---|
| **mesh** | a shape: the triangles of a cube, a tree, a character | `pix_gen_cube(...)`, or from a model file |
| **material** | how a surface looks: colour, texture, shiny or rough, glowing | `pix_make_material(...)` |
| **transform** | where an object is, how it is rotated, how big it is (a `mat4`) | `mat4_translate(...)`, `mat4_rotate_y(...)`, ... |
| **instance** | one mesh drawn with one material at one transform, this frame | `pix_draw_mesh(mesh, material, transform)` |
| **camera** | where you look from and in which direction | a `camera` struct |
| **light** | the sun and sky, plus any lamps, torches or fires | the sun is on by default; add lights with `pix_add_light` |

Meshes and materials are created **once**, before the game loop. They live on
the GPU and you refer to them by handle (`idx`). Instances are drawn **every
frame**. Drawing the same mesh 500 times with different transforms is normal,
and pix makes it cheap.

## A first scene

```cpp
#define PIX_IMPLEMENTATION
#include "pix.h"

int main() {
    if (!pix_init("3D", 1280, 720)) return 1;

    // created once
    idx ground = pix_gen_plane(40, 40);                       // 40 x 40 metres, facing up
    idx cube   = pix_gen_cube(v3(1, 1, 1));                   // 1 metre on each side
    idx grass  = pix_make_material(pix_rgba(110, 150, 80));
    idx red    = pix_make_material(pix_rgba(200, 50, 45), 0.0f, 0.35f);   // colour, metallic, roughness

    while (pix_frame()) {
        if (pix_key_pressed(KEY_ESC)) pix_quit();
        float t = (float)pix_time();

        camera cam = {};
        cam.position  = v3(0, 4, 8);                          // above and behind the origin
        cam.direction = v3(0, -0.4f, -1);                     // looking forward and a little down

        pix_begin_3d(cam);
        pix_draw_mesh(ground, grass, mat4_identity());
        pix_draw_mesh(cube, red, mat4_translate(0, 0.5f, 0) * mat4_rotate_y(t));   // spins
        pix_end_3d();

        pix_draw_fps(10, 10);                                 // 2D after 3D
    }
    pix_shutdown();
}
```

You get a spinning red cube on a green floor, lit by a morning sun, with a sky
behind it and a soft shadow under the cube.

The steps every frame are:

1. `pix_begin_3d(cam)` starts the 3D frame from a camera.
2. `pix_draw_*` calls add what to draw.
3. `pix_end_3d()` renders it all to the window.

Then you draw any 2D on top.

## Coordinates and units

- **Y is up.** The ground is the XZ plane at `y = 0`.
- Units are yours to choose, but the defaults for fog, shadow distance,
  light falloff and physics gravity are tuned for **1 unit = 1 metre**.
  A person is about 1.8 units tall.
- Angles are in **radians**. `PIX_DEG2RAD` converts degrees:
  `mat4_rotate_y(90 * PIX_DEG2RAD)`. Also available: `PIX_PI`, `PIX_TAU`.

## Transforms

A transform is a `mat4` that places an object. Build it from pieces:

```cpp
mat4 a = mat4_translate(x, y, z);         // move
mat4 b = mat4_rotate_y(angle);            // turn around the vertical axis (also _x, _z)
mat4 c = mat4_scale(2, 2, 2);             // resize
mat4 d = mat4_identity();                 // no change: at the origin, unrotated, actual size
```

Combine them by multiplying (`*` or `mat4_mul`). **Read a product from right
to left**: the rightmost piece happens first.

```cpp
// scale, then rotate, then move: the usual order for placing an object
mat4 m = mat4_translate(5, 0, 2) * mat4_rotate_y(angle) * mat4_scale(2, 2, 2);
```

If you swap the order, `mat4_rotate_y(a) * mat4_translate(5, 0, 0)` moves the
object out first and then swings it around the origin, like a planet in orbit.

For the very common "position, heading, size" case there is a shortcut:

```cpp
mat4 m = mat4_trs_y(v3(5, 0, 2), angle, 2.0f);   // same as the line above, computed directly
```

Arbitrary 3D rotations use quaternions:
`mat4_from_trs(position, quat_from_euler(pitch, yaw, roll), scale)`.

## The camera

```cpp
camera cam = {};                   // start from all zeros
cam.position  = v3(0, 2, 6);
cam.direction = v3(0, 0, -1);      // which way it looks; length doesn't matter
```

Fields left at zero use defaults:

| field | default | meaning |
|---|---|---|
| `up` | (0, 1, 0) | which way is up for the camera |
| `fovy` | about 60° | vertical field of view, radians. Larger is wider. |
| `near_plane`, `far_plane` | 0.1, 1000 | nothing closer or farther than these is drawn |
| `ortho_height` | 0 | when above 0, the camera is orthographic (no perspective) and shows this many units vertically. Good for isometric and strategy views. |

To look **at** a point, subtract positions:

```cpp
cam.direction = target - cam.position;
```

### An orbit camera

This camera circles a point, steered with the right mouse button and the
wheel. It is from `examples/hello_3d.cpp`:

```cpp
float yaw = 0.8f, pitch = 0.45f, dist = 16.0f;     // outside the loop

// each frame:
if (pix_mouse_down(PIX_MOUSE_RIGHT)) {
    vec2 d = pix_mouse_delta();
    yaw  += d.x * 0.006f;
    pitch = clampf(pitch + d.y * 0.006f, 0.05f, 1.45f);
}
dist = clampf(dist - pix_mouse_wheel() * 1.5f, 4.0f, 60.0f);

vec3 focus = v3(0, 1, 0);
camera cam = {};
cam.position  = focus + v3(cosf(yaw) * cosf(pitch), sinf(pitch), sinf(yaw) * cosf(pitch)) * dist;
cam.direction = focus - cam.position;
```

### A third-person follow camera

This assumes the player is drawn with `mat4_rotate_y(player_yaw)` and its
model faces -Z, so it looks along `(-sin yaw, 0, -cos yaw)`:

```cpp
vec3 behind = v3(sinf(player_yaw), 0, cosf(player_yaw)) * 6.0f;   // 6 m behind the player
vec3 want = player_pos + behind + v3(0, 3, 0);
cam_pos += (want - cam_pos) * (1.0f - expf(-5.0f * pix_dt()));   // smooth follow
cam.position  = cam_pos;
cam.direction = (player_pos + v3(0, 1.5f, 0)) - cam_pos;          // look at the head
```

## Meshes

### Built-in shapes

```cpp
idx cube   = pix_gen_cube(v3(2, 1, 1));            // width, height, depth
idx sphere = pix_gen_sphere(0.5f);                 // radius; optional rings, segments
idx plane  = pix_gen_plane(10, 10, 4.0f);          // width, depth, texture repeats
idx pillar = pix_gen_cylinder(0.3f, 3.0f);         // radius, height
```

Shapes are **centred on the origin**. A 1-metre cube drawn at `y = 0` is half
under the floor, so lift it by half its height: `mat4_translate(0, 0.5f, 0)`.
Planes lie flat and face up.

These shapes are good for prototypes, level blockouts and simple games. For
anything detailed, load a model.

### Your own geometry

If you generate geometry in code (terrain, procedural buildings), fill arrays
of `vertex` (position, normal, uv) and `uint32_t` indices (three per
triangle), then upload them:

```cpp
idx mesh = load_mesh_data(pix_get_renderer(), vertices, vertex_count, indices, index_count);
```

Once uploaded, pix doesn't need your arrays any more.

## Materials

A material describes a surface. The quick way to make one:

```cpp
idx stone = pix_make_material(pix_rgba(180, 176, 168), /*metallic*/ 0.0f, /*roughness*/ 0.8f);
idx gold  = pix_make_material(pix_rgba(255, 200, 90),  1.0f, 0.25f);
idx glass = pix_make_material(pix_rgba(120, 200, 255, 110), 0.0f, 0.05f);   // alpha < 255: see-through
idx crate = pix_make_material_texture(pix_load_texture("assets/crate.png"));
```

- **metallic** is 0 for almost everything (wood, stone, plastic, skin, cloth)
  and 1 for bare metal.
- **roughness** is 0 for a mirror-smooth surface with sharp highlights and 1
  for a dull, matte one. Most things sit between 0.4 and 0.9.
- A colour with alpha below 255 makes the material **transparent**.

### Changing a material

`pix_get_material(renderer, id)` returns a pointer to the material, so you can
change any of its fields, at any time:

```cpp
pix_renderer& r = pix_get_renderer();
material* m = pix_get_material(r, lava);
m->emissive = v3(6.0f, 1.5f, 0.2f);     // glows; values above 1 bloom
m->unlit = true;                        // ignores lighting entirely
```

To build a material field by field, start from the defaults:

```cpp
material m = pix_material_default();
m.color = v3(0.9f, 0.9f, 1.0f);
m.toon = 1.0f;                          // cartoon shading
m.rim = 0.4f;
idx hero_mat = pix_add_material(pix_get_renderer(), m);
```

### Material fields

| field | default | meaning |
|---|---|---|
| `color` | (1, 1, 1) | colour multiplied with the texture, 0..1 per channel |
| `texture` | 0 (plain white) | GL texture id, such as `pix_load_texture(...).id` |
| `metallic` | 0 | 0 = non-metal, 1 = metal |
| `roughness` | 0.6 | 0 = polished, 1 = matte |
| `emissive` | (0, 0, 0) | light the surface gives off. Above 1 it glows through bloom. |
| `alpha` | 1 | opacity, for blended materials |
| `blend` | `PIX_BLEND_OPAQUE` | `PIX_BLEND_ALPHA` for transparency, `PIX_BLEND_ADDITIVE` for glows and fire |
| `alpha_cutoff` | 0.5 | on opaque materials, texels with alpha below this are cut out (leaves, fences) |
| `cutout` | false | set it on materials whose textures really have holes (foliage), for faster drawing |
| `double_sided` | false | draw back faces too: leaves, cloth, single planes |
| `unlit` | false | ignore lighting |
| `cast_shadows` | true | |
| `shadow_only` | false | casts a shadow but is invisible: a cheap stand-in for something detailed |
| `toon` | 0 | 0 = realistic shading, 1 = cartoon (cel) shading; in between blends them |
| `toon_bands` | 3 | how many light steps cartoon shading has, 2..5 |
| `rim` | 0 | brightness of an outline glow at the edges of the object |
| `toon_specular` | 0 | size of a hard cartoon highlight; about 0.3 is typical |
| `normal_map` | 0 | a texture of surface bumps (tangent-space normals, green up): detail without extra triangles |
| `normal_scale` | 1 | how strong the bumps are |
| `orm_map` | 0 | red = occlusion, green = roughness, blue = metallic (the glTF packing); multiplies those fields |
| `occlusion` | 1 | how much of the map's occlusion darkens the surface |
| `emissive_map` | 0 | multiplies `emissive`: glowing windows, screens, eyes |
| `planar_reflection`, `refraction` | 0 | mirror and glass effects; see `examples/reflections.cpp` |
| `shader`, `textures[4]`, `params0/1` | | custom shaders; see `examples/pipeline.cpp` |

### Ready-made materials

```cpp
pix_renderer& r = pix_get_renderer();
idx glass  = pix_add_material(r, pix_material_glass(v3(0.9f, 1, 1)));
idx water  = pix_add_material(r, pix_material_water(r));        // animated waves, foam
idx toon_w = pix_add_material(r, pix_material_water_toon(r));
idx mirror = pix_add_material(r, pix_material_mirror());
```

Water and mirrors look best with a reflection plane at their height, for
example `pix_set_reflection_plane(r, v3(0, 1, 0), v3(0, water_height, 0))`.
See `examples/reflections.cpp`.

## Models

pix loads `.obj` (with its `.mtl` file) and glTF (`.gltf` / `.glb`) models.
For new assets, prefer `.glb`: it is a single file with textures and
animations inside.

```cpp
pix_model house = pix_load_model("assets/house.glb");    // once, before the loop
if (house.mesh == PIX_INVALID) pix_log("house failed to load");

// each frame:
pix_draw_model(house, mat4_translate(10, 0, 0));
```

A `pix_model` holds a `mesh`, a `material` made from its texture, and its
bounds (`bounds_min`, `bounds_max`). Those are useful for sizing a model that
was exported at the wrong scale:

```cpp
float height = house.bounds_max.y - house.bounds_min.y;
float s = 4.0f / height;                                       // make it 4 metres tall
pix_draw_model(house, mat4_translate(10, 0, 0) * mat4_scale(s, s, s));
```

You can replace the model's material with your own:
`pix_draw_mesh(house.mesh, my_material, transform)`.

A glTF model brings its whole metal/rough material: colour, normal map,
roughness/metallic/occlusion and emissive maps (PNG textures; other image
formats are skipped), and its metallic, roughness and emissive values.

When a model is no longer needed - leaving a level, say -
`pix_unload_model(house)` frees its mesh, material, textures and animations.
The space it used in the GPU buffers is reused by the next load. The lower
level calls are `pix_unload_mesh(r, mesh)`, `pix_unload_material(r, material)`
and, for sounds, `pix_unload_sound(id)`.

### Animated characters

A glTF model with a skeleton (`model.skinned == true`) comes with its
animation clips. An **animator** plays them:

```cpp
pix_model fox = pix_load_model("assets/glTF/Fox/Fox.glb");
animator anim = pix_model_animator(fox);       // has every clip; plays clip 0
animator_play(anim, 2);                        // switch to clip 2

// each frame:
animator_update(anim, pix_dt());               // advance the animation
pix_begin_3d(cam);
pix_draw_model_animated(fox, anim.pose, transform);
pix_end_3d();
```

`fox.clip_count` tells you how many clips there are. Each clip in
`fox.clips[i]` has a name, so you can find a clip like "Run" by searching for
its name. `examples/animation.cpp` shows smooth blending between clips and
playing footstep sounds in time with the feet. `examples/crowd.cpp` shows
`pix_anim_system`, which animates hundreds of characters on worker threads.

## Lights

A new scene already has a **sun** and a **sky**, set to 10:00 in the morning.
The sun is the only light that casts shadows.

### Time of day

The quickest way to set the mood is the time of day. It sets the sun's angle
and colour, the sky colours, the fog and the exposure together:

```cpp
pix_time_of_day(17.5f);                 // 0..24: 6 = sunrise, 12 = noon, 18 = sunset, 22 = night
```

A day/night cycle is one line per frame:

```cpp
hour = fmodf(hour + pix_dt() * 0.1f, 24.0f);     // a full day in four minutes
pix_time_of_day(hour);
```

### Lamps, torches and fires

Point and spot lights are added **every frame**, like instances:

```cpp
// position, radius (the light reaches zero here), colour (brightness included)
pix_add_light(pix_point_light(v3(4, 1.5f, -1), 9.0f, v3(9.0f, 4.5f, 1.8f)));

// position, direction, radius, inner and outer cone angles (radians), colour
pix_add_light(pix_spot_light(lamp_pos, v3(0, -1, 0), 12.0f, 0.3f, 0.5f, v3(10, 9, 8)));
```

The colour is **brightness times colour**, and values above 1 are normal:
`v3(9, 4.5f, 1.8f)` is a strong orange. If a light seems too weak, multiply
its colour.

Up to 512 lights can be added per frame. The 32 nearest to the camera are
used. Point and spot lights don't cast shadows.

A flickering fire combines a glowing mesh, a light and some particles:

```cpp
float flicker = 0.85f + 0.15f * sinf(t * 17.0f);
pix_draw_mesh(sphere, ember_material, mat4_translate(fire.x, fire.y, fire.z));   // emissive material
pix_add_light(pix_point_light(fire + v3(0, 1, 0), 9.0f, v3(9, 4.5f, 1.8f) * flicker));
```

### Full control

Games without a day cycle can set each part directly. These take the
renderer:

```cpp
pix_renderer& r = pix_get_renderer();
pix_set_sun(r, pix_sun_direction(/*elevation*/ 40, /*azimuth*/ 120), v3(3, 2.8f, 2.5f));
pix_set_sky_colors(r, zenith, horizon, ground);
pix_set_ambient(r, sky_light, ground_light);   // the light in shadowed areas
pix_set_fog(r, v3(0.7f, 0.75f, 0.8f), 0.004f);  // colour, density (0 = off)
r.sky_enabled = false;                          // plain background (r.clear_color) instead of the sky
```

## Particles

For sparks, embers and magic, push particles every frame. Each is a glowing
dot:

```cpp
// position, half-size in metres, colour (above 1 glows), opacity
push_particle(pix_get_renderer(), p, 0.06f, v3(8.0f, 3.0f, 0.8f), 0.7f);
```

You move the particles yourself; pix only draws them. A simple fountain of
sparks from `examples/hello_3d.cpp`:

```cpp
for (int i = 0; i < 40; i++) {
    float life = fmodf(t * 0.7f + i * 0.137f, 1.0f);              // 0..1, repeating
    vec3 p = fire + v3(sinf(i * 12.9f) * 0.3f, 0.4f + life * 2.5f, cosf(i * 7.3f) * 0.3f);
    push_particle(r, p, 0.06f * (1.0f - life), v3(8.0f, 3.0f, 0.8f), 1.0f - life);
}
```

## Billboards and 3D text

Pictures and text standing in the world: trees far away, health bars, names
over heads, damage numbers, signs. They are not lit (fog still applies).

```cpp
pix_draw_billboard(tree_tex, v3(10, 2, 0), 4.0f);                         // 4 m tall, turns to face you
pix_draw_billboard(tree_tex, pos, 4.0f, PIX_WHITE, PIX_FACE_UPRIGHT, true); // turns about y only, hard edges
pix_draw_text3d("Bob", head + v3(0, 0.4f, 0), 0.25f, PIX_WHITE);          // 25 cm tall letters, centred
pix_draw_text3d_fixed("EXIT", wall_transform, 0.5f, PIX_GREEN);            // written on a wall
```

| facing | |
|---|---|
| `PIX_FACE_CAMERA` | square to the camera: labels, particles, effects |
| `PIX_FACE_UPRIGHT` | turns about the vertical only: trees and characters as sprites |
| `PIX_FACE_FIXED` | stays on its own axes (`pix_draw_text3d_fixed`) |

The last argument, `cutout`, draws hard-edged pictures (leaves with clear
gaps) in the opaque pass, where they need no sorting and hide what is behind
them. Without it they are blended and sorted back to front. For a frame of
an atlas, `pix_draw_billboard_pro(tex, src_rect, pos, size, origin, ...)`;
`origin (0.5, 0)` hangs it from its bottom centre.

## Levels of detail

A mesh can carry simpler stand-ins that pix swaps in by itself as it gets
smaller on screen. You keep drawing the full mesh:

```cpp
pix_generate_mesh_lods(pix_get_renderer(), rock.mesh);   // once: 3 simpler versions, by simplification
// or your own: pix_set_mesh_lods(r, base, lods, screen_sizes, count)
```

The change happens over a short distance as a fine dither between the two
versions, so it does not pop. `r.lod_band` sets that distance (0.2; 0 = a hard
switch) and `r.lod_bias` trades detail for speed (above 1 keeps detail
longer). Shadows and reflections use the same choice as the main view.

## Tinting instances

Every draw call takes an optional **tint** colour that multiplies the
material's colour for that one instance. It is a cheap way to vary many
copies of one object, flash an enemy red when hit, or fade something out:

```cpp
pix_draw_mesh(crate, wood, m, pix_rgba(255, 220, 200));       // slightly warmer
pix_draw_model(enemy, m, hit ? PIX_RED : PIX_WHITE);
```

## How the picture looks: shadows, quality, post-processing

### Quality presets

```cpp
pix_set_quality(PIX_QUALITY_HIGH);      // LOW, MEDIUM, HIGH (the default), ULTRA
```

A preset sets shadow sharpness and size, bloom, anti-aliasing, the
reflection probe and how small an object may get before it is skipped
(`r.cull_pixels`, see [Performance](#performance)). Offer it as a graphics
setting in your menu, and use `LOW` on weak laptops.

| | `LOW` | `MEDIUM` | `HIGH` | `ULTRA` |
|---|---|---|---|---|
| shadow resolution | 1024 | 1536 | 2048 | 4096 |
| shadow softness | 0 (hard) | 1 | 1 | 2 (softest) |
| bloom | off | on | on | on |
| temporal anti-aliasing | off | on | on | on |
| reflection probe | off | off | 128 px | 256 px |
| objects skipped below | 1.5 px | 1 px | 0.5 px | 0.25 px |

### Anti-aliasing

Temporal AA (`r.aa.taa`, on by default) moves the camera by a fraction of a
pixel every frame and blends the frames: smooth edges, steady thin wires and
highlights, for the price of one pass. Things that move by themselves stay
sharp when you give them a `motion_id` - any number of yours, the same every
frame:

```cpp
pix_draw_model(car, car_transform, PIX_WHITE, /*motion_id*/ 1);
```

Without one they still look right at ordinary speeds; very fast objects can
leave a faint trail. After a camera cut, `pix_taa_reset(r)` stops the old view
blending into the new. `r.aa.taa_feedback` (0.9) is how much of the past each
frame keeps.

`r.aa.msaa = 4` renders the main view with 4 samples per pixel instead (or
as well): sharp, no blending over time, more memory and fill rate.
`r.post.fxaa` is the cheapest option of all.

### A real sky: HDR environments

An HDR panorama (a Radiance `.hdr` file, the format sites like Poly Haven
offer) can light the whole scene:

```cpp
pix_load_environment(pix_get_renderer(), "assets/sky/meadow.hdr");   // once
```

It becomes the sky behind everything, the reflections in shiny surfaces
(blurrier on rougher ones) and the soft light from all around. The sun still
adds its light and shadows on top: set it to match the panorama's sun with
`pix_set_sun`. `pix_clear_environment(r)` goes back to the built-in sky.

### Shadows

`pix_shadows()` returns the shadow settings. Change any field directly:

```cpp
pix_shadows().distance = 60.0f;     // how far from the camera shadows reach, in metres
pix_shadows().softness = 2;         // 0 hard .. 2 soft
pix_shadows().enabled  = false;     // no shadows at all
```

| field | meaning |
|---|---|
| `enabled` | turning it off frees the shadow memory |
| `resolution` | sharpness: texels per cascade side |
| `distance` | how far from the camera shadows reach. Shorter is sharper. |
| `strength` | 0..1, how dark full shadow is |
| `softness` | 0 = hard edges, 1, 2 = softest |
| `skinned` | whether animated characters cast shadows |
| `depth_range`, `skinned_cascades`, `far_resolution`, `stagger` | fine-tuning for speed; see the comments in `renderer.hpp` |

Shadows use three **cascades**: a sharp one near the camera and coarser ones
farther away.

### Post-processing

After the scene is rendered, the image goes through post-processing: bloom
(glow around bright things), tone mapping (turning very bright values into
displayable colours), colour grading and anti-aliasing. `pix_post()` holds
the settings:

```cpp
pix_post().bloom_strength = 0.5f;
pix_post().exposure       = 1.2f;              // brighter
pix_post().saturation     = 1.1f;              // more colourful
pix_post().vignette       = 0.3f;              // darker corners
pix_post().color_filter   = v3(1.0f, 0.95f, 0.85f);   // warm tint
pix_post().tonemap        = PIX_TONEMAP_ACES;  // or PIX_TONEMAP_REINHARD, PIX_TONEMAP_NONE
pix_post().fxaa           = true;              // smooth jagged edges
```

| field | meaning |
|---|---|
| `exposure` | overall brightness multiplier |
| `tonemap` | `PIX_TONEMAP_ACES` (filmic, the default), `PIX_TONEMAP_REINHARD` (softer), `PIX_TONEMAP_NONE` (exact colours, for flat stylised games) |
| `bloom`, `bloom_strength`, `bloom_threshold`, `bloom_knee` | glow around anything brighter than the threshold |
| `saturation` | 1 = unchanged, 0 = black and white |
| `contrast` | 0 = unchanged, about 0.3 = punchy |
| `vignette` | darkened corners, 0 = off |
| `sharpen` | 0 = off, about 0.3 = subtle |
| `color_filter` | multiplies the final image; (1, 1, 1) = none |
| `fxaa` | anti-aliasing |
| `dither` | removes colour banding in skies and fog |

Settings changes take effect on the next frame and cost nothing to make.

### Screen effects

Full-screen effects run after everything else. Add them once, then switch
them on and off:

```cpp
int crt = pix_add_effect(PIX_EFFECT_CRT, v4(0.6f, 0.08f, 0, 0));   // once
pix_effect_enable(crt, false);
// later:
pix_effect_enable(crt, true);
pix_effect_params(crt, v4(0.8f, 0.1f, 0, 0));
```

| effect | what the `params` numbers mean |
|---|---|
| `PIX_EFFECT_GRAYSCALE` | x: amount, 0..1 |
| `PIX_EFFECT_SEPIA` | x: amount, 0..1 |
| `PIX_EFFECT_PIXELATE` | x: block size in pixels |
| `PIX_EFFECT_CHROMATIC` | x: colour fringe width in pixels at the edges |
| `PIX_EFFECT_CRT` | x: scanline strength 0..1, y: screen curve (about 0.1) |
| `PIX_EFFECT_OUTLINE` | cartoon ink lines. x: thickness in pixels, y: sensitivity (0.05), z: darkness 0..1 |
| `PIX_EFFECT_DEPTH_BLUR` | blur away from a focus distance. x: focus distance, y: focus range, z: maximum blur in pixels |

The same effects work in 2D games; see
[Screen effects in 2D](2d-rendering.md#screen-effects-in-2d).

Up to 8 effects can be added. You can also write your own as a short GLSL
`main()` function:

```cpp
int invert = pix_add_effect(R"( void main() { frag = vec4(1.0 - texture(uScene, vUV).rgb, 1.0); } )");
```

Inside it, `uScene` is the image so far, `vUV` the pixel's position (0..1),
`uParams` your four numbers, and `linear_depth(vUV)` the distance to the
scene. `pix_add_effect` returns -1 if the shader doesn't compile, and the
compiler error is printed to the console.

## Debug drawing

Lines for visualising things while you develop. They are drawn in 3D, hidden
behind objects, and unaffected by lighting:

```cpp
pix_draw_line3d(a, b, PIX_RED);
pix_draw_bounds(box_min, box_max, PIX_GREEN);           // a wireframe box
pix_draw_grid(40, 40, pix_rgba(255, 255, 255, 60));     // size, cells, colour
```

## Picking: what is under the mouse

`pix_screen_to_ray` turns a screen position into a direction in the world,
starting at the camera. Test that ray against your objects. For example, to
find where the mouse points on the ground:

```cpp
pix_renderer& r = pix_get_renderer();
vec2 m = pix_mouse_position();
vec3 dir = pix_screen_to_ray(r, m.x, m.y);
float dist;
if (ray_vs_plane(cam.position, dir, plane_make(v3(0, 1, 0), 0), 1000.0f, false, &dist)) {
    vec3 hit = cam.position + dir * dist;      // the ground point under the mouse
}
```

Call it after `pix_begin_3d`, so it uses this frame's camera.
`ray_vs_sphere`, `ray_vs_aabb` and the physics worlds' raycasts work the same
way. See [Physics](physics.md#shape-tests-without-a-world).

The other direction, from world to screen, places 2D labels over 3D objects:

```cpp
vec2 px;
if (pix_world_to_screen(r, enemy_pos + v3(0, 2.2f, 0), &px))   // false if behind the camera
    pix_draw_text("Wolf", px.x - 20, px.y, 18, PIX_WHITE);
```

## Performance

pix is built so that ordinary scenes are fast without any effort on your part:

- **Draw the same mesh many times freely.** All instances of a mesh with the
  same material go to the GPU in one call. A city of 2,600 objects draws in
  about 40 calls.
- **Objects outside the view are skipped** automatically, and so are objects
  too small to see: smaller than `r.cull_pixels` pixels across on screen
  (0.5 by default), or than that many shadow texels in a shadow cascade.
  Raising it to 1 or 2 drops far more of a large scene's distant clutter;
  the cost is that a field of thousands of far-off specks, which together
  do colour the horizon, thins out. 0 draws everything.
- **Fewer distinct materials help.** Use one material with different tints
  rather than many nearly identical materials.
- Measure: after `pix_end_3d()`, `r.frame_draws`, `r.frame_instances` and
  `r.frame_culled` describe the last frame. For per-stage GPU times, call
  `pix_enable_gpu_timing(r, true)` and read
  `pix_gpu_pass_ms(r, PIX_GPU_SHADOW)` (also `_REFLECT`, `_VIEW`, `_POST`).

Default limits, which you can raise by defining them before including pix:

| macro | default |
|---|---|
| `PIX_MAX_INSTANCES` | 65536 objects per frame |
| `PIX_MAX_ANIMATED` | 8192 animated characters per frame |
| `PIX_MAX_MESHES`, `PIX_MAX_MATERIALS` | 8192 each |
| `PIX_MAX_PARTICLES` | 16384 per frame |
| `PIX_MAX_LINES` | 32768 debug lines per frame |
| `PIX_MAX_QUADS` | 16384 billboards and 3D-text letters per frame |

## Screenshots

```cpp
if (pix_key_pressed(KEY_F12)) pix_screenshot("shot.png");
```

saves the frame exactly as shown, 2D on top included, when it is presented.

## Going further

These are covered by the examples for now:

| topic | example |
|---|---|
| reflections, water, glass | `examples/reflections.cpp` |
| several cameras, minimaps, layers, custom shaders, custom passes | `examples/pipeline.cpp` |
| animation blending, footsteps | `examples/animation.cpp` |
| crowds | `examples/crowd.cpp` |
| GPU compute, render hooks, huge instance counts | `examples/grassland.cpp` |
| the renderer without the app layer | `examples/city.cpp` |

## Reference

### App layer

| function | |
|---|---|
| `pix_begin_3d(camera& cam)` | start the 3D frame |
| `pix_end_3d()` | render it to the window |
| `pix_draw_mesh(mesh, material, transform, tint = WHITE, motion_id = 0)` | |
| `pix_draw_model(model, transform, tint = WHITE, motion_id = 0)` | |
| `pix_unload_model(model)`, `pix_unload_sound(id)` | free what a level no longer needs |
| `pix_draw_billboard(tex, pos, height, tint, facing, cutout)`, `pix_draw_billboard_pro(...)` | pictures in the world |
| `pix_draw_text3d(text, pos, height, color, facing)`, `pix_draw_text3d_fixed(text, transform, height, color)` | text in the world |
| `pix_screenshot(path)` | saves this frame as a PNG |
| `pix_draw_model_animated(model, pose, transform, tint = WHITE)` | `pose` (usually `anim.pose`) must stay alive until `pix_end_3d` |
| `pix_draw_line3d(a, b, color)` | |
| `pix_draw_bounds(min, max, color)` | |
| `pix_draw_grid(size, cells, color = GRAY)` | |
| `pix_add_light(light)` | this frame only |
| `pix_time_of_day(hour)` | |
| `pix_gen_cube(size)`, `pix_gen_sphere(radius, rings = 16, segments = 32)`, `pix_gen_plane(w, d, uv_scale = 1)`, `pix_gen_cylinder(radius, height)` | return mesh handles |
| `pix_make_material(color, metallic = 0, roughness = 0.6)` | |
| `pix_make_material_texture(texture, color = WHITE, metallic = 0, roughness = 0.6)` | |
| `pix_load_model(path)` | `.obj`, `.gltf`, `.glb` |
| `pix_model_animator(model)` | |
| `pix_set_quality(q)` | `PIX_QUALITY_LOW` .. `_ULTRA` |
| `pix_shadows()`, `pix_post()` | settings, edited in place |
| `pix_add_effect(body, params)`, `pix_effect_enable(fx, on)`, `pix_effect_params(fx, params)` | |
| `pix_get_renderer()` | the renderer, for everything below |

### Renderer functions used on this page

| function | |
|---|---|
| `pix_get_material(r, id)` | pointer to edit a material |
| `pix_unload_mesh(r, id)`, `pix_unload_material(r, id)` | |
| `pix_generate_mesh_lods(r, mesh, levels, ratio)`, `pix_set_mesh_lods(r, base, lods, screen, count)` | levels of detail |
| `pix_load_environment(r, path, intensity)`, `pix_set_environment(r, rgb, w, h, intensity)`, `pix_clear_environment(r)` | HDR sky and lighting |
| `pix_taa_reset(r)` | after a camera cut |
| `pix_material_default()`, `pix_add_material(r, m)` | build a material field by field |
| `pix_material_glass/mirror/water/water_toon(...)` | ready-made materials |
| `load_mesh_data(r, vertices, vertex_count, indices, index_count)` | upload your own mesh |
| `pix_set_sun(r, dir, color)`, `pix_set_sky_colors(r, zenith, horizon, ground)`, `pix_set_ambient(r, sky, ground)`, `pix_set_fog(r, color, density)` | lighting by hand |
| `pix_set_reflection_plane(r, normal, point)` | mirror plane for water and mirrors |
| `push_particle(r, pos, half_size, color, alpha)` | |
| `pix_screen_to_ray(r, x, y)`, `pix_world_to_screen(r, world, &px)` | |
| `pix_enable_gpu_timing(r, on)`, `pix_gpu_pass_ms(r, pass)`, `pix_cpu_pass_ms(r, pass)` | |

### Lights (`core/lighting.hpp`)

| function | |
|---|---|
| `pix_point_light(position, radius, color)` | |
| `pix_spot_light(position, direction, radius, inner, outer, color)` | angles in radians |
| `pix_sun_direction(elevation_deg, azimuth_deg)` | a direction for `pix_set_sun` |

### Transforms (`core/math.hpp`)

| function | |
|---|---|
| `mat4_identity()` | |
| `mat4_translate(x, y, z)`, `mat4_scale(x, y, z)` | |
| `mat4_rotate_x/y/z(radians)`, `mat4_rotate(axis, radians)` | |
| `mat4_trs_y(position, yaw, scale)` | position + heading + uniform scale |
| `mat4_from_trs(position, quat, scale)` | full 3D |
| `mat4_mul(a, b)` or `a * b` | `b` is applied first |
| `mat4_inverse(m)`, `mat4_mul_point(m, p)`, `mat4_mul_dir(m, d)`, `mat4_get_translation(m)` | |
| `quat_from_euler(pitch, yaw, roll)`, `quat_from_axis_angle(axis, radians)`, `quat_slerp(a, b, t)` | |
