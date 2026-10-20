# Cheatsheet

The common calls on one page. Each section links to the page that explains
it.

## Setup and the frame loop ([Getting started](getting-started.md))

```cpp
#define PIX_IMPLEMENTATION          // in exactly one .cpp
#include "pix.h"

int main() {
    if (!pix_init("title", 1280, 720)) return 1;     // optional 4th arg: PIX_WINDOW_* | PIX_NO_* flags
    // load assets here, once
    while (pix_frame()) {
        float dt = pix_dt();                          // seconds this frame (max 0.1)
        // input, update, draw 3D, then draw 2D
    }
    pix_shutdown();
}
```

```
cl /std:c++14 /O2 /EHsc game.cpp          (with dist/pix.h next to game.cpp)
```

| | |
|---|---|
| `pix_quit()` | end the loop |
| `pix_time()` | seconds since start |
| `pix_fps()` | frames per second |
| `pix_width()`, `pix_height()` | window size |
| `pix_set_title(s)`, `pix_toggle_fullscreen()`, `pix_set_vsync_enabled(b)` | |
| `pix_log("hp %d", hp)` | print to the console |

## Input ([Input](inputs.md))

```cpp
pix_key_down('W')              // held        (letters upper-case)
pix_key_pressed(KEY_SPACE)     // this frame only
pix_key_released(KEY_ESC)
vec2 move = pix_input_axis();  // WASD / arrows / left stick, -1..1, +y = up

pix_mouse_position()           // pixels, top-left origin
pix_mouse_delta()              // movement this frame
pix_mouse_wheel()              // notches
pix_mouse_pressed(PIX_MOUSE_LEFT)        // _RIGHT, _MIDDLE
pix_set_mouse_locked(true)               // mouse look

const pix_gamepad& pad = pix_get_gamepad(0);
pad.connected   pad.buttons[PAD_A].pressed   pad.left_x   pad.left_y   pad.right_trigger
pix_set_gamepad_rumble(pix_get_window(), 0, low, high);   // 0..1, held until changed

int n; const unsigned int* typed = pix_text_input(&n);   // typed characters
```

Keys: `KEY_UP/DOWN/LEFT/RIGHT`, `KEY_SPACE`, `KEY_RETURN`, `KEY_ESC`,
`KEY_TAB`, `KEY_BACKSPACE`, `KEY_SHIFT/CTRL/ALT`, `KEY_F1..F12`,
`KEY_DELETE`, `KEY_HOME`, `KEY_END`, `KEY_PAGE_UP/DOWN`.
Pad buttons: `PAD_A/B/X/Y`, `PAD_LEFT/RIGHT_BUMPER`, `PAD_BACK`, `PAD_START`,
`PAD_LEFT/RIGHT_STICK`, `PAD_DPAD_UP/DOWN/LEFT/RIGHT`.

## 2D ([2D rendering](2d-rendering.md))

Pixels, `(0, 0)` at top-left, Y down. Later draws cover earlier ones.

```cpp
pix_clear(color);
pix_draw_rect(x, y, w, h, color);                 pix_draw_rect(rect, color);
pix_draw_rect_lines(rect, thickness, color);      pix_draw_rect_rounded(rect, radius, color);
pix_draw_rect_gradient(rect, top, bottom);
pix_draw_circle(center, r, color);                pix_draw_circle_lines(center, r, thickness, color);
pix_draw_line(a, b, thickness, color);            pix_draw_triangle(a, b, c, color);

pix_texture t = pix_load_texture("a.png", TEXTURE_PIXELATED);   // or TEXTURE_LINEAR; id == 0 on failure
pix_draw_texture(t, x, y, tint);
pix_draw_texture_pro(t, src, dst, origin, rotation_deg, tint);  // negative src.w flips

pix_draw_text("hi", x, y, size, color);           // x, y = top-left
pix_draw_textf(x, y, size, color, "score %d", s);
vec2 size = pix_measure_text("hi", 32);
pix_draw_fps(x, y);

pix_camera2d cam = {}; cam.zoom = 1;  cam.target = player;  cam.offset = v2(w / 2, h / 2);
pix_begin_2d(cam);  /* world */  pix_end_2d();    /* HUD after */
pix_begin_clip(rect);  /* ... */  pix_end_clip();
```

Colours: `pix_rgba(r, g, b, a = 255)`, `pix_hex(0xRRGGBBAA)`,
`pix_fade(c, 0.5f)`, `PIX_WHITE`, `PIX_BLACK`, `PIX_BLANK`, `PIX_GRAY`,
`PIX_LIGHTGRAY`, `PIX_DARKGRAY`, `PIX_RED`, `PIX_GREEN`, `PIX_BLUE`,
`PIX_YELLOW`, `PIX_ORANGE`, `PIX_PURPLE`, `PIX_SKYBLUE`.

## 3D ([3D rendering](3d-rendering.md))

Y up, 1 unit = 1 metre, angles in radians.

```cpp
// once
idx cube  = pix_gen_cube(v3(1, 1, 1));    // also pix_gen_sphere(r), pix_gen_plane(w, d), pix_gen_cylinder(r, h)
idx mat   = pix_make_material(pix_rgba(200, 50, 45), /*metallic*/ 0, /*roughness*/ 0.4f);
idx tex_m = pix_make_material_texture(pix_load_texture("crate.png"));
pix_model hero = pix_load_model("hero.glb");          // .obj, .gltf, .glb
animator anim  = pix_model_animator(hero);            // animator_play(anim, clip)

// each frame
camera cam = {};  cam.position = v3(0, 4, 8);  cam.direction = target - cam.position;
animator_update(anim, dt);
pix_begin_3d(cam);
pix_draw_mesh(cube, mat, mat4_translate(0, 0.5f, 0), tint);
pix_draw_model(hero, transform);
pix_draw_model_animated(hero, anim.pose, transform);
pix_add_light(pix_point_light(pos, radius, v3(9, 4.5f, 1.8f)));
pix_end_3d();
// 2D here
```

```cpp
material* m = pix_get_material(pix_get_renderer(), mat);
m->emissive = v3(4, 1, 0);   m->toon = 1;   m->unlit = true;   m->double_sided = true;
```

| look | |
|---|---|
| `pix_time_of_day(17.5f)` | sun, sky, fog for an hour 0..24 |
| `pix_set_quality(PIX_QUALITY_HIGH)` | `LOW`, `MEDIUM`, `HIGH`, `ULTRA` |
| `pix_shadows().distance = 60;` | also `.softness` 0..2, `.enabled`, `.resolution` |
| `pix_post().bloom_strength = 0.5f;` | also `.exposure`, `.saturation`, `.vignette`, `.fxaa`, `.tonemap` |
| `int fx = pix_add_effect(PIX_EFFECT_CRT, v4(...));` | `GRAYSCALE`, `SEPIA`, `PIXELATE`, `CHROMATIC`, `CRT`, `OUTLINE`, `DEPTH_BLUR` |
| `pix_effect_enable(fx, on)` | |
| `pix_set_fog(r, color, density)` | `r` = `pix_get_renderer()` |

| debug and picking | |
|---|---|
| `pix_draw_line3d(a, b, color)`, `pix_draw_bounds(min, max, color)`, `pix_draw_grid(size, cells)` | |
| `push_particle(r, pos, half_size, color, alpha)` | |
| `vec3 dir = pix_screen_to_ray(r, mx, my)` | ray from the camera through a pixel |
| `pix_world_to_screen(r, world, &px)` | false if behind the camera |

## Transforms and math

```cpp
mat4_identity()   mat4_translate(x, y, z)   mat4_scale(x, y, z)   mat4_rotate_y(rad)   // _x, _z
mat4 m = mat4_translate(p.x, p.y, p.z) * mat4_rotate_y(yaw) * mat4_scale(s, s, s);   // right to left
mat4_trs_y(position, yaw, scale)          // the same, faster
mat4_from_trs(position, quat_from_euler(pitch, yaw, roll), scale)
mat4_inverse(m)   mat4_mul_point(m, p)   mat4_mul_dir(m, d)   mat4_get_translation(m)
```

```cpp
v2(x, y)  v3(x, y, z)  v4(x, y, z, w)      // + - * / += -= *= work on vec2 and vec3
v2len  v2norm  v2dot  v2dist  v2lerp  v2rotate(v, rad)
v3len  v3norm  v3dot  v3cross  v3dist  v3lerp  v3min  v3max
clampf(v, lo, hi)  lerpf(a, b, t)  minf  maxf  smoothstepf(e0, e1, x)  approachf(cur, target, max_step)
damp(cur, target, rate, dt)           // smooth follow, same at any frame rate
damp_angle(cur, target, rate, dt)     // the same for angles, takes the short way round
angle_delta(from, to)                 // shortest signed angle between two headings
PIX_PI  PIX_TAU  PIX_DEG2RAD  PIX_RAD2DEG
```

Random numbers:

```cpp
rng r = rng_seed(1234);
rng_float(r)           // 0..1
rng_range(r, lo, hi)   // float
rng_int(r, lo, hi)     // inclusive
rng_chance(r, 0.25f)   // true 25% of the time
hash2_float(x, y, seed)                 // the same value for the same cell, every run
value_noise2(x, y, seed)   fbm2(x, y, octaves, seed)    // smooth noise, 0..1
```

## Physics ([Physics](physics.md))

```cpp
static phys_world world;
phys_create_world(world);                         // gravity (0,-22,0), ground_y 0
world.ground_at = my_height_fn;                   // optional uneven ground
phys_add_static_box(world, min, max);             // walls
phys_add_static_prop(world, base, yaw, half_xz, height);
phys_build_statics(world);                        // after adding statics

phys_body b = {};
b.shape = PHYS_CYLINDER;  b.radius = 0.4f;  b.height = 1.8f;   // or PHYS_BOX: half, yaw
b.position = feet;  b.inv_mass = 1;  b.gravity = true;
b.group = PHYS_LAYER_PLAYER;  b.collides = PHYS_LAYER_ALL;
idx id = phys_add_body(world, b);

// each frame
phys_body* p = phys_get_body(world, id);
p->velocity.x = ...;  p->velocity.z = ...;
if (p->on_ground && jump) p->velocity.y = 8;
phys_step(world, dt);
// draw at p->position (feet)

phys_raycast(world, from, dir, max, &dist)                               // statics
phys_raycast_bodies(world, from, dir, max, mask, ignore, &dist)          // bodies -> id
phys_query_neighbours(world, centre, radius, out, capacity)
phys_point_blocked(world, pos, radius, height)
phys_remove_body(world, id);    phys_destroy_world(world);
```

Shape tests (`contact` pointer required):

```cpp
contact3 c;  collide_sphere_sphere(a, b, &c)  // also sphere_plane/aabb/capsule, capsule_capsule/plane/aabb
contact2 c2; collide_circle_rect(circle, rect, &c2)   // also circle_circle/obb, obb_obb, obb_rect
ray_vs_sphere / ray_vs_capsule / ray_vs_plane / ray_vs_aabb (takes 1/dir)
```

## Audio

```cpp
idx boom  = pix_load_sound("assets/boom.wav");                       // once; PIX_INVALID on failure
idx voice = pix_play_sound(boom, /*volume*/ 1, /*pitch*/ 1.2f, /*loop*/ false, /*pan*/ -0.5f);
pix_stop_sound(voice);
pix_set_volume(0.8f);                                                // master
```

Only WAV files are supported.

## Memory ([Memory](memory.md), optional)

```cpp
unsigned char* f = pix_read_file("a.txt", &size);   pix_free(f);     // buffers you own: pix_free

pix_mem_scope s = pix_scope_begin();                                 // temporary, freed together
unsigned char* t = pix_read_file("a.txt", &size, PIX_MEM_SCOPED);    // same for pix_load_image_rgba,
pix_mesh_data m = pix_mesh_copy(file, PIX_MEM_SCOPED);               // pix_mesh_copy/merge, pix_anim_bone_mask
float* tmp = pix_new<float>(n, PIX_MEM_SCOPED);
pix_scope_end(s);

T* p = pix_new<T>(n);  T* z = pix_new<T>(n, PIX_MEM_ZERO);  pix_free(p);   // your own heap memory
PIX_STACK_BUFFER(int, ids, count, 256);                              // on the stack if it fits
mem_arena a = pix_arena_create(256 * MB);  T* x = allocate<T>(a, n);  arena_reset(a);
pix_mem_log_stats("label");                                          // what is in use
```

## The modules underneath

`pix_get_window()`, `pix_get_renderer()`, `pix_get_batch2d()`,
`pix_get_audio()`, `pix_get_loader()`, `pix_get_jobs()`,
`pix_get_animations()`.
