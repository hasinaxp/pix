# Math

Everything here lives in `core/math.hpp`, with shape tests in
`core/colliders.hpp`. It's all plain functions on plain structs, with no
setup.

If you are new, read [Getting started](getting-started.md#the-basic-types)
for vectors and colours, and [3D rendering](3d-rendering.md#transforms) for
transforms. This page is the full toolbox: start at the top and stop when
you have what you need.

## Conventions

| | |
|---|---|
| up axis | **+Y**. The ground is the XZ plane. |
| handedness | **right-handed**. In view space the camera looks down **-Z**, with +X to its right. |
| units | your choice, but pix's defaults (gravity, fog, shadow distance, light falloff) assume **1 unit = 1 metre** |
| angles | **radians** everywhere in math. `PIX_DEG2RAD` and `PIX_RAD2DEG` convert. The few things that take degrees are 2D drawing rotations (`rotation_deg`, `pix_camera2d.rotation`), hues, `pix_sun_direction` and crease angles. |
| matrices | **column-major** (`data[col * 4 + row]`), and `a * b` applies `b` first |
| Euler order | `quat_from_euler(pitch, yaw, roll)`: yaw about +Y, then pitch about +X, then roll about +Z |
| depth range | OpenGL's default, -1..1 in clip space |
| colours | `pix_color` and colours you type are **sRGB**; lighting values (`emissive`, light colours) are **linear** |
| winding | a triangle's front is the side its corners run **counter-clockwise** from |

## Everyday helpers

```cpp
float t = inverse_lerpf(min_hp, max_hp, hp);         // where hp sits, as 0..1
float w = remapf(hp, 0, 100, 0, 200);                // hp onto a 200 px bar
float a = clampf(x, 0, 1);    float s = saturatef(x);   // the same thing for 0..1
float l = lerpf(a, b, t);     float m = smoothstepf(0, 1, t);
float f = fractf(time);       float h = wrapf(heading, 0, PIX_TAU);   // looping values
float g = signf(v);           float k = approachf(speed, top_speed, accel * dt);
bool  same = approx_equalf(a, b);                    // tolerant float compare
float c = damp(camera_x, player_x, 8.0f, dt);        // smooth follow at any frame rate
float d = angle_delta(heading, target_heading);      // shortest way round, -pi..pi
```

Vectors have `v3dot`, `v3cross`, `v3len`, `v3norm`, `v3dist`, `v3lerp`,
`v3min`, `v3max` (and the `v2` versions), plus:

```cpp
vec3 bounce = v3reflect(velocity, wall_normal);       // off a wall
vec3 slide  = v3reject(velocity, wall_normal);        // along a wall: the into-the-wall part removed
vec3 along  = v3project(velocity, rail_direction);    // the part along a rail
```

## Rotations

```cpp
quat q = quat_from_euler(pitch, yaw, roll);
quat r = quat_from_axis_angle(v3(0, 1, 0), 0.5f);
quat both = r * q;                                    // q first, then r
vec3 p = quat_rotate(q, v3(0, 0, -1));                // where "forward" points now
quat mid = quat_slerp(q, r, 0.5f);                    // halfway, the short way round

float pitch, yaw, roll;  quat_to_euler(q, &pitch, &yaw, &roll);
vec3 axis; float angle;  quat_to_axis_angle(q, &axis, &angle);
quat from_matrix = quat_from_mat4(transform);         // scale is ignored
```

Looking straight up or down, yaw and roll turn about the same axis and can't
be told apart. `quat_to_euler` then reports roll as 0 and puts the whole
turn in yaw, which gives back the same rotation.

## Colour

`pix_color` is 8-bit sRGB: the format of images and colour pickers. For
maths that mixes light, use linear values:

```cpp
vec4 lin = pix_color_to_linear(c);                    // rgb in linear light, a unchanged
pix_color back = pix_color_from_linear(lin);
float l = pix_srgb_to_linear(0.5f);                   // 0.214: sRGB "half" is a fifth of the light
```

HSV and HSL are friendlier for picking and animating colours:

```cpp
pix_color rainbow = pix_hsv((float)pix_time() * 90.0f, 1, 1);   // hue in degrees, wraps
pix_color pastel  = pix_hsl(200, 0.6f, 0.8f);            // lightness 0.5 = pure, 1 = white
vec4 hsv = pix_color_to_hsv(c);                          // x hue, y saturation, z value, w alpha
vec4 hsl = pix_color_to_hsl(c);
```

## Animating values

### Easing

Feed in `t` from 0 to 1, get back a reshaped 0 to 1, and lerp with it:

```cpp
float t = clampf(timer / 0.4f, 0, 1);
float s = lerpf(0.0f, 1.0f, ease_out_back(t));        // a button that pops in with a little overshoot
```

| family | feel |
|---|---|
| `quad`, `cubic`, `quart` | gentle to strong acceleration |
| `sine` | the softest |
| `expo`, `circ` | sharp |
| `back` | overshoots a little, then settles |
| `elastic` | springs |
| `bounce` | bounces |

Each family comes as `ease_in_*` (starts slow), `ease_out_*` (ends slow) and
`ease_in_out_*` (both). For example: `ease_in_quad`, `ease_out_bounce`,
`ease_in_out_sine`.

### Curves

These work on `float`, `vec2` and `vec3`, with `t` from 0 to 1:

```cpp
vec3 p = catmull_rom(p0, p1, p2, p3, t);              // through p1 (t=0) and p2 (t=1)
vec3 b = bezier_cubic(start, handle1, handle2, end, t);
vec3 h = hermite(p0, tangent0, p1, tangent1, t);
vec3 q = bezier_quadratic(start, handle, end, t);
vec3 dir = bezier_cubic_tangent(start, handle1, handle2, end, t);   // which way it's heading
```

A camera rail or patrol route through a list of points:

```cpp
// pts[0..n-1]; u runs from 0 to n-1, and segment i goes from pts[i] to pts[i+1]
int i = (int)u;  float t = u - i;
int i0 = i > 0 ? i - 1 : 0, i2 = i + 1 < n ? i + 1 : n - 1, i3 = i + 2 < n ? i + 2 : n - 1;
vec3 pos = catmull_rom(pts[i0], pts[i], pts[i2], pts[i3], t);
```

### Throwing things

```cpp
vec3 v;
if (ballistic_launch(hand, target, /*speed*/ 18.0f, /*gravity*/ 9.8f, /*high arc*/ false, &v))
    spawn_arrow(hand, v);                               // lands on target

vec3 where = ballistic_position(hand, v, v3(0, -9.8f, 0), t);   // position after t seconds
```

`ballistic_launch` returns false when the target is out of range at that
speed. Every target in range can be reached by two arcs: a flat shot and a
lob.

## Geometry

Shapes and their tests (spheres, capsules, boxes, oriented boxes, planes,
triangles, rays, closest points, barycentric coordinates) are on the
[Physics page](physics.md#shape-tests-without-a-world). Frustum tests:

```cpp
frustum f = frustum_from_viewproj(view_proj);
if (frustum_test_sphere(f, centre, radius)) draw_it();
```

## 3×3 matrices

`mat3` holds rotation and scale without translation, column-major like
`mat4`. Use it for normals, oriented box axes and inertia:

```cpp
mat3 r = mat3_from_quat(q);           mat3 m = mat3_from_mat4(transform);
vec3 v = r * v3(1, 0, 0);             mat3 c = r * m;            // operators, like mat4
mat3 inv = mat3_inverse(m);           float det = mat3_determinant(m);
mat3 n = mat3_normal_matrix(model);   // transforms normals correctly under uneven scale
```

Also available: `mat3_identity`, `mat3_from_cols`, `mat3_col`, `mat3_mul`,
`mat3_mul_vec3`, `mat3_transpose`, `mat3_scale`, `mat3_rotate`,
`mat4_from_mat3`.

## Fast approximations

These are for hot loops that run millions of times a frame (particles,
noise, per-vertex work) and can give up a little accuracy for speed.
Everywhere else, use the normal functions.

| function | instead of | speed | worst error |
|---|---|---|---|
| `fast_inv_sqrt(x)` | `1 / sqrtf(x)` | 2.0× | 0.18% relative. The Quake III bit trick with one Newton step. |
| `fast_rsqrt(x)` | `1 / sqrtf(x)` | 1.2× | 3e-7 relative (CPU estimate plus one Newton step) |
| `v3norm_fast(v)`, `v2norm_fast(v)` | `v3norm`, `v2norm` | | as `fast_rsqrt` |
| `fast_sin(x)`, `fast_cos(x)` | `sinf`, `cosf` | 2.0× | 8e-7 within ±2π; 6e-5 out to ±1000 (wrap your angles) |
| `fast_atan(x)` | `atanf` | | 1.8e-6 radians |
| `fast_atan2(y, x)` | `atan2f` | 2.7× | 1.9e-6 radians |

Speeds were measured against the C library with MSVC `/O2` on x64. Errors
are the worst found over the stated ranges, and the test suite checks them.

`sqrtf`, `1/x`, `expf`, `exp2f`, `logf`, `log2f` and `powf` have no fast
version on purpose. The hardware or the C library already beats any
polynomial stand-in, so use them as they are.

## Reference

| group | functions |
|---|---|
| scalar | `clampf` `saturatef` `lerpf` `inverse_lerpf` `remapf` `smoothstepf` `approachf` `damp` `damp_angle` `angle_delta` `fractf` `wrapf` `signf` `minf` `maxf` `approx_equalf` |
| constants | `PIX_PI` `PIX_TAU` `PIX_DEG2RAD` `PIX_RAD2DEG` |
| vec2 | `v2` `v2add` `v2sub` `v2scale` `v2dot` `v2len` `v2norm` `v2norm_fast` `v2dist` `v2lerp` `v2rotate` `v2reflect` `v2project` `v2reject` `v2xz` |
| vec3 | `v3` `v3add` `v3sub` `v3scale` `v3mul` `v3neg` `v3dot` `v3cross` `v3len` `v3norm` `v3norm_fast` `v3dist` `v3lerp` `v3min` `v3max` `v3reflect` `v3project` `v3reject` `v3approx_equal` |
| vec4 | `v4` `v4lerp` |
| quat | `quat_identity` `quat_norm` `quat_mul` `quat_slerp` `quat_conjugate` `quat_rotate` `quat_from_axis_angle` `quat_from_euler` `quat_from_mat4` `quat_to_euler` `quat_to_axis_angle` |
| mat4 | `mat4_identity` `mat4_mul` `mat4_translate` `mat4_scale` `mat4_rotate` `mat4_rotate_x/y/z` `mat4_trs_y` `mat4_from_trs` `mat4_from_quat` `mat4_from_mat3` `mat4_decompose` `mat4_inverse` `mat4_transpose` `mat4_mul_point` `mat4_mul_dir` `mat4_mul_vec4` `mat4_get_translation` `mat4_max_scale` `mat4_perspective` `mat4_ortho` `mat4_lookat` |
| mat3 | `mat3_identity` `mat3_from_cols` `mat3_col` `mat3_mul` `mat3_mul_vec3` `mat3_transpose` `mat3_determinant` `mat3_inverse` `mat3_scale` `mat3_rotate` `mat3_from_mat4` `mat3_from_quat` `mat3_normal_matrix` |
| frustum | `frustum_from_viewproj` `frustum_test_sphere` `frustum_test_aabb` |
| colour | `pix_srgb_to_linear` `pix_linear_to_srgb` `pix_color_to_linear` `pix_color_from_linear` `pix_hsv` `pix_hsl` `pix_color_to_hsv` `pix_color_to_hsl` |
| curves | `hermite` `catmull_rom` `bezier_quadratic` `bezier_cubic` `bezier_cubic_tangent` (float, vec2, vec3) |
| easing | `ease_{in,out,in_out}_{quad,cubic,quart,sine,expo,circ,back,elastic,bounce}` |
| trajectories | `ballistic_position` `ballistic_launch` |
| fast | `fast_inv_sqrt` `fast_rsqrt` `fast_sin` `fast_cos` `fast_atan` `fast_atan2` |
| random | `rng_seed` `rng_u32` `rng_float` `rng_range` `rng_int` `rng_chance` `hash_u32` `hash2` `hash2_float` `rng_at` `value_noise2` `fbm2` |

Vector, matrix and quaternion operators (`+ - * /`, `+= -= *=`) can be
switched off with `#define PIX_NO_OPERATORS` if they clash with another math
library.
