# Physics

All of pix's physics comes with `#include "pix.h"`, or on its own with
`#include "core/physics.hpp"`, which needs no graphics at all, so a server or
tool can simulate without a window. It holds three worlds and a set of shape
tests. Pick by what your game needs:

| you want | use | page |
|---|---|---|
| things that tumble, roll, stack, hang from joints, ragdolls, in 3D | `phys3d_*` (`physics3d.hpp`) | [3D physics](physics-3d.md) |
| the same in a 2D game | `phys2d_*` (`physics2d.hpp`) | [2D physics](physics-2d.md) |
| many characters and vehicles pushing each other and walls on the ground, as cheaply as possible | `phys3d_flat_*` (`physics3d_flat.hpp`) | [Flat 3D physics](physics-3d-flat.md) |
| "do these overlap?" and "where does this ray hit?" with no world at all | `collide_*`, `ray_vs_*` (`colliders.hpp`) | [below](#shape-tests-without-a-world) |

The flat world is 2.5D: collisions only sideways, nothing tips or stacks,
but a thousand bodies cost well under a millisecond. The 3D world is a full
rigid-body simulation. Both use metres with y up, and a game can run one of
each (a crowd in the flat world, debris in the 3D one).

## The same everywhere

The prefix is the world; everything after it means the same thing in all
three:

| | 2D | 3D | flat 3D |
|---|---|---|---|
| create, destroy | `phys2d_create_world`, `_destroy_world` | `phys3d_create_world`, ... | `phys3d_flat_create_world`, ... |
| step | `phys2d_update(w, pix_dt())` fixed steps, `phys2d_step(w, dt)` one | `phys3d_update`, `phys3d_step` | `phys3d_flat_update`, `phys3d_flat_step` |
| a body with defaults | `phys2d_make_body(type, shape, pos)` | `phys3d_make_body(type, shape, pos)` | `phys3d_flat_make_cylinder` / `_make_box` |
| add, get, remove | `phys2d_add_body` / `_get_body` / `_remove_body` (`idx`, `nullptr` once removed) | `phys3d_...` | `phys3d_flat_...` |
| draw smoothly | `phys2d_lerp_position` | `phys3d_lerp_position`, `phys3d_body_transform` | `phys3d_flat_lerp_position` |
| raycast | `phys2d_raycast(w, from, dir, max, &hit, mask, ignore)` | `phys3d_raycast(...)` | `phys3d_flat_raycast(...)` |
| overlap | `phys2d_overlap(...)` returns a count, writes up to `capacity` | `phys3d_overlap(...)` | `phys3d_flat_overlap(...)` |
| debug view | `pix_draw_physics2d(w)` | `pix_draw_physics3d(w)` | `pix_draw_physics3d_flat(w)` |

Body fields carry the same names and meanings everywhere: `position`,
`velocity`, `friction`, `restitution`, `linear_damping`, `gravity_scale`,
`layer`, `mask`, `user`, and the read-only `on_ground` and `active`.

### Collision layers

Each body is on some layers (`layer`) and collides with some layers
(`mask`). In every world, two bodies collide only if **each** accepts the
other:

```
phys_layers_collide(a.layer, a.mask, b.layer, b.mask)
    == (a.mask & b.layer) && (b.mask & a.layer)
```

There are 16 bits, `PHYS_LAYER_0` to `PHYS_LAYER_15`, and `PHYS_LAYER_ALL`.
The first four have names that most games want:

| name | bit |
|---|---|
| `PHYS_LAYER_PLAYER` | `PHYS_LAYER_0` |
| `PHYS_LAYER_CHARACTER` | `PHYS_LAYER_1` |
| `PHYS_LAYER_VEHICLE` | `PHYS_LAYER_2` |
| `PHYS_LAYER_PROP` | `PHYS_LAYER_3` |

The names have no built-in meaning. Name your own bits the same way:

```cpp
#define LAYER_GHOST PHYS_LAYER_4

ghost.layer = LAYER_GHOST;
ghost.mask  = 0;                 // accepts nothing: passes through every body
```

Because both sides must agree, one side refusing is enough: a body whose
mask leaves out the player is walked through by the player, whatever the
player's mask says. Query masks (`raycast`, `overlap`) only ask about the
body found: does its `layer` match the mask?

## Shape tests without a world

`core/colliders.hpp` is pure geometry: no bodies, no time, no world. Every
overlap test fills in a **contact**, which holds the direction to push the
second shape out of the first (`normal`, unit length) and how far (`depth`).
The fix for an overlap is always the same line:

```cpp
contact3 c;
if (collide_sphere_sphere(a, b, &c))
    b.centre += c.normal * c.depth;     // push b out of a
```

### 3D shapes

| shape | make it with |
|---|---|
| `sphere` | `sphere_make(centre, radius)` |
| `capsule` | `capsule_make(a, b, radius)`, or `capsule_upright(base, height, radius)` for a standing character |
| `aabb` (axis-aligned box) | `aabb_make(min, max)`, `aabb_from_centre(centre, half_size)` |
| `plane` | `plane_make(normal, distance)`, `plane_through(normal, point)` |
| `obb` (a box at any angle) | `obb_make(centre, half_size, rotation_quat)`, or `obb_from_aabb(model_bounds, transform)` to carry a mesh's bounds through its transform |
| triangle | just three `vec3` corners. The front is the side they run counter-clockwise from. |

Overlap tests (each returns `bool` and fills a `contact3*`). The contact
pointer must not be null. Pass a local `contact3` even when you only need yes
or no:

`collide_sphere_sphere`, `collide_sphere_plane`, `collide_sphere_aabb`,
`collide_sphere_capsule`, `collide_sphere_obb`, `collide_sphere_triangle`,
`collide_capsule_capsule`, `collide_capsule_plane`, `collide_capsule_aabb`,
`collide_obb_obb`, and `aabb_overlap(a, b)` (yes or no only).

`obb_bounds(box)` gives the axis-aligned box around an `obb`, for quick
rejection before the exact test. `obb_to_local(box, p)` puts a point into the
box's own frame.

Ray tests (direction must be **normalised**, distance written to `*out_t`):

```cpp
float t;
ray_vs_sphere(origin, dir, s, max_distance, &t);
ray_vs_capsule(origin, dir, c, max_distance, &t);
ray_vs_plane(origin, dir, p, max_distance, /*two_sided*/ false, &t);
vec3 inv = v3(1 / dir.x, 1 / dir.y, 1 / dir.z);     // ray_vs_aabb takes 1/direction
ray_vs_aabb(origin, inv, box, max_distance, &t);
ray_vs_obb(origin, dir, rotated_box, max_distance, &t);
vec3 bary;
ray_vs_triangle(origin, dir, a, b, c, max_distance, /*two_sided*/ false, &t, &bary);
```

A one-sided `ray_vs_plane` or `ray_vs_triangle` only hits the front, the
side the normal points to. A ray coming from below a floor passes through it.

Closest points: `closest_point_segment(a, b, p)`, `closest_point_aabb(box, p)`,
`closest_point_obb(box, p)`, `closest_point_triangle(p, a, b, c)`,
`closest_point_plane(plane, p)`, and `plane_distance_to(plane, p)` (signed:
positive in front).

### Triangles and barycentric coordinates

Barycentric coordinates say where a point sits inside a triangle, as three
weights that add up to 1, one per corner. Use them to read anything stored
per vertex at an exact point: a UV under the mouse, a terrain height, a
blended normal.

```cpp
float t; vec3 w;
if (ray_vs_triangle(cam_pos, ray, a, b, c, 1000.0f, false, &t, &w)) {
    vec2 uv = v2(a_uv.x * w.x + b_uv.x * w.y + c_uv.x * w.z,
                 a_uv.y * w.x + b_uv.y * w.y + c_uv.y * w.z);    // the texel under the mouse
}
vec3 w2 = barycentric(p, a, b, c);         // for a point you already have; all three in 0..1 = inside
vec3 n  = triangle_normal(a, b, c);
```

`collide_sphere_triangle` lets a ball or character roll over arbitrary mesh
geometry. Test it against the triangles near the sphere and push the sphere
out along each contact.

A trigger zone, for example a door that opens when the player is near:

```cpp
aabb door_zone = aabb_make(v3(9, 0, -1), v3(11, 2.5f, 1));
capsule pc = capsule_upright(player_pos, 1.8f, 0.4f);
contact3 c;
if (collide_capsule_aabb(pc, door_zone, &c)) open_door();
```

### 2D shapes

The 2D shapes are defined in the **XZ plane**, where the flat 3D world does
its collisions. Their `vec2` is simply two coordinates, so they work equally
well for a 2D game in screen coordinates: use them as `(x, y)`.

| shape | fields |
|---|---|
| `circle2` | `centre`, `radius` |
| `rect2` | `min`, `max` (axis-aligned) |
| `obb2` | `centre`, `half`, `yaw` (a rotated rectangle) |

Tests (each fills a `contact2*` with `normal` and `depth`; the pointer must
not be null):
`collide_circle_circle`, `collide_circle_rect`, `collide_circle_obb`,
`collide_obb_obb`, `collide_obb_rect`. Closest points:
`closest_point_rect(r, p)`, `closest_point_obb(b, p)`.

A 2D player pushed out of walls, with no physics world at all:

```cpp
rect2 walls[] = { { v2(0, 400), v2(800, 440) }, { v2(300, 250), v2(340, 400) } };

player_pos += velocity * dt;
circle2 me = { player_pos, 16 };
for (const rect2& w : walls) {
    contact2 c;
    if (collide_circle_rect(me, w, &c)) {
        me.centre -= c.normal * c.depth;     // normal points from me towards the wall: move back
        player_pos = me.centre;
        // stop moving into the wall, keep sliding along it
        float into = v2dot(velocity, c.normal);
        if (into > 0) velocity -= c.normal * into;
    }
}
```

## Reference: shape tests (`core/colliders.hpp`)

| 3D | |
|---|---|
| shapes | `sphere`, `capsule`, `aabb`, `obb`, `plane`, triangles as three `vec3` |
| overlaps (fill a `contact3`) | `collide_sphere_sphere`, `_sphere_plane`, `_sphere_aabb`, `_sphere_capsule`, `_sphere_obb`, `_sphere_triangle`, `_capsule_capsule`, `_capsule_plane`, `_capsule_aabb`, `_obb_obb`; `aabb_overlap` |
| rays (normalised direction) | `ray_vs_sphere`, `ray_vs_capsule`, `ray_vs_plane`, `ray_vs_aabb` (takes 1/direction), `ray_vs_obb`, `ray_vs_triangle` |
| closest points | `closest_point_segment`, `_aabb`, `_obb`, `_triangle`, `_plane`, `closest_points_segments`; `plane_distance_to` |

| 2D | |
|---|---|
| shapes | `circle2`, `rect2`, `obb2` |
| overlaps (fill a `contact2`) | `collide_circle_circle`, `_circle_rect`, `_circle_obb`, `_obb_obb`, `_obb_rect` |
| closest points | `closest_point_rect`, `closest_point_obb` |
