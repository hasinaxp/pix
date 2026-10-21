# Flat 3D physics

`core/physics3d_flat.hpp` (`phys3d_flat_*`) is the cheapest pix physics world.
It moves bodies with gravity, stops them at walls, and pushes them apart
from each other. Use it for characters, crowds, vehicles and crates in a 3D
game played on the ground.

It comes with `#include "pix.h"`, or with `#include "core/physics.hpp"` on
its own. See [Physics](physics.md) for how it relates to the other worlds.

## What it is, and what it isn't

The world is deliberately **2.5D**. Bodies move freely in 3D, but collisions
are resolved sideways, in the ground plane (X and Z). Height is only gravity
and a floor. This fits games played on the ground: action games,
third-person adventures, top-down shooters, racing, strategy. It is fast
enough to run a thousand bodies in well under a millisecond, and objects
never pass through walls at speed.

What it does **not** do:

- **No tumbling or rotation.** Bodies don't spin, tip over or roll. Boxes
  keep the heading (`yaw`) you give them.
- **No stacking.** You can't stand on top of a box or another body. Floors
  and ledges come from the ground height (see [The ground](#the-ground)).
- **Only two body shapes:** upright cylinders and upright boxes.

For things that tumble, stack, roll or hang from joints, use
[3D physics](physics-3d.md). Both use metres with y up, so a game can run one
world for its crowd and the other for its crates.

## A complete example

A player that walks with WASD, jumps with Space, and is stopped by a wall:

```cpp
#define PIX_IMPLEMENTATION
#include "pix.h"

static phys3d_flat_world world;    // static: keeps it off the stack and alive for the whole game

int main() {
    if (!pix_init("physics", 1280, 720)) return 1;

    // 1. the world
    phys3d_flat_create_world(world);

    // 2. static geometry: walls and props that never move
    phys3d_flat_add_static_box(world, v3(3, 0, -10), v3(4, 3, 10));   // a wall: min corner, max corner
    phys3d_flat_build_statics(world);                                 // after adding all statics

    // 3. a body for the player: feet at the origin, 0.4 m radius, 1.8 m tall
    idx player = phys3d_flat_add_body(world, phys3d_flat_make_cylinder(v3(0, 0, 0), 0.4f, 1.8f));

    idx cyl = pix_gen_cylinder(0.4f, 1.8f);
    idx box = pix_gen_cube(v3(1, 3, 20));
    idx floor_mesh = pix_gen_plane(40, 40);
    idx blue = pix_make_material(PIX_BLUE), grey = pix_make_material(PIX_GRAY);

    while (pix_frame()) {
        phys3d_flat_body* p = phys3d_flat_get_body(world, player);

        // 4. steer: set the sideways velocity directly, leave y to gravity
        vec2 move = pix_input_axis();
        p->velocity.x = move.x * 5.0f;
        p->velocity.z = -move.y * 5.0f;
        if (p->on_ground && pix_key_pressed(KEY_SPACE)) p->velocity.y = 8.0f;

        // 5. simulate, in fixed steps
        phys3d_flat_update(world, pix_dt());

        // 6. draw where the physics put things (smoothed between steps)
        vec3 at = phys3d_flat_lerp_position(world, *p);
        camera cam = {};
        cam.position  = at + v3(0, 6, 9);
        cam.direction = at - cam.position;
        pix_begin_3d(cam);
        pix_draw_mesh(floor_mesh, grey, mat4_identity());
        pix_draw_mesh(box, grey, mat4_translate(3.5f, 1.5f, 0));
        pix_draw_mesh(cyl, blue, mat4_translate(at.x, at.y + 0.9f, at.z));
        pix_end_3d();
    }
    phys3d_flat_destroy_world(world);
    pix_shutdown();
}
```

The pattern is always the same:

1. Create the world.
2. Add static geometry, then build it.
3. Add bodies.
4. Each frame: set velocities from input or AI, call `phys3d_flat_update`,
   then draw each body.

## The world

`phys3d_flat_create_world` allocates the world's arrays on the heap; the
struct itself is small. Fields you might change after creating it:

| field | default | meaning |
|---|---|---|
| `gravity` | (0, -22, 0) | stronger than real gravity (9.8), because real gravity makes characters feel floaty |
| `ground_y` | 0 | the height of a flat floor |
| `ground_at` | null | a function for uneven ground; see below |
| `step_height` | 0.45 | static boxes this short or shorter are walked over, not blocked (kerbs, steps) |
| `fixed_dt`, `max_steps` | 1/60, 4 | how `phys3d_flat_update` steps |
| `jobs` | null | a thread pool to run the step on several cores; see [Threading](#threading) |

### The ground

Bodies fall until they reach the ground and stop there. On flat ground that
is `world.ground_y`. For hills, terrain, ledges or a bridge, give the world a
function that returns the ground height at any point:

```cpp
float terrain_height(void* user, float x, float z) {
    return sinf(x * 0.1f) * 2.0f + cosf(z * 0.13f) * 1.5f;   // your terrain here
}

world.ground_at = terrain_height;
world.ground_user = nullptr;      // passed back to you as `user`, e.g. a pointer to your terrain
```

This function is called from several threads at once when the world has a
thread pool, so it must only **read** data. It must not allocate, cache
results, or change anything.

Because floors come only from the ground height, a body cannot land on top of
a static box. If you want a platform the player can stand on, include it in
your `ground_at` function.

## Static geometry

Statics are the parts of the level that never move: walls, buildings, rocks,
fences.

```cpp
phys3d_flat_add_static_box(world, v3(-5, 0, 3), v3(5, 4, 3.5f));                       // min, max corners
phys3d_flat_add_static_prop(world, v3(8, 0, 2), /*yaw*/ 0.6f, v2(1.0f, 0.5f), /*height*/ 1.2f);   // rotated
phys3d_flat_build_statics(world);       // once, after adding them all
```

- `phys3d_flat_add_static_box` takes the box's minimum and maximum corners.
- `phys3d_flat_add_static_prop` takes the centre of the box's base, a
  rotation around the vertical axis (radians, the same as `mat4_rotate_y`),
  half its size in X and Z, and its height. Use it for things placed at an
  angle.
- `phys3d_flat_build_statics` sorts the statics into a grid so that each body
  only checks the ones near it. **Call it after adding statics**, or the
  world won't see them.

To rebuild the level (a new map, a destroyed building), call
`phys3d_flat_reset_statics(world)`, add the new statics, and build again.
Bodies are kept.

## Bodies

A body is anything that moves: the player, enemies, vehicles, loose crates.
Start from one of the two makers, which fill in sensible defaults, then
change what you need:

```cpp
phys3d_flat_body b = phys3d_flat_make_cylinder(v3(0, 0, 5), 0.4f, 1.8f);   // feet, radius, height
b.layer = PHYS_LAYER_CHARACTER;
b.user  = &my_enemy;               // optional: find your own data from a body
idx id = phys3d_flat_add_body(world, b);

phys3d_flat_body car = phys3d_flat_make_box(v3(4, 0, 0), v2(1.0f, 2.2f), 1.4f, /*yaw*/ 0.3f);   // base, half, height
car.inv_mass = 1.0f / 1200.0f;     // heavier: pushed less
```

`phys3d_flat_add_body` copies the body into the world and returns its id, or
`PIX_INVALID` when the world is full (2,048 bodies by default). From then on,
read and change it through `phys3d_flat_get_body(world, id)`, which returns a
pointer, or `nullptr` once the body is removed with
`phys3d_flat_remove_body(world, id)`.

### Shapes

| shape | fields | origin | use for |
|---|---|---|---|
| `PHYS3D_FLAT_CYLINDER` | `radius`, `height` | centre of the **bottom** | characters, creatures, barrels, trees |
| `PHYS3D_FLAT_BOX` | `half` (half width in x, half length in y), `height`, `yaw` | centre of the **bottom** | vehicles, crates, furniture |

`position` is always at the **feet**: X and Z at the centre, Y at the bottom.
To draw a 1.8 m tall cylinder mesh (which is centred on its middle), add half
the height: `mat4_translate(p.x, p.y + 0.9f, p.z)`.

A box's `yaw` is never changed by the solver. Set it yourself, for example
when a car steers.

### Body fields

| field | default | meaning |
|---|---|---|
| `position`, `velocity` | | feet position; metres per second. Set the velocity to move the body |
| `inv_mass` | 1 | 1 / mass. **0 makes the body immovable**: others bounce off it, and it doesn't move by itself. A heavier body has a smaller `inv_mass` and gets pushed less |
| `gravity_scale` | 1 | 0 floats |
| `restitution` | 0 | bounciness: 0 = stops dead, 1 = perfect bounce |
| `friction` | 0.4 | how much bodies drag each other sideways when in contact (0..1). With 0, a character leaning on a moving car slides off its side; with friction, it gets carried along |
| `linear_damping` | 0 | slows sideways motion every second: air resistance, rolling friction |
| `layer`, `mask` | layer 0, all | collision layers, see [Physics](physics.md#collision-layers) |
| `user` | null | your own pointer, never touched by pix |

Set by the world each step, for you to read:

| field | meaning |
|---|---|
| `on_ground` | the body is standing on the ground. Use it to allow jumping |
| `touched` | something pushed this body during the last step |
| `impact` | the hardest hit this step, as a change in speed (m/s). Small for a shove, large for a crash. Use it for damage, sounds and camera shake |
| `active` | false once removed |

### Moving a character

The usual way to control a character is to **set its sideways velocity**
every frame and leave the vertical velocity to gravity:

```cpp
phys3d_flat_body* p = phys3d_flat_get_body(world, player);
vec3 wish = v3(move.x, 0, -move.y) * run_speed;     // where the player wants to go
p->velocity.x = wish.x;
p->velocity.z = wish.z;
if (p->on_ground && pix_key_pressed(KEY_SPACE)) p->velocity.y = jump_speed;
```

The world then stops the character at walls, slides it along them, and
pushes other characters out of the way.

For a softer, accelerating feel, blend towards the wished velocity instead of
setting it outright:

```cpp
float k = 1.0f - expf(-10.0f * dt);     // larger 10 = snappier
p->velocity.x += (wish.x - p->velocity.x) * k;
p->velocity.z += (wish.z - p->velocity.z) * k;
```

## Stepping the world

Call `phys3d_flat_update(world, pix_dt())` once per frame, after setting
velocities and before drawing. It runs fixed steps of `world.fixed_dt` (1/60
s), as many as the frame took, at most `world.max_steps`, and returns how many
ran. Fixed steps keep jump heights and replays the same at any frame rate.
Draw at `phys3d_flat_lerp_position(world, body)` for motion that stays
smooth between steps.

`phys3d_flat_step(world, dt)` runs exactly one step of `dt`, for code with
its own clock. Each step:

1. applies gravity and damping and moves every body,
2. clamps bodies to the ground,
3. pushes overlapping bodies apart and stops them at statics.

### Threading

Give the world a thread pool, and the parts of the step that can safely run
in parallel will use it:

```cpp
world.jobs = &pix_get_jobs();
```

This only helps with hundreds of bodies or very dense static geometry.

## Asking the world questions

### Raycasts

One call finds the nearest thing a ray meets, wall or body:

```cpp
phys3d_flat_ray_hit hit;
if (phys3d_flat_raycast(world, gun_pos, aim_dir, 100.0f, &hit, PHYS_LAYER_CHARACTER, /*ignore*/ player)) {
    if (hit.body != PIX_INVALID) damage(phys3d_flat_get_body(world, hit.body)->user);
    else spawn_spark(hit.point, hit.normal);      // a wall
}
```

The direction doesn't need to be normalised. `hit.body` is `PIX_INVALID`
when the ray hit static geometry. The optional `mask` limits which bodies
count (walls always do), and `ignore` skips one body, usually the shooter.

### Who is nearby

```cpp
idx found[64];
size_t n = phys3d_flat_query_neighbours(world, explosion_pos, 5.0f, found, 64);
for (size_t i = 0; i < n && i < 64; i++) {
    phys3d_flat_body* b = phys3d_flat_get_body(world, found[i]);
    vec3 away = v3norm(b->position - explosion_pos);
    b->velocity += away * 12.0f;          // knock back
}
```

`phys3d_flat_query_neighbours` uses the grid built by the last step, so it
is fast enough for a crowd asking every frame. It doesn't see bodies added or
moved by hand since that step. `phys3d_flat_overlap` answers the same
question by checking every body, so it is always exact. Both return how many
bodies they found, which can be more than `capacity`; only the first
`capacity` ids are written. Both take an optional layer mask.

### Is there room here?

```cpp
if (!phys3d_flat_point_blocked(world, spawn_pos, 0.4f, 1.8f)) spawn_enemy(spawn_pos);
```

`phys3d_flat_point_blocked` reports whether a cylinder of that radius and
height at that position would overlap static geometry. Use it to place
spawns, check building placement, or test a dash destination before moving.

## Seeing what the physics does

`pix_draw_physics3d_flat(world)`, between `pix_begin_3d` and `pix_end_3d`,
draws every static (rotated props at their true angle) and every body as
lines: green moving, yellow on the ground, blue immovable.

## Reference

| function | |
|---|---|
| `phys3d_flat_create_world(w)`, `phys3d_flat_destroy_world(w)` | |
| `int phys3d_flat_update(w, frame_dt)`, `phys3d_flat_step(w, dt)` | fixed steps; one step |
| `phys3d_flat_add_static_box(w, min, max)` | axis-aligned wall or block |
| `phys3d_flat_add_static_prop(w, position, yaw, half, height)` | rotated block; `position` is the base centre |
| `phys3d_flat_build_statics(w)`, `phys3d_flat_reset_statics(w)` | after adding statics; remove them all |
| `phys3d_flat_make_cylinder(feet, radius, height)`, `phys3d_flat_make_box(base, half, height, yaw)` | bodies with defaults |
| `idx phys3d_flat_add_body(w, body)` | `PIX_INVALID` when full |
| `phys3d_flat_get_body(w, id)`, `phys3d_flat_remove_body(w, id)` | |
| `phys3d_flat_lerp_position(w, body)` | where to draw it |
| `bool phys3d_flat_raycast(w, origin, dir, max, &hit, mask, ignore)` | walls and bodies |
| `size_t phys3d_flat_overlap(w, centre, radius, out, capacity, mask)` | exact |
| `size_t phys3d_flat_query_neighbours(w, centre, radius, out, capacity, mask)` | fast, from the last step's grid |
| `bool phys3d_flat_point_blocked(w, position, radius, height)` | |
| `pix_draw_physics3d_flat(w)`, `phys3d_flat_debug_draw(renderer, w)` | debug view |

| macro | default |
|---|---|
| `PHYS3D_FLAT_MAX_BODIES` | 2048 |
| `PHYS3D_FLAT_MAX_STATICS` | 32768 |

Define them before including pix to change them.
