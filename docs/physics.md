# Physics

pix has two parts for collisions and movement:

- **The physics world** (`core/physics.hpp`) moves bodies with gravity,
  stops them at walls, and pushes them apart from each other. Use it for
  characters, crowds, vehicles and crates in a 3D game played on the ground.
- **Shape tests** (`core/colliders.hpp`) answer "do these two shapes overlap,
  and by how much?" and "where does this ray hit?" without any simulation. Use
  them for triggers, bullets, picking, and for 2D games.

## What the physics world is, and what it isn't

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

A full 3D rigid-body solver and 2D physics are planned but not written yet.
For a 2D game today, use the [shape tests](#shape-tests-without-a-world).

## A complete example

A capsule-like player that walks with WASD, jumps with Space, and is stopped
by a wall:

```cpp
#define PIX_IMPLEMENTATION
#include "pix.h"

static phys_world world;          // static: keeps it off the stack and alive for the whole game

int main() {
    if (!pix_init("physics", 1280, 720)) return 1;

    // 1. the world
    phys_create_world(world);

    // 2. static geometry: walls and props that never move
    phys_add_static_box(world, v3(3, 0, -10), v3(4, 3, 10));    // a wall: min corner, max corner
    phys_build_statics(world);                                  // after adding all statics

    // 3. a body for the player
    phys_body b = {};
    b.shape    = PHYS_CYLINDER;
    b.radius   = 0.4f;
    b.height   = 1.8f;
    b.position = v3(0, 0, 0);       // feet
    b.inv_mass = 1.0f;              // movable
    b.gravity  = true;
    b.friction = 0.4f;
    b.group    = PHYS_LAYER_PLAYER;
    b.collides = PHYS_LAYER_ALL;
    idx player = phys_add_body(world, b);

    idx cyl = pix_gen_cylinder(0.4f, 1.8f);
    idx box = pix_gen_cube(v3(1, 3, 20));
    idx floor_mesh = pix_gen_plane(40, 40);
    idx blue = pix_make_material(PIX_BLUE), grey = pix_make_material(PIX_GRAY);

    while (pix_frame()) {
        float dt = pix_dt();
        phys_body* p = phys_get_body(world, player);

        // 4. steer: set the sideways velocity directly, leave y to gravity
        vec2 move = pix_input_axis();
        p->velocity.x = move.x * 5.0f;
        p->velocity.z = -move.y * 5.0f;
        if (p->on_ground && pix_key_pressed(KEY_SPACE)) p->velocity.y = 8.0f;

        // 5. simulate
        phys_step(world, dt);

        // 6. draw where the physics put things
        camera cam = {};
        cam.position  = p->position + v3(0, 6, 9);
        cam.direction = p->position - cam.position;
        pix_begin_3d(cam);
        pix_draw_mesh(floor_mesh, grey, mat4_identity());
        pix_draw_mesh(box, grey, mat4_translate(3.5f, 1.5f, 0));
        pix_draw_mesh(cyl, blue, mat4_translate(p->position.x, p->position.y + 0.9f, p->position.z));
        pix_end_3d();
    }
    phys_destroy_world(world);
    pix_shutdown();
}
```

The pattern is always the same:

1. Create the world.
2. Add static geometry, then build it.
3. Add bodies.
4. Each frame: set velocities from input or AI, call `phys_step`, then draw
   each body at its `position`.

## The world

```cpp
static phys_world world;
phys_create_world(world);
// ...
phys_destroy_world(world);
```

`phys_create_world` allocates the world's arrays on the heap. The struct
itself is small. Fields you might change after creating it:

| field | default | meaning |
|---|---|---|
| `gravity` | (0, -22, 0) | stronger than real gravity (9.8), because real gravity makes characters feel floaty |
| `ground_y` | 0 | the height of a flat floor |
| `ground_at` | null | a function for uneven ground; see below |
| `step_height` | 0.45 | static boxes this short or shorter are walked over, not blocked (kerbs, steps) |
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
phys_add_static_box(world, v3(-5, 0, 3), v3(5, 4, 3.5f));               // axis-aligned: min, max corners
phys_add_static_prop(world, v3(8, 0, 2), /*yaw*/ 0.6f, v2(1.0f, 0.5f), /*height*/ 1.2f);   // rotated
phys_build_statics(world);       // call once after adding them all
```

- `phys_add_static_box` takes the box's minimum and maximum corners.
- `phys_add_static_prop` takes the centre of the box's base, a rotation
  around the vertical axis (radians, the same as `mat4_rotate_y`), half its
  size in X and Z, and its height. Use it for things placed at an angle.
- `phys_build_statics` sorts the statics into a grid so that each body only
  checks the ones near it. **Call it after adding statics**, or the world
  won't see them.

To rebuild the level (a new map, a destroyed building), call
`phys_reset_statics(world)`, add the new statics, and build again. Bodies are
kept.

Up to 32,768 statics are allowed (`PHYS_MAX_STATICS`).

## Bodies

A body is anything that moves: the player, enemies, vehicles, loose crates.

```cpp
phys_body b = {};                 // always start from zero
b.shape    = PHYS_CYLINDER;
b.radius   = 0.4f;
b.height   = 1.8f;
b.position = v3(0, 0, 5);
b.inv_mass = 1.0f;
b.gravity  = true;
b.group    = PHYS_LAYER_CHARACTER;
b.collides = PHYS_LAYER_ALL;
b.user     = &my_enemy;           // optional: find your own data from a body
idx id = phys_add_body(world, b);
```

`phys_add_body` copies the body into the world and returns its id. It returns
`PIX_INVALID` when the world is full (2,048 bodies by default,
`PHYS_MAX_BODIES`). From then on, read and change the body through
`phys_get_body(world, id)`, which returns a pointer, or `nullptr` if the body
was removed.

`phys_remove_body(world, id)` removes it. Its slot is reused by the next
`phys_add_body`.

### Shapes

| shape | fields | origin | use for |
|---|---|---|---|
| `PHYS_CYLINDER` | `radius`, `height` | centre of the **bottom** | characters, creatures, barrels, trees |
| `PHYS_BOX` | `half` (half width in x, half length in y), `height`, `yaw` | centre of the **bottom** | vehicles, crates, furniture |

`position` is always at the **feet**: X and Z at the centre, Y at the bottom.
To draw a 1.8 m tall cylinder mesh (which is centred on its middle), add half
the height: `mat4_translate(p.x, p.y + 0.9f, p.z)`.

A box's `yaw` is never changed by the solver. Set it yourself, for example
when a car steers.

### Body fields

| field | meaning |
|---|---|
| `position` | feet position |
| `velocity` | metres per second. Set it to move the body. |
| `inv_mass` | 1 / mass. **0 makes the body immovable**: others bounce off it, and it doesn't move by itself. A heavier body has a smaller `inv_mass` and gets pushed less. |
| `gravity` | whether gravity pulls it down |
| `restitution` | bounciness: 0 = stops dead, 1 = perfect bounce |
| `friction` | how much bodies drag each other sideways when in contact (0..1). With 0, a character leaning on a moving car slides off its side; with friction, it gets carried along. |
| `drag` | slows sideways motion every second; a simple form of air resistance or rolling friction |
| `group`, `collides` | collision layers; see below |
| `user` | your own pointer, never touched by pix |

Set by the world each step, for you to read:

| field | meaning |
|---|---|
| `on_ground` | the body is standing on the ground. Use it to allow jumping. |
| `touched` | something pushed this body during the last step |
| `impact` | the hardest hit this step, as a change in speed (m/s). Small for a shove, large for a crash. Use it for damage, sounds and camera shake. |
| `active` | false once removed |

### Moving a character

The usual way to control a character is to **set its sideways velocity**
every frame and leave the vertical velocity to gravity:

```cpp
phys_body* p = phys_get_body(world, player);
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

### Collision layers

Each body belongs to a **group** and has a **collides** mask. Two bodies
collide only if one of them collides with the other's group:

```
(a.collides & b.group) != 0  ||  (b.collides & a.group) != 0
```

There are 16 layer bits, `PHYS_LAYER_0` to `PHYS_LAYER_7` plus higher bits.
The first four have convenience names:

| name | bit |
|---|---|
| `PHYS_LAYER_PLAYER` | `PHYS_LAYER_0` |
| `PHYS_LAYER_CHARACTER` | `PHYS_LAYER_1` |
| `PHYS_LAYER_VEHICLE` | `PHYS_LAYER_2` |
| `PHYS_LAYER_PROP` | `PHYS_LAYER_3` |
| `PHYS_LAYER_ALL` | all 16 |

The names have no built-in meaning. Name your own bits the same way:

```cpp
#define LAYER_GHOST PHYS_LAYER_4

ghost.group    = LAYER_GHOST;
ghost.collides = 0;              // walks through everything
```

A body with `group = 0` and `collides = 0` passes through all other bodies,
but walls still stop it.

## Stepping the world

Call `phys_step(world, dt)` once per frame, after setting velocities and
before drawing. Each step:

1. applies gravity and drag and moves every body,
2. clamps bodies to the ground,
3. pushes overlapping bodies apart and stops them at statics.

`pix_dt()` is capped at 0.1 seconds, so a frame hitch never launches bodies
through the level.

### A fixed time step

Physics behaves most predictably when every step is the same length. If you
need that (for replays, networking, or consistent jump heights at any frame
rate), step in fixed slices:

```cpp
static float accumulator = 0.0f;
const float step = 1.0f / 60.0f;
accumulator += pix_dt();
while (accumulator >= step) {
    phys_step(world, step);
    accumulator -= step;
}
```

### Threading

Give the world a thread pool, and the parts of the step that can safely run
in parallel will use it:

```cpp
world.jobs = &pix_get_jobs();
```

This only helps with hundreds of bodies or very dense static geometry.

## Asking the world questions

### Raycasts

Use a raycast to check line of sight, shoot bullets, or find a wall in front
of the camera:

```cpp
float dist;
if (phys_raycast(world, eye, dir, 50.0f, &dist)) {          // static geometry only
    vec3 hit = eye + v3norm(dir) * dist;
}

idx target = phys_raycast_bodies(world, gun_pos, aim_dir, 100.0f,
                                 /*mask*/ PHYS_LAYER_CHARACTER, /*ignore*/ player, &dist);
if (target != PIX_INVALID) damage(phys_get_body(world, target)->user);
```

`phys_raycast` sees only statics, and `phys_raycast_bodies` sees only bodies.
For a bullet that can hit either, ask both and keep the nearer hit:

```cpp
float wall_d = 1e30f, body_d = 1e30f;
bool  wall = phys_raycast(world, from, dir, range, &wall_d);
idx   who  = phys_raycast_bodies(world, from, dir, range, PHYS_LAYER_ALL, shooter, &body_d);
if (who != PIX_INVALID && body_d < wall_d) { /* hit a body */ }
else if (wall)                             { /* hit a wall */ }
```

The direction doesn't need to be normalised. `ignore` skips one body,
usually the shooter's own. Pass `PIX_INVALID` to skip none.

### Who is nearby

```cpp
idx found[64];
size_t n = phys_query_neighbours(world, explosion_pos, 5.0f, found, 64);
for (size_t i = 0; i < n; i++) {
    phys_body* b = phys_get_body(world, found[i]);
    vec3 away = v3norm(b->position - explosion_pos);
    b->velocity += away * 12.0f;          // knock back
}
```

`phys_query_neighbours` uses the grid built by the last `phys_step`, so it is
fast. `phys_overlap_bodies` answers the same question by checking every body;
it is correct even before the first step.

### Is there room here?

```cpp
if (!phys_point_blocked(world, spawn_pos, 0.4f, 1.8f)) spawn_enemy(spawn_pos);
```

`phys_point_blocked` reports whether a cylinder of that radius and height at
that position would overlap static geometry. Use it to place spawns, check
building placement, or test a dash destination before moving.

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

The 2D shapes are defined in the **XZ plane**, where the 2.5D world does its
collisions. Their `vec2` is simply two coordinates, so they work equally well
for a 2D game in screen coordinates: use them as `(x, y)`.

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

## Reference

### World (`core/physics.hpp`)

| function | |
|---|---|
| `phys_create_world(w)`, `phys_destroy_world(w)` | |
| `phys_add_static_box(w, min, max)` | axis-aligned wall or block |
| `phys_add_static_prop(w, position, yaw, half, height)` | rotated block; `position` is the base centre |
| `phys_build_statics(w)` | after adding statics |
| `phys_reset_statics(w)` | remove all statics, keep bodies |
| `idx phys_add_body(w, body)` | `PIX_INVALID` when full |
| `phys_body* phys_get_body(w, id)` | `nullptr` if removed |
| `phys_remove_body(w, id)` | |
| `phys_step(w, dt)` | |
| `bool phys_raycast(w, origin, dir, max_distance, &dist)` | statics only |
| `idx phys_raycast_bodies(w, origin, dir, max_distance, mask, ignore, &dist)` | bodies only |
| `size_t phys_query_neighbours(w, centre, radius, out, capacity)` | bodies near a point, using the step's grid |
| `size_t phys_overlap_bodies(w, centre, radius, out, capacity)` | the same, by checking every body |
| `bool phys_point_blocked(w, position, radius, height)` | would a cylinder here overlap statics? |

### Limits

| macro | default |
|---|---|
| `PHYS_MAX_BODIES` | 2048 |
| `PHYS_MAX_STATICS` | 32768 |

Define them before including pix to change them.
