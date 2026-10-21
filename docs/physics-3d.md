# 3D physics

`core/physics3d.hpp` is a 3D rigid-body world. Boxes, balls and capsules
fall, tumble, roll, stack and come to rest on floors, ramps and terrain.
Joints build doors, chains, ragdolls, seesaws and lifts.

Units are **metres**, **y is up**, and gravity is 9.81 m/s² downward. A body's
size, speed and mass are real-world numbers: a crate is about 1 m, a person
1.8 m tall, and a ball thrown hard moves at about 20 m/s.

> pix also has a simpler **flat 3D world** ([Flat 3D physics](physics-3d-flat.md)):
> upright cylinders and boxes pushing each other around on the ground. It's
> the fastest choice for crowds and characters. Use this world when things
> need to tumble, stack or hang from joints. [Physics](physics.md) compares
> the worlds.

## A complete example

A floor, a crate that falls onto it, and a ball thrown with the mouse:

```cpp
#define PIX_IMPLEMENTATION
#include "pix.h"

int main() {
    if (!pix_init("3d physics", 1280, 720)) return 1;
    idx cube = pix_gen_cube(v3(1, 1, 1));
    idx ball_mesh = pix_gen_sphere(0.25f);
    idx red = pix_make_material(PIX_RED);

    phys3d_world world;
    phys3d_create_world(world);
    phys3d_add(world, PHYS3D_STATIC, phys3d_plane(v3(0, 1, 0)), v3(0, 0, 0));          // the ground
    idx crate = phys3d_add(world, PHYS3D_DYNAMIC, phys3d_box(v3(1, 1, 1)), v3(0, 4, 0));

    camera cam = {};
    cam.position = v3(6, 4, 8);
    cam.direction = v3(-6, -3, -8);

    while (pix_frame()) {
        if (pix_mouse_pressed(PIX_MOUSE_LEFT)) {
            vec3 dir = pix_screen_to_ray(pix_get_renderer(), pix_mouse_position().x, pix_mouse_position().y);
            phys3d_body b = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_sphere(0.25f), cam.position);
            b.velocity = dir * 20.0f;
            phys3d_add_body(world, b);
        }

        phys3d_update(world, pix_dt());           // move everything

        pix_begin_3d(cam);
        pix_draw_mesh(cube, red, phys3d_body_transform(world, *phys3d_get_body(world, crate)));
        for (size_t i = 0; i < world.body_count; i++) {
            const phys3d_body& b = world.bodies[i];
            if (b.active && b.shape.type == PHYS3D_SPHERE)
                pix_draw_mesh(ball_mesh, red, phys3d_body_transform(world, b));
        }
        pix_draw_physics3d(world);                // outlines of every body
        pix_end_3d();
    }
    phys3d_destroy_world(world);
    pix_shutdown();
}
```

`phys3d_body_transform` gives the matrix to draw a body with: its position
and rotation, smoothed between physics steps.

`examples/physics_3d.cpp` is the big playground: stacks, a wrecking ball on a
chain, a ragdoll, a seesaw, a lift, a door, a hill made from a heightfield, a
trigger zone, a player that walks and jumps, dragging bodies with the mouse
and throwing balls. Build it with `build physics_3d run`.

## Shapes

| constructor | what it is | body types |
|---|---|---|
| `phys3d_sphere(radius)` | a ball | any |
| `phys3d_box(size)` | a box; `size` is the full width, height and depth | any |
| `phys3d_capsule(radius, height)` | a pill standing along the body's y axis, `height` from top to bottom | any |
| `phys3d_plane(normal)` | an endless flat floor (or wall) through the body's position | static, kinematic |
| `phys3d_mesh_shape(mesh)` | triangles: a level, a terrain | static, kinematic |

A body's `position` is its centre (for a plane, any point on it), and its
`rotation` is a quaternion (`quat_from_axis_angle(axis, radians)`).

### Meshes and terrain

Collision triangles come from a `phys3d_mesh`. You build it once and keep
it alive as long as bodies use it:

```cpp
phys3d_mesh level;
phys3d_create_mesh(level, vertices, vertex_count, indices, index_count);   // vec3s and uint32 triangles
phys3d_add(world, PHYS3D_STATIC, phys3d_mesh_shape(level), v3(0, 0, 0));
// ...
phys3d_destroy_mesh(level);
```

For terrain, a grid of heights is enough:

```cpp
// heights[row * columns + column], samples `spacing` metres apart,
// starting at (0, 0, 0) and running along +x (columns) and +z (rows)
phys3d_create_heightfield(terrain, heights, columns, rows, 1.0f);
```

The mesh keeps its own copy of the data, so you can free yours afterwards.

Mesh triangles are **one-sided**. A triangle's front is the side its corners
run counter-clockwise from. Bodies collide with the front and pass through
from behind, so something that ends up under the terrain falls out instead
of getting stuck. Rays from behind pass through too.

## Body types

| type | moves? | example |
|---|---|---|
| `PHYS3D_STATIC` | never on its own | the ground, walls, ramps |
| `PHYS3D_DYNAMIC` | gravity and collisions move it | crates, balls, the player, ragdoll parts |
| `PHYS3D_KINEMATIC` | only at the velocity you set | moving platforms, doors driven by a script |

## Bodies

`phys3d_add(world, type, shape, position)` returns an `idx` handle, and
`phys3d_get_body(world, id)` gives you the body (`nullptr` once removed with
`phys3d_remove_body`). To set more than type, shape and position, start from
`phys3d_make_body`:

```cpp
phys3d_body ball = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_sphere(0.3f), v3(0, 2, 0));
ball.restitution = 0.8f;          // bouncy
ball.velocity = v3(3, 0, 0);
idx id = phys3d_add_body(world, ball);
```

### Body fields

| field | default | |
|---|---|---|
| `position`, `rotation` | | centre and orientation |
| `velocity`, `angular_velocity` | 0 | metres per second; radians per second about each axis |
| `density` | 1000 | kg per cubic metre (water). Mass comes from density and size |
| `friction` | 0.6 | 0 ice .. 1 rubber |
| `restitution` | 0 | bounciness: 0 dead stop, 1 perfect bounce |
| `rolling_friction` | 0.01 | slows rolling and spinning. 0 lets a ball roll forever |
| `linear_damping`, `angular_damping` | 0 | air drag |
| `gravity_scale` | 1 | 0 floats |
| `fixed_rotation` | false | never rotates (characters) |
| `trigger` | false | see [Triggers](#triggers) |
| `layer`, `mask` | layer 0, all | see [Collision layers](#collision-layers) |
| `never_sleep` | false | see [Sleeping](#sleeping) |
| `user` | null | your own pointer |

Read-only:

| field | |
|---|---|
| `on_ground`, `ground_normal` | touching something below it this step, and that surface's up direction |
| `sleeping` | at rest and not being simulated |

After changing `shape`, `density`, `type` or `fixed_rotation` on a body in
the world, call `phys3d_update_mass(world, id)`.

### Moving a body

- **Set its velocity.** This is the usual way, for players and thrown things.
- **Push it:** `phys3d_apply_impulse_centre(b, impulse)` is an instant kick,
  `phys3d_apply_force_centre(b, force)` a steady push over the next step.
  `phys3d_apply_force(b, force, point)` and `phys3d_apply_impulse(b, impulse,
  point)` push at a point, which also spins the body. `phys3d_apply_torque`
  and `phys3d_apply_angular_impulse` only spin it.
- **Teleport it:** `phys3d_teleport(world, id, position, rotation)`.

### A walking character

```cpp
phys3d_body p = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_capsule(0.35f, 1.8f), v3(0, 0.9f, 0));
p.fixed_rotation = true;    // stays upright
p.friction = 0;             // we set its speed ourselves
idx player = phys3d_add_body(world, p);

// each frame, before phys3d_update
phys3d_body* pl = phys3d_get_body(world, player);
pl->velocity.x = move.x * 5;              // keep y: that's gravity and jumping
pl->velocity.z = move.z * 5;
if (pix_key_pressed(KEY_SPACE) && pl->on_ground) pl->velocity.y = 6;
```

## Stepping the world

Call `phys3d_update(world, pix_dt())` once per frame. It runs fixed steps of
`world.fixed_dt` (1/60 s), as many as the frame took (at most
`world.max_steps`), and returns how many ran. Draw bodies with
`phys3d_body_transform` (or `phys3d_lerp_position` / `phys3d_lerp_rotation`)
for motion that stays smooth between steps. `phys3d_step(world, dt)` runs
exactly one step, for code with its own clock.

## Contact events

After `phys3d_update`, `world.events` lists what started and stopped touching:

```cpp
for (size_t i = 0; i < world.event_count; i++) {
    const phys3d_event& e = world.events[i];
    if (e.type == PHYS3D_CONTACT_BEGIN && e.speed > 3) play_sound(thud, e.point);
}
```

| type | `a`, `b` | also |
|---|---|---|
| `PHYS3D_CONTACT_BEGIN` | the two bodies | `point`, `normal` (from a to b), `speed` of the impact |
| `PHYS3D_CONTACT_END` | the two bodies | |
| `PHYS3D_TRIGGER_ENTER`, `PHYS3D_TRIGGER_EXIT` | `a` is the trigger, `b` the visitor | |
| `PHYS3D_JOINT_BREAK` | the joint's two bodies | `joint`, `speed`: the force that broke it |

A body resting across many terrain triangles is still one contact: one begin
when it lands, one end when it leaves.

## Triggers

A trigger reports bodies entering and leaving it, and never pushes:

```cpp
phys3d_body zone = phys3d_make_body(PHYS3D_STATIC, phys3d_box(v3(4, 3, 4)), v3(10, 1.5f, 0));
zone.trigger = true;
idx zone_id = phys3d_add_body(world, zone);
```

## Collision layers

The same as in 2D: each body is on `layer` and collides with `mask`, and a
pair collides only if each accepts the other,
`(a.mask & b.layer) && (b.mask & a.layer)`. Name the `PHYS_LAYER_*` bits for
your game:

```cpp
#define LAYER_PLAYER PHYS_LAYER_0
#define LAYER_DEBRIS PHYS_LAYER_1
debris.layer = LAYER_DEBRIS;
debris.mask  = PHYS_LAYER_ALL & ~LAYER_PLAYER;      // debris never trips the player up
```

## Joints

Joints connect two bodies, or a body to the world (`PIX_INVALID`). Anchors
and axes are given in world space, at the bodies' current poses.

| function | joint | used for |
|---|---|---|
| `phys3d_add_ball(w, a, b, anchor, twist_axis)` | a shared pivot, free to turn any way | chains, ragdoll shoulders and hips, hanging lamps |
| `phys3d_add_hinge(w, a, b, anchor, axis)` | turns about one axis | doors, wheels, elbows, knees, levers |
| `phys3d_add_slider(w, a, b, anchor, axis)` | slides along one axis, never turns | pistons, drawers, lifts |
| `phys3d_add_fixed(w, a, b, anchor)` | glues two bodies together | breakable things, attachments |
| `phys3d_add_distance(w, a, b, anchor_a, anchor_b)` | keeps two points apart | rods, springs, ropes |
| `phys3d_add_mouse(w, body, grab)` | pulls the point under `grab` toward `target` | picking things up |

`phys3d_get_joint(w, id)` gives the joint to edit, and
`phys3d_remove_joint(w, id)` removes it. Jointed bodies don't collide with each
other unless `collide_connected` is set.

### Limits, motors and springs

All off until turned on:

| field | ball | hinge | slider | distance |
|---|---|---|---|---|
| `swing_limit` | how far the twist axis may swing, radians (a cone) | | | |
| `enable_limit`, `lower`, `upper` | twist about the twist axis | angle | travel | length |
| `enable_motor`, `motor_speed`, `max_motor_force` | | speed (rad/s), torque (N·m) | speed (m/s), force (N) | |
| `enable_spring`, `hertz`, `damping_ratio` | | | | spring at `length` |

`phys3d_hinge_angle(w, joint)` reads a hinge's current angle. Springs, ropes,
the soft fixed joint (`hertz` > 0), the mouse joint and `break_force` all work
the same way as in 2D ([2D physics: joints](physics-2d.md#joints)).

A ragdoll is capsules joined by ball joints with swing limits (shoulders,
hips, neck) and hinges with limits (elbows, knees). `add_ragdoll` in
`examples/physics_3d.cpp` builds one in 25 lines.

```cpp
phys3d_joint* knee = phys3d_get_joint(w, phys3d_add_hinge(w, thigh, shin, knee_point, v3(1, 0, 0)));
knee->enable_limit = true;  knee->lower = 0;  knee->upper = 2.4f;     // bends one way only
```

## Sleeping

Bodies still for half a second fall asleep and cost nothing until something
wakes them: something moving touches them, something they rest on is
removed, you set their velocity, move or push them, or call
`phys3d_wake(world, id)`. Touching and jointed bodies sleep and wake as one
group. `never_sleep` keeps one body awake; `world.allow_sleep = false` turns
sleeping off.

A pile of a thousand mixed bodies is fully asleep about 13 seconds after
being dropped, and a sleeping world costs almost nothing per step.

## Asking the world questions

```cpp
phys3d_ray_hit hit;
if (phys3d_raycast(world, eye, direction, 100, &hit)) {
    // hit.body, hit.point, hit.normal, hit.distance, hit.triangle (on meshes)
}
// a thick ray: would a 0.3 m sphere moving this way hit something?
phys3d_shape_cast(world, phys3d_sphere(0.3f), from, quat_identity(), direction, 10, &hit);
// everything inside a sphere (or any shape)
idx found[64];
size_t n = phys3d_overlap_sphere(world, centre, 5.0f, found, 64);
```

Raycasts ignore triggers and bodies the ray starts inside, and take an
optional layer `mask` and one body to `ignore` (the player, usually). Shape
casts don't rotate the shape on the way, and report distance 0 if it already
overlaps something. Overlap queries include triggers.

## Seeing what the physics does

`pix_draw_physics3d(world, flags)` draws everything as lines, inside
`pix_begin_3d` / `pix_end_3d`:

| flag | draws |
|---|---|
| `PHYS3D_DRAW_SHAPES` | wireframes: grey static, green moving, yellow on the ground, dark green asleep, blue kinematic, cyan trigger |
| `PHYS3D_DRAW_CONTACTS` | contact points with their normal and friction directions; red touching, yellow near misses |
| `PHYS3D_DRAW_JOINTS` | joint anchors, and hinge and slider axes |
| `PHYS3D_DRAW_AABBS` | broadphase bounds |
| `PHYS3D_DRAW_MESHES` | every mesh triangle: can be a lot of lines |

The default is shapes, contacts and joints.

## Tuning

| field | default | |
|---|---|---|
| `gravity` | (0, -9.81, 0) | |
| `fixed_dt`, `max_steps` | 1/60, 4 | |
| `substeps` | 4 | more is stiffer: tall stacks, long chains, heavy on light |
| `iterations`, `relax_iterations` | 1, 1 | solver passes per substep |
| `cell_size` | 2 | broadphase cell, metres. Around the size of a typical body |
| `slop` | 0.005 | overlap allowed to stay, metres |
| `speculative` | 0.02 | how far ahead of touching contacts start, metres |
| `bounce_threshold` | 1 | slower impacts don't bounce, m/s |
| `correction` | 0.2 | fraction of an overlap pushed out per step |
| `joint_hertz`, `joint_damping` | 60, 2 | stiffness of "rigid" joints |
| `sleep_linear`, `sleep_angular`, `time_to_sleep` | 0.05 m/s, 0.05 rad/s, 0.5 s | |

### How it works

The 2D world's design, lifted to 3D. Each part names the method it uses, so
the literature applies:

- **Broadphase:** a hashed uniform grid, with statics in their own grid that's
  rebuilt only when one changes. Bodies too big for the grid (planes, level
  meshes) are kept in a short list tested against everything. A ray walks
  the grid cells in order (Amanatides & Woo).
- **Narrowphase:** every shape is a convex core plus a radius: a point
  (sphere), a segment (capsule), a box, or a triangle. Rounded shapes use
  GJK for their closest points, plus a face clip that gives a capsule lying
  on a surface two contact points. Box against box or triangle uses the
  separating axis test over face and edge axes, then clips against the
  reference face (Gregorius' GDC talks, the qu3e engine): up to four
  contact points, with faces preferred over edges on a near tie so a
  resting box never flickers.
- **Meshes:** triangles in a bounding volume hierarchy; each nearby triangle
  makes its own contact.
- **Contacts:** points are matched to last step's by where they sit on the
  body, and start from last step's impulses (warm starting). They are
  speculative, so a 300 m/s bullet stops at a 5 cm wall.
- **Solver:** sequential impulses; friction as a true circle in the contact
  plane; rolling friction; substeps with a relax pass (Box2D v3's soft
  step); restitution after the solve. The normal impulses of a manifold's
  points get two passes each time it is solved. One pass leaves the first
  corner carrying too much, and a tall stack then rocks from corner to corner
  and creeps sideways. Measured on a 12-box stack: 0.27 m/s of leftover
  motion with one pass, 0.005 with two.
- **Joints:** soft constraints (springs solved implicitly), so stiff ones stay
  stable.

Measured on this library's development machine, one thread: 1000 mixed
bodies falling into a walled pit take about 4 ms per step while they move,
and nothing escapes. A 12-box stack stands and sleeps, and an 80:1 mass ratio
(a heavy box on a light one) holds.

## Limits

| macro | default |
|---|---|
| `PHYS3D_MAX_BODIES` | 4096 |
| `PHYS3D_MAX_JOINTS` | 4096 |

Not done yet: convex hulls and compound bodies (several shapes on one
body), gyroscopic effects, threading, continuous collision for fast-spinning
bodies, and smoothing of the small bumps a box can catch on where two mesh
triangles meet.

## Reference

| function | |
|---|---|
| `phys3d_create_world(w)`, `phys3d_destroy_world(w)` | |
| `int phys3d_update(w, frame_dt)`, `phys3d_step(w, dt)` | |
| `phys3d_sphere(r)`, `phys3d_box(size)`, `phys3d_capsule(r, h)`, `phys3d_plane(n)`, `phys3d_mesh_shape(mesh)` | shapes |
| `phys3d_create_mesh(m, verts, count, indices, count)`, `phys3d_create_heightfield(m, heights, cols, rows, spacing)`, `phys3d_destroy_mesh(m)` | |
| `idx phys3d_add(w, type, shape, position)`, `phys3d_make_body(type, shape, position)`, `phys3d_add_body(w, body)` | |
| `phys3d_get_body(w, id)`, `phys3d_remove_body(w, id)`, `phys3d_update_mass(w, id)`, `phys3d_teleport(w, id, p, q)`, `phys3d_wake(w, id)` | |
| `phys3d_apply_force`, `_impulse`, `_torque`, `_angular_impulse`, `_force_centre`, `_impulse_centre` | |
| `phys3d_body_transform(w, b)`, `phys3d_lerp_position(w, b)`, `phys3d_lerp_rotation(w, b)` | for drawing |
| `phys3d_add_ball`, `_hinge`, `_slider`, `_fixed`, `_distance`, `_mouse`, `phys3d_add_joint(w, joint)` | |
| `phys3d_get_joint(w, id)`, `phys3d_remove_joint(w, id)`, `phys3d_joint_anchors(w, j, &a, &b)`, `phys3d_hinge_angle(w, j)` | |
| `phys3d_raycast(w, origin, dir, max, &hit, mask, ignore)` | |
| `phys3d_shape_cast(w, shape, pos, rot, dir, max, &hit, mask, ignore)` | |
| `phys3d_overlap(w, shape, pos, rot, out, capacity, mask)`, `phys3d_overlap_sphere(w, centre, r, out, capacity, mask)` | |
| `pix_draw_physics3d(w, flags)`, `phys3d_debug_draw(renderer, w, flags)` | |
