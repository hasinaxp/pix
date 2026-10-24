# 2D physics

`core/physics2d.hpp` is a 2D rigid-body world. Shapes fall, slide, bounce,
spin, stack and rest on each other. It is built for platformers, puzzle
games, top-down games and anything else drawn with the 2D API.

It uses the same coordinates as 2D drawing: **x right, y down**, in whatever
units you draw in. The defaults assume **pixels** (gravity is 980 pixels per
second squared, downward).

You can read this page top to bottom. Every section works without the ones
after it, and the last sections are for tuning.

## A complete example

A floor, a crate that falls onto it, and a ball you drop with the mouse:

```cpp
#define PIX_IMPLEMENTATION
#include "pix.h"

int main() {
    if (!pix_init("2d physics", 1280, 720)) return 1;

    phys2d_world world;
    phys2d_create_world(world);
    phys2d_add(world, PHYS2D_STATIC,  phys2d_box(1200, 40), v2(640, 680));   // floor
    phys2d_add(world, PHYS2D_DYNAMIC, phys2d_box(60, 60),   v2(640, 100));   // crate

    while (pix_frame()) {
        if (pix_mouse_pressed(0))
            phys2d_add(world, PHYS2D_DYNAMIC, phys2d_circle(20), pix_mouse_position());

        phys2d_update(world, pix_dt());       // move everything

        pix_clear(PIX_BLACK);
        pix_draw_physics2d(world);            // outlines of every body
    }
    phys2d_destroy_world(world);
    pix_shutdown();
}
```

`pix_draw_physics2d` draws outlines only. It's how you see what the physics
is doing. Your game draws its own sprites at the bodies' positions (see
[Drawing bodies](#drawing-bodies)).

`examples/physics_2d.cpp` is a larger playground: a player that walks and
jumps, a moving platform, a coin trigger, a raycast, a pyramid of crates, a
plank bridge and a wrecking ball on joints, and dragging bodies with the
mouse. Build it with `build physics_2d run`.

## Shapes

| constructor | what it is |
|---|---|
| `phys2d_circle(radius)` | a circle |
| `phys2d_box(width, height)` | a box that rotates when knocked over |
| `phys2d_aabb(width, height)` | a box that never rotates: tiles, platforms, simple characters |
| `phys2d_capsule(radius, height)` | an upright pill shape, `height` from top to bottom. The usual shape for a character |
| `phys2d_polygon(points, count, radius)` | any convex outline: triangles, hexagons, ramps, rocks |

Sizes are full widths and heights. A body's `position` is its centre.

`phys2d_polygon` takes any points in any order and keeps their convex hull,
at most 8 corners (for more, the ones that matter least are dropped).
`radius` rounds the corners. The polygon is centred on its centre of mass;
`shape.offset` is where that was among your points, for drawing a sprite
that lines up:

```cpp
vec2 tri[3] = { v2(0, -30), v2(30, 20), v2(-30, 20) };
phys2d_shape wedge = phys2d_polygon(tri, 3);
idx id = phys2d_add(world, PHYS2D_DYNAMIC, wedge, v2(300, 100));
```

## Body types

| type | moves? | example |
|---|---|---|
| `PHYS2D_STATIC` | never on its own | floors, walls, ramps |
| `PHYS2D_DYNAMIC` | gravity and collisions move it | crates, balls, the player |
| `PHYS2D_KINEMATIC` | only at the `velocity` you set | moving platforms, doors, elevators |

A kinematic body pushes dynamic bodies and carries the ones standing on it,
but nothing pushes it back. Walls don't stop it either.

## Bodies

`phys2d_add(world, type, shape, position)` returns an `idx` handle. Ask the
world for the body itself when you need it:

```cpp
idx crate = phys2d_add(world, PHYS2D_DYNAMIC, phys2d_box(60, 60), v2(400, 0));

phys2d_body* b = phys2d_get_body(world, crate);
b->velocity = v2(200, -300);     // throw it
```

`phys2d_get_body` returns `nullptr` once the body has been removed with
`phys2d_remove_body(world, id)`.

To set anything other than type, shape and position, start from
`phys2d_make_body`, edit the fields, then add it:

```cpp
phys2d_body ball = phys2d_make_body(PHYS2D_DYNAMIC, phys2d_circle(16), v2(300, 0));
ball.restitution = 0.8f;      // bouncy
ball.friction = 0.2f;
idx id = phys2d_add_body(world, ball);
```

### Body fields

| field | default | |
|---|---|---|
| `position`, `angle` | | centre, and rotation in radians (clockwise on screen) |
| `velocity`, `angular_velocity` | 0 | pixels per second, radians per second |
| `friction` | 0.6 | 0 is ice, 1 is rubber |
| `restitution` | 0 | bounciness: 0 is a dead stop, 1 bounces back to the same height |
| `density` | 1 | mass is density times area, so a bigger crate is heavier |
| `gravity_scale` | 1 | 0 floats, 2 falls twice as fast, -1 falls upward |
| `linear_damping`, `angular_damping` | 0 | slows the body down over time, like air |
| `fixed_rotation` | false | never rotates. Use it for characters |
| `trigger` | false | see [Triggers](#triggers) |
| `layer`, `mask` | layer 0, all | see [Collision layers](#collision-layers) |
| `user` | null | your own pointer, for finding your game object from a body |

Read-only, filled in by the world:

| field | |
|---|---|
| `on_ground` | touching something below it this step (below means "in the direction of gravity") |
| `ground_normal` | the surface it stands on, pointing up out of it |

After changing `shape`, `density`, `type` or `fixed_rotation` on a body
that's already in the world, call `phys2d_update_mass(world, id)`.

### Moving a body

- **Set the velocity.** This is the usual way. A player's horizontal speed
  and a jump are both just velocity.
- **Push it.** `phys2d_apply_impulse_centre(b, impulse)` is an instant kick.
  `phys2d_apply_force_centre(b, force)` is a steady push applied over the
  next step. The `_force` / `_impulse` versions without `_centre` take a
  world point, so the push also spins the body.
- **Teleport it.** `phys2d_teleport(world, id, position, angle)`. Use it
  rather than writing `position` directly, so the drawn position doesn't
  smear between the old and new places.

### A platformer character

```cpp
phys2d_body p = phys2d_make_body(PHYS2D_DYNAMIC, phys2d_capsule(16, 64), v2(100, 400));
p.fixed_rotation = true;   // stays upright
p.friction = 0;            // we set its speed ourselves, so walls don't grab it
idx player = phys2d_add_body(world, p);

// each frame, before phys2d_update
phys2d_body* pl = phys2d_get_body(world, player);
float move = (pix_key_down('D') ? 1.0f : 0.0f) - (pix_key_down('A') ? 1.0f : 0.0f);
pl->velocity.x = move * 300;
if (pix_key_pressed(KEY_SPACE) && pl->on_ground) pl->velocity.y = -550;   // up is negative y
```

A capsule's round bottom slides up slopes and over small bumps, where a box
would catch on them.

## Stepping the world

Call `phys2d_update(world, pix_dt())` once per frame. It runs as many
**fixed steps** of `world.fixed_dt` (1/60 of a second) as the frame took,
and returns how many ran. Fixed steps make jumps the same height at any
frame rate.

If a frame takes very long (a hitch, a breakpoint), at most
`world.max_steps` (4) steps run and the rest of the time is skipped, so the
game slows down for a moment instead of freezing.

If your code runs its own clock, `phys2d_step(world, dt)` runs exactly one
step of `dt`.

### Drawing bodies

```cpp
for (size_t i = 0; i < world.body_count; i++) {
    const phys2d_body& b = world.bodies[i];
    if (!b.active) continue;
    vec2  pos   = phys2d_lerp_position(world, b);
    float angle = phys2d_lerp_angle(world, b) * PIX_RAD2DEG;
    // draw your sprite at pos, rotated by angle
}
```

Because steps are fixed, a frame usually lands between two of them.
`phys2d_lerp_position` and `phys2d_lerp_angle` give the in-between pose, so
motion looks smooth on a 144 Hz monitor too. Drawing `b.position` directly
also works, but motion can look slightly jerky.

## Contact events

After `phys2d_update`, `world.events` lists everything that started or
stopped touching during that update:

```cpp
for (size_t i = 0; i < world.event_count; i++) {
    const phys2d_event& e = world.events[i];
    if (e.type == PHYS2D_CONTACT_BEGIN && e.speed > 300) play_sound(thud);
}
```

| type | `a`, `b` | also |
|---|---|---|
| `PHYS2D_CONTACT_BEGIN` | the two bodies | `point`, `normal` (from a to b), `speed`: how hard they hit |
| `PHYS2D_CONTACT_END` | the two bodies | |
| `PHYS2D_TRIGGER_ENTER` | `a` is the trigger, `b` the body that entered | |
| `PHYS2D_TRIGGER_EXIT` | `a` is the trigger, `b` the body that left | |
| `PHYS2D_JOINT_BREAK` | the two bodies the joint held | `joint`: which joint broke (it is already removed), `speed`: the force it took |

Removing a body ends all of its contacts, and the end events arrive after
the next step.

## Triggers

A trigger detects overlaps but never pushes and is never pushed: coins,
checkpoints, damage zones, doors that open when you walk up.

```cpp
phys2d_body coin = phys2d_make_body(PHYS2D_STATIC, phys2d_circle(14), v2(500, 300));
coin.trigger = true;
idx coin_id = phys2d_add_body(world, coin);

// after phys2d_update
for (size_t i = 0; i < world.event_count; i++)
    if (world.events[i].type == PHYS2D_TRIGGER_ENTER && world.events[i].a == coin_id)
        collect(world.events[i].b);
```

Triggers can also be dynamic or kinematic, for example a sword's hit area
that moves with the player. Two triggers never detect each other.

## Joints

Joints connect two bodies, or one body to the world (pass `PIX_INVALID` for
the other body). You give anchors and axes in world space, where the bodies
are right now.

```cpp
// a door: a box hinged at its left edge
idx door = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(10, 80), v2(205, 300));
phys2d_add_revolute(w, PIX_INVALID, door, v2(200, 260));
```

| function | joint | used for |
|---|---|---|
| `phys2d_add_revolute(w, a, b, anchor)` | a shared pin both bodies turn about | hinges, wheels, chains, ragdolls |
| `phys2d_add_distance(w, a, b, anchor_a, anchor_b)` | keeps two points apart | rods, springs, ropes |
| `phys2d_add_prismatic(w, a, b, anchor, axis)` | `b` slides along an axis fixed in `a`, without turning | pistons, lifts, sliding doors |
| `phys2d_add_weld(w, a, b, anchor)` | glues two bodies together | breakable structures, springy attachments |
| `phys2d_add_mouse(w, body, grab)` | pulls the point under `grab` toward `target` | dragging things with the mouse |

Each returns an `idx`. `phys2d_get_joint(w, id)` gives the joint to edit
(`nullptr` once it's removed), and `phys2d_remove_joint(w, id)` removes it.
Removing a body removes its joints too.

Two jointed bodies don't collide with each other. Set
`collide_connected = true` to change that.

### Limits, motors and springs

These are all off until you turn them on:

```cpp
phys2d_joint* j = phys2d_get_joint(w, hinge);
j->enable_limit = true;  j->lower = -0.5f;  j->upper = 0.5f;            // radians either way
j->enable_motor = true;  j->motor_speed = 3.0f;  j->max_motor_force = 5e6f;  // a powered wheel
```

| field | revolute | prismatic | distance |
|---|---|---|---|
| `enable_limit`, `lower`, `upper` | angle, radians | travel along the axis | shortest and longest length |
| `enable_motor`, `motor_speed`, `max_motor_force` | turning speed (rad/s) and torque | sliding speed and force | |
| `enable_spring`, `hertz`, `damping_ratio` | | | a spring at `length` |

A distance joint is a rigid rod by default. For a **spring**, set
`enable_spring = true` and `hertz` (how many times a second it bounces: 1
to 5 feels springy). `damping_ratio` is how fast the bounce dies out: 0
never, 1 with no overshoot. For a **rope**, set `enable_spring = true` with
`hertz = 0` (no spring at all) and `enable_limit` with `upper` as the rope's
length. It goes slack when shorter and holds taut at its length.

A weld is rigid. Give it `hertz` to make it springy.

A mouse joint drags with a spring of `hertz` 5 and a pull of at most
`max_force`. Move `j->target` every frame and remove the joint on release:

```cpp
if (pix_mouse_pressed(0)) drag = phys2d_add_mouse(w, picked_body, mouse);
if (phys2d_joint* d = phys2d_get_joint(w, drag)) d->target = mouse;
if (pix_mouse_released(0)) { phys2d_remove_joint(w, drag); drag = PIX_INVALID; }
```

### Breaking

`break_force` > 0 makes a joint snap when it has to pull harder than that.
It is removed and a `PHYS2D_JOINT_BREAK` event is reported. `reaction` holds
how hard every joint pulled last step, so you can show stress or play a creak
before it goes.

## Sleeping

Bodies that have been still for half a second fall asleep. They stop costing
anything and stay exactly where they are until something wakes them:

- something moving touches them, or something they rest on is removed;
- you set their velocity, move them, or push them with a force or impulse;
- `phys2d_wake(w, id)`.

Bodies that touch or are jointed sleep and wake together as one group, so a
crate landing on a sleeping pile wakes the whole pile. `b->sleeping` tells
you whether a body is asleep. A body that must never sleep (one the game
moves by hand every frame, say) sets `never_sleep`. `w.allow_sleep = false`
turns sleeping off for the whole world.

## Collision layers

Each body is on some layers (`layer`) and collides with some layers
(`mask`). Two bodies collide only if **each** accepts the other:
`(a.mask & b.layer) && (b.mask & a.layer)`. Layers are the `PHYS_LAYER_*`
bits, and you name them for your game:

```cpp
#define LAYER_PLAYER PHYS_LAYER_0
#define LAYER_ENEMY  PHYS_LAYER_1
#define LAYER_COIN   PHYS_LAYER_2

coin.layer = LAYER_COIN;
coin.mask  = LAYER_PLAYER;          // only the player picks coins up

enemy.layer = LAYER_ENEMY;
enemy.mask  = PHYS_LAYER_ALL & ~LAYER_ENEMY;   // enemies pass through each other
```

The same rule applies to triggers, so a trigger whose mask is the player's
layer reports only the player. It is the same rule in every pix world
([Physics: collision layers](physics.md#collision-layers)).

## Asking the world questions

### Raycasts

```cpp
phys2d_ray_hit hit;
if (phys2d_raycast(world, eye, target - eye, 800, &hit)) {
    // hit.body, hit.point, hit.normal, hit.distance
}
```

The direction doesn't have to be normalised. The ray finds the closest
solid body. It ignores triggers, and it ignores bodies the ray starts
inside (so a ray from the player's centre doesn't hit the player). Two
optional arguments filter the search: `mask` (which layers to hit) and
`ignore` (one body to skip).

### Shape casts

A raycast with thickness: slide a shape along a direction and find what it
would hit first.

```cpp
// would a 16-radius ball thrown right hit something within 300 pixels?
if (phys2d_shape_cast(world, phys2d_circle(16), pos, 0, v2(1, 0), 300, &hit)) { ... }
```

The shape moves without rotating. If it already overlaps something at the
start, the hit distance is 0.

### Overlap queries

```cpp
idx found[32];
size_t n = phys2d_overlap(world, phys2d_circle(100), explosion_centre, 0, found, 32);
for (size_t i = 0; i < n && i < 32; i++) { ... }

n = phys2d_overlap_point(world, pix_mouse_world_2d(cam), found, 32);   // what's under the mouse?
```

These return every body touching the shape, triggers included. The count
can be larger than the array: only the first `capacity` ids are written.

## Seeing what the physics does

`pix_draw_physics2d(world, flags, thickness)` draws through the current 2D
camera, so call it between `pix_begin_2d` and `pix_end_2d` if you use one.

| flag | draws |
|---|---|
| `PHYS2D_DRAW_SHAPES` | every body's outline: grey static, green dynamic, yellow while on the ground, dark green asleep, blue kinematic, cyan trigger. The line from the centre shows rotation |
| `PHYS2D_DRAW_JOINTS` | each joint's anchors, in purple, joined to the bodies' centres |
| `PHYS2D_DRAW_CONTACTS` | red dots where bodies touch, with the contact normal. Faint yellow dots are near misses the solver is watching |
| `PHYS2D_DRAW_AABBS` | the bounding boxes the broadphase uses (stretched along the motion of moving bodies) |
| `PHYS2D_DRAW_GRID` | the broadphase grid cells that hold moving bodies |

The default is shapes, contacts and joints. When using the batch directly, call
`phys2d_debug_draw(batch, world, flags, thickness)`.

## Tuning

Everything here is a plain field on the world. Change it at any time.

| field | default | |
|---|---|---|
| `gravity` | (0, 980) | (0, 0) for a top-down game |
| `fixed_dt` | 1/60 | 1/120 for faster or more precise games, at twice the cost |
| `max_steps` | 4 | steps per `phys2d_update` before time is dropped |
| `substeps` | 4 | each step is solved in this many short substeps. More is stiffer: long chains, tall stacks, heavy bodies on light ones |
| `iterations` | 1 | solver passes per substep |
| `relax_iterations` | 1 | passes per substep that remove the velocity left over from pushing bodies apart |
| `joint_hertz`, `joint_damping` | 60, 2 | how stiff "rigid" joints are |
| `allow_sleep` | true | |
| `sleep_linear`, `sleep_angular` | 8, 0.05 | slower than this counts as still (units/s, radians/s) |
| `time_to_sleep` | 0.5 | seconds still before a group sleeps |
| `cell_size` | 64 | broadphase cell. Around the size of a typical body |
| `slop` | 0.5 | overlap that's allowed to stay, which keeps resting contacts from jittering |
| `correction` | 0.2 | fraction of the remaining overlap pushed out per step |
| `speculative` | 4 | how far ahead of touching a contact is created |
| `bounce_threshold` | 30 | impacts slower than this don't bounce, so resting bodies don't buzz |

If your world isn't in pixels (say, metres with 1 unit = 1 m), scale
`gravity`, `cell_size`, `slop`, `speculative`, `bounce_threshold` and
`sleep_linear` with it. For metres that's roughly `(0, 9.8)`, `1`, `0.005`,
`0.04`, `0.5` and `0.05`.

### How it works

It's the design Box2D uses, so Box2D's documentation and Erin Catto's
talks apply directly:

- **Broadphase:** a hashed uniform grid. Static bodies live in their own grid,
  which is rebuilt only when a static body is added, removed or moved. Moving
  bodies are rebuilt every step. A pair of bodies that shares several cells is
  tested only by the cell holding the corner of their overlap, so no pair is
  tested twice.
- **Narrowphase:** every shape is a core (a point, a segment or a box) plus
  a radius. One routine handles every pair of shapes: SAT when the cores
  overlap, closest points when they don't, and clipping against a
  reference face to get two contact points when flat sides meet.
- **Solver:** sequential impulses with warm starting (each contact point
  starts from last step's impulse, matched by which edges made it). The
  two points of a face contact are solved together (Box2D v2's block
  solver), which is what makes stacks stand still. Coulomb friction,
  Baumgarte position correction, and restitution applied after the solve.
- **Substeps:** contacts are found once per step, then the step is solved
  as several short substeps, each ending with a relax pass that removes the
  velocity left over from pushing overlaps apart (Box2D v3's "soft step").
  Short substeps are what stop a chain stretching or a heavy box sinking
  into a light one. They work far better than more iterations of one long
  step at the same cost: measured, a hanging chain's links stretched 8.9
  pixels with 8 iterations of one step and 1.6 with 4 substeps.
- **Joints:** soft constraints. Each one behaves like a damped spring of a
  chosen stiffness and is solved implicitly, so even a very stiff one stays
  stable. A "rigid" joint is a spring stiff enough that you can't tell.
- **Islands:** each step, touching and jointed bodies are grouped
  (union-find), and a group sleeps only when all of it is still.
- **No tunnelling:** contacts are speculative. A pair is checked while it's
  closer than it can travel in one step, and the solver only lets it close
  that gap. A fast ball stops at a thin wall instead of passing through it.
  One consequence: a very fast bouncing ball can turn around up to one step
  before reaching the surface.

Measured on this library's development machine: 2000 dynamic bodies (circles,
boxes, capsules) settling in one pile, plus 2000 static tiles, take about
2.4 ms per step on one thread while they move.

## Limits

| macro | default |
|---|---|
| `PHYS2D_MAX_BODIES` | 4096 |
| `PHYS2D_MAX_JOINTS` | 4096 |

Define them before including pix to change them. Contacts and events grow
as needed.

Not done yet: threading, several shapes on one body (compounds), the wheel,
motor, gear and pulley joints the 3D world has, a character controller like
the 3D one, one-way platforms, and joint motors on distance joints.

## Reference

| function | |
|---|---|
| `phys2d_create_world(w)`, `phys2d_destroy_world(w)` | |
| `int phys2d_update(w, frame_dt)` | fixed steps, returns how many ran |
| `phys2d_step(w, dt)` | one step |
| `phys2d_circle(r)`, `phys2d_box(w, h)`, `phys2d_aabb(w, h)`, `phys2d_capsule(r, h)` | shapes |
| `idx phys2d_add(w, type, shape, position)` | `PIX_INVALID` when full |
| `phys2d_body phys2d_make_body(type, shape, position)` | defaults, to edit |
| `idx phys2d_add_body(w, body)` | |
| `phys2d_body* phys2d_get_body(w, id)` | `nullptr` once removed |
| `phys2d_remove_body(w, id)` | |
| `phys2d_update_mass(w, id)` | after editing shape, density, type, fixed_rotation |
| `phys2d_teleport(w, id, position, angle)` | |
| `phys2d_wake(w, id)` | |
| `phys2d_apply_force(b, force, point)`, `phys2d_apply_force_centre(b, force)` | |
| `phys2d_apply_impulse(b, impulse, point)`, `phys2d_apply_impulse_centre(b, impulse)` | |
| `phys2d_lerp_position(w, b)`, `phys2d_lerp_angle(w, b)` | pose to draw |
| `bool phys2d_raycast(w, origin, dir, max, &hit, mask, ignore)` | |
| `bool phys2d_shape_cast(w, shape, pos, angle, dir, max, &hit, mask, ignore)` | |
| `size_t phys2d_overlap(w, shape, pos, angle, out, capacity, mask)` | |
| `size_t phys2d_overlap_point(w, point, out, capacity, mask)` | |
| `phys2d_add_revolute(w, a, b, anchor)`, `phys2d_add_weld(w, a, b, anchor)` | |
| `phys2d_add_prismatic(w, a, b, anchor, axis)` | |
| `phys2d_add_distance(w, a, b, anchor_a, anchor_b)` | rod; spring and rope by fields |
| `phys2d_add_mouse(w, body, grab, max_force = auto)` | |
| `idx phys2d_add_joint(w, joint)` | a joint filled in by hand, anchors in local space |
| `phys2d_joint* phys2d_get_joint(w, id)`, `phys2d_remove_joint(w, id)` | |
| `phys2d_joint_anchors(w, joint, &a, &b)` | the anchors in world space |
| `pix_draw_physics2d(w, flags, thickness)` | debug view |
| `phys2d_debug_draw(batch, w, flags, thickness)` | the same, on a batch |
