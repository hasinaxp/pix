# pix documentation

pix is a header-only C++ library for making games on Windows. It opens a
window, reads the keyboard, mouse and gamepads, draws 2D and 3D graphics,
plays sounds, loads images, models and audio, and simulates simple physics.
It needs nothing but Visual Studio. Every part of it, down to the image
decoder, is included.

## Start here

New to pix? Read [Getting started](getting-started.md) first. It walks
through installing, compiling and running a first program, and explains the
ideas the other pages build on: the frame loop, coordinates, the basic types,
and how pix reports errors.

## Pages

| page | covers |
|---|---|
| [Getting started](getting-started.md) | setup, compiling, the frame loop, timing, coordinates, types, errors, the examples |
| [Input](inputs.md) | keyboard, mouse, mouse look, gamepads and rumble, typed text |
| [2D rendering](2d-rendering.md) | shapes, colours, images, sprite sheets, text, 2D cameras, clipping |
| [3D rendering](3d-rendering.md) | cameras, transforms, meshes, materials, models, animation, lights, shadows, post-processing, picking |
| [Physics](physics.md) | the physics world (walls, bodies, raycasts) and shape tests for 2D and 3D |
| [Memory](memory.md) | optional: scoped temporary memory, stack buffers, arenas, statistics, limits, a custom heap |
| [Cheatsheet](cheatsheet.md) | the common calls on one page, including math, random numbers and audio |

Not covered yet: custom shaders, composing your own render pipeline,
reflections, compute shaders, crowd animation and mesh processing. Until
those pages exist, the `examples/` folder shows each of them in a working
program, and the comments in the headers describe every function.

## Where things live

| path | |
|---|---|
| `dist/pix.h` | the whole library in one file; regenerate it with `python compile.py` |
| `src/pix.h` | the same library as separate modules, for reading |
| `src/core/app.hpp` | the easy layer most games use: `pix_init`, `pix_frame`, `pix_draw_*` |
| `src/core/memory.hpp` | the allocators every module uses |
| `src/core/*.hpp`, `src/loader/*.hpp` | the modules underneath. Each declares its API at the top with a comment on every function. |
| `examples/` | one complete program per topic |
