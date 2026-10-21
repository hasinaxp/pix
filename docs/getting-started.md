# Getting started

This page takes you from nothing to a running program and explains the ideas
that every other page assumes. If you have never used pix, read it top to
bottom once.

## What pix is

pix is a C++ library for making games on Windows. It opens a window, reads the
keyboard, mouse and gamepads, draws 2D and 3D graphics with OpenGL, plays
sounds, loads images, models and audio files, and simulates physics in 2D and 3D.

It is **header-only**. There is no `.lib` to build and nothing to install. You
include one header and compile your game as usual.

It has **no dependencies**. Everything, including the PNG decoder, the 3D
model loaders and the audio mixer, is part of pix. The only things it needs
are the Windows SDK and the C/C++ standard library, which come with Visual
Studio.

## What you need

- Windows 10 or 11.
- Visual Studio 2017 or newer, or the "Build Tools for Visual Studio", with
  the **Desktop development with C++** workload. Any edition works, including
  Community.
- A graphics card and driver that support **OpenGL 4.4**. Any GPU from roughly
  the last ten years qualifies, including Intel integrated graphics.
- Python 3, only if you want to regenerate the single-file header yourself.

## Two ways to include pix

The repository ships pix in two forms. Both contain the same code.

| file | when to use it |
|---|---|
| `dist/pix.h` | One file with the whole library. Copy it next to your game. |
| `src/pix.h` | The same library split into about 30 readable modules. Use it if you want to read or change pix. |

If `dist/pix.h` is missing or out of date, run `python compile.py` in the
repository root to regenerate it.

## Your first program

Create a folder for your game, copy `dist/pix.h` into it, and save this as
`game.cpp` next to it:

```cpp
#define PIX_IMPLEMENTATION
#include "pix.h"

int main() {
    if (!pix_init("my game", 1280, 720)) return 1;   // open a 1280x720 window

    while (pix_frame()) {                             // once per frame, until the window closes
        if (pix_key_pressed(KEY_ESC)) pix_quit();

        pix_clear(PIX_SKYBLUE);                       // fill the window with a colour
        pix_draw_rect(100, 100, 200, 80, PIX_RED);    // x, y, width, height
        pix_draw_text("hello pix", 110, 120, 32, PIX_WHITE);
    }

    pix_shutdown();                                   // close the window, free everything
    return 0;
}
```

### Compiling it

Open **"x64 Native Tools Command Prompt for VS"** from the Start menu. It is a
command prompt where the Microsoft compiler `cl` is already on the path. Go to
your game folder and run:

```
cl /std:c++14 /O2 /EHsc game.cpp
game.exe
```

A window opens with a red rectangle and some text. Press Esc to close it.

You don't list any libraries on the command line. pix asks the linker for the
Windows libraries it needs (`opengl32`, `gdi32`, `user32`, `winmm`) from inside
the header.

To use the modular sources instead of the single file, point the compiler at
`src`:

```
cl /std:c++14 /O2 /EHsc /I path\to\pix\src game.cpp
```

### What `PIX_IMPLEMENTATION` does

A header-only library has to put its function *bodies* somewhere. pix puts
them in the header behind a switch. When you `#define PIX_IMPLEMENTATION`
before `#include "pix.h"`, that file compiles the whole library. Without the
define, you only get the declarations.

The rule is: **define it in exactly one `.cpp` file of your program.** In a
one-file game that is your only file. In a bigger game, pick one file (such as
`main.cpp`), and have every other file include `pix.h` without the define.

```cpp
// main.cpp
#define PIX_IMPLEMENTATION
#include "pix.h"

// player.cpp, enemies.cpp, ...
#include "pix.h"
```

Defining it in two files gives "already defined" linker errors. Defining it
nowhere gives "unresolved external symbol" errors.

## The frame loop

A game is a loop that runs once per frame, typically 60 or more times a
second. Every frame it reads input, updates the game, and draws everything
again from scratch.

```cpp
while (pix_frame()) {
    // 1. read input
    // 2. update the game state
    // 3. draw
}
```

`pix_frame()` does three things each time it is called:

1. It shows the frame you drew during the previous loop iteration.
2. It collects new keyboard, mouse, gamepad and window events.
3. It advances the clock.

It returns `false` when the user closes the window or you call `pix_quit()`,
which ends the loop.

Nothing you draw stays on screen. If you stop drawing something, it
disappears next frame. That is normal: the whole picture is redrawn each
frame.

### Moving things at the same speed on every computer

Frames don't take the same time on every machine. If you move a player 5
pixels per frame, it moves twice as fast at 120 fps as at 60 fps. Instead,
multiply speeds by **`pix_dt()`**, the number of seconds the last frame took:

```cpp
float speed = 300.0f;                    // pixels per second
player_x += speed * pix_dt();            // the same speed at any frame rate
```

`pix_dt()` is capped at 0.1 seconds. Without the cap, a long pause (say,
dragging the window) would make everything jump forward.

Other clock functions are `pix_time()` (seconds since `pix_init`) and
`pix_fps()` (frames per second, smoothed).

## Drawing order: 3D, then 2D

pix draws 3D scenes and 2D graphics. In a frame that has both, draw the 3D
scene first and the 2D on top of it:

```cpp
while (pix_frame()) {
    pix_begin_3d(cam);
    // ... 3D draws ...
    pix_end_3d();                  // renders the 3D scene over the whole window

    // ... 2D draws: HUD, menus, text ...
}
```

`pix_end_3d()` paints the entire window, so 2D drawn before it is hidden. A
pure 2D game never calls `pix_begin_3d` and uses `pix_clear` to fill the
background instead.

Within 2D, later draws cover earlier ones, like layers of paint.

## Coordinates

**2D** uses pixels. `(0, 0)` is the **top-left** corner of the window. X grows
to the right and **Y grows downward**. `pix_width()` and `pix_height()` give
the size you draw in: the window's, unless you fix a resolution with
`pix_set_resolution` (see [2D rendering](2d-rendering.md#fixed-resolution-and-pixel-art)).

**3D** uses **Y up**. The ground is the XZ plane. Distances are in whatever
unit you choose, but the defaults (fog, shadow distance, light falloff,
gravity in the 3D physics worlds) are tuned for **1 unit = 1 metre**.

## The basic types

These come from the core of pix and appear everywhere:

| type | what it is |
|---|---|
| `vec2`, `vec3`, `vec4` | 2, 3 and 4 floats. Make them with `v2(x, y)`, `v3(x, y, z)`, `v4(x, y, z, w)`. |
| `mat4` | a 4×4 transform matrix (position, rotation, scale of an object in 3D) |
| `quat` | a rotation, used by animation |
| `pix_color` | an 8-bit RGBA colour. Make one with `pix_rgba(r, g, b, a = 255)` or `pix_hex(0xRRGGBBAA)`. |
| `pix_rect` | a rectangle `{ x, y, w, h }` in pixels |
| `idx` | a handle (an unsigned 32-bit number) for things pix owns: meshes, materials, sounds |

Vectors support the usual operators:

```cpp
vec2 pos = v2(100, 200);
vec2 vel = v2(3, -1);
pos += vel * pix_dt();            // +, -, * by a float, / by a float
float d = v2dist(pos, target);    // also v2len, v2norm, v2dot, v3cross, v3lerp, ...
```

There are ready-made colours: `PIX_WHITE`, `PIX_BLACK`, `PIX_BLANK`
(transparent), `PIX_GRAY`, `PIX_LIGHTGRAY`, `PIX_DARKGRAY`, `PIX_RED`,
`PIX_GREEN`, `PIX_BLUE`, `PIX_YELLOW`, `PIX_ORANGE`, `PIX_PURPLE`,
`PIX_SKYBLUE`. `pix_fade(color, 0.5f)` returns a colour at half its opacity.

The [cheatsheet](cheatsheet.md#math) lists the math helpers.

## Handles and errors

When you load something, pix usually gives you back a handle (an `idx`) rather
than a pointer. You pass the handle to other pix functions later:

```cpp
idx boom = pix_load_sound("boom.wav");
if (boom == PIX_INVALID) { /* the file was missing or broken */ }
pix_play_sound(boom);
```

pix does not throw exceptions. When something fails, the function returns a
failure value (`PIX_INVALID`, `false`, a texture with `id == 0`, ...) and
writes the reason with `pix_log`. Those messages go to the console window and
to Visual Studio's Output window. **When something doesn't appear, read the
console first.**

You can use `pix_log` for your own messages too. It works like `printf`:

```cpp
pix_log("player hp %d", hp);
```

## Memory

You don't have to manage memory for anything pix loads. Handles and the
memory behind them belong to pix and are released by `pix_shutdown`. A few
functions hand you a buffer of your own, such as `pix_read_file` and
`pix_load_image_rgba`. Their descriptions say so, and you release the buffer
with `pix_free`.

For faster temporary memory, your own arenas, or tuning, see
[Memory](memory.md). It is optional.

## Files and the working directory

Paths like `"assets/hero.png"` are relative to the **working directory**: the
folder the program was started from, not necessarily the folder the `.exe`
is in. Run your game from the folder that contains your assets, or set the
working directory in Visual Studio's debugging settings.

pix reads PNG images, OBJ and glTF (`.gltf` / `.glb`) models, WAV sounds,
TrueType (`.ttf`) fonts, Aseprite / TexturePacker sprite atlases, and JSON.

### Reading your own JSON files

For settings, levels or dialogue, pix includes a small JSON reader. It
reads values straight out of the file text without copying:

```cpp
size_t size;
char* text = (char*)pix_read_file("assets/level1.json", &size);
js_doc doc;
int root = js_parse(doc, text, size);                 // -1 if the file is not valid JSON

int enemies = js_get(doc, root, "enemies");
for (int e = js_first(doc, enemies); e >= 0; e = js_next(doc, enemies, e)) {
    float x = js_float(doc, js_get(doc, e, "x"), 0.0f);   // the last argument is the default
    float y = js_float(doc, js_get(doc, e, "y"), 0.0f);
    char kind[32]; js_str(doc, js_get(doc, e, "kind"), kind, sizeof(kind));
    spawn(kind, x, y);
}
js_free(doc);
pix_free(text);
```

Every lookup returns a token index, or -1 when the key or element is
missing. Passing -1 along is harmless and simply gives the default at the
end, so you don't need to check each step. Also available: `js_int`,
`js_bool`, `js_len`, `js_at(doc, array, i)`, `js_type`, and, for walking an
object's keys, `js_first` / `js_value_of` / `js_next`.

## Window options

`pix_init` takes optional flags as its fourth argument. The default is
`PIX_WINDOW_RESIZABLE | PIX_WINDOW_VSYNC`.

```cpp
pix_init("my game", 1280, 720, PIX_WINDOW_RESIZABLE | PIX_WINDOW_VSYNC | PIX_NO_AUDIO);
```

| flag | effect |
|---|---|
| `PIX_WINDOW_RESIZABLE` | the user can resize the window by dragging its border |
| `PIX_WINDOW_FULLSCREEN` | start in borderless fullscreen |
| `PIX_WINDOW_VSYNC` | match the monitor's refresh rate. Without it the frame rate is uncapped. |
| `PIX_WINDOW_DEBUG_GL` | report OpenGL errors through `pix_log` (slower; for debugging) |
| `PIX_WINDOW_HIDDEN` | create the window without showing it |
| `PIX_NO_3D_EFFECTS` | start the 3D renderer without shadows and bloom |
| `PIX_NO_AUDIO` | never open the audio device |

You can change things while the game runs:

```cpp
pix_set_title("my game - level 2");
pix_toggle_fullscreen();
pix_set_vsync_enabled(false);
```

## Two layers of API

Everything shown so far (`pix_init`, `pix_draw_rect`, `pix_load_sound`, ...)
is the **app layer**. It keeps one window, one renderer and one audio device
in a hidden global, so the calls stay short. Most games only need this layer.

Underneath are the **modules** the app layer is built from: `pix_window`,
`pix_renderer`, `pix_batch2d`, `pix_audio`, the file loaders, the physics
world. They are plain structs with free functions, and you can use them
directly when you need more control. The app layer gives you its instances:

| function | returns |
|---|---|
| `pix_get_window()` | the window, with raw input state |
| `pix_get_renderer()` | the 3D renderer (created the first time you ask) |
| `pix_get_batch2d()` | the 2D draw batch |
| `pix_get_audio()` | the audio device (opened the first time you ask) |
| `pix_get_loader()` | the asset tables used by the file loaders |
| `pix_get_jobs()` | a thread pool sized to your CPU |
| `pix_get_animations()` | a crowd animation system |

You can mix the layers freely. For example, `pix_draw_mesh(...)` is just
`push_instance(pix_get_renderer(), ...)`.

## Running the examples

The repository has example programs in `examples/` and a `build.bat` that
compiles them. `build.bat` finds Visual Studio for you, so a normal command
prompt or PowerShell window works. Run it from the repository root, because
the examples load files from `assets/`.

```
build                 build every example into build\
build hello_2d run    build one example, then run it
build test            build and run the test suite
build single          regenerate dist\pix.h and compile every example against it
build debug city      build without optimisation, with debug info
```

| example | shows |
|---|---|
| `hello_2d` | shapes, text, a 2D camera that follows the player, keyboard/gamepad movement |
| `physics_2d` | 2D physics: a walking, jumping player, stacked crates, bouncing balls, a moving platform, a bridge and a wrecking ball on joints, a trigger, mouse dragging |
| `physics_3d` | 3D physics: stacks, a wrecking ball on a chain, a ragdoll, a seesaw, a lift, a door, a heightfield hill, a trigger, a walking player, mouse dragging |
| `hello_3d` | 3D primitives, a model, an animated character, shadows, lights, particles, post effects |
| `animation` | animated characters, blending between animations, footstep sounds, gamepad rumble |
| `reflections` | reflections, water, glass |
| `pipeline` | building a custom frame: a minimap, layers, custom shaders |
| `crowd` | 1,000 animated characters |
| `grassland` | a large open-world scene with grass, trees, creatures, physics and combat |
| `city` | thousands of objects using the lower-level renderer API |

`hello_2d` and `hello_3d` are the best places to start reading.

## Where to go next

- [Input](inputs.md): keyboard, mouse, gamepads, text entry
- [2D rendering](2d-rendering.md): shapes, images, sprites, text, 2D cameras
- [3D rendering](3d-rendering.md): cameras, meshes, materials, models, lights, shadows
- [3D physics](physics-3d.md): rigid bodies, joints and ragdolls
- [2D physics](physics-2d.md): rigid bodies for 2D games
- [Physics](physics.md): which physics world to use, what they share, and shape tests
- [Flat 3D physics](physics-3d-flat.md): characters and crowds on the ground
- [Memory](memory.md): optional control over where memory comes from
- [Cheatsheet](cheatsheet.md): every common call on one page

## Reference: lifetime, frame and window

| function | |
|---|---|
| `bool pix_init(title, width, height, flags = RESIZABLE \| VSYNC)` | open the window and set everything up; `false` on failure |
| `void pix_shutdown()` | close the window and free everything the app layer made |
| `bool pix_frame()` | show the last frame, read input, advance the clock; `false` when it's time to quit |
| `void pix_quit()` | make the next `pix_frame` return `false` |
| `float pix_dt()` | seconds the last frame took, at most 0.1 |
| `double pix_time()` | seconds since `pix_init` |
| `int pix_fps()` | frames per second, smoothed |
| `int pix_width()`, `int pix_height()` | the size you draw in: the window's, or a [fixed resolution](2d-rendering.md#fixed-resolution-and-pixel-art) |
| `void pix_set_title(title)` | |
| `void pix_set_vsync_enabled(on)` | |
| `void pix_toggle_fullscreen()` | |
| `void pix_log(fmt, ...)` | printf-style message to the console and debugger |
