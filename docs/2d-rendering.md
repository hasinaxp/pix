# 2D rendering

pix draws 2D shapes, images and text. You can use it for a whole 2D game, or
for the HUD and menus on top of a 3D one.

2D drawing is **immediate**: you call a draw function every frame for
everything you want on screen. Nothing persists between frames.

## The basics

```cpp
while (pix_frame()) {
    pix_clear(pix_rgba(24, 26, 34));                       // background

    pix_draw_rect(50, 50, 200, 100, PIX_RED);              // x, y, width, height
    pix_draw_circle(v2(400, 120), 40, PIX_YELLOW);         // centre, radius
    pix_draw_line(v2(50, 300), v2(500, 260), 4, PIX_GREEN);   // from, to, thickness
    pix_draw_text("score: 10", 50, 400, 32, PIX_WHITE);    // text, x, y, size
}
```

### Coordinates

Positions are in **pixels**. `(0, 0)` is the **top-left** corner of the
window. X grows to the right and **Y grows down**, so a larger Y is lower on
the screen. `pix_width()` and `pix_height()` give the window size, which can
change if the window is resizable.

```cpp
float cx = pix_width() * 0.5f;                      // horizontal centre
pix_draw_rect(cx - 50, pix_height() - 60.0f, 100, 40, PIX_BLUE);   // centred near the bottom
```

### Draw order

Things drawn later appear on top of things drawn earlier. Draw the background
first, then the world, then the HUD.

### Clearing the screen

`pix_clear(color)` fills the whole window with one colour. Call it at the
start of each frame in a 2D game. In a 3D game you don't need it, because
`pix_end_3d()` already fills the window.

## Colours

Every draw call takes a `pix_color`, which has red, green, blue and alpha
(opacity), each 0..255.

```cpp
pix_color orange = pix_rgba(255, 140, 0);          // alpha defaults to 255 (opaque)
pix_color glass  = pix_rgba(120, 200, 255, 100);   // see-through
pix_color teal   = pix_hex(0x2BB5A0FF);            // 0xRRGGBBAA, as in design tools
pix_color faded  = pix_fade(PIX_WHITE, 0.3f);      // 30% of its opacity
```

Named colours: `PIX_WHITE`, `PIX_BLACK`, `PIX_BLANK` (fully transparent),
`PIX_GRAY`, `PIX_LIGHTGRAY`, `PIX_DARKGRAY`, `PIX_RED`, `PIX_GREEN`,
`PIX_BLUE`, `PIX_YELLOW`, `PIX_ORANGE`, `PIX_PURPLE`, `PIX_SKYBLUE`.

## Shapes

```cpp
pix_rect box = { 100, 100, 240, 120 };             // x, y, w, h

pix_draw_rect(box, PIX_DARKGRAY);                  // filled
pix_draw_rect_lines(box, 3, PIX_WHITE);            // outline, 3 px thick
pix_draw_rect_rounded(box, 12, PIX_BLUE);          // corner radius 12
pix_draw_rect_gradient(box, PIX_SKYBLUE, PIX_BLUE);   // top colour, bottom colour

pix_draw_circle(v2(400, 300), 50, PIX_RED);
pix_draw_circle_lines(v2(400, 300), 50, 4, PIX_WHITE);

pix_draw_line(v2(0, 0), v2(200, 150), 2, PIX_GREEN);
pix_draw_triangle(v2(500, 100), v2(560, 200), v2(440, 200), PIX_PURPLE);
```

`pix_draw_rect` also has a form that takes `x, y, w, h` directly.

A health bar, as an example of combining shapes:

```cpp
void draw_health(float x, float y, float hp, float max_hp) {
    pix_draw_rect_rounded({ x, y, 200, 20 }, 6, pix_rgba(0, 0, 0, 160));
    pix_draw_rect_rounded({ x + 2, y + 2, 196 * (hp / max_hp), 16 }, 5, PIX_RED);
}
```

## Images

### Loading

```cpp
pix_texture hero = pix_load_texture("assets/hero.png", TEXTURE_PIXELATED);
if (hero.id == 0) { /* failed: the console says why */ }
```

Load images **once**, before the game loop, not every frame. A
`pix_texture` holds the GPU texture `id`, its `width` and its `height` in
pixels. Only PNG files are supported.

The second argument controls how the image looks when scaled:

| flag | use it for |
|---|---|
| `TEXTURE_PIXELATED` | pixel art: scaled-up pixels stay sharp squares |
| `TEXTURE_LINEAR` | smooth 2D art, UI, sprite atlases |
| `TEXTURE_BILINEAR` (the default) | textures on 3D models; adds mipmaps |

For 2D, use `TEXTURE_PIXELATED` or `TEXTURE_LINEAR`. You can add
`| TEXTURE_CLAMP` to stop the edges from wrapping around.

Free a texture you no longer need with `pix_unload_texture(hero)`.

### Drawing

```cpp
pix_draw_texture(hero, 100, 200);                     // top-left at (100, 200), actual size
pix_draw_texture(hero, 100, 200, pix_rgba(255, 120, 120));   // tinted red
```

The tint multiplies the image's colours. White leaves it unchanged, and lower
alpha makes it see-through.

### Scaling, rotating, cropping: `pix_draw_texture_pro`

```cpp
pix_draw_texture_pro(texture, src, dst, origin, rotation_degrees, tint);
```

- `src` is the rectangle of the image to use, in the image's pixels.
- `dst` gives the position (`dst.x`, `dst.y`) and the size on screen
  (`dst.w`, `dst.h`).
- `origin` is the pivot point, measured from the top-left of the drawn image.
  The pivot lands at `(dst.x, dst.y)`, and rotation turns around it.
- `rotation_degrees` turns clockwise on screen.

```cpp
pix_rect src = { 0, 0, (float)ship.width, (float)ship.height };   // the whole image
pix_rect dst = { ship_x, ship_y, 64, 64 };                        // drawn at 64x64
vec2 origin  = v2(32, 32);                                        // pivot at its centre
pix_draw_texture_pro(ship, src, dst, origin, angle_degrees, PIX_WHITE);
```

With `origin = (0, 0)`, `dst.x, dst.y` is the top-left corner, like
`pix_draw_texture`. With `origin = (w/2, h/2)` it is the centre, which is
usually what you want for things that rotate.

To **flip** an image, make the source width (or height) negative:

```cpp
pix_rect src = { 0, 0, -(float)hero.width, (float)hero.height };   // mirrored horizontally
```

### Sprite sheets and animation

A sprite sheet is one image holding many frames in a grid. Draw one frame by
choosing its `src` rectangle:

```cpp
// hero.png: frames 32x32, laid out in a row
pix_texture sheet = pix_load_texture("assets/hero.png", TEXTURE_PIXELATED);
int frame_count = 6;
float fps = 10;

// each frame:
int frame = (int)(pix_time() * fps) % frame_count;
pix_rect src = { frame * 32.0f, 0, facing_left ? -32.0f : 32.0f, 32 };
pix_rect dst = { x, y, 96, 96 };                  // 3x scale
pix_draw_texture_pro(sheet, src, dst, v2(48, 96), 0, PIX_WHITE);   // pivot at the feet
```

### Keeping it fast

All 2D drawing goes into one batch, which is sent to the GPU (one draw call)
at the end of the frame or whenever the texture changes. Shapes count as one
texture, each font as another, and each image as its own. Drawing a shape,
then text, then a shape again costs three draw calls. Drawing all the shapes
and then all the text costs two.

Put many small images into **one sprite sheet (an atlas)** instead of many
separate files, and group draws that use the same image. A few hundred draw
calls a frame is still fine, so this only matters once you draw thousands of
sprites.

## Text

```cpp
pix_draw_text("Hello", 20, 20, 32, PIX_WHITE);                 // text, x, y, size, colour
pix_draw_textf(20, 60, 24, PIX_YELLOW, "score %d  time %.1f", score, t);   // printf-style
```

- `x, y` is the **top-left** of the first line.
- `size` is the height of a line in pixels.
- `\n` starts a new line.
- Text stays sharp at any size, because pix renders it from a signed distance
  field rather than a fixed-size bitmap.

The default font is Segoe UI from Windows (falling back to Arial, Tahoma or
Consolas). It covers printable ASCII characters.

### Measuring and centring

```cpp
const char* msg = "GAME OVER";
vec2 size = pix_measure_text(msg, 64);                     // width, height in pixels
pix_draw_text(msg, (pix_width() - size.x) * 0.5f, (pix_height() - size.y) * 0.5f, 64, PIX_RED);
```

### A frame counter

`pix_draw_fps(x, y)` draws the current frame rate, coloured green, orange or
red.

### Other fonts

```cpp
font_data* title_font = pix_load_font("assets/fonts/Title.ttf");   // once, before the loop
pix_draw_text_font(*title_font, "My Game", 40, 40, 72, PIX_WHITE);
// at shutdown:
pix_unload_font(title_font);
```

### Outlines, bold and letter spacing

The batch API has a styled text call:

```cpp
pix_text_style2d st = pix_text_style(40);    // size 40
st.outline = 0.2f;                           // 0 = none, about 0.1..0.3 is visible
st.outline_color = PIX_BLACK;
st.weight = 0.1f;                            // -0.25 thin .. +0.25 bold
st.spacing = 2;                              // extra pixels between letters
pix_batch_text_ex(pix_get_batch2d(), pix_default_font(), "READY?", v2(100, 100), st, PIX_YELLOW);
```

## 2D camera

For a world bigger than the screen (a platformer level, a top-down map), draw
it through a **camera**. You keep positions in world coordinates, and the
camera decides which part of the world is visible, how zoomed in, and whether
it is rotated.

```cpp
pix_camera2d cam = {};
cam.zoom = 1.0f;

// each frame:
cam.target = player;                                         // the world point to look at...
cam.offset = v2(pix_width() * 0.5f, pix_height() * 0.5f);    // ...appears at the screen centre

pix_begin_2d(cam);
    draw_level();                      // world coordinates
    draw_player();
pix_end_2d();

pix_draw_textf(10, 10, 24, PIX_WHITE, "hp %d", hp);   // after pix_end_2d: screen coordinates again
```

| field | meaning |
|---|---|
| `target` | the world point the camera looks at |
| `offset` | where on screen that point appears, in pixels. Usually the screen centre. |
| `zoom` | 1 = actual size, 2 = twice as big. 0 is treated as 1. |
| `rotation` | degrees |

Draw the HUD after `pix_end_2d()`, so it stays fixed on screen while the
world scrolls.

A camera that follows smoothly instead of snapping:

```cpp
cam.target += (player - cam.target) * (1.0f - expf(-6.0f * pix_dt()));   // larger 6 = tighter
cam.zoom = clampf(cam.zoom + pix_mouse_wheel() * 0.1f, 0.3f, 3.0f);
```

### Screen and world coordinates

The mouse position is in screen pixels. To find which world point is under
the mouse, undo the camera:

```cpp
vec2 screen_to_world(const pix_camera2d& cam, vec2 screen) {
    float zoom = cam.zoom != 0.0f ? cam.zoom : 1.0f;
    vec2 d = (screen - cam.offset) / zoom;
    return cam.target + v2rotate(d, -cam.rotation * PIX_DEG2RAD);
}

vec2 world_to_screen(const pix_camera2d& cam, vec2 world) {
    float zoom = cam.zoom != 0.0f ? cam.zoom : 1.0f;
    return cam.offset + v2rotate((world - cam.target) * zoom, cam.rotation * PIX_DEG2RAD);
}

vec2 mouse_world = screen_to_world(cam, pix_mouse_position());
```

## Clipping

To keep drawing inside a rectangle (a scrolling list, a minimap frame), use
clipping. The rectangle is in screen pixels:

```cpp
pix_begin_clip({ 20, 100, 300, 200 });
    draw_inventory_list(scroll);       // anything outside the rectangle is cut off
pix_end_clip();
```

## 2D on top of 3D

In a 3D game, draw the 3D scene first, then 2D:

```cpp
pix_begin_3d(cam3d);
// ... 3D ...
pix_end_3d();

pix_draw_rect_rounded({ 12, 12, 300, 60 }, 10, pix_rgba(0, 0, 0, 140));
pix_draw_text("Quest: find the key", 24, 28, 24, PIX_WHITE);
```

Anything 2D drawn before `pix_end_3d()` is covered by the 3D scene.

To show a 3D render target (such as a minimap rendered from above) as a 2D
rectangle, use `pix_draw_target(target, dst_rect)`. See
`examples/pipeline.cpp`.

## Lower level: `pix_batch2d`

The `pix_draw_*` functions are wrappers over a `pix_batch2d`, which you can
get with `pix_get_batch2d()` or create yourself. The batch versions have the
same names with `pix_batch_` instead of `pix_draw_` and take the batch first,
plus a few extras: `pix_batch_rect_pro` (a rotated rectangle) and
`pix_batch_polygon` (a filled convex polygon).

```cpp
pix_batch2d& b = pix_get_batch2d();
vec2 pts[5] = { {100,100}, {160,80}, {200,130}, {170,190}, {110,180} };
pix_batch_polygon(b, pts, 5, PIX_ORANGE);
pix_batch_rect_pro(b, { 300, 200, 80, 20 }, v2(40, 10), 30.0f, PIX_GREEN);   // rotated 30 degrees about its centre
```

pix also has retained sprite and text batches (`sprite_batch` in
`core/sprite.hpp`, `pixi_text` in `core/text.hpp`) that keep their vertices
on the GPU between frames. They are lower-level and need your own shader and
projection. Most games don't need them.

## Reference

### App layer

| function | |
|---|---|
| `pix_clear(color)` | fill the window |
| `pix_draw_rect(x, y, w, h, color)` / `pix_draw_rect(rect, color)` | filled rectangle |
| `pix_draw_rect_lines(rect, thickness, color)` | outline |
| `pix_draw_rect_rounded(rect, radius, color)` | |
| `pix_draw_rect_gradient(rect, top, bottom)` | vertical gradient |
| `pix_draw_circle(center, radius, color)` | |
| `pix_draw_circle_lines(center, radius, thickness, color)` | |
| `pix_draw_line(a, b, thickness, color)` | |
| `pix_draw_triangle(a, b, c, color)` | |
| `pix_draw_texture(texture, x, y, tint = WHITE)` | |
| `pix_draw_texture_pro(texture, src, dst, origin, rotation_deg, tint = WHITE)` | |
| `pix_draw_text(text, x, y, size, color)` | default font |
| `pix_draw_text_font(font, text, x, y, size, color)` | |
| `pix_draw_textf(x, y, size, color, fmt, ...)` | printf-style |
| `vec2 pix_measure_text(text, size)` | width and height, multi-line aware |
| `pix_draw_fps(x, y)` | |
| `pix_begin_2d(camera)`, `pix_end_2d()` | world-space drawing |
| `pix_begin_clip(rect)`, `pix_end_clip()` | |
| `pix_draw_target(framebuffer, dst, tint = WHITE)` | show a render target |
| `font_data* pix_load_font(ttf_path)`, `pix_unload_font(font)`, `pix_default_font()` | |

### Textures (`core/texture.hpp`)

| function | |
|---|---|
| `pix_texture pix_load_texture(path, flags = TEXTURE_BILINEAR)` | PNG; `id == 0` on failure |
| `pix_texture pix_texture_from_pixels(pixels, w, h, channels, flags)` | from your own 8-bit pixel data, rows top to bottom |
| `pix_unload_texture(texture)` | |
| `unsigned char* pix_load_image_rgba(path, &w, &h, mem = PIX_MEM_GLOBAL)` | decode to memory without the GPU (heightmaps, collision masks); free with `pix_free`, or pass `PIX_MEM_SCOPED` ([Memory](memory.md)) |

### Batch (`core/draw2d.hpp`)

| function | |
|---|---|
| `pix_batch2d_create(b, capacity)`, `pix_batch2d_destroy(b)` | |
| `pix_batch2d_begin(b, width, height)` | start screen-space drawing into a target this size |
| `pix_batch2d_set_camera(b, &cam)` | `nullptr` returns to screen space |
| `pix_batch2d_scissor(b, rect)` | `w <= 0` turns clipping off |
| `pix_batch2d_flush(b)` | send what is pending to the GPU |
| `pix_batch_rect`, `_rect_gradient`, `_rect_lines`, `_rect_rounded`, `_rect_pro`, `_line`, `_circle`, `_circle_lines`, `_triangle`, `_polygon` | shapes |
| `pix_batch_texture`, `pix_batch_texture_pro` | images |
| `pix_batch_text(b, font, text, x, y, size, color)`, `pix_batch_text_ex(b, font, text, pos, style, color)` | text |
| `pix_text_style(size)` | a `pix_text_style2d` with defaults |
| `pix_measure_text(font, text, size, spacing = 0)` | |
