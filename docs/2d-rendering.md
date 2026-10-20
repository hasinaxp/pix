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
the screen. `pix_width()` and `pix_height()` give the size you draw in. That
is the window's size, which can change if the window is resizable, unless you
fixed the resolution with [`pix_set_resolution`](#fixed-resolution-and-pixel-art).

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

More shapes:

```cpp
pix_draw_ellipse(v2(100, 80), 60, 30, PIX_ORANGE);                 // centre, x radius, y radius
pix_draw_ellipse_lines(v2(100, 80), 60, 30, 3, PIX_WHITE);
pix_draw_rect_rounded_lines(box, 12, 3, PIX_WHITE);                // rounded outline
pix_draw_rect_gradient_h(box, PIX_RED, PIX_YELLOW);                // left colour, right colour

pix_draw_ring(v2(300, 80), 20, 30, -90, 180, PIX_GREEN);           // inner, outer radius, from, to (degrees)
pix_draw_circle_sector(v2(400, 80), 30, 200, 340, PIX_YELLOW);     // a pie slice

vec2 path[4] = { {20, 200}, {80, 150}, {140, 200}, {200, 150} };
pix_draw_polyline(path, 4, 6, PIX_PURPLE);                         // joined segments, clean corners
pix_draw_polyline(path, 4, 6, PIX_PURPLE, /*closed*/ true);        // back to the start
pix_draw_polygon(hexagon, 6, PIX_RED);                             // filled, convex, points in order
```

Angles are in **degrees**. 0 points right, and angles grow clockwise on
screen, because Y points down. A ring from -90 to 180 starts at the top and
sweeps three quarters of the way round, which makes a cooldown or health
gauge:

```cpp
float left = cooldown / cooldown_max;                              // 1 .. 0
pix_draw_ring(icon_centre, 18, 24, -90, -90 + 360 * left, pix_rgba(255, 255, 255, 180));
```

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

### Sprites and animation

For characters, items and effects, use an **atlas**: a texture plus the
rectangles of the sprites in it. Each rectangle becomes a **frame**, and
animations are sequences of frames. Everything drawn from one atlas shares a
texture, so it costs a single draw call.

#### Making an atlas

You give pix the texture and an array of rectangles, in pixels of that
texture. Frame `i` is `rects[i]`:

```cpp
pix_texture tex = pix_load_texture("assets/hero.png", TEXTURE_PIXELATED);   // once

pix_rect rects[3] = {
    {  0, 0, 32, 32 },          // frame 0
    { 32, 0, 32, 32 },          // frame 1
    { 64, 0, 32, 48 },          // frame 2: any size, anywhere in the image
};
pix_atlas hero = pix_create_atlas(tex, rects, 3);
```

Most sprite sheets are regular, so helpers fill the array for you. Each
helper writes into your array, stops at its size, and returns how many
rectangles it wrote:

```cpp
pix_rect rects[64];

// equal cells, left to right then top to bottom
int n = pix_rects_grid(rects, 64, tex.width, tex.height, 32, 32);
// the same with a 1-pixel margin around the sheet and 2 pixels between cells
n = pix_rects_grid(rects, 64, tex.width, tex.height, 32, 32, 1, 2);

// one animation strip: 6 cells of 32x32 starting at (0, 64)
n = pix_rects_row(rects, 64, 0, 64, 32, 32, 6);
// strips of different sizes, one after another in the same array
int k = pix_rects_row(rects, 64, 0, 0, 32, 32, 8);            // frames 0..7: walk
k += pix_rects_row(rects + k, 64 - k, 0, 32, 48, 32, 4);      // frames 8..11: wide attack
// pix_rects_column takes the same arguments and goes downwards

pix_atlas hero = pix_create_atlas(tex, rects, k);
```

The texture stays yours. `pix_unload_atlas` frees the frames and animations,
not the texture.

Then define animations as runs of frames, or as any list of frames:

```cpp
pix_atlas_add_anim(hero, "walk", 0, 8, 12.0f);                   // frames 0..7 at 12 fps
pix_atlas_add_anim(hero, "attack", 8, 4, 16.0f, PIX_ANIM_ONCE);
int order[4] = { 0, 1, 2, 1 };
pix_atlas_add_anim_frames(hero, "idle", order, 4, 6.0f);         // any order, repeats allowed
pix_atlas_set_pivot(hero, v2(0.5f, 1.0f));                       // draw from the feet (default: centre)
```

And play them:

```cpp
pix_sprite_player anim = {};                           // one per animated thing

// each frame:
pix_sprite_play(anim, hero, moving ? "walk" : "idle");  // switches only when it changes
pix_sprite_update(anim, hero, pix_dt());
pix_draw_sprite_anim(hero, anim, player_pos, 3.0f, 0, PIX_WHITE, facing_left ? PIX_FLIP_X : 0);
pix_draw_sprite(hero, 2, v2(100, 100), 2.0f);           // or one frame, by index
```

#### Rectangles from Aseprite or TexturePacker

If your sprites come from Aseprite or TexturePacker, their JSON export
already lists the rectangles. In Aseprite, use *File → Export Sprite Sheet*
with "JSON Data" ticked, in either Hash or Array format. A helper reads the
rectangles into your array the same way, and a second call adds what else
the file knows:

```cpp
pix_rect rects[256];
char image[256];
int n = pix_rects_from_json("assets/hero.json", rects, 256, image, sizeof(image));
pix_texture tex = pix_load_texture("assets/hero.png", TEXTURE_PIXELATED);   // `image` holds the name the file gives
pix_atlas hero = pix_create_atlas(tex, rects, n);
pix_atlas_apply_json(hero, "assets/hero.json");        // names, trimming, durations, tags
```

What `pix_atlas_apply_json` adds:

- **Tags become animations.** A tag called "run" plays with
  `pix_sprite_play(anim, hero, "run")`. Forward, reverse and ping-pong
  directions are honoured.
- **Per-frame durations** from Aseprite set the timing, so a held pose stays
  up for as long as the artist set.
- **Trimmed frames** (empty borders cut off to save space) are drawn exactly
  where the untrimmed frame would be. Nothing jitters.
- **Frame names** work with `pix_draw_sprite(hero, "sword", pos)`.
  TexturePacker's pivots and `"animations"` lists come across too.

#### Packing loose images into one atlas

Many separate PNG files mean many textures. The atlas builder packs them into
one when the game loads:

```cpp
pix_atlas_builder b = {};
pix_atlas_builder_add_file(b, "coin", "assets/coin.png");
pix_atlas_builder_add_file(b, "heart", "assets/heart.png");
pix_atlas_builder_add_sheet(b, "slime", "assets/slime.png", 24, 24, 8.0f);  // frames slime_0.. + animation "slime"
pix_atlas items = pix_atlas_build(b);                                        // one texture

pix_draw_sprite(items, "heart", v2(20, 20));
```

Images are packed tightly (largest first, each going wherever it sits
lowest), with a small gap between them. The gap is filled with each image's
own edge pixels, so smooth scaling never pulls in a neighbour's colour.

#### Playback

`pix_sprite_player` is a playhead. A zero-initialised one is ready to use.

| | |
|---|---|
| `pix_sprite_play(p, atlas, "name")` | start an animation; calling it again with the same name keeps playing (pass `true` to restart) |
| `pix_sprite_update(p, atlas, dt)` | advance the time |
| `p.speed` | playback rate (0 counts as 1, 2 is double speed) |
| `p.paused` | hold the current frame |
| `p.finished` | a play-once animation reached its end |
| `p.loops` | how many times it has wrapped round |
| `pix_sprite_entered(p, step)` | true if the last update reached that step of the animation |

Play modes, the last argument of `pix_atlas_add_anim`: `PIX_ANIM_LOOP` (the
default), `PIX_ANIM_ONCE` (stops on the last frame, for deaths and attacks),
`PIX_ANIM_PINGPONG` (forward then back).

Sounds and hits that belong to a frame use `pix_sprite_entered`. It counts
every step passed during the update, even when a slow frame skips several at
once:

```cpp
pix_sprite_update(anim, hero, dt);
if (pix_sprite_entered(anim, 2) || pix_sprite_entered(anim, 6)) pix_play_sound(step_sound);
if (attacking && pix_sprite_entered(anim, 3)) deal_damage();
if (attacking && anim.finished) attacking = false;
```

#### Drawing

```cpp
pix_draw_sprite(atlas, frame_index, pos, scale, rotation_deg, tint, flags);
pix_draw_sprite(atlas, "frame name", pos, ...);       // a name lookup each call
pix_draw_sprite_anim(atlas, player, pos, ...);        // the player's current frame
```

The sprite's **pivot** goes to `pos`, and rotation and flipping happen around
it. `flags` is `PIX_FLIP_X`, `PIX_FLIP_Y` or both.

Every frame lookup by name compares strings. For things drawn thousands of
times a frame, look the index up once with `pix_atlas_find(atlas, "name")`
and draw by index.

Free an atlas with `pix_unload_atlas(atlas)`. It frees the frames and
animations. The texture you passed to `pix_create_atlas` is still yours to
unload; an atlas made by `pix_atlas_build` frees its own texture.

### Nine-slice panels

UI panels, buttons and speech bubbles have to stretch to any size without
their frame getting thick or blurry. Nine-slice drawing keeps the corners at
their real size, stretches the edges in one direction only, and stretches the
middle in both:

```cpp
pix_texture panel = pix_load_texture("assets/ui/panel.png", TEXTURE_PIXELATED);   // e.g. 48x48
pix_rect src = { 0, 0, 48, 48 };                        // the panel's part of the image
vec4 border = v4(8, 8, 8, 8);                           // frame width: left, top, right, bottom (source pixels)
pix_draw_nine_slice(panel, src, border, { 100, 100, 400, 160 });
pix_draw_nine_slice(panel, src, border, button_rect, hovered ? PIX_YELLOW : PIX_WHITE);   // tinted
```

If the destination is smaller than the borders, they shrink together, so a
panel never turns inside out.

### Keeping it fast

All 2D drawing goes into one batch. The batch goes to the GPU as a single
draw call at the end of the frame, or earlier only when it has to:

- **Up to 16 textures share one draw call.** Sprites from several sheets,
  text and shapes can be drawn in any order. Shapes don't use a texture slot
  at all.
- A new draw call starts when a 17th texture appears, or when you change the
  blend mode, the camera or the clip rectangle.
- Quads take 4 vertices instead of 6, and each shape reserves its space once,
  so drawing many small things stays cheap.

You don't have to sort anything. Interleaving sprites from 5 textures with
text and circles still costs one draw call. 20,000 sprites cycling through 12
different textures take 2 draw calls and about 4.5 ms on an Intel laptop GPU.

For tens of thousands of sprites, keep the number of distinct images low by
putting small images into an [atlas](#sprites-and-animation), and skip what is
off screen (see [the camera's visible rectangle](#what-is-on-screen)).

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

The mouse is in screen pixels and your world is in world coordinates. These
convert between the two:

```cpp
vec2 aim   = pix_mouse_world_2d(cam);                          // the world point under the mouse
vec2 world = pix_screen_to_world_2d(cam, screen_point);
vec2 label = pix_world_to_screen_2d(cam, enemy_pos);           // e.g. to draw a name tag in the HUD
```

### What is on screen

`pix_camera2d_visible(cam)` returns the world rectangle the camera currently
shows (its bounding box, if the camera is rotated). Skip anything outside
it. For big worlds this is the single biggest saving:

```cpp
pix_rect view = pix_camera2d_visible(cam);
for (int i = 0; i < enemy_count; i++)
    if (pix_rect_overlaps(view, enemies[i].bounds)) draw_enemy(enemies[i]);
```

`pix_rect_overlaps(a, b)` and `pix_rect_contains(rect, point)` are handy for
UI hit tests too.

### Screen shake

Hits, explosions and landings feel stronger with a shake. pix uses
**trauma**: each hit adds some (0 to 1), it drains over time, and the shake
is trauma squared. Small hits barely nudge the screen and big ones slam it.
The movement follows smooth noise rather than random jumps, so it reads as a
shake and not as a glitch.

```cpp
static pix_shake shake;                        // all zero = good defaults

if (player_hit)   pix_shake_add(shake, 0.3f);
if (boss_slammed) pix_shake_add(shake, 0.8f);

pix_begin_2d(pix_shake_camera(shake, cam, pix_dt()));   // the camera, shaken
draw_world();
pix_end_2d();
```

To tune it, set the fields: `max_offset` (pixels at full trauma, default 20),
`max_angle` (degrees, 4), `decay` (trauma lost per second, 1.2) and
`frequency` (how fast it wobbles, 18). For a 3D camera, call
`pix_shake_update(shake, dt)` and use `shake.offset` and `shake.angle`
yourself.

## Clipping

To keep drawing inside a rectangle (a scrolling list, a minimap frame), use
clipping. The rectangle is in screen pixels:

```cpp
pix_begin_clip({ 20, 100, 300, 200 });
    draw_inventory_list(scroll);       // anything outside the rectangle is cut off
pix_end_clip();
```

## Parallax backgrounds and tilemaps

A background layer that scrolls slower than the world:

```cpp
pix_begin_2d(cam);
pix_draw_parallax(sky, v2(0, 0));                    // fixed
pix_draw_parallax(hills, v2(0.3f, 0.3f), v2(0, 120));  // a third of the camera's speed, repeating sideways
draw_world();
pix_end_2d();
```

Levels built from tiles, including maps made in the Tiled editor, are covered
on the [Tilemaps](tilemaps.md) page: layers, autotiling, animated tiles,
objects and parallax tile layers.

## Blend modes

Blending decides how a colour combines with what is already drawn. The
default is normal transparency (alpha). Two others are the classic tools for
light and shadow:

```cpp
pix_begin_blend(PIX_BLEND_ADDITIVE);          // adds light: overlaps get brighter
draw_sparks();  draw_lasers();  draw_glow();
pix_begin_blend(PIX_BLEND_MULTIPLY);          // darkens: shadows, night tint
pix_draw_rect({ 0, 0, (float)pix_width(), (float)pix_height() }, pix_rgba(80, 90, 160));   // dusk
pix_end_blend();                              // back to normal
```

| mode | effect |
|---|---|
| `PIX_BLEND_ALPHA` | normal transparency (the default) |
| `PIX_BLEND_ADDITIVE` | adds colour. Overlapping red, green and blue make white. For fire, magic, lasers, glows. |
| `PIX_BLEND_MULTIPLY` | multiplies. White changes nothing, darker colours darken. For shadows, tinted glass, day/night. |
| `PIX_BLEND_PREMULTIPLIED` | for images whose colour is already multiplied by alpha |

Changing the blend mode starts a new draw call, so group the additive things
together rather than switching for every sprite.

## Canvases: drawing into an image

A canvas is an image you draw into with the normal 2D calls, then draw like
any texture. Use it for a minimap, a background you build once and reuse, a
paint layer, trails that fade, or a picture-in-picture view:

```cpp
framebuffer map = pix_create_canvas(256, 256);          // once

// whenever it changes (or every frame):
pix_begin_canvas(map);
    pix_clear(PIX_BLANK);                               // clears the canvas, not the screen
    for (...) pix_draw_rect(tile_rect, tile_color);     // draw into it
pix_end_canvas();

pix_draw_target(map, { 20, 20, 256, 256 });             // show it, as often as you like
pix_draw_target(map, { 300, 20, 128, 128 }, pix_rgba(255, 255, 255, 128));   // smaller, half see-through

pix_destroy_canvas(map);                                // when done
```

Things to know:

- Coordinates inside the canvas start at its own top-left corner.
- `pix_clear` and every 2D call (cameras and clipping included) apply to the
  canvas until `pix_end_canvas`.
- Canvases don't nest: end one before beginning another.
- Pass `TEXTURE_PIXELATED` as the third argument of `pix_create_canvas` for
  crisp pixels when the canvas is drawn larger.
- `pix_draw_target` handles the canvas's transparency correctly. To draw a
  canvas with `pix_draw_texture_pro` (to rotate it, say), get a texture with
  `pix_canvas_texture(map)` and draw it inside
  `pix_begin_blend(PIX_BLEND_PREMULTIPLIED)`.
- Drawing a canvas only costs one textured quad, so a complex background
  drawn once into a canvas is far cheaper than redrawing it every frame.

## Fixed resolution and pixel art

Pixel-art games are drawn at a small fixed resolution and scaled up with
every pixel kept square. One call sets that up:

```cpp
pix_init("my game", 1280, 720);
pix_set_resolution(320, 180);       // the game now draws in 320 x 180
```

From then on everything works in those 320×180 pixels: 2D drawing,
`pix_width()` and `pix_height()`, the mouse position, and 3D scenes, which
render at that resolution too, for the chunky retro look. pix scales the
result up to the window by the largest **whole number** that fits, so every
pixel is the same size, and fills the rest with black bars. Resizing the
window keeps it right.

| mode | |
|---|---|
| `PIX_SCALE_INTEGER` (default) | whole multiples only: pixel-perfect, bars around |
| `PIX_SCALE_FIT` | as large as fits while keeping the shape; pixels may differ by one |
| `\| PIX_SCALE_SMOOTH` | add to either for smooth filtering instead of crisp pixels |

```cpp
pix_set_resolution(640, 360, PIX_SCALE_FIT | PIX_SCALE_SMOOTH);   // a fixed-size HD-ish game
pix_set_resolution(0, 0);                                         // back to the window's size
```

Load pixel-art images with `TEXTURE_PIXELATED`, and keep positions on whole
pixels (for example `floorf(x)`) if you want the sprites to move in
pixel-sized steps.

The scaling is one GPU copy per frame, so it costs almost nothing.

## Screen effects in 2D

The post effects from 3D games (`pix_add_effect`: CRT, pixelate, chromatic
aberration, grayscale, sepia, or your own shader) work in 2D games as well:

```cpp
int crt = pix_add_effect(PIX_EFFECT_CRT, v4(0.5f, 0.06f, 0, 0));    // once

// each frame:
draw_world();
pix_apply_effects();          // optional: run them now, so the HUD below stays clean
draw_hud();
```

Without `pix_apply_effects()`, the effects run over the finished frame,
including the HUD. In a 3D game they run inside `pix_end_3d()`. Effects that
read depth (outline, depth blur) have nothing to work with in 2D. See
[3D rendering](3d-rendering.md#screen-effects) for the list of effects and
their parameters.

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
`pix_batch_rect_colors` (a colour per corner).

```cpp
pix_batch2d& b = pix_get_batch2d();
pix_batch_rect_pro(b, { 300, 200, 80, 20 }, v2(40, 10), 30.0f, PIX_GREEN);   // rotated 30 degrees about its centre
pix_batch_rect_colors(b, { 0, 0, 200, 100 }, PIX_RED, PIX_GREEN, PIX_BLUE, PIX_YELLOW);
int calls = b.draws;                              // draw calls since the frame began
```

pix also has retained sprite and text batches (`sprite_batch` in
`core/sprite.hpp`, `pixi_text` in `core/text.hpp`) that keep their vertices
on the GPU between frames. They are lower-level and need your own shader and
projection. Most games don't need them.

## Reference

### App layer

| function | |
|---|---|
| `pix_clear(color)` | fill what you are drawing into: the window, the fixed-resolution image, or a canvas |
| `pix_draw_rect(x, y, w, h, color)` / `pix_draw_rect(rect, color)` | filled rectangle |
| `pix_draw_rect_lines(rect, thickness, color)` | outline |
| `pix_draw_rect_rounded(rect, radius, color)`, `pix_draw_rect_rounded_lines(rect, radius, thickness, color)` | |
| `pix_draw_rect_gradient(rect, top, bottom)`, `pix_draw_rect_gradient_h(rect, left, right)` | |
| `pix_draw_circle(center, radius, color)`, `pix_draw_circle_lines(center, radius, thickness, color)` | |
| `pix_draw_ellipse(center, rx, ry, color)`, `pix_draw_ellipse_lines(center, rx, ry, thickness, color)` | |
| `pix_draw_ring(center, inner, outer, start_deg, end_deg, color)` | arcs and gauges |
| `pix_draw_circle_sector(center, radius, start_deg, end_deg, color)` | pie slice |
| `pix_draw_line(a, b, thickness, color)` | |
| `pix_draw_polyline(points, count, thickness, color, closed = false)` | |
| `pix_draw_triangle(a, b, c, color)`, `pix_draw_polygon(points, count, color)` | polygon: convex |
| `pix_draw_texture(texture, x, y, tint = WHITE)` | |
| `pix_draw_texture_pro(texture, src, dst, origin, rotation_deg, tint = WHITE)` | |
| `pix_draw_nine_slice(texture, src, border, dst, tint = WHITE)` | `border` = left, top, right, bottom |
| `pix_draw_sprite(atlas, frame or "name", pos, scale = 1, rotation_deg = 0, tint = WHITE, flags = 0)` | flags: `PIX_FLIP_X`, `PIX_FLIP_Y` |
| `pix_draw_sprite_anim(atlas, player, pos, scale = 1, rotation_deg = 0, tint = WHITE, flags = 0)` | |
| `pix_begin_blend(mode)`, `pix_end_blend()` | `PIX_BLEND_ALPHA`, `_ADDITIVE`, `_MULTIPLY`, `_PREMULTIPLIED` |
| `pix_mouse_world_2d(cam)`, `pix_camera2d_visible(cam)` | the mouse in world coordinates; the visible world rect |
| `pix_screen_to_world_2d(cam, p)`, `pix_world_to_screen_2d(cam, p)` | |
| `pix_rect_overlaps(a, b)`, `pix_rect_contains(r, p)` | |
| `pix_shake`, `pix_shake_add(s, trauma)`, `pix_shake_camera(s, cam, dt)`, `pix_shake_update(s, dt)` | screen shake |
| `framebuffer pix_create_canvas(w, h, filter = TEXTURE_LINEAR)`, `pix_destroy_canvas(c)` | |
| `pix_begin_canvas(c)`, `pix_end_canvas()`, `pix_canvas_texture(c)` | |
| `pix_set_resolution(w, h, mode = PIX_SCALE_INTEGER)` | fixed resolution, scaled to the window |
| `pix_apply_effects()` | run the screen effects now |
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

### Atlases and sprite animation (`core/atlas.hpp`)

| function | |
|---|---|
| `pix_atlas pix_create_atlas(texture, rects, count)` | frame `i` = `rects[i]`; the texture stays yours |
| `int pix_rects_grid(out, max, image_w, image_h, cell_w, cell_h, margin = 0, spacing = 0)` | equal cells, row by row |
| `int pix_rects_row(out, max, x, y, cell_w, cell_h, count, spacing = 0)` | a strip, rightwards |
| `int pix_rects_column(out, max, x, y, cell_w, cell_h, count, spacing = 0)` | a strip, downwards |
| `int pix_rects_from_json(json_path, out, max, image_out = null, image_cap = 0)` | an Aseprite / TexturePacker export's rects |
| `bool pix_atlas_apply_json(atlas, json_path)` | its names, trimming, pivots, durations and tags |
| `pix_unload_atlas(atlas)` | |
| `int pix_atlas_find(atlas, name)` | frame index, -1 if missing |
| `pix_atlas_set_pivot(atlas, v2(nx, ny))` | every frame's pivot as a fraction of its size |
| `pix_atlas_add_anim(atlas, name, first, count, fps, mode = PIX_ANIM_LOOP)` | a run of frames |
| `pix_atlas_add_anim_frames(atlas, name, frames, count, fps, mode)` | any list of frames |
| `int pix_atlas_find_anim(atlas, name)` | |
| `pix_sprite_play(player, atlas, name, restart = false)`, `pix_sprite_play(player, index, restart)` | |
| `pix_sprite_update(player, atlas, dt)` | |
| `int pix_sprite_frame(player, atlas)` | the frame to draw now |
| `bool pix_sprite_entered(player, step)` | the last update reached that step |
| `pix_atlas_builder_add(b, name, rgba, w, h)`, `_add_file(b, name, png)` | |
| `int pix_atlas_builder_add_sheet(b, name, png, cell_w, cell_h, fps = 10, mode = LOOP, margin = 0, spacing = 0)` | returns the frames added |
| `pix_atlas_build(b, flags = TEXTURE_PIXELATED, max_size = 4096, padding = 1)` | packs; frees the builder's images |
| `pix_batch_sprite(batch, atlas, frame, pos, scale, rotation_deg, tint, flags)` | |

Each frame is a `pix_atlas_frame` (`src`, `offset`, `size`, `pivot`,
`duration`). You can change any of them after loading, for example to give
one frame its own pivot.

### Batch (`core/draw2d.hpp`)

| function | |
|---|---|
| `pix_batch2d_create(b, capacity)`, `pix_batch2d_destroy(b)` | |
| `pix_batch2d_begin(b, width, height, top_down = false)` | start screen-space drawing into a target this size; `top_down` for canvases |
| `pix_batch2d_set_camera(b, &cam)` | `nullptr` returns to screen space |
| `pix_batch2d_scissor(b, rect)` | `w <= 0` turns clipping off |
| `pix_batch2d_set_blend(b, mode)` | |
| `pix_rect pix_batch2d_visible(b)` | the world rectangle on screen right now |
| `pix_batch_parallax(b, texture, factor, offset, scale, repeat_x, repeat_y, tint)` | |
| `pix_batch2d_flush(b)` | send what is pending to the GPU |
| `pix_batch_rect`, `_rect_colors`, `_rect_gradient`, `_rect_lines`, `_rect_rounded`, `_rect_rounded_lines`, `_rect_pro`, `_line`, `_polyline`, `_circle`, `_circle_lines`, `_ellipse`, `_ellipse_lines`, `_ring`, `_triangle`, `_polygon` | shapes |
| `pix_batch_texture`, `pix_batch_texture_pro`, `pix_batch_nine_slice` | images |
| `pix_batch_quad(b, gl_texture, corners[4], uvs[4], tint)` | any textured quad: skewed cards, custom shapes |
| `pix_batch_text(b, font, text, x, y, size, color)`, `pix_batch_text_ex(b, font, text, pos, style, color)` | text |
| `pix_text_style(size)` | a `pix_text_style2d` with defaults |
| `pix_measure_text(font, text, size, spacing = 0)` | |
