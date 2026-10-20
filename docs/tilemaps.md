# Tilemaps

A tilemap builds a level from a grid of small images (**tiles**): ground,
walls, water, decorations. pix draws tilemaps in **layers**, can load maps
made in the [Tiled](https://www.mapeditor.org/) editor, picks the right edge
and corner tiles for you (**autotiling**), and scrolls background layers at
their own speed (**parallax**).

Tilemaps are drawn through the 2D camera. If you haven't used one yet, read
[2D cameras](2d-rendering.md#2d-camera) first.

## Loading a Tiled map

Make your map in Tiled. In Tiled's preferences, set the map format to
**JSON** and save the map as `.tmj` or `.json`. Then:

```cpp
pix_tilemap level = pix_load_tilemap("assets/level1.tmj");    // once
if (level.width == 0) { /* failed: the console says why */ }

pix_camera2d cam = {};
cam.zoom = 2.0f;

while (pix_frame()) {
    cam.target = player_pos;
    cam.offset = v2(pix_width() * 0.5f, pix_height() * 0.5f);

    pix_clear(level.background);         // the map's background colour from Tiled
    pix_begin_2d(cam);
    pix_draw_tilemap(level);             // every visible layer, in order
    draw_player();
    pix_end_2d();
}

pix_unload_tilemap(level);
```

What comes across from Tiled:

| | |
|---|---|
| tilesets | embedded or external (`.tsj`), image sheets (with margin and spacing) or image collections. They are all packed into one texture when the map loads. |
| tile layers | CSV, base64, zlib and gzip data; flipped and rotated tiles |
| tile animations | animated tiles (water, torches) play by themselves |
| layer settings | offset, parallax factor, opacity, tint colour, visibility; groups combine them |
| image layers | backgrounds, with parallax and repeat |
| object layers | objects with name, class/type, position, size, rotation, tile and custom properties |
| map | size, tile size, background colour |

pix reads orthogonal, fixed-size maps. Infinite maps and isometric or
hexagonal maps are refused with a message. zstd-compressed layers aren't
supported; use CSV, base64, zlib or gzip in the map's properties.

### Drawing sprites between layers

`pix_draw_tilemap` draws every layer. To put characters between the ground
and the treetops, draw the layers yourself:

```cpp
int ground = pix_tilemap_find_layer(level, "ground");
int trees  = pix_tilemap_find_layer(level, "treetops");

pix_begin_2d(cam);
pix_draw_tilemap_layer(level, ground);
draw_characters();
pix_draw_tilemap_layer(level, trees);      // in front of the characters
pix_end_2d();
```

### Objects: spawn points, doors, triggers

Object layers are for placing things that aren't tiles. Give objects names,
types (Tiled calls them classes) and custom properties in Tiled, then read
them when the level starts:

```cpp
for (int i = 0; i < level.object_count; i++) {
    const pix_map_object& o = level.objects[i];
    if (strcmp(o.type, "enemy") == 0) {
        const char* hp = pix_tilemap_object_property(level, o, "hp");   // text, or null if not set
        spawn_enemy(v2(o.x, o.y), hp ? atoi(hp) : 3);
    }
}
int s = pix_tilemap_find_object(level, "player_start");
if (s >= 0) player_pos = v2(level.objects[s].x, level.objects[s].y);
```

Positions are in world units (pixels, unless you scale the map). As in Tiled,
a tile object's `y` is its **bottom** edge, and other objects' `y` is their
top.

## Building a map in code

For generated levels, or simple games without an editor:

```cpp
pix_texture tex = pix_load_texture("assets/tiles.png", TEXTURE_PIXELATED);
pix_rect cells[256];
pix_atlas tiles = pix_create_atlas(tex, cells, pix_rects_grid(cells, 256, tex.width, tex.height, 16, 16));   // the tileset
pix_tilemap map = pix_create_tilemap(100, 60, 16, 16, tiles);        // 100x60 tiles, 16 units each
int ground = pix_tilemap_add_layer(map, "ground");

pix_tilemap_fill(map, ground, 0, 50, 100, 10, 1);      // x, y, w, h, tile
pix_tilemap_set(map, ground, 10, 49, 7);               // one tile
pix_tilemap_set(map, ground, 11, 49, 7, PIX_TILE_FLIP_X);   // mirrored
pix_tilemap_set(map, ground, 12, 49, -1);              // -1 clears a cell
```

A map made with `pix_create_tilemap` borrows the atlas you give it. Keep the
atlas (and its texture) alive until the map is unloaded, and unload them
yourself afterwards.

Tile numbers are frames of the tileset atlas, counted from 0: left to right,
top to bottom for a grid.

Flips: `PIX_TILE_FLIP_X`, `PIX_TILE_FLIP_Y` and `PIX_TILE_FLIP_D` (the
diagonal). Combining the diagonal with X turns a tile 90° clockwise, and with
Y turns it 90° counter-clockwise.

### Changing the map while the game runs

Calling `pix_tilemap_set` during play is cheap. It changes one cell and marks
its 32×32 chunk, and the chunk is re-uploaded once, the next time it is on
screen. Breaking blocks, opening doors and growing crops cost almost nothing.

### Reading the map

```cpp
int tx, ty;
if (pix_tilemap_tile_at(level, mouse_world, &tx, &ty)) {     // the cell under a world point
    int t = pix_tilemap_get(level, ground, tx, ty);          // -1 = empty
    pix_rect cell = pix_tilemap_tile_rect(level, tx, ty);    // its rectangle in the world
}
```

For collisions, a simple approach is to check the cells a character's
rectangle overlaps for non-empty tiles on a "solid" layer.

### Animated tiles

Tiles animated in Tiled play automatically. In code:

```cpp
int frames[3] = { 20, 21, 22 };
float seconds[3] = { 0.2f, 0.2f, 0.3f };
pix_tilemap_animate_tile(map, 20, frames, seconds, 3);       // tile 20 now cycles 20, 21, 22
```

Every cell using tile 20 animates, at no extra cost per cell.

## Autotiling

Painting terrain by hand means choosing the right edge, corner and end tile
for every cell. With autotiling you only say where the terrain is, and pix
picks each tile from its neighbours:

```cpp
pix_tilemap_set_terrain(map, 1, pix_autotile_make(PIX_AUTOTILE_47, 64));   // terrain 1: 47 tiles from tile 64
pix_tilemap_paint(map, ground, x, y, 1);       // paint grass here; it and its neighbours re-tile
pix_tilemap_paint(map, ground, x, y, 0);       // erase
```

There are two standard tile sets:

| mode | tiles | looks at |
|---|---|---|
| `PIX_AUTOTILE_16` | 16 | the 4 edges (north, east, south, west). Good for paths, pipes, walls. |
| `PIX_AUTOTILE_47` | 47 | edges and corners (the "blob" set). Good for terrain with inner corners: grass, water, cliffs. |

`pix_autotile_make(mode, first_tile)` expects the tiles in a fixed order,
starting at `first_tile`. To find out which arrangement each tile must show,
`pix_autotile_masks` lists them:

```cpp
uint8_t masks[47];
int n = pix_autotile_masks(PIX_AUTOTILE_47, masks);
// tile first_tile + i shows the terrain connected toward the neighbours set in masks[i]:
// N 1, NE 2, E 4, SE 8, S 16, SW 32, W 64, NW 128
```

For the 16-tile set, tile `first_tile + m` uses the edge mask `m`, with N 1,
E 2, S 4 and W 8.

If your art is laid out differently, fill the table yourself: `rules.table[mask]`
is the tile for each of the 256 neighbour masks. For the 47-tile set,
corners only count when both edges beside them are set.

Up to 15 terrains can be used (1 to 15), each with its own rules. A terrain
connects only to itself, and the edge of the map counts as connected, so
terrain runs cleanly off the side.

## Parallax layers

A background that scrolls slower than the world looks far away. Image layers
from Tiled do this automatically from their parallax settings. Anywhere else,
draw one with `pix_draw_parallax`:

```cpp
pix_texture sky = pix_load_texture("assets/sky.png");
pix_texture hills = pix_load_texture("assets/hills.png", TEXTURE_PIXELATED);

pix_begin_2d(cam);
pix_draw_parallax(sky, v2(0.0f, 0.0f));                      // never moves
pix_draw_parallax(hills, v2(0.3f, 0.3f), v2(0, 120));        // a third of the camera's speed, 120 units down
pix_draw_tilemap(level);                                     // the world, at full speed
pix_end_2d();
```

`factor` is how much the layer follows the camera: 0 stays fixed, 1 moves with
the world. The image repeats across the view horizontally by default (pass
`repeat_x` / `repeat_y`), and the repetition happens on the GPU, so a whole
endless background costs one quad. Use a texture of its own for this, not one
packed into an atlas. A tile layer with a parallax factor (set in Tiled, or
`map.layers[i].parallax`) scrolls the same way.

## Performance

Tilemaps are built to stay cheap at any size:

- **Each tile is 8 bytes on the GPU.** The vertex shader turns it into a
  quad. Nothing is rebuilt per frame: a static map costs no CPU work beyond
  deciding what's on screen.
- **Only what's on screen is drawn.** The map is split into 32×32 chunks, and
  only chunks overlapping the camera's view are drawn, one draw call each.
- **Edits are lazy.** Changing a tile re-uploads its chunk once, the next
  time it's visible.
- **Animated tiles change one table entry,** not the cells that use them.

A 1024×1024 map (a million tiles) drawn through a 640×300 view takes 0.85 ms
and 6 draw calls on an Intel laptop GPU.

Seams between tiles: tiles from the Tiled loader and the atlas builder have
1-pixel extruded borders, so they never bleed into each other, even zoomed
or with smooth filtering. A grid atlas without borders is fine with
`TEXTURE_PIXELATED` at whole-number zoom levels. With smooth filtering or
fractional zoom, pack it with the [atlas builder](2d-rendering.md#packing-loose-images-into-one-atlas)
(`pix_atlas_builder_add_sheet`).

## Reference

| function | |
|---|---|
| `pix_tilemap pix_load_tilemap(path, flags = TEXTURE_PIXELATED)` | a Tiled JSON map; `width == 0` on failure |
| `pix_tilemap pix_create_tilemap(w, h, tile_w, tile_h, tileset)` | an empty map in code; borrows the atlas |
| `pix_unload_tilemap(map)` | |
| `int pix_tilemap_add_layer(map, name)` | a tile layer |
| `int pix_tilemap_add_image_layer(map, name, texture, parallax, repeat_x, repeat_y)` | a background; the texture stays yours |
| `int pix_tilemap_find_layer(map, name)` | -1 if missing |
| `pix_tilemap_set(map, layer, x, y, tile, flags = 0)` | tile -1 clears |
| `pix_tilemap_fill(map, layer, x, y, w, h, tile)` | |
| `int pix_tilemap_get(map, layer, x, y)` | -1 empty or outside |
| `bool pix_tilemap_tile_at(map, world, &tx, &ty)` | false outside the map |
| `pix_rect pix_tilemap_tile_rect(map, tx, ty)` | |
| `pix_tilemap_animate_tile(map, tile, frames, seconds, count)` | |
| `pix_autotile pix_autotile_make(mode, first_tile)` | `PIX_AUTOTILE_16` or `_47` |
| `int pix_autotile_masks(mode, out)` | the masks in tile order |
| `pix_tilemap_set_terrain(map, terrain, rules)` | terrain 1..15 |
| `pix_tilemap_paint(map, layer, x, y, terrain)` | 0 erases |
| `int pix_tilemap_terrain(map, layer, x, y)` | |
| `int pix_tilemap_find_object(map, name)` | index into `map.objects` |
| `const char* pix_tilemap_object_property(map, object, key)` | text or null |
| `pix_draw_tilemap(map)`, `pix_draw_tilemap_layer(map, layer)` | |
| `pix_draw_parallax(texture, factor, offset = 0, scale = 1, repeat_x = true, repeat_y = false, tint = WHITE)` | |
| `pix_batch_tilemap(batch, map, time)`, `pix_batch_tilemap_layer(batch, map, layer, time)`, `pix_batch_parallax(...)` | lower level |

`pix_tilemap` fields you may read or change: `width`, `height`, `tile_w`,
`tile_h`, `background`, `layers[i]` (`name`, `visible`, `offset`, `parallax`,
`tint`), `objects` and `object_count`, `tileset`.
