// A procedural town from the KayKit City Builder pieces: tile-based roads
// whose pieces are rotated to connect, random buildings, thousands of
// instances drawn in a handful of instanced batches. Uses the lower-level
// loader + renderer calls directly, to show that layer.
//   build.bat city          (needs assets/KayKit_City_Builder)
//
// Controls: right drag orbit, wheel zoom, R regenerates, T advances time.
#define PIX_IMPLEMENTATION
#include "pix.h"

#define ASSET_DIR "assets/KayKit_City_Builder/Assets/obj/"
#define GRID 40
#define CELL 2.0f            // every KayKit road/base tile is 2 x 2

// connection bitmask: bit0 +X, bit1 +Z, bit2 -X, bit3 -Z. Each piece's
// default openings, rotated a quarter turn at a time until they match.
#define ROAD_STRAIGHT_MASK 0x0A
#define ROAD_CORNER_MASK   0x03
#define ROAD_TSPLIT_MASK   0x0B
#define ROAD_JUNCTION_MASK 0x0F

static int popcount4(int m) { return (m & 1) + ((m >> 1) & 1) + ((m >> 2) & 1) + ((m >> 3) & 1); }
static int rotr4(int m, int k) { k &= 3; return ((m >> k) | (m << (4 - k))) & 0xF; }

struct road_set { idx straight, corner, tsplit, junction; };

static idx pick_road(int mask, const road_set& rs, float* rot) {
    int n = popcount4(mask);
    for (int k = 0; k < 4; k++) {
        float a = (float)k * PIX_PI * 0.5f;
        if (n == 4 && rotr4(ROAD_JUNCTION_MASK, k) == mask) { *rot = a; return rs.junction; }
        if (n == 3 && rotr4(ROAD_TSPLIT_MASK,   k) == mask) { *rot = a; return rs.tsplit;   }
        if (n == 2 && rotr4(ROAD_STRAIGHT_MASK, k) == mask) { *rot = a; return rs.straight; }
        if (n == 2 && rotr4(ROAD_CORNER_MASK,   k) == mask) { *rot = a; return rs.corner;   }
    }
    *rot = (mask & 0x05) ? PIX_PI * 0.5f : 0.0f;   // dead end: a straight along its axis
    return rs.straight;
}

static bool is_road(int gx, int gz) {
    if (gx < 0 || gz < 0 || gx >= GRID || gz >= GRID) return false;
    return (gx % 6 == 0) || (gz % 5 == 0);
}

// OBJ -> GPU, keeping nothing on the CPU afterwards
static idx load_piece(pix_renderer& r, pix_data_loader& loader, const char* name) {
    char path[260];
    snprintf(path, sizeof(path), "%s%s.obj", ASSET_DIR, name);
    pix_loader_mark mark = pix_loader_mark_now(loader);
    idx fd = load_mesh_obj_file(loader, path);
    idx mesh = fd == PIX_INVALID ? PIX_INVALID : load_mesh(r, loader.mesh_files[fd]);
    pix_loader_rewind(loader, mark);
    if (mesh == PIX_INVALID) pix_log("missing %s", path);
    return mesh;
}

int main() {
    if (!pix_init("pix - city", 1280, 720)) return 1;
    pix_renderer& r = pix_get_renderer();
    pix_data_loader& loader = pix_get_loader();
    pix_shadows().distance = 80.0f;
    float hour = 16.0f;
    pix_time_of_day(hour);

    const char* building_names[8] = { "building_A", "building_B", "building_C", "building_D",
                                      "building_E", "building_F", "building_G", "building_H" };
    idx buildings[8];
    for (int i = 0; i < 8; i++) buildings[i] = load_piece(r, loader, building_names[i]);
    road_set roads = { load_piece(r, loader, "road_straight"), load_piece(r, loader, "road_corner"),
                       load_piece(r, loader, "road_tsplit"), load_piece(r, loader, "road_junction") };
    idx base = load_piece(r, loader, "base");
    idx tree = load_piece(r, loader, "bush");
    // KayKit's atlas is a grid of colour swatches: linear, no mipmaps, or they bleed
    idx atlas = load_material(r, ASSET_DIR "citybits_texture.png", v3(1, 1, 1), 0.0f, 0.7f, TEXTURE_LINEAR);

    // the town is regenerated into this list; each frame just resubmits it
    pix_render_instance* town = pix_alloc<pix_render_instance>(GRID * GRID * 3);
    int town_count = 0;
    uint32_t seed = 1;
    bool regenerate = true;
    float yaw = 0.7f, pitch = 0.65f, dist = GRID * CELL * 0.6f;

    while (pix_frame()) {
        float dt = pix_dt();
        if (pix_key_pressed(KEY_ESC)) pix_quit();
        if (pix_key_pressed('R')) { seed++; regenerate = true; }
        if (pix_key_down('T')) { hour += dt * 2.0f; pix_time_of_day(hour); }
        if (pix_mouse_down(PIX_MOUSE_RIGHT)) {
            yaw += pix_mouse_delta().x * 0.006f;
            pitch = clampf(pitch + pix_mouse_delta().y * 0.006f, 0.1f, 1.45f);
        }
        dist = clampf(dist - pix_mouse_wheel() * 4.0f, 8.0f, 200.0f);

        if (regenerate) {
            regenerate = false;
            rng g = rng_seed(seed);
            town_count = 0;
            for (int gx = 0; gx < GRID; gx++)
                for (int gz = 0; gz < GRID; gz++) {
                    float x = (gx - GRID * 0.5f) * CELL, z = (gz - GRID * 0.5f) * CELL;
                    pix_render_instance tile;
                    tile.material = atlas;
                    float rot = 0.0f;
                    bool road = is_road(gx, gz);
                    if (road) {
                        int mask = (is_road(gx + 1, gz) ? 1 : 0) | (is_road(gx, gz + 1) ? 2 : 0)
                                 | (is_road(gx - 1, gz) ? 4 : 0) | (is_road(gx, gz - 1) ? 8 : 0);
                        tile.mesh = pick_road(mask, roads, &rot);
                    } else {
                        tile.mesh = base;
                    }
                    tile.transform = mat4_mul(mat4_translate(x, 0, z), mat4_rotate_y(rot));
                    town[town_count++] = tile;
                    if (road) continue;
                    pix_render_instance b;
                    b.material = atlas;
                    if (rng_chance(g, 0.8f)) {
                        b.mesh = buildings[rng_int(g, 0, 7)];
                        float s = rng_range(g, 0.8f, 1.2f);
                        b.transform = mat4_mul(mat4_translate(x, 0, z),
                                               mat4_mul(mat4_rotate_y(rng_int(g, 0, 3) * PIX_PI * 0.5f), mat4_scale(s, s, s)));
                    } else {
                        b.mesh = tree;
                        b.transform = mat4_mul(mat4_translate(x, 0, z), mat4_scale(1.5f, 1.5f, 1.5f));
                    }
                    town[town_count++] = b;
                }
        }

        camera cam = {};
        cam.position = v3(cosf(yaw) * cosf(pitch), sinf(pitch), sinf(yaw) * cosf(pitch)) * dist;
        cam.direction = v3(0, 0, 0) - cam.position;
        cam.far_plane = 600.0f;
        pix_begin_3d(cam);
        for (int i = 0; i < town_count; i++) push_instance(r, town[i]);
        pix_end_3d();

        pix_draw_rect_rounded({ 12, 12, 470, 60 }, 10, pix_rgba(0, 0, 0, 140));
        pix_draw_textf(24, 20, 20, PIX_WHITE, "%d instances   %d draws   %d culled", town_count, r.frame_draws, r.frame_culled);
        pix_draw_text("right drag orbit   wheel zoom   R regenerate   T time", 24, 46, 16, PIX_LIGHTGRAY);
        pix_draw_fps((float)pix_width() - 90, 12);
    }
    pix_free(town);
    pix_shutdown();
    return 0;
}
