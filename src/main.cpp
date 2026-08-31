#define _CRT_SECURE_NO_WARNINGS
#include <time.h>
#include <stdio.h>
#include "platform.hpp"
#include "opengl_api.hpp"
#include "opengl_utils.hpp"
#include "shader_sources.hpp"
#include "math.hpp"
#include "data_loader.hpp"
#include "renderer.hpp"

#define DISPLAY_WIDTH  1280
#define DISPLAY_HEIGHT 720
#define ASSET_DIR "assets/KayKit_City_Builder/Assets/obj/"
#define GRID 18
#define CELL 2.0f
#define HALF_PI 1.5707963f

static idx load_obj(pix_data_loader& loader, pix_renderer& renderer, const char* name) {
    char path[260];
    snprintf(path, sizeof(path), "%s%s.obj", ASSET_DIR, name);
    idx fd = load_mesh_obj_file(loader, path);
    if (fd == (idx)-1) { MessageBoxA(0, path, "missing OBJ file", MB_OK); }
    return load_mesh(renderer, loader.mesh_files[fd]);
}

// connection bitmask: bit0 +X(east), bit1 +Z(north), bit2 -X(west), bit3 -Z(south)
#define ROAD_STRAIGHT_MASK 0x0A  // +Z -Z
#define ROAD_CORNER_MASK   0x03  // +X +Z
#define ROAD_TSPLIT_MASK   0x0B  // +X +Z -Z
#define ROAD_JUNCTION_MASK 0x0F

static int popcount4(int m) { return (m & 1) + ((m >> 1) & 1) + ((m >> 2) & 1) + ((m >> 3) & 1); }
static int rotr4(int m, int k) { k &= 3; return ((m >> k) | (m << (4 - k))) & 0xF; }

// choose road piece + Y rotation so its openings line up with `mask`
static idx pick_road(int mask, idx straight, idx corner, idx tsplit, idx junction, float* rot) {
    int n = popcount4(mask);
    for (int k = 0; k < 4; k++) {
        float a = (float)k * HALF_PI;
        if (n == 4 && rotr4(ROAD_JUNCTION_MASK, k) == mask) { *rot = a; return junction; }
        if (n == 3 && rotr4(ROAD_TSPLIT_MASK,   k) == mask) { *rot = a; return tsplit;   }
        if (n == 2 && rotr4(ROAD_STRAIGHT_MASK, k) == mask) { *rot = a; return straight; }
        if (n == 2 && rotr4(ROAD_CORNER_MASK,   k) == mask) { *rot = a; return corner;   }
    }
    *rot = (mask & 0x05) ? HALF_PI : 0.0f; // dead end / isolated: lay a straight along its axis
    return straight;
}

static bool is_road_cell(int gx, int gz) {
    if (gx < 0 || gz < 0 || gx >= GRID || gz >= GRID) return false;
    return (gx % 5 == 0) || (gz % 5 == 0);
}

int main() {
    srand((unsigned)time(0));

    pix_window window = pix_create_window("pix - random city", DISPLAY_WIDTH, DISPLAY_HEIGHT);
    if (!opengl_load_functions()) {
        MessageBoxA(0, "failed to load OpenGL 4.4 functions", "fatal", MB_OK);
        return 1;
    }

    idx shader = opengl_create_shader(VSHDER_BASIC, FSHDER_BASIC);
    static pix_renderer renderer;
    renderer = pix_create_renderer(DISPLAY_WIDTH, DISPLAY_HEIGHT, shader, v3(0.53f, 0.72f, 0.92f));

    pix_data_loader loader = pix_create_data_loader();

    const char* building_names[8] = {
        "building_A", "building_B", "building_C", "building_D",
        "building_E", "building_F", "building_G", "building_H",
    };
    idx building_mesh[8];
    for (int i = 0; i < 8; i++) building_mesh[i] = load_obj(loader, renderer, building_names[i]);
    idx road_straight = load_obj(loader, renderer, "road_straight");
    idx road_corner   = load_obj(loader, renderer, "road_corner");
    idx road_tsplit   = load_obj(loader, renderer, "road_tsplit");
    idx road_junction = load_obj(loader, renderer, "road_junction");
    idx base_mesh     = load_obj(loader, renderer, "base");

    char tex_path[260];
    snprintf(tex_path, sizeof(tex_path), "%scitybits_texture.png", ASSET_DIR);
    idx city_mat = load_material(renderer, tex_path, v3(1.0f, 1.0f, 1.0f), 0.1f, 0.6f);

    // --- random city, generated once ---
    static pix_render_instance city[GRID * GRID * 2];
    int city_count = 0;
    for (int gx = 0; gx < GRID; gx++)
        for (int gz = 0; gz < GRID; gz++) {
            float x = (gx - GRID * 0.5f) * CELL;
            float z = (gz - GRID * 0.5f) * CELL;
            bool road = is_road_cell(gx, gz);
            float rot = (float)(rand() % 4) * HALF_PI;

            pix_render_instance tile = {};
            tile.material = city_mat;
            if (road) {
                int mask = (is_road_cell(gx + 1, gz) ? 1 : 0)
                         | (is_road_cell(gx, gz + 1) ? 2 : 0)
                         | (is_road_cell(gx - 1, gz) ? 4 : 0)
                         | (is_road_cell(gx, gz - 1) ? 8 : 0);
                tile.mesh = pick_road(mask, road_straight, road_corner, road_tsplit, road_junction, &rot);
            } else {
                tile.mesh = base_mesh;
            }
            tile.trainsform = mat4_mul(mat4_translate(x, 0.0f, z), mat4_rotate_y(rot));
            city[city_count++] = tile;

            if (!road && (rand() % 5) != 0) {
                pix_render_instance b = {};
                b.material = city_mat;
                b.mesh = building_mesh[rand() % 8];
                float s = 0.80f + (float)(rand() % 100) * 0.004f;
                b.trainsform = mat4_mul(mat4_translate(x, 0.0f, z),
                    mat4_mul(mat4_rotate_y((float)(rand() % 4) * HALF_PI), mat4_scale(s, s, s)));
                city[city_count++] = b;
            }
        }

    LARGE_INTEGER freq, prev, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prev);

    float yaw = 0.7f, pitch = 0.6f, dist = GRID * CELL * 0.85f;

    while (!window.should_close) {
        pix_update_window(window);
        if (window.keystates[KEY_ESC].pressed) break;

        QueryPerformanceCounter(&now);
        float dt = (float)(now.QuadPart - prev.QuadPart) / (float)freq.QuadPart;
        prev = now;

        if (window.keystates[MOUSE_BUTTON_LEFT].held) {
            yaw   += window.mouse_rel_x * 0.005f;
            pitch += window.mouse_rel_y * 0.005f;
        } else {
            yaw += dt * 0.10f;
        }
        if (pitch < 0.15f) pitch = 0.15f;
        if (pitch > 1.40f) pitch = 1.40f;
        if (window.keystates[KEY_UP].held)   dist -= dt * 20.0f;
        if (window.keystates[KEY_DOWN].held) dist += dt * 20.0f;
        if (dist < 4.0f) dist = 4.0f;

        camera cam = {};
        cam.position = v3(cosf(yaw) * cosf(pitch) * dist,
                          sinf(pitch) * dist,
                          sinf(yaw) * cosf(pitch) * dist);
        cam.direction = v3norm(v3sub(v3(0.0f, 1.0f, 0.0f), cam.position));
        cam.up = v3(0.0f, 1.0f, 0.0f);

        begin_frame(renderer, cam);
        for (int i = 0; i < city_count; i++) push_instance(renderer, city[i]);
        end_frame(renderer);

    }

    pix_destory_renderer(renderer);
    return 0;
}
