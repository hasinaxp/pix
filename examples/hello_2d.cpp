// A 2D starter: a player square you move with WASD / arrows / a gamepad,
// a scrolling world under a 2D camera, and a HUD on top.
//   build.bat hello_2d
#define PIX_IMPLEMENTATION
#include "pix.h"

int main() {
    if (!pix_init("pix - hello 2D", 1280, 720)) return 1;

    vec2 player = v2(0.0f, 0.0f);
    pix_camera2d cam = {};
    cam.zoom = 1.0f;
    rng r = rng_seed(7);

    // a field of coins, collected by walking over them
    vec2 coins[64];
    bool taken[64] = {};
    for (int i = 0; i < 64; i++) coins[i] = v2(rng_range(r, -900, 900), rng_range(r, -600, 600));
    int score = 0;

    while (pix_frame()) {
        if (pix_key_pressed(KEY_ESC)) pix_quit();
        if (pix_key_pressed(KEY_F11)) pix_toggle_fullscreen();
        float dt = pix_dt();

        vec2 axis = pix_input_axis();
        player += v2(axis.x, -axis.y) * (360.0f * dt);       // screen y grows downward
        cam.zoom = clampf(cam.zoom + pix_mouse_wheel() * 0.1f, 0.3f, 3.0f);
        cam.offset = v2(pix_width() * 0.5f, pix_height() * 0.5f);
        cam.target += (player - cam.target) * (1.0f - expf(-6.0f * dt));   // smooth follow

        for (int i = 0; i < 64; i++)
            if (!taken[i] && v2dist(coins[i], player) < 30.0f) { taken[i] = true; score++; }

        pix_clear(pix_rgba(24, 26, 34));

        pix_begin_2d(cam);
        for (int x = -1000; x <= 1000; x += 100) pix_draw_line(v2((float)x, -700), v2((float)x, 700), 2, pix_rgba(40, 44, 58));
        for (int y = -700; y <= 700; y += 100)   pix_draw_line(v2(-1000, (float)y), v2(1000, (float)y), 2, pix_rgba(40, 44, 58));
        for (int i = 0; i < 64; i++)
            if (!taken[i]) {
                float bob = sinf((float)pix_time() * 4.0f + i) * 4.0f;
                pix_draw_circle(v2(coins[i].x, coins[i].y + bob), 12, PIX_YELLOW);
                pix_draw_circle_lines(v2(coins[i].x, coins[i].y + bob), 12, 3, PIX_ORANGE);
            }
        pix_draw_rect_rounded({ player.x - 20, player.y - 20, 40, 40 }, 8, PIX_SKYBLUE);
        pix_end_2d();

        pix_draw_rect_rounded({ 12, 12, 330, 84 }, 10, pix_rgba(0, 0, 0, 150));
        pix_draw_textf(24, 20, 30, PIX_WHITE, "coins: %d / 64", score);
        pix_draw_text("WASD / arrows / pad to move, wheel to zoom", 24, 60, 18, PIX_LIGHTGRAY);
        pix_draw_fps((float)pix_width() - 90, 12);
    }
    pix_shutdown();
    return 0;
}
