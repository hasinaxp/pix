// Reflections and refraction: a water pool (planar mirror + refraction), glass
// panes, and metal spheres of increasing roughness picking up the reflection
// probe - all through the settings interface.
//   build.bat reflections
//
// Controls: right drag orbit, wheel zoom, P toggles the probe, M the planar
// mirror, R cycles probe resolution, T advances the time of day, Esc quits.
#define PIX_IMPLEMENTATION
#include "pix.h"

int main() {
    if (!pix_init("pix - reflections", 1280, 720)) return 1;
    pix_renderer& r = pix_get_renderer();
    pix_set_quality(PIX_QUALITY_HIGH);           // shadows, bloom, fxaa and the probe
    pix_shadows().distance = 50.0f;
    pix_time_of_day(17.0f);

    // the pool: a water plane a little below the ground, and the plane it mirrors across
    const float water_y = -0.35f;
    pix_set_reflection_plane(r, v3(0, 1, 0), v3(0, water_y, 0));

    idx plane  = pix_gen_plane(1, 1);
    idx cube   = pix_gen_cube(v3(1, 1, 1));
    idx sphere = pix_gen_sphere(1.0f, 32, 64);
    idx m_water  = pix_add_material(r, pix_material_water(r));
    idx m_toon   = pix_add_material(r, pix_material_water_toon(r));
    idx m_glass  = pix_add_material(r, pix_material_glass(v3(0.8f, 0.95f, 1.0f), 0.1f, 0.05f));
    idx m_amber  = pix_add_material(r, pix_material_glass(v3(1.0f, 0.65f, 0.25f), 0.15f, 0.08f));
    idx m_stone  = pix_make_material(pix_rgba(170, 164, 150), 0.0f, 0.85f);
    idx m_rim    = pix_make_material(pix_rgba(110, 104, 96), 0.0f, 0.9f);
    idx m_colors[4] = { pix_make_material(pix_rgba(220, 60, 50), 0.0f, 0.5f),
                        pix_make_material(pix_rgba(60, 170, 90), 0.0f, 0.5f),
                        pix_make_material(pix_rgba(60, 110, 220), 0.0f, 0.5f),
                        pix_make_material(pix_rgba(240, 200, 60), 0.0f, 0.5f) };
    idx m_metal[4];
    for (int i = 0; i < 4; i++) m_metal[i] = pix_make_material(pix_rgba(235, 235, 240), 1.0f, 0.05f + i * 0.25f);

    float yaw = 0.6f, pitch = 0.35f, dist = 18.0f, hour = 17.0f;
    int probe_res = 1;
    const int res_options[3] = { 64, 128, 256 };

    while (pix_frame()) {
        float dt = pix_dt(), t = (float)pix_time();
        if (pix_key_pressed(KEY_ESC)) pix_quit();
        if (pix_key_pressed('P')) r.reflections.probe = !r.reflections.probe;
        if (pix_key_pressed('M')) r.reflections.planar = !r.reflections.planar;
        if (pix_key_pressed('R')) { probe_res = (probe_res + 1) % 3; r.reflections.probe_resolution = res_options[probe_res]; }
        if (pix_key_down('T')) { hour += dt * 2.0f; pix_time_of_day(hour); }
        if (pix_mouse_down(PIX_MOUSE_RIGHT)) {
            vec2 d = pix_mouse_delta();
            yaw += d.x * 0.006f;
            pitch = clampf(pitch + d.y * 0.006f, 0.05f, 1.45f);
        }
        dist = clampf(dist - pix_mouse_wheel() * 1.5f, 5.0f, 60.0f);
        camera cam = {};
        vec3 focus = v3(0, 1.0f, 0);
        cam.position = focus + v3(cosf(yaw) * cosf(pitch), sinf(pitch), sinf(yaw) * cosf(pitch)) * dist;
        cam.direction = focus - cam.position;

        pix_begin_3d(cam);
        // ground with a 12 x 8 hole for the pool, built from four slabs
        pix_draw_mesh(plane, m_stone, mat4_mul(mat4_translate(0, 0, -22), mat4_scale(60, 1, 36)));
        pix_draw_mesh(plane, m_stone, mat4_mul(mat4_translate(0, 0, 22), mat4_scale(60, 1, 36)));
        pix_draw_mesh(plane, m_stone, mat4_mul(mat4_translate(-18, 0, 0), mat4_scale(24, 1, 8)));
        pix_draw_mesh(plane, m_stone, mat4_mul(mat4_translate(18, 0, 0), mat4_scale(24, 1, 8)));
        pix_draw_mesh(cube, m_rim, mat4_mul(mat4_translate(0, -1.6f, 0), mat4_scale(12, 1, 8)));   // pool floor, 1.1 deep
        pix_draw_mesh(cube, m_rim, mat4_mul(mat4_translate(0, -0.55f, -4.1f), mat4_scale(12.4f, 1.1f, 0.2f)));   // walls
        pix_draw_mesh(cube, m_rim, mat4_mul(mat4_translate(0, -0.55f,  4.1f), mat4_scale(12.4f, 1.1f, 0.2f)));
        pix_draw_mesh(cube, m_rim, mat4_mul(mat4_translate(-6.1f, -0.55f, 0), mat4_scale(0.2f, 1.1f, 8.0f)));
        pix_draw_mesh(cube, m_rim, mat4_mul(mat4_translate( 6.1f, -0.55f, 0), mat4_scale(0.2f, 1.1f, 8.0f)));
        pix_draw_mesh(plane, m_water, mat4_mul(mat4_translate(0, water_y, 0), mat4_scale(12, 1, 8)));

        // colourful things around the pool, to have something to reflect
        for (int i = 0; i < 8; i++) {
            float a = i * PIX_TAU / 8.0f;
            float h = 1.5f + (i % 3);
            pix_draw_mesh(cube, m_colors[i % 4], mat4_mul(mat4_translate(cosf(a) * 11, h * 0.5f, sinf(a) * 8.5f),
                                                        mat4_mul(mat4_rotate_y(a), mat4_scale(1.2f, h, 1.2f))));
        }
        // metal spheres, mirror-smooth to rough, floating over the water
        for (int i = 0; i < 4; i++)
            pix_draw_mesh(sphere, m_metal[i], mat4_mul(mat4_translate(-4.5f + i * 3.0f, 1.3f + sinf(t + i) * 0.15f, 0),
                                                      mat4_scale(0.9f, 0.9f, 0.9f)));
        // glass panes beside the pool
        pix_draw_mesh(cube, m_glass, mat4_mul(mat4_translate(-3, 1.5f, 5.5f), mat4_scale(3.0f, 3.0f, 0.15f)));
        pix_draw_mesh(cube, m_amber, mat4_mul(mat4_translate(3, 1.5f, 5.5f), mat4_mul(mat4_rotate_y(t * 0.3f), mat4_scale(1.5f, 3.0f, 1.5f))));
        pix_end_3d();

        pix_draw_rect_rounded({ 12, 12, 520, 84 }, 10, pix_rgba(0, 0, 0, 140));
        pix_draw_textf(24, 18, 22, PIX_WHITE, "probe %s (%d px)   mirror %s   %d draws",
                       r.reflections.probe ? "on" : "off", r.reflections.probe_resolution,
                       r.reflections.planar ? "on" : "off", r.frame_draws);
        pix_draw_text("right drag orbit   wheel zoom   P probe   M mirror   R probe res   T time", 24, 52, 17, PIX_LIGHTGRAY);
        pix_draw_fps((float)pix_width() - 90, 12);
    }
    pix_shutdown();
    return 0;
}
