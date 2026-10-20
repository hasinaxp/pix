// A 3D starter: primitives, a textured OBJ, an animated glTF character,
// cascaded shadows, point lights, transparency, particles and a HUD.
//   build.bat hello_3d        (run from the repo root so assets/ resolves)
//
// Controls: right mouse drag orbits, wheel zooms, T advances the time of day,
// Q cycles quality presets, F toggles FXAA, G the debug grid, 1-4 toggle post
// effects (outline, CRT, pixelate, chromatic aberration), Esc quits.
#define PIX_IMPLEMENTATION
#include "pix.h"

int main() {
    if (!pix_init("pix - hello 3D", 1280, 720)) return 1;
    pix_renderer& r = pix_get_renderer();

    // the look: plain settings, edited in place
    pix_shadows().distance = 60.0f;      // world units the cascades cover
    pix_shadows().softness = 1;          // 3x3 PCF
    pix_post().bloom_strength = 0.35f;
    int quality = PIX_QUALITY_HIGH;
    const char* quality_names[] = { "low", "medium", "high", "ultra" };

    // post effects, added once and toggled at will (off until a key is pressed)
    int fx[4] = { pix_add_effect(PIX_EFFECT_OUTLINE, v4(1.0f, 0.04f, 0.85f, 0)),
                  pix_add_effect(PIX_EFFECT_CRT, v4(0.6f, 0.08f, 0, 0)),
                  pix_add_effect(PIX_EFFECT_PIXELATE, v4(4.0f, 0, 0, 0)),
                  pix_add_effect(PIX_EFFECT_CHROMATIC, v4(3.0f, 0, 0, 0)) };
    const char* fx_names[] = { "outline", "crt", "pixelate", "chromatic" };
    bool fx_on[4] = {};
    for (int i = 0; i < 4; i++) pix_effect_enable(fx[i], false);

    // procedural geometry + materials
    idx ground = pix_gen_plane(80, 80, 20);
    idx cube   = pix_gen_cube(v3(1, 1, 1));
    idx sphere = pix_gen_sphere(0.6f);
    idx pillar = pix_gen_cylinder(0.35f, 3.0f);
    idx m_ground = pix_make_material(pix_rgba(120, 124, 110), 0.0f, 0.9f);
    idx m_red    = pix_make_material(pix_rgba(200, 50, 45), 0.0f, 0.35f);
    idx m_gold   = pix_make_material(pix_rgba(255, 200, 90), 1.0f, 0.25f);
    idx m_stone  = pix_make_material(pix_rgba(180, 176, 168), 0.0f, 0.8f);
    idx m_glass  = pix_make_material(pix_rgba(120, 200, 255, 110), 0.0f, 0.05f);   // alpha < 255 -> blended
    idx m_glow   = pix_make_material(pix_rgba(255, 140, 60));
    pix_get_material(r, m_glow)->emissive = v3(6.0f, 2.4f, 0.8f);                 // HDR, blooms

    // assets from disk (each is optional: the demo still runs without them)
    pix_model building = pix_load_model("assets/KayKit_City_Builder/Assets/obj/building_B.obj");
    if (building.material != PIX_INVALID) {
        // KayKit's atlas is colour swatches: sample it without mipmaps or the swatches bleed
        material* m = pix_get_material(r, building.material);
        opengl_destroy_texture(m->texture);
        m->texture = pix_load_texture("assets/KayKit_City_Builder/Assets/obj/citybits_texture.png", TEXTURE_LINEAR).id;
    }
    pix_model fox = pix_load_model("assets/glTF/Fox/Fox.glb");
    animator fox_anim = pix_model_animator(fox);
    if (fox_anim.clip_count > 2) animator_play(fox_anim, 2);    // run

    float yaw = 0.8f, pitch = 0.45f, dist = 16.0f, hour = 16.5f;
    bool grid = false;
    pix_set_time_of_day(r, hour);

    while (pix_frame()) {
        float dt = pix_dt(), t = (float)pix_time();
        if (pix_key_pressed(KEY_ESC)) pix_quit();
        if (pix_key_pressed('F')) pix_post().fxaa = !pix_post().fxaa;
        if (pix_key_pressed('Q')) { quality = (quality + 1) % 4; pix_set_quality(quality); }
        for (int i = 0; i < 4; i++)
            if (pix_key_pressed('1' + i)) { fx_on[i] = !fx_on[i]; pix_effect_enable(fx[i], fx_on[i]); }
        if (pix_key_pressed('G')) grid = !grid;
        if (pix_key_down('T')) { hour += dt * 2.0f; pix_set_time_of_day(r, hour); }
        if (pix_mouse_down(PIX_MOUSE_RIGHT)) {
            vec2 d = pix_mouse_delta();
            yaw += d.x * 0.006f;
            pitch = clampf(pitch + d.y * 0.006f, 0.05f, 1.45f);
        }
        dist = clampf(dist - pix_mouse_wheel() * 1.5f, 4.0f, 60.0f);

        camera cam = {};
        vec3 focus = v3(0, 1.0f, 0);
        cam.position = focus + v3(cosf(yaw) * cosf(pitch), sinf(pitch), sinf(yaw) * cosf(pitch)) * dist;
        cam.direction = focus - cam.position;

        animator_update(fox_anim, dt);

        pix_begin_3d(cam);
        pix_draw_mesh(ground, m_ground, mat4_identity());
        for (int i = 0; i < 12; i++) {                               // a ring of pillars and orbs
            float a = i * PIX_TAU / 12.0f;
            vec3 p = v3(cosf(a) * 9.0f, 0, sinf(a) * 9.0f);
            pix_draw_mesh(pillar, m_stone, mat4_translate(p.x, 1.5f, p.z));
            pix_draw_mesh(sphere, (i & 1) ? m_gold : m_red, mat4_translate(p.x, 3.6f + sinf(t * 2 + i) * 0.2f, p.z));
        }
        for (int x = -2; x <= 2; x++)                                // a stack of spinning crates
            pix_draw_mesh(cube, m_red, mat4_mul(mat4_translate(x * 1.6f, 0.5f, -3.0f), mat4_rotate_y(t * 0.5f + x)),
                          pix_rgba(255, 255 - (x + 2) * 40, 255));
        pix_draw_mesh(cube, m_glass, mat4_mul(mat4_translate(0, 1.2f, 3.0f), mat4_scale(3.0f, 2.4f, 0.2f)));

        if (building.mesh != PIX_INVALID)
            pix_draw_model(building, mat4_mul(mat4_translate(-2.8f, 0, 0.5f), mat4_scale(1.4f, 1.4f, 1.4f)));
        if (fox.mesh != PIX_INVALID) {
            float fa = t * 0.6f - 2.0f;                              // the fox runs laps
            mat4 m = mat4_mul(mat4_translate(cosf(fa) * 6.0f, 0, sinf(fa) * 6.0f),
                              mat4_mul(mat4_rotate_y(-fa), mat4_scale(0.02f, 0.02f, 0.02f)));
            pix_draw_model_animated(fox, fox_anim.pose, m);
        }

        // a flickering brazier: an emissive core, a warm point light and sparks
        vec3 fire = v3(4.0f, 0.5f, -1.0f);
        pix_draw_mesh(sphere, m_glow, mat4_mul(mat4_translate(fire.x, fire.y, fire.z), mat4_scale(0.5f, 0.5f, 0.5f)));
        pix_add_light(pix_point_light(fire + v3(0, 1, 0), 9.0f, v3(9.0f, 4.5f, 1.8f) * (0.85f + 0.15f * sinf(t * 17.0f))));
        for (int i = 0; i < 40; i++) {
            float life = fmodf(t * 0.7f + i * 0.137f, 1.0f);
            vec3 p = fire + v3(sinf(i * 12.9f) * 0.3f, 0.4f + life * 2.5f, cosf(i * 7.3f) * 0.3f);
            push_particle(r, p, 0.06f * (1.0f - life), v3(8.0f, 3.0f, 0.8f), 1.0f - life);
        }
        if (grid) pix_draw_grid(40, 40, pix_rgba(255, 255, 255, 60));
        pix_end_3d();

        pix_draw_rect_rounded({ 12, 12, 520, 96 }, 10, pix_rgba(0, 0, 0, 140));
        pix_draw_textf(24, 18, 24, PIX_WHITE, "%02d:%02d   %d draws   %d instances   %d culled",
                       (int)r.hour, (int)(fmodf(r.hour, 1.0f) * 60), r.frame_draws, r.frame_instances, r.frame_culled);
        pix_draw_text("right drag orbit  wheel zoom  T time  Q quality  F fxaa  G grid  1-4 fx", 24, 52, 17, PIX_LIGHTGRAY);
        char fx_line[128] = "";
        for (int i = 0; i < 4; i++) if (fx_on[i]) { strcat(fx_line, fx_names[i]); strcat(fx_line, " "); }
        pix_draw_textf(24, 76, 17, PIX_LIGHTGRAY, "quality %s   shadows %d px   fxaa %s   %s", quality_names[quality],
                       pix_shadows().resolution, pix_post().fxaa ? "on" : "off", fx_line);
        pix_draw_fps((float)pix_width() - 90, 12);
    }
    pix_shutdown();
    return 0;
}
