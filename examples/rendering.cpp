// The renderer's look, one toggle at a time: screen AO, contact shadows, soft
// (PCSS) sun shadows, spot and point light shadows, screen and hull outlines,
// height fog, volumetric light, depth of field, motion blur, eye adaptation,
// film grain and a wireframe view - over a colonnade the low sun shines
// through, so the light shafts have something to fall between.
//   build.bat rendering run
//
//   1 AO   2 contact shadows   3 soft shadows   4 lamp shadows   5 outlines
//   6 height fog   7 volumetric light   8 depth of field   9 motion blur
//   0 auto exposure   G grain   F wireframe   right drag orbit, wheel zoom
#define PIX_IMPLEMENTATION
#include "pix.h"

int main() {
    if (!pix_init("pix - rendering", 1280, 720)) return 1;
    pix_renderer& r = pix_get_renderer();
    pix_set_quality(PIX_QUALITY_HIGH);
    pix_set_time_of_day(r, 17.2f);
    pix_shadows().distance = 50.0f;

    // ---- the look: plain settings on the renderer ----
    r.ao.enabled = true;
    r.shadows.contact = 0.3f;
    r.shadows.penumbra = 0.03f;
    r.shadows.local = true;
    r.outline.enabled = true;
    r.outline.color = v3(0.06f, 0.05f, 0.08f);
    r.outline.fade_distance = 40.0f;
    r.fog.height = 0.0f;
    r.fog.height_falloff = 0.25f;
    r.fog.sun_scatter = 1.0f;
    pix_set_fog(r, v3(0.62f, 0.66f, 0.74f), 0.03f);
    r.volumetrics.enabled = true;
    r.volumetrics.density = 0.03f;
    r.dof.focus_distance = 14.0f;
    r.dof.focus_range = 10.0f;
    r.motion_blur.strength = 0.8f;
    r.post.exposure_key = 0.22f;

    idx ground = pix_gen_plane(120, 120, 30);
    idx cube   = pix_gen_cube(v3(1, 1, 1));
    idx ball   = pix_gen_sphere(1.0f);
    idx pillar = pix_gen_cylinder(0.45f, 5.0f);
    idx m_ground = pix_make_material(pix_rgba(150, 140, 120), 0.0f, 0.9f);
    idx m_stone  = pix_make_material(pix_rgba(205, 198, 182), 0.0f, 0.8f);
    idx m_crate  = pix_make_material(pix_rgba(176, 120, 70), 0.0f, 0.7f);
    idx m_metal  = pix_make_material(pix_rgba(220, 200, 160), 1.0f, 0.25f);
    idx m_red    = pix_make_material(pix_rgba(200, 50, 45), 0.0f, 0.4f);
    material toon = pix_material_default();
    toon.color = v3(0.35f, 0.7f, 1.0f);
    toon.toon = 1.0f;
    toon.rim = 0.4f;
    toon.outline_width = 0.04f;               // inverted-hull ink, in metres
    toon.outline_color = v3(0.05f, 0.05f, 0.1f);
    toon.outline_group = 1;
    idx m_toon = pix_add_material(r, toon);

    pix_model fox = pix_load_model("assets/glTF/Fox/Fox.glb");
    animator fox_anim = pix_model_animator(fox);
    if (fox_anim.clip_count > 1) animator_play(fox_anim, 1);
    if (fox.material != PIX_INVALID) {
        material* m = pix_get_material(r, fox.material);
        m->outline_width = 0.03f;             // world metres, whatever the model's own units
        m->outline_color = v3(0.15f, 0.08f, 0.04f);
        m->outline_group = 2;
    }

    float yaw = 0.6f, pitch = 0.32f, dist = 22.0f;
    bool dof = false, blur = false, wire = false;

    while (pix_frame()) {
        float dt = pix_dt(), t = (float)pix_time();
        if (pix_key_pressed(KEY_ESC)) pix_quit();
        if (pix_key_pressed('1')) r.ao.enabled = !r.ao.enabled;
        if (pix_key_pressed('2')) r.shadows.contact = r.shadows.contact > 0 ? 0.0f : 0.3f;
        if (pix_key_pressed('3')) r.shadows.penumbra = r.shadows.penumbra > 0 ? 0.0f : 0.03f;
        if (pix_key_pressed('4')) r.shadows.local = !r.shadows.local;
        if (pix_key_pressed('5')) r.outline.enabled = !r.outline.enabled;
        if (pix_key_pressed('6')) r.fog.height_falloff = r.fog.height_falloff > 0 ? 0.0f : 0.25f;
        if (pix_key_pressed('7')) r.volumetrics.enabled = !r.volumetrics.enabled;
        if (pix_key_pressed('8')) r.dof.enabled = dof = !dof;
        if (pix_key_pressed('9')) r.motion_blur.enabled = blur = !blur;
        if (pix_key_pressed('0')) r.post.auto_exposure = !r.post.auto_exposure;
        if (pix_key_pressed('G')) r.post.grain = r.post.grain > 0 ? 0.0f : 0.04f;
        if (pix_key_pressed('F')) r.wireframe = wire = !wire;
        if (pix_mouse_down(PIX_MOUSE_RIGHT)) {
            vec2 d = pix_mouse_delta();
            yaw += d.x * 0.006f;
            pitch = clampf(pitch + d.y * 0.006f, 0.05f, 1.4f);
        }
        dist = clampf(dist - pix_mouse_wheel() * 1.5f, 5.0f, 60.0f);
        animator_update(fox_anim, dt);

        camera cam = {};
        vec3 focus = v3(0, 2.0f, 0);
        cam.position = focus + v3(cosf(yaw) * cosf(pitch), sinf(pitch), sinf(yaw) * cosf(pitch)) * dist;
        cam.direction = focus - cam.position;

        pix_begin_3d(cam);
        pix_draw_mesh(ground, m_ground, mat4_identity());
        // a colonnade with a roof: the sun comes in between the pillars
        for (int i = 0; i < 10; i++) {
            float x = -9.0f + i * 2.0f;
            pix_draw_mesh(pillar, m_stone, mat4_translate(x, 2.5f, -4.0f));
            pix_draw_mesh(pillar, m_stone, mat4_translate(x, 2.5f, 4.0f));
        }
        pix_draw_mesh(cube, m_stone, mat4_mul(mat4_translate(0, 5.25f, 0), mat4_scale(21.0f, 0.5f, 9.5f)));
        // crates, spheres and the fox inside it
        for (int i = 0; i < 4; i++)
            pix_draw_mesh(cube, m_crate, mat4_mul(mat4_translate(-5.0f + (i % 2) * 1.02f, 0.5f + (i / 2) * 1.0f, 1.0f),
                                                  mat4_rotate_y(0.2f * i)));
        pix_draw_mesh(ball, m_metal, mat4_mul(mat4_translate(2.0f, 1.0f, -1.5f), mat4_scale(1, 1, 1)));
        pix_draw_mesh(ball, m_red, mat4_mul(mat4_translate(4.5f, 0.7f, 1.0f), mat4_scale(0.7f, 0.7f, 0.7f)));
        pix_draw_mesh(ball, m_toon, mat4_mul(mat4_translate(-1.5f, 0.8f, -1.5f), mat4_scale(0.8f, 0.8f, 0.8f)));
        if (fox.mesh != PIX_INVALID)
            pix_draw_model_animated(fox, fox_anim.pose, mat4_mul(mat4_translate(6.0f, 0, -1.0f),
                                    mat4_mul(mat4_rotate_y(-1.2f), mat4_scale(0.025f, 0.025f, 0.025f))));
        // a spot light under the roof and a lamp by the crates, both casting shadows
        pix_light spot = pix_spot_light(v3(-1.0f, 4.8f, 0.0f), v3(0.15f, -1.0f, 0.1f), 9.0f, 0.35f, 0.6f, v3(60.0f, 45.0f, 25.0f));
        spot.cast_shadows = true;
        pix_add_light(spot);
        pix_light lamp = pix_point_light(v3(-4.0f, 2.6f + sinf(t) * 0.2f, -1.5f), 7.0f, v3(4.0f, 9.0f, 14.0f));
        lamp.cast_shadows = true;
        pix_add_light(lamp);
        pix_end_3d();

        pix_draw_rect_rounded({ 12, 12, 760, 72 }, 10, pix_rgba(0, 0, 0, 140));
        pix_draw_textf(24, 18, 18, PIX_WHITE, "1 AO %s  2 contact %s  3 soft %s  4 lamp shadows %s  5 outlines %s  6 height fog %s",
                       r.ao.enabled ? "on" : "off", r.shadows.contact > 0 ? "on" : "off", r.shadows.penumbra > 0 ? "on" : "off",
                       r.shadows.local ? "on" : "off", r.outline.enabled ? "on" : "off", r.fog.height_falloff > 0 ? "on" : "off");
        pix_draw_textf(24, 44, 18, PIX_LIGHTGRAY, "7 volumetric %s  8 depth of field %s  9 motion blur %s  0 auto exposure %s  G grain  F wireframe",
                       r.volumetrics.enabled ? "on" : "off", dof ? "on" : "off", blur ? "on" : "off",
                       r.post.auto_exposure ? "on" : "off");
        pix_draw_fps((float)pix_width() - 90, 12);
    }
    pix_shutdown();
    return 0;
}
