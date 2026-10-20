// Composing your own frame from the renderer's stages, plus custom shaders:
//   * the main view and a top-down minimap view of the same scene, the
//     minimap drawn into its own render target and shown in the HUD
//   * layers: markers that only the minimap draws
//   * a row of spheres going from pure PBR to pure toon shading
//   * a custom "hologram" surface shader driven by named material uniforms
//   * a custom fullscreen pass (a scanline tint) applied to the minimap only
//   build.bat pipeline
#define PIX_IMPLEMENTATION
#include "pix.h"

#define LAYER_WORLD   0x1u
#define LAYER_MINIMAP 0x2u

static const char* HOLOGRAM_FS = R"(#version 440 core
in vec3 vNormal;
in vec2 vUV;
in vec3 vWorld;
in vec4 vTint;
out vec4 frag;
uniform vec3  uGlow;          // set per material with pix_material_set_vec3
uniform float uScanDensity;   // set per material with pix_material_set_float
void main() {
    vec3 v = normalize(uCameraPos - vWorld);
    float rim = 1.0 - abs(dot(normalize(vNormal), v));
    float scan = 0.5 + 0.5 * sin(vWorld.y * uScanDensity - uTime * 4.0);
    frag = vec4(uGlow * (rim * 2.0 + scan * 0.5), clamp(rim + 0.2 * scan, 0.0, 1.0));
}
)";

static const char* MINIMAP_TINT = R"(
void main() {
    vec3 c = texture(uTex0, vUV).rgb;
    float line = 0.85 + 0.15 * step(0.5, fract(vUV.y / uTexel.y * 0.25));
    frag = vec4(mix(c, c * uParams.rgb, 0.35) * line, 1.0);
}
)";

int main() {
    if (!pix_init("pix - custom pipeline", 1280, 720)) return 1;
    pix_renderer& r = pix_get_renderer();
    pix_shadows().distance = 40.0f;
    pix_time_of_day(15.0f);

    idx ground = pix_gen_plane(60, 60, 12), cube = pix_gen_cube(v3(1, 1, 1)), sphere = pix_gen_sphere(0.7f, 24, 48);
    idx m_ground = pix_make_material(pix_rgba(120, 130, 110), 0.0f, 0.9f);
    idx m_marker = pix_make_material(pix_rgba(255, 60, 60));
    pix_get_material(r, m_marker)->unlit = true;

    // PBR -> toon, left to right
    idx m_mix[5];
    for (int i = 0; i < 5; i++) {
        material m = pix_material_default();
        m.color = v3(0.95f, 0.45f, 0.2f);
        m.roughness = 0.35f;
        m.toon = i / 4.0f;          // the blend
        m.toon_bands = 3.0f;
        m.rim = 0.6f;
        m.toon_specular = 0.4f;
        m_mix[i] = pix_add_material(r, m);
    }

    // a custom surface shader with its own named uniforms
    material holo = pix_material_default();
    holo.shader = load_surface_shader(r, HOLOGRAM_FS);
    holo.blend = PIX_BLEND_ADDITIVE;
    holo.cast_shadows = false;
    holo.double_sided = true;
    idx m_holo = pix_add_material(r, holo);
    pix_material_set_vec3(r, m_holo, "uGlow", v3(0.2f, 0.9f, 1.6f));
    pix_material_set_float(r, m_holo, "uScanDensity", 12.0f);

    // the minimap's own targets and its fullscreen pass
    framebuffer minimap_hdr = pix_create_render_target(256, 256, true);
    framebuffer minimap_ldr = pix_create_render_target(256, 256, false, false, false);
    framebuffer minimap_out = pix_create_render_target(256, 256, false, false, false);
    idx tint_pass = pix_load_fullscreen_shader(r, MINIMAP_TINT);

    float yaw = 0.9f;
    while (pix_frame()) {
        float t = (float)pix_time();
        if (pix_key_pressed(KEY_ESC)) pix_quit();
        if (pix_mouse_down(PIX_MOUSE_RIGHT)) yaw += pix_mouse_delta().x * 0.006f;

        camera cam = {};
        cam.position = v3(cosf(yaw) * 14.0f, 6.0f, sinf(yaw) * 14.0f);
        cam.direction = v3(0, 1, 0) - cam.position;

        pix_begin_3d(cam);
        pix_draw_mesh(ground, m_ground, mat4_identity());
        for (int i = 0; i < 5; i++) {
            vec3 p = v3(-6.0f + i * 3.0f, 0.7f, 0.0f);
            push_instance(r, sphere, m_mix[i], mat4_translate(p.x, p.y, p.z));
            // a marker above each sphere, only on the minimap layer
            push_instance(r, cube, m_marker, mat4_mul(mat4_translate(p.x, 3.0f, p.z), mat4_scale(1.2f, 0.1f, 1.2f)),
                          v4(1, 1, 1, 1), LAYER_MINIMAP);
        }
        for (int i = 0; i < 6; i++) {
            float a = i * PIX_TAU / 6.0f + t * 0.2f;
            pix_draw_mesh(cube, m_mix[i % 5], mat4_mul(mat4_translate(cosf(a) * 7.0f, 0.5f, sinf(a) * 5.0f), mat4_rotate_y(a)));
        }
        push_instance(r, sphere, m_holo, mat4_mul(mat4_translate(0, 2.2f + sinf(t) * 0.3f, -4.0f), mat4_scale(1.6f, 1.6f, 1.6f)));

        // ---- the frame, composed by hand ----
        pix_render_shadows(r);

        pix_view main_view = pix_view_default(r, cam);
        main_view.layers = LAYER_WORLD;                 // no minimap markers here
        pix_render_view(r, main_view);

        pix_view mini = pix_view_default(r, cam);
        mini.cam.position = v3(0, 40, 0);
        mini.cam.direction = v3(0, -1, 0);
        mini.cam.up = v3(0, 0, -1);
        mini.cam.ortho_height = 30.0f;                  // orthographic, 30 units tall
        mini.target = &minimap_hdr;
        mini.layers = LAYER_WORLD | LAYER_MINIMAP;
        mini.sky = false;
        mini.clear_color = v3(0.1f, 0.12f, 0.15f);
        mini.particles = mini.lines = false;
        pix_render_view(r, mini);

        pix_post_process(r, r.scene_target, nullptr);    // the main view to the window
        pix_post_process(r, minimap_hdr, &minimap_ldr);  // the minimap to its own LDR target
        idx src = minimap_ldr.color;
        pix_fullscreen_pass(r, tint_pass, &src, 1, &minimap_out, v4(0.4f, 1.0f, 0.6f, 0.0f));
        pix_finish_3d();

        // ---- HUD ----
        float mx = (float)pix_width() - 276.0f;
        pix_draw_rect_rounded({ mx - 6, 14, 268, 268 }, 8, pix_rgba(0, 0, 0, 160));
        pix_draw_target(minimap_out, { mx, 20, 256, 256 });
        pix_draw_rect_rounded({ 12, 12, 560, 60 }, 10, pix_rgba(0, 0, 0, 140));
        pix_draw_text("spheres: PBR  ->  toon     hologram: custom shader     right drag: orbit", 24, 22, 18, PIX_WHITE);
        pix_draw_textf(24, 44, 16, PIX_LIGHTGRAY, "%d draws this frame (2 views + shadows)", r.frame_draws);
        pix_draw_fps(12, (float)pix_height() - 32);
    }
    pix_destroy_framebuffer(minimap_hdr);
    pix_destroy_framebuffer(minimap_ldr);
    pix_destroy_framebuffer(minimap_out);
    pix_shutdown();
    return 0;
}
