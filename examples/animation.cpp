// Skeletal animation: two different glTF rigs, clip switching and blending,
// footstep sounds panned by position, gamepad rumble.
//   build.bat animation
//
// Controls: 1/2/3 switch clip (blended over 0.3 s), space pause, M mute,
// right drag orbit, wheel zoom. Gamepad: right stick orbit, A pause, bumpers clip.
#define PIX_IMPLEMENTATION
#include "pix.h"

struct actor {
    pix_model* model;
    animator   anim;
    animation  pose;          // what gets drawn: the blend of the previous and current clip
    idx   prev_clip;
    float prev_time;
    float blend;              // 0..1 toward the current clip
    mat4  transform;
    int   steps_per_cycle;
    int   last_step;
    float pitch;
};

static void switch_clip(actor& a, int clip) {
    if ((size_t)clip >= a.anim.clip_count || (idx)clip == a.anim.clip) return;
    a.prev_clip = a.anim.clip;
    a.prev_time = a.anim.time;
    a.blend = 0.0f;
    animator_play(a.anim, (idx)clip);
    a.last_step = -1;
}

int main() {
    if (!pix_init("pix - animation", 1280, 720)) return 1;
    pix_shadows().distance = 30.0f;
    pix_time_of_day(9.5f);

    pix_model fox = pix_load_model("assets/glTF/Fox/Fox.glb");
    pix_model man = pix_load_model("assets/glTF/CesiumMan/CesiumMan.glb");
    idx ground = pix_gen_plane(60, 60, 12);
    idx ground_m = pix_make_material(pix_rgba(96, 112, 88), 0.0f, 0.95f);
    idx step  = pix_load_sound("assets/audio/step.wav");
    idx chime = pix_load_sound("assets/audio/chime.wav");

    // heap, not stack: each actor carries two 8 KB poses
    const int count = 8;
    actor* actors = pix_alloc<actor>(count);
    int n = 0;
    for (int i = 0; i < 5 && fox.skinned; i++, n++) {
        actor& a = actors[n];
        a.model = &fox;
        a.anim = pix_model_animator(fox);
        animator_play(a.anim, (idx)(i % 3));
        a.anim.time = i * 0.37f;
        a.anim.speed = 0.8f + (i % 3) * 0.25f;
        a.transform = mat4_mul(mat4_translate((i - 2) * 2.0f, 0, 1.0f), mat4_scale(0.01f, 0.01f, 0.01f));
        a.steps_per_cycle = 4;
        a.last_step = -1;
        a.blend = 1.0f;
        a.pitch = 0.9f + i * 0.05f;
    }
    for (int i = 0; i < 3 && man.skinned; i++, n++) {
        actor& a = actors[n];
        a.model = &man;
        a.anim = pix_model_animator(man);
        a.anim.time = i * 0.5f;
        a.anim.speed = 0.7f + i * 0.3f;
        a.transform = mat4_translate((i - 1) * 2.2f, 0, -2.5f);
        a.steps_per_cycle = 2;
        a.last_step = -1;
        a.blend = 1.0f;
        a.pitch = 0.7f + i * 0.04f;
    }
    if (!n) { pix_log("no glTF models found under assets/glTF"); pix_shutdown(); return 1; }

    float yaw = 0.9f, pitch = 0.35f, dist = 9.0f, step_cooldown = 0.0f, rumble = 0.0f;
    bool paused = false, muted = false;

    while (pix_frame()) {
        float dt = pix_dt();
        const pix_gamepad& pad = pix_get_gamepad(0);
        if (pix_key_pressed(KEY_ESC)) pix_quit();
        if (pix_key_pressed(KEY_SPACE) || pad.buttons[PAD_A].pressed) paused = !paused;
        if (pix_key_pressed('M')) { muted = !muted; pix_set_volume(muted ? 0.0f : 0.8f); }
        int clip = -1;
        for (int k = 0; k < 3; k++) if (pix_key_pressed('1' + k)) clip = k;
        if (pad.buttons[PAD_RIGHT_BUMPER].pressed) clip = (int)(actors[0].anim.clip + 1) % 3;
        if (pad.buttons[PAD_LEFT_BUMPER].pressed)  clip = (int)(actors[0].anim.clip + 2) % 3;
        if (clip >= 0) {
            for (int i = 0; i < n; i++) switch_clip(actors[i], clip);
            pix_play_sound(chime, 0.6f, 1.0f + clip * 0.06f);
        }

        if (pix_mouse_down(PIX_MOUSE_RIGHT)) { yaw += pix_mouse_delta().x * 0.006f; pitch += pix_mouse_delta().y * 0.006f; }
        yaw += pad.right_x * dt * 2.5f;
        pitch = clampf(pitch - pad.right_y * dt * 1.8f, 0.05f, 1.4f);
        dist = clampf(dist - pix_mouse_wheel() * 0.8f, 3.0f, 30.0f);

        if (step_cooldown > 0.0f) step_cooldown -= dt;
        if (rumble > 0.0f) { rumble = maxf(0.0f, rumble - dt * 2.2f); pix_set_gamepad_rumble(pix_get_window(), 0, rumble * 0.6f, rumble); }

        for (int i = 0; i < n; i++) {
            actor& a = actors[i];
            if (!paused) {
                animator_update(a.anim, dt);
                a.prev_time += dt * a.anim.speed;
                a.blend = minf(1.0f, a.blend + dt / 0.3f);
            }
            // cross-fade on local bone transforms, so limbs never shear mid-blend
            animator_sample_blend(a.anim, a.prev_clip, a.prev_time, a.anim.clip, a.anim.time, a.blend, &a.pose);

            float dur = animator_duration(a.anim, a.anim.clip);
            if (paused || dur <= 0.0f) continue;
            int s = (int)(a.anim.time / dur * a.steps_per_cycle);
            if (s == a.last_step) continue;
            a.last_step = s;
            if (step_cooldown > 0.0f) continue;          // eight actors would machine-gun the mix
            step_cooldown = 0.18f;
            float pan = clampf(a.transform.data[12] / 5.0f, -1.0f, 1.0f);
            pix_play_sound(step, 0.7f, a.pitch, false, pan);
            rumble = 0.25f;
        }

        camera cam = {};
        vec3 focus = v3(0, 0.8f, 0);
        cam.position = focus + v3(cosf(yaw) * cosf(pitch), sinf(pitch), sinf(yaw) * cosf(pitch)) * dist;
        cam.direction = focus - cam.position;
        pix_begin_3d(cam);
        pix_draw_mesh(ground, ground_m, mat4_identity());
        for (int i = 0; i < n; i++) pix_draw_model_animated(*actors[i].model, actors[i].pose, actors[i].transform);
        pix_end_3d();

        const animator& lead = actors[0].anim;
        pix_draw_rect_rounded({ 12, 12, 560, 84 }, 10, pix_rgba(0, 0, 0, 140));
        pix_draw_textf(24, 18, 22, PIX_WHITE, "%d actors   clip '%s'  %.2f / %.2f s%s", n,
                       animator_clip_name(lead, lead.clip), lead.time, animator_duration(lead, lead.clip),
                       paused ? "   [paused]" : "");
        pix_draw_textf(24, 52, 17, PIX_LIGHTGRAY, "1/2/3 clip   space pause   M mute   right drag orbit   pad: %s",
                       pad.connected ? "connected" : "none");
        pix_draw_fps((float)pix_width() - 90, 12);
    }
    pix_stop_gamepad_rumble(pix_get_window());
    pix_free(actors);
    pix_shutdown();
    return 0;
}
