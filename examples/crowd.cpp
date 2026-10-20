// A crowd: a thousand animated characters through pix_anim_system.
//
//   * every actor picks a clip from its speed (idle / walk / run) and the
//     system cross-fades between them
//   * O toggles a masked overlay: foxes keep running with their legs while
//     the neck and head play the "look around" clip
//   * footsteps are clip events; the ones near the camera make a sound
//   * far actors are sampled at a lower rate (distance LOD), all of it spread
//     over the worker threads
//   * the renderer skins the whole crowd with instanced draws - a few draw
//     calls for every fox on screen, shadows included
//   build.bat crowd
//
// Controls: right drag orbit, wheel zoom, O overlay, +/- crowd size.
#define PIX_IMPLEMENTATION
#include "pix.h"

#define MAX_ACTORS 1600
#define FIELD      60.0f

struct wanderer {
    idx   anim;          // actor in the animation system
    bool  is_fox;
    vec3  pos;
    float heading, speed, target_speed, think;
    int   gait;          // 0 idle, 1 walk, 2 run
};

int main() {
    if (!pix_init("pix - crowd", 1280, 720)) return 1;
    pix_renderer& r = pix_get_renderer();
    pix_shadows().distance = 60.0f;
    pix_time_of_day(16.0f);

    pix_model fox = pix_load_model("assets/glTF/Fox/Fox.glb");
    pix_model man = pix_load_model("assets/glTF/CesiumMan/CesiumMan.glb");
    if (!fox.skinned) { pix_log("assets/glTF/Fox/Fox.glb is needed"); pix_shutdown(); return 1; }
    idx ground = pix_gen_plane(FIELD * 2.5f, FIELD * 2.5f, 24);
    idx ground_m = pix_make_material(pix_rgba(92, 110, 80), 0.0f, 0.95f);
    idx step = pix_load_sound("assets/audio/step.wav");

    pix_anim_system& anims = pix_get_animations();
    // footstep events: the frames a foot lands in each fox gait
    enum { EV_STEP = 1 };
    pix_anim_add_event(anims, &fox.clips[1], 0.10f, EV_STEP);    // walk
    pix_anim_add_event(anims, &fox.clips[1], 0.45f, EV_STEP);
    pix_anim_add_event(anims, &fox.clips[2], 0.20f, EV_STEP);    // run
    pix_anim_add_event(anims, &fox.clips[2], 0.75f, EV_STEP);
    // the overlay mask: the neck and everything above it
    float* head_mask = pix_anim_bone_mask(*fox.skeleton, "b_Neck_04");

    wanderer* crowd = pix_alloc<wanderer>(MAX_ACTORS);
    int count = 0, wanted = 1000;
    rng g = rng_seed(42);
    bool overlay = false;
    float yaw = 0.8f, pitch = 0.5f, dist = 40.0f, step_cooldown = 0.0f;

    while (pix_frame()) {
        float dt = pix_dt();
        if (pix_key_pressed(KEY_ESC)) pix_quit();
        if (pix_key_pressed('O')) {
            overlay = !overlay;
            for (int i = 0; i < count; i++)
                if (crowd[i].is_fox) pix_anim_overlay(anims, crowd[i].anim, 0, overlay ? 1.0f : 0.0f, head_mask, 0.4f);
        }
        if (pix_key_pressed(0xBB) || pix_key_pressed(0x6B)) wanted = (int)minf((float)MAX_ACTORS, (float)wanted + 200);   // +
        if (pix_key_pressed(0xBD) || pix_key_pressed(0x6D)) wanted = (int)maxf(0.0f, (float)wanted - 200);               // -
        if (pix_mouse_down(PIX_MOUSE_RIGHT)) {
            yaw += pix_mouse_delta().x * 0.006f;
            pitch = clampf(pitch + pix_mouse_delta().y * 0.006f, 0.1f, 1.45f);
        }
        dist = clampf(dist - pix_mouse_wheel() * 3.0f, 6.0f, 120.0f);

        // grow / shrink the crowd
        while (count < wanted) {
            wanderer& w = crowd[count];
            w.is_fox = !man.skinned || rng_chance(g, 0.8f);
            const pix_model& m = w.is_fox ? fox : man;
            w.anim = pix_anim_add(anims, m.skeleton, m.clips, m.clip_count);
            if (w.anim == PIX_INVALID) break;
            w.pos = v3(rng_range(g, -FIELD, FIELD), 0, rng_range(g, -FIELD, FIELD));
            w.heading = rng_range(g, 0, PIX_TAU);
            w.think = rng_range(g, 0, 3);
            w.gait = -1;
            pix_anim_set_time(anims, w.anim, rng_range(g, 0, 2));
            if (overlay && w.is_fox) pix_anim_overlay(anims, w.anim, 0, 1.0f, head_mask, 0.0f);
            count++;
        }
        while (count > wanted) pix_anim_remove(anims, crowd[--count].anim);

        // brains: wander, pick a speed, pick the gait that matches it
        for (int i = 0; i < count; i++) {
            wanderer& w = crowd[i];
            w.think -= dt;
            if (w.think <= 0.0f) {
                w.think = rng_range(g, 1.5f, 5.0f);
                float roll = rng_float(g);
                w.target_speed = !w.is_fox ? (roll < 0.2f ? 0.0f : 1.4f)
                                           : (roll < 0.2f ? 0.0f : roll < 0.65f ? 1.3f : 4.5f);
                w.heading += rng_range(g, -1.2f, 1.2f);
            }
            if (fabsf(w.pos.x) > FIELD || fabsf(w.pos.z) > FIELD)       // turn back toward the middle
                w.heading = damp_angle(w.heading, atan2f(-w.pos.x, -w.pos.z), 3.0f, dt);
            w.speed = damp(w.speed, w.target_speed, 3.0f, dt);
            w.pos += v3(sinf(w.heading), 0, cosf(w.heading)) * (w.speed * dt);

            int gait = w.speed < 0.3f ? 0 : (w.speed < 2.6f || !w.is_fox ? 1 : 2);
            if (!w.is_fox) gait = 0;                                      // the man has one clip: a walk
            if (gait != w.gait) {
                w.gait = gait;
                pix_anim_play(anims, w.anim, (idx)gait, 0.3f);            // fox clips: 0 idle, 1 walk, 2 run
            }
            // play the clip at the speed the feet actually move
            float natural = w.is_fox ? (gait == 2 ? 4.5f : 1.3f) : 1.4f;
            pix_anim_set_speed(anims, w.anim, gait == 0 && w.is_fox ? 1.0f : maxf(0.3f, w.speed / natural));
            pix_anim_set_position(anims, w.anim, w.pos);
        }

        camera cam = {};
        cam.position = v3(cosf(yaw) * cosf(pitch), sinf(pitch), sinf(yaw) * cosf(pitch)) * dist;
        cam.direction = v3(0, 0, 0) - cam.position;
        pix_anim_update(anims, dt, cam.position);

        // footstep sounds, only from feet near the camera, a few per second at most
        int nev = 0;
        const pix_anim_fired* ev = pix_anim_events(anims, &nev);
        step_cooldown -= dt;
        for (int e = 0; e < nev && step_cooldown <= 0.0f; e++) {
            for (int i = 0; i < count; i++) {
                if (crowd[i].anim != ev[e].actor) continue;
                float d = v3dist(crowd[i].pos, cam.position);
                if (d < 15.0f) { pix_play_sound(step, 0.5f * (1.0f - d / 15.0f), rng_range(g, 0.9f, 1.1f)); step_cooldown = 0.12f; }
                break;
            }
        }

        pix_begin_3d(cam);
        pix_draw_mesh(ground, ground_m, mat4_identity());
        for (int i = 0; i < count; i++) {
            const wanderer& w = crowd[i];
            int nb = 0;
            const mat4* bones = pix_anim_bones(anims, w.anim, &nb);
            mat4 m = mat4_mul(mat4_translate(w.pos.x, 0, w.pos.z), mat4_rotate_y(w.heading));
            if (w.is_fox) m = mat4_mul(m, mat4_scale(0.01f, 0.01f, 0.01f));
            pix_draw_model_bones(w.is_fox ? fox : man, bones, nb, m);
        }
        pix_end_3d();

        pix_draw_rect_rounded({ 12, 12, 600, 84 }, 10, pix_rgba(0, 0, 0, 140));
        pix_draw_textf(24, 18, 22, PIX_WHITE, "%d actors   %ld sampled this frame (LOD)   %d draws",
                       count, anims.sampled, r.frame_draws);
        pix_draw_textf(24, 52, 17, PIX_LIGHTGRAY, "O head overlay (%s)   +/- crowd size   right drag orbit   wheel zoom",
                       overlay ? "on" : "off");
        pix_draw_fps((float)pix_width() - 90, 12);
    }
    pix_free(head_mask);
    pix_free(crowd);
    pix_shutdown();
    return 0;
}
