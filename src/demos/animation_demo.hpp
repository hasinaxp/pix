#pragma once
#include <time.h>
#include <stdio.h>
#include "../core/platform.hpp"
#include "../core/opengl_api.hpp"
#include "../core/opengl_utils.hpp"
#include "../core/shader_sources.hpp"
#include "../core/math.hpp"
#include "../loader/data_loader.hpp"
#include "../core/renderer.hpp"
#include "../core/animation.hpp"
#include "../core/font.hpp"
#include "../core/sprite.hpp"
#include "../core/text.hpp"
#include "../core/sound.hpp"

// End-to-end test of the animation pipeline: glTF load -> skeleton + clips ->
// animator -> skinned draw. Runs two different rigs at once (a 24 bone fox and
// a 19 bone man) so the renderer is exercised with more than one skeleton.
void test_animation_demo();

//----------------------------implementation----------------------------

#define ANIM_WIDTH   1280
#define ANIM_HEIGHT  720
#define FOX_MODEL    "assets/glTF/Fox/Fox.glb"
#define MAN_MODEL    "assets/glTF/CesiumMan/CesiumMan.glb"
#define ANIM_FOXES   5
#define ANIM_MEN     3
#define ANIM_ACTORS  (ANIM_FOXES + ANIM_MEN)
#define STEP_MIN_GAP 0.18f   // seconds between footstep sounds, across all actors

struct demo_actor {
    animator anim;
    idx  mesh;
    idx  material;
    mat4 transform;
    int  steps_per_cycle;  // footfalls per animation loop
    int  last_step;        // which footfall we last played a sound for
    float sound_pitch;     // so the actors do not sound identical
};

// loads a glb and registers its mesh + texture with the renderer;
// returns false (leaving *mesh_id at -1) if the file is not there
static bool anim_demo__load(pix_data_loader& loader, pix_renderer& renderer, const char* path,
                            vec3 tint, idx* mesh_id, idx* mat_id, idx* model_id) {
    idx model = load_model_gltf_file(loader, path);
    if (model == (idx)-1) return false;
    skinned_mesh_file_data* md = get_model_mesh(loader, model);
    if (!md || !get_model_skeleton(loader, model)) return false;

    model_file_data& mf = loader.model_files[model];
    *model_id = model;
    *mesh_id  = load_skinned_mesh(renderer, *md);
    // CesiumMan's texture is a jpeg, which png_loader cannot decode - it falls
    // back to the white texture, so the tint is what gives it its colour
    *mat_id   = load_material_image(renderer,
        mf.image >= 0 ? &loader.image_files[mf.image] : 0, tint, 0.1f, 0.7f);
    return true;
}

void test_animation_demo() {
    srand((unsigned)time(0));

    pix_window window = pix_create_window("pix - animation", ANIM_WIDTH, ANIM_HEIGHT);
    if (!opengl_load_functions()) {
        MessageBoxA(0, "failed to load OpenGL 4.4 functions", "fatal", MB_OK);
        return;
    }

    idx shader = opengl_create_shader(VSHDER_BASIC, FSHDER_BASIC);
    static pix_renderer renderer;
    renderer = pix_create_renderer(ANIM_WIDTH, ANIM_HEIGHT, shader, v3(0.16f, 0.18f, 0.24f));

    idx text_shader = opengl_create_shader(VSHDER_TEXT, FSHDER_TEXT);
    font_data hud_font = pix_load_font_ttf("C:/Windows/Fonts/arial.ttf");
    text_style hud_style = pix_default_style(v4(0.85f, 0.9f, 1.0f, 1.0f));
    hud_style.size = 22.0f;
    pixi_text hud = pix_create_text(&hud_font, "", { 0.0f, 0.0f }, hud_style, 128);
    mat4 ui_proj = mat4_ortho(0.0f, (float)ANIM_WIDTH, (float)ANIM_HEIGHT, 0.0f, -1.0f, 1.0f);

    // ---- audio ----
    // deliberately sparse: quiet one-shots only, no continuous bed. a drone
    // plus a stream of clicky onsets is fatiguing to sit next to.
    static pix_audio audio;
    bool audio_ok = pix_create_audio(audio, 8 * MB);
    set_master_volume(audio, 0.7f);

    pix_data_loader loader = pix_create_data_loader();

    idx snd_step = SOUND_INVALID, snd_chime = SOUND_INVALID;
    idx wav_step  = load_sound_wav_file(loader, "assets/audio/step.wav");
    idx wav_chime = load_sound_wav_file(loader, "assets/audio/chime.wav");
    if (wav_step  != (idx)-1) snd_step  = load_sound(audio, loader.sound_files[wav_step]);
    if (wav_chime != (idx)-1) snd_chime = load_sound(audio, loader.sound_files[wav_chime]);

    idx fox_mesh = (idx)-1, fox_mat = (idx)-1, fox_model = (idx)-1;
    idx man_mesh = (idx)-1, man_mat = (idx)-1, man_model = (idx)-1;
    bool have_fox = anim_demo__load(loader, renderer, FOX_MODEL,
        v3(1.0f, 1.0f, 1.0f), &fox_mesh, &fox_mat, &fox_model);
    bool have_man = anim_demo__load(loader, renderer, MAN_MODEL,
        v3(0.62f, 0.72f, 0.92f), &man_mesh, &man_mat, &man_model);
    if (!have_fox && !have_man) {
        MessageBoxA(0, "no glTF models found under assets/glTF", "fatal", MB_OK);
        return;
    }

    // one animator per actor: actors share a skeleton and its clips, but each
    // keeps its own clip / time / speed
    static demo_actor actors[ANIM_ACTORS];
    size_t actor_count = 0;

    if (have_fox) {
        model_file_data& mf = loader.model_files[fox_model];
        for (int i = 0; i < ANIM_FOXES; i++) {
            demo_actor& a = actors[actor_count++];
            a.anim = pix_create_animator(get_model_skeleton(loader, fox_model));
            for (size_t c = 0; c < mf.animation_count; c++)
                animator_add_clip(a.anim, &loader.animation_clips[mf.first_animation + c]);
            animator_play(a.anim, (idx)(i % (a.anim.clip_count ? a.anim.clip_count : 1)));
            a.anim.time  = (float)i * 0.37f;                 // stagger the phase
            a.anim.speed = 0.8f + (float)(i % 3) * 0.25f;
            a.mesh = fox_mesh;
            a.material = fox_mat;
            a.steps_per_cycle = 4;                           // quadruped
            a.last_step = -1;
            a.sound_pitch = 0.90f + (float)i * 0.05f;
            float x = ((float)i - (ANIM_FOXES - 1) * 0.5f) * 70.0f;
            a.transform = mat4_translate(x, 0.0f, 40.0f);    // fox runs ~154 along Z
        }
    }

    if (have_man) {
        model_file_data& mf = loader.model_files[man_model];
        for (int i = 0; i < ANIM_MEN; i++) {
            demo_actor& a = actors[actor_count++];
            a.anim = pix_create_animator(get_model_skeleton(loader, man_model));
            for (size_t c = 0; c < mf.animation_count; c++)
                animator_add_clip(a.anim, &loader.animation_clips[mf.first_animation + c]);
            animator_play(a.anim, 0);
            a.anim.time  = (float)i * 0.5f;
            a.anim.speed = 0.7f + (float)i * 0.3f;
            a.mesh = man_mesh;
            a.material = man_mat;
            a.steps_per_cycle = 2;                           // biped
            a.last_step = -1;
            a.sound_pitch = 0.70f + (float)i * 0.04f;        // heavier than a fox
            // the man is authored in metres (~1.5 tall); scale him into the
            // fox's centimetre-ish world
            float x = ((float)i - (ANIM_MEN - 1) * 0.5f) * 110.0f;
            a.transform = mat4_mul(mat4_translate(x, 0.0f, -150.0f),
                                   mat4_scale(100.0f, 100.0f, 100.0f));
        }
    }

    // a ground quad so the actors are not floating in the void
    static vertex ground_verts[4];
    const float G = 400.0f;
    ground_verts[0].position = v3(-G, 0.0f, -G);
    ground_verts[1].position = v3( G, 0.0f, -G);
    ground_verts[2].position = v3( G, 0.0f,  G);
    ground_verts[3].position = v3(-G, 0.0f,  G);
    for (int i = 0; i < 4; i++) ground_verts[i].normal = v3(0.0f, 1.0f, 0.0f);
    static uint16_t ground_inds[6] = { 0, 1, 2, 0, 2, 3 };
    mesh_file_data ground_data = {};
    ground_data.vertex_count = 4;
    ground_data.vertex_data  = ground_verts;
    ground_data.index_count  = 6;
    ground_data.index_data   = ground_inds;
    idx ground_mesh = load_mesh(renderer, ground_data);
    idx ground_mat  = load_material(renderer, nullptr, v3(0.30f, 0.34f, 0.40f), 0.0f, 1.0f);

    LARGE_INTEGER freq, prev, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&prev);

    float yaw = 0.9f, pitch = 0.40f, dist = 460.0f;
    float fps = 0.0f;
    bool paused = false;
    bool muted = false;
    float step_cooldown = 0.0f;
    float rumble = 0.0f;

    while (!window.should_close) {
        pix_update_window(window);
        if (window.keystates[KEY_ESC].pressed) break;

        QueryPerformanceCounter(&now);
        float dt = (float)(now.QuadPart - prev.QuadPart) / (float)freq.QuadPart;
        prev = now;
        if (dt > 0.1f) dt = 0.1f;
        fps += ((dt > 0.0f ? 1.0f / dt : fps) - fps) * 0.1f;
        if (step_cooldown > 0.0f) step_cooldown -= dt;
        if (rumble > 0.0f) {
            rumble -= dt * 2.2f;
            if (rumble < 0.0f) rumble = 0.0f;
            pix_set_gamepad_rumble(window, 0, rumble * 0.6f, rumble);
        }

        // --- input (keyboard + pad 0) ---
        const pix_gamepad& gp = window.gamepads[0];
        if (window.keystates[(int)' '].pressed || gp.buttons[PAD_A].pressed) paused = !paused;
        if (gp.buttons[PAD_Y].pressed) {
            muted = !muted;
            set_master_volume(audio, muted ? 0.0f : 0.7f);
        }
        // bumpers step through the clips
        int pad_clip = -1;
        if (gp.buttons[PAD_RIGHT_BUMPER].pressed) pad_clip = (int)((actors[0].anim.clip + 1) % 3);
        if (gp.buttons[PAD_LEFT_BUMPER].pressed)  pad_clip = (int)((actors[0].anim.clip + 2) % 3);
        if (pad_clip >= 0) {
            for (size_t i = 0; i < actor_count; i++)
                if ((size_t)pad_clip < actors[i].anim.clip_count) {
                    animator_play(actors[i].anim, (idx)pad_clip);
                    actors[i].last_step = -1;
                }
            play_sound(audio, snd_chime, 1.0f);
        }

        if (window.keystates[(int)'M'].pressed) {
            muted = !muted;
            set_master_volume(audio, muted ? 0.0f : 0.7f);
        }
        for (int k = 0; k < 3; k++)                       // 1/2/3 switch clip where the rig has one
            if (window.keystates[(int)'1' + k].pressed) {
                for (size_t i = 0; i < actor_count; i++)
                    if ((size_t)k < actors[i].anim.clip_count) {
                        animator_play(actors[i].anim, (idx)k);
                        actors[i].last_step = -1;
                    }
                play_sound(audio, snd_chime, 1.0f, 1.0f + (float)k * 0.06f);
            }

        // right stick orbits; falls back to a slow drift when nothing is driving it
        bool steering = false;
        if (window.keystates[MOUSE_BUTTON_LEFT].held) {
            yaw   += window.mouse_rel_x * 0.005f;
            pitch += window.mouse_rel_y * 0.005f;
            steering = true;
        }
        if (gp.right_x != 0.0f || gp.right_y != 0.0f) {
            yaw   += gp.right_x * dt * 2.5f;
            pitch -= gp.right_y * dt * 1.8f;      // stick up tilts the camera up
            steering = true;
        }
        if (!steering) yaw += dt * 0.15f;
        if (pitch < 0.10f) pitch = 0.10f;
        if (pitch > 1.35f) pitch = 1.35f;
        if (window.keystates[KEY_UP].held)   dist -= dt * 300.0f;
        if (window.keystates[KEY_DOWN].held) dist += dt * 300.0f;
        dist -= gp.right_trigger * dt * 320.0f;   // triggers zoom
        dist += gp.left_trigger  * dt * 320.0f;
        if (dist < 80.0f) dist = 80.0f;
        if (dist > 1400.0f) dist = 1400.0f;

        // --- animate, and fire a footstep whenever a stride passes a footfall ---
        if (!paused) {
            for (size_t i = 0; i < actor_count; i++) {
                demo_actor& a = actors[i];
                animator_update(a.anim, dt);

                float dur = animator_duration(a.anim, a.anim.clip);
                if (dur <= 0.0f || a.steps_per_cycle <= 0) continue;
                int step = (int)(a.anim.time / dur * (float)a.steps_per_cycle);
                if (step == a.last_step) continue;
                a.last_step = step;

                // eight actors striding at once would machine-gun the mix, so
                // let at most one footfall through every STEP_MIN_GAP seconds
                if (step_cooldown > 0.0f) continue;
                step_cooldown = STEP_MIN_GAP;

                float x = a.transform.data[12];              // place it in the stereo field
                float pan = x / 260.0f;
                if (pan < -1.0f) pan = -1.0f;
                if (pan >  1.0f) pan =  1.0f;
                play_sound(audio, snd_step, 0.80f, a.sound_pitch, false, pan);
                rumble = 0.28f;                              // a light tap, decayed below
            }
        }

        camera cam = {};
        cam.position = v3(cosf(yaw) * cosf(pitch) * dist,
                          sinf(pitch) * dist + 60.0f,
                          sinf(yaw) * cosf(pitch) * dist);
        cam.direction = v3norm(v3sub(v3(0.0f, 60.0f, 0.0f), cam.position));
        cam.up = v3(0.0f, 1.0f, 0.0f);

        begin_frame(renderer, cam);

        pix_render_instance ground = {};
        ground.mesh = ground_mesh;
        ground.material = ground_mat;
        ground.trainsform = mat4_identity();
        push_instance(renderer, ground);

        for (size_t i = 0; i < actor_count; i++) {
            pix_render_instance inst = {};
            inst.mesh = actors[i].mesh;
            inst.material = actors[i].material;
            inst.trainsform = actors[i].transform;
            push_animated_instance(renderer, inst, actors[i].anim.pose);
        }

        end_frame(renderer);

        const animator& lead = actors[0].anim;
        char buf[288];
        snprintf(buf, sizeof(buf),
            "%.0f fps   %zu actors   fox %zu bones / man %zu bones   clip '%s'  %.2f/%.2fs%s\n"
            "audio %s   %zu voices%s   |   pad: %s   |   1/2/3 clip   space pause   m mute   drag orbit",
            fps, actor_count,
            have_fox ? get_model_skeleton(loader, fox_model)->bone_count : (size_t)0,
            have_man ? get_model_skeleton(loader, man_model)->bone_count : (size_t)0,
            animator_clip_name(lead, lead.clip), lead.time, animator_duration(lead, lead.clip),
            paused ? "   [PAUSED]" : "",
            audio_ok ? "on" : "unavailable", active_voice_count(audio), muted ? "   [MUTED]" : "",
            gp.connected ? "connected (stick orbit, triggers zoom, A pause, LB/RB clip)" : "none");
        hud.position = { 16.0f, 30.0f };
        pix_update_text(hud, buf);
        draw_text(hud, text_shader, ui_proj);
    }

    pix_stop_gamepad_rumble(window);
    pix_destroy_audio(audio);
    pix_destroy_text(hud);
    pix_free_font(hud_font);
    pix_destroy_data_loader(loader);
    pix_destory_renderer(renderer);
}
