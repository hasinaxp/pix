#pragma once
#include <math.h>
#include "dtype.hpp"
#include "math.hpp"
#include "../loader/asset_types.hpp"

// Runtime side of the skinning pipeline: turns a clip + a time into one
// skinning matrix per bone, ready to hand straight to the vertex shader.
//
// Posing is a single linear pass because gltf_loader.hpp guarantees bones are
// topologically sorted (a bone's parent always sits at a lower index), and a
// clip's tracks are dense - tracks[i] belongs to bone i, so there is no lookup.

#define MAX_ANIM_BONES     128  // must match the uBones[] size in shader_sources.hpp
#define MAX_ANIMATOR_CLIPS 16

// a resolved pose: bones[i] = global_transform(i) * inverse_bind(i)
struct animation {
    mat4   bones[MAX_ANIM_BONES];
    size_t bone_count;
};

struct animator {
    skeleton_file_data*       skeleton;
    animation_clip_file_data* clips[MAX_ANIMATOR_CLIPS];
    size_t clip_count;

    // playback state, advanced by animator_update
    idx   clip;
    float time;      // seconds into `clip`
    float speed;
    bool  loop;
    animation pose;  // refreshed by animator_update
};

static animator pix_create_animator(skeleton_file_data* skeleton);
static idx  animator_add_clip(animator& a, animation_clip_file_data* clip);

// the core query: a clip index and a time in seconds -> every bone's matrix.
// `time` is wrapped into the clip automatically, so raw elapsed time is fine.
static void animator_sample(const animator& a, idx clip, float time, animation* out);

// convenience playback on top of it
static void animator_play(animator& a, idx clip, bool loop = true, float speed = 1.0f);
static void animator_update(animator& a, float dt);

static void  animation_rest_pose(const skeleton_file_data& sk, animation* out);
static float animator_duration(const animator& a, idx clip);
static const char* animator_clip_name(const animator& a, idx clip);

// ---------------- implementation ----------------

static animator pix_create_animator(skeleton_file_data* skeleton) {
    animator a = {};
    a.skeleton = skeleton;
    a.speed = 1.0f;
    a.loop = true;
    if (skeleton) animation_rest_pose(*skeleton, &a.pose);
    return a;
}

static idx animator_add_clip(animator& a, animation_clip_file_data* clip) {
    if (!clip || a.clip_count >= MAX_ANIMATOR_CLIPS) return (idx)-1;
    idx id = (idx)a.clip_count;
    a.clips[a.clip_count++] = clip;
    return id;
}

static float animator_duration(const animator& a, idx clip) {
    return (clip < a.clip_count && a.clips[clip]) ? a.clips[clip]->duration : 0.0f;
}

static const char* animator_clip_name(const animator& a, idx clip) {
    return (clip < a.clip_count && a.clips[clip]) ? a.clips[clip]->name : "";
}

// rest pose: every bone at its bind transform, so the skin matrices collapse
// to the skeleton's root transform
static void animation_rest_pose(const skeleton_file_data& sk, animation* out) {
    size_t n = sk.bone_count < MAX_ANIM_BONES ? sk.bone_count : MAX_ANIM_BONES;
    out->bone_count = n;
    mat4 global[MAX_ANIM_BONES];
    for (size_t i = 0; i < n; i++) {
        const bone& b = sk.bones[i];
        mat4 local = mat4_from_trs(b.local_position, b.local_rotation, b.local_scale);
        global[i] = (b.parent < 0) ? mat4_mul(sk.root_transform, local)
                                   : mat4_mul(global[b.parent], local);
        out->bones[i] = mat4_mul(global[i], b.inverse_bind);
    }
}

// keyframe segment containing `t`; keys are ascending so this is a plain bisect
static size_t anim__find_key(const bone_keyframe* k, size_t n, float t) {
    if (n < 2 || t <= k[0].time) return 0;
    if (t >= k[n - 1].time) return n - 1;
    size_t lo = 0, hi = n - 1;
    while (lo + 1 < hi) {
        size_t mid = (lo + hi) >> 1;
        if (k[mid].time <= t) lo = mid; else hi = mid;
    }
    return lo;
}

static void animator_sample(const animator& a, idx clip, float time, animation* out) {
    if (!a.skeleton) { out->bone_count = 0; return; }
    const skeleton_file_data& sk = *a.skeleton;
    const animation_clip_file_data* c = (clip < a.clip_count) ? a.clips[clip] : 0;
    if (!c) { animation_rest_pose(sk, out); return; }

    if (c->duration > 0.0f) {
        time = fmodf(time, c->duration);
        if (time < 0.0f) time += c->duration;
    }

    size_t n = sk.bone_count < MAX_ANIM_BONES ? sk.bone_count : MAX_ANIM_BONES;
    out->bone_count = n;

    mat4 global[MAX_ANIM_BONES];
    for (size_t i = 0; i < n; i++) {
        const bone& b = sk.bones[i];
        vec3 p = b.local_position, s = b.local_scale;
        quat r = b.local_rotation;

        if (i < c->track_count && c->tracks[i].keyframe_count) {
            const bone_animation_track& tr = c->tracks[i];
            size_t k = anim__find_key(tr.keyframes, tr.keyframe_count, time);
            const bone_keyframe& k0 = tr.keyframes[k];
            if (k + 1 < tr.keyframe_count) {
                const bone_keyframe& k1 = tr.keyframes[k + 1];
                float span = k1.time - k0.time;
                float u = (span > 1e-8f) ? (time - k0.time) / span : 0.0f;
                if (u < 0.0f) u = 0.0f; else if (u > 1.0f) u = 1.0f;
                p = v3lerp(k0.position, k1.position, u);
                s = v3lerp(k0.scale, k1.scale, u);
                r = quat_slerp(k0.rotation, k1.rotation, u);
            } else {
                p = k0.position; s = k0.scale; r = k0.rotation;
            }
        }

        mat4 local = mat4_from_trs(p, r, s);
        global[i] = (b.parent < 0) ? mat4_mul(sk.root_transform, local)
                                   : mat4_mul(global[b.parent], local); // parent is already resolved
        out->bones[i] = mat4_mul(global[i], b.inverse_bind);
    }
}

static void animator_play(animator& a, idx clip, bool loop, float speed) {
    a.clip = clip;
    a.time = 0.0f;
    a.loop = loop;
    a.speed = speed;
    animator_sample(a, a.clip, a.time, &a.pose);
}

static void animator_update(animator& a, float dt) {
    float dur = animator_duration(a, a.clip);
    a.time += dt * a.speed;
    if (dur > 0.0f) {
        if (a.loop) {
            a.time = fmodf(a.time, dur);
            if (a.time < 0.0f) a.time += dur;
        } else if (a.time > dur) {
            a.time = dur;
        }
    }
    animator_sample(a, a.clip, a.time, &a.pose);
}
