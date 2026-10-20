#ifndef PIX_ANIMATION_HPP
#define PIX_ANIMATION_HPP
#include <math.h>
#include <string.h>
#include "dtype.hpp"
#include "math.hpp"
#include "jobs.hpp"
#include "../loader/asset_types.hpp"

// Runtime side of the skinning pipeline: turns a clip + a time into one
// skinning matrix per bone, ready to hand straight to the vertex shader.
//
// Posing is a single linear pass because gltf_loader.hpp guarantees bones are
// topologically sorted (a bone's parent always sits at a lower index), and a
// clip's tracks are dense - tracks[i] belongs to bone i, so there is no lookup.

#define MAX_ANIM_BONES     128  // bones per rig
// Clips one animator can hold. A character rig from a typical asset pack
// ships twenty to thirty of them, and an animator that silently drops the
// tail of that list is a bug nobody sees until a rarely-played clip turns
// out to be the bind pose - so the ceiling is set well above one file's
// worth, with room for clips retargeted onto the rig from elsewhere.
#define MAX_ANIMATOR_CLIPS 40

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

PIXDEF animator pix_create_animator(skeleton_file_data* skeleton);
PIXDEF idx  animator_add_clip(animator& a, animation_clip_file_data* clip);

// the core query: a clip index and a time in seconds -> every bone's matrix.
// `time` is wrapped into the clip automatically, so raw elapsed time is fine.
PIXDEF void animator_sample(const animator& a, idx clip, float time, animation* out);

// Two clips at once, blended `weight` of the way from `a_clip` to `b_clip`.
//
// The blend happens on each bone's local translation/rotation/scale, before the
// hierarchy is walked - never on the finished skinning matrices. Lerping two
// matrices that differ by a large rotation shears and shrinks the limb between
// them, and between two poses that are far apart that is worse than the pop it
// was meant to hide.
PIXDEF void animator_sample_blend(const animator& a, idx a_clip, float a_time,
                                  idx b_clip, float b_time, float weight, animation* out);

// convenience playback on top of it
PIXDEF void animator_play(animator& a, idx clip, bool loop = true, float speed = 1.0f);
PIXDEF void animator_update(animator& a, float dt);

PIXDEF void  animation_rest_pose(const skeleton_file_data& sk, animation* out);
PIXDEF float animator_duration(const animator& a, idx clip);
PIXDEF const char* animator_clip_name(const animator& a, idx clip);

// Index of the first bone whose name matches any of `names` (rigs disagree on
// naming, so pass several candidates), or -1.
PIXDEF int  animator_find_bone(const animator& a, const char* const* names, int count);
// A bone's transform in the character's model space for a given pose - where
// to put a prop held in a hand or a hat on a head.
PIXDEF mat4 animation_bone_world(const animator& a, const animation& pose, int bone);


// ============================================================================
// ---- the animation system: many characters at once ----
//
// `animator` above is one character playing one clip - fine for a player and a
// few NPCs. pix_anim_system is for crowds: hundreds or thousands of actors,
//
//   * every actor's bone matrices in ONE packed palette (only as many bones
//     as its rig has), which the renderer uploads in one go and skins with
//     instanced draws - a crowd of one character model is one draw call
//   * a base layer that cross-fades between clips, plus an overlay layer
//     with a per-bone mask (wave while walking, aim while running)
//   * clip events (footsteps, hit frames), collected thread-safely
//   * distance LOD: far actors sample at a lower rate
//   * updated in parallel across the job pool
//
//   pix_anim_system anims;
//   pix_anim_system_create(anims, 1024, &jobs);
//   idx a = pix_anim_add(anims, skeleton, clips, clip_count);
//   pix_anim_play(anims, a, walk_clip);                  // 0.2 s cross-fade by default
//   ...each frame...
//   pix_anim_set_position(anims, a, pos);                // for LOD
//   pix_anim_update(anims, dt, camera_pos);
//   int n; const mat4* bones = pix_anim_bones(anims, a, &n);
//   push_skinned(renderer, mesh, material, transform, bones, n);

struct pix_anim_layer {
    idx   clip, prev_clip;          // playing / fading out (PIX_INVALID = none)
    float time, prev_time;
    float speed;
    float fade, fade_duration;      // cross-fade progress, seconds
    float weight, target_weight;    // overlay layer strength, eased toward target
    bool  loop, prev_loop;
};

struct pix_anim_actor {
    const skeleton_file_data*  skeleton;
    animation_clip_file_data*  clips;       // the rig's clips, contiguous
    size_t                     clip_count;
    pix_anim_layer base;
    pix_anim_layer overlay;
    const float*   overlay_mask;            // per-bone overlay weight 0..1 (null = every bone)
    vec3     position;                      // world position, for LOD
    float    pending_dt;                    // time not yet sampled (LOD skips frames)
    uint32_t bone_offset;                   // this actor's slice of the palette
    uint32_t bone_capacity;
    bool     alive;
    bool     finished;                      // a non-looping base clip reached its end
};

struct pix_anim_event_def { const animation_clip_file_data* clip; float time; int id; };
struct pix_anim_fired     { idx actor; int id; };

#define PIX_ANIM_MAX_EVENT_DEFS 256
#define PIX_ANIM_MAX_FIRED      1024

struct pix_anim_system {
    pix_anim_actor* actors;
    uint32_t capacity;
    uint32_t count;              // high-water mark of used slots
    mat4*    palette;            // every actor's skinning matrices
    uint32_t palette_used, palette_capacity;
    pix_jobs* jobs;              // optional: null updates on the calling thread
    // LOD: full rate within lod_near of the eye, down to lod_min_rate (Hz) by lod_far
    float lod_near, lod_far, lod_min_rate;
    pix_anim_event_def event_defs[PIX_ANIM_MAX_EVENT_DEFS];
    int  event_def_count;
    pix_anim_fired fired[PIX_ANIM_MAX_FIRED];
    volatile long fired_count;
    volatile long sampled;       // actors actually re-posed by the last update (LOD skips the rest)
    float dt;                    // this update's step, read by the workers
    vec3  eye;
};

PIXDEF void pix_anim_system_create(pix_anim_system& s, uint32_t capacity, pix_jobs* jobs = nullptr);
PIXDEF void pix_anim_system_destroy(pix_anim_system& s);
PIXDEF idx  pix_anim_add(pix_anim_system& s, const skeleton_file_data* skeleton,
                         animation_clip_file_data* clips, size_t clip_count);
PIXDEF void pix_anim_remove(pix_anim_system& s, idx actor);
// cross-fades from whatever is playing to `clip` over `fade` seconds
PIXDEF void pix_anim_play(pix_anim_system& s, idx actor, idx clip, float fade = 0.2f,
                          bool loop = true, float speed = 1.0f);
PIXDEF void pix_anim_set_speed(pix_anim_system& s, idx actor, float speed);
PIXDEF void pix_anim_set_time(pix_anim_system& s, idx actor, float seconds);
// Plays `clip` on top of the base layer on the bones `mask` selects (see
// pix_anim_bone_mask), fading its weight to `weight`. Weight 0 fades it out.
PIXDEF void pix_anim_overlay(pix_anim_system& s, idx actor, idx clip, float weight,
                             const float* mask = nullptr, float fade = 0.2f, bool loop = true);
PIXDEF void pix_anim_set_position(pix_anim_system& s, idx actor, vec3 position);
// fires `id` whenever any actor's base clip passes `time` seconds into `clip`
PIXDEF void pix_anim_add_event(pix_anim_system& s, const animation_clip_file_data* clip, float time, int id);
// Advances and samples every actor (in parallel when a job pool was given).
PIXDEF void pix_anim_update(pix_anim_system& s, float dt, vec3 eye);
// Events fired by the last update.
PIXDEF const pix_anim_fired* pix_anim_events(const pix_anim_system& s, int* count);
// The actor's skinning matrices, valid until the next update.
PIXDEF const mat4* pix_anim_bones(const pix_anim_system& s, idx actor, int* count);
PIXDEF float pix_anim_time(const pix_anim_system& s, idx actor);
PIXDEF bool  pix_anim_finished(const pix_anim_system& s, idx actor);
PIXDEF idx   pix_anim_find_clip(const pix_anim_system& s, idx actor, const char* name);
// A per-bone weight array: 1 for `root_bone` and everything under it, 0
// elsewhere (e.g. "Spine2" for an upper-body mask). Global (free with
// pix_free) or PIX_MEM_SCOPED.
PIXDEF float* pix_anim_bone_mask(const skeleton_file_data& skeleton, const char* root_bone,
                                 unsigned mem = PIX_MEM_GLOBAL);
#endif // PIX_ANIMATION_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_ANIMATION_IMPL)
#define PIX_ANIMATION_IMPL
#include <windows.h>

animator pix_create_animator(skeleton_file_data* skeleton) {
    animator a = {};
    a.skeleton = skeleton;
    a.speed = 1.0f;
    a.loop = true;
    if (skeleton) animation_rest_pose(*skeleton, &a.pose);
    return a;
}

idx animator_add_clip(animator& a, animation_clip_file_data* clip) {
    if (!clip || a.clip_count >= MAX_ANIMATOR_CLIPS) return (idx)-1;
    idx id = (idx)a.clip_count;
    a.clips[a.clip_count++] = clip;
    return id;
}

float animator_duration(const animator& a, idx clip) {
    return (clip < a.clip_count && a.clips[clip]) ? a.clips[clip]->duration : 0.0f;
}

const char* animator_clip_name(const animator& a, idx clip) {
    return (clip < a.clip_count && a.clips[clip]) ? a.clips[clip]->name : "";
}

// rest pose: every bone at its bind transform, so the skin matrices collapse
// to the skeleton's root transform
void animation_rest_pose(const skeleton_file_data& sk, animation* out) {
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

void animator_sample(const animator& a, idx clip, float time, animation* out) {
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

// one bone's local pose out of a clip; falls back to the bind pose when the
// clip has no track for it
static void anim__local(const skeleton_file_data& sk, const animation_clip_file_data* c,
                        size_t i, float time, vec3* p, quat* r, vec3* s) {
    const bone& b = sk.bones[i];
    *p = b.local_position; *s = b.local_scale; *r = b.local_rotation;
    if (!c || i >= c->track_count || !c->tracks[i].keyframe_count) return;

    const bone_animation_track& tr = c->tracks[i];
    size_t k = anim__find_key(tr.keyframes, tr.keyframe_count, time);
    const bone_keyframe& k0 = tr.keyframes[k];
    if (k + 1 >= tr.keyframe_count) { *p = k0.position; *s = k0.scale; *r = k0.rotation; return; }

    const bone_keyframe& k1 = tr.keyframes[k + 1];
    float span = k1.time - k0.time;
    float u = (span > 1e-8f) ? (time - k0.time) / span : 0.0f;
    if (u < 0.0f) u = 0.0f; else if (u > 1.0f) u = 1.0f;
    *p = v3lerp(k0.position, k1.position, u);
    *s = v3lerp(k0.scale, k1.scale, u);
    *r = quat_slerp(k0.rotation, k1.rotation, u);
}

static float anim__wrap(const animation_clip_file_data* c, float time) {
    if (!c || c->duration <= 0.0f) return time;
    time = fmodf(time, c->duration);
    return time < 0.0f ? time + c->duration : time;
}

void animator_sample_blend(const animator& a, idx a_clip, float a_time,
                                  idx b_clip, float b_time, float weight, animation* out) {
    if (!a.skeleton) { out->bone_count = 0; return; }
    if (weight <= 0.001f) { animator_sample(a, a_clip, a_time, out); return; }
    if (weight >= 0.999f) { animator_sample(a, b_clip, b_time, out); return; }

    const skeleton_file_data& sk = *a.skeleton;
    const animation_clip_file_data* ca = (a_clip < a.clip_count) ? a.clips[a_clip] : 0;
    const animation_clip_file_data* cb = (b_clip < a.clip_count) ? a.clips[b_clip] : 0;
    a_time = anim__wrap(ca, a_time);
    b_time = anim__wrap(cb, b_time);

    size_t n = sk.bone_count < MAX_ANIM_BONES ? sk.bone_count : MAX_ANIM_BONES;
    out->bone_count = n;

    mat4 global[MAX_ANIM_BONES];
    for (size_t i = 0; i < n; i++) {
        vec3 pa, sa, pb, sb; quat ra, rb;
        anim__local(sk, ca, i, a_time, &pa, &ra, &sa);
        anim__local(sk, cb, i, b_time, &pb, &rb, &sb);

        mat4 local = mat4_from_trs(v3lerp(pa, pb, weight),
                                   quat_slerp(ra, rb, weight),
                                   v3lerp(sa, sb, weight));
        const bone& b = sk.bones[i];
        global[i] = (b.parent < 0) ? mat4_mul(sk.root_transform, local)
                                   : mat4_mul(global[b.parent], local);
        out->bones[i] = mat4_mul(global[i], b.inverse_bind);
    }
}

void animator_play(animator& a, idx clip, bool loop, float speed) {
    a.clip = clip;
    a.time = 0.0f;
    a.loop = loop;
    a.speed = speed;
    animator_sample(a, a.clip, a.time, &a.pose);
}

void animator_update(animator& a, float dt) {
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

// ---- attaching something to a bone ----
//
// A pose holds skinning matrices - global(i) * inverse_bind(i) - because that
// is what the vertex shader wants and nothing else needs more. Parenting a
// prop to a bone does: a held object has to be placed at the *global*
// transform of that bone, in the character's own model space, and the skinning
// matrix is exactly that transform with the bind pose already divided out of
// it. Multiplying the bind pose back in is what these two do.
//
// Bone names differ between rigs, so the lookup takes a list of candidates and
// returns the first that exists. It costs one 4x4 inverse per call, which is
// why it is a per-attachment, per-frame call rather than something the pose
// does for all 128 bones.
int animator_find_bone(const animator& a, const char* const* names, int count) {
    if (!a.skeleton) return -1;
    for (int n = 0; n < count; n++) {
        if (!names[n]) break;
        for (size_t i = 0; i < a.skeleton->bone_count; i++)
            if (strcmp(a.skeleton->bones[i].name, names[n]) == 0) return (int)i;
    }
    return -1;
}

mat4 animation_bone_world(const animator& a, const animation& pose, int bone) {
    if (!a.skeleton || bone < 0 || (size_t)bone >= pose.bone_count) return mat4_identity();
    return mat4_mul(pose.bones[bone], mat4_inverse(a.skeleton->bones[bone].inverse_bind));
}


// ---------------- the animation system ----------------

void pix_anim_system_create(pix_anim_system& s, uint32_t capacity, pix_jobs* jobs) {
    memset(&s, 0, sizeof(s));
    s.capacity = capacity ? capacity : 1;
    s.actors = pix_alloc<pix_anim_actor>(s.capacity);
    s.palette_capacity = 4096;
    s.palette = pix_alloc<mat4>(s.palette_capacity);
    s.jobs = jobs;
    s.lod_near = 20.0f;
    s.lod_far = 80.0f;
    s.lod_min_rate = 10.0f;
}

void pix_anim_system_destroy(pix_anim_system& s) {
    pix_free(s.actors);
    pix_free(s.palette);
    memset(&s, 0, sizeof(s));
}

static pix_anim_actor* anim__actor(pix_anim_system& s, idx a) {
    return (a < s.count && s.actors[a].alive) ? &s.actors[a] : 0;
}

idx pix_anim_add(pix_anim_system& s, const skeleton_file_data* sk, animation_clip_file_data* clips,
                 size_t clip_count) {
    if (!sk || !sk->bone_count) return PIX_INVALID;
    uint32_t bones = (uint32_t)(sk->bone_count < MAX_ANIM_BONES ? sk->bone_count : MAX_ANIM_BONES);
    // a free slot whose palette slice is big enough, else a fresh slot
    idx slot = PIX_INVALID;
    for (uint32_t i = 0; i < s.count; i++)
        if (!s.actors[i].alive && s.actors[i].bone_capacity >= bones) { slot = i; break; }
    if (slot == PIX_INVALID) {
        if (s.count >= s.capacity) return PIX_INVALID;
        slot = s.count++;
        if (s.palette_used + bones > s.palette_capacity) {
            uint32_t cap = s.palette_capacity;
            while (s.palette_used + bones > cap) cap *= 2;
            mat4* grown = (mat4*)pix_mem_realloc(s.palette, cap * sizeof(mat4));
            if (!grown) { s.count--; return PIX_INVALID; }
            s.palette = grown;
            s.palette_capacity = cap;
        }
        s.actors[slot].bone_offset = s.palette_used;
        s.actors[slot].bone_capacity = bones;
        s.palette_used += bones;
    }
    pix_anim_actor& a = s.actors[slot];
    uint32_t offset = a.bone_offset, capacity = a.bone_capacity;
    memset(&a, 0, sizeof(a));
    a.bone_offset = offset;
    a.bone_capacity = capacity;
    a.skeleton = sk;
    a.clips = clips;
    a.clip_count = clip_count;
    a.base.clip = a.base.prev_clip = PIX_INVALID;
    a.overlay.clip = a.overlay.prev_clip = PIX_INVALID;
    a.base.speed = a.overlay.speed = 1.0f;
    a.base.loop = true;
    a.alive = true;
    // start from the rest pose so the actor is drawable before its first update
    animation rest;
    animation_rest_pose(*sk, &rest);
    memcpy(s.palette + a.bone_offset, rest.bones, bones * sizeof(mat4));
    return slot;
}

void pix_anim_remove(pix_anim_system& s, idx a) {
    if (pix_anim_actor* p = anim__actor(s, a)) p->alive = false;
}

static void anim__start(pix_anim_layer& L, idx clip, float fade, bool loop, float speed) {
    if (fade > 0.0f && L.clip != PIX_INVALID) {
        L.prev_clip = L.clip;
        L.prev_time = L.time;
        L.prev_loop = L.loop;
        L.fade = 0.0f;
        L.fade_duration = fade;
    } else {
        L.prev_clip = PIX_INVALID;
        L.fade_duration = 0.0f;
    }
    L.clip = clip;
    L.time = 0.0f;
    L.loop = loop;
    L.speed = speed;
}

void pix_anim_play(pix_anim_system& s, idx a, idx clip, float fade, bool loop, float speed) {
    pix_anim_actor* p = anim__actor(s, a);
    if (!p || clip >= p->clip_count) return;
    if (p->base.clip == clip && !p->finished) { p->base.speed = speed; p->base.loop = loop; return; }
    anim__start(p->base, clip, fade, loop, speed);
    p->finished = false;
}

void pix_anim_set_speed(pix_anim_system& s, idx a, float speed) { if (pix_anim_actor* p = anim__actor(s, a)) p->base.speed = speed; }
void pix_anim_set_time(pix_anim_system& s, idx a, float t)      { if (pix_anim_actor* p = anim__actor(s, a)) p->base.time = t; }
void pix_anim_set_position(pix_anim_system& s, idx a, vec3 pos)  { if (pix_anim_actor* p = anim__actor(s, a)) p->position = pos; }

void pix_anim_overlay(pix_anim_system& s, idx a, idx clip, float weight, const float* mask, float fade, bool loop) {
    pix_anim_actor* p = anim__actor(s, a);
    if (!p) return;
    p->overlay_mask = mask;
    p->overlay.target_weight = weight < 0.0f ? 0.0f : (weight > 1.0f ? 1.0f : weight);
    p->overlay.fade_duration = fade;
    if (clip < p->clip_count && clip != p->overlay.clip) {
        p->overlay.clip = clip;
        p->overlay.time = 0.0f;
        p->overlay.loop = loop;
        p->overlay.prev_clip = PIX_INVALID;
    }
    if (fade <= 0.0f) p->overlay.weight = p->overlay.target_weight;
}

void pix_anim_add_event(pix_anim_system& s, const animation_clip_file_data* clip, float time, int id) {
    if (s.event_def_count >= PIX_ANIM_MAX_EVENT_DEFS) return;
    pix_anim_event_def& e = s.event_defs[s.event_def_count++];
    e.clip = clip; e.time = time; e.id = id;
}

static const animation_clip_file_data* anim__clip(const pix_anim_actor& a, idx c) {
    return c < a.clip_count ? &a.clips[c] : 0;
}

// advances a layer's clock; returns the time before the step for event checks
static float anim__advance(pix_anim_layer& L, const animation_clip_file_data* c, float dt, bool* finished) {
    float before = L.time;
    L.time += dt * L.speed;
    if (c && c->duration > 0.0f) {
        if (L.loop) {
            // keep the clock small: an actor left running for hours must not lose precision
            if (L.time >= c->duration || L.time < 0.0f) L.time = fmodf(L.time, c->duration) + (L.time < 0.0f ? c->duration : 0.0f);
        } else if (L.time >= c->duration) {
            L.time = c->duration;
            if (finished) *finished = true;
        }
    }
    if (L.prev_clip != PIX_INVALID) {
        L.prev_time += dt * L.speed;
        L.fade += dt;
        if (L.fade >= L.fade_duration) L.prev_clip = PIX_INVALID;
    }
    return before;
}

// one bone's local pose from a layer, cross-fade included
static void anim__layer_bone(const pix_anim_actor& a, const pix_anim_layer& L, size_t bone,
                             vec3* p, quat* r, vec3* s) {
    const animation_clip_file_data* c = anim__clip(a, L.clip);
    anim__local(*a.skeleton, c, bone, anim__wrap(c, L.time), p, r, s);
    if (L.prev_clip == PIX_INVALID || L.fade_duration <= 0.0f) return;
    const animation_clip_file_data* pc = anim__clip(a, L.prev_clip);
    vec3 pp, ps; quat pr;
    anim__local(*a.skeleton, pc, bone, anim__wrap(pc, L.prev_time), &pp, &pr, &ps);
    float w = clampf(L.fade / L.fade_duration, 0.0f, 1.0f);
    w = w * w * (3.0f - 2.0f * w);   // ease, so the blend has no visible start or stop
    *p = v3lerp(pp, *p, w);
    *s = v3lerp(ps, *s, w);
    *r = quat_slerp(pr, *r, w);
}

static void anim__update_actor(pix_anim_system& s, idx id) {
    pix_anim_actor& a = s.actors[id];
    if (!a.alive || !a.skeleton) return;

    // distance LOD: far actors accumulate time and sample less often
    a.pending_dt += s.dt;
    float d = v3dist(a.position, s.eye);
    if (d > s.lod_near && s.lod_far > s.lod_near) {
        float t = clampf((d - s.lod_near) / (s.lod_far - s.lod_near), 0.0f, 1.0f);
        float interval = lerpf(0.0f, 1.0f / (s.lod_min_rate > 0.1f ? s.lod_min_rate : 0.1f), t);
        if (a.pending_dt < interval) return;
    }
    float dt = a.pending_dt;
    a.pending_dt = 0.0f;
    InterlockedIncrement(&s.sampled);

    const animation_clip_file_data* base_clip = anim__clip(a, a.base.clip);
    float before = anim__advance(a.base, base_clip, dt, &a.finished);
    if (a.overlay.clip != PIX_INVALID) {
        anim__advance(a.overlay, anim__clip(a, a.overlay.clip), dt, 0);
        float rate = a.overlay.fade_duration > 0.0f ? dt / a.overlay.fade_duration : 1.0f;
        a.overlay.weight = approachf(a.overlay.weight, a.overlay.target_weight, rate);
    }

    // events: a crossing of the event time between the old clock and the new
    if (base_clip && s.event_def_count) {
        float after = a.base.time;
        bool wrapped = after < before;
        for (int e = 0; e < s.event_def_count; e++) {
            const pix_anim_event_def& ev = s.event_defs[e];
            if (ev.clip != base_clip) continue;
            bool hit = wrapped ? (ev.time > before || ev.time <= after) : (ev.time > before && ev.time <= after);
            if (!hit) continue;
            long slot = InterlockedIncrement(&s.fired_count) - 1;
            if (slot < PIX_ANIM_MAX_FIRED) { s.fired[slot].actor = id; s.fired[slot].id = ev.id; }
        }
    }

    // sample: base (with its cross-fade), overlay on top by mask, then the hierarchy
    const skeleton_file_data& sk = *a.skeleton;
    size_t n = sk.bone_count < a.bone_capacity ? sk.bone_count : a.bone_capacity;
    bool overlay = a.overlay.clip != PIX_INVALID && a.overlay.weight > 0.001f;
    mat4 global[MAX_ANIM_BONES];
    mat4* out = s.palette + a.bone_offset;
    for (size_t i = 0; i < n; i++) {
        vec3 p, sc; quat r;
        anim__layer_bone(a, a.base, i, &p, &r, &sc);
        if (overlay) {
            float w = a.overlay.weight * (a.overlay_mask ? a.overlay_mask[i] : 1.0f);
            if (w > 0.001f) {
                vec3 op, os; quat orr;
                anim__layer_bone(a, a.overlay, i, &op, &orr, &os);
                p = v3lerp(p, op, w);
                sc = v3lerp(sc, os, w);
                r = quat_slerp(r, orr, w);
            }
        }
        mat4 local = mat4_from_trs(p, r, sc);
        const bone& b = sk.bones[i];
        global[i] = (b.parent < 0) ? mat4_mul(sk.root_transform, local) : mat4_mul(global[b.parent], local);
        out[i] = mat4_mul(global[i], b.inverse_bind);
    }
}

static void anim__job(void* user, size_t begin, size_t end, int worker) {
    (void)worker;
    pix_anim_system& s = *(pix_anim_system*)user;
    for (size_t i = begin; i < end; i++) anim__update_actor(s, (idx)i);
}

void pix_anim_update(pix_anim_system& s, float dt, vec3 eye) {
    s.dt = dt;
    s.eye = eye;
    s.fired_count = 0;
    s.sampled = 0;
    if (s.jobs) pix_parallel_for(*s.jobs, s.count, anim__job, &s, 8);
    else        anim__job(&s, 0, s.count, 0);
    if (s.fired_count > PIX_ANIM_MAX_FIRED) s.fired_count = PIX_ANIM_MAX_FIRED;
}

const pix_anim_fired* pix_anim_events(const pix_anim_system& s, int* count) {
    if (count) *count = (int)s.fired_count;
    return s.fired;
}

const mat4* pix_anim_bones(const pix_anim_system& s, idx a, int* count) {
    if (a >= s.count || !s.actors[a].alive) { if (count) *count = 0; return 0; }
    const pix_anim_actor& p = s.actors[a];
    if (count) *count = (int)(p.skeleton->bone_count < p.bone_capacity ? p.skeleton->bone_count : p.bone_capacity);
    return s.palette + p.bone_offset;
}

float pix_anim_time(const pix_anim_system& s, idx a)    { return a < s.count ? s.actors[a].base.time : 0.0f; }
bool  pix_anim_finished(const pix_anim_system& s, idx a) { return a < s.count && s.actors[a].finished; }

idx pix_anim_find_clip(const pix_anim_system& s, idx a, const char* name) {
    if (a >= s.count || !name) return PIX_INVALID;
    const pix_anim_actor& p = s.actors[a];
    for (size_t i = 0; i < p.clip_count; i++) if (strcmp(p.clips[i].name, name) == 0) return (idx)i;
    return PIX_INVALID;
}

float* pix_anim_bone_mask(const skeleton_file_data& sk, const char* root_bone, unsigned mem) {
    float* mask = pix_new<float>(sk.bone_count, (mem & PIX_MEM_SCOPED) | PIX_MEM_ZERO);
    if (!mask) return 0;
    int root = -1;
    for (size_t i = 0; i < sk.bone_count; i++) if (strcmp(sk.bones[i].name, root_bone) == 0) { root = (int)i; break; }
    if (root < 0) return mask;   // unknown bone: an empty mask
    // parents always come first, so one forward pass marks the whole subtree
    for (size_t i = 0; i < sk.bone_count; i++)
        mask[i] = ((int)i == root || (sk.bones[i].parent >= 0 && mask[sk.bones[i].parent] > 0.0f)) ? 1.0f : 0.0f;
    return mask;
}
#endif // PIX_ANIMATION_IMPL
