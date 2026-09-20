#pragma once
#include <stdint.h>
#include "../core/dtype.hpp"

// Data model shared by the individual file loaders (obj/gltf/png) and by
// data_loader.hpp, which owns the tables these get parsed into.

struct vertex {
    vec3 position;
    vec3 normal;
    vec2 uv;
};

struct vertex_rigged {
    vec3 position;
    vec3 normal;
    vec2 uv;
    vec4 bone_ids;      // indices into skeleton_file_data::bones (whole numbers held as floats)
    vec4 bone_weights;  // normalized, sums to 1
};

// A named `g`/`o` run inside one OBJ file: a contiguous slice of the emitted
// vertices, plus the centre of its own bounds so the part can be pulled out
// and rotated about itself. Anything with a moving part authored in place -
// a wheel, a turret, a door - is shaped like this.
struct mesh_group {
    char   name[48];
    size_t vertex_offset;
    size_t vertex_count;
    vec3   pivot;
};

struct mesh_file_data {
    size_t   vertex_count;
    vertex*  vertex_data;
    size_t   index_count;
    uint16_t* index_data;

    size_t      group_count;
    mesh_group* groups;

    vec3 bounds_min;   // model space, before any placement scale
    vec3 bounds_max;
};

struct image_file_data {
    int   width;
    int   height;
    int   channel;
    char* data;
};

// decoded pcm, always 16-bit signed and interleaved, at the file's own rate
struct sound_file_data {
    int16_t* samples;
    size_t   frame_count;
    int      channels;      // 1 or 2
    int      rate;
};

// ---------------- skinning ----------------

// One joint of a skeleton. Bones are stored topologically sorted: `parent` is
// always < the bone's own index, so a pose can be resolved in one linear pass
//   global[i] = (parent < 0 ? root_transform : global[parent]) * local[i]
struct bone {
    int32_t parent;           // -1 for a root bone
    mat4    inverse_bind;     // mesh space -> bone space
    vec3    local_position;   // rest pose, used for bones a clip does not animate
    quat    local_rotation;
    vec3    local_scale;
    char    name[32];
};

struct skeleton_file_data {
    size_t bone_count;
    bone*  bones;
    mat4   root_transform;    // scene transform sitting above the joints (unit/axis fixups)
};

// ---------------- animation ----------------

// T/R/S sampled onto one shared timeline, so posing a bone is a single search
// plus one lerp/slerp rather than three independent channel lookups.
struct bone_keyframe {
    float time;               // seconds from clip start
    vec3  position;
    quat  rotation;
    vec3  scale;
};

struct bone_animation_track {
    int32_t        bone;            // index into skeleton_file_data::bones (== index in clip.tracks)
    size_t         keyframe_count;  // 0 = not animated by this clip, hold the bone's rest pose
    bone_keyframe* keyframes;       // ascending by time
};

struct animation_clip_file_data {
    char    name[32];
    float   duration;               // seconds
    int32_t skeleton;               // index into pix_data_loader::skeletons
    size_t  track_count;            // == skeleton bone_count; dense, index it by bone
    bone_animation_track* tracks;
};

struct skinned_mesh_file_data {
    size_t         vertex_count;
    vertex_rigged* vertex_data;
    size_t         index_count;
    uint16_t*      index_data;
    int32_t        skeleton;        // index into pix_data_loader::skeletons, -1 if unskinned
};

// What one model file yielded. Clips are contiguous:
// [first_animation, first_animation + animation_count)
struct model_file_data {
    int32_t skinned_mesh;
    int32_t skeleton;
    int32_t first_animation;
    size_t  animation_count;
    int32_t image;                  // embedded base colour texture, -1 if absent/undecodable
    // `image` is a generated colour palette (one cell per material) rather than
    // a uv-mapped texture, so it must be sampled with NEAREST or neighbouring
    // cells bleed into each other
    bool    image_is_palette;
    // Where a single-node load's geometry stood in the file it came out of.
    //
    // A one-node load is re-based on that node's own origin (see
    // gltf_load_file) so a prop pulled out of a gallery row does not arrive
    // still offset by where it was parked next to its siblings. That is the
    // right default and it throws away the one thing a caller assembling a
    // model out of several of its own nodes needs: a part has to go back on
    // the spot it came off. This is that translation, kept rather than
    // discarded. Zero for a whole-file load, where it is baked into the
    // vertices instead.
    vec3    node_offset;
};
