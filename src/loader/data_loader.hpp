#pragma once
#include <stdlib.h>
#include <string.h>
#include "../core/dtype.hpp"
#include "../core/math.hpp"
#include "asset_types.hpp"
#include "png_loader.hpp"
#include "obj_loader.hpp"
#include "gltf_loader.hpp"
#include "wav_loader.hpp"

// Owns the arena every asset is parsed into, plus the tables the individual
// loaders (obj/gltf/png) fill. Everything handed out stays valid until the
// arena is reset or freed.

#define MAX_MESH_FILES      256
#define MAX_IMAGE_FILES     64
#define MAX_SKELETONS       64
#define MAX_SKINNED_MESHES  128
#define MAX_ANIMATIONS      128
#define MAX_MODEL_FILES     64
#define MAX_SOUND_FILES     64

struct pix_data_loader {
    mem_arena arena;

    mesh_file_data*  mesh_files;
    size_t mesh_file_count;
    image_file_data* image_files;
    size_t image_file_count;

    // skinned / animated
    skeleton_file_data*       skeletons;
    size_t                    skeleton_count;
    skinned_mesh_file_data*   skinned_mesh_files;
    size_t                    skinned_mesh_file_count;
    animation_clip_file_data* animation_clips;
    size_t                    animation_clip_count;
    model_file_data*          model_files;
    size_t                    model_file_count;

    sound_file_data*          sound_files;
    size_t                    sound_file_count;
};

static pix_data_loader pix_create_data_loader(size_t capacity = 128 * MB) {
    pix_data_loader loader = {};
    loader.arena.capacity = capacity;
    loader.arena.data = (char*)malloc(capacity);
    loader.mesh_files  = allocate<mesh_file_data>(loader.arena, MAX_MESH_FILES);
    loader.image_files = allocate<image_file_data>(loader.arena, MAX_IMAGE_FILES);

    loader.skeletons          = allocate<skeleton_file_data>(loader.arena, MAX_SKELETONS);
    loader.skinned_mesh_files = allocate<skinned_mesh_file_data>(loader.arena, MAX_SKINNED_MESHES);
    loader.animation_clips    = allocate<animation_clip_file_data>(loader.arena, MAX_ANIMATIONS);
    loader.model_files        = allocate<model_file_data>(loader.arena, MAX_MODEL_FILES);
    loader.sound_files        = allocate<sound_file_data>(loader.arena, MAX_SOUND_FILES);
    return loader;
}

static void pix_destroy_data_loader(pix_data_loader& loader) {
    free(loader.arena.data);
    loader.arena = mem_arena();
}

static idx load_mesh_obj_file(pix_data_loader& loader, const char* filepath);
static idx load_image_file(pix_data_loader& loader, const char* filepath, size_t channels);
// decodes a .wav into loader.sound_files; hand the result to sound.hpp's load_sound
static idx load_sound_wav_file(pix_data_loader& loader, const char* filepath);

// Loads a .glb / .gltf into the skeleton, skinned mesh and animation tables in
// one call. Returns an index into loader.model_files describing what came out
// of the file, or (idx)-1 if it could not be read.
static idx load_model_gltf_file(pix_data_loader& loader, const char* filepath);

// convenience lookups over the tables a model call filled in
static skinned_mesh_file_data* get_model_mesh(pix_data_loader& loader, idx model);
static skeleton_file_data*     get_model_skeleton(pix_data_loader& loader, idx model);
static animation_clip_file_data* find_animation(pix_data_loader& loader, idx model, const char* name);

// ----------- implementation --------------------

static idx load_mesh_obj_file(pix_data_loader& loader, const char* filepath) {
    if (loader.mesh_file_count >= MAX_MESH_FILES) return (idx)-1;
    mesh_file_data m = {};
    if (!obj_load_file(loader.arena, filepath, &m)) return (idx)-1;
    idx id = (idx)loader.mesh_file_count++;
    loader.mesh_files[id] = m;
    return id;
}

static idx load_sound_wav_file(pix_data_loader& loader, const char* filepath) {
    if (loader.sound_file_count >= MAX_SOUND_FILES) return (idx)-1;
    sound_file_data s = {};
    if (!wav_load_file(loader.arena, filepath, &s)) return (idx)-1;
    idx id = (idx)loader.sound_file_count++;
    loader.sound_files[id] = s;
    return id;
}

static idx load_image_file(pix_data_loader& loader, const char* filepath, size_t channels) {
    (void)channels; // png_loader always returns RGBA8
    if (loader.image_file_count >= MAX_IMAGE_FILES) return (idx)-1;
    idx id = (idx)loader.image_file_count++;
    image_file_data* img = &loader.image_files[id];

    png_result png = {};
    if (png_load_file(loader.arena, filepath, &png)) {
        img->width   = png.width;
        img->height  = png.height;
        img->channel = png.channels;
        img->data    = (char*)png.pixels;
        return id;
    }

    // fallback: 2x2 white so the caller still gets a usable texture
    img->width = 2;
    img->height = 2;
    img->channel = 4;
    img->data = allocate<char>(loader.arena, 2 * 2 * 4);
    memset(img->data, 0xFF, 2 * 2 * 4);
    return id;
}

static idx load_model_gltf_file(pix_data_loader& loader, const char* filepath) {
    if (loader.model_file_count >= MAX_MODEL_FILES) return (idx)-1;

    gltf_result res = {};
    if (!gltf_load_file(loader.arena, filepath, &res)) return (idx)-1;

    idx id = (idx)loader.model_file_count++;
    model_file_data* mf = &loader.model_files[id];
    *mf = model_file_data();
    mf->skinned_mesh = mf->skeleton = mf->first_animation = mf->image = -1;

    if (res.skeleton.bone_count && loader.skeleton_count < MAX_SKELETONS) {
        mf->skeleton = (int32_t)loader.skeleton_count;
        loader.skeletons[loader.skeleton_count++] = res.skeleton;
    }

    if (res.mesh.vertex_count && loader.skinned_mesh_file_count < MAX_SKINNED_MESHES) {
        res.mesh.skeleton = mf->skeleton;
        mf->skinned_mesh = (int32_t)loader.skinned_mesh_file_count;
        loader.skinned_mesh_files[loader.skinned_mesh_file_count++] = res.mesh;
    }

    for (size_t i = 0; i < res.clip_count; i++) {
        if (loader.animation_clip_count >= MAX_ANIMATIONS) break;
        if (mf->first_animation < 0) mf->first_animation = (int32_t)loader.animation_clip_count;
        res.clips[i].skeleton = mf->skeleton;
        loader.animation_clips[loader.animation_clip_count++] = res.clips[i];
        mf->animation_count++;
    }

    if (res.image.data && loader.image_file_count < MAX_IMAGE_FILES) {
        mf->image = (int32_t)loader.image_file_count;
        loader.image_files[loader.image_file_count++] = res.image;
    }
    return id;
}

static skinned_mesh_file_data* get_model_mesh(pix_data_loader& loader, idx model) {
    if (model >= loader.model_file_count) return 0;
    int32_t i = loader.model_files[model].skinned_mesh;
    return (i >= 0) ? &loader.skinned_mesh_files[i] : 0;
}

static skeleton_file_data* get_model_skeleton(pix_data_loader& loader, idx model) {
    if (model >= loader.model_file_count) return 0;
    int32_t i = loader.model_files[model].skeleton;
    return (i >= 0) ? &loader.skeletons[i] : 0;
}

static animation_clip_file_data* find_animation(pix_data_loader& loader, idx model, const char* name) {
    if (model >= loader.model_file_count) return 0;
    const model_file_data& mf = loader.model_files[model];
    for (size_t i = 0; i < mf.animation_count; i++) {
        animation_clip_file_data* c = &loader.animation_clips[mf.first_animation + i];
        if (!name || strcmp(c->name, name) == 0) return c;
    }
    return 0;
}
