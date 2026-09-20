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

// Table sizes, not memory: each entry is a handful of pointers into the arena,
// so these are cheap and running out of one is silent from the outside - the
// asset loaded after a table fills simply does not exist.
//
// They are sized for the modern-asset-pack case rather than the shared-atlas
// one. A pack that ships one glTF per prop, each with its own embedded
// texture, burns an image slot per model where a kit of OBJ pieces over one
// atlas burns a single slot for hundreds; and a character rig carrying
// twenty-odd clips fills MAX_ANIMATIONS far faster than the mesh tables fill.
#define MAX_MESH_FILES      512
#define MAX_IMAGE_FILES     256
#define MAX_SKELETONS       96
#define MAX_SKINNED_MESHES  320
#define MAX_ANIMATIONS      512
#define MAX_MODEL_FILES     256
#define MAX_SOUND_FILES     64

struct pix_data_loader {
    mem_arena arena;

    // colours pulled out of .mtl files that carry no texture; one generated
    // image built from this lets every such model share a single material
    obj_palette palette;

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

// ---- transient loads ----
//
// A prop's glTF is parsed into the arena, uploaded to the GPU, and then never
// looked at again: the catalogue keeps a mesh handle, a material handle and a
// bounding box, and not one byte of the file it came from. Keeping all of it
// alive anyway is what fills a large arena with a couple of hundred props -
// each load copies the whole .glb binary chunk plus a JSON token table, and a
// library file holding five props is parsed five times over, once per prop.
//
// So a caller that only wants the GPU side of a file marks the loader before
// the load and rewinds it after: the arena's bump pointer goes back, and so
// do the table counts, because a model_file entry left behind would point at
// vertex data that is about to be overwritten by the next load.
//
// Anything whose CPU-side data is read again later - a character rig's
// skeleton, its clips, the mesh a pose is measured against - must NOT be
// loaded this way: rewinding the arena leaves the animator pointing at bytes
// the next load is about to overwrite.
struct pix_loader_mark {
    size_t arena, mesh_files, image_files, skeletons, skinned_meshes, clips, model_files;
};

static pix_loader_mark pix_loader_mark_now(const pix_data_loader& l) {
    pix_loader_mark m;
    m.arena          = l.arena.size;
    m.mesh_files     = l.mesh_file_count;
    m.image_files    = l.image_file_count;
    m.skeletons      = l.skeleton_count;
    m.skinned_meshes = l.skinned_mesh_file_count;
    m.clips          = l.animation_clip_count;
    m.model_files    = l.model_file_count;
    return m;
}

static void pix_loader_rewind(pix_data_loader& l, const pix_loader_mark& m) {
    l.arena.size              = m.arena;
    l.mesh_file_count         = m.mesh_files;
    l.image_file_count        = m.image_files;
    l.skeleton_count          = m.skeletons;
    l.skinned_mesh_file_count = m.skinned_meshes;
    l.animation_clip_count    = m.clips;
    l.model_file_count        = m.model_files;
}

static idx load_mesh_obj_file(pix_data_loader& loader, const char* filepath);
// splits a named `g`/`o` part of an already loaded model into a mesh of its
// own, centred on its own pivot, so the part can be animated independently of
// the body it came off; returns an index into loader.mesh_files
static idx load_mesh_obj_group(pix_data_loader& loader, idx mesh_file, const char* group,
                               vec3* out_pivot);
static idx load_image_file(pix_data_loader& loader, const char* filepath, size_t channels);
// decodes a .wav into loader.sound_files; hand the result to sound.hpp's load_sound
static idx load_sound_wav_file(pix_data_loader& loader, const char* filepath);

// Loads a .glb / .gltf into the skeleton, skinned mesh and animation tables in
// one call. Returns an index into loader.model_files describing what came out
// of the file, or (idx)-1 if it could not be read.
//
// `only_node` loads a single mesh node instead of merging every one the file
// has - see gltf_load_file for why a library file holding several unrelated
// props needs that instead of the usual "one character split across several
// meshes" merge.
//
// `skip_nodes` excludes mesh nodes instead of selecting one, which is what
// taking a vehicle's shell without its wheels needs - see gltf_load_file.
static idx load_model_gltf_file(pix_data_loader& loader, const char* filepath, int only_node = -1,
                                int only_material = -1,
                                const int* skip_nodes = 0, int skip_count = 0);

// convenience lookups over the tables a model call filled in
static skinned_mesh_file_data* get_model_mesh(pix_data_loader& loader, idx model);
static skeleton_file_data*     get_model_skeleton(pix_data_loader& loader, idx model);
static animation_clip_file_data* find_animation(pix_data_loader& loader, idx model, const char* name);

// ----------- implementation --------------------

static idx load_mesh_obj_file(pix_data_loader& loader, const char* filepath) {
    if (loader.mesh_file_count >= MAX_MESH_FILES) return (idx)-1;
    mesh_file_data m = {};
    if (!obj_load_file(loader.arena, filepath, &m, &loader.palette)) return (idx)-1;
    idx id = (idx)loader.mesh_file_count++;
    loader.mesh_files[id] = m;
    return id;
}

static idx load_mesh_obj_group(pix_data_loader& loader, idx mesh_file, const char* group,
                               vec3* out_pivot) {
    if (mesh_file >= loader.mesh_file_count) return (idx)-1;
    if (loader.mesh_file_count >= MAX_MESH_FILES) return (idx)-1;
    int g = obj_find_group(loader.mesh_files[mesh_file], group);
    if (g < 0) return (idx)-1;
    mesh_file_data m = {};
    if (!obj_extract_group(loader.arena, loader.mesh_files[mesh_file], (size_t)g, &m, out_pivot))
        return (idx)-1;
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

static idx load_model_gltf_file(pix_data_loader& loader, const char* filepath, int only_node,
                                int only_material, const int* skip_nodes, int skip_count) {
    if (loader.model_file_count >= MAX_MODEL_FILES) return (idx)-1;

    gltf_result res = {};
    if (!gltf_load_file(loader.arena, filepath, &res, only_node, only_material,
                        skip_nodes, skip_count))
        return (idx)-1;

    idx id = (idx)loader.model_file_count++;
    model_file_data* mf = &loader.model_files[id];
    *mf = model_file_data();
    mf->skinned_mesh = mf->skeleton = mf->first_animation = mf->image = -1;
    mf->node_offset = res.node_offset;

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
        mf->image_is_palette = res.image_is_palette;
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
