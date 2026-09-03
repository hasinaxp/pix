#pragma once
#include <stdlib.h>
#include <stddef.h>
#include "dtype.hpp"
#include "math.hpp"
#include "../loader/data_loader.hpp"
#include "../loader/png_loader.hpp"
#include "opengl_utils.hpp"

#define MAX_RENDERABLE 1024
#define MAX_MESHES     128
#define MAX_MATERIALS  128
#define MAX_SHADERS    32
#define MESH_POOL_VERTICES (1 << 18)   // shared vertex buffer capacity
#define MESH_POOL_INDICES  (1 << 18)   // shared index buffer capacity

struct mesh {
    idx vbo;
    idx vertex_offset;   // first vertex in the shared buffer
    idx vertex_size;     // stride, bytes
    idx vertex_count;
    idx index_offset;    // first index in the shared buffer
    idx index_count;
};

struct material {
    idx shader;
    idx texture;
    vec3 color;
    float metallic;
    float roughness;
};

struct pix_render_instance {
    idx mesh;
    idx material;
    mat4 trainsform;
};

struct camera {
    vec3 position;
    vec3 direction;
    vec3 up;
};

struct pix_renderer {
    int width;
    int height;
    vec3 clear_color;

    pix_render_instance instances[MAX_RENDERABLE];
    pix_render_instance animated_instances[MAX_RENDERABLE];
    mesh     meshes[MAX_MESHES];
    mesh     skinned_meshes[MAX_MESHES];
    material materials[MAX_MATERIALS];
    idx      shaders[MAX_SHADERS];
    idx      shader;

    size_t mesh_count;
    size_t material_count;
    size_t shader_count;
    size_t renderable_count;

    idx vertex_cursor;  // bump allocator into the shared buffers
    idx index_cursor;

    mat4 view_matrix;
    mat4 projection_matrix;

    idx vao;
    idx models_vbo;      // every mesh's vertices
    idx instances_vbo;   // per-frame transforms, streamed with glBufferSubData
    idx ebo;             // every mesh's indices

    idx vao_animated;
    idx models_vbo_animated;
    idx instances_vbo_animated;
    idx ebo_animated;

    idx white_texture;   // stand-in for untextured materials
    mem_arena tex_arena; // scratch for image decoding
};


static pix_renderer pix_create_renderer(
    int width, int height, idx shader, vec3 clear_color = { 0.0f, 0.5f, 0.8f });
static void pix_destory_renderer(pix_renderer& renderer);

static idx load_mesh(pix_renderer& renderer, mesh_file_data& mesh_data);
staitc idx load_skinned_mesh(pix_render& renderer, skinned_mesh_file_data& mesh_file_data);
static idx load_material(pix_renderer& renderer,
    const char* texture_path = nullptr, vec3 color = { 1.0f, 1.0f, 1.0f },
    float metalic = 0.1f, float roughness = 0.5f);
static idx get_default_material(pix_renderer& renderer);
static idx load_shader(pix_renderer& renderer, const char* vsrc, const char* fsrc);

static void begin_frame(pix_renderer& renderer, camera& cam);
static void push_instance(pix_renderer& renderer, pix_render_instance& instance);
static void push_animated_instance(pix_renderer& renderer, pix_render_instance& instance, animation& animation);
static void end_frame(pix_renderer& renderer, bool clear_instances = true);


