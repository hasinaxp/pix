#pragma once
#include <stdlib.h>
#include <stddef.h>
#include "dtype.hpp"
#include "math.hpp"
#include "data_loader.hpp"
#include "png_loader.hpp"
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
    mesh     meshes[MAX_MESHES];
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

    idx white_texture;   // stand-in for untextured materials
    mem_arena tex_arena; // scratch for image decoding
};


static pix_renderer pix_create_renderer(
    int width, int height, idx shader, vec3 clear_color = { 0.0f, 0.5f, 0.8f });
static void pix_destory_renderer(pix_renderer& renderer);

static idx load_mesh(pix_renderer& renderer, mesh_file_data& mesh_data);
static idx load_material(pix_renderer& renderer,
    const char* texture_path = nullptr, vec3 color = { 1.0f, 1.0f, 1.0f },
    float metalic = 0.1f, float roughness = 0.5f);
static idx get_default_material(pix_renderer& renderer);
static idx load_shader(pix_renderer& renderer, const char* vsrc, const char* fsrc);

static void begin_frame(pix_renderer& renderer, camera& cam);
static void push_instance(pix_renderer& renderer, pix_render_instance& instance);
static void end_frame(pix_renderer& renderer, bool clear_instances = true);


// ------------------- implementation ---------------------

static pix_renderer pix_create_renderer(int width, int height, idx shader, vec3 clear_color) {
    pix_renderer r = {};
    r.width = width;
    r.height = height;
    r.shader = shader;
    r.clear_color = clear_color;
    r.tex_arena.capacity = 32 * MB;
    r.tex_arena.data = (char*)malloc(r.tex_arena.capacity);

    glGenVertexArrays(1, &r.vao);
    glBindVertexArray(r.vao);

    glGenBuffers(1, &r.models_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, r.models_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(MESH_POOL_VERTICES * sizeof(vertex)), 0, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, position));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, normal));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, uv));
    glEnableVertexAttribArray(2);

    glGenBuffers(1, &r.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(MESH_POOL_INDICES * sizeof(uint16_t)), 0, GL_STATIC_DRAW);

    glGenBuffers(1, &r.instances_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, r.instances_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(MAX_RENDERABLE * sizeof(mat4)), 0, GL_DYNAMIC_DRAW);
    for (int c = 0; c < 4; c++) {
        glVertexAttribPointer(3 + c, 4, GL_FLOAT, GL_FALSE, sizeof(mat4), (void*)(size_t)(c * 4 * sizeof(float)));
        glEnableVertexAttribArray(3 + c);
        glVertexAttribDivisor(3 + c, 1);
    }

    glBindVertexArray(0);

    unsigned char white[4] = { 255, 255, 255, 255 };
    r.white_texture = opengl_create_texture2d(1, 1, 4, white, TEXTURE_PIXELATED);

    r.projection_matrix = mat4_perspective(1.05f, (float)width / (float)height, 0.1f, 800.0f);
    r.view_matrix = mat4_identity();
    return r;
}

static void pix_destory_renderer(pix_renderer& renderer) {
    free(renderer.tex_arena.data);
    renderer.tex_arena.data = 0;
    renderer.tex_arena.capacity = 0;
    renderer.tex_arena.size = 0;
}

static idx load_mesh(pix_renderer& r, mesh_file_data& md) {
    idx id = (idx)r.mesh_count++;
    mesh& m = r.meshes[id];
    m.vbo = r.models_vbo;
    m.vertex_offset = r.vertex_cursor;
    m.vertex_size = sizeof(vertex);
    m.vertex_count = (idx)md.vertex_count;
    m.index_offset = r.index_cursor;
    m.index_count = (idx)md.index_count;

    glBindVertexArray(r.vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.models_vbo);
    glBufferSubData(GL_ARRAY_BUFFER,
        (GLintptr)(r.vertex_cursor * sizeof(vertex)),
        (GLsizeiptr)(md.vertex_count * sizeof(vertex)), md.vertex_data);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r.ebo);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,
        (GLintptr)(r.index_cursor * sizeof(uint16_t)),
        (GLsizeiptr)(md.index_count * sizeof(uint16_t)), md.index_data);
    glBindVertexArray(0);

    r.vertex_cursor += (idx)md.vertex_count;
    r.index_cursor += (idx)md.index_count;
    return id;
}

static idx load_material(pix_renderer& r, const char* texture_path, vec3 color, float metalic, float roughness) {
    idx id = (idx)r.material_count++;
    material& mat = r.materials[id];
    mat.shader = r.shader;
    mat.texture = 0;
    mat.color = color;
    mat.metallic = metalic;
    mat.roughness = roughness;

    if (texture_path) {
        png_result png = {};
        if (png_load_file(r.tex_arena, texture_path, &png)) {
            // the KayKit atlas is a grid of flat colour swatches - mipmaps would
            // bleed neighbouring swatches together and wash everything brown.
            mat.texture = opengl_create_texture2d(png.width, png.height, 4, png.pixels, TEXTURE_LINEAR);
        }
        arena_reset(r.tex_arena); // decoded pixels now live on the GPU
    }
    return id;
}

static idx get_default_material(pix_renderer& r) {
    return load_material(r, nullptr, v3(1.0f, 1.0f, 1.0f), 0.1f, 0.5f);
}

static idx load_shader(pix_renderer& r, const char* vsrc, const char* fsrc) {
    idx prog = opengl_create_shader(vsrc, fsrc);
    r.shaders[r.shader_count++] = prog;
    return prog;
}

static void begin_frame(pix_renderer& r, camera& cam) {
    r.view_matrix = mat4_lookat(cam.position, v3add(cam.position, cam.direction), cam.up);
    r.renderable_count = 0;
}

static void push_instance(pix_renderer& r, pix_render_instance& instance) {
    if (r.renderable_count < MAX_RENDERABLE)
        r.instances[r.renderable_count++] = instance;
}

static int pix__instance_cmp(const void* a, const void* b) {
    const pix_render_instance* x = (const pix_render_instance*)a;
    const pix_render_instance* y = (const pix_render_instance*)b;
    if (x->mesh != y->mesh) return (x->mesh < y->mesh) ? -1 : 1;
    if (x->material != y->material) return (x->material < y->material) ? -1 : 1;
    return 0;
}

static void end_frame(pix_renderer& r, bool clear_instances) {
    static mat4 batch[MAX_RENDERABLE];

    mat4 vp = mat4_mul(r.projection_matrix, r.view_matrix);

    // group instances so each (mesh, material) run is one instanced draw
    qsort(r.instances, r.renderable_count, sizeof(pix_render_instance), pix__instance_cmp);

    glViewport(0, 0, r.width, r.height);
    glEnable(GL_DEPTH_TEST);
    glClearColor(r.clear_color.x, r.clear_color.y, r.clear_color.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(r.shader);
    glUniformMatrix4fv(glGetUniformLocation(r.shader, "uViewProj"), 1, GL_FALSE, vp.data);
    glUniform1i(glGetUniformLocation(r.shader, "uTex"), 0);
    GLint u_color = glGetUniformLocation(r.shader, "uColor");

    glBindVertexArray(r.vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.instances_vbo);

    size_t i = 0;
    while (i < r.renderable_count) {
        idx mi = r.instances[i].mesh;
        idx ma = r.instances[i].material;
        size_t j = i;
        while (j < r.renderable_count && r.instances[j].mesh == mi && r.instances[j].material == ma) {
            batch[j - i] = r.instances[j].trainsform;
            j++;
        }
        int n = (int)(j - i);
        glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(n * sizeof(mat4)), batch);

        material& mat = r.materials[ma];
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, mat.texture ? mat.texture : r.white_texture);
        glUniform3f(u_color, mat.color.x, mat.color.y, mat.color.z);

        mesh& me = r.meshes[mi];
        glDrawElementsInstancedBaseVertex(
            GL_TRIANGLES, (GLsizei)me.index_count, GL_UNSIGNED_SHORT,
            (void*)(size_t)(me.index_offset * sizeof(uint16_t)), n, (GLint)me.vertex_offset);

        i = j;
    }

    glBindVertexArray(0);
    if (clear_instances) r.renderable_count = 0;
}
