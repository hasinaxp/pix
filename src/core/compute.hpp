// pix - compute shaders and GPU buffers.
//
//   pix_gpu_buffer particles = pix_create_gpu_buffer(sizeof(particle) * N, initial_data);
//   idx sim = pix_create_compute_shader(R"(
//       layout(local_size_x = 256) in;
//       struct particle { vec4 pos; vec4 vel; };
//       layout(std430, binding = 0) buffer Particles { particle p[]; };
//       uniform float uDt;
//       void main() {
//           uint i = gl_GlobalInvocationID.x;
//           if (i >= p.length()) return;
//           p[i].vel.y -= 9.8 * uDt;
//           p[i].pos += p[i].vel * uDt;
//       }
//   )");
//   pix_bind_storage(particles, 0);
//   pix_set_float(sim, "uDt", dt);
//   pix_dispatch_for(sim, N, 256);                     // enough groups to cover N
//   pix_compute_barrier(PIX_BARRIER_VERTEX | PIX_BARRIER_STORAGE);
//
// Sources are written without a #version line: pix adds `#version 440 core`
// and, by default, the scene block (uTime, uCameraPos, uViewProj, the sun...),
// the colour helpers and the noise library (value_noise, fbm), so a compute
// shader sees the same world a surface shader does.
#ifndef PIX_COMPUTE_HPP
#define PIX_COMPUTE_HPP
#include "dtype.hpp"
#include "math.hpp"
#include "opengl_api.hpp"
#include "opengl_utils.hpp"
#include "shader_sources.hpp"

// A GPU buffer usable as a shader storage buffer (std430 `buffer` blocks), a
// vertex buffer, or a source of indirect draw / dispatch arguments.
struct pix_gpu_buffer {
    GLuint id;
    size_t size;
};

PIXDEF pix_gpu_buffer pix_create_gpu_buffer(size_t size, const void* data = nullptr);
PIXDEF void pix_update_gpu_buffer(pix_gpu_buffer& buffer, const void* data, size_t size = 0, size_t offset = 0);
// Reads back into CPU memory. This waits for the GPU to finish writing, so
// keep it off the hot path (debugging, occasional queries).
PIXDEF void pix_read_gpu_buffer(const pix_gpu_buffer& buffer, void* out, size_t size = 0, size_t offset = 0);
PIXDEF void pix_clear_gpu_buffer(pix_gpu_buffer& buffer);   // to zeros
PIXDEF void pix_destroy_gpu_buffer(pix_gpu_buffer& buffer);
// Binds to `layout(std430, binding = N) buffer ...`. Binding points are
// shared by every shader; pix itself uses none of the storage bindings except
// 8..9 during its own skinned draws (see PIX_SKIN_*_BINDING).
PIXDEF void pix_bind_storage(const pix_gpu_buffer& buffer, GLuint binding);

// `with_library` adds the scene block, colour helpers and noise after #version.
PIXDEF idx  pix_create_compute_shader(const char* source, bool with_library = true);
PIXDEF void pix_dispatch(idx program, int groups_x, int groups_y = 1, int groups_z = 1);
// ceil(items / group_size) groups along x - the usual 1D case
PIXDEF void pix_dispatch_for(idx program, size_t items, int group_size);
// arguments (x, y, z group counts, three uints) read from `args` at `offset`
PIXDEF void pix_dispatch_indirect(idx program, const pix_gpu_buffer& args, size_t offset = 0);

// What a later stage is about to read that a compute shader wrote. Or them.
#define PIX_BARRIER_STORAGE  GL_SHADER_STORAGE_BARRIER_BIT      // another shader reads the buffer
#define PIX_BARRIER_VERTEX   GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT // used as vertex / instance data
#define PIX_BARRIER_INDIRECT GL_COMMAND_BARRIER_BIT             // used as indirect draw / dispatch args
#define PIX_BARRIER_TEXTURE  (GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT)
#define PIX_BARRIER_ALL      GL_ALL_BARRIER_BITS
PIXDEF void pix_compute_barrier(GLbitfield what);

// A texture a compute shader writes with imageStore / reads with imageLoad.
// `format` is a sized GL format such as GL_RGBA16F, GL_RGBA32F, GL_R32F.
PIXDEF idx  pix_create_storage_texture(int width, int height, GLenum format);
PIXDEF void pix_bind_image(GLuint unit, idx texture, GLenum access, GLenum format);

// Uniform setters for any program (compute or not). Each looks the name up,
// which is cheap but not free: fine for a handful per frame.
PIXDEF void pix_set_float(idx program, const char* name, float v);
PIXDEF void pix_set_int(idx program, const char* name, int v);
PIXDEF void pix_set_vec2(idx program, const char* name, vec2 v);
PIXDEF void pix_set_vec3(idx program, const char* name, vec3 v);
PIXDEF void pix_set_vec4(idx program, const char* name, vec4 v);
PIXDEF void pix_set_mat4(idx program, const char* name, const mat4& m);

// Indirect draw argument layouts, as GL reads them from a buffer.
struct pix_draw_arrays_indirect   { uint32_t count, instance_count, first, base_instance; };
struct pix_draw_elements_indirect { uint32_t count, instance_count, first_index; int32_t base_vertex; uint32_t base_instance; };

#endif // PIX_COMPUTE_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_COMPUTE_IMPL)
#define PIX_COMPUTE_IMPL

pix_gpu_buffer pix_create_gpu_buffer(size_t size, const void* data) {
    pix_gpu_buffer b = {};
    b.size = size ? size : 16;
    glGenBuffers(1, &b.id);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, b.id);
    glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)b.size, data, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    return b;
}

void pix_update_gpu_buffer(pix_gpu_buffer& b, const void* data, size_t size, size_t offset) {
    if (!b.id || !data || offset >= b.size) return;
    if (!size || offset + size > b.size) size = b.size - offset;
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, b.id);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, (GLintptr)offset, (GLsizeiptr)size, data);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void pix_read_gpu_buffer(const pix_gpu_buffer& b, void* out, size_t size, size_t offset) {
    if (!b.id || !out || offset >= b.size) return;
    if (!size || offset + size > b.size) size = b.size - offset;
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, b.id);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, (GLintptr)offset, (GLsizeiptr)size, out);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void pix_clear_gpu_buffer(pix_gpu_buffer& b) {
    void* zeros = calloc(1, b.size);
    if (!zeros) return;
    pix_update_gpu_buffer(b, zeros);
    free(zeros);
}

void pix_destroy_gpu_buffer(pix_gpu_buffer& b) {
    if (b.id) glDeleteBuffers(1, &b.id);
    b = pix_gpu_buffer();
}

void pix_bind_storage(const pix_gpu_buffer& b, GLuint binding) {
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, b.id);
}

idx pix_create_compute_shader(const char* source, bool with_library) {
    static const char* const version = "#version 440 core\n";
    size_t n = strlen(version) + strlen(source) + 1;
    char* src = pix_new<char>(n);
    if (!src) return 0;
    strcpy(src, version);
    strcat(src, source);
    char* full = src;
    if (with_library) {
        const char* chunks[] = { GLSL_SCENE, GLSL_COMMON, GLSL_NOISE };
        full = pix_shader_splice(src, chunks, 3);
    }
    GLuint s = glCreateShader(GL_COMPUTE_SHADER);
    const char* text = full;
    glShaderSource(s, 1, &text, 0);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    GLuint p = 0;
    if (!ok) {
        char log[2048] = {};
        glGetShaderInfoLog(s, sizeof(log), 0, log);
        pix_log("compute shader compile error\n%s", log);
        FILE* f = fopen("shader_error.txt", "a");
        if (f) { fprintf(f, "compute shader compile error\n%s\n\n", log); fclose(f); }
    } else {
        p = glCreateProgram();
        glAttachShader(p, s);
        glLinkProgram(p);
        glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[2048] = {};
            glGetProgramInfoLog(p, sizeof(log), 0, log);
            pix_log("compute shader link error\n%s", log);
            glDeleteProgram(p);
            p = 0;
        }
    }
    glDeleteShader(s);
    if (full != src) pix_free(full);
    pix_free(src);
    return p;
}

void pix_dispatch(idx program, int gx, int gy, int gz) {
    if (!program) return;
    glUseProgram(program);
    glDispatchCompute((GLuint)(gx > 0 ? gx : 1), (GLuint)(gy > 0 ? gy : 1), (GLuint)(gz > 0 ? gz : 1));
}

void pix_dispatch_for(idx program, size_t items, int group_size) {
    if (group_size < 1) group_size = 64;
    size_t groups = (items + (size_t)group_size - 1) / (size_t)group_size;
    pix_dispatch(program, (int)(groups ? groups : 1));
}

void pix_dispatch_indirect(idx program, const pix_gpu_buffer& args, size_t offset) {
    if (!program) return;
    glUseProgram(program);
    glBindBuffer(GL_DISPATCH_INDIRECT_BUFFER, args.id);
    glDispatchComputeIndirect((GLintptr)offset);
    glBindBuffer(GL_DISPATCH_INDIRECT_BUFFER, 0);
}

void pix_compute_barrier(GLbitfield what) { glMemoryBarrier(what); }

idx pix_create_storage_texture(int width, int height, GLenum format) {
    GLuint t = 0;
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_2D, t);
    glTexStorage2D(GL_TEXTURE_2D, 1, format, width, height);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    return t;
}

void pix_bind_image(GLuint unit, idx texture, GLenum access, GLenum format) {
    glBindImageTexture(unit, texture, 0, GL_FALSE, 0, access, format);
}

void pix_set_float(idx p, const char* n, float v) { glUseProgram(p); glUniform1f(glGetUniformLocation(p, n), v); }
void pix_set_int(idx p, const char* n, int v)     { glUseProgram(p); glUniform1i(glGetUniformLocation(p, n), v); }
void pix_set_vec2(idx p, const char* n, vec2 v)   { glUseProgram(p); glUniform2f(glGetUniformLocation(p, n), v.x, v.y); }
void pix_set_vec3(idx p, const char* n, vec3 v)   { glUseProgram(p); glUniform3f(glGetUniformLocation(p, n), v.x, v.y, v.z); }
void pix_set_vec4(idx p, const char* n, vec4 v)   { glUseProgram(p); glUniform4f(glGetUniformLocation(p, n), v.x, v.y, v.z, v.w); }
void pix_set_mat4(idx p, const char* n, const mat4& m) {
    glUseProgram(p);
    glUniformMatrix4fv(glGetUniformLocation(p, n), 1, GL_FALSE, m.data);
}

#endif // PIX_COMPUTE_IMPL
