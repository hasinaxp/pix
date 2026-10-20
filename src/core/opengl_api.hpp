// pix - OpenGL 4.4 core function loader. No GLEW/GLAD: every entry point the
// library uses is typedef'd here and resolved by opengl_load_functions().
//
// The pointers are declared extern and defined once in the implementation, so
// game code in any .cpp can call GL directly (custom shaders, custom passes).
#ifndef PIX_OPENGL_API_HPP
#define PIX_OPENGL_API_HPP
#include "dtype.hpp"
#include <windows.h>
#include <GL/gl.h>

// ---- types missing from the OS <GL/gl.h> (OpenGL 1.1) ----
typedef char      GLchar;
typedef unsigned long long GLuint64;
typedef ptrdiff_t GLsizeiptr;
typedef ptrdiff_t GLintptr;

// ---- enums used by the 4.4 core entry points below ----
#define GL_VERTEX_SHADER        0x8B31
#ifndef GL_SAMPLES_PASSED
#define GL_SAMPLES_PASSED       0x8914
#endif
#ifndef GL_CLIP_DISTANCE0
#define GL_CLIP_DISTANCE0       0x3000
#endif
#define GL_FRAGMENT_SHADER      0x8B30
#define GL_COMPILE_STATUS       0x8B81
#define GL_LINK_STATUS          0x8B82
#define GL_ARRAY_BUFFER         0x8892
#define GL_ELEMENT_ARRAY_BUFFER 0x8893
#define GL_STATIC_DRAW          0x88E4
#define GL_DYNAMIC_DRAW         0x88E8
#define GL_STREAM_DRAW          0x88E0
#define GL_UNIFORM_BUFFER       0x8A11
#define GL_COPY_READ_BUFFER     0x8F36
#define GL_COPY_WRITE_BUFFER    0x8F37
#define GL_SHADER_STORAGE_BUFFER 0x90D2
#define GL_DRAW_INDIRECT_BUFFER  0x8F3F
#define GL_DISPATCH_INDIRECT_BUFFER 0x90EE
#define GL_COMPUTE_SHADER        0x91B9
#define GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT 0x00000001
#define GL_ELEMENT_ARRAY_BARRIER_BIT       0x00000002
#define GL_UNIFORM_BARRIER_BIT             0x00000004
#define GL_TEXTURE_FETCH_BARRIER_BIT       0x00000008
#define GL_SHADER_IMAGE_ACCESS_BARRIER_BIT 0x00000020
#define GL_COMMAND_BARRIER_BIT             0x00000040
#define GL_BUFFER_UPDATE_BARRIER_BIT       0x00000200
#define GL_SHADER_STORAGE_BARRIER_BIT      0x00002000
#define GL_ALL_BARRIER_BITS                0xFFFFFFFF
#define GL_READ_ONLY             0x88B8
#define GL_WRITE_ONLY            0x88B9
#define GL_READ_WRITE            0x88BA
#define GL_RGBA32F               0x8814
#define GL_R32F                  0x822E
#define GL_RG32F                 0x8230
#define GL_R32UI                 0x8236
#define GL_DYNAMIC_COPY          0x88EA
#define GL_STATIC_COPY           0x88E6
#define GL_TEXTURE_CUBE_MAP     0x8513
#define GL_TEXTURE_CUBE_MAP_POSITIVE_X 0x8515
#define GL_TEXTURE_CUBE_MAP_SEAMLESS   0x884F
#define GL_TEXTURE_WRAP_R       0x8072
#define GL_TEXTURE_MAX_LEVEL    0x813D
#define GL_DEPTH_COMPONENT32    0x81A7
#define GL_TEXTURE0             0x84C0
#define GL_TEXTURE1             0x84C1
#define GL_TEXTURE2             0x84C2
#define GL_TEXTURE3             0x84C3
#define GL_TEXTURE6             0x84C6
#define GL_CLAMP_TO_EDGE        0x812F
#define GL_CLAMP_TO_BORDER      0x812D
#define GL_MIRRORED_REPEAT      0x8370
#define GL_TEXTURE_BORDER_COLOR 0x1004
#define GL_MULTISAMPLE          0x809D
#define GL_TEXTURE_MAX_ANISOTROPY 0x84FE
#define GL_MAX_TEXTURE_MAX_ANISOTROPY 0x84FF
#define GL_TEXTURE_SWIZZLE_RGBA 0x8E46
#define GL_RED                  0x1903
#define GL_RG                   0x8227
#define GL_R8                   0x8229
#define GL_RG8                  0x822B
#define GL_SRGB8_ALPHA8         0x8C43
#define GL_FRAMEBUFFER_SRGB     0x8DB9
#define GL_BGRA                 0x80E1
#define GL_UNSIGNED_INT_8_8_8_8_REV 0x8367

// ---- framebuffers, render targets and the formats they need ----
#define GL_FRAMEBUFFER          0x8D40
#define GL_READ_FRAMEBUFFER     0x8CA8
#define GL_DRAW_FRAMEBUFFER     0x8CA9
#define GL_RENDERBUFFER         0x8D41
#define GL_COLOR_ATTACHMENT0    0x8CE0
#define GL_DEPTH_ATTACHMENT     0x8D00
#define GL_DEPTH_STENCIL_ATTACHMENT 0x821A
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#define GL_DEPTH_COMPONENT24    0x81A6
#define GL_DEPTH_COMPONENT32F   0x8CAC
#define GL_DEPTH24_STENCIL8     0x88F0
#define GL_RGBA16F              0x881A
#define GL_RGB16F               0x881B
#define GL_R11F_G11F_B10F       0x8C3A
#define GL_HALF_FLOAT           0x140B
#define GL_TIME_ELAPSED         0x88BF
#define GL_QUERY_RESULT         0x8866
#define GL_QUERY_RESULT_AVAILABLE 0x8867
#define GL_TEXTURE_COMPARE_MODE 0x884C
#define GL_TEXTURE_COMPARE_FUNC 0x884D
#define GL_COMPARE_REF_TO_TEXTURE 0x884E

// ---- debug output (GL 4.3) ----
#define GL_DEBUG_OUTPUT             0x92E0
#define GL_DEBUG_OUTPUT_SYNCHRONOUS 0x8242
#define GL_DEBUG_SEVERITY_HIGH      0x9146
#define GL_DEBUG_SEVERITY_MEDIUM    0x9147
#define GL_DEBUG_SEVERITY_LOW       0x9148
#define GL_DEBUG_SEVERITY_NOTIFICATION 0x826B

typedef void (APIENTRY* GLDEBUGPROC)(GLenum source, GLenum type, GLuint id, GLenum severity,
                                     GLsizei length, const GLchar* message, const void* user);

typedef GLuint(APIENTRY* PFN_glCreateShader)(GLenum);
typedef void  (APIENTRY* PFN_glShaderSource)(GLuint, GLsizei, const GLchar* const*, const GLint*);
typedef void  (APIENTRY* PFN_glCompileShader)(GLuint);
typedef void  (APIENTRY* PFN_glGetShaderiv)(GLuint, GLenum, GLint*);
typedef void  (APIENTRY* PFN_glGetShaderInfoLog)(GLuint, GLsizei, GLsizei*, GLchar*);
typedef GLuint(APIENTRY* PFN_glCreateProgram)(void);
typedef void  (APIENTRY* PFN_glAttachShader)(GLuint, GLuint);
typedef void  (APIENTRY* PFN_glLinkProgram)(GLuint);
typedef void  (APIENTRY* PFN_glGetProgramiv)(GLuint, GLenum, GLint*);
typedef void  (APIENTRY* PFN_glGetProgramInfoLog)(GLuint, GLsizei, GLsizei*, GLchar*);
typedef void  (APIENTRY* PFN_glDeleteShader)(GLuint);
typedef void  (APIENTRY* PFN_glDeleteProgram)(GLuint);
typedef void  (APIENTRY* PFN_glUseProgram)(GLuint);
typedef void  (APIENTRY* PFN_glGenVertexArrays)(GLsizei, GLuint*);
typedef void  (APIENTRY* PFN_glBindVertexArray)(GLuint);
typedef void  (APIENTRY* PFN_glGenBuffers)(GLsizei, GLuint*);
typedef void  (APIENTRY* PFN_glBindBuffer)(GLenum, GLuint);
typedef void  (APIENTRY* PFN_glBufferData)(GLenum, GLsizeiptr, const void*, GLenum);
typedef void  (APIENTRY* PFN_glVertexAttribPointer)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void*);
typedef void  (APIENTRY* PFN_glVertexAttribIPointer)(GLuint, GLint, GLenum, GLsizei, const void*);
typedef void  (APIENTRY* PFN_glEnableVertexAttribArray)(GLuint);
typedef void  (APIENTRY* PFN_glDisableVertexAttribArray)(GLuint);
typedef void  (APIENTRY* PFN_glVertexAttrib4f)(GLuint, GLfloat, GLfloat, GLfloat, GLfloat);
typedef void  (APIENTRY* PFN_glVertexAttribDivisor)(GLuint, GLuint);
typedef void  (APIENTRY* PFN_glDrawElementsInstanced)(GLenum, GLsizei, GLenum, const void*, GLsizei);
typedef void  (APIENTRY* PFN_glBufferSubData)(GLenum, GLintptr, GLsizeiptr, const void*);
typedef void  (APIENTRY* PFN_glBindBufferBase)(GLenum, GLuint, GLuint);
typedef void  (APIENTRY* PFN_glDrawElementsInstancedBaseVertex)(GLenum, GLsizei, GLenum, const void*, GLsizei, GLint);
typedef void  (APIENTRY* PFN_glDrawElementsBaseVertex)(GLenum, GLsizei, GLenum, const void*, GLint);
typedef void  (APIENTRY* PFN_glActiveTexture)(GLenum);
typedef void  (APIENTRY* PFN_glGenerateMipmap)(GLenum);
typedef GLint (APIENTRY* PFN_glGetUniformLocation)(GLuint, const GLchar*);
typedef void  (APIENTRY* PFN_glUniformMatrix4fv)(GLint, GLsizei, GLboolean, const GLfloat*);
typedef void  (APIENTRY* PFN_glUniform3f)(GLint, GLfloat, GLfloat, GLfloat);
typedef void  (APIENTRY* PFN_glUniform1i)(GLint, GLint);
typedef void  (APIENTRY* PFN_glUniform2f)(GLint, GLfloat, GLfloat);
typedef void  (APIENTRY* PFN_glUniform4f)(GLint, GLfloat, GLfloat, GLfloat, GLfloat);
typedef void  (APIENTRY* PFN_glUniform4fv)(GLint, GLsizei, const GLfloat*);
typedef void  (APIENTRY* PFN_glUniform1f)(GLint, GLfloat);
typedef void  (APIENTRY* PFN_glDrawArraysInstanced)(GLenum, GLint, GLsizei, GLsizei);
typedef void  (APIENTRY* PFN_glGenFramebuffers)(GLsizei, GLuint*);
typedef void  (APIENTRY* PFN_glBindFramebuffer)(GLenum, GLuint);
typedef void  (APIENTRY* PFN_glDeleteFramebuffers)(GLsizei, const GLuint*);
typedef void  (APIENTRY* PFN_glFramebufferTexture2D)(GLenum, GLenum, GLenum, GLuint, GLint);
typedef GLenum(APIENTRY* PFN_glCheckFramebufferStatus)(GLenum);
typedef void  (APIENTRY* PFN_glGenRenderbuffers)(GLsizei, GLuint*);
typedef void  (APIENTRY* PFN_glBindRenderbuffer)(GLenum, GLuint);
typedef void  (APIENTRY* PFN_glDeleteRenderbuffers)(GLsizei, const GLuint*);
typedef void  (APIENTRY* PFN_glRenderbufferStorage)(GLenum, GLenum, GLsizei, GLsizei);
typedef void  (APIENTRY* PFN_glFramebufferRenderbuffer)(GLenum, GLenum, GLenum, GLuint);
typedef void  (APIENTRY* PFN_glBlitFramebuffer)(GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLbitfield, GLenum);
typedef void  (APIENTRY* PFN_glDeleteBuffers)(GLsizei, const GLuint*);
typedef void  (APIENTRY* PFN_glDeleteVertexArrays)(GLsizei, const GLuint*);
// Draws a run of instances starting part way into the instance buffer, which
// is what lets a whole frame's transforms go up in one upload instead of one
// per batch - see pix__instance_upload.
typedef void  (APIENTRY* PFN_glDrawElementsInstancedBaseVertexBaseInstance)(GLenum, GLsizei, GLenum, const void*, GLsizei, GLint, GLuint);
// timer queries: GL_TIME_ELAPSED around a pass is the only way to find out
// what the GPU actually spent on it - every draw call returns long before the
// work does, so nothing measured on the CPU can tell one pass from another
typedef void  (APIENTRY* PFN_glGenQueries)(GLsizei, GLuint*);
typedef void  (APIENTRY* PFN_glDeleteQueries)(GLsizei, const GLuint*);
typedef void  (APIENTRY* PFN_glBeginQuery)(GLenum, GLuint);
typedef void  (APIENTRY* PFN_glEndQuery)(GLenum);
typedef void  (APIENTRY* PFN_glGetQueryObjectiv)(GLuint, GLenum, GLint*);
typedef void  (APIENTRY* PFN_glGetQueryObjectui64v)(GLuint, GLenum, GLuint64*);
typedef void  (APIENTRY* PFN_glDebugMessageCallback)(GLDEBUGPROC, const void*);
typedef void  (APIENTRY* PFN_glBlendFuncSeparate)(GLenum, GLenum, GLenum, GLenum);
typedef void  (APIENTRY* PFN_glCopyBufferSubData)(GLenum, GLenum, GLintptr, GLintptr, GLsizeiptr);
typedef void  (APIENTRY* PFN_glDispatchCompute)(GLuint, GLuint, GLuint);
typedef void  (APIENTRY* PFN_glDispatchComputeIndirect)(GLintptr);
typedef void  (APIENTRY* PFN_glMemoryBarrier)(GLbitfield);
typedef void  (APIENTRY* PFN_glBindImageTexture)(GLuint, GLuint, GLint, GLboolean, GLint, GLenum, GLenum);
typedef void  (APIENTRY* PFN_glGetBufferSubData)(GLenum, GLintptr, GLsizeiptr, void*);
typedef void  (APIENTRY* PFN_glBindBufferRange)(GLenum, GLuint, GLuint, GLintptr, GLsizeiptr);
typedef void  (APIENTRY* PFN_glDrawArraysIndirect)(GLenum, const void*);
typedef void  (APIENTRY* PFN_glDrawElementsIndirect)(GLenum, GLenum, const void*);
typedef void  (APIENTRY* PFN_glMultiDrawElementsIndirect)(GLenum, GLenum, const void*, GLsizei, GLsizei);
typedef void  (APIENTRY* PFN_glTexStorage2D)(GLenum, GLsizei, GLenum, GLsizei, GLsizei);
typedef void  (APIENTRY* PFN_glUniform1ui)(GLint, GLuint);
typedef void  (APIENTRY* PFN_glUniform2i)(GLint, GLint, GLint);
typedef void  (APIENTRY* PFN_glUniform1fv)(GLint, GLsizei, const GLfloat*);
typedef void  (APIENTRY* PFN_glUniform3fv)(GLint, GLsizei, const GLfloat*);
typedef void  (APIENTRY* PFN_glCopyImageSubData)(GLuint, GLenum, GLint, GLint, GLint, GLint, GLuint, GLenum, GLint, GLint, GLint, GLint, GLsizei, GLsizei, GLsizei);

#define GL_FUNC_LIST \
    E(glCreateShader) E(glShaderSource) E(glCompileShader) E(glGetShaderiv) \
    E(glGetShaderInfoLog) E(glCreateProgram) E(glAttachShader) E(glLinkProgram) \
    E(glGetProgramiv) E(glGetProgramInfoLog) E(glDeleteShader) E(glDeleteProgram) E(glUseProgram) \
    E(glGenVertexArrays) E(glBindVertexArray) E(glGenBuffers) E(glBindBuffer) \
    E(glBufferData) E(glBufferSubData) E(glBindBufferBase) E(glVertexAttribPointer) E(glVertexAttribIPointer) \
    E(glEnableVertexAttribArray) E(glDisableVertexAttribArray) E(glVertexAttrib4f) \
    E(glVertexAttribDivisor) E(glDrawElementsInstanced) E(glDrawElementsInstancedBaseVertex) \
    E(glDrawElementsBaseVertex) E(glDrawArraysInstanced) \
    E(glActiveTexture) E(glGenerateMipmap) E(glGetUniformLocation) \
    E(glUniformMatrix4fv) E(glUniform3f) E(glUniform1i) E(glUniform2f) E(glUniform4f) E(glUniform1f) \
    E(glUniform4fv) \
    E(glGenFramebuffers) E(glBindFramebuffer) E(glDeleteFramebuffers) E(glFramebufferTexture2D) \
    E(glCheckFramebufferStatus) E(glGenRenderbuffers) E(glBindRenderbuffer) \
    E(glDeleteRenderbuffers) E(glRenderbufferStorage) E(glFramebufferRenderbuffer) \
    E(glBlitFramebuffer) E(glDeleteBuffers) E(glDeleteVertexArrays) \
    E(glDrawElementsInstancedBaseVertexBaseInstance) \
    E(glGenQueries) E(glDeleteQueries) E(glBeginQuery) E(glEndQuery) \
    E(glGetQueryObjectiv) E(glGetQueryObjectui64v) E(glBlendFuncSeparate) E(glCopyBufferSubData) E(glCopyImageSubData) \
    E(glDispatchCompute) E(glDispatchComputeIndirect) E(glMemoryBarrier) E(glBindImageTexture) \
    E(glGetBufferSubData) E(glBindBufferRange) E(glDrawArraysIndirect) E(glDrawElementsIndirect) \
    E(glMultiDrawElementsIndirect) E(glTexStorage2D) E(glUniform1ui) E(glUniform2i) E(glUniform1fv) E(glUniform3fv)

#define E(name) extern PFN_##name name;
GL_FUNC_LIST
#undef E
// optional: null on a driver without GL 4.3 debug output
extern PFN_glDebugMessageCallback glDebugMessageCallback;

// Resolves every entry point above. Needs a current context (pix_create_window
// makes one). Returns false if any required function is missing.
PIXDEF bool opengl_load_functions();
// Routes the driver's own error/warning messages to pix_log. Only useful on a
// context created with PIX_WINDOW_DEBUG_GL.
PIXDEF void opengl_enable_debug_output();

#endif // PIX_OPENGL_API_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_OPENGL_API_IMPL)
#define PIX_OPENGL_API_IMPL

#define E(name) PFN_##name name;
GL_FUNC_LIST
#undef E
PFN_glDebugMessageCallback glDebugMessageCallback;

static void* gl__get(const char* name) {
    void* p = (void*)wglGetProcAddress(name);
    if (p == 0 || p == (void*)1 || p == (void*)2 || p == (void*)3 || p == (void*)-1) {
        HMODULE m = GetModuleHandleA("opengl32.dll");
        p = (void*)GetProcAddress(m, name);
    }
    return p;
}

bool opengl_load_functions() {
    bool ok = true;
#define E(name) name = (PFN_##name)gl__get(#name); if (!name) { ok = false; pix_log("pix: missing GL entry point %s", #name); }
    GL_FUNC_LIST
#undef E
    glDebugMessageCallback = (PFN_glDebugMessageCallback)gl__get("glDebugMessageCallback");
    return ok;
}

static void APIENTRY gl__debug_message(GLenum source, GLenum type, GLuint id, GLenum severity,
                                       GLsizei length, const GLchar* message, const void* user) {
    (void)source; (void)type; (void)id; (void)length; (void)user;
    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) return;
    pix_log("GL %s: %s", severity == GL_DEBUG_SEVERITY_HIGH ? "error" :
                         severity == GL_DEBUG_SEVERITY_MEDIUM ? "warning" : "note", message);
}

void opengl_enable_debug_output() {
    if (!glDebugMessageCallback) return;
    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDebugMessageCallback(gl__debug_message, 0);
}

#endif // PIX_OPENGL_API_IMPL
