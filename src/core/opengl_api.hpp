#pragma once
#include <windows.h>
#include <GL/gl.h>

// ---- types missing from the OS <GL/gl.h> (OpenGL 1.1) ----
typedef char      GLchar;
typedef unsigned long long GLuint64;
typedef ptrdiff_t GLsizeiptr;
typedef ptrdiff_t GLintptr;

// ---- enums used by the 4.4 core entry points below ----
#define GL_VERTEX_SHADER        0x8B31
#define GL_FRAGMENT_SHADER      0x8B30
#define GL_COMPILE_STATUS       0x8B81
#define GL_LINK_STATUS          0x8B82
#define GL_ARRAY_BUFFER         0x8892
#define GL_ELEMENT_ARRAY_BUFFER 0x8893
#define GL_STATIC_DRAW          0x88E4
#define GL_DYNAMIC_DRAW         0x88E8
#define GL_STREAM_DRAW          0x88E0
#define GL_UNIFORM_BUFFER       0x8A11
#define GL_TEXTURE0             0x84C0
#define GL_CLAMP_TO_EDGE        0x812F
#define GL_CLAMP_TO_BORDER      0x812D
#define GL_TEXTURE_BORDER_COLOR 0x1004
#define GL_MULTISAMPLE          0x809D

// ---- framebuffers, render targets and the formats they need ----
#define GL_FRAMEBUFFER          0x8D40
#define GL_READ_FRAMEBUFFER     0x8CA8
#define GL_DRAW_FRAMEBUFFER     0x8CA9
#define GL_RENDERBUFFER         0x8D41
#define GL_COLOR_ATTACHMENT0    0x8CE0
#define GL_DEPTH_ATTACHMENT     0x8D00
#define GL_FRAMEBUFFER_COMPLETE 0x8CD5
#define GL_DEPTH_COMPONENT24    0x81A6
#define GL_DEPTH_COMPONENT32F   0x8CAC
#define GL_RGBA16F              0x881A
#define GL_TIME_ELAPSED         0x88BF
#define GL_QUERY_RESULT         0x8866
#define GL_QUERY_RESULT_AVAILABLE 0x8867
#define GL_RGB16F               0x881B
#define GL_HALF_FLOAT           0x140B
#define GL_TEXTURE_COMPARE_MODE 0x884C
#define GL_TEXTURE_COMPARE_FUNC 0x884D
#define GL_COMPARE_REF_TO_TEXTURE 0x884E
#define GL_TEXTURE1             0x84C1
#define GL_TEXTURE2             0x84C2
#define GL_TEXTURE3             0x84C3
#define GL_MIRRORED_REPEAT      0x8370

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
typedef void  (APIENTRY* PFN_glUseProgram)(GLuint);
typedef void  (APIENTRY* PFN_glGenVertexArrays)(GLsizei, GLuint*);
typedef void  (APIENTRY* PFN_glBindVertexArray)(GLuint);
typedef void  (APIENTRY* PFN_glGenBuffers)(GLsizei, GLuint*);
typedef void  (APIENTRY* PFN_glBindBuffer)(GLenum, GLuint);
typedef void  (APIENTRY* PFN_glBufferData)(GLenum, GLsizeiptr, const void*, GLenum);
typedef void  (APIENTRY* PFN_glVertexAttribPointer)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void*);
typedef void  (APIENTRY* PFN_glVertexAttribIPointer)(GLuint, GLint, GLenum, GLsizei, const void*);
typedef void  (APIENTRY* PFN_glEnableVertexAttribArray)(GLuint);
typedef void  (APIENTRY* PFN_glVertexAttribDivisor)(GLuint, GLuint);
typedef void  (APIENTRY* PFN_glDrawElementsInstanced)(GLenum, GLsizei, GLenum, const void*, GLsizei);
typedef void  (APIENTRY* PFN_glBufferSubData)(GLenum, GLintptr, GLsizeiptr, const void*);
typedef void  (APIENTRY* PFN_glBindBufferBase)(GLenum, GLuint, GLuint);
typedef void  (APIENTRY* PFN_glDrawElementsInstancedBaseVertex)(GLenum, GLsizei, GLenum, const void*, GLsizei, GLint);
typedef void  (APIENTRY* PFN_glActiveTexture)(GLenum);
typedef void  (APIENTRY* PFN_glGenerateMipmap)(GLenum);
typedef GLint (APIENTRY* PFN_glGetUniformLocation)(GLuint, const GLchar*);
typedef void  (APIENTRY* PFN_glUniformMatrix4fv)(GLint, GLsizei, GLboolean, const GLfloat*);
typedef void  (APIENTRY* PFN_glUniform3f)(GLint, GLfloat, GLfloat, GLfloat);
typedef void  (APIENTRY* PFN_glUniform1i)(GLint, GLint);
typedef void  (APIENTRY* PFN_glUniform2f)(GLint, GLfloat, GLfloat);
typedef void  (APIENTRY* PFN_glUniform4f)(GLint, GLfloat, GLfloat, GLfloat, GLfloat);
// arrays: a frame's punctual lights go up as three of these
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

#define GL_FUNC_LIST \
    E(glCreateShader) E(glShaderSource) E(glCompileShader) E(glGetShaderiv) \
    E(glGetShaderInfoLog) E(glCreateProgram) E(glAttachShader) E(glLinkProgram) \
    E(glGetProgramiv) E(glGetProgramInfoLog) E(glDeleteShader) E(glUseProgram) \
    E(glGenVertexArrays) E(glBindVertexArray) E(glGenBuffers) E(glBindBuffer) \
    E(glBufferData) E(glBufferSubData) E(glBindBufferBase) E(glVertexAttribPointer) E(glVertexAttribIPointer) E(glEnableVertexAttribArray) \
    E(glVertexAttribDivisor) E(glDrawElementsInstanced) E(glDrawElementsInstancedBaseVertex) \
    E(glDrawArraysInstanced) \
    E(glActiveTexture) E(glGenerateMipmap) E(glGetUniformLocation) \
    E(glUniformMatrix4fv) E(glUniform3f) E(glUniform1i) E(glUniform2f) E(glUniform4f) E(glUniform1f) \
    E(glUniform4fv) \
    E(glGenFramebuffers) E(glBindFramebuffer) E(glDeleteFramebuffers) E(glFramebufferTexture2D) \
    E(glCheckFramebufferStatus) E(glGenRenderbuffers) E(glBindRenderbuffer) \
    E(glDeleteRenderbuffers) E(glRenderbufferStorage) E(glFramebufferRenderbuffer) \
    E(glBlitFramebuffer) E(glDeleteBuffers) E(glDeleteVertexArrays) \
    E(glDrawElementsInstancedBaseVertexBaseInstance) \
    E(glGenQueries) E(glDeleteQueries) E(glBeginQuery) E(glEndQuery) \
    E(glGetQueryObjectiv) E(glGetQueryObjectui64v)

#define E(name) static PFN_##name name;
GL_FUNC_LIST
#undef E

// -------------------- implementation --------------------

static void* gl__get(const char* name) {
    void* p = (void*)wglGetProcAddress(name);
    if (p == 0 || p == (void*)1 || p == (void*)2 || p == (void*)3 || p == (void*)-1) {
        HMODULE m = GetModuleHandleA("opengl32.dll");
        p = (void*)GetProcAddress(m, name);
    }
    return p;
}

static bool opengl_load_functions() {
    bool ok = true;
#define E(name) name = (PFN_##name)gl__get(#name); if (!name) ok = false;
    GL_FUNC_LIST
#undef E
    return ok;
}
