#pragma once
#include <windows.h>
#include <GL/gl.h>

// ---- types missing from the OS <GL/gl.h> (OpenGL 1.1) ----
typedef char      GLchar;
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
#define GL_TEXTURE0             0x84C0
#define GL_CLAMP_TO_EDGE        0x812F
#define GL_MULTISAMPLE          0x809D

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
typedef void  (APIENTRY* PFN_glDrawElementsInstancedBaseVertex)(GLenum, GLsizei, GLenum, const void*, GLsizei, GLint);
typedef void  (APIENTRY* PFN_glActiveTexture)(GLenum);
typedef void  (APIENTRY* PFN_glGenerateMipmap)(GLenum);
typedef GLint (APIENTRY* PFN_glGetUniformLocation)(GLuint, const GLchar*);
typedef void  (APIENTRY* PFN_glUniformMatrix4fv)(GLint, GLsizei, GLboolean, const GLfloat*);
typedef void  (APIENTRY* PFN_glUniform3f)(GLint, GLfloat, GLfloat, GLfloat);
typedef void  (APIENTRY* PFN_glUniform1i)(GLint, GLint);
typedef void  (APIENTRY* PFN_glUniform2f)(GLint, GLfloat, GLfloat);
typedef void  (APIENTRY* PFN_glUniform4f)(GLint, GLfloat, GLfloat, GLfloat, GLfloat);
typedef void  (APIENTRY* PFN_glUniform1f)(GLint, GLfloat);
typedef void  (APIENTRY* PFN_glDrawArraysInstanced)(GLenum, GLint, GLsizei, GLsizei);

#define GL_FUNC_LIST \
    E(glCreateShader) E(glShaderSource) E(glCompileShader) E(glGetShaderiv) \
    E(glGetShaderInfoLog) E(glCreateProgram) E(glAttachShader) E(glLinkProgram) \
    E(glGetProgramiv) E(glGetProgramInfoLog) E(glDeleteShader) E(glUseProgram) \
    E(glGenVertexArrays) E(glBindVertexArray) E(glGenBuffers) E(glBindBuffer) \
    E(glBufferData) E(glBufferSubData) E(glVertexAttribPointer) E(glVertexAttribIPointer) E(glEnableVertexAttribArray) \
    E(glVertexAttribDivisor) E(glDrawElementsInstanced) E(glDrawElementsInstancedBaseVertex) \
    E(glDrawArraysInstanced) \
    E(glActiveTexture) E(glGenerateMipmap) E(glGetUniformLocation) \
    E(glUniformMatrix4fv) E(glUniform3f) E(glUniform1i) E(glUniform2f) E(glUniform4f) E(glUniform1f)

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
