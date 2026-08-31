#pragma once
#include <windows.h>
#include "dtype.hpp"
#include "opengl_api.hpp"

#define TEXTURE_PIXELATED 1
#define TEXTURE_LINEAR    2
#define TEXTURE_BILINEAR  3 // linear + mipmaps

static idx opengl__compile(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, 0);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024] = {};
        glGetShaderInfoLog(s, sizeof(log), 0, log);
        MessageBoxA(0, log, "shader compile error", MB_OK);
    }
    return s;
}

static idx opengl_create_shader(const char* vsrc, const char* fsrc) {
    GLuint v = opengl__compile(GL_VERTEX_SHADER, vsrc);
    GLuint f = opengl__compile(GL_FRAGMENT_SHADER, fsrc);
    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024] = {};
        glGetProgramInfoLog(p, sizeof(log), 0, log);
        MessageBoxA(0, log, "shader link error", MB_OK);
    }
    glDeleteShader(v);
    glDeleteShader(f);
    return p;
}

// data is expected to be tightly packed RGBA8
static idx opengl_create_texture2d(int width, int height, int channel, void* data, idx flag = TEXTURE_PIXELATED) {
    (void)channel;
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

    if (flag == TEXTURE_BILINEAR) {
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    } else if (flag == TEXTURE_LINEAR) {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    return tex;
}
