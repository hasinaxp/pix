#pragma once
#include <stddef.h>
#include "dtype.hpp"
#include "math.hpp"
#include "opengl_api.hpp"
#include "opengl_utils.hpp"

#define MAX_SPRITES 1024
#define MAX_SPRITE_TEXTURES 32 // keep in sync with the uTex/uTexSize array sizes in shader_sources.hpp

struct sprite {
    vec4 box;    // x, y, w, h in the space of the view_proj passed to draw_sprites
    vec4 crop;   // atlas pixel rect x, y, w, h
    idx texture; // index into sprite_batch::tex_ids, from pix_batch_texture() - not a raw GL id
};


struct sprite_batch {
    idx vao;
    idx vbo;

    size_t size;
    size_t capacity;

    idx tex_ids[MAX_SPRITE_TEXTURES]; // distinct GL textures registered in this batch, first-seen order
    size_t tex_count;                 // <= MAX_SPRITE_TEXTURES
};

static sprite_batch pix_create_sprite_batch(size_t capacity = MAX_SPRITES);
static void pix_destroy_sprite_batch(sprite_batch& b);

// registers a GL texture with the batch (deduping) and returns its sprite::texture index
static idx pix_batch_texture(sprite_batch& b, idx gl_texture);

static void push_sprite(sprite_batch& b, const sprite & s);

static void replace_sprites(
    sprite_batch& b,
    const sprite* s,
    size_t count = 1,
    size_t offset = 0);

static void clear_sprites(sprite_batch& b);

static void draw_sprites(
    const sprite_batch& b,
    idx shader,
    const mat4& view_proj);

//----------------------------implementation----------------------------

// one unit quad shared by every batch; per-instance box/crop/texture-index
// attributes (divisor 1) place, crop and texture it
static idx sprite__quad_vbo() {
    static idx vbo = 0;
    if (!vbo) {
        float corners[8] = { 0,0, 1,0, 0,1, 1,1 };
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(corners), corners, GL_STATIC_DRAW);
    }
    return vbo;
}

// a sprite_batch is a baked GPU resource: push_sprite/replace_sprites write
// straight into b.vbo via glBufferSubData, and draw_sprites never touches its
// contents - it just binds the batch's registered textures and issues one
// instanced draw over whatever is already resident
static sprite_batch pix_create_sprite_batch(size_t capacity) {
    sprite_batch b = {};
    b.capacity = capacity;

    glGenVertexArrays(1, &b.vao);
    glBindVertexArray(b.vao);

    glBindBuffer(GL_ARRAY_BUFFER, sprite__quad_vbo());
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &b.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(capacity * sizeof(sprite)), 0, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(sprite), (void*)offsetof(sprite, box));
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(sprite), (void*)offsetof(sprite, crop));
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);
    glVertexAttribIPointer(3, 1, GL_UNSIGNED_INT, sizeof(sprite), (void*)offsetof(sprite, texture));
    glEnableVertexAttribArray(3);
    glVertexAttribDivisor(3, 1);

    glBindVertexArray(0);
    return b;
}

static void pix_destroy_sprite_batch(sprite_batch& b) {
    (void)b; // nothing heap-allocated to free; the VAO/VBO live for the program's lifetime
}

static idx pix_batch_texture(sprite_batch& b, idx gl_texture) {
    for (size_t i = 0; i < b.tex_count; i++) if (b.tex_ids[i] == gl_texture) return (idx)i;
    if (b.tex_count >= MAX_SPRITE_TEXTURES) return (idx)(b.tex_count - 1); // full: reuse the last slot
    b.tex_ids[b.tex_count] = gl_texture;
    return (idx)b.tex_count++;
}

static void push_sprite(sprite_batch& b, const sprite& s) {
    if (b.size >= b.capacity) return;
    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferSubData(GL_ARRAY_BUFFER, (GLintptr)(b.size * sizeof(sprite)), sizeof(sprite), &s);
    b.size++;
}

static void replace_sprites(sprite_batch& b, const sprite* s, size_t count, size_t offset) {
    if (offset >= b.capacity) return;
    if (offset + count > b.capacity) count = b.capacity - offset;

    glBindBuffer(GL_ARRAY_BUFFER, b.vbo);
    glBufferSubData(GL_ARRAY_BUFFER, (GLintptr)(offset * sizeof(sprite)), (GLsizeiptr)(count * sizeof(sprite)), s);
    if (offset + count > b.size) b.size = offset + count;
}

static void clear_sprites(sprite_batch& b) {
    b.size = 0;
}

static void draw_sprites(const sprite_batch& b, idx shader, const mat4& view_proj) {
    if (!b.size || !b.tex_count) return;

    glUseProgram(shader);
    glUniformMatrix4fv(glGetUniformLocation(shader, "uViewProj"), 1, GL_FALSE, view_proj.data);

    // array elements sit at consecutive uniform locations after the first
    GLint u_tex0     = glGetUniformLocation(shader, "uTex[0]");
    GLint u_texsize0 = glGetUniformLocation(shader, "uTexSize[0]");
    for (size_t i = 0; i < b.tex_count; i++) {
        glActiveTexture(GL_TEXTURE0 + (GLenum)i);
        glBindTexture(GL_TEXTURE_2D, b.tex_ids[i]);
        glUniform1i(u_tex0 + (GLint)i, (GLint)i);

        GLint tw = 1, th = 1;
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &tw);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &th);
        glUniform2f(u_texsize0 + (GLint)i, (float)tw, (float)th);
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);

    glBindVertexArray(b.vao);
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, (GLsizei)b.size);
    glBindVertexArray(0);

    glDisable(GL_BLEND);
}
