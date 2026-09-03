#pragma once
#include <stdint.h>

#define MB (1024 * 1024)

typedef uint32_t idx;

struct vec2 { float x; float y; };
struct vec3 { float x; float y; float z; };
struct vec4 { float x; float y; float z; float w; };

struct quat { float x; float y; float z; float w; }; // rotation, w last (glTF order)

struct mat4 { float data[16]; }; // column-major

struct mem_arena {
    char*  data;
    size_t size;
    size_t capacity;
};

template<typename T>
static T* allocate(mem_arena& arena, size_t count) {
    size_t bytes = count * sizeof(T);
    size_t at = (arena.size + 15) & ~(size_t)15; // 16-byte align
    if (at + bytes > arena.capacity) return nullptr;
    T* ptr = (T*)(arena.data + at);
    arena.size = at + bytes;
    return ptr;
}

static void arena_reset(mem_arena& arena) { arena.size = 0; }
