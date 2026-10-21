# Memory

**You don't need this page to use pix.** Every function works without you
thinking about memory. Things you load are owned by pix and freed by
`pix_shutdown`. The few functions that hand you a buffer of your own say so,
and you release it with `pix_free`.

Read on when you want more control: faster temporary memory, fewer heap
allocations in your own code, tighter limits, a custom allocator, or a look
at how much memory is in use.

## Level 0: what you have to know

Most functions give you a **handle** (`idx`) or a small struct, and pix keeps
the memory behind it. You never free those. Releasing them happens in
`pix_shutdown`, or with an explicit unload such as `pix_unload_texture` and
`pix_unload_font`.

A handful of functions return a **buffer that you own**:

| function | returns |
|---|---|
| `pix_read_file(path, &size)` | a whole file, zero-terminated |
| `pix_load_image_rgba(path, &w, &h)` | decoded RGBA pixels |
| `pix_mesh_copy(...)`, `pix_mesh_copy_data(...)`, `pix_mesh_merge(...)` | a `pix_mesh_data` |
| `pix_anim_bone_mask(skeleton, bone)` | a per-bone weight array |

Free these with `pix_free(p)`, or `pix_mesh_free(mesh)` for meshes, when you
are done:

```cpp
size_t size;
unsigned char* level = pix_read_file("assets/level1.txt", &size);
parse_level(level, size);
pix_free(level);
```

That is all you need. Everything below is optional.

## Level 1: temporary results without freeing

Often a buffer is needed only for a moment: read a file, parse it, throw it
away. Instead of freeing each buffer, open a **scope** and ask for the
results to be **scoped**. Everything allocated in the scope disappears
together when it ends:

```cpp
pix_mem_scope s = pix_scope_begin();

size_t size;
unsigned char* level = pix_read_file("assets/level1.txt", &size, PIX_MEM_SCOPED);
int w, h;
unsigned char* heights = pix_load_image_rgba("assets/height.png", &w, &h, PIX_MEM_SCOPED);
build_terrain(level, heights, w, h);

pix_scope_end(s);          // level and heights are gone; no pix_free calls
```

Every function in the table above takes this flag as its last argument. The
default is `PIX_MEM_GLOBAL`, which is the "free it yourself" behaviour from
Level 0.

Scoped memory is also **much faster** than the heap. Allocating is a pointer
bump, and ending the scope frees everything in one step, however many
allocations there were. Use it for loading screens, level generation and
per-frame scratch work.

Rules:

- A scoped allocation needs an open scope on the same thread. Without one it
  returns null and logs a message.
- Scopes nest. End them in reverse order, on the thread that began them.
- Don't keep pointers to scoped memory after its scope ends.
- Calling `pix_free` on scoped memory is harmless: it does nothing.

### Meshes

A mesh copied or merged as scoped stays scoped. Operations that rebuild its
arrays (`pix_mesh_subdivide`, `pix_mesh_recompute_normals`) allocate the new
arrays in the scope as well. This makes it cheap to build a mesh, upload it,
and forget the CPU copy:

```cpp
pix_mem_scope s = pix_scope_begin();
pix_mesh_data m = pix_mesh_copy(loader.mesh_files[id], PIX_MEM_SCOPED);
pix_mesh_weld(m);
pix_mesh_recompute_normals(m, 45.0f);
idx mesh = load_mesh_data(pix_get_renderer(), m.vertices, m.vertex_count, m.indices, m.index_count);
pix_scope_end(s);          // the CPU copy is gone; the GPU mesh stays
```

## Level 2: using pix's allocators in your own code

The same allocators are available for your own data.

### Allocating

```cpp
enemy* list = pix_new<enemy>(count);                         // global; contents undefined
float* grid = pix_new<float>(w * h, PIX_MEM_ZERO);           // global, zero-filled
int*   tmp  = pix_new<int>(n, PIX_MEM_SCOPED);               // scoped (inside a scope)
list = (enemy*)pix_mem_realloc(list, new_count * sizeof(enemy));
pix_free(list);
```

| flag | meaning |
|---|---|
| `PIX_MEM_GLOBAL` (default) | heap memory, lives until `pix_free` |
| `PIX_MEM_SCOPED` | lives until the enclosing `pix_scope_end` |
| `PIX_MEM_ZERO` | combine with either to get zero-filled memory |

`pix_alloc<T>(count)` is shorthand for a zero-filled global allocation.

### Stack buffers

For small temporary arrays in hot code, `PIX_STACK_BUFFER` puts the array on
the call stack, with no allocator call at all. If the count turns out larger
than the capacity you gave, it uses scoped memory instead, so the code stays
correct for any size:

```cpp
void find_targets(const enemy* enemies, int count) {
    pix_mem_scope s = pix_scope_begin();            // only used if count > 256
    PIX_STACK_BUFFER(int, hits, count, 256);         // int* hits, room for `count`
    int n = 0;
    for (int i = 0; i < count; i++) if (in_range(enemies[i])) hits[n++] = i;
    // ...
    pix_scope_end(s);
}
```

The capacity must be a constant, and the array may not exceed 16 KB
(`PIX_STACK_MAX_BYTES`), so a deep call chain can't overflow the stack.

### Arenas

An **arena** reserves a large range of address space and uses physical
memory only for the part you actually fill. Allocating from it is a pointer
bump. You free everything in it at once with `arena_reset` (keep the memory
for reuse) or `pix_arena_destroy` (give it back).

```cpp
mem_arena level_mem = pix_arena_create(256 * MB);    // costs nothing until used

// while loading a level:
tile*   tiles = allocate<tile>(level_mem, w * h);
entity* ents  = allocate<entity>(level_mem, entity_count);

// on level change:
arena_reset(level_mem);                              // everything gone, memory kept warm
pix_arena_trim(level_mem);                           // optional: return unused pages to the OS
```

Arenas suit data with one shared lifetime: a level, a loaded file set, a
procedural world. Pointers into an arena never move, so it is safe to keep
them until the reset.

You can also use an arena as a **growable table**: reserve the maximum size,
index it directly, and call `pix_arena_commit(arena, bytes)` as it grows.
Only the committed part uses memory.

The low-level file loaders (`obj_load_file`, `png_load_file`,
`gltf_load_file`, `wav_load_file`) take an arena to load into. The
`pix_data_loader` that `pix_get_loader()` returns is built on one.

## Level 3: tuning

### Statistics

```cpp
pix_mem_log_stats("after level load");
// prints something like:
// after level load: global 22.10 MB in 34 blocks (peak 23.00 MB), arenas 3.00 MB, scoped 0.00 MB on 1 threads

pix_mem_stats st = pix_mem_get_stats();
if (st.global_blocks > last_blocks) pix_log("something is leaking");
```

| field | |
|---|---|
| `global_bytes`, `global_blocks`, `global_peak` | live heap memory, how many blocks, and the highest it has been |
| `arena_committed` | physical memory backing all arenas |
| `scoped_committed`, `scoped_threads` | memory backing the scope stacks, and how many threads have one |

A `global_blocks` count that keeps rising while the game runs steadily
usually means something isn't being freed.

### Limits

Define any of these before including pix:

| macro | default | |
|---|---|---|
| `PIX_MAX_INSTANCES` | 65536 | 3D objects per frame |
| `PIX_MAX_ANIMATED` | 8192 | animated characters per frame |
| `PIX_MAX_MESHES`, `PIX_MAX_MATERIALS` | 8192 each | |
| `PIX_MAX_PARTICLES` | 16384 | per frame |
| `PIX_MAX_LINES` | 32768 | debug lines per frame |
| `PHYS3D_FLAT_MAX_BODIES`, `PHYS3D_FLAT_MAX_STATICS` | 2048, 32768 | flat 3D physics world |
| `PHYS2D_MAX_BODIES`, `PHYS2D_MAX_JOINTS` | 4096 each | 2D physics world |
| `PHYS3D_MAX_BODIES`, `PHYS3D_MAX_JOINTS` | 4096 each | 3D physics world |
| `PIX_SCOPE_RESERVE` | 256 MB (64-bit) | the largest a single thread's scope stack can grow |
| `PIX_SCOPE_KEEP` | 8 MB | how much a scope stack keeps after its outermost scope ends, to stay warm |
| `PIX_SCOPE_THREADS` | 64 | threads that can hold a scope stack at once |
| `PIX_STACK_MAX_BYTES` | 16 KB | the largest `PIX_STACK_BUFFER` |

```cpp
#define PIX_MAX_INSTANCES 200000
#define PIX_SCOPE_KEEP (32 * 1024 * 1024)
#define PIX_IMPLEMENTATION
#include "pix.h"
```

Define them identically in every file that includes pix, or at least in the
one that defines `PIX_IMPLEMENTATION`, which is where they take effect.

The renderer's and physics world's tables are currently sized to these
limits when they are created. Lowering a limit that your game never comes
close to saves memory.

### Your own heap

All global allocations go through five macros. Define them before including
pix to plug in your own allocator, for example one that tracks memory per
system:

```cpp
#define PIX_MALLOC(bytes)       my_malloc(bytes)
#define PIX_CALLOC(count, size) my_calloc(count, size)
#define PIX_REALLOC(p, bytes)   my_realloc(p, bytes)
#define PIX_FREE(p)             my_free(p)
#define PIX_MSIZE(p)            my_block_size(p)    // usable size of a block, for the statistics
#define PIX_IMPLEMENTATION
#include "pix.h"
```

Define all five together. Scoped memory and arenas don't use them; they take
address space straight from the operating system.

### Threads

Each thread gets its own scope stack the first time it opens a scope, so
scoped memory never needs locking. pix's own worker threads (`pix_jobs`) give
theirs back when they stop. A thread of your own that used scoped memory
should call `pix_mem_thread_exit()` before it ends.

## Reference

| function | |
|---|---|
| `void* pix_mem_alloc(bytes, flags = PIX_MEM_GLOBAL)` | null on failure, or for scoped with no scope open |
| `void* pix_mem_realloc(p, bytes, flags = PIX_MEM_GLOBAL)` | keeps contents; the new part isn't zeroed. `flags` only matters when `p` is null. |
| `void pix_mem_free(p)`, `pix_free(p)` | global blocks; does nothing for scoped or null |
| `T* pix_new<T>(count, flags = PIX_MEM_GLOBAL)` | |
| `T* pix_alloc<T>(count)` | zero-filled global |
| `bool pix_mem_is_scoped(p)` | |
| `pix_mem_scope pix_scope_begin()`, `pix_scope_end(scope)` | |
| `pix_mem_thread_exit()` | release this thread's scope stack |
| `PIX_STACK_BUFFER(T, name, count, capacity)` | declares `T* name` |
| `mem_arena pix_arena_create(reserve)` | |
| `T* allocate<T>(arena, count)` | 16-byte aligned; null when full |
| `arena_reset(arena)`, `pix_arena_trim(arena)`, `pix_arena_destroy(arena)` | |
| `bool pix_arena_commit(arena, bytes)` | make the first `bytes` usable, for table-style arenas |
| `pix_mem_stats pix_mem_get_stats()`, `pix_mem_log_stats(label)` | |

Functions that return memory you own, each taking `PIX_MEM_GLOBAL` (default)
or `PIX_MEM_SCOPED` last:

| function | release a global result with |
|---|---|
| `pix_read_file(path, &size, mem)` | `pix_free` |
| `pix_load_image_rgba(path, &w, &h, mem)` | `pix_free` |
| `pix_mesh_copy(file, mem)`, `pix_mesh_copy_data(v, vc, i, ic, mem)`, `pix_mesh_merge(meshes, transforms, count, mem)` | `pix_mesh_free` |
| `pix_anim_bone_mask(skeleton, root_bone, mem)` | `pix_free` |
