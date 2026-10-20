// pix - memory. Every allocation pix makes goes through this module, in one of
// four places, picked by how long the memory has to live:
//
//   global   the heap. Lives until pix_mem_free. For anything owned long term.
//
//   scoped   a per-thread stack of address space. pix_scope_begin marks it,
//            allocations bump a pointer, pix_scope_end drops everything since
//            the mark in one step. For temporaries: a loader's working arrays,
//            a mesh operation's scratch, a decoded file about to be uploaded.
//            Costs an add and a compare, and nothing is ever freed one by one.
//
//   stack    PIX_STACK_BUFFER: a small array on the call stack itself, for
//            temporaries known to be small. No allocator call at all. When a
//            request does not fit, it falls back to the scoped stack.
//
//   arena    mem_arena: a bump allocator over reserved address space whose
//            pages are committed only as allocations reach them. For tables
//            sized to a maximum that is rarely reached (instances, bodies,
//            loaded assets): the reservation costs nothing, and the table
//            costs memory only for the part in use. Never moves, so pointers
//            into it stay valid.
//
// Functions that hand memory back to the caller take a PIX_MEM_* flag saying
// whether the result is global (free it with pix_mem_free) or scoped (gone at
// the caller's pix_scope_end).
//
//   pix_mem_scope s = pix_scope_begin();
//   float* tmp = pix_new<float>(n, PIX_MEM_SCOPED);
//   ...
//   pix_scope_end(s);                                // tmp is gone
//
//   PIX_STACK_BUFFER(uint32_t, ids, count, 256);     // up to 256 on the stack
//
// pix_mem_get_stats reports what each kind holds, which is how a leak or an
// oversized table shows up.
#ifndef PIX_MEMORY_HPP
#define PIX_MEMORY_HPP

// The single header puts this module first, so it carries dtype's rule:
// defined before the first CRT header anywhere in the file.
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef PIXDEF
#define PIXDEF extern
#endif

PIXDEF void pix_log(const char* fmt, ...);   // dtype.hpp; reports failures here

// ---- where an allocation lives ----
#define PIX_MEM_GLOBAL 0x0   // the heap, until pix_mem_free
#define PIX_MEM_SCOPED 0x1   // this thread's scope stack, until the enclosing pix_scope_end
#define PIX_MEM_ZERO   0x2   // zero-filled; without it the contents are undefined

// The global heap is the C runtime's unless these are defined before
// including pix. PIX_MSIZE must report the usable size of a block, for the
// statistics.
#ifndef PIX_MALLOC
#define PIX_MALLOC(bytes)       malloc(bytes)
#define PIX_CALLOC(count, size) calloc(count, size)
#define PIX_REALLOC(p, bytes)   realloc(p, bytes)
#define PIX_FREE(p)             free(p)
#define PIX_MSIZE(p)            _msize(p)
#endif

// ---- global and scoped allocation ----

// `bytes` from the place `flags` names. 0 bytes still returns a valid,
// distinct pointer. Null on failure, or for PIX_MEM_SCOPED when this thread
// has no scope open.
PIXDEF void* pix_mem_alloc(size_t bytes, unsigned flags = PIX_MEM_GLOBAL);
// Grows or shrinks a block from either place, keeping its contents; the new
// part is not zeroed. A scoped block grows in place when it is the newest one
// in its scope. Null `p` allocates (globally unless `flags` says scoped).
PIXDEF void* pix_mem_realloc(void* p, size_t bytes, unsigned flags = PIX_MEM_GLOBAL);
// Frees a global block. Scoped blocks are left to their scope, so passing one
// here is harmless and does nothing. Null is fine.
PIXDEF void  pix_mem_free(void* p);
// True if `p` lies in any thread's scope stack.
PIXDEF bool  pix_mem_is_scoped(const void* p);

template<typename T>
static inline T* pix_new(size_t count, unsigned flags = PIX_MEM_GLOBAL) {
    return (T*)pix_mem_alloc(count * sizeof(T), flags);
}

// zeroed global allocation of `count` T's; free with pix_free
template<typename T>
static inline T* pix_alloc(size_t count) { return pix_new<T>(count, PIX_MEM_GLOBAL | PIX_MEM_ZERO); }
static inline void pix_free(void* p) { pix_mem_free(p); }

// ---- scopes ----
//
// Scopes nest, and must end in the reverse order they began, on the thread
// that began them. Each thread has its own scope stack (up to
// PIX_SCOPE_THREADS threads at once), so job workers never contend.
struct pix_mem_scope { size_t mark; };
PIXDEF pix_mem_scope pix_scope_begin(void);
PIXDEF void          pix_scope_end(pix_mem_scope scope);
// Gives this thread's scope stack back. A thread that used scoped memory
// calls it before exiting (pix's own job workers do), or its slot stays taken.
PIXDEF void          pix_mem_thread_exit(void);

// ---- stack buffers ----
//
// Declares `T* name` holding `count` elements: an array of `capacity` on the
// call stack when it fits, else scoped memory (so a large request needs an
// open scope, and `name` is null without one). `capacity` is a constant, and
// the whole array must stay under PIX_STACK_MAX_BYTES so a deep call chain
// cannot overflow the stack.
#ifndef PIX_STACK_MAX_BYTES
#define PIX_STACK_MAX_BYTES (16 * 1024)
#endif
#define PIX_STACK_BUFFER(T, name, count, capacity)                                        \
    static_assert(sizeof(T) * (capacity) <= PIX_STACK_MAX_BYTES, "stack buffer too big"); \
    T name##__stack[capacity];                                                            \
    T* name = (size_t)(count) <= (size_t)(capacity) ? name##__stack                       \
                                                    : pix_new<T>((count), PIX_MEM_SCOPED)

// ---- arenas ----
//
// A bump allocator. Two flavours share the struct:
//
//   pix_arena_create(reserve)  reserves `reserve` bytes of *address space* and
//                              commits physical pages only as allocations
//                              reach them. Reserving a gigabyte costs nothing;
//                              a loader that only ever parses 3 MB of assets
//                              only ever holds 3 MB of RAM. This is the one to
//                              use.
//   { data, 0, capacity }      a caller-owned fixed buffer, filled in by hand.
//                              Still supported, never grows.
//
// Pointers handed out stay valid until the arena is reset or destroyed - the
// reservation never moves, which is the whole point of reserving up front
// rather than realloc-ing.
struct mem_arena {
    char*  data;
    size_t size;        // bytes handed out
    size_t capacity;    // bytes that may ever be handed out (the reservation)
    size_t committed;   // virtual arenas: bytes backed by physical pages so far
    bool   virt;        // true when created by pix_arena_create
};

PIXDEF mem_arena pix_arena_create(size_t reserve);
PIXDEF void      pix_arena_destroy(mem_arena& arena);
// Hands committed pages beyond the current size back to the OS. arena_reset
// keeps them, since the next frame or load usually wants them again.
PIXDEF void      pix_arena_trim(mem_arena& arena);
// Makes the first `end` bytes usable without handing them out - for an arena
// used as a table, indexed directly, that grows by commit alone. Newly
// committed pages read as zero.
PIXDEF bool      pix_arena_commit(mem_arena& arena, size_t end);
static inline bool pix__arena_commit(mem_arena& arena, size_t end) { return pix_arena_commit(arena, end); }

template<typename T>
static inline T* allocate(mem_arena& arena, size_t count) {
    size_t bytes = count * sizeof(T);
    size_t at = (arena.size + 15) & ~(size_t)15; // 16-byte align
    if (at + bytes > arena.capacity) return nullptr;
    if (arena.virt && at + bytes > arena.committed && !pix_arena_commit(arena, at + bytes))
        return nullptr;
    T* ptr = (T*)(arena.data + at);
    arena.size = at + bytes;
    return ptr;
}

static inline void arena_reset(mem_arena& arena) { arena.size = 0; }

// ---- statistics ----
struct pix_mem_stats {
    long long global_bytes;      // live global allocations
    long long global_peak;
    long long global_blocks;     // how many are live
    long long scoped_committed;  // pages backing every thread's scope stack
    long long arena_committed;   // pages backing every virtual arena
    int       scoped_threads;    // threads holding a scope stack
};
PIXDEF pix_mem_stats pix_mem_get_stats(void);
// Writes the stats through pix_log, prefixed by `label`.
PIXDEF void pix_mem_log_stats(const char* label);

#endif // PIX_MEMORY_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_MEMORY_IMPL)
#define PIX_MEMORY_IMPL
#include <malloc.h>
#include <windows.h>

// ---- statistics ----
//
// Plain interlocked adds: allocation is spread over the job workers, and a
// counter is cheaper than any lock around the heap.

static volatile long long pix__mem_global_bytes;
static volatile long long pix__mem_global_peak;
static volatile long long pix__mem_global_blocks;
static volatile long long pix__mem_arena_committed;
static volatile long long pix__mem_scoped_committed;

static void pix__mem_count_global(long long bytes, long long blocks) {
    long long now = InterlockedExchangeAdd64(&pix__mem_global_bytes, bytes) + bytes;
    if (blocks) InterlockedExchangeAdd64(&pix__mem_global_blocks, blocks);
    long long peak = pix__mem_global_peak;
    while (now > peak) {
        long long seen = InterlockedCompareExchange64(&pix__mem_global_peak, now, peak);
        if (seen == peak) break;
        peak = seen;
    }
}

// ---- scope stacks ----
//
// Every thread's scope stack lives in one reservation, slot after slot, so
// "is this pointer scoped" is a single range check no matter which thread
// made it. The reservation is address space only; pages are committed as a
// thread's stack grows. 64 x 256 MB is 16 GB of address space, a rounding
// error on x64.

#ifndef PIX_SCOPE_THREADS
#define PIX_SCOPE_THREADS 64
#endif
#ifndef PIX_SCOPE_RESERVE
#if defined(_WIN64)
#define PIX_SCOPE_RESERVE ((size_t)256 * 1024 * 1024)
#else
#define PIX_SCOPE_RESERVE ((size_t)16 * 1024 * 1024)
#endif
#endif
// What a stack keeps committed when its outermost scope ends: enough that the
// next frame's temporaries reuse warm pages, not so much that one huge load
// pins its memory for good.
#ifndef PIX_SCOPE_KEEP
#define PIX_SCOPE_KEEP ((size_t)8 * 1024 * 1024)
#endif

#define PIX__PAGE_STEP ((size_t)64 * 1024)          // commit granularity
#define PIX__MAX_COMMIT_STEP ((size_t)64 * 1024 * 1024)

struct pix__scope_slot {
    volatile long taken;
    size_t used;        // bytes handed out, the top of the stack
    size_t committed;
    int    depth;       // open scopes
};

static char* volatile   pix__scope_base;
static pix__scope_slot  pix__scope_slots[PIX_SCOPE_THREADS];
static volatile long    pix__scope_threads;
static __declspec(thread) int pix__my_slot = -1;

// A scoped block carries its size just before it, so realloc can copy it.
struct pix__scope_header { size_t size; size_t pad; };

static char* pix__scope_region(void) {
    char* base = pix__scope_base;
    if (base) return base;
    char* fresh = (char*)VirtualAlloc(0, PIX_SCOPE_RESERVE * PIX_SCOPE_THREADS, MEM_RESERVE, PAGE_READWRITE);
    if (!fresh) return 0;
    char* won = (char*)InterlockedCompareExchangePointer((void* volatile*)&pix__scope_base, fresh, 0);
    if (won) { VirtualFree(fresh, 0, MEM_RELEASE); return won; }   // another thread got there first
    return fresh;
}

static pix__scope_slot* pix__my_scope(void) {
    if (pix__my_slot >= 0) return &pix__scope_slots[pix__my_slot];
    if (!pix__scope_region()) return 0;
    for (int i = 0; i < PIX_SCOPE_THREADS; i++)
        if (InterlockedCompareExchange(&pix__scope_slots[i].taken, 1, 0) == 0) {
            pix__my_slot = i;
            InterlockedIncrement(&pix__scope_threads);
            return &pix__scope_slots[i];
        }
    return 0;
}

static char* pix__slot_base(int slot) { return pix__scope_base + (size_t)slot * PIX_SCOPE_RESERVE; }

// grows a committed prefix to cover `end`, by at least doubling (capped), in
// whole commit steps; `limit` is the reservation
static bool pix__commit_prefix(char* base, size_t& committed, size_t end, size_t limit,
                               volatile long long* counter) {
    if (end <= committed) return true;
    if (end > limit) return false;
    size_t grow = committed < PIX__MAX_COMMIT_STEP ? committed : PIX__MAX_COMMIT_STEP;
    size_t want = end > committed + grow ? end : committed + grow;
    want = (want + PIX__PAGE_STEP - 1) & ~(PIX__PAGE_STEP - 1);
    if (want > limit) want = limit;
    if (!VirtualAlloc(base + committed, want - committed, MEM_COMMIT, PAGE_READWRITE)) return false;
    InterlockedExchangeAdd64(counter, (long long)(want - committed));
    committed = want;
    return true;
}

static void pix__decommit_above(char* base, size_t& committed, size_t keep, volatile long long* counter) {
    keep = (keep + PIX__PAGE_STEP - 1) & ~(PIX__PAGE_STEP - 1);
    if (keep >= committed) return;
    VirtualFree(base + keep, committed - keep, MEM_DECOMMIT);
    InterlockedExchangeAdd64(counter, -(long long)(committed - keep));
    committed = keep;
}

static void* pix__scoped_alloc(size_t bytes, unsigned flags) {
    pix__scope_slot* s = pix__my_scope();
    if (!s) { pix_log("pix: no scope stack free (PIX_SCOPE_THREADS = %d)", PIX_SCOPE_THREADS); return 0; }
    if (s->depth == 0) { pix_log("pix: scoped allocation with no pix_scope_begin on this thread"); return 0; }
    size_t at = (s->used + 15) & ~(size_t)15;
    size_t end = at + sizeof(pix__scope_header) + bytes;
    char* base = pix__slot_base(pix__my_slot);
    if (!pix__commit_prefix(base, s->committed, end, PIX_SCOPE_RESERVE, &pix__mem_scoped_committed)) {
        pix_log("pix: scope stack full (%zu bytes asked, PIX_SCOPE_RESERVE = %zu)", bytes, (size_t)PIX_SCOPE_RESERVE);
        return 0;
    }
    pix__scope_header* h = (pix__scope_header*)(base + at);
    h->size = bytes;
    s->used = end;
    void* p = h + 1;
    // pages fresh from the OS are zero, but a reused stack holds old data
    if (flags & PIX_MEM_ZERO) memset(p, 0, bytes);
    return p;
}

bool pix_mem_is_scoped(const void* p) {
    const char* base = pix__scope_base;
    return base && (const char*)p >= base && (const char*)p < base + PIX_SCOPE_RESERVE * PIX_SCOPE_THREADS;
}

pix_mem_scope pix_scope_begin(void) {
    pix_mem_scope sc = { 0 };
    pix__scope_slot* s = pix__my_scope();
    if (!s) return sc;
    s->depth++;
    sc.mark = s->used;
    return sc;
}

void pix_scope_end(pix_mem_scope sc) {
    if (pix__my_slot < 0) return;
    pix__scope_slot& s = pix__scope_slots[pix__my_slot];
    if (s.depth <= 0) return;
    s.depth--;
    s.used = sc.mark;
    if (s.depth == 0 && s.committed > PIX_SCOPE_KEEP)
        pix__decommit_above(pix__slot_base(pix__my_slot), s.committed, PIX_SCOPE_KEEP, &pix__mem_scoped_committed);
}

void pix_mem_thread_exit(void) {
    if (pix__my_slot < 0) return;
    pix__scope_slot& s = pix__scope_slots[pix__my_slot];
    pix__decommit_above(pix__slot_base(pix__my_slot), s.committed, 0, &pix__mem_scoped_committed);
    s.used = 0;
    s.depth = 0;
    InterlockedDecrement(&pix__scope_threads);
    InterlockedExchange(&s.taken, 0);
    pix__my_slot = -1;
}

// ---- global heap ----

void* pix_mem_alloc(size_t bytes, unsigned flags) {
    if (flags & PIX_MEM_SCOPED) return pix__scoped_alloc(bytes, flags);
    if (!bytes) bytes = 1;
    void* p = (flags & PIX_MEM_ZERO) ? PIX_CALLOC(1, bytes) : PIX_MALLOC(bytes);
    if (p) pix__mem_count_global((long long)PIX_MSIZE(p), 1);
    return p;
}

void* pix_mem_realloc(void* p, size_t bytes, unsigned flags) {
    if (!p) return pix_mem_alloc(bytes, flags);
    if (pix_mem_is_scoped(p)) {
        pix__scope_header* h = (pix__scope_header*)p - 1;
        pix__scope_slot* s = pix__my_slot >= 0 ? &pix__scope_slots[pix__my_slot] : 0;
        char* base = s ? pix__slot_base(pix__my_slot) : 0;
        // the newest block in this thread's stack: move its end, no copy
        if (s && (char*)p + h->size == base + s->used) {
            size_t end = (size_t)((char*)p - base) + bytes;
            if (!pix__commit_prefix(base, s->committed, end, PIX_SCOPE_RESERVE, &pix__mem_scoped_committed))
                return 0;
            h->size = bytes;
            s->used = end;
            return p;
        }
        void* q = pix__scoped_alloc(bytes, PIX_MEM_SCOPED);
        if (q) memcpy(q, p, h->size < bytes ? h->size : bytes);
        return q;
    }
    if (!bytes) bytes = 1;
    long long before = (long long)PIX_MSIZE(p);
    void* q = PIX_REALLOC(p, bytes);
    if (q) pix__mem_count_global((long long)PIX_MSIZE(q) - before, 0);
    return q;
}

void pix_mem_free(void* p) {
    if (!p || pix_mem_is_scoped(p)) return;
    pix__mem_count_global(-(long long)PIX_MSIZE(p), -1);
    PIX_FREE(p);
}

// ---- arenas ----

mem_arena pix_arena_create(size_t reserve) {
    mem_arena a = {};
    reserve = (reserve + 0xFFFF) & ~(size_t)0xFFFF;          // allocation granularity
    a.data = (char*)VirtualAlloc(0, reserve, MEM_RESERVE, PAGE_READWRITE);
    if (!a.data) return a;
    a.capacity = reserve;
    a.virt = true;
    return a;
}

void pix_arena_destroy(mem_arena& a) {
    if (a.virt && a.data) {
        InterlockedExchangeAdd64(&pix__mem_arena_committed, -(long long)a.committed);
        VirtualFree(a.data, 0, MEM_RELEASE);
    }
    a = mem_arena();
}

bool pix_arena_commit(mem_arena& a, size_t end) {
    if (!a.virt) return end <= a.capacity;
    return pix__commit_prefix(a.data, a.committed, end, a.capacity, &pix__mem_arena_committed);
}

void pix_arena_trim(mem_arena& a) {
    if (!a.virt) return;
    pix__decommit_above(a.data, a.committed, a.size, &pix__mem_arena_committed);
}

// ---- statistics ----

pix_mem_stats pix_mem_get_stats(void) {
    pix_mem_stats st;
    st.global_bytes     = pix__mem_global_bytes;
    st.global_peak      = pix__mem_global_peak;
    st.global_blocks    = pix__mem_global_blocks;
    st.scoped_committed = pix__mem_scoped_committed;
    st.arena_committed  = pix__mem_arena_committed;
    st.scoped_threads   = (int)pix__scope_threads;
    return st;
}

void pix_mem_log_stats(const char* label) {
    pix_mem_stats st = pix_mem_get_stats();
    pix_log("%s: global %.2f MB in %lld blocks (peak %.2f MB), arenas %.2f MB, scoped %.2f MB on %d threads",
            label ? label : "pix memory", st.global_bytes / 1048576.0, st.global_blocks,
            st.global_peak / 1048576.0, st.arena_committed / 1048576.0,
            st.scoped_committed / 1048576.0, st.scoped_threads);
}

#endif // PIX_MEMORY_IMPL
