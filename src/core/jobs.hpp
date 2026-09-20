#pragma once
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "dtype.hpp"

// The only piece of threading in the project, and deliberately the smallest
// one that does the job.
//
// Two things are wanted and nothing else:
//
//   pix_parallel_for   split a flat range of work across every core and wait
//                      for it. Crowd steering and the physics solver's static
//                      pass are both "the same function over N independent
//                      items", which is the one shape that parallelises
//                      without a single lock in the work itself.
//
//   pix_task           one long-lived side thread that runs a function while
//                      the caller carries on. Used for the skinning: sampling
//                      a few dozen skeletons is pure maths on data the
//                      physics step never touches, so it runs *alongside*
//                      the solver rather than before or after it.
//
// There is no work stealing, no futures, no allocation and no job graph. The
// frame is a fixed sequence of phases and every phase knows what it needs, so
// a generation counter and an atomic cursor cover all of it.
//
// The rules the callers have to keep, since nothing here can enforce them:
//   * a parallel body writes only to item i's own storage
//   * anything that adds or removes an entity (a physics body, an actor)
//     stays on the main thread - entity pools are not thread safe and making
//     them so would cost more than the work being split
//   * anything drawing from a shared rng takes the per-worker one instead,
//     which is why every job body is handed its worker index

#define PIX_MAX_WORKERS 7   // plus the calling thread, so up to 8 cores busy

// `begin`..`end` is a half open slice of the dispatched range; `worker` is 0
// for the thread that called pix_parallel_for and 1..n for the pool.
typedef void (*pix_job_fn)(void* user, size_t begin, size_t end, int worker);

struct pix_jobs {
    std::thread threads[PIX_MAX_WORKERS];
    int         worker_count;

    std::mutex              lock;
    std::condition_variable go, done;
    // What is being run right now. Written under `lock` before `generation`
    // is bumped, so a worker that sees a new generation sees all of it.
    pix_job_fn  fn;
    void*       user;
    size_t      total;
    size_t      grain;
    uint64_t    generation;
    bool        quit;

    std::atomic<size_t> cursor;    // next unclaimed chunk
    std::atomic<int>    busy;      // workers still inside the body
};

// One function on one thread, started when asked and waited on later. Kept
// apart from the pool above because it overlaps with the pool rather than
// feeding off it: the whole point is that it is still running while the main
// thread drives a pix_parallel_for of its own.
struct pix_task {
    std::thread             thread;
    std::mutex              lock;
    std::condition_variable go, done;
    void  (*fn)(void*);
    void*   user;
    bool    running;
    bool    quit;
    bool    started;
};

static void pix_jobs_start(pix_jobs& j, int workers = -1);
static void pix_jobs_stop(pix_jobs& j);
// Runs `fn` over [0, count) and returns once every item is done. `grain` is
// the smallest slice a worker will claim; below it the range is not worth
// splitting and the calling thread just runs the lot.
static void pix_parallel_for(pix_jobs& j, size_t count, pix_job_fn fn, void* user,
                             size_t grain = 16);

static void pix_task_start(pix_task& t);
static void pix_task_stop(pix_task& t);
static void pix_task_run(pix_task& t, void (*fn)(void*), void* user);
static void pix_task_wait(pix_task& t);

// ---------------- implementation ----------------

static void pix__jobs_run_chunks(pix_jobs& j, pix_job_fn fn, void* user,
                                 size_t total, size_t grain, int worker) {
    for (;;) {
        size_t start = j.cursor.fetch_add(grain, std::memory_order_relaxed);
        if (start >= total) break;
        size_t end = start + grain;
        if (end > total) end = total;
        fn(user, start, end, worker);
    }
}

static void pix__worker_main(pix_jobs* j, int worker) {
    uint64_t seen = 0;
    for (;;) {
        pix_job_fn fn;
        void* user;
        size_t total, grain;
        {
            std::unique_lock<std::mutex> guard(j->lock);
            j->go.wait(guard, [&] { return j->quit || j->generation != seen; });
            if (j->quit) return;
            seen = j->generation;
            fn = j->fn; user = j->user; total = j->total; grain = j->grain;
        }

        pix__jobs_run_chunks(*j, fn, user, total, grain, worker);

        // The count going to zero is what the dispatcher is waiting on, and
        // the notify has to happen with the lock held or it can slip between
        // the dispatcher's predicate check and its wait.
        {
            std::lock_guard<std::mutex> guard(j->lock);
            if (j->busy.fetch_sub(1, std::memory_order_acq_rel) == 1) j->done.notify_all();
        }
    }
}

static void pix_jobs_start(pix_jobs& j, int workers) {
    j.worker_count = 0;
    j.fn = 0; j.user = 0; j.total = 0; j.grain = 1;
    j.generation = 0;
    j.quit = false;
    j.cursor.store(0);
    j.busy.store(0);

    if (workers < 0) {
        unsigned hw = std::thread::hardware_concurrency();
        workers = hw > 1 ? (int)hw - 1 : 0;      // the calling thread is one of them
    }
    if (workers > PIX_MAX_WORKERS) workers = PIX_MAX_WORKERS;
    if (workers < 0) workers = 0;

    for (int i = 0; i < workers; i++)
        j.threads[j.worker_count++] = std::thread(pix__worker_main, &j, i + 1);
}

static void pix_jobs_stop(pix_jobs& j) {
    {
        std::lock_guard<std::mutex> guard(j.lock);
        j.quit = true;
        j.generation++;
    }
    j.go.notify_all();
    for (int i = 0; i < j.worker_count; i++)
        if (j.threads[i].joinable()) j.threads[i].join();
    j.worker_count = 0;
}

static void pix_parallel_for(pix_jobs& j, size_t count, pix_job_fn fn, void* user, size_t grain) {
    if (!count) return;
    if (grain < 1) grain = 1;

    // Too small to be worth a hand-off, or nowhere to hand it to.
    if (j.worker_count == 0 || count <= grain) {
        fn(user, 0, count, 0);
        return;
    }

    // Aim for a few chunks per thread: enough that an uneven item cost
    // evens out, few enough that the atomic is not the bottleneck.
    size_t want = count / (size_t)(j.worker_count + 1) / 4;
    if (want > grain) grain = want;

    {
        std::lock_guard<std::mutex> guard(j.lock);
        j.fn = fn; j.user = user; j.total = count; j.grain = grain;
        j.cursor.store(0, std::memory_order_relaxed);
        j.busy.store(j.worker_count + 1, std::memory_order_relaxed);
        j.generation++;
    }
    j.go.notify_all();

    pix__jobs_run_chunks(j, fn, user, count, grain, 0);

    {
        std::unique_lock<std::mutex> guard(j.lock);
        if (j.busy.fetch_sub(1, std::memory_order_acq_rel) != 1)
            j.done.wait(guard, [&] { return j.busy.load(std::memory_order_acquire) == 0; });
    }
}

static void pix__task_main(pix_task* t) {
    for (;;) {
        void (*fn)(void*);
        void* user;
        {
            std::unique_lock<std::mutex> guard(t->lock);
            t->go.wait(guard, [&] { return t->quit || t->running; });
            if (t->quit) return;
            fn = t->fn; user = t->user;
        }
        if (fn) fn(user);
        {
            std::lock_guard<std::mutex> guard(t->lock);
            t->running = false;
        }
        t->done.notify_all();
    }
}

static void pix_task_start(pix_task& t) {
    t.fn = 0; t.user = 0; t.running = false; t.quit = false;
    t.thread = std::thread(pix__task_main, &t);
    t.started = true;
}

static void pix_task_stop(pix_task& t) {
    if (!t.started) return;
    pix_task_wait(t);
    {
        std::lock_guard<std::mutex> guard(t.lock);
        t.quit = true;
    }
    t.go.notify_all();
    if (t.thread.joinable()) t.thread.join();
    t.started = false;
}

static void pix_task_run(pix_task& t, void (*fn)(void*), void* user) {
    if (!t.started) { fn(user); return; }     // no thread: do it here, same result
    pix_task_wait(t);                         // one at a time, by design
    {
        std::lock_guard<std::mutex> guard(t.lock);
        t.fn = fn; t.user = user; t.running = true;
    }
    t.go.notify_one();
}

static void pix_task_wait(pix_task& t) {
    if (!t.started) return;
    std::unique_lock<std::mutex> guard(t.lock);
    t.done.wait(guard, [&] { return !t.running; });
}
