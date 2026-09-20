#pragma once
#include <stdio.h>
#include <string.h>
#include <windows.h>

// ---- the frame breakdown ----
//
// Where the milliseconds went, per section, averaged over a second. The whole
// reason it exists is that the answer is never the one anybody guesses: "the
// AI is slow" and "it is the shadow pass" are both cheap to say and cost an
// afternoon each to act on when they are wrong.
//
// It is off unless PIX_PROF is set in the environment, and when it is off the
// scopes below still call QueryPerformanceCounter - which is a few tens of
// nanoseconds against sections measured in whole milliseconds, and cheaper
// than the branch mispredicts that guarding every one of them would cost.
//
// Sections are declared by the caller as a plain enum ending in a count, so
// this file has no idea what a "sim" or a "cull" is - it counts labelled
// intervals, and one game's labels are not an engine concern.

#define PIX_PROF_MAX_SECTIONS 16

struct pix_profiler {
    const char* names[PIX_PROF_MAX_SECTIONS];
    double      accum[PIX_PROF_MAX_SECTIONS];   // seconds this window
    int         sections;
    bool        enabled;

    double      inv_frequency;
    long long   window_start;
    double      window;                         // seconds of accumulation
    int         frames;
};

static void pix_profiler_init(pix_profiler& p, const char* const* names, int count) {
    memset(&p, 0, sizeof(p));
    p.sections = count < PIX_PROF_MAX_SECTIONS ? count : PIX_PROF_MAX_SECTIONS;
    for (int i = 0; i < p.sections; i++) p.names[i] = names[i];
    p.enabled = getenv("PIX_PROF") != 0;

    LARGE_INTEGER f, now;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&now);
    p.inv_frequency = 1.0 / (double)f.QuadPart;
    p.window_start = now.QuadPart;
}

static long long pix_profile_now() {
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t.QuadPart;
}

static void pix_profile_add(pix_profiler& p, int section, long long begin) {
    if ((unsigned)section >= (unsigned)p.sections) return;
    p.accum[section] += (double)(pix_profile_now() - begin) * p.inv_frequency;
}

// Called once per frame, after the last section. Prints and resets on the
// second, so a run's output is one line a second rather than one a frame -
// per-frame numbers of a section that costs 0.4 ms are noise, and a second of
// them is a measurement.
// Returns true on the frame that closes an accumulation window and prints, so
// a caller with numbers of its own to report can put them next to these rather
// than on some unrelated clock of its own.
static bool pix_profile_frame(pix_profiler& p) {
    p.frames++;
    long long now = pix_profile_now();
    double elapsed = (double)(now - p.window_start) * p.inv_frequency;
    if (elapsed < 1.0) return false;

    if (p.enabled && p.frames > 0) {
        char line[512];
        int n = snprintf(line, sizeof(line), "prof  %5.1f fps  %5.2f ms/frame  |",
                         (double)p.frames / elapsed, elapsed * 1000.0 / (double)p.frames);
        for (int i = 0; i < p.sections && n < (int)sizeof(line); i++)
            n += snprintf(line + n, sizeof(line) - (size_t)n, "  %s %5.2f",
                          p.names[i], p.accum[i] * 1000.0 / (double)p.frames);
        printf("%s\n", line);
        fflush(stdout);
    }

    for (int i = 0; i < p.sections; i++) p.accum[i] = 0.0;
    p.frames = 0;
    p.window_start = now;
    return p.enabled;
}

// One section, timed by scope, so an early `return` or a `break` out of the
// middle of a frame cannot leave a section running.
struct pix_profile_scope {
    pix_profiler* p;
    int           section;
    long long     begin;
    pix_profile_scope(pix_profiler& prof, int s)
        : p(&prof), section(s), begin(pix_profile_now()) {}
    ~pix_profile_scope() { pix_profile_add(*p, section, begin); }
};

#define PIX_PROFILE(prof, section) pix_profile_scope pix__scope_##section(prof, section)
