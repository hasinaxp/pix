#pragma once
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "dtype.hpp"
#include "platform_audio.hpp"
#include "../loader/asset_types.hpp"

// Game-facing audio: load or generate a sound, then fire and forget.
//
//   pix_audio audio;
//   pix_create_audio(audio);
//   idx wav  = load_sound_wav_file(loader, "assets/step.wav");   // loader/
//   idx step = load_sound(audio, loader.sound_files[wav]);        // core/
//   play_sound(audio, step, 0.8f);          // volume
//   idx music = play_sound(audio, track, 0.5f, 1.0f, true);   // looping handle
//   stop_sound(audio, music);
//
// play_sound hands back a voice handle that carries a generation counter, so a
// handle kept past the end of a one-shot is simply ignored rather than
// stopping whatever reused that slot.

#define MAX_SOUNDS 64
#define MAX_VOICES 32
#define SOUND_INVALID ((idx)-1)

struct sound {
    int16_t* samples;      // interleaved
    size_t   frame_count;
    int      channels;     // 1 or 2
    int      rate;         // resampled to the device rate at playback
};

struct sound_voice {
    idx      sound;
    double   cursor;       // fractional source frame
    double   step;         // source frames advanced per output frame
    float    volume;
    float    pan;          // -1 hard left .. +1 hard right
    bool     loop;
    bool     active;
    uint16_t generation;
};

struct pix_audio {
    pix_audio_device device;
    mem_arena arena;

    sound  sounds[MAX_SOUNDS];
    size_t sound_count;
    sound_voice voices[MAX_VOICES];
    float  master_volume;
    bool   device_ok;      // false when no output device could be opened

    CRITICAL_SECTION lock; // voices are touched by the game and audio threads
    float  scratch[PIX_AUDIO_FRAMES * PIX_AUDIO_CHANNELS];
};

static bool pix_create_audio(pix_audio& audio, size_t capacity = 16 * MB);
static void pix_destroy_audio(pix_audio& audio);

// takes ownership of a copy, mirroring how load_mesh copies onto the GPU: the
// decoded file in the data loader's arena can be dropped afterwards
static idx load_sound(pix_audio& audio, const sound_file_data& data);
// takes a copy of `samples`, so callers can build sounds on the stack
static idx create_sound(pix_audio& audio, const int16_t* samples, size_t frames, int channels, int rate);

static idx  play_sound(pix_audio& audio, idx sound_id, float volume = 1.0f,
                       float pitch = 1.0f, bool loop = false, float pan = 0.0f);
static void stop_sound(pix_audio& audio, idx voice);
static void stop_all_sounds(pix_audio& audio);
static bool is_sound_playing(pix_audio& audio, idx voice);
static void set_sound_volume(pix_audio& audio, idx voice, float volume);
static void set_sound_pan(pix_audio& audio, idx voice, float pan);
static void set_master_volume(pix_audio& audio, float volume);
static size_t active_voice_count(pix_audio& audio);

// mixes the active voices; the device calls this, and tests can drive it directly
static void pix_audio_mix(pix_audio& audio, int16_t* out, size_t frames);

// ---------------- implementation ----------------

static idx snd__handle(uint16_t generation, uint16_t slot) {
    return ((idx)generation << 16) | (idx)slot;
}

static sound_voice* snd__voice(pix_audio& audio, idx handle) {
    if (handle == SOUND_INVALID) return 0;
    uint16_t slot = (uint16_t)(handle & 0xFFFF);
    uint16_t gen  = (uint16_t)(handle >> 16);
    if (slot >= MAX_VOICES) return 0;
    sound_voice* v = &audio.voices[slot];
    return (v->generation == gen) ? v : 0;   // stale handle -> nothing
}

static void snd__device_fill(int16_t* frames, size_t count, void* user) {
    pix_audio_mix(*(pix_audio*)user, frames, count);
}

static bool pix_create_audio(pix_audio& audio, size_t capacity) {
    audio = pix_audio();
    audio.master_volume = 1.0f;
    audio.arena.capacity = capacity;
    audio.arena.data = (char*)malloc(capacity);
    if (!audio.arena.data) return false;
    InitializeCriticalSection(&audio.lock);

    // a missing or busy device is not fatal: sounds still load and play into
    // the void, so callers do not need an audio-less code path
    audio.device_ok = pix_open_audio(audio.device, snd__device_fill, &audio);
    return audio.device_ok;
}

static void pix_destroy_audio(pix_audio& audio) {
    pix_close_audio(audio.device);       // stop the thread before freeing anything it reads
    DeleteCriticalSection(&audio.lock);
    free(audio.arena.data);
    audio.arena = mem_arena();
    audio.device_ok = false;
}

static idx create_sound(pix_audio& audio, const int16_t* samples, size_t frames, int channels, int rate) {
    if (audio.sound_count >= MAX_SOUNDS || !frames || channels < 1 || channels > 2) return SOUND_INVALID;
    int16_t* copy = allocate<int16_t>(audio.arena, frames * (size_t)channels);
    if (!copy) return SOUND_INVALID;
    memcpy(copy, samples, frames * (size_t)channels * sizeof(int16_t));

    idx id = (idx)audio.sound_count++;
    sound& s = audio.sounds[id];
    s.samples     = copy;
    s.frame_count = frames;
    s.channels    = channels;
    s.rate        = rate > 0 ? rate : PIX_AUDIO_RATE;
    return id;
}

// ---- playback ----

static idx load_sound(pix_audio& audio, const sound_file_data& data) {
    return create_sound(audio, data.samples, data.frame_count, data.channels, data.rate);
}

static idx play_sound(pix_audio& audio, idx sound_id, float volume, float pitch, bool loop, float pan) {
    if (sound_id >= audio.sound_count) return SOUND_INVALID;
    const sound& s = audio.sounds[sound_id];
    if (!s.frame_count) return SOUND_INVALID;

    EnterCriticalSection(&audio.lock);
    int slot = -1;
    for (int i = 0; i < MAX_VOICES; i++) if (!audio.voices[i].active) { slot = i; break; }
    if (slot < 0) { LeaveCriticalSection(&audio.lock); return SOUND_INVALID; } // all voices busy

    sound_voice& v = audio.voices[slot];
    v.generation++;
    v.sound   = sound_id;
    v.cursor  = 0.0;
    v.step    = ((double)s.rate / (double)PIX_AUDIO_RATE) * (double)(pitch > 0.0f ? pitch : 1.0f);
    v.volume  = volume < 0.0f ? 0.0f : volume;
    v.pan     = pan < -1.0f ? -1.0f : (pan > 1.0f ? 1.0f : pan);
    v.loop    = loop;
    v.active  = true;
    idx handle = snd__handle(v.generation, (uint16_t)slot);
    LeaveCriticalSection(&audio.lock);
    return handle;
}

static void stop_sound(pix_audio& audio, idx voice) {
    EnterCriticalSection(&audio.lock);
    sound_voice* v = snd__voice(audio, voice);
    if (v) v->active = false;
    LeaveCriticalSection(&audio.lock);
}

static void stop_all_sounds(pix_audio& audio) {
    EnterCriticalSection(&audio.lock);
    for (int i = 0; i < MAX_VOICES; i++) audio.voices[i].active = false;
    LeaveCriticalSection(&audio.lock);
}

static bool is_sound_playing(pix_audio& audio, idx voice) {
    EnterCriticalSection(&audio.lock);
    sound_voice* v = snd__voice(audio, voice);
    bool playing = v && v->active;
    LeaveCriticalSection(&audio.lock);
    return playing;
}

static void set_sound_volume(pix_audio& audio, idx voice, float volume) {
    EnterCriticalSection(&audio.lock);
    sound_voice* v = snd__voice(audio, voice);
    if (v) v->volume = volume < 0.0f ? 0.0f : volume;
    LeaveCriticalSection(&audio.lock);
}

static void set_sound_pan(pix_audio& audio, idx voice, float pan) {
    EnterCriticalSection(&audio.lock);
    sound_voice* v = snd__voice(audio, voice);
    if (v) v->pan = pan < -1.0f ? -1.0f : (pan > 1.0f ? 1.0f : pan);
    LeaveCriticalSection(&audio.lock);
}

static void set_master_volume(pix_audio& audio, float volume) {
    audio.master_volume = volume < 0.0f ? 0.0f : volume;
}

static size_t active_voice_count(pix_audio& audio) {
    size_t n = 0;
    EnterCriticalSection(&audio.lock);
    for (int i = 0; i < MAX_VOICES; i++) if (audio.voices[i].active) n++;
    LeaveCriticalSection(&audio.lock);
    return n;
}

// ---- mixing ----

// Gentle limiter instead of a hard clamp: below the knee samples pass through
// untouched, above it they bend smoothly toward full scale. Stacked voices then
// lose a little headroom rather than turning into harsh clipping distortion.
static int16_t snd__soft_clip(float v) {
    const float knee = 0.80f * 32767.0f;
    float a = v < 0.0f ? -v : v;
    if (a > knee) {
        float over = (a - knee) / (32767.0f - knee);
        a = knee + (32767.0f - knee) * (1.0f - expf(-over));   // C1 continuous at the knee
        v = v < 0.0f ? -a : a;
    }
    if (v > 32767.0f) v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
}

// one source frame, linearly interpolated, as left/right
static void snd__fetch(const sound& s, double cursor, bool loop, float* l, float* r) {
    size_t i0 = (size_t)cursor;
    if (i0 >= s.frame_count) { *l = *r = 0.0f; return; }
    size_t i1 = i0 + 1;
    if (i1 >= s.frame_count) i1 = loop ? 0 : i0;
    float f = (float)(cursor - (double)i0);

    if (s.channels == 1) {
        float a = (float)s.samples[i0], b = (float)s.samples[i1];
        *l = *r = a + (b - a) * f;
    } else {
        float al = (float)s.samples[i0 * 2],     bl = (float)s.samples[i1 * 2];
        float ar = (float)s.samples[i0 * 2 + 1], br = (float)s.samples[i1 * 2 + 1];
        *l = al + (bl - al) * f;
        *r = ar + (br - ar) * f;
    }
}

static void snd__mix_chunk(pix_audio& audio, int16_t* out, size_t frames) {
    float* acc = audio.scratch;
    memset(acc, 0, frames * PIX_AUDIO_CHANNELS * sizeof(float));

    EnterCriticalSection(&audio.lock);
    for (int i = 0; i < MAX_VOICES; i++) {
        sound_voice& v = audio.voices[i];
        if (!v.active || v.sound >= audio.sound_count) continue;
        const sound& s = audio.sounds[v.sound];

        // equal-ish power is overkill here; a simple linear taper reads fine
        float lg = v.volume * (v.pan > 0.0f ? 1.0f - v.pan : 1.0f);
        float rg = v.volume * (v.pan < 0.0f ? 1.0f + v.pan : 1.0f);

        for (size_t f = 0; f < frames; f++) {
            if (v.cursor >= (double)s.frame_count) {
                if (!v.loop) { v.active = false; break; }
                v.cursor -= (double)s.frame_count;
            }
            float l, r;
            snd__fetch(s, v.cursor, v.loop, &l, &r);
            acc[f * 2 + 0] += l * lg;
            acc[f * 2 + 1] += r * rg;
            v.cursor += v.step;
        }
    }
    float master = audio.master_volume;
    LeaveCriticalSection(&audio.lock);

    for (size_t i = 0; i < frames * PIX_AUDIO_CHANNELS; i++)
        out[i] = snd__soft_clip(acc[i] * master);
}

static void pix_audio_mix(pix_audio& audio, int16_t* out, size_t frames) {
    while (frames) {
        size_t n = frames < PIX_AUDIO_FRAMES ? frames : PIX_AUDIO_FRAMES;
        snd__mix_chunk(audio, out, n);
        out += n * PIX_AUDIO_CHANNELS;
        frames -= n;
    }
}
