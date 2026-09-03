#pragma once
#include <windows.h>
#include <mmsystem.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#pragma comment(lib, "winmm.lib")

// Lowest layer of the audio stack: opens the default output device and keeps a
// ring of PCM blocks in flight, asking a callback to fill each one as it drains.
// Nothing above this file touches the OS.
//
// Built on waveOut, which needs no COM and ships with windows.h. The callback
// runs on a dedicated thread, so whatever it touches must be synchronized.

#define PIX_AUDIO_RATE     44100
#define PIX_AUDIO_CHANNELS 2
#define PIX_AUDIO_BLOCKS   4     // blocks queued on the device
#define PIX_AUDIO_FRAMES   1024  // frames per block (4 x 1024 @ 44.1k ~ 93ms of slack)

// fills `frame_count` interleaved stereo frames; called from the audio thread
typedef void (*pix_audio_fill)(int16_t* frames, size_t frame_count, void* user);

struct pix_audio_device {
    HWAVEOUT handle;
    HANDLE   thread;
    HANDLE   block_done;      // signalled by the driver as blocks finish
    WAVEHDR  headers[PIX_AUDIO_BLOCKS];
    int16_t* blocks;          // PIX_AUDIO_BLOCKS * PIX_AUDIO_FRAMES * PIX_AUDIO_CHANNELS
    pix_audio_fill fill;
    void*    user;
    volatile LONG running;
};

// `dev` is handed to the audio thread by address, so it must outlive the device
static bool pix_open_audio(pix_audio_device& dev, pix_audio_fill fill, void* user);
static void pix_close_audio(pix_audio_device& dev);

// ---------------- implementation ----------------

#define PIX_AUDIO_BLOCK_SAMPLES (PIX_AUDIO_FRAMES * PIX_AUDIO_CHANNELS)

static void pix_audio__fill_block(pix_audio_device* dev, int block) {
    int16_t* dst = dev->blocks + (size_t)block * PIX_AUDIO_BLOCK_SAMPLES;
    memset(dst, 0, PIX_AUDIO_BLOCK_SAMPLES * sizeof(int16_t));
    if (dev->fill) dev->fill(dst, PIX_AUDIO_FRAMES, dev->user);
}

static DWORD WINAPI pix_audio__thread(LPVOID param) {
    pix_audio_device* dev = (pix_audio_device*)param;
    while (InterlockedCompareExchange(&dev->running, 1, 1) != 0) {
        bool queued = false;
        for (int i = 0; i < PIX_AUDIO_BLOCKS; i++) {
            WAVEHDR* h = &dev->headers[i];
            if (!(h->dwFlags & WHDR_DONE)) continue;
            h->dwFlags &= ~WHDR_DONE;
            pix_audio__fill_block(dev, i);
            waveOutWrite(dev->handle, h, sizeof(WAVEHDR));
            queued = true;
        }
        // nothing drained yet: sleep until the driver retires a block
        if (!queued) WaitForSingleObject(dev->block_done, 100);
    }
    return 0;
}

static bool pix_open_audio(pix_audio_device& dev, pix_audio_fill fill, void* user) {
    dev = pix_audio_device();
    dev.fill = fill;
    dev.user = user;

    WAVEFORMATEX fmt = {};
    fmt.wFormatTag      = WAVE_FORMAT_PCM;
    fmt.nChannels       = PIX_AUDIO_CHANNELS;
    fmt.nSamplesPerSec  = PIX_AUDIO_RATE;
    fmt.wBitsPerSample  = 16;
    fmt.nBlockAlign     = (WORD)(fmt.nChannels * fmt.wBitsPerSample / 8);
    fmt.nAvgBytesPerSec = fmt.nSamplesPerSec * fmt.nBlockAlign;

    dev.block_done = CreateEventA(0, FALSE, FALSE, 0);
    if (!dev.block_done) return false;

    if (waveOutOpen(&dev.handle, WAVE_MAPPER, &fmt,
                    (DWORD_PTR)dev.block_done, 0, CALLBACK_EVENT) != MMSYSERR_NOERROR) {
        CloseHandle(dev.block_done);
        dev = pix_audio_device();
        return false;
    }

    dev.blocks = (int16_t*)calloc(PIX_AUDIO_BLOCKS * PIX_AUDIO_BLOCK_SAMPLES, sizeof(int16_t));
    if (!dev.blocks) { pix_close_audio(dev); return false; }

    // prime the queue before the thread starts, so playback begins gapless
    for (int i = 0; i < PIX_AUDIO_BLOCKS; i++) {
        WAVEHDR* h = &dev.headers[i];
        h->lpData = (LPSTR)(dev.blocks + (size_t)i * PIX_AUDIO_BLOCK_SAMPLES);
        h->dwBufferLength = PIX_AUDIO_BLOCK_SAMPLES * sizeof(int16_t);
        if (waveOutPrepareHeader(dev.handle, h, sizeof(WAVEHDR)) != MMSYSERR_NOERROR) {
            pix_close_audio(dev);
            return false;
        }
        pix_audio__fill_block(&dev, i);
        waveOutWrite(dev.handle, h, sizeof(WAVEHDR));
    }

    dev.running = 1;
    dev.thread = CreateThread(0, 0, pix_audio__thread, &dev, 0, 0);
    if (!dev.thread) { pix_close_audio(dev); return false; }
    SetThreadPriority(dev.thread, THREAD_PRIORITY_ABOVE_NORMAL);
    return true;
}

static void pix_close_audio(pix_audio_device& dev) {
    if (dev.thread) {
        InterlockedExchange(&dev.running, 0);
        SetEvent(dev.block_done);
        WaitForSingleObject(dev.thread, 2000);
        CloseHandle(dev.thread);
        dev.thread = 0;
    }
    if (dev.handle) {
        waveOutReset(dev.handle);
        for (int i = 0; i < PIX_AUDIO_BLOCKS; i++)
            if (dev.headers[i].dwFlags & WHDR_PREPARED)
                waveOutUnprepareHeader(dev.handle, &dev.headers[i], sizeof(WAVEHDR));
        waveOutClose(dev.handle);
        dev.handle = 0;
    }
    if (dev.block_done) { CloseHandle(dev.block_done); dev.block_done = 0; }
    free(dev.blocks);
    dev = pix_audio_device();
}
