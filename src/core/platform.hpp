// pix - window, OpenGL context, keyboard / mouse / gamepad input and time.
#ifndef PIX_PLATFORM_HPP
#define PIX_PLATFORM_HPP
#include "dtype.hpp"
#include <windows.h>
#include <GL/gl.h>

#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "user32.lib")

struct pix_key_state {
    bool pressed;   // went down this frame
    bool released;  // went up this frame
    bool held;      // currently down
    char code;
};

// ---- key codes ----
//
// keystates[] is indexed by the Windows virtual-key code, which for letters
// and digits is simply their ASCII capital: keystates['W'], keystates['1'].
// The handful of keys whose virtual codes would read as punctuation are moved
// up out of the way, and the mouse buttons live at the very top.
#define MOUSE_BUTTON_LEFT  255
#define MOUSE_BUTTON_RIGHT 254
#define MOUSE_BUTTON_MID   253

#define KEY_UP    201
#define KEY_DOWN  202
#define KEY_LEFT  203
#define KEY_RIGHT 204
#define KEY_ESC   205
#define KEY_TAB   206
// VK_OEM_4 / VK_OEM_6 on a US layout; remapped out of the OEM range so a key
// code never collides with an ascii letter
#define KEY_LEFT_BRACKET  207
#define KEY_RIGHT_BRACKET 208
// function keys already land on their own VK codes (see pix__map_vk's
// passthrough); these just give the commonly bound ones names
#define KEY_F1 0x70
#define KEY_F2 0x71
#define KEY_F3 0x72
#define KEY_F4 0x73
#define KEY_F5 0x74
#define KEY_F6 0x75
#define KEY_F7 0x76
#define KEY_F8 0x77
#define KEY_F9 0x78
#define KEY_F10 0x79
#define KEY_F11 0x7A
#define KEY_F12 0x7B
#define KEY_RETURN 0x0D
#define KEY_ENTER  KEY_RETURN
// these keep their windows virtual key codes, which pix__map_vk passes through
#define KEY_BACKSPACE 0x08
#define KEY_SHIFT     16
#define KEY_CTRL      17
#define KEY_ALT       18
#define KEY_PAUSE     0x13
#define KEY_CAPS_LOCK 0x14
#define KEY_SPACE     32
#define KEY_PAGE_UP   0x21
#define KEY_PAGE_DOWN 0x22
#define KEY_END       0x23
#define KEY_HOME      0x24
#define KEY_INSERT    0x2D
#define KEY_DELETE    0x2E

// ---- gamepads ----
// XInput, resolved at runtime so there is no link dependency and a machine
// without it simply reports no pads. Covers Xbox style controllers, which is
// what most modern pads present themselves as.

#define PIX_MAX_GAMEPADS 4

#define PAD_A            0
#define PAD_B            1
#define PAD_X            2
#define PAD_Y            3
#define PAD_LEFT_BUMPER  4
#define PAD_RIGHT_BUMPER 5
#define PAD_BACK         6
#define PAD_START        7
#define PAD_LEFT_STICK   8   // stick pressed in
#define PAD_RIGHT_STICK  9
#define PAD_DPAD_UP      10
#define PAD_DPAD_DOWN    11
#define PAD_DPAD_LEFT    12
#define PAD_DPAD_RIGHT   13
#define PAD_BUTTON_COUNT 14

struct pix_gamepad {
    bool connected;
    pix_key_state buttons[PAD_BUTTON_COUNT];  // same pressed/released/held edges as keystates

    // sticks are -1..1 with a radial deadzone already applied; +y is up
    float left_x,  left_y;
    float right_x, right_y;
    float left_trigger, right_trigger;        // 0..1

    float rumble_low, rumble_high;            // last values handed to the motors
};

// ---- window flags ----
#define PIX_WINDOW_RESIZABLE  0x01   // user may drag the border; watch window.resized
#define PIX_WINDOW_FULLSCREEN 0x02   // borderless fullscreen on the current monitor
#define PIX_WINDOW_VSYNC      0x04   // start with vsync on (default is uncapped)
#define PIX_WINDOW_DEBUG_GL   0x08   // debug context; GL errors are printed via pix_log
#define PIX_WINDOW_HIDDEN     0x10   // create without showing (tools, tests)

#define PIX_MAX_TEXT_INPUT 32

struct pix_window {
    HWND  handle;
    HDC   dc;
    void* gl_context;               // HGLRC
    pix_key_state keystates[256];   // ascii-indexed; mouse buttons + special keys use the codes above
    int mouse_x;      int mouse_y;
    int mouse_rel_x;  int mouse_rel_y;
    int mouse_wheel;                 // notches this frame, +1 per detent forward; see WM_MOUSEWHEEL
    bool mouse_captured;            // cursor hidden and re-centred every frame
    bool should_close;

    pix_gamepad gamepads[PIX_MAX_GAMEPADS];

    // Client area in pixels. `resized` is true for exactly the frame the size
    // changed on, which is when render targets need to follow it.
    int  width;
    int  height;
    bool resized;
    bool focused;
    bool minimized;
    bool fullscreen;
    unsigned int flags;

    // Unicode codepoints typed this frame, in order - for text boxes and chat,
    // where keystates would lose the shift state, the layout and key repeat.
    unsigned int text_input[PIX_MAX_TEXT_INPUT];
    int          text_input_count;

    // private: restoring from fullscreen
    RECT windowed_rect;
    DWORD windowed_style;
    wchar_t pending_surrogate;
};

PIXDEF pix_window pix_create_window(const char* title, int width, int height, unsigned int flags = 0);
PIXDEF void pix_destroy_window(pix_window& window);
PIXDEF void pix_update_window(pix_window& window);   // swap the last frame, pump events, refresh input
PIXDEF void pix_swap_buffers(pix_window& window);

// Hides the cursor and warps it back to the middle of the client area after
// every frame, so mouse_rel keeps accumulating without the pointer ever
// reaching a screen edge - what a mouselook camera needs.
PIXDEF void pix_set_mouse_capture(pix_window& window, bool captured);
PIXDEF void pix_show_cursor(pix_window& window, bool visible);

// Buffer swap pacing. Off means the frame rate is uncapped, which is what you
// want while measuring - vsync pins a scene that would run at 150fps to a flat
// 60 and it reads exactly like a real bottleneck. On is what a shipped game
// wants. The window starts uncapped unless PIX_WINDOW_VSYNC is passed.
PIXDEF void pix_set_vsync(pix_window& window, bool enabled);

// Borderless fullscreen on whichever monitor the window is on, and back.
PIXDEF void pix_set_fullscreen(pix_window& window, bool fullscreen);
PIXDEF void pix_set_window_title(pix_window& window, const char* title);

// low = the heavy motor, high = the light one; both 0..1, held until changed
PIXDEF void pix_set_gamepad_rumble(pix_window& window, int pad, float low, float high);
PIXDEF void pix_stop_gamepad_rumble(pix_window& window);   // all pads, e.g. on shutdown

// Seconds since the first call, from the high resolution counter.
PIXDEF double pix_time_seconds(void);

// Some Win32/WGL entry points the loader in opengl_api.hpp also needs.
PIXDEF void* pix_gl_get_proc(const char* name);

#endif // PIX_PLATFORM_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_PLATFORM_IMPL)
#define PIX_PLATFORM_IMPL
#include <windowsx.h>
#include <math.h>
#include <stdio.h>

// ---- WGL extension constants ----
#define WGL_CONTEXT_MAJOR_VERSION_ARB    0x2091
#define WGL_CONTEXT_MINOR_VERSION_ARB    0x2092
#define WGL_CONTEXT_FLAGS_ARB            0x2094
#define WGL_CONTEXT_DEBUG_BIT_ARB        0x0001
#define WGL_CONTEXT_PROFILE_MASK_ARB     0x9126
#define WGL_CONTEXT_CORE_PROFILE_BIT_ARB 0x00000001
#define WGL_DRAW_TO_WINDOW_ARB           0x2001
#define WGL_SUPPORT_OPENGL_ARB           0x2010
#define WGL_DOUBLE_BUFFER_ARB            0x2011
#define WGL_PIXEL_TYPE_ARB               0x2013
#define WGL_TYPE_RGBA_ARB                0x202B
#define WGL_COLOR_BITS_ARB               0x2014
#define WGL_DEPTH_BITS_ARB               0x2022
#define WGL_STENCIL_BITS_ARB             0x2023

typedef HGLRC(WINAPI* PFNWGLCREATECONTEXTATTRIBSARBPROC)(HDC, HGLRC, const int*);
typedef BOOL (WINAPI* PFNWGLCHOOSEPIXELFORMATARBPROC)(HDC, const int*, const FLOAT*, UINT, int*, UINT*);
typedef BOOL (WINAPI* PFNWGLSWAPINTERVALEXTPROC)(int);
// resolved once the real context exists; null on a driver without the extension
static PFNWGLSWAPINTERVALEXTPROC pix__swap_interval;

static pix_window* pix__active; // valid only while pix_update_window is pumping

void* pix_gl_get_proc(const char* name) {
    void* p = (void*)wglGetProcAddress(name);
    if (p == 0 || p == (void*)1 || p == (void*)2 || p == (void*)3 || p == (void*)-1) {
        HMODULE m = GetModuleHandleA("opengl32.dll");
        p = (void*)GetProcAddress(m, name);
    }
    return p;
}

double pix_time_seconds(void) {
    static LARGE_INTEGER freq, start;
    if (!freq.QuadPart) { QueryPerformanceFrequency(&freq); QueryPerformanceCounter(&start); }
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (double)(now.QuadPart - start.QuadPart) / (double)freq.QuadPart;
}

// ---- xinput, declared here rather than pulling in <xinput.h> ----

struct pix__xi_pad {
    WORD  buttons;
    BYTE  left_trigger;
    BYTE  right_trigger;
    SHORT thumb_lx, thumb_ly, thumb_rx, thumb_ry;
};
struct pix__xi_state { DWORD packet; pix__xi_pad pad; };
struct pix__xi_rumble { WORD low, high; };

typedef DWORD (WINAPI* PFN_XInputGetState)(DWORD, pix__xi_state*);
typedef DWORD (WINAPI* PFN_XInputSetState)(DWORD, pix__xi_rumble*);

static PFN_XInputGetState pix__xi_get;
static PFN_XInputSetState pix__xi_set;
static ULONGLONG pix__pad_retry[PIX_MAX_GAMEPADS];  // next tick an empty slot may be probed

#define PIX__XI_DPAD_UP        0x0001
#define PIX__XI_DPAD_DOWN      0x0002
#define PIX__XI_DPAD_LEFT      0x0004
#define PIX__XI_DPAD_RIGHT     0x0008
#define PIX__XI_START          0x0010
#define PIX__XI_BACK           0x0020
#define PIX__XI_LEFT_THUMB     0x0040
#define PIX__XI_RIGHT_THUMB    0x0080
#define PIX__XI_LEFT_SHOULDER  0x0100
#define PIX__XI_RIGHT_SHOULDER 0x0200
#define PIX__XI_A              0x1000
#define PIX__XI_B              0x2000
#define PIX__XI_X              0x4000
#define PIX__XI_Y              0x8000

#define PIX__XI_LEFT_DEADZONE   7849.0f
#define PIX__XI_RIGHT_DEADZONE  8689.0f
#define PIX__XI_TRIGGER_FLOOR   30.0f
#define PIX__PAD_RETRY_MS       2000    // probing an empty slot is slow, so back off

static void pix__load_xinput() {
    if (pix__xi_get) return;
    // newest first; 9_1_0 is the one guaranteed present since Vista
    const char* dlls[3] = { "xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll" };
    for (int i = 0; i < 3; i++) {
        HMODULE m = LoadLibraryA(dlls[i]);
        if (!m) continue;
        pix__xi_get = (PFN_XInputGetState)GetProcAddress(m, "XInputGetState");
        pix__xi_set = (PFN_XInputSetState)GetProcAddress(m, "XInputSetState");
        if (pix__xi_get) return;
    }
}

static void pix__pad_button(pix_gamepad& pad, int index, bool down) {
    pix_key_state& s = pad.buttons[index];
    if (down && !s.held)  s.pressed = true;
    if (!down && s.held)  s.released = true;
    s.held = down;
    s.code = (char)index;
}

// radial deadzone: kill the dead area by magnitude and rescale what is left to
// 0..1, so the stick keeps its direction and does not feel square at the edges
static void pix__pad_stick(SHORT sx, SHORT sy, float deadzone, float* ox, float* oy) {
    *ox = 0.0f; *oy = 0.0f;
    float x = (float)sx, y = (float)sy;
    float mag = sqrtf(x * x + y * y);
    if (mag <= deadzone || mag < 1e-5f) return;
    float nx = x / mag, ny = y / mag;              // direction from the raw magnitude
    if (mag > 32767.0f) mag = 32767.0f;
    float scaled = (mag - deadzone) / (32767.0f - deadzone);
    *ox = nx * scaled;
    *oy = ny * scaled;
}

static float pix__pad_trigger(BYTE v) {
    float f = (float)v;
    if (f <= PIX__XI_TRIGGER_FLOOR) return 0.0f;
    return (f - PIX__XI_TRIGGER_FLOOR) / (255.0f - PIX__XI_TRIGGER_FLOOR);
}

static void pix__clear_gamepad(pix_gamepad& pad) {
    for (int b = 0; b < PAD_BUTTON_COUNT; b++) {
        if (pad.buttons[b].held) pad.buttons[b].released = true;   // never leave one stuck down
        pad.buttons[b].held = false;
    }
    pad.left_x = pad.left_y = pad.right_x = pad.right_y = 0.0f;
    pad.left_trigger = pad.right_trigger = 0.0f;
}

static void pix__update_gamepads(pix_window& w) {
    ULONGLONG now = GetTickCount64();
    for (int i = 0; i < PIX_MAX_GAMEPADS; i++) {
        pix_gamepad& pad = w.gamepads[i];
        for (int b = 0; b < PAD_BUTTON_COUNT; b++) {
            pad.buttons[b].pressed = false;
            pad.buttons[b].released = false;
        }
        if (!pix__xi_get) continue;
        // an unplugged slot costs real time to query, so only retry now and then
        if (!pad.connected && now < pix__pad_retry[i]) continue;

        pix__xi_state st = {};
        if (pix__xi_get((DWORD)i, &st) != 0) {
            if (pad.connected) pix__clear_gamepad(pad);            // just unplugged
            pad.connected = false;
            pad.rumble_low = pad.rumble_high = 0.0f;
            pix__pad_retry[i] = now + PIX__PAD_RETRY_MS;
            continue;
        }

        pad.connected = true;
        WORD b = st.pad.buttons;
        pix__pad_button(pad, PAD_A,            (b & PIX__XI_A) != 0);
        pix__pad_button(pad, PAD_B,            (b & PIX__XI_B) != 0);
        pix__pad_button(pad, PAD_X,            (b & PIX__XI_X) != 0);
        pix__pad_button(pad, PAD_Y,            (b & PIX__XI_Y) != 0);
        pix__pad_button(pad, PAD_LEFT_BUMPER,  (b & PIX__XI_LEFT_SHOULDER) != 0);
        pix__pad_button(pad, PAD_RIGHT_BUMPER, (b & PIX__XI_RIGHT_SHOULDER) != 0);
        pix__pad_button(pad, PAD_BACK,         (b & PIX__XI_BACK) != 0);
        pix__pad_button(pad, PAD_START,        (b & PIX__XI_START) != 0);
        pix__pad_button(pad, PAD_LEFT_STICK,   (b & PIX__XI_LEFT_THUMB) != 0);
        pix__pad_button(pad, PAD_RIGHT_STICK,  (b & PIX__XI_RIGHT_THUMB) != 0);
        pix__pad_button(pad, PAD_DPAD_UP,      (b & PIX__XI_DPAD_UP) != 0);
        pix__pad_button(pad, PAD_DPAD_DOWN,    (b & PIX__XI_DPAD_DOWN) != 0);
        pix__pad_button(pad, PAD_DPAD_LEFT,    (b & PIX__XI_DPAD_LEFT) != 0);
        pix__pad_button(pad, PAD_DPAD_RIGHT,   (b & PIX__XI_DPAD_RIGHT) != 0);

        pix__pad_stick(st.pad.thumb_lx, st.pad.thumb_ly, PIX__XI_LEFT_DEADZONE,  &pad.left_x,  &pad.left_y);
        pix__pad_stick(st.pad.thumb_rx, st.pad.thumb_ry, PIX__XI_RIGHT_DEADZONE, &pad.right_x, &pad.right_y);
        pad.left_trigger  = pix__pad_trigger(st.pad.left_trigger);
        pad.right_trigger = pix__pad_trigger(st.pad.right_trigger);
    }
}

void pix_set_gamepad_rumble(pix_window& window, int pad, float low, float high) {
    if (pad < 0 || pad >= PIX_MAX_GAMEPADS || !pix__xi_set) return;
    if (!window.gamepads[pad].connected) return;
    if (low  < 0.0f) low  = 0.0f;
    if (low  > 1.0f) low  = 1.0f;
    if (high < 0.0f) high = 0.0f;
    if (high > 1.0f) high = 1.0f;

    pix__xi_rumble v;
    v.low  = (WORD)(low  * 65535.0f);
    v.high = (WORD)(high * 65535.0f);
    pix__xi_set((DWORD)pad, &v);
    window.gamepads[pad].rumble_low  = low;
    window.gamepads[pad].rumble_high = high;
}

void pix_stop_gamepad_rumble(pix_window& window) {
    for (int i = 0; i < PIX_MAX_GAMEPADS; i++) pix_set_gamepad_rumble(window, i, 0.0f, 0.0f);
}

static int pix__map_vk(int vk) {
    switch (vk) {
        case VK_UP:     return KEY_UP;
        case VK_DOWN:   return KEY_DOWN;
        case VK_LEFT:   return KEY_LEFT;
        case VK_RIGHT:  return KEY_RIGHT;
        case VK_ESCAPE: return KEY_ESC;
        case VK_TAB:    return KEY_TAB;
        case VK_OEM_4:  return KEY_LEFT_BRACKET;
        case VK_OEM_6:  return KEY_RIGHT_BRACKET;
        // left/right variants report as the generic modifier
        case VK_LSHIFT: case VK_RSHIFT:     return KEY_SHIFT;
        case VK_LCONTROL: case VK_RCONTROL: return KEY_CTRL;
        case VK_LMENU: case VK_RMENU:       return KEY_ALT;
    }
    if (vk > 0 && vk < 253) return vk; // letters/digits land on their ascii code
    return -1;
}

static void pix__key_down(pix_window* w, int code) {
    if (code < 0) return;
    if (!w->keystates[code].held) w->keystates[code].pressed = true;
    w->keystates[code].held = true;
    w->keystates[code].code = (char)code;
}

static void pix__key_up(pix_window* w, int code) {
    if (code < 0) return;
    if (w->keystates[code].held) w->keystates[code].released = true;
    w->keystates[code].held = false;
    w->keystates[code].code = (char)code;
}

// Alt-tabbing away with a key held never delivers its key-up to this window,
// which leaves the key stuck down until it is pressed again. Losing focus lets
// go of everything instead.
static void pix__release_all(pix_window* w) {
    for (int i = 0; i < 256; i++) if (w->keystates[i].held) pix__key_up(w, i);
}

static void pix__push_codepoint(pix_window* w, unsigned int cp) {
    if (cp < 32 && cp != '\t' && cp != '\r' && cp != '\b') return;  // control chars stay in keystates
    if (cp == 127) return;
    if (w->text_input_count < PIX_MAX_TEXT_INPUT) w->text_input[w->text_input_count++] = cp;
}

static LRESULT CALLBACK pix__wndproc(HWND h, UINT msg, WPARAM wp, LPARAM lp) {
    pix_window* w = pix__active;
    if (!w) return DefWindowProcA(h, msg, wp, lp);

    switch (msg) {
        case WM_CLOSE:
        case WM_DESTROY:
            w->should_close = true;
            return 0;

        case WM_SIZE: {
            int nw = LOWORD(lp), nh = HIWORD(lp);
            w->minimized = (wp == SIZE_MINIMIZED);
            if (!w->minimized && nw > 0 && nh > 0 && (nw != w->width || nh != w->height)) {
                w->width = nw;
                w->height = nh;
                w->resized = true;
            }
            return 0;
        }
        case WM_SETFOCUS:  w->focused = true;  return 0;
        case WM_KILLFOCUS: w->focused = false; pix__release_all(w); return 0;

        case WM_KEYDOWN:    pix__key_down(w, pix__map_vk((int)wp)); return 0;
        case WM_KEYUP:      pix__key_up  (w, pix__map_vk((int)wp)); return 0;
        // Alt and F10 arrive as system keys; they are recorded like any other
        // and then still handed to Windows so Alt+F4 and Alt+Space keep working.
        case WM_SYSKEYDOWN: pix__key_down(w, pix__map_vk((int)wp)); break;
        case WM_SYSKEYUP:   pix__key_up  (w, pix__map_vk((int)wp)); break;

        case WM_CHAR: {
            wchar_t c = (wchar_t)wp;
            if (c >= 0xD800 && c < 0xDC00) { w->pending_surrogate = c; return 0; }
            if (c >= 0xDC00 && c < 0xE000 && w->pending_surrogate) {
                unsigned int cp = 0x10000 + (((unsigned)w->pending_surrogate - 0xD800) << 10)
                                + ((unsigned)c - 0xDC00);
                w->pending_surrogate = 0;
                pix__push_codepoint(w, cp);
                return 0;
            }
            pix__push_codepoint(w, (unsigned int)c);
            return 0;
        }

        case WM_LBUTTONDOWN: pix__key_down(w, MOUSE_BUTTON_LEFT);  SetCapture(h); return 0;
        case WM_LBUTTONUP:   pix__key_up  (w, MOUSE_BUTTON_LEFT);  ReleaseCapture(); return 0;
        case WM_RBUTTONDOWN: pix__key_down(w, MOUSE_BUTTON_RIGHT); return 0;
        case WM_RBUTTONUP:   pix__key_up  (w, MOUSE_BUTTON_RIGHT); return 0;
        case WM_MBUTTONDOWN: pix__key_down(w, MOUSE_BUTTON_MID);   return 0;
        case WM_MBUTTONUP:   pix__key_up  (w, MOUSE_BUTTON_MID);   return 0;

        case WM_MOUSEWHEEL:
            w->mouse_wheel += GET_WHEEL_DELTA_WPARAM(wp) / WHEEL_DELTA;
            return 0;

        case WM_MOUSEMOVE: {
            int nx = GET_X_LPARAM(lp);
            int ny = GET_Y_LPARAM(lp);
            w->mouse_rel_x += nx - w->mouse_x;
            w->mouse_rel_y += ny - w->mouse_y;
            w->mouse_x = nx;
            w->mouse_y = ny;
            return 0;
        }
        case WM_ERASEBKGND:
            return 1;   // GL paints every pixel; skipping the erase avoids a flash on resize
    }
    return DefWindowProcA(h, msg, wp, lp);
}

static DWORD pix__window_style(unsigned int flags) {
    if (flags & PIX_WINDOW_RESIZABLE) return WS_OVERLAPPEDWINDOW;
    return WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX;
}

pix_window pix_create_window(const char* title, int width, int height, unsigned int flags) {
    pix_window win = {};
    win.flags = flags;

    // Without this Windows bitmap-stretches the whole window on a high DPI
    // display and every pixel the game draws comes out blurred.
    {
        typedef BOOL (WINAPI* PFN_SetProcessDPIAware)(void);
        HMODULE user32 = GetModuleHandleA("user32.dll");
        PFN_SetProcessDPIAware aware = user32
            ? (PFN_SetProcessDPIAware)GetProcAddress(user32, "SetProcessDPIAware") : 0;
        if (aware) aware();
    }

    WNDCLASSA wc = {};
    wc.style = CS_OWNDC | CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = pix__wndproc;
    wc.hInstance = GetModuleHandleA(0);
    wc.lpszClassName = "pix_window_class";
    wc.hCursor = LoadCursor(0, IDC_ARROW);
    wc.hIcon = LoadIcon(0, IDI_APPLICATION);
    RegisterClassA(&wc);

    // dummy context so we can resolve the modern WGL entry points
    HWND dummy = CreateWindowExA(0, wc.lpszClassName, "dummy", 0, 0, 0, 1, 1, 0, 0, wc.hInstance, 0);
    HDC ddc = GetDC(dummy);

    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    SetPixelFormat(ddc, ChoosePixelFormat(ddc, &pfd), &pfd);

    HGLRC drc = wglCreateContext(ddc);
    wglMakeCurrent(ddc, drc);

    PFNWGLCHOOSEPIXELFORMATARBPROC wglChoosePixelFormatARB =
        (PFNWGLCHOOSEPIXELFORMATARBPROC)wglGetProcAddress("wglChoosePixelFormatARB");
    PFNWGLCREATECONTEXTATTRIBSARBPROC wglCreateContextAttribsARB =
        (PFNWGLCREATECONTEXTATTRIBSARBPROC)wglGetProcAddress("wglCreateContextAttribsARB");

    wglMakeCurrent(0, 0);
    wglDeleteContext(drc);
    ReleaseDC(dummy, ddc);
    DestroyWindow(dummy);

    if (!wglChoosePixelFormatARB || !wglCreateContextAttribsARB) {
        MessageBoxA(0, "This graphics driver does not support modern OpenGL.", "pix", MB_OK | MB_ICONERROR);
        win.should_close = true;
        return win;
    }

    // real window
    DWORD style = pix__window_style(flags);
    RECT r = { 0, 0, width, height };
    AdjustWindowRect(&r, style, FALSE);
    win.handle = CreateWindowExA(0, wc.lpszClassName, title, style,
        CW_USEDEFAULT, CW_USEDEFAULT, r.right - r.left, r.bottom - r.top,
        0, 0, wc.hInstance, 0);
    win.dc = GetDC(win.handle);
    win.width = width;
    win.height = height;
    win.focused = true;

    const int pf_attr[] = {
        WGL_DRAW_TO_WINDOW_ARB, 1,
        WGL_SUPPORT_OPENGL_ARB, 1,
        WGL_DOUBLE_BUFFER_ARB,  1,
        WGL_PIXEL_TYPE_ARB,     WGL_TYPE_RGBA_ARB,
        WGL_COLOR_BITS_ARB,     32,
        WGL_DEPTH_BITS_ARB,     24,
        WGL_STENCIL_BITS_ARB,   8,
        0
    };
    int pf = 0;
    UINT pf_count = 0;
    wglChoosePixelFormatARB(win.dc, pf_attr, 0, 1, &pf, &pf_count);
    DescribePixelFormat(win.dc, pf, sizeof(pfd), &pfd);
    SetPixelFormat(win.dc, pf, &pfd);

    const int ctx_attr[] = {
        WGL_CONTEXT_MAJOR_VERSION_ARB, 4,
        WGL_CONTEXT_MINOR_VERSION_ARB, 4,
        WGL_CONTEXT_PROFILE_MASK_ARB,  WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
        WGL_CONTEXT_FLAGS_ARB,         (flags & PIX_WINDOW_DEBUG_GL) ? WGL_CONTEXT_DEBUG_BIT_ARB : 0,
        0
    };
    HGLRC rc = wglCreateContextAttribsARB(win.dc, 0, ctx_attr);
    if (!rc) {
        MessageBoxA(0, "OpenGL 4.4 is required but this driver could not create a 4.4 core context.\n"
                       "Updating the graphics driver usually fixes this.", "pix", MB_OK | MB_ICONERROR);
        win.should_close = true;
        return win;
    }
    win.gl_context = rc;
    wglMakeCurrent(win.dc, rc);

    // The driver's default is vsync on; start uncapped so a fresh window
    // reports what the engine actually costs rather than what the monitor
    // allows. pix_set_vsync puts it back.
    pix__swap_interval = (PFNWGLSWAPINTERVALEXTPROC)wglGetProcAddress("wglSwapIntervalEXT");
    pix_set_vsync(win, (flags & PIX_WINDOW_VSYNC) != 0);

    if (!(flags & PIX_WINDOW_HIDDEN)) ShowWindow(win.handle, SW_SHOW);
    if (flags & PIX_WINDOW_FULLSCREEN) {
        pix__active = &win;                 // so the WM_SIZE this sends lands in win
        pix_set_fullscreen(win, true);
        pix__active = 0;
    }

    pix__load_xinput();   // absent xinput just means no pads are ever reported

    // prime the mouse position so the first frame's relative delta is 0
    POINT p;
    GetCursorPos(&p);
    ScreenToClient(win.handle, &p);
    win.mouse_x = p.x;
    win.mouse_y = p.y;
    pix_time_seconds();   // start the clock
    return win;
}

void pix_destroy_window(pix_window& window) {
    if (window.gl_context) {
        wglMakeCurrent(0, 0);
        wglDeleteContext((HGLRC)window.gl_context);
    }
    if (window.handle) {
        if (window.dc) ReleaseDC(window.handle, window.dc);
        DestroyWindow(window.handle);
    }
    window.handle = 0;
    window.dc = 0;
    window.gl_context = 0;
}

void pix_set_vsync(pix_window& window, bool enabled) {
    (void)window;   // one context per window here, and it is already current
    if (pix__swap_interval) pix__swap_interval(enabled ? 1 : 0);
}

void pix_set_mouse_capture(pix_window& window, bool captured) {
    if (window.mouse_captured == captured) return;
    window.mouse_captured = captured;
    ShowCursor(captured ? FALSE : TRUE);
}

void pix_show_cursor(pix_window& window, bool visible) {
    (void)window;
    // ShowCursor keeps a counter rather than a flag; drive it to the state asked for
    CURSORINFO ci = { sizeof(ci) };
    GetCursorInfo(&ci);
    bool shown = (ci.flags & CURSOR_SHOWING) != 0;
    if (shown != visible) ShowCursor(visible ? TRUE : FALSE);
}

void pix_set_window_title(pix_window& window, const char* title) {
    SetWindowTextA(window.handle, title);
}

void pix_set_fullscreen(pix_window& window, bool fullscreen) {
    if (window.fullscreen == fullscreen || !window.handle) return;
    if (fullscreen) {
        window.windowed_style = (DWORD)GetWindowLongA(window.handle, GWL_STYLE);
        GetWindowRect(window.handle, &window.windowed_rect);
        MONITORINFO mi = { sizeof(mi) };
        GetMonitorInfoA(MonitorFromWindow(window.handle, MONITOR_DEFAULTTONEAREST), &mi);
        SetWindowLongA(window.handle, GWL_STYLE, (LONG)(window.windowed_style & ~WS_OVERLAPPEDWINDOW) | WS_POPUP);
        SetWindowPos(window.handle, HWND_TOP, mi.rcMonitor.left, mi.rcMonitor.top,
                     mi.rcMonitor.right - mi.rcMonitor.left, mi.rcMonitor.bottom - mi.rcMonitor.top,
                     SWP_NOOWNERZORDER | SWP_FRAMECHANGED | SWP_SHOWWINDOW);
    } else {
        SetWindowLongA(window.handle, GWL_STYLE, (LONG)window.windowed_style);
        const RECT& r = window.windowed_rect;
        SetWindowPos(window.handle, 0, r.left, r.top, r.right - r.left, r.bottom - r.top,
                     SWP_NOOWNERZORDER | SWP_FRAMECHANGED | SWP_SHOWWINDOW);
    }
    window.fullscreen = fullscreen;
}

void pix_swap_buffers(pix_window& window) {
    SwapBuffers(window.dc);
}

void pix_update_window(pix_window& window) {
    // clear per-frame edges before draining this frame's events
    SwapBuffers(window.dc);

    for (int i = 0; i < 256; i++) {
        window.keystates[i].pressed = false;
        window.keystates[i].released = false;
    }
    window.mouse_rel_x = 0;
    window.mouse_rel_y = 0;
    window.mouse_wheel = 0;
    window.resized = false;
    window.text_input_count = 0;

    pix__active = &window;
    MSG msg;
    while (PeekMessageA(&msg, 0, 0, 0, PM_REMOVE)) {
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }
    pix__active = 0;

    pix__update_gamepads(window);   // polled, not message driven

    // Re-centre last: the deltas above were measured against the previous
    // centre, and moving the pointer now also moves the reference point, so
    // the warp itself never shows up as motion.
    if (window.mouse_captured && GetForegroundWindow() == window.handle) {
        RECT rc;
        GetClientRect(window.handle, &rc);
        POINT centre = { (rc.right - rc.left) / 2, (rc.bottom - rc.top) / 2 };
        window.mouse_x = centre.x;
        window.mouse_y = centre.y;
        ClientToScreen(window.handle, &centre);
        SetCursorPos(centre.x, centre.y);
    }
}

#endif // PIX_PLATFORM_IMPL
