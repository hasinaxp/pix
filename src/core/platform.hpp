#pragma once
#include <windows.h>
#include <windowsx.h>
#include <math.h>
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

#define MOUSE_BUTTON_LEFT  255
#define MOUSE_BUTTON_RIGHT 254
#define MOUSE_BUTTON_MID   253

#define KEY_UP    201
#define KEY_DOWN  202
#define KEY_LEFT  203
#define KEY_RIGHT 204
#define KEY_ESC   205
#define KEY_TAB   206

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

struct pix_window {
    HWND  handle;
    HDC   dc;
    void* gl_context;               // HGLRC
    pix_key_state keystates[256];   // ascii-indexed; mouse buttons + special keys use the codes above
    int mouse_x;      int mouse_y;
    int mouse_rel_x;  int mouse_rel_y;
    bool should_close;

    pix_gamepad gamepads[PIX_MAX_GAMEPADS];
};

static pix_window pix_create_window(const char* title, int width, int height);
static void pix_update_window(pix_window& window);   // pump events + refresh input state
static void pix_swap_buffers(pix_window& window);

// low = the heavy motor, high = the light one; both 0..1, held until changed
static void pix_set_gamepad_rumble(pix_window& window, int pad, float low, float high);
static void pix_stop_gamepad_rumble(pix_window& window);   // all pads, e.g. on shutdown

// ---- WGL extension constants ----
#define WGL_CONTEXT_MAJOR_VERSION_ARB    0x2091
#define WGL_CONTEXT_MINOR_VERSION_ARB    0x2092
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


// -------------------- implementation --------------------

static pix_window* pix__active; // valid only while pix_update_window is pumping

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
#define PIX__XI_NOT_CONNECTED   1167    // ERROR_DEVICE_NOT_CONNECTED
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

static void pix_set_gamepad_rumble(pix_window& window, int pad, float low, float high) {
    if (pad < 0 || pad >= PIX_MAX_GAMEPADS || !pix__xi_set) return;
    if (!window.gamepads[pad].connected) return;
    if (low  < 0.0f) low  = 0.0f;  if (low  > 1.0f) low  = 1.0f;
    if (high < 0.0f) high = 0.0f;  if (high > 1.0f) high = 1.0f;

    pix__xi_rumble v;
    v.low  = (WORD)(low  * 65535.0f);
    v.high = (WORD)(high * 65535.0f);
    pix__xi_set((DWORD)pad, &v);
    window.gamepads[pad].rumble_low  = low;
    window.gamepads[pad].rumble_high = high;
}

static void pix_stop_gamepad_rumble(pix_window& window) {
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
    w->keystates[code].held = false;
    w->keystates[code].released = true;
    w->keystates[code].code = (char)code;
}

static LRESULT CALLBACK pix__wndproc(HWND h, UINT msg, WPARAM wp, LPARAM lp) {
    pix_window* w = pix__active;
    if (!w) return DefWindowProcA(h, msg, wp, lp);

    switch (msg) {
        case WM_CLOSE:
        case WM_DESTROY:
            w->should_close = true;
            return 0;

        case WM_KEYDOWN:    pix__key_down(w, pix__map_vk((int)wp)); return 0;
        case WM_KEYUP:      pix__key_up  (w, pix__map_vk((int)wp)); return 0;

        case WM_LBUTTONDOWN: pix__key_down(w, MOUSE_BUTTON_LEFT);  return 0;
        case WM_LBUTTONUP:   pix__key_up  (w, MOUSE_BUTTON_LEFT);  return 0;
        case WM_RBUTTONDOWN: pix__key_down(w, MOUSE_BUTTON_RIGHT); return 0;
        case WM_RBUTTONUP:   pix__key_up  (w, MOUSE_BUTTON_RIGHT); return 0;
        case WM_MBUTTONDOWN: pix__key_down(w, MOUSE_BUTTON_MID);   return 0;
        case WM_MBUTTONUP:   pix__key_up  (w, MOUSE_BUTTON_MID);   return 0;

        case WM_MOUSEMOVE: {
            int nx = GET_X_LPARAM(lp);
            int ny = GET_Y_LPARAM(lp);
            w->mouse_rel_x += nx - w->mouse_x;
            w->mouse_rel_y += ny - w->mouse_y;
            w->mouse_x = nx;
            w->mouse_y = ny;
            return 0;
        }
    }
    return DefWindowProcA(h, msg, wp, lp);
}

static pix_window pix_create_window(const char* title, int width, int height) {
    pix_window win = {};

    WNDCLASSA wc = {};
    wc.lpfnWndProc = pix__wndproc;
    wc.hInstance = GetModuleHandleA(0);
    wc.lpszClassName = "pix_window_class";
    wc.hCursor = LoadCursorA(0, IDC_ARROW);
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

    // real window
    RECT r = { 0, 0, width, height };
    DWORD style = WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX;
    AdjustWindowRect(&r, style, FALSE);
    win.handle = CreateWindowExA(0, wc.lpszClassName, title, style,
        CW_USEDEFAULT, CW_USEDEFAULT, r.right - r.left, r.bottom - r.top,
        0, 0, wc.hInstance, 0);
    win.dc = GetDC(win.handle);

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
        0
    };
    HGLRC rc = wglCreateContextAttribsARB(win.dc, 0, ctx_attr);
    win.gl_context = rc;
    wglMakeCurrent(win.dc, rc);

    ShowWindow(win.handle, SW_SHOW);

    pix__load_xinput();   // absent xinput just means no pads are ever reported

    // prime the mouse position so the first frame's relative delta is 0
    POINT p;
    GetCursorPos(&p);
    ScreenToClient(win.handle, &p);
    win.mouse_x = p.x;
    win.mouse_y = p.y;
    return win;
}

static void pix_update_window(pix_window& window) {
    // clear per-frame edges before draining this frame's events
    SwapBuffers(window.dc);

    for (int i = 0; i < 256; i++) {
        window.keystates[i].pressed = false;
        window.keystates[i].released = false;
    }
    window.mouse_rel_x = 0;
    window.mouse_rel_y = 0;

    pix__active = &window;
    MSG msg;
    while (PeekMessageA(&msg, 0, 0, 0, PM_REMOVE)) {
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }
    pix__active = 0;

    pix__update_gamepads(window);   // polled, not message driven
}

