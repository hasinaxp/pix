#pragma once
#include <windows.h>
#include <windowsx.h>
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

struct pix_window {
    HWND  handle;
    HDC   dc;
    void* gl_context;               // HGLRC
    pix_key_state keystates[256];   // ascii-indexed; mouse buttons + special keys use the codes above
    int mouse_x;      int mouse_y;
    int mouse_rel_x;  int mouse_rel_y;
    bool should_close;
};

static pix_window pix_create_window(const char* title, int width, int height);
static void pix_update_window(pix_window& window);   // pump events + refresh input state
static void pix_swap_buffers(pix_window& window);

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
}

