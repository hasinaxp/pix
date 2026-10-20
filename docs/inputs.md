# Input

pix reads the keyboard, the mouse, up to four gamepads, and typed text. All
input is collected once per frame by `pix_frame()`, so within one frame every
query gives a consistent answer.

## Held, pressed, released

Every key, mouse button and gamepad button has three states. Choosing the
right one matters:

| question | function | true for |
|---|---|---|
| Is it down right now? | `pix_key_down` | every frame while it is held |
| Did it just go down? | `pix_key_pressed` | only the single frame it went down |
| Did it just come up? | `pix_key_released` | only the single frame it was let go |

Use **down** for continuous actions such as walking or holding a brake. Use
**pressed** for one-shot actions such as jumping, firing, or opening a menu.
If you used `pix_key_down` for a jump, holding the key would jump again every
frame.

```cpp
if (pix_key_down('D'))         player_x += speed * pix_dt();   // keeps moving while held
if (pix_key_pressed(KEY_SPACE)) jump();                         // once per press
if (pix_key_released('E'))      release_charged_attack();
```

## Keyboard

### Key codes

**Letters and digits** use their character, and letters must be
**upper-case**: `'W'`, `'A'`, `'1'`. Writing `'w'` will not work, because key
codes identify physical keys rather than the characters they type.

Other keys have names:

| keys | codes |
|---|---|
| arrows | `KEY_UP`, `KEY_DOWN`, `KEY_LEFT`, `KEY_RIGHT` |
| common | `KEY_SPACE`, `KEY_RETURN` (also `KEY_ENTER`), `KEY_ESC`, `KEY_TAB`, `KEY_BACKSPACE` |
| modifiers | `KEY_SHIFT`, `KEY_CTRL`, `KEY_ALT` (left and right report the same code) |
| function | `KEY_F1` .. `KEY_F12` |
| navigation | `KEY_INSERT`, `KEY_DELETE`, `KEY_HOME`, `KEY_END`, `KEY_PAGE_UP`, `KEY_PAGE_DOWN` |
| other | `KEY_PAUSE`, `KEY_CAPS_LOCK`, `KEY_LEFT_BRACKET`, `KEY_RIGHT_BRACKET` |

```cpp
if (pix_key_down(KEY_SHIFT) && pix_key_pressed('S')) save_game();
if (pix_key_pressed(KEY_F11)) pix_toggle_fullscreen();
```

### Rebindable controls

Key codes are plain `int`s, so you can store bindings in variables and let the
player change them:

```cpp
int key_jump = KEY_SPACE;
// ...
if (pix_key_pressed(key_jump)) jump();
```

## Movement axis

For the common "move with WASD, the arrow keys or a stick" case, use
`pix_input_axis()`. It returns a `vec2` with each component between -1 and 1:

- `x` is -1 for left (A / ←) and +1 for right (D / →).
- `y` is +1 for up / forward (W / ↑) and -1 for down / back (S / ↓).
- Diagonals are normalised, so moving diagonally isn't faster.
- If no key is held, it reads the left stick of gamepad 0.

```cpp
vec2 move = pix_input_axis();

// 2D: screen Y grows downward, so flip y
player += v2(move.x, -move.y) * (300.0f * pix_dt());

// 3D on the ground plane: forward is -Z in this example
pos += v3(move.x, 0, -move.y) * (5.0f * pix_dt());
```

## Mouse

```cpp
vec2 m = pix_mouse_position();             // pixels from the window's top-left corner
if (pix_mouse_pressed(PIX_MOUSE_LEFT)) shoot_at(m);
if (pix_mouse_down(PIX_MOUSE_RIGHT))   drag_camera(pix_mouse_delta());
zoom += pix_mouse_wheel() * 0.1f;          // +1 per notch forward, -1 back
```

| function | |
|---|---|
| `pix_mouse_position()` | cursor position in window pixels, `(0, 0)` top-left, Y down |
| `pix_mouse_delta()` | how far the mouse moved this frame, in pixels |
| `pix_mouse_wheel()` | wheel notches this frame |
| `pix_mouse_down/pressed/released(button)` | `PIX_MOUSE_LEFT`, `PIX_MOUSE_RIGHT`, `PIX_MOUSE_MIDDLE` |

### Mouse look

For a first-person or over-the-shoulder camera, lock the mouse.
`pix_set_mouse_locked(true)` hides the cursor and keeps it centred, so it never
hits the edge of the screen. `pix_mouse_delta()` keeps reporting movement:

```cpp
pix_set_mouse_locked(true);
// each frame:
vec2 d = pix_mouse_delta();
yaw   -= d.x * 0.003f;
pitch -= d.y * 0.003f;
pitch = clampf(pitch, -1.5f, 1.5f);
// unlock for menus:
if (pix_key_pressed(KEY_ESC)) pix_set_mouse_locked(false);
```

### Clicking on things in a 2D camera view

`pix_mouse_position()` is in screen pixels. If you draw your world through a
2D camera, convert the mouse into world coordinates first. The
[2D rendering page](2d-rendering.md#screen-and-world-coordinates) shows how.

For 3D, `pix_screen_to_ray` turns the mouse position into a ray you can test
against your objects. See [3D rendering](3d-rendering.md#picking-what-is-under-the-mouse).

## Gamepads

pix supports up to four Xbox-style controllers through XInput. Most modern
controllers present themselves this way. A computer without XInput just
reports no gamepads; nothing fails.

```cpp
const pix_gamepad& pad = pix_get_gamepad(0);      // 0..3
if (pad.connected) {
    if (pad.buttons[PAD_A].pressed) jump();
    vec2 stick = v2(pad.left_x, pad.left_y);      // -1..1, +y is up
    float throttle = pad.right_trigger;           // 0..1
}
```

`pix_gamepad` fields:

| field | |
|---|---|
| `connected` | whether a controller is plugged into this slot |
| `buttons[PAD_*]` | each has `.held`, `.pressed` and `.released`, like keys |
| `left_x`, `left_y`, `right_x`, `right_y` | sticks, -1..1, +Y is up. A dead zone is already applied, so a stick at rest reads exactly 0. |
| `left_trigger`, `right_trigger` | 0..1 |

Buttons: `PAD_A`, `PAD_B`, `PAD_X`, `PAD_Y`, `PAD_LEFT_BUMPER`,
`PAD_RIGHT_BUMPER`, `PAD_BACK`, `PAD_START`, `PAD_LEFT_STICK` and
`PAD_RIGHT_STICK` (sticks pressed in), `PAD_DPAD_UP`, `PAD_DPAD_DOWN`,
`PAD_DPAD_LEFT`, `PAD_DPAD_RIGHT`.

### Rumble

Rumble goes through the window module. `low` drives the heavy motor and
`high` the light one, both from 0 to 1. The motors keep the value until you
change it, so set it back to 0 when the effect ends:

```cpp
pix_set_gamepad_rumble(pix_get_window(), 0, 0.8f, 0.3f);   // pad 0: a hit
// later:
pix_set_gamepad_rumble(pix_get_window(), 0, 0.0f, 0.0f);
```

A simple way to make short bursts is to keep a timer:

```cpp
rumble = maxf(rumble - pix_dt() * 3.0f, 0.0f);       // fades over a third of a second
pix_set_gamepad_rumble(pix_get_window(), 0, rumble, rumble * 0.5f);
// on a hit: rumble = 1.0f;
```

`pix_shutdown` stops all rumble.

## Typed text

Key codes are the wrong tool for text fields. They don't know about Shift,
the keyboard layout, accents, or key repeat. For typing, use
`pix_text_input`, which returns the characters typed this frame as Unicode
code points:

```cpp
char name[32] = "";
// each frame:
int count;
const unsigned int* typed = pix_text_input(&count);
for (int i = 0; i < count; i++) {
    unsigned int c = typed[i];
    size_t len = strlen(name);
    if (c >= 32 && c < 127 && len + 1 < sizeof(name)) {   // printable ASCII
        name[len] = (char)c;
        name[len + 1] = 0;
    }
}
if (pix_key_pressed(KEY_BACKSPACE) && name[0]) name[strlen(name) - 1] = 0;
pix_draw_text(name, 100, 100, 32, PIX_WHITE);
```

Backspace, Enter and the arrow keys are not text, so check them with
`pix_key_pressed`. Up to 32 characters are kept per frame.

The built-in font only contains printable ASCII, so the example ignores other
characters. If you store them, keep them as code points or encode them as
UTF-8 yourself.

## Window state

A few window events are input too. The `pix_window` from `pix_get_window()`
exposes them:

```cpp
const pix_window& win = pix_get_window();
if (win.resized)  layout_ui(pix_width(), pix_height());   // true on the frame the size changed
if (!win.focused) pause_game();                           // the player switched to another window
if (win.minimized) { /* skip expensive work */ }
```

## Lower level

The app-layer functions are thin wrappers over the window's state.
`pix_get_window().keystates[code]` is a `pix_key_state` with `pressed`,
`released` and `held`. Mouse buttons live in the same array at
`MOUSE_BUTTON_LEFT`, `MOUSE_BUTTON_RIGHT` and `MOUSE_BUTTON_MID`. The fields
`mouse_x`, `mouse_y`, `mouse_rel_x`, `mouse_rel_y`, `mouse_wheel`,
`gamepads[4]` and `text_input[]` hold the rest.

## Reference

| function | |
|---|---|
| `bool pix_key_down(int key)` | held now |
| `bool pix_key_pressed(int key)` | went down this frame |
| `bool pix_key_released(int key)` | went up this frame |
| `bool pix_mouse_down/pressed/released(int button)` | `PIX_MOUSE_LEFT`, `_RIGHT`, `_MIDDLE` |
| `vec2 pix_mouse_position()` | window pixels, top-left origin |
| `vec2 pix_mouse_delta()` | movement this frame |
| `int pix_mouse_wheel()` | notches this frame |
| `void pix_set_mouse_locked(bool)` | hide and centre the cursor for mouse look |
| `vec2 pix_input_axis()` | WASD / arrows / left stick, -1..1, +y up |
| `const pix_gamepad& pix_get_gamepad(int index)` | index 0..3 |
| `const unsigned int* pix_text_input(int* count)` | code points typed this frame |
| `void pix_set_gamepad_rumble(pix_window&, int pad, float low, float high)` | 0..1, held until changed |
| `void pix_stop_gamepad_rumble(pix_window&)` | all pads |
| `void pix_show_cursor(pix_window&, bool visible)` | |
