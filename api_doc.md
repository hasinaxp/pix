# pix API reference

Header-only C-style engine, no OOP, no third-party dependencies (only `windows.h` +
`GL/gl.h` + libc). The headers hold their own implementations; `src/main.cpp` is the
only translation unit. Windows only, OpenGL 4.4 core profile.

```
src/core/     dtype math platform opengl_api opengl_utils shader_sources
              animation renderer font sprite text
              platform_audio sound
src/loader/   asset_types png_loader obj_loader gltf_loader wav_loader data_loader
src/demos/    city_demo animation_demo
```

Include order matters — later headers assume earlier ones are already visible:

```
core/platform -> core/opengl_api -> core/opengl_utils -> core/shader_sources
              -> core/math -> loader/data_loader -> core/renderer
              -> core/font -> core/sprite -> core/text
```

`loader/data_loader.hpp` is the front door for assets: it pulls in the individual
format readers (`png_loader`, `obj_loader`, `gltf_loader`, `wav_loader`) and owns
the arena plus the tables they parse into. `core/sound.hpp` sits on
`core/platform_audio.hpp` the same way `core/renderer.hpp` sits on the GL headers.

---

## dtype.hpp

Base types shared by every other header.

```cpp
typedef uint32_t idx;              // generic handle: GL object id, table index, ...

struct vec2 { float x, y; };
struct vec3 { float x, y, z; };
struct vec4 { float x, y, z, w; };
struct quat { float x, y, z, w; }; // rotation, w last (glTF's component order)
struct mat4 { float data[16]; };   // column-major

struct mem_arena { char* data; size_t size; size_t capacity; };

template<typename T> T* allocate(mem_arena& arena, size_t count); // bump-allocate, 16-byte aligned, null on OOM
void arena_reset(mem_arena& arena);                               // size = 0, keeps the backing block

#define MB (1024 * 1024)
```

---

## math.hpp

Free functions on `vec3`/`mat4`. No SIMD, no operator overloading.

```cpp
vec3 v3(float x, float y, float z);
vec3 v3add(vec3, vec3);
vec3 v3sub(vec3, vec3);
vec3 v3scale(vec3, float);
float v3dot(vec3, vec3);
vec3 v3cross(vec3, vec3);
vec3 v3norm(vec3);
vec3 v3lerp(vec3 a, vec3 b, float t);
vec4 v4(float x, float y, float z, float w);

quat quat_identity();
quat quat_norm(quat);
quat quat_mul(quat a, quat b);
quat quat_slerp(quat a, quat b, float t);      // shortest arc, nlerp for tiny arcs

mat4 mat4_identity();
mat4 mat4_mul(mat4 a, mat4 b);                 // a * b
mat4 mat4_translate(float x, float y, float z);
mat4 mat4_scale(float x, float y, float z);
mat4 mat4_rotate_y(float radians);
mat4 mat4_perspective(float fovy, float aspect, float near, float far);
mat4 mat4_ortho(float left, float right, float bottom, float top, float near, float far);
    // maps [left,right]x[bottom,top] to NDC; pass top=0,bottom=height for
    // y-down screen space (the convention sprite.hpp/text.hpp use)
mat4 mat4_lookat(vec3 eye, vec3 center, vec3 up);

mat4 mat4_from_quat(quat);
mat4 mat4_from_trs(vec3 t, quat r, vec3 s);    // T * R * S, built directly
void mat4_decompose(const mat4&, vec3* t, quat* r, vec3* s); // assumes no shear
```

---

## platform.hpp

Window + input, no GL calls of its own beyond context creation.

```cpp
struct pix_key_state { bool pressed, released, held; char code; };

#define MOUSE_BUTTON_LEFT  255
#define MOUSE_BUTTON_RIGHT 254
#define MOUSE_BUTTON_MID   253
#define KEY_UP 201  KEY_DOWN 202  KEY_LEFT 203  KEY_RIGHT 204  KEY_ESC 205  KEY_TAB 206

struct pix_window {
    HWND handle; HDC dc; void* gl_context;
    pix_key_state keystates[256];  // ascii-indexed; mouse/special keys use the codes above
    int mouse_x, mouse_y, mouse_rel_x, mouse_rel_y;
    bool should_close;
    pix_gamepad gamepads[PIX_MAX_GAMEPADS];   // PIX_MAX_GAMEPADS == 4
};

pix_window pix_create_window(const char* title, int width, int height);
    // creates a 4.4 core-profile context, shows the window
void pix_update_window(pix_window& window);
    // swaps buffers, refreshes pressed/released/held, pumps messages, polls pads
```

Call once per frame: `pix_update_window` **swaps first, then pumps**, so it belongs at
the very top of the loop, before reading `keystates`.

### gamepads

XInput, resolved at runtime from `xinput1_4 / 1_3 / 9_1_0.dll`, so there is no link
dependency and a machine without it simply reports no pads.

```cpp
#define PAD_A 0  PAD_B 1  PAD_X 2  PAD_Y 3
#define PAD_LEFT_BUMPER 4  PAD_RIGHT_BUMPER 5  PAD_BACK 6  PAD_START 7
#define PAD_LEFT_STICK 8   PAD_RIGHT_STICK 9   // sticks pressed in
#define PAD_DPAD_UP 10  PAD_DPAD_DOWN 11  PAD_DPAD_LEFT 12  PAD_DPAD_RIGHT 13
#define PAD_BUTTON_COUNT 14

struct pix_gamepad {
    bool connected;
    pix_key_state buttons[PAD_BUTTON_COUNT];  // same edges as keystates
    float left_x, left_y, right_x, right_y;   // -1..1, radial deadzone applied, +y is up
    float left_trigger, right_trigger;        // 0..1
    float rumble_low, rumble_high;            // last values sent to the motors
};

void pix_set_gamepad_rumble(pix_window&, int pad, float low, float high);  // 0..1, held until changed
void pix_stop_gamepad_rumble(pix_window&);                                 // all pads
```

Read it exactly like the keyboard:

```cpp
const pix_gamepad& gp = window.gamepads[0];
if (gp.connected && gp.buttons[PAD_A].pressed) jump();
yaw += gp.right_x * dt * 2.5f;
```

Notes:

- **Radial deadzone**, not per-axis: the dead area is removed by magnitude and the
  remainder rescaled to 0..1, so direction is preserved and a full diagonal lands on
  the unit circle rather than feeling square.
- **Hot-plug works**, but an unplugged XInput slot is expensive to query, so empty
  slots are only re-probed every 2s. A pad plugged in mid-game is picked up within
  that window; connected pads are polled every frame as normal.
- Unplugging a pad releases its held buttons and zeroes its axes, so nothing sticks.
- Rumble calls on a disconnected or out-of-range pad are safe no-ops.

---

## opengl_api.hpp

Function-pointer typedefs + loader for every GL 3+ entry point the engine uses.
`GL_FUNC_LIST` (X-macro) declares them as file-scope `static` globals and resolves
them all in one shot.

```cpp
bool opengl_load_functions(); // wglGetProcAddress, falling back to GetProcAddress
                               // on opengl32.dll; false if anything failed to resolve
```

Everything else (`glCreateShader`, `glBindVertexArray`, `glDrawArraysInstanced`,
`glVertexAttribIPointer`, ...) is just used directly after loading — see
`GL_FUNC_LIST` for the exact set. Core GL 1.1 entry points (`glEnable`, `glBlendFunc`,
`glGetTexLevelParameteriv`, ...) come straight from `<GL/gl.h>` and need no loading.

---

## opengl_utils.hpp

Small shader/texture helpers built on top of `opengl_api.hpp`.

```cpp
#define TEXTURE_PIXELATED 1   // nearest, no mipmaps
#define TEXTURE_LINEAR    2   // linear, no mipmaps
#define TEXTURE_BILINEAR  3   // linear + mipmaps

idx opengl_create_shader(const char* vsrc, const char* fsrc);
    // compiles + links; pops a MessageBox with the log on failure

idx opengl_create_texture2d(int width, int height, int channel, void* data, idx flag = TEXTURE_PIXELATED);
    // data must be tightly packed RGBA8 regardless of `channel`
```

---

## shader_sources.hpp

Raw GLSL string constants, no C++ dependencies.

```cpp
VSHDER_BASIC / FSHDER_BASIC   // instanced mesh shader used by renderer.hpp
                              // layout: pos(0) normal(1) uv(2), mat4 aModel(3..6)
                              // uniforms: uViewProj, uTex, uColor

VSHDER_SKINNED                // skinned mesh shader, pairs with FSHDER_BASIC
                              // layout: pos(0) normal(1) uv(2) boneIds(3) weights(4)
                              // uniforms: uViewProj, uModel, uBones[128], uTex, uColor
                              // no V flip - glTF uvs already start at the top-left

VSHDER_SPRITE / FSHDER_SPRITE // instanced quad shader used by sprite.hpp
VSHDER_TEXT   / FSHDER_TEXT   // same vertex stage; fragment samples an SDF atlas
                              // layout: aCorner(0) aBox(1) aCrop(2) aTexIndex(3, uint)
                              // uniforms: uViewProj, uTex[32], uTexSize[32], uColor (text only)
```

`uTex`/`uTexSize` are sized 32 to match `MAX_SPRITE_TEXTURES` in sprite.hpp — keep
them in sync if that constant ever changes.

---

## loader/asset_types.hpp

The data model every loader fills in and `data_loader.hpp` stores.

```cpp
struct vertex        { vec3 position; vec3 normal; vec2 uv; };
struct vertex_rigged { vec3 position; vec3 normal; vec2 uv; vec4 bone_ids; vec4 bone_weights; };
struct mesh_file_data  { size_t vertex_count; vertex* vertex_data; size_t index_count; uint16_t* index_data; };
struct image_file_data { int width, height, channel; char* data; };
struct sound_file_data { int16_t* samples; size_t frame_count; int channels; int rate; };

// bones are topologically sorted: parent < own index, so a pose resolves in ONE
// linear pass -> global[i] = (parent < 0 ? root_transform : global[parent]) * local[i]
struct bone {
    int32_t parent;             // -1 for a root bone
    mat4    inverse_bind;       // mesh space -> bone space
    vec3    local_position;     // rest pose, used for bones a clip does not animate
    quat    local_rotation;
    vec3    local_scale;
    char    name[32];
};
struct skeleton_file_data {
    size_t bone_count; bone* bones;
    mat4   root_transform;      // scene transform above the joints (unit/axis fixups)
};

// T/R/S share one timeline, so posing a bone is one search + one lerp/slerp
struct bone_keyframe { float time; vec3 position; quat rotation; vec3 scale; };
struct bone_animation_track {
    int32_t bone;
    size_t  keyframe_count;     // 0 = not animated by this clip, hold the rest pose
    bone_keyframe* keyframes;   // ascending by time
};
struct animation_clip_file_data {
    char name[32]; float duration; int32_t skeleton;
    size_t track_count;         // == bone_count; dense, index it BY BONE
    bone_animation_track* tracks;
};

struct skinned_mesh_file_data {
    size_t vertex_count; vertex_rigged* vertex_data;
    size_t index_count;  uint16_t* index_data;
    int32_t skeleton;
};
struct model_file_data {        // what one model file yielded
    int32_t skinned_mesh, skeleton, first_animation;
    size_t  animation_count;    // clips are contiguous from first_animation
    int32_t image;              // embedded base colour texture, -1 if none
};
```

---

## loader/data_loader.hpp

Owns the arena and the asset tables; delegates parsing to the format readers.

```cpp
struct pix_data_loader {
    mem_arena arena;
    mesh_file_data*  mesh_files;  size_t mesh_file_count;
    image_file_data* image_files; size_t image_file_count;
    skeleton_file_data*       skeletons;          size_t skeleton_count;
    skinned_mesh_file_data*   skinned_mesh_files; size_t skinned_mesh_file_count;
    animation_clip_file_data* animation_clips;    size_t animation_clip_count;
    model_file_data*          model_files;        size_t model_file_count;
    sound_file_data*          sound_files;        size_t sound_file_count;
};

pix_data_loader pix_create_data_loader(size_t capacity = 128 * MB);
void pix_destroy_data_loader(pix_data_loader& loader);

idx load_mesh_obj_file(pix_data_loader&, const char* filepath);
idx load_image_file(pix_data_loader&, const char* filepath, size_t channels);
idx load_sound_wav_file(pix_data_loader&, const char* filepath);
    // decodes into loader.sound_files; hand the result to sound.hpp's load_sound
idx load_model_gltf_file(pix_data_loader&, const char* filepath);
    // fills the skeleton / skinned mesh / animation tables in one call;
    // returns an index into model_files, or (idx)-1 on failure

skinned_mesh_file_data*   get_model_mesh(pix_data_loader&, idx model);
skeleton_file_data*       get_model_skeleton(pix_data_loader&, idx model);
animation_clip_file_data* find_animation(pix_data_loader&, idx model, const char* name);
    // name == nullptr returns the model's first clip
```

Everything allocated lives in `loader.arena` and stays valid until it is reset or
freed. `renderer.hpp` copies what it needs onto the GPU immediately.

Typical animated-model load:

```cpp
idx fox = load_model_gltf_file(loader, "assets/glTF/Fox/Fox.glb");
skinned_mesh_file_data* mesh = get_model_mesh(loader, fox);
skeleton_file_data*     skel = get_model_skeleton(loader, fox);
animation_clip_file_data* run = find_animation(loader, fox, "Run");
```

---

## loader/obj_loader.hpp

```cpp
bool obj_load_file(mem_arena&, const char* path, mesh_file_data* out);
```

Positions/uvs/normals + polygon faces, fan triangulated, vertices emitted unshared.
Generates flat normals when the file has none.

---

## loader/wav_loader.hpp

```cpp
bool wav_load_file(mem_arena&, const char* path, sound_file_data* out);
```

RIFF/WAVE: PCM 8/16/24/32-bit and 32-bit IEEE float, mono or stereo, all
converted to interleaved 16-bit signed (what the mixer works in). The raw file
goes through malloc so only decoded pcm lands in the arena. Compressed formats
(ADPCM and friends) and >2 channels are not handled.

---

## loader/gltf_loader.hpp

From-scratch glTF 2.0 reader (`.glb`, or `.gltf` with an external `.bin`) covering
skinned meshes, skeletons and skeletal animation. Includes its own JSON tokenizer.

```cpp
struct gltf_result {
    skinned_mesh_file_data    mesh;
    skeleton_file_data        skeleton;
    animation_clip_file_data* clips;  size_t clip_count;
    image_file_data           image;  // .data == 0 when absent/undecodable
};
bool gltf_load_file(mem_arena&, const char* path, gltf_result* out);
```

What it does for you:
- **Generates whatever the file omits** — sequential indices when a primitive is
  non-indexed, normals derived from the triangles, default uvs/joints, and weights
  re-normalized to sum to 1 (unweighted vertices get pinned to bone 0).
- **Re-orders bones topologically** and remaps `JOINTS_0` to match, so a pose is a
  single linear pass at runtime.
- **Resamples animation onto one timeline per bone** — glTF stores translation,
  rotation and scale as independent samplers; they get merged so runtime does one
  search plus one lerp/slerp instead of three.
- Honours `byteStride` (interleaved buffers), normalized integer attributes, node
  `matrix` *or* TRS form, `STEP`/`LINEAR` interpolation, and merges all primitives
  of a mesh into one buffer.
- Captures ancestor transforms above the joints into `skeleton.root_transform`
  (this is what rights a Z-up model such as CesiumMan).

Not handled: sparse accessors, base64 `data:` uris, morph targets, CUBICSPLINE
tangents (sampled linearly off the key value), and non-PNG embedded textures.
Meshes above 65535 vertices are rejected since the renderer draws with
`GL_UNSIGNED_SHORT`.

---

## loader/png_loader.hpp

From-scratch PNG decoder (8-bit, colour types 0/2/3/4/6, non-interlaced) with its
own DEFLATE/zlib inflate. No GL dependency.

```cpp
struct png_result { int width, height, channels /* always 4 */; unsigned char* pixels; };

bool png_load_file(mem_arena& arena, const char* path, png_result* out);
bool png_load_memory(mem_arena& arena, const unsigned char* file, size_t size, png_result* out);
    // pixels is width*height*4, allocated from `arena`;
    // the memory form is what gltf_loader uses for textures embedded in a .glb
```

---

## animation.hpp

Runtime half of skinning: turns a clip + a time into one skinning matrix per bone.

```cpp
#define MAX_ANIM_BONES     128   // must match uBones[] in shader_sources.hpp
#define MAX_ANIMATOR_CLIPS 16

struct animation {              // a resolved pose, ready for the GPU
    mat4   bones[MAX_ANIM_BONES];   // bones[i] = global_transform(i) * inverse_bind(i)
    size_t bone_count;
};

struct animator {
    skeleton_file_data*       skeleton;
    animation_clip_file_data* clips[MAX_ANIMATOR_CLIPS];
    size_t clip_count;
    idx clip; float time; float speed; bool loop;   // playback state
    animation pose;                                  // refreshed by animator_update
};

animator pix_create_animator(skeleton_file_data* skeleton);
idx  animator_add_clip(animator&, animation_clip_file_data* clip);

// the core query - clip index + seconds -> every bone's matrix.
// `time` is wrapped into the clip, so raw elapsed time is fine.
void animator_sample(const animator&, idx clip, float time, animation* out);

// convenience playback on top of it
void animator_play(animator&, idx clip, bool loop = true, float speed = 1.0f);
void animator_update(animator&, float dt);          // advances time, refills .pose

void  animation_rest_pose(const skeleton_file_data&, animation* out);
float animator_duration(const animator&, idx clip);
const char* animator_clip_name(const animator&, idx clip);
```

Posing is a single linear pass with no recursion and no per-bone track lookup —
`gltf_loader` guarantees bones are topologically sorted and clip tracks are dense
(`tracks[i]` belongs to bone `i`). Finding a keyframe is a bisect over the bone's
merged T/R/S timeline, then one `v3lerp` + one `quat_slerp`. An out-of-range clip
index falls back to the rest pose.

One animator per actor: several actors can share a skeleton and clips while
playing independently.

---

## platform_audio.hpp

The only file that talks to the OS about sound. Opens the default output device
and keeps a ring of PCM blocks in flight, asking a callback to refill each one as
it drains. Built on waveOut, which needs no COM and ships with `windows.h`.

```cpp
#define PIX_AUDIO_RATE     44100
#define PIX_AUDIO_CHANNELS 2
#define PIX_AUDIO_BLOCKS   4     // blocks queued on the device
#define PIX_AUDIO_FRAMES   1024  // frames per block (~93ms of slack in total)

// fills `frame_count` interleaved stereo frames; runs on the audio thread
typedef void (*pix_audio_fill)(int16_t* frames, size_t frame_count, void* user);

bool pix_open_audio(pix_audio_device&, pix_audio_fill fill, void* user);
void pix_close_audio(pix_audio_device&);
```

The device is handed to the audio thread by address, so it must outlive the
thread - keep it in a static or a long-lived struct. Whatever the callback
touches is shared with that thread and needs synchronizing.

---

## sound.hpp

Game-facing audio. Load or generate a sound, then fire and forget.

```cpp
#define MAX_SOUNDS 64
#define MAX_VOICES 32
#define SOUND_INVALID ((idx)-1)

struct sound { int16_t* samples; size_t frame_count; int channels; int rate; };

bool pix_create_audio(pix_audio&, size_t capacity = 16 * MB);
void pix_destroy_audio(pix_audio&);

idx load_sound(pix_audio&, const sound_file_data&);   // from loader/wav_loader
idx create_sound(pix_audio&, const int16_t* samples, size_t frames, int channels, int rate);

idx  play_sound(pix_audio&, idx sound, float volume = 1.0f, float pitch = 1.0f,
                 bool loop = false, float pan = 0.0f);   // -> voice handle
void stop_sound(pix_audio&, idx voice);
void stop_all_sounds(pix_audio&);
bool is_sound_playing(pix_audio&, idx voice);
void set_sound_volume(pix_audio&, idx voice, float volume);
void set_sound_pan(pix_audio&, idx voice, float pan);
void set_master_volume(pix_audio&, float volume);
size_t active_voice_count(pix_audio&);

void pix_audio_mix(pix_audio&, int16_t* out, size_t frames);  // the device calls this
```

Loading a wav mirrors how meshes flow - parse in `loader/`, then hand it to the
subsystem that owns it:

```cpp
idx wav  = load_sound_wav_file(loader, "assets/step.wav");  // loader table
idx step = load_sound(audio, loader.sound_files[wav]);      // copied into audio
play_sound(audio, step, 0.8f, 1.0f, false, -0.5f);          // quieter, to the left
```

Notes:

- **Voice handles carry a generation counter.** A handle kept past the end of a
  one-shot is ignored rather than stopping whatever later reused that slot.
- Sounds play at any rate; `pitch` and the rate difference are folded into one
  fractional step with linear interpolation, so a 22kHz asset just works.
- `play_sound` returns `SOUND_INVALID` when all `MAX_VOICES` are busy; it never
  steals a voice.
- A missing or busy output device is **not** fatal: `pix_create_audio` returns
  false but everything still loads and "plays", so callers need no audio-less path.
- `pix_audio_mix` is exposed so the mixer can be driven and tested without a device.

---

## renderer.hpp

Owns the GPU-side mesh/material/instance pipeline: one shared vertex buffer, one
shared index buffer, one streamed per-frame instance buffer.

```cpp
struct mesh     { idx vbo, vertex_offset, vertex_size, vertex_count, index_offset, index_count; };
struct material { idx shader, texture; vec3 color; float metallic, roughness; };
struct pix_render_instance { idx mesh, material; mat4 trainsform; };  // [sic]
struct camera   { vec3 position, direction, up; };

struct pix_renderer {
    int width, height; vec3 clear_color;
    pix_render_instance instances[MAX_RENDERABLE];   // MAX_RENDERABLE = 1024
    mesh meshes[MAX_MESHES]; material materials[MAX_MATERIALS]; idx shaders[MAX_SHADERS];
    // MAX_MESHES = MAX_MATERIALS = 128, MAX_SHADERS = 32
    idx shader; size_t mesh_count, material_count, shader_count, renderable_count;
    idx vertex_cursor, index_cursor;      // bump allocators into the shared buffers
    mat4 view_matrix, projection_matrix;
    idx vao, models_vbo, instances_vbo, ebo;
    idx white_texture; mem_arena tex_arena;
};

pix_renderer pix_create_renderer(int width, int height, idx shader,
                                  vec3 clear_color = { 0.0f, 0.5f, 0.8f });
void pix_destory_renderer(pix_renderer& renderer);   // [sic] frees tex_arena only

idx load_mesh(pix_renderer& renderer, mesh_file_data& mesh_data);
    // appends into models_vbo/ebo via glBufferSubData; records vertex/index offsets
idx load_skinned_mesh(pix_renderer& renderer, skinned_mesh_file_data& mesh_data);
    // same, into the animated pool; returns an index into skinned_meshes[]
idx load_material(pix_renderer& renderer, const char* texture_path = nullptr,
                   vec3 color = {1,1,1}, float metallic = 0.1f, float roughness = 0.5f);
idx load_material_image(pix_renderer& renderer, image_file_data* image,
                   vec3 color = {1,1,1}, float metallic = 0.1f, float roughness = 0.5f);
    // for an already-decoded texture, e.g. one embedded in a .glb
idx get_default_material(pix_renderer& renderer);     // white, untextured
idx load_shader(pix_renderer& renderer, const char* vsrc, const char* fsrc);

void begin_frame(pix_renderer& renderer, camera& cam);   // rebuilds view_matrix, resets both instance lists
void push_instance(pix_renderer& renderer, pix_render_instance& instance);
void push_animated_instance(pix_renderer& renderer, pix_render_instance& instance,
                             const animation& pose);
    // `instance.mesh` indexes skinned_meshes[]; `pose` is referenced, not copied,
    // so it must stay alive until end_frame (an animator's .pose does)
void end_frame(pix_renderer& renderer, bool clear_instances = true);
```

`end_frame` runs two passes:

- **static** — sorts instances by (mesh, material) and issues one
  `glDrawElementsInstancedBaseVertex` per (mesh, material) run.
- **animated** — one draw per instance, with `uModel` and the pose's `uBones[]`
  uploaded per draw. Instancing buys nothing here because every instance needs its
  own bone matrices, which is also why the animated path has no instance VBO.

Both pools live in their own VAO: static meshes use the `vertex` layout
(pos/normal/uv + per-instance mat4 at locations 3..6), skinned meshes use
`vertex_rigged` (pos/normal/uv/bone_ids/bone_weights at locations 0..4).
`pix_create_renderer` compiles `VSHDER_SKINNED + FSHDER_BASIC` into
`renderer.skinned_shader`; overwrite that field to use your own.

Typical frame:

```cpp
begin_frame(renderer, cam);
for (...) push_instance(renderer, instance);

animator_update(actor, dt);
push_animated_instance(renderer, fox_instance, actor.pose);

end_frame(renderer);
```

---

## font.hpp

From-scratch TrueType loader (`glyf` outlines, cmap formats 0/4/6/12, simple +
composite glyphs) that bakes a **signed distance field** atlas. No GL dependency —
produces a plain CPU bitmap; `text.hpp` uploads it lazily.

```cpp
#define FONT_ATLAS_WIDTH  1024
#define FONT_ATLAS_HEIGHT 1024
#define FONT_ATLAS_GLYPH_WIDTH 64        // 16x16 grid of cells
#define FONT_SDF_SPREAD 6                // atlas pixels of signed distance around each outline

const char* FONT_CHAR_SET;               // printable ASCII 0x20..0x7E, baked at slot == codepoint

#define PIX_CHAR_ARROW_UP 1  ARROW_DOWN 2  ARROW_LEFT 3  ARROW_RIGHT 4
#define PIX_CHAR_CHECK 5  CROSS 6  CIRCLE 7  SQUARE 8  TRIANGLE_UP 9  TRIANGLE_DOWN 10
// icon glyphs baked into slots 1..10, keyed by real Unicode codepoint (U+2191, U+2713, ...)
// via font_icon_entries[]; embed them in a UTF-8 string literal to render them.

struct glyph {
    vec4 crop;    // atlas pixel rect {x, y, w, h}, already grown by FONT_SDF_SPREAD
    vec2 offset;  // pen (baseline) -> crop top-left, y grows down
    float advance;
};

struct font_data {
    size_t width, height;      // atlas dimensions, == FONT_ATLAS_WIDTH/HEIGHT
    char* bitmap;               // 1 channel, 8-bit signed distance field (0.5 = outline)
    float ascent, descent, line_height, sdf_spread;
    idx atlas_texture;           // 0 until text.hpp lazily uploads it
    glyph glyphs[128];
};

font_data pix_load_font_ttf(const char* path);
void pix_free_font(font_data& font);          // frees bitmap only (atlas_texture is a GL object)

idx pix_font_slot(idx codepoint);
    // maps a Unicode codepoint to its glyphs[] index (0 if unsupported)
```

Shader-side decode of the SDF texel `v` (0..1, 0.5 = outline):
`alpha = smoothstep(0.5 - w, 0.5 + w, v)`, with `w` typically `fwidth(v)` for
resolution-independent antialiasing — see `FSHDER_TEXT`.

---

## sprite.hpp

Instanced 2D quad batches. **A `sprite_batch` is a baked GPU resource**:
`push_sprite`/`replace_sprites` write straight into its VBO via `glBufferSubData`;
`draw_sprites` never touches the buffer's contents, it only binds textures and
issues one instanced draw over whatever is already resident.

```cpp
#define MAX_SPRITES 1024
#define MAX_SPRITE_TEXTURES 32   // distinct textures one batch can hold; must match the
                                  // uTex[]/uTexSize[] array sizes in shader_sources.hpp

struct sprite {
    vec4 box;    // x, y, w, h, in the space of the view_proj passed to draw_sprites
    vec4 crop;   // atlas pixel rect x, y, w, h
    idx texture; // index into sprite_batch::tex_ids (see pix_batch_texture) - NOT a raw GL id
};

struct sprite_batch {
    idx vao, vbo;
    size_t size, capacity;
    idx tex_ids[MAX_SPRITE_TEXTURES]; size_t tex_count;
};

sprite_batch pix_create_sprite_batch(size_t capacity = MAX_SPRITES);
void pix_destroy_sprite_batch(sprite_batch& b);   // no-op today (nothing heap-allocated)

idx pix_batch_texture(sprite_batch& b, idx gl_texture);
    // registers/deduplicates a GL texture in the batch, returns its sprite::texture index

void push_sprite(sprite_batch& b, const sprite& s);              // appends, GPU write immediately
void replace_sprites(sprite_batch& b, const sprite* s, size_t count = 1, size_t offset = 0);
void clear_sprites(sprite_batch& b);                              // just resets size to 0

void draw_sprites(const sprite_batch& b, idx shader, const mat4& view_proj);
    // binds tex_ids[0..tex_count) to units 0..tex_count-1, sets uTex/uTexSize,
    // then ONE glDrawArraysInstanced covering the whole batch - the shader
    // picks each instance's sampler via its texture index
```

Usage pattern:

```cpp
sprite_batch b = pix_create_sprite_batch(64);
idx slot = pix_batch_texture(b, some_gl_texture);
sprite s = {}; s.box = {x,y,w,h}; s.crop = {u,v,cw,ch}; s.texture = slot;
push_sprite(b, s);
// ... later, every frame:
draw_sprites(b, sprite_shader, ortho_view_proj);
```

---

## text.hpp

Sprite-batch text built from a `font_data`'s SDF atlas.

```cpp
struct pixi_text { font_data* font; sprite_batch batch; vec4 color; };

pixi_text pix_create_text(font_data* font, const char* text, vec2 position,
                           vec4 color = {1,1,1,1});
    // `text` is UTF-8; `position` is the baseline of the first character.
    // '\n' advances by font->line_height and resets x. Lazily uploads the
    // font's atlas texture on first use (cached on font_data::atlas_texture).
void pix_destroy_text(pixi_text& text);

void draw_text(const pixi_text& text, idx shader, const mat4& view_proj);
    // sets uColor, then delegates to draw_sprites(text.batch, shader, view_proj)
```

A `pixi_text` is baked at creation time, same as a `sprite_batch` — changing the
string means creating a new one (or calling `replace_sprites`/`clear_sprites`
directly on `text.batch` yourself).

---

## Typical frame (3D + 2D overlay)

```cpp
pix_window window = pix_create_window("pix", W, H);
opengl_load_functions();

idx mesh_shader = opengl_create_shader(VSHDER_BASIC, FSHDER_BASIC);
idx text_shader = opengl_create_shader(VSHDER_TEXT, FSHDER_TEXT);
pix_renderer renderer = pix_create_renderer(W, H, mesh_shader);

font_data font = pix_load_font_ttf("C:/Windows/Fonts/arial.ttf");
pixi_text fps_label = pix_create_text(&font, "0 fps", {10, 24});

mat4 ui_proj = mat4_ortho(0, (float)W, (float)H, 0, -1, 1);

while (!window.should_close) {
    pix_update_window(window);

    begin_frame(renderer, cam);
    // push_instance(...) for everything in the scene
    end_frame(renderer);                       // 3D pass, depth test on

    draw_text(fps_label, text_shader, ui_proj); // 2D overlay, depth test off, blending on
}
```
