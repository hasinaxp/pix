# pix API reference

Header-only C-style engine, no OOP, no third-party dependencies (only `windows.h` +
`GL/gl.h` + libc). The headers hold their own implementations; `src/main.cpp` is the
only translation unit. Windows only, OpenGL 4.4 core profile.

```
src/core/     dtype math random platform opengl_api opengl_utils shader_sources
              lighting framebuffer renderer animation font sprite text
              colliders physics jobs profile
              platform_audio sound
src/loader/   asset_types png_loader png_writer obj_loader gltf_loader
              wav_loader data_loader
src/demos/    city_demo animation_demo
```

Include order matters — later headers assume earlier ones are already visible:

```
core/platform -> core/opengl_api -> core/opengl_utils -> core/shader_sources
              -> core/math -> core/lighting -> core/framebuffer
              -> loader/data_loader -> core/renderer
              -> core/font -> core/sprite -> core/text

core/math -> core/random
core/math -> core/colliders -> core/jobs -> core/physics
```

Nothing in the simulation half (`colliders`, `physics`, `jobs`, `random`,
`profile`) touches GL, and nothing in the rendering half touches the simulation.
A game that only wants one of them pays for one of them.

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

Free functions on `vec2`/`vec3`/`vec4`/`quat`/`mat4`. No operator overloading.
`mat4_mul` has an SSE2 path (guarded by `PIX_MATH_SSE`, unaligned loads, chosen
because a character rig is dozens of matrix products deep); everything else is
three or four floats wide, where shuffling costs more than it saves.

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

vec2 v2(float x, float y);
vec2 v2xz(vec3);                               // the horizontal part of a vec3
vec2 v2add(vec2, vec2);  vec2 v2sub(vec2, vec2);  vec2 v2scale(vec2, float);
float v2dot(vec2, vec2); float v2len(vec2);
float v3len(vec3);

quat quat_identity();
quat quat_norm(quat);
quat quat_mul(quat a, quat b);
quat quat_slerp(quat a, quat b, float t);      // shortest arc, nlerp for tiny arcs

mat4 mat4_identity();
mat4 mat4_mul(mat4 a, mat4 b);                 // a * b
mat4 mat4_translate(float x, float y, float z);
mat4 mat4_scale(float x, float y, float z);
mat4 mat4_rotate_x(float radians);
mat4 mat4_rotate_y(float radians);
mat4 mat4_rotate_z(float radians);
mat4 mat4_perspective(float fovy, float aspect, float near, float far);
mat4 mat4_ortho(float left, float right, float bottom, float top, float near, float far);
    // maps [left,right]x[bottom,top] to NDC; pass top=0,bottom=height for
    // y-down screen space (the convention sprite.hpp/text.hpp use)
mat4 mat4_lookat(vec3 eye, vec3 center, vec3 up);

mat4 mat4_from_quat(quat);
mat4 mat4_from_trs(vec3 t, quat r, vec3 s);    // T * R * S, built directly
void mat4_decompose(const mat4&, vec3* t, quat* r, vec3* s); // assumes no shear
mat4 mat4_inverse(const mat4&);                // cofactor expansion; identity if singular
vec3 mat4_mul_point(const mat4&, vec3);        // transforms a point, w assumed 1

// "place a prop" transforms: a position, a yaw about +Y and a scale, built
// directly rather than as three mat4_mul calls
mat4 mat4_trs_y (vec3 position, float yaw, float scale);
mat4 mat4_trs_y2(vec3 position, float yaw, float scale_xz, float scale_y);
mat4 mat4_trs_y3(vec3 position, float yaw, float sx, float sy, float sz);

float clampf(float v, float lo, float hi);
float lerpf(float a, float b, float t);
float angle_delta(float from, float to);       // shortest signed turn, (-pi, pi]
float damp(float current, float target, float rate, float dt);
float damp_angle(float current, float target, float rate, float dt);
    // framerate-independent exponential smoothing; damp_angle takes the short
    // way round so a heading never spins the long way to get a few degrees over
```

### frustum culling

```cpp
struct frustum { vec4 planes[6]; };            // world space, interior positive

frustum frustum_from_viewproj(const mat4&);    // Gribb/Hartmann row combinations
bool frustum_test_sphere(const frustum&, vec3 centre, float radius);
bool frustum_test_aabb(const frustum&, vec3 min_corner, vec3 max_corner);
    // positive-vertex test: one corner per plane decides it
```

`begin_frame` builds a frustum from the camera and `end_frame` culls against it,
so most callers never touch these directly. They are here for game-side culling —
skipping the AI, the animation sampling or the audio for things nobody can see.

---

## platform.hpp

Window + input, no GL calls of its own beyond context creation.

```cpp
struct pix_key_state { bool pressed, released, held; char code; };

#define MOUSE_BUTTON_LEFT  255
#define MOUSE_BUTTON_RIGHT 254
#define MOUSE_BUTTON_MID   253
#define KEY_UP 201  KEY_DOWN 202  KEY_LEFT 203  KEY_RIGHT 204  KEY_ESC 205  KEY_TAB 206
#define KEY_LEFT_BRACKET 207  KEY_RIGHT_BRACKET 208
#define KEY_SHIFT 16  KEY_CTRL 17  KEY_SPACE 32  KEY_RETURN 0x0D
#define KEY_F1 0x70 ... KEY_F12 0x7B    // function keys keep their VK codes

struct pix_window {
    HWND handle; HDC dc; void* gl_context;
    pix_key_state keystates[256];  // ascii-indexed; mouse/special keys use the codes above
    int mouse_x, mouse_y, mouse_rel_x, mouse_rel_y;
    int mouse_wheel;              // notches this frame, +1 per detent forward
    bool mouse_captured;
    bool should_close;
    pix_gamepad gamepads[PIX_MAX_GAMEPADS];   // PIX_MAX_GAMEPADS == 4
};

pix_window pix_create_window(const char* title, int width, int height);
    // creates a 4.4 core-profile context, shows the window, starts vsync off
void pix_update_window(pix_window& window);
    // swaps buffers, refreshes pressed/released/held, pumps messages, polls pads

void pix_set_mouse_capture(pix_window&, bool captured);
    // hides the cursor and re-centres it after every frame, so mouse_rel keeps
    // accumulating without the pointer ever reaching a screen edge — what a
    // mouselook camera needs. The re-centring happens last in
    // pix_update_window, so the warp itself never shows up as motion.
void pix_set_vsync(pix_window&, bool enabled);
    // off is the default: vsync pins a scene that would run at 150fps to a flat
    // 60 and it reads exactly like a real bottleneck. Turn it on to ship.
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
// a named `g`/`o` run inside one OBJ: a contiguous slice of the emitted
// vertices plus the centre of its own bounds, so a part authored in place (a
// wheel, a turret, a door) can be pulled out and rotated about itself
struct mesh_group { char name[48]; size_t vertex_offset, vertex_count; vec3 pivot; };

struct mesh_file_data {
    size_t vertex_count; vertex* vertex_data;
    size_t index_count;  uint16_t* index_data;
    size_t group_count;  mesh_group* groups;
    vec3 bounds_min, bounds_max;    // model space, before any placement scale
};
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
idx load_mesh_obj_group(pix_data_loader&, idx mesh_file, const char* group, vec3* out_pivot);
    // splits a named part out of an already-loaded model into a mesh of its own,
    // centred on its own pivot, so it can be animated apart from the body
idx load_image_file(pix_data_loader&, const char* filepath, size_t channels);
idx load_sound_wav_file(pix_data_loader&, const char* filepath);
    // decodes into loader.sound_files; hand the result to sound.hpp's load_sound
idx load_model_gltf_file(pix_data_loader&, const char* filepath, int only_node = -1,
                         int only_material = -1,
                         const int* skip_nodes = 0, int skip_count = 0);
    // fills the skeleton / skinned mesh / animation tables in one call;
    // returns an index into model_files, or (idx)-1 on failure.
    // See gltf_loader.hpp for what the three narrowing arguments are for.

skinned_mesh_file_data*   get_model_mesh(pix_data_loader&, idx model);
skeleton_file_data*       get_model_skeleton(pix_data_loader&, idx model);
animation_clip_file_data* find_animation(pix_data_loader&, idx model, const char* name);
    // name == nullptr returns the model's first clip
```

Everything allocated lives in `loader.arena` and stays valid until it is reset or
freed. `renderer.hpp` copies what it needs onto the GPU immediately.

### transient loads

```cpp
struct pix_loader_mark { size_t arena, mesh_files, image_files, skeletons,
                         skinned_meshes, clips, model_files; };

pix_loader_mark pix_loader_mark_now(const pix_data_loader&);
void            pix_loader_rewind(pix_data_loader&, const pix_loader_mark&);
```

A static prop's file is parsed into the arena, uploaded to the GPU, and never
looked at again — the catalogue keeps a mesh handle, a material handle and a
bounding box, not one byte of the file. Keeping all of it alive anyway is what
fills a large arena with a couple of hundred props, since each load copies the
whole binary chunk plus a JSON token table, and a library file holding five props
is parsed five times over.

So mark before the load and rewind after: the bump pointer goes back, and so do
the table counts, because a `model_file` entry left behind would point at vertex
data the next load is about to overwrite.

**Anything whose CPU-side data is read again later** — a rig's skeleton, its
clips, the mesh a pose is measured against — must not be loaded this way.

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
bool obj_load_file(mem_arena&, const char* path, mesh_file_data* out,
                   obj_palette* palette = 0);
int  obj_find_group(const mesh_file_data&, const char* name);
bool obj_extract_group(mem_arena&, const mesh_file_data& src, size_t group,
                       mesh_file_data* out, vec3* out_pivot);
```

Positions/uvs/normals + polygon faces, fan triangulated, vertices emitted unshared.
Generates flat normals when the file has none, and records each `g`/`o` run as a
`mesh_group` plus the whole file's bounds.

`palette` collects the colours named by a companion `.mtl` for materials that
carry no texture, so one generated image can back every such model instead of one
material each.

`obj_extract_group` pulls a named part out into its own mesh, re-centred on its
own pivot — the pivot is taken from the part's bounds centre, where a moving
part's true axis sits far more reliably than its vertex average would.

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
    bool image_is_palette;            // sample it NEAREST, not LINEAR
    vec3 node_offset;                 // where a one-node load stood in its file
};

bool gltf_load_file(mem_arena&, const char* path, gltf_result* out,
                    int only_node = -1, int only_material = -1,
                    const int* skip_nodes = 0, int skip_count = 0);

// the distinct materials a file's primitives use, in first-seen order;
// `node` narrows it to one mesh node, or -1 for the whole file
int gltf_list_materials(mem_arena&, const char* path, int node, int* out, int max_out);

// every mesh-carrying node's name and index, for picking one out by name
struct gltf_node_info { char name[64]; int node; };
int gltf_list_nodes(mem_arena&, const char* path, gltf_node_info* out, int max_out);
```

The three narrowing arguments exist because "one file, one model" is not how asset
packs ship:

- `only_node` — load a single mesh node instead of merging every one in the file.
  A library file holding several unrelated props needs this; the usual merge is
  for one character split across several meshes. A one-node load is re-based on
  that node's own origin, with the translation it discarded kept in `node_offset`.
- `only_material` — restrict further to the primitives painted with one glTF
  material. A model built from two materials has two textures, and a merged
  single-texture draw can only wear one of them, so the second renders as
  scribble. Load such a file once per material and draw the results together.
- `skip_nodes` — the opposite of `only_node`: every mesh node *except* these.
  Taking a vehicle's shell without its wheels needs this, because there is often
  no single node that *is* the shell to ask for by index.

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
#define MAX_ANIMATOR_CLIPS 40    // a rig from an asset pack ships 20-30

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

// two clips at once, `weight` of the way from a to b
void animator_sample_blend(const animator&, idx a_clip, float a_time,
                           idx b_clip, float b_time, float weight, animation* out);

// convenience playback on top of it
void animator_play(animator&, idx clip, bool loop = true, float speed = 1.0f);
void animator_update(animator&, float dt);          // advances time, refills .pose

void  animation_rest_pose(const skeleton_file_data&, animation* out);
float animator_duration(const animator&, idx clip);
const char* animator_clip_name(const animator&, idx clip);

// ---- attaching a prop to a bone ----
int  animator_find_bone(const animator&, const char* const* names, int count);
    // first name that exists, because rigs disagree about what a wrist is called
mat4 animation_bone_world(const animator&, const animation& pose, int bone);
    // the bone's global transform in the character's own model space
```

`animator_sample_blend` blends each bone's local T/R/S **before** the hierarchy is
walked, never the finished skinning matrices — lerping two matrices that differ by
a large rotation shears and shrinks the limb between them, which between two poses
that are far apart is worse than the pop it was meant to hide. A weight at either
extreme falls through to the single-clip path, so it is free to call it always.

A pose holds `global(i) * inverse_bind(i)`, which is what the vertex shader wants
and is the wrong thing for placing a held prop. `animation_bone_world` multiplies
the bind pose back in to recover the bone's actual transform; it costs one 4x4
inverse, so it is a per-attachment call, not something the pose does for all 128
bones.

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
shared index buffer, one streamed per-frame instance buffer. On top of that sits
an optional multi-pass path — cascaded shadows, an analytic sky, water with a
planar reflection, particles, bloom and a tonemap/grade.

```cpp
struct mesh     { idx vbo, vertex_offset, vertex_size, vertex_count, index_offset, index_count; };

struct material {
    idx shader, texture;
    vec3 color; float metallic, roughness;
    vec3 emissive;       // linear radiance the surface gives off; what bloom serves
    float glass;         // how far the texture's dark texels go toward a mirror
    vec3 window_glow;    // emission weighted toward the dark texels
};

struct pix_render_instance {
    idx mesh, material;
    mat4 transform;
    uint32_t dist_bucket;   // filled in by the sort; PIX_DIST_BUCKET metre steps
};

struct camera { vec3 position, direction, up; };
```

Pool sizes (all compile-time, all flat arrays inside `pix_renderer`):

```
MAX_RENDERABLE 24576   MAX_ANIMATED 96      MAX_MESHES 512
MAX_MATERIALS  768     MAX_SHADERS  32      MAX_PARTICLES 4096
MAX_WATER_INSTANCES 12288
MESH_POOL_VERTICES/INDICES (1<<21)          SKIN_POOL_VERTICES (1<<18) / INDICES (1<<19)
```

Running out of a mesh pool is silent from the outside — meshes loaded after it
fills simply do not exist — so anything loading a large catalogue should report
how full it is. `load_material` and `load_material_image` bounds-check instead of
overrunning, because an overrun there corrupts whatever the compiler placed after
`materials[]` and fails several frames later somewhere else entirely.

### loading

```cpp
pix_renderer pix_create_renderer(int width, int height, idx shader,
                                  vec3 clear_color = { 0.0f, 0.5f, 0.8f });
void pix_destory_renderer(pix_renderer&);   // [sic]

idx load_mesh(pix_renderer&, mesh_file_data&);
    // appends into models_vbo/ebo via glBufferSubData; records vertex/index offsets
idx load_skinned_mesh(pix_renderer&, skinned_mesh_file_data&);
idx load_material(pix_renderer&, const char* texture_path = nullptr,
                  vec3 color = {1,1,1}, float metallic = 0.1f, float roughness = 0.5f);
idx load_material_image(pix_renderer&, image_file_data*, vec3 color = {1,1,1},
                  float metallic = 0.1f, float roughness = 0.5f, bool pixelated = false);
    // for an already-decoded texture, e.g. one embedded in a .glb. `pixelated`
    // is for generated colour palettes, where a linear filter blends one
    // material's cell into the next
idx clone_material(pix_renderer&, idx source, vec3 color, float metallic, float roughness);
    // a second material over the *same* GL texture — per-instance tints without
    // decoding and uploading the image again
idx get_default_material(pix_renderer&);     // white, untextured
idx load_shader(pix_renderer&, const char* vsrc, const char* fsrc);
```

### the frame

```cpp
void begin_frame(pix_renderer&, camera&);
    // rebuilds view/projection, extracts the frustum, clears the instance lists
    // and the light list, fits the shadow cascades

void push_instance(pix_renderer&, pix_render_instance&);
void push_instance(pix_renderer&, idx mesh, idx material, const mat4& transform);
void push_animated_instance(pix_renderer&, pix_render_instance&, const animation& pose);
    // `pose` is referenced, not copied — it must stay alive until end_frame
    // (an animator's .pose does)
void push_water(pix_renderer&, idx mesh, idx material, const mat4& transform);
void push_particle(pix_renderer&, vec3 at, float size, vec3 color, float alpha);
    // `size` is a half-width in metres; `color` is linear radiance, deliberately
    // unclamped so a bright spark reaches the bloom threshold
void push_light(pix_renderer&, const pix_light&);

void end_frame(pix_renderer&, bool clear_instances = true);
```

Lights are submitted per frame the way draw calls are, and a light that stops
being submitted stops existing — which is what makes a row of lamps fading up at
dusk one line of gameplay code rather than a resource to manage. `end_frame`
keeps the `MAX_SHADER_LIGHTS` nearest the camera (see `lighting.hpp`).

`end_frame` sorts the opaque list by (mesh, material) and then near-to-far inside
each run, uploads the whole frame's transforms in **one** `glBufferSubData`, and
issues one `glDrawElementsInstancedBaseVertexBaseInstance` per batch. The
near-to-far ordering is what lets the shadow and reflection passes take a near
slice of a batch without compacting a second list for it.

The animated path is one draw per instance with its own `uBones[]`, because every
instance needs different bone matrices. Static meshes use the `vertex` layout
(pos/normal/uv, per-instance mat4 at locations 3..6); skinned meshes use
`vertex_rigged` (pos/normal/uv/bone_ids/bone_weights at 0..4) in their own VAO.

### effects

```cpp
void pix_enable_effects(pix_renderer&, int shadow_resolution = 2048);
    // allocates the HDR scene target, the shadow atlas and the bloom chain, and
    // switches end_frame to the multi-pass path
void pix_set_time(pix_renderer&, float seconds);        // drives waves, rain, foliage sway
void pix_set_time_of_day(pix_renderer&, float hour);    // sun, sky, fog, night factor
void pix_set_weather(pix_renderer&, const pix_weather&);// cloud deck, rain, ground wetness
```

**Everything above is opt-in.** Skip `pix_enable_effects` and `end_frame` degrades
to the plain forward draw, with no framebuffers allocated and no extra passes.
With it on, a frame is: shadow cascades → (reflection, if water was pushed) →
sky → opaque → water → particles → bloom → post.

Tunables live as plain fields on `pix_renderer`, all safe to write between
frames: `exposure`, `saturation`, `contrast`, `split_tone`, `vignette`,
`sharpen`, `bloom_threshold`, `bloom_knee`, `bloom_strength`, `shadow_strength`
(0 turns shadows off without unbinding anything), `shadow_extent`, `shadow_depth`.

### measuring it

```cpp
void   pix_enable_gpu_timing(pix_renderer&, bool on);
double pix_gpu_pass_ms(const pix_renderer&, int pass);   // PIX_GPU_PASS_NAMES[]
```

`GL_TIME_ELAPSED` around each pass. Off by default: the queries are cheap to keep
but the driver has to fence around them, and a measurement that changes what it
measures is only worth taking while somebody is reading it. Nothing measured on
the CPU can tell one pass from another — every GL call returns long before its
work does — so this is the only way to find out where a frame actually went.

Typical frame:

```cpp
begin_frame(renderer, cam);
for (...) push_instance(renderer, mesh, material, transform);
for (...) push_light(renderer, pix_point_light(at, 12.0f, colour));

animator_update(actor, dt);
push_animated_instance(renderer, actor_instance, actor.pose);

end_frame(renderer);
```

---

## lighting.hpp

Everything that lights the world, kept apart from the thing that draws it. Three
kinds of light, deliberately different things rather than one general case.

```cpp
#define MAX_SCENE_LIGHTS   512   // what a frame may collect before culling
#define MAX_SHADER_LIGHTS  32    // what one draw uploads and shades with
#define SHADOW_CASCADES    3

struct pix_light { vec3 position; float radius; vec3 color;
                   float cos_inner; vec3 direction; float cos_outer; };
struct pix_sun   { vec3 direction;   // normalised, pointing *toward* the sun
                   vec3 color; };    // linear radiance, well above 1 in daylight
struct pix_sky   { vec3 zenith, horizon, ground, diffuse_up, diffuse_down, fog;
                   float fog_density; };
struct pix_weather { float overcast, rain, wetness; };   // each 0..1

pix_light pix_point_light(vec3 position, float radius, vec3 color);
pix_light pix_spot_light (vec3 position, vec3 direction, float radius,
                          float inner, float outer, vec3 color);
```

- **sun** — one directional light, the only one that casts a shadow map.
- **sky** — an environment, not a light: hemisphere irradiance plus a dome colour
  every glossy surface reflects. Analytic, so the backdrop, a reflection in a
  glossy surface and a reflection in water are one function evaluated three times.
- **punctual** — point and spot lights, gathered per frame and culled to the ones
  nearest the camera. Stored the way the shader wants them, so uploading a frame
  is three `glUniform4fv` calls and no per-light work.

A light's falloff is *windowed* to reach exactly zero at `radius`, so a light
leaving the shader's 32 slots never pops.

### time of day and weather

```cpp
void  pix_daylight_at(float hour, pix_sun*, pix_sky*, float* exposure);
void  pix_weather_apply(const pix_weather&, pix_sun*, pix_sky*, float* exposure);
float pix_night_factor(const pix_sun&);   // 0 by day, 1 after dusk
```

Keyframed rather than derived from a physical sky model, so every hour looks
deliberately chosen and an hour in between is a straight interpolation of two
setups that were each picked to look right. The keys wrap, so 23:00 blends into
05:00 through the night key rather than racing backwards through noon. Exposure
is part of the hour, not a constant: a night scene carries a hundredth of the
light a noon scene does and no amount of adding lamps closes that gap.

`pix_weather_apply` runs *after* the hour is chosen and bends that setup toward
the cloud deck, so a new hour keyframe is automatically correct in the rain.
`pix_night_factor` is what artificial light should ride on — lamps fading in over
dusk instead of switching, and deliberately read off the *clear* sun elevation so
a dark overcast does not switch every lamp in the world on at noon.

---

## framebuffer.hpp

Offscreen render targets. Two shapes cover nearly everything:

```cpp
struct framebuffer { idx fbo, color, depth; int width, height; bool hdr; };

framebuffer pix_create_render_target(int w, int h, bool hdr = true, bool mipmapped = false);
framebuffer pix_create_shadow_map(int w, int h);
void pix_destroy_framebuffer(framebuffer&);

void pix_bind_framebuffer(const framebuffer&);   // also sets the viewport
void pix_bind_backbuffer(int width, int height);
void pix_draw_fullscreen(void);                  // no VBO; the VS builds it from gl_VertexID
```

Depth is always a texture, never a renderbuffer, because fog and any later depth
effect need to sample it. A shadow map's depth texture is set up for hardware
comparison sampling (`sampler2DShadow`), which with `GL_LINEAR` gives free 2x2
percentage-closer filtering, and clamps to a white border so anything outside the
light's view is lit rather than shadowed. Binding a target sets the viewport,
since forgetting that is the classic way to spend an hour wondering why half the
screen is black.

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

---

## random.hpp

Deterministic, seekable noise. Procedural generation needs the same world every
run from the same seed, and it needs to ask "what belongs at cell (x, z)?" out of
order — so alongside the streaming generator there is a stateless hash.

```cpp
struct rng { uint32_t state; };

rng   rng_seed(uint32_t seed);          // 0 is remapped; it is a xorshift fixed point
uint32_t rng_u32(rng&);
float rng_float(rng&);                  // [0, 1)
float rng_range(rng&, float lo, float hi);
int   rng_int(rng&, int lo, int hi);    // inclusive both ends
bool  rng_chance(rng&, float probability);

uint32_t hash_u32(uint32_t);
uint32_t hash2(int x, int y, uint32_t seed);
float    hash2_float(int x, int y, uint32_t seed);   // [0, 1) from a coordinate pair
rng      rng_at(int x, int y, uint32_t seed);        // a whole stream seeded from a cell
```

`hash2_float` is the workhorse for per-cell decisions; `rng_at` is for when one
cell needs a sequence of them rather than a single value.

---

## colliders.hpp

Collision shapes and the tests between them. Pure geometry — nothing here knows
about bodies, mass, velocity or time. `physics.hpp` is the layer that turns these
answers into motion; anything else needing a shape query (placement checks, camera
probes, triggers, editor tools) can use this header without the simulation.

Every test that reports an overlap returns the **minimum translation**: a unit
normal pointing from the first shape toward the second, and the depth to push them
apart along it. So every contact resolves the same two ways.

### 2D, in the XZ plane

The tests a broadly flat game resolves its motion with — cheaper, and they never
let a character drift off a floor it is standing on.

```cpp
struct aabb    { vec3 min, max; };
struct rect2   { vec2 min, max; };
struct circle2 { vec2 centre; float radius; };
struct obb2    { vec2 centre, half; float yaw; };   // yaw matches mat4_rotate_y
struct contact2 { vec2 normal; float depth; };

aabb  aabb_make(vec3 min, vec3 max);
aabb  aabb_from_centre(vec3 centre, vec3 half_extents);
aabb  aabb_from_obb(const obb2&, float base_y, float height);
vec3  aabb_centre(const aabb&);   vec3 aabb_half(const aabb&);
float aabb_height(const aabb&);   rect2 aabb_footprint(const aabb&);
bool  aabb_overlap(const aabb&, const aabb&);
bool  aabb_contains_xz(const aabb&, vec2 p);
bool  span_overlap(float a0, float ah, float b0, float bh);   // vertical bands

vec2 closest_point_rect(const rect2&, vec2 p);
vec2 closest_point_obb (const obb2&,  vec2 p);

bool collide_circle_circle(const circle2&, const circle2&, contact2*);
bool collide_circle_rect  (const circle2&, const rect2&,   contact2*);
bool collide_circle_obb   (const circle2&, const obb2&,    contact2*);
bool collide_obb_obb      (const obb2&,    const obb2&,    contact2*);   // SAT
bool collide_obb_rect     (const obb2&,    const rect2&,   contact2*);

bool ray_vs_aabb(vec3 origin, vec3 inv_dir, const aabb&, float max_distance, float* out_t);
    // `inv_dir` is the componentwise reciprocal of a *normalised* direction,
    // precomputed because a raycast tests one ray against many boxes
```

### 3D

For the queries a flattened test gets wrong rather than merely approximates — a
shot arcing over a wall, something standing on a sloped surface, a trigger volume
a player can jump out of the top of.

```cpp
struct sphere  { vec3 centre; float radius; };
struct capsule { vec3 a, b; float radius; };     // a segment swept by a sphere
struct plane   { vec3 normal; float distance; }; // dot(normal, p) == distance
struct contact3 { vec3 normal; float depth; };

sphere  sphere_make(vec3 centre, float radius);
capsule capsule_make(vec3 a, vec3 b, float radius);
capsule capsule_upright(vec3 base, float height, float radius);
    // the character case: standing on `base`, `height` tall overall. A height
    // under two radii cannot be a capsule and collapses to a sphere.
plane   plane_make(vec3 normal, float distance);
plane   plane_through(vec3 normal, vec3 point);

aabb  sphere_bounds(const sphere&);
aabb  capsule_bounds(const capsule&);
float plane_distance_to(const plane&, vec3 point);   // signed, positive in front

vec3 closest_point_segment(vec3 a, vec3 b, vec3 p);
vec3 closest_point_aabb(const aabb&, vec3 p);
vec3 closest_point_plane(const plane&, vec3 point);
void closest_points_segments(vec3 p1, vec3 q1, vec3 p2, vec3 q2, vec3* c1, vec3* c2);

bool collide_sphere_sphere  (const sphere&,  const sphere&,  contact3*);
bool collide_sphere_plane   (const sphere&,  const plane&,   contact3*);
bool collide_sphere_aabb    (const sphere&,  const aabb&,    contact3*);
bool collide_sphere_capsule (const sphere&,  const capsule&, contact3*);
bool collide_capsule_capsule(const capsule&, const capsule&, contact3*);
bool collide_capsule_plane  (const capsule&, const plane&,   contact3*);
bool collide_capsule_aabb   (const capsule&, const aabb&,    contact3*);

bool ray_vs_sphere (vec3 origin, vec3 dir, const sphere&,  float max_d, float* out_t);
bool ray_vs_plane  (vec3 origin, vec3 dir, const plane&,   float max_d,
                    bool two_sided, float* out_t);
bool ray_vs_capsule(vec3 origin, vec3 dir, const capsule&, float max_d, float* out_t);
```

Rays take a **normalised** direction and report the near hit distance, clamped to
0 when the ray starts inside. `ray_vs_plane`'s one-sided default is what a floor
wants: a ray travelling with the normal passes through rather than hitting the
underside.

A capsule is stored as a segment rather than centre/height/radius because that is
the form every test wants, and because it costs nothing to let one lie on its side
or lean. `collide_capsule_aabb` resolves the capsule as a sphere at the point on
its axis nearest the box — exact for a face or end-cap contact, which is nearly
every contact a character makes, and slightly generous on a steep edge hit.

---

## jobs.hpp

The only threading in the library, and deliberately the smallest thing that does
the job. No work stealing, no futures, no allocation, no job graph.

```cpp
#define PIX_MAX_WORKERS 15   // plus the calling thread, so up to 16 cores busy

typedef void (*pix_job_fn)(void* user, size_t begin, size_t end, int worker);

void pix_jobs_start(pix_jobs&, int workers = -1);   // -1 = one per hardware core
void pix_jobs_stop(pix_jobs&);
void pix_parallel_for(pix_jobs&, size_t count, pix_job_fn, void* user, size_t grain);
    // runs fn over [0, count) and returns once every item is done. `grain` is the
    // smallest slice a worker claims; below it the range is run inline.

void pix_task_start(pix_task&);                     // spins the side thread up
void pix_task_run(pix_task&, void (*fn)(void*), void* user);   // hand it work
void pix_task_wait(pix_task&);                      // block until that work is done
void pix_task_stop(pix_task&);
```

`worker` is 0 for the thread that called `pix_parallel_for` and 1..n for the pool,
so a job body that needs per-thread scratch (an rng, a scratch buffer) can index
it without a lock.

`pix_task` is kept apart from the pool because it *overlaps* with it rather than
feeding off it: the point is that it is still running while the main thread drives
a `pix_parallel_for` of its own. Sampling a few dozen skeletons alongside the
physics step is the shape it exists for.

Rules the caller has to keep, since nothing here can enforce them:

- a parallel body writes only to item `i`'s own storage
- anything that adds or removes an entity stays on the main thread — the pools
  are not thread safe and making them so would cost more than the work being split
- anything drawing from a shared rng takes the per-worker one instead

---

## physics.hpp

A small, allocation-light physics world: an immovable set of axis-aligned boxes
plus a population of upright cylinders (characters) and oriented boxes (vehicles,
crates) that push against them and each other.

It is deliberately **2.5D**. For a game played on broadly flat ground every
collision that matters resolves in the XZ plane, and Y is only gravity plus a
ground clamp. That buys a solver that is a couple of hundred lines, runs a
thousand bodies in well under a millisecond, and never tunnels at the speeds a
vehicle reaches. A game that needs true 3D contact wants a different solver, not
a taller version of this one.

```cpp
#define PHYS_MAX_BODIES  2048    #define PHYS_MAX_STATICS 32768
#define PHYS_CELL 10.0f          // metres per broadphase cell
#define PHYS_CYLINDER 0          // upright; radius wide, height tall, origin at the feet
#define PHYS_BOX      1          // upright box; `half` extents in local XZ, rotated by yaw

// a body tests a candidate only when (a.collides & b.group) || (b.collides & a.group)
#define PHYS_LAYER_0..PHYS_LAYER_7        // the engine gives the bits no meaning
#define PHYS_LAYER_PLAYER    PHYS_LAYER_0 // names most games end up wanting
#define PHYS_LAYER_CHARACTER PHYS_LAYER_1
#define PHYS_LAYER_VEHICLE   PHYS_LAYER_2
#define PHYS_LAYER_PROP      PHYS_LAYER_3
#define PHYS_LAYER_ALL       0xFFFFu

struct phys_body {
    vec3 position, velocity;     // position is the XZ centre, Y at the feet
    float yaw, radius; vec2 half; float height;
    float inv_mass;              // 0 = immovable
    float restitution, drag, friction;
    uint8_t shape;
    bool active, gravity, on_ground, touched;
    float impact;                // largest normal impulse this step, in m/s
    uint16_t group, collides;
    void* user;                  // the game-side owner, so a hit traces back
};

void phys_create_world(phys_world&);
void phys_destroy_world(phys_world&);

void phys_add_static_box(phys_world&, vec3 min, vec3 max);
void phys_add_static_prop(phys_world&, vec3 position, float yaw, vec2 half, float height);
void phys_build_statics(phys_world&);    // buckets them into the uniform grid
void phys_reset_statics(phys_world&);    // drops the statics, keeps the bodies

idx        phys_add_body(phys_world&, const phys_body&);
phys_body* phys_get_body(phys_world&, idx);
void       phys_remove_body(phys_world&, idx);

void phys_step(phys_world&, float dt);
```

### queries

```cpp
bool   phys_raycast(const phys_world&, vec3 origin, vec3 direction,
                    float max_distance, float* out_distance);          // statics only
idx    phys_raycast_bodies(const phys_world&, vec3 origin, vec3 direction,
                    float max_distance, uint16_t mask, idx ignore, float* out_distance);
size_t phys_overlap_bodies(const phys_world&, vec3 centre, float radius,
                    idx* out, size_t capacity);                        // linear scan
size_t phys_query_neighbours(const phys_world&, vec3 centre, float radius,
                    idx* out, size_t capacity);                        // via the hash grid
bool   phys_point_blocked(const phys_world&, vec3 position, float radius, float height);
```

A projectile needs both raycasts and needs to know which came first, so it asks
each in turn and keeps the nearer hit. `phys_query_neighbours` reads the dynamic
grid `phys_step` rebuilt, so game code running between steps can ask it instead of
walking every body.

### uneven ground

```cpp
float (*ground_at)(void* user, float x, float z);   // field on phys_world
void*  ground_user;
```

One `ground_y` is right for a single flat plane and wrong for a ledge, a slope or
a bridge deck. Set this and bodies land on whatever height it returns. **It is
called from the threaded integration phase, so it must only read** — no lazily
built caches, no allocation, no writing back into the terrain it reads.
`step_height` is the separate allowance for stepping *up* onto something short
rather than being stopped by it.

### threading

Hand the world a `pix_jobs*` and two of the three phases split across it. Which
two is decided by what each phase writes, not by what it costs:

| phase | writes | threaded |
|---|---|---|
| integration | body `i` writes body `i` | yes |
| static contacts | body `i`, reading a grid nothing mutates | yes — and the expensive one |
| dynamic contacts | resolving a pair writes **both** bodies | no |

Splitting only what is provably independent is why there is not a lock anywhere in
the step.

---

## profile.hpp

Where the milliseconds went, per section, averaged over a second. It exists
because the answer is never the one anybody guesses.

```cpp
#define PIX_PROF_MAX_SECTIONS 16

void pix_profiler_init(pix_profiler&, const char* const* names, int count);
bool pix_profile_frame(pix_profiler&);   // call once per frame; true on a print frame

PIX_PROFILE(prof, SECTION_ENUM);         // scoped; an early return cannot leak it
```

Off unless `PIX_PROF` is set in the environment. When it is off the scopes still
call `QueryPerformanceCounter`, which is tens of nanoseconds against sections
measured in whole milliseconds and cheaper than the branch mispredicts guarding
every one of them would cost. It prints one line a second, not one a frame:
per-frame numbers for a section costing 0.4 ms are noise, and a second of them is
a measurement. Sections are a plain caller-side enum ending in a count, so this
file has no idea what a "sim" or a "cull" is.

---

## loader/png_writer.hpp

Writes an RGBA8 buffer out as a PNG — stored (uncompressed) deflate blocks plus
the CRC and Adler checksums, since the point is a screenshot or a debug dump, not
a small file.

```cpp
bool png_write_file(const char* path, int width, int height, const unsigned char* rgba);
```

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
