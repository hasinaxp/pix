// pix - the 3D renderer.
//
//   pix_renderer r = pix_create_renderer(width, height, 0, v3(0.2f, 0.2f, 0.25f));
//   pix_enable_effects(r);                            // shadows + bloom (optional)
//   idx cube = pix_mesh_cube(r, v3(1, 1, 1));
//   idx mat  = load_material(r, "crate.png");
//   ...
//   begin_frame(r, cam);
//   push_instance(r, cube, mat, mat4_translate(0, 1, 0));
//   push_light(r, pix_point_light(v3(0, 3, 0), 10, v3(4, 3, 2)));
//   end_frame(r);
//
// How a frame is built, and why it is cheap:
//
//   * Every static mesh lives in ONE shared vertex buffer and ONE index buffer.
//     A mesh is an offset into them, so the whole scene draws from one VAO.
//   * Instances are submitted every frame. end_frame frustum-culls them, sorts
//     them by (material, mesh) with a radix sort, uploads the whole frame's
//     transforms in one go per list, and issues one instanced draw per run -
//     each draw naming its own slice with baseInstance, so nothing is copied
//     twice and no upload waits on a draw.
//   * The sun's shadow map is three cascades in one atlas; each cascade gets
//     its own culled caster list, so the near cascade costs a handful of draws
//     rather than the whole world.
//   * Opaque draws go front to back within a batch (early-z), blended ones are
//     sorted back to front.
//
// All storage is heap allocated and every GPU buffer grows on demand, so the
// struct is small, safe to return by value, and costs memory in proportion to
// what the game actually uses.
#ifndef PIX_RENDERER_HPP
#define PIX_RENDERER_HPP
#include "dtype.hpp"
#include "math.hpp"
#include "opengl_utils.hpp"
#include "texture.hpp"
#include "lighting.hpp"
#include "framebuffer.hpp"
#include "shader_sources.hpp"
#include "compute.hpp"
#include "animation.hpp"
#include "../loader/asset_types.hpp"

// Capacities. Define any of these before including pix to change them; they
// size heap allocations, not the struct.
#ifndef PIX_MAX_INSTANCES
#define PIX_MAX_INSTANCES  65536   // static instances submitted per frame
#endif
#ifndef PIX_MAX_ANIMATED
#define PIX_MAX_ANIMATED   8192    // skinned instances per frame (they draw instanced)
#endif
#ifndef PIX_MAX_MESHES
#define PIX_MAX_MESHES     8192
#endif
#ifndef PIX_MAX_MATERIALS
#define PIX_MAX_MATERIALS  8192
#endif
#ifndef PIX_MAX_PARTICLES
#define PIX_MAX_PARTICLES  16384
#endif
#ifndef PIX_MAX_LINES
#define PIX_MAX_LINES      32768
#endif
#define MAX_SHADERS        64

// kept for code written against the older fixed tables
#define MAX_RENDERABLE PIX_MAX_INSTANCES
#define MAX_MESHES     PIX_MAX_MESHES
#define MAX_MATERIALS  PIX_MAX_MATERIALS
#define MAX_ANIMATED   PIX_MAX_ANIMATED
#define MAX_PARTICLES  PIX_MAX_PARTICLES

struct mesh {
    idx vbo;             // the shared pool buffer it lives in (informational)
    idx vertex_offset;   // first vertex in the shared buffer
    idx vertex_size;     // stride, bytes
    idx vertex_count;
    idx index_offset;    // first index in the shared buffer
    idx index_count;
    // model-space bounds, for culling and for fitting models to the world
    vec3 bounds_min;
    vec3 bounds_max;
    vec3 center;
    float radius;
};

// A custom uniform on a material: its location in the material's program and
// the value to send whenever that material is bound.
#define PIX_MAX_MATERIAL_UNIFORMS 6
struct pix_material_uniform {
    GLint location;
    int   components;   // 1..4 floats, or -1 for an int
    float value[4];
};

// material.blend
// (PIX_BLEND_* live in dtype.hpp, shared with 2D; blended materials are
// sorted back to front)

struct material {
    idx shader;          // 0 = the built-in PBR surface; else see load_surface_shader
    idx texture;         // GL texture, 0 = plain white
    vec3 color;          // sRGB multiplier on the texture
    float metallic;
    float roughness;
    vec3 emissive;       // linear radiance the surface gives off by itself
    float alpha;         // opacity multiplier, used by blended materials
    // Opaque materials discard texels whose alpha is below this - foliage,
    // fences, grates. 0 ignores texture alpha entirely.
    float alpha_cutoff;
    // The texture really has holes (leaves, grass cards, fences). Such
    // surfaces lay down their depth first in a cheap pass of their own, so the
    // lit pass shades each pixel once instead of every card stacked behind it
    // (see r.cutout_prepass). Leave false for textures that are solid.
    bool cutout;
    uint8_t blend;       // PIX_BLEND_*
    bool double_sided;   // draw back faces too (leaves, cloth, single planes)
    bool unlit;          // ignore lighting: UI in world space, skyboxes, emissive signs
    bool cast_shadows;
    // Casts a shadow but is never seen: a coarse stand-in for something too
    // detailed to be worth drawing into every cascade (give the detailed one
    // cast_shadows = false).
    bool shadow_only;
    // 0..1: how much of the planar mirror (r.reflections.planar) this surface
    // shows - 1 with metallic 1 and roughness ~0 is a perfect mirror, 1 on a
    // dielectric gives the Fresnel reflection of a polished floor or water.
    float planar_reflection;
    // > 0: the surface is see-through and bends what is behind it by this much
    // (0.02 - 0.1 is glass to thick water). lpha then means opacity of the
    // surface itself over the refracted view. Drawn in the transparent pass.
    float refraction;
    // Free for custom surface shaders (uParams0 / uParams1). The built-in water
    // uses them - see pix_material_water.
    vec4 params0;
    vec4 params1;

    // ---- stylised shading, mixed with PBR ----
    // toon 0 = physically based, 1 = cel shaded, anything between blends the
    // two - e.g. 0.6 keeps PBR's reflections under cartoon lighting bands.
    float toon;
    float toon_bands;       // light steps, 2..5 (default 3)
    float rim;              // rim light strength, 0 = off
    float toon_specular;    // size of the hard highlight, 0 = none, ~0.3 typical

    // the program skinned meshes use with this material (0 = built-in; see
    // load_skinned_surface_shader)
    idx skinned_shader;

    // extra textures for custom shaders: uTex1..uTex4 (0 = unbound)
    idx textures[4];
    // named uniforms for custom shaders, set with pix_material_set_*
    pix_material_uniform uniforms[PIX_MAX_MATERIAL_UNIFORMS];
    int uniform_count;
};

struct pix_render_instance {
    idx  mesh;
    idx  material;
    mat4 transform;
    vec4 tint = { 1.0f, 1.0f, 1.0f, 1.0f };   // per-instance colour multiplier, a = opacity
    // Which views draw it: a view renders instances with (layers & view.layers) != 0.
    // Bit 0 is the default world layer; the rest are yours (first-person arms,
    // minimap icons, reflection-only proxies, ...).
    uint32_t layers = 1;
};

// What one particle is once it has stopped being simulation: a point, a size
// and a colour, laid out to be uploaded as-is.
struct pix_particle_vertex {
    float centre[3];
    float size;        // half-width in world units
    float color[4];    // rgb linear radiance, a coverage
};

struct camera {
    vec3 position;
    vec3 direction;
    vec3 up;
    // 0 picks the renderer's defaults (about 60 degrees, 0.1, 1000)
    float fovy;          // radians, vertical
    float near_plane;
    float far_plane;
    // > 0 switches to an orthographic projection this many units tall
    float ortho_height;
};

// CPU mirror of GLSL_SCENE. All vec4/mat4 so std140 needs no hand padding.
struct pix_scene_block {
    float view_proj[16];
    float view[16];
    float sun_dir[4];
    float sun_color[4];
    float sky_color[4];
    float ground_color[4];
    float sky_zenith[4];
    float sky_horizon[4];
    float sky_ground[4];
    float fog[4];
    float camera[4];
    float camera_dir[4];
    float shadow[4];
    float light_space[SHADOW_CASCADES][16];
    float cascade_far[4];
    float cascade_texel[4];
    float refl_view_proj[16];
    float clip_plane[4];
    float reflect[4];
    float screen[4];
    float shadow_eye[4];
    float shadow_dir[4];
};

struct pix_light_block {
    float pos_radius[MAX_SHADER_LIGHTS * 4];
    float color_inner[MAX_SHADER_LIGHTS * 4];
    float dir_outer[MAX_SHADER_LIGHTS * 4];
    float meta[4];
};

// The pipeline stages, timed separately; what pix_gpu_pass_ms indexes.
enum {
    PIX_GPU_SHADOW,     // pix_render_shadows
    PIX_GPU_REFLECT,    // pix_render_reflections (probe faces + mirror)
    PIX_GPU_VIEW,       // pix_render_view (the last one of the frame)
    PIX_GPU_POST,       // pix_post_process
    PIX_GPU_PASSES
};
static const char* const PIX_GPU_PASS_NAMES[PIX_GPU_PASSES] = { "shadow", "reflect", "view", "post" };

// ---- shadows and post processing ----
//
// Both are plain data living on the renderer (r.shadows, r.post). Change any
// field whenever you like - the renderer compares against what it has
// allocated at the next frame and rebuilds only what changed, so turning
// bloom off frees its targets, and raising the shadow resolution reallocates
// the atlas once. Nothing here costs anything per frame beyond the passes it
// switches on. pix_set_quality fills both from a preset.
struct pix_shadow_settings {
    bool  enabled;
    int   resolution;    // texels per cascade side; the atlas holds 3 cascades
    float distance;      // shadows reach this far from the camera, world units
    float strength;      // 0..1, how dark a fully shadowed surface gets
    float depth_range;   // how far toward the sun casters are gathered
    int   softness;      // PCF radius: 0 = hardware 2x2, 1 = 3x3, 2 = 5x5 (softest, costliest)
    bool  skinned;       // animated characters cast shadows
    int   skinned_cascades; // ...into this many cascades from the near one (1..3): small
                            // bodies far off cast shadows too small to see
    float far_resolution; // cascades past the first render at this fraction of
                          // `resolution` (0.25..1): they cover more ground at coarser
                          // detail anyway, and fill is what shadows cost
    bool  stagger;       // the near cascade every frame, the two far ones on alternate
                         // frames: about a third less shadow work, and what moves far
                         // off updates its shadow at half rate
};

#define PIX_TONEMAP_ACES     0   // filmic, rolls highlights off - the default
#define PIX_TONEMAP_REINHARD 1   // softer, less contrast
#define PIX_TONEMAP_NONE     2   // clamp; for stylised/unlit games that want exact colours

struct pix_post_settings {
    float exposure;        // multiplier on the time-of-day exposure (1 = as keyed)
    int   tonemap;         // PIX_TONEMAP_*
    bool  bloom;           // glow around anything brighter than bloom_threshold
    float bloom_strength;
    float bloom_threshold; // linear radiance where glow starts; emissive > 1 always blooms
    float bloom_knee;      // soft transition width around the threshold
    float saturation;      // 1 = unchanged, 0 = greyscale
    float contrast;        // 0 = unchanged, ~0.3 = punchy
    float vignette;        // 0 = off
    float sharpen;         // 0 = off, ~0.3 subtle
    vec3  color_filter;    // multiplies the final image; (1,1,1) = none
    bool  fxaa;            // cheap edge anti-aliasing
    bool  dither;          // breaks up banding in smooth gradients (skies, fog)
};

// A user fullscreen pass run after tonemapping, in the order added.
#define PIX_MAX_POST_EFFECTS 8
struct pix_post_effect {
    idx  program;
    bool enabled;
    vec4 params;           // uParams in the shader, free for the effect to use
    GLint u_scene, u_depth, u_texel, u_params, u_near_far;
};

// Reflections beyond the analytic sky. Like the other settings: plain data,
// applied at the next frame, resources made and freed on demand.
struct pix_reflection_settings {
    // A cubemap of the scene around a point, re-rendered a few faces per frame
    // and mip-filtered, so every glossy surface reflects the real world instead
    // of just the sky, and ambient light picks up the colour around it.
    bool  probe;
    int   probe_resolution;      // per face; 64..512 (128 is plenty for blurred reflections)
    int   probe_faces_per_frame; // 1..6: 1 spreads the cost over six frames
    bool  probe_follow_camera;   // else it sits at probe_position
    vec3  probe_position;
    float probe_distance;        // how far the probe sees
    float probe_intensity;
    // An exact mirror across one plane, for water, mirrors and polished floors.
    // Only surfaces with material.planar_reflection > 0 use it.
    bool  planar;
    vec4  plane;                 // xyz unit normal, w = -dot(normal, point on plane)
    float planar_scale;          // resolution relative to the screen (0.5 = half)
    float planar_distance;       // how far the mirror camera draws
};

enum { PIX_QUALITY_LOW, PIX_QUALITY_MEDIUM, PIX_QUALITY_HIGH, PIX_QUALITY_ULTRA };

// A growable pair of vertex + index buffers with one VAO over them.
struct pix__geometry_pool {
    GLuint vao, vbo, ebo;
    size_t vertex_size;
    size_t vertex_capacity, index_capacity;   // elements
    size_t vertex_cursor, index_cursor;
    bool   skinned;
};

struct pix__program_uniforms {
    GLuint program;
    GLint tex, color, metallic, roughness, emissive, cutoff, unlit, shadow_map;
    GLint model, tint, bones, light_view_proj;
    GLint env_map, planar_map, refract_map, planar, refraction;
    GLint scene_depth, params0, params1;
    GLint toon, extra_tex[4];
    GLint skin_base;
    int   clips;        // how many gl_ClipDistance planes its vertex stage writes
};

// A culled, sorted run of instance indices and where its transforms landed in
// the instance stream.
struct pix__draw_list {
    uint32_t* items;
    size_t    count;
    GLuint    base;
};

struct pix__line_vertex { float pos[3]; pix_color color; };
struct pix__gpu_instance { mat4 model; vec4 tint; };

// ---- render hooks: your own GPU work inside every view ----
//
// A hook is called from inside the pipeline with the view's framebuffer,
// viewport, depth buffer, scene block (matrices, sun, fog, time...) and
// shadow map all bound - the place for compute-driven instancing (grass,
// crowds of particles, GPU terrain) that should look and light like the rest
// of the scene. Compile its programs with pix_create_scene_shader to get the
// same lighting library the built-in materials use.
#define PIX_HOOK_OPAQUE      0   // after a view's opaque + skinned geometry (depth test and writes on)
#define PIX_HOOK_TRANSPARENT 1   // after its blended geometry (depth writes off)
#define PIX_HOOK_SHADOW      2   // inside each shadow cascade: draw depth only with ctx.view_proj

#define PIX_VIEW_USER   0
#define PIX_VIEW_PROBE  1   // a reflection-probe face: low detail is plenty
#define PIX_VIEW_MIRROR 2   // the planar mirror

struct pix_view;
struct pix_hook_context {
    int     pass;        // PIX_HOOK_*
    int     view_kind;   // PIX_VIEW_*
    int     cascade;     // shadow pass: which cascade, else -1
    mat4    view_proj;   // the camera (or light) this pass renders with
    vec3    eye;         // camera position (for shadows: the frame camera)
    frustum frustum;     // for culling against view_proj
    int     target_w, target_h;
};
struct pix_renderer;
typedef void (*pix_render_hook)(pix_renderer& renderer, const pix_hook_context& ctx, void* user);
#define PIX_MAX_HOOKS 16
struct pix__hook { pix_render_hook fn; void* user; uint32_t layers; };
struct pix_renderer {
    int  width;
    int  height;
    vec3 clear_color;       // behind everything when sky_enabled is false

    // ---- resources ----
    mesh*     meshes;        size_t mesh_count;
    mesh*     skinned_meshes; size_t skinned_mesh_count;
    material* materials;     size_t material_count;
    idx       shaders[MAX_SHADERS]; size_t shader_count;
    // Depth-only twins of surface shaders with their own vertex stage, so what
    // a vertex shader moves (wind, waves) casts a shadow that moves with it.
    struct { idx surface, depth; } depth_twins[MAX_SHADERS]; size_t depth_twin_count;
    // programs whose vertex stage writes gl_ClipDistance, how many planes each,
    // and how many are enabled right now (only while one of them draws)
    struct { idx program; int planes; } clip_programs[MAX_SHADERS]; size_t clip_program_count;
    // How much of the mirror's surfaces was really seen: a samples query
    // around their draws in the main view, read two frames later so it never
    // waits on the GPU. Hidden (behind a hill, say) or down to a sliver, the
    // mirror is not drawn and those surfaces reflect the sky instead.
    GLuint    mirror_query[4][4];    // per frame (a ring of four), one per run of mirror surfaces
    int       mirror_issued[4];      // how many of them were used
    bool      mirror_seen_last;      // the last decision, kept while a result is still on its way
    unsigned  mirror_frame;
    bool      mirror_tracking;       // set while the main view's blended list draws
    long long mirror_pixels;         // mirror-surface pixels last measured (-1: not known)
    int       clip_on;
    // materials marked cutout get a depth-only pass before the lit one in
    // every view (on by default; it only touches cutout materials)
    bool      cutout_prepass;
    idx       shader;          // built-in static surface program
    idx       skinned_shader;  // built-in skinned surface program
    idx       water_shader;    // compiled by the first pix_material_water
    idx       water_toon_shader;

    // ---- this frame's submissions ----
    pix_render_instance* instances;          size_t renderable_count;
    pix_render_instance* animated_instances; size_t animated_count;
    const mat4**         animated_bones;       // each instance's skinning matrices
    uint32_t*            animated_bone_count;
    pix_particle_vertex* particles;          size_t particle_count;
    pix__line_vertex*    lines;              size_t line_count;   // vertices, two per line

    // ---- the frame camera, set by begin_frame; shadows, reflections and the
    // default view are built around it ----
    camera frame_camera;
    mat4 frame_view_proj;
    bool frame_prepared;              // bounds, lights and the instance stream are ready

    // ---- the view being rendered right now (the frame camera between frames) ----
    mat4 view_matrix;
    mat4 projection_matrix;
    mat4 view_proj;
    vec3 camera_position;
    vec3 camera_dir;
    vec3 camera_up;
    float camera_far;
    float camera_near;
    frustum view_frustum;
    float default_fovy;

    // ---- lighting ----
    pix_sun sun;
    pix_sky sky;
    float hour;            // time of day, 0..24
    float time;            // seconds, handed to shaders as uTime
    bool  sky_enabled;     // analytic sky backdrop; off = clear_color
    pix_light_list* lights;

    // ---- look ----
    // Edit these freely at any time - see pix_shadow_settings / pix_post_settings.
    pix_shadow_settings shadows;
    pix_post_settings   post;
    pix_reflection_settings reflections;
    pix_post_effect     post_effects[PIX_MAX_POST_EFFECTS];
    int                 post_effect_count;
    float exposure;          // set by the time of day; post.exposure scales it

    // ---- statistics for the last frame ----
    int frame_draws;
    int frame_instances;
    int frame_culled;

    // ---- GPU timing, see pix_enable_gpu_timing ----
    bool   gpu_timing;
    GLuint gpu_query[PIX_GPU_PASSES][2];
    int    gpu_slot;
    double gpu_ms[PIX_GPU_PASSES];
    double cpu_ms[PIX_GPU_PASSES];
    long long cpu_begin;

    // ---- render hooks ----
    pix__hook hooks[PIX_MAX_HOOKS];
    int       hook_count;

    // ---- internals ----
    pix__geometry_pool pool;          // static meshes
    pix__geometry_pool skin_pool;     // skinned meshes
    GLuint instances_vbo;
    size_t instance_capacity;         // pix__gpu_instance slots in instances_vbo
    size_t instance_cursor;
    pix__gpu_instance* staging;       // PIX_MAX_INSTANCES
    vec4*     bounds;                 // per instance: world sphere
    uint8_t*  flags;                  // per instance: visibility / cascade bits
    uint64_t* sort_keys;  uint64_t* sort_keys_tmp;
    uint32_t* sort_vals;  uint32_t* sort_vals_tmp;
    pix__draw_list list_opaque, list_transparent, list_cascade[SHADOW_CASCADES];
    uint32_t* list_storage;           // backing for the lists above
    // instanced skinning: per-frame bone palette + instance records, per-view index lists
    GLuint skin_bones_ssbo, skin_inst_ssbo, skin_list_ssbo;
    size_t skin_bones_capacity, skin_list_capacity, skin_list_cursor;   // in elements
    mat4*  skin_bone_staging;  size_t skin_bone_staging_capacity;
    uint32_t* skin_list_staging;      // PIX_MAX_ANIMATED
    pix__program_uniforms uniform_cache[MAX_SHADERS * 2];
    int uniform_cache_count;

    GLuint scene_ubo, lights_ubo;
    idx white_texture;
    idx dummy_shadow;                 // 1x1 depth texture bound when shadows are off

    framebuffer scene_target;         // HDR
    framebuffer ldr_a, ldr_b;         // tonemapped ping-pong for FXAA and post effects
    framebuffer shadow_map;           // depth atlas, cascades side by side
    framebuffer bloom_a, bloom_b;
    GLint u_post[12];                 // cached uniform locations, see pix__cache_uniforms
    GLint u_bloom[6];
    GLint u_fxaa[2];
    GLint u_sky, u_particle[2];

    // reflections and refraction
    GLuint env_cube, env_fbo, env_depth;   // probe cubemap + its render target
    int    env_resolution;
    int    env_next_face;                  // round robin over the six faces
    int    env_faces_done;                 // reaches 6 once every face holds real data
    vec3   env_position;
    GLuint dummy_cube;                     // bound when there is no probe
    framebuffer planar_target;
    bool   planar_active;                  // this frame rendered a mirror image
    mat4   planar_view_proj;                // the mirror camera, for sampling its image
    GLuint refract_tex;
    GLuint refract_depth;                  // scene depth behind see-through surfaces
    int    refract_w, refract_h;
    bool   refract_needed;                 // a refractive material is visible this frame


    idx depth_shader, depth_skinned_shader, sky_shader, post_shader, fxaa_shader;
    idx bloom_prefilter_shader, bloom_blur_shader, particle_shader, line_shader;
    GLuint particle_vao, particle_vbo, line_vao, line_vbo;

    mat4  light_space[SHADOW_CASCADES];
    mat4  cascade_view[SHADOW_CASCADES];
    float cascade_radius[SHADOW_CASCADES];
    float cascade_far[SHADOW_CASCADES];
    float cascade_texel[SHADOW_CASCADES];
    float cascade_depth;
    float cascade_scale[SHADOW_CASCADES];   // share of its atlas tile each cascade fills
    int   cascade_mask;              // cascades rendered this frame (bit per cascade)
    unsigned shadow_frame;           // counts frames with shadows, for the stagger
    float cascade_fit_key[3];        // resolution, distance, depth range last fitted with
};

// ---- lifetime ----

// `shader` may be 0 for the built-in PBR surface (the usual choice).
PIXDEF pix_renderer pix_create_renderer(int width, int height, idx shader = 0,
                                        vec3 clear_color = { 0.0f, 0.5f, 0.8f });
PIXDEF void pix_destory_renderer(pix_renderer& renderer);
static inline void pix_destroy_renderer(pix_renderer& renderer) { pix_destory_renderer(renderer); }
// Call when the window size changes; reallocates the screen-sized targets.
PIXDEF void pix_renderer_resize(pix_renderer& renderer, int width, int height);

// Cascaded sun shadows and bloom at this resolution; the same as setting
// r.shadows.enabled / r.post.bloom yourself.
PIXDEF void pix_enable_effects(pix_renderer& renderer, int shadow_resolution = 2048);
// Shadow resolution and softness, bloom, FXAA and the reflection probe in one go (PIX_QUALITY_*).
// Leaves distances, strengths and the grade alone - those belong to the scene.
PIXDEF void pix_set_quality(pix_renderer& renderer, int quality);

// A custom fullscreen effect, run after tonemapping in the order added. `body`
// is GLSL that defines main() and writes `frag`; it can read
//   uScene (the image so far), uDepth + linear_depth(uv) (scene depth),
//   uTexel (one pixel in uv), uParams (vec4, yours), vUV,
//   and everything in the scene block (uTime, uCameraPos, uSunDir...).
// Returns the effect's index into r.post_effects (toggle .enabled, set
// .params), or -1 if it failed to compile. PIX_EFFECT_* are ready-made bodies.
PIXDEF int pix_add_post_effect(pix_renderer& renderer, const char* body,
                               vec4 params = { 1.0f, 0.0f, 0.0f, 0.0f });

// ---- meshes ----

PIXDEF idx load_mesh(pix_renderer& renderer, mesh_file_data& mesh_data);
PIXDEF idx load_mesh_data(pix_renderer& renderer, const vertex* vertices, size_t vertex_count,
                          const uint32_t* indices, size_t index_count);
PIXDEF idx load_skinned_mesh(pix_renderer& renderer, skinned_mesh_file_data& mesh_data);
// A skinned mesh file (glTF) with no skeleton, loaded as an ordinary instanced
// static mesh - how props and scenery from glTF files should be loaded.
PIXDEF idx load_mesh_from_skinned(pix_renderer& renderer, const skinned_mesh_file_data& mesh_data);

// Procedural primitives, centred on the origin (planes and boxes sit on y=0
// only if you move them there). Useful for prototyping, level blockouts and
// debug geometry.
PIXDEF idx pix_mesh_cube(pix_renderer& renderer, vec3 size);
PIXDEF idx pix_mesh_plane(pix_renderer& renderer, float width, float depth, float uv_scale = 1.0f);
PIXDEF idx pix_mesh_sphere(pix_renderer& renderer, float radius, int rings = 16, int segments = 32);
PIXDEF idx pix_mesh_cylinder(pix_renderer& renderer, float radius, float height, int segments = 24);

// ---- materials ----

// The defaults every other material call starts from.
PIXDEF material pix_material_default(void);
PIXDEF idx pix_add_material(pix_renderer& renderer, const material& m);
PIXDEF material* pix_get_material(pix_renderer& renderer, idx material_id);

// `texture_flags` picks the sampling: TEXTURE_BILINEAR (mipmapped, for real
// uv-mapped textures) or TEXTURE_LINEAR / TEXTURE_PIXELATED for colour-swatch
// atlases and pixel art, where mipmaps would bleed neighbouring cells.
PIXDEF idx load_material(pix_renderer& renderer,
    const char* texture_path = nullptr, vec3 color = { 1.0f, 1.0f, 1.0f },
    float metalic = 0.0f, float roughness = 0.6f, idx texture_flags = TEXTURE_BILINEAR);
// Same, for a texture already decoded in memory (glTF embeds its textures).
PIXDEF idx load_material_image(pix_renderer& renderer, image_file_data* image,
    vec3 color = { 1.0f, 1.0f, 1.0f }, float metalic = 0.0f, float roughness = 0.6f,
    bool pixelated = false);
// A second material over the same GL texture, differing only in tint and
// finish - no second decode or upload.
PIXDEF idx clone_material(pix_renderer& renderer, idx source, vec3 color,
                          float metallic, float roughness);
PIXDEF idx get_default_material(pix_renderer& renderer);

// Raw program from two complete sources.
PIXDEF idx load_shader(pix_renderer& renderer, const char* vsrc, const char* fsrc);
// A custom surface: `fsrc` is a complete fragment shader that receives vNormal,
// vUV, vWorld, vTint and the material uniforms FSHDER_BASIC declares, and has
// the whole lighting library (shade_surface, sky_env, apply_fog, uTime...)
// spliced in. Assign the result to material.shader.
PIXDEF idx load_surface_shader(pix_renderer& renderer, const char* fsrc, const char* vsrc = nullptr);
// `vsrc`, when given, replaces VSHDER_BASIC: same attribute layout (0 pos,
// 1 normal, 2 uv, 3..6 instance matrix, 7 instance tint), same outputs
// (vNormal, vUV, vWorld, vTint), scene block available - for wind, waves,
// vertex animation, billboards. The shadow pass then runs the same vertex
// shader too (with uViewProj standing for the light's matrix, and
// PIX_DEPTH_PASS defined), so the shadow follows whatever it moves; the
// material's uParams0/uParams1 reach it there as well. A vertex shader that
// writes gl_ClipDistance[0..n] (redeclare `out float gl_ClipDistance[n+1];`)
// gets those clip planes enabled while it draws, in both passes: cut a mesh
// into pieces with planes kept in the material params.

// Named uniforms on a material, for custom shaders: resolved to a location in
// the material's program once, sent only when the material is bound. Set the
// material's shader first. Returns false if the program has no such uniform.
PIXDEF bool pix_material_set_float(pix_renderer& renderer, idx material, const char* name, float value);
PIXDEF bool pix_material_set_vec2(pix_renderer& renderer, idx material, const char* name, vec2 value);
PIXDEF bool pix_material_set_vec3(pix_renderer& renderer, idx material, const char* name, vec3 value);
PIXDEF bool pix_material_set_vec4(pix_renderer& renderer, idx material, const char* name, vec4 value);
PIXDEF bool pix_material_set_int(pix_renderer& renderer, idx material, const char* name, int value);
// extra texture for custom shaders, read as uTex1..uTex4 (slot 1..4)
PIXDEF void pix_material_set_texture(pix_renderer& renderer, idx material, int slot, idx gl_texture);

// ---- reflection / refraction helpers ----
// Turns the planar mirror on for a plane through `point` facing `normal`
// (water at height h: normal (0,1,0), point (0,h,0)).
PIXDEF void pix_set_reflection_plane(pix_renderer& renderer, vec3 normal, vec3 point);
// Ready-made materials. Each returns a material to tweak further and add with
// pix_add_material.
//   glass   see-through, refracting, reflecting the probe/sky
//   mirror  a perfect mirror (needs pix_set_reflection_plane for its plane)
//   water   the built-in animated water surface; pair with a reflection plane
PIXDEF material pix_material_glass(vec3 tint, float opacity = 0.08f, float refraction = 0.04f);
PIXDEF material pix_material_mirror(vec3 tint = { 1.0f, 1.0f, 1.0f });
// Water, realistic or cartoon. shallow and deep are sRGB colours, depth
// the thickness at which it reaches the deep colour. Tune the waves and foam
// afterwards through material.params1 (see GLSL_WATER_COMMON). Works best with
// pix_set_reflection_plane at the water's height.
PIXDEF material pix_material_water(pix_renderer& renderer, vec3 shallow = { 0.10f, 0.42f, 0.45f },
                                   vec3 deep = { 0.01f, 0.07f, 0.12f }, float depth = 3.0f);
PIXDEF material pix_material_water_toon(pix_renderer& renderer, vec3 shallow = { 0.35f, 0.85f, 0.85f },
                                        vec3 deep = { 0.10f, 0.40f, 0.75f }, float depth = 1.5f);
// Your own water look: `fsrc` is a fragment shader (#version line and a main)
// with everything the built-in ones use in scope - the lighting library plus
// GLSL_WATER_COMMON: water_normal, water_thickness, screen_uv, uRefractMap,
// planar_reflection, the material's uColor / uParams0 / uParams1. Use it as
// the shader of a pix_material_water material.
PIXDEF idx load_water_shader(pix_renderer& renderer, const char* fsrc);
// ---- lighting ----

// Sun, sky, fog and exposure from an hour of the day (0..24), keyframed in lighting.hpp.
PIXDEF void pix_set_time_of_day(pix_renderer& renderer, float hour);
// Direct control, for games that do not want a day cycle. `direction` points
// toward the sun; colours are linear radiance.
PIXDEF void pix_set_sun(pix_renderer& renderer, vec3 direction, vec3 color);
PIXDEF void pix_set_ambient(pix_renderer& renderer, vec3 sky_irradiance, vec3 ground_irradiance);
PIXDEF void pix_set_sky_colors(pix_renderer& renderer, vec3 zenith, vec3 horizon, vec3 ground);
// sRGB colour; density ~0.001 is light haze at 1 unit = 1 metre, 0 disables
PIXDEF void pix_set_fog(pix_renderer& renderer, vec3 color, float density);
PIXDEF void pix_set_time(pix_renderer& renderer, float seconds);

// Punctual lights are submitted per frame like instances; the nearest
// MAX_SHADER_LIGHTS to the camera are shaded.
PIXDEF void push_light(pix_renderer& renderer, const pix_light& light);

// ---- per frame ----

PIXDEF void begin_frame(pix_renderer& renderer, camera& cam);
PIXDEF void push_instance(pix_renderer& renderer, pix_render_instance& instance);
PIXDEF void push_instance(pix_renderer& renderer, idx mesh, idx material, const mat4& transform,
                          vec4 tint = { 1.0f, 1.0f, 1.0f, 1.0f }, uint32_t layers = 1);
// `pose` must stay alive until end_frame; it is referenced, not copied
PIXDEF void push_animated_instance(pix_renderer& renderer, pix_render_instance& instance,
                                   const animation& pose);
// A skinned instance from any bone source - an animator's pose, or a
// pix_anim_system palette slice. `bones` must stay valid until end_frame.
PIXDEF void push_skinned(pix_renderer& renderer, idx mesh, idx material, const mat4& transform,
                         const mat4* bones, int bone_count, vec4 tint = { 1.0f, 1.0f, 1.0f, 1.0f },
                         uint32_t layers = 1);
// Hooks run in every view whose layers overlap `layers`, and in the shadow
// pass. Returns a handle for pix_remove_render_hook, or -1.
PIXDEF int  pix_add_render_hook(pix_renderer& renderer, pix_render_hook fn, void* user, uint32_t layers = 1);
PIXDEF void pix_remove_render_hook(pix_renderer& renderer, int handle);
// A program built like pix's own: `vs` gets the scene block and helpers,
// `fs` also the whole lighting library when `lit` (shade_surface, shade_toon,
// shadows, fog...). Both are complete sources with their own #version line.
PIXDEF idx pix_create_scene_shader(pix_renderer& renderer, const char* vs, const char* fs, bool lit = true);
// The skinned counterpart of load_surface_shader: your fragment shader, pix's
// instanced skinning vertex shader. Assign to material.skinned_shader.
PIXDEF idx load_skinned_surface_shader(pix_renderer& renderer, const char* fsrc);
// Points a program's pix samplers (uShadowMap, uEnvMap, uPlanarMap,
// uRefractMap, uSceneDepth) at the units the pipeline binds them to. Call it
// once for any program you draw with from a hook.
PIXDEF void pix_setup_scene_program(idx program);// `size` is a half width; `color` is linear radiance, deliberately unclamped
// so values above 1 glow through the bloom.
PIXDEF void push_particle(pix_renderer& renderer, vec3 at, float size, vec3 color, float alpha);
// Debug geometry, depth tested, unlit.
PIXDEF void push_line(pix_renderer& renderer, vec3 a, vec3 b, pix_color color);
PIXDEF void push_aabb_lines(pix_renderer& renderer, vec3 min, vec3 max, pix_color color);
PIXDEF void push_grid_lines(pix_renderer& renderer, vec3 center, float size, int cells, pix_color color);
PIXDEF void end_frame(pix_renderer& renderer, bool clear_instances = true);

// ---- the pipeline: composable stages ----
//
// end_frame runs the default composition:
//     pix_render_shadows(r);                                  // sun cascades
//     pix_render_reflections(r);                              // probe + mirror
//     pix_render_view(r, pix_view_default(r, frame camera));  // into r.scene_target
//     pix_post_process(r, r.scene_target, nullptr);           // HDR -> window
//     pix_finish_frame(r);
// Call the stages yourself instead to compose any frame: several views into
// several targets (split screen, minimap, portals, security cameras,
// render-to-texture), shader-override passes (IDs, normals, outlines), your
// own fullscreen passes between them. Everything submitted since begin_frame is
// available to every stage; each view culls and sorts it for its own camera.

// One scene render: a camera, a target, and what to draw.
struct pix_view {
    camera       cam;
    framebuffer* target;        // null = r.scene_target (screen-sized HDR)
    pix_rect     viewport;      // pixels inside the target, top-left origin; w <= 0 = all of it
    uint32_t     layers;        // draw instances with (instance.layers & layers) != 0
    bool clear;                 // clear colour + depth first (colour = clear_color, or the sky)
    vec3 clear_color;           // sRGB
    bool sky;
    bool opaque, skinned, transparent, particles, lines;
    bool shadows;               // receive the sun's shadows (needs pix_render_shadows first)
    bool reflections;           // sample the probe and planar mirror
    vec4 clip_plane;            // keep geometry with dot(xyz, p) + w >= 0; zero = no clip
    float max_distance;         // skip instances further than this (0 = camera far plane)
    idx  override_shader;       // draw every static instance with this program (0 = their materials')
    pix_rect scissor;           // pixels (top-left origin) outside which nothing is drawn; w <= 0 = none
};

// Everything on, targeting r.scene_target - the main view end_frame draws.
PIXDEF pix_view pix_view_default(pix_renderer& renderer, const camera& cam);
PIXDEF void pix_render_shadows(pix_renderer& renderer);
PIXDEF void pix_render_reflections(pix_renderer& renderer);
PIXDEF void pix_render_view(pix_renderer& renderer, const pix_view& view);
// HDR `source` -> tonemap, grade, bloom, post effects, FXAA -> `dest` (null =
// the window). Bloom, effects and FXAA run when the source is screen sized;
// other sizes get the tonemap and grade.
PIXDEF void pix_post_process(pix_renderer& renderer, const framebuffer& source, const framebuffer* dest);
// The enabled post effects alone, run over whatever is already drawn in
// `target` (null = the window) and written back into it: how a 2D frame,
// which never passes through the tonemap, gets the same screen effects.
// `target` must be the renderer's size. There is no depth, so depth-driven
// effects (outline, depth blur) see a flat scene.
PIXDEF void pix_apply_effects(pix_renderer& renderer, const framebuffer* target);
// A fullscreen pass with your own shader (from pix_load_fullscreen_shader):
// `textures` bound to uTex0..uTex3, uParams = params, uTexel = one dest pixel.
PIXDEF void pix_fullscreen_pass(pix_renderer& renderer, idx program, const idx* textures, int texture_count,
                                const framebuffer* dest, vec4 params = { 0.0f, 0.0f, 0.0f, 0.0f });
// `body` defines main() writing `frag`; vUV, uTex0..uTex3, uTexel, uParams and
// the scene block are declared for it.
PIXDEF idx pix_load_fullscreen_shader(pix_renderer& renderer, const char* body);
// Clears the frame's submissions (unless told not to) and restores the frame camera.
PIXDEF void pix_finish_frame(pix_renderer& renderer, bool clear_instances = true);
// World position -> window pixel (y down). Returns false if behind the camera.
PIXDEF bool pix_world_to_screen(const pix_renderer& renderer, vec3 world, vec2* out_pixel);
// Window pixel -> normalised world-space ray from the camera, for picking.
PIXDEF vec3 pix_screen_to_ray(const pix_renderer& renderer, float x, float y);

// ---- timing ----
PIXDEF void   pix_enable_gpu_timing(pix_renderer& renderer, bool on);
PIXDEF double pix_gpu_pass_ms(const pix_renderer& renderer, int pass);
PIXDEF double pix_cpu_pass_ms(const pix_renderer& renderer, int pass);

#endif // PIX_RENDERER_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_RENDERER_IMPL)
#define PIX_RENDERER_IMPL
#include <math.h>

// bits in pix_renderer::flags

static idx pix__compile(const char* vs, const char* fs, bool pbr_fragment) {
    char* v = shader_with_common(vs);
    char* f = pbr_fragment ? shader_with_pbr(fs) : shader_with_common(fs);
    idx p = (v && f) ? opengl_create_shader(v, f) : 0;
    pix_free(v);
    pix_free(f);
    return p;
}

// Skinned programs also need the storage-buffer declarations of GLSL_SKINNING.
static idx pix__compile_skinned(const char* vs, const char* fs, bool lit) {
    const char* vchunks[] = { GLSL_SCENE, GLSL_COMMON, GLSL_SKINNING };
    char* v = pix_shader_splice(vs, vchunks, 3);
    char* f = lit ? shader_with_pbr(fs) : 0;
    idx p = v ? opengl_create_shader(v, lit ? f : fs) : 0;
    pix_free(v);
    pix_free(f);
    return p;
}
static pix__program_uniforms* pix__uniforms_for(pix_renderer& r, GLuint program) {
    for (int i = 0; i < r.uniform_cache_count; i++)
        if (r.uniform_cache[i].program == program) return &r.uniform_cache[i];
    int slot = r.uniform_cache_count < (int)PIX_ARRAY_COUNT(r.uniform_cache)
             ? r.uniform_cache_count++ : (int)PIX_ARRAY_COUNT(r.uniform_cache) - 1;
    pix__program_uniforms& u = r.uniform_cache[slot];
    u.program     = program;
    u.tex         = glGetUniformLocation(program, "uTex");
    u.color       = glGetUniformLocation(program, "uColor");
    u.metallic    = glGetUniformLocation(program, "uMetallic");
    u.roughness   = glGetUniformLocation(program, "uRoughness");
    u.emissive    = glGetUniformLocation(program, "uEmissive");
    u.cutoff      = glGetUniformLocation(program, "uAlphaCutoff");
    u.unlit       = glGetUniformLocation(program, "uUnlit");
    u.shadow_map  = glGetUniformLocation(program, "uShadowMap");
    u.model       = glGetUniformLocation(program, "uModel");
    u.tint        = glGetUniformLocation(program, "uTint");
    u.bones       = glGetUniformLocation(program, "uBones[0]");
    if (u.bones < 0) u.bones = glGetUniformLocation(program, "uBones");
    u.light_view_proj = glGetUniformLocation(program, "uLightViewProj");
    u.skin_base   = glGetUniformLocation(program, "uSkinBase");
    u.env_map     = glGetUniformLocation(program, "uEnvMap");
    u.planar_map  = glGetUniformLocation(program, "uPlanarMap");
    u.refract_map = glGetUniformLocation(program, "uRefractMap");
    u.planar      = glGetUniformLocation(program, "uPlanar");
    u.refraction  = glGetUniformLocation(program, "uRefraction");
    u.scene_depth = glGetUniformLocation(program, "uSceneDepth");
    u.params0     = glGetUniformLocation(program, "uParams0");
    u.params1     = glGetUniformLocation(program, "uParams1");
    u.toon        = glGetUniformLocation(program, "uToon");
    const char* extra[4] = { "uTex1", "uTex2", "uTex3", "uTex4" };
    for (int i = 0; i < 4; i++) u.extra_tex[i] = glGetUniformLocation(program, extra[i]);
    u.clips = 0;
    for (size_t i = 0; i < r.clip_program_count; i++)
        if (r.clip_programs[i].program == program) u.clips = r.clip_programs[i].planes;
    return &u;
}

// ---------------- geometry pools ----------------

static void pix__pool_attribs(pix_renderer& r, pix__geometry_pool& p) {
    glBindVertexArray(p.vao);
    glBindBuffer(GL_ARRAY_BUFFER, p.vbo);
    if (!p.skinned) {
        GLsizei s = sizeof(vertex);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex, position));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex, normal));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex, uv));
        for (int i = 0; i < 3; i++) glEnableVertexAttribArray(i);
        // per-instance model matrix (3..6) and tint (7)
        glBindBuffer(GL_ARRAY_BUFFER, r.instances_vbo);
        for (int c = 0; c < 4; c++) {
            glVertexAttribPointer(3 + c, 4, GL_FLOAT, GL_FALSE, sizeof(pix__gpu_instance),
                                  (void*)(size_t)(c * 4 * sizeof(float)));
            glEnableVertexAttribArray(3 + c);
            glVertexAttribDivisor(3 + c, 1);
        }
        glVertexAttribPointer(7, 4, GL_FLOAT, GL_FALSE, sizeof(pix__gpu_instance),
                              (void*)offsetof(pix__gpu_instance, tint));
        glEnableVertexAttribArray(7);
        glVertexAttribDivisor(7, 1);
    } else {
        GLsizei s = sizeof(vertex_rigged);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex_rigged, position));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex_rigged, normal));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex_rigged, uv));
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex_rigged, bone_ids));
        glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex_rigged, bone_weights));
        for (int i = 0; i < 5; i++) glEnableVertexAttribArray(i);
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, p.ebo);
    glBindVertexArray(0);
}

static void pix__pool_create(pix_renderer& r, pix__geometry_pool& p, bool skinned,
                             size_t vertices, size_t indices) {
    p = pix__geometry_pool();
    p.skinned = skinned;
    p.vertex_size = skinned ? sizeof(vertex_rigged) : sizeof(vertex);
    p.vertex_capacity = vertices;
    p.index_capacity = indices;
    glGenVertexArrays(1, &p.vao);
    glGenBuffers(1, &p.vbo);
    glGenBuffers(1, &p.ebo);
    glBindBuffer(GL_ARRAY_BUFFER, p.vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(vertices * p.vertex_size), 0, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, p.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(indices * sizeof(uint32_t)), 0, GL_STATIC_DRAW);
    pix__pool_attribs(r, p);
}

// Replaces `buffer` with a larger one holding the same first `used` bytes.
static GLuint pix__grow_buffer(GLuint buffer, size_t used, size_t new_size) {
    GLuint grown = 0;
    glGenBuffers(1, &grown);
    glBindBuffer(GL_COPY_WRITE_BUFFER, grown);
    glBufferData(GL_COPY_WRITE_BUFFER, (GLsizeiptr)new_size, 0, GL_STATIC_DRAW);
    if (used) {
        glBindBuffer(GL_COPY_READ_BUFFER, buffer);
        glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0, (GLsizeiptr)used);
    }
    glDeleteBuffers(1, &buffer);
    return grown;
}

// Makes room for `vertices` + `indices` more, doubling as needed. Growth copies
// on the GPU, so the pool starts small and a game pays only for what it loads.
static void pix__pool_reserve(pix_renderer& r, pix__geometry_pool& p, size_t vertices, size_t indices) {
    bool changed = false;
    if (p.vertex_cursor + vertices > p.vertex_capacity) {
        size_t cap = p.vertex_capacity;
        while (p.vertex_cursor + vertices > cap) cap *= 2;
        p.vbo = pix__grow_buffer(p.vbo, p.vertex_cursor * p.vertex_size, cap * p.vertex_size);
        p.vertex_capacity = cap;
        changed = true;
    }
    if (p.index_cursor + indices > p.index_capacity) {
        size_t cap = p.index_capacity;
        while (p.index_cursor + indices > cap) cap *= 2;
        p.ebo = pix__grow_buffer(p.ebo, p.index_cursor * sizeof(uint32_t), cap * sizeof(uint32_t));
        p.index_capacity = cap;
        changed = true;
    }
    if (changed) {
        pix__pool_attribs(r, p);
        mesh* list = p.skinned ? r.skinned_meshes : r.meshes;
        size_t n = p.skinned ? r.skinned_mesh_count : r.mesh_count;
        for (size_t i = 0; i < n; i++) list[i].vbo = p.vbo;
    }
}

static void pix__pool_upload(pix__geometry_pool& p, const void* vertices, size_t vertex_count,
                             const uint32_t* indices, size_t index_count) {
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, p.vbo);
    glBufferSubData(GL_ARRAY_BUFFER, (GLintptr)(p.vertex_cursor * p.vertex_size),
                    (GLsizeiptr)(vertex_count * p.vertex_size), vertices);
    glBindBuffer(GL_COPY_WRITE_BUFFER, p.ebo);
    glBufferSubData(GL_COPY_WRITE_BUFFER, (GLintptr)(p.index_cursor * sizeof(uint32_t)),
                    (GLsizeiptr)(index_count * sizeof(uint32_t)), indices);
}

static void pix__mesh_bounds(mesh& m, const void* vertices, size_t count, size_t stride) {
    vec3 lo = v3(0, 0, 0), hi = v3(0, 0, 0);
    for (size_t i = 0; i < count; i++) {
        const vec3& p = *(const vec3*)((const char*)vertices + i * stride);   // position is first in both layouts
        if (i == 0) { lo = hi = p; continue; }
        lo = v3min(lo, p);
        hi = v3max(hi, p);
    }
    m.bounds_min = lo;
    m.bounds_max = hi;
    m.center = v3scale(v3add(lo, hi), 0.5f);
    float r2 = 0.0f;
    for (size_t i = 0; i < count; i++) {
        const vec3& p = *(const vec3*)((const char*)vertices + i * stride);
        vec3 d = v3sub(p, m.center);
        float d2 = v3dot(d, d);
        if (d2 > r2) r2 = d2;
    }
    m.radius = sqrtf(r2);
}

// ---------------- creation ----------------

// Everything screen sized. The optional ones (LDR ping-pong, bloom) are made
// on demand by pix__sync_settings, so a resize only has to drop them.
static void pix__destroy_screen_targets(pix_renderer& r) {
    framebuffer* all[] = { &r.scene_target, &r.ldr_a, &r.ldr_b, &r.bloom_a, &r.bloom_b };
    for (size_t i = 0; i < PIX_ARRAY_COUNT(all); i++)
        if (all[i]->fbo) pix_destroy_framebuffer(*all[i]);
}

static void pix__cache_uniforms(pix_renderer& r) {
    const char* post[] = { "uScene", "uBloom", "uTexel", "uExposure", "uSaturation", "uContrast",
                           "uVignette", "uSharpen", "uBloomStrength", "uTonemap", "uColorFilter", "uDither" };
    for (int i = 0; i < 12; i++) r.u_post[i] = glGetUniformLocation(r.post_shader, post[i]);
    r.u_bloom[0] = glGetUniformLocation(r.bloom_prefilter_shader, "uScene");
    r.u_bloom[1] = glGetUniformLocation(r.bloom_prefilter_shader, "uTexel");
    r.u_bloom[2] = glGetUniformLocation(r.bloom_prefilter_shader, "uThreshold");
    r.u_bloom[3] = glGetUniformLocation(r.bloom_prefilter_shader, "uKnee");
    r.u_bloom[4] = glGetUniformLocation(r.bloom_blur_shader, "uSource");
    r.u_bloom[5] = glGetUniformLocation(r.bloom_blur_shader, "uDirection");
    r.u_fxaa[0] = glGetUniformLocation(r.fxaa_shader, "uSource");
    r.u_fxaa[1] = glGetUniformLocation(r.fxaa_shader, "uTexel");
    r.u_sky = glGetUniformLocation(r.sky_shader, "uInvViewProj");
    r.u_particle[0] = glGetUniformLocation(r.particle_shader, "uCamRight");
    r.u_particle[1] = glGetUniformLocation(r.particle_shader, "uCamUp");
}

static int pix__enabled_effects(const pix_renderer& r) {
    int n = 0;
    for (int i = 0; i < r.post_effect_count; i++) if (r.post_effects[i].enabled && r.post_effects[i].program) n++;
    return n;
}

// Brings the GPU resources in line with r.shadows / r.post: allocates what a
// setting now needs, frees what it no longer does. Cheap when nothing changed
// - a handful of compares - so it runs every frame.
static void pix__sync_settings(pix_renderer& r) {
    if (!r.scene_target.fbo) r.scene_target = pix_create_render_target(r.width, r.height, true);

    pix_shadow_settings& s = r.shadows;
    if (s.resolution < 256) s.resolution = 256;
    if (s.resolution > 8192) s.resolution = 8192;
    if (s.enabled && (!r.shadow_map.fbo || r.shadow_map.height != s.resolution)) {
        if (r.shadow_map.fbo) pix_destroy_framebuffer(r.shadow_map);
        // one atlas, the cascades side by side: one sampler, one bind
        r.shadow_map = pix_create_shadow_map(s.resolution * SHADOW_CASCADES, s.resolution);
    } else if (!s.enabled && r.shadow_map.fbo) {
        pix_destroy_framebuffer(r.shadow_map);
    }

    bool bloom = r.post.bloom && r.post.bloom_strength > 0.0f;
    if (bloom && !r.bloom_a.fbo) {
        // quarter resolution: it is about to be blurred anyway
        int bw = r.width / 4 > 1 ? r.width / 4 : 1;
        int bh = r.height / 4 > 1 ? r.height / 4 : 1;
        r.bloom_a = pix_create_render_target(bw, bh, true, false, false);
        r.bloom_b = pix_create_render_target(bw, bh, true, false, false);
    } else if (!bloom && r.bloom_a.fbo) {
        pix_destroy_framebuffer(r.bloom_a);
        pix_destroy_framebuffer(r.bloom_b);
    }

    // LDR targets: one when anything runs after the tonemap, two to ping-pong
    int effects = pix__enabled_effects(r);
    int ldr_needed = (r.post.fxaa ? 1 : 0) + effects;
    int want = ldr_needed >= 2 ? 2 : ldr_needed;
    if (want >= 1 && !r.ldr_a.fbo) r.ldr_a = pix_create_render_target(r.width, r.height, false, false, false);
    if (want >= 2 && !r.ldr_b.fbo) r.ldr_b = pix_create_render_target(r.width, r.height, false, false, false);
    if (want < 2 && r.ldr_b.fbo) pix_destroy_framebuffer(r.ldr_b);
    if (want < 1 && r.ldr_a.fbo) pix_destroy_framebuffer(r.ldr_a);

    // ---- reflections ----
    pix_reflection_settings& rs = r.reflections;

    int probe_res = rs.probe_resolution < 32 ? 32 : (rs.probe_resolution > 1024 ? 1024 : rs.probe_resolution);
    if (r.env_cube && (!rs.probe || r.env_resolution != probe_res)) {
        glDeleteTextures(1, &r.env_cube);
        glDeleteFramebuffers(1, &r.env_fbo);
        glDeleteRenderbuffers(1, &r.env_depth);
        r.env_cube = r.env_fbo = r.env_depth = 0;
        r.env_faces_done = 0;
    }
    if (rs.probe && !r.env_cube) {
        r.env_resolution = probe_res;
        glGenTextures(1, &r.env_cube);
        glBindTexture(GL_TEXTURE_CUBE_MAP, r.env_cube);
        for (int f = 0; f < 6; f++)
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + f, 0, GL_R11F_G11F_B10F, probe_res, probe_res, 0,
                         GL_RGB, GL_FLOAT, 0);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
        glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
        glGenRenderbuffers(1, &r.env_depth);
        glBindRenderbuffer(GL_RENDERBUFFER, r.env_depth);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, probe_res, probe_res);
        glGenFramebuffers(1, &r.env_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, r.env_fbo);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, r.env_depth);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        r.env_faces_done = 0;
        r.env_next_face = 0;
    }

    float scale = clampf(rs.planar_scale, 0.1f, 1.0f);
    int pw = (int)((float)r.width * scale), ph = (int)((float)r.height * scale);
    if (pw < 1) pw = 1;
    if (ph < 1) ph = 1;
    if (r.planar_target.fbo && (!rs.planar || r.planar_target.width != pw || r.planar_target.height != ph))
        pix_destroy_framebuffer(r.planar_target);
    if (rs.planar && !r.planar_target.fbo)
        r.planar_target = pix_create_render_target(pw, ph, true, true, true);   // mipmapped: rough mirrors blur

}

// Matches the scene target's format so the copy is a raw GPU-side memcpy.
static void pix__ensure_refract_tex(pix_renderer& r, int w, int h) {
    if (r.refract_tex && r.refract_w == w && r.refract_h == h) return;
    if (r.refract_tex) { glDeleteTextures(1, &r.refract_tex); glDeleteTextures(1, &r.refract_depth); }
    glGenTextures(1, &r.refract_tex);
    glBindTexture(GL_TEXTURE_2D, r.refract_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R11F_G11F_B10F, w, h, 0, GL_RGB, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glGenTextures(1, &r.refract_depth);
    glBindTexture(GL_TEXTURE_2D, r.refract_depth);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w, h, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    r.refract_w = w;
    r.refract_h = h;
}

pix_renderer pix_create_renderer(int width, int height, idx shader, vec3 clear_color) {
    pix_renderer r = {};
    r.width  = width  > 0 ? width  : 1;
    r.height = height > 0 ? height : 1;
    r.clear_color = clear_color;

    r.meshes             = pix_alloc<mesh>(PIX_MAX_MESHES);
    r.skinned_meshes     = pix_alloc<mesh>(PIX_MAX_MESHES);
    r.materials          = pix_alloc<material>(PIX_MAX_MATERIALS);
    r.instances          = pix_alloc<pix_render_instance>(PIX_MAX_INSTANCES);
    r.animated_instances = pix_alloc<pix_render_instance>(PIX_MAX_ANIMATED);
    r.animated_bones     = pix_alloc<const mat4*>(PIX_MAX_ANIMATED);
    r.animated_bone_count = pix_alloc<uint32_t>(PIX_MAX_ANIMATED);
    r.skin_list_staging  = pix_alloc<uint32_t>(PIX_MAX_ANIMATED);
    r.particles          = pix_alloc<pix_particle_vertex>(PIX_MAX_PARTICLES);
    r.lines              = pix_alloc<pix__line_vertex>(PIX_MAX_LINES * 2);
    r.lights             = pix_alloc<pix_light_list>(1);
    r.staging            = pix_alloc<pix__gpu_instance>(PIX_MAX_INSTANCES);
    r.bounds             = pix_alloc<vec4>(PIX_MAX_INSTANCES);
    r.flags              = pix_alloc<uint8_t>(PIX_MAX_INSTANCES);
    r.sort_keys          = pix_alloc<uint64_t>(PIX_MAX_INSTANCES);
    r.sort_keys_tmp      = pix_alloc<uint64_t>(PIX_MAX_INSTANCES);
    r.sort_vals          = pix_alloc<uint32_t>(PIX_MAX_INSTANCES);
    r.sort_vals_tmp      = pix_alloc<uint32_t>(PIX_MAX_INSTANCES);
    r.list_storage       = pix_alloc<uint32_t>(PIX_MAX_INSTANCES * (2 + SHADOW_CASCADES));
    r.list_opaque.items      = r.list_storage;
    r.list_transparent.items = r.list_storage + PIX_MAX_INSTANCES;
    for (int c = 0; c < SHADOW_CASCADES; c++)
        r.list_cascade[c].items = r.list_storage + PIX_MAX_INSTANCES * (2 + c);

    // The instance stream. Sized for a modest frame and regrown by end_frame
    // whenever a frame needs more - see pix__instances_begin.
    r.instance_capacity = 4096;
    glGenBuffers(1, &r.instances_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, r.instances_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(r.instance_capacity * sizeof(pix__gpu_instance)), 0, GL_STREAM_DRAW);

    // Pools start at a few MB and double as meshes are loaded.
    pix__pool_create(r, r.pool, false, 1 << 16, 1 << 17);
    pix__pool_create(r, r.skin_pool, true, 1 << 14, 1 << 15);

    r.shader         = shader ? shader : pix__compile(VSHDER_BASIC, FSHDER_BASIC, true);
    r.skinned_shader = pix__compile_skinned(VSHDER_SKINNED, FSHDER_BASIC, true);
    r.sky_shader     = pix__compile(VSHDER_SKY, FSHDER_SKY, false);
    r.post_shader    = pix__compile(VSHDER_POST, FSHDER_POST, false);
    r.fxaa_shader    = opengl_create_shader(VSHDER_POST, FSHDER_FXAA);
    r.line_shader    = pix__compile(VSHDER_LINE, FSHDER_LINE, false);
    r.particle_shader = pix__compile(VSHDER_PARTICLE, FSHDER_PARTICLE, false);
    r.depth_shader         = opengl_create_shader(VSHDER_DEPTH, FSHDER_DEPTH);
    r.depth_skinned_shader = pix__compile_skinned(VSHDER_DEPTH_SKINNED, FSHDER_DEPTH, false);
    {
        // the skinning storage buffers; sized on first use and grown as crowds do
        GLuint b[3];
        glGenBuffers(3, b);
        r.skin_bones_ssbo = b[0]; r.skin_inst_ssbo = b[1]; r.skin_list_ssbo = b[2];
    }
    r.bloom_prefilter_shader = opengl_create_shader(VSHDER_POST, FSHDER_BLOOM_PREFILTER);
    r.bloom_blur_shader      = opengl_create_shader(VSHDER_POST, FSHDER_BLOOM_BLUR);
    pix__cache_uniforms(r);

    unsigned char white[4] = { 255, 255, 255, 255 };
    r.white_texture = opengl_create_texture2d(1, 1, 4, white, TEXTURE_PIXELATED);
    {
        // sampler2DShadow must always see a depth texture, even with shadows off
        GLuint t = 0;
        float one = 1.0f;
        glGenTextures(1, &t);
        glBindTexture(GL_TEXTURE_2D, t);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, 1, 1, 0, GL_DEPTH_COMPONENT, GL_FLOAT, &one);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
        r.dummy_shadow = t;
    }
    {
        // and samplerCube a cube map
        GLuint c = 0;
        float black[3] = { 0.0f, 0.0f, 0.0f };
        glGenTextures(1, &c);
        glBindTexture(GL_TEXTURE_CUBE_MAP, c);
        for (int f = 0; f < 6; f++)
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + f, 0, GL_R11F_G11F_B10F, 1, 1, 0, GL_RGB, GL_FLOAT, black);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        r.dummy_cube = c;
    }
    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);   // no visible seams where probe faces meet

    // particles: per-instance attributes only, the quad comes from gl_VertexID
    glGenVertexArrays(1, &r.particle_vao);
    glGenBuffers(1, &r.particle_vbo);
    glBindVertexArray(r.particle_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.particle_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(PIX_MAX_PARTICLES * sizeof(pix_particle_vertex)), 0, GL_STREAM_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(pix_particle_vertex), (void*)0);
    glVertexAttribDivisor(0, 1);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(pix_particle_vertex), (void*)(sizeof(float) * 4));
    glVertexAttribDivisor(1, 1);

    glGenVertexArrays(1, &r.line_vao);
    glGenBuffers(1, &r.line_vbo);
    glBindVertexArray(r.line_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.line_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(PIX_MAX_LINES * 2 * sizeof(pix__line_vertex)), 0, GL_STREAM_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pix__line_vertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(pix__line_vertex),
                          (void*)offsetof(pix__line_vertex, color));
    glBindVertexArray(0);

    // the two per-frame uniform blocks, bound to their binding points once
    glGenBuffers(1, &r.scene_ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, r.scene_ubo);
    glBufferData(GL_UNIFORM_BUFFER, (GLsizeiptr)sizeof(pix_scene_block), 0, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, r.scene_ubo);
    glGenBuffers(1, &r.lights_ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, r.lights_ubo);
    glBufferData(GL_UNIFORM_BUFFER, (GLsizeiptr)sizeof(pix_light_block), 0, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, 1, r.lights_ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    r.default_fovy     = 1.05f;
    r.projection_matrix = mat4_perspective(r.default_fovy, (float)r.width / (float)r.height, 0.1f, 1000.0f);
    r.view_matrix      = mat4_identity();
    r.view_proj        = r.projection_matrix;
    r.sky_enabled      = true;

    // off until asked for: pix_enable_effects, pix_set_quality, or r.shadows.enabled = true
    r.shadows.enabled     = false;
    r.shadows.resolution  = 2048;
    r.shadows.distance    = 120.0f;
    r.shadows.strength    = 1.0f;
    r.shadows.depth_range = 400.0f;
    r.shadows.softness    = 1;
    r.shadows.skinned     = true;
    r.shadows.skinned_cascades = SHADOW_CASCADES;
    r.cutout_prepass = true;
    r.shadows.stagger     = false;
    r.shadows.far_resolution = 1.0f;

    r.post.exposure        = 1.0f;
    r.post.tonemap         = PIX_TONEMAP_ACES;
    r.post.bloom           = false;
    r.post.bloom_strength  = 0.25f;
    r.post.bloom_threshold = 1.0f;
    r.post.bloom_knee      = 0.5f;
    r.post.saturation      = 1.05f;
    r.post.contrast        = 0.15f;
    r.post.vignette        = 0.20f;
    r.post.sharpen         = 0.0f;
    r.post.color_filter    = v3(1.0f, 1.0f, 1.0f);
    r.post.fxaa            = true;
    r.post.dither          = true;

    r.reflections.probe                 = false;
    r.reflections.probe_resolution      = 128;
    r.reflections.probe_faces_per_frame = 1;
    r.reflections.probe_follow_camera   = true;
    r.reflections.probe_distance        = 150.0f;
    r.reflections.probe_intensity       = 1.0f;
    r.reflections.planar                = false;
    r.reflections.plane                 = v4(0.0f, 1.0f, 0.0f, 0.0f);
    r.reflections.planar_scale          = 0.5f;
    r.reflections.planar_distance       = 200.0f;
    pix__sync_settings(r);

    pix_lights_clear(*r.lights);
    pix_set_time_of_day(r, 10.0f);
    for (int c = 0; c < SHADOW_CASCADES; c++) r.light_space[c] = mat4_identity();
    return r;
}

void pix_enable_effects(pix_renderer& r, int shadow_resolution) {
    r.shadows.enabled = true;
    r.shadows.resolution = shadow_resolution;
    r.post.bloom = true;
    pix__sync_settings(r);
}

void pix_set_quality(pix_renderer& r, int quality) {
    //                            low    medium  high   ultra
    static const int  res[4]  = { 1024,  1536,   2048,  4096 };
    static const int  soft[4] = { 0,     1,      1,     2 };
    static const bool fx[4]   = { false, true,   true,  true };
    static const bool glow[4] = { false, true,   true,  true };
    static const bool probe[4] = { false, false, true,  true };
    static const int  probe_res[4] = { 64, 64,   128,   256 };
    int q = quality < 0 ? 0 : (quality > 3 ? 3 : quality);
    r.shadows.enabled = true;
    r.shadows.resolution = res[q];
    r.shadows.softness = soft[q];
    r.post.fxaa = fx[q];
    r.post.bloom = glow[q];
    r.reflections.probe = probe[q];
    r.reflections.probe_resolution = probe_res[q];
    r.reflections.probe_faces_per_frame = q == PIX_QUALITY_ULTRA ? 2 : 1;
    pix__sync_settings(r);
}

// The declarations every post effect body gets for free.
static const char* const PIX__EFFECT_PRELUDE = R"(#version 440 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uScene;   // the image so far: tonemapped, sRGB
uniform sampler2D uDepth;   // scene depth, 0..1
uniform vec2 uTexel;        // one pixel in uv
uniform vec4 uParams;       // the effect's params
uniform vec2 uNearFar;
float linear_depth(vec2 uv) {
    float z = texture(uDepth, uv).r * 2.0 - 1.0;
    return 2.0 * uNearFar.x * uNearFar.y / (uNearFar.y + uNearFar.x - z * (uNearFar.y - uNearFar.x));
}
)";

int pix_add_post_effect(pix_renderer& r, const char* body, vec4 params) {
    if (r.post_effect_count >= PIX_MAX_POST_EFFECTS || !body) return -1;
    size_t n = strlen(PIX__EFFECT_PRELUDE) + strlen(body) + 2;
    char* src = pix_new<char>(n);
    if (!src) return -1;
    strcpy(src, PIX__EFFECT_PRELUDE);
    strcat(src, body);
    char* full = shader_with_common(src);   // scene block and helpers after #version
    idx prog = full ? opengl_create_shader(VSHDER_POST, full) : 0;
    pix_free(full);
    pix_free(src);
    if (!prog) return -1;
    pix_post_effect& e = r.post_effects[r.post_effect_count];
    e.program = prog;
    e.enabled = true;
    e.params = params;
    e.u_scene    = glGetUniformLocation(prog, "uScene");
    e.u_depth    = glGetUniformLocation(prog, "uDepth");
    e.u_texel    = glGetUniformLocation(prog, "uTexel");
    e.u_params   = glGetUniformLocation(prog, "uParams");
    e.u_near_far = glGetUniformLocation(prog, "uNearFar");
    return r.post_effect_count++;
}

void pix_renderer_resize(pix_renderer& r, int width, int height) {
    if (width <= 0 || height <= 0) return;             // minimised
    if (width == r.width && height == r.height) return;
    r.width = width;
    r.height = height;
    pix__destroy_screen_targets(r);
    pix__sync_settings(r);
}

void pix_destory_renderer(pix_renderer& r) {
    pix__destroy_screen_targets(r);
    if (r.shadow_map.fbo) pix_destroy_framebuffer(r.shadow_map);
    GLuint bufs[] = { r.instances_vbo, r.pool.vbo, r.pool.ebo, r.skin_pool.vbo, r.skin_pool.ebo,
                      r.particle_vbo, r.line_vbo, r.scene_ubo, r.lights_ubo,
                      r.skin_bones_ssbo, r.skin_inst_ssbo, r.skin_list_ssbo };
    glDeleteBuffers((GLsizei)PIX_ARRAY_COUNT(bufs), bufs);
    GLuint vaos[] = { r.pool.vao, r.skin_pool.vao, r.particle_vao, r.line_vao };
    glDeleteVertexArrays((GLsizei)PIX_ARRAY_COUNT(vaos), vaos);
    idx progs[] = { r.shader, r.skinned_shader, r.sky_shader, r.post_shader, r.fxaa_shader, r.line_shader,
                    r.particle_shader, r.depth_shader, r.depth_skinned_shader,
                    r.bloom_prefilter_shader, r.bloom_blur_shader };
    for (size_t i = 0; i < PIX_ARRAY_COUNT(progs); i++) opengl_destroy_shader(progs[i]);
    for (size_t i = 0; i < r.shader_count; i++) opengl_destroy_shader(r.shaders[i]);
    for (int i = 0; i < r.post_effect_count; i++) opengl_destroy_shader(r.post_effects[i].program);
    opengl_destroy_texture(r.white_texture);
    opengl_destroy_texture(r.dummy_shadow);
    opengl_destroy_texture(r.dummy_cube);
    opengl_destroy_texture(r.env_cube);
    opengl_destroy_texture(r.refract_tex);
    opengl_destroy_texture(r.refract_depth);
    if (r.env_fbo) glDeleteFramebuffers(1, &r.env_fbo);
    if (r.env_depth) glDeleteRenderbuffers(1, &r.env_depth);
    if (r.planar_target.fbo) pix_destroy_framebuffer(r.planar_target);
    void* heap[] = { r.meshes, r.skinned_meshes, r.materials, r.instances, r.animated_instances,
                     (void*)r.animated_bones, r.animated_bone_count, r.skin_list_staging, r.skin_bone_staging, r.particles, r.lines, r.lights, r.staging, r.bounds, r.flags,
                     r.sort_keys, r.sort_keys_tmp, r.sort_vals, r.sort_vals_tmp, r.list_storage };
    for (size_t i = 0; i < PIX_ARRAY_COUNT(heap); i++) pix_free(heap[i]);
    r = pix_renderer();
}

void pix_set_reflection_plane(pix_renderer& r, vec3 normal, vec3 point) {
    vec3 n = v3norm(normal);
    r.reflections.planar = true;
    r.reflections.plane = v4(n.x, n.y, n.z, -v3dot(n, point));
}

material pix_material_glass(vec3 tint, float opacity, float refraction) {
    material m = pix_material_default();
    m.color = tint;
    m.alpha = opacity;
    m.roughness = 0.03f;
    m.refraction = refraction > 0.0f ? refraction : 0.001f;
    m.cast_shadows = false;
    m.double_sided = true;
    return m;
}

material pix_material_mirror(vec3 tint) {
    material m = pix_material_default();
    m.color = tint;
    m.metallic = 1.0f;
    m.roughness = 0.02f;
    m.planar_reflection = 1.0f;
    return m;
}

// Water sources are a main() plus GLSL_WATER_COMMON, which has to land after
// the lighting library it calls into: splice it first, then the library goes
// in ahead of it, right after #version.
static idx pix__water_program(pix_renderer& r, const char* fsrc) {
    const char* chunks[] = { GLSL_WATER_COMMON };
    char* with_common = pix_shader_splice(fsrc, chunks, 1);
    idx prog = with_common ? load_surface_shader(r, with_common) : 0;
    pix_free(with_common);
    return prog;
}

static material pix__water_material(idx shader, vec3 shallow, vec3 deep, float depth) {
    material m = pix_material_default();
    m.shader = shader;
    m.color = shallow;
    m.params0 = v4(deep.x, deep.y, deep.z, depth);
    m.params1 = v4(1.0f, 1.0f, 1.0f, 1.0f);   // wave scale, speed, foam, strength
    m.roughness = 0.05f;
    m.planar_reflection = 1.0f;
    m.refraction = 0.04f;
    m.cast_shadows = false;
    m.double_sided = true;
    return m;
}

idx load_water_shader(pix_renderer& r, const char* fsrc) { return pix__water_program(r, fsrc); }

material pix_material_water(pix_renderer& r, vec3 shallow, vec3 deep, float depth) {
    if (!r.water_shader) r.water_shader = pix__water_program(r, FSHDER_WATER_SURFACE);
    return pix__water_material(r.water_shader, shallow, deep, depth);
}

material pix_material_water_toon(pix_renderer& r, vec3 shallow, vec3 deep, float depth) {
    if (!r.water_toon_shader) r.water_toon_shader = pix__water_program(r, FSHDER_WATER_TOON);
    material m = pix__water_material(r.water_toon_shader, shallow, deep, depth);
    m.planar_reflection = 0.0f;     // stylised water rarely wants a photographic mirror
    m.refraction = 0.02f;
    return m;
}
// ---------------- lighting ----------------

void pix_set_time_of_day(pix_renderer& r, float hour) {
    // wrapped here so winding the clock back past midnight reads 23:00, not -1:00
    hour = fmodf(hour, 24.0f);
    if (hour < 0.0f) hour += 24.0f;
    r.hour = hour;
    pix_daylight_at(hour, &r.sun, &r.sky, &r.exposure);
}

void pix_set_sun(pix_renderer& r, vec3 direction, vec3 color) {
    r.sun.direction = v3norm(direction);
    r.sun.color = color;
}

void pix_set_ambient(pix_renderer& r, vec3 sky_irradiance, vec3 ground_irradiance) {
    r.sky.diffuse_up = sky_irradiance;
    r.sky.diffuse_down = ground_irradiance;
}

void pix_set_sky_colors(pix_renderer& r, vec3 zenith, vec3 horizon, vec3 ground) {
    r.sky.zenith = zenith;
    r.sky.horizon = horizon;
    r.sky.ground = ground;
}

void pix_set_fog(pix_renderer& r, vec3 color, float density) {
    r.sky.fog = color;
    r.sky.fog_density = density;
}

void pix_set_time(pix_renderer& r, float seconds) { r.time = seconds; }

void push_light(pix_renderer& r, const pix_light& light) { pix_lights_add(*r.lights, light); }

// ---------------- meshes ----------------

idx load_mesh_data(pix_renderer& r, const vertex* vertices, size_t vertex_count,
                   const uint32_t* indices, size_t index_count) {
    if (r.mesh_count >= PIX_MAX_MESHES || !vertex_count || !index_count) return PIX_INVALID;
    pix__pool_reserve(r, r.pool, vertex_count, index_count);
    idx id = (idx)r.mesh_count++;
    mesh& m = r.meshes[id];
    m.vbo = r.pool.vbo;
    m.vertex_offset = (idx)r.pool.vertex_cursor;
    m.vertex_size = sizeof(vertex);
    m.vertex_count = (idx)vertex_count;
    m.index_offset = (idx)r.pool.index_cursor;
    m.index_count = (idx)index_count;
    pix__mesh_bounds(m, vertices, vertex_count, sizeof(vertex));
    pix__pool_upload(r.pool, vertices, vertex_count, indices, index_count);
    r.pool.vertex_cursor += vertex_count;
    r.pool.index_cursor += index_count;
    return id;
}

idx load_mesh(pix_renderer& r, mesh_file_data& md) {
    return load_mesh_data(r, md.vertex_data, md.vertex_count, md.index_data, md.index_count);
}

idx load_skinned_mesh(pix_renderer& r, skinned_mesh_file_data& md) {
    if (r.skinned_mesh_count >= PIX_MAX_MESHES || !md.vertex_count || !md.index_count) return PIX_INVALID;
    pix__pool_reserve(r, r.skin_pool, md.vertex_count, md.index_count);
    idx id = (idx)r.skinned_mesh_count++;
    mesh& m = r.skinned_meshes[id];
    m.vbo = r.skin_pool.vbo;
    m.vertex_offset = (idx)r.skin_pool.vertex_cursor;
    m.vertex_size = sizeof(vertex_rigged);
    m.vertex_count = (idx)md.vertex_count;
    m.index_offset = (idx)r.skin_pool.index_cursor;
    m.index_count = (idx)md.index_count;
    pix__mesh_bounds(m, md.vertex_data, md.vertex_count, sizeof(vertex_rigged));
    pix__pool_upload(r.skin_pool, md.vertex_data, md.vertex_count, md.index_data, md.index_count);
    r.skin_pool.vertex_cursor += md.vertex_count;
    r.skin_pool.index_cursor += md.index_count;
    return id;
}

idx load_mesh_from_skinned(pix_renderer& r, const skinned_mesh_file_data& md) {
    vertex* v = pix_alloc<vertex>(md.vertex_count);
    if (!v) return PIX_INVALID;
    for (size_t i = 0; i < md.vertex_count; i++) {
        v[i].position = md.vertex_data[i].position;
        v[i].normal   = md.vertex_data[i].normal;
        v[i].uv       = md.vertex_data[i].uv;
    }
    idx id = load_mesh_data(r, v, md.vertex_count, md.index_data, md.index_count);
    pix_free(v);
    return id;
}

// -- primitives --

struct pix__mesh_builder { vertex* v; uint32_t* i; size_t nv, ni; };

static void pix__mb_vert(pix__mesh_builder& b, vec3 p, vec3 n, vec2 uv) {
    vertex& o = b.v[b.nv++];
    o.position = p; o.normal = n; o.uv = uv;
}
static void pix__mb_quad(pix__mesh_builder& b, uint32_t a, uint32_t c, uint32_t d, uint32_t e) {
    b.i[b.ni++] = a; b.i[b.ni++] = c; b.i[b.ni++] = d;
    b.i[b.ni++] = a; b.i[b.ni++] = d; b.i[b.ni++] = e;
}

idx pix_mesh_cube(pix_renderer& r, vec3 size) {
    pix__mesh_builder b = { pix_alloc<vertex>(24), pix_alloc<uint32_t>(36), 0, 0 };
    vec3 h = v3scale(size, 0.5f);
    // each face: normal, and the two in-plane axes (right, up) as seen from outside
    const vec3 N[6] = { {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1} };
    const vec3 R[6] = { {0,0,-1}, {0,0,1}, {1,0,0}, {1,0,0}, {1,0,0}, {-1,0,0} };
    const vec3 U[6] = { {0,1,0}, {0,1,0}, {0,0,-1}, {0,0,1}, {0,1,0}, {0,1,0} };
    for (int f = 0; f < 6; f++) {
        uint32_t base = (uint32_t)b.nv;
        for (int k = 0; k < 4; k++) {
            float sx = (k == 1 || k == 2) ? 1.0f : -1.0f;
            float sy = (k >= 2) ? 1.0f : -1.0f;
            vec3 p = v3add(v3scale(N[f], 1.0f), v3add(v3scale(R[f], sx), v3scale(U[f], sy)));
            p = v3mul(p, h);
            pix__mb_vert(b, p, N[f], v2(sx * 0.5f + 0.5f, 0.5f - sy * 0.5f));
        }
        pix__mb_quad(b, base, base + 1, base + 2, base + 3);
    }
    idx id = load_mesh_data(r, b.v, b.nv, b.i, b.ni);
    pix_free(b.v); pix_free(b.i);
    return id;
}

idx pix_mesh_plane(pix_renderer& r, float width, float depth, float uv_scale) {
    vertex v[4];
    float hx = width * 0.5f, hz = depth * 0.5f;
    vec3 up = v3(0, 1, 0);
    v[0].position = v3(-hx, 0, -hz); v[0].normal = up; v[0].uv = v2(0, 0);
    v[1].position = v3(-hx, 0,  hz); v[1].normal = up; v[1].uv = v2(0, uv_scale);
    v[2].position = v3( hx, 0,  hz); v[2].normal = up; v[2].uv = v2(uv_scale, uv_scale);
    v[3].position = v3( hx, 0, -hz); v[3].normal = up; v[3].uv = v2(uv_scale, 0);
    uint32_t i[6] = { 0, 1, 2, 0, 2, 3 };
    return load_mesh_data(r, v, 4, i, 6);
}

idx pix_mesh_sphere(pix_renderer& r, float radius, int rings, int segments) {
    if (rings < 2) rings = 2;
    if (segments < 3) segments = 3;
    size_t nv = (size_t)(rings + 1) * (size_t)(segments + 1);
    pix__mesh_builder b = { pix_alloc<vertex>(nv), pix_alloc<uint32_t>((size_t)rings * segments * 6), 0, 0 };
    for (int y = 0; y <= rings; y++) {
        float v = (float)y / (float)rings, phi = v * PIX_PI;
        for (int x = 0; x <= segments; x++) {
            float u = (float)x / (float)segments, theta = u * PIX_TAU;
            vec3 n = v3(-cosf(theta) * sinf(phi), cosf(phi), sinf(theta) * sinf(phi));
            pix__mb_vert(b, v3scale(n, radius), n, v2(u, v));
        }
    }
    for (int y = 0; y < rings; y++)
        for (int x = 0; x < segments; x++) {
            uint32_t a = (uint32_t)(y * (segments + 1) + x), c = a + (uint32_t)segments + 1;
            pix__mb_quad(b, a, c, c + 1, a + 1);
        }
    idx id = load_mesh_data(r, b.v, b.nv, b.i, b.ni);
    pix_free(b.v); pix_free(b.i);
    return id;
}

idx pix_mesh_cylinder(pix_renderer& r, float radius, float height, int segments) {
    if (segments < 3) segments = 3;
    size_t nv = (size_t)(segments + 1) * 2 + (size_t)(segments + 2) * 2;
    pix__mesh_builder b = { pix_alloc<vertex>(nv), pix_alloc<uint32_t>((size_t)segments * 12), 0, 0 };
    float hh = height * 0.5f;
    for (int s = 0; s <= segments; s++) {                     // side wall
        float u = (float)s / (float)segments, a = u * PIX_TAU;
        vec3 n = v3(cosf(a), 0.0f, -sinf(a));
        pix__mb_vert(b, v3(n.x * radius, -hh, n.z * radius), n, v2(u, 1.0f));
        pix__mb_vert(b, v3(n.x * radius,  hh, n.z * radius), n, v2(u, 0.0f));
    }
    for (int s = 0; s < segments; s++) {
        uint32_t a = (uint32_t)s * 2;
        pix__mb_quad(b, a, a + 2, a + 3, a + 1);
    }
    for (int cap = 0; cap < 2; cap++) {                       // caps: centre + rim fan
        float y = cap ? hh : -hh;
        vec3 n = v3(0.0f, cap ? 1.0f : -1.0f, 0.0f);
        uint32_t centre = (uint32_t)b.nv;
        pix__mb_vert(b, v3(0, y, 0), n, v2(0.5f, 0.5f));
        for (int s = 0; s <= segments; s++) {
            float a = (float)s / (float)segments * PIX_TAU;
            pix__mb_vert(b, v3(cosf(a) * radius, y, -sinf(a) * radius), n,
                         v2(cosf(a) * 0.5f + 0.5f, sinf(a) * 0.5f + 0.5f));
        }
        for (int s = 0; s < segments; s++) {
            uint32_t k = centre + 1 + (uint32_t)s;
            b.i[b.ni++] = centre;
            b.i[b.ni++] = cap ? k : k + 1;
            b.i[b.ni++] = cap ? k + 1 : k;
        }
    }
    idx id = load_mesh_data(r, b.v, b.nv, b.i, b.ni);
    pix_free(b.v); pix_free(b.i);
    return id;
}

// ---------------- materials ----------------

material pix_material_default(void) {
    material m = {};
    m.color = v3(1.0f, 1.0f, 1.0f);
    m.metallic = 0.0f;
    m.roughness = 0.6f;
    m.alpha = 1.0f;
    m.alpha_cutoff = 0.5f;    // cut-out cards work out of the box; opaque textures have alpha 1
    m.blend = PIX_BLEND_OPAQUE;
    m.cast_shadows = true;
    m.planar_reflection = 0.0f;
    m.refraction = 0.0f;
    m.toon = 0.0f;
    m.toon_bands = 3.0f;
    m.rim = 0.0f;
    m.toon_specular = 0.0f;
    return m;
}

idx pix_add_material(pix_renderer& r, const material& m) {
    // An overrun here would corrupt whatever follows the table and fail frames
    // later somewhere else entirely; refuse loudly instead.
    if (r.material_count >= PIX_MAX_MATERIALS) {
        pix_log("pix: material table full (PIX_MAX_MATERIALS = %d)", PIX_MAX_MATERIALS);
        return (idx)(r.material_count ? 0 : PIX_INVALID);
    }
    idx id = (idx)r.material_count++;
    r.materials[id] = m;
    return id;
}

material* pix_get_material(pix_renderer& r, idx material_id) {
    return material_id < r.material_count ? &r.materials[material_id] : 0;
}

idx load_material(pix_renderer& r, const char* texture_path, vec3 color, float metalic,
                  float roughness, idx texture_flags) {
    material m = pix_material_default();
    m.color = color;
    m.metallic = metalic;
    m.roughness = roughness;
    if (texture_path) m.texture = pix_load_texture(texture_path, texture_flags).id;
    return pix_add_material(r, m);
}

idx load_material_image(pix_renderer& r, image_file_data* image, vec3 color, float metalic,
                        float roughness, bool pixelated) {
    material m = pix_material_default();
    m.color = color;
    m.metallic = metalic;
    m.roughness = roughness;
    // A generated palette must stay unfiltered and unmipmapped - each cell is a
    // whole material's colour, and blending two is a colour that is not in the file.
    if (image && image->data)
        m.texture = opengl_create_texture2d(image->width, image->height, 4, image->data,
                                            pixelated ? TEXTURE_PIXELATED : TEXTURE_BILINEAR);
    return pix_add_material(r, m);
}

idx clone_material(pix_renderer& r, idx source, vec3 color, float metallic, float roughness) {
    if (source >= r.material_count) return source;
    material m = r.materials[source];   // texture shared, not re-uploaded
    m.color = color;
    m.metallic = metallic;
    m.roughness = roughness;
    return pix_add_material(r, m);
}

idx get_default_material(pix_renderer& r) {
    return pix_add_material(r, pix_material_default());
}

idx load_shader(pix_renderer& r, const char* vsrc, const char* fsrc) {
    idx prog = opengl_create_shader(vsrc, fsrc);
    if (prog && r.shader_count < MAX_SHADERS) r.shaders[r.shader_count++] = prog;
    return prog;
}

// uViewProj is the scene block's camera; inside the twin it names the light's matrix
static const char* const PIX__DEPTH_REDIRECT =
    "#define PIX_DEPTH_PASS 1\n"
    "uniform mat4 uLightViewProj;\n"
    "#undef uViewProj\n"
    "#define uViewProj uLightViewProj\n";

idx load_surface_shader(pix_renderer& r, const char* fsrc, const char* vsrc) {
    idx prog = pix__compile(vsrc ? vsrc : VSHDER_BASIC, fsrc, true);
    if (prog && r.shader_count < MAX_SHADERS) r.shaders[r.shader_count++] = prog;
    // clip planes: one past the highest gl_ClipDistance[i] the vertex stage writes
    int clips = 0;
    for (const char* c = vsrc ? strstr(vsrc, "gl_ClipDistance[") : 0; c; c = strstr(c + 1, "gl_ClipDistance[")) {
        char d = c[16];
        if (d >= '0' && d <= '7' && d - '0' + 1 > clips) clips = d - '0' + 1;
    }
    if (prog && clips && r.clip_program_count < MAX_SHADERS) r.clip_programs[r.clip_program_count++] = { prog, clips };
    if (prog && vsrc && r.depth_twin_count < MAX_SHADERS) {
        // the same chunks the main vertex stage gets (see pix__compile), plus the redirect
        const char* chunks[] = { GLSL_SCENE, GLSL_COMMON, PIX__DEPTH_REDIRECT };
        char* v = pix_shader_splice(vsrc, chunks, 3);
        idx depth = v ? opengl_create_shader(v, FSHDER_DEPTH) : 0;
        pix_free(v);
        if (depth) {
            r.depth_twins[r.depth_twin_count].surface = prog;
            r.depth_twins[r.depth_twin_count++].depth = depth;
            if (clips && r.clip_program_count < MAX_SHADERS) r.clip_programs[r.clip_program_count++] = { depth, clips };
            if (r.shader_count < MAX_SHADERS) r.shaders[r.shader_count++] = depth;
        } else {
            pix_log("pix: the vertex shader has no shadow-pass twin; its shadows stay static");
        }
    }
    return prog;
}

static idx pix__depth_program(const pix_renderer& r, const material& m) {
    if (!m.shader) return r.depth_shader;
    for (size_t i = 0; i < r.depth_twin_count; i++)
        if (r.depth_twins[i].surface == m.shader) return r.depth_twins[i].depth;
    return r.depth_shader;
}

static bool pix__material_uniform(pix_renderer& r, idx material_id, const char* name, int components,
                                  const float* v) {
    if (material_id >= r.material_count || !name) return false;
    material& m = r.materials[material_id];
    GLuint program = m.shader ? m.shader : r.shader;
    GLint loc = glGetUniformLocation(program, name);
    if (loc < 0) { pix_log("pix: material uniform '%s' not found in its shader", name); return false; }
    int slot = -1;
    for (int i = 0; i < m.uniform_count; i++) if (m.uniforms[i].location == loc) slot = i;
    if (slot < 0) {
        if (m.uniform_count >= PIX_MAX_MATERIAL_UNIFORMS) {
            pix_log("pix: material has more than %d named uniforms", PIX_MAX_MATERIAL_UNIFORMS);
            return false;
        }
        slot = m.uniform_count++;
    }
    pix_material_uniform& u = m.uniforms[slot];
    u.location = loc;
    u.components = components;
    int n = components < 0 ? 1 : components;
    for (int i = 0; i < 4; i++) u.value[i] = i < n ? v[i] : 0.0f;
    return true;
}

bool pix_material_set_float(pix_renderer& r, idx m, const char* n, float v) { return pix__material_uniform(r, m, n, 1, &v); }
bool pix_material_set_vec2(pix_renderer& r, idx m, const char* n, vec2 v)   { return pix__material_uniform(r, m, n, 2, &v.x); }
bool pix_material_set_vec3(pix_renderer& r, idx m, const char* n, vec3 v)   { return pix__material_uniform(r, m, n, 3, &v.x); }
bool pix_material_set_vec4(pix_renderer& r, idx m, const char* n, vec4 v)   { return pix__material_uniform(r, m, n, 4, &v.x); }
bool pix_material_set_int(pix_renderer& r, idx m, const char* n, int v) {
    float f = (float)v;
    return pix__material_uniform(r, m, n, -1, &f);
}

void pix_material_set_texture(pix_renderer& r, idx material_id, int slot, idx gl_texture) {
    if (material_id >= r.material_count || slot < 1 || slot > 4) return;
    r.materials[material_id].textures[slot - 1] = gl_texture;
}
// ---------------- per-frame submission ----------------

static mat4 pix__projection(const pix_renderer& r, const camera& cam, float aspect, float* out_far) {
    float n = cam.near_plane > 0.0f ? cam.near_plane : 0.1f;
    float f = cam.far_plane > 0.0f ? cam.far_plane : 1000.0f;
    *out_far = f;
    if (cam.ortho_height > 0.0f) {
        float h = cam.ortho_height * 0.5f, w = h * aspect;
        return mat4_ortho(-w, w, -h, h, n, f);
    }
    return mat4_perspective(cam.fovy > 0.0f ? cam.fovy : r.default_fovy, aspect, n, f);
}

// Fits one orthographic box per cascade around the slice of the view it
// covers. Each slice is bounded by a sphere, so the box's size does not change
// as the camera turns, and its centre is snapped to whole shadow texels in the
// light's own frame, so it only ever moves in texel steps: the two things that
// stop shadow edges swimming as the camera moves.
static void pix__fit_cascades(pix_renderer& r, const camera& cam) {
    // which cascades are redrawn this frame: all of them, unless staggered -
    // and all of them whenever what they are fitted to has changed
    r.shadow_frame++;
    float far_scale = clampf(r.shadows.far_resolution, 0.25f, 1.0f);
    bool changed = r.cascade_fit_key[0] != (float)r.shadows.resolution * far_scale || r.cascade_fit_key[1] != r.shadows.distance
                || r.cascade_fit_key[2] != r.shadows.depth_range;
    r.cascade_fit_key[0] = (float)r.shadows.resolution * far_scale;
    r.cascade_fit_key[1] = r.shadows.distance;
    r.cascade_fit_key[2] = r.shadows.depth_range;
    int all = (1 << SHADOW_CASCADES) - 1;
    r.cascade_mask = (!r.shadows.stagger || changed || r.shadow_frame < 3) ? all : (1 | ((r.shadow_frame & 1) ? 2 : 4));
    int tile = r.shadows.resolution;
    float dist = r.shadows.distance > 1.0f ? r.shadows.distance : 1.0f;
    float n = cam.near_plane > 0.0f ? cam.near_plane : 0.1f;
    float aspect = (float)r.width / (float)r.height;
    float fovy = cam.fovy > 0.0f ? cam.fovy : r.default_fovy;
    float ty = tanf(fovy * 0.5f), tx = ty * aspect;
    float diag = sqrtf(1.0f + tx * tx + ty * ty);   // corner distance per unit of view depth

    vec3 sun = v3norm(r.sun.direction);
    vec3 up = fabsf(sun.y) > 0.99f ? v3(0.0f, 0.0f, 1.0f) : v3(0.0f, 1.0f, 0.0f);
    mat4 rot = mat4_lookat(v3(0, 0, 0), v3neg(sun), up);
    mat4 rot_inv = mat4_transpose(rot);

    float prev = n;
    for (int c = 0; c < SHADOW_CASCADES; c++) {
        // practical split: mostly logarithmic, a little uniform
        float t = (float)(c + 1) / (float)SHADOW_CASCADES;
        float split = lerpf(n + (dist - n) * t, n * powf(dist / n, t), 0.8f);
        float near_d = prev, far_d = split;
        prev = split;
        r.cascade_far[c] = far_d;

        // bounding sphere of the slice: centred on the axis, reaching its far corners
        float mid = (near_d + far_d) * 0.5f;
        vec3 centre = v3add(cam.position, v3scale(v3norm(cam.direction), mid));
        float rn = sqrtf((mid - near_d) * (mid - near_d) + near_d * near_d * (diag * diag - 1.0f));
        float rf = sqrtf((far_d - mid) * (far_d - mid) + far_d * far_d * (diag * diag - 1.0f));
        float radius = ceilf((rn > rf ? rn : rf) * 16.0f) / 16.0f;

        // a far cascade fills only part of its tile: its texels are that much bigger
        float scale = c == 0 ? 1.0f : far_scale;
        float texel = (radius * 2.0f) / ((float)tile * scale);
        vec3 lc = mat4_mul_point(rot, centre);
        lc.x = floorf(lc.x / texel) * texel;
        lc.y = floorf(lc.y / texel) * texel;
        vec3 snapped = mat4_mul_point(rot_inv, lc);

        float depth = r.shadows.depth_range > radius * 2.0f ? r.shadows.depth_range : radius * 2.0f;
        // a cascade not redrawn this frame keeps the box its depth was drawn in
        if (!(r.cascade_mask & (1 << c))) continue;
        vec3 eye = v3add(snapped, v3scale(sun, depth * 0.5f));
        mat4 view = mat4_lookat(eye, snapped, up);
        r.cascade_view[c] = view;
        r.cascade_radius[c] = radius;
        r.cascade_texel[c] = texel;
        r.cascade_scale[c] = scale;
        r.cascade_depth = depth;
        r.light_space[c] = mat4_mul(mat4_ortho(-radius, radius, -radius, radius, 0.0f, depth), view);
    }
}

// Makes `cam` the view being rendered: the matrices every draw helper reads.
static void pix__set_current_view(pix_renderer& r, const camera& cam, float aspect) {
    r.camera_position = cam.position;
    r.camera_dir = v3norm(cam.direction);
    r.camera_up = v3len(cam.up) > 1e-6f ? cam.up : v3(0.0f, 1.0f, 0.0f);
    r.view_matrix = mat4_lookat(cam.position, v3add(cam.position, r.camera_dir), r.camera_up);
    r.projection_matrix = pix__projection(r, cam, aspect, &r.camera_far);
    r.camera_near = cam.near_plane > 0.0f ? cam.near_plane : 0.1f;
    r.view_proj = mat4_mul(r.projection_matrix, r.view_matrix);
    r.view_frustum = frustum_from_viewproj(r.view_proj);
}

void begin_frame(pix_renderer& r, camera& cam) {
    pix__sync_settings(r);
    if (v3len(cam.up) < 1e-6f) cam.up = v3(0.0f, 1.0f, 0.0f);
    r.frame_camera = cam;
    pix__set_current_view(r, cam, (float)r.width / (float)r.height);
    r.frame_view_proj = r.view_proj;
    r.frame_prepared = false;
    r.renderable_count = 0;
    r.animated_count = 0;
    r.particle_count = 0;
    r.line_count = 0;
    pix_lights_clear(*r.lights);
    if (r.shadows.enabled) pix__fit_cascades(r, cam);
}

void push_instance(pix_renderer& r, pix_render_instance& instance) {
    if (r.renderable_count < PIX_MAX_INSTANCES && instance.mesh < r.mesh_count
        && instance.material < r.material_count)
        r.instances[r.renderable_count++] = instance;
}

void push_instance(pix_renderer& r, idx mesh, idx material, const mat4& transform, vec4 tint, uint32_t layers) {
    if (mesh >= r.mesh_count || material >= r.material_count) return;
    if (r.renderable_count >= PIX_MAX_INSTANCES) return;
    pix_render_instance& it = r.instances[r.renderable_count++];
    it.mesh = mesh;
    it.material = material;
    it.transform = transform;
    it.tint = tint;
    it.layers = layers;
}

void push_skinned(pix_renderer& r, idx mesh, idx material, const mat4& transform,
                  const mat4* bones, int bone_count, vec4 tint, uint32_t layers) {
    if (r.animated_count >= PIX_MAX_ANIMATED || mesh >= r.skinned_mesh_count || material >= r.material_count) return;
    // a pose with no bones would skin this mesh with garbage: drop the instance instead
    if (!bones || bone_count <= 0) return;
    pix_render_instance& it = r.animated_instances[r.animated_count];
    it.mesh = mesh;
    it.material = material;
    it.transform = transform;
    it.tint = tint;
    it.layers = layers;
    r.animated_bones[r.animated_count] = bones;
    r.animated_bone_count[r.animated_count] = (uint32_t)bone_count;
    r.animated_count++;
}

void push_animated_instance(pix_renderer& r, pix_render_instance& instance, const animation& pose) {
    push_skinned(r, instance.mesh, instance.material, instance.transform, pose.bones, (int)pose.bone_count,
                 instance.tint, instance.layers);
}

int pix_add_render_hook(pix_renderer& r, pix_render_hook fn, void* user, uint32_t layers) {
    if (!fn) return -1;
    for (int i = 0; i < PIX_MAX_HOOKS; i++)
        if (!r.hooks[i].fn) {
            r.hooks[i].fn = fn;
            r.hooks[i].user = user;
            r.hooks[i].layers = layers;
            if (i >= r.hook_count) r.hook_count = i + 1;
            return i;
        }
    return -1;
}

void pix_remove_render_hook(pix_renderer& r, int handle) {
    if (handle >= 0 && handle < PIX_MAX_HOOKS) r.hooks[handle].fn = 0;
}

idx pix_create_scene_shader(pix_renderer& r, const char* vs, const char* fs, bool lit) {
    idx prog = pix__compile(vs, fs, lit);
    if (prog) pix_setup_scene_program(prog);
    if (prog && r.shader_count < MAX_SHADERS) r.shaders[r.shader_count++] = prog;
    return prog;
}

void pix_setup_scene_program(idx p) {
    glUseProgram(p);
    const char* names[5] = { "uShadowMap", "uEnvMap", "uPlanarMap", "uRefractMap", "uSceneDepth" };
    for (int i = 0; i < 5; i++) {
        GLint loc = glGetUniformLocation(p, names[i]);
        if (loc >= 0) glUniform1i(loc, 1 + i);
    }
}

idx load_skinned_surface_shader(pix_renderer& r, const char* fsrc) {
    idx prog = pix__compile_skinned(VSHDER_SKINNED, fsrc, true);
    if (prog && r.shader_count < MAX_SHADERS) r.shaders[r.shader_count++] = prog;
    return prog;
}

void push_particle(pix_renderer& r, vec3 at, float size, vec3 color, float alpha) {
    if (r.particle_count >= PIX_MAX_PARTICLES || alpha <= 0.0f || size <= 0.0f) return;
    pix_particle_vertex& v = r.particles[r.particle_count++];
    v.centre[0] = at.x; v.centre[1] = at.y; v.centre[2] = at.z;
    v.size = size;
    v.color[0] = color.x; v.color[1] = color.y; v.color[2] = color.z; v.color[3] = alpha;
}

void push_line(pix_renderer& r, vec3 a, vec3 b, pix_color color) {
    if (r.line_count + 2 > PIX_MAX_LINES * 2) return;
    pix__line_vertex* v = &r.lines[r.line_count];
    v[0].pos[0] = a.x; v[0].pos[1] = a.y; v[0].pos[2] = a.z; v[0].color = color;
    v[1].pos[0] = b.x; v[1].pos[1] = b.y; v[1].pos[2] = b.z; v[1].color = color;
    r.line_count += 2;
}

void push_aabb_lines(pix_renderer& r, vec3 lo, vec3 hi, pix_color c) {
    vec3 p[8];
    for (int i = 0; i < 8; i++) p[i] = v3((i & 1) ? hi.x : lo.x, (i & 2) ? hi.y : lo.y, (i & 4) ? hi.z : lo.z);
    static const int E[12][2] = { {0,1},{2,3},{4,5},{6,7},{0,2},{1,3},{4,6},{5,7},{0,4},{1,5},{2,6},{3,7} };
    for (int e = 0; e < 12; e++) push_line(r, p[E[e][0]], p[E[e][1]], c);
}

void push_grid_lines(pix_renderer& r, vec3 center, float size, int cells, pix_color c) {
    if (cells < 1) cells = 1;
    float h = size * 0.5f, step = size / (float)cells;
    for (int i = 0; i <= cells; i++) {
        float o = -h + step * (float)i;
        push_line(r, v3add(center, v3(o, 0, -h)), v3add(center, v3(o, 0, h)), c);
        push_line(r, v3add(center, v3(-h, 0, o)), v3add(center, v3(h, 0, o)), c);
    }
}

bool pix_world_to_screen(const pix_renderer& r, vec3 world, vec2* out) {
    vec4 clip = mat4_mul_vec4(r.view_proj, v4(world.x, world.y, world.z, 1.0f));
    if (clip.w <= 1e-6f) return false;
    float x = clip.x / clip.w, y = clip.y / clip.w;
    out->x = (x * 0.5f + 0.5f) * (float)r.width;
    out->y = (0.5f - y * 0.5f) * (float)r.height;
    return true;
}

vec3 pix_screen_to_ray(const pix_renderer& r, float x, float y) {
    mat4 inv = mat4_inverse(r.view_proj);
    float nx = x / (float)r.width * 2.0f - 1.0f, ny = 1.0f - y / (float)r.height * 2.0f;
    vec4 a = mat4_mul_vec4(inv, v4(nx, ny, -1.0f, 1.0f));
    vec4 b = mat4_mul_vec4(inv, v4(nx, ny, 1.0f, 1.0f));
    vec3 pa = v3(a.x / a.w, a.y / a.w, a.z / a.w), pb = v3(b.x / b.w, b.y / b.w, b.z / b.w);
    return v3norm(v3sub(pb, pa));
}

// ---------------- timing ----------------

void pix_enable_gpu_timing(pix_renderer& r, bool on) {
    if (on == r.gpu_timing) return;
    if (on) {
        for (int i = 0; i < PIX_GPU_PASSES; i++) glGenQueries(2, r.gpu_query[i]);
        r.gpu_slot = 0;
    } else {
        for (int i = 0; i < PIX_GPU_PASSES; i++) {
            glDeleteQueries(2, r.gpu_query[i]);
            r.gpu_query[i][0] = r.gpu_query[i][1] = 0;
            r.gpu_ms[i] = 0.0;
        }
    }
    r.gpu_timing = on;
}

double pix_gpu_pass_ms(const pix_renderer& r, int pass) {
    return (unsigned)pass < PIX_GPU_PASSES ? r.gpu_ms[pass] : 0.0;
}

double pix_cpu_pass_ms(const pix_renderer& r, int pass) {
    return (unsigned)pass < PIX_GPU_PASSES ? r.cpu_ms[pass] : 0.0;
}

static double pix__cpu_seconds() {
    LARGE_INTEGER f, t;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&t);
    return (double)t.QuadPart / (double)f.QuadPart;
}

// Brackets a pass. Queries are read a frame late, so nothing ever stalls on a result.
static void pix__gpu_begin(pix_renderer& r, int pass) {
    if (!r.gpu_timing) return;
    r.cpu_begin = (long long)(pix__cpu_seconds() * 1.0e6);
    glBeginQuery(GL_TIME_ELAPSED, r.gpu_query[pass][r.gpu_slot]);
}

static void pix__gpu_end(pix_renderer& r, int pass) {
    if (!r.gpu_timing) return;
    glEndQuery(GL_TIME_ELAPSED);
    double cpu = (pix__cpu_seconds() * 1.0e6 - (double)r.cpu_begin) / 1000.0;
    r.cpu_ms[pass] += (cpu - r.cpu_ms[pass]) * 0.1;
    GLuint previous = r.gpu_query[pass][r.gpu_slot ^ 1];
    if (!previous) return;
    GLint ready = 0;
    glGetQueryObjectiv(previous, GL_QUERY_RESULT_AVAILABLE, &ready);
    if (!ready) return;
    GLuint64 ns = 0;
    glGetQueryObjectui64v(previous, GL_QUERY_RESULT, &ns);
    r.gpu_ms[pass] += ((double)ns / 1.0e6 - r.gpu_ms[pass]) * 0.1;
}

// ---------------- sorting ----------------

// LSD radix sort of (key, value) pairs, 8 bits a pass, skipping every pass
// whose digit is the same for all keys - which for keys made of a few small
// fields is most of them. Linear in the count, no comparisons, stable.
static void pix__radix_sort(pix_renderer& r, size_t n) {
    uint64_t* k = r.sort_keys;     uint64_t* kt = r.sort_keys_tmp;
    uint32_t* v = r.sort_vals;     uint32_t* vt = r.sort_vals_tmp;
    for (int shift = 0; shift < 64; shift += 8) {
        size_t count[257] = {};
        for (size_t i = 0; i < n; i++) count[((k[i] >> shift) & 255) + 1]++;
        bool trivial = false;
        for (int d = 1; d <= 256; d++) if (count[d] == n) { trivial = true; break; }
        if (trivial) continue;
        for (int d = 0; d < 256; d++) count[d + 1] += count[d];
        for (size_t i = 0; i < n; i++) {
            size_t at = count[(k[i] >> shift) & 255]++;
            kt[at] = k[i];
            vt[at] = v[i];
        }
        uint64_t* tk = k; k = kt; kt = tk;
        uint32_t* tv = v; v = vt; vt = tv;
    }
    if (v != r.sort_vals) memcpy(r.sort_vals, v, n * sizeof(uint32_t));
}

static uint64_t pix__batch_key(const pix_render_instance& it) {
    return ((uint64_t)(it.material & 0xFFFF) << 48) | ((uint64_t)(it.mesh & 0xFFFF) << 32);
}

static uint32_t pix__depth16(float d, float far_plane) {
    float t = d / (far_plane > 1e-3f ? far_plane : 1.0f);
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return (uint32_t)(t * 65535.0f);
}

// ---------------- the frame's shared data ----------------

// bits in pix_renderer::flags
#define PIX__HIDDEN    0x01   // shadow_only: in no view list
#define PIX__BLENDED   0x02
#define PIX__CASTER    0x04
#define PIX__REFRACT   0x08
#define PIX__CASCADE0  0x10   // << cascade index

static void pix__upload_lights(pix_renderer& r);

// Work every stage shares, done once per frame on first use: world bounds and
// classification of every instance, the light list, and a fresh instance
// stream (orphaned: the driver hands back new storage instead of waiting for
// last frame's draws to finish reading the old).
// Every skinned instance of the frame, uploaded once: all bone palettes packed
// into one buffer, and one record per instance (model, tint, first bone).
// Views then only upload small index lists naming which records they draw.
struct pix__skin_record { mat4 model; vec4 tint; uint32_t info[4]; };

static void pix__upload_skinning(pix_renderer& r) {
    r.skin_list_cursor = 0;
    if (!r.animated_count) return;
    size_t total = 0;
    for (size_t k = 0; k < r.animated_count; k++) total += r.animated_bone_count[k];
    if (total > r.skin_bone_staging_capacity) {
        size_t cap = r.skin_bone_staging_capacity ? r.skin_bone_staging_capacity : 4096;
        while (cap < total) cap *= 2;
        mat4* grown = (mat4*)pix_mem_realloc(r.skin_bone_staging, cap * sizeof(mat4));
        if (!grown) { r.animated_count = 0; return; }
        r.skin_bone_staging = grown;
        r.skin_bone_staging_capacity = cap;
    }
    // the records borrow the instance staging array, which is idle until the first view streams
    static_assert(sizeof(pix__gpu_instance) * PIX_MAX_INSTANCES >= sizeof(pix__skin_record) * PIX_MAX_ANIMATED,
                  "PIX_MAX_ANIMATED too large for the shared staging array");
    pix__skin_record* rec = (pix__skin_record*)r.staging;
    size_t at = 0;
    for (size_t k = 0; k < r.animated_count; k++) {
        uint32_t n = r.animated_bone_count[k];
        memcpy(r.skin_bone_staging + at, r.animated_bones[k], n * sizeof(mat4));
        rec[k].model = r.animated_instances[k].transform;
        rec[k].tint = r.animated_instances[k].tint;
        rec[k].info[0] = (uint32_t)at;
        rec[k].info[1] = rec[k].info[2] = rec[k].info[3] = 0;
        at += n;
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, r.skin_bones_ssbo);
    if (total > r.skin_bones_capacity) r.skin_bones_capacity = total * 2;
    glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)(r.skin_bones_capacity * sizeof(mat4)), 0, GL_STREAM_DRAW);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, (GLsizeiptr)(total * sizeof(mat4)), r.skin_bone_staging);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, r.skin_inst_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)(r.animated_count * sizeof(pix__skin_record)), rec, GL_STREAM_DRAW);
    // index lists: a few views and cascades per frame, each at most every instance
    size_t list_cap = r.animated_count * 16;
    if (list_cap > r.skin_list_capacity) r.skin_list_capacity = list_cap;
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, r.skin_list_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)(r.skin_list_capacity * sizeof(uint32_t)), 0, GL_STREAM_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}
static void pix__prepare_frame(pix_renderer& r) {
    if (r.frame_prepared) return;
    pix__sync_settings(r);
    r.frame_draws = r.frame_instances = r.frame_culled = 0;
    for (size_t i = 0; i < r.renderable_count; i++) {
        const pix_render_instance& it = r.instances[i];
        const mesh& m = r.meshes[it.mesh];
        const material& mat = r.materials[it.material];
        vec3 c = mat4_mul_point(it.transform, m.center);
        r.bounds[i] = v4(c.x, c.y, c.z, m.radius * mat4_max_scale(it.transform));
        bool blended = mat.blend != PIX_BLEND_OPAQUE || it.tint.w < 0.999f || mat.refraction > 0.0f;
        uint8_t f = 0;
        if (blended) f |= PIX__BLENDED;
        if (mat.shadow_only) f |= PIX__HIDDEN;
        if (mat.refraction > 0.0f) f |= PIX__REFRACT;
        if (!blended && mat.cast_shadows) f |= PIX__CASTER;
        r.flags[i] = f;
    }
    pix__upload_lights(r);
    glBindBuffer(GL_ARRAY_BUFFER, r.instances_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(r.instance_capacity * sizeof(pix__gpu_instance)), 0, GL_STREAM_DRAW);
    r.instance_cursor = 0;
    pix__upload_skinning(r);
    r.frame_prepared = true;
}

// Uploads a sorted list's transforms into the instance stream and records
// where they landed; the draws then name that slice with baseInstance. No two
// uploads in a frame overlap, so none waits on a draw. When the frame outgrows
// the buffer, it is re-specified larger: draws already issued keep the storage
// they were issued against, so nothing in flight is disturbed.
static void pix__stream_list(pix_renderer& r, pix__draw_list& list) {
    list.base = 0;
    if (!list.count) return;
    glBindBuffer(GL_ARRAY_BUFFER, r.instances_vbo);
    if (r.instance_cursor + list.count > r.instance_capacity) {
        while (list.count > r.instance_capacity || r.instance_capacity < r.instance_cursor + list.count)
            r.instance_capacity *= 2;
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(r.instance_capacity * sizeof(pix__gpu_instance)), 0, GL_STREAM_DRAW);
        r.instance_cursor = 0;
    }
    for (size_t i = 0; i < list.count; i++) {
        const pix_render_instance& it = r.instances[list.items[i]];
        r.staging[i].model = it.transform;
        r.staging[i].tint = it.tint;
    }
    glBufferSubData(GL_ARRAY_BUFFER, (GLintptr)(r.instance_cursor * sizeof(pix__gpu_instance)),
                    (GLsizeiptr)(list.count * sizeof(pix__gpu_instance)), r.staging);
    list.base = (GLuint)r.instance_cursor;
    r.instance_cursor += list.count;
}

// A view's opaque list (by batch, front to back inside it for early depth
// rejection) and blended list (strictly back to front), culled against its
// frustum, its layers and its distance cap.
static void pix__build_view_lists(pix_renderer& r, const frustum& fr, vec3 eye, float max_dist, float far_plane,
                                  uint32_t layers, bool opaque, bool blended_too) {
    size_t n = r.renderable_count, count = 0;
    r.refract_needed = false;
    if (opaque) {
        for (size_t i = 0; i < n; i++) {
            if ((r.flags[i] & (PIX__BLENDED | PIX__HIDDEN)) || !(r.instances[i].layers & layers)) continue;
            vec3 c = v3(r.bounds[i].x, r.bounds[i].y, r.bounds[i].z);
            float d = v3dist(c, eye);
            if (d - r.bounds[i].w > max_dist || !frustum_test_sphere(fr, c, r.bounds[i].w)) { r.frame_culled++; continue; }
            r.sort_keys[count] = pix__batch_key(r.instances[i]) | ((uint64_t)pix__depth16(d, far_plane) << 16);
            r.sort_vals[count++] = (uint32_t)i;
        }
        pix__radix_sort(r, count);
        memcpy(r.list_opaque.items, r.sort_vals, count * sizeof(uint32_t));
    }
    r.list_opaque.count = count;

    count = 0;
    if (blended_too) {
        for (size_t i = 0; i < n; i++) {
            if (!(r.flags[i] & PIX__BLENDED) || (r.flags[i] & PIX__HIDDEN) || !(r.instances[i].layers & layers)) continue;
            vec3 c = v3(r.bounds[i].x, r.bounds[i].y, r.bounds[i].z);
            float d = v3dist(c, eye);
            if (d - r.bounds[i].w > max_dist || !frustum_test_sphere(fr, c, r.bounds[i].w)) { r.frame_culled++; continue; }
            if (r.flags[i] & PIX__REFRACT) r.refract_needed = true;
            uint32_t back = 65535u - pix__depth16(d, far_plane);
            r.sort_keys[count] = ((uint64_t)back << 48) | (pix__batch_key(r.instances[i]) >> 16);
            r.sort_vals[count++] = (uint32_t)i;
        }
        pix__radix_sort(r, count);
        memcpy(r.list_transparent.items, r.sort_vals, count * sizeof(uint32_t));
    }
    r.list_transparent.count = count;
}

// cube face orientations in the order GL numbers them (+X -X +Y -Y +Z -Z)
static void pix__probe_face(int face, vec3* dir, vec3* up) {
    static const vec3 dirs[6] = { {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1} };
    static const vec3 ups[6]  = { {0,-1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}, {0,-1,0}, {0,-1,0} };
    *dir = dirs[face];
    *up = ups[face];
}

// ---------------- uniform blocks ----------------

static void pix__put4(float* d, vec3 v, float w) { d[0] = v.x; d[1] = v.y; d[2] = v.z; d[3] = w; }
static vec3 pix__lin(vec3 c) { return v3(powf(c.x, 2.2f), powf(c.y, 2.2f), powf(c.z, 2.2f)); }

// The scene block for the current view (pix__set_current_view). `planar` and
// `probe` say which reflection sources this view may sample - a probe face must
// never sample the probe it is being drawn into.
static void pix__upload_scene(pix_renderer& r, bool planar, bool probe, vec4 clip, int target_w, int target_h) {
    pix_scene_block b;
    memset(&b, 0, sizeof(b));
    memcpy(b.view_proj, r.view_proj.data, sizeof(b.view_proj));
    memcpy(b.view, r.view_matrix.data, sizeof(b.view));
    pix__put4(b.sun_dir, v3norm(r.sun.direction), 0.0f);
    pix__put4(b.sun_color, r.sun.color, 0.0f);
    pix__put4(b.sky_color, r.sky.diffuse_up, 0.0f);
    pix__put4(b.ground_color, r.sky.diffuse_down, 0.0f);
    pix__put4(b.sky_zenith, r.sky.zenith, 0.0f);
    pix__put4(b.sky_horizon, r.sky.horizon, 0.0f);
    pix__put4(b.sky_ground, r.sky.ground, 0.0f);
    pix__put4(b.fog, pix__lin(r.sky.fog), r.sky.fog_density);
    pix__put4(b.camera, r.camera_position, r.time);
    pix__put4(b.camera_dir, r.camera_dir, 0.0f);
    bool shadows = r.shadows.enabled && r.shadow_map.fbo;
    b.shadow[0] = shadows ? 1.0f / (float)r.shadow_map.width : 0.0f;
    b.shadow[1] = shadows ? 1.0f / (float)r.shadow_map.height : 0.0f;
    b.shadow[2] = shadows ? clampf(r.shadows.strength, 0.0f, 1.0f) : 0.0f;
    b.shadow[3] = (float)(r.shadows.softness < 0 ? 0 : (r.shadows.softness > 2 ? 2 : r.shadows.softness));
    for (int c = 0; c < SHADOW_CASCADES; c++) {
        // looked up through the part of the tile it was drawn into: x, y in
        // [-1, 1] squeezed toward the tile's corner by its scale
        float sc = r.cascade_scale[c] > 0.0f ? r.cascade_scale[c] : 1.0f;
        mat4 squeeze = mat4_identity();
        squeeze.data[0] = sc; squeeze.data[5] = sc;
        squeeze.data[12] = sc - 1.0f; squeeze.data[13] = sc - 1.0f;
        mat4 lookup = mat4_mul(squeeze, r.light_space[c]);
        memcpy(b.light_space[c], lookup.data, sizeof(float) * 16);
        b.cascade_far[c] = r.cascade_far[c];
        b.cascade_texel[c] = r.cascade_texel[c];
    }
    memcpy(b.refl_view_proj, r.planar_view_proj.data, sizeof(b.refl_view_proj));
    float len = sqrtf(clip.x * clip.x + clip.y * clip.y + clip.z * clip.z);
    if (len > 1e-6f) {
        b.clip_plane[0] = clip.x / len; b.clip_plane[1] = clip.y / len; b.clip_plane[2] = clip.z / len;
        b.clip_plane[3] = clip.w / len;
    }
    b.reflect[0] = (planar && r.planar_active) ? 1.0f : 0.0f;
    b.reflect[1] = (probe && r.env_cube && r.env_faces_done >= 6) ? 1.0f : 0.0f;
    b.reflect[2] = r.env_resolution > 1 ? log2f((float)r.env_resolution) : 0.0f;
    b.reflect[3] = r.reflections.probe_intensity;
    b.screen[0] = 1.0f / (float)(target_w > 0 ? target_w : 1);
    b.screen[1] = 1.0f / (float)(target_h > 0 ? target_h : 1);
    b.screen[2] = r.camera_near;
    b.screen[3] = r.camera_far;
    // cascades are fitted around the frame camera, whichever view is drawing
    pix__put4(b.shadow_eye, r.frame_camera.position, 0.0f);
    pix__put4(b.shadow_dir, v3norm(r.frame_camera.direction), 0.0f);
    glBindBuffer(GL_UNIFORM_BUFFER, r.scene_ubo);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, (GLsizeiptr)sizeof(b), &b);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

static void pix__upload_lights(pix_renderer& r) {
    pix_lights_pack(*r.lights, r.frame_camera.position);
    pix_light_block lb;
    memcpy(lb.pos_radius, r.lights->pos_radius, sizeof(lb.pos_radius));
    memcpy(lb.color_inner, r.lights->color_inner, sizeof(lb.color_inner));
    memcpy(lb.dir_outer, r.lights->dir_outer, sizeof(lb.dir_outer));
    lb.meta[0] = (float)r.lights->packed_count;
    lb.meta[1] = lb.meta[2] = lb.meta[3] = 0.0f;
    glBindBuffer(GL_UNIFORM_BUFFER, r.lights_ubo);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, (GLsizeiptr)sizeof(lb), &lb);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

// The textures every surface shader may sample, on their fixed units. What a
// view must not read (the probe while drawing into it) gets a harmless stand-in
// of the same type, since a sampler must always see a matching texture.
static void pix__bind_frame_textures(pix_renderer& r, bool shadows, bool env, bool planar, bool refract) {
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, (shadows && r.shadow_map.depth) ? r.shadow_map.depth : r.dummy_shadow);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_CUBE_MAP, (env && r.env_cube) ? r.env_cube : r.dummy_cube);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, (planar && r.planar_target.color) ? r.planar_target.color : r.white_texture);
    glActiveTexture(GL_TEXTURE0 + 4);
    glBindTexture(GL_TEXTURE_2D, (refract && r.refract_tex) ? r.refract_tex : r.white_texture);
    glActiveTexture(GL_TEXTURE0 + 5);
    glBindTexture(GL_TEXTURE_2D, (refract && r.refract_depth) ? r.refract_depth : r.white_texture);
    glActiveTexture(GL_TEXTURE0);
}

// ---------------- drawing ----------------

// What the colour passes need to know about the state they left behind, so a
// run of instances sharing a program or material does not re-send it.
struct pix__pass_state {
    GLuint program;
    pix__program_uniforms* u;
    idx material;
    int cull;          // -1 unknown, 0 off, 1 back faces
    int blend;         // -1 unknown, else PIX_BLEND_*
};

static void pix__set_cull(pix__pass_state& st, bool cull) {
    int want = cull ? 1 : 0;
    if (st.cull == want) return;
    if (cull) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
    st.cull = want;
}

// Clipping is only on while a program that writes gl_ClipDistance draws: any
// other shader would leave the distance undefined and lose random triangles.
static void pix__set_clip(pix_renderer& r, int planes) {
    if (r.clip_on == planes) return;
    for (int i = 0; i < 8; i++) {
        bool was = i < r.clip_on, want = i < planes;
        if (want && !was) glEnable(GL_CLIP_DISTANCE0 + i);
        if (was && !want) glDisable(GL_CLIP_DISTANCE0 + i);
    }
    r.clip_on = planes;
}

static void pix__use_program(pix_renderer& r, pix__pass_state& st, GLuint program) {
    if (st.program == program) { if (st.u) pix__set_clip(r, st.u->clips); return; }
    glUseProgram(program);
    st.program = program;
    st.u = pix__uniforms_for(r, program);
    pix__set_clip(r, st.u->clips);
    st.material = PIX_INVALID;
    if (st.u->tex >= 0) glUniform1i(st.u->tex, 0);
    if (st.u->shadow_map >= 0) glUniform1i(st.u->shadow_map, 1);
    // fixed texture units for the whole frame: 2 probe, 3 planar mirror,
    // 4 refraction colour, 5 scene depth, 6..9 the material's extra textures
    if (st.u->env_map >= 0) glUniform1i(st.u->env_map, 2);
    if (st.u->planar_map >= 0) glUniform1i(st.u->planar_map, 3);
    if (st.u->refract_map >= 0) glUniform1i(st.u->refract_map, 4);
    if (st.u->scene_depth >= 0) glUniform1i(st.u->scene_depth, 5);
    for (int i = 0; i < 4; i++) if (st.u->extra_tex[i] >= 0) glUniform1i(st.u->extra_tex[i], 6 + i);
}

static void pix__bind_material(pix_renderer& r, pix__pass_state& st, idx material_id, bool blended) {
    if (st.material == material_id) return;
    const material& m = r.materials[material_id];
    pix__program_uniforms* u = st.u;
    glUniform4f(u->color, m.color.x, m.color.y, m.color.z, m.alpha);
    glUniform1f(u->metallic, m.metallic);
    glUniform1f(u->roughness, m.roughness);
    glUniform3f(u->emissive, m.emissive.x, m.emissive.y, m.emissive.z);
    // blended surfaces keep their soft edges; only fully clear texels go
    glUniform1f(u->cutoff, blended ? 0.002f : m.alpha_cutoff);
    glUniform1i(u->unlit, m.unlit ? 1 : 0);
    glUniform1f(u->planar, m.planar_reflection);
    glUniform1f(u->refraction, r.refract_tex ? m.refraction : 0.0f);
    if (u->params0 >= 0) glUniform4f(u->params0, m.params0.x, m.params0.y, m.params0.z, m.params0.w);
    if (u->params1 >= 0) glUniform4f(u->params1, m.params1.x, m.params1.y, m.params1.z, m.params1.w);
    if (u->toon >= 0) glUniform4f(u->toon, m.toon, m.toon_bands, m.rim, m.toon_specular);
    for (int i = 0; i < 4; i++)
        if (m.textures[i] && u->extra_tex[i] >= 0) {
            glActiveTexture(GL_TEXTURE6 + i);
            glBindTexture(GL_TEXTURE_2D, m.textures[i]);
            glActiveTexture(GL_TEXTURE0);
        }
    for (int i = 0; i < m.uniform_count; i++) {
        const pix_material_uniform& mu = m.uniforms[i];
        switch (mu.components) {
            case -1: glUniform1i(mu.location, (GLint)mu.value[0]); break;
            case 1:  glUniform1f(mu.location, mu.value[0]); break;
            case 2:  glUniform2f(mu.location, mu.value[0], mu.value[1]); break;
            case 3:  glUniform3f(mu.location, mu.value[0], mu.value[1], mu.value[2]); break;
            default: glUniform4f(mu.location, mu.value[0], mu.value[1], mu.value[2], mu.value[3]); break;
        }
    }
    glBindTexture(GL_TEXTURE_2D, m.texture ? m.texture : r.white_texture);
    pix__set_cull(st, !m.double_sided);
    if (blended) {
        // refractive surfaces composite the background themselves: no blending
        int mode = m.refraction > 0.0f ? PIX_BLEND_OPAQUE : m.blend;
        if (st.blend != mode) {
            if (mode == PIX_BLEND_ADDITIVE)           glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            else if (mode == PIX_BLEND_ALPHA)         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            else if (mode == PIX_BLEND_MULTIPLY)      glBlendFunc(GL_DST_COLOR, GL_ONE_MINUS_SRC_ALPHA);
            else if (mode == PIX_BLEND_PREMULTIPLIED) glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
            else                                      glBlendFunc(GL_ONE, GL_ZERO);
            st.blend = mode;
        }
    }
    st.material = material_id;
}

static void pix__draw_run(pix_renderer& r, const mesh& me, GLuint base_instance, size_t count) {
    r.frame_draws++;
    r.frame_instances += (int)count;
    glDrawElementsInstancedBaseVertexBaseInstance(
        GL_TRIANGLES, (GLsizei)me.index_count, GL_UNSIGNED_INT,
        (void*)(size_t)(me.index_offset * sizeof(uint32_t)), (GLsizei)count,
        (GLint)me.vertex_offset, base_instance);
}

// One instanced draw per (material, mesh) run of a sorted list. `override`
// replaces every material's program (ID / normal / outline buffers).
// The depth of the view's cut-out surfaces alone, drawn with their cheap depth
// programs. Their lit pass then runs with depth writes off: a shader that may
// discard gets no early depth test while it writes depth, so without this
// every leaf card stacked behind the nearest one is fully shaded and thrown
// away. Pushed back a hair so the lit pass, computed by another program,
// always passes where it should. Returns whether there were any.
static bool pix__cutout_prepass(pix_renderer& r, const pix__draw_list& list, pix__pass_state& st) {
    if (!r.cutout_prepass || !list.count) return false;
    bool any = false;
    GLuint sent = 0;
    size_t i = 0;
    while (i < list.count) {
        const pix_render_instance& a = r.instances[list.items[i]];
        size_t j = i + 1;
        while (j < list.count && r.instances[list.items[j]].mesh == a.mesh && r.instances[list.items[j]].material == a.material) j++;
        const material& m = r.materials[a.material];
        if (m.cutout && m.texture) {
            if (!any) {
                glBindVertexArray(r.pool.vao);
                glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                glEnable(GL_POLYGON_OFFSET_FILL);
                glPolygonOffset(1.0f, 1.0f);
                any = true;
            }
            GLuint prog = (GLuint)pix__depth_program(r, m);
            pix__use_program(r, st, prog);
            if (sent != prog) { glUniformMatrix4fv(st.u->light_view_proj, 1, GL_FALSE, r.view_proj.data); sent = prog; }
            glUniform1f(st.u->cutoff, m.alpha_cutoff);
            if (st.u->params0 >= 0) glUniform4f(st.u->params0, m.params0.x, m.params0.y, m.params0.z, m.params0.w);
            if (st.u->params1 >= 0) glUniform4f(st.u->params1, m.params1.x, m.params1.y, m.params1.z, m.params1.w);
            glBindTexture(GL_TEXTURE_2D, m.texture);
            pix__set_cull(st, !m.double_sided);
            pix__draw_run(r, r.meshes[a.mesh], list.base + (GLuint)i, j - i);
        }
        i = j;
    }
    if (any) {
        glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
        glDisable(GL_POLYGON_OFFSET_FILL);
        st.material = PIX_INVALID;       // the lit pass rebinds everything
        st.program = 0;
    }
    pix__set_clip(r, 0);
    return any;
}

static void pix__draw_sorted(pix_renderer& r, const pix__draw_list& list, pix__pass_state& st, bool blended,
                             idx override_program = 0, bool prepassed = false) {
    if (!list.count) return;
    glBindVertexArray(r.pool.vao);
    bool writes = true, querying = false;
    unsigned qslot = r.mirror_frame % 4;
    size_t i = 0;
    while (i < list.count) {
        const pix_render_instance& a = r.instances[list.items[i]];
        size_t j = i + 1;
        // a blended run only merges consecutive instances, which keeps its order
        while (j < list.count) {
            const pix_render_instance& b = r.instances[list.items[j]];
            if (b.mesh != a.mesh || b.material != a.material) break;
            j++;
        }
        const material& m = r.materials[a.material];
        // a cut-out surface whose depth is already down only colours it in
        bool write = !(prepassed && m.cutout && m.texture);
        if (write != writes) { glDepthMask(write ? GL_TRUE : GL_FALSE); writes = write; }
        if (r.mirror_tracking) {
            // each run of mirror surfaces counted on its own (up to four runs;
            // past that the last one stays open: never hides a mirror that shows)
            bool mirror = m.planar_reflection > 0.0f;
            if (querying && !mirror && r.mirror_issued[qslot] < 4) { glEndQuery(GL_SAMPLES_PASSED); querying = false; }
            if (!querying && mirror && r.mirror_issued[qslot] < 4) {
                glBeginQuery(GL_SAMPLES_PASSED, r.mirror_query[qslot][r.mirror_issued[qslot]++]);
                querying = true;
            }
        }
        pix__use_program(r, st, override_program ? override_program : (m.shader ? m.shader : r.shader));
        pix__bind_material(r, st, a.material, blended);
        pix__draw_run(r, r.meshes[a.mesh], list.base + (GLuint)i, j - i);
        i = j;
    }
    if (!writes) glDepthMask(GL_TRUE);
    if (querying) glEndQuery(GL_SAMPLES_PASSED);
    pix__set_clip(r, 0);
}

// Skinned instances for one view or cascade: culled (`fr`, null = none),
// filtered by `layers`, sorted by (material, mesh), their record indices
// streamed into the list buffer, then one instanced draw per run.
static void pix__draw_skinned(pix_renderer& r, pix__pass_state& st, const mat4* light_vp,
                              const frustum* fr, uint32_t layers) {
    if (!r.animated_count) return;
    size_t count = 0;
    for (size_t k = 0; k < r.animated_count; k++) {
        const pix_render_instance& inst = r.animated_instances[k];
        if (!(inst.layers & layers)) continue;
        if (fr) {
            const mesh& me = r.skinned_meshes[inst.mesh];
            vec3 c = mat4_mul_point(inst.transform, me.center);
            float rad = me.radius * mat4_max_scale(inst.transform) * 1.5f;   // poses reach past the bind pose
            if (!frustum_test_sphere(*fr, c, rad)) { r.frame_culled++; continue; }
        }
        r.sort_keys[count] = pix__batch_key(inst);
        r.sort_vals[count++] = (uint32_t)k;
    }
    if (!count) return;
    pix__radix_sort(r, count);
    if (r.skin_list_cursor + count > r.skin_list_capacity) {
        // more views than budgeted: a fresh buffer; draws already issued keep theirs
        r.skin_list_capacity = (r.skin_list_capacity + count) * 2;
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, r.skin_list_ssbo);
        glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)(r.skin_list_capacity * sizeof(uint32_t)), 0, GL_STREAM_DRAW);
        r.skin_list_cursor = 0;
    }
    size_t base = r.skin_list_cursor;
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, r.skin_list_ssbo);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, (GLintptr)(base * sizeof(uint32_t)),
                    (GLsizeiptr)(count * sizeof(uint32_t)), r.sort_vals);
    r.skin_list_cursor += count;
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 8, r.skin_inst_ssbo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 9, r.skin_bones_ssbo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 10, r.skin_list_ssbo);

    bool depth_only = light_vp != 0;
    glBindVertexArray(r.skin_pool.vao);
    idx bound = PIX_INVALID;
    size_t i = 0;
    while (i < count) {
        const pix_render_instance& a = r.animated_instances[r.sort_vals[i]];
        size_t j = i + 1;
        while (j < count) {
            const pix_render_instance& b = r.animated_instances[r.sort_vals[j]];
            if (b.mesh != a.mesh || b.material != a.material) break;
            j++;
        }
        const material& m = r.materials[a.material];
        if (depth_only) {
            pix__use_program(r, st, r.depth_skinned_shader);
            if (bound == PIX_INVALID) glUniformMatrix4fv(st.u->light_view_proj, 1, GL_FALSE, light_vp->data);
            if (a.material != bound) {
                glUniform1f(st.u->cutoff, m.texture ? m.alpha_cutoff : 0.0f);
                glBindTexture(GL_TEXTURE_2D, m.texture ? m.texture : r.white_texture);
                bound = a.material;
            }
        } else {
            pix__use_program(r, st, m.skinned_shader ? m.skinned_shader : r.skinned_shader);
            pix__bind_material(r, st, a.material, false);
        }
        glUniform1i(st.u->skin_base, (GLint)(base + i));
        const mesh& me = r.skinned_meshes[a.mesh];
        r.frame_draws++;
        r.frame_instances += (int)(j - i);
        glDrawElementsInstancedBaseVertex(GL_TRIANGLES, (GLsizei)me.index_count, GL_UNSIGNED_INT,
                                          (void*)(size_t)(me.index_offset * sizeof(uint32_t)),
                                          (GLsizei)(j - i), (GLint)me.vertex_offset);
        i = j;
    }
}

// Calls every hook whose layers overlap `layers`, then forgets the GL state
// they may have changed.
static void pix__run_hooks(pix_renderer& r, pix__pass_state& st, int pass, int kind, int cascade,
                           const mat4& view_proj, uint32_t layers, int target_w, int target_h) {
    pix__set_clip(r, 0);
    if (!r.hook_count) return;
    pix_hook_context ctx;
    ctx.pass = pass;
    ctx.view_kind = kind;
    ctx.cascade = cascade;
    ctx.view_proj = view_proj;
    ctx.eye = cascade >= 0 ? r.frame_camera.position : r.camera_position;
    ctx.frustum = frustum_from_viewproj(view_proj);
    ctx.target_w = target_w;
    ctx.target_h = target_h;
    for (int i = 0; i < r.hook_count; i++) {
        if (!r.hooks[i].fn || !(r.hooks[i].layers & layers)) continue;
        r.hooks[i].fn(r, ctx, r.hooks[i].user);
        st.program = 0; st.material = PIX_INVALID; st.cull = -1; st.blend = -1;
        glBindVertexArray(0);
    }
}
// Drawn after the opaque geometry, on the far plane with the depth test on:
// only the pixels nothing else covered pay for the sky.
static void pix__sky_pass(pix_renderer& r, const mat4& view_proj) {
    mat4 inv = mat4_inverse(view_proj);
    glDepthMask(GL_FALSE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glUseProgram(r.sky_shader);
    glUniformMatrix4fv(r.u_sky, 1, GL_FALSE, inv.data);
    pix_draw_fullscreen();
    glDepthMask(GL_TRUE);
}

static void pix__particle_pass(pix_renderer& r) {
    if (!r.particle_count) return;
    // billboard basis straight from the view matrix: square to the screen even
    // looking straight down, where a cross product with world up degenerates
    vec3 right = v3(r.view_matrix.data[0], r.view_matrix.data[4], r.view_matrix.data[8]);
    vec3 up    = v3(r.view_matrix.data[1], r.view_matrix.data[5], r.view_matrix.data[9]);
    glUseProgram(r.particle_shader);
    glUniform3f(r.u_particle[0], right.x, right.y, right.z);
    glUniform3f(r.u_particle[1], up.x, up.y, up.z);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE);
    glBindVertexArray(r.particle_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.particle_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(PIX_MAX_PARTICLES * sizeof(pix_particle_vertex)), 0, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(r.particle_count * sizeof(pix_particle_vertex)), r.particles);
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, (GLsizei)r.particle_count);
    r.frame_draws++;
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}

static void pix__line_pass(pix_renderer& r) {
    if (!r.line_count) return;
    glUseProgram(r.line_shader);
    glBindVertexArray(r.line_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.line_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(PIX_MAX_LINES * 2 * sizeof(pix__line_vertex)), 0, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(r.line_count * sizeof(pix__line_vertex)), r.lines);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDrawArrays(GL_LINES, 0, (GLsizei)r.line_count);
    r.frame_draws++;
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}

// ---------------- stage: a view ----------------

pix_view pix_view_default(pix_renderer& r, const camera& cam) {
    pix_view v;
    memset(&v, 0, sizeof(v));
    v.cam = cam;
    v.layers = 0xFFFFFFFFu;
    v.clear = true;
    v.clear_color = r.clear_color;
    v.sky = r.sky_enabled;
    v.opaque = v.skinned = v.transparent = v.particles = v.lines = true;
    v.shadows = true;
    v.reflections = true;
    return v;
}

enum { PIX__KIND_USER, PIX__KIND_PROBE, PIX__KIND_MIRROR };

// Renders `v` into whatever framebuffer is bound by the caller (`fbo`, sized
// fb_w x fb_h). `color_tex`/`depth_tex` are that framebuffer's attachments,
// copied from for refraction; 0 when it has none.
static void pix__render_view_into(pix_renderer& r, const pix_view& v, GLuint fbo, int fb_w, int fb_h,
                                  GLuint color_tex, GLuint depth_tex, int kind) {
    pix__prepare_frame(r);

    int vx = 0, vy = 0, vw = fb_w, vh = fb_h;
    if (v.viewport.w > 0.0f && v.viewport.h > 0.0f) {
        vx = (int)v.viewport.x;
        vw = (int)v.viewport.w;
        vh = (int)v.viewport.h;
        vy = fb_h - (int)v.viewport.y - vh;       // GL counts rows from the bottom
    }
    if (vw < 1 || vh < 1) return;

    camera cam = v.cam;
    if (kind == PIX__KIND_PROBE) { cam.fovy = PIX_PI * 0.5f; cam.ortho_height = 0.0f; }
    pix__set_current_view(r, cam, kind == PIX__KIND_PROBE ? 1.0f : (float)vw / (float)vh);
    float max_dist = v.max_distance > 0.0f ? v.max_distance : r.camera_far;
    pix__build_view_lists(r, r.view_frustum, r.camera_position, max_dist, r.camera_far, v.layers,
                          v.opaque, v.transparent);
    pix__stream_list(r, r.list_opaque);
    pix__stream_list(r, r.list_transparent);

    bool shadows = v.shadows && r.shadows.enabled && r.shadow_map.fbo && r.shadows.strength > 0.0f;
    bool planar = v.reflections && kind == PIX__KIND_USER;
    bool probe  = v.reflections && kind != PIX__KIND_PROBE;
    pix__upload_scene(r, planar, probe, v.clip_plane, fb_w, fb_h);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(vx, vy, vw, vh);
    bool partial = vx != 0 || vy != 0 || vw != fb_w || vh != fb_h;
    int sx = vx, sy = vy, sw = vw, sh = vh;
    if (v.scissor.w > 0.0f && v.scissor.h > 0.0f) {
        // intersect with the scissor, flipped to GL's bottom-left rows
        int x0 = (int)v.scissor.x, x1 = (int)(v.scissor.x + v.scissor.w + 1.0f);
        int y1 = fb_h - (int)v.scissor.y, y0 = fb_h - (int)(v.scissor.y + v.scissor.h + 1.0f);
        int nx0 = x0 > sx ? x0 : sx, ny0 = y0 > sy ? y0 : sy;
        int nx1 = x1 < sx + sw ? x1 : sx + sw, ny1 = y1 < sy + sh ? y1 : sy + sh;
        sx = nx0; sy = ny0; sw = nx1 - nx0; sh = ny1 - ny0;
        if (sw < 1 || sh < 1) return;
        partial = true;
    }
    if (partial) { glEnable(GL_SCISSOR_TEST); glScissor(sx, sy, sw, sh); }
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    if (v.clear) {
        vec3 cc = pix__lin(v.clear_color);
        glClearColor(cc.x, cc.y, cc.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    pix__bind_frame_textures(r, shadows, probe, planar, false);
    pix__pass_state st = { 0, 0, PIX_INVALID, -1, -1 };
    bool prepassed = v.opaque && !v.override_shader && kind == PIX__KIND_USER && pix__cutout_prepass(r, r.list_opaque, st);
    if (v.opaque) pix__draw_sorted(r, r.list_opaque, st, false, v.override_shader, prepassed);
    if (v.skinned && r.animated_count)
        pix__draw_skinned(r, st, 0, kind == PIX__KIND_MIRROR ? 0 : &r.view_frustum, v.layers);
    pix__run_hooks(r, st, PIX_HOOK_OPAQUE, kind, -1, r.view_proj, v.layers, fb_w, fb_h);
    if (v.sky) { pix__sky_pass(r, r.view_proj); st.program = 0; st.cull = -1; }
    if (v.lines) pix__line_pass(r);

    if (r.list_transparent.count) {
        if (r.refract_needed && color_tex && depth_tex) {
            // what is behind the glass: one GPU-side copy of this view's opaque result
            pix__ensure_refract_tex(r, fb_w, fb_h);
            glCopyImageSubData(color_tex, GL_TEXTURE_2D, 0, 0, 0, 0, r.refract_tex, GL_TEXTURE_2D, 0, 0, 0, 0, fb_w, fb_h, 1);
            glCopyImageSubData(depth_tex, GL_TEXTURE_2D, 0, 0, 0, 0, r.refract_depth, GL_TEXTURE_2D, 0, 0, 0, 0, fb_w, fb_h, 1);
            pix__bind_frame_textures(r, shadows, probe, planar, true);
        }
        glEnable(GL_BLEND);
        glDepthMask(GL_FALSE);
        st.blend = -1;
        st.material = PIX_INVALID;
        r.mirror_tracking = kind == PIX__KIND_USER && r.mirror_query[0][0] != 0;
        pix__draw_sorted(r, r.list_transparent, st, true, v.override_shader);
        r.mirror_tracking = false;
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
    if (r.hook_count) {
        glDepthMask(GL_FALSE);
        pix__run_hooks(r, st, PIX_HOOK_TRANSPARENT, kind, -1, r.view_proj, v.layers, fb_w, fb_h);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
    if (v.particles) pix__particle_pass(r);

    glDisable(GL_CULL_FACE);
    glDisable(GL_SCISSOR_TEST);
    glBindVertexArray(0);
}

void pix_render_view(pix_renderer& r, const pix_view& v) {
    pix__prepare_frame(r);
    const framebuffer& t = v.target ? *v.target : r.scene_target;
    pix__gpu_begin(r, PIX_GPU_VIEW);
    pix__render_view_into(r, v, t.fbo, t.width, t.height, t.color, t.depth, PIX__KIND_USER);
    pix__gpu_end(r, PIX_GPU_VIEW);
}

// ---------------- stage: shadows ----------------

// The sun's view of the world, one cascade per atlas tile, each with its own
// culled caster list - the near cascade is a handful of draws, not the world.
// Faces are not culled so single-sided geometry still casts; a slope-scaled
// polygon offset plus the shader's normal offset keep surfaces from shadowing
// themselves.
void pix_render_shadows(pix_renderer& r) {
    pix__prepare_frame(r);
    if (!r.shadows.enabled || !r.shadow_map.fbo || r.shadows.strength <= 0.0f) return;
    pix__gpu_begin(r, PIX_GPU_SHADOW);

    size_t n = r.renderable_count;
    for (size_t i = 0; i < n; i++) {
        r.flags[i] &= (uint8_t)~0xF0;
        if (!(r.flags[i] & PIX__CASTER)) continue;
        vec3 c = v3(r.bounds[i].x, r.bounds[i].y, r.bounds[i].z);
        float rad = r.bounds[i].w;
        for (int k = 0; k < SHADOW_CASCADES; k++) {
            if (!(r.cascade_mask & (1 << k))) continue;
            vec3 lp = mat4_mul_point(r.cascade_view[k], c);
            float e = r.cascade_radius[k] + rad;
            if (fabsf(lp.x) <= e && fabsf(lp.y) <= e && -lp.z >= -rad && -lp.z <= r.cascade_depth + rad)
                r.flags[i] |= (uint8_t)(PIX__CASCADE0 << k);
        }
    }
    for (int k = 0; k < SHADOW_CASCADES; k++) {
        size_t count = 0;
        r.list_cascade[k].count = 0;
        if (!(r.cascade_mask & (1 << k))) continue;
        uint8_t bit = (uint8_t)(PIX__CASCADE0 << k);
        for (size_t i = 0; i < n; i++) {
            if (!(r.flags[i] & bit)) continue;
            r.sort_keys[count] = pix__batch_key(r.instances[i]);
            r.sort_vals[count++] = (uint32_t)i;
        }
        pix__radix_sort(r, count);
        memcpy(r.list_cascade[k].items, r.sort_vals, count * sizeof(uint32_t));
        r.list_cascade[k].count = count;
        pix__stream_list(r, r.list_cascade[k]);
    }

    pix_bind_framebuffer(r.shadow_map);
    glDepthMask(GL_TRUE);
    int tile = r.shadow_map.height;
    if (r.cascade_mask == (1 << SHADOW_CASCADES) - 1) glClear(GL_DEPTH_BUFFER_BIT);
    else {
        // only the tiles redrawn: the others keep last frame's depth
        glEnable(GL_SCISSOR_TEST);
        for (int c = 0; c < SHADOW_CASCADES; c++)
            if (r.cascade_mask & (1 << c)) { glScissor(c * tile, 0, tile, tile); glClear(GL_DEPTH_BUFFER_BIT); }
        glDisable(GL_SCISSOR_TEST);
    }
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.5f, 2.0f);

    pix__pass_state st = { 0, 0, PIX_INVALID, 0, -1 };
    for (int c = 0; c < SHADOW_CASCADES; c++) {
        if (!(r.cascade_mask & (1 << c))) continue;
        int size = (int)((float)tile * (r.cascade_scale[c] > 0.0f ? r.cascade_scale[c] : 1.0f));
        glViewport(c * tile, 0, size, size);
        st.program = 0;                  // each cascade re-sends its light matrix to whichever program runs first
        const pix__draw_list& list = r.list_cascade[c];
        if (list.count) {
            glBindVertexArray(r.pool.vao);
            idx bound = PIX_INVALID;
            size_t i = 0;
            while (i < list.count) {
                const pix_render_instance& a = r.instances[list.items[i]];
                size_t j = i + 1;
                while (j < list.count && r.instances[list.items[j]].mesh == a.mesh
                       && r.instances[list.items[j]].material == a.material) j++;
                if (a.material != bound) {
                    const material& m = r.materials[a.material];
                    GLuint prog = (GLuint)pix__depth_program(r, m);
                    if (st.program != prog) {
                        pix__use_program(r, st, prog);
                        glUniformMatrix4fv(st.u->light_view_proj, 1, GL_FALSE, r.light_space[c].data);
                    }
                    glUniform1f(st.u->cutoff, m.texture ? m.alpha_cutoff : 0.0f);
                    if (st.u->params0 >= 0) glUniform4f(st.u->params0, m.params0.x, m.params0.y, m.params0.z, m.params0.w);
                    if (st.u->params1 >= 0) glUniform4f(st.u->params1, m.params1.x, m.params1.y, m.params1.z, m.params1.w);
                    glBindTexture(GL_TEXTURE_2D, m.texture ? m.texture : r.white_texture);
                    bound = a.material;
                }
                pix__draw_run(r, r.meshes[a.mesh], list.base + (GLuint)i, j - i);
                i = j;
            }
            pix__set_clip(r, 0);
        }
        // skinned casters: instanced and culled per cascade, so every cascade can afford them
        if (r.animated_count && r.shadows.skinned && c < r.shadows.skinned_cascades) {
            frustum cf = frustum_from_viewproj(r.light_space[c]);
            pix__draw_skinned(r, st, &r.light_space[c], &cf, 0xFFFFFFFFu);
        }
        pix__run_hooks(r, st, PIX_HOOK_SHADOW, PIX_VIEW_USER, c, r.light_space[c], 0xFFFFFFFFu, tile, tile);
    }
    glDisable(GL_POLYGON_OFFSET_FILL);
    glBindVertexArray(0);
    pix__gpu_end(r, PIX_GPU_SHADOW);
}

// ---------------- stage: reflections ----------------

// The probe refreshes a few of its faces around the frame camera; the planar
// mirror renders the frame camera reflected across r.reflections.plane. Both
// are ordinary views underneath, just into their own targets.
void pix_render_reflections(pix_renderer& r) {
    pix__prepare_frame(r);
    pix_reflection_settings& rs = r.reflections;
    pix__gpu_begin(r, PIX_GPU_REFLECT);

    if (rs.probe && r.env_cube) {
        vec3 pos = rs.probe_follow_camera ? r.frame_camera.position : rs.probe_position;
        r.env_position = pos;
        int n = rs.probe_faces_per_frame < 1 ? 1 : (rs.probe_faces_per_frame > 6 ? 6 : rs.probe_faces_per_frame);
        pix_view pv = pix_view_default(r, r.frame_camera);
        pv.transparent = pv.skinned = pv.particles = pv.lines = false;
        pv.reflections = false;
        pv.max_distance = rs.probe_distance > 1.0f ? rs.probe_distance : 1.0f;
        pv.cam.position = pos;
        pv.cam.near_plane = 0.1f;
        pv.cam.far_plane = pv.max_distance;
        glBindFramebuffer(GL_FRAMEBUFFER, r.env_fbo);
        for (int k = 0; k < n; k++) {
            int f = r.env_next_face;
            r.env_next_face = (f + 1) % 6;
            pix__probe_face(f, &pv.cam.direction, &pv.cam.up);
            glBindFramebuffer(GL_FRAMEBUFFER, r.env_fbo);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + f, r.env_cube, 0);
            pix__render_view_into(r, pv, r.env_fbo, r.env_resolution, r.env_resolution, 0, 0, PIX__KIND_PROBE);
        }
        // box-filtered mips stand in for roughness: rough surfaces read blurrier levels
        glBindTexture(GL_TEXTURE_CUBE_MAP, r.env_cube);
        glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
        r.env_faces_done = r.env_faces_done + n > 6 ? 6 : r.env_faces_done + n;
    }

    r.planar_active = false;
    // a mirror is a whole second view of the scene: skipped unless a surface
    // that shows it is in sight this frame
    bool mirror_seen = false;
    if (rs.planar && r.planar_target.fbo) {
        if (!r.mirror_query[0][0]) { glGenQueries(16, &r.mirror_query[0][0]); r.mirror_seen_last = true; }
        // the query of three frames ago: too few samples passed means nothing much showed the mirror
        r.mirror_frame++;
        unsigned oldest = (r.mirror_frame + 1) % 4;
        // not drawn then: the frustum decides; else worth a mirror past 1/2000 of the screen
        bool seen_then = r.mirror_issued[oldest] == 0;
        long long pixels = 0, enough = (long long)r.width * r.height / 2000 + 1;
        for (int q = 0; q < r.mirror_issued[oldest] && !seen_then; q++) {
            GLint ready = 0, count = 0;
            glGetQueryObjectiv(r.mirror_query[oldest][q], GL_QUERY_RESULT_AVAILABLE, &ready);
            if (!ready) { seen_then = r.mirror_seen_last; break; }   // not known yet: as before
            glGetQueryObjectiv(r.mirror_query[oldest][q], GL_QUERY_RESULT, &count);
            pixels += count;
            seen_then = pixels >= enough;
        }
        r.mirror_pixels = r.mirror_issued[oldest] ? pixels : -1;
        r.mirror_seen_last = seen_then;
        r.mirror_issued[r.mirror_frame % 4] = 0;
        frustum ff = frustum_from_viewproj(r.frame_view_proj);
        for (size_t i = 0; i < r.renderable_count && !mirror_seen; i++) {
            if (r.materials[r.instances[i].material].planar_reflection <= 0.0f) continue;
            mirror_seen = frustum_test_sphere(ff, v3(r.bounds[i].x, r.bounds[i].y, r.bounds[i].z), r.bounds[i].w);
        }
        mirror_seen = mirror_seen && seen_then;
    }
    if (mirror_seen) {
        vec3 n = v3(rs.plane.x, rs.plane.y, rs.plane.z);
        float len = v3len(n);
        const camera& fc = r.frame_camera;
        if (len > 1e-6f) {
            n = v3scale(n, 1.0f / len);
            float w = rs.plane.w / len;
            float side = v3dot(n, fc.position) + w;
            // from behind the mirror (under the water) there is nothing to reflect
            if (side > 0.01f) {
                pix_view mv = pix_view_default(r, fc);
                vec3 dir = v3norm(fc.direction);
                mv.cam.position  = v3sub(fc.position, v3scale(n, 2.0f * side));
                mv.cam.direction = v3sub(dir, v3scale(n, 2.0f * v3dot(n, dir)));
                mv.cam.up        = v3sub(fc.up, v3scale(n, 2.0f * v3dot(n, fc.up)));
                mv.transparent = mv.particles = mv.lines = false;
                mv.max_distance = rs.planar_distance > 1.0f ? rs.planar_distance : 1.0f;
                // a hair below the surface, so the waterline itself is not clipped
                mv.clip_plane = v4(n.x, n.y, n.z, w + 0.02f);
                framebuffer& t = r.planar_target;
                // only the part of the mirror image the surfaces showing it cover
                {
                    float far_d;
                    vec3 md = v3norm(mv.cam.direction);
                    vec3 mup = v3len(mv.cam.up) > 1e-6f ? mv.cam.up : v3(0.0f, 1.0f, 0.0f);
                    mat4 mvp = mat4_mul(pix__projection(r, mv.cam, (float)t.width / (float)t.height, &far_d),
                                        mat4_lookat(mv.cam.position, v3add(mv.cam.position, md), mup));
                    float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
                    bool whole = false;
                    for (size_t i = 0; i < r.renderable_count && !whole; i++) {
                        if (r.materials[r.instances[i].material].planar_reflection <= 0.0f) continue;
                        vec4 b = r.bounds[i];
                        for (int k = 0; k < 8 && !whole; k++) {
                            vec4 c = mat4_mul_vec4(mvp, v4(b.x + (k & 1 ? b.w : -b.w), b.y + (k & 2 ? b.w : -b.w),
                                                           b.z + (k & 4 ? b.w : -b.w), 1.0f));
                            if (c.w <= 0.01f) { whole = true; break; }
                            float px = c.x / c.w, py = c.y / c.w;
                            x0 = minf(x0, px); x1 = maxf(x1, px); y0 = minf(y0, py); y1 = maxf(y1, py);
                        }
                    }
                    if (!whole) {
                        x0 = clampf(x0, -1.0f, 1.0f); x1 = clampf(x1, -1.0f, 1.0f);
                        y0 = clampf(y0, -1.0f, 1.0f); y1 = clampf(y1, -1.0f, 1.0f);
                        mv.scissor.x = (x0 * 0.5f + 0.5f) * t.width;
                        mv.scissor.w = (x1 - x0) * 0.5f * t.width;
                        mv.scissor.y = (0.5f - y1 * 0.5f) * t.height;
                        mv.scissor.h = (y1 - y0) * 0.5f * t.height;
                        if (mv.scissor.w < 1.0f || mv.scissor.h < 1.0f) mv.scissor.w = mv.scissor.h = 1.0f;
                    }
                }
                pix__render_view_into(r, mv, t.fbo, t.width, t.height, 0, 0, PIX__KIND_MIRROR);
                r.planar_view_proj = r.view_proj;   // what the surfaces project into to sample it
                glBindTexture(GL_TEXTURE_2D, t.color);
                glGenerateMipmap(GL_TEXTURE_2D);
                r.planar_active = true;
            }
        }
    }
    pix__gpu_end(r, PIX_GPU_REFLECT);
}

// ---------------- stage: post processing ----------------

// Glare: soft-knee threshold into quarter resolution, then two separable blurs.
static void pix__bloom_pass(pix_renderer& r, GLuint source_color) {
    glDisable(GL_DEPTH_TEST);
    pix_bind_framebuffer(r.bloom_a);
    glUseProgram(r.bloom_prefilter_shader);
    glUniform1i(r.u_bloom[0], 0);
    glUniform2f(r.u_bloom[1], 1.0f / (float)r.width, 1.0f / (float)r.height);
    glUniform1f(r.u_bloom[2], r.post.bloom_threshold);
    glUniform1f(r.u_bloom[3], r.post.bloom_knee > 1e-3f ? r.post.bloom_knee : 1e-3f);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, source_color);
    pix_draw_fullscreen();

    glUseProgram(r.bloom_blur_shader);
    glUniform1i(r.u_bloom[4], 0);
    for (int pass = 0; pass < 2; pass++) {        // two rounds widen the glow cheaply
        pix_bind_framebuffer(r.bloom_b);
        glUniform2f(r.u_bloom[5], (1.0f + pass) / (float)r.bloom_a.width, 0.0f);
        glBindTexture(GL_TEXTURE_2D, r.bloom_a.color);
        pix_draw_fullscreen();
        pix_bind_framebuffer(r.bloom_a);
        glUniform2f(r.u_bloom[5], 0.0f, (1.0f + pass) / (float)r.bloom_a.height);
        glBindTexture(GL_TEXTURE_2D, r.bloom_b.color);
        pix_draw_fullscreen();
    }
}

static void pix__bind_dest(pix_renderer& r, const framebuffer* dest) {
    if (dest) pix_bind_framebuffer(*dest);
    else      pix_bind_backbuffer(r.width, r.height);
}

// HDR -> display, then whatever runs after the tonemap: user effects in order,
// then FXAA last so it smooths their edges too. Stages ping-pong between two
// LDR targets and the final stage writes the destination directly, so with
// nothing enabled after the tonemap the whole chain is one fullscreen pass.
void pix_post_process(pix_renderer& r, const framebuffer& src, const framebuffer* dest) {
    pix__prepare_frame(r);
    pix__gpu_begin(r, PIX_GPU_POST);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glDisable(GL_SCISSOR_TEST);
    const pix_post_settings& p = r.post;
    bool full = src.width == r.width && src.height == r.height;
    bool bloom = full && p.bloom && p.bloom_strength > 0.0f && r.bloom_a.fbo;
    if (bloom) pix__bloom_pass(r, src.color);

    int effects = full ? pix__enabled_effects(r) : 0;
    bool fxaa = full && p.fxaa;
    int stages = effects + (fxaa ? 1 : 0);   // passes after the tonemap
    framebuffer* ping[2] = { &r.ldr_a, &r.ldr_b };
    int at = 0;
    if (stages) pix_bind_framebuffer(r.ldr_a);
    else        pix__bind_dest(r, dest);

    glUseProgram(r.post_shader);
    glUniform1i(r.u_post[0], 0);
    glUniform1i(r.u_post[1], 1);
    glUniform2f(r.u_post[2], 1.0f / (float)src.width, 1.0f / (float)src.height);
    glUniform1f(r.u_post[3], r.exposure * p.exposure);
    glUniform1f(r.u_post[4], p.saturation);
    glUniform1f(r.u_post[5], p.contrast);
    glUniform1f(r.u_post[6], full ? p.vignette : 0.0f);
    glUniform1f(r.u_post[7], p.sharpen);
    glUniform1f(r.u_post[8], bloom ? p.bloom_strength : 0.0f);
    glUniform1i(r.u_post[9], p.tonemap);
    glUniform3f(r.u_post[10], p.color_filter.x, p.color_filter.y, p.color_filter.z);
    glUniform1f(r.u_post[11], p.dither ? 1.0f : 0.0f);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, bloom ? r.bloom_a.color : r.white_texture);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, src.color);
    pix_draw_fullscreen();

    int stage = 0;
    for (int i = 0; i < r.post_effect_count && effects; i++) {
        const pix_post_effect& e = r.post_effects[i];
        if (!e.enabled || !e.program) continue;
        bool last = ++stage == stages;
        if (last) pix__bind_dest(r, dest);
        else      pix_bind_framebuffer(*ping[at ^ 1]);
        glUseProgram(e.program);
        glUniform1i(e.u_scene, 0);
        glUniform1i(e.u_depth, 1);
        glUniform2f(e.u_texel, 1.0f / (float)r.width, 1.0f / (float)r.height);
        glUniform4f(e.u_params, e.params.x, e.params.y, e.params.z, e.params.w);
        glUniform2f(e.u_near_far, r.camera_near, r.camera_far);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, src.depth ? src.depth : r.white_texture);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, ping[at]->color);
        pix_draw_fullscreen();
        at ^= 1;
    }

    if (fxaa) {
        pix__bind_dest(r, dest);
        glUseProgram(r.fxaa_shader);
        glUniform1i(r.u_fxaa[0], 0);
        glUniform2f(r.u_fxaa[1], 1.0f / (float)r.width, 1.0f / (float)r.height);
        glBindTexture(GL_TEXTURE_2D, ping[at]->color);
        pix_draw_fullscreen();
    }
    pix__gpu_end(r, PIX_GPU_POST);
}

void pix_apply_effects(pix_renderer& r, const framebuffer* target) {
    pix__sync_settings(r);
    int effects = pix__enabled_effects(r);
    if (!effects || !r.ldr_a.fbo) return;
    // copy the frame so far into the first LDR target, then ping-pong the
    // chain through the pair, the last effect writing back into `target`
    glDisable(GL_SCISSOR_TEST);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, target ? target->fbo : 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, r.ldr_a.fbo);
    glBlitFramebuffer(0, 0, r.width, r.height, 0, 0, r.width, r.height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    framebuffer* ping[2] = { &r.ldr_a, &r.ldr_b };
    int at = 0, stage = 0;
    for (int i = 0; i < r.post_effect_count; i++) {
        const pix_post_effect& e = r.post_effects[i];
        if (!e.enabled || !e.program) continue;
        if (++stage == effects) pix__bind_dest(r, target);
        else                    pix_bind_framebuffer(*ping[at ^ 1]);
        glUseProgram(e.program);
        glUniform1i(e.u_scene, 0);
        glUniform1i(e.u_depth, 1);
        glUniform2f(e.u_texel, 1.0f / (float)r.width, 1.0f / (float)r.height);
        glUniform4f(e.u_params, e.params.x, e.params.y, e.params.z, e.params.w);
        glUniform2f(e.u_near_far, 0.1f, 1000.0f);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, r.white_texture);      // no depth: everything at the far plane
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, ping[at]->color);
        pix_draw_fullscreen();
        at ^= 1;
    }
}

// ---------------- stage: your own fullscreen pass ----------------

static const char* const PIX__FULLSCREEN_PRELUDE = R"(#version 440 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uTex0;
uniform sampler2D uTex1;
uniform sampler2D uTex2;
uniform sampler2D uTex3;
uniform vec2 uTexel;
uniform vec4 uParams;
)";

idx pix_load_fullscreen_shader(pix_renderer& r, const char* body) {
    size_t n = strlen(PIX__FULLSCREEN_PRELUDE) + strlen(body) + 1;
    char* src = pix_new<char>(n);
    if (!src) return 0;
    strcpy(src, PIX__FULLSCREEN_PRELUDE);
    strcat(src, body);
    char* full = shader_with_common(src);
    idx prog = full ? opengl_create_shader(VSHDER_POST, full) : 0;
    pix_free(full);
    pix_free(src);
    if (prog && r.shader_count < MAX_SHADERS) r.shaders[r.shader_count++] = prog;
    return prog;
}

void pix_fullscreen_pass(pix_renderer& r, idx program, const idx* textures, int count,
                         const framebuffer* dest, vec4 params) {
    if (!program) return;
    pix__prepare_frame(r);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    pix__bind_dest(r, dest);
    glUseProgram(program);
    static const char* names[4] = { "uTex0", "uTex1", "uTex2", "uTex3" };
    for (int i = 0; i < count && i < 4; i++) {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_2D, textures[i]);
        glUniform1i(glGetUniformLocation(program, names[i]), i);
    }
    glActiveTexture(GL_TEXTURE0);
    int w = dest ? dest->width : r.width, h = dest ? dest->height : r.height;
    glUniform2f(glGetUniformLocation(program, "uTexel"), 1.0f / (float)w, 1.0f / (float)h);
    glUniform4f(glGetUniformLocation(program, "uParams"), params.x, params.y, params.z, params.w);
    pix_draw_fullscreen();
    r.frame_draws++;
}

// ---------------- the default frame ----------------

void pix_finish_frame(pix_renderer& r, bool clear_instances) {
    if (r.gpu_timing) r.gpu_slot ^= 1;
    // back to the frame camera, so picking and world_to_screen answer for it
    pix__set_current_view(r, r.frame_camera, (float)r.width / (float)r.height);
    glUseProgram(0);
    glBindVertexArray(0);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_SCISSOR_TEST);
    glDepthMask(GL_TRUE);
    pix_bind_backbuffer(r.width, r.height);
    if (clear_instances) {
        r.renderable_count = 0;
        r.animated_count = 0;
        r.particle_count = 0;
        r.line_count = 0;
    }
    r.frame_prepared = false;
}

void end_frame(pix_renderer& r, bool clear_instances) {
    pix__prepare_frame(r);
    pix_render_shadows(r);
    pix_render_reflections(r);
    pix_render_view(r, pix_view_default(r, r.frame_camera));
    pix_post_process(r, r.scene_target, 0);
    pix_finish_frame(r, clear_instances);
}

#endif // PIX_RENDERER_IMPL
