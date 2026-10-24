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
#include "meshops.hpp"
#include "../loader/hdr_loader.hpp"
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
#ifndef PIX_MAX_QUADS
#define PIX_MAX_QUADS      16384   // billboards + 3D text glyphs per frame
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

    // ---- PBR maps (GL textures, 0 = none) - pix_load_model fills them from glTF ----
    idx   normal_map;    // tangent-space normals, green = up (OpenGL / glTF convention)
    float normal_scale;  // bump strength, 1 = as authored
    idx   orm_map;       // r occlusion, g roughness, b metallic: multiplies the fields above
    float occlusion;     // how much of the map's occlusion applies, 0..1
    idx   emissive_map;  // sRGB, multiplies emissive

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
    // (0.02 - 0.1 is glass to thick water). alpha then means opacity of the
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

    // ---- outlines ----
    // > 0: an ink line this many world units wide around the surface (the mesh
    // drawn again, pushed out along its normals, back faces only)
    float outline_width;
    vec3  outline_color;    // sRGB
    // 0..255: which group the screen outline (r.outline) separates this
    // surface into; group edges get inked, and with r.outline.groups_only only
    // groups other than 0 are - a selection highlight
    uint8_t outline_group;

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
    // Set by the renderer while a mesh with LODs cross-fades between two
    // levels (see pix_set_mesh_lods); leave it 0.
    float lod_fade = 0.0f;
    // For temporal AA: a number of your own, the same every frame, for an
    // object that moves (0 = still, or not worth it). With it the renderer
    // knows where the object was last frame and moving edges stay sharp
    // instead of trailing. The camera's own motion needs no ids.
    uint32_t motion_id = 0;
};

// ---- anti-aliasing (r.aa) ----
// TAA jitters the camera by a fraction of a pixel each frame and blends the
// frames, following motion: smooth edges, thin wires and specular highlights
// for the price of one pass. MSAA renders the main view with several samples
// per pixel: sharp, no blending over time, costs fill rate and memory.
// Both can be on; FXAA (r.post.fxaa) is the cheap fallback.
struct pix_aa_settings {
    bool  taa;
    float taa_feedback;   // 0..0.98: how much of the past each frame keeps (0.9; higher = smoother, more trailing)
    int   msaa;           // samples on the main view: 1 (off), 2, 4 or 8
};

// What one particle is once it has stopped being simulation: a point, a size
// and a colour, laid out to be uploaded as-is.
struct pix_particle_vertex {
    float centre[3];
    float size;        // half-width in world units
    float color[4];    // rgb linear radiance, a coverage
};

// A static mesh's simpler stand-ins; see pix_set_mesh_lods.
#define PIX_MAX_LODS 4
struct pix_mesh_lods {
    idx   meshes[PIX_MAX_LODS];   // simpler and simpler
    float screen[PIX_MAX_LODS];   // meshes[i] is used below this fraction of the screen's height
    int   count;
};

// How a world quad (billboard, 3D text) turns.
#define PIX_FACE_CAMERA  0   // square to every view: labels, impostors, effects
#define PIX_FACE_UPRIGHT 1   // turns about world y only: trees, characters as sprites
#define PIX_FACE_FIXED   2   // stays on the given right/up axes: signs, decals

// One textured rectangle in the world. Drawn unlit (fog still applies), after
// the opaque scene; `cutout` ones alpha-test and write depth instead of blending.
struct pix_quad3d {
    vec3  anchor;        // the world point the rectangle hangs off
    int   facing;        // PIX_FACE_*
    vec3  right, up;     // PIX_FACE_FIXED only: world axes (their lengths scale the rect)
    vec4  rect;          // x0, y0, x1, y1 along right/up from the anchor, world units, y up
    vec4  uv;            // u0, v0 (top-left), u1, v1 (bottom-right)
    vec4  color;         // linear rgb (can exceed 1 to glow), a = opacity
    idx   texture;       // GL texture, 0 = plain white
    bool  sdf;           // the texture's red channel is a signed distance field (text)
    bool  cutout;
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
    float inv_view_proj[16];
    float prev_view_proj[16];
    float proj[16];
    float fog_height[4];
    float screen_fx[4];
    float shadow_soft[4];
    float ibl[4];
    float sh[9 * 4];
};

#define PIX_MAX_SHADOW_TILES 16   // local light shadow atlas tiles (a spot uses 1, a point 6)
struct pix_light_block {
    float pos_radius[MAX_SHADER_LIGHTS * 4];
    float color_inner[MAX_SHADER_LIGHTS * 4];
    float dir_outer[MAX_SHADER_LIGHTS * 4];
    float meta[4];
    float shadow[MAX_SHADER_LIGHTS * 4];
    float tiles[PIX_MAX_SHADOW_TILES * 16];
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
    // > 0: soft shadows that widen with distance from the caster (PCSS) - the
    // sun's angular size in radians; the real sun is ~0.01, 0.02..0.05 reads soft
    float penumbra;
    // > 0: screen-space contact shadows this long (metres) - the small gaps
    // under feet and objects that the shadow map is too coarse to see
    float contact;
    // shadows from spot and point lights with light.cast_shadows set
    bool  local;
    int   local_resolution;   // the atlas side, texels (tiles are a quarter of it)
};

// What the main view keeps beside its colour: depth plus view-space normals.
// AUTO runs the prepass whenever something needs it (screen AO, contact
// shadows, normal or group outlines, a post effect that reads uNormal); with
// it on, the lit pass shades each visible pixel once.
#define PIX_PREPASS_AUTO 0
#define PIX_PREPASS_ON   1
#define PIX_PREPASS_OFF  2

// Screen-space ambient occlusion: corners, creases and contact darken.
struct pix_ao_settings {
    bool  enabled;
    float radius;            // metres the occlusion reaches, 0.6
    float intensity;         // 1.5: higher is darker
    int   samples;           // 12; 8 is cheaper, 16 smoother
    bool  half_resolution;   // true: a quarter of the cost, a little softer
};

// Ink lines from the screen: where depth jumps, surfaces fold, or one
// material group (material.outline_group) meets another.
struct pix_outline_settings {
    bool  enabled;
    vec3  color;             // sRGB, black by default
    float thickness;         // pixels, 1
    float depth_threshold;   // relative depth jump that counts as an edge, 0.1
    float normal_threshold;  // 0..2 bend that counts as an edge, 0.4; 0 = none
    bool  object_edges;      // ink where outline groups meet (true)
    bool  groups_only;       // only around groups other than 0: selection highlights
    float fade_distance;     // fade lines out by this distance, 0 = never
};

// Height and colour of the fog; its colour and density come from the time of
// day or pix_set_fog.
struct pix_fog_settings {
    float height;            // where height fog is at full density, world y
    float height_falloff;    // per metre above it; 0 = even fog everywhere (and density means the old exp2 fog)
    float sun_scatter;       // 0..: fog glows toward the sun
    float max_opacity;       // 1: the fog never hides more than this much
};

// Light you can see in the air: sun shafts through gaps (from the shadow
// map), cones under spot lights, halos round lamps.
struct pix_volumetric_settings {
    bool  enabled;
    float density;           // 0.02: how much the air scatters, per metre
    float anisotropy;        // -1..1: 0.6 glows toward the light
    float intensity;         // sun shafts, 1
    float light_intensity;   // spot and point light volumes, 1
    float max_distance;      // metres marched, 80
    int   steps;             // 24
    bool  lights;            // spot and point lights scatter too
};

struct pix_dof_settings {
    bool  enabled;
    float focus_distance;    // metres
    float focus_range;       // metres either side that stay sharp-ish
    float max_blur;          // pixels, 8
    bool  auto_focus;        // focus on whatever is in the middle of the screen
};

struct pix_motion_blur_settings {
    bool  enabled;
    float strength;          // 1 = a full frame of motion
    int   samples;           // 8
    float max_length;        // fraction of the screen, 0.05
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
    // eye adaptation: exposure follows the scene's average brightness
    bool  auto_exposure;
    float exposure_key;    // the average it aims for, 0.18
    float exposure_speed;  // how quickly it adapts, 1.5
    float exposure_min, exposure_max;   // limits on the adapted multiplier, 0.25..4
    idx   lut;             // a colour-grading cube from pix_load_color_lut, 0 = none
    float lut_strength;    // 1
    float grain;           // film grain, 0 = none, ~0.04 visible
};

// A user fullscreen pass run after tonemapping, in the order added.
#define PIX_MAX_POST_EFFECTS 8
struct pix_post_effect {
    idx  program;
    bool enabled;
    vec4 params;           // uParams in the shader, free for the effect to use
    GLint u_scene, u_depth, u_texel, u_params, u_near_far, u_normal;
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

// Holes left in a pool by unloaded meshes, sorted by start and never touching
// each other or the cursor (those merge on free).
struct pix__range { size_t start, count; };
struct pix__free_list { pix__range* ranges; size_t count, capacity; };

// A growable pair of vertex + index buffers with one VAO over them.
struct pix__geometry_pool {
    GLuint vao, vbo, ebo;
    size_t vertex_size;
    size_t vertex_capacity, index_capacity;   // elements
    size_t vertex_cursor, index_cursor;       // high-water marks
    pix__free_list free_vertices, free_indices;
    bool   skinned;
};

struct pix__program_uniforms {
    GLuint program;
    GLint screen_fx, local_shadow, shadow_raw, outline_group, outline_width, outline_color;
    bool  patches;      // has tessellation stages: draws GL_PATCHES
    GLint tex, color, metallic, roughness, emissive, cutoff, unlit, shadow_map;
    GLint model, tint, bones, light_view_proj;
    GLint env_map, planar_map, refract_map, planar, refraction;
    GLint scene_depth, params0, params1;
    GLint toon, extra_tex[4];
    GLint normal_map, orm_map, emissive_map, maps, map_scale;
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
struct pix__gpu_instance { mat4 model; vec4 tint; float lod; };

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
    // slots given back by pix_unload_*, reused before the tables grow
    idx* free_mesh_ids;     size_t free_mesh_id_count;
    idx* free_skinned_ids;  size_t free_skinned_id_count;
    idx* free_material_ids; size_t free_material_id_count;
    // ---- anti-aliasing ----
    pix_aa_settings aa;
    framebuffer taa_history[2];       // the resolved image, ping-ponged
    int    taa_index;                 // which history holds last frame
    bool   taa_valid;                 // false: no usable past (first frame, resize, pix_taa_reset)
    vec2   taa_jitter;                // this frame's sub-pixel offset, pixels
    bool   taa_jittered;              // the main view drew jittered this frame
    mat4   taa_view_proj;             // this frame's main camera, unjittered
    mat4   taa_jittered_view_proj;
    mat4   taa_prev_view_proj;        // last frame's, unjittered
    GLuint velocity_fbo, velocity_tex, velocity_vao, velocity_vbo;
    size_t velocity_capacity;         // instances velocity_vbo holds
    float* velocity_staging;
    // motion_id -> last frame's transform: open addressing, entries stamped by frame
    struct pix__motion_entry { uint32_t id, frame; mat4 cur, prev; }* motion_table;
    idx    taa_shader, velocity_shader;
    GLuint msaa_fbo, msaa_color, msaa_depth;
    int    msaa_samples;              // what msaa_fbo was made with
    // the HDR environment (pix_set_environment): sky, reflections, diffuse light
    struct {
        idx   cube;          // GGX-prefiltered: mip m is roughness m / max_mip
        int   size, max_mip;
        float intensity;
        vec3  sh[9];         // diffuse, as sh_irradiance reads it
        idx   convert_shader, prefilter_shader;   // built on first use
    } environment;
    // LOD chains: lod_of[mesh] is 1 + its chain's index, 0 for none
    uint16_t*      lod_of;
    pix_mesh_lods* lod_chains;  size_t lod_chain_count;
    float lod_band;   // how far past a switch point (fraction of it) the two levels cross-fade; 0 = hard switch
    float lod_bias;   // multiplies screen sizes: > 1 keeps detail longer, < 1 drops it sooner
    // Instances smaller on screen than this many pixels across (their bounding
    // sphere) are not drawn, and casters smaller than this many shadow texels
    // are left out of a cascade: specks the rasteriser would mostly miss
    // anyway. 0.5 by default, which changes nothing you can see; 1 to 2 drops
    // far more of a big scene's distant clutter, but thousands of specks
    // together do tint a far field, which then thins out. 0 draws everything.
    float cull_pixels;
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
    int                 prepass;          // PIX_PREPASS_*
    pix_ao_settings     ao;
    pix_outline_settings outline;
    pix_fog_settings    fog;
    pix_volumetric_settings volumetrics;
    pix_dof_settings    dof;
    pix_motion_blur_settings motion_blur;
    bool      wireframe;                  // draw every opaque mesh's edges over it
    pix_color wireframe_color;
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
    uint32_t* skin_list_staging;      // PIX_MAX_ANIMATED: the last culled, sorted skinned list
    // That list's view: a view's prepass, colour and outline passes all ask
    // for the same one, and reuse it rather than cull, sort and upload again.
    struct { bool valid, culled; frustum fr; uint32_t layers; size_t base, count; int dropped; } skin_reuse;
    pix__program_uniforms uniform_cache[MAX_SHADERS * 2];
    int uniform_cache_count;

    GLuint scene_ubo, lights_ubo;
    idx white_texture;
    idx dummy_shadow;                 // 1x1 depth texture bound when shadows are off

    framebuffer scene_target;         // HDR
    framebuffer ldr_a, ldr_b;         // tonemapped ping-pong for FXAA and post effects
    framebuffer shadow_map;           // depth atlas, cascades side by side
    framebuffer bloom_a, bloom_b;
    GLint u_post[32];                 // cached uniform locations, see pix__cache_uniforms
    GLint u_bloom[6];
    GLint u_fxaa[2];
    GLint u_sky, u_particle[2], u_quad[3];
    pix_quad3d* quads;  size_t quad_count;
    float*      quad_staging;   // 24 floats per quad, in draw order
    uint32_t*   quad_order;
    float*      quad_depth;     // per quad, view z: the blended ones' sort key
    idx    quad_shader;
    GLuint quad_vao, quad_vbo;

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

    // ---- scene buffers and the passes that read them ----
    GLuint normal_tex, prepass_fbo;      // prepass normals; depth is scene_target's
    bool   prepass_done;                 // this frame's main view ran the prepass
    framebuffer fx_a, fx_b;              // screen AO / contact shadows and its blur
    bool   fx_valid;
    framebuffer vol_a, vol_b;            // volumetric light, half resolution
    framebuffer hdr_a, hdr_b;            // full-size HDR ping-pong: motion blur, DoF
    framebuffer dof_half;
    GLuint lum_tex, lum_fbo;             // log luminance, mipmapped to its average
    GLuint adapt_tex[2], adapt_fbo[2];   // adapted luminance, ping-pong 1x1
    int    adapt_index;
    bool   adapt_fresh;
    float  last_time;
    framebuffer local_shadow;            // spot / point shadow atlas
    int    local_tiles;                  // tiles used this frame
    mat4   local_tile_vp[PIX_MAX_SHADOW_TILES];
    vec3   local_tile_eye[PIX_MAX_SHADOW_TILES];
    float  local_tile_far[PIX_MAX_SHADOW_TILES];
    float  local_info[MAX_SHADER_LIGHTS * 4];
    GLuint shadow_raw_sampler;           // the sun atlas read without comparison (PCSS)
    mat4   prev_view_proj;
    unsigned frame_number;
    GLenum draw_mode;                    // GL_TRIANGLES, or GL_PATCHES while a tessellated program draws
    idx prepass_shader, prepass_skinned_shader, fx_shader, bilateral_shader;
    idx outline_shader, outline_skinned_shader, volumetric_shader, motion_blur_shader;
    idx dof_blur_shader, dof_composite_shader, luminance_shader, adapt_shader, wire_shader;
    struct { idx surface, prepass; } prepass_twins[MAX_SHADERS]; size_t prepass_twin_count;
    struct { idx program; } patch_programs[MAX_SHADERS]; size_t patch_program_count;
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
// Gives the mesh's space in the shared buffers back; the next load reuses it
// and may reuse the id. Instances already pushed this frame draw nothing.
PIXDEF void pix_unload_mesh(pix_renderer& renderer, idx mesh);
PIXDEF void pix_unload_skinned_mesh(pix_renderer& renderer, idx mesh);

// ---- levels of detail (static meshes) ----
//
// Give a mesh simpler stand-ins and the renderer swaps them in by itself as the
// mesh gets smaller on screen: push the base mesh as usual. The switch is a
// short dithered cross-fade (r.lod_band), so nothing pops. Chosen once per frame
// from the frame's camera, so shadows and reflections agree with what you see.
// (pix_mesh_lods is declared with the other public structs, above.)
// `lods` and `screen` are `count` long, `screen` decreasing (e.g. 0.3, 0.15, 0.07).
// count 0 removes them. Unloading `base` unloads its LODs too.
PIXDEF bool pix_set_mesh_lods(pix_renderer& renderer, idx base, const idx* lods, const float* screen, int count);
// Builds `levels` LODs by simplification (pix_mesh_simplify, each `ratio` of the
// one before) and sets them with screen sizes 0.3, 0.15, ... Returns how many
// it made: it stops early when the mesh will not simplify further.
PIXDEF int pix_generate_mesh_lods(pix_renderer& renderer, idx base, int levels = 3, float ratio = 0.5f);

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
// Frees the slot for the next pix_add_material. Textures are not deleted:
// materials can share them (clone_material), so they belong to the caller.
PIXDEF void pix_unload_material(pix_renderer& renderer, idx material_id);

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
// The same with every programmable stage: a geometry shader (`gsrc`) and/or
// tessellation (`tcsrc` control + `tesrc` evaluation, drawn as triangle
// patches). Each stage receives what the one before it outputs and must pass
// on vNormal, vUV, vWorld and vTint to the fragment stage (the tessellation
// evaluation or geometry stage sets gl_Position). Null stages are skipped.
// Shadows and the prepass run the vertex stage only.
PIXDEF idx load_surface_shader_stages(pix_renderer& renderer, const char* fsrc, const char* vsrc,
                                      const char* gsrc, const char* tcsrc = nullptr, const char* tesrc = nullptr);
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
// a billboard, a glyph of 3D text, a sign: see pix_quad3d
PIXDEF void push_quad3d(pix_renderer& renderer, const pix_quad3d& quad);
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
    bool quads;                 // billboards and 3D text
    bool shadows;             // receive the sun's shadows (needs pix_render_shadows first)
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

// A colour-grading cube from a PNG laid out as N slices of N x N side by side
// (the usual 256 x 16 or 1024 x 32 strip): red across each slice, green down,
// blue from slice to slice. Assign the result to r.post.lut.
PIXDEF idx pix_load_color_lut(const char* path);

// ---- HDR environment (image-based lighting) ----
//
// An equirectangular HDR image (linear RGB floats, rows top to bottom) becomes
// the sky, the light every surface reflects (prefiltered by roughness) and the
// diffuse light from all around (spherical harmonics). Replaces the analytic
// sky and hemisphere ambient; the sun and other lights still add on top.
// With the reflection probe on, the probe captures this sky and takes over
// reflections near the camera.
PIXDEF bool pix_set_environment(pix_renderer& renderer, const float* rgb, int width, int height,
                                float intensity = 1.0f);
// the same from a Radiance .hdr file
PIXDEF bool pix_load_environment(pix_renderer& renderer, const char* hdr_path, float intensity = 1.0f);
// back to the analytic sky
PIXDEF void pix_clear_environment(pix_renderer& renderer);

// Temporal AA forgets the past: call on a camera cut or teleport, so the new
// view does not start blended with the old one.
PIXDEF void pix_taa_reset(pix_renderer& renderer);
// The diffuse part on its own: nine coefficients from an equirect image, as
// sh_irradiance reads them (exposed for tests and for baking your own).
PIXDEF void pix_environment_sh(const float* rgb, int width, int height, vec3 out_sh[9]);

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
// a skinned vertex stage with an unlit fragment stage that still wants the scene block
static idx pix__compile_skinned_common(const char* vs, const char* fs) {
    const char* vchunks[] = { GLSL_SCENE, GLSL_COMMON, GLSL_SKINNING };
    char* v = pix_shader_splice(vs, vchunks, 3);
    char* fr = shader_with_common(fs);
    idx p = (v && fr) ? opengl_create_shader(v, fr) : 0;
    pix_free(v);
    pix_free(fr);
    return p;
}

// Every pix sampler on its own unit. A program whose samplers of different
// types share a unit (the default, 0) fails to draw at all, so any program
// built with the lighting library gets this once.
static void pix__scene_sampler_units(GLuint p) {
    static const char* names[] = { "uShadowMap", "uEnvMap", "uPlanarMap", "uRefractMap", "uSceneDepth",
                                   "uScreenFx", "uLocalShadow", "uShadowRaw" };
    static const int units[] = { 1, 2, 3, 4, 5, 10, 11, 12 };
    glUseProgram(p);
    for (int i = 0; i < 8; i++) {
        GLint loc = glGetUniformLocation(p, names[i]);
        if (loc >= 0) glUniform1i(loc, units[i]);
    }
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
    u.screen_fx   = glGetUniformLocation(program, "uScreenFx");
    u.local_shadow = glGetUniformLocation(program, "uLocalShadow");
    u.shadow_raw  = glGetUniformLocation(program, "uShadowRaw");
    u.outline_group = glGetUniformLocation(program, "uOutlineGroup");
    u.outline_width = glGetUniformLocation(program, "uOutlineWidth");
    u.outline_color = glGetUniformLocation(program, "uOutlineColor");
    u.normal_map  = glGetUniformLocation(program, "uNormalMap");
    u.orm_map     = glGetUniformLocation(program, "uOrmMap");
    u.emissive_map = glGetUniformLocation(program, "uEmissiveMap");
    u.maps        = glGetUniformLocation(program, "uMaps");
    u.map_scale   = glGetUniformLocation(program, "uMapScale");
    u.patches = false;
    for (size_t i = 0; i < r.patch_program_count; i++)
        if (r.patch_programs[i].program == program) u.patches = true;
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
        glVertexAttribPointer(8, 4, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex, tangent));
        for (int i = 0; i < 3; i++) glEnableVertexAttribArray(i);
        glEnableVertexAttribArray(8);
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
        glVertexAttribPointer(9, 1, GL_FLOAT, GL_FALSE, sizeof(pix__gpu_instance),
                              (void*)offsetof(pix__gpu_instance, lod));
        glEnableVertexAttribArray(9);
        glVertexAttribDivisor(9, 1);
    } else {
        GLsizei s = sizeof(vertex_rigged);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex_rigged, position));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex_rigged, normal));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex_rigged, uv));
        glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex_rigged, bone_ids));
        glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex_rigged, bone_weights));
        glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, s, (void*)offsetof(vertex_rigged, tangent));
        for (int i = 0; i < 6; i++) glEnableVertexAttribArray(i);
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

// First fit: the first hole big enough, trimmed from its front.
static bool pix__free_take(pix__free_list& f, size_t count, size_t* out) {
    for (size_t i = 0; i < f.count; i++) {
        pix__range& h = f.ranges[i];
        if (h.count < count) continue;
        *out = h.start;
        h.start += count;
        h.count -= count;
        if (!h.count) { memmove(&f.ranges[i], &f.ranges[i + 1], (f.count - i - 1) * sizeof(pix__range)); f.count--; }
        return true;
    }
    return false;
}

// Returns [start, start + count) to the list, merging with the holes either
// side; a hole that reaches the cursor lowers the cursor instead.
static void pix__free_give(pix__free_list& f, size_t start, size_t count, size_t& cursor) {
    size_t i = 0;
    while (i < f.count && f.ranges[i].start < start) i++;
    bool join_prev = i > 0 && f.ranges[i - 1].start + f.ranges[i - 1].count == start;
    bool join_next = i < f.count && start + count == f.ranges[i].start;
    if (join_prev && join_next) {
        f.ranges[i - 1].count += count + f.ranges[i].count;
        memmove(&f.ranges[i], &f.ranges[i + 1], (f.count - i - 1) * sizeof(pix__range));
        f.count--;
    } else if (join_prev) {
        f.ranges[i - 1].count += count;
    } else if (join_next) {
        f.ranges[i].start = start;
        f.ranges[i].count += count;
    } else {
        if (f.count == f.capacity) {
            size_t cap = f.capacity ? f.capacity * 2 : 32;
            pix__range* grown = pix_alloc<pix__range>(cap);
            if (!grown) return;   // the space is lost, nothing else breaks
            if (f.count) memcpy(grown, f.ranges, f.count * sizeof(pix__range));
            pix_free(f.ranges);
            f.ranges = grown;
            f.capacity = cap;
        }
        memmove(&f.ranges[i + 1], &f.ranges[i], (f.count - i) * sizeof(pix__range));
        f.ranges[i].start = start;
        f.ranges[i].count = count;
        f.count++;
    }
    pix__range& last = f.ranges[f.count - 1];
    if (last.start + last.count == cursor) { cursor = last.start; f.count--; }
}

// Space for one mesh: holes first, else the end of the pool (growing it).
static void pix__pool_alloc(pix_renderer& r, pix__geometry_pool& p, size_t vertices, size_t indices,
                            size_t* vertex_at, size_t* index_at) {
    bool hole_v = pix__free_take(p.free_vertices, vertices, vertex_at);
    bool hole_i = pix__free_take(p.free_indices, indices, index_at);
    pix__pool_reserve(r, p, hole_v ? 0 : vertices, hole_i ? 0 : indices);
    if (!hole_v) { *vertex_at = p.vertex_cursor; p.vertex_cursor += vertices; }
    if (!hole_i) { *index_at = p.index_cursor; p.index_cursor += indices; }
}

static void pix__pool_upload(pix__geometry_pool& p, size_t vertex_at, const void* vertices, size_t vertex_count,
                             size_t index_at, const uint32_t* indices, size_t index_count) {
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, p.vbo);
    glBufferSubData(GL_ARRAY_BUFFER, (GLintptr)(vertex_at * p.vertex_size),
                    (GLsizeiptr)(vertex_count * p.vertex_size), vertices);
    glBindBuffer(GL_COPY_WRITE_BUFFER, p.ebo);
    glBufferSubData(GL_COPY_WRITE_BUFFER, (GLintptr)(index_at * sizeof(uint32_t)),
                    (GLsizeiptr)(index_count * sizeof(uint32_t)), indices);
}

static void pix__pool_free(pix__geometry_pool& p, mesh& m) {
    pix__free_give(p.free_vertices, m.vertex_offset, m.vertex_count, p.vertex_cursor);
    pix__free_give(p.free_indices, m.index_offset, m.index_count, p.index_cursor);
    m = mesh();
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
static void pix__destroy_prepass(pix_renderer& r) {
    if (r.prepass_fbo) glDeleteFramebuffers(1, &r.prepass_fbo);
    if (r.normal_tex) glDeleteTextures(1, &r.normal_tex);
    r.prepass_fbo = r.normal_tex = 0;
}

static void pix__destroy_aa(pix_renderer& r) {
    for (int i = 0; i < 2; i++) if (r.taa_history[i].fbo) pix_destroy_framebuffer(r.taa_history[i]);
    if (r.velocity_fbo) glDeleteFramebuffers(1, &r.velocity_fbo);
    if (r.velocity_tex) glDeleteTextures(1, &r.velocity_tex);
    if (r.msaa_fbo) glDeleteFramebuffers(1, &r.msaa_fbo);
    GLuint rbs[2] = { r.msaa_color, r.msaa_depth };
    glDeleteRenderbuffers(2, rbs);
    r.velocity_fbo = r.velocity_tex = r.msaa_fbo = r.msaa_color = r.msaa_depth = 0;
    r.msaa_samples = 0;
    r.taa_valid = false;
}

static void pix__destroy_screen_targets(pix_renderer& r) {
    framebuffer* all[] = { &r.scene_target, &r.ldr_a, &r.ldr_b, &r.bloom_a, &r.bloom_b,
                           &r.fx_a, &r.fx_b, &r.vol_a, &r.vol_b, &r.hdr_a, &r.hdr_b, &r.dof_half };
    for (size_t i = 0; i < PIX_ARRAY_COUNT(all); i++)
        if (all[i]->fbo) pix_destroy_framebuffer(*all[i]);
    pix__destroy_prepass(r);
    pix__destroy_aa(r);
}

static framebuffer pix__color_target(int w, int h, GLint internal, GLenum format, GLenum type, bool linear);

// TAA's history pair and velocity buffer (which tests against the scene's
// depth), and MSAA's multisampled colour + depth: made when asked for,
// dropped when not.
static void pix__sync_aa(pix_renderer& r) {
    pix_aa_settings& a = r.aa;
    if (a.taa && !r.taa_history[0].fbo) {
        for (int i = 0; i < 2; i++)
            r.taa_history[i] = pix__color_target(r.width, r.height, GL_RGBA16F, GL_RGBA, GL_HALF_FLOAT, true);
        glGenTextures(1, &r.velocity_tex);
        glBindTexture(GL_TEXTURE_2D, r.velocity_tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, r.width, r.height, 0, GL_RG, GL_FLOAT, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glGenFramebuffers(1, &r.velocity_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, r.velocity_fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, r.velocity_tex, 0);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, r.scene_target.depth, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        r.taa_valid = false;
    } else if (!a.taa && r.taa_history[0].fbo) {
        for (int i = 0; i < 2; i++) pix_destroy_framebuffer(r.taa_history[i]);
        glDeleteFramebuffers(1, &r.velocity_fbo);
        glDeleteTextures(1, &r.velocity_tex);
        r.velocity_fbo = r.velocity_tex = 0;
    }

    int samples = a.msaa >= 8 ? 8 : (a.msaa >= 4 ? 4 : (a.msaa >= 2 ? 2 : 1));
    if (r.msaa_fbo && samples != r.msaa_samples) {
        glDeleteFramebuffers(1, &r.msaa_fbo);
        GLuint rbs[2] = { r.msaa_color, r.msaa_depth };
        glDeleteRenderbuffers(2, rbs);
        r.msaa_fbo = r.msaa_color = r.msaa_depth = 0;
        r.msaa_samples = 0;
    }
    if (samples > 1 && !r.msaa_fbo) {
        // the scene target's own formats, so both resolve with a blit
        GLuint rbs[2];
        glGenRenderbuffers(2, rbs);
        glBindRenderbuffer(GL_RENDERBUFFER, rbs[0]);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_R11F_G11F_B10F, r.width, r.height);
        glBindRenderbuffer(GL_RENDERBUFFER, rbs[1]);
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH_COMPONENT24, r.width, r.height);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
        glGenFramebuffers(1, &r.msaa_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, r.msaa_fbo);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rbs[0]);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbs[1]);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            pix_log("pix: %dx MSAA target incomplete", samples);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        r.msaa_color = rbs[0];
        r.msaa_depth = rbs[1];
        r.msaa_samples = samples;
    }
}

// A colour-only target of any format, for the passes whose format the
// render-target helper does not offer (RGBA16F with alpha, mipmapped R16F).
static framebuffer pix__color_target(int w, int h, GLint internal, GLenum format, GLenum type, bool linear) {
    framebuffer t = {};
    t.width = w; t.height = h; t.hdr = true;
    GLuint tex = 0, fbo = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, internal, w, h, 0, format, type, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, linear ? GL_LINEAR : GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, linear ? GL_LINEAR : GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    t.fbo = fbo; t.color = tex;
    return t;
}

// Does anything this frame read the prepass buffers?
static bool pix__wants_prepass(const pix_renderer& r) {
    if (r.prepass == PIX_PREPASS_OFF) return false;
    if (r.prepass == PIX_PREPASS_ON) return true;
    if (r.ao.enabled || r.shadows.contact > 0.0f) return true;
    const pix_outline_settings& o = r.outline;
    if (o.enabled && (o.normal_threshold > 0.0f || o.object_edges || o.groups_only)) return true;
    for (int i = 0; i < r.post_effect_count; i++)
        if (r.post_effects[i].enabled && r.post_effects[i].program && r.post_effects[i].u_normal >= 0) return true;
    return false;
}

static void pix__sync_extras(pix_renderer& r) {
    // ---- prepass: normals beside the scene depth ----
    bool pre = pix__wants_prepass(r);
    if (pre && !r.prepass_fbo) {
        glGenTextures(1, &r.normal_tex);
        glBindTexture(GL_TEXTURE_2D, r.normal_tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, r.width, r.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glGenFramebuffers(1, &r.prepass_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, r.prepass_fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, r.normal_tex, 0);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, r.scene_target.depth, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    } else if (!pre && r.prepass_fbo) {
        pix__destroy_prepass(r);
    }

    // ---- screen AO / contact shadows ----
    bool fx = pre && (r.ao.enabled || r.shadows.contact > 0.0f);
    int div = r.ao.half_resolution ? 2 : 1;
    int fw = r.width / div > 1 ? r.width / div : 1, fh = r.height / div > 1 ? r.height / div : 1;
    if (r.fx_a.fbo && (!fx || r.fx_a.width != fw)) { pix_destroy_framebuffer(r.fx_a); pix_destroy_framebuffer(r.fx_b); }
    if (fx && !r.fx_a.fbo) {
        r.fx_a = pix_create_render_target(fw, fh, false, false, false);
        r.fx_b = pix_create_render_target(fw, fh, false, false, false);
    }

    // ---- volumetric light ----
    bool vol = r.volumetrics.enabled;
    if (vol && !r.vol_a.fbo) {
        int vw = r.width / 2 > 1 ? r.width / 2 : 1, vh = r.height / 2 > 1 ? r.height / 2 : 1;
        r.vol_a = pix_create_render_target(vw, vh, true, false, false);
        r.vol_b = pix_create_render_target(vw, vh, true, false, false);
    } else if (!vol && r.vol_a.fbo) {
        pix_destroy_framebuffer(r.vol_a);
        pix_destroy_framebuffer(r.vol_b);
    }

    // ---- HDR ping-pong for motion blur and depth of field ----
    int hdr_need = (r.motion_blur.enabled ? 1 : 0) + (r.dof.enabled ? 1 : 0);
    if (hdr_need >= 1 && !r.hdr_a.fbo) r.hdr_a = pix_create_render_target(r.width, r.height, true, false, false);
    if (hdr_need >= 2 && !r.hdr_b.fbo) r.hdr_b = pix_create_render_target(r.width, r.height, true, false, false);
    if (hdr_need < 2 && r.hdr_b.fbo) pix_destroy_framebuffer(r.hdr_b);
    if (hdr_need < 1 && r.hdr_a.fbo) pix_destroy_framebuffer(r.hdr_a);
    if (r.dof.enabled && !r.dof_half.fbo) {
        int dw = r.width / 2 > 1 ? r.width / 2 : 1, dh = r.height / 2 > 1 ? r.height / 2 : 1;
        r.dof_half = pix__color_target(dw, dh, GL_RGBA16F, GL_RGBA, GL_HALF_FLOAT, true);
    } else if (!r.dof.enabled && r.dof_half.fbo) {
        pix_destroy_framebuffer(r.dof_half);
    }

    // ---- auto exposure ----
    if (r.post.auto_exposure && !r.lum_tex) {
        glGenTextures(1, &r.lum_tex);
        glBindTexture(GL_TEXTURE_2D, r.lum_tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, 256, 256, 0, GL_RED, GL_FLOAT, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glGenerateMipmap(GL_TEXTURE_2D);
        glGenFramebuffers(1, &r.lum_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, r.lum_fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, r.lum_tex, 0);
        for (int i = 0; i < 2; i++) {
            glGenTextures(1, &r.adapt_tex[i]);
            glBindTexture(GL_TEXTURE_2D, r.adapt_tex[i]);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, 1, 1, 0, GL_RED, GL_FLOAT, 0);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glGenFramebuffers(1, &r.adapt_fbo[i]);
            glBindFramebuffer(GL_FRAMEBUFFER, r.adapt_fbo[i]);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, r.adapt_tex[i], 0);
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        r.adapt_fresh = true;
    } else if (!r.post.auto_exposure && r.lum_tex) {
        glDeleteTextures(1, &r.lum_tex); glDeleteFramebuffers(1, &r.lum_fbo);
        glDeleteTextures(2, r.adapt_tex); glDeleteFramebuffers(2, r.adapt_fbo);
        r.lum_tex = r.lum_fbo = 0;
        r.adapt_tex[0] = r.adapt_tex[1] = r.adapt_fbo[0] = r.adapt_fbo[1] = 0;
    }

    // ---- local light shadow atlas ----
    pix_shadow_settings& s = r.shadows;
    int lres = s.local_resolution < 512 ? 512 : (s.local_resolution > 8192 ? 8192 : s.local_resolution);
    if (r.local_shadow.fbo && (!s.local || r.local_shadow.width != lres)) pix_destroy_framebuffer(r.local_shadow);
    if (s.local && !r.local_shadow.fbo) r.local_shadow = pix_create_shadow_map(lres, lres);
}

static void pix__cache_uniforms(pix_renderer& r) {
    const char* post[] = { "uScene", "uBloom", "uTexel", "uExposure", "uSaturation", "uContrast",
                           "uVignette", "uSharpen", "uBloomStrength", "uTonemap", "uColorFilter", "uDither",
                           "uOutlineColor", "uOutlineParams", "uOutlineMode", "uAutoExposure", "uGrade" };
    for (int i = 0; i < 17; i++) r.u_post[i] = glGetUniformLocation(r.post_shader, post[i]);
    glUseProgram(r.post_shader);
    const char* units[] = { "uDepthTex", "uNormalTex", "uExposureTex", "uLut", "uVolumetric" };
    for (int i = 0; i < 5; i++) glUniform1i(glGetUniformLocation(r.post_shader, units[i]), 2 + i);
    glUseProgram(0);
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

    pix__sync_extras(r);
    pix__sync_aa(r);
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
    r.free_mesh_ids      = pix_alloc<idx>(PIX_MAX_MESHES);
    r.free_skinned_ids   = pix_alloc<idx>(PIX_MAX_MESHES);
    r.free_material_ids  = pix_alloc<idx>(PIX_MAX_MATERIALS);
    r.lod_of             = pix_alloc<uint16_t>(PIX_MAX_MESHES);
    r.lod_chains         = pix_alloc<pix_mesh_lods>(PIX_MAX_MESHES);
    r.lod_band = 0.2f;
    r.lod_bias = 1.0f;
    r.cull_pixels = 0.5f;
    r.instances         = pix_alloc<pix_render_instance>(PIX_MAX_INSTANCES);
    r.animated_instances = pix_alloc<pix_render_instance>(PIX_MAX_ANIMATED);
    r.animated_bones     = pix_alloc<const mat4*>(PIX_MAX_ANIMATED);
    r.animated_bone_count = pix_alloc<uint32_t>(PIX_MAX_ANIMATED);
    r.skin_list_staging  = pix_alloc<uint32_t>(PIX_MAX_ANIMATED);
    r.particles          = pix_alloc<pix_particle_vertex>(PIX_MAX_PARTICLES);
    r.quads              = pix_alloc<pix_quad3d>(PIX_MAX_QUADS);
    r.quad_staging       = pix_alloc<float>(PIX_MAX_QUADS * 24);
    r.quad_order         = pix_alloc<uint32_t>(PIX_MAX_QUADS);
    r.quad_depth         = pix_alloc<float>(PIX_MAX_QUADS);
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
    r.quad_shader = pix__compile(VSHDER_QUAD3D, FSHDER_QUAD3D, false);
    r.taa_shader = opengl_create_shader(VSHDER_POST, FSHDER_TAA);
    r.velocity_shader = opengl_create_shader(VSHDER_VELOCITY, FSHDER_VELOCITY);
    glGenVertexArrays(1, &r.velocity_vao);
    glGenBuffers(1, &r.velocity_vbo);
    r.u_quad[0] = glGetUniformLocation(r.quad_shader, "uCamRight");
    r.u_quad[1] = glGetUniformLocation(r.quad_shader, "uCamUp");
    r.u_quad[2] = glGetUniformLocation(r.quad_shader, "uCutout");
    {
        // the built-in static programs carry the LOD cross-fade (see FSHDER_DEPTH)
        const char* lod[] = { "#define PIX_LOD_FADE\n" };
        char* df = pix_shader_splice(FSHDER_DEPTH, lod, 1);
        r.depth_shader = df ? opengl_create_shader(VSHDER_DEPTH, df) : 0;
        pix_free(df);
        const char* pre[] = { "#define PIX_LOD_FADE\n", GLSL_SCENE, GLSL_COMMON };
        char* pv = shader_with_common(VSHDER_BASIC);
        char* pf = pix_shader_splice(FSHDER_PREPASS, pre, 3);
        r.prepass_shader = (pv && pf) ? opengl_create_shader(pv, pf) : 0;
        pix_free(pv);
        pix_free(pf);
    }
    r.depth_skinned_shader = pix__compile_skinned(VSHDER_DEPTH_SKINNED, FSHDER_DEPTH, false);
    {
        // the skinning storage buffers; sized on first use and grown as crowds do
        GLuint b[3];
        glGenBuffers(3, b);
        r.skin_bones_ssbo = b[0]; r.skin_inst_ssbo = b[1]; r.skin_list_ssbo = b[2];
    }
    r.bloom_prefilter_shader = opengl_create_shader(VSHDER_POST, FSHDER_BLOOM_PREFILTER);
    r.bloom_blur_shader      = opengl_create_shader(VSHDER_POST, FSHDER_BLOOM_BLUR);
    r.prepass_skinned_shader = pix__compile_skinned_common(VSHDER_SKINNED, FSHDER_PREPASS);
    r.fx_shader              = pix__compile(VSHDER_POST, FSHDER_SCREEN_FX, false);
    r.bilateral_shader       = pix__compile(VSHDER_POST, FSHDER_BILATERAL, false);
    r.outline_shader         = pix__compile(VSHDER_OUTLINE, FSHDER_OUTLINE, false);
    r.outline_skinned_shader = pix__compile_skinned_common(VSHDER_OUTLINE_SKINNED, FSHDER_OUTLINE);
    r.volumetric_shader      = pix__compile(VSHDER_POST, FSHDER_VOLUMETRIC, true);
    r.motion_blur_shader     = pix__compile(VSHDER_POST, FSHDER_MOTION_BLUR, false);
    r.dof_blur_shader        = pix__compile(VSHDER_POST, FSHDER_DOF_BLUR, false);
    r.dof_composite_shader   = opengl_create_shader(VSHDER_POST, FSHDER_DOF_COMPOSITE);
    r.luminance_shader       = opengl_create_shader(VSHDER_POST, FSHDER_LUMINANCE);
    r.adapt_shader           = opengl_create_shader(VSHDER_POST, FSHDER_ADAPT);
    r.wire_shader            = pix__compile(VSHDER_BASIC, FSHDER_OUTLINE, false);
    {
        idx lit[] = { r.shader, r.skinned_shader, r.volumetric_shader };
        for (int i = 0; i < 3; i++) if (lit[i]) pix__scene_sampler_units((GLuint)lit[i]);
        glUseProgram(0);
    }
    glGenSamplers(1, &r.shadow_raw_sampler);
    glSamplerParameteri(r.shadow_raw_sampler, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glSamplerParameteri(r.shadow_raw_sampler, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glSamplerParameteri(r.shadow_raw_sampler, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glSamplerParameteri(r.shadow_raw_sampler, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glSamplerParameteri(r.shadow_raw_sampler, GL_TEXTURE_COMPARE_MODE, GL_NONE);
    glPatchParameteri(GL_PATCH_VERTICES, 3);
    r.draw_mode = GL_TRIANGLES;
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

    // world quads: six vec4s per instance (see pix__quad_pass), the corner from gl_VertexID
    glGenVertexArrays(1, &r.quad_vao);
    glGenBuffers(1, &r.quad_vbo);
    glBindVertexArray(r.quad_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.quad_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(PIX_MAX_QUADS * 24 * sizeof(float)), 0, GL_STREAM_DRAW);
    for (int i = 0; i < 6; i++) {
        glEnableVertexAttribArray(i);
        glVertexAttribPointer(i, 4, GL_FLOAT, GL_FALSE, 24 * sizeof(float), (void*)(i * 4 * sizeof(float)));
        glVertexAttribDivisor(i, 1);
    }

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
    r.aa.taa = true;
    r.aa.taa_feedback = 0.9f;
    r.aa.msaa = 1;
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

    r.shadows.penumbra = 0.0f;
    r.shadows.contact = 0.0f;
    r.shadows.local = false;
    r.shadows.local_resolution = 2048;
    r.prepass = PIX_PREPASS_AUTO;
    r.ao.enabled = false;
    r.ao.radius = 0.6f;
    r.ao.intensity = 1.5f;
    r.ao.samples = 12;
    r.ao.half_resolution = true;
    r.outline.enabled = false;
    r.outline.color = v3(0.0f, 0.0f, 0.0f);
    r.outline.thickness = 1.0f;
    r.outline.depth_threshold = 0.1f;
    r.outline.normal_threshold = 0.4f;
    r.outline.object_edges = true;
    r.outline.groups_only = false;
    r.outline.fade_distance = 0.0f;
    r.fog.height = 0.0f;
    r.fog.height_falloff = 0.0f;
    r.fog.sun_scatter = 0.0f;
    r.fog.max_opacity = 1.0f;
    r.volumetrics.enabled = false;
    r.volumetrics.density = 0.02f;
    r.volumetrics.anisotropy = 0.6f;
    r.volumetrics.intensity = 1.0f;
    r.volumetrics.light_intensity = 1.0f;
    r.volumetrics.max_distance = 80.0f;
    r.volumetrics.steps = 24;
    r.volumetrics.lights = true;
    r.dof.enabled = false;
    r.dof.focus_distance = 10.0f;
    r.dof.focus_range = 8.0f;
    r.dof.max_blur = 8.0f;
    r.dof.auto_focus = false;
    r.motion_blur.enabled = false;
    r.motion_blur.strength = 1.0f;
    r.motion_blur.samples = 8;
    r.motion_blur.max_length = 0.05f;
    r.post.auto_exposure = false;
    r.post.exposure_key = 0.18f;
    r.post.exposure_speed = 1.5f;
    r.post.exposure_min = 0.25f;
    r.post.exposure_max = 4.0f;
    r.post.lut = 0;
    r.post.lut_strength = 1.0f;
    r.post.grain = 0.0f;
    r.wireframe = false;
    r.wireframe_color = pix_rgba(235, 235, 235);
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
    static const bool taa[4]  = { false, true,   true,  true };
    static const bool glow[4] = { false, true,   true,  true };
    static const bool probe[4] = { false, false, true,  true };
    static const int  probe_res[4] = { 64, 64,   128,   256 };
    static const float speck[4] = { 1.5f, 1.0f,  0.5f,  0.25f };   // cull_pixels
    int q = quality < 0 ? 0 : (quality > 3 ? 3 : quality);
    r.shadows.enabled = true;
    r.shadows.resolution = res[q];
    r.shadows.softness = soft[q];
    r.aa.taa = taa[q];
    r.post.fxaa = false;          // TAA covers it; FXAA stays available on its own
    r.post.bloom = glow[q];
    r.reflections.probe = probe[q];
    r.reflections.probe_resolution = probe_res[q];
    r.reflections.probe_faces_per_frame = q == PIX_QUALITY_ULTRA ? 2 : 1;
    r.cull_pixels = speck[q];
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
uniform sampler2D uNormal;  // view-space normal + outline group (reading it turns the prepass on)
float linear_depth(vec2 uv) {
    float z = texture(uDepth, uv).r * 2.0 - 1.0;
    return 2.0 * uNearFar.x * uNearFar.y / (uNearFar.y + uNearFar.x - z * (uNearFar.y - uNearFar.x));
}
vec3 view_normal(vec2 uv) { return normalize(texture(uNormal, uv).xyz * 2.0 - 1.0); }
float outline_group(vec2 uv) { return floor(texture(uNormal, uv).a * 255.0 + 0.5); }
vec3 world_position(vec2 uv) {
    vec4 p = bInvViewProj * vec4(uv * 2.0 - 1.0, texture(uDepth, uv).r * 2.0 - 1.0, 1.0);
    return p.xyz / p.w;
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
    e.u_normal   = glGetUniformLocation(prog, "uNormal");
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
                      r.particle_vbo, r.quad_vbo, r.velocity_vbo, r.line_vbo, r.scene_ubo, r.lights_ubo,
                      r.skin_bones_ssbo, r.skin_inst_ssbo, r.skin_list_ssbo };
    glDeleteBuffers((GLsizei)PIX_ARRAY_COUNT(bufs), bufs);
    GLuint vaos[] = { r.pool.vao, r.skin_pool.vao, r.particle_vao, r.quad_vao, r.velocity_vao, r.line_vao };
    glDeleteVertexArrays((GLsizei)PIX_ARRAY_COUNT(vaos), vaos);
    idx progs[] = { r.shader, r.skinned_shader, r.sky_shader, r.post_shader, r.fxaa_shader, r.line_shader,
                    r.particle_shader, r.quad_shader, r.taa_shader, r.velocity_shader,
                    r.depth_shader, r.depth_skinned_shader,
                    r.bloom_prefilter_shader, r.bloom_blur_shader, r.prepass_shader, r.prepass_skinned_shader,
                    r.fx_shader, r.bilateral_shader, r.outline_shader, r.outline_skinned_shader,
                    r.volumetric_shader, r.motion_blur_shader, r.dof_blur_shader, r.dof_composite_shader,
                    r.luminance_shader, r.adapt_shader, r.wire_shader };
    if (r.local_shadow.fbo) pix_destroy_framebuffer(r.local_shadow);
    if (r.shadow_raw_sampler) glDeleteSamplers(1, &r.shadow_raw_sampler);
    if (r.lum_tex) { glDeleteTextures(1, &r.lum_tex); glDeleteFramebuffers(1, &r.lum_fbo);
                     glDeleteTextures(2, r.adapt_tex); glDeleteFramebuffers(2, r.adapt_fbo); }
    for (size_t i = 0; i < PIX_ARRAY_COUNT(progs); i++) opengl_destroy_shader(progs[i]);
    for (size_t i = 0; i < r.shader_count; i++) opengl_destroy_shader(r.shaders[i]);
    for (int i = 0; i < r.post_effect_count; i++) opengl_destroy_shader(r.post_effects[i].program);
    opengl_destroy_texture(r.white_texture);
    opengl_destroy_texture(r.dummy_shadow);
    opengl_destroy_texture(r.dummy_cube);
    opengl_destroy_texture(r.env_cube);
    opengl_destroy_texture(r.environment.cube);
    opengl_destroy_shader(r.environment.convert_shader);
    opengl_destroy_shader(r.environment.prefilter_shader);
    opengl_destroy_texture(r.refract_tex);
    opengl_destroy_texture(r.refract_depth);
    if (r.env_fbo) glDeleteFramebuffers(1, &r.env_fbo);
    if (r.env_depth) glDeleteRenderbuffers(1, &r.env_depth);
    if (r.planar_target.fbo) pix_destroy_framebuffer(r.planar_target);
    void* heap[] = { r.meshes, r.skinned_meshes, r.materials, r.free_mesh_ids, r.free_skinned_ids,
                     r.free_material_ids, r.lod_of, r.lod_chains, r.pool.free_vertices.ranges, r.pool.free_indices.ranges,
                     r.skin_pool.free_vertices.ranges, r.skin_pool.free_indices.ranges, r.instances, r.animated_instances,
                     (void*)r.animated_bones, r.animated_bone_count, r.skin_list_staging, r.skin_bone_staging, r.particles, r.quads, r.quad_staging, r.quad_order, r.quad_depth, r.velocity_staging, r.motion_table, r.lines, r.lights, r.staging, r.bounds, r.flags,
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
    if (!vertex_count || !index_count) return PIX_INVALID;
    if (r.mesh_count >= PIX_MAX_MESHES && !r.free_mesh_id_count) return PIX_INVALID;
    // tangents the caller left zero are derived here, on a copy (the input is const)
    vertex* derived = 0;
    if (pix_tangents_missing(vertices, vertex_count, sizeof(vertex), offsetof(vertex, tangent))) {
        derived = pix_new<vertex>(vertex_count);
        if (derived) {
            memcpy(derived, vertices, vertex_count * sizeof(vertex));
            pix_compute_tangents(derived, vertex_count, sizeof(vertex), offsetof(vertex, tangent),
                                 indices, index_count, true);
            vertices = derived;
        }
    }
    size_t vo, io;
    pix__pool_alloc(r, r.pool, vertex_count, index_count, &vo, &io);
    idx id = r.free_mesh_id_count ? r.free_mesh_ids[--r.free_mesh_id_count] : (idx)r.mesh_count++;
    mesh& m = r.meshes[id];
    m.vbo = r.pool.vbo;
    m.vertex_offset = (idx)vo;
    m.vertex_size = sizeof(vertex);
    m.vertex_count = (idx)vertex_count;
    m.index_offset = (idx)io;
    m.index_count = (idx)index_count;
    pix__mesh_bounds(m, vertices, vertex_count, sizeof(vertex));
    pix__pool_upload(r.pool, vo, vertices, vertex_count, io, indices, index_count);
    pix_free(derived);
    return id;
}

void pix_unload_mesh(pix_renderer& r, idx id) {
    if (id >= r.mesh_count || !r.meshes[id].index_count) return;
    if (r.lod_of[id]) {
        pix_mesh_lods& c = r.lod_chains[r.lod_of[id] - 1];
        for (int i = 0; i < c.count; i++) pix_unload_mesh(r, c.meshes[i]);
        pix_set_mesh_lods(r, id, 0, 0, 0);
    }
    pix__pool_free(r.pool, r.meshes[id]);
    r.free_mesh_ids[r.free_mesh_id_count++] = id;
}

bool pix_set_mesh_lods(pix_renderer& r, idx base, const idx* lods, const float* screen, int count) {
    if (base >= r.mesh_count || count < 0 || count > PIX_MAX_LODS) return false;
    if (count == 0) {
        if (r.lod_of[base]) r.lod_chains[r.lod_of[base] - 1].count = 0;   // the slot is free again
        r.lod_of[base] = 0;
        return true;
    }
    for (int i = 0; i < count; i++) if (lods[i] >= r.mesh_count) return false;
    size_t slot = r.lod_of[base] ? r.lod_of[base] - 1 : r.lod_chain_count;
    if (!r.lod_of[base])
        for (size_t i = 0; i < r.lod_chain_count; i++) if (!r.lod_chains[i].count) { slot = i; break; }
    if (slot >= PIX_MAX_MESHES) return false;
    if (slot == r.lod_chain_count) r.lod_chain_count++;
    pix_mesh_lods& c = r.lod_chains[slot];
    c.count = count;
    for (int i = 0; i < count; i++) { c.meshes[i] = lods[i]; c.screen[i] = screen[i]; }
    r.lod_of[base] = (uint16_t)(slot + 1);
    return true;
}

int pix_generate_mesh_lods(pix_renderer& r, idx base, int levels, float ratio) {
    if (base >= r.mesh_count || !r.meshes[base].index_count || levels < 1) return 0;
    if (levels > PIX_MAX_LODS) levels = PIX_MAX_LODS;
    const mesh& m = r.meshes[base];
    // the mesh lives only on the GPU: read it back (a load-time stall, once)
    pix_mesh_data md = {};
    md.vertices = pix_new<vertex>(m.vertex_count);
    md.indices = pix_new<uint32_t>(m.index_count);
    if (!md.vertices || !md.indices) { pix_mesh_free(md); return 0; }
    md.vertex_count = m.vertex_count;
    md.index_count = m.index_count;
    glBindBuffer(GL_COPY_READ_BUFFER, r.pool.vbo);
    glGetBufferSubData(GL_COPY_READ_BUFFER, (GLintptr)(m.vertex_offset * sizeof(vertex)),
                       (GLsizeiptr)(m.vertex_count * sizeof(vertex)), md.vertices);
    glBindBuffer(GL_COPY_READ_BUFFER, r.pool.ebo);
    glGetBufferSubData(GL_COPY_READ_BUFFER, (GLintptr)(m.index_offset * sizeof(uint32_t)),
                       (GLsizeiptr)(m.index_count * sizeof(uint32_t)), md.indices);
    glBindBuffer(GL_COPY_READ_BUFFER, 0);
    pix_mesh_weld(md);   // per-corner files simplify to nothing unless their vertices are shared

    idx lods[PIX_MAX_LODS];
    float screen[PIX_MAX_LODS];
    int made = 0;
    size_t tris = md.index_count / 3;
    for (int i = 0; i < levels; i++) {
        size_t left = pix_mesh_simplify(md, ratio);
        if (left < 4 || left > tris * 9 / 10) break;   // nothing left, or it would not simplify
        idx lod = load_mesh_data(r, md.vertices, md.vertex_count, md.indices, md.index_count);
        if (lod == PIX_INVALID) break;
        lods[made] = lod;
        screen[made] = 0.3f * powf(0.5f, (float)made);
        made++;
        tris = left;
    }
    pix_mesh_free(md);
    if (made) pix_set_mesh_lods(r, base, lods, screen, made);
    return made;
}

void pix_unload_skinned_mesh(pix_renderer& r, idx id) {
    if (id >= r.skinned_mesh_count || !r.skinned_meshes[id].index_count) return;
    pix__pool_free(r.skin_pool, r.skinned_meshes[id]);
    r.free_skinned_ids[r.free_skinned_id_count++] = id;
}

idx load_mesh(pix_renderer& r, mesh_file_data& md) {
    return load_mesh_data(r, md.vertex_data, md.vertex_count, md.index_data, md.index_count);
}

idx load_skinned_mesh(pix_renderer& r, skinned_mesh_file_data& md) {
    if (!md.vertex_count || !md.index_count) return PIX_INVALID;
    if (r.skinned_mesh_count >= PIX_MAX_MESHES && !r.free_skinned_id_count) return PIX_INVALID;
    // the file data is the caller's to edit, so missing tangents are filled in place
    if (pix_tangents_missing(md.vertex_data, md.vertex_count, sizeof(vertex_rigged), offsetof(vertex_rigged, tangent)))
        pix_compute_tangents(md.vertex_data, md.vertex_count, sizeof(vertex_rigged), offsetof(vertex_rigged, tangent),
                             md.index_data, md.index_count, true);
    size_t vo, io;
    pix__pool_alloc(r, r.skin_pool, md.vertex_count, md.index_count, &vo, &io);
    idx id = r.free_skinned_id_count ? r.free_skinned_ids[--r.free_skinned_id_count] : (idx)r.skinned_mesh_count++;
    mesh& m = r.skinned_meshes[id];
    m.vbo = r.skin_pool.vbo;
    m.vertex_offset = (idx)vo;
    m.vertex_size = sizeof(vertex_rigged);
    m.vertex_count = (idx)md.vertex_count;
    m.index_offset = (idx)io;
    m.index_count = (idx)md.index_count;
    pix__mesh_bounds(m, md.vertex_data, md.vertex_count, sizeof(vertex_rigged));
    pix__pool_upload(r.skin_pool, vo, md.vertex_data, md.vertex_count, io, md.index_data, md.index_count);
    return id;
}

idx load_mesh_from_skinned(pix_renderer& r, const skinned_mesh_file_data& md) {
    vertex* v = pix_alloc<vertex>(md.vertex_count);
    if (!v) return PIX_INVALID;
    for (size_t i = 0; i < md.vertex_count; i++) {
        v[i].position = md.vertex_data[i].position;
        v[i].normal   = md.vertex_data[i].normal;
        v[i].uv       = md.vertex_data[i].uv;
        v[i].tangent  = md.vertex_data[i].tangent;
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
    m.normal_scale = 1.0f;
    m.occlusion = 1.0f;
    m.alpha_cutoff = 0.5f;    // cut-out cards work out of the box; opaque textures have alpha 1
    m.blend = PIX_BLEND_OPAQUE;
    m.cast_shadows = true;
    m.planar_reflection = 0.0f;
    m.refraction = 0.0f;
    m.toon = 0.0f;
    m.toon_bands = 3.0f;
    m.rim = 0.0f;
    m.toon_specular = 0.0f;
    m.outline_width = 0.0f;
    m.outline_color = v3(0.0f, 0.0f, 0.0f);
    m.outline_group = 0;
    return m;
}

idx pix_add_material(pix_renderer& r, const material& m) {
    // An overrun here would corrupt whatever follows the table and fail frames
    // later somewhere else entirely; refuse loudly instead.
    if (r.material_count >= PIX_MAX_MATERIALS && !r.free_material_id_count) {
        pix_log("pix: material table full (PIX_MAX_MATERIALS = %d)", PIX_MAX_MATERIALS);
        return (idx)(r.material_count ? 0 : PIX_INVALID);
    }
    idx id = r.free_material_id_count ? r.free_material_ids[--r.free_material_id_count] : (idx)r.material_count++;
    r.materials[id] = m;
    return id;
}

void pix_unload_material(pix_renderer& r, idx id) {
    if (id >= r.material_count) return;
    for (size_t i = 0; i < r.free_material_id_count; i++) if (r.free_material_ids[i] == id) return;
    r.materials[id] = pix_material_default();
    r.free_material_ids[r.free_material_id_count++] = id;
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
        // and a prepass twin, so what the vertex stage moves has matching normals and depth
        if (r.prepass_twin_count < MAX_SHADERS) {
            const char* pchunks[] = { GLSL_SCENE, GLSL_COMMON };
            char* pv = pix_shader_splice(vsrc, pchunks, 2);
            char* pf = shader_with_common(FSHDER_PREPASS);
            idx pre = (pv && pf) ? opengl_create_shader(pv, pf) : 0;
            pix_free(pv);
            pix_free(pf);
            if (pre) {
                r.prepass_twins[r.prepass_twin_count].surface = prog;
                r.prepass_twins[r.prepass_twin_count++].prepass = pre;
                if (clips && r.clip_program_count < MAX_SHADERS) r.clip_programs[r.clip_program_count++] = { pre, clips };
                if (r.shader_count < MAX_SHADERS) r.shaders[r.shader_count++] = pre;
            }
        }
    }
    if (prog) pix__scene_sampler_units((GLuint)prog);
    return prog;
}

idx load_surface_shader_stages(pix_renderer& r, const char* fsrc, const char* vsrc, const char* gsrc,
                               const char* tcsrc, const char* tesrc) {
    if (!gsrc && !tcsrc && !tesrc) return load_surface_shader(r, fsrc, vsrc);
    const char* chunks[] = { GLSL_SCENE, GLSL_COMMON };
    char* v  = pix_shader_splice(vsrc ? vsrc : VSHDER_BASIC, chunks, 2);
    char* g  = gsrc  ? pix_shader_splice(gsrc, chunks, 2)  : 0;
    char* tc = tcsrc ? pix_shader_splice(tcsrc, chunks, 2) : 0;
    char* te = tesrc ? pix_shader_splice(tesrc, chunks, 2) : 0;
    char* fr = shader_with_pbr(fsrc);
    idx prog = (v && fr) ? opengl_create_program(v, tc, te, g, fr) : 0;
    pix_free(v); pix_free(g); pix_free(tc); pix_free(te); pix_free(fr);
    if (!prog) return 0;
    if (r.shader_count < MAX_SHADERS) r.shaders[r.shader_count++] = prog;
    if ((tcsrc || tesrc) && r.patch_program_count < MAX_SHADERS) r.patch_programs[r.patch_program_count++].program = prog;
    pix__scene_sampler_units((GLuint)prog);
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
    if (r.frame_number == 0) r.prev_view_proj = r.view_proj;
    r.frame_prepared = false;
    r.renderable_count = 0;
    r.animated_count = 0;
    r.particle_count = 0;
    r.quad_count = 0;
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
    it.lod_fade = 0.0f;
    it.motion_id = 0;
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
    pix__scene_sampler_units((GLuint)p);
}

idx load_skinned_surface_shader(pix_renderer& r, const char* fsrc) {
    idx prog = pix__compile_skinned(VSHDER_SKINNED, fsrc, true);
    if (prog && r.shader_count < MAX_SHADERS) r.shaders[r.shader_count++] = prog;
    if (prog) pix__scene_sampler_units((GLuint)prog);
    return prog;
}

void push_quad3d(pix_renderer& r, const pix_quad3d& q) {
    if (r.quad_count >= PIX_MAX_QUADS || q.color.w <= 0.0f) return;
    r.quads[r.quad_count++] = q;
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
// fields is most of them. Linear in the count, no comparisons, stable. All
// eight digit histograms come from one read of the keys, and the sorted
// values are returned where the last pass left them (sort_vals or
// sort_vals_tmp) rather than copied back: callers copy them once, to where
// they are kept.
static const uint32_t* pix__radix_sort(pix_renderer& r, size_t n) {
    uint64_t* k = r.sort_keys;     uint64_t* kt = r.sort_keys_tmp;
    uint32_t* v = r.sort_vals;     uint32_t* vt = r.sort_vals_tmp;
    if (n < 2) return v;
    uint32_t count[8][256];
    memset(count, 0, sizeof(count));
    for (size_t i = 0; i < n; i++) {
        uint64_t key = k[i];
        for (int d = 0; d < 8; d++) count[d][(key >> (d * 8)) & 255]++;
    }
    for (int d = 0; d < 8; d++) {
        int shift = d * 8;
        uint32_t* c = count[d];
        if (c[(k[0] >> shift) & 255] == n) continue;   // every key has this digit: nothing moves
        uint32_t sum = 0;
        for (int b = 0; b < 256; b++) { uint32_t t = c[b]; c[b] = sum; sum += t; }
        for (size_t i = 0; i < n; i++) {
            uint32_t at = c[(k[i] >> shift) & 255]++;
            kt[at] = k[i];
            vt[at] = v[i];
        }
        uint64_t* tk = k; k = kt; kt = tk;
        uint32_t* tv = v; v = vt; vt = tv;
    }
    return v;
}

static uint64_t pix__batch_key(const pix_render_instance& it) {
    return ((uint64_t)(it.material & 0xFFFF) << 48) | ((uint64_t)(it.mesh & 0xFFFF) << 32);
}

static uint32_t pix__depth16(float d, float far_plane) {
    float t = d / (far_plane > 1e-3f ? far_plane : 1.0f);
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return (uint32_t)(t * 65535.0f);
}

// Maps part of the buffer bound to `target` for writing only, without
// waiting on the GPU: for storage just (re)specified this frame, or a range
// no draw has used yet. Null if the driver declines; the caller then fills a
// staging copy and uploads that instead.
static void* pix__map_write(GLenum target, size_t offset, size_t bytes) {
    if (!bytes) return 0;
    return glMapBufferRange(target, (GLintptr)offset, (GLsizeiptr)bytes,
                            GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_RANGE_BIT | GL_MAP_UNSYNCHRONIZED_BIT);
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
    r.skin_reuse.valid = false;     // a new frame: new instances, a fresh list buffer
    if (!r.animated_count) return;
    size_t total = 0;
    for (size_t k = 0; k < r.animated_count; k++) total += r.animated_bone_count[k];
    // every palette back to back, copied from the animators straight into the
    // freshly specified buffer (through a staging copy only if it cannot be mapped)
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, r.skin_bones_ssbo);
    if (total > r.skin_bones_capacity) r.skin_bones_capacity = total * 2;
    glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)(r.skin_bones_capacity * sizeof(mat4)), 0, GL_STREAM_DRAW);
    mat4* bones = (mat4*)pix__map_write(GL_SHADER_STORAGE_BUFFER, 0, total * sizeof(mat4));
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!bones) {
            if (total > r.skin_bone_staging_capacity) {
                size_t cap = r.skin_bone_staging_capacity ? r.skin_bone_staging_capacity : 4096;
                while (cap < total) cap *= 2;
                mat4* grown = (mat4*)pix_mem_realloc(r.skin_bone_staging, cap * sizeof(mat4));
                if (!grown) { r.animated_count = 0; glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); return; }
                r.skin_bone_staging = grown;
                r.skin_bone_staging_capacity = cap;
            }
        }
        mat4* out = bones ? bones : r.skin_bone_staging;
        size_t at = 0;
        for (size_t k = 0; k < r.animated_count; k++) {
            memcpy(out + at, r.animated_bones[k], r.animated_bone_count[k] * sizeof(mat4));
            at += r.animated_bone_count[k];
        }
        if (bones && glUnmapBuffer(GL_SHADER_STORAGE_BUFFER)) break;
        if (!bones) {
            glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, (GLsizeiptr)(total * sizeof(mat4)), r.skin_bone_staging);
            break;
        }
        bones = 0;    // the mapping's contents were lost: once more through the staging copy
    }
    // the records borrow the instance staging array, which streaming only
    // uses when it cannot map the instance buffer - and never before this
    static_assert(sizeof(pix__gpu_instance) * PIX_MAX_INSTANCES >= sizeof(pix__skin_record) * PIX_MAX_ANIMATED,
                  "PIX_MAX_ANIMATED too large for the shared staging array");
    pix__skin_record* rec = (pix__skin_record*)r.staging;
    size_t at = 0;
    for (size_t k = 0; k < r.animated_count; k++) {
        rec[k].model = r.animated_instances[k].transform;
        rec[k].tint = r.animated_instances[k].tint;
        rec[k].info[0] = (uint32_t)at;
        rec[k].info[1] = rec[k].info[2] = rec[k].info[3] = 0;
        at += r.animated_bone_count[k];
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, r.skin_inst_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)(r.animated_count * sizeof(pix__skin_record)), rec, GL_STREAM_DRAW);
    // index lists: a few views and cascades per frame, each at most every instance
    size_t list_cap = r.animated_count * 16;
    if (list_cap > r.skin_list_capacity) r.skin_list_capacity = list_cap;
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, r.skin_list_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)(r.skin_list_capacity * sizeof(uint32_t)), 0, GL_STREAM_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}
// Swaps each instance of a mesh with LODs for the level its size on screen
// calls for. Just past a switch point (within r.lod_band of it) the instance
// is drawn at both levels, sharing the pixels by a dither pattern whose split
// follows the size - so the change happens gradually as the camera moves.
static void pix__select_lods(pix_renderer& r) {
    const camera& cam = r.frame_camera;
    vec3 eye = cam.position;
    float half = cam.ortho_height > 0.0f ? cam.ortho_height * 0.5f
               : tanf((cam.fovy > 0.0f ? cam.fovy : r.default_fovy) * 0.5f);
    float band = r.lod_band > 0.0f ? r.lod_band : 0.0f;
    size_t n = r.renderable_count;
    for (size_t i = 0; i < n; i++) {
        pix_render_instance& it = r.instances[i];
        uint16_t chain = r.lod_of[it.mesh];
        if (!chain) continue;
        const pix_mesh_lods& c = r.lod_chains[chain - 1];
        const mesh& base = r.meshes[it.mesh];
        float rad = base.radius * mat4_max_scale(it.transform);
        float d = v3dist(mat4_mul_point(it.transform, base.center), eye);
        float span = cam.ortho_height > 0.0f ? half : d * half;   // half the screen's height, world units, there
        float s = rad / (span > 1e-4f ? span : 1e-4f) * r.lod_bias;
        int level = 0;
        while (level < c.count && s < c.screen[level]) level++;
        idx here = level == 0 ? it.mesh : c.meshes[level - 1];
        if (level < c.count && band > 0.0f && s < c.screen[level] * (1.0f + band)
            && r.renderable_count < PIX_MAX_INSTANCES) {
            // k: how far the size is through the band, 0 at the switch point
            float k = (s - c.screen[level]) / (c.screen[level] * band);
            k = k < 0.001f ? 0.001f : (k > 0.999f ? 0.999f : k);
            pix_render_instance& next = r.instances[r.renderable_count++];
            next = it;
            next.mesh = c.meshes[level];
            next.lod_fade = -k;     // the coarser level: the pattern above k
            it.lod_fade = k;        // this level: below k
        }
        it.mesh = here;
    }
}

static void pix__prepare_frame(pix_renderer& r) {
    if (r.frame_prepared) return;
    pix__sync_settings(r);
    r.frame_draws = r.frame_instances = r.frame_culled = 0;
    if (r.lod_chain_count) pix__select_lods(r);
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

static void pix__fill_instances(const pix_renderer& r, const pix__draw_list& list, pix__gpu_instance* out) {
    for (size_t i = 0; i < list.count; i++) {
        const pix_render_instance& it = r.instances[list.items[i]];
        out[i].model = it.transform;
        out[i].tint = it.tint;
        out[i].lod = it.lod_fade;
    }
}

// Writes a sorted list's transforms into the instance stream and records
// where they landed; the draws then name that slice with baseInstance. No two
// uploads in a frame overlap, so none waits on a draw - which is also what
// lets them go straight into the mapped buffer instead of through a staging
// copy. When the frame outgrows the buffer, it is re-specified larger: draws
// already issued keep the storage they were issued against, so nothing in
// flight is disturbed.
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
    size_t offset = r.instance_cursor * sizeof(pix__gpu_instance), bytes = list.count * sizeof(pix__gpu_instance);
    pix__gpu_instance* dst = (pix__gpu_instance*)pix__map_write(GL_ARRAY_BUFFER, offset, bytes);
    if (dst) pix__fill_instances(r, list, dst);
    if (!dst || !glUnmapBuffer(GL_ARRAY_BUFFER)) {   // no mapping, or its contents were lost
        pix__fill_instances(r, list, r.staging);
        glBufferSubData(GL_ARRAY_BUFFER, (GLintptr)offset, (GLsizeiptr)bytes, r.staging);
    }
    list.base = (GLuint)r.instance_cursor;
    r.instance_cursor += list.count;
}

// A view's opaque list (by batch, front to back inside it for early depth
// rejection) and blended list (strictly back to front), culled against its
// frustum, its layers, its distance cap and r.cull_pixels. `pixels` is how
// many pixels one world unit covers at distance 1 (perspective; d away it is
// pixels / d) or anywhere (`ortho`).
static void pix__build_view_lists(pix_renderer& r, const frustum& fr, vec3 eye, float max_dist, float far_plane,
                                  uint32_t layers, bool opaque, bool blended_too, float pixels, bool ortho) {
    size_t n = r.renderable_count, count = 0;
    r.refract_needed = false;
    // too small to see: diameter * pixels / d < cull_pixels, kept free of the division
    float speck = r.cull_pixels > 0.0f ? r.cull_pixels / (2.0f * pixels) : 0.0f;
    if (opaque) {
        for (size_t i = 0; i < n; i++) {
            if ((r.flags[i] & (PIX__BLENDED | PIX__HIDDEN)) || !(r.instances[i].layers & layers)) continue;
            vec3 c = v3(r.bounds[i].x, r.bounds[i].y, r.bounds[i].z);
            float d = v3dist(c, eye);
            if (d - r.bounds[i].w > max_dist || r.bounds[i].w < speck * (ortho ? 1.0f : d)
                || !frustum_test_sphere(fr, c, r.bounds[i].w)) { r.frame_culled++; continue; }
            r.sort_keys[count] = pix__batch_key(r.instances[i]) | ((uint64_t)pix__depth16(d, far_plane) << 16);
            r.sort_vals[count++] = (uint32_t)i;
        }
        memcpy(r.list_opaque.items, pix__radix_sort(r, count), count * sizeof(uint32_t));
    }
    r.list_opaque.count = count;

    count = 0;
    if (blended_too) {
        for (size_t i = 0; i < n; i++) {
            if (!(r.flags[i] & PIX__BLENDED) || (r.flags[i] & PIX__HIDDEN) || !(r.instances[i].layers & layers)) continue;
            vec3 c = v3(r.bounds[i].x, r.bounds[i].y, r.bounds[i].z);
            float d = v3dist(c, eye);
            if (d - r.bounds[i].w > max_dist || r.bounds[i].w < speck * (ortho ? 1.0f : d)
                || !frustum_test_sphere(fr, c, r.bounds[i].w)) { r.frame_culled++; continue; }
            if (r.flags[i] & PIX__REFRACT) r.refract_needed = true;
            uint32_t back = 65535u - pix__depth16(d, far_plane);
            r.sort_keys[count] = ((uint64_t)back << 48) | (pix__batch_key(r.instances[i]) >> 16);
            r.sort_vals[count++] = (uint32_t)i;
        }
        memcpy(r.list_transparent.items, pix__radix_sort(r, count), count * sizeof(uint32_t));
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
static void pix__upload_scene(pix_renderer& r, bool planar, bool probe, vec4 clip, int target_w, int target_h,
                              bool screen_fx = false) {
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
    mat4 inv = mat4_inverse(r.view_proj);
    memcpy(b.inv_view_proj, inv.data, sizeof(b.inv_view_proj));
    memcpy(b.prev_view_proj, r.prev_view_proj.data, sizeof(b.prev_view_proj));
    memcpy(b.proj, r.projection_matrix.data, sizeof(b.proj));
    b.fog_height[0] = r.fog.height;
    b.fog_height[1] = r.fog.height_falloff > 0.0f ? r.fog.height_falloff : 0.0f;
    b.fog_height[2] = r.fog.sun_scatter;
    b.fog_height[3] = r.fog.max_opacity > 0.0f ? r.fog.max_opacity : 1.0f;
    b.screen_fx[0] = screen_fx ? 1.0f : 0.0f;
    b.screen_fx[3] = (float)(r.frame_number & 0xFFFF);
    b.shadow_soft[0] = shadows ? r.shadows.penumbra : 0.0f;
    // texels per radian of a 90-degree tile, the angle one texel spans
    float tile = r.local_shadow.width > 0 ? (float)r.local_shadow.width / 4.0f : 512.0f;
    b.shadow_soft[1] = 2.0f / tile;
    b.shadow_soft[2] = r.cascade_depth;
    b.shadow_soft[3] = 4.0f;
    if (r.environment.cube) {
        b.ibl[0] = 1.0f;
        b.ibl[1] = (float)r.environment.max_mip;
        b.ibl[2] = r.environment.intensity;
        for (int k = 0; k < 9; k++) pix__put4(b.sh + k * 4, r.environment.sh[k], 0.0f);
    }
    glBindBuffer(GL_UNIFORM_BUFFER, r.scene_ubo);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, (GLsizeiptr)sizeof(b), &b);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

// The nearest shadow-casting lights get atlas tiles until there are none
// left: a spot one, a point light six (one per cube face).
static void pix__assign_local_shadows(pix_renderer& r) {
    for (int i = 0; i < MAX_SHADER_LIGHTS; i++) {
        r.local_info[i * 4 + 0] = -1.0f;
        r.local_info[i * 4 + 1] = r.local_info[i * 4 + 2] = r.local_info[i * 4 + 3] = 0.0f;
    }
    r.local_tiles = 0;
    if (!r.shadows.local || !r.local_shadow.fbo) return;
    const pix_light_list& L = *r.lights;
    static const vec3 dirs[6] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
    static const vec3 ups[6]  = { { 0, -1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 }, { 0, -1, 0 }, { 0, -1, 0 } };
    for (int i = 0; i < L.packed_count; i++) {
        const pix_light& l = L.lights[L.packed_index[i]];
        if (!l.cast_shadows) continue;
        bool spot = l.cos_outer > -1.0f;
        int need = spot ? 1 : 6;
        if (r.local_tiles + need > PIX_MAX_SHADOW_TILES) continue;
        r.local_info[i * 4 + 0] = (float)r.local_tiles;
        r.local_info[i * 4 + 1] = (float)need;
        float near_p = 0.05f, far_p = l.radius > 0.1f ? l.radius : 0.1f;
        for (int k = 0; k < need; k++) {
            vec3 dir = spot ? v3norm(l.direction) : dirs[k];
            vec3 up = spot ? (fabsf(dir.y) > 0.99f ? v3(1, 0, 0) : v3(0, 1, 0)) : ups[k];
            // a spot's tile frames its outer cone; a cube face is a little over 90
            // degrees so filtering at a face's border still has texels to read
            float fov = spot ? clampf(2.0f * acosf(clampf(l.cos_outer, -1.0f, 1.0f)) + 0.08f, 0.2f, 2.9f)
                             : 1.6f;
            mat4 proj = mat4_perspective(fov, 1.0f, near_p, far_p);
            int t = r.local_tiles++;
            r.local_tile_vp[t] = mat4_mul(proj, mat4_lookat(l.position, v3add(l.position, dir), up));
            r.local_tile_eye[t] = l.position;
            r.local_tile_far[t] = far_p;
        }
    }
}

static void pix__upload_lights(pix_renderer& r) {
    pix_lights_pack(*r.lights, r.frame_camera.position);
    pix__assign_local_shadows(r);
    pix_light_block lb;
    memcpy(lb.pos_radius, r.lights->pos_radius, sizeof(lb.pos_radius));
    memcpy(lb.color_inner, r.lights->color_inner, sizeof(lb.color_inner));
    memcpy(lb.dir_outer, r.lights->dir_outer, sizeof(lb.dir_outer));
    lb.meta[0] = (float)r.lights->packed_count;
    lb.meta[1] = lb.meta[2] = lb.meta[3] = 0.0f;
    memcpy(lb.shadow, r.local_info, sizeof(lb.shadow));
    for (int t = 0; t < PIX_MAX_SHADOW_TILES; t++)
        memcpy(lb.tiles + t * 16, (t < r.local_tiles ? r.local_tile_vp[t] : mat4_identity()).data, sizeof(float) * 16);
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
    // the probe when this view may sample it, else the HDR environment (bIbl picks)
    glBindTexture(GL_TEXTURE_CUBE_MAP, (env && r.env_cube) ? r.env_cube
                                     : (r.environment.cube ? r.environment.cube : r.dummy_cube));
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, (planar && r.planar_target.color) ? r.planar_target.color : r.white_texture);
    glActiveTexture(GL_TEXTURE0 + 4);
    glBindTexture(GL_TEXTURE_2D, (refract && r.refract_tex) ? r.refract_tex : r.white_texture);
    glActiveTexture(GL_TEXTURE0 + 5);
    glBindTexture(GL_TEXTURE_2D, (refract && r.refract_depth) ? r.refract_depth : r.white_texture);
    glActiveTexture(GL_TEXTURE0 + 10);
    glBindTexture(GL_TEXTURE_2D, r.fx_a.color ? r.fx_a.color : r.white_texture);
    glActiveTexture(GL_TEXTURE0 + 11);
    glBindTexture(GL_TEXTURE_2D, r.local_shadow.depth ? r.local_shadow.depth : r.dummy_shadow);
    glActiveTexture(GL_TEXTURE0 + 12);
    glBindTexture(GL_TEXTURE_2D, (shadows && r.shadow_map.depth) ? r.shadow_map.depth : r.dummy_shadow);
    glBindSampler(12, r.shadow_raw_sampler);
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
    // 10 screen AO / contact shadows, 11 local light shadows, 12 the sun atlas unfiltered (PCSS)
    if (st.u->screen_fx >= 0) glUniform1i(st.u->screen_fx, 10);
    if (st.u->local_shadow >= 0) glUniform1i(st.u->local_shadow, 11);
    if (st.u->shadow_raw >= 0) glUniform1i(st.u->shadow_raw, 12);
    // 13..15 the material's normal, occlusion/roughness/metallic and emissive maps
    if (st.u->normal_map >= 0) glUniform1i(st.u->normal_map, 13);
    if (st.u->orm_map >= 0) glUniform1i(st.u->orm_map, 14);
    if (st.u->emissive_map >= 0) glUniform1i(st.u->emissive_map, 15);
    r.draw_mode = st.u->patches ? GL_PATCHES : GL_TRIANGLES;
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
    if (u->maps >= 0) {
        // the shader samples only the maps the bits name, so stale bindings are harmless
        idx maps[3] = { m.normal_map, m.orm_map, m.emissive_map };
        int bits = 0;
        for (int i = 0; i < 3; i++) {
            if (!maps[i]) continue;
            bits |= 1 << i;
            glActiveTexture(GL_TEXTURE0 + 13 + i);
            glBindTexture(GL_TEXTURE_2D, maps[i]);
        }
        glActiveTexture(GL_TEXTURE0);
        glUniform1i(u->maps, bits);
        glUniform2f(u->map_scale, m.normal_scale, m.occlusion);
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
        r.draw_mode, (GLsizei)me.index_count, GL_UNSIGNED_INT,
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

// Is this material drawn by the scene-buffer prepass? Everything opaque is,
// except a custom vertex stage that has no prepass twin.
static bool pix__in_prepass(const pix_renderer& r, const material& m) {
    if (!m.shader) return true;
    bool custom_vs = false;
    for (size_t i = 0; i < r.depth_twin_count; i++) if (r.depth_twins[i].surface == m.shader) custom_vs = true;
    if (!custom_vs) return true;
    for (size_t i = 0; i < r.prepass_twin_count; i++) if (r.prepass_twins[i].surface == m.shader) return true;
    return false;
}

static GLuint pix__prepass_program(const pix_renderer& r, const material& m) {
    if (m.shader)
        for (size_t i = 0; i < r.prepass_twin_count; i++)
            if (r.prepass_twins[i].surface == m.shader) return (GLuint)r.prepass_twins[i].prepass;
    return (GLuint)r.prepass_shader;
}

// `full_prepass`: the scene-buffer prepass already laid down this list's
// depth, so its surfaces only colour in (depth tested, not written) - which
// also keeps early depth testing on for shaders that may discard.
static void pix__draw_sorted(pix_renderer& r, const pix__draw_list& list, pix__pass_state& st, bool blended,
                             idx override_program = 0, bool prepassed = false, bool full_prepass = false) {
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
        bool write = full_prepass ? !pix__in_prepass(r, m) : !(prepassed && m.cutout && m.texture);
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
#define PIX__SKIN_COLOR   0
#define PIX__SKIN_PREPASS 1
#define PIX__SKIN_OUTLINE 2
static void pix__draw_skinned(pix_renderer& r, pix__pass_state& st, const mat4* light_vp,
                              const frustum* fr, uint32_t layers, int mode = PIX__SKIN_COLOR) {
    if (!r.animated_count) return;
    auto& reuse = r.skin_reuse;
    const uint32_t* order = r.skin_list_staging;
    size_t count = 0, base = 0;
    if (reuse.valid && reuse.layers == layers && reuse.culled == (fr != 0) && (!fr || !memcmp(fr, &reuse.fr, sizeof(frustum)))) {
        count = reuse.count;          // this view's list is up already
        base = reuse.base;
        r.frame_culled += reuse.dropped;
    } else {
        int dropped = 0;
        for (size_t k = 0; k < r.animated_count; k++) {
            const pix_render_instance& inst = r.animated_instances[k];
            if (!(inst.layers & layers)) continue;
            if (fr) {
                const mesh& me = r.skinned_meshes[inst.mesh];
                vec3 c = mat4_mul_point(inst.transform, me.center);
                float rad = me.radius * mat4_max_scale(inst.transform) * 1.5f;   // poses reach past the bind pose
                if (!frustum_test_sphere(*fr, c, rad)) { dropped++; continue; }
            }
            r.sort_keys[count] = pix__batch_key(inst);
            r.sort_vals[count++] = (uint32_t)k;
        }
        r.frame_culled += dropped;
        if (count) {
            memcpy(r.skin_list_staging, pix__radix_sort(r, count), count * sizeof(uint32_t));
            if (r.skin_list_cursor + count > r.skin_list_capacity) {
                // more views than budgeted: a fresh buffer; draws already issued keep theirs
                r.skin_list_capacity = (r.skin_list_capacity + count) * 2;
                glBindBuffer(GL_SHADER_STORAGE_BUFFER, r.skin_list_ssbo);
                glBufferData(GL_SHADER_STORAGE_BUFFER, (GLsizeiptr)(r.skin_list_capacity * sizeof(uint32_t)), 0, GL_STREAM_DRAW);
                r.skin_list_cursor = 0;
            }
            base = r.skin_list_cursor;
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, r.skin_list_ssbo);
            glBufferSubData(GL_SHADER_STORAGE_BUFFER, (GLintptr)(base * sizeof(uint32_t)),
                            (GLsizeiptr)(count * sizeof(uint32_t)), r.skin_list_staging);
            r.skin_list_cursor += count;
        }
        reuse.valid = true;
        reuse.culled = fr != 0;
        if (fr) reuse.fr = *fr;
        reuse.layers = layers;
        reuse.base = base;
        reuse.count = count;
        reuse.dropped = dropped;
    }
    if (!count) return;
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 8, r.skin_inst_ssbo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 9, r.skin_bones_ssbo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 10, r.skin_list_ssbo);

    bool depth_only = light_vp != 0;
    glBindVertexArray(r.skin_pool.vao);
    idx bound = PIX_INVALID;
    size_t i = 0;
    while (i < count) {
        const pix_render_instance& a = r.animated_instances[order[i]];
        size_t j = i + 1;
        while (j < count) {
            const pix_render_instance& b = r.animated_instances[order[j]];
            if (b.mesh != a.mesh || b.material != a.material) break;
            j++;
        }
        const material& m = r.materials[a.material];
        if (mode == PIX__SKIN_OUTLINE && m.outline_width <= 0.0f) { i = j; continue; }
        if (mode == PIX__SKIN_PREPASS) {
            pix__use_program(r, st, r.prepass_skinned_shader);
            if (a.material != bound) {
                glUniform1f(st.u->cutoff, m.texture ? m.alpha_cutoff : 0.0f);
                if (st.u->outline_group >= 0) glUniform1f(st.u->outline_group, (float)m.outline_group / 255.0f);
                glBindTexture(GL_TEXTURE_2D, m.texture ? m.texture : r.white_texture);
                pix__set_cull(st, !m.double_sided);
                bound = a.material;
            }
        } else if (mode == PIX__SKIN_OUTLINE) {
            pix__use_program(r, st, r.outline_skinned_shader);
            glUniform1f(st.u->outline_width, m.outline_width);
            glUniform3f(st.u->outline_color, m.outline_color.x, m.outline_color.y, m.outline_color.z);
        } else if (depth_only) {
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
        glDrawElementsInstancedBaseVertex(r.draw_mode, (GLsizei)me.index_count, GL_UNSIGNED_INT,
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
enum { PIX__KIND_USER, PIX__KIND_PROBE, PIX__KIND_MIRROR };

// The scene-buffer prepass: every opaque surface's depth, view-space normal
// and outline group, drawn with the surfaces' own vertex stages and pushed
// back a hair so the lit pass, with another program, always passes.
static void pix__prepass(pix_renderer& r, const pix_view& v, pix__pass_state& st, int kind) {
    glBindFramebuffer(GL_FRAMEBUFFER, r.prepass_fbo);
    glClearColor(0.5f, 0.5f, 1.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    const pix__draw_list& list = r.list_opaque;
    if (list.count) glBindVertexArray(r.pool.vao);
    size_t i = 0;
    while (i < list.count) {
        const pix_render_instance& a = r.instances[list.items[i]];
        size_t j = i + 1;
        while (j < list.count && r.instances[list.items[j]].mesh == a.mesh && r.instances[list.items[j]].material == a.material) j++;
        const material& m = r.materials[a.material];
        if (pix__in_prepass(r, m)) {
            pix__use_program(r, st, pix__prepass_program(r, m));
            glUniform1f(st.u->cutoff, m.texture ? m.alpha_cutoff : 0.0f);
            if (st.u->outline_group >= 0) glUniform1f(st.u->outline_group, (float)m.outline_group / 255.0f);
            if (st.u->params0 >= 0) glUniform4f(st.u->params0, m.params0.x, m.params0.y, m.params0.z, m.params0.w);
            if (st.u->params1 >= 0) glUniform4f(st.u->params1, m.params1.x, m.params1.y, m.params1.z, m.params1.w);
            glBindTexture(GL_TEXTURE_2D, m.texture ? m.texture : r.white_texture);
            pix__set_cull(st, !m.double_sided);
            pix__draw_run(r, r.meshes[a.mesh], list.base + (GLuint)i, j - i);
        }
        i = j;
    }
    pix__set_clip(r, 0);
    if (v.skinned && r.animated_count)
        pix__draw_skinned(r, st, 0, kind == PIX__KIND_MIRROR ? 0 : &r.view_frustum, v.layers, PIX__SKIN_PREPASS);
    glDisable(GL_POLYGON_OFFSET_FILL);
    st.program = 0; st.material = PIX_INVALID;
}

// Screen AO and contact shadows from the prepass, then an edge-aware blur
// across and down.
static void pix__screen_fx_pass(pix_renderer& r) {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    pix_bind_framebuffer(r.fx_a);
    GLuint p = (GLuint)r.fx_shader;
    glUseProgram(p);
    mat4 inv_proj = mat4_inverse(r.projection_matrix);
    glUniform1i(glGetUniformLocation(p, "uDepth"), 0);
    glUniform1i(glGetUniformLocation(p, "uNormal"), 1);
    glUniformMatrix4fv(glGetUniformLocation(p, "uInvProj"), 1, GL_FALSE, inv_proj.data);
    int samples = r.ao.samples < 4 ? 4 : (r.ao.samples > 32 ? 32 : r.ao.samples);
    glUniform4f(glGetUniformLocation(p, "uParams"), r.ao.enabled ? r.ao.radius : 0.0f, r.ao.intensity,
                (float)samples, r.shadows.enabled && r.shadows.strength > 0.0f ? r.shadows.contact : 0.0f);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, r.normal_tex);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, r.scene_target.depth);
    pix_draw_fullscreen();

    GLuint b = (GLuint)r.bilateral_shader;
    glUseProgram(b);
    glUniform1i(glGetUniformLocation(b, "uSource"), 0);
    glUniform1i(glGetUniformLocation(b, "uDepth"), 1);
    GLint dir = glGetUniformLocation(b, "uDirection");
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, r.scene_target.depth);
    glActiveTexture(GL_TEXTURE0);
    pix_bind_framebuffer(r.fx_b);
    glUniform2f(dir, 1.0f / (float)r.fx_a.width, 0.0f);
    glBindTexture(GL_TEXTURE_2D, r.fx_a.color);
    pix_draw_fullscreen();
    pix_bind_framebuffer(r.fx_a);
    glUniform2f(dir, 0.0f, 1.0f / (float)r.fx_a.height);
    glBindTexture(GL_TEXTURE_2D, r.fx_b.color);
    pix_draw_fullscreen();
    glEnable(GL_DEPTH_TEST);
}

// Ink: every outlined opaque surface again, grown along its normals, front
// faces culled so only the shell behind the surface shows.
static void pix__hull_outlines(pix_renderer& r, const pix_view& v, pix__pass_state& st, int kind) {
    const pix__draw_list& list = r.list_opaque;
    bool any = false;
    for (size_t k = 0; k < list.count && !any; k++) any = r.materials[r.instances[list.items[k]].material].outline_width > 0.0f;
    bool any_skinned = false;
    for (size_t k = 0; k < r.animated_count && !any_skinned; k++)
        any_skinned = r.materials[r.animated_instances[k].material].outline_width > 0.0f;
    if (!any && !any_skinned) return;
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
    st.cull = -1;
    if (any) {
        glBindVertexArray(r.pool.vao);
        pix__use_program(r, st, r.outline_shader);
        size_t i = 0;
        while (i < list.count) {
            const pix_render_instance& a = r.instances[list.items[i]];
            size_t j = i + 1;
            while (j < list.count && r.instances[list.items[j]].mesh == a.mesh && r.instances[list.items[j]].material == a.material) j++;
            const material& m = r.materials[a.material];
            if (m.outline_width > 0.0f) {
                glUniform1f(st.u->outline_width, m.outline_width);
                glUniform3f(st.u->outline_color, m.outline_color.x, m.outline_color.y, m.outline_color.z);
                pix__draw_run(r, r.meshes[a.mesh], list.base + (GLuint)i, j - i);
            }
            i = j;
        }
    }
    if (any_skinned && v.skinned)
        pix__draw_skinned(r, st, 0, kind == PIX__KIND_MIRROR ? 0 : &r.view_frustum, v.layers, PIX__SKIN_OUTLINE);
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
    st.cull = 1;
    st.program = 0; st.material = PIX_INVALID;
}

// Every opaque mesh's edges over the shaded image.
static void pix__wireframe_pass(pix_renderer& r, pix__pass_state& st) {
    const pix__draw_list& list = r.list_opaque;
    if (!list.count) return;
    glBindVertexArray(r.pool.vao);
    pix__use_program(r, st, r.wire_shader);
    pix_color c = r.wireframe_color;
    glUniform3f(st.u->outline_color, c.r / 255.0f, c.g / 255.0f, c.b / 255.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glEnable(GL_POLYGON_OFFSET_LINE);
    glPolygonOffset(-1.0f, -1.0f);
    glDisable(GL_CULL_FACE);
    st.cull = 0;
    size_t i = 0;
    while (i < list.count) {
        const pix_render_instance& a = r.instances[list.items[i]];
        size_t j = i + 1;
        while (j < list.count && r.instances[list.items[j]].mesh == a.mesh && r.instances[list.items[j]].material == a.material) j++;
        pix__draw_run(r, r.meshes[a.mesh], list.base + (GLuint)i, j - i);
        i = j;
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDisable(GL_POLYGON_OFFSET_LINE);
    st.program = 0; st.material = PIX_INVALID;
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
    // the environment borrows unit 0 (uSkyCube's default); the next material rebinds it
    if (r.environment.cube) glBindTexture(GL_TEXTURE_CUBE_MAP, r.environment.cube);
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

// Billboards and 3D text: the cutout ones in the opaque phase (depth written,
// any order), the rest blended back to front. Runs sharing a texture draw as
// one instanced call; a text label is one texture, so one draw.
static float* pix__quad_sort_depth;
static int pix__quad_cmp(const void* a, const void* b) {
    uint32_t i = *(const uint32_t*)a, j = *(const uint32_t*)b;
    float di = pix__quad_sort_depth[i], dj = pix__quad_sort_depth[j];
    if (di != dj) return di < dj ? -1 : 1;       // view z: more negative is further: drawn first
    return i < j ? -1 : (i > j ? 1 : 0);         // stable, so a label's glyphs keep their order
}

static void pix__quad_pass(pix_renderer& r, bool cutout) {
    if (!r.quad_count) return;
    size_t n = 0;
    for (size_t i = 0; i < r.quad_count; i++)
        if (r.quads[i].cutout == cutout) r.quad_order[n++] = (uint32_t)i;
    if (!n) return;
    if (!cutout) {
        // view-space z of each anchor
        const float* m = r.view_matrix.data;
        for (size_t i = 0; i < r.quad_count; i++) {
            vec3 p = r.quads[i].anchor;
            r.quad_depth[i] = m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14];
        }
        pix__quad_sort_depth = r.quad_depth;
        qsort(r.quad_order, n, sizeof(uint32_t), pix__quad_cmp);
    }
    for (size_t k = 0; k < n; k++) {
        const pix_quad3d& q = r.quads[r.quad_order[k]];
        float* o = r.quad_staging + k * 24;
        o[0] = q.anchor.x; o[1] = q.anchor.y; o[2] = q.anchor.z; o[3] = (float)q.facing;
        o[4] = q.rect.x;   o[5] = q.rect.y;   o[6] = q.rect.z;   o[7] = q.rect.w;
        o[8] = q.uv.x;     o[9] = q.uv.y;     o[10] = q.uv.z;    o[11] = q.uv.w;
        o[12] = q.color.x; o[13] = q.color.y; o[14] = q.color.z; o[15] = q.color.w;
        o[16] = q.right.x; o[17] = q.right.y; o[18] = q.right.z; o[19] = q.sdf ? 1.0f : 0.0f;
        o[20] = q.up.x;    o[21] = q.up.y;    o[22] = q.up.z;    o[23] = 0.0f;
    }
    vec3 right = v3(r.view_matrix.data[0], r.view_matrix.data[4], r.view_matrix.data[8]);
    vec3 up    = v3(r.view_matrix.data[1], r.view_matrix.data[5], r.view_matrix.data[9]);
    glUseProgram(r.quad_shader);
    glUniform3f(r.u_quad[0], right.x, right.y, right.z);
    glUniform3f(r.u_quad[1], up.x, up.y, up.z);
    glUniform1i(r.u_quad[2], cutout ? 1 : 0);
    if (cutout) { glDisable(GL_BLEND); glDepthMask(GL_TRUE); }
    else        { glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); glDepthMask(GL_FALSE); }
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glBindVertexArray(r.quad_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.quad_vbo);
    // the views of a frame each upload their own order: orphan, then fill
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(PIX_MAX_QUADS * 24 * sizeof(float)), 0, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(n * 24 * sizeof(float)), r.quad_staging);
    glActiveTexture(GL_TEXTURE0);
    for (size_t k = 0; k < n;) {
        idx tex = r.quads[r.quad_order[k]].texture;
        size_t end = k + 1;
        while (end < n && r.quads[r.quad_order[end]].texture == tex) end++;
        glBindTexture(GL_TEXTURE_2D, tex ? tex : r.white_texture);
        glDrawArraysInstancedBaseInstance(GL_TRIANGLE_STRIP, 0, 4, (GLsizei)(end - k), (GLuint)k);
        r.frame_draws++;
        k = end;
    }
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
    v.opaque = v.skinned = v.transparent = v.particles = v.lines = v.quads = true;
    v.shadows = true;
    v.reflections = true;
    return v;
}


// Renders `v` into whatever framebuffer is bound by the caller (`fbo`, sized
// fb_w x fb_h). `color_tex`/`depth_tex` are that framebuffer's attachments,
// copied from for refraction; 0 when it has none.
// The multisampled main view into the scene target: colour averaged, depth
// one sample per pixel (the screen effects read it as plain depth).
static void pix__resolve_msaa(pix_renderer& r) {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, r.msaa_fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, r.scene_target.fbo);
    glBlitFramebuffer(0, 0, r.width, r.height, 0, 0, r.width, r.height,
                      GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, r.scene_target.fbo);
}

// Where the object with this motion_id was last frame (its transform now, if
// it was not seen then). One table, entries stamped with the frame they were
// last updated in, so nothing has to be cleared between frames.
#define PIX__MOTION_SLOTS 16384
static mat4 pix__motion_prev(pix_renderer& r, uint32_t id, const mat4& now) {
    uint32_t frame = (uint32_t)r.frame_number + 1;   // + 1: 0 marks a never-used slot
    uint32_t h = id * 2654435761u, free_slot = 0xFFFFFFFFu;
    for (uint32_t probe = 0; probe < PIX__MOTION_SLOTS; probe++) {
        pix_renderer::pix__motion_entry& e = r.motion_table[(h + probe) & (PIX__MOTION_SLOTS - 1)];
        if (e.frame && e.id == id) {
            if (e.frame == frame) return e.prev;     // seen already this frame (a LOD's twin)
            e.prev = e.frame + 1 == frame ? e.cur : now;
            e.cur = now;
            e.frame = frame;
            return e.prev;
        }
        // stale entries (not seen last frame) can be taken over
        if (free_slot == 0xFFFFFFFFu && (!e.frame || e.frame + 1 < frame)) free_slot = (h + probe) & (PIX__MOTION_SLOTS - 1);
        if (!e.frame) break;   // the end of this id's chain
    }
    if (free_slot != 0xFFFFFFFFu) {
        pix_renderer::pix__motion_entry& e = r.motion_table[free_slot];
        e.id = id; e.frame = frame; e.cur = now; e.prev = now;
    }
    return now;
}

// The velocity buffer: cleared to "ask the depth" (the camera's motion), then
// every opaque instance with a motion_id draws its own motion over it.
static void pix__velocity_pass(pix_renderer& r) {
    glBindFramebuffer(GL_FRAMEBUFFER, r.velocity_fbo);
    glViewport(0, 0, r.width, r.height);
    glClearColor(1000.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    size_t n = 0;
    for (size_t i = 0; i < r.renderable_count; i++)
        if (r.instances[i].motion_id && !(r.flags[i] & (PIX__BLENDED | PIX__HIDDEN))) n++;
    if (!n) return;
    if (!r.motion_table) {
        r.motion_table = pix_alloc<pix_renderer::pix__motion_entry>(PIX__MOTION_SLOTS);
        if (!r.motion_table) return;
    }
    if (n > r.velocity_capacity) {
        size_t cap = r.velocity_capacity ? r.velocity_capacity : 256;
        while (cap < n) cap *= 2;
        pix_free(r.velocity_staging);
        r.velocity_staging = pix_alloc<float>(cap * 32);
        if (!r.velocity_staging) { r.velocity_capacity = 0; return; }
        r.velocity_capacity = cap;
    }
    // this frame's and last frame's transform per instance, written straight
    // into the freshly specified buffer (pix__motion_prev answers the same
    // twice in a frame, so the staging fallback can simply fill again)
    auto fill = [&](float* out) {
        size_t k = 0;
        for (size_t i = 0; i < r.renderable_count; i++) {
            const pix_render_instance& it = r.instances[i];
            if (!it.motion_id || (r.flags[i] & (PIX__BLENDED | PIX__HIDDEN))) continue;
            mat4 prev = pix__motion_prev(r, it.motion_id, it.transform);
            memcpy(out + k * 32, it.transform.data, 16 * sizeof(float));
            memcpy(out + k * 32 + 16, prev.data, 16 * sizeof(float));
            k++;
        }
    };
    // the pool's buffers change when it grows, so the layout is set each time
    glBindVertexArray(r.velocity_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.pool.vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, position));
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, r.velocity_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(r.velocity_capacity * 32 * sizeof(float)), 0, GL_STREAM_DRAW);
    float* mapped = (float*)pix__map_write(GL_ARRAY_BUFFER, 0, n * 32 * sizeof(float));
    if (mapped) fill(mapped);
    if (!mapped || !glUnmapBuffer(GL_ARRAY_BUFFER)) {
        fill(r.velocity_staging);
        glBufferSubData(GL_ARRAY_BUFFER, 0, (GLsizeiptr)(n * 32 * sizeof(float)), r.velocity_staging);
    }
    for (int c = 0; c < 8; c++) {
        glVertexAttribPointer(1 + c, 4, GL_FLOAT, GL_FALSE, 32 * sizeof(float), (void*)(size_t)(c * 4 * sizeof(float)));
        glEnableVertexAttribArray(1 + c);
        glVertexAttribDivisor(1 + c, 1);
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r.pool.ebo);
    GLuint p = (GLuint)r.velocity_shader;
    glUseProgram(p);
    glUniformMatrix4fv(glGetUniformLocation(p, "uJitteredViewProj"), 1, GL_FALSE, r.taa_jittered_view_proj.data);
    glUniformMatrix4fv(glGetUniformLocation(p, "uCurViewProj"), 1, GL_FALSE, r.taa_view_proj.data);
    glUniformMatrix4fv(glGetUniformLocation(p, "uPrevViewProj"), 1, GL_FALSE,
                       (r.taa_valid ? r.taa_prev_view_proj : r.taa_view_proj).data);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    size_t k = 0;
    for (size_t i = 0; i < r.renderable_count; i++) {
        const pix_render_instance& it = r.instances[i];
        if (!it.motion_id || (r.flags[i] & (PIX__BLENDED | PIX__HIDDEN))) continue;
        const mesh& me = r.meshes[it.mesh];
        glDrawElementsInstancedBaseVertexBaseInstance(GL_TRIANGLES, (GLsizei)me.index_count, GL_UNSIGNED_INT,
            (void*)(size_t)(me.index_offset * sizeof(uint32_t)), 1, (GLint)me.vertex_offset, (GLuint)k);
        k++;
    }
    glDepthMask(GL_TRUE);
    glBindVertexArray(0);
}

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
    // the screen-sized main view: TAA jitters it, MSAA draws it multisampled
    bool main_view = kind == PIX__KIND_USER && fbo == r.scene_target.fbo && fb_w == r.width && fb_h == r.height
                  && vw == fb_w && vh == fb_h;
    bool taa = main_view && r.aa.taa && r.taa_history[0].fbo && !r.taa_jittered;
    bool msaa = main_view && r.msaa_fbo;
    if (taa) {
        // Halton (2, 3): eight offsets that cover the pixel evenly in any window of frames
        int k = (int)(r.frame_number % 8) + 1;
        float hx = 0.0f, hy = 0.0f;
        for (float f = 0.5f, i = (float)k; i > 0.0f; f *= 0.5f, i = floorf(i / 2.0f)) hx += f * fmodf(i, 2.0f);
        for (float f = 1.0f / 3.0f, i = (float)k; i > 0.0f; f /= 3.0f, i = floorf(i / 3.0f)) hy += f * fmodf(i, 3.0f);
        r.taa_jitter = v2(hx - 0.5f, hy - 0.5f);
        r.taa_view_proj = r.view_proj;
        // shift the projection in clip space, so it works for ortho cameras too
        mat4 shift = mat4_translate(2.0f * r.taa_jitter.x / (float)fb_w, 2.0f * r.taa_jitter.y / (float)fb_h, 0.0f);
        r.projection_matrix = mat4_mul(shift, r.projection_matrix);
        r.view_proj = mat4_mul(r.projection_matrix, r.view_matrix);
        r.taa_jittered_view_proj = r.view_proj;
        r.taa_jittered = true;
    }
    GLuint draw_fbo = msaa ? r.msaa_fbo : fbo;
    float max_dist = v.max_distance > 0.0f ? v.max_distance : r.camera_far;
    // the projection's y scale: how big a world unit is on this view's rows
    float pixels = r.projection_matrix.data[5] * 0.5f * (float)vh;
    pix__build_view_lists(r, r.view_frustum, r.camera_position, max_dist, r.camera_far, v.layers,
                          v.opaque, v.transparent, pixels, r.projection_matrix.data[15] != 0.0f);
    pix__stream_list(r, r.list_opaque);
    pix__stream_list(r, r.list_transparent);

    bool shadows = v.shadows && r.shadows.enabled && r.shadow_map.fbo && r.shadows.strength > 0.0f;
    bool planar = v.reflections && kind == PIX__KIND_USER;
    bool probe  = v.reflections && kind != PIX__KIND_PROBE;
    bool partial = vx != 0 || vy != 0 || vw != fb_w || vh != fb_h;
    // the scene buffers belong to the screen-sized main view
    bool pre = kind == PIX__KIND_USER && v.opaque && !v.override_shader && !partial && v.scissor.w <= 0.0f
            && r.prepass_fbo && depth_tex && depth_tex == r.scene_target.depth && fb_w == r.width && fb_h == r.height;
    bool fx = pre && r.fx_a.fbo && (r.ao.enabled || (r.shadows.contact > 0.0f && shadows));
    if (kind == PIX__KIND_USER) { r.prepass_done = pre; r.fx_valid = fx; }
    pix__upload_scene(r, planar, probe, v.clip_plane, fb_w, fb_h, fx);

    glBindFramebuffer(GL_FRAMEBUFFER, draw_fbo);
    glViewport(vx, vy, vw, vh);
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

    pix__pass_state st = { 0, 0, PIX_INVALID, -1, -1 };
    if (pre) {
        pix__prepass(r, v, st, kind);
        if (fx) pix__screen_fx_pass(r);
        glBindFramebuffer(GL_FRAMEBUFFER, draw_fbo);
        glViewport(vx, vy, vw, vh);
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        st.cull = -1;
    }
    pix__bind_frame_textures(r, shadows, probe, planar, false);
    // multisampled, the lit pass has its own depth: the prepass laid it in the
    // single-sample scene depth, which the screen effects read, not in this one
    bool depth_laid = pre && !msaa;
    bool prepassed = depth_laid || (v.opaque && !v.override_shader && kind == PIX__KIND_USER && pix__cutout_prepass(r, r.list_opaque, st));
    if (v.opaque) pix__draw_sorted(r, r.list_opaque, st, false, v.override_shader, prepassed, depth_laid);
    if (v.skinned && r.animated_count) {
        if (depth_laid) glDepthMask(GL_FALSE);
        pix__draw_skinned(r, st, 0, kind == PIX__KIND_MIRROR ? 0 : &r.view_frustum, v.layers);
        glDepthMask(GL_TRUE);
    }
    if (v.opaque && !v.override_shader && kind != PIX__KIND_PROBE) pix__hull_outlines(r, v, st, kind);
    if (r.wireframe && kind == PIX__KIND_USER && v.opaque) pix__wireframe_pass(r, st);
    pix__run_hooks(r, st, PIX_HOOK_OPAQUE, kind, -1, r.view_proj, v.layers, fb_w, fb_h);
    if (v.quads) { pix__quad_pass(r, true); st.program = 0; st.cull = -1; }
    if (v.sky) { pix__sky_pass(r, r.view_proj); st.program = 0; st.cull = -1; }
    if (v.lines) pix__line_pass(r);

    if (r.list_transparent.count) {
        if (r.refract_needed && color_tex && depth_tex) {
            // what is behind the glass: one GPU-side copy of this view's opaque result
            if (msaa) {   // resolved first: the copy reads the single-sample target
                pix__resolve_msaa(r);
                glBindFramebuffer(GL_FRAMEBUFFER, draw_fbo);
            }
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
    if (v.quads) pix__quad_pass(r, false);
    if (v.particles) pix__particle_pass(r);

    glDisable(GL_CULL_FACE);
    glDisable(GL_SCISSOR_TEST);
    if (msaa) pix__resolve_msaa(r);
    if (taa) pix__velocity_pass(r);
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
// A culled depth list drawn with each material's depth program, from `vp`.
static void pix__draw_depth_list(pix_renderer& r, pix__pass_state& st, const pix__draw_list& list, const mat4& vp) {
    if (!list.count) return;
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
                glUniformMatrix4fv(st.u->light_view_proj, 1, GL_FALSE, vp.data);
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

// Spot and point light shadows: each tile of the local atlas gets its own
// culled caster list - only what lies inside that light's frustum and reach.
static void pix__render_local_shadows(pix_renderer& r) {
    if (!r.local_tiles || !r.local_shadow.fbo) return;
    pix_bind_framebuffer(r.local_shadow);
    glDepthMask(GL_TRUE);
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.5f, 2.0f);
    int tile = r.local_shadow.width / 4;
    pix__pass_state st = { 0, 0, PIX_INVALID, 0, -1 };
    pix__draw_list& list = r.list_cascade[0];
    for (int t = 0; t < r.local_tiles; t++) {
        frustum fr = frustum_from_viewproj(r.local_tile_vp[t]);
        vec3 eye = r.local_tile_eye[t];
        float reach = r.local_tile_far[t];
        size_t count = 0;
        for (size_t i = 0; i < r.renderable_count; i++) {
            if (!(r.flags[i] & PIX__CASTER)) continue;
            vec3 c = v3(r.bounds[i].x, r.bounds[i].y, r.bounds[i].z);
            float rad = r.bounds[i].w;
            vec3 to = v3sub(c, eye);
            float near_side = reach + rad;    // |to| - rad > reach, without the square root
            if (v3dot(to, to) > near_side * near_side || !frustum_test_sphere(fr, c, rad)) continue;
            r.sort_keys[count] = pix__batch_key(r.instances[i]);
            r.sort_vals[count++] = (uint32_t)i;
        }
        memcpy(list.items, pix__radix_sort(r, count), count * sizeof(uint32_t));
        list.count = count;
        pix__stream_list(r, list);
        glViewport((t % 4) * tile, (t / 4) * tile, tile, tile);
        st.program = 0;
        pix__draw_depth_list(r, st, list, r.local_tile_vp[t]);
        if (r.animated_count && r.shadows.skinned) pix__draw_skinned(r, st, &r.local_tile_vp[t], &fr, 0xFFFFFFFFu);
    }
    list.count = 0;
    glDisable(GL_POLYGON_OFFSET_FILL);
    glBindVertexArray(0);
}

void pix_render_shadows(pix_renderer& r) {
    pix__prepare_frame(r);
    pix__render_local_shadows(r);
    if (!r.shadows.enabled || !r.shadow_map.fbo || r.shadows.strength <= 0.0f) return;
    pix__gpu_begin(r, PIX_GPU_SHADOW);

    size_t n = r.renderable_count;
    // a caster under cull_pixels texels across is a speck in that cascade's map
    float speck[SHADOW_CASCADES];
    for (int k = 0; k < SHADOW_CASCADES; k++) speck[k] = 0.5f * r.cull_pixels * r.cascade_texel[k];
    for (size_t i = 0; i < n; i++) {
        r.flags[i] &= (uint8_t)~0xF0;
        if (!(r.flags[i] & PIX__CASTER)) continue;
        vec3 c = v3(r.bounds[i].x, r.bounds[i].y, r.bounds[i].z);
        float rad = r.bounds[i].w;
        for (int k = 0; k < SHADOW_CASCADES; k++) {
            if (!(r.cascade_mask & (1 << k)) || rad < speck[k]) continue;
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
        memcpy(r.list_cascade[k].items, pix__radix_sort(r, count), count * sizeof(uint32_t));
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
        pix__draw_depth_list(r, st, r.list_cascade[c], r.light_space[c]);
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
// The HDR passes that run before the tonemap, each reading the image so far
// and the scene depth: motion blur, depth of field, volumetric light, and the
// luminance measurement auto exposure adapts to. Returns the image to tonemap.
static const framebuffer* pix__hdr_stages(pix_renderer& r, const framebuffer& src, bool* volumetric) {
    *volumetric = false;
    const framebuffer* cur = &src;
    if (!src.depth) return cur;
    framebuffer* next = &r.hdr_a;

    if (r.taa_jittered && src.fbo == r.scene_target.fbo && r.taa_history[0].fbo) {
        // TAA first: everything after it (blur, DoF, bloom) works on the clean image
        int into = r.taa_index ^ 1;
        pix_bind_framebuffer(r.taa_history[into]);
        GLuint p = (GLuint)r.taa_shader;
        glUseProgram(p);
        glUniform1i(glGetUniformLocation(p, "uScene"), 0);
        glUniform1i(glGetUniformLocation(p, "uHistory"), 1);
        glUniform1i(glGetUniformLocation(p, "uVelocity"), 2);
        glUniform1i(glGetUniformLocation(p, "uDepth"), 3);
        mat4 inv = mat4_inverse(r.taa_view_proj);
        glUniformMatrix4fv(glGetUniformLocation(p, "uInvViewProj"), 1, GL_FALSE, inv.data);
        glUniformMatrix4fv(glGetUniformLocation(p, "uPrevViewProj"), 1, GL_FALSE, r.taa_prev_view_proj.data);
        glUniform2f(glGetUniformLocation(p, "uTexel"), 1.0f / (float)r.width, 1.0f / (float)r.height);
        glUniform1f(glGetUniformLocation(p, "uFeedback"), clampf(r.aa.taa_feedback, 0.0f, 0.98f));
        glUniform1i(glGetUniformLocation(p, "uHasHistory"), r.taa_valid ? 1 : 0);
        glActiveTexture(GL_TEXTURE3); glBindTexture(GL_TEXTURE_2D, src.depth);
        glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, r.velocity_tex);
        glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, r.taa_history[r.taa_index].color);
        glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, src.color);
        pix_draw_fullscreen();
        r.taa_index = into;
        r.taa_valid = true;
        cur = &r.taa_history[into];
    }

    if (r.motion_blur.enabled && r.hdr_a.fbo) {
        pix_bind_framebuffer(*next);
        GLuint p = (GLuint)r.motion_blur_shader;
        glUseProgram(p);
        glUniform1i(glGetUniformLocation(p, "uScene"), 0);
        glUniform1i(glGetUniformLocation(p, "uDepth"), 1);
        int n = r.motion_blur.samples < 2 ? 2 : (r.motion_blur.samples > 32 ? 32 : r.motion_blur.samples);
        glUniform4f(glGetUniformLocation(p, "uParams"), r.motion_blur.strength, (float)n, r.motion_blur.max_length, 0.0f);
        glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, src.depth);
        glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, cur->color);
        pix_draw_fullscreen();
        cur = next;
        next = r.hdr_b.fbo ? &r.hdr_b : 0;
    }
    if (r.dof.enabled && r.dof_half.fbo && next) {
        pix_bind_framebuffer(r.dof_half);
        GLuint p = (GLuint)r.dof_blur_shader;
        glUseProgram(p);
        glUniform1i(glGetUniformLocation(p, "uScene"), 0);
        glUniform1i(glGetUniformLocation(p, "uDepth"), 1);
        glUniform4f(glGetUniformLocation(p, "uParams"), r.dof.focus_distance, r.dof.focus_range,
                    clampf(r.dof.max_blur, 1.0f, 32.0f), r.dof.auto_focus ? 1.0f : 0.0f);
        glUniform2f(glGetUniformLocation(p, "uTexel"), 1.0f / (float)r.width, 1.0f / (float)r.height);
        glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, src.depth);
        glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, cur->color);
        pix_draw_fullscreen();
        pix_bind_framebuffer(*next);
        GLuint c = (GLuint)r.dof_composite_shader;
        glUseProgram(c);
        glUniform1i(glGetUniformLocation(c, "uScene"), 0);
        glUniform1i(glGetUniformLocation(c, "uBlur"), 1);
        glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, r.dof_half.color);
        glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, cur->color);
        pix_draw_fullscreen();
        cur = next;
    }
    if (r.volumetrics.enabled && r.vol_a.fbo) {
        pix_bind_framebuffer(r.vol_a);
        GLuint p = (GLuint)r.volumetric_shader;
        glUseProgram(p);
        glUniform1i(glGetUniformLocation(p, "uDepth"), 0);
        glUniform1i(glGetUniformLocation(p, "uShadowMap"), 1);
        glUniform1i(glGetUniformLocation(p, "uLocalShadow"), 11);
        int steps = r.volumetrics.steps < 4 ? 4 : (r.volumetrics.steps > 128 ? 128 : r.volumetrics.steps);
        glUniform4f(glGetUniformLocation(p, "uParams"), r.volumetrics.density, clampf(r.volumetrics.anisotropy, -0.95f, 0.95f),
                    r.volumetrics.max_distance, (float)steps);
        glUniform4f(glGetUniformLocation(p, "uParams2"), r.volumetrics.intensity, r.volumetrics.lights ? 1.0f : 0.0f,
                    r.volumetrics.light_intensity, 0.0f);
        bool sh = r.shadows.enabled && r.shadow_map.fbo && r.shadows.strength > 0.0f;
        pix__bind_frame_textures(r, sh, false, false, false);
        glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, src.depth);
        pix_draw_fullscreen();
        GLuint b = (GLuint)r.bilateral_shader;
        glUseProgram(b);
        glUniform1i(glGetUniformLocation(b, "uSource"), 0);
        glUniform1i(glGetUniformLocation(b, "uDepth"), 1);
        GLint dir = glGetUniformLocation(b, "uDirection");
        glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, src.depth);
        glActiveTexture(GL_TEXTURE0);
        pix_bind_framebuffer(r.vol_b);
        glUniform2f(dir, 1.0f / (float)r.vol_a.width, 0.0f);
        glBindTexture(GL_TEXTURE_2D, r.vol_a.color);
        pix_draw_fullscreen();
        pix_bind_framebuffer(r.vol_a);
        glUniform2f(dir, 0.0f, 1.0f / (float)r.vol_a.height);
        glBindTexture(GL_TEXTURE_2D, r.vol_b.color);
        pix_draw_fullscreen();
        *volumetric = true;
    }
    if (r.post.auto_exposure && r.lum_tex) {
        glBindFramebuffer(GL_FRAMEBUFFER, r.lum_fbo);
        glViewport(0, 0, 256, 256);
        glUseProgram(r.luminance_shader);
        glUniform1i(glGetUniformLocation(r.luminance_shader, "uScene"), 0);
        glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, cur->color);
        pix_draw_fullscreen();
        glBindTexture(GL_TEXTURE_2D, r.lum_tex);
        glGenerateMipmap(GL_TEXTURE_2D);
        float dt = r.time - r.last_time;
        if (dt <= 0.0f || dt > 0.5f) dt = 1.0f / 60.0f;
        r.last_time = r.time;
        int to = r.adapt_index ^ 1;
        glBindFramebuffer(GL_FRAMEBUFFER, r.adapt_fbo[to]);
        glViewport(0, 0, 1, 1);
        GLuint a = (GLuint)r.adapt_shader;
        glUseProgram(a);
        glUniform1i(glGetUniformLocation(a, "uLuminance"), 0);
        glUniform1i(glGetUniformLocation(a, "uPrevious"), 1);
        glUniform4f(glGetUniformLocation(a, "uParams"), 8.0f, 1.0f - expf(-dt * r.post.exposure_speed),
                    r.adapt_fresh ? 1.0f : 0.0f, 0.0f);
        glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, r.adapt_tex[r.adapt_index]);
        glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, r.lum_tex);
        pix_draw_fullscreen();
        r.adapt_index = to;
        r.adapt_fresh = false;
    }
    return cur;
}

void pix_post_process(pix_renderer& r, const framebuffer& src_in, const framebuffer* dest) {
    pix__prepare_frame(r);
    pix__gpu_begin(r, PIX_GPU_POST);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glDisable(GL_SCISSOR_TEST);
    const pix_post_settings& p = r.post;
    bool full = src_in.width == r.width && src_in.height == r.height;
    bool volumetric = false;
    const framebuffer& src = full ? *pix__hdr_stages(r, src_in, &volumetric) : src_in;
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
    {
        // outlines, eye adaptation, grading cube, grain, light shafts
        const pix_outline_settings& o = r.outline;
        bool outline = full && o.enabled && src_in.depth;
        vec3 oc = pix__lin(o.color);
        glUniform4f(r.u_post[12], oc.x, oc.y, oc.z, outline ? 1.0f : 0.0f);
        glUniform4f(r.u_post[13], o.thickness, o.depth_threshold > 1e-4f ? o.depth_threshold : 1e-4f,
                    o.normal_threshold, o.fade_distance);
        glUniform4f(r.u_post[14], o.object_edges ? 1.0f : 0.0f, r.prepass_done ? 1.0f : 0.0f, o.groups_only ? 1.0f : 0.0f, 0.0f);
        bool ae = full && p.auto_exposure && r.lum_tex;
        glUniform4f(r.u_post[15], ae ? 1.0f : 0.0f, p.exposure_key, p.exposure_min, p.exposure_max);
        glUniform4f(r.u_post[16], p.lut ? p.lut_strength : 0.0f, 0.0f, p.grain, volumetric ? 1.0f : 0.0f);
        glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, src_in.depth ? src_in.depth : r.white_texture);
        glActiveTexture(GL_TEXTURE3); glBindTexture(GL_TEXTURE_2D, r.prepass_done && r.normal_tex ? r.normal_tex : r.white_texture);
        glActiveTexture(GL_TEXTURE0 + 4); glBindTexture(GL_TEXTURE_2D, ae ? r.adapt_tex[r.adapt_index] : r.white_texture);
        glActiveTexture(GL_TEXTURE0 + 5); glBindTexture(GL_TEXTURE_3D, p.lut);
        glActiveTexture(GL_TEXTURE0 + 6); glBindTexture(GL_TEXTURE_2D, volumetric ? r.vol_a.color : r.white_texture);
    }
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
        if (e.u_normal >= 0) {
            glUniform1i(e.u_normal, 2);
            glActiveTexture(GL_TEXTURE2);
            glBindTexture(GL_TEXTURE_2D, r.prepass_done && r.normal_tex ? r.normal_tex : r.white_texture);
        }
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, src_in.depth ? src_in.depth : r.white_texture);
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
    r.prev_view_proj = r.frame_view_proj;     // motion blur's "where it was"
    if (r.taa_jittered) r.taa_prev_view_proj = r.taa_view_proj;
    r.taa_jittered = false;
    r.frame_number++;
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
        r.quad_count = 0;
        r.line_count = 0;
    }
    r.frame_prepared = false;
}

// ---------------- HDR environment ----------------

#ifndef GL_RGB32F
#define GL_RGB32F 0x8815
#endif

// Projects the image onto order-2 spherical harmonics, then applies the
// cosine-lobe convolution (A0 = pi, A1 = 2pi/3, A2 = pi/4) and divides by pi,
// so evaluating them gives the light to multiply albedo by.
void pix_environment_sh(const float* rgb, int w, int h, vec3 out[9]) {
    double acc[9][3] = {};
    for (int y = 0; y < h; y++) {
        float theta = ((float)y + 0.5f) / (float)h * PIX_PI;   // 0 = straight up
        float st = sinf(theta), ct = cosf(theta);
        float d_omega = (2.0f * PIX_PI / (float)w) * (PIX_PI / (float)h) * st;
        for (int x = 0; x < w; x++) {
            float phi = (((float)x + 0.5f) / (float)w - 0.5f) * 2.0f * PIX_PI;
            float dx = st * cosf(phi), dy = ct, dz = st * sinf(phi);   // matches the cube conversion
            float basis[9] = { 0.282095f, 0.488603f * dy, 0.488603f * dz, 0.488603f * dx,
                               1.092548f * dx * dy, 1.092548f * dy * dz, 0.315392f * (3.0f * dz * dz - 1.0f),
                               1.092548f * dx * dz, 0.546274f * (dx * dx - dy * dy) };
            const float* p = rgb + ((size_t)y * w + x) * 3;
            for (int k = 0; k < 9; k++)
                for (int c = 0; c < 3; c++) acc[k][c] += (double)(p[c] * basis[k] * d_omega);
        }
    }
    const float band[9] = { 1.0f, 2.0f / 3.0f, 2.0f / 3.0f, 2.0f / 3.0f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f };   // A_l / pi
    for (int k = 0; k < 9; k++)
        out[k] = v3((float)acc[k][0] * band[k], (float)acc[k][1] * band[k], (float)acc[k][2] * band[k]);
}

// sh_irradiance on the CPU
static vec3 pix__sh_eval(const vec3 sh[9], vec3 n) {
    float b[9] = { 0.282095f, 0.488603f * n.y, 0.488603f * n.z, 0.488603f * n.x, 1.092548f * n.x * n.y,
                   1.092548f * n.y * n.z, 0.315392f * (3.0f * n.z * n.z - 1.0f), 1.092548f * n.x * n.z,
                   0.546274f * (n.x * n.x - n.y * n.y) };
    vec3 s = v3(0, 0, 0);
    for (int k = 0; k < 9; k++) s = s + sh[k] * b[k];
    return v3(maxf(s.x, 0.0f), maxf(s.y, 0.0f), maxf(s.z, 0.0f));
}

// face-local [-1, 1] coordinates to the direction GL samples there (the cube map table)
static const char* const GLSL_CUBE_FACE = R"(
uniform int uFace;
vec3 face_dir(vec2 uv) {
    vec2 c = uv * 2.0 - 1.0;
    if (uFace == 0) return normalize(vec3( 1.0, -c.y, -c.x));
    if (uFace == 1) return normalize(vec3(-1.0, -c.y,  c.x));
    if (uFace == 2) return normalize(vec3( c.x,  1.0,  c.y));
    if (uFace == 3) return normalize(vec3( c.x, -1.0, -c.y));
    if (uFace == 4) return normalize(vec3( c.x, -c.y,  1.0));
    return normalize(vec3(-c.x, -c.y, -1.0));
}
)";

static const char* const FSHDER_EQUIRECT_TO_CUBE = R"(#version 440 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uEquirect;
void main() {
    vec3 d = face_dir(vUV);
    vec2 uv = vec2(atan(d.z, d.x) / 6.28318530718 + 0.5, acos(clamp(d.y, -1.0, 1.0)) / 3.14159265359);
    frag = vec4(textureLod(uEquirect, uv, 0.0).rgb, 1.0);
}
)";

// Split-sum prefilter (Karis 2013): GGX importance samples around the
// direction, with N = V = R. Each sample reads the source mip whose texel
// matches the sample's solid angle (Krivanek & Colbert 2008), so a few dozen
// samples give a clean result instead of fireflies from the sun.
static const char* const FSHDER_PREFILTER_GGX = R"(#version 440 core
in vec2 vUV;
out vec4 frag;
uniform samplerCube uSource;
uniform float uRough;
uniform float uSourceSize;
const float PI = 3.14159265359;
vec2 hammersley(uint i, uint n) {
    uint b = i;
    b = (b << 16u) | (b >> 16u);
    b = ((b & 0x55555555u) << 1u) | ((b & 0xAAAAAAAAu) >> 1u);
    b = ((b & 0x33333333u) << 2u) | ((b & 0xCCCCCCCCu) >> 2u);
    b = ((b & 0x0F0F0F0Fu) << 4u) | ((b & 0xF0F0F0F0u) >> 4u);
    b = ((b & 0x00FF00FFu) << 8u) | ((b & 0xFF00FF00u) >> 8u);
    return vec2(float(i) / float(n), float(b) * 2.3283064365386963e-10);
}
void main() {
    vec3 n = face_dir(vUV);
    if (uRough <= 0.0) { frag = vec4(textureLod(uSource, n, 0.0).rgb, 1.0); return; }
    vec3 up = abs(n.y) < 0.999 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    vec3 tx = normalize(cross(up, n)), ty = cross(n, tx);
    float a = uRough * uRough, a2 = a * a;
    float texel = 4.0 * PI / (6.0 * uSourceSize * uSourceSize);
    const uint COUNT = 128u;
    vec3 sum = vec3(0.0);
    float weight = 0.0;
    for (uint i = 0u; i < COUNT; i++) {
        vec2 xi = hammersley(i, COUNT);
        float phi = 2.0 * PI * xi.x;
        float cos_t = sqrt((1.0 - xi.y) / (1.0 + (a2 - 1.0) * xi.y));
        float sin_t = sqrt(1.0 - cos_t * cos_t);
        vec3 h = tx * (cos(phi) * sin_t) + ty * (sin(phi) * sin_t) + n * cos_t;
        vec3 l = 2.0 * dot(n, h) * h - n;
        float ndl = dot(n, l);
        if (ndl <= 0.0) continue;
        float d = (cos_t * cos_t * (a2 - 1.0) + 1.0);
        float pdf = a2 / (PI * d * d) * 0.25;            // D * NoH / (4 VoH), with N = V
        float omega = 1.0 / (float(COUNT) * pdf + 1e-6);
        float mip = max(0.5 * log2(omega / texel) + 1.0, 0.0);
        sum += textureLod(uSource, l, mip).rgb * ndl;
        weight += ndl;
    }
    frag = vec4(sum / max(weight, 1e-4), 1.0);
}
)";

static GLuint pix__make_cube(int size, int levels) {
    GLuint t = 0;
    glGenTextures(1, &t);
    glBindTexture(GL_TEXTURE_CUBE_MAP, t);
    for (int m = 0; m < levels; m++) {
        int s = size >> m;
        for (int f = 0; f < 6; f++)
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + f, m, GL_RGBA16F, s, s, 0, GL_RGBA, GL_FLOAT, 0);
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAX_LEVEL, levels - 1);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, levels > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    return t;
}

bool pix_set_environment(pix_renderer& r, const float* rgb, int w, int h, float intensity) {
    if (!rgb || w < 4 || h < 2) return false;
    auto& e = r.environment;
    if (!e.convert_shader) {
        const char* chunks[] = { GLSL_CUBE_FACE };
        char* a = pix_shader_splice(FSHDER_EQUIRECT_TO_CUBE, chunks, 1);
        char* b = pix_shader_splice(FSHDER_PREFILTER_GGX, chunks, 1);
        e.convert_shader = a ? opengl_create_shader(VSHDER_POST, a) : 0;
        e.prefilter_shader = b ? opengl_create_shader(VSHDER_POST, b) : 0;
        pix_free(a);
        pix_free(b);
        if (!e.convert_shader || !e.prefilter_shader) return false;
    }
    pix_environment_sh(rgb, w, h, e.sh);
    e.intensity = intensity;

    // a cube with about the image's detail: a quarter of its width per face, capped
    int size = 32;
    while (size * 2 <= w / 4 && size < 512) size *= 2;
    int levels = 1;
    while ((size >> levels) >= 8) levels++;   // down to 8 x 8: roughness 1 needs no less

    GLuint equirect = 0;
    glGenTextures(1, &equirect);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, equirect);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, w, h, 0, GL_RGB, GL_FLOAT, rgb);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    int full = 1;
    while ((size >> full) >= 1) full++;
    GLuint source = pix__make_cube(size, full);       // the sharp sky, every mip (for the prefilter)
    GLuint cube = pix__make_cube(size, levels);       // the result
    GLuint fbo = 0;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_SCISSOR_TEST);

    glUseProgram(e.convert_shader);
    GLint face_u = glGetUniformLocation(e.convert_shader, "uFace");
    glViewport(0, 0, size, size);
    for (int f = 0; f < 6; f++) {
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + f, source, 0);
        glUniform1i(face_u, f);
        pix_draw_fullscreen();
    }
    glBindTexture(GL_TEXTURE_CUBE_MAP, source);
    glGenerateMipmap(GL_TEXTURE_CUBE_MAP);

    glUseProgram(e.prefilter_shader);
    face_u = glGetUniformLocation(e.prefilter_shader, "uFace");
    GLint rough_u = glGetUniformLocation(e.prefilter_shader, "uRough");
    glUniform1f(glGetUniformLocation(e.prefilter_shader, "uSourceSize"), (float)size);
    for (int m = 0; m < levels; m++) {
        glViewport(0, 0, size >> m, size >> m);
        glUniform1f(rough_u, levels > 1 ? (float)m / (float)(levels - 1) : 0.0f);
        for (int f = 0; f < 6; f++) {
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_CUBE_MAP_POSITIVE_X + f, cube, m);
            glUniform1i(face_u, f);
            pix_draw_fullscreen();
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fbo);
    glDeleteTextures(1, &source);
    glDeleteTextures(1, &equirect);
    glEnable(GL_DEPTH_TEST);

    opengl_destroy_texture(e.cube);
    e.cube = cube;
    e.size = size;
    e.max_mip = levels - 1;
    // toon shading still reads the hemisphere colours: give it the environment's, from above and below
    r.sky.diffuse_up = pix__sh_eval(e.sh, v3(0, 1, 0)) * intensity;
    r.sky.diffuse_down = pix__sh_eval(e.sh, v3(0, -1, 0)) * intensity;
    return true;
}

bool pix_load_environment(pix_renderer& r, const char* path, float intensity) {
    int w = 0, h = 0;
    float* rgb = hdr_load_file(path, &w, &h);
    if (!rgb) { pix_log("pix: could not load environment '%s' (Radiance .hdr expected)", path); return false; }
    bool ok = pix_set_environment(r, rgb, w, h, intensity);
    pix_free(rgb);
    return ok;
}

void pix_taa_reset(pix_renderer& r) { r.taa_valid = false; }

void pix_clear_environment(pix_renderer& r) {
    opengl_destroy_texture(r.environment.cube);
    r.environment.cube = 0;
}

idx pix_load_color_lut(const char* path) {
    int w = 0, h = 0;
    unsigned char* px = pix_load_image_rgba(path, &w, &h);
    if (!px) { pix_log("pix: could not load colour LUT '%s'", path); return 0; }
    int n = h;
    if (n < 2 || w != n * n) {
        pix_log("pix: colour LUT '%s' should be %d slices of %dx%d side by side (got %dx%d)", path, n, n, n, w, h);
        pix_free(px);
        return 0;
    }
    // slice b, row g, column r -> cube texel (r, g, b)
    unsigned char* cube = pix_alloc<unsigned char>((size_t)n * n * n * 4);
    for (int b = 0; b < n; b++)
        for (int g = 0; g < n; g++)
            memcpy(cube + ((size_t)(b * n + g) * n) * 4, px + ((size_t)g * w + (size_t)b * n) * 4, (size_t)n * 4);
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_3D, tex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, n, n, n, 0, GL_RGBA, GL_UNSIGNED_BYTE, cube);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_3D, 0);
    pix_free(cube);
    pix_free(px);
    return tex;
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
