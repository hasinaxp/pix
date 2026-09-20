#pragma once
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include "dtype.hpp"
#include "math.hpp"
#include "../loader/data_loader.hpp"
#include "../loader/png_loader.hpp"
#include "opengl_utils.hpp"
#include "lighting.hpp"
#include "framebuffer.hpp"
#include "shader_sources.hpp"
#include "animation.hpp"

#define MAX_RENDERABLE 24576           // opaque instances that survive culling in one frame
// How many full instance lists the stream buffer has to hold at once: the
// opaque list, one compaction per shadow cascade, and one for the reflection,
// plus water and slack. Only the size of an allocation - overrunning it costs
// a wrapped cursor and one stall, not a wrong picture.
// Instances are sorted near-to-far within each batch, in steps this wide. The
// step is what a "draw everything within N metres" pass rounds its cutoff up
// to, so a coarse one costs a few instances the pass did not need and a fine
// one costs nothing but sort buckets - 64 of these covers half a kilometre,
// well past any view distance the renderer is asked for.
#define PIX_DIST_BUCKET   8.0f
#define PIX_DIST_BUCKETS  64
#define MAX_ANIMATED   96              // animated instances drawn per frame
#define MAX_MESHES     512
// Water surface tiles drawn in one frame. Water is tiled rather than drawn as
// one huge quad so that it can be culled and so the wave shader keeps its
// detail near the camera; a map that is mostly open water reaches several
// thousand tiles inside a normal view distance, so this is sized for that
// rather than for a river running through dry land.
#define MAX_WATER_INSTANCES 12288
// Particles. One quad each, all of them in one streamed buffer and one draw -
// see pix__particle_pass. A cap rather than a pool: past a few thousand blobs
// on screen nothing is legible anyway, so the oldest simply stop being drawn.
#define MAX_PARTICLES  4096

// Materials. One per distinct texture, plus one per tint or finish variant
// that clone_material makes over a shared texture. An asset pack that ships a
// real texture per model burns a slot per model, where a kit sharing one atlas
// over hundreds of pieces burns a single slot - so this is sized for the
// former. load_material and load_material_image bounds-check against it,
// because overrunning `materials[]` corrupts whatever the compiler placed
// after it and fails several frames later somewhere else entirely.
#define MAX_MATERIALS  768
#define MAX_SHADERS    32
// Shared vertex/index buffer capacity. Every static mesh in the world lives in
// these two buffers, which is what lets a frame be a handful of instanced draws
// rather than a bind per model - so this is not a per-model budget, it is the
// whole catalogue's, and running out of it is silent from the outside: the
// models loaded after the pool fills simply do not exist. Anything loading a
// large catalogue should report how full it is for that reason; at 32 bytes a
// vertex, two million of them is a 64 MB buffer.
#define MESH_POOL_VERTICES (1 << 21)   // shared vertex buffer capacity
#define MESH_POOL_INDICES  (1 << 21)   // shared index buffer capacity
// Shared skinned vertex buffer. The old note here said uint16 indices capped
// this at 65536 anyway - they do not: every skinned draw goes through
// glDrawElementsInstancedBaseVertex with the mesh's own vertex_offset as the
// base, so the 16-bit range limits one *mesh* to 65535 vertices and says
// nothing about the pool. At 65536 the pool filled after five character rigs
// and the sixth, seventh and eighth were silently dropped - which is most of
// what "half the cast is missing" was.
#define SKIN_POOL_VERTICES (1 << 18)
#define SKIN_POOL_INDICES  (1 << 19)

struct mesh {
    idx vbo;
    idx vertex_offset;   // first vertex in the shared buffer
    idx vertex_size;     // stride, bytes
    idx vertex_count;
    idx index_offset;    // first index in the shared buffer
    idx index_count;
};

struct material {
    idx shader;
    idx texture;
    vec3 color;
    float metallic;
    float roughness;
    // Linear radiance the surface gives off by itself. Zero for almost
    // everything; it is what a brake light or a lamp head is, and it is the
    // channel the bloom pass exists to serve.
    vec3 emissive;
    // How much of this material is glass, found the same way the window glow
    // below is: by how dark the texture is.
    //
    // Art sets commonly paint a model's windows straight into its one texture,
    // leaving no second material to give a different finish to - and glazing
    // rendered at the same roughness as the wall around it is the single thing
    // that most says "untextured box". The dark texels are the glazing; this
    // says how far toward a mirror they go.
    float glass;
    // Emission weighted toward the *dark* parts of the texture.
    //
    // Where a set shares one atlas and its windows are simply its darkest
    // swatches, there is no second texture to say where the glass is - but
    // there does not need to be. Weighting the glow by how dark the albedo is
    // lights the windows and leaves the walls alone, which after dusk is the
    // entire difference between a lived-in skyline and a row of black boxes.
    vec3 window_glow;
};

struct pix_render_instance {
    idx mesh;
    idx material;
    mat4 transform;
    // How far this instance is from the camera, in PIX_DIST_BUCKET metre
    // steps, filled in by the sort. It is what lets the shadow and reflection
    // passes take a near slice of a batch without compacting one - see
    // pix__sort_instances.
    uint32_t dist_bucket;
};

// What one particle is, once it has stopped being simulation and become
// geometry: a point, a size and a colour. Laid out to be uploaded as-is.
struct pix_particle_vertex {
    float centre[3];
    float size;        // half-width in metres
    float color[4];    // rgb linear radiance, a coverage
};

struct camera {
    vec3 position;
    vec3 direction;
    vec3 up;
};

// The CPU side of the two uniform blocks the shaders declare (GLSL_SCENE and
// GLSL_LIGHTS). Field for field, in order, all of it vec4 or mat4 so std140
// needs no padding rules applied by hand - the one way this kind of thing goes
// wrong is the two sides drifting apart, and the only defence is that they are
// trivially comparable.
struct pix_scene_block {
    float sun_dir[4];
    float sun_color[4];
    float sky_color[4];
    float ground_color[4];
    float sky_zenith[4];
    float sky_horizon[4];
    float fog[4];             // rgb, w density
    float camera[4];          // xyz eye, w wetness
    float shadow[4];          // xy atlas texel, z strength
    float light_space[SHADOW_CASCADES][16];
    float cascade_far[4];
    float cascade_texel[4];
};

struct pix_light_block {
    float pos_radius[MAX_SHADER_LIGHTS * 4];
    float color_inner[MAX_SHADER_LIGHTS * 4];
    float dir_outer[MAX_SHADER_LIGHTS * 4];
    float meta[4];            // x: live light count
};

// The passes end_frame is made of, in the order it runs them. Anything worth
// timing separately is worth being able to switch off separately, which is why
// this enum is also what pix_set_pass_enabled indexes.
enum {
    PIX_GPU_SHADOW,
    PIX_GPU_REFLECTION,
    PIX_GPU_SKY,
    PIX_GPU_OPAQUE,
    PIX_GPU_SKINNED,
    PIX_GPU_WATER,
    PIX_GPU_PARTICLES,
    PIX_GPU_BLOOM,
    PIX_GPU_POST,
    PIX_GPU_PASSES
};
static const char* const PIX_GPU_PASS_NAMES[PIX_GPU_PASSES] = {
    "shadow", "reflect", "sky", "opaque", "skinned", "water", "particles", "bloom", "post"
};

struct pix_renderer {
    int width;
    int height;
    vec3 clear_color;

    pix_render_instance instances[MAX_RENDERABLE];
    mesh     meshes[MAX_MESHES];
    material materials[MAX_MATERIALS];
    idx      shaders[MAX_SHADERS];
    idx      shader;

    // animated pass: each instance carries the pose it should be drawn with
    pix_render_instance animated_instances[MAX_ANIMATED];
    const animation*    animated_poses[MAX_ANIMATED];
    mesh   skinned_meshes[MAX_MESHES];
    size_t skinned_mesh_count;
    size_t animated_count;
    idx    skinned_shader;

    size_t mesh_count;
    size_t material_count;
    size_t shader_count;
    size_t renderable_count;

    idx vertex_cursor;  // bump allocator into the shared buffers
    idx index_cursor;
    idx skin_vertex_cursor;
    idx skin_index_cursor;

    mat4 view_matrix;
    mat4 projection_matrix;
    vec3 camera_dir;    // forward, as handed to begin_frame - the reflection pass mirrors it
    vec3 camera_up;

    idx vao;
    idx models_vbo;      // every mesh's vertices
    idx instances_vbo;   // per-frame transforms - see pix__instance_upload
    idx ebo;             // every mesh's indices

    // ---- the instance stream ----
    //
    // Every draw in a frame reads its transforms out of one buffer, and the
    // rule that makes that cheap is that no two writes in a frame touch the
    // same bytes. Uploading each batch over the top of the last one is the
    // obvious way to do it and the expensive one: the driver cannot start the
    // upload until the draw that is still reading those bytes has finished, so
    // a frame of a few hundred batches becomes a few hundred pipeline stalls -
    // which is exactly what it was doing, and what put eighteen milliseconds
    // of a twenty-one millisecond frame inside end_frame on a GPU that only
    // had five milliseconds of work in it.
    //
    // Instead the buffer is a bump allocator: each upload takes fresh space,
    // the draw names its own start with baseInstance, and the whole thing is
    // orphaned once at the top of the frame (glBufferData with a null pointer,
    // which tells the driver to hand back new storage rather than wait for the
    // old one to be free). Nothing in a frame ever waits for anything.
    size_t instance_cursor;    // in instances, not bytes
    size_t instance_capacity;
    // Where this frame's sorted opaque transforms landed. The shadow pass, the
    // reflection pass and the colour pass all draw the same list, so it goes
    // up once and all three read it.
    size_t opaque_base;
    bool   opaque_uploaded;

    idx vao_animated;
    idx models_vbo_animated;  // every skinned mesh's vertices
    idx ebo_animated;

    idx white_texture;   // stand-in for untextured materials
    mem_arena tex_arena; // scratch for image decoding

    // ---- lighting, shadows and post processing ----
    // Off until pix_enable_effects is called, in which case end_frame draws
    // straight to the back buffer exactly as it always did.
    bool        effects;
    framebuffer scene_target;   // HDR colour the post pass reads
    framebuffer shadow_map;     // depth only, from the sun
    idx         depth_shader;
    idx         depth_skinned_shader;
    idx         post_shader;
    idx         water_shader;
    idx         sky_shader;    // analytic sky drawn behind the world

    // bloom: quarter resolution, ping-ponged between two targets
    framebuffer bloom_a, bloom_b;
    idx         bloom_prefilter_shader;
    idx         bloom_blur_shader;
    float       bloom_threshold;
    float       bloom_knee;
    float       bloom_strength;

    // The sun and the sky are described by lighting.hpp and mirrored into the
    // flat fields the shader uniforms are fed from, so a caller can set an hour
    // of the day and everything downstream follows.
    pix_sun sun;
    pix_sky sky;
    pix_weather weather;   // the cloud deck the hour is then filtered through
    float   hour;          // time of day, 0..24
    float   night;         // 0 by day, 1 after dusk; what artificial light rides on

    pix_light_list lights;

    // Per-frame uniform blocks, bound once and shared by every program: the
    // scene at binding 0, the light list at binding 1. See GLSL_SCENE.
    GLuint scene_ubo;
    GLuint lights_ubo;

    float shadow_strength; // 0 turns shadows off without unbinding anything
    float shadow_extent;   // half width of the furthest cascade, metres
    float shadow_depth;    // how far along the sun direction the box reaches
    float exposure;
    float saturation;
    float vignette;
    float sharpen;
    // The grade, applied after the tonemap - see `grade` in FSHDER_POST.
    // contrast is how hard the midtone curve bites, split_tone how far the
    // shadows go blue and the highlights warm.
    float contrast;
    float split_tone;

    // one shadow projection per cascade, near to far, plus where each ends
    mat4  light_space[SHADOW_CASCADES];
    float cascade_far[SHADOW_CASCADES];
    float cascade_texel[SHADOW_CASCADES];   // one shadow texel in world metres
    vec3 camera_position;

    // Water is drawn after the opaque pass, blended, from its own list. It has
    // to be big enough for every water cell that can be on screen at once: a
    // river six cells wide running the length of a VIEW_DISTANCE draw is well
    // over a thousand tiles, and a queue that silently stops accepting them
    // ends the river in a straight line across the middle of the view.
    pix_render_instance water_instances[MAX_WATER_INSTANCES];
    size_t water_count;
    vec3   water_shallow;
    vec3   water_deep;
    float  water_depth;    // metres from the surface to the bed, for the murk term
    float  time;

    // ---- particles ----
    // Submitted per frame like everything else and drawn after the water, so
    // smoke sits in front of a river and a muzzle flash lights up over one.
    pix_particle_vertex particles[MAX_PARTICLES];
    size_t particle_count;
    idx    particle_vao;
    idx    particle_vbo;
    idx    particle_shader;

    // ---- planar water reflection ----
    // The opaque scene and sky, rendered again from a camera mirrored across
    // the water's surface, half resolution, so the water shader can sample the
    // real scene out of a texture instead of only the analytic sky.
    // See pix__reflection_pass.
    // ---- what the GPU spends the frame on ----
    //
    // The CPU-side profiler cannot see any of this: every GL call in end_frame
    // returns long before the work it queued has run, so a frame that is
    // really being held up by the shadow pass shows up on the CPU as time
    // spent in the buffer swap and nowhere else. These are GL_TIME_ELAPSED
    // queries around each pass, read a frame late so nothing ever blocks
    // waiting for a result, and filled in only while gpu_timing is on.
    // Draw calls and instances issued this frame, across every pass. A frame
    // that is slow on the CPU with an idle GPU is almost always a count
    // problem, and this is the number that says so.
    int    frame_draws;
    int    frame_instances;

    bool   gpu_timing;
    GLuint gpu_query[PIX_GPU_PASSES][2];   // one pair per pass, written and read alternately
    int    gpu_slot;                       // which of the two this frame writes
    double gpu_ms[PIX_GPU_PASSES];         // last completed reading, milliseconds
    // The same brackets on the CPU clock. The two together are the question
    // worth asking of a pass - a pass whose CPU time is far above its GPU time
    // is one where the driver is being asked to do too much per draw, and no
    // amount of shader work will touch it.
    double cpu_ms[PIX_GPU_PASSES];
    long long cpu_begin;

    framebuffer reflection_target;
    float       water_plane_y;       // world height reflections mirror across, from this frame's water
    bool        reflection_rendered; // true once this frame's pass has actually drawn into it
    mat4        reflection_view_proj; // the mirrored camera's matrix, for projecting a water
                                       // fragment's world position into reflection_target's UV space
};


static pix_renderer pix_create_renderer(
    int width, int height, idx shader, vec3 clear_color = { 0.0f, 0.5f, 0.8f });
static void pix_destory_renderer(pix_renderer& renderer);

// Sets the sun, the sky, the fog and how much artificial light the world should
// be running, all from one hour of the day. See lighting.hpp for the keyframes.
static void pix_set_time_of_day(pix_renderer& renderer, float hour);

// Sets the cloud deck, the rain and how wet the ground is, and re-derives the
// lighting from the current hour through it. See pix_weather_apply.
static void pix_set_weather(pix_renderer& renderer, const pix_weather& weather);

// Lights are submitted per frame the way draw calls are: cleared in
// begin_frame, added by whoever owns them, and reduced to the nearest few in
// end_frame. A light that stops being submitted stops existing, which is what
// makes a row of lamps that fade up at dusk one line of gameplay code rather
// than a resource to manage.
static void push_light(pix_renderer& renderer, const pix_light& light);

static idx load_mesh(pix_renderer& renderer, mesh_file_data& mesh_data);
static idx load_skinned_mesh(pix_renderer& renderer, skinned_mesh_file_data& mesh_data);
static idx load_material(pix_renderer& renderer,
    const char* texture_path = nullptr, vec3 color = { 1.0f, 1.0f, 1.0f },
    float metalic = 0.1f, float roughness = 0.5f);
// same, for a texture already decoded in memory (glTF embeds its textures).
// `pixelated` is for generated colour palettes, where a linear filter would
// blend one material's cell into the next.
static idx load_material_image(pix_renderer& renderer, image_file_data* image,
    vec3 color = { 1.0f, 1.0f, 1.0f }, float metalic = 0.1f, float roughness = 0.5f,
    bool pixelated = false);
// A second material over the same GL texture, differing only in tint and
// finish. Per-instance colour variation - outfit tints, painted-on shades -
// wants this rather than load_material, which would decode and upload the
// same image all over again.
static idx clone_material(pix_renderer& renderer, idx source, vec3 color,
                          float metallic, float roughness);
static idx get_default_material(pix_renderer& renderer);
static idx load_shader(pix_renderer& renderer, const char* vsrc, const char* fsrc);

// Allocates the scene target and shadow map and switches end_frame over to the
// three pass path. Safe to skip entirely; everything below degrades to the
// plain forward draw when it has not been called.
static void pix_enable_effects(pix_renderer& renderer, int shadow_resolution = 2048);
static void pix_set_time(pix_renderer& renderer, float seconds);
// queued separately from push_instance: water is transparent, so it has to be
// drawn after everything opaque and with its own shader
static void push_water(pix_renderer& renderer, idx mesh, idx material, const mat4& transform);
// `size` is a half width in metres and `color` is linear radiance that is
// deliberately not clamped - see VSHDER_PARTICLE.
static void push_particle(pix_renderer& renderer, vec3 at, float size, vec3 color, float alpha);

static void begin_frame(pix_renderer& renderer, camera& cam);
static void push_instance(pix_renderer& renderer, pix_render_instance& instance);
static void push_instance(pix_renderer& renderer, idx mesh, idx material, const mat4& transform);
// `pose` must stay alive until end_frame; it is referenced, not copied
static void push_animated_instance(pix_renderer& renderer, pix_render_instance& instance,
    const animation& pose);
static void end_frame(pix_renderer& renderer, bool clear_instances = true);

// GL_TIME_ELAPSED around each pass. Off by default: the queries themselves are
// cheap to keep, but the driver has to fence around them, and a measurement
// that changes what it measures is only worth taking while somebody is reading
// it.
static void pix_enable_gpu_timing(pix_renderer& renderer, bool on);
// Milliseconds the named pass took, from the last frame whose result is back.
static double pix_gpu_pass_ms(const pix_renderer& renderer, int pass);


// ------------------- implementation ---------------------

static void pix_set_time_of_day(pix_renderer& r, float hour) {
    // Wrapped here rather than only inside the lookup, so winding the clock
    // backwards past midnight leaves a clock reading of 23:00 and not -1:00.
    hour = fmodf(hour, 24.0f);
    if (hour < 0.0f) hour += 24.0f;
    r.hour = hour;
    pix_daylight_at(hour, &r.sun, &r.sky, &r.exposure);
    pix_weather_apply(r.weather, &r.sun, &r.sky, &r.exposure);
    // Deliberately off the *clear* sun elevation, which pix_weather_apply does
    // not move: a dark enough overcast would otherwise switch every lamp in the
    // world on at noon.
    r.night = pix_night_factor(r.sun);
}

static void pix_set_weather(pix_renderer& r, const pix_weather& w) {
    r.weather = w;
    pix_set_time_of_day(r, r.hour);      // re-derive the hour through the new sky
}

static void push_light(pix_renderer& r, const pix_light& light) {
    pix_lights_add(r.lights, light);
}

static pix_renderer pix_create_renderer(int width, int height, idx shader, vec3 clear_color) {
    pix_renderer r = {};
    r.width = width;
    r.height = height;
    r.shader = shader;
    r.clear_color = clear_color;
    r.tex_arena.capacity = 32 * MB;
    r.tex_arena.data = (char*)malloc(r.tex_arena.capacity);

    glGenVertexArrays(1, &r.vao);
    glBindVertexArray(r.vao);

    glGenBuffers(1, &r.models_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, r.models_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(MESH_POOL_VERTICES * sizeof(vertex)), 0, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, position));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, normal));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)offsetof(vertex, uv));
    glEnableVertexAttribArray(2);

    glGenBuffers(1, &r.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(MESH_POOL_INDICES * sizeof(uint16_t)), 0, GL_STATIC_DRAW);

    glGenBuffers(1, &r.instances_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, r.instances_vbo);
    // Exactly what one frame writes: the opaque list once, and the water once.
    // Every other pass reads the opaque upload rather than making its own.
    r.instance_capacity = MAX_RENDERABLE + MAX_WATER_INSTANCES;
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(r.instance_capacity * sizeof(mat4)), 0, GL_STREAM_DRAW);
    for (int c = 0; c < 4; c++) {
        glVertexAttribPointer(3 + c, 4, GL_FLOAT, GL_FALSE, sizeof(mat4), (void*)(size_t)(c * 4 * sizeof(float)));
        glEnableVertexAttribArray(3 + c);
        glVertexAttribDivisor(3 + c, 1);
    }

    // ---- skinned pass: its own vao/pools, vertex_rigged layout ----
    glGenVertexArrays(1, &r.vao_animated);
    glBindVertexArray(r.vao_animated);

    glGenBuffers(1, &r.models_vbo_animated);
    glBindBuffer(GL_ARRAY_BUFFER, r.models_vbo_animated);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(SKIN_POOL_VERTICES * sizeof(vertex_rigged)), 0, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_rigged), (void*)offsetof(vertex_rigged, position));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_rigged), (void*)offsetof(vertex_rigged, normal));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(vertex_rigged), (void*)offsetof(vertex_rigged, uv));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(vertex_rigged), (void*)offsetof(vertex_rigged, bone_ids));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(vertex_rigged), (void*)offsetof(vertex_rigged, bone_weights));
    glEnableVertexAttribArray(4);

    glGenBuffers(1, &r.ebo_animated);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r.ebo_animated);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)(SKIN_POOL_INDICES * sizeof(uint16_t)), 0, GL_STATIC_DRAW);

    glBindVertexArray(0);

    r.skinned_shader = opengl_create_shader(VSHDER_SKINNED, shader_with_pbr(FSHDER_BASIC));

    unsigned char white[4] = { 255, 255, 255, 255 };
    r.white_texture = opengl_create_texture2d(1, 1, 4, white, TEXTURE_PIXELATED);

    r.projection_matrix = mat4_perspective(1.05f, (float)width / (float)height, 0.1f, 800.0f);
    r.view_matrix = mat4_identity();

    // A late morning sun: high enough that shadows are short and legible from
    // a third person camera, off-axis enough that every face of a box gets a
    // different amount of light.
    // Lighting is done in linear space now (the post pass owns the tonemap and
    // the single sRGB encode), so these are radiances, not 0..1 screen colours:
    // the sun sits well above 1, the sky/ground terms are pre-integrated diffuse
    // irradiance. Ambient still has to stay well under the sun or shadowed faces
    // wash out and the shadow map stops being visible - the point of casting it.
    pix_lights_clear(r.lights);
    r.shadow_strength = 1.0f;
    r.shadow_extent = 220.0f;
    r.shadow_depth  = 620.0f;
    r.saturation    = 1.14f;
    // A third of the way to the full smoothstep curve. Past about a half the
    // midtones start to crush and a shadowed wall goes to black, which is the
    // failure mode of every contrast slider; a third is the amount that reads
    // as "the image has depth" rather than as "the image has been graded".
    r.contrast      = 0.28f;
    r.split_tone    = 0.55f;
    // Dropped to just under diffuse white. At 1.05 nothing but a mirror
    // highlight ever crossed it, so the bloom pass ran every frame and put
    // almost nothing on screen; a shade below white catches a sunlit wall's
    // brightest face and the sky behind it, which is where the glow in a
    // photograph actually comes from. Held back in strength to pay for the
    // wider catchment.
    r.bloom_threshold = 0.92f;
    r.bloom_knee      = 0.45f;
    r.bloom_strength  = 0.34f;
    // The vignette curve moved out to the corners (see FSHDER_POST), so the
    // same visible darkening needs a larger number than it did.
    r.vignette      = 0.30f;
    r.sharpen       = 0.30f;
    // A real river is a green-blue the sky sits on top of, not a flat cyan
    // slab - the blue mostly comes from the reflection.
    //
    // But these are sRGB values that the shader linearises, and that squares
    // them: the 0.13 / 0.03 pair this used to hold arrived in the lighting as
    // 0.015 and 0.002, which is charcoal. Since the fresnel term underfoot is
    // only ~0.06, that body colour is nearly all of what a river reads as
    // from anywhere except looking at the far bank - so the water was
    // near-black by day with a bright horizon reflection at the far end, and
    // "the water looks black" is exactly that. Roughly doubled: still a
    // silty green rather than a pool blue, but a colour rather than a shadow.
    // Pushed from a flat silty green toward the green-blue open water actually
    // is, and the two ends pulled further apart. The old pair were close enough
    // in both hue and value that the depth term had almost nothing to work
    // with, so the whole surface came out as one shade whatever angle it was
    // seen from - and the shallows, which are the part you stand next to, were
    // the muddiest thing in the frame.
    //
    // Shallow is now a bright aqua: shallow water over a lit bed is the
    // lightest, most saturated water there is. Deep goes to a proper blue and
    // much darker, so the middle of a channel reads as depth rather than as
    // more of the same green, and the gradient between them across a bank is
    // the thing that makes water read as having a bottom.
    r.water_shallow = v3(0.22f, 0.52f, 0.54f);
    r.water_deep    = v3(0.03f, 0.13f, 0.24f);
    // Deeper, to match: the murk term is 1 - exp(-depth), so a shallower
    // number reached the deep colour within a metre of the bank and flattened
    // the gradient the two colours above exist to make.
    r.water_depth   = 2.9f;
    r.weather.overcast = 0.0f;
    r.weather.rain = 0.0f;
    r.weather.wetness = 0.0f;
    // The two per-frame uniform blocks. Bound to their binding points once,
    // here, and never rebound: the shaders name the binding themselves
    // (layout(std140, binding = N)), so there is nothing per program to do.
    glGenBuffers(1, &r.scene_ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, r.scene_ubo);
    glBufferData(GL_UNIFORM_BUFFER, (GLsizeiptr)sizeof(pix_scene_block), 0, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, r.scene_ubo);

    glGenBuffers(1, &r.lights_ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, r.lights_ubo);
    glBufferData(GL_UNIFORM_BUFFER, (GLsizeiptr)sizeof(pix_light_block), 0, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, 1, r.lights_ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    pix_set_time_of_day(r, 10.0f);      // a late morning, shadows short and legible
    for (int c = 0; c < SHADOW_CASCADES; c++) r.light_space[c] = mat4_identity();
    return r;
}

static void pix_enable_effects(pix_renderer& r, int shadow_resolution) {
    if (r.effects) return;
    r.scene_target = pix_create_render_target(r.width, r.height, true);
    // One atlas, the cascades side by side. Three separate maps would need
    // three samplers and three binds per draw; one texture needs neither, and
    // the only cost is remembering to inset each tile so a filter tap cannot
    // read into its neighbour.
    r.shadow_map   = pix_create_shadow_map(shadow_resolution * SHADOW_CASCADES,
                                           shadow_resolution);
    // quarter res is plenty: the result is about to be blurred anyway, and it
    // makes the two Gaussian passes essentially free
    int bw = r.width / 4 > 1 ? r.width / 4 : 1;
    int bh = r.height / 4 > 1 ? r.height / 4 : 1;
    r.bloom_a = pix_create_render_target(bw, bh, true);
    r.bloom_b = pix_create_render_target(bw, bh, true);
    // A quarter of linear resolution: the water shader distorts and blends
    // this, so it never needs to be as sharp as the scene it is mirroring,
    // and the opaque pass it costs is the single most expensive thing this
    // adds - keeping the pixel count down is most of how it stays affordable.
    int rw = r.width / 4 > 1 ? r.width / 4 : 1;
    int rh = r.height / 4 > 1 ? r.height / 4 : 1;
    r.reflection_target = pix_create_render_target(rw, rh, true, true);
    r.bloom_prefilter_shader = opengl_create_shader(VSHDER_POST, FSHDER_BLOOM_PREFILTER);
    r.bloom_blur_shader      = opengl_create_shader(VSHDER_POST, FSHDER_BLOOM_BLUR);
    r.depth_shader         = opengl_create_shader(VSHDER_DEPTH, FSHDER_DEPTH);
    r.depth_skinned_shader = opengl_create_shader(VSHDER_DEPTH_SKINNED, FSHDER_DEPTH);
    r.post_shader          = opengl_create_shader(VSHDER_POST, shader_with_common(FSHDER_POST));
    // both stages evaluate the same wave field, so both get it spliced in
    {
        const char* wvs = shader_with_waves(VSHDER_WATER);
        const char* wfs = shader_with_waves(FSHDER_WATER);
        r.water_shader = opengl_create_shader(wvs, wfs);
    }
    r.sky_shader           = opengl_create_shader(VSHDER_SKY,  shader_with_common(FSHDER_SKY));

    // Particles: one buffer, streamed whole every frame, and no geometry to
    // go with it - the quad's corners come out of gl_VertexID, so the only
    // attributes bound here are the per-instance ones.
    r.particle_shader = opengl_create_shader(VSHDER_PARTICLE, FSHDER_PARTICLE);
    glGenVertexArrays(1, (GLuint*)&r.particle_vao);
    glGenBuffers(1, (GLuint*)&r.particle_vbo);
    glBindVertexArray(r.particle_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.particle_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)sizeof(r.particles), 0, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(pix_particle_vertex), (void*)0);
    glVertexAttribDivisor(0, 1);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(pix_particle_vertex),
                          (void*)(sizeof(float) * 4));
    glVertexAttribDivisor(1, 1);
    glBindVertexArray(0);

    r.effects = true;
}

static void pix_set_time(pix_renderer& r, float seconds) { r.time = seconds; }

static void pix_enable_gpu_timing(pix_renderer& r, bool on) {
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

static double pix_gpu_pass_ms(const pix_renderer& r, int pass) {
    return (unsigned)pass < PIX_GPU_PASSES ? r.gpu_ms[pass] : 0.0;
}

static double pix_cpu_pass_ms(const pix_renderer& r, int pass) {
    return (unsigned)pass < PIX_GPU_PASSES ? r.cpu_ms[pass] : 0.0;
}

// A pass is bracketed by these two. Only one GL_TIME_ELAPSED query can be
// running at a time, so they never nest: each pass opens one and closes it
// before the next one begins.
static double pix__cpu_seconds() {
    LARGE_INTEGER f, t;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&t);
    return (double)t.QuadPart / (double)f.QuadPart;
}

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

    // Last frame's query for this pass. The GPU is a whole frame past it by
    // now, so the answer is already sitting there and asking costs nothing;
    // the first frames have nothing to read, which the availability check
    // answers without ever stalling the pipeline.
    GLuint previous = r.gpu_query[pass][r.gpu_slot ^ 1];
    if (!previous) return;
    GLint ready = 0;
    glGetQueryObjectiv(previous, GL_QUERY_RESULT_AVAILABLE, &ready);
    if (!ready) return;
    GLuint64 ns = 0;
    glGetQueryObjectui64v(previous, GL_QUERY_RESULT, &ns);
    // Smoothed for the same reason the CPU side averages over a second: one
    // frame of one pass is noise even when the average is solid.
    r.gpu_ms[pass] += ((double)ns / 1.0e6 - r.gpu_ms[pass]) * 0.1;
}

static void push_water(pix_renderer& r, idx mesh, idx material, const mat4& transform) {
    if (mesh == (idx)-1 || r.water_count >= MAX_WATER_INSTANCES) return;
    // Every water tile in one river sits at the same surface height, so the
    // last one pushed this frame is as good a plane as the first; this is
    // what the reflection camera mirrors across.
    r.water_plane_y = transform.data[13];
    pix_render_instance& it = r.water_instances[r.water_count++];
    it.mesh = mesh;
    it.material = material;
    it.transform = transform;
}

static void push_particle(pix_renderer& r, vec3 at, float size, vec3 color, float alpha) {
    if (r.particle_count >= MAX_PARTICLES || alpha <= 0.0f || size <= 0.0f) return;
    pix_particle_vertex& v = r.particles[r.particle_count++];
    v.centre[0] = at.x; v.centre[1] = at.y; v.centre[2] = at.z;
    v.size = size;
    v.color[0] = color.x; v.color[1] = color.y; v.color[2] = color.z; v.color[3] = alpha;
}

static void pix_destory_renderer(pix_renderer& renderer) {
    if (renderer.effects) {
        pix_destroy_framebuffer(renderer.scene_target);
        pix_destroy_framebuffer(renderer.shadow_map);
        pix_destroy_framebuffer(renderer.reflection_target);
        pix_destroy_framebuffer(renderer.bloom_a);
        pix_destroy_framebuffer(renderer.bloom_b);
        renderer.effects = false;
    }
    free(renderer.tex_arena.data);
    renderer.tex_arena.data = 0;
    renderer.tex_arena.capacity = 0;
    renderer.tex_arena.size = 0;
}

static idx load_mesh(pix_renderer& r, mesh_file_data& md) {
    // Overrunning the shared pool used to fail silently: glBufferSubData past
    // the end of a buffer is simply ignored, so the last models loaded came out
    // as invisible geometry rather than as an error. Refuse instead.
    if (r.mesh_count >= MAX_MESHES) return (idx)-1;
    if (r.vertex_cursor + md.vertex_count > MESH_POOL_VERTICES) return (idx)-1;
    if (r.index_cursor  + md.index_count  > MESH_POOL_INDICES)  return (idx)-1;
    if (md.vertex_count > 65536) return (idx)-1;   // indices are uint16 per mesh

    idx id = (idx)r.mesh_count++;
    mesh& m = r.meshes[id];
    m.vbo = r.models_vbo;
    m.vertex_offset = r.vertex_cursor;
    m.vertex_size = sizeof(vertex);
    m.vertex_count = (idx)md.vertex_count;
    m.index_offset = r.index_cursor;
    m.index_count = (idx)md.index_count;

    glBindVertexArray(r.vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.models_vbo);
    glBufferSubData(GL_ARRAY_BUFFER,
        (GLintptr)(r.vertex_cursor * sizeof(vertex)),
        (GLsizeiptr)(md.vertex_count * sizeof(vertex)), md.vertex_data);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r.ebo);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,
        (GLintptr)(r.index_cursor * sizeof(uint16_t)),
        (GLsizeiptr)(md.index_count * sizeof(uint16_t)), md.index_data);
    glBindVertexArray(0);

    r.vertex_cursor += (idx)md.vertex_count;
    r.index_cursor += (idx)md.index_count;
    return id;
}

static idx load_skinned_mesh(pix_renderer& r, skinned_mesh_file_data& md) {
    if (r.skinned_mesh_count >= MAX_MESHES) return (idx)-1;
    if (r.skin_vertex_cursor + md.vertex_count > SKIN_POOL_VERTICES) return (idx)-1;
    if (r.skin_index_cursor  + md.index_count  > SKIN_POOL_INDICES)  return (idx)-1;
    idx id = (idx)r.skinned_mesh_count++;
    mesh& m = r.skinned_meshes[id];
    m.vbo = r.models_vbo_animated;
    m.vertex_offset = r.skin_vertex_cursor;
    m.vertex_size = sizeof(vertex_rigged);
    m.vertex_count = (idx)md.vertex_count;
    m.index_offset = r.skin_index_cursor;
    m.index_count = (idx)md.index_count;

    glBindVertexArray(r.vao_animated);
    glBindBuffer(GL_ARRAY_BUFFER, r.models_vbo_animated);
    glBufferSubData(GL_ARRAY_BUFFER,
        (GLintptr)(r.skin_vertex_cursor * sizeof(vertex_rigged)),
        (GLsizeiptr)(md.vertex_count * sizeof(vertex_rigged)), md.vertex_data);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, r.ebo_animated);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,
        (GLintptr)(r.skin_index_cursor * sizeof(uint16_t)),
        (GLsizeiptr)(md.index_count * sizeof(uint16_t)), md.index_data);
    glBindVertexArray(0);

    r.skin_vertex_cursor += (idx)md.vertex_count;
    r.skin_index_cursor += (idx)md.index_count;
    return id;
}

static idx load_material(pix_renderer& r, const char* texture_path, vec3 color, float metalic, float roughness) {
    // Reuse the last slot rather than writing past the table: an overrun
    // here corrupts whatever field the compiler placed right after
    // `materials[]` in pix_renderer, which does not fail loudly - it fails
    // as "half the world stopped rendering" several frames later.
    if (r.material_count >= MAX_MATERIALS) return (idx)(MAX_MATERIALS - 1);
    idx id = (idx)r.material_count++;
    material& mat = r.materials[id];
    mat.shader = r.shader;
    mat.texture = 0;
    mat.color = color;
    mat.metallic = metalic;
    mat.roughness = roughness;
    mat.emissive = v3(0.0f, 0.0f, 0.0f);
    mat.window_glow = v3(0.0f, 0.0f, 0.0f);
    mat.glass = 0.0f;

    if (texture_path) {
        png_result png = {};
        if (png_load_file(r.tex_arena, texture_path, &png)) {
            // No mipmaps. A kit atlas is typically a grid of flat colour
            // swatches with no gutters, and mipmapping one bleeds neighbouring
            // swatches into each other until the whole scene washes out. A
            // caller wanting trilinear filtering on a real uv-mapped texture
            // should load it through load_material_image instead.
            mat.texture = opengl_create_texture2d(png.width, png.height, 4, png.pixels, TEXTURE_LINEAR);
        }
        arena_reset(r.tex_arena); // decoded pixels now live on the GPU
    }
    return id;
}

static idx load_material_image(pix_renderer& r, image_file_data* image, vec3 color,
                               float metalic, float roughness, bool pixelated) {
    if (r.material_count >= MAX_MATERIALS) return (idx)(MAX_MATERIALS - 1);
    idx id = (idx)r.material_count++;
    material& mat = r.materials[id];
    mat.shader = r.shader;
    mat.texture = 0;
    mat.color = color;
    mat.metallic = metalic;
    mat.roughness = roughness;
    mat.emissive = v3(0.0f, 0.0f, 0.0f);
    mat.window_glow = v3(0.0f, 0.0f, 0.0f);
    mat.glass = 0.0f;
    // Mipmaps, unless it is a generated palette.
    //
    // The newer packs each ship one real texture per model and *tile* it -
    // a tree's leaf texture runs from uv -7 to +7 across its canopy. Sampled
    // with no mip chain, every screen pixel takes one texel out of a
    // high-frequency 1024 x 1024 image, and neighbouring pixels land nowhere
    // near each other: the canopy turns into the scribble of hatched slivers
    // that made the trees look like broken geometry rather than aliasing.
    //
    // A palette must stay unfiltered and unmipmapped whatever happens - each
    // cell in it is a whole material's colour, and blending two of them
    // together is a character wearing a colour that appears nowhere in the
    // file.
    if (image && image->data)
        mat.texture = opengl_create_texture2d(image->width, image->height, 4, image->data,
                                              pixelated ? TEXTURE_PIXELATED : TEXTURE_BILINEAR);
    return id;
}

static idx clone_material(pix_renderer& r, idx source, vec3 color,
                          float metallic, float roughness) {
    if (source >= r.material_count || r.material_count >= MAX_MATERIALS) return source;
    idx id = (idx)r.material_count++;
    material& mat = r.materials[id];
    mat.shader = r.materials[source].shader;
    mat.texture = r.materials[source].texture;   // shared, not re-uploaded
    mat.color = color;
    mat.metallic = metallic;
    mat.roughness = roughness;
    mat.emissive = r.materials[source].emissive;
    mat.window_glow = r.materials[source].window_glow;
    mat.glass = r.materials[source].glass;
    return id;
}

static idx get_default_material(pix_renderer& r) {
    return load_material(r, nullptr, v3(1.0f, 1.0f, 1.0f), 0.1f, 0.5f);
}

static idx load_shader(pix_renderer& r, const char* vsrc, const char* fsrc) {
    idx prog = opengl_create_shader(vsrc, fsrc);
    r.shaders[r.shader_count++] = prog;
    return prog;
}

// Fits one orthographic box per cascade around the slice of the view it covers,
// and snaps each to whole shadow-map texels.
//
// Without the snap the box slides continuously as the player walks and every
// shadow edge crawls and shimmers; with it the box only ever moves in whole
// texel steps and the edges sit still. The splits are geometric rather than
// even, because perspective means the near half of the view occupies most of
// the screen and deserves most of the resolution.
static void pix__fit_cascades(pix_renderer& r, const camera& cam) {
    const float near_plane = 6.0f;
    int tile = r.shadow_map.height;                 // the atlas is square per tile

    for (int c = 0; c < SHADOW_CASCADES; c++) {
        // 6 m, 40 m, 220 m for three cascades over a 220 m extent
        float t = (float)(c + 1) / (float)SHADOW_CASCADES;
        float far_d = near_plane * powf(r.shadow_extent / near_plane, t);
        r.cascade_far[c] = far_d;

        // The box has to cover the slice from whatever angle the camera is at,
        // so it is sized by the slice's diagonal rather than its length - which
        // is also what makes its size independent of where the camera looks,
        // and therefore stable under rotation.
        float near_d = (c == 0) ? 0.0f
                                : near_plane * powf(r.shadow_extent / near_plane,
                                                    (float)c / (float)SHADOW_CASCADES);
        float extent = (far_d - near_d) * 0.75f + far_d * 0.35f;

        vec3 focus = v3add(cam.position, v3scale(cam.direction, (near_d + far_d) * 0.5f));
        focus.y = 0.0f;

        float texel = (extent * 2.0f) / (float)tile;
        focus.x = floorf(focus.x / texel) * texel;
        focus.z = floorf(focus.z / texel) * texel;
        r.cascade_texel[c] = texel;

        vec3 eye = v3add(focus, v3scale(r.sun.direction, r.shadow_depth * 0.5f));
        mat4 light_view = mat4_lookat(eye, focus, v3(0.0f, 1.0f, 0.0f));
        mat4 light_proj = mat4_ortho(-extent, extent, -extent, extent, 1.0f, r.shadow_depth);
        r.light_space[c] = mat4_mul(light_proj, light_view);
    }
}

static void begin_frame(pix_renderer& r, camera& cam) {
    r.view_matrix = mat4_lookat(cam.position, v3add(cam.position, cam.direction), cam.up);
    r.camera_position = cam.position;
    r.camera_dir = cam.direction;
    r.camera_up = cam.up;
    r.renderable_count = 0;
    r.animated_count = 0;
    r.water_count = 0;
    pix_lights_clear(r.lights);
    if (r.effects) pix__fit_cascades(r, cam);
}

static void push_instance(pix_renderer& r, pix_render_instance& instance) {
    if (r.renderable_count < MAX_RENDERABLE)
        r.instances[r.renderable_count++] = instance;
}

// the form world generation actually uses: no temporary to fill in first
static void push_instance(pix_renderer& r, idx mesh, idx material, const mat4& transform) {
    if (mesh == (idx)-1 || material == (idx)-1) return;
    if (r.renderable_count >= MAX_RENDERABLE) return;
    pix_render_instance& it = r.instances[r.renderable_count++];
    it.mesh = mesh;
    it.material = material;
    it.transform = transform;
}

static void push_animated_instance(pix_renderer& r, pix_render_instance& instance, const animation& pose) {
    if (r.animated_count >= MAX_ANIMATED) return;
    r.animated_instances[r.animated_count] = instance;
    r.animated_poses[r.animated_count] = &pose;   // referenced, not copied
    r.animated_count++;
}

// Grouping instances by (mesh, material) is what turns the frame into a handful
// of instanced draws, and with tens of thousands of them a comparison sort
// shows up in the profile. The key space is small and known - MAX_MESHES *
// MAX_MATERIALS - so a counting sort does it in one linear pass instead.
//
// It sorts on distance first, and that second pass is what pays for itself
// several times over. A counting sort is stable, so ordering by distance and
// then re-ordering by (mesh, material) leaves every batch internally sorted
// near to far - which means "every instance of this batch within N metres" is
// a *prefix* of the batch, not a scattered subset of it. The shadow cascades
// and the reflection are exactly that query, and being able to answer it with
// an offset and a length is what lets all five passes of a frame read one
// upload instead of each compacting and uploading its own copy.
#define PIX__SORT_KEYS (MAX_MESHES * MAX_MATERIALS)

static void pix__sort_instances(pix_render_instance* items, size_t count, vec3 eye) {
    static uint32_t counts[PIX__SORT_KEYS + 1];
    static pix_render_instance scratch[MAX_RENDERABLE];
    static uint32_t keys[MAX_RENDERABLE];
    if (count < 1) return;

    // ---- pass one: distance ----
    for (size_t i = 0; i < count; i++) {
        const mat4& t = items[i].transform;
        float dx = t.data[12] - eye.x, dy = t.data[13] - eye.y, dz = t.data[14] - eye.z;
        float d = sqrtf(dx * dx + dy * dy + dz * dz) * (1.0f / PIX_DIST_BUCKET);
        uint32_t b = d <= 0.0f ? 0u : (uint32_t)d;
        items[i].dist_bucket = b < PIX_DIST_BUCKETS ? b : PIX_DIST_BUCKETS - 1;
    }
    if (count < 2) return;

    memset(counts, 0, (PIX_DIST_BUCKETS + 1) * sizeof(uint32_t));
    for (size_t i = 0; i < count; i++) counts[items[i].dist_bucket + 1]++;
    for (uint32_t b = 0; b < PIX_DIST_BUCKETS; b++) counts[b + 1] += counts[b];
    for (size_t i = 0; i < count; i++) scratch[counts[items[i].dist_bucket]++] = items[i];
    memcpy(items, scratch, count * sizeof(pix_render_instance));

    // ---- pass two: the batch key, stably, so the distance order survives ----
    uint32_t lo = PIX__SORT_KEYS, hi = 0;
    for (size_t i = 0; i < count; i++) {
        uint32_t k = (uint32_t)items[i].mesh * MAX_MATERIALS + (uint32_t)items[i].material;
        if (k >= PIX__SORT_KEYS) k = PIX__SORT_KEYS - 1;   // malformed instance, keep it in range
        keys[i] = k;
        if (k < lo) lo = k;
        if (k > hi) hi = k;
    }
    if (lo == hi) return;                                  // already one batch, already in order

    memset(counts + lo, 0, (hi - lo + 2) * sizeof(uint32_t));
    for (size_t i = 0; i < count; i++) counts[keys[i] + 1]++;
    for (uint32_t k = lo; k <= hi; k++) counts[k + 1] += counts[k];
    for (size_t i = 0; i < count; i++) scratch[counts[keys[i]]++] = items[i];
    memcpy(items, scratch, count * sizeof(pix_render_instance));
}

// ---- the instance stream ----
//
// Takes fresh space in instances_vbo, uploads `count` transforms into it, and
// hands back the instance index the draw should start at. Nothing in a frame
// is ever written twice, so no upload has to wait for a draw that is still
// reading the bytes it wants.
//
// The wrap is a safety net, not a mechanism: the buffer is sized for
// everything a frame actually uploads, and a frame that somehow overruns it
// starts again at zero - one stall, right picture.
static GLuint pix__instance_upload(pix_renderer& r, const mat4* data, size_t count) {
    if (count == 0) return 0;
    if (r.instance_cursor + count > r.instance_capacity) r.instance_cursor = 0;
    size_t base = r.instance_cursor;
    glBufferSubData(GL_ARRAY_BUFFER, (GLintptr)(base * sizeof(mat4)),
                    (GLsizeiptr)(count * sizeof(mat4)), data);
    r.instance_cursor = base + count;
    return (GLuint)base;
}

// Discards the whole buffer's contents so the driver hands back new storage
// instead of making this frame wait for the last one to finish reading the old
// storage. Called once, at the top of end_frame.
static void pix__instance_frame_begin(pix_renderer& r) {
    glBindBuffer(GL_ARRAY_BUFFER, r.instances_vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(r.instance_capacity * sizeof(mat4)), 0,
                 GL_STREAM_DRAW);
    r.instance_cursor = 0;
    r.opaque_uploaded = false;
}

// This frame's sorted opaque transforms, uploaded on the first pass that asks
// for them and read by every later one - the shadow cascades, the reflection
// and the colour pass all draw the same list in the same order.
static void pix__instance_upload_opaque(pix_renderer& r) {
    if (r.opaque_uploaded || !r.renderable_count) return;
    static mat4 transforms[MAX_RENDERABLE];
    for (size_t i = 0; i < r.renderable_count; i++) transforms[i] = r.instances[i].transform;
    glBindBuffer(GL_ARRAY_BUFFER, r.instances_vbo);
    r.opaque_base = pix__instance_upload(r, transforms, r.renderable_count);
    r.opaque_uploaded = true;
}

// Walks the sorted instance list once, issuing one instanced draw per
// (mesh, material) run. Shared by the shadow pass and the colour pass; only the
// shadow pass skips the material work, since depth does not care about it.
static void pix__draw_instanced(pix_renderer& r, idx program, bool with_material) {
    pix__instance_upload_opaque(r);

    GLint u_color    = with_material ? glGetUniformLocation(program, "uColor")     : -1;
    GLint u_metallic = with_material ? glGetUniformLocation(program, "uMetallic")  : -1;
    GLint u_rough    = with_material ? glGetUniformLocation(program, "uRoughness") : -1;
    GLint u_emissive = with_material ? glGetUniformLocation(program, "uEmissive")  : -1;
    GLint u_glow     = with_material ? glGetUniformLocation(program, "uWindowGlow") : -1;
    GLint u_glass    = with_material ? glGetUniformLocation(program, "uGlass")      : -1;

    glBindVertexArray(r.vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.instances_vbo);

    size_t i = 0;
    while (i < r.renderable_count) {
        idx mi = r.instances[i].mesh;
        idx ma = r.instances[i].material;
        size_t j = i;
        while (j < r.renderable_count && r.instances[j].mesh == mi && r.instances[j].material == ma)
            j++;
        int n = (int)(j - i);

        if (with_material) {
            material& mat = r.materials[ma];
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, mat.texture ? mat.texture : r.white_texture);
            glUniform3f(u_color, mat.color.x, mat.color.y, mat.color.z);
            glUniform1f(u_metallic, mat.metallic);
            glUniform1f(u_rough, mat.roughness);
            glUniform3f(u_emissive, mat.emissive.x, mat.emissive.y, mat.emissive.z);
            glUniform3f(u_glow, mat.window_glow.x, mat.window_glow.y, mat.window_glow.z);
            glUniform1f(u_glass, mat.glass);
        }

        // The run is already sitting in the buffer at its own offset - this
        // draw just says where it starts rather than copying it there.
        mesh& me = r.meshes[mi];
        r.frame_draws++; r.frame_instances += n;
        glDrawElementsInstancedBaseVertexBaseInstance(
            GL_TRIANGLES, (GLsizei)me.index_count, GL_UNSIGNED_SHORT,
            (void*)(size_t)(me.index_offset * sizeof(uint16_t)), n, (GLint)me.vertex_offset,
            (GLuint)(r.opaque_base + i));

        i = j;
    }
}

// Same as pix__draw_instanced, but only the instances within max_dist metres
// of the camera. Of the camera specifically, not of an arbitrary point: the
// distance it tests against is the one pix__sort_instances already measured
// and bucketed, which is what makes the test a prefix length rather than a
// scan. Used by the shadow pass (each cascade only has to consider
// what could possibly fall in its own, much smaller box - see
// pix__fit_cascades) and by the reflection pass (a wave-distorted,
// third-resolution mirror image has no use for the real camera's full view
// distance). Cutting instances here removes the vertices and the draw calls
// both, not just the fragments.
//
// Because pix__sort_instances leaves every batch ordered near to far, the
// instances that survive the cutoff are the front of the batch and nothing
// else: this walks each run until the first instance past the cutoff and draws
// the prefix. No compaction, no copy and no upload - it draws out of the same
// bytes the colour pass does. The cutoff is rounded up to a whole bucket, so
// what it draws is always a superset of what the test asks for.
static void pix__draw_instanced_near(pix_renderer& r, idx program, bool with_material,
                                     float max_dist) {
    pix__instance_upload_opaque(r);

    GLint u_color    = with_material ? glGetUniformLocation(program, "uColor")     : -1;
    GLint u_metallic = with_material ? glGetUniformLocation(program, "uMetallic")  : -1;
    GLint u_rough    = with_material ? glGetUniformLocation(program, "uRoughness") : -1;
    GLint u_emissive = with_material ? glGetUniformLocation(program, "uEmissive")  : -1;
    GLint u_glow     = with_material ? glGetUniformLocation(program, "uWindowGlow") : -1;
    GLint u_glass    = with_material ? glGetUniformLocation(program, "uGlass")      : -1;

    uint32_t cutoff = (uint32_t)(max_dist / PIX_DIST_BUCKET) + 1;
    if (cutoff >= PIX_DIST_BUCKETS) cutoff = PIX_DIST_BUCKETS - 1;

    glBindVertexArray(r.vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.instances_vbo);

    size_t i = 0;
    while (i < r.renderable_count) {
        idx mi = r.instances[i].mesh;
        idx ma = r.instances[i].material;
        size_t j = i, near_end = i;
        while (j < r.renderable_count && r.instances[j].mesh == mi && r.instances[j].material == ma) {
            if (r.instances[j].dist_bucket <= cutoff) near_end = j + 1;
            j++;
        }
        int n = (int)(near_end - i);
        if (n <= 0) { i = j; continue; }     // the whole batch is too far away

        if (with_material) {
            material& mat = r.materials[ma];
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, mat.texture ? mat.texture : r.white_texture);
            glUniform3f(u_color, mat.color.x, mat.color.y, mat.color.z);
            glUniform1f(u_metallic, mat.metallic);
            glUniform1f(u_rough, mat.roughness);
            glUniform3f(u_emissive, mat.emissive.x, mat.emissive.y, mat.emissive.z);
            glUniform3f(u_glow, mat.window_glow.x, mat.window_glow.y, mat.window_glow.z);
            glUniform1f(u_glass, mat.glass);
        }

        mesh& me = r.meshes[mi];
        r.frame_draws++; r.frame_instances += n;
        glDrawElementsInstancedBaseVertexBaseInstance(
            GL_TRIANGLES, (GLsizei)me.index_count, GL_UNSIGNED_SHORT,
            (void*)(size_t)(me.index_offset * sizeof(uint16_t)), n, (GLint)me.vertex_offset,
            (GLuint)(r.opaque_base + i));

        i = j;
    }
}

// One draw per animated instance: each needs its own uBones upload, so there is
// nothing instancing could save here.
static void pix__draw_skinned(pix_renderer& r, idx program, bool with_material) {
    GLint u_color    = with_material ? glGetUniformLocation(program, "uColor")     : -1;
    GLint u_metallic = with_material ? glGetUniformLocation(program, "uMetallic")  : -1;
    GLint u_rough    = with_material ? glGetUniformLocation(program, "uRoughness") : -1;
    GLint u_emissive = with_material ? glGetUniformLocation(program, "uEmissive")  : -1;
    GLint u_glow     = with_material ? glGetUniformLocation(program, "uWindowGlow") : -1;
    GLint u_glass    = with_material ? glGetUniformLocation(program, "uGlass")      : -1;
    GLint u_model = glGetUniformLocation(program, "uModel");
    GLint u_bones = glGetUniformLocation(program, "uBones[0]");
    if (u_bones < 0) u_bones = glGetUniformLocation(program, "uBones");

    glBindVertexArray(r.vao_animated);
    for (size_t k = 0; k < r.animated_count; k++) {
        pix_render_instance& inst = r.animated_instances[k];
        const animation* pose = r.animated_poses[k];
        if (!pose || inst.mesh >= r.skinned_mesh_count) continue;
        // uBones[] persists between draws, so a pose carrying no bones would
        // skin this mesh with whatever rig was drawn before it - a torn giant
        // rather than a missing character. Drop the instance instead.
        if (!pose->bone_count) continue;

        if (with_material) {
            material& mat = r.materials[inst.material];
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, mat.texture ? mat.texture : r.white_texture);
            glUniform3f(u_color, mat.color.x, mat.color.y, mat.color.z);
            glUniform1f(u_metallic, mat.metallic);
            glUniform1f(u_rough, mat.roughness);
            glUniform3f(u_emissive, mat.emissive.x, mat.emissive.y, mat.emissive.z);
            glUniform3f(u_glow, mat.window_glow.x, mat.window_glow.y, mat.window_glow.z);
            glUniform1f(u_glass, mat.glass);
        }
        glUniformMatrix4fv(u_model, 1, GL_FALSE, inst.transform.data);

        size_t nb = pose->bone_count < MAX_ANIM_BONES ? pose->bone_count : MAX_ANIM_BONES;
        if (nb) glUniformMatrix4fv(u_bones, (GLsizei)nb, GL_FALSE, pose->bones[0].data);

        mesh& me = r.skinned_meshes[inst.mesh];

        glDrawElementsInstancedBaseVertex(
            GL_TRIANGLES, (GLsizei)me.index_count, GL_UNSIGNED_SHORT,
            (void*)(size_t)(me.index_offset * sizeof(uint16_t)), 1, (GLint)me.vertex_offset);
    }
}

// One upload of everything the whole frame shares.
//
// Called once per frame, after the shadow cascades are fitted (their matrices
// are in here) and before any pass draws. What is left at the call sites is
// what is genuinely per program or per pass: a view-projection, a sampler
// binding, a clip plane.
static void pix__upload_scene(pix_renderer& r) {
    static pix_scene_block b;

    b.sun_dir[0] = r.sun.direction.x; b.sun_dir[1] = r.sun.direction.y;
    b.sun_dir[2] = r.sun.direction.z; b.sun_dir[3] = 0.0f;
    b.sun_color[0] = r.sun.color.x; b.sun_color[1] = r.sun.color.y;
    b.sun_color[2] = r.sun.color.z; b.sun_color[3] = 0.0f;
    b.sky_color[0] = r.sky.diffuse_up.x; b.sky_color[1] = r.sky.diffuse_up.y;
    b.sky_color[2] = r.sky.diffuse_up.z; b.sky_color[3] = 0.0f;
    b.ground_color[0] = r.sky.diffuse_down.x; b.ground_color[1] = r.sky.diffuse_down.y;
    b.ground_color[2] = r.sky.diffuse_down.z; b.ground_color[3] = 0.0f;
    b.sky_zenith[0] = r.sky.zenith.x; b.sky_zenith[1] = r.sky.zenith.y;
    b.sky_zenith[2] = r.sky.zenith.z; b.sky_zenith[3] = 0.0f;
    b.sky_horizon[0] = r.sky.horizon.x; b.sky_horizon[1] = r.sky.horizon.y;
    b.sky_horizon[2] = r.sky.horizon.z; b.sky_horizon[3] = 0.0f;
    b.fog[0] = r.sky.fog.x; b.fog[1] = r.sky.fog.y; b.fog[2] = r.sky.fog.z;
    b.fog[3] = r.sky.fog_density;
    b.camera[0] = r.camera_position.x; b.camera[1] = r.camera_position.y;
    b.camera[2] = r.camera_position.z; b.camera[3] = r.weather.wetness;
    b.shadow[0] = 1.0f / (float)r.shadow_map.width;
    b.shadow[1] = 1.0f / (float)r.shadow_map.height;
    b.shadow[2] = r.effects ? r.shadow_strength : 0.0f;
    b.shadow[3] = 0.0f;
    for (int c = 0; c < SHADOW_CASCADES; c++)
        memcpy(b.light_space[c], r.light_space[c].data, sizeof(float) * 16);
    for (int c = 0; c < 4; c++) {
        b.cascade_far[c]   = c < SHADOW_CASCADES ? r.cascade_far[c] : 0.0f;
        b.cascade_texel[c] = c < SHADOW_CASCADES ? r.cascade_texel[c] : 0.0f;
    }

    glBindBuffer(GL_UNIFORM_BUFFER, r.scene_ubo);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, (GLsizeiptr)sizeof(b), &b);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

// The frame's punctual lights, already reduced to the nearest few.
static void pix__upload_lights(pix_renderer& r) {
    static pix_light_block b;
    memcpy(b.pos_radius,  r.lights.pos_radius,  sizeof(b.pos_radius));
    memcpy(b.color_inner, r.lights.color_inner, sizeof(b.color_inner));
    memcpy(b.dir_outer,   r.lights.dir_outer,   sizeof(b.dir_outer));
    b.meta[0] = (float)r.lights.packed_count;
    b.meta[1] = b.meta[2] = b.meta[3] = 0.0f;

    glBindBuffer(GL_UNIFORM_BUFFER, r.lights_ubo);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, (GLsizeiptr)sizeof(b), &b);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

// What is left of the old per-program lighting upload: the clip plane, which
// really is per pass - pix__reflection_pass sets it to hide whatever lies
// below the water it is mirroring, and every ordinary draw turns it off.
static void pix__set_lighting(pix_renderer& r, idx program) {
    (void)r;
    glUniform1f(glGetUniformLocation(program, "uClipHeight"), 0.0f);
    glUniform1f(glGetUniformLocation(program, "uClipSign"), 0.0f);
}

static void pix__set_clip(idx program, float height, float sign) {
    glUniform1f(glGetUniformLocation(program, "uClipHeight"), height);
    glUniform1f(glGetUniformLocation(program, "uClipSign"), sign);
}

// Lights live in a uniform block now (pix__upload_lights), so binding them per
// program is nothing at all - this is kept as a no-op rather than deleted
// because the call sites read better naming what they need than not.
static void pix__bind_lights(pix_renderer& r, idx program) { (void)r; (void)program; }

// The cascade matrices and their ranges ride in the scene block; what is left
// here is the one thing a uniform block cannot hold, which is the sampler.
static void pix__bind_shadow(pix_renderer& r, idx program) {
    glUniform1i(glGetUniformLocation(program, "uShadowMap"), 1);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, r.effects ? r.shadow_map.depth : r.white_texture);
    glActiveTexture(GL_TEXTURE0);
}

// Renders the world from the sun into the depth-only target. Front faces are
// culled so the depth recorded is the *back* of each object, which moves the
// self-shadowing artefact to surfaces the camera cannot see anyway.
static void pix__shadow_pass(pix_renderer& r) {
    pix_bind_framebuffer(r.shadow_map);
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    int tile = r.shadow_map.height;
    for (int c = 0; c < SHADOW_CASCADES; c++) {
        glViewport(c * tile, 0, tile, tile);

        glUseProgram(r.depth_shader);
        glUniformMatrix4fv(glGetUniformLocation(r.depth_shader, "uViewProj"), 1, GL_FALSE,
                           r.light_space[c].data);
        // pix__fit_cascades boxes each cascade to a slice of the camera's own
        // view frustum, so its far distance bounds how far from the *camera*
        // (not the light) anything in it can possibly be - the box's half
        // width tracks cascade_far closely (see that function), so 2.8x plus
        // a flat margin is comfortably outside it in every direction rather
        // than tuned to the current numbers. Skipping instances beyond that
        // is most of why three cascades do not cost three full passes: the
        // near cascade in particular is a tiny box that a handful of nearby
        // props satisfy, not the thousands the far one has to consider.
        float cascade_cutoff = r.cascade_far[c] * 2.8f + 15.0f;
        pix__draw_instanced_near(r, r.depth_shader, false, cascade_cutoff);

        // Characters only cast into the near cascades. A person is under two
        // metres across; by the far cascade one is smaller than a shadow texel,
        // so the draw costs a full skinned pass and puts nothing on screen.
        if (r.animated_count && c < SHADOW_CASCADES - 1) {
            glUseProgram(r.depth_skinned_shader);
            glUniformMatrix4fv(glGetUniformLocation(r.depth_skinned_shader, "uViewProj"), 1,
                               GL_FALSE, r.light_space[c].data);
            pix__draw_skinned(r, r.depth_skinned_shader, false);
        }
    }
    glViewport(0, 0, r.shadow_map.width, r.shadow_map.height);

    glCullFace(GL_BACK);
    glDisable(GL_CULL_FACE);
    glBindVertexArray(0);
}

// ---- bloom ----
//
// The scene target is real HDR, so a sun glint off a car roof, a lamp head or a
// wave crest is genuinely brighter than 1 while the tonemap squashes it back to
// near white. This is what puts that energy back on screen as glare, and it is
// the single thing that makes a bright highlight read as *bright* rather than
// as a pale patch. Threshold with a soft knee, then two separable Gaussians at
// quarter resolution.
static void pix__bloom_pass(pix_renderer& r) {
    if (r.bloom_strength <= 0.0f || !r.bloom_prefilter_shader) return;

    pix_bind_framebuffer(r.bloom_a);
    glDisable(GL_DEPTH_TEST);
    glUseProgram(r.bloom_prefilter_shader);
    glUniform1i(glGetUniformLocation(r.bloom_prefilter_shader, "uScene"), 0);
    glUniform2f(glGetUniformLocation(r.bloom_prefilter_shader, "uTexel"),
                1.0f / (float)r.width, 1.0f / (float)r.height);
    glUniform1f(glGetUniformLocation(r.bloom_prefilter_shader, "uThreshold"), r.bloom_threshold);
    glUniform1f(glGetUniformLocation(r.bloom_prefilter_shader, "uKnee"), r.bloom_knee);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, r.scene_target.color);
    pix_draw_fullscreen();

    float tx = 1.0f / (float)r.bloom_a.width;
    float ty = 1.0f / (float)r.bloom_a.height;
    glUseProgram(r.bloom_blur_shader);
    glUniform1i(glGetUniformLocation(r.bloom_blur_shader, "uSource"), 0);

    pix_bind_framebuffer(r.bloom_b);
    glUniform2f(glGetUniformLocation(r.bloom_blur_shader, "uDirection"), tx, 0.0f);
    glBindTexture(GL_TEXTURE_2D, r.bloom_a.color);
    pix_draw_fullscreen();

    pix_bind_framebuffer(r.bloom_a);
    glUniform2f(glGetUniformLocation(r.bloom_blur_shader, "uDirection"), 0.0f, ty);
    glBindTexture(GL_TEXTURE_2D, r.bloom_b.color);
    pix_draw_fullscreen();

    glEnable(GL_DEPTH_TEST);
}

static void pix__water_pass(pix_renderer& r, const mat4& view_proj) {
    if (!r.water_count || !r.water_shader) return;
    static mat4 batch[MAX_WATER_INSTANCES];

    glUseProgram(r.water_shader);
    glUniformMatrix4fv(glGetUniformLocation(r.water_shader, "uViewProj"), 1, GL_FALSE, view_proj.data);
    glUniform1f(glGetUniformLocation(r.water_shader, "uTime"), r.time);
    glUniform1f(glGetUniformLocation(r.water_shader, "uBedDepth"), r.water_depth);
    glUniform3f(glGetUniformLocation(r.water_shader, "uShallowColor"),
                r.water_shallow.x, r.water_shallow.y, r.water_shallow.z);
    glUniform3f(glGetUniformLocation(r.water_shader, "uDeepColor"),
                r.water_deep.x, r.water_deep.y, r.water_deep.z);
    pix__set_lighting(r, r.water_shader);

    // The captured planar reflection from pix__reflection_pass, or the white
    // texture with the mix pulled to zero when it did not run this frame -
    // FSHDER_WATER then falls back to the analytic sky entirely, exactly as
    // it always did.
    bool have_reflection = r.reflection_rendered;
    glUniform1f(glGetUniformLocation(r.water_shader, "uReflectionMix"), have_reflection ? 1.0f : 0.0f);
    glUniformMatrix4fv(glGetUniformLocation(r.water_shader, "uReflViewProj"), 1, GL_FALSE,
                       r.reflection_view_proj.data);
    glUniform1i(glGetUniformLocation(r.water_shader, "uReflection"), 1);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, have_reflection ? r.reflection_target.color : r.white_texture);
    glActiveTexture(GL_TEXTURE0);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);          // water does not occlude the water behind it

    glBindVertexArray(r.vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.instances_vbo);

    // Same rule as the opaque list: one upload for the whole pass, and each
    // run draws from its own offset inside it.
    for (size_t k = 0; k < r.water_count; k++) batch[k] = r.water_instances[k].transform;
    GLuint base = pix__instance_upload(r, batch, r.water_count);

    size_t i = 0;
    while (i < r.water_count) {
        idx mi = r.water_instances[i].mesh;
        size_t j = i;
        while (j < r.water_count && r.water_instances[j].mesh == mi) j++;
        mesh& me = r.meshes[mi];
        glDrawElementsInstancedBaseVertexBaseInstance(
            GL_TRIANGLES, (GLsizei)me.index_count, GL_UNSIGNED_SHORT,
            (void*)(size_t)(me.index_offset * sizeof(uint16_t)), (GLsizei)(j - i),
            (GLint)me.vertex_offset, base + (GLuint)i);
        i = j;
    }

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}

// Every particle this frame, as one instanced draw of one quad.
//
// Blended rather than sorted, depth tested against the world but not writing
// depth: smoke that goes behind a wall is hidden, and two puffs overlapping
// each other are simply added in whatever order they were submitted. Sorting
// a few hundred blobs back to front would cost more than the artefact it
// fixes is worth on effects that live for a fraction of a second.
static void pix__particle_pass(pix_renderer& r, const mat4& view_proj) {
    if (!r.particle_count || !r.particle_shader) return;

    // The billboard basis. Taking right from the view matrix rather than
    // recomputing it from the camera's direction and up keeps the quads square
    // to the screen even when the camera is looking almost straight down,
    // where a cross product with world up degenerates.
    vec3 right = v3(r.view_matrix.data[0], r.view_matrix.data[4], r.view_matrix.data[8]);
    vec3 up    = v3(r.view_matrix.data[1], r.view_matrix.data[5], r.view_matrix.data[9]);

    glUseProgram(r.particle_shader);
    glUniformMatrix4fv(glGetUniformLocation(r.particle_shader, "uViewProj"), 1, GL_FALSE,
                       view_proj.data);
    glUniform3f(glGetUniformLocation(r.particle_shader, "uCamRight"), right.x, right.y, right.z);
    glUniform3f(glGetUniformLocation(r.particle_shader, "uCamUp"), up.x, up.y, up.z);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    glBindVertexArray(r.particle_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r.particle_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    (GLsizeiptr)(r.particle_count * sizeof(pix_particle_vertex)), r.particles);
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, (GLsizei)r.particle_count);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}

// The backdrop. A full-screen triangle turned back into world-space view rays
// by the inverse view-projection, shaded by the same analytic sky the surface
// and water shaders sample - so what fills the horizon and what a car reflects
// are the same function. Drawn straight after the clear with depth writes off,
// so every later opaque pixel paints over it for free.
static void pix__sky_pass(pix_renderer& r, const mat4& view_proj) {
    if (!r.sky_shader) return;
    mat4 inv = mat4_inverse(view_proj);

    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);

    glUseProgram(r.sky_shader);
    // The sky it draws and the camera it draws from both come out of the scene
    // block; the only thing this pass owns is the matrix that turns a screen
    // corner back into a view ray.
    glUniformMatrix4fv(glGetUniformLocation(r.sky_shader, "uInvViewProj"), 1, GL_FALSE, inv.data);
    pix_draw_fullscreen();

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
}

// ---- planar water reflection ----
//
// Renders the opaque scene and sky again from a camera mirrored across the
// water's surface, into reflection_target - the texture the water shader
// projects each fragment's world position into (see FSHDER_WATER, uReflViewProj).
// Mirroring the camera rather than the geometry means every existing draw
// path runs unmodified: same instances, same shader, same shadow map, just a
// different view matrix.
//
// Two things are deliberately left out to keep this cheap:
//   skinned meshes   people and the player are a few pixels across in a
//                    reflection seen from a walking camera; skipping them
//                    halves the draws this pass costs.
//   the far side     anything below the water plane is clipped (uClipHeight/
//                    uClipSign on FSHDER_BASIC) - the riverbed is invisible
//                    to the real camera and would otherwise show up mirrored
//                    into the reflected sky.
static void pix__reflection_pass(pix_renderer& r) {
    r.reflection_rendered = false;
    if (!r.water_count || !r.reflection_target.fbo || !r.shader) return;
    // A camera at or below the water's surface has no meaningful mirror
    // image - it would be looking along the plane or from underneath it.
    if (r.camera_position.y <= r.water_plane_y + 0.05f) return;

    vec3 eye = r.camera_position;
    eye.y = 2.0f * r.water_plane_y - eye.y;
    vec3 dir = r.camera_dir; dir.y = -dir.y;
    vec3 up  = r.camera_up;  up.y  = -up.y;

    mat4 view = mat4_lookat(eye, v3add(eye, dir), up);
    mat4 vp   = mat4_mul(r.projection_matrix, view);
    r.reflection_view_proj = vp;

    pix_bind_framebuffer(r.reflection_target);
    glEnable(GL_DEPTH_TEST);
    glClearColor(r.clear_color.x, r.clear_color.y, r.clear_color.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    pix__sky_pass(r, vp);

    glUseProgram(r.shader);
    glUniformMatrix4fv(glGetUniformLocation(r.shader, "uViewProj"), 1, GL_FALSE, vp.data);
    glUniform1i(glGetUniformLocation(r.shader, "uTex"), 0);
    pix__set_lighting(r, r.shader);
    // Punctual lights and shadow sampling are skipped here: both are a real
    // per-fragment cost the main pass already pays in full, and a mirrored,
    // wave-distorted, third-resolution reflection does not read the
    // difference - the sun, the sky ambient and each material's own emissive
    // (window glow, headlights) already carry it. uShadowStrength <= 0 makes
    // sun_shadow() return early without a single texture tap.
    glUniform1i(glGetUniformLocation(r.shader, "uLightCount"), 0);
    glUniform1f(glGetUniformLocation(r.shader, "uShadowStrength"), 0.0f);
    pix__set_clip(r.shader, r.water_plane_y, 1.0f);
    // 160m instead of the real camera's 420m view distance: a wave-distorted,
    // third-resolution mirror image loses nothing worth having past that, and
    // it is most of why this pass does not cost as much as the main one.
    pix__draw_instanced_near(r, r.shader, true, 160.0f);

    glBindVertexArray(0);
    // Rebuilt every frame - the capture is new each time - so the water's
    // sample of it lands on whichever mip its own screen-space footprint
    // calls for, softening the reflection at distance and at the grazing
    // angles a river is almost always seen from instead of holding it at
    // full sharpness edge to edge.
    glBindTexture(GL_TEXTURE_2D, r.reflection_target.color);
    glGenerateMipmap(GL_TEXTURE_2D);
    r.reflection_rendered = true;
}

static void pix__post_pass(pix_renderer& r) {
    pix_bind_backbuffer(r.width, r.height);
    glDisable(GL_DEPTH_TEST);

    glUseProgram(r.post_shader);
    glUniform1i(glGetUniformLocation(r.post_shader, "uScene"), 0);
    glUniform2f(glGetUniformLocation(r.post_shader, "uTexel"),
                1.0f / (float)r.width, 1.0f / (float)r.height);
    glUniform1f(glGetUniformLocation(r.post_shader, "uExposure"), r.exposure);
    glUniform1f(glGetUniformLocation(r.post_shader, "uSaturation"), r.saturation);
    glUniform1f(glGetUniformLocation(r.post_shader, "uContrast"), r.contrast);
    glUniform1f(glGetUniformLocation(r.post_shader, "uSplitTone"), r.split_tone);
    glUniform1f(glGetUniformLocation(r.post_shader, "uVignette"), r.vignette);
    glUniform1f(glGetUniformLocation(r.post_shader, "uSharpen"), r.sharpen);
    glUniform1f(glGetUniformLocation(r.post_shader, "uBloomStrength"), r.bloom_strength);
    glUniform1f(glGetUniformLocation(r.post_shader, "uRain"), r.weather.rain);
    glUniform1f(glGetUniformLocation(r.post_shader, "uTime"), r.time);
    glUniform1i(glGetUniformLocation(r.post_shader, "uBloom"), 1);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, r.bloom_a.color);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, r.scene_target.color);
    pix_draw_fullscreen();

    glEnable(GL_DEPTH_TEST);
}

static void end_frame(pix_renderer& r, bool clear_instances) {
    mat4 vp = mat4_mul(r.projection_matrix, r.view_matrix);

    // group instances so each (mesh, material) run is one instanced draw
    pix__sort_instances(r.instances, r.renderable_count, r.camera_position);
    // and give the frame a clean instance buffer to stream into
    pix__instance_frame_begin(r);
    r.frame_draws = 0;
    r.frame_instances = 0;
    // and reduce this frame's lights to the handful a fragment will evaluate
    pix_lights_pack(r.lights, r.camera_position);
    pix__upload_lights(r);

    // The shadow pass fits the cascades, and their matrices are part of what
    // every later pass reads out of the scene block - so the block goes up
    // between the two, once, for the whole frame.
    if (r.effects) {
        pix__gpu_begin(r, PIX_GPU_SHADOW);
        pix__shadow_pass(r);
        pix__gpu_end(r, PIX_GPU_SHADOW);
    }
    pix__upload_scene(r);
    if (r.effects) {
        pix__gpu_begin(r, PIX_GPU_REFLECTION);
        pix__reflection_pass(r);
        pix__gpu_end(r, PIX_GPU_REFLECTION);
    }

    if (r.effects) pix_bind_framebuffer(r.scene_target);
    else           pix_bind_backbuffer(r.width, r.height);

    glEnable(GL_DEPTH_TEST);
    glClearColor(r.clear_color.x, r.clear_color.y, r.clear_color.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (r.effects) {
        pix__gpu_begin(r, PIX_GPU_SKY);
        pix__sky_pass(r, vp);
        pix__gpu_end(r, PIX_GPU_SKY);
    }

    pix__gpu_begin(r, PIX_GPU_OPAQUE);
    glUseProgram(r.shader);
    glUniformMatrix4fv(glGetUniformLocation(r.shader, "uViewProj"), 1, GL_FALSE, vp.data);
    glUniform1i(glGetUniformLocation(r.shader, "uTex"), 0);
    pix__set_lighting(r, r.shader);
    pix__bind_lights(r, r.shader);
    pix__bind_shadow(r, r.shader);
    pix__draw_instanced(r, r.shader, true);
    pix__gpu_end(r, PIX_GPU_OPAQUE);

    pix__gpu_begin(r, PIX_GPU_SKINNED);
    if (r.animated_count && r.skinned_shader) {
        glUseProgram(r.skinned_shader);
        glUniformMatrix4fv(glGetUniformLocation(r.skinned_shader, "uViewProj"), 1, GL_FALSE, vp.data);
        glUniform1i(glGetUniformLocation(r.skinned_shader, "uTex"), 0);
        pix__set_lighting(r, r.skinned_shader);
        pix__bind_lights(r, r.skinned_shader);
        pix__bind_shadow(r, r.skinned_shader);
        pix__draw_skinned(r, r.skinned_shader, true);
    }
    pix__gpu_end(r, PIX_GPU_SKINNED);

    pix__gpu_begin(r, PIX_GPU_WATER);
    pix__water_pass(r, vp);
    pix__gpu_end(r, PIX_GPU_WATER);

    pix__gpu_begin(r, PIX_GPU_PARTICLES);
    pix__particle_pass(r, vp);
    pix__gpu_end(r, PIX_GPU_PARTICLES);

    glBindVertexArray(0);
    if (r.effects) {
        pix__gpu_begin(r, PIX_GPU_BLOOM);
        pix__bloom_pass(r);
        pix__gpu_end(r, PIX_GPU_BLOOM);

        pix__gpu_begin(r, PIX_GPU_POST);
        pix__post_pass(r);
        pix__gpu_end(r, PIX_GPU_POST);
    }
    if (r.gpu_timing) r.gpu_slot ^= 1;

    if (clear_instances) {
        r.renderable_count = 0;
        r.animated_count = 0;
        r.water_count = 0;
        r.particle_count = 0;
    }
}
