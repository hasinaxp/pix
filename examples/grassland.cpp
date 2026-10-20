// A storybook valley: a third-person character running through meadows of
// instanced grass, forests swaying in the wind, rocky barrens, rolling hills.
//
//   biomes   two low-frequency noise fields split the land into open
//            grassland, forest (dense trees, leaf litter, thinner grass) and
//            rocky barrens (gravel, cliffs, boulders, dead trees). They are baked
//            into the same field map as height and grass density, so the
//            ground shader, the grass and the prop scatter all agree.
//
//   grass    the patch in assets/models/grass, split into its blades, shuffled
//            (so drawing the first N of them thins the patch evenly), and
//            instanced thousands of times. A compute pass culls the patches per
//            view and sorts them into three distance LODs, each one indexed
//            indirect draw: near, the modelled blades; further off, fewer and
//            wider blades of one triangle each. Nothing pops as you move:
//            blades thin out by shrinking into the ground one by one, the
//            rest widen to cover for them, and a modelled blade has already
//            morphed into its one-triangle shape where the LOD changes. Every blade is rooted on the
//            terrain and bent by the wind and by the player in the vertex
//            shader, which also colours it once per corner.
//   trees    one vertex shader sways whole trees and flutters their leaves;
//            the shadow pass runs the same shader, so the shadows move too.
//            Blows (yours as a wolf, a wolfman's) wear a tree down until it
//            snaps at the root, tips over and falls as a rigid body - bouncing
//            on its crown, glancing off trunks, crushing what it lands on -
//            and lies as a log in the way, until more blows burst it apart.
//   ground   coloured from the same field map the grass is planted from, so
//            turf sits under grass and bare earth where there is none.
//   ponds    still water in U-shaped basins carved into natural hollows, all
//            at one level so one planar mirror serves them all: the banks and
//            trees reflected, the lakebed seen through the shallows (bent by
//            the ripples), deepening in cel bands, a foam line along the
//            shore, and the grass giving way to wet mud at the water's edge.
//            You wade in the shallows; the deep middle holds you back.
//   look     grass, ground, trees, rocks and the player all share one
//            lighting function: wrapped sun and sky fill, or (toon, the
//            default) two hard tones with cool shade and a rim light. No
//            reflections anywhere - thin leaf cards and blades seen edge-on
//            would otherwise mirror the bright sky and turn white.
//   leaves   mipmaps built so each level keeps the same share of texels past
//            the alpha cut-off, so a far canopy stays as full as a near one.
//
// The player (assets/models/player/player.glb) has five in-place clips -
// Stand, Walk, Run, Jump, Roll - played through pix_anim_system with
// cross-fades. Walk/run speeds and footstep times are measured from the clips
// themselves, so the playback rate follows the ground speed and the feet do
// not skate.
//
// Wolves (assets/models/creatures/wolf_beast_1) roam in packs: resting,
// eating, wandering, and - once one spots you - howling up the pack and
// hunting: two at a time close in with claws, bites, lunges, pounces and
// running rams while the rest circle and square up, bounding in with long
// leaps. A tackle, pounce or ram starts with a flat-out charge at the target
// and springs from it, carrying the speed into the blow. The player can take the same wolf form (Q) to fight back: a claw
// combo, heavy tackles, a slam out of a leap, and a howl that drives the pack
// back. Every hit lands on the frame the clip's paws or jaws reach furthest,
// and whatever dies goes up in smoke. Wolfmen (wolf_man_1.glb), the alphas,
// roam alone: a howl from one brings the packs, and up close they circle,
// sidestep your swings and break in with claws, bites and pounces. A villager (assets/models/player/
// runner.glb) strolls near the start and runs from hunting wolves - and from
// you, as one.
//   build.bat grassland
//
// Keyboard: WASD/arrows move, Shift run, Space jump/leap, C/Ctrl roll (wolf:
//           spring back), Q change form, as a wolf J/LMB claw combo, K/E/RMB
//           tackle (ram at a run, slam in the air), H howl,
//           click to capture the mouse for mouse-look (Esc releases it),
//           right drag orbit, wheel zoom, F free-fly camera (WASD, Space/C
//           up/down, Shift fast), T time of day, G toon, 1/2 wind, B the
//           collision boxes of broken trees
// Gamepad:  left stick move (push fully to run), right stick camera,
//           A jump, B roll, Y change form, wolf: X claw, RT tackle, d-pad up howl,
//           bumpers zoom, Back free camera
#define PIX_IMPLEMENTATION
#include "pix.h"

#define TERRAIN   150.0f      // half size of the terrain
#define GRID      300         // terrain cells per side (1 m)
#define FIELD     140.0f      // half size of the planted area
#define HUMAN_RUN 3.0f     // m/s
#define WOLF_RUN  5.0f     // m/s, the packs and the player's wolf form
#define WOLF_RUN_RATE 1.1f // gallop playback at full speed
#define CHARGE_SPEED 1.5f  // a tackle's run-up, times the running speed
#define WOLF_SCALE 1.3f    // the wolves are drawn this much bigger than modelled (the player's another 10%)
#define GRASS_PATH "assets/models/grass/Grass Patch by Danni Bittman - dz_TvM39dC7.glb"

// ---- biomes: 0..1 amounts, the spawn is always open grassland ----
static float spawn_clear(float x, float z) { return smoothstepf(14.0f, 30.0f, sqrtf(x * x + z * z)); }
static float forest_amount(float x, float z) {
    return smoothstepf(0.5f, 0.6f, fbm2(x * 0.016f + 31.0f, z * 0.016f - 17.0f, 3, 41)) * spawn_clear(x, z);
}
static float rocky_amount(float x, float z) {
    return smoothstepf(0.52f, 0.62f, fbm2(x * 0.019f - 53.0f, z * 0.019f + 71.0f, 3, 43))
         * (1.0f - forest_amount(x, z)) * spawn_clear(x, z);
}

// ---- ponds ----
// A pond is a basin of still water. Its outline is a few overlapping round
// lobes melted into one (a smooth union of their distances) and roughened by
// noise at two scales, so it bays, narrows and bulges like a shore worn by
// water. The basin follows the outline wherever it goes: its depth grows with
// the distance in from the shore, steeply at first and then level - the U -
// and outside it the banks rise gently back to the land. All share one water
// level: the reflection is a single mirror plane.
#define MAX_PONDS  6
#define POND_LOBES 4
struct pond {
    float x, z, bound;          // centre, and a radius the whole shoreline lies within
    int   lobes;
    float lx[POND_LOBES], lz[POND_LOBES], lr[POND_LOBES];
    float depth, shelf;         // full depth, and how far in from the shore it is reached
    uint32_t seed;              // of its shoreline's wander
};
static pond  g_ponds[MAX_PONDS];
static int   g_pond_count;
static float g_water_level = -1e9f;

// signed metres to this pond's shore: negative out in the water
static float pond_distance(const pond& p, float x, float z) {
    float d = 1e9f;
    for (int i = 0; i < p.lobes; i++) {
        float dx = x - p.lx[i], dz = z - p.lz[i];
        float di = sqrtf(dx * dx + dz * dz) - p.lr[i];
        // melted together, not meeting at a notch
        float k = 3.0f, h = maxf(k - fabsf(d - di), 0.0f) / k;
        d = minf(d, di) - h * h * k * 0.25f;
    }
    // broad bays and headlands, and small nicks along the edge
    d += (fbm2(x * 0.085f, z * 0.085f, 2, p.seed) - 0.5f) * 4.6f
       + (fbm2(x * 0.4f, z * 0.4f, 1, p.seed + 7u) - 0.5f) * 0.9f;
    return d;
}
// metres from the nearest shore: negative out in the water
static float pond_shore(float x, float z) {
    float best = 1e9f;
    for (int i = 0; i < g_pond_count; i++) {
        const pond& p = g_ponds[i];
        float dx = x - p.x, dz = z - p.z, reach = p.bound + 12.0f;
        if (dx * dx + dz * dz > reach * reach) continue;
        best = minf(best, pond_distance(p, x, z));
    }
    return best;
}

static float terrain_base(float x, float z) {
    float h = fbm2(x * 0.025f + 100.0f, z * 0.025f + 100.0f, 4, 7) * 9.0f - 4.0f;
    h += fbm2(x * 0.12f, z * 0.12f, 2, 11) * 0.6f;
    // the barrens are broken, lumpier ground
    float rocky = rocky_amount(x, z);
    if (rocky > 0.0f) h += rocky * (fbm2(x * 0.07f + 9.0f, z * 0.07f, 3, 13) * 4.0f - 1.2f);
    return h;
}

// the land with the pond basins cut into it
static float terrain_height(float x, float z) {
    float h = terrain_base(x, z);
    for (int i = 0; i < g_pond_count; i++) {
        const pond& p = g_ponds[i];
        float dx = x - p.x, dz = z - p.z, reach = p.bound + 12.0f;
        if (dx * dx + dz * dz > reach * reach) continue;
        float sd = pond_distance(p, x, z), target;
        if (sd < 0.0f) {
            // the U: steep just inside the shore, levelling out to the bed
            float q = 1.0f - minf(-sd / p.shelf, 1.0f);
            target = g_water_level - p.depth * (1.0f - powf(q, 2.5f));
        } else {
            target = g_water_level + sd * 0.42f;                 // the bank back up to the land
        }
        // a smooth minimum, so the bank meets the land without a crease
        float k = 0.8f, m = maxf(k - fabsf(h - target), 0.0f) / k;
        h = minf(h, target) - m * m * k * 0.25f;
    }
    return h;
}

// Finds the hollows: the water level is set low in the valley's range, then
// pond sites are taken where the land all around a basin stands above it but
// not far above (a hollow, not a hilltop crater), clear of the spawn, the
// barrens and each other.
static void choose_ponds(uint32_t seed) {
    rng rg = rng_seed(seed);
    const int S = 48;
    static float heights[S * S];
    for (int i = 0; i < S * S; i++)
        heights[i] = terrain_base(-FIELD + 20.0f + (FIELD - 20.0f) * 2.0f * (i % S) / S,
                                  -FIELD + 20.0f + (FIELD - 20.0f) * 2.0f * (i / S) / S);
    qsort(heights, S * S, sizeof(float), [](const void* a, const void* b) {
        float x = *(const float*)a, y = *(const float*)b; return x < y ? -1 : (x > y ? 1 : 0); });
    g_water_level = heights[S * S * 30 / 100];
    g_pond_count = 0;
    for (int t = 0; t < 8000 && g_pond_count < 5; t++) {
        pond p = {};
        p.x = rng_range(rg, -FIELD + 32.0f, FIELD - 32.0f);
        p.z = rng_range(rg, -FIELD + 32.0f, FIELD - 32.0f);
        // a main lobe and one to three smaller ones leaning off it
        float r0 = rng_range(rg, 5.0f, 8.5f);
        p.lobes = 3 + (int)(rng_float(rg) * 2.0f) % 2;
        p.lx[0] = p.x; p.lz[0] = p.z; p.lr[0] = r0;
        p.bound = r0;
        for (int i = 1; i < p.lobes; i++) {
            float a = rng_range(rg, 0.0f, PIX_TAU), d = r0 * rng_range(rg, 0.65f, 1.25f);
            p.lx[i] = p.x + cosf(a) * d;
            p.lz[i] = p.z + sinf(a) * d;
            p.lr[i] = r0 * rng_range(rg, 0.35f, 0.75f);
            p.bound = maxf(p.bound, d + p.lr[i]);
        }
        p.bound += 3.2f;                                   // the wander of the shore
        if (sqrtf(p.x * p.x + p.z * p.z) < p.bound + 22.0f || rocky_amount(p.x, p.z) > 0.15f) continue;
        bool clear = true;
        for (int i = 0; i < g_pond_count && clear; i++) {
            float dx = p.x - g_ponds[i].x, dz = p.z - g_ponds[i].z, need = p.bound + g_ponds[i].bound + 12.0f;
            clear = dx * dx + dz * dz > need * need;
        }
        if (!clear) continue;
        // the land over the whole basin and its banks: above the water, and a hollow
        float lowest = 1e9f, sum = 0.0f;
        int n = 0;
        for (int ring = 0; ring <= 5; ring++)
            for (int a = 0; a < 28; a++) {
                float r = (p.bound + 4.0f) * ring / 5.0f, an = PIX_TAU * a / 28.0f;
                float h = terrain_base(p.x + cosf(an) * r, p.z + sinf(an) * r);
                lowest = minf(lowest, h);
                sum += h;
                n++;
                if (ring == 0) break;
            }
        if (lowest < g_water_level + 0.35f || sum / n > g_water_level + 2.2f) continue;
        p.depth = rng_range(rg, 1.3f, 2.2f);
        p.shelf = rng_range(rg, 2.8f, 4.2f);
        p.seed = 900u + (uint32_t)t;
        g_ponds[g_pond_count++] = p;
    }
}
// 0 bare earth .. 1 thick grass
static float grass_density(float x, float z) {
    float d = smoothstepf(0.2f, 0.42f, fbm2(x * 0.03f, z * 0.03f, 3, 21));
    return d * (1.0f - 0.97f * rocky_amount(x, z)) * (1.0f - 0.3f * forest_amount(x, z))
             * smoothstepf(0.6f, 3.0f, pond_shore(x, z));        // mud at the water's edge
}

// ============================================================================
// ---- shared look ----
//
// Spliced into the grass and ground fragment shaders (after the lighting
// library), so both are lit by exactly the same model.
#define LOOK_GLSL R"(
uniform float uLookToon;     // 0 soft shading .. 1 cel shading
float look_shadow(vec3 world, vec3 n) { return sun_shadow(world, n, max(dot(n, normalize(uSunDir)), 0.2)); }
// `shadow` from look_shadow: taken once by the caller, since grass reuses it
vec3 look_shade(vec3 n, vec3 albedo, float ao, float wrap, float shadow) {
    vec3 l = normalize(uSunDir);
    float diff = clamp((dot(n, l) + wrap) / (1.0 + wrap), 0.0, 1.0);
    float lit = diff * shadow;
    // cel: two flat tones, lit or in shade, with only a sliver of a soft edge
    float w = max(fwidth(lit), 0.01);
    lit = mix(lit, smoothstep(0.33 - w, 0.33 + w, lit), uLookToon);
    // and colour, not just darkness, tells them apart: cool shade, warm light
    vec3 amb = mix(uGroundColor, uSkyColor, n.y * 0.5 + 0.5) * ao * mix(vec3(1.0), vec3(0.78, 0.92, 1.35), uLookToon);
    vec3 sun = uSunColor * lit / PI * mix(vec3(1.0), vec3(1.06, 1.0, 0.9), uLookToon);
    return albedo * (sun + amb);
}
// a thin bright outline where a surface turns away from the eye (cel only)
vec3 look_rim(vec3 n, vec3 v, float strength) {
    float rim = 1.0 - max(dot(n, v), 0.0);
    float w = max(fwidth(rim), 1e-3);
    return (uSkyColor + uSunColor * 0.08) * smoothstep(0.62 - w, 0.62 + w, rim) * strength * uLookToon;
}
)"

// ============================================================================
// ---- ground ----
static const char* GROUND_FS = "#version 440 core\n" LOOK_GLSL R"(
in vec3 vNormal;
in vec2 vUV;
in vec3 vWorld;
in vec4 vTint;
out vec4 frag;
uniform sampler2D uTex1;     // field map: r height, g grass density, b forest, a rocky
uniform sampler2D uTex2;     // metres from the nearest pond's shore, negative under the water
uniform vec4 uParams0;       // x uv offset, y uv scale
void main() {
    clip_test(vWorld);
    vec2 fuv = vWorld.xz * uParams0.y + uParams0.x;
    vec4 f = texture(uTex1, fuv);
    float shore = texture(uTex2, fuv).r;
    float dens = f.g, forest = f.b, rocky = f.a;
    float nz = value_noise(vWorld.xz * 0.4) * 0.6 + value_noise(vWorld.xz * 2.1) * 0.4;
    vec3 n = normalize(vNormal);
    vec3 dirt   = mix(vec3(0.11, 0.09, 0.045), vec3(0.17, 0.14, 0.07), nz);
    vec3 turf   = mix(vec3(0.035, 0.09, 0.016), vec3(0.06, 0.13, 0.022), nz);
    vec3 litter = mix(vec3(0.045, 0.04, 0.018), vec3(0.10, 0.07, 0.03), nz);     // needles and old leaves
    vec3 gravel = mix(vec3(0.10, 0.09, 0.075), vec3(0.19, 0.17, 0.13), value_noise(vWorld.xz * 1.3));
    // earth exactly where no grass is planted (density < 0.12), turf under it
    vec3 albedo = mix(dirt, turf, smoothstep(0.09, 0.3, dens));
    albedo = mix(albedo, litter, smoothstep(0.2, 0.7, forest) * 0.8);
    // the barrens, and any slope too steep to hold soil, are bare stone
    float stone = max(smoothstep(0.25, 0.7, rocky), smoothstep(0.3, 0.5, 1.0 - n.y));
    albedo = mix(albedo, gravel, stone);
    // a pond's bed of silt and sand, darker the deeper, and a wet dark band
    // along the bank above the waterline
    float under = 1.0 - smoothstep(-0.3, 0.1, shore);
    vec3 bed = mix(vec3(0.20, 0.17, 0.10), vec3(0.08, 0.09, 0.05), smoothstep(0.0, 3.0, -shore)) * (0.85 + 0.3 * nz);
    albedo = mix(albedo, bed, under);
    albedo *= 1.0 - 0.45 * (1.0 - smoothstep(0.0, 1.4, shore)) * (1.0 - under);
    frag = vec4(apply_fog(look_shade(n, albedo, 1.0, 0.25, look_shadow(vWorld, n)), vWorld), 1.0);
}
)";

// ============================================================================
// ---- pond water ----
//
// The look of the grass and trees carried onto water: lit by the same cel
// model, the lakebed seen through the shallows and bent by the ripples,
// swallowed by the deep colour in hard bands, the banks and sky mirrored more
// the flatter you look, a foam line where it thins against the shore and a
// hard glint of the sun. Material: uColor shallow, uParams0.rgb deep and .w
// the depth where it is fully deep, uParams1 waves (scale, speed, foam, tilt).
static const char* POND_FS = "#version 440 core\n" LOOK_GLSL R"(
void main() {
    vec2 p = vWorld.xz / wave_scale();
    float t = uTime * (uParams1.y > 0.0 ? uParams1.y : 1.0);
    vec3 n = water_normal(p, t);
    vec3 v = normalize(uCameraPos - vWorld);
    vec3 up = vec3(0.0, 1.0, 0.0);
    vec2 uv = screen_uv();
    float surface = fragment_depth();

    // what lies under the water, bent by the ripples - never something in front of it
    float here = water_thickness(uv);
    vec2 ruv = clamp(uv + n.xz * uRefraction * min(here, 1.0) * 4.0, 0.002, 0.998);
    if (scene_depth_at(ruv) < surface) ruv = uv;
    vec3 under = texture(uRefractMap, ruv).rgb;

    // the deeper, the more the bed is swallowed by the deep colour - in cel
    // bands, measured straight through (the bent look-up would break the bands up)
    float depth_scale = uParams0.w > 0.0 ? uParams0.w : 1.5;
    float d = 1.0 - exp(-here / depth_scale);
    float band = mix(d, floor(d * 4.0 + 0.35) / 4.0, uLookToon);
    vec3 shallow = srgb_to_linear(uColor.rgb), deep = srgb_to_linear(uParams0.rgb);
    float sh = look_shadow(vWorld, up);
    vec3 body = look_shade(up, mix(shallow, deep, band), 1.0, 0.5, sh);
    vec3 col = mix(under * mix(vec3(1.0), shallow * 1.8, 0.45), body, clamp(band * 1.1 + 0.15, 0.0, 1.0));

    // the banks and the sky mirrored, more of them the flatter the view
    float fres = 0.03 + 0.97 * pow(1.0 - max(dot(n, v), 0.0), 5.0);
    fres = mix(fres, smoothstep(0.08, 0.45, fres) * 0.75 + 0.08, uLookToon * 0.6);
    vec3 refl = (uPlanar > 0.0 && bReflect.x > 0.5) ? planar_reflection(vWorld, n, 0.06) : sky_env(reflect(-v, n));
    col = mix(col, refl, clamp(fres * uPlanar + fres * 0.2, 0.0, 0.85));

    // a broken line of foam where the water thins out against the shore,
    // drifting along it in clumps rather than ringing the pond
    float wob = value_noise(p * 2.5 + vec2(t * 0.4, -t * 0.25)) * 0.1;
    float foam = smoothstep(0.02, 0.08, here) * (1.0 - smoothstep(0.12 + wob, 0.26 + wob, here)) * uParams1.z;
    foam *= smoothstep(0.4, 0.62, value_noise(p * 4.0 + vec2(t * 0.2, t * 0.13)));
    col = mix(col, look_shade(up, vec3(0.7, 0.8, 0.78), 1.0, 0.5, sh), foam * 0.35);
    // right at the waterline the water is a film over the mud: no hard edge
    col = mix(under, col, smoothstep(0.0, 0.05, here));

    // a hard cel glint of the sun
    vec3 h = normalize(normalize(uSunDir) + v);
    col += uSunColor * step(0.993, dot(n, h)) * 0.9 * sh;
    frag = vec4(apply_fog(col, vWorld), 1.0);
}
)";

// ============================================================================
// ---- grass ----

struct grass_vertex {
    float pos[3];
    float normal[3];
    float root[4];     // xyz the blade's root in patch space, w its height
    float info[4];     // x random 0..1, y 0 root .. 1 tip, z shade (which of the model's materials),
                       // w rank: the blade's place in the thinning order, 0..1
    float morph[3];    // where this corner lies on the blade's one-triangle shape
    float morph_n[3];  // and the one normal that shape has
};

struct gpu_patch {     // std430, 32 bytes
    float a[4];        // x, z, yaw, random
    float b[4];        // x horizontal scale, y vertical scale
};

static const char* GRASS_CULL = R"(
layout(local_size_x = 64) in;
struct Patch { vec4 a; vec4 b; };
layout(std430, binding = 0) readonly buffer Patches { Patch patches[]; };
layout(std430, binding = 1) writeonly buffer Visible { uint visible[]; };
struct Cmd { uint count; uint instances; uint first; int base_vertex; uint base_instance; };
layout(std430, binding = 2) buffer Args { Cmd cmd[3]; };
uniform vec4  uPlanes[6];
uniform vec3  uEye;
uniform vec3  uLod;          // LOD 0 / LOD 1 / draw distance
uniform uint  uCapacity;
uniform sampler2D uField;
uniform vec2  uFieldXf;      // uv = xz * y + x

void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= patches.length()) return;
    Patch p = patches[i];
    float ground = textureLod(uField, p.a.xy * uFieldXf.y + uFieldXf.x, 0.0).r;
    vec3 c = vec3(p.a.x, ground + 0.35 * p.b.y, p.a.y);
    float r = 1.9 * p.b.x;
    for (int k = 0; k < 6; k++) if (dot(uPlanes[k].xyz, c) + uPlanes[k].w < -r) return;
    // dithered per patch, so a LOD change is a soft band rather than a line
    float d = distance(c, uEye) * (0.85 + 0.3 * p.a.w);
    if (d > uLod.z) return;
    uint lod = d < uLod.x ? 0u : (d < uLod.y ? 1u : 2u);
    uint slot = atomicAdd(cmd[lod].instances, 1u);
    visible[lod * uCapacity + slot] = i;
}
)";

static const char* GRASS_VS = R"(#version 440 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aRoot;
layout(location = 3) in vec4 aInfo;
layout(location = 4) in vec3 aMorph;
layout(location = 5) in vec3 aMorphN;
struct Patch { vec4 a; vec4 b; };
layout(std430, binding = 0) readonly buffer Patches { Patch patches[]; };
layout(std430, binding = 1) readonly buffer Visible { uint visible[]; };
uniform int   uSeg;
uniform uint  uCapacity;
uniform sampler2D uField;
uniform vec2  uFieldXf;
uniform vec3  uWind;         // xy direction, z strength
uniform vec4  uPushers[8];   // xyz feet of whoever walks through, w radius
uniform int   uPusherCount;
uniform float uWidth;        // blade width, times the model's
uniform vec3  uLod;          // where the near LOD ends, the middle one, and the grass
uniform vec3  uLodEye;       // the eye the cull measured from
out vec3 vWorld;
out vec3 vNormal;
out vec3 vAlbedo;
out float vAO;
out float vT;
out float vGust;

float vhash(vec2 p) { vec3 q = fract(vec3(p.xyx) * 0.1031); q += dot(q, q.yzx + 33.33); return fract((q.x + q.y) * q.z); }
float vnoise(vec2 p) {
    vec2 i = floor(p), f = fract(p), u = f * f * (3.0 - 2.0 * f);
    return mix(mix(vhash(i), vhash(i + vec2(1, 0)), u.x), mix(vhash(i + vec2(0, 1)), vhash(i + vec2(1, 1)), u.x), u.y);
}

// The share of a patch's blades kept at distance d (the cull's own, dithered
// per patch): falling smoothly from all the near LOD holds to what the middle
// one holds where it takes over, to what the far one holds, to none at the end.
float kept(float d) {
    if (d < uLod.x * 0.6) return 0.72;
    if (d < uLod.x) return mix(0.72, 0.40, (d - uLod.x * 0.6) / (uLod.x * 0.4));
    if (d < uLod.y) return mix(0.40, 0.16, (d - uLod.x) / (uLod.y - uLod.x));
    return mix(0.16, 0.0, clamp((d - uLod.y) / (uLod.z - uLod.y), 0.0, 1.0));
}

void main() {
    Patch p = patches[visible[uint(uSeg) * uCapacity + uint(gl_InstanceID)]];
    float c = cos(p.a.z), s = sin(p.a.z);
    mat2 rot = mat2(c, s, -s, c);
    vec2 root = p.a.xy + rot * (aRoot.xz * p.b.x);
    float ground = textureLod(uField, root * uFieldXf.y + uFieldXf.x, 0.0).r;

    // how far off this patch is, exactly as the cull measured it
    float centre = textureLod(uField, p.a.xy * uFieldXf.y + uFieldXf.x, 0.0).r;
    float dist = distance(vec3(p.a.x, centre + 0.35 * p.b.y, p.a.y), uLodEye) * (0.85 + 0.3 * p.a.w);
    float keep = kept(dist);
    // a blade past the kept share shrinks into the ground rather than vanishing
    float grow = 1.0 - smoothstep(keep - 0.06, keep, aInfo.w);
    // the modelled blade becomes its one-triangle self before the middle LOD takes over
    float morph = uSeg == 0 ? smoothstep(uLod.x * 0.65, uLod.x, dist) : 0.0;
    vec3 local = (mix(aPos, aMorph, morph) - aRoot.xyz) * grow;
    vec3 normal = normalize(mix(aNormal, aMorphN, morph));
    // widened across the blade's face only (as tall as modelled, just fuller),
    // and more as the grass thins out, so the meadow covers as much
    float widen = uWidth * clamp(sqrt(0.72 / max(keep, 0.05)), 1.0, 2.2);
    vec3 across = cross(normal, vec3(0.0, 1.0, 0.0));
    float al = length(across);
    if (al > 1e-3) { across /= al; local += across * dot(local, across) * (widen - 1.0); }
    local.xz = rot * (local.xz * p.b.x);
    local.y *= p.b.y;
    float h = max(aRoot.w * p.b.y * grow, 0.05);
    float t = aInfo.y;

    // gusts roll across the field as broad bands, with a quick flutter on top
    vec2 wd = uWind.xy;
    float gust = vnoise(root * 0.04 - wd * uTime * 0.5) * 0.75 + vnoise(root * 0.13 - wd * uTime * 1.2) * 0.25;
    float flutter = sin(uTime * 3.3 + aInfo.x * 6.283 + dot(root, vec2(0.7, 0.9)));
    vec2 bend = wd * uWind.z * (0.1 + gust * 0.9) + vec2(-wd.y, wd.x) * flutter * 0.12 * uWind.z;

    // the player, the wolves and the villager part the grass around them
    for (int k = 0; k < uPusherCount; k++) {
        vec4 P = uPushers[k];
        vec2 away = root - P.xz;
        float dist = length(away);
        float push = (1.0 - smoothstep(P.w * 0.3, P.w, dist)) * step(abs(ground - P.y), 1.2);
        bend += (dist > 1e-3 ? away / dist : vec2(1.0, 0.0)) * push * 0.9;
    }

    vec2 off = bend * h * t * t;
    float len = length(off), max_len = h * 0.9;
    if (len > max_len) off *= max_len / len;
    local.xz += off;
    local.y *= sqrt(max(1.0 - dot(off, off) / (h * h), 0.15));   // keep the blade's length

    vWorld = vec3(root.x, ground, root.y) + local;
    vec3 n = normal;
    n.xz = rot * n.xz;
    vNormal = n;
    vT = t;
    vGust = smoothstep(0.45, 0.85, gust) * uWind.z;

    // broad drifts of cooler and warmer green, and here and there a dry patch -
    // one colour per blade, worked out per corner rather than per pixel
    float broad = vnoise(root * 0.03);
    float fine = vnoise(root * 0.17);
    vec3 tip = mix(vec3(0.11, 0.25, 0.03), vec3(0.06, 0.20, 0.045), smoothstep(0.3, 0.75, broad));
    tip = mix(tip, vec3(0.24, 0.26, 0.05), smoothstep(0.62, 0.9, fine) * 0.5);
    tip *= 0.8 + 0.4 * aInfo.x;
    vec3 base = mix(vec3(0.03, 0.085, 0.016), tip, 0.25);    // the base is shaded, but still green
    vAlbedo = mix(base, tip, smoothstep(0.0, 0.8, t)) * mix(1.0, 0.88, aInfo.z);
    vAO = mix(0.62, 1.0, smoothstep(0.0, 0.6, t));
    gl_Position = uViewProj * vec4(vWorld, 1.0);
}
)";

static const char* GRASS_FS = "#version 440 core\n" LOOK_GLSL R"(
in vec3 vWorld;
in vec3 vNormal;
in vec3 vAlbedo;
in float vAO;
in float vT;
in float vGust;
out vec4 frag;
void main() {
    vec3 up = vec3(0.0, 1.0, 0.0);
    vec3 v = normalize(uCameraPos - vWorld);
    vec3 bn = normalize(vNormal);
    if (dot(bn, v) < 0.0) bn = -bn;
    // mostly the ground's normal: a meadow reads as one soft surface, not as
    // thousands of flat-shaded cards
    vec3 n = normalize(mix(up, bn, 0.3));

    vec3 albedo = vAlbedo;
    float ao = vAO;

    float shadow = look_shadow(vWorld, up);
    vec3 col = look_shade(n, albedo, ao, 0.5, shadow);
    // a gust passing over bends the blades to show their paler sides: the wind
    // becomes visible as bright waves rolling across the meadow
    col += uSunColor * albedo * vGust * vT * 0.28 * shadow;
    // sunlight through the blades when looking toward the sun
    float through = pow(max(dot(-v, normalize(uSunDir)), 0.0), 6.0) * vT;
    col += uSunColor * albedo * through * 0.5 * shadow;
    frag = vec4(apply_fog(col, vWorld), 1.0);
}
)";

#define GRASS_LODS 3

struct grass {
    pix_gpu_buffer patches, visible, args;
    uint32_t patch_count;
    GLuint vao, vbo, ibo, field;
    idx cull, draw;
    uint32_t lod_indices[GRASS_LODS], lod_first[GRASS_LODS];   // each LOD's run of the index buffer
    float lod_dist[GRASS_LODS];
    vec2 field_xf;
    vec3 wind;
    vec4 pushers[8];
    int pusher_count;
    float width;
    float toon;
};

// runs inside every main view, with the scene block and shadow map bound
static void grass_hook(pix_renderer& r, const pix_hook_context& ctx, void* user) {
    (void)r;
    grass& g = *(grass*)user;
    if (ctx.pass != PIX_HOOK_OPAQUE || !g.patch_count) return;
    // the main view, and the pond mirror (with only the one-triangle blades)
    bool mirror = ctx.view_kind == PIX_VIEW_MIRROR;
    if (ctx.view_kind != PIX_VIEW_USER && !mirror) return;

    pix_draw_elements_indirect reset[GRASS_LODS];
    for (int k = 0; k < GRASS_LODS; k++) reset[k] = { g.lod_indices[k], 0, g.lod_first[k], 0, 0 };
    pix_update_gpu_buffer(g.args, reset, sizeof(reset));
    pix_bind_storage(g.patches, 0);
    pix_bind_storage(g.visible, 1);
    pix_bind_storage(g.args, 2);
    glActiveTexture(GL_TEXTURE0 + 10);
    glBindTexture(GL_TEXTURE_2D, g.field);
    glActiveTexture(GL_TEXTURE0);

    glUseProgram(g.cull);
    glUniform4fv(glGetUniformLocation(g.cull, "uPlanes"), 6, &ctx.frustum.planes[0].x);
    vec3 eye = ctx.eye;
    glUniform3f(glGetUniformLocation(g.cull, "uEye"), eye.x, eye.y, eye.z);
    glUniform3f(glGetUniformLocation(g.cull, "uLod"), mirror ? 0.0f : g.lod_dist[0], g.lod_dist[1], mirror ? 45.0f : g.lod_dist[2]);
    glUniform1ui(glGetUniformLocation(g.cull, "uCapacity"), g.patch_count);
    glUniform1i(glGetUniformLocation(g.cull, "uField"), 10);
    glUniform2f(glGetUniformLocation(g.cull, "uFieldXf"), g.field_xf.x, g.field_xf.y);
    pix_dispatch_for(g.cull, g.patch_count, 64);
    pix_compute_barrier(PIX_BARRIER_STORAGE | PIX_BARRIER_INDIRECT);

    glUseProgram(g.draw);
    glUniform1ui(glGetUniformLocation(g.draw, "uCapacity"), g.patch_count);
    glUniform1i(glGetUniformLocation(g.draw, "uField"), 10);
    glUniform2f(glGetUniformLocation(g.draw, "uFieldXf"), g.field_xf.x, g.field_xf.y);
    glUniform3f(glGetUniformLocation(g.draw, "uWind"), g.wind.x, g.wind.y, g.wind.z);
    glUniform4fv(glGetUniformLocation(g.draw, "uPushers"), 8, &g.pushers[0].x);
    glUniform1i(glGetUniformLocation(g.draw, "uPusherCount"), g.pusher_count);
    glUniform1f(glGetUniformLocation(g.draw, "uWidth"), g.width);
    glUniform1f(glGetUniformLocation(g.draw, "uLookToon"), g.toon);
    GLint seg = glGetUniformLocation(g.draw, "uSeg");
    // the same distances the cull sorted by: the blades thin and morph along them
    glUniform3f(glGetUniformLocation(g.draw, "uLod"), mirror ? 0.0f : g.lod_dist[0], g.lod_dist[1], mirror ? 45.0f : g.lod_dist[2]);
    glUniform3f(glGetUniformLocation(g.draw, "uLodEye"), eye.x, eye.y, eye.z);
    glDisable(GL_CULL_FACE);                                   // blades have two sides
    glBindVertexArray(g.vao);
    glBindBuffer(GL_DRAW_INDIRECT_BUFFER, g.args.id);
    for (int k = 0; k < GRASS_LODS; k++) {
        glUniform1i(seg, k);
        glDrawElementsIndirect(GL_TRIANGLES, GL_UNSIGNED_INT, (const void*)(size_t)(k * sizeof(pix_draw_elements_indirect)));
    }
    glBindBuffer(GL_DRAW_INDIRECT_BUFFER, 0);
    glBindVertexArray(0);
}

// Splits the grass model into blades - runs of triangles that share corners -
// and shuffles them, so the first N blades are an even thinning of the patch.
// qsort comparators for the blade split (qsort takes no context pointer)
static const vertex_rigged* g_sort_verts;
static const int* g_sort_key;
static int by_position(const void* a, const void* b) {
    vec3 p = g_sort_verts[*(const int*)a].position, q = g_sort_verts[*(const int*)b].position;
    if (p.x != q.x) return p.x < q.x ? -1 : 1;
    if (p.y != q.y) return p.y < q.y ? -1 : 1;
    return p.z < q.z ? -1 : (p.z > q.z ? 1 : 0);
}
static int by_key(const void* a, const void* b) {
    int x = g_sort_key[*(const int*)a], y = g_sort_key[*(const int*)b];
    return x < y ? -1 : (x > y ? 1 : 0);
}

static bool load_grass_blades(grass& g, rng& rg) {
    pix_data_loader& l = pix_get_loader();
    pix_loader_mark mark = pix_loader_mark_now(l);
    int mats[4];
    int mc = gltf_list_materials(l.arena, GRASS_PATH, -1, mats, 4);
    if (mc <= 0) { mats[0] = -1; mc = 1; }

    // a blade is a set of triangles joined through shared corners: union-find
    // over corner positions, then each set's triangles gathered together
    struct blade { const vertex_rigged* v; uint32_t* tri; int tris; float shade; };
    const int MAX_BLADES = 4096;
    blade* blades = pix_alloc<blade>(MAX_BLADES);
    int count = 0;
    for (int m = 0; m < mc; m++) {
        idx id = load_model_gltf_file(l, GRASS_PATH, -1, mats[m]);
        skinned_mesh_file_data* md = id == PIX_INVALID ? 0 : get_model_mesh(l, id);
        if (!md) continue;
        const vertex_rigged* v = md->vertex_data;
        const uint32_t* ind = md->index_data;
        int nv = (int)md->vertex_count, tris = (int)(md->index_count / 3);
        int* parent = allocate<int>(l.arena, nv);
        int* order = allocate<int>(l.arena, nv);
        for (int i = 0; i < nv; i++) parent[i] = order[i] = i;
        auto find = [&](int x) { while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; };
        auto join = [&](int a2, int b2) { a2 = find(a2); b2 = find(b2); if (a2 != b2) parent[b2] = a2; };
        // weld corners at the same spot: sort by position, join neighbours that match
        g_sort_verts = v;
        qsort(order, nv, sizeof(int), by_position);
        for (int i = 1; i < nv; i++)
            if (v3dist(v[order[i]].position, v[order[i - 1]].position) < 1e-5f) join(order[i], order[i - 1]);
        for (int t = 0; t < tris; t++) { join(ind[t * 3], ind[t * 3 + 1]); join(ind[t * 3], ind[t * 3 + 2]); }
        // bucket triangles by their set
        int* tri_of = allocate<int>(l.arena, tris);
        for (int t = 0; t < tris; t++) tri_of[t] = t;
        int* set_of = allocate<int>(l.arena, tris);
        for (int t = 0; t < tris; t++) set_of[t] = find(ind[t * 3]);
        g_sort_key = set_of;
        qsort(tri_of, tris, sizeof(int), by_key);
        uint32_t* sorted = allocate<uint32_t>(l.arena, tris * 3);
        for (int t = 0; t < tris; t++)
            for (int k = 0; k < 3; k++) sorted[t * 3 + k] = ind[tri_of[t] * 3 + k];
        for (int t = 0; t < tris && count < MAX_BLADES; ) {
            int e = t;
            while (e < tris && set_of[tri_of[e]] == set_of[tri_of[t]]) e++;
            blades[count++] = { v, sorted + t * 3, e - t, (float)m };
            t = e;
        }
    }
    if (!count) { pix_free(blades); pix_loader_rewind(l, mark); return false; }
    for (int i = count - 1; i > 0; i--) {                    // Fisher-Yates
        int j = (int)(rng_float(rg) * (i + 1)) % (i + 1);
        blade tmp = blades[i]; blades[i] = blades[j]; blades[j] = tmp;
    }

    // Two sets of geometry in one buffer. The modelled blades, indexed: a
    // blade's corners are shared by its triangles, so each is shaded once,
    // not once per triangle. Then every blade again as a single tapered
    // triangle - base corners and tip - for the LODs where a blade is a few
    // pixels tall. A LOD draws the first fraction of the (shuffled) blades.
    int full_verts = 0, full_indices = 0;
    for (int i = 0; i < count; i++) full_indices += blades[i].tris * 3;
    int max_verts = full_indices + count * 3;
    grass_vertex* out = pix_alloc<grass_vertex>(max_verts);
    uint32_t* indices = pix_alloc<uint32_t>(full_indices + count * 3);
    uint32_t* blade_end = pix_alloc<uint32_t>(count);         // index count after each full blade
    int* blade_verts = pix_alloc<int>(count + 1);             // first corner of each full blade
    int n = 0, ni = 0;
    struct simple { vec3 left, right, tip, normal, lo; float h, rnd, shade; };
    simple* simples = pix_alloc<simple>(count);
    for (int i = 0; i < count; i++) {
        const blade& b = blades[i];
        // the blade's facing: its normals summed, each turned to agree with the
        // first - a blade modelled as two faces back to back would cancel out
        vec3 lo = v3(0, 1e9f, 0), tip = v3(0, -1e9f, 0), normal = v3(0, 0, 0);
        vec3 face = b.v[b.tri[0]].normal;
        for (int q = 0; q < b.tris * 3; q++) {
            vec3 p = b.v[b.tri[q]].position, nq = b.v[b.tri[q]].normal;
            if (p.y < lo.y) lo = p;
            if (p.y > tip.y) tip = p;
            normal += v3dot(nq, face) < 0.0f ? nq * -1.0f : nq;
        }
        float h = maxf(tip.y - lo.y, 0.01f), rnd = rng_float(rg);
        // the blade's corners, each spot once: the model splits them per face
        // for flat shading, but a blade is lit mostly by the ground's normal,
        // so welded corners with averaged normals look the same and are shaded
        // once instead of up to six times
        int first = n;
        blade_verts[i] = n;
        for (int q = 0; q < b.tris * 3; q++) {
            const vertex_rigged& sv = b.v[b.tri[q]];
            int found = -1;
            for (int k = first; k < n && found < 0; k++)
                if (fabsf(out[k].pos[0] - sv.position.x) + fabsf(out[k].pos[1] - sv.position.y) + fabsf(out[k].pos[2] - sv.position.z) < 1e-5f)
                    found = k;
            if (found < 0) {
                grass_vertex& d = out[n];
                d.pos[0] = sv.position.x; d.pos[1] = sv.position.y; d.pos[2] = sv.position.z;
                d.normal[0] = d.normal[1] = d.normal[2] = 0.0f;
                d.root[0] = lo.x; d.root[1] = lo.y; d.root[2] = lo.z; d.root[3] = h;
                d.info[0] = rnd; d.info[1] = clampf((sv.position.y - lo.y) / h, 0.0f, 1.0f); d.info[2] = b.shade;
                d.info[3] = (i + 0.5f) / count;           // its rank: the order the LODs cut it off in
                found = n++;
            }
            vec3 nq = v3dot(sv.normal, face) < 0.0f ? sv.normal * -1.0f : sv.normal;   // same side as the blade
            out[found].normal[0] += nq.x; out[found].normal[1] += nq.y; out[found].normal[2] += nq.z;
            indices[ni++] = (uint32_t)found;
        }
        for (int k = first; k < n; k++) {
            vec3 nn = v3(out[k].normal[0], out[k].normal[1], out[k].normal[2]);
            float l = v3len(nn);
            nn = l > 1e-6f ? nn * (1.0f / l) : face;
            out[k].normal[0] = nn.x; out[k].normal[1] = nn.y; out[k].normal[2] = nn.z;
        }
        blade_end[i] = (uint32_t)ni;
        // its base: the two lowest corners furthest apart
        vec3 left = lo, right = lo;
        float widest = -1.0f;
        for (int q = 0; q < b.tris * 3; q++) {
            vec3 p = b.v[b.tri[q]].position;
            if (p.y > lo.y + 0.2f * h) continue;
            for (int r2 = 0; r2 < b.tris * 3; r2++) {
                vec3 o = b.v[b.tri[r2]].position;
                if (o.y > lo.y + 0.2f * h) continue;
                float d2 = (p.x - o.x) * (p.x - o.x) + (p.z - o.z) * (p.z - o.z);
                if (d2 > widest) { widest = d2; left = p; right = o; }
            }
        }
        float nl = v3len(normal);
        simples[i] = { left, right, tip, nl > 1e-5f ? normal * (1.0f / nl) : v3(0, 0, 1), lo, h, rnd, b.shade };
        // Each corner's place on that triangle: at its own height up the blade,
        // as far to the side as it is across the blade there. Morphing to it
        // gives the one-triangle blade the middle LOD draws.
        vec3 base = (left + right) * 0.5f, dir = right - left;
        dir.y = 0.0f;
        float half = v3len(dir) * 0.5f;
        dir = half > 1e-5f ? dir * (0.5f / half) : v3(1, 0, 0);
        for (int k = first; k < n; k++) {
            vec3 q = v3(out[k].pos[0], out[k].pos[1], out[k].pos[2]);
            float t = clampf((q.y - lo.y) / h, 0.0f, 1.0f);
            vec3 axis = base + (tip - base) * t;
            float wide = half * (1.0f - t);
            float a = wide > 1e-4f ? clampf(v3dot(q - axis, dir) / wide, -1.0f, 1.0f) : 0.0f;
            vec3 m = axis + dir * (a * wide);
            out[k].morph[0] = m.x; out[k].morph[1] = m.y; out[k].morph[2] = m.z;
            vec3 bn = simples[i].normal;
            out[k].morph_n[0] = bn.x; out[k].morph_n[1] = bn.y; out[k].morph_n[2] = bn.z;
        }
    }
    full_verts = n;
    int simple_first = ni;
    for (int i = 0; i < count; i++) {
        const simple& sb = simples[i];
        const vec3 corner[3] = { sb.left, sb.right, sb.tip };
        for (int c = 0; c < 3; c++) {
            grass_vertex& d = out[n];
            d.pos[0] = corner[c].x; d.pos[1] = corner[c].y; d.pos[2] = corner[c].z;
            d.normal[0] = sb.normal.x; d.normal[1] = sb.normal.y; d.normal[2] = sb.normal.z;
            d.root[0] = sb.lo.x; d.root[1] = sb.lo.y; d.root[2] = sb.lo.z; d.root[3] = sb.h;
            d.info[0] = sb.rnd; d.info[1] = c == 2 ? 1.0f : 0.0f; d.info[2] = sb.shade;
            d.info[3] = (i + 0.5f) / count;
            d.morph[0] = corner[c].x; d.morph[1] = corner[c].y; d.morph[2] = corner[c].z;
            d.morph_n[0] = sb.normal.x; d.morph_n[1] = sb.normal.y; d.morph_n[2] = sb.normal.z;
            indices[ni++] = (uint32_t)n++;
        }
    }
    // near: most modelled blades; middle and far: fewer one-triangle blades
    // (the shares kept() in GRASS_VS starts each LOD from)
    const float fraction[GRASS_LODS] = { 0.72f, 0.4f, 0.16f };
    for (int k = 0; k < GRASS_LODS; k++) {
        int keep = (int)(count * fraction[k]) > 1 ? (int)(count * fraction[k]) : 1;
        g.lod_first[k] = k == 0 ? 0u : (uint32_t)simple_first;
        g.lod_indices[k] = k == 0 ? blade_end[keep - 1] : (uint32_t)(keep * 3);
    }
    int total = n;

    glGenVertexArrays(1, &g.vao);
    glGenBuffers(1, &g.vbo);
    glBindVertexArray(g.vao);
    glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(grass_vertex) * total, out, GL_STATIC_DRAW);
    glGenBuffers(1, &g.ibo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g.ibo);                 // part of the VAO
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * ni, indices, GL_STATIC_DRAW);
    const GLsizei stride = sizeof(grass_vertex);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(grass_vertex, pos));
    glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(grass_vertex, normal));
    glEnableVertexAttribArray(2); glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(grass_vertex, root));
    glEnableVertexAttribArray(3); glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(grass_vertex, info));
    glEnableVertexAttribArray(4); glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(grass_vertex, morph));
    glEnableVertexAttribArray(5); glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, stride, (void*)offsetof(grass_vertex, morph_n));
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    pix_log("grass: %d blades per patch, %d triangles over %d corners (were %d); LOD corners %u / %u / %u",
            count, full_indices / 3, full_verts, full_indices, g.lod_indices[0], g.lod_indices[1], g.lod_indices[2]);
    pix_free(out);
    pix_free(indices);
    pix_free(blade_end);
    pix_free(blade_verts);
    pix_free(simples);
    pix_free(blades);
    pix_loader_rewind(l, mark);
    return true;
}

// ============================================================================
// ---- trees and rocks from assets/models ----
//
// A tree is two materials - bark and alpha-cut leaves - and a merged draw can
// only wear one texture, so each material is loaded as its own part. Every
// file of a species embeds the same textures again; they are uploaded once and
// shared, keyed by their contents.

// The whole tree leans with the gusts (more the higher up), leaves flutter on
// top. uParams0: x wind strength, y leaf flutter (0 bark, 1 leaves), z..w the
// model heights drawn (the whole tree, or one piece of a broken one).
static const char* TREE_VS = R"(#version 440 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;
layout(location = 3) in mat4 aModel;
layout(location = 7) in vec4 aTint;
out vec3 vNormal;
out vec2 vUV;
out vec3 vWorld;
out vec4 vTint;
out float gl_ClipDistance[2];
uniform vec4 uParams0;
void main() {
    // a piece of a broken tree: only model heights uParams0.z..w of it
    gl_ClipDistance[0] = aPos.y - uParams0.z;
    gl_ClipDistance[1] = uParams0.w - aPos.y;
    vec4 world = aModel * vec4(aPos, 1.0);
    vec3 base = aModel[3].xyz;
    float up = max(world.y - base.y, 0.0);
    float phase = dot(base.xz, vec2(0.13, 0.17));
    float t = uTime;
    vec2 wd = normalize(vec2(0.8, 0.6));
    float gust = 0.55 + 0.45 * sin(t * 0.7 + phase) * sin(t * 0.31 + phase * 1.7);
    // a toppled tree lies still: the sway fades as its up axis leans over
    float upright = clamp(aModel[1].y / max(length(aModel[1].xyz), 1e-4), 0.0, 1.0);
    upright *= upright * upright;
    float sway = (0.35 * sin(t * 1.4 + phase) + gust) * uParams0.x * upright;
    world.xz += wd * sway * (up * up) * 0.0022;
    if (uParams0.y > 0.0) {
        float ph = dot(world.xyz, vec3(1.9, 2.7, 1.3));
        vec3 f = vec3(sin(t * 5.3 + ph), 0.6 * sin(t * 6.7 + ph * 1.3), cos(t * 4.9 + ph * 0.7));
        world.xyz += f * 0.05 * uParams0.x * uParams0.y * min(up * 0.25, 1.0);
    }
    vWorld = world.xyz;
    gl_Position = uViewProj * world;
    vNormal = mat3(aModel) * aNormal;
    vUV = aUV;
    vTint = aTint;
}
)";

// Trees, rocks and the player: the same lighting as the grass and ground.
// uParams0.y marks foliage (softer, sky-bent normals and light through the
// leaves), uParams1.x is the rim strength, uParams1.y a glow (eyes).
static const char* MODEL_FS = "#version 440 core\n" LOOK_GLSL R"(
in vec3 vNormal;
in vec2 vUV;
in vec3 vWorld;
in vec4 vTint;
out vec4 frag;
uniform sampler2D uTex;
uniform vec4  uColor;
uniform float uAlphaCutoff;
uniform vec4  uParams0;
uniform vec4  uParams1;
void main() {
    clip_test(vWorld);
    vec4 tex = texture(uTex, vUV);
    if (tex.a * uColor.a < uAlphaCutoff) discard;
    vec3 albedo = srgb_to_linear(tex.rgb * uColor.rgb * vTint.rgb);
    vec3 n = normalize(vNormal);
    if (!gl_FrontFacing) n = -n;
    vec3 v = normalize(uCameraPos - vWorld);
    float leaf = uParams0.y;
    // a canopy is lit as one soft volume, not as hundreds of flat cards
    n = normalize(mix(n, vec3(0.0, 1.0, 0.0), leaf * 0.5));
    float sh = look_shadow(vWorld, n);
    vec3 col = look_shade(n, albedo, mix(1.0, 0.85, leaf), mix(0.15, 0.6, leaf), sh);
    col += leaf * uSunColor * albedo * pow(max(dot(-v, normalize(uSunDir)), 0.0), 4.0) * 0.6 * sh;
    col += look_rim(n, v, uParams1.x * (1.0 - leaf));
    col += albedo * uParams1.y;
    frag = vec4(apply_fog(col, vWorld), 1.0);
}
)";

// Mipmaps for an alpha-tested texture that keep its coverage: plain
// averaging lets alpha sink below the cut-off level by level, so distant
// foliage thins to a few speckles. Each level's alpha is rescaled until the
// same share of texels passes the cut-off as at full size.
static void cutout_mipmaps(idx texture, const image_file_data* im, float cutoff) {
    int w = im->width, h = im->height;
    const uint8_t cut = (uint8_t)(cutoff * 255.0f);
    size_t passing = 0;
    for (size_t i = 0; i < (size_t)w * h; i++) passing += (uint8_t)im->data[i * 4 + 3] >= cut;
    float target = (float)passing / (float)((size_t)w * h);
    uint8_t* src = pix_alloc<uint8_t>((size_t)w * h * 4);
    memcpy(src, im->data, (size_t)w * h * 4);
    glBindTexture(GL_TEXTURE_2D, texture);
    for (int level = 1; w > 1 || h > 1; level++) {
        int nw = w > 1 ? w / 2 : 1, nh = h > 1 ? h / 2 : 1;
        uint8_t* dst = pix_alloc<uint8_t>((size_t)nw * nh * 4);
        for (int y = 0; y < nh; y++)
            for (int x = 0; x < nw; x++)
                for (int c = 0; c < 4; c++) {
                    int x0 = x * 2, y0 = y * 2, x1 = x0 + (w > 1), y1 = y0 + (h > 1);
                    int sum = src[(y0 * w + x0) * 4 + c] + src[(y0 * w + x1) * 4 + c]
                            + src[(y1 * w + x0) * 4 + c] + src[(y1 * w + x1) * 4 + c];
                    dst[(y * nw + x) * 4 + c] = (uint8_t)(sum / 4);
                }
        float lo = 0.5f, hi = 8.0f;
        for (int it = 0; it < 14; it++) {
            float mid = (lo + hi) * 0.5f;
            size_t pass = 0;
            for (size_t i = 0; i < (size_t)nw * nh; i++) pass += dst[i * 4 + 3] * mid >= cut;
            if ((float)pass / (float)((size_t)nw * nh) < target) lo = mid; else hi = mid;
        }
        for (size_t i = 0; i < (size_t)nw * nh; i++) {
            float a = dst[i * 4 + 3] * hi;
            dst[i * 4 + 3] = (uint8_t)(a > 255.0f ? 255.0f : a);
        }
        glTexSubImage2D(GL_TEXTURE_2D, level, 0, 0, nw, nh, GL_RGBA, GL_UNSIGNED_BYTE, dst);
        pix_free(src);
        src = dst; w = nw; h = nh;
    }
    pix_free(src);
    glBindTexture(GL_TEXTURE_2D, 0);
}

#define MAX_PARTS 3

// A tree breaks along its height at these bounds: below the stump line, the
// lower and upper log, the crown. A broken piece is a run of slabs lo..hi.
enum { CUT_BOTTOM, CUT_STUMP, CUT_MID, CUT_CROWN, CUT_TOP, CUTS };
#define SLABS 10         // runs lo < hi

static int slab_index(int lo, int hi) {                 // (0,1) (0,2) ... (3,4) -> 0..9
    static const int first[CUTS] = { 0, 4, 7, 9, 10 };
    return first[lo] + (hi - lo - 1);
}

// where a species is cut, as fractions of its model height from the lowest point
static void tree_cut_fractions(float model_height, float* f) {
    f[CUT_BOTTOM] = -100.0f;
    f[CUT_STUMP]  = (0.15f + 0.04f * model_height) / model_height;   // just over the sunk roots
    f[CUT_CROWN]  = maxf(0.55f, f[CUT_STUMP] + 0.2f);
    f[CUT_MID]    = (f[CUT_STUMP] + f[CUT_CROWN]) * 0.5f;
    f[CUT_TOP]    = 100.0f;
}

struct prop_kind {
    idx   mesh[MAX_PARTS], material[MAX_PARTS];
    bool  leaves[MAX_PARTS];
    int   parts;
    float height;       // model units, from the lowest point up
    float base;         // the lowest point: some models start well above their origin
    float footprint;    // widest reach of the geometry below 1.5 units: what the player bumps into
    float spread;       // widest reach of the upper part: the crown a fallen tree rests on
    // trees, for each run of slabs: its material clipped to it, and the box
    // around it in model units (leaves drawn in a little) - and around its
    // bark alone - for the pieces of a broken tree
    float cut[CUTS];
    idx   slab[MAX_PARTS][SLABS];
    vec3  box_min[SLABS], box_max[SLABS];
    vec3  wood_min[SLABS], wood_max[SLABS];
    bool  wood[SLABS], leafy[SLABS];
};

struct texture_cache_entry { uint64_t key; idx texture; };
static texture_cache_entry g_textures[64];
static int g_texture_count;

static bool has_cutout_alpha(const image_file_data* im) {
    if (!im || !im->data) return false;
    size_t n = (size_t)im->width * im->height;
    for (size_t i = 0; i < n; i += 7) if ((uint8_t)im->data[i * 4 + 3] < 128) return true;
    return false;
}

static idx cached_texture(const image_file_data* im, bool palette) {
    if (!im || !im->data) return 0;
    // FNV-1a over the size and a spread of samples is plenty to tell textures apart
    uint64_t h = 1469598103934665603ull ^ ((uint64_t)im->width << 32 | (uint64_t)im->height);
    size_t bytes = (size_t)im->width * im->height * 4;
    for (size_t i = 0; i < bytes; i += 61) { h ^= (uint8_t)im->data[i]; h *= 1099511628211ull; }
    for (int i = 0; i < g_texture_count; i++) if (g_textures[i].key == h) return g_textures[i].texture;
    idx t = opengl_create_texture2d(im->width, im->height, 4, im->data, palette ? TEXTURE_PIXELATED : TEXTURE_BILINEAR);
    if (!palette && has_cutout_alpha(im)) cutout_mipmaps(t, im, 0.5f);
    if (g_texture_count < 64) g_textures[g_texture_count++] = { h, t };
    return t;
}

static prop_kind load_prop(const char* path, idx shader) {
    pix_renderer& r = pix_get_renderer();
    pix_data_loader& l = pix_get_loader();
    prop_kind k = {};
    pix_loader_mark mark = pix_loader_mark_now(l);       // the CPU copies go once they are on the GPU
    int mats[MAX_PARTS];
    int mc = gltf_list_materials(l.arena, path, -1, mats, MAX_PARTS);
    if (mc <= 0) { mats[0] = -1; mc = 1; }
    float top = -1e9f, bottom = 1e9f;
    const skinned_mesh_file_data* loaded[MAX_PARTS];
    for (int i = 0; i < mc; i++) {
        idx id = load_model_gltf_file(l, path, -1, mats[i]);
        if (id == PIX_INVALID) continue;
        model_file_data& mf = l.model_files[id];
        skinned_mesh_file_data* md = get_model_mesh(l, id);
        if (!md) continue;
        const image_file_data* im = mf.image >= 0 ? &l.image_files[mf.image] : 0;
        material m = pix_material_default();
        m.texture = cached_texture(im, mf.image_is_palette);
        m.roughness = 1.0f;
        m.shader = shader;
        bool leafy = !mf.image_is_palette && has_cutout_alpha(im);
        if (leafy) { m.double_sided = true; m.cutout = true; }
        m.params0 = v4(1.0f, leafy ? 1.0f : 0.0f, -1e4f, 1e4f);        // z..w: nothing clipped
        m.params1 = v4(0.1f, 0.0f, 0.0f, 0.0f);          // just a hint of rim on bark and stone
        for (size_t v = 0; v < md->vertex_count; v++) {
            vec3 p = md->vertex_data[v].position;
            top = maxf(top, p.y);
            bottom = minf(bottom, p.y);
        }
        loaded[k.parts] = md;
        k.mesh[k.parts] = load_mesh_from_skinned(r, *md);
        k.material[k.parts] = pix_add_material(r, m);
        k.leaves[k.parts] = leafy;
        k.parts++;
    }
    // the footprint is measured up from the model's own lowest point, not its origin
    float reach = 0.0f, spread = 0.0f;
    for (int i = 0; i < k.parts; i++)
        for (size_t v = 0; v < loaded[i]->vertex_count; v++) {
            vec3 p = loaded[i]->vertex_data[v].position;
            float rr = sqrtf(p.x * p.x + p.z * p.z);
            if (p.y < bottom + 1.5f) reach = maxf(reach, rr);
            if (p.y > bottom + 0.4f * (top - bottom)) spread = maxf(spread, rr);
        }
    // the box of each run of slabs: every triangle clipped to the run's
    // heights, so a trunk modelled as one long quad still fills the middle
    tree_cut_fractions(maxf(top - bottom, 0.01f), k.cut);
    for (int sl = 0; sl < SLABS; sl++) {
        k.box_min[sl] = k.wood_min[sl] = v3(1e9f, 1e9f, 1e9f);
        k.box_max[sl] = k.wood_max[sl] = v3(-1e9f, -1e9f, -1e9f);
    }
    for (int i = 0; i < k.parts; i++) {
        const skinned_mesh_file_data* md = loaded[i];
        size_t tris = md->index_count ? md->index_count / 3 : md->vertex_count / 3;
        for (size_t tri = 0; tri < tris; tri++) {
            vec3 v[3];
            for (int c = 0; c < 3; c++) {
                size_t vi = md->index_count ? md->index_data[tri * 3 + c] : tri * 3 + c;
                v[c] = md->vertex_data[vi].position;
                if (k.leaves[i]) { v[c].x *= 0.75f; v[c].z *= 0.75f; }      // twigs give
            }
            for (int lo = 0; lo < CUTS; lo++)
                for (int hi = lo + 1; hi < CUTS; hi++) {
                    float y0 = bottom + k.cut[lo] * (top - bottom), y1 = bottom + k.cut[hi] * (top - bottom);
                    int sl = slab_index(lo, hi);
                    auto add = [&](vec3 q) {
                        k.box_min[sl] = v3(minf(k.box_min[sl].x, q.x), minf(k.box_min[sl].y, q.y), minf(k.box_min[sl].z, q.z));
                        k.box_max[sl] = v3(maxf(k.box_max[sl].x, q.x), maxf(k.box_max[sl].y, q.y), maxf(k.box_max[sl].z, q.z));
                        if (k.leaves[i]) { k.leafy[sl] = true; return; }
                        k.wood[sl] = true;
                        k.wood_min[sl] = v3(minf(k.wood_min[sl].x, q.x), minf(k.wood_min[sl].y, q.y), minf(k.wood_min[sl].z, q.z));
                        k.wood_max[sl] = v3(maxf(k.wood_max[sl].x, q.x), maxf(k.wood_max[sl].y, q.y), maxf(k.wood_max[sl].z, q.z));
                    };
                    for (int c = 0; c < 3; c++) {
                        vec3 a = v[c], b = v[(c + 1) % 3];
                        if (a.y >= y0 && a.y <= y1) add(a);
                        // where the edge crosses either bound
                        float ys[2] = { y0, y1 };
                        for (int e = 0; e < 2; e++)
                            if ((a.y - ys[e]) * (b.y - ys[e]) < 0.0f) add(a + (b - a) * ((ys[e] - a.y) / (b.y - a.y)));
                    }
                }
        }
    }
    for (int sl = 0; sl < SLABS; sl++) {
        if (k.box_min[sl].x > k.box_max[sl].x) k.box_min[sl] = k.box_max[sl] = v3(0, 0, 0);
        if (!k.wood[sl]) k.wood_min[sl] = k.wood_max[sl] = v3(0, 0, 0);
    }
    pix_loader_rewind(l, mark);
    k.base = k.parts ? bottom : 0.0f;
    k.height = k.parts ? top - bottom : 0.0f;
    k.footprint = reach;
    k.spread = spread;
    if (!k.parts) pix_log("could not load %s", path);
    return k;
}

struct prop {
    int   kind;
    vec3  pos;
    float yaw, scale;
    float radius;       // collision circle in world units, 0 = walk through (pebbles)
};

struct obstacle { float x, z, r; };

// ============================================================================
// ---- trees that break ----
//
// Enough damage and a tree snaps just above the ground, leaving its stump.
// The rest first tips over the stump - slowly, then faster, a pendulum falling
// the wrong way - until it has leaned far over, touches the ground, or strikes
// another trunk. Then it is a free rigid body: an oriented box fitted to its
// part of the mesh, bouncing and sliding on the terrain (its corners and
// points along its edges), off trunks and stumps, and off the other broken
// pieces lying about. A hard landing snaps the crown off. What lies there is
// a log in the way; more blows crack it into sections that jump apart, and a
// few more burst each one into chips and leaves.
//
// The pieces are never cut out of the mesh: the tree is sliced along its
// height into slabs (see CUT_*), a piece is a run of neighbouring slabs, and
// it is drawn as the whole tree clipped to its slabs by two planes in TREE_VS.
//
// A point p of the standing tree is at `com + rot * (p - com0)` in a piece.

enum { TREE_STANDING, TREE_TIPPING, TREE_BROKEN };
enum { PIECE_FALLING, PIECE_DOWN, PIECE_GONE };
#define TREE_PIECES  3
#define PIECE_POINTS 40

struct tree_piece {
    int   lo, hi;               // CUT_* bounds
    int   state;
    float hp, len;              // len: metres of the tree it spans
    // Its box, standing: centre (also the centre of mass), axes, half sizes.
    // Leaves are drawn in a little (branches give); `wood` is the bark alone,
    // what creatures bump into - a crown's twigs are walked through.
    vec3  com0, axis0[3], half;
    bool  wood;
    vec3  wood0, wood_half;
    bool  leafy;                // has leaves: lands softly, sheds them
    float mass, inv_i[3];       // inverse inertia about the box axes
    int   points;
    vec3  pt[PIECE_POINTS];     // corners and points along the edges, standing
    vec3  com, vel, avel;       // centre now, and how it moves
    quat  rot;                  // turn from the standing pose
    float still, fall_time, gone_for, down_for;
    vec3  rest_com, rest_axis;  // where it was at the start of this half second
    float shiver;               // a blow rocks it for a moment
    vec3  shiver_dir;
    float impact;               // hardest landing this frame, m/s into the ground
    vec3  impact_at;
};

struct tree_body {
    int   prop, obstacle, kind, state;
    const prop_kind* model;
    mat4  standing;                // its transform before it fell
    float hp, max_hp;
    float height, trunk;           // metres
    vec3  root;                    // the model's lowest point, standing
    float cut[CUTS];               // metres up the axis from root
    vec3  hinge;                   // tipping over the stump: piece 0
    float angle, spin;
    tree_piece piece[TREE_PIECES];
    int   pieces;
    bool  crushed;                 // its first heavy landing has hurt whatever was under it
    float shiver;                  // standing, a blow rocks it about its root
    vec3  shiver_dir;
    float shown;                   // seconds its health bar stays up
    int   shown_piece;             // over which piece (-1: the standing tree)
};

static vec3 tree_at(const tree_body& t, float h) { return t.root + v3(0, h, 0); }

static void tree_init(tree_body& t, int prop_index, int obstacle_index, int kind, const prop_kind* model,
                      const mat4& standing, vec3 root, float height, float trunk) {
    t = {};
    t.prop = prop_index;
    t.obstacle = obstacle_index;
    t.kind = kind;
    t.model = model;
    t.standing = standing;
    t.root = root;
    t.height = height;
    t.trunk = trunk;
    t.max_hp = t.hp = 60.0f + 7.0f * height;
    for (int i = 0; i < CUTS; i++) t.cut[i] = model->cut[i] * height;
}

// The shape of a piece spanning slabs lo..hi: its boxes, sample points, mass.
static void piece_shape(const tree_body& t, tree_piece& p, int lo, int hi) {
    p.lo = lo;
    p.hi = hi;
    p.len = minf(t.cut[hi], t.height) - t.cut[lo];
    p.leafy = t.model->leafy[slab_index(lo, hi)];
    int s = slab_index(lo, hi);
    vec3 mn = t.model->box_min[s], mx = t.model->box_max[s];
    p.com0 = mat4_mul_point(t.standing, (mn + mx) * 0.5f);
    for (int i = 0; i < 3; i++) {
        vec3 e = v3(i == 0, i == 1, i == 2);
        vec3 d = mat4_mul_dir(t.standing, e);
        float l = v3len(d);
        p.axis0[i] = d * (1.0f / l);
        float h = ((i == 0 ? mx.x - mn.x : i == 1 ? mx.y - mn.y : mx.z - mn.z) * 0.5f) * l;
        (i == 0 ? p.half.x : i == 1 ? p.half.y : p.half.z) = maxf(h, 0.05f);
    }
    p.wood = t.model->wood[s];
    vec3 wn = t.model->wood_min[s], wx = t.model->wood_max[s];
    p.wood0 = mat4_mul_point(t.standing, (wn + wx) * 0.5f);
    vec3 wl = mat4_mul_dir(t.standing, v3(1, 0, 0));
    float sc = v3len(wl);
    p.wood_half = v3(maxf((wx.x - wn.x) * 0.5f * sc, 0.05f), maxf((wx.y - wn.y) * 0.5f * sc, 0.05f),
                     maxf((wx.z - wn.z) * 0.5f * sc, 0.05f));
    // mass from the wood (and a little for the leaves), inertia of a solid box
    float wood_vol = p.wood ? 8.0f * p.wood_half.x * p.wood_half.y * p.wood_half.z : 0.0f;
    float vol = 8.0f * p.half.x * p.half.y * p.half.z;
    p.mass = maxf(0.3f, wood_vol * 0.6f + (p.leafy ? vol * 0.02f : 0.0f));
    float hx = p.half.x * p.half.x, hy = p.half.y * p.half.y, hz = p.half.z * p.half.z;
    p.inv_i[0] = 3.0f / (p.mass * (hy + hz));
    p.inv_i[1] = 3.0f / (p.mass * (hx + hz));
    p.inv_i[2] = 3.0f / (p.mass * (hx + hy));
    // the corners, and points along each long edge so a long side lies flat
    int n = 0;
    vec3 a0 = p.axis0[0] * p.half.x, a1 = p.axis0[1] * p.half.y, a2 = p.axis0[2] * p.half.z;
    for (int c = 0; c < 8; c++)
        p.pt[n++] = p.com0 + a0 * (c & 1 ? 1.0f : -1.0f) + a1 * (c & 2 ? 1.0f : -1.0f) + a2 * (c & 4 ? 1.0f : -1.0f);
    for (int c = 0; c < 8 && n < PIECE_POINTS; c++)
        for (int e = 0; e < 3; e++) {
            int d = c ^ (1 << e);
            if (d < c) continue;                          // each edge once
            vec3 a = p.pt[c], b = p.pt[d];
            int steps = (int)(v3dist(a, b) / 1.2f);
            for (int k = 1; k < steps && n < PIECE_POINTS; k++) p.pt[n++] = a + (b - a) * ((float)k / steps);
        }
    p.points = n;
    p.hp = 18.0f + 2.5f * p.len;
}

static vec3 piece_place(const tree_piece& p, vec3 standing) { return p.com + quat_rotate(p.rot, standing - p.com0); }
static vec3 piece_point(const tree_piece& p, int i) { return piece_place(p, p.pt[i]); }
static void piece_axes(const tree_piece& p, vec3* ax) { for (int i = 0; i < 3; i++) ax[i] = quat_rotate(p.rot, p.axis0[i]); }

// the point of box (c, ax, h) nearest to q
static vec3 obb_closest(vec3 c, const vec3* ax, vec3 h, vec3 q) {
    vec3 d = q - c, r = c;
    r += ax[0] * clampf(v3dot(d, ax[0]), -h.x, h.x);
    r += ax[1] * clampf(v3dot(d, ax[1]), -h.y, h.y);
    r += ax[2] * clampf(v3dot(d, ax[2]), -h.z, h.z);
    return r;
}
static float obb_top(vec3 c, const vec3* ax, vec3 h) {
    return c.y + fabsf(ax[0].y) * h.x + fabsf(ax[1].y) * h.y + fabsf(ax[2].y) * h.z;
}

// its axis as a segment, for the dust and chips of a burst
static void piece_segment(const tree_body& t, const tree_piece& p, vec3* a, vec3* b) {
    *a = piece_place(p, tree_at(t, t.cut[p.lo]));
    *b = piece_place(p, tree_at(t, minf(t.cut[p.hi], t.height)));
}

static vec3 closest_on_segment(vec3 a, vec3 b, vec3 p) {
    vec3 ab = b - a;
    float l2 = v3dot(ab, ab);
    float s = l2 > 1e-8f ? clampf(v3dot(p - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
    return a + ab * s;
}

// Knocked down: the stump stays, the rest starts to lean away from the blow.
static void tree_topple(tree_body& t, vec3 dir) {
    dir.y = 0.0f;
    if (v3len(dir) < 1e-3f) dir = v3(1, 0, 0);
    dir = v3norm(dir);
    t.state = TREE_TIPPING;
    t.hinge = v3norm(v3cross(v3(0, 1, 0), dir));    // turning up about it leans the top toward dir
    t.angle = 0.0f;
    t.spin = 0.25f;
    t.pieces = 1;
    tree_piece& p = t.piece[0];
    p = {};
    piece_shape(t, p, CUT_STUMP, CUT_TOP);
    p.com = p.com0;
    p.rot = quat_identity();
    p.state = PIECE_FALLING;
}

// Break piece i at bound `at`: it keeps the part below, a new piece takes the
// part above, both moving on as the one body was.
static tree_piece* piece_split(tree_body& t, int i, int at) {
    if (t.pieces >= TREE_PIECES || at <= t.piece[i].lo || at >= t.piece[i].hi) return 0;
    tree_piece old = t.piece[i];
    tree_piece& lower = t.piece[i];
    tree_piece& upper = t.piece[t.pieces++];
    upper = old;
    piece_shape(t, lower, old.lo, at);
    piece_shape(t, upper, at, old.hi);
    tree_piece* both[2] = { &lower, &upper };
    for (int k = 0; k < 2; k++) {
        tree_piece& p = *both[k];
        p.com = piece_place(old, p.com0);
        p.vel = old.vel + v3cross(old.avel, p.com - old.com);
        p.state = PIECE_FALLING;
        p.still = p.fall_time = p.down_for = 0.0f;
        p.rest_com = v3(1e9f, 0, 0);
    }
    return &upper;
}

static vec3 piece_inv_inertia(const tree_piece& p, vec3 x) {
    vec3 ax[3];
    piece_axes(p, ax);
    return ax[0] * (v3dot(x, ax[0]) * p.inv_i[0]) + ax[1] * (v3dot(x, ax[1]) * p.inv_i[1])
         + ax[2] * (v3dot(x, ax[2]) * p.inv_i[2]);
}

static vec3 terrain_normal(float x, float z) {
    float e = 0.4f;
    return v3norm(v3(terrain_height(x - e, z) - terrain_height(x + e, z), 2.0f * e,
                     terrain_height(x, z - e) - terrain_height(x, z + e)));
}

// An impulse at `at` along n between a and b (b null: the ground, a trunk):
// cancel their closing speed with a little bounce, rub off the sliding with
// friction. Returns the impulse.
static float piece_impulse(tree_piece& a, tree_piece* b, vec3 at, vec3 n, float bounce, float friction) {
    vec3 ra = at - a.com, rb = b ? at - b->com : v3(0, 0, 0);
    vec3 v = a.vel + v3cross(a.avel, ra);
    if (b) v -= b->vel + v3cross(b->avel, rb);
    float vn = v3dot(v, n);
    if (vn >= 0.0f) return 0.0f;
    auto k_along = [&](vec3 d) {
        float k = 1.0f / a.mass + v3dot(d, v3cross(piece_inv_inertia(a, v3cross(ra, d)), ra));
        if (b) k += 1.0f / b->mass + v3dot(d, v3cross(piece_inv_inertia(*b, v3cross(rb, d)), rb));
        return k;
    };
    auto apply = [&](vec3 j) {
        a.vel += j * (1.0f / a.mass);
        a.avel += piece_inv_inertia(a, v3cross(ra, j));
        if (b) {
            b->vel -= j * (1.0f / b->mass);
            b->avel -= piece_inv_inertia(*b, v3cross(rb, j));
        }
    };
    float jn = -(1.0f + bounce) * vn / k_along(n);
    apply(n * jn);
    vec3 vt = v - n * vn;
    float vtl = v3len(vt);
    if (vtl > 1e-4f) {
        vec3 tan = vt * (1.0f / vtl);
        apply(tan * -minf(vtl / k_along(tan), friction * jn));
    }
    return jn;
}

// Box against a standing trunk or stump: a vertical capsule from the ground
// up to `top`. Out: where they touch, the way out for the box and how deep.
static bool piece_hits_trunk(const tree_piece& p, const vec3* ax, const obstacle& o, float top,
                             vec3* at, vec3* n, float* depth) {
    float reach = v3len(p.half) + o.r;
    float dx = p.com.x - o.x, dz = p.com.z - o.z;
    if (dx * dx + dz * dz > reach * reach) return false;
    float bottom = terrain_height(o.x, o.z) - 1.0f;
    vec3 q = v3(o.x, clampf(p.com.y, bottom, top), o.z);
    vec3 c = obb_closest(p.com, ax, p.half, q);
    q.y = clampf(c.y, bottom, top);
    c = obb_closest(p.com, ax, p.half, q);
    vec3 d = c - q;
    float l = v3len(d);
    if (l >= o.r) return false;
    if (l < 1e-4f) { d = v3(dx, 0.0f, dz); l = maxf(v3len(d), 1e-4f); }
    *n = d * (1.0f / l);
    *at = c;
    *depth = o.r - minf(l, o.r);
    return true;
}

// Advance every tree that is falling or lies broken by dt. `obs`/`tops` are
// the standing trunks and stumps; `acc` carries the fixed step between frames.
static void trees_update(tree_body* trees, int count, float dt, obstacle* obs, int obs_count, const float* tops, float& acc) {
    for (int ti = 0; ti < count; ti++) {
        tree_body& t = trees[ti];
        t.shiver = maxf(0.0f, t.shiver - dt * 2.5f);
        t.shown = maxf(0.0f, t.shown - dt);
        for (int i = 0; i < t.pieces; i++) {
            tree_piece& p = t.piece[i];
            p.impact = 0.0f;
            p.shiver = maxf(0.0f, p.shiver - dt * 2.5f);
            if (p.state == PIECE_GONE) p.gone_for += dt;
            if (p.state == PIECE_DOWN) {
                // a section left lying long enough sinks away into the grass
                p.down_for += dt;
                if (p.hi - p.lo == 1 && p.down_for > 60.0f) {
                    p.com.y -= 0.25f * dt;
                    if (p.down_for > 66.0f) { p.state = PIECE_GONE; p.gone_for = 1.0f; }
                }
            }
        }
    }
    const float H = 1.0f / 120.0f, G = 9.8f;
    acc = minf(acc + dt, 0.1f);
    while (acc >= H) {
        acc -= H;
        for (int ti = 0; ti < count; ti++) {
            tree_body& t = trees[ti];
            if (t.state == TREE_STANDING) continue;
            for (int i = 0; i < t.pieces; i++) {
                tree_piece& p = t.piece[i];
                if (p.state != PIECE_FALLING) continue;
                vec3 ax[3];
                if (t.state == TREE_TIPPING) {
                    // a rod swinging over the stump: the torque of its weight about the hinge
                    vec3 pivot = tree_at(t, t.cut[CUT_STUMP]);
                    vec3 r = p.com0 - pivot;
                    float d = v3len(r), ip = p.len * p.len / 12.0f + d * d;
                    t.spin += G * d * sinf(t.angle) / ip * H;
                    t.angle += t.spin * H;
                    p.rot = quat_from_axis_angle(t.hinge, t.angle);
                    p.com = pivot + quat_rotate(p.rot, r);
                    // it tears free of the stump once well over, or when it meets anything
                    piece_axes(p, ax);
                    bool touch = t.angle > 1.15f;
                    for (int k = 0; k < p.points && !touch; k++) {
                        if (p.pt[k].y < pivot.y + 0.3f) continue;          // the broken end, on the stump
                        vec3 at = piece_point(p, k);
                        touch = at.y < terrain_height(at.x, at.z);
                    }
                    for (int o = 0; o < obs_count && !touch; o++) {
                        vec3 at, n;
                        float depth;
                        touch = o != t.obstacle && obs[o].r > 0.0f && piece_hits_trunk(p, ax, obs[o], tops[o], &at, &n, &depth);
                    }
                    if (touch) {
                        t.state = TREE_BROKEN;
                        p.avel = t.hinge * t.spin;
                        p.vel = v3cross(p.avel, p.com - pivot);
                        p.rest_com = v3(1e9f, 0, 0);
                    }
                    continue;
                }
                // gravity, then the ground under each sample point
                p.vel.y -= G * H;
                piece_axes(p, ax);
                int touching = 0;
                vec3 push = v3(0, 0, 0);
                for (int k = 0; k < p.points; k++) {
                    vec3 at = piece_point(p, k);
                    float ground = terrain_height(at.x, at.z);
                    if (at.y >= ground) continue;
                    vec3 n = terrain_normal(at.x, at.z);
                    vec3 v = p.vel + v3cross(p.avel, at - p.com);
                    float into = -v3dot(v, n);
                    if (into > p.impact) { p.impact = into; p.impact_at = v3(at.x, ground, at.z); }
                    touching++;
                    // leaves give and scrape; wood thuds and grips
                    piece_impulse(p, 0, at, n, p.leafy ? 0.05f : 0.15f, p.leafy ? 0.5f : 0.8f);
                    push.y = maxf(push.y, (ground - at.y) * n.y);
                }
                // off the trunks and stumps still standing
                for (int o = 0; o < obs_count; o++) {
                    vec3 at, n;
                    float depth;
                    if (o == t.obstacle || obs[o].r <= 0.0f || !piece_hits_trunk(p, ax, obs[o], tops[o], &at, &n, &depth)) continue;
                    piece_impulse(p, 0, at, n, 0.1f, 0.4f);
                    p.com += n * (depth * 0.5f);
                }
                p.com += push * 0.8f;
                // resting on the ground it rolls and slides only reluctantly
                p.vel = p.vel * (1.0f - (touching > 1 ? 1.5f : 0.1f) * H);
                p.avel = p.avel * (1.0f - (touching > 1 ? 4.0f : 0.6f) * H);
            }
        }
        // pieces against pieces, of any tree: each one's sample points pushed
        // out of the other's box through its nearest face
        for (int ta = 0; ta < count; ta++) {
            if (trees[ta].state != TREE_BROKEN) continue;
            for (int ia = 0; ia < trees[ta].pieces; ia++) {
                tree_piece& a = trees[ta].piece[ia];
                if (a.state != PIECE_FALLING) continue;
                for (int tb = 0; tb < count; tb++) {
                    if (trees[tb].state != TREE_BROKEN) continue;
                    for (int ib = 0; ib < trees[tb].pieces; ib++) {
                        tree_piece& b = trees[tb].piece[ib];
                        if (&a == &b || b.state == PIECE_GONE) continue;
                        float reach = v3len(a.half) + v3len(b.half);
                        if (v3dist(a.com, b.com) > reach) continue;
                        vec3 bx[3];
                        piece_axes(b, bx);
                        for (int k = 0; k < a.points; k++) {
                            vec3 at = piece_point(a, k), d = at - b.com;
                            float l[3] = { v3dot(d, bx[0]), v3dot(d, bx[1]), v3dot(d, bx[2]) };
                            float h[3] = { b.half.x, b.half.y, b.half.z };
                            int axis = -1;
                            float pen = 1e9f;
                            for (int e = 0; e < 3; e++) {
                                float in = h[e] - fabsf(l[e]);
                                if (in <= 0.0f) { axis = -1; break; }
                                if (in < pen) { pen = in; axis = e; }
                            }
                            if (axis < 0) continue;
                            vec3 n = bx[axis] * (l[axis] < 0.0f ? -1.0f : 1.0f);
                            float j = piece_impulse(a, &b, at, n, 0.1f, 0.5f);
                            float share = b.state == PIECE_FALLING ? 0.5f : 1.0f;
                            a.com += n * (pen * 0.3f * share);
                            if (b.state == PIECE_FALLING) b.com -= n * (pen * 0.3f * 0.5f);
                            // knocked hard enough, a log lying there rolls again
                            if (b.state == PIECE_DOWN && j / b.mass > 0.4f) {
                                b.state = PIECE_FALLING;
                                b.still = b.fall_time = 0.0f;
                                b.rest_com = v3(1e9f, 0, 0);
                            } else if (b.state == PIECE_DOWN) {
                                b.vel = b.avel = v3(0, 0, 0);
                            }
                        }
                    }
                }
            }
        }
        // move, and settle
        for (int ti = 0; ti < count; ti++) {
            tree_body& t = trees[ti];
            if (t.state != TREE_BROKEN) continue;
            for (int i = 0; i < t.pieces; i++) {
                tree_piece& p = t.piece[i];
                if (p.state != PIECE_FALLING) continue;
                // integrate the orientation: dq/dt = 0.5 * w * q
                quat w = { p.avel.x, p.avel.y, p.avel.z, 0.0f };
                quat dq = quat_mul(w, p.rot);
                p.rot.x += 0.5f * H * dq.x; p.rot.y += 0.5f * H * dq.y;
                p.rot.z += 0.5f * H * dq.z; p.rot.w += 0.5f * H * dq.w;
                p.rot = quat_norm(p.rot);
                p.com += p.vel * H;
                // settled once it has hardly moved for half a second (the
                // contacts jitter a little forever), or after long enough
                p.fall_time += H;
                p.still += H;
                bool settled = false;
                if (p.still >= 0.5f) {
                    vec3 axis = quat_rotate(p.rot, v3(0, 1, 0));
                    settled = v3dist(p.com, p.rest_com) < 0.08f && v3dist(axis, p.rest_axis) < 0.03f
                           && v3len(p.vel) < 0.5f && v3len(p.avel) < 0.5f;
                    p.rest_com = p.com;
                    p.rest_axis = axis;
                    p.still = 0.0f;
                }
                if (settled || p.fall_time > 10.0f) { p.state = PIECE_DOWN; p.vel = p.avel = v3(0, 0, 0); p.down_for = 0.0f; }
                // a hard landing snaps the crown off, and it goes its own way
                if (p.lo < CUT_CROWN && p.hi > CUT_CROWN && p.impact > 6.5f) {
                    float hit = p.impact;
                    vec3 at = p.impact_at;
                    if (tree_piece* crown = piece_split(t, i, CUT_CROWN)) {
                        crown->impact = hit;
                        crown->impact_at = at;
                        t.piece[i].impact = 0.0f;
                    }
                }
            }
        }
    }
}

// the standing tree's transform turned into piece p's (or the stump's, p null)
static mat4 piece_transform(const tree_body& t, const tree_piece* p, const mat4& standing, float time) {
    if (!p) {
        if (t.shiver <= 0.0f) return standing;
        // a blow rocks a standing tree about its root
        vec3 axis = v3norm(v3cross(v3(0, 1, 0), t.shiver_dir));
        quat k = quat_from_axis_angle(axis, t.shiver * t.shiver * sinf(time * 22.0f) * 0.035f);
        vec3 c = tree_at(t, t.cut[CUT_STUMP]);
        return mat4_translate(c.x, c.y, c.z) * mat4_from_quat(k) * mat4_translate(-c.x, -c.y, -c.z) * standing;
    }
    quat q = p->rot;
    if (p->shiver > 0.0f) {
        vec3 axis = v3norm(v3cross(v3(0, 1, 0), p->shiver_dir));
        q = quat_mul(quat_from_axis_angle(axis, p->shiver * p->shiver * sinf(time * 22.0f) * 0.06f), q);
    }
    float s = p->state == PIECE_GONE ? 1.0f - smoothstepf(0.0f, 0.25f, p->gone_for) : 1.0f;
    vec3 c = p->com, c0 = p->com0;
    return mat4_translate(c.x, c.y, c.z) * mat4_from_quat(q) * mat4_scale(s, s, s)
         * mat4_translate(-c0.x, -c0.y, -c0.z) * standing;
}

// the box of a piece as twelve lines (the B key)
static void draw_piece_box(vec3 c, const vec3* ax, vec3 h, pix_color color) {
    vec3 corner[8];
    for (int k = 0; k < 8; k++)
        corner[k] = c + ax[0] * (k & 1 ? h.x : -h.x) + ax[1] * (k & 2 ? h.y : -h.y) + ax[2] * (k & 4 ? h.z : -h.z);
    for (int k = 0; k < 8; k++)
        for (int e = 0; e < 3; e++)
            if (!(k & (1 << e))) pix_draw_line3d(corner[k], corner[k | (1 << e)], color);
}

// ============================================================================
// ---- the player ----

enum { CLIP_STAND, CLIP_WALK, CLIP_RUN, CLIP_JUMP, CLIP_ROLL, CLIP_CROUCH, CLIP_COUNT };
static const char* CLIP_NAMES[CLIP_COUNT] = { "Stand", "Walk", "Run", "Jump", "Roll", "Crouch" };

// roll: the human's tumble or the wolf's leap back; attack: wolf form only
enum { MOVE_GROUND, MOVE_TAKEOFF, MOVE_AIR, MOVE_LAND, MOVE_ROLL, MOVE_ATTACK, MOVE_HURT, MOVE_DEAD, MOVE_HOWL };

// The jump clip's timeline, read off how high it lifts the hips: the physics
// launches on the take-off frame, the clip holds its falling pose until the
// ground is actually reached, then plays the landing squat.
struct jump_timing {
    float enter;     // start here: the bottom of the crouch, minus a moment
    float launch;    // the hips pass their rest height on the way up
    float hold;      // well into the fall - held while still airborne
    float touch;     // back down at rest height: feet on the ground
    float recover;   // enough of the squat to hand back to locomotion
};

static jump_timing measure_jump(const pix_model& m, idx clip, int hips) {
    jump_timing j = { 0.2f, 0.33f, 0.65f, 0.8f, 0.95f };
    if (!m.skinned || clip >= m.clip_count || hips < 0) return j;
    animator a = pix_model_animator(m);
    animation* pose = pix_alloc<animation>(1);
    const int N = 96;
    float dur = animator_duration(a, clip), y[N];
    for (int i = 0; i < N; i++) {
        animator_sample(a, clip, dur * i / (N - 1), pose);
        y[i] = animation_bone_world(a, *pose, hips).data[13];
    }
    pix_free(pose);
    int top = 0;
    for (int i = 0; i < N; i++) if (y[i] > y[top]) top = i;
    int low = 0;
    for (int i = 0; i < top; i++) if (y[i] < y[low]) low = i;
    float rest = y[0], peak = y[top];
    if (peak - rest < 0.05f) return j;                       // not a jump that leaves the ground
    auto at = [&](int i) { return dur * i / (N - 1); };
    int launch = low;
    while (launch < top && y[launch] < rest) launch++;
    int hold = top;
    while (hold < N - 1 && y[hold] > rest + (peak - rest) * 0.45f) hold++;
    int touch = hold;
    while (touch < N - 1 && y[touch] > rest) touch++;
    j.enter = maxf(0.0f, at(low) - 0.08f);
    j.launch = at(launch);
    j.hold = at(hold);
    j.touch = at(touch);
    j.recover = at(touch) + (dur - at(touch)) * 0.55f;
    return j;
}

#define EVENT_STEP 1

// What a looping gait does, measured from the clip itself.
struct gait {
    float speed;        // metres per second it is played at
    float clip_speed;   // metres per second the clip covers at playback speed 1 (0 = could not tell)
    float facing;       // +1 when the model walks toward +Z, -1 toward -Z
    float steps[4];     // clip times of foot plants
    int   step_count;
};

// `axis` is the matrix element the stride runs along: 14 (Z) for walking
// forward and back, 12 (X) for a sidestep.
static gait measure_gait(const pix_model& m, idx clip, int foot_l, int foot_r, int axis = 14) {
    gait g = {};
    g.facing = 1.0f;
    if (!m.skinned || clip >= m.clip_count || foot_l < 0 || foot_r < 0) return g;
    animator a = pix_model_animator(m);
    animation* pose = pix_alloc<animation>(1);
    const int N = 64;
    float dur = animator_duration(a, clip);
    float z[2][N];
    for (int i = 0; i < N; i++) {
        animator_sample(a, clip, dur * i / N, pose);
        for (int f = 0; f < 2; f++) {
            mat4 w = animation_bone_world(a, *pose, f ? foot_r : foot_l);
            z[f][i] = w.data[axis];
        }
    }
    pix_free(pose);
    // Relative to the hips a foot only ever travels backwards while it is on
    // the ground, and it swings forward faster than it pushes back: the slower
    // of the two phases is the stance, and the distance it sweeps over that
    // time is the ground speed. The sign says which way the model faces.
    float dist = 0.0f, stance_time = 0.0f, facing = 0.0f;
    for (int f = 0; f < 2; f++) {
        float back = 0.0f, fwd = 0.0f;           // samples moving toward -Z / +Z
        float travel = 0.0f;
        for (int i = 0; i < N; i++) {
            float dz = z[f][(i + 1) % N] - z[f][i];
            if (dz < 0.0f) back += 1.0f; else fwd += 1.0f;
            travel += fabsf(dz);
        }
        float sign = back >= fwd ? 1.0f : -1.0f;  // +1: stance moves toward -Z, so the model faces +Z
        facing += sign;
        dist += travel * 0.5f;                     // the cycle returns to its start: half the path is stance
        stance_time += maxf(back, fwd) * dur / N;
        for (int i = 0; i < N; i++) {             // a foot plant: the moment it starts pushing back
            float d0 = (z[f][i] - z[f][(i + N - 1) % N]) * sign, d1 = (z[f][(i + 1) % N] - z[f][i]) * sign;
            if (d0 >= 0.0f && d1 < 0.0f && g.step_count < 4) g.steps[g.step_count++] = dur * i / N;
        }
    }
    if (stance_time > 0.0f) {
        g.speed = g.clip_speed = dist / stance_time;
        g.facing = facing < 0.0f ? -1.0f : 1.0f;
    }
    return g;
}

struct player {
    vec3  pos, vel;
    float heading;       // radians, 0 = facing +Z
    int   move;
    float roll_dir_x, roll_dir_z;
    int   clip;          // locomotion clip currently chosen (CLIP_* as a human, WC_* as a wolf)
    bool  grounded;
    bool  wolf;          // which form
    float hp;
    float invuln;        // seconds of grace after a hit
    bool  evading;       // in the untouchable part of a roll or a leap back
    float flash;         // hit flash, fading
    float morph;         // 1 right after changing form, down to 0
    float timer;         // seconds in the current move
    float calm;          // seconds since last hurt: health returns after a while
    int   attack;        // wolf form: ATK_*
    int   struck;        // strikes of this attack resolved
    float attack_rate;   // its playback speed
    float carry;         // speed it came into the attack with (a ram keeps it)
    bool  charged;       // this attack came out of a run-up
    bool  charging;      // wolf form: sprinting in for a tackle (charge_attack)
    int   charge_attack, charge_kind, charge_index;   // kind: -1 none, 0 wolf, 1 wolfman
    float charge_time;
    int   combo;         // wolf form: how far into the claw chain
    bool  slam;          // wolf form: heavy attack pressed in the air, smash on landing
    int   jump_clip;     // wolf form: which leap (WC_*) the jump timing comes from
    float air;           // seconds off the ground
    float root;          // root motion being cancelled, eased out over a cross-fade
    bool  gone;          // down and vanished in smoke
    float gone_for;      // seconds since
};

static void play(pix_anim_system& s, idx actor, const idx* clips, int clip, float fade, bool loop, float speed) {
    pix_anim_play(s, actor, clips[clip], fade, loop, speed);
}

static idx find_clip(const animator& a, const char* name, idx fallback) {
    for (size_t i = 0; i < a.clip_count; i++) {
        const char* n = animator_clip_name(a, (idx)i);
        if (n && !strcmp(n, name)) return (idx)i;
    }
    return fallback;
}

// When a one-shot clip (a death) comes to rest: the start of the still tail
// every bone holds until the end. Clips often keep a second or so of the body
// lying there; anything that vanishes on death should go at this moment, not
// at the clip's end.
static float measure_settle(const pix_model& m, idx clip) {
    animator a = pix_model_animator(m);
    float dur = animator_duration(a, clip);
    if (!m.skinned || clip >= m.clip_count || dur <= 0.0f) return dur;
    const int N = 64, B = (int)m.skeleton->bone_count;
    const float STILL = 0.1f;                  // model units per second: slower than this is lying still
    vec3* prev = pix_alloc<vec3>(B);
    animation* pose = pix_alloc<animation>(1);
    float settle = 0.0f, step = dur * 0.999f / N;
    for (int i = 0; i <= N; i++) {
        animator_sample(a, clip, step * i, pose);
        float move = 0.0f;
        for (int b = 0; b < B; b++) {
            mat4 w = animation_bone_world(a, *pose, b);
            vec3 q = v3(w.data[12], w.data[13], w.data[14]);
            if (i) move = maxf(move, v3dist(q, prev[b]));
            prev[b] = q;
        }
        if (i && move / step > STILL) settle = step * i;     // still moving here
    }
    pix_free(pose);
    pix_free(prev);
    return settle;
}

// When an attack connects: the moments any of `bones` reaches furthest
// forward. A double swipe has two such peaks; up to `max` are returned, in
// time order, at least a fifth of a second apart.
static int measure_strikes(const pix_model& m, idx clip, const int* bones, int n, float facing, float* out, int max) {
    animator a = pix_model_animator(m);
    float dur = animator_duration(a, clip);
    const int N = 64;
    float reach[N];
    animation* pose = pix_alloc<animation>(1);
    for (int i = 0; i < N; i++) {
        animator_sample(a, clip, dur * i / (N - 1), pose);
        reach[i] = -1e9f;
        for (int b = 0; b < n; b++)
            if (bones[b] >= 0) reach[i] = maxf(reach[i], animation_bone_world(a, *pose, bones[b]).data[14] * facing);
    }
    pix_free(pose);
    float lo = reach[0], hi = reach[0];
    for (int i = 1; i < N; i++) { lo = minf(lo, reach[i]); hi = maxf(hi, reach[i]); }
    int count = 0;
    // local peaks well forward of the rest pose, strongest first
    while (count < max && hi - lo > 1e-4f) {
        int best = -1;
        for (int i = 1; i < N - 1; i++) {
            if (reach[i] < reach[i - 1] || reach[i] < reach[i + 1] || reach[i] < lo + (hi - lo) * 0.35f) continue;
            bool taken = false;
            for (int k = 0; k < count; k++) taken |= fabsf(out[k] - dur * i / (N - 1)) < 0.2f;
            if (!taken && (best < 0 || reach[i] > reach[best])) best = i;
        }
        if (best < 0) break;
        out[count++] = dur * best / (N - 1);
    }
    if (!count) out[count++] = dur * 0.45f;
    if (count == 2 && out[1] < out[0]) { float t = out[0]; out[0] = out[1]; out[1] = t; }
    return count;
}

// ============================================================================
// ---- wolves ----
//
// assets/models/creatures/wolf_beast_1 is one rig with a clip for everything a
// wolf does: resting, prowling, howling, a dozen ways to claw, bite and tackle,
// leaps, hits and a death. The same model is the packs roaming the valley and
// the player's wolf form. As with the hero, gait speeds, which way the head
// points, the moments claws and jaws land and the leaps' timing are all read
// off the clips.

#define WOLF_PATH   "assets/models/creatures/wolf_beast_1/wolf_beast_1.glb"
#define RUNNER_PATH "assets/models/player/runner.glb"

enum { WC_IDLE, WC_WALK, WC_RUN, WC_AGGRO, WC_HOWL, WC_INTIMIDATE,
       WC_CLAW_L, WC_CLAW_R, WC_CLAW_DOUBLE, WC_CLAW_SLAM, WC_BITE_SNAP, WC_BITE_DOUBLE, WC_BITE_LUNGE,
       WC_TACKLE_LUNGE, WC_TACKLE_POUNCE, WC_TACKLE_RAM,
       WC_HIT_L, WC_HIT_R, WC_DEATH, WC_JUMP_BACK, WC_JUMP_FWD, WC_RUN_JUMP, WC_RUN_LEAP, WC_LAND,
       WC_LOOK, WC_REST, WC_EAT, WC_COUNT };
static const char* WOLF_CLIP_NAMES[WC_COUNT] = { "Idle", "Walk", "Run", "Aggro", "Howl", "Intimidate",
    "Claw_L", "Claw_R", "Claw_Double", "Claw_Slam", "Bite_Snap", "Bite_Double", "Bite_Lunge",
    "Tackle_Lunge", "Tackle_Pounce", "Tackle_Ram",
    "HitLeft", "HitRight", "Death", "JumpBack", "JumpForward", "Run_JumpForward", "Run_JumpLong", "Land",
    "LookAround", "Rest", "Eat" };
// stand-ins from the first export, for a file that lacks a clip
static const char* WOLF_CLIP_ALIASES[WC_COUNT] = { "Idle", "Walk", "Run", "Aggro", "Aggro", "Aggro",
    "ClawSwipe", "ClawSwipe", "ClawSwipe", "ClawSwipe", "Tackle", "Tackle", "Tackle",
    "Tackle", "Tackle", "Tackle",
    "HitLeft", "HitRight", "Death", "JumpBack", "JumpForward", "JumpForward", "JumpForward", "Idle",
    "LookAround", "Rest", "Eat" };

// Every attack, for the packs and the player alike. Damage is what a wild wolf
// deals the player; the player's wolf hits harder.
enum { ATK_CLAW_L, ATK_CLAW_R, ATK_BITE_SNAP, ATK_CLAW_DOUBLE, ATK_BITE_DOUBLE, ATK_SLAM,
       ATK_BITE_LUNGE, ATK_TACKLE_LUNGE, ATK_POUNCE, ATK_RAM, ATK_COUNT };
struct attack_def {
    int   clip;
    bool  jaws;         // lands with the head, else the front paws
    int   hits;         // strikes in the clip
    float reach;        // metres from the attacker's centre
    float cone;         // cos of the half-angle in front it covers (-1: all round)
    float damage;       // per strike
    float knock;        // m/s shove on the one hit
    float lunge;        // m/s spring forward at the target (0: a step instead, -1: keeps its running speed)
    float push;         // metres a swipe or bite steps in behind the blow
};
static const attack_def ATTACKS[ATK_COUNT] = {
    { WC_CLAW_L,        false, 1, 2.1f,  0.25f,  7.0f, 3.0f,  0.0f, 0.3f },
    { WC_CLAW_R,        false, 1, 2.1f,  0.25f,  7.0f, 3.0f,  0.0f, 0.3f },
    { WC_BITE_SNAP,     true,  1, 2.2f,  0.45f,  9.0f, 3.5f,  0.0f, 0.35f },
    { WC_CLAW_DOUBLE,   false, 2, 2.2f,  0.2f,   6.0f, 3.5f,  0.0f, 0.45f },
    { WC_BITE_DOUBLE,   true,  2, 2.2f,  0.45f,  6.0f, 3.0f,  0.0f, 0.45f },
    { WC_CLAW_SLAM,     false, 1, 2.9f, -1.0f,  12.0f, 7.0f,  0.0f, 0.25f },
    { WC_BITE_LUNGE,    true,  1, 2.4f,  0.45f, 11.0f, 5.0f,  6.0f, 0.0f },
    { WC_TACKLE_LUNGE,  true,  1, 2.4f,  0.4f,  13.0f, 8.0f,  8.0f, 0.0f },
    { WC_TACKLE_POUNCE, true,  1, 2.5f,  0.4f,  14.0f, 8.0f,  9.0f, 0.0f },
    { WC_TACKLE_RAM,    true,  1, 2.4f,  0.35f, 15.0f, 9.0f, -1.0f, 0.0f },
};

#define ROOT_SAMPLES 33

struct beast_rig {
    pix_model   model;
    idx         clip[WC_COUNT];
    float       len[WC_COUNT];     // clip durations
    float       death_settle;      // Death has come to rest: the body vanishes now
    gait        walk, run;
    float       facing;            // +1 when the head points toward +Z
    float       strike[ATK_COUNT][2];
    int         strikes[ATK_COUNT];
    jump_timing jump[WC_COUNT];    // filled for the leaps (and Land), read off the body's height
    // JumpForward and JumpBack move the whole body away from the origin (root
    // motion). How far forward it is over the clip, so the drawing can hold
    // the body on the physics position instead of popping back at the end.
    bool        travels[WC_COUNT];
    float       root[WC_COUNT][ROOT_SAMPLES];
};

static float root_travel(const beast_rig& b, int clip, float t) {
    if (clip < 0 || clip >= WC_COUNT || !b.travels[clip] || b.len[clip] <= 0.0f) return 0.0f;
    float f = clampf(t / b.len[clip], 0.0f, 1.0f) * (ROOT_SAMPLES - 1);
    int i = (int)f;
    if (i >= ROOT_SAMPLES - 1) return b.root[clip][ROOT_SAMPLES - 1];
    return lerpf(b.root[clip][i], b.root[clip][i + 1], f - i);
}

// How fast an attack carries the body forward at clip time `t` (m/s).
// Swipes and bites step in behind the blow: a surge that builds through the
// wind-up, peaks as it lands and dies away in the follow-through, covering
// `push` metres in all. Lunges and pounces spring at full speed and carry on a
// moment past the impact - or at the speed of the charge that led into them,
// if faster; a ram keeps the gallop it came in with (`carry`).
// `rate` is the clip's playback speed: the same step taken quicker is faster.
// Tackles are not thrown from a standstill: the attacker first sprints at the
// target and springs from the charge once close enough.
static bool is_tackle(int atk) { return atk == ATK_TACKLE_LUNGE || atk == ATK_POUNCE || atk == ATK_RAM; }

static float attack_push(const beast_rig& b, int atk, float t, float carry, float rate, float scale) {
    const attack_def& A = ATTACKS[atk];
    float first = b.strike[atk][0], last = b.strike[atk][b.strikes[atk] - 1];
    float follow = 1.0f - smoothstepf(first, first + 0.15f, t);
    if (A.lunge < 0.0f) return carry * follow;
    if (A.lunge > 0.0f) return maxf(A.lunge * scale * smoothstepf(0.0f, first, t), carry) * follow;
    float a = first * 0.35f, e = last + 0.22f;
    if (t <= a || t >= e) return 0.0f;
    float peak = A.push * scale * rate * PIX_PI / (2.0f * (e - a));   // the half sine covers `push`
    return peak * sinf(PIX_PI * (t - a) / (e - a));
}

static bool load_beast(beast_rig& b, const char* path) {
    b.model = pix_load_model(path);
    if (!b.model.skinned) return false;
    animator a = pix_model_animator(b.model);
    for (int i = 0; i < WC_COUNT; i++) {
        b.clip[i] = find_clip(a, WOLF_CLIP_NAMES[i], find_clip(a, WOLF_CLIP_ALIASES[i], 0));
        b.len[i] = animator_duration(a, b.clip[i]);
    }
    const char* HEAD[] = { "Head", "head" };
    const char* TAIL[] = { "Tail1", "tail", "Back" };
    const char* PAW_L[] = { "FF.L", "FrontFoot.L" };
    const char* PAW_R[] = { "FF.R", "FrontFoot.R" };
    const char* BODY[] = { "Body", "Hips", "pelvis" };
    int head = animator_find_bone(a, HEAD, 2), tail = animator_find_bone(a, TAIL, 3);
    int body = animator_find_bone(a, BODY, 3);
    int paws[2] = { animator_find_bone(a, PAW_L, 2), animator_find_bone(a, PAW_R, 2) };
    animation* pose = pix_alloc<animation>(1);
    b.facing = 1.0f;
    if (head >= 0 && tail >= 0) {
        animator_sample(a, b.clip[WC_IDLE], 0.0f, pose);
        b.facing = animation_bone_world(a, *pose, head).data[14] >= animation_bone_world(a, *pose, tail).data[14] ? 1.0f : -1.0f;
    }
    // the front paws carry the gait
    b.walk = measure_gait(b.model, b.clip[WC_WALK], paws[0], paws[1]);
    b.run  = measure_gait(b.model, b.clip[WC_RUN], paws[0], paws[1]);
    if (b.walk.speed < 0.3f) b.walk.speed = b.walk.clip_speed = 1.4f;
    if (b.run.speed < b.walk.speed * 1.5f) b.run.speed = b.run.clip_speed = b.walk.speed * 3.5f;
    for (int k = 0; k < ATK_COUNT; k++) {
        const attack_def& d = ATTACKS[k];
        b.strikes[k] = measure_strikes(b.model, b.clip[d.clip], d.jaws ? &head : paws, d.jaws ? 1 : 2,
                                       b.facing, b.strike[k], d.hits);
        if (!d.jaws && d.hits == 2 && b.strikes[k] < 2) {
            // a swipe with each paw: one strike per paw
            float l, r;
            measure_strikes(b.model, b.clip[d.clip], &paws[0], 1, b.facing, &l, 1);
            measure_strikes(b.model, b.clip[d.clip], &paws[1], 1, b.facing, &r, 1);
            if (fabsf(l - r) > 0.15f) { b.strike[k][0] = minf(l, r); b.strike[k][1] = maxf(l, r); b.strikes[k] = 2; }
        }
    }
    b.death_settle = measure_settle(b.model, b.clip[WC_DEATH]);
    const int LEAPS[] = { WC_JUMP_FWD, WC_RUN_JUMP, WC_RUN_LEAP };
    for (int k = 0; k < 3; k++) b.jump[LEAPS[k]] = measure_jump(b.model, b.clip[LEAPS[k]], body);
    // Land starts in the air: touching down at once, back on its feet a little past halfway
    b.jump[WC_LAND] = { 0.0f, 0.0f, 0.0f, 0.0f, b.len[WC_LAND] * 0.55f };
    if (body >= 0)
        for (int c = 0; c < WC_COUNT; c++) {
            float z0 = 0.0f;
            for (int i = 0; i < ROOT_SAMPLES; i++) {
                // (a time of exactly the duration wraps round to the first frame)
                animator_sample(a, b.clip[c], b.len[c] * 0.999f * i / (ROOT_SAMPLES - 1), pose);
                float z = animation_bone_world(a, *pose, body).data[14] * b.facing;
                if (!i) z0 = z;
                b.root[c][i] = z - z0;
            }
            // attacks sway forward and back again; only a clip that ends somewhere else travels
            b.travels[c] = fabsf(b.root[c][ROOT_SAMPLES - 1]) > 0.25f;
        }
    pix_free(pose);
    return true;
}

// ALERT: the one who spots you howls, the rest snarl. TAUNT: squares up while
// circling. LEAP: a long running bound that closes the gap.
enum { AI_IDLE, AI_WANDER, AI_ALERT, AI_CHASE, AI_TAUNT, AI_LEAP, AI_ATTACK, AI_DODGE, AI_HIT, AI_DEAD, AI_CHARGE };

struct wolf {
    idx   actor;
    vec3  pos, vel, home, goal;
    float heading;
    int   state, clip, pack;
    float timer;        // seconds in the current state
    float wait;         // how long this idle or wander lasts
    float hp;
    float cooldown;     // until it may attack again
    float flash;        // hit flash, fading
    float anger;        // seconds it keeps hunting after being hurt, however far the player runs
    float orbit;        // +1 / -1: which way it circles while waiting for an opening
    float respawn;      // dead: seconds until another takes its place
    float root;         // root motion being cancelled, eased out over a cross-fade
    int   attack;       // ATK_*
    int   struck;       // strikes of this attack resolved
    float carry;        // speed it came into the attack with (a ram keeps it)
    bool  charged;      // this attack came out of a run-up
    bool  gone;         // dead and vanished in smoke
    float gone_for;     // seconds since
};

// ============================================================================
// ---- the wolfmen ----
//
// assets/models/creatures/wolf_beast_1/wolf_man_1.glb: a two-metre werewolf
// on its own biped rig. A few roam the valley alone - the alphas. One that
// spots you roars, or howls and brings every pack in earshot running, then
// hunts upright, drops to all fours to gallop you down from afar, and up
// close squares off: circling, sidestepping your swings, backing off, and
// breaking in with claws, bites, headbutts, a leaping pounce and a
// three-blow combo. It shrugs off light blows mid-swing; enough damage at
// once staggers it.
//
// The loops are in place; dodges, the pounce, the leap back and the fall
// carry the body with them (root motion), which here moves the physics
// position so they respect trees and rocks.

#define WOLFMAN_PATH "assets/models/creatures/wolf_beast_1/wolf_man_1.glb"
#define WM_SCALE 1.1f      // drawn this much bigger than modelled
#define WM_PARTS 4

enum { MC_IDLE, MC_COMBAT_IDLE, MC_WALK, MC_WALK_BACK, MC_STRAFE_L, MC_STRAFE_R, MC_RUN, MC_RUN_BEAST,
       MC_CLAW_R, MC_CLAW_L, MC_DOUBLE, MC_BITE, MC_HEADBUTT, MC_POUNCE, MC_COMBO,
       MC_JUMP_BACK, MC_JUMP, MC_DODGE_L, MC_DODGE_R, MC_HIT, MC_HIT_HEAVY, MC_DEATH, MC_HOWL, MC_ROAR, MC_COUNT };
static const char* WOLFMAN_CLIP_NAMES[MC_COUNT] = { "Idle", "CombatIdle", "Walk", "WalkBack", "StrafeLeft", "StrafeRight",
    "Run", "Run_Beast", "Attack_ClawRight", "Attack_ClawLeft", "Attack_DoubleSlash", "Attack_Bite", "Attack_Headbutt",
    "Attack_Pounce", "Attack_Combo", "JumpBack", "Jump", "DodgeLeft", "DodgeRight", "Hit_Front", "Hit_Heavy", "Death",
    "Howl", "Roar" };

// When each blow lands, as authored: frame `hit` of a clip `frames` long,
// kept as a share of the clip so a re-export at another length still lines up.
// `fx` picks the look of the hit from the wolves' attacks.
enum { BA_CLAW_R, BA_CLAW_L, BA_DOUBLE, BA_BITE, BA_HEADBUTT, BA_POUNCE, BA_COMBO, BA_COUNT };
struct brute_attack_def {
    int   clip, frames, hits;
    int   hit[3];
    int   fx[3];
    float reach;        // metres from its centre, past the target's own radius
    float cone;         // cos of the half-angle in front it covers (-1: all round)
    float damage[3];
    float knock;
};
static const brute_attack_def BRUTE_ATTACKS[BA_COUNT] = {
    { MC_CLAW_R,    36, 1, { 15 },         { ATK_CLAW_R },                        1.9f,  0.3f, { 14 },         4.0f },
    { MC_CLAW_L,    36, 1, { 15 },         { ATK_CLAW_L },                        1.9f,  0.3f, { 14 },         4.0f },
    { MC_DOUBLE,    46, 1, { 21 },         { ATK_CLAW_DOUBLE },                   2.0f,  0.2f, { 22 },         6.0f },
    { MC_BITE,      38, 1, { 18 },         { ATK_BITE_SNAP },                     1.8f,  0.5f, { 18 },         3.5f },
    { MC_HEADBUTT,  38, 1, { 19 },         { ATK_TACKLE_LUNGE },                  1.7f,  0.5f, { 16 },         9.0f },
    { MC_POUNCE,    50, 1, { 31 },         { ATK_SLAM },                          2.2f, -1.0f, { 24 },         8.0f },
    { MC_COMBO,     62, 3, { 12, 26, 44 }, { ATK_CLAW_L, ATK_CLAW_R, ATK_BITE_SNAP }, 1.9f, 0.3f, { 10, 10, 16 }, 3.0f },
};

struct brute_rig {
    pix_model   model;
    idx         part_mesh[WM_PARTS], part_mat[WM_PARTS];   // body, eyes, mouth: one material each
    int         parts;
    idx         clip[MC_COUNT];
    float       len[MC_COUNT];
    float       death_settle;      // Death has come to rest: the body vanishes now
    gait        walk, back, strafe, run, gallop;
    jump_timing jump;
    float       strike[BA_COUNT][3];
    // how far the hips have moved off the origin (x, z in model space) over
    // a clip that travels
    bool        travels[MC_COUNT];
    vec2        root[MC_COUNT][ROOT_SAMPLES];
};

static vec2 brute_root(const brute_rig& b, int clip, float t) {
    if (clip < 0 || clip >= MC_COUNT || !b.travels[clip] || b.len[clip] <= 0.0f) return v2(0, 0);
    float f = clampf(t / b.len[clip], 0.0f, 1.0f) * (ROOT_SAMPLES - 1);
    int i = (int)f;
    if (i >= ROOT_SAMPLES - 1) return b.root[clip][ROOT_SAMPLES - 1];
    vec2 p = b.root[clip][i], q = b.root[clip][i + 1];
    float k = f - i;
    return v2(lerpf(p.x, q.x, k), lerpf(p.y, q.y, k));
}

// a model-space offset (x left, z ahead) turned to a heading
static vec3 yaw_rotate(vec2 local, float heading) {
    float c = cosf(heading), s = sinf(heading);
    return v3(local.x * c + local.y * s, 0.0f, -local.x * s + local.y * c);
}

static bool load_brute(brute_rig& b, const char* path) {
    b.model = pix_load_model(path);
    if (!b.model.skinned) return false;
    // Fur, glowing eyes and the inside of the mouth are three materials, and
    // a merged draw wears only the fur texture: each is loaded on its own
    // and all are drawn with the same bones.
    pix_renderer& r = pix_get_renderer();
    pix_data_loader& l = pix_get_loader();
    pix_loader_mark mark = pix_loader_mark_now(l);       // only the GPU copies are kept
    int mats[WM_PARTS];
    int mc = gltf_list_materials(l.arena, path, -1, mats, WM_PARTS);
    b.parts = 0;
    for (int i = 0; i < mc && b.parts < WM_PARTS; i++) {
        idx id = load_model_gltf_file(l, path, -1, mats[i]);
        if (id == PIX_INVALID) continue;
        model_file_data& mf = l.model_files[id];
        skinned_mesh_file_data* md = get_model_mesh(l, id);
        if (!md) continue;
        image_file_data* im = mf.image >= 0 ? &l.image_files[mf.image] : 0;
        b.part_mesh[b.parts] = load_skinned_mesh(r, *md);
        b.part_mat[b.parts] = load_material_image(r, im, v3(1, 1, 1), 0.0f, 0.8f, mf.image_is_palette);
        // the loader reads no emission: a flat-coloured part as bright as the
        // eyes are is taken to be them, and glows
        bool glow = mf.image_is_palette && im && im->data
                 && maxf((float)(uint8_t)im->data[0], maxf((float)(uint8_t)im->data[1], (float)(uint8_t)im->data[2])) > 100.0f;
        if (material* m = pix_get_material(r, b.part_mat[b.parts])) {
            m->params1 = v4(0.35f, glow ? 4.0f : 0.0f, 0.0f, 0.0f);
            if (!mf.image_is_palette) m->color = v3(1.35f, 1.3f, 1.25f);   // dark fur, lifted enough to read against the grass
            if (glow) m->cast_shadows = false;
        }
        b.parts++;
    }
    pix_loader_rewind(l, mark);
    if (!b.parts) { b.part_mesh[0] = b.model.mesh; b.part_mat[0] = b.model.material; b.parts = 1; }

    animator a = pix_model_animator(b.model);
    for (int i = 0; i < MC_COUNT; i++) {
        b.clip[i] = find_clip(a, WOLFMAN_CLIP_NAMES[i], find_clip(a, i >= MC_CLAW_R && i <= MC_COMBO ? "Attack_ClawRight" : "Idle", 0));
        b.len[i] = animator_duration(a, b.clip[i]);
    }
    const char* HIPS[] = { "hips", "Hips", "pelvis" };
    const char* FOOT_L[] = { "foot.L", "LeftFoot" };
    const char* FOOT_R[] = { "foot.R", "RightFoot" };
    int hips = animator_find_bone(a, HIPS, 3);
    int fl = animator_find_bone(a, FOOT_L, 2), fr = animator_find_bone(a, FOOT_R, 2);
    b.walk   = measure_gait(b.model, b.clip[MC_WALK], fl, fr);
    b.back   = measure_gait(b.model, b.clip[MC_WALK_BACK], fl, fr);
    b.strafe = measure_gait(b.model, b.clip[MC_STRAFE_L], fl, fr, 12);
    b.run    = measure_gait(b.model, b.clip[MC_RUN], fl, fr);
    b.gallop = measure_gait(b.model, b.clip[MC_RUN_BEAST], fl, fr);
    if (b.gallop.speed < b.run.speed * 1.3f) {
        // on all fours the hind feet land together and barely sweep back:
        // the front paws (its hands) carry the stride
        const char* HAND_L[] = { "hand.L", "LeftHand" };
        const char* HAND_R[] = { "hand.R", "RightHand" };
        gait h = measure_gait(b.model, b.clip[MC_RUN_BEAST], animator_find_bone(a, HAND_L, 2), animator_find_bone(a, HAND_R, 2));
        if (h.speed > b.gallop.speed) { h.step_count = b.gallop.step_count; for (int i = 0; i < 4; i++) h.steps[i] = b.gallop.steps[i]; b.gallop = h; }
        if (b.gallop.speed < b.run.speed * 1.3f) b.gallop.speed = b.gallop.clip_speed = b.run.speed * 1.6f;
    }
    gait* all[] = { &b.walk, &b.back, &b.strafe, &b.run, &b.gallop };
    const float fallback[] = { 1.3f, 1.0f, 1.1f, 3.8f, 5.5f };
    for (int i = 0; i < 5; i++) {
        if (all[i]->speed < 0.3f) all[i]->speed = all[i]->clip_speed = fallback[i];
        all[i]->speed *= WM_SCALE;
        all[i]->clip_speed *= WM_SCALE;
    }
    b.jump = measure_jump(b.model, b.clip[MC_JUMP], hips);
    b.death_settle = measure_settle(b.model, b.clip[MC_DEATH]);
    for (int k = 0; k < BA_COUNT; k++) {
        const brute_attack_def& d = BRUTE_ATTACKS[k];
        for (int h = 0; h < d.hits; h++) b.strike[k][h] = b.len[d.clip] * d.hit[h] / (float)d.frames;
    }
    animation* pose = pix_alloc<animation>(1);
    if (hips >= 0)
        for (int c = 0; c < MC_COUNT; c++) {
            vec2 r0 = v2(0, 0);
            for (int i = 0; i < ROOT_SAMPLES; i++) {
                // (a time of exactly the duration wraps round to the first frame)
                animator_sample(a, b.clip[c], b.len[c] * 0.999f * i / (ROOT_SAMPLES - 1), pose);
                mat4 w = animation_bone_world(a, *pose, hips);
                if (!i) r0 = v2(w.data[12], w.data[14]);
                b.root[c][i] = v2(w.data[12] - r0.x, w.data[14] - r0.y);
            }
            vec2 e = b.root[c][ROOT_SAMPLES - 1];
            b.travels[c] = sqrtf(e.x * e.x + e.y * e.y) > 0.25f;
        }
    pix_free(pose);
    return true;
}

// ROAR: squaring up (or the howl that calls the packs). LEAP: a bound in
// over the gap. DODGE: a sidestep or a leap back. STAGGER: poise broken.
enum { BR_IDLE, BR_WANDER, BR_ROAR, BR_CHASE, BR_LEAP, BR_ATTACK, BR_DODGE, BR_HIT, BR_STAGGER, BR_DEAD };

struct brute {
    idx   actor;
    vec3  pos, vel, home, goal;
    float heading;
    int   state, clip;
    float timer, wait;
    float hp;
    float cooldown;     // until it may attack again
    float flash;
    float anger;        // seconds it keeps hunting however far the player runs
    float orbit;        // +1 / -1: which way it circles
    float poise;        // recent damage: past a limit it staggers
    float respawn;
    float leap;         // m/s it bounds across the gap at
    float root_scale;   // how much further than authored a root-motion clip carries it
    vec2  root_prev;    // root offset already applied to the position
    vec2  root_draw;    // root offset cancelled when drawing, eased out after the clip
    int   attack, struck;
    bool  gone;
    float gone_for;     // seconds since it vanished
};

// the villager wandering the meadow
struct npc {
    idx   actor;
    vec3  pos, vel, goal;
    float heading, timer, wait;
    int   clip;
};

// ---- a few hundred lit puffs: fur, dust and the transformation ----
struct particle {
    vec3  pos, vel;
    float life, max_life, size;
    pix_color color;
    bool  glow;         // a hot spark: additive, weightless, fades out instead of shrinking
};
#define MAX_PUFFS 1536

// ---- hit effects: glowing slashes, bite flashes and impact stars ----
// A streak is a thin bar of light swept out along `axis`: it grows to full
// length over the first quarter of its life, then thins away.
struct streak {
    vec3  start, axis, up;
    float len, width, life, max_life;
    pix_color color;
};
#define MAX_STREAKS 96

// the damage dealt, floating up from the one who took it
struct popup {
    vec3  pos;
    float life;
    int   amount;
    float size;
    pix_color color;
};
#define MAX_POPUPS 32

static void burst(particle* ps, int& n, rng& rg, vec3 at, int count, float speed, float size, pix_color c, float up) {
    for (int i = 0; i < count && n < MAX_PUFFS; i++) {
        particle& p = ps[n++];
        float a = rng_range(rg, 0.0f, PIX_TAU), e = rng_range(rg, -0.3f, 1.0f);
        p.pos = at + v3(rng_range(rg, -0.2f, 0.2f), rng_range(rg, -0.2f, 0.2f), rng_range(rg, -0.2f, 0.2f));
        p.vel = v3(cosf(a) * cosf(e), sinf(e) + up, sinf(a) * cosf(e)) * (speed * rng_range(rg, 0.4f, 1.0f));
        p.max_life = p.life = rng_range(rg, 0.35f, 0.8f);
        p.size = size * rng_range(rg, 0.6f, 1.2f);
        p.color = c;
        p.glow = false;
    }
}

// hot sparks thrown out flat from `at`: a shockwave ring
static void ring(particle* ps, int& n, rng& rg, vec3 at, int count, float speed, float size, pix_color c) {
    for (int i = 0; i < count && n < MAX_PUFFS; i++) {
        particle& p = ps[n++];
        float a = PIX_TAU * (i + rng_float(rg) * 0.4f) / count;
        p.pos = at;
        p.vel = v3(cosf(a), rng_range(rg, 0.0f, 0.25f), sinf(a)) * (speed * rng_range(rg, 0.85f, 1.1f));
        p.max_life = p.life = rng_range(rg, 0.22f, 0.32f);
        p.size = size;
        p.color = c;
        p.glow = true;
    }
}
// hot sparks sprayed along `dir`
static void sparks(particle* ps, int& n, rng& rg, vec3 at, vec3 dir, int count, float speed, float size, pix_color c) {
    for (int i = 0; i < count && n < MAX_PUFFS; i++) {
        particle& p = ps[n++];
        vec3 j = v3(rng_range(rg, -1.0f, 1.0f), rng_range(rg, -0.4f, 1.0f), rng_range(rg, -1.0f, 1.0f));
        p.pos = at;
        p.vel = v3norm(dir + j * 0.8f) * (speed * rng_range(rg, 0.5f, 1.0f));
        p.max_life = p.life = rng_range(rg, 0.15f, 0.3f);
        p.size = size * rng_range(rg, 0.6f, 1.1f);
        p.color = c;
        p.glow = true;
    }
}
static void add_streak(streak* ss, int& n, vec3 start, vec3 axis, vec3 up, float len, float width, float life, pix_color c) {
    if (n >= MAX_STREAKS) return;
    streak& s = ss[n++];
    s.start = start; s.axis = axis; s.up = up;
    s.len = len; s.width = width;
    s.life = s.max_life = life;
    s.color = c;
}

// ============================================================================
// ---- the look ----

static void apply_sky(pix_renderer& r, float hour) {
    pix_time_of_day(hour);
    // clear storybook daylight: a deep blue dome fading to a pale, slightly
    // green-blue horizon, and the distance melting into a soft blue haze
    float day = smoothstepf(6.5f, 9.0f, hour) * (1.0f - smoothstepf(16.5f, 19.0f, hour));
    if (day > 0.0f) {
        vec3 zen = v3lerp(r.sky.zenith, v3(0.03f, 0.15f, 0.55f), day);
        vec3 hor = v3lerp(r.sky.horizon, v3(0.40f, 0.60f, 0.80f), day);
        pix_set_sky_colors(r, zen, hor, v3lerp(r.sky.ground, v3(0.10f, 0.16f, 0.07f), day));
    }
    // light and blue: distance should read as air, not as milk
    vec3 haze = v3lerp(v3(0.86f, 0.74f, 0.62f), v3(0.52f, 0.68f, 0.86f), day);
    pix_set_fog(r, haze, 0.0042f);
}

// uLookToon lives in each program's own uniform state, shared by every
// material drawn with it
static void set_look_toon(const idx* programs, int count, float toon) {
    for (int i = 0; i < count; i++) {
        if (!programs[i]) continue;
        glUseProgram(programs[i]);
        glUniform1f(glGetUniformLocation(programs[i], "uLookToon"), toon);
    }
    glUseProgram(0);
}

int main() {
    if (!pix_init("pix - grassland", 1280, 720)) return 1;
    pix_renderer& r = pix_get_renderer();
    choose_ponds(5);                      // before anything reads the terrain
    pix_log("ponds: %d at water level %.2f", g_pond_count, g_water_level);
    pix_shadows().distance = 65.0f;
    pix_shadows().softness = 0;           // hardware 2x2: crisp edges suit cel shading, and grass is a lot of pixels
    pix_shadows().strength = 0.9f;
    pix_shadows().stagger = true;         // the far cascades on alternate frames
    pix_shadows().skinned_cascades = 2;   // a wolf 40 m off casts no shadow worth drawing
    pix_shadows().far_resolution = 0.6f;  // the far cascades at 60% resolution: they are fill, not detail
    pix_post().bloom_strength = 0.18f;
    pix_post().saturation = 1.0f;
    pix_post().contrast = 0.06f;
    pix_post().vignette = 0.18f;
    float hour = 10.0f;
    apply_sky(r, hour);

    // ---- field map: height, grass density and the biomes on one grid ----
    // The terrain mesh, the grass roots and the ground colour all read this
    // one grid, so nothing floats and turf lines up with the grass on it.
    const int N = GRID + 1;
    float* field = pix_alloc<float>(N * N * 4);
    float forest_share = 0.0f, rocky_share = 0.0f;
    for (int z = 0; z < N; z++)
        for (int x = 0; x < N; x++) {
            float wx = -TERRAIN + 2.0f * TERRAIN * x / GRID, wz = -TERRAIN + 2.0f * TERRAIN * z / GRID;
            float* f = &field[(z * N + x) * 4];
            f[0] = terrain_height(wx, wz);
            f[1] = grass_density(wx, wz);
            f[2] = forest_amount(wx, wz);
            f[3] = rocky_amount(wx, wz);
            forest_share += f[2] > 0.5f;
            rocky_share += f[3] > 0.5f;
        }
    pix_log("biomes: %.0f%% forest, %.0f%% rocky", 100.0f * forest_share / (N * N), 100.0f * rocky_share / (N * N));
    // uv = xz * scale + offset, landing on texel centres
    vec2 field_xf = v2(0.5f / N + TERRAIN / (2.0f * TERRAIN) * GRID / N, (float)GRID / N / (2.0f * TERRAIN));
    GLuint field_tex;
    glGenTextures(1, &field_tex);
    glBindTexture(GL_TEXTURE_2D, field_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, N, N, 0, GL_RGBA, GL_FLOAT, field);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);

    // metres from the nearest shore on the same grid: the ground's lakebed and wet banks
    float* shore = pix_alloc<float>(N * N);
    for (int z = 0; z < N; z++)
        for (int x = 0; x < N; x++)
            shore[z * N + x] = clampf(pond_shore(-TERRAIN + 2.0f * TERRAIN * x / GRID, -TERRAIN + 2.0f * TERRAIN * z / GRID), -8.0f, 8.0f);
    GLuint shore_tex;
    glGenTextures(1, &shore_tex);
    glBindTexture(GL_TEXTURE_2D, shore_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, N, N, 0, GL_RED, GL_FLOAT, shore);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    pix_free(shore);

    // ---- terrain mesh ----
    vertex* verts = pix_alloc<vertex>(N * N);
    uint32_t* inds = pix_alloc<uint32_t>(GRID * GRID * 6);
    for (int z = 0; z < N; z++)
        for (int x = 0; x < N; x++) {
            float wx = -TERRAIN + 2.0f * TERRAIN * x / GRID, wz = -TERRAIN + 2.0f * TERRAIN * z / GRID;
            vertex& v = verts[z * N + x];
            v.position = v3(wx, field[(z * N + x) * 4], wz);
            float e = 0.5f;
            v.normal = v3norm(v3(terrain_height(wx - e, wz) - terrain_height(wx + e, wz), 2.0f * e,
                                 terrain_height(wx, wz - e) - terrain_height(wx, wz + e)));
            v.uv = v2(wx * 0.25f, wz * 0.25f);
        }
    int ni = 0;
    for (int z = 0; z < GRID; z++)
        for (int x = 0; x < GRID; x++) {
            uint32_t a = z * N + x, b = a + 1, c = a + N, d = c + 1;
            inds[ni++] = a; inds[ni++] = c; inds[ni++] = b;
            inds[ni++] = b; inds[ni++] = c; inds[ni++] = d;
        }
    idx terrain = load_mesh_data(r, verts, N * N, inds, ni);
    // The hills' shadows come from a coarse copy (every third grid point, a
    // ninth of the triangles), sunk a little so that where it cuts corners it
    // stays under the real ground rather than shading it from inside.
    const int SC = 3, SN = GRID / SC + 1;
    vertex* sverts = pix_alloc<vertex>(SN * SN);
    for (int z = 0; z < SN; z++)
        for (int x = 0; x < SN; x++) {
            sverts[z * SN + x] = verts[(z * SC) * N + x * SC];
            sverts[z * SN + x].position.y -= 0.4f;
        }
    int sni = 0;
    for (int z = 0; z < SN - 1; z++)
        for (int x = 0; x < SN - 1; x++) {
            uint32_t a = z * SN + x, b = a + 1, c = a + SN, d = c + 1;
            inds[sni++] = a; inds[sni++] = c; inds[sni++] = b;
            inds[sni++] = b; inds[sni++] = c; inds[sni++] = d;
        }
    idx terrain_shadow = load_mesh_data(r, sverts, SN * SN, inds, sni);
    pix_free(sverts);
    pix_free(verts);
    pix_free(inds);
    idx ground_shader = load_surface_shader(r, GROUND_FS);
    if (!ground_shader) { pix_log("ground shader failed"); pix_shutdown(); return 1; }
    material gm = pix_material_default();
    gm.shader = ground_shader;
    gm.params0 = v4(field_xf.x, field_xf.y, 0, 0);
    gm.cast_shadows = false;                  // the coarse copy casts instead
    idx soil = pix_add_material(r, gm);
    pix_material_set_texture(r, soil, 1, field_tex);
    pix_material_set_texture(r, soil, 2, shore_tex);

    // ---- ponds: one surface each, a little past the shore (the banks hide the rest) ----
    idx pond_shader = load_water_shader(r, POND_FS);
    if (!pond_shader) { pix_log("pond shader failed"); pix_shutdown(); return 1; }
    material pond_mat = pix_material_water(r, v3(0.42f, 0.76f, 0.66f), v3(0.04f, 0.2f, 0.27f), 1.6f);
    pond_mat.shader = pond_shader;
    pond_mat.params1 = v4(1.4f, 0.6f, 1.0f, 0.7f);   // broad slow ripples: still water
    pond_mat.refraction = 0.05f;
    pond_mat.planar_reflection = 1.0f;
    idx water = pix_add_material(r, pond_mat);
    // Each pond's surface: a 1 m grid over it, keeping the cells in or near
    // the water (past the shore the banks hide it), so it takes any shape.
    idx pond_mesh[MAX_PONDS];
    for (int i = 0; i < g_pond_count; i++) {
        const pond& pd = g_ponds[i];
        int side = (int)ceilf(2.0f * (pd.bound + 2.0f)) + 1;
        float x0 = pd.x - (pd.bound + 2.0f), z0 = pd.z - (pd.bound + 2.0f);
        vertex* pv = pix_alloc<vertex>((side + 1) * (side + 1));
        float* sd = pix_alloc<float>((side + 1) * (side + 1));
        uint32_t* pi = pix_alloc<uint32_t>(side * side * 6);
        for (int z = 0; z <= side; z++)
            for (int x = 0; x <= side; x++) {
                vertex& v = pv[z * (side + 1) + x];
                v.position = v3(x0 + x, g_water_level, z0 + z);
                v.normal = v3(0, 1, 0);
                v.uv = v2(v.position.x, v.position.z);
                sd[z * (side + 1) + x] = pond_distance(pd, v.position.x, v.position.z);
            }
        int ni = 0;
        for (int z = 0; z < side; z++)
            for (int x = 0; x < side; x++) {
                uint32_t a = z * (side + 1) + x, b = a + 1, c = a + side + 1, d = c + 1;
                if (minf(minf(sd[a], sd[b]), minf(sd[c], sd[d])) > 1.2f) continue;
                pi[ni++] = a; pi[ni++] = c; pi[ni++] = b;
                pi[ni++] = b; pi[ni++] = c; pi[ni++] = d;
            }
        pond_mesh[i] = load_mesh_data(r, pv, (side + 1) * (side + 1), pi, ni);
        pix_free(pv);
        pix_free(sd);
        pix_free(pi);
    }
    // one mirror at the water level serves every pond; half resolution is
    // plenty under ripples, and it is only drawn while a pond is in sight
    if (g_pond_count) {
        r.reflections.planar = true;
        r.reflections.planar_scale = 0.5f;
        r.reflections.planar_distance = 90.0f;
        pix_set_reflection_plane(r, v3(0, 1, 0), v3(0, g_water_level, 0));
    }
    material sm = pix_material_default();
    sm.shadow_only = true;
    idx soil_shadow = pix_add_material(r, sm);

    // ---- props ----
    idx tree_shader = load_surface_shader(r, MODEL_FS, TREE_VS);
    idx model_shader = load_surface_shader(r, MODEL_FS);
    idx hero_shader = load_skinned_surface_shader(r, MODEL_FS);
    if (!tree_shader || !model_shader || !hero_shader) { pix_log("model shaders failed"); pix_shutdown(); return 1; }
    static const char* TREE_FILES[] = {
        "assets/models/trees/oak/oak_01.glb",       "assets/models/trees/oak/oak_02.glb",
        "assets/models/trees/oak/oak_03.glb",       "assets/models/trees/pine/pine_01.glb",
        "assets/models/trees/pine/pine_02.glb",     "assets/models/trees/pine/pine_03.glb",
        "assets/models/trees/willow/willow_01.glb", "assets/models/trees/willow/willow_02.glb",
        "assets/models/trees/columnar/columnar_01.glb", "assets/models/trees/columnar/columnar_02.glb",
        "assets/models/trees/swiggly/swiggly_01.glb", "assets/models/trees/swiggly/swiggly_02.glb",
        "assets/models/trees/oak/oak_dead_01.glb",  "assets/models/trees/pine/pine_dead_03.glb",
    };
    static const char* ROCK_FILES[] = {
        "assets/models/rocks/boulders/rock_boulder_01.glb", "assets/models/rocks/boulders/rock_boulder_02.glb",
        "assets/models/rocks/boulders/rock_boulder_03.glb", "assets/models/rocks/boulders/rock_boulder_04.glb",
        "assets/models/rocks/cliffs/rock_cliff_01.glb",     "assets/models/rocks/cliffs/rock_cliff_02.glb",
        "assets/models/rocks/cliffs/rock_cliff_03.glb",     "assets/models/rocks/large/rock_large_01.glb",
        "assets/models/rocks/pillars/rock_pillar_01.glb",   "assets/models/rocks/pillars/rock_pillar_02.glb",
        "assets/models/rocks/pebbles/rock_pebble_01.glb",   "assets/models/rocks/pebbles/rock_pebble_02.glb",
        "assets/models/rocks/pebbles/rock_pebble_03.glb",
    };
    const int TREE_KINDS = sizeof(TREE_FILES) / sizeof(TREE_FILES[0]);
    const int ROCK_KINDS = sizeof(ROCK_FILES) / sizeof(ROCK_FILES[0]);
    const int KINDS = TREE_KINDS + ROCK_KINDS;
    prop_kind* kinds = pix_alloc<prop_kind>(KINDS);
    for (int i = 0; i < TREE_KINDS; i++) kinds[i] = load_prop(TREE_FILES[i], tree_shader);
    for (int i = 0; i < ROCK_KINDS; i++) kinds[TREE_KINDS + i] = load_prop(ROCK_FILES[i], model_shader);
    // rocks come out of the palette path as a flat cool grey; warm them toward sandstone
    for (int i = TREE_KINDS; i < KINDS; i++)
        for (int p = 0; p < kinds[i].parts; p++)
            if (material* m = pix_get_material(r, kinds[i].material[p])) m->color = v3(0.78f, 0.72f, 0.62f);

    const int MAX_PROPS = 1400;
    prop* props = pix_alloc<prop>(MAX_PROPS);
    obstacle* obstacles = pix_alloc<obstacle>(MAX_PROPS);
    int prop_count = 0, obstacle_count = 0;
    {
        rng rg = rng_seed(11);
        // Place `count` of kinds [first, first+span) with spacing, keeping the
        // spawn clear. `where` is 0..1: how likely a spot is to take one.
        enum { ANYWHERE, FOREST, GRASSLAND, ROCKY, ROCKY_OR_EDGE };
        auto weight = [&](int where, float x, float z) {
            float f = forest_amount(x, z), rk = rocky_amount(x, z);
            switch (where) {
            case FOREST:        return f * f;
            case GRASSLAND:     return (1.0f - f) * (1.0f - rk) * 0.25f;
            case ROCKY:         return rk;
            case ROCKY_OR_EDGE: return maxf(rk, 0.08f);
            default:            return 1.0f;
            }
        };
        auto scatter = [&](int first, int span, int count, float lo_h, float hi_h, bool fixed_scale,
                           int where, float max_edge, float spacing, bool solid) {
            for (int n = 0, tries = 0; n < count && tries < count * 200 && prop_count < MAX_PROPS; tries++) {
                float x = rng_range(rg, -max_edge, max_edge), z = rng_range(rg, -max_edge, max_edge);
                if (x * x + z * z < 12.0f * 12.0f || rng_float(rg) > weight(where, x, z)) continue;
                int k = first + (int)(rng_float(rg) * span) % span;
                if (!kinds[k].parts || pond_shore(x, z) < 2.5f) continue;     // nothing grows in the water
                float s = fixed_scale ? rng_range(rg, lo_h, hi_h)
                                      : rng_range(rg, lo_h, hi_h) / maxf(kinds[k].height, 0.1f);
                float rad = kinds[k].footprint * s;
                bool clear = true;
                for (int o = 0; o < prop_count && clear; o++) {
                    float dx = props[o].pos.x - x, dz = props[o].pos.z - z;
                    float need = props[o].radius + rad + spacing;
                    if (props[o].radius > 0.0f && dx * dx + dz * dz < need * need) clear = false;
                }
                if (!clear) continue;
                // sit on the lowest point under the footprint so nothing floats on a slope
                float y = terrain_height(x, z);
                float fr = maxf(rad * 0.6f, 0.3f);
                y = minf(y, minf(minf(terrain_height(x + fr, z), terrain_height(x - fr, z)),
                                 minf(terrain_height(x, z + fr), terrain_height(x, z - fr))));
                prop& p = props[prop_count++];
                p.kind = k;
                // lowest vertex on the lowest ground, sunk a little so roots never show daylight
                p.pos = v3(x, y - (kinds[k].base + 0.15f) * s, z);
                p.yaw = rng_range(rg, 0.0f, PIX_TAU);
                p.scale = s;
                p.radius = solid ? rad : 0.0f;
                n++;
            }
        };
        int R = TREE_KINDS;
        // kinds: 0-2 oak, 3-5 pine, 6-7 willow, 8-9 columnar, 10-11 swiggly, 12-13 dead
        const float E = FIELD;
        scatter(0, 6, 380, 13.0f, 21.0f, false, FOREST, E, 0.8f, true);       // forest: oaks and pines, close together
        scatter(8, 2, 100, 12.0f, 18.0f, false, FOREST, E, 0.8f, true);       // columnar trees among them
        scatter(6, 6, 40, 11.0f, 16.0f, false, GRASSLAND, E, 10.0f, true);    // lone willows and swiggly trees in the meadows
        scatter(12, 2, 30, 0.7f, 1.1f, true, ROCKY, E, 3.0f, true);           // dead trees on the barrens
        scatter(R + 4, 4, 70, 1.2f, 2.4f, true, ROCKY, E, 1.0f, true);        // cliffs and the big rock
        scatter(R + 8, 2, 6, 0.8f, 1.3f, true, ROCKY, E, 12.0f, true);        // a few stone pillars
        scatter(R + 0, 4, 90, 0.6f, 1.8f, true, ROCKY_OR_EDGE, E, 2.0f, true);    // boulders, mostly on the barrens
        scatter(R + 10, 3, 200, 0.8f, 1.8f, true, ROCKY_OR_EDGE, E, 0.0f, false); // pebbles, walked over
        pix_log("props: %d", prop_count);
        for (int i = 0; i < prop_count; i++)
            if (props[i].radius > 0.0f) {
                // trunks are narrower than branches reaching down; rocks are about their footprint
                float r0 = props[i].kind < TREE_KINDS ? minf(props[i].radius * 0.35f, 0.9f * props[i].scale + 0.3f)
                                                      : props[i].radius * 0.8f;
                obstacles[obstacle_count++] = { props[i].pos.x, props[i].pos.z, r0 };
            }
    }
    // every tree can be felled; tops[] is how high each obstacle stands, for the falling ones
    tree_body* trees = pix_alloc<tree_body>(prop_count);
    int* prop_tree = pix_alloc<int>(prop_count);
    float* tops = pix_alloc<float>(obstacle_count + 1);
    int tree_count = 0;
    float tree_clock = 0.0f;         // the fixed step's leftover between frames
    // each tree material again for every run of slabs a broken piece can be
    for (int i = 0; i < TREE_KINDS; i++) {
        prop_kind& k = kinds[i];
        if (!k.parts) continue;
        for (int q = 0; q < k.parts; q++)
            for (int lo = 0; lo < CUTS; lo++)
                for (int hi = lo + 1; hi < CUTS; hi++) {
                    material m = *pix_get_material(r, k.material[q]);
                    m.params0.z = k.base + k.cut[lo] * k.height;
                    m.params0.w = k.base + k.cut[hi] * k.height;
                    k.slab[q][slab_index(lo, hi)] = pix_add_material(r, m);
                }
    }
    for (int i = 0, o = 0; i < prop_count; i++) {
        prop_tree[i] = -1;
        const prop& p = props[i];
        const prop_kind& k = kinds[p.kind];
        if (p.radius <= 0.0f) continue;
        tops[o] = p.pos.y + (k.base + k.height) * p.scale;
        if (p.kind < TREE_KINDS) {
            vec3 root = v3(p.pos.x, p.pos.y + k.base * p.scale, p.pos.z);
            prop_tree[i] = tree_count;
            tree_init(trees[tree_count++], i, o, p.kind, &k, mat4_trs_y(p.pos, p.yaw, p.scale), root,
                      k.height * p.scale, obstacles[o].r);
        }
        o++;
    }

    // ---- grass: patches planted by the density map ----
    grass g = {};
    g.field = field_tex;
    g.field_xf = field_xf;
    g.lod_dist[0] = 11.0f; g.lod_dist[1] = 26.0f; g.lod_dist[2] = 60.0f;
    g.toon = 1.0f;
    g.width = 1.8f;
    {
        rng rg = rng_seed(3);
        if (!load_grass_blades(g, rg)) { pix_log("could not load %s", GRASS_PATH); pix_shutdown(); return 1; }
        // tight enough, with only partial jitter, that neighbouring patches
        // always overlap: a patch is a ragged disc, not a square tile
        const float SPACING = 1.2f;
        int side = (int)(2.0f * FIELD / SPACING);
        gpu_patch* patches = pix_alloc<gpu_patch>(side * side);
        uint32_t n = 0;
        for (int iz = 0; iz < side; iz++)
            for (int ix = 0; ix < side; ix++) {
                float x = -FIELD + (ix + 0.2f + 0.6f * rng_float(rg)) * SPACING, z = -FIELD + (iz + 0.2f + 0.6f * rng_float(rg)) * SPACING;
                float dens = grass_density(x, z);
                if (dens < 0.12f) continue;                           // bare earth: no grass at all
                bool blocked = false;                    // nothing grows through a rock or a trunk
                for (int o = 0; o < obstacle_count && !blocked; o++) {
                    float dx = x - obstacles[o].x, dz = z - obstacles[o].z, rr = obstacles[o].r + 0.6f;
                    blocked = dx * dx + dz * dz < rr * rr;
                }
                if (blocked) continue;
                gpu_patch& p = patches[n++];
                p.a[0] = x; p.a[1] = z; p.a[2] = rng_range(rg, 0.0f, PIX_TAU); p.a[3] = rng_float(rg);
                p.b[0] = rng_range(rg, 0.8f, 1.1f);
                p.b[1] = rng_range(rg, 1.05f, 1.4f) * (0.3f + 0.7f * smoothstepf(0.08f, 0.6f, dens))   // short at the edges
                       * (1.0f - 0.35f * forest_amount(x, z));                                          // and under the trees
                p.b[2] = p.b[3] = 0.0f;
            }
        g.patch_count = n;
        g.patches = pix_create_gpu_buffer(sizeof(gpu_patch) * n, patches);
        pix_free(patches);
        pix_log("grass: %u patches", n);
    }
    pix_free(field);
    g.visible = pix_create_gpu_buffer(sizeof(uint32_t) * g.patch_count * GRASS_LODS);
    g.args = pix_create_gpu_buffer(sizeof(pix_draw_elements_indirect) * GRASS_LODS);
    g.cull = pix_create_compute_shader(GRASS_CULL);
    g.draw = pix_create_scene_shader(r, GRASS_VS, GRASS_FS, true);
    if (!g.cull || !g.draw) { pix_log("grass shaders failed"); pix_shutdown(); return 1; }
    pix_add_render_hook(r, grass_hook, &g);

    // ---- the player ----
    pix_model hero = pix_load_model("assets/models/player/player.glb");
    if (!hero.skinned) { pix_log("assets/models/player/player.glb missing or not rigged"); pix_shutdown(); return 1; }
    pix_anim_system& anims = pix_get_animations();
    idx actor = pix_anim_add(anims, hero.skeleton, hero.clips, hero.clip_count);
    idx clips[CLIP_COUNT];
    for (int i = 0; i < CLIP_COUNT; i++) {
        clips[i] = pix_anim_find_clip(anims, actor, CLIP_NAMES[i]);
        if (clips[i] == PIX_INVALID) clips[i] = 0;
    }
    animator probe = pix_model_animator(hero);
    float roll_len = animator_duration(probe, clips[CLIP_ROLL]);
    const char* LFOOT[] = { "LeftFoot", "foot.L", "mixamorig:LeftFoot" };
    const char* RFOOT[] = { "RightFoot", "foot.R", "mixamorig:RightFoot" };
    int foot_l = animator_find_bone(probe, LFOOT, 3), foot_r = animator_find_bone(probe, RFOOT, 3);
    gait walk = measure_gait(hero, clips[CLIP_WALK], foot_l, foot_r);
    gait run  = measure_gait(hero, clips[CLIP_RUN], foot_l, foot_r);
    if (walk.speed < 0.3f) walk.speed = walk.clip_speed = 1.3f;
    if (run.speed < walk.speed * 1.3f) run.speed = run.clip_speed = walk.speed * 2.4f;
    run.speed = HUMAN_RUN;                          // the clip plays faster to match
    float model_facing = walk.facing;
    for (int i = 0; i < walk.step_count; i++) pix_anim_add_event(anims, &hero.clips[clips[CLIP_WALK]], walk.steps[i], EVENT_STEP);
    for (int i = 0; i < run.step_count; i++)  pix_anim_add_event(anims, &hero.clips[clips[CLIP_RUN]], run.steps[i], EVENT_STEP);
    const char* HIPS[] = { "Hips", "hips", "mixamorig:Hips", "pelvis" };
    jump_timing jt = measure_jump(hero, clips[CLIP_JUMP], animator_find_bone(probe, HIPS, 4));
    pix_log("gait: walk %.2f m/s, run %.2f m/s, facing %+.0f; jump launch %.2f s, lands %.2f s",
            walk.speed, run.speed, model_facing, jt.launch, jt.touch);

    if (material* hm = pix_get_material(r, hero.material)) {
        hm->skinned_shader = hero_shader;
        hm->params1 = v4(0.45f, 0.0f, 0.0f, 0.0f);     // a strong rim picks the hero out of the grass
    }

    // ---- wolves: the packs in the valley, and the player's other form ----
    beast_rig beast = {};
    if (!load_beast(beast, WOLF_PATH)) { pix_log("%s missing or not rigged", WOLF_PATH); pix_shutdown(); return 1; }
    for (int i = 0; i < beast.walk.step_count; i++) pix_anim_add_event(anims, &beast.model.clips[beast.clip[WC_WALK]], beast.walk.steps[i], EVENT_STEP);
    for (int i = 0; i < beast.run.step_count; i++)  pix_anim_add_event(anims, &beast.model.clips[beast.clip[WC_RUN]], beast.run.steps[i], EVENT_STEP);
    beast.walk.speed *= WOLF_SCALE;          // a bigger body covers more ground per stride
    beast.walk.clip_speed *= WOLF_SCALE;
    beast.run.clip_speed *= WOLF_SCALE;
    beast.run.speed = WOLF_RUN;
    pix_log("wolf: walk %.2f m/s, run %.2f m/s (clip %.2f), facing %+.0f; claw L %.2f s, double %.2f/%.2f s, ram %.2f s; long leap %.2f..%.2f s",
            beast.walk.speed, beast.run.speed, beast.run.clip_speed, beast.facing, beast.strike[ATK_CLAW_L][0],
            beast.strike[ATK_CLAW_DOUBLE][0], beast.strike[ATK_CLAW_DOUBLE][beast.strikes[ATK_CLAW_DOUBLE] - 1],
            beast.strike[ATK_RAM][0], beast.jump[WC_RUN_LEAP].launch, beast.jump[WC_RUN_LEAP].touch);
    pix_log("wolf: death settles %.2f of %.2f s", beast.death_settle, beast.len[WC_DEATH]);
    if (material* wm = pix_get_material(r, beast.model.material)) {
        wm->skinned_shader = hero_shader;
        wm->params1 = v4(0.3f, 0.0f, 0.0f, 0.0f);
    }
    idx wolf_actor = pix_anim_add(anims, beast.model.skeleton, beast.model.clips, beast.model.clip_count);
    const float WOLF_HP = 70.0f, PLAYER_HP = 100.0f;

    // a pack sits somewhere open: inside the planted field, clear of trunks and rocks
    rng ai = rng_seed(7);
    auto open_spot = [&](vec3 around, float spread, float keep_from, vec3 avoid, vec3* out) {
        for (int t = 0; t < 96; t++) {
            float a = rng_range(ai, 0.0f, PIX_TAU), d = spread * sqrtf(rng_float(ai));
            float x = around.x + cosf(a) * d, z = around.z + sinf(a) * d;
            if (fabsf(x) > FIELD - 8.0f || fabsf(z) > FIELD - 8.0f) continue;
            float ax = x - avoid.x, az = z - avoid.z;
            if (ax * ax + az * az < keep_from * keep_from || rocky_amount(x, z) > 0.7f || pond_shore(x, z) < 1.5f) continue;
            bool blocked = false;
            for (int o = 0; o < obstacle_count && !blocked; o++) {
                float dx = x - obstacles[o].x, dz = z - obstacles[o].z, rr = obstacles[o].r + 1.0f;
                blocked = dx * dx + dz * dz < rr * rr;
            }
            if (blocked) continue;
            *out = v3(x, terrain_height(x, z), z);
            return true;
        }
        return false;
    };
    auto wolf_clip = [&](idx a, int& cur, int c, float fade, bool loop, float speed) {
        if (c != cur) { pix_anim_play(anims, a, beast.clip[c], fade, loop, speed); cur = c; }
        else pix_anim_set_speed(anims, a, speed);
    };
    auto wolf_restart = [&](idx a, int& cur, int c, float fade, float speed) {
        pix_anim_play(anims, a, beast.clip[c], fade, false, speed);
        pix_anim_set_time(anims, a, 0.0f);
        cur = c;
    };
    auto wolf_rest = [&](wolf& w) {           // idle a while, doing one of the calm things wolves do
        static const int CALM[] = { WC_IDLE, WC_IDLE, WC_LOOK, WC_REST, WC_EAT };
        w.state = AI_IDLE;
        w.timer = 0.0f;
        w.wait = rng_range(ai, 3.0f, 9.0f);
        wolf_clip(w.actor, w.clip, CALM[(int)(rng_float(ai) * 5) % 5], 0.4f, true, 1.0f);
    };

    const int MAX_WOLVES = 16;
    wolf* wolves = pix_alloc<wolf>(MAX_WOLVES);
    int wolf_count = 0;
    {
        // four packs around the spawn; the first close enough to meet early
        const float PACKS[][3] = { { 0.9f, 30.0f, 2 }, { 2.6f, 55.0f, 4 }, { 4.2f, 72.0f, 3 }, { 5.5f, 45.0f, 3 } };
        for (int p = 0; p < 4; p++) {
            vec3 centre = v3(cosf(PACKS[p][0]) * PACKS[p][1], 0.0f, sinf(PACKS[p][0]) * PACKS[p][1]);
            for (int k = 0; k < (int)PACKS[p][2] && wolf_count < MAX_WOLVES; k++) {
                vec3 at;
                if (!open_spot(centre, 8.0f, 20.0f, v3(0, 0, 0), &at)) continue;
                idx a = pix_anim_add(anims, beast.model.skeleton, beast.model.clips, beast.model.clip_count);
                if (a == PIX_INVALID) break;
                wolf& w = wolves[wolf_count++];
                w = wolf{};
                w.actor = a;
                w.pos = w.home = at;
                w.pack = p;
                w.hp = WOLF_HP;
                w.heading = rng_range(ai, 0.0f, PIX_TAU);
                w.orbit = rng_float(ai) < 0.5f ? -1.0f : 1.0f;
                w.clip = -1;
                wolf_rest(w);
                pix_anim_set_time(anims, a, rng_range(ai, 0.0f, 2.0f));     // not all in step
            }
        }
        pix_log("wolves: %d in 4 packs", wolf_count);
    }

    // ---- the wolfmen: a few alphas, each alone on its own ground ----
    brute_rig wm = {};
    const int MAX_BRUTES = 4;
    brute* brutes = pix_alloc<brute>(MAX_BRUTES);
    int brute_count = 0;
    const float BRUTE_HP = 280.0f;
    auto brute_restart = [&](brute& b, int c, float fade, float speed) {
        pix_anim_play(anims, b.actor, wm.clip[c], fade, false, speed);
        pix_anim_set_time(anims, b.actor, 0.0f);
        b.clip = c;
        b.root_prev = v2(0, 0);
        b.root_scale = 1.0f;
    };
    auto brute_loop = [&](brute& b, int c, float fade, float speed) {
        if (c != b.clip) { pix_anim_play(anims, b.actor, wm.clip[c], fade, true, speed); b.clip = c; b.root_prev = v2(0, 0); }
        else pix_anim_set_speed(anims, b.actor, speed);
    };
    auto brute_rest = [&](brute& b) {           // stands a while; now and then howls at nothing
        b.state = BR_IDLE;
        b.timer = 0.0f;
        b.wait = rng_range(ai, 4.0f, 10.0f);
        if (rng_float(ai) < 0.25f) brute_restart(b, MC_HOWL, 0.4f, 1.0f);
        else brute_loop(b, MC_IDLE, 0.4f, 1.0f);
    };
    if (load_brute(wm, WOLFMAN_PATH)) {
        for (int p = 0; p < wm.parts; p++)
            if (material* m = pix_get_material(r, wm.part_mat[p])) m->skinned_shader = hero_shader;
        for (int c : { MC_WALK, MC_WALK_BACK, MC_STRAFE_L, MC_STRAFE_R, MC_RUN, MC_RUN_BEAST }) {
            const gait& gg = c == MC_WALK ? wm.walk : c == MC_WALK_BACK ? wm.back : c == MC_RUN ? wm.run
                           : c == MC_RUN_BEAST ? wm.gallop : wm.strafe;
            if (c == MC_STRAFE_R) {       // measured on the left sidestep: its mirror plants at the same times
                for (int i = 0; i < gg.step_count; i++) pix_anim_add_event(anims, &wm.model.clips[wm.clip[c]], gg.steps[i] * wm.len[c] / maxf(wm.len[MC_STRAFE_L], 1e-3f), EVENT_STEP);
                continue;
            }
            for (int i = 0; i < gg.step_count; i++) pix_anim_add_event(anims, &wm.model.clips[wm.clip[c]], gg.steps[i], EVENT_STEP);
        }
        pix_log("wolfman: death settles %.2f of %.2f s", wm.death_settle, wm.len[MC_DEATH]);
    pix_log("wolfman: walk %.2f, back %.2f, strafe %.2f, run %.2f, gallop %.2f m/s; jump %.2f..%.2f s; claw hits %.2f s, pounce %.2f s (travels %.2f m)",
                wm.walk.speed, wm.back.speed, wm.strafe.speed, wm.run.speed, wm.gallop.speed, wm.jump.launch, wm.jump.touch,
                wm.strike[BA_CLAW_R][0], wm.strike[BA_POUNCE][0], wm.root[MC_POUNCE][ROOT_SAMPLES - 1].y);
        // between the packs, further out the further round
        const float LAIRS[][2] = { { 1.75f, 42.0f }, { 3.4f, 68.0f }, { 0.1f, 92.0f } };
        for (int k = 0; k < 3 && brute_count < MAX_BRUTES; k++) {
            vec3 centre = v3(cosf(LAIRS[k][0]) * LAIRS[k][1], 0.0f, sinf(LAIRS[k][0]) * LAIRS[k][1]), at;
            if (!open_spot(centre, 10.0f, 25.0f, v3(0, 0, 0), &at)) continue;
            idx a = pix_anim_add(anims, wm.model.skeleton, wm.model.clips, wm.model.clip_count);
            if (a == PIX_INVALID) break;
            brute& b = brutes[brute_count++];
            b = brute{};
            b.actor = a;
            b.pos = b.home = at;
            b.hp = BRUTE_HP;
            b.heading = rng_range(ai, 0.0f, PIX_TAU);
            b.orbit = rng_float(ai) < 0.5f ? -1.0f : 1.0f;
            b.clip = -1;
            b.root_scale = 1.0f;
            brute_rest(b);
        }
        pix_log("wolfmen: %d", brute_count);
    } else {
        pix_log("%s missing or not rigged: no wolfmen", WOLFMAN_PATH);
    }

    // ---- the villager: the runner model on the hero's rig ----
    pix_model runner = pix_load_model(RUNNER_PATH);
    npc vil = {};
    vil.actor = PIX_INVALID;
    idx vclips[CLIP_COUNT];
    gait vwalk = {}, vrun = {};
    if (runner.skinned) {
        vil.actor = pix_anim_add(anims, runner.skeleton, runner.clips, runner.clip_count);
        animator va = pix_model_animator(runner);
        for (int i = 0; i < CLIP_COUNT; i++) vclips[i] = find_clip(va, CLIP_NAMES[i], 0);
        int vl = animator_find_bone(va, LFOOT, 3), vr = animator_find_bone(va, RFOOT, 3);
        vwalk = measure_gait(runner, vclips[CLIP_WALK], vl, vr);
        vrun  = measure_gait(runner, vclips[CLIP_RUN], vl, vr);
        if (vwalk.speed < 0.3f) vwalk.speed = vwalk.clip_speed = 1.3f;
        if (vrun.speed < vwalk.speed * 1.3f) vrun.speed = vrun.clip_speed = vwalk.speed * 2.4f;
        for (int i = 0; i < vwalk.step_count; i++) pix_anim_add_event(anims, &runner.clips[vclips[CLIP_WALK]], vwalk.steps[i], EVENT_STEP);
        for (int i = 0; i < vrun.step_count; i++)  pix_anim_add_event(anims, &runner.clips[vclips[CLIP_RUN]], vrun.steps[i], EVENT_STEP);
        if (material* vm = pix_get_material(r, runner.material)) {
            vm->skinned_shader = hero_shader;
            vm->params1 = v4(0.35f, 0.0f, 0.0f, 0.0f);
        }
        vil.pos = vil.goal = v3(4.0f, terrain_height(4.0f, 3.0f), 3.0f);
        vil.heading = -2.2f;
        vil.clip = CLIP_STAND;
        vil.wait = 2.0f;
        play(anims, vil.actor, vclips, CLIP_STAND, 0.0f, true, 1.0f);
    } else {
        pix_log("%s missing or not rigged: no villager", RUNNER_PATH);
    }

    // ---- puffs ----
    particle* parts = pix_alloc<particle>(MAX_PUFFS);
    int part_count = 0;
    idx puff_mesh = pix_gen_sphere(1.0f, 6, 8);
    material pm = pix_material_default();
    pm.shader = model_shader;
    pm.roughness = 1.0f;
    idx puff_mat = pix_add_material(r, pm);
    // hot light: unlit, added on top, bright enough for the bloom to catch
    material hm = pix_material_default();
    hm.unlit = true;
    hm.blend = PIX_BLEND_ADDITIVE;
    hm.cast_shadows = false;
    hm.double_sided = true;
    hm.color = v3(2.5f, 2.5f, 2.5f);
    idx glow_mat = pix_add_material(r, hm);
    idx bar_mesh = pix_gen_cube(v3(1.0f, 1.0f, 1.0f));
    streak* streaks = pix_alloc<streak>(MAX_STREAKS);
    int streak_count = 0;
    popup* popups = pix_alloc<popup>(MAX_POPUPS);
    int popup_count = 0;
    float hitstop = 0.0f, shake = 0.0f;    // a frozen instant on impact, and a jolt of the camera

    // one toon switch for everything drawn with the shared look
    const idx look_programs[] = { ground_shader, tree_shader, model_shader, hero_shader, pond_shader };
    const int LOOK_PROGRAMS = sizeof(look_programs) / sizeof(look_programs[0]);
    set_look_toon(look_programs, LOOK_PROGRAMS, 1.0f);

    idx step = pix_load_sound("assets/audio/step.wav");
    idx chime = pix_load_sound("assets/audio/chime.wav");

    const vec3 SPAWN = v3(0, terrain_height(0, 0), 0);
    player pl = {};
    pl.pos = SPAWN;
    pl.heading = 0.6f;
    pl.clip = CLIP_STAND;
    pl.grounded = true;
    pl.hp = PLAYER_HP;
    pl.calm = 10.0f;
    pl.jump_clip = WC_JUMP_FWD;
    play(anims, actor, clips, CLIP_STAND, 0.0f, true, 1.0f);
    wolf_restart(wolf_actor, pl.clip, WC_IDLE, 0.0f, 1.0f);      // the other form, ready
    pl.clip = CLIP_STAND;

    float wind = 1.0f, yaw = PIX_PI * 0.5f + 0.6f, pitch = 0.3f, dist = 6.5f, rumble = 0.0f;
    bool toon_on = true, mouse_captured = false, flying = false, show_boxes = false;
    vec3 fly_pos = v3(0, 0, 0);
    vec3 focus = pl.pos + v3(0, 1.1f, 0);
    rng fx = rng_seed(99);
    const float GRAVITY = 22.0f, JUMP_V = 6.2f;
    int kills = 0, brute_kills = 0;
    bool player_swung = false;    // the player began an attack this frame: a wolfman nearby may sidestep it
    float howl_cd = 0.0f;
    float peace = 0.0f;           // after rising again the packs leave you be for a moment

    auto forward = [](float heading) { return v3(sinf(heading), 0.0f, cosf(heading)); };
    auto flat_to = [](vec3 from, vec3 to, float* len) {
        vec3 d = v3(to.x - from.x, 0.0f, to.z - from.z);
        *len = sqrtf(d.x * d.x + d.z * d.z);
        return *len > 1e-4f ? d * (1.0f / *len) : v3(0, 0, 0);
    };
    auto collide = [&](vec3& pos, vec3& vel, float radius) {
        for (int o = 0; o < obstacle_count; o++) {
            float dx = pos.x - obstacles[o].x, dz = pos.z - obstacles[o].z;
            float need = obstacles[o].r + radius, d2 = dx * dx + dz * dz;
            if (obstacles[o].r <= 0.0f || d2 >= need * need || d2 < 1e-8f) continue;   // r 0: a felled tree
            float d = sqrtf(d2), nx = dx / d, nz = dz / d;
            pos.x = obstacles[o].x + nx * need;
            pos.z = obstacles[o].z + nz * need;
            float into = vel.x * nx + vel.z * nz;              // slide along it
            if (into < 0.0f) { vel.x -= nx * into; vel.z -= nz * into; }
        }
        // the pieces of broken trees are in the way too (their wood: a crown's
        // twigs are walked through), unless jumped over
        for (int i = 0; i < tree_count; i++)
            for (int k = 0; k < trees[i].pieces; k++) {
                const tree_piece& tp = trees[i].piece[k];
                if (tp.state == PIECE_GONE || !tp.wood) continue;
                vec3 ax[3];
                piece_axes(tp, ax);
                vec3 c = piece_place(tp, tp.wood0);
                float bound = v3len(tp.wood_half) + radius;
                if (fabsf(pos.x - c.x) > bound || fabsf(pos.z - c.z) > bound) continue;
                if (pos.y > obb_top(c, ax, tp.wood_half) - 0.05f) continue;
                vec3 q = pos + v3(0, 0.4f, 0);
                vec3 near_pt = obb_closest(c, ax, tp.wood_half, q);
                float dx = q.x - near_pt.x, dz = q.z - near_pt.z, d2 = dx * dx + dz * dz;
                if (d2 >= radius * radius) continue;
                float d = sqrtf(d2);
                if (d < 1e-4f) { dx = q.x - c.x; dz = q.z - c.z; d = maxf(sqrtf(dx * dx + dz * dz), 1e-4f); }
                float nx = dx / d, nz = dz / d, out = radius - minf(sqrtf(d2), radius);
                pos.x += nx * out;
                pos.z += nz * out;
                float into = vel.x * nx + vel.z * nz;
                if (into < 0.0f) { vel.x -= nx * into; vel.z -= nz * into; }
            }
        // into a pond only as far as the shallows: about knee deep
        for (int i = 0; i < g_pond_count; i++) {
            const pond& pd = g_ponds[i];
            float dx = pos.x - pd.x, dz = pos.z - pd.z;
            if (dx * dx + dz * dz > (pd.bound + 1.0f) * (pd.bound + 1.0f)) continue;
            const float lim = -0.5f;                      // the U deepens fast past its rim
            float sd = pond_distance(pd, pos.x, pos.z);
            if (sd >= lim) continue;
            // out along the shape's own slope: toward the nearest shore
            float e = 0.3f;
            float gx = pond_distance(pd, pos.x + e, pos.z) - pond_distance(pd, pos.x - e, pos.z);
            float gz = pond_distance(pd, pos.x, pos.z + e) - pond_distance(pd, pos.x, pos.z - e);
            float gl = sqrtf(gx * gx + gz * gz);
            if (gl < 1e-6f) continue;
            float nx = gx / gl, nz = gz / gl;
            pos.x += nx * (lim - sd);
            pos.z += nz * (lim - sd);
            float into = vel.x * nx + vel.z * nz;
            if (into < 0.0f) { vel.x -= nx * into; vel.z -= nz * into; }
        }
        pos.x = clampf(pos.x, -TERRAIN + 4.0f, TERRAIN - 4.0f);
        pos.z = clampf(pos.z, -TERRAIN + 4.0f, TERRAIN - 4.0f);
    };
    // At full speed the gallop plays only a little faster than as authored:
    // long bounding strides covering the ground, not the clip on fast-forward.
    auto wolf_run_rate = [&](float speed) { return clampf(speed / beast.run.speed, 0.6f, CHARGE_SPEED) * WOLF_RUN_RATE; };
    auto player_radius = [&]() { return pl.wolf ? 0.6f * WOLF_SCALE : 0.35f; };
    auto form_actor = [&]() { return pl.wolf ? wolf_actor : actor; };
    auto form_play = [&](int c, float fade, bool loop, float speed) {
        pix_anim_play(anims, form_actor(), pl.wolf ? beast.clip[c] : clips[c], fade, loop, speed);
    };
    // the puff of the change of form - and of a creature's end
    auto smoke = [&](vec3 at, float amount) {
        burst(parts, part_count, fx, at, (int)(46 * amount), 5.0f, 0.13f, pix_rgba(210, 205, 195), 0.3f);
        burst(parts, part_count, fx, at, (int)(18 * amount), 7.0f, 0.05f, pix_rgba(255, 210, 120), 0.8f);
    };
    auto alert_pack = [&](const wolf& from) {
        for (int i = 0; i < wolf_count; i++) {
            wolf& o = wolves[i];
            if (o.pack != from.pack || o.state == AI_DEAD || v3dist(o.pos, from.pos) > 30.0f) continue;
            if (o.state == AI_IDLE || o.state == AI_WANDER) {
                o.state = AI_ALERT;
                o.timer = -rng_range(ai, 0.0f, 0.5f);           // they look up one after another
                wolf_restart(o.actor, o.clip, WC_AGGRO, 0.25f, 1.0f);
            }
        }
    };
    // What a landed blow looks like, by the kind of attack: claw marks raked
    // across the target, a snap of jaws, the shockwave of a tackle, a ring
    // over the ground for a slam - with the damage floating up, a frozen
    // instant and a jolt of the camera. `fwd` points from attacker to target.
    auto hit_fx = [&](vec3 at, float height, vec3 fwd, int atk, int strike, float dmg, bool on_player, bool killing) {
        const attack_def& A = ATTACKS[atk];
        vec3 up = v3(0, 1, 0), side = v3(fwd.z, 0.0f, -fwd.x);
        vec3 chest = at + up * height - fwd * 0.35f;           // on the side facing the attacker
        pix_color hot = on_player ? pix_rgba(255, 60, 40) : pix_rgba(255, 236, 190);
        bool heavy = A.lunge < 0.0f || A.lunge > 3.0f || atk == ATK_SLAM;
        if (atk == ATK_SLAM) {
            ring(parts, part_count, fx, at + v3(0, 0.15f, 0), 28, 9.0f, 0.06f, hot);
            burst(parts, part_count, fx, at + v3(0, 0.2f, 0), 18, 5.0f, 0.1f, pix_rgba(140, 120, 90), 0.2f);
        } else if (!A.jaws) {
            // three claw marks raked down across the target; a left paw one way, a right the other
            float sgn = (atk == ATK_CLAW_R || (atk == ATK_CLAW_DOUBLE && strike == 1)) ? -1.0f : 1.0f;
            vec3 axis = v3norm(side * sgn - up * 0.8f);
            vec3 across = v3norm(side * 0.8f + up * sgn);
            for (int k = -1; k <= 1; k++)
                add_streak(streaks, streak_count, chest - axis * 0.45f + across * (0.15f * k) + fwd * (0.03f * k),
                           axis, across, 0.95f - 0.12f * (k * k), 0.05f, 0.28f, hot);
            sparks(parts, part_count, fx, chest, fwd + axis * 0.5f, 8, 5.0f, 0.03f, hot);
        } else if (A.lunge <= 3.0f && A.lunge >= 0.0f) {
            // a bite: two bars of light snapping shut, and a spray of sparks
            add_streak(streaks, streak_count, chest + up * 0.16f - side * 0.3f, side, up, 0.6f, 0.04f, 0.2f, hot);
            add_streak(streaks, streak_count, chest - up * 0.16f + side * 0.3f, side * -1.0f, up, 0.6f, 0.04f, 0.2f, hot);
            sparks(parts, part_count, fx, chest, fwd, 12, 6.0f, 0.035f, hot);
        } else {
            // a tackle, pounce or ram: a star of light and a shockwave around the target
            for (int k = 0; k < 6; k++) {
                float a = PIX_TAU * k / 6.0f + 0.3f;
                vec3 dir = v3norm(side * cosf(a) + up * sinf(a));
                add_streak(streaks, streak_count, chest + dir * 0.15f, dir, v3norm(v3cross(dir, fwd)), 0.7f, 0.05f, 0.22f, hot);
            }
            ring(parts, part_count, fx, at + up * (height * 0.6f), 20, 7.0f, 0.05f, hot);
            burst(parts, part_count, fx, at + v3(0, 0.2f, 0), 10, 3.0f, 0.1f, pix_rgba(140, 120, 90), 0.3f);
        }
        if (popup_count < MAX_POPUPS) {
            popup& p = popups[popup_count++];
            p.pos = at + up * (height + 0.8f);
            p.life = 0.9f;
            p.amount = (int)(dmg + 0.5f);
            p.size = heavy ? 30.0f : 22.0f;
            p.color = on_player ? pix_rgba(255, 90, 70) : killing ? pix_rgba(255, 200, 60) : heavy ? pix_rgba(255, 170, 90) : PIX_WHITE;
        }
        // a heavier blow freezes the moment longer; the finishing blow longest
        hitstop = maxf(hitstop, killing ? 0.16f : heavy ? 0.09f : 0.05f);
        shake = maxf(shake, killing ? 0.8f : heavy ? 0.6f : on_player ? 0.5f : 0.3f);
    };
    auto hurt_wolf = [&](wolf& w, float dmg, vec3 dir, float knock) {
        w.hp -= dmg;
        w.flash = 1.0f;
        w.anger = 20.0f;
        w.vel = dir * knock;
        burst(parts, part_count, fx, w.pos + v3(0, 0.8f, 0), 14, 4.0f, 0.07f, pix_rgba(120, 112, 104), 0.6f);
        pix_play_sound(step, 0.9f, rng_range(fx, 0.45f, 0.55f));
        alert_pack(w);
        if (w.hp <= 0.0f) {
            w.hp = 0.0f;
            w.state = AI_DEAD;
            w.timer = 0.0f;
            w.respawn = 25.0f;
            w.gone = false;
            wolf_restart(w.actor, w.clip, WC_DEATH, 0.1f, 1.0f);
            kills++;
            return;
        }
        // struck from its left or its right
        vec3 f = forward(w.heading);
        bool left = f.z * dir.x - f.x * dir.z > 0.0f;
        w.state = AI_HIT;
        w.timer = 0.0f;
        w.cooldown = maxf(w.cooldown, 0.6f);
        wolf_restart(w.actor, w.clip, left ? WC_HIT_L : WC_HIT_R, 0.06f, 1.2f);
    };
    // A wolfman shrugs off a light blow while swinging; one heavy enough, or
    // a few in a row, breaks its poise and it staggers.
    auto hurt_brute = [&](brute& b, float dmg, vec3 dir, float knock) {
        b.hp -= dmg;
        b.flash = 1.0f;
        b.anger = 30.0f;
        b.poise += dmg;
        burst(parts, part_count, fx, b.pos + v3(0, 1.3f * WM_SCALE, 0), 16, 4.0f, 0.08f, pix_rgba(90, 80, 76), 0.6f);
        pix_play_sound(step, 1.0f, rng_range(fx, 0.38f, 0.45f));
        if (b.hp <= 0.0f) {
            b.hp = 0.0f;
            b.state = BR_DEAD;
            b.timer = 0.0f;
            b.respawn = 45.0f;
            b.gone = false;
            b.vel = v3(0, 0, 0);
            brute_restart(b, MC_DEATH, 0.1f, 1.0f);
            brute_kills++;
            return;
        }
        if (b.poise > 55.0f || dmg >= 30.0f) {
            b.poise = 0.0f;
            b.state = BR_STAGGER;
            b.timer = 0.0f;
            b.vel = dir * (knock * 0.6f);
            b.cooldown = maxf(b.cooldown, 0.8f);
            brute_restart(b, MC_HIT_HEAVY, 0.08f, 1.1f);
        } else if (b.state != BR_ATTACK && b.state != BR_LEAP && b.state != BR_STAGGER) {
            b.state = BR_HIT;
            b.timer = 0.0f;
            b.vel = dir * (knock * 0.4f);
            brute_restart(b, MC_HIT, 0.06f, 1.2f);
        } else if (b.state == BR_IDLE || b.state == BR_WANDER || b.state == BR_ROAR) {
            b.state = BR_CHASE;
            b.timer = 0.0f;
        }
    };
    auto hurt_player = [&](float dmg, vec3 from, float knock, int atk, int strike) {
        if (pl.move == MOVE_DEAD || pl.evading || pl.invuln > 0.0f) return;
        float l;
        vec3 dir = flat_to(from, pl.pos, &l);
        hit_fx(pl.pos, pl.wolf ? 0.7f * WOLF_SCALE : 1.0f, dir, atk, strike, dmg, true, pl.hp <= dmg);
        pl.hp -= dmg;
        pl.flash = 1.0f;
        pl.invuln = 0.45f;
        pl.calm = 0.0f;
        pl.vel.x = dir.x * knock;
        pl.vel.z = dir.z * knock;
        rumble = maxf(rumble, 0.55f);
        pix_play_sound(step, 1.0f, rng_range(fx, 0.5f, 0.6f));
        burst(parts, part_count, fx, pl.pos + v3(0, pl.wolf ? 0.8f : 1.1f, 0), 10, 3.5f, 0.06f, pix_rgba(150, 40, 36), 0.5f);
        idx pa = form_actor();
        if (pl.hp <= 0.0f) {
            pl.hp = 0.0f;
            pl.move = MOVE_DEAD;
            pl.timer = 0.0f;
            form_play(pl.wolf ? WC_DEATH : CLIP_CROUCH, 0.15f, false, 1.0f);
            pix_anim_set_time(anims, pa, 0.0f);
            return;
        }
        if (!pl.grounded || pl.move == MOVE_ATTACK) return;     // no flinch mid-air or mid-swing
        pl.move = MOVE_HURT;
        pl.timer = 0.0f;
        if (pl.wolf) {
            vec3 f = forward(pl.heading);
            form_play(f.z * -dir.x - f.x * -dir.z > 0.0f ? WC_HIT_L : WC_HIT_R, 0.06f, false, 1.3f);
            pix_anim_set_time(anims, pa, 0.0f);
            pl.clip = -1;
        }
    };
    // A blow on a tree: bark flies and leaves come down, and once it has
    // taken enough it snaps above the stump and topples away from the blow.
    // A blow on a fallen log splits it; enough of them and it cracks into
    // sections that jump apart, and a section (or a crown) bursts.
    const pix_color BARK = pix_rgba(118, 86, 58), LEAF = pix_rgba(96, 150, 62), PALE_WOOD = pix_rgba(214, 186, 140);
    const pix_color DUST = pix_rgba(150, 132, 100);
    auto burst_piece = [&](tree_body& t, tree_piece& p) {
        p.state = PIECE_GONE;
        p.gone_for = 0.0f;
        vec3 a, b;
        piece_segment(t, p, &a, &b);
        int n = 2 + (int)(p.len / 1.5f);
        for (int k = 0; k <= n; k++) {
            vec3 at = a + (b - a) * ((float)k / n);
            burst(parts, part_count, fx, at, 12, 6.5f, 0.09f, k & 1 ? BARK : PALE_WOOD, 0.6f);
            burst(parts, part_count, fx, at, 4, 3.0f, 0.2f, pix_rgba(200, 190, 170), 0.3f);
        }
        if (p.leafy) {
            vec3 ax[3];
            piece_axes(p, ax);
            for (int i = 0; i < 6; i++) {
                vec3 at = p.com + ax[0] * (p.half.x * rng_range(fx, -0.8f, 0.8f)) + ax[1] * (p.half.y * rng_range(fx, -0.8f, 0.8f))
                        + ax[2] * (p.half.z * rng_range(fx, -0.8f, 0.8f));
                burst(parts, part_count, fx, at, 12, 4.0f, 0.07f, LEAF, 0.5f);
            }
        }
        smoke(p.com, 0.7f);
        hitstop = maxf(hitstop, 0.1f);
        shake = maxf(shake, 0.6f);
        rumble = maxf(rumble, 0.5f);
        pix_play_sound(step, 1.0f, 0.3f);
    };
    // a log cracks at every slab bound it spans, and the sections jump apart
    auto crack_piece = [&](tree_body& t, int i) {
        int first_new = t.pieces;
        vec3 axis = quat_rotate(t.piece[i].rot, v3(0, 1, 0));
        for (int at = t.piece[i].hi - 1; at > t.piece[i].lo; at--) {
            vec3 where = piece_place(t.piece[i], tree_at(t, t.cut[at]));
            if (!piece_split(t, i, at)) break;
            burst(parts, part_count, fx, where, 26, 6.0f, 0.08f, PALE_WOOD, 0.6f);
            burst(parts, part_count, fx, where, 8, 3.0f, 0.18f, DUST, 0.3f);
        }
        vec3 side = v3norm(v3cross(axis, v3(0, 1, 0)));
        for (int k = first_new - 1; k < t.pieces; k++) {
            tree_piece& p = t.piece[k == first_new - 1 ? i : k];
            float along = k == first_new - 1 ? -1.0f : 1.0f;
            p.vel += v3(0, rng_range(fx, 2.0f, 3.2f), 0) + side * rng_range(fx, -1.5f, 1.5f) + axis * along;
            p.avel += v3(rng_range(fx, -1.5f, 1.5f), rng_range(fx, -1.0f, 1.0f), rng_range(fx, -1.5f, 1.5f));
            p.hp = 12.0f + 2.0f * p.len;
        }
        hitstop = maxf(hitstop, 0.1f);
        shake = maxf(shake, 0.6f);
        rumble = maxf(rumble, 0.5f);
        pix_play_sound(step, 1.0f, 0.22f);
    };
    // piece -1: the standing tree
    auto hurt_tree = [&](tree_body& t, int piece, float dmg, vec3 dir, vec3 at) {
        t.shown = 4.0f;
        t.shown_piece = piece;
        sparks(parts, part_count, fx, at, dir * -1.0f + v3(0, 0.4f, 0), 10, 5.0f, 0.05f, PALE_WOOD);
        burst(parts, part_count, fx, at, 8, 3.0f, 0.06f, BARK, 0.5f);
        pix_play_sound(step, 1.0f, rng_range(fx, 0.35f, 0.42f));
        if (piece < 0) {
            t.shiver = 1.0f;
            t.shiver_dir = dir;
            t.hp -= dmg;
            // leaves shaken loose drift down out of the crown
            burst(parts, part_count, fx, tree_at(t, t.height * 0.65f), 14, 1.6f, 0.07f, LEAF, 0.1f);
            if (t.hp > 0.0f) return;
            t.hp = 0.0f;
            tree_topple(t, dir);
            vec3 snap = tree_at(t, t.cut[CUT_STUMP]);
            tops[t.obstacle] = snap.y;               // only the stump stands there now
            burst(parts, part_count, fx, snap, 30, 6.0f, 0.08f, PALE_WOOD, 0.5f);
            burst(parts, part_count, fx, snap - v3(0, 0.3f, 0), 16, 3.0f, 0.18f, DUST, 0.2f);
            pix_play_sound(step, 1.0f, 0.25f);
            shake = maxf(shake, 0.5f);
            return;
        }
        tree_piece& p = t.piece[piece];
        p.shiver = 1.0f;
        p.shiver_dir = dir;
        p.hp -= dmg;
        if (p.hp > 0.0f) return;
        if (p.hi - p.lo > 1) crack_piece(t, piece);
        else burst_piece(t, p);
    };
    // every tree and log in reach in front of `from`; returns how many were hit
    auto strike_trees = [&](vec3 from, vec3 f, float reach, float cone, float dmg, int atk, int strike) {
        int hits = 0;
        for (int i = 0; i < tree_count; i++) {
            tree_body& t = trees[i];
            if (t.state == TREE_TIPPING) continue;
            // the standing trunk, or the nearest piece lying there
            int piece = -1;
            vec3 at = v3(t.root.x, from.y + 0.9f, t.root.z);
            float radius = t.trunk;
            if (t.state == TREE_BROKEN) {
                float best = 1e9f;
                for (int k = 0; k < t.pieces; k++) {
                    const tree_piece& tp = t.piece[k];
                    if (tp.state == PIECE_GONE) continue;
                    vec3 ax[3];
                    piece_axes(tp, ax);
                    vec3 c = obb_closest(tp.com, ax, tp.half, from + v3(0, 0.6f, 0));
                    float d = v3dist(c, from);
                    if (d < best) { best = d; piece = k; at = c; radius = 0.0f; }
                }
                if (piece < 0) continue;
            }
            float l;
            vec3 d = flat_to(from, at, &l);
            if (l - radius > reach || fabsf(at.y - from.y) > 2.5f || (l > 0.5f && v3dot(d, f) < cone)) continue;
            if (l < 1e-3f) d = f;
            vec3 surface = at - d * minf(radius, maxf(l - 0.2f, 0.0f));
            hit_fx(surface - v3(0, 1.0f, 0), 1.0f, d, atk, strike, dmg, false, false);
            hurt_tree(t, piece, dmg, d, surface);
            hits++;
        }
        return hits;
    };

    // the wolf form's attacks land on every wolf in reach in front of it
    auto player_strike = [&](int atk, int strike) {
        const attack_def& A = ATTACKS[atk];
        float reach = A.reach * WOLF_SCALE + 0.4f, dmg = A.damage * 2.6f;
        vec3 f = forward(pl.heading);
        int hits = 0;
        for (int i = 0; i < wolf_count; i++) {
            wolf& w = wolves[i];
            if (w.state == AI_DEAD) continue;
            float l;
            vec3 d = flat_to(pl.pos, w.pos, &l);
            if (l > reach || (l > 0.5f && v3dot(d, f) < A.cone)) continue;
            hit_fx(w.pos, 0.7f * WOLF_SCALE, l > 1e-3f ? d : f, atk, strike, dmg, false, w.hp <= dmg);
            hurt_wolf(w, dmg, l > 1e-3f ? d : f, A.knock);
            hits++;
        }
        for (int i = 0; i < brute_count; i++) {
            brute& b = brutes[i];
            if (b.state == BR_DEAD) continue;
            // mid-sidestep or leaping back it is out of reach
            if (b.state == BR_DODGE && pix_anim_time(anims, b.actor) < wm.len[b.clip] * 0.6f) continue;
            float l;
            vec3 d = flat_to(pl.pos, b.pos, &l);
            if (l > reach + 0.3f || (l > 0.5f && v3dot(d, f) < A.cone)) continue;
            hit_fx(b.pos, 1.2f * WM_SCALE, l > 1e-3f ? d : f, atk, strike, dmg, false, b.hp <= dmg);
            hurt_brute(b, dmg, l > 1e-3f ? d : f, A.knock);
            hits++;
        }
        hits += strike_trees(pl.pos, f, reach, A.cone, dmg, atk, strike);
        if (hits) rumble = maxf(rumble, 0.4f);
    };

    auto start_attack = [&](int atk, vec3 aim) {
        const attack_def& A = ATTACKS[atk];
        pl.move = MOVE_ATTACK;
        pl.attack = atk;
        pl.charged = false;
        pl.charging = false;
        pl.struck = 0;
        pl.timer = 0.0f;
        pl.clip = -1;
        // lock on: turn toward the nearest wolf in reach (further for a lunge), else where the stick points
        int best = -1;
        float best_d = A.reach + (A.lunge != 0.0f && A.lunge < 3.0f ? 2.4f : 5.0f);
        for (int i = 0; i < wolf_count; i++) {
            if (wolves[i].state == AI_DEAD) continue;
            float l = v3dist(wolves[i].pos, pl.pos);
            if (l < best_d) { best_d = l; best = i; }
        }
        vec3 target = best >= 0 ? wolves[best].pos : v3(0, 0, 0);
        for (int i = 0; i < brute_count; i++) {
            if (brutes[i].state == BR_DEAD) continue;
            float l = v3dist(brutes[i].pos, pl.pos) - 0.3f;
            if (l < best_d) { best_d = l; best = i; target = brutes[i].pos; }
        }
        player_swung = true;
        if (best >= 0 && A.cone > -1.0f) pl.heading = atan2f(target.x - pl.pos.x, target.z - pl.pos.z);
        else if (v3len(aim) > 0.0f) pl.heading = atan2f(aim.x, aim.z);
        float rate = A.lunge < 0.0f ? 1.0f : (A.hits > 1 || A.lunge > 3.0f || atk == ATK_SLAM ? 1.15f : 1.35f);
        pl.attack_rate = rate;
        form_play(A.clip, 0.06f, false, rate);
        pix_anim_set_time(anims, form_actor(), 0.0f);
    };

    // Where a frame's milliseconds go, section by section, printed once a
    // second when PIX_PROF is set in the environment (with each GPU pass).
    enum { PR_PRESENT, PR_PLAYER, PR_WOLVES, PR_BRUTES, PR_MISC, PR_TREES, PR_PUFFS, PR_ANIM, PR_GRASS,
           PR_SUBMIT, PR_RENDER, PR_HUD, PR_COUNT };
    static const char* PR_NAMES[PR_COUNT] = { "present", "player", "wolves", "wolfmen", "misc", "trees", "puffs",
                                               "anim", "grass+cam", "submit", "render", "hud" };
    pix_profiler prof;
    pix_profiler_init(prof, PR_NAMES, PR_COUNT);
    if (prof.enabled) pix_enable_gpu_timing(r, true);
    long long prof_t = pix_profile_now();
    auto lap = [&](int section) { pix_profile_add(prof, section, prof_t); prof_t = pix_profile_now(); };

    while (pix_frame()) {
        lap(PR_PRESENT);
        // on impact the world holds still for an instant (effects and camera keep going)
        float real_dt = pix_dt();
        float dt = hitstop > 0.0f ? real_dt * 0.05f : real_dt;
        hitstop = maxf(0.0f, hitstop - real_dt);
        shake = maxf(0.0f, shake - real_dt * 3.0f);
        const pix_gamepad& pad = pix_get_gamepad(0);
        if (pix_key_pressed(KEY_ESC)) {
            if (mouse_captured) { mouse_captured = false; pix_set_mouse_locked(false); }
            else pix_quit();
        }
        bool just_captured = false;
        if (!mouse_captured && pix_mouse_pressed(PIX_MOUSE_LEFT)) { mouse_captured = true; pix_set_mouse_locked(true); just_captured = true; }
        if (pix_key_down('T')) { hour += dt; if (hour >= 24.0f) hour -= 24.0f; apply_sky(r, hour); }
        if (pix_key_pressed('B')) show_boxes = !show_boxes;
        if (pix_key_pressed('G')) {
            toon_on = !toon_on;
            g.toon = toon_on ? 1.0f : 0.0f;
            set_look_toon(look_programs, LOOK_PROGRAMS, g.toon);
        }
        if (pix_key_pressed('1') || pix_key_pressed('2')) {
            wind = pix_key_pressed('1') ? maxf(0.0f, wind - 0.5f) : minf(3.0f, wind + 0.5f);
            for (int i = 0; i < TREE_KINDS; i++)
                for (int p = 0; p < kinds[i].parts; p++)
                {
                    if (material* m = pix_get_material(r, kinds[i].material[p])) m->params0.x = wind;
                    for (int k = 0; k < SLABS; k++)
                        if (material* m = pix_get_material(r, kinds[i].slab[p][k])) m->params0.x = wind;
                }
        }

        // ---- camera input: captured mouse, right drag or the right stick ----
        if (pix_key_pressed('F') || pad.buttons[PAD_BACK].pressed) {
            flying = !flying;
            if (flying) fly_pos = focus + v3(cosf(yaw) * cosf(pitch), sinf(pitch), sinf(yaw) * cosf(pitch)) * dist;
        }
        float look = mouse_captured ? 0.0035f : (pix_mouse_down(PIX_MOUSE_RIGHT) ? 0.006f : 0.0f);
        yaw += pix_mouse_delta().x * look;
        pitch += pix_mouse_delta().y * look;
        yaw += pad.right_x * dt * 2.6f;
        pitch -= pad.right_y * dt * 1.8f;
        pitch = flying ? clampf(pitch, -1.45f, 1.45f) : clampf(pitch, -0.2f, 1.3f);
        dist = clampf(dist - pix_mouse_wheel() * 0.8f, 2.5f, 25.0f);
        if (pad.buttons[PAD_LEFT_BUMPER].held)  dist = minf(25.0f, dist + dt * 8.0f);
        if (pad.buttons[PAD_RIGHT_BUMPER].held) dist = maxf(2.5f, dist - dt * 8.0f);

        // ---- movement input, relative to the camera ----
        const gait& gw = pl.wolf ? beast.walk : walk;
        const gait& gr = pl.wolf ? beast.run : run;
        auto J = [&]() -> const jump_timing& { return pl.wolf ? beast.jump[pl.jump_clip] : jt; };
        vec2 key = v2(0, 0);
        if (pix_key_down('W') || pix_key_down(KEY_UP))    key.y += 1;
        if (pix_key_down('S') || pix_key_down(KEY_DOWN))  key.y -= 1;
        if (pix_key_down('D') || pix_key_down(KEY_RIGHT)) key.x += 1;
        if (pix_key_down('A') || pix_key_down(KEY_LEFT))  key.x -= 1;
        vec2 stick = v2(pad.left_x, pad.left_y);
        if (flying) {
            // the free camera takes the movement keys; the hero stands still
            vec3 fwd = v3(-cosf(yaw) * cosf(pitch), -sinf(pitch), -sinf(yaw) * cosf(pitch));
            vec3 side = v3norm(v3(-fwd.z, 0, fwd.x));
            vec2 mv = v2(key.x + stick.x, key.y + stick.y);
            float up = (pix_key_down(KEY_SPACE) ? 1.0f : 0.0f) - (pix_key_down('C') || pix_key_down(KEY_CTRL) ? 1.0f : 0.0f)
                     + pad.right_trigger - pad.left_trigger;
            float speed = pix_key_down(KEY_SHIFT) ? 40.0f : 12.0f;
            fly_pos += (fwd * mv.y + side * mv.x + v3(0, up, 0)) * (speed * dt);
            fly_pos.y = maxf(fly_pos.y, terrain_height(fly_pos.x, fly_pos.z) + 0.3f);
            key = v2(0, 0);
            stick = v2(0, 0);
        }
        float stick_mag = minf(1.0f, sqrtf(stick.x * stick.x + stick.y * stick.y));
        vec2 in; float want_speed;
        if (key.x != 0 || key.y != 0) {
            float l = sqrtf(key.x * key.x + key.y * key.y);
            in = v2(key.x / l, key.y / l);
            want_speed = pix_key_down(KEY_SHIFT) ? gr.speed : gw.speed;
        } else if (stick_mag > 0.0f) {
            in = v2(stick.x / stick_mag, stick.y / stick_mag);
            // gentle tilt walks, a full push runs
            want_speed = stick_mag < 0.7f ? gw.speed * maxf(stick_mag / 0.7f, 0.35f)
                                          : lerpf(gw.speed, gr.speed, (stick_mag - 0.7f) / 0.3f);
        } else {
            in = v2(0, 0);
            want_speed = 0.0f;
        }
        vec3 cam_fwd = v3(-cosf(yaw), 0, -sinf(yaw)), cam_right = v3(-cam_fwd.z, 0, cam_fwd.x);
        vec3 wish = cam_fwd * in.y + cam_right * in.x;           // unit or zero
        bool alive = pl.move != MOVE_DEAD;
        bool jump = !flying && alive && (pix_key_pressed(KEY_SPACE) || pad.buttons[PAD_A].pressed);
        bool roll = !flying && alive && (pix_key_pressed('C') || pix_key_pressed(KEY_CTRL) || pad.buttons[PAD_B].pressed
                                         || (!pl.wolf && pad.buttons[PAD_X].pressed));
        bool shift = !flying && alive && (pix_key_pressed('Q') || pad.buttons[PAD_Y].pressed);
        bool claw = !flying && alive && pl.wolf && (pix_key_pressed('J') || pad.buttons[PAD_X].pressed
                                                    || (mouse_captured && !just_captured && pix_mouse_pressed(PIX_MOUSE_LEFT)));
        static float last_trigger = 0.0f;
        bool tackle = !flying && alive && pl.wolf && (pix_key_pressed('K') || pix_key_pressed('E')
                                                      || (pad.right_trigger > 0.6f && last_trigger <= 0.6f)
                                                      || (mouse_captured && pix_mouse_pressed(PIX_MOUSE_RIGHT)));
        last_trigger = pad.right_trigger;
        bool howl = !flying && alive && pl.wolf && (pix_key_pressed('H') || pad.buttons[PAD_DPAD_UP].pressed);

        idx pa = form_actor();
        float t_clip = pix_anim_time(anims, pa);
        float hspeed = sqrtf(pl.vel.x * pl.vel.x + pl.vel.z * pl.vel.z);
        pl.timer += dt;
        pl.air = pl.grounded ? 0.0f : pl.air + dt;
        pl.invuln = maxf(0.0f, pl.invuln - dt);
        pl.flash = maxf(0.0f, pl.flash - dt * 4.0f);
        pl.morph = maxf(0.0f, pl.morph - dt / 0.35f);
        pl.calm += dt;
        peace = maxf(0.0f, peace - dt);
        howl_cd = maxf(0.0f, howl_cd - dt);
        if (alive && pl.calm > 5.0f) pl.hp = minf(PLAYER_HP, pl.hp + 5.0f * dt);

        // ---- state machine ----
        bool on_feet = pl.move == MOVE_GROUND || pl.move == MOVE_LAND;
        // an attack can be chained once its last strike has landed
        bool swing_done = pl.move == MOVE_ATTACK && pl.struck >= beast.strikes[pl.attack]
                       && t_clip > beast.strike[pl.attack][beast.strikes[pl.attack] - 1] + 0.12f;
        if (on_feet && shift && pl.morph <= 0.0f) {
            // a burst of dust and fur, and the other body takes over mid-stride
            pl.wolf = !pl.wolf;
            pl.move = MOVE_GROUND;
            pl.clip = -1;
            pl.morph = 1.0f;
            pl.invuln = maxf(pl.invuln, 0.4f);
            pa = form_actor();
            if (pl.wolf) { int c = -1; wolf_restart(pa, c, WC_AGGRO, 0.0f, 1.4f); pl.move = MOVE_HURT; pl.timer = 0.0f; }
            else play(anims, pa, clips, CLIP_STAND, 0.0f, true, 1.0f);
            smoke(pl.pos + v3(0, 0.9f, 0), 1.0f);
            pix_play_sound(chime, 0.7f, pl.wolf ? 0.7f : 1.2f);
            rumble = maxf(rumble, 0.5f);
            t_clip = 0.0f;
        } else if (pl.wolf && on_feet && howl && howl_cd <= 0.0f) {
            // the howl: every hunting wolf close by loses its nerve and springs back
            pl.move = MOVE_HOWL;
            pl.timer = 0.0f;
            howl_cd = 8.0f;
            form_play(WC_HOWL, 0.15f, false, 1.0f);
            pix_anim_set_time(anims, pa, 0.0f);
            pl.clip = -1;
            pix_play_sound(chime, 0.8f, 0.5f);
            for (int i = 0; i < wolf_count; i++) {
                wolf& w = wolves[i];
                if (w.state == AI_DEAD || v3dist(w.pos, pl.pos) > 14.0f) continue;
                if (w.state == AI_IDLE || w.state == AI_WANDER) { alert_pack(w); continue; }
                float l;
                vec3 away = flat_to(pl.pos, w.pos, &l);
                w.heading = atan2f(-away.x, -away.z);          // face the howler, and back off
                w.state = AI_DODGE;
                w.timer = 0.0f;
                w.cooldown = maxf(w.cooldown, 2.5f);
                wolf_restart(w.actor, w.clip, WC_JUMP_BACK, 0.1f, 1.1f);
            }
            // an alpha is not cowed: it roars back, and comes on
            for (int i = 0; i < brute_count; i++) {
                brute& b = brutes[i];
                if (b.state == BR_DEAD || b.state == BR_ATTACK || b.state == BR_STAGGER || v3dist(b.pos, pl.pos) > 18.0f) continue;
                b.state = BR_ROAR;
                b.timer = 0.0f;
                b.anger = maxf(b.anger, 15.0f);
                brute_restart(b, MC_ROAR, 0.2f, 1.0f);
            }
        } else if ((pl.move == MOVE_GROUND || (pl.move == MOVE_LAND && t_clip > J().recover - 0.1f)) && jump) {
            pl.move = MOVE_TAKEOFF;
            // standing: a leap forward; walking: a bounding hop; running: a long flying leap
            if (pl.wolf) pl.jump_clip = hspeed > (gw.speed + gr.speed) * 0.5f ? WC_RUN_LEAP : (hspeed > 0.5f ? WC_RUN_JUMP : WC_JUMP_FWD);
            form_play(pl.wolf ? pl.jump_clip : CLIP_JUMP, 0.1f, false, pl.wolf && pl.jump_clip != WC_JUMP_FWD ? 1.1f : 1.3f);
            pix_anim_set_time(anims, pa, J().enter);
            t_clip = J().enter;
        } else if ((on_feet || swing_done) && roll) {
            // the human tumbles the way it is heading, the wolf springs back from the fight
            pl.move = MOVE_ROLL;
            pl.timer = 0.0f;
            vec3 d = v3len(wish) > 0.0f ? wish : forward(pl.heading);
            if (pl.wolf) d = forward(pl.heading) * -1.0f;
            else pl.heading = atan2f(d.x, d.z);
            pl.roll_dir_x = d.x; pl.roll_dir_z = d.z;
            form_play(pl.wolf ? WC_JUMP_BACK : CLIP_ROLL, 0.08f, false, 1.25f);
            pix_anim_set_time(anims, pa, 0.0f);
            t_clip = 0.0f;
            rumble = maxf(rumble, 0.2f);
        } else if (pl.wolf && pl.move == MOVE_AIR && tackle) {
            pl.slam = true;                                  // comes down claws first
        } else if (pl.wolf && (on_feet || swing_done) && (claw || tackle)) {
            // light: claw, claw, snap, double claw. heavy: a ram at a run, a lunging
            // bite to finish a chain, else a pounce or a lunge
            static const int CHAIN[4] = { ATK_CLAW_L, ATK_CLAW_R, ATK_BITE_SNAP, ATK_CLAW_DOUBLE };
            int atk;
            if (claw) {
                pl.combo = swing_done ? (pl.combo + 1) % 4 : 0;
                atk = CHAIN[pl.combo];
            } else {
                static bool pounce = false;
                pounce = !pounce;
                atk = hspeed > (gw.speed + gr.speed) * 0.5f ? ATK_RAM
                    : swing_done ? ATK_BITE_LUNGE : (pounce ? ATK_POUNCE : ATK_TACKLE_LUNGE);
                pl.combo = 0;
            }
            if (is_tackle(atk)) {
                // run it down first: the nearest foe ahead, else a short dash straight on
                vec3 aim = v3len(wish) > 0.0f ? wish : forward(pl.heading);
                pl.charging = true;
                pl.charge_attack = atk;
                pl.charge_time = 0.0f;
                pl.charge_kind = -1;
                float best = 14.0f;
                for (int i = 0; i < wolf_count; i++) {
                    float l;
                    vec3 d = flat_to(pl.pos, wolves[i].pos, &l);
                    if (wolves[i].state == AI_DEAD || l >= best || (l > 1.0f && v3dot(d, aim) < 0.3f)) continue;
                    best = l; pl.charge_kind = 0; pl.charge_index = i;
                }
                for (int i = 0; i < brute_count; i++) {
                    float l;
                    vec3 d = flat_to(pl.pos, brutes[i].pos, &l);
                    if (brutes[i].state == BR_DEAD || l >= best || (l > 1.0f && v3dot(d, aim) < 0.3f)) continue;
                    best = l; pl.charge_kind = 1; pl.charge_index = i;
                }
                if (pl.move != MOVE_GROUND) { pl.move = MOVE_GROUND; pl.clip = -1; pl.timer = 0.0f; }
            } else {
                start_attack(atk, v3len(wish) > 0.0f ? wish : v3(0, 0, 0));
                t_clip = 0.0f;
            }
        }

        // the run-up of a tackle: flat out at the target, springing once in range
        if (pl.charging && pl.move != MOVE_GROUND) pl.charging = false;
        if (pl.charging) {
            pl.charge_time += dt;
            bool has = false;
            vec3 tp = v3(0, 0, 0);
            if (pl.charge_kind == 0 && wolves[pl.charge_index].state != AI_DEAD) { has = true; tp = wolves[pl.charge_index].pos; }
            if (pl.charge_kind == 1 && brutes[pl.charge_index].state != BR_DEAD) { has = true; tp = brutes[pl.charge_index].pos; }
            float l = 0.0f;
            vec3 aim = has ? flat_to(pl.pos, tp, &l) : forward(pl.heading);
            if (v3len(aim) < 0.5f) aim = forward(pl.heading);
            wish = aim;
            want_speed = gr.speed * CHARGE_SPEED;
            const attack_def& A = ATTACKS[pl.charge_attack];
            float launch = A.reach * WOLF_SCALE + 0.4f + hspeed * beast.strike[pl.charge_attack][0] * 0.8f;
            bool facing = v3dot(forward(pl.heading), aim) > 0.85f;
            bool go = has ? l < launch && facing : pl.charge_time > 0.45f;
            if (go || pl.charge_time > 2.0f) {
                start_attack(pl.charge_attack, aim);
                pl.charged = true;
                t_clip = 0.0f;
            }
        }

        vec3 target_vel = wish * want_speed;
        pl.evading = false;
        switch (pl.move) {
        case MOVE_GROUND: {
            pl.vel.x = damp(pl.vel.x, target_vel.x, 8.0f, dt);
            pl.vel.z = damp(pl.vel.z, target_vel.z, 8.0f, dt);
            float speed = sqrtf(pl.vel.x * pl.vel.x + pl.vel.z * pl.vel.z);
            if (speed > 0.2f && want_speed > 0.0f) pl.heading = damp_angle(pl.heading, atan2f(wish.x, wish.z), pl.wolf ? 9.0f : 12.0f, dt);
            // walk / run chosen from the actual ground speed, with a little hysteresis
            int c_stand = pl.wolf ? WC_IDLE : CLIP_STAND, c_walk = pl.wolf ? WC_WALK : CLIP_WALK, c_run = pl.wolf ? WC_RUN : CLIP_RUN;
            float mid = (gw.speed + gr.speed) * 0.5f;
            int c = speed < 0.15f ? c_stand
                  : (pl.clip == c_run ? (speed > mid - 0.3f ? c_run : c_walk)
                                      : (speed > mid + 0.3f ? c_run : c_walk));
            float rate = c == c_walk ? clampf(speed / gw.clip_speed, 0.6f, 1.6f)
                       : c == c_run  ? (pl.wolf ? wolf_run_rate(speed) : clampf(speed / gr.clip_speed, 0.7f, 2.2f)) : 1.0f;
            if (c != pl.clip) form_play(c, c == c_stand ? 0.3f : 0.22f, true, rate);
            else pix_anim_set_speed(anims, pa, rate);
            pl.clip = c;
        } break;
        case MOVE_TAKEOFF:
            pl.vel.x = damp(pl.vel.x, target_vel.x, 4.0f, dt);
            pl.vel.z = damp(pl.vel.z, target_vel.z, 4.0f, dt);
            if (t_clip >= J().launch) {
                pl.vel.y = JUMP_V;
                if (pl.wolf) {
                    // a wolf's leap carries it forward: further the faster it was going
                    vec3 f = forward(pl.heading);
                    float s = pl.jump_clip == WC_RUN_LEAP ? maxf(hspeed, gr.speed) * 1.15f
                            : pl.jump_clip == WC_RUN_JUMP ? maxf(hspeed, gw.speed * 1.6f)
                            : gr.speed * 0.6f;
                    pl.vel.y = JUMP_V * (pl.jump_clip == WC_RUN_LEAP ? 1.15f : pl.jump_clip == WC_RUN_JUMP ? 0.85f : 1.1f);
                    pl.vel.x = f.x * s; pl.vel.z = f.z * s;
                }
                pl.move = MOVE_AIR;
                pl.grounded = false;
                pix_anim_set_speed(anims, pa, 1.05f);
            }
            break;
        case MOVE_AIR:
            pl.vel.x += (target_vel.x - pl.vel.x) * minf(1.0f, (pl.wolf ? 0.4f : 2.0f) * dt);      // a little air control
            pl.vel.z += (target_vel.z - pl.vel.z) * minf(1.0f, (pl.wolf ? 0.4f : 2.0f) * dt);
            if (v3len(wish) > 0.0f) pl.heading = damp_angle(pl.heading, atan2f(wish.x, wish.z), 4.0f, dt);
            if (t_clip >= J().hold) { pix_anim_set_speed(anims, pa, 0.0f); pix_anim_set_time(anims, pa, J().hold); }
            break;
        case MOVE_LAND:
            pl.vel.x = damp(pl.vel.x, target_vel.x * 0.5f, 10.0f, dt);
            pl.vel.z = damp(pl.vel.z, target_vel.z * 0.5f, 10.0f, dt);
            if (t_clip >= J().recover || (want_speed > 0.0f && t_clip >= J().touch + 0.1f) || pix_anim_finished(anims, pa)) {
                pl.move = MOVE_GROUND;
                pl.clip = -1;                               // re-pick a locomotion clip next frame
            }
            break;
        case MOVE_ROLL: {
            float len = pl.wolf ? beast.len[WC_JUMP_BACK] : roll_len;
            float k = t_clip / len;
            float top = pl.wolf ? 7.0f : maxf(run.speed * 1.5f, 3.0f);
            float spd = top * sqrtf(maxf(sinf(PIX_PI * minf(k, 1.0f)), 0.0f));
            pl.vel.x = pl.roll_dir_x * spd;
            pl.vel.z = pl.roll_dir_z * spd;
            pl.evading = k < (pl.wolf ? 0.6f : 0.7f);
            if (k >= 0.92f || pix_anim_finished(anims, pa)) { pl.move = MOVE_GROUND; pl.clip = -1; }
        } break;
        case MOVE_ATTACK: {
            const attack_def& A = ATTACKS[pl.attack];
            vec3 f = forward(pl.heading);
            // the body goes with the blow: a step into a swipe, a spring into a lunge
            if (pl.timer < dt * 1.5f) pl.carry = A.lunge > 0.0f && !pl.charged ? 0.0f : maxf(hspeed, gr.speed * 0.9f);
            float push = attack_push(beast, pl.attack, t_clip, pl.carry, pl.attack_rate, 1.1f * WOLF_SCALE);
            pl.vel.x = damp(pl.vel.x, f.x * push, 16.0f, dt);
            pl.vel.z = damp(pl.vel.z, f.z * push, 16.0f, dt);
            while (pl.struck < beast.strikes[pl.attack] && t_clip >= beast.strike[pl.attack][pl.struck]) {
                pl.struck++;
                player_strike(pl.attack, pl.struck - 1);
                if (pl.attack == ATK_SLAM) {
                    burst(parts, part_count, fx, pl.pos + v3(0, 0.2f, 0), 30, 6.0f, 0.1f, pix_rgba(140, 120, 90), 0.2f);
                    rumble = maxf(rumble, 0.7f);
                }
            }
            if (t_clip >= beast.len[A.clip] * 0.9f || pix_anim_finished(anims, pa)) { pl.move = MOVE_GROUND; pl.clip = -1; }
        } break;
        case MOVE_HOWL:
            pl.vel.x = damp(pl.vel.x, 0.0f, 6.0f, dt);
            pl.vel.z = damp(pl.vel.z, 0.0f, 6.0f, dt);
            if (t_clip >= beast.len[WC_HOWL] * 0.9f || pix_anim_finished(anims, pa)
                || (t_clip > beast.len[WC_HOWL] * 0.5f && want_speed > 0.0f)) { pl.move = MOVE_GROUND; pl.clip = -1; }
            break;
        case MOVE_HURT: {
            pl.vel.x = damp(pl.vel.x, 0.0f, 5.0f, dt);
            pl.vel.z = damp(pl.vel.z, 0.0f, 5.0f, dt);
            // after a change of form the new body snarls for a moment instead
            float hold = pl.morph > 0.0f || pl.clip < 0 ? 0.55f : 0.3f;
            if (pl.timer > hold) { pl.move = MOVE_GROUND; pl.clip = -1; }
        } break;
        case MOVE_DEAD:
            pl.vel.x = damp(pl.vel.x, 0.0f, 4.0f, dt);
            pl.vel.z = damp(pl.vel.z, 0.0f, 4.0f, dt);
            // once down, the body goes up in smoke
            if (pl.gone) pl.gone_for += dt;
            else if (pl.wolf ? pix_anim_time(anims, wolf_actor) >= beast.death_settle || pix_anim_finished(anims, wolf_actor)
                             : pl.timer > 1.4f) {
                pl.gone = true;
                pl.gone_for = 0.0f;
                smoke(pl.pos + v3(0, 0.6f, 0), 1.3f);
                pix_play_sound(chime, 0.6f, 0.6f);
            }
            if (pl.timer > 4.0f) {
                // rise again at the start, human, and the packs go home
                smoke(SPAWN + v3(0, 0.9f, 0), 1.0f);
                pl.pos = SPAWN;
                pl.vel = v3(0, 0, 0);
                pl.hp = PLAYER_HP;
                pl.wolf = false;
                pl.gone = false;
                pl.move = MOVE_GROUND;
                pl.clip = -1;
                pl.grounded = true;
                pl.morph = 1.0f;
                pl.invuln = 2.0f;
                peace = 8.0f;
                play(anims, actor, clips, CLIP_STAND, 0.0f, true, 1.0f);
                pix_play_sound(chime, 0.6f, 1.0f);
                for (int i = 0; i < wolf_count; i++) {
                    wolf& w = wolves[i];
                    w.anger = 0.0f;
                    if (w.state != AI_DEAD && w.state != AI_IDLE) {
                        w.state = AI_WANDER; w.timer = 0.0f; w.wait = 30.0f; w.goal = w.home;
                    }
                }
                for (int i = 0; i < brute_count; i++) {
                    brute& b = brutes[i];
                    b.anger = 0.0f;
                    if (b.state != BR_DEAD && b.state != BR_IDLE) {
                        b.state = BR_WANDER; b.timer = 0.0f; b.wait = 40.0f; b.goal = b.home;
                    }
                }
            }
            break;
        }

        // ---- integrate, collide, stick to the ground ----
        pl.pos.x += pl.vel.x * dt;
        pl.pos.z += pl.vel.z * dt;
        collide(pl.pos, pl.vel, player_radius());
        float ground = terrain_height(pl.pos.x, pl.pos.z);
        if (pl.grounded) {
            pl.pos.y = ground;
            pl.vel.y = 0.0f;
        } else {
            pl.vel.y -= GRAVITY * dt;
            pl.pos.y += pl.vel.y * dt;
            if (pl.pos.y <= ground && pl.vel.y < 0.0f) {
                float impact = -pl.vel.y;
                pl.pos.y = ground;
                pl.vel.y = 0.0f;
                pl.grounded = true;
                if (pl.move == MOVE_AIR || pl.move == MOVE_TAKEOFF) {
                    if (pl.wolf && pl.slam) {
                        start_attack(ATK_SLAM, v3(0, 0, 0));
                    } else if (pl.wolf && pl.air > 0.9f) {
                        // a long fall: the wolf takes it on all fours
                        pl.jump_clip = WC_LAND;
                        pl.move = MOVE_LAND;
                        form_play(WC_LAND, 0.05f, false, 1.2f);
                        pix_anim_set_time(anims, pa, 0.0f);
                    } else {
                        pl.move = MOVE_LAND;
                        pix_anim_set_speed(anims, pa, 1.3f);
                        pix_anim_set_time(anims, pa, maxf(pix_anim_time(anims, pa), J().touch));
                    }
                }
                pl.slam = false;
                pix_play_sound(step, minf(1.0f, impact * 0.15f), 0.75f);
                rumble = maxf(rumble, minf(0.6f, impact * 0.06f));
            }
        }

        lap(PR_PLAYER);
        // ---- the packs ----
        int attackers = 0;
        for (int i = 0; i < wolf_count; i++) attackers += wolves[i].state == AI_ATTACK || wolves[i].state == AI_CHARGE;
        for (int i = 0; i < wolf_count; i++) {
            wolf& w = wolves[i];
            w.timer += dt;
            w.cooldown -= dt;
            w.anger = maxf(0.0f, w.anger - dt);
            w.flash = maxf(0.0f, w.flash - dt * 4.0f);
            float d;
            vec3 to = flat_to(w.pos, pl.pos, &d);
            bool prey = pl.move != MOVE_DEAD && !flying && peace <= 0.0f;
            float t = pix_anim_time(anims, w.actor);
            float pr = player_radius();
            float wspeed = sqrtf(w.vel.x * w.vel.x + w.vel.z * w.vel.z);
            float mid = (beast.walk.speed + beast.run.speed) * 0.5f;
            vec3 want = v3(0, 0, 0);          // direction to go (and face), zero = stand
            float want_speed = 0.0f, turn = 6.0f;
            auto begin_attack = [&](int atk) {
                w.charged = w.state == AI_CHARGE;
                if (!w.charged) attackers++;
                w.state = AI_ATTACK;
                w.attack = atk;
                w.timer = 0.0f;
                w.struck = 0;
                wolf_restart(w.actor, w.clip, ATTACKS[atk].clip, 0.1f, 1.0f);
            };
            auto begin_charge = [&](int atk) {
                w.state = AI_CHARGE;
                w.attack = atk;
                w.timer = 0.0f;
                attackers++;
            };

            // anything calm notices the player close by, howls, and the pack answers
            if ((w.state == AI_IDLE || w.state == AI_WANDER) && prey && d < (pl.wolf ? 17.0f : 15.0f)) {
                w.state = AI_ALERT;
                w.timer = 0.0f;
                wolf_restart(w.actor, w.clip, WC_HOWL, 0.25f, 1.0f);
                alert_pack(w);
            }
            switch (w.state) {
            case AI_IDLE: {
                if (w.timer > w.wait) {
                    vec3 at;
                    w.state = AI_WANDER;
                    w.timer = 0.0f;
                    w.wait = 20.0f;
                    w.goal = open_spot(w.home, 12.0f, 0.0f, w.home, &at) ? at : w.home;
                }
            } break;
            case AI_WANDER: {
                float l;
                want = flat_to(w.pos, w.goal, &l);
                want_speed = beast.walk.speed;
                turn = 4.0f;
                if (l < 1.0f || w.timer > w.wait) wolf_rest(w);
            } break;
            case AI_ALERT:
                want = to;                           // face the player: a howl or a snarl
                turn = 5.0f;
                if (w.timer > beast.len[w.clip >= 0 ? w.clip : WC_AGGRO] * 0.85f) { w.state = AI_CHASE; w.timer = 0.0f; }
                break;
            case AI_CHASE: {
                if (!prey || (d > 34.0f && w.anger <= 0.0f)) {
                    w.state = AI_WANDER; w.timer = 0.0f; w.wait = 30.0f; w.goal = w.home;
                    break;
                }
                bool free = w.cooldown <= 0.0f && attackers < 2;
                if (free && d < 1.6f * WOLF_SCALE + pr) {
                    // up close: claws and jaws, now and then a leaping slam
                    static const int CLOSE[] = { ATK_CLAW_L, ATK_CLAW_R, ATK_BITE_SNAP, ATK_CLAW_DOUBLE, ATK_BITE_DOUBLE,
                                                 ATK_CLAW_L, ATK_CLAW_R, ATK_SLAM };
                    begin_attack(CLOSE[(int)(rng_float(ai) * 8) % 8]);
                    break;
                }
                // from further off: a charge that ends in a tackle, pounce or ram
                if (free && d > 5.0f && d < 12.0f && rng_float(ai) < dt * 0.9f) {
                    static const int TACKLES[] = { ATK_TACKLE_LUNGE, ATK_POUNCE, ATK_RAM };
                    begin_charge(wspeed > mid ? ATK_RAM : TACKLES[(int)(rng_float(ai) * 3) % 3]);
                    break;
                }
                if (free && d > 3.2f && d < 6.0f && rng_float(ai) < dt * 0.5f) { begin_attack(ATK_BITE_LUNGE); break; }
                if (wspeed > mid && d > 7.0f && d < 11.0f && w.timer > 2.0f && rng_float(ai) < dt * 0.5f) {
                    // a long bound to close the gap
                    w.state = AI_LEAP;
                    w.timer = 0.0f;
                    wolf_restart(w.actor, w.clip, WC_RUN_LEAP, 0.12f, 1.0f);
                    break;
                }
                if (d < 4.5f && !free) {
                    // wait for an opening: circle at a snapping distance, sometimes squaring up
                    if (w.cooldown > 0.8f && rng_float(ai) < dt * 0.15f) {
                        w.state = AI_TAUNT;
                        w.timer = 0.0f;
                        wolf_restart(w.actor, w.clip, WC_INTIMIDATE, 0.2f, 1.0f);
                        break;
                    }
                    vec3 side = v3(-to.z, 0.0f, to.x) * w.orbit;
                    vec3 v = side + to * clampf((d - 3.2f * WOLF_SCALE) * 0.8f, -1.0f, 1.0f);
                    float l = v3len(v);
                    want = l > 1e-4f ? v * (1.0f / l) : to;
                    want_speed = beast.walk.speed * 1.2f;
                } else {
                    want = to;
                    want_speed = d < 3.0f ? beast.walk.speed : beast.run.speed * 0.92f;
                    turn = 8.0f;
                }
            } break;
            case AI_CHARGE: {
                // flat out at the player, springing once close enough that the clip's
                // wind-up carries it the rest of the way
                const attack_def& A = ATTACKS[w.attack];
                want = to;
                turn = 9.0f;
                want_speed = beast.run.speed * CHARGE_SPEED;
                float launch = A.reach * WOLF_SCALE + pr + wspeed * beast.strike[w.attack][0] * 0.8f;
                if (!prey) { w.state = AI_CHASE; w.timer = 0.0f; break; }
                if (d < launch && v3dot(forward(w.heading), to) > 0.8f) { begin_attack(w.attack); break; }
                if (w.timer > 2.5f) { w.state = AI_CHASE; w.timer = 0.0f; w.cooldown = 0.5f; }
            } break;
            case AI_TAUNT:
                want = to;
                turn = 4.0f;
                if (t >= beast.len[WC_INTIMIDATE] * 0.9f || pix_anim_finished(anims, w.actor)) { w.state = AI_CHASE; w.timer = 0.0f; }
                break;
            case AI_LEAP: {
                // airborne at a gallop and a half, steering a little
                const jump_timing& lj = beast.jump[WC_RUN_LEAP];
                w.heading = damp_angle(w.heading, atan2f(to.x, to.z), 2.0f, dt);
                vec3 f = forward(w.heading);
                float s = t < lj.touch ? beast.run.speed * 1.25f : beast.run.speed * 0.8f;
                w.vel.x = f.x * s; w.vel.z = f.z * s;
                if (t >= beast.len[WC_RUN_LEAP] * 0.9f || pix_anim_finished(anims, w.actor)) { w.state = AI_CHASE; w.timer = 0.0f; }
                want_speed = -1.0f;
            } break;
            case AI_ATTACK: {
                const attack_def& A = ATTACKS[w.attack];
                float first = beast.strike[w.attack][0];
                // track the player through the wind-up
                if (t < first) w.heading = damp_angle(w.heading, atan2f(to.x, to.z), A.lunge < 0.0f ? 3.0f : 7.0f, dt);
                if (w.timer < dt * 1.5f) w.carry = A.lunge > 0.0f && !w.charged ? 0.0f : maxf(wspeed, beast.run.speed * 0.9f);
                // the body goes with the blow - shoving into the player rather than through them
                float push = attack_push(beast, w.attack, t, w.carry, 1.0f, WOLF_SCALE);
                if (A.lunge == 0.0f) push *= clampf((d - (0.8f * WOLF_SCALE + pr)) / 0.6f, 0.35f, 1.0f);
                vec3 f = forward(w.heading);
                w.vel.x = damp(w.vel.x, f.x * push, 16.0f, dt);
                w.vel.z = damp(w.vel.z, f.z * push, 16.0f, dt);
                while (w.struck < beast.strikes[w.attack] && t >= beast.strike[w.attack][w.struck]) {
                    w.struck++;
                    if (prey && d < A.reach * WOLF_SCALE + pr && (d < 0.5f || A.cone <= -1.0f || v3dot(to, forward(w.heading)) > A.cone))
                        hurt_player(A.damage, w.pos, A.knock, w.attack, w.struck - 1);
                    if (w.attack == ATK_SLAM)
                        burst(parts, part_count, fx, w.pos + v3(0, 0.2f, 0), 16, 5.0f, 0.08f, pix_rgba(140, 120, 90), 0.2f);
                }
                if (t >= beast.len[A.clip] * 0.9f || pix_anim_finished(anims, w.actor)) {
                    w.cooldown = rng_range(ai, 1.3f, 2.8f);
                    w.orbit = rng_float(ai) < 0.5f ? -1.0f : 1.0f;
                    w.timer = 0.0f;
                    if (rng_float(ai) < 0.3f) { w.state = AI_DODGE; wolf_restart(w.actor, w.clip, WC_JUMP_BACK, 0.08f, 1.2f); }
                    else w.state = AI_CHASE;
                }
                want_speed = -1.0f;                   // velocity set above
            } break;
            case AI_DODGE: {
                float k = t / beast.len[WC_JUMP_BACK];
                vec3 b = forward(w.heading) * (-6.0f * sqrtf(maxf(sinf(PIX_PI * minf(k, 1.0f)), 0.0f)));
                w.vel.x = b.x; w.vel.z = b.z;
                if (k >= 0.9f || pix_anim_finished(anims, w.actor)) { w.state = AI_CHASE; w.timer = 0.0f; }
                want_speed = -1.0f;
            } break;
            case AI_HIT:
                w.vel.x = damp(w.vel.x, 0.0f, 5.0f, dt);
                w.vel.z = damp(w.vel.z, 0.0f, 5.0f, dt);
                if (w.timer > beast.len[w.clip] * 0.75f || pix_anim_finished(anims, w.actor)) { w.state = AI_CHASE; w.timer = 0.0f; }
                want_speed = -1.0f;
                break;
            case AI_DEAD:
                w.vel.x = damp(w.vel.x, 0.0f, 4.0f, dt);
                w.vel.z = damp(w.vel.z, 0.0f, 4.0f, dt);
                // once it has fallen, it goes up in smoke
                if (w.gone) w.gone_for += dt;
                else if (t >= beast.death_settle || pix_anim_finished(anims, w.actor)) {
                    w.gone = true;
                    w.gone_for = 0.0f;
                    smoke(w.pos + v3(0, 0.6f, 0), 1.2f);
                    if (v3dist(w.pos, pl.pos) < 30.0f) pix_play_sound(chime, 0.5f * (1.0f - v3dist(w.pos, pl.pos) / 30.0f), 0.55f);
                }
                w.respawn -= dt;
                if (w.respawn <= 0.0f) {
                    // another wolf wanders in near the pack's home, out of sight of the player
                    vec3 at;
                    if (open_spot(w.home, 12.0f, 30.0f, pl.pos, &at)) {
                        w.pos = at;
                        w.vel = v3(0, 0, 0);
                        w.hp = WOLF_HP;
                        w.anger = 0.0f;
                        w.cooldown = 0.0f;
                        w.gone = false;
                        w.root = 0.0f;
                        w.clip = -1;
                        wolf_rest(w);
                    } else w.respawn = 5.0f;
                }
                want_speed = -1.0f;
                break;
            }
            if (want_speed >= 0.0f) {
                // locomotion: turn toward where it wants to go, move along its own heading
                if (v3len(want) > 0.0f) w.heading = damp_angle(w.heading, atan2f(want.x, want.z), turn, dt);
                vec3 f = forward(w.heading);
                float along = want_speed * maxf(v3dot(f, want), 0.2f);
                w.vel.x = damp(w.vel.x, f.x * along, 6.0f, dt);
                w.vel.z = damp(w.vel.z, f.z * along, 6.0f, dt);
                if (w.state == AI_WANDER || w.state == AI_CHASE || w.state == AI_CHARGE) {
                    float speed = sqrtf(w.vel.x * w.vel.x + w.vel.z * w.vel.z);
                    int c = speed < 0.2f ? WC_IDLE : (speed > mid ? WC_RUN : WC_WALK);
                    float rate = c == WC_WALK ? clampf(speed / beast.walk.clip_speed, 0.6f, 1.6f)
                               : c == WC_RUN  ? wolf_run_rate(speed) : 1.0f;
                    wolf_clip(w.actor, w.clip, c, 0.25f, true, rate);
                }
            }
            w.pos.x += w.vel.x * dt;
            w.pos.z += w.vel.z * dt;
            collide(w.pos, w.vel, 0.6f * WOLF_SCALE);
            w.pos.y = terrain_height(w.pos.x, w.pos.z);
        }

        lap(PR_WOLVES);
        // ---- the wolfmen ----
        for (int i = 0; i < brute_count; i++) {
            brute& b = brutes[i];
            b.timer += dt;
            b.cooldown -= dt;
            b.anger = maxf(0.0f, b.anger - dt);
            b.flash = maxf(0.0f, b.flash - dt * 4.0f);
            b.poise = maxf(0.0f, b.poise - 8.0f * dt);
            float d;
            vec3 to = flat_to(b.pos, pl.pos, &d);
            bool prey = pl.move != MOVE_DEAD && !flying && peace <= 0.0f;
            float t = pix_anim_time(anims, b.actor);
            float pr = player_radius();
            float close = 1.5f + pr;                  // in reach of its claws
            vec3 want = v3(0, 0, 0);
            float want_speed = 0.0f, turn = 6.0f;
            bool square = false;                      // keeps facing the player whichever way it moves
            bool hunting = b.state != BR_IDLE && b.state != BR_WANDER;
            auto go = [&](int st, int c, float fade, float speed) { b.state = st; b.timer = 0.0f; brute_restart(b, c, fade, speed); };
            auto begin_attack = [&](int atk) {
                b.attack = atk;
                b.struck = 0;
                go(BR_ATTACK, BRUTE_ATTACKS[atk].clip, 0.12f, 1.0f);
                if (atk == BA_POUNCE)             // the leap covers the gap, however wide
                    b.root_scale = clampf((d - 1.0f) / maxf(wm.root[MC_POUNCE][ROOT_SAMPLES - 1].y * WM_SCALE, 0.3f), 1.0f, 3.5f);
            };

            // spotting the player: a roar, or a howl every pack within earshot answers
            if ((b.state == BR_IDLE || b.state == BR_WANDER) && prey && d < (pl.wolf ? 22.0f : 18.0f)) {
                bool howl = rng_float(ai) < 0.5f;
                go(BR_ROAR, howl ? MC_HOWL : MC_ROAR, 0.25f, howl ? 1.3f : 1.0f);
                if (howl)
                    for (int k = 0; k < wolf_count; k++) {
                        wolf& w = wolves[k];
                        if (w.state == AI_DEAD || v3dist(w.pos, b.pos) > 45.0f) continue;
                        w.anger = maxf(w.anger, 12.0f);
                        if (w.state == AI_IDLE || w.state == AI_WANDER) alert_pack(w);
                    }
                pix_play_sound(chime, maxf(0.0f, 0.7f * (1.0f - d / 40.0f)), howl ? 0.4f : 0.3f);
            }
            switch (b.state) {
            case BR_IDLE:
                if (b.clip == MC_HOWL && t >= wm.len[MC_HOWL] * 0.95f) brute_loop(b, MC_IDLE, 0.4f, 1.0f);
                if (b.timer > b.wait && b.clip != MC_HOWL) {
                    vec3 at;
                    b.state = BR_WANDER;
                    b.timer = 0.0f;
                    b.wait = 25.0f;
                    b.goal = open_spot(b.home, 14.0f, 0.0f, b.home, &at) ? at : b.home;
                }
                break;
            case BR_WANDER: {
                float l;
                want = flat_to(b.pos, b.goal, &l);
                want_speed = wm.walk.speed;
                turn = 3.5f;
                if (l < 1.0f || b.timer > b.wait) brute_rest(b);
            } break;
            case BR_ROAR:
                want = to;
                turn = 4.0f;
                if (t >= wm.len[b.clip] * 0.85f || pix_anim_finished(anims, b.actor)) { b.state = BR_CHASE; b.timer = 0.0f; }
                break;
            case BR_CHASE: {
                if (!prey || (d > 40.0f && b.anger <= 0.0f)) {
                    b.state = BR_WANDER; b.timer = 0.0f; b.wait = 40.0f; b.goal = b.home;
                    break;
                }
                bool free = b.cooldown <= 0.0f;
                // the player swings at it: now and then it steps out of the way
                if (player_swung && d < 3.8f && b.timer > 0.4f && rng_float(ai) < 0.4f) {
                    int c = rng_float(ai) < 0.3f ? MC_JUMP_BACK : (rng_float(ai) < 0.5f ? MC_DODGE_L : MC_DODGE_R);
                    go(BR_DODGE, c, 0.06f, 1.15f);
                    b.root_scale = c == MC_JUMP_BACK ? 1.8f : 1.6f;
                    break;
                }
                if (free && d < close) {
                    // hurt, it presses harder: the combo comes out more
                    static const int CLOSE[] = { BA_CLAW_R, BA_CLAW_L, BA_DOUBLE, BA_BITE, BA_HEADBUTT, BA_CLAW_R, BA_CLAW_L, BA_COMBO };
                    int pick = (int)(rng_float(ai) * 8) % 8;
                    begin_attack(b.hp < BRUTE_HP * 0.5f && rng_float(ai) < 0.35f ? BA_COMBO : CLOSE[pick]);
                    break;
                }
                if (free && d > 3.5f && d < 7.0f && rng_float(ai) < dt * 0.9f) { begin_attack(BA_POUNCE); break; }
                if (d > 7.0f && d < 10.5f && b.timer > 1.5f && rng_float(ai) < dt * 0.4f) {
                    // a bound in over the gap, landing within reach
                    go(BR_LEAP, MC_JUMP, 0.12f, 1.0f);
                    b.leap = clampf((d - close) / maxf(wm.jump.touch - wm.jump.launch, 0.2f), 3.0f, 12.0f);
                    break;
                }
                if (free && d > 3.0f) {
                    // meaning to strike: closes the last few metres at a run
                    want = to;
                    want_speed = wm.run.speed * 0.8f;
                    turn = 7.0f;
                } else if (d < 6.5f) {
                    // squared off: stepping in when it means to strike, else circling
                    // just out of reach, backing off if crowded
                    square = true;
                    turn = 7.0f;
                    if (free) { want = to; want_speed = wm.walk.speed * 1.2f; }
                    else {
                        if (b.cooldown > 1.0f && b.timer > 2.0f && rng_float(ai) < dt * 0.12f) { go(BR_ROAR, MC_ROAR, 0.2f, 1.0f); break; }
                        vec3 side = v3(-to.z, 0.0f, to.x) * b.orbit;
                        vec3 v = side * 0.8f + to * clampf((d - 3.6f) * 0.8f, -1.0f, 1.0f);
                        float l = v3len(v);
                        want = l > 1e-4f ? v * (1.0f / l) : to;
                        want_speed = d < 2.6f ? wm.back.speed : wm.strafe.speed;
                    }
                } else {
                    want = to;
                    // far off it drops to all fours
                    want_speed = d > 14.0f || (b.clip == MC_RUN_BEAST && d > 10.0f) ? wm.gallop.speed : wm.run.speed;
                    turn = 5.0f;
                }
            } break;
            case BR_LEAP: {
                b.heading = damp_angle(b.heading, atan2f(to.x, to.z), 2.5f, dt);
                vec3 f = forward(b.heading);
                float s = t >= wm.jump.launch && t < wm.jump.touch ? b.leap : 0.0f;
                b.vel.x = damp(b.vel.x, f.x * s, 20.0f, dt);
                b.vel.z = damp(b.vel.z, f.z * s, 20.0f, dt);
                if (t >= wm.len[MC_JUMP] * 0.85f || pix_anim_finished(anims, b.actor)) {
                    b.state = BR_CHASE; b.timer = 0.0f;
                    if (d < close + 0.6f && prey) { b.cooldown = 0.0f; begin_attack(rng_float(ai) < 0.5f ? BA_DOUBLE : BA_HEADBUTT); }
                }
                want_speed = -1.0f;
            } break;
            case BR_ATTACK: {
                const brute_attack_def& A = BRUTE_ATTACKS[b.attack];
                float first = wm.strike[b.attack][0], last = wm.strike[b.attack][A.hits - 1];
                // tracks the player through the wind-up (and between the combo's blows)
                if (t < last) b.heading = damp_angle(b.heading, atan2f(to.x, to.z), b.attack == BA_POUNCE ? 3.0f : 6.0f, dt);
                vec3 f = forward(b.heading);
                float push = 0.0f;
                if (!wm.travels[A.clip]) {
                    // a step in behind the blow, short of walking through the player
                    float a0 = first * 0.3f, e = last + 0.2f;
                    if (t > a0 && t < e) push = 0.6f * WM_SCALE * PIX_PI / (2.0f * (e - a0)) * sinf(PIX_PI * (t - a0) / (e - a0));
                    push *= clampf((d - (0.7f * WM_SCALE + pr)) / 0.6f, 0.0f, 1.0f);
                }
                b.vel.x = damp(b.vel.x, f.x * push, 16.0f, dt);
                b.vel.z = damp(b.vel.z, f.z * push, 16.0f, dt);
                while (b.struck < A.hits && t >= wm.strike[b.attack][b.struck]) {
                    int s = b.struck++;
                    if (prey && d < A.reach + pr && (d < 0.5f || A.cone <= -1.0f || v3dot(to, f) > A.cone))
                        hurt_player(A.damage[s], b.pos, A.knock, A.fx[s], s);
                    // what it swings through, it hits: a wolfman can fell a tree fighting beside it
                    strike_trees(b.pos, f, A.reach + 0.3f, A.cone, A.damage[s] * 2.0f, A.fx[s], s);
                    if (b.attack == BA_POUNCE) {
                        burst(parts, part_count, fx, b.pos + f * 0.8f + v3(0, 0.2f, 0), 22, 5.5f, 0.1f, pix_rgba(140, 120, 90), 0.2f);
                        if (d < 12.0f) shake = maxf(shake, 0.4f);
                    }
                }
                if (t >= wm.len[A.clip] * 0.92f || pix_anim_finished(anims, b.actor)) {
                    bool enraged = b.hp < BRUTE_HP * 0.4f;
                    b.cooldown = enraged ? rng_range(ai, 0.5f, 1.3f) : rng_range(ai, 1.0f, 2.3f);
                    b.orbit = rng_float(ai) < 0.5f ? -1.0f : 1.0f;
                    if (rng_float(ai) < 0.25f) { go(BR_DODGE, MC_JUMP_BACK, 0.1f, 1.1f); b.root_scale = 1.8f; }
                    else { b.state = BR_CHASE; b.timer = 0.0f; }
                }
                want_speed = -1.0f;
            } break;
            case BR_DODGE:
                b.vel.x = damp(b.vel.x, 0.0f, 10.0f, dt);      // the clip carries it
                b.vel.z = damp(b.vel.z, 0.0f, 10.0f, dt);
                if (t < wm.len[b.clip] * 0.3f) b.heading = damp_angle(b.heading, atan2f(to.x, to.z), 4.0f, dt);
                if (t >= wm.len[b.clip] * 0.9f || pix_anim_finished(anims, b.actor)) { b.state = BR_CHASE; b.timer = 0.0f; }
                want_speed = -1.0f;
                break;
            case BR_HIT:
            case BR_STAGGER:
                b.vel.x = damp(b.vel.x, 0.0f, 5.0f, dt);
                b.vel.z = damp(b.vel.z, 0.0f, 5.0f, dt);
                if (t >= wm.len[b.clip] * (b.state == BR_HIT ? 0.8f : 0.9f) || pix_anim_finished(anims, b.actor)) {
                    b.state = BR_CHASE; b.timer = 0.0f;
                }
                want_speed = -1.0f;
                break;
            case BR_DEAD:
                b.vel.x = damp(b.vel.x, 0.0f, 4.0f, dt);
                b.vel.z = damp(b.vel.z, 0.0f, 4.0f, dt);
                if (b.gone) b.gone_for += dt;
                else if (t >= wm.death_settle || pix_anim_finished(anims, b.actor)) {
                    b.gone = true;
                    b.gone_for = 0.0f;
                    smoke(b.pos + v3(0, 0.5f, 0), 1.8f);
                    if (d < 30.0f) pix_play_sound(chime, 0.6f * (1.0f - d / 30.0f), 0.45f);
                }
                b.respawn -= dt;
                if (b.respawn <= 0.0f) {
                    // another alpha takes the ground, out of the player's sight
                    vec3 at;
                    if (open_spot(b.home, 14.0f, 35.0f, pl.pos, &at)) {
                        b.pos = at;
                        b.vel = v3(0, 0, 0);
                        b.hp = BRUTE_HP;
                        b.anger = b.cooldown = b.poise = 0.0f;
                        b.gone = false;
                        b.root_draw = v2(0, 0);
                        b.clip = -1;
                        brute_rest(b);
                    } else b.respawn = 5.0f;
                }
                want_speed = -1.0f;
                break;
            }
            if (want_speed >= 0.0f) {
                vec3 f = forward(b.heading);
                if (square) {
                    // faces the player and moves any way it likes: sidesteps, backs off
                    b.heading = damp_angle(b.heading, atan2f(to.x, to.z), turn, dt);
                    b.vel.x = damp(b.vel.x, want.x * want_speed, 6.0f, dt);
                    b.vel.z = damp(b.vel.z, want.z * want_speed, 6.0f, dt);
                } else {
                    if (v3len(want) > 0.0f) b.heading = damp_angle(b.heading, atan2f(want.x, want.z), turn, dt);
                    float along = want_speed * maxf(v3dot(f, want), 0.2f);
                    b.vel.x = damp(b.vel.x, f.x * along, 5.0f, dt);
                    b.vel.z = damp(b.vel.z, f.z * along, 5.0f, dt);
                }
                if (b.state == BR_WANDER || b.state == BR_CHASE) {
                    // the clip follows how it is moving relative to where it faces
                    f = forward(b.heading);
                    vec3 left = v3(f.z, 0.0f, -f.x);          // the model's +X
                    float ahead = v3dot(b.vel, f), aside = v3dot(b.vel, left);
                    float speed = sqrtf(b.vel.x * b.vel.x + b.vel.z * b.vel.z);
                    float run_mid = (wm.walk.speed + wm.run.speed) * 0.5f, gallop_mid = (wm.run.speed + wm.gallop.speed) * 0.5f;
                    int c; float rate = 1.0f;
                    if (speed < 0.25f) c = hunting ? MC_COMBAT_IDLE : MC_IDLE;
                    else if (fabsf(aside) > fabsf(ahead) * 1.2f) {
                        c = aside > 0.0f ? MC_STRAFE_L : MC_STRAFE_R;
                        rate = clampf(fabsf(aside) / wm.strafe.clip_speed, 0.6f, 1.6f);
                    } else if (ahead < 0.0f) {
                        c = MC_WALK_BACK;
                        rate = clampf(-ahead / wm.back.clip_speed, 0.6f, 1.6f);
                    } else if (speed > gallop_mid - (b.clip == MC_RUN_BEAST ? 0.4f : -0.4f)) {
                        c = MC_RUN_BEAST;
                        rate = clampf(speed / wm.gallop.clip_speed, 0.7f, 1.3f);
                    } else if (speed > run_mid - (b.clip == MC_RUN ? 0.3f : -0.3f)) {
                        c = MC_RUN;
                        rate = clampf(speed / wm.run.clip_speed, 0.7f, 1.6f);
                    } else {
                        c = MC_WALK;
                        rate = clampf(speed / wm.walk.clip_speed, 0.6f, 1.6f);
                    }
                    brute_loop(b, c, c == MC_RUN_BEAST || b.clip == MC_RUN_BEAST ? 0.3f : 0.22f, rate);
                }
            }
            b.pos.x += b.vel.x * dt;
            b.pos.z += b.vel.z * dt;
            collide(b.pos, b.vel, 0.55f * WM_SCALE);
            b.pos.y = terrain_height(b.pos.x, b.pos.z);
        }
        player_swung = false;

        lap(PR_BRUTES);
        // ---- bodies push each other apart ----
        for (int i = 0; i < wolf_count; i++) {
            wolf& a = wolves[i];
            if (a.state == AI_DEAD) continue;
            for (int j = i + 1; j < wolf_count; j++) {
                wolf& b = wolves[j];
                if (b.state == AI_DEAD) continue;
                float dx = b.pos.x - a.pos.x, dz = b.pos.z - a.pos.z, d2 = dx * dx + dz * dz;
                const float gap = 1.3f * WOLF_SCALE;
                if (d2 >= gap * gap || d2 < 1e-8f) continue;
                float d = sqrtf(d2), push = (gap - d) * 0.5f;
                a.pos.x -= dx / d * push; a.pos.z -= dz / d * push;
                b.pos.x += dx / d * push; b.pos.z += dz / d * push;
            }
            if (pl.move == MOVE_DEAD || !pl.grounded) continue;
            float need = 0.65f * WOLF_SCALE + player_radius();
            float dx = pl.pos.x - a.pos.x, dz = pl.pos.z - a.pos.z, d2 = dx * dx + dz * dz;
            if (d2 >= need * need || d2 < 1e-8f) continue;
            float d = sqrtf(d2), push = need - d;
            // the player gives way a little less than the wolf does
            pl.pos.x += dx / d * push * 0.4f; pl.pos.z += dz / d * push * 0.4f;
            a.pos.x -= dx / d * push * 0.6f; a.pos.z -= dz / d * push * 0.6f;
        }

        // a wolfman is heavy: whatever it bumps gives way more than it does
        for (int i = 0; i < brute_count; i++) {
            brute& b = brutes[i];
            if (b.state == BR_DEAD) continue;
            auto shove = [&](vec3& other, float need, float mine) {
                float dx = other.x - b.pos.x, dz = other.z - b.pos.z, d2 = dx * dx + dz * dz;
                if (d2 >= need * need || d2 < 1e-8f) return;
                float d = sqrtf(d2), push = need - d;
                other.x += dx / d * push * (1.0f - mine); other.z += dz / d * push * (1.0f - mine);
                b.pos.x -= dx / d * push * mine; b.pos.z -= dz / d * push * mine;
            };
            for (int j = 0; j < wolf_count; j++)
                if (wolves[j].state != AI_DEAD) shove(wolves[j].pos, 0.55f * WM_SCALE + 0.6f * WOLF_SCALE, 0.25f);
            for (int j = i + 1; j < brute_count; j++)
                if (brutes[j].state != BR_DEAD) shove(brutes[j].pos, 1.2f * WM_SCALE, 0.5f);
            if (pl.move != MOVE_DEAD && pl.grounded) shove(pl.pos, 0.55f * WM_SCALE + player_radius(), 0.35f);
        }

        // ---- the villager: strolls near the start, flees any hunting wolf, and you as one ----
        if (vil.actor != PIX_INVALID) {
            vil.timer += dt;
            vec3 threat = v3(0, 0, 0);
            float near_d = 1e9f;
            for (int i = 0; i < wolf_count; i++) {
                const wolf& w = wolves[i];
                if (w.state == AI_DEAD) continue;
                bool hunting = w.state >= AI_ALERT;
                float d = v3dist(w.pos, vil.pos);
                if (d < (hunting ? 14.0f : 6.0f) && d < near_d) { near_d = d; threat = w.pos; }
            }
            for (int i = 0; i < brute_count; i++) {
                const brute& b = brutes[i];
                if (b.state == BR_DEAD) continue;
                float d = v3dist(b.pos, vil.pos);
                if (d < (b.state >= BR_ROAR ? 18.0f : 9.0f) && d < near_d) { near_d = d; threat = b.pos; }
            }
            if (pl.wolf && pl.move != MOVE_DEAD) {
                float d = v3dist(pl.pos, vil.pos);
                if (d < 8.0f && d < near_d) { near_d = d; threat = pl.pos; }
            }
            vec3 want = v3(0, 0, 0);
            float speed_to = 0.0f;
            if (near_d < 1e8f) {
                float l;
                want = flat_to(threat, vil.pos, &l);
                // and not off the edge of the world: bend back toward the start
                float home_d;
                vec3 home = flat_to(vil.pos, SPAWN, &home_d);
                if (home_d > 30.0f) { want = v3norm(want + home * ((home_d - 30.0f) * 0.2f)); }
                speed_to = vrun.speed;
                vil.wait = 0.0f;
                vil.goal = vil.pos;
            } else {
                float l;
                vec3 dir = flat_to(vil.pos, vil.goal, &l);
                if (l > 0.8f) { want = dir; speed_to = vwalk.speed; vil.timer = 0.0f; }
                else if (vil.timer > vil.wait) {
                    vec3 at;
                    if (open_spot(SPAWN, 16.0f, 0.0f, SPAWN, &at)) vil.goal = at;
                    vil.wait = rng_range(ai, 2.0f, 7.0f);
                    vil.timer = 0.0f;
                } else if (!pl.wolf && v3dist(pl.pos, vil.pos) < 4.0f) {
                    // turns to greet the hero
                    float ld;
                    vec3 f = flat_to(vil.pos, pl.pos, &ld);
                    vil.heading = damp_angle(vil.heading, atan2f(f.x, f.z), 5.0f, dt);
                }
            }
            if (speed_to > 0.0f) vil.heading = damp_angle(vil.heading, atan2f(want.x, want.z), 8.0f, dt);
            vec3 f = forward(vil.heading);
            float along = speed_to * maxf(v3dot(f, want), 0.2f);
            vil.vel.x = damp(vil.vel.x, f.x * along, 6.0f, dt);
            vil.vel.z = damp(vil.vel.z, f.z * along, 6.0f, dt);
            vil.pos.x += vil.vel.x * dt;
            vil.pos.z += vil.vel.z * dt;
            collide(vil.pos, vil.vel, 0.35f);
            vil.pos.y = terrain_height(vil.pos.x, vil.pos.z);
            float speed = sqrtf(vil.vel.x * vil.vel.x + vil.vel.z * vil.vel.z);
            float mid = (vwalk.speed + vrun.speed) * 0.5f;
            int c = speed < 0.15f ? CLIP_STAND : (speed > mid ? CLIP_RUN : CLIP_WALK);
            float rate = c == CLIP_WALK ? clampf(speed / vwalk.clip_speed, 0.6f, 1.6f)
                       : c == CLIP_RUN  ? clampf(speed / vrun.clip_speed, 0.7f, 2.2f) : 1.0f;
            if (c != vil.clip) play(anims, vil.actor, vclips, c, 0.25f, true, rate);
            else pix_anim_set_speed(anims, vil.actor, rate);
            vil.clip = c;
        }

        lap(PR_MISC);
        // ---- falling trees ----
        trees_update(trees, tree_count, dt, obstacles, obstacle_count, tops, tree_clock);
        for (int i = 0; i < tree_count; i++) {
            tree_body& t = trees[i];
            for (int k = 0; k < t.pieces; k++) {
                const tree_piece& tp = t.piece[k];
                if (tp.impact < 2.5f) continue;
                // where it lands: dust, leaves, a thud felt through the ground
                float f = minf(tp.impact / 8.0f, 1.0f);
                burst(parts, part_count, fx, tp.impact_at + v3(0, 0.2f, 0), (int)(8 + 16 * f), 3.0f + 3.0f * f, 0.16f, DUST, 0.25f);
                if (tp.leafy) burst(parts, part_count, fx, tp.impact_at + v3(0, 0.5f, 0), (int)(6 + 14 * f), 3.5f, 0.07f, LEAF, 0.6f);
                float nearby = 1.0f - smoothstepf(8.0f, 40.0f, v3dist(tp.impact_at, pl.pos));
                shake = maxf(shake, 0.7f * f * nearby);
                rumble = maxf(rumble, 0.5f * f * nearby);
                if (tp.impact > 4.0f) pix_play_sound(step, minf(1.0f, 0.3f + f) * maxf(nearby, 0.2f), rng_range(fx, 0.2f, 0.28f));
                if (t.crushed || tp.impact < 4.0f) continue;
                // the first heavy landing comes down on whatever is beneath it
                t.crushed = true;
                vec3 ax[3];
                piece_axes(tp, ax);
                vec3 a = tp.com, b = tp.com;       // the blow comes from the box's middle
                auto under = [&](vec3 at, float rad) {
                    vec3 c = obb_closest(tp.com, ax, tp.half, at + v3(0, 0.5f, 0));
                    return v3dist(v3(c.x, 0, c.z), v3(at.x, 0, at.z)) < rad + 0.3f;
                };
                float dmg = 30.0f + 4.0f * t.height;
                for (int w = 0; w < wolf_count; w++) {
                    wolf& o = wolves[w];
                    if (o.state == AI_DEAD || !under(o.pos, 0.6f * WOLF_SCALE)) continue;
                    float l;
                    vec3 d = flat_to(closest_on_segment(a, b, o.pos), o.pos, &l);
                    hit_fx(o.pos, 0.7f * WOLF_SCALE, d, ATK_SLAM, 0, dmg, false, o.hp <= dmg);
                    hurt_wolf(o, dmg, d, 6.0f);
                }
                for (int w = 0; w < brute_count; w++) {
                    brute& o = brutes[w];
                    if (o.state == BR_DEAD || !under(o.pos, 0.8f * WM_SCALE)) continue;
                    float l;
                    vec3 d = flat_to(closest_on_segment(a, b, o.pos), o.pos, &l);
                    hit_fx(o.pos, 1.2f * WM_SCALE, d, ATK_SLAM, 0, dmg, false, o.hp <= dmg);
                    hurt_brute(o, dmg, d, 6.0f);
                }
                if (under(pl.pos, player_radius())) hurt_player(dmg * 0.5f, closest_on_segment(a, b, pl.pos), 6.0f, ATK_SLAM, 0);
            }
        }

        lap(PR_TREES);
        // ---- puffs ----
        for (int i = 0; i < part_count; ) {
            particle& p = parts[i];
            p.life -= dt;
            if (p.life <= 0.0f) { p = parts[--part_count]; continue; }
            if (!p.glow) p.vel.y -= 6.0f * dt;
            p.vel = p.vel * (1.0f - minf(1.0f, (p.glow ? 5.0f : 2.5f) * dt));
            p.pos += p.vel * dt;
            p.pos.y = maxf(p.pos.y, terrain_height(p.pos.x, p.pos.z) + p.size);
            i++;
        }
        for (int i = 0; i < streak_count; ) {
            streaks[i].life -= real_dt;
            if (streaks[i].life <= 0.0f) { streaks[i] = streaks[--streak_count]; continue; }
            i++;
        }
        for (int i = 0; i < popup_count; ) {
            popups[i].life -= real_dt;
            popups[i].pos.y += real_dt * 1.1f;
            if (popups[i].life <= 0.0f) { popups[i] = popups[--popup_count]; continue; }
            i++;
        }

        lap(PR_PUFFS);
        // ---- animate ----
        pix_anim_set_position(anims, actor, pl.wolf ? pl.pos + v3(0, -500, 0) : pl.pos);     // the idle form is far away: its LOD drops
        pix_anim_set_position(anims, wolf_actor, pl.wolf ? pl.pos : pl.pos + v3(0, -500, 0));
        for (int i = 0; i < wolf_count; i++) pix_anim_set_position(anims, wolves[i].actor, wolves[i].pos);
        for (int i = 0; i < brute_count; i++) pix_anim_set_position(anims, brutes[i].actor, brutes[i].pos);
        if (vil.actor != PIX_INVALID) pix_anim_set_position(anims, vil.actor, vil.pos);
        pix_anim_update(anims, dt, focus);
        // a wolfman's travelling clips move its position by as much as they
        // move its hips (root_scale further for a long pounce or a spring
        // back); the drawing takes the clip's own offset back off
        for (int i = 0; i < brute_count; i++) {
            brute& b = brutes[i];
            if (b.clip >= 0 && wm.travels[b.clip]) {
                vec2 rt = brute_root(wm, b.clip, pix_anim_time(anims, b.actor));
                vec2 step_by = v2(rt.x - b.root_prev.x, rt.y - b.root_prev.y);
                b.root_prev = rt;
                b.root_draw = rt;
                b.pos += yaw_rotate(step_by, b.heading) * (WM_SCALE * b.root_scale);
                vec3 v0 = v3(0, 0, 0);
                collide(b.pos, v0, 0.55f * WM_SCALE);
                b.pos.y = terrain_height(b.pos.x, b.pos.z);
            } else {
                b.root_draw.x = damp(b.root_draw.x, 0.0f, 14.0f, dt);
                b.root_draw.y = damp(b.root_draw.y, 0.0f, 14.0f, dt);
            }
        }
        int ev_count = 0;
        const pix_anim_fired* ev = pix_anim_events(anims, &ev_count);
        for (int i = 0; i < ev_count; i++) {
            if (ev[i].id != EVENT_STEP) continue;
            if (ev[i].actor == form_actor() && pl.move == MOVE_GROUND) {
                float speed = sqrtf(pl.vel.x * pl.vel.x + pl.vel.z * pl.vel.z);
                pix_play_sound(step, 0.25f + 0.1f * speed, rng_range(fx, 0.9f, 1.1f) * (pl.wolf ? 0.8f : 1.0f));
                if (pl.clip == (pl.wolf ? WC_RUN : CLIP_RUN)) rumble = maxf(rumble, 0.08f);
                continue;
            }
            // everyone else's footfalls, quieter with distance
            vec3 at = ev[i].actor == vil.actor ? vil.pos : v3(1e9f, 0, 0);
            for (int k = 0; k < wolf_count; k++) if (wolves[k].actor == ev[i].actor) at = wolves[k].pos;
            bool heavy = false;
            for (int k = 0; k < brute_count; k++) if (brutes[k].actor == ev[i].actor) { at = brutes[k].pos; heavy = true; }
            float d = v3dist(at, pl.pos), range = heavy ? 24.0f : 16.0f;
            if (d < range) pix_play_sound(step, (heavy ? 0.55f : 0.3f) * (1.0f - d / range), heavy ? rng_range(fx, 0.55f, 0.65f) : rng_range(fx, 0.75f, 0.95f));
        }
        if (rumble > 0.0f) {
            rumble = maxf(0.0f, rumble - dt * 2.5f);
            pix_set_gamepad_rumble(pix_get_window(), 0, rumble * 0.7f, rumble);
        }

        lap(PR_ANIM);
        // ---- grass: the wind, and whoever walks through it ----
        g.wind = v3(0.8f, 0.6f, wind);
        g.pusher_count = 0;
        g.pushers[g.pusher_count++] = v4(pl.pos.x, pl.pos.y, pl.pos.z, pl.move == MOVE_ROLL ? 0.6f : (pl.wolf ? 0.75f * WOLF_SCALE : 0.45f));
        if (vil.actor != PIX_INVALID) g.pushers[g.pusher_count++] = v4(vil.pos.x, vil.pos.y, vil.pos.z, 0.45f);
        for (int i = 0; i < brute_count && g.pusher_count < 8; i++)
            if (!brutes[i].gone && v3dist(brutes[i].pos, focus) < 60.0f)
                g.pushers[g.pusher_count++] = v4(brutes[i].pos.x, brutes[i].pos.y, brutes[i].pos.z, (brutes[i].state == BR_DEAD ? 1.0f : 0.7f) * WM_SCALE);
        {
            // the nearest wolves to the camera
            int order[MAX_WOLVES];
            for (int i = 0; i < wolf_count; i++) order[i] = i;
            for (int i = 1; i < wolf_count; i++)
                for (int j = i; j > 0 && v3dist(wolves[order[j]].pos, focus) < v3dist(wolves[order[j - 1]].pos, focus); j--) {
                    int tmp = order[j]; order[j] = order[j - 1]; order[j - 1] = tmp;
                }
            for (int i = 0; i < wolf_count && g.pusher_count < 8; i++) {
                const wolf& w = wolves[order[i]];
                if (w.gone) continue;
                g.pushers[g.pusher_count++] = v4(w.pos.x, w.pos.y, w.pos.z, (w.state == AI_DEAD ? 0.9f : 0.75f) * WOLF_SCALE);
            }
        }

        // ---- camera: follows smoothly, never dips under the hills ----
        vec3 want_focus = pl.pos + v3(0, pl.wolf ? 1.0f * WOLF_SCALE : 1.2f, 0);
        focus.x = damp(focus.x, want_focus.x, 10.0f, dt);
        focus.z = damp(focus.z, want_focus.z, 10.0f, dt);
        focus.y = damp(focus.y, want_focus.y, pl.grounded ? 8.0f : 3.0f, dt);
        camera cam = {};
        cam.position = focus + v3(cosf(yaw) * cosf(pitch), sinf(pitch), sinf(yaw) * cosf(pitch)) * dist;
        float cam_ground = terrain_height(cam.position.x, cam.position.z) + 0.4f;
        if (cam.position.y < cam_ground) cam.position.y = cam_ground;
        cam.direction = focus - cam.position;
        if (flying) {
            cam.position = fly_pos;
            cam.direction = v3(-cosf(yaw) * cosf(pitch), -sinf(pitch), -sinf(yaw) * cosf(pitch));
        }
        if (shake > 0.0f && !flying) {
            float t = (float)pix_time();
            cam.position += v3(sinf(t * 53.0f), sinf(t * 47.0f + 1.3f), sinf(t * 61.0f + 2.1f)) * (shake * shake * 0.12f);
        }
        cam.fovy = 55.0f * PIX_PI / 180.0f;
        cam.far_plane = 400.0f;

        lap(PR_GRASS);
        pix_begin_3d(cam);
        pix_draw_mesh(terrain, soil, mat4_identity());
        pix_draw_mesh(terrain_shadow, soil_shadow, mat4_identity());
        for (int i = 0; i < g_pond_count; i++) pix_draw_mesh(pond_mesh[i], water, mat4_identity());
        for (int i = 0; i < prop_count; i++) {
            const prop& p = props[i];
            const prop_kind& k = kinds[p.kind];
            mat4 m = mat4_trs_y(p.pos, p.yaw, p.scale);
            if (prop_tree[i] >= 0 && trees[prop_tree[i]].state != TREE_STANDING) {
                // broken: the stump where it stood, and each piece where it went
                const tree_body& tb = trees[prop_tree[i]];
                int stump = slab_index(CUT_BOTTOM, CUT_STUMP);
                for (int q = 0; q < k.parts; q++) pix_draw_mesh(k.mesh[q], k.slab[q][stump], m);
                for (int n = 0; n < tb.pieces; n++) {
                    const tree_piece& tp = tb.piece[n];
                    if (tp.state == PIECE_GONE && tp.gone_for > 0.25f) continue;
                    mat4 pm = piece_transform(tb, &tp, m, (float)pix_time());
                    int sl = slab_index(tp.lo, tp.hi);
                    for (int q = 0; q < k.parts; q++) pix_draw_mesh(k.mesh[q], k.slab[q][sl], pm);
                }
                continue;
            }
            if (prop_tree[i] >= 0) m = piece_transform(trees[prop_tree[i]], 0, m, (float)pix_time());
            for (int q = 0; q < k.parts; q++) pix_draw_mesh(k.mesh[q], k.material[q], m);
        }
        if (show_boxes)
            for (int i = 0; i < tree_count; i++)
                for (int k = 0; k < trees[i].pieces; k++) {
                    const tree_piece& tp = trees[i].piece[k];
                    if (tp.state == PIECE_GONE) continue;
                    vec3 ax[3];
                    piece_axes(tp, ax);
                    draw_piece_box(tp.com, ax, tp.half, tp.state == PIECE_FALLING ? pix_rgba(255, 220, 60) : pix_rgba(120, 200, 255));
                    if (tp.wood) draw_piece_box(piece_place(tp, tp.wood0), ax, tp.wood_half, pix_rgba(255, 120, 60));
                }
        // a hit flashes red; the change of form pops the new body out of the dust
        auto flash_tint = [](pix_color base, float f) {
            return pix_rgba(base.r, (int)(base.g * (1.0f - 0.3f * f)), (int)(base.b * (1.0f - 0.3f * f)));
        };
        float e = 1.0f - pl.morph;
        float morph_scale = pl.morph > 0.0f ? 0.25f + 0.75f * e + 0.2f * sinf(e * PIX_PI) : 1.0f;
        // a body going up in smoke shrinks into the puff
        auto vanish_scale = [](float since) { return 1.0f - smoothstepf(0.0f, 0.2f, since); };
        float pl_vanish = pl.gone ? vanish_scale(pl.gone_for) : 1.0f;
        // Clips with root motion carry the body away from the origin; draw it
        // back by as much, so the body stays on the physics position. After the
        // clip the offset eases out with the cross-fade instead of snapping.
        int pl_rc = !pl.wolf ? -1 : pl.move == MOVE_ROLL ? WC_JUMP_BACK
                  : (pl.move == MOVE_TAKEOFF || pl.move == MOVE_AIR || pl.move == MOVE_LAND) ? pl.jump_clip : -1;
        pl.root = pl_rc >= 0 && beast.travels[pl_rc] ? root_travel(beast, pl_rc, pix_anim_time(anims, wolf_actor))
                                                     : damp(pl.root, 0.0f, 14.0f, dt);
        int bone_count = 0;
        const mat4* bones = pix_anim_bones(anims, form_actor(), &bone_count);
        if (pl_vanish <= 0.0f) {
            // gone: nothing to draw until it rises again
        } else if (pl.wolf) {
            float turn = pl.heading + (beast.facing < 0.0f ? PIX_PI : 0.0f);
            float sc = 1.1f * WOLF_SCALE * morph_scale * pl_vanish;
            vec3 at = pl.pos - forward(pl.heading) * (pl.root * sc);
            pix_draw_model_bones(beast.model, bones, bone_count, mat4_trs_y(at, turn, sc), flash_tint(PIX_WHITE, pl.flash));
        } else {
            float turn = pl.heading + (model_facing < 0.0f ? PIX_PI : 0.0f);
            pix_draw_model_bones(hero, bones, bone_count, mat4_trs_y(pl.pos, turn, morph_scale * pl_vanish), flash_tint(PIX_WHITE, pl.flash));
        }
        for (int i = 0; i < wolf_count; i++) {
            wolf& w = wolves[i];
            w.root = w.clip >= 0 && beast.travels[w.clip] ? root_travel(beast, w.clip, pix_anim_time(anims, w.actor))
                                                          : damp(w.root, 0.0f, 14.0f, dt);
            float sc = w.gone ? vanish_scale(w.gone_for) : 1.0f;
            if (sc <= 0.0f) continue;
            const mat4* wb = pix_anim_bones(anims, w.actor, &bone_count);
            float turn = w.heading + (beast.facing < 0.0f ? PIX_PI : 0.0f);
            sc *= WOLF_SCALE;
            vec3 at = w.pos - forward(w.heading) * (w.root * sc);
            // the wild wolves wear a darker, cooler coat than the player's
            pix_draw_model_bones(beast.model, wb, bone_count, mat4_trs_y(at, turn, sc),
                                 flash_tint(pix_rgba(150, 150, 168), w.flash));
        }
        for (int i = 0; i < brute_count; i++) {
            const brute& b = brutes[i];
            float sc = b.gone ? vanish_scale(b.gone_for) : 1.0f;
            if (sc <= 0.0f) continue;
            sc *= WM_SCALE;
            const mat4* bb = pix_anim_bones(anims, b.actor, &bone_count);
            vec3 at = b.pos - yaw_rotate(b.root_draw, b.heading) * sc;
            mat4 m = mat4_trs_y(at, b.heading, sc);
            pix_model part = wm.model;
            for (int p = 0; p < wm.parts; p++) {
                part.mesh = wm.part_mesh[p];
                part.material = wm.part_mat[p];
                pix_draw_model_bones(part, bb, bone_count, m, p == 0 ? flash_tint(PIX_WHITE, b.flash) : PIX_WHITE);
            }
        }
        if (vil.actor != PIX_INVALID) {
            const mat4* vb = pix_anim_bones(anims, vil.actor, &bone_count);
            float turn = vil.heading + (vwalk.facing < 0.0f ? PIX_PI : 0.0f);
            pix_draw_model_bones(runner, vb, bone_count, mat4_trs_y(vil.pos, turn, 1.0f));
        }
        for (int i = 0; i < part_count; i++) {
            const particle& p = parts[i];
            float k = p.life / p.max_life;
            if (p.glow) {
                pix_color c = pix_rgba((int)(p.color.r * k), (int)(p.color.g * k), (int)(p.color.b * k));
                pix_draw_mesh(puff_mesh, glow_mat, mat4_trs_y(p.pos, 0.0f, p.size), c);
            } else {
                pix_draw_mesh(puff_mesh, puff_mat, mat4_trs_y(p.pos, 0.0f, p.size * (0.4f + 0.6f * k)), p.color);
            }
        }
        for (int i = 0; i < streak_count; i++) {
            // a unit cube stretched along the streak: long axis, thin width, thinner depth
            const streak& st = streaks[i];
            float age = 1.0f - st.life / st.max_life;
            float grow = smoothstepf(0.0f, 0.25f, age), fade = 1.0f - smoothstepf(0.3f, 1.0f, age);
            float len = st.len * grow, w = st.width * (0.3f + 0.7f * fade);
            vec3 depth = v3norm(v3cross(st.axis, st.up));
            vec3 c = st.start + st.axis * (len * 0.5f);
            mat4 m = mat4_identity();
            m.data[0] = st.axis.x * len; m.data[1] = st.axis.y * len; m.data[2] = st.axis.z * len;
            m.data[4] = st.up.x * w;     m.data[5] = st.up.y * w;     m.data[6] = st.up.z * w;
            m.data[8] = depth.x * w * 0.4f; m.data[9] = depth.y * w * 0.4f; m.data[10] = depth.z * w * 0.4f;
            m.data[12] = c.x; m.data[13] = c.y; m.data[14] = c.z;
            pix_draw_mesh(bar_mesh, glow_mat, m, pix_rgba((int)(st.color.r * fade), (int)(st.color.g * fade), (int)(st.color.b * fade)));
        }
        lap(PR_SUBMIT);
        pix_end_3d();
        lap(PR_RENDER);

        // ---- damage numbers ----
        for (int i = 0; i < popup_count; i++) {
            const popup& p = popups[i];
            vec2 sp;
            if (!pix_world_to_screen(r, p.pos, &sp)) continue;
            float a = minf(1.0f, p.life / 0.3f);
            float pop = 1.0f + 0.5f * smoothstepf(0.75f, 0.9f, p.life);     // swells in, then settles
            char buf[16];
            snprintf(buf, sizeof(buf), "%d", p.amount);
            float size = p.size * pop, x = sp.x - size * 0.3f * (float)strlen(buf);
            pix_draw_text(buf, x + 2, sp.y + 2, size, pix_rgba(0, 0, 0, (int)(160 * a)));
            pix_draw_text(buf, x, sp.y, size, pix_rgba(p.color.r, p.color.g, p.color.b, (int)(255 * a)));
        }
        // being hurt reddens the edges of the view
        if (pl.flash > 0.0f) {
            // nested frames, each a little fainter inward: a soft red glow at the edges
            float W = (float)pix_width(), H = (float)pix_height();
            const int BANDS = 10;
            const float e = 14.0f;
            for (int b = 0; b < BANDS; b++) {
                float o = b * e, fall = 1.0f - (float)b / BANDS;
                pix_color red = pix_rgba(190, 10, 5, (int)(34 * pl.flash * fall * fall));
                pix_draw_rect(o, o, W - 2 * o, e, red);
                pix_draw_rect(o, H - o - e, W - 2 * o, e, red);
                pix_draw_rect(o, o + e, e, H - 2 * (o + e), red);
                pix_draw_rect(W - o - e, o + e, e, H - 2 * (o + e), red);
            }
        }

        // ---- health over the wolves that are fighting or hurt ----
        for (int i = 0; i < wolf_count; i++) {
            const wolf& w = wolves[i];
            if (w.state == AI_DEAD || (w.state < AI_ALERT && w.hp >= WOLF_HP) || v3dist(w.pos, cam.position) > 30.0f) continue;
            vec2 s;
            if (!pix_world_to_screen(r, w.pos + v3(0, 1.65f * WOLF_SCALE, 0), &s)) continue;
            pix_draw_rect(s.x - 26, s.y - 3, 52, 6, pix_rgba(0, 0, 0, 150));
            pix_draw_rect(s.x - 25, s.y - 2, 50 * w.hp / WOLF_HP, 4, pix_rgba(220, 70, 50));
        }

        // ...and over a tree or log being cut down
        for (int i = 0; i < tree_count; i++) {
            const tree_body& tb = trees[i];
            if (tb.shown <= 0.0f || tb.state == TREE_TIPPING) continue;
            bool standing = tb.state == TREE_STANDING;
            const tree_piece* tp = !standing && tb.shown_piece >= 0 ? &tb.piece[tb.shown_piece] : 0;
            if (!standing && (!tp || tp->state == PIECE_GONE)) continue;
            vec2 s;
            if (!pix_world_to_screen(r, standing ? tb.root + v3(0, 2.8f, 0) : tp->com + v3(0, tb.trunk + 1.0f, 0), &s)) continue;
            float frac = standing ? tb.hp / tb.max_hp : tp->hp / (18.0f + 2.5f * tp->len);
            int a = (int)(255 * minf(tb.shown, 1.0f));
            pix_draw_rect(s.x - 36, s.y - 3, 72, 6, pix_rgba(0, 0, 0, a * 150 / 255));
            pix_draw_rect(s.x - 35, s.y - 2, 70 * maxf(frac, 0.0f), 4, pix_rgba(200, 160, 90, a));
        }

        for (int i = 0; i < brute_count; i++) {
            const brute& b = brutes[i];
            if (b.state == BR_DEAD || (b.state <= BR_WANDER && b.hp >= BRUTE_HP) || v3dist(b.pos, cam.position) > 40.0f) continue;
            vec2 s;
            if (!pix_world_to_screen(r, b.pos + v3(0, 2.35f * WM_SCALE, 0), &s)) continue;
            pix_draw_rect(s.x - 46, s.y - 4, 92, 8, pix_rgba(0, 0, 0, 160));
            pix_draw_rect(s.x - 45, s.y - 3, 90 * b.hp / BRUTE_HP, 6, b.state == BR_STAGGER ? pix_rgba(255, 200, 80) : pix_rgba(200, 40, 60));
        }

        static const char* MOVE_NAMES[] = { "ground", "take-off", "air", "landing", "roll", "attack", "hurt", "down", "howl" };
        pix_draw_rect_rounded({ 12, 12, 640, 88 }, 10, pix_rgba(0, 0, 0, 110));
        pix_draw_textf(24, 20, 20, PIX_WHITE, "%s   %.1f m/s   %s   wolves slain: %d   wolfmen: %d   pad: %s",
                       pl.wolf ? "WOLF" : "human", sqrtf(pl.vel.x * pl.vel.x + pl.vel.z * pl.vel.z), MOVE_NAMES[pl.move],
                       kills, brute_kills, pad.connected ? "connected" : "none");
        pix_draw_rect(24, 46, 204, 12, pix_rgba(0, 0, 0, 160));
        pix_draw_rect(26, 48, 200 * pl.hp / PLAYER_HP, 8, pl.hp < 30.0f ? pix_rgba(230, 70, 50) : pix_rgba(110, 210, 90));
        pix_draw_text(flying ? "FREE CAMERA  WASD fly  Space/C up/down  Shift fast  F back to the hero"
                    : pl.wolf ? "WASD prowl  Shift run  Space leap  LMB/J claw combo  RMB/K tackle (in air: slam)  C spring back  H howl  Q human"
                              : "WASD move  Shift run  Space jump  C roll  Q wolf form  click: mouse look  F fly  G toon",
                      24, 66, 15, PIX_LIGHTGRAY);
        if (pl.move == MOVE_DEAD) {
            const char* msg = "The pack brought you down...";
            pix_draw_text(msg, (float)pix_width() * 0.5f - 150, (float)pix_height() * 0.4f, 26, PIX_WHITE);
        }
        pix_draw_fps((float)pix_width() - 90, 12);
        lap(PR_HUD);
        if (pix_profile_frame(prof))
            printf("gpu   shadow %.2f  mirror %.2f  view %.2f  post %.2f ms   %d draws  %d instances\n",
                   pix_gpu_pass_ms(r, PIX_GPU_SHADOW), pix_gpu_pass_ms(r, PIX_GPU_REFLECT), pix_gpu_pass_ms(r, PIX_GPU_VIEW),
                   pix_gpu_pass_ms(r, PIX_GPU_POST), r.frame_draws, r.frame_instances);
    }
    pix_stop_gamepad_rumble(pix_get_window());
    glDeleteVertexArrays(1, &g.vao);
    glDeleteBuffers(1, &g.vbo);
    glDeleteBuffers(1, &g.ibo);
    glDeleteTextures(1, &field_tex);
    pix_destroy_gpu_buffer(g.patches);
    pix_destroy_gpu_buffer(g.visible);
    pix_destroy_gpu_buffer(g.args);
    pix_free(wolves);
    pix_free(brutes);
    pix_free(parts);
    pix_free(streaks);
    pix_free(popups);
    pix_free(props);
    pix_free(obstacles);
    pix_free(kinds);
    pix_shutdown();
    return 0;
}
