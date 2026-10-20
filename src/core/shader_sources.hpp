// pix - built-in GLSL. Everything the renderer draws with, plus the building
// blocks (scene block, BRDF, lights, shadows) for writing your own surface
// shaders that light exactly like the built-in one - see load_surface_shader.
#ifndef PIX_SHADER_SOURCES_HPP
#define PIX_SHADER_SOURCES_HPP
#include "dtype.hpp"

// ---- the per-frame block ----
//
// Everything about the frame that every shader agrees on - the sun, the sky,
// the camera, the shadow cascades - in one std140 uniform buffer, uploaded
// once and bound for the whole frame at binding 0. Every member is a vec4 or
// mat4 on purpose: std140 pads a vec3 to sixteen bytes anyway, and keeping
// both sides trivially comparable is the only real defence against the C and
// GLSL layouts drifting apart. Mirrored by pix_scene_block in renderer.hpp.
static const char* const GLSL_SCENE = R"(
layout(std140, binding = 0) uniform SceneBlock {
    mat4 bViewProj;       // the main camera
    mat4 bView;
    vec4 bSunDir;         // xyz normalised, toward the sun
    vec4 bSunColor;       // rgb linear radiance
    vec4 bSkyColor;       // rgb hemisphere irradiance from above
    vec4 bGroundColor;    // rgb hemisphere irradiance bounced from below
    vec4 bSkyZenith;      // rgb dome colour straight up
    vec4 bSkyHorizon;     // rgb dome colour at the horizon
    vec4 bSkyGround;      // rgb dome colour below the horizon
    vec4 bFog;            // rgb linear colour, w density
    vec4 bCamera;         // xyz eye, w seconds (pix_set_time)
    vec4 bCameraDir;      // xyz forward
    vec4 bShadow;         // xy one atlas texel, z strength (0 = off), w PCF radius
    mat4 bLightSpace[3];  // one projection per cascade, near to far
    vec4 bCascadeFar;     // xyz view depth each cascade ends at
    vec4 bCascadeTexel;   // xyz one shadow texel in world units
    mat4 bReflViewProj;   // the planar mirror's camera
    vec4 bClipPlane;      // mirror pass: keep dot(n, p) + w >= 0; all zero = no clip
    vec4 bReflect;        // x planar map ready, y env probe ready, z probe max mip, w probe intensity
    vec4 bScreen;         // xy one pixel of the render target in uv
    vec4 bShadowEye;      // xyz the main camera, which the cascades are fitted around
    vec4 bShadowDir;      // xyz its forward
};
#define uViewProj       bViewProj
#define uSunDir         bSunDir.xyz
#define uSunColor       bSunColor.rgb
#define uSkyColor       bSkyColor.rgb
#define uGroundColor    bGroundColor.rgb
#define uSkyZenith      bSkyZenith.rgb
#define uSkyHorizon     bSkyHorizon.rgb
#define uSkyGround      bSkyGround.rgb
#define uFogColor       bFog.rgb
#define uFogDensity     bFog.w
#define uCameraPos      bCamera.xyz
#define uTime           bCamera.w
#define uShadowTexel    bShadow.xy
#define uShadowStrength bShadow.z
#define uLightSpace     bLightSpace
#define uCascadeFar     bCascadeFar
#define uCascadeTexel   bCascadeTexel
)";

static const char* const GLSL_COMMON = R"(
vec3 srgb_to_linear(vec3 c) { return pow(max(c, 0.0), vec3(2.2)); }
vec3 linear_to_srgb(vec3 c) { return pow(max(c, 0.0), vec3(1.0 / 2.2)); }

// Linear radiance of the analytic sky along `dir`: a horizon-to-zenith
// gradient, a ground half below the horizon, and the sun as a disc with two
// softer glow lobes. The backdrop and every reflection sample this one
// function, so they always agree.
vec3 sky_env(vec3 dir) {
    float up = dir.y;
    vec3 sky;
    if (up >= 0.0) sky = mix(uSkyZenith, uSkyHorizon, pow(1.0 - up, 5.0));
    else           sky = mix(uSkyHorizon, uSkyGround, clamp(-up * 3.0, 0.0, 1.0));
    float sd = max(dot(dir, normalize(uSunDir)), 0.0);
    sky += uSunColor * pow(sd, 1800.0) * 16.0;         // disc
    sky += uSunColor * pow(sd, 28.0)   * 0.30;         // tight glow
    sky += uSunColor * pow(sd, 5.0)    * 0.06;         // wide forward scatter
    return sky;
}

vec3 apply_fog(vec3 color, vec3 world) {
    float d = length(uCameraPos - world);
    float fog = 1.0 - exp(-d * d * uFogDensity * uFogDensity);
    return mix(color, uFogColor, clamp(fog, 0.0, 1.0));
}
)";

// ---- shared BRDF ----
//
// Standard metallic/roughness model, named after the papers it comes from, so
// a surface behaves the way the published model says rather than the way it
// happened to be tuned.
static const char* const GLSL_BRDF = R"(
const float PI = 3.14159265359;

// GGX / Trowbridge-Reitz normal distribution (Walter et al. 2007)
float d_ggx(float ndh, float a) {
    float a2 = a * a;
    float d = ndh * ndh * (a2 - 1.0) + 1.0;
    return a2 / max(PI * d * d, 1e-8);
}
// height-correlated Smith visibility (Heitz 2014) - folds in the 1/(4 ndl ndv)
float v_smith(float ndv, float ndl, float a) {
    float a2 = a * a;
    float gv = ndl * sqrt(ndv * ndv * (1.0 - a2) + a2);
    float gl = ndv * sqrt(ndl * ndl * (1.0 - a2) + a2);
    return 0.5 / max(gv + gl, 1e-5);
}
vec3 f_schlick(float u, vec3 f0) {
    float m = clamp(1.0 - u, 0.0, 1.0);
    return f0 + (1.0 - f0) * (m * m * m * m * m);
}
// Karis' analytic environment BRDF fit, so ambient specular carries the right
// Fresnel and roughness weight without a lookup table
vec3 env_brdf(vec3 f0, float rough, float ndv) {
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572,  0.022);
    const vec4 c1 = vec4( 1.0,  0.0425,  1.04,  -0.04);
    vec4 r = rough * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * ndv)) * r.x + r.y;
    vec2 ab = vec2(-1.04, 1.04) * a004 + r.zw;
    return f0 * ab.x + ab.y;
}

vec3 surface_brdf(vec3 n, vec3 v, vec3 l, vec3 diff_albedo, vec3 f0, float a, vec3 radiance) {
    vec3  h = normalize(l + v);
    float ndl = max(dot(n, l), 0.0);
    if (ndl <= 0.0) return vec3(0.0);
    float ndv = max(dot(n, v), 1e-4);
    float ndh = max(dot(n, h), 0.0);
    float vdh = max(dot(v, h), 0.0);
    vec3 F = f_schlick(vdh, f0);
    vec3 spec = d_ggx(ndh, a) * v_smith(ndv, ndl, a) * F;
    vec3 kd = vec3(1.0) - F;
    return (kd * diff_albedo / PI + spec) * radiance * ndl;
}

// Where a rough surface's specular lobe actually points (Lagarde 2014)
vec3 specular_dominant_dir(vec3 n, vec3 r, float rough) {
    float smoothness = clamp(1.0 - rough, 0.0, 1.0);
    float lerp_factor = smoothness * (sqrt(smoothness) + rough);
    return normalize(mix(n, r, lerp_factor));
}

// Windowed inverse-square falloff (Karis 2013): physically correct near the
// light, exactly zero at `radius` so lights can be culled without a seam.
float light_falloff(float dist2, float radius) {
    float r2 = radius * radius;
    float f = clamp(1.0 - (dist2 * dist2) / max(r2 * r2, 1e-6), 0.0, 1.0);
    return (f * f) / max(dist2, 0.01);
}
)";

// ---- punctual lights, binding 1 ----
static const char* const GLSL_LIGHTS = R"(
layout(std140, binding = 1) uniform LightBlock {
    vec4 uLightPosRadius[32];    // xyz position, w radius
    vec4 uLightColorInner[32];   // rgb linear radiance, w cos(inner cone)
    vec4 uLightDirOuter[32];     // xyz direction, w cos(outer cone), < -1 = point
    vec4 uLightMeta;             // x: how many of the above are live
};
#define uLightCount int(uLightMeta.x)

vec3 punctual_lights(vec3 world, vec3 n, vec3 v, vec3 diff_albedo, vec3 f0, float a) {
    vec3 sum = vec3(0.0);
    for (int i = 0; i < uLightCount; ++i) {
        vec3  to = uLightPosRadius[i].xyz - world;
        float d2 = dot(to, to);
        float radius = uLightPosRadius[i].w;
        if (d2 > radius * radius) continue;
        vec3  l = to * inversesqrt(max(d2, 1e-8));
        float atten = light_falloff(d2, radius);
        float cos_outer = uLightDirOuter[i].w;
        if (cos_outer > -1.0) {                       // a cone, not a bulb
            float cd = dot(-l, uLightDirOuter[i].xyz);
            float t = clamp((cd - cos_outer) / max(uLightColorInner[i].w - cos_outer, 1e-4), 0.0, 1.0);
            atten *= t * t;
        }
        if (atten <= 0.0) continue;
        sum += surface_brdf(n, v, l, diff_albedo, f0, a, uLightColorInner[i].rgb * atten);
    }
    return sum;
}
)";

// ---- cascaded shadow maps ----
//
// The view is split by depth into three slices, each with its own tightly
// fitted, texel-snapped map, side by side in one atlas. Normal-offset sampling
// instead of a large depth bias, and the last stretch of each cascade
// cross-fades into the next so the resolution change is not a visible line.
static const char* const GLSL_SHADOW = R"(
uniform sampler2DShadow uShadowMap;

float shadow_cascade(int c, vec3 world, vec3 n, float ndl) {
    float slope = clamp(1.0 - ndl, 0.0, 1.0);
    vec3 p_world = world + n * (uCascadeTexel[c] * (1.0 + 2.0 * slope) * 1.4);
    vec4 lp = uLightSpace[c] * vec4(p_world, 1.0);
    vec3 p = lp.xyz / lp.w * 0.5 + 0.5;
    if (p.z > 1.0 || p.z < 0.0) return 1.0;
    float inset = uShadowTexel.x * 2.0;
    p.x = clamp(p.x, inset, 1.0 - inset) / 3.0 + float(c) / 3.0;
    p.y = clamp(p.y, uShadowTexel.y * 2.0, 1.0 - uShadowTexel.y * 2.0);
    float bias = 0.0004;
    // radius 0 is one hardware-filtered tap (2x2 PCF for free); 1 and 2 widen the kernel
    int k = int(bShadow.w);
    float sum = 0.0;
    for (int y = -k; y <= k; ++y)
        for (int x = -k; x <= k; ++x)
            sum += texture(uShadowMap, vec3(p.xy + vec2(float(x) * uShadowTexel.x,
                                                        float(y) * uShadowTexel.y), p.z - bias));
    float taps = float((2 * k + 1) * (2 * k + 1));
    return sum / taps;
}

float sun_shadow(vec3 world, vec3 n, float ndl) {
    if (uShadowStrength <= 0.0) return 1.0;
    // measured from the main camera even while rendering a mirror or a probe:
    // that is who the cascades were fitted around
    float depth = dot(world - bShadowEye.xyz, bShadowDir.xyz);
    if (depth > uCascadeFar.z) return 1.0;
    int c = 2;
    if (depth < uCascadeFar.x)      c = 0;
    else if (depth < uCascadeFar.y) c = 1;
    float s = shadow_cascade(c, world, n, ndl);
    float far_edge = uCascadeFar[c];
    float band = far_edge * 0.12;
    float t = clamp((depth - (far_edge - band)) / band, 0.0, 1.0);
    if (t > 0.0) s = mix(s, c < 2 ? shadow_cascade(c + 1, world, n, ndl) : 1.0, t);
    return mix(1.0, s, uShadowStrength);
}
)";

// ---- noise ----
// Value noise and fbm, for procedural surfaces (water, lava, clouds, dissolve).
static const char* const GLSL_NOISE = R"(
float hash12(vec2 p) {
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}
float value_noise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash12(i), hash12(i + vec2(1, 0)), u.x),
               mix(hash12(i + vec2(0, 1)), hash12(i + vec2(1, 1)), u.x), u.y);
}
float fbm(vec2 p, int octaves) {
    float sum = 0.0, amp = 0.5;
    for (int i = 0; i < octaves; ++i) { sum += amp * value_noise(p); p = p * 2.03 + 17.1; amp *= 0.5; }
    return sum;
}
)";

// ---- the standard lit surface ----//
// shade_surface() is the whole lighting model in one call: sun with cascaded
// shadows, every punctual light, ambient from the sky (or the reflection probe
// when one is running), Fresnel-weighted reflections, emission and fog.
// Custom surface shaders compute their own albedo / normal / roughness and
// hand them to it. shade_surface_ex adds a planar-mirror weight.
//
// Reflection sources, best available first:
//   planar mirror  exact reflection across r.reflections.plane (uPlanarMap)
//   probe          the scene around the camera, as a cubemap (uEnvMap)
//   analytic sky   always there, costs nothing
static const char* const GLSL_SURFACE = R"(
uniform samplerCube uEnvMap;
uniform sampler2D   uPlanarMap;
uniform sampler2D   uRefractMap;
uniform sampler2D   uSceneDepth;   // depth of the opaque scene, for see-through surfaces

// view-space distance from a 0..1 depth-buffer value (perspective cameras)
float linearize_depth(float d) {
    float n = bScreen.z, f = bScreen.w;
    return 2.0 * n * f / (f + n - (d * 2.0 - 1.0) * (f - n));
}
// how far away the opaque scene is behind this pixel, and how far this pixel is:
// their difference is the thickness of water / glass / fog volume in between
float scene_depth_at(vec2 uv) { return linearize_depth(texture(uSceneDepth, uv).r); }
float fragment_depth() { return linearize_depth(gl_FragCoord.z); }
vec2  screen_uv() { return gl_FragCoord.xy * bScreen.xy; }
// the mirror pass renders only what lies on the visible side of the mirror
void clip_test(vec3 world) {
    if (dot(bClipPlane.xyz, bClipPlane.xyz) > 0.0 && dot(bClipPlane.xyz, world) + bClipPlane.w < 0.0) discard;
}

// incoming light from direction `dir`, blurred to match `rough`
vec3 env_specular(vec3 dir, float rough) {
    if (bReflect.y > 0.5) return textureLod(uEnvMap, dir, rough * bReflect.z).rgb * bReflect.w;
    vec3 flat_dome = mix(uSkyHorizon, uSkyZenith, clamp(dir.y * 0.5 + 0.5, 0.0, 1.0));
    return mix(sky_env(dir), flat_dome, rough * rough);
}

// the mirror image at this surface point; nudged by the normal so ripples and
// bumps break the reflection up
vec3 planar_reflection(vec3 world, vec3 n, float rough) {
    vec4 rc = bReflViewProj * vec4(world + vec3(n.x, 0.0, n.z) * 0.25, 1.0);
    vec2 uv = clamp(rc.xy / max(rc.w, 1e-4) * 0.5 + 0.5, vec2(0.002), vec2(0.998));
    return textureLod(uPlanarMap, uv, rough * 6.0).rgb;
}

// what lies behind this pixel, bent by the surface normal - glass, water, heat haze
vec3 refraction_color(vec3 n, float strength) {
    vec2 uv = gl_FragCoord.xy * bScreen.xy;
    vec3 vn = (bView * vec4(n, 0.0)).xyz;
    return texture(uRefractMap, clamp(uv + vn.xy * strength, vec2(0.001), vec2(0.999))).rgb;
}

vec3 shade_surface_ex(vec3 world, vec3 n, vec3 albedo, float metal, float rough, vec3 emissive, float planar) {
    vec3 v = normalize(uCameraPos - world);
    float ndv = max(dot(n, v), 1e-4);
    rough = clamp(rough, 0.02, 1.0);
    metal = clamp(metal, 0.0, 1.0);
    float a = rough * rough;
    vec3 f0 = mix(vec3(0.04), albedo, metal);
    vec3 diff_albedo = albedo * (1.0 - metal);

    vec3  l = normalize(uSunDir);
    float ndl = max(dot(n, l), 0.0);
    vec3 direct = vec3(0.0);
    if (ndl > 0.0) direct = surface_brdf(n, v, l, diff_albedo, f0, a, uSunColor) * sun_shadow(world, n, ndl);
    direct += punctual_lights(world, n, v, diff_albedo, f0, a);

    float hemi = n.y * 0.5 + 0.5;
    vec3 irradiance = mix(uGroundColor, uSkyColor, hemi);
    // the probe's blurriest mip is a cheap stand-in for bounced light
    if (bReflect.y > 0.5) irradiance = mix(irradiance, textureLod(uEnvMap, n, bReflect.z).rgb * bReflect.w, 0.5);
    vec3 amb_diffuse = irradiance * diff_albedo;

    vec3 refl = specular_dominant_dir(n, reflect(-v, n), rough);
    vec3 env = env_specular(refl, rough);
    if (planar > 0.0 && bReflect.x > 0.5) env = mix(env, planar_reflection(world, n, rough), planar);
    float horizon = clamp(1.0 + dot(reflect(-v, n), n), 0.0, 1.0);   // Frostbite horizon occlusion
    vec3 amb_spec = env * env_brdf(f0, rough, ndv) * (horizon * horizon);
    float ao = mix(0.45, 1.0, hemi);   // down-facing crevices see less sky

    return apply_fog(direct + amb_diffuse * ao + amb_spec * mix(ao, 1.0, planar) + emissive, world);
}

vec3 shade_surface(vec3 world, vec3 n, vec3 albedo, float metal, float rough, vec3 emissive) {
    return shade_surface_ex(world, n, albedo, metal, rough, emissive, 0.0);
}

// ---- cel / toon shading ----
// params: x (unused here - the blend amount), y light bands, z rim strength,
// w hard-highlight size. Band edges are antialiased with fwidth so they stay
// one pixel wide at any distance. Shadows, lights, ambient and fog all come
// from the same sources as the PBR path, so the two mix cleanly.
float toon_step(float x, float bands) {
    float b = max(bands, 1.0);
    float s = x * b;
    float w = max(fwidth(s), 1e-4);
    return (floor(s) + smoothstep(0.5 - w, 0.5 + w, fract(s))) / b;
}

vec3 shade_toon(vec3 world, vec3 n, vec3 albedo, vec4 params, vec3 emissive) {
    vec3 v = normalize(uCameraPos - world);
    vec3 l = normalize(uSunDir);
    float ndl = max(dot(n, l), 0.0);
    float lit = toon_step(ndl * sun_shadow(world, n, ndl), params.y);
    vec3 light = uSunColor * lit / PI + mix(uGroundColor, uSkyColor, n.y * 0.5 + 0.5);

    for (int i = 0; i < uLightCount; ++i) {
        vec3  to = uLightPosRadius[i].xyz - world;
        float d2 = dot(to, to);
        float radius = uLightPosRadius[i].w;
        if (d2 > radius * radius) continue;
        vec3 pl = to * inversesqrt(max(d2, 1e-8));
        float atten = clamp(1.0 - d2 / (radius * radius), 0.0, 1.0);
        float cos_outer = uLightDirOuter[i].w;
        if (cos_outer > -1.0)
            atten *= clamp((dot(-pl, uLightDirOuter[i].xyz) - cos_outer)
                           / max(uLightColorInner[i].w - cos_outer, 1e-4), 0.0, 1.0);
        light += uLightColorInner[i].rgb * toon_step(max(dot(n, pl), 0.0) * atten, params.y) * 0.25;
    }
    vec3 color = albedo * light;

    if (params.w > 0.0) {   // a hard, round highlight
        float spec = dot(n, normalize(l + v));
        float edge = 1.0 - params.w * 0.08;
        float w = max(fwidth(spec), 1e-4);
        color += uSunColor * smoothstep(edge - w, edge + w, spec) * lit * 0.3;
    }
    if (params.z > 0.0) {   // rim: a bright outline where the surface turns away
        float rim = 1.0 - max(dot(n, v), 0.0);
        float w = max(fwidth(rim), 1e-4);
        color += (uSkyColor + uSunColor * 0.15) * smoothstep(0.65 - w, 0.65 + w, rim) * params.z;
    }
    return apply_fog(color + emissive, world);
})";
// ---- static meshes ----
// per-vertex pos/normal/uv (loc 0..2), per-instance model matrix (loc 3..6)
// and tint (loc 7). Instanced: one draw per (material, mesh) run.
static const char* const VSHDER_BASIC = R"(#version 440 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aUV;
layout(location=3) in mat4 aModel;   // consumes 3..6
layout(location=7) in vec4 aTint;
out vec3 vNormal;
out vec2 vUV;
out vec3 vWorld;
out vec4 vTint;
void main() {
    vec4 world = aModel * vec4(aPos, 1.0);
    vWorld = world.xyz;
    gl_Position = uViewProj * world;
    vNormal = mat3(aModel) * aNormal;
    vUV = aUV;
    vTint = aTint;
}
)";

// ---- skinned meshes: instanced GPU skinning ----
//
// Every animated instance of the frame lives in storage buffers - its model
// matrix, tint and where its bones start in one shared palette - and each draw
// names a run of them through a per-view index list. So a crowd sharing a
// mesh and material is one instanced draw, whatever its size, and a rig's
// bone count is limited only by memory.
static const char* const GLSL_SKINNING = R"(
struct SkinInstance { mat4 model; vec4 tint; uvec4 info; };   // info.x: first bone in the palette
layout(std430, binding = 8)  readonly buffer PixSkinInstances { SkinInstance skin_instances[]; };
layout(std430, binding = 9)  readonly buffer PixSkinBones     { mat4 skin_bones[]; };
layout(std430, binding = 10) readonly buffer PixSkinList       { uint skin_list[]; };
uniform int uSkinBase;          // where this draw's run starts in skin_list

SkinInstance skin_instance() { return skin_instances[skin_list[uSkinBase + gl_InstanceID]]; }
mat4 skin_matrix(SkinInstance si, vec4 ids, vec4 weights) {
    uint b = si.info.x;
    return skin_bones[b + uint(ids.x)] * weights.x + skin_bones[b + uint(ids.y)] * weights.y
         + skin_bones[b + uint(ids.z)] * weights.z + skin_bones[b + uint(ids.w)] * weights.w;
}
)";

static const char* const VSHDER_SKINNED = R"(#version 440 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aUV;
layout(location=3) in vec4 aBoneIds;
layout(location=4) in vec4 aWeights;
out vec3 vNormal;
out vec2 vUV;
out vec3 vWorld;
out vec4 vTint;
void main() {
    SkinInstance si = skin_instance();
    mat4 skin = skin_matrix(si, aBoneIds, aWeights);
    vec4 world = si.model * (skin * vec4(aPos, 1.0));
    vWorld = world.xyz;
    gl_Position = uViewProj * world;
    vNormal = mat3(si.model) * (mat3(skin) * aNormal);
    vUV = aUV;
    vTint = si.tint;
}
)";
// The default material. Albedo and tint arrive as sRGB and are linearised;
// all lighting is in linear space and the post pass owns tonemap + encode.
static const char* const FSHDER_BASIC = R"(#version 440 core
in vec3 vNormal;
in vec2 vUV;
in vec3 vWorld;
in vec4 vTint;
out vec4 frag;

uniform sampler2D uTex;
uniform vec4  uColor;         // rgb tint (sRGB), a opacity
uniform float uMetallic;
uniform float uRoughness;
uniform vec3  uEmissive;      // linear radiance added on top
uniform float uAlphaCutoff;   // > 0: cut-out, texels below it are discarded
uniform int   uUnlit;
uniform float uPlanar;        // 0..1: how much of the planar mirror this surface shows
uniform float uRefraction;    // > 0: see-through, bending what is behind by this much
uniform vec4  uToon;          // x toon amount (0 PBR .. 1 cel), y bands, z rim, w highlight size

void main() {
    clip_test(vWorld);
    vec4 tex = texture(uTex, vUV);
    float alpha = tex.a * uColor.a * vTint.a;
    if (alpha < uAlphaCutoff) discard;
    vec3 albedo = srgb_to_linear(tex.rgb * uColor.rgb * vTint.rgb);
    if (uUnlit != 0) { frag = vec4(albedo + uEmissive, alpha); return; }
    vec3 n = normalize(vNormal);
    if (!gl_FrontFacing) n = -n;   // double-sided materials light their back faces too
    vec3 lit;
    if (uToon.x >= 0.999)     lit = shade_toon(vWorld, n, albedo, uToon, uEmissive);
    else if (uToon.x > 0.0)   lit = mix(shade_surface_ex(vWorld, n, albedo, uMetallic, uRoughness, uEmissive, uPlanar),
                                        shade_toon(vWorld, n, albedo, uToon, uEmissive), uToon.x);
    else                      lit = shade_surface_ex(vWorld, n, albedo, uMetallic, uRoughness, uEmissive, uPlanar);
    if (uRefraction > 0.0) {
        // glass: the scene behind, tinted by the surface colour, with the lit
        // surface (its reflections and highlights) laid over it by opacity
        frag = vec4(mix(refraction_color(n, uRefraction) * albedo, lit, alpha), 1.0);
        return;
    }
    frag = vec4(lit, alpha);
}
)";

// ---- built-in water ----
//
// Both water shaders read the same material fields:
//   color      shallow colour (sRGB)            params0.rgb  deep colour (sRGB)
//   roughness  reflection blur                  params0.w    depth where it is fully deep
//   planar_reflection, refraction               params1.x    wave scale (1 = default size)
//                                               params1.y    wave speed
//                                               params1.z    foam amount (0 = none)
//                                               params1.w    wave strength (normal tilt)
// Depth behind every pixel (uSceneDepth) is what makes water read as water:
// clear at the shore, deep-coloured in the middle, foam where it meets things.
static const char* const GLSL_WATER_COMMON = R"(
in vec3 vNormal;
in vec2 vUV;
in vec3 vWorld;
in vec4 vTint;
out vec4 frag;
uniform vec4  uColor;
uniform float uRoughness;
uniform float uPlanar;
uniform float uRefraction;
uniform vec4  uParams0;
uniform vec4  uParams1;

float wave_scale() { return uParams1.x > 0.0 ? uParams1.x : 1.0; }

// height of the water surface pattern at p: two swells plus scrolling noise
// for chop, each octave faded out once a pixel is too coarse to show it
float water_height(vec2 p, float t, float footprint) {
    float s = uParams1.y > 0.0 ? uParams1.y : 1.0;
    float h = 0.0;
    h += sin(dot(p, vec2(0.62, 0.78)) * 0.9 + t * 1.1 * s) * 0.5;
    h += sin(dot(p, vec2(-0.71, 0.35)) * 1.4 + t * 1.5 * s) * 0.3;
    float fade = 1.0 - smoothstep(0.3, 1.5, footprint * 2.0);
    h += (value_noise(p * 1.7 + vec2(t * 0.35, t * 0.22) * s) - 0.5) * 0.9 * fade;
    h += (value_noise(p * 3.9 - vec2(t * 0.28, -t * 0.4) * s) - 0.5) * 0.45 * fade;
    return h;
}

vec3 water_normal(vec2 p, float t) {
    float footprint = max(fwidth(p.x), fwidth(p.y));
    float e = 0.05;
    float h  = water_height(p, t, footprint);
    float hx = water_height(p + vec2(e, 0.0), t, footprint);
    float hz = water_height(p + vec2(0.0, e), t, footprint);
    float k = (uParams1.w > 0.0 ? uParams1.w : 1.0) * 0.12;
    // calmer toward the horizon, where per-pixel ripples would only shimmer
    k *= 1.0 - smoothstep(0.5, 4.0, footprint * 8.0) * 0.8;
    return normalize(vec3(-(hx - h) / e * k, 1.0, -(hz - h) / e * k));
}

// thickness of water between this pixel and whatever is behind it
float water_thickness(vec2 uv) {
    return max(scene_depth_at(uv) - fragment_depth(), 0.0);
}
)";

// Realistic water: Beer-Lambert absorption toward the deep colour, refraction
// that fades in with depth, Fresnel between the view through and the reflected
// world, a sharp sun glint and foam in the shallows.
static const char* const FSHDER_WATER_SURFACE = R"(#version 440 core
void main() {
    clip_test(vWorld);
    vec2 p = vWorld.xz / wave_scale();
    vec3 n = water_normal(p, uTime);
    vec3 v = normalize(uCameraPos - vWorld);
    vec2 uv = screen_uv();

    float depth_scale = uParams0.w > 0.0 ? uParams0.w : 3.0;
    float thick = water_thickness(uv);
    // refract less where the water is thin, so the shore does not smear
    vec2 offs = (bView * vec4(n, 0.0)).xy * uRefraction * clamp(thick, 0.0, 1.0);
    vec2 ruv = clamp(uv + offs, vec2(0.001), vec2(0.999));
    // never refract in something that is in front of the water
    if (scene_depth_at(ruv) < fragment_depth()) ruv = uv;
    vec3 behind = texture(uRefractMap, ruv).rgb;
    thick = water_thickness(ruv);

    vec3 shallow = srgb_to_linear(uColor.rgb * vTint.rgb);
    vec3 deep = srgb_to_linear(uParams0.rgb);
    float absorb = 1.0 - exp(-thick / depth_scale);
    vec3 lightness = uSunColor * max(uSunDir.y, 0.0) * 0.25 + uSkyColor;   // what lights the body of the water
    vec3 through = mix(behind * shallow, deep * lightness, absorb);

    // reflection only: an albedo of zero leaves the specular and environment terms
    vec3 reflected = shade_surface_ex(vWorld, n, vec3(0.0), 0.0, uRoughness, vec3(0.0), uPlanar);
    float ndv = clamp(dot(n, v), 0.0, 1.0);
    float fresnel = 0.02 + 0.98 * pow(1.0 - ndv, 5.0);
    vec3 color = through * (1.0 - fresnel) + reflected;

    // sun glint, tighter than the BRDF's so it sparkles on the ripples
    vec3 h = normalize(normalize(uSunDir) + v);
    color += uSunColor * pow(max(dot(n, h), 0.0), 600.0) * 4.0 * sun_shadow(vWorld, n, 1.0);

    // foam where the water is shallow, broken up by noise
    float foam_amt = uParams1.z;
    if (foam_amt > 0.0) {
        float edge = 1.0 - smoothstep(0.0, 0.5 * foam_amt, thick);
        float pattern = value_noise(vWorld.xz * 3.0 + uTime * 0.3) * value_noise(vWorld.xz * 7.0 - uTime * 0.2);
        float foam = clamp(edge * smoothstep(0.1, 0.35, pattern + edge * 0.3), 0.0, 1.0);
        color = mix(color, lightness * 1.2 + uSunColor * 0.2, foam * 0.8);
    }
    frag = vec4(apply_fog(color, vWorld), 1.0);
}
)";

// Cartoon water: flat colour bands by depth, a crisp animated foam line at
// every shore and around anything standing in it, drifting foam streaks and a
// cel-shaded sun highlight. Reads the same material fields as the realistic one.
static const char* const FSHDER_WATER_TOON = R"(#version 440 core
void main() {
    clip_test(vWorld);
    vec2 p = vWorld.xz / wave_scale();
    float t = uTime * (uParams1.y > 0.0 ? uParams1.y : 1.0);
    vec2 uv = screen_uv();
    float thick = water_thickness(uv);
    float depth_scale = uParams0.w > 0.0 ? uParams0.w : 2.0;

    // three hard bands, shallow to deep
    float d = clamp(thick / depth_scale, 0.0, 1.0);
    float band = floor(d * 3.0 + 0.5) / 3.0;
    vec3 shallow = srgb_to_linear(uColor.rgb * vTint.rgb);
    vec3 deep = srgb_to_linear(uParams0.rgb);
    vec3 color = mix(shallow, deep, band);

    // a little see-through in the shallowest band
    if (band < 0.1 && uRefraction > 0.0) {
        vec2 wob = vec2(value_noise(p * 2.0 + t), value_noise(p * 2.0 - t)) - 0.5;
        color = mix(texture(uRefractMap, clamp(uv + wob * uRefraction, 0.001, 0.999)).rgb * shallow, color, 0.45);
    }

    // lit flat, stepped: one bright and one dark level
    float lit = max(uSunDir.y, 0.0);
    color *= mix(uSkyColor * 1.5 + 0.35, uSunColor * 0.45 + uSkyColor, step(0.15, lit));

    // foam line: a wobbling threshold on depth, so it ripples along the shore
    float foam_amt = uParams1.z > 0.0 ? uParams1.z : 1.0;
    float wobble = value_noise(p * 4.0 + vec2(t * 0.6, 0.0)) * 0.25;
    float line = step(thick, 0.18 * foam_amt + wobble * foam_amt);
    // drifting streaks on open water
    float streak = step(0.78, value_noise(p * vec2(1.3, 5.0) + vec2(t * 0.25, t * 0.1)));
    color = mix(color, vec3(1.0), max(line, streak * 0.6));

    // cel sun highlight from a wavy normal
    vec3 n = water_normal(p, t);
    vec3 h = normalize(normalize(uSunDir) + normalize(uCameraPos - vWorld));
    color += uSunColor * step(0.985, dot(n, h)) * 0.8;

    if (uPlanar > 0.0 && bReflect.x > 0.5)
        color = mix(color, planar_reflection(vWorld, n, 0.3), uPlanar * 0.35);
    frag = vec4(apply_fog(color, vWorld), 1.0);
}
)";
// ---- depth only, for the shadow pass ----
// uv rides along so cut-out materials (foliage, fences) cast cut-out shadows
static const char* const VSHDER_DEPTH = R"(#version 440 core
layout(location=0) in vec3 aPos;
layout(location=2) in vec2 aUV;
layout(location=3) in mat4 aModel;
uniform mat4 uLightViewProj;
out vec2 vUV;
void main() { vUV = aUV; gl_Position = uLightViewProj * (aModel * vec4(aPos, 1.0)); }
)";

static const char* const VSHDER_DEPTH_SKINNED = R"(#version 440 core
layout(location=0) in vec3 aPos;
layout(location=2) in vec2 aUV;
layout(location=3) in vec4 aBoneIds;
layout(location=4) in vec4 aWeights;
uniform mat4 uLightViewProj;
out vec2 vUV;
void main() {
    SkinInstance si = skin_instance();
    vUV = aUV;
    gl_Position = uLightViewProj * (si.model * (skin_matrix(si, aBoneIds, aWeights) * vec4(aPos, 1.0)));
}
)";
static const char* const FSHDER_DEPTH = R"(#version 440 core
in vec2 vUV;
uniform sampler2D uTex;
uniform float uAlphaCutoff;
void main() {
    if (uAlphaCutoff > 0.0 && texture(uTex, vUV).a < uAlphaCutoff) discard;
}
)";

// ---- fullscreen passes ----
// One triangle larger than the screen from gl_VertexID, no vertex buffer: no
// diagonal seam and every pixel shaded exactly once.
static const char* const VSHDER_POST = R"(#version 440 core
out vec2 vUV;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2) * 2.0 - 1.0;
    vUV = p * 0.5 + 0.5;
    gl_Position = vec4(p, 0.0, 1.0);
}
)";

// the backdrop: each pixel turned back into a world-space ray
static const char* const VSHDER_SKY = R"(#version 440 core
out vec2 vNDC;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2) * 2.0 - 1.0;
    vNDC = p;
    gl_Position = vec4(p, 1.0, 1.0);   // z = w -> sits on the far plane
}
)";

static const char* const FSHDER_SKY = R"(#version 440 core
in vec2 vNDC;
out vec4 frag;
uniform mat4 uInvViewProj;
void main() {
    vec4 far = uInvViewProj * vec4(vNDC, 1.0, 1.0);
    vec3 dir = normalize(far.xyz / far.w - uCameraPos);
    vec3 col = sky_env(dir);
    // settle the lower sky into the fog so the horizon is not a hard seam
    float haze = smoothstep(0.10, -0.05, dir.y) * clamp(uFogDensity * 400.0, 0.0, 1.0);
    frag = vec4(mix(col, uFogColor, haze * 0.85), 1.0);
}
)";

// Bloom: soft-knee threshold with a 4-tap downsample, then separable blur.
static const char* const FSHDER_BLOOM_PREFILTER = R"(#version 440 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uScene;
uniform vec2  uTexel;
uniform float uThreshold;
uniform float uKnee;
void main() {
    vec3 c = texture(uScene, vUV + vec2(-uTexel.x, -uTexel.y)).rgb
           + texture(uScene, vUV + vec2( uTexel.x, -uTexel.y)).rgb
           + texture(uScene, vUV + vec2(-uTexel.x,  uTexel.y)).rgb
           + texture(uScene, vUV + vec2( uTexel.x,  uTexel.y)).rgb;
    c *= 0.25;
    float lum = dot(c, vec3(0.2126, 0.7152, 0.0722));
    float soft = clamp(lum - uThreshold + uKnee, 0.0, 2.0 * uKnee);
    soft = soft * soft / (4.0 * uKnee + 1e-5);
    float w = max(soft, lum - uThreshold) / max(lum, 1e-5);
    frag = vec4(min(c * w, vec3(64.0)), 1.0);   // clamp: one NaN/inf pixel must not flood the screen
}
)";

// five taps standing in for a nine-tap Gaussian via the linear filter
static const char* const FSHDER_BLOOM_BLUR = R"(#version 440 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uSource;
uniform vec2 uDirection;
void main() {
    vec3 c = texture(uSource, vUV).rgb * 0.2270270270;
    c += (texture(uSource, vUV + uDirection * 1.3846153846).rgb
        + texture(uSource, vUV - uDirection * 1.3846153846).rgb) * 0.3162162162;
    c += (texture(uSource, vUV + uDirection * 3.2307692308).rgb
        + texture(uSource, vUV - uDirection * 3.2307692308).rgb) * 0.0702702703;
    frag = vec4(c, 1.0);
}
)";

// HDR -> display: bloom, exposure, ACES, grade, vignette, sRGB encode. Writes
// luma to alpha for the FXAA pass that may follow.
static const char* const FSHDER_POST = R"(#version 440 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uScene;
uniform sampler2D uBloom;
uniform vec2  uTexel;
uniform float uExposure;
uniform float uSaturation;
uniform float uContrast;
uniform float uVignette;
uniform float uSharpen;
uniform float uBloomStrength;
uniform int   uTonemap;       // 0 ACES, 1 Reinhard, 2 none
uniform vec3  uColorFilter;
uniform float uDither;

vec3 tonemap(vec3 x) {
    if (uTonemap == 1) return x / (1.0 + x);                  // Reinhard
    if (uTonemap == 2) return clamp(x, 0.0, 1.0);
    // Narkowicz's ACES fit: rolls highlights off instead of clipping them
    const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

void main() {
    vec3 color = texture(uScene, vUV).rgb;
    if (uSharpen > 0.0) {
        vec3 blur = texture(uScene, vUV + vec2( uTexel.x, 0.0)).rgb
                  + texture(uScene, vUV + vec2(-uTexel.x, 0.0)).rgb
                  + texture(uScene, vUV + vec2(0.0,  uTexel.y)).rgb
                  + texture(uScene, vUV + vec2(0.0, -uTexel.y)).rgb;
        color = max(color + (color - blur * 0.25) * uSharpen, vec3(0.0));
    }
    if (uBloomStrength > 0.0) color += texture(uBloom, vUV).rgb * uBloomStrength;
    color = tonemap(color * uExposure);
    color = mix(color, color * color * (3.0 - 2.0 * color), uContrast);   // midtone S-curve
    float luma = dot(color, vec3(0.2126, 0.7152, 0.0722));
    color = mix(vec3(luma), color, uSaturation);
    vec2 d = vUV - 0.5;
    color *= 1.0 - uVignette * smoothstep(0.16, 0.62, dot(d, d));
    color *= uColorFilter;
    color = linear_to_srgb(clamp(color, 0.0, 1.0));
    // half an 8-bit step of noise: invisible, but it stops smooth gradients banding
    float n = fract(sin(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 43758.5453);
    color += (n - 0.5) / 255.0 * uDither;
    frag = vec4(color, 1.0);
}
)";

// FXAA 3.11-style (Lottes), quality preset simplified: edge detect on luma,
// blend along the edge.
static const char* const FSHDER_FXAA = R"(#version 440 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uSource;
uniform vec2 uTexel;
float luma(vec3 c) { return dot(c, vec3(0.299, 0.587, 0.114)); }
void main() {
    vec3 rgbM = texture(uSource, vUV).rgb;
    float lM  = luma(rgbM);
    float lNW = luma(textureOffset(uSource, vUV, ivec2(-1, -1)).rgb);
    float lNE = luma(textureOffset(uSource, vUV, ivec2( 1, -1)).rgb);
    float lSW = luma(textureOffset(uSource, vUV, ivec2(-1,  1)).rgb);
    float lSE = luma(textureOffset(uSource, vUV, ivec2( 1,  1)).rgb);
    float lMin = min(lM, min(min(lNW, lNE), min(lSW, lSE)));
    float lMax = max(lM, max(max(lNW, lNE), max(lSW, lSE)));
    if (lMax - lMin < max(0.0312, lMax * 0.125)) { frag = vec4(rgbM, 1.0); return; }

    vec2 dir = vec2(-((lNW + lNE) - (lSW + lSE)), ((lNW + lSW) - (lNE + lSE)));
    float reduce = max((lNW + lNE + lSW + lSE) * 0.25 * 0.125, 1.0 / 128.0);
    float rcp = 1.0 / (min(abs(dir.x), abs(dir.y)) + reduce);
    dir = clamp(dir * rcp, vec2(-8.0), vec2(8.0)) * uTexel;

    vec3 a = 0.5 * (texture(uSource, vUV + dir * (1.0 / 3.0 - 0.5)).rgb
                  + texture(uSource, vUV + dir * (2.0 / 3.0 - 0.5)).rgb);
    vec3 b = a * 0.5 + 0.25 * (texture(uSource, vUV + dir * -0.5).rgb
                             + texture(uSource, vUV + dir *  0.5).rgb);
    float lB = dot(b, vec3(0.299, 0.587, 0.114));
    frag = vec4((lB < lMin || lB > lMax) ? a : b, 1.0);
}
)";

// ---- particles ----
// One camera-facing quad per particle from gl_VertexID; colour is linear
// radiance and deliberately unclamped, so a flash at 14.0 blows through the
// bloom threshold and glares while smoke at 0.05 does not.
static const char* const VSHDER_PARTICLE = R"(#version 440 core
layout(location=0) in vec4 aCentreSize;   // xyz world centre, w half-size
layout(location=1) in vec4 aColor;        // rgb linear radiance, a coverage
uniform vec3 uCamRight;
uniform vec3 uCamUp;
out vec2 vUV;
out vec4 vColor;
void main() {
    vec2 corner = vec2((gl_VertexID & 1) == 0 ? -1.0 : 1.0, (gl_VertexID & 2) == 0 ? -1.0 : 1.0);
    vUV = corner;
    vColor = aColor;
    vec3 world = aCentreSize.xyz + (uCamRight * corner.x + uCamUp * corner.y) * aCentreSize.w;
    gl_Position = uViewProj * vec4(world, 1.0);
}
)";

static const char* const FSHDER_PARTICLE = R"(#version 440 core
in vec2 vUV;
in vec4 vColor;
out vec4 frag;
void main() {
    float r2 = dot(vUV, vUV);
    if (r2 > 1.0) discard;
    float falloff = 1.0 - r2;
    frag = vec4(vColor.rgb, vColor.a * falloff * falloff);
}
)";

// ---- debug lines ----
static const char* const VSHDER_LINE = R"(#version 440 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec4 aColor;
out vec4 vColor;
void main() { vColor = aColor; gl_Position = uViewProj * vec4(aPos, 1.0); }
)";

static const char* const FSHDER_LINE = R"(#version 440 core
in vec4 vColor;
out vec4 frag;
void main() { frag = vec4(srgb_to_linear(vColor.rgb), vColor.a); }
)";

// ---- retained sprites and text (sprite.hpp / text.hpp) ----
static const char* const VSHDER_SPRITE = R"(#version 440 core
layout(location=0) in vec2 aCorner;   // unit quad, 0..1
layout(location=1) in vec4 aBox;      // x, y, w, h
layout(location=2) in vec4 aCrop;     // atlas pixel rect x, y, w, h
layout(location=3) in uint aTexIndex; // index into uTex/uTexSize
uniform mat4 uViewProj;
uniform vec2 uTexSize[32];
out vec2 vUV;
flat out uint vTexIndex;
void main() {
    vec2 pos = aBox.xy + aCorner * aBox.zw;
    gl_Position = uViewProj * vec4(pos, 0.0, 1.0);
    vUV = (aCrop.xy + aCorner * aCrop.zw) / uTexSize[aTexIndex];
    vTexIndex = aTexIndex;
}
)";

static const char* const FSHDER_SPRITE = R"(#version 440 core
in vec2 vUV;
flat in uint vTexIndex;
out vec4 frag;
uniform sampler2D uTex[32];
void main() { frag = texture(uTex[vTexIndex], vUV); }
)";

// uItalic shears each glyph quad in place: the top edge shifts, the baseline stays
static const char* const VSHDER_TEXT = R"(#version 440 core
layout(location=0) in vec2 aCorner;
layout(location=1) in vec4 aBox;
layout(location=2) in vec4 aCrop;
layout(location=3) in uint aTexIndex;
uniform mat4 uViewProj;
uniform vec2 uTexSize[32];
uniform float uItalic;
out vec2 vUV;
flat out uint vTexIndex;
void main() {
    vec2 local = aCorner * aBox.zw;
    local.x += uItalic * (aBox.w - local.y);
    gl_Position = uViewProj * vec4(aBox.xy + local, 0.0, 1.0);
    vUV = (aCrop.xy + aCorner * aCrop.zw) / uTexSize[aTexIndex];
    vTexIndex = aTexIndex;
}
)";

// SDF text: fwidth() keeps the edge one pixel wide at any scale; uWeight
// biases the edge for bold/thin, uOutlineWidth carves a second band below it.
static const char* const FSHDER_TEXT = R"(#version 440 core
in vec2 vUV;
flat in uint vTexIndex;
out vec4 frag;
uniform sampler2D uTex[32];
uniform vec4 uColor;
uniform vec4 uOutlineColor;
uniform float uWeight;
uniform float uOutlineWidth;
void main() {
    float sdf = texture(uTex[vTexIndex], vUV).r;
    float w = max(fwidth(sdf), 1e-4);
    float fillEdge = 0.5 - uWeight;
    float outlineEdge = fillEdge - uOutlineWidth;
    float fillAlpha = smoothstep(fillEdge - w, fillEdge + w, sdf);
    float outlineAlpha = smoothstep(outlineEdge - w, outlineEdge + w, sdf);
    float t = outlineAlpha > 1e-4 ? clamp(fillAlpha / outlineAlpha, 0.0, 1.0) : 0.0;
    vec3 rgb = mix(uOutlineColor.rgb, uColor.rgb, t);
    float alpha = outlineAlpha * mix(uOutlineColor.a, uColor.a, t);
    frag = vec4(rgb, alpha);
}
)";

// ---- ready-made post effects, for pix_add_post_effect ----
//
// Each is just a main(): the effect prelude supplies uScene, uDepth,
// linear_depth(), uTexel, uParams, vUV and frag. They double as examples of
// writing your own. The comment on each says what its uParams mean.

// x: amount 0..1
static const char* const PIX_EFFECT_GRAYSCALE = R"(
void main() {
    vec3 c = texture(uScene, vUV).rgb;
    frag = vec4(mix(c, vec3(dot(c, vec3(0.299, 0.587, 0.114))), uParams.x), 1.0);
}
)";

// x: amount 0..1
static const char* const PIX_EFFECT_SEPIA = R"(
void main() {
    vec3 c = texture(uScene, vUV).rgb;
    vec3 s = vec3(dot(c, vec3(0.393, 0.769, 0.189)), dot(c, vec3(0.349, 0.686, 0.168)),
                  dot(c, vec3(0.272, 0.534, 0.131)));
    frag = vec4(mix(c, min(s, 1.0), uParams.x), 1.0);
}
)";

// x: block size in pixels (e.g. 4)
static const char* const PIX_EFFECT_PIXELATE = R"(
void main() {
    vec2 block = uTexel * max(uParams.x, 1.0);
    frag = vec4(texture(uScene, (floor(vUV / block) + 0.5) * block).rgb, 1.0);
}
)";

// x: offset in pixels at the screen edge (e.g. 3)
static const char* const PIX_EFFECT_CHROMATIC = R"(
void main() {
    vec2 d = (vUV - 0.5) * uTexel * uParams.x * 2.0;
    frag = vec4(texture(uScene, vUV + d).r, texture(uScene, vUV).g, texture(uScene, vUV - d).b, 1.0);
}
)";

// x: scanline strength 0..1, y: screen curvature (e.g. 0.1)
static const char* const PIX_EFFECT_CRT = R"(
void main() {
    vec2 uv = vUV * 2.0 - 1.0;
    uv *= 1.0 + uParams.y * dot(uv.yx, uv.yx);
    uv = uv * 0.5 + 0.5;
    if (uv.x < 0.0 || uv.y < 0.0 || uv.x > 1.0 || uv.y > 1.0) { frag = vec4(0.0, 0.0, 0.0, 1.0); return; }
    vec3 c = texture(uScene, uv).rgb;
    float line = 0.5 + 0.5 * sin(uv.y / uTexel.y * 3.14159);
    c *= mix(1.0, 0.65 + 0.35 * line, uParams.x);
    frag = vec4(c * 1.1, 1.0);
}
)";

// Toon ink from depth discontinuities. x: thickness in pixels, y: sensitivity
// (e.g. 0.05 = 5% depth jump), z: 0..1 darkness
static const char* const PIX_EFFECT_OUTLINE = R"(
void main() {
    vec2 o = uTexel * max(uParams.x, 1.0);
    float d = linear_depth(vUV);
    float e = 0.0;
    e = max(e, abs(linear_depth(vUV + vec2(o.x, 0.0)) - d));
    e = max(e, abs(linear_depth(vUV - vec2(o.x, 0.0)) - d));
    e = max(e, abs(linear_depth(vUV + vec2(0.0, o.y)) - d));
    e = max(e, abs(linear_depth(vUV - vec2(0.0, o.y)) - d));
    float edge = smoothstep(0.0, 1.0, e / (d * max(uParams.y, 1e-4)));
    vec3 c = texture(uScene, vUV).rgb;
    frag = vec4(c * (1.0 - edge * uParams.z), 1.0);
}
)";

// Depth of field lite: blurs what is far from a focus distance.
// x: focus distance (world units), y: focus range, z: max blur in pixels
static const char* const PIX_EFFECT_DEPTH_BLUR = R"(
void main() {
    float d = linear_depth(vUV);
    float amount = clamp(abs(d - uParams.x) / max(uParams.y, 1e-3), 0.0, 1.0) * uParams.z;
    vec3 c = texture(uScene, vUV).rgb;
    if (amount > 0.5) {
        vec3 sum = c;
        const vec2 taps[8] = vec2[8](vec2(1,0), vec2(-1,0), vec2(0,1), vec2(0,-1),
                                     vec2(0.7,0.7), vec2(-0.7,0.7), vec2(0.7,-0.7), vec2(-0.7,-0.7));
        for (int i = 0; i < 8; i++) sum += texture(uScene, vUV + taps[i] * uTexel * amount).rgb;
        c = sum / 9.0;
    }
    frag = vec4(c, 1.0);
}
)";

// ---- splicing ----
//
// Inserts the named GLSL chunks right after a shader's #version line. The
// result is malloc'd; free it with pix_free once compiled.
PIXDEF char* pix_shader_splice(const char* src, const char* const* chunks, int chunk_count);
// The full lighting library (scene block, sky, BRDF, lights, shadows,
// shade_surface) spliced into `src`.
PIXDEF char* shader_with_pbr(const char* src);
// Just the scene block + sky/colour helpers.
PIXDEF char* shader_with_common(const char* src);

#endif // PIX_SHADER_SOURCES_HPP

// ============================================================================
#if defined(PIX_IMPLEMENTATION) && !defined(PIX_SHADER_SOURCES_IMPL)
#define PIX_SHADER_SOURCES_IMPL

char* pix_shader_splice(const char* src, const char* const* chunks, int chunk_count) {
    const char* nl = strchr(src, '\n');
    size_t head = nl ? (size_t)(nl - src) + 1 : 0;
    size_t total = strlen(src) + 1;
    for (int i = 0; i < chunk_count; i++) total += strlen(chunks[i]) + 1;
    char* out = pix_new<char>(total);
    if (!out) return 0;
    memcpy(out, src, head);
    size_t n = head;
    for (int i = 0; i < chunk_count; i++) {
        size_t len = strlen(chunks[i]);
        memcpy(out + n, chunks[i], len);
        n += len;
        out[n++] = '\n';
    }
    strcpy(out + n, src + head);
    return out;
}

char* shader_with_pbr(const char* src) {
    const char* chunks[] = { GLSL_SCENE, GLSL_COMMON, GLSL_NOISE, GLSL_BRDF, GLSL_LIGHTS, GLSL_SHADOW, GLSL_SURFACE };
    return pix_shader_splice(src, chunks, 7);
}

char* shader_with_common(const char* src) {
    const char* chunks[] = { GLSL_SCENE, GLSL_COMMON };
    return pix_shader_splice(src, chunks, 2);
}

#endif // PIX_SHADER_SOURCES_IMPL
