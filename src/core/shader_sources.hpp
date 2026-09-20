#pragma once
#include <string.h>
#include <stdio.h>

// instanced: per-vertex pos/normal/uv (loc 0..2), per-instance model matrix (loc 3..6)
const char* VSHDER_BASIC = R"(#version 440 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aUV;
layout(location=3) in mat4 aModel;   // consumes 3..6
uniform mat4 uViewProj;
out vec3 vNormal;
out vec2 vUV;
out vec3 vWorld;
void main() {
    vec4 world = aModel * vec4(aPos, 1.0);
    vWorld = world.xyz;
    gl_Position = uViewProj * world;
    vNormal = mat3(aModel) * aNormal;
    vUV = vec2(aUV.x, 1.0 - aUV.y); // OBJ/Blender V origin is bottom, GL texture V is top
}
)";

// Shared GLSL: an analytic daytime sky and the sRGB<->linear helpers. Prepended
// to every shader that needs to sample the environment (surface reflections,
// water reflections, the sky pass itself) so there is exactly one sky in the
// scene and reflections always agree with the backdrop.
//
// sky_env returns *linear* radiance for a view direction: a horizon-to-zenith
// gradient, a ground half below the horizon, and the sun as a sharp disc with
// two softer glow lobes around it. Everything downstream tonemaps and gamma
// encodes in the post pass, so nothing here clamps.
// ---- the per-frame block ----
//
// Everything about the frame that every shader agrees on - where the sun is,
// what the sky is doing, where the camera stands, how the shadow cascades are
// laid out - in one std140 uniform buffer, uploaded once and bound for the
// whole frame.
//
// Before this it was thirty-odd glUniform calls *per program per pass*, each
// one preceded by a glGetUniformLocation, which is a string lookup inside the
// driver: the surface shader, the skinned shader, the water and the sky each
// re-sent the same sun direction several times a frame, and the shadow and
// reflection passes sent it again. None of it was per-draw data and none of it
// changed between those calls.
//
// Everything in here is a vec4 or a mat4 on purpose. std140 pads a vec3 out to
// sixteen bytes anyway and aligns arrays to sixteen, so packing by hand into
// vec4s is not a saving - it is how the C side and the GLSL side are kept
// obviously identical, which is the only real hazard with a uniform block.
// The names the shaders already used are then macros onto the members, so the
// bodies below read exactly as they did.
const char* GLSL_SCENE = R"(
layout(std140, binding = 0) uniform SceneBlock {
    vec4 bSunDir;         // xyz normalised, toward the sun
    vec4 bSunColor;       // rgb linear radiance
    vec4 bSkyColor;       // rgb hemisphere irradiance from above
    vec4 bGroundColor;    // rgb hemisphere irradiance bounced from below
    vec4 bSkyZenith;      // rgb dome colour straight up
    vec4 bSkyHorizon;     // rgb dome colour at the horizon
    vec4 bFog;            // rgb colour, w density
    vec4 bCamera;         // xyz eye, w wetness
    vec4 bShadow;         // xy one atlas texel, z strength
    mat4 bLightSpace[3];  // one projection per cascade, near to far
    vec4 bCascadeFar;     // xyz view distance each cascade ends at
    vec4 bCascadeTexel;   // xyz one shadow texel in world metres
};

#define uSunDir         bSunDir.xyz
#define uSunColor       bSunColor.rgb
#define uSkyColor       bSkyColor.rgb
#define uGroundColor    bGroundColor.rgb
#define uSkyZenith      bSkyZenith.rgb
#define uSkyHorizon     bSkyHorizon.rgb
#define uFogColor       bFog.rgb
#define uFogDensity     bFog.w
#define uCameraPos      bCamera.xyz
#define uWetness        bCamera.w
#define uShadowTexel    bShadow.xy
#define uShadowStrength bShadow.z
#define uLightSpace     bLightSpace
#define uCascadeFar     bCascadeFar
#define uCascadeTexel   bCascadeTexel
)";

const char* GLSL_COMMON = R"(
vec3 srgb_to_linear(vec3 c) { return pow(max(c, 0.0), vec3(2.2)); }
vec3 linear_to_srgb(vec3 c) { return pow(max(c, 0.0), vec3(1.0 / 2.2)); }

vec3 sky_env(vec3 dir, vec3 sunDir, vec3 sunColor, vec3 zenith, vec3 horizon, vec3 ground) {
    float up = dir.y;
    vec3 sky;
    if (up >= 0.0) {
        float t = pow(1.0 - up, 5.0);                 // haze bunched near the horizon
        sky = mix(zenith, horizon, t);
    } else {
        sky = mix(horizon, ground, clamp(-up * 3.0, 0.0, 1.0));
    }
    float sd = max(dot(dir, normalize(sunDir)), 0.0);
    sky += sunColor * pow(sd, 1800.0) * 16.0;         // disc
    sky += sunColor * pow(sd, 28.0)   * 0.30;         // tight glow
    sky += sunColor * pow(sd, 5.0)    * 0.06;         // wide forward scatter
    return sky;
}
)";

// Splices GLSL_COMMON in immediately after a shader's #version line, so every
// fragment shader that samples the sky (surfaces, water, the sky pass) shares
// one definition. Returns a pointer into a static buffer - compile it before
// calling again.
static const char* shader_with_common(const char* src) {
    static char buf[24576];
    const char* nl = strchr(src, '\n');
    if (!nl) return src;
    size_t head = (size_t)(nl - src) + 1;
    memcpy(buf, src, head);
    int n = (int)head;
    n += sprintf(buf + n, "%s\n%s\n", GLSL_SCENE, GLSL_COMMON);
    sprintf(buf + n, "%s", src + head);
    return buf;
}

// ---- shared BRDF ----
//
// Split out of the surface shader so the skinned pass, the water and anything
// added later evaluate exactly the same material model. The pieces are the
// standard ones and are named after the papers they come from, because the
// point of a physically based renderer is that a surface behaves the way the
// published model says it does rather than the way it was tuned to.
const char* GLSL_BRDF = R"(
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
// Karis' analytic environment BRDF fit (the "mobile" split-sum approximation),
// so the ambient specular carries the right Fresnel and roughness weight
// without a precomputed lookup table
vec3 env_brdf(vec3 f0, float rough, float ndv) {
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572,  0.022);
    const vec4 c1 = vec4( 1.0,  0.0425,  1.04,  -0.04);
    vec4 r = rough * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * ndv)) * r.x + r.y;
    vec2 ab = vec2(-1.04, 1.04) * a004 + r.zw;
    return f0 * ab.x + ab.y;
}

// One evaluation of the full material against one light direction. `radiance`
// is what arrives at the surface; everything above it is geometry.
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

// The direction a rough surface's specular lobe actually points. A perfect
// mirror reflects along reflect(-v, n); as roughness climbs the lobe leans back
// toward the normal, and using the mirror direction anyway is what makes rough
// metal look like a warped mirror instead of like brushed metal.
// (Frostbite's getSpecularDominantDir, Lagarde & de Rousiers 2014.)
vec3 specular_dominant_dir(vec3 n, vec3 r, float rough) {
    float smoothness = clamp(1.0 - rough, 0.0, 1.0);
    float lerp_factor = smoothness * (sqrt(smoothness) + rough);
    return normalize(mix(n, r, lerp_factor));
}

// Windowed inverse-square falloff (Karis 2013). Physically an inverse square
// never reaches zero, which would mean every light in the world touches every
// pixel; the window pulls it to exactly zero at `radius` so a light can be
// culled at a known distance without a visible edge where it stops.
float light_falloff(float dist2, float radius) {
    float r2 = radius * radius;
    float f = clamp(1.0 - (dist2 * dist2) / max(r2 * r2, 1e-6), 0.0, 1.0);
    return (f * f) / max(dist2, 0.01);
}
)";

// ---- punctual lights ----
//
// One uniform block shared by the surface and skinned shaders. Lights arrive
// already reduced to the ones nearest the camera and packed into three vec4
// arrays, so binding a frame's lighting is three uploads and no per-light work
// in the draw loop.
const char* GLSL_LIGHTS = R"(
// The frame's punctual lights, in a block of their own rather than in the
// scene block above: this one is rewritten every frame after the light list is
// packed, and the two have no reason to share an upload.
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
            float t = clamp((cd - cos_outer)
                            / max(uLightColorInner[i].w - cos_outer, 1e-4), 0.0, 1.0);
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
// One directional light over a whole world cannot be served by one shadow map:
// a single 2048 map stretched over the view distance gives texels metres across,
// and stretched over the near field it stops casting anything past the first few
// metres. So the view is split by distance into SHADOW_CASCADES slices, each
// gets its own tightly fitted map, and a fragment picks the nearest slice it
// falls inside.
//
// The three maps live side by side in one texture so the lookup needs one
// sampler and one bind. Two details do most of the work in making the result
// look solid rather than crawling:
//
//   normal offset  the sample position is pushed along the surface normal by
//                  about a texel's worth of world space before it is projected.
//                  A depth-only bias has to grow with the slope until it
//                  detaches the shadow from whatever cast it; moving sideways
//                  along the surface instead does not.
//   cascade blend  the last stretch of each slice cross-fades into the next, so
//                  the resolution change is a soft band rather than a visible
//                  line drawn across the ground.
const char* GLSL_SHADOW = R"(
// The cascade projections, their ranges and the atlas texel size all live in
// the scene block (GLSL_SCENE); the sampler is the one thing that cannot.
uniform sampler2DShadow uShadowMap;

float shadow_cascade(int c, vec3 world, vec3 n, float ndl) {
    // normal offset, scaled by how oblique the light is to this surface
    float slope = clamp(1.0 - ndl, 0.0, 1.0);
    vec3 p_world = world + n * (uCascadeTexel[c] * (1.0 + 2.0 * slope) * 1.4);

    vec4 lp = uLightSpace[c] * vec4(p_world, 1.0);
    vec3 p = lp.xyz / lp.w * 0.5 + 0.5;
    if (p.z > 1.0 || p.z < 0.0) return 1.0;

    // the cascade's own tile inside the atlas, inset so a filter tap can never
    // read across into its neighbour
    float inset = uShadowTexel.x * 2.0;
    p.x = clamp(p.x, inset, 1.0 - inset) / 3.0 + float(c) / 3.0;
    p.y = clamp(p.y, uShadowTexel.y * 2.0, 1.0 - uShadowTexel.y * 2.0);

    float bias = 0.0006;
    float sum = 0.0;
    for (int y = -1; y <= 1; ++y)
        for (int x = -1; x <= 1; ++x)
            sum += texture(uShadowMap, vec3(p.xy + vec2(float(x) * uShadowTexel.x,
                                                        float(y) * uShadowTexel.y),
                                            p.z - bias));
    return sum * (1.0 / 9.0);
}

float sun_shadow(vec3 world, vec3 n, float ndl, float view_depth) {
    if (uShadowStrength <= 0.0) return 1.0;

    int c = 2;
    if (view_depth < uCascadeFar.x)      c = 0;
    else if (view_depth < uCascadeFar.y) c = 1;

    float s = shadow_cascade(c, world, n, ndl);

    // cross-fade the last tenth of a cascade into the next one
    if (c < 2) {
        float far_edge = (c == 0) ? uCascadeFar.x : uCascadeFar.y;
        float band = far_edge * 0.12;
        float t = clamp((view_depth - (far_edge - band)) / band, 0.0, 1.0);
        if (t > 0.0) s = mix(s, shadow_cascade(c + 1, world, n, ndl), t);
    }
    return mix(1.0, s, uShadowStrength);
}
)";

// Splices the sky, the BRDF, the light loop and the shadow lookup into a
// shader in that order - each depends on the one before it.
static const char* shader_with_pbr(const char* src) {
    static char buf[3][32768];
    static int  slot = 0;
    char* out = buf[slot];
    slot = (slot + 1) % 3;

    const char* nl = strchr(src, '\n');
    if (!nl) return src;
    size_t head = (size_t)(nl - src) + 1;
    memcpy(out, src, head);
    int n = (int)head;
    n += sprintf(out + n, "%s\n%s\n%s\n%s\n%s\n",
                 GLSL_SCENE, GLSL_COMMON, GLSL_BRDF, GLSL_LIGHTS, GLSL_SHADOW);
    sprintf(out + n, "%s", src + head);
    return out;
}

// The surface shader every solid thing in the world is drawn with. Metallic /
// roughness PBR: a Cook-Torrance specular lobe over a Lambertian diffuse, lit by
//
//   * one shadowed directional sun, through the cascades above
//   * an image based ambient that samples the same analytic sky the backdrop
//     uses, along the roughness-corrected dominant reflection direction - so a
//     car roof mirrors the real sky gradient and a brick wall picks up only its
//     broad tint
//   * every punctual light near enough to matter, through the same BRDF
//
// Albedo and tint arrive in sRGB and are linearised on read; all lighting is
// done in linear space and the post pass owns the tonemap and the single gamma
// encode.
const char* FSHDER_BASIC = R"(#version 440 core
in vec3 vNormal;
in vec2 vUV;
in vec3 vWorld;
out vec4 frag;

uniform sampler2D uTex;
uniform vec3  uColor;
uniform float uMetallic;
uniform float uRoughness;
uniform vec3  uEmissive;      // linear radiance added straight to the result
uniform vec3  uWindowGlow;    // the same, but only out of the texture's dark patches
uniform float uGlass;         // how far the texture's dark patches go toward glass

// The sun, the sky, the fog and the camera all arrive in the scene block.

// Used only by the water's planar reflection pass (see pix__reflection_pass):
// a mirrored camera renders this same shader again, and anything below the
// water's surface has to be discarded there - it is invisible to the real
// camera, and left in it would show up as a mirrored island hanging in the
// reflected sky. uClipSign is 0 for every ordinary draw, which multiplies the
// test away entirely.
uniform float uClipHeight;
uniform float uClipSign;


// Value noise over world XZ, for breaking the water film into puddles. Ground
// that is uniformly wet edge to edge is ground that has been given a different
// material, not ground it has rained on: what says "rain" is that some of it is
// holding water and some of it has already drained.
float wet_hash(vec2 p) {
    return fract(sin(dot(p, vec2(41.7, 289.1))) * 43758.5453);
}
float wet_noise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float a = wet_hash(i), b = wet_hash(i + vec2(1.0, 0.0));
    float c = wet_hash(i + vec2(0.0, 1.0)), d = wet_hash(i + vec2(1.0, 1.0));
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

void main() {
    if (uClipSign > 0.5 && vWorld.y < uClipHeight) discard;
    vec3 n = normalize(vNormal);
    vec3 v = normalize(uCameraPos - vWorld);
    float ndv = max(dot(n, v), 1e-4);

    // Cut-out foliage.
    //
    // Foliage, fences, grates and anything else billboarded onto a quad are
    // alpha-masked cards: a texture with everything that is not the subject
    // left transparent. Sampling only .rgb draws those transparent regions as
    // opaque black, which turns a bush into a ball of black and green shards.
    // A plain cut-out test costs nothing on an opaque texture, whose alpha is
    // one everywhere, and is the whole fix for the cards.
    vec4  tex = texture(uTex, vUV);
    if (tex.a < 0.5) discard;

    vec3  albedo = srgb_to_linear(tex.rgb * uColor);
    float rough  = clamp(uRoughness, 0.045, 1.0);
    float metal  = clamp(uMetallic, 0.0, 1.0);

    // ---- wet ground ----
    //
    // A film of water does two opposite things at once, and wet ground only
    // reads as wet when both are there. It fills the surface roughness in, so
    // what was a diffuse sheet becomes close to a mirror - that is the
    // reflection of the sky and of whatever is standing on it, stretching away
    // from the viewer. And it traps light by internal reflection, so the
    // surface underneath that mirror goes much darker than it was dry.
    //
    // Only faces the rain can land on, hence the n.y weight: a wall stays dry
    // while the ground it meets shines.
    // ---- glazing ----
    //
    // The windows are painted into the same texture as the wall, so there is
    // no second material to give a different finish to and the only thing to
    // go on is what the texel looks like. This finds the panes two ways,
    // because this art set paints glass two ways.
    //
    // The first is darkness. What a camera sees looking into a window in
    // daylight is mostly a dim room, so a swatch far darker than the wall
    // around it is glazing. That test was cubed, which made it so selective
    // that only the very darkest core of a window counted and the frame of
    // reflective glass around it did not - squared, and with the luminance
    // knee opened up, a whole pane qualifies instead of the middle of one.
    //
    // The second is the one that was missing entirely, and it is the reason
    // the bakery had no reflective glass at all: its shopfront is not painted
    // dark, it is painted as a pale blue-white pane with the sky already in
    // it. That is the other convention this art set uses for glass, and it is
    // exactly what the darkness test cannot see. A light, blue-leaning swatch
    // is glazing too - both gates held tight, because a merely blue-grey wall
    // (which half the downtown kit is) must not come out as a mirror: the
    // luminance gate is what separates a pale pane from a painted wall, and
    // the blue gate is what separates it from white plaster.
    float luma = dot(albedo, vec3(0.2126, 0.7152, 0.0722));
    float dark = 1.0 - clamp(luma * 4.6, 0.0, 1.0);
    dark = dark * dark;
    float blue = albedo.b - max(albedo.r, albedo.g);
    float pale = smoothstep(0.03, 0.09, blue) * smoothstep(0.45, 0.65, luma);
    // whichever cue found it; the same term drives the lit-window glow further
    // down, which is why it is computed once, here, rather than twice
    float pane = max(dark, pale);
    float glass = clamp(uGlass, 0.0, 1.0) * pane;

    // What a pane actually does differently from the wall it sits in.
    //
    // Sharper: glass is usually the smoothest thing in a scene, and the mirror
    // image in it survives to a distance nothing else in the frame manages.
    //
    // Darker underneath: glass transmits rather than scatters, so almost
    // nothing comes back off it diffusely. Leaving the diffuse term in was
    // most of why a window read as a shaded wall - the reflection was there
    // and it was sitting on top of a lit surface, which is not what looking at
    // glass is like.
    //
    // Stronger: 0.04 is a bare dielectric and glass is nearer 0.08, and the
    // Fresnel rise off that is steep enough that a facade seen at a glancing
    // angle goes to a near-mirror at its far end while the pane square in
    // front of you stays a dim room. That gradient along a row of windows is
    // the whole difference between glazing and grey paint.
    rough = mix(rough, 0.028, glass);

    float wet = uWetness * clamp(n.y * 2.2 - 0.5, 0.0, 1.0);
    // two scales of puddle: broad wet patches, and smaller pools inside them
    float pud = wet_noise(vWorld.xz * 0.11) * 0.65 + wet_noise(vWorld.xz * 0.37) * 0.35;
    float sheen = wet * mix(0.5, 1.0, smoothstep(0.3, 0.75, pud));
    albedo *= mix(1.0, 0.32, sheen);
    rough   = mix(rough, 0.035, sheen);

    float a      = rough * rough;

    // A water film is its own dielectric surface sitting on top of whatever is
    // underneath, so its reflectance is water at normal incidence rather than
    // the surface it covers - and, far more importantly, it is smooth enough
    // for the Fresnel rise toward grazing angles to actually survive, which is
    // the whole reason wet ground mirrors a skyline down its length.
    vec3 f0 = mix(vec3(0.04), albedo, metal);
    f0 = mix(f0, vec3(0.05), sheen);
    // glass is a stronger reflector than the plaster around it
    f0 = mix(f0, vec3(0.08), glass);
    // and it transmits rather than scatters - see the glazing block above
    vec3 diff_albedo = albedo * (1.0 - metal) * (1.0 - glass * 0.80);

    // ---- direct sun ----
    vec3  l = normalize(uSunDir);
    float ndl = max(dot(n, l), 0.0);
    float view_depth = length(uCameraPos - vWorld);
    float shadow = sun_shadow(vWorld, n, ndl, view_depth);
    vec3  direct = surface_brdf(n, v, l, diff_albedo, f0, a, uSunColor) * shadow;

    // ---- punctual lights ----
    direct += punctual_lights(vWorld, n, v, diff_albedo, f0, a);

    // ---- ambient, off the analytic sky ----
    float hemi = n.y * 0.5 + 0.5;
    vec3  amb_diffuse = mix(uGroundColor, uSkyColor, hemi) * diff_albedo;

    vec3 refl = specular_dominant_dir(n, reflect(-v, n), rough);
    vec3 mirror = sky_env(refl, uSunDir, uSunColor, uSkyZenith, uSkyHorizon, uGroundColor);
    vec3 flat_dome = mix(uSkyHorizon, uSkyZenith, clamp(refl.y * 0.5 + 0.5, 0.0, 1.0));
    // the mirror image survives longer on a smooth surface than a straight lerp
    // on roughness would allow, which is what keeps car paint glossy
    vec3 env = mix(mirror, flat_dome, rough * rough);

    // a reflection that has bent below the surface it came off cannot be seen
    // (Frostbite's horizon occlusion)
    float horizon = clamp(1.0 + dot(reflect(-v, n), n), 0.0, 1.0);
    vec3 amb_spec = env * env_brdf(f0, rough, ndv) * (horizon * horizon);
    // Standing in for a reflection this pass cannot give an arbitrary surface.
    //
    // A real film of water reflects the whole scene, and the only environment
    // a surface can sample here is the analytic sky. Under the overcast that
    // comes with rain that sky is a flat grey dome, so a physically weighted
    // mirror of it is indistinguishable from the dry surface underneath.
    //
    // The weighting is pushed onto the grazing angles rather than spread flat,
    // because that is where wet ground actually reads as wet: the surface at
    // your feet is dark and almost matte, and the mirror builds up with
    // distance until the far ground is a sheet of reflected sky. Brightening
    // every angle equally just turns it pale, which is dry, not wet.
    float graze = 1.0 - ndv;
    graze *= graze;
    graze *= graze;
    amb_spec *= 1.0 + sheen * (0.25 + 5.0 * graze);
    // The same shape, for the same reason, on glazing: env_brdf's split-sum
    // fit is a rough surface's answer and it flattens the Fresnel rise that a
    // near-mirror actually has. A window facing you returns its dim room; the
    // same window seen along a facade returns the scene outside. Weighting the
    // boost onto the grazing angles is what gives a row of windows that
    // gradient rather than uniformly pale glass.
    amb_spec *= 1.0 + glass * (0.35 + 4.0 * graze);

    // down-facing crevices see less of the sky
    float ao = mix(0.45, 1.0, hemi);

    // The window term rides on the same darkness test the glazing above uses:
    // what is glass by day is what lights up after dark.
    vec3 color = direct + (amb_diffuse + amb_spec) * ao
               + uEmissive + uWindowGlow * pane;

    float fog = 1.0 - exp(-view_depth * view_depth * uFogDensity * uFogDensity);
    frag = vec4(mix(color, srgb_to_linear(uFogColor), clamp(fog, 0.0, 1.0)), 1.0);
}
)";

// ---- sky pass ----
//
// A full-screen triangle drawn before the world, at the far plane, with depth
// writes off. The fragment shader turns each pixel back into a world-space view
// ray through the inverse view-projection and asks sky_env what colour that
// direction is - the same function the surface and water shaders sample for
// reflections, so the backdrop and every reflection of it are consistent.
const char* VSHDER_SKY = R"(#version 440 core
out vec2 vNDC;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2) * 2.0 - 1.0;
    vNDC = p;
    gl_Position = vec4(p, 1.0, 1.0);   // z = w -> sits on the far plane
}
)";

const char* FSHDER_SKY = R"(#version 440 core
in vec2 vNDC;
out vec4 frag;
uniform mat4  uInvViewProj;   // everything else it needs is in the scene block
void main() {
    vec4 far = uInvViewProj * vec4(vNDC, 1.0, 1.0);
    vec3 dir = normalize(far.xyz / far.w - uCameraPos);
    vec3 col = sky_env(dir, uSunDir, uSunColor, uSkyZenith, uSkyHorizon, uGroundColor);
    // let the lower sky settle into the same fog the world fades into, so the
    // horizon line does not read as a hard seam
    float haze = smoothstep(0.10, -0.05, dir.y);
    col = mix(col, srgb_to_linear(uFogColor), haze * 0.85);
    frag = vec4(col, 1.0);
}
)";

// skinned: per-vertex pos/normal/uv (loc 0..2) + bone ids/weights (loc 3..4).
// One draw per animated instance, since every instance needs its own uBones,
// so the model matrix rides along as a uniform rather than an instance buffer.
// Pairs with FSHDER_BASIC. uBones[] must match MAX_ANIM_BONES in animation.hpp.
// Note glTF's uv origin is already top-left, so unlike VSHDER_BASIC there is no V flip.
const char* VSHDER_SKINNED = R"(#version 440 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aUV;
layout(location=3) in vec4 aBoneIds;
layout(location=4) in vec4 aWeights;
uniform mat4 uViewProj;
uniform mat4 uModel;
uniform mat4 uBones[128];
out vec3 vNormal;
out vec2 vUV;
out vec3 vWorld;
void main() {
    mat4 skin = uBones[int(aBoneIds.x)] * aWeights.x
              + uBones[int(aBoneIds.y)] * aWeights.y
              + uBones[int(aBoneIds.z)] * aWeights.z
              + uBones[int(aBoneIds.w)] * aWeights.w;
    vec4 world = uModel * (skin * vec4(aPos, 1.0));
    vWorld = world.xyz;
    gl_Position = uViewProj * world;
    vNormal = mat3(uModel) * (mat3(skin) * aNormal);
    vUV = aUV;
}
)";

// ---- depth only, for the shadow pass ----
//
// The shadow map records how far the sun can see and nothing else, so these
// carry no varyings and write no colour. They share the instancing and skinning
// layouts of the shaders above so the same vertex arrays draw both passes.

const char* VSHDER_DEPTH = R"(#version 440 core
layout(location=0) in vec3 aPos;
layout(location=3) in mat4 aModel;
uniform mat4 uViewProj;   // the light's view-projection
void main() { gl_Position = uViewProj * (aModel * vec4(aPos, 1.0)); }
)";

const char* VSHDER_DEPTH_SKINNED = R"(#version 440 core
layout(location=0) in vec3 aPos;
layout(location=3) in vec4 aBoneIds;
layout(location=4) in vec4 aWeights;
uniform mat4 uViewProj;
uniform mat4 uModel;
uniform mat4 uBones[128];
void main() {
    mat4 skin = uBones[int(aBoneIds.x)] * aWeights.x
              + uBones[int(aBoneIds.y)] * aWeights.y
              + uBones[int(aBoneIds.z)] * aWeights.z
              + uBones[int(aBoneIds.w)] * aWeights.w;
    gl_Position = uViewProj * (uModel * (skin * vec4(aPos, 1.0)));
}
)";

const char* FSHDER_DEPTH = R"(#version 440 core
void main() { }
)";

// ---- water ----
//
// The wave field is one function, GLSL_WAVE, shared by the vertex and fragment
// stages: the vertex stage uses its height to actually move the surface, the
// fragment stage uses its gradient for a normal at full pixel resolution. It is
// evaluated from world position, so neighbouring tiles agree exactly along
// their shared edge and the river is one continuous sheet rather than a grid of
// independently rippling squares.
//
// Everything that makes water read as water is here:
//
//   * the surface moves. A flat plane with a clever shader on it is a painted
//     floor; a surface whose silhouette rises and falls against the far bank is
//     immediately liquid. This is the single biggest cue and it needs real
//     geometry, which is why the tile is subdivided.
//   * four octaves - two slow swells crossing, two fast chops - because one
//     sine reads as corrugated metal
//   * Fresnel drives the mix from body colour to sky reflection: near
//     transparent underfoot, a mirror toward the horizon
//   * the body colour is a green-brown that deepens with distance below the
//     surface, so shallows near a bank read differently from midstream
//   * a tight specular for sun glitter over a broad one for sheen
const char* GLSL_WAVE = R"(
// .xyz is the surface normal, .w the height offset in metres.
//
// Six directional waves over three scales. The scales matter as much as the
// count: the swell moves the silhouette, the chop is what the eye reads as
// "liquid" from a few metres away, and the ripple is fine enough to keep
// breaking up the sky reflection out to the far bank. Drop the ripple and a
// river seen at a glancing angle - which is nearly always, from a camera
// standing on the bank - turns into one unbroken mirror of a smooth sky
// gradient, and a mirror that clean reads as sheet metal, not water.
vec4 wave_field(vec2 p, float time) {
    // direction, spatial frequency, amplitude (m), speed
    const vec2  D[6] = vec2[6](vec2( 0.80,  0.60), vec2(-0.45,  0.89),
                               vec2( 0.99, -0.14), vec2( 0.20,  0.98),
                               vec2( 0.71, -0.71), vec2(-0.92, -0.39));
    const float F[6] = float[6](0.085, 0.130, 0.480, 0.730, 1.850, 2.410);
    const float A[6] = float[6](0.300, 0.190, 0.070, 0.045, 0.016, 0.011);
    const float S[6] = float[6](0.550, 0.410, 1.700, 2.300, 3.400, 4.100);

    float h = 0.0, dx = 0.0, dz = 0.0;
    for (int i = 0; i < 6; ++i) {
        float phase = dot(p, D[i]) * F[i] + time * S[i];
        h  += A[i] * sin(phase);
        float c = cos(phase) * A[i] * F[i];
        dx += c * D[i].x;
        dz += c * D[i].y;
    }

    // The gradient of a field this gentle is nearly flat, so the normal is
    // exaggerated - the ripple has to be visible in the lighting, not merely
    // present in the geometry.
    return vec4(normalize(vec3(-dx * 14.0, 1.0, -dz * 14.0)), h);
}

// Same six waves, but built for a fragment that already knows how many world
// metres one pixel covers (`footprint`, from fwidth of the world position).
//
// A river is looked at nearly edge-on from a walking camera, and world-space
// derivatives explode toward the far bank the way they do for any ground
// plane at a grazing angle. wave_field's finer octaves have wavelengths of a
// couple of metres; once footprint approaches that, each pixel is sampling a
// different point on the sine mid-cycle and the *unfiltered* normal - and the
// tight specular lobe built from it - flickers between crest and trough from
// one pixel to the next. That is the hard-edged diagonal banding a plain
// wave_field() read on the water: it is aliasing, not ripple.
//
// The fix is the standard one for procedural normals: fade each octave out
// once a pixel can no longer resolve it, so the normal degrades toward the
// low-frequency swell (which stays smooth at any distance) instead of
// aliasing. Height is untouched - only the fragment-stage normal needs this.
vec3 wave_normal_aa(vec2 p, float time, float footprint) {
    const vec2  D[6] = vec2[6](vec2( 0.80,  0.60), vec2(-0.45,  0.89),
                               vec2( 0.99, -0.14), vec2( 0.20,  0.98),
                               vec2( 0.71, -0.71), vec2(-0.92, -0.39));
    const float F[6] = float[6](0.085, 0.130, 0.480, 0.730, 1.850, 2.410);
    const float A[6] = float[6](0.300, 0.190, 0.070, 0.045, 0.016, 0.011);
    const float S[6] = float[6](0.550, 0.410, 1.700, 2.300, 3.400, 4.100);

    float dx = 0.0, dz = 0.0;
    for (int i = 0; i < 6; ++i) {
        // ~2 pixels per wavelength is the Nyquist floor; start rolling the
        // octave off well before that so the fade itself never aliases.
        float wavelength = 6.28318531 / F[i];
        float fade = 1.0 - smoothstep(wavelength * 0.25, wavelength * 1.0, footprint);
        if (fade <= 0.001) continue;
        float phase = dot(p, D[i]) * F[i] + time * S[i];
        float c = cos(phase) * A[i] * F[i] * fade;
        dx += c * D[i].x;
        dz += c * D[i].y;
    }
    return normalize(vec3(-dx * 14.0, 1.0, -dz * 14.0));
}
)";

// Splices both GLSL_COMMON and GLSL_WAVE in after the #version line.
//
// Unlike shader_with_common this alternates between two buffers, because a
// vertex and a fragment source have to be live at the same moment to be handed
// to one opengl_create_shader call - and the water pass is the one place where
// both stages need the same helper spliced in.
static const char* shader_with_waves(const char* src) {
    static char buf[2][24576];
    static int  slot = 0;
    char* out = buf[slot];
    slot ^= 1;

    const char* nl = strchr(src, '\n');
    if (!nl) return src;
    size_t head = (size_t)(nl - src) + 1;
    memcpy(out, src, head);
    int n = (int)head;
    n += sprintf(out + n, "%s\n%s\n%s\n", GLSL_SCENE, GLSL_COMMON, GLSL_WAVE);
    sprintf(out + n, "%s", src + head);
    return out;
}

const char* VSHDER_WATER = R"(#version 440 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aUV;
layout(location=3) in mat4 aModel;
uniform mat4  uViewProj;
uniform float uTime;
out vec3 vWorld;
void main() {
    vec4 world = aModel * vec4(aPos, 1.0);
    // Displace in world space, not model space: the tile is scaled by its cell
    // size, and lifting before that scale would make the waves taller on a
    // bigger tile.
    world.y += wave_field(world.xz, uTime).w;
    vWorld = world.xyz;
    gl_Position = uViewProj * world;
}
)";

const char* FSHDER_WATER = R"(#version 440 core
in vec3 vWorld;
out vec4 frag;
uniform vec3  uShallowColor;
uniform vec3  uDeepColor;
uniform float uTime;
uniform float uBedDepth;      // metres from the surface down to the bed

// The planar reflection captured this frame by pix__reflection_pass: the
// opaque scene and sky, drawn again from a camera mirrored across the water's
// surface. A water fragment finds its own reflection the way any projected
// texture is looked up: run its world position through that camera's own
// view-projection to get back the screen position *it* was drawn at, and
// sample there. That is where the reflected scene across the water actually
// comes from; the analytic sky_env is kept only as what fills the
// gaps - outside the sun's cone, past the far edge of the frustum, or
// whenever the pass did not run this frame (effects off, no camera above the
// water to mirror).
uniform sampler2D uReflection;
uniform mat4      uReflViewProj;
uniform float     uReflectionMix;   // 0 = analytic sky only, 1 = captured reflection

void main() {
    vec4 wave = wave_field(vWorld.xz, uTime);
    // How many world metres one screen pixel spans here - large at a distant,
    // near-grazing view of the surface, small underfoot. Feeds the fade in
    // wave_normal_aa so the shading normal never carries more ripple detail
    // than this pixel could actually resolve; see that function for why.
    float footprint = max(fwidth(vWorld.x), fwidth(vWorld.z));
    vec3 n = wave_normal_aa(vWorld.xz, uTime, footprint);
    vec3 view = normalize(uCameraPos - vWorld);

    // Schlick, with the floor raised a little past water's real ~0.02
    // normal-incidence reflectance: the planar reflection now genuinely
    // carries the scene, so straight-down water is worth showing more of it
    // rather than mostly the body colour underneath.
    float fresnel = 0.06 + 0.94 * pow(1.0 - clamp(dot(n, view), 0.0, 1.0), 5.0);

    // How far the eye is looking through the water: straight down it is the
    // depth to the bed, at a glancing angle it is far further, and that is what
    // makes the far side of a river read darker than the near edge. Wave height
    // rides on top of it, so a crest is lighter than the trough beside it.
    float through = uBedDepth / max(abs(view.y), 0.06);
    float murk = 1.0 - exp(-through * 0.30);
    float crest = clamp(wave.w * 1.3 + 0.5, 0.0, 1.0);
    vec3 body = mix(srgb_to_linear(uShallowColor), srgb_to_linear(uDeepColor),
                    clamp(murk - crest * 0.25, 0.0, 1.0));
    // The body colour is a fixed pigment, but what lights it is not: the same
    // silty water is bright jade at noon and nearly black by starlight. Tying
    // it to the sky's own brightness (already on hand as uSkyZenith, for the
    // reflection above) is what keeps the stretch of river nearest the camera
    // - where the low fresnel above leaves the body colour dominant - from
    // reading as a flat, time-of-day-blind black pool in broad daylight.
    float sky_luma = dot(uSkyZenith, vec3(0.2126, 0.7152, 0.0722));
    body *= clamp(mix(0.4, 1.6, sky_luma / 1.8), 0.4, 1.7);

    // The reflection is a real mirror sample of the same analytic sky the
    // backdrop and every other surface use, taken along the wave-perturbed
    // reflection vector - so the water reflects the actual horizon gradient and
    // catches the sun where the geometry says it should.
    //
    // Scaled down on the way in, because a surface that returns the sky at full
    // strength is a mirror, and water is not: about four fifths of it comes
    // back and the rest goes into the body of the water.
    vec3 refl = reflect(-view, n);
    refl.y = abs(refl.y);   // waves that tilt the normal below horizontal still see sky
    vec3 analytic_sky = sky_env(refl, uSunDir, uSunColor, uSkyZenith, uSkyHorizon, uGroundColor);

    // Project this point of the surface - nudged sideways by the wave normal,
    // so the skyline shimmers and breaks up with the same ripple that shapes
    // the sun glitter below - through the reflection camera's own matrix to
    // find where it was drawn in reflection_target. Skipped entirely when the
    // pass did not run this frame: uReflViewProj is then stale, and dividing
    // by a stale w is exactly the kind of thing that produces a NaN that
    // mix()'s zero weight would not actually protect against.
    vec3 captured = vec3(0.0);
    if (uReflectionMix > 0.5) {
        vec3 sample_at = vWorld + vec3(n.x, 0.0, n.z) * 0.6;
        vec4 rc = uReflViewProj * vec4(sample_at, 1.0);
        vec2 refl_uv = clamp((rc.xy / max(rc.w, 1e-4)) * 0.5 + 0.5, vec2(0.003), vec2(0.997));
        // A manual mip bias on top of whatever the hardware's own derivative
        // picks: the UV came out of a perspective divide rather than a plain
        // varying, so its screen-space derivative - and therefore automatic
        // mip selection - is not as reliable a measure of this fragment's
        // real footprint in the capture as it would be for an ordinary
        // texture lookup. Forcing a couple of extra mips softens it in every
        // case rather than only the ones the derivative happens to catch.
        captured = texture(uReflection, refl_uv, 1.6).rgb;
    }
    // Pulled down further than the analytic-only version needed: a captured
    // reflection is a real, high-contrast photo of the scene rather than a
    // soft sky gradient, and returning it at the same strength read as
    // polished metal even with the mip-blurred capture and the wave
    // distortion above. Held back more, and blended with a fifth of the
    // still-perfectly-smooth analytic sky even when the capture is
    // available, it settles back into looking like a reflection *in* water
    // rather than a mirror sitting on top of it.
    vec3 sky = mix(analytic_sky, mix(analytic_sky, captured, 0.8), uReflectionMix) * 0.62;

    vec3 half_v = normalize(uSunDir + view);
    // Two lobes, both kept modest. A very tight, very bright highlight on a
    // surface whose waves are smaller than a pixel at any distance does not
    // sparkle, it smears into a chrome band across the whole river.
    //
    // The glitter lobe's own exponent is widened with distance too, in step
    // with wave_normal_aa's fade above: a filtered normal removes the
    // per-pixel flicker in the normal itself, but a pow(x, 200) highlight
    // built from it can still crawl as the (now-smoother) normal sweeps a
    // whole lobe-width between neighbouring pixels. Softening the lobe as
    // footprint grows keeps the sun glitter as a sparkle near the bank and a
    // calm, broad sheen toward the far side rather than a hard band.
    float far = clamp(footprint / 3.0, 0.0, 1.0);
    float glitter_pow = mix(200.0, 24.0, far);
    float ndh = max(dot(n, half_v), 0.0);
    float spec = pow(ndh, glitter_pow) * mix(1.6, 0.5, far);   // sun glitter
    spec += pow(ndh, 18.0) * 0.06;                             // broad sheen

    // Never a pure reflection even edge on. Some of what comes back off water at
    // a glancing angle is still the water, and holding a little of the body
    // colour in is what keeps a river reading as a river rather than as a strip
    // of polished metal laid in the ground.
    vec3 color = mix(body, sky, fresnel * 0.80) + uSunColor * spec;

    // atmospheric fog only at real distance - the near-field colour above is
    // left alone so nearby water still reads as water, not haze
    float dist = length(uCameraPos - vWorld);
    float fog = 1.0 - exp(-dist * dist * uFogDensity * uFogDensity);
    color = mix(color, srgb_to_linear(uFogColor), clamp(fog, 0.0, 1.0) * 0.85);

    // Opacity tracks the same view-through-the-water term as the colour: you
    // can see the bed at your feet and cannot see it across the river.
    frag = vec4(color, clamp(mix(0.55, 0.98, max(murk, fresnel)), 0.0, 1.0));
}
)";

// ---- post processing ----
//
// One triangle larger than the screen, built from gl_VertexID with no vertex
// buffer bound at all. A quad made of two triangles would shade the pixels
// along its shared diagonal twice and can show a seam; this cannot.
const char* VSHDER_POST = R"(#version 440 core
out vec2 vUV;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2) * 2.0 - 1.0;
    vUV = p * 0.5 + 0.5;
    gl_Position = vec4(p, 0.0, 1.0);
}
)";

// ---- bloom ----
//
// The scene target is real HDR, so a sun glint off a car roof or a wave crest
// is genuinely brighter than 1 while the tonemap squashes it back to near
// white. Bloom is what puts that energy back on screen as glare, and it is the
// single thing that makes a specular highlight read as *bright* rather than as
// a pale patch. Threshold with a soft knee (a hard cutoff bands visibly as
// something crosses it), then two separable Gaussian passes at quarter res.
const char* FSHDER_BLOOM_PREFILTER = R"(#version 440 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uScene;
uniform vec2  uTexel;      // one source texel
uniform float uThreshold;
uniform float uKnee;
void main() {
    // 4-tap box downsample first: a one-pixel spark left to alias would crawl
    vec3 c = texture(uScene, vUV + vec2(-uTexel.x, -uTexel.y)).rgb
           + texture(uScene, vUV + vec2( uTexel.x, -uTexel.y)).rgb
           + texture(uScene, vUV + vec2(-uTexel.x,  uTexel.y)).rgb
           + texture(uScene, vUV + vec2( uTexel.x,  uTexel.y)).rgb;
    c *= 0.25;

    float lum = dot(c, vec3(0.2126, 0.7152, 0.0722));
    float soft = clamp(lum - uThreshold + uKnee, 0.0, 2.0 * uKnee);
    soft = soft * soft / (4.0 * uKnee + 1e-5);
    float w = max(soft, lum - uThreshold) / max(lum, 1e-5);
    frag = vec4(c * w, 1.0);
}
)";

// five taps standing in for a nine-tap Gaussian: the hardware's linear filter
// fetches each outer pair in one sample at a weighted offset
const char* FSHDER_BLOOM_BLUR = R"(#version 440 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uSource;
uniform vec2 uDirection;   // one texel along the axis being blurred
void main() {
    vec3 c = texture(uSource, vUV).rgb * 0.2270270270;
    c += (texture(uSource, vUV + uDirection * 1.3846153846).rgb
        + texture(uSource, vUV - uDirection * 1.3846153846).rgb) * 0.3162162162;
    c += (texture(uSource, vUV + uDirection * 3.2307692308).rgb
        + texture(uSource, vUV - uDirection * 3.2307692308).rgb) * 0.0702702703;
    frag = vec4(c, 1.0);
}
)";

const char* FSHDER_POST = R"(#version 440 core
in vec2 vUV;
out vec4 frag;
uniform sampler2D uScene;
uniform sampler2D uBloom;
uniform vec2  uTexel;
uniform float uExposure;
uniform float uSaturation;
uniform float uVignette;
uniform float uSharpen;
uniform float uBloomStrength;
// The grade. One number for how hard the contrast curve bites and one for how
// far the shadows and highlights are pulled apart in hue - see `grade` below.
uniform float uContrast;
uniform float uSplitTone;

// ---- rain ----
//
// Screen space, not particles. Rain seen from a chase camera is a wall of
// near-identical streaks with no parallax worth the name, so simulating it in
// world space buys nothing a scrolling noise field does not already give, and
// costs a draw call and a buffer per frame. uRain is the downpour, 0..1.
uniform float uRain;
uniform float uTime;

float rain_hash(vec2 p) {
    return fract(sin(dot(p, vec2(41.7, 289.1))) * 43758.5453);
}

// One layer of falling drops.
//
// The screen is cut into columns; a column is a stream of drops, each one a
// short comet - bright at the leading end, fading out behind it, which is what
// a real drop looks like on any exposure long enough to see it at all. The two
// things that make this read as rain rather than as noise are that a drop is
// far shorter than the gap to the next one in its column, and that it is
// roughly a pixel and a half wide however far away the layer is meant to be.
//
// `uv` arrives aspect corrected, so a column is as wide on screen as it is
// tall-per-unit, and a streak stays vertical instead of shearing with the
// window. `rows` is how many drop-lengths fit down the screen, so it and
// `len` together set both the length of a drop and the spacing between drops.
float rain_layer(vec2 uv, float cols, float rows, float speed, float len,
                 float wind, float seed) {
    vec2 g = vec2((uv.x + uv.y * wind) * cols + seed * 13.7, uv.y * rows);

    float col = floor(g.x);
    float r1 = rain_hash(vec2(col, seed));

    // Plus, not minus: vUV.y here is the fullscreen triangle's clip-space y,
    // so it runs bottom to top and scrolling the field the intuitive way sends
    // the rain up into the sky. The r1 term offsets each column by a random
    // amount so neighbouring columns are not falling in step.
    float y = g.y + uTime * speed * (0.75 + r1 * 0.55) + r1 * 41.0;
    float row = floor(y);

    // Whether this slot carries a drop at all is rolled per (column, slot), so
    // the spacing down a column is irregular. A drop in every slot is a grid,
    // and a grid reads as a texture stuck to the screen.
    float present = rain_hash(vec2(col, row * 1.7 + seed));
    if (present < 0.45) return 0.0;

    float fy = fract(y);
    float streak = smoothstep(len, 0.0, fy);              // head at fy 0, tail behind
    float across = smoothstep(0.05, 0.0, abs(fract(g.x) - 0.5));
    // and not every drop the same brightness - depth within the layer
    return streak * across * (0.45 + present * 0.55);
}

// Narkowicz's fit of the ACES filmic curve: cheap, and it rolls highlights off
// instead of clipping them flat white the way a plain clamp does
vec3 tonemap(vec3 x) {
    const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

// The grade: the last thing done to the image, and the cheapest place to buy
// the largest change in how it reads.
//
// Two things, both of which every graded photograph has and a raw tonemap
// does not.
//
// A contrast curve. The tonemap's job is to fit a huge range into zero to one
// and it does that by flattening the middle, which is why a straight ACES
// image looks correct and washed at the same time. A smoothstep-shaped curve
// about mid grey puts the separation back where the eye actually looks - the
// midtones - while leaving the rolled-off ends where the tonemap put them.
// Done per channel rather than on luminance, because a per-channel curve also
// deepens saturation as it steepens, which is what film does.
//
// A split tone. Shadows toward blue, highlights toward warm. Daylight is two
// light sources - a warm sun and a blue sky - and everything in shadow is lit
// by the second one, so this is not a stylistic tint so much as the thing the
// single-colour ambient term in the surface shader cannot express. It is what
// separates a grey surface in sun from the same grey surface in shade.
vec3 grade(vec3 c) {
    vec3 curved = c * c * (3.0 - 2.0 * c);          // smoothstep about mid grey
    c = mix(c, curved, uContrast);

    float luma = dot(c, vec3(0.2126, 0.7152, 0.0722));
    // Weighted so each end owns its half of the range and neither reaches the
    // other: a split tone that meets in the middle is a colour cast.
    float shadow = 1.0 - smoothstep(0.0, 0.45, luma);
    float light  = smoothstep(0.55, 1.0, luma);
    // Small numbers on purpose. This is a *lean*, not a tint: the shadows are
    // already blue - they are lit by a blue sky ambient - and a heavy hand here
    // does not deepen that, it lays a colour cast over the whole frame, which
    // is what twice these values does.
    const vec3 COOL = vec3(-0.010, 0.000, 0.022);   // sky in the shadows
    const vec3 WARM = vec3( 0.028, 0.011, -0.014);  // sun in the highlights
    return c + (COOL * shadow + WARM * light) * uSplitTone;
}

void main() {
    vec3 color = texture(uScene, vUV).rgb;

    // a light unsharp mask puts the edges back that the resolve softened
    if (uSharpen > 0.0) {
        vec3 blur = texture(uScene, vUV + vec2( uTexel.x, 0.0)).rgb
                  + texture(uScene, vUV + vec2(-uTexel.x, 0.0)).rgb
                  + texture(uScene, vUV + vec2(0.0,  uTexel.y)).rgb
                  + texture(uScene, vUV + vec2(0.0, -uTexel.y)).rgb;
        color += (color - blur * 0.25) * uSharpen;
    }

    // glare is added before the tonemap, in linear light, so it rolls off with
    // everything else instead of clipping on top of an already-mapped image
    if (uBloomStrength > 0.0) color += texture(uBloom, vUV).rgb * uBloomStrength;

    // the scene target holds linear HDR radiance; tonemap in linear, then this
    // is the one place the whole pipeline encodes back to sRGB for the 8-bit
    // backbuffer
    color = tonemap(color * uExposure);
    color = grade(color);

    float luma = dot(color, vec3(0.2126, 0.7152, 0.0722));
    color = mix(vec3(luma), color, uSaturation);

    if (uRain > 0.001) {
        // Aspect corrected, so a streak is vertical and a pixel wide at any
        // window shape, and slanted per layer: rain that all falls at exactly
        // one angle looks painted on, and rain that falls straight down looks
        // like it is in a lift shaft.
        float aspect = uTexel.y / uTexel.x;
        vec2 ruv = vec2(vUV.x * aspect, vUV.y);

        // three depths. The near layer is long, fast, sparse and bright; the
        // far one is short, slow, dense and dim, which is what puts distance
        // between them - a single layer at any setting reads as a flat sheet.
        float drops = rain_layer(ruv,  70.0,  6.5, 2.9, 0.62, 0.16, 0.0) * 1.00
                    + rain_layer(ruv, 130.0, 12.0, 2.2, 0.50, 0.21, 3.0) * 0.62
                    + rain_layer(ruv, 240.0, 22.0, 1.7, 0.42, 0.26, 8.0) * 0.36;

        // A drop is not a light source; it is a lens full of whatever is
        // behind it, which in daylight is mostly the sky. Scaling by the local
        // brightness is what keeps rain at noon bright, rain against a dark
        // wall dim, and rain at night nearly invisible except where a lamp
        // catches it - all of which a fixed grey gets wrong.
        float lit = clamp(luma * 1.5 + 0.20, 0.0, 1.5);
        float wash = clamp(drops * uRain, 0.0, 1.0);
        color = mix(color, color * 0.88 + vec3(0.62, 0.68, 0.80) * lit * 0.55, wash);
    }

    // A smoothstep rather than the plain quadratic this was: dot(d, d) starts
    // falling from the very centre of the frame, so the darkening covered the
    // whole image and showed as a gradient band across a flat sky rather than
    // as a corner falloff. Held off until well out from the middle, it does
    // what a vignette is for - keeping the eye in the frame - and does not
    // touch the middle of the picture at all.
    vec2 d = vUV - 0.5;
    color *= 1.0 - uVignette * smoothstep(0.16, 0.62, dot(d, d));

    frag = vec4(linear_to_srgb(color), 1.0);
}
)";

// instanced: unit quad corner (loc 0), per-instance box + atlas crop + texture
// index (loc 1..3). aTexIndex selects both the sampler and its uTexSize entry,
// so one batch draws in a single call across up to MAX_SPRITE_TEXTURES(32)
// distinct textures - keep the array sizes below in sync with sprite.hpp.
const char* VSHDER_SPRITE = R"(#version 440 core
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

const char* FSHDER_SPRITE = R"(#version 440 core
in vec2 vUV;
flat in uint vTexIndex;
out vec4 frag;
uniform sampler2D uTex[32];
void main() {
    frag = texture(uTex[vTexIndex], vUV);
}
)";


// uItalic shears the quad in place (no extra per-instance data): the top
// edge of each glyph's own box shifts right, the bottom edge doesn't move -
// a cheap approximation of a true glyph-shape slant, good enough for UI text
const char* VSHDER_TEXT = R"(#version 440 core
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

// atlas holds a signed distance field (0.5 = outline); fwidth() adapts the
// smoothstep band to the glyph's current on-screen scale, so edges stay crisp
// whether the text is shrunk or magnified. uWeight biases the fill edge to
// synthesize bold(+)/thin(-); uOutlineWidth carves a second band below the
// fill edge, in the same normalized sdf units, filled with uOutlineColor.
const char* FSHDER_TEXT = R"(#version 440 core
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
    // fillAlpha/outlineAlpha is 0 in the ring-only band and ramps to 1 exactly
    // at the fill edge; when outlineWidth is 0 the two edges coincide, the
    // ratio collapses to 1 everywhere, and uOutlineColor drops out entirely -
    // a plain mix(outline, color, fillAlpha) would instead tint every glyph's
    // antialiased rim toward outline colour even with no outline requested
    float t = outlineAlpha > 1e-4 ? clamp(fillAlpha / outlineAlpha, 0.0, 1.0) : 0.0;
    vec3 rgb = mix(uOutlineColor.rgb, uColor.rgb, t);
    float alpha = outlineAlpha * mix(uOutlineColor.a, uColor.a, t);
    frag = vec4(rgb, alpha);
}
)";

// ---- particles ----
//
// One camera-facing quad per particle, four vertices apiece, with no vertex
// buffer at all: the corner is derived from gl_VertexID and everything else is
// per-instance. That is deliberate - a muzzle flash is a handful of quads that
// live for a tenth of a second, and the cheapest thing a system like that can
// do is not touch a mesh pool it would immediately have to give back.
//
// Colour is linear radiance and is not clamped, so a flash written at 14.0
// blows straight through the bloom threshold and glares, while smoke written
// at 0.05 sits under it and does not. Which is the whole reason particles are
// drawn into the HDR target rather than composited afterwards.
const char* VSHDER_PARTICLE = R"(#version 440 core
layout(location=0) in vec4 aCentreSize;   // xyz world centre, w half-size in metres
layout(location=1) in vec4 aColor;        // rgb linear radiance, a coverage
uniform mat4 uViewProj;
uniform vec3 uCamRight;
uniform vec3 uCamUp;
out vec2 vUV;
out vec4 vColor;
void main() {
    // 0,1,2,3 as a triangle strip: (-1,-1) (1,-1) (-1,1) (1,1)
    vec2 corner = vec2((gl_VertexID & 1) == 0 ? -1.0 : 1.0,
                       (gl_VertexID & 2) == 0 ? -1.0 : 1.0);
    vUV = corner;
    vColor = aColor;
    vec3 world = aCentreSize.xyz + (uCamRight * corner.x + uCamUp * corner.y) * aCentreSize.w;
    gl_Position = uViewProj * vec4(world, 1.0);
}
)";

// A soft round blob rather than a square: the quad is only a carrier, and the
// squared falloff is what keeps a spark from reading as a pixel and a smoke
// puff from reading as a card.
const char* FSHDER_PARTICLE = R"(#version 440 core
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
