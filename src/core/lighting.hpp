#pragma once
#include <math.h>
#include "dtype.hpp"
#include "math.hpp"

// Everything that lights the world, kept apart from the thing that draws it.
//
// The renderer owns meshes, materials and draw calls; what those draws are lit
// by is described here and handed to it. That split is what lets the lighting
// grow - more lights, a different sky, a time of day, a second shadow cascade -
// without any of it reaching into the draw loop.
//
// Three kinds of light, and they are deliberately different things rather than
// one general case:
//
//   sun      one directional light, the only one that casts a shadow map. It is
//            by far the brightest thing in a daytime scene and it is what the
//            cascades are built around.
//   sky      an environment, not a light: a hemisphere of diffuse irradiance
//            plus a dome colour that every glossy surface reflects. Analytic,
//            so the backdrop, a reflection in a glossy surface and a reflection
//            in water are all the same function evaluated three times.
//   punctual point and spot lights - lamps, torches, headlights, windows.
//            Gathered per frame and culled to the ones nearest the camera.
//
// A punctual light is stored the way the shader wants it, so uploading a frame
// is three glUniform4fv calls and no per-light work.

// what a frame may collect before culling
#define MAX_SCENE_LIGHTS   512
// what one draw actually uploads. Every one of these is evaluated per fragment,
// so this is a real cost; the nearest N to the camera are the ones that survive.
#define MAX_SHADER_LIGHTS  32
// cascades in the sun's shadow atlas, near to far
#define SHADOW_CASCADES    3

struct pix_light {
    vec3  position;
    float radius;      // metres; the falloff is windowed to reach exactly zero here
    vec3  color;       // linear radiance, intensity already folded in
    float cos_inner;   // spot: cosine of the cone the light is at full strength in
    vec3  direction;   // spot only, normalised, pointing the way the light shines
    float cos_outer;   // spot: cosine of the cone it has faded out by. < -1 = point
};

// a point light: falls off to nothing at `radius`, shines in every direction
static pix_light pix_point_light(vec3 position, float radius, vec3 color) {
    pix_light l;
    l.position = position;
    l.radius = radius;
    l.color = color;
    l.cos_inner = 1.0f;
    l.direction = v3(0.0f, -1.0f, 0.0f);
    l.cos_outer = -2.0f;          // the sentinel that means "not a spot"
    return l;
}

// a cone light: full strength inside `inner` radians of the axis, gone by `outer`
static pix_light pix_spot_light(vec3 position, vec3 direction, float radius,
                                float inner, float outer, vec3 color) {
    pix_light l;
    l.position = position;
    l.radius = radius;
    l.color = color;
    l.direction = v3norm(direction);
    l.cos_inner = cosf(inner);
    l.cos_outer = cosf(outer);
    if (l.cos_outer >= l.cos_inner) l.cos_outer = l.cos_inner - 0.001f;
    return l;
}

struct pix_sun {
    vec3 direction;    // normalised, pointing *toward* the sun
    vec3 color;        // linear radiance; well above 1 in daylight
};

struct pix_sky {
    vec3 zenith;       // dome colour straight up   (backdrop and reflections)
    vec3 horizon;      // dome colour at the horizon
    vec3 ground;       // what the dome fades to below the horizon
    vec3 diffuse_up;   // hemisphere irradiance arriving from above
    vec3 diffuse_down; // hemisphere irradiance bounced back off the ground
    vec3 fog;
    float fog_density;
};

// ---- time of day ----
//
// Keyframed rather than derived from a physical sky model, because the point of
// this is that every hour of the day looks deliberately chosen. Each keyframe is
// a complete lighting setup that was picked to look right; the hour in between
// is a straight interpolation of two of them, which cannot land anywhere
// surprising.
//
// The keys wrap, so 23:00 blends into 05:00 through the night key rather than
// racing backwards through noon.
struct pix_daylight {
    float  hour;
    float  elevation;   // degrees above the horizon
    float  azimuth;     // degrees, 0 = +X, turning toward +Z
    // What the camera's aperture does about it. A night scene lit by lamps
    // carries a hundredth of the light a noon scene does, and no amount of
    // adding lamps closes that gap - the eye adapts to it, and so does every
    // camera, so the exposure is part of the hour rather than a constant.
    float  exposure;
    pix_sun sun;
    pix_sky sky;
};

#define PIX_DAY_KEYS 6

// The two irradiance columns are the ones worth understanding: they are what
// lights everything the sun cannot see, which after cascaded shadows went in is
// most of a scene. A clear midday sky delivers roughly a fifth of what the sun
// does onto a horizontal surface, and setting them much below that is what
// makes shaded ground read as dusk in the middle of the afternoon.
static const pix_daylight PIX_DAY[PIX_DAY_KEYS] = {
    //           hr   elev   azim   sun colour                sky zenith            horizon              ground             up irradiance       down irradiance    fog                 density
    { 0.0f,   -18.0f, 300.0f, 1.70f, { {}, { 0.05f, 0.06f, 0.11f } }, { { 0.010f, 0.014f, 0.035f }, { 0.030f, 0.038f, 0.070f }, { 0.006f, 0.006f, 0.010f }, { 0.060f, 0.070f, 0.115f }, { 0.026f, 0.026f, 0.034f }, { 0.05f, 0.06f, 0.10f }, 0.0016f } },
    { 6.0f,     4.0f,  75.0f, 1.30f, { {}, { 1.45f, 0.72f, 0.42f } }, { { 0.090f, 0.170f, 0.360f }, { 0.560f, 0.430f, 0.360f }, { 0.070f, 0.060f, 0.055f }, { 0.220f, 0.245f, 0.320f }, { 0.095f, 0.085f, 0.070f }, { 0.62f, 0.55f, 0.50f }, 0.0022f } },
    { 9.0f,    38.0f,  60.0f, 0.95f, { {}, { 2.00f, 1.68f, 1.32f } }, { { 0.130f, 0.310f, 0.620f }, { 0.540f, 0.670f, 0.840f }, { 0.120f, 0.115f, 0.100f }, { 0.360f, 0.450f, 0.610f }, { 0.180f, 0.172f, 0.145f }, { 0.58f, 0.69f, 0.84f }, 0.0012f } },
    { 13.0f,   66.0f,  30.0f, 0.90f, { {}, { 2.15f, 1.95f, 1.70f } }, { { 0.120f, 0.300f, 0.640f }, { 0.560f, 0.690f, 0.860f }, { 0.140f, 0.135f, 0.115f }, { 0.390f, 0.480f, 0.640f }, { 0.200f, 0.192f, 0.165f }, { 0.60f, 0.71f, 0.86f }, 0.0011f } },
    { 18.5f,    8.0f, 285.0f, 1.10f, { {}, { 2.10f, 1.10f, 0.58f } }, { { 0.100f, 0.190f, 0.400f }, { 0.780f, 0.510f, 0.340f }, { 0.090f, 0.075f, 0.065f }, { 0.250f, 0.245f, 0.290f }, { 0.120f, 0.100f, 0.080f }, { 0.74f, 0.56f, 0.44f }, 0.0020f } },
    { 21.0f,  -12.0f, 300.0f, 1.60f, { {}, { 0.16f, 0.16f, 0.26f } }, { { 0.020f, 0.026f, 0.060f }, { 0.070f, 0.070f, 0.110f }, { 0.010f, 0.010f, 0.016f }, { 0.085f, 0.095f, 0.145f }, { 0.038f, 0.038f, 0.048f }, { 0.09f, 0.10f, 0.15f }, 0.0018f } }
};

static vec3 pix__mix3(vec3 a, vec3 b, float t) { return v3lerp(a, b, t); }

// Sun direction from an elevation and azimuth in degrees. Azimuth turns from
// +X toward +Z, so it agrees with the direction convention the rest of the
// world uses.
static vec3 pix_sun_direction(float elevation_deg, float azimuth_deg) {
    float e = elevation_deg * 0.01745329f;
    float a = azimuth_deg * 0.01745329f;
    float ce = cosf(e);
    return v3norm(v3(ce * cosf(a), sinf(e), ce * sinf(a)));
}

// The complete lighting setup for an hour of the day, 0..24.
// `hour` is wrapped, so a caller may wind the clock past either end of a day.
static void pix_daylight_at(float hour, pix_sun* out_sun, pix_sky* out_sky,
                            float* out_exposure) {
    hour = fmodf(hour, 24.0f);
    if (hour < 0.0f) hour += 24.0f;

    int i = PIX_DAY_KEYS - 1;
    for (int k = 0; k < PIX_DAY_KEYS; k++)
        if (PIX_DAY[k].hour <= hour) i = k;
    int j = (i + 1) % PIX_DAY_KEYS;

    float from = PIX_DAY[i].hour;
    float to   = PIX_DAY[j].hour;
    float span = to - from;
    if (span <= 0.0f) span += 24.0f;         // the key pair that wraps midnight
    float t = span > 1e-4f ? (hour - from) / span : 0.0f;
    if (t < 0.0f) t += 24.0f / span;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);

    const pix_daylight& A = PIX_DAY[i];
    const pix_daylight& B = PIX_DAY[j];

    // Elevation and azimuth are interpolated rather than the direction vector:
    // two directions that differ by most of the sky lerp through the middle of
    // the dome, which puts the sun underground at dusk.
    float azim = A.azimuth + (B.azimuth - A.azimuth) * t;
    float elev = A.elevation + (B.elevation - A.elevation) * t;
    out_sun->direction = pix_sun_direction(elev, azim);
    out_sun->color = pix__mix3(A.sun.color, B.sun.color, t);
    *out_exposure = A.exposure + (B.exposure - A.exposure) * t;

    out_sky->zenith       = pix__mix3(A.sky.zenith, B.sky.zenith, t);
    out_sky->horizon      = pix__mix3(A.sky.horizon, B.sky.horizon, t);
    out_sky->ground       = pix__mix3(A.sky.ground, B.sky.ground, t);
    out_sky->diffuse_up   = pix__mix3(A.sky.diffuse_up, B.sky.diffuse_up, t);
    out_sky->diffuse_down = pix__mix3(A.sky.diffuse_down, B.sky.diffuse_down, t);
    out_sky->fog          = pix__mix3(A.sky.fog, B.sky.fog, t);
    out_sky->fog_density  = A.sky.fog_density + (B.sky.fog_density - A.sky.fog_density) * t;
}

// ---- weather ----
//
// Weather is a filter over the hour, not a second lighting setup. Every key in
// PIX_DAY is a clear sky; a cloud deck is what happens to that clear sky when
// something grey is put between it and the sun, and that is a transformation
// the same at every hour: the sun loses most of its strength and all of its
// colour, the dome flattens toward the grey of the cloud base, the fog thickens
// and the camera opens up to compensate. Doing it this way means a new hour
// keyframe is automatically correct in the rain, and an overcast noon and an
// overcast dusk stay as different from each other as the clear ones are.
//
// `overcast` is the cloud deck, 0..1. `rain` is how hard it is falling, which
// only ever exists under cloud; `wetness` lags rain so the ground stays shiny
// for a while after it stops.
struct pix_weather {
    float overcast;
    float rain;
    float wetness;
};

// The grey a thick cloud base scatters. Slightly blue, because it is still lit
// by a sky - a neutral grey here is what makes overcast renders look dead.
static const vec3 PIX_OVERCAST_GREY = { 0.44f, 0.47f, 0.52f };

// Bends a clear-sky setup toward that cloud deck. Applied after
// pix_daylight_at, so the hour is chosen first and the weather then acts on it.
static void pix_weather_apply(const pix_weather& w, pix_sun* sun, pix_sky* sky,
                              float* exposure) {
    float o = w.overcast < 0.0f ? 0.0f : (w.overcast > 1.0f ? 1.0f : w.overcast);
    if (o <= 0.0f) return;

    // The sun behind cloud is not a shadow-casting disc any more, it is the
    // brightest part of a uniform dome. Killing most of its radiance is what
    // makes the shadows go away, which is the single strongest cue that the
    // sky has closed over.
    float lum = sun->color.x * 0.2126f + sun->color.y * 0.7152f + sun->color.z * 0.0722f;
    vec3 grey_sun = v3(lum, lum, lum);
    sun->color = v3scale(pix__mix3(sun->color, grey_sun, o * 0.85f), 1.0f - o * 0.88f);

    // How bright the sky is at this hour at all.
    //
    // The deck colour above is the grey of a cloud base at *midday*. Used as
    // an absolute it turns an overcast midnight into an overcast noon: the
    // dome is a fixed mid grey, the exposure is still the one night needs, and
    // the result is half past nine at night lit like an afternoon. A cloud base
    // is only ever as bright as the sky lighting it, so the grey is scaled by
    // what this hour's clear sky was already doing - which makes
    // one deck colour work at every hour instead of only at the one it was
    // picked for.
    float sky_lum = sky->horizon.x * 0.2126f + sky->horizon.y * 0.7152f
                  + sky->horizon.z * 0.0722f;
    float k = sky_lum / 0.66f;              // 0.66 is that midday horizon
    if (k > 1.4f) k = 1.4f;
    if (k < 0.015f) k = 0.015f;
    vec3 grey = v3scale(PIX_OVERCAST_GREY, k);

    // The dome loses its gradient: overcast sky is nearly the same brightness
    // straight up as at the horizon, which is why it has no depth to it.
    vec3 deck_up   = v3scale(grey, 0.62f);
    vec3 deck_horiz = grey;
    float dim = 1.0f - o * 0.35f;
    sky->zenith  = pix__mix3(sky->zenith,  v3scale(deck_up, dim), o);
    sky->horizon = pix__mix3(sky->horizon, v3scale(deck_horiz, dim), o);
    sky->ground  = pix__mix3(sky->ground,  v3scale(grey, 0.18f * dim), o);

    // The irradiance the sun stopped delivering does not all vanish - cloud
    // scatters a good part of it back down as a huge soft source. Overcast is
    // dimmer than clear noon, but nothing like as dim as the sun term alone
    // would suggest, and this is the term that keeps a rainy scene readable.
    float up_lum = sky->diffuse_up.x * 0.2126f + sky->diffuse_up.y * 0.7152f + sky->diffuse_up.z * 0.0722f;
    vec3 flat_up = v3scale(PIX_OVERCAST_GREY, up_lum * 1.55f + 0.02f * k);
    sky->diffuse_up   = pix__mix3(sky->diffuse_up, flat_up, o);
    sky->diffuse_down = pix__mix3(sky->diffuse_down, v3scale(flat_up, 0.45f), o);

    // Rain is haze you can see through less far.
    vec3 fog_grey = pix__mix3(sky->fog, grey, 0.75f);
    sky->fog = pix__mix3(sky->fog, fog_grey, o);
    sky->fog_density *= 1.0f + o * 1.8f + w.rain * 1.6f;

    // and the eye opens up under it, the way it does walking out into a grey day
    *exposure *= 1.0f + o * 0.30f;
}

// How much artificial light the world should be running, 0 by day and 1 at
// night. Lamps and headlights fade in and out on this rather than switching,
// so dusk is a gradient instead of an event.
static float pix_night_factor(const pix_sun& sun) {
    // -0.08 is a few degrees below the horizon; by then the lamps are fully on
    float t = (0.14f - sun.direction.y) / 0.22f;
    return t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
}

// ---- the per-frame light list ----
//
// Lights are submitted every frame the same way draw calls are, then reduced to
// the MAX_SHADER_LIGHTS nearest the camera. Selection is a partial sort - the
// list is only ever a few hundred long, and taking the best 32 out of it costs
// far less than the fragments those 32 will be evaluated in.
struct pix_light_list {
    pix_light lights[MAX_SCENE_LIGHTS];
    size_t    count;

    // packed for upload: xyz position + w radius, rgb colour + w cos inner,
    // xyz direction + w cos outer
    float pos_radius[MAX_SHADER_LIGHTS * 4];
    float color_inner[MAX_SHADER_LIGHTS * 4];
    float dir_outer[MAX_SHADER_LIGHTS * 4];
    int   packed_count;
};

static void pix_lights_clear(pix_light_list& list) {
    list.count = 0;
    list.packed_count = 0;
}

static void pix_lights_add(pix_light_list& list, const pix_light& light) {
    if (list.count < MAX_SCENE_LIGHTS) list.lights[list.count++] = light;
}

// Chooses the lights this frame will actually shade with and packs them.
//
// Ordering is by how close a light's sphere of influence comes to the camera,
// with a small bonus for brightness - a bright lamp just out of range is worth
// more than a dim one underfoot, but only a little more.
//
// The weight on that bonus is the whole subtlety here. Punctual intensities
// are candela-like and run into the hundreds, so scoring them against a
// distance in metres one-for-one does not rank lights, it ranks their wattage.
// At half a unit per candela a bright light two hundred metres away outscores
// a dim one directly overhead, every slot in the frame goes to whatever is
// brightest somewhere across the map, and nothing near the camera lights
// anything at all. A hundredth of a candela per metre keeps the tie-break it
// was there for and takes away its ability to win outright.
static void pix_lights_pack(pix_light_list& list, vec3 eye) {
    static float score[MAX_SCENE_LIGHTS];
    static int   order[MAX_SCENE_LIGHTS];

    size_t n = list.count;
    for (size_t i = 0; i < n; i++) {
        const pix_light& l = list.lights[i];
        float dx = l.position.x - eye.x, dy = l.position.y - eye.y, dz = l.position.z - eye.z;
        float d = sqrtf(dx * dx + dy * dy + dz * dz);
        float reach = d - l.radius;               // negative once the eye is inside it
        float power = l.color.x + l.color.y + l.color.z;
        score[i] = reach - power * 0.01f;          // lower is more worth keeping
        order[i] = (int)i;
    }

    int keep = (int)(n < MAX_SHADER_LIGHTS ? n : MAX_SHADER_LIGHTS);
    // selection sort over just the first `keep`: n is small and this touches
    // each survivor once rather than sorting the whole list
    for (int a = 0; a < keep; a++) {
        int best = a;
        for (int b = a + 1; b < (int)n; b++)
            if (score[order[b]] < score[order[best]]) best = b;
        int tmp = order[a]; order[a] = order[best]; order[best] = tmp;
    }

    list.packed_count = 0;
    for (int a = 0; a < keep; a++) {
        const pix_light& l = list.lights[order[a]];
        int o = list.packed_count * 4;
        list.pos_radius[o + 0] = l.position.x;
        list.pos_radius[o + 1] = l.position.y;
        list.pos_radius[o + 2] = l.position.z;
        list.pos_radius[o + 3] = l.radius;
        list.color_inner[o + 0] = l.color.x;
        list.color_inner[o + 1] = l.color.y;
        list.color_inner[o + 2] = l.color.z;
        list.color_inner[o + 3] = l.cos_inner;
        list.dir_outer[o + 0] = l.direction.x;
        list.dir_outer[o + 1] = l.direction.y;
        list.dir_outer[o + 2] = l.direction.z;
        list.dir_outer[o + 3] = l.cos_outer;
        list.packed_count++;
    }
}
