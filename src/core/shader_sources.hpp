#pragma once

// instanced: per-vertex pos/normal/uv (loc 0..2), per-instance model matrix (loc 3..6)
const char* VSHDER_BASIC = R"(#version 440 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aUV;
layout(location=3) in mat4 aModel;   // consumes 3..6
uniform mat4 uViewProj;
out vec3 vNormal;
out vec2 vUV;
void main() {
    vec4 world = aModel * vec4(aPos, 1.0);
    gl_Position = uViewProj * world;
    vNormal = mat3(aModel) * aNormal;
    vUV = vec2(aUV.x, 1.0 - aUV.y); // OBJ/Blender V origin is bottom, GL texture V is top
}
)";

const char* FSHDER_BASIC = R"(#version 440 core
in vec3 vNormal;
in vec2 vUV;
out vec4 frag;
uniform sampler2D uTex;
uniform vec3 uColor;
void main() {
    vec3 n = normalize(vNormal);
    vec3 l = normalize(vec3(0.5, 0.85, 0.35));
    float d = max(dot(n, l), 0.0);
    vec3 base = texture(uTex, vUV).rgb * uColor;
    frag = vec4(base * (0.35 + 0.75 * d), 1.0);
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
void main() {
    mat4 skin = uBones[int(aBoneIds.x)] * aWeights.x
              + uBones[int(aBoneIds.y)] * aWeights.y
              + uBones[int(aBoneIds.z)] * aWeights.z
              + uBones[int(aBoneIds.w)] * aWeights.w;
    vec4 world = uModel * (skin * vec4(aPos, 1.0));
    gl_Position = uViewProj * world;
    vNormal = mat3(uModel) * (mat3(skin) * aNormal);
    vUV = aUV;
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