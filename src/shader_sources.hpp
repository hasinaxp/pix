#pragma once

// instanced: per-vertex pos/normal/uv (loc 0..2), per-instance model matrix (loc 3..6)
const char* VSHDER_BASIC =
    "#version 440 core\n"
    "layout(location=0) in vec3 aPos;\n"
    "layout(location=1) in vec3 aNormal;\n"
    "layout(location=2) in vec2 aUV;\n"
    "layout(location=3) in mat4 aModel;\n"   // consumes 3..6
    "uniform mat4 uViewProj;\n"
    "out vec3 vNormal;\n"
    "out vec2 vUV;\n"
    "void main() {\n"
    "    vec4 world = aModel * vec4(aPos, 1.0);\n"
    "    gl_Position = uViewProj * world;\n"
    "    vNormal = mat3(aModel) * aNormal;\n"
    "    vUV = vec2(aUV.x, 1.0 - aUV.y);\n" // OBJ/Blender V origin is bottom, GL texture V is top

    "}\n";

const char* FSHDER_BASIC =
    "#version 440 core\n"
    "in vec3 vNormal;\n"
    "in vec2 vUV;\n"
    "out vec4 frag;\n"
    "uniform sampler2D uTex;\n"
    "uniform vec3 uColor;\n"
    "void main() {\n"
    "    vec3 n = normalize(vNormal);\n"
    "    vec3 l = normalize(vec3(0.5, 0.85, 0.35));\n"
    "    float d = max(dot(n, l), 0.0);\n"
    "    vec3 base = texture(uTex, vUV).rgb * uColor;\n"
    "    frag = vec4(base * (0.35 + 0.75 * d), 1.0);\n"
    "}\n";
