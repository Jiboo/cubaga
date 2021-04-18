// Based on https://github.com/KhronosGroup/glTF-Sample-Viewer/

#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 0) uniform UniformBufferObject {
    mat4 proj;
    mat4 view;
    vec4 camera_pos;
    vec4 sun_dir;
    int debug;
} ubo;

layout(location = 0) in uint in_v_dword0;
layout(location = 1) in uint in_v_dword1;
layout(location = 2) in uint in_v_dword2;

layout(location = 3) in vec4 in_i_color_roughness_r8g8b8a8_unorm;
layout(location = 4) in vec4 in_i_emissive_metallic_r8g8b8a8_unorm;
layout(location = 5) in mat4 in_i_transform;
layout(location = 9) in mat4 in_i_normal;

layout(location = 0) out vec2 out_uv;
layout(location = 1) out vec3 out_pos;
layout(location = 2) out flat vec4 out_color_roughness;
layout(location = 3) out flat vec4 out_emissive_metallic;
layout(location = 4) out flat mat3 out_TBN;

out gl_PerVertex {
    vec4 gl_Position;
};

float dequantize_snorm(uint _input, int _bitsize, int _bitoffset) {
    const uint mask = (1u << (_bitsize - 1)) - 1u;
    const int val = bitfieldExtract(int(_input), _bitoffset, _bitsize);
    return float(val) / mask;
}

float dequantize_unorm(uint _input, int _bitsize, int _bitoffset) {
    const uint mask = (1u << _bitsize) - 1u;
    const uint val = bitfieldExtract(_input, _bitoffset, _bitsize);
    return float(val) / mask;
}

void main() {
    vec3 position = vec3(
        dequantize_unorm(in_v_dword0, 11, 21),
        dequantize_unorm(in_v_dword0, 11, 10),
        dequantize_unorm(in_v_dword0, 10, 0)
    );
    vec3 normal = vec3(
        dequantize_snorm(in_v_dword1, 8, 24),
        dequantize_snorm(in_v_dword1, 8, 16),
        dequantize_snorm(in_v_dword1, 8, 8)
    );

    float tan_w = ((in_v_dword1 & 0x80u) == 0x80u) ? 1 : -1;
    vec4 tangent = vec4(
        dequantize_snorm(in_v_dword1, 7, 0),
        dequantize_snorm(in_v_dword2, 6, 26),
        dequantize_snorm(in_v_dword2, 6, 20),
        tan_w
    );

    out_uv = vec2(
        dequantize_unorm(in_v_dword2, 10, 10),
        dequantize_unorm(in_v_dword2, 10, 0)
    );

    vec4 pos = in_i_transform * vec4(position, 1);
    out_pos = pos.xyz / pos.w;
    gl_Position = ubo.proj * ubo.view * pos;

    vec3 normalW = normalize(vec3(in_i_normal * vec4(normal.xyz, 0)));
    vec3 tangentW = normalize(vec3(in_i_normal * vec4(tangent.xyz, 0)));
    vec3 bitangentW = cross(normalW, tangentW) * tangent.w;
    out_TBN = mat3(tangentW, bitangentW, normalW);

    out_color_roughness = in_i_color_roughness_r8g8b8a8_unorm;
    out_emissive_metallic = in_i_emissive_metallic_r8g8b8a8_unorm;
}
