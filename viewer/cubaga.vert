// Based on https://github.com/KhronosGroup/glTF-Sample-Viewer/

#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 0) uniform UniformBufferObject {
    mat4 proj;
    mat4 view;
    vec3 camera;
} ubo;

layout(location = 0) in uint inDword0;
layout(location = 1) in uint inDword1;
layout(location = 2) in uint inDword2;

layout(location = 3) in vec4 inInstanceColorRoughness;
layout(location = 4) in vec4 inInstanceEmissiveMetallic;
layout(location = 5) in mat4 inInstanceModelMat;
layout(location = 9) in mat4 inInstanceNormalMat;

layout(location = 0) out vec3 outPosition;
layout(location = 1) out vec3 outCamera;
layout(location = 2) out vec2 outTexCoord;
layout(location = 3) out vec4 outColorRoughness;
layout(location = 4) out vec4 outEmissiveMetallic;
layout(location = 5) out mat3 outTBN;

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
        dequantize_unorm(inDword0, 11, 21),
        dequantize_unorm(inDword0, 11, 10),
        dequantize_unorm(inDword0, 10, 0)
    );
    vec3 normal = vec3(
        dequantize_snorm(inDword1, 8, 24),
        dequantize_snorm(inDword1, 8, 16),
        dequantize_snorm(inDword1, 8, 8)
    );

    float tan_w = ((inDword1 & 0x80u) == 0x80u) ? 1 : -1;
    vec4 tangent = vec4(
        dequantize_snorm(inDword1, 7, 0),
        dequantize_snorm(inDword2, 6, 26),
        dequantize_snorm(inDword2, 6, 20),
        tan_w
    );

    outTexCoord = vec2(
        dequantize_unorm(inDword2, 10, 10),
        dequantize_unorm(inDword2, 10, 0)
    );

    vec4 pos = inInstanceModelMat * vec4(position, 1);
    outPosition = pos.xyz / pos.w;
    gl_Position = ubo.proj * ubo.view * pos;

    vec3 normalW = normalize(vec3(inInstanceNormalMat * vec4(normal.xyz, 0)));
    vec3 tangentW = normalize(vec3(inInstanceModelMat * vec4(tangent.xyz, 0)));
    vec3 bitangentW = cross(normalW, tangentW) * tangent.w;
    outTBN = mat3(tangentW, bitangentW, normalW);

    outColorRoughness = inInstanceColorRoughness;
    outEmissiveMetallic = inInstanceEmissiveMetallic;
    outCamera = ubo.camera;
}
