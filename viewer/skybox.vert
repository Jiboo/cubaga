#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 0) uniform UniformBufferObject {
    mat4 proj;
    mat4 view;
    vec4 camera_pos;
    vec4 sun_dir;
    int debug;
} ubo;

layout(location = 0) in vec3 in_v_pos;
layout(location = 2) in mat4 in_i_transform;

layout(location = 0) out vec3 out_uvw;

out gl_PerVertex {
    vec4 gl_Position;
};

void main() {
    gl_Position = ubo.proj * ubo.view * in_i_transform * vec4(in_v_pos, 1.0);
    out_uvw = in_v_pos;
}
