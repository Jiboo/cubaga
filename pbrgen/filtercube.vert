#version 450

layout(binding = 0) uniform UniformBufferObject {
	mat4 mvp;
	float roughness;
} ubo;

layout (location = 0) in vec3 in_v_pos;
layout (location = 0) out vec3 out_uvw;

out gl_PerVertex {
	vec4 gl_Position;
};

void main() {
	out_uvw = in_v_pos;
	gl_Position = ubo.mvp * vec4(in_v_pos, 1.0);
}
