// Generates an irradiance cube from an environment map using convolution

#version 450

layout (location = 0) in vec3 in_pos;
layout (location = 0) out vec4 out_col;
layout (binding = 1) uniform samplerCube samplerEnv;

#define PI 3.1415926535897932384626433832795
const float TWO_PI = PI * 2.0;
const float HALF_PI = PI * 0.5;
const float deltaPhi = TWO_PI / 180.0f;
const float deltaTheta = HALF_PI / 64.0f;

void main() {
	vec3 N = normalize(in_pos);
	vec3 up = vec3(0.0, 1.0, 0.0);
	vec3 right = normalize(cross(up, N));
	up = cross(N, right);

	vec3 color = vec3(0.0);
	uint sampleCount = 0u;
	for (float phi = 0.0; phi < TWO_PI; phi += deltaPhi) {
		for (float theta = 0.0; theta < HALF_PI; theta += deltaTheta) {
			vec3 tempVec = cos(phi) * right + sin(phi) * up;
			vec3 sampleVector = cos(theta) * N + sin(theta) * tempVec;
			color += texture(samplerEnv, sampleVector).rgb * cos(theta) * sin(theta);
			sampleCount++;
		}
	}
	out_col = vec4(PI * color / float(sampleCount), 1.0);
}
