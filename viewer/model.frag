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

layout(binding = 1) uniform sampler2D albedo_sampler;
layout(binding = 2) uniform sampler2D emissive_sampler;
layout(binding = 3) uniform sampler2D normals_sampler;
layout(binding = 4) uniform sampler2D orm_sampler;

layout(binding = 5) uniform samplerCube irr_sampler;
layout(binding = 6) uniform samplerCube pre_sampler;
layout(binding = 7) uniform sampler2D brdflut_sampler;

layout(location = 0) in vec2 in_uv;
layout(location = 1) in vec3 in_pos;
layout(location = 2) in vec4 in_color_roughness;
layout(location = 3) in vec4 in_emissive_metallic;
layout(location = 4) in mat3 in_TBN;

layout(location = 0) out vec4 out_color;

// Encapsulate the various inputs used by the various functions in the shading equation
// We store values in this struct to simplify the integration of alternative implementations
// of the shading terms, outlined in the Readme.MD Appendix.
struct PBRInfo {
    float NdotL;                  // cos angle between normal and light direction
    float NdotV;                  // cos angle between normal and view direction
    float NdotH;                  // cos angle between normal and half vector
    float LdotH;                  // cos angle between light direction and half vector
    float VdotH;                  // cos angle between view direction and half vector
    float perceptualRoughness;    // roughness value, as authored by the model creator (input to shader)
    float metalness;              // metallic value at the surface
    vec3 reflectance0;            // full reflectance color (normal incidence angle)
    vec3 reflectance90;           // reflectance color at grazing angle
    float alphaRoughness;         // roughness mapped to a more linear change in the roughness (proposed by [2])
    vec3 diffuseColor;            // color contribution from diffuse lighting
    vec3 specularColor;           // color contribution from specular lighting
};

const float M_PI = 3.141592653589793;
const float c_MinRoughness = 0.04;
const float exposure = 4.5;
const float gamma = 2.2;
const float prefilteredCubeMipLevels = floor(log2(512)) - 2;

#define MANUAL_SRGB 1

vec3 Uncharted2Tonemap(vec3 color) {
    float A = 0.15;
    float B = 0.50;
    float C = 0.10;
    float D = 0.20;
    float E = 0.02;
    float F = 0.30;
    float W = 11.2;
    return ((color*(A*color+C*B)+D*E)/(color*(A*color+B)+D*F))-E/F;
}

vec4 tonemap(vec4 color) {
    vec3 outcol = Uncharted2Tonemap(color.rgb * exposure);
    outcol = outcol * (1.0f / Uncharted2Tonemap(vec3(11.2f)));
    return vec4(pow(outcol, vec3(1.0f / gamma)), color.a);
}

vec4 SRGBtoLINEAR(vec4 srgbIn) {
    #ifdef MANUAL_SRGB
        #ifdef SRGB_FAST_APPROXIMATION
            vec3 linOut = pow(srgbIn.xyz,vec3(2.2));
        #else //SRGB_FAST_APPROXIMATION
            vec3 bLess = step(vec3(0.04045),srgbIn.xyz);
            vec3 linOut = mix( srgbIn.xyz/vec3(12.92), pow((srgbIn.xyz+vec3(0.055))/vec3(1.055),vec3(2.4)), bLess );
        #endif //SRGB_FAST_APPROXIMATION
        return vec4(linOut, srgbIn.w);
    #else //MANUAL_SRGB
        return srgbIn;
    #endif //MANUAL_SRGB
}

// Calculation of the lighting contribution from an optional Image Based Light source.
// Precomputed Environment Maps are required uniform inputs and are computed as outlined in [1].
// See our README.md on Environment Maps [3] for additional discussion.
vec3 getIBLContribution(PBRInfo pbrInputs, vec3 n, vec3 reflection) {
    float lod = (pbrInputs.perceptualRoughness * prefilteredCubeMipLevels);
    vec3 brdf = (texture(brdflut_sampler, vec2(pbrInputs.NdotV, 1.0 - pbrInputs.perceptualRoughness))).rgb;
    vec3 diffuseLight = SRGBtoLINEAR(tonemap(texture(irr_sampler, n))).rgb;
    vec3 specularLight = SRGBtoLINEAR(tonemap(textureLod(pre_sampler, reflection, lod))).rgb;

    vec3 diffuse = diffuseLight * pbrInputs.diffuseColor;
    vec3 specular = specularLight * (pbrInputs.specularColor * brdf.x + brdf.y);

    return diffuse + specular;
}

// Basic Lambertian diffuse
// Implementation from Lambert's Photometria https://archive.org/details/lambertsphotome00lambgoog
// See also [1], Equation 1
vec3 diffuse(PBRInfo pbrInputs) {
    return pbrInputs.diffuseColor / M_PI;
}

// The following equation models the Fresnel reflectance term of the spec equation (aka F())
// Implementation of fresnel from [4], Equation 15
vec3 specularReflection(PBRInfo pbrInputs) {
    return pbrInputs.reflectance0 + (pbrInputs.reflectance90 - pbrInputs.reflectance0) * pow(clamp(1.0 - pbrInputs.VdotH, 0.0, 1.0), 5.0);
}

// This calculates the specular geometric attenuation (aka G()),
// where rougher material will reflect less light back to the viewer.
// This implementation is based on [1] Equation 4, and we adopt their modifications to
// alphaRoughness as input as originally proposed in [2].
float geometricOcclusion(PBRInfo pbrInputs) {
    float NdotL = pbrInputs.NdotL;
    float NdotV = pbrInputs.NdotV;
    float r = pbrInputs.alphaRoughness;

    float attenuationL = 2.0 * NdotL / (NdotL + sqrt(r * r + (1.0 - r * r) * (NdotL * NdotL)));
    float attenuationV = 2.0 * NdotV / (NdotV + sqrt(r * r + (1.0 - r * r) * (NdotV * NdotV)));
    return attenuationL * attenuationV;
}

// The following equation(s) model the distribution of microfacet normals across the area being drawn (aka D())
// Implementation from "Average Irregularity Representation of a Roughened Surface for Ray Reflection" by T. S. Trowbridge, and K. P. Reitz
// Follows the distribution function recommended in the SIGGRAPH 2013 course notes from EPIC Games [1], Equation 3.
float microfacetDistribution(PBRInfo pbrInputs) {
    float roughnessSq = pbrInputs.alphaRoughness * pbrInputs.alphaRoughness;
    float f = (pbrInputs.NdotH * roughnessSq - pbrInputs.NdotH) * pbrInputs.NdotH + 1.0;
    return roughnessSq / (M_PI * f * f);
}

// Gets metallic factor from specular glossiness workflow inputs
float convertMetallic(vec3 diffuse, vec3 specular, float maxSpecular) {
    float perceivedDiffuse = sqrt(0.299 * diffuse.r * diffuse.r + 0.587 * diffuse.g * diffuse.g + 0.114 * diffuse.b * diffuse.b);
    float perceivedSpecular = sqrt(0.299 * specular.r * specular.r + 0.587 * specular.g * specular.g + 0.114 * specular.b * specular.b);
    if (perceivedSpecular < c_MinRoughness) {
        return 0.0;
    }
    float a = c_MinRoughness;
    float b = perceivedDiffuse * (1.0 - maxSpecular) / (1.0 - c_MinRoughness) + perceivedSpecular - 2.0 * c_MinRoughness;
    float c = c_MinRoughness - perceivedSpecular;
    float D = max(b * b - 4.0 * a * c, 0.0);
    return clamp((-b + sqrt(D)) / (2.0 * a), 0.0, 1.0);
}

void main() {
    const vec4 ormSample = texture(orm_sampler, in_uv);
    //out_color = vec4(1) - vec4(ormSample.ggg, 0);

    const float perceptualRoughness = ormSample.g * in_color_roughness.a;
    const float metallic = ormSample.b * in_emissive_metallic.a;

    const vec4 albedoSample = texture(albedo_sampler, in_uv);
    const vec4 baseColor = albedoSample * SRGBtoLINEAR(vec4(in_color_roughness.rgb, 1.0));
    const vec3 f0 = vec3(0.04);
    const vec3 diffuseColor = baseColor.rgb * (vec3(1.0) - f0) * (1.0 - metallic);

    const float alphaRoughness = perceptualRoughness * perceptualRoughness;
    const vec3 specularColor = mix(f0, baseColor.rgb, metallic);
    const float reflectance = max(max(specularColor.r, specularColor.g), specularColor.b);
    const float reflectance90 = clamp(reflectance * 25.0, 0.0, 1.0);
    const vec3 specularEnvironmentR0 = specularColor.rgb;
    const vec3 specularEnvironmentR90 = vec3(1.0, 1.0, 1.0) * reflectance90;

    const vec3 normalsSample = texture(normals_sampler, in_uv).rgb;
    const vec3 n = normalize(in_TBN * (2.0 * normalsSample - 1.0));
    const vec3 v = normalize(ubo.camera_pos.xyz - in_pos);
    const vec3 l = normalize(ubo.sun_dir.xyz);
    const vec3 h = normalize(l+v);
    const vec3 reflection = -normalize(reflect(v, n));

    const float NdotL = clamp(dot(n, l), 0.001, 1.0);
    const float NdotV = clamp(abs(dot(n, v)), 0.001, 1.0);
    const float NdotH = clamp(dot(n, h), 0.0, 1.0);
    const float LdotH = clamp(dot(l, h), 0.0, 1.0);
    const float VdotH = clamp(dot(v, h), 0.0, 1.0);

    const PBRInfo pbrInputs = PBRInfo(
        NdotL,
        NdotV,
        NdotH,
        LdotH,
        VdotH,
        perceptualRoughness,
        metallic,
        specularEnvironmentR0,
        specularEnvironmentR90,
        alphaRoughness,
        diffuseColor,
        specularColor
    );

    const vec3 F = specularReflection(pbrInputs);
    const float G = geometricOcclusion(pbrInputs);
    const float D = microfacetDistribution(pbrInputs);

    const vec3 u_LightColor = vec3(1.0);
    const vec3 diffuseContrib = (1.0 - F) * diffuse(pbrInputs);
    const vec3 specContrib = F * G * D / (4.0 * NdotL * NdotV);
    const vec3 colorLight = NdotL * u_LightColor * (diffuseContrib + specContrib);
    const vec3 colorIBL = colorLight + getIBLContribution(pbrInputs, n, reflection);

    const float u_OcclusionStrength = 1.0f;
    const vec3 colorOccluded = mix(colorIBL, colorIBL * ormSample.r, u_OcclusionStrength);

    const vec4 emissiveSample = texture(emissive_sampler, in_uv);
    const vec3 emissive = emissiveSample.rgb * SRGBtoLINEAR(vec4(in_emissive_metallic.rgb, 1.0)).rgb;
    const vec3 colorEmissive = colorOccluded + emissive;

    out_color = vec4(colorEmissive, baseColor.a);

    switch (ubo.debug) {
        case 1: out_color = vec4(ormSample.rrr, 1); break;
        case 2: out_color = vec4(ormSample.ggg, 1); break;
        case 3: out_color = vec4(ormSample.bbb, 1); break;

        case 4: out_color = vec4(vec3(perceptualRoughness), 1); break;
        case 5: out_color = vec4(vec3(metallic), 1); break;

        case 6: out_color = albedoSample; break;
        case 7: out_color = baseColor; break;

        case 8: out_color = emissiveSample; break;
        case 9: out_color = vec4(emissive, 1); break;

        case 10: out_color = vec4(colorLight, 1); break;
        case 11: out_color = vec4(colorIBL, 1); break;
        case 12: out_color = vec4(colorOccluded, 1); break;
        case 13: out_color = vec4(colorEmissive, 1); break;

        case 14: out_color = vec4(normalsSample, 1); break;
        case 15: out_color = vec4(n, 1); break;

        case 16: out_color = vec4(diffuseContrib, 1); break;
        case 17: out_color = vec4(F, 1); break;
        case 18: out_color = vec4(vec3(G), 1); break;
        case 19: out_color = vec4(vec3(D), 1); break;
        case 20: out_color = vec4(specContrib, 1); break;
    }
}
