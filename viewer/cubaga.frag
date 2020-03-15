// Based on https://github.com/KhronosGroup/glTF-Sample-Viewer/

#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 1) uniform sampler2D albedoSampler;
layout(binding = 2) uniform sampler2D emissiveSampler;
layout(binding = 3) uniform sampler2D normalsSampler;
layout(binding = 4) uniform sampler2D ormSampler;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inCamera;
layout(location = 2) in vec2 inTexCoord;
layout(location = 3) in vec4 inColorRoughness;
layout(location = 4) in vec4 inEmissiveMetallic;
layout(location = 5) in mat3 inTBN;

layout(location = 0) out vec4 outColor;

const float GAMMA = 2.2;
const float INV_GAMMA = 1.0 / GAMMA;
const float M_PI = 3.141592653589793;
const float c_MinReflectance = 0.04;

struct Light {
    vec3 direction;
    float range;
    vec3 color;
    float intensity;
    vec3 position;
    float innerConeCos;
    float outerConeCos;
    vec3 padding;
};

struct AngularInfo {
    float NdotL;                  // cos angle between normal and light direction
    float NdotV;                  // cos angle between normal and view direction
    float NdotH;                  // cos angle between normal and half vector
    float LdotH;                  // cos angle between light direction and half vector
    float VdotH;                  // cos angle between view direction and half vector
    vec3 padding;
};

struct MaterialInfo {
    float perceptualRoughness;    // roughness value, as authored by the model creator (input to shader)
    vec3 reflectance0;            // full reflectance color (normal incidence angle)

    float alphaRoughness;         // roughness mapped to a more linear change in the roughness (proposed by [2])
    vec3 diffuseColor;            // color contribution from diffuse lighting

    vec3 reflectance90;           // reflectance color at grazing angle
    vec3 specularColor;           // color contribution from specular lighting
};

vec3 LINEARtoSRGB(vec3 color) {
    return pow(color, vec3(INV_GAMMA));
}

vec4 SRGBtoLINEAR(vec4 srgbIn) {
    return vec4(pow(srgbIn.xyz, vec3(GAMMA)), srgbIn.w);
}

vec3 getNormal() {
    vec3 n = texture(normalsSampler, inTexCoord).rgb;
    n = normalize(inTBN * (2.0 * n - 1.0));
    return n;
}

AngularInfo getAngularInfo(vec3 pointToLight, vec3 normal, vec3 view) {
    vec3 n = normalize(normal);           // Outward direction of surface point
    vec3 v = normalize(view);             // Direction from surface point to view
    vec3 l = normalize(pointToLight);     // Direction from surface point to light
    vec3 h = normalize(l + v);            // Direction of the vector between l and v

    float NdotL = clamp(dot(n, l), 0.0, 1.0);
    float NdotV = clamp(dot(n, v), 0.0, 1.0);
    float NdotH = clamp(dot(n, h), 0.0, 1.0);
    float LdotH = clamp(dot(l, h), 0.0, 1.0);
    float VdotH = clamp(dot(v, h), 0.0, 1.0);

    return AngularInfo(
        NdotL,
        NdotV,
        NdotH,
        LdotH,
        VdotH,
        vec3(0, 0, 0)
    );
}

vec3 specularReflection(MaterialInfo materialInfo, AngularInfo angularInfo) {
    return materialInfo.reflectance0 + (materialInfo.reflectance90 - materialInfo.reflectance0) * pow(clamp(1.0 - angularInfo.VdotH, 0.0, 1.0), 5.0);
}

float visibilityOcclusion(MaterialInfo materialInfo, AngularInfo angularInfo) {
    float NdotL = angularInfo.NdotL;
    float NdotV = angularInfo.NdotV;
    float alphaRoughnessSq = materialInfo.alphaRoughness * materialInfo.alphaRoughness;

    float GGXV = NdotL * sqrt(NdotV * NdotV * (1.0 - alphaRoughnessSq) + alphaRoughnessSq);
    float GGXL = NdotV * sqrt(NdotL * NdotL * (1.0 - alphaRoughnessSq) + alphaRoughnessSq);

    float GGX = GGXV + GGXL;
    return GGX > 0.0 ? (0.5 / GGX) : 0.0;
}

float microfacetDistribution(MaterialInfo materialInfo, AngularInfo angularInfo) {
    float alphaRoughnessSq = materialInfo.alphaRoughness * materialInfo.alphaRoughness;
    float f = (angularInfo.NdotH * alphaRoughnessSq - angularInfo.NdotH) * angularInfo.NdotH + 1.0;
    return alphaRoughnessSq / (M_PI * f * f);
}

vec3 diffuse(MaterialInfo materialInfo) {
    return materialInfo.diffuseColor / M_PI;
}

vec3 getPointShade(vec3 pointToLight, MaterialInfo materialInfo, vec3 normal, vec3 view) {
    AngularInfo angularInfo = getAngularInfo(pointToLight, normal, view);

    if (angularInfo.NdotL > 0.0 || angularInfo.NdotV > 0.0) {
        vec3 F = specularReflection(materialInfo, angularInfo);
        float Vis = visibilityOcclusion(materialInfo, angularInfo);
        float D = microfacetDistribution(materialInfo, angularInfo);

        vec3 diffuseContrib = (1.0 - F) * diffuse(materialInfo);
        vec3 specContrib = F * Vis * D;

        return angularInfo.NdotL * (diffuseContrib + specContrib);
    }

    return vec3(0.0, 0.0, 0.0);
}

vec3 applyDirectionalLight(Light light, MaterialInfo materialInfo, vec3 normal, vec3 view) {
    vec3 pointToLight = -light.direction;
    vec3 shade = getPointShade(pointToLight, materialInfo, normal, view);
    return light.intensity * light.color * shade;
}

vec3 toneMapHejlRichard(vec3 color) {
    color = max(vec3(0.0), color - vec3(0.004));
    return (color*(6.2*color+.5))/(color*(6.2*color+1.7)+0.06);
}

void main() {
    vec3 f0 = vec3(0.04);

    vec3 ormSample = texture(ormSampler, inTexCoord).rgb;

    float perceptualRoughness = ormSample.g * inColorRoughness.w;
    float metallic = ormSample.b * inEmissiveMetallic.w;
    perceptualRoughness = clamp(perceptualRoughness, 0.0, 1.0);
    metallic = clamp(metallic, 0.0, 1.0);

    vec3 baseColor = SRGBtoLINEAR(texture(albedoSampler, inTexCoord)).rgb * inColorRoughness.rgb;
    vec3 diffuseColor = baseColor * (vec3(1.0) - f0) * (1.0 - metallic);
    vec3 specularColor = mix(f0, baseColor, metallic);

    float alphaRoughness = perceptualRoughness * perceptualRoughness;
    float reflectance = max(max(specularColor.r, specularColor.g), specularColor.b);
    vec3 specularEnvironmentR0 = specularColor.rgb;
    vec3 specularEnvironmentR90 = vec3(clamp(reflectance * 50.0, 0.0, 1.0));

    MaterialInfo materialInfo = MaterialInfo(
        perceptualRoughness,
        specularEnvironmentR0,
        alphaRoughness,
        diffuseColor,
        specularEnvironmentR90,
        specularColor
    );

    // LIGHTING

    vec3 color = vec3(0.0, 0.0, 0.0);
    vec3 normal = getNormal();
    vec3 view = normalize(inCamera - inPosition);

    Light defaultLight;
    defaultLight.direction = vec3(-0.7399, -0.6428, -0.1983);
    defaultLight.position = vec3(0, 0, 0);
    defaultLight.color = vec3(1,1,1);
    defaultLight.intensity = 5;
    defaultLight.innerConeCos = cos(0);
    defaultLight.outerConeCos = cos(M_PI / 4);
    defaultLight.range = -1;
    color += applyDirectionalLight(defaultLight, materialInfo, normal, view);

    float ao = ormSample.r;
    color = color * ao;

    vec3 emissive = SRGBtoLINEAR(texture(emissiveSampler, inTexCoord)).rgb * inEmissiveMetallic.rgb;
    color += emissive;
    outColor = vec4(toneMapHejlRichard(color), 1);
}
