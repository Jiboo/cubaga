#include "material.hpp"

#include <cmath>

#include "utils.hpp"
#include "context.hpp"
#include "texture.hpp"

void material::import(context &_ctx, const cgltf_material &_input) {
  if (_input.has_pbr_specular_glossiness)
    throw error("material has specular/glossiness");

  if (!_input.has_pbr_metallic_roughness)
    throw error("material isn't metallic/roughness");

  auto srgb_format = _ctx.opt_.raw_textures_ ? texture::R8G8B8A8_SRGB : texture::BC7_SRGB;
  auto unorm_format = _ctx.opt_.raw_textures_ ? texture::R8G8B8A8_UNORM : texture::BC7_UNORM;

  if (_input.pbr_metallic_roughness.base_color_texture.texture != nullptr)
    albedo_ = _ctx.import(_input.pbr_metallic_roughness.base_color_texture.texture, srgb_format);
  else
    albedo_ = 0xff;

  if (_input.emissive_texture.texture != nullptr)
    emissive_ = _ctx.import(_input.emissive_texture.texture, srgb_format);
  else
    emissive_ = 0xff;

  if (_input.normal_texture.texture != nullptr)
    normals_ = _ctx.import(_input.normal_texture.texture, unorm_format);
  else
    normals_ = 0xff;

  const bool has_occlusion = _input.occlusion_texture.texture != nullptr;
  const bool has_mr = _input.pbr_metallic_roughness.metallic_roughness_texture.texture != nullptr;
  if (has_occlusion && has_mr) {
    omr_ = _ctx.import(_input.pbr_metallic_roughness.metallic_roughness_texture.texture, unorm_format);
    if (_input.occlusion_texture.texture != _input.pbr_metallic_roughness.metallic_roughness_texture.texture) {
      auto occlusion = texture::import(*_input.occlusion_texture.texture, unorm_format);
      _ctx.textures_[omr_].merge(occlusion, {255, 0, 0, 0}, {0, 255, 255, 0});
    }
  }
  else if (has_occlusion) {
    omr_ = _ctx.import(_input.occlusion_texture.texture, unorm_format);
    _ctx.textures_[omr_].update({255, 0, 0, 0}, {0, 255, 255, 0});
  }
  else if (has_mr) {
    omr_ = _ctx.import(_input.pbr_metallic_roughness.metallic_roughness_texture.texture, unorm_format);
    _ctx.textures_[omr_].update({0, 255, 255, 0}, {255, 0, 0, 0});
  }
  else
    omr_ = 0xff;

  color_factor_[0] = round(_input.pbr_metallic_roughness.base_color_factor[0]);
  color_factor_[1] = round(_input.pbr_metallic_roughness.base_color_factor[1]);
  color_factor_[2] = round(_input.pbr_metallic_roughness.base_color_factor[2]);

  emissive_factor_[0] = round(_input.emissive_factor[0]);
  emissive_factor_[1] = round(_input.emissive_factor[1]);
  emissive_factor_[2] = round(_input.emissive_factor[2]);

  metallic_factor_ = round(_input.pbr_metallic_roughness.metallic_factor);
  roughness_factor_ = round(_input.pbr_metallic_roughness.roughness_factor);
}

void material::process(context &_ctx) {
  // Nothing to do?
}

void material::dump(context &_ctx, std::ostream &_os) {
  _os.put(albedo_);
  _os.put(emissive_);
  _os.put(normals_);
  _os.put(omr_);
  write_u8vec3(_os, color_factor_);
  _os.put(to_u8(roughness_factor_));
  write_u8vec3(_os, emissive_factor_);
  _os.put(to_u8(metallic_factor_));
}
