#pragma once

#include "utils.hpp"
#include "hut/color.hpp"

struct texture {
  enum target_format {
    BC7_SRGB = 0,
    BC7_UNORM = 1,
    R8G8B8A8_SRGB = 2,
    R8G8B8A8_UNORM = 3,
  };

  static VkFormat vulkan_format(const target_format &_format) {
    switch (_format) {
      case BC7_SRGB: return VK_FORMAT_BC7_SRGB_BLOCK;
      case BC7_UNORM: return VK_FORMAT_BC7_UNORM_BLOCK;
      case R8G8B8A8_SRGB: return VK_FORMAT_R8G8B8A8_SRGB;
      case R8G8B8A8_UNORM: return VK_FORMAT_R8G8B8A8_UNORM;
      default: throw error("invalid format for vulkan_format");
    }
  }

  static hut::format_info format_info(const target_format &_format) { return hut::info(vulkan_format(_format)); }

  static texture import(uint8_t *_buffer, size_t _size, target_format _format);
  static texture import(const cgltf_texture &_input, target_format _format);

  glm::u16vec2 size_;
  unique_ptr<pixel_t[]> raw_;
  target_format format_;

  void import(context &_ctx, const cgltf_texture &_input, target_format _format);
  void update(pixel_t _mask, pixel_t _add);
  void merge(texture &_other, pixel_t _other_mask, pixel_t _this_mask);

  vector<data_item> compressed_mips_;

  void process(context &_ctx);
  size_t layout(size_t _offset);
  void dump(context &_ctx, std::ostream &_os);
};
