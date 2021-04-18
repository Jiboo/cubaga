#pragma once

#include "utils.hpp"

struct texture {
  enum target_format {
    BC7_SRGB = 0,
    BC7_UNORM = 1,
  };

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
