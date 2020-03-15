#pragma once

#include "utils.hpp"

struct texture {
  static texture import(uint8_t *_buffer, size_t _size, bool _srgb);
  static texture import(const cgltf_texture &_input, bool _srgb);

  glm::uvec2 size_;
  unique_ptr<pixel_t[]> raw_;
  bool srgb_;

  void import(context &_ctx, const cgltf_texture &_input, bool _srgb);
  void update(pixel_t _mask, pixel_t _add);
  void merge(texture &_other, pixel_t _other_mask, pixel_t _this_mask);

  data_item compressed_mips_;

  void process(context &_ctx);
  size_t layout(size_t _offset);
  void dump(context &_ctx, std::ostream &_os);
};
