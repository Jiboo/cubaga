#include "texture.hpp"

#include <cmath>
#include <cstring>

#include "stb_image.h"
#include "stb_image_resize.h"

#include "rdo_bc_encoder.h"

#ifdef CUBAGA_DEBUG
#include <fstream>
#include "stb_image_write.h"
#endif

#include "context.hpp"

void stbi_free(pixel_t *_data) {
  stbi_image_free(_data);
}

void texture::import(context &_ctx, const cgltf_texture &_input, target_format _format) {
  *this = std::move(import(_input, _format));
}

texture texture::import(uint8_t *_buffer, size_t _size, target_format _format) {
  texture result {};
  int xraw, yraw, channels;
  if (stbi_info_from_memory(_buffer, _size, &xraw, &yraw, &channels) == 0)
    throw error("couldn't retrieve image info");

  if (!std::has_single_bit(unsigned(xraw)) || !std::has_single_bit(unsigned(yraw)))
    throw error("image size not power of 2");

  if ((_format == BC7_SRGB || _format == BC7_UNORM) && channels != 3)
    throw error("unexpected number of channels");
  /*if ((_format == BC5_UNORM) && channels != 2)
    throw error("unexpected number of channels");*/

  auto *decodedraw = reinterpret_cast<pixel_t*>(stbi_load_from_memory(_buffer, _size, &xraw, &yraw, &channels, 4));
  if (decodedraw == nullptr)
    throw error("couldn't decode image");

  uptr<pixel_t> decoded {decodedraw, &stbi_free};
  int xresized, yresized;

#if defined(CUBAGA_PACKAGER_DEBUG) && 1
  stbi_write_png("debug.png", xraw, yraw, 4, decoded.get(), 0);
#endif

  if (xraw > 1024 || yraw > 1024) {
    if (xraw > yraw) {
      xresized = std::min(1024, xraw);
      yresized = std::floor(float(xresized) / float(xraw) * float(yraw));
    }
    else {
      yresized = std::min(1024, yraw);
      xresized = std::floor(float(yresized) / float(yraw) * float(xraw));
    }
    result.raw_.reset(new pixel_t[yraw * xraw]);

    int resize_result;
    if (_format == BC7_SRGB && false) {
      resize_result = stbir_resize_uint8_srgb(reinterpret_cast<uint8_t *>(decoded.get()), xraw, yraw, 0,
          reinterpret_cast<uint8_t *>(result.raw_.get()), xresized, yresized, 0, 4, 0, 0);
    }
    else {
      resize_result = stbir_resize_uint8(reinterpret_cast<uint8_t *>(decoded.get()), xraw, yraw, 0,
          reinterpret_cast<uint8_t *>(result.raw_.get()), xresized, yresized, 0, 4);
    }
    if (resize_result == 0)
      throw error("couldn't resize image");
  }
  else {
    xresized = xraw;
    yresized = yraw;
    result.raw_.reset(decoded.release());
  }

  result.format_ = _format;
  result.size_ = {xresized, yresized};

#if defined(CUBAGA_PACKAGER_DEBUG) && 1
  stbi_write_png("debug.png", result.size_.x, result.size_.y, 4, result.raw_.get(), 0);
#endif

  return result;
}

texture texture::import(const cgltf_texture &_input, target_format _format) {
  if (_input.image == nullptr)
    throw error("no image in texture");

  auto &buffer = *_input.image->buffer_view;
  if (buffer.buffer == nullptr)
    throw error("image buffer isn't loaded");

  if (_input.sampler != nullptr && (_input.sampler->wrap_s != 10497 || _input.sampler->wrap_t != 10497))
    throw error("sampler isn't in repeat mode");

  return import((uint8_t*)buffer.buffer->data + buffer.offset, buffer.size, _format);
}

void texture::merge(texture &_other, pixel_t _other_mask, pixel_t _this_mask) {
  if (_other.size_ != size_)
    throw error("occlusion and MR textures size differ");

  for (unsigned x = 0; x < size_.x; x++) {
    for (unsigned y = 0; y < size_.y; y++) {
      pixel_t other_pixel = _other.raw_[x + y * size_.x];
      pixel_t &this_pixel = raw_[x + y * size_.x];
      other_pixel &= _other_mask;
      this_pixel &= _this_mask;
      this_pixel |= other_pixel;
    }
  }
}

void texture::update(pixel_t _mask, pixel_t _add) {
  for (unsigned x = 0; x < size_.x; x++) {
    for (unsigned y = 0; y < size_.y; y++) {
      pixel_t &pixel = raw_[x + y * size_.x];
      pixel &= _mask;
      pixel |= _add;
    }
  }
}

void texture::process(context &_ctx) {
  const uint min_extent = std::min(size_.x, size_.y);
  const uint levels = floor(std::log2(min_extent)) - 1;
  unique_ptr<pixel_t[]> resized {new pixel_t[size_.x * size_.y]};

  for (uint level = 0; level < levels; level++) {
    data_item mip_data_item;
    glm::u16vec2 level_size = {size_.x >> level, size_.y >> level};
    mip_data_item.size_ = level_size.x * level_size.y;
    mip_data_item.data_.reset(new uint8_t[mip_data_item.size_]);

    int resize_result;
    if (level_size != size_) {
      if (format_ == BC7_SRGB && false) {
        resize_result = stbir_resize_uint8_srgb(reinterpret_cast<uint8_t *>(raw_.get()), size_.x, size_.y, 0,
                                                reinterpret_cast<uint8_t *>(resized.get()), level_size.x, level_size.y, 0, 4, STBIR_ALPHA_CHANNEL_NONE, 0);
      } else {
        resize_result = stbir_resize_uint8(reinterpret_cast<uint8_t *>(raw_.get()), size_.x, size_.y, 0,
                                           reinterpret_cast<uint8_t *>(resized.get()), level_size.x, level_size.y, 0, 4);
      }
      if (resize_result == 0)
        throw error("couldn't resize mip level");
    }
    else {
      memcpy(resized.get(), raw_.get(), sizeof(pixel_t) * level_size.x * level_size.y);
    }

#if defined(CUBAGA_PACKAGER_DEBUG) && 1
    stbi_write_png("debug.png", level_size.x, level_size.y, 4, resized.get(), 0);
#endif

    utils::image_u8 source_image(level_size.x, level_size.y);
    memcpy(source_image.get_pixels().data(), resized.get(), sizeof(pixel_t) * level_size.x * level_size.y);

    rdo_bc::rdo_bc_params rp;
    switch (format_) {
      case BC7_SRGB: rp.m_dxgi_format = /*DXGI_FORMAT_BC7_UNORM_SRGB*/ DXGI_FORMAT_BC7_UNORM; break;
      case BC7_UNORM: rp.m_dxgi_format = DXGI_FORMAT_BC7_UNORM; break;
    }

    rdo_bc::rdo_bc_encoder encoder;
    if (!encoder.init(source_image, rp)) {
      throw std::runtime_error("failed to init BC7 encoder");
    }
    if (!encoder.encode()) {
      throw std::runtime_error("failed to encode mip in BC7");
    }

    assert(encoder.get_total_blocks_size_in_bytes() == mip_data_item.size_);
    memcpy(mip_data_item.data_.get(), encoder.get_blocks(), encoder.get_total_blocks_size_in_bytes());

#if defined(CUBAGA_PACKAGER_DEBUG) && 1
    utils::image_u8 unpacked_image;
    encoder.unpack_blocks(unpacked_image);
    stbi_write_png("debug.png", level_size.x, level_size.y, 4, unpacked_image.get_pixels().data(), 0);
#endif

    compressed_mips_.emplace_back(std::move(mip_data_item));
  }
}

size_t texture::layout(size_t _offset) {
  for (auto &mip : compressed_mips_) {
    mip.offset_ = _offset = align(_offset, 16);
    _offset += mip.size_;
  }

  return _offset;
}

void texture::dump(context &_ctx, std::ostream &_os) {
  _os.put(std::log2(size_.x));
  _os.put(std::log2(size_.y));
  _os.put(format_);
  _os.put(0); // reserved

  for (auto &mip : compressed_mips_) {
    write_u32(_os, mip.offset_);
    write_u32(_os, mip.size_);

    memcpy(_ctx.data_.get() + mip.offset_, mip.data_.get(), mip.size_);
  }
}
