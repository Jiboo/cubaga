#include "texture.hpp"

#include <cmath>
#include <cstring>

#include "stb_image.h"
#include "stb_image_resize.h"

#include "bc7enc16.h"

#ifdef CUBAGA_DEBUG
#include <fstream>
#include "stb_image_write.h"
#endif

#include "context.hpp"

void stbi_free(pixel_t *_data) {
  stbi_image_free(_data);
}

void texture::import(context &_ctx, const cgltf_texture &_input, bool _srgb) {
  *this = std::move(import(_input, _srgb));
}

texture texture::import(uint8_t *_buffer, size_t _size, bool _srgb) {
  texture result {};
  int xraw, yraw, channels;
  if (stbi_info_from_memory(_buffer, _size, &xraw, &yraw, &channels) == 0)
    throw error("couldn't retrieve image info");

  if (!std::has_single_bit(unsigned(xraw)) || !std::has_single_bit(unsigned(yraw)))
    throw error("image size not power of 2");

  auto *decodedraw = reinterpret_cast<pixel_t*>(stbi_load_from_memory(_buffer, _size, &xraw, &yraw, &channels, 4));
  if (decodedraw == nullptr)
    throw error("couldn't decode image");

  uptr<pixel_t> decoded {decodedraw, &stbi_free};
  int xresized, yresized;

#if defined(CUBAGA_DEBUG) && 0
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
    if (_srgb && false) {
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

  result.srgb_ = _srgb;
  result.size_ = {xresized, yresized};

#if defined(CUBAGA_DEBUG) && 0
  stbi_write_png("debug.png", result.size_.x, result.size_.y, 4, result.raw_.get(), 0);
#endif

  return result;
}

texture texture::import(const cgltf_texture &_input, bool _srgb) {
  if (_input.image == nullptr)
    throw error("no image in texture");

  auto &buffer = *_input.image->buffer_view;
  if (buffer.buffer == nullptr)
    throw error("image buffer isn't loaded");

  if (_input.sampler != nullptr && (_input.sampler->wrap_s != 10497 || _input.sampler->wrap_t != 10497))
    throw error("sampler isn't in repeat mode");

  return import((uint8_t*)buffer.buffer->data + buffer.offset, buffer.size, _srgb);
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
  unsigned byte_size = 0;
  unsigned mip_levels = 0;
  glm::uvec2 size = size_;
  while (size.x >= 4 && size.y >= 4) {
    byte_size += size.x * size.y;
    size /= 2;
    mip_levels++;
  }

#if defined(CUBAGA_DEBUG) && 0
  stbi_write_png("debug.png", size_.x, size_.y, 4, raw_.get(), 0);

  std::ofstream debug_ktx("debug.ktx", std::ios::binary);
  uint8_t ktx_FileIdentifier[] = {0xAB, 0x4B, 0x54, 0x58, 0x20, 0x31, 0x31, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A};
  debug_ktx.write((char*)ktx_FileIdentifier, sizeof(ktx_FileIdentifier));
  uint32_t ktx_buffer = 0x04030201;
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // endianness
  ktx_buffer = 0;
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // glType
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // glTypeSize
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // glFormat
  ktx_buffer = 0x8E8C; //BC7
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // glInternalFormat
  ktx_buffer = 0x1907; //GL_RGB
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // glBaseInternalFormat
  ktx_buffer = size_.x;
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // pixelWidth
  ktx_buffer = size_.y;
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // pixelHeight
  ktx_buffer = 0;
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // pixelDepth
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // numberOfArrayElements
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // numberOfFaces
  ktx_buffer = mip_levels;
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // numberOfMipmapLevels
  ktx_buffer = 0;
  debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // bytesOfKeyValueData
#endif

  compressed_mips_.data_.reset(new uint8_t[byte_size]);
  compressed_mips_.size_ = byte_size;
  size = size_;
  byte_size = 0;
  unique_ptr<pixel_t[]> resized {new pixel_t[size.x * size.y]};

  while (size.x >= 4 && size.y >= 4) {
    unsigned mip_byte_size = size.x * size.y;
    uint8_t *mip_target = compressed_mips_.data_.get() + byte_size;

    int resize_result;
    if (size != size_) {
      if (srgb_ && false) {
        resize_result = stbir_resize_uint8_srgb(reinterpret_cast<uint8_t *>(raw_.get()), size_.x, size_.y, 0,
                                                reinterpret_cast<uint8_t *>(resized.get()), size.x, size.y, 0, 4, 0, 0);
      } else {
        resize_result = stbir_resize_uint8(reinterpret_cast<uint8_t *>(raw_.get()), size_.x, size_.y, 0,
                                           reinterpret_cast<uint8_t *>(resized.get()), size.x, size.y, 0, 4);
      }
      if (resize_result == 0)
        throw error("couldn't resize mip level");
    }
    else {
      memcpy(resized.get(), raw_.get(), sizeof(pixel_t) * size.x * size.y);
    }

#if defined(CUBAGA_DEBUG) && 0
    stbi_write_png("debug.png", size.x, size.y, 4, resized.get(), 0);
#endif

    bc7enc16_compress_block_params bparams;
    bc7enc16_compress_block_params_init(&bparams);
    bparams.m_uber_level = BC7ENC16_MAX_UBER_LEVEL;
    bparams.m_try_least_squares = BC7ENC16_FALSE;
    bparams.m_mode1_partition_estimation_filterbank = BC7ENC16_FALSE;
    bc7enc16_compress_block_init();

    glm::uvec2 block_size = size / 4;
    for (auto block_x = 0; block_x < block_size.x; block_x++) {
      for (auto block_y = 0; block_y < block_size.y; block_y++) {
        unsigned block_offset = block_x + block_y * block_size.x;
        uint8_t *block_target = mip_target + block_offset * 16;
        pixel_t block_source[16];
        for (unsigned source_x = 0; source_x < 4; source_x++) {
          for (unsigned source_y = 0; source_y < 4; source_y++) {
            unsigned offset_x = block_x * 4 + source_x;
            unsigned offset_y = block_y * 4 * size.x + source_y * size.x;
            block_source[source_x + source_y * 4] = resized[offset_x + offset_y];
          }
        }
#if defined(CUBAGA_DEBUG) && 0
        stbi_write_png("tile.png", 4, 4, 4, block_source, 0);
#endif

        bc7enc16_compress_block(block_target, &block_source, &bparams);
      }
    }

#if defined(CUBAGA_DEBUG) && 0
    ktx_buffer = mip_byte_size;
    debug_ktx.write((char*)&ktx_buffer, sizeof(ktx_buffer)); // imageSize
    debug_ktx.write((char*)mip_target, mip_byte_size);
#endif

    byte_size += mip_byte_size;
    size /= 2;
  }
}

size_t texture::layout(size_t _offset) {
  compressed_mips_.offset_ = _offset;
  return compressed_mips_.size_;
}

void texture::dump(context &_ctx, std::ostream &_os) {
  uint8_t powsize_x = (uint8_t)std::log2(size_.x) & 0xF;
  uint8_t powsize_y = (uint8_t)std::log2(size_.y) & 0xF;
  uint8_t powsize = (powsize_x << 4) | powsize_y;
  _os.put(powsize);
  write_uleb128(_os, compressed_mips_.size_);
  write_uleb128(_os, compressed_mips_.offset_);

  memcpy(_ctx.data_.get() + compressed_mips_.offset_, compressed_mips_.data_.get(), compressed_mips_.size_);
}
