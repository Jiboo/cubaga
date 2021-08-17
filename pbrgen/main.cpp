#include <filesystem>
#include <iostream>

#include "rdo_bc_encoder.h"

#include "hut/display.hpp"
#include "hut/offscreen.hpp"
#include "hut/pipeline.hpp"
#include "hut/color.hpp"

#include "hut_ktx2.hpp"

#include "cubaga_pbrgen_shaders_refl.hpp"

#ifdef CUBAGA_DEBUG
#include <fstream>
#include "stb_image_write.h"
#endif

#define CUBAGA_PBRGEN_DEBUG

using namespace hut;
using namespace std;
using namespace std::filesystem;
using namespace std::chrono;

shared_image load_envmap(display &_d, const path &_in) {
  std::ifstream is(_in, ios::binary);
  if (!is.is_open())
    throw std::runtime_error(sstream("can't open file") << _in);

  is.seekg(0, std::ios::end);
  size_t in_byte_size = is.tellg();
  is.seekg(0, std::ios::beg);

  unique_ptr<u8[]> in{new u8[in_byte_size]};
  is.read((char *) in.get(), in_byte_size);
  is.close();

  hut::ktx::load_params params;
  params.tiling_ = VK_IMAGE_TILING_OPTIMAL;
  auto result = hut::ktx::load(_d, span{in.get(), in_byte_size}, params);

  if (!result)
    throw std::runtime_error(sstream("couldn't load envmap ktx") << _in);

  return result.value();
}

constexpr size_t dfd_header_size = 1 + KHR_DF_WORD_SAMPLESTART;

// DFD generation taken from https://github.com/KhronosGroup/KTX-Software/tree/master/lib/dfdutils
void dfd_write_header(u32 *_dst, u32 _samples, u32 _bytes, format_suffix _suffix) {
  _dst[0] = sizeof(uint32_t) * (dfd_header_size + _samples * KHR_DF_WORD_SAMPLEWORDS);

  uint32_t* BDFD = _dst+1;
  BDFD[KHR_DF_WORD_VENDORID] =
      (KHR_DF_VENDORID_KHRONOS << KHR_DF_SHIFT_VENDORID) |
      (KHR_DF_KHR_DESCRIPTORTYPE_BASICFORMAT << KHR_DF_SHIFT_DESCRIPTORTYPE);
  BDFD[KHR_DF_WORD_VERSIONNUMBER] =
      (KHR_DF_VERSIONNUMBER_LATEST << KHR_DF_SHIFT_VERSIONNUMBER) |
      (((uint32_t)sizeof(uint32_t) *
        (KHR_DF_WORD_SAMPLESTART +
            _samples * KHR_DF_WORD_SAMPLEWORDS)
          << KHR_DF_SHIFT_DESCRIPTORBLOCKSIZE));
  BDFD[KHR_DF_WORD_MODEL] =
      ((KHR_DF_MODEL_RGBSDA << KHR_DF_SHIFT_MODEL) | /* Only supported model */
       (KHR_DF_PRIMARIES_BT709 << KHR_DF_SHIFT_PRIMARIES) | /* Assumed */
       (KHR_DF_FLAG_ALPHA_STRAIGHT << KHR_DF_SHIFT_FLAGS));
  if (_suffix == format_suffix::SRGB) {
    BDFD[KHR_DF_WORD_TRANSFER] |= KHR_DF_TRANSFER_SRGB << KHR_DF_SHIFT_TRANSFER;
  } else {
    BDFD[KHR_DF_WORD_TRANSFER] |= KHR_DF_TRANSFER_LINEAR << KHR_DF_SHIFT_TRANSFER;
  }
  BDFD[KHR_DF_WORD_TEXELBLOCKDIMENSION0] = 0; /* Only 1x1x1x1 texel blocks supported */
  BDFD[KHR_DF_WORD_BYTESPLANE0] = _bytes; /* bytesPlane0 = bytes, bytesPlane3..1 = 0 */
  BDFD[KHR_DF_WORD_BYTESPLANE4] = 0; /* bytesPlane7..5 = 0 */
}

u32 dfd_channel_flags(u32 _channel, format_suffix _suffix)
{
  switch (_suffix) {
    case format_suffix::UNORM: break;
    case format_suffix::SNORM: _channel |= KHR_DF_SAMPLE_DATATYPE_SIGNED; break;
    case format_suffix::USCALED: break;
    case format_suffix::SSCALED: _channel |= KHR_DF_SAMPLE_DATATYPE_SIGNED; break;
    case format_suffix::UINT: break;
    case format_suffix::SINT: _channel |= KHR_DF_SAMPLE_DATATYPE_SIGNED; break;
    case format_suffix::SFLOAT: _channel |= KHR_DF_SAMPLE_DATATYPE_FLOAT | KHR_DF_SAMPLE_DATATYPE_SIGNED; break;
    case format_suffix::UFLOAT: _channel |= KHR_DF_SAMPLE_DATATYPE_FLOAT; break;
    case format_suffix::SRGB:
      if (_channel == KHR_DF_CHANNEL_RGBSDA_ALPHA) {
        _channel |= KHR_DF_SAMPLE_DATATYPE_LINEAR;
      }
      break;
  }
  return _channel;
}

void dfd_write_sample(u32 *_dst, u32 _sample, u32 _bits, u32 _offset, format_suffix _suffix) {
  union {
    uint32_t i;
    float f;
  } lower, upper;
  constexpr u32 top_sample = 1;
  constexpr u32 bottom_sample = 1;

  uint32_t *sample = _dst + dfd_header_size + _sample * KHR_DF_WORD_SAMPLEWORDS;
  if (_sample == 3) _sample = KHR_DF_CHANNEL_RGBSDA_ALPHA;
  _sample = dfd_channel_flags(_sample, _suffix);

  sample[KHR_DF_SAMPLEWORD_BITOFFSET] =
      (_offset << KHR_DF_SAMPLESHIFT_BITOFFSET) |
      ((_bits - 1) << KHR_DF_SAMPLESHIFT_BITLENGTH) |
      (_sample << KHR_DF_SAMPLESHIFT_CHANNELID);

  sample[KHR_DF_SAMPLEWORD_SAMPLEPOSITION_ALL] = 0;

  switch (_suffix) {
    case format_suffix::UNORM:
    case format_suffix::SRGB:
    default:
      if (_bits > 32) {
        upper.i = 0xFFFFFFFFU;
      } else {
        upper.i = (uint32_t)((1U << _bits) - 1U);
      }
      lower.i = 0U;
      break;
    case format_suffix::SNORM:
      if (_bits > 32) {
        upper.i = 0x7FFFFFFF;
      } else {
        upper.i = top_sample ? (1U << (_bits - 1)) - 1 : (1U << _bits) - 1;
      }
      lower.i = ~upper.i;
      if (bottom_sample) lower.i += 1;
      break;
    case format_suffix::USCALED:
    case format_suffix::UINT:
      upper.i = bottom_sample ? 1U : 0U;
      lower.i = 0U;
      break;
    case format_suffix::SSCALED:
    case format_suffix::SINT:
      upper.i = bottom_sample ? 1U : 0U;
      lower.i = ~0U;
      break;
    case format_suffix::SFLOAT:
      upper.f = 1.0f;
      lower.f = -1.0f;
      break;
    case format_suffix::UFLOAT:
      upper.f = 1.0f;
      lower.f = 0.0f;
      break;
  }
  sample[KHR_DF_SAMPLEWORD_SAMPLELOWER] = lower.i;
  sample[KHR_DF_SAMPLEWORD_SAMPLEUPPER] = upper.i;
}

size_t dfd_write_unpacked(u32 *_dst, int _channels, int _bytes, format_suffix _suffix) {
  dfd_write_header(_dst, _channels, _channels * _bytes, _suffix);
  for (int sample = 0; sample < _channels; ++sample) {
    dfd_write_sample(_dst, sample, 8 * _bytes, 8 * sample * _bytes, _suffix);
  }
  return dfd_header_size + _channels * KHR_DF_WORD_SAMPLEWORDS;
}

static khr_df_model_e compModelMapping[] = {
  KHR_DF_MODEL_BC1A,   /*!< BC1, aka DXT1, no alpha. */
  KHR_DF_MODEL_BC1A,   /*!< BC1, aka DXT1, punch-through alpha. */
  KHR_DF_MODEL_BC2,    /*!< BC2, aka DXT2 and DXT3. */
  KHR_DF_MODEL_BC3,    /*!< BC3, aka DXT4 and DXT5. */
  KHR_DF_MODEL_BC4,    /*!< BC4. */
  KHR_DF_MODEL_BC5,    /*!< BC5. */
  KHR_DF_MODEL_BC6H,   /*!< BC6h HDR format. */
  KHR_DF_MODEL_BC7,    /*!< BC7. */
  KHR_DF_MODEL_ETC2,   /*!< ETC2 no alpha. */
  KHR_DF_MODEL_ETC2,   /*!< ETC2 punch-through alpha. */
  KHR_DF_MODEL_ETC2,   /*!< ETC2 independent alpha. */
  KHR_DF_MODEL_ETC2,   /*!< R11 ETC2 single-channel. */
  KHR_DF_MODEL_ETC2,   /*!< R11G11 ETC2 dual-channel. */
  KHR_DF_MODEL_ASTC,   /*!< ASTC. */
  KHR_DF_MODEL_ETC1S,  /*!< ETC1S. */
  KHR_DF_MODEL_PVRTC,  /*!< PVRTC(1). */
  KHR_DF_MODEL_PVRTC2  /*!< PVRTC2. */
};

static uint32_t compSampleCount[] = {
  1U, /*!< BC1, aka DXT1, no alpha. */
  1U, /*!< BC1, aka DXT1, punch-through alpha. */
  2U, /*!< BC2, aka DXT2 and DXT3. */
  2U, /*!< BC3, aka DXT4 and DXT5. */
  1U, /*!< BC4. */
  2U, /*!< BC5. */
  1U, /*!< BC6h HDR format. */
  1U, /*!< BC7. */
  1U, /*!< ETC2 no alpha. */
  2U, /*!< ETC2 punch-through alpha. */
  2U, /*!< ETC2 independent alpha. */
  1U, /*!< R11 ETC2 single-channel. */
  2U, /*!< R11G11 ETC2 dual-channel. */
  1U, /*!< ASTC. */
  1U, /*!< ETC1S. */
  1U, /*!< PVRTC. */
  1U  /*!< PVRTC2. */
};

static khr_df_model_channels_e compFirstChannel[] = {
  KHR_DF_CHANNEL_BC1A_COLOR,        /*!< BC1, aka DXT1, no alpha. */
  KHR_DF_CHANNEL_BC1A_ALPHAPRESENT, /*!< BC1, aka DXT1, punch-through alpha. */
  KHR_DF_CHANNEL_BC2_ALPHA,         /*!< BC2, aka DXT2 and DXT3. */
  KHR_DF_CHANNEL_BC3_ALPHA,         /*!< BC3, aka DXT4 and DXT5. */
  KHR_DF_CHANNEL_BC4_DATA,          /*!< BC4. */
  KHR_DF_CHANNEL_BC5_RED,           /*!< BC5. */
  KHR_DF_CHANNEL_BC6H_COLOR,        /*!< BC6h HDR format. */
  KHR_DF_CHANNEL_BC7_COLOR,         /*!< BC7. */
  KHR_DF_CHANNEL_ETC2_COLOR,        /*!< ETC2 no alpha. */
  KHR_DF_CHANNEL_ETC2_COLOR,        /*!< ETC2 punch-through alpha. */
  KHR_DF_CHANNEL_ETC2_ALPHA,        /*!< ETC2 independent alpha. */
  KHR_DF_CHANNEL_ETC2_RED,          /*!< R11 ETC2 single-channel. */
  KHR_DF_CHANNEL_ETC2_RED,          /*!< R11G11 ETC2 dual-channel. */
  KHR_DF_CHANNEL_ASTC_DATA,         /*!< ASTC. */
  KHR_DF_CHANNEL_ETC1S_RGB,         /*!< ETC1S. */
  KHR_DF_CHANNEL_PVRTC_COLOR,       /*!< PVRTC. */
  KHR_DF_CHANNEL_PVRTC2_COLOR       /*!< PVRTC2. */
};

static khr_df_model_channels_e compSecondChannel[] = {
  KHR_DF_CHANNEL_BC1A_COLOR,        /*!< BC1, aka DXT1, no alpha. */
  KHR_DF_CHANNEL_BC1A_ALPHAPRESENT, /*!< BC1, aka DXT1, punch-through alpha. */
  KHR_DF_CHANNEL_BC2_COLOR,         /*!< BC2, aka DXT2 and DXT3. */
  KHR_DF_CHANNEL_BC3_COLOR,         /*!< BC3, aka DXT4 and DXT5. */
  KHR_DF_CHANNEL_BC4_DATA,          /*!< BC4. */
  KHR_DF_CHANNEL_BC5_GREEN,         /*!< BC5. */
  KHR_DF_CHANNEL_BC6H_COLOR,        /*!< BC6h HDR format. */
  KHR_DF_CHANNEL_BC7_COLOR,         /*!< BC7. */
  KHR_DF_CHANNEL_ETC2_COLOR,        /*!< ETC2 no alpha. */
  KHR_DF_CHANNEL_ETC2_ALPHA,        /*!< ETC2 punch-through alpha. */
  KHR_DF_CHANNEL_ETC2_COLOR,        /*!< ETC2 independent alpha. */
  KHR_DF_CHANNEL_ETC2_RED,          /*!< R11 ETC2 single-channel. */
  KHR_DF_CHANNEL_ETC2_GREEN,        /*!< R11G11 ETC2 dual-channel. */
  KHR_DF_CHANNEL_ASTC_DATA,         /*!< ASTC. */
  KHR_DF_CHANNEL_ETC1S_RGB,         /*!< ETC1S. */
  KHR_DF_CHANNEL_PVRTC_COLOR,       /*!< PVRTC. */
  KHR_DF_CHANNEL_PVRTC2_COLOR       /*!< PVRTC2. */
};

static uint32_t compSecondChannelOffset[] = {
  0U,  /*!< BC1, aka DXT1, no alpha. */
  0U,  /*!< BC1, aka DXT1, punch-through alpha. */
  64U, /*!< BC2, aka DXT2 and DXT3. */
  64U, /*!< BC3, aka DXT4 and DXT5. */
  0U,  /*!< BC4. */
  64U, /*!< BC5. */
  0U,  /*!< BC6h HDR format. */
  0U,  /*!< BC7. */
  0U,  /*!< ETC2 no alpha. */
  0U,  /*!< ETC2 punch-through alpha. */
  64U, /*!< ETC2 independent alpha. */
  0U,  /*!< R11 ETC2 single-channel. */
  64U, /*!< R11G11 ETC2 dual-channel. */
  0U,  /*!< ASTC. */
  0U,  /*!< ETC1S. */
  0U,  /*!< PVRTC. */
  0U   /*!< PVRTC2. */
};

static uint32_t compChannelBits[] = {
  64U,  /*!< BC1, aka DXT1, no alpha. */
  64U,  /*!< BC1, aka DXT1, punch-through alpha. */
  64U,  /*!< BC2, aka DXT2 and DXT3. */
  64U,  /*!< BC3, aka DXT4 and DXT5. */
  64U,  /*!< BC4. */
  64U,  /*!< BC5. */
  128U, /*!< BC6h HDR format. */
  128U, /*!< BC7. */
  64U,  /*!< ETC2 no alpha. */
  64U,  /*!< ETC2 punch-through alpha. */
  64U,  /*!< ETC2 independent alpha. */
  64U,  /*!< R11 ETC2 single-channel. */
  64U,  /*!< R11G11 ETC2 dual-channel. */
  128U, /*!< ASTC. */
  64U,  /*!< ETC1S. */
  64U,  /*!< PVRTC. */
  64U   /*!< PVRTC2. */
};

static uint32_t compBytes[] = {
  8U,  /*!< BC1, aka DXT1, no alpha. */
  8U,  /*!< BC1, aka DXT1, punch-through alpha. */
  16U, /*!< BC2, aka DXT2 and DXT3. */
  16U, /*!< BC3, aka DXT4 and DXT5. */
  8U,  /*!< BC4. */
  16U, /*!< BC5. */
  16U, /*!< BC6h HDR format. */
  16U, /*!< BC7. */
  8U,  /*!< ETC2 no alpha. */
  8U,  /*!< ETC2 punch-through alpha. */
  16U, /*!< ETC2 independent alpha. */
  8U,  /*!< R11 ETC2 single-channel. */
  16U, /*!< R11G11 ETC2 dual-channel. */
  16U, /*!< ASTC. */
  8U,  /*!< ETC1S. */
  8U,  /*!< PVRTC. */
  8U   /*!< PVRTC2. */
};

/** Compression scheme, in Vulkan terms. */
enum class comp_scheme {
  BC1_RGB,       /*!< BC1, aka DXT1, no alpha. */
  BC1_RGBA,      /*!< BC1, aka DXT1, punch-through alpha. */
  BC2,           /*!< BC2, aka DXT2 and DXT3. */
  BC3,           /*!< BC3, aka DXT4 and DXT5. */
  BC4,           /*!< BC4. */
  BC5,           /*!< BC5. */
  BC6H,          /*!< BC6h HDR format. */
  BC7,           /*!< BC7. */
  ETC2_R8G8B8,   /*!< ETC2 no alpha. */
  ETC2_R8G8B8A1, /*!< ETC2 punch-through alpha. */
  ETC2_R8G8B8A8, /*!< ETC2 independent alpha. */
  EAC_R11,       /*!< R11 ETC2 single-channel. */
  EAC_R11G11,    /*!< R11G11 ETC2 dual-channel. */
  ASTC,          /*!< ASTC. */
  ETC1S,         /*!< ETC1S. */
  PVRTC,         /*!< PVRTC(1). */
  PVRTC2         /*!< PVRTC2. */
};

size_t dfd_write_comp(u32 *_dst, comp_scheme _comp, int _bwidth, int _bheight, int _bdepth, format_suffix _suffix) {
  uint32_t numSamples = compSampleCount[(int)_comp];
  uint32_t* BDFD = _dst + 1;
  uint32_t *sample;
  uint32_t channel;
  // Use union to avoid type-punning complaints from gcc optimizer
  // with -Wall.
  union {
      uint32_t i;
      float f;
  } lower, upper;

  BDFD[KHR_DF_WORD_VENDORID] =
          (KHR_DF_VENDORID_KHRONOS << KHR_DF_SHIFT_VENDORID) |
          (KHR_DF_KHR_DESCRIPTORTYPE_BASICFORMAT << KHR_DF_SHIFT_DESCRIPTORTYPE);
  BDFD[KHR_DF_WORD_VERSIONNUMBER] =
          (KHR_DF_VERSIONNUMBER_LATEST << KHR_DF_SHIFT_VERSIONNUMBER) |
          (((uint32_t)sizeof(uint32_t) *
          (KHR_DF_WORD_SAMPLESTART +
          numSamples * KHR_DF_WORD_SAMPLEWORDS)
          << KHR_DF_SHIFT_DESCRIPTORBLOCKSIZE));
  BDFD[KHR_DF_WORD_MODEL] =
          ((compModelMapping[(int)_comp] << KHR_DF_SHIFT_MODEL) |
           (KHR_DF_PRIMARIES_BT709 << KHR_DF_SHIFT_PRIMARIES) | /* Assumed */
          (KHR_DF_FLAG_ALPHA_STRAIGHT << KHR_DF_SHIFT_FLAGS));

  if (_suffix == format_suffix::SRGB) {
    BDFD[KHR_DF_WORD_TRANSFER] |= KHR_DF_TRANSFER_SRGB << KHR_DF_SHIFT_TRANSFER;
  } else {
    BDFD[KHR_DF_WORD_TRANSFER] |= KHR_DF_TRANSFER_LINEAR << KHR_DF_SHIFT_TRANSFER;
  }
  BDFD[KHR_DF_WORD_TEXELBLOCKDIMENSION0] =
          (_bwidth - 1) | ((_bheight - 1) << KHR_DF_SHIFT_TEXELBLOCKDIMENSION1) | ((_bdepth - 1) << KHR_DF_SHIFT_TEXELBLOCKDIMENSION2);
  /* bytesPlane0 = bytes, bytesPlane3..1 = 0 */
  BDFD[KHR_DF_WORD_BYTESPLANE0] = compBytes[(int)_comp];
  BDFD[KHR_DF_WORD_BYTESPLANE4] = 0; /* bytesPlane7..5 = 0 */

  sample = BDFD + KHR_DF_WORD_SAMPLESTART;
  channel = compFirstChannel[(int)_comp];
  channel = dfd_channel_flags(channel, _suffix);

  sample[KHR_DF_SAMPLEWORD_BITOFFSET] =
          (0 << KHR_DF_SAMPLESHIFT_BITOFFSET) |
          ((compChannelBits[(int)_comp] - 1) << KHR_DF_SAMPLESHIFT_BITLENGTH) |
          (channel << KHR_DF_SAMPLESHIFT_CHANNELID);

  sample[KHR_DF_SAMPLEWORD_SAMPLEPOSITION_ALL] = 0;
  switch (_suffix) {
    case format_suffix::UNORM: [[fallthrough]];
    case format_suffix::SRGB:
    default:
      upper.i = 0xFFFFFFFFU;
      lower.i = 0U;
      break;
    case format_suffix::SNORM:
      upper.i = 0x7FFFFFFF;
      lower.i = ~upper.i;
      break;
    case format_suffix::USCALED: [[fallthrough]];
    case format_suffix::UINT:
      upper.i = 1U;
      lower.i = 0U;
      break;
    case format_suffix::SSCALED: [[fallthrough]];
    case format_suffix::SINT:
      upper.i = 1U;
      lower.i = ~0U;
      break;
    case format_suffix::SFLOAT:
      upper.f = 1.0f;
      lower.f = -1.0f;
      break;
    case format_suffix::UFLOAT:
      upper.f = 1.0f;
      lower.f = 0.0f;
      break;
  }
  sample[KHR_DF_SAMPLEWORD_SAMPLELOWER] = lower.i;
  sample[KHR_DF_SAMPLEWORD_SAMPLEUPPER] = upper.i;

  if (compSampleCount[(int)_comp] > 1) {
    sample += KHR_DF_WORD_SAMPLEWORDS;
    channel = compSecondChannel[(int)_comp];
    channel = dfd_channel_flags(channel, _suffix);

    sample[KHR_DF_SAMPLEWORD_BITOFFSET] =
            (compSecondChannelOffset[(int)_comp] << KHR_DF_SAMPLESHIFT_BITOFFSET) |
            ((compChannelBits[(int)_comp] - 1) << KHR_DF_SAMPLESHIFT_BITLENGTH) |
            (channel << KHR_DF_SAMPLESHIFT_CHANNELID);

    sample[KHR_DF_SAMPLEWORD_SAMPLEPOSITION_ALL] = 0;

    sample[KHR_DF_SAMPLEWORD_SAMPLELOWER] = lower.i;
    sample[KHR_DF_SAMPLEWORD_SAMPLEUPPER] = upper.i;
  }
  return sizeof(uint32_t) * (1 + KHR_DF_WORD_SAMPLESTART + numSamples * KHR_DF_WORD_SAMPLEWORDS);
}

size_t dfd_write(u32 *_dst, VkFormat _format) {
  switch (_format) {
    case VK_FORMAT_R8G8_UNORM: return dfd_write_unpacked(_dst, 2, 1, format_suffix::UNORM);
    case VK_FORMAT_R8G8B8A8_SRGB: return dfd_write_unpacked(_dst, 4, 1, format_suffix::SRGB);
    case VK_FORMAT_R8G8B8A8_UNORM: return dfd_write_unpacked(_dst, 4, 1, format_suffix::UNORM);
    case VK_FORMAT_R16G16_SFLOAT: return dfd_write_unpacked(_dst, 2, 2, format_suffix::SFLOAT);
    case VK_FORMAT_R16G16B16A16_UNORM: return dfd_write_unpacked(_dst, 4, 2, format_suffix::UNORM);
    case VK_FORMAT_R16G16B16A16_SFLOAT: return dfd_write_unpacked(_dst, 4, 2, format_suffix::SFLOAT);
    case VK_FORMAT_R32G32B32A32_SFLOAT: return dfd_write_unpacked(_dst, 4, 4, format_suffix::SFLOAT);
    case VK_FORMAT_BC7_UNORM_BLOCK: return dfd_write_comp(_dst, comp_scheme::BC7, 4, 4, 1, format_suffix::UNORM);
    case VK_FORMAT_BC7_SRGB_BLOCK: return dfd_write_comp(_dst, comp_scheme::BC7, 4, 4, 1, format_suffix::SRGB);
    default:
      throw runtime_error(sstream("unknown dfd for VkFormat: ") << _format);
      return -1;
  }
}

struct cpu_image {
  VkFormat format_;
  u16vec2 size_;

  struct level {
    vector<unique_ptr<u8[]>> layers_;
  };
  vector<level> levels_;
  shared_image texture_;

  shared_image upload(display &_display) {
    if (texture_)
      return texture_;

    image_params iparams;
    iparams.format_ = format_;
    iparams.size_ = size_;
    iparams.levels_ = levels_.size();
    assert(!levels_.empty());
    iparams.layers_ = levels_[0].layers_.size();
    iparams.flags_ = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
    texture_ = std::make_shared<image>(_display, iparams);

    for (u16 level = 0; level < iparams.levels_; level++) {
      u16vec2 level_size = {size_.x >> level, size_.y >> level};
      for (u16 layer = 0; layer < iparams.layers_; layer++) {
        auto bit_stride = texture_->bpp() * level_size.x;
        assert(bit_stride >= 8);
        auto byte_stride = bit_stride / 8;
        auto src = span<const u8>(levels_[level].layers_[layer].get(), byte_stride * level_size.y);
        texture_->update({{0, 0, level_size}, level, layer}, src, byte_stride);
      }
    }

    return texture_;
  }

  void write(const path &_output_file) {
    ofstream os(_output_file, ios::binary | ios::trunc);
    if (!os.is_open())
      throw runtime_error(sstream("can't open for write: ") << _output_file);

    auto write_u32 = [&](u32 _in) -> void {
      os.put(_in & 0xFF);
      os.put((_in >> 8) & 0xFF);
      os.put((_in >> 16) & 0xFF);
      os.put((_in >> 24) & 0xFF);
    };

    auto write_u64 = [&](u64 _in) -> void {
      write_u32(_in & 0xFFFF'FFFF);
      write_u32((_in >> 32) & 0xFFFF'FFFF);
    };

    auto write_data = [&](const u8 *_expected, size_t _byte_size) -> void {
      os.write((const char*)_expected, _byte_size);
    };

    constexpr static u8 expected_ktx2_header_[12] = {
        0xAB, 0x4B, 0x54, 0x58, 0x20, 0x32, 0x30, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A
    };
    write_data(expected_ktx2_header_, 12);

    auto format_infos = info(format_);

    write_u32(format_);

    // type_size
    if (format_infos.compressed()) // "For formats whose Vulkan names have the suffix _BLOCK it must equal 1."
      write_u32(1); // type_size
    else if (format_infos.packed()) // "For formats with the suffix _PACKxx it must equal the value of xx/8."
      write_u32(format_infos.pack_ / 8);
    else if (format_ == VK_FORMAT_D16_UNORM_S8_UINT) // "For VK_FORMAT_D16_UNORM_S8_UINT, using the layout defined in this specification, the value will be 2"
      write_u32(2);
    else if (format_infos.target_ == format_target::DEPTH_STENCIL) // "and for the other combined depth/stencil formats the value will be 4."
      write_u32(4);
    else // "For unpacked formats, except combined depth/stencil formats, it must equal the number of bytes needed for a single component which can be derived from the format name."
      write_u32(format_infos.max_component_bits() / 8);

    write_u32(size_.x);
    write_u32(size_.y);
    write_u32(0); // pixel depth
    write_u32(0); // layers

    uint face_count = levels_[0].layers_.size();
    assert(face_count == 1 || face_count == 6);
    write_u32(face_count);
    write_u32(levels_.size());
    write_u32(0); // compression

    constexpr u32 dfd_max_size = dfd_header_size + 4 * KHR_DF_WORD_SAMPLEWORDS;
    u32 dfd[dfd_max_size];
    u32 dfd_byte_size = dfd_write(dfd, format_) * sizeof(u32);

    const size_t before_index = os.tellp();
    constexpr size_t index_byte_size = sizeof(u32) * 4 + sizeof(u64) * 2;
    const size_t after_index = before_index + index_byte_size;

    const size_t before_level_ranges = after_index;
    constexpr size_t level_range_byte_size = sizeof(u64) * 3;
    const size_t after_level_ranges = before_level_ranges + levels_.size() * level_range_byte_size;
    const size_t dfd_byte_offset = after_level_ranges;
    const size_t data_start_offset = align(dfd_byte_offset + dfd_byte_size, 16);

    size_t total_data_size = 0;
    for (int level = 0; level < levels_.size(); level++) {
      u16vec2 level_size = size_ >> u16(level);
      u32 bit_stride = level_size.x * info(format_).bpp();
      assert(bit_stride > 8);
      u32 byte_stride = bit_stride / 8;
      u32 layer_byte_size = byte_stride * level_size.y;
      u32 level_byte_size = layer_byte_size * face_count;
      total_data_size += level_byte_size;
    }

    const size_t data_end_offset = data_start_offset + total_data_size;

    write_u32(dfd_byte_offset); // dfd_byte_offset
    write_u32(dfd_byte_size); // dfd_byte_length
    write_u32(0); // kvd_byte_offset
    write_u32(0); // kvd_byte_length
    write_u64(0); // sgd_byte_offset
    write_u64(0); // sgd_byte_length

    assert(os.tellp() == after_index);

    size_t level_offset = data_end_offset;
    for (u16 level = 0; level < levels_.size(); level++) {
      u16vec2 level_size = size_ >> level;
      u32 bit_stride = level_size.x * info(format_).bpp();
      assert(bit_stride > 8);
      u32 byte_stride = bit_stride / 8;
      u32 layer_byte_size = byte_stride * level_size.y;
      u32 level_byte_size = layer_byte_size * face_count;
      level_offset -= level_byte_size;

      write_u64(level_offset);
      assert((level_offset % 16) == 0);
      write_u64(level_byte_size);
      write_u64(level_byte_size);
    }
    assert(os.tellp() == after_level_ranges);

    write_data(reinterpret_cast<const u8*>(dfd), dfd_byte_size);
    assert(os.tellp() <= data_start_offset);
    os.seekp(data_start_offset, std::ios::beg);

    for (int level = int(levels_.size()) - 1; level >= 0; level--) {
      u16vec2 level_size = size_ >> u16(level);
      u32 bit_stride = level_size.x * info(format_).bpp();
      assert(bit_stride > 8);
      u32 byte_stride = bit_stride / 8;
      u32 layer_byte_size = byte_stride * level_size.y;
      for (uint layer = 0; layer < face_count; layer++) {
        write_data(levels_[level].layers_[layer].get(), layer_byte_size);
      }
    }

    assert(os.tellp() == data_end_offset);
  }

  void rdo_enc(VkFormat _target) {
    auto start_total = high_resolution_clock::now();
    size_t levels = levels_.size();
    assert(levels > 0);
    size_t layers = levels_[0].layers_.size();
    assert(layers > 0);

    rdo_bc::rdo_bc_params rp;
    switch(_target) {
      case VK_FORMAT_BC7_SRGB_BLOCK: /*rp.m_dxgi_format = DXGI_FORMAT_BC7_UNORM_SRGB; break;*/
      case VK_FORMAT_BC7_UNORM_BLOCK: rp.m_dxgi_format = DXGI_FORMAT_BC7_UNORM; break;
      default:
        throw runtime_error("unknown target format");
    }

    for (uint level = 0; level < levels; level++) {
      auto start_level = high_resolution_clock::now();
      u16vec2 level_size = size_ >> u16(level);
      u32 bit_stride = level_size.x * info(format_).bpp();
      u32 byte_stride = bit_stride / 8;
      uint byte_size = level_size.y * byte_stride;
      for (uint layer = 0; layer < layers; layer++) {
        auto start_layer = high_resolution_clock::now();
        utils::image_u8 source_image(level_size.x, level_size.y);
        memcpy(source_image.get_pixels().data(), levels_[level].layers_[layer].get(), byte_size);
        rdo_bc::rdo_bc_encoder encoder;
        if (!encoder.init(source_image, rp)) {
          throw std::runtime_error("failed to init BC7 encoder");
        }
        if (!encoder.encode()) {
          throw std::runtime_error("failed to encode level in BC7");
        }
        levels_[level].layers_[layer].reset(new u8[encoder.get_total_blocks_size_in_bytes()]);
        memcpy(levels_[level].layers_[layer].get(), encoder.get_blocks(), encoder.get_total_blocks_size_in_bytes());

        std::cout << "\t\t\tencoded level " << level << " layer " << layer << " in " << (high_resolution_clock::now() - start_layer) << std::endl;
      }
      std::cout << "\t\tencoded level " << level << " in " << (high_resolution_clock::now() - start_level) << std::endl;
    }
    std::cout << "\tencoded in " << (high_resolution_clock::now() - start_total) << std::endl;

    format_ = _target;
  }

  void to_bc7() {
    if (format_ == VK_FORMAT_R8G8B8A8_SRGB)
      rdo_enc(VK_FORMAT_BC7_SRGB_BLOCK);
    else if (format_ == VK_FORMAT_R8G8B8A8_UNORM)
      rdo_enc(VK_FORMAT_BC7_UNORM_BLOCK);
    else
      throw runtime_error("can't convert to BC7");
  }
};

void gen_brdflut(const path &_output_file, display &_display, u16 _size) {
/*#ifdef HUT_ENABLE_RENDERDOC
  hut::renderdoc::frame_begin();
#endif*/

  auto start_total = high_resolution_clock::now();
  using brdflut_pipeline = pipeline<uint16_t, cubaga_pbrgen_shaders::genbrdflut_vert_spv_refl, cubaga_pbrgen_shaders::genbrdflut_frag_spv_refl>;

  cpu_image result;
  result.size_ = {_size, _size};
  result.format_ = VK_FORMAT_R8G8B8A8_UNORM;

  const uint levels = floor(std::log2(_size)) - 1;
  for (uint level = 0; level < levels; level++) {
    uint level_size = _size >> level;
    uint bit_stride = level_size * info(result.format_).bpp();
    uint byte_stride = bit_stride / 8;
    auto start_level = high_resolution_clock::now();

    image_params iparams;
    iparams.size_ = {level_size, level_size};
    iparams.format_ = result.format_;
    iparams.usage_ |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    auto img = std::make_shared<image>(_display, iparams);

    offscreen_params oparams;
    oparams.flags_ |= offscreen_params::flag::FMULTISAMPLING;
    auto ofs = offscreen(img, oparams);

    pipeline_params pparams;
    pparams.max_sets_ = 0;
    auto pipeline = std::make_shared<brdflut_pipeline>(ofs, pparams);

    _display.flush_staged(); // staging has to be explicitly flushed in offscreen mode

    ofs.draw([&](VkCommandBuffer _cb) {
      pipeline->bind_pipeline(_cb);
      pipeline->draw(_cb, 3, 1, 0, 0);
    });

    cpu_image::level lod;
    auto size = level_size * byte_stride;
    auto &pixels = lod.layers_.emplace_back(new u8[size]);
    ofs.download(std::span<u8>(pixels.get(), size), byte_stride);

#if defined(CUBAGA_PBRGEN_DEBUG)
    auto output = path(_output_file).replace_extension((sstream(".debug.level") << level << ".png").str()).u8string();
    stbi_write_png((char*)output.c_str(), level_size, level_size, info(result.format_).bpp() / 8, pixels.get(), byte_stride);
#endif

    result.levels_.emplace_back(std::move(lod));
    std::cout << "\t\tgenerated " << _output_file << " level " << level << " in " << (high_resolution_clock::now() - start_level) << std::endl;
  }

  result.to_bc7();
  result.write(_output_file);

  std::cout << "\tgenerated " << _output_file << " in " << (high_resolution_clock::now() - start_total) << std::endl;
/*#ifdef HUT_ENABLE_RENDERDOC
  hut::renderdoc::frame_end("brdflut");
#endif*/
}

void gen_irradiance(const path &_output_file, display &_display, const shared_image &_envmap, u16 _size) {
/*#ifdef HUT_ENABLE_RENDERDOC
  hut::renderdoc::frame_begin();
#endif*/
  auto start_total = high_resolution_clock::now();

  struct ubo_t {
    mat4 mvp;
    float roughness;
  };
  using shared_ubo = shared_ref<ubo_t>;

  using irradiance_pipeline = pipeline<uint16_t, cubaga_pbrgen_shaders::filtercube_vert_spv_refl, cubaga_pbrgen_shaders::irradiancecube_frag_spv_refl, const shared_ubo&, const shared_image&, const shared_sampler&>;

  auto b = _display.alloc_buffer(1024*1024);
  auto indices = b->allocate<uint16_t>(36);
  indices->set({
    0, 1, 2, 2, 3, 0,
    1, 5, 6, 6, 2, 1,
    7, 6, 5, 5, 4, 7,
    4, 0, 3, 3, 7, 4,
    4, 5, 1, 1, 0, 4,
    3, 2, 6, 6, 7, 3,
  });
  auto vertices = b->allocate<irradiance_pipeline::vertex>(8);
  vertices->set({
    irradiance_pipeline::vertex{{-.5, -.5, +.5}},
    irradiance_pipeline::vertex{{+.5, -.5, +.5}},
    irradiance_pipeline::vertex{{+.5, +.5, +.5}},
    irradiance_pipeline::vertex{{-.5, +.5, +.5}},
    irradiance_pipeline::vertex{{-.5, -.5, -.5}},
    irradiance_pipeline::vertex{{+.5, -.5, -.5}},
    irradiance_pipeline::vertex{{+.5, +.5, -.5}},
    irradiance_pipeline::vertex{{-.5, +.5, -.5}},
  });

  std::vector<glm::mat4> matrices = {
    lookAt(vec3{ 1, 0, 0}, vec3{0, 0, 0}, vec3{0, -1,  0}),
    lookAt(vec3{-1, 0, 0}, vec3{0, 0, 0}, vec3{0, -1,  0}),
    lookAt(vec3{ 0, 1, 0}, vec3{0, 0, 0}, vec3{0,  0,  1}),
    lookAt(vec3{ 0,-1, 0}, vec3{0, 0, 0}, vec3{0,  0, -1}),
    lookAt(vec3{ 0, 0, 1}, vec3{0, 0, 0}, vec3{0, -1,  0}),
    lookAt(vec3{ 0, 0,-1}, vec3{0, 0, 0}, vec3{0, -1,  0}),
  };

  ubo_t stack_ubo{
      glm::perspective((float)(M_PI / 2.0), 1.0f, 0.1f, 512.0f) * matrices[0],
      0.f
  };
  shared_ubo ubo = _display.alloc_ubo(b, stack_ubo);
  auto envmap_sampler = make_shared<sampler>(_display);

  cpu_image result;
  result.size_ = {_size, _size};
  result.format_ = VK_FORMAT_R8G8B8A8_SRGB;

  const uint levels = floor(std::log2(_size)) - 1;
  for (uint level = 0; level < levels; level++) {
    uint level_size = _size >> level;
    uint bit_stride = level_size * info(result.format_).bpp();
    uint byte_stride = bit_stride / 8;
    auto start_level = high_resolution_clock::now();

    image_params iparams;
    iparams.size_ = {level_size, level_size};
    iparams.format_ = result.format_;
    iparams.usage_ |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    auto img = std::make_shared<image>(_display, iparams);

    offscreen_params oparams;
    oparams.flags_ |= offscreen_params::flag::FDEPTH;
    auto ofs = offscreen(img, oparams);

    auto pipeline = std::make_unique<irradiance_pipeline>(ofs);
    pipeline->write(0, ubo, _envmap, envmap_sampler);

    cpu_image::level lod;
    for (uint layer = 0; layer < 6; layer++) {
      auto start_layer = high_resolution_clock::now();
      stack_ubo.mvp = glm::perspective((float)(M_PI / 2.0), 1.0f, 0.1f, (float)level_size) * matrices[layer];
      ubo->update_one(0, stack_ubo);
      _display.flush_staged(); // staging has to be explicitly flushed in offscreen mode

      ofs.draw([&](VkCommandBuffer _cb) {
        pipeline->draw(_cb, 0, indices, irradiance_pipeline::shared_instances{}, vertices);
      });

      auto size = byte_stride * level_size;
      auto &pixels = lod.layers_.emplace_back(new u8[size]);
      ofs.download(std::span<u8>(pixels.get(), size), byte_stride);

#if defined(CUBAGA_PBRGEN_DEBUG)
      auto output = path(_output_file).replace_extension((sstream(".debug.level") << level << ".layer" << layer << ".png").str()).u8string();
      stbi_write_png((char*)output.c_str(), level_size, level_size, info(result.format_).bpp() / 8, pixels.get(), byte_stride);
#endif
      std::cout << "\t\t\tgenerated " << _output_file << " level " << level << " layer " << layer << " in " << (high_resolution_clock::now() - start_layer) << std::endl;
    }
    result.levels_.emplace_back(std::move(lod));
    std::cout << "\t\tgenerated " << _output_file << " level " << level << " in " << (high_resolution_clock::now() - start_level) << std::endl;
  }

  result.to_bc7();
  result.write(_output_file);
  std::cout << "\tgenerated " << _output_file << " in " << (high_resolution_clock::now() - start_total) << std::endl;
/*#ifdef HUT_ENABLE_RENDERDOC
  hut::renderdoc::frame_end("gen_irradiance");
#endif*/
}

void gen_prefiltered(const path &_output_file, display &_display, const shared_image &_envmap, u16 _size) {
/*#ifdef HUT_ENABLE_RENDERDOC
  hut::renderdoc::frame_begin();
#endif*/
  auto start_total = high_resolution_clock::now();
  struct ubo_t {
    mat4 mvp;
    float roughness;
  };
  using shared_ubo = shared_ref<ubo_t>;

  using prefiltered_pipeline = pipeline<uint16_t, cubaga_pbrgen_shaders::filtercube_vert_spv_refl, cubaga_pbrgen_shaders::prefilterenvmap_frag_spv_refl, const shared_ubo&, const shared_image&, const shared_sampler&>;

  auto b = _display.alloc_buffer(1024*1024);
  auto indices = b->allocate<uint16_t>(36);
  indices->set({
    0, 1, 2, 2, 3, 0,
    1, 5, 6, 6, 2, 1,
    7, 6, 5, 5, 4, 7,
    4, 0, 3, 3, 7, 4,
    4, 5, 1, 1, 0, 4,
    3, 2, 6, 6, 7, 3,
  });
  auto vertices = b->allocate<prefiltered_pipeline::vertex>(8);
  vertices->set({
    // back
    prefiltered_pipeline::vertex{{-.5, -.5, +.5}},
    prefiltered_pipeline::vertex{{+.5, -.5, +.5}},
    prefiltered_pipeline::vertex{{+.5, +.5, +.5}},
    prefiltered_pipeline::vertex{{-.5, +.5, +.5}},
    // front
    prefiltered_pipeline::vertex{{-.5, -.5, -.5}},
    prefiltered_pipeline::vertex{{+.5, -.5, -.5}},
    prefiltered_pipeline::vertex{{+.5, +.5, -.5}},
    prefiltered_pipeline::vertex{{-.5, +.5, -.5}},
  });

  std::vector<glm::mat4> matrices = {
    lookAt(vec3{ 1, 0, 0}, vec3{0, 0, 0}, vec3{0, -1,  0}),
    lookAt(vec3{-1, 0, 0}, vec3{0, 0, 0}, vec3{0, -1,  0}),
    lookAt(vec3{ 0, 1, 0}, vec3{0, 0, 0}, vec3{0,  0,  1}),
    lookAt(vec3{ 0,-1, 0}, vec3{0, 0, 0}, vec3{0,  0, -1}),
    lookAt(vec3{ 0, 0, 1}, vec3{0, 0, 0}, vec3{0, -1,  0}),
    lookAt(vec3{ 0, 0,-1}, vec3{0, 0, 0}, vec3{0, -1,  0}),
  };

  ubo_t stack_ubo{
    glm::perspective((float)(M_PI / 2.0), 1.0f, 0.1f, 512.0f) * matrices[0],
    0.f
  };
  shared_ubo ubo = _display.alloc_ubo(b, stack_ubo);
  auto envmap_sampler = make_shared<sampler>(_display);

  cpu_image result;
  result.size_ = {_size, _size};
  result.format_ = VK_FORMAT_R8G8B8A8_SRGB;

  const uint levels = floor(std::log2(_size)) - 1;
  for (uint level = 0; level < levels; level++) {
    uint level_size = _size >> level;
    uint bit_stride = level_size * info(result.format_).bpp();
    uint byte_stride = bit_stride / 8;
    auto start_level = high_resolution_clock::now();

    image_params iparams;
    iparams.size_ = {level_size, level_size};
    iparams.format_ = result.format_;
    iparams.usage_ |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    auto img = std::make_shared<image>(_display, iparams);

    offscreen_params oparams;
    oparams.flags_ |= offscreen_params::flag::FDEPTH;
    auto ofs = offscreen(img, oparams);

    auto pipeline = std::make_unique<prefiltered_pipeline>(ofs);
    pipeline->write(0, ubo, _envmap, envmap_sampler);

    cpu_image::level lod;
    for (uint layer = 0; layer < 6; layer++) {
      auto start_layer = high_resolution_clock::now();
      stack_ubo.mvp = glm::perspective((float)(M_PI / 2.0), 1.0f, 0.001f, float(level_size)) * matrices[layer];
      stack_ubo.roughness = (float)level / (levels + 1);
      ubo->set(stack_ubo);
      _display.flush_staged(); // staging has to be explicitly flushed in offscreen mode

      ofs.draw([&](VkCommandBuffer _cb) {
        pipeline->draw(_cb, 0, indices, prefiltered_pipeline::shared_instances{}, vertices);
      });

      auto size = byte_stride * level_size;
      auto &pixels = lod.layers_.emplace_back(new u8[size]);
      ofs.download(span<u8>(pixels.get(), size), byte_stride);

#if defined(CUBAGA_PBRGEN_DEBUG)
      auto output = path(_output_file).replace_extension((sstream(".debug.level") << level << ".layer" << layer << ".png").str()).u8string();
      stbi_write_png((char*)output.c_str(), level_size, level_size, info(result.format_).bpp() / 8, pixels.get(), byte_stride);
#endif
      std::cout << "\t\t\tgenerated " << _output_file << " level " << level << " layer " << layer << " in " << (high_resolution_clock::now() - start_layer) << std::endl;
    }
    result.levels_.emplace_back(std::move(lod));
    std::cout << "\t\tgenerated " << _output_file << " level " << level << " in " << (high_resolution_clock::now() - start_level) << std::endl;
  }

  result.to_bc7();
  result.write(_output_file);
  std::cout << "\tgenerated " << _output_file << " in " << (high_resolution_clock::now() - start_total) << std::endl;
/*#ifdef HUT_ENABLE_RENDERDOC
  hut::renderdoc::frame_end("gen_prefiltered");
#endif*/
}

int main(int argc, char **argv) {
  auto start_total = high_resolution_clock::now();

  hut::display d("pbrgen");

  if (argc != 3)
    throw runtime_error(sstream("usage: ") << argv[0] << " <output path> <input path with envmaps>");

  path output_path = argv[1];
  path input_path = argv[2];

  if (!exists(input_path))
    throw std::runtime_error(sstream("input file ") << input_path << " doesn't exists");

  if (!exists(output_path))
    create_directories(output_path);

  gen_brdflut(output_path / "brdflut.ktx2", d, 512);

  for(auto& file : directory_iterator(input_path)) {
    if (file.is_regular_file() && file.path().extension() == ".ktx2") {
      auto tex_envmap = load_envmap(d, file.path());
      gen_irradiance(output_path / file.path().filename().replace_extension(".irr.ktx2"), d, tex_envmap, 64);
      gen_prefiltered(output_path / file.path().filename().replace_extension(".pre.ktx2"), d, tex_envmap, 512);
    }
  }

  std::cout << "generated everything in " << (high_resolution_clock::now() - start_total) << std::endl;
  return EXIT_SUCCESS;
}
