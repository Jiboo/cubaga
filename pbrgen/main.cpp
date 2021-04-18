#include <filesystem>

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

//#define CUBAGA_PBRGEN_DEBUG

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
        auto stride = texture_->bpp() * level_size.x;
        auto src = span<const u8>(levels_[level].layers_[layer].get(), stride * level_size.y);
        texture_->update({{0, 0, level_size}, level, layer}, src, stride);
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

    write_u32(format_);
    write_u32(0); // type_size
    write_u32(size_.x);
    write_u32(size_.y);
    write_u32(1); // pixel depth
    write_u32(1); // layers

    uint face_count = levels_[0].layers_.size();
    assert(face_count == 1 || face_count == 6);
    write_u32(face_count);
    write_u32(levels_.size());
    write_u32(0); // compression

    {
      const size_t before_index = os.tellp();
      constexpr size_t index_byte_size = sizeof(u32) * 4 + sizeof(u64) * 2;
      const size_t after_index = before_index + index_byte_size;

      write_u32(after_index); // dfd_byte_offset
      write_u32(0); // dfd_byte_length
      write_u32(after_index); // kvd_byte_offset
      write_u32(0); // kvd_byte_length
      write_u64(after_index); // sgd_byte_offset
      write_u64(0); // sgd_byte_length

      assert(os.tellp() == after_index);
    }

    {
      const size_t before_level_ranges = os.tellp();
      constexpr size_t level_range_byte_size = sizeof(u64) * 3;
      const size_t after_level_ranges = before_level_ranges + levels_.size() * level_range_byte_size;
      size_t current_offset = after_level_ranges;

      for (u16 level = 0; level < levels_.size(); level++) {
        u16vec2 level_size = size_ >> level;
        u32 byte_stride = level_size.x * format_bpp(format_);
        u32 layer_byte_size = byte_stride * level_size.y;
        u32 level_byte_size = layer_byte_size * face_count;

        write_u64(current_offset);
        write_u64(level_byte_size);
        write_u64(level_byte_size);

        current_offset += level_byte_size;
      }
      assert(os.tellp() == after_level_ranges);
    }

    for (u16 level = 0; level < levels_.size(); level++) {
      u16vec2 level_size = size_ >> level;
      u32 byte_stride = level_size.x * format_bpp(format_);
      u32 layer_byte_size = byte_stride * level_size.y;
      for (u16 layer = 0; layer < face_count; layer++) {
        write_data(levels_[level].layers_[layer].get(), layer_byte_size);
      }
    }
  }

  void rdo_enc(VkFormat _target) {
    auto start_total = high_resolution_clock::now();
    size_t levels = levels_.size();
    assert(levels > 0);
    size_t layers = levels_[0].layers_.size();
    assert(layers > 0);

    rdo_bc::rdo_bc_params rp;
    switch(_target) {
      case VK_FORMAT_BC7_SRGB_BLOCK: rp.m_dxgi_format = /*DXGI_FORMAT_BC7_UNORM_SRGB*/DXGI_FORMAT_BC7_UNORM; break;
      case VK_FORMAT_BC7_UNORM_BLOCK: rp.m_dxgi_format = DXGI_FORMAT_BC7_UNORM; break;
      default:
        throw runtime_error("unknown target format");
    }

    u16vec2 level_size = size_;
    for (uint level = 0; level < levels; level++, level_size.x>>=1, level_size.y>>=1) {
      auto start_level = high_resolution_clock::now();
      uint byte_size = level_size.x * level_size.y * format_bpp(format_);
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

        std::cout << "\t\t\tencoded level " << level << " layer " << layer << " in " << duration<double, milli>(high_resolution_clock::now() - start_layer).count() << std::endl;
      }
      std::cout << "\t\tencoded level " << level << " in " << duration<double, milli>(high_resolution_clock::now() - start_level).count() << std::endl;
    }
    std::cout << "\tencoded in " << duration<double, milli>(high_resolution_clock::now() - start_total).count() << std::endl;

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

  const uint pixel_byte_size = format_bpp(result.format_);
  const uint levels = floor(std::log2(_size)) - 1;
  uint level_size = _size;
  for (uint level = 0; level < levels; level++, level_size>>=1) {
    const uint stride = level_size * pixel_byte_size;
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
    auto &pixels = lod.layers_.emplace_back(new u8[level_size * stride]);
    ofs.download(pixels.get(), stride);

#if defined(CUBAGA_PBRGEN_DEBUG)
    string output = path(_output_file).replace_extension((sstream(".debug.level") << level << ".png").str());
    stbi_write_png(output.c_str(), level_size, level_size, pixel_byte_size, pixels.get(), stride);
#endif

    result.levels_.emplace_back(std::move(lod));
    std::cout << "\t\tgenerated " << _output_file << " level " << level << " in " << duration<double, milli>(high_resolution_clock::now() - start_level).count() << std::endl;
  }

  result.to_bc7();
  result.write(_output_file);

  std::cout << "\tgenerated " << _output_file << " in " << duration<double, milli>(high_resolution_clock::now() - start_total).count() << std::endl;
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

  const uint pixel_byte_size = format_bpp(result.format_);
  const uint levels = floor(std::log2(_size)) - 1;
  uint level_size = _size;
  for (uint level = 0; level < levels; level++, level_size>>=1) {
    const uint stride = level_size * pixel_byte_size;
    auto start_level = high_resolution_clock::now();

    image_params iparams;
    iparams.size_ = {level_size, level_size};
    iparams.format_ = VK_FORMAT_R8G8B8A8_SRGB;
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

      auto &pixels = lod.layers_.emplace_back(new u8[stride * level_size]);
      ofs.download(pixels.get(), stride);

#if defined(CUBAGA_PBRGEN_DEBUG)
      string output = path(_output_file).replace_extension((sstream(".debug.level") << level << ".layer" << layer << ".png").str());
      stbi_write_png(output.c_str(), level_size, level_size, pixel_byte_size, pixels.get(), stride);
#endif
      std::cout << "\t\t\tgenerated " << _output_file << " level " << level << " layer " << layer << " in " << duration<double, milli>(high_resolution_clock::now() - start_layer).count() << std::endl;
    }
    result.levels_.emplace_back(std::move(lod));
    std::cout << "\t\tgenerated " << _output_file << " level " << level << " in " << duration<double, milli>(high_resolution_clock::now() - start_level).count() << std::endl;
  }

  result.to_bc7();
  result.write(_output_file);
  std::cout << "\tgenerated " << _output_file << " in " << duration<double, milli>(high_resolution_clock::now() - start_total).count() << std::endl;
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

  const uint pixel_byte_size = format_bpp(result.format_);
  const uint levels = floor(std::log2(_size)) - 1;
  uint level_size = _size;
  for (uint level = 0; level < levels; level++, level_size>>=1) {
    const uint stride = level_size * pixel_byte_size;
    auto start_level = high_resolution_clock::now();

    image_params iparams;
    iparams.size_ = {level_size, level_size};
    iparams.format_ = VK_FORMAT_R8G8B8A8_SRGB;
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

      auto &pixels = lod.layers_.emplace_back(new u8[stride * level_size]);
      ofs.download(pixels.get(), stride);

#if defined(CUBAGA_PBRGEN_DEBUG)
      string output = path(_output_file).replace_extension((sstream(".debug.level") << level << ".layer" << layer << ".png").str());
      stbi_write_png(output.c_str(), level_size, level_size, pixel_byte_size, pixels.get(), stride);
#endif
      std::cout << "\t\t\tgenerated " << _output_file << " level " << level << " layer " << layer << " in " << duration<double, milli>(high_resolution_clock::now() - start_layer).count() << std::endl;
    }
    result.levels_.emplace_back(std::move(lod));
    std::cout << "\t\tgenerated " << _output_file << " level " << level << " in " << duration<double, milli>(high_resolution_clock::now() - start_level).count() << std::endl;
  }

  result.to_bc7();
  result.write(_output_file);
  std::cout << "\tgenerated " << _output_file << " in " << duration<double, milli>(high_resolution_clock::now() - start_total).count() << std::endl;
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

  std::cout << "generated everything in " << duration<double, milli>(high_resolution_clock::now() - start_total).count() << std::endl;
  return EXIT_SUCCESS;
}
