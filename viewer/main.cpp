/*  _ _ _   _       _
 * | |_| |_| |_ _ _| |_
 * | | | . |   | | |  _|
 * |_|_|___|_|_|___|_|
 * Hobby graphics and GUI library under the MIT License (MIT)
 *
 * Copyright (c) 2014 Jean-Baptiste Lepesme github.com/jiboo/libhut
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <cstdint>

#include <fstream>
#include <iostream>
#include <vector>

#include "hut/display.hpp"
#include "hut/drawable.hpp"
#include "hut/font.hpp"
#include "hut/window.hpp"

#include "meshoptimizer.h"

#include "viewer_shaders.hpp"

using namespace hut;

struct model_ubo {
  mat4 proj_;
  mat4 view_;
  vec3 camera_;
};

namespace hut::details {
  template<typename T>
  constexpr static uint normals_offset(uint _column) {
    return offsetof(typename T::instance, normal_) + sizeof(vec4) * _column;
  }

  struct axis {
    using impl = drawable<details::axis>;
    using ubo = model_ubo;

    struct vertex {
      vec3 pos_;
      vec3 color_;
    };

    struct instance {
      mat4 transform_;
    };

    constexpr static uint max_descriptor_sets_ = 1;

    constexpr static std::array<VkDescriptorPoolSize, 1> descriptor_pools_ {
        VkDescriptorPoolSize{.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, .descriptorCount = max_descriptor_sets_},
    };

    constexpr static std::array<VkDescriptorSetLayoutBinding, 1> descriptor_bindings_{
        VkDescriptorSetLayoutBinding{.binding = 0, .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, .descriptorCount = 1, .stageFlags = VK_SHADER_STAGE_VERTEX_BIT, .pImmutableSamplers = nullptr},
    };

    static void fill_extra_descriptor_writes(impl::descriptor_write_context &) {
    }

    constexpr static std::array<VkVertexInputBindingDescription, 2> vertices_binding_ = {
        VkVertexInputBindingDescription{.binding = 0, .stride = sizeof(vertex),   .inputRate = VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{.binding = 1, .stride = sizeof(instance), .inputRate = VK_VERTEX_INPUT_RATE_INSTANCE},
    };

    constexpr static std::array<VkVertexInputAttributeDescription, 6> vertices_description_ {
        VkVertexInputAttributeDescription{.location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = offsetof(vertex, pos_)},
        VkVertexInputAttributeDescription{.location = 1, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = offsetof(vertex, color_)},
        VkVertexInputAttributeDescription{.location = 2, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = transform_offset<axis>(0)},
        VkVertexInputAttributeDescription{.location = 3, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = transform_offset<axis>(1)},
        VkVertexInputAttributeDescription{.location = 4, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = transform_offset<axis>(2)},
        VkVertexInputAttributeDescription{.location = 5, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = transform_offset<axis>(3)},
    };

    constexpr static decltype(hut_shaders::rgb_frag_spv) &frag_bytecode_ = hut_shaders::rgb_frag_spv;
    constexpr static decltype(viewer_shaders::axis_vert_spv) &vert_bytecode_ = viewer_shaders::axis_vert_spv;
  };

  struct model {
    using impl = drawable<hut::details::model, const shared_image &, const shared_image &, const shared_image &, const shared_image &, const shared_sampler &>;
    using ubo = model_ubo;

    struct instance {
      u8vec4 color_roughness_;
      u8vec4 emissive_metallic_;
      mat4 transform_;
      mat4 normal_;
    };

    struct vertex {
      uint32_t dword0_;
      uint32_t dword1_;
      uint32_t dword2_;
    };

    constexpr static uint max_descriptor_sets_ = 1;

    constexpr static std::array<VkDescriptorPoolSize, 2> descriptor_pools_{
        VkDescriptorPoolSize{.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, .descriptorCount = max_descriptor_sets_},
        VkDescriptorPoolSize{.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, .descriptorCount = 4*max_descriptor_sets_},
    };

    constexpr static std::array<VkDescriptorSetLayoutBinding, 5> descriptor_bindings_{
        VkDescriptorSetLayoutBinding{.binding = 0, .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, .descriptorCount = 1, .stageFlags = VK_SHADER_STAGE_VERTEX_BIT, .pImmutableSamplers = nullptr},
        VkDescriptorSetLayoutBinding{.binding = 1, .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, .descriptorCount = 1, .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT, .pImmutableSamplers = nullptr},
        VkDescriptorSetLayoutBinding{.binding = 2, .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, .descriptorCount = 1, .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT, .pImmutableSamplers = nullptr},
        VkDescriptorSetLayoutBinding{.binding = 3, .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, .descriptorCount = 1, .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT, .pImmutableSamplers = nullptr},
        VkDescriptorSetLayoutBinding{.binding = 4, .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, .descriptorCount = 1, .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT, .pImmutableSamplers = nullptr},
    };

    static void fill_extra_descriptor_writes(impl::descriptor_write_context &_context, const shared_image &_albedo,
        const shared_image &_emissive, const shared_image &_normals, const shared_image &_orm,
        const shared_sampler &_sampler) {
      _context.images_.resize(4);
      _context.texture(1, _albedo, _sampler);
      _context.texture(2, _emissive, _sampler);
      _context.texture(3, _normals, _sampler);
      _context.texture(4, _orm, _sampler);
    }

    constexpr static std::array<VkVertexInputBindingDescription, 2> vertices_binding_ = {
        VkVertexInputBindingDescription{.binding = 0, .stride = sizeof(vertex), .inputRate = VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{.binding = 1, .stride = sizeof(instance), .inputRate = VK_VERTEX_INPUT_RATE_INSTANCE},
    };

    constexpr static std::array<VkVertexInputAttributeDescription, 13> vertices_description_ {
        VkVertexInputAttributeDescription{.location = 0, .binding = 0, .format = VK_FORMAT_R32_UINT, .offset = offsetof(vertex, dword0_)},
        VkVertexInputAttributeDescription{.location = 1, .binding = 0, .format = VK_FORMAT_R32_UINT, .offset = offsetof(vertex, dword1_)},
        VkVertexInputAttributeDescription{.location = 2, .binding = 0, .format = VK_FORMAT_R32_UINT, .offset = offsetof(vertex, dword2_)},
        VkVertexInputAttributeDescription{.location = 3, .binding = 1, .format = VK_FORMAT_R8G8B8A8_UNORM, .offset = offsetof(instance, color_roughness_)},
        VkVertexInputAttributeDescription{.location = 4, .binding = 1, .format = VK_FORMAT_R8G8B8A8_UNORM, .offset = offsetof(instance, emissive_metallic_)},
        VkVertexInputAttributeDescription{.location = 5, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = transform_offset<model>(0)},
        VkVertexInputAttributeDescription{.location = 6, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = transform_offset<model>(1)},
        VkVertexInputAttributeDescription{.location = 7, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = transform_offset<model>(2)},
        VkVertexInputAttributeDescription{.location = 8, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = transform_offset<model>(3)},
        VkVertexInputAttributeDescription{.location = 9, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = normals_offset<model>(0)},
        VkVertexInputAttributeDescription{.location = 10, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = normals_offset<model>(1)},
        VkVertexInputAttributeDescription{.location = 11, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = normals_offset<model>(2)},
        VkVertexInputAttributeDescription{.location = 12, .binding = 1, .format = VK_FORMAT_R32G32B32A32_SFLOAT, .offset = normals_offset<model>(3)},
    };

    constexpr static decltype(viewer_shaders::cubaga_frag_spv) &frag_bytecode_ = viewer_shaders::cubaga_frag_spv;
    constexpr static decltype(viewer_shaders::cubaga_vert_spv) &vert_bytecode_ = viewer_shaders::cubaga_vert_spv;
  };
}
using axis_d = hut::details::axis::impl;
using model = hut::details::model::impl;

mat4 make_mat(vec3 _translate, vec3 _scale) {
  mat4 m(1);
  m = translate(m, _translate);
  m = scale(m, _scale);
  return m;
}

uint8_t *read_uleb128(uint8_t *_input, uint *_out) {
  uint result = 0;
  uint shift = 0;
  while (true) {
    uint8_t byte = *_input++;
    result |= (byte & 0x7Fu) << shift;
    if ((byte & 0x80u) == 0)
      break;
    shift += 7;
  }
  *_out = result;
  return _input;
}

uint8_t *read_float(uint8_t *_input, float *_out) {
  static_assert(std::endian::native == std::endian::little);
  *_out = *reinterpret_cast<float*>(_input);
  return _input + sizeof(float);
}

uint8_t *read_u16(uint8_t *_input, uint16_t *_out) {
  static_assert(std::endian::native == std::endian::little);
  *_out = *reinterpret_cast<uint16_t*>(_input);
  return _input + sizeof(uint16_t);
}

uint8_t *read_u32(uint8_t *_input, uint32_t *_out) {
  static_assert(std::endian::native == std::endian::little);
  *_out = *reinterpret_cast<uint32_t*>(_input);
  return _input + sizeof(uint32_t);
}

uint8_t *read_vec3(uint8_t *_input, vec3 *_out) {
  _input = read_float(_input, &_out->x);
  _input = read_float(_input, &_out->y);
  return read_float(_input, &_out->z);
}

uint8_t *read_u8vec4(uint8_t *_input, u8vec4 *_out) {
  _out->x = *_input++;
  _out->y = *_input++;
  _out->z = *_input++;
  _out->w = *_input++;
  return _input;
}

struct data_layout {
  uint offset_, size_;

  uint8_t *parse(uint8_t *_input) {
    _input = read_uleb128(_input, &size_);
    return read_uleb128(_input, &offset_);
  }
};

struct texture {
  uvec2 extents_;
  data_layout layout_;

  uint8_t *parse(uint8_t *_input) {
    uint8_t size = *_input++;
    extents_.x = pow(2, size & 0xF);
    extents_.y = pow(2, (size >> 4) & 0xF);
    return layout_.parse(_input);
  }
};

struct material {
  u8 albedo_, emissive_, normals_, orm_;
  u8vec4 color_roughness_factor_;
  u8vec4 emissive_metallic_factor_;

  uint8_t *parse(uint8_t *_input) {
    albedo_ = *_input++;
    emissive_ = *_input++;
    normals_ = *_input++;
    orm_ = *_input++;
    _input = read_u8vec4(_input, &color_roughness_factor_);
    return read_u8vec4(_input, &emissive_metallic_factor_);
  }
};

struct mesh {
  vec3 translate_, scale_;
  u16 vertices_count_;
  data_layout vertices_;

  struct lod {
    uint indices_count_;
    data_layout indices_;

    uint8_t *parse(uint8_t *_input) {
      _input = read_uleb128(_input, &indices_count_);
      return indices_.parse(_input);
    }
  };
  std::vector<lod> lods_;

  uint8_t *parse(uint8_t *_input) {
    _input = read_vec3(_input, &translate_);
    _input = read_vec3(_input, &scale_);
    _input = read_u16(_input, &vertices_count_);
    _input = vertices_.parse(_input);
    lods_.resize(*_input++);
    for (auto &lod : lods_)
      _input = lod.parse(_input);
    return _input;
  }
};

struct cubaga {
  std::vector<texture> textures_;
  std::vector<material> materials_;
  std::vector<mesh> meshes_;
  uint data_size_;

  uint8_t *parse(uint8_t *_input) {
    static_assert(std::endian::native == std::endian::little);
    uint32_t magic;
    _input = read_u32(_input, &magic);
    constexpr uint32_t expected_magic = 'c' << 24 | 'b' << 16 | 'r' << 8 | '1';
    if (magic != expected_magic)
      throw std::runtime_error("wrong magic");
    textures_.resize(*_input++);
    for (auto &tex : textures_)
      _input = tex.parse(_input);
    materials_.resize(*_input++);
    for (auto &mat : materials_)
      _input = mat.parse(_input);
    meshes_.resize(*_input++);
    for (auto &mesh : meshes_)
      _input = mesh.parse(_input);
    return read_uleb128(_input, &data_size_);
  }
};

int main(int _argc, char **_argv) {
  if (_argc != 4) {
    std::cerr << "usage: <cbr file> <material indice> <mesh indice>" << std::endl;
    return EXIT_FAILURE;
  }
  std::ifstream fcbr(_argv[1], std::ios::binary | std::ios::ate);
  if (!fcbr.is_open()) {
    std::cerr << "couldn't open file " << _argv[1] << std::endl;
    return EXIT_FAILURE;
  }
  uint fcbr_size = fcbr.tellg();
  std::unique_ptr<uint8_t[]> fcbr_data { new uint8_t[fcbr_size] };
  fcbr.seekg(0, std::ios::beg);
  fcbr.read((char*)fcbr_data.get(), fcbr_size);
  fcbr.close();
  cubaga cbr;
  uint8_t *cbr_data = cbr.parse(fcbr_data.get());

  material cbr_mat = cbr.materials_[atoi(_argv[2])];
  mesh cbr_mesh = cbr.meshes_[atoi(_argv[3])];

  display d("cubaga-viewer");

  window w(d);
  w.clear_color({1, 1, 1, 1});
  w.title(sstream("cubaga-viewer ") << _argv[1] << " " << _argv[2] << " " << _argv[3]);

  auto b = d.alloc_buffer(256*1024*1024);

  auto axis_pipeline = std::make_unique<axis_d>(w, VK_PRIMITIVE_TOPOLOGY_LINE_LIST, VK_COMPARE_OP_NEVER);
  auto axis_indices = b->allocate<uint16_t>(6);
  axis_indices->set({0, 1, 2, 3, 4, 5});
  auto axis_instance = b->allocate<axis_d::instance>(1);
  axis_instance->set(axis_d::instance{make_mat({0, 0, 0}, {1, 1, 1})});
  auto axis_vertices = b->allocate<axis_d::vertex>(6);
  axis_vertices->set({
                         axis_d::vertex{{0, 0, 0}, {1, 0, 0}},
                         axis_d::vertex{{1, 0, 0}, {1, 0, 0}},
                         axis_d::vertex{{0, 0, 0}, {0, 1, 0}},
                         axis_d::vertex{{0, 1, 0}, {0, 1, 0}},
                         axis_d::vertex{{0, 0, 0}, {0, 0, 1}},
                         axis_d::vertex{{0, 0, 1}, {0, 0, 1}},
  });

  auto lod0 = cbr_mesh.lods_[0];
  auto indices = b->allocate<uint16_t>(lod0.indices_count_);
  {
    auto indices_updator = indices->update_raw_indirect(0, indices->size_bytes());
    if(meshopt_decodeIndexBuffer(indices_updator.data(), indices->size(), sizeof(uint16_t), cbr_data + lod0.indices_.offset_, lod0.indices_.size_) != 0)
      throw std::runtime_error("couldn't decode indices");
  }

  auto model_pipeline = std::make_unique<model>(w);
  auto model_instance = b->allocate<model::instance>(1);
  auto model_mat = make_mat(cbr_mesh.translate_, cbr_mesh.scale_);
  model_instance->set(model::instance{
    cbr_mat.color_roughness_factor_,
    cbr_mat.emissive_metallic_factor_,
    model_mat,
    glm::transpose(glm::inverse(model_mat))
  });

  auto vertices = b->allocate<model::vertex>(cbr_mesh.vertices_count_);
  {
    auto vertices_updator = vertices->update_raw_indirect(0, vertices->size_bytes());
    if (meshopt_decodeVertexBuffer(vertices_updator.data(), vertices->size(), sizeof(model::vertex), cbr_data + cbr_mesh.vertices_.offset_, cbr_mesh.vertices_.size_) != 0)
      throw std::runtime_error("couldn't decode vertices");
  }

  auto import_tex = [&d, &cbr, &cbr_data](uint8_t _index, VkFormat _format) {
    auto &tex = cbr.textures_[_index];
    auto data = std::span<uint8_t>{cbr_data + tex.layout_.offset_, tex.layout_.size_};
    return image::load_raw(d, tex.extents_, data, _format, VK_IMAGE_TILING_OPTIMAL);
  };
  auto default_tex = [&d] (u8vec3 _color) {
    auto data = std::span<uint8_t>{&_color.x, 3};
    return image::load_raw(d, {1, 1}, data, VK_FORMAT_R8G8B8_SRGB);
  };

  shared_image albedo = cbr_mat.albedo_ != 0xff ? import_tex(cbr_mat.albedo_, VK_FORMAT_BC7_SRGB_BLOCK) : default_tex({255, 255, 255});
  shared_image emissive = cbr_mat.emissive_ != 0xff ? import_tex(cbr_mat.emissive_, VK_FORMAT_BC7_SRGB_BLOCK) : default_tex({255, 255, 255});
  shared_image normals = cbr_mat.normals_ != 0xff ? import_tex(cbr_mat.normals_, VK_FORMAT_BC7_UNORM_BLOCK) : default_tex({255, 255, 255});
  shared_image orm = cbr_mat.orm_ != 0xff ? import_tex(cbr_mat.orm_, VK_FORMAT_BC7_UNORM_BLOCK) : default_tex({255, 255, 255});

  float scale_max = compMax(cbr_mesh.scale_);
  vec3 mesh_center = cbr_mesh.scale_ + cbr_mesh.translate_;
  vec3 orbit_base = vec3(0, 0, scale_max * 2);
  model_ubo default_ubo{
      perspective(glm::radians(45.0f), w.size().x / (float) w.size().y, 0.001f, 1000.0f),
      lookAt(
          orbit_base,
          vec3(0, 0, 0),
          vec3{0, -1, 0}),
      orbit_base,
  };
  shared_ubo<model_ubo> ubo = alloc_ubo(d, b, default_ubo);

  shared_sampler samp = std::make_shared<sampler>(d);

  model_pipeline->bind(0, ubo, albedo, emissive, normals, orm, samp);
  axis_pipeline->bind(0, ubo);

  w.on_draw.connect([&](VkCommandBuffer _buffer) {
    model_pipeline->draw(_buffer, 0, indices, model_instance, vertices);
    axis_pipeline->draw(_buffer, 0, axis_indices, axis_instance, axis_vertices);
    return false;
  });

  w.on_resize.connect([&](const uvec2 &_size) {
    mat4 new_proj = glm::perspective(glm::radians(45.0f), _size.x / (float) _size.y, 0.1f, 10.0f);
    ubo->update_subone(0, offsetof(model_ubo, proj_), sizeof(mat4), &new_proj);
    return false;
  });

  w.on_key.connect([&w](keysym c, bool _press) {
    if (c == KESCAPE && !_press)
      w.close();
    return true;
  });

  vec2 camera_rot = {0, 0};
  float camera_dist = scale_max * 2;
  w.on_mouse.connect([&](uint8_t _button, mouse_event_type _type, vec2 _pos) {
    static bool button_clicked = false;
    static vec2 last_pos = {0, 0};

    if (_type == MUP) {
      button_clicked = false;
      return true;
    }
    else if (_type == MDOWN) {
      button_clicked = true;
      last_pos = _pos;
      return true;
    }
    else if (_type == MWHEEL_UP) {
      camera_dist -= scale_max;
    }
    else if (_type == MWHEEL_DOWN) {
      camera_dist += scale_max;
    }
    else if (_type == MMOVE) {
      if (!button_clicked)
        return true;
      vec2 offset {_pos - last_pos};
      camera_rot += offset / vec2(w.size()) * float(M_PI);
      constexpr float max_y = M_PI_2 - 0.01;
      camera_rot.y = clamp(camera_rot.y, -max_y, max_y);
      last_pos = _pos;
    }
    camera_dist = clamp(camera_dist, scale_max, 5 * scale_max);
    orbit_base = vec3(0, 0, camera_dist);

    mat4 mrot = mat4(1);
    mrot = rotate(mrot, -camera_rot.x, vec3{0, 1, 0});
    mrot = rotate(mrot, -camera_rot.y, vec3{1, 0, 0});
    vec3 orbit = mrot * vec4(orbit_base, 1);

    mat4 m = lookAt(
        orbit,
        vec3(0, 0, 0),
        vec3{0, -1, 0});
    ubo->update_subone(0, offsetof(model_ubo, view_), sizeof(mat4), &m);
    ubo->update_subone(0, offsetof(model_ubo, camera_), sizeof(vec3), &orbit);

    w.invalidate(false);

    return true;
  });

  return d.dispatch();
}
