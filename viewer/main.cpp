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
#include "hut/pipeline.hpp"
#include "hut/font.hpp"
#include "hut/window.hpp"

#include "hut_ktx2.hpp"
#include "imgui_impl_hut.hpp"

#include "meshoptimizer.h"

#include "viewer_shaders.hpp"
#include "viewer_shaders_refl.hpp"
#include "cubaga_pbr_data.hpp"

using namespace std;
using namespace hut;

struct model_ubo {
  mat4 proj_;
  mat4 view_;
  vec4 camera_pos_;
  vec4 sun_dir_;
  int debug_;
};
using shared_ubo = shared_ref<model_ubo>;

using axis_d = pipeline<uint16_t, viewer_shaders::axis_vert_spv_refl, hut_shaders::rgb_frag_spv_refl, const shared_ubo&>;
using skybox_d = pipeline<uint16_t, viewer_shaders::skybox_vert_spv_refl, viewer_shaders::skybox_frag_spv_refl, const shared_ubo&, const shared_image&, const shared_sampler&>;
using model_d = pipeline<uint16_t, viewer_shaders::model_vert_spv_refl, viewer_shaders::model_frag_spv_refl,
            const shared_ubo&,
            const shared_image&, const shared_image&, const shared_image&, const shared_image&, const shared_sampler&, // albedo emissive normals ORM
            const shared_image&, const shared_image&, const shared_image&, const shared_sampler&>; // irradiance prefiltered brdflut

mat4 make_mat(vec3 _translate, vec3 _scale) {
  mat4 m(1);
  m = translate(m, _translate);
  m = scale(m, _scale);
  return m;
}

u8 *read_float(u8 *_input, float *_out) {
  static_assert(endian::native == endian::little);
  *_out = *reinterpret_cast<float*>(_input);
  return _input + sizeof(float);
}

u8 *read_u32(u8 *_input, u32 *_out) {
  static_assert(endian::native == endian::little);
  *_out = *reinterpret_cast<uint32_t*>(_input);
  return _input + sizeof(uint32_t);
}

u8 *read_vec3(u8 *_input, vec3 *_out) {
  _input = read_float(_input, &_out->x);
  _input = read_float(_input, &_out->y);
  return read_float(_input, &_out->z);
}

u8 *read_u8vec4(u8 *_input, u8vec4 *_out) {
  _out->x = *_input++;
  _out->y = *_input++;
  _out->z = *_input++;
  _out->w = *_input++;
  return _input;
}

struct data_layout {
  u32 offset_, size_;

  u8 *parse(u8 *_input) {
    _input = read_u32(_input, &offset_);
    return read_u32(_input, &size_);
  }
};

struct texture {
  enum target_format {
    BC7_SRGB = 0,
    BC7_UNORM = 1,
    R8G8B8A8_SRGB = 2,
    R8G8B8A8_UNORM = 3,
  };

  u16vec2 extents_;
  VkFormat format_;
  vector<data_layout> mips_;

  u8 *parse(u8 *_input) {
    extents_.x = pow(2, *_input++);
    extents_.y = pow(2, *_input++);

    u8 input_format = *_input++;
    switch (input_format) {
      case BC7_SRGB: format_ = VK_FORMAT_BC7_SRGB_BLOCK; break;
      case BC7_UNORM: format_ = VK_FORMAT_BC7_UNORM_BLOCK; break;
      case R8G8B8A8_SRGB: format_ = VK_FORMAT_R8G8B8A8_SRGB; break;
      case R8G8B8A8_UNORM: format_ = VK_FORMAT_R8G8B8A8_UNORM; break;
      default: assert(false);
    }

    _input++; // reserved;

    uint min_extent = std::min(extents_.x, extents_.y);
    uint levels = floor(std::log2(min_extent)) - 1;
    for (uint level = 0; level < levels; level++) {
      auto &layout = mips_.emplace_back();
      _input = layout.parse(_input);
    }

    return _input;
  }
};

struct material {
  u8 albedo_, emissive_, normals_, orm_;
  u8vec4 color_roughness_factor_;
  u8vec4 emissive_metallic_factor_;

  u8 *parse(u8 *_input) {
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
  u32 vertices_count_;
  data_layout vertices_;

  struct lod {
    u32 indices_count_;
    data_layout indices_;

    u8 *parse(u8 *_input) {
      _input = read_u32(_input, &indices_count_);
      return indices_.parse(_input);
    }
  };
  vector<lod> lods_;

  uint8_t *parse(uint8_t *_input) {
    _input = read_vec3(_input, &translate_);
    _input = read_vec3(_input, &scale_);
    _input = read_u32(_input, &vertices_count_);
    _input = vertices_.parse(_input);

    u32 lod_count;
    _input = read_u32(_input, &lod_count);
    lods_.resize(lod_count);
    for (auto &lod : lods_)
      _input = lod.parse(_input);

    return _input;
  }
};

struct cubaga {
  vector<texture> textures_;
  vector<material> materials_;
  vector<mesh> meshes_;
  u32 data_size_;

  u8 *parse(u8 *_input) {
    static_assert(endian::native == endian::little);
    uint32_t magic;
    _input = read_u32(_input, &magic);
    constexpr uint32_t expected_magic = 'c' << 0 | 'b' << 8 | 'r' << 16 | '1' << 24;
    if (magic != expected_magic)
      throw runtime_error("wrong magic");

    u32 tex_count;
    _input = read_u32(_input, &tex_count);
    textures_.resize(tex_count);
    for (auto &tex : textures_)
      _input = tex.parse(_input);

    u32 mat_count;
    _input = read_u32(_input, &mat_count);
    materials_.resize(mat_count);
    for (auto &mat : materials_)
      _input = mat.parse(_input);

    u32 mesh_count;
    _input = read_u32(_input, &mesh_count);
    meshes_.resize(mesh_count);
    for (auto &mesh : meshes_)
      _input = mesh.parse(_input);

    return read_u32(_input, &data_size_);
  }
};

vec4 make_light_dir(vec2 _rot_deg = {75, 40}) {
  return vec4{
      sin(glm::radians(_rot_deg.x)) * cos(glm::radians(_rot_deg.y)),
      sin(glm::radians(_rot_deg.y)),
      cos(glm::radians(_rot_deg.x)) * cos(glm::radians(_rot_deg.y)),
      0
  };
}

int main(int _argc, char **_argv) {
  using clock = std::chrono::high_resolution_clock;
  auto profile_log = [start = clock::now()](const char *_message) {
    std::cout << "[cubaga] " << (clock::now() - start) << "\t" << _message << std::endl;
  };

  IMGUI_CHECKVERSION();
  ImGui::SetCurrentContext(ImGui::CreateContext());
  ImGui::StyleColorsDark();

  profile_log("imgui initialized");

  if (_argc != 2) {
    cerr << "usage: <crb file>" << endl;
    return EXIT_FAILURE;
  }
  ifstream fcrb(_argv[1], ios::binary | ios::ate);
  if (!fcrb.is_open()) {
    cerr << "couldn't open file " << _argv[1] << endl;
    return EXIT_FAILURE;
  }
  uint fcrb_size = fcrb.tellg();
  unique_ptr<uint8_t[]> fcrb_data {new uint8_t[fcrb_size] };
  fcrb.seekg(0, ios::beg);
  fcrb.read((char*)fcrb_data.get(), fcrb_size);
  fcrb.close();
  cubaga crb;
  uint8_t *crb_data = crb.parse(fcrb_data.get());
  profile_log("cbr parsed");

  display d("cubaga-viewer");

  profile_log("hut::display created");

  window_params wparams;
  wparams.flags_ |= window_params::FMULTISAMPLING;
  wparams.flags_ |= window_params::FDEPTH;
  window w(d, wparams);
  w.clear_color({1, 1, 1, 1});
  w.title(sstream("cubaga-viewer ") << _argv[1] << " " << _argv[2] << " " << _argv[3]);

  profile_log("hut::window created");

  if (!ImGui_ImplHut_Init(&d, &w, true))
    return EXIT_FAILURE;

  profile_log("hut::imgui initialized");

  auto b = d.alloc_buffer(256*1024*1024);

  pipeline_params axis_params;
  axis_params.topology_ = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
  axis_params.depthCompare_ = VK_COMPARE_OP_ALWAYS;
  auto axis_pipeline = make_unique<axis_d>(w, axis_params);
  profile_log("axis_pipeline initialized");

  auto axis_indices = b->allocate<uint16_t>(6);
  axis_indices->set({0, 1, 2, 3, 4, 5});
  auto axis_instance = b->allocate<axis_d::instance>(1);
  axis_instance->set(axis_d::instance{make_mat({0, 0, 0}, {1, 1, 1})});
  auto axis_vertices = b->allocate<axis_d::vertex>(6);
  axis_vertices->set({
    axis_d::vertex{{0, 0, 0}, {1, 0, 0}},
    axis_d::vertex{{1, 0, 0}, {1, 0, 0}}, // red => X

    axis_d::vertex{{0, 0, 0}, {0, 1, 0}},
    axis_d::vertex{{0, 1, 0}, {0, 1, 0}}, // green => Y

    axis_d::vertex{{0, 0, 0}, {0, 0, 1}},
    axis_d::vertex{{0, 0, 1}, {0, 0, 1}}, // blue => Z
  });

  auto skybox_pipeline = make_unique<skybox_d>(w);
  profile_log("skybox_pipeline initialized");

  auto skybox_instance = b->allocate<skybox_d::instance>(1);
  skybox_instance->set(skybox_d::instance{make_mat({0, 0, 0}, {1000, 1000, 1000})});
  auto skybox_indices = b->allocate<uint16_t>(36);
  skybox_indices->set({
    0, 1, 2, 2, 3, 0,
    1, 5, 6, 6, 2, 1,
    7, 6, 5, 5, 4, 7,
    4, 0, 3, 3, 7, 4,
    4, 5, 1, 1, 0, 4,
    3, 2, 6, 6, 7, 3,
  });
  auto skybox_vertices = b->allocate<skybox_d::vertex>(8);
  skybox_vertices->set({
    skybox_d::vertex{{-.5, -.5, +.5}},
    skybox_d::vertex{{+.5, -.5, +.5}},
    skybox_d::vertex{{+.5, +.5, +.5}},
    skybox_d::vertex{{-.5, +.5, +.5}},
    skybox_d::vertex{{-.5, -.5, -.5}},
    skybox_d::vertex{{+.5, -.5, -.5}},
    skybox_d::vertex{{+.5, +.5, -.5}},
    skybox_d::vertex{{-.5, +.5, -.5}},
  });

  auto model_pipeline = make_unique<model_d>(w);
  profile_log("model_pipeline initialized");

  struct mesh_bufset {
    vector<model_d::shared_indices> indices_;
    model_d::shared_vertices vertices_;
  };
  vector<mesh_bufset> mesh_bufsets;
  int current_mesh_selection = 0;
  int current_lod_selection = 0;
  const auto mesh_count = crb.meshes_.size();
  for (uint mesh_id = 0; mesh_id < mesh_count; mesh_id++) {
    auto &bufset = mesh_bufsets.emplace_back();
    const auto &mesh = crb.meshes_[mesh_id];
    const auto lod_count = mesh.lods_.size();

    for (uint lod_id = 0; lod_id < lod_count; lod_id++) {
      const auto &lod = mesh.lods_[lod_id];
      auto indices = b->allocate<uint16_t>(lod.indices_count_);
      {
        auto indices_updator = indices->update_raw_indirect(0, indices->size_bytes());
        if(meshopt_decodeIndexBuffer(indices_updator.data(), indices->size(), sizeof(uint16_t), crb_data + lod.indices_.offset_, lod.indices_.size_) != 0)
          throw runtime_error("couldn't decode indices");
      }
      bufset.indices_.emplace_back(indices);
    }
    auto vertices = b->allocate<model_d::vertex>(mesh.vertices_count_);
    {
      auto vertices_updator = vertices->update_raw_indirect(0, vertices->size_bytes());
      if (meshopt_decodeVertexBuffer(vertices_updator.data(), vertices->size(), sizeof(model_d::vertex), crb_data + mesh.vertices_.offset_, mesh.vertices_.size_) != 0)
        throw runtime_error("couldn't decode vertices");
    }
    bufset.vertices_ = vertices;
  }

  profile_log("mesh staged");

  auto import_tex = [&d, &crb, &crb_data](uint8_t _index) {
    auto &tex = crb.textures_[_index];

    image_params params;
    params.size_ = tex.extents_;
    params.format_ = tex.format_;
    params.tiling_ = VK_IMAGE_TILING_OPTIMAL;
    params.levels_ = tex.mips_.size();

    auto result = std::make_shared<image>(d, params);
    for (u16 level = 0; level < tex.mips_.size(); level++) {
      auto data = span<const u8>{crb_data + tex.mips_[level].offset_, tex.mips_[level].size_};
      auto row_pixel_count = tex.extents_.x >> level;
      auto row_stride = (row_pixel_count * info(tex.format_).bpp()) / 8;
      result->update({u16vec4{0, 0, tex.extents_ >> level}, level}, data, row_stride);
    }
    return result;
  };
  auto default_tex_rgba = [&d] (u8vec4 _color) {
    auto data = span<uint8_t>{&_color.x, 4};
    image_params params;
    params.size_ = {1, 1};
    params.format_ = VK_FORMAT_R8G8B8A8_UNORM;
    return image::load_raw(d, data, 4, params);
  };

  shared_image deftex_white = default_tex_rgba({255, 255, 255, 255});
  shared_image deftex_black = default_tex_rgba({0, 0, 0, 255});
  shared_image deftex_red = default_tex_rgba({255, 0, 0, 255});
  shared_image deftex_normals = default_tex_rgba({128, 128, 255, 255});

  struct material_texset {
    shared_image albedo_, emissive_, normals_, orm_;
  };
  vector<material_texset> material_texsets;
  int current_material_selection = 0;
  const auto material_count = crb.materials_.size();
  for (uint material_id = 0; material_id < material_count; material_id++) {
    const auto &mat = crb.materials_[material_id];
    material_texsets.emplace_back(material_texset{
        mat.albedo_ != 0xff ? import_tex(mat.albedo_) : deftex_white,
        mat.emissive_ != 0xff ? import_tex(mat.emissive_) : deftex_black,
        mat.normals_ != 0xff ? import_tex(mat.normals_) : deftex_normals,
        mat.orm_ != 0xff ? import_tex(mat.orm_) : deftex_black});
  }

  profile_log("textures staged");

  hut::ktx::load_params ktx_params;
  ktx_params.tiling_ = VK_IMAGE_TILING_OPTIMAL;
  shared_image brdflut = hut::ktx::load(d, cubaga_pbr_data::brdflut_ktx2, ktx_params).value_or(deftex_red);
  struct envmap_texset {
    shared_image irr_, pre_;
  };
  vector<envmap_texset> envmaps_texsets {
      {
          hut::ktx::load(d, cubaga_pbr_data::chromatic_rgba8_cube_irr_ktx2, ktx_params).value_or(deftex_white),
          hut::ktx::load(d, cubaga_pbr_data::chromatic_rgba8_cube_pre_ktx2, ktx_params).value_or(deftex_white),
      },
      {
          hut::ktx::load(d, cubaga_pbr_data::directional_rgba8_cube_irr_ktx2, ktx_params).value_or(deftex_white),
          hut::ktx::load(d, cubaga_pbr_data::directional_rgba8_cube_pre_ktx2, ktx_params).value_or(deftex_white),
      },
      {
          hut::ktx::load(d, cubaga_pbr_data::neutral_rgba8_cube_irr_ktx2, ktx_params).value_or(deftex_white),
          hut::ktx::load(d, cubaga_pbr_data::neutral_rgba8_cube_pre_ktx2, ktx_params).value_or(deftex_white),
      },
      {
          hut::ktx::load(d, cubaga_pbr_data::papermill_rgba8_cube_irr_ktx2, ktx_params).value_or(deftex_white),
          hut::ktx::load(d, cubaga_pbr_data::papermill_rgba8_cube_pre_ktx2, ktx_params).value_or(deftex_white),
      },
  };
  int current_envmap_selection = 0;

  profile_log("envmap staged");

  const auto &default_mesh = crb.meshes_[current_mesh_selection];
  const auto &default_mat = crb.materials_[current_material_selection];

  auto model_mat = make_mat(default_mesh.translate_, default_mesh.scale_);
  auto model_instance = b->allocate<model_d::instance>(1);
  model_d::instance cpu_instance = {
      default_mat.color_roughness_factor_,
      default_mat.emissive_metallic_factor_,
      model_mat,
      glm::transpose(glm::inverse(model_mat))
  };
  model_instance->set(cpu_instance);

  float scale_max = compMax(default_mesh.scale_);
  vec3 mesh_center = default_mesh.scale_ + default_mesh.translate_;
  vec3 orbit_base = vec3(0, 0, scale_max * 2);
  vec2 camera_rot = {0, 0};
  float camera_dist = scale_max * 2;

  vec2 light_rot {75, 40};
  int current_debug_selection = 0;
  model_ubo default_ubo{
      perspective(glm::radians(45.0f), w.size().x / (float) w.size().y, 0.001f, 1000.0f),
      lookAt(orbit_base, vec3(0, 0, 0), vec3{0, -1, 0}),
      vec4(orbit_base, 0),
      make_light_dir(light_rot),
      current_debug_selection
  };
  shared_ubo ubo = d.alloc_ubo(b, default_ubo);

  shared_sampler samp = make_shared<sampler>(d);

  axis_pipeline->write(0, ubo);
  bool draw_axis = true;

  size_t materials_count = std::max(1ULL, material_texsets.size());
  size_t total_set_number = envmaps_texsets.size() * materials_count;
  model_pipeline->alloc_next_descriptors(total_set_number - 1);
  skybox_pipeline->alloc_next_descriptors(total_set_number - 1);
  for (int material = 0; material < materials_count; material++) {
    for (int envmap = 0; envmap < envmaps_texsets.size(); envmap++) {
      uint descriptor_index = material * materials_count + envmap;
      const auto &mat_texset = material_texsets[material];
      const auto &env_texset = envmaps_texsets[envmap];
      model_pipeline->write(descriptor_index, ubo,
                            mat_texset.albedo_, mat_texset.emissive_, mat_texset.normals_, mat_texset.orm_, samp,
                            env_texset.irr_, env_texset.pre_, brdflut, samp);
      skybox_pipeline->write(descriptor_index, ubo, env_texset.pre_, samp);
    }
  }

  profile_log("hut/render prepared");

  w.on_draw.connect([&](VkCommandBuffer _buffer) {
    uint descriptor_index = current_material_selection * materials_count + current_envmap_selection;
    auto &bufset = mesh_bufsets[current_mesh_selection];
    model_pipeline->draw(_buffer, descriptor_index, bufset.indices_[current_lod_selection], model_instance, bufset.vertices_);
    skybox_pipeline->draw(_buffer, descriptor_index, skybox_indices, skybox_instance, skybox_vertices);
    if (draw_axis)
      axis_pipeline->draw(_buffer, 0, axis_indices, axis_instance, axis_vertices);

    ImGui_ImplHut_NewFrame();
    ImGui::NewFrame();

    if (ImGui::Begin("Viewer")) {
      if (ImGui::DragFloat2("Light direction", &light_rot.x, 1, -180, +180)) {
        auto light_dir = make_light_dir(light_rot);
        ubo->update_subone(0, offsetof(model_ubo, sun_dir_), sizeof(vec3), &light_dir);
      }
      const std::vector<const char*> debug_items = {
          "None",
          "Occlusion map", "Roughness map", "Metallic map",
          "Roughness", "Metallic",
          "Albedo map", "Base color",
          "Emissive map", "Emissive",
          "Color light", "Color ibl", "Color occluded", "Color emissive",
          "Normals map", "Normals",
          "diffuseContrib", "F", "G", "D", "specContrib", "BRDF", "BRDFx", "BRDFy", "BRDFy 2linear"
      };
      if (ImGui::Combo("Debug", &current_debug_selection, debug_items.data(), debug_items.size())) {
        ubo->update_subone(0, offsetof(model_ubo, debug_), sizeof(int), &current_debug_selection);
      }
      ImGui::Checkbox("Draw axes (R=>X, G=>Y, Z=>B)", &draw_axis);
      const std::vector<const char*> envmap_names = {
          "Chromatic",
          "Directional",
          "Neutral",
          "Papermill",
      };
      ImGui::Combo("Envmap", &current_envmap_selection, envmap_names.data(), envmap_names.size());

      // TODO Material selection
      // TODO Mesh selection
      // TODO LOD selection

      if (ImGui::DragScalarN("Color", ImGuiDataType_U8, &cpu_instance.color_roughness_, 3)) {
        model_instance->update_subone(0, offsetof(model_d::instance, color_roughness_), sizeof(u8vec3), &cpu_instance.color_roughness_);
      }
      if (ImGui::DragScalarN("Roughness", ImGuiDataType_U8, &cpu_instance.color_roughness_[3], 1)) {
        model_instance->update_subone(0, offsetof(model_d::instance, color_roughness_) + 3, sizeof(u8), &cpu_instance.color_roughness_[3]);
      }
      if (ImGui::DragScalarN("Emissive", ImGuiDataType_U8, &cpu_instance.emissive_metallic_, 3)) {
        model_instance->update_subone(0, offsetof(model_d::instance, emissive_metallic_), sizeof(u8vec3), &cpu_instance.emissive_metallic_);
      }
      if (ImGui::DragScalarN("Metallic", ImGuiDataType_U8, &cpu_instance.emissive_metallic_[3], 1)) {
        model_instance->update_subone(0, offsetof(model_d::instance, emissive_metallic_) + 3, sizeof(u8), &cpu_instance.emissive_metallic_[3]);
      }
    }
    ImGui::End();

    ImGui::Render();
    ImGui_ImplHut_RenderDrawData(_buffer, ImGui::GetDrawData());

    static bool already_announced = false;
    if (!already_announced) {
      profile_log("first command list recorded");
      already_announced = true;
    }

    return false;
  });

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
      camera_rot.y = std::clamp(camera_rot.y, -max_y, max_y);
      last_pos = _pos;
    }
    camera_dist = std::clamp(camera_dist, scale_max, 5 * scale_max);
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
    ubo->update_subone(0, offsetof(model_ubo, camera_pos_), sizeof(vec3), &orbit);

    w.invalidate(false);

    return true;
  });

  w.on_frame.connect([&](display::duration _dt) {
    d.post([&](auto){
      w.invalidate(true);
    });

    return false;
  });

  w.on_resize.connect([&](const uvec2 &_size) {
    mat4 new_proj = glm::perspective(glm::radians(45.0f), _size.x / (float) _size.y, 0.1f, 10.0f);
    ubo->update_subone(0, offsetof(model_ubo, proj_), sizeof(mat4), &new_proj);
    return false;
  });

  w.on_key.connect([&w](keycode, keysym c, bool _press) {
    if (c == KSYM_ESC && !_press)
      w.close();
    return true;
  });

  int error_code = d.dispatch();

  ImGui_ImplHut_Shutdown();
  ImGui::DestroyContext();

  return error_code;
}
