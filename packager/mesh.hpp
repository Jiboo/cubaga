#pragma once

#include "utils.hpp"

struct vertex {
  glm::vec3 pos;
  glm::vec3 nor {0.f};
  glm::vec4 tan {0.f};
  glm::vec2 tex;
};

struct mesh {
  using vertices_t = unique_ptr<vertex[]>;
  using indices_t = unique_ptr<uint32_t[]>;

  uint32_t vertices_count_;
  vertices_t vertices_;
  uint32_t indices_count_;
  indices_t indices_;

  glm::vec3 scale_, translate_;

  data_item encoded_vertices_;
  struct lod {
    data_item data_;
    uint16_t indices_count_;
  };
  std::vector<lod> encoded_lods_;

  void import(context &_ctx, const cgltf_mesh &_input);
  void import(context &_ctx, const cgltf_primitive &_input);
  void import_indices(context &_ctx, const cgltf_accessor &_input);
  void import_attr(context &_ctx, const cgltf_accessor &_input, size_t _output_offset);

  void process(context &_ctx);
  size_t layout(size_t _offset);
  void dump(context &_ctx, std::ostream &_os);

  void encode_vertices();
  void gen_tangents();
  void optimize();
  void normalize();
  void gen_lods();
};
