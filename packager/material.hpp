#pragma once

#include "utils.hpp"

struct material {
  uint8_t albedo_;
  uint8_t normals_;
  uint8_t emissive_;
  uint8_t omr_;
  glm::vec3 color_factor_;
  glm::vec3 emissive_factor_;
  uint8_t metallic_factor_;
  uint8_t roughness_factor_;

  void import(context &_ctx, const cgltf_material &_input);
  void process(context &_ctx);
  void dump(context &_ctx, std::ostream &_os);
};
