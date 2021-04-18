#pragma once

#include "utils.hpp"
#include "options.hpp"
#include "texture.hpp"

struct context {
  options opt_;
  cgltf_data *gltf_;

  // Objects to ouput
  std::vector<texture> textures_;
  std::vector<material> materials_;
  std::vector<mesh> meshes_;

  // Cache of cgltf instances, reset for each processed gltf input
  template <typename T>
  using cache_t = unordered_map<T*, unsigned>;
  cache_t<cgltf_image> cache_images_;
  cache_t<cgltf_texture> cache_textures_;
  cache_t<cgltf_material> cache_materials_;

  unique_ptr<uint8_t> data_;
  size_t data_size_;

  void clear_caches();
  unsigned import(cgltf_texture *_input, texture::target_format _format);
  unsigned import(cgltf_material *_input);

  void process();
  void dump(std::ostream &_os);
};
