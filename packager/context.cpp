#include "context.hpp"

#include "texture.hpp"
#include "material.hpp"
#include "mesh.hpp"

template<typename TGLTF, typename TCBG, typename... TExtras>
unsigned cache_or_insert(context &_ctx, TGLTF *_input,
    context::cache_t<TGLTF> &_cache, vector<TCBG> &_storage, TExtras... _extras) {
  auto it = _cache.find(_input);
  if (it == _cache.end()) {
    _storage.emplace_back().import(_ctx, *_input, std::forward<TExtras>(_extras)...);
    size_t id = _storage.size() - 1;
    _cache[_input] = id;
    return id;
  }
  return it->second;
}

unsigned context::import(cgltf_texture *_input, bool _srgb) {
  return cache_or_insert(*this, _input, cache_textures_, textures_, _srgb);
}
unsigned context::import(cgltf_material *_input) {
  return cache_or_insert(*this, _input, cache_materials_, materials_);
}

void context::clear_caches() {
  cache_images_.clear();
  cache_textures_.clear();
  cache_materials_.clear();
}

void context::process() {
  size_t byte_size = 0;
  for (auto &tex : textures_) {
    tex.process(*this);
    byte_size += tex.layout(byte_size);
  }
  for (auto &mat : materials_)
    mat.process(*this);
  for (auto &mesh : meshes_) {
    mesh.process(*this);
    byte_size += mesh.layout(byte_size);
  }
  data_.reset(new uint8_t[byte_size]);
  data_size_ = byte_size;
}

void context::dump(std::ostream &_os) {
  uint32_t magic = 'c' << 24 | 'b' << 16 | 'r' << 8 | '1';
  _os.write(reinterpret_cast<char*>(&magic), sizeof(uint32_t));

  if (textures_.size() > 254)
    throw error("too many textures");
  _os.put((uint8_t)textures_.size());
  for (auto &tex : textures_)
    tex.dump(*this, _os);

  if (materials_.size() > 255)
    throw error("too many materials");
  _os.put((uint8_t)materials_.size());
  for (auto &mat : materials_)
    mat.dump(*this, _os);

  if (meshes_.size() > 255)
    throw error("too many meshes");
  _os.put((uint8_t)meshes_.size());
  for (auto &mesh : meshes_)
    mesh.dump(*this, _os);

  write_uleb128(_os, data_size_);
  _os.write(reinterpret_cast<char*>(data_.get()), data_size_);
}
