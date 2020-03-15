#pragma once

#include <cstdint>

#include <bit>
#include <filesystem>
#include <iosfwd>
#include <memory>
#include <span>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "cgltf.h"
#include <glm/glm.hpp>
#include <glm/gtx/scalar_multiplication.hpp>
#include <glm/gtx/vec_swizzle.hpp>

template<typename T>
using vector = std::vector<T>;
template<typename K, typename V>
using unordered_map = std::unordered_map<K, V>;

template<typename T>
using uptr = std::unique_ptr<T, void (*)(T*)>;
template<typename T>
using unique_ptr = std::unique_ptr<T>;

using path = std::filesystem::path;
using error = std::runtime_error;

using pixel_t = glm::u8vec4;

struct options;
struct context;
struct texture;
struct material;
struct mesh;

struct data_item {
  unique_ptr<uint8_t[]> data_;
  size_t size_;
  size_t offset_;
};

template<typename TInput, typename TOutput>
uint8_t *read(uint8_t *_input, TOutput &_dst) {
  static_assert(std::endian::native == std::endian::little);

  auto *input = reinterpret_cast<TInput*>(_input);
  _dst = *input;
  return _input + sizeof(TInput);
}

template<typename TInput, typename TOutput>
uint8_t *read_norm(uint8_t *_input, TOutput &_dst) {
  static_assert(std::endian::native == std::endian::little);

  auto *input = reinterpret_cast<TInput*>(_input);
  _dst = *input / TOutput(std::numeric_limits<TInput>::max());
  if constexpr (std::is_signed_v<TInput>)
    _dst = _dst * 2. - 1.;
  return _input + sizeof(TInput);
}

inline void write_uleb128(std::ostream &_os, size_t _val) {
  do {
    uint8_t byte = _val & 0x7Fu;
    _val >>= 7u;
    if (_val != 0)
      byte |= 0x80u;
    _os.put(byte);
  } while (_val != 0);
}

inline uint8_t to_u8(float _val) {
  return _val * 255;
}

inline void write_u16(std::ostream &_os, uint16_t _val) {
  static_assert(std::endian::native == std::endian::little);
  _os.write(reinterpret_cast<char*>(&_val), sizeof(uint16_t));
}

inline void write_float(std::ostream &_os, float _val) {
  static_assert(std::endian::native == std::endian::little);
  _os.write(reinterpret_cast<char*>(&_val), sizeof(float));
}

inline void write_vec3(std::ostream &_os, glm::vec3 &_col) {
  write_float(_os, _col.r);
  write_float(_os, _col.g);
  write_float(_os, _col.b);
}

inline void write_u8vec3(std::ostream &_os, glm::vec3 &_col) {
  _os.put(to_u8(_col.r));
  _os.put(to_u8(_col.g));
  _os.put(to_u8(_col.b));
}
