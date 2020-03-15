#include "mesh.hpp"

#include <cstring>

#include "mikktspace.h"
#include "meshoptimizer.h"

#ifdef CUBAGA_DEBUG
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/matrix_decompose.hpp"
#endif

#include "context.hpp"

void transform_vec3(glm::vec3 &_input, bool _vector, const glm::mat4 &_tranform) {
  glm::vec4 i(_input, _vector ? 0 : 1);
  glm::vec4 transformed = _tranform * i;
  _input = glm::xyz(transformed) / (_vector ? 1 : transformed.w);
}

void mesh::import(context &_ctx, const cgltf_mesh &_input) {
  if (_input.primitives_count != 1)
    throw error("mesh primitive count != 1");
  import(_ctx, _input.primitives[0]);

  for (uint inode = 0; inode < _ctx.gltf_->nodes_count; inode++) {
    cgltf_node &node = _ctx.gltf_->nodes[inode];
    if (node.mesh == &_input) {
      glm::mat4 transform = glm::mat4(1);
      cgltf_node_transform_world(&node, &transform[0][0]);

#ifdef CUBAGA_DEBUG
      glm::vec3 scale;
      glm::quat rotation;
      glm::vec3 translation;
      glm::vec3 skew;
      glm::vec4 perspective;
      glm::decompose(transform, scale, rotation, translation, skew, perspective);
      glm::vec3 euler = glm::eulerAngles(rotation) * (180.f / M_PI);
#endif

      for (unsigned i = 0; i < vertices_count_; i++) {
        vertex &v = vertices_[i];
        transform_vec3(v.pos, false, transform);
        transform_vec3(v.nor, true, transform);
        v.tan = v.tan * transform;
      }
      break;
    }
  }
}

void mesh::import(context &_ctx, const cgltf_primitive &_input) {
  if (_input.has_draco_mesh_compression)
    throw error("primitive requires draco mesh compression");
  if (_input.type != cgltf_primitive_type_triangles)
    throw error("primitive is not a triangle list");

  _ctx.import(_input.material);

  if (_input.indices == nullptr)
    throw error("primitive has no indices");

  bool has_tangents = false;
  import_indices(_ctx, *_input.indices);
  for (unsigned i = 0; i < _input.attributes_count; i++) {
    auto &attr = _input.attributes[i];
    switch (attr.type) {
      case cgltf_attribute_type_position: import_attr(_ctx, *attr.data, offsetof(vertex, pos)); break;
      case cgltf_attribute_type_normal: import_attr(_ctx, *attr.data, offsetof(vertex, nor)); break;
      case cgltf_attribute_type_texcoord: import_attr(_ctx, *attr.data, offsetof(vertex, tex)); break;
      case cgltf_attribute_type_tangent:
        import_attr(_ctx, *attr.data, offsetof(vertex, tan));
        has_tangents = true;
        break;
      default:
        throw error("unexpected attribute");
    }
  }
  if (!has_tangents) {
    gen_tangents();
  }
}

template<typename TOutput>
void read_accessor(const cgltf_accessor &_input, TOutput *_output, size_t _output_offset, size_t _output_stride) {
  unsigned element_count = _input.count;

  if (_input.is_sparse)
    throw error("sparse not supported");

  uint8_t *(*reader)(uint8_t*, TOutput&);
  unsigned component_size;
  switch(_input.component_type) {
    case cgltf_component_type_r_8:
      reader = _input.normalized ? &read_norm<int8_t, TOutput> : &read<int8_t, TOutput>;
      component_size = 1;
      break;
    case cgltf_component_type_r_8u:
      reader = _input.normalized ? &read_norm<uint8_t,  TOutput> : &read<uint8_t, TOutput>;
      component_size = 1;
      break;
    case cgltf_component_type_r_16:
      reader = _input.normalized ? &read_norm<int16_t,  TOutput> : &read<int16_t, TOutput>;
      component_size = 2;
      break;
    case cgltf_component_type_r_16u:
      reader = _input.normalized ? &read_norm<uint16_t, TOutput> : &read<uint16_t, TOutput>;
      component_size = 2;
      break;
    case cgltf_component_type_r_32f:
      reader = _input.normalized ? &read_norm<float,  TOutput> : &read<float, TOutput>;
      component_size = 4;
      break;
    case cgltf_component_type_r_32u:
      reader = _input.normalized ? &read_norm<uint32_t, TOutput> : &read<uint32_t, TOutput>;
      component_size = 4;
      break;
    default:
      throw error("unexpected component type");
  }

  unsigned component_count;
  switch (_input.type) {
    case cgltf_type_scalar: component_count = 1; break;
    case cgltf_type_vec2: component_count = 2; break;
    case cgltf_type_vec3: component_count = 3; break;
    case cgltf_type_vec4: component_count = 4; break;
    default:
      throw error("unexpected component count");
  }

  auto &view = *_input.buffer_view;
  auto offset = _input.offset + view.offset;
  auto &buffer = *view.buffer;
  if (buffer.data == nullptr)
    throw error("buffer not loaded");

  auto *i = static_cast<uint8_t*>(buffer.data) + offset;
  auto *o = reinterpret_cast<uint8_t*>(_output) + _output_offset;
  unsigned i_jump = _input.stride - component_count * component_size;
  unsigned o_jump = _output_stride - component_count * sizeof(TOutput);
  for (unsigned element = 0; element < element_count; element++) {
    for (unsigned component = 0; component < component_count; component++) {
      i = reader(i, *reinterpret_cast<TOutput*>(o));
      o += sizeof(TOutput);
    }
    i += i_jump;
    o += o_jump;
  }
}

void mesh::import_indices(context &_ctx, const cgltf_accessor &_input) {
  if (_input.type != cgltf_type_scalar)
    throw error("indice type isn't scalar");

  if (_input.count == 0)
    throw error("no indices");

  indices_count_ = _input.count;
  indices_.reset(new uint32_t[indices_count_]);
  read_accessor<uint32_t>(_input, indices_.get(), 0, sizeof(uint32_t));
}

void mesh::import_attr(context &_ctx, const cgltf_accessor &_input, size_t _output_offset) {
  if (_input.count == 0)
    throw error("no attr data");

  if (!vertices_) {
    vertices_count_ = _input.count;
    vertices_ = std::make_unique<vertex[]>(vertices_count_);
  }
  else if (_input.count != vertices_count_) {
    throw error("attribute sizes are different");
  }
  read_accessor<float>(_input, reinterpret_cast<float*>(vertices_.get()), _output_offset, sizeof(vertex));
}

int mikkt_getNumFaces(const SMikkTSpaceContext * pContext) {
  auto *m = static_cast<mesh*>(pContext->m_pUserData);
  return m->indices_count_ / 3;
}

int mikkt_getNumVerticesOfFace(const SMikkTSpaceContext * pContext, const int iFace) {
  return 3;
}

void mikkt_getPosition(const SMikkTSpaceContext * pContext, float fvPosOut[], const int iFace, const int iVert) {
  auto *m = static_cast<mesh*>(pContext->m_pUserData);
  unsigned vtx = 3 * iFace + iVert;
  unsigned idx = m->indices_[vtx];
  vertex &v = m->vertices_[idx];
  fvPosOut[0] = v.pos.x;
  fvPosOut[1] = v.pos.y;
  fvPosOut[2] = v.pos.z;
}

void mikkt_getNormal(const SMikkTSpaceContext * pContext, float fvNormOut[], const int iFace, const int iVert) {
  auto *m = static_cast<mesh*>(pContext->m_pUserData);
  unsigned vtx = 3 * iFace + iVert;
  unsigned idx = m->indices_[vtx];
  vertex &v = m->vertices_[idx];
  fvNormOut[0] = v.nor.x;
  fvNormOut[1] = v.nor.y;
  fvNormOut[2] = v.nor.z;
}

void mikkt_getTexCoord(const SMikkTSpaceContext * pContext, float fvTexcOut[], const int iFace, const int iVert) {
  auto *m = static_cast<mesh*>(pContext->m_pUserData);
  unsigned vtx = 3 * iFace + iVert;
  unsigned idx = m->indices_[vtx];
  vertex &v = m->vertices_[idx];
  fvTexcOut[0] = v.tex.x;
  fvTexcOut[1] = v.tex.y;
}

void mikkt_setTSpaceBasic(const SMikkTSpaceContext * pContext, const float fvTangent[], const float fSign,
                          const int iFace, const int iVert) {
  auto *m = static_cast<mesh*>(pContext->m_pUserData);
  unsigned vtx = 3 * iFace + iVert;
  unsigned idx = m->indices_[vtx];
  vertex &v = m->vertices_[idx];
  v.tan.x = fvTangent[0];
  v.tan.y = fvTangent[1];
  v.tan.z = fvTangent[2];
  if (glm::xyz(v.tan) == glm::vec3(0,0,0)) {
    // TODO JBL: Mikktspace returns a null vector for vert 2760 (11355 with opt) of DamagedHelmet
    v.tan.x = 0.5;
  }
  v.tan.w = fSign;
}

void mesh::gen_tangents() {
  SMikkTSpaceInterface interface;
  interface.m_getNormal = mikkt_getNormal;
  interface.m_getNumFaces = mikkt_getNumFaces;
  interface.m_getNumVerticesOfFace = mikkt_getNumVerticesOfFace;
  interface.m_getPosition = mikkt_getPosition;
  interface.m_getTexCoord = mikkt_getTexCoord;
  interface.m_setTSpaceBasic = mikkt_setTSpaceBasic;
  interface.m_setTSpace = nullptr;
  SMikkTSpaceContext context;
  context.m_pUserData = this;
  context.m_pInterface = &interface;
  if (!genTangSpaceDefault(&context))
    throw error("error while computing tangents");
}

void mesh::process(context &_ctx) {
  optimize();
  normalize();
  encode_vertices();
  gen_lods();
}

void mesh::optimize() {
  if (vertices_count_ > 0xFFFF) {
    throw error("mesh too complex, reduce vertices count to under 65535");
    // FIXME Do a series of meshopt_simplify and remaps until we are under limit?
  }
  meshopt_optimizeVertexCache(indices_.get(), indices_.get(), indices_count_, vertices_count_);
  meshopt_optimizeOverdraw(indices_.get(), indices_.get(), indices_count_, reinterpret_cast<float*>(vertices_.get()),
                           vertices_count_, sizeof(vertex), 1.05f);
  vertices_count_ = meshopt_optimizeVertexFetch(vertices_.get(), indices_.get(), indices_count_, vertices_.get(),
      vertices_count_, sizeof(vertex));
}

void mesh::normalize() {
  glm::vec3 min_pos(std::numeric_limits<float>::max()), max_pos(-std::numeric_limits<float>::max());
  for (unsigned i = 0; i < vertices_count_; i++) {
    vertex &v = vertices_[i];
    v.nor = glm::normalize(v.nor);
    v.tan = glm::vec4(glm::normalize(glm::xyz(v.tan)), v.tan.w);
    max_pos = max(v.pos, max_pos);
    min_pos = min(v.pos, min_pos);
    // Simulate a "REPEAT" on texcoords
    if (v.tex.x < 0)
      v.tex.x = 1 - v.tex.x;
    else if (v.tex.x > 1)
      v.tex.x = v.tex.x - 1;
    if (v.tex.y < 0)
      v.tex.y = 1 - v.tex.y;
    else if (v.tex.y > 1)
      v.tex.y = v.tex.y - 1;
  }
  glm::vec3 translate = min_pos;
  glm::vec3 scale = max_pos - min_pos;
  for (unsigned i = 0; i < vertices_count_; i++) {
    vertex &v = vertices_[i];
    v.pos -= translate;
    v.pos /= scale;
    assert(min(v.pos, glm::vec3(0, 0, 0)) == glm::vec3(0,0,0));
    assert(max(v.pos, glm::vec3(1, 1, 1)) == glm::vec3(1,1,1));
  }
  translate_ = translate;
  scale_ = scale;
}

uint32_t quantize_unorm(float _input, uint _bitsize, uint _bitoffset) {
  assert(_input >= 0);
  assert(_input <= 1);

  const uint32_t mask = (1u << _bitsize) - 1u;
  const uint32_t result = lroundf(_input * mask);
  return (result & mask) << _bitoffset;
}

uint32_t quantize_snorm(float _input, uint _bitsize, uint _bitoffset) {
  assert(_input >= -1);
  assert(_input <= +1);

  const uint32_t mask = (1u << _bitsize) - 1u;
  const auto scale = float((1u << (_bitsize - 1u)) - 1u);
  const uint32_t result = lroundf(_input * scale);
  return (result & mask) << _bitoffset;
}

void encode_vertex(uint8_t *_dst, const vertex &_vertex) {
  auto *buffer = reinterpret_cast<uint32_t*>(_dst);

  *buffer = quantize_unorm(_vertex.pos.x, 11, 21)
          | quantize_unorm(_vertex.pos.y, 11, 10)
          | quantize_unorm(_vertex.pos.z, 10,  0);
  buffer++;

  *buffer = quantize_snorm(_vertex.nor.x, 8, 24)
          | quantize_snorm(_vertex.nor.y, 8, 16)
          | quantize_snorm(_vertex.nor.z, 8,  8)
          | quantize_snorm(_vertex.tan.x, 7,  0);
  if (_vertex.tan.w > 0)
    *buffer |= 0x80u;
  buffer++;

  *buffer = quantize_snorm(_vertex.tan.y,  6, 26)
          | quantize_snorm(_vertex.tan.z,  6, 20)
          | quantize_unorm(_vertex.tex.x, 10, 10)
          | quantize_unorm(_vertex.tex.y, 10,  0);
}

void mesh::encode_vertices() {
  constexpr unsigned quantized_vertex_size = sizeof(uint32_t) * 3;
  unsigned quantized_buffer_size = vertices_count_ * quantized_vertex_size;
  unique_ptr<uint8_t[]> quantized_vertices {new uint8_t[quantized_buffer_size]};
  for (unsigned i = 0; i < vertices_count_; i++)
    encode_vertex(quantized_vertices.get() + i * quantized_vertex_size, vertices_[i]);

  unsigned guessed_size = meshopt_encodeVertexBufferBound(vertices_count_, quantized_vertex_size);
  encoded_vertices_.data_.reset(new uint8_t[guessed_size]);
  encoded_vertices_.size_ = meshopt_encodeVertexBuffer(
      encoded_vertices_.data_.get(), guessed_size,
      quantized_vertices.get(), vertices_count_, quantized_vertex_size);
}

void mesh::gen_lods() {
  lod lod0;

  unsigned guessed_size = meshopt_encodeIndexBufferBound(indices_count_, sizeof(uint32_t));
  lod0.indices_count_ = indices_count_;
  lod0.data_.data_.reset(new uint8_t[guessed_size]);
  lod0.data_.size_ = meshopt_encodeIndexBuffer(
      lod0.data_.data_.get(), guessed_size,
      indices_.get(), indices_count_);

  encoded_lods_.emplace_back(std::move(lod0));
  // TODO Generate more LODs with meshopt_simplify
}

size_t mesh::layout(size_t _offset) {
  encoded_vertices_.offset_ = _offset;
  size_t size = encoded_vertices_.size_;
  for (auto &lod : encoded_lods_) {
    lod.data_.offset_ = _offset + size;
    size += lod.data_.size_;
  }
  return size;
}

void mesh::dump(context &_ctx, std::ostream &_os) {
  write_vec3(_os, translate_);
  write_vec3(_os, scale_);
  write_u16(_os, vertices_count_);
  write_uleb128(_os, encoded_vertices_.size_);
  write_uleb128(_os, encoded_vertices_.offset_);
  size_t offset = encoded_vertices_.offset_;
  memcpy(_ctx.data_.get() + offset, encoded_vertices_.data_.get(), encoded_vertices_.size_);
  offset += encoded_vertices_.size_;

  _os.put((uint8_t)encoded_lods_.size());
  for (auto &lod : encoded_lods_) {
    write_uleb128(_os, lod.indices_count_);
    write_uleb128(_os, lod.data_.size_);
    write_uleb128(_os, lod.data_.offset_);
    memcpy(_ctx.data_.get() + offset, lod.data_.data_.get(), lod.data_.size_);
    offset += lod.data_.size_;
  }
}
