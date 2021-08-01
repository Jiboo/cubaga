#include <cstring>

#include <iostream>
#include <fstream>

#include "utils.hpp"
#include "context.hpp"
#include "texture.hpp"
#include "material.hpp"
#include "mesh.hpp"
#include "options.hpp"

int main(int _argc, char** _argv) {
  context ctx;

  options &opt = ctx.opt_;
  if (!opt.parse(_argc, _argv)) {
    std::cerr << "couldn't parse command line" << std::endl;
    std::cerr << "usage: " << _argv[0] << " [options] <output> <inputs...>" << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "\t--raw_textures: disables BC7 compression" << std::endl;
    return EXIT_FAILURE;
  }

  for (const path &input : opt.inputs_) {
    cgltf_options options {};
    cgltf_data* rawdata = nullptr;
    if (cgltf_parse_file(&options, input.c_str(), &rawdata) != cgltf_result_success) {
      std::cerr << "error while parsing " << input << std::endl;
      return EXIT_FAILURE;
    }
    uptr<cgltf_data> data{rawdata, &cgltf_free};
    ctx.gltf_ = rawdata;

    if (cgltf_load_buffers(&options, data.get(), input.c_str()) != cgltf_result_success) {
      std::cerr << "error while loading " << input << std::endl;
      return EXIT_FAILURE;
    }

    if (cgltf_validate(data.get()) != cgltf_result_success) {
      std::cerr << "error while validating " << input << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "Processing gltf " << input << std::endl;
    ctx.clear_caches();

    for (unsigned i = 0; i < data->meshes_count; i++)
      ctx.meshes_.emplace_back().import(ctx, data->meshes[i]);
  }

  ctx.process();

  std::ofstream output(opt.output_, std::ios::binary);
  ctx.dump(output);
  return EXIT_SUCCESS;
}
