#pragma once

#include "utils.hpp"

struct options {
  vector<path> inputs_;
  path output_;
  bool raw_textures_ = false;

  bool parse(int _argc, char** _argv);
};
