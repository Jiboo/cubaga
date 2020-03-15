#pragma once

#include "utils.hpp"

struct options {
  vector<path> inputs_;
  path output_;

  void parse(int _argc, char** _argv);
};
