#include "options.hpp"

#include <cstring>
#include <iostream>

bool options::parse(int _argc, char** _argv) {
  if (_argc < 3)
    return false;

  int offset = 1;
  while (strlen(_argv[offset]) >= 2 && strncmp(_argv[offset], "--", 2) == 0) {
    if (strcmp(_argv[offset], "--raw_textures") == 0) {
      raw_textures_ = true;
    }
    else {
      std::cerr << "ignoring unknown option " << _argv[offset] << std::endl;
    }
    offset++;
  }

  output_ = _argv[offset];
  for (unsigned i = offset + 1; i < _argc; i++)
    inputs_.emplace_back(_argv[i]);

  return true;
}
