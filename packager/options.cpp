#include "options.hpp"

void options::parse(int _argc, char** _argv) {
  if (_argc < 3)
    throw error("expected 3 or more args: <output> <inputs...>");

  output_ = _argv[1];
  for (unsigned i = 2; i < _argc; i++)
    inputs_.emplace_back(_argv[i]);
}
