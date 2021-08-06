#include <cstdint>

#include "dsa/debug.h"
#include "dsa/core/singleton.h"

namespace dsa {

ContextFlags::ContextFlags() {
  LOG(CF) << "am i constructed?";
}

ContextFlags &ContextFlags::Global() {
  static ContextFlags *instance_ = new ContextFlags();
  return *instance_;
}

void ContextFlags::Load(const cxxopts::ParseResult &parsed) {
  this->dummy = parsed.count("dummy");
  this->verbose = parsed.count("verbose");
  this->timeout = parsed["timeout"].as<int>();
  this->bitstream = parsed["print-bitstream"].as<bool>();
  this->max_iters = parsed["max-iters"].as<int>();
  this->tolerate_unuse = parsed["tolerate-unuse"].as<bool>();
}

}
