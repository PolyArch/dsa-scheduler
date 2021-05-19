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

}