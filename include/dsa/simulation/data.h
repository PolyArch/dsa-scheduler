#pragma once

#include <utility>
#include <vector>

#include "dsa/arch/ssinst.h"

namespace dsa {
namespace simulation {

struct Data {
  uint64_t available_at;
  uint64_t value;
  bool valid;
  Data(uint64_t aa, uint64_t value, bool valid)
      : available_at(aa), value(value), valid(valid) {}
};

}  // namespace simulation
}  // namespace dsa