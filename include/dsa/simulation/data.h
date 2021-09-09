#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "dsa/arch/ssinst.h"

namespace dsa {
namespace sim {

struct SpatialPacket {
  /*!
   * \brief The latency of this cycle to be produced.
   */
  uint64_t available_at;
  /*!
   * \brief The payload of this packet.
   */
  uint64_t value;
  /*!
   * \brief The data predicate of this packet.
   */
  bool valid;

  SpatialPacket(uint64_t aa, uint64_t value, bool valid)
      : available_at(aa), value(value), valid(valid) {}
};

}  // namespace simulation
}  // namespace dsa
