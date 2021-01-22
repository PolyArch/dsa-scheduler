#pragma once

#include <cstdint>
#include <vector>

namespace dsa {
namespace adt {

/*!
 * \brief C++ bitset does not support compilation time variable
 *        length.
 */
struct BitVector {
  /*!
   * \brief Construct a new bit vector with the given length.
   */
  BitVector(int length_) : length(length_), values((length_ - 1) / 64 + 1, 0) {}

  bool operator==(const BitVector& b) const {
    if (values.size() != b.values.size()) {
      return false;
    }
    for (int i = 0, n = values.size(); i < n; ++i) {
      if (values[i] != b.values[i]) {
        return false;
      }
    }
  }

  void Set(int i) { values[i / 64] |= 1 << (i % 64); }

  void Reset(int i) { values[i / 64] &= (~0ull) ^ (1 << (i % 64)); }

  bool Get(int i) { return (values[i / 64] & (1 << (i % 64))) != 0; }

  bool All(int l, int r) {
    bool res = true;
    for (int i = l; i <= r; i += 64 - i % 64) {
      int width = std::min(64 - i % 64, r - i + 1);
      uint64_t full_mask = (~0ull) >> (64 - width);
      res = res && (full_mask & (values[i / 64] >> (i % 64))) == full_mask;
    }
    return res;
  }

  int length;
  std::vector<uint64_t> values;
};

}  // namespace adt
}  // namespace dsa