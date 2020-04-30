#pragma once

#include <algorithm>
#include <vector>

namespace dsa {
namespace vector_utils {

template<typename T>
inline int indexing(T v, std::vector<T> a) {
  auto iter = std::find(a.begin(), a.end(), v);
  if (iter != a.end()) {
    return iter - a.begin();
  }
  return -1;
}

}
}