#pragma once

#include <algorithm>
#include <vector>

namespace dsa {
namespace vector_utils {

template <typename T>
inline int indexing(T v, std::vector<T> a) {
  auto iter = std::find(a.begin(), a.end(), v);
  if (iter != a.end()) {
    return iter - a.begin();
  }
  return -1;
}

template <typename T>
int count_unique(std::vector<T>& vec) {
  sort(vec.begin(), vec.end());
  return unique(vec.begin(), vec.end()) - vec.begin();
}

template <typename Tfrom, typename Tto>
std::vector<Tto> cast_vector(const std::vector<Tfrom>& a) {
  std::vector<Tto> res(a.size());
  for (size_t i = 0; i < a.size(); ++i) res[i] = a[i];
  return res;
}

}  // namespace vector_utils
}  // namespace dsa