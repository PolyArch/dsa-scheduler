#pragma once

#include <algorithm>
#include <map>
#include <tuple>

#include "dsa/dfg/ssdfg.h"

namespace cm {

inline int ColorOf(dsa::dfg::Value* val, bool reset = false) {
  auto* node = val->node();
  if (static_cast<int>(node->ops().size()) == 1 && node->ops()[0].edges.size() == 1) {
    auto res = val->parent->edges[node->ops()[0].edges[0]].val();
    return ColorOf(res);
  }
  static std::map<dsa::dfg::Node*, std::tuple<int, int, int>> colorMap;
  if (colorMap.count(node) == 0 || reset) {
    int x = 0, y = 0, z = 0;
    float lum = 0;
    while (lum < 0.36f || lum > 0.95f) {  // prevent dark colors
      //  || abs(x-y) + abs(y-z) + abs(z-x) < 100
      x = rand() % 256;
      y = rand() % 256;
      z = rand() % 256;
      lum = sqrt(x * x * 0.241f + y * y * 0.691f + z * z * 0.068f) / 255.0f;
      // lum = (x*0.299f+y*0.587f+z*0.114f)/255.0f;
    }
    colorMap[node] = {x, y, z};
  }
  auto rgb = colorMap[node];
  int r = std::max(std::get<0>(rgb) - val->index * 15, 0);
  int g = std::max(std::get<1>(rgb) - val->index * 10, 0);
  int b = std::max(std::get<2>(rgb) - val->index * 20, 0);
  return r | (g << 8) | (b << 16);
}

}  // namespace cm