#include <vector>

#include "dsa/arch/fabric.h"

namespace dsa {
namespace arch {
namespace pass {

inline std::vector<std::vector<int>> ShortestPaths(SpatialFabric* fabric) {
  int n = fabric->node_list().size();
  std::vector<std::vector<int>> res(n, std::vector<int>(n, 1e9));
  for (auto elem : fabric->node_list()) {
    res[elem->id()][elem->id()] = 0;
  }
  for (int i = 0; i < n; ++i) {
    std::priority_queue<std::tuple<int, int>> q;
    q.emplace(0, i);
    while (!q.empty()) {
      int src_id = -1;
      ssnode* src = nullptr;
      int dist = 0;
      do {
        auto poll = q.top();
        q.pop();
        dist = -std::get<0>(poll);
        src_id = std::get<1>(poll);
        src = fabric->node_list()[src_id];
        LOG(SHORT) << src_id << " " << res[i][src_id] << " " << src->name();
      } while (res[i][src_id] != dist);
      for (auto link : src->out_links()) {
        LOG(SHORT) << res[i][src->id()] << " " << res[i][link->sink()->id()];
        if (res[i][src->id()] + 1 < res[i][link->sink()->id()]) {
          res[i][link->sink()->id()] = res[i][src->id()] + 1;
          LOG(SHORT) << res[i][link->sink()->id()];
          q.emplace(-res[i][link->sink()->id()], link->sink()->id());
        }
      }
    }
  }
  return res;
}

}  // namespace pass
}  // namespace arch
}  // namespace dsa