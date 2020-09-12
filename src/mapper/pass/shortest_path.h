#include <vector>

#include "dsa/arch/fabric.h"

namespace dsa {
namespace arch {
namespace pass {

inline std::vector<std::vector<int>>
ShortestPaths(SpatialFabric *fabric) {
  int n = fabric->node_list().size();
  std::vector<std::vector<int>> res(n, std::vector<int>(n, 1e6));
  for (auto elem : fabric->node_list()) {
    res[elem->id()][elem->id()] = 0;
    for (auto link : elem->out_links()) {
      res[elem->id()][link->dest()->id()] = 1;
    }
  }
  for (int k = 0; k < n; ++k) {
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        if (res[i][k] + res[k][j] < res[i][j]) {
          res[i][j] = res[i][k] + res[k][j];
        }
      }
    }
  }
  return res;
}


}
}
}