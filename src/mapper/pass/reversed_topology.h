#pragma once
#include <vector>

#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/visitor.h"

namespace dsa {
namespace dfg {
namespace pass {

inline void Dfs(SSDfgNode *node, std::vector<bool> &visited, std::vector<SSDfgNode*> &order) {
  if (visited[node->id()]) {
    return;
  }

  visited[node->id()] = true;

  for (auto op : node->ops()) {
    for (auto eid : op.edges) {
      auto edge = &node->ssdfg()->edges[eid];
      Dfs(edge->def(), visited, order);
    }
  }

  order.push_back(node);
}

/* \brief Return the reversed topological order of the dataflow graph */
inline std::vector<SSDfgNode*> ReversedTopology(SSDfg *dfg) {
  struct Rooter : Visitor {
    Rooter(int n) : visited(n, false) {
      res.reserve(n);
    }
    std::vector<bool> visited;
    std::vector<SSDfgNode*> res;
    void Visit(SSDfgVecOutput *out) override {
      Dfs(out, visited, res);
    }
  };
  Rooter rooter(dfg->nodes.size());
  dfg->Apply(&rooter);
  return rooter.res;
}

}
}
}