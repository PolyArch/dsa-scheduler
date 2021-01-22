#pragma once
#include <vector>

#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/visitor.h"

namespace dsa {
namespace dfg {
namespace pass {

inline void Dfs(Node* node, std::vector<bool>& visited, std::vector<Node*>& order) {
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
inline std::vector<Node*> ReversedTopology(SSDfg* dfg) {
  struct Rooter : Visitor {
    Rooter(int n) : visited(n, false) { res.reserve(n); }
    std::vector<bool> visited;
    std::vector<Node*> res;
    void Visit(OutputPort* out) override { Dfs(out, visited, res); }
  };
  Rooter rooter(dfg->nodes.size());
  dfg->Apply(&rooter);
  return rooter.res;
}

}  // namespace pass
}  // namespace dfg
}  // namespace dsa