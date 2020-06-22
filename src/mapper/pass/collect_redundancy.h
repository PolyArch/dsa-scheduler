#include <tuple>
#include <vector>

#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {
namespace pass {

inline
std::tuple<std::vector<std::vector<SSDfgEdge*>>,
           std::vector<std::vector<SSDfgEdge*>>>
CollectRedundancy(SSDfg *dfg) {

  struct RedundancyCollector : Visitor {
    std::vector<std::vector<SSDfgEdge*>> operands;
    std::vector<std::vector<SSDfgEdge*>> users;

    RedundancyCollector(int n) : operands(n), users(n) {}

    void Visit(SSDfgNode *node) {
      for (auto op : node->ops()) {
        for (auto elem : op.edges) {
          operands[node->id()].push_back(elem);
        }
      }
      for (auto value : node->values()) {
        for (auto elem : value->edges()) {
          users[node->id()].push_back(elem);
        }
      }
    }
  } rc(dfg->nodes<SSDfgNode*>().size());

  dfg->Apply(&rc);

  return {rc.operands, rc.users};
}

}
}
}