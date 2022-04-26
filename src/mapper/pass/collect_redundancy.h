#include <tuple>
#include <vector>

#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {
namespace pass {

inline std::tuple<std::vector<std::vector<Edge*>>, std::vector<std::vector<Edge*>>>
CollectRedundancy(SSDfg* dfg) {
  struct RedundancyCollector : Visitor {
    std::vector<std::vector<Edge*>> operands;
    std::vector<std::vector<Edge*>> users;
    SSDfg* dfg;

    RedundancyCollector(SSDfg* dfg)
        : operands(dfg->nodes.size()), users(dfg->nodes.size()), dfg(dfg) {}

    void Visit(Node* node) {
      DSA_LOG(COLLECT) << "Collecting redundancy for " << node->name();
      DSA_LOG(COLLECT) << "Operands: " << node->ops().size();
      DSA_LOG(COLLECT) << "Values: " << node->values.size();
      for (auto& op : node->ops()) {
        for (auto eid : op.edges) {
          auto* edge = &dfg->edges[eid];
          operands[node->id()].push_back(edge);
          DSA_LOG(COLLECT) << node->name() << " <- " << eid;
          DSA_LOG(COLLECT) << edge->name();
        }
      }
      for (auto& value : node->values) {
        for (auto eid : value.uses) {
          auto* edge = &dfg->edges[eid];
          users[node->id()].push_back(edge);
          DSA_CHECK(edge->parent == node->ssdfg())
              << edge->parent << " " << node->ssdfg() << " " << dfg;
          DSA_CHECK(edge->def() == node);
          DSA_LOG(COLLECT) << node->name() << " -> " << eid;
          DSA_LOG(COLLECT) << edge->name();
        }
      }
    }
  } 
  rc(dfg);

  dfg->Apply(&rc);

  return {rc.operands, rc.users};
}

}  // namespace pass
}  // namespace dfg
}  // namespace dsa