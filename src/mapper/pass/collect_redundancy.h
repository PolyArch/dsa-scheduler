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
      for (auto& op : node->ops()) {
        for (auto eid : op.edges) {
          auto* edge = &dfg->edges[eid];
          operands[node->id()].push_back(edge);
          LOG(COLLECT) << node->name() << " <- " << eid;
          LOG(COLLECT) << edge->name();
        }
      }
      for (auto& value : node->values) {
        for (auto eid : value.uses) {
          auto* edge = &dfg->edges[eid];
          users[node->id()].push_back(edge);
          CHECK(edge->parent == node->ssdfg())
              << edge->parent << " " << node->ssdfg() << " " << dfg;
          CHECK(edge->def() == node);
          LOG(COLLECT) << node->name() << " -> " << eid;
          LOG(COLLECT) << edge->name();
        }
      }
    }
  } rc(dfg);

  dfg->Apply(&rc);

  return {rc.operands, rc.users};
}

}  // namespace pass
}  // namespace dfg
}  // namespace dsa