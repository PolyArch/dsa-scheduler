#include <tuple>
#include <vector>

#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {
namespace pass {

inline void CollectRedundancy(
  SSDfg* dfg,
  std::vector<std::vector<Edge*>> &operands,
  std::vector<std::vector<Edge*>> &users) {

  struct RedundancyCollector : Visitor {
    std::vector<std::vector<Edge*>> &operands;
    std::vector<std::vector<Edge*>> &users;
    SSDfg* dfg;

    RedundancyCollector(
      SSDfg* dfg, std::vector<std::vector<Edge*>> &operands_,
      std::vector<std::vector<Edge*>> &users_) : operands(operands_), users(users_), dfg(dfg) {
      operands.resize(dfg->nodes.size(), std::vector<Edge*>());
      users.resize(dfg->nodes.size(), std::vector<Edge*>());
    }

    void Visit(Node* node) {
      DSA_LOG(COLLECT) << "Collecting redundancy for " << node->name();
      DSA_LOG(COLLECT) << "Operands: " << node->ops().size();
      DSA_LOG(COLLECT) << "Values: " << node->values.size();
      for (auto& op : node->ops()) {
        for (auto eid : op.edges) {
          auto* edge = &dfg->edges[eid];
          DSA_CHECK(node->id() >= 0 && node->id() < operands.size());
          operands[node->id()].push_back(edge);
          DSA_LOG(COLLECT) << node->name() << " <- " << eid;
          DSA_LOG(COLLECT) << edge->name();
        }
      }
      for (auto& value : node->values) {
        for (auto eid : value.uses) {
          auto* edge = &dfg->edges[eid];
          DSA_CHECK(node->id() >= 0 && node->id() < users.size());
          users[node->id()].push_back(edge);
          DSA_CHECK(edge->parent == node->ssdfg())
              << edge->parent << " " << node->ssdfg() << " " << dfg;
          DSA_CHECK(edge->def() == node);
          DSA_LOG(COLLECT) << node->name() << " -> " << eid;
          DSA_LOG(COLLECT) << edge->name();
        }
      }
    }
  } rc(dfg, operands, users);

  dfg->Apply(&rc);
}

}  // namespace pass
}  // namespace dfg
}  // namespace dsa
