#pragma once
#include <vector>

#include "dsa/arch/ssinst.h"
#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/visitor.h"

namespace dsa {
namespace dfg {
namespace pass {

/* \brief Return the throughput of each group. */
inline std::vector<int> GroupThroughput(SSDfg* dfg,
                                        const std::vector<SSDfgNode*>& reversed_topo) {
  std::vector<int> throughput(dfg->nodes.size(), 1);
  std::vector<int> res(dfg->num_groups(), 1);
  for (auto& inst : dfg->instructions) {
    throughput[inst.id()] = inst_thr(inst.inst());
  }
  for (auto node : reversed_topo) {
    for (auto& value : node->values) {
      for (auto& use : value.uses) {
        int uid = dfg->edges[use].uid;
        throughput[node->id()] = std::max(throughput[node->id()], throughput[uid]);
      }
    }
  }
  for (auto elem : dfg->nodes) {
    res[elem->group_id()] = std::max(res[elem->group_id()], throughput[elem->id()]);
  }
  return res;
}

}  // namespace pass
}  // namespace dfg
}  // namespace dsa