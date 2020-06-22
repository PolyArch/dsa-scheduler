#pragma once

#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/visitor.h"

namespace dsa {
namespace dfg {
namespace pass {

inline std::vector<bool> PropagateControl(const std::vector<SSDfgNode*> &reversed_topo) {
  std::vector<bool> res(reversed_topo.size(), false);

  struct InstFilter : Visitor {
    InstFilter(std::vector<bool> &res_) : res(res_) {}
    std::vector<bool> &res;
    void Visit(SSDfgInst *inst) override {
      if (inst->ctrlBits().is_dynamic || inst->selfBits().is_dynamic) {
        res[inst->id()] = true;
      }
    }
  } filter(res);

  for (auto elem : reversed_topo) {
    elem->Accept(&filter);
  }

  struct ControlPropagate : Visitor {
    ControlPropagate(std::vector<bool> &res_) : res(res_) {}
    std::vector<bool> &res;
    void Visit(SSDfgNode *node) override {
      if (res[node->id()]) {
        for (auto &op : node->ops()) {
          for (auto edge : op.edges) {
            res[edge->def()->id()] = true;
          }
        }
      }
    }
  } propagate(res);

  for (auto elem : reversed_topo) {
    elem->Accept(&propagate);
  }

  return res;
}

}
}
}