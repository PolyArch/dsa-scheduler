#include "dsa/dfg/ssdfg.h"

#include <iomanip>
#include <list>
#include <set>
#include <string>
#include <vector>

#include "../utils/model_parsing.h"
#include "../utils/vector_utils.h"
#include "dfg-parser.tab.h"
#include "dsa/dfg/instruction.h"
#include "dsa/dfg/utils.h"
#include "dsa/dfg/visitor.h"
#include "dsa/mapper/schedule.h"

using namespace std;
using namespace dsa;

/// { SSDfgNode

/// }

/// { SSDfg

void SSDfg::check_for_errors() {
  struct ErrorChecker : dsa::dfg::Visitor {
    bool HasUse(dsa::dfg::Node* node) {
      bool ok = false;
      for (auto& elem : node->values) {
        ok |= !elem.uses.empty();
      }
      return ok;
    }
    bool HasOperands(dsa::dfg::Node* node) {
      bool ok = false;
      for (auto& elem : node->ops()) {
        ok |= !elem.edges.empty();
      }
      return ok;
    }
    void Visit(dsa::dfg::InputPort* vi) {
      CHECK(HasUse(vi)) << "No user on input " << vi->name();
    }
    void Visit(dsa::dfg::OutputPort* vo) {
      CHECK(HasOperands(vo)) << "No operand on output " << vo->name();
    }
    void Visit(dsa::dfg::Instruction* inst) {
      CHECK(HasUse(inst)) << "No user on instruction " << inst->name();
      CHECK(HasOperands(inst)) << "No operand on instruction " << inst->name();
    }
  };
  ErrorChecker ec;
  Apply(&ec);
}

void SSDfg::normalize() {
  nodes.resize(instructions.size() + vins.size() + vouts.size() + operations.size());
#define NORMALIZE_IMPL(a)       \
  do {                          \
    for (auto& elem : a) {      \
      nodes[elem.id()] = &elem; \
    }                           \
  } while (false)
  NORMALIZE_IMPL(instructions);
  NORMALIZE_IMPL(operations);
  NORMALIZE_IMPL(vins);
  NORMALIZE_IMPL(vouts);
#undef NORMALIZE_IMPL
}

SSDfg::SSDfg() {}

void SSDfg::set_pragma(const std::string& c, const std::string& s) {
  if (c == string("dfg")) {
    cout << "No pragmas yet for dfg\n";
  } else if (c == string("group")) {
    if (s == "temporal") {
      CHECK(!_groupProps.empty());
      _groupProps.back().is_temporal = true;
    }
  } else if (c == "frequency" || c == "unroll") {
    std::istringstream iss(s);
    auto& ref =
        c == "frequency" ? _groupProps.back().frequency : _groupProps.back().unroll;
    iss >> ref;
  } else {
    cout << "Context \"" << c << "\" not recognized.";
  }
}

void SSDfg::start_new_dfg_group() { _groupProps.emplace_back(GroupProp()); }

void SSDfg::create_new_task_dependence_map(int s, int d) {
  _current_src_grp=s;
  _current_dst_grp=d;
}

void SSDfg::add_new_task_dependence_map(std::vector<std::string> producer, std::vector<std::string> consumer) { 
  _dependence_maps[_current_src_grp][_current_dst_grp].push_back(make_pair(producer, consumer)); 
}

task_def_t SSDfg::producer_consumer_map(int src_group, int dst_group) {
  return _dependence_maps[src_group][dst_group];
}

SSDfg::SSDfg(string filename_) : filename(filename_) {
  string line;
  start_new_dfg_group();
  parse_dfg(filename_.c_str(), this);
  check_for_errors();
}

/// }

using dsa::SpatialFabric;

void SSDfg::Apply(dsa::dfg::Visitor* visitor) {
  for (auto elem : nodes) {
    elem->Accept(visitor);
  }
}

int SSDfg::forward(bool asap) {
  clear_issued();
  std::vector<bool> group_ready(true, num_groups());
  int old[2] = {dyn_issued[0], dyn_issued[1]};
  if (!asap) {
    for (auto& node : nodes) {
      if (!group_ready[node->group_id()]) {
        continue;
      }
      if (auto vi = dynamic_cast<dsa::dfg::InputPort*>(node)) {
        bool ready = true;
        for (auto& value : vi->values) {
          if (!value.forward(true)) {
            ready = false;
            break;
          }
        }
        if (!ready) {
          group_ready[node->group_id()] = false;
        }
      }
    }
  }
  for (auto elem : nodes) {
    if (auto vec = dynamic_cast<dsa::dfg::VectorPort*>(elem)) {
      if (asap || group_ready[elem->group_id()]) {
        elem->forward();
      }
    } else {
      elem->forward();
    }
  }
  ++_cur_cycle;
  return (dyn_issued[0] - old[0]) + (dyn_issued[1] - old[1]);
}

SSDfg::SSDfg(const SSDfg& dfg)
    : filename(dfg.filename),
      instructions(dfg.instructions),
      operations(dfg.operations),
      vins(dfg.vins),
      vouts(dfg.vouts),
      edges(dfg.edges),
      _groupProps(dfg._groupProps) {
  normalize();
  for (auto node : nodes) {
    node->ssdfg() = this;
    for (auto& value : node->values) {
      value.parent = this;
    }
    for (auto& op : node->ops()) {
      op.parent = this;
    }
  }
  for (auto& edge : edges) {
    edge.parent = this;
  }
}
