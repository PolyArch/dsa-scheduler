#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/port.h"

namespace dsa {
namespace dfg {

const char* MetaPort::DataText[] = {
    "memory",
    "spad",
    "localport",
    "remoteport",
};

const char* MetaPort::OperationText[] = {
    "read", "write", "indread", "indwrite", "atomic",
};

CompileMeta::CompileMeta(const MetaPort& meta, VectorPort* parent)
    : MetaPort(meta), parent(parent) {
  CHECK(parent);
  if (dest == Data::LocalPort && !dest_port.empty()) {
    bool found = false;
    for (auto iv : parent->ssdfg()->type_filter<dsa::dfg::InputPort>()) {
      if (iv.name() == dest_port) {
        destination = &iv;
        found = true;
      }
    }
    CHECK(found) << "No destination found!";
  } else {
    destination = nullptr;
  }
}

VectorPort::VectorPort(V_TYPE v, int len, int bitwidth, const std::string& name,
                       SSDfg* ssdfg, const MetaPort& meta_)
    : Node(ssdfg, v, name), bitwidth_(bitwidth), meta(meta_, this) {}

InputPort::InputPort(int len, int width, const std::string& name, SSDfg* ssdfg,
                     const dsa::dfg::MetaPort& meta)
    : VectorPort(V_INPUT, len, width, name, ssdfg, meta) {
  int n = std::max(1, len);
  for (int i = 0; i < n; ++i) {
    values.emplace_back(ssdfg, id(), i);
  }
}

void InputPort::forward() {
  if (is_temporal()) {
    if (!values[current_].forward(true)) {
      return;
    }
    values[current_].forward(false);
    current_ = (current_ + 1) % ((int)values.size());
    return;
  }

  for (auto& elem : values) {
    if (!elem.forward(true)) return;
  }
  for (auto& elem : values) {
    CHECK(elem.forward(false));
  }
}

bool InputPort::can_push() {
  for (auto& elem : values) {
    if (!elem.fifo.empty()) {
      return false;
    }
  }
  return true;
}

void OutputPort::pop(std::vector<uint64_t>& data, std::vector<bool>& data_valid) {
  for (auto& operand : _ops) {
    data.push_back(operand.poll());
    data_valid.push_back(operand.predicate());
    operand.pop();
  }
}

int OutputPort::slot_for_op(dsa::dfg::Edge* edge, int node_slot) {
  // need to figure out which operand, then count bits within that operand
  for (auto& op : ops()) {
    int slot = 0;
    for (int eid : op.edges) {
      if (eid == edge->id) return slot;
      slot += _ssdfg->edges[eid].bitwidth() / 8;
    }
  }
  CHECK(false) << "edge not present in any operands";
  throw;
}

bool OutputPort::can_pop() {
  int j = 0;
  for (auto& elem : _ops) {
    ++j;
    if (!elem.ready()) {
      DSA_LOG(FORWARD) << ssdfg()->cur_cycle() << ": Cannot pop because of operand " << j;
      return false;
    }
  }
  return true;
}

}  // namespace dfg
}  // namespace dsa
