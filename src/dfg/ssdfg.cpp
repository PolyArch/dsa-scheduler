#include "dsa/dfg/ssdfg.h"

#include <iomanip>
#include <list>
#include <set>
#include <string>
#include <vector>

#include "../utils/model_parsing.h"
#include "../utils/vector_utils.h"
#include "dfg-parser.tab.h"
#include "dsa/dfg/utils.h"
#include "dsa/dfg/visitor.h"
#include "dsa/mapper/schedule.h"

using namespace std;
using namespace dsa;

/// { SSDfgNode

uint64_t SSDfgNode::invalid() { return _invalid; }

bool SSDfgNode::is_temporal() { return _ssdfg->group_prop(_group_id).is_temporal; }

SSDfgNode::SSDfgNode(SSDfg* ssdfg, V_TYPE v, const std::string& name)
    : _ssdfg(ssdfg), _name(name), _vtype(v) {
  _ID = ssdfg->instructions.size() + ssdfg->vins.size() + ssdfg->vouts.size();
  _group_id = _ssdfg->num_groups() - 1;
}

dsa::dfg::Edge* SSDfgNode::getLinkTowards(SSDfgNode* to) {
  auto pred = [this, to](int eid) -> bool { return ssdfg()->edges[eid].use() == to; };
  for (auto& value : values) {
    auto& uses = value.uses;
    auto res = std::find_if(uses.begin(), uses.end(), pred);
    if (res != uses.end()) return &ssdfg()->edges[*res];
  }
  return nullptr;
}

/// }

/// { Parsing data structure

void CtrlBits::set(uint64_t val, Control b) {
  if (b == CtrlBits::B1 || b == CtrlBits::B2) {
    const_cast<bool&>(is_dynamic) = true;
  }

  int loc = val * Total + b;
  assert(loc >= 0 && loc < 64);
  mask |= (1 << loc);
}

bool CtrlBits::test(uint64_t val, Control b) {
  int loc = val * Total + b;
  assert(loc >= 0 && loc < 64);
  return (mask >> loc) & 1;
}

void CtrlBits::test(uint64_t val, std::vector<bool>& back_array, bool& discard,
                    bool& predicate, bool& reset) {
  if (!mask) return;
  back_array[0] = back_array[0] || test(val, CtrlBits::B1);
  back_array[1] = back_array[1] || test(val, CtrlBits::B2);
  discard = discard || test(val, CtrlBits::Discard);
  predicate = predicate && !(test(val, CtrlBits::Abstain));
  reset = reset || test(val, CtrlBits::Reset);
}

CtrlBits::CtrlBits(const std::map<int, std::vector<std::string>>& raw) {
  for (auto& elem : raw)
    for (auto& s : elem.second) set(elem.first, str_to_enum(s));
}

/// }

/// { SSDfgInst

std::string SSDfgInst::name() {
  std::stringstream ss;
  ss << _name << "(" << dsa::name_of_inst(opcode) << " " << id() << ")";
  return ss.str();
}

/// }

/// { SSDfg

void SSDfg::check_for_errors() {
  struct ErrorChecker : dsa::dfg::Visitor {
    bool HasUse(SSDfgNode* node) {
      bool ok = false;
      for (auto& elem : node->values) {
        ok |= !elem.uses.empty();
      }
      return ok;
    }
    bool HasOperands(SSDfgNode* node) {
      bool ok = false;
      for (auto& elem : node->ops()) {
        ok |= !elem.edges.empty();
      }
      return ok;
    }
    void Visit(SSDfgVecInput* vi) {
      CHECK(HasUse(vi)) << "No user on input " << vi->name();
    }
    void Visit(SSDfgVecOutput* vi) {
      CHECK(HasOperands(vi)) << "No operand on output " << vi->name();
    }
    void Visit(SSDfgInst* inst) {
      CHECK(HasUse(inst)) << "No user on instruction " << inst->name();
      CHECK(HasOperands(inst)) << "No operand on instruction " << inst->name();
    }
  };
  ErrorChecker ec;
  Apply(&ec);
}

void SSDfg::normalize() {
  nodes.resize(instructions.size() + vins.size() + vouts.size());
#define NORMALIZE_IMPL(a)       \
  do {                          \
    for (auto& elem : a) {      \
      nodes[elem.id()] = &elem; \
    }                           \
  } while (false)
  NORMALIZE_IMPL(instructions);
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
      assert(!_groupProps.empty());
      _groupProps[_groupProps.size() - 1].is_temporal = true;
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

SSDfg::SSDfg(string filename_) : filename(filename_) {
  string line;
  start_new_dfg_group();
  parse_dfg(filename_.c_str(), this);
  check_for_errors();
}

SSDfgVec::SSDfgVec(V_TYPE v, int len, int bitwidth, const std::string& name, SSDfg* ssdfg,
                   const dsa::dfg::MetaPort& meta_)
    : SSDfgNode(ssdfg, v, name), _bitwidth(bitwidth), _vp_len(len), meta(meta_, this) {
  _port_width = _bitwidth;
}

uint64_t SSDfgInst::do_compute(bool& discard) {
  last_execution = _ssdfg->cur_cycle();

#define EXECUTE(bw)                                                                     \
  case bw: {                                                                            \
    std::vector<uint##bw##_t> input =                                                   \
        vector_utils::cast_vector<uint64_t, uint##bw##_t>(_input_vals);                 \
    std::vector<uint##bw##_t> outputs(values.size());                                   \
    output = dsa::execute##bw(opcode, input, outputs, (uint##bw##_t*)&_reg[0], discard, \
                              _back_array);                                             \
    outputs[0] = output;                                                                \
    _output_vals = vector_utils::cast_vector<uint##bw##_t, uint64_t>(outputs);          \
    return output;                                                                      \
  }

  uint64_t output;
  switch (bitwidth()) {
    EXECUTE(64)
    EXECUTE(32)
    EXECUTE(16)
    EXECUTE(8)
  }

#undef EXECUTE

  CHECK(false) << "Weird bitwidth: " << bitwidth() << "\n";
  throw;
}

/// }

using dsa::SpatialFabric;

void SSDfg::Apply(dsa::dfg::Visitor* visitor) {
  for (auto elem : nodes) {
    elem->Accept(visitor);
  }
}

SSDfgVecInput::SSDfgVecInput(int len, int width, const std::string& name, SSDfg* ssdfg,
                             const dsa::dfg::MetaPort& meta)
    : SSDfgVec(V_INPUT, len, width, name, ssdfg, meta) {
  int n = std::max(1, len / (64 / width));
  for (int i = 0; i < n; ++i) {
    values.emplace_back(ssdfg, id(), i);
  }
}

int SSDfgInst::bitwidth() {
  if (opcode != dsa::SS_NONE) {
    return dsa::bitwidth[opcode];
  }
  return ssdfg()->edges[ops()[0].edges[0]].bitwidth();
}

void SSDfgInst::forward() {
  int inst_throughput = inst_thr(inst());
  if (_ssdfg->cur_cycle() - last_execution < (uint64_t)inst_throughput) {
    LOG(COMP) << "Throughput: " << _ssdfg->cur_cycle() << ", " << last_execution << ", "
              << inst_throughput;
    return;
  }

  // Check the avaiability of output buffer
  if (!values.front().fifo.empty()) {
    for (auto& elem : values) {
      CHECK(!elem.fifo.empty());
      if (!elem.forward(true)) {
        LOG(FORWARD) << _ssdfg->cur_cycle() << ": " << name()
                     << " Cannot forward because of " << elem.name();
        return;
      }
    }
    for (auto& elem : values) {
      CHECK(elem.forward(false));
    }
    return;
  }

  CHECK(_ops.size() <= 3);

  _back_array.resize(_ops.size());
  std::fill(_back_array.begin(), _back_array.end(), 0);

  // Check all the operands ready to go!
  for (size_t i = 0; i < _ops.size(); ++i) {
    if (!_ops[i].ready()) {
      LOG(FORWARD) << ssdfg()->cur_cycle() << ": " << name() << " Cannot forward since "
                   << (i + 1) << " th op not ready ";
      return;
    }
  }

  bool discard(false), reset(false), pred(true);
  uint64_t output = 0;

  std::ostringstream reason;
  std::ostringstream compute_dump;

  compute_dump << dsa::name_of_inst(inst()) << "(";
  _invalid = false;

  _input_vals.resize(_ops.size(), 0);

  for (unsigned i = 0; i < _ops.size(); ++i) {
    if (_ops[i].is_imm()) {
      _input_vals[i] = _ops[i].imm;
    } else {
      _input_vals[i] = _ops[i].poll();
      if (!_ops[i].predicate()) {
        _invalid = true;
        reason << "operand " << i << " not valid!";
      } else if (_ops[i].type != dsa::dfg::OperandType::data) {
        LOG(PRED) << "bits: " << predicate.bits() << ", pred: " << _input_vals[i];
        predicate.test(_input_vals[i], _back_array, discard, pred, reset);
      }
    }
    if (i) compute_dump << ", ";
    compute_dump << _input_vals[i];
  }
  compute_dump << ") = (" << name() << ") ";

  // we set this instruction to invalid
  if (!pred) {
    _invalid = true;
  }

  if (!_invalid) {  // IF VALID

    _ssdfg->inc_total_dyn_insts(is_temporal());

    // Read in some temp value and set _val after inst_lat cycles
    output = do_compute(discard);
    self_predicate.test(output, _back_array, discard, pred, reset);

    compute_dump << output;

  } else {
    compute_dump << "???";
    _output_vals.resize(values.size());
  }

  // TODO/FIXME: change to all registers
  if (reset) {
    for (size_t i = 0; i < _reg.size(); ++i) _reg[i] = 0;
  }

  LOG(COMP) << compute_dump.str() << (_invalid ? " invalid " : " valid ") << reason.str()
            << " " << (discard ? " and output discard!" : "");

  for (size_t i = 0; i < _back_array.size(); ++i) {
    if (_back_array[i]) {
      LOG(COMP) << "backpressure on " << i << " input\n";
    } else {
      _ops[i].pop();
    }
  }

  _invalid |= discard;

  // TODO(@were): We need a better name for this flag.
  if (!_invalid || !getenv("DSCDIVLD")) {
    for (size_t i = 0; i < values.size(); ++i) {
      values[i].push(_output_vals[i], !_invalid, lat_of_inst());
    }
  }
  // std::cerr << _ssdfg->cur_cycle() << ": " << name() << " (" << loc.first << ", " <<
  // loc.second << ") issued!" << std::endl;

  for (auto& elem : values) {
    if (!elem.forward(true)) return;
  }

  for (auto& elem : values) {
    assert(elem.forward(false));
  }
}

void SSDfgVecInput::forward() {
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
    assert(elem.forward(false));
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
      if (auto vi = dynamic_cast<SSDfgVecInput*>(node)) {
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
  for (auto elem : type_filter<SSDfgNode*>()) {
    if (auto vec = dynamic_cast<SSDfgVec*>(elem)) {
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

bool SSDfgVecInput::can_push() {
  for (auto& elem : values) {
    if (!elem.fifo.empty()) {
      return false;
    }
  }
  return true;
}

void SSDfgVecOutput::pop(std::vector<uint64_t>& data, std::vector<bool>& data_valid) {
  for (auto& operand : _ops) {
    data.push_back(operand.poll());
    data_valid.push_back(operand.predicate());
    operand.pop();
  }
}

int SSDfgVecOutput::slot_for_op(dsa::dfg::Edge* edge, int node_slot) {
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

bool SSDfgVecOutput::can_pop() {
  int j = 0;
  for (auto& elem : _ops) {
    ++j;
    if (!elem.ready()) {
      LOG(FORWARD) << ssdfg()->cur_cycle() << ": Cannot pop because " << j;
      return false;
    }
  }
  return true;
}

SSDfg::SSDfg(const SSDfg& dfg)
    : filename(dfg.filename),
      instructions(dfg.instructions),
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

CompileMeta::CompileMeta(const MetaPort& meta, SSDfgVec* parent)
    : MetaPort(meta), parent(parent) {
  assert(parent);
  if (dest == Data::LocalPort && !dest_port.empty()) {
    bool found = false;
    for (auto iv : parent->ssdfg()->type_filter<SSDfgVecInput>()) {
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

}  // namespace dfg
}  // namespace dsa
