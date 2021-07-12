#include "dsa/dfg/instruction.h"

#include "../utils/vector_utils.h"
#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {

void CtrlBits::set(uint64_t val, Control b) {
  if (b == CtrlBits::B1 || b == CtrlBits::B2) {
    const_cast<bool&>(is_dynamic) = true;
  }

  int loc = val * Total + b;
  CHECK(loc >= 0 && loc < 64) << val << " * Total + " << b << " = " << loc;
  mask |= (1 << loc);
}

bool CtrlBits::test(uint64_t val, Control b) {
  int loc = val * Total + b;
  CHECK(loc >= 0 && loc < 64) << val << " * Total + " << b << " = " << loc;
  return (mask >> loc) & 1;
}

void CtrlBits::test(uint64_t val, CtrlBits::Behavior &b) {
  if (!mask) return;
  b.backpressure[0] = b.backpressure[0] || test(val, CtrlBits::B1);
  b.backpressure[1] = b.backpressure[1] || test(val, CtrlBits::B2);
  b.discard = b.discard || test(val, CtrlBits::Discard);
  b.predicate = b.predicate && !(test(val, CtrlBits::Abstain));
  b.reset = b.reset || test(val, CtrlBits::Reset);
  b.write = b.write || test(val, CtrlBits::Write);
}


CtrlBits::CtrlBits(const std::map<int, std::vector<std::string>>& raw) {
  for (auto& elem : raw)
    for (auto& s : elem.second) set(elem.first, str_to_enum(s));
}

Instruction::Instruction(SSDfg* ssdfg, dsa::OpCode inst)
    : Node(ssdfg, V_INST), _reg(8, 0), opcode(inst) {
  CHECK(values.empty());
  int n = dsa::num_values(opcode);
  for (int i = 0; i < n; ++i) {
    values.emplace_back(ssdfg, id(), i);
  }
}

int Instruction::bitwidth() {
  if (opcode != dsa::SS_NONE) {
    return dsa::bitwidth[opcode];
  }
  return ssdfg()->edges[ops()[0].edges[0]].bitwidth();
}

void Instruction::forward() {
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

  // Check all the operands ready to go!
  for (size_t i = 0; i < _ops.size(); ++i) {
    if (!_ops[i].ready()) {
      LOG(FORWARD) << ssdfg()->cur_cycle() << ": " << name() << " Cannot forward since "
                   << (i + 1) << " th op not ready ";
      return;
    }
  }

  CtrlBits::Behavior bh(_ops.size());
  uint64_t output = 0;

  std::ostringstream reason;
  std::ostringstream compute_dump;

  compute_dump << dsa::name_of_inst(inst()) << "(";

  _input_vals.resize(_ops.size(), 0);

  for (unsigned i = 0; i < _ops.size(); ++i) {
    if (_ops[i].isImm()) {
      _input_vals[i] = _ops[i].imm;
    } else if (_ops[i].type == OperandType::local_reg) {
      uint64_t a;
      int idx = _ops[i].regIdx();
      int bytes = (_ops[i].regDtype() / 8);
      memcpy(&a, ((uint8_t*)&_reg[0]) + idx * bytes, bytes);
      _input_vals[i] = a;
      LOG(COMP) << "Register " << a << " " << _reg[0];
    }else {
      _input_vals[i] = _ops[i].poll();
      if (!_ops[i].predicate()) {
        bh.predicate = false;
        reason << "operand " << i << " not valid!";
      } else if (_ops[i].type != dsa::dfg::OperandType::data) {
        LOG(PRED) << "bits: " << predicate.bits() << ", pred: " << _input_vals[i];
        predicate.test(_input_vals[i], bh);
      }
    }
    if (i) compute_dump << ", ";
    compute_dump << _input_vals[i];
  }
  compute_dump << ") = (" << name() << ") ";


  if (bh.predicate) {  // IF VALID

    _ssdfg->inc_total_dyn_insts(is_temporal());

    // Read in some temp value and set _val after inst_lat cycles
    output = do_compute(bh.discard, bh.backpressure);
    self_predicate.test(output, bh);
    if (bh.write) {
      int idx = 0;
      int bytes = bitwidth() / 8;
      memcpy(((uint8_t*)&_reg[0]) + idx * bytes, &output, bytes);
      LOG(COMP) << "Write " << output << " to register!";
    }

    compute_dump << output;

  } else {
    compute_dump << "???";
    _output_vals.resize(values.size());
  }

  // TODO/FIXME: change to all registers
  if (bh.reset) {
    for (size_t i = 0; i < _reg.size(); ++i) {
      _reg[i] = 0;
    }
  }

  LOG(COMP) << compute_dump.str() << (bh.predicate ? " valid " : " invalid ") << reason.str()
            << " " << (bh.discard ? " and output discard!" : "");

  for (size_t i = 0; i < bh.backpressure.size(); ++i) {
    if (bh.backpressure[i]) {
      LOG(COMP) << "backpressure on " << i << " input\n";
    } else {
      _ops[i].pop();
    }
  }

  // TODO(@were): We need a better name for this flag.
  if (bh.predicate) {
    for (size_t i = 0; i < values.size(); ++i) {
      values[i].push(_output_vals[i], !bh.discard, lat_of_inst());
    }
  }

  for (auto& elem : values) {
    if (!elem.forward(true)) return;
  }

  for (auto& elem : values) {
    CHECK(elem.forward(false));
  }
}

std::string Instruction::name() {
  std::stringstream ss;
  ss << _name << "(" << dsa::name_of_inst(opcode) << " " << id() << ")";
  return ss.str();
}

uint64_t Instruction::do_compute(bool& discard, std::vector<bool> &backpressure) {
  last_execution = _ssdfg->cur_cycle();

#define EXECUTE(bw)                                                                     \
  case bw: {                                                                            \
    std::vector<uint##bw##_t> input =                                                   \
        vector_utils::cast_vector<uint64_t, uint##bw##_t>(_input_vals);                 \
    std::vector<uint##bw##_t> outputs(values.size());                                   \
    output = dsa::execute##bw(opcode, input, outputs, (uint##bw##_t*)&_reg[0], discard, \
                              backpressure);                                            \
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

std::string Operation::name() {
  std::ostringstream oss;
  oss << "Operation<";
  for (int i = 0, n = opcodes.size(); i < n; ++i) {
    if (i) oss << ", ";
    oss << name_of_inst(opcodes[i]) << ":" << cnt[i];
  }
  oss << "> " << id();
  return oss.str();
}

int Operation::bitwidth() {
  int res = 0;
  for (int i = 0, n = opcodes.size(); i < n; ++i) {
    res = std::max(res, dsa::bitwidth[opcodes[i]]);
  }
  return res;
}

}  // namespace dfg
}  // namespace dsa
