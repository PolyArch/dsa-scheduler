#include "dsa/dfg/instruction.h"
#include <vector>

#include "../utils/vector_utils.h"
#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {

int CtrlBits::entryIdx2lutIdx(int entryIdx) {
  // Calculate the log2 
  auto log2 = [](int val) {
    return ((val) ? ((31) - __builtin_clz((uint32_t)(val))) : 0);
  };
  // Extract true from BMSS
  std::vector<int> extracted_bits;
  int tempBMSS = bmss;
  while (tempBMSS) {
    auto bit = tempBMSS & -tempBMSS; // lsb 1
    int idx = log2(bit);
    extracted_bits.push_back(idx);
    tempBMSS -= bit;
  }
  int lutIdx = 0;
  for (int i = 0; i < extracted_bits.size(); ++i) {
    if (entryIdx >> extracted_bits[i] & 1) {
      lutIdx |= (1 << i);
    }
  }
  // DSA_INFO << "Entry Idx = " << entryIdx << ", BMSS = " << bmss << ", LUTIdx = " << lutIdx;
  DSA_CHECK(bmss > 0) << "BMSS is not positive, means that control is not enabled, 
    but the function to dump bitstream for control is called";
  DSA_CHECK(lutIdx >=0 && lutIdx < 8) << "LUT index is not legal : " << lutIdx;
  return lutIdx;

}

void CtrlBits::test(uint64_t val, CtrlBits::Behavior &b) {
  val &= bmss;
  auto iter = lut.find(val);
  if (iter == lut.end()) return;
  auto f = [iter](CtrlBits::Control cc) {
    auto enum_iter = std::find(iter->second.begin(), iter->second.end(), cc);
    return enum_iter != iter->second.end();
  };
  b.backpressure[0] = b.backpressure[0] || f(CtrlBits::B1);
  b.backpressure[1] = b.backpressure[1] || f(CtrlBits::B2);
  b.discard = b.discard || f(CtrlBits::Discard);
  b.exec = b.exec && !f(CtrlBits::Abstain);
  b.reset = b.reset || f(CtrlBits::Reset);
}

std::vector<int> CtrlBits::encode() {
  std::vector<int> res;
  for (auto &elem : lut) {
    res.push_back(elem.first);
    int v = 0;
    for (auto &c : elem.second) {
      v |= 1 << c;
    }
    res.push_back(v);
  }
  return res;
}

CtrlBits::CtrlBits(const std::vector<int> &a) {
  DSA_CHECK(a.size() % 2 == 0);
  for (int i = 0; i < (int) a.size(); i += 2) {
    std::vector<CtrlBits::Control> v;
    for (int j = 0; j < CtrlBits::Control::Total; ++j) {
      if (a[i + 1] >> j & 1) {
        v.push_back((CtrlBits::Control) j);
      }
    }
    lut[a[i]] = v;
  }
}

std::string CtrlBits::toString() const {
  std::ostringstream oss;
  oss << "{";
  for (auto &elem : lut) {
    oss << "(" << elem.first << ":";
    bool first = true;
    for (auto c : elem.second) {
      if (!first) {
        oss << ",";
      }
      oss << c;
      first = false;
    }
    oss << ") ";
  }
  oss << "}";
  return oss.str();
}

CtrlBits::CtrlBits(const std::map<int, std::vector<std::string>>& raw, int bmss_) : bmss(bmss_) {
  for (auto& elem : raw) {
    std::vector<CtrlBits::Control> v;
    for (auto& s : elem.second) {
      auto e = str_to_enum(s);
      if (std::find(v.begin(), v.end(), e) == v.end()) {
        v.push_back(e);
        if (e == CtrlBits::B1 || e == CtrlBits::B2) {
          const_cast<bool&>(is_dynamic) = true;
        }
      }
    }
    lut[elem.first] = v;
  }
}

Instruction::Instruction(SSDfg* ssdfg, dsa::OpCode inst)
    : Node(ssdfg, V_INST), _reg(8, 0), opcode(inst) {
  DSA_CHECK(values.empty());
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
    DSA_LOG(COMP) << "Throughput: " << _ssdfg->cur_cycle() << ", " << last_execution << ", "
              << inst_throughput;
    return;
  }

  // Check the avaiability of output buffer
  if (!values.front().fifo.empty()) {
    for (auto& elem : values) {
      DSA_CHECK(!elem.fifo.empty());
      if (!elem.forward(true)) {
        DSA_LOG(FORWARD) << _ssdfg->cur_cycle() << ": " << name()
                     << " Cannot forward because of " << elem.name();
        return;
      }
    }
    for (auto& elem : values) {
      DSA_CHECK(elem.forward(false));
    }
    return;
  }

  DSA_CHECK(_ops.size() <= 3);

  // Check all the operands ready to go!
  for (size_t i = 0; i < _ops.size(); ++i) {
    if (!_ops[i].ready()) {
      DSA_LOG(FORWARD)
        << ssdfg()->cur_cycle() << ": " << name() << " Cannot forward since "
        << (i + 1) << " th op not ready ";
      return;
    }
  }

  CtrlBits::Behavior bh(_ops.size());
  uint64_t output = 0;

  std::ostringstream reason;
  std::ostringstream compute_dump;

  compute_dump << id() << ": " << dsa::name_of_inst(inst()) << "(";

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
      DSA_LOG(COMP) << "Register " << a << " " << _reg[0];
    } else {
      _input_vals[i] = _ops[i].poll();
      if (!_ops[i].predicate()) {
        bh.discard = true;
        reason << "[operand " << i << " invalid]";
      } else if (_ops[i].type != dsa::dfg::OperandType::data) {
        DSA_LOG(PRED) << "bits: " << predicate.toString() << ", pred: " << _input_vals[i];
        predicate.test(_input_vals[i], bh);
      }
    }
    if (i) {
      compute_dump << ", ";
    }
    compute_dump << _input_vals[i];
  }
  compute_dump << ") = ";


  if (bh.exec) {  // IF VALID
    ++_ssdfg->dyn_issued[is_temporal()];
    // Read in some temp value and set _val after inst_lat cycles
    output = do_compute(bh.discard, bh.backpressure);
    self_predicate.test(output, bh);
    for (int i = 0; i < (int) _output_vals.size(); ++i) {
      if (values[i].reg != -1) {
        int idx = values[i].reg;
        int bytes = bitwidth() / 8;
        memcpy(((uint8_t*)&_reg[values[i].reg]) + idx * bytes, &output, bytes);
        DSA_LOG(COMP) << "Write " << output << " to register" << values[i].reg;
      }
    }
    compute_dump << output;
  } else {
    compute_dump << "???";
    _output_vals.resize(values.size());
  }

  if (bh.reset) {
    for (size_t i = 0; i < _reg.size(); ++i) {
      _reg[i] = 0;
    }
  }

  compute_dump << (bh.exec ? std::string(" [valid]") : (" [invalid, " + reason.str() + "]"));
  if (bh.discard) {
    compute_dump << (bh.discard ? " [and output discard!]" : " ");
  }

  for (size_t i = 0; i < _ops.size(); ++i) {
    if (i < bh.backpressure.size() && bh.backpressure[i]) {
      compute_dump << " [backpressure " << i << "]";
    } else {
      compute_dump << " [pop " << i << "]";
      _ops[i].pop();
    }
  }

  // TODO(@were): We need a better name for this flag.
  if (bh.exec) {
    for (size_t i = 0; i < values.size(); ++i) {
      values[i].push(_output_vals[i], !bh.discard, lat_of_inst());
      compute_dump << " [pushed to value " << i << ", " << lat_of_inst() << "]";
    }
  }

  DSA_LOG(COMP) << compute_dump.str();

  for (auto& elem : values) {
    if (!elem.forward(true)) return;
  }

  for (auto& elem : values) {
    DSA_CHECK(elem.forward(false));
  }
}

std::string Instruction::name() {
  std::stringstream ss;
  ss << _name << "(" << dsa::name_of_inst(opcode) << " " << id() << ")";
  return ss.str();
}

int Instruction::lane() {
  if (lane_ != -1) {
    return lane_;
  }
  if (values.size() == 1) {
    const std::string &name = values[0].symbol;
    int i = name.size() - 1;
    while (i >= 0 && isdigit(name[i])) {
      --i;
    }
    DSA_CHECK(i >= 0) << name;
    std::string laneno(name.begin() + i + 1, name.end());
    if (!laneno.empty()) {
      std::istringstream iss(laneno);
      int res;
      if (iss >> res) {
        return lane_ = res;
      }
    }
    return lane_ = -2;
  }
  return -1;
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

  DSA_CHECK(false) << "Weird bitwidth: " << bitwidth() << "\n";
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
