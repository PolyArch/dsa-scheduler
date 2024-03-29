#pragma once

#include <assert.h>

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "dsa/arch/ssinst.h"
#include "dsa/debug.h"

namespace dsa {

/*! \brief The function capability of each processing element. */
class Capability {
 public:
  /*! \brief Each FU entry of the capability. */
  struct Entry {
    /*! \brief The opcode of the instruction capability. */
    OpCode op;
    /*! \brief The number of this kind of */
    int count;
    Entry(OpCode op, int count) : op(op), count(count) {}
  };

  /*!
   * \brief Add a FU capability to this PE.
   * \param op The FU capability.
   * \param count The number of FU's.
   */
  void Add(OpCode op, int count) {
    DSA_CHECK(op != SS_ERR) << op;
    for (auto& elem : capability) {
      if (elem.op == op) {
        DSA_WARNING << name_of_inst(op) << " already added to " << name << "! "
                << "Instead, increase the count.";
        elem.count += count;
        return;
      }
    }
    capability.emplace_back(op, count);
    if ( dsa::num_ops[op] > max_num_operand) {
      max_num_operand =  dsa::num_ops[op];
    }
  }

  /*!
   * \brief Erase a FU capability.
   * \param j The index of the capability.
   */
  void Erase(int j) {
    bool recompute = dsa::num_ops[capability[j].op] == max_num_operand;
    capability.erase(capability.begin() + j);
    if (recompute) {
      int old = max_num_operand;
      max_num_operand = 0;
      for (auto& elem : capability) {
        if ( dsa::num_ops[elem.op] > max_num_operand) {
          max_num_operand =  dsa::num_ops[elem.op];
          if (max_num_operand == old) {
            break;
          }
        }
      }
    }
  }

  int get_encoding(OpCode op) {
    auto iter = std::find_if(capability.begin(), capability.end(),
                             [op](const Entry& entry) { return entry.op == op; });
    return iter == capability.end() ? -1 : iter - capability.begin();
  }

  bool Capable(OpCode op) {
    auto iter = std::find_if(capability.begin(), capability.end(),
                             [op](const Entry& entry) { return entry.op == op; });
    return iter != capability.end();
  }

  Capability() {}
  Capability(std::string name) : name(name) {}

  std::string name;
  int max_num_operand{0};
  std::vector<Entry> capability;

  // Area and Power
  double area();
  double power();
  double TotalLut();
  double LogicLut();
  double RamLut();
  double SRL();
  double FlipFlop();
  double RamB36();
  double RamB18();
  double URam();
  double DSP();
};

}  // namespace dsa
