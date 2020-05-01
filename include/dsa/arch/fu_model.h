#pragma once

#include <assert.h>

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "ssinst.h"

namespace dsa {

class Capability {
 public:
  struct Entry {
    OpCode op;
    int encoding;
    bool count;
    Entry(OpCode op, int encoding, bool count) : op(op), encoding(encoding), count(count) {}
  };

  void Add(OpCode op, int encoding, bool count = true) {
    power_ = area_ = -1;
    for (auto &elem : capability) {
      assert(elem.encoding != encoding && "Op encoding already occupied!");
    }
    for (auto &elem : capability) {
      if (elem.op == op) {
        std::cerr << "Warning: " << name_of_inst(op) << " already added to " << name << std::endl;
        return;
      }
    }
    capability.emplace_back(op, encoding, count);
  }

  void Erase(int j) {
    power_ = area_ = -1;
    capability.erase(capability.begin() + j);
  }

  int get_encoding(OpCode op){
    auto iter = std::find_if(capability.begin(), capability.end(), [op](const Entry &entry) {
      return entry.op == op;
    });
    return iter == capability.end() ? -1 : iter->encoding;
  }

  bool Capable(OpCode op) {
    for (auto &elem : capability) {
      if (elem.op == op)
        return true;
    }
    return false;
  }

  Capability() {}
  Capability(std::string name) : name(name) {}

  std::string name;
  std::vector<Entry> capability{Entry(SS_Copy, 1, true)};

  uint get_max_num_operand(){
    int max_num_operand = -1;
    for(auto & elem: capability){
      if(num_ops[elem.op] > max_num_operand){
        max_num_operand = num_ops[elem.op];
      }
    }
    return max_num_operand;
  }

 private:
  double power_{-1.0};
  double area_{-1.0};
 public:

  // Area and Power
  double area();
  double power();

 private:
  void determine_inst_accountability();
  double get_area_from_inst_set();
  double get_power_from_inst_set();
  std::set<OpCode> find_all_insts_covered(OpCode source_inst);
  // this map a instruction's function name (like the function name of FxMul16x2 is Mul)
};

std::vector<Capability*> ParseFuType(std::istream& istream);

}  // namespace dsa