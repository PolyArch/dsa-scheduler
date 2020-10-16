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

class Capability {
 public:
  struct Entry {
    OpCode op;
    int encoding;
    bool count;
    Entry(OpCode op, int encoding, bool count) : op(op), encoding(encoding), count(count) {}
  };

  void Add(OpCode op, int encoding, bool count = true) {
    for (auto &elem : capability) {
      CHECK(elem.encoding != encoding) << elem.encoding << ", Op encoding already occupied!";
    }
    for (auto &elem : capability) {
      if (elem.op == op) {
        // std::cerr << "Warning: " << name_of_inst(op) << " already added to " << name << std::endl;
        return;
      }
    }
    capability.emplace_back(op, encoding, count);
  }

  void Erase(int j) {
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

  // Area and Power
  double area();
  double power();
};

}  // namespace dsa
