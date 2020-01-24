#ifndef __SS_FU_MODEL_H__
#define __SS_FU_MODEL_H__

#include <assert.h>

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "ssinst.h"

namespace SS_CONFIG {

class Capability {
 public:
  struct Entry {
    OpCode op;
    int encoding;
    bool count;
    Entry(OpCode op, int encoding, bool count) : op(op), encoding(encoding), count(count) {}
  };

  void Add(OpCode op, int encoding, bool count = true) {
    power_ = area_ = -114514.1919810;
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

  bool Capable(OpCode op) {
    for (auto &elem : capability) {
      if (elem.op == op)
        return true;
    }
    return false;
  }

  Capability(std::string name) : name(name) {}

  std::string name;
  std::vector<Entry> capability{Entry(SS_Copy, 1, true)};

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

std::vector<Capability> ParseFuType(std::istream& istream);

}  // namespace SS_CONFIG

#endif
