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

class func_unit_def {
 public:
  func_unit_def(std::string name_in) {
    _name = name_in;
    add_cap(ss_inst_t::SS_Copy);  // All FUs can copy!
  }

  std::string name() { return _name; }

  void add_cap(ss_inst_t ss_inst) {
    _area_dirty_bit = true;
    _power_dirty_bit = true;
    _accountability_dirty_bit = true;
    _account_for_hw.insert(std::pair<ss_inst_t, bool>(ss_inst, true));
    _cap.insert(ss_inst);
  }
  void set_encoding(ss_inst_t ss_inst, unsigned i) {
    if (i == 0) {
      assert(0 && "Encoding for Instruction cannot be zero.  Zero is reserved for Blank");
    }
    if (i == 1) {
      assert(0 && "Encoding for Instruction cannot be 1.  1 is reserved for Copy");
    }
    _cap2encoding[ss_inst] = i;
    _encoding2cap[i] = ss_inst;
  }

  bool is_cap(ss_inst_t inst) { return _cap.count(inst) > 0; }
  unsigned encoding_of(ss_inst_t inst) {
    if (inst == SS_Copy) {
      return 1;
    } else {
      return _cap2encoding[inst];
    }
  }

  ss_inst_t inst_of_encoding(unsigned i) {
    if (i == 1) {
      return SS_Copy;
    }
    assert(_encoding2cap.count(i));
    return _encoding2cap[i];
  }
  std::set<ss_inst_t> cap() { return _cap; }
  int num_inst() { return _cap.size(); }

  // Area and Power
  bool is_accountable_for_fw(ss_inst_t inst) { return _account_for_hw[inst]; }

  double area();
  double power();

 private:
  void determine_inst_accountability();
  double get_area_from_inst_set();
  double get_power_from_inst_set();
  std::set<ss_inst_t> find_all_insts_covered(ss_inst_t source_inst);
  // this map a instruction's function name (like the function name of FxMul16x2 is Mul)
  // to its group, like Add -> {Acc, Sub, Add}, FDiv -> {FDiv, FSqrt}
  std::map<std::string, std::set<std::string>> _inst_cover_group_map{
      {"Add", {"Add", "Sub", "Acc"}},     {"Sub", {"Add", "Sub", "Acc"}},
      {"Acc", {"Add", "Sub", "Acc"}},     {"FDiv", {"FDiv", "FSqrt"}},
      {"FAdd", {"FAdd", "FSub", "FAcc"}}, {"FSub", {"FAdd", "FSub", "FAcc"}},
      {"FAcc", {"FAdd", "FSub", "FAcc"}}};
  std::string _name;
  std::set<ss_inst_t> _cap;
  std::map<ss_inst_t, bool> _account_for_hw;  // whether this instruction is account for
  // hardware overhead calculation (power & area), used for decoupled instruction:
  // like Mul32x2 is supported by Mul64, which means Mul32x2 is not accountable for
  // power & area calculation
  std::map<ss_inst_t, unsigned> _cap2encoding;
  std::map<unsigned, ss_inst_t> _encoding2cap;
  bool _area_dirty_bit = true;
  bool _power_dirty_bit = true;
  bool _accountability_dirty_bit = true;
  double _area;
  double _power;
  friend class FuModel;
};

class FuModel {
 public:
  FuModel() {}
  FuModel(std::istream& istream);
  func_unit_def* GetFUDef(char*);
  func_unit_def* GetFUDef(std::string& fu_string);

  std::vector<func_unit_def>& fu_defs() { return func_defs; }

 private:
  void AddCapabilities(func_unit_def& fu, std::string& cap_string);

  std::vector<func_unit_def> func_defs;
};

}  // namespace SS_CONFIG

#endif
