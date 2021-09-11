#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "dsa/debug.h"
#include "dsa/dfg/metadata.h"
#include "dsa/dfg/node.h"

// struct SSDfgValue;
// struct SSDfgNode;
// struct CtrlBits;
// struct TaskPortMap;

namespace dsa {
namespace dfg {

struct ParseResult {
  virtual ~ParseResult() {}
};

/*! \brief Const data, including literal values and literal tuples. */
struct ConstDataEntry : ParseResult {
  ConstDataEntry(uint64_t i) : data(i) {}

  ConstDataEntry(uint64_t i1, uint64_t i2) : data((i2 << 32) | (i1 & 0xFFFFFFFF)) {}

  ConstDataEntry(uint64_t i1, uint64_t i2, uint64_t i3, uint64_t i4)
      : data(((i4 & 0xFFFF) << 48) | ((i3 & 0xFFFF) << 32) | ((i2 & 0xFFFF) << 16) |
             (i1 & 0xFFFF)) {}

  ConstDataEntry(double d) : d(d) {}

  ConstDataEntry(float f0, float f1) : f0(f0), f1(f1) {}

  union {
    struct {
      float f0, f1;
    };
    double d;
    uint64_t data;
  };
};

/*!
 * \brief A value of computation or input vector with slices
 * Example 1:
 * A,B = F(C,D)
 * A and B are values.
 *
 * Example 2:
 * A = C:0:63
 * Here `C:0:63' is a sliced value
 */
struct ValueEntry : ParseResult {
  ValueEntry(int nid_, int vid_, int l_ = 0, int r_ = 63)
      : nid(nid_), vid(vid_), l(l_), r(r_) {}
  int nid, vid;
  int l, r;
};

/*!
 * \brief A node (?vertex) of the DFG.
 *
 * Example 1:
 * A = F(B, C)
 * F(B, C) the whole is the node entry.
 *
 * Example 2:
 * A, B = F<Add:1, Sub:1>(C, D)
 * F<Add:1, Sub:1>(C, D) the whole is the node entry
 */
struct NodeEntry : ParseResult {
  int nid;
  NodeEntry(int nid_) : nid(nid_) {}
};

/*!
 * \brief Value concatnation
 * A = B:0:7 C:0:7
 * Here `B:0:7' and `C:0:7' are concatenated and bind to A
 */
struct ConvergeEntry : ParseResult {
  std::vector<ValueEntry*> entries;
};

/*!
 * \brief The converted predication
 *  A = F(B, C, ctrl=D{0:b0, 1:b1})
 *  Here `ctrl=xx` will be parsed as control entry
 */
struct ControlEntry : ParseResult {
  ControlEntry(const std::string& s, ParseResult* controller_);

  ControlEntry(const std::string& s, std::map<int, std::vector<std::string>>& bits_,
               ParseResult* controller_);

  OperandType flag;
  ParseResult* controller;
  std::map<int, std::vector<std::string>> raw;
};


/*!
 * \brief The converted task mapping
 *  TaskDep: (A:C, B:D)
 *  Here `(A:C, B:D)` will be parsed as control entry
 * @vidushi: I am not sure why is this here compared to in ssdfg.h (just copies?)
 * actually this is not required...was just required for ControlEntry for some reason...
 */
struct TaskMapEntry : ParseResult {
  TaskMapEntry(ParseResult* controller_);

  TaskMapEntry(std::unordered_map<std::string, std::string>& port_map_,
             ParseResult* controller_);

  ParseResult* controller;
  std::unordered_map<std::string, std::string> port_map; // FIXME: what is this type??
};

/*!
 * \brief A = Add(Reg0, B). The local register in a PE.
 */
struct RegisterEntry : ParseResult {
  /*! \brief The data type of the register. */
  int dtype{0};
  /*! \brief The index of the register. */
  int idx{-1};

  RegisterEntry(int d, int i) : dtype(d), idx(i) {}
};

/*! \brief The symbol table of the parsed DFG. */
class SymbolTable {
 public:
  void Set(const std::string& s, ParseResult* pr) {
    CHECK(!Has(s)) << "Duplicated Symbol: " << s;
    table_[s] = pr;
  }

  bool Has(const std::string& s) { return table_.count(s); }

  /*!
   * \brief Check if this symbol is a local register.
   *        If so, return the index of the register.
   *        If not, return -1.
   */
  static RegisterEntry isLocalRegister(const std::string &s);

  ParseResult* Get(const std::string& s) {
    auto re = isLocalRegister(s);
    if (re.idx != -1) {
      if (!Has(s)) {
        Set(s, new RegisterEntry(re));
      }
    }
    CHECK(Has(s)) << "Symbol " << s << " not found";
    return table_[s];
  }

 private:
  std::unordered_map<std::string, ParseResult*> table_;
};

/*!
 * \brief Update the properties of a DFG node by the arguments
 * \param node The DFG node to be updated
 * \param args The arguments of the instruction or operation
 */
void UpdateNodeByArgs(Node* node, std::vector<ParseResult*>& args);

}  // namespace dfg
}  // namespace dsa
