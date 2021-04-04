#pragma once

#include <assert.h>
#include <math.h>

#include <algorithm>
#include <bitset>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dsa/arch/model.h"
#include "dsa/arch/ssinst.h"
#include "dsa/dfg/instruction.h"
#include "dsa/dfg/metadata.h"
#include "dsa/dfg/node.h"
#include "dsa/dfg/port.h"
#include "dsa/dfg/symbols.h"
#include "dsa/simulation/data.h"

#define NUM_GROUPS 6

using dsa::SpatialFabric;

// Feature Possibilities
// 1. Commutative Instruction Groups.  Be able to mark a group of instructions as
// commutative.
//    All of their inputs may be processed in any order.  Scheduler is free to re-order
//    arbitrarily. This will prevent the compiler from having to reason about whether
//    chains of instructions or trees are better for a given architecture/topology.
// 2. Allow temporal reasoning within dataflow graphs.
// a)  Instance offset dependences.  Ie. enable operands to cross computation instance
// boundaries.
//     One Possible syntax:
//     A = op(B<2>,C<1>)
//     This would enable specification of systolic arrays
//     Scheduler needs to be updated to enforce constraint that delayed inputs arrive
//     later than expected, according to the II of the region.
// b)  Cyclic dependences within the dataflow graph.  (optionally define initial constant
// value, or
//     one-time constant?)

class SSDfg;

namespace dsa {
namespace dfg {

struct Visitor;

}  // namespace dfg
}  // namespace dsa

// post-parsing task dependence signal definitions (mapping of string of flag to it's value?)
// typedef std::unordered_map<std::string, std::string> task_def_t;
typedef std::pair<std::vector<std::string>, std::vector<std::string>> string_pair;
typedef std::vector<string_pair> task_def_t;
typedef std::vector<std::pair<std::vector<int>, std::vector<int>>> port_map_def_t;
// associated with each dfg-group that can be created dynamically..
// FIXME: is it used anywhere??
/*struct TaskPortMap {

  // return the port number from this string...
  TaskPortMap(const std::map<int, std::vector<std::string>>& raw);
  TaskPortMap(task_def_t port_map_) : _mapping(port_map_) {}

  // set another mapping of bits (Concatenate into the port)
  void set(std::string prod_port, std::string cons_port) {
    _mapping.insert(make_pair(prod_port, cons_port));
  }
  string getMappedPort(string prod_port) {
    auto it = _mapping.find(prod_port);
    assert(it->begin()!-it->end());
    return it->second;
  }
  int getNumMappedPorts() {
    return _mapping.size();
  }
  task_def_t mapping() { return _mapping; }

 private:
  task_def_t _mapping;
};*/

typedef std::vector<std::string> string_vec_t;

// post-parsing control signal definitions (mapping of string of flag to it's value?)
typedef std::map<int, string_vec_t> ctrl_def_t;

<<<<<<< HEAD
=======
struct CtrlBits {
  enum Control { B1, B2, Discard, Reset, Abstain, Total };

  CtrlBits(const std::map<int, std::vector<std::string>>& raw);
  CtrlBits(uint64_t mask_) : mask(mask_) {}
  CtrlBits() : mask(0) {}

  void set(uint64_t val, Control b);
  bool test(uint64_t val, Control b);
  void test(uint64_t val, std::vector<bool>& back_array, bool& discard, bool& predicate,
            bool& reset);
  CtrlBits &operator=(const CtrlBits &b) {
    mask = b.mask;
    const_cast<bool&>(is_dynamic) = b.is_dynamic;
    return *this;
  }

  uint64_t bits() { return mask; }

  bool needs_ctrl_dep() { return is_dynamic; }

  const bool is_dynamic{false};

 private:
  uint64_t mask{0};

  static Control str_to_enum(const std::string& s) {
    if (s == "b1") return B1;
    if (s == "b2") return B2;
    if (s == "d") return Discard;
    if (s == "r") return Reset;
    if (s == "a") return Abstain;
    assert(false && "Not a valid command");
  }
};

/*! \brief IR node for the instructions in the DFG. */
class SSDfgInst : public SSDfgNode {
 public:
  static const int KindValue = V_INST;

  /*! \brief The entrance function for the visitor pattern. */
  void Accept(dsa::dfg::Visitor *) final;

  /*! \brief The default constructor. */
  SSDfgInst() {}

  /*!
   * \brief The constructor with instruction opcode.
   * \param ssdfg The DFG this instruciton belongs to.
   * \param inst The instruction opcode.
   */
  SSDfgInst(SSDfg* ssdfg, dsa::OpCode inst = dsa::SS_NONE) :
    SSDfgNode(ssdfg, V_INST), _reg(8, 0), opcode(inst) {
    CHECK(values.empty());
    int n = dsa::num_values(opcode);
    for (int i = 0; i < n; ++i) {
      values.emplace_back(ssdfg, id(), i);
    }
  }

  /*!
   * \brief The latency of the instruction execution.
   * \return The latency of the instruction execution.
   */
  int lat_of_inst() override { return inst_lat(inst()); }

  void reset_regs() {
    for(unsigned i=0; i<_reg.size(); ++i) {
      _reg[i]=0;
    }
  }

  /*!
   * \brief The instruction opcode.
   * \return The instruction opcode.
   */
  dsa::OpCode inst() { return opcode; }

  /*! \brief The name of this instruction. */
  // TODO(@were): Do we want to rename this to ToString?
  virtual std::string name() override;

  /*! \brief The predication affected by an upstream operand. */
  CtrlBits predicate;
  /*! \brief The predication affected by itself. */
  CtrlBits self_predicate;

  int bitwidth() override;

  // TODO(@were): Move these to simulation.
  // @{
  int last_execution{-1};
  void forward() override;
  uint64_t do_compute(bool& discard);
  uint64_t invalid() override { return _invalid; }
  // @}
 private:

  // TODO(@were): These are data structures for simulation. Move them out later.
  std::vector<uint64_t> _input_vals;
  std::vector<uint64_t> _output_vals;
  std::vector<uint64_t> _reg;

  dsa::OpCode opcode{dsa::OpCode::SS_NONE};
};

// vector class
class SSDfgVec : public SSDfgNode {
 public:
  friend class SSDfg;

  SSDfgVec() {}

  SSDfgVec(V_TYPE v, int len, int bitwidth, const std::string& name, SSDfg* ssdfg,
           const dsa::dfg::MetaPort& meta, bool indirect=false);

  virtual void Accept(dsa::dfg::Visitor *);

  void set_port_width(int n) { _port_width = n; }

  int get_port_width() { return _port_width; }

  void set_vp_len(int n) { _vp_len = n; }

  int get_vp_len() { return _vp_len; }

  int logical_len() { return _vp_len; }

  int length() { return _ops.size(); }

  virtual std::string name() override { return _name; }

  virtual int bitwidth() override { return _bitwidth; }

  int phys_bitwidth() { return is_temporal() ? 64 : (values.size() * bitwidth()); }

 protected:
  int _bitwidth;  // element bitwidth
  int _port_width;
  int _vp_len;
 public:
  dsa::dfg::CompileMeta meta;
};

class SSDfgVecInput : public SSDfgVec {
 public:
  static std::string Suffix() { return ""; }
  static bool IsInput() { return true; }

  static const int KindValue = V_INPUT;

  void Accept(dsa::dfg::Visitor *) final;

  SSDfgVecInput() {}

  SSDfgVecInput(int len, int width, const std::string& name, SSDfg* ssdfg,
                const dsa::dfg::MetaPort& meta, bool indirect=false);

  int current_{0};
  void forward() override;
  bool can_push();
};

class SSDfgVecOutput : public SSDfgVec {
 public:
  static std::string Suffix() { return "_out"; }
  static bool IsInput() { return false; }

  static const int KindValue = V_OUTPUT;

  SSDfgVecOutput() {}

  SSDfgVecOutput(int len, int width, const std::string& name, SSDfg* ssdfg,
                 const dsa::dfg::MetaPort& meta, bool indirect=false);

  void Accept(dsa::dfg::Visitor *) override;

  virtual int slot_for_op(dsa::dfg::Edge* edge, int node_slot) override;

  void forward() override {}
  bool can_pop();
  void pop(std::vector<uint64_t>& data, std::vector<bool>& data_valid);
};

>>>>>>> trying out to route only a single port
struct GroupProp {
  bool is_temporal{false};
  int64_t frequency{-1};
  int64_t unroll{1}; // TODO: @vidushi: should be changed with vector ID
  // int64_t id=0; // TODO: @vidushi: add an ID here..
};

/*! \brief The data structure for the dataflow graph. */
class SSDfg {
 public:
  /*! \brief The file name this DFG is loaded. */
  const std::string filename;

  /*! \brief The default consturctor. */
  SSDfg();

  /*! \brief Copy the given DFG*/
  SSDfg(const SSDfg&);

  /*! \brief Parse a DFG from the given file. */
  SSDfg(std::string filename);

  /*! \brief Reset registers in the dfg instructions. */
  void reset_dfg();

  /*! \brief The entrance for the visitor pattern. */
  void Apply(dsa::dfg::Visitor*);

  /*! \brief Start a new sub-dfg. */
  void start_new_dfg_group();

  /*! \brief Set a new dependence among dfg groups. */
  void create_new_task_dependence_map(int s, int d);
  void add_new_task_dependence_characteristic(std::string s, std::string d);
  void add_new_task_dependence_map(std::vector<std::string> producer, std::vector<std::string> consumer);
  task_def_t producer_consumer_map(int src_group, int dst_group);
  std::vector<std::pair<std::string, std::string>> coalescer_input_output_map(int src_group, int dst_group);

  std::unordered_map<std::string, std::string> dependence_characteristics(int src_group, int dst_group) {
    return _dependence_characteristics[src_group][dst_group];
  }

  void set_pragma(const std::string& c, const std::string& s);

  template <typename T, typename... Args>
  inline T& emplace_back(Args&&... args);

  template <typename T>
  inline std::vector<T>& type_filter();

  GroupProp& group_prop(int i) { return _groupProps[i]; }

  int num_groups() { return _groupProps.size(); }

  int forward(bool asap);

  // ---------------------------------------------------------------------------

  void check_for_errors();

  void inc_total_dyn_insts(bool is_temporal) { dyn_issued[is_temporal]++; }

  int total_dyn_insts(bool is_temporal) { return dyn_issued[is_temporal]; }

  void clear_issued() { memset(dyn_issued, 0, sizeof dyn_issued); }

  uint64_t cur_cycle() { return _cur_cycle; }

  /*void insert_port_mapping(std::string name, SSDfgVec* vector_port) {
    auto it = _map_name_port.find(name);
    assert(it==_map_name_port.end() && "same port should not come again");
    _map_name_port.insert(std::make_pair(name, vector_port));
  }

  SSDfgVec* get_port_mapping(std::string &name) {
    auto it = _map_name_port.find(name);
    assert(it!=_map_name_port.end());
    return it->second;
  }*/

  std::string get_task_charac(int i) {
    return _default_task_characs[i].first;
  }

  /*! \brief The instances of the instructions. */
  std::vector<dsa::dfg::Instruction> instructions;
  /*! \brief The instances of the FU-occupy instructions. */
  std::vector<dsa::dfg::Operation> operations;
  /*! \brief The instances of the vector inputs. */
  std::vector<dsa::dfg::InputPort> vins;
  /*! \brief The instances of the vector outputs. */
  std::vector<dsa::dfg::OutputPort> vouts;
  /*! \brief The summary vector of all the nodes above. */
  std::vector<dsa::dfg::Node*> nodes;
  /*! \brief The instances of all the edges. */
  std::vector<dsa::dfg::Edge> edges;
  /*! \brief Mapping informtion for dependencies among taskflow. */
  task_def_t _dependence_maps[NUM_GROUPS][NUM_GROUPS];
  /*! \brief Mapping informtion via coalescing buffer. */
  std::vector<std::pair<std::string, std::string>> _coalescer_dependence_maps[NUM_GROUPS][NUM_GROUPS];
  /*! \brief Mapping characteristics for dependencies among taskflow. */
  std::unordered_map<std::string, std::string> _dependence_characteristics[NUM_GROUPS][NUM_GROUPS];
  /*! \brief Default mapping characteristics for dependencies among taskflow. */
  std::pair<std::string, std::string> _default_task_characs[6] = {{"type", "argument"}, {"id","-1"}, {"gran","1"}, {"bytes","64"},{"init_order","-1"},{"index", "-1"}};

 private:
  // @{
  // TODO(@were): These are for simulation. Move them out later!
  uint64_t _cur_cycle = 0;
  int dyn_issued[2] = {0, 0};
  // @}

  void normalize();

  /*! \brief The property information of each sub DFG. */
  std::vector<GroupProp> _groupProps;
  // std::unordered_map<std::string, SSDfgVec*> _map_name_port;
  int _current_src_grp=-1;
  int _current_dst_grp=-1;
};

template <>
inline std::vector<dsa::dfg::Node>& SSDfg::type_filter() {
  CHECK(false) << "Should not be called!";
  throw;
}
template <>
inline std::vector<dsa::dfg::Node*>& SSDfg::type_filter() {
  return nodes;
}
template <>
inline std::vector<dsa::dfg::Instruction>& SSDfg::type_filter() {
  return instructions;
}
template <>
inline std::vector<dsa::dfg::Operation>& SSDfg::type_filter() {
  return operations;
}
template <>
inline std::vector<dsa::dfg::InputPort>& SSDfg::type_filter() {
  return vins;
}
template <>
inline std::vector<dsa::dfg::OutputPort>& SSDfg::type_filter() {
  return vouts;
}

template <typename T, typename... Args>
inline T& SSDfg::emplace_back(Args&&... args) {
  auto& vec = type_filter<T>();
  auto capacity = vec.capacity();
  vec.emplace_back(args...);
  nodes.push_back(&vec.back());
  if (vec.capacity() != capacity) {
    normalize();
  }
}
