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

  /*! \brief The entrance for the visitor pattern. */
  void Apply(dsa::dfg::Visitor*);

  /*! \brief Start a new sub-dfg. */
  void start_new_dfg_group();

  /*! \brief Set a new dependence among dfg groups. */
  void create_new_task_dependence_map(int s, int d);
  void add_new_task_dependence_map(std::vector<std::string> producer, std::vector<std::string> consumer);
  task_def_t producer_consumer_map(int src_group, int dst_group);

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

  void insert_input_mapping(std::string &name, SSDfgVecInput* input) {
    _map_name_input.insert(std::make_pair(name, input));
  }

  void insert_output_mapping(std::string &name, SSDfgVecOutput* output) {
    _map_name_output.insert(std::make_pair(name, output));
  }

  SSDfgVecInput* get_input_mapping(std::string &name) {
    auto it = _map_name_input.find(name);
    assert(it!=_map_name_input.end());
    return it->second;
  }

  SSDfgVecOutput* get_output_mapping(std::string &name) {
    auto it = _map_name_output.find(name);
    assert(it!=_map_name_output.end());
    return it->second;
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

 private:
  // @{
  // TODO(@were): These are for simulation. Move them out later!
  uint64_t _cur_cycle = 0;
  int dyn_issued[2] = {0, 0};
  // @}

  void normalize();

  /*! \brief The property information of each sub DFG. */
  std::vector<GroupProp> _groupProps;
  task_def_t _dependence_maps[5][5]; // NUM_GROUPS][NUM_GROUPS];
  std::unordered_map<std::string, SSDfgVecInput*> _map_name_input;
  std::unordered_map<std::string, SSDfgVecOutput*> _map_name_output;
  int _current_src_grp=0;
  int _current_dst_grp=0;
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
