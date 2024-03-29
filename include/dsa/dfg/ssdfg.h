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
#include "dsa/dfg/array.h"
#include "dsa/dfg/symbols.h"
#include "dsa/simulation/data.h"
#include "dsa-ext/spec.h"



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

typedef std::pair<std::vector<std::string>, std::vector<std::string>> string_pair;
typedef std::vector<string_pair> task_def_t;
typedef std::vector<std::string> string_vec_t;

// post-parsing control signal definitions (mapping of string of flag to it's value?)
typedef std::map<int, string_vec_t> ctrl_def_t;

namespace dsa {

/*!
 * \brief The metadata of a sub-DFG.
 */
struct MetaDfg {
  /*!
   * \brief If this sub DFG is temporal.
   */
  bool is_temporal{false};
  /*!
   * \brief The relative execution frequency.
   */
  int64_t frequency{1};
  /*!
   * \brief The unrolling degree of this block
   */
  int64_t unroll{1};

  MetaDfg(bool temporal = false, int64_t freq = 1, int64_t u = 1) :
    is_temporal(temporal), frequency(freq), unroll(u) {}
};

}

/*! \brief The data structure for the dataflow graph. */
class SSDfg {
 public:
  /*! \brief The file name this DFG is loaded. */
  const std::string filename;

  /*! \brief The default consturctor. */
  SSDfg();

  ~SSDfg();

  /*! \brief Copy the given DFG*/
  SSDfg(const SSDfg&);

  SSDfg &operator=(const SSDfg &);

  /*! \brief Parse a DFG from the given file. */
  SSDfg(std::string filename);

  /*! \brief The entrance for the visitor pattern. */
  void Apply(dsa::dfg::Visitor*);

  /*! \brief Set a new dependence among dfg groups. */
  void create_new_task_type(int id);
  // void create_new_task_type(std::string id);
  void create_new_task_dependence_map(int s, int d);
  void add_new_task_property(std::string property, std::string value);
  void add_new_task_dependence_characteristic(std::string s, std::string d);
  void add_new_task_dependence_map(std::vector<std::string> producer, std::vector<std::string> consumer);
  task_def_t producer_consumer_map(int src_group, int dst_group);
  std::vector<std::pair<std::string, std::string>> coalescer_input_output_map(int src_group, int dst_group);
  std::pair<std::string, std::string> streaming_input_output_map(int src_group, int dst_group);
  std::unordered_map<std::string, std::string> task_type_characteristics(int task_type);

  std::unordered_map<std::string, std::string> coalescer_dependence_characteristics(int src_group, int dst_group) {
    return _coalescer_dependence_characteristics[src_group][dst_group];
  }
  std::unordered_map<std::string, std::string> argument_dependence_characteristics(int src_group, int dst_group) {
    return _dependence_characteristics[src_group][dst_group];
  }
  std::unordered_map<std::string, std::string> streaming_dependence_characteristics(int src_group, int dst_group) {
    return _streaming_dependence_characteristics[src_group][dst_group];
  }

  void set_pragma(const std::string& c, const std::string& s);

  template <typename T, typename... Args>
  inline T& emplace_back(Args&&... args);

  template <typename T>
  inline std::vector<T>& type_filter();

  int forward(bool asap);

  // ---------------------------------------------------------------------------

  void check_for_errors();

  void print_graphviz(std::string output_filename);  

  void inc_total_dyn_insts(bool is_temporal) { dyn_issued[is_temporal]++; }

  int total_dyn_insts(bool is_temporal) { return dyn_issued[is_temporal]; }

  void clear_issued() { memset(dyn_issued, 0, sizeof dyn_issued); }

  uint64_t cur_cycle() { return _cur_cycle; }

  std::string get_task_dep_charac(int i) {
    return _default_task_dep_characs[i].first;
  }

  std::string get_task_type_charac(int i) {
    return _default_task_type_characs[i].first;
  }

  void add_total_task_types() {
    ++_total_task_types;
  }

  int get_total_task_types() {
    return _total_task_types;
  }

  int _total_task_types=0;

  /*! \brief The instances of the instructions. */
  std::vector<dsa::dfg::Instruction> instructions;
  /*! \brief The instances of the FU-occupy instructions. */
  std::vector<dsa::dfg::Operation> operations;
  /*! \brief The instances of the vector inputs. */
  std::vector<dsa::dfg::InputPort> vins;
  /*! \brief The instances of the vector outputs. */
  std::vector<dsa::dfg::OutputPort> vouts;
  /*! \brief The instances of the DMAs. */
  std::vector<dsa::dfg::DMA> dmas;
  /*! \brief The instances of the Scratchpads. */
  std::vector<dsa::dfg::Scratchpad> spms;
  /*! \brief The instances of the Recturrance. */
  std::vector<dsa::dfg::Recurrance> recs;
  /*! \brief The instances of the Generates. */
  std::vector<dsa::dfg::Generate> gens;
  /*! \brief The instances of the Registers. */
  std::vector<dsa::dfg::Register> regs;
  /*! \brief The summary vector of all the nodes above. */
  std::vector<dsa::dfg::Node*> nodes;
  /*! \brief The instances of all the edges. */
  std::vector<dsa::dfg::Edge> edges;
  /*! \brief The property information of each sub DFG. */
  std::vector<dsa::MetaDfg> meta;
  /* \brief Dynamic instructions executed. */
  int dyn_issued[2] = {0, 0};
  /*! \brief Mapping informtion for dependencies among taskflow. */
  task_def_t _dependence_maps[NUM_GROUPS][NUM_GROUPS];
  /*! \brief Mapping informtion via coalescing buffer. */
  std::vector<std::pair<std::string, std::string>> _coalescer_dependence_maps[NUM_GROUPS][NUM_GROUPS];
  /*! \brief Mapping informtion for recurrence stream. */
  std::pair<std::string, std::string> _streaming_dependence_maps[NUM_GROUPS][NUM_GROUPS]; // = {{std::make_pair("-1","-1")}};
  /*! \brief Mapping characteristics for dependencies among taskflow. */
  std::unordered_map<std::string, std::string> _task_type_characteristics[NUM_GROUPS];
  std::unordered_map<std::string, std::string> _dependence_characteristics[NUM_GROUPS][NUM_GROUPS];
  std::unordered_map<std::string, std::string> _coalescer_dependence_characteristics[NUM_GROUPS][NUM_GROUPS];
  std::unordered_map<std::string, std::string> _streaming_dependence_characteristics[NUM_GROUPS][NUM_GROUPS];
  /*! \brief Default mapping characteristics for dependencies among taskflow. */
  // std::pair<std::string, std::string> _default_task_characs[NUM_TASK_CHARAC] = {{"aaa", "argument"}, {"id","-1"}, {"bytes","-1"},{"init_order","-1"},{"index", "-1"},{"ack","-1"}, {"gran","1"}};
  std::pair<std::string, std::string> _default_task_type_characs[NUM_TASK_TYPE_CHARAC] = {{"coreMask", "1000"}, {"remote", "-1"}, {"prio", "fifo"}, {"gran","1"}};
  std::pair<std::string, std::string> _default_task_dep_characs[NUM_TASK_DEP_CHARAC] = {{"aaa", "argument"}, {"id","-1"}, {"bytes","-1"},{"init_order","-1"},{"index", "-1"},{"ack","-1"}};

 private:
  // @{
  // TODO(@were): These are for simulation. Move them out later!
  uint64_t _cur_cycle = 0;
  // @}

  void normalize();

  /*! \brief The property information of each sub DFG. */
  // std::unordered_map<std::string, SSDfgVec*> _map_name_port;
  int _current_task_type=-1;
  int _current_src_grp=-1;
  int _current_dst_grp=-1;
  std::string _current_dependence_type="unknown";
};

template <>
inline std::vector<dsa::dfg::Node>& SSDfg::type_filter() {
  DSA_CHECK(false) << "Should not be called!";
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
template <>
inline std::vector<dsa::dfg::DMA>& SSDfg::type_filter() {
  return dmas;
}
template <>
inline std::vector<dsa::dfg::Scratchpad>& SSDfg::type_filter() {
  return spms;
}
template <>
inline std::vector<dsa::dfg::Recurrance>& SSDfg::type_filter() {
  return recs;
}
template <>
inline std::vector<dsa::dfg::Generate>& SSDfg::type_filter() {
  return gens;
}
template <>
inline std::vector<dsa::dfg::Register>& SSDfg::type_filter() {
  return regs;
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
  return vec.back();
}
