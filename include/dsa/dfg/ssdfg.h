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
#include "dsa/dfg/metadata.h"
#include "dsa/dfg/symbols.h"

using dsa::SpatialFabric;

// Feature Possibilities
// 1. Commutative Instruction Groups.  Be able to mark a group of instructions as commutative.  
//    All of their inputs may be processed in any order.  Scheduler is free to re-order arbitrarily.
//    This will prevent the compiler from having to reason about whether chains of instructions or 
//    trees are better for a given architecture/topology.
// 2. Allow temporal reasoning within dataflow graphs.
// a)  Instance offset dependences.  Ie. enable operands to cross computation instance boundaries.
//     One Possible syntax:
//     A = op(B<2>,C<1>)
//     This would enable specification of systolic arrays
//     Scheduler needs to be updated to enforce constraint that delayed inputs arrive later than expected,
//     according to the II of the region.
// b)  Cyclic dependences within the dataflow graph.  (optionally define initial constant value, or
//     one-time constant?)

class Schedule;
class SSDfgNode;
class SSDfg;
class SSDfgEdge;
class SSDfgVecInput;
class SSDfgVec;

namespace dsa {
namespace dfg {

struct CompileMeta : MetaPort {
  SSDfgVec *parent, *destination;
  CompileMeta(const MetaPort&, SSDfgVec*);
  CompileMeta(){};
};

struct Visitor;

}  // namespace ssdfg
}

namespace simulation {

struct Data {
  uint64_t available_at;
  uint64_t value;
  bool valid;
  Data(uint64_t aa, uint64_t value, bool valid)
      : available_at(aa), value(value), valid(valid) {}
};

struct ComputeStatus {
  int temporal{0};
  int dedicated{0};
  int total() { return temporal + dedicated; }
};

}  // namespace simulation

// Datastructure describing a value created by an input vector or an instruction
class SSDfgValue {
 public:
  friend SSDfgNode;
  friend SSDfgVecInput;
  friend SSDfg;

  SSDfgValue() {}  // for serialization
  SSDfgValue(SSDfgNode* node, int index, int bitwidth) : _node(node), _index(index), _bitwidth(bitwidth) {}

  SSDfgNode* node() const { return _node; }

  int index() const { return _index; }
  const std::vector<SSDfgEdge*> edges() const { return _uses; }

  int bitwidth() const { return _bitwidth; }

  void addOutEdge(SSDfgEdge* edge);
  SSDfg* ssdfg();
  void slice(SSDfgEdge* e1, int bitwidth);
  void slice_overlapping_edge(SSDfgEdge* e1, SSDfgEdge* e2);

  // This is a performance-non-critical function, so do it in a stupid slow
  // way that is gauranteed to work
  void slice_overlapping_edges();

  // For simulation
  std::queue<simulation::Data> fifo;
  void push(uint64_t value, bool valid, int delay);
  bool forward(bool attempt);

  std::string name();

 private:
  SSDfgNode* _node = nullptr;  // This is the node that it belongs to
  int _index = 0;              // within the node, what index is it in its value list
  int _bitwidth = 64;
  std::vector<SSDfgEdge*> _uses;  // storage for all outgoing edges
};

using EdgeType = dsa::dfg::OperandType;

class SSDfgEdge {
 public:
  SSDfgEdge() {}

  SSDfgEdge(SSDfgValue* def, SSDfgNode* use, SSDfg* ssdfg, int l = 0, int r = 63);

  /// Source and destination
  SSDfgNode* def() const;
  SSDfgValue* val() const;
  SSDfgNode* use() const;
  SSDfgNode* get(int) const;

  int id();
  int buffer_size() { return buf_len; }
  std::string name();

  void set_delay(int d) { _delay = d; }

  int delay() { return _delay; }

  // Return true if this edges is a subset of the other edge at the given position
  bool is_subset_at_pos(SSDfgEdge* alt_edge, int pos);

  void set_r(int r) { _r = r; }

  virtual int bitwidth();
  int l();
  int r();

  void set_operand_slot(int s) { _op_slot = s; }
  int operand_slot() { return _op_slot; }

  uint64_t get_value();

 private:
  int _ID = -1;
  SSDfg* _ssdfg = 0;
  SSDfgValue* _value = 0;  // value which produces the node
  SSDfgNode* _use = 0;     // operand which consumes the edge
  int _l = -1, _r = -1;
  int _op_slot = -1;

  // using 2 since 1st entry is used for bp
  unsigned int buf_len = 9;

  int _delay = 0;
};

class SSDfgInst;

// Datastructure describing the operand
struct SSDfgOperand {
  SSDfgOperand() {}

  SSDfgOperand(SSDfgEdge* e);

  SSDfgOperand(std::vector<SSDfgEdge*> es);

  SSDfgOperand(uint64_t);

  // Helper functions
  SSDfgEdge* get_first_edge() const;

  dsa::dfg::OperandType type;

  bool is_imm();

  // An Operand is valid as long as ALL of its edges are valid
  bool valid();

  // Edges concatenated in bit order from least to most significant
  std::vector<SSDfgEdge*> edges;
  uint64_t imm = 0;
  // Simulation stuff.
  std::vector<std::queue<simulation::Data>> fifos;

  bool ready();

  uint64_t poll();

  bool predicate() {
    CHECK(ready());
    for (int i = fifos.size() - 1; i >= 0; --i) {
      if (!fifos[i].front().valid) {
        return false;
      }
    }
    return true;
  }

  void pop();
};

// DFG Node -- abstract base class
// DFG Nodes are intended to be the scheduling unit
class SSDfgNode {
 public:
  friend SSDfgOperand;
  friend SSDfgValue;

  virtual ~SSDfgNode() {}

  virtual void Accept(dsa::dfg::Visitor *);

  SSDfgNode() {}

  enum V_TYPE { V_INVALID, V_INPUT, V_OUTPUT, V_INST, V_NUM_TYPES };

  virtual void printGraphviz(std::ostream& os, Schedule* sched = nullptr);

  virtual std::vector<std::pair<int, dsa::ssnode*>> candidates(Schedule*,
                                                               dsa::SSModel*,
                                                               int n) = 0;

  // Get the slot corresponding to this edge
  virtual int slot_for_use(SSDfgEdge* edge, int node_slot) {
    int slot = node_slot + edge->l() / 8;
    assert(slot < 8);
    return slot;
  }
  virtual int slot_for_op(SSDfgEdge* edge, int node_slot) { return node_slot; }

  // This is a preprocessing function that backwards marks nodes as requiring
  // dynanmic scheduling
  virtual void backprop_ctrl_dep();

  // some issue with this function
  virtual uint64_t invalid();

  SSDfgNode(SSDfg* ssdfg, V_TYPE v);
  SSDfgNode(SSDfg* ssdfg, V_TYPE v, const std::string& name);

  // Add edge to operand in least to most significant bit order
  void addOperand(unsigned pos, SSDfgEdge* e, int pos_within_op);
  void addOutEdge(SSDfgEdge* edge);

  virtual int maxThroughput();

  virtual int lat_of_inst() { return 0; }

  virtual std::string name() = 0;  // pure func

  SSDfgEdge* getLinkTowards(SSDfgNode* to);

  size_t num_out() const { return _uses.size(); }

  bool has_name() { return !_name.empty(); }

  void set_name(std::string name) { _name = name; }

  std::vector<SSDfgOperand>& ops() { return _ops; }

  const std::vector<SSDfgEdge*>& in_edges() { return _inc_edge_list; }

  std::vector<SSDfgValue*>& values() { return _values; }

  const std::vector<SSDfgEdge*>& uses() { return _uses; }

  int id() { return _ID; }

  virtual void forward(Schedule*) = 0;

  //--------------------------------------------

  int min_lat() { return _min_lat; }

  void set_min_lat(int i) { _min_lat = i; }

  int sched_lat() { return _sched_lat; }

  void set_sched_lat(int i) { _sched_lat = i; }

  bool is_temporal();

  virtual int bitwidth() = 0;
  //---------------------------------------------------------------------------

  V_TYPE type() {
    assert(_vtype != V_INVALID);
    return _vtype;
  }

  int group_id() { return _group_id; }

  void set_group_id(int id) { _group_id = id; }

  int num_inc_edges() { return _inc_edge_list.size(); }

  SSDfg* ssdfg() { return _ssdfg; }

  bool needs_ctrl_dep() { return _needs_ctrl_dep; }

  int candidates_cnt() { return candidates_cnt_; }

 protected:
  SSDfg* _ssdfg = 0;  // sometimes this is just nice to have : )

  // Dynamic stuff
  bool _invalid = false;
  std::vector<bool> _back_array;  // in edges

  // Static Stuff
  int _ID;
  std::string _name;
  std::vector<SSDfgOperand> _ops;          // in edges
  std::vector<SSDfgEdge*> _inc_edge_list;  // in edges in flat form

  std::vector<SSDfgValue*> _values;  // out edges by index
  std::vector<SSDfgEdge*> _uses;     // out edges in flat form

  bool _needs_ctrl_dep = false;

  int _min_lat = 0;
  int _sched_lat = 0;
  int _max_thr = 0;
  int _group_id = 0;  // which group do I belong to

  V_TYPE _vtype;

  int candidates_cnt_{-1};
};

typedef std::vector<std::string> string_vec_t;

// post-parsing control signal definitions (mapping of string of flag to it's value?)
typedef std::map<int, string_vec_t> ctrl_def_t;

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

// Instruction
class SSDfgInst : public SSDfgNode {
 public:
  static const int KindValue = V_INST;

  SSDfgInst() {}

  std::vector<std::pair<int, dsa::ssnode*>> candidates(Schedule*,
                                                       dsa::SSModel*,
                                                       int n) override;

  void Accept(dsa::dfg::Visitor *) final;

  SSDfgInst(SSDfg* ssdfg, dsa::OpCode inst) : SSDfgNode(ssdfg, V_INST), _reg(8, 0), opcode(inst) {
    // DFG Node makes a value by default, so start at 1
    for (int i = _values.size(); i < dsa::num_values(opcode); ++i) {
      _values.push_back(new SSDfgValue(this, i, bitwidth()));
    }
  }

  SSDfgInst(SSDfg* ssdfg) : SSDfgNode(ssdfg, V_INST), _reg(8, 0) {}

  int last_execution{-1};

  void forward(Schedule*) override;

  virtual int lat_of_inst() override { return inst_lat(inst()); }

  dsa::OpCode inst() { return opcode; }

  virtual int maxThroughput() override;

  virtual std::string name() override;

  uint64_t do_compute(bool& discard);

  void print_output(std::ostream& os);

  virtual uint64_t invalid() override { return _invalid; }

  /// Control signal, either controlled by an dependent inst or itself
  /// {
  void set_ctrl_bits(const CtrlBits& c) {
    _ctrl_bits = c;
    if (_ctrl_bits.needs_ctrl_dep()) backprop_ctrl_dep();
  }
  uint64_t ctrl_bits() { return _ctrl_bits.bits(); }
  CtrlBits ctrlBits() { return _ctrl_bits; }

  void set_self_ctrl(const CtrlBits& c) {
    _self_bits = c;
    if (_self_bits.needs_ctrl_dep()) backprop_ctrl_dep();
  }
  CtrlBits selfBits() { return _self_bits; }
  uint64_t self_bits() { return _self_bits.bits(); }
  /// }

  virtual int bitwidth() override { return dsa::bitwidth[opcode]; }

  uint64_t getout(int i) { return _output_vals[i]; }

 private:

  void resize_vals(int n);

  std::vector<uint64_t> _input_vals;
  std::vector<uint64_t> _output_vals;

  CtrlBits _ctrl_bits;
  CtrlBits _self_bits;

  std::vector<uint64_t> _reg;

  dsa::OpCode opcode;
};

// vector class
class SSDfgVec : public SSDfgNode {
 public:
  friend class SSDfg;

  SSDfgVec() {}

  SSDfgVec(V_TYPE v, int len, int bitwidth, const std::string& name, int id, SSDfg* ssdfg,
           const dsa::dfg::MetaPort& meta);

  virtual void Accept(dsa::dfg::Visitor *);

  void set_port_width(int n) { _port_width = n; }

  int get_port_width() { return _port_width; }

  void set_vp_len(int n) { _vp_len = n; }

  int get_vp_len() { return _vp_len; }

  int logical_len() { return _vp_len; }

  int length() { return _ops.size(); }

  virtual std::string name() override { return _name; }

  virtual int bitwidth() override { return _bitwidth; }

  int phys_bitwidth() { return is_temporal() ? 64 : (_values.size() * bitwidth()); }

 protected:
  int _bitwidth;  // element bitwidth
  int _port_width;
  int _vp_len;
  dsa::dfg::CompileMeta meta;
};

class SSDfgVecInput : public SSDfgVec {
 public:
  static std::string Suffix() { return ""; }
  static bool IsInput() { return true; }

  static const int KindValue = V_INPUT;

  void Accept(dsa::dfg::Visitor *) final;

  SSDfgVecInput() {}

  SSDfgVecInput(int len, int width, const std::string& name, int id, SSDfg* ssdfg,
                const dsa::dfg::MetaPort& meta)
      : SSDfgVec(V_INPUT, len, width, name, id, ssdfg, meta) {
    for (int i = 0; i < std::max(1, len / (64 / width)); ++i) {
      _values.push_back(new SSDfgValue(this, i, width));
    }
  }

  std::vector<std::pair<int, dsa::ssnode*>> candidates(Schedule*, dsa::SSModel*, int n) override;

  int current_{0};
  void forward(Schedule*) override;
  bool can_push();
};

class SSDfgVecOutput : public SSDfgVec {
 public:
  static std::string Suffix() { return "_out"; }
  static bool IsInput() { return false; }

  static const int KindValue = V_OUTPUT;

  SSDfgVecOutput() {}

  SSDfgVecOutput(int len, int width, const std::string& name, int id, SSDfg* ssdfg,
                 const dsa::dfg::MetaPort& meta)
      : SSDfgVec(V_OUTPUT, len, width, name, id, ssdfg, meta) {}

  void Accept(dsa::dfg::Visitor *) override;

  std::vector<std::pair<int, dsa::ssnode*>> candidates(Schedule*, dsa::SSModel*, int n) override;

  // SSDfgNode *at(int i) {
  //  return _ops[i]->node();
  //}

  virtual int slot_for_op(SSDfgEdge* edge, int node_slot) override {
    // need to figure out which operand, then count bits within that operand
    for (SSDfgOperand& op : _ops) {
      int slot = 0;
      for (SSDfgEdge* cur_edge : op.edges) {
        if (cur_edge == edge) return slot;
        slot += cur_edge->bitwidth() / 8;
      }
    }
    assert(0 && "edge not present in any operands");
    return -1;
  }

  void forward(Schedule*) override {}
  bool can_pop();
  void pop(std::vector<uint64_t>& data, std::vector<bool>& data_valid);
};

struct GroupProp {
  bool is_temporal{false};
  int64_t frequency{-1};
  int64_t unroll{1};
};

class SSDfg {
 public:
  const std::string filename;

  SSDfg();

  SSDfg(std::string filename);

  ~SSDfg() {}

  void Apply(dsa::dfg::Visitor *);

  void printGraphviz(std::ostream& os, Schedule* sched = nullptr);

  void printGraphviz(const char* fname, Schedule* sched = nullptr);

  void start_new_dfg_group();

  void set_pragma(const std::string& c, const std::string& s);


  template <typename T>
  inline void add(T*);

  SSDfgEdge* connect(SSDfgValue* orig, SSDfgNode* dest, int slot,
                     EdgeType etype, int l = 0, int r = 63,
                     int operand_pos = -1);

  template <typename T>
  inline std::vector<T>& nodes();

  const std::vector<SSDfgInst*>& inst_vec() { return _insts; }

  GroupProp& group_prop(int i) { return _groupProps[i]; }

  int num_groups() { return _groupProps.size(); }

  int maxGroupThroughput(int group);

  int cycle(bool print, bool verif);

  int forward(bool asap, Schedule*);

  double estimated_performance(Schedule*, bool);

  // ---------------------------------------------------------------------------

  void preprocess_graph();

  void calc_minLats();

  void check_for_errors();

  void inc_total_dyn_insts(bool is_temporal) { dyn_issued[is_temporal]++; }

  int total_dyn_insts(bool is_temporal) { return dyn_issued[is_temporal]; }

  void clear_issued() { memset(dyn_issued, 0, sizeof dyn_issued); }

  int get_max_lat() { return MAX_LAT; }

  int num_node_ids() { return _num_node_ids; }

  std::vector<SSDfgEdge*>& edges() { return _edges; }

  int num_edge_ids() { return _num_edge_ids; }

  int inc_node_id() { return _num_node_ids++; }

  int inc_edge_id() { return _num_edge_ids++; }

  uint64_t cur_cycle() { return _cur_cycle; }

 private:
  static constexpr int MAX_LAT = 1000;
  // to keep track of number of cycles---------------------
  int cur_node_ptr = 0;
  int cur_buf_ptr = 0;
  uint64_t _cur_cycle = 0;

  std::unordered_map<int, uint64_t> _complex_fu_free_cycle;

  // stats
  int dyn_issued[2] = {0, 0};

  std::vector<SSDfgNode*> _nodes;

  // redundant storage:
  std::vector<SSDfgInst*> _insts;
  std::vector<SSDfgVecInput*> _vecInputs;
  std::vector<SSDfgVecOutput*> _vecOutputs;

  std::vector<SSDfgEdge*> _edges;

  std::vector<GroupProp> _groupProps;

  int _num_node_ids = 0;
  int _num_edge_ids = 0;
};

template <> inline std::vector<SSDfgNode*>& SSDfg::nodes() { return _nodes; }
template <> inline std::vector<SSDfgInst*>& SSDfg::nodes() { return _insts; }
template <> inline std::vector<SSDfgVecInput*>& SSDfg::nodes() { return _vecInputs; }
template <> inline std::vector<SSDfgVecOutput*>& SSDfg::nodes() { return _vecOutputs; }

// TODO(@were): add enable_if!
template <typename T>
inline void SSDfg::add(T* node) {
  nodes<T*>().push_back(node);
  _nodes.push_back(node);
}