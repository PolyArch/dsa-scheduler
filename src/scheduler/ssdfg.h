
#ifndef __SSDFG_H__
#define __SSDFG_H__

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

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include "ss-scheduler/metadata.h"
#include "ss-config/model.h"
#include "ss-config/ssinst.h"


using SS_CONFIG::SubModel;

void checked_system(const char* command);

class Schedule;
class SSDfgNode;
class SSDfg;
class SSDfgEdge;
class SSDfgVecInput;

namespace simulation {

struct Data {
  int64_t available_at;
  uint64_t value;
  bool valid;
  Data(int64_t aa, uint64_t value, bool valid) : available_at(aa), value(value), valid(valid) {}
};

}

class SSDfgVec;

namespace ssdfg {

struct CompileMeta : MetaPort {
  SSDfgVec *parent, *destination;
  CompileMeta(const MetaPort &, SSDfgVec *);
  CompileMeta() {};
};

}

// Datastructure describing a value created by an input vector or an instruction
class SSDfgValue {
 public:
  friend SSDfgNode;
  friend SSDfgVecInput;
  friend SSDfg;

  SSDfgValue() {}  // for serialization
  SSDfgValue(SSDfgNode* node, int index, int bitwidth)
      : _node(node), _index(index), _bitwidth(bitwidth) {}

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

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);

  // TODO: Deprecate these two
  // Dynamic Stuff
  void set_val(uint64_t v) { _val = v; }
  uint64_t get_val() { return _val; }

  std::queue<simulation::Data> fifo;

  void push(uint64_t value, bool valid, int delay);
  bool forward(bool attempt);

 private:
  SSDfgNode* _node = nullptr;  // This is the node that it belongs to
  int _index = 0;              // within the node, what index is it in its value list
  int _bitwidth = 64;
  std::vector<SSDfgEdge*> _uses;  // storage for all outgoing edges

  // Dynamic Stuff
  uint64_t _val = 0;  // dynamic var (setting the default value)
};

class SSDfgEdge {
 public:
  SSDfgEdge() {}

  enum EdgeType { data, ctrl, ctrl_true, ctrl_false };

  EdgeType etype() { return _etype; }

  SSDfgEdge(SSDfgValue* def, SSDfgNode* use, EdgeType etype, SSDfg* ssdfg, int l = 0,
            int r = 63);

  /// Source and destination
  SSDfgNode* def() const;
  SSDfgValue* val() const;
  SSDfgNode* use() const;
  SSDfgNode* get(int) const;

  int id();
  int buffer_size() { return buf_len; }
  std::string gamsName();
  std::string name();

  void set_delay(int d) { _delay = d; }

  int delay() { return _delay; }

  // void compute_next();
  void compute_after_push(bool print, bool verif);

  void compute_after_pop(bool print, bool verif);

  void push_in_buffer(uint64_t v, bool valid, bool print, bool verif);

  bool is_buffer_full();

  bool is_buffer_empty();

  // Calculate the value based on the origin's value, and the edge's
  // index and bitwidth fields
  uint64_t extract_value(uint64_t v_in);

  uint64_t get_buffer_val();

  bool get_buffer_valid();

  void pop_buffer_val(bool print, bool verif);

  // Return true if this edges is a subset of the other edge at the given position
  bool is_subset_at_pos(SSDfgEdge* alt_edge, int pos);

  void set_r(int r) { _r = r; }

  virtual int bitwidth();
  int l();
  int r();

  void set_operand_slot(int s) { _op_slot = s; }
  int operand_slot() { return _op_slot; }

  uint64_t get_value();

  void reset_associated_buffer();

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);

 private:
  int _ID = -1;
  SSDfg* _ssdfg = 0;
  SSDfgValue* _value = 0;  // value which produces the node
  SSDfgNode* _use = 0;     // operand which consumes the edge
  EdgeType _etype;
  int _l = -1, _r = -1;
  int _op_slot = -1;

  // Runtime Types
  std::queue<std::pair<uint64_t, bool>> _data_buffer;
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

  void clear();

  bool is_ctrl();

  bool is_imm();

  bool is_composed();

  // Functions which manipulate dynamic state
  uint64_t get_value();

  uint64_t get_buffer_val();

  uint64_t get_buffer_valid();

  uint64_t is_buffer_empty();

  void pop_buffer_val(bool print, bool verif);

  // An Operand is valid as long as ALL of its edges are valid
  bool valid();

  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);

  // Edges concatenated in bit order from least to most significant
  std::vector<SSDfgEdge*> edges;
  uint64_t imm = 0;
  // Simulation stuff.
  std::vector<std::queue<simulation::Data>> fifos;

  bool ready();

  uint64_t poll();

  bool predicate() {
    assert(ready());
    bool res = true;
    for (int i = fifos.size() - 1; i >= 0; --i) {
      res = res && fifos[i].front().valid;
    }
    return res;
  }

  void pop() {
    assert(ready());
    for (auto &elem : fifos) {
      elem.pop();
    }
  }

};

// DFG Node -- abstract base class
// DFG Nodes are intended to be the scheduling unit
class SSDfgNode {
 public:
  friend SSDfgOperand;
  friend SSDfgValue;

  virtual ~SSDfgNode() {}

  SSDfgNode() {
    //_values.resize(1,new SSDfgValue(this,0,bitwidth()));
  }

  enum V_TYPE { V_INVALID, V_INPUT, V_OUTPUT, V_INST, V_NUM_TYPES };

  virtual void printGraphviz(std::ostream& os, Schedule* sched = nullptr);

  virtual std::vector<std::pair<int, SS_CONFIG::ssnode*>> candidates(Schedule*,
                                                                     SS_CONFIG::SSModel*,
                                                                     int n) = 0;

  // Get the slot corresponding to this edge
  virtual int slot_for_use(SSDfgEdge* edge, int node_slot) {
    int slot = node_slot + edge->l() / 8;
    assert(slot < 8);
    return slot;
  }
  virtual int slot_for_op(SSDfgEdge* edge, int node_slot) { return node_slot; }

  // some issue with this function
  virtual uint64_t invalid();

  SSDfgNode(SSDfg* ssdfg, V_TYPE v);
  SSDfgNode(SSDfg* ssdfg, V_TYPE v, const std::string& name);

  typedef std::vector<SSDfgEdge*>::const_iterator const_edge_iterator;
  typedef std::vector<SSDfgOperand>::const_iterator const_op_iterator;

  // Add edge to operand in least to most significant bit order
  void addOperand(unsigned pos, SSDfgEdge* e, int pos_within_op);
  void addOutEdge(SSDfgEdge* edge);
  void remove_edge(SSDfgNode* to_erase, int idx);
  void removeIncEdge(SSDfgNode* orig);
  void removeOutEdge(SSDfgNode* dest);
  void reset_node();

  void validate();

  virtual int compute(bool print, bool verif) { return 0; }

  //-----------------------------------------
  virtual int update_next_nodes(bool print, bool verif) { return 0; }
  virtual int compute_backcgra(bool print, bool verif) {
    _inputs_ready = 0;
    return 0;
  }
  //-------------------------------------------------------

  virtual int maxThroughput();

  virtual int lat_of_inst() { return 0; }

  virtual void depInsts(std::vector<SSDfgInst*>& insts);

  virtual std::string name() = 0;  // pure func

  virtual std::string gamsName() = 0;

  SSDfgEdge* getLinkTowards(SSDfgNode* to);

  int num_inputs_ready() { return _inputs_ready; }

  SSDfgEdge* first_inc_edge() { return _ops[0].edges[0]; }

  SSDfgOperand& first_operand() { return _ops[0]; }

  SSDfgNode* first_op_node() { return (_ops[0].edges[0]->def()); }

  SSDfgNode* first_use() { return (_uses[0]->use()); }

  size_t num_inc() const { return _ops.size(); }

  size_t num_out() const { return _uses.size(); }

  bool has_name() { return !_name.empty(); }

  void set_name(std::string name) { _name = name; }

  std::vector<SSDfgOperand>& ops() { return _ops; }

  const std::vector<SSDfgEdge*>& in_edges() { return _inc_edge_list; }

  std::vector<SSDfgValue*>& values() { return _values; }

  const std::vector<SSDfgEdge*>& uses() { return _uses; }

  int id() { return _ID; }

  virtual void forward() = 0;

  bool get_bp();

  // retrieve the value of the def
  uint64_t retrieve(int operand_index) {
    assert(_ops.size() > (unsigned) operand_index);
    return _ops[operand_index].get_value();
  }

  // Check's all consumers for backpressure-freedom,
  // If backpressure, delay the push to next cycle
  void set_node(SSDfgValue* dfg_val, 
      uint64_t v, bool valid, bool avail, bool print, bool verif);

  // sets value in non-backcgra
  void set_value(int value_id, uint64_t v, bool valid) {
    assert(value_id < (int)_values.size() && "Array access out of bound!");
    _values[value_id]->set_val(v);
    _invalid = !valid;
  }

  // sets value at this cycle
  void set_value(SSDfgValue* dfg_val, uint64_t v, bool valid, bool avail, int cycle, bool is_compute);

  // sets value with val index (this interface makes more sense?)
  void set_value(int i, uint64_t v, bool valid, bool avail, int cycle, bool is_compute);

  //--------------------------------------------

  int min_lat() { return _min_lat; }

  void set_min_lat(int i) { _min_lat = i; }

  int sched_lat() { return _sched_lat; }

  void set_sched_lat(int i) { _sched_lat = i; }

  int inc_inputs_ready(bool print, bool verif);

  int inc_inputs_ready_backcgra(bool print, bool verif);

  int get_inputs_ready() { return _inputs_ready; }

  void push_buf_dummy_node();

  void set_node_id(int i) { _node_id = i; }

  int node_id() { return _node_id; }

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

 private:
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);

 protected:
  SSDfg* _ssdfg = 0;  // sometimes this is just nice to have : )

  int _node_id = -1;  // hack for temporal simulator to remember _node_id

  // Dynamic stuff
  bool _invalid = false;
  int _inputs_ready = 0;          // dynamic inputs ready
  std::vector<bool> _back_array;  // in edges

  // Static Stuff
  int _ID;
  std::string _name;
  std::vector<SSDfgOperand> _ops;          // in edges
  std::vector<SSDfgEdge*> _inc_edge_list;  // in edges in flat form

  std::vector<SSDfgValue*> _values;  // out edges by index
  std::vector<SSDfgEdge*> _uses;     // out edges in flat form

  int _min_lat = 0;
  int _sched_lat = 0;
  int _max_thr = 0;
  int _group_id = 0;  // which group do I belong to

  V_TYPE _vtype;
};

typedef std::vector<std::string> string_vec_t;

// post-parsing control signal definitions (mapping of string of flag to it's value?)
typedef std::map<int, string_vec_t> ctrl_def_t;

struct ParseResult {
  virtual ~ParseResult() {}
};

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

struct ValueEntry : ParseResult {
  ValueEntry(SSDfgValue* value_, int l_ = 0, int r_ = 63) : value(value_), l(l_), r(r_) {}

  SSDfgValue* value = 0;
  int l, r;
};

struct ConvergeEntry : ParseResult {
  std::vector<ValueEntry*> entries;
};

struct CtrlBits {
  enum Control { B1, B2, Discard, Reset, Abstain, Total };

  CtrlBits(const std::map<int, std::vector<std::string>>& raw);
  CtrlBits() : mask(0) {}

  void set(uint64_t val, Control b);
  bool test(uint64_t val, Control b);
  void test(uint64_t val, std::vector<bool>& back_array, bool& discard, bool& predicate,
            bool& reset);

  uint64_t bits() { return mask; }

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned version);

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

struct ControlEntry : ParseResult {
  void set_flag(const std::string& s);

  ControlEntry(const std::string& s, ParseResult* controller_)
      : controller(controller_), bits() {
    set_flag(s);
  }

  ControlEntry(const std::string& s, std::map<int, std::vector<std::string>>& bits_,
               ParseResult* controller_)
      : controller(controller_), bits(bits_) {
    set_flag(s);
  }

  SSDfgEdge::EdgeType flag;
  ParseResult* controller;
  CtrlBits bits;
};

// Instruction
class SSDfgInst : public SSDfgNode {
 public:
  static const int KindValue = V_INST;

  SSDfgInst() {}

  std::vector<std::pair<int, SS_CONFIG::ssnode*>> candidates(Schedule*,
                                                             SS_CONFIG::SSModel*,
                                                             int n) override;

  SSDfgInst(SSDfg* ssdfg, SS_CONFIG::ss_inst_t inst, bool is_dummy = false)
      : SSDfgNode(ssdfg, V_INST),
        _predInv(false),
        _isDummy(is_dummy),
        _imm_slot(-1),
        _subFunc(0),
        _reg(8, 0),
        _ssinst(inst) {
    // DFG Node makes a value by default, so start at 1
    for (int i = _values.size(); i < SS_CONFIG::num_values(_ssinst); ++i) {
      _values.push_back(new SSDfgValue(this, i, bitwidth()));
    }
  }

  SSDfgInst(SSDfg* ssdfg)
      : SSDfgNode(ssdfg, V_INST),
        _predInv(false),
        _isDummy(false),
        _imm_slot(-1),
        _subFunc(0),
        _reg(8, 0) {}

  int last_execution{-1};

  void forward() override;

  virtual int lat_of_inst() override { return inst_lat(inst()); }

  void setImm(uint64_t val) { _imm = val; }

  uint64_t imm() { return _imm; }

  void setPredInv(bool predInv) { _predInv = predInv; }

  bool predInv() { return _predInv; }

  bool isDummy() { return _isDummy; }

  SS_CONFIG::ss_inst_t inst() { return _ssinst; }

  // Adding new function in the header file
  int update_next_nodes(bool print, bool verif) override;

  virtual int maxThroughput() override;

  virtual void depInsts(std::vector<SSDfgInst*>& insts) override;

  virtual std::string name() override;

  std::string gamsName() override;

  void setImmSlot(int i);

  int immSlot() const { return _imm_slot; }

  void setSubFunc(int i) { _subFunc = i; }

  int subFunc() const { return _subFunc; }

  uint64_t do_compute(bool& discard);

  virtual int compute(bool print, bool verif) override;

  void print_output(std::ostream& os);

  // new line added
  virtual int compute_backcgra(bool print, bool verif) override;

  void set_verif_id(std::string s) { _verif_id = s; }

  virtual uint64_t invalid() override { return _invalid; }

  /// Control signal, either controlled by an dependent inst or itself
  /// {
  void set_ctrl_bits(const CtrlBits& c) { _ctrl_bits = c; }
  uint64_t ctrl_bits() { return _ctrl_bits.bits(); }
  CtrlBits ctrlBits() { return _ctrl_bits; }

  void set_self_ctrl(const CtrlBits& c) { _self_bits = c; }
  uint64_t self_bits() { return _self_bits.bits(); }
  /// }

  virtual int bitwidth() override { return SS_CONFIG::bitwidth[_ssinst]; }

  uint64_t getout(int i) {
    return _output_vals[i];
  }

 private:
  friend class boost::serialization::access;

  void resize_vals(int n);

  template <class Archive>
  void serialize(Archive& ar, const unsigned version);

  std::ofstream _verif_stream;
  std::string _verif_id;

  std::vector<uint64_t> _input_vals;
  std::vector<uint64_t> _output_vals;

  bool _predInv;
  bool _isDummy;
  int _imm_slot;
  int _subFunc;
  CtrlBits _ctrl_bits;
  CtrlBits _self_bits;

  std::vector<uint64_t> _reg;

  uint64_t _imm;
  SS_CONFIG::ss_inst_t _ssinst;
};

// vector class
class SSDfgVec : public SSDfgNode {
 public:

  friend class SSDfg;

  SSDfgVec() {}

  SSDfgVec(V_TYPE v, int len, int bitwidth, const std::string& name, int id, SSDfg* ssdfg,
           const ssdfg::MetaPort &meta);

  void set_port_width(int n) { _port_width = n; }

  int get_port_width() { return _port_width; }

  void set_vp_len(int n) { _vp_len = n; }

  int get_vp_len() { return _vp_len; }

  int logical_len() { return _vp_len; }

  int length() { return _ops.size(); }

  virtual std::string gamsName() override { return _name; }

  virtual std::string name() override { return _name; }

  virtual int bitwidth() override { return _bitwidth; }

  int phys_bitwidth() { return is_temporal() ? 64 : (_values.size() * bitwidth()); }

 private:
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned version);

 protected:
  int _bitwidth;  // element bitwidth
  int _port_width;
  int _vp_len;
  ssdfg::CompileMeta meta;
};

class SSDfgVecInput : public SSDfgVec {
 public:
  static std::string Suffix() { return ""; }
  static std::string GamsPort() { return "IN PORT "; }
  static bool IsInput() { return true; }

  static const int KindValue = V_INPUT;

  SSDfgVecInput() {}

  SSDfgVecInput(int len, int width, const std::string& name, int id, SSDfg* ssdfg,
                const ssdfg::MetaPort &meta)
      : SSDfgVec(V_INPUT, len, width, name, id, ssdfg, meta) {}

  std::vector<std::pair<int, SS_CONFIG::ssnode*>> candidates(Schedule*,
                                                             SS_CONFIG::SSModel*,
                                                             int n) override;

  friend class boost::serialization::access;

  void forward() override;
  bool can_push();

  template <class Archive>
  void serialize(Archive& ar, const unsigned version);
};

class SSDfgVecOutput : public SSDfgVec {
 public:
  static std::string Suffix() { return "_out"; }
  static bool IsInput() { return false; }
  static std::string GamsPort() { return "OUT PORT "; }

  static const int KindValue = V_OUTPUT;

  SSDfgVecOutput() {}

  SSDfgVecOutput(int len, int width, const std::string& name, int id, SSDfg* ssdfg,
                 const ssdfg::MetaPort &meta)
      : SSDfgVec(V_OUTPUT, len, width, name, id, ssdfg, meta) {}

  std::vector<std::pair<int, SS_CONFIG::ssnode*>> candidates(Schedule*,
                                                             SS_CONFIG::SSModel*,
                                                             int n) override;

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

  void forward() override {}
  bool can_pop();
  void pop(std::vector<uint64_t>& data, std::vector<bool>& data_valid);

  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned version);
};

class EntryTable {
  void assert_exists(const std::string& s) {
    if (symbol_table_.count(s) == 0) {
      std::cerr << "Could not find" + s + "\n";
      assert(false);
    }
  }

 public:
  void set(const std::string& s, ParseResult* pr, bool override = false);

  void set(const std::string& s, SSDfgValue* n) { symbol_table_[s] = new ValueEntry(n); }

  void set(std::string& s, uint64_t n) { symbol_table_[s] = new ConstDataEntry(n); }

  void set(std::string& s, double n) { symbol_table_[s] = new ConstDataEntry(n); }

  bool has_sym(const std::string& s) { return symbol_table_.count(s); }

  ParseResult* get_sym(const std::string& s);

 private:
  std::map<std::string, ParseResult*> symbol_table_;
};

struct GroupProp {
  bool is_temporal{false};
  int64_t frequency{-1};
  int64_t unroll{1};

 private:
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned version);
};

class SSDfg {
 public:
  const std::string filename;

  SSDfg();

  SSDfg(std::string filename);

  ~SSDfg() {}

  void reset_simulation_state();

  static void order_nodes(SSDfgNode* node, std::set<SSDfgNode*>& done_nodes,
                          std::vector<SSDfgNode*>& ordered_nodes);

  std::vector<SSDfgNode*>& ordered_nodes();

  void printGraphviz(std::ostream& os, Schedule* sched = nullptr);

  void printGraphviz(const char* fname, Schedule* sched = nullptr);

  void start_new_dfg_group();

  void set_pragma(const std::string& c, const std::string& s);

  void printGams(std::ostream& os, std::unordered_map<std::string, SSDfgNode*>&,
                 std::unordered_map<std::string, SSDfgEdge*>&,
                 std::unordered_map<std::string, SSDfgVec*>&);

  template <int is_io, typename VecType>
  void printPortCompatibilityWith(std::ostream& os, SS_CONFIG::SSModel* ssModel);

  // remove instruction from nodes and insts
  void removeInst(SSDfgInst* inst) {
    _insts.erase(std::remove(_insts.begin(), _insts.end(), inst), _insts.end());
    _nodes.erase(std::remove(_nodes.begin(), _nodes.end(), inst), _nodes.end());
  }

  template <typename T>
  inline void add(T*);
  template <typename T>
  inline void insert_vec(T*);

  template <typename T>
  inline void add_parsed_vec(const std::string& name, int len, EntryTable& syms, int width,
                             const ssdfg::MetaPort &meta);

  void addVecOutput(const std::string& name, int len, EntryTable& syms, int width,
                    const ssdfg::MetaPort &meta);

  void addVecInput(const std::string& name, int len, EntryTable& syms, int width,
                   const ssdfg::MetaPort &meta);

  SSDfgEdge* connect(SSDfgValue* orig, SSDfgNode* dest, int slot,
                     SSDfgEdge::EdgeType etype, int l = 0, int r = 63,
                     int operand_pos = -1);

  void disconnect(SSDfgNode* orig, SSDfgNode* dest);

  ParseResult* create_inst(std::string opcode, std::vector<ParseResult*>& args);

  template <typename T>
  inline std::vector<T>& nodes();

  const std::vector<SSDfgInst*>& inst_vec() { return _insts; }

  int num_vec_input() { return _vecInputs.size(); }

  int num_vec_output() { return _vecOutputs.size(); }

  void insert_vec_in(SSDfgVecInput* in) {
    _vecInputs.push_back(in);
    _vecInputGroups.back().push_back(in);
  }

  void insert_vec_out(SSDfgVecOutput* out) {
    _vecOutputs.push_back(out);
    _vecOutputGroups.back().push_back(out);
  }

  void insert_vec_in_group(SSDfgVecInput* in, unsigned group) {
    _vecInputs.push_back(in);
    if (_vecInputGroups.size() <= group) {
      _vecInputGroups.resize(group + 1);
    }
    _vecInputGroups[group].push_back(in);
  }

  void insert_vec_out_group(SSDfgVecOutput* out, unsigned group) {
    _vecOutputs.push_back(out);
    if (_vecOutputGroups.size() <= group) {
      _vecOutputGroups.resize(group + 1);
    }
    _vecOutputGroups[group].push_back(out);
  }

  std::vector<SSDfgVecInput*> vec_inputs() { return _vecInputs; }

  std::vector<SSDfgVecOutput*> vec_outputs() { return _vecOutputs; }

  SSDfgVecInput* vec_in(int i) { return _vecInputs[i]; }

  SSDfgVecOutput* vec_out(int i) { return _vecOutputs[i]; }

  std::vector<SSDfgVecInput*>& vec_in_group(int i) { return _vecInputGroups[i]; }

  std::vector<SSDfgVecOutput*>& vec_out_group(int i) { return _vecOutputGroups[i]; }

  GroupProp& group_prop(int i) { return _groupProps[i]; }

  int num_groups() { return _vecInputGroups.size(); }

  template <typename T>
  inline void sort();

  int compute(bool print, bool verif, int group);  // atomically compute
  int maxGroupThroughput(int group);

  void instsForGroup(int g, std::vector<SSDfgInst*>& insts);

  // --- New Cycle-by-cycle interface for more advanced CGRA -----------------

  double count_starving_nodes();

  // Simulator pushes data to vector given by vector_id
  bool push_vector(SSDfgVecInput* vec_in, std::vector<uint64_t> data,
                   std::vector<bool> valid, bool print, bool verif);

  void push_transient(SSDfgValue* dfg_val, uint64_t v, bool valid, bool avail,
                      int cycle, bool is_computed) {
    struct cycle_result* temp = new cycle_result(dfg_val, v, valid, avail);
    // If this is a newly computed value, append to the back.
    // Otherwise, it is a value delayed from previous cycles, so it should be inserted before
    // any element of this cycle.
    auto &to_insert = transient_values[(cycle + cur_node_ptr) % get_max_lat()];
    to_insert.insert(is_computed ? to_insert.end() : to_insert.begin(), temp);
  }

  void push_buf_transient(SSDfgEdge* e, bool is_dummy, int cycle) {
    struct buffer_pop_info* temp = new buffer_pop_info(e, is_dummy);
    auto &to_insert = buf_transient_values[(cycle + cur_buf_ptr) % get_max_lat()];
    to_insert.push_back(temp);
  }

  int cycle(bool print, bool verif);

  int forward(bool);

  double esitimated_performance(Schedule *);

  // ---------------------------------------------------------------------------

  void preprocess_graph();

  void calc_minLats();

  void set_dbg_stream(std::ostream* dbg_stream) { _dbg_stream = dbg_stream; }

  std::ostream& dbg_stream() { return *_dbg_stream; }

  void check_for_errors();

  void inc_total_dyn_insts() { _total_dyn_insts++; }

  int total_dyn_insts() { return _total_dyn_insts; }

  int get_max_lat() { return MAX_LAT; }

  int num_node_ids() { return _num_node_ids; }

  std::vector<SSDfgEdge*>& edges() { return _edges; }

  int num_edge_ids() { return _num_edge_ids; }

  int inc_node_id() { return _num_node_ids++; }

  int inc_edge_id() { return _num_edge_ids++; }

  void push_ready_node(SSDfgNode* node) { _ready_nodes.push_back(node); }

  int64_t cur_cycle() { return _cur_cycle; }

 private:
  // to keep track of number of cycles---------------------
  struct cycle_result {
    SSDfgValue* dfg_val;
    uint64_t val;
    bool valid;
    bool avail;

    cycle_result(SSDfgValue* dfg_val_, uint64_t value, bool valid_in, bool a) {
      dfg_val = dfg_val_;
      val = value;
      valid = valid_in;
      avail = a;
    }
  };

  struct buffer_pop_info {
    SSDfgEdge* e;
    bool is_dummy;

    buffer_pop_info(SSDfgEdge* edge, bool dummy) {
      e = edge;
      is_dummy = dummy;
    }
  };

  std::vector<SSDfgNode*> _ready_nodes;

  int MAX_LAT = 1000;
  std::list<struct cycle_result*> transient_values[1000];
  int cur_node_ptr = 0;
  int cur_buf_ptr = 0;
  uint64_t _cur_cycle = 0;

  std::list<struct buffer_pop_info*> buf_transient_values[1000];

  std::unordered_map<int, uint64_t> _complex_fu_free_cycle;

  // stats
  int _total_dyn_insts = 0;

  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive& ar, const unsigned version);

  std::vector<SSDfgNode*> _nodes;

  // redundant storage:
  std::vector<SSDfgInst*> _insts;

  std::vector<SSDfgNode*> _orderedNodes;

  std::vector<SSDfgVecInput*> _vecInputs;
  std::vector<SSDfgVecOutput*> _vecOutputs;

  std::vector<SSDfgEdge*> _edges;

  std::vector<std::vector<SSDfgVecInput*>> _vecInputGroups;
  std::vector<std::vector<SSDfgVecOutput*>> _vecOutputGroups;
  std::vector<GroupProp> _groupProps;

  int _num_node_ids = 0;
  int _num_edge_ids = 0;

  std::ostream* _dbg_stream;
};

// slightly obscure syntax sugar  :p
template <>
inline std::vector<SSDfgNode*>& SSDfg::nodes() {
  return _nodes;
}
template <>
inline std::vector<SSDfgInst*>& SSDfg::nodes() {
  return _insts;
}
template <>
inline std::vector<SSDfgVecInput*>& SSDfg::nodes() {
  return _vecInputs;
}
template <>
inline std::vector<SSDfgVecOutput*>& SSDfg::nodes() {
  return _vecOutputs;
}
template <>
inline std::vector<std::vector<SSDfgVecInput*>>& SSDfg::nodes() {
  return _vecInputGroups;
}
template <>
inline std::vector<std::vector<SSDfgVecOutput*>>& SSDfg::nodes() {
  return _vecOutputGroups;
}

// TODO(@were): add enable_if!
template <typename T>
inline void SSDfg::add(T* node) {
  nodes<T*>().push_back(node);
  _nodes.push_back(node);
}

template <typename T>
inline void SSDfg::insert_vec(T* node) {
  nodes<T*>().push_back(node);
  nodes<std::vector<T*>>().back().push_back(node);
}

template <typename T>
inline void SSDfg::add_parsed_vec(const std::string& name, int len, EntryTable& syms,
                                  int width, const ssdfg::MetaPort &meta) {
  int n = std::max(1, len);
  int slice = 64 / width;
  // int t = ceil(n / float(slice)); -- FIXME: do we need this?
  // I think it's somewhat likely i am breaking decomposability
  T* vec = new T(len, width, name, (int)nodes<T*>().size(), this, meta);
  add(vec);
  vec->set_port_width(width);
  vec->set_vp_len(n);
  int left_len = 0;
  for (int i = 0, cnt = 0; i < n; i += slice) {
    left_len = slice;
    if (n - i > 0) {
      left_len = std::min(n - i, slice);
    }

    for (int j = 0; j < left_len * width; j += width) {
      std::stringstream ss;
      ss << name;
      if (len) ss << cnt++;

      // TODO(@were): Do I need to modularize these two clean up segment?
      if (std::is_same<T, SSDfgVecOutput>::value) {
        auto sym = syms.get_sym(ss.str());
        if (auto ce = dynamic_cast<ConvergeEntry*>(sym)) {
          int num_entries = ce->entries.size();
          assert(num_entries > 0 && num_entries <= 16);
          for (auto elem : ce->entries)
            connect(elem->value, vec, i, SSDfgEdge::data, elem->l, elem->r);
        } else if (auto ne = dynamic_cast<ValueEntry*>(sym)) {
          connect(ne->value, vec, i, SSDfgEdge::data, ne->l, ne->r);
        }
      } else if (std::is_same<T, SSDfgVecInput>::value) {
        SSDfgValue* value = vec->values()[i];
        syms.set(ss.str(), new ValueEntry(value, j, j + width - 1));
      }
    }
  }
  //{
  //  std::ostringstream oss;
  //  meta.to_pragma(oss);
  //  std::cout << "Parse pragmas:\n";
  //  std::cout << oss.str() << std::endl;
  //}
}

template <int is_io, typename VecType>
void SSDfg::printPortCompatibilityWith(std::ostream& os, SS_CONFIG::SSModel* ssModel) {
  bool first = true;

  for (auto& vec : nodes<VecType*>()) {
    std::vector<int> matching_ports;

    for (auto& port_interf : ssModel->subModel()->io_interf().vports_map[is_io]) {
      const std::vector<int>& port_m = port_interf.second->port_vec();

      if (port_m.size() >= (unsigned)vec->length()) {
        matching_ports.push_back(port_interf.first);
        if (!first)
          os << ", ";
        else
          first = false;
        os << vec->gamsName() << ".ip" << port_interf.first << " ";
      }
    }

    if (matching_ports.size() == 0) {
      std::cout << VecType::GamsPort() << vec->gamsName()
                << "\" DID NOT MATCH ANY HARDWARE PORT INTERFACE\n";
      assert(0);
    }
  }
}

#endif
