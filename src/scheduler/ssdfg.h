#ifndef __SSDFG_H__
#define __SSDFG_H__

#include "ssinst.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <map>
#include <vector>
#include <queue>
#include <list>
#include <assert.h>
#include <sstream>
#include <algorithm>
#include "model.h"
#include <bitset>
#include <unordered_set>
#include <math.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


using SS_CONFIG::SubModel;

void checked_system(const char* command);

class Schedule;
class SSDfgNode;
class SSDfg;
class SSDfgEdge;

// Datastructure describing a value created by an input vector or an instruction
// Values are always routed "together" on adjacent subnetworks
class SSDfgValue {
public:
  SSDfgValue() {} //for serialization
  SSDfgValue(SSDfgNode* node, int index) : _node(node), _index(index) {}

  SSDfgNode *node() const {return _node;}
  int index() const {return _index;}
  const std::vector<SSDfgEdge*> edges() const {return _uses;}
  int bitwidth() const {return _bitwidth;}

  void addOutEdge(SSDfgEdge* edge);

  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive &ar, const unsigned int version);

  //Dynamic Stuff
  void set_val(uint64_t v) { _val = v;}
  uint64_t get_val() {return _val;}

private: 
  SSDfgNode *_node=nullptr; //This is the node that it belongs to
  int _index=0; //within the node, what index is it in its value list
  int _bitwidth=64;
  std::vector<SSDfgEdge *> _uses; //storage for all outgoing edges

  //Dynamic Stuff
  uint64_t _val = 0; //dynamic var (setting the default value)

};


class SSDfgEdge {
public:
  SSDfgEdge() {}

  enum EdgeType { data, ctrl, ctrl_true, ctrl_false };

  EdgeType etype() { return _etype; }

  SSDfgEdge(SSDfgValue *def, SSDfgNode *use,
            EdgeType etype, SSDfg *ssdfg, int l = 0, int r = 63);

  /// Source and destination
  SSDfgNode *def() const;
  SSDfgValue *val() const;
  SSDfgNode *use() const;
  SSDfgNode *get(int) const;

  int id();
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

  //Calculate the value based on the origin's value, and the edge's
  //index and bitwidth fields
  uint64_t extract_value(uint64_t v_in);

  uint64_t get_buffer_val();

  bool get_buffer_valid();

  void pop_buffer_val(bool print, bool verif);

  int bitwidth();
  int l();
  int r();

  uint64_t get_value();

  void reset_associated_buffer();

  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive &ar, const unsigned int version);

private:
  int _ID;
  SSDfg *_ssdfg;
  SSDfgValue* _value; //value which produces the node
  SSDfgNode* _use; //operand which consumes the edge
  EdgeType _etype;
  int _l, _r;

  //Runtime Types
  std::queue<std::pair<uint64_t, bool>> _data_buffer;
  // using 2 since 1st entry is used for bp
  unsigned int buf_len = 9;

  int _delay = 0;
};

class SSDfgInst;

// Datastructure describing the operand
struct SSDfgOperand {
  SSDfgOperand() {}

  SSDfgOperand(SSDfgEdge *e);

  SSDfgOperand(std::vector<SSDfgEdge *> es);

  SSDfgOperand(uint64_t);

  //Helper functions
  SSDfgEdge *get_first_edge() const;

  void clear();

  bool is_ctrl();

  bool is_imm();

  bool is_composed();

  //Functions which manipulate dynamic state
  uint64_t get_value();

  uint64_t get_buffer_val();

  uint64_t get_buffer_valid();

  uint64_t is_buffer_empty();

  void pop_buffer_val(bool print, bool verif);

  // An Operand is valid as long as ALL of its edges are valid
  bool valid();

  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned int version);

  //Edges concatenated in bit order from least to most significant
  std::vector<SSDfgEdge *> edges;
  uint64_t imm = 0;
};

//DFG Node -- abstract base class
//DFG Nodes are intended to be the scheduling unit
class SSDfgNode {
public:
  virtual ~SSDfgNode() {}

  SSDfgNode() {
    _values.resize(1,new SSDfgValue(this,0));
  }

  enum V_TYPE {
    V_INVALID, V_INPUT, V_OUTPUT, V_INST, V_NUM_TYPES
  };

  virtual void printGraphviz(std::ostream &os, Schedule *sched = nullptr);

  // some issue with this function
  virtual uint64_t invalid();

  SSDfgNode(SSDfg *ssdfg, V_TYPE v);
  SSDfgNode(SSDfg *ssdfg, V_TYPE v, const std::string &name);

  typedef std::vector<SSDfgEdge *>::const_iterator const_edge_iterator;
  typedef std::vector<SSDfgOperand>::const_iterator const_op_iterator;

  //Add edge to operand in least to most significant bit order
  void addOperand(unsigned pos, SSDfgEdge *e);
  void addOutEdge(SSDfgEdge *edge);
  void remove_edge(SSDfgNode *to_erase, int idx);
  void removeIncEdge(SSDfgNode *orig);
  void removeOutEdge(SSDfgNode *dest);
  void reset_node();

  void validate();

  virtual int compute(bool print, bool verif) { return 0; }

  //-----------------------------------------
  virtual int update_next_nodes(bool print, bool verif) { return 0; }
  virtual int compute_backcgra(bool print, bool verif) { _inputs_ready = 0; return 0; }
  //-------------------------------------------------------

  virtual int maxThroughput(); 

  virtual int lat_of_inst() { return 0; }

  virtual void depInsts(std::vector<SSDfgInst *> &insts);

  virtual std::string name() = 0;     //pure func

  virtual std::string gamsName() = 0;

  SSDfgEdge *getLinkTowards(SSDfgNode *to);

  int num_inputs_ready() { return _inputs_ready; }

  SSDfgEdge *first_inc_edge() { return _ops[0].edges[0]; }

  SSDfgOperand &first_operand() { return _ops[0]; }

  SSDfgNode *first_op_node() { return (_ops[0].edges[0]->def()); }

  SSDfgNode *first_use() { return (_uses[0]->use()); }

  size_t num_inc() const { return _ops.size(); }

  size_t num_out() const { return _uses.size(); }

  void setName(std::string name) { _name = name; }

  const std::vector<SSDfgOperand> &ops() {return _ops; }

  const std::vector<SSDfgEdge*> &in_edges() { return _inc_edge_list; }

  std::vector<SSDfgValue*> &values() { return _values; }

  const std::vector<SSDfgEdge*> &uses() { return _uses; }

  int id() { return _ID; }

  bool get_bp();

  // Check's all consumers for backpressure-freedom,
  // If backpressure,
  void set_node(SSDfgValue* dfg_val, 
      uint64_t v, bool valid, bool avail, bool print, bool verif);

  // sets value in non-backcgra
  void set_value(uint64_t v, bool valid) { 
    _values[0]->set_val(v); 
    _invalid = !valid; 
  }

  // sets value at this cycle
  void set_value(SSDfgValue* dfg_val,uint64_t v, bool valid, bool avail, int cycle);
  //--------------------------------------------

  bool get_avail() { return _avail; }

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

  virtual int bitwidth() { return 64; }
  //---------------------------------------------------------------------------

  V_TYPE type() { return _vtype; }

  int group_id() { return _group_id; }

  int num_inc_edges() { return _inc_edge_list.size(); }

private:

  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned int version);

protected:
  SSDfg *_ssdfg;  //sometimes this is just nice to have : )

  int _node_id = -1; //hack for temporal simulator to remember _node_id

  //Dynamic stuff
  bool _avail = false; // if their is data in the output buffer
  bool _invalid = false;
  int _inputs_ready = 0; //dynamic inputs ready
  std::vector<bool> _back_array;     //in edges

  //Static Stuff
  int _ID;
  std::string _name;
  std::vector<SSDfgOperand> _ops;     //in edges
  std::vector<SSDfgEdge *> _inc_edge_list; //in edges in flat form

  std::vector<SSDfgValue*> _values;    //out edges by index
  std::vector<SSDfgEdge *> _uses;       //out edges in flat form

  int _min_lat = 0;
  int _sched_lat = 0;
  int _max_thr = 0;
  int _group_id = 0; //which group do I belong to

  V_TYPE _vtype;
};

class ScheduleUnit {
public:
  ScheduleUnit() = default;
  virtual std::vector<std::pair<int, int>> candidates(Schedule *, SS_CONFIG::SSModel *, int n) = 0;
  virtual std::vector<std::pair<int, SS_CONFIG::ssnode*>>
  ready_to_map(SS_CONFIG::SSModel *, const std::pair<int, int> &) = 0;
  virtual std::vector<SSDfgNode*> ready_to_map() = 0;
};

typedef std::vector<std::string> string_vec_t;

//post-parsing control signal definitions (mapping of string of flag to it's value?)
typedef std::map<int,string_vec_t> ctrl_def_t;

struct ParseResult {

  virtual ~ParseResult() {}
};

struct ConstDataEntry : ParseResult {

  ConstDataEntry(uint64_t i) : data(i) {}

  ConstDataEntry(uint64_t i1, uint64_t i2) : data((i2 << 32) | (i1 & 0xFFFFFFFF)) {}

  ConstDataEntry(uint64_t i1, uint64_t i2, uint64_t i3, uint64_t i4) :
    data(((i4 & 0xFFFF) << 48) | ((i3 & 0xFFFF) << 32) | ((i2 & 0xFFFF) << 16) | (i1 & 0xFFFF)) {}

  ConstDataEntry(double d) : d(d) {}

  ConstDataEntry(float f0, float f1) : f0(f0), f1(f1) {}

  union {
    struct { float f0, f1; };
    double d;
    uint64_t data;
  };

};

struct ValueEntry : ParseResult {

  ValueEntry(SSDfgValue *value_, int l_ = 0, int r_ = 63) : value(value_), l(l_), r(r_) {}

  SSDfgValue *value;
  int l, r;
};

struct ConvergeEntry : ParseResult {
  std::vector<ValueEntry*> entries;
};

struct CtrlBits {
  enum Control {
    B1, B2, Discard, Reset, Abstain, Total
  };

  CtrlBits(const std::map<int, std::vector<std::string>> &raw);
  CtrlBits() : mask(0) {}

  void set(uint64_t val, Control b);
  bool test(uint64_t val, Control b);
  void test(uint64_t val, std::vector<bool> &back_array, bool &discard,
            bool &predicate, bool &reset);

  uint64_t bits() { return mask; }

  friend class boost::serialization::access;
  template<class Archive> void serialize(Archive & ar, const unsigned version);

private:
  uint64_t mask{0};

  static Control str_to_enum(const std::string &s) {
    if (s == "b1") return B1;
    if (s == "b2") return B2;
    if (s == "d") return Discard;
    if (s == "r") return Reset;
    if (s == "a") return Abstain;
    assert(false && "Not a valid command");
  }
};

struct ControlEntry : ParseResult {

  void set_flag(const std::string &s);

  ControlEntry(const std::string &s, ParseResult *controller_) : controller(controller_), bits() {
    set_flag(s);
  }

  ControlEntry(const std::string &s, std::map<int, std::vector<std::string>> &bits_,
               ParseResult *controller_) :
    controller(controller_), bits(bits_) { set_flag(s); }

  SSDfgEdge::EdgeType flag;
  ParseResult *controller;
  CtrlBits bits;
};

//Instruction
class SSDfgInst : public SSDfgNode, ScheduleUnit {
public:
  using MapsTo = SS_CONFIG::ssfu;
  static const int KindValue = V_INST;

  SSDfgInst() {}

  std::vector<std::pair<int, int>> candidates(Schedule *, SS_CONFIG::SSModel *, int n) override;

  std::vector<std::pair<int, SS_CONFIG::ssnode*>>
  ready_to_map(SS_CONFIG::SSModel *, const std::pair<int, int> &) override;

  std::vector<SSDfgNode*> ready_to_map() override {
    return {this};
  }

  SSDfgInst(SSDfg *ssdfg, SS_CONFIG::ss_inst_t inst, bool is_dummy = false) :
    SSDfgNode(ssdfg, V_INST), _predInv(false), _isDummy(is_dummy), _imm_slot(-1),
    _subFunc(0), _reg(8, 0), _ssinst(inst) {
    //DFG Node makes a value by default, so start at 1
    for(int i = 1; i < SS_CONFIG::num_values(_ssinst); ++ i) {
      _values.push_back(new SSDfgValue(this,i));
    }
  }


  SSDfgInst(SSDfg *ssdfg) :
    SSDfgNode(ssdfg, V_INST), _predInv(false), _isDummy(false), _imm_slot(-1), _subFunc(0),
    _reg(8, 0) {}

  virtual int lat_of_inst() override { return inst_lat(inst()); }

  void setImm(uint64_t val) { _imm = val; }

  uint64_t imm() { return _imm; }

  void setPredInv(bool predInv) { _predInv = predInv; }

  bool predInv() { return _predInv; }

  bool isDummy() { return _isDummy; }

  SS_CONFIG::ss_inst_t inst() { return _ssinst; }

  //Adding new function in the header file
  int update_next_nodes(bool print, bool verif);

  virtual int maxThroughput();

  virtual void depInsts(std::vector<SSDfgInst *> &insts);

  std::string name();

  std::string gamsName();

  void setImmSlot(int i);

  int immSlot() const { return _imm_slot; }

  void setSubFunc(int i) { _subFunc = i; }

  int subFunc() const { return _subFunc; }

  uint64_t do_compute(bool &discard);

  virtual int compute(bool print, bool verif);

  void print_output(std::ostream& os);

  // new line added
  virtual int compute_backcgra(bool print, bool verif);

  void set_verif_id(std::string s) { _verif_id = s; }

  virtual uint64_t invalid() { return _invalid; }

  /// Control signal, either controlled by an dependent inst or itself
  /// {
  void set_ctrl_bits(const CtrlBits &c) { _ctrl_bits = c; }
  uint64_t ctrl_bits() { return _ctrl_bits.bits(); }
  CtrlBits ctrlBits() { return _ctrl_bits; }

  void set_self_ctrl(const CtrlBits &c) { _self_bits = c; }
  uint64_t self_bits() { return _self_bits.bits(); }
  /// }

  virtual int bitwidth() { return SS_CONFIG::bitwidth[_ssinst]; }

  bool yield(Schedule *, SS_CONFIG::SubModel *) { return true; }

  uint64_t getout(int i) {
    switch(bitwidth()) {
      case 64: return _output_vals[i];
      case 32: return _output_vals_32[i];
      case 16: return _output_vals_16[i];
      case 8:  return _output_vals_8[i];
      default:
        std::cout << "Weird bitwidth: " << bitwidth() << "\n";
        assert(0 && "weird bitwidth");
    }
  }

private:
  friend class boost::serialization::access;

  void resize_vals(int n);

  template<class Archive>
  void serialize(Archive &ar, const unsigned version);

  std::ofstream _verif_stream;
  std::string _verif_id;
  std::vector<uint64_t> _input_vals;
  std::vector<uint32_t> _input_vals_32;
  std::vector<uint16_t> _input_vals_16;
  std::vector<uint8_t> _input_vals_8;

  std::vector<uint64_t> _output_vals;
  std::vector<uint32_t> _output_vals_32;
  std::vector<uint16_t> _output_vals_16;
  std::vector<uint8_t> _output_vals_8;

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

class SSDfgVec;
class SSDfgIO : public SSDfgNode {
public:
  SSDfgIO() {}

  SSDfgIO(SSDfg *ssdfg, const std::string& name, SSDfgVec *vec_, V_TYPE v);

  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned version);
  SSDfgVec *vector() { return vec_; }

protected:
  SSDfgVec *vec_;

};

class SSDfgVecInput;

class SSDfgInput : public SSDfgIO {
public:
  using MapsTo = SS_CONFIG::ssinput;

  static const int KindValue = V_INPUT;

  SSDfgInput() {}

  SSDfgInput(SSDfg *ssdfg, const std::string &name, SSDfgVec *vec_) : SSDfgIO(ssdfg, name, vec_, V_INPUT) {}

  SSDfgVecInput *input_vec();

  std::string gamsName() override;

  std::string name() override;

  // after inc inputs ready?
  int compute_backcgra(bool print, bool verif) override;

  int compute(bool print, bool verif) override;

private:
  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned int version);

};

class SSDfgVecOutput;
class SSDfgOutput : public SSDfgIO {
public:
  SSDfgOutput() {}

  using MapsTo = SS_CONFIG::ssoutput;
  static const int KindValue = V_OUTPUT;

  void printDirectAssignments(std::ostream &os, std::string dfg_name);
  //virtual void printEmuDFG(std::ostream& os, std::string dfg_name, std::string* realName, int* iter, std::vector<int>* output_sizes);

  SSDfgOutput(SSDfg *ssdfg, const std::string &name, SSDfgVec *vec_) : SSDfgIO(ssdfg, name, vec_, V_OUTPUT) {}

  SSDfgVecOutput *output_vec();

  std::string gamsName() override;

  std::string name() override;

  //returns the instruction producing the
  //value to this output node
  //Returns nullptr if the producing instruction is an input!
  //TODO:FIXME: This might not be safe for decomp-CGRA
  SSDfgInst *out_inst() { return dynamic_cast<SSDfgInst *>(_ops[0].edges[0]->def()); }

  //retrieve the value of the def
  uint64_t retrieve() { assert(_ops.size() == 1); return _ops[0].get_value(); }

  uint64_t parent_invalid() { return _ops[0].edges[0]->def()->invalid(); }

  virtual uint64_t invalid() override { return _invalid; }

private:
  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned int version);

};

//vector class
class SSDfgVec {
public:
  SSDfgVec() {}

  SSDfgVec(int len, const std::string &name, int id, SSDfg *ssdfg);

  bool yield(Schedule *, SS_CONFIG::SubModel *);

  int id() { return _ID; }

  void set_group_id(int id) { _group_id = id; }

  int group_id() { return _group_id; }

  bool is_temporal();

  virtual std::string gamsName() = 0;

  virtual std::string name() { return _name; }

  void set_port_width(int n) { _port_width=n; }

  int get_port_width() { return _port_width; }

  void set_vp_len(int n) { _vp_len=n; }

  int get_vp_len() { return _vp_len; }

  int logical_len() { return _vp_len; }

  virtual int wasted_width(Schedule *, SubModel *) = 0;

  std::vector<SSDfgIO*> &vector() { return vector_; }

  virtual void add(SSDfgIO *) = 0;

  unsigned length() { return vector().size(); }

private:
  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned version);

protected:
  std::string _name;
  int _ID;
  SSDfg *_ssdfg;
  int _group_id = 0; //which group do I belong to
  int _port_width;
  int _vp_len;

  std::vector<SSDfgIO*> vector_;

  std::vector<SSDfgNode*> _to_node_vector() {
    SSDfgNode **from = (SSDfgNode**)(&vector_[0]);
    SSDfgNode **to = from + vector_.size();
    return std::vector<SSDfgNode*>(from, to);
  }
};


class SSDfgVecInput : public SSDfgVec, ScheduleUnit {
public:
  using Scalar = SSDfgInput;

  static std::string Suffix() { return ""; }
  static std::string GamsPort() { return "IN PORT "; }
  static bool IsInput() { return true; }

  SSDfgVecInput() {}

  SSDfgVecInput(int len, const std::string &name, int id, SSDfg *ssdfg) : SSDfgVec(len, name, id, ssdfg) {}

  virtual std::string gamsName() override;

  int wasted_width(Schedule *, SubModel *) override;

  bool backPressureOn();

  void add(SSDfgIO *in)  override {
    assert(dynamic_cast<Scalar*>(in));
    vector_.push_back(in);
  }

  SSDfgInput *at(int i) {
    return dynamic_cast<Scalar*>(vector_[i]);
  }

  std::vector<std::pair<int, int>> candidates(Schedule *, SS_CONFIG::SSModel *, int n) override;

  std::vector<std::pair<int, SS_CONFIG::ssnode*>>
  ready_to_map(SS_CONFIG::SSModel *, const std::pair<int, int> &) override;

  std::vector<SSDfgNode*> ready_to_map() override {
    return _to_node_vector();
  }

  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned version);

};

class SSDfgVecOutput : public SSDfgVec, ScheduleUnit {
public:
  using Scalar = SSDfgOutput;

  static std::string Suffix() { return "_out"; }
  static bool IsInput() { return false; }
  static std::string GamsPort() { return "OUT PORT "; }

  SSDfgVecOutput() {}

  SSDfgVecOutput(int len, const std::string &name, int id, SSDfg *ssdfg) : SSDfgVec(len, name, id, ssdfg) {}

  std::vector<std::pair<int, int>> candidates(Schedule *, SS_CONFIG::SSModel *, int n) override;

  std::vector<std::pair<int, SS_CONFIG::ssnode*>>
  ready_to_map(SS_CONFIG::SSModel *, const std::pair<int, int> &) override;

  int wasted_width(Schedule *, SubModel *) override;

  virtual std::string gamsName() override;

  std::vector<SSDfgNode*> ready_to_map() override {
    return _to_node_vector();
  }

  void add(SSDfgIO *in)  override {
    assert(dynamic_cast<Scalar*>(in));
    vector_.push_back(in);
  }

  SSDfgOutput *at(int i) {
    return dynamic_cast<Scalar*>(vector_[i]);
  }

  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned version);

};

class EntryTable {
  void assert_exists(const std::string &s) {
    if (symbol_table_.count(s) == 0) {
      std::cerr << "Could not find" + s + "\n";
      assert(false);
    }
  }

public:

  void set(const std::string &s, ParseResult *pr, bool override = false);

  void set(const std::string &s, SSDfgValue *n) { symbol_table_[s] = new ValueEntry(n); }

  void set(std::string &s, uint64_t n) { symbol_table_[s] = new ConstDataEntry(n); }

  void set(std::string &s, double n) { symbol_table_[s] = new ConstDataEntry(n); }

  bool has_sym(const std::string &s) { return symbol_table_.count(s); }

  ParseResult *get_sym(const std::string &s);

private:
  std::map<std::string, ParseResult*> symbol_table_;
};

struct GroupProp {
  bool is_temporal = false;

private:
  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned version);
};

class SSDfg {
public:
  SSDfg();

  SSDfg(std::string filename);

  ~SSDfg() {
  }

  void remap(int num_HW_FU);

  bool remappingNeeded();

  void rememberDummies(std::set<SSDfgOutput *> d);

  void removeDummies();

  void reset_simulation_state();

  static void order_insts(SSDfgInst *inst, std::set<SSDfgInst *> &done_nodes, // done insts
                          std::vector<SSDfgInst *> &ordered_insts);

  std::vector<SSDfgInst *> &ordered_insts(); 

  void printGraphviz(std::ostream &os, Schedule *sched = nullptr);

  void printGraphviz(const char *fname, Schedule *sched = nullptr);

  void start_new_dfg_group();

  void set_pragma(std::string &c, std::string &s);

  void printGams(std::ostream &os, std::unordered_map<std::string, SSDfgNode *> &,
                 std::unordered_map<std::string, SSDfgEdge *> &,
                 std::unordered_map<std::string, SSDfgVec *> &);

  template<int is_io, typename VecType>
  void printPortCompatibilityWith(std::ostream &os, SS_CONFIG::SSModel *ssModel);

  // remove instruction from nodes and insts
  void removeInst(SSDfgInst *inst) {
    _insts.erase(std::remove(_insts.begin(), _insts.end(), inst), _insts.end());
    _nodes.erase(std::remove(_nodes.begin(), _nodes.end(), inst), _nodes.end());
  }

  template<typename T> inline void add(T *);
  template<typename T> inline void insert_vec(T *);

  template<typename T>
  inline void add_parsed_vec(const std::string &name, int len, EntryTable &syms, int width);

  void addVecOutput(const std::string &name, int len, EntryTable &syms, int width);
  
  void addVecInput(const std::string &name, int len, EntryTable &syms, int width);


  SSDfgEdge *connect(SSDfgValue *orig, SSDfgNode *dest, int slot,
                      SSDfgEdge::EdgeType etype, int l = 0, int r = 63);


  void disconnect(SSDfgNode *orig, SSDfgNode *dest);

  ParseResult *create_inst(std::string opcode, std::vector<ParseResult*> &args);

  typedef std::vector<SSDfgOutput *>::const_iterator const_output_iterator;

  template<typename T> inline std::vector<T> &nodes();

  const std::vector<SSDfgInst *> &inst_vec() { return _insts; }

  const std::vector<SSDfgInput*> &inputs() { return _inputs; }

  const std::vector<SSDfgOutput*> &outputs() { return _outputs; }

  int num_vec_input() { return _vecInputs.size(); }

  int num_vec_output() { return _vecOutputs.size(); }

  void insert_vec_in(SSDfgVecInput *in) {
    _vecInputs.push_back(in);
    _vecInputGroups.back().push_back(in);
  }

  void insert_vec_out(SSDfgVecOutput *out) {
    _vecOutputs.push_back(out);
    _vecOutputGroups.back().push_back(out);
  }

  void insert_vec_in_group(SSDfgVecInput *in, unsigned group) {
    _vecInputs.push_back(in);
    if (_vecInputGroups.size() <= group) {
      _vecInputGroups.resize(group + 1);
    }
    _vecInputGroups[group].push_back(in);
  }

  void insert_vec_out_group(SSDfgVecOutput *out, unsigned group) {
    _vecOutputs.push_back(out);
    if (_vecOutputGroups.size() <= group) {
      _vecOutputGroups.resize(group + 1);
    }
    _vecOutputGroups[group].push_back(out);
  }

  int find_group_for_vec(SSDfgVecInput *in) {
    for (unsigned i = 0; i < _vecInputGroups.size(); ++i) {
      for (SSDfgVecInput *v : _vecInputGroups[i]) {
        if (v == in) {
          return i;
        }
      }
    }
    assert(0 && "Vec Input not found");
  }

  int find_group_for_vec(SSDfgVecOutput *out) {
    for (unsigned i = 0; i < _vecOutputGroups.size(); ++i) {
      for (SSDfgVecOutput *v : _vecOutputGroups[i]) {
        if (v == out) {
          return i;
        }
      }
    }
    assert(0 && "Vec Output not found");
  }

  SSDfgVecInput *vec_in(int i) { return _vecInputs[i]; }

  SSDfgVecOutput *vec_out(int i) { return _vecOutputs[i]; }

  std::vector<SSDfgVecInput *> &vec_in_group(int i) { return _vecInputGroups[i]; }

  std::vector<SSDfgVecOutput *> &vec_out_group(int i) { return _vecOutputGroups[i]; }

  GroupProp &group_prop(int i) { return _groupProps[i]; }

  int num_groups() { return _vecInputGroups.size(); }

  template<typename T> inline void sort();

  int compute(bool print, bool verif, int group);  //atomically compute
  int maxGroupThroughput(int group);

  void instsForGroup(int g, std::vector<SSDfgInst *> &insts);


  // --- New Cycle-by-cycle interface for more advanced CGRA -----------------

  double count_starving_nodes();



  //Simulator pushes data to vector given by vector_id
  bool push_vector(SSDfgVecInput *vec_in, std::vector<uint64_t> data, std::vector<bool> valid, bool print, bool verif);

  // check if some value present at input node or if
  // there is some backpressure or invalid value
  bool can_push_input(SSDfgVecInput *vec_in); 

  //Simulator would like to pop size elements from vector port (vector_id)
  bool can_pop_output(SSDfgVecOutput *vec_out, unsigned int len);

  //Simulator grabs size elements from vector port (vector_id)
  //assertion failure on insufficient size
  void pop_vector_output(SSDfgVecOutput *vec_out, std::vector<uint64_t> &data,
                         std::vector<bool> &data_valid, unsigned int len, bool print, bool verif);

  void push_transient(SSDfgValue *dfg_val, uint64_t v, bool valid, 
      bool avail, int cycle) {
    struct cycle_result *temp = new cycle_result(dfg_val, v, valid, avail);
    transient_values[(cycle + cur_node_ptr) % get_max_lat()].push_back(temp);
  }

  void push_buf_transient(SSDfgEdge *e, bool is_dummy, int cycle) {
    struct buffer_pop_info *temp = new buffer_pop_info(e, is_dummy);
    buf_transient_values[(cycle + cur_buf_ptr) % get_max_lat()].push_back(temp);
  }

  int cycle(bool print, bool verif);

// ---------------------------------------------------------------------------

  std::set<SSDfgOutput *> getDummiesOutputs() { return dummiesOutputs; }

  void calc_minLats();

  void set_dbg_stream(std::ostream *dbg_stream) { _dbg_stream = dbg_stream; }

  std::ostream &dbg_stream() { return *_dbg_stream; }

  void check_for_errors();

  void inc_total_dyn_insts() { _total_dyn_insts++; }

  int total_dyn_insts() { return _total_dyn_insts; }

  int get_max_lat() { return MAX_LAT; }

  int num_node_ids() { return _num_node_ids; }

  std::vector<SSDfgEdge*>& edges() {return _edges;}

  int num_edge_ids() { return _num_edge_ids; }

  int inc_node_id() { return _num_node_ids++; }

  int inc_edge_id() { return _num_edge_ids++; }

  void push_ready_node(SSDfgNode *node) { _ready_nodes.push_back(node); }

private:
  // to keep track of number of cycles---------------------
  struct cycle_result {
    SSDfgValue *dfg_val;
    uint64_t val;
    bool valid;
    bool avail;

    cycle_result(SSDfgValue *dfg_val_, uint64_t value, bool valid_in, bool a) {
      dfg_val = dfg_val_;
      val = value;
      valid = valid_in;
      avail = a;
    }
  };

  struct buffer_pop_info {
    SSDfgEdge *e;
    bool is_dummy;

    buffer_pop_info(SSDfgEdge *edge, bool dummy) {
      e = edge;
      is_dummy = dummy;
    }
  };

  std::vector<SSDfgNode *> _ready_nodes;

  int MAX_LAT = 1000;
  std::list<struct cycle_result *> transient_values[1000];
  int cur_node_ptr = 0;
  int cur_buf_ptr = 0;
  uint64_t _cur_cycle = 0;

  std::list<struct buffer_pop_info *> buf_transient_values[1000];

  std::unordered_map<int, uint64_t> _complex_fu_free_cycle;

  //stats
  int _total_dyn_insts = 0;

  friend class boost::serialization::access;

  template<class Archive>
  void serialize(Archive &ar, const unsigned version);

  std::vector<SSDfgNode *> _nodes;

  //redundant storage:
  std::vector<SSDfgInst *> _insts;
  std::vector<SSDfgInput *> _inputs;
  std::vector<SSDfgOutput *> _outputs;

  std::vector<SSDfgInst *> _orderedInsts;
  std::vector<std::vector<SSDfgInst *>> _orderedInstsGroup;

  std::vector<SSDfgVecInput *> _vecInputs;
  std::vector<SSDfgVecOutput *> _vecOutputs;

  std::vector<SSDfgEdge *> _edges;

  std::map<std::pair<SSDfgNode *, SSDfgNode *>, SSDfgEdge *> removed_edges;

  std::vector<std::vector<SSDfgVecInput *>> _vecInputGroups;
  std::vector<std::vector<SSDfgVecOutput *>> _vecOutputGroups;
  std::vector<GroupProp> _groupProps;

  int _num_node_ids = 0;
  int _num_edge_ids = 0;

  //Dummy Stuffs:
  std::map<SSDfgOutput *, SSDfgInst *> dummy_map;
  std::map<SSDfgNode *, int> dummys_per_port;
  std::set<SSDfgInst *> dummies;
  std::set<SSDfgOutput *> dummiesOutputs;

  std::ostream *_dbg_stream;
};

template<> inline std::vector<SSDfgNode*> & SSDfg::nodes() { return _nodes; }
template<> inline std::vector<SSDfgInput*> & SSDfg::nodes() { return _inputs; }
template<> inline std::vector<SSDfgOutput*> & SSDfg::nodes() { return _outputs; }
template<> inline std::vector<SSDfgInst*> & SSDfg::nodes() { return _insts; }
template<> inline std::vector<SSDfgVecInput*> & SSDfg::nodes() { return _vecInputs; }
template<> inline std::vector<SSDfgVecOutput*> & SSDfg::nodes() { return _vecOutputs; }
template<> inline std::vector<std::vector<SSDfgVecInput*>> & SSDfg::nodes() { return _vecInputGroups; }
template<> inline std::vector<std::vector<SSDfgVecOutput*>> & SSDfg::nodes() { return _vecOutputGroups; }

template<typename T> inline void SSDfg::sort() {
  auto &to_sort = nodes<T*>();
  std::sort(to_sort.begin(), to_sort.end(), [](T *&left, T *&right) {
      return left->vector().size() > right->vector().size();
  });
}

// TODO(@were): add enable_if!
template<typename T> inline void SSDfg::add(T *node) {
  nodes<T*>().push_back(node);
  _nodes.push_back(node);
}

template<typename T> inline void SSDfg::insert_vec(T *node) {
  nodes<T*>().push_back(node);
  nodes<std::vector<T*>>().back().push_back(node);
}

template<typename T>
inline void SSDfg::add_parsed_vec(const std::string &name, int len, EntryTable &syms, int width) {

  int n = std::max(1, len);
  int slice = 64 / width;
  int t = ceil(n / float(slice));
  T *vec = new T(t, name, (int) nodes<T*>().size(), this);
  insert_vec<T>(vec);
  vec->set_port_width(width);
  vec->set_vp_len(n);
  int left_len = 0;
  for (int i = 0, cnt = 0; i < n; i += slice) {
    typename T::Scalar *node = new typename T::Scalar(this, name + T::Suffix(), vec);
 
    left_len = slice;
    if(n-i>0) {
      left_len = std::min(n-i,slice);
    }
 
    add<typename T::Scalar>(node);
    vec->add(node);
 
    for (int j = 0; j < left_len*width; j += width) {
      std::stringstream ss;
      ss << name;
      if (len)
        ss << cnt++;
      
      // TODO(@were): Do I need to modularize these two clean up segment?
      if (std::is_same<T, SSDfgVecOutput>::value) {
        auto sym = syms.get_sym(ss.str());
        if (auto ce = dynamic_cast<ConvergeEntry*>(sym)) {
          int num_entries = ce->entries.size();
          assert(num_entries > 0 && num_entries <= 16);
          for (auto elem : ce->entries)
            connect(elem->value, node, 0, SSDfgEdge::data, elem->l, elem->r);
        } else if (auto ne = dynamic_cast<ValueEntry*>(sym)) {
          connect(ne->value, node, 0, SSDfgEdge::data, ne->l, ne->r);
        }
      } else if (std::is_same<T, SSDfgVecInput>::value) {
        syms.set(ss.str(), new ValueEntry(node->values()[0], j, j + width - 1));
      }

    }
  }
}

template<int is_io, typename VecType>
void SSDfg::printPortCompatibilityWith(std::ostream &os, SS_CONFIG::SSModel *ssModel) {
  bool first = true;

  for(auto& vec : nodes<VecType*>()) {
    std::vector<int> matching_ports;

    for(auto& port_interf : ssModel->subModel()->io_interf().vports_map[is_io]) {
      const std::vector<int>& port_m = port_interf.second->port_vec();

      if(port_m.size() >= vec->length()) {
        matching_ports.push_back(port_interf.first);
        if (!first)
          os << ", ";
        else
          first = false;
        os << vec->gamsName() << ".ip" << port_interf.first << " ";
      }
    }

    if(matching_ports.size()==0) {
      std::cout << VecType::GamsPort() << vec->gamsName() << "\" DID NOT MATCH ANY HARDWARE PORT INTERFACE\n";
      assert(0);
    }
  }

}

#endif
