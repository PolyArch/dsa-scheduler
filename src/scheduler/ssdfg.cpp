#include "ssdfg.h"
#include <iomanip>
#include <list>
#include <set>
#include <string>
#include <vector>
#include "dfg-parser.tab.h"
#include "ss-config/model_parsing.h"
#include "schedule.h"

using namespace std;
using namespace SS_CONFIG;

/// { misc

void checked_system(const char* command) {
  int ret = system(command);
  if (ret) {
    std::cout << "Command: \"" << command << "\" failed with return value: " << ret
              << "\n";
  }
}

std::string SSDfgEdge::name() {
  std::stringstream ss;
  ss << def()->name() << "->" << use()->name();
  return ss.str();
}

// -- Gams names --
std::string SSDfgEdge::gamsName() {
  std::stringstream ss;
  ss << def()->gamsName() << "_" << use()->gamsName() << "i" << _ID;
  return ss.str();
}

SSDfgEdge::SSDfgEdge(SSDfgValue* val, SSDfgNode* use, EdgeType etype, SSDfg* ssdfg, int l,
                     int r)
    : _ID(ssdfg->inc_edge_id()), _ssdfg(ssdfg), _etype(etype), _l(l), _r(r) {
  _value = val;
  _use = use;
}

void SSDfgEdge::compute_after_push(bool print, bool verif) {
  if (_data_buffer.size() == 1) {
    use()->inc_inputs_ready_backcgra(print, verif);
  }
}

void SSDfgEdge::compute_after_pop(bool print, bool verif) {
  if (_data_buffer.size() > 0) {
    use()->inc_inputs_ready_backcgra(print, verif);
  }
}

SSDfgValue* SSDfgEdge::val() const { return _value; }

SSDfgNode* SSDfgEdge::def() const { return _value->node(); }

SSDfgNode* SSDfgEdge::use() const { return _use; }

// wtf is this for?
SSDfgNode* SSDfgEdge::get(int x) const {
  if (x == 0) return def();
  if (x == 1) return _use;
  assert(0);
  return NULL;
}

int SSDfgEdge::id() { return _ID; }

uint64_t SSDfgEdge::extract_value(uint64_t val) {
  if (_r - _l + 1 == 64) {  // this is special cased because << 64 is weird in c
    return val;
  } else {
    uint64_t mask = (((uint64_t)1 << bitwidth()) - 1);
    return (val >> _l) & mask;  // little endian machine
  }
}

uint64_t SSDfgEdge::get_value() {
  assert(def());
  return extract_value(_value->get_val());  // the whole value
}

void SSDfgEdge::push_in_buffer(uint64_t v, bool valid, bool print, bool verif) {
  assert(_data_buffer.size() < buf_len && "Trying to push in full buffer\n");
  _data_buffer.push(std::make_pair(v, valid));
  compute_after_push(print, verif);
}

bool SSDfgEdge::is_buffer_full() { return (_data_buffer.size() == buf_len); }

bool SSDfgEdge::is_buffer_empty() { return _data_buffer.empty(); }

uint64_t SSDfgEdge::get_buffer_val() {
  assert(!_data_buffer.empty());
  uint64_t value = _data_buffer.front().first;
  value = extract_value(value);
  return value;
}

bool SSDfgEdge::get_buffer_valid() {
  assert(!_data_buffer.empty());
  return _data_buffer.front().second;
}

void SSDfgEdge::pop_buffer_val(bool print, bool verif) {
  assert(!_data_buffer.empty() && "Trying to pop from empty queue\n");
  // std::cout << "came here to pop buffer val\n";
  _data_buffer.pop();
  compute_after_pop(print, verif);
}

int is_subset_at_pos(SSDfgEdge* alt_edge, int pos) { return 0; }

int SSDfgEdge::bitwidth() { return _r - _l + 1; }

int SSDfgEdge::l() { return _l; }

int SSDfgEdge::r() { return _r; }

void SSDfgEdge::reset_associated_buffer() {
  decltype(_data_buffer) empty;
  std::swap(_data_buffer, empty);
}

/// }

/// { SSDfgOperand

SSDfgOperand::SSDfgOperand(SSDfgEdge* e) : edges{e}, fifos(1) {}

SSDfgOperand::SSDfgOperand(std::vector<SSDfgEdge*> es) : edges(es), fifos(es.size()) {}

SSDfgOperand::SSDfgOperand(uint64_t imm_) : imm(imm_) {}

SSDfgEdge* SSDfgOperand::get_first_edge() const {
  return edges.empty() ? nullptr : edges[0];
}

void SSDfgOperand::clear() {
  imm = 0;
  edges.clear();
}

bool SSDfgOperand::is_ctrl() {
  for (SSDfgEdge* e : edges) {
    if (e->etype() == SSDfgEdge::ctrl) {
      return true;
    }
  }
  return false;
}

bool SSDfgOperand::is_imm() { return edges.empty(); }

bool SSDfgOperand::is_composed() { return edges.size() > 1; }

// Functions which manipulate dynamic state
uint64_t SSDfgOperand::get_value() {  // used by simple simulator
  uint64_t base = imm;
  int cur_bit_pos = 0;
  for (SSDfgEdge* e : edges) {
    base |= e->get_value() << cur_bit_pos;
    cur_bit_pos += e->bitwidth();
  }
  assert(cur_bit_pos <= 64);  // max bitwidth is 64
  return base;
}

uint64_t SSDfgOperand::get_buffer_val() {  // used by backcgra simulator
  uint64_t base = imm;
  int cur_bit_pos = 0;
  for (SSDfgEdge* e : edges) {
    base |= e->get_buffer_val() << cur_bit_pos;
    cur_bit_pos += e->bitwidth();
  }
  assert(cur_bit_pos <= 64);  // max bitwidth is 64
  return base;
}

uint64_t SSDfgOperand::get_buffer_valid() {  // used by backcgra simulator
  for (SSDfgEdge* e : edges) {
    if (!e->get_buffer_valid()) {
      return false;
    }
  }
  return true;
}

uint64_t SSDfgOperand::is_buffer_empty() {  // used by backcgra simulator
  for (SSDfgEdge* e : edges) {
    if (e->is_buffer_empty()) {
      return true;
    }
  }
  return false;
}

void SSDfgOperand::pop_buffer_val(bool print, bool verif) {
  for (SSDfgEdge* e : edges) {
    e->pop_buffer_val(print, verif);
  }
}

bool SSDfgOperand::valid() {
  for (SSDfgEdge* e : edges) {
    SSDfgNode* n = e->def();
    if (n->invalid()) return false;
  }
  return true;
}

/// }

/// { SSDfgNode

void SSDfgNode::backprop_ctrl_dep() {
  if(!_needs_ctrl_dep) {
    _needs_ctrl_dep=true;
    for(auto* e : _inc_edge_list) {
      e->def()->backprop_ctrl_dep();
    }
  }
}

uint64_t SSDfgNode::invalid() { return _invalid; }

// Add edge to operand in least to most significant bit order
void SSDfgNode::addOperand(unsigned operand_slot, SSDfgEdge* e, int pos_within_op) {
  assert(operand_slot <= 1000);
  if (_ops.size() <= operand_slot) {
    _ops.resize(operand_slot + 1);
  }
  e->set_operand_slot(operand_slot);
  auto& edge_vec = _ops[operand_slot].edges;
  if (pos_within_op == -1) {
    edge_vec.push_back(e);
  } else {
    edge_vec.insert(edge_vec.begin() + pos_within_op, e);
  }

  _inc_edge_list.push_back(e);
}

SSDfg* SSDfgValue::ssdfg() { return _node->ssdfg(); }

void SSDfgValue::addOutEdge(SSDfgEdge* edge) {
  _uses.push_back(edge);
  _node->addOutEdge(edge);  // also add to host node
}

void SSDfgValue::slice(SSDfgEdge* e, int bitwidth) {
  if (e->bitwidth() == bitwidth) return;
  assert(e->bitwidth() > bitwidth);
  assert(bitwidth != 0);

  int orig_bitwidth = e->bitwidth();

  cout << "Slicing edge \"" << e->name() << "\" into " << bitwidth << " bits pieces\n";

  // Modify existing edge to be of smaller bitwidth
  e->set_r(e->l() + bitwidth - 1);

  // Need to find what position within the vector of edges to put the
  // newly created edges in the destination
  auto& edge_list_for_use_op = e->use()->ops()[e->operand_slot()].edges;
  unsigned ith_edge = 0;
  for (; ith_edge < edge_list_for_use_op.size(); ++ith_edge) {
    if (edge_list_for_use_op[ith_edge] == e) {
      break;
    }
  }

  // Add additional edges and update the operand structures
  for (int i = bitwidth; i < orig_bitwidth; i += bitwidth) {
    ssdfg()->connect(this, e->use(), e->operand_slot(), e->etype(), e->l() + i,
                     e->l() + i + bitwidth - 1, ith_edge + 1);
    ith_edge++;
  }
}

// Gauranteed here the e1->l() is < e2->l()
void SSDfgValue::slice_overlapping_edge(SSDfgEdge* e1, SSDfgEdge* e2) {
  std::vector<int> points;
  points.push_back(e1->l());
  points.push_back(e1->r() + 1);
  points.push_back(e2->l());
  points.push_back(e2->r() + 1);
  std::sort(points.begin(), points.end());
  int prev_p = points[0];
  int max_diff = 0;
  for (int p : points) {
    if (p == prev_p) continue;
    int diff = p - prev_p;
    if (max_diff < diff) max_diff = diff;
    prev_p = p;
  }

  slice(e1, max_diff);
  slice(e2, max_diff);
}

void SSDfgValue::slice_overlapping_edges() {
  // Just do n^2 algorithm for now
  bool change = true;
  while (change) {
    change = false;
    for (SSDfgEdge* e1 : _uses) {
      for (SSDfgEdge* e2 : _uses) {
        if (e1 == e2) continue;
        if (e1->l() == e2->l() && e1->r() == e2->r()) continue;
        if (e1->l() <= e2->r() && e1->r() >= e2->l()) {
          slice_overlapping_edge(e1, e2);
          change = true;
          break;
        }
      }
      if (change) break;
    }
  }
}

void SSDfgNode::addOutEdge(SSDfgEdge* edge) { _uses.push_back(edge); }

void SSDfgNode::validate() {
  for (size_t i = 0; i < _ops.size(); ++i) {
    SSDfgEdge* edge = _inc_edge_list[i];
    assert(edge == nullptr || edge->use() == this);
  }
  for (size_t i = 0; i < _uses.size(); ++i) {
    SSDfgEdge* edge = _uses[i];
    assert(edge->def() == this);
  }
}

void SSDfgNode::remove_edge(SSDfgNode* node, int idx) {
  for (unsigned i = 0; i < _ops.size(); ++i) {
    for (auto I = _ops[i].edges.begin(), E = _ops[i].edges.end(); I != E; ++i) {
      if ((*I)->get(idx) == node) {
        _ops[i].edges.erase(I);
        return;
      }
    }
  }
  assert(false && "edge was not found");
}

void SSDfgNode::removeIncEdge(SSDfgNode* orig) { remove_edge(orig, 0); }

void SSDfgNode::removeOutEdge(SSDfgNode* dest) { remove_edge(dest, 1); }

bool SSDfgNode::is_temporal() { return _ssdfg->group_prop(_group_id).is_temporal; }

void SSDfgNode::push_buf_dummy_node() {
  _ssdfg->push_buf_transient(this->first_inc_edge(), true, 1);  // can it be immediate?
}

// TODO: free all buffers and clear inputs ready
void SSDfgNode::reset_node() {
  _inputs_ready = 0;
  for (auto in_edges : _inc_edge_list) {
    (*in_edges).reset_associated_buffer();
  }
  for (auto out_edges : _uses) {
    (*out_edges).reset_associated_buffer();
  }
}

SSDfgNode::SSDfgNode(SSDfg* ssdfg, V_TYPE v)
    : _ssdfg(ssdfg), _ID(ssdfg->inc_node_id()), _vtype(v) {
  _group_id = _ssdfg->num_groups() - 1;
}

SSDfgNode::SSDfgNode(SSDfg* ssdfg, V_TYPE v, const std::string& name)
    : _ssdfg(ssdfg), _ID(ssdfg->inc_node_id()), _name(name), _vtype(v) {
  _group_id = _ssdfg->num_groups() - 1;
}

int SSDfgNode::inc_inputs_ready_backcgra(bool print, bool verif) {
  if (++_inputs_ready == num_inc_edges()) {
    _ssdfg->push_ready_node(this);
  }
  return 0;
}

SSDfgEdge* SSDfgNode::getLinkTowards(SSDfgNode* to) {
  auto pred = [to](SSDfgEdge* e) -> bool { return e->use() == to; };
  auto res = std::find_if(_uses.begin(), _uses.end(), pred);
  return (res == _uses.end()) ? nullptr : *res;
}

int SSDfgNode::maxThroughput() {
  if (_max_thr == 0) {
    for (auto elem : _uses) {
      _max_thr = std::max(_max_thr, elem->use()->maxThroughput());
    }
  }
  return _max_thr;
}

void SSDfgNode::depInsts(std::vector<SSDfgInst*>& insts) {
  for (auto it : _uses) {
    SSDfgNode* use = it->use();
    if (std::find(insts.begin(), insts.end(), use) != insts.end()) {
      use->depInsts(insts);
    }
  }
}

bool SSDfgNode::get_bp() {
  bool bp = false;
  for (auto elem : _uses)
    if (elem->is_buffer_full()) bp = true;
  return bp;
}

void SSDfgNode::set_node(SSDfgValue* dfg_val, uint64_t v, bool valid, bool avail,
                         bool print, bool verif) {

  // no need to do anything for output node
  if (this->num_out() == 0) return;
  if (avail) {
    if (!get_bp()) {
      for (auto* use : dfg_val->edges()) {
        use->push_in_buffer(v, valid, print, verif);
      }
    } else {
      set_value(dfg_val, v, valid, avail, 1, false);  // after 1 cycle
    }
  }
}

int SSDfgNode::inc_inputs_ready(bool print, bool verif) {
  if (++_inputs_ready == num_inc_edges()) {
    int num_computed = compute(print, verif);
    _inputs_ready = 0;
    return num_computed;
  }
  return 0;
}

/// }

/// { Parsing data structure

void CtrlBits::set(uint64_t val, Control b) {
  if(b == CtrlBits::B1 || b == CtrlBits::B2) {
    _needs_ctrl_dep=true;
  } 

  int loc = val * Total + b;
  assert(loc >= 0 && loc < 64);
  mask |= (1 << loc);
}

bool CtrlBits::test(uint64_t val, Control b) {
  int loc = val * Total + b;
  assert(loc >= 0 && loc < 64);
  return (mask >> loc) & 1;
}

void CtrlBits::test(uint64_t val, std::vector<bool>& back_array, bool& discard,
                    bool& predicate, bool& reset) {
  if (!mask) return;
  back_array[0] = back_array[0] || test(val, CtrlBits::B1);
  back_array[1] = back_array[1] || test(val, CtrlBits::B2);
  discard = discard || test(val, CtrlBits::Discard);
  predicate = predicate && !(test(val, CtrlBits::Abstain));
  reset = reset || test(val, CtrlBits::Reset);
}

CtrlBits::CtrlBits(const std::map<int, std::vector<std::string>>& raw) {
  for (auto& elem : raw)
    for (auto& s : elem.second) set(elem.first, str_to_enum(s));
}

void ControlEntry::set_flag(const std::string& s) {
  if (s == "pred") {
    flag = SSDfgEdge::ctrl_true;
  } else if (s == "inv_pred") {
    flag = SSDfgEdge::ctrl_false;
  } else if (s == "control" || s == "self") {
    flag = SSDfgEdge::ctrl;
  } else {
    printf("qualifier: %s unknown", s.c_str());
    assert(0 && "Invalid argument qualifier");
  }
}

/// }

/// { SSDfgInst

int SSDfgInst::maxThroughput() {
  if (_max_thr == 0) {
    _max_thr = inst_thr(inst());
    for (auto it = _uses.begin(); it != _uses.end(); it++) {
      _max_thr = std::max(_max_thr, (*it)->use()->maxThroughput());
    }
  }
  return _max_thr;
}

void SSDfgInst::depInsts(std::vector<SSDfgInst*>& insts) {
  insts.push_back(this);
  for (auto it = _uses.begin(); it != _uses.end(); it++) {
    SSDfgNode* use = (*it)->use();
    if (std::find(insts.begin(), insts.end(), use) != insts.end()) {
      use->depInsts(insts);
    }
  }
}

std::string SSDfgInst::name() {
  std::stringstream ss;
  ss << _name << "(" << SS_CONFIG::name_of_inst(_ssinst) << " " << id() << ")";
  return ss.str();
}

/// }

/// { EntryTable

void EntryTable::set(const std::string& s, ParseResult* pr, bool override) {
  if (!symbol_table_.count(s))
    symbol_table_[s] = pr;
  else if (override)
    symbol_table_[s] = pr;
  else {
    std::cerr << "duplicated symbol: " << s << std::endl;
    assert(0 && "Add existing symbol entrying w/o overriding");
  }
}

ParseResult* EntryTable::get_sym(const std::string& s) {
  if (!symbol_table_.count(s)) {
    cout << "Error: Could not find Symbol \"" << s << "\"\n";
  }
  assert(symbol_table_.count(s));
  return symbol_table_[s];
}

/// }

/// { SSDfg

std::vector<SSDfgNode*>& SSDfg::ordered_nodes() {
  if (_orderedNodes.size() == 0) {
    std::set<SSDfgNode*> done_nodes;
    for (SSDfgVecOutput* out : _vecOutputs) {
      order_nodes(out, done_nodes, _orderedNodes);
    }
  }
  return _orderedNodes;
}

void SSDfg::order_nodes(SSDfgNode* node,
                        std::set<SSDfgNode*>& done_nodes,  // done insts
                        std::vector<SSDfgNode*>& ordered_insts) {
  if (done_nodes.count(node)) {
    return;
  }

  // insert the new inst
  done_nodes.insert(node);

  // incoming edges to a node
  for (auto edge : node->in_edges()) {
    assert(edge);
    order_nodes(edge->def(), done_nodes, ordered_insts);
  }

  ordered_insts.push_back(node);
}

void SSDfg::printGraphviz(const char* fname, Schedule* sched) {
  std::ofstream os(fname);
  assert(os.good());
  printGraphviz(os, sched);
  os.flush();
}

void SSDfg::check_for_errors() {
  //printGraphviz("viz/error_check_dfg.dot");

  bool error = false;
  for (auto elem : _vecInputs) {
    if (elem->num_out() == 0) {
      cerr << "Error: No uses on input " << elem->name() << "\n";
      error = true;
    }
  }

  for (auto elem : _insts) {
    if (elem->num_out() == 0) {
      cerr << "Error: No uses on inst " << elem->name() << "\n";
      error = true;
    }
    if (elem->num_inc() == 0) {
      cerr << "Error: No operands on inst " << elem->name() << "\n";
      error = true;
    }
  }

  for (auto elem : _vecOutputs) {
    if (elem->num_inc() == 0) {
      cerr << "Error: No operands on output " << elem->name() << "\n";
      error = true;
    }
  }

  assert(!error && "ERROR: BAD DFG");
}

// This function is called from the simulator to
int SSDfg::compute(bool print, bool verif, int g) {
  int num_computed = 0;

  assert(g < (int)_vecInputGroups.size());
  for (unsigned i = 0; i < _vecInputGroups[g].size(); ++i) {
    SSDfgVecInput* vec = _vecInputGroups[g][i];
    for (auto elem : vec->uses()) {
      num_computed += elem->use()->compute(print, verif);  // producer->consumer
    }
  }
  return num_computed;
}

// Calculates max group throughput based on functional unit type
int SSDfg::maxGroupThroughput(int g) {
  int maxgt = 0;

  assert(g < (int)_vecInputGroups.size());
  for (unsigned i = 0; i < _vecInputGroups[g].size(); ++i) {
    SSDfgVecInput* vec = _vecInputGroups[g][i];
    for (auto elem : vec->uses()) {
      maxgt = std::max(maxgt, elem->use()->maxThroughput());
    }
  }
  return maxgt;
}

void SSDfg::instsForGroup(int g, std::vector<SSDfgInst*>& insts) {
  assert(g < (int)_vecInputGroups.size());
  for (unsigned i = 0; i < _vecInputGroups[g].size(); ++i) {
    SSDfgVecInput* vec = _vecInputGroups[g][i];
    for (auto elem : vec->uses()) {
      elem->use()->depInsts(insts);
    }
  }
}

// Necessary for BOOST::SERIALIZATION
SSDfg::SSDfg() {}

// COMMA IF NOT FIRST
void CINF(std::ostream& os, bool& first) {
  if (first) {
    first = false;
  } else {
    os << ", ";
  }
}

bool conv_to_int(std::string s, uint64_t& ival) {
  try {
    ival = (uint64_t)stol(s, 0, 0);
    return true;
  } catch (...) {
  }
  return false;
}

bool conv_to_double(std::string s, double& dval) {
  try {
    dval = stod(s);
    return true;
  } catch (...) {
  }
  return false;
}

ParseResult* SSDfg::create_inst(std::string opcode, std::vector<ParseResult*>& args) {
  SS_CONFIG::ss_inst_t inst = inst_from_string(opcode.c_str());
  auto* dfg_inst = new SSDfgInst(this, inst);

  for (unsigned i = 0; i < args.size(); ++i) {
    if (auto data = dynamic_cast<ConstDataEntry*>(args[i])) {
      dfg_inst->setImm(data->data);
      dfg_inst->setImmSlot(i);
    } else if (auto ne = dynamic_cast<ValueEntry*>(args[i])) {
      connect(ne->value, dfg_inst, i, SSDfgEdge::data, ne->l, ne->r);
    } else if (auto ce = dynamic_cast<ConvergeEntry*>(args[i])) {
      for (auto elem : ce->entries) {
        if (auto ne = dynamic_cast<ValueEntry*>(elem))
          connect(ne->value, dfg_inst, i, SSDfgEdge::data, ne->l, ne->r);
      }
    } else if (auto ce = dynamic_cast<ControlEntry*>(args[i])) {
      // External control
      if (ce->controller) {
        auto ne = dynamic_cast<ValueEntry*>(ce->controller);
        connect(ne->value, dfg_inst, i, ce->flag, ne->l, ne->r);
        dfg_inst->set_ctrl_bits(ce->bits);
      } else {
        // Self control
        dfg_inst->set_self_ctrl(ce->bits);
      }
    } else {
      assert(false && "Invalide Node type");
    }
  }

  ParseResult* res =
      new ValueEntry(dfg_inst->values()[0], 0, SS_CONFIG::bitwidth[inst] - 1);
  add<SSDfgInst>(dfg_inst);
  return res;
}

void SSDfg::set_pragma(const std::string &c, const std::string &s) {
  if (c == string("dfg")) {
    cout << "No pragmas yet for dfg\n";
  } else if (c == string("group")) {
    if (s == "temporal") {
      assert(!_groupProps.empty());
      _groupProps[_groupProps.size() - 1].is_temporal = true;
    }
  } else if (c == "frequency" || c == "unroll") {
    std::istringstream iss(s);
    auto &ref = c == "frequency" ? _groupProps.back().frequency : _groupProps.back().unroll;
    iss >> ref;
  } else {
    cout << "Context \"" << c << "\" not recognized.";
  }
}

void SSDfg::start_new_dfg_group() {
  _vecInputGroups.emplace_back(std::vector<SSDfgVecInput*>());
  _vecOutputGroups.emplace_back(std::vector<SSDfgVecOutput*>());
  _groupProps.emplace_back(GroupProp());
}

SSDfg::SSDfg(string filename_) : filename(filename_) {
  string line;
  start_new_dfg_group();
  parse_dfg(filename_.c_str(), this);
  preprocess_graph();
  calc_minLats();
  check_for_errors();
}

std::string SSDfgInst::gamsName() {
  std::stringstream ss;
  ss << "FV" << _ID;
  return ss.str();
}
void SSDfgInst::setImmSlot(int i) {
  assert(i < 4);

  if ((int)_ops.size() <= i) {
    _ops.resize(i + 1);
  }

  _imm_slot = i;
}

template<typename Tfrom, typename Tto>
std::vector<Tto> cast_vector(const vector<Tfrom> &a) {
  std::vector<Tto> res(a.size());
  for (size_t i = 0; i < a.size(); ++i)
    res[i] = a[i];
  return res;
}

uint64_t SSDfgInst::do_compute(bool &discard) {
  last_execution = _ssdfg->cur_cycle();

#define EXECUTE(bw)                                                                            \
  case bw: {                                                                                   \
    std::vector<uint##bw##_t> input = cast_vector<uint64_t, uint##bw##_t>(_input_vals);        \
    std::vector<uint##bw##_t> outputs(values().size());                                        \
      output = SS_CONFIG::execute##bw(_ssinst, input,                                          \
                                      outputs, (uint##bw##_t*)&_reg[0], discard, _back_array); \
      outputs[0] = output;                                                                     \
      _output_vals = cast_vector<uint##bw##_t, uint64_t>(outputs);                             \
      return output;                                                                           \
  }

  uint64_t output;
  switch (bitwidth()) {
    EXECUTE(64)
    EXECUTE(32)
    EXECUTE(16)
    EXECUTE(8)
    default:
      cout << "Weird bitwidth: " << bitwidth() << "\n";
      assert(0 && "weird bitwidth");
  }

#undef EXECUTE

}

// compute:actual compute called from SSDfg class (slightly modify this)
int SSDfgInst::compute(bool print, bool verif) {
  assert(_ops.size() <= 3);

  if (_input_vals.size() == 0) {
    resize_vals(_ops.size());
  }
  assert(_input_vals.size() <= _ops.size());

  if (print) {
    _ssdfg->dbg_stream() << name() << " (" << _ID << "): ";
  }

  _invalid = false;

  for (unsigned i = 0; i < _ops.size(); ++i) {
    if (immSlot() == (int)i) {
      _input_vals[i] = imm();
    } else {
      _input_vals[i] = _ops[i].get_value();
      if (!_ops[i].valid()) {
        _invalid = true;
      }
    }
    if (print) {
      _ssdfg->dbg_stream() << std::hex << _input_vals[i] << " ";
    }
  }

  do_compute(_invalid);
  for (int i = 0; i < (int)_values.size(); ++i) {
    _values[i]->set_val(getout(i));
  }

  if (print) {
    print_output(_ssdfg->dbg_stream());
  }

  if (verif) {
    // if (!_verif_stream.is_open()) {
    //  checked_system("mkdir -p verif");
    //  _verif_stream.open(("verif/fu" + _verif_id + ".txt").c_str());
    //  assert(_verif_stream.is_open());
    //}
    //_verif_stream << hex << setw(16) << setfill('0') << _val << "\n";
    //_verif_stream.flush();
  }

  int num_computed = !_invalid;

  for (auto iter = _uses.begin(); iter != _uses.end(); iter++) {
    SSDfgNode* use = (*iter)->use();
    num_computed += use->inc_inputs_ready(print, verif);  // recursively call compute
  }

  return num_computed;
}

void SSDfgInst::resize_vals(int n) {
  _input_vals.resize(n);
}

void SSDfgInst::print_output(std::ostream& os) {
  os << " = ";
  for (int i = 0; i < (int)_values.size(); ++i) {
      os << _output_vals[i];
    os << " ";
  }
}

// new compute for back cgra-----------------------------
int SSDfgInst::compute_backcgra(bool print, bool verif) {
  assert(_ops.size() <= 3);

  if (_input_vals.size() == 0) {
    resize_vals(_ops.size());
  }

  // initializing back pressure
  _back_array.clear();
  _back_array.resize(_ops.size(), 0);

  if (print) {
    _ssdfg->dbg_stream() << name() << " (" << _ID << "): ";
  }

  bool discard(false), reset(false), pred(true);
  uint64_t output = 0;

  std::ostringstream reason;
  _invalid = false;

  for (unsigned i = 0; i < _ops.size(); ++i) {
    if (_ops[i].is_imm()) {
      _input_vals[i] = imm();
    } else if (_ops[i].is_ctrl()) {
      int c_val = _ops[i].get_buffer_val();
      // FIXME: confirm that this is correct
      _input_vals[i] = c_val;
      if (!_ops[i].get_buffer_valid()) {
        _invalid = true;
        reason << "ctrl signal not ready!";
      } else {
        _ctrl_bits.test(c_val, _back_array, discard, pred, reset);
      }
    } else {
      _input_vals[i] = _ops[i].get_buffer_val();
      if (!_ops[i].get_buffer_valid()) {
        _invalid = true;
        reason << "operand " << i << " not valid!";
      }
    }
  }

  // we set this instruction to invalid
  // pred = 1; // for now--check why is it here?
  if (!pred) {
    _invalid = true;
  }

  if (!_invalid) {  // IF VALID
    if (print) {
      for (size_t i = 0; i < _ops.size(); ++i)
        _ssdfg->dbg_stream() << std::hex << _input_vals[i] << " ";
    }

    _ssdfg->inc_total_dyn_insts();

    // Read in some temp value and set _val after inst_lat cycles
    output = do_compute(discard);
    _self_bits.test(output, _back_array, discard, pred, reset);

    if (print) {
      for (size_t i = 0; i < _output_vals.size(); ++i) {
        std::cout << _output_vals[i] << " ";
      }
      std::cout << "\n";
    }
  } else if (print) {
    std::cout << "\n";
  }

  // TODO/FIXME: change to all registers
  if (reset) {
    std::cout << "Reset the register file! " << _reg[0] << "\n";
    _reg[0] = 0;
  }

  if (print) {
    for (size_t i = 0; i < _back_array.size(); ++i)
      if (_back_array[i]) std::cout << "backpressure on " << i << " input";
  }

  if (this->name() == ":Phi") {
    _ssdfg->inc_total_dyn_insts();
    assert(_input_vals.size() == 3 && "Not enough input in phi node");
    for (unsigned i = 0; i < _ops.size(); ++i) {
      if (_ops[i].get_buffer_valid()) {
        output = _ops[i].get_buffer_val();
      }
    }

    if (print) {
      _ssdfg->dbg_stream() << " = " << output << "\n";
    }
    discard = false;
    _invalid = false;
  }

  _inputs_ready = 0;

  if (print) {
    std::cout << (_invalid ? " invalid " : " valid ") << reason.str() << " "
              << (discard ? " and output discard!\n" : "\n");
  }

  _invalid |= discard;
  for (unsigned i = 0; i < values().size(); ++i) {
    set_value(_values[i], getout(i), !_invalid, true, inst_lat(inst()), /*is compute=*/true);
  }

  // pop the inputs after inst_thr+back_press here
  int inst_throughput = inst_thr(inst());

  for (unsigned i = 0; i < _ops.size(); ++i) {
    if (_ops[i].is_imm()) {
      continue;
    }
    if (_back_array[i]) {
      _inputs_ready++;
      //_inputs_ready += _ops[i].edges.size();
    } else {
      // TODO:FIXME:CHECK:IMPORTANT
      // Iterate over edges in an operand and push transients for all of them?
      for (SSDfgEdge* e : _ops[i].edges) {
        _ssdfg->push_buf_transient(e, false, inst_throughput);
      }
    }
  }

  if (verif) {
    // if (!_verif_stream.is_open()) {
    //  checked_system("mkdir -p verif");
    //  _verif_stream.open(("verif/fu" + _verif_id + ".txt").c_str());
    //  assert(_verif_stream.is_open());
    //}
    //_verif_stream << hex << setw(16) << setfill('0') << _val << "\n";
    //_verif_stream.flush();
  }

  return 1;
}

// Virtual function-------------------------------------------------

int SSDfgInst::update_next_nodes(bool print, bool verif) { return 0; }

SSDfgVec::SSDfgVec(V_TYPE v, int num_values, int bitwidth, const std::string& name,
                   int id, SSDfg* ssdfg, const ssdfg::MetaPort &meta_)
    : SSDfgNode(ssdfg, v, name), _bitwidth(bitwidth), meta(meta_, this) {
  _values.push_back(new SSDfgValue(this, 0, bitwidth));
  for (int i = 1; i < num_values; ++i) {
    _values.push_back(new SSDfgValue(this, i, bitwidth));
  }
  _group_id = ssdfg->num_groups() - 1;
}

void SSDfgNode::set_value(SSDfgValue* dfg_val, uint64_t v, bool valid, bool avail,
                          int cycle, bool is_compute) {
  _ssdfg->push_transient(dfg_val, v, valid, avail, cycle, is_compute);
}

void SSDfgNode::set_value(int i, uint64_t v, bool valid, bool avail, int cycle, bool is_compute) {
  set_value(_values[i], v, valid, avail, cycle, is_compute);
}

//------------------------------------------------------------------

void SSDfgNode::printGraphviz(ostream& os, Schedule* sched) {
  string ncolor = "black";
  os << "N" << _ID << " [ label = \"" << name();

  if (is_temporal()) {
    os << ":TMP";
  }

  if (_needs_ctrl_dep) {
    os << ":C";
  } 

  if (sched) {
    os << "\\n lat=" << sched->latOf(this) << " ";
  }

  // os << "min:" << _min_lat;

  if (sched) {
    auto p = sched->lat_bounds(this);
    os << "\\n bounds=" << p.first << " to " << p.second;
    os << "\\n vio=" << sched->vioOf(this);
  }

  os << "\", color= \"" << ncolor << "\"]; ";

  os << "\n";

  // print edges
  for (auto v : _values) {
    for (auto e : v->edges()) {
      if (e->etype() == SSDfgEdge::data) {
        ncolor = "black";
      } else if (e->etype() == SSDfgEdge::ctrl_true) {
        ncolor = "blue";
      } else if (e->etype() == SSDfgEdge::ctrl_false) {
        ncolor = "red";
      }

      SSDfgNode* n = e->use();
      os << "N" << _ID << " -> N" << n->_ID << "[ color=";
      os << ncolor;
      os << " label = \"";
      if (v->index() != 0) {
        os << "v" << v->index() << " ";
      }
      if (sched) {
        os << "l:" << sched->link_count(e) << "\\nex:" << sched->edge_delay(e)
           << "\\npt:" << sched->num_passthroughs(e);
      }
      os << e->l() << ":" << e->r();
      os << "\"];\n";
    }
  }

  os << "\n";
}

// Connect two nodes in DFG
// assumption is that each operand's edges are
// added to in least to most significant order!
SSDfgEdge* SSDfg::connect(SSDfgValue* orig, SSDfgNode* dest, int dest_slot,
                          SSDfgEdge::EdgeType etype, int l, int r, int operand_pos) {
  SSDfgEdge* new_edge = new SSDfgEdge(orig, dest, etype, this, l, r);
  dest->addOperand(dest_slot, new_edge, operand_pos);
  orig->addOutEdge(new_edge);  // this also adds to the node
  _edges.push_back(new_edge);

  return new_edge;
}

// Disconnect two nodes in DFG
void SSDfg::disconnect(SSDfgNode* orig, SSDfgNode* dest) {
  dest->removeIncEdge(orig);
  orig->removeOutEdge(dest);
  assert(false && "edge was not found");
}

void SSDfg::reset_simulation_state() {
  for (auto& list : transient_values) {
    list.clear();
  }
  for (auto& list : buf_transient_values) {
    list.clear();
  }
  _complex_fu_free_cycle.clear();
  _ready_nodes.clear();
  for (auto in : _nodes) {
    (*in).reset_node();
  }
}

void SSDfg::printGraphviz(ostream& os, Schedule* sched) {
  os << "Digraph G { \nnewrank=true;\n ";

  // Insts
  for (auto node : _nodes) {
    node->printGraphviz(os, sched);
  }

  os << "\t{ rank = same; ";
  for (auto in : _vecInputs) {
    os << "N" << in->id() << " ";
  }
  os << "}\n";

  os << "\t{ rank = same; ";
  for (auto out : _vecOutputs) {
    os << "N" << out->id() << " ";
  }
  os << "}\n";

  os << "}\n";
}

// After parsing, preprocess graph to establish reuqired invariants
void SSDfg::preprocess_graph() {
  for (auto node : _nodes) {
    for (auto val : node->values()) {
      val->slice_overlapping_edges();
    }
  }
}

void SSDfg::calc_minLats() {
  list<SSDfgNode*> openset;
  set<bool> seen;
  for (auto elem : _vecInputs) {
    openset.push_back(elem);
    seen.insert(elem);
  }

  // populate the schedule object
  while (!openset.empty()) {
    SSDfgNode* n = openset.front();
    openset.pop_front();

    int cur_lat = 0;

    for (auto elem : n->in_edges()) {
      SSDfgNode* dn = elem->def();
      if (dn->min_lat() > cur_lat) {
        cur_lat = dn->min_lat();
      }
    }

    if (SSDfgInst* inst_n = dynamic_cast<SSDfgInst*>(n)) {
      cur_lat += inst_lat(inst_n->inst()) + 1;
    } else if (dynamic_cast<SSDfgVecInput*>(n)) {
      cur_lat = 0;
    } else if (dynamic_cast<SSDfgVecOutput*>(n)) {
      cur_lat += 1;
    }

    n->set_min_lat(cur_lat);

    for (auto elem : n->uses()) {
      SSDfgNode* un = elem->use();

      bool ready = true;
      for (auto elem : un->in_edges()) {
        SSDfgNode* dn = elem->def();
        if (!seen.count(dn)) {
          ready = false;
          break;
        }
      }
      if (ready) {
        seen.insert(un);
        openset.push_back(un);
      }
    }
  }
}

// Gams related
void SSDfg::printGams(std::ostream& os, std::unordered_map<string, SSDfgNode*>& node_map,
                      std::unordered_map<std::string, SSDfgEdge*>& edge_map,
                      std::unordered_map<std::string, SSDfgVec*>& port_map) {
  os << "$onempty\n";

  {
    bool is_first = true;
    os << "set v \"verticies\" \n /";  // Print the set of Nodes:
    for (auto elem : _nodes) {
      if (!is_first) os << ", ";
      os << elem->gamsName();
      assert(elem);
      node_map[elem->gamsName()] = elem;
      is_first = false;
    }
    os << "/;\n";
  }

  {
    bool is_first = true;
    os << "set inV(v) \"input verticies\" /";  // Print the set of Nodes:
    for (auto elem : _vecInputs) {
      if (!is_first) os << ", ";
      assert(elem);
      os << elem->gamsName();
      is_first = false;
    }
    os << "/;\n";
  }

  {
    bool is_first = true;
    os << "set outV(v) \"output verticies\" /";  // Print the set of Nodes:
    for (auto elem : _vecOutputs) {
      if (!is_first) os << ", ";
      os << elem->gamsName();
      assert(elem);
      is_first = false;
    }
    os << "/;\n";
  }

  {
    os << "parameter minT(v) \"Minimum Vertex Times\" \n /";
    for (auto Ii = _nodes.begin(), Ei = _nodes.end(); Ii != Ei; ++Ii) {
      if (Ii != _nodes.begin()) os << ", ";
      SSDfgNode* n = *Ii;
      int l = n->min_lat();
      if (SSDfgInst* inst = dynamic_cast<SSDfgInst*>(n)) {
        l -= inst_lat(inst->inst());
      }
      os << n->gamsName() << " " << l;
    }
    os << "/;\n";
  }

  os << "set iv(v) \"instruction verticies\";\n";
  os << "iv(v) = (not inV(v)) and (not outV(v));\n";

  for (int i = 2; i < SS_NUM_TYPES; ++i) {
    ss_inst_t ss_inst = (ss_inst_t)i;

    os << "set " << name_of_inst(ss_inst) << "V(v) /";
    bool first = true;

    for (auto dfg_inst : _insts) {
      if (ss_inst == dfg_inst->inst()) {
        CINF(os, first);
        os << dfg_inst->gamsName();
      }
    }
    os << "/;\n";
  }

  bool first = true;
  os << "set pv(*) \"Port Vectors\" \n /";  // Print the set of port vertices:
  for (auto& i : _vecInputs) {
    CINF(os, first);
    os << i->gamsName() << " ";
    port_map[i->gamsName()] = i;
  }
  for (auto& i : _vecOutputs) {
    CINF(os, first);
    os << i->gamsName() << " ";
    port_map[i->gamsName()] = i;
  }
  os << "/;\n";

  // TODO: Gams ports need tobe removed
  /*
  first=true;
  os << "parameter VI(pv,v) \"Port Vector Definitions\" \n /";   // Print the set of port
  vertices mappings: for(auto& i : _vecInputs) { int ind=0; for (auto ssin : i->vector())
  { CINF(os,first); os << i->gamsName() << "." << ssin->gamsName() << " " << ind+1;
    }
  }
  for(auto& i : _vecOutputs) {
    int ind=0;
    for (auto ssout : i->vector()) {
      CINF(os,first);
      os << i->gamsName() << "." << ssout->gamsName() << " " << ind+1;
    }
  }
  os << "/;\n";
*/
  // -------------------edges ----------------------------
  os << "set e \"edges\" \n /";  // Print the set of edges:

  for (auto Ie = _edges.begin(), Ee = _edges.end(); Ie != Ee; ++Ie) {
    if (Ie != _edges.begin()) os << ", ";
    os << (*Ie)->gamsName();
    edge_map[(*Ie)->gamsName()] = *Ie;
  }
  os << "/;\n";

  // create the kindC Set
  os << "set kindV(K,v) \"Vertex Type\"; \n";

  // --------------------------- Enable the Sets ------------------------
  os << "kindV('Input', inV(v))=YES;\n";
  os << "kindV('Output', outV(v))=YES;\n";

  for (int i = 2; i < SS_NUM_TYPES; ++i) {
    ss_inst_t ss_inst = (ss_inst_t)i;
    os << "kindV(\'" << name_of_inst(ss_inst) << "\', " << name_of_inst(ss_inst)
       << "V(v))=YES;\n";
  }

  // --------------------------- Print the linkage ------------------------
  os << "parameter Gve(v,e) \"vetex to edge\" \n /";  // def edges
  for (auto Ie = _edges.begin(), Ee = _edges.end(); Ie != Ee; ++Ie) {
    if (Ie != _edges.begin()) os << ", ";

    SSDfgEdge* edge = *Ie;
    os << edge->def()->gamsName() << "." << edge->gamsName() << " 1";
  }
  os << "/;\n";

  os << "parameter Gev(e,v) \"edge to vertex\" \n /";  // use edges
  for (auto Ie = _edges.begin(), Ee = _edges.end(); Ie != Ee; ++Ie) {
    if (Ie != _edges.begin()) os << ", ";

    SSDfgEdge* edge = *Ie;
    os << edge->gamsName() << "." << edge->use()->gamsName() << " 1";
  }
  os << "/;\n";

  os << "set intedges(e) \"edges\" \n /";  // Internal Edges
  first = true;
  for (auto Ie = _edges.begin(), Ee = _edges.end(); Ie != Ee; ++Ie) {
    SSDfgEdge* edge = *Ie;

    if (!dynamic_cast<SSDfgVecInput*>(edge->def()) &&
        !dynamic_cast<SSDfgVecOutput*>(edge->use())) {
      if (first)
        first = false;
      else
        os << ", ";
      os << edge->gamsName();
    }
  }
  os << "/;\n";

  os << "parameter delta(e) \"delay of edge\" \n /";  // Print the set of edges:
  for (auto Ie = _edges.begin(), Ee = _edges.end(); Ie != Ee; ++Ie) {
    if (Ie != _edges.begin()) os << ", ";
    SSDfgEdge* edge = *Ie;

    if (SSDfgInst* dfginst = dynamic_cast<SSDfgInst*>(edge->def())) {
      os << (*Ie)->gamsName() << " " << inst_lat(dfginst->inst());
    } else {
      os << (*Ie)->gamsName() << " "
         << "0";  // TODO: WHAT LATENCY SHOULD I USE??
    }
  }
  os << "/;\n";
}

void SSDfg::addVecOutput(const std::string& name, int len, EntryTable& syms, int width,
                         const ssdfg::MetaPort &meta) {
  add_parsed_vec<SSDfgVecOutput>(name, len, syms, width, meta);
}

void SSDfg::addVecInput(const std::string& name, int len, EntryTable& syms, int width,
                        const ssdfg::MetaPort &meta) {
  add_parsed_vec<SSDfgVecInput>(name, len, syms, width, meta);
}

double SSDfg::count_starving_nodes() {
  double count = 0;
  double num_unique_dfg_nodes = 0;
  for (auto it : _vecInputs) {
    SSDfgVecInput* vec_in = it;
    for (auto edge : vec_in->uses()) {  // each scalar node
      // for(auto node : elem->uses()) {
      SSDfgNode* node = edge->use();  // FIXME: how does it work for dgra
      num_unique_dfg_nodes += 1 / node->num_inc();
      if (node->num_inputs_ready()) {
        count += 1 / node->num_inc();
      }
      // }
    }
  }
  assert(num_unique_dfg_nodes >= 0);
  if (num_unique_dfg_nodes == 0) return 0;  // What is this case? none nodes
  return count / num_unique_dfg_nodes;
}

int SSDfg::cycle(bool print, bool verif) {

  //FIXME(@were): Why 2?
  for (auto it = buf_transient_values[cur_buf_ptr].begin();
       it != buf_transient_values[cur_buf_ptr].end();) {
    // set the values
    buffer_pop_info* temp = *it;
    SSDfgEdge* e = temp->e;
    e->pop_buffer_val(print, verif);

    it = buf_transient_values[cur_buf_ptr].erase(it);
  }

  for (auto it = transient_values[cur_node_ptr].begin();
       it != transient_values[cur_node_ptr].end();) {
    struct cycle_result* temp = *it;
    SSDfgNode* ss_node = temp->dfg_val->node();
    ss_node->set_node(temp->dfg_val, temp->val, temp->valid, temp->avail, print, verif);
    it = transient_values[cur_buf_ptr].erase(it);
  }

  /// FIXME(@were): I think this can be simplified; I think this local variable can be
  /// deleted
  std::unordered_set<int> nodes_complete;

  for (auto I = _ready_nodes.begin(); I != _ready_nodes.end();) {
    SSDfgNode* n = *I;

    int node_id = n->node_id();
    bool should_fire = (node_id == -1) || (nodes_complete.count(node_id) == 0);

    unsigned inst_throughput = 1;

    // If inst_throughput cycles is great than 1, lets mark the throughput
    // make sure nobody else can also schedule a complex instruction during
    // that period
    if (should_fire && node_id != -1) {
      if (SSDfgInst* inst = dynamic_cast<SSDfgInst*>(n)) {
        inst_throughput = inst_thr(inst->inst());
        if (inst_throughput > 1) {
          if (_complex_fu_free_cycle[node_id] > _cur_cycle) {
            should_fire = false;
          }
        }
      }
    }

    if (should_fire) {
      n->compute_backcgra(print, verif);
      I = _ready_nodes.erase(I);
      nodes_complete.insert(node_id);

      if (node_id != -1 && inst_throughput > 1) {
        _complex_fu_free_cycle[node_id] = _cur_cycle + inst_throughput;
      }

    } else {
      ++I;
    }
  }

  cur_buf_ptr = (cur_buf_ptr + 1) % get_max_lat();
  cur_node_ptr = (cur_node_ptr + 1) % get_max_lat();
  _cur_cycle = _cur_cycle + 1;
  int temp = _total_dyn_insts;
  _total_dyn_insts = 0;
  return temp;
}
/// }

using SS_CONFIG::SubModel;

std::vector<std::pair<int, ssnode*>> SSDfgInst::candidates(Schedule* sched,
                                                           SSModel* ssmodel, int n) {
  SubModel* model = ssmodel->subModel();
  std::vector<std::pair<int, ssnode*>> spots;
  std::vector<std::pair<int, ssnode*>> not_chosen_spots;

  std::vector<ssfu*>& fus = model->nodes<SS_CONFIG::ssfu*>();
  // For Dedicated-required Instructions
  for (size_t i = 0; i < fus.size(); ++i) {
    ssfu* cand_fu = fus[i];

    if ((cand_fu->fu_def() != nullptr && !cand_fu->fu_def()->is_cap(this->inst())) ||
        (cand_fu->num_non_self_out_links() < (int)this->values().size())) {
      continue;
    }

    if (!is_temporal()) {
      if (sched->isPassthrough(0, cand_fu))  // FIXME -- this can't be right
        continue;
      // Normal Dedidated Instructions

      if (cand_fu->is_shared() && !spots.empty()) {
        continue;
      }

      for (int k = 0; k < 8; k += this->bitwidth() / 8) {
        int cnt=0;
        for (int sub_slot = k; sub_slot < k + this->bitwidth() / 8; ++sub_slot) {
          cnt += sched->dfg_nodes_of(sub_slot, cand_fu).size();
        }
        cnt=cnt/8+1;
         
        if (rand() % (cnt * cnt) == 0) {
          spots.emplace_back(k, fus[i]);
        } else {
          not_chosen_spots.emplace_back(k, fus[i]);
        }
      }

    } else if (cand_fu->is_shared()) {
      // For temporaly-shared instructions
      // For now the approach is to *not* consume dedicated resources, although
      // this can be changed later if that's helpful.
      if ((int)sched->dfg_nodes_of(0, cand_fu).size() + 1 < cand_fu->max_util()) {
        spots.emplace_back(0, fus[i]);
      } else {
        not_chosen_spots.emplace_back(0,fus[i]);
      }
    }
  }

  // If we couldn't find any good spots, we can just pick a bad spot for now
  if(spots.size() == 0) {
    spots=not_chosen_spots;
  }

  std::random_shuffle(spots.begin(), spots.end());

  if (n > (int)spots.size() || n == 0) n = spots.size();

  return std::vector<std::pair<int, ssnode*>>(spots.begin(), spots.begin() + n);
}

// TODO: Marge these two functions.
std::vector<std::pair<int, ssnode*>> SSDfgVecInput::candidates(Schedule* sched,
                                                               SSModel* model, int n) {
  auto& vports = model->subModel()->input_list();
  // Lets write size in units of bits
  std::vector<std::pair<int, ssnode*>> spots;
  for (size_t i = 0; i < vports.size(); ++i) {
    auto cand = vports[i];
    if ((int)cand->input_bitwidth() >= phys_bitwidth()) {
      spots.push_back(make_pair(0, cand));
    }
  }
  return spots;
}

std::vector<std::pair<int, ssnode*>> SSDfgVecOutput::candidates(Schedule* sched,
                                                                SSModel* model, int n) {
  auto& vports = model->subModel()->output_list();
  // Lets write size in units of bits
  std::vector<std::pair<int, ssnode*>> spots;
  for (size_t i = 0; i < vports.size(); ++i) {
    auto cand = vports[i];
    if ((int)cand->output_bitwidth() >= phys_bitwidth()) {
      spots.push_back(make_pair(0, cand));
    }
  }

  //if(spots.size() == 0) {
  //  cout << "No Spots Left\n";
  //  cout << "Phys Bitwidth: " << phys_bitwidth() << "\n";

  //  for (size_t i = 0; i < vports.size(); ++i) {
  //    auto cand = vports[i];
  //    cout << cand->name() << "output bitwidth: " << cand->output_bitwidth() << "\n";
  //  }
  //}

  return spots;
}

void SSDfgValue::push(uint64_t val, bool valid, int delay) {
  // TODO: Support FIFO length backpressure.
  fifo.push(simulation::Data(_node->_ssdfg->cur_cycle() + delay, val, valid));
}

// new compute for back cgra-----------------------------
void SSDfgInst::forward() {

  static bool print = getenv("COMP") != nullptr;

  // Check the instruction throughput
  int inst_throughput = inst_thr(inst());
  if (_ssdfg->cur_cycle() - last_execution < inst_throughput) {
    return;
  }

  // Check the avaiability of output buffer
  if (!values()[0]->fifo.empty()) {
    for (auto elem : values()) {
      assert(!elem->fifo.empty());
      if (!elem->forward(true))
        return;
    }
    for (auto elem : values()) {
      assert(elem->forward(false));
    }
    return;
  }

  assert(_ops.size() <= 3);

  _back_array.resize(_ops.size());
  std::fill(_back_array.begin(), _back_array.end(), 0);

  // Check all the operands ready to go!
  for (size_t i = 0; i < _ops.size(); ++i) {
    if (!_ops[i].ready()) {
      return;
    }
  }

  if (print) {
    _ssdfg->dbg_stream() << name() << " (" << _ID << "): ";
  }

  bool discard(false), reset(false), pred(true);
  uint64_t output = 0;

  std::ostringstream reason;
  _invalid = false;

  _input_vals.resize(_ops.size(), 0);

  for (unsigned i = 0; i < _ops.size(); ++i) {
    if (_ops[i].is_imm()) {
      _input_vals[i] = imm();
    } else {
      _input_vals[i] = _ops[i].poll();
      if (!_ops[i].predicate()) {
        _invalid = true;
        reason << "operand " << i << " not valid!";
      } else if (_ops[i].is_ctrl()) {
        _ctrl_bits.test(_input_vals[i], _back_array, discard, pred, reset);
      }
    }
  }

  // we set this instruction to invalid
  if (!pred) {
    _invalid = true;
  }

  if (!_invalid) {  // IF VALID
    if (print) {
      for (size_t i = 0; i < _ops.size(); ++i)
        _ssdfg->dbg_stream() << std::hex << _input_vals[i] << " ";
    }

    _ssdfg->inc_total_dyn_insts();

    // Read in some temp value and set _val after inst_lat cycles
    output = do_compute(discard);
    _self_bits.test(output, _back_array, discard, pred, reset);

    if (print) {
      print_output(_ssdfg->dbg_stream());
    }
  } else {
    _output_vals.resize(values().size());
  }

  // TODO/FIXME: change to all registers
  if (reset) {
    std::cout << "Reset the register file! " << _reg[0] << "\n";
    for (size_t i = 0; i < _reg.size(); ++i)
      _reg[i] = 0;
  }

  if (print) {
    std::cout << (_invalid ? " invalid " : " valid ") << reason.str() << " "
              << (discard ? " and output discard!" : "") << std::endl;
  }

  for (size_t i = 0; i < _back_array.size(); ++i) {
    if (_back_array[i]) {
      if (print) {
        std::cout << "backpressure on " << i << " input\n";
      }
    } else {
      _ops[i].pop();
    }
  }

  _invalid |= discard;
  for (size_t i = 0; i < values().size(); ++i) {
    values()[i]->push(_output_vals[i], !_invalid, lat_of_inst());
  }

  for (auto elem : values()) {
    if (!elem->forward(true))
      return;
  }

  for (auto elem : values()) {
    assert(elem->forward(false));
  }

}

bool SSDfgOperand::ready() {
  if (edges.empty()) {
    return true;
  }
  for (size_t i = 0; i < fifos.size(); ++i) {
    if (fifos[i].empty()) {
      return false;
    }
    if (edges[i]->use()->_ssdfg->cur_cycle() < fifos[i].front().available_at) {
      return false;
    }
  }
  return true;
}

bool SSDfgValue::forward(bool attempt) {
  if (fifo.empty())
    return false;
  simulation::Data data(fifo.front());
  for (auto user : _uses) {
    int j = 0;
    for (auto &operand : user->use()->ops()) {
      ++j;
      for (size_t i = 0; i < operand.edges.size(); ++i) {
        auto edge = operand.edges[i];
        if (edge == user) {
          // TODO: delete this!
          //std::cout << (attempt ? "[attempt] " : "[actual] ") << _node->name()
          //          << " pushes " << data.value << "(" << data.valid << ")" << "to "
          //          << user->use()->name() << "'s " << j << "th operand "
          //          << operand.fifos[i].size() + 1 << "/" << edge->buffer_size()
          //          << std::endl;
          if (operand.fifos[i].size() < edge->buffer_size()
              /*FIXME: The buffer size should be something more rigorous*/) {
            simulation::Data entry(data.available_at + edge->delay(), data.value, data.valid);
            if (!attempt) {
              operand.fifos[i].push(entry);
            }
          } else {
            return false;
          }
        }
      }
    }
  }
  if (!attempt) {
    fifo.pop();
  }
  return true;
}

void SSDfgVecInput::forward() {
  for (auto elem : values()) {
    if (!elem->forward(true))
      return;
  }
  for (auto elem : values()) {
    assert(elem->forward(false));
  }
}

int SSDfg::forward(bool asap) {
  std::vector<int> to_forward;
  if (!asap) {
    for (size_t i = 0; i < num_groups(); ++i) {
      auto group_ready = [] (std::vector<SSDfgVecInput *> &ins) {
        for (auto &elem : ins) {
          for (auto &value : elem->values()) {
            if (!value->forward(true)) {
              return false;
            }
          }
        }
        return true;
      };
      if (group_ready(vec_in_group(i))) {
        to_forward.push_back(i);
      }
    }
  }
  for (auto elem : nodes<SSDfgNode*>()) {
    auto in_ready_group = [to_forward] (int x) {
      for (auto elem : to_forward) {
        if (elem == x) {
          return true;
        }
      }
      return false;
    };
    if (asap || in_ready_group(elem->group_id())) {
      //std::cout << elem->name() << " forward" << std::endl;
      elem->forward();
    } else {
      std::cout << "cannot forward " << elem->name() << std::endl;
    }
  }
  ++_cur_cycle;
  int res = _total_dyn_insts;
  _total_dyn_insts = 0;
  return res;
}

bool SSDfgVecInput::can_push() {
  for (auto elem : values()) {
    if (!elem->fifo.empty()) {
      return false;
    }
  }
  return true;
}

void SSDfgVecOutput::pop(std::vector<uint64_t>& data, std::vector<bool>& data_valid) {
  for (auto &operand : _ops) {
    data.push_back(operand.poll());
    data_valid.push_back(operand.predicate());
    operand.pop();
  }
}

bool SSDfgVecOutput::can_pop() {
  for (auto &elem : _ops) {
    if (!elem.ready()) {
      return false;
    }
  }
  return true;
}

uint64_t SSDfgOperand::poll() {
  assert(ready());
  uint64_t res = 0;
  for (int i = fifos.size() - 1; i >= 0; --i) {
    uint64_t full = ~0ull >> (64 - edges[i]->bitwidth());
    uint64_t sliced = (fifos[i].front().value >> edges[i]->l()) & full;
    res = res << edges[i]->bitwidth() | sliced;
  }
  return res;
}

double SSDfg::estimated_performance(Schedule *sched, bool verbose) {
  std::vector<std::vector<double>> bw(num_groups(), std::vector<double>(2));
  double coef = 1.0;
  
  for (auto &elem : nodes<SSDfgVecInput*>()) {
    if (elem->meta.op == (int) ssdfg::MetaPort::Operation::Read &&
        elem->meta.source != ssdfg::MetaPort::Data::Unknown) {
      bw[elem->group_id()][elem->meta.source == ssdfg::MetaPort::Data::SPad] += 
        elem->get_vp_len() * elem->get_port_width() / 8;
    } else if (elem->meta.op == (int) ssdfg::MetaPort::Operation::IndRead ||
               elem->meta.op == (int) ssdfg::MetaPort::Operation::Atomic) {
      if (!sched->ssModel()->indirect())
        coef = 0.0;
    }
    coef *= elem->meta.cmd;
  }

  std::vector<int> inst_cnt(num_groups(), 0);
  for (auto &elem : nodes<SSDfgInst*>()) {
    ++inst_cnt[elem->group_id()];
  }

  std::vector<double> bw_coef(num_groups(), 1.0);
  for (int i = 0; i < num_groups(); ++i) {
    for (int j = 0; j < 2; ++j) {
      if (bw[i][j] > 64) {
        bw_coef[i] = std::min(bw_coef[i], 64. / bw[i][j]);
      }
    }
  }

  std::vector<double> nmlz_freq;
  for (int i = 0; i < num_groups(); ++i) {
    nmlz_freq.push_back(group_prop(i).frequency);
  }
  double nmlz = *std::max_element(nmlz_freq.begin(), nmlz_freq.end());
  for (int i = 0; i < num_groups(); ++i) {
    nmlz_freq[i] /= nmlz;
  }

  std::vector<double> rec_lat(num_groups(), 0.0);
  std::vector<double> rec_hide(num_groups(), 0.0);
  for (auto &elem : nodes<SSDfgVecOutput*>()) {
    if (elem->meta.dest == ssdfg::MetaPort::Data::LocalPort) {
      double lat = sched->latOf(elem);
      double hide = elem->meta.conc / group_prop(elem->group_id()).unroll;
      if (lat > hide) {
        rec_lat[elem->group_id()] = lat;
        rec_hide[elem->group_id()] = hide;
      }
    }
  }

  double overall = 0.0;

  for (int i = 0; i < num_groups(); ++i) {
    double v = std::min(bw_coef[i], rec_hide[i] / rec_lat[i]) * inst_cnt[i] * nmlz_freq[i];
    if (verbose) {
      std::cerr << "[Group " << i << "] Freq: " << group_prop(i).frequency
                << ", #Insts:" << inst_cnt[i] << ", Memory: " << bw[i][0]
                << ", SPad: " << bw[i][1] << ", Rec: " << rec_hide[i] << "/" << rec_lat[i]
                << ", Overall: " << v << std::endl;
    }
    overall += v;
  }

  return overall * coef;
}

namespace ssdfg {

const char *MetaPort::DataText[] = {
  "memory",
  "spad",
  "localport",
  "remoteport",
};

const char *MetaPort::OperationText[] = {
  "read",
  "write",
  "indread",
  "indwrite",
  "atomic",
};

CompileMeta::CompileMeta(const MetaPort &meta, SSDfgVec *parent) : MetaPort(meta), parent(parent) {
  assert(parent);
  if (dest == Data::LocalPort && !dest_port.empty()) {
    bool found = false;
    for (auto iv : parent->ssdfg()->nodes<SSDfgVecInput*>()) {
      if (iv->name() == dest_port) {
        destination = iv;
        found = true;
      }
    }
    assert(found && "No destination found!");
  } else {
    destination = nullptr;
  }
}

}
