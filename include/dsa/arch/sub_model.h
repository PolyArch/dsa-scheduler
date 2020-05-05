#ifndef __SS_SUB_MODEL_H__
#define __SS_SUB_MODEL_H__

#include <algorithm>
#include <boost/any.hpp>
#include <boost/utility/binary.hpp>
#include <bitset>
#include <climits>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <functional>

#include "dsa/debug.h"
#include "fu_model.h"
#include "predict.h"
#include "json/visitor.h"


#define log2ceil(x) (63U - __builtin_clzl(static_cast<uint64_t>(x)) + 1)

namespace dsa {

namespace adg {

class Visitor;

}

class SpatialFabric;

using std::ofstream;
using std::ostream;
using std::to_string;

const int MAX_SUBNETS = 8;

class ssnode;
class ssvport;

template <typename T>
T get_prop_attr(plain::Object& prop, const std::string& name, T dft) {
  auto iter = prop.find(name);
  if (iter != prop.end())
    return *iter->second->As<T>();
  else
    return dft;
}

template <typename T>
void fix_id(std::vector<T>& vec) {
  for (int i = 0, n = vec.size(); i < n; ++i) {
    vec[i]->set_id(i);
  }
}

template <typename T>
void vec_delete_by_id(std::vector<T>& vec, std::vector<int>& indices) {
  auto new_end = std::remove_if(vec.begin(), vec.end(), [&](T fu) {
    for (int index : indices) {
      if (fu->id() == index) return true;
    }
    return false;
  });
  vec.erase(new_end, vec.end());
}

// TODO: Should we delete this class?
class ssio_interface {
 public:
  std::map<int, ssvport*> vports_map[2];

  using EntryType = std::pair<int, ssvport*>;
  std::vector<EntryType> vports_vec[2];

  std::map<int, ssvport*>& vports(bool is_input) { return vports_map[is_input]; }

  ssvport* get(bool is_input, int id) {
    auto& ports = vports(is_input);
    auto iter = ports.find(id);
    assert(iter != ports.end());
    return iter->second;
  }

  void fill_vec();
};

class sslink {
 public:
  sslink() {}

  ~sslink();

  ssnode* orig() const { return _orig; }

  ssnode* dest() const { return _dest; }

  // Constructor
  sslink(ssnode* orig, ssnode* dest) {
    _orig = orig;
    _dest = dest;
    _ID = -1;
  }

  std::string name() const;

  int id() { return _ID; }

  void set_id(int id) { _ID = id; }

  int max_util() { return _max_util; }

  int set_max_util(int m) { return _max_util = m; }

  // Right I ignore this because area model doesn't use it anyways
  void set_flow_control(bool v) {
    _flow_control = v;
    assert(0);
  }
  bool flow_control();

  int bitwidth() { return _bitwidth; }

  int decomp_bitwidth() { return _decomp_bitwidth; }

  /* To check the connectivity in O(1), here we apply a tricky idea:
   * Originally, subnet[i][j] indicates subnet `i' is connected to subnet `j'.
   * This is unfriendly to check a bulk of connectivity: when the width is `w', and we
   * want to go from `i' to `j', we need to check subnet[i+k][j+k] where k = 0..(w-1),
   * which is O(w).
   *
   * Here we first transform the meaning of subnet[i][delta], which means
   * it is able to go from `i' to `i+delta' subnet. If we want to check connectivity
   * between `i' and `j', under width `w', where delta=j-i, the check becomes:
   * subnet[i+k][delta], where k=0..(w-1), which is still O(w).
   *
   * Then two tricks together enables O(1) check:
   * 1. Transposing the matrix makes the check subnet[delta][i+k], this makes access
   * continuous.
   * 2. Squeezing the inner dimension of subnet to bit representation, so that we can
   * check the one's in O(1) by bit operation.
   * */
  std::vector<int64_t> subnet;

  int slots(int slot, int width) {
    uint64_t res = 0;
    int n = subnet.size();
    auto f = [](int64_t a, int bits) {
      int full_mask = ~0ull >> (64 - bits);
      return (a & full_mask) == full_mask;
    };
    for (int i = 0; i < n; ++i) {
      if (slot + width <= n) {
        if (f(subnet[i] >> slot, width)) {
          res |= 1 << (i + slot) % n;
        }
      } else {
        int high = n - slot;
        int low = width - high;
        if (f(subnet[i] >> slot, high) && f(subnet[i], low)) {
          res |= 1 << (i + slot) % n;
        }
      }
    }
    return res;
  }

 protected:
  int _ID = -1;

  // By default, assume single-flopped, dedicated, and flow-control, 64-bit
  int _max_util = 1;          // max instructions can map to this link
  bool _flow_control = true;  // whether link supports backpressure
  int _bitwidth = 64;         // bitwidth of link
  int _decomp_bitwidth = 8;   // minimum bitwidth the link may be decomposed into

  ssnode* _orig;
  ssnode* _dest;

 private:
  friend class SpatialFabric;
};

class ssnode {
 public:
  enum NodeType { FU, Switch, InPort, OutPort, Unknown };

  ssnode() {}

  // TODO(@were): Deprecate this in the visitor pattern.
  virtual ssnode* copy() = 0;

  virtual void Accept(adg::Visitor *visitor) = 0;

  sslink* add_link(ssnode* node);

  virtual std::string name() const = 0;

  virtual int delay_fifo_depth() { return 0; }

  // just for visualization
  void setXY(int x, int y) {
    _x = x;
    _y = y;
  }

  int x() const { return _x; }

  int y() const { return _y; }

  const std::vector<sslink*>& in_links() { return links[1]; }

  const std::vector<sslink*>& out_links() { return links[0]; }

  virtual bool is_hanger() { return false; }

  std::string nodeType() { return node_type; }

  int id() { return _ID; }

  void set_ssnode_prop(plain::Object &prop) {
    node_type = *prop["nodeType"] -> As<std::string>();
    data_width = *prop["data_width"] -> As<int64_t>();

    // Parse Decomposer
    if (node_type != "vector port") {
      granularity = get_prop_attr(prop, "granularity", static_cast<int64_t>(granularity));
    }
    decomposer = data_width / granularity;
  }

  void set_id(int id) { _ID = id; }
  virtual void dumpIdentifier(ostream& os) = 0;
  virtual void dumpFeatures(ostream& os) = 0;
  virtual uint64_t get_config_bits() = 0;

  int node_dist(int slot) { return _node_dist[slot]; }

  std::pair<int, sslink*> came_from(int slot) { return _came_from[slot]; }

  int done(int slot) { return _done[slot]; }
  void set_done(int slot, int n) { _done[slot] = n; }

  void update_dist_only(int slot, int dist) { _node_dist[slot] = dist; }

  void update_dist(int slot, int dist, int from_slot, sslink* from) {
    _node_dist[slot] = dist;
    _came_from[slot] = std::make_pair(from_slot, from);
  }

  void reset_runtime_vals() {
    memset(_node_dist, -1, sizeof _node_dist);
    memset(_came_from, 0, sizeof _came_from);
    memset(_done, 0, sizeof _done);
  }

  int max_util() { return _max_util; }

  int max_util(dsa::OpCode inst) {
    return _max_util * 64 / dsa::bitwidth[inst];
  }

  int set_max_util(int m) { return _max_util = m; }

  bool is_shared() { return _max_util > 1; }  // TODO: max_util > 1

  void set_flow_control(bool v) { _flow_control = v; }
  bool &flow_control() { return _flow_control; }

  int bitwidth() { return _bitwidth; }

  virtual ~ssnode(){};

  int decomposer{8};
  int granularity{8};
  int mf_decomposer{8};
  int data_width{64};

 protected:
  std::string node_type = "empty";
  SpatialFabric *parent{nullptr};
  int num_node();
  int _ID = -1;
  int _x = -1, _y = -1;  // just for visualization

  int _max_util = 1;          // Convert from "share_slot_size"
  bool _flow_control = true;  // convert from "flow_control"
  int _bitwidth = 64;         // maximum bitwidth of PE, convert from "bit_width"

  std::vector<sslink*> links[2];  // {output, input}

  // Variables used for scheduling -- these should be moved out at some point (TODO)
  int _node_dist[8];
  int _done[8];
  std::pair<int, sslink*> _came_from[8];

  // convert from decomposer // to be integrate with subnet_table
  // TODO: most-fine-grain decomposer, to be removed or need to be defined by user

 private:
  friend class SpatialFabric;
  friend class sslink;
};

class ssswitch : public ssnode {
 public:
  ssswitch() : ssnode() {}

  void Accept(adg::Visitor *visitor) override;

  ssnode* copy() override {
    auto res = new ssswitch();
    *res = *this;
    return res;
  }

  virtual std::string name() const override {
    std::stringstream ss;
    if (_x != -1 && _y != -1) {
      ss << "SW"
         << "_" << _x << "_" << _y;
    } else {
      ss << "SW" << _ID;
    }
    return ss.str();
  }
  void dumpIdentifier(ostream& os) override {
    os << "[" + to_string(_ID) + ",\"switch\"" + "]";
  }
  void dumpFeatures(ostream& os) override {
    os << "{\n";
    // ID
    os << "\"id\" : " << id() << ",\n";
    // NodeType
    os << "\"nodeType\" : "
       << "\"switch\""
       << ",\n";
    // data width
    os << "\"data_width\" : " << data_width << ",\n";
    // granularity
    os << "\"granularity\" : " << granularity << ",\n";
    // number of input
    int num_input = in_links().size();
    os << "\"num_input\" : " << num_input << ",\n";
    // number of output
    int num_output = out_links().size();
    os << "\"num_output\" : " << num_output << ",\n";
    // flow control
    os << "\"flow_control\" : " << (flow_control() ? "true" : "false") << ",\n";
    // max util
    os << "\"max_util\" : " << max_util() << ",\n";
    // input nodes
    os << "\"input_nodes\" : [";
    int idx_link = 0;
    for (auto in_link : in_links()) {
      in_link->orig()->dumpIdentifier(os);
      if (idx_link < num_input - 1) {
        idx_link++;
        os << ", ";
      }
    }
    os << "],\n";
    // output nodes
    os << "\"output_nodes\" : [";
    idx_link = 0;
    for (auto out_link : out_links()) {
      out_link->dest()->dumpIdentifier(os);
      if (idx_link < num_output - 1) {
        idx_link++;
        os << ", ";
      }
    }
    os << "]";

    os << "}\n";
  }

  void set_prop(plain::Object & prop) {
    set_ssnode_prop(prop);
    // max output fifo depth
    max_fifo_depth = get_prop_attr(prop, "max_fifo_depth", static_cast<int64_t>(max_fifo_depth));
  }

  void collect_features() {
    if (decomposer == 0) decomposer = mf_decomposer;
    features[0] = _max_util > 1 ? 0.0 : 1.0;
    features[1] = _max_util > 1 ? 1.0 : 0.0;

    assert(features[0] || features[1]);
    features[2] = _flow_control ? 0.0 : 1.0;
    features[3] = _flow_control ? 1.0 : 0.0;
    assert((features[2] || features[3]) &&
           "Either Data(Static) or DataValidReady(Dynamic)");
    features[4] = decomposer;
    if (decomposer == 0 && (decomposer & (decomposer - 1))) {
      std::cout << "Problem: Decomposer for node " << name() << " is: " << decomposer
                << "\n";
      assert(0 && "Decomposer need to be power of two");
    }
    features[5] = max_fifo_depth;
    features[6] = links[1].size();
    features[7] = links[0].size();
    features[8] = _max_util;
  }

  virtual ~ssswitch(){};

  void print_features() {
    std::cout << "------ Features : >>>>>> ";
    std::cout << "Not Shared ? " << features[0] << ", "
              << "Shared ? " << features[1] << ", "
              << "Not Flow Control ? " << features[2] << ", "
              << "Flow Control ? " << features[3] << ", "
              << "decomposer = " << features[4] << ", "
              << "max fifo depth = " << features[5] << ", "
              << "# input links = " << features[6] << ", "
              << "# output links = " << features[7] << ", "
              << "max util = " << features[8] << ", ";
    std::cout << " ------ Feature Ends <<<<<<\n";
  }

  void dump_features() {
    for (int i = 0; i < 9; ++i) {
      std::cout << features[i] << " ";
    }
    std::cout << "\n";
  }

  void route_io(int in_idx, int out_idx){
    routing_lookup[out_idx] = in_idx;
  }


  uint64_t get_config_bits() override {
    uint64_t left_bits = 64;
    // TODO: add shared config bits
    uint64_t num_config_index_bit = log2ceil(_max_util);
    // -- configure which configuration to write
    if(_max_util > 1){ 
      // dedicated dont need to have this field
      config_bits = config_bits << num_config_index_bit;
      left_bits -= num_config_index_bit;
      // config_bits += `config_index`;
    }
    // -- configure the current utility
    if(_max_util > 1){
      // dedicated dont need to have this field
      config_bits = config_bits << num_config_index_bit;
      left_bits -= num_config_index_bit;
      // config_bits += `curr_util`;
    }
    // -- config the ID
    assert(num_node() > 0 && "number of node not parsed?");
    uint64_t num_id_bit = log2ceil(num_node());
    config_bits = config_bits << num_id_bit;
    left_bits -= num_id_bit;
    config_bits += id();
    // -- config source select
    uint64_t num_source_sel_bit  = log2ceil(in_links().size()+1);
    // + 1 is because zero means connect to ground
    for(uint output_idx = 0; output_idx < out_links().size(); output_idx++){
      config_bits = config_bits << num_source_sel_bit;
      left_bits -= num_source_sel_bit;
      if(routing_lookup.count(output_idx)){
        config_bits += routing_lookup[output_idx] + 1;
      }
    }
    // -- config offset for decomposability //TODO
    config_bits = config_bits << left_bits;
    
    return config_bits;
  }

 protected:
  double features[9];
  int max_fifo_depth = 2;
  std::map<int, int> routing_lookup;
  uint64_t config_bits = 0;
  // Sihao: routing_lookup = {0->3, 1->4, 3->1}
  // means output0 receive input3
  //       output1 receive input4
  //       output3 receive input1
  // for those output port not mapped, they are connect to ground
};

class ssfu : public ssnode {
 public:
  ssfu() : ssnode() {}

  ssnode* copy() override {
    auto res = new ssfu();
    *res = *this;
    return res;
  }

  void Accept(adg::Visitor *visitor);
  void set_prop(plain::Object & prop) {

    set_ssnode_prop(prop);

    std::string nodeType = *prop["nodeType"]->As<std::string>();

    // delay_fifo_depth
    _delay_fifo_depth = get_prop_attr(prop, "max_delay_fifo_depth", static_cast<int64_t>(_delay_fifo_depth));

    // register_file_size
    register_file_size = get_prop_attr(prop, "register_file_size", static_cast<int64_t>(register_file_size));
  }

  void dumpIdentifier(ostream& os) override {
    os << "[" + to_string(_ID) + ",\"function unit\"" + "]";
  }

  void dumpFeatures(ostream& os) override {
    os << "{\n";
    // ID
    os << "\"id\" : " << id() << ",\n";
    // NodeType
    os << "\"nodeType\" : "
       << "\"function unit\""
       << ",\n";
    // data width
    os << "\"data_width\" : " << data_width << ",\n";
    // granularity
    os << "\"granularity\" : " << granularity << ",\n";
    // number of input
    int num_input = in_links().size();
    os << "\"num_input\" : " << num_input << ",\n";
    // number of output
    int num_output = out_links().size();
    os << "\"num_output\" : " << num_output << ",\n";
    // flow control
    os << "\"flow_control\" : " << (flow_control() ? "true" : "false") << ",\n";
    // max util
    os << "\"max_util\" : " << max_util() << ",\n";
    // max delay fifo depth
    os << "\"max_delay_fifo_depth\" : " << _delay_fifo_depth << ",\n";
    // number of register
    os << "\"num_register\" : " << register_file_size << ",\n";
    // Instructions
    os << "\"instructions\" : [";
    int idx_inst = 0;
    int num_inst = fu_type_.capability.size();
    for (auto &elem : fu_type_.capability) {
      os << "\"" << dsa::name_of_inst(elem.op) << "\"";
      if (idx_inst < num_inst - 1) {
        os << ", ";
        idx_inst++;
      }
    }
    os << "],\n";
    // input nodes
    os << "\"input_nodes\" : [";
    int idx_link = 0;
    for (auto in_link : in_links()) {
      in_link->orig()->dumpIdentifier(os);
      if (idx_link < num_input - 1) {
        idx_link++;
        os << ", ";
      }
    }
    os << "],\n";
    // output nodes
    os << "\"output_nodes\" : [";
    idx_link = 0;
    for (auto out_link : out_links()) {
      out_link->dest()->dumpIdentifier(os);
      if (idx_link < num_output - 1) {
        idx_link++;
        os << ", ";
      }
    }
    os << "]";
    os << "}\n";
  }

  std::string name() const override {
    std::stringstream ss;
    if (_x != -1 && _y != -1) {
      ss << "FU" << _x << "_" << _y;
    } else {
      ss << "FU" << _ID;
    }
    return ss.str();
  }

  double* collect_features() {
    if (decomposer == 0)
      decomposer = mf_decomposer;

    features[0] = _max_util > 1 ? 0.0 : 1.0;
    features[1] = _max_util > 1 ? 1.0 : 0.0;

    assert(features[0] || features[1]);
    features[2] = !_flow_control ? 1.0 : 0.0;
    features[3] = _flow_control ? 1.0 : 0.0;
    assert((features[2] || features[3]) &&
           "Either Data(Static) or DataValidReady(Dynamic)");
    features[6] = decomposer;
    if (decomposer == 0 && (decomposer & (decomposer - 1))) {
      std::cout << "Problem: Decomposer for node " << name() << " is: " << decomposer
                << "\n";
      assert(0 && "Decomposer need to be power of two");
    }
    features[7] = _delay_fifo_depth;
    features[8] = links[1].size();
    features[9] = links[0].size();
    features[10] = register_file_size;
    features[11] = _max_util;

    // print_features();

    return features;
  }

  virtual ~ssfu(){};

  void print_features() {
    std::cout << " ------ Features : >>>>>> ";
    std::cout << "Not Shared ? " << features[0] << ", "
              << "Shared ? " << features[1] << ", "
              << "Not Flow Control ? " << features[2] << ", "
              << "Flow Control ? " << features[3] << ", "
              << "Output Mode Individual ? " << features[4] << ", "
              << "Output Mode Universal ? " << features[5] << ", "
              << "decomposer = " << features[6] << ", "
              << "max delay fifo depth = " << features[7] << ", "
              << "# input links = " << features[8] << ", "
              << "# output links = " << features[9] << ", "
              << "# register = " << features[10] << ", "
              << "max util = " << features[11] << ", ";
    std::cout << " ------ Feature Ends <<<<<<\n";
  }

  void dump_features() {
    for (int i = 0; i < 12; ++i) {
      std::cout << features[i] << " ";
    }
    std::cout << "\n";
  }

  bool is_hanger() override { return in_links().size() <= 1 || out_links().size() < 1; }

  void set_delay_fifo_depth(int d) { _delay_fifo_depth = d; }

  virtual int delay_fifo_depth() override { return _delay_fifo_depth; }

  void add_delay(int operand_idx, int delay_cycle){
    delay_map[operand_idx] = delay_cycle;
  }
  void add_operand_sel(int operand_idx, int input_port_idx){
    operand_sel[operand_idx] = input_port_idx;
  }
  void set_curr_opcode(int local_opcode){
    curr_opcode = local_opcode;
  }

  uint64_t get_config_bits() override {
    uint64_t left_bits = 64;
    // TODO: add shared config bits
    uint64_t num_config_index_bit = log2ceil(_max_util);
    // -- configure which configuration to write
    if(_max_util > 1){ 
      // dedicated dont need to have this field
      config_bits = config_bits << num_config_index_bit;
      left_bits -= num_config_index_bit;
      // config_bits += `config_index`;
    }
    // -- configure the current utility
    if(_max_util > 1){
      // dedicated dont need to have this field
      config_bits = config_bits << num_config_index_bit;
      left_bits -= num_config_index_bit;
      // config_bits += `curr_util`;
    }
    // -- config the ID
    assert(num_node() > 0 && "number of node not parsed?");
    uint64_t num_id_bit = log2ceil(num_node());
    config_bits = config_bits << num_id_bit;
    left_bits -= num_id_bit;
    config_bits += id();
    // -- config the operand source
    uint64_t num_operand_sel_bit = log2ceil(in_links().size() + 1);
    uint max_num_operand = fu_type_.get_max_num_operand();
    // + 1 is because zero connect to ground
    for(uint operand_idx = 0; operand_idx < max_num_operand; operand_idx ++){
      config_bits = config_bits << num_operand_sel_bit;
      left_bits -= num_operand_sel_bit;
      if(operand_sel.count(operand_idx)){
        config_bits += operand_sel[operand_idx] + 1;
      }
    }
    // -- config the delay cycle
    uint64_t num_delay_sel_bit = log2ceil(_delay_fifo_depth + 1);
    std::cout << "delay fifo depth = " << _delay_fifo_depth << std::endl;
    // + 1 mean we allow zero cycle delay
    for(uint operand_idx = 0; operand_idx < max_num_operand; operand_idx ++){
      config_bits = config_bits << num_delay_sel_bit;
      left_bits -= num_delay_sel_bit;
      if(delay_map.count(operand_idx)){
        config_bits += delay_map[operand_idx];
      }
    }
    // -- config opcode
    uint64_t num_opcode_bit = log2ceil(static_cast<uint64_t>(fu_type_.capability.size()));
    config_bits = config_bits << num_opcode_bit;
    left_bits -= num_opcode_bit;
    config_bits += curr_opcode;
    // -- config offset for decomposability //TODO
    // -- config output mode for power saving and shared mode
    config_bits = config_bits << left_bits;
    return config_bits;
  }

  Capability fu_type_;

 protected:
  double features[12];
  int _delay_fifo_depth = 8;
  int register_file_size = 4;
  std::map<int,int> delay_map;
  std::map<int,int> operand_sel;
  int curr_opcode = -1;
  uint64_t config_bits = 0;
 private:
  friend class SpatialFabric;
};

// This should be improved later
class ssvport : public ssnode {
 public:
  ssnode* copy() override {
    auto res = new ssvport();
    *res = *this;
    return res;
  }

  void Accept(adg::Visitor *vistor);

  std::vector<int>& port_vec() { return _port_vec; }
  void set_port_vec(std::vector<int> p) { _port_vec = p; }
  size_t size() { return _port_vec.size(); }
  std::string name() const override {
    std::stringstream ss;
    if (links[0].size() > 0)
      ss << "I";
    else
      ss << "O";
    if (_port != -1) {
      ss << "P" << _port;
    } else {
      ss << _ID;
    }
    return ss.str();
  }

  void dumpIdentifier(ostream& os) override {
    os << "[" + std::to_string(_ID) + ",\"vector port\"" + "]";
  }
  void dumpFeatures(ostream& os) override {
    os << "{\n";
    // ID
    os << "\"id\" : " << id() << ",\n";
    // # Port
    os << "\"port\" : " << port() << ",\n";
    // NodeType
    os << "\"nodeType\" : "
       << "\"vector port\""
       << ",\n";
    // data width
    os << "\"data_width\" : " << data_width << ",\n";
    // granularity
    os << "\"granularity\" : " << granularity << ",\n";
    // number of input
    int num_input = in_links().size();
    os << "\"num_input\" : " << num_input << ",\n";
    // number of output
    int num_output = out_links().size();
    os << "\"num_output\" : " << num_output << ",\n";
    // flow control
    os << "\"flow_control\" : " << (flow_control() ? "true" : "false") << ",\n";
    // max util
    os << "\"max_util\" : " << max_util() << ",\n";
    // input nodes
    os << "\"input_nodes\" : [";
    int idx_link = 0;
    for (auto in_link : in_links()) {
      in_link->orig()->dumpIdentifier(os);
      if (idx_link < num_input - 1) {
        idx_link++;
        os << ", ";
      }
    }
    os << "],\n";
    // output nodes
    os << "\"output_nodes\" : [";
    idx_link = 0;
    for (auto out_link : out_links()) {
      out_link->dest()->dumpIdentifier(os);
      if (idx_link < num_output - 1) {
        idx_link++;
        os << ", ";
      }
    }
    os << "]";
    os << "}\n";
  }

  int bitwidth_capability() {
    int res = 0;
    CHECK((int) links[0].empty() + (int) links[1].empty() == 1);
    for (auto &elem : links) {
      for (auto link : elem) {
        res += link->bitwidth();
      }
    }
    return res;
  }

  void set_port2node(std::string portname, ssnode* node) { port2node[portname] = node; }
  ssnode* convert_port2node(std::string portname) { return port2node[portname]; }
  int port() { return _port; }

  virtual ~ssvport(){};
  void set_port(int port) { _port = port; }

  bool is_hanger() override { return in_links().empty() && out_links().empty(); }

  uint64_t get_config_bits() override {
    return 0;
  }

 private:
  int _port = -1;
  std::vector<int> _port_vec;
  std::string io_type;
  int channel_buffer;
  std::map<std::string, ssnode*> port2node;
};

}  // namespace dsa

#endif
