#ifndef __SS_SUB_MODEL_H__
#define __SS_SUB_MODEL_H__

#include "fu_model.h"
#include "direction.h"
#include <string>
#include <vector>
#include <sstream>
#include <unordered_map>
#include <map>
#include <utility>
#include <algorithm>
#include "./predict.h"
#include <yaml-cpp/yaml.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/any.hpp>

namespace SS_CONFIG {

const int MAX_SUBNETS = 8;

class ssnode;
class ssvport;

//TODO: Should we delete this class?
class ssio_interface {
public:

  std::map<int, ssvport*> vports_map[2];

  using EntryType = std::pair<int, ssvport*>;
  std::vector<EntryType> vports_vec[2];

  std::map<int, ssvport*> &vports(bool is_input) {
    return vports_map[is_input];
  }

  ssvport* get(bool is_input, int id) {
    auto &ports = vports(is_input);
    auto iter = ports.find(id);
    assert(iter != ports.end());
    return iter->second;
  }

  void fill_vec();

private:        
    void sort(std::vector<std::pair<int,int>>& portID2size, 
         std::map<int,ssvport*>& vports);
};

class sslink {
public:

  sslink() {}

  ssnode *orig() const { return _orig; }

  ssnode *dest() const { return _dest; }

  SwitchDir::DIR dir() const { return _dir; }

  sslink* setdir(SwitchDir::DIR dir) { _dir = dir; return this;}

  //Constructor
  sslink(ssnode *orig, ssnode *dest) {
    _orig = orig;
    _dest = dest;
    _ID = -1;
  }

  sslink* getCycleLink();

  std::string name() const;

  std::string gams_name(int config) const;

  std::string gams_name(int, int) const;

  int id() { return _ID; }

  void set_id(int id) { _ID = id; }

  int max_util() { return _max_util; }

  int set_max_util(int m) { return _max_util = m; }

  bool flow_control() {return _flow_control;}

  int  bitwidth() {return _bitwidth;}

  int  decomp_bitwidth() {return _decomp_bitwidth;}

  void set_lat(int i) {_lat=i;}
  int lat() {return _lat;}

  void set_out_index(int i) {_out_index=i;}
  void set_in_index(int i) {_in_index=i;}

  int in_index() {return _in_index;}
  int out_index() {return _out_index;}

protected:
  int _ID = -1;

  // By default, assume single-flopped, dedicated, and flow-control, 64-bit
  int _lat = 1; //whether there is a latch at the end of this stage
  int _max_util = 1; // max instructions can map to this link
  bool _flow_control = true; // whether link supports backpressure
  int _bitwidth = 64;  // bitwidth of link
  int _decomp_bitwidth = 8; // minimum bitwidth the link may be decomposed into

  //this is so we know the relative index of this link in the input and output
  //nodes to which this node belongs
  int _in_index=-1; 
  int _out_index=-1;

  ssnode *_orig;
  ssnode *_dest;
  SwitchDir::DIR _dir;

private:
  friend class SubModel;
};
    
    
class ssnode {
public:
  ssnode() {}

  sslink *add_link(ssnode *node) {
    sslink *link = new sslink(this, node);
    if(this==node) {
      if(_cycle_link) assert(0 && "two cycle links, why?");
      _cycle_link=link;
    }
    link->set_out_index(_out_links.size());
    _out_links.push_back(link);

    node->add_back_link(link);
    return link;
  }

  void add_back_link(sslink *link) {
    link->set_in_index(_in_links.size());
    _in_links.push_back(link);
  }

  virtual std::string name() const = 0; 

  virtual std::string gams_name(int ) const = 0; 

  virtual bool is_input() {return false;}
  virtual bool is_output() {return false;}

  //just for visualization
  void setXY(int x, int y) {
    _x = x;
    _y = y;
  }

  int x() const { return _x; }

  int y() const { return _y; }

  typedef std::vector<sslink *>::const_iterator const_iterator;

  const std::vector<sslink *> &in_links() { return _in_links; }

  const std::vector<sslink *> &out_links() { return _out_links; }

  int num_non_self_out_links() {return _out_links.size() - (_cycle_link!=0);}

  sslink *getFirstOutLink() {
    return _out_links.empty() ? nullptr : _out_links[0];
  }

  sslink *getFirstInLink() {
    return _in_links.empty() ? nullptr : _in_links[0];
  }

  sslink *getInLink(SwitchDir::DIR dir) {
    for (auto &dlink: in_links()) {
      if (dlink->dir() == dir) return dlink;
    }
    return nullptr;
  }

  sslink *getOutLink(SwitchDir::DIR dir) {
    for (auto &dlink: out_links()) {
      if (dlink->dir() == dir) return dlink;
    }
    return nullptr;
  }

  sslink *get_cycle_link() {
    return _cycle_link;
  }

  int id() { return _ID; }

  void set_ssnode_prop(boost::property_tree::ptree prop){

    std::string nodeType = prop.get_child("nodeType").get_value<std::string>();

    // Count the number of input port and output port
    // vector port can have 0 input_node or 0 output_node
    auto input_nodes = prop.get_child_optional("input_nodes");int num_input_nodes = 0;
    auto output_nodes = prop.get_child_optional("output_nodes");int num_output_nodes = 0;
    if(input_nodes.is_initialized())
      num_input_nodes = prop.get_child("input_nodes").size();
    if(output_nodes.is_initialized())
      num_output_nodes = prop.get_child("output_nodes").size();
    
    // Parse Decomposer
    if (nodeType != "vector port"){
      auto d_node = prop.get_child_optional("decomposer");
      if(!d_node.is_initialized()){
        decomposer = mf_decomposer;
        std::cout << "Warning: non-vectorport-node need to have decomposer as properties,"<<
         "use most fine grain decomposer instead\n";
      }else{
        decomposer = prop.get_child("decomposer").get_value<int>();
      }
    }
    
    // parser the subnet table
    auto subnet_table_node = prop.get_child_optional("subnet_table");
    if (subnet_table_node.is_initialized()){
      /*
      std::cout<< "Parsing new subnet table : \n";
      std::cout << "num_input_nodes = " << num_input_nodes <<
              ", num_output_nodes = " << num_output_nodes<<
              ", decomposer = "<< decomposer <<"\n";
      */
      // Convert to temp subnet_table
      std::vector<std::vector<bool>> temp_subnet_table;
      int output_slot_size, input_slot_size;
      assert(num_input_nodes > 0 && num_output_nodes >0 &&
       "why subnet routing without input/output node(s) connected?");
      int output_slot_idx=0;
      auto & output_slots = prop.get_child("subnet_table");
      output_slot_size = output_slots.size();
      assert(output_slot_size == (num_output_nodes * decomposer));
      temp_subnet_table.resize(output_slot_size);
      for(auto & output_slot : output_slots){
        int input_slot_idx=0;
        auto & input_slots = output_slot.second;
        input_slot_size = input_slots.size();
        assert(input_slot_size == (num_input_nodes * decomposer));
        temp_subnet_table[output_slot_idx].resize(input_slot_size);
        for (auto & input_slot : input_slots){
          temp_subnet_table[output_slot_idx][input_slot_idx] =
            input_slot.second.get_value<bool>();
          //std::cout << temp_subnet_table[output_slot_idx][input_slot_idx];
          input_slot_idx++;
          if(input_slot_idx % decomposer == 0){
            //std::cout <<" ";
          }
        }
        //std::cout << "\n";
        output_slot_idx++;
        if(output_slot_idx % decomposer == 0){
          //std::cout <<"\n";
        }
      }
      // Test temp subnet table print out
      // std::cout << "parse temp subnet table successfully, result: \n";
      
      // Initialize the subnet table
      for (int op_idx = 0; op_idx < num_output_nodes; op_idx++){
        _subnet_table.resize(num_output_nodes);
        for (int os_idx = 0; os_idx < mf_decomposer; os_idx++){
          _subnet_table[op_idx].resize(mf_decomposer);
          for (int ip_idx = 0; ip_idx < num_input_nodes; ip_idx++){
            _subnet_table[op_idx][os_idx].resize(num_input_nodes);
            for (int is_idx = 0; is_idx < mf_decomposer; is_idx++){
              _subnet_table[op_idx][os_idx][ip_idx].resize(mf_decomposer);
            }// end of input_slot
          }// end of input_port
        }// end of output_slot
      }//end of output_port

      // Convert from temp subnet table
      int decompose_ratio = mf_decomposer / decomposer;
      for (int op_idx = 0; op_idx < num_output_nodes; op_idx++){
        for (int os_idx = 0; os_idx < decomposer; os_idx++){
          for (int ip_idx = 0; ip_idx < num_input_nodes; ip_idx++){
            for (int is_idx = 0; is_idx < decomposer; is_idx++){
              // Assign to subnet table
              for (int os = os_idx*decompose_ratio;
              os < (1+os_idx)*decompose_ratio; os++){
                for (int is = is_idx*decompose_ratio;
                is < (1+is_idx)*decompose_ratio; is++){
                  if((os%decompose_ratio)==(is%decompose_ratio)){
                    _subnet_table[op_idx][os][ip_idx][is] =
                      temp_subnet_table [op_idx * decomposer + os_idx]
                                        [ip_idx * decomposer + is_idx];
                  }else{
                    _subnet_table[op_idx][os][ip_idx][is] = false;
                  }
                }
              }
            }// end of input_slot
          }// end of input_port
        }// end of output_slot
      }//end of output_port

      // Debug : Test Print subnet table
      /*
      for (int op_idx = 0; op_idx < num_output_nodes; op_idx++){
        for (int os_idx = 0; os_idx < mf_decomposer; os_idx++){
          for (int ip_idx = 0; ip_idx < num_input_nodes; ip_idx++){
            for (int is_idx = 0; is_idx < mf_decomposer; is_idx++){
              std::cout << _subnet_table[op_idx][os_idx][ip_idx][is_idx];
            }// end of input_slot
            std::cout << " ";
          }// end of input_port
          std::cout << "\n";
        }// end of output_slot
        std::cout << "\n";
      }//end of output_port
      // End of Parse subnet_table
      */
    }
  }

  void set_id(int id){
    _ID = id;
  }

  void set_id(std::vector<ssnode *> &node_list,
              std::vector<sslink *> &link_list) {
    _ID = (int) node_list.size();
    node_list.push_back(this);
    for (unsigned i = 0; i < _out_links.size(); ++i) {
      sslink *link = _out_links[i];
      assert(link->id() == -1);
      link->set_id((int) link_list.size());
      link_list.push_back(link);
    }
  }

  int node_dist(int slot) { return _node_dist[slot]; }

  std::pair<int, sslink *> came_from(int slot) { return _came_from[slot]; }

  int done(int slot) {return _done[slot];}
  void set_done(int slot, int n) {_done[slot] = n;}

  void update_dist_only(int slot, int dist) {
    _node_dist[slot] = dist;
  }


  void update_dist(int slot, int dist, int from_slot, sslink *from) {
    _node_dist[slot] = dist;
    _came_from[slot] = std::make_pair(from_slot, from);
  }

  void reset_runtime_vals() {
    memset(_node_dist, -1, sizeof _node_dist);
    memset(_came_from, 0, sizeof _came_from);
    memset(_done, 0, sizeof _done);
  }

  int max_util() { return _max_util; }

  int max_util(SS_CONFIG::ss_inst_t inst) { return _max_util * 64 / SS_CONFIG::bitwidth[inst]; }

  int set_max_util(int m) { return _max_util = m; }

  bool is_shared(){ return _max_util > 1;} // TODO: max_util > 1

  bool flow_control() {return _flow_control;}

  int  bitwidth() {return _bitwidth;}

  void setup_default_routing_table(sslink* inlink, sslink* outlink);
  void setup_routing_memo();
  void setup_routing_memo_node(int i, int slot, int bitwidth);
  std::vector<std::pair<int,sslink*>>& linkslots_for(std::pair<int,sslink*>& p, int bitwidth) {
    int slot = p.first;
    sslink* l = p.second;
    int l_ind = 0;
    if(l) l_ind = l->in_index();
    auto & ret = _routing_memo.at(l_ind*4*MAX_SUBNETS + slot*4 + (31-__builtin_clz(bitwidth)));
    assert(is_output() || ret.size()!=0);
    return ret;
  }


protected:
  int _ID = -1;
  int _x=-1, _y=-1; //just for visualization

  int _max_util = 1; // Convert from "share_slot_size"
  bool _flow_control = true; // whether PE supports backpressure, convert from "flow_control"
  int _bitwidth = 64;  // maximum bitwidth of PE, convert from "bit_width"

  sslink *_cycle_link=nullptr; //to

  std::vector<sslink *> _in_links; //Incomming and outgoing links
  std::vector<sslink *> _out_links;

  // This table stores the subnetwork routing
  // [output][slot][input][slot]
  std::vector<std::vector<std::vector<std::vector<bool>>>> _subnet_table; //convert from subnet_table

  // Maps [input_link_id][slot_number][bitwidth] -> vector of legal links/slots
  std::vector<std::vector<std::pair<int,sslink*>>> _routing_memo;

  //Variables used for scheduling -- these should be moved out at some point (TODO)
  int _node_dist[8];
  int _done[8];
  std::pair<int, sslink*>_came_from[8];
  
  // Decomposability
  // TODO:Please replace 8 with mf_decomposer 
  int decomposer; // convert from decomposer // to be integrate with subnet_table
  int mf_decomposer = 8;
  //TODO: most-fine-grain decomposer, to be removed or need to be defined by user
  
private:
  friend class SubModel;
};
    
class ssswitch : public ssnode {
public:

  ssswitch() : ssnode() {}

  virtual std::string name() const {
    std::stringstream ss;
    ss << "SW" << "_" << _x << "_" << _y;
    return ss.str();
  }

  virtual std::string gams_name(int config) const {
    std::stringstream ss;
    if (config != 0) {
      ss << "Sw" << _x << _y << "c" << config;
    } else {
      ss << "Sw" << _x << _y;
    }
    return ss.str();
  }

  void set_prop(boost::property_tree::ptree prop){
    set_ssnode_prop(prop);
  }

  void collect_features(){
    features[0] = _max_util > 1 ? 0.0 : 1.0;
    features[1] = _max_util > 1 ? 1.0 : 0.0;
    assert(features[0] || features[1]);
    features[2] = !_flow_control ? 1.0:0.0;
    features[3] = _flow_control ?1.0:0.0;
    assert((features[2] || features[3]) && "Either Data(Static) or DataValidReady(Dynamic)");
    features[4] = decomposer;
    assert(!(decomposer == 0) && !(decomposer & (decomposer - 1))&&"Decomposer need to be power of two");
    features[5] = max_fifo_depth;
    features[6] = _in_links.size();
    features[7] = _out_links.size();
    features[8] = _max_util;
  }

  double get_area(){
    collect_features();
    return router_area_predict(features);
  }
  double get_power(){
    collect_features();
    return router_power_predict(features);
  }

protected:
  double features[9];
  int max_fifo_depth;
};
    
class ssfu : public ssnode {
public:

  ssfu() : ssnode() {}

  void setFUDef(func_unit_def *fu_def) { _fu_def = fu_def; }

  void set_prop(boost::property_tree::ptree prop){
    set_ssnode_prop(prop);
    /*
    // Default
    YAML::Node default_setting = prop["<<"];
    // delay_fifo_depth 
    if(default_setting["delay_fifo_depth"] || prop["delay_fifo_depth"])
    try{
      delay_fifo_depth = prop["delay_fifo_depth"].as<int>();
    }catch(...){
      delay_fifo_depth = default_setting["delay_fifo_depth"].as<int>();
    }
    // output_select_mode 
    if(default_setting["output_select_mode"] || prop["output_select_mode"])
    try{
      output_select_mode = prop["output_select_mode"].as<std::string>();
    }catch(...){
      output_select_mode = default_setting["output_select_mode"].as<std::string>();
    }
    // register_file_size 
    if(default_setting["register_file_size"] || prop["register_file_size"])
    try{
      register_file_size = prop["register_file_size"].as<int>();
    }catch(...){
      register_file_size = default_setting["register_file_size"].as<int>();
    }
    */
  }

  virtual std::string name() const {
    std::stringstream ss;
    ss << "FU" << _x << "_" << _y;
    return ss.str();
  }

  virtual std::string gams_name(int config) const {
    std::stringstream ss;
    if (config != 0) {
      ss << "Fu" << _x << _y << "c" << config;
    } else {
      ss << "Fu" << _x << _y;
    }
    return ss.str();
  }

  double* collect_features(){
    features[0] = _max_util > 1 ? 0.0 : 1.0;
    features[1] = _max_util > 1 ? 1.0 : 0.0;
    assert(features[0] || features[1]);
    features[2] = !_flow_control ? 1.0:0.0;
    features[3] = _flow_control ? 1.0:0.0;
    assert((features[2] || features[3]) && "Either Data(Static) or DataValidReady(Dynamic)");
    features[4] = output_select_mode=="Individual" ? 1.0:0.0;
    features[5] = output_select_mode=="Universal" ? 1.0:0.0;
    assert((features[4] || features[5]) && "Either Individual or Universal");
    features[6] = decomposer;
    assert(!(decomposer == 0) && !(decomposer & (decomposer - 1))&&"Decomposer need to be power of two");
    features[7] = delay_fifo_depth;
    features[8] = _in_links.size();
    features[9] = _out_links.size();
    features[10] = register_file_size;
    features[11] = _max_util;
    return features;
  }

  double get_area(){
    collect_features();
    return pe_area_predict(features);
  }

  double get_power(){
    collect_features();
    return pe_power_predict(features);
  }


  func_unit_def *fu_def() { return _fu_def; }

protected:
  func_unit_def *_fu_def;
  double features[12];
  std::string output_select_mode;
  int delay_fifo_depth;
  int register_file_size;
  std::vector<int> max_delay_depth;

private:
  friend class SubModel;
};

//This should be improved later
class ssvport : public ssnode {
public:
  ssvport(int port_num) : _port(port_num) {}

  std::vector<int>& port_vec() {return _port_vec;}
  void set_port_vec(std::vector<int> p) {_port_vec=p;}
  size_t size() {return _port_vec.size();}
  void set_prop(YAML::Node prop){
    YAML::Node default_set = prop["<<"];
    if(default_set["channel_buffer"]||prop["channel_buffer"])
    try{
      channel_buffer = prop["channel_buffer"].as<int>();
    }catch(...){
      channel_buffer = default_set["channel_buffer"].as<int>();
    }
    if(default_set["io_type"]||prop["io_type"])
    try{
      io_type = prop["io_type"].as<std::string>();
    }catch(...){
      io_type = default_set["io_type"].as<std::string>();
    }
  }
  virtual std::string name() const {
    std::stringstream ss;
    if(_out_links.size()>0) ss << "I";
    else ss << "O";
    ss<< _port; 
    return ss.str(); 
  }

  virtual std::string gams_name(int i) const {
    return name();
  }

  int output_bitwidth() {
    int bitwidth = 0; 
    for(auto link : _in_links) {
      bitwidth += link->bitwidth();
    }
    return bitwidth;
  }

  int input_bitwidth() {
    int bitwidth = 0; 
    for(auto link : _out_links) {
      bitwidth += link->bitwidth();
    }
    return bitwidth;
  }

  void set_port2node(std::string portname,ssnode * node){
    port2node[portname] = node;
  }
  ssnode * convert_port2node(std::string portname){
    return port2node[portname];
  }
  int port() {return _port;}
  virtual bool is_input() {return _in_links.size()==0;}
  virtual bool is_output() {return _out_links.size()==0;}

private:
  int _port = -1;
  std::vector<int> _port_vec;
  std::string io_type;
  int channel_buffer;
  std::map<std::string,ssnode * > port2node;
};

class SubModel {
public:

  //Port type of the substrate nodes
  //opensp -- dyser opensplyser N + N -1 ips
  //three ins -- Softbrain 3 x N
  //everywitch -- all switches has ops and ips
  enum class PortType {
    opensp, everysw, threein, threetwo
  };

  SubModel() {}

  SubModel(std::istream &istream, FuModel *, bool multi_config = true);

  SubModel(int x, int y, PortType pt = PortType::opensp, int ips = 2, int ops = 2, bool multi_config = true);

  void PrintGraphviz(std::ostream &ofs);

  template<int is_input, typename T> void PrintGamsIO(std::ostream &os);

  void PrintGamsModel(std::ostream &ofs,
                      std::unordered_map<std::string, std::pair<ssnode *, int> > &,
                      std::unordered_map<std::string, std::pair<sslink *, int> > &,
                      std::unordered_map<std::string, std::pair<ssswitch *, int> > &,
                      std::unordered_map<std::string, std::pair<bool, int>> &,  /*isInput, port*/
                      int n_configs = 1);

  int sizex() { return _sizex; }

  int sizey() { return _sizey; }

  ssswitch *switchAt(int x, int y) { return _switches[x][y]; }

  std::vector<std::vector<ssfu*> > &fus() { return _fus; }

  template<typename T> std::vector<T> &nodes();

  std::vector<ssfu* > &fu_list() { return _fu_list; }

  std::vector<ssswitch* > &switch_list() { return _switch_list; }

  std::vector<std::vector<ssswitch*> > &switches() { return _switches; }

  bool multi_config() { return _multi_config; }

  size_t num_fu() { return _fus.size(); }

  void parse_io(std::istream &istream);

  ssio_interface &io_interf() { return _ssio_interf; }

  void clear_all_runtime_vals();

  const std::vector<sslink *> &link_list() { return _link_list; }

  const std::vector<ssnode *> &node_list() { return _node_list; }

  const std::vector<ssvport *> &input_list() { return _input_list; }

  const std::vector<ssvport *> &output_list() { return _output_list; }



  void add_input(int i, ssnode* n) {
    _io_map[true][i]=n;
  }
  void add_output(int i, ssnode* n) {
    _io_map[false][i]=n;
  }


  ssfu* add_fu(){
    auto * fu = new ssfu();
    _fu_list.push_back(fu);
    fu -> setFUDef(nullptr);
    return fu;
  }

  ssfu* add_fu(int x, int y) {
    auto * fu = new ssfu();
    if(x >= (int)_fus.size()) _fus.resize(x+1);
    if(y >= (int)_fus[x].size()) _fus[x].resize(y+1);
    _fus[x][y]=fu;
    _fu_list.push_back(fu);
    fu->setXY(x,y);
    fu->setFUDef(nullptr);
    return fu;
  }

  ssswitch* add_switch() {
    auto* sw = new ssswitch();
    _switch_list.push_back(sw);
    return sw;
  }

  ssswitch* add_switch(int x, int y) {
    auto* sw = new ssswitch();
    if(x >= (int)_switches.size()) _switches.resize(x+1);
    if(y >= (int)_switches[x].size()) _switches[x].resize(y+1);
    _switches[x][y]=sw;
    _switch_list.push_back(sw);
    sw->setXY(x,y);
    return sw;
  }

  ssvport* add_vport(bool is_input, int port_num) {
    auto vport = new ssvport(port_num);
    if(_ssio_interf.vports_map[is_input].count(port_num)) {
      std::cout << "Error: Multiple " << (is_input ? "input" : "output") 
                << " ports with port number " << port_num << "created\n\n";
      assert(0 && "port duplication error");
    }
    _ssio_interf.vports_map[is_input][port_num] = vport;
    _node_list.push_back(vport);
    _vport_list.push_back(vport);
    if(is_input) _input_list.push_back(vport);
    else         _output_list.push_back(vport); 
    return vport;
  }

  //This function should be called when the entire topology complete
  void post_process(); 

private:
  void build_substrate(int x, int y);

  void connect_substrate(int x, int y, PortType pt, int ips, int ops, 
      bool multi_config, int temp_x, int temp_y, int temp_width, 
      int temp_height, int skip_hv_dist, int skip_diag_dist, int skip_delay);

  //temporary datastructure to help construct mapping
  std::map<int, ssnode*> _io_map[2];

  int _sizex, _sizey;  //size of SS cgra
  bool _multi_config;
  std::vector<std::vector<ssfu*> > _fus;
  std::vector<std::vector<ssswitch*> > _switches;

  //These are only valid after regroup_vecs()
  std::vector<ssnode *> _node_list;
  std::vector<sslink *> _link_list;
  std::vector<ssvport *> _vport_list; 
  std::vector<ssvport *> _input_list;
  std::vector<ssvport *> _output_list;

  std::vector<ssfu *> _fu_list;
  std::vector<ssswitch *> _switch_list;

  ssio_interface _ssio_interf;
};

}

#endif
