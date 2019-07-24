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

namespace SS_CONFIG {

class ssnode;
class ssinput;
class ssoutput;
class ssvport;

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

  void sort_in_vports(std::vector<std::pair<int,int>>& portID2size) {
    sort(portID2size, vports_map[1]);
  }
  
  void sort_out_vports(std::vector<std::pair<int,int>>& portID2size) {
    sort(portID2size, vports_map[0]);
  }

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
  sslink(ssnode *source, ssnode *sink,
        std::string source_port, std::string sink_port){
          _orig = source;
          _dest = sink;
          _source_port = source_port;
          _sink_port = sink_port;
        }

  sslink* getCycleLink();

  std::string name() const;

  std::string gams_name(int config) const;

  std::string gams_name(int, int) const;

  int id() { return _ID; }

  void set_id(int id) { _ID = id; }

  int max_util() { return _max_util; }

  int set_max_util(int m) { return _max_util = m; }

  void set_lat(int i) {_lat=i;}
  int lat() {return _lat;}

protected:
  int _ID = -1;
  int _max_util = 1; // by default, assume its a dedicated link
  int _lat=1;

  ssnode *_orig;
  ssnode *_dest;
  SwitchDir::DIR _dir;
  std::string _source_port; // Sihao Added 
  std::string _sink_port;  // Sihao Added

private:
  friend class SubModel;
};
    
    
class ssnode {
public:
  ssnode() {}

  sslink *add_link(ssnode *node) {
    sslink *link = new sslink(this, node);
    _out_links.push_back(link);
    node->add_back_link(link);
    return link;
  }
  sslink * add_link(ssnode *sink_n,std::string source_port, std::string sink_port){
    ssnode * source_node;
    ssnode * sink_node;

    source_node = this;
    sink_node = sink_n;
    
    sslink * link = new sslink(source_node, sink_node, source_port, sink_port);
    _out_links.push_back(link);
    sink_node -> add_back_link(link);
    return link;
  }

  void add_back_link(sslink *link) {
    _in_links.push_back(link);
  }

  virtual std::string name() const {
    return std::string("loadslice");
  }

  virtual std::string gams_name(int config = 0) const {
    return std::string("loadslice");
  }

  typedef std::vector<sslink *>::const_iterator const_iterator;

  const std::vector<sslink *> &in_links() { return _in_links; }

  const std::vector<sslink *> &out_links() { return _out_links; }

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
    for (auto &dlink: out_links()) {
      if (dlink->dest() == this) {
        return dlink;
      }
    }
    return nullptr;
  }

  std::vector<std::string> get_output_ports(){
    return output_ports;
  }

  std::vector<std::string> get_input_ports(){
    return input_ports;
  }

  int id() { return _ID; }

  void set_properties(YAML::Node prop){
    // Default
    YAML::Node default_setting = prop["<<"];

    // Module Type
    if(default_setting["module_type"] || prop["module_type"])
    try{
      module_type = prop["module_type"].as<std::string>();
    }catch(...){
      module_type = default_setting["module_type"].as<std::string>();
    }
    // I/O
    // Config Input Port
    if(default_setting["config_input_port"] || prop["config_input_port"])
    try{
      config_input_port = prop["config_input_port"].as<std::string>();
    }catch(...){
      config_input_port = default_setting["config_input_port"].as<std::string>();
    }
    // Config Output Port
    if(default_setting["config_output_port"] || prop["config_output_port"])
    try{
      config_output_port = prop["config_output_port"].as<std::string>();
    }catch(...){
      config_output_port = default_setting["config_output_port"].as<std::string>();
    }
    // Input Ports
    if(default_setting["input_ports"] || prop["input_ports"])
    try{
      input_ports = prop["input_ports"].as<std::vector<std::string>>();
    }catch(...){
      input_ports = default_setting["input_ports"].as<std::vector<std::string>>();
    }
    
    // Fuck Insts
    std::vector<std::string> insts;
    if(default_setting["instructions"] || prop["instructions"])
    try{
      insts = prop["instructions"].as<std::vector<std::string>>();
    }catch(...){
      insts = default_setting["instructions"].as<std::vector<std::string>>();
    }
    
    
    // Output Ports
    if(default_setting["output_ports"] || prop["output_ports"])
    try{
      output_ports = prop["output_ports"].as<std::vector<std::string>>();
    }catch(...){
      output_ports = default_setting["output_ports"].as<std::vector<std::string>>();
    }
    // decomposer 
    if(default_setting["decomposer"] || prop["decomposer"])
    try{
      decomposer = prop["decomposer"].as<int>();
    }catch(...){
      decomposer = default_setting["decomposer"].as<int>();
    }
    // Shared / Dedicated
    if(default_setting["isShared"] || prop["isShared"])
    try{
      isShared = prop["isShared"].as<bool>();
    }catch(...){
      isShared = default_setting["isShared"].as<bool>();
    }
    // Shared Slot Size
    if(default_setting["shared_slot_size"]||prop["shared_slot_size"])
    try{
      shared_slot_size = prop["shared_slot_size"].as<int>();
    }catch(...){
      shared_slot_size = default_setting["shared_slot_size"].as<int>();
    }
    // Static / Dynamic
    if(default_setting["protocol"]||prop["protocol"])
    try{
      protocol = prop["protocol"].as<std::string>();
    }catch(...){
      protocol = default_setting["protocol"].as<std::string>();
    }
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

  bool is_shared(){ return isShared;}

  void set_name(std::string na){module_name = na;}

  std::string get_name(){return module_name;}

protected:
  int _ID = -1;
  std::string module_name;
  std::string module_type;
  int _node_dist[8];
  int _done[8];
  std::pair<int, sslink*>_came_from[8];

  int _max_util = 1; // by default, assume its a dedicated link
  std::vector<sslink *> _in_links;
  std::vector<sslink *> _out_links;

  //Sihao 
  // I/O
  std::string config_input_port;
  std::string config_output_port;
  std::vector<std::string> input_ports;
  std::vector<std::string> output_ports;
  // Decomposability
  int decomposer;
  // Shared / Dedicated
  bool isShared;
  int shared_slot_size;
  // Static / Dynamic
  std::string protocol;

private:
  friend class SubModel;
};
    
class ssswitch : public ssnode {
public:

  ssswitch() : ssnode() {}

  void setXY(int x, int y) {
    _x = x;
    _y = y;
  }

  int x() const { return _x; }

  int y() const { return _y; }

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

  void set_prop(YAML::Node prop){
    // Default
    YAML::Node default_setting = prop["<<"];
    // back_pressure_fifo_depth 
    if(default_setting["back_pressure_fifo_depth "] || prop["back_pressure_fifo_depth"])
    try{
      back_pressure_fifo_depth = prop["back_pressure_fifo_depth"].as<int>();
    }catch(...){
      back_pressure_fifo_depth = default_setting["back_pressure_fifo_depth"].as<int>();
    }
  }

  void collect_features(){
    features[0] = isShared ? 0.0 : 1.0;
    features[1] = isShared ? 1.0 : 0.0;
    assert(features[0] || features[1]);
    features[2] = protocol == "Data" ? 1.0:0.0;
    features[3] = protocol == "DataValidReady" ?1.0:0.0;
    assert((features[2] || features[3]) && "Either Data(Static) or DataValidReady(Dynamic)");
    features[4] = decomposer;
    assert(!(decomposer == 0) && !(decomposer & (decomposer - 1))&&"Decomposer need to be power of two");
    features[5] = back_pressure_fifo_depth;
    features[6] = input_ports.size();
    features[7] = output_ports.size();
    features[8] = shared_slot_size;
  }

  double get_area(){
    collect_features();
    return router_area_predict(features);
  }
  double get_power(){
    collect_features();
    return router_power_predict(features);
  }

  ssinput *getInput(int i);

  ssoutput *getOutput(int i);

protected:
  int _x, _y;
  double features[9];
  int back_pressure_fifo_depth;
};
    
class ssfu : public ssnode {
public:

  ssfu() : ssnode() {}

  void setFUDef(func_unit_def *fu_def) { _fu_def = fu_def; }

  void setXY(int x, int y) {
    _x = x;
    _y = y;
  }

  int x() const { return _x; }

  int y() const { return _y; }

  void set_prop(YAML::Node prop){
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
  }

  virtual std::string name() const {
    std::stringstream ss;
    ss << "FU" << "_" << _x << "_" << _y;
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
    features[0] = isShared ? 0.0 : 1.0;
    features[1] = isShared ? 1.0 : 0.0;
    assert(features[0] || features[1]);
    features[2] = protocol == "Data" ? 1.0:0.0;
    features[3] = protocol == "DataValidReady" ?1.0:0.0;
    assert((features[2] || features[3]) && "Either Data(Static) or DataValidReady(Dynamic)");
    features[4] = output_select_mode=="Individual" ? 1.0:0.0;
    features[5] = output_select_mode=="Universal" ? 1.0:0.0;
    assert((features[4] || features[5]) && "Either Individual or Universal");
    features[6] = decomposer;
    assert(!(decomposer == 0) && !(decomposer & (decomposer - 1))&&"Decomposer need to be power of two");
    features[7] = delay_fifo_depth;
    features[8] = input_ports.size();
    features[9] = output_ports.size();
    features[10] = register_file_size;
    features[11] = shared_slot_size;
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
  int _x, _y;
  func_unit_def *_fu_def;
  double features[12];
  std::string output_select_mode;
  int delay_fifo_depth;
  int register_file_size;


private:
  friend class SubModel;
};

class ssinput : public ssnode { 
    public:
    
    ssinput() : ssnode() {}
      
    void setPort(int port) {_port=port;}
    int port() const {return _port;}
    
    std::string name() const {
        std::stringstream ss;
        ss << "IP" << "_" << _port;
        return ss.str();
    }
    std::string gams_name(int config) const {
        std::stringstream ss;
        if(config!=0) {
          ss << "I" << _port << "c" << config;
        } else {
          ss << "I" << _port;
        }
        return ss.str();
    }
    
    protected:
      int fifo_depth;
      int _port;
};  

class ssoutput : public ssnode {
    public:
    ssoutput() : ssnode() {}
      
    void setPort(int port) {_port=port;}
    int port() const {return _port;}
    
    std::string name() const {
        std::stringstream ss;
        ss << "OP" << "_" << _port;
        return ss.str();
    }
    
    std::string gams_name(int config) const {
        std::stringstream ss;
        if(config!=0) {
          ss << "O" << _port << "i" << config;
        } else {
          ss << "O" << _port;
        }
        return ss.str();
    }
    
    protected:
      int fifo_depth;
      int _port;
};

//This should be improved later
class ssvport : public ssnode {
public:
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
  void set_port2node(std::string portname,ssnode * node){
    port2node[portname] = node;
  }
  ssnode * convert_port2node(std::string portname){
    return port2node[portname];
  }

private:
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

  typedef std::vector<ssinput>::const_iterator const_input_iterator;
  typedef std::vector<ssoutput>::const_iterator const_output_iterator;

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

  std::vector<ssinput*> &inputs() { return _inputs; }

  std::vector<ssoutput*> &outputs() { return _outputs; }

  std::vector<std::vector<ssfu*> > &fus() { return _fus; }

  template<typename T> std::vector<T> &nodes();

  std::vector<ssfu* > &fu_list() { return _fu_list; }

  std::vector<ssswitch* > &switch_list() { return _switch_list; }

  std::vector<std::vector<ssswitch*> > &switches() { return _switches; }

  bool multi_config() { return _multi_config; }

  ssswitch *cross_switch() { return &_cross_switch; }

  ssnode *load_slice() { return &_load_slice; }

  size_t num_fu() { return _fus.size(); }

  void parse_io(std::istream &istream);

  ssio_interface &io_interf() { return _ssio_interf; }

  void clear_all_runtime_vals();

  void clear_fu_runtime_vals();

  const std::vector<sslink *> &link_list() { return _link_list; }

  const std::vector<ssnode *> &node_list() { return _node_list; }

  void add_inputs(int n) {
    _inputs.resize(n);
    //Port num to each switch
    for(unsigned i = 0; i < _inputs.size(); ++i) {
      add_input(i);
    }
  }

  void add_outputs(int n) {
    _outputs.resize(n);
    //Port num to each switch
    for(unsigned i = 0; i < _outputs.size(); ++i) {
      add_output(i);
    }
  }

  ssinput* add_input(int i) {
    auto* in = new ssinput();
    if(i >= (int)_inputs.size()) _inputs.resize(i+1);
    assert(_inputs[i]==NULL);
    _inputs[i]=in;
    in->setPort(i);
    return in;
  }

  ssoutput* add_output(int i) {
    auto* out = new ssoutput();
    if(i >= (int)_outputs.size()) _outputs.resize(i+1);
    assert(_outputs[i]==NULL);
    _outputs[i]=out;
    out->setPort(i);
    return out;
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

  void regroup_vecs(); //fills in the linear lists

private:
  //void CreateFUArray(int,int);
  //void SetTotalFUByRatio();
  //void RandDistributeFUs();
  void build_substrate(int x, int y);

  void connect_substrate(int x, int y, PortType pt, int ips, int ops, bool multi_config, int temp_x, int temp_y,
                         int temp_width, int temp_height, int skip_hv_dist, int skip_diag_dist, int skip_delay);


  int _sizex, _sizey;  //size of SS cgra
  bool _multi_config;
  std::vector<ssinput*> _inputs;
  std::vector<ssoutput*> _outputs;
  std::vector<std::vector<ssfu*> > _fus;
  std::vector<std::vector<ssswitch*> > _switches;

  //These are only valid after regroup_vecs()
  std::vector<ssnode *> _io_list;
  std::vector<ssnode *> _node_list;
  std::vector<sslink *> _link_list;

  std::vector<ssfu *> _fu_list;
  std::vector<ssswitch *> _switch_list;

  ssswitch _cross_switch;
  ssnode _load_slice;
  ssio_interface _ssio_interf;
};

}

#endif
