#include "model.h"
#include <assert.h>
#include <math.h>
#include <yaml-cpp/yaml.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include "model_parsing.h"
#include "ssinst.h"

namespace pt = boost::property_tree;

using namespace std;
using namespace SS_CONFIG;

void SSModel::printGamsKinds(ostream& os) {
  os << "set K \"Type of Node\" /Input,Output";

  for (int i = 2; i < SS_NUM_TYPES; ++i) {
    os << "," << name_of_inst((ss_inst_t)i);
  }
  os << "/";
}

SSModel::SSModel(SubModel* subModel, bool multi_config) {
  if (subModel) {
    _subModel = subModel;
  } else {
    _subModel = new SubModel(5, 5, SubModel::PortType::everysw, multi_config);
  }
}

SSModel::SSModel(bool multi_config) {
  _subModel = new SubModel(5, 5, SubModel::PortType::everysw, multi_config);
}

void SSModel::parse_exec(std::istream& istream) {
  string param, value;
  while (istream.good()) {
    if (istream.peek() == '[') break;  // break out if done

    ModelParsing::ReadPair(istream, param, value);

    ModelParsing::trim(param);
    ModelParsing::trim(value);

    if (param.length() == 0) {
      continue;
    }

    if (param == string("CMD_DISPATCH")) {
      if (value == string("INORDER")) {
        set_dispatch_inorder(true);
      } else if (value == string("OOO")) {
        set_dispatch_inorder(false);
      } else {
        assert(0 && "Dispatch was not INORDER or OOO");
      }
    } else if (param == string("CMD_DISPATCH_WIDTH")) {
      istringstream(value) >> _dispatch_width;
    }
  }
}

// trim from start (in place)
static inline void ltrim(std::string& s) {
  s.erase(s.begin(),
          std::find_if(s.begin(), s.end(), [](int ch) { return !std::isspace(ch); }));
}

// trim from end (in place)
static inline void rtrim(std::string& s) {
  s.erase(
      std::find_if(s.rbegin(), s.rend(), [](int ch) { return !std::isspace(ch); }).base(),
      s.end());
}

// trim from both ends (in place)
static inline void trim(std::string& s) {
  ltrim(s);
  rtrim(s);
}
bool ends_with(string& s, string ending) {
  if (ending.size() > s.size()) return false;
  return std::equal(ending.rbegin(), ending.rend(), s.rbegin());
}
bool contains(std::string& s1, std::string s2) {
  return s1.find(s2) != std::string::npos;
}
std::vector<std::string> split(std::string raw_string, string delimiter) {
  vector<string> strings;
  size_t pos = 0;
  std::string token;
  while ((pos = raw_string.find(delimiter)) != std::string::npos) {
    token = raw_string.substr(0, pos);
    trim(token);
    strings.push_back(token);
    raw_string.erase(0, pos + delimiter.length());
  }
  strings.push_back(raw_string);
  return strings;
}

ssnode* if_isVector(ssnode* n, std::string portname) {
  // In case source is a vector port, get I/O node instead
  ssvport* s_vp = dynamic_cast<ssvport*>(n);
  if (s_vp != nullptr)
    return s_vp->convert_port2node(portname);
  else {
    return n;
  }
}

// File constructor
SSModel::SSModel(const char* filename, bool multi_config) {
  ifstream ifs(filename, ios::in);
  string param, value;
  bool failed_read = ifs.fail();
  if (failed_read) {
    cerr << "Could Not Open: " << filename << "\n";
    return;
  }

  // Parse the JSON-format IR
  string fn = string(filename);
  bool isJSON = ends_with(fn, string(".json"));
  bool isYAML = ends_with(fn, string(".yaml"));
  if (isJSON) {
    parse_json(ifs);
    return;
  }

  // Parse the YAML-format IR
  if (isYAML) {
    std::cout << "Start the YAML Parser\n";
    parse_yaml(fn);
    return;
  }

  char line[512];

  while (ifs.good()) {
    ifs.getline(line, 512);
    // string line;

    if (ModelParsing::StartsWith(line, "[exec-model]")) {
      parse_exec(ifs);
    }

    if (ModelParsing::StartsWith(line, "[fu-model]")) {
      _fuModel = new FuModel(ifs);
    }

    if (ModelParsing::StartsWith(line, "[sub-model]")) {
      if (_fuModel == nullptr) {
        cerr << "No Fu Model Specified\n";
        exit(1);
      }
      _subModel = new SubModel(ifs, _fuModel, multi_config);
    }

    if (ModelParsing::StartsWith(line, "[io-model]")) {
      if (_subModel == nullptr) {
        cerr << "No Sub Model Specified\n";
        exit(1);
      }

      _subModel->parse_io(ifs);
    }
  }
  _subModel->post_process();
}

// TODO YAML Parser (broken for now - Sihao will fix it later)
void SSModel::parse_yaml(const std::string& fn) {
  // TODO: fix when yaml-cpp library is able to parse Seq[Seq[bool]]

  assert(0 && "Yaml Parser is currently not working due to UNDEFINED Node Type");

  // Initialize SubModel
  /*
  _subModel = new SubModel();
  std::map<std::string,ssnode*> all_modules;

  // Read YAML File
  YAML::Node ssfabric = YAML::LoadFile(fn);

  // Set System Variables
  _fuModel = NULL;

  // TODO: Yaml is not working when parsing subnet_table
  //   inter_subnet_connection:
  //   - [true, true, false, false]
  //   - [false, false, true, true]
  // Cannot be Extracted
  YAML::Node ssnodes;
  if(ssfabric["nodes"]){
    ssnodes = ssfabric["nodes"];
    for (unsigned i = 0; i < ssnodes.size();i++){
      YAML::Node node = ssnodes[i];
      string nodeType= node["nodeType"].as<std::string>();
      cout << "Initialize " << nodeType << "\n" ;
      if(nodeType == "switch"){
        ssswitch* sw = _subModel -> add_switch();
        int id = node["id"].as<int>();
        sw -> set_prop(node);
      }
      if(nodeType == "function unit"){

      }
      if(nodeType == "vector port"){

      }
    }
  }

  // Instantiate Routers
  YAML::Node routers;
  if (ssfabric["routers"]) {
    routers = ssfabric["routers"];
    for(YAML::const_iterator it=routers.begin();it!=routers.end();++it) {
      // Get the name of router
      const std::string router_name = it->first.as<std::string>();
      // Skip the default
      if (router_name.rfind("default", 0) == 0) continue;
      // Extract the router properties
      YAML::Node router_properties = routers[router_name];
      ssswitch* sw = _subModel -> add_switch();
      sw -> set_name(router_name);
      sw -> set_prop(router_properties);
      all_modules[router_name] = sw;
      // Output for debug
      std::cout << "Initiatie " << router_name << "\n";
    }
  }

  // Inistantiate Processing Elements
  YAML::Node processing_elements;
  YAML::Node default_node;
  if(ssfabric["dedicated_pes"]) {
    processing_elements = ssfabric["dedicated_pes"];
    for (YAML::const_iterator it = processing_elements.begin();it !=
  processing_elements.end();++it){
      // Get the name of Processing Elements
      const std::string pe_name = it->first.as<std::string>();
      // Skip default
      if (pe_name.rfind("default",0) == 0) {
        default_node = processing_elements[pe_name];
        continue;
      }
      // Extract the Processing Properties
      YAML::Node pe_properties = processing_elements[pe_name];
      ssfu * fu = _subModel -> add_fu();
      fu -> set_name(pe_name);
      fu -> set_prop(pe_properties);
      // We can have a traditional shared processing element
      if(fu -> is_shared() ){
        fu -> set_max_util(64);
        fu -> add_link(fu);
      }
      // Get Instructions
      std::vector<std::string> insts;
      YAML::Node default_setting = pe_properties["<<"];

    if(default_setting["instructions"] || pe_properties["instructions"])
    try{
      insts = pe_properties["instructions"].as<std::vector<std::string>>();
    }catch(...){
      insts = default_setting["instructions"].as<std::vector<std::string>>();
    }

      // Set Function Unit Definition
      func_unit_def * fudef = new func_unit_def(pe_name);
      fu -> setFUDef(fudef);
      int encoding = 2; // 0 is taken by blank, 1 is taken by copy
      for (auto & inst_name : insts){
        ss_inst_t ss_inst = SS_CONFIG::inst_from_string(inst_name.c_str());
        fudef -> add_cap(ss_inst);
        fudef -> set_encoding(ss_inst,encoding++);
      }
      // Store in Name Lookup table
      all_modules[pe_name] = fu;
      // Test for Analytical Model
      fu -> collect_features();
      double area = fu -> get_area();
      std::cout << pe_name << "'s area = " << area <<" um^2\n";
      // Output for Debug
      std::cout << "Initiate " << pe_name << "\n";
    }
  }

  // Instantiate Triggered Processing Elements
  YAML::Node trig_processing_elements;
  if(ssfabric["shared_pes"]) {
    trig_processing_elements = ssfabric["shared_pes"];
    for(YAML::const_iterator it = trig_processing_elements.begin();it !=
  trig_processing_elements.end();++it){
      // Get the name of Triggered Processing Elements
      const std::string trig_pe_name = it->first.as<std::string>();
      // Skip Default
      if (trig_pe_name.rfind("default",0)==0) {
        default_node = trig_processing_elements[trig_pe_name];
        continue;
      }
      // Get Properties
      YAML::Node trig_pe_prop = trig_processing_elements[trig_pe_name];
      ssfu * fu = _subModel -> add_fu();
      fu -> set_name(trig_pe_name);
      fu -> set_prop(trig_pe_prop);
      // Triggered-Inst Processing Element is naturally Shared
      fu -> set_max_util(64);
      fu -> add_link(fu);
      // Get Instructions
      std::vector<std::string> insts;
      try{
        insts = trig_pe_prop["instructions"].as<vector<std::string>>();
      }catch(...){
        insts = default_node["instructions"].as<vector<std::string>>();
      }
      // Set Function Unit Definition
      func_unit_def * fudef = new func_unit_def(trig_pe_name);
      fu -> setFUDef(fudef);
      int encoding = 2; // 0 is taken by blank, 1 is taken by copy
      for (auto & inst_name : insts){
        ss_inst_t ss_inst = SS_CONFIG::inst_from_string(inst_name.c_str());
        fudef -> add_cap(ss_inst);
        fudef -> set_encoding(ss_inst,encoding++);
      }

      // Store in Name-Lookup Table
      all_modules[trig_pe_name] = fu;
      // Output for Debug
      std::cout << "Initiate " << trig_pe_name << "\n";
    }
  }

  // Instatiate VectorPort
  YAML::Node vector_ports;
  auto & io = _subModel -> io_interf();
  // Num of I/O to the fabric net
  int num_inputs=0;
  int num_outputs=0;
  // Num of vector port
  int num_ivp=0;
  int num_ovp=0;
  if(ssfabric["vector_ports"]) {
    vector_ports = ssfabric["vector_ports"];
    for (YAML::const_iterator it = vector_ports.begin();it!=vector_ports.end();++it){
      // Get Name
      const std::string vp_name = it->first.as<std::string>();
      // Get default
      if (vp_name.rfind("default",0)==0){
        default_node = vector_ports[vp_name];
        continue;
      }
      // Get Properties
      YAML::Node vp_prop = vector_ports[vp_name];
      std::string io_type = vp_prop["io_type"].as<std::string>();
      bool is_input = io_type == "in";
      ssvport * vp;
      if(io_type == "in"){
        cout << "init in vport " << num_ivp <<"\n";
        vp = _subModel->add_vport(is_input,num_ivp++);
      }
      else if(io_type == "out"){
        cout << "init out vport " << num_ovp <<"\n";
        vp = _subModel->add_vport(is_input,num_ovp++);
      }
      vp -> set_prop(vp_prop); // Set Vector Port Specific Prop
      all_modules[vp_name] = vp;

      if (io_type == "in"){
        int port_idx = num_ivp;
        for(auto & output_port : vp -> out_links()){
          int input_node_idx = num_inputs ++;
          vp -> port_vec().push_back(input_node_idx);
        }
      }else if(io_type == "out"){
        int port_idx = num_ovp;
        for (auto & input_port : vp -> in_links()){
          int output_node_index = num_outputs ++;
          vp -> port_vec().push_back(output_node_index);
        }
      }else{
        assert(0 && "unknown vector port type");
      }
    }
  }else{
    assert(0 && "No Vector Port?");
  }

  // Connect Everything Up
  std::vector<std::string> topology;
  if(ssfabric["topology"]){
    topology = ssfabric["topology"].as<vector<std::string>>();
    for (auto & connection : topology){
      // Skip "this"
      ssnode * left_module;
      ssnode * right_module;
      if(contains(connection, "this")) continue;

      std::string sleft ; std::string sright;
      std::string left_module_name; std::string left_port_name;
      std::string right_module_name; std::string right_port_name;
      if(contains(connection,"->")){
        sleft = split(connection,"->")[0];
        sright = split(connection,"->")[1];
      }else if(contains(connection, "<-")){
        sleft = split(connection,"<-")[0];
        sright = split(connection,"<-")[1];
      }else if(contains(connection, "<->")){
        sleft = split(connection,"<->")[0];
        sright = split(connection,"<->")[1];
      }else{
        assert(0 && "No Such Connect Symbol");
      }

      left_module_name = split(sleft,".")[0];
      left_port_name = split(sleft,".")[1];
      right_module_name = split(sright,".")[0];
      right_port_name = split(sright,".")[1];

      left_module = all_modules[left_module_name];
      right_module = all_modules[right_module_name];

      if(contains(connection,"<->")){
        left_module -> add_link(right_module,left_port_name,right_port_name);
        right_module -> add_link(left_module,right_port_name,left_port_name);
      }else if(contains(connection, "<-")){
        right_module -> add_link(left_module,right_port_name,left_port_name);
      }else if(contains(connection, "->")){
        left_module -> add_link(right_module,left_port_name,right_port_name);
      }else{
        assert(0 && "No Such Connect Symbol");
      }
    }
  }else{
    assert(0 && "No Connection ?");
  }
  _subModel->post_process();
   assert(num_inputs>0);
  assert(num_outputs>0);

  cout << "YAML IR Parser finished\n";
  */
  return;
}

// JSON Format is flat format with all objects defined
void SSModel::parse_json(std::istream& istream) {
  pt::ptree root;
  read_json(istream, root);

  std::map<int, ssnode*> sym_tab;

  // Null Fu Model
  _fuModel = NULL;

  // Now create the submodel
  _subModel = new SubModel();

  // Row and Col number of Cgra
  int logical_rows = root.get<int>("numRow", 0);
  int logical_cols = root.get<int>("numCol", 0);
  printf("JSON Rows: %d, Cols %d\n", logical_rows, logical_cols);

  // Instruction Set of this Design
  std::map<ss_inst_t, int> inst_enc_map;
  for (auto& p : root.get_child("Instruction Set")) {
    std::string inst_name = p.first;
    int enc = p.second.get_value<int>();
    ss_inst_t ss_inst = SS_CONFIG::inst_from_string(inst_name.c_str());
    if (ss_inst == SS_NONE || ss_inst == SS_ERR) {
      cerr << "ERROR IN PARSING INSTRUCTION: \"" << inst_name << "\"\n";
      assert(0);
      return;
    }
    inst_enc_map[ss_inst] = enc;
  }

  // Vector Port
  auto& io = _subModel->io_interf();
  int num_ivp = 0;
  int num_ovp = 0;  // Count the number of vector port
  int num_inputs = 0;
  int num_outputs = 0;  // Calculate the num of Input and Output in the fabric
  // Look through all of the children of grid IR
  for (auto& p : root.get_child("nodes")) {
    // Get Node and its properties
    auto& node_def = p.second;
    int y = node_def.get<int>("row_idx", 0);
    int x = node_def.get<int>("col_idx", 0);
    // Construct it Identification
    string type = node_def.get<std::string>("nodeType", "");
    int id = node_def.get<int>("id", -1);

    // Initialize based on Type
    if (type == "switch") {
      ssswitch* sw = _subModel->add_switch(x, y);
      sw->set_prop(node_def);
      sym_tab[id] = sw;
    } else if (type == "function unit") {
      // Set Possible x,y for visualization
      ssfu* fu = _subModel->add_fu(x, y);
      fu -> set_prop(node_def); 
      auto link = fu->add_link(fu);  // For decomposability
      link->setdir(SwitchDir::IP0);

      sym_tab[id]=fu;
      auto& insts = node_def.get_child("instructions");
    
      //TODO: FIXME: Memory leak 
      func_unit_def* fudef = new func_unit_def("NA");
      fu->setFUDef(fudef);

      for (auto& inst : insts) {
        std::string inst_name = inst.second.get_value<std::string>();
        ss_inst_t ss_inst = SS_CONFIG::inst_from_string(inst_name.c_str());
        int enc = inst_enc_map[ss_inst];
        cout << "adding capability " << name_of_inst(ss_inst) << " to fu " << id << "\n";
        fudef->add_cap(ss_inst);
        fudef->set_encoding(ss_inst, enc);
      }
    } else if (type == "vector port") {
      bool is_input;
      // the number of input port of this vector port, input vector port has zero input port
      int in_vec_width = node_def.get_child("num_input").get_value<int>();
      // the number of output port of this vector port, output vector port has zero output port
      int out_vec_width = node_def.get_child("num_output").get_value<int>();

      // whether is a input/output vector port
      if(in_vec_width > 0){
        is_input = false; num_ovp++; num_outputs += in_vec_width;
      }else if(out_vec_width > 0){
        is_input = true; num_ivp++; num_inputs += out_vec_width;
      }else{
        assert(0 && "vector port without connection?");
      }

      int port_num = is_input ? num_ivp : num_ovp;

      cout << "new " << (is_input ? "input" : "output") << " port: \"" << id << "\" \n";

      ssvport * vp = _subModel->add_vport(is_input, port_num);
      vp -> set_ssnode_prop(node_def);
      sym_tab[id] = vp;

    } else {
      std::cerr << id << "has unknown type" << type << "\n";
      assert(0 && "unknown type");
    }
  }

  // Connect everything up
  for (auto& p : root.get_child("links")) {
    std::string elem_name = p.first;
    auto& p_def = p.second;

    int source_id;
    auto& source = p_def.get_child("source");
    int sink_id;
    auto& sink = p_def.get_child("sink");
    for (auto identifier : source) {
      source_id = identifier.second.get_value<int>();
      break;  // only take the first element (id) out
    }
    for (auto identifier : sink) {
      sink_id = identifier.second.get_value<int>();
      break;  // only take the first element (id) out
    }
    ssnode* from_module = sym_tab[source_id];
    ssnode* to_module = sym_tab[sink_id];
    assert(from_module && to_module);
    from_module->add_link(to_module);
    std::cout << "connect : " << from_module->nodeType() << "_" << from_module->id() << " --> ";
    std::cout << to_module-> nodeType() << "_" << to_module->id() << "\n";
  }

  _subModel->post_process();

  // Print out the Analytical Model for FU
  int id = 0;
  for (auto fu : _subModel -> fu_list()){
      // Test for Analytical Model
      std::cout << "fu " << id << "'s area = " << fu -> get_area() <<" um^2\n";
      std::cout << "fu " << id << "'s power = " << fu -> get_power() <<" mW\n";
      ++id;
  }

  // Print out the Switches Analytical Model Estimation
  id = 0;
  for (auto sw : _subModel -> switch_list()){
      // Test for Analytical Model
      std::cout << "switch " << id << "'s area = " << sw -> get_area() <<" um^2\n";
      std::cout << "switch " << id << "'s power = " << sw -> get_power() <<" mW\n";
      ++id;
  }

  std::cout << "Overall Area = " << _subModel -> get_overall_area() << " um^2\n";
  std::cout << "Overall Power = " << _subModel -> get_overall_power() << " mW\n";

  assert(num_outputs > 0);
  assert(num_inputs > 0);
}

extern "C" void libssconfig_is_present() {}
