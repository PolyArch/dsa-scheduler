#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdlib> 
#include <math.h>

#include <assert.h>

#include "model.h"
#include "model_parsing.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "ssinst.h"

#include <yaml-cpp/yaml.h>

namespace pt = boost::property_tree;


using namespace std;
using namespace SS_CONFIG;

void SSModel::printGamsKinds(ostream& os) {
  os << "set K \"Type of Node\" /Input,Output";
  
  for(int i = 2; i < SS_NUM_TYPES; ++i) {
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
    string param,value;
    while(istream.good()) {
        if(istream.peek()=='[') break;  //break out if done

        ModelParsing::ReadPair(istream,param,value);

        ModelParsing::trim(param);
        ModelParsing::trim(value);

        if(param.length()==0) {
          continue;
        }

        if(param == string("CMD_DISPATCH")) {
          if(value == string("INORDER")) {
            set_dispatch_inorder(true);
          } else if (value == string("OOO")) {
            set_dispatch_inorder(false);
          } else {
            assert(0 && "Dispatch was not INORDER or OOO");
          }
        } else if(param == string("CMD_DISPATCH_WIDTH")) {
            istringstream(value) >> _dispatch_width;
        }

    }
}

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}
bool ends_with(string& s, string ending) {
  if (ending.size() > s.size()) return false;
  return std::equal(ending.rbegin(), ending.rend(), s.rbegin());
}
bool contains(std::string & s1, std::string s2){
    return s1.find(s2) != std::string::npos;
  }
std::vector<std::string> split(std::string raw_string, string delimiter){
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

ssnode * if_isVector(ssnode * n, std::string portname){
  // In case source is a vector port, get I/O node instead
  ssvport * s_vp = dynamic_cast <ssvport*>(n);
  if(s_vp != nullptr)
    return s_vp -> convert_port2node(portname);
  else{
    return n;
  }
}

//File constructor
SSModel::SSModel(const char* filename, bool multi_config) {
    ifstream ifs(filename, ios::in);
    string param,value;
    bool failed_read = ifs.fail();
    if(failed_read)
    {
        cerr << "Could Not Open: " << filename << "\n";
        return;
    }
    
    // Parse the JSON-format IR
    string fn = string(filename);
    bool isJSON = ends_with(fn,string(".json"));
    bool isYAML = ends_with(fn,string(".yaml"));
    if(isJSON) {
      parse_json(ifs);
      return;
    }

    // Parse the YAML-format IR
    if(isYAML){
      std::cout << "Start the YAML Parser\n";
      parse_yaml(fn);
      return;
    }

    char line[512];
    
    while(ifs.good())
    {
        ifs.getline(line,512);
        //string line;

        if(ModelParsing::StartsWith(line,"[exec-model]")) {
          parse_exec(ifs);
        }

        if(ModelParsing::StartsWith(line,"[fu-model]")){
            _fuModel= new FuModel(ifs);
        }
        
        if(ModelParsing::StartsWith(line,"[sub-model]")){
            if(_fuModel==nullptr) { 
                cerr<< "No Fu Model Specified\n";
                exit(1);
            }
            _subModel=new SubModel(ifs, _fuModel, multi_config);
        }

        if(ModelParsing::StartsWith(line,"[io-model]")) {
            if(_subModel==nullptr) { 
                cerr<< "No Sub Model Specified\n";
                exit(1);
            }

            _subModel->parse_io(ifs);
        }
    }
    _subModel->post_process();
}

//YAML Parser
void SSModel::parse_yaml(const std::string& fn) {
  // Initialize SubModel
  _subModel = new SubModel();
  std::map<std::string,ssnode*> all_modules;

  // Read YAML File
  YAML::Node hw_desc = YAML::LoadFile(fn);
  
  // Set System Variables
  YAML::Node cgra_system;
  if (hw_desc["system"]) cgra_system = hw_desc["system"];
  _fuModel = NULL;

  // Instantiate Routers
  YAML::Node routers;
  if (hw_desc["routers"]) {
    routers = hw_desc["routers"];
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
      sw -> set_properties(router_properties);
      all_modules[router_name] = sw;
      // Output for debug
      std::cout << "Initiatie " << router_name << "\n";
    }
  }

  // Inistantiate Processing Elements
  YAML::Node processing_elements;
  YAML::Node default_node;
  if(hw_desc["dedicated_pes"]) {
    processing_elements = hw_desc["dedicated_pes"];
    for (YAML::const_iterator it = processing_elements.begin();it != processing_elements.end();++it){
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
      fu -> set_properties(pe_properties);
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
  if(hw_desc["shared_pes"]) {
    trig_processing_elements = hw_desc["shared_pes"];
    for(YAML::const_iterator it = trig_processing_elements.begin();it != trig_processing_elements.end();++it){
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
      fu -> set_properties(trig_pe_prop);
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
  if(hw_desc["vector_ports"]) {
    vector_ports = hw_desc["vector_ports"];
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
      vp -> set_properties(vp_prop); // Set Universal Prop
      vp -> set_prop(vp_prop); // Set Vector Port Specific Prop
      all_modules[vp_name] = vp;

      if (io_type == "in"){
        int port_idx = num_ivp;
        for(auto & output_port : vp -> get_output_ports()){
          int input_node_idx = num_inputs ++;
          vp -> port_vec().push_back(input_node_idx);
          //ssinput * in = _subModel -> add_input(input_node_idx);
          //vp -> set_port2node(output_port,in);
        }
      }else if(io_type == "out"){
        int port_idx = num_ovp;
        for (auto & input_port : vp -> get_input_ports()){
          int output_node_index = num_outputs ++;
          vp -> port_vec().push_back(output_node_index);
          //ssoutput * out = _subModel -> add_output(output_node_index);
          //vp -> set_port2node(input_port,out);
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
  if(hw_desc["topology"]){
    topology = hw_desc["topology"].as<vector<std::string>>();
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
  return;
}

//JSON Format is flat format with all objects defined
void SSModel::parse_json(std::istream& istream) {
  pt::ptree root;
  read_json(istream,root);

  std::map<std::string,ssnode*> sym_tab;

  //Null Fu Model
  _fuModel = NULL;

  //Now create the submodel
  _subModel=new SubModel();

  int logical_rows = root.get<int>("numRows", 0);
  int logical_cols = root.get<int>("numCols", 0);
  printf("JSON Rows: %d, Cols %d\n", logical_rows, logical_cols);


  std::map<int,ss_inst_t> inst_map;
  for(auto& p : root.get_child("ISAencode")) {
    std::string inst_name = p.first;
    int idx =  p.second.get_value<int>();

    if(inst_name=="numISA") continue;

    ss_inst_t ss_inst = SS_CONFIG::inst_from_string(inst_name.c_str());
        
    if(ss_inst==SS_NONE || ss_inst==SS_ERR) {
      cerr << "ERROR IN PARSING INSTRUCTION: \"" << inst_name << "\"\n";
      assert(0);
      return;
    }

    inst_map[idx] = ss_inst;
  }

  //Look through all of the children of grid IR
  for(auto& p : root.get_child("GridIR")) {
    std::string elem_name = p.first;
    auto& node_def = p.second;
    int y = node_def.get<int>("row", 0);
    int x = node_def.get<int>("col", 0);

    string type = node_def.get<std::string>("type", "");
    if(type=="Switch") {
       ssswitch* sw = _subModel->add_switch(x,y);
       sym_tab[elem_name]=sw;
    } else if(type=="FU") {
       ssfu* fu = _subModel->add_fu(x,y);

       auto link = fu->add_link(fu); //For decomposability
       link->setdir(SwitchDir::IP0);

       sym_tab[elem_name]=fu;
       auto& fu_def1 = node_def.get_child("Instructions");
       auto& fu_def2 = fu_def1.get_child("outPut_0");
    
       //TODO: FIXME: Memory leak 
       func_unit_def* fudef = new func_unit_def("NA");
       fu->setFUDef(fudef);

       for(auto& enc_def : fu_def2.get_child("subNet_0")) {
         int num = enc_def.second.get_value<int>();

         ss_inst_t inst = inst_map[num];
         cout << "adding capability " << name_of_inst(inst) << "to fu" << elem_name << "\n";

         fudef->add_cap(inst);
         fudef->set_encoding(inst,num);
       }
    } else {
      std::cerr << elem_name << "has unknown type" << type << "\n";
      assert(0&&"unknown type");
    }
  }

  auto& io = _subModel->io_interf();

  int num_inputs=0;
  int num_outputs=0;

  int num_ivp=0;
  int num_ovp=0;

  //Instantiate all the input and output ports
  for(auto& p : root.get_child("InterfacePort")) {
    std::string elem_name = p.first;
    auto& port_def = p.second;
    std::string type = port_def.get<std::string>("InOrOut", "");

    int is_input = type=="InputPorts";
    auto* vp = _subModel->add_vport(is_input,-1); //FIXME: port number
    sym_tab[elem_name]=vp;

    cout << "new port: \"" << elem_name << "\" \n";

    std::vector<ssnode*> nodes;
    for(auto& p : port_def.get_child("gridModules")) {
      string s = p.second.get_value<std::string>();
      ssnode* n = sym_tab[s];
      assert(n);
      nodes.push_back(n);
    }
 
    // TODO(@were): merge this
    if(type=="InputPorts") {
      int port_num = num_ivp++;
      io.vports_map[is_input][port_num]=vp;
      for(ssnode* n:nodes) {
        int node_id = num_inputs++;
        cout << "added input to vec: " << node_id << "\n";

        vp->port_vec().push_back(node_id);
        //ssinput* in = _subModel->add_input(node_id);  
        //in->add_link(n);
      }
    } else if(type=="OutputPorts") {
      int port_num = num_ovp++;
      io.vports_map[is_input][port_num]=vp;
      for(ssnode* n:nodes) {
        int node_id = num_outputs++;
        cout << "added output to vec: " << node_id << "\n";

        vp->port_vec().push_back(node_id);
        //ssoutput* out = _subModel->add_output(node_id);  
        //n->add_link(out);
      }
    } else {
      assert(0 && "unknown type");
    }
  }

  //Connect everything up
  for(auto& p : root.get_child("ConnectionIR")) {
    std::string elem_name = p.first;
    auto& p_def = p.second;
    string from_str = p_def.get<std::string>("fromModule", "");
    string to_str   = p_def.get<std::string>("toModule", "");

    ssnode* from_module = sym_tab[from_str]; 
    ssnode* to_module   = sym_tab[to_str]; 
    assert(from_module && to_module);

    from_module->add_link(to_module);
    
  }

  _subModel->post_process();

  assert(num_inputs>0);
  assert(num_outputs>0);
}






extern "C" void libssconfig_is_present() {}

