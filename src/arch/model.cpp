
#include <assert.h>
#include <math.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

#include "../utils/model_parsing.h"
#include "dsa/arch/ssinst.h"
#include "dsa/arch/model.h"
#include "dsa/debug.h"

#include "json.lex.h"
#include "json.tab.h"
#include "json/visitor.h"

namespace pt = boost::property_tree;

using namespace std;
using namespace dsa;

SSModel::SSModel(SubModel* subModel) {
  CHECK(subModel);
  _subModel = subModel;
}

void SSModel::setMaxEdgeDelay(int d) {
  for (auto* fu : _subModel->fu_list()) {
    fu->set_delay_fifo_depth(d);
  }
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
SSModel::SSModel(const char* filename_) : filename(filename_) {
  ifstream ifs(filename, ios::in);
  string param, value;
  bool failed_read = ifs.fail();
  if (failed_read) {
    cerr << "Could Not Open: " << filename << "\n";
    return;
  }

  // Parse the JSON-format IR
  bool isJSON = ModelParsing::EndsWith(filename, ".json");
  if (isJSON) {
    //dsa::SubModel * _subModel = new SubModel();
    _subModel -> parse_json_without_boost(filename);
    //parse_json(ifs);
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
      this->fu_types = ParseFuType(ifs);
    }

    if (ModelParsing::StartsWith(line, "[sub-model]")) {
      if (fu_types.empty()) {
        cerr << "No Fu Model Specified\n";
        exit(1);
      }
      _subModel = new SubModel(ifs, fu_types);
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


/*
// JSON Format is flat format with all objects defined
void SSModel::parse_json(std::istream& istream) {
  pt::ptree root;
  read_json(istream, root);

  std::map<int, ssnode*> sym_tab;

  // Now create the submodel
  _subModel = new SubModel();

  // Vector Port
  int num_ivp = 0;
  int num_ovp = 0;  // Count the number of vector port
  int num_inputs = 0;
  int num_outputs = 0;  // Calculate the num of Input and Output in the fabric
  // Look through all of the children of grid IR
  for (auto& p : root.get_child("nodes")) {
    // Get Node and its properties
    auto& node_def = p.second;
    // Construct it Identification
    string type = node_def.get<std::string>("nodeType", "");
    int id = node_def.get<int>("id", -1);

    // Initialize based on Type
    if (type == "switch") {
      ssswitch* sw = _subModel->add_switch();
      sw->set_id(id);
      sw->set_prop(node_def);
      sym_tab[id] = sw;
    } else if (type == "function unit") {
      // Set Possible x,y for visualization
      ssfu* fu = _subModel->add_fu();
      fu->set_id(id);
      fu->set_prop(node_def);

      sym_tab[id] = fu;
      auto& insts = node_def.get_child("instructions");

      stringstream fudef_name;
      fudef_name << "function unit_" << id;
      fu_types.push_back(new Capability(fudef_name.str()));
      auto fu_type = fu_types.back();
      
      int enc = 2; // the initial encoding for opcode is 0 (usual for PASS)
      for (auto& inst : insts) {
        std::string inst_name = inst.second.get_value<std::string>();
        OpCode ss_inst = dsa::inst_from_string(inst_name.c_str());
        fu_type->Add(ss_inst, enc++);
      }
      fu->fu_type_ = *fu_type;

      // debug
      
      std::cout << fu ->name() << " can perform : ";
      for(auto capable_ssinst : fu->fu_type_.capability){
        std::cout << SS_CONFIG::name_of_inst(capable_ssinst.op) << ", ";
      }
      
    } else if (type == "vector port") {
      bool is_input = false;
      // the number of input port of this vector port, input vector port has zero input
      // port
      int in_vec_width = node_def.get_child("num_input").get_value<int>();
      // the number of output port of this vector port, output vector port has zero output
      // port
      int out_vec_width = node_def.get_child("num_output").get_value<int>();

      // Sihao: I think it is the wrong way to get port num
      //int port_num = node_def.get_child("port").get_value<int>();
      int port_num = -1;

      // whether is a input/output vector port
      if (in_vec_width > 0) {
        is_input = false;
        port_num = num_ovp++;
        num_outputs += in_vec_width;
        std::cout << "node " << id << " is output vector port\n";
      } else if (out_vec_width > 0) {
        is_input = true;
        port_num = num_ivp++;
        num_inputs += out_vec_width;
        std::cout << "node " << id << " is input vector port\n";
      } else {
        continue;
        //assert(0 && "vector port without connection?");
      }

      ssvport* vp = _subModel->add_vport(is_input, port_num);
      vp->set_ssnode_prop(node_def);
      sym_tab[id] = vp;
      vp->set_id(id);

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
    
    // Sihao @jian: Why this way of `add_link` not works ?
    //_subModel->add_link(from_module, to_module);
    if (from_module != to_module) {
      from_module->add_link(to_module);
      DEBUG(CONNECTIVITY) << from_module->nodeType() << "_" << from_module->id()
                          << " --> "; std::cout << to_module-> nodeType()
                          << "_" << to_module->id();
    }
  }

  _subModel->post_process();
  // for (auto node : _subModel->node_list()) {
  //  node->setup_routing_memo();
  //}

  // Print out the Analytical Model for FU
  int id = 0;
  for (auto fu : _subModel->fu_list()) {
    // Test for Analytical Model
    // std::cout << "fu " << id << "'s area = " << fu -> get_area() <<" um^2\n";
    // std::cout << "fu " << id << "'s power = " << fu -> get_power() <<" mW\n";
    (void)fu;
    ++id;
  }

  // Print out the Switches Analytical Model Estimation
  id = 0;
  for (auto sw : _subModel->switch_list()) {
    // Test for Analytical Model
    // std::cout << "switch " << id << "'s area = " << sw -> get_area() <<" um^2\n";
    // std::cout << "switch " << id << "'s power = " << sw -> get_power() <<" mW\n";
    (void)sw;
    ++id;
  }

  // std::cout << "Overall Area = " << _subModel -> get_overall_area() << " um^2\n";
  // std::cout << "Overall Power = " << _subModel -> get_overall_power() << " mW\n";

  assert(num_outputs > 0);
  assert(num_inputs > 0);
  std::cout << "Json parsed" << std::endl;
}
*/
extern "C" void libssconfig_is_present() {}
