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
#include "sbinst.h"

namespace pt = boost::property_tree;


using namespace std;
using namespace SB_CONFIG;

void SbModel::printGamsKinds(ostream& os) {
  os << "set K \"Type of Node\" /Input,Output";
  
  for(int i = 2; i < SB_NUM_TYPES; ++i) {
    os << "," << name_of_inst((sb_inst_t)i);
  }
  os << "/";
}

SbModel::SbModel(SubModel* subModel, bool multi_config) {
  
  if (subModel) {
    _subModel = subModel;
  } else {
    _subModel = new SubModel(5, 5, SubModel::PortType::everysw, multi_config);
  }
}

SbModel::SbModel(bool multi_config) {
  _subModel = new SubModel(5, 5, SubModel::PortType::everysw, multi_config);
}

void SbModel::parse_exec(std::istream& istream) {
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

bool ends_with(string& s, string ending) {
  if (ending.size() > s.size()) return false;
  return std::equal(ending.rbegin(), ending.rend(), s.rbegin());
}

//File constructor
SbModel::SbModel(const char* filename, bool multi_config) {
    ifstream ifs(filename, ios::in);
    string param,value;
    
    if(ifs.fail())
    {
        cerr << "Could Not Open: " << filename << "\n";
        return;
    }
    
    string fn = string(filename);
    if(ends_with(fn,string(".json"))) {
      parse_json(ifs);
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
}

//JSON Format is flat format with all objects defined
void SbModel::parse_json(std::istream& istream) {
  pt::ptree root;
  read_json(istream,root);

  std::map<std::string,sbnode*> sym_tab;

  //Null Fu Model
  _fuModel = NULL;

  //Now create the submodel
  _subModel=new SubModel();

  int logical_rows = root.get<int>("numRows", 0);
  int logical_cols = root.get<int>("numCols", 0);
  printf("JSON Rows: %d, Cols %d\n", logical_rows, logical_cols);


  std::map<int,sb_inst_t> inst_map;
  for(auto& p : root.get_child("ISAencode")) {
    std::string inst_name = p.first;
    int idx =  p.second.get_value<int>();

    if(inst_name=="numISA") continue;

    sb_inst_t sb_inst = SB_CONFIG::inst_from_string(inst_name.c_str());
        
    if(sb_inst==SB_NONE || sb_inst==SB_ERR) {
      cerr << "ERROR IN PARSING INSTRUCTION: \"" << inst_name << "\"\n";
      assert(0);
      return;
    }

    inst_map[idx] = sb_inst;
  }

  //Look through all of the children of grid IR
  for(auto& p : root.get_child("GridIR")) {
    std::string elem_name = p.first;
    auto& node_def = p.second;
    int y = node_def.get<int>("row", 0);
    int x = node_def.get<int>("col", 0);

    string type = node_def.get<std::string>("type", "");
    if(type=="Switch") {
       sbswitch* sw = _subModel->add_switch(x,y);
       sym_tab[elem_name]=sw;
    } else if(type=="FU") {
       sbfu* fu = _subModel->add_fu(x,y);

       auto link = fu->add_link(fu); //For decomposability
       link->setdir(SbDIR::IP0);


       sym_tab[elem_name]=fu;
       auto& fu_def1 = node_def.get_child("Instructions");
       auto& fu_def2 = fu_def1.get_child("outPut_0");
    
       //TODO: FIXME: Memory leak 
       func_unit_def* fudef = new func_unit_def("NA");
       fu->setFUDef(fudef);

       for(auto& enc_def : fu_def2.get_child("subNet_0")) {
         int num = enc_def.second.get_value<int>();

         sb_inst_t inst = inst_map[num];
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

    auto* vp = new sbvport();
    sym_tab[elem_name]=vp;

    cout << "new port: \"" << elem_name << "\" \n";

    std::vector<sbnode*> nodes;
    for(auto& p : port_def.get_child("gridModules")) {
      string s = p.second.get_value<std::string>();
      sbnode* n = sym_tab[s];
      assert(n);
      nodes.push_back(n);
    }
 
    if(type=="InputPorts") {
      int port_num = num_ivp++;
      io.in_vports[port_num]=vp;
      for(sbnode* n:nodes) {
        int node_id = num_inputs++;
        cout << "added input to vec: " << node_id << "\n";

        vp->port_vec().push_back(node_id);
        sbinput* in = _subModel->add_input(node_id);  
        in->add_link(n);
      }
    } else if(type=="OutputPorts") {
      int port_num = num_ovp++;
      io.out_vports[port_num]=vp;
      for(sbnode* n:nodes) {
        int node_id = num_outputs++;
        cout << "added output to vec: " << node_id << "\n";

        vp->port_vec().push_back(node_id);
        sboutput* out = _subModel->add_output(node_id);  
        n->add_link(out);
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

    sbnode* from_module = sym_tab[from_str]; 
    sbnode* to_module   = sym_tab[to_str]; 
    assert(from_module && to_module);

    from_module->add_link(to_module);
    
  }

  _subModel->regroup_vecs();

  assert(num_inputs>0);
  assert(num_outputs>0);
}






extern "C" void libsbconfig_is_present() {}

