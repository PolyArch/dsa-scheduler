#include "dypdg.h"

#include <boost/regex.hpp>
#include "model_parsing.h"
#include "dypdg.h"
#include <vector>

using namespace std;
using namespace SB_CONFIG;

int DyPDG_Node::ID_SOURCE=0;
int DyPDG_Edge::ID_SOURCE=0;



DyPDG::DyPDG() {
  std::cout << "hello\n";
} 

DyPDG::DyPDG(string filename) {
  cout << "file: " << filename << "\n";

  string line;
  ifstream ifs(filename.c_str());

  if(ifs.fail()) {
    cerr << "Could Not Open: " << filename << "\n";
    assert(0);
  }

  boost::regex re_input_vec("InputVec:\\s*(\\d+)\\s*(\\d+)\\s*(\\w+)\\s*\\[((:?(:?\\d+\\s*)+\\s*)\\,?\\s*)+\\]\\s*"); //bits num id
  boost::regex re_input("Input:\\s*(\\d+)\\s*(\\w+)\\s*"); //bits id
  boost::regex re_output("Output:\\s*(\\w+)\\s*");                          //out
  boost::regex re_Op3("(\\w+)\\s*=\\s*(\\w+)\\(\\s*(\\w+)\\s*,\\s*(\\w+)\\s*,\\s*(\\w+)\\s*\\)");//id dep dep
  boost::regex re_Op2("(\\w+)\\s*=\\s*(\\w+)\\(\\s*(\\w+)\\s*,\\s*(\\w+)\\s*\\)");//id dep dep
  boost::regex re_Op1("(\\w+)\\s*=\\s*(\\w+)\\(\\s*(\\w+)\\s*\\)");//id dep dep
  boost::smatch m;

  int num_inputs=0;
  int cur_line=0;
  map<string,DyPDG_Node*> syms;

  while(ifs.good()) {  
    getline(ifs,line);
    ModelParsing::trim_comments(line);
    ModelParsing::trim(line);
    cur_line++;
    if(ModelParsing::StartsWith(line,"#") || line.empty()) {
      continue;
    }

    if (regex_search(line,m,re_input)) {
      //cout << m[1] << " " << m[2] << " " << m[3] << "\n";
      int bitsper = std::stoi(m[1]); bitsper+=0;
      string name = m[2];

      DyPDG_Input* pdg_in = new DyPDG_Input();
      syms[name]=pdg_in;
      pdg_in->setName(name);
      addInput(pdg_in);

      num_inputs++; //not used currently

    } else if (regex_search(line,m,re_input_vec)) {
      int bitsper = std::stoi(m[1]); bitsper+=0;
      int entries = std::stoi(m[2]);
      string name = m[3];
      string vec = m[4];

      //parse string that looks like this: [1 2, 1 4, 2 6  3]
      unsigned first = line.find("[");
      string cur_cap = line.substr (first,line.find("]")-first);
      stringstream ss(cur_cap);
      vector<vector<int>> pm;
      while (getline(ss, cur_cap, ',')) {
        if(cur_cap.empty()) {continue;} 
        istringstream ssc(cur_cap);
        std::vector<int> m;
        while (getline(ssc, cur_cap, ' ')) {
          if(cur_cap.empty()) {continue;} 
          int val;
          istringstream(cur_cap)>>val;
          m.push_back(val);
        }
        pm.push_back(m);
      }
 
      addVecInput(entries,name,pm,syms);

    } else if (regex_search(line,m,re_Op1)) {
      //cout << "o1:" << m[1] << " " << m[2] << " " << m[3] << "\n";

      string var_out = m[1];
      string opcode  = m[2];
      string var_in  = m[3];

      DyPDG_Node* inc_node = syms[var_in];
      if(inc_node==NULL) {
        cerr << "Could not find \"" + var_in + "\" \n (op1)";
        assert("0");
      }

      DyPDG_Inst* pdg_inst = new DyPDG_Inst();
      pdg_inst->setName(var_out);
      syms[var_out]=pdg_inst;
      pdg_inst->setInst(inst_from_config_name(opcode.c_str()));
      connect(inc_node, pdg_inst,0,DyPDG_Edge::data);
      addInst(pdg_inst);

    } else if (regex_search(line,m,re_Op2)) {     
      //cout << "o2:" << m[1] << " " << m[2] << " " << m[3] << " " << m[4] << "\n";

      string var_out = m[1];
      string opcode  = m[2];
      string var_in1 = m[3];
      string var_in2 = m[4];

      DyPDG_Node* inc_node1 = syms[var_in1];
      DyPDG_Node* inc_node2 = syms[var_in2];
      if(inc_node1==NULL) {
        cerr << "Could not find \"" + var_in1 + "\" (in1,op2) \n";
        assert("0");
      } else if(inc_node2==NULL) {
        cerr << "Could not find \"" + var_in2 + "\"\n (in2,op2) ";
        assert("0");
      }

      DyPDG_Inst* pdg_inst = new DyPDG_Inst();
      pdg_inst->setName(var_out);
      syms[var_out]=pdg_inst;
      pdg_inst->setInst(inst_from_config_name(opcode.c_str()));
      connect(inc_node1, pdg_inst,0,DyPDG_Edge::data);
      connect(inc_node2, pdg_inst,1,DyPDG_Edge::data);
      addInst(pdg_inst);

     } else if (regex_search(line,m,re_Op3)) {
      //cout << "o2:" << m[1] << " " << m[2] << " " << m[3] << " " << m[4] << "\n";

      string var_out = m[1];
      string opcode  = m[2];
      string var_in1 = m[3];
      string var_in2 = m[4];
      string var_in3 = m[5];

      DyPDG_Node* inc_node1 = syms[var_in1];
      DyPDG_Node* inc_node2 = syms[var_in2];
      DyPDG_Node* inc_node3 = syms[var_in3];

      if(inc_node1==NULL) {
        cerr << "Could not find \"" + var_in1 + "\" (in1,op3) \n";
        assert("0");
      } else if(inc_node2==NULL) {
        cerr << "Could not find \"" + var_in2 + "\"\n (in2,op3) ";
        assert("0");
      } else if(inc_node3==NULL) {
        cerr << "Could not find \"" + var_in3 + "\"\n (in3,op3) ";
        assert("0");
      }

      DyPDG_Inst* pdg_inst = new DyPDG_Inst();
      pdg_inst->setName(var_out);
      syms[var_out]=pdg_inst;
      pdg_inst->setInst(inst_from_config_name(opcode.c_str()));
      connect(inc_node1, pdg_inst,0,DyPDG_Edge::data);
      connect(inc_node2, pdg_inst,1,DyPDG_Edge::data);
      connect(inc_node3, pdg_inst,2,DyPDG_Edge::ctrl_true);
      addInst(pdg_inst);

    } else if (regex_search(line,m,re_output)) {     
      //cout << "out:" << m[1] << "\n";    
      string var_out = m[1];

      DyPDG_Node* out_node = syms[var_out];
      if(out_node==NULL) {
        cerr << "Could not find" + var_out + "\n";
        assert("0");
      }

      DyPDG_Output* pdg_out = new DyPDG_Output();
      pdg_out->setName(var_out);
      syms[var_out]=pdg_out;
      connect(out_node, pdg_out,0,DyPDG_Edge::data);
      addOutput(pdg_out);

      //pdg_out->setVPort(num_output_vectors++);

    } else {
      //cout << "Line: \"" << line << "\"\n";
      assert(0&&"I don't know what this line is for\n");
    }


  }

}



// -- Gams names --
std::string DyPDG_Edge::gamsName() {
  std::stringstream ss;
  ss << _def->gamsName() << "_" << _use->gamsName() << "i" << _ID ;
  return ss.str();
}

std::string DyPDG_Input::gamsName() {
  std::stringstream ss;
  ss << "IV" << _ID;
  return ss.str();
}

std::string DyPDG_Output::gamsName() {
  std::stringstream ss;
  ss << "OV" << _ID;
  return ss.str();
}

std::string DyPDG_Inst::gamsName() {
  std::stringstream ss;
  ss << "FV" << _ID;
  return ss.str();
}


void DyPDG_Node::printGraphviz(ostream& os) {
  string ncolor = "black";
  os << "N" << _ID << " [ label = \"" << name();
        
  os  << "\", color= \"" << ncolor << "\"]; ";

  os << "\n";
  
  //print edges
  DyPDG_Node::const_edge_iterator I,E;
  for(I=uses_begin(),E=uses_end();I!=E;++I) {
    DyPDG_Edge* e = (*I);
    
    if(e->etype()==DyPDG_Edge::data) {
       ncolor="black";
    } else if(e->etype()==DyPDG_Edge::ctrl_true) {
       ncolor="blue";
    } else if(e->etype()==DyPDG_Edge::ctrl_false) {
       ncolor="red";
    }
    
    DyPDG_Node* n = e->use();
    os << "N" << _ID << " -> N" << n->_ID << "[ color=";
    os << ncolor;
    os << "];\n";
  }
 
  os << "\n";

}

void DyPDG_Inst::printGraphviz(ostream& os) {
  DyPDG_Node::printGraphviz(os);
}

void DyPDG_Output::printGraphviz(ostream& os) {
  DyPDG_Node::printGraphviz(os);
}

void DyPDG_Input::printGraphviz(ostream& os) {
  DyPDG_Node::printGraphviz(os);
}




DyPDG_Edge* DyPDG::connect(DyPDG_Node* orig, DyPDG_Node* dest,int slot,DyPDG_Edge::EdgeType etype) {
  DyPDG_Edge* new_edge = new DyPDG_Edge(orig,dest,etype);


  DyPDG_Inst* inst = 0;
  if(etype==DyPDG_Edge::ctrl_true) {
    if((inst = dynamic_cast<DyPDG_Inst*>(dest))) {
      //std::cout << "true edge" << orig->name() << "->" << dest->name() << "\n";
      slot = 2;
      inst->setPredInv(false);
    } else {
      assert(0 && "not a Inst dest"); 
    }
  } else if (etype==DyPDG_Edge::ctrl_false) {
    if((inst = dynamic_cast<DyPDG_Inst*>(dest))) {
      //std::cout << "false edge" << orig->name() << "->" << dest->name() << "\n";
      slot = 2;
      inst->setPredInv(true);
    } else {
      assert(0 && "not an Inst dest"); 
    }
  } else if (etype==DyPDG_Edge::data) {
      //std::cout << "data edge" << orig->name() << "->" << dest->name() << "\n";
      assert(slot >=0);
      assert(slot <=1);
  }
  
  dest->addIncEdge(slot,new_edge);
  orig->addOutEdge(orig->num_out(),new_edge);
  _edges.push_back(new_edge);
  
  return new_edge;
}


void DyPDG::printGraphviz(ostream& os)
{
  os << "Digraph G { \n" ;
  const_inst_iterator Ii,Ei;
  for (Ii=_insts.begin(),Ei=_insts.end();Ii!=Ei;++Ii)  { (*Ii)->printGraphviz(os); }
  
  const_input_iterator Iin,Ein;
  for (Iin=_inputs.begin(),Ein=_inputs.end();Iin!=Ein;++Iin)  { (*Iin)->printGraphviz(os); }
  
  const_output_iterator Iout,Eout;
  for (Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)  { (*Iout)->printGraphviz(os); }

  os << "\t{ rank = same; ";
  for (Iin=_inputs.begin(),Ein=_inputs.end();Iin!=Ein;++Iin)   { os << "N" << (*Iin)->id() << " ";  }
  os << "}\n";
  
  os << "\t{ rank = same; ";
  for (Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)   { os << "N" << (*Iout)->id() << " "; }
  os << "}\n";

  os << "}\n";
} 

void DyPDG::printPortCompatibilityWith(std::ostream& os, SB_CONFIG::DyModel* dyModel) {
  os << "set cp(pv,pn) \"Port Compatibility\" \n /";   // Print the port compatibility
  bool first=true;  

  for(auto& vec_in : _vecInputs) {
    vector<vector<int> >& vec_m = vec_in->locMap();
    std::vector<int> matching_ports;

    for(auto& port_interf : dyModel->subModel()->io_interf().in_vports) {
      std::vector<std::pair<int, std::vector<int> > >& port_m = port_interf.second;

      if(vec_m.size() != port_m.size()) {
        continue;
      }
      bool didnt_work=false;
      for(unsigned i = 0; i < vec_m.size(); ++i) {
        if(vec_m[i].size() != port_m[i].second.size()) {
          didnt_work=true;
          break;
        }
        for(unsigned j = 0; j < vec_m[i].size(); ++j) {
          if(vec_m[i][j] != port_m[i].second[j]) {
            didnt_work=true;
            break;
          }
        }
      }
      if(!didnt_work) {
        matching_ports.push_back(port_interf.first);
        if(first) {
          first=false;
        } else {
          os << ",";
        }
        os << vec_in->gamsName() << ".ip" << port_interf.first << " ";
      }
    }

    if(matching_ports.size()==0) {
      cout << "PORT \"" << vec_in->gamsName() << "\" DID NOT MATCH ANY HARDWARE PORT INTERFACE\n";
    }

  }
  os << "/;\n";
}


void DyPDG::printGams(std::ostream& os,std::unordered_map<string,DyPDG_Node*>& node_map,std::unordered_map<std::string,DyPDG_Edge*>& edge_map) {
  
  os << "$onempty\n";
  
  os << "set v \"verticies\" \n /";   // Print the set of Nodes:
  
  const_node_iterator Ii,Ei;
  for (Ii=_nodes.begin(),Ei=_nodes.end();Ii!=Ei;++Ii)  { 
    if(Ii!=_nodes.begin()) os << ", ";
    os << (*Ii)->gamsName();
    node_map[(*Ii)->gamsName()]=*Ii;
  }
  os << "/;\n";

  os << "set inV(v) \"input verticies\" /" ;   // Print the set of Nodes:
  const_input_iterator Iin,Ein;
  for (Iin=_inputs.begin(),Ein=_inputs.end();Iin!=Ein;++Iin)  { 
    if(Iin!=_inputs.begin()) os << ", ";
    os << (*Iin)->gamsName();
  }
  os << "/;\n";
  
  os << "set outV(v) \"output verticies\" /" ;   // Print the set of Nodes:
  const_output_iterator Iout,Eout;
  for (Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)  { 
    if(Iout!=_outputs.begin()) os << ", ";
    os << (*Iout)->gamsName();
  }
  os << "/;\n";
  
  os << "set iv(v) \"instruction verticies\";\n";
  os << "iv(v) = (not inV(v)) and (not outV(v));\n";
  
  for(int i = 2; i < DY_NUM_TYPES; ++i) {
    dy_inst_t dy_inst = (dy_inst_t)i;
    
    os << "set " << name_of_inst(dy_inst) << "V(v) /";
    bool first=true;
    
    const_inst_iterator Ii,Ei;
    for (Ii=_insts.begin(),Ei=_insts.end();Ii!=Ei;++Ii)  { 
      DyPDG_Inst* pdg_inst= *Ii;
      
      if(dy_inst == pdg_inst->inst()) {
        if(first) {
          first = false;
        } else {
          os << ", ";   
        }
        os << pdg_inst->gamsName();
      }
    }
    os << "/;\n";
  }

  bool first=true;
  os << "set pv(*) \"Port Vectors\" \n /";   // Print the set of port vertices:
  for(auto& i : _vecInputs) {
    if(first) {
      first=false;
    } else {
      os << ", ";
    } 
    os << i->gamsName() << " ";
  }
  os << "/;\n";

  first=true;
  os << "parameter VI(pv,v) \"Port Vector Definitions\" \n /";   // Print the set of port vertices mappings:
  for(auto& i : _vecInputs) {
    int ind=0;
    std::vector<DyPDG_Input*>::iterator I,E;
    for(I=i->input_begin(),E=i->input_end();I!=E;++I,++ind) {
      DyPDG_Input* dyin = *I;
      if(first) {
        first=false;
      } else {
        os << ", ";
      } 
      os << i->gamsName() << "." << dyin->gamsName() << " " << ind+1;
    }
  }
   os << "/;\n";

  // -------------------edges ----------------------------
  os << "set e \"edges\" \n /";   // Print the set of edges:

  const_edge_iterator Ie,Ee;
  for (Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
    if(Ie!=_edges.begin()) os << ", ";
    os << (*Ie)->gamsName();
    edge_map[(*Ie)->gamsName()]=*Ie;
  }
  os << "/;\n";
  


  //create the kindC Set
  os << "set kindV(K,v) \"Vertex Type\"; \n";
  
  // --------------------------- Enable the Sets ------------------------
  os << "kindV('Input', inV(v))=YES;\n";
  os << "kindV('Output', outV(v))=YES;\n";

  for(int i = 2; i < DY_NUM_TYPES; ++i) {
    dy_inst_t dy_inst = (dy_inst_t)i;
    os << "kindV(\'" << name_of_inst(dy_inst) << "\', " << name_of_inst(dy_inst) << "V(v))=YES;\n";
  }

  // --------------------------- Print the linkage ------------------------
  os << "parameter Gve(v,e) \"vetex to edge\" \n /";   // def edges
  for (Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
    if(Ie!=_edges.begin()) os << ", ";
    
    DyPDG_Edge* edge = *Ie;  
    os << edge->def()->gamsName() << "." << edge->gamsName() << " 1";
  }
  os << "/;\n";
  
  os << "parameter Gev(e,v) \"edge to vertex\" \n /";   // use edges
  for (Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
    if(Ie!=_edges.begin()) os << ", ";
    
    DyPDG_Edge* edge = *Ie;  
    os << edge->gamsName() << "." << edge->use()->gamsName() << " 1";
  }
  os << "/;\n";
  
  os << "set intedges(e) \"edges\" \n /";   // Internal Edges
  first =true;
  for (Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
    DyPDG_Edge* edge = *Ie;  
    
    if(!dynamic_cast<DyPDG_Input*>(edge->def()) && !dynamic_cast<DyPDG_Output*>(edge->use()) ) {
      if (first) first = false;
      else os << ", ";
      os << edge->gamsName();
    }
  }
  os << "/;\n";
  
  os << "parameter delta(e) \"delay of edge\" \n /";   // Print the set of edges:
  for (Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
    if(Ie!=_edges.begin()) os << ", ";
    DyPDG_Edge* edge = *Ie;
    
    if(DyPDG_Inst* pdginst = dynamic_cast<DyPDG_Inst*>(edge->def())) {       
       os << (*Ie)->gamsName() << " " << inst_lat(pdginst->inst());
    } else {
       os << (*Ie)->gamsName() << " " << "0";  //TODO: WHAT LATENCY SHOULD I USE??
    }
    
   
  }
  os << "/;\n";
  
}
