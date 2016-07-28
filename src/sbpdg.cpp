#include "model_parsing.h"
#include "sbpdg.h"
#include <vector>
#include <regex>
#include <set>

using namespace std;
using namespace SB_CONFIG;

int SbPDG_Node::ID_SOURCE=0;
int SbPDG_Edge::ID_SOURCE=0;

void order_insts(SbPDG_Inst* inst,
                 std::set<SbPDG_Inst*>& done_nodes,         //done insts
                 std::vector<SbPDG_Inst*>& ordered_insts) {

  if(done_nodes.count(inst)) {
    return;
  }

  //insert the new inst
  done_nodes.insert(inst);
 
  //incoming edges to a node
  for(auto i = inst->ops_begin(), e=inst->ops_end();i!=e;++i) {
    SbPDG_Edge* edge=*i;
   
    //if there is a defintion node
    if(SbPDG_Inst* op_inst = dynamic_cast<SbPDG_Inst*>(edge->def()) ) {
      order_insts(op_inst, done_nodes, ordered_insts);                  //recursive call until the top inst with the last incoming edge 
    }
  }

  ordered_insts.push_back(inst);
}

void SbPDG::compute(bool print) {

  if(_orderedInsts.size()==0) {
    std::set<SbPDG_Inst*> done_nodes;

    //for each output node traverse 
    //the incoming node
    for(SbPDG_Output* out : _outputs) {
      order_insts(out->out_inst(), done_nodes, _orderedInsts);
    } 
  }

  for(SbPDG_Inst* inst : _orderedInsts) {
    inst->compute(print);
  }

}

SbPDG::SbPDG() {

} 


//COMMA IF NOT FIRST
void CINF(std::ostream& os, bool& first) {
  if(first) {
    first=false;
  } else {
    os << ", " ;
  }
}

//Parse the string and add the vector
void SbPDG::parse_and_add_vec(string name, string line, map<string,SbPDG_Node*>& syms, bool input) {
   //parse string that looks like this: [1 2, 1 4, 2 6  3]
   
   unsigned first = line.find("[");
   string cur_cap = line.substr (first,line.find("]")-first);
   stringstream ss(cur_cap);
   
   vector<vector<int>> pm;

   //Parse the vector string from the dfg
   while (getline(ss, cur_cap, ',')) {

     if(cur_cap.empty()) {continue;} 
     istringstream ssc(cur_cap);
     std::vector<int> m;
    
     //vector of ints seperated by ' '
     while (getline(ssc, cur_cap, ' ')) {
       if(cur_cap.empty()) {continue;} 
       int val;
       istringstream(cur_cap)>>val;
       m.push_back(val);
     }

     pm.push_back(m);
   }

   if(input) {
     addVecInput(name,pm,syms);
   } else {
     addVecOutput(name,pm,syms);
   }
}

SbPDG::SbPDG(string filename) {

  string line;
  ifstream ifs(filename.c_str());

  if(ifs.fail()) {
    cerr << "Could Not Open: " << filename << "\n";
    assert(0);
  }

  regex re_input_vec("InputVec:\\s*(\\w+)\\s*\\[((:?(:?\\d+\\s*)+\\s*)\\,?\\s*)+\\]\\s*"); //bits num id
  regex re_output_vec("OutputVec:\\s*(\\w+)\\s*\\[((:?(:?\\d+\\s*)+\\s*)\\,?\\s*)+\\]\\s*"); //bits num id
  regex re_input("Input:\\s*(\\w+)\\s*"); //bits id  
  regex re_output("Output:\\s*(\\w+)\\s*");                          //out
  regex re_Op3("(\\w+)\\s*=\\s*(\\w+)\\(\\s*(\\w+)\\s*,\\s*(\\w+)\\s*,\\s*(\\w+)\\s*\\)");//id dep dep   -- 3 ip
  regex re_Op2("(\\w+)\\s*=\\s*(\\w+)\\(\\s*(\\w+)\\s*,\\s*(\\w+)\\s*\\)");//id dep dep -- 2 ip
  regex re_Op1("(\\w+)\\s*=\\s*(\\w+)\\(\\s*(\\w+)\\s*\\)");//id dep dep -- 1 ip
  smatch m;

  int cur_line=0;

  //Mapping of string to sbpdg node
  map<string, SbPDG_Node*> syms;

  while(ifs.good()) {  
    getline(ifs,line);
    ModelParsing::trim_comments(line);
    ModelParsing::trim(line);
    cur_line++;
    
    if(ModelParsing::StartsWith(line,"#") || line.empty()) {
      continue;
    }

    if (regex_search(line, m, re_input)) {
      //cout << m[1] << " " << m[2] << " " << m[3] << "\n";
      string name = m[1];
      addScalarInput(name,syms);

    } else if (regex_search(line,m,re_input_vec)) {
      string name = m[1];
      string vec = m[2];
      parse_and_add_vec(name, line, syms, true /*input*/);

    } else if (regex_search(line,m,re_Op1)) {
      //cout << "o1:" << m[1] << " " << m[2] << " " << m[3] << "\n";
      string var_out = m[1];
      string opcode  = m[2];
      string var_in  = m[3];

      SbPDG_Node* inc_node = syms[var_in];
      if(inc_node==NULL) {
        cerr << "Could not find \"" + var_in + "\" \n (op1)";
        assert("0");
      }

      SbPDG_Inst* pdg_inst = new SbPDG_Inst();
      pdg_inst->setName(var_out);
      syms[var_out]=pdg_inst;
      pdg_inst->setInst(inst_from_config_name(opcode.c_str()));
      connect(inc_node, pdg_inst,0,SbPDG_Edge::data);
      addInst(pdg_inst);

    } else if (regex_search(line,m,re_Op2)) {     
      //cout << "o2:" << m[1] << " " << m[2] << " " << m[3] << " " << m[4] << "\n";

      string var_out = m[1];
      string opcode  = m[2];
      string var_in1 = m[3];
      string var_in2 = m[4];

      SbPDG_Node* inc_node1 = syms[var_in1];
      SbPDG_Node* inc_node2 = syms[var_in2];
      
      if(inc_node1==NULL) {
        cerr << "Could not find \"" + var_in1 + "\" (in1,op2) \n";
        cerr << "in line: " << line << "\n";
        assert("0");
      } else if(inc_node2==NULL) {
        cerr << "Could not find \"" + var_in2 + "\"\n (in2,op2) ";
        assert("0");
      }

      SbPDG_Inst* pdg_inst = new SbPDG_Inst();
      pdg_inst->setName(var_out);
      syms[var_out]=pdg_inst;
      pdg_inst->setInst(inst_from_config_name(opcode.c_str()));
      connect(inc_node1, pdg_inst,0,SbPDG_Edge::data);
      connect(inc_node2, pdg_inst,1,SbPDG_Edge::data);
      addInst(pdg_inst);

     } else if (regex_search(line,m,re_Op3)) {
      //cout << "o2:" << m[1] << " " << m[2] << " " << m[3] << " " << m[4] << "\n";

      string var_out = m[1];
      string opcode  = m[2];
      string var_in1 = m[3];
      string var_in2 = m[4];
      string var_in3 = m[5];

      SbPDG_Node* inc_node1 = syms[var_in1];
      SbPDG_Node* inc_node2 = syms[var_in2];
      SbPDG_Node* inc_node3 = syms[var_in3];

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

      SbPDG_Inst* pdg_inst = new SbPDG_Inst();
      pdg_inst->setName(var_out);
      syms[var_out]=pdg_inst;
      pdg_inst->setInst(inst_from_config_name(opcode.c_str()));
      connect(inc_node1, pdg_inst,0,SbPDG_Edge::data);
      connect(inc_node2, pdg_inst,1,SbPDG_Edge::data);
      connect(inc_node3, pdg_inst,2,SbPDG_Edge::ctrl_true);
      addInst(pdg_inst);

    } else if (regex_search(line,m,re_output)) {  //SCALAR OUTPUT ONLY
      //cout << "out:" << m[1] << "\n";    
      string var_out = m[1];
      addScalarOutput(var_out,syms);
    } else {
      cout << "Line: \"" << line << "\"\n";
      assert(0&&"I don't know what this line is for\n");
    }


  }

}



// -- Gams names --
std::string SbPDG_Edge::gamsName() {
  std::stringstream ss;
  ss << _def->gamsName() << "_" << _use->gamsName() << "i" << _ID ;
  return ss.str();
}

std::string SbPDG_Input::gamsName() {
  std::stringstream ss;
  ss << "IV" << _ID;
  return ss.str();
}

std::string SbPDG_Output::gamsName() {
  std::stringstream ss;
  ss << "OV" << _ID;
  return ss.str();
}

std::string SbPDG_Inst::gamsName() {
  std::stringstream ss;
  ss << "FV" << _ID;
  return ss.str();
}


void SbPDG_Node::printGraphviz(ostream& os) {
  
  string ncolor = "black";
  os << "N" << _ID << " [ label = \"" << name();
        
  os  << "\", color= \"" << ncolor << "\"]; ";

  os << "\n";
  
  //print edges
  SbPDG_Node::const_edge_iterator I,E;
  for(I=uses_begin(),E=uses_end();I!=E;++I) {
    SbPDG_Edge* e = (*I);
    
    if(e->etype()==SbPDG_Edge::data) {
       ncolor="black";
    } else if(e->etype()==SbPDG_Edge::ctrl_true) {
       ncolor="blue";
    } else if(e->etype()==SbPDG_Edge::ctrl_false) {
       ncolor="red";
    }
    
    SbPDG_Node* n = e->use();
    os << "N" << _ID << " -> N" << n->_ID << "[ color=";
    os << ncolor;
    os << "];\n";
  }
 
  os << "\n";

}

void SbPDG_Inst::printGraphviz(ostream& os) {
  SbPDG_Node::printGraphviz(os);
}

void SbPDG_Output::printGraphviz(ostream& os) {
  SbPDG_Node::printGraphviz(os);
}

void SbPDG_Input::printGraphviz(ostream& os) {
  SbPDG_Node::printGraphviz(os);
}

//Connect two nodes in PDG
SbPDG_Edge* SbPDG::connect(SbPDG_Node* orig, SbPDG_Node* dest,int slot,SbPDG_Edge::EdgeType etype) {
  
  SbPDG_Edge* new_edge = new SbPDG_Edge(orig,dest,etype);

  SbPDG_Inst* inst = 0;
  if(etype==SbPDG_Edge::ctrl_true) {
    if((inst = dynamic_cast<SbPDG_Inst*>(dest))) {
      //std::cout << "true edge" << orig->name() << "->" << dest->name() << "\n";
      slot = 2;
      inst->setPredInv(false);
    } else {
      assert(0 && "not a Inst dest"); 
    }
  } else if (etype==SbPDG_Edge::ctrl_false) {
    if((inst = dynamic_cast<SbPDG_Inst*>(dest))) {
      //std::cout << "false edge" << orig->name() << "->" << dest->name() << "\n";
      slot = 2;
      inst->setPredInv(true);
    } else {
      assert(0 && "not an Inst dest"); 
    }
  } else if (etype==SbPDG_Edge::data) {
      //std::cout << "data edge" << orig->name() << "->" << dest->name() << "\n";
      assert(slot >=0);
      assert(slot <=1);
  }
  
  dest->addIncEdge(slot,new_edge);
  orig->addOutEdge(orig->num_out(),new_edge);
  _edges.push_back(new_edge);
  
  return new_edge;
}


void SbPDG::printGraphviz(ostream& os)
{
  os << "Digraph G { \n" ;
  const_inst_iterator Ii,Ei;

  //Insts
  for (Ii=_insts.begin(),Ei=_insts.end();Ii!=Ei;++Ii)  { (*Ii)->printGraphviz(os); }
  
  const_input_iterator Iin,Ein;

  //Inputs
  for (Iin=_inputs.begin(),Ein=_inputs.end();Iin!=Ein;++Iin)  { (*Iin)->printGraphviz(os); }
  
  const_output_iterator Iout,Eout;

  //Outputs
  for (Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)  { (*Iout)->printGraphviz(os); }

  os << "\t{ rank = same; ";
  for (Iin=_inputs.begin(),Ein=_inputs.end();Iin!=Ein;++Iin)   { os << "N" << (*Iin)->id() << " ";  }
  os << "}\n";
  
  os << "\t{ rank = same; ";
  for (Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)   { os << "N" << (*Iout)->id() << " "; }
  os << "}\n";

  os << "}\n";
} 


bool is_compatible(vector<vector<int>>& vec_m, vector<pair<int, vector<int>>>& port_m) {
  
  if(vec_m.size() != port_m.size()) {
    return false;
  }
  
  for(unsigned i = 0; i < vec_m.size(); ++i) {
    if(vec_m[i].size() != port_m[i].second.size()) {
      return false;
    }

    for(unsigned j = 0; j < vec_m[i].size(); ++j) {
      if(vec_m[i][j] != port_m[i].second[j]) {
        return false;
      }
    }
  }
  return true;
}

//IO-Model has the hardware vector io interface mapping
void SbPDG::printPortCompatibilityWith(std::ostream& os, SB_CONFIG::SbModel* sbModel) {
  os << "set cp(pv,pn) \"Port Compatibility\" \n /";   // Print the port compatibility
  bool first=true;  

  for(auto& vec_in : _vecInputs) {
    vector<vector<int> >& vec_m = vec_in->locMap();
    std::vector<int> matching_ports;

    for(auto& port_interf : sbModel->subModel()->io_interf().in_vports) {
      std::vector<std::pair<int, std::vector<int> > >& port_m = port_interf.second;
       
      if(is_compatible(vec_m,port_m)) {
        matching_ports.push_back(port_interf.first);
        CINF(os,first);
        os << vec_in->gamsName() << ".ip" << port_interf.first << " ";
      }
    }

    if(matching_ports.size()==0) {
      cout << "IN PORT \"" << vec_in->gamsName() << "\" DID NOT MATCH ANY HARDWARE PORT INTERFACE\n";
      assert(0);
    }
  }

  for(auto& vec_out : _vecOutputs) {
    vector<vector<int> >& vec_m = vec_out->locMap();
    std::vector<int> matching_ports;

    for(auto& port_interf : sbModel->subModel()->io_interf().out_vports) {

      std::vector<std::pair<int, std::vector<int> > >& port_m = port_interf.second;

      if(is_compatible(vec_m,port_m)) {
        matching_ports.push_back(port_interf.first);
        CINF(os,first);
        os << vec_out->gamsName() << ".op" << port_interf.first << " ";
      }
    }

    if(matching_ports.size()==0) {
      cout << "OUT PORT \"" << vec_out->gamsName() << "\" DID NOT MATCH ANY HARDWARE PORT INTERFACE\n";
      assert(0);
    }

  }

  os << "/;\n";
}

//Gams related
void SbPDG::printGams(std::ostream& os,
                      std::unordered_map<string,SbPDG_Node*>& node_map,
                      std::unordered_map<std::string,SbPDG_Edge*>& edge_map,
                      std::unordered_map<std::string, SbPDG_Vec*>& port_map) {
  
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
  
  for(int i = 2; i < SB_NUM_TYPES; ++i) {
    sb_inst_t sb_inst = (sb_inst_t)i;
    
    os << "set " << name_of_inst(sb_inst) << "V(v) /";
    bool first=true;
    
    const_inst_iterator Ii,Ei;
    for (Ii=_insts.begin(),Ei=_insts.end();Ii!=Ei;++Ii)  { 
      SbPDG_Inst* pdg_inst= *Ii;
      
      if(sb_inst == pdg_inst->inst()) {
        CINF(os,first);
        os << pdg_inst->gamsName();
      }
    }
    os << "/;\n";
  }

  bool first=true;
  os << "set pv(*) \"Port Vectors\" \n /";   // Print the set of port vertices:
  for(auto& i : _vecInputs) {
    CINF(os,first);
    os << i->gamsName() << " ";
    port_map[i->gamsName()]=i;
  }
  for(auto& i : _vecOutputs) {
    CINF(os,first);
    os << i->gamsName() << " ";
    port_map[i->gamsName()]=i;
  }
  os << "/;\n";

  first=true;
  os << "parameter VI(pv,v) \"Port Vector Definitions\" \n /";   // Print the set of port vertices mappings:
  for(auto& i : _vecInputs) {
    int ind=0;
    std::vector<SbPDG_Input*>::iterator I,E;
    for(I=i->input_begin(),E=i->input_end();I!=E;++I,++ind) {
      SbPDG_Input* sbin = *I;
      CINF(os,first);
      os << i->gamsName() << "." << sbin->gamsName() << " " << ind+1;
    }
  }
  for(auto& i : _vecOutputs) {
    int ind=0;
    std::vector<SbPDG_Output*>::iterator I,E;
    for(I=i->output_begin(),E=i->output_end();I!=E;++I,++ind) {
      SbPDG_Output* sbout = *I;
      CINF(os,first);
      os << i->gamsName() << "." << sbout->gamsName() << " " << ind+1;

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

  for(int i = 2; i < SB_NUM_TYPES; ++i) {
    sb_inst_t sb_inst = (sb_inst_t)i;
    os << "kindV(\'" << name_of_inst(sb_inst) << "\', " << name_of_inst(sb_inst) << "V(v))=YES;\n";
  }

  // --------------------------- Print the linkage ------------------------
  os << "parameter Gve(v,e) \"vetex to edge\" \n /";   // def edges
  for (Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
    if(Ie!=_edges.begin()) os << ", ";
    
    SbPDG_Edge* edge = *Ie;  
    os << edge->def()->gamsName() << "." << edge->gamsName() << " 1";
  }
  os << "/;\n";
  
  os << "parameter Gev(e,v) \"edge to vertex\" \n /";   // use edges
  for (Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
    if(Ie!=_edges.begin()) os << ", ";
    
    SbPDG_Edge* edge = *Ie;  
    os << edge->gamsName() << "." << edge->use()->gamsName() << " 1";
  }
  os << "/;\n";
  
  os << "set intedges(e) \"edges\" \n /";   // Internal Edges
  first =true;
  for (Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
    SbPDG_Edge* edge = *Ie;  
    
    if(!dynamic_cast<SbPDG_Input*>(edge->def()) && !dynamic_cast<SbPDG_Output*>(edge->use()) ) {
      if (first) first = false;
      else os << ", ";
      os << edge->gamsName();
    }
  }
  os << "/;\n";
  
  os << "parameter delta(e) \"delay of edge\" \n /";   // Print the set of edges:
  for (Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
    if(Ie!=_edges.begin()) os << ", ";
    SbPDG_Edge* edge = *Ie;
    
    if(SbPDG_Inst* pdginst = dynamic_cast<SbPDG_Inst*>(edge->def())) {       
       os << (*Ie)->gamsName() << " " << inst_lat(pdginst->inst());
    } else {
       os << (*Ie)->gamsName() << " " << "0";  //TODO: WHAT LATENCY SHOULD I USE??
    }
    
   
  }
  os << "/;\n";
  
}
