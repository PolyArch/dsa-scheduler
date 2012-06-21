
#include "dypdg.h"

using namespace std;
using namespace DY_MODEL;

int DyPDG_Node::ID_SOURCE=0;
int DyPDG_Edge::ID_SOURCE=0;

DyPDG::DyPDG() {
  
}
// -- Gams names --
std::string DyPDG_Edge::gamsName() {
  std::stringstream ss;
  ss << _def->gamsName() << "t" << _use->gamsName() << "i" << _ID ;
  return ss.str();
}

std::string DyPDG_Input::gamsName() {
  std::stringstream ss;
  ss << "I" << _ID;
  return ss.str();
}

std::string DyPDG_Output::gamsName() {
  std::stringstream ss;
  ss << "O" << _ID;
  return ss.str();
}

std::string DyPDG_Inst::gamsName() {
  std::stringstream ss;
  ss << "N" << _ID;
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
  bool first =true;
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
