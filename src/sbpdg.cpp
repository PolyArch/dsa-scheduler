#include "model_parsing.h"
#include "sbpdg.h"
#include <vector>
#include <regex>
#include <set>
#include <iomanip>
#include <string>
#include <list>
#include "schedule.h"

using namespace std;
using namespace SB_CONFIG;

int SbPDG_Node::ID_SOURCE=0;
int SbPDG_Edge::ID_SOURCE=0;

bool SbPDG_VecInput::backPressureOn() {
    return false;
}



void order_insts(SbPDG_Inst* inst,
                 std::set<SbPDG_Inst*>& done_nodes,         //done insts
                 std::vector<SbPDG_Inst*>& ordered_insts) {

  if(done_nodes.count(inst)) {
    return;
  }

  //insert the new inst
  done_nodes.insert(inst);
 
  //incoming edges to a node
  int index=0;
  for(auto i = inst->ops_begin(), e=inst->ops_end();i!=e;++i,++index) {
    SbPDG_Edge* edge=*i;
    if(!edge) {
      assert(inst->immSlot()==index);
      continue;
    }
   
    //if there is a defintion node
    if(SbPDG_Inst* op_inst = dynamic_cast<SbPDG_Inst*>(edge->def()) ) {
      order_insts(op_inst, done_nodes, ordered_insts);                  
      //recursive call until the top inst with the last incoming edge 
    }
  }

  ordered_insts.push_back(inst);
}

// Adding new code for cycle-by-cycle CGRA functionality
int SbPDG::cycle_compute(bool print, bool verif) {
 
  // storing the output every cycles: solving other issues first
  // cycle_store();

  int num_computed = 0;
  SbPDG_VecInput* vec;

  for (int vector_id=0; vector_id<num_vec_input(); ++vector_id){
    vec = _vecInputs[vector_id];
    for (unsigned int i=0; i<vec->num_inputs(); ++i){
      // should call compute only if the data is available: that's correct
      num_computed += vec->getInput(i)->compute_backcgra(print, verif);
    } 
  }

  return num_computed;
}

// Adding new code for cycle-by-cycle CGRA functionality
int SbPDG::compute_backcgra(SbPDG_VecInput* vec_in, bool print, bool verif) {
 
    int num_computed = 0;
    for (unsigned int i=0; i<vec_in->num_inputs(); ++i){
      // std::cout << "Trigger computation for each input in this vector: "<<i<<"\n";
      num_computed += vec_in->getInput(i)->compute_backcgra(print, verif);
    } 

  return num_computed;
}
// --------------------------------------------------



void SbPDG::check_for_errors() {
  bool error = false;
  for (auto I=_inputs.begin(),E=_inputs.end();I!=E;++I)  { 
    if((*I)->num_out()==0) {
      cerr << "Error: No uses on input " << (*I)->name() << "\n";  
      error=true;
    }
  }
  for (auto I=_insts.begin(),E=_insts.end();I!=E;++I)  { 
    if((*I)->num_out()==0) {
      cerr << "Error: No uses on inst " << (*I)->name() << "\n";  
      error=true;
    }
    if((*I)->num_inc()==0) {
      cerr << "Error: No operands on inst " << (*I)->name() << "\n";
      error=true;
    }
  }
  for (auto I=_outputs.begin(),E=_outputs.end();I!=E;++I)  { 
    if((*I)->num_inc()==0) {
      cerr << "Error: No operands on output " << (*I)->name() << "\n";
      error=true;
    }
  }

  if(error) {
    assert(0 && "ERROR: BAD PDG");
  }
}

// This function is called from the simulator to 
int SbPDG::compute(bool print, bool verif, int g) {
  //if(_orderedInstsGroup.size()<=(unsigned)g) {
  //  _orderedInstsGroup.resize(g+1);
  //}

  //if(_orderedInsts.size()==0) {
  //  std::set<SbPDG_Inst*> done_nodes;

  //  //for each output node traverse 
  //  //the incoming node
  //  for(SbPDG_Output* out : _outputs) {
  //    if(SbPDG_Inst* producing_node = out->out_inst()) {
  //      order_insts(producing_node, done_nodes, _orderedInsts);
  //    }
  //  } 
  //}

  //for(SbPDG_Inst* inst : _orderedInsts) {
  //  inst->compute(print,verif);
  //}

  //printf("came inside compute function at line:74\n");


  int num_computed=0;

  assert(g < (int)_vecInputGroups.size());
  for(unsigned i = 0; i < _vecInputGroups[g].size(); ++i) {
    SbPDG_VecInput* vec = _vecInputGroups[g][i];
    for(unsigned j = 0; j < vec->num_inputs(); ++j) {
      num_computed += vec->getInput(j)->compute(print,verif); //calling some other compute
    }
  }
  return num_computed;
}


int SbPDG::maxGroupThroughput(int g) {
  int maxgt=0;

  assert(g < (int)_vecInputGroups.size());
  for(unsigned i = 0; i < _vecInputGroups[g].size(); ++i) {
    SbPDG_VecInput* vec = _vecInputGroups[g][i];
    for(unsigned j = 0; j < vec->num_inputs(); ++j) {
      maxgt=std::max(maxgt,vec->getInput(j)->maxThroughput());
    }
  }
  return maxgt;
}

SbPDG::SbPDG() {
  _vecInputGroups.push_back({});
  _vecOutputGroups.push_back({});
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

bool conv_to_int(std::string s, uint64_t& ival){ 
  try {
     ival = stol(s,0,0);
     return true;
  } catch(...){}
  return false;
}

bool conv_to_double(std::string s, double& dval){ 
  try {
     dval = stod(s);
     return true;
  } catch(...){}
  return false;
}

void SbPDG::parse_and_add_inst(string var_out, string opcode, map<string,SbPDG_Node*>& syms,
                               vector<string> inc_strings) {

  uint64_t ival;
  double dval;
  bool is_int = conv_to_int(var_out,ival);

  if(is_int) {
    assert(0 && "variable name cannot be a valid integer");
  }

  SbPDG_Inst* pdg_inst = new SbPDG_Inst(this);
  pdg_inst->setName(var_out);
  syms[var_out]=pdg_inst;
  pdg_inst->setInst(inst_from_config_name(opcode.c_str()));

  int imm_offset=0;
  for(unsigned i = 0; i < inc_strings.size(); ++i) {
    string var_in = inc_strings[i];

    bool is_int    = conv_to_int(var_in,ival);
    bool is_double = conv_to_double(var_in,dval);

    if(is_int && (ival!=0 || dval==0) ) { //some tricky logic here, tread lightly
      assert(imm_offset==0 && "only one immediate per instr. is allowed");
      pdg_inst->setImm(ival);
      pdg_inst->setImmSlot(i); 
      imm_offset=1;
    } else if (is_double) {
      assert(imm_offset==0 && "only one immediate per instr. is allowed");
      pdg_inst->setImm(SB_CONFIG::as_uint64(dval));
      pdg_inst->setImmSlot(i);
      imm_offset=1;
    } else {
      SbPDG_Node* inc_node = syms[var_in];
      if(inc_node==NULL) {
        cerr << "Could not find variable \"" + var_in + "\" \n";
        assert("0");
      } 
      connect(inc_node, pdg_inst,i,SbPDG_Edge::data);
    }
  }

  addInst(pdg_inst);
} 

vector<vector<int>> simple_pm(string& s) {
  int vec_len;
  istringstream(s)>>vec_len;
  vector<vector<int>> pm;
  for(int i = 0; i < vec_len; ++i) {
    std::vector<int> m;
    m.push_back(i);
    pm.push_back(m);
  }
  return pm;
}

SbPDG::SbPDG(string filename) : SbPDG() {

  string line;
  ifstream ifs(filename.c_str());

  if(ifs.fail()) {
    cerr << "Could Not Open: " << filename << "\n";
    exit(1);
  }

  regex re_input_vec("InputVec:\\s*(\\w+)\\s*\\[\\s*((:?(:?\\d+\\s*)+\\s*)\\,?\\s*)+\\]\\s*"); //bits num id
  regex re_output_vec("OutputVec:\\s*(\\w+)\\s*\\[\\s*((:?(:?\\d+\\s*)+\\s*)\\,?\\s*)+\\]\\s*"); 
  regex re_input_len("Input:\\s*(\\w+)\\s*\\[\\s*(\\d+)\\s*\\]\\s*");  //bits num id
  regex re_output_len("Output:\\s*(\\w+)\\s*\\[\\s*(\\d+)\\s*\\]\\s*");  
  regex re_input("Input:\\s*(\\w+)\\s*"); //bits id  
  regex re_output("Output:\\s*(\\w+)\\s*");                          //out
  regex re_Op3("(\\w+)\\s*=\\s*(\\w+)\\(\\s*(\\w+|\\d*\\.?\\d*)\\s*,\\s*(\\w+|\\d*\\.?\\d*)\\s*,\\s*(\\w+|\\d*\\.?\\d*)\\s*\\)");//id dep dep   -- 3 ip
  regex re_Op2("(\\w+)\\s*=\\s*(\\w+)\\(\\s*(\\w+|\\d*\\.?\\d*)\\s*,\\s*(\\w+|\\d*\\.?\\d*)\\s*\\)");//id dep dep -- 2 ip
  regex re_Op1("(\\w+)\\s*=\\s*(\\w+)\\(\\s*(\\w+|\\d*\\.?\\d*)\\s*\\)");//id dep dep -- 1 ip
  regex re_rename("(\\w+)\\s*=\\s*(\\w+)\\s*");//rename

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
    if(ModelParsing::StartsWith(line,"---")) {
      //need to skip
      _vecInputGroups.push_back({});
      _vecOutputGroups.push_back({});
    } else if (regex_search(line, m, re_input_len)) {
      string name = m[1];
      string vec_len = m[2];
      vector<vector<int>> pm = simple_pm(vec_len);
      //cout << "input_len:" << name << " " << vec_len << " " << pm.size() << "\n";
      addVecInput(name,pm,syms);

    } else if (regex_search(line, m, re_output_len)) {
      string name = m[1];
      string vec_len = m[2];
      vector<vector<int>> pm = simple_pm(vec_len);
      addVecOutput(name,pm,syms);

    } else if (regex_search(line, m, re_input)) {
      //cout << m[1] << " " << m[2] << " " << m[3] << "\n";
      string name = m[1];
      addScalarInput(name,syms);

    } else if (regex_search(line,m,re_input_vec)) {
      string name = m[1];
      string vec = m[2];
      parse_and_add_vec(name, line, syms, true /*input*/);

    } else if (regex_search(line,m,re_output_vec)) {
      string name = m[1];
      string vec = m[2];
      parse_and_add_vec(name, line, syms, false /*output*/);   
    
    } else if (regex_search(line,m,re_Op1)) {
      //cout << "o1:" << m[1] << " " << m[2] << " " << m[3] << "\n";
      string var_out = m[1];
      string opcode  = m[2];
      string var_in  = m[3];

      parse_and_add_inst(var_out,opcode,syms,{var_in});

    } else if (regex_search(line,m,re_Op2)) {     
      //cout << "o2:" << m[1] << " " << m[2] << " " << m[3] << " " << m[4] << "\n";

      string var_out = m[1];
      string opcode  = m[2];
      string var_in1 = m[3];
      string var_in2 = m[4];

      parse_and_add_inst(var_out,opcode,syms,{var_in1,var_in2});

     } else if (regex_search(line,m,re_Op3)) {
      //cout << "o2:" << m[1] << " " << m[2] << " " << m[3] << " " << m[4] << "\n";
      string var_out = m[1];
      string opcode  = m[2];
      string var_in1 = m[3];
      string var_in2 = m[4];
      string var_in3 = m[5];
      parse_and_add_inst(var_out,opcode,syms,{var_in1,var_in2,var_in3});


    } else if (regex_search(line,m,re_output)) {  //SCALAR OUTPUT ONLY
      //cout << "out:" << m[1] << "\n";    
      string var_out = m[1];
      addScalarOutput(var_out,syms);
    } else if (regex_search(line,m,re_rename)) {
      string var_out = m[1];
      string var_in  = m[2];
      SbPDG_Node* inc_node = syms[var_in];

      if(inc_node==NULL) {
        cerr << "Could not find \"" + var_in + "\" (rename) \n";
        assert("0");
      } 

      syms[var_out] = inc_node;

    } else {
      cout << "Line: \"" << line << "\"\n";
      assert(0&&"I don't know what this line is for\n");
    }


  }

  calc_minLats();
  check_for_errors();
}


std::string SbPDG_Edge::name() {
  std::stringstream ss;
  ss << _def->name() << "->" << _use->name();
  return ss.str();
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
void SbPDG_Inst::setImmSlot(int i) {
  assert(i <4);
  if((int)_ops.size()<=i) { 
     _ops.resize(i+1,NULL); 
   }

  _imm_slot=i;
}


//compute:actual compute called from SbPDG class (slightly modify this)
int SbPDG_Inst::compute(bool print, bool verif) {
  assert(_ops.size() <=3);

  if(_input_vals.size()==0) {
    _input_vals.resize(_ops.size());
  }
  assert(_input_vals.size() <= _ops.size());

  if(print) {
    _sbpdg->dbg_stream() << name() << " (" << _ID << "): ";
  }

  _discard=false;

  for(unsigned i = 0; i < _ops.size(); ++i) {
    if(immSlot() == (int)i) {
      _input_vals[i]=imm();
    } else {
      SbPDG_Node*  n =  _ops[i]->def();
      _input_vals[i] = n->get_value();
      if(n->discard()) {
        _discard=true;
      }
    }
    if(print) {
      _sbpdg->dbg_stream() << std::hex << _input_vals[i] << " ";
    }
  }

  
  _val=SB_CONFIG::execute(_sbinst,_input_vals,_reg,_discard,_back_array);

   if(print) {
    _sbpdg->dbg_stream() << " = " << _val << "\n";
  }

  if(verif) {
    if (_verif_stream.is_open()) {
      _verif_stream << hex << setw(16) << setfill('0') << _val << "\n";
      _verif_stream.flush();
    } else {
      _verif_stream.open(("verif/fu" + _verif_id + ".txt").c_str(), ofstream::trunc | ofstream::out);
      assert(_verif_stream.is_open());
    }
  }

  int num_computed=!_discard;

  for(auto iter = _uses.begin(); iter != _uses.end(); iter++) {
      SbPDG_Node* use = (*iter)->use();
      num_computed += use->inc_inputs_ready(print, verif); //recursively call compute
  }

  return num_computed;

}

// new compute for back cgra-----------------------------
int SbPDG_Inst::compute_backcgra(bool print, bool verif) {

  assert(_ops.size() <=3);

  if(_input_vals.size()==0) {
    _input_vals.resize(_ops.size());
  }
  assert(_input_vals.size() <= _ops.size());

  if(print) {
    _sbpdg->dbg_stream() << name() << " (" << _ID << "): ";
  }

  _discard=false;

  for(unsigned i = 0; i < _ops.size(); ++i) {
    if(immSlot() == (int)i) {
      _input_vals[i]=imm();
    } 
    else {
      SbPDG_Node*  n =  _ops[i]->def();
      _input_vals[i] = n->get_value();
      assert(n->get_value()!=1000000 && "Input node didn't have data :/\n");
      }
      /*if(n->discard()) {
        _discard=true;
        std::cout<< "discard of this input value is true?\n";
      }*/

    if(print) {
      _sbpdg->dbg_stream() << std::hex << _input_vals[i] << " ";
    }
  }

  // if discard of i/p is true, compute but set the output also discard:
  // shouldn't be the case for intermediate inputs, right?
  
  
  
  //if(_discard)
  //   return 0;
  
  
  // initializing back pressure
  if(_back_array.size() == 0){
      _back_array.resize(_ops.size());
      for(unsigned int i=0; i<_ops.size(); ++i) {
          _back_array[i] = 0;
      }
   }

  // Read in some temp value and set _val after inst_lat cycles
  int temp = 0;
  temp=SB_CONFIG::execute(_sbinst,_input_vals,_reg,_discard,_back_array);

  // std::cout << "Print input values: " << _input_vals[0] << " " << _input_vals[1] << "\n";
  // std::cout << "Intermediate vals calculated each cycle: " << temp << " and their discard values: " << _discard << "\n";

  // setting the output node (current node)
  SbPDG_Inst* pdginst = dynamic_cast<SbPDG_Inst*>(this); 
  this->set_value(temp, !_discard, inst_lat(pdginst->inst()));
  // _discard = true;


  // pop the inputs after inst_thr+back_press here
  int inst_throughput = inst_thr(pdginst->inst());
  // std::cout << "instruction throughput is " << inst_throughput << "\n";
  // std::cout << "instruction latency is " << inst_lat(pdginst->inst()) << "\n";


  int index=0;
  
   for(unsigned i = 0; i < _ops.size(); ++i) {

        SbPDG_Node*  n =  _ops[i]->def();


        // only if n is an input node, how do i check?
        if(_sbpdg->num_vec_input()!=0 && n->num_inc()==0) {

            int vec_id = _sbpdg->find_vec_for_scalar(n, index);
            SbPDG_VecInput* vec_in = _sbpdg->get_vector_input(vec_id); // check if something available

            if(index==0)
                vec_in->setBackBit(_back_array[i]); 
            if(vec_in->getBackBit()==1)
               n->set_value(_input_vals[i],true, inst_throughput);
            else
               n->set_value(1000000, true, inst_throughput);
        }


        else {
            if(_back_array[i]==1)
                n->set_value(_input_vals[i],true, inst_throughput);
            else
               n->set_value(1000000, true, inst_throughput);
        }
   }
            
            
        








  // wrong: just for test
  print = false; verif = false;

  if(print) {
    _sbpdg->dbg_stream() << " = " << _val << "\n";
  }

  // what is this?
  if(verif) {
    if (_verif_stream.is_open()) {
      _verif_stream << hex << setw(16) << setfill('0') << _val << "\n";
      _verif_stream.flush();
    } else {
      _verif_stream.open(("verif/fu" + _verif_id + ".txt").c_str(), ofstream::trunc | ofstream::out);
      assert(_verif_stream.is_open());
    }
  }

  // int num_computed=!_discard;

  // should be recursively called here?
  // update_next_nodes for that
  // return num_computed;
  return 1;
}




// Virtual function-------------------------------------------------


void SbPDG_Inst::update_next_nodes(bool print, bool verif){
  
  // if it is popping input from input node--don't do anything: done while
  // calling
  if(_val==1000000)
     return;

  int t = 0;
  // SbPDG_Node* use;

  for(auto iter = _uses.begin(); iter != _uses.end(); iter++) {
    SbPDG_Node* use = (*iter)->use();
    t = use->inc_inputs_ready_backcgra(print, verif); //recursively call compute
    // std::cout << "update next node with t: " << t << " value: " << _val << " and discard: " << use->discard() << "\n";
    if(t==0) { // t=1 for output nodes: I don't know
        use->set_value(_val, use->discard()); // assuming no discard now    
    }
  }

  
}


void SbPDG_Node::set_value(uint64_t v, bool valid, int cycle) {
    _sbpdg->push_transient(this, v,valid,cycle);
}


//------------------------------------------------------------------


void SbPDG_Node::printGraphviz(ostream& os, Schedule* sched) {
  
  string ncolor = "black";
  os << "N" << _ID << " [ label = \"" << name();

  if(sched) {
    os << "\\n lat=" << sched->latOf(this)  << " ";
  }
  os << "min:" << _min_lat;

  if(sched) {
    auto p = sched->lat_bounds(this);
    os << "\\n bounds=" << p.first << " to " << p.second;
    os << "\\n vio=" << sched->vioOf(this);
  }

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
    os << " label = \"";
    if(sched) {
      os << "l:" << sched->link_count(e) 
         << "\\nex:" << sched->edge_delay(e)
         << "\\npt:" << sched->num_passthroughs(e);
    }
      
    os << "\"];\n";
  }
 
  os << "\n";

}

void SbPDG_Inst::printGraphviz(ostream& os, Schedule* sched) {
  SbPDG_Node::printGraphviz(os,sched);
}

void SbPDG_Output::printGraphviz(ostream& os,Schedule* sched) {
  SbPDG_Node::printGraphviz(os,sched);
}

void SbPDG_Input::printGraphviz(ostream& os,Schedule* sched) {
  SbPDG_Node::printGraphviz(os,sched);
}

void SbPDG_Node::printEmuDFG(ostream& os, string dfg_name) {
  
  os << "The ID for this node is " << _ID << endl;
  os << "The name for this node is " << _name << endl;
  for(auto iter = _ops.begin(); iter != _ops.end(); iter++) {
    os << "Name for the ops " << (*iter)->name() << endl;  
  }
  for(auto iter = _uses.begin(); iter != _uses.end(); iter++) {
    os << "Name for the uses " << (*iter)->name() << endl;  
  }
  os << "The gams name is " << gamsName() << endl; 
  os << "\n";
}

int SbPDG_Node::findDepth(ostream& os, string dfg_name, int level) {
  int returned_level = level;
  for(auto name_iter = _uses.begin(); name_iter != _uses.end(); name_iter++) {
    if((*name_iter)->use()->output) {
      //Don't do anything
    } else {
      int candidate_level = (*name_iter)->use()->findDepth(os, dfg_name, level+1);
      if(candidate_level > returned_level) {
	returned_level = candidate_level;
      }
    }
  }
  return returned_level;
}

void SbPDG_Inst::printEmuDFG(ostream& os, string dfg_name) {
  //os << "INSTRUCTION " << dfg_name << "_" << _name << endl;
  auto name_iter = _uses.begin();
  if((*name_iter)->use()->output) {
    os << "   outputs[" << (*name_iter)->use()->_iter << "]";
    string outputArray = (*name_iter)->use()->name();
    if(!(*name_iter)->use()->getScalar()) {
      outputArray = outputArray.substr(0, outputArray.find_first_of(":"));
      if(outputArray.find_first_of("0123456789") < outputArray.length()) {
	outputArray = outputArray.substr(outputArray.find_first_of("0123456789"), outputArray.length());
	//this absolutely, should NOT have an _
	if(outputArray.find_first_of("_") < outputArray.length()) {
	  outputArray = outputArray.substr(0, outputArray.find_first_of("_"));
	}
	//Get subIter
	os << "[" << outputArray << "]  = ";
      } else {
	os << "[0] = ";
      }
    } else {
      os << "[0] = ";
    }
  } else {
    string outputArray = (*name_iter)->def()->name();
    outputArray = outputArray.substr(0, outputArray.find_first_of(":"));
    os << "   uint64_t " << outputArray << " = ";
  } 
  string instName = (*name_iter)->def()->name();
  instName = instName.substr(instName.find_first_of(":")+1, instName.length());
  os << instName << "(std::array<uint64_t,2>{";
  uint ops_amt = 1;
  for(auto ops_iter = _ops.begin(); ops_iter != _ops.end(); ops_iter++) {
    if((*ops_iter)->def()->input) {
      os << "inputs[" << (*ops_iter)->def()->_iter << "]";
      string inputArray = (*ops_iter)->def()->name();
      if(!(*ops_iter)->def()->getScalar()) {
	inputArray = inputArray.substr(0, inputArray.find_first_of(":"));
	if(inputArray.find_first_of("0123456789") < inputArray.length()) {
	  inputArray = inputArray.substr(inputArray.find_first_of("0123456789"), inputArray.length());
	  //Get subIter
	  os << "[" << inputArray << "]";
	} else {
	  os << "[0]";
	}
      } else {
	os << "[0]";
      }
    } else {
      string inputArray = (*ops_iter)->def()->name();
      inputArray = inputArray.substr(0, inputArray.find_first_of(":"));
      os << inputArray;
    }
    if(ops_amt != _ops.size()) {
      os << ",";
    } else {
      os << "});" << endl;
    }
    ops_amt++;
  }
  //SbPDG_Node::printEmuDFG(os, dfg_name);
}

void SbPDG_Output::printDirectAssignments(ostream& os, string dfg_name) {
  for(auto ops_iter = _ops.begin(); ops_iter != _ops.end(); ops_iter++) {
    if((*ops_iter)->def()->input) {
      //Print our formatted name and such
      os << "   outputs[" << _iter << "]";
      string outputArray = name();
      if(!getScalar()) {
	outputArray = outputArray.substr(0, outputArray.find_first_of(":"));
	if(outputArray.find_first_of("0123456789") < outputArray.length()) {
	  outputArray = outputArray.substr(outputArray.find_first_of("0123456789"), outputArray.length());
	  //this absolutely, should NOT have an _
	  if(outputArray.find_first_of("_") < outputArray.length()) {
	    outputArray = outputArray.substr(0, outputArray.find_first_of("_"));
	  }
	  //Get subIter
	  os << "[" << outputArray << "]  = ";
	} else {
	  os << "[0] = ";
	}
      } else {
	os << "[0] = ";
      }
      os << "inputs[" << (*ops_iter)->def()->_iter << "]";
      string inputArray = (*ops_iter)->def()->name();
      if(!(*ops_iter)->def()->getScalar()) {
	inputArray = inputArray.substr(0, inputArray.find_first_of(":"));
	if(inputArray.find_first_of("0123456789") < inputArray.length()) {
	  inputArray = inputArray.substr(inputArray.find_first_of("0123456789"), inputArray.length());
	  //Get subIter
	  os << "[" << inputArray << "]";
	} else {
	  os << "[0]";
	}
      } else {
	os << "[0]";
      }
      os << ";\n";
    }
  }
}

void SbPDG_Output::printEmuDFG(ostream& os, string dfg_name, string* realName, int* iter, vector<int>* output_sizes) {
  output = true;
  //First, split name into realName and subIter
  if((_name.find_first_of("0123456789") < _name.length()) && !_scalar) {
    _realName = _name.substr(0, _name.find_first_of("0123456789"));
    _realName = _realName.substr(0, _realName.find_last_of("_"));
    //Get subIter
    _subIter = atoi(_name.substr(_name.find_first_of("0123456789"), _name.length()).c_str());
  } else {
    _realName = _name;
    _realName = _realName.substr(0, _realName.find_last_of("_"));
    _subIter = 0;
  }
  if((realName->compare(_realName) == 0) && !_scalar) {
    //same input or output as last time, don't do anything
    _iter = *iter - 1;
    output_sizes->back() = output_sizes->back()+1;
  } else {
    _iter = *iter;
    _size = 1;
    *iter = (*iter) + 1;
    os << "#define P_" << dfg_name << "_"  << _realName << " " << _iter << endl;
    output_sizes->push_back(_size);
  }
  //SbPDG_Node::printEmuDFG(os, dfg_name);
  *realName = _realName;
}

void SbPDG_Input::printEmuDFG(ostream& os, string dfg_name, string* realName, int* iter, vector<int>* input_sizes) {
  //First, split name into realName and subIter
  input = true;
  if((_name.find_first_of("0123456789") < _name.length()) && !_scalar) {
    _realName = _name.substr(0, _name.find_first_of("0123456789"));
    //Get subIter
    _subIter = atoi(_name.substr(_name.find_first_of("0123456789"), _name.length()).c_str());
  } else {
    _realName = _name;
    _subIter = 0;
  }
  if(realName->compare(_realName) == 0) {
    //same input or output as last time, don't do anything
    _iter = *iter - 1;
    input_sizes->back() = input_sizes->back()+1;
  } else {
    _iter = *iter;
    _size = 1;
    *iter = (*iter) + 1;
    os << "#define P_" << dfg_name << "_"  << _realName << " " << _iter << endl;
    input_sizes->push_back(_size);
  }
  //SbPDG_Node::printEmuDFG(os, dfg_name);
  *realName = _realName;
}

//Connect two nodes in PDG
SbPDG_Edge* SbPDG::connect(SbPDG_Node* orig, SbPDG_Node* dest,int slot,SbPDG_Edge::EdgeType etype) {
 
  assert(orig != dest && "we only allow acyclic pdgs");

  SbPDG_Edge* new_edge = 0; //check if it's a removed edge first
  auto edge_it = removed_edges.find(make_pair(orig,dest));
  if(edge_it != removed_edges.end()) {
    new_edge = edge_it->second;
  } else { 
    new_edge = new SbPDG_Edge(orig, dest, etype, this);
  }

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
      //assert(slot >=0);
      //assert(slot <=1);
  }
  
  dest->addIncEdge(slot,new_edge);
  orig->addOutEdge(orig->num_out(),new_edge);
  _edges.push_back(new_edge);
  
  return new_edge;
}

//Disconnect two nodes in PDG
void SbPDG::disconnect(SbPDG_Node* orig, SbPDG_Node* dest) {
  
  assert(orig != dest && "we only allow acyclic pdgs");

  dest->removeIncEdge(orig);
  orig->removeOutEdge(dest);
  for (auto it=_edges.begin(); it!=_edges.end(); it++) {
    if ((*it)->def() == orig && (*it)->use() == dest) {
        removed_edges[make_pair(orig,dest)] = *it;
        _edges.erase(it);
        return;
    }
  }
  assert(false && "edge was not found");
}

int SbPDG::remappingNeeded(int num_HW_FU) {
  if(dummy_map.size()==0) {
    //Count the number of dummy nodes needed
    const_output_iterator Iout,Eout;
    for (Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)  {
      SbPDG_Output* pdg_out = (*Iout);
      SbPDG_Inst* inst = pdg_out->out_inst();
      SbPDG_Node* node = pdg_out->first_operand();
      //if producing instruction is an input or
      // if producing instruction has more than one uses
      if (!inst || inst->num_out() > 1) {
        SbPDG_Inst* newNode = new SbPDG_Inst(this);
        //TODO: insert information about this dummy node
        newNode->setInst(SB_CONFIG::sb_inst_t::SB_Copy);
        disconnect(node, pdg_out);
        connect(node, newNode, 0, SbPDG_Edge::data);
        connect(newNode, pdg_out, 0, SbPDG_Edge::data);
        addInst(newNode);
        newNode->setIsDummy(true);
        dummy_map[pdg_out] = newNode;
      }
    }
  }
  return dummy_map.size();
}

void SbPDG::removeDummies() {
  for (auto Ii=_insts.begin(),Ei=_insts.end();Ii!=Ei;++Ii)  {
    SbPDG_Inst* inst = *Ii;
    if(inst->isDummy()) {
       SbPDG_Node* input = inst->first_operand();
       SbPDG_Node* output = inst->first_use();
       disconnect(input,inst);
       disconnect(inst,output);
       connect(input,output,0,SbPDG_Edge::data);
    }
  }

  for(auto i : dummy_map) {
    removeInst(i.second); //remove from list of nodes
  }
  
  dummies.clear();
  dummiesOutputs.clear();
  dummys_per_port.clear();
}


void SbPDG::remap(int num_HW_FU) {
  //First Disconnect any Dummy Nodes  (this is n^2 because of remove, but w/e)
  removeDummies();

  for (auto Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)  {
    SbPDG_Output* pdg_out = (*Iout);
    SbPDG_Inst* inst = pdg_out->out_inst();
    SbPDG_Node* node = pdg_out->first_operand();

    if ((!inst || inst->num_out() > 1) && ((rand()&3)==0) ) { //25% chance
      disconnect(node, pdg_out);
      SbPDG_Inst* newNode = dummy_map[pdg_out]; //get the one we saved earlier
      connect(node, newNode, 0, SbPDG_Edge::data);
      connect(newNode, pdg_out, 0, SbPDG_Edge::data);
      addInst(newNode); //add to list of nodes -- TODO: check if this borked anything
      dummies.insert(newNode);
      dummiesOutputs.insert(pdg_out);

      if (num_insts() + (int)dummies.size() > num_HW_FU) {
        //cerr <<"No more FUs left, so we can't put more dummy nodes,\n"
        //     <<"  so probabily we will face problems when it comes to fix timing!\n";
        break;
      }

    }
  }
}

//We may have forgotten which dummies we included in the PDG, if we went on to
//some other solution.  This function recalls dummies and reconnects things
//appropriately
void SbPDG::rememberDummies(std::set<SbPDG_Output*> d) {
  removeDummies();

  for (auto Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)  {
    SbPDG_Output* pdg_out = (*Iout);
    SbPDG_Node* node = pdg_out->first_operand();

    if (d.count(pdg_out)) {
      disconnect(node, pdg_out);
      SbPDG_Inst* newNode = dummy_map[pdg_out]; //get the one we saved earlier
      connect(node, newNode, 0, SbPDG_Edge::data);
      connect(newNode, pdg_out, 0, SbPDG_Edge::data);
      addInst(newNode); //add to list of nodes -- TODO: check if this borked anything
      dummies.insert(newNode);
      dummiesOutputs.insert(pdg_out);
    }
  }
}



void SbPDG::printGraphviz(ostream& os, Schedule* sched)
{
  os << "Digraph G { \nnewrank=true;\n " ;
  const_inst_iterator Ii,Ei;

  //Insts
  for (Ii=_insts.begin(),Ei=_insts.end();Ii!=Ei;++Ii)  { 
    (*Ii)->printGraphviz(os,sched); 
  }
  
  const_input_iterator Iin,Ein;

  //Inputs
  for (Iin=_inputs.begin(),Ein=_inputs.end();Iin!=Ein;++Iin)  { 
    (*Iin)->printGraphviz(os,sched); 
  }
  
  const_output_iterator Iout,Eout;

  //Outputs
  for (Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)  { 
    (*Iout)->printGraphviz(os,sched); 
  }

  int cluster_num=0;

  os << "\n";
  for(auto& i : _vecInputs) {
    os << "subgraph cluster_" << cluster_num++ << " {" ;
    for(auto I=i->input_begin(),E=i->input_end();I!=E;++I) {
      SbPDG_Input* sbin = *I;
      os << "N" << sbin->id() << " ";
    }
    os << "}\n";
  }

  for(auto& i : _vecOutputs) {
    os << "subgraph cluster_" << cluster_num++ << " {" ;
    for(auto I=i->output_begin(),E=i->output_end();I!=E;++I) {
      SbPDG_Output* sbout = *I;
      os << "N" << sbout->id() << " ";
    }
    os << "}\n";
  }
  os << "\n";


  os << "\t{ rank = same; ";
  for (Iin=_inputs.begin(),Ein=_inputs.end();Iin!=Ein;++Iin)   { os << "N" << (*Iin)->id() << " ";  }
  os << "}\n";
  
  os << "\t{ rank = same; ";
  for (Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)   { os << "N" << (*Iout)->id() << " "; }
  os << "}\n";

  os << "}\n";
}

void SbPDG::printEmuDFG(ostream& os, string dfg_name)
{
  string realName = "";
  vector<int> input_sizes;
  int input_iter = 0;
  const_input_iterator Iin,Ein;
  os << "#include \"sb_emu.h\"" << endl;
  os << "#include \"sb_c_insts.h\" "<< endl << endl;

  os << "#define " << dfg_name << "_size 64" << endl;

  //Inputs
  int level = 1;
  for (Iin=_inputs.begin(),Ein=_inputs.end();Iin!=Ein;++Iin)  { 
    (*Iin)->printEmuDFG(os, dfg_name, &realName, &input_iter, &input_sizes); 
  }
  int output_iter = 0;
  const_output_iterator Iout,Eout;

  //Outputs
  vector<int> output_sizes;  
  for (Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)  { (*Iout)->printEmuDFG(os, dfg_name, &realName, &output_iter, &output_sizes); }

  for (Iin=_inputs.begin(),Ein=_inputs.end();Iin!=Ein;++Iin)  { 
    int new_level = (*Iin)->findDepth(os, dfg_name, 0);
    if(new_level > level) {
      level = new_level;
    }
  }

  const_inst_iterator Ii,Ei;

  int iter = 0;
  int ionodes = 0;
  //Insts
  os << endl << "inline void dfg_func_" << dfg_name << "(uint64_t** inputs, uint64_t** outputs) {" << endl;
  for (Ii=_insts.begin(),Ei=_insts.end();Ii!=Ei;++Ii)  { 
    (*Ii)->printEmuDFG(os, dfg_name); 
  }
  //Next, we need to assure no outputs are just direct assignments of an input;
   for (Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)  {
     (*Iout)->printDirectAssignments(os, dfg_name);
   }
   os << "}" << endl << endl << endl;
  os << "static sb_config " << dfg_name << "_config = {&dfg_func_" << dfg_name << ", " << input_iter << ", new int["  << input_iter << "]{";
  iter = 1;
  for(auto viter = input_sizes.begin(); viter != input_sizes.end(); viter++) {
    os << *viter;
    ionodes += *viter;
    if(iter != input_iter) {
      os << ",";
    }
    iter++;
  }
  os << "}, " << output_iter << ", new int[" << output_iter << "]{"; 
  iter = 1;
  for(auto viter = output_sizes.begin(); viter != output_sizes.end(); viter++) {
    os << *viter;
    ionodes += *viter;
    if(iter != output_iter) {
      os << ",";
    }
    iter++;
  }
  os << "}, " << num_nodes()-ionodes << ", "  << level << " };" << endl;
} 
 


//TODO: Fix this for more complex patterns
bool is_compatible(vector<vector<int>>& vec_m, vector<pair<int, vector<int>>>& port_m) {
  if(vec_m.size() > port_m.size()) {
    return false;
  }
  
/*  for(unsigned i = 0; i < vec_m.size(); ++i) {
    if(vec_m[i].size() != port_m[i].second.size()) {
      return false;
    }

    for(unsigned j = 0; j < vec_m[i].size(); ++j) {
      if(vec_m[i][j] != port_m[i].second[j]) {
        return false;
      }
    }
  }*/
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




void SbPDG::calc_minLats() {
  list<SbPDG_Node* > openset;
  set<bool> seen;
  for(auto I=input_begin(), E=input_end(); I!=E; ++I) {
    openset.push_back(*I);
    seen.insert(*I);
  }

  //populate the schedule object
  while(!openset.empty()) {
    SbPDG_Node* n = openset.front(); 
    openset.pop_front();

    int cur_lat = 0;

    for(auto I=n->ops_begin(), E=n->ops_end();I!=E;++I) {
      if(*I == 0) continue;
      SbPDG_Node* dn = (*I)->def();
      if(dn->min_lat() > cur_lat) {
        cur_lat = dn->min_lat();
      }
    }

    if(SbPDG_Inst* inst_n = dynamic_cast<SbPDG_Inst*>(n)) {
      cur_lat += inst_lat(inst_n->inst()) + 1;
    } else if(dynamic_cast<SbPDG_Input*>(n)) {
      cur_lat=0;      
    } else if(dynamic_cast<SbPDG_Output*>(n)) {
      cur_lat+=1;   
    }

    n->set_min_lat(cur_lat);

    for(auto I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Node* un = (*I)->use();

      bool ready = true;
      for(auto II=un->ops_begin(), EE=un->ops_end(); II!=EE; ++II) {
        if(*II == 0) continue;
        SbPDG_Node* dn = (*II)->def();
        if(!seen.count(dn)) {
          ready = false;
          break;
        }
      }
      if(ready) {
        seen.insert(un);
        openset.push_back(un);
      }
    }
  }



}

//Gams related
void SbPDG::printGams(std::ostream& os,
                      std::unordered_map<string,SbPDG_Node*>& node_map,
                      std::unordered_map<std::string,SbPDG_Edge*>& edge_map,
                      std::unordered_map<std::string, SbPDG_Vec*>& port_map) {
  
  os << "$onempty\n";
  
  os << "set v \"verticies\" \n /";   // Print the set of Nodes:
  
  for (auto Ii=_nodes.begin(),Ei=_nodes.end();Ii!=Ei;++Ii)  { 
    if(Ii!=_nodes.begin()) os << ", ";
    os << (*Ii)->gamsName();
    assert(*Ii);
    node_map[(*Ii)->gamsName()]=*Ii;
  }
  os << "/;\n";

  os << "set inV(v) \"input verticies\" /" ;   // Print the set of Nodes:
  const_input_iterator Iin,Ein;
  for (Iin=_inputs.begin(),Ein=_inputs.end();Iin!=Ein;++Iin)  { 
    if(Iin!=_inputs.begin()) os << ", ";
    assert(*Iin);
    os << (*Iin)->gamsName();
  }
  os << "/;\n";
  
  os << "set outV(v) \"output verticies\" /" ;   // Print the set of Nodes:
  const_output_iterator Iout,Eout;
  for (Iout=_outputs.begin(),Eout=_outputs.end();Iout!=Eout;++Iout)  { 
    if(Iout!=_outputs.begin()) os << ", ";
    os << (*Iout)->gamsName();
    assert(*Iout);
  }
  os << "/;\n";
  

  os << "parameter minT(v) \"Minimum Vertex Times\" \n /";
  for (auto Ii=_nodes.begin(),Ei=_nodes.end();Ii!=Ei;++Ii)  { 
    if(Ii!=_nodes.begin()) os << ", ";
    SbPDG_Node* n = *Ii;
    int l = n->min_lat();
    if(SbPDG_Inst* inst = dynamic_cast<SbPDG_Inst*>(n)) {
      l -= inst_lat(inst->inst());
    }
    os << n->gamsName() << " " << l;
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

  for (auto Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
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
  for (auto Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
    if(Ie!=_edges.begin()) os << ", ";
    
    SbPDG_Edge* edge = *Ie;  
    os << edge->def()->gamsName() << "." << edge->gamsName() << " 1";
  }
  os << "/;\n";
  
  os << "parameter Gev(e,v) \"edge to vertex\" \n /";   // use edges
  for (auto Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
    if(Ie!=_edges.begin()) os << ", ";
    
    SbPDG_Edge* edge = *Ie;  
    os << edge->gamsName() << "." << edge->use()->gamsName() << " 1";
  }
  os << "/;\n";
  
  os << "set intedges(e) \"edges\" \n /";   // Internal Edges
  first =true;
  for (auto Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
    SbPDG_Edge* edge = *Ie;  
    
    if(!dynamic_cast<SbPDG_Input*>(edge->def()) && !dynamic_cast<SbPDG_Output*>(edge->use()) ) {
      if (first) first = false;
      else os << ", ";
      os << edge->gamsName();
    }
  }
  os << "/;\n";
  
  os << "parameter delta(e) \"delay of edge\" \n /";   // Print the set of edges:
  for (auto Ie=_edges.begin(),Ee=_edges.end();Ie!=Ee;++Ie)  { 
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
