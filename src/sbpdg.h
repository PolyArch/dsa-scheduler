#ifndef __SBPDG_H__
#define __SBPDG_H__

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "sbinst.h"
#include <unordered_map>
#include <map>
#include <vector>
#include <assert.h>
#include <sstream>
#include <algorithm>
#include "model.h"

class Schedule;
class SbPDG_Node;
class SbPDG;

class SbPDG_Edge {
  public:
    enum EdgeType { data, ctrl_true, ctrl_false };
    
    EdgeType etype() {return _etype;}
    
    SbPDG_Edge(SbPDG_Node* def, SbPDG_Node* use, EdgeType etype, SbPDG* sbpdg) {
       _def=def;
       _use=use;
       _etype=etype;
       _ID=ID_SOURCE++;
       _sbpdg=sbpdg;
    }

    SbPDG_Node* def() const {return _def;}
    SbPDG_Node* use() const {return _use;}
    
    std::string gamsName();
    std::string name();

    void set_delay(int d) {_delay=d;}
    int delay() {return _delay;}
    //uint64_t data; //Vignesh
    //bool back_array[1] = {false}; //Vignesh
  private:
    int _ID;
    SbPDG* _sbpdg;  //sometimes this is just nice to have : )
    SbPDG_Node *_def, *_use;
    EdgeType _etype;

    int _delay =0;

  

  private:
    static int ID_SOURCE;
};

class SbPDG_Inst;

//PDG Node -- abstract base class
class SbPDG_Node {
 public:
    virtual void printGraphviz(std::ostream& os, Schedule* sched=NULL);
    virtual void printEmuDFG(std::ostream& os, std::string dfg_name);
    virtual uint64_t discard() {return _discard;} //execution-related

    

    void setScalar() {_scalar = true;}
    bool getScalar() {return _scalar;}
    int findDepth(std::ostream& os, std::string dfg_name, int level);
    SbPDG_Node(SbPDG* sbpdg) {
        _sbpdg=sbpdg;
        _ID=ID_SOURCE++;
    }
    
    typedef std::vector<SbPDG_Edge*>::const_iterator const_edge_iterator;
     
    void addIncEdge(unsigned pos, SbPDG_Edge *edge) { 
      _num_inc_edges++;
      assert(pos <=4);
      if(_ops.size()<=pos) { 
         _ops.resize(pos+1,NULL); 
       }

       if(_ops[pos]) {
         std::cerr << "ERROR: overwriting op at pos" << pos 
                   << " name:" << _ops[pos]->def()->name() << "\n";
         assert(0);
       }
       _ops[pos]=edge;
    }
    
    void addOutEdge(unsigned pos, SbPDG_Edge *edge) {
      assert(pos <= 64 && "more than 64 users, check this! (may be okay if really large grid\n"); 
       if(_uses.size()<=pos) { 
         _uses.resize(pos+1,NULL); 
       }

       if(_uses[pos]) {
         std::cerr << "ERROR: overwriting use at pos" << pos 
                   << " name: " << _uses[pos]->use()->name() << "\n";
         assert(0);
       }
       
       _uses[pos]=edge;
    }
    
    void removeIncEdge(SbPDG_Node* orig) { 
      _num_inc_edges--;
      for (unsigned i = 0; i < _ops.size(); ++i) {
        SbPDG_Edge* edge = _ops[i];
        if (edge->def() == orig) {
            _ops[i]=0;
            return;
        }
      }
      assert(false && "edge was not found");
    }
    
    void removeOutEdge(SbPDG_Node* dest) {
      for (auto it=_uses.begin(); it!=_uses.end(); it++) {
        if ((*it)->use() == dest) {
            _uses.erase(it);
            return;
        }
      }
      assert(false && "edge was not found");
    }

    virtual int compute(bool print, bool verif) {return 0;} 
 
    SbPDG_Edge* getLinkTowards(SbPDG_Node* to) {
       for(unsigned i = 0; i < _uses.size(); ++ i) {
         if(_uses[i] && _uses[i]->use()==to) {
           return _uses[i];
         }
       }
       return NULL;
    }

    virtual int maxThroughput() {
       if(_max_thr==0) {
         for (auto it=_uses.begin(); it!=_uses.end(); it++) {
           _max_thr=std::max(_max_thr,(*it)->use()->maxThroughput());
         }    
       }
       return _max_thr; 
    }

    virtual void depInsts(std::vector<SbPDG_Inst*>& insts) {
      for (auto it=_uses.begin(); it!=_uses.end(); it++) {
        SbPDG_Node* use = (*it)->use();
        if(std::find(insts.begin(),insts.end(),use)!=insts.end()) {
          use->depInsts(insts); 
        }
      }  
    }

    SbPDG_Node* first_operand() {
      return (_ops[0]->def());
    }

    SbPDG_Node* first_use() {
      return (_uses[0]->use());
    }

    int num_inc() const { return  _ops.size();  }
    int num_out() const { return  _uses.size(); }
    
    virtual std::string name() = 0;     //pure func
    void setName(std::string& name) {_name = name;}
    virtual std::string gamsName() = 0;
    
    const_edge_iterator ops_begin() const {return _ops.begin();}
    const_edge_iterator ops_end() const {return _ops.end();}
    const_edge_iterator uses_begin() const {return _uses.begin();}
    const_edge_iterator uses_end() const {return _uses.end();}
    
    int id() {return _ID;}
    
    void     set_value(uint64_t v, bool valid) {_val=v; _discard=!valid;}
    uint64_t get_value()           {return _val;}
    bool input = false;
    bool output = false;
    int _iter;
    //bool back_array = false; Vignesh
    //uint64_t backPressure_result; //Vignesh

    int min_lat() {return _min_lat;}
    void set_min_lat(int i) {_min_lat = i;}

    int sched_lat() {return _sched_lat;}
    void set_sched_lat(int i) {_sched_lat = i;}

    int inc_inputs_ready(bool print, bool verif) {
      _inputs_ready+=1;
      if(_inputs_ready == _num_inc_edges) {
        int num_computed = compute(print,verif);
        _inputs_ready=0;
        return num_computed;
      }
      return 0;
    }

  protected:    
    SbPDG* _sbpdg;  //sometimes this is just nice to have : )

    uint64_t _val; //dynamic var
    uint64_t _discard=false;
    int _inputs_ready=0; //dynamic inputs ready
    int _num_inc_edges=0; //number of incomming edges, not including immmediates

    int _ID;
    std::string _name;
    std::vector<SbPDG_Edge *> _ops;     //in edges 
    std::vector<bool>  _back_array;     //in edges 
    std::vector<SbPDG_Edge *> _uses;   //out edges  
    bool _scalar = false;
    int _min_lat=0;
    int _sched_lat=0;
    int _max_thr=0;

  private:
    static int ID_SOURCE;
};


//Instruction
class SbPDG_Inst : public SbPDG_Node {
  public:
    SbPDG_Inst(SbPDG* sbpdg) : SbPDG_Node(sbpdg), _predInv(false), _isDummy(false),
                     _imm_slot(-1), _subFunc(0) {
      _reg.resize(8,0);
    }


    void printGraphviz(std::ostream& os, Schedule* sched=NULL);
    virtual void printEmuDFG(std::ostream& os, std::string dfg_name);

    void setImm( uint64_t val ) { _imm=val; }
//    void setImm( float val ) { _imm=*reinterpret_cast<int32_t*>(&val); }

//    float getImmFloat() { return *reinterpret_cast<float*>(&_imm); } 
    int   getImmInt() { return _imm; }

    uint64_t   imm() { return _imm; }

    void setPredInv(bool predInv) { _predInv=predInv;}
    bool predInv() {return _predInv;}

    void setIsDummy(bool d) { _isDummy = d; }
    bool isDummy() { return _isDummy; }

    void setInst(SB_CONFIG::sb_inst_t sbinst) { _sbinst=sbinst; }
    SB_CONFIG::sb_inst_t inst() { return _sbinst; }

    virtual int maxThroughput() {
       if(_max_thr==0) {
         _max_thr=inst_thr(inst());
         for (auto it=_uses.begin(); it!=_uses.end(); it++) {
           _max_thr=std::max(_max_thr,(*it)->use()->maxThroughput());
         }    
       }
       return _max_thr; 
    }

    virtual void depInsts(std::vector<SbPDG_Inst*>& insts) {
      insts.push_back(this);
      for (auto it=_uses.begin(); it!=_uses.end(); it++) {
        SbPDG_Node* use = (*it)->use();
        if(std::find(insts.begin(),insts.end(),use)!=insts.end()) {
          use->depInsts(insts); 
        }
      }  
    }

    std::string name() {
        std::stringstream ss;
        ss << _name << ":";
        ss << SB_CONFIG::name_of_inst(_sbinst);
        if(_imm_slot!=-1) {
          ss<<" Imm:"<<_imm;
        }
        return ss.str();
    }

    std::string gamsName();

    void setImmSlot(int i);
    int immSlot() const { return _imm_slot; }

    void setSubFunc(int i) {_subFunc=i;}
    int subFunc() const {return _subFunc;}

    virtual int compute(bool print, bool verif); 

    void set_verif_id(std::string s) {_verif_id = s;}

    virtual uint64_t discard() {return _discard;}

  private:
    std::ofstream _verif_stream;
    std::string _verif_id;
    std::vector<uint64_t> _input_vals;
    bool _predInv;
    bool _isDummy;
    int _imm_slot;
    int _subFunc;
    std::vector<uint64_t> _reg;
    uint64_t _imm;
    SB_CONFIG::sb_inst_t _sbinst;
};

class SbPDG_IO : public SbPDG_Node {
  public:
  void setVPort(int vport) { _vport = vport; } 
  int vport() {return _vport;}

  SbPDG_IO(SbPDG* sbpdg) : SbPDG_Node(sbpdg) {}

  protected:
    SbPDG* _sbpdg;
    int _vport;
};


class SbPDG_Input : public SbPDG_IO {       //inturn inherits sbnode
  public:
    void printGraphviz(std::ostream& os, Schedule* sched=NULL);
    virtual void printEmuDFG(std::ostream& os, std::string dfg_name, std::string* realName, int* iter, std::vector<int>* input_sizes);
    
    SbPDG_Input(SbPDG* sbpdg) : SbPDG_IO(sbpdg) {}

    std::string name() {
        std::stringstream ss;
        ss << _name << ":";
        ss << "I" << _vport;
        ss << _name;
        return ss.str();
    }
    std::string gamsName();

    virtual int compute(bool print, bool verif) {
       int num_computed=0;
       for(auto iter = _uses.begin(); iter != _uses.end(); iter++) {
         SbPDG_Node* use = (*iter)->use();
         num_computed+=use->inc_inputs_ready(print, verif); //recursively call compute
       }
       return num_computed;
    }

    std::string _realName;
    int _subIter;
    int _size;
};

class SbPDG_Output : public SbPDG_IO {
  public:
    void printGraphviz(std::ostream& os, Schedule* sched=NULL);
    void printDirectAssignments(std::ostream& os, std::string dfg_name);
    virtual void printEmuDFG(std::ostream& os, std::string dfg_name, std::string* realName, int* iter, std::vector<int>* output_sizes);

    SbPDG_Output(SbPDG* sbpdg) : SbPDG_IO(sbpdg) {}

    std::string name() {
        std::stringstream ss;
        ss << _name << ":";
        ss << "O" << _vport;
        ss << _name;
        return ss.str();
    }
    std::string gamsName();

    //returns the instruction producing the
    //value to this output node
    //Returns NULL if the producing instruction is an input!
    SbPDG_Inst* out_inst() {
      return dynamic_cast<SbPDG_Inst*>(_ops[0]->def());
    }

    SbPDG_Node* first_operand() {
      return (_ops[0]->def());
    }
    //retrieve the value of the def
    uint64_t retrieve() {
      assert(_ops.size()==1);
      return _ops[0]->def()->get_value();
    }

    virtual uint64_t discard() {
      return _ops[0]->def()->discard();
    }

    std::string _realName;
    int _subIter;
    int _size;
};

//vector class
class SbPDG_Vec {
  public:
  SbPDG_Vec(std::string name, int id) : _name(name), _ID(id) {
    _locMap.resize(1); //set up default loc map
    _locMap[0].push_back(0);
  }

  void setLocMap(std::vector<std::vector<int> >& vec) { _locMap=vec;}
  std::vector<std::vector<int> >& locMap() {return _locMap;}

  int id() {return _ID;}
  
  virtual std::string gamsName() = 0;
  virtual std::string name() {return _name;}

  protected:
    std::string _name;
    std::vector<std::vector<int>> _locMap;
    int _ID;
};

static std::vector<std::vector<int>> simple_pm(int vec_len) {
  std::vector<std::vector<int>> pm;
  for(int i = 0; i < vec_len; ++i) {
    std::vector<int> m;
    m.push_back(i);
    pm.push_back(m);
  }
  return pm;
}
//static std::vector<std::vector<int>> simple_pm(std::string& s) {
//  int vec_len;
//  std::istringstream(s)>>vec_len;
//  return simple_pm(vec_len);
//}

class SbPDG_VecInput : public SbPDG_Vec {
  public:
    
  SbPDG_VecInput(std::string name, int id) : SbPDG_Vec(name,id) {}

  virtual std::string gamsName() {
    std::stringstream ss;
    ss << "IPV_" << _name ;
    return ss.str();
  }

  //Dynamic Function to communicate to simulator if there is backpressure on this cycle
  bool backPressureOn();

  void addInput(SbPDG_Input* in) { _inputs.push_back(in); }
  std::vector<SbPDG_Input*>::iterator input_begin() {return _inputs.begin();}
  std::vector<SbPDG_Input*>::iterator input_end() {return _inputs.end();}
  unsigned num_inputs() const {return _inputs.size();}

  SbPDG_Input* getInput(int i) {return _inputs[i];}

	/*bool operator < (const SbPDG_VecInput& s) const
  {
     return (this->num_inputs() > s.num_inputs());
  }*/

  private:
    std::vector<SbPDG_Input*> _inputs;
};


class SbPDG_VecOutput : public SbPDG_Vec {
  public:

  SbPDG_VecOutput(std::string name, int id) : SbPDG_Vec(name,id) {}

  virtual std::string gamsName() {
    std::stringstream ss;
    ss << "OPV_" << _name ;
    return ss.str();
  }

  void addOutput(SbPDG_Output* out) { _outputs.push_back(out); }
  std::vector<SbPDG_Output*>::iterator output_begin() {return _outputs.begin();}
  std::vector<SbPDG_Output*>::iterator output_end() {return _outputs.end();}
  unsigned num_outputs() const {return _outputs.size();}

  SbPDG_Output* getOutput(int i) {return _outputs[i];}

	/*bool operator < (const SbPDG_VecOutput& s) const
  {
     return (this->num_outputs() > s.num_outputs());
  }*/

  private:
    std::vector<SbPDG_Output*> _outputs;
};

struct SymEntry {
  enum enum_type {SYM_INV,SYM_INT, SYM_DUB, SYM_NODE};
  enum enum_type type;
  union union_data {
    uint64_t i;
    double d;	 
    SbPDG_Node* node;
  } data;
  SymEntry() {
    
  }
  SymEntry(uint64_t i) {
    type=SYM_INT;
    data.i=i;
  }
  SymEntry(double d) {
    type=SYM_INT;
    data.d=d;
  }
  SymEntry(SbPDG_Node* node) {
    type=SYM_INT;
    data.node=node;
  }
};

class SymTab {
  std::map<std::string, SymEntry> _sym_tab;
  void assert_exists(std::string& s) {
    if(_sym_tab.count(s)==0) {
       std::cerr << "Could not find" + s + "\n";
       assert("0");
    }
  }
public:
  SymEntry get_node(std::string& s) { 
    assert_exists(s);
    return _sym_tab[s];
  }
  SbPDG_Node* get_node(std::string& s) { 
    assert_exists(s);
    if(_sym_tab[s].type!=SymEntry::SYM_NODE) {
      std::cerr << "symbol \"" + s +"\" is not a node\"";
    }
    return _sym_tab[s].data.node;
  }
  void set(std::string& s, SbPDG_Node* n) {_sym_tab[s]=SymEntry(n);}
  void set(std::string& s, uint64_t n)    {_sym_tab[s]=SymEntry(n);}
  void set(std::string& s, float n)       {_sym_tab[s]=SymEntry(n);}

  int get_int(std::string& s) { 
    assert_exists(s);
    if(_sym_tab[s].type!=SymEntry::SYM_INT) {
      std::cerr << "symbol \"" + s +"\" is not an int\"";
    }
    return _sym_tab[s].data.i;
  }
  int get_float(std::string& s) { 
    assert_exists(s);
    if(_sym_tab[s].type!=SymEntry::SYM_DUB) {
      std::cerr << "symbol \"" + s +"\" is not a double\"";
    }
    return _sym_tab[s].data.d;
  }
};

class SbPDG {

  public:
    SbPDG();
    SbPDG(std::string filename);

    ~SbPDG(){
    }

    void remap(int num_HW_FU);
    int  remappingNeeded(int num_HW_FU);
    void rememberDummies(std::set<SbPDG_Output*> d);
    void removeDummies();


    void printGraphviz(std::ostream& os, Schedule* sched=NULL);
    void printEmuDFG(std::ostream& os, std::string dfg_name);
    void printGraphviz(const char *fname, Schedule* sched=NULL) {
      std::ofstream os(fname);
      assert(os.good());
      printGraphviz(os, sched);
    }
    
    void printGams(std::ostream& os, std::unordered_map<std::string,SbPDG_Node*>&,
                                     std::unordered_map<std::string,SbPDG_Edge*>&,
                                     std::unordered_map<std::string, SbPDG_Vec*>&);

    void printPortCompatibilityWith(std::ostream& os, SB_CONFIG::SbModel* sbModel);


    void addInst(SbPDG_Inst* inst) {
        _insts.push_back(inst); 
        _nodes.push_back(inst);
    }

    // remove instruction from nodes and insts
    void removeInst(SbPDG_Inst* inst)
    {
      _insts.erase(std::remove(_insts.begin(), _insts.end(), inst), _insts.end());
      _nodes.erase(std::remove(_nodes.begin(), _nodes.end(), inst), _nodes.end());
  
      //_insts.erase(inst);
      //_nodes.erase(inst);
    }

    //Just for adding single input without keeping track of name/sym-table
    void addInput(SbPDG_Input* input) {
        _inputs.push_back(input); 
        _nodes.push_back(input);
    }

    void addOutput(SbPDG_Output* output) {
        _outputs.push_back(output); 
        _nodes.push_back(output);
    }

    void addScalarInput(std::string name, SymTab& syms) {
      SbPDG_VecInput* vec_input = new SbPDG_VecInput(name, _vecInputs.size()); 
      insert_vec_in(vec_input);
 
      SbPDG_Input* pdg_in = new SbPDG_Input(this);  //new input nodes
      syms.set(name,pdg_in);
      pdg_in->setName(name);
      pdg_in->setVPort(_vecInputs.size());
      pdg_in->setScalar();
      addInput(pdg_in);
      vec_input->addInput(pdg_in);
    } 

    //scalar output node
    void addScalarOutput(std::string name, SymTab& syms) {
      SbPDG_Node* out_node = syms.get_node(name);
 
      //new vector output
      SbPDG_VecOutput* vec_output = new SbPDG_VecOutput(name,_vecOutputs.size()); 
      insert_vec_out(vec_output);
 
      SbPDG_Output* pdg_out = new SbPDG_Output(this);
      std::string out_name=name+"_out";
      syms.set(out_name,pdg_out);
      pdg_out->setName(out_name);
      pdg_out->setVPort(_vecOutputs.size());
      pdg_out->setScalar();
      addOutput(pdg_out);
      vec_output->addOutput(pdg_out);       //its own vector of out nodes

      connect(out_node, pdg_out,0,SbPDG_Edge::data);
    } 

    void addVecOutput(std::string name,
                     std::vector<std::vector<int> >& pm,
                     SymTab& syms ) {
        
      SbPDG_VecOutput* vec_output = new SbPDG_VecOutput(name,_vecOutputs.size()); 
      vec_output->setLocMap(pm);
      insert_vec_out(vec_output); 
       
      int entries = pm.size();
      //std::cout << "entries: " << entries << "\n";

      for(int i = 0; i < entries; ++i) {
        std::stringstream ss;
        ss << name << i;
        //std::cout << "name: " << name << "\n";
        std::string dep_name = ss.str();

        SbPDG_Node* out_node = syms.get_node(dep_name);
        SbPDG_Output* pdg_out = new SbPDG_Output(this);
        std::string out_name = dep_name + "_out";
        syms.set(out_name,pdg_out); 
        pdg_out->setName(out_name);
        pdg_out->setVPort(_vecOutputs.size());
        addOutput(pdg_out);
        vec_output->addOutput(pdg_out);

        connect(out_node, pdg_out,0,SbPDG_Edge::data);
      } 


      //assert(0 && "addVecOutput not implemented");
    }
    void addVecOutput(std::string name, int len, SymTab& syms ) {
       std::vector<std::vector<int>> pm = simple_pm(len);
       addVecOutput(name,pm,syms);
    }

    void addVecInput(std::string name,
                     std::vector<std::vector<int> >& pm,
                     SymTab& syms ) {

      SbPDG_VecInput* vec_input = new SbPDG_VecInput(name,_vecInputs.size()); 
      vec_input->setLocMap(pm);
      insert_vec_in(vec_input);

      //number of vector entries -- each vector element is a input node
      int entries = pm.size();
      //std::cout << "entries: " << entries << "\n";

      for(int i = 0; i < entries; ++i) {
        std::stringstream ss;
        ss << name << i;                //Vector input names: A0, A1
        //std::cout << "name: " << name << "\n";
        SbPDG_Input* pdg_in = new SbPDG_Input(this);
        std::string name = ss.str();
        syms.set(name,pdg_in);
        pdg_in->setName(name);
        pdg_in->setVPort(_vecInputs.size());
        addInput(pdg_in);
        vec_input->addInput(pdg_in);
      }
    }
    void addVecInput(std::string name, int len, SymTab& syms) {
      std::vector<std::vector<int>> pm = simple_pm(len);
      this->addVecInput(name,pm,syms);
    }
    //void parse_and_add_vec(std::string name, std::string line,
    //                       std::map<std::string,SbPDG_Node*>& syms ,bool input);

    SbPDG_Edge* connect(SbPDG_Node* orig, SbPDG_Node* dest,int slot,SbPDG_Edge::EdgeType etype);
    void disconnect(SbPDG_Node* orig, SbPDG_Node* dest);

    SymEntry* createInst(std::string opcode, std::vector<SymEntry>* args);

    /*void parse_and_add_inst(std::string var_out, std::string opcode, 
                            std::map<std::string,SbPDG_Node*>& syms,
                            std::vector<std::string> inc_nodes);*/
    
    typedef std::vector<SbPDG_Node*>::const_iterator   const_node_iterator;
    typedef std::vector<SbPDG_Inst*>::const_iterator   const_inst_iterator;
    typedef std::vector<SbPDG_Input*>::const_iterator  const_input_iterator;
    typedef std::vector<SbPDG_Output*>::const_iterator const_output_iterator;
    typedef std::vector<SbPDG_Edge*>::const_iterator   const_edge_iterator;

    const_node_iterator nodes_begin() {return _nodes.begin();}
    const_node_iterator nodes_end() {return _nodes.end();}
    int num_nodes() {return _nodes.size();}

    const_inst_iterator inst_begin() {return _insts.begin();}
    const_inst_iterator inst_end() {return _insts.end();}
    int num_insts() {return _insts.size();}
    
    const_input_iterator input_begin() {return _inputs.begin();}
    const_input_iterator input_end() {return _inputs.end();}
    int num_inputs() { return _inputs.size(); }
 
    const_output_iterator output_begin() {return _outputs.begin();}
    const_output_iterator output_end() {return _outputs.end();}
    int num_outputs() { return _outputs.size(); }


    int num_vec_input() {return _vecInputs.size();}
    int num_vec_output() {return _vecOutputs.size();}

    void insert_vec_in(SbPDG_VecInput*    in) {
      _vecInputs.push_back(in);
      _vecInputGroups[_vecInputGroups.size()-1].push_back(in);
    }
    void insert_vec_out(SbPDG_VecOutput*    out) {
      _vecOutputs.push_back(out);
      _vecOutputGroups[_vecOutputGroups.size()-1].push_back(out);
    }

    void insert_vec_in_group(SbPDG_VecInput* in, unsigned group) {
      _vecInputs.push_back(in);
      if(_vecInputGroups.size() <= group) {
       _vecInputGroups.resize(group+1);
      }
      _vecInputGroups[group].push_back(in);
    }
    void insert_vec_out_group(SbPDG_VecOutput* out, unsigned group) {
      _vecOutputs.push_back(out);
      if(_vecOutputGroups.size() <= group) {
       _vecOutputGroups.resize(group+1);
      }
      _vecOutputGroups[group].push_back(out);
    }



    int find_group_for_vec(SbPDG_VecInput* in) {
      for(unsigned i =0; i < _vecInputGroups.size(); ++i) {
        for(SbPDG_VecInput* v : _vecInputGroups[i]) {
          if(v == in) {
            return i;
          }
        }
      }
      assert(0 && "Vec Input not found");
    }
    int find_group_for_vec(SbPDG_VecOutput* out) {
      for(unsigned i =0; i < _vecOutputGroups.size(); ++i) {
        for(SbPDG_VecOutput* v : _vecOutputGroups[i]) {
          if(v == out) {
            return i;
          }
        }
      }
      assert(0 && "Vec Output not found");
    }

    SbPDG_VecInput*  vec_in(int i) {return _vecInputs[i];}
    SbPDG_VecOutput* vec_out(int i) {return _vecOutputs[i];}

    std::vector<SbPDG_VecInput*>&  vec_in_group(int i) {return _vecInputGroups[i];}
    std::vector<SbPDG_VecOutput*>&  vec_out_group(int i) {return _vecOutputGroups[i];}
    int num_groups() {return _vecInputGroups.size();}

    void sort_vec_in() {
    	sort(_vecInputs.begin(), _vecInputs.end(),[](SbPDG_VecInput*& left, SbPDG_VecInput*& right){
    		return left->num_inputs() > right->num_inputs();
    	});
    }
    
    void sort_vec_out() {
    	sort(_vecOutputs.begin(), _vecOutputs.end(),[](SbPDG_VecOutput*& left, SbPDG_VecOutput*& right){	
    		return left->num_outputs() > right->num_outputs();
    	});
    }
    int compute(bool print, bool verif, int group); 
    int maxGroupThroughput(int group); 
    void instsForGroup(int g, std::vector<SbPDG_Inst*>& insts);


    std::set<SbPDG_Output*> getDummiesOutputs() {return dummiesOutputs;}

    void calc_minLats();

    void set_dbg_stream(std::ostream* dbg_stream) {_dbg_stream=dbg_stream;}
    std::ostream& dbg_stream() {return *_dbg_stream;}

    void check_for_errors();

  private:
    std::vector<SbPDG_Node*> _nodes;
    
    //redundant storage:
    std::vector<SbPDG_Inst*> _insts;
    std::vector<SbPDG_Input*> _inputs;
    std::vector<SbPDG_Output*> _outputs;

    std::vector<SbPDG_Inst*> _orderedInsts;
    std::vector<std::vector<SbPDG_Inst*>> _orderedInstsGroup;

    std::vector<SbPDG_VecInput*> _vecInputs;
    std::vector<SbPDG_VecOutput*> _vecOutputs;

    std::vector<SbPDG_Edge*> _edges;
    
    std::map<std::pair<SbPDG_Node*,SbPDG_Node*>,SbPDG_Edge*> removed_edges;

    std::vector<std::vector<SbPDG_VecInput*>> _vecInputGroups;
    std::vector<std::vector<SbPDG_VecOutput*>> _vecOutputGroups;

    int _total_dyn_insts=0;

    //Dummy Stuffs:
    std::map<SbPDG_Output*,SbPDG_Inst*> dummy_map;
    std::map<SbPDG_Node*,int> dummys_per_port;
    std::set<SbPDG_Inst*> dummies;
    std::set<SbPDG_Output*> dummiesOutputs;

    std::ostream* _dbg_stream;

    int span;
    int work;
};

#endif
