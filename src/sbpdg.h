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
#include <list>
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

    SbPDG_Node* def() const {
      return _def;
    }
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




    // SERIOUS PROBLEMS WITH THESE 2 FUNCTIONS: ONE ALWAYS RETURNS TRUE AND
    // OTHER ALWAYS FALSE

    // some issue with this function
    virtual uint64_t discard() {
        // std::cout << "came in node class's discard to send back: " << _discard << "\n";
        return _discard;} //execution-related

    // for test---------------------------
    /*bool get_discard() {
        return _discard;
    }*/
    //------------------------------------

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

    virtual int compute(bool print, bool verif) {
      return 0;
    } 

    //Extra function-----------------------------------------
    virtual int update_next_nodes(bool print, bool verif) {
        // comes here if we set some value of i/p node
        int num_computed = 0;
        if(get_avail()){
          SbPDG_Node* n = this->first_use();
          num_computed = n->inc_inputs_ready_backcgra(print, verif);
        }
        return num_computed;
    }
    
      
    virtual int compute_backcgra(bool print, bool verif) {
     // if 'this' is a node and not an instruction
     assert(this->num_inc()==1 && "not an output node?\n");
     assert(this->num_out()==0 && "not an output node?\n");
     auto it = this->ops_begin();
     SbPDG_Node* n = (*it)->def();
     // std::cout << "In virtual function: val: "<< n->get_value() << " and avail: " << n->get_avail() << "\n";
     if(!n->discard()){
       this->set_node(n->get_value(), n->discard(), n->get_avail());
     }
     // unset the prev_inst
     n->set_node(0, true, false);
     _inputs_ready=0; // hopefully if it comes from inc_inputs_ready
     return 0;
    } 

    // void inc_inputs_wait(bool _back_array[]);

    //-------------------------------------------------------
    
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
    
    void     set_value(uint64_t v, bool valid) {
      _val=v; 
      _discard=!valid;
    }


    void set_node(uint64_t v, bool valid, bool avail) {
      _val=v; 
      _discard=!valid;
      _avail = avail;
    }

     // sets value at this cycle
     void set_value(uint64_t v, bool valid, bool avail, int cycle);
    //--------------------------------------------

    uint64_t get_value() {return _val;}
    bool get_avail() {return _avail;}
    bool input = false;
    bool output = false;
    int _iter;

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
    
    int inc_inputs_ready_backcgra(bool print, bool verif) {
      if(_inputs_ready!=_num_inc_edges){
          _inputs_ready+=1;
      }
      // std::cout << "came to increase inputs ready by 1 " << name() << "with inputs_ready: " << _inputs_ready << " and required: " << _num_inc_edges << "\n";
      // new compute cannot enter when it is already computing some other value
      // even if the inputs are ready
      // if(_inputs_ready == _num_inc_edges && this->get_value()==1000000) {
      if(_inputs_ready == _num_inc_edges && !this->get_avail()) {
        int num_computed = compute_backcgra(print,verif);//it's 0 or 1
        // _inputs_ready=0;
        return num_computed;
      }
      // this means that sufficient inputs were not present: not output node
      return 0;
    }

    int inc_reset_req(){
        return ++reset_req_times;
    }
    void reset_reset_req(){
        reset_req_times=0;
    }

   //---------------------------------------------------------------------------
    
    
    protected:    
    SbPDG* _sbpdg;  //sometimes this is just nice to have : )

    uint64_t _val = 0; //dynamic var (setting the default value)
    bool _avail = false; // new variable
    // uint64_t _val; //dynamic var
    int reset_req_times=0;
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

    //Adding new function in the header file
    int update_next_nodes(bool print, bool verif);

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

    // new line added
    virtual int compute_backcgra(bool print, bool verif);

    void set_verif_id(std::string s) {_verif_id = s;}

    virtual uint64_t discard() {
        return _discard;
    }


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
    // newly added: for now size of 1 
    // std::pair<std::vector<int>,int> _input_bufs;

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

    // just the same function?
    virtual int compute_backcgra(bool print, bool verif) {
       int num_computed=0;
       for(auto iter = _uses.begin(); iter != _uses.end(); iter++) {
         // std::cout << "starts computation for this vector at an input\n";
         SbPDG_Node* use = (*iter)->use();
         num_computed += use->inc_inputs_ready_backcgra(print, verif);
       }
       return num_computed;
    }
    //---------------------------------------

    virtual int compute(bool print, bool verif) {
       int num_computed=0;
       for(auto iter = _uses.begin(); iter != _uses.end(); iter++) {
         SbPDG_Node* use = (*iter)->use();
   
         num_computed += use->inc_inputs_ready(print, verif);
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

class SbPDG_VecInput : public SbPDG_Vec {
  public:

  SbPDG_VecInput(std::string name, int id) : SbPDG_Vec(name,id) {}

  virtual std::string gamsName() {
    std::stringstream ss;
    ss << "IPV_" << _name ;
    return ss.str();
  }

  bool backPressureOn();

  void addInput(SbPDG_Input* in) { _inputs.push_back(in); }
  std::vector<SbPDG_Input*>::iterator input_begin() {return _inputs.begin();}
  std::vector<SbPDG_Input*>::iterator input_end() {return _inputs.end();}
  unsigned num_inputs() const {return _inputs.size();}

  // scalar input corresponding to index i
  SbPDG_Input* getInput(int i) {return _inputs[i];}

  //---------------------------------
  int getBackBit() {return back_bit;}
  void setBackBit(int i) {
      std::cout << "came to set back bit here as: " << i << "\n";
      back_bit=i;}
  //---------------------------------

	/*bool operator < (const SbPDG_VecInput& s) const
  {
     return (this->num_inputs() > s.num_inputs());
  }*/

  private:
    std::vector<SbPDG_Input*> _inputs;
    int back_bit = 0; // new
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



    SbPDG_Inst* CreateInst() {
      return new SbPDG_Inst(this);
    }

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

    void addScalarInput(std::string name, std::map<std::string, SbPDG_Node*>& syms) {
      SbPDG_VecInput* vec_input = new SbPDG_VecInput(name, _vecInputs.size()); 
      insert_vec_in(vec_input);
 
      SbPDG_Input* pdg_in = new SbPDG_Input(this);  //new input nodes
      syms[name]=pdg_in;
      pdg_in->setName(name);
      pdg_in->setVPort(_vecInputs.size());
      pdg_in->setScalar();
      addInput(pdg_in);
      vec_input->addInput(pdg_in);
    } 

    //scalar output node
    void addScalarOutput(std::string name, std::map<std::string,SbPDG_Node*>& syms) {

      SbPDG_Node* out_node = syms[name];
      if(out_node==NULL) {
        std::cerr << "Could not find" + name + "\n";
        assert("0");
      }

      //new vector output
      SbPDG_VecOutput* vec_output = new SbPDG_VecOutput(name,_vecOutputs.size()); 
      insert_vec_out(vec_output);
 
      SbPDG_Output* pdg_out = new SbPDG_Output(this);
      std::string out_name=name+"_out";
      syms[out_name]=pdg_out;
      pdg_out->setName(out_name);
      pdg_out->setVPort(_vecOutputs.size());
      pdg_out->setScalar();
      addOutput(pdg_out);
      vec_output->addOutput(pdg_out);       //its own vector of out nodes

      connect(out_node, pdg_out,0,SbPDG_Edge::data);
    } 


    //Need to confirm the functionality here
    void addVecOutput(std::string name,
                     std::vector<std::vector<int> >& pm,
                     std::map<std::string,SbPDG_Node*>& syms ) {
        
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

        SbPDG_Node* out_node = syms[dep_name];
        if(out_node==NULL) {
          std::cerr << "Could not find \"" + dep_name + "\"\n";
          assert(0);
        }

        SbPDG_Output* pdg_out = new SbPDG_Output(this);
        std::string out_name = dep_name + "_out";
        syms[out_name]=pdg_out;
        pdg_out->setName(out_name);
        pdg_out->setVPort(_vecOutputs.size());
        addOutput(pdg_out);
        vec_output->addOutput(pdg_out);

        connect(out_node, pdg_out,0,SbPDG_Edge::data);
      } 


      //assert(0 && "addVecOutput not implemented");
    }

    void addVecInput(std::string name,
                     std::vector<std::vector<int> >& pm,
                     std::map<std::string,SbPDG_Node*>& syms ) {

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
        syms[name]=pdg_in;
        pdg_in->setName(name);
        pdg_in->setVPort(_vecInputs.size());
        addInput(pdg_in);
        vec_input->addInput(pdg_in);
      }
    }

    void parse_and_add_vec(std::string name, std::string line,
                           std::map<std::string,SbPDG_Node*>& syms ,bool input);

    SbPDG_Edge* connect(SbPDG_Node* orig, SbPDG_Node* dest,int slot,SbPDG_Edge::EdgeType etype);
    void disconnect(SbPDG_Node* orig, SbPDG_Node* dest);

    void parse_and_add_inst(std::string var_out, std::string opcode, 
                            std::map<std::string,SbPDG_Node*>& syms,
                            std::vector<std::string> inc_nodes);
    
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
      // add to the group we are creating right now
      // std::cout << "It does come in inserting vec_in" << "\n";
      _vecInputGroups[_vecInputGroups.size()-1].push_back(in);
    }
    void insert_vec_out(SbPDG_VecOutput*    out) {
      _vecOutputs.push_back(out);
      _vecOutputGroups[_vecOutputGroups.size()-1].push_back(out);
    }


    void insert_vec_in_group(SbPDG_VecInput* in, unsigned group) {
      if(_vecInputGroups.size() <= group) {
       _vecInputGroups.resize(group+1);
      }
      _vecInputGroups[group].push_back(in);
    }
    void insert_vec_out_group(SbPDG_VecOutput* out, unsigned group) {
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

//------------------------------
 int find_vec_for_scalar(SbPDG_Node* in, int &index) {
      for(unsigned int i =0; i < _vecInputs.size(); ++i) {
          SbPDG_VecInput* vec_in = _vecInputs[i];
          for(unsigned int j=0; j<vec_in->num_inputs(); ++j) {
              SbPDG_Node* v = dynamic_cast<SbPDG_Node*>(_vecInputs[i]->getInput(j));
              // std::cout << "Vector_id: " << i << " and v: " << v << " and in: " << in << "\n";
              if(v == in) {
                 index=j;
                 return i;
              }
          }
    }
      assert(0 && "Scalar input not found\n");
}

SbPDG_VecInput* get_vector_input(int i){
    return _vecInputs[i];
}

    //------------------


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
    int compute(bool print, bool verif, int group);  //atomically compute
    int maxGroupThroughput(int group); 
    void instsForGroup(int g, std::vector<SbPDG_Inst*>& insts);


    // --- New Cycle-by-cycle interface for more advanced CGRA -----------------

    
    //Simulator pushes data to vector given by vector_id
    bool push_vector(SbPDG_VecInput* vec_in, std::vector<uint64_t> data) {
      //std::cout<< "pushing data: " << data[0] << " and " << data[1] << "\n";
      assert(data.size() == vec_in->num_inputs() && "insufficient data available");
      // int num_computed = 0;
      int t=0;
      for (unsigned int i =0 ; i<vec_in->num_inputs(); ++i){
        SbPDG_Input* temp = vec_in->getInput(i); 
        // temp->set_value(data[i],true); //when push, discard is false?
        temp->set_node(data[i],true, true); //when push, discard is false?
        // change if this works
        // temp->update_next_nodes(false, false);
        // num_computed += temp->compute_backcgra(false, false);
        t = temp->update_next_nodes(false, false);

     }
      if(t>0)
          return true;
      else
          return false;

   }

    // check if some value present at input node or there is some backpressure or invalid value
    bool can_push_input(SbPDG_VecInput* vec_in){
      // can push if whole vector is available
      for(unsigned int i=0; i<vec_in->num_inputs(); ++i) {
        SbPDG_Input* temp = vec_in->getInput(i);
        // std::cout << "Is there data already? " << temp->get_avail() << "\n";
        if(temp->get_avail())
          return false;
      }
      return true;
    }

    //Advances simulation by one cycle  (return whether there was activity)
    int cycle_compute(bool print, bool verif);
    int compute_backcgra(SbPDG_VecInput* vec_in, bool print, bool verif);


    // bool none_config = false;
    bool backcgra_cycle(SbPDG_VecInput* vec_in) {

    // if(num_inputs() == num_outputs())
       //     none_config = true;
       //     inactive now!
      /*  
      if(none_config) {
        for (unsigned int i=0; i<vec_in->num_inputs(); ++i) {
            SbPDG_Node* n = vec_in->getInput(i);
            assert(n->num_out()==1 && "incorrectly detected none config\n");
            if(n->get_value()==1000000)
                return false;
            // assert(n->get_value()!=1000000 && "input not present\n");

            n->update_next_nodes(false, true);
            auto it = n->uses_begin();
            SbPDG_Node* use = (*it)->use();
            use->set_value(n->get_value(), true);
            n->set_value(1000000, true);
            // push_transient(use, n->get_value() ,true, 2); // set this value to output in next cycle(2 cycles for this)
            // push_transient(n, 1000000, true, 1); // popping the input: assuming througput=latency here
        }
        return true;
    }*/



    
      int num_computed = 0;
      // num_computed = compute_backcgra(vec_in, false, true); //should be true of expect to print

      if (num_computed ==0)
           return false;
      else 
       return true;
    }


    //Simulator would like to op size elements from vector port (vector_id)
    bool can_pop_output(SbPDG_VecOutput* vec_out, unsigned int len){
        
      assert(len>0 && "Cannot pop 0 length output\n");
      assert(vec_out->num_outputs()==len && "asked for different number of outputs than the supposed length\n");

      unsigned int v=0;
      for (unsigned int i=0; i<vec_out->num_outputs(); ++i){
        SbPDG_Node* temp = vec_out->getOutput(i); 
        if(temp->get_avail()) {
          // std::cout << "data: " << temp->get_value() << " and is it data: " << temp->get_avail() << "\n";
          v++;
        }
      }
      if(v==len){
        return true;
      }
      else
        return false;
    }

    //Simulator grabs size elements from vector port (vector_id)
    //assertion failure on insufficient size 
    void pop_vector_output(SbPDG_VecOutput* vec_out, std::vector<uint64_t>& data, unsigned int len, bool &discard){
      assert(vec_out->num_outputs()==len && "insufficient output available\n");
      
      // we don't need discard now!
      for (unsigned int i =0 ; i<vec_out->num_outputs(); i++){
        SbPDG_Node* temp = vec_out->getOutput(i); 
        SbPDG_Node* prev_inst = temp->first_operand(); 

        // CHECK THIS!
        // data.push_back(prev_inst->get_value());
        data.push_back(temp->get_value());
    
        discard = prev_inst->discard();
        assert(temp!=prev_inst && "previous node and this node should not be same in pop_vector\n");
 
        // temp->set_value(1000000, true); // pop means set it to invalid?: no discard(pop from output)
        temp->set_node(0, true, false); // pop means set it to invalid?: no discard(pop from output)
        // prev_inst->set_value(1000000, true);
        // prev_inst->set_node(0, true, false);

      }

      // Insufficient output size
      // assert(data.size()==len); 
    }

    void push_transient(SbPDG_Node* n, uint64_t v, bool valid, bool avail, int cycle){
        struct cycle_result* temp = new cycle_result(n, v, valid, avail);
        std::cout << "pushing into transient, name:  " << n->name() << " and val: " << v << " avail: " << avail << "\n";
        transient_values.push_back(std::make_pair(cycle, temp));
    }

    int cycle(bool print, bool verif){ 
        // std::list<std::pair<int, struct cycle_result*>>::iterator it;

        // std::list<std::pair<int, struct cycle_result*>>::iterator it1;
        // std::list<std::pair<int, struct cycle_result*>>::iterator it2;
        // sum up the duplicates first: very inefficent!
        for(auto it1=transient_values.begin(); it1!=transient_values.end(); ++it1){
            for(auto it2=std::next(it1,1); it2!=transient_values.end(); ++it2){
                if((*it1).second->n==(*it2).second->n){ // remove later one
                    // std::cout << "duplicates were present here\n";
                    (*it1).first = (*it1).first + (*it2).first - 1;
                    // What about discard here?
                    // std::cout << "Check the validity of duplicate nodes here: " << (*it1).second->valid << " " << (*it2).second->valid << "\n";
                    // std::cout << "Check the availability of duplicate nodes here: " << (*it1).second->avail << " " << (*it2).second->avail << "\n";
                    transient_values.erase(it2);
                    --it2;
                }
            }
        }

        for(auto it=transient_values.begin(); it!=transient_values.end(); ++it){
          // if((*it).first <= 1){
          if((*it).first < 1){
            SbPDG_Node* sb_node = (*it).second->n;
            // sb_node->set_value((*it).second->val, (*it).second->valid);
            if(!((*it).second->avail==0 && sb_node->inc_reset_req()!=sb_node->num_out())){
            // reset_req_times=0;
            sb_node->reset_reset_req();
            sb_node->set_node((*it).second->val, (*it).second->valid,(*it).second->avail);
            std::cout << "pushing in transient to " << sb_node->name() << " val:" << (*it).second->val << " avail: " << (*it).second->avail << "\n";
            sb_node->update_next_nodes(print, verif);
            transient_values.erase(it);
            it--;
            }
            else {
                std::cout << "came to reset the node for multiple output edges\n";
            }
          }
          else{
                (*it).first--;
          }
        }



        int temp = _total_dyn_insts;
        _total_dyn_insts=0;
        return temp; 
     }

    SbPDG_Node* get_scalar_output(SbPDG_VecOutput* vec_out, int id){
        return vec_out->getOutput(id);
    }

// ---------------------------------------------------------------------------

    std::set<SbPDG_Output*> getDummiesOutputs() {return dummiesOutputs;}

    void calc_minLats();

    void set_dbg_stream(std::ostream* dbg_stream) {_dbg_stream=dbg_stream;}
    std::ostream& dbg_stream() {return *_dbg_stream;}
    
    void check_for_errors();

    void inc_total_dyn_insts() {_total_dyn_insts++;}
    int  total_dyn_insts()     {return _total_dyn_insts;}


  private:
    // to keep track of number of cycles---------------------
    struct cycle_result{
       SbPDG_Node* n;
       uint64_t val;
       bool valid;
       bool avail;

       cycle_result(SbPDG_Node* node, uint64_t value, bool valid_in, bool a){
          n = node;
          val = value;
          valid = valid_in;
          avail = a;
       }
    };

    std::list<std::pair<int,struct cycle_result*>> transient_values;
    //--------------------------------------

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
