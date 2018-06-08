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
#include <queue>
#include <list>
#include <assert.h>
#include <sstream>
#include <algorithm>
#include "model.h"
#include <bitset>

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
    // void compute_next();
    void compute_after_push();
    void compute_after_pop();
    void print_buf_state(){
        /*std::cout << _data_buffer.front() << " ";
        if(_data_buffer.size()>1)
          std::cout << _data_buffer.back() << "\n";
          */
    }
    void push_in_buffer(uint64_t v, bool valid) {
      // std::cout << "data already in the buffer: " << _data_buffer.size() << " and max_size allowed is: " << buf_len << "\n";
      // std::cout << "val I was trying to push: " << v << "\n";
      assert(_data_buffer.size()<buf_len && "Trying to push in full buffer\n");
      // std::cout << "Before push state: \n";
      print_buf_state();

      _data_buffer.push(std::make_pair(v,valid));
      // std::cout << "pushed new value of: " << v << " here and top is: "<<_data_buffer.front()<<"\n";
      compute_after_push();      
    }
    bool is_buffer_full(){
        return (_data_buffer.size()==buf_len);
    }
    bool is_buffer_empty(){
      return _data_buffer.empty();
      // return (_data_buffer.size()!=0);
    }
    uint64_t get_buffer_val() {
      assert(_data_buffer.size() > 0);
      return _data_buffer.front().first;
    }
    bool get_buffer_valid() {
      assert(_data_buffer.size() > 0);
      return _data_buffer.front().second;
    }

    void pop_buffer_val() {
      assert(!_data_buffer.empty() && "Trying to pop from empty queue\n");
      // std::cout << "came here to pop data: " << _data_buffer.front() << " from input buffer with buffer size now: " << _data_buffer.size() << "\n";
      // std::cout << "Before pop state: \n";
      print_buf_state();
      _data_buffer.pop();
      compute_after_pop();      
     
    }

  private:
    int _ID;
    SbPDG* _sbpdg;  //sometimes this is just nice to have : )
    SbPDG_Node *_def, *_use;
    EdgeType _etype;
    std::queue<std::pair<uint64_t,bool>> _data_buffer;
    // unsigned int buf_len = 1;
    // using 2 since 1st entry is used for bp
    // unsigned int buf_len = 3;
    unsigned int buf_len = 1024;
    //-------------

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

    // some issue with this function
    virtual uint64_t invalid() { return _invalid;} //execution-related
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

    //-----------------------------------------
    virtual int update_next_nodes(bool print, bool verif) {
        return 0;
    }
    /*
    virtual int update_next_nodes(bool print, bool verif) {
        // called from push_vector: i/p node
        int num_computed = 0;
        SbPDG_Node* n = this->first_use();
        num_computed = n->inc_inputs_ready_backcgra(print, verif);

        return num_computed;
    }
        auto it = this->uses_begin();
        if(!(*it)->is_buffer_full()){
          num_computed = n->inc_inputs_ready_backcgra(print, verif);
        }
   */ 
      
    virtual int compute_backcgra(bool print, bool verif) {
     //for an output node
     //assert(this->num_inc()==1 && "not an output node?\n");
     //assert(this->num_out()==0 && "not an output node?\n");
     //auto it = this->ops_begin();
     //SbPDG_Edge* e = *it;
     //this->set_outputnode(e->get_buffer_val(),e->get_buffer_valid(), true);
     //e->pop_buffer_val();
     _inputs_ready=0; // hopefully if it comes from inc_inputs_ready
     return 0;
    } 


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

    SbPDG_Edge* first_inc_edge() {
      return _ops[0];
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
      _invalid=!valid;
    }

    bool get_bp(){
        bool bp=false;
        for(auto it=this->uses_begin(); it!=this->uses_end(); ++it){
            if((*it)->is_buffer_full()){
                bp=true;
            }
        }
        return bp;
    }

    void set_outputnode(uint64_t v, bool valid, bool avail) {
      _val=v; 
      _invalid=!valid;
      _avail = avail;
    }

    void set_node(uint64_t v, bool valid, bool avail) {
      _val=v; 
      _invalid=!valid;
      _avail = avail;
    
      // std::cout << "came here to set the node: " << name() <<" to value: "<<v<<" and avail: "<<avail<<"\n";
      // no need to do anything for output node
      if(this->num_out()==0) { return; }
      if(avail){
        if(!get_bp()){
          for(auto iter=_uses.begin(); iter != _uses.end(); ++iter) {
            (*iter)->push_in_buffer(v, valid);
          }
          _avail=false;
        }
        else{
            this->set_value(v, valid, avail, 1); // after 1 cycle
        }
      }
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
      _inputs_ready+=1;
      // std::cout<<"Came to inc the inputs avail are: "<<_inputs_ready << " and required: "<<_num_inc_edges<<"\n";
      if(_inputs_ready == _num_inc_edges && _avail==0) {
        int num_computed = compute_backcgra(print,verif);//it's 0 or 1
        return num_computed;
      }
      return 0;
    } 

    int get_inputs_ready() {
        return _inputs_ready;
    }
   //---------------------------------------------------------------------------
   
    protected:    
    SbPDG* _sbpdg;  //sometimes this is just nice to have : )

    uint64_t _val = 0; //dynamic var (setting the default value)
    bool _avail = false; // if their is data in the output buffer
    // bool _backPressure=false;
    uint64_t _invalid=false;
    //bool _is_new_val = 0; // new variable
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


//Control map defines the set of control signals which will be passed into
//the configuration if any of these bits are set.
class CtrlMap {
public:
  enum ctrl_flag {BACKP1, BACKP2, DISCARD, RESET, ABSTAIN, NUM_CTRL};

  CtrlMap() {
    add_ctrl("b1", BACKP1);
    add_ctrl("b2", BACKP2);
    add_ctrl("d" , DISCARD);
    add_ctrl("r" , RESET);
    add_ctrl("a" , ABSTAIN);
  }

  std::string decode_control(ctrl_flag c) {
    if(_decode_map.count(c)) {
      return _decode_map[c];
    } else {
      assert(0);
    }
  }

  ctrl_flag encode_control(std::string s) {
    if(_encode_map.count(s)) {
      return _encode_map[s];
    } else {
      std::cout << "Bad Control Symobol: " << s <<"\n";
      assert("Bad Control Symbol" && 0); 
    }
  }

private:
  std::map<std::string,ctrl_flag> _encode_map;
  std::map<ctrl_flag,std::string> _decode_map; //this could be an array i suppose

  void add_ctrl(const char* s, ctrl_flag c) {
    _encode_map[std::string(s)]=c;
    _decode_map[c]=std::string(s);
  }
};
 
typedef std::vector<std::string> string_vec_t;

//post-parsing control signal definitions 
typedef std::map<int,string_vec_t> ctrl_def_t; 


class CtrlBits {
public:
  static int bit_loc(int c_val, CtrlMap::ctrl_flag flag) {
    return c_val * CtrlMap::NUM_CTRL + flag; 
  }
  CtrlBits(ctrl_def_t d) {
    for(auto i : d) {
      int key = i.first;
      auto vec = i.second;
      for(auto s : vec) {
        CtrlMap::ctrl_flag  local_bit_pos = ctrl_map.encode_control(s);
        int final_pos = bit_loc(key,local_bit_pos);
        _bits.set(final_pos);
      }
    }
  }
  CtrlBits(uint64_t b) {
    _bits = b;
  }
  CtrlBits(){}

  void print_rep() {
    for(int i = 0; i < 4; ++i) {
      printf("ctrl input %d:", i);
      for(int c = 0; c<CtrlMap::NUM_CTRL;++c) {
        if(isSet(i,(CtrlMap::ctrl_flag)c)) {
          printf(" %s",ctrl_map.decode_control((CtrlMap::ctrl_flag)c).c_str()); 
        } 
      }
      printf("\n");
    }
  }

  uint64_t bits() {return _bits.to_ullong();}

  bool isSet(int c_val, CtrlMap::ctrl_flag flag) {
    return _bits.test(bit_loc(c_val,flag));
  }
  
private:
  static CtrlMap ctrl_map;
  std::bitset<32> _bits;
};

struct SymEntry {
  enum enum_type {SYM_INV,SYM_INT, SYM_DUB, SYM_NODE} type;
  enum enum_flag {FLAG_NONE, FLAG_INV, FLAG_PRED, FLAG_INV_PRED, 
                  FLAG_BGATE, FLAG_CONTROL} 
    flag = FLAG_NONE;
  CtrlBits ctrl_bits;
  int width;
  union union_data {
    uint64_t i;
    double d;	 
    struct struct_data {float f1, f2;} f;
    SbPDG_Node* node;
  } data;
  SymEntry() {
    type=SYM_INV; 
    flag=FLAG_INV;
  }
  SymEntry(uint64_t i) {
    type=SYM_INT;
    data.i=i;
    width=1;
  }
  SymEntry(uint64_t i1, uint64_t i2) {
    type=SYM_INT;
    data.i= ((i2&0xFFFFFFFF) << 32) | ((i1&0xFFFFFFFF) <<0);
    width=2;
  }
  SymEntry(uint64_t i1, uint64_t i2, uint64_t i3, uint64_t i4) {
    type=SYM_INT;
    data.i= ((i4&0xFFFF) << 48) | ((i3&0xFFFF) <<32) | 
            ((i2&0xFFFF) << 16) | ((i1&0xFFFF) << 0);
    width=4;
  }
  SymEntry(double d) {
    type=SYM_DUB;
    data.d=d;
    width=1;
  }
  SymEntry(double d1, double d2) {
    float f1=d1, f2=d2;
    type=SYM_DUB;
    data.f.f1=f1;
    data.f.f2=f2;
    width=2;
  }
  SymEntry(SbPDG_Node* node) {
    type=SYM_NODE;
    data.node=node;
    width=1;
  }
  void set_flag(std::string& s) {
    if(s==std::string("pred")) {
      flag=FLAG_PRED;
    } else if (s==std::string("inv_pred")) {
      flag=FLAG_INV_PRED;
    } else if (s==std::string("control")) {
      flag=FLAG_CONTROL;
    } else {
      printf("qualifier: %s unknown",s.c_str());
      assert(0 && "Invalid argument qualifier");
    }
  }

  void set_control_list(ctrl_def_t& d) {
    ctrl_bits = CtrlBits(d);
  }
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
    int  getImmInt() { return _imm; }

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

    virtual uint64_t invalid() {
        return _invalid;
    }

   void set_ctrl_bits(CtrlBits c) {
     _ctrl_bits=c;
     //if(_ctrl_bits.bits() != 0) {
     //  _ctrl_bits.print_rep();
     //} 
   }
   uint64_t ctrl_bits() {return _ctrl_bits.bits();}

  private:
    std::ofstream _verif_stream;
    std::string _verif_id;
    std::vector<uint64_t> _input_vals;
    bool _predInv;
    bool _isDummy;
    int _imm_slot;
    int _subFunc;
    CtrlBits _ctrl_bits;

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

    // after inc inputs ready?
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

    uint64_t parent_invalid() {
      return _ops[0]->def()->invalid();
    }

    virtual uint64_t invalid() {
      return _invalid;
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

  bool backPressureOn();

  void addInput(SbPDG_Input* in) { _inputs.push_back(in); }
  std::vector<SbPDG_Input*>::iterator input_begin() {return _inputs.begin();}
  std::vector<SbPDG_Input*>::iterator input_end() {return _inputs.end();}
  unsigned num_inputs() const {return _inputs.size();}

  // scalar input corresponding to index i
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

class SymTab {
  std::map<std::string, SymEntry> _sym_tab;
  void assert_exists(std::string& s) {
    if(_sym_tab.count(s)==0) {
       std::cerr << "Could not find" + s + "\n";
       assert("0");
    }
  }
public:
  void set(std::string& s, SymEntry n) {_sym_tab[s]=n;}
  void set(std::string& s, SbPDG_Node* n) {_sym_tab[s]=SymEntry(n);}
  void set(std::string& s, uint64_t n)    {_sym_tab[s]=SymEntry(n);}
  void set(std::string& s, double n)      {_sym_tab[s]=SymEntry(n);}
  SymEntry get_sym(std::string& s) { 
    assert_exists(s);
    return _sym_tab[s];
  }
  SbPDG_Node* get_node(std::string& s) { 
    assert_exists(s);
    if(_sym_tab[s].type!=SymEntry::SYM_NODE) {
      std::cerr << "symbol \"" + s +"\" is not a node\"";
      assert(0);
    }
    return _sym_tab[s].data.node;
  }
  int get_int(std::string& s) { 
    assert_exists(s);
    if(_sym_tab[s].type!=SymEntry::SYM_INT) {
      std::cerr << "symbol \"" + s +"\" is not an int\"";
      assert(0);
    }
    return _sym_tab[s].data.i;
  }
  int get_float(std::string& s) { 
    assert_exists(s);
    if(_sym_tab[s].type!=SymEntry::SYM_DUB) {
      std::cerr << "symbol \"" + s +"\" is not a double\"";
      assert(0);
    }
    return _sym_tab[s].data.d;
  }
};

struct GroupProp {
  bool is_temporal=false;
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

    static void order_insts(SbPDG_Inst* inst,
                 std::set<SbPDG_Inst*>& done_nodes,         //done insts
                 std::vector<SbPDG_Inst*>& ordered_insts);

    std::vector<SbPDG_Inst*>& ordered_insts() {
      if(_orderedInsts.size()==0) {
        std::set<SbPDG_Inst*> done_nodes;
        for(SbPDG_Output* out : _outputs) {
          if(SbPDG_Inst* producing_node = out->out_inst()) {
            order_insts(producing_node, done_nodes, _orderedInsts);
          }
        } 
      }
      return _orderedInsts;
    }

    void printGraphviz(std::ostream& os, Schedule* sched=NULL);
    void printEmuDFG(std::ostream& os, std::string dfg_name);
    void printGraphviz(const char *fname, Schedule* sched=NULL) {
      std::ofstream os(fname);
      assert(os.good());
      printGraphviz(os, sched);
    }
    
    void start_new_dfg_group();
    void set_pragma(std::string& c, std::string& s);

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

    SbPDG_Edge* connect(SbPDG_Node* orig, SbPDG_Node* dest,int slot,SbPDG_Edge::EdgeType etype);
    void disconnect(SbPDG_Node* orig, SbPDG_Node* dest);

    SymEntry createInst(std::string opcode, std::vector<SymEntry>& args);

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

//------------------------------
/*
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
*/
    //------------------


    SbPDG_VecInput*  vec_in(int i) {return _vecInputs[i];}
    SbPDG_VecOutput* vec_out(int i) {return _vecOutputs[i];}

    std::vector<SbPDG_VecInput*>&  vec_in_group(int i) {return _vecInputGroups[i];}
    std::vector<SbPDG_VecOutput*>& vec_out_group(int i) {return _vecOutputGroups[i];}
    GroupProp&  prop_group(int i) {return _propGroups[i];}

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
    bool push_vector(SbPDG_VecInput* vec_in, std::vector<uint64_t> data, std::vector<bool> valid) {
      assert(data.size() == vec_in->num_inputs() && "insufficient data available");
      // int num_computed = 0;
      // int t=0;
      for (unsigned int i =0 ; i<vec_in->num_inputs(); ++i){
        SbPDG_Input* sb_node = vec_in->getInput(i); 
        sb_node->set_node(data[i],valid[i], true);
        /*
        if(!sb_node->get_avail()){
          t = sb_node->update_next_nodes(false, false);
        }
        */
        
        //std::cout << (valid[i] ? "input valid " : " input invalid ") << "\n";
      }
    //  std::cout <<"\n";

      return true;
      /*
      if(t>0)
          return true;
      else
          return false;
      */

   }

    // check if some value present at input node or there is some backpressure or invalid value
    bool can_push_input(SbPDG_VecInput* vec_in){
      for(unsigned int i=0; i<vec_in->num_inputs(); ++i) {
        SbPDG_Input* temp = vec_in->getInput(i);
        // std::cout << "Is there data already? " << temp->get_avail() << "\n";
        if(temp->get_avail()){
          // std::cout << "DATA AVAIL AT PORT: " << temp->name() << " is: " << temp->get_value() << "\n";
          return false;
        }
      }
      return true;
    }

    //Advances simulation by one cycle  (return whether there was activity)
    int cycle_compute(bool print, bool verif);
    int compute_backcgra(SbPDG_VecInput* vec_in, bool print, bool verif);


    // bool none_config = false;
    bool backcgra_cycle(SbPDG_VecInput* vec_in) {
    
      int num_computed = 0;

      if (num_computed ==0)
           return false;
      else 
       return true;
    }


    //Simulator would like to op size elements from vector port (vector_id)
    bool can_pop_output(SbPDG_VecOutput* vec_out, unsigned int len){
        
      assert(len>0 && "Cannot pop 0 length output\n");
      assert(vec_out->num_outputs()==len && "asked for different number of outputs than the supposed length\n");

      unsigned int ready_outputs=0;
      for (unsigned int i=0; i<vec_out->num_outputs(); ++i){
        SbPDG_Edge* inc_edge = *vec_out->getOutput(i)->ops_begin(); 
        if(!inc_edge->is_buffer_empty()) {
          ready_outputs++;
        }
      }
      if(ready_outputs==len){
        return true;
      } else {
        return false;
      }
    }

    //Simulator grabs size elements from vector port (vector_id)
    //assertion failure on insufficient size 
    void pop_vector_output(SbPDG_VecOutput* vec_out, std::vector<uint64_t>& data, 
                           std::vector<bool>& data_valid, unsigned int len){
      assert(vec_out->num_outputs()==len && "insufficient output available\n");
      
      // we don't need discard now!
      for (unsigned int i =0 ; i<vec_out->num_outputs(); i++){
        SbPDG_Edge* e = *vec_out->getOutput(i)->ops_begin(); 
        data.push_back(e->get_buffer_val());
        data_valid.push_back(e->get_buffer_valid());
        //std::cout << e->get_buffer_valid();
        e->pop_buffer_val();
      }

      // Insufficient output size
      // assert(data.size()==len); 
    }

    void push_transient(SbPDG_Node* n, uint64_t v, bool valid, bool avail, int cycle){
      struct cycle_result* temp = new cycle_result(n, v, valid, avail);
      transient_values[(cycle+cur_node_ptr)%get_max_lat()].push_back(temp);
    }
    
    void push_buf_transient(SbPDG_Edge* e, int cycle){
      buf_transient_values[(cycle+cur_buf_ptr)%get_max_lat()].push_back(e);
      // std::cout << "POP FROM BUFFER AT: " << (cycle+cur_buf_ptr)%get_max_lat() << "\n";
    }

    int cycle(bool print, bool verif){ 

      // int num_computed=0;
       
      for(auto it=buf_transient_values[cur_buf_ptr].begin();it!= buf_transient_values[cur_buf_ptr].end();++it){
        // std::cout << "CUR_BUF_PTR IS: " << cur_buf_ptr << "\n";
        // set the values
        SbPDG_Edge* e = *it;
        e->pop_buffer_val();
        /*if(!e->is_buffer_empty()){
            SbPDG_Node* n = e->use();
            n->set_is_new_val(1);
            n->inc_inputs_ready_backcgra(false, false);
        }
        */
        buf_transient_values[cur_buf_ptr].erase(it);
        it--;
      }


      for(auto it=transient_values[cur_node_ptr].begin(); it!=transient_values[cur_node_ptr].end();++it){
        struct cycle_result* temp = *it;
        SbPDG_Node* sb_node = temp->n;
        //std::cout << "transient_value update valid: " << temp->valid << "\n";

        sb_node->set_node(temp->val, temp->valid, temp->avail);
        /*
        if(!sb_node->get_avail()){
          num_computed += sb_node->update_next_nodes(print, verif);
        }
        */ 
        
        transient_values[cur_buf_ptr].erase(it);
        it--;
       }
       
       cur_buf_ptr = (cur_buf_ptr+1)%get_max_lat();
       cur_node_ptr = (cur_node_ptr+1)%get_max_lat();
       int temp = _total_dyn_insts;
       _total_dyn_insts=0;
       return temp; 
    }

// ---------------------------------------------------------------------------

    std::set<SbPDG_Output*> getDummiesOutputs() {return dummiesOutputs;}

    void calc_minLats();

    void set_dbg_stream(std::ostream* dbg_stream) {_dbg_stream=dbg_stream;}
    std::ostream& dbg_stream() {return *_dbg_stream;}
    
    void check_for_errors();

    void inc_total_dyn_insts() {_total_dyn_insts++;}
    int  total_dyn_insts()     {return _total_dyn_insts;}
    int get_max_lat() { return MAX_LAT; }


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

    int MAX_LAT = 1000;
    std::list<struct cycle_result*> transient_values[1000];
    int cur_node_ptr=0;
    int cur_buf_ptr=0;
    std::list<SbPDG_Edge*> buf_transient_values[1000];
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
    std::vector<GroupProp> _propGroups;


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
