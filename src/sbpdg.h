#ifndef __SBPDG_H__
#define __SBPDG_H__

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "sbinst.h"
#include <unordered_map>
#include <map>
#include <assert.h>
#include <sstream>

#include "model.h"

class SbPDG_Node;

class SbPDG_Edge {
  public:
    enum EdgeType { data, ctrl_true, ctrl_false };
    
    EdgeType etype() {return _etype;}
    
    SbPDG_Edge(SbPDG_Node* def, SbPDG_Node* use, EdgeType etype) {
       _def=def;
       _use=use;
       _etype=etype;
       _ID=ID_SOURCE++;
    }

    SbPDG_Node* def() const {return _def;}
    SbPDG_Node* use() const {return _use;}
    
    std::string gamsName();
    
  private:
    int _ID;
    SbPDG_Node *_def, *_use;
    EdgeType _etype;
  private:
    static int ID_SOURCE;
};

//PDG Node -- abstract base class
class SbPDG_Node {
 public:
    virtual void printGraphviz(std::ostream& os);
    
    SbPDG_Node() {
        _ID=ID_SOURCE++;
    }
    
    typedef std::vector<SbPDG_Edge*>::const_iterator const_edge_iterator;
     
    void addIncEdge(unsigned pos, SbPDG_Edge *edge) { 
      if(_ops.size()<=pos) { 
         _ops.resize(pos+1,NULL); 
       }

       if(_ops[pos]) {
         std::cout << "overwrite" << _ops[pos]->def()->name() << "\n";
         assert(0);
       }
       _ops[pos]=edge;
    }
    
    void addOutEdge(unsigned pos, SbPDG_Edge *edge) {
       if(_uses.size()<=pos) { 
         _uses.resize(pos+1,NULL); 
       }

       if(_uses[pos]) {
         std::cout << "overwrite" << _uses[pos]->use()->name() << "\n";
         assert(0);
       }
       
       _uses[pos]=edge;
    }
    
    SbPDG_Edge* getLinkTowards(SbPDG_Node* to) {
       for(unsigned i = 0; i < _uses.size(); ++ i) {
         if(_uses[i] && _uses[i]->use()==to) {
           return _uses[i];
         }
       }
       return NULL;
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
    
    void     set_value(uint64_t v) {_val=v;}
    uint64_t get_value()           {return _val;}

  protected:    
    uint64_t _val;
    int _ID;
    std::string _name;
    std::vector<SbPDG_Edge *> _ops;     //in edges 
    std::vector<SbPDG_Edge *> _uses;   //out edges  

  private:
    static int ID_SOURCE;
};


class SbPDG_IO : public SbPDG_Node {
  public:
  void setVPort(int vport) { _vport = vport; } 
  int vport() {return _vport;}

  protected:
    int _vport;
};

//Instruction
class SbPDG_Inst : public SbPDG_Node {
  public:
    SbPDG_Inst() : SbPDG_Node(), _predInv(false), _imm_slot(-1), _subFunc(0) {
    }

    void printGraphviz(std::ostream& os);

    void setImm( uint32_t val ) { _imm=val; }
//    void setImm( float val ) { _imm=*reinterpret_cast<int32_t*>(&val); }

//    float getImmFloat() { return *reinterpret_cast<float*>(&_imm); } 
    int   getImmInt() { return _imm; }

    void setPredInv(bool predInv) { _predInv=predInv;}
    bool predInv() {return _predInv;}

    void setInst(SB_CONFIG::sb_inst_t sbinst) { _sbinst=sbinst; }
    SB_CONFIG::sb_inst_t inst() { return _sbinst; }

    std::string name() {
        std::stringstream ss;
        ss << SB_CONFIG::name_of_inst(_sbinst);
        if(_imm_slot!=-1) {
          ss<<" Imm:"<<_imm;
        }
        return ss.str();
    }

    std::string gamsName();

    void setImmSlot(int i) {_imm_slot=i;}
    int immSlot() const { return _imm_slot; }

    void setSubFunc(int i) {_subFunc=i;}
    int subFunc() const {return _subFunc;}

     void compute(bool print) {
       if(_input_vals.size()==0) {
         _input_vals.resize(_ops.size());
       }
       if(print) {
         std::cout << name() << " (" << _ID << "): ";
       }
        
       for(unsigned i = 0; i < _ops.size(); ++i) {
         _input_vals[i]=_ops[i]->def()->get_value();
         if(print) {
           std::cout << std::hex << _input_vals[i] << " ";
         }
       }
       
       _val=SB_CONFIG::execute(_sbinst,_input_vals);
       
       if(print) {
         std::cout << " = " << _val << "\n";
       }
     }

  private:
    std::vector<uint64_t> _input_vals;
    bool _predInv;
    int _imm_slot;
    int _subFunc;
    uint32_t _imm;
    SB_CONFIG::sb_inst_t _sbinst;
};

class SbPDG_Input : public SbPDG_IO {       //inturn inherits sbnode
  public:
    void printGraphviz(std::ostream& os);
    
    std::string name() {
        std::stringstream ss;
        ss << "I" << _vport;
        return ss.str();
    }
    std::string gamsName();
    
};

class SbPDG_Output : public SbPDG_IO {
  public:
    void printGraphviz(std::ostream& os);
    
    std::string name() {
        std::stringstream ss;
        ss << "O" << _vport;
        return ss.str();
    }
    std::string gamsName();

    //returns the instruction producing the
    //value to this output node
    SbPDG_Inst* out_inst() {
      return static_cast<SbPDG_Inst*>(_ops[0]->def());
    }

    //retrieve the value of the def
    uint64_t retrieve() {
      assert(_ops.size()==1);
      return _ops[0]->def()->get_value();
    }
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

  void addInput(SbPDG_Input* in) { _inputs.push_back(in); }
  std::vector<SbPDG_Input*>::iterator input_begin() {return _inputs.begin();}
  std::vector<SbPDG_Input*>::iterator input_end() {return _inputs.end();}

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

  private:
    std::vector<SbPDG_Output*> _outputs;
};


class SbPDG {
  public:
    SbPDG();
    SbPDG(std::string filename);

    ~SbPDG(){
    }

    void printGraphviz(std::ostream& os);
    void printGraphviz(const char *fname) {
      std::ofstream os(fname);
      assert(os.good());
      printGraphviz(os);
    }
    
    void printGams(std::ostream& os, std::unordered_map<std::string,SbPDG_Node*>&,
                                     std::unordered_map<std::string,SbPDG_Edge*>&,
                                     std::unordered_map<std::string, SbPDG_Vec*>&);

    void printPortCompatibilityWith(std::ostream& os, SB_CONFIG::SbModel* sbModel);



    SbPDG_Inst* CreateInst() {
      return new SbPDG_Inst();
    }

    void addInst(SbPDG_Inst* inst) {
        _insts.push_back(inst); 
        _nodes.push_back(inst);}

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
      _vecInputs.push_back(vec_input);
 
      SbPDG_Input* pdg_in = new SbPDG_Input();  //new input nodes
      syms[name]=pdg_in;
      pdg_in->setName(name);
      pdg_in->setVPort(_vecInputs.size());
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
      _vecOutputs.push_back(vec_output);
 
      SbPDG_Output* pdg_out = new SbPDG_Output();
      std::string out_name=name+"_out";
      syms[out_name]=pdg_out;
      pdg_out->setName(out_name);
      pdg_out->setVPort(_vecOutputs.size());
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
      _vecOutputs.push_back(vec_output); 
       
      int entries = pm.size();
      std::cout << "entries: " << entries << "\n";

      for(int i = 0; i < entries; ++i) {
        std::stringstream ss;
        ss << name << i;
        std::cout << "name: " << name << "\n";
        SbPDG_Output* pdg_out = new SbPDG_Output();
        std::string name = ss.str();
        syms[name]=pdg_out;
        pdg_out->setName(name);
        pdg_out->setVPort(_vecOutputs.size());
        addOutput(pdg_out);
        vec_output->addOutput(pdg_out);
      } 
      //assert(0 && "addVecOutput not implemented");
    }

    void addVecInput(std::string name,
                     std::vector<std::vector<int> >& pm,
                     std::map<std::string,SbPDG_Node*>& syms ) {

      SbPDG_VecInput* vec_input = new SbPDG_VecInput(name,_vecInputs.size()); 
      vec_input->setLocMap(pm);
      _vecInputs.push_back(vec_input);

      //number of vector entries -- each vector element is a input node
      int entries = pm.size();
      std::cout << "entries: " << entries << "\n";

      for(int i = 0; i < entries; ++i) {
        std::stringstream ss;
        ss << name << i;                //Vector input names: A0, A1
        std::cout << "name: " << name << "\n";
        SbPDG_Input* pdg_in = new SbPDG_Input();
        std::string name = ss.str();
        syms[name]=pdg_in;
        pdg_in->setName(name);
        pdg_in->setVPort(_vecInputs.size());
        addInput(pdg_in);
        vec_input->addInput(pdg_in);
      }
    }

    void parse_and_add_vec(std::string name,
                           std::string line,
                           std::map<std::string,SbPDG_Node*>& syms,bool input);

    SbPDG_Edge* connect(SbPDG_Node* orig, SbPDG_Node* dest,int slot,SbPDG_Edge::EdgeType etype);
    
    typedef std::vector<SbPDG_Node*>::const_iterator   const_node_iterator;
    typedef std::vector<SbPDG_Inst*>::const_iterator   const_inst_iterator;
    typedef std::vector<SbPDG_Input*>::const_iterator  const_input_iterator;
    typedef std::vector<SbPDG_Output*>::const_iterator const_output_iterator;
    typedef std::vector<SbPDG_Edge*>::const_iterator   const_edge_iterator;
    
    const_inst_iterator inst_begin() {return _insts.begin();}
    const_inst_iterator inst_end() {return _insts.end();}
    
    const_input_iterator input_begin() {return _inputs.begin();}
    const_input_iterator input_end() {return _inputs.end();}
    
    const_output_iterator output_begin() {return _outputs.begin();}
    const_output_iterator output_end() {return _outputs.end();}

    int num_nodes() {return _nodes.size();}

    SbPDG_VecInput*  vec_in(int i) {return _vecInputs[i];}
    SbPDG_VecOutput* vec_out(int i) {return _vecOutputs[i];}

    void compute(bool print);

  private:
    std::vector<SbPDG_Node*> _nodes;
    
    //redundant storage:
    std::vector<SbPDG_Inst*> _insts;
    std::vector<SbPDG_Input*> _inputs;
    std::vector<SbPDG_Output*> _outputs;

    std::vector<SbPDG_Inst*> _orderedInsts;


    std::vector<SbPDG_VecInput*> _vecInputs;
    std::vector<SbPDG_VecOutput*> _vecOutputs;

    std::vector<SbPDG_Edge*> _edges;
    
};

#endif
