#ifndef DYPDG_H
#define DYPDG_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include "dyinst.h"
#include <unordered_map>
#include <map>
#include <assert.h>
#include <sstream>

#include "model.h"

class DyPDG_Node;

class DyPDG_Edge {
  public:
    enum EdgeType { data, ctrl_true, ctrl_false };
    
    EdgeType etype() {return _etype;}
    
    DyPDG_Edge(DyPDG_Node* def, DyPDG_Node* use, EdgeType etype) {
       _def=def;
       _use=use;
       _etype=etype;
       _ID=ID_SOURCE++;
    }

    DyPDG_Node* def() const {return _def;}
    DyPDG_Node* use() const {return _use;}
    
    std::string gamsName();
    
  private:
    int _ID;
    DyPDG_Node *_def, *_use;
    EdgeType _etype;
  private:
    static int ID_SOURCE;
};


class DyPDG_Node {
 public:
    virtual void printGraphviz(std::ostream& os);
    
    DyPDG_Node() {
        _ID=ID_SOURCE++;
    }
    
    typedef std::vector<DyPDG_Edge*>::const_iterator const_edge_iterator;
     
     void addIncEdge(unsigned pos, DyPDG_Edge *edge) { 
       if(_ops.size()<=pos) { 
          _ops.resize(pos+1,NULL); 
        }
        if(_ops[pos]) {
          std::cout << "overwrite" << _ops[pos]->def()->name() << "\n";
          assert(0);
        }
        _ops[pos]=edge;
     }
     
     void addOutEdge(unsigned pos, DyPDG_Edge *edge) {
        if(_uses.size()<=pos) { 
          _uses.resize(pos+1,NULL); 
        }
        if(_uses[pos]) {
          std::cout << "overwrite" << _uses[pos]->use()->name() << "\n";
          assert(0);
        }
        
        _uses[pos]=edge;
     }
     
     DyPDG_Edge* getLinkTowards(DyPDG_Node* to) {
        for(unsigned i = 0; i < _uses.size(); ++ i) {
          if(_uses[i] && _uses[i]->use()==to) {
            return _uses[i];
          }
        }
        return NULL;
     }
     
     int num_inc() const { return  _ops.size();  }
     int num_out() const { return  _uses.size(); }
     
     virtual std::string name() = 0;
     void setName(std::string& name) {_name = name;}
     virtual std::string gamsName() = 0;
     
     const_edge_iterator ops_begin() const {return _ops.begin();}
     const_edge_iterator ops_end() const {return _ops.end();}
     const_edge_iterator uses_begin() const {return _uses.begin();}
     const_edge_iterator uses_end() const {return _uses.end();}
     
     int id() {return _ID;}
     
  protected:    
    int _ID;
    std::string _name;
    std::vector<DyPDG_Edge *> _ops;
    std::vector<DyPDG_Edge *> _uses;
  private:
    static int ID_SOURCE;
};


class DyPDG_IO : public DyPDG_Node {
  public:
  void setVPort(int vport) { _vport = vport; } 
  int vport() {return _vport;}

  protected:
    int _vport;
};

class DyPDG_Input : public DyPDG_IO {
  public:
    void printGraphviz(std::ostream& os);
    
    std::string name() {
        std::stringstream ss;
        ss << "I" << _vport;
        return ss.str();
    }
    std::string gamsName();
    
};

class DyPDG_Output : public DyPDG_IO {
  public:
    void printGraphviz(std::ostream& os);
    
    std::string name() {
        std::stringstream ss;
        ss << "O" << _vport;
        return ss.str();
    }
    
    std::string gamsName();
};


class DyPDG_Vec {
  public:

  DyPDG_Vec(std::string name, int id) : _name(name), _ID(id) {
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

class DyPDG_VecInput : public DyPDG_Vec {
  public:

  DyPDG_VecInput(std::string name, int id) : DyPDG_Vec(name,id) {}

  virtual std::string gamsName() {
    std::stringstream ss;
    ss << "IPV_" << _name ;
    return ss.str();
  }

  void addInput(DyPDG_Input* in) { _inputs.push_back(in); }
  std::vector<DyPDG_Input*>::iterator input_begin() {return _inputs.begin();}
  std::vector<DyPDG_Input*>::iterator input_end() {return _inputs.end();}

  private:
    std::vector<DyPDG_Input*> _inputs;
};


class DyPDG_VecOutput : public DyPDG_Vec {
  public:

  DyPDG_VecOutput(std::string name, int id) : DyPDG_Vec(name,id) {}

  virtual std::string gamsName() {
    std::stringstream ss;
    ss << "OPV_" << _name ;
    return ss.str();
  }

  void addOutput(DyPDG_Output* out) { _outputs.push_back(out); }
  std::vector<DyPDG_Output*>::iterator output_begin() {return _outputs.begin();}
  std::vector<DyPDG_Output*>::iterator output_end() {return _outputs.end();}

  private:
    std::vector<DyPDG_Output*> _outputs;
};















class DyPDG_Inst : public DyPDG_Node {
  public:
    DyPDG_Inst() : DyPDG_Node(), _predInv(false), _imm_slot(-1), _subFunc(0) {
    }

    void printGraphviz(std::ostream& os);

    void setImm( uint32_t val ) { _imm=val; }
    void setImm( float val ) { val=*reinterpret_cast<int32_t*>(&val); }

    float getImmFloat() { return *reinterpret_cast<float*>(&_imm); } 
    int   getImmInt() { return _imm; }

    void setPredInv(bool predInv) { _predInv=predInv;}
    bool predInv() {return _predInv;}

     

    void setInst(SB_CONFIG::dy_inst_t dyinst) { _dyinst=dyinst; }
    SB_CONFIG::dy_inst_t inst() { return _dyinst; }

    std::string name() {
        std::stringstream ss;
        ss << SB_CONFIG::name_of_inst(_dyinst);
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
    
  private:
    bool _predInv;
    int _imm_slot;
    int _subFunc;
    uint32_t _imm;
    SB_CONFIG::dy_inst_t _dyinst;
};

class DyPDG {
  public:
    DyPDG();
    DyPDG(std::string filename);

    ~DyPDG() {
      }
    void printGraphviz(std::ostream& os);
    void printGraphviz(const char *fname) {
      std::ofstream os(fname);
      assert(os.good());
      printGraphviz(os);
    }
    void printGams(std::ostream& os, std::unordered_map<std::string,DyPDG_Node*>&,std::unordered_map<std::string,DyPDG_Edge*>&,std::unordered_map<std::string, DyPDG_Vec*>&);
    void printPortCompatibilityWith(std::ostream& os, SB_CONFIG::DyModel* dyModel);



    DyPDG_Inst *CreateInst() {
      return new DyPDG_Inst();
    }

    void addInst(DyPDG_Inst* inst) {_insts.push_back(inst); _nodes.push_back(inst);}

    //Just for adding single input without keeping track of name/sym-table
    void addInput(DyPDG_Input* input) {_inputs.push_back(input); _nodes.push_back(input);}
    void addOutput(DyPDG_Output* output) {_outputs.push_back(output); _nodes.push_back(output);}

    void addScalarInput(std::string name, std::map<std::string,DyPDG_Node*>& syms) {
      DyPDG_VecInput* vec_input = new DyPDG_VecInput(name,_vecInputs.size()); 
      _vecInputs.push_back(vec_input);
 
      DyPDG_Input* pdg_in = new DyPDG_Input();
      syms[name]=pdg_in;
      pdg_in->setName(name);
      pdg_in->setVPort(_vecInputs.size());
      addInput(pdg_in);
      vec_input->addInput(pdg_in);
    } 

    void addScalarOutput(std::string name, std::map<std::string,DyPDG_Node*>& syms) {
      DyPDG_Node* out_node = syms[name];
      if(out_node==NULL) {
        std::cerr << "Could not find" + name + "\n";
        assert("0");
      }

      DyPDG_VecOutput* vec_output = new DyPDG_VecOutput(name,_vecOutputs.size()); 
      _vecOutputs.push_back(vec_output);
 
      DyPDG_Output* pdg_out = new DyPDG_Output();
      std::string out_name=name+"_out";
      syms[out_name]=pdg_out;
      pdg_out->setName(out_name);
      pdg_out->setVPort(_vecOutputs.size());
      addOutput(pdg_out);
      vec_output->addOutput(pdg_out);

      connect(out_node, pdg_out,0,DyPDG_Edge::data);
    } 

    void addVecOutput(std::string name,
                     std::vector<std::vector<int> >& pm,
                     std::map<std::string,DyPDG_Node*>& syms ) {
      assert(0 && "addVecOutput not implemented");
    }

    void addVecInput(std::string name,
                     std::vector<std::vector<int> >& pm,
                     std::map<std::string,DyPDG_Node*>& syms ) {
      DyPDG_VecInput* vec_input = new DyPDG_VecInput(name,_vecInputs.size()); 
      vec_input->setLocMap(pm);
      _vecInputs.push_back(vec_input);

      int entries = pm.size();
      std::cout << "entries: " << entries << "\n";

      for(int i = 0; i < entries; ++i) {
        std::stringstream ss;
        ss << name << i;
        std::cout << "name: " << name << "\n";
        DyPDG_Input* pdg_in = new DyPDG_Input();
        std::string name = ss.str();
        syms[name]=pdg_in;
        pdg_in->setName(name);
        pdg_in->setVPort(_vecInputs.size());
        addInput(pdg_in);
        vec_input->addInput(pdg_in);
      }
    }

    void parse_and_add_vec(std::string name,std::string line,
                           std::map<std::string,DyPDG_Node*>& syms,bool input);

    DyPDG_Edge* connect(DyPDG_Node* orig, DyPDG_Node* dest,int slot,DyPDG_Edge::EdgeType etype);
    
    typedef std::vector<DyPDG_Node*>::const_iterator   const_node_iterator;
    typedef std::vector<DyPDG_Inst*>::const_iterator   const_inst_iterator;
    typedef std::vector<DyPDG_Input*>::const_iterator  const_input_iterator;
    typedef std::vector<DyPDG_Output*>::const_iterator const_output_iterator;
    typedef std::vector<DyPDG_Edge*>::const_iterator   const_edge_iterator;
    
    const_inst_iterator inst_begin() {return _insts.begin();}
    const_inst_iterator inst_end() {return _insts.end();}
    
    const_input_iterator input_begin() {return _inputs.begin();}
    const_input_iterator input_end() {return _inputs.end();}
    
    const_output_iterator output_begin() {return _outputs.begin();}
    const_output_iterator output_end() {return _outputs.end();}

    int num_nodes() {return _nodes.size();}

  private:
    std::vector<DyPDG_Node*> _nodes;
    
    //redundant storage:
    std::vector<DyPDG_Inst*> _insts;
    std::vector<DyPDG_Input*> _inputs;
    std::vector<DyPDG_Output*> _outputs;

    std::vector<DyPDG_VecInput*> _vecInputs;
    std::vector<DyPDG_VecOutput*> _vecOutputs;


    std::vector<DyPDG_Edge*> _edges;
    
};


#endif
