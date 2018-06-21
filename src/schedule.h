#ifndef __SB__SCHEDULE_H__
#define __SB__SCHEDULE_H__

#include <map>
#include <unordered_set>
#include "model.h"
#include "sub_model.h"
#include "sbpdg.h"
#include <iostream>
#include <fstream>
#include "bitslice.h"
#include "config_defs.h"

using namespace SB_CONFIG;

//How do you choose which switch in each row and FU to pass ??
//struct sw_config {
//  unsigned pred:2;
//  unsigned opcode:5;
//  unsigned fu_in1:2;    //cfg for 1st input of FU
//  unsigned fu_in2:2;    //cfg for 2nd input of FU
//  unsigned fu_out:2;    //cfg for 3rd input of FU
//  unsigned  sw_s:3;     //select lien for output muxes
//  unsigned sw_se:3;
//  unsigned  sw_e:3;
//  unsigned sw_ne:3;
//  unsigned  sw_n:3;
//  unsigned sw_nw:3;
//  unsigned  sw_w:3;
//  unsigned sw_sw:3;
//  unsigned   row:2;  //can address only 4 rows -- need to update it
//};

#define MAX_SCHED_LAT 1000000

class Schedule {
  public:
    Schedule(std::string filename); //Read in schedule (both sbmodel, sbpdg, and schedule from file)

    Schedule(SbModel* model, SbPDG* pdg ) : _sbModel(model), _sbPDG(pdg) {
      allocate_space();  
    }
    Schedule(SbModel* model) : _sbModel(model), _sbPDG(NULL) {
      allocate_space(); 
    }

    //Scheduling Interface:
    bool spilled(SbPDG_Node*);
    
    //Old Interface:
    
    int getPortFor(SbPDG_Node*);
    
    void printConfigText(std::ostream& os);
    void printConfigText(const char *fname) {
      std::ofstream os((std::string(fname)+".cfg").c_str());
      assert(os.good()); 
      printConfigText(os);
    }
   
    void printConfigBits(std::ostream& os, std::string cfg_name );
    void printConfigVerif(std::ostream& os);
    
    //Rest of Stuff
    SbPDG* sbpdg() const {return _sbPDG;}

    
    std::map<SB_CONFIG::sblink*,SB_CONFIG::sblink*>& link_map_for_sw(sbswitch* sbsw) {
      return _assignSwitch[sbsw];
    }

    //check if inlink
    sblink* get_switch_in_link(sbswitch* sbsw, sblink* out_link) {
      auto iter = _assignSwitch[sbsw].find(out_link);
      if(iter == _assignSwitch[sbsw].end()) {
        return NULL;
      } else {
        return _assignSwitch[sbsw][out_link];
      }
    }

    bool have_switch_links() {
      return _assignSwitch.size() !=0;
    }

    //For a switch, assign the outlink to inlink
    void assign_switch(sbswitch* sbsw, 
                       sblink* slink,
                       sblink* slink_out) {
      assert(sbsw);
      assert(slink);
      assert(slink_out);
      _assignSwitch[sbsw][slink_out]=slink;     //out to in for a sw      
    }

    void assign_lat(SbPDG_Vec* vec, int lat) {
      _vecProp[vec].lat=lat;
    }

    void assign_lat(SbPDG_Node* pdgnode, int lat) {
      _vertexProp[pdgnode->id()].lat=lat;
    }
    int latOf(SbPDG_Node* pdgnode) {
      return _vertexProp[pdgnode->id()].lat;
    }

    void assign_lat_bounds(SbPDG_Vec* vec, int min, int max) {
      auto& vec_prop = _vecProp[vec];
      vec_prop.min_lat=min;
      vec_prop.max_lat=max;
    }

    void assign_lat_bounds(SbPDG_Node* pdgnode, int min, int max) {
      auto& vertex_prop = _vertexProp[pdgnode->id()];
      vertex_prop.min_lat=min;
      vertex_prop.max_lat=max;
    }

    std::pair<int,int> lat_bounds(SbPDG_Node* pdgnode) {
      auto& vertex_prop = _vertexProp[pdgnode->id()];
      return std::make_pair(vertex_prop.min_lat, vertex_prop.max_lat);
    }

    void assign_edge_pt(SbPDG_Edge* edge, sbnode* pt) {
      add_passthrough_node(pt);
      _edgeProp[edge->id()].passthroughs.insert(pt);
    }

    //Computes maximum throughput both given the latency mismatch &&
    //functional unit throughput
    //int maxMismatch(int g) {
      //int maxgt=_sbPDG->maxGroupThroughput(g);
      
    //}

    int violation() {return _totalViolation;}
    void add_violation(int violation) {
      _totalViolation += violation;
      _max_lat_mis = std::max(_max_lat_mis,violation);
    }

    int vioOf(SbPDG_Node* n) { return _vertexProp[n->id()].vio;}
    void record_violation(SbPDG_Node* n, int violation) {
      _vertexProp[n->id()].vio=violation; 
    }


    //Assign the sbnode to pdgnode and vice verse 
    void assign_node(SbPDG_Node* pdgnode, sbnode* snode) {
      _num_mapped[pdgnode->type()]++;
      //std::cout << "node " << pdgnode->name() << " assigned to " 
      //          << snode->name() << "\n";
      assert(pdgnode); 
      _nodeProp[snode->id()].vertices.insert(pdgnode);
      _vertexProp[pdgnode->id()].node = snode;
    }


    void calc_out_lat() {
      for(int i = 0; i < _sbPDG->num_vec_output(); ++i) {
        calc_out_vport_lat(_sbPDG->vec_out(i));
      }
    }

    void calc_out_vport_lat(SbPDG_VecOutput* pdgvec_out) {
      int max_lat=0;
      for(auto Io=pdgvec_out->output_begin(),Eo=pdgvec_out->output_end();Io!=Eo;++Io) {
        SbPDG_Output* pdgout = *Io;
        //sbnode* out_sbnode = locationOf(pdgout).first;
        int lat_of_out_sbnode = _vertexProp[pdgout->id()].lat;
        std::cout << pdgvec_out->gamsName() << " lat:" << lat_of_out_sbnode << "\n";
        max_lat=std::max(max_lat,lat_of_out_sbnode);
      }
      //std::cout << "max lat: " << max_lat<< "\n";
      _vecProp[pdgvec_out].lat=max_lat;
    }
   
    //Get the mask for a software vector 
    std::vector<bool> maskOf(SbPDG_Vec* pdgvec) {
      auto& vp = _vecProp[pdgvec];
      return vp.mask;
    }

    //Get the software vector for a hardware vector port identifier
    SbPDG_Vec* vportOf(std::pair<bool,int> pn) {
      if(_assignVPort.count(pn)) {
        return _assignVPort[pn];
      } else {
        return NULL;
      }
    }

    //vector to port num
    //true for input
    void assign_vport(SbPDG_Vec* pdgvec, std::pair<bool,int> pn, 
                                         std::vector<bool>mask) {
      /*std::cout << (pn.first ? "input" : "output" )
                << " vector port" << pn.second
                << " assigned to " << pdgvec->gamsName() << "\n";*/
      _assignVPort[pn]=pdgvec;
      auto& vp = _vecProp[pdgvec];
      vp.vport=pn;
      vp.mask=mask;
    }

    bool vecMapped(SbPDG_Vec* p) {
      return _vecProp[p].vport.second!=-1;
    }

    std::pair<bool,int> vecPortOf(SbPDG_Vec* p) {
      return _vecProp[p].vport;
    } 

    //unassign all the input nodes and vector
    void unassign_input_vec(SbPDG_VecInput* pdgvec) {
      for(auto i = pdgvec->input_begin(), e= pdgvec->input_end(); i!=e; ++i) {
        SbPDG_Input* in = *i;
        unassign_pdgnode(in);
      }
      
      auto& vp = _vecProp[pdgvec];
      std::pair<bool,int> pn = vp.vport;
      _assignVPort.erase(pn);
      _vecProp.erase(pdgvec);
    }

    //unassign all the input nodes and vector
    void unassign_output_vec(SbPDG_VecOutput* pdgvec) {
      for(auto i = pdgvec->output_begin(), e= pdgvec->output_end(); i!=e; ++i) {
        SbPDG_Output* out = *i;
        unassign_pdgnode(out);
      }

      auto& vp = _vecProp[pdgvec];
      std::pair<bool,int> pn = vp.vport;
      _assignVPort.erase(pn);
      _vecProp.erase(pdgvec);
    }

    void unassign_edge(SbPDG_Edge* edge) {
      auto& ep = _edgeProp[edge->id()];

      //std::cout << "unassign: " <<edge->name() << "\n";

      _edge_links_mapped-=ep.links.size();
      assert(_edge_links_mapped >=0);

      //Remove all the edges for each of links
      for(auto& link : ep.links) {
        auto& lp = _linkProp[link->id()];
        lp.edges.erase(edge);
        if(lp.edges.size()==0) {
          _links_mapped--;
          assert(_links_mapped >=0);
        }
      }

      //Remove all passthroughs associated with this edge
      for(auto& pt : ep.passthroughs) {
        auto& np = _nodeProp[pt->id()];
        np.is_passthrough-=1; //take one passthrough edge away
        assert(np.is_passthrough>=0);
      }

      _edgeProp[edge->id()].reset();
    }

    //Delete all scheduling data associated with pdgnode, including its
    //mapped locations, and mapping information and metadata for edges
    void unassign_pdgnode(SbPDG_Node* pdgnode) { 
      _num_mapped[pdgnode->type()]--;
      for(auto it=pdgnode->ops_begin(); it!=pdgnode->ops_end(); ++it){
        SbPDG_Edge* edge = *it;
        if(!edge) continue;
        unassign_edge(edge);
      }
      for(auto it=pdgnode->uses_begin(); it!=pdgnode->uses_end(); ++it){
        SbPDG_Edge* edge = *it;
        unassign_edge(edge);
      }

      auto& vp=_vertexProp[pdgnode->id()];
      sbnode* node = vp.node;
      assert(node);
      vp.node=NULL;
      _nodeProp[node->id()].vertices.erase(pdgnode);
    }

    std::unordered_set<SbPDG_Edge*>& edge_list(sblink* link) {
      return _linkProp[link->id()].edges;
    }

    void print_all_mapped() {
      std::cout << "Vertices: ";
      //TODO: can't get vertex/edge from id, need to modify pdg to maintain
      for(unsigned i = 0; i <_vertexProp.size(); ++i) {
        auto& v = _vertexProp[i];
        if(v.node) {
          std::cout << i << "->" << v.node->name() << " ";
        }
      }
      std::cout << "\nEdges: ";
      for(unsigned i = 0; i < _edgeProp.size(); ++i) {
        auto& e = _edgeProp[i];
        if(e.links.size()) {
          std::cout << i << " ";
        }
      }
      std::cout << "\nVec: ";
      for(auto& v : _vecProp) {
        if(v.first && vecMapped(v.first)) {
          std::cout << v.first->name() << " ";
        }
      }
      std::cout << "\n";
    }

    //pdf edge to sblink 
    void assign_edgelink(SbPDG_Edge* pdgedge,sblink* slink) {
      assert(slink);
      assert(pdgedge);
      //assert(_assignLink[slink]==NULL || _assignLink[slink] == pdgedge->def());
      //std::cout << pdgedge->name() << " assigned to " 
      //          << slink->name() << "\n";

      _edge_links_mapped++;
      auto& lp = _linkProp[slink->id()];
      if(lp.edges.size()==0) _links_mapped++;
      lp.edges.insert(pdgedge);
      _edgeProp[pdgedge->id()].links.insert(slink);
    }
 
    //void print_links(SbPDG_Edge* pdgedge) {
    //  for(auto& i : _assignLinkEdge[pdgedge]) {
    //    cout << i->name() << " ";
    //  }
    //  cout << "\n";
    //}
 
    int link_count(SbPDG_Edge* pdgedge) {
      return _edgeProp[pdgedge->id()].links.size();
    }

    std::unordered_set<sblink*>& links_of(SbPDG_Edge* edge) {
      auto& ep = _edgeProp[edge->id()];
      return ep.links;
    }

    void setLatOfLink(sblink* link, int l) {_linkProp[link->id()].lat = l;}
    int  latOfLink(sblink* link) {return _linkProp[link->id()].lat;}

    bool linkAssigned(sblink* link) {
      return _linkProp[link->id()].edges.size();
    }

    //find first node for
    SbPDG_Node* pdgNodeOf(sblink* link) {
      assert(link);
      auto& vec = _linkProp[link->id()].edges;
      return vec.size()==0 ? NULL : (*vec.begin())->def();
    }

    bool nodeAssigned(sbnode* node) {
      return _nodeProp[node->id()].vertices.size()>0;
    }

    //find first node for
    SbPDG_Node* pdgNodeOf(sbnode* node) {
      auto& vec = _nodeProp[node->id()].vertices;
      return vec.size()==0 ? NULL : *vec.begin();
    }
    
    sbnode* locationOf(SbPDG_Node* pdgnode) {
      return _vertexProp[pdgnode->id()].node;
    }
    
    bool isScheduled(SbPDG_Node* pdgnode) {
      return _vertexProp[pdgnode->id()].node!=NULL;
    }
   
    void stat_printOutputLatency();

    //typedef std::vector<sblink*>::const_iterator link_iterator;
    //link_iterator links_begin(SbPDG_Node* n) {return _linksOf[n].begin();}
    //link_iterator links_end(SbPDG_Node* n)   {return _linksOf[n].end();}
    
    
    //wide vector ports
    int numWidePorts() { return _wide_ports.size(); }
    std::vector<int>& widePort(int i) {return _wide_ports[i];}
    void addWidePort(std::vector<int>& port) {_wide_ports.push_back(port);}
    
    
    SbModel* sbModel() {return _sbModel;}
    
    void  calcLatency(int& lat, int& latmis, bool warnMismatch=false);
    void  cheapCalcLatency(int& lat, int& latmis);
    void  calcNodeLatency(SbPDG_Inst*, int& lat, int& latmis);


    bool  fixLatency(int& lat, int& latmis);
    bool  fixLatency_fwd(int& lat, int& latmis);
    bool  fixLatency_bwd(int& lat, int& latmis);
    bool  fixDelay(SbPDG_Output* pdgout, int ed, std::unordered_set<SbPDG_Node*>& visited);
    void  checkOutputMatch(int& latmis);
    
    void calcAssignEdgeLink_single(SbPDG_Node* pdgnode);
    void calcAssignEdgeLink();
   
    void clearAll() {
      _totalViolation=0;
      _assignSwitch.clear();
      _assignVPort.clear();
      _vecProp.clear();
      _vertexProp.clear();
      _edgeProp.clear();
      //
      _nodeProp.clear();
      _linkProp.clear();

      allocate_space();
    }

    //assign outlink to inlink for a switch
    //if the pdgnode is associated with the switch
    //whats is the outlink of the inlink from that node
    void xfer_link_to_switch() {
      using namespace SB_CONFIG;
      using namespace std;
      
      if(_assignSwitch.size()!=0) { //switches already assigned!
        return;
      }
      
      vector< vector<sbswitch> >& switches = _sbModel->subModel()->switches();  //2d switches
      for(int i = 0; i < _sbModel->subModel()->sizex()+1; ++i) {
        for(int j = 0; j < _sbModel->subModel()->sizey()+1; ++j) {
          
          sbswitch* sbsw = &switches[i][j];
          
          sbnode::const_iterator I,E;           //iterate over the links
          for(I=sbsw->ibegin(), E=sbsw->iend(); I!=E; ++I) {
            sblink* inlink = *I;

            if(linkAssigned(inlink)) {
              //inlink to sw associated with a pdg node
              SbPDG_Node* innode = pdgNodeOf(inlink);
              
              sbnode::const_iterator II,EE;
              for(II=sbsw->obegin(), EE=sbsw->oend(); II!=EE; ++II) {
                sblink* outlink = *II;
               
                //check if the pdgnode has same outlink and inlink
                if(linkAssigned(outlink)!=0 && innode ==pdgNodeOf(outlink)) {
                  _assignSwitch[sbsw][outlink]=inlink;
                }
              }//end for out links

            }
          }//end for sin links

        }//end for sizex
      }//end for sizey
    }

  //NOTE/WARN: If you call interpretConfigBits, this will create an
  //sbPDG that you need to delete later
  std::map<SB_CONFIG::sb_inst_t,int> interpretConfigBits();
  void clear_sbpdg();

  bitslices<uint64_t>& slices() {return _bitslices;}

  void add_passthrough_node(sbnode* n) {
    assert(pdgNodeOf(n)==NULL);

    _nodeProp[n->id()].is_passthrough+=1; //add one to edges passing through
  }

  bool isPassthrough(sbnode* n) {return _nodeProp[n->id()].is_passthrough;}

  void print_bit_loc() {
    std::cout << "Primary Config\n";
    std::cout << "Row: " << ROW_LOC 
                         << ":" << ROW_LOC+ROW_BITS-1 << "\n";
    std::cout << "Switches: " << SWITCH_LOC  << ":" 
                              << SWITCH_LOC + SWITCH_BITS-1 << "\n";
    std::cout << "FU Dir: " << FU_DIR_LOC << ":" 
                            << FU_DIR_LOC + FU_DIR_BITS-1 << "\n";
    std::cout << "FU Pred Inv: " << FU_PRED_INV_LOC << ":" 
                                 << FU_PRED_INV_LOC + FU_PRED_INV_BITS-1 << "\n";
    std::cout << "Opcode: " << OPCODE_LOC << ":" 
                            << OPCODE_LOC + OPCODE_BITS-1 << "\n";
    std::cout << "In Del.: "<< IN_DELAY_LOC<<":"
                            << IN_DELAY_LOC + IN_DELAY_BITS-1 << "\n";
  }

  void set_edge_delay(int i, SbPDG_Edge* e) { _edgeProp[e->id()].extra_lat=i; }
  int edge_delay(SbPDG_Edge* e) { return _edgeProp[e->id()].extra_lat;}

  void set_num_links(int i, SbPDG_Edge* e) { _edgeProp[e->id()].num_links=i; }
  int edge_links(SbPDG_Edge* e) { return _edgeProp[e->id()].num_links;}

  void set_link_order(sblink* l, int i) { _linkProp[l->id()].order=i; }
  int link_order(sblink* l) { return _linkProp[l->id()].order; }

  struct LinkProp;
  std::vector<LinkProp>& get_link_prop() {return _linkProp;}

  int num_passthroughs(SbPDG_Edge* e) {
    return _edgeProp[e->id()].passthroughs.size();
  }

  int max_lat() {assert(_max_lat!=-1);  return _max_lat;}
  int max_lat_mis() {return _max_lat_mis;}

  private:


  //called by reconstructSchedule to trace link assignment
  void tracePath(sbnode* ,SbPDG_Node* , 
                 std::map<sbnode*, std::map<SbDIR::DIR,SbDIR::DIR>>&,
                 std::map<sbnode*, SbPDG_Node* >&,
                 std::map<SbPDG_Node*, std::vector<SbDIR::DIR> >&);

  //helper for reconstructing Schedule
  void reconstructSchedule(
    std::map<sbnode*,std::map<SbDIR::DIR,SbDIR::DIR>>&,
    std::map<sbnode*, SbPDG_Node* >&,
    std::map<SbPDG_Node*, std::vector<SbDIR::DIR> >&);


    //Private Data
    SbDIR sbdir;

    bitslices<uint64_t> _bitslices;
    SbModel* _sbModel;
    SbPDG*   _sbPDG;

    int _totalViolation=0;
    int _max_lat=-1, _max_lat_mis=-1; //filled from interpretConfigBits + calcLatency

  public:
    struct VertexProp {
      int min_lat=0, max_lat=0, lat=0, vio=0;
      sbnode* node=NULL;
    };

    struct EdgeProp {
      int num_links=0;
      int extra_lat=0;
      std::unordered_set<sblink*> links;
      std::unordered_set<sbnode*> passthroughs;
      void reset() {
        num_links=0;
        extra_lat=0;
        links.clear();
        passthroughs.clear();
      }
    };

    struct NodeProp {
      int is_passthrough=0;
      std::unordered_set<SbPDG_Node*> vertices;
    };

    struct LinkProp {
      int lat=0, order=-1;
      std::unordered_set<SbPDG_Edge*> edges;
    };

    struct VecProp{ 
      int min_lat=0, max_lat=0,lat=0;
      std::pair<bool,int> vport = {false,-1};
      std::vector<bool> mask;
    };

    int num_insts_mapped() {return _num_mapped[SbPDG_Node::V_INST];}
    int num_inputs_mapped() {return _num_mapped[SbPDG_Node::V_INPUT];}
    int num_outputs_mapped() {return _num_mapped[SbPDG_Node::V_OUTPUT];}
    int num_mapped() { 
      return _num_mapped[SbPDG_Node::V_INST] + 
             _num_mapped[SbPDG_Node::V_INPUT] +
             _num_mapped[SbPDG_Node::V_OUTPUT];
    }
    int inputs_complete() {return num_inputs_mapped() == _sbPDG->num_inputs();}
    int outputs_complete() {return num_outputs_mapped() == _sbPDG->num_outputs();}
    int insts_complete() {return num_insts_mapped() == _sbPDG->num_insts();}

    int isComplete() {return num_mapped() == _sbPDG->num_nodes();}

    int num_links_mapped() {return _links_mapped;}
    int num_edge_links_mapped() {return _edge_links_mapped;}

    int num_left() {return _sbPDG->num_nodes() - num_mapped();}

    void allocate_space() {
      if(_sbPDG) {
        _vertexProp.resize(_sbPDG->num_node_ids());
        _edgeProp.resize(_sbPDG->num_edge_ids());
      }
      if(_sbModel) {
        _nodeProp.resize(_sbModel->subModel()->num_node_ids());
        _linkProp.resize(_sbModel->subModel()->num_link_ids());
      }
    }

  private:
    int _num_mapped[SbPDG_Node::V_NUM_TYPES] = {0}; //init all to zero
    int _links_mapped = 0, _edge_links_mapped = 0;

    std::vector< std::vector<int> > _wide_ports;

    std::map<std::pair<bool,int>, SbPDG_Vec*> _assignVPort; 

    std::unordered_map< SbPDG_Vec*, VecProp > _vecProp;
    std::vector< VertexProp > _vertexProp;
    std::vector< EdgeProp > _edgeProp;

    //vport prop missing datastructure, imiplicit pair for now
    std::vector< NodeProp > _nodeProp;
    std::vector< LinkProp > _linkProp;

    std::map<sbswitch*, std::map<sblink*,sblink*>> _assignSwitch; //out to in
};

#endif
