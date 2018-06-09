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


class Schedule {
  public:
    Schedule(std::string filename); //Read in schedule (both sbmodel, sbpdg, and schedule from file)

    Schedule(SbModel* model, SbPDG* pdg ) : _sbModel(model), _sbPDG(pdg) {}
    Schedule(SbModel* model) : _sbModel(model), _sbPDG(NULL) {}

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

    void assign_lat(SbPDG_Node* pdgnode, int lat) {
      _vertexProp[pdgnode].lat=lat;
    }
    int latOf(SbPDG_Node* pdgnode) {
      return _vertexProp[pdgnode].lat;
    }

    void assign_lat_bounds(SbPDG_Node* pdgnode, int min, int max) {
      auto& vertex_prop = _vertexProp[pdgnode];
      vertex_prop.min_lat=min;
      vertex_prop.max_lat=max;
    }
    std::pair<int,int> lat_bounds(SbPDG_Node* pdgnode) {
      auto& vertex_prop = _vertexProp[pdgnode];
      return std::make_pair(vertex_prop.min_lat, vertex_prop.max_lat);
    }

    void assign_edge_pt(SbPDG_Edge* edge, sbnode* pt) {
      add_passthrough_node(pt);
      _edgeProp[edge].passthroughs.insert(pt);
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

    int vioOf(SbPDG_Node* n) { return _vertexProp[n].vio;}
    void record_violation(SbPDG_Node* n, int violation) {
      _vertexProp[n].vio=violation; 
    }


    //Assign the sbnode to pdgnode and vice verse 
    void assign_node(SbPDG_Node* pdgnode, sbnode* snode) {
      //std::cout << snode->name() << " assigned to " 
      //          << pdgnode->gamsName() << "\n";
      assert(pdgnode); 
      _nodeProp[snode].vertices.insert(pdgnode);
      _vertexProp[pdgnode].node = snode;
    }

    //Un assign the sbnode to pdgnode and vice verse 
    void unassign_node(SbPDG_Node* pdgnode, sbnode* snode) {
      assert(pdgnode); 
      _nodeProp[snode].vertices.erase(pdgnode);
      _vertexProp[pdgnode].node=NULL;
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
        int lat_of_out_sbnode = _vertexProp[pdgout].lat;
        std::cout << pdgvec_out->gamsName() << " lat:" << lat_of_out_sbnode << "\n";
        max_lat=std::max(max_lat,lat_of_out_sbnode);
      }
      //std::cout << "max lat: " << max_lat<< "\n";
      _latOfVPort[pdgvec_out]=max_lat;
    }
   
    //Get the mask for a software vector 
    std::vector<bool> maskOf(SbPDG_Vec* pdgvec) {
      assert(_maskOf.count(pdgvec));
      return _maskOf[pdgvec];
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
      _vportOf[pdgvec]=pn;
      _maskOf[pdgvec]=mask;
    }

    std::pair<bool,int> vecPortOf(SbPDG_Vec* p) {
      return _vportOf[p];
    } 

    void unassign_link(SbPDG_Node* pdgnode, sblink* slink) {
      unassign_link_internal(pdgnode, slink);
      auto& links = _vertexProp[pdgnode].links;
      std::remove(links.begin(),links.end(),slink);
    }

    //Delete all scheduling data associated with pdgnode, including its
    //mapped locations, and mapping information and metadata for edges
    void unassign_pdgnode_completely(SbPDG_Node* pdgnode) { 
//      for(auto it=pdgnode->uses_begin(); it!=pdgnode->uses_end(); ++it){
//        SbPDG_Edge* edge = *it;
//        SbPDG_Node* inc_node = edge->def();
//      }

      sbnode* node = locationOf(pdgnode);
      assert(node);
      for(sblink* slink : _vertexProp[pdgnode].links) {
        unassign_link_internal(pdgnode, slink);
      }
      _vertexProp[pdgnode].links.clear();

      unassign_node(pdgnode,node);

      //TODO: need to reset things like lat bound, etc
    }

    //sblink to pdgnode
    void assign_link(SbPDG_Node* pdgnode, sblink* slink) {
      sbnode* src = slink->orig();
      sbfu* src_fu = dynamic_cast<sbfu*>(src);
      assert(!src_fu || pdgNodeOf(src_fu)==NULL || 
             pdgNodeOf(src_fu) == pdgnode); 
  
       assert(slink);
      _linkProp[slink].vertices.insert(pdgnode);
      _vertexProp[pdgnode].links.push_back(slink);
    }

    std::unordered_set<SbPDG_Edge*>& edge_list(sblink* link) {
      return _linkProp[link].edges;
    }

    //pdf edge to sblink 
    void assign_edgelink(SbPDG_Edge* pdgedge,sblink* slink) {
      assert(slink);
      assert(pdgedge);
      //assert(_assignLink[slink]==NULL || _assignLink[slink] == pdgedge->def());
      
      assign_link(pdgedge->def(),slink);

      _linkProp[slink].edges.insert(pdgedge);
      _edgeProp[pdgedge].links.insert(slink);
    }
 
    //void print_links(SbPDG_Edge* pdgedge) {
    //  for(auto& i : _assignLinkEdge[pdgedge]) {
    //    cout << i->name() << " ";
    //  }
    //  cout << "\n";
    //}
 
    int link_count(SbPDG_Edge* pdgedge) {
      return _edgeProp[pdgedge].links.size();
    }

    void setLatOfLink(sblink* link, int l) {_linkProp[link].lat = l;}
    int  latOfLink(sblink* link) {return _linkProp[link].lat;}

    bool linkAssigned(sblink* link) {
      return _linkProp[link].vertices.size();
    }

    //find first node for
    SbPDG_Node* pdgNodeOf(sblink* link) {
      assert(link);
      auto& vec = _linkProp[link].vertices;
      return vec.size()==0 ? NULL : *vec.begin();
    }

    bool nodeAssigned(sbnode* node) {
      return _nodeProp[node].vertices.size()>0;
    }

    //find first node for
    SbPDG_Node* pdgNodeOf(sbnode* node) {
      auto& vec = _nodeProp[node].vertices;
      return vec.size()==0 ? NULL : *vec.begin();
    }
    
    sbnode* locationOf(SbPDG_Node* pdgnode) {
      return _vertexProp[pdgnode].node;
    }
    
    bool isScheduled(SbPDG_Node* pdgnode) {
      return _vertexProp[pdgnode].node!=NULL;
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

    bool  fixLatency(int& lat, int& latmis);
    bool  fixLatency_fwd(int& lat, int& latmis);
    bool  fixLatency_bwd(int& lat, int& latmis);
    bool  fixDelay(SbPDG_Output* pdgout, int ed, std::unordered_set<SbPDG_Node*>& visited);
    void  checkOutputMatch(int& latmis);
    
    void calcAssignEdgeLink_single(SbPDG_Node* pdgnode);
    void calcAssignEdgeLink();
   
    typedef std::unordered_map<sbnode*,
              std::unordered_set<SbPDG_Node*>>::iterator assign_node_iterator;
    typedef std::unordered_map<sblink*,
              std::unordered_set<SbPDG_Node*>>::iterator assign_link_iterator;
    typedef std::unordered_map<sblink*,
            std::unordered_set<SbPDG_Edge*>>::iterator assign_edgelink_iterator;

    //assign_node_iterator assign_node_begin() { return _assignNode.begin(); }
    //assign_node_iterator assign_node_end() { return _assignNode.end(); }
    //auto assign_vport_begin() { return _assignVPort.begin(); }
    //auto assign_vport_end() { return _assignVPort.end(); }
    //assign_link_iterator assign_link_begin() { return _assignLink.begin(); }
    //assign_link_iterator assign_link_end() { return _assignLink.end(); }
    //assign_edgelink_iterator assign_edgelink_begin() { 
    //  return _assignEdgeLink.begin(); }
    //assign_edgelink_iterator assign_edgelink_end() { 
    //  return _assignEdgeLink.end(); }
    
    void clearAll() {
      _totalViolation=0;
      _assignSwitch.clear();
      _assignVPort.clear();
      _vportOf.clear();
      _latOfVPort.clear();
      _edgeProp.clear();
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

  void add_passthrough_node(sbnode* passthrough) {
    assert(pdgNodeOf(passthrough)==NULL);

    _nodeProp[passthrough].is_passthrough=true;
  }

  bool isPassthrough(sbnode* n) {return _nodeProp[n].is_passthrough;}

  static const int IN_ACT_SLICE=0;
  static const int OUT_ACT_SLICE=1;
  //static const int DELAY_SLICE_1=2;
  //static const int DELAY_SLICE_2=3;
  //static const int DELAY_SLICE_3=4;
  //static const int BITS_PER_DELAY=4;
  static const int VP_MAP_SLICE_1=2;
  static const int VP_MAP_SLICE_2=3;
  static const int VP_MAP_SLICE_OUT=4;

  static const int NUM_DFG_GROUPS=6;
  static const int IN_ACT_GROUP12=5;
  static const int IN_ACT_GROUP34=6;
  static const int IN_ACT_GROUP56=7;

  static const int OUT_ACT_GROUP12=8;
  static const int OUT_ACT_GROUP34=9;
  static const int OUT_ACT_GROUP56=10;

  static const int SWITCH_SLICE=11;        //the starting slice position in bitslice

  static const int NUM_IN_DIRS=8;
  static const int NUM_OUT_DIRS=8;
  static const int NUM_IN_FU_DIRS=3;
  static const int BITS_PER_DIR=3;
  static const int BITS_PER_FU_DIR=3;

  static const int ROW_LOC=0;
  static const int ROW_BITS=4;

  static const int SWITCH_LOC = ROW_LOC + ROW_BITS;
  static const int SWITCH_BITS = BITS_PER_DIR * NUM_OUT_DIRS;

  static const int FU_DIR_LOC = SWITCH_LOC + SWITCH_BITS;
  static const int FU_DIR_BITS = BITS_PER_FU_DIR * NUM_IN_FU_DIRS;

  static const int FU_PRED_INV_LOC = FU_DIR_LOC + FU_DIR_BITS;
  static const int FU_PRED_INV_BITS = 1;

  static const int OPCODE_LOC = FU_PRED_INV_LOC + FU_PRED_INV_BITS;
  static const int OPCODE_BITS = 6;

  static const int IN_DELAY_LOC = OPCODE_LOC + OPCODE_BITS;
  static const int BITS_PER_DELAY = 4;
  static const int NUM_DELAY = 3;
  static const int IN_DELAY_BITS = BITS_PER_DELAY*NUM_DELAY;

  //Config Message uses same row loc:
  //static const int ROW_LOC=0;
  //static const int ROW_BITS=4;
  static const int COL_LOC=ROW_LOC+ROW_BITS;
  static const int COL_BITS=4;

  static const int IS_IMM_LOC=COL_LOC+COL_BITS;
  static const int IS_IMM_BITS=1;

  static const int CTRL_LOC=IS_IMM_LOC+IS_IMM_BITS;
  static const int CTRL_BITS=32;

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

  void set_edge_delay(int i, SbPDG_Edge* e) { _edgeProp[e].extra_lat=i; }
  int edge_delay(SbPDG_Edge* e) { return _edgeProp[e].extra_lat;}

  void set_num_links(int i, SbPDG_Edge* e) { _edgeProp[e].num_links=i; }
  int edge_links(SbPDG_Edge* e) { return _edgeProp[e].num_links;}

  void set_link_order(sblink* l, int i) { _linkProp[l].order=i; }
  int link_order(sblink* l) { return _linkProp[l].order; }

  struct LinkProp;
  std::unordered_map<sblink*,LinkProp>& get_link_prop() {return _linkProp;}

  int num_passthroughs(SbPDG_Edge* e) {return _edgeProp[e].passthroughs.size();}

  int max_lat() {assert(_max_lat!=-1);  return _max_lat;}
  int max_lat_mis() {return _max_lat_mis;}

  private:
  void unassign_link_internal(SbPDG_Node* pdgnode, sblink* slink) {
    _linkProp[slink].vertices.erase(pdgnode); 
  }


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
    struct EdgeProp {
      int num_links=0;
      int extra_lat=0;
      std::unordered_set<sblink*> links;
      std::unordered_set<sbnode*> passthroughs;
    };

    struct VertexProp {
      int min_lat=0, max_lat=0, lat=0, vio=0;
      sbnode* node=NULL;
      std::vector<sblink*> links;
    };

    struct NodeProp {
      bool is_passthrough=false;
      std::unordered_set<SbPDG_Node*> vertices;
    };

    struct LinkProp {
      int lat=0, order=-1;
      std::unordered_set<SbPDG_Node*> vertices;
      std::unordered_set<SbPDG_Edge*> edges;
    };
  private:

    //std::unordered_map<sblink*, std::unordered_set<SbPDG_Node*>> _assignLink;   
    //std::unordered_map<sblink*, std::unordered_set<SbPDG_Edge*>> _assignEdgeLink; 



    std::vector< std::vector<int> > _wide_ports;

    std::unordered_map<SbPDG_Vec*, int> _latOfVPort; 
    std::map<std::pair<bool,int>, SbPDG_Vec*> _assignVPort;     //vecport to pdfvec
    std::map<SbPDG_Vec*, std::pair<bool,int> > _vportOf;
    std::map<SbPDG_Vec*, std::vector<bool> > _maskOf;


    std::unordered_map< SbPDG_Edge*, EdgeProp > _edgeProp;
    std::unordered_map< SbPDG_Node*, VertexProp > _vertexProp;
    std::unordered_map< sbnode*, NodeProp > _nodeProp;
    std::unordered_map< sblink*, LinkProp > _linkProp;


    std::map<sbswitch*, std::map<sblink*,sblink*>> _assignSwitch; //out to in
};


#endif
