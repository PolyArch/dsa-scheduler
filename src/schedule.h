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
      _latOf[pdgnode]=lat;
    }
    int latOf(SbPDG_Node* pdgnode) {
      return _latOf[pdgnode];
    }

    void assign_lat_bounds(SbPDG_Node* pdgnode, int min, int max) {
      _latBounds[pdgnode]=std::make_pair(min,max);
    }
    std::pair<int,int> lat_bounds(SbPDG_Node* pdgnode) {
      return _latBounds[pdgnode];
    }

    void assign_edge_pt(SbPDG_Edge* edge, sbnode* pt) {
      add_passthrough_node(pt);
      _passthroughsOf[edge].insert(pt);
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

    int vioOf(SbPDG_Node* n) { return _vioOf[n];}
    void record_violation(SbPDG_Node* n, int violation) {
      _vioOf[n]=violation; 
    }


    //Assign the sbnode to pdgnode and vice verse 
    void assign_node(SbPDG_Node* pdgnode, sbnode* snode) {
      //std::cout << snode->name() << " assigned to " 
      //          << pdgnode->gamsName() << "\n";
      assert(_assignNode.count(snode)==0);
      assert(pdgnode && (uint64_t)pdgnode != 0x1); //don't ask
      _assignNode[snode] = pdgnode;
      _sbnodeOf[pdgnode] = snode;
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
        int lat_of_out_sbnode = _latOf[pdgout];
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

    //sblink to pdgnode
    void assign_link(SbPDG_Node* pdgnode, sblink* slink) {
      //This ensures no 2-way cyclic links -- but this is okay now
      //sbnode* src = slink->orig();
      //for(auto I = src->ibegin(), E = src->iend(); I!=E; ++I) {
      //   sblink* opp_link = *I;
      //   if(opp_link->orig() == slink->dest()) {
      //     if(pdgNodeOf(opp_link) == pdgnode) {
      //       std::cout << pdgnode->name() << " " 
      //            << slink->name() << " " << opp_link->name() << "\n";
      //       assert(0);
      //     }
      //   }
      //}
       assert(slink);
      _assignLink[slink]=pdgnode;
      _linksOf[pdgnode].push_back(slink);
    }

    //pdf edge to sblink 
    void assign_edgelink(SbPDG_Edge* pdgedge,sblink* slink) {
      assert(slink);
      assert(pdgedge);
      assert(_assignLink[slink]==NULL || _assignLink[slink] == pdgedge->def());
      
      assign_link(pdgedge->def(),slink);

      _assignEdgeLink[slink].insert(pdgedge);
      _assignLinkEdge[pdgedge].insert(slink);
    }
  
    int link_count(SbPDG_Edge* pdgedge) {
      return _assignLinkEdge[pdgedge].size();
    }

    void setLatOfLink(sblink* link, int l) {_latOfLink[link] = l;}
    int  latOfLink(sblink* link) {return _latOfLink[link];}

    SbPDG_Node* pdgNodeOf(sblink* link) {
      assert(link);
      if(_assignLink.count(link)==0) {
        return NULL;
      } else {
        return _assignLink[link];
      }
    }

    //sbnode to pdgnode
    SbPDG_Node* pdgNodeOf(sbnode* node) {
      if(_assignNode.count(node)==0) {
        return NULL;
      } else {
        return _assignNode[node];
      }
    }
    
    sbnode* locationOf(SbPDG_Node* pdgnode) {
      if(_sbnodeOf.count(pdgnode)==0) {
        return NULL;
      } else {
        return _sbnodeOf[pdgnode]; 
      }
    }
    
    bool isScheduled(SbPDG_Node* pdgnode) {
      return _sbnodeOf.count(pdgnode)!=0;
    }
   
    void updateLinkCount(sblink* link){
      linkCount[link]++;
    } 

    void stat_printLinkCount();
    void stat_printOutputLatency();

    typedef std::vector<sblink*>::const_iterator link_iterator;
    
    link_iterator links_begin(SbPDG_Node* n) {return _linksOf[n].begin();}
    link_iterator links_end(SbPDG_Node* n)   {return _linksOf[n].end();}
    
    
    //wide vector ports
    int numWidePorts() { return _wide_ports.size(); }
    std::vector<int>& widePort(int i) {return _wide_ports[i];}
    void addWidePort(std::vector<int>& port) {_wide_ports.push_back(port);}
    
    
    SbModel* sbModel() {return _sbModel;}
    
    void  calcLatency(int& lat, int& latmis, bool warnMismatch=false);
    bool  fixLatency(int& lat, int& latmis);
    bool  fixLatency_fwd(int& lat, int& latmis);
    bool  fixLatency_bwd(int& lat, int& latmis);
    bool  fixDelay(SbPDG_Output* pdgout, int ed, std::unordered_set<SbPDG_Node*>& visited);
    void  checkOutputMatch(int& latmis);
    
    void calcAssignEdgeLink_single(SbPDG_Node* pdgnode);
    void calcAssignEdgeLink();
    
    typedef std::unordered_map<sbnode*,SbPDG_Node*>::iterator assign_node_iterator;
    typedef std::unordered_map<sblink*,SbPDG_Node*>::iterator assign_link_iterator;
    typedef std::unordered_map<sblink*,std::set<SbPDG_Edge*>>::iterator assign_edgelink_iterator;
    
    assign_node_iterator assign_node_begin() { return _assignNode.begin(); }
    assign_node_iterator assign_node_end() { return _assignNode.end(); }
//    assign_node_iterator assign_vport_begin() { return _assignVPort.begin(); }
//    assign_node_iterator assign_vport_end() { return _assignVPort.end(); }
    assign_link_iterator assign_link_begin() { return _assignLink.begin(); }
    assign_link_iterator assign_link_end() { return _assignLink.end(); }
    assign_edgelink_iterator assign_edgelink_begin() { return _assignEdgeLink.begin(); }
    assign_edgelink_iterator assign_edgelink_end() { return _assignEdgeLink.end(); }
    
    void clearAll() {
      _totalViolation=0;
      _assignSwitch.clear();
      _assignVPort.clear();
      _vportOf.clear();
      _assignNode.clear();
      _sbnodeOf.clear();
      _latOf.clear();
      _latBounds.clear();
      _passthrough_nodes.clear();
      _vioOf.clear();
      _passthroughsOf.clear();
      _latOfVPort.clear();
      _latOfLink.clear();
      _assignLink.clear();
      _linksOf.clear();
      _assignEdgeLink.clear();
      _assignLinkEdge.clear();
      _extraLatOfEdge.clear();
      _linkOrder.clear();
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

            if(_assignLink.count(inlink)!=0) {
              //inlink to sw associated with a pdg node
              SbPDG_Node* innode = _assignLink[inlink];
              
              sbnode::const_iterator II,EE;
              for(II=sbsw->obegin(), EE=sbsw->oend(); II!=EE; ++II) {
                sblink* outlink = *II;
               
                //check if the pdgnode has same outlink and inlink
                if(_assignLink.count(outlink)!=0 && innode ==_assignLink[outlink]) {
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
    _passthrough_nodes.insert(passthrough);
  }

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

  //static const int FU_PRED_INV_LOC = FU_DIR_LOC + FU_DIR_BITS;
  //static const int FU_PRED_INV_BITS = 1;

  static const int OPCODE_LOC = FU_DIR_LOC + FU_DIR_BITS;
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

  void print_bit_loc() {
    std::cout << "Row: " << ROW_LOC << ":" << ROW_LOC+ROW_BITS-1 << "\n";
    std::cout << "Switches: " << SWITCH_LOC  << ":" << SWITCH_LOC + SWITCH_BITS-1 << "\n";
    std::cout << "FU Dir: " << FU_DIR_LOC << ":" << FU_DIR_LOC + FU_DIR_BITS-1 << "\n";
    std::cout << "Opcode: " << OPCODE_LOC << ":" << OPCODE_LOC + OPCODE_BITS-1 << "\n";
    std::cout << "In Del.: "<< IN_DELAY_LOC<<":"<< IN_DELAY_LOC + IN_DELAY_BITS-1 << "\n";
  }


  void set_edge_delay(int i, SbPDG_Edge* e) { _extraLatOfEdge[e]=i; }
  int edge_delay(SbPDG_Edge* e) { 
    if(_extraLatOfEdge.count(e)) {
      return _extraLatOfEdge[e]; 
    }
    return 0;
  }


  void set_link_order(sblink* l, int i) { _linkOrder[l]=i; }
  int link_order(sblink* l) { return _linkOrder[l]; }

  std::unordered_map<sblink*,int>& get_link_order() {return _linkOrder;}

  int num_passthroughs(SbPDG_Edge* e) {return _passthroughsOf[e].size();}

  int max_lat() {assert(_max_lat!=-1);  return _max_lat;}
  int max_lat_mis() {return _max_lat_mis;}

  private:

    SbDIR sbdir;

    bitslices<uint64_t> _bitslices;
    SbModel* _sbModel;
    SbPDG*   _sbPDG;

    int _totalViolation=0;

    int _max_lat=-1, _max_lat_mis=-1; //filled from interpretConfigBits + calcLatency

    std::unordered_set<sbnode*> _passthrough_nodes; //only for _n_configs > 1
    std::unordered_map<SbPDG_Edge*, std::set<sbnode*>> _passthroughsOf; //for stats
    std::unordered_map<SbPDG_Node*, int> _vioOf; //for stats


    std::unordered_map<sblink*, int> linkCount;
    std::unordered_map<sblink*, int> _latOfLink;
    std::unordered_map<sbnode*, SbPDG_Node*> _assignNode;  //sbnode to pdgnode
    std::unordered_map<SbPDG_Node*, sbnode* > _sbnodeOf;    //pdgnode to sbnode
    std::unordered_map<SbPDG_Node*, int> _latOf; 
    std::unordered_map<SbPDG_Node*, std::pair<int,int>> _latBounds;  //min, max bounds

    std::unordered_map<SbPDG_Edge*, int> _latOfEdge; 
    std::unordered_map<SbPDG_Vec*, int> _latOfVPort; 
    std::unordered_map<sblink*, SbPDG_Node*> _assignLink;   //sblink to pdgnode
    std::unordered_map<SbPDG_Node*, std::vector<sblink*> > _linksOf; //pdgnode to sblink 
    std::unordered_map<sblink*, std::set<SbPDG_Edge*>> _assignEdgeLink; //sblink to pdgedgelinks
    std::unordered_map<SbPDG_Edge*, std::set<sblink*>> _assignLinkEdge; //


    std::unordered_map<SbPDG_Edge*, int> _extraLatOfEdge; 
    std::vector< std::vector<int> > _wide_ports;

    std::unordered_map<sblink*,int> _linkOrder;

    std::map<std::pair<bool,int>, SbPDG_Vec*> _assignVPort;     //vecport to pdfvec
    std::map<SbPDG_Vec*, std::pair<bool,int> > _vportOf;
    std::map<SbPDG_Vec*, std::vector<bool> > _maskOf;


    std::map<sbswitch*,
             std::map<sblink*,sblink*>> _assignSwitch; //out to in
   
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

};


#endif
