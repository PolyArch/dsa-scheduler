#ifndef __SB__SCHEDULE_H__
#define __SB__SCHEDULE_H__

#include <map>
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
      std::ofstream os(fname);
      assert(os.good()); 
      printConfigText(os);
    }
   
    void printConfigBits(std::ostream& os, std::string cfg_name );
    void printConfigVerif(std::ostream& os);
    
    //Rest of Stuff
    SbPDG* sbpdg() const {return _sbPDG;}

    //For a switch, assign the outlink to inlink
    void assign_switch(sbswitch* sbsw, 
                       sblink* slink,
                       sblink* slink_out) {
      _assignSwitch[sbsw][slink_out]=slink;                           //out to in for a sw      
    }

    void assign_lat(SbPDG_Node* pdgnode, int lat=0) {
      _latOf[pdgnode]=lat;
    }
 
    int latOf(SbPDG_Node* pdgnode) {
      return _latOf[pdgnode];
    }

    //Assign the sbnode to pdgnode and vice verse 
    void assign_node(SbPDG_Node* pdgnode, sbnode* snode) {
      //std::cout << snode->name() << " assigned to " 
      //          << pdgnode->gamsName() << "\n";
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
    void assign_vport(SbPDG_Vec* pdgvec, std::pair<bool,int> pn, 
                                         std::vector<bool>mask) {
      /*std::cout << (pn.first ? "input" : "output" )
                << " vector port" << pn.second
                << " assigned to " << pdgvec->gamsName() << "\n";*/
      _assignVPort[pn]=pdgvec;
      _vportOf[pdgvec]=pn;
      _maskOf[pdgvec]=mask;
    }

    //sblink to pdgnode
    void assign_link(SbPDG_Node* pdgnode, sblink* slink) {
      assert(slink);
      _assignLink[slink]=pdgnode;
      _linksOf[pdgnode].push_back(slink);
    }

   //pdf edge to sblink 
    void assign_edgelink(SbPDG_Edge* pdgedge,sblink* slink) {
      assert(slink);
      assert(pdgedge);
      _assignEdgeLink[slink].insert(pdgedge);
    }
    
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

    typedef std::vector<sblink*>::const_iterator link_iterator;
    
    link_iterator links_begin(SbPDG_Node* n) {return _linksOf[n].begin();}
    link_iterator links_end(SbPDG_Node* n)   {return _linksOf[n].end();}
    
    
    //wide vector ports
    int numWidePorts() { return _wide_ports.size(); }
    std::vector<int>& widePort(int i) {return _wide_ports[i];}
    void addWidePort(std::vector<int>& port) {_wide_ports.push_back(port);}
    
    
    SbModel* sbModel() {return _sbModel;}
    
    void calcLatency(int& lat, int& latmis);
    
    void calcAssignEdgeLink_single(SbPDG_Node* pdgnode);
    void calcAssignEdgeLink();
    
    typedef std::map<sbnode*,SbPDG_Node*>::iterator assign_node_iterator;
    typedef std::map<sblink*,SbPDG_Node*>::iterator assign_link_iterator;
    typedef std::map<sblink*,std::set<SbPDG_Edge*>>::iterator assign_edgelink_iterator;
    
    assign_node_iterator assign_node_begin() { return _assignNode.begin(); }
    assign_node_iterator assign_node_end() { return _assignNode.end(); }
//    assign_node_iterator assign_vport_begin() { return _assignVPort.begin(); }
//    assign_node_iterator assign_vport_end() { return _assignVPort.end(); }
    assign_link_iterator assign_link_begin() { return _assignLink.begin(); }
    assign_link_iterator assign_link_end() { return _assignLink.end(); }
    assign_edgelink_iterator assign_edgelink_begin() { return _assignEdgeLink.begin(); }
    assign_edgelink_iterator assign_edgelink_end() { return _assignEdgeLink.end(); }
    
    void clearAll() {
      _assignVPort.clear();
      _vportOf.clear();
      _assignNode.clear();
      _sbnodeOf.clear();
      _latOf.clear();
      _latOfVPort.clear();
      _assignLink.clear();
      _linksOf.clear();
      _assignEdgeLink.clear();
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

  static const int SWITCH_SLICE=5;        //the starting slice position in bitslice

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


  private:

    SbDIR sbdir;

    bitslices<uint64_t> _bitslices;
    SbModel* _sbModel;
    SbPDG*   _sbPDG;
    

    std::set<sbnode*> _passthrough_nodes; //only for _n_configs > 1

    std::map<sblink*, int> linkCount;
    std::map<sbnode*, SbPDG_Node*> _assignNode;  //sbnode to pdgnode
    std::map<SbPDG_Node*, sbnode* > _sbnodeOf;    //pdgnode to sbnode
    std::map<SbPDG_Node*, int> _latOf; 
    std::map<SbPDG_Vec*, int> _latOfVPort; 
    std::map<sblink*, SbPDG_Node*> _assignLink;   //sblink to pdgnode
    std::map<SbPDG_Node*, std::vector<sblink*> > _linksOf; //pdgnode to sblink 
    std::map<sblink*, std::set<SbPDG_Edge*>> _assignEdgeLink; //sblink to pdgedgelinks
    std::vector< std::vector<int> > _wide_ports;

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
