#ifndef __SB__SCHEDULE_H__
#define __SB__SCHEDULE_H__

#include <map>
#include "model.h"
#include "sub_model.h"
#include "sbpdg.h"
#include <iostream>
#include <fstream>
#include "bitslice.h"


//How do you choose which switch in each row and FU to pass ??
struct sw_config {
  unsigned pred:2;
  unsigned opcode:5;
  unsigned fu_in1:2;    //cfg for 1st input of FU
  unsigned fu_in2:2;    //cfg for 2nd input of FU
  unsigned fu_out:2;    //cfg for 3rd input of FU
  unsigned  sw_s:3;     //select lien for output muxes
  unsigned sw_se:3;
  unsigned  sw_e:3;
  unsigned sw_ne:3;
  unsigned  sw_n:3;
  unsigned sw_nw:3;
  unsigned  sw_w:3;
  unsigned sw_sw:3;
  unsigned   row:2;  //can address only 4 rows -- need to update it
};


class Schedule {
  public:
    Schedule(std::string filename, bool multi_config=false); //Read in schedule (both sbmodel, sbpdg, and schedule from file)

    Schedule(SB_CONFIG::SbModel* model, SbPDG* pdg ) : _sbModel(model), _sbPDG(pdg) {}
    Schedule(SB_CONFIG::SbModel* model) : _sbModel(model), _sbPDG(NULL) {}


    //Scheduling Interface:
    void printAllConfigs(const char *base);
    bool spilled(SbPDG_Node*);
    
    std::pair<int,int> getConfigAndPort(SbPDG_Node*);
    
    int nConfigs() {return _n_configs;}
    
    //Old Interface:
    
    int getPortFor(SbPDG_Node*);
    
    void printConfigText(std::ostream& os, int config=0);
    void printConfigText(const char *fname, int config=0) {
      std::ofstream os(fname);
      assert(os.good()); 
      printConfigText(os,config);
    }
   
    void printConfigBits(std::ostream& os, std::string cfg_name );
    
    
    //Rest of Stuff
    SbPDG* sbpdg() const {return _sbPDG;}

    //For a switch, assign the outlink to inlink
    void assign_switch(SB_CONFIG::sbswitch* sbsw, 
                       SB_CONFIG::sblink* slink,
                       SB_CONFIG::sblink* slink_out) {
      _assignSwitch[sbsw][slink_out]=slink;                           //out to in for a sw      
    }


    //Assign the sbnode, config pair to pdgnode and vice verse 
    void assign_node(SbPDG_Node* pdgnode, SB_CONFIG::sbnode* snode, int config=0) {
      _assignNode[std::make_pair(snode,config)] = pdgnode;
      _sbnodeOf[pdgnode]=std::make_pair(snode,config);
    }

    //vector to port num
    void assign_vport(SbPDG_Vec* pdgvec, std::pair<bool,int> pn) {
      _assignVPort[pn]=pdgvec;
      _vportOf[pdgvec]=pn;
    }

    //sblink to pdgnode
    void assign_link(SbPDG_Node* pdgnode, SB_CONFIG::sblink* slink, int config=0) {
      assert(slink);
      std::pair<SB_CONFIG::sblink*,int> new_pair = std::make_pair(slink,config);
      _assignLink[new_pair]=pdgnode;
      _linksOf[pdgnode].push_back(new_pair);
    }

   //pdf edge to sblink 
    void assign_edgelink(SbPDG_Edge* pdgedge,SB_CONFIG::sblink* slink, int config=0) {
      assert(slink);
      assert(pdgedge);
      std::pair<SB_CONFIG::sblink*,int> new_pair = std::make_pair(slink,config);
      _assignEdgeLink[new_pair].insert(pdgedge);
    }
    
    SbPDG_Node* pdgNodeOf(SB_CONFIG::sblink* link, int config) {
      assert(link);
      std::pair<SB_CONFIG::sblink*,int> thing = std::make_pair(link,config);
      if(_assignLink.count(thing)==0) {
        return NULL;
      } else {
        return _assignLink[thing];
      }
    }

    SbPDG_Node* pdgNodeOf(SB_CONFIG::sbnode* node) {
      return pdgNodeOf(node,0);
    }

    //sbnode to pdgnode
    SbPDG_Node* pdgNodeOf(SB_CONFIG::sbnode* node, int config) {
      std::pair<SB_CONFIG::sbnode*,int> thing = std::make_pair(node,config);
      if(_assignNode.count(thing)==0) {
        return NULL;
      } else {
        return _assignNode[thing];
      }
    }
    
    std::pair<SB_CONFIG::sbnode*,int> locationOf(SbPDG_Node* pdgnode) {
      if(_sbnodeOf.count(pdgnode)==0) {
        return std::make_pair((SB_CONFIG::sbnode*)0,-1);
      } else {
        return _sbnodeOf[pdgnode]; 
      }
    }
    
    bool isScheduled(SbPDG_Node* pdgnode) {
      return _sbnodeOf.count(pdgnode)!=0;
    }
    
    
    typedef std::vector<std::pair<SB_CONFIG::sblink*,int>>::const_iterator link_iterator;
    
    link_iterator links_begin(SbPDG_Node* n) {return _linksOf[n].begin();}
    link_iterator links_end(SbPDG_Node* n)   {return _linksOf[n].end();}
    
    /*
    bool isNodeScheduled(SB_CONFIG::sbnode* dnode, int config=0) {
      return  
    }*/
    
    void setNConfigs(int n) {_n_configs=n;}

    void addForward(int config1, int port1, int config2, int port2) {
       _forwardMap[std::make_pair(config1,port1)]=std::make_pair(config2,port2); 
    }
   
    //wide vector ports
    int numWidePorts() { return _wide_ports.size(); }
    std::vector<int>& widePort(int i) {return _wide_ports[i];}
    void addWidePort(std::vector<int>& port) {_wide_ports.push_back(port);}
    
    typedef std::map< std::pair<int,int>,std::pair<int,int> >::iterator forward_iterator;
    forward_iterator fbegin() {return _forwardMap.begin();} 
    forward_iterator fend() {return _forwardMap.end();}
    
    SB_CONFIG::SbModel* sbModel() {return _sbModel;}
    
    void calcLatency(int& lat, int& latmis);
    
    void calcAssignEdgeLink_single(SbPDG_Node* pdgnode);
    void calcAssignEdgeLink();
    
    typedef std::map<std::pair<SB_CONFIG::sbnode*,int>,SbPDG_Node*>::iterator assign_node_iterator;
    typedef std::map<std::pair<SB_CONFIG::sblink*,int>,SbPDG_Node*>::iterator assign_link_iterator;
    typedef std::map<std::pair<SB_CONFIG::sblink*,int>,std::set<SbPDG_Edge*>>::iterator assign_edgelink_iterator;
    
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
      _assignLink.clear();
      _linksOf.clear();
      _assignEdgeLink.clear();
      _forwardMap.clear();
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
      
      int config=0;
      vector< vector<sbswitch> >& switches = _sbModel->subModel()->switches();  //2d switches
      for(int i = 0; i < _sbModel->subModel()->sizex()+1; ++i) {
        for(int j = 0; j < _sbModel->subModel()->sizey()+1; ++j) {
          
          sbswitch* sbsw = &switches[i][j];
          
          sbnode::const_iterator I,E;           //iterate over the links
          for(I=sbsw->ibegin(), E=sbsw->iend(); I!=E; ++I) {
            sblink* inlink = *I;

            if(_assignLink.count(make_pair(inlink,config))!=0) {
              //inlink to sw associated with a pdg node
              SbPDG_Node* innode = _assignLink[make_pair(inlink,config)];
              
              sbnode::const_iterator II,EE;
              for(II=sbsw->obegin(), EE=sbsw->oend(); II!=EE; ++II) {
                sblink* outlink = *II;
               
                //check if the pdgnode has same outlink and inlink
                if(_assignLink.count(make_pair(outlink,config))!=0 && innode ==_assignLink[make_pair(outlink,config)]) {
                  _assignSwitch[sbsw][outlink]=inlink;
                }
              }//end for out links

            }
          }//end for sin links

        }//end for sizex
      }//end for sizey

    }

  void interpretConfigBits();

  bitslices<uint64_t>& slices() {return _bitslices;}

  void add_passthrough_node(SB_CONFIG::sbnode* passthrough) {
    _passthrough_nodes.insert(passthrough);
  }

  private:
    static const int IN_ACT_SLICE=0;
    static const int OUT_ACT_SLICE=1;
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

    static const int FU_PRED_INV_LOC = FU_DIR_LOC + FU_DIR_BITS;
    static const int FU_PRED_INV_BITS = 1;

    static const int OPCODE_LOC = FU_PRED_INV_LOC + FU_PRED_INV_BITS;
    static const int OPCODE_BITS = 5;

    SB_CONFIG::SbDIR sbdir;

    bitslices<uint64_t> _bitslices;
    int _n_configs;   
    SB_CONFIG::SbModel* _sbModel;
    SbPDG*   _sbPDG;

    std::set<SB_CONFIG::sbnode*> _passthrough_nodes;

    std::map<std::pair<SB_CONFIG::sbnode*, int>, SbPDG_Node*> _assignNode;  //sbnode to pdgnode
    std::map<SbPDG_Node*, std::pair<SB_CONFIG::sbnode*,int> > _sbnodeOf;    //pdgnode to sbnode
    std::map<SbPDG_Node*, int> _latOf; 
    std::map<std::pair<SB_CONFIG::sblink*,int>, SbPDG_Node*> _assignLink;   //sblink to pdgnode
    std::map<SbPDG_Node*, std::vector<std::pair<SB_CONFIG::sblink*, int>> > _linksOf; //pdgnode to sblink 
    std::map<std::pair<SB_CONFIG::sblink*,int>, std::set<SbPDG_Edge*>> _assignEdgeLink; //sblink to pdgedgelinks
    std::map< std::pair<int,int>,std::pair<int,int> > _forwardMap;      //forward maps of ports
    std::vector< std::vector<int> > _wide_ports;

    std::map<std::pair<bool,int>, SbPDG_Vec*> _assignVPort;     //vecport to pdfvec
    std::map<SbPDG_Vec*, std::pair<bool,int> > _vportOf;

    std::map<SB_CONFIG::sbswitch*,
             std::map<SB_CONFIG::sblink*,SB_CONFIG::sblink*>> _assignSwitch; //out to in
   
    //called by reconstructSchedule to trace link assignment
    void tracePath(SB_CONFIG::sbnode* ,SbPDG_Node* , 
                   std::map<SB_CONFIG::sbnode*, std::map<SB_CONFIG::SbDIR::DIR,SB_CONFIG::SbDIR::DIR>>&,
                   std::map<SB_CONFIG::sbnode*, SbPDG_Node* >&,
                   std::map<SbPDG_Node*, std::vector<SB_CONFIG::SbDIR::DIR> >&);

    //helper for reconstructing Schedule
    void reconstructSchedule(
      std::map<SB_CONFIG::sbnode*,std::map<SB_CONFIG::SbDIR::DIR,SB_CONFIG::SbDIR::DIR>>&,
      std::map<SB_CONFIG::sbnode*, SbPDG_Node* >&,
      std::map<SbPDG_Node*, std::vector<SB_CONFIG::SbDIR::DIR> >&);

};

#endif
