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

    Schedule(SbModel* model, SbPDG* pdg ) : _sbModel(model), _sbPDG(pdg) {}
    Schedule(SbModel* model) : _sbModel(model), _sbPDG(NULL) {}


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

    //Assign the sbnode, config pair to pdgnode and vice verse 
    void assign_node(SbPDG_Node* pdgnode, sbnode* snode, int config=0) {
      _assignNode[std::make_pair(snode,config)] = pdgnode;
      _sbnodeOf[pdgnode]=std::make_pair(snode,config);
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
      std::cout << "max lat: " << max_lat<< "\n";
      _latOfVPort[pdgvec_out]=max_lat;
    }

    //vector to port num
    void assign_vport(SbPDG_Vec* pdgvec, std::pair<bool,int> pn) {
      //std::cout << " -- Assigning vport " << pdgvec->gamsName() << " " << pn.first << "\n";
      _assignVPort[pn]=pdgvec;
      _vportOf[pdgvec]=pn;

      //if(pn.first == false) { //false is output
      //  calc_out_vport_lat(pdgvec);
      //}
    }

    //sblink to pdgnode
    void assign_link(SbPDG_Node* pdgnode, sblink* slink, int config=0) {
      assert(slink);
      std::pair<sblink*,int> new_pair = std::make_pair(slink,config);
      _assignLink[new_pair]=pdgnode;
      _linksOf[pdgnode].push_back(new_pair);
    }

   //pdf edge to sblink 
    void assign_edgelink(SbPDG_Edge* pdgedge,sblink* slink, int config=0) {
      assert(slink);
      assert(pdgedge);
      std::pair<sblink*,int> new_pair = std::make_pair(slink,config);
      _assignEdgeLink[new_pair].insert(pdgedge);
    }
    
    SbPDG_Node* pdgNodeOf(sblink* link, int config) {
      assert(link);
      std::pair<sblink*,int> thing = std::make_pair(link,config);
      if(_assignLink.count(thing)==0) {
        return NULL;
      } else {
        return _assignLink[thing];
      }
    }

    SbPDG_Node* pdgNodeOf(sbnode* node) {
      return pdgNodeOf(node,0);
    }

    //sbnode to pdgnode
    SbPDG_Node* pdgNodeOf(sbnode* node, int config) {
      std::pair<sbnode*,int> thing = std::make_pair(node,config);
      if(_assignNode.count(thing)==0) {
        return NULL;
      } else {
        return _assignNode[thing];
      }
    }
    
    std::pair<sbnode*,int> locationOf(SbPDG_Node* pdgnode) {
      if(_sbnodeOf.count(pdgnode)==0) {
        return std::make_pair((sbnode*)0,-1);
      } else {
        return _sbnodeOf[pdgnode]; 
      }
    }
    
    bool isScheduled(SbPDG_Node* pdgnode) {
      return _sbnodeOf.count(pdgnode)!=0;
    }
    
    
    typedef std::vector<std::pair<sblink*,int>>::const_iterator link_iterator;
    
    link_iterator links_begin(SbPDG_Node* n) {return _linksOf[n].begin();}
    link_iterator links_end(SbPDG_Node* n)   {return _linksOf[n].end();}
    
    /*
    bool isNodeScheduled(sbnode* dnode, int config=0) {
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
    
    SbModel* sbModel() {return _sbModel;}
    
    void calcLatency(int& lat, int& latmis);
    
    void calcAssignEdgeLink_single(SbPDG_Node* pdgnode);
    void calcAssignEdgeLink();
    
    typedef std::map<std::pair<sbnode*,int>,SbPDG_Node*>::iterator assign_node_iterator;
    typedef std::map<std::pair<sblink*,int>,SbPDG_Node*>::iterator assign_link_iterator;
    typedef std::map<std::pair<sblink*,int>,std::set<SbPDG_Edge*>>::iterator assign_edgelink_iterator;
    
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

  void add_passthrough_node(sbnode* passthrough) {
    _passthrough_nodes.insert(passthrough);
  }

  private:
    static const int IN_ACT_SLICE=0;
    static const int OUT_ACT_SLICE=1;
    static const int DELAY_SLICE_1=2;
    static const int DELAY_SLICE_2=3;
    static const int DELAY_SLICE_3=4;
    static const int BITS_PER_DELAY=4;
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

    SbDIR sbdir;

    bitslices<uint64_t> _bitslices;
    int _n_configs;   
    SbModel* _sbModel;
    SbPDG*   _sbPDG;

    std::set<sbnode*> _passthrough_nodes;

    std::map<std::pair<sbnode*, int>, SbPDG_Node*> _assignNode;  //sbnode to pdgnode
    std::map<SbPDG_Node*, std::pair<sbnode*,int> > _sbnodeOf;    //pdgnode to sbnode
    std::map<SbPDG_Node*, int> _latOf; 
    std::map<SbPDG_Vec*, int> _latOfVPort; 
    std::map<std::pair<sblink*,int>, SbPDG_Node*> _assignLink;   //sblink to pdgnode
    std::map<SbPDG_Node*, std::vector<std::pair<sblink*, int>> > _linksOf; //pdgnode to sblink 
    std::map<std::pair<sblink*,int>, std::set<SbPDG_Edge*>> _assignEdgeLink; //sblink to pdgedgelinks
    std::map< std::pair<int,int>,std::pair<int,int> > _forwardMap;      //forward maps of ports
    std::vector< std::vector<int> > _wide_ports;

    std::map<std::pair<bool,int>, SbPDG_Vec*> _assignVPort;     //vecport to pdfvec
    std::map<SbPDG_Vec*, std::pair<bool,int> > _vportOf;

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
