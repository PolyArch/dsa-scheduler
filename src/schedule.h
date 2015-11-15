#ifndef SCHEDULE_H
#define SCHEDULE_H

#include <map>
#include "model.h"
#include "sub_model.h"
#include "dypdg.h"
#include <iostream>
#include <fstream>

class Schedule {
  public:
    Schedule(std::string filename, bool multi_config=false); //Read in schedule (both dymodel, dypdg, and schedule from file)
    Schedule(SB_CONFIG::DyModel* model, DyPDG* pdg ) : _dyModel(model),_dyPDG(pdg) {}
    
    //Scheduling Interface:
    void printAllConfigs(const char *base);
    bool spilled(DyPDG_Node*);
    
    std::pair<int,int> getConfigAndPort(DyPDG_Node*);
    
    int nConfigs() {return _n_configs;}
    
    //Old Interface:
    
    int getPortFor(DyPDG_Node*);
    
    void printConfigText(std::ostream& os, int config=0);
    void printConfigText(const char *fname, int config=0) {
      std::ofstream os(fname);
      assert(os.good()); 
      printConfigText(os,config);
    }
    
    
    
    //Rest of Stuff
    DyPDG* dypdg() const {return _dyPDG;}

    void assign_switch(SB_CONFIG::dyswitch* dysw, 
                       SB_CONFIG::dylink* dlink,
                       SB_CONFIG::dylink* dlink_out) {
      _assignSwitch[dysw][dlink_out]=dlink;
    }

    void assign_node(DyPDG_Node* pdgnode,SB_CONFIG::dynode* dnode, int config=0) {
      _assignNode[std::make_pair(dnode,config)]=pdgnode;
      _dynodeOf[pdgnode]=std::make_pair(dnode,config);
    }

    void assign_link(DyPDG_Node* pdgnode,SB_CONFIG::dylink* dlink, int config=0) {
      assert(dlink);
      std::pair<SB_CONFIG::dylink*,int> thing = std::make_pair(dlink,config);
      _assignLink[thing]=pdgnode;
      _linksOf[pdgnode].push_back(thing);
    }
    
    void assign_edgelink(DyPDG_Edge* pdgedge,SB_CONFIG::dylink* dlink, int config=0) {
      assert(dlink);
      assert(pdgedge);
      std::pair<SB_CONFIG::dylink*,int> thing = std::make_pair(dlink,config);
      //_assignLink[thing]=pdgnode;
      //_linksOf[pdgnode].push_back(thing);
      _assignEdgeLink[thing].insert(pdgedge);
    }
    
    DyPDG_Node* pdgNodeOf(SB_CONFIG::dylink* link, int config) {
      assert(link);
      std::pair<SB_CONFIG::dylink*,int> thing = std::make_pair(link,config);
      if(_assignLink.count(thing)==0) {
        return NULL;
      } else {
        return _assignLink[thing];
      }
    }
    
    DyPDG_Node* pdgNodeOf(SB_CONFIG::dynode* node, int config) {
      std::pair<SB_CONFIG::dynode*,int> thing = std::make_pair(node,config);
      if(_assignNode.count(thing)==0) {
        return NULL;
      } else {
        return _assignNode[thing];
      }
    }
    
    std::pair<SB_CONFIG::dynode*,int> locationOf(DyPDG_Node* pdgnode) {
      if(_dynodeOf.count(pdgnode)==0) {
        return std::make_pair((SB_CONFIG::dynode*)0,-1);
      } else {
        return _dynodeOf[pdgnode]; 
      }
    }
    
    bool isScheduled(DyPDG_Node* pdgnode) {
      return _dynodeOf.count(pdgnode)!=0;
    }
    
    
    typedef std::vector<std::pair<SB_CONFIG::dylink*,int>>::const_iterator link_iterator;
    
    link_iterator links_begin(DyPDG_Node* n) {return _linksOf[n].begin();}
    link_iterator links_end(DyPDG_Node* n)   {return _linksOf[n].end();}
    
    /*
    bool isNodeScheduled(SB_CONFIG::dynode* dnode, int config=0) {
      return  
    }*/
    
    void setNConfigs(int n) {_n_configs=n;}

    void addForward(int config1, int port1, int config2, int port2) {
       _forwardMap[std::make_pair(config1,port1)]=std::make_pair(config2,port2); 
    }
    
    int numWidePorts() { return _wide_ports.size(); }
    std::vector<int>& widePort(int i) {return _wide_ports[i];}
    void addWidePort(std::vector<int>& port) {_wide_ports.push_back(port);}
    
    typedef std::map< std::pair<int,int>,std::pair<int,int> >::iterator forward_iterator;
    forward_iterator fbegin() {return _forwardMap.begin();} 
    forward_iterator fend() {return _forwardMap.end();}
    
    SB_CONFIG::DyModel* dyModel() {return _dyModel;}
    
    void calcLatency(int& lat,int& latmis);
    
    void calcAssignEdgeLink_single(DyPDG_Node* pdgnode);
    void calcAssignEdgeLink();
    
    typedef std::map<std::pair<SB_CONFIG::dynode*,int>,DyPDG_Node*>::iterator assign_node_iterator;
    typedef std::map<std::pair<SB_CONFIG::dylink*,int>,DyPDG_Node*>::iterator assign_link_iterator;
    typedef std::map<std::pair<SB_CONFIG::dylink*,int>,std::set<DyPDG_Edge*>>::iterator assign_edgelink_iterator;
    
    assign_node_iterator assign_node_begin() { return _assignNode.begin(); }
    assign_node_iterator assign_node_end() { return _assignNode.end(); }
    assign_link_iterator assign_link_begin() { return _assignLink.begin(); }
    assign_link_iterator assign_link_end() { return _assignLink.end(); }
    assign_edgelink_iterator assign_edgelink_begin() { return _assignEdgeLink.begin(); }
    assign_edgelink_iterator assign_edgelink_end() { return _assignEdgeLink.end(); }
    
    void clearAll() {
      _assignNode.clear();
      _dynodeOf.clear();
      _latOf.clear();
      _assignLink.clear();
      _linksOf.clear();
      _assignEdgeLink.clear();
      _forwardMap.clear();
    }

    void xfer_link_to_switch() {
      using namespace SB_CONFIG;
      using namespace std;
      if(_assignSwitch.size()!=0) { //switches already assigned!
        return;
      }
      int config=0;
      vector< vector<dyswitch> >& switches = _dyModel->subModel()->switches();
      for(int i = 0; i < _dyModel->subModel()->sizex()+1; ++i) {
        for(int j = 0; j < _dyModel->subModel()->sizey()+1; ++j) {
          dyswitch* dysw = &switches[i][j];
          
          dynode::const_iterator I,E;
          for(I=dysw->ibegin(), E=dysw->iend(); I!=E; ++I) {
            dylink* inlink = *I;
            if(_assignLink.count(make_pair(inlink,config))!=0) {
              DyPDG_Node* innode = _assignLink[make_pair(inlink,config)];
              
              dynode::const_iterator II,EE;
              for(II=dysw->obegin(), EE=dysw->oend(); II!=EE; ++II) {
                dylink* outlink = *II;
                
                if(_assignLink.count(make_pair(outlink,config))!=0 && innode==_assignLink[make_pair(outlink,config)]) {
                  _assignSwitch[dysw][outlink]=inlink;
                }
              }
            }
          }
        }
      }
    }


  private:
    int _n_configs;   
    SB_CONFIG::DyModel *_dyModel;
    DyPDG*   _dyPDG;
    std::map<std::pair<SB_CONFIG::dynode*,int>,DyPDG_Node*> _assignNode;
    std::map<DyPDG_Node*, std::pair<SB_CONFIG::dynode*,int> > _dynodeOf;
    std::map<DyPDG_Node*, int> _latOf; 
    std::map<std::pair<SB_CONFIG::dylink*,int>,DyPDG_Node*> _assignLink;
    std::map<DyPDG_Node*, std::vector<std::pair<SB_CONFIG::dylink*,int>> > _linksOf;
    std::map<std::pair<SB_CONFIG::dylink*,int>,std::set<DyPDG_Edge*>> _assignEdgeLink;
    std::map< std::pair<int,int>,std::pair<int,int> > _forwardMap;
    std::vector< std::vector<int> > _wide_ports;
    

    std::map<SB_CONFIG::dyswitch*,
             std::map<SB_CONFIG::dylink*,SB_CONFIG::dylink*>> _assignSwitch; //out to in
   
    

        //helper for Schedule(filename) constructor
    void tracePath(SB_CONFIG::dynode* ,DyPDG_Node* , 
       std::map<SB_CONFIG::dynode*, std::map<SB_CONFIG::DyDIR::DIR,SB_CONFIG::DyDIR::DIR> >&,
       std::map<SB_CONFIG::dynode*, DyPDG_Node* >&,
       std::map<DyPDG_Node*, std::vector<SB_CONFIG::DyDIR::DIR> >&);
  
    
};

#endif
