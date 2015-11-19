#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "dypdg.h"
#include "model.h"
#include "schedule.h"

#include <map>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <map>


class CandidateRouting {
public:
 std::map< std::pair<SB_CONFIG::dylink*,int>,DyPDG_Edge* > routing;
 std::map< std::pair<int,int>,std::pair<int,int> > forwarding;
};

class Scheduler {
  public:

  Scheduler(SB_CONFIG::DyModel* dyModel) :
  _gams_files_setup(false), _use_server(false),
    _gams_work_dir("gams"), _dyModel(dyModel),
    _optcr(0.1f), _optca(0.0f), _reslim(100000.0f), _showGams(true) { }

  bool scheduleGAMS(DyPDG* dyPDG, Schedule*& schedule);
  
  Schedule* scheduleGreedyBFS(DyPDG* dyPDG);
  
  

  void use_server() { _use_server = true; }
  static bool requestGams(const char *filename);
  
  void setGap(float relative, float absolute=1.0f) {
    _optcr=relative;
    _optca=absolute;
  }

  void setTimeout(float timeout) {
    _reslim=timeout;
  }
  
  void showGams(bool show) {
    _showGams=show; 
  }
  
  private:
  
  void applyRouting(Schedule*, DyPDG_Node*, SB_CONFIG::dynode*, int,CandidateRouting*);
  void fillInputSpots(Schedule*,DyPDG_Input*, int config, 
                    std::vector<SB_CONFIG::dynode*>& spots);
  void fillOutputSpots(Schedule*,DyPDG_Output*, int config, 
                    std::vector<SB_CONFIG::dynode*>& spots);
  void fillInstSpots(Schedule*,DyPDG_Inst*, int config, 
                     std::vector<SB_CONFIG::dynode*>& spots);
  void scheduleNode(Schedule*, DyPDG_Node* );
  int scheduleHere(Schedule*, DyPDG_Node*, SB_CONFIG::dynode*, 
               int config, CandidateRouting&, int bestScore);
  int route(Schedule* sched, DyPDG_Edge* pdgnode,
            SB_CONFIG::dynode* source, SB_CONFIG::dynode* dest, int config, 
            CandidateRouting&, int scoreLeft);
   int route_to_output(Schedule* sched, DyPDG_Edge* pdgnode,
            SB_CONFIG::dynode* source, int config, 
            CandidateRouting&, int scoreLeft);
  
  bool _gams_files_setup;
  bool _use_server;
  std::string _gams_work_dir;
  SB_CONFIG::DyModel* _dyModel;
  float _optcr,_optca,_reslim;
  bool _showGams;
  //std::unordered_map<std::string,SB_CONFIG::dynode*> gamsToDynode;  
  //std::unordered_map<std::string,DyPDG_Node*> gamsToPdgnode;
  
  //typedef SchedulerMap std::unordered_map;

  std::unordered_map<std::string,std::pair<SB_CONFIG::dynode*,int> >  gamsToDynode;  
  std::unordered_map<std::string,std::pair<SB_CONFIG::dylink*,int> > gamsToDylink;
  std::unordered_map<std::string,std::pair<SB_CONFIG::dyswitch*,int>>  gamsToDyswitch;
  std::unordered_map<std::string,DyPDG_Node*> gamsToPdgnode;
  std::unordered_map<std::string,DyPDG_Edge*> gamsToPdgedge;
};










#endif
