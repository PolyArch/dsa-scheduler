#ifndef __SB_SCHEDULER_H__
#define __SB_SCHEDULER_H__

#include "sbpdg.h"
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
 std::map< std::pair<SB_CONFIG::sblink*,int>, SbPDG_Edge* > routing;
 std::map< std::pair<int,int>,std::pair<int,int> > forwarding;
};

class Scheduler {
  public:

  Scheduler(SB_CONFIG::SbModel* sbModel) :
  _gams_files_setup(false), _use_server(false),
    _gams_work_dir("gams"), _sbModel(sbModel),
    _optcr(0.1f), _optca(0.0f), _reslim(100000.0f), _showGams(true) { }

  bool scheduleGAMS(SbPDG* sbPDG, Schedule*& schedule);         //GAMs sepcific scheduler (sbpdg and scheulde object)
  
  Schedule* scheduleGreedyBFS(SbPDG* sbPDG);
  
  

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
  
  void applyRouting(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*, int, CandidateRouting*);
  
  void fillInputSpots(Schedule*,SbPDG_Input*, int config, 
                    std::vector<SB_CONFIG::sbnode*>& spots);
  void fillOutputSpots(Schedule*,SbPDG_Output*, int config, 
                    std::vector<SB_CONFIG::sbnode*>& spots);
  void fillInstSpots(Schedule*,SbPDG_Inst*, int config, 
                     std::vector<SB_CONFIG::sbnode*>& spots);
  void scheduleNode(Schedule*, SbPDG_Node* );
  int scheduleHere(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*, 
               int config, CandidateRouting&, int bestScore);
  int route(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, int config, 
            CandidateRouting&, int scoreLeft);
   int route_to_output(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, int config, 
            CandidateRouting&, int scoreLeft);
  
  bool _gams_files_setup;
  bool _use_server;
  std::string _gams_work_dir;
  SB_CONFIG::SbModel* _sbModel;
  float _optcr,_optca,_reslim;
  bool _showGams;
  
  //std::unordered_map<std::string,SB_CONFIG::sbnode*> gamsToSbnode;  
  //std::unordered_map<std::string,SbPDG_Node*> gamsToPdgnode;
  
  //typedef SchedulerMap std::unordered_map;

  std::unordered_map<std::string, std::pair<SB_CONFIG::sbnode*,int> >  gamsToSbnode;  
  std::unordered_map<std::string, std::pair<SB_CONFIG::sblink*,int> > gamsToSblink;
  std::unordered_map<std::string, std::pair<SB_CONFIG::sbswitch*,int>>  gamsToSbswitch;


  std::unordered_map<std::string, SbPDG_Node*> gamsToPdgnode;
  std::unordered_map<std::string, SbPDG_Edge*> gamsToPdgedge;
  std::unordered_map<std::string, std::pair<bool,int> >  gamsToPortN;  
  std::unordered_map<std::string, SbPDG_Vec*>  gamsToPortV;  

};



#endif
