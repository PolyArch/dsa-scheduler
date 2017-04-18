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

#define MAX_ROUTE 100000000



template <typename T,typename U>                           
std::pair<T,U> operator+(const std::pair<T,U> & l,
                         const std::pair<T,U> & r) {   
    return {l.first+r.first,l.second+r.second};
}        

template <typename T,typename U>                           
std::pair<T,U> operator-(const std::pair<T,U> & l,
                         const std::pair<T,U> & r) {   
    return {l.first-r.first,l.second-r.second};
}        



class CandidateRouting {
	public:
  std::map< std::pair<SB_CONFIG::sblink*,int>, SbPDG_Edge* > routing;
  std::map< std::pair<int,int>,std::pair<int,int> > forwarding;

  void clear() {
    routing.clear();
    forwarding.clear();
  }
};

class Scheduler {
	public:
  Scheduler(SB_CONFIG::SbModel* sbModel) :_sbModel(sbModel) {}

  bool check_res(SbPDG* sbPDG,    SbModel* sbmodel);

  virtual bool schedule(SbPDG* sbPDG, Schedule*& schedule) = 0;

	void error(const char *msg){
  	perror(msg);
  	exit(0);
	}
	protected:
	SB_CONFIG::SbModel* getSBModel(){return _sbModel;} 
  SB_CONFIG::SbModel* _sbModel;
  
};

class HeuristicScheduler : public Scheduler {

	public:

	HeuristicScheduler(SB_CONFIG::SbModel* sbModel) : Scheduler(sbModel) {}
  virtual bool schedule(SbPDG* sbPDG, Schedule*& schedule) = 0;
  virtual bool scheduleNode(Schedule*, SbPDG_Node*) = 0;
  virtual std::pair<int,int> scheduleHere(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*, 
               int config, CandidateRouting&,std::pair<int,int> bestScore) = 0;
  virtual std::pair<int,int> route(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, int config, 
            CandidateRouting&,std::pair<int,int> scoreLeft) = 0;

 std::pair<int,int> route_minimizeDistance(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, int config, 
            CandidateRouting&,std::pair<int,int> scoreLeft);
 std::pair<int,int> route_minimizeOverlapping(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, int config, 
            CandidateRouting&,std::pair<int,int> scoreLeft);

	protected:

  bool assignVectorInputs(SbPDG*, Schedule*);
  bool assignVectorOutputs(SbPDG*, Schedule*);

  void applyRouting(Schedule*, int, CandidateRouting*);
  void applyRouting(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*, int, CandidateRouting*);
  
  void fillInputSpots(Schedule*,SbPDG_Input*, int config, 
                    std::vector<SB_CONFIG::sbnode*>& spots);
  void fillOutputSpots(Schedule*,SbPDG_Output*, int config, 
                    std::vector<SB_CONFIG::sbnode*>& spots);
  void fillInstSpots(Schedule*,SbPDG_Inst*, int config, 
                     std::vector<SB_CONFIG::sbnode*>& spots);

   int route_to_output(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, int config, 
            CandidateRouting&, int scoreLeft);
 
};

class GamsScheduler : public Scheduler {
	public:
	GamsScheduler(SB_CONFIG::SbModel* sbModel) :
	Scheduler(sbModel),
  _gams_files_setup(false), _use_server(false),
  _gams_work_dir("gams"),
  _optcr(0.1f), _optca(0.0f), _reslim(100000.0f), _showGams(true) { }

  virtual bool schedule(SbPDG* sbPDG, Schedule*& schedule) = 0;
  bool requestGams(const char *filename);

  void showGams(bool show) {
    _showGams=show; 
  }
  void use_server() { _use_server = true; }
	void setGap(float relative, float absolute=1.0f) {
    _optcr=relative;
    _optca=absolute;
  }

  void setTimeout(float timeout) {
    _reslim=timeout;
  }

  protected:

  bool _gams_files_setup;
  bool _use_server;
  std::string _gams_work_dir;
  float _optcr,_optca,_reslim;
  bool _showGams;

	std::unordered_map<std::string, std::pair<SB_CONFIG::sbnode*,int> >  gamsToSbnode;  
  std::unordered_map<std::string, std::pair<SB_CONFIG::sblink*,int> > gamsToSblink;
  std::unordered_map<std::string, std::pair<SB_CONFIG::sbswitch*,int>>  gamsToSbswitch;


  std::unordered_map<std::string, SbPDG_Node*> gamsToPdgnode;
  std::unordered_map<std::string, SbPDG_Edge*> gamsToPdgedge;
  std::unordered_map<std::string, std::pair<bool,int> >  gamsToPortN;  
  std::unordered_map<std::string, SbPDG_Vec*>  gamsToPortV;  

};
#endif
