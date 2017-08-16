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
  std::unordered_map< SB_CONFIG::sblink*, SbPDG_Edge* > routing;
  std::map< std::pair<int,int>,std::pair<int,int> > forwarding;

  void clear() {
    routing.clear();
    forwarding.clear();
  }
};

class Triplet {
  public: 
  Triplet(int x, int y, int z) : _x(x), _y(y), _z(z) {}
    

  private:
  int _x, _y, _z;
};

class Scheduler {
  public:
  Scheduler(SB_CONFIG::SbModel* sbModel) :_sbModel(sbModel) {}

  bool check_res(SbPDG* sbPDG,    SbModel* sbmodel);

  virtual bool schedule(SbPDG* sbPDG, Schedule*& schedule) = 0;

  int numFASched;
  int numInputSched;
  int numOutputSched;

  int bestFASched;
  int bestInputSched;
  int bestOutputSched;

  bool verbose;

  enum StatType {FA, Input, Output};

  void progress_updateCurNum(StatType s, int init = 0){
    if (s == FA) {
      numFASched = init;
    } else if (s == Input) {
      numInputSched = init;
    } else if (s == Output) {
      numOutputSched = init;
    } else {
      std::cout<<"Bad Stat Type\n";
      exit(0);
    }
  }

  
  void progress_updateBestNum(StatType s, int init = 0){
    if (s == FA) {
      bestFASched = init;
    } else if (s == Input) {
      bestInputSched = init;
    } else if (s == Output) {
      bestOutputSched = init;
    } else {
      std::cout<<"Bad Stat Type\n";
      exit(0);
    }
  }

  void progress_saveBestNum(StatType s) {
    int progress_cur = progress_getCurNum(s);
    int progress_best =  progress_getBestNum(s);
    if ( progress_cur > progress_best) {
      progress_updateBestNum(s, progress_cur);
    }
  }

  void progress_incCurNum(StatType s) {
    if (s == FA) {
      numFASched++;
    } else if (s == Input) {
      numInputSched++;
    } else if (s == Output) {
      numOutputSched++;
    } else {
      std::cout<<"Bad Stat Type\n";
      exit(0);
    }
  }

  int progress_getCurNum(StatType s) {
    int ret = 0;
    if (s == FA) {
      ret = numFASched;
    } else if (s == Input) {
      ret = numInputSched;
    } else if (s == Output) {
      ret = numOutputSched;
    } else {
      std::cout<<"Bad Stat Type\n";
      exit(0);
    }
    return ret;
  }

  int progress_getBestNum(StatType s) {
    int ret = 0;
    if (s == FA) {
      ret = bestFASched;
    } else if (s == Input) {
      ret = bestInputSched;
    } else if (s == Output) {
      ret = bestOutputSched;
    } else {
      std::cout<<"Bad Stat Type\n";
      exit(0);
    }
    return ret;
  }


  void progress_initCurNums(){
    progress_updateCurNum(FA,0);
    progress_updateCurNum(Input,0);
    progress_updateCurNum(Output,0);
  }

  void progress_initBestNums(){
    progress_updateBestNum(FA,-1);
    progress_updateBestNum(Input,-1);
    progress_updateBestNum(Output,-1);
  }


  std::string AUX(int x) {
    return (x==-1 ? "-" : std::to_string(x));
  }  

  void progress_printBests(){
    std::cout<<"Progress: ("<<AUX(bestInputSched)<<", "<<AUX(bestFASched)<<", "<<AUX(bestOutputSched)<<")\n";
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
               CandidateRouting&,std::pair<int,int> bestScore) = 0;
  virtual std::pair<int,int> route(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, 
            CandidateRouting&,std::pair<int,int> scoreLeft) = 0;

 std::pair<int,int> route_minimizeDistance(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, 
            CandidateRouting&,std::pair<int,int> scoreLeft);
 std::pair<int,int> route_minimizeOverlapping(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, 
            CandidateRouting&,std::pair<int,int> scoreLeft);

protected:

  bool assignVectorInputs(SbPDG*, Schedule*);
  bool assignVectorOutputs(SbPDG*, Schedule*);

  void applyRouting(Schedule*, CandidateRouting*);
  void applyRouting(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*, CandidateRouting*);
  
  void fillInputSpots(Schedule*,SbPDG_Input*,
                    std::vector<SB_CONFIG::sbnode*>& spots);
  void fillOutputSpots(Schedule*,SbPDG_Output*, 
                    std::vector<SB_CONFIG::sbnode*>& spots);
  void fillInstSpots(Schedule*,SbPDG_Inst*,
                     std::vector<SB_CONFIG::sbnode*>& spots);

   int route_to_output(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source,
            CandidateRouting&, int scoreLeft);
 
};

#endif
