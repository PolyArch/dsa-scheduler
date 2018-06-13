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
#include <chrono>

#define MAX_ROUTE 100000000

# include <chrono>
using usec = std::chrono::microseconds;
using get_time = std::chrono::steady_clock;


void System(const char* command);


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
  struct EdgeProp {
    int num_links=0;
    int num_passthroughs=0; 
    std::unordered_set<sblink*> links;
  };

  std::unordered_map< SB_CONFIG::sblink*, std::unordered_set<SbPDG_Edge*> > routing;
  std::map< std::pair<int,int>,std::pair<int,int> > forwarding;

  std::unordered_map< SbPDG_Edge*, EdgeProp> edge_prop;

  void fill_lat(Schedule* sched,
                int& min_node_lat, int& max_node_lat, bool print=false) {
    min_node_lat = 0; //need minimax, so that's why this is odd
    max_node_lat = MAX_ROUTE;
    SbPDG_Node* n = (*edge_prop.begin()).first->use();
    bool output = dynamic_cast<SbPDG_Output*>(n);

    for(auto I=edge_prop.begin(), E=edge_prop.end();I!=E;++I) {
      SbPDG_Edge* source_pdgedge = (*I).first;
      auto i = edge_prop[source_pdgedge];
      int num_links = i.num_links;
      int num_passthroughs = i.num_passthroughs;
  
      auto p = sched->lat_bounds(source_pdgedge->def());

      int min_inc_lat = p.first + num_links; 
      int max_inc_lat = p.second + num_links +
              sched->sbModel()->maxEdgeDelay() * ((!output)+num_passthroughs);

    if(print) {
      std::cout << "  links: " << num_links << " pts: " << num_passthroughs << "\n";
      std::cout << "  b low: " << p.first << " pts: " << p.second << "\n";
      std::cout << "  max_extra:" << sched->sbModel()->maxEdgeDelay() * ((!output)+num_passthroughs) << "\n";
    }


      //earliest starting time is *latest* incomming edge
      if(min_inc_lat > min_node_lat) {
        min_node_lat = min_inc_lat;
      }
      //latest starting time is *earliest* incomming edge
      if(max_inc_lat < max_node_lat) {
        max_node_lat = max_inc_lat;
      }
    }
    if(print) {
      std::cout << "  min_inc_lat" << min_node_lat << " " << max_node_lat << "\n";
    }

  }


  void clear() {
    routing.clear();
    forwarding.clear();
    edge_prop.clear();
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
  Scheduler(SB_CONFIG::SbModel* sbModel) :_sbModel(sbModel),
  _optcr(0.1f), _optca(0.0f), _reslim(100000.0f)  {}

  bool check_res(SbPDG* sbPDG,    SbModel* sbmodel);

  virtual bool schedule(SbPDG* sbPDG, Schedule*& schedule) = 0;

  int numFASched;
  int numInputSched;
  int numOutputSched;

  int bestFASched;
  int bestInputSched;
  int bestOutputSched;

  bool verbose;
  bool suppress_timing_print=false;
  void set_max_iters(int i) {_max_iters=i;}

  std::string str_subalg;
  std::string AUX(int x) {
    return (x==-1 ? "-" : std::to_string(x));
  }  

  double total_msec() {
    auto end = get_time::now();
    auto diff = end - _start;
    return ((double)std::chrono::duration_cast<usec>(diff).count())/1000.0;
  }

  virtual bool schedule_timed(SbPDG* sbPDG, Schedule*& sched) {
    _start = get_time::now();

    bool succeed_sched = schedule(sbPDG,sched);

    if(verbose && ! suppress_timing_print) {
      printf("sched_time: %0.4f seconds\n", total_msec()/1000.0);
    }

    return succeed_sched;
  }

  void setGap(float relative, float absolute=1.0f) {          
    _optcr=relative;
    _optca=absolute;
  }

  void setTimeout(float timeout) {_reslim=timeout;}

  //virtual void unroute(Schedule* sched, SbPDG_Edge* pdgnode, 
  //                     SB_CONFIG::sbnode* source);

  protected:
  SB_CONFIG::SbModel* getSBModel(){return _sbModel;} 
  SB_CONFIG::SbModel* _sbModel;
 
  int _max_iters = 20000;

  float _optcr,_optca,_reslim;
  std::chrono::time_point<std::chrono::steady_clock> _start;
};




class HeuristicScheduler : public Scheduler { 
public:

  HeuristicScheduler(SB_CONFIG::SbModel* sbModel) : Scheduler(sbModel), 
  fscore(std::make_pair(MAX_ROUTE,MAX_ROUTE)) {}
  
  virtual bool scheduleNode(Schedule*, SbPDG_Node*) = 0;
  virtual std::pair<int,int> scheduleHere(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*, 
               CandidateRouting&,std::pair<int,int> bestScore) = 0;
  virtual std::pair<int,int> route(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, 
            CandidateRouting&,std::pair<int,int> scoreLeft) = 0;

  virtual int routing_cost(SbPDG_Edge*, sblink*, Schedule*, CandidateRouting&, sbnode* dest);

  std::pair<int,int> route_minimizeDistance(Schedule* sched, SbPDG_Edge* pdgnode,
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

  const std::pair<int,int> fscore;
};

#endif
