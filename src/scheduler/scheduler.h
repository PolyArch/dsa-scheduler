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
#include <random>
#include <stdlib.h>
#include <boost/functional.hpp>

#define MAX_ROUTE 100000000

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
    int num_links = 0;
    int num_passthroughs = 0;
    std::unordered_set<sblink *> links;
  };


  void take_union(CandidateRouting &r) {
    for (auto &link_iter : r.routing) {
      for (SbPDG_Edge *edge : link_iter.second) {
        routing[link_iter.first].insert(edge);
      }
    }
    for (auto &edge_iter : r.edge_prop) {
      SbPDG_Edge *edge = edge_iter.first;
      assert(edge_prop.count(edge) == 0);
      edge_prop[edge] = edge_iter.second;
    }
  }

  std::unordered_map<std::pair<int, SB_CONFIG::sblink*>, std::unordered_set<SbPDG_Edge*>, boost::hash<std::pair<int, SB_CONFIG::sblink*>>> routing;
  std::unordered_map<SbPDG_Edge *, EdgeProp> edge_prop;

  void fill_lat(Schedule *sched,
                int &min_node_lat, int &max_node_lat, bool print = false) {
    min_node_lat = 0; //need minimax, so that's why this is odd
    max_node_lat = MAX_ROUTE;

    if (edge_prop.empty())
      return;

    SbPDG_Node *n = (*edge_prop.begin()).first->use();
    bool output = dynamic_cast<SbPDG_Output *>(n);

    for (auto I = edge_prop.begin(), E = edge_prop.end(); I != E; ++I) {
      SbPDG_Edge *source_pdgedge = (*I).first;
      auto i = edge_prop[source_pdgedge];
      int num_links = i.num_links;
      int num_passthroughs = i.num_passthroughs;

      auto p = sched->lat_bounds(source_pdgedge->def());

      int min_inc_lat = p.first + num_links;
      int max_inc_lat = p.second + num_links +
                        sched->sbModel()->maxEdgeDelay() * ((!output) + num_passthroughs);

      if (print) {
        std::cout << "  links: " << num_links << " pts: " << num_passthroughs << "\n";
        std::cout << "  b low: " << p.first << " pts: " << p.second << "\n";
        std::cout << "  max_extra:" << sched->sbModel()->maxEdgeDelay() * ((!output) + num_passthroughs) << "\n";
      }

      if (min_inc_lat > min_node_lat) min_node_lat = min_inc_lat;
      if (max_inc_lat < max_node_lat) max_node_lat = max_inc_lat;
    }
    if (print) {
      std::cout << "  min_inc_lat" << min_node_lat << " " << max_node_lat << "\n";
    }

  }


  void clear() {
    routing.clear();
    edge_prop.clear();
  }
};


class Scheduler {
public:
  Scheduler(SB_CONFIG::SbModel *sbModel) : _sbModel(sbModel),
                                           _optcr(0.1f), _optca(0.0f), _reslim(100000.0f) {}

  bool check_res(SbPDG *sbPDG, SbModel *sbmodel);

  virtual bool schedule(SbPDG *sbPDG, Schedule *&schedule) = 0;

  int numFASched;
  int numInputSched;
  int numOutputSched;

  int bestFASched;
  int bestInputSched;
  int bestOutputSched;

  bool verbose;
  bool suppress_timing_print = false;

  void set_max_iters(int i) { _max_iters = i; }

  std::string str_subalg;

  std::string AUX(int x) {
    return (x == -1 ? "-" : std::to_string(x));
  }

  double total_msec() {
    auto end = get_time::now();
    auto diff = end - _start;
    return ((double) std::chrono::duration_cast<usec>(diff).count()) / 1000.0;
  }

  virtual bool schedule_timed(SbPDG *sbPDG, Schedule *&sched) {
    _start = get_time::now();

    bool succeed_sched = schedule(sbPDG, sched);

    if (verbose && !suppress_timing_print) {
      printf("sched_time: %0.4f seconds\n", total_msec() / 1000.0);
    }

    return succeed_sched;
  }

  void setGap(float relative, float absolute = 1.0f) {
    _optcr = relative;
    _optca = absolute;
  }

  void setTimeout(float timeout) { _reslim = timeout; }

  //virtual void unroute(Schedule* sched, SbPDG_Edge* pdgnode, 
  //                     SB_CONFIG::sbnode* source);

protected:
  SB_CONFIG::SbModel *getSBModel() { return _sbModel; }

  SB_CONFIG::SbModel *_sbModel;

  int _max_iters = 20000;

  float _optcr, _optca, _reslim;
  std::chrono::time_point<std::chrono::steady_clock> _start;
};




class HeuristicScheduler : public Scheduler {
public:

  HeuristicScheduler(SB_CONFIG::SbModel *sbModel) : Scheduler(sbModel),
                                                    fscore(std::make_pair(MAX_ROUTE, MAX_ROUTE)) {}

  virtual bool scheduleNode(Schedule *, SbPDG_Node *) = 0;

  virtual std::pair<int, int> scheduleHere(Schedule *, SbPDG_Node *, std::pair<int, SB_CONFIG::sbnode *>,
                                           CandidateRouting &) = 0;

  virtual std::pair<int, int> route(Schedule *sched, SbPDG_Edge *pdgnode,
                                    std::pair<int, SB_CONFIG::sbnode *> source, std::pair<int, SB_CONFIG::sbnode *> dest,
                                    CandidateRouting &) = 0;

  virtual int routing_cost(SbPDG_Edge *, int, int, sblink *, Schedule *, CandidateRouting &, const std::pair<int, sbnode *> &);

  std::pair<int, int> route_minimize_distance(Schedule *sched, SbPDG_Edge *pdgnode,
                                              std::pair<int, SB_CONFIG::sbnode*> source,
                                              std::pair<int, SB_CONFIG::sbnode*> dest,
                                              CandidateRouting &);

protected:
  bool assignVectorInputs(SbPDG *, Schedule *);

  bool assignVectorOutputs(SbPDG *, Schedule *);

  void apply_routing(Schedule *, CandidateRouting *);

  void apply_routing(Schedule *, SbPDG_Node *, std::pair<int, SB_CONFIG::sbnode *>, CandidateRouting *);

  std::vector<std::pair<int, sbnode *>> fill_input_spots(Schedule *, SbPDG_Input *);

  std::vector<std::pair<int, sbnode *>> fill_output_spots(Schedule *, SbPDG_Output *);

  // Find all the candidate spots for the given instruction.
  std::vector<std::pair<int, SB_CONFIG::sbnode *>> fill_inst_spots(Schedule *, SbPDG_Inst *);

  const std::pair<int, int> fscore;

  void random_order(int n, std::vector<int> &order);

  std::vector<bool> rand_node_choose_k(int k,
                                       std::vector<sbnode *> &input_nodes,
                                       std::vector<sbnode *> &output_nodes);

  void rand_n_choose_k(int n, int m, std::vector<int> &indices);

  int rand_bt(int s, int e) {
    return rand() % (e - s) + s;
  }

  int _route_times = 0;
};

#endif
