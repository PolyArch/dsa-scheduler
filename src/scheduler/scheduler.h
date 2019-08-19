#ifndef __SS_SCHEDULER_H__
#define __SS_SCHEDULER_H__

#include "ssdfg.h"
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
#include <memory>
#include <boost/functional.hpp>

#define MAX_ROUTE 100000000

using usec = std::chrono::microseconds;
using get_time = std::chrono::steady_clock;


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

class Scheduler {
public:
  Scheduler(SS_CONFIG::SSModel *ssModel) : _ssModel(ssModel),
                                           _optcr(0.1f), _optca(0.0f), _reslim(100000.0f) {}

  bool check_res(SSDfg *ssDFG, SSModel *ssmodel);

  virtual bool schedule(SSDfg *ssDFG, Schedule *&schedule) = 0;

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

  virtual bool schedule_timed(SSDfg *ssDFG, Schedule *&sched) {
    _start = get_time::now();

    bool succeed_sched = schedule(ssDFG, sched);

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

  //virtual void unroute(Schedule* sched, SSDfgEdge* dfgnode,
  //                     SS_CONFIG::ssnode* source);

  bool running() {return !_should_stop;}
  void stop() {_should_stop=true;}

  void set_srand(int i) {_srand=i;}

protected:
  SS_CONFIG::SSModel *getSSModel() { return _ssModel; }

  SS_CONFIG::SSModel *_ssModel;

  int _max_iters = 20000;
  bool _should_stop = false;
  int _srand=0;

  float _optcr, _optca, _reslim;
  std::chrono::time_point<std::chrono::steady_clock> _start;

  std::shared_ptr<Schedule*> best, current;
};




class HeuristicScheduler : public Scheduler {
public:

  HeuristicScheduler(SS_CONFIG::SSModel *ssModel) : Scheduler(ssModel),
                                                    fscore(std::make_pair(MAX_ROUTE, MAX_ROUTE)) {}

protected:

  const std::pair<int, int> fscore;

  void random_order(int n, std::vector<int> &order);

  std::vector<bool> rand_node_choose_k(int k,
                                       std::vector<ssnode *> &input_nodes,
                                       std::vector<ssnode *> &output_nodes);

  void rand_n_choose_k(int n, int m, std::vector<int> &indices);

  int rand_bt(int s, int e) {
    return rand() % (e - s) + s;
  }

  int _route_times = 0;
};

#endif
