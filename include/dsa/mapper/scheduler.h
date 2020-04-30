#ifndef __SS_SCHEDULER_H__
#define __SS_SCHEDULER_H__

#include <stdlib.h>

#include <boost/functional.hpp>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "dsa/mapper/schedule.h"
#include "dsa/arch/model.h"
#include "dsa/ir/ssdfg.h"

#define MAX_ROUTE 100000000

using usec = std::chrono::microseconds;
using get_time = std::chrono::steady_clock;

int rand_bt(int s, int e);
int rand_bt_large(int s, int e);

std::string basename(const std::string& filename);

template <typename T, typename U>
std::pair<T, U> operator+(const std::pair<T, U>& l, const std::pair<T, U>& r) {
  return {l.first + r.first, l.second + r.second};
}

template <typename T, typename U>
std::pair<T, U> operator-(const std::pair<T, U>& l, const std::pair<T, U>& r) {
  return {l.first - r.first, l.second - r.second};
}

class CodesignInstance;

class Scheduler {
 public:
  Scheduler(dsa::SSModel* ssModel, double timelimit = 100000, bool verbose_ = false)
      : _ssModel(ssModel), _reslim(timelimit), verbose(verbose_) {}

  bool check_feasible(SSDfg* ssDFG, SSModel* ssmodel, bool verbose);

  bool vport_feasible(SSDfg* ssDFG, SSModel* ssmodel, bool verbose);

  virtual bool schedule(SSDfg* ssDFG, Schedule*& schedule) = 0;

  virtual bool incrementalSchedule(CodesignInstance& incr_table) {
    assert(0 && "not supported");
  }

  bool suppress_timing_print = false;

  std::string AUX(int x) { return (x == -1 ? "-" : std::to_string(x)); }

  double total_msec() {
    auto end = get_time::now();
    auto diff = end - _start;
    return ((double)std::chrono::duration_cast<usec>(diff).count()) / 1000.0;
  }

  void set_start_time() { _start = get_time::now(); }

  virtual bool schedule_timed(SSDfg* ssDFG, Schedule*& sched) {
    set_start_time();

    bool succeed_sched = schedule(ssDFG, sched);

    if (verbose && !suppress_timing_print) {
      printf("sched_time: %0.4f seconds\n", total_msec() / 1000.0);
    }

    return succeed_sched;
  }

  void setTimeout(float timeout) { _reslim = timeout; }

  bool running() { return !_should_stop; }
  void stop() { _should_stop = true; }

  Schedule* invoke(SSModel* model, SSDfg* dfg, bool);

 protected:
  dsa::SSModel* getSSModel() { return _ssModel; }

  dsa::SSModel* _ssModel;

  float _reslim;
  bool verbose{false};
  bool _should_stop{false};
  std::string mapping_file{""};
  bool dump_mapping_if_improved{false};

  std::chrono::time_point<std::chrono::steady_clock> _start;
};

void make_directories(const std::string& s);

#endif
