#pragma once

#include <stdlib.h>

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

#include "dsa/arch/model.h"
#include "dsa/core/singleton.h"
#include "dsa/dfg/ssdfg.h"
#include "dsa/mapper/schedule.h"

#define MAX_ROUTE 100000000

using usec = std::chrono::microseconds;
using get_time = std::chrono::steady_clock;

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
  Scheduler(dsa::SSModel* ssModel) : _ssModel(ssModel) {}

  bool check_feasible(SSDfg* ssDFG, SSModel* ssmodel);

  virtual bool schedule(SSDfg* ssDFG, Schedule*& schedule, int stopping=-1) = 0;

  virtual bool incrementalSchedule(CodesignInstance& incr_table, int stopping=-1) {
    DSA_CHECK(0) << "not supported";
    return false;
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
    bool verbose = dsa::ContextFlags::Global().verbose;

    if (verbose && !suppress_timing_print) {
      printf("sched_time: %0.4f seconds\n", total_msec() / 1000.0);
    }

    return succeed_sched;
  }

  bool running() { return !_should_stop; }
  void stop() { _should_stop = true; }

  /*!
   * \return 0: done, 1: count failed, 2: routing failed
   */
  int invoke(SSModel* model, SSDfg* dfg);

 protected:
  dsa::SSModel* getSSModel() { return _ssModel; }

  dsa::SSModel* _ssModel;

  bool _should_stop{false};
  std::string mapping_file{""};
  bool dump_mapping_if_improved{false};

  std::chrono::time_point<std::chrono::steady_clock> _start;
};

void make_directories(const std::string& s);
