#ifndef __SS__SCHEDULE_SIMULATEDANNEALING_H__
#define __SS__SCHEDULE_SIMULATEDANNEALING_H__

#include "scheduler.h"
#include <iostream>

#define DEBUG_SCHED (false)

class SchedulerSimulatedAnnealing : public HeuristicScheduler {
public:
  void initialize(SSDfg *, Schedule *&);

  SchedulerSimulatedAnnealing(SS_CONFIG::SSModel *ssModel) :
          HeuristicScheduler(ssModel) {}

  bool schedule(SSDfg *, Schedule *&) override;

  void set_fake_it() { _fake_it = true; }

  bool schedule_internal(SSDfg *ssDFG, Schedule *&sched);

protected:
  std::pair<int, int> obj(Schedule*& sched, int& lat, 
      int& lat_mis, int& ovr, int& agg_ovr, int& max_util); 

  bool scheduleNode(Schedule *sched, SSDfgInst *dfgnode) override;

  template<typename T>
  bool scheduleHere(Schedule *sched, const std::vector<T> &nodes,
                    const std::vector<std::pair<int, SS_CONFIG::ssnode*>> &slots,
                    std::pair<int, int> &score) {
    assert(slots.size() == nodes.size());
    for (int i = 0; i < (int) nodes.size(); ++i) {
      if (!scheduleHere(sched, nodes[i], slots[i], score)) {
        for (int j = 0; j < i; ++j) {
          sched->unassign_dfgnode(nodes[j]);
        }
        return false;
      }
    }
    return true;
  }
  // OK, I now have these four functions to rewrite, after that is done, I will kill
  // the candidate routing version.
  bool scheduleHere(Schedule *, SSDfgNode *, std::pair<int, SS_CONFIG::ssnode *>,
                    std::pair<int, int> &score);
  bool route(Schedule *sched, SSDfgEdge *dfgnode,
             std::pair<int, SS_CONFIG::ssnode *> source,
             std::pair<int, SS_CONFIG::ssnode *> dest,
             std::pair<int, int> &score);
  bool route_minimize_distance(Schedule *sched, SSDfgEdge *dfgnode,
                               std::pair<int, SS_CONFIG::ssnode *> source,
                               std::pair<int, SS_CONFIG::ssnode *> dest,
                               std::pair<int, int> &score);
  int routing_cost(SSDfgEdge *, int, int, sslink *, Schedule *, const std::pair<int, ssnode *> &);

  bool timingIsStillGood(Schedule *sched);

  bool map_to_completion(SSDfg *ssDFG, Schedule *sched);

  bool map_io_to_completion(SSDfg *ssDFG, Schedule *sched);

  template<typename T> inline bool schedule_it(T *node, SSDfg *ssDFG, Schedule *sched);
  template<typename T> inline bool schedule_io(T *vec, SSDfg *ssDFG, Schedule *sched);
  template<typename T> inline bool map_one(SSDfg *ssDFG, Schedule *sched) {
    auto &nodes = ssDFG->nodes<T*>();
    int n = nodes.size();
    int p = rand() % n;
    for(int i = 0; i < n; ++i) {
      p = (p + 1) % n;
      T* to_map = nodes[p];
      if (!sched->is_scheduled(to_map))
        return schedule_it<T>(to_map, ssDFG, sched);
    }
    return false;
  }

  template<typename T> inline void unmap_one(SSDfg *ssDFG, Schedule *sched);
  void unmap_some(SSDfg *ssDFG, Schedule *sched);


  std::vector<std::pair<int, int>> _sd_in;  //port, length pair
  std::vector<std::pair<int, int>> _sd_out; //port, length pair

  std::vector<std::pair<int, int>> &sd(bool is_input) {
    return is_input ? _sd_in : _sd_out;
  }

  int _max_iters_zero_vio = 1000000000;
  bool _integrate_timing = true;
  int _best_latmis, _best_lat, _best_violation;
  bool _strict_timing = true;

  bool _fake_it = false;

};

template<typename T>
bool SchedulerSimulatedAnnealing::schedule_io(T* vec, SSDfg* ssDFG, Schedule* sched) {
  using std::vector;
  using std::pair;
  using std::make_pair;

  SS_CONFIG::SubModel *subModel = _ssModel->subModel();
  ssio_interface &si = subModel->io_interf();

  vector<bool> bestMask;
  pair<int, int> bestScore = std::make_pair(INT_MIN, INT_MIN);
  int best_candidate = -1;
  pair<bool, int> bestVportid;

  std::vector<pair<int, int>> candidates = vec->candidates(sched, _ssModel, 0);

  for (size_t i = 0; i < candidates.size(); ++i) {

    int id = candidates[i].first;
    int mask = candidates[i].second;

    std::pair<int, int> score;
    if (scheduleHere(sched, vec->vector(), vec->ready_to_map(_ssModel, candidates[i]), score)) {
      int lat = INT_MAX, latmis = INT_MAX, ovr = INT_MAX,  agg_ovr = INT_MAX, max_util = INT_MAX;
      pair<int, int> candScore = obj(sched, lat, latmis, ovr, agg_ovr, max_util);
      if (candScore > bestScore) {
        best_candidate = i;
        bestScore = candScore;

        pair<bool, int> vport_id(T::IsInput(), si.vports_vec[T::IsInput()][id].first);
        vector<int> &vport_desc = si.vports_vec[T::IsInput()][id].second->port_vec();
        bestMask.resize(vport_desc.size(), false);
        for (int m = 0; m < (int) vport_desc.size(); m++) {
          bestMask[m] = mask >> m & 1;
        }
        bestVportid = vport_id;

      }
      sched->unassign_vec(vec);
    }

  }

  if (best_candidate != -1) {
    pair<int, int> score;
    scheduleHere(sched, vec->vector(), vec->ready_to_map(_ssModel, candidates[best_candidate]), score);
    sched->assign_vport(vec, bestVportid, bestMask);
    return true;
  }

  return false;
}

template<> inline bool SchedulerSimulatedAnnealing::schedule_it(SSDfgVecInput *vec, SSDfg *ssDFG, Schedule *sched) {
  return schedule_io<SSDfgVecInput>(vec, ssDFG, sched);
}

template<> inline bool SchedulerSimulatedAnnealing::schedule_it(SSDfgVecOutput *vec, SSDfg *ssDFG, Schedule *sched) {
  return schedule_io<SSDfgVecOutput>(vec, ssDFG, sched);
}

template<> inline bool SchedulerSimulatedAnnealing::schedule_it(SSDfgInst *node, SSDfg *ssDFG, Schedule *sched) {
  return scheduleNode(sched, node);
}

template<typename T> inline void SchedulerSimulatedAnnealing::unmap_one(SSDfg *dfg, Schedule *sched) {
  const auto &nodes = dfg->nodes<T*>();
  int n = nodes.size();
  int p = rand() % n;
  while (true) {
    for (int i = 0; i < n; ++i) {
      p = (p + 1) % n;
      // TODO(@were): add extra estimation to this function...
      if (sched->is_scheduled(nodes[i]) && nodes[i]->yield(sched, _ssModel->subModel())) {
        sched->unassign<T*>(nodes[i]);
        return;
      }
    }
  }
}

#endif
