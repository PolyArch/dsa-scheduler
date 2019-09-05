#ifndef __SS__SCHEDULE_SIMULATEDANNEALING_H__
#define __SS__SCHEDULE_SIMULATEDANNEALING_H__

#include "scheduler.h"
#include <iostream>

#define DEBUG_SCHED (false)

struct CandidateRoute {
  std::vector<std::pair<SSDfgEdge*, std::pair<int, SS_CONFIG::ssnode*>>> thrus;
  std::vector<std::pair<SSDfgEdge*, std::pair<int, SS_CONFIG::sslink*>>> links;

  void swap(CandidateRoute &other) {
    thrus.swap(other.thrus);
    links.swap(other.links);
  }
};

class SchedulerSimulatedAnnealing : public HeuristicScheduler {
public:
  int candidates_tried{0}, candidates_succ{0};

  void initialize(SSDfg *, Schedule *&);

  SchedulerSimulatedAnnealing(SS_CONFIG::SSModel *ssModel) :
          HeuristicScheduler(ssModel) {}

  bool schedule(SSDfg *, Schedule *&) override;

  void set_fake_it() { _fake_it = true; }

  bool schedule_internal(SSDfg *ssDFG, Schedule *&sched);

protected:
  std::pair<int, int> obj(Schedule*& sched, int& lat, 
      int& lat_mis, int& ovr, int& agg_ovr, int& max_util); 

  template<typename T>
  bool scheduleHere(Schedule *sched, const std::vector<T> &nodes,
                    const std::vector<std::pair<int, SS_CONFIG::ssnode*>> &slots,
                    CandidateRoute &path) {
    assert(slots.size() == nodes.size());
    for (int i = 0; i < (int) nodes.size(); ++i) {
      if (!scheduleHere(sched, nodes[i], slots[i], path)) {
        for (int j = 0; j < i; ++j) {
          sched->unassign_dfgnode(nodes[j]);
        }
        return false;
      }
    }
    return true;
  }

  bool scheduleHere(Schedule *, SSDfgNode *, std::pair<int, SS_CONFIG::ssnode *>,
                    CandidateRoute &path);

  bool route(Schedule *sched, SSDfgEdge *dfgnode, std::pair<int, SS_CONFIG::ssnode *> source,
             std::pair<int, SS_CONFIG::ssnode *> dest, CandidateRoute &path);

  bool route_minimize_distance(Schedule *sched, SSDfgEdge *dfgnode,
                               std::pair<int, SS_CONFIG::ssnode *> source,
                               std::pair<int, SS_CONFIG::ssnode *> dest, CandidateRoute &path);

  int routing_cost(SSDfgEdge *, int, int, sslink *, Schedule *, const std::pair<int, ssnode *> &);

  bool timingIsStillGood(Schedule *sched);

  bool map_to_completion(SSDfg *ssDFG, Schedule *sched);

  bool map_io_to_completion(SSDfg *ssDFG, Schedule *sched);

  template<typename T> inline int
  try_candidates(const std::vector<std::pair<int, ssnode*>> &candidates, Schedule *, T* node);

  template<typename T>
  static bool schedule_vec_impl(SchedulerSimulatedAnnealing *engine, T *vec, SSDfg *dfg,
                                Schedule *sched);

  template<typename T> inline bool map_one(SSDfg *ssDFG, Schedule *sched) {
    auto &nodes = ssDFG->nodes<T*>();
    int n = nodes.size();
    int p = rand() % n;
    for(int i = 0; i < n; ++i) {
      p = (p + 1) % n;
      T* node = nodes[p];
      if (!sched->is_scheduled(node)) {
        auto candidates = node->candidates(sched, _ssModel, 50);
        int best_candidate = try_candidates<SSDfgNode>(candidates, sched, node);
        return best_candidate != -1;
      }
    }
    return false;
  }

  template<typename T> inline void unmap_one(SSDfg *ssDFG, Schedule *sched);
  void unmap_some(SSDfg *ssDFG, Schedule *sched);

  bool _integrate_timing = true;
  int _best_latmis, _best_lat, _best_violation;
  bool _strict_timing = true;

  bool _fake_it = false;

};

template<typename T>
int SchedulerSimulatedAnnealing::try_candidates(
  const std::vector<std::pair<int, ssnode*>> &candidates, Schedule *sched, T* node) {

  using std::vector;
  using std::pair;
  using std::make_pair;
  SSModel *model = sched->ssModel();
  CandidateRoute best_path;

  pair<int, int> bestScore = std::make_pair(INT_MIN, INT_MIN);
  int best_candidate = -1;
  bool find_best = rand() % 128;

  for (size_t i = 0; i < candidates.size(); ++i) {
    ++candidates_tried;

    CandidateRoute path;
    if (scheduleHere(sched, node,candidates[i], path)) {
      ++candidates_succ;

      int lat = INT_MAX, latmis = INT_MAX, ovr = INT_MAX,  agg_ovr = INT_MAX, max_util = INT_MAX;
      pair<int, int> candScore = obj(sched, lat, latmis, ovr, agg_ovr, max_util);

      if (!find_best) {
        return i;
      }

      if (candScore > bestScore) {
        best_candidate = i;
        bestScore = candScore;
        best_path.swap(path);
      }

      sched->unassign_dfgnode(node);
    }


  }

  if (best_candidate != -1) {
    for (auto elem : best_path.thrus) {
      sched->assign_edge_pt(elem.first, elem.second);
    }
    for (auto elem : best_path.links) {
      sched->assign_edgelink(elem.first, elem.second.first, elem.second.second);
    }
    sched->assign_node(node, candidates[best_candidate]);
  }

  return best_candidate;
}

template<typename T> inline void
SchedulerSimulatedAnnealing::unmap_one(SSDfg *dfg, Schedule *sched) {
  const auto &nodes = dfg->nodes<T*>();
  int n = nodes.size();
  int p = rand() % n;
  while (true) {
    for (int i = 0; i < n; ++i) {
      p = (p + 1) % n;
      // TODO(@were): add extra estimation to this function...
      if (sched->is_scheduled(nodes[p])) {
        sched->unassign_dfgnode(nodes[p]);
        return;
      }
    }
  }
}

#endif
