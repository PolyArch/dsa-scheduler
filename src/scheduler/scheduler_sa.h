#ifndef __SS__SCHEDULE_SIMULATEDANNEALING_H__
#define __SS__SCHEDULE_SIMULATEDANNEALING_H__

#include <iostream>

#include "scheduler.h"
#include "dse.h"

#define DEBUG_SCHED (false)

struct CandidateRoute {
  struct EdgeProp {
    std::list<std::pair<int, ssnode*>> thrus;
    std::list<std::pair<int, sslink*>> links;
  };

  // record an edge from the schedule
  void fill_edge(SSDfgEdge* edge, Schedule* sched) {
    edges[edge].links = sched->links_of(edge);
    edges[edge].thrus = sched->thrus_of(edge);
  }
  void fill_from(SSDfgNode* node, Schedule* sched) {
    for (SSDfgEdge* e : node->in_edges()) fill_edge(e, sched);
    for (SSDfgEdge* e : node->uses()) fill_edge(e, sched);
  }

  // Fill paths that were recorded to later undo them
  void fill_paths_from_undo(CandidateRoute& undo_path, Schedule* sched) {
    for (auto it : undo_path.edges) {
      fill_edge(it.first, sched);
    }
  }

  void apply(Schedule* sched) {
    for (auto edge_prop : edges) {
      sched->unassign_edge(edge_prop.first);  // for partial routes
      for (auto elem : edge_prop.second.thrus) {
        sched->assign_edge_pt(edge_prop.first, elem);
      }
      for (auto elem : edge_prop.second.links) {
        sched->assign_edgelink(edge_prop.first, elem.first, elem.second);
      }
    }
  }

  std::unordered_map<SSDfgEdge*, EdgeProp> edges;
};

class SchedulerSimulatedAnnealing : public Scheduler {
 public:
  bool is_dse{false};

  int routing_times{0};

  int candidates_tried{0}, candidates_succ{0};

  void initialize(SSDfg*, Schedule*&);

  SchedulerSimulatedAnnealing(SS_CONFIG::SSModel* ssModel, double timeout = 1000000.,
                              int max_iters_ = 20000,
                              bool verbose = false) : Scheduler(ssModel, timeout, verbose), max_iters(max_iters_) {}

  virtual bool schedule(SSDfg*, Schedule*&) override;

  virtual bool incrementalSchedule(CodesignInstance& incr_table) override;

  void set_fake_it() { _fake_it = true; }

  int schedule_internal(SSDfg* ssDFG, Schedule*& sched);

 protected:
  std::pair<int, int> obj(Schedule*& sched, SchedStats& s);

  std::pair<int, int> obj_creep(Schedule*& sched, SchedStats& s,
                                CandidateRoute& undo_path);

  bool length_creep(Schedule* sched, SSDfgEdge* edge, int& num, CandidateRoute& cand);

  template <typename T>
  bool scheduleHere(Schedule* sched, const std::vector<T>& nodes,
                    const std::vector<std::pair<int, SS_CONFIG::ssnode*>>& cand) {
    assert(cand.size() == nodes.size());
    for (int i = 0; i < (int)nodes.size(); ++i) {
      if (!scheduleHere(sched, nodes[i], cand[i])) {
        for (int j = 0; j < i; ++j) {
          sched->unassign_dfgnode(nodes[j]);
        }
        return false;
      }
    }
    return true;
  }

  bool scheduleHere(Schedule*, SSDfgNode*, std::pair<int, SS_CONFIG::ssnode*>);

  int route(Schedule* sched, SSDfgEdge* dfgnode,
            std::pair<int, SS_CONFIG::ssnode*> source,
            std::pair<int, SS_CONFIG::ssnode*> dest,
            std::list<std::pair<int, sslink*>>::iterator* ins_it, int max_path_lengthen);

  int routing_cost(SSDfgEdge*, int, int, sslink*, Schedule*,
                   const std::pair<int, ssnode*>&);

  bool timingIsStillGood(Schedule* sched);

  // 0: Lack of candidates
  // -1: Cannot route
  // 1: Success
  int map_to_completion(SSDfg* ssDFG, Schedule* sched);

  bool map_io_to_completion(SSDfg* ssDFG, Schedule* sched);

  inline int try_candidates(const std::vector<std::pair<int, ssnode*>>& candidates,
                            Schedule*, SSDfgNode* node);

  template <typename T>
  static bool schedule_vec_impl(SchedulerSimulatedAnnealing* engine, T* vec, SSDfg* dfg,
                                Schedule* sched);

  // 0: No candidates
  // -1: Failed to route
  // other: Successfully mapped one
  template <typename T>
  inline int map_one(SSDfg* ssDFG, Schedule* sched) {
    auto& nodes = ssDFG->nodes<T*>();
    int n = nodes.size();
    int p = rand() % n;
    for (int i = 0; i < n; ++i) {
      p = (p + 1) % n;
      T* node = nodes[p];
      if (!sched->is_scheduled(node)) {
        auto candidates = node->candidates(sched, _ssModel, 50);

        if (candidates.empty()) {
          return 0;
        }

        int best_candidate = try_candidates(candidates, sched, node);
        return best_candidate == -1 ? -1 : 1;
      }
    }
    assert(false);
  }

  template <typename T>
  inline void unmap_one(SSDfg* ssDFG, Schedule* sched);
  void unmap_some(SSDfg* ssDFG, Schedule* sched);

  bool _integrate_timing = true;
  int _best_latmis, _best_lat, _best_violation;
  bool _strict_timing = true;

  bool _fake_it = false;

  int max_iters;
};

template <typename T>
inline void SchedulerSimulatedAnnealing::unmap_one(SSDfg* dfg, Schedule* sched) {
  const auto& nodes = dfg->nodes<T*>();
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
