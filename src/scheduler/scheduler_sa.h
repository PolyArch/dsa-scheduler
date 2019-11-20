#ifndef __SS__SCHEDULE_SIMULATEDANNEALING_H__
#define __SS__SCHEDULE_SIMULATEDANNEALING_H__

#include <iostream>
#include "scheduler.h"

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

class SchedulerSimulatedAnnealing : public HeuristicScheduler {
 public:
  int candidates_tried{0}, candidates_succ{0};

  void initialize(SSDfg*, Schedule*&);

  SchedulerSimulatedAnnealing(SS_CONFIG::SSModel* ssModel)
      : HeuristicScheduler(ssModel) {}

  virtual bool schedule(SSDfg*, Schedule*&) override;

  virtual bool incrementalSchedule(CodesignInstance& incr_table) override;

  void set_fake_it() { _fake_it = true; }

  bool schedule_internal(SSDfg* ssDFG, Schedule*& sched);

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

  bool map_to_completion(SSDfg* ssDFG, Schedule* sched);

  bool map_io_to_completion(SSDfg* ssDFG, Schedule* sched);

  template <typename T>
  inline int try_candidates(const std::vector<std::pair<int, ssnode*>>& candidates,
                            Schedule*, T* node);

  template <typename T>
  static bool schedule_vec_impl(SchedulerSimulatedAnnealing* engine, T* vec, SSDfg* dfg,
                                Schedule* sched);

  template <typename T>
  inline bool map_one(SSDfg* ssDFG, Schedule* sched) {
    auto& nodes = ssDFG->nodes<T*>();
    int n = nodes.size();
    int p = rand() % n;
    for (int i = 0; i < n; ++i) {
      p = (p + 1) % n;
      T* node = nodes[p];
      if (!sched->is_scheduled(node)) {
        auto candidates = node->candidates(sched, _ssModel, 50);
        
        if(candidates.size()==0) {
          return false;
        }

        int best_candidate = try_candidates<SSDfgNode>(candidates, sched, node);
        return best_candidate != -1;
      }
    }
    return false;
  }

  template <typename T>
  inline void unmap_one(SSDfg* ssDFG, Schedule* sched);
  void unmap_some(SSDfg* ssDFG, Schedule* sched);

  bool _integrate_timing = true;
  int _best_latmis, _best_lat, _best_violation;
  bool _strict_timing = true;

  bool _fake_it = false;
};

template <typename T>
int SchedulerSimulatedAnnealing::try_candidates(
    const std::vector<std::pair<int, ssnode*>>& candidates, Schedule* sched, T* node) {
  using std::make_pair;
  using std::pair;
  using std::vector;
  CandidateRoute best_path;

  pair<int, int> bestScore = std::make_pair(INT_MIN, INT_MIN);
  int best_candidate = -1;
  bool find_best = rand() % 128;

  for (size_t i = 0; i < candidates.size(); ++i) {
    ++candidates_tried;

    if (scheduleHere(sched, node, candidates[i])) {
      ++candidates_succ;

      SchedStats s; 
      CandidateRoute undo_path;
      pair<int, int> candScore = obj_creep(sched, s, undo_path);

      if (!find_best) {
        return i;
      }

      if (candScore > bestScore) {
        best_candidate = i;
        bestScore = candScore;
        best_path.edges.clear();
        best_path.fill_from(node, sched);
        best_path.fill_paths_from_undo(undo_path, sched);
      }

      undo_path.apply(sched);  // revert to paths before creep-based path lengthening
      sched->unassign_dfgnode(node);
    }
  }

  if (best_candidate != -1) {
    best_path.apply(sched);
    sched->assign_node(node, candidates[best_candidate]);
  } else {
    std::cout << "Fail to schedule " << node->name() << std::endl;
    for (int i = 0, n_ = candidates.size(); i < n_; ++i) {
      std::cout << "Cand " << candidates[i].first << ", " << candidates[i].second->name()
                << std::endl;
    }
  }

  return best_candidate;
}

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
