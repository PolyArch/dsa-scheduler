#pragma once

#include <iostream>
#include <utility>
#include <map>

#include "scheduler.h"
#include "dse.h"

#define DEBUG_SCHED (false)

struct CandidateRoute {
  struct EdgeProp {
    std::vector<std::pair<int, ssnode*>> thrus;
    std::vector<std::pair<int, sslink*>> links;
  };

  // record an edge from the schedule
  void fill_edge(dsa::dfg::Edge* edge, Schedule* sched) {
    edges[edge].links = sched->links_of(edge);
    edges[edge].thrus = sched->thrus_of(edge);
  }
  void fill_from(SSDfgNode* node, Schedule* sched) {
    for (auto &op : node->ops()) {
      for (int eid : op.edges) {
        dsa::dfg::Edge *e = &node->ssdfg()->edges[eid];
        fill_edge(e, sched);
      }
    }
    for (auto &value : node->values) {
      for (int eid : value.uses) {
        dsa::dfg::Edge *e = &node->ssdfg()->edges[eid];
        fill_edge(e, sched);
      }
    }
  }

  // Fill paths that were recorded to later undo them
  void fill_paths_from_undo(CandidateRoute& undo_path, Schedule* sched) {
    for (auto it : undo_path.edges) {
      fill_edge(it.first, sched);
    }
  }

  void apply(Schedule* sched);

  std::unordered_map<dsa::dfg::Edge*, EdgeProp> edges;
};

class SchedulerSimulatedAnnealing : public Scheduler {
 public:
  bool is_dse{false};

  int routing_times{0};

  int candidates_tried{0}, candidates_succ{0};

  void initialize(SSDfg*, Schedule*&);

  SchedulerSimulatedAnnealing(dsa::SSModel* ssModel, double timeout = 1000000.,
                              int max_iters_ = 20000,
                              bool verbose = false,
                              std::string mapping_file_ = "",
                              bool dump_mapping_if_improved_ = false) : Scheduler(ssModel, timeout, verbose), 
                                                                        mapping_file(mapping_file_),
                                                                        dump_mapping_if_improved(dump_mapping_if_improved_),
                                                                        max_iters(max_iters_) {}

  virtual bool schedule(SSDfg*, Schedule*&) override;

  virtual bool incrementalSchedule(CodesignInstance& incr_table) override;

  int schedule_internal(SSDfg* ssDFG, Schedule*& sched);

 protected:
  std::pair<int, int> obj(Schedule*& sched, SchedStats& s);

  std::pair<int, int> obj_creep(Schedule*& sched, SchedStats& s,
                                CandidateRoute& undo_path);

  bool length_creep(Schedule* sched, dsa::dfg::Edge* edge, int& num, CandidateRoute& cand);

  template <typename T>
  bool scheduleHere(Schedule* sched, const std::vector<T>& nodes,
                    const std::vector<std::pair<int, dsa::ssnode*>>& cand) {
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

  bool scheduleHere(Schedule*, SSDfgNode*, std::pair<int, dsa::ssnode*>);

  int route(Schedule* sched, dsa::dfg::Edge* dfgnode,
            std::pair<int, dsa::ssnode*> source,
            std::pair<int, dsa::ssnode*> dest,
            std::vector<std::pair<int, sslink*>>::iterator* ins_it, int max_path_lengthen);

  int routing_cost(dsa::dfg::Edge*, int, int, sslink*, Schedule*,
                   const std::pair<int, ssnode*>&);

  bool timingIsStillGood(Schedule* sched);

  // 0: Lack of candidates
  // -1: Cannot route
  // 1: Success
  int map_to_completion(SSDfg* ssDFG, Schedule* sched);

  inline int try_candidates(const std::vector<std::pair<int, ssnode*>>& candidates,
                            Schedule*, SSDfgNode* node);

  void unmap_some(SSDfg* ssDFG, Schedule* sched);

  bool _integrate_timing = true;
  int _best_latmis, _best_lat, _best_violation;
  bool _strict_timing = true;

  std::string mapping_file{""};
  bool dump_mapping_if_improved{false};
  int max_iters;
};
