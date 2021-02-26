#include "dsa/mapper/scheduler_sa.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <list>
#include <unordered_map>

#include "dsa/debug.h"
#include "dsa/dfg/visitor.h"
#include "dsa/mapper/scheduler.h"

using namespace dsa;
using namespace std;

// Major things left to try that might improve the algorithm:
// 1. Overprovisioning while routing/placing
//  (DONE -- iterative+overprovisioning is incredibly useful for large graphs)
// 2. True Simulated Annealing -- include a temperature that controls randomness?
// 3. Prioritizing nodes based on what their presumed effect on scheduling
//   quality might be.  This may be difficult to gauge though.
// 4. Including multiple schedules to work on concurrently.  Sometimes its useful
//   to work on very different schedules, as the scheduler easily gets stuck
//   in local optima.
// 5. Incorperating Multithreading -- this might work nicely with 4, where each
//   thread works on a different schedule.
// 6. Better way to fix_latency.  One option is max_flow formulation as suggested
// by Jian; not sure it will be faster than the iterative heuristic (it could be
// exactly the same).  Another option is a proper ILP -- this is no fun fun to
// maintain with an interface to another language (eg. gams), but otherwise its
// okay.
// 7. Introduce more randomness into routing itself?  Maybe randomly de-prioritize
// a path.
// 8. For paths where you multi-cast a value (A->B A->C ...), finding the optimal
// version is a steiner-tree problem for graphs.  A direct implementation may be better
// than our approximate version (Which uses multiple djkstra's instances).  Warning
// that steiner problems often don't have polynomial solutions, so *some* approximation
// is necessary.
// 9. For delay matching, it may be useful to insert dummy nodes at the places
// in the graph where there is high latency violation -- OR -- change the router
// so that it tries to route empty functional units when there it is a high-violation
// edge

void SchedulerSimulatedAnnealing::initialize(SSDfg* ssDFG, Schedule*& sched) {
  if (sched == nullptr) {
    sched = new Schedule(getSSModel(), ssDFG);  // just a dummy one
  }
}

std::pair<int, int> SchedulerSimulatedAnnealing::obj(Schedule*& sched, SchedStats& s) {
  int num_left = sched->num_left();
  bool succeed_sched = (num_left == 0);

  sched->get_overprov(s.ovr, s.agg_ovr, s.max_util);
  sched->fixLatency(s.lat, s.latmis);

  int violation = sched->violation();

  int obj = s.agg_ovr * 1000 + violation * 200 + s.latmis * 200 + s.lat +
            (s.max_util - 1) * 3000 + sched->total_passthrough;
  obj = obj * 100 + sched->num_links_mapped();

  return make_pair(succeed_sched - num_left, -obj);
}

bool SchedulerSimulatedAnnealing::length_creep(Schedule* sched, dsa::dfg::Edge* edge,
                                               int& num, CandidateRoute& undo_routing) {
  bool changed = false;

  int chances_left = 40;
  while (chances_left-- && num > 0) {
    auto& links = sched->links_of(edge);

    if (links.empty()) continue;

    int rand_link_no = rand() % links.size();
    auto it = links.begin();
    for (int i = 0; i < rand_link_no; ++i) ++it;
    std::pair<int, sslink*> rand_link = *it;

    auto& edge_list = sched->edge_list(rand_link.first, rand_link.second);

    // Back to normal stuff
    bool bad_spot = false;
    for (auto& it : edge_list) {
      dsa::dfg::Edge* alt_edge = &sched->ssdfg()->edges[it.eid];
      if (sched->vioOf(alt_edge) < num + 1) {
        bad_spot = true;
        break;
      }
    }
    if (bad_spot) continue;

    for (auto& it : edge_list) {
      dsa::dfg::Edge* edge = &sched->ssdfg()->edges[it.eid];
      if (!undo_routing.edges.count(edge)) undo_routing.fill_edge(edge, sched);
    }

    if (ssswitch* sw = dynamic_cast<ssswitch*>(rand_link.second->sink())) {
      auto source = make_pair(rand_link.first, sw);
      int inserted = route(sched, edge, source, source, &(++it), num + 1);
      if (inserted) {
        for (auto& it : edge_list) {
          dsa::dfg::Edge* alt_edge = &sched->ssdfg()->edges[it.eid];
          sched->record_violation(alt_edge, sched->vioOf(alt_edge) - inserted);
          if (alt_edge == edge) continue;

          auto from_it = links.begin();
          for (int i = 0; i < rand_link_no + 1; ++i) ++from_it;
          for (int i = 0; i < inserted; ++i) {
            std::pair<int, sslink*> from_link = *from_it;
            auto& alt_links = sched->links_of(alt_edge);
            auto alt_it = alt_links.begin() + rand_link_no + 1 + i;
            sched->assign_edgelink(alt_edge, from_link.first, from_link.second, alt_it);
            if (auto fu = dynamic_cast<ssfu*>(from_link.second->sink())) {
              if (i + 1 != inserted) {
                sched->assign_edge_pt(alt_edge, make_pair(from_link.first, fu));
              }
            }

            from_it++;  // increment the iterator to copy a new node
            // alt_it++;   -- don't need to do this because we insert before...
          }
        }
        num -= inserted;
        chances_left++;
        changed = true;
      }

      // for (auto& it : edge_list) {
      //  dsa::dfg::Edge* alt_edge = it.first;
      //  sched->check_links_consistency(alt_edge);
      //}
    }
  }
  return changed;
}

std::pair<int, int> SchedulerSimulatedAnnealing::obj_creep(Schedule*& sched,
                                                           SchedStats& s,
                                                           CandidateRoute& undo_routing) {
  int num_left = sched->num_left();
  std::pair<int, int> curScore = obj(sched, s);

  std::unordered_map<dsa::dfg::Node*, int> accum_vio;
  if (s.agg_ovr == 0 && num_left == 0) {
    accum_vio.clear();  // just clear this
    std::vector<dsa::dfg::Node*> ordered_non_temp;
    std::copy_if(sched->reversed_topo.begin(), sched->reversed_topo.end(),
                 std::back_inserter(ordered_non_temp),
                 [](dsa::dfg::Node* node) { return !node->is_temporal(); });

    auto creep_it = ordered_non_temp.rbegin();

    while (creep_it != ordered_non_temp.rend()) {
      if (s.latmis == 0) break;

      dsa::dfg::Node* v = *creep_it;
      if (v->is_temporal()) continue;
      // int node_vio = sched->vioOf(v);

      int r = rand() % 4;
      if (r != 0) continue;

      for (auto& op : v->ops()) {
        for (auto eid : op.edges) {
          auto* e = &v->ssdfg()->edges[eid];
          int vio = sched->vioOf(e);
          if (vio > 0) {
            LOG(CREEP) << e->name() << ": " << vio;
            vio = rand() % vio;
            bool changed = false;
            changed |= length_creep(sched, e, vio, undo_routing);
            if (changed) obj(sched, s);
          }
        }
      }

      ++creep_it;
    }

    // while(creep_it != ordered_non_temp.rend()) {
    //  if(latmis==0) break;

    //  //int r = rand(0) % 4;
    //  //if(r!=0) continue;

    //  dsa::dfg::Node* v = *creep_it;
    //  //int node_vio = sched->vioOf(v);

    //  bool changed=false;
    //  for(auto e : v->in_edges()) {
    //    int vio = sched->vioOf(e);
    //    changed |= length_creep(sched,e,vio,undo_routing);
    //  }

    //  if(changed) obj(sched,lat,latmis,ovr,agg_ovr,max_util);

    //  ++creep_it;
    //}
    return obj(sched, s);
  }
  return curScore;
}

/* This function will perform one iteration of an incremental scheduling of all
 * workloads which are stored in the scheduling tabl
 */
bool SchedulerSimulatedAnnealing::incrementalSchedule(CodesignInstance& inst) {
  // need to make sure the scheduler has the right submodel
  _ssModel = inst.ss_model();

  // Just for fun, lets see if we can:
  // 1. create a deep copy of the sub model
  // 2. create a copy of the schedule
  // 3. make the schedules consistent with the new deep copy
  // 4. try to schedule something on it.
  // 5. profit?

  is_dse = true;

  int i = 0;
  for (WorkloadSchedules& ws : inst.workload_array) {
    int j = 0;
    for (Schedule& sr : ws.sched_array) {
      Schedule* sched = &sr;

      SchedStats s;

      schedule(sched->ssdfg(), sched);
      auto p = obj(sched, s);
      cout << "### Schedule (" << sched->ssdfg()->filename << "): " << -p.second
           << " ###\n";
      ++j;
    }
    ++i;
  }

  return true;
}

bool SchedulerSimulatedAnnealing::schedule(SSDfg* ssDFG, Schedule*& sched) {
  std::cout << "Start Schedule" << std::endl;

  initialize(ssDFG, sched);  // initialize if null, otherwise its fine
  auto pdgname = basename(ssDFG->filename);
  auto modelname = basename(_ssModel->filename);
  if (!check_feasible(sched->ssdfg(), sched->ssModel(), false /*silent*/)) {
    LOG(MAP) << "Cannot be mapped, give up!\n";
    return false;
  }

  int max_iters_no_improvement = _ssModel->subModel()->node_list().size() * 50;

  Schedule* cur_sched = new Schedule(getSSModel(), ssDFG);
  *cur_sched = *sched;

  std::pair<int, int> best_score = make_pair(0, 0);
  bool best_succeeded = false;
  bool best_mapped = false;

  int last_improvement_iter = 0;

  _best_lat = MAX_ROUTE;
  _best_violation = MAX_ROUTE;

  int presize = ssDFG->type_filter<dsa::dfg::Instruction>().size();

  int iter = 0;
  int fail_to_route = 0;
  for (iter = 0; iter < max_iters; ++iter) {
    if ((total_msec() > _reslim * 1000) || _should_stop) {
      break;
    }

    bool print_stat = (iter & (256 - 1)) == 0;

    // if we don't improve for some time, lets reset
    if (iter - last_improvement_iter > 1024) {
      *cur_sched = *sched;
    }

    int status = schedule_internal(ssDFG, cur_sched);
    if (status == 0) {
      LOG(MAP) << "Insufficient candidates!";
      return false;
    }
    if (status == -1) {
      if (++fail_to_route > 32) {
        return false;
      }
      LOG(ROUTING) << "Problem with Topology -- Mapping Impossible";
      continue;
    }

    fail_to_route = 0;

    bool succeed_sched = cur_sched->is_complete<dsa::dfg::Node*>();

    SchedStats s;
    std::pair<int, int> score = obj(cur_sched, s);

    int succeed_timing = (s.latmis == 0) && (s.ovr == 0);

    if (verbose && ((score > best_score) || print_stat)) {
      stringstream ss;
      ss << "viz/iter/" << iter << ".gv";
      cur_sched->printGraphviz(ss.str().c_str());

      for (auto& elem : ssDFG->type_filter<dsa::dfg::InputPort>()) {
        std::cout << cur_sched->vecPortOf(&elem) << " ";
      }
      std::cout << "|";
      for (auto& elem : ssDFG->type_filter<dsa::dfg::OutputPort>()) {
        std::cout << cur_sched->vecPortOf(&elem) << " ";
      }

      fprintf(stdout,
              "Iter: %4d, time:%0.2f, kRPS:%0.1f, left: %3d, "
              "lat: %3d, vio: %d, mis: %d, ovr: %d, agg_ovr: %d, util: %d, "
              "obj:%d, ins: %d/%d, outs: %d/%d,"
              " insts: %d,%d, pts:%d, links:%d, edge-links:%d  %s%s",
              iter, total_msec() / 1000.f, routing_times / total_msec(),
              cur_sched->num_left(), s.lat, cur_sched->violation(), s.latmis, s.ovr,
              s.agg_ovr, s.max_util, -score.second,
              cur_sched->num_mapped<dsa::dfg::InputPort>(),
              (int)ssDFG->type_filter<dsa::dfg::InputPort>().size(),
              cur_sched->num_mapped<dsa::dfg::OutputPort>(),
              (int)ssDFG->type_filter<dsa::dfg::OutputPort>().size(),
              cur_sched->num_mapped<dsa::dfg::Instruction>(), presize,
              cur_sched->total_passthrough, cur_sched->num_links_mapped(),
              cur_sched->num_edge_links_mapped(), succeed_sched ? ", all mapped" : "",
              succeed_timing ? ", mismatch == 0" : "");
      if (score > best_score) {
        std::cout << std::endl;
      } else {
        std::cout.flush();
        std::cout << "\r";
      }
    }

    if (score > best_score) {
      sched->printGraphviz("viz/cur-best.gv");

      best_score = score;
      *sched = *cur_sched;  // shallow copy of sched should work?

      best_mapped = succeed_sched;
      best_succeeded = succeed_timing;
      last_improvement_iter = iter;

      if (dump_mapping_if_improved) {
        stringstream mapping_file_str;
        std::string mapping_base = mapping_file.substr(0, mapping_file.find_last_of("."));
        mapping_file_str << mapping_base << "-iter-" << iter << ".json";
        std::string mapping_file_iter = mapping_file_str.str();
        sched->DumpMappingInJson(mapping_file_iter);
      }
    }

    if (((iter - last_improvement_iter) > max_iters_no_improvement)) {
      break;
    }

    if (best_succeeded) {
      break;
    }
  }

  if (verbose) {
    std::cout << "Breaking at Iter " << iter
              << ", candidates success / candidates tried:  " << this->candidates_succ
              << "/" << this->candidates_tried << std::endl;
  }

  if (cur_sched) {
    delete cur_sched;
  }

  return best_mapped;
}

struct CandidateFinder : dfg::Visitor {
  CandidateFinder(Schedule* sched_) : sched(sched_) {
    CHECK(sched->ssModel())
        << "The given schedule should have an underlying spatial architecture!";
    model = sched->ssModel();
  }
  Schedule* sched;
  SSModel* model;

  void Visit(dsa::dfg::Instruction* inst) {}
};

#include "./pass/candidates.h"

int SchedulerSimulatedAnnealing::map_to_completion(SSDfg* ssDFG, Schedule* sched) {
  auto nodes = sched->ssdfg()->nodes;
  int n = nodes.size();
  dsa::mapper::CandidateSpotVisitor cpv(sched, 50);

  std::sort(nodes.begin(), nodes.end(), [sched](dsa::dfg::Node* a, dsa::dfg::Node* b) {
    return sched->candidate_cnt[a->id()] < sched->candidate_cnt[b->id()];
  });

  int from = 0;
  for (int i = 1; i < n; ++i) {
    if (sched->candidate_cnt[nodes[i - 1]->id()] !=
        sched->candidate_cnt[nodes[i]->id()]) {
      std::random_shuffle(nodes.begin() + from, nodes.begin() + i);
      from = i;
    }
  }
  std::random_shuffle(nodes.begin() + from, nodes.begin() + n);

  for (int i = 0; i < n; ++i) {
  }

  for (int i = 0; i < 3; ++i) {
    LOG(CAND) << i;
    for (int j = 0; j < n; ++j) {
      dsa::dfg::Node* node = nodes[j];
      if (!sched->is_scheduled(node)) {
        node->Accept(&cpv);
        auto& candidates = cpv.candidates[node->id()];
        if (candidates.empty()) {
          LOG(CAND) << ssDFG->filename << ": "
                    << "Cannot map " << node->name();
          break;
        }
        LOG(CAND) << node->name() << " has " << candidates.size()
                  << " candidate(s)";
        int best_candidate = try_candidates(candidates, sched, node);
        if (best_candidate == -1) {
          unmap_some(ssDFG, sched);
          std::random_shuffle(nodes.begin(), nodes.end());
          break;
        }
        sched->candidate_cnt[node->id()] = candidates.size();
      }
    }
  }

  return sched->is_complete<dsa::dfg::Node*>() ? 1 : -1;
}

void SchedulerSimulatedAnnealing::unmap_some(SSDfg* ssDFG, Schedule* sched) {
  int r = rand() % 1000;  // upper limit defines ratio of input/output scheduling
  int num_to_unmap = (r < 5) ? 10 : (r < 250 ? 4 : 2);

  struct Unmapper : dfg::Visitor {
    Schedule* sched;
    int total, to_do;

    Unmapper(Schedule* sched_, int to_do_)
        : sched(sched_), total(sched->num_mapped<dsa::dfg::Node*>()), to_do(to_do_) {}

    void Visit(dsa::dfg::Node* node) override {
      if (sched->is_scheduled(node) && rand() % total < to_do) {
        sched->unassign_dfgnode(node);
        --total;
      }
    }
  };

  Unmapper u(sched, num_to_unmap);

  if (sched->num_mapped<dsa::dfg::Node*>()) ssDFG->Apply(&u);
}

int SchedulerSimulatedAnnealing::schedule_internal(SSDfg* ssDFG, Schedule*& sched) {
  unmap_some(ssDFG, sched);
  return map_to_completion(ssDFG, sched);
}

int SchedulerSimulatedAnnealing::routing_cost(dsa::dfg::Edge* edge, int from_slot,
                                              int next_slot, sslink* link,
                                              Schedule* sched,
                                              const pair<int, ssnode*>& dest) {
  // TODO(@were): Do I have a better way to implement this?
  dsa::dfg::Node* def_dfgnode = edge->def();
  dsa::dfg::Node* use_dfgnode = edge->use();

  bool is_in = def_dfgnode->type() == dsa::dfg::Node::V_INPUT;
  bool is_out = use_dfgnode->type() == dsa::dfg::Node::V_OUTPUT;
  bool is_io = is_in || is_out;

  bool is_temporal = def_dfgnode->is_temporal();
  bool is_temporal_inst = is_temporal && !is_io;
  bool is_temporal_in = is_temporal && is_in;
  bool is_temporal_out = is_temporal && is_out;

  int internet_dis = abs(from_slot - next_slot);
  internet_dis = min(internet_dis, 8 - internet_dis);

  // For now, links only route on their own network
  // ie. temporal_io and non-temporal route on dedicated network
  if ((!is_temporal_inst && link->max_util() > 1) ||
      (is_temporal_inst && link->max_util() <= 1)) {
    return -1;
  }

  ssnode* next = link->sink();

  // check if connection is closed..
  // 0: free
  // 1: empty
  // 2: already there
  // FIXME(@were): Move these to a virtual method!
  int t_cost;
  if (is_temporal_in) {
    t_cost = sched->routing_cost_temporal_in(
        link, dynamic_cast<dsa::dfg::InputPort*>(def_dfgnode));
  } else if (is_temporal_out) {
    t_cost = sched->routing_cost_temporal_out(
        make_pair(from_slot, link), def_dfgnode,
        dynamic_cast<dsa::dfg::OutputPort*>(use_dfgnode));
  } else {  // NORMAL CASE!
    t_cost = sched->routing_cost(make_pair(from_slot, link), edge);
  }

  if (t_cost >= 2) {  // square law avoidance of existing routes
    if (!is_temporal_inst) {
      return (t_cost) * (t_cost)*10;
    }
  }

  bool is_dest = (next == dest.second && next_slot == dest.first);

  ssfu* fu = dynamic_cast<ssfu*>(next);
  if (fu && !is_dest) {
    t_cost += 10;
    int count = sched->dfg_nodes_of(next_slot, fu).size() +
                sched->node_prop()[fu->id()].slots[next_slot].passthrus.size();
    t_cost += 20 * count * count;
  }

  t_cost += internet_dis;
  return t_cost;
}

void insert_edge(std::pair<int, sslink*> link, Schedule* sched, dsa::dfg::Edge* edge,
                 std::vector<std::pair<int, sslink*>>::iterator it,
                 std::pair<int, ssnode*> dest, std::pair<int, ssnode*> x) {
  sched->assign_edgelink(edge, link.first, link.second, it);

  if (dynamic_cast<ssfu*>(link.second->sink())) {
    if ((dest != x) && !sched->dfgNodeOf(link.first, link.second)) {
      sched->assign_edge_pt(edge, x);
    }
  }
}

int SchedulerSimulatedAnnealing::route(
    Schedule* sched, dsa::dfg::Edge* edge, std::pair<int, dsa::ssnode*> source,
    std::pair<int, dsa::ssnode*> dest) {

  struct DfsRouting {
    DfsRouting(SchedulerSimulatedAnnealing *ssa_,
               Schedule *sched_,
               const std::pair<int, dsa::ssnode*> &src_,
               const std::pair<int, dsa::ssnode*> &dst_,
               dsa::dfg::Edge *edge_) :
      ssa(ssa_), sched(sched_), src(src_), dst(dst_), edge(edge_) {
      visited.resize(sched_->ssModel()->subModel()->node_list().size(),
                     std::vector<bool>(8, false));
    }

  #define LOG2(x) (31 - __builtin_clz(x))
    void DfsImpl(const std::pair<int, dsa::ssnode*> &cur) {
      if (visited[cur.second->id()][cur.first]) {
        return;
      }
      visited[cur.second->id()][cur.first] = true;
      {
        for (auto elem : path) {
          LOG(ROUTE) << elem.first << ", " << elem.second->name();
        }
      }
      if (cur == dst) {
        LOG(ROUTE) << "Arrived!";
        found = true;
        return;
      } else {
        LOG(ROUTE) << "==========================";
      }
      int slot = cur.first;
      auto *node = cur.second;
      std::vector<std::tuple<int, int, int, dsa::ssnode*, int>> q;
      LOG(ROUTE) << "od: " << node->out_links().size();
      for (int i = 0, n = node->out_links().size(); i < n; ++i) {
        auto link = node->out_links()[i];
        int slots = link->slots(slot, edge->bitwidth() / 8);
        auto next = link->sink();
        int dist = sched->distances[next->id()][dst.second->id()];
        if (dist >= (int)1e9) {
          continue;
        }
        while (slots) {
          int raw = slots & -slots;
          slots -= raw;
          int next_slot = LOG2(raw);
          std::pair<int, sslink*> next_pair(next_slot, link);

          int route_cost = ssa->routing_cost(edge, slot, next_slot, link, sched, dst);

          LOG(ROUTE) << "!!!! " << node->name() << " -> " << next->name()
                     << ": " << route_cost << ", " << dist;
          q.emplace_back(route_cost + dist, route_cost, next_slot, next, i);
        }
      }
      std::sort(q.begin(), q.end());
      // if (rand() % 10 == 0) {
      //   int i = 0;
      //   while (i < q.size() && std::get<1>(q[i]) <= 1) {
      //     ++i;
      //   }
      //   std::random_shuffle(q.begin(), q.begin() + i);
      // }
      for (auto &elem : q) {
        int cost = std::get<0>(elem);
        int dist = std::get<1>(elem);
        std::pair<int, dsa::ssnode*> next(std::get<2>(elem), std::get<3>(elem));
        path.emplace_back(std::get<2>(elem), node->out_links()[std::get<4>(elem)]);
        DfsImpl(next);
        if (found) {
          break;
        }
        path.pop_back();
      }
    }
  #undef LOG2

    void Route() {
      LOG(ROUTE) << src.second->name() << "," << src.first << "(" << src.second->id() << ") -> "
                 << dst.second->name() << "," << dst.first << "(" << dst.second->id() << ")";
      LOG(ROUTE) << "distance: " << sched->distances[src.second->id()][dst.second->id()];
      DfsImpl(src);
      if (found) {
        for (auto elem : path) {
          sched->assign_edgelink(edge, elem.first, elem.second);
          if (dynamic_cast<dsa::ssfu*>(elem.second->sink())) {
            if (elem.first == dst.first && elem.second->sink() == dst.second) {
              continue;
            }
            sched->assign_edge_pt(edge, {elem.first, elem.second->sink()});
          }
        }
      }
    }

    SchedulerSimulatedAnnealing *ssa;
    Schedule *sched;
    std::pair<int, dsa::ssnode*> src;
    std::pair<int, dsa::ssnode*> dst;
    dsa::dfg::Edge *edge;
    std::vector<std::pair<int, dsa::sslink*>> path;
    std::vector<std::vector<bool>> visited;
    bool found{false};
  };
  // if (rand() % 2 == 0) {
    DfsRouting dr(this, sched, source, dest, edge);
    dr.Route();
    return dr.found;
  // }
  // return route(sched, edge, source, dest, nullptr, false);
}


int SchedulerSimulatedAnnealing::route(
    Schedule* sched, dsa::dfg::Edge* edge, std::pair<int, dsa::ssnode*> source,
    std::pair<int, dsa::ssnode*> dest,
    std::vector<std::pair<int, sslink*>>::iterator* ins_it, int max_path_lengthen) {
  // if (!sched->ssModel()->subModel()->connected[source.second->id()][dest.second->id()])
  // {
  //  return 0;
  //}

  bool path_lengthen = ins_it != nullptr;

  _ssModel->subModel()->clear_all_runtime_vals();

  if (!path_lengthen) ++routing_times;

  if (source == dest && !path_lengthen) {
    return 1;
  }

  // FIXME: comment/delete this later
  // if(!path_lengthen) sched->edge_prop()[edge->id()].sched_index.clear();
  // sched->edge_prop()[edge->id()].sched_index.push_back(_route_times);

  CHECK(path_lengthen || sched->link_count(edge) == 0)
      << "Edge: " << edge->name() << " is already routed!";

  // Distance, random priority, slot, node
  set<std::tuple<int, int, int, ssnode*>> openset;

  if (!path_lengthen) source.first = edge->def()->slot_for_use(edge, source.first);
  dest.first = edge->use()->slot_for_op(edge, dest.first);

  int new_rand_prio = 0;                                 // just pick zero
  source.second->set_done(source.first, new_rand_prio);  // remeber for deleting
  openset.emplace(0, new_rand_prio, source.first, source.second);
  source.second->update_dist(source.first, 0, 0, nullptr);

  while (!openset.empty()) {
    int cur_dist = std::get<0>(*openset.begin());
    int slot = std::get<2>(*openset.begin());
    ssnode* node = std::get<3>(*openset.begin());

    openset.erase(openset.begin());

    if (!path_lengthen) {
      if (slot == dest.first && node == dest.second) break;
    } else {
      if ((slot == dest.first && node == dest.second && cur_dist > 0) ||
          (cur_dist > max_path_lengthen)) {
        break;
      }
    }

    for (auto link : node->out_links()) {
      int slots = link->slots(slot, edge->bitwidth() / 8);
      while (slots) {
        int raw = slots & -slots;
        slots -= raw;
        int next_slot = 31 - __builtin_clz(raw);
        sslink* next_link = link;
        ssnode* next = next_link->sink();
        std::pair<int, sslink*> next_pair(next_slot, next_link);

        LOG(SLOTS) << "width: " << edge->bitwidth() << ", From " << link->source()->name()
                   << "'s " << slot << " to " << link->sink()->name() << "'s "
                   << next_slot << "\n";

        int route_cost;
        if (!path_lengthen) {  // Normal thing
          route_cost = routing_cost(edge, slot, next_slot, next_link, sched, dest);
        } else {
          // For path lengthening, only route on free spaces
          route_cost = sched->routing_cost(next_pair, edge);
          if (route_cost != 1 || sched->dfgNodeOf(next_slot, next)) route_cost = -1;
        }

        if (route_cost == -1) continue;

        int new_dist = cur_dist + route_cost;

        int next_dist = next->node_dist(next_slot);

        bool over_ride = (path_lengthen && make_pair(next_slot, next) == dest);

        if (next_dist == -1 || next_dist > new_dist || over_ride) {
          if (next_dist != -1) {
            int next_rand_prio = next->done(next_slot);
            auto iter =
                openset.find(std::make_tuple(next_dist, next_rand_prio, next_slot, next));
            if (iter != openset.end()) openset.erase(iter);
          }
          int new_rand_prio = rand() % 16;
          next->set_done(next_slot, new_rand_prio);  // remeber for later for deleting
          openset.emplace(new_dist, new_rand_prio, next_slot, next);
          next->update_dist(next_slot, new_dist, slot, next_link);
        }
      }
    }
  }

  if (dest.second->node_dist(dest.first) == -1 ||
      (path_lengthen && dest.second->node_dist(dest.first) == 0)) {
    return false;  // routing failed, no routes exist!
  }

  auto it = ins_it ? *ins_it : sched->links_of(edge).begin();
  auto idx = ins_it ? *ins_it - sched->links_of(edge).begin() : 0;
  auto x = dest;

  int count = 0;
  dsa::dfg::Edge* alt_edge = nullptr;
  pair<int, sslink*> link;

  while (x != source || (path_lengthen && count == 0)) {
    count++;
    link = x.second->came_from(x.first);

    auto link_backup = link;

    link.first = x.first;

    if ((alt_edge = sched->alt_edge_for_link(link, edge))) {
      break;
    }
    insert_edge(link, sched, edge, sched->links_of(edge).begin() + idx, dest, x);

    x = std::make_pair(link_backup.first, link.second->source());
  }

  // Need to make sure we add back the other links in-order.
  // With path lengthening (which has cycles) the previous
  // code can't gaurantee that.
  if (alt_edge) {
    auto& alt_links = sched->links_of(alt_edge);
    for (auto alt_link : alt_links) {
      x = std::make_pair(alt_link.first, alt_link.second->source());
      insert_edge(alt_link, sched, edge, sched->links_of(edge).begin() + idx, dest, x);
      idx++;

      if (alt_link == link) break;  // we added the final link
    }
  }

  return count;
}

bool SchedulerSimulatedAnnealing::scheduleHere(Schedule* sched, dsa::dfg::Node* node,
                                               pair<int, dsa::ssnode*> here) {
  std::vector<dsa::dfg::Edge*> to_revert;

#define process(edge_, node_, src, dest)                                           \
  do {                                                                             \
    auto& edges = edge_;                                                           \
    int n = edge_.size();                                                          \
    for (int i = 0; i < n; ++i) {                                                  \
      auto edge = edges[i];                                                        \
      LOG(ROUTE) << "Routing " << edge->name();                                    \
      CHECK(sched->link_count(edge) == 0)                                          \
          << "Edge: " << edge->name() << " is already routed!\n";                  \
      auto node = edge->node_();                                                   \
      if (sched->is_scheduled(node)) {                                             \
        auto loc = sched->location_of(node);                                       \
        if (!route(sched, edge, src, dest)) {                                      \
          LOG(ROUTE) << "Cannot route " << edge->name() << " " << src.second->id() \
                     << " -> " << dest.second->id() << ":"                         \
                     << sched->distances[src.second->id()][dest.second->id()];     \
          for (auto revert : to_revert) sched->unassign_edge(revert);              \
          for (int j = 0; j < i; ++j) sched->unassign_edge(edges[j]);              \
          return false;                                                            \
        }                                                                          \
      }                                                                            \
    }                                                                              \
  } while (false)

  process(sched->operands[node->id()], def, loc, here);
  to_revert = sched->operands[node->id()];
  process(sched->users[node->id()], use, here, loc);

#undef process

  sched->assign_node(node, here);

  return true;
}

int SchedulerSimulatedAnnealing::try_candidates(
    const std::vector<std::pair<int, ssnode*>>& candidates, Schedule* sched,
    dsa::dfg::Node* node) {
  LOG(MAP) << "Mapping " << node->name();
  using std::make_pair;
  using std::pair;
  using std::vector;
  CandidateRoute best_path;

  pair<int, int> bestScore = std::make_pair(INT_MIN, INT_MIN);
  int best_candidate = -1;
  bool find_best = rand() % 128;

  if (candidates.empty()) return 0;

  std::vector<int> src, dst, idx, keys;

  for (auto edge : sched->operands[node->id()]) {
    if (auto node = sched->locationOf(edge->def())) {
      src.push_back(node->id());
    }
  }
  for (auto edge : sched->users[node->id()]) {
    if (auto node = sched->locationOf(edge->use())) {
      dst.push_back(node->id());
    }
  }
  for (size_t i = 0; i < candidates.size(); ++i) {
    idx.push_back(i);
  }
  for (size_t i = 0; i < candidates.size(); ++i) {
    int sum = 0;
    for (auto elem : src) {
      sum += sched->distances[elem][candidates[i].second->id()];
    }
    for (auto elem : dst) {
      sum += sched->distances[candidates[i].second->id()][elem];
    }
    keys.push_back(sum);
  }
  std::sort(idx.begin(), idx.end(), [&keys](int a, int b) { return keys[a] < keys[b]; });

  int no_imporve = 0;
  for (size_t i = 0; i < candidates.size(); ++i) {
    ++candidates_tried;
    LOG(MAP) << "Try " << i << ": " << candidates[idx[i]].second->name();

    if (scheduleHere(sched, node, candidates[idx[i]])) {
      ++candidates_succ;
      LOG(MAP) << "succ!";

      SchedStats s;
      CandidateRoute undo_path;
      LOG(MAP) << "creeping!!";
      pair<int, int> candScore = obj_creep(sched, s, undo_path);
      LOG(MAP) << candScore.first << ", " << candScore.second;
      std::cout << candScore.first << ", " << candScore.second;

      if (!find_best) {
        return i;
      }

      if (candScore > bestScore) {
        best_candidate = idx[i];
        bestScore = candScore;
        best_path.edges.clear();
        best_path.fill_from(node, sched);
        best_path.fill_paths_from_undo(undo_path, sched);
        no_imporve = 0;
      } else {
        ++no_imporve;
      }

      undo_path.apply(sched);  // revert to paths before creep-based path lengthening
      sched->unassign_dfgnode(node);
      if (no_imporve > 8) {
        break;
      }
    }
  }

  if (best_candidate != -1) {
    best_path.apply(sched);
    sched->assign_node(node, candidates[best_candidate]);
    LOG(MAP) << node->name() << " maps to " << candidates[best_candidate].second->name();
  }

  LOG(MAP) << "return for best candidate!" << best_candidate;
  return best_candidate;
}

void CandidateRoute::apply(Schedule* sched) {
  std::map<std::tuple<int, int, int, int>, int> passthru_values;

  for (auto& edge_prop : edges) {
    sched->unassign_edge(edge_prop.first);  // for partial routes
    LOG(PASSTHRU) << edge_prop.first->name();
    for (auto elem : edge_prop.second.thrus) {
      sched->assign_edge_pt(edge_prop.first, elem);
    }
    for (auto elem : edge_prop.second.links) {
      sched->assign_edgelink(edge_prop.first, elem.first, elem.second);
    }
  }
}
