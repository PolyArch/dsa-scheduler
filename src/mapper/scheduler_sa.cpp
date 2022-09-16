#include <stdio.h>
#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <set>
#include <ctpl_stl.h>


#include <fstream>
#include <sstream>
#include <list>

#include "dsa/core/singleton.h"
#include "dsa/debug.h"
#include "dsa/dfg/visitor.h"
#include "dsa/dfg/node.h"
#include "dsa/dfg/port.h"
#include "../utils/vector_utils.h"
#include "dsa/mapper/schedule.h"
#include "dsa/mapper/scheduler.h"
#include "dsa/mapper/scheduler_sa.h"

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

std::pair<int64_t, int64_t> SchedulerSimulatedAnnealing::obj(Schedule*& sched, SchedStats& s) {
  int num_left = sched->num_left();
  bool succeed_sched = (num_left == 0);
  sched->violation_penalty = pair<int, int>(0, 0);

  sched->get_overprov(s.ovr, s.agg_ovr, s.max_util);
  sched->fixLatency(s.lat, s.latmis, sched->violation_penalty);

  int violation = sched->violation();

  int obj = s.agg_ovr * 1000 + violation * 200 + s.latmis * 200 + s.lat +
            (s.max_util - 1) * 3000 + sched->total_passthrough;
  obj = obj * 100 + sched->num_links_mapped();

  std::vector<double> nmlz_freq;
  for (int i = 0; i < sched->ssdfg()->meta.size(); ++i) {
    nmlz_freq.push_back(sched->ssdfg()->meta[i].frequency);
  }

  double nmlz = std::accumulate(nmlz_freq.begin(), nmlz_freq.end(), (double) 0);
  for (int i = 0; (long unsigned int) i < sched->ssdfg()->meta.size(); ++i) {
    nmlz_freq[i] /= nmlz;
  }

  double spm_performance_factor = 1.0;
  for (auto spad : sched->ssModel()->subModel()->scratch_list()) {
    // the production rate for this scratchpad node
    int productionRate = spad->readWidth() * spad->memUnitBits();

    double inputConsumption = 0;
    double outputConsumption = 0;

    // Populate input and output consumption rates
    auto vertices = sched->node_prop()[spad->id()].slots[0].vertices;
    for (auto vertex : vertices) {
      if (dfg::Array* arr = dynamic_cast<dfg::Array*>(vertex.first)) {
        auto input_consumtion = arr->Consumption(true);
        auto output_consumption = arr->Consumption(false);
        for (int i = 0; i < sched->ssdfg()->meta.size(); ++i) {
          inputConsumption += input_consumtion[i] * nmlz_freq[i];
          outputConsumption += output_consumption[i] * nmlz_freq[i];
        }
      }
    }
  
    spm_performance_factor =
        std::min(spm_performance_factor, (productionRate / inputConsumption));
    spm_performance_factor =
        std::min(spm_performance_factor, (productionRate / outputConsumption));
  }
  obj += 100 * (1 / sched->spmPerformance());

  
  
  for (auto data : sched->ssModel()->subModel()->data_list()) {
    int64_t capacity = data->capacity();
    int64_t capacityUsed = (int64_t) 0;
    
    // Get Software Vertices Mapped To Node
    auto vertices = sched->node_prop()[data->id()].slots[0].vertices;
    for (auto vertex : vertices) {
      if (dfg::Array* arr = dynamic_cast<dfg::Array*>(vertex.first)) {
        // Get the size of the array
        capacityUsed += arr->size();
      } else {
        DSA_CHECK(false) << "Memory node " << data->name() << " has non-array vertex: " << vertex.first->name();
      }
    }

    // We cant have overprovisioned memory
    if (capacityUsed > capacity) {
      succeed_sched = false;
      num_left++;
    }
  }

  return make_pair(succeed_sched - num_left, -obj);
}

bool SchedulerSimulatedAnnealing::length_creep(Schedule* sched, dsa::dfg::Edge* edge, 
                                               int& num, CandidateRoute& undo_routing) {
  bool changed = false;
  return false;
  sched->verify();

  int chances_left = 40;
  while (chances_left-- && num > 0) {
    auto& links = sched->links_of(edge);

    // Check conditions where we can't do length creep
    if (links.empty()) {
      break;
    }
    
    // Get a random link
    int rand_link_no = rand() % links.size();
    auto link_it = links.begin() + rand_link_no;
    auto link = *link_it;
    
    // Get the node that we are path lengthening from
    ssnode* node = link.second->sink();
    // Make sure this node is a switch, as we can only path lengthen from switches
    if (node->type() != ssnode::NodeType::Switch) {
      continue;
    }

    DSA_CHECK(link.first % link.second->granularity() == 0);

    auto& edge_list = sched->edge_list(link.first / link.second->granularity(), link.second);

    // TODO: Extend to include alternate edges
    if (edge_list.size() > 1) {
      continue;
    }

    // Create a Vertex Prop to route to and from
    mapper::VertexProp vp;
    vp.slot.lane_no = link.first / node->granularity();
    vp.slot.ref = node;
    
    DSA_LOG(ROUTE) << "creep " << link.first << ", " << link.second->name() << " for edge: " << edge->name();
    
    // Route with Path lengthening
    int inserted = route(sched, edge, vp, vp, rand_link_no + 1, num + 1);
    sched->verify();
    
    // If routing is successful
    if (inserted) {
      num -= inserted;
      chances_left++;
      changed = true;
    }
  }
  return changed;
}

std::pair<int64_t, int64_t> SchedulerSimulatedAnnealing::obj_creep(Schedule*& sched,
                                                           SchedStats& s,
                                                           CandidateRoute& undo_routing) {
  int num_left = sched->num_left();
  std::pair<int64_t, int64_t> curScore = obj(sched, s);

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
            DSA_LOG(CREEP) << e->name() << ": " << vio;
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
 * workloads which are stored in the scheduling table
 */
bool SchedulerSimulatedAnnealing::incrementalSchedule(CodesignInstance& inst, int stopping) {
  // need to make sure the scheduler has the right submodel
  _ssModel = inst.ss_model();

  // Just for fun, lets see if we can:
  // 1. create a deep copy of the sub model
  // 2. create a copy of the schedule
  // 3. make the schedules consistent with the new deep copy
  // 4. try to schedule something on it.
  // 5. profit?

  is_dse = true;
  bool succeed = true;
  int num_workers = dsa::ContextFlags::Global().num_schedule_workers;
  if (num_workers > 1) {
    ctpl::thread_pool workers(num_workers);
    DSA_INFO << "Workers: " << num_workers;

    for (WorkloadSchedules& ws : inst.workload_array) {
      bool workload_succeed = false;
      for (Schedule& sr : ws.sched_array) {
        workers.push([this, &sr, &stopping, &workload_succeed](int id) {
          Schedule* sched = &sr;
          DSA_CHECK(sched->ssModel() == _ssModel);

          SchedStats s;
          DSA_INFO << "### Start Schedule (" 
                    << sched->ssdfg()->filename << "): (Thread "
                    << id << ") ###";

          bool local_succeed = schedule(sched->ssdfg(), sched, stopping);
          if (local_succeed)
            workload_succeed = true;

          auto p = obj(sched, s);
          
          std::stringstream endStream;
          DSA_INFO << "### End Schedule (" << sched->ssdfg()->filename
                    << "): " << -p.second << " (" << s.ovr << "," << s.latmis << "," << sched->num_links_mapped() << ")  ( Thread " << id << ") ###";
        });
      }
      if (!workload_succeed)
        succeed = false;
    }
    
    workers.stop(true);
  } else {
    for (WorkloadSchedules& ws : inst.workload_array) {
      bool workload_succeed = false;
      for (Schedule& sr : ws.sched_array) {
        Schedule* sched = &sr;
        DSA_CHECK(sched->ssModel() == _ssModel);

        SchedStats s;
        DSA_INFO << "### Start Schedule (" << 
          sched->ssdfg()->filename << "): ###";

        bool local_succeed = schedule(sched->ssdfg(), sched, stopping);
        if (local_succeed)
            workload_succeed = true;

        auto p = obj(sched, s);

        DSA_INFO << "### End Schedule (" << sched->ssdfg()->filename
                    << "): " << -p.second << " (" << s.ovr << "," << s.latmis << "," << sched->num_links_mapped() << ") ###";
      }
      if (!workload_succeed)
        succeed = false;
    }
  }
  return succeed;
}

bool SchedulerSimulatedAnnealing::schedule(SSDfg* ssDFG, Schedule*& sched, int stopping) {
  initialize(ssDFG, sched);  // initialize if null, otherwise its fine
  auto pdgname = basename(ssDFG->filename);
  auto modelname = basename(_ssModel->filename);
  if (!check_feasible(sched->ssdfg(), sched->ssModel())) {
    DSA_LOG(MAP) << "Cannot be mapped, give up!\n";
    return false;
  }

  int max_iters_no_improvement = _ssModel->subModel()->node_list().size() * 50;

  Schedule* cur_sched = new Schedule(getSSModel(), ssDFG);
  *cur_sched = *sched;

  std::pair<int64_t, int64_t> best_score = make_pair(0, 0);
  bool best_succeeded = false;
  bool best_mapped = false;

  int last_improvement_iter = 0;

  //int _best_lat = MAX_ROUTE;
  //int _best_violation = MAX_ROUTE;

  int presize = ssDFG->type_filter<dsa::dfg::Instruction>().size();

  int iter = 0;
  int fail_to_route = 0;
  int max_iters = dsa::ContextFlags::Global().max_iters;
  if (stopping > 0) {
    max_iters = stopping;
  }
  int timeout = dsa::ContextFlags::Global().timeout;
  bool verbose = dsa::ContextFlags::Global().verbose;

  set_start_time();
  for (iter = 0; iter < max_iters; ++iter) {

    // okay so new port has same name..
    // indirect_map_to_index.clear();
    if ((total_msec() > timeout * 1000) || _should_stop) {
      break;
    }

    bool print_stat = (iter & (256 - 1)) == 0;

    // if we don't improve for some time, lets reset
    if (iter - last_improvement_iter > 1024 || fail_to_route > 32) {
      *cur_sched = *sched;
    }

    int status = schedule_internal(ssDFG, cur_sched);
    
    if (dsa::ContextFlags::Global().dummy && status) {
      *sched = *cur_sched;  // shallow copy of sched should work?
      return true;
    }
    if (status == 0) {
      DSA_LOG(MAP) << "Insufficient candidates!";
      return false;
    }
    if (status == -1) {
      ++fail_to_route;
      DSA_LOG(ROUTING) << "Problem with Topology -- Mapping Impossible";
      continue;
    }

    fail_to_route = 0;

    bool succeed_sched = cur_sched->is_complete<dsa::dfg::Node*>();

    SchedStats s;
    std::pair<int64_t, int64_t> score = obj(cur_sched, s);

    int succeed_timing = (s.latmis == 0) && (s.ovr == 0);

    if (verbose && ((score > best_score) || print_stat)) {

      for (auto& elem : ssDFG->type_filter<dsa::dfg::InputPort>()) {
        std::cout << cur_sched->vecPortOf(&elem) << " ";
      }

      std::cout << "|";
      for (auto& elem : ssDFG->type_filter<dsa::dfg::OutputPort>()) {
        std::cout << cur_sched->vecPortOf(&elem) << " ";
      }


      if (score > best_score) {

      fprintf(stdout,
              "Iter: %4d, time:%0.2f, kRPS:%0.1f, left: %3d, "
              "lat: %ld, vio: %d, mis: %ld, ovr: %ld, agg_ovr: %ld, util: %ld, "
              "obj: %ld, ins: %d/%d, outs: %d/%d,"
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
          std::cout << std::endl;
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

    // if (((iter - last_improvement_iter) > max_iters_no_improvement)) {
    //   break;
    // }

    if (best_succeeded) {
      break;
    }
  }

  double seconds = total_msec() / 1000.0;
  sched->scheduled_seconds = seconds;

  if (verbose) {
    DSA_INFO << "Breaking at Iter " << iter << " after " << seconds << " seconds";
  }

  if (cur_sched) {
    delete cur_sched;
  }

  return best_mapped;
}

struct CandidateFinder : dfg::Visitor {
  CandidateFinder(Schedule* sched_) : sched(sched_) {
    DSA_CHECK(sched->ssModel())
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
  bool dummy = dsa::ContextFlags::Global().dummy;
  dsa::mapper::CandidateSpotVisitor cpv(sched, 50);

  // Sort nodes based upon number of candidates for each of them
  std::sort(nodes.begin(), nodes.end(), [sched](dsa::dfg::Node* a, dsa::dfg::Node* b) {
    return sched->candidate_cnt[a->id()] < sched->candidate_cnt[b->id()];
  });
  
  // Shuffle nodes with same number of candidates
  int from = 0;
  for (int i = 1; i < n; ++i) {
    if (sched->candidate_cnt[nodes[i - 1]->id()] !=
        sched->candidate_cnt[nodes[i]->id()]) {
      std::random_shuffle(nodes.begin() + from, nodes.begin() + i);
      from = i;
    }
  }
  std::random_shuffle(nodes.begin() + from, nodes.begin() + n);

  // Log the previous node candidate counts
  for (int i = 0; i < n; ++i) {
    DSA_LOG(CAND) << i << " node ("
                  << nodes[i]->name() 
                  << ") has " << sched->candidate_cnt[nodes[i]->id()]
                  << " candidate(s)";
  }

  for (int i = 0; i < 3; ++i) {
    bool success = true;
    for (int j = 0; j < n; ++j) {
      dsa::dfg::Node* node = nodes[j];
      if (dummy && !dynamic_cast<dsa::dfg::VectorPort*>(node)) {
        continue;
      }
      if (!sched->is_scheduled(node)) {
        node->Accept(&cpv);
        auto& candidates = cpv.candidates[node->id()];
        DSA_LOG(CAND) << i << " node (" << node->name() << ") has " 
                      << candidates.size() << " candidates";
        if (candidates.empty()) {
          DSA_LOG(CAND) << ssDFG->filename << ": " << "Cannot map " << node->name();
          success = false;
          break;
        }
        // if this node name is already scheduled, assign this new node also to previous ssnode
        // assign node should just copy the properties of the hardware ssnode to the logical node
        // no need to call the function below
        // FIXME: @vidushi: this works but I am not sure when we should clear... it delays convergence..
        if (0) { // indirect_map_to_index.find(node->name())!=indirect_map_to_index.end()) {
          auto it = indirect_map_to_index.find(node->name());
          sched->assign_node(node, it->second);
        } else {
          int best_candidate = try_candidates(candidates, sched, node);
          if (best_candidate == -1) {
            unmap_some(ssDFG, sched);
            std::random_shuffle(nodes.begin(), nodes.end());
            success = false;
            break;
          }
        }
      }
      DSA_CHECK(sched->locationOf(node).node());
    }
    if (dummy && success) {
      return 1;
    }
  }
  if (sched->is_complete<dsa::dfg::Node*>()) {
    return 1;
  } else {
    DSA_LOG(COMPLETE) << "Not Completely Mapped!";
    DSA_LOG(COMPLETE) << "Input Ports Complete: " << 
                         sched->is_complete<dsa::dfg::InputPort>()
                      << " Output Ports Complete: " << 
                         sched->is_complete<dsa::dfg::OutputPort>()
                      << " Instructions: " <<
                         sched->is_complete<dsa::dfg::Instruction>();
    return -1;  
  }
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

/**
 * @brief Heuristic to calculate the distance from a node to the destination
 * 
 * 
 * @param edge 
 * @param from_bit 
 * @param dest_bit 
 * @param link 
 * @param sched 
 * @param dest 
 * @return int 
 */
int SchedulerSimulatedAnnealing::routing_cost(dsa::dfg::Edge* edge, 
                                              int from_bit, int dest_bit,
                                              sslink* link,
                                              Schedule* sched,
                                              const pair<int, ssnode*>& dest) {
  // TODO(@were): Do I have a better way to implement this?
  dsa::dfg::Node* def_dfgnode = edge->def();
  dsa::dfg::Node* use_dfgnode = edge->use();
  auto link_prop = sched->link_prop()[link->id()];

  bool is_in = def_dfgnode->type() == dsa::dfg::Node::V_INPUT;
  bool is_out = use_dfgnode->type() == dsa::dfg::Node::V_OUTPUT;
  bool is_io = is_in || is_out;

  bool is_temporal = def_dfgnode->is_temporal();
  bool is_temporal_inst = is_temporal && !is_io;
  bool is_temporal_in = is_temporal && is_in;
  bool is_temporal_out = is_temporal && is_out;

  int internet_dis = abs(from_bit - dest.first) / link->granularity();
  int num_of_slots = link_prop.slots.size();
  internet_dis = min(internet_dis, std::max(num_of_slots - internet_dis, 1));
  DSA_CHECK(internet_dis >= 0);

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
        make_pair(dest_bit, link), def_dfgnode,
        dynamic_cast<dsa::dfg::OutputPort*>(use_dfgnode));
  } else {  // NORMAL CASE!
    t_cost = sched->routing_cost(make_pair(dest_bit, link), edge);
  }

  if (t_cost >= 2) {  // square law avoidance of existing routes
    if (!is_temporal_inst) {
      return (t_cost) * (t_cost) * 10;
    }
  }

  bool is_dest = (next == dest.second && dest_bit == dest.first);

  ssfu* fu = dynamic_cast<ssfu*>(next);
  if (fu && !is_dest) {
    return -1;
    t_cost += 10;
    if (!sched->dfg_nodes_of(dest_bit / fu->granularity(), fu).empty()) {
      return -1;
    }
    if (!sched->node_prop()[fu->id()].slots[dest_bit / fu->granularity()].passthrus.empty()) {
      return -1;
    }
    // int count = sched->dfg_nodes_of(next_slot, fu).size() +
    //             sched->node_prop()[fu->id()].slots[next_slot].passthrus.size();
    t_cost += 200;
  }

  t_cost += internet_dis;
  return t_cost;
}

void inputCreep(int &currentBit, int &currentLink, int value, dfg::InputPort* input, ssivport* vport) {

  // Get the current link
  sslink* link = vport->out_links()[currentLink];
  
  // The number of lanes needed to map
  int lanes = input->vectorLanes();
  // The bitwidth of each lane
  int bitwidth = input->bitwidth();
  int startValue = value;

  // If needed add in the bits for the stated part of the link
  if (vport->vp_stated()) {
    if (input->stated) {
      if (value == 0) {
        return;
      } else {
        value--;
      }
    }
    currentLink++;
    DSA_CHECK(currentLink < vport->out_links().size());
    link = vport->out_links()[currentLink];
  }

  // Creep through all the lanes
  for (; value >= 0; --value) {
    while (link->bitwidth() < currentBit + bitwidth) {
      currentLink++;
      currentBit = 0;
      DSA_CHECK(currentLink < vport->out_links().size()) << "currentLink: " << currentLink << " out_links.size(): " << vport->out_links().size() << " startValue: " << startValue;
      link = vport->out_links()[currentLink];
    }
    currentBit += bitwidth;
  }

  currentBit -= bitwidth;
  return;
}

void outputCreep(int &currentBit, int &currentLink, int value, dfg::OutputPort* output, ssovport* vport) {

  // Get the current link
  sslink* link = vport->in_links()[currentLink];
  bool stated = output->penetrated_state >= 0;

  // The number of lanes needed to map
  int lanes = output->vectorLanes();
  // The bitwidth of each lane
  int bitwidth = output->bitwidth();

  int startValue = value;

  // If needed add in the bits for the stated part of the link
  if (vport->vp_stated()) {
    if (stated) {  
      if (value == 0) {
        return;
      } else {
        value--;
      }
    }
    currentLink++;
    DSA_CHECK(currentLink < vport->in_links().size());
    link = vport->in_links()[currentLink];
  }

  // Creep through all the lanes
  for (; value >= 0; --value) {
    while (link->bitwidth() < currentBit + bitwidth) {
      currentLink++;
      currentBit = 0;
      DSA_CHECK(currentLink < vport->in_links().size());
      link = vport->in_links()[currentLink];
    }
    currentBit += bitwidth;
  }

  currentBit -= bitwidth;
  return;
}

/**
 * @brief Insert a link into an edge
 * 
 * @param link the link to insert
 * @param sched the schedule that contains edge
 * @param edge the edge to insert the link into
 * @param it position to insert the link
 * @param dest the final link in the edge (to check for passthrus)
 */
void insert_edge(std::pair<int, sslink*> link, 
                 Schedule* sched, 
                 dsa::dfg::Edge* edge,
                 std::vector<std::pair<int, sslink*>>::iterator it,
                 std::pair<int, ssnode*> dest) {
  sched->assign_edgelink(edge, link.first, link.second, it);
  
  std::pair<int, ssnode*> next_node = std::make_pair(link.first, link.second->sink());
  if ((dest != next_node)) {
    sched->assign_edge_pt(edge, next_node);
  }
}

/**
 * @brief The Routing Function
 * 
 * The function first performs Djikstra's algorithm to find the shortest path
 * from the source to the destination. Then, it assigns the shortest path
 * to the edge object.
 * 
 * 
 * @param sched the schedule to use
 * @param edge the edge to route
 * @param vps source vertex
 * @param vpd end vertex
 * @param ins_it iterator for where to insert
 * @param max_path_lengthen maximum path length
 * @return int latency of the route
 */

int SchedulerSimulatedAnnealing::route(
    Schedule* sched, 
    dsa::dfg::Edge* edge,
    mapper::VertexProp &vps,
    mapper::VertexProp &vpd,
    int insert_position, 
    int max_path_lengthen) {

  bool path_lengthen = insert_position != 0;

  // Set the source and destination nodes
  std::pair<int, ssnode*> source = {vps.lane() * vps.node()->granularity(), vps.node()};
  std::pair<int, ssnode*> dest = {vpd.lane() * vpd.node()->granularity(), vpd.node()};

  if (path_lengthen) {
    DSA_CHECK(source.second->type() != ssnode::InputVectorPort);
    DSA_CHECK(dest.second->type() != ssnode::OutputVectorPort);
  }

  // Can't have route between same node
  // TODO: Rethink this? Maybe we can have the same node be routed between
  if (source.second == dest.second && !path_lengthen) {
    return false;
  }


  DSA_LOG(ROUTE) << "Routing edge " << edge->name() << " (ID:" << edge->id << ") from "
                 << source.second->name() << "(" << source.first << ")"
                 << " to " << dest.second->name() << "(" << dest.first << ")";

  // Declaration of vectors used in Djikstra's algorithm
  int n_nodes = sched->ssModel()->subModel()->node_list().size();
  std::vector<std::vector<int>> node_dist(n_nodes);
  std::vector<std::vector<int>> done(n_nodes);
  std::vector<std::vector<std::pair<int, sslink*>>> came_from(n_nodes);

  // Booleans for checking at end if we need to add in edge
  std::pair<int, sslink*> removed_input_vport{-1, nullptr};
  std::pair<int, sslink*> removed_output_vport{-1, nullptr};

  // Initialize the vectors used for Djikstra's algorithm
  for (ssnode* n : sched->ssModel()->subModel()->node_list()) {
    node_dist[n->id()] = std::vector<int>(n->lanes(), -1);
    came_from[n->id()] = std::vector<std::pair<int, sslink*>>(n->lanes(), {-1, nullptr});
    done[n->id()] = std::vector<int>(n->lanes(), false);
  }

  
  /* For Some Reason, this fails in python
  if (!path_lengthen) {
    DSA_INFO << routing_times;
    ++routing_times;
  } */

  // Special case where source and destination are the same
  if (source == dest && !path_lengthen) {
    return 1;
  }

  DSA_CHECK(path_lengthen || sched->link_count(edge) == 0)
      << "Edge: " << edge->name() << " is already routed!";

  // Declaration of the openset, used for dijkstra's algorithm
  // Distance, random priority, bit, link
  set<std::tuple<int, int, int, sslink*>> openset;

  int new_rand_prio = 0;

  
  // Check to see if source is an Input Vector Port, then we statically assign
  if (auto vport = dynamic_cast<ssivport*>(source.second)) {
    auto* inNode = dynamic_cast<dsa::dfg::InputPort*> (edge->def());
    // Get the value of this edge
    int value = edge->vid;

    // Set initial parameters
    int currentBit = 0;
    int currentLink = 0;
    
    // Get the bit and link of next inputVectorPort
    inputCreep(currentBit, currentLink, value, inNode, vport);
    
    // get the next link
    std::pair<int, ssnode*> next_source = {currentBit, vport->out_links()[currentLink]->sink()};
    
    // Set the removed link so that it can be referenced later
    removed_input_vport = {currentBit, vport->out_links()[currentLink]};

    // Set the source bit to use when printing edge
    sched->edge_prop()[edge->id].source_bit = currentBit;

    // Check to make sure that it follows granularity constraint
    if (currentBit % vport->out_links()[currentLink]->granularity() != 0) {
      return 0;
    }
    
    source = next_source;
  } else {

    // Set the source bit to use when printing edge
    sched->edge_prop()[edge->id].source_bit = source.first;
  }

  
  // Check if destination is an output Vector Port
  if (auto vport = dynamic_cast<ssovport*>(dest.second)) {
    // Get output dfg node
    dsa::dfg::OutputPort* outNode = dynamic_cast<dsa::dfg::OutputPort*> (edge->use());

    // get the slot that this edge must be mapped to
    int sourceIdx = edge->oid;
    int currentBit = 0;
    int currentLink = 0;

    // Get the bit and link of next output link
    outputCreep(currentBit, currentLink, sourceIdx, outNode, vport);

    // get the next link
    std::pair<int, ssnode*> next_dest = {currentBit, vport->in_links()[currentLink]->source()};

    // Set the removed link so that it can be referenced later
    removed_output_vport = {currentBit, vport->in_links()[currentLink]};

    // Make sure that it follows granularity constraint
    if (currentBit % vport->in_links()[currentLink]->granularity() != 0) {
      return 0;
    }

    // set new destination
    dest = next_dest;
  }

  // Check to see if unaligned functional unit
  if (source.second->type() == ssnode::NodeType::FunctionUnit) {
    if (source.first % edge->bitwidth() != 0) {
      return false;
    }
  }

  // Check to see if unaligned functional unit
  if (dest.second->type() == ssnode::NodeType::FunctionUnit) {
    if (dest.first % edge->bitwidth() != 0) {
      return false;
    }
  }

  // Make sure the source is aligned
  if (source.first % source.second->granularity() != 0) {
    return false;
  }

  // Make sure the destination is aligned
  if (dest.first % dest.second->granularity() != 0) {
    return false;
  }

  // Special Case where stated makes path shorter and become the same node
  if (source == dest && !path_lengthen) {
    int count = 0;

    // First check if the output port was removed
    if (removed_output_vport.second != nullptr) {
      dest = {removed_output_vport.first, removed_output_vport.second->sink()};
      insert_edge(removed_output_vport, sched, edge, sched->links_of(edge).end(), dest);
      count++;
    }

    // Second check if the input port was removed
    if (removed_input_vport.second != nullptr) {
      insert_edge(removed_input_vport, sched, edge, sched->links_of(edge).begin(), dest);
      count++;
    }

    return std::max(count, 1);
  } else if (!path_lengthen && source.second == dest.second) {
    // If they have the same slot but not aligned, then we need to return false
    if (source.first / source.second->granularity() == dest.first / dest.second->granularity()) {
      return false;
    }
  }

  // Add all the information for the source node
  done[source.second->id()][source.first / source.second->granularity()] = new_rand_prio;
  node_dist[source.second->id()][source.first /  source.second->granularity()] = 0;
  came_from[source.second->id()][source.first /  source.second->granularity()] = std::make_pair(0, nullptr);

  if (path_lengthen) {
    auto link_to_insert = sched->links_of(edge).begin() + (insert_position - 1);
    DSA_CHECK(link_to_insert->second->sink() == source.second);
    openset.insert(std::make_tuple(0, new_rand_prio, source.first, link_to_insert->second));
  } else {
    if (removed_input_vport.second != nullptr) {
      openset.insert(std::make_tuple(0, new_rand_prio, source.first, removed_input_vport.second));
    } else {
      DSA_CHECK(source.second->type() != ssnode::Switch);
      // TODO: Make it so that this link is always correct. Currently relying on connectivity table of non-links to be just the same slot
      if (source.second->in_links().size() == 0) {
        return 0;
      }
      openset.insert(std::make_tuple(0, new_rand_prio, source.first, source.second->in_links()[0]));
    }
  }
  // Log the routing information
  DSA_LOG(ROUTE) << "Routing from " << source.second->name() << " (B:" << source.first << ") to " << dest.second->name() << " (B:" << dest.first << ")";

  // Stash if the edge is a memory edge
  bool memEdge = edge->memory();
  int num_visited = 0;

  // Begin Djikstra's algorithm
  while (!openset.empty()) {
    // Get items from the openset
    auto &front_elem = *openset.begin();
    int dist = std::get<0>(front_elem);
    int rand_prio = std::get<1>(front_elem);
    int bit = std::get<2>(front_elem);
    sslink* link = std::get<3>(front_elem);
    ssnode* node = link->sink();
    DSA_CHECK(bit % node->granularity() == 0) << bit << " " << node->granularity() << " " << node->name();

    // Erase the front element of openset
    openset.erase(openset.begin());

    // Check if we have reached the destination
    if (!path_lengthen) {
      if (bit == dest.first && node == dest.second) {
        break;
      }
    } else {
      if ((bit == dest.first && node == dest.second && dist > 0) ||
          (dist > max_path_lengthen)) {
        break;
      }
    }

    // Iterate through out-links of the node
    for (auto out_link : node->out_links()) {
      
      // Check if the edge is a memory edge
      if (memEdge) {
        if (out_link->sink()->spatial() || out_link->source()->spatial()) {
          continue;
        } 
      } else {
        if (out_link->sink()->data() || out_link->source()->data()) {
          continue;
        }
      }

      int slotSize = std::ceil(edge->bitwidth() / (float) out_link->granularity());

      // Get list of slots that this edge can be mapped to
      int64_t slots = node->slots(link, out_link, bit / out_link->granularity(), slotSize);

      //DSA_INFO << " Routing Table" << node->routing_table(link, out_link)[0];

      //DSA_INFO << "  " << link->name() << " - " << out_link->name() << " can be mapped to " << slots << " with bit " << bit / out_link->granularity() << " and slot " << edge->bitwidth() / out_link->granularity();

      // Iterate through the slots
      while (slots) {
        // Get the next link and node
        sslink* next_link = out_link;
        ssnode* next_node = next_link->sink();

        // Get the next slot
        int raw = slots & -slots;
        slots -= raw;
        int next_bit = ((31 - __builtin_clz(raw)) % next_node->lanes()) * next_link->granularity();

        int next_slot = (next_bit / next_node->granularity()) % next_node->lanes();

        // Restrict the places where the slots can map to
        if (next_link->sink()->type() == ssnode::NodeType::FunctionUnit) {
          if (next_bit % edge->bitwidth() != 0) {
            continue;
          } else if (next_bit + edge->bitwidth() > next_node->datawidth()) {
            continue;
          }
        }

        // We have to be on the first spot of the link and node
        if (next_bit % next_node->granularity() != 0 || next_bit % next_link->granularity() != 0) {
          continue;
        }

        // Declare the next_pair to be placed in schedule
        std::pair<int, sslink*> next_pair(next_bit, next_link);

        // Calculate the Route Cost, used as a heuristic for shortest path
        int route_cost = routing_cost(edge, bit, next_bit, next_link, sched, dest);
        
        if (path_lengthen || !this->allow_overprov) {
          // Only route on open links, ensures no additional overprovisioning
          if (!(next_node != dest.second && next_slot != dest.first)) {
            // For path lengthening, only route on free spaces
            int start = bit / next_link->granularity();
            int end = start + std::ceil(edge->bitwidth() / (float) next_link->granularity());
            auto link_prop = sched->link_prop()[next_link->id()];
            for (int i = start; i < end; i++) {
              int slot = i % next_link->lanes();
              auto linkslot = link_prop.slots[slot];
              
              if (linkslot.edges.size() == 0)
                continue;

              auto value_edge = sched->ssdfg()->edges[linkslot.edges[0].eid];
              std::pair<dsa::dfg::Value*, int> value = std::make_pair(value_edge.val(), value_edge.l);
              std::pair<dsa::dfg::Value*, int> current_value = std::make_pair(edge->val(), edge->l);
              if (value != current_value) {
                route_cost = -1;
                break;
              }
            }
          }
        }

        // Means that this node is not able to route through
        if (route_cost == -1) {
          continue;
        }

        // The different distances for the next node
        int next_dist = dist + route_cost;
        int prev_dist = node_dist[next_node->id()][next_slot];

        // Check if this route is better than the previous one, or is first to reach node
        if (prev_dist == -1 || prev_dist > next_dist ||
            (path_lengthen && make_pair(next_bit, next_node) == dest)) {
          
          // If its better than the previous one, remove old one
          if (prev_dist != -1) {
            int next_priority = done[next_node->id()][next_slot];

            auto iter =
                openset.find(std::make_tuple(prev_dist, next_priority, next_bit, next_link));
            
            if (iter != openset.end()) {
              openset.erase(iter);
            }
          }

          // Set the priority
          int priority = 0;
          if (this->deterministic) {
            priority = num_visited;
            num_visited++;
          } else {
            priority = rand() % 16;
          }

          done[next_node->id()][next_slot] = priority;


          // Add this node to the matrix
          openset.emplace(next_dist, priority, next_bit, next_link);

          // Set the final matrices
          node_dist[next_node->id()][next_slot] = next_dist;
          came_from[next_node->id()][next_slot] = std::make_pair(bit, next_link);
        }
      }
    }
  }

  // Check to see if theres a valid route to the destination
  if (node_dist[dest.second->id()][dest.first / dest.second->granularity()] == -1 ||
      (path_lengthen &&
       node_dist[dest.second->id()][dest.first / dest.second->granularity()] == 0)) {
    DSA_LOG(ROUTE) << "No route found";

    return false;  // routing failed, no routes exist!
  }

  // There is a route!
  DSA_LOG(ROUTE) << "Route Found!";

  // Start with destination link
  int count = 1;
  dsa::dfg::Edge* alt_edge = nullptr;

  pair<int, sslink*> current_link;
  std::pair<int, ssnode*> last_node = dest;

  // This means that we had an output vector port at beginnning of edge
  if (removed_output_vport.second != nullptr) {
    dest = {removed_output_vport.first, removed_output_vport.second->sink()};
    insert_edge(removed_output_vport, sched, edge, sched->links_of(edge).end(), dest);
    count++;
  }

  while (last_node != source || (path_lengthen && count == 1)) {
    DSA_CHECK(last_node.first / last_node.second->granularity() < last_node.second->lanes()) << last_node.first << " " << last_node.second->granularity() << " " << last_node.second->name() << " " << last_node.second->lanes() << " source: " << source.first << " " << source.second->name() << " " << source.second->lanes() << " dest: " << dest.first << " " << dest.second->name() << " " << dest.second->lanes();
    current_link = came_from[last_node.second->id()][last_node.first / last_node.second->granularity()];
    // save the slot
    auto link_backup = current_link;
    
    // Change the slot to be the previous slot
    current_link.first = last_node.first;

    // Check if theres an alternate edge
    if (alt_edge = sched->alt_edge_for_link(current_link, edge)) {
      if (!path_lengthen)
        break;
    }
    
    // Insert link into the edge
    insert_edge(current_link, sched, edge, sched->links_of(edge).begin() + insert_position, dest);

    last_node = std::make_pair(link_backup.first, current_link.second->source());
    count++;
  }


  // Now we need to change the edge
  if (alt_edge && !path_lengthen) {
    auto& alt_links = sched->links_of(alt_edge);
    for (auto alt_link : alt_links) {
      insert_edge(alt_link, sched, edge, sched->links_of(edge).begin() + insert_position, dest);
      insert_position++;
      count++;
      
      // Check to see if we hit the final link
      if (alt_link == current_link)  {
        break;
      }
    }
  }
  
  if (removed_input_vport.second != nullptr && !alt_edge) {
    insert_edge(removed_input_vport, sched, edge, sched->links_of(edge).begin(),dest);
    count++;
  }
 
  DSA_CHECK(count != false); 
  return count;
}

// Assign a node, indices and hardware port from here...
bool SchedulerSimulatedAnnealing::scheduleHere(Schedule* sched, dsa::dfg::Node* node, mapper::Slot<ssnode*> here) {
  // TODO: @vidushi: we do not need to route if the same name port (ie. source) was routed earlier
  // if(node->indirect()) {
  //   // if input is indirect, then create a mapping for the output node with the same name (search)
  //   // if output is indirect, skip!!
  //   printf("Found an indirect node, just assigned it to the same id\n");
  //   // mapping to the hardware
  //   sched->assign_node(node, here);
  //   return true;
  // } else {
  //   printf("Found a normal node, just assigned it to the same id\n");
  // }
  std::vector<dsa::dfg::Edge*> to_revert;

#define process(edge_, node_, src, dest)                              \
  do {                                                                \
    auto& edges = edge_;                                              \
    int n = edge_.size();                                             \
    for (int i = 0; i < n; ++i) {                                     \
      auto edge = edges[i];                                           \
      DSA_CHECK(sched->link_count(edge) == 0)                         \
          << "Edge: " << edge->name() << " is already routed!\n";     \
      auto n = edge->node_();                                         \
      if (sched->is_scheduled(n)) {                                   \
        auto loc = sched->locationOf(n);                              \
        if (!route(sched, edge, src, dest, 0, 0)){                                                             \
          DSA_LOG(ROUTE)                                              \
            << "Cannot route [" << edge->id << ", " << edge->name()   \
            << "] " << src.node()->id()                               \
            << " -> " << dest.node()->id() << ", dist: "              \
            << sched->distances[src.node()->id()][dest.node()->id()]; \
          for (auto revert : to_revert) sched->unassign_edge(revert); \
          for (int j = 0; j < i; ++j) sched->unassign_edge(edges[j]); \
          sched->unassign_dfgnode(node);                              \
          return false;                                               \
        }                                                             \
      }                                                               \
    }                                                                 \
  } while (false)

  sched->assign_node(node, {here.lane_no, here.ref});
  auto here_ = sched->locationOf(node);
  do {
    auto edge_ = sched->operands[node->id()];
    auto& edges = edge_;
    int n = edge_.size();
    for (int i = 0; i < n; ++i) {

      auto edge = edges[i];
      DSA_CHECK(sched->link_count(edge) == 0)
        << "Edge [" << edge->id << "]: "  << edge->name() << " is already routed!\n";
      auto n = edge->def();

      if (sched->is_scheduled(n)) {
        auto loc = sched->locationOf(n);
        if (!route(sched, edge, loc, here_, 0, 0)) {
          DSA_LOG(ROUTE) << "Cannot route [" << edge->id << ", " << edge->name() << "] "
                         << loc.node()->id() << " -> " << here_.node()->id() << ", dist: "
                         << sched->distances[loc.node()->id()][here_.node()->id()];
          
          for (auto revert : to_revert) sched->unassign_edge(revert);
          for (int j = 0; j < i; ++j) sched->unassign_edge(edges[j]);
          sched->unassign_dfgnode(node);

          return false;
        }
      }
    }
  } while (false);

  //process(sched->operands[node->id()], def, loc, here_);
  to_revert = sched->operands[node->id()];
  // TODO: @vidushi: we do not need to route if the same name port (ie. source) was routed earlier
  //process(sched->users[node->id()], use, here_, loc);
  //DSA_INFO << "Schedule Here Second!";
  do {
    auto edge_ = sched->users[node->id()];
    auto& edges = edge_;
    int n = edge_.size();
    for (int i = 0; i < n; ++i) {
      auto edge = edges[i];
      DSA_CHECK(sched->link_count(edge) == 0) << "Edge [" << edge->id << "]: " << edge->name() << " is already routed!\n";
      auto n = edge->use();

      if (sched->is_scheduled(n)) {
        auto loc = sched->locationOf(n);
        if (!route(sched, edge, here_, loc, 0, 0)) {
          DSA_LOG(ROUTE) << "Cannot route [" << edge->id << ", " << edge->name() << "] "
                         << here_.node()->id() << " -> " << loc.node()->id() << ", dist: "
                         << sched->distances[here_.node()->id()][loc.node()->id()];
          for (auto revert : to_revert) sched->unassign_edge(revert);
          for (int j = 0; j < i; ++j) sched->unassign_edge(edges[j]);
          sched->unassign_dfgnode(node);
          return false;
        }
      }
    }
  } while (false);

#undef process


  return true;
}

// okay so this function is trying among candidates for this node...
// for any output node, it should discard all *indirect* ports
// for the indirect port, chose the candidate which is the same as its corresponding indirect port
int SchedulerSimulatedAnnealing::try_candidates(
    const std::vector<mapper::MapSpot>& candidates, Schedule* sched,
    dsa::dfg::Node* node) {
  DSA_LOG(MAP) << "Mapping " << node->name();
  using std::make_pair;
  using std::pair;
  using std::vector;
  CandidateRoute best_path;

  pair<int64_t, int64_t> bestScore = std::make_pair(INT_MIN, INT_MIN);
  int best_candidate = -1;
  bool find_best = rand() % 128;

  if (candidates.empty()) return 0;

  std::vector<int> src, dst, idx, keys;

  for (auto edge : sched->operands[node->id()]) {
    auto loc = sched->locationOf(edge->def());
    if (loc.node()) {
      src.push_back(node->id());
    }
  }
  for (auto edge : sched->users[node->id()]) {
    auto loc = sched->locationOf(edge->use());
    if (loc.node()) {
      dst.push_back(loc.node()->id());
    }
  }
  for (size_t i = 0; i < candidates.size(); ++i) {
    idx.push_back(i);
  }
  for (size_t i = 0; i < candidates.size(); ++i) {
    int sum = 0;
    for (auto elem : src) {
      sum += sched->distances[elem][candidates[i].slot.ref->id()];
    }
    for (auto elem : dst) {
      sum += sched->distances[candidates[i].slot.ref->id()][elem];
    }
    keys.push_back(sum);
  }
  // based on the keys, sort the indices. So we prioritize according to that...
  std::sort(idx.begin(), idx.end(), [&keys](int a, int b) { return keys[a] < keys[b]; });

  int no_imporve = 0;
  // for all ports and instructions whose names are printed
  for (size_t i = 0; i < candidates.size(); ++i) {
    // maybe here, that it will access the next index now..
    DSA_LOG(TRYMAP)
      << "  Try: " << candidates[idx[i]].slot.lane_no << ", " << candidates[idx[i]].slot.ref->name();

    if (scheduleHere(sched, node, candidates[idx[i]].slot)) {
      SchedStats s;
      CandidateRoute undo_path;
      pair<int64_t, int64_t> candScore = obj_creep(sched, s, undo_path);
      DSA_LOG(MAP) << "  Score: " << candScore.first << ", " << candScore.second;

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
    auto &cand = candidates[best_candidate];
    DSA_LOG(MAP) << node->name() << " maps to " << cand.node()->name() << " at slot: " << cand.lane();
    sched->assign_node(node, {cand.lane(), cand.node()});
    // FIXME: is first not the assign port number???? need port for the hardware port
    // auto vport = candidates[best_candidate].second; // ssnode*
    // auto ne = dynamic_cast<SSDfgVec*>(node);
    // indirect_map_to_index.insert(make_pair(node->name(), sched->vecPortOf(ne)));
    // std::cout << "Assigned node: " << node->name() << " id: " << vport->id() << std::endl;
    // indirect_map_to_index.insert(make_pair(node->name(), vport->id()));
    // indirect_map_to_index.insert(make_pair(node->name(), candidates[best_candidate]));
    // DSA_LOG(MAPPING) << node->name() << " maps to "
    //                << candidates[best_candidate].second->name();
  } else {
    DSA_LOG(MAP) << "******No candidate found for " << node->name();
  }

  DSA_LOG(TRYMAP) << "return for best candidate!";
  return best_candidate;
}

void CandidateRoute::apply(Schedule* sched) {
  std::map<std::tuple<int, int, int, int>, int> passthru_values;

  for (auto& edge_prop : edges) {
    sched->unassign_edge(edge_prop.first);  // for partial routes
    DSA_LOG(PASSTHRU) << edge_prop.first->name();
    for (auto elem : edge_prop.second.thrus) {
      sched->assign_edge_pt(edge_prop.first, elem);
    }
    for (auto elem : edge_prop.second.links) {
      DSA_LOG(UNASSIGN) << "Assigning " << elem.second->name() << " to edge [" << edge_prop.first->id << "]: " << edge_prop.first->name();
      sched->assign_edgelink(edge_prop.first, elem.first, elem.second);
    }
  }
}
