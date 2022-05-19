#include <stdio.h>
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

std::pair<int, int> SchedulerSimulatedAnnealing::obj(Schedule*& sched, SchedStats& s) {
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
  double nmlz = std::accumulate(nmlz_freq.begin(), nmlz_freq.end(), 0);
  for (int i = 0; (long unsigned int) i < sched->ssdfg()->meta.size(); ++i) {
    nmlz_freq[i] /= nmlz;
  }

  // ArgSort nmlz frequency
  std::vector<double> nmlz_freq_args(sched->ssdfg()->meta.size(), 0);
  std::size_t n(0);
  std::generate(std::begin(nmlz_freq_args), std::end(nmlz_freq_args), [&]{ return n++; });
  std::sort(  std::begin(nmlz_freq_args), 
              std::end(nmlz_freq_args),
              [&](int i1, int i2) { return nmlz_freq[i1] > nmlz_freq[i2]; } );

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
          inputConsumption += (input_consumtion[i] * nmlz_freq[i] / nmlz_freq[nmlz_freq_args[0]]);
          outputConsumption += (output_consumption[i] * nmlz_freq[i] / nmlz_freq[nmlz_freq_args[0]]);
        }
      }
    }
  
    spm_performance_factor =
        std::min(spm_performance_factor, (productionRate / inputConsumption));
    spm_performance_factor =
        std::min(spm_performance_factor, (productionRate / outputConsumption));
  }
  obj += 100 * (1 / spm_performance_factor);

  
  
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
      mapper::VertexProp vp;
      vp.slot.lane_no = rand_link.first;
      vp.slot.ref = sw;
      DSA_LOG(ROUTE)
        << "creep " << rand_link.first << ", " << rand_link.second->name() << " for edge: " << edge->name();
      int inserted = route(sched, edge, vp, vp, &(++it), num + 1);
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
            auto alt_it = alt_links.begin() + std::min(rand_link_no + 1 + i, (int) alt_links.size());
            sched->assign_edgelink(alt_edge, from_link.first, from_link.second, alt_it);
            if (auto fu = dynamic_cast<ssfu*>(from_link.second->sink())) {
              if (i + 1 != inserted) {
                sched->assign_edge_pt(alt_edge, std::make_pair(from_link.first, fu));
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
 * workloads which are stored in the scheduling tabl
 */
bool SchedulerSimulatedAnnealing::incrementalSchedule(CodesignInstance& inst, int max_vector) {
  // need to make sure the scheduler has the right submodel
  _ssModel = inst.ss_model();

  // Just for fun, lets see if we can:
  // 1. create a deep copy of the sub model
  // 2. create a copy of the schedule
  // 3. make the schedules consistent with the new deep copy
  // 4. try to schedule something on it.
  // 5. profit?

  is_dse = true;
  int num_workers = dsa::ContextFlags::Global().num_schedule_workers;
  if (num_workers > 1) {
    ctpl::thread_pool workers(num_workers);
    DSA_INFO << "Workers: " << num_workers;

    for (WorkloadSchedules& ws : inst.workload_array) {
      for (Schedule& sr : ws.sched_array) {
        if (max_vector != -1 && sr.unrollDegree() > (int) max_vector)
          continue;
        workers.push([this, &sr](int id) {
          Schedule* sched = &sr;
          DSA_CHECK(sched->ssModel() == _ssModel);

          SchedStats s;
          DSA_INFO << "### Start Schedule (" 
                    << sched->ssdfg()->filename << "): (Thread "
                    << id << ") ###";

          schedule(sched->ssdfg(), sched);
          auto p = obj(sched, s);
          
          std::stringstream endStream;
          DSA_INFO << "### End Schedule (" << sched->ssdfg()->filename
                    << "): " << -p.second << " (Thread " << id << ") ###";
        });
      }
    }
    
    workers.stop(true);
  } else {
    for (WorkloadSchedules& ws : inst.workload_array) {
      for (Schedule& sr : ws.sched_array) {
        if (max_vector != -1 && sr.unrollDegree() > (int) max_vector) {
          continue;
        }
        
        Schedule* sched = &sr;
        DSA_CHECK(sched->ssModel() == _ssModel);

        SchedStats s;
        DSA_INFO << "### Start Schedule (" << 
          sched->ssdfg()->filename << "): ###";

        schedule(sched->ssdfg(), sched);
        auto p = obj(sched, s);

        DSA_INFO <<  "### End Schedule (" << sched->ssdfg()->filename << "): " 
                 << -p.second << " ###";
      }
    }
  }
  return true;
}

bool SchedulerSimulatedAnnealing::schedule(SSDfg* ssDFG, Schedule*& sched) {
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

  std::pair<int, int> best_score = make_pair(0, 0);
  bool best_succeeded = false;
  bool best_mapped = false;

  int last_improvement_iter = 0;

  //int _best_lat = MAX_ROUTE;
  //int _best_violation = MAX_ROUTE;

  int presize = ssDFG->type_filter<dsa::dfg::Instruction>().size();

  int iter = 0;
  int fail_to_route = 0;
  int max_iters = dsa::ContextFlags::Global().max_iters;
  int timeout = dsa::ContextFlags::Global().timeout;
  bool verbose = dsa::ContextFlags::Global().verbose;

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
    std::pair<int, int> score = obj(cur_sched, s);

    int succeed_timing = (s.latmis == 0) && (s.ovr == 0);

    if (verbose && ((score > best_score) || print_stat)) {
      stringstream ss;
      ss << "viz/iters/" << iter << "-sched.gv";
      cur_sched->printGraphviz(ss.str().c_str());

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

  if (verbose) {
    DSA_INFO << "Breaking at Iter " << iter;
              //<< ", candidates success / candidates tried:  " << this->candidates_succ
              //<< "/" << this->candidates_tried << std::endl;
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
  int num_of_slots = sched->lanes_involved(link, link->bitwidth());
  // link->bitwidth() / link->source()->granularity();
  internet_dis = min(internet_dis, num_of_slots - internet_dis);

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
        make_pair(next_slot, link), def_dfgnode,
        dynamic_cast<dsa::dfg::OutputPort*>(use_dfgnode));
  } else {  // NORMAL CASE!
    t_cost = sched->routing_cost(make_pair(next_slot, link), edge);
  }

  if (t_cost >= 2) {  // square law avoidance of existing routes
    if (!is_temporal_inst) {
      return (t_cost) * (t_cost)*10;
    }
  }

  bool is_dest = (next == dest.second && next_slot == dest.first);

  ssfu* fu = dynamic_cast<ssfu*>(next);
  if (fu && !is_dest) {
    return -1;
    t_cost += 10;
    if (!sched->dfg_nodes_of(next_slot, fu).empty()) {
      return -1;
    }
    if (!sched->node_prop()[fu->id()].slots[next_slot].passthrus.empty()) {
      return -1;
    }
    // int count = sched->dfg_nodes_of(next_slot, fu).size() +
    //             sched->node_prop()[fu->id()].slots[next_slot].passthrus.size();
    t_cost += 200;
  }

  t_cost += internet_dis;
  return t_cost;
}

/**
 * @brief Insert a link into an edge
 * 
 * @param link the link to insert
 * @param sched the schedule that contains edge
 * @param edge the edge to insert the link into
 * @param it position to insert the link
 * @param dest the final link (to check for passthrus)
 * @param x the current node and slot
 */
void insert_edge(std::pair<int, sslink*> link, 
                 Schedule* sched, 
                 dsa::dfg::Edge* edge,
                 std::vector<std::pair<int, sslink*>>::iterator it,
                 std::pair<int, ssnode*> dest, 
                 std::pair<int, ssnode*> x) {
  DSA_LOG(ROUTE_BT) << link.first << ", " << link.second->name();
  sched->assign_edgelink(edge, link.first, link.second, it);


  if (dynamic_cast<ssfu*>(link.second->sink())) {
      DSA_LOG(PASSTHROUGH) << "CHECKING " << x.second->name() << " with link " << link.second->sink()->name() << " on edge " << edge->id;
    // Dylan: So deleting the second part seems to assign pass-throughs correctly
    // but I don't know if this is the right way to do it.

    if ((dest != x) /*&& !sched->dfgNodeOf(link.first, link.second) */) {
      DSA_LOG(PASSTHROUGH) << "Inserting Passthrough " << x.second->name();
      sched->assign_edge_pt(edge, x);
    }
  }
}

/**
 * @brief Insert a link into an edge
 * 
 * Different in that it doesn't check for passthrus
 * 
 * @param link the link to insert
 * @param sched the schedule that contains edge
 * @param edge the edge to insert the link into
 * @param it position to insert the link
 */
void insert_edge(std::pair<int, sslink*> link, 
                 Schedule* sched, 
                 dsa::dfg::Edge* edge,
                 std::vector<std::pair<int, sslink*>>::iterator it) {
  DSA_CHECK(link.second != nullptr);
  DSA_CHECK(link.second->source() != nullptr);
  DSA_CHECK(link.second->sink() != nullptr);
  
  sched->assign_edgelink(edge, link.first, link.second, it);
}

/**
 * @brief The Routing Function
 * 
 * First perform Djikstra's algorithm to find the shortest path
 * Then insert the shortest path into the edge
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
    std::vector<std::pair<int, sslink*>>::iterator* ins_it, 
    int max_path_lengthen) {

  bool path_lengthen = ins_it != nullptr;

  std::pair<int, ssnode*> source = {vps.lane(), vps.node()};
  std::pair<int, ssnode*> dest = {vpd.lane(), vpd.node()};
  DSA_LOG(ROUTE) << "Routing edge " << edge->name() << " (ID:" << edge->id << ") from "
                 << source.second->name() << "(" << source.first << ")"
                 << " to " << dest.second->name() << "(" << dest.first << ")";

  int n_nodes = sched->ssModel()->subModel()->node_list().size();
  std::vector<std::vector<int>> node_dist(n_nodes);
  std::vector<std::vector<int>> done(n_nodes);
  std::vector<std::vector<std::pair<int, sslink*>>> came_from(n_nodes);

  // Booleans for checking at end if we need to add in edge
  std::pair<int, sslink*> removed_input_vport{-1, nullptr};
  std::pair<int, sslink*> removed_output_vport{-1, nullptr};

  for (ssnode* n : sched->ssModel()->subModel()->node_list()) {
    int node_id = n->id();
    DSA_CHECK(node_id >= 0 && node_id < n_nodes);
    node_dist[node_id] = std::vector<int>(n->lanes(), -1);
    int n_lanes = n->lanes();
    DSA_CHECK(n_lanes > 0) << "N lanes has to be positive, but it is " << n_lanes;
    came_from[node_id] = std::vector<std::pair<int, sslink*>>(n_lanes, {-1, nullptr});
    done[node_id] = std::vector<int>(n->lanes(), false);
  }

  if (!path_lengthen) ++routing_times;

  if (source == dest && !path_lengthen) {
    return 1;
  }

  DSA_CHECK(path_lengthen || sched->link_count(edge) == 0)
      << "Edge: " << edge->name() << " is already routed!";

  // Distance, random priority, slot, node
  set<std::tuple<int, int, int, ssnode*>> openset;

  if (!path_lengthen) source.first = edge->def()->slot_for_use(edge, source.first);
  dest.first = edge->use()->slot_for_op(edge, dest.first);

  int new_rand_prio = 0;

  // Check to see if source is an Input Vector Port
  if (auto vport = dynamic_cast<ssivport*>(source.second)) {
    if (vport->vp_impl() == 2) {
      auto* inNode = dynamic_cast<dsa::dfg::InputPort*> (edge->def());

      // Get the index of the next node to route through
      int slot = edge->vid;
      

      // If we are mapping a non-stated software input port to a stated hardware port
      // then we must reserve the first slot of the input port
      if (vport->vp_stated() && !inNode->stated)
        slot++;
      
      // Sanity Check
      DSA_CHECK(slot < vport->out_links().size())
        << "Slot " << slot << " out of bounds for ivport " << vport->name()
        << " with size: " << vport->out_links().size() << " and stated: " << vport->vp_stated();

      // Get node associated with slot
      ssnode* next = vport->out_links()[slot]->sink();

      // Bookkeeping to add in at the end
      removed_input_vport = {0, vport->out_links()[slot]};
      
      // Replace source with the next node
      //source.first = slot;
      source.second = next;
    } 
  }

  // Check if destination is an output Vector Port
  if (auto vport = dynamic_cast<ssovport*>(dest.second)) {
    if (vport->vp_impl() == 2) {
      // Get output dfg node
      dsa::dfg::OutputPort* outNode = dynamic_cast<dsa::dfg::OutputPort*> (edge->use());

      // get the slot that this edge must be mapped to
      int sourceIdx = edge->oid;

      if (vport->vp_stated() && outNode->penetrated_state == -1)
        sourceIdx++;

      // Sanity Check
      DSA_CHECK(sourceIdx < vport->in_links().size()) << "Slot " << sourceIdx << " out of bounds for ovport " << vport->name() << " with size: " << vport->in_links().size() << " and stated: " << vport->vp_stated();

      // Bookkeeping for later
      removed_output_vport = {0, vport->in_links()[sourceIdx]};

      // Replace destination with the node that should be routed
      // dest.first = sourceIdx;
      dest.second = vport->in_links()[sourceIdx]->source();
    }
  }

  // Special Case where stated makes path shorter and become the same node
  if (source.second == dest.second && !path_lengthen) {
    int count = 0;
    auto idx = ins_it ? *ins_it - sched->links_of(edge).begin() : 0;

    // First check if the output port was removed
    if (removed_output_vport.second != nullptr) {
      dest = {removed_output_vport.first, removed_output_vport.second->sink()};
      auto node_slot = std::make_pair(removed_output_vport.first, removed_output_vport.second->sink());

      insert_edge(removed_output_vport, sched, edge, sched->links_of(edge).end(), dest, node_slot);
      count++;
    }

    // Second check if the input port was removed
    if (removed_input_vport.second != nullptr) {
      auto node_slot = std::make_pair(removed_input_vport.first, removed_input_vport.second->sink());
      insert_edge(removed_input_vport, sched, edge, sched->links_of(edge).begin() + idx, dest, node_slot);
      count++;
    }

    return std::max(count, 1);
  }

  int source_node_id = source.second->id();
  DSA_CHECK(source_node_id < n_nodes) << "Source node is should be less than total node number";
  done[source_node_id][source.first] = new_rand_prio;
  node_dist[source_node_id][source.first] = 0;
  DSA_CHECK(source.first < came_from[source_node_id].size()) << "N Lanes is problematic";
  came_from[source_node_id][source.first] = std::make_pair(0, nullptr);
  openset.emplace(0, new_rand_prio, source.first, source.second);

  bool memEdge = edge->memory();

  while (!openset.empty()) {
    auto &front_elem = *openset.begin();
    int cur_dist = std::get<0>(front_elem);
    int slot = std::get<2>(front_elem);
    ssnode* node = std::get<3>(front_elem);
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
      if (edge->memory()) {
        if (link->sink()->spatial() || link->source()->spatial()) {
          continue;
        } 
      } else {
        if (link->sink()->data() && link->source()->data()) {
          continue;
        }
      }

      int granularity =
          std::max(link->source()->granularity(), link->sink()->granularity());
      int slots = link->slots(slot, edge->bitwidth() / granularity);
      DSA_LOG(SLOTS) << "Link " << link->name() << " subnet size: " << link->subnet.size();
      while (slots) {
        int raw = slots & -slots;
        slots -= raw;
        int next_slot = 31 - __builtin_clz(raw);

        if (next_slot * node->granularity() % edge->bitwidth()
             && link->sink()->type() != ssnode::NodeType::Switch) {
          continue;
        }
        sslink* next_link = link;
        ssnode* next = next_link->sink();

        DSA_CHECK(next_slot < next->lanes())
          << "Next slot is out of range: " << came_from[next->id()].size()
          << " " << next_slot << " " << next->lanes();

        // Check to see if we are mapping to memory or the spatial architecture
        // We shouldn't have edges going through both
        if (memEdge) {
          if(next->spatial()) {
            continue;
          }
        } else {
          if (next->data()) {
            continue;
          }
        }

        std::pair<int, sslink*> next_pair(next_slot, next_link);

        DSA_LOG(SLOTS) << "width: " << edge->bitwidth() << ", From "
                       << link->source()->name() << "'s " << slot << " to "
                       << link->sink()->name() << "'s " << next_slot;

        int route_cost;
        if (!path_lengthen) {  // Normal thing
          route_cost = routing_cost(edge, slot, next_slot, next_link, sched, dest);
        } else {
          // For path lengthening, only route on free spaces
          route_cost = sched->routing_cost(next_pair, edge);
          if (sched->dfgNodeOf(next_slot, next)) {
            route_cost = -1;
          }
        }

        if (route_cost == -1) continue;

        int new_dist = cur_dist + route_cost;

        DSA_CHECK(next_slot < node_dist[next->id()].size())
          << "Next slot is out of range: " << node_dist[next->id()].size()
          << " " << next_slot << " " << node->lanes();

        int next_dist = node_dist[next->id()][next_slot];

        bool over_ride = (path_lengthen && make_pair(next_slot, next) == dest);

        if (next_dist == -1 || next_dist > new_dist || over_ride) {
          if (next_dist != -1) {
            int next_rand_prio = done[next->id()][next_slot];

            auto iter =
                openset.find(std::make_tuple(next_dist, next_rand_prio, next_slot, next));
            if (iter != openset.end()) openset.erase(iter);
          }
          int new_rand_prio = rand() % 16;
          done[next->id()][next_slot] = new_rand_prio;


          openset.emplace(new_dist, new_rand_prio, next_slot % next->lanes(), next);

          

          node_dist[next->id()][next_slot] = new_dist;
          DSA_CHECK(next->id() < n_nodes);
          DSA_CHECK(next_slot < came_from[next->id()].size());
          DSA_CHECK(next_slot < next->lanes());
          came_from[next->id()][next_slot] = std::make_pair(slot, next_link);
        }
      }
    }
  }

  if (node_dist[dest.second->id()][dest.first] == -1 ||
      (path_lengthen && node_dist[dest.second->id()][dest.first] == 0)) {
    return false;  // routing failed, no routes exist!
  }

  auto idx = ins_it ? *ins_it - sched->links_of(edge).begin() : 0;
  
  auto node_slot = dest;

  int count = 0;
  dsa::dfg::Edge* alt_edge = nullptr;
  pair<int, sslink*> link;
  
  // This means that we had an output vector port at beginnning of edge
  if (removed_output_vport.second != nullptr) {
    auto old_dest = dest;
    dest = {removed_output_vport.first, removed_output_vport.second->sink()};
    node_slot = dest;
    insert_edge(removed_output_vport, sched, edge, sched->links_of(edge).end(), dest, node_slot);
    count++;
    
    node_slot = old_dest;
  }
  

  // Go through djikstra path in reverse order starting at dest
  while (node_slot != source || (path_lengthen && count == 0)) {
    // Increase Count of selected Nodes
    count++;

    // Get the link, slot pair for next node
    link = came_from[node_slot.second->id()][node_slot.first];

    // save the slot
    auto link_backup = link;
    
    // Change the slot to be the previous slot
    link.first = node_slot.first;

    // Check if theres an alternate edge
    if ((alt_edge = sched->alt_edge_for_link(link, edge))) {
      break;
    }
    
    // Insert link into the edge
    insert_edge(link, sched, edge, sched->links_of(edge).begin() + idx, dest, node_slot);

    node_slot = std::make_pair(link_backup.first, link.second->source());
  }

  // Need to make sure we add back the other links in-order.
  // With path lengthening (which has cycles) the previous
  // code can't gaurantee that.

  if (alt_edge) {
    DSA_LOG(ROUTE) << "Inserting alternate edge " << alt_edge->name();
    auto& alt_links = sched->links_of(alt_edge);
    for (auto alt_link : alt_links) {
      node_slot = std::make_pair(alt_link.first, alt_link.second->sink());
      insert_edge(alt_link, sched, edge, sched->links_of(edge).begin() + idx, dest, node_slot);
      idx++;

      if (alt_link == link) break;  // we added the final link
    }
  }
  
  if (removed_input_vport.second != nullptr && !alt_edge) {
    node_slot = std::make_pair(removed_input_vport.first, removed_input_vport.second->sink());
    
    insert_edge(removed_input_vport, sched, edge, sched->links_of(edge).begin(),dest, node_slot);
    count++;
  }
  
  return count;
}

// Assign a node, indices and hardware port from here...
bool SchedulerSimulatedAnnealing::scheduleHere(Schedule* sched, dsa::dfg::Node* node,
                                               mapper::Slot<ssnode*> here) {

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
        if (!route(sched, edge, src, dest, nullptr, 0)) {             \
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
  
  //DSA_INFO << "Schedule Here: " << node->name() << " at " << here.lane_no << " " << here.ref->name();
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
        if (!route(sched, edge, loc, here_, nullptr, 0)) {
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
        if (!route(sched, edge, here_, loc, nullptr, 0)) {
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

  pair<int, int> bestScore = std::make_pair(INT_MIN, INT_MIN);
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
    DSA_LOG(MAP)
      << "  Try: " << candidates[idx[i]].slot.lane_no << ", " << candidates[idx[i]].slot.ref->name();

    if (scheduleHere(sched, node, candidates[idx[i]].slot)) {
      SchedStats s;
      CandidateRoute undo_path;
      pair<int, int> candScore = obj_creep(sched, s, undo_path);
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
  }

  DSA_LOG(MAP) << "return for best candidate!";
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
