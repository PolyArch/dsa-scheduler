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

  bool schedule(SSDfg *, Schedule *&);

  std::pair<int, int> route(Schedule *sched, SSDfgEdge *dfgnode,
                            std::pair<int, SS_CONFIG::ssnode *> source, std::pair<int, SS_CONFIG::ssnode *> dest,
                            CandidateRouting &);

  int routing_cost(SSDfgEdge *, int, int, sslink *, Schedule *, CandidateRouting &, const std::pair<int, ssnode*> &);

  void set_fake_it() { _fake_it = true; }

  bool schedule_internal(SSDfg *ssDFG, Schedule *&sched);

protected:
  std::pair<int, int> obj(Schedule*& sched, int& lat, 
      int& lat_mis, int& ovr, int& agg_ovr, int& max_util); 

  bool scheduleNode(Schedule *sched, SSDfgInst *dfgnode) override;

  std::pair<int, int> scheduleHere(Schedule *, SSDfgNode *, std::pair<int, SS_CONFIG::ssnode *>, CandidateRouting &);

  void findFirstIndex(ssio_interface &si, unsigned int numIO, unsigned int &index, bool is_input);

  bool genRandomIndexBW(std::pair<bool, int> &vport_id, std::vector<int> &vport_desc,
                        ssio_interface &si, unsigned int index, Schedule *&sched, bool s);

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
      T* to_map = nodes[i];
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
bool SchedulerSimulatedAnnealing::schedule_io(T*  vec, SSDfg* ssDFG, Schedule* sched) {
  using std::vector;
  using std::pair;
  using std::make_pair;

  SS_CONFIG::SubModel *subModel = _ssModel->subModel();
  ssio_interface &si = subModel->io_interf();
  int n_vertex = (int) vec->vector().size();
  int n_vertex_physical = n_vertex;

  if (vec->is_temporal()) {
    n_vertex_physical = 1;  // temporal vectors are always scheduled to one node
  }

  unsigned int index = 0;
  int num_found = 0;
  //use physical number of vertices to decide a port
  findFirstIndex(si, n_vertex_physical, index, T::IsInput() /*is i/o*/);
  unsigned int attempt = 0;

  vector<int> order; //temp variable used for randomly iterating

  CandidateRouting r1, r2; //do this so that function scope de-allocates these
  CandidateRouting *bestRouting = &r1, *candRouting = &r2;

  vector<bool> bestMask;
  pair<int, int> bestScore = std::make_pair(INT_MIN, INT_MIN);
  pair<bool, int> bestVportid;
  std::vector<ssnode *> bestInputs;

  while (num_found < 9 &&  (attempt++ < sd(T::IsInput()).size())) {
    pair<bool, int> vport_id;
    vector<int> vport_desc;

    //TODO: put this code in schedule_output as well
    bool found=false;
    while(!found) {
      if (!genRandomIndexBW(vport_id, vport_desc, si, index, sched, T::IsInput() /*is i/o*/)) {
        unmap_one<T>(ssDFG, sched);
      } else {
        found = vec->is_temporal() || si.get(T::IsInput(), vport_id.second)->size() >= vec->vector().size();
      }
    }


    candRouting->clear();
    std::vector<ssnode*> possInputs;

    for (unsigned m = 0; m < vport_desc.size(); m++) {
      int cgra_port_num = vport_desc[m];
      typename T::Scalar::MapsTo *in = subModel->nodes<typename T::Scalar::MapsTo*>()[cgra_port_num];
      if (sched->dfgNodeOf(in) == nullptr) {
        possInputs.push_back(in);
      }
    }

    if ((int) possInputs.size() < n_vertex_physical) continue;

    vector<ssnode *> candInputs;
    rand_node_choose_k(n_vertex_physical, possInputs, candInputs);

    bool ports_okay_to_use = true;
    random_order(n_vertex, order);
    int num_links_used = 0;
    for (int i : order) {
      //In a temporal region, just select the 0th input
      int cand_index = vec->is_temporal() ? 0 : i;
      ssnode *node = candInputs[cand_index];
      SSDfgNode *vertex = vec->vector()[i];

      pair<int, int> n_score = scheduleHere(sched, vertex, make_pair(0, node), *candRouting);
      if (n_score >= fscore) {
        ports_okay_to_use = false;
        break;
      }

      num_links_used += n_score.second;
    }

    if (!ports_okay_to_use) continue;
    num_found++;

    apply_routing(sched, candRouting);
    for (int i = 0; i < n_vertex; ++i) {
      int cand_index = vec->is_temporal() ? 0 : i;
      ssnode *node = candInputs[cand_index];
      sched->assign_node(vec->vector()[i], make_pair(0, node));
    }

    int lat = INT_MAX, latmis = INT_MAX, ovr = INT_MAX,  agg_ovr = INT_MAX, max_util = INT_MAX;
    pair<int, int> candScore = obj(sched, lat, latmis, ovr, agg_ovr, max_util);
    candScore.second -= num_links_used;

    //TODO: does this help at all?
    int hw_port_size = vport_desc.size();
    int extra = hw_port_size - vec->vector().size();
    candScore.second-=extra*10;

    sched->unassign_vec(vec);

    if (candScore > bestScore) {
      //cout << candScore.first << " " << candScore.second <<"\n";
      //cout << "lat: " << lat << " latmis: " << latmis 
      //    << " ovr " << ovr << " num links used " << num_links_used << "\n";
      bestScore = candScore;
      bestInputs = candInputs;
      bestVportid = vport_id;
      std::swap(bestRouting, candRouting);
      bestMask.clear();
      bestMask.resize(vport_desc.size());
      int num_cgra_ports = 0; //for debugging
      for (int m = 0; m < (int) vport_desc.size(); m++) {
        int cgra_port_num = vport_desc[m];
        for (int i = 0; i < (int) candInputs.size(); ++i) {
          if (static_cast<ssinput *>(candInputs[i])->port() == cgra_port_num) {
            assert(bestMask[m] == false);
            bestMask[m] = true;
            num_cgra_ports++;
            break;
          }
        }
      }
      //assert(n_vertex == num_cgra_ports);
    }
  }
  //cout << " -- \n";

  if (num_found > 0) {
    for (int i = 0; i < n_vertex; ++i) {
      int cand_index = vec->is_temporal() ? 0 : i;
      ssnode *node = bestInputs[cand_index];
      sched->assign_node(vec->vector()[i], make_pair(0, node));
    }
    sched->assign_vport(vec, bestVportid, bestMask);
    apply_routing(sched, bestRouting); //Commit the routing
  }
  return num_found > 0;
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