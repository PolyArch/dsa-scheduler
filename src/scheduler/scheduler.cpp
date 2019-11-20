#include "scheduler.h"

using namespace SS_CONFIG;
using namespace std;

#include <fstream>
#include <sstream>
#include <unordered_map>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <list>

//Utility functions
int rand_bt(int s, int e) { 
  assert(e>s && "bad range for rand_bt");
  return rand() % (e - s) + s; 
}

int rand_bt_large(int s, int e) { 
  return (rand() * RAND_MAX + rand()) % (e - s) + s; 
}


// floyd's unique-number sampling algorithm (ordered)
// CACM Programming Pearls, 1987
void HeuristicScheduler::rand_n_choose_k(int n, int k, std::vector<int>& indices) {
  std::vector<bool> mask;
  mask.resize(n);
  indices.clear();

  for (int j = n - k; j < n; ++j) {
    int t = rand_bt(0, j + 1);
    if (mask[t] == false) {
      mask[t] = true;
    } else {
      mask[j] = true;
    }
  }
  for (int i = 0; i < n; ++i) {
    if (mask[i]) {
      indices.push_back(i);
    }
  }
}

void HeuristicScheduler::random_order(int n, std::vector<int>& order) {
  order.clear();
  for (int i = 0; i < n; ++i) order.push_back(i);
  std::random_shuffle(order.begin(), order.end());
}

// could make this a template really!
vector<bool> HeuristicScheduler::rand_node_choose_k(int k,
                                                    std::vector<ssnode*>& poss_nodes,
                                                    std::vector<ssnode*>& chosen_nodes) {
  std::vector<bool> mask;
  int n = poss_nodes.size();
  mask.resize(n);
  chosen_nodes.clear();

  for (int j = n - k; j < n; ++j) {
    int t = rand_bt(0, j + 1);
    if (mask[t] == false) {
      mask[t] = true;
    } else {
      mask[j] = true;
    }
  }

  for (int i = 0; i < n; ++i) {
    if (mask[i]) {
      chosen_nodes.push_back(poss_nodes[i]);
    }
  }
  return mask;
}

bool Scheduler::vport_feasible(SSDfg* dfg, SSModel* ssmodel, bool verbose) {
  auto* sub = ssmodel->subModel();

  //Fast algorithm for checking if there are enough vports
  std::vector<int> vec_in_sizes;
  std::vector<int> vec_out_sizes;
  std::vector<int> dfg_in_sizes;
  std::vector<int> dfg_out_sizes;
  for(auto& p : sub->input_list()) vec_in_sizes.push_back(p->input_bitwidth());
  for(auto& p : sub->output_list()) vec_out_sizes.push_back(p->output_bitwidth());
  for(auto& v : dfg->vec_inputs()) dfg_in_sizes.push_back(v->phys_bitwidth());
  for(auto& v : dfg->vec_outputs()) dfg_out_sizes.push_back(v->phys_bitwidth());

  if(dfg_in_sizes.size() > vec_in_sizes.size()) return false;
  if(dfg_out_sizes.size() > vec_out_sizes.size()) return false;

  std::sort(vec_in_sizes.begin(),vec_in_sizes.end(),greater<int>());
  std::sort(vec_out_sizes.begin(),vec_out_sizes.end(),greater<int>());
  std::sort(dfg_in_sizes.begin(),dfg_in_sizes.end(),greater<int>());
  std::sort(dfg_out_sizes.begin(),dfg_out_sizes.end(),greater<int>());

  for(int i = 0; i < dfg_in_sizes.size(); ++i) {
    if(dfg_in_sizes[i] > vec_in_sizes[i]) {
      if(verbose) { 
        std::cerr << "Vector Inputs Insufficient\n";
      }
      return false;
    }
  }
  for(int i = 0; i < dfg_out_sizes.size(); ++i) {
    if(dfg_out_sizes[i] > vec_out_sizes[i]) {
      if(verbose) { 
        std::cerr << "Vector Outputs Insufficient\n";
      }
      return false;
    }
  }
  return true; 
}

bool Scheduler::check_feasible(SSDfg* ssDFG, SSModel* ssmodel, bool verbose) {
  if(!vport_feasible(ssDFG,ssmodel,verbose)) {
    return false;
  }
  int dedicated_insts = 0;
  int temporal_insts = 0;
  for (auto i : ssDFG->inst_vec()) {
    if (!i->is_temporal()) {
      dedicated_insts++;
    } else {
      temporal_insts++;
    }
  }

  int temporal_fus = 0;
  int temporal_inst_slots = 0;
  for (auto elem : _ssModel->subModel()->fu_list()) {
    temporal_inst_slots += elem->max_util();
    temporal_fus += 1;
  }

  // int nfus = ssmodel->subModel()->sizex() * ssmodel->subModel()->sizey();
  // if (dedicated_insts > nfus) {
  //  cerr << "\n\nError: Too many dedicated instructions ("
  //       << dedicated_insts << ") in SSDfg for given SSCONFIG (has "
  //       << nfus << " fus)\n\n";
  //  return false;
  //}
  
  if (temporal_insts > temporal_inst_slots) {
    cerr << "\n\nError: Too many temporal instructions (" << temporal_insts
         << ") in SSDfg for given SSCONFIG (has " << temporal_inst_slots
         << " temporal slots)\n\n";
    return false;
  }

  bool failed_count_check = false;

  std::map<ss_inst_t, int> dedicated_count_types, temporal_count_types;
  for (auto elem : ssDFG->inst_vec()) {
    if (!elem->is_temporal()) {
      dedicated_count_types[elem->inst()]++;
    } else {
      temporal_count_types[elem->inst()]++;
    }
  }

  for (auto& pair : dedicated_count_types) {
    ss_inst_t ss_inst = pair.first;
    int dfg_count = pair.second;

    int fu_count = 0;
    for (ssfu* cand_fu : _ssModel->subModel()->fu_list()) {
      if (cand_fu->fu_def()->is_cap(ss_inst)) {
        fu_count += 64 / SS_CONFIG::bitwidth[ss_inst];
      }
    }
    if (fu_count < dfg_count) {
      failed_count_check = true;
      cerr << "Error: DFG has " << dfg_count << " " << name_of_inst(ss_inst)
           << " dedicated insts, but only " << fu_count
           << " dedicated fus to support them\n";
    }
  }

  for (auto& pair : temporal_count_types) {
    ss_inst_t ss_inst = pair.first;
    int dfg_count = pair.second;

    int fu_count = 0;
    for(ssfu* cand_fu : _ssModel->subModel()->fu_list()) {
      if (cand_fu->max_util() > 1 && cand_fu->fu_def()->is_cap(ss_inst)) {
        fu_count += cand_fu->max_util(ss_inst);
      }
    }
    if (fu_count < dfg_count) {
      failed_count_check = true;
      cerr << "Error: DFG has " << dfg_count << " " << name_of_inst(ss_inst)
           << " temporal insts, but only " << fu_count
           << " temporal fu slots to support them\n";
    }
  }

  if (failed_count_check) {
    cerr << "\n\nError: FAILED Basic FU Count Check\n\n";
    return false;
  }
  // TODO: add code from printPortcompatibility here

  return true;
}
