#include "scheduler.h"
#include "scheduler_sa.h"

using namespace SS_CONFIG;
using namespace std;

#include <fstream>
#include <sstream>
#include <unordered_map>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <list>
#include <signal.h>

//Utility functions
int rand_bt(int s, int e) { 
  assert(e > s && "bad range for rand_bt");
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

  for(unsigned i = 0; i < dfg_in_sizes.size(); ++i) {
    if(dfg_in_sizes[i] > vec_in_sizes[i]) {
      if(verbose) { 
        std::cerr << "Vector Inputs Insufficient\n";
      }
      return false;
    }
  }
  for(unsigned i = 0; i < dfg_out_sizes.size(); ++i) {
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

std::string basename(const std::string& filename) {
  size_t lastindex = filename.find_last_of(".");
  string res = filename.substr(0, lastindex);

  lastindex = filename.find_last_of("\\/");
  if (lastindex != string::npos) {
    res = res.substr(lastindex + 1);
  }
  return res;
}

std::string basedir(const std::string& filename) {
  size_t lastindex = filename.find_last_of("\\/");
  if (lastindex == string::npos) {
    return std::string("./");
  }
  return filename.substr(0, lastindex);
}


Schedule *Scheduler::invoke(SSModel *model, SSDfg *dfg, bool print_bits) {
  bool succeed_sched = false;
  Schedule *sched = nullptr;

  string dfg_base = basename(dfg->filename);  // the name without preceeding dirs or file extension
  string pdg_dir = basedir(dfg->filename);  // preceeding directories only
  if (pdg_dir[pdg_dir.length() - 1] != '\\' || pdg_dir[pdg_dir.length() - 1] != '/') {
    pdg_dir += "/";
  }
  string viz_dir = pdg_dir + "viz/";
  string iter_dir = pdg_dir + "viz/iter/";
  string verif_dir = pdg_dir + "verif/";
  string sched_dir = pdg_dir + "sched/";  // Directory for cheating on the scheduler

  checked_system(("mkdir -p " + viz_dir).c_str());
  checked_system(("mkdir -p " + iter_dir).c_str());
  checked_system(("mkdir -p " + verif_dir).c_str());
  checked_system(("mkdir -p " + sched_dir).c_str());
  checked_system("mkdir -p gams/");  // gams will remain at top level

  std::string model_filename = model->filename;
  int lastindex = model_filename.find_last_of(".");
  string model_rawname = model_filename.substr(0, lastindex);
  string model_base =
        model_rawname.substr(model_rawname.find_last_of("\\/") + 1, model_rawname.size());

  if (check_feasible(dfg, model, verbose)) {

    auto sigint_handler = [] (int) {
      exit(1);
    };

    // Setup signal so we can stop if we need to
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = sigint_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // Debug
    // string hw_config_filename = model_rawname + ".xml";
    // sched -> printConfigBits_Hw(hw_config_filename);

    succeed_sched = schedule_timed(dfg, sched);

    int lat = 0, latmis = 0;
    if (succeed_sched) {
      sched->cheapCalcLatency(lat, latmis);
      sched->set_decode_lat_mis(latmis);

      // ofstream ctxs(viz_dir + dfg_base + ".config", ios::out);
      // sched->printConfigText(ctxs); // text form of config fed to gui
    }

    if (verbose) {
      sched->cheapCalcLatency(lat, latmis);
      int ovr = 0, agg_ovr = 0, max_util = 0;
      sched->get_overprov(ovr, agg_ovr, max_util);
      int violation = sched->violation();

      // sched->checkOutputMatch(latmis);
      if (succeed_sched) {
        // Also check final latency

        cout << "latency: " << lat << "\n";
        cout << "lat-mismatch-max: " << latmis << "\n";
        cout << "lat-mismatch-sum: " << violation << "\n";
        cout << "overprov-max: " << ovr << "\n";
        cout << "overprov-sum: " << agg_ovr << "\n";
      } else {
        cout << "latency: " << -1 << "\n";
        cout << "latency mismatch: " << -1 << "\n";
        cout << "Scheduling Failed!\n";
      }
      sched->stat_printOutputLatency();
      dfg->printGraphviz("viz/final.dot", sched);
    }

    ofstream ofs(viz_dir + dfg_base + ".dot", ios::out);
    assert(ofs.good());
    dfg->printGraphviz(ofs);
    ofs.close();

    std::string sched_viz = viz_dir + dfg_base + "." + model_base + ".gv";
    sched->printGraphviz(sched_viz.c_str());

    std::string verif_header = verif_dir + dfg_base + ".configbits";
    std::ofstream vsh(verif_header);
    assert(vsh.good());
    sched->printConfigVerif(vsh);
  }

  lastindex = dfg->filename.find_last_of(".");
  string pdg_rawname = dfg->filename.substr(0, lastindex);

  if (!succeed_sched || sched == nullptr) {
    cout << "We're going to print the DFG for simulation purposes...  have fun!\n\n";
    // This is just a fake schedule!
    // sched = new Schedule(&ssmodel,&sspdg);
    SchedulerSimulatedAnnealing* s = new SchedulerSimulatedAnnealing(model);
    s->set_fake_it();

    s->initialize(dfg, sched);
    succeed_sched = s->schedule_internal(dfg, sched);
  }

  // TODO: Print Hardware Config Information @ Sihao
  // string hw_config_filename = model_rawname + ".xml";
  // sched -> printConfigBits_Hw(hw_config_filename);

  sched->set_name(pdg_rawname);
  std::string config_header = pdg_rawname + ".dfg.h";
  std::ofstream osh(config_header);
  assert(osh.good());
  sched->printConfigHeader(osh, dfg_base);
  if (verbose) {
    std::cout << "Performance: " << dfg->estimated_performance(sched, false) << std::endl;
  }

  if (print_bits) {
    std::string config_header_bits = pdg_rawname + ".dfg.bits.h";
    std::ofstream oshb(config_header_bits);
    assert(oshb.good());
    sched->printConfigHeader(oshb, dfg_base, true);
  }

  return sched;  // just to calm HEAPCHECK
}
