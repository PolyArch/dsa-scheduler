#include <iostream>
#include <iomanip>

#include "dsa/core/singleton.h"
#include "dsa/dfg/node.h"
#include "dsa/mapper/dse.h"
#include "dsa/mapper/scheduler_sa.h"

CodesignInstance::CodesignInstance(SSModel* model) : _ssModel(*model) {
  unused_nodes = std::vector<bool>(model->subModel()->node_list().size(), true);
  unused_links = std::vector<bool>(model->subModel()->link_list().size(), true);
}

namespace dsa {

const std::string log_header = "Time,Iteration,Meaningful Iterations,Temperature,Current Objective,Best Objective,Current Performance,Best Performance,Current Normalized Resources,Best Normalized Resources,Current Util Overall,Current Link Util,Current Node Util,Best Util Overall,Best Link Util,Best Node Util,DSE Change Details,Current Resources,Current FU Resource,Current Switch Resource,Current VPort Resource,Current Mem Resource,Best Resources,Best FU Resource,Best Switch Resource,Best VPort Resource,Best Mem Resource,Current Workload Weights,Best Workload Weights,Current Workload Performance,Best Workload Performance,Current DFG Performances,Best DFG Performances";

// Print Resources for given type
std::string resources(int type, dsa::adg::estimation::Result& estimated) {
  std::ostringstream s;
  switch(type) {
    case 0: // Sum
      s << estimated.sum()->dump();
      break;
    case 1: // FU
      s << estimated.resource_bd(0)->dump();
      break;
    case 2: // Switch
      s << estimated.resource_bd(1)->dump();
      break;
    case 3: // VPort
      s << estimated.resource_bd(2)->dump();
      break;
    case 4: // Memory
      s << estimated.resource_bd(3)->dump();
      break;
    case 5: { // Normalized Sum
      auto normalized_estimated = estimated.sum();
      normalized_estimated->normalize();
      s << normalized_estimated->dump();
      break;
    }
    case 6: { // Normalized FU 
      auto normalized_estimated_fu = estimated.resource_bd(0);
      normalized_estimated_fu->normalize();
      s << normalized_estimated_fu->dump();
      break;
    }
    case 7: { // Normalized Switch
      auto normalized_estimated_switch = estimated.resource_bd(1);
      normalized_estimated_switch->normalize();
      s << normalized_estimated_switch->dump();
      break;
    }
    case 8: { // Normalized VPort
      auto normalized_estimated_vport = estimated.resource_bd(2);
      normalized_estimated_vport->normalize();
      s << normalized_estimated_vport->dump();
      break;
    }
    case 9: { // Normalized Memory
      auto normalized_estimated_mem = estimated.resource_bd(3);
      normalized_estimated_mem->normalize();
      s << normalized_estimated_mem->dump();
      break;
    }
  }
  return s.str();
}

std::string dump_log(const double& time, const int& iteration, const int& last_improve, const double& temp, CodesignInstance* curr_ci, CodesignInstance* best_ci) {
  auto curr_util = curr_ci->utilization();
  auto best_util = best_ci->utilization();

  auto best_estimated = dsa::adg::estimation::EstimatePowerAera(best_ci->ss_model());
  auto curr_estimated = dsa::adg::estimation::EstimatePowerAera(curr_ci->ss_model());
  
  std::ostringstream s;
  s << time << ","
    << iteration << ","
    << last_improve << "," 
    << temp << ","
    << curr_ci->weight_obj() << ","
    << best_ci->weight_obj() << ","
    << curr_ci->performance << ","
    << best_ci->performance << ",\""
    << resources(5, curr_estimated) << "\",\""
    << resources(5, best_estimated) << "\","
    << std::get<0>(curr_util) << ","
    << std::get<1>(curr_util) << ","
    << std::get<2>(curr_util) << ","
    << std::get<0>(best_util) << ","
    << std::get<1>(best_util) << ","
    << std::get<2>(best_util) << ","
    << curr_ci->get_changes_log() << ",\"" 
    << resources(0, curr_estimated) << "\",\"" 
    << resources(1, curr_estimated) << "\",\"" 
    << resources(2, curr_estimated) << "\",\"" 
    << resources(3, curr_estimated) << "\",\""
    << resources(4, curr_estimated) << "\",\"" 
    << resources(0, best_estimated) << "\",\"" 
    << resources(1, best_estimated) << "\",\"" 
    << resources(2, best_estimated) << "\",\"" 
    << resources(3, best_estimated) << "\",\""
    << resources(4, best_estimated) << "\"," 
    << curr_ci->get_workload_weights() << ","
    << best_ci->get_workload_weights() << ","
    << curr_ci->get_workload_performances() << ","
    << best_ci->get_workload_performances() << ","
    << curr_ci->get_dfg_performances() << ","
    << best_ci->get_dfg_performances();
  return s.str();
}

void dump_hw(CodesignInstance*& ci, int i) {
  std::stringstream hw_ss;
  hw_ss << "viz/iters/dse-sched-" << i << ".json";
  ci->ss_model()->subModel()->DumpHwInJson(hw_ss.str().c_str());
}

void initialize_workloads(CodesignInstance*& ci, const std::string &pdg_filename) {
  std::string curline;
  std::ifstream dfg_names(pdg_filename);
  while (std::getline(dfg_names, curline)) {
    if (curline == "%%") {
      ci->workload_array.emplace_back();
      ci->weight.push_back(1);
    } else if (curline.find("weight=") == 0) {
      std::istringstream ssin(curline.substr(8, curline.size()));
      ssin >> ci->weight.back();
    } else {
      ci->workload_array.back().sched_array.emplace_back(ci->ss_model(), new SSDfg(curline));
    }
  }
}

void setup_indirect(CodesignInstance*& ci, SchedulerSimulatedAnnealing*& scheduler) {
  double best_indir = -1;
  double best_obj = -1;
  scheduler->incrementalSchedule(*ci);
  for (int indirect = 0; indirect <= 2; ++indirect) {
    ci->ss_model()->indirect(indirect);
    double indir_obj = ci->weight_obj();
    if (indir_obj > best_obj) {
      best_indir = indirect;
      best_obj = indir_obj;
    }
  }
  ci->ss_model()->indirect(best_indir);
}

void filter_useless_function_units(CodesignInstance*& ci) {
  auto& ssmodel = *ci->ss_model();
  std::set<dsa::OpCode> used_insts;
  for (auto& elem : ci->workload_array) {
    for (auto& dfg : elem.sched_array) {
      struct InstCounter : dfg::Visitor {
        void Visit(dfg::Instruction* inst) { res.insert(inst->inst()); }
        std::set<dsa::OpCode> res;
      } counter;
      dfg.ssdfg()->Apply(&counter);
      for (auto& inst : counter.res) {
        used_insts.insert(inst);
      }
    }
  }

  //  TODO: remove hack to make is so zero size capabilities are removed
  for (int i = 0; i < (int)ssmodel.fu_types.size(); ++i) {
    auto& fudef = ssmodel.fu_types[i];
    for (int j = 0; j < (int)fudef->capability.size(); ++j) {
      if (used_insts.find(fudef->capability[j].op) == used_insts.end() && fudef->capability[j].op != dsa::OpCode::SS_Copy) {
        fudef->Erase(j);
        --j;
      }
    }
    if ((int)fudef->capability.size() == 0) {
      //DSA_CHECK(false);
    } else if ((int)fudef->capability.size() > used_insts.size()) {
      DSA_CHECK(false) << "Instruction Capability " << fudef->capability.size() << " is larger than used instructions " << used_insts.size();
    }
  }

  for (int i = 0; i < (int)ssmodel.subModel()->fu_list().size(); ++i) {
    auto* fu = ssmodel.subModel()->fu_list()[i];
    for (int j = 0; j < (int)fu->fu_type_.capability.size(); ++j) {
      if (used_insts.find(fu->fu_type_.capability[j].op) == used_insts.end()) {
          fu->fu_type_.Erase(j);
          --j;
      }
    }
    if ((int)fu->fu_type_.capability.size() == 0) {
      //DSA_CHECK(false);
    } else if ((int)fu->fu_type_.capability.size() > used_insts.size()) {
      DSA_CHECK(false) << "Instruction Capability " << fu->fu_type_.capability.size() << " is larger than used instructions " << used_insts.size();
    }
  }
}

void dump_checkpoint(Schedule* sched, const std::string& filename, double performance, bool isFinal=false) {
  if (!sched) return;
  std::string path = "viz/iters/" + filename;
  if (isFinal)
    path = "viz/" + filename;
  std::cout << "Dumping " << sched->ssdfg()->filename << path << "/ " << performance << std::endl;
  ENFORCED_SYSTEM(("mkdir -p " + path).c_str());
  sched->printGraphviz((path + "/graph.gv").c_str());
  std::ofstream ofs(path + "/" + filename + ".dfg.h");
  sched->printConfigHeader(ofs, filename);
}

void DesignSpaceExploration(SSModel &ssmodel, const std::string &pdg_filename) {
  // Create Objective CSV File
  std::string path = "viz/objectives.csv";
  ofstream ofs(path);
  DSA_CHECK(ofs.good()) << path << " not opened!";
  ofs << log_header << std::endl;

  // Set up Scheduler
  auto &ci = dsa::ContextFlags::Global();
  auto scheduler = new SchedulerSimulatedAnnealing(&ssmodel);
  clock_t start_time = clock();
  scheduler->set_start_time();

  // Set up Codesign Instance
  CodesignInstance* cur_ci = new CodesignInstance(&ssmodel);
  initialize_workloads(cur_ci, pdg_filename);
  setup_indirect(cur_ci, scheduler);
  filter_useless_function_units(cur_ci);
  cur_ci->verify();

  // Schedule First Node
  clock_t StartSchedule = clock();
  scheduler->incrementalSchedule(*cur_ci);
  clock_t ScheduleCollapse = clock() - StartSchedule;

  CodesignInstance* best_ci = cur_ci;

  // Create DSE Loop Variables
  int improv_iter = 0;
  double temperature = 24;
  int i = 0;
  int last_improve = 0;
  double last_best_obj = -999;

  // Dump Start log and hw;
  ofs << dump_log(static_cast<double>(clock() - start_time) / CLOCKS_PER_SEC, i, last_improve, temperature, cur_ci, cur_ci) << std::endl;
  dump_hw(cur_ci, i);

  // Start DSE Iterations. Will end if it doesn't improve in 750 meaningful iterations
  while (last_improve < 750) {
    std::cout << " ### Begin DSE Iteration " << i << " (" << std::setprecision(2) <<temperature <<") ###" << std::endl;

    // Setup Next Iteration
    clock_t current_time = clock();
    double time_elps = static_cast<double>(current_time - start_time) / CLOCKS_PER_SEC;
    if (dsa::ContextFlags::Global().dse_timeout != -1) {
      if (time_elps > dsa::ContextFlags::Global().dse_timeout) {
        std::cout
          << time_elps << "s elapsed, the cutoff is "
          << dsa::ContextFlags::Global().dse_timeout
          << "s, break DSE" << std::endl;
        break;
      }
    }

    // Create new Codesign Instance
    CodesignInstance* cand_ci = new CodesignInstance(*cur_ci, false);
    
    // Verify all Codesign Instances
    cur_ci->verify();
    cand_ci->verify();
    best_ci->verify();

    // Make modification to current Codesign Instance
    cand_ci->make_random_modification(temperature);

    // Verify the Modification
    cand_ci->verify();
    
    // Print time for Modification
    std::cout << "dse modification: "
              << static_cast<double>(clock() - current_time) / CLOCKS_PER_SEC << "s"
              << std::endl;

    // Schedule the Modification
    clock_t StartSchedule = clock();
    scheduler->incrementalSchedule(*cand_ci);
    clock_t ScheduleCollapse = clock() - StartSchedule;
    cand_ci->verify();

    // Calcuate Objective
    double new_obj = cand_ci->weight_obj();
    double best_obj = best_ci->weight_obj();
    double init_obj = cur_ci->weight_obj();

    DSA_CHECK(best_obj > last_best_obj) << " best obj went down from " << last_best_obj << " to " << best_obj;

    // Print Objectives
    std::cout << "DSE OBJ: " << std::setprecision(4) << new_obj << "(Best:" << best_obj << ") (Iteration:" << init_obj << ")" << std::endl;
    
    auto util = cand_ci->utilization();

    std::cout << std::setprecision(2)
              << "Utilization ratio overall: " << std::get<0>(util)
              << ", nodes: " << std::get<1>(util) << ", links: " << std::get<2>(util)
              << "\n"
              << std::setprecision(7);

    ofs << dump_log(time_elps, i, last_improve, temperature, cand_ci, best_ci) << std::endl;

    // Reject a bad modification that causes everything not to schedule
    
    if (new_obj < (1.0 + 1e-3)) {
      delete cand_ci;
      ++i;
      continue;
    }
    

    cand_ci->dump_breakdown(ci.verbose);

    if (new_obj > best_obj) {
      // Yay! We have improved the solution
      
      last_best_obj = best_obj;

      // Change the Codesign Instances
      delete cur_ci;
      best_ci = cur_ci = cand_ci;

      // Print the Improvement
      std::cout << "----------------- IMPROVED OBJ! --------------------" << std::endl;
      std::cout << "New Objective: " << new_obj << " (from: " << best_obj << ")"<< std::endl;
      std::cout << "Execution Time: " << std::setprecision(6)
                << static_cast<double>(clock() - start_time) / CLOCKS_PER_SEC 
                << ", "
                << static_cast<double>(ScheduleCollapse) / CLOCKS_PER_SEC
                << std::endl;

      // dump the new hw json
      dump_hw(best_ci, i);

      for (int x = 0; x < best_ci->workload_array.size(); ++x) {
        std::string path = "viz/iters/iter_" + std::to_string(i);
        auto sched = best_ci->res[x];
        std::cout << "Dumping " << sched->ssdfg()->filename << "at" << path << "/" << x << " " <<  best_ci->dse_sched_obj(sched) << std::endl;
        ENFORCED_SYSTEM(("mkdir -p " + path).c_str());
        std::string filename = path + "/" + std::to_string(x);
        sched->printGraphviz((path + "/" + sched->ssdfg()->filename + ".gv").c_str());
        std::ofstream ofs(filename + ".dfg.h");
        sched->printConfigHeader(ofs, std::to_string(x));
      }

      // Modify the temperature
      ++improv_iter;
      temperature *= 0.98;
      last_improve = 0;

    } else {
      // Awww. We did not improve

      // If its been 50 iterations since the last improvement, then we should only decrease the temperature
      if (last_improve >= 50) {
        temperature *= 0.99;
        cur_ci = best_ci;
      } else {
        // Otherwise, we do simulated annealing and randomly accept modification based on objective and temperature
        double p = (double)rand() / RAND_MAX;
        double target =
            exp(-(best_obj - new_obj) / temperature);
        if (p < target) {
          // Accept the modification
          std::cout << p << " < " << target << ", accept a worse point!" << std::endl;
          if (cur_ci != best_ci) {
            delete cur_ci;
          }
          cur_ci = cand_ci;
        } else {
          // dont accept the modification
          delete cand_ci;
        }
      }
    }
    temperature = std::max(temperature, 1.0);
    ++last_improve;
    ++i;
  }
  // We have completed everything! Yay!

  std::cout << "DSE Complete!\n";
  std::cout << "Improv Iters: " << improv_iter << "\n";
  best_ci->verify();

  // Print Final Results
  double best_obj = best_ci->weight_obj();
  std::cout << "FINAL DSE OBJ: " << best_obj << "\n";
  auto util = cur_ci->utilization();
  std::cout << std::setprecision(2) << "Utilization ratio overall: " << std::get<0>(util)
            << ", nodes: " << std::get<1>(util) << ", links: " << std::get<2>(util)
            << "\n"
            << std::setprecision(7);

  best_ci->dump_breakdown(ci.verbose);

  // Prune finalized HW
  best_ci->prune_all_unused();

  double pruned_obj = best_ci->weight_obj();

  for (int x = 0; x < best_ci->workload_array.size(); ++x) {
    std::string path = "viz/final_" + std::to_string(x);
    auto sched = best_ci->res[x];
    std::cout << "Dumping " << sched->ssdfg()->filename << " " << path << " " <<  best_ci->dse_sched_obj(sched) << std::endl;
    ENFORCED_SYSTEM(("mkdir -p " + path).c_str());
    std::string filename = path + "/" + std::to_string(x);
    sched->printGraphviz((path + "/" + sched->ssdfg()->filename + ".gv").c_str());
    std::ofstream ofs(filename + ".dfg.h");
    sched->printConfigHeader(ofs, std::to_string(x));
  }

  best_ci->ss_model()->subModel()->DumpHwInJson("viz/pruned.json");
  std::cout << "Pruned DSE OBJ: " << best_ci->weight_obj() << "\n";
  best_ci->dump_breakdown(ci.verbose);

  std::cout << "Total Time: " << static_cast<double>(clock() - start_time) / CLOCKS_PER_SEC << std::endl;

  // Clean up
  ofs.close();
  /*
  for (auto work : cur_ci->workload_array) {
    for (auto sched : work.sched_array) {
      delete sched.ssdfg();
    }
  }
  for (auto capability : cur_ci->ss_model()->fu_types) {
    delete capability;
  }
  */
  delete best_ci;
}

}