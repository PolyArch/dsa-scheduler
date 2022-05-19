#include <iostream>
#include <iomanip>
#include <ctpl_stl.h>

#include "dsa/core/singleton.h"
#include "dsa/dfg/node.h"
#include "dsa/mapper/dse.h"
#include "dsa/mapper/scheduler_sa.h"

CodesignInstance::CodesignInstance(SSModel* model) : _ssModel(*model) {
  unused_nodes = std::vector<bool>(model->subModel()->node_list().size(), true);
  unused_links = std::vector<bool>(model->subModel()->link_list().size(), true);
}

namespace dsa {

const std::string log_header = "Time,Iteration,Meaningful Iterations,Temperature,Current Objective Performance,Best Objective Performance,Current Objective Area,Best Objective Area,Current Number of Cores,Best Number of Cores,Current Number of Banks,Best Number of Banks,Current System Bus Size,Best System Bus Size,Current Single-Core Normalized Resources,Best Single-Core Normalized Resources,Current Normalized Resources,Best Normalized Resources,Current Nodes,Best Nodes,Current Links,Best Links,Current Util Overall,Current Link Util,Current Node Util,Best Util Overall,Best Link Util,Best Node Util,DSE Fail Reason,DSE Change Details,Current Workload Weights,Best Workload Weights,Current Workload Performance,Best Workload Performance,Current DFG Performances,Best DFG Performances,Current SPM DFG Performances,Best SPM DFG Performances,Current L2 DFG Performances,Best L2 DFG Performances,Current DRAM DFG Performances,Best DRAM DFG Performances,Current Single-Core Resources,Best Single-Core Resources,Current Resources,Best Resources,Current Functional Unit Resources,Current Switch Resources,Current IVPort Resources,Current OVPort Resources,Current Scratchpad Resources,Current DMA Resources,Current Recurrance Resources,Current Generate Resources,Current Register Resources,Current Core Resources,Current System Bus Resources,Best Functional Unit Resources,Best Switch Resources,Best IVPort Resources,Best OVPort Resources,Best Scratchpad Resources,Best DMA Resources,Best Recurrance Resources,Best Generate Resources,Best Register Resources,Best Core Resources,Best System Bus Resources";

// Print Resources for given type
std::string resources(int type, dsa::adg::estimation::Result& estimated) {
  
  std::ostringstream s;
  switch(type) {
    case 0: { // Sum 
      auto sum = estimated.sum()->clone();
      s << sum->dump();
      delete sum;
      break;
    }
    case 1: { // FU
      auto fu = estimated.resource_bd(0)->clone();
      s << fu->dump();
      delete fu;
      break;
    }
    case 2: { // Switch
      auto sw = estimated.resource_bd(1)->clone();
      s << sw->dump();
      delete sw;
      break;
    }
    case 3: { // IVPort
      auto ivport = estimated.resource_bd(2)->clone();
      s << ivport->dump();
      delete ivport;
      break;
    }
    case 4: { // OVPort
      auto ovport = estimated.resource_bd(3)->clone();
      s << ovport->dump();
      delete ovport;
      break;
    }
    case 5: { // Scratchpad
      auto scratch = estimated.resource_bd(4)->clone();
      s << scratch->dump();
      delete scratch;
      break;
    }
    case 6: { // DMA
      auto dma = estimated.resource_bd(5)->clone();
      s << dma->dump();
      delete dma;
      break;
    }
    case 7: { // Recurrance
      auto recurrance = estimated.resource_bd(6)->clone();
      s << recurrance->dump();
      delete recurrance;
      break;
    }
    case 8: { // Generate
      auto generate = estimated.resource_bd(7)->clone();
      s << generate->dump();
      delete generate;
      break;
    }
    case 9: { // Register
      auto registers = estimated.resource_bd(8)->clone();
      s << registers->dump();
      delete registers;
      break;
    }
    case 10: { // Core
      auto core = estimated.resource_bd(9)->clone();
      s << core->dump();
      delete core;
      break;
    }
    case 11: { // System Bus
      auto system_bus = estimated.resource_bd(10)->clone();
      s << system_bus->dump();
      delete system_bus;
      break;
    }
    case 12: { // Normalize Sum 
      auto sum = estimated.sum()->clone();
      sum->normalize();
      s << sum->dump();
      delete sum;
      break;
    }
    case 13: { // FU
      auto fu = estimated.resource_bd(0)->clone();
      fu->normalize();
      s << fu->dump();
      delete fu;
      break;
    }
    case 14: { // Switch
      auto sw = estimated.resource_bd(1)->clone();
      sw->normalize();
      s << sw->dump();
      delete sw;
      break;
    }
    case 15: { // IVPort
      auto ivport = estimated.resource_bd(2)->clone();
      ivport->normalize();
      s << ivport->dump();
      delete ivport;
      break;
    }
    case 16: { // OVPort
      auto ovport = estimated.resource_bd(3)->clone();
      ovport->normalize();
      s << ovport->dump();
      delete ovport;
      break;
    }
    case 17: { // Scratchpad
      auto scratch = estimated.resource_bd(4)->clone();
      scratch->normalize();
      s << scratch->dump();
      delete scratch;
      break;
    }
    case 18: { // DMA
      auto dma = estimated.resource_bd(5)->clone();
      dma->normalize();
      s << dma->dump();
      delete dma;
      break;
    }
    case 19: { // Recurrance
      auto recurrance = estimated.resource_bd(6)->clone();
      recurrance->normalize();
      s << recurrance->dump();
      delete recurrance;
      break;
    }
    case 20: { // Generate
      auto generate = estimated.resource_bd(7)->clone();
      generate->normalize();
      s << generate->dump();
      delete generate;
      break;
    }
    case 21: { // Register
      auto registers = estimated.resource_bd(8)->clone();
      registers->normalize();
      s << registers->dump();
      delete registers;
      break;
    }
    case 22: { // Core
      auto core = estimated.resource_bd(9)->clone();
      core->normalize();
      s << core->dump();
      delete core;
      break;
    }
    case 23: { // System Bus
      auto system_bus = estimated.resource_bd(10)->clone();
      system_bus->normalize();
      s << system_bus->dump();
      delete system_bus;
      break;
    }
  }
  return s.str();
}


std::string get_dfg_names(CodesignInstance* ci) {
  ci->dse_obj();
  std::ostringstream s;
  for (int x = 0; x < ci->workload_array.size(); ++x) {
    s << ci->res[x]->ssdfg()->filename;
    if (x != ci->workload_array.size() - 1) {
      s << ",";
    }
  }
  return s.str();
}

std::string dump_log(const double& time, const int& iteration, const int& last_improve, const double& temp, CodesignInstance* curr_ci, CodesignInstance* best_ci) {
  auto curr_util = curr_ci->utilization();
  auto best_util = best_ci->utilization();

  auto best_estimated = dsa::adg::estimation::EstimatePowerAera(best_ci->ss_model());
  auto curr_estimated = dsa::adg::estimation::EstimatePowerAera(curr_ci->ss_model());
  best_estimated.add_core_overhead();
  curr_estimated.add_core_overhead();


  std::string curr_single_core_resources = resources(0, curr_estimated);
  std::string best_single_core_resources = resources(0, best_estimated);

  std::string curr_single_core_normalized_resources = resources(12, curr_estimated);
  std::string best_single_core_normalized_resources = resources(12, best_estimated);

  best_estimated.scale_cores(best_ci->num_cores);
  curr_estimated.scale_cores(curr_ci->num_cores);
  
  best_estimated.add_system_bus_overhead(best_ci->num_cores, best_ci->num_banks, best_ci->system_bus);
  curr_estimated.add_system_bus_overhead(curr_ci->num_cores, curr_ci->num_banks, curr_ci->system_bus);
  
  std::ostringstream s;
  s << time << ","
    << iteration << ","
    << last_improve << "," 
    << temp << ","
    << curr_ci->weight_obj().first << ","
    << best_ci->weight_obj().first << ","
    << curr_ci->weight_obj().second << ","
    << best_ci->weight_obj().second << ","
    << curr_ci->num_cores << ","
    << best_ci->num_cores << ","
    << curr_ci->num_banks << ","
    << best_ci->num_banks << ","
    << curr_ci->system_bus << ","
    << best_ci->system_bus << ",\""
    << curr_single_core_normalized_resources << "\",\""
    << best_single_core_normalized_resources << "\",\""
    << resources(12, curr_estimated) << "\",\""
    << resources(12, best_estimated) << "\","
    << curr_ci->ss_model()->subModel()->node_list().size() << ","
    << best_ci->ss_model()->subModel()->node_list().size() << ","
    << curr_ci->ss_model()->subModel()->link_list().size() << ","
    << best_ci->ss_model()->subModel()->link_list().size() << ","
    << std::get<0>(curr_util) << ","
    << std::get<1>(curr_util) << ","
    << std::get<2>(curr_util) << ","
    << std::get<0>(best_util) << ","
    << std::get<1>(best_util) << ","
    << std::get<2>(best_util) << ","
    << curr_ci->dse_fail_reason << ",\""
    << curr_ci->get_changes_log() << "\",\""
    << curr_ci->get_workload_weights() << "\",\""
    << best_ci->get_workload_weights() << "\",\""
    << curr_ci->get_workload_performances() << "\",\""
    << best_ci->get_workload_performances() << "\",\""
    << curr_ci->get_dfg_performances() << "\",\""
    << best_ci->get_dfg_performances() << "\",\""
    << curr_ci->get_spm_performances() << "\",\""
    << best_ci->get_spm_performances() << "\",\""
    << curr_ci->get_l2_performances() << "\",\""
    << best_ci->get_l2_performances() << "\",\""
    << curr_ci->get_dram_performances() << "\",\""
    << best_ci->get_dram_performances() << "\",\""
    << curr_single_core_resources << "\",\"" 
    << best_single_core_resources << "\",\""
    << resources(0, curr_estimated) << "\",\""
    << resources(0, best_estimated) << "\",\""
    << resources(1, curr_estimated) << "\",\"" 
    << resources(2, curr_estimated) << "\",\"" 
    << resources(3, curr_estimated) << "\",\""
    << resources(4, curr_estimated) << "\",\""
    << resources(5, curr_estimated) << "\",\""
    << resources(6, curr_estimated) << "\",\""
    << resources(7, curr_estimated) << "\",\""
    << resources(8, curr_estimated) << "\",\""
    << resources(9, curr_estimated) << "\",\""
    << resources(10, curr_estimated) << "\",\""
    << resources(11, curr_estimated) << "\",\""
    << resources(1, best_estimated) << "\",\""
    << resources(2, best_estimated) << "\",\""
    << resources(3, best_estimated) << "\",\""
    << resources(4, best_estimated) << "\",\""
    << resources(5, best_estimated) << "\",\""
    << resources(6, best_estimated) << "\",\""
    << resources(7, best_estimated) << "\",\""
    << resources(8, best_estimated) << "\",\""
    << resources(9, best_estimated) << "\",\""
    << resources(10, best_estimated) << "\",\""
    << resources(11, best_estimated) << "\"";

  return s.str();
}

void dump_hw(CodesignInstance*& ci, int i, int max_vector_size) {
  std::stringstream hw_ss;
  if (max_vector_size=-1)
    hw_ss << "viz/iters/dse-sched-" << i << ".json";
  else
    hw_ss << "viz/iters/" << max_vector_size << "/dse-sched-" << i << ".json";
  
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
      ci->weight.back() = std::stod(curline.substr(curline.find("=") + 1));
    } else {
      DSA_INFO << "Parsing: " << curline;
      ci->workload_array.back().sched_array.emplace_back(ci->ss_model(), new SSDfg(curline));
    }
  }
}

bool checkIndirect(CodesignInstance*& ci) {
  for (auto& w : ci->workload_array) {
    for (auto& sched : w.sched_array) {
      auto dfg = sched.ssdfg();
      for (auto port : dfg->vins) {
        if (port.indirect())
          return true;
      }
      for (auto port : dfg->vouts) {
        if (port.indirect())
          return true;
      }
    }
  }
  return false;
}

void initialize_indirect(CodesignInstance*& ci) {
  if (checkIndirect(ci)) {
    for (auto data : ci->ss_model()->subModel()->data_list()) {
      data->indirectIndexStream(true);
      data->indirectLength1DStream(true);
      data->indirectStride2DStream(true);
    }
  } else {
    for (auto data : ci->ss_model()->subModel()->data_list()) {
      data->indirectIndexStream(false);
      data->indirectLength1DStream(false);
      data->indirectStride2DStream(false);
    }
  }
}


void dump_schedules(CodesignInstance*& ci, std::string base_path) {
  for (int i = 0; i < ci->workload_array.size(); i++) {
    std::string path = base_path + std::to_string(i);
    auto sched = ci->res[i];
    if (sched == nullptr) continue;
    
    DSA_INFO << "Dumping "<< sched->ssdfg()->filename 
              << " at " << path;
    ENFORCED_SYSTEM(("mkdir -p " + path).c_str());
    std::string filename = path + "/" + std::to_string(i);
    sched->printGraphviz((path + "/graph.gv").c_str());
    std::ofstream ofs(filename + ".dfg.h");
    DSA_CHECK(ofs.good()) << filename << ".dfg.h" << " not opened!";
    sched->printConfigHeader(ofs, std::to_string(i));
    ofs.close();
  }
}

void setup_indirect(CodesignInstance*& ci, SchedulerSimulatedAnnealing*& scheduler, int max_vector_size) {
  double best_indir = -1;
  std::pair<double, double> best_obj{-1, -1};
  scheduler->incrementalSchedule(*ci, max_vector_size);
  for (int indirect = 0; indirect <= 2; ++indirect) {
    ci->ss_model()->indirect(indirect);
    std::pair<double, double> indir_obj = ci->dse_obj(false);
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
  std::set<dsa::OpCode> adg_insts;
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

  for (int i = 0; i < (int)ssmodel.subModel()->fu_list().size(); ++i) {
    auto* fu = ssmodel.subModel()->fu_list()[i];
    for (int j = 0; j < (int)fu->fu_type().capability.size(); ++j) {
      adg_insts.insert(fu->fu_type().capability[j].op);
    }
  }

  for (auto& inst : used_insts) {
    if (adg_insts.find(inst) == adg_insts.end()) {
      DSA_CHECK(false) << "Instruction " << dsa::name_of_inst(inst) << " is not used in ADG";
    }
  }

  used_insts.insert(dsa::OpCode::SS_Copy);

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
    for (int j = 0; j < (int)fu->fu_type().capability.size(); ++j) {
      if (used_insts.find(fu->fu_type().capability[j].op) == used_insts.end()) {
          auto fu_type = fu->fu_type();
          fu_type.Erase(j);
          fu->fu_type(fu_type);
          --j;
      }
    }
    if ((int)fu->fu_type().capability.size() == 0) {
      //DSA_CHECK(false);
    } else if ((int)fu->fu_type().capability.size() > used_insts.size()) {
      DSA_CHECK(false) << "Instruction Capability " << fu->fu_type().capability.size() << " is larger than used instructions " << used_insts.size();
    }
  }
  
}

bool Compare(std::pair<double, double> a, std::pair<double, double> b) {
  if (a.first > b.first) return true;
  if (a.first == b.first) return a.second < b.second;
  return false;
}

void VectorDesignSpaceExploration(SSModel &ssmodel, const std::string &pdg_filename, int max_size) {
  std::vector<int> sizes = {};
  int max_size_log = std::log2(max_size);
  for (int i = 0; i <= max_size_log; ++i) {
    sizes.push_back(1 << i);
  }


  CodesignInstance* cur_ci = new CodesignInstance(&ssmodel);
  initialize_workloads(cur_ci, pdg_filename);

  ctpl::thread_pool workers(sizes.size());
  for (int size : sizes) {
    DSA_INFO << "Starting size: " << size;
    ENFORCED_SYSTEM(("mkdir -p viz/" + std::to_string(size)).c_str());
    ENFORCED_SYSTEM(("mkdir -p viz/iters/" + std::to_string(size)).c_str());
    CodesignInstance* vector_ci = new CodesignInstance(*cur_ci, true);
    vector_ci->max_vector = size;
    if (vector_ci->countSchedules() == 0) {
      delete vector_ci;
      continue;
    }
    workers.push([&ssmodel, vector_ci, size](int id) {
      ExploreDesign(vector_ci, ssmodel, size);
    });
  }
}

void DesignSpaceExploration(SSModel &ssmodel, const std::string &pdg_filename, int max_vector_size) {

  // Set up Codesign Instance
  CodesignInstance* cur_ci = new CodesignInstance(&ssmodel);
  initialize_workloads(cur_ci, pdg_filename);
  cur_ci->max_vector = max_vector_size;

  ExploreDesign(cur_ci, ssmodel, max_vector_size);
}


void ExploreDesign(CodesignInstance* cur_ci, SSModel& ssmodel, int max_vector_size) {
  DSA_INFO << "Vector Size: " << max_vector_size;
  // Create Objective CSV File
  std::string path;
  if (max_vector_size == -1)
    path = "viz/objectives.csv";
  else 
    path = "viz/objectives_" + std::to_string(max_vector_size) + ".csv";
  
  ofstream objectives(path);

  DSA_CHECK(objectives.good()) << path << " not opened!";
  objectives << log_header << std::endl;

  // Set up Scheduler
  auto &ci = dsa::ContextFlags::Global();
  auto scheduler = new SchedulerSimulatedAnnealing(&ssmodel);
  clock_t start_time = clock();
  scheduler->set_start_time();

  setup_indirect(cur_ci, scheduler, max_vector_size);
  initialize_indirect(cur_ci);
  filter_useless_function_units(cur_ci);

  if (cur_ci->workload_array.size() == 0) {
    DSA_INFO << "No workloads found!";
    return;
  }
  for (int i = 0; i < cur_ci->workload_array.size(); i++) {
    if (cur_ci->workload_array[i].sched_array.size() == 0) {
      DSA_INFO << "Workload " << i << " has no schedules";
      return;
    }
  }

  // Schedule First Node
  clock_t StartSchedule = clock();
  scheduler->incrementalSchedule(*cur_ci, max_vector_size);
  clock_t ScheduleCollapse = clock() - StartSchedule;

  CodesignInstance* best_ci = cur_ci;

  // Create DSE Loop Variables
  int improv_iter = 0;
  double temperature = 25;
  int i = 0;
  int last_improve = 0;
  std::pair<double, double> last_best_obj{-999, -999};

  // Dump Start log and hw;
  objectives << dump_log(static_cast<double>(clock() - start_time) / CLOCKS_PER_SEC, i, last_improve, temperature, cur_ci, cur_ci) << std::endl;
  dump_hw(cur_ci, i, max_vector_size);

  // Start DSE Iterations. Will end if it doesn't improve in 750 meaningful iterations
  while (last_improve < 750) {
    DSA_INFO << "--- ### Begin DSE Iteration " << i << " (" << std::setprecision(2) << temperature << ") ### ---";

    // Setup Next Iteration
    clock_t current_time = clock();
    double time_elps = static_cast<double>(current_time - start_time) / CLOCKS_PER_SEC;
    if (dsa::ContextFlags::Global().dse_timeout != -1) {
      if (time_elps > dsa::ContextFlags::Global().dse_timeout) {
        DSA_INFO
          << time_elps << "s elapsed, the cutoff is "
          << dsa::ContextFlags::Global().dse_timeout
          << "s, break DSE";
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
    DSA_INFO << "dse modification: "
              << static_cast<double>(clock() - current_time) / CLOCKS_PER_SEC << "s";

    // Schedule the Modification
    clock_t StartSchedule = clock();
    scheduler->incrementalSchedule(*cand_ci, max_vector_size);
    clock_t ScheduleCollapse = clock() - StartSchedule;
    cand_ci->verify();

    // Calcuate Objective
    std::pair<double, double> new_obj = cand_ci->weight_obj();
    std::pair<double, double> best_obj = best_ci->weight_obj();
    std::pair<double, double> init_obj = cur_ci->weight_obj();

    DSA_CHECK(Compare(best_obj, last_best_obj)) << " best obj went down from " << last_best_obj << " to " << best_obj;

    // Print Objectives
    DSA_INFO << "DSE OBJ: " << std::setprecision(4) << new_obj << "(Best:" << best_obj << ") (Iteration:" << init_obj << ")";
    
    auto util = cand_ci->utilization();

    std::stringstream utilizationStream;
    utilizationStream << std::setprecision(2) << "Utilization ratio overall: " 
              << std::get<0>(util) << ", nodes: " 
              << std::get<1>(util) << ", links: " 
              << std::get<2>(util) << std::setprecision(7);

    DSA_INFO << utilizationStream.str();

    objectives << dump_log(time_elps, i, last_improve, temperature, cand_ci, best_ci) << std::endl;

    // Reject a bad modification that causes everything not to schedule
    
    if (new_obj.first < 2) {
      delete cand_ci;
      ++i;
      continue;
    }
    

    cand_ci->dump_breakdown(ci.verbose);

    if (Compare(new_obj, best_obj)) {
      // Yay! We have improved the solution
      
      last_best_obj = best_obj;

      // Change the Codesign Instances
      delete cur_ci;
      best_ci = cur_ci = cand_ci;

      // Print the Improvement
      DSA_INFO << "----------------- IMPROVED OBJ! --------------------\n" <<
                  "New Objective: " << new_obj << " (from: " << best_obj << ")";
      DSA_INFO << "Execution Time: " << std::setprecision(6)
                << static_cast<double>(clock() - start_time) / CLOCKS_PER_SEC 
                << ", "
                << static_cast<double>(ScheduleCollapse) / CLOCKS_PER_SEC;

      // dump the new hw json
      dump_hw(best_ci, i, max_vector_size);

      if (max_vector_size == -1) {
        std::ostringstream oss;
        oss << "viz/iters/iter_" << i << "/sched_";
        dump_schedules(best_ci, oss.str());
      } else {
        std::ostringstream oss;
        oss << "viz/iters/" << max_vector_size << "/iter_" << i << "/sched_";
        dump_schedules(best_ci, oss.str());
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
            exp(-((best_obj.first - new_obj.first) * 100) / temperature);
        if (p < target) {
          // Accept the modification
          DSA_INFO << p << " < " << target << ", accept a worse point!";
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

  DSA_INFO << "DSE Complete!";
  DSA_INFO << "Improv Iters: " << improv_iter;
  best_ci->verify();

  // Print Final Results
  std::pair<double, double> best_obj = best_ci->weight_obj();
  DSA_INFO << "FINAL DSE OBJ: " << best_obj;
  auto util = cur_ci->utilization();

  std::stringstream utilizationStream;
  utilizationStream << std::setprecision(2) << "Utilization ratio overall: " 
            << std::get<0>(util) << ", nodes: " 
            << std::get<1>(util) << ", links: " 
            << std::get<2>(util) << std::setprecision(7);

  
  DSA_INFO << utilizationStream.str();

  CodesignInstance* prunned_ci = new CodesignInstance(*best_ci, false);
  prunned_ci->verify();
  
  //scheduler->incrementalSchedule(*prunned_ci);
  prunned_ci->all_stated_collapse();
  prunned_ci->prune_all_unused();

  //scheduler->incrementalSchedule(*prunned_ci);

  DSA_INFO << "Pruned DSE OBJ: " << prunned_ci->weight_obj();

  std::pair<double, double> pruned_obj = prunned_ci->weight_obj();
  auto final_util = best_ci->utilization();
  auto pruned_util = prunned_ci->utilization();

  DSA_INFO << std::setprecision(2) << "Prunned utilization ratio overall: " 
           << std::get<0>(pruned_util) << ", nodes: " 
           << std::get<1>(pruned_util) << ", links: " 
           << std::get<2>(pruned_util) << std::setprecision(7);

  DSA_INFO << "BEST CI DSE OBJ: " << best_obj;
  DSA_INFO << std::setprecision(2) << "Best utilization ratio overall: " 
           << std::get<0>(final_util) << ", nodes: " 
           << std::get<1>(final_util) << ", links: " 
           << std::get<2>(final_util) << std::setprecision(7);

  if (max_vector_size == -1) {
    dump_schedules(prunned_ci, "viz/final_prunned_");
    dump_schedules(best_ci, "viz/final_");
  } else {
    dump_schedules(prunned_ci, "viz/" + std::to_string(max_vector_size) + "/final_prunned_");
    dump_schedules(best_ci, "viz/" + std::to_string(max_vector_size) + "/final_");
  }

  if (max_vector_size == -1) {
    best_ci->ss_model()->subModel()->DumpHwInJson("viz/final.json");
    prunned_ci->ss_model()->subModel()->DumpHwInJson("viz/prunned.json");
  } else {
    std::string final = "viz/" + std::to_string(max_vector_size) + "/final.json";
    std::string prunned = "viz/" + std::to_string(max_vector_size) + "/prunned.json";
    best_ci->ss_model()->subModel()->DumpHwInJson(final.c_str());
    prunned_ci->ss_model()->subModel()->DumpHwInJson(prunned.c_str());
  }
  best_ci->dump_breakdown(ci.verbose);
  if (prunned_ci->dse_obj().first < 2) {
    DSA_INFO << "Prunned Obj is less than 1";
  }
  //DSA_CHECK(prunned_ci->dse_obj() > 1) << "Pruned obj is less than 1";

  DSA_INFO << "Total Time: " << static_cast<double>(clock() - start_time) / CLOCKS_PER_SEC;

  // Clean up
  if (objectives.is_open()) {
    objectives.close();
  }
  delete best_ci;
  delete prunned_ci;
  DSA_INFO << "DSE Finished!";
}

}
