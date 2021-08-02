#include <iomanip>

#include "dsa/core/singleton.h"
#include "dsa/dfg/node.h"
#include "dsa/mapper/dse.h"
#include "dsa/mapper/scheduler_sa.h"

CodesignInstance::CodesignInstance(SSModel* model) : _ssModel(*model) {
  verify();
  unused_nodes = std::vector<bool>(model->subModel()->node_list().size(), true);
  unused_links = std::vector<bool>(model->subModel()->link_list().size(), true);
}

namespace dsa {

void DesignSpaceExploration(SSModel &ssmodel, const std::string &pdg_filename) {

  auto &ci = dsa::ContextFlags::Global();

  auto scheduler = new SchedulerSimulatedAnnealing(&ssmodel);

  clock_t start_time = clock();
  scheduler->set_start_time();

  CodesignInstance* cur_ci = new CodesignInstance(&ssmodel);
  CodesignInstance* best_ci = cur_ci;
  cur_ci->verify();
  std::vector<WorkloadSchedules*> _incr_sched;

  {
    std::string curline;
    std::ifstream dfg_names(pdg_filename);
    while (std::getline(dfg_names, curline)) {
      if (curline == "%%") {
        cur_ci->workload_array.emplace_back();
        cur_ci->weight.push_back(1);
      } else if (curline.find("weight=") == 0) {
        std::istringstream ssin(curline.substr(8, curline.size()));
        ssin >> cur_ci->weight.back();
      } else {
        cur_ci->workload_array.back().sched_array.emplace_back(cur_ci->ss_model(),
                                                               new SSDfg(curline));
      }
    }
  }

  int improv_iter = 0;

  auto dump_checkpoint = [](Schedule* sched, const std::string& filename,
                            double performance) {
    if (!sched) return;
    std::cout << "Dumping " << sched->ssdfg()->filename << " viz/" << filename << "/ "
              << performance << std::endl;
    std::string path = "viz/" + filename;
    ENFORCED_SYSTEM(("mkdir -p " + path).c_str());
    sched->printGraphviz((path + "/graph.gv").c_str());
    std::ofstream ofs(path + "/" + filename + ".dfg.h");
    sched->printConfigHeader(ofs, filename);
  };

  double temperature = 24;
  int i = 0;
  int last_improve = 0;

  {
    double best_indir = -1;
    double best_obj = -1;
    scheduler->incrementalSchedule(*cur_ci);
    for (int indirect = 0; indirect <= 2; ++indirect) {
      cur_ci->ss_model()->indirect(indirect);
      double indir_obj = cur_ci->weight_obj();
      if (indir_obj > best_obj) {
        best_indir = indirect;
        best_obj = indir_obj;
      }
    }
    cur_ci->ss_model()->indirect(best_indir);
  }

  std::cout << " ### Begin DSE Iteration " << i << " ### \n"
            << "DSE OBJ: " << cur_ci->weight_obj() << std::endl
            << "Execution Time: "
            << static_cast<double>(clock() - start_time) / CLOCKS_PER_SEC << std::endl;
  cur_ci->dump_breakdown(ci.verbose);
  ++i;

  {
    auto& ssmodel = *cur_ci->ss_model();
    // Filter out useless fu models.
    std::set<dsa::OpCode> used_insts;
    for (auto& elem : cur_ci->workload_array) {
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

    for (int i = 0; i < (int)ssmodel.fu_types.size(); ++i) {
      auto& fudef = ssmodel.fu_types[i];
      for (int j = 0; j < (int)fudef->capability.size(); ++j) {
        if (used_insts.find(fudef->capability[j].op) == used_insts.end()) {
          fudef->Erase(j);
          --j;
        }
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
    }
  }
  std::cout << " ### Begin DSE Iteration " << ++i << " ### \n"
            << "DSE OBJ: " << cur_ci->weight_obj() << std::endl
            << "Execution Time: "
            << static_cast<double>(clock() - start_time) / CLOCKS_PER_SEC << std::endl;
  cur_ci->dump_breakdown(ci.verbose);
  {
    // dump the new hw json
    std::stringstream hw_ss;
    hw_ss << "viz/dse-sched-" << i << ".json";
    cur_ci->ss_model()->subModel()->DumpHwInJson(hw_ss.str().c_str());
  }

  while (i - last_improve <= 750) {
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
    std::cout << " ### Begin DSE Iteration " << i << " ### \n";
    cur_ci->verify();
    CodesignInstance* cand_ci;
    cand_ci = new CodesignInstance(*cur_ci, false);
    cur_ci->verify();
    cand_ci->verify();
    cand_ci->make_random_modification(temperature);
    cand_ci->verify();
    std::cout << "dse modification: "
              << static_cast<double>(clock() - current_time) / CLOCKS_PER_SEC << "s"
              << std::endl;

    clock_t StartSchedule = clock();
    scheduler->incrementalSchedule(*cand_ci);
    clock_t ScheduleCollapse = clock() - StartSchedule;
    cand_ci->verify();

    double obj_func = cand_ci->weight_obj();
    double best_obj = best_ci->weight_obj();
    double init_obj = cur_ci->weight_obj();

    std::cout << "DSE OBJ: " << obj_func << "(" << best_obj << ") (" << init_obj << ")"
              << std::endl;
    
    auto util = cand_ci->utilization();

    std::cout << std::setprecision(2)
              << "Utilization ratio overall: " << std::get<0>(util)
              << ", nodes: " << std::get<1>(util) << ", links: " << std::get<2>(util)
              << "\n"
              << std::setprecision(7);

    if (obj_func < (1.0 + 1e-3)) {
      continue;
    }

    cand_ci->dump_breakdown(ci.verbose);

    if (cand_ci->weight_obj() > best_ci->weight_obj()) {
      improv_iter = i;
      delete cur_ci;
      best_ci = cur_ci = cand_ci;
      std::cout << "----------------- IMPROVED OBJ! --------------------\n";
      std::cout << "Execution Time: " << std::setprecision(6)
                << static_cast<double>(clock() - start_time) / CLOCKS_PER_SEC << ", "
                << static_cast<double>(ScheduleCollapse) / CLOCKS_PER_SEC << std::endl;

      for (int x = 0, ew = best_ci->workload_array.size(); x < ew; ++x) {
        std::ostringstream oss;
        oss << "iter_" << i << "_" << x;
        dump_checkpoint(best_ci->res[x], oss.str(),
                        best_ci->dse_sched_obj(best_ci->res[x]).first);
      }

      // dump the new hw json
      std::ostringstream hw_ss;
      hw_ss << "viz/dse-sched-" << i << ".json";
      best_ci->ss_model()->subModel()->DumpHwInJson(hw_ss.str().c_str());
      temperature *= 0.98;
      last_improve = i;

    } else {
      if (i - last_improve >= 50) {
        temperature *= 0.99;
        cur_ci = best_ci;
      } else {
        double p = (double)rand() / RAND_MAX;
        double target =
            exp(-(best_ci->weight_obj() - cand_ci->weight_obj()) / temperature);
        if (p < target) {
          std::cout << p << " < " << target << ", accept a worse point!" << std::endl;
          if (cur_ci != best_ci) {
            delete cur_ci;
          }
          cur_ci = cand_ci;
        } else {
          delete cand_ci;
        }
      }
    }
    temperature = std::max(temperature, 1.0);
    ++i;
  }

  std::cout << "DSE Complete!\n";
  std::cout << "Improv Iters: " << improv_iter << "\n";

  cur_ci = best_ci;
  cur_ci->verify();

  double best_obj = cur_ci->weight_obj();
  std::cout << "FINAL DSE OBJ: " << best_obj << "\n";
  auto util = cur_ci->utilization();
  std::cout << std::setprecision(2) << "Utilization ratio overall: " << std::get<0>(util)
            << ", nodes: " << std::get<1>(util) << ", links: " << std::get<2>(util)
            << "\n"
            << std::setprecision(7);

  cur_ci->dump_breakdown(ci.verbose);

  for (int x = 0, ew = cur_ci->workload_array.size(); x < ew; ++x) {
    std::ostringstream oss;
    oss << "final_" << x;
    dump_checkpoint(cur_ci->res[x], oss.str(), cur_ci->dse_sched_obj(cur_ci->res[x]).first);
  }

  cur_ci->prune_all_unused();
  cur_ci->ss_model()->subModel()->DumpHwInJson("viz/pruned.json");
  std::cout << "Pruned DSE OBJ: " << cur_ci->weight_obj() << "\n";
  cur_ci->dump_breakdown(ci.verbose);

  std::cout << "Total Time: " << static_cast<double>(clock() - start_time) / CLOCKS_PER_SEC
            << std::endl;
}

}
