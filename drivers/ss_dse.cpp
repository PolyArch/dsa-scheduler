#include <assert.h>
#include <getopt.h>
#include <signal.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "dsa/arch/model.h"
#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/visitor.h"
#include "dsa/mapper/scheduler.h"
#include "dsa/mapper/scheduler_sa.h"

using namespace std;
using sec = chrono::seconds;
using get_time = chrono::steady_clock;

using namespace dsa;

// clang-format off
static struct option long_options[] = {
    {"from-scratch",   no_argument,       nullptr, 's',},
    {"verbose",        no_argument,       nullptr, 'v',},
    {"print-bits",     no_argument,       nullptr, 'b',},
    {"no-int-time",    no_argument,       nullptr, 'n',},
    {"design-space",   no_argument,       nullptr, 'f',},
    {"indir-mem",      no_argument,       nullptr, 'c',},
    {"print-bit",      no_argument,       nullptr, 'b',},
    {"timeout",        required_argument, nullptr, 't',},
    {"max-iters",      required_argument, nullptr, 'i',},
    {"max-edge-delay", required_argument, nullptr, 'd',},
    {"seed",           required_argument, nullptr, 'e',},
    {"decomposer",     required_argument, nullptr, 'r',},
    {"control-flow",   required_argument, nullptr, 'l',},
    {"memory-size",    required_argument, nullptr, 'm',},
    {0, 0, 0, 0,},
};
// clang-format on

Scheduler* scheduler;

int main(int argc, char* argv[]) {
  int opt;
  bool verbose = false;
  int seed = time(0);

  float timeout = 86400.0f;
  int max_edge_delay = 15;
  int max_iters = 20000;

  int indirect = 0;
  bool from_scratch = false;
  int decomposer = 8;
  int contrl_flow = -1;
  int memory_size = 4096;

  while ((opt = getopt_long(argc, argv, "vst:fc:d:e:l:r:m:", long_options, nullptr)) !=
         -1) {
    switch (opt) {
      case 's': from_scratch = true; break;
      case 'v': verbose = true; break;
      case 'c': indirect = atoi(optarg); break;
      case 't': timeout = atof(optarg); break;
      case 'i': max_iters = atoi(optarg); break;
      case 'd': max_edge_delay = atoi(optarg); break;
      case 'e': seed = atoi(optarg); break;
      case 'l': contrl_flow = atoi(optarg); break;
      case 'r': decomposer = atoi(optarg); break;
      case 'm': memory_size = atoi(optarg); break;
      default: exit(1);
    }
  }

  argc -= optind;
  argv += optind;

  if (argc != 2) {
    cerr << "Usage: ss_sched [FLAGS] config.ssmodel compute.ssdfg \n";
    exit(1);
  }

  srand(seed);

  std::string model_filename = argv[0];
  std::string pdg_filename = argv[1];

  SSModel ssmodel(model_filename.c_str());
  ssmodel.setMaxEdgeDelay(max_edge_delay);
  ssmodel.indirect(indirect);
  ssmodel.memory_size = memory_size;

  if (decomposer != -1) {
    for (auto elem : ssmodel.subModel()->node_list()) {
      elem->granularity(elem->datawidth() / decomposer);
    }
  }
  if (contrl_flow != -1) {
    ssmodel.setCtrl(contrl_flow);
  }

  scheduler = new SchedulerSimulatedAnnealing(&ssmodel, timeout, max_iters, verbose);

  clock_t StartTime = clock();
  scheduler->set_start_time();

  CodesignInstance* cur_ci = new CodesignInstance(&ssmodel);
  CodesignInstance* best_ci = cur_ci;
  cur_ci->verify();
  std::vector<WorkloadSchedules*> _incr_sched;

  {
    std::string curline;
    ifstream dfg_names(pdg_filename);
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
            << static_cast<double>(clock() - StartTime) / CLOCKS_PER_SEC << std::endl;
  cur_ci->dump_breakdown(verbose);
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
            << static_cast<double>(clock() - StartTime) / CLOCKS_PER_SEC << std::endl;
  cur_ci->dump_breakdown(verbose);
  {
    // dump the new hw json
    stringstream hw_ss;
    hw_ss << "viz/dse-sched-" << i << ".json";
    cur_ci->ss_model()->subModel()->DumpHwInJson(hw_ss.str().c_str());
  }

  while (i - last_improve <= 750) {
    clock_t StartChange = clock();
    std::cout << " ### Begin DSE Iteration " << i << " ### \n";
    cur_ci->verify();
    CodesignInstance* cand_ci;
    cand_ci = new CodesignInstance(*cur_ci, from_scratch);
    cur_ci->verify();
    cand_ci->verify();
    cand_ci->make_random_modification(temperature);
    cand_ci->verify();
    std::cout << "dse modification: "
              << static_cast<double>(clock() - StartChange) / CLOCKS_PER_SEC << "s"
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

    cand_ci->dump_breakdown(verbose);

    if (cand_ci->weight_obj() > best_ci->weight_obj()) {
      improv_iter = i;
      delete cur_ci;
      best_ci = cur_ci = cand_ci;
      std::cout << "----------------- IMPROVED OBJ! --------------------\n";
      std::cout << "Execution Time: " << std::setprecision(6)
                << static_cast<double>(clock() - StartTime) / CLOCKS_PER_SEC << ", "
                << static_cast<double>(ScheduleCollapse) / CLOCKS_PER_SEC << std::endl;

      for (int x = 0, ew = best_ci->workload_array.size(); x < ew; ++x) {
        ostringstream oss;
        oss << "iter_" << i << "_" << x;
        dump_checkpoint(best_ci->res[x], oss.str(),
                        best_ci->dse_sched_obj(best_ci->res[x]).first);
      }

      // dump the new hw json
      stringstream hw_ss;
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

  cur_ci->dump_breakdown(verbose);

  for (int x = 0, ew = cur_ci->workload_array.size(); x < ew; ++x) {
    ostringstream oss;
    oss << "final_" << x;
    dump_checkpoint(cur_ci->res[x], oss.str(),
                    cur_ci->dse_sched_obj(cur_ci->res[x]).first);
  }

  cur_ci->prune_all_unused();
  cur_ci->ss_model()->subModel()->DumpHwInJson("viz/pruned.json");
  std::cout << "Pruned DSE OBJ: " << cur_ci->weight_obj() << "\n";
  cur_ci->dump_breakdown(verbose);

  std::cout << "Total Time: " << static_cast<double>(clock() - StartTime) / CLOCKS_PER_SEC
            << std::endl;

  return 0;
}
