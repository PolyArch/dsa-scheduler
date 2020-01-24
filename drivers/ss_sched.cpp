#include <assert.h>
#include <getopt.h>
#include <signal.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include "ss-config/model.h"
#include "ss-scheduler/scheduler.h"
#include "ss-scheduler/scheduler_sa.h"
#include "ss-scheduler/ssdfg.h"

using namespace std;
using sec = chrono::seconds;
using get_time = chrono::steady_clock;

using namespace SS_CONFIG;

// clang-format off
static struct option long_options[] = {
    {"algorithm",      required_argument, nullptr, 'a',},
    {"sub-alg",        required_argument, nullptr, 's',},
    {"verbose",        no_argument,       nullptr, 'v',},
    {"print-bits",     no_argument,       nullptr, 'b',},
    {"no-int-time",    no_argument,       nullptr, 'n',},
    {"design-space",   no_argument,       nullptr, 'f',},
    {"estmt-perf",     no_argument,       nullptr, 'p',},
    {"indir-mem",      no_argument,       nullptr, 'c',},
    {"relative-gap",   required_argument, nullptr, 'r',},
    {"absolute-gap",   required_argument, nullptr, 'g',},
    {"timeout",        required_argument, nullptr, 't',},
    {"max-iters",      required_argument, nullptr, 'i',},
    {"max-edge-delay", required_argument, nullptr, 'd',},
    {"seed",           required_argument, nullptr, 'e',},
    {0, 0, 0, 0,},
};
// clang-format on

Scheduler* scheduler;

int main(int argc, char* argv[]) {
  int opt;
  bool verbose = false;
  int seed = time(0);

  string str_schedType = string("sa");
  string str_subalg = string("");
  bool print_bits = false;

  float absolute_gap = 1.0f;
  float relative_gap = 0.1f;
  float timeout = 86400.0f;
  int max_edge_delay = 15;
  int max_iters = 20000;

  bool is_dse = false;
  bool est_perf = false;
  bool indirect = false;

  while ((opt = getopt_long(argc, argv, "va:s:r:g:t:fpcd:e:", long_options, nullptr)) !=
         -1) {
    switch (opt) {
      case 'a': str_schedType = string(optarg); break;
      case 's': str_subalg = string(optarg); break;
      case 'v': verbose = true; break;
      case 'f': is_dse = true; break;
      case 'p': est_perf = true; break;
      case 'b': print_bits = true; break;
      case 'c': indirect = true; break;

      case 'r': relative_gap = atof(optarg); break;
      case 'g': absolute_gap = atof(optarg); break;
      case 't': timeout = atof(optarg); break;
      case 'i': max_iters = atoi(optarg); break;

      case 'd': max_edge_delay = atoi(optarg); break;
      case 'e': seed = atoi(optarg); break;

      default: exit(1);
    }
  }

  argc -= optind;
  argv += optind;

  if (argc != 2) {
    cerr << "Usage: ss_sched [FLAGS] config.ssmodel compute.ssdfg \n";
    exit(1);
  }

  std::string model_filename = argv[0];
  std::string pdg_filename = argv[1];

  SSModel ssmodel(model_filename.c_str());
  ssmodel.setMaxEdgeDelay(max_edge_delay);
  ssmodel.indirect(indirect);

  if (str_schedType == "sa") { /*simulated annealing*/
    scheduler = new SchedulerSimulatedAnnealing(&ssmodel);
  } else {
    cerr << "Something Went Wrong with Default Scheduler String";
    exit(1);
  }

  scheduler->set_srand(seed);
  scheduler->verbose = verbose;
  scheduler->str_subalg = str_subalg;
  scheduler->setGap(relative_gap, absolute_gap);
  scheduler->setTimeout(timeout);
  scheduler->set_max_iters(max_iters);

  if (is_dse) {
    scheduler->set_start_time();

    CodesignInstance* cur_ci = new CodesignInstance(&ssmodel);
    cur_ci->verify();
    std::vector<WorkloadSchedules*> _incr_sched;

    {
      std::string curline;
      ifstream dfg_names(pdg_filename);
      while (std::getline(dfg_names, curline)) {
        if (curline == "%%") {
          cur_ci->workload_array.emplace_back();
        } else {
          cur_ci->workload_array.back().sched_array.emplace_back(cur_ci->ss_model(),
                                                                 new SSDfg(curline));
        }
      }
    }

    {
      // Filter out useless fu models.
      std::set<SS_CONFIG::OpCode> used_insts;
      for (auto& elem : cur_ci->workload_array) {
        for (auto& dfg : elem.sched_array) {
          std::set<SS_CONFIG::OpCode> delta = dfg.ssdfg()->insts_used();
          for (auto inst : delta) {
            used_insts.insert(inst);
          }
        }
      }
      for (int i = 0, n = ssmodel.fu_types.size(); i < n; ++i) {
        auto& fudef = ssmodel.fu_types[i];
        bool intersect = false;
        for (auto& elem : used_insts) {
          if (fudef.Capable(elem)) {
            intersect = true;
          }
        }
        if (!intersect) {
          ssmodel.fu_types.erase(ssmodel.fu_types.begin() + i);
          --i;
        }
      }
    }

    int max_iters_no_improvement = 300;

    int improv_iter = 0;

    auto dump_checkpoint = [](Schedule* sched, const std::string& filename,
                              double performance) {
      if (!sched) return;
      std::cout << "Dumping " << sched->ssdfg()->filename << " viz/" << filename << "/ "
                << performance << std::endl;
      std::string path = "viz/" + filename;
      assert(system(("mkdir -p " + path).c_str()) == 0);
      sched->printGraphviz((path + "/graph.gv").c_str());
      std::ofstream ofs(path + "/" + filename + ".dfg.h");
      sched->printConfigHeader(ofs, filename);
    };

    for (int i = 0; i < 2000000; ++i) {
      if ((i - improv_iter) > max_iters_no_improvement) {
        break;
      }

      cout << " ### Begin DSE Iteration " << i << " ### \n";
      cur_ci->verify();
      CodesignInstance* cand_ci = new CodesignInstance(*cur_ci);
      cur_ci->verify();
      cand_ci->verify();
      cand_ci->make_random_modification();
      cand_ci->verify();

      scheduler->incrementalSchedule(*cand_ci);
      cand_ci->verify();

      cout << "DSE OBJ: " << cand_ci->dse_obj() << "(" << cur_ci->dse_obj() << ") -- ";

      if (cand_ci->dse_obj() < 1e-6) {
        continue;
      }

      auto* sub = cand_ci->ss_model()->subModel();
      cout << "FUs: " << sub->fu_list().size() << " " << sub->get_fu_total_area()
           << "um2\n"
           << "Switches: " << sub->switch_list().size() << " " << sub->get_sw_total_area()
           << "um2\n"
           << "VPorts: " << sub->vport_list().size() << " " << sub->get_vport_area()
           << "um2\n"
           << "Ctrl: " << cand_ci->ss_model()->host_area() << "um2" << std::endl;

      // std::cout << "======================= ALL
      // ===============================================\n"; for (int x = 0, ew =
      // cand_ci->workload_array.size(); x < ew; ++x) {
      //  for (int y = 0; y < cand_ci->workload_array[x].sched_array.size(); ++y) {
      //    std::cout << i << ": " << x << " " << y << ": "
      //      << cand_ci->dse_sched_obj(&cand_ci->workload_array[x].sched_array[y]).first
      //      << " left: " << cand_ci->workload_array[x].sched_array[y].num_left() <<
      //      std::endl;
      //  }
      //}

      if (cand_ci->dse_obj() > cur_ci->dse_obj()) {
        improv_iter = i;
        delete cur_ci;
        cur_ci = cand_ci;
        cout << "----------------- IMPROVED OBJ! --------------------\n";

        for (int x = 0, ew = cur_ci->workload_array.size(); x < ew; ++x) {
          ostringstream oss;
          oss << "iter_" << i << "_" << x;
          dump_checkpoint(cur_ci->res[x], oss.str(),
                          cur_ci->dse_sched_obj(cur_ci->res[x]).first);
        }

        // dump the new hw json
        stringstream hw_ss;
        hw_ss << "viz/dse-sched-" << i << ".json";
        cur_ci->ss_model()->subModel()->DumpHwInJSON(hw_ss.str().c_str());

      } else {
        delete cand_ci;
      }
    }

    cout << "DSE Complete!\n";
    cout << "Improv Iters: " << improv_iter << "\n";

    scheduler->incrementalSchedule(*cur_ci);
    cur_ci->verify();

    cout << "FINAL DSE OBJ: " << cur_ci->dse_obj() << " -- ";

    auto* sub = cur_ci->ss_model()->subModel();
    cout << "FUs: " << sub->fu_list().size() << " " << sub->get_fu_total_area() << "um2\n"
         << "Switches: " << sub->switch_list().size() << " " << sub->get_sw_total_area()
         << "um2\n"
         << "VPorts: " << sub->vport_list().size() << " " << sub->get_vport_area()
         << "um2\n"
         << "Ctrl: " << cur_ci->ss_model()->host_area() << "um2" << std::endl;

    for (int x = 0, ew = cur_ci->workload_array.size(); x < ew; ++x) {
      ostringstream oss;
      oss << "final_" << x;
      dump_checkpoint(cur_ci->res[x], oss.str(),
                      cur_ci->dse_sched_obj(cur_ci->res[x]).first);
    }

    return 0;
  }

  SSDfg ssdfg(pdg_filename);
  Schedule* sched = scheduler->invoke(&ssmodel, &ssdfg, print_bits);
  if (est_perf) {
    double est = ssdfg.estimated_performance(sched, true);
    std::cout << "Estimated overall performance: " << est << std::endl;
  }
  return 0;
}
