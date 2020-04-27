#include <assert.h>
#include <getopt.h>
#include <signal.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <boost/optional.hpp>

#include "dsa/arch/model.h"
#include "dsa/mapper/scheduler.h"
#include "dsa/mapper/scheduler_sa.h"
#include "dsa/ir/ssdfg.h"

using namespace std;
using sec = chrono::seconds;
using get_time = chrono::steady_clock;

using namespace dsa;

// clang-format off
static struct option long_options[] = {
    {"verbose",        no_argument,       nullptr, 'v',},
    {"print-bits",     no_argument,       nullptr, 'b',},
    {"no-int-time",    no_argument,       nullptr, 'n',},
    {"design-space",   no_argument,       nullptr, 'f',},
    {"estmt-perf",     no_argument,       nullptr, 'p',},
    {"indir-mem",      no_argument,       nullptr, 'c',},
    {"print-bit",      no_argument,       nullptr, 'b',},
    {"timeout",        required_argument, nullptr, 't',},
    {"max-iters",      required_argument, nullptr, 'i',},
    {"max-edge-delay", required_argument, nullptr, 'd',},
    {"exec-timing",    required_argument, nullptr, 'm',},
    {"seed",           required_argument, nullptr, 'e',},
    {"control-flow",   required_argument, nullptr, 'l',},
    {"decomposer",     required_argument, nullptr, 'r',},
    {0, 0, 0, 0,},
};
// clang-format on

Scheduler* scheduler;

int main(int argc, char* argv[]) {
  int opt;
  bool verbose = false;
  int seed = time(0);

  bool print_bits = false;

  float timeout = 86400.0f;
  int max_edge_delay = 15;
  int max_iters = 20000;
  int decomposer = 8;

  bool est_perf = false;
  int indirect = false;
  int memory_size = 4096;

  std::string timing;
  boost::optional<bool> contrl_flow;

  while ((opt = getopt_long(argc, argv, "m:vt:pc:bd:e:l:r:", long_options, nullptr)) != -1) {
    switch (opt) {
      case 'v': verbose = true; break;
      case 'p': est_perf = true; break;
      case 'c': indirect = atoi(optarg); break;
      case 'b': print_bits = true; break;
      case 't': timeout = atof(optarg); break;
      case 'i': max_iters = atoi(optarg); break;
      case 'd': max_edge_delay = atoi(optarg); break;
      case 'm': memory_size = atoi(optarg);
      case 'e': seed = atoi(optarg); break;
      case 'l': contrl_flow = atoi(optarg); break;
      case 'r': decomposer = atoi(optarg); break;
      default: exit(1);
    }
  }

  argc -= optind;
  argv += optind;

  if (!est_perf && argc != 2) {
    cerr << "Usage: ss_sched [FLAGS] config.ssmodel [compute.dfg]\n";
    exit(1);
  }


  srand(seed);

  std::string model_filename = argv[0];
  SSModel ssmodel(model_filename.c_str());

  ssmodel.memory_size = memory_size;

  if (max_edge_delay != - 1) {
    ssmodel.setMaxEdgeDelay(max_edge_delay);
  }
  if (contrl_flow == false || contrl_flow == true) {
    ssmodel.setCtrl(contrl_flow.get());
  }
  if (decomposer != -1) {
    for (auto elem : ssmodel.subModel()->node_list()) {
      elem->decomposer = decomposer;
    }
  }

  ssmodel.indirect(indirect);

  if (argc == 2) {
    std::string pdg_filename = argv[1];

    scheduler = new SchedulerSimulatedAnnealing(&ssmodel, timeout, max_iters, verbose);

    SSDfg ssdfg(pdg_filename);
    Schedule* sched = scheduler->invoke(&ssmodel, &ssdfg, print_bits);
    if (est_perf) {
      double est = ssdfg.estimated_performance(sched, true);
      std::cout << "Estimated overall performance: " << est << std::endl;
    }
  }

  if (est_perf) {
    auto sub = ssmodel.subModel();
    cout << "FUs: " << sub->fu_list().size() << " " << sub->get_fu_total_area() << "um2, "
         <<  sub->get_fu_total_power() << "mw\n"
         << "Switches: " << sub->switch_list().size() << " " << sub->get_sw_total_area() << "um2, "
         <<  sub->get_sw_total_power() << "mw\n"
         << "Sync: " << sub->vport_list().size() << " " << sub->get_sync_area() << "um2, "
         <<  sub->get_sync_power() << "mw\n"
         <<  "Memory: " << ssmodel.memory_area() << "um2, "<< ssmodel.memory_power() << "mw\n"
         << std::endl;
  }

  return 0;
}
