#include <assert.h>
#include <getopt.h>
#include <signal.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include "dsa/arch/model.h"
#include "dsa/arch/estimation.h"
#include "dsa/mapper/scheduler.h"
#include "dsa/mapper/scheduler_sa.h"
#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/utils.h"

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
    {"dump-mapping-if-improved",   no_argument, nullptr, 'u',},
    {"timeout",        required_argument, nullptr, 't',},
    {"max-iters",      required_argument, nullptr, 'i',},
    {"max-edge-delay", required_argument, nullptr, 'd',},
    {"exec-timing",    required_argument, nullptr, 'm',},
    {"seed",           required_argument, nullptr, 'e',},
    {"control-flow",   required_argument, nullptr, 'l',},
    {"decomposer",     required_argument, nullptr, 'r',},
    {"hardware-json",  required_argument, nullptr, 'h',},
    {"software-json",  required_argument, nullptr, 's',},
    {"mapping-json",   required_argument, nullptr, 'a',},
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
  int contrl_flow = -1;

  std::string hw_json_filename = "";
  std::string sw_json_filename = "";
  std::string mapping_json_filename = "";
  bool dump_mapping_if_improved = false;

  while ((opt = getopt_long(argc, argv, "m:vt:pc:bd:e:l:r:h:s:a:u", long_options, nullptr)) != -1) {
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
      case 'h': hw_json_filename = optarg; break;
      case 's': sw_json_filename = optarg; break;
      case 'a': mapping_json_filename = optarg; break;
      case 'u': dump_mapping_if_improved = true; break;
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
  DEBUG(MODEL) << "sub: " << ssmodel.subModel();

  ssmodel.memory_size = memory_size;

  if (max_edge_delay != - 1) {
    ssmodel.setMaxEdgeDelay(max_edge_delay);
  }
  if (contrl_flow != -1) {
    ssmodel.setCtrl(contrl_flow);
  }
  if (decomposer != -1) {
    for (auto elem : ssmodel.subModel()->node_list()) {
      elem->decomposer = decomposer;
    }
  }

  ssmodel.indirect(indirect);

  if (argc == 2) {
    std::string pdg_filename = argv[1];

    scheduler = new SchedulerSimulatedAnnealing(&ssmodel, timeout, max_iters, verbose, mapping_json_filename, dump_mapping_if_improved);

    SSDfg ssdfg(pdg_filename);

    Schedule* sched = scheduler->invoke(&ssmodel, &ssdfg, print_bits);
    if (est_perf) {
      double est = ssdfg.estimated_performance(sched, true);
      std::cout << "Estimated overall performance: " << est << std::endl;
    }
    // Dump hardware
    if(hw_json_filename != "") {
      ssmodel.subModel()->DumpHwInJson(hw_json_filename.c_str());
    }

    // Dump software
    if(!sw_json_filename.empty()){
      for (auto edge : ssdfg.edges()) {
        edge->set_delay(sched->edge_delay(edge));
      }
      dfg::Export(&ssdfg, sw_json_filename);
    }

    // Dump Final Mapping
    if(mapping_json_filename != ""){
      sched->DumpMappingInJson(mapping_json_filename);
    }
  }



  if (est_perf) {
    auto res = dsa::adg::estimation::EstimatePowerAera(&ssmodel);
    res.Dump(std::cout);
  }

  return 0;
}
