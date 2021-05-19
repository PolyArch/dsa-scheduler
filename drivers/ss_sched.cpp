#include <assert.h>
#include <getopt.h>
#include <signal.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include "cxxopts.hpp"

#include "dsa/arch/estimation.h"
#include "dsa/arch/model.h"
#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/utils.h"
#include "dsa/core/singleton.h"
#include "dsa/mapper/scheduler.h"
#include "dsa/mapper/scheduler_sa.h"

using namespace dsa;

Scheduler* scheduler;

#define EXECUTABLE "ss_sched"

int main(int argc, char* argv[]) {
  cxxopts::Options
    options(EXECUTABLE,
            "Mapping data dependence graph of instructions onto spatial architectures.");

  options.add_options()
    ("v,verbose", "Dump verbosed scheduling log.")
    ("b,print-bitstream", "Dump the binary of spatial scheduling.", cxxopts::value<bool>()->default_value("false"))
    ("t,timeout", "Kill the scheduling if it times longer than the cutoff.", cxxopts::value<int>()->default_value(std::to_string(24 * 3600)))
    ("m,max-iters", "The maxium iterations of scheduling attemps.", cxxopts::value<int>()->default_value("20000"))
    ("e,seed", "The seed of randomization.", cxxopts::value<int>())
    ("h,help", "Print the help information.");
  options.allow_unrecognised_options();
  options.custom_help(
    "[adg] [dfg] [Options...] # Schedule the DFG\n"
    "  " EXECUTABLE " [adg] # Estimate the power/area");

  auto parsed = options.parse(argc, argv);

  if (parsed.count("help")) {
    std::cout << options.help() << std::endl;
    return 0;
  }
  int seed = time(0);
  if (parsed.count("seed")) {
    seed = parsed["seed"].as<int>();
  }
  srand(seed);
  auto &ci = dsa::ContextFlags::Global();
  ci.verbose = parsed.count("verbose");
  ci.timeout = parsed["timeout"].as<int>();
  ci.bitstream = parsed["print-bitstream"].as<bool>();
  ci.max_iters = parsed["max-iters"].as<int>();

  ENFORCED_SYSTEM("mkdir -p .sched");
  auto args = parsed.unmatched();
  if (args.size() != 1 && args.size() != 2) {
    std::cerr << options.help() << std::endl;
    std::cerr << "But " << args.size() << " arguments get." << std::endl;
    std::cerr  << "Exit with 1." << std::endl;
    return 1;
  }

  std::string adg_file = args[0];
  SSModel ssmodel(adg_file.c_str());
  if (args.size() == 1) {
    auto res = dsa::adg::estimation::EstimatePowerAera(&ssmodel);
    std::string model_visual_filename = adg_file + ".gv";
    ofstream os(model_visual_filename.c_str());
    std::cout << "Hardware GV file is " << model_visual_filename << std::endl;
    ssmodel.subModel()->PrintGraphviz(os);
    res.Dump(std::cout);
    return 0;
  }

  CHECK(args.size() == 2);
  std::string dfg_file = args[1];
  SSDfg dfg(dfg_file);
  scheduler = new SchedulerSimulatedAnnealing(&ssmodel, "", false);
  Schedule* sched = scheduler->invoke(&ssmodel, &dfg);

  return 0;
}
