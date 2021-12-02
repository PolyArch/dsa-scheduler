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
#include "dsa/mapper/dse.h"

using namespace dsa;

Scheduler* scheduler;

#define EXECUTABLE "ss_sched"

int main(int argc, char* argv[]) {
  cxxopts::Options
    options(EXECUTABLE,
            "Mapping data dependence graph of instructions onto spatial architectures.");
  auto default_false = cxxopts::value<bool>()->default_value("false");
  auto default_24_36 = cxxopts::value<int>()->default_value(std::to_string(24 * 3600));
  auto default_20000 = cxxopts::value<int>()->default_value("20000");
  auto default_string = cxxopts::value<std::string>();
  auto default_neg_1 = cxxopts::value<int>()->default_value("-1");
  auto default_d0 = cxxopts::value<double>()->default_value("0.0");
  auto default_1 = cxxopts::value<int>()->default_value("1");

  options.add_options()
    ("v,verbose", "Dump verbosed scheduling log.", default_false)
    ("d,dummy", "Only schedule the i/o ports.", default_false)
    ("x,design-explore", "Design space exploration for the given DFG's.", default_false)
    ("r,tolerate-unuse", "Do not throw an error if there are unused values", default_false)
    ("f,fpga", "Design space exploration for FPGA overlay.", default_false)
    ("b,print-bitstream", "Dump the binary of spatial scheduling.", default_false)
    ("t,timeout", "Kill the scheduling if it times longer than the cutoff.", default_24_36)
    ("m,max-iters", "The maxium iterations of scheduling attemps.", default_20000)
    ("e,seed", "The seed of randomization.", cxxopts::value<int>())
    ("dse-timeout", "The timeout cut-off for design space exploration.", default_neg_1)
    ("w,sched-workers", "The number of workers for scheduling.", default_1)
    ("route-along", "Route along with a master lane when scheduling deomposable arch.", default_d0)
    ("a,compat-adg", "Parse the ADG in compatible version.", default_false)
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
  dsa::ContextFlags::Global().Load(parsed);

  auto args = parsed.unmatched();
  if (args.size() != 1 && args.size() != 2) {
    std::cerr << options.help() << std::endl;
    std::cerr << "But " << args.size() << " arguments get." << std::endl;
    std::cerr << "Exit with 1." << std::endl;
    return 1;
  }

  std::string adg_file = args[0];
  SSModel ssmodel(adg_file.c_str());

  // Evlove the underlying hardware.
  ENFORCED_SYSTEM("mkdir -p .sched;");
  if (parsed.count("design-explore")) {
    ENFORCED_SYSTEM("mkdir -p viz; mkdir -p viz/iters; touch viz/objectives.csv");
    dsa::DesignSpaceExploration(ssmodel, args[1]);
    return 0;
  }

  // Estimate the hardware cost.
  if (args.size() == 1) {
    auto res = dsa::adg::estimation::EstimatePowerAera(&ssmodel);
    std::string model_visual_filename = adg_file + ".gv";
    ofstream os(model_visual_filename.c_str());
    std::cout << "Hardware GV file is " << model_visual_filename << std::endl;
    ssmodel.subModel()->PrintGraphviz(os);
    res.Dump(std::cout);
    return 0;
  }

  // Map the DFG to the spatial architecture.

  DSA_CHECK(args.size() == 2);
  std::string dfg_file = args[1];
  SSDfg dfg(dfg_file);
  scheduler = new SchedulerSimulatedAnnealing(&ssmodel, "", false);

  return scheduler->invoke(&ssmodel, &dfg);
}
