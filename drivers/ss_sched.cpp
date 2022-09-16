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

#define EXECUTABLE "ss_sched"

inline bool ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

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
  
  //options.allow_unrecognised_options();
  options.custom_help(
    "[adg] [dfg] [Options...] # Schedule the DFG\n"
    "  " EXECUTABLE " [adg] # Print and estimate the power/area of adg\n"
    "  " EXECUTABLE " [dfg] # print the dfg file\n");

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

  // Get array of all args inputted
  auto args = parsed.unmatched();

  // See if the Argument List is Invalid
  if (args.size() != 1 && args.size() != 2) {
    std::cerr << options.help() << std::endl;
    std::cerr << "But " << args.size() << " arguments get." << std::endl;
    std::cerr << "Exit with 1." << std::endl;
    return 1;
  }

  // See if we are given only the DFG File. If so, print the DFG
  if (args.size() == 1 && ends_with(args[0], ".dfg")) {
    std::string name = args[0].substr(0, args[0].size() - 4);
    DSA_INFO << "DFG-Only Detected. Printing DFG: " << args[0];
    DSA_INFO << "Print with Command:";
    auto dfg = new SSDfg(args[0]);
    dfg->print_graphviz(name + ".gv");
    
    DSA_INFO << "neato -Tpng -overlap=false -Gepsilon=.0001 -o " << name << ".png " << name << ".gv";
    
    delete dfg;
    return 0;
  }

  std::string adg_file = "";
  int dfg_file_number = -1;
  if (ends_with(args[0], ".json")) {
    adg_file = args[0];
    dfg_file_number = 1;
  } else if (args.size() == 2 && ends_with(args[1], ".json")) {
    adg_file = args[1];
    dfg_file_number = 0;
  } else {
    if (args.size() == 1)
      std::cerr << "No ADG File Given. Given: " << args[0] << std::endl;
    else 
      std::cerr << "No ADG File Given. Given: " << args[0] << "," << args[1] << std::endl;

    return 1;
  }
  SSModel ssmodel(adg_file.c_str());
  CodesignInstance* codesign = new CodesignInstance(&ssmodel);

  // Check if we only have adg file
  if (args.size() == 1) {
    DSA_INFO << "ADG-Only Detected. Printing ADG: " << adg_file;
    auto res = codesign->EstimatePowerArea();
    res.Dump(std::cout);
    ssmodel.subModel()->PrintGraphviz(adg_file.substr(0, adg_file.size() - 5) + ".gv");

    delete codesign;
    return 0;
  }

  // Get the DFG File
  std::string dfg_file = args[dfg_file_number];
  
  DSA_CHECK(ends_with(dfg_file, ".dfg") || ends_with(dfg_file, ".list")) << "DFG File must be a .dfg or .list file. Given: " << dfg_file;


  // Check if the DFG File is a list of DFGs
  bool dfg_list = !ends_with(dfg_file, ".dfg");

  // Create Codesign Instance and add workloads
  codesign->add_workloads(dfg_file, dfg_list);

  // Make the vizualization folder to place logfiles and results
  ENFORCED_SYSTEM("mkdir -p viz; mkdir -p .sched");
  
  // Check if running the Design Space Explorer
  if (parsed.count("design-explore")) {
    DSA_INFO << "Running Design Space Exploration with ADG: " << adg_file << " and DFG: " << dfg_file;

    // Make Directories for logging exploration and objectives
    ENFORCED_SYSTEM("mkdir -p viz/iters; touch viz/objectives.csv");
    
    // Explore the Design space
    dsa::DesignSpaceExploration(codesign);
    
    return 0;
  } else {
    DSA_INFO << "Running the Scheduler with ADG: " << adg_file << " and DFG: " << dfg_file;

    // First Create the Scheduler
    auto scheduler = new SchedulerSimulatedAnnealing(codesign->ss_model());
    
    // Schedule all workloads
    bool succeed = scheduler->incrementalSchedule(*codesign);
    
    // Calculate DSE Objective and Get System Performance Parameters
    auto obj = codesign->dse_obj();

    if (!succeed) {
      DSA_INFO << "Scheduling Failed.";
      delete scheduler;
      delete codesign;
      return 1;
    }

    DSA_INFO << "Schedule Performance Objective: " << obj;
    DSA_INFO << "Cores: " << codesign->num_cores << " Banks: " << codesign->num_banks << " System Bus: " << codesign->system_bus;

    auto util = codesign->utilization();
    std::stringstream utilizationStream;
    utilizationStream << std::setprecision(2) << "Utilization ratio overall: " 
              << std::get<0>(util) << ", nodes: " 
              << std::get<1>(util) << ", links: " 
              << std::get<2>(util) << std::setprecision(7);

    DSA_INFO << utilizationStream.str();

    // Dump the Area Model
    auto res = codesign->EstimatePowerArea();
    res.Dump(std::cout);

    // Print the overall scheduled adg
    codesign->printJson("viz/sched_adg.json");
    
    // Now print the individual schedule stats
    codesign->printScheduleStats();

    delete codesign;
    return succeed ? 0 : 1;
  }
}
