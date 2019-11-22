#include <getopt.h>

#include <assert.h>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <string>
#include <chrono>
#include <iostream>
#include <signal.h>

#include "ss-config/model.h"

#include "ss-scheduler/scheduler.h"
#include "ss-scheduler/scheduler_gams.h"
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
    {"show-gams",      no_argument,       nullptr, 'G',},
    {"mipstart",       no_argument,       nullptr, 'm',},
    {"sll",            no_argument,       nullptr, 'S',},
    {"no-int-time",    no_argument,       nullptr, 'n',},
    {"relative-gap",   required_argument, nullptr, 'r',},
    {"absolute-gap",   required_argument, nullptr, 'g',},
    {"timeout",        required_argument, nullptr, 't',},
    {"max-iters",      required_argument, nullptr, 'i',},
    {"max-edge-delay", required_argument, nullptr, 'd',},
    {"seed",           required_argument, nullptr, 'e',},
    {"fake-incr",      required_argument, nullptr, 'f',},
    {0, 0, 0, 0,},
};
// clang-format on

Scheduler* scheduler;

int main(int argc, char* argv[]) {
  int opt;
  bool verbose = false;
  bool show_gams = false, mipstart = false, sll = false;
  int seed = 0;

  string str_schedType = string("sa");
  string str_subalg = string("");
  bool print_bits = false;

  float absolute_gap = 1.0f;
  float relative_gap = 0.1f;
  float timeout = 86400.0f;
  int max_edge_delay = 15;
  int max_iters = 20000;

  int fake_incr=0;

  while ((opt = getopt_long(argc, argv, "vGa:s:r:g:t:mf:d:e:", 
          long_options, nullptr)) != -1) {
    switch (opt) {
      case 'a': str_schedType = string(optarg); break;
      case 's': str_subalg = string(optarg); break;
      case 'v': verbose = true; break;
      case 'G': show_gams = true; break;
      case 'm': mipstart = true; break;
      case 'f': fake_incr = atoi(optarg); break;
      case 'S': sll = true; break;
      case 'b': print_bits = true; break;

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
  ssmodel.maxEdgeDelay();
  // sspdg object based on the dfg
  SSDfg ssdfg(pdg_filename);


  if (str_schedType == "gams") {
    auto* scheduler_gams = new GamsScheduler(&ssmodel);
    scheduler_gams->showGams(show_gams);
    scheduler_gams->setMipstart(mipstart);
    scheduler_gams->setSll(sll);
    scheduler = scheduler_gams;
  } else if (str_schedType == "sa") { /*simulated annealing*/
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

  if(fake_incr) {
    scheduler->set_start_time();

    for(auto& node : ssmodel.subModel()->node_list()) {
      assert(node->subnet_table().size() == node->out_links().size());
    }

    CodesignInstance* cur_ci = new CodesignInstance(&ssmodel);
    cur_ci->verify();
    std::vector<WorkloadSchedules*> _incr_sched;
    
    cur_ci->workload_array.resize(fake_incr);

    for(int w = 0; w < fake_incr; ++w) {
      for(int i = 0; i < fake_incr; ++i) {
        cur_ci->workload_array[w].sched_array.emplace_back(cur_ci->ss_model(), &ssdfg);
      }
    }


    int max_iters_no_improvement = 100;

    int improv_iter = 0;

    for(int i = 0; i < 2000000; ++i) {
      if( (i-improv_iter) > max_iters_no_improvement) {
        break;
      }
      cout << " ### Begin DSE Iteration " << i << " ### \n";
      CodesignInstance* cand_ci = new CodesignInstance(*cur_ci);
      cand_ci->verify();
      cand_ci->make_random_modification();
      cand_ci->verify();

      scheduler->incrementalSchedule(*cand_ci);

      //stringstream name_ss;
      //name_ss << "viz/hw" << i << ".dot";
      //ofstream ofsb = ofstream(name_ss.str().c_str());
      //cand_ci->ss_model()->subModel()->PrintGraphviz(ofsb);

      cout << "DSE OBJ: " << cand_ci->dse_obj() << " -- ";
      
      auto* sub = cand_ci->ss_model()->subModel();
      cout << "Area: " << sub->get_overall_area()
           << " Nodes: " << sub->node_list().size() 
           << " Switches: " << sub->switch_list().size() << "\n";


      if(cand_ci->dse_obj() > cur_ci->dse_obj()) {
         improv_iter = i;
         delete cur_ci;
         cur_ci = cand_ci;
         cout << "----------------- IMPROVED OBJ! --------------------\n";

         for(int w = 0; w < fake_incr; ++w) {
           for(int o = 0; o < fake_incr; ++o) {
             stringstream name_ss;
             name_ss << "viz/dse-sched-" << i << "-" << w << "-" << o << ".gv";
             cur_ci->workload_array[w].sched_array[o].printGraphviz(name_ss.str().c_str()); 
           }
         }

        // dump the new hw json
        stringstream hw_ss;
        hw_ss << "viz/dse-sched-" << i <<".json";
        cur_ci -> ss_model()->subModel()->DumpHwInJSON(hw_ss.str().c_str());

        //cout << "### just reschedule the current cur_cit to be sure ###\n";
        //scheduler->incrementalSchedule(*cur_ci);
        //cout << "CUR DSE OBJ: " << cur_ci->dse_obj() << "\n"; 
        //auto* sub = cur_ci->ss_model()->subModel();
        //cout << "Area: " << sub->get_overall_area()
        //     << " Nodes: " << sub->node_list().size() 
        //     << " Switches: " << sub->switch_list().size() << "\n";
      }
    }
    /*
    std::string name_hw = "viz/dse-sched-final.json";
    auto* sub = cur_ci->ss_model()->subModel();
    sub -> DumpHwInJSON(name_hw.c_str());
    */

    cout << "DSE Complete!\n";
    cout << "Improv Iters: " << improv_iter << "\n";

    scheduler->incrementalSchedule(*cur_ci);

    cout << "FINAL DSE OBJ: " << cur_ci->dse_obj() << " -- ";
        
    auto* sub = cur_ci->ss_model()->subModel();
    cout << "Area: " << sub->get_overall_area()
         << " Nodes: " << sub->node_list().size() 
         << " Switches: " << sub->switch_list().size() << "\n";

    for(int w = 0; w < fake_incr; ++w) {
      for(int i = 0; i < fake_incr; ++i) {
        stringstream name_ss;
        name_ss << "viz/dse-sched-final-" << w << "-" << i << ".gv";
        cur_ci->workload_array[w].sched_array[i].printGraphviz(name_ss.str().c_str()); 
      }
    }


    return 0;
  }

  scheduler->invoke(&ssmodel, &ssdfg, print_bits);
}
