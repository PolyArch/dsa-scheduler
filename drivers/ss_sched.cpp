#include <getopt.h>

#include <assert.h>
#include <fstream>
#include <iostream>
#include "model.h"

#include <cstdlib>
#include <string>
#include "scheduler.h"
#include "scheduler_gams.h"
#include "scheduler_sa.h"
#include "ssdfg.h"

#include <chrono>
#include <iostream>

#include <signal.h>

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

void sigint_handler(int s) {
  if (scheduler) {
    if (scheduler->running()) {
      scheduler->stop();
      return;
    }
  }
  exit(1);
}

std::string basename(std::string& filename) {
  size_t lastindex = filename.find_last_of(".");
  string basename = filename.substr(0, lastindex);

  lastindex = filename.find_last_of("\\/");
  if (lastindex != string::npos) {
    basename = basename.substr(lastindex + 1);
  }
  return basename;
}

std::string basedir(std::string& filename) {
  size_t lastindex = filename.find_last_of("\\/");
  if (lastindex == string::npos) {
    return std::string("./");
  }
  return filename.substr(0, lastindex);
}

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
  int lastindex = model_filename.find_last_of(".");
  string model_rawname = model_filename.substr(0, lastindex);
  string model_base =
      model_rawname.substr(model_rawname.find_last_of("\\/") + 1, model_rawname.size());

  SSModel ssmodel(model_filename.c_str());
  cout << "Using max edge delay: " << max_edge_delay << "\n";
  ssmodel.setMaxEdgeDelay(max_edge_delay);
  ssmodel.maxEdgeDelay();

  std::string pdg_filename = argv[1];
  lastindex = pdg_filename.find_last_of(".");
  string pdg_rawname = pdg_filename.substr(0, lastindex);

  string dfg_base =
      basename(pdg_filename);  // the name without preceeding dirs or file extension
  string pdg_dir = basedir(pdg_filename);  // preceeding directories only
  if (pdg_dir[pdg_dir.length() - 1] != '\\' || pdg_dir[pdg_dir.length() - 1] != '/') {
    pdg_dir += "/";
  }
  string viz_dir = pdg_dir + "viz/";
  string iter_dir = pdg_dir + "viz/iter/";
  string verif_dir = pdg_dir + "verif/";
  string sched_dir = pdg_dir + "sched/";  // Directory for cheating on the scheduler

  checked_system(("mkdir -p " + iter_dir).c_str());
  checked_system(("mkdir -p " + verif_dir).c_str());
  checked_system(("mkdir -p " + sched_dir).c_str());
  checked_system("mkdir -p gams/");  // gams will remain at top level

  // ssdfg object based on the dfg
  SSDfg ssdfg(pdg_filename);

  ofstream ofs(viz_dir + dfg_base + ".dot", ios::out);
  assert(ofs.good());
  ssdfg.printGraphviz(ofs);
  ofs.close();

  Schedule* sched = nullptr;

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

  bool succeed_sched = false;

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
        cur_ci->workload_array[w].sched_array.emplace_back(cur_ci->ss_model(),&ssdfg);
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

        //cout << "### just reschedule the current cur_cit to be sure ###\n";
        //scheduler->incrementalSchedule(*cur_ci);
        //cout << "CUR DSE OBJ: " << cur_ci->dse_obj() << "\n"; 
        //auto* sub = cur_ci->ss_model()->subModel();
        //cout << "Area: " << sub->get_overall_area()
        //     << " Nodes: " << sub->node_list().size() 
        //     << " Switches: " << sub->switch_list().size() << "\n";

      }
    }

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

  if (scheduler->check_feasible(&ssdfg, &ssmodel,true/*verbose*/)) {
    // At least it's possible to schedule

    // Setup signal so we can stop if we need to
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = sigint_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // Debug
    // string hw_config_filename = model_rawname + ".xml";
    // sched->printConfigBits_Hw(hw_config_filename);

    succeed_sched = scheduler->schedule_timed(&ssdfg, sched);

    int lat = 0, latmis = 0;
    if (succeed_sched) {
      sched->cheapCalcLatency(lat, latmis);
      sched->set_decode_lat_mis(latmis);

      // ofstream ctxs(viz_dir + dfg_base + ".config", ios::out);
      // sched->printConfigText(ctxs); // text form of config fed to gui
    }

    if (verbose) {
      sched->cheapCalcLatency(lat, latmis);
      int ovr = 0, agg_ovr = 0, max_util = 0;
      sched->get_overprov(ovr, agg_ovr, max_util);
      int violation = sched->violation();

      // sched->checkOutputMatch(latmis);
      if (succeed_sched) {
        // Also check final latency

        /*
        if(agg_ovr==0) {
          int sim_lat=-1, sim_latmis=-1;
          sched->calcLatency(sim_lat,sim_latmis,true);
          cout << "simulated-lat:  " << sim_lat << "\n";
          cout << "simulated-lat-mis:  " << sim_latmis << "\n";
          if(lat!=sim_lat) lat=-1;
          if(latmis!=sim_latmis) latmis=-1;
        }
        */

        cout << "latency: " << lat << "\n";
        cout << "lat-mismatch-max: " << latmis << "\n";
        cout << "lat-mismatch-sum: " << violation << "\n";
        cout << "overprov-max: " << ovr << "\n";
        cout << "overprov-sum: " << agg_ovr << "\n";
      } else {
        cout << "latency: " << -1 << "\n";
        cout << "latency mismatch: " << -1 << "\n";
        cout << "Scheduling Failed!\n";
      }
      sched->stat_printOutputLatency();
      ssdfg.printGraphviz("viz/final.dot", sched);
    }

    std::string sched_viz = viz_dir + dfg_base + "." + model_base + ".gv";
    sched->printGraphviz(sched_viz.c_str());

    std::string verif_header = verif_dir + dfg_base + ".configbits";
    std::ofstream vsh(verif_header);
    assert(vsh.good());
    sched->printConfigVerif(vsh);
  }

  if (!succeed_sched || sched == nullptr) {
    cout << "We're going to print the DFG for simulation purposes...  have fun!\n\n";
    // This is just a fake schedule!
    // sched = new Schedule(&ssmodel,&ssdfg);
    SchedulerSimulatedAnnealing* s = new SchedulerSimulatedAnnealing(&ssmodel);
    s->set_fake_it();

    s->initialize(&ssdfg, sched);
    succeed_sched = s->schedule_internal(&ssdfg, sched);
  }

  // TODO: Print Hardware Config Information @ Sihao
  // string hw_config_filename = model_rawname + ".xml";
  // sched->printConfigBits_Hw(hw_config_filename);

  sched->set_name(pdg_rawname);
  std::string config_header = pdg_rawname + ".dfg.h";
  std::ofstream osh(config_header);
  assert(osh.good());
  sched->printConfigHeader(osh, dfg_base);

  if (print_bits) {
    std::string config_header_bits = pdg_rawname + ".dfg.bits.h";
    std::ofstream oshb(config_header_bits);
    assert(oshb.good());
    sched->printConfigHeader(oshb, dfg_base, true);
  }

  delete sched;  // just to calm HEAPCHECK
}
