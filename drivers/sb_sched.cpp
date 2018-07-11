#include <getopt.h>

#include "model.h"
#include <assert.h>
#include <iostream>
#include <fstream>

#include "scheduler.h"
#include "scheduler_gams.h"
#include "scheduler_sg.h"
#include "scheduler_sa.h"
#include "sbpdg.h"
#include <cstdlib>
#include <string>

# include <iostream>
# include <chrono>
using namespace std;
using sec = chrono::seconds;
using get_time = chrono::steady_clock ;

using namespace SB_CONFIG;

static struct option long_options[] = {
  { "algorithm", required_argument, NULL, 'a', },
  { "sub-alg", required_argument, NULL, 's', },
  { "verbose", no_argument, NULL, 'v', },

  { "show-gams", no_argument, NULL, 'G', },
  { "mipstart", no_argument, NULL, 'm', },
  { "sll", no_argument, NULL, 'S', },
  { "no-int-time", no_argument, NULL, 'n', },

  { "relative-gap", required_argument, NULL, 'r', },
  { "absolute-gap", required_argument, NULL, 'g', },
  { "timeout",      required_argument, NULL, 't', },
  { "max-iters",    required_argument, NULL, 'i', },

  { "max-edge-delay", required_argument, NULL, 'd', },

  { 0, 0, 0, 0, },
};

std::string basename(std::string& filename) {
  size_t lastindex = filename.find_last_of("."); 
  string basename = filename.substr(0, lastindex); 
 
  lastindex = filename.find_last_of("\\/"); 
  if(lastindex != string::npos) {
    basename = basename.substr(lastindex+1);
  }
  return basename;
}

std::string basedir(std::string& filename) {
  size_t lastindex = filename.find_last_of("\\/"); 
  if(lastindex == string::npos) {
    return std::string("./");
  }
  return filename.substr(0, lastindex);  
}


int main(int argc, char* argv[])
{    
  int opt;
  bool verbose = false;
  bool show_gams = false, mipstart=false, sll=false;
  string str_schedType = string("sg"); 
  string str_subalg = string("");

  float absolute_gap=1.0f;
  float relative_gap=0.1f;
  float timeout=86400.0f;
  int max_edge_delay=15;
  int max_iters=20000;
  
  bool no_int_time=false;

  while ((opt = getopt_long(argc, argv, "vGa:s:r:g:t:md:", long_options, NULL)) != -1) {
    switch (opt) {
    case 'a': str_schedType = string(optarg); break;
    case 's': str_subalg = string(optarg); break;
    case 'v': verbose = true; break;
    case 'G': show_gams = true; break;
    case 'm': mipstart=true; break;
    case 'S': sll=true; break;
    case 'n': no_int_time=true; break;

    case 'r': relative_gap=atof(optarg); break;
    case 'g': absolute_gap=atof(optarg); break;
    case 't': timeout=atof(optarg); break;
    case 'i': max_iters=atoi(optarg); break;

    case 'd': max_edge_delay=atoi(optarg); break;

    default: exit(1);
    }
  }

  argc -= optind;
  argv += optind;

  if(argc != 2) {
    cerr <<  "Usage: sb_sched [FLAGS] config.sbmodel compute.sbpdg \n";
    exit(1);
  }

  std::string model_filename = argv[0];
  int lastindex = model_filename.find_last_of(".");
  string model_rawname = model_filename.substr(0,lastindex);
  string model_base =
    model_rawname.substr(model_rawname.find_last_of("\\/") + 1, model_rawname.size());

  SbModel sbmodel(model_filename.c_str());
  sbmodel.setMaxEdgeDelay(max_edge_delay);
  sbmodel.maxEdgeDelay();

  //cout << "Softbrain CGRA Size:" << sbmodel.subModel()->sizex() << "x"
  //                               << sbmodel.subModel()->sizey() <<"\n";

  std::string pdg_filename=argv[1];
  lastindex = pdg_filename.find_last_of("."); 
  string pdg_rawname = pdg_filename.substr(0, lastindex); 

  

  string dfg_base = basename(pdg_filename); // the name without preceeding dirs or file extension
  string pdg_dir = basedir(pdg_filename);   // preceeding directories only
  if (pdg_dir[pdg_dir.length() - 1] != '\\' || pdg_dir[pdg_dir.length() - 1] != '/') {
    pdg_dir += "/";
  }
  string viz_dir = pdg_dir + "viz/";
  string verif_dir = pdg_dir + "verif/";
  string sched_dir = pdg_dir + "sched/"; // Directory for cheating on the scheduler
  System(("mkdir -p " + viz_dir).c_str());
  System(("mkdir -p " + verif_dir).c_str());
  System(("mkdir -p " + sched_dir).c_str());
  System("mkdir -p gams/"); // gams will remain at top level

  //sbpdg object based on the dfg
  SbPDG sbpdg(pdg_filename);

  /* scheduler for testing purposes
  for (int i=0; i<100; i++){
    cycle();
  }
  */


  //cout << "file: " << filename << "\n";

  // ofstream ofs("viz/"+basename(pdg_filename)+".dot", ios::out);
  ofstream ofs(viz_dir + dfg_base + ".dot", ios::out);
  assert(ofs.good());
  sbpdg.printGraphviz(ofs);
  ofs.close();

  Schedule* sched=NULL;

  //Scheduler scheduler(&sbmodel);
  Scheduler* scheduler;
  if(str_schedType == "gams") {
    auto* scheduler_gams = new GamsScheduler(&sbmodel);
    scheduler_gams->showGams(show_gams);
    scheduler_gams->setMipstart(mipstart);
    scheduler_gams->setSll(sll);
    scheduler = scheduler_gams;
  } else if(str_schedType == "sg") { /*stochastic greedy*/
    auto* scheduler_sg = new SchedulerStochasticGreedy(&sbmodel);
    scheduler_sg->set_integrate_timing(!no_int_time);
    scheduler = scheduler_sg;
  } else if(str_schedType == "sa") { /*simulated annealing*/
    scheduler = new SchedulerSimulatedAnnealing(&sbmodel);
  } else {
    cerr <<  "Something Went Wrong with Default Scheduler String";
    exit(1);
  }

  scheduler->verbose = verbose;
  scheduler->str_subalg = str_subalg;
  scheduler->setGap(relative_gap,absolute_gap);
  scheduler->setTimeout(timeout);
  scheduler->set_max_iters(max_iters);

  scheduler->check_res(&sbpdg,&sbmodel);

  bool succeed_sched = false;

  succeed_sched = scheduler->schedule_timed(&sbpdg,sched);

  //if(succeed_sched) {
  //  sched->printConfigText((viz_dir + dfg_base).c_str()); // text form of config fed to gui
  //}

  if(verbose) {
    int lat=0,latmis=0;
    sched->cheapCalcLatency(lat,latmis);
    //sched->checkOutputMatch(latmis);
    if(succeed_sched) {
      cout << "latency: " << lat << "\n";  
      cout << "latency mismatch: " << latmis << "\n";
      cout << "Scheduling Successful!\n";
    } else { 
      cout << "latency: " << 0 << "\n";  
      cout << "latency mismatch: " << 0 << "\n";  
      cout << "Scheduling Failed!\n";
    }
    sched->stat_printOutputLatency();
    sbpdg.printGraphviz("viz/final.dot",sched);
  } 

  std::string sched_viz = viz_dir + dfg_base + "." + model_base + ".gv";
  sched->printGraphviz(sched_viz.c_str());

  std::string config_header = pdg_rawname + ".dfg.h";
  std::ofstream osh(config_header);     
  assert(osh.good()); 
  sched->printConfigHeader(osh, dfg_base);
 
  std::string verif_header = verif_dir + dfg_base + ".configbits";
  std::ofstream vsh(verif_header);
  assert(vsh.good()); 
  sched->printConfigVerif(vsh);

  delete sched; // just to calm HEAPCHECK

//  int num_pdgnodes=
//  (sched->sbpdg()->inst_end()-  sched->sbpdg()->inst_begin())+
//  (sched->sbpdg()->input_end()- sched->sbpdg()->input_begin())+
//  (sched->sbpdg()->output_end()-sched->sbpdg()->output_begin());

}

