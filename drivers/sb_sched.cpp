#include "model.h"
#include <assert.h>
#include <iostream>
#include <fstream>

#include "scheduler.h"
#include "scheduler_greedy.h"
#include "scheduler_bkt.h"
#include "scheduler_mlg.h"
#include "scheduler_gams.h"
#include "scheduler_sg.h"
#include "scheduler_sa.h"
#include "sbpdg.h"
#include <cstdlib>
#include <string>

using namespace std;
using namespace SB_CONFIG;

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

  //enum SchedType {GAMS, GREEDY, MLG, BKT, SG, SA};
  //SchedType schedType = GAMS;

    
  if(argc != 4) {
    cerr <<  "Usage: sb_sched config.sbmodel compute.sbpdg [gams,greedy,bkt,sg,sa,mlg]\n";
    exit(1);
  }


  //Sbmodel object based on the hw config
  SbModel sbmodel(argv[1]);

  cout << "Softbrain CGRA Size:" << sbmodel.subModel()->sizex() << "x"
                        << sbmodel.subModel()->sizey() <<"\n";

  std::string pdg_filename=argv[2];
  int lastindex = pdg_filename.find_last_of("."); 
  string pdg_rawname = pdg_filename.substr(0, lastindex); 


  string dfg_base = basename(pdg_filename); // the name without preceeding dirs or file extension
  string pdg_dir = basedir(pdg_filename);   // preceeding directories only
  if (pdg_dir[pdg_dir.length() - 1] != '\\' || pdg_dir[pdg_dir.length() - 1] != '/') {
    pdg_dir += "/";
  }
  string viz_dir = pdg_dir + "viz/";
  string verif_dir = pdg_dir + "verif/";
  system(("mkdir -p " + viz_dir).c_str());
  system(("mkdir -p " + verif_dir).c_str());
  system("mkdir -p gams/"); // gams will remain at top level

  //sbpdg object based on the dfg
  SbPDG sbpdg(pdg_filename);

  //cout << "file: " << filename << "\n";

  // ofstream ofs("viz/"+basename(pdg_filename)+".dot", ios::out);
  ofstream ofs(viz_dir + dfg_base + ".dot", ios::out);
  assert(ofs.good());
  sbpdg.printGraphviz(ofs);
  ofs.close();

  Schedule* sched=NULL;

  string str_schedType = string(argv[3]);
  //Scheduler scheduler(&sbmodel);
  Scheduler* scheduler;
  if(str_schedType == "gams") {
    //schedType = GAMS;
    scheduler = new GamsScheduler(&sbmodel);
  } else if(str_schedType == "greedy") { /*original*/
    //schedType = GREEDY;
    scheduler = new SchedulerGreedy(&sbmodel);
  } else if(str_schedType == "mlg") { /*multiple-link greedy*/
    //schedType = MLG;
    scheduler = new SchedulerMultipleLinkGreedy(&sbmodel);
  } else if(str_schedType == "sg") { /*stochastic greedy*/
    //schedType = SG;
    scheduler = new SchedulerStochasticGreedy(&sbmodel);
  } else if(str_schedType == "sa") { /*simulated annealing*/
    //schedType = SA;
    scheduler = new SchedulerSimulatedAnnealing(&sbmodel);
  } else if(str_schedType == "bkt") {
    //schedType = BKT;
    scheduler = new SchedulerBacktracking(&sbmodel);
  } else {
    cerr <<  "Usage: sb_sched config.sbmodel compute.sbpdg [gams,greedy,bkt,sg,sa,mlg]\n";
    exit(1);
  }



  scheduler->check_res(&sbpdg,&sbmodel);

  bool succeed_sched = false;

  int attempts = 0;
  int upperbound;
  upperbound = (str_schedType == "sg" ? 100000 : 1);
  srand(2);
  scheduler->progress_initBestNums();
  while (!succeed_sched && attempts < upperbound) {
    scheduler->progress_initCurNums();
    succeed_sched = scheduler->schedule(&sbpdg,sched);
    attempts++;
  }

  if (str_schedType == "sg" || str_schedType == "greedy") {
    scheduler->progress_printBests();
  }

  sched->printConfigText((viz_dir + dfg_base).c_str()); // text form of config fed to gui

  if(!succeed_sched) {
    cout << "Scheduling Failed!\n";
    return 1;
  } else {
    sched->stat_printLinkCount(); 
    cout << "Scheduling Successful!\n";
  }

  int lat,latmis;

  //calculate latency
  sched->calcLatency(lat,latmis);

  std::string config_header = pdg_rawname + ".h";
  std::ofstream osh(config_header);     
  assert(osh.good()); 
  sched->printConfigBits(osh, dfg_base);
  std::string verif_header = verif_dir + dfg_base + ".configbits";
  std::ofstream vsh(verif_header);
  assert(vsh.good()); 
  sched->printConfigVerif(vsh);

//  int num_pdgnodes=
//  (sched->sbpdg()->inst_end()-  sched->sbpdg()->inst_begin())+
//  (sched->sbpdg()->input_end()- sched->sbpdg()->input_begin())+
//  (sched->sbpdg()->output_end()-sched->sbpdg()->output_begin());

  cout << "latency: " << lat << "\n";  
  cout << "latency mismatch: " << latmis << "\n";  
}

