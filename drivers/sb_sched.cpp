#include "model.h"
#include <assert.h>
#include <iostream>
#include <fstream>

#include "scheduler.h"
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

  enum SchedType {GAMS,GREEDY,BKT};
  SchedType schedType = GAMS;
  bool bad_parsing = false;

  if(argc==4) {
    string str_schedType = string(argv[3]);
    if(str_schedType == "gams") {
      schedType = GAMS;
    } else if(str_schedType == "greedy") {
      schedType = GREEDY;
    } else if(str_schedType == "bkt") {
      schedType = BKT;
    } else {
      bad_parsing = true;
    }
  }

  if(argc<3 || argc >4 || bad_parsing) {
    cerr <<  "Usage: sb_sched config.sbmodel compute.sbpdg [gams,greedy,bkt]\n";
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
  Scheduler scheduler(&sbmodel);

  scheduler.check_res(&sbpdg,&sbmodel);

  bool succeed_sched = false;

  if(schedType == GAMS) {
    succeed_sched = scheduler.scheduleGAMS(&sbpdg,sched);
  } else if(schedType == BKT) {
    succeed_sched = scheduler.scheduleBacktracking(&sbpdg,sched); 
  } else if(schedType == GREEDY) {
    succeed_sched = scheduler.scheduleGreedyBFS(&sbpdg,sched); 
  } else {
    cout << "NO SCHEDULER FOUND\n";
    return 1;
  }

  if(!succeed_sched) {
    cout << "ERROR: SCHEDULING FAILED -- EXITING\n";
    return 1;
  } else {
  	sched->stat_printLinkCount(); 
    cout << "Scheduling Successful!\n";
  }

  int lat,latmis;

  //calculate latency
  sched->calcLatency(lat,latmis);
  sched->printAllConfigs((viz_dir + dfg_base).c_str()); // text form of config fed to gui

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

