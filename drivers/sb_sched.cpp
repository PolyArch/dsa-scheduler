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
  
  if(argc<3) {
    cerr <<  "Usage: sb_sched config.sbmodel compute.sbpdg\n";
    exit(1);
  }

  //Sbmodel object based on the hw config
  SbModel sbmodel(argv[1]);

  cout << "Softbrain CGRA Size:" << sbmodel.subModel()->sizex() << "x"
                        << sbmodel.subModel()->sizey() <<"\n";

  std::string pdg_filename=argv[2];
  int lastindex = pdg_filename.find_last_of("."); 
  string pdg_rawname = pdg_filename.substr(0, lastindex); 

  system("mkdir -p gams/");
  system("mkdir -p dots/");

  //sbpdg object based on the dfg
  SbPDG sbpdg(pdg_filename);

  //cout << "file: " << filename << "\n";

  ofstream ofs("dots/"+basename(pdg_filename)+".dot", ios::out);
  assert(ofs.good());
  sbpdg.printGraphviz(ofs);
  ofs.close();

  Schedule* sched=NULL;
  Scheduler scheduler(&sbmodel);
  scheduler.scheduleGAMS(&sbpdg,sched);

  int lat,latmis;

  //calculate latency
  sched->calcLatency(lat,latmis);
  sched->printAllConfigs(pdg_rawname.c_str());      //text form of config fed to gui

  std::string config_header=pdg_rawname+string(".h");
  std::ofstream osh(config_header);     
  assert(osh.good()); 
  sched->printConfigBits(osh,basename(pdg_filename));

//  int num_pdgnodes=
//  (sched->sbpdg()->inst_end()-  sched->sbpdg()->inst_begin())+
//  (sched->sbpdg()->input_end()- sched->sbpdg()->input_begin())+
//  (sched->sbpdg()->output_end()-sched->sbpdg()->output_begin());

  cout << "latency: " << lat << "\n";  
  cout << "latency mismatch: " << latmis << "\n";  
}

