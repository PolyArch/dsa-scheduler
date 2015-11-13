#include "model.h"
#include <assert.h>
#include <iostream>
#include <fstream>

#include "scheduler.h"
#include "dypdg.h"
#include <cstdlib>

using namespace std;
using namespace DY_MODEL;

int main(int argc, char* argv[])
{
  
  if(argc<3) {
    cerr <<  "Usage: sb_sched config.dymodel compute.sbdp\n";
    exit(1);
  }
  DyModel dymodel(argv[1]);

  cout << "DySER Size:" << dymodel.subModel()->sizex() << "x"
                        << dymodel.subModel()->sizex() <<"\n";

  std::string pdg_filename=argv[2];
  int lastindex = pdg_filename.find_last_of("."); 
  string pdg_rawname = pdg_filename.substr(0, lastindex); 

  system("mkdir -p gams/");
  system("mkdir -p dots/");

  DyPDG dypdg(pdg_filename);

  ofstream ofs("dots/pdgout.dot", ios::out);
  assert(ofs.good());
  dypdg.printGraphviz(ofs);
  ofs.close();

  Schedule* sched;
  Scheduler scheduler(&dymodel);
  scheduler.scheduleGAMS(&dypdg,sched);

  int lat,latmis;
  sched->calcLatency(lat,latmis);
  
  sched->printAllConfigs(pdg_rawname.c_str());

//  int num_pdgnodes=
//  (sched->dypdg()->inst_end()-  sched->dypdg()->inst_begin())+
//  (sched->dypdg()->input_end()- sched->dypdg()->input_begin())+
//  (sched->dypdg()->output_end()-sched->dypdg()->output_begin());

  cout << "latency: " << lat << "\n";  
  cout << "latency mismatch: " << latmis << "\n";  

}

