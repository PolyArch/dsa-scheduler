#include "model.h"
#include <assert.h>
#include <iostream>
#include <fstream>

#include "scheduler.h"
#include "sbpdg.h"
#include <cstdlib>
#include <string>
#include "model_parsing.h"


using namespace std;
using namespace SB_CONFIG;

int main(int argc, char* argv[])
{
  
  if(argc<2) {
    cerr <<  "Usage: resched config_file\n";
    exit(1);
  }
  
  Schedule sched(argv[1]);
  
  int lat,latmis;

  sched.calcLatency(lat,latmis);

  int num_pdgnodes=
  (sched.sbpdg()->inst_end()-sched.sbpdg()->inst_begin())+
  (sched.sbpdg()->input_end()-sched.sbpdg()->input_begin())+
  (sched.sbpdg()->output_end()-sched.sbpdg()->output_begin());

  cout << "latency: " << lat << "\n";  
  cout << "latency mismatch: " << latmis << "\n";  
  cout << "num nodes: " << num_pdgnodes << "\n";  

  std::string g(argv[1]);
  ModelParsing::trim_comments(g);

  
}

