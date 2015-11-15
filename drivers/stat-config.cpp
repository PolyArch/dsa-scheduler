#include "model.h"
#include <assert.h>
#include <iostream>
#include <fstream>

#include "scheduler.h"

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
  (sched.dypdg()->inst_end()-sched.dypdg()->inst_begin())+
  (sched.dypdg()->input_end()-sched.dypdg()->input_begin())+
  (sched.dypdg()->output_end()-sched.dypdg()->output_begin());

  cout << "latency: " << lat << "\n";  
  cout << "latency mismatch: " << latmis << "\n";  
  cout << "num nodes: " << num_pdgnodes << "\n";  
  
}

