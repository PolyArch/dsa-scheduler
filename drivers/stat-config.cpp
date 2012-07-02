#include "model.h"
#include <assert.h>
#include <iostream>
#include <fstream>

#include "scheduler.h"

using namespace std;
using namespace DY_MODEL;

int main(int argc, char* argv[])
{
  
  if(argc<2) {
    cerr <<  "Usage: resched config_file\n";
    exit(1);
  }
  
  Schedule sched(argv[1]);
  
  cout << "latency: " << sched.calcLatency() << "\n";  
}

