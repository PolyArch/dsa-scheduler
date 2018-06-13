#include "scheduler_sa.h"

using namespace SB_CONFIG;
using namespace std;

#include <unordered_map>
#include <fstream>
#include <list>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>

bool SchedulerSimulatedAnnealing::schedule(SbPDG* sbPDG, Schedule*& sched) {  
  return false;
}



bool SchedulerSimulatedAnnealing::scheduleNode(Schedule* sched, SbPDG_Node* pdgnode) {
	return false;
}

std::pair<int,int> SchedulerSimulatedAnnealing::scheduleHere(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*,
               CandidateRouting&,std::pair<int,int> bestScore){
	return make_pair(0,0);
}

pair<int,int> SchedulerSimulatedAnnealing::route(Schedule* sched, 
    SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest, 
    CandidateRouting& candRouting, pair<int,int> scoreLeft) {

  pair<int,int> score = route_minimizeDistance(sched, pdgedge, source, dest, candRouting, scoreLeft);
    return score;
}



