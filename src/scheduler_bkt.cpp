#include "scheduler_bkt.h"

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


bool SchedulerBacktracking::schedule(SbPDG* sbPDG, Schedule*& sched) {

  return 0;
}

bool SchedulerBacktracking::scheduleNode(Schedule* sched, SbPDG_Node* pdgnode) {
	return false;
}

std::pair<int,int> SchedulerBacktracking::scheduleHere(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*,
               CandidateRouting&,std::pair<int,int> bestScore){
	return make_pair(0,0);
}

pair<int,int> SchedulerBacktracking::route(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest, CandidateRouting& candRouting, pair<int,int> scoreLeft) {

	return make_pair(0,0);
}




