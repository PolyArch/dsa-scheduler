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

bool SchedulerSimulatedAnnealing::schedule(SbPDG*, Schedule*&) {
	return false;
}

bool SchedulerSimulatedAnnealing::scheduleNode(Schedule* sched, SbPDG_Node* pdgnode) {
	return false;
}

std::pair<int,int> SchedulerSimulatedAnnealing::scheduleHere(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*,
               CandidateRouting&,std::pair<int,int> bestScore){
	return make_pair(0,0);
}

std::pair<int,int> SchedulerSimulatedAnnealing::route(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, 
            CandidateRouting&,std::pair<int,int> scoreLeft) {
	return make_pair(0,0);
}
