#include "scheduler_sg.h"

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

bool SchedulerStochasticGreedy::schedule(SbPDG*, Schedule*&) {
	return false;
}

bool SchedulerStochasticGreedy::scheduleNode(Schedule* sched, SbPDG_Node* pdgnode) {
	return false;
}

std::pair<int,int> SchedulerStochasticGreedy::scheduleHere(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*,
               int config, CandidateRouting&,std::pair<int,int> bestScore){
	return make_pair(0,0);
}

std::pair<int,int> SchedulerStochasticGreedy::route(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, int config,
            CandidateRouting&,std::pair<int,int> scoreLeft) {
	return make_pair(0,0);
}
