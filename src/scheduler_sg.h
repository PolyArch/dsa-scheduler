#ifndef __SB__SCHEDULE_STOCHASTICGREEDY_H__
#define __SB__SCHEDULE_STOCHASTICGREEDY_H__

#include "scheduler.h"

class SchedulerStochasticGreedy : public HeuristicScheduler {
	public:
	SchedulerStochasticGreedy(SB_CONFIG::SbModel* sbModel) : HeuristicScheduler(sbModel) {}
	bool schedule(SbPDG*, Schedule*&);
	bool scheduleNode(Schedule* sched, SbPDG_Node* pdgnode);
	std::pair<int,int> scheduleHere(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*,
               CandidateRouting&,std::pair<int,int> bestScore);
	 std::pair<int,int> route(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, 
            CandidateRouting&,std::pair<int,int> scoreLeft);

	void findFirstIndex(std::vector<std::pair<int,int>>& sd, sbio_interface& si, unsigned int numIO, unsigned int& index, StatType s);
	void genRandomIndexBW(std::pair<bool, int>& vport_id, std::vector<std::pair<int, std::vector<int>>>& vport_desc, std::vector<std::pair<int,int>>& sd, sbio_interface& si, unsigned int size, unsigned int index, Schedule*& sched, StatType s);
	bool assignVectorOutputs(SbPDG* sbPDG, Schedule* sched);
	bool assignVectorInputs(SbPDG* sbPDG, Schedule* sched);
};

#endif
