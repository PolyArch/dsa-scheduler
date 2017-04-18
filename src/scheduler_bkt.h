#ifndef __SB__SCHEDULE_BKT_H__
#define __SB__SCHEDULE_BKT_H__

#include "scheduler.h"

class SchedulerBacktracking : public HeuristicScheduler {
	public:
	SchedulerBacktracking(SB_CONFIG::SbModel* sbModel) : HeuristicScheduler(sbModel) {}
	bool schedule(SbPDG*, Schedule*&);
	bool scheduleNode(Schedule* sched, SbPDG_Node* pdgnode);
	std::pair<int,int> scheduleHere(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*,
               int config, CandidateRouting&,std::pair<int,int> bestScore);
	 std::pair<int,int> route(Schedule* sched, SbPDG_Edge* pdgnode,
            SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, int config,
            CandidateRouting&,std::pair<int,int> scoreLeft);
};

#endif
