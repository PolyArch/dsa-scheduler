#ifndef __SB__SCHEDULE_SIMULATEDANNEALING_H__
#define __SB__SCHEDULE_SIMULATEDANNEALING_H__

#include "scheduler.h"

class SchedulerSimulatedAnnealing : public HeuristicScheduler {
	public:
        void initialize(SbPDG*, Schedule*&);

	SchedulerSimulatedAnnealing(SB_CONFIG::SbModel* sbModel) : 
          HeuristicScheduler(sbModel) {}
	bool schedule(SbPDG*, Schedule*&);
	bool scheduleNode(Schedule* sched, SbPDG_Node* pdgnode);
	std::pair<int,int> scheduleHere(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*,
               CandidateRouting&,std::pair<int,int> bestScore);
        std::pair<int,int> route(Schedule* sched, SbPDG_Edge* pdgnode,
          SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, 
          CandidateRouting&,std::pair<int,int> scoreLeft);

        protected:
  std::vector<std::pair<int,int>> _sd_in;  //port, length pair 
  std::vector<std::pair<int,int>> _sd_out; //port, length pair

  int _max_iters_zero_vio=1000000000;
  bool _integrate_timing = true;
  int _best_latmis, _best_lat, _best_violation;
  bool _strict_timing = true;

};

#endif
