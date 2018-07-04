#ifndef __SB__SCHEDULE_SIMULATEDANNEALING_H__
#define __SB__SCHEDULE_SIMULATEDANNEALING_H__

#include "scheduler.h"

class SchedulerSimulatedAnnealing : public HeuristicScheduler {
public:
  void initialize(SbPDG*, Schedule*&);

  SchedulerSimulatedAnnealing(SB_CONFIG::SbModel* sbModel) : 
    HeuristicScheduler(sbModel) {}
  bool schedule(SbPDG*, Schedule*&);

  std::pair<int,int> route(Schedule* sched, SbPDG_Edge* pdgnode,
    SB_CONFIG::sbnode* source, SB_CONFIG::sbnode* dest, 
    CandidateRouting&,std::pair<int,int> scoreLeft);

  virtual int routing_cost(SbPDG_Edge*, sblink*, Schedule*, 
      CandidateRouting&, sbnode* dest);


protected:
  bool schedule_internal(SbPDG* sbPDG, Schedule*& sched);
  std::pair<int, int> obj(Schedule*& sched, int& lat, 
      int& lat_mis, int& ovr); 


  bool schedule_input( SbPDG_VecInput*  vec, SbPDG* sbPDG, Schedule* sched);
  bool schedule_output(SbPDG_VecOutput* vec, SbPDG* sbPDG, Schedule* sched);
  bool scheduleNode(Schedule* sched, SbPDG_Node* pdgnode);
  std::pair<int,int> scheduleHere(Schedule*, SbPDG_Node*, SB_CONFIG::sbnode*,
         CandidateRouting&,std::pair<int,int> bestScore);

  void findFirstIndex(std::vector<std::pair<int,int>>& sd, sbio_interface& si, 
    unsigned int numIO, unsigned int& index, bool is_input);

 bool genRandomIndexBW(std::pair<bool, int>& vport_id, std::vector<std::pair<int, std::vector<int>>>& vport_desc, std::vector<std::pair<int,int>>& sd, sbio_interface& si, unsigned int size, unsigned int index, Schedule*& sched, bool s);

  bool timingIsStillGood(Schedule* sched); 

  bool map_to_completion(SbPDG* sbPDG, Schedule* sched);

  bool map_one_input(   SbPDG* sbPDG, Schedule* sched);
  bool map_one_inst(    SbPDG* sbPDG, Schedule* sched);
  bool map_one_output(  SbPDG* sbPDG, Schedule* sched);
  void unmap_one_input( SbPDG* sbPDG, Schedule* sched);
  void unmap_one_inst(  SbPDG* sbPDG, Schedule* sched);
  void unmap_one_output(SbPDG* sbPDG, Schedule* sched);

  void unmap_some(SbPDG* sbPDG, Schedule* sched);

  std::vector<std::pair<int,int>> _sd_in;  //port, length pair 
  std::vector<std::pair<int,int>> _sd_out; //port, length pair

  int _max_iters_zero_vio=1000000000;
  bool _integrate_timing = true;
  int _best_latmis, _best_lat, _best_violation;
  bool _strict_timing = true;

};

#endif
