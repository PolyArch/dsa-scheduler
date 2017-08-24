#ifndef __SB__SCHEDULE_GAMS_H__
#define __SB__SCHEDULE_GAMS_H__

#include <iostream>
#include <fstream>

#include "scheduler.h"

class GamsScheduler : public Scheduler {
public:
  GamsScheduler(SB_CONFIG::SbModel* sbModel) :
    Scheduler(sbModel),
    _gams_files_setup(false), _use_server(false),
    _gams_work_dir("gams"),
    _optcr(0.1f), _optca(0.0f), _reslim(100000.0f), _showGams(true) { }

  virtual bool schedule(SbPDG* sbPDG, Schedule*& schedule);
  bool requestGams(const char *filename);

  void showGams(bool show) {
    _showGams=show; 
  }
  void use_server() { _use_server = true; }
	void setGap(float relative, float absolute=1.0f) {
    _optcr=relative;
    _optca=absolute;
  }

  void setTimeout(float timeout) {
    _reslim=timeout;
  }

  void print_mipstart(std::ofstream& ofs,  Schedule* sched, SbPDG* sbPDG, 
                      bool fix);


  protected:

  bool _gams_files_setup;
  bool _use_server;
  std::string _gams_work_dir;
  float _optcr,_optca,_reslim;
  bool _showGams;

  std::unordered_map<std::string, std::pair<SB_CONFIG::sbnode*,int> >  gamsToSbnode;  
  std::unordered_map<std::string, std::pair<SB_CONFIG::sblink*,int> > gamsToSblink;
  std::unordered_map<std::string, std::pair<SB_CONFIG::sbswitch*,int>>  gamsToSbswitch;


  std::unordered_map<std::string, SbPDG_Node*> gamsToPdgnode;
  std::unordered_map<std::string, SbPDG_Edge*> gamsToPdgedge;
  std::unordered_map<std::string, std::pair<bool,int> >  gamsToPortN;  
  std::unordered_map<std::string, SbPDG_Vec*>  gamsToPortV;  

};

#endif
