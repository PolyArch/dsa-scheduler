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
    _showGams(true), _mipstart(false) { }

  virtual bool schedule(SbPDG* sbPDG, Schedule*& schedule);
  bool requestGams(const char *filename);

  void showGams(bool show) {
    _showGams=show; 
  }
  void setMipstart(bool mipstart) {_mipstart=mipstart;}
  void use_server() { _use_server = true; }

  void print_mipstart(std::ofstream& ofs,  Schedule* sched, SbPDG* sbPDG, 
                      bool fix);

  protected:

  bool _gams_files_setup;
  bool _use_server;
  std::string _gams_work_dir;
  bool _showGams, _mipstart;

  std::unordered_map<std::string, std::pair<SB_CONFIG::sbnode*,int> >  gamsToSbnode;  
  std::unordered_map<std::string, std::pair<SB_CONFIG::sblink*,int> > gamsToSblink;
  std::unordered_map<std::string, std::pair<SB_CONFIG::sbswitch*,int>>  gamsToSbswitch;


  std::unordered_map<std::string, SbPDG_Node*> gamsToPdgnode;
  std::unordered_map<std::string, SbPDG_Edge*> gamsToPdgedge;
  std::unordered_map<std::string, std::pair<bool,int> >  gamsToPortN;  
  std::unordered_map<std::string, SbPDG_Vec*>  gamsToPortV;  

};

#endif
