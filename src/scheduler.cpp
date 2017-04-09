#include "scheduler.h"

using namespace SB_CONFIG;
using namespace std;

#include <unordered_map>
#include <fstream>
#include <sstream>

#include "model_parsing.h"

//#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <list>
/*
class proposedPaths {
  public:
  //vector<pair<PDG_ID,
  //vector<pair<int,vector<int> > path;
  std::map<int,int>;
}*/

#define MAX_ROUTE 100000000


Schedule* Scheduler::scheduleGreedyBFS(SbPDG* sbPDG) {
  Schedule* sched = new Schedule(_sbModel,sbPDG);
  sched->setNConfigs(1);
  
  map<SbPDG_Inst*,bool> seen;
  
  list<SbPDG_Inst* > openset;
  SbPDG::const_input_iterator I,E;
 
  //pdg input nodes
  for(I=sbPDG->input_begin(),E=sbPDG->input_end();I!=E;++I) {
    SbPDG_Input* n = *I;
    
    SbPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(use_pdginst) {
        openset.push_back(use_pdginst);
      }
    }
  }
 
  //populate the schedule object
  while(!openset.empty()) {
    SbPDG_Inst* n = openset.front(); 
    openset.pop_front();
    
    if(!seen[n]) {
      scheduleNode(sched,n);
    }
    seen[n]=true;
    
    SbPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(use_pdginst) {
        openset.push_back(use_pdginst);
      }
    }

  }
  
  
  /*
  SbPDG::const_inst_iterator Ii,Ei;
  for(Ii=sbPDG->inst_begin(), Ei=sbPDG->inst_end(); Ii!=Ei; ++Ii) {
    SbPDG_Inst* pdginst = *Ii; 
    scheduleNode(sched,pdginst);
  }*/
  
  
  
  
  //Scheduling the inputs 
  for(I=sbPDG->input_begin(),E=sbPDG->input_end();I!=E;++I) {
    SbPDG_Input* pdgin = *I;
    scheduleNode(sched,pdgin);
  }
   
  //schedule the outputs
  SbPDG::const_output_iterator Io,Eo;
  for(Io=sbPDG->output_begin(),Eo=sbPDG->output_end();Io!=Eo;++Io) {
    SbPDG_Output* pdgout = *Io;
    scheduleNode(sched,pdgout);
  }
  return sched;
}

void Scheduler::applyRouting(Schedule* sched, SbPDG_Node* pdgnode,
                             sbnode* here, int config, CandidateRouting* candRouting){
  
  //cout << "pdgnode: " << pdgnode->name()  << " sbnode: " << here->name() 
  //<< " nlinks: " << candRouting->routing.size() << "\n";
  
  sched->assign_node(pdgnode,here,config);
  std::map< std::pair<SB_CONFIG::sblink*,int>,SbPDG_Edge* >::iterator I,E;
  for(I= candRouting->routing.begin(), E=candRouting->routing.end();I!=E;++I) {
    sched->assign_link(I->second->def(),I->first.first, I->first.second);
    //sched->assign_edgelink(I->second,I->first.first, I->first.second);
    /*if(candRouting->routing.size() > 10) {
      cout << I->first.first->name() << "\n";
    }*/
  }
  
  //TODO: Apply forwarding
}

void Scheduler::fillInputSpots(Schedule* sched,SbPDG_Input* pdginst,
                              int config, vector<sbnode*>& spots) {
  spots.clear();
  
  SubModel::const_input_iterator I,E;
  for(I=_sbModel->subModel()->input_begin(),
      E=_sbModel->subModel()->input_end(); I!=E; ++I) {
     sbinput* cand_input = const_cast<sbinput*>(&(*I));
    
    if(sched->pdgNodeOf(cand_input,config)==NULL) {
       spots.push_back(cand_input);
    }
  }
}

void Scheduler::fillOutputSpots(Schedule* sched,SbPDG_Output* pdginst,
                              int config, vector<sbnode*>& spots) {
  spots.clear();
  
  SubModel::const_output_iterator I,E;
  for(I=_sbModel->subModel()->output_begin(),
      E=_sbModel->subModel()->output_end(); I!=E; ++I) {
     sboutput* cand_output = const_cast<sboutput*>(&(*I));
    
    if(sched->pdgNodeOf(cand_output,config)==NULL) {
       spots.push_back(cand_output);
    }

  }
}

void Scheduler::fillInstSpots(Schedule* sched,SbPDG_Inst* pdginst,
                              int config, vector<sbnode*>& spots) {
  spots.clear();
  
  for(int i = 2; i < _sbModel->subModel()->sizex(); ++i) {
    sbfu* cand_fu = _sbModel->subModel()->fuAt(i,0);
    
    if((cand_fu->fu_def()==NULL || cand_fu->fu_def()->is_cap(pdginst->inst()))
       && sched->pdgNodeOf(cand_fu,config)==NULL) {
       spots.push_back(cand_fu);
    }
  }
  
  for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
      sbfu* cand_fu = _sbModel->subModel()->fuAt(i,j);
      
      if((cand_fu->fu_def()==NULL||cand_fu->fu_def()->is_cap(pdginst->inst()))
         && sched->pdgNodeOf(cand_fu,config)==NULL) {
         spots.push_back(cand_fu);
      }
    }
  }
}

void Scheduler::scheduleNode(Schedule* sched, SbPDG_Node* pdgnode) {
  
  int bestScore=MAX_ROUTE; //a big number
  CandidateRouting* bestRouting = new CandidateRouting();
  sbnode* bestspot;
  int bestconfig;
  
  CandidateRouting* curRouting = new CandidateRouting();
  
  std::vector<sbnode*> spots;
  
  //for each configuration
  for(int config = 0; config < sched->nConfigs(); ++config) {
    if(SbPDG_Inst* pdginst= dynamic_cast<SbPDG_Inst*>(pdgnode))  { 
      fillInstSpots(sched, pdginst, config, spots);             //all possible candidates based on FU capability 
    } else if(SbPDG_Input* pdg_in = dynamic_cast<SbPDG_Input*>(pdgnode)) {
      fillInputSpots(sched,pdg_in,config,spots); 
    } else if(SbPDG_Output* pdg_out = dynamic_cast<SbPDG_Output*>(pdgnode)) {
      fillOutputSpots(sched,pdg_out,config,spots); 
    }
   
    //populate a scheduling score for each of canidate sbspot
    for(unsigned i=0; i < spots.size(); i++) {
      sbnode* cand_spot = spots[i];
      
      curRouting->routing.clear();
      curRouting->forwarding.clear();
      
      int curScore = scheduleHere(sched, pdgnode, cand_spot, config,*curRouting,bestScore);
                  
      if(curScore < bestScore) {
        bestScore=curScore;
        bestspot=cand_spot;
        bestconfig=config;
        std::swap(bestRouting,curRouting);
      }
      
      if(bestScore<=1)  {
        applyRouting(sched,pdgnode,bestspot,bestconfig,bestRouting);
        return;
      }//apply routing step
    
    }//for loop -- check for all sbnode spots
  }
  
  
  //TODO: If not scheduled, then increase the numConfigs, and try again
  
  if(bestScore < MAX_ROUTE) {
    applyRouting(sched,pdgnode,bestspot,bestconfig,bestRouting);
  } else {
    cout << "no route found for pdgnode: " << pdgnode->name() << "\n";
    
    /*
    SbPDG_Node::const_edge_iterator I,E;
    for(I=pdgnode->ops_begin(), E=pdgnode->ops_end();I!=E;++I) {
      if(*I == NULL) { continue; } //could be immediate
      SbPDG_Edge* source_pdgegde = (*I);
      SbPDG_Node* source_pdgnode = source_pdgegde->def();

      //route edge if source pdgnode is scheduled
      if(sched->isScheduled(source_pdgnode)) {
        pair<sbnode*,int> source_loc = sched->locationOf(source_pdgnode);
       
        curRouting->routing.clear();
        curRouting->forwarding.clear();
        int score = route_to_output(sched, source_pdgegde, source_loc.first,source_loc.second,*curRouting,0);
        if(score < MAX_ROUTE) {
          applyRouting(sched,source_pdgnode,NULL,source_loc.second,curRouting);
        } else {
          cout << "could not patch from " << source_pdgnode->name() << "\n";
        }
      }
    }
    */
    
    
  }
}

int Scheduler::scheduleHere(Schedule* sched, SbPDG_Node* n, 
                                sbnode* here, int config, 
                                CandidateRouting& candRouting,
                                int bestScore) {
  int score=0;
  
  SbPDG_Node::const_edge_iterator I,E;
  
  for(I=n->ops_begin(), E=n->ops_end();I!=E;++I) {
    if(*I == NULL) { continue; } //could be immediate
    SbPDG_Edge* source_pdgegde = (*I);
    SbPDG_Node* source_pdgnode = source_pdgegde->def();     //could be input node also

    //route edge if source pdgnode is scheduled
    if(sched->isScheduled(source_pdgnode)) {
      pair<sbnode*,int> source_loc = sched->locationOf(source_pdgnode); //scheduled location
     
      //route using source node, sbnode
      score += route(sched, source_pdgegde, source_loc.first, here,config,candRouting,bestScore-score);
      //cout << n->name() << " " << here->name() << " " << score << "\n";
      if(score>bestScore) return MAX_ROUTE;
    }
  
  }
  
  SbPDG_Node::const_edge_iterator Iu,Eu;
  for(Iu=n->uses_begin(), Eu=n->uses_end();Iu!=Eu;++Iu) {
    SbPDG_Edge* use_pdgedge = (*Iu); 
    SbPDG_Node* use_pdgnode = use_pdgedge->use();
     
     //route edge if source pdgnode is scheduled
     if(sched->isScheduled(use_pdgnode)) {
       pair<sbnode*,int> use_loc = sched->locationOf(use_pdgnode);
       
       score += route(sched, use_pdgedge, here, use_loc.first,config,candRouting,bestScore-score);
       //cout << n->name() << " " << here->name() << " " << score << "\n";
       if(score>bestScore) return MAX_ROUTE;
     }
  }
  
  return score;
}

//routes only inside a configuration
int Scheduler::route(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest,
                     int config, CandidateRouting& candRouting, int scoreLeft) {
  
  list<sbnode*> openset;
  map<sbnode*,int> node_dist;
  map<sbnode*,sblink*> came_from;
  bool found_dest = false;
  
  openset.push_back(source);
  
  SbPDG_Node* pdgnode = pdgedge->def();
  
  //fill up worklist with nodes reachable 
  Schedule::link_iterator I,E;
  for(I=sched->links_begin(pdgnode),E=sched->links_end(pdgnode);I!=E;++I) {
    if(I->second==config) {
      openset.push_back(I->first->dest());
    }
  }
  
  while(!openset.empty()) {
    sbnode* node = openset.front(); 
    openset.pop_front();
    
    //don't search this node if it's too much anyways
    //if(x->links+1>=cost_allotted) continue; 
    
    //check neighboring nodes
    sbnode::const_iterator I,E;
    for(I = node->obegin(), E = node->oend(); I!=E; ++I)
    {
      sblink* link = *I;
      
      //check if connection is closed..
      SbPDG_Node* sched_exist_pdg = sched->pdgNodeOf(link,config);
      if(sched_exist_pdg!=NULL && sched_exist_pdg!=pdgnode) continue;
      
      pair<sblink*,int> p = make_pair(link,config);
      SbPDG_Node* cand_exist_pdg = candRouting.routing.count(p)==0 ? NULL :
                                  candRouting.routing[p]->def();
      if(cand_exist_pdg!=NULL && cand_exist_pdg!=pdgnode) continue;
      
      
      sbnode* next = link->dest();
      
      if(next==_sbModel->subModel()->cross_switch()) continue;
      if(next==_sbModel->subModel()->load_slice()  ) continue;
      if(node_dist.count(next)!=0) continue; //it has been looked at, or will be looked at
      
      found_dest=(next==dest);

      if(dynamic_cast<sbfu*>(next) && !found_dest) continue;  //don't route through fu

      came_from[next] = link;
      node_dist[next] = node_dist[node]+1;

      if(found_dest) break; 
      
      openset.push_back(next);
    }
    if(found_dest) break;
  }
    
  if(!found_dest) return MAX_ROUTE;  //routing failed, no routes exist!
  
  int score;
  score = node_dist[dest];
  
  sbnode* x = dest;
  while(came_from.count(x)!=0) {
    sblink* link = came_from[x];
    candRouting.routing[make_pair(link,config)]=pdgedge;
    x=link->orig();
  }
  
  return score;
}

//routes only inside a configuration
int Scheduler::route_to_output(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source,
                     int config, CandidateRouting& candRouting, int scoreLeft) {
  
  list<sbnode*> openset;
  map<sbnode*,int> node_dist;
  map<sbnode*,sblink*> came_from;
  bool found_dest = false;
  
  openset.push_back(source);
  
  SbPDG_Node* pdgnode = pdgedge->def();
  
  //fill up worklist with nodes reachable 
  Schedule::link_iterator I,E;
  for(I=sched->links_begin(pdgnode),E=sched->links_end(pdgnode);I!=E;++I) {
    if(I->second==config) {
      openset.push_back(I->first->dest());
    }
  }
  
  sboutput* the_output = NULL;
      
  while(!openset.empty()) {
    sbnode* node = openset.front(); 
    openset.pop_front();
    
    //don't search this node if it's too much anyways
    //if(x->links+1>=cost_allotted) continue; 
    
    //check neighboring nodes
    sbnode::const_iterator I,E;
    for(I = node->obegin(), E = node->oend(); I!=E; ++I)
    {
      sblink* link = *I;
      
      //check if connection is closed..
      SbPDG_Node* sched_exist_pdg = sched->pdgNodeOf(link,config);
      if(sched_exist_pdg!=NULL && sched_exist_pdg!=pdgnode) continue;
      
      pair<sblink*,int> p = make_pair(link,config);
      SbPDG_Node* cand_exist_pdg = candRouting.routing.count(p)==0 ? NULL :
                                  candRouting.routing[p]->def();
      if(cand_exist_pdg!=NULL && cand_exist_pdg!=pdgnode) continue;
      
      
      sbnode* next = link->dest();
      
      if(next==_sbModel->subModel()->cross_switch()) continue;
      if(next==_sbModel->subModel()->load_slice()  ) continue;
      if(node_dist.count(next)!=0) continue; //it has been looked at, or will be looked at
      
      the_output = dynamic_cast<sboutput*>(next);
      found_dest=(the_output!=NULL);

      if(dynamic_cast<sbfu*>(next) && !found_dest) continue;  //don't route through fu

      came_from[next] = link;
      node_dist[next] = node_dist[node]+1;

      if(found_dest) break; 
      
      openset.push_back(next);
    }
    if(found_dest) break;
  }
    
  
  if(!found_dest) return MAX_ROUTE;  //routing failed, no routes exist!
  
  int score;
  score = node_dist[the_output];
  
  sbnode* x = the_output;
  while(came_from.count(x)!=0) {
    sblink* link = came_from[x];
    candRouting.routing[make_pair(link,config)]=pdgedge;
    x=link->orig();
  }
  
  return score;
}

bool Scheduler::check_res(SbPDG* sbPDG, SbModel* sbmodel) {
  int ninsts = sbPDG->inst_end() - sbPDG->inst_begin();

  int nfus = sbmodel->subModel()->sizex() * sbmodel->subModel()->sizey();

  if(ninsts > nfus) {
    cerr << "\n\nError: Too many instructions in SbPDG for given SBCONIG\n\n";
    exit(1);
  }

  bool failed_count_check=false;

  std::map<sb_inst_t,int> count_types;
  for(auto Ii=sbPDG->inst_begin(), Ei=sbPDG->inst_end(); Ii!=Ei; ++Ii) {
    count_types[(*Ii)->inst()]++;
  }

  for(auto& pair : count_types) {
    sb_inst_t sb_inst = pair.first;
    int pdg_count = pair.second;

    int fu_count =0;
    for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
      for(int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
        sbfu* cand_fu = _sbModel->subModel()->fuAt(i,j);
        if(cand_fu->fu_def()->is_cap(sb_inst)) {
          fu_count++;
        }
      }
    }
    if(fu_count < pdg_count) {
      failed_count_check=true;
      cerr << "Error: PDG has " << pdg_count << " " << name_of_inst(sb_inst) 
           << " insts, but only " << fu_count << " fus to support them\n";
    }
  }

  if(failed_count_check) {
    cerr << "\n\nError: FAILED Basic FU Count Check\n\n";
    exit(1);
  }
  //TODO: add code from printPortcompatibility here
  
  return true;
}


//GAMS Specific

#include "gams_models/softbrain_gams.h"
#include "gams_models/softbrain_gams_hw.h"
#include "gams_models/spill_model.h"
#include "gams_models/multi_model.h"
#include "gams_models/single_fixed_general.h"
#include "gams_models/timing_model.h"
#include "gams_models/hw_model.h"
#include "gams_models/stage_model.h"

//MIP START IS DEFUNCT NOW THAT THE SCHEDULER CAN'T KEEP UP WITH REQs OF PROBLEM
#define USE_MIP_START 0 

bool Scheduler::scheduleGAMS(SbPDG* sbPDG,Schedule*& schedule) {
  string hw_model          = string((const char*)gams_models_hw_model_gms);
  string timing_model      = string((const char*)gams_models_timing_model_gms);
  string stage_model       = string((const char*)gams_models_stage_model_gms);
  string softbrain_gams    = string((const char*)gams_models_softbrain_gams_gms);
  string softbrain_gams_hw = string((const char*)gams_models_softbrain_gams_hw_gms);

  //mkfifo("/tmp/gams_fifo",S_IRWXU);
  stringstream ss;
  ss << _gams_work_dir << "/softbrain.out";
  string gams_out_file = ss.str();
  
  ss.str(std::string());
  ss << "softbrain.gams";
  string gams_file_name = ss.str();
  
  system(("rm -f " + gams_out_file).c_str());
  
  #if USE_MIP_START 
  schedule = scheduleGreedyBFS(sbPDG); // Get the scheduled pdg object
  schedule->calcAssignEdgeLink();
  #else
  schedule = new Schedule(_sbModel,sbPDG);
  #endif

 
  //bool use_hw=true;
  bool use_hw=false;

  // ----------------- setup the sbmodel gams files --------------------------
  if(!_gams_files_setup) {
        
    // Print the Constraints
    ofstream ofs_constraints(_gams_work_dir+"/constraints.gams", ios::out);
    assert(ofs_constraints.good());
    //ofs_constraints << multi_model;
    if(use_hw) {
      ofs_constraints << hw_model;
    } else {
      //ofs_constraints << timing_model;
      ofs_constraints << stage_model;
    }

    ofs_constraints.close();

     // Print the kinds of instructions
    ofstream ofs_kinds(_gams_work_dir+"/softbrain_kind.gams", ios::out);
    assert(ofs_kinds.good());
    _sbModel->printGamsKinds(ofs_kinds);
    ofs_kinds.close();
  
    _gams_files_setup=true;
  }
  
  // Print the controlling file
  ofstream ofs_sb_gams(_gams_work_dir+"/"+gams_file_name, ios::out);
  assert(ofs_sb_gams.good());
  
  ofs_sb_gams << "option reslim=" << _reslim << ";\n"
                 << "option optcr="  <<  _optcr << ";\n"   
                 << "option optca="  <<  _optca << ";\n";
  if(use_hw) {
    ofs_sb_gams << softbrain_gams_hw;
  } else {
    ofs_sb_gams << softbrain_gams;
  }
  ofs_sb_gams.close();
  
  ofstream ofs_mipstart(_gams_work_dir+"/mip_start.gams", ios::out);
  assert(ofs_mipstart.good());

  #if USE_MIP_START
  //print mipstart
  Schedule::assign_node_iterator I,E;
  for(I = schedule->assign_node_begin(), E= schedule->assign_node_end();I!=E;++I) {
    sbnode* spot = I->first.first;
    int config = I->first.second;
    SbPDG_Node* pdgnode = I->second;
    ofs_mipstart << "Mn.l('" << pdgnode->gamsName() 
                 << "','" << spot->gams_name(config) << "')=1;\n";
  }
 

  Schedule::assign_link_iterator Il,El;
  for(Il = schedule->assign_link_begin(), 
      El= schedule->assign_link_end();Il!=El;++Il) {
    sblink* link = Il->first.first;
    int config = Il->first.second;
    SbPDG_Node* pdgnode = Il->second;
    //SbPDG_Edge* pdgedge = pdgnode->getLinkTowards(
    //ofs_sb_gams << "Mvl.l(" << pdgnode->gamsName() << "," << link->gamsName(config) << ")=1;\n";
    ofs_mipstart << "Mvl.l('" << pdgnode->gamsName() 
                 << "','" << link->gams_name(config) << "')=1;\n";
  }
  
  Schedule::assign_edgelink_iterator Ile,Ele;
  for(Ile = schedule->assign_edgelink_begin(), 
      Ele= schedule->assign_edgelink_end();Ile!=Ele;++Ile) {
    
    sblink* link = Ile->first.first;
    int config = Ile->first.second;
    set<SbPDG_Edge*>& edgelist= Ile->second;
    
    set<SbPDG_Edge*>::const_iterator Ie,Ee;
    for(Ie=edgelist.begin(), Ee=edgelist.end(); Ie!=Ee; ++Ie) {
      SbPDG_Edge* pdgedge = *Ie;
      
      ofs_mipstart << "Mel.l('" << pdgedge->gamsName() << "','" 
                 << link->gams_name(config) << "')=1;\n";
    }
  }
  
  ofs_mipstart << "Tv.l(v)=0;\n";
  
  {
   map<SbPDG_Node*,bool> seen;
  
  list<SbPDG_Node* > openset;
  SbPDG::const_input_iterator I,E;
  for(I=sbPDG->input_begin(),E=sbPDG->input_end();I!=E;++I) {
    SbPDG_Input* n = *I;
    //openset.push_back(n);
    ofs_mipstart << "Tv.l('" << n->gamsName() << "')=0;\n";
    SbPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(use_pdginst) {
        openset.push_back(use_pdginst);
      }
    }
  }
  
  while(!openset.empty()) {
    SbPDG_Node* n = openset.front(); 
    openset.pop_front();

    if(!seen[n]) {
      ofs_mipstart << "Tv.l('" << n->gamsName() << "')="
                   << "smax((v1,e)$(Gve(v1,e) and "
                   << "Gev(e,'" << n->gamsName() << "')),Tv.l(v1)"
                   << "+ sum(l,Mel.l(e,l)) + delta(e));\n";

    }
    seen[n]=true;
    
    SbPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(use_pdginst) {
        openset.push_back(use_pdginst);
      }
    }
  }

  }

  //ofs_mipstart << "Tv.l(v2) = smax((v1,e)$(Gve(v1,e) and Gev(e,v2)),Tv.l(v1) + sum(l,Ml.l(e,l))) + delta.l(e);\n";
  ofs_mipstart << "length.l=smax(v,Tv.l(v));\n";
  ofs_mipstart << "cost.l = 1000000* sum((iv,k)$kindV(K,iv),(1-sum(n$(kindN(K,n)), Mn.l(iv, n)))) +  1000 * length.l + sum(l,sum(v,Mvl.l(v,l)));\n";
  ofs_mipstart << "display Tv.l;\n";
  ofs_mipstart << "display length.l;\n";
  ofs_mipstart << "display cost.l;\n";
  ofs_mipstart.close();
#endif

  schedule->clearAll();

  cout << "Total Nodes: " << sbPDG->num_nodes() << "\n";
  
  int numInsts = sbPDG->inst_end()-sbPDG->inst_begin();
  int configSize = _sbModel->subModel()->sizex() * _sbModel->subModel()->sizey();
  int n_configs = max(numInsts / configSize + (numInsts % configSize>0),1);
  cout << "Total Insts: " <<  numInsts << "\n";
  //assert(numInsts > 0);

  //cout << "Softbrain Scheduler -- Using: " << n_configs << " Configs\n";
  

  // Print the softbrain model   
  ofstream ofs_sb_model(_gams_work_dir + "/softbrain_model.gams", ios::out);
  assert(ofs_sb_model.good());
  gamsToSbnode.clear(); gamsToSblink.clear();
  _sbModel->subModel()->PrintGamsModel(ofs_sb_model,gamsToSbnode,gamsToSblink,
                                       gamsToSbswitch,gamsToPortN,n_configs);

  cout << gamsToSbnode.size() << " " << gamsToSblink.size() << " " << gamsToSbswitch.size() << "\n";

  ofs_sb_model.close();
  
  // ----------------- setup the pdg gams files ------------------------------
  ofstream ofs_sb_pdg(_gams_work_dir + "/softbrain_pdg.gams", ios::out);
  if(ofs_sb_pdg.fail()) {
    cerr << "could not open " + _gams_work_dir + "/softbrain_pdg.gams";
    return false;
  }
  gamsToPdgnode.clear(); 
  gamsToPdgedge.clear();
  gamsToPortV.clear();

  //Also populates these maps
  //--gamsToPdgnode
  //--gamsToPdgegde
  //--gamsToPortV
  sbPDG->printGams(ofs_sb_pdg,gamsToPdgnode,gamsToPdgedge,gamsToPortV);
  sbPDG->printPortCompatibilityWith(ofs_sb_pdg,_sbModel);
  ofs_sb_pdg.close();
  
  // ----------------- run gams! --------------------------------------------
  if (_use_server) {
    char buf[1024];
    std::string fullname = std::string(getcwd(buf, 1023));
    fullname += std::string("/") + gams_file_name;
    requestGams(fullname.c_str());
  } else {
    stringstream ss_cmd;
    ss_cmd << "gams " << gams_file_name << " wdir=" << _gams_work_dir;
    if(_showGams) {
      ss_cmd << " -lo=3";
    } else {
       ss_cmd << " -o=/dev/null -lo=2"; 
    }
    cout << ss_cmd.str().c_str() << "\n";
    system(ss_cmd.str().c_str());
  }

  // ----------------- parse output -----------------------------------------


  schedule->setNConfigs(n_configs); //update this if need less schedules?
  
  string line, edge_name, vertex_name, switch_name, link_name, out_link_name, sbnode_name,list_of_links,latency_str;
  ifstream gamsout(gams_out_file.c_str());
  enum {VtoN,VtoL,LtoL,EL,EDGE_DELAY,TIMING,PortMap,PASSTHROUGH,Parse_None}parse_stage;
  parse_stage=Parse_None;
  bool message_start=false, message_fus_ok=false, message_ports_ok=false;

  while(gamsout.good()) {  
    getline(gamsout,line);
    ModelParsing::trim_comments(line);
    ModelParsing::trim(line);

    if(line.empty()) {
      continue;
    }
    //if(ModelParsing::StartsWith(line,"#")) continue;
    if(line[0]=='[') {
      parse_stage = Parse_None; 
      if(ModelParsing::StartsWith(line,"[vertex-node-map]")) {
        parse_stage = VtoN; continue;
      } else if(ModelParsing::StartsWith(line,"[vertex-link-map]")) {
        parse_stage = VtoL; continue;
      } else if(ModelParsing::StartsWith(line,"[switch-map]")) {
        parse_stage = LtoL; continue;
      } else if(ModelParsing::StartsWith(line,"[extra-lat]")) {
        parse_stage = EL; continue;
      } else if(ModelParsing::StartsWith(line,"[edge-delay]")) {
        parse_stage = EDGE_DELAY; continue;
      } else if(ModelParsing::StartsWith(line,"[timing]")) {
        parse_stage = TIMING; continue;
      } else if(ModelParsing::StartsWith(line,"[passthrough]")) {
        parse_stage = PASSTHROUGH; continue;
      } else if(ModelParsing::StartsWith(line,"[port-port-map]")) {
        parse_stage = PortMap; continue;
      } else if(ModelParsing::StartsWith(line,"[status_message_begin_scheduling]")) {
        message_start=true; continue;
      } else if(ModelParsing::StartsWith(line,"[status_message_fus_ok]")) {
        message_fus_ok=true; continue;
      } else if(ModelParsing::StartsWith(line,"[status_message_ports_ok]")) {
        message_ports_ok=true; continue;
      } 
    }

    if(parse_stage==PortMap) {
      stringstream ss(line);
      getline(ss, vertex_name, ':');
      ss >> std::ws;
      getline(ss, sbnode_name, ' ');

      //cout << "PORT MAP: " << vertex_name << " " <<sbnode_name << "\n";

      ModelParsing::trim(vertex_name);
      ModelParsing::trim(sbnode_name);

      if(sbnode_name.empty()) {
        cout << "failed to parse line: \"" << line << "\"\n";
        assert(0);
      }

      SbPDG_Vec* pv = gamsToPortV[vertex_name];
      assert(pv);
      std::pair<bool,int> pn = gamsToPortN[sbnode_name];  

      unsigned size_of_vp;
      if(pn.first) {
       size_of_vp = _sbModel->subModel()->io_interf().in_vports[pn.second].size();
      } else {
       size_of_vp = _sbModel->subModel()->io_interf().out_vports[pn.second].size();
      }

      std::vector<bool> mask;
      //mask.resize(pv->locMap().size());
      mask.resize(size_of_vp);
      
      while(ss.good()) {
        string ind_str;
        getline(ss, ind_str, ' ');
        ModelParsing::trim(ind_str);
        if(ind_str.empty()) continue;
        unsigned ind = (int)(stof(ind_str))-1;
        
        //cout << vertex_name << " " << sbnode_name << " " << ind << " " << size_of_vp << "\n";
        assert(ind < size_of_vp && "went off end of vec");
        assert(mask[ind]==false && "I already assigned this place in the vec!");

        mask[ind]=true;
      }
      //cout << "\n";

      schedule->assign_vport(pv,pn,mask);

    } else if(parse_stage==PASSTHROUGH) {
      stringstream ss(line);
      
      while(ss.good()) {
        getline(ss,sbnode_name, ' ');
        ModelParsing::trim(sbnode_name);
        if(sbnode_name.empty()) continue;
        
        sbnode* sbnode  = gamsToSbnode[sbnode_name].first;  
        if(sbnode==NULL) {
          cerr << "null sbnode:\"" << sbnode_name << "\"\n";
        }
        schedule->add_passthrough_node(sbnode);

      }

    } else if(parse_stage==TIMING) {
      stringstream ss(line);
      getline(ss, vertex_name, ':');
      getline(ss, latency_str, '.');
      ModelParsing::trim(vertex_name);
      ModelParsing::trim(latency_str);
      SbPDG_Node* pdgnode = gamsToPdgnode[vertex_name];

      int lat = stoi(latency_str);
      schedule->assign_lat(pdgnode,lat);

    } else if(parse_stage==EL) {
      stringstream ss(line);
      getline(ss, edge_name, ':');
      getline(ss, sbnode_name);
      //TODO: FINISH THIS IF EVER NEED EDGE -> LINK MAPPING

    } else if(parse_stage==EDGE_DELAY) {
      stringstream ss(line);
      getline(ss, edge_name, ':');

      ModelParsing::trim(edge_name);
      SbPDG_Edge* pdgedge = gamsToPdgedge[edge_name];
      assert(pdgedge);

      string delay_str;
      getline(ss, delay_str);
      ModelParsing::trim(delay_str);
      if(delay_str.empty()) continue;
      unsigned delay = (unsigned)(stof(delay_str));

      pdgedge->set_delay(delay);

    } else if(parse_stage==VtoN) {
      stringstream ss(line);
      getline(ss, vertex_name, ':');
      getline(ss, sbnode_name);
      ModelParsing::trim(vertex_name);
      ModelParsing::trim(sbnode_name);
      
      if(sbnode_name.empty()) {
        return false;
      }

      SbPDG_Node* pdgnode = gamsToPdgnode[vertex_name];
      sbnode* sbnode  = gamsToSbnode[sbnode_name].first;  
      int config = gamsToSbnode[sbnode_name].second; 
      
      if(vertex_name.empty()) continue;
      
      /*
      if(pdgnode==NULL) {
        cerr << "null pdgnode:" << vertex_name << "|" << sbnode_name << "\"\n";
      }
      if(sbnode==NULL) {
        cerr << "null sbnode: \"" << vertex_name << "|" << sbnode_name << "\"\n";
      }
      */
      
      schedule->assign_node(pdgnode,sbnode,config);
      
        /*if(sboutput* sbout = dynamic_cast<sboutput*>(sbnode) ) {
           cout << pdgnode->name() << " new=" << schedule->getPortFor(pdgnode) << "\n";
        }*/
        
      //schedule
    } else if (parse_stage==LtoL) { //PARSE SWITCH MAP --------------------------
      stringstream ss(line);
      getline(ss, switch_name, ':');
      ModelParsing::trim(switch_name);
      if(switch_name.empty()) continue;

      sbswitch* sbsw = gamsToSbswitch[switch_name].first;
      if(sbsw==NULL) {
        cerr << "null sbsw:\"" << switch_name << "\"\n";
      }

      while(ss.good()) {
        getline(ss, link_name, ' ');
        getline(ss, out_link_name, ',');

        ModelParsing::trim(link_name);
        ModelParsing::trim(out_link_name);

        if(link_name.empty()) continue;
        if(out_link_name.empty()) continue;
        sblink* slink = gamsToSblink[link_name].first;
        sblink* slink_out = gamsToSblink[out_link_name].first;
        assert(slink);
        assert(slink_out);
        schedule->assign_switch(sbsw,slink,slink_out);

      }
      
    } else if (parse_stage==VtoL) {
//      if(_assignSwitch.size()!=0) {
//        continue;
//      }

      stringstream ss(line);
      getline(ss, vertex_name, ':');
      //getline(ss, list_of_links);
      
      ModelParsing::trim(vertex_name);
      if(vertex_name.empty()) continue;
      
      SbPDG_Node* pdgnode = gamsToPdgnode[vertex_name];
      if(pdgnode==NULL) {
        cerr << "null pdgnode:\"" << vertex_name << "\"\n";
      }
      
      while(ss.good()) {
        getline(ss, link_name, ' ');
        ModelParsing::trim(link_name);
        if(link_name.empty()) continue;
        sblink* slink = gamsToSblink[link_name].first;
        int config = gamsToSblink[link_name].second; 
        
        if(slink==NULL) {
          cerr << "null slink:\"" << link_name << "\"\n";
        }
        
        schedule->assign_link(pdgnode,slink,config);
        
        if(_sbModel->subModel()->multi_config()) {
          if(slink->dest()==_sbModel->subModel()->load_slice()) {
            sboutput* sbout = dynamic_cast<sboutput*>(slink->orig());
            assert(sbout);
            
            //find output for this output edge
            SbPDG_Node::const_edge_iterator I,E;
            for(I=pdgnode->uses_begin(), E=pdgnode->uses_end();I!=E;++I) {
              if(SbPDG_Output* pdg_out = dynamic_cast<SbPDG_Output*>((*I)->use())) {
                schedule->assign_node(pdg_out,sbout,config);
              }
            }
            
            //if(SbPDG_Output* pdg_out = dynamic_cast<SbPDG_Output*>((*(pdgnode->uses_begin()))->use())) {
              //schedule->assign_node(pdg_out,sbout,config);
            //}
          }
          if(slink->orig()==_sbModel->subModel()->load_slice()) {
            sbinput* sbin = dynamic_cast<sbinput*>(slink->dest());
            assert(sbin);
            schedule->assign_node(pdgnode,sbin,config); 
          }
        } else {
          if(sbinput* sbin = dynamic_cast<sbinput*>(slink->orig())) {
            schedule->assign_node(pdgnode,sbin,config); 
          } else if(sboutput* sbout = dynamic_cast<sboutput*>(slink->dest())) {
            //find output for this output edge
            SbPDG_Node::const_edge_iterator I,E;
            for(I=pdgnode->uses_begin(), E=pdgnode->uses_end();I!=E;++I) {
              if(SbPDG_Output* pdg_out = dynamic_cast<SbPDG_Output*>((*I)->use())) {
                schedule->assign_node(pdg_out,sbout,config);
              }
            }
          }
        }
        
        /*
        if(sboutput* sbout = dynamic_cast<sboutput*>() ) {
           //cout << (*(pdgnode->uses_begin()))->use()->name() << " old=" << schedule->getPortFor(pdgnode);
           schedule->assign_node((*(pdgnode->uses_begin()))->use(),sbout);
           //cout << " , new=" << schedule->getPortFor(pdgnode) << "\n";
        }
        */
        
        
      }
    }
    
  }
 
  if(!message_start) {
    cerr << "\n\nError: Scheduling Not Started -- Likely Error in Gams Code Gen\n\n";
    exit(1);
  } else if (!message_fus_ok) {
    cerr << "\n\nError: Combination of FUs requested are NOT satisfiable with given SBCONFIG.\n\n";
    exit(1);
  } else if (!message_ports_ok) {
    cerr << "\n\nError: Port specifications are NOT satisfiable with given SBCONFIG.\n\n";
    exit(1);
  }

  //populate the forwardMap
  for(int config = 0; config < n_configs-1; ++config) {
    sbswitch* cross_switch = _sbModel->subModel()->cross_switch();
    
    sbnode::const_iterator I = cross_switch->ibegin(), E = cross_switch->iend();
    for(;I!=E;++I) {
      sblink* inlink = *I;
      sboutput* output = dynamic_cast<sboutput*>(inlink->orig());
      assert(output);
      if(SbPDG_Node* node = schedule->pdgNodeOf(inlink,config)) {
         
         for(int to_config = config +1; to_config < n_configs; ++to_config) {
           sbnode::const_iterator Io = cross_switch->obegin(), Eo = cross_switch->oend(); 
           for(;Io!=Eo;++Io) {
              sblink* outlink = *Io;
              if(schedule->pdgNodeOf(outlink,to_config-1)==node) {
                  sbinput* input = dynamic_cast<sbinput*>(outlink->dest());
                  if(input==0) {
                    cout << outlink->dest()->name(); 
                  }
                  assert(input);
                  schedule->addForward(config, output->port(),to_config,input->port());
              }
            }
         }
         
      }
    }
  }
  
  if(_showGams) {
    //Print the I/Os
    std::cout << "in/out mapping:";

    SbPDG::const_input_iterator Ii,Ei;
    for(Ii=schedule->sbpdg()->input_begin(),Ei=schedule->sbpdg()->input_end();Ii!=Ei;++Ii) {
      SbPDG_Input* in = *Ii;
      pair<int,int> p = schedule->getConfigAndPort(in);
      cout << in->name() << " " << p.second << ", ";
    }

    SbPDG::const_output_iterator Io,Eo;
    for(Io=schedule->sbpdg()->output_begin(),Eo=schedule->sbpdg()->output_end();Io!=Eo;++Io) {
      SbPDG_Output* out = *Io;
      pair<int,int> p = schedule->getConfigAndPort(out);
      cout << out->name() << " " << p.second << ", ";
    }
    std::cout << "\n";
  }
  
  
  return true;
}

void error(const char *msg)
{
  perror(msg);
  exit(0);
}

bool Scheduler::requestGams(const char *filename)
{
  int sockfd, portno, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;

  char cmd_buf[256];
  char buffer[1024];

  portno = 20202;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0)
    error("ERROR opening socket");
  server = gethostbyname("arcturus.cs.wisc.edu");
  if (server == NULL) {
    fprintf(stderr,"ERROR, no such host\n");
    exit(0);
  }
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr,
        (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
  serv_addr.sin_port = htons(portno);
  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    error("ERROR connecting");

  strcpy(cmd_buf, "run-gams");
  n = write(sockfd, cmd_buf, strlen(cmd_buf));
  if (n < 0)
    error("ERROR writing to socket");

  n = write(sockfd, filename,strlen(filename));
  if (n < 0)
    error("ERROR writing to socket");
  bzero(buffer,256);
  n = read(sockfd,buffer,255);
  if (n < 0)
    error("ERROR reading from socket");
  if (strcmp(buffer, "__DONE__") == 0)
    return true;
  error("Error running gams");
  return false;
}



/*
    ofstream sbpdg_ofs("softbrain_pdg.gams", ios::out);
    if(ofs_kinds.fail()) {
      cerr << "could not open pdgout.dot";
    }
    */
    
    
    
    
    
    
    
    
#if 0
//old code:

// Scrapped Equations


      
      *equation enforceHetero(K,v);
      *enforceHetero(K,v)$kindV(K,v)..     sum(n$(notKindN(K,n)), Mn(v, n)) =e= 0;
      
      

//debugging edges->links stuff
    option Mn:0:0:1;
    display Mn.l;
    option Ml:0:0:1;
    display Ml.l;

put "# Edge -> Links" /
loop((e),
    put e.tl ":"
    loop((l)$(Ml.l(e,l)<>0),
        put l.tl
    );
    put /
);
put "# Links -> Edges" /
loop((l),
    put l.tl ":"
    loop((e)$(Ml.l(e,l)<>0),
        put e.tl
    );
    put /
);

// non-literal string stuff
/*
    ofs_sb_gams << "$batinclude softbrain_kind.gams\n"
                   << "$batinclude softbrain_model.gams\n"
                   << "$batinclude softbrain_pdg.gams\n"
                   << "$batinclude constraints.gams\n";

    ofs_sb_gams <<  "option Mn:0:0:1;\n"
                   <<     "display Mn.l;\n"
                   <<     "option Ml:0:0:1;\n"
                   <<     "display Ml.l;\n";

    //ofs_sb_gams <<     "file outfile / \"/dev/tty\" /;\n"
    //ofs_sb_gams <<     "file outfile / \"/tmp/gams_fifo\" /;\n"
    ofs_sb_gams <<     "file outfile / \"" << gams_out_file << "\" /;\n"
                   <<     "outfile.pc=8;\n"
                   <<     "outfile.pw=4096\n"
                   <<     "put outfile;\n";

                  
    ofs_sb_gams <<     "put \"# Nodes -> Vertecies\" /\n"
                   <<     "loop((n),\n"
                   <<     "    put n.tl \":\";\n"
                   <<     "    loop((v)$(Mn.l(v,n)<>0),\n"
                   <<     "        put v.tl \n"
                   <<     "    );\n"
                   <<     "    put /\n"
                   <<     ");\n";
                   
    ofs_sb_gams <<   "put \"# Edge -> Links\" /\n"
                        "loop((e), \n"
                        "    put e.tl \":\"\n"
                        "    loop((l)$(Ml.l(e,l)<>0),\n"
                        "        put l.tl\n"
                        "    );\n"
              block of text          "    put /\n"
                        ");\n"
                        "put \"# Links -> Edges\" /\n"
                        "loop((l), \n"
                        "    put l.tl \":\"\n"
                        "    loop((e)$(Ml.l(e,l)<>0),\n"
                        "        put e.tl\n"
                        "    );\n"
                        "    put /\n"
                        ");\n";

                   
    ofs_sb_gams <<     "put \"[vertex-node-map]\" /\n"
                   <<     "loop((v),\n"
                   <<     "    put v.tl \":\";\n"
                   <<     "    loop((n)$(Mn.l(v,n)<>0),\n"
                   <<     "        put n.tl \n"
                   <<     "    );\n"
                   <<     "    put /\n"
                   <<     ");\n";                   
                   
    ofs_sb_gams <<     "put \"[vertex-link-map]\" /\n"
                   <<     "loop((v), \n"
                   <<     "    put v.tl \":\"\n"
                   <<     "    loop((l)$(Mvl.l(v,l)<>0),\n"
                   <<     "        put l.tl\n"
                   <<     "    );\n"
                   <<     "    put /\n"
                   <<     ");\n";
*/











//FIFO STUFF


      FILE *in;
  char buff[512];

  if(!(in = popen("gams softbrain.gams", "r"))) {
    return 1;
  }

  while(fgets(buff, sizeof(buff), in)!=NULL) {
    cout << "----------------------------xxx------------------------------";
    cout << buff;
    cout << "----------------------------xxx------------------------------";
  }
  pclose(in);

  gams_dump.txt
  
  
  FILE * gams_fifo;
  char line [4096];

  gams_fifo = fopen ("/tmp/gams_fifo" , "r");
  if (gams_fifo == NULL) perror ("Error opening file");
  else {
    while ( fgets ( line, sizeof line, gams_fifo ) != NULL ) {
      cout << "***" << line << "***\n";
      /*if(strcmp("[vertex-node-map]")==0) {
        
      }*/
    }

    fclose (gams_fifo);
  }
#endif
