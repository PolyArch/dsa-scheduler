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


Schedule* Scheduler::scheduleGreedyBFS(DyPDG* dyPDG) {
  Schedule* sched = new Schedule(_dyModel,dyPDG);
  sched->setNConfigs(1);
  
  map<DyPDG_Inst*,bool> seen;
  
  list<DyPDG_Inst* > openset;
  DyPDG::const_input_iterator I,E;
  for(I=dyPDG->input_begin(),E=dyPDG->input_end();I!=E;++I) {
    DyPDG_Input* n = *I;
    
    DyPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      DyPDG_Inst* use_pdginst = dynamic_cast<DyPDG_Inst*>((*I)->use());
      if(use_pdginst) {
        openset.push_back(use_pdginst);
      }
    }
  }
  
  while(!openset.empty()) {
    DyPDG_Inst* n = openset.front(); 
    openset.pop_front();
    
    if(!seen[n]) {
      scheduleNode(sched,n);
    }
    seen[n]=true;
    
    DyPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      DyPDG_Inst* use_pdginst = dynamic_cast<DyPDG_Inst*>((*I)->use());
      if(use_pdginst) {
        openset.push_back(use_pdginst);
      }
    }
  }
  
  
  /*
  DyPDG::const_inst_iterator Ii,Ei;
  for(Ii=dyPDG->inst_begin(), Ei=dyPDG->inst_end(); Ii!=Ei; ++Ii) {
    DyPDG_Inst* pdginst = *Ii; 
    scheduleNode(sched,pdginst);
  }*/
  
  
  
  
  
  for(I=dyPDG->input_begin(),E=dyPDG->input_end();I!=E;++I) {
    DyPDG_Input* pdgin = *I;
    scheduleNode(sched,pdgin);
  }
    
  DyPDG::const_output_iterator Io,Eo;
  for(Io=dyPDG->output_begin(),Eo=dyPDG->output_end();Io!=Eo;++Io) {
    DyPDG_Output* pdgout = *Io;
    scheduleNode(sched,pdgout);
  }
  return sched;
}

void Scheduler::applyRouting(Schedule* sched, DyPDG_Node* pdgnode,
                             dynode* here, int config, CandidateRouting* candRouting){
  
  //cout << "pdgnode: " << pdgnode->name()  << " dynode: " << here->name() 
  //<< " nlinks: " << candRouting->routing.size() << "\n";
  
  sched->assign_node(pdgnode,here,config);
  std::map< std::pair<SB_CONFIG::dylink*,int>,DyPDG_Edge* >::iterator I,E;
  for(I= candRouting->routing.begin(), E=candRouting->routing.end();I!=E;++I) {
    sched->assign_link(I->second->def(),I->first.first, I->first.second);
    //sched->assign_edgelink(I->second,I->first.first, I->first.second);
    /*if(candRouting->routing.size() > 10) {
      cout << I->first.first->name() << "\n";
    }*/
  }
  
  //TODO: Apply forwarding
}

void Scheduler::fillInputSpots(Schedule* sched,DyPDG_Input* pdginst,
                              int config, vector<dynode*>& spots) {
  spots.clear();
  
  SubModel::const_input_iterator I,E;
  for(I=_dyModel->subModel()->input_begin(),
      E=_dyModel->subModel()->input_end(); I!=E; ++I) {
     dyinput* cand_input = const_cast<dyinput*>(&(*I));
    
    if(sched->pdgNodeOf(cand_input,config)==NULL) {
       spots.push_back(cand_input);
    }
  }
}

void Scheduler::fillOutputSpots(Schedule* sched,DyPDG_Output* pdginst,
                              int config, vector<dynode*>& spots) {
  spots.clear();
  
  SubModel::const_output_iterator I,E;
  for(I=_dyModel->subModel()->output_begin(),
      E=_dyModel->subModel()->output_end(); I!=E; ++I) {
     dyoutput* cand_output = const_cast<dyoutput*>(&(*I));
    
    if(sched->pdgNodeOf(cand_output,config)==NULL) {
       spots.push_back(cand_output);
    }

  }
}

void Scheduler::fillInstSpots(Schedule* sched,DyPDG_Inst* pdginst,
                              int config, vector<dynode*>& spots) {
  spots.clear();
  
  for(int i = 2; i < _dyModel->subModel()->sizex(); ++i) {
    dyfu* cand_fu = _dyModel->subModel()->fuAt(i,0);
    
    if((cand_fu->fu_def()==NULL||cand_fu->fu_def()->is_cap(pdginst->inst()))
       && sched->pdgNodeOf(cand_fu,config)==NULL) {
       spots.push_back(cand_fu);
    }
  }
  
  for(int i = 0; i < _dyModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _dyModel->subModel()->sizey(); ++j) {
      dyfu* cand_fu = _dyModel->subModel()->fuAt(i,j);
      
      if((cand_fu->fu_def()==NULL||cand_fu->fu_def()->is_cap(pdginst->inst()))
         && sched->pdgNodeOf(cand_fu,config)==NULL) {
         spots.push_back(cand_fu);
      }
    }
  }
}

void Scheduler::scheduleNode(Schedule* sched,DyPDG_Node* pdgnode) {
  
  int bestScore=MAX_ROUTE; //a big number
  CandidateRouting* bestRouting = new CandidateRouting();
  dynode* bestspot;
  int bestconfig;
  
  CandidateRouting* curRouting = new CandidateRouting();
  
  std::vector<dynode*> spots;
  for(int config = 0; config < sched->nConfigs(); ++config) {
    if(DyPDG_Inst* pdginst= dynamic_cast<DyPDG_Inst*>(pdgnode))  { 
      fillInstSpots(sched,pdginst,config,spots); 
    } else if(DyPDG_Input* pdg_in = dynamic_cast<DyPDG_Input*>(pdgnode)) {
      fillInputSpots(sched,pdg_in,config,spots); 
    } else if(DyPDG_Output* pdg_out = dynamic_cast<DyPDG_Output*>(pdgnode)) {
      fillOutputSpots(sched,pdg_out,config,spots); 
    }
    
    for(unsigned i=0; i < spots.size(); i++) {
      dynode* cand_spot = spots[i];
      
      curRouting->routing.clear();
      curRouting->forwarding.clear();
      
      int curScore = scheduleHere(sched,pdgnode,cand_spot,config,*curRouting,bestScore);
                  
      if(curScore < bestScore) {
        bestScore=curScore;
        bestspot=cand_spot;
        bestconfig=config;
        std::swap(bestRouting,curRouting);
      }
      
      if(bestScore<=1)  {
        applyRouting(sched,pdgnode,bestspot,bestconfig,bestRouting);
        return;
      }
    }
  }
  
  
  //TODO: If not scheduled, then increase the numConfigs, and try again
  
  if(bestScore < MAX_ROUTE) {
    applyRouting(sched,pdgnode,bestspot,bestconfig,bestRouting);
  } else {
    cout << "no route found for pdgnode: " << pdgnode->name() << "\n";
    
    /*
    DyPDG_Node::const_edge_iterator I,E;
    for(I=pdgnode->ops_begin(), E=pdgnode->ops_end();I!=E;++I) {
      if(*I == NULL) { continue; } //could be immediate
      DyPDG_Edge* source_pdgegde = (*I);
      DyPDG_Node* source_pdgnode = source_pdgegde->def();

      //route edge if source pdgnode is scheduled
      if(sched->isScheduled(source_pdgnode)) {
        pair<dynode*,int> source_loc = sched->locationOf(source_pdgnode);
       
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

int Scheduler::scheduleHere(Schedule* sched, DyPDG_Node* n, 
                                dynode* here, int config, 
                                CandidateRouting& candRouting,
                                int bestScore) {
  int score=0;
  
  DyPDG_Node::const_edge_iterator I,E;
  for(I=n->ops_begin(), E=n->ops_end();I!=E;++I) {
    if(*I == NULL) { continue; } //could be immediate
    DyPDG_Edge* source_pdgegde = (*I);
    DyPDG_Node* source_pdgnode = source_pdgegde->def();

    //route edge if source pdgnode is scheduled
    if(sched->isScheduled(source_pdgnode)) {
      pair<dynode*,int> source_loc = sched->locationOf(source_pdgnode);
      
      score += route(sched, source_pdgegde, source_loc.first, here,config,candRouting,bestScore-score);
      //cout << n->name() << " " << here->name() << " " << score << "\n";
      if(score>bestScore) return MAX_ROUTE;
    }
  }
  
  DyPDG_Node::const_edge_iterator Iu,Eu;
  for(Iu=n->uses_begin(), Eu=n->uses_end();Iu!=Eu;++Iu) {
    DyPDG_Edge* use_pdgedge = (*Iu); 
    DyPDG_Node* use_pdgnode = use_pdgedge->use();
     
     //route edge if source pdgnode is scheduled
     if(sched->isScheduled(use_pdgnode)) {
       pair<dynode*,int> use_loc = sched->locationOf(use_pdgnode);
       
       score += route(sched, use_pdgedge, here, use_loc.first,config,candRouting,bestScore-score);
       //cout << n->name() << " " << here->name() << " " << score << "\n";
       if(score>bestScore) return MAX_ROUTE;
     }
  }
  
  return score;
}

//routes only inside a configuration
int Scheduler::route(Schedule* sched, DyPDG_Edge* pdgedge, dynode* source, dynode* dest,
                     int config, CandidateRouting& candRouting, int scoreLeft) {
  
  list<dynode*> openset;
  map<dynode*,int> node_dist;
  map<dynode*,dylink*> came_from;
  bool found_dest = false;
  
  openset.push_back(source);
  
  DyPDG_Node* pdgnode = pdgedge->def();
  
  //fill up worklist with nodes reachable 
  Schedule::link_iterator I,E;
  for(I=sched->links_begin(pdgnode),E=sched->links_end(pdgnode);I!=E;++I) {
    if(I->second==config) {
      openset.push_back(I->first->dest());
    }
  }
  
  while(!openset.empty()) {
    dynode* node = openset.front(); 
    openset.pop_front();
    
    //don't search this node if it's too much anyways
    //if(x->links+1>=cost_allotted) continue; 
    
    //check neighboring nodes
    dynode::const_iterator I,E;
    for(I = node->obegin(), E = node->oend(); I!=E; ++I)
    {
      dylink* link = *I;
      
      //check if connection is closed..
      DyPDG_Node* sched_exist_pdg = sched->pdgNodeOf(link,config);
      if(sched_exist_pdg!=NULL && sched_exist_pdg!=pdgnode) continue;
      
      pair<dylink*,int> p = make_pair(link,config);
      DyPDG_Node* cand_exist_pdg = candRouting.routing.count(p)==0 ? NULL :
                                  candRouting.routing[p]->def();
      if(cand_exist_pdg!=NULL && cand_exist_pdg!=pdgnode) continue;
      
      
      dynode* next = link->dest();
      
      if(next==_dyModel->subModel()->cross_switch()) continue;
      if(next==_dyModel->subModel()->load_slice()  ) continue;
      if(node_dist.count(next)!=0) continue; //it has been looked at, or will be looked at
      
      found_dest=(next==dest);

      if(dynamic_cast<dyfu*>(next) && !found_dest) continue;  //don't route through fu

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
  
  dynode* x = dest;
  while(came_from.count(x)!=0) {
    dylink* link = came_from[x];
    candRouting.routing[make_pair(link,config)]=pdgedge;
    x=link->orig();
  }
  
  return score;
}

//routes only inside a configuration
int Scheduler::route_to_output(Schedule* sched, DyPDG_Edge* pdgedge, dynode* source,
                     int config, CandidateRouting& candRouting, int scoreLeft) {
  
  list<dynode*> openset;
  map<dynode*,int> node_dist;
  map<dynode*,dylink*> came_from;
  bool found_dest = false;
  
  openset.push_back(source);
  
  DyPDG_Node* pdgnode = pdgedge->def();
  
  //fill up worklist with nodes reachable 
  Schedule::link_iterator I,E;
  for(I=sched->links_begin(pdgnode),E=sched->links_end(pdgnode);I!=E;++I) {
    if(I->second==config) {
      openset.push_back(I->first->dest());
    }
  }
  
  dyoutput* the_output = NULL;
      
  while(!openset.empty()) {
    dynode* node = openset.front(); 
    openset.pop_front();
    
    //don't search this node if it's too much anyways
    //if(x->links+1>=cost_allotted) continue; 
    
    //check neighboring nodes
    dynode::const_iterator I,E;
    for(I = node->obegin(), E = node->oend(); I!=E; ++I)
    {
      dylink* link = *I;
      
      //check if connection is closed..
      DyPDG_Node* sched_exist_pdg = sched->pdgNodeOf(link,config);
      if(sched_exist_pdg!=NULL && sched_exist_pdg!=pdgnode) continue;
      
      pair<dylink*,int> p = make_pair(link,config);
      DyPDG_Node* cand_exist_pdg = candRouting.routing.count(p)==0 ? NULL :
                                  candRouting.routing[p]->def();
      if(cand_exist_pdg!=NULL && cand_exist_pdg!=pdgnode) continue;
      
      
      dynode* next = link->dest();
      
      if(next==_dyModel->subModel()->cross_switch()) continue;
      if(next==_dyModel->subModel()->load_slice()  ) continue;
      if(node_dist.count(next)!=0) continue; //it has been looked at, or will be looked at
      
      the_output = dynamic_cast<dyoutput*>(next);
      found_dest=(the_output!=NULL);

      if(dynamic_cast<dyfu*>(next) && !found_dest) continue;  //don't route through fu

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
  
  dynode* x = the_output;
  while(came_from.count(x)!=0) {
    dylink* link = came_from[x];
    candRouting.routing[make_pair(link,config)]=pdgedge;
    x=link->orig();
  }
  
  return score;
}


#include "spill_model.cpp"
#include "multi_model.cpp"
#include "single_fixed_general.cpp"
#include "dyser_gams.cpp"
#include "dyser_gams_hw.cpp"
#include "timing_model.cpp"
#include "hw_model.cpp"


bool Scheduler::scheduleGAMS(DyPDG* dyPDG,Schedule*& schedule) {
  //mkfifo("/tmp/gams_fifo",S_IRWXU);
  stringstream ss;
  ss << _gams_work_dir << "/dyser.out";
  string gams_out_file = ss.str();
  
  ss.str(std::string());
  ss << "dyser.gams";
  string gams_file_name = ss.str();
  
  
  //schedule = new Schedule(_dyModel,dyPDG);
  schedule = scheduleGreedyBFS(dyPDG);
  schedule->calcAssignEdgeLink();
  
  //bool use_hw=true;
  bool use_hw=false;

  // ----------------- setup the dymodel gams files --------------------------
  if(!_gams_files_setup) {
        
    // Print the Constraints
    ofstream ofs_constraints(_gams_work_dir+"/constraints.gams", ios::out);
    assert(ofs_constraints.good());
    //ofs_constraints << multi_model;
    if(use_hw) {
      ofs_constraints << hw_model;
    } else {
      ofs_constraints << timing_model;
    }

    ofs_constraints.close();

     // Print the kinds of instructions
    ofstream ofs_kinds(_gams_work_dir+"/dyser_kind.gams", ios::out);
    assert(ofs_kinds.good());
    _dyModel->printGamsKinds(ofs_kinds);
    ofs_kinds.close();
  
    _gams_files_setup=true;
  }
  
  // Print the controlling file
  ofstream ofs_dyser_gams(_gams_work_dir+"/"+gams_file_name, ios::out);
  assert(ofs_dyser_gams.good());
  
  ofs_dyser_gams << "option reslim=" << _reslim << ";\n"
                 << "option optcr="  <<  _optcr << ";\n"   
                 << "option optca="  <<  _optca << ";\n";
  if(use_hw) {
    ofs_dyser_gams << dyser_gams_hw;
  } else {
    ofs_dyser_gams << dyser_gams;
  }
  ofs_dyser_gams.close();
  
  ofstream ofs_mipstart(_gams_work_dir+"/mip_start.gams", ios::out);
  assert(ofs_mipstart.good());

  
  //print mipstart
  Schedule::assign_node_iterator I,E;
  for(I = schedule->assign_node_begin(), E= schedule->assign_node_end();I!=E;++I) {
    dynode* spot = I->first.first;
    int config = I->first.second;
    DyPDG_Node* pdgnode = I->second;
    ofs_mipstart << "Mn.l('" << pdgnode->gamsName() 
                 << "','" << spot->gams_name(config) << "')=1;\n";
  }
 

  Schedule::assign_link_iterator Il,El;
  for(Il = schedule->assign_link_begin(), 
      El= schedule->assign_link_end();Il!=El;++Il) {
    dylink* link = Il->first.first;
    int config = Il->first.second;
    DyPDG_Node* pdgnode = Il->second;
    //DyPDG_Edge* pdgedge = pdgnode->getLinkTowards(
    //ofs_dyser_gams << "Mvl.l(" << pdgnode->gamsName() << "," << link->gamsName(config) << ")=1;\n";
    ofs_mipstart << "Mvl.l('" << pdgnode->gamsName() 
                 << "','" << link->gams_name(config) << "')=1;\n";
  }
  
  Schedule::assign_edgelink_iterator Ile,Ele;
  for(Ile = schedule->assign_edgelink_begin(), 
      Ele= schedule->assign_edgelink_end();Ile!=Ele;++Ile) {
    
    dylink* link = Ile->first.first;
    int config = Ile->first.second;
    set<DyPDG_Edge*>& edgelist= Ile->second;
    
    set<DyPDG_Edge*>::const_iterator Ie,Ee;
    for(Ie=edgelist.begin(), Ee=edgelist.end(); Ie!=Ee; ++Ie) {
      DyPDG_Edge* pdgedge = *Ie;
      
      ofs_mipstart << "Mel.l('" << pdgedge->gamsName() << "','" 
                 << link->gams_name(config) << "')=1;\n";
    }
  }
  
  ofs_mipstart << "Tv.l(v)=0;\n";
  
  {
   map<DyPDG_Node*,bool> seen;
  
  list<DyPDG_Node* > openset;
  DyPDG::const_input_iterator I,E;
  for(I=dyPDG->input_begin(),E=dyPDG->input_end();I!=E;++I) {
    DyPDG_Input* n = *I;
    //openset.push_back(n);
    ofs_mipstart << "Tv.l('" << n->gamsName() << "')=0;\n";
    DyPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      DyPDG_Inst* use_pdginst = dynamic_cast<DyPDG_Inst*>((*I)->use());
      if(use_pdginst) {
        openset.push_back(use_pdginst);
      }
    }
  }
  
  while(!openset.empty()) {
    DyPDG_Node* n = openset.front(); 
    openset.pop_front();

    if(!seen[n]) {
      ofs_mipstart << "Tv.l('" << n->gamsName() << "')="
                   << "smax((v1,e)$(Gve(v1,e) and "
                   << "Gev(e,'" << n->gamsName() << "')),Tv.l(v1)"
                   << "+ sum(l,Mel.l(e,l)) + delta(e));\n";

    }
    seen[n]=true;
    
    DyPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      DyPDG_Inst* use_pdginst = dynamic_cast<DyPDG_Inst*>((*I)->use());
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
  
  schedule->clearAll();

  cout << "numNodes in scheduler: " << dyPDG->num_nodes() << "\n";
  
  int numInsts = dyPDG->inst_end()-dyPDG->inst_begin();
  int configSize = _dyModel->subModel()->sizex() * _dyModel->subModel()->sizey();
  int n_configs = numInsts / configSize + (numInsts%configSize>0);
  cout << "NumInsts in DyPDG: " <<  numInsts << "\n";
  assert(numInsts > 0);

  cout << "DySER Scheduler -- Using: " << n_configs << " Configs\n";
  
  // Print the dyser model   
  ofstream ofs_dyser_model(_gams_work_dir + "/dyser_model.gams", ios::out);
  assert(ofs_dyser_model.good());
  gamsToDynode.clear(); gamsToDylink.clear();
  _dyModel->subModel()->PrintGamsModel(ofs_dyser_model,gamsToDynode,gamsToDylink,
                                       gamsToDyswitch,gamsToPortN,n_configs);

  cout << gamsToDynode.size() << " " << gamsToDylink.size() << " " << gamsToDyswitch.size() << "\n";

  ofs_dyser_model.close();
  
  // ----------------- setup the pdg gams files ------------------------------
  ofstream ofs_dyser_pdg(_gams_work_dir + "/dyser_pdg.gams", ios::out);
  if(ofs_dyser_pdg.fail()) {
    cerr << "could not open " + _gams_work_dir + "/dyser_pdg.gams";
    return false;
  }
  gamsToPdgnode.clear(); 
  gamsToPdgedge.clear();
  gamsToPortV.clear();
  dyPDG->printGams(ofs_dyser_pdg,gamsToPdgnode,gamsToPdgedge,gamsToPortV);
  dyPDG->printPortCompatibilityWith(ofs_dyser_pdg,_dyModel);
  ofs_dyser_pdg.close();
  
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
  
  string line, edge_name, vertex_name, switch_name, link_name, out_link_name, dynode_name,list_of_links;
  ifstream gamsout(gams_out_file.c_str());
  enum  { VtoN,VtoL,LtoL,EL,PPL, Parse_None } parse_stage;
  parse_stage=Parse_None;
  while(gamsout.good()) {  
    getline(gamsout,line);
    ModelParsing::trim_comments(line);
    ModelParsing::trim(line);

    if(line.empty()) {
      continue;
    }
    //if(ModelParsing::StartsWith(line,"#")) continue;
    if(line[0]=='[') {
      if(ModelParsing::StartsWith(line,"[vertex-node-map]")) {
        parse_stage = VtoN; continue;
      } else if(ModelParsing::StartsWith(line,"[vertex-link-map]")) {
        parse_stage = VtoL; continue;
      } else if(ModelParsing::StartsWith(line,"[switch-map]")) {
        parse_stage = LtoL; continue;
      } else if(ModelParsing::StartsWith(line,"[extra-lat]")) {
        parse_stage = EL; continue;
      } else if(ModelParsing::StartsWith(line,"[port-port-map]")) {
        parse_stage = PPL; continue;
      } else {
        parse_stage = Parse_None;
      }
    }

    if(parse_stage==PPL) {
      stringstream ss(line);
      getline(ss, vertex_name, ':');
      getline(ss, dynode_name);

      //cout << "PORT MAP: " << vertex_name << " " <<dynode_name << "\n";

      ModelParsing::trim(vertex_name);
      ModelParsing::trim(dynode_name);

      DyPDG_Vec* pv = gamsToPortV[vertex_name];
      assert(pv);
      std::pair<bool,int> pn = gamsToPortN[dynode_name];  


      schedule->assign_vport(pv,pn);

    } else if(parse_stage==EL) {

      stringstream ss(line);
      getline(ss, edge_name, ':');
      getline(ss, dynode_name);
      //TODO: FINISH THIS IF EVER NEED EDGE -> LINK MAPPING

    } else if(parse_stage==VtoN) {
      stringstream ss(line);
      getline(ss, vertex_name, ':');
      getline(ss, dynode_name);
      ModelParsing::trim(vertex_name);
      ModelParsing::trim(dynode_name);
      
      
      DyPDG_Node* pdgnode = gamsToPdgnode[vertex_name];
      dynode* dysernode  = gamsToDynode[dynode_name].first;  
      int config = gamsToDynode[dynode_name].second; 
      
      if(vertex_name.empty()) continue;
      
      /*
      if(pdgnode==NULL) {
        cerr << "null pdgnode:" << vertex_name << "|" << dynode_name << "\"\n";
      }
      if(dysernode==NULL) {
        cerr << "null dysernode: \"" << vertex_name << "|" << dynode_name << "\"\n";
      }
      */
      
      
      schedule->assign_node(pdgnode,dysernode,config);
      
        /*if(dyoutput* dyout = dynamic_cast<dyoutput*>(dysernode) ) {
           cout << pdgnode->name() << " new=" << schedule->getPortFor(pdgnode) << "\n";
        }*/
        
      //schedule
    } else if (parse_stage==LtoL) { //PARSE SWITCH MAP --------------------------
      stringstream ss(line);
      getline(ss, switch_name, ':');
      ModelParsing::trim(switch_name);
      if(switch_name.empty()) continue;

      dyswitch* dysw = gamsToDyswitch[switch_name].first;
      if(dysw==NULL) {
        cerr << "null dysw:\"" << switch_name << "\"\n";
      }

      while(ss.good()) {
        getline(ss, link_name, ' ');
        getline(ss, out_link_name, ',');

        ModelParsing::trim(link_name);
        ModelParsing::trim(out_link_name);

        if(link_name.empty()) continue;
        if(out_link_name.empty()) continue;
        dylink* dlink = gamsToDylink[link_name].first;
        dylink* dlink_out = gamsToDylink[out_link_name].first;
        assert(dlink);
        assert(dlink_out);
        schedule->assign_switch(dysw,dlink,dlink_out);

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
      
      DyPDG_Node* pdgnode = gamsToPdgnode[vertex_name];
      if(pdgnode==NULL) {
        cerr << "null pdgnode:\"" << vertex_name << "\"\n";
      }
      
      while(ss.good()) {
        getline(ss, link_name, ' ');
        ModelParsing::trim(link_name);
        if(link_name.empty()) continue;
        dylink* dlink = gamsToDylink[link_name].first;
        int config = gamsToDylink[link_name].second; 
        
        if(dlink==NULL) {
          cerr << "null dlink:\"" << link_name << "\"\n";
        }
        
        schedule->assign_link(pdgnode,dlink,config);
        
        if(_dyModel->subModel()->multi_config()) {
          if(dlink->dest()==_dyModel->subModel()->load_slice()) {
            dyoutput* dyout = dynamic_cast<dyoutput*>(dlink->orig());
            assert(dyout);
            
            //find output for this output edge
            DyPDG_Node::const_edge_iterator I,E;
            for(I=pdgnode->uses_begin(), E=pdgnode->uses_end();I!=E;++I) {
              if(DyPDG_Output* pdg_out = dynamic_cast<DyPDG_Output*>((*I)->use())) {
                schedule->assign_node(pdg_out,dyout,config);
              }
            }
            
            //if(DyPDG_Output* pdg_out = dynamic_cast<DyPDG_Output*>((*(pdgnode->uses_begin()))->use())) {
              //schedule->assign_node(pdg_out,dyout,config);
            //}
          }
          if(dlink->orig()==_dyModel->subModel()->load_slice()) {
            dyinput* dyin = dynamic_cast<dyinput*>(dlink->dest());
            assert(dyin);
            schedule->assign_node(pdgnode,dyin,config); 
          }
        } else {
          if(dyinput* dyin = dynamic_cast<dyinput*>(dlink->orig())) {
            schedule->assign_node(pdgnode,dyin,config); 
          } else if(dyoutput* dyout = dynamic_cast<dyoutput*>(dlink->dest())) {
            //find output for this output edge
            DyPDG_Node::const_edge_iterator I,E;
            for(I=pdgnode->uses_begin(), E=pdgnode->uses_end();I!=E;++I) {
              if(DyPDG_Output* pdg_out = dynamic_cast<DyPDG_Output*>((*I)->use())) {
                schedule->assign_node(pdg_out,dyout,config);
              }
            }
          }
        }
        
        /*
        if(dyoutput* dyout = dynamic_cast<dyoutput*>() ) {
           //cout << (*(pdgnode->uses_begin()))->use()->name() << " old=" << schedule->getPortFor(pdgnode);
           schedule->assign_node((*(pdgnode->uses_begin()))->use(),dyout);
           //cout << " , new=" << schedule->getPortFor(pdgnode) << "\n";
        }
        */
        
        
      }
    }
    
  }
  
  //populate the forwardMap
  for(int config = 0; config < n_configs-1; ++config) {
    dyswitch* cross_switch = _dyModel->subModel()->cross_switch();
    
    dynode::const_iterator I = cross_switch->ibegin(), E = cross_switch->iend();
    for(;I!=E;++I) {
      dylink* inlink = *I;
      dyoutput* output = dynamic_cast<dyoutput*>(inlink->orig());
      assert(output);
      if(DyPDG_Node* node = schedule->pdgNodeOf(inlink,config)) {
         
         for(int to_config = config +1; to_config < n_configs; ++to_config) {
           dynode::const_iterator Io = cross_switch->obegin(), Eo = cross_switch->oend(); 
           for(;Io!=Eo;++Io) {
              dylink* outlink = *Io;
              if(schedule->pdgNodeOf(outlink,to_config-1)==node) {
                  dyinput* input = dynamic_cast<dyinput*>(outlink->dest());
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

    DyPDG::const_input_iterator Ii,Ei;
    for(Ii=schedule->dypdg()->input_begin(),Ei=schedule->dypdg()->input_end();Ii!=Ei;++Ii) {
      DyPDG_Input* in = *Ii;
      pair<int,int> p = schedule->getConfigAndPort(in);
      cout << in->name() << " " << p.second << ", ";
    }

    DyPDG::const_output_iterator Io,Eo;
    for(Io=schedule->dypdg()->output_begin(),Eo=schedule->dypdg()->output_end();Io!=Eo;++Io) {
      DyPDG_Output* out = *Io;
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
    ofstream dypdg_ofs("dyser_pdg.gams", ios::out);
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
    ofs_dyser_gams << "$batinclude dyser_kind.gams\n"
                   << "$batinclude dyser_model.gams\n"
                   << "$batinclude dyser_pdg.gams\n"
                   << "$batinclude constraints.gams\n";

    ofs_dyser_gams <<  "option Mn:0:0:1;\n"
                   <<     "display Mn.l;\n"
                   <<     "option Ml:0:0:1;\n"
                   <<     "display Ml.l;\n";

    //ofs_dyser_gams <<     "file outfile / \"/dev/tty\" /;\n"
    //ofs_dyser_gams <<     "file outfile / \"/tmp/gams_fifo\" /;\n"
    ofs_dyser_gams <<     "file outfile / \"" << gams_out_file << "\" /;\n"
                   <<     "outfile.pc=8;\n"
                   <<     "outfile.pw=4096\n"
                   <<     "put outfile;\n";

                  
    ofs_dyser_gams <<     "put \"# Nodes -> Vertecies\" /\n"
                   <<     "loop((n),\n"
                   <<     "    put n.tl \":\";\n"
                   <<     "    loop((v)$(Mn.l(v,n)<>0),\n"
                   <<     "        put v.tl \n"
                   <<     "    );\n"
                   <<     "    put /\n"
                   <<     ");\n";
                   
    ofs_dyser_gams <<   "put \"# Edge -> Links\" /\n"
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

                   
    ofs_dyser_gams <<     "put \"[vertex-node-map]\" /\n"
                   <<     "loop((v),\n"
                   <<     "    put v.tl \":\";\n"
                   <<     "    loop((n)$(Mn.l(v,n)<>0),\n"
                   <<     "        put n.tl \n"
                   <<     "    );\n"
                   <<     "    put /\n"
                   <<     ");\n";                   
                   
    ofs_dyser_gams <<     "put \"[vertex-link-map]\" /\n"
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

  if(!(in = popen("gams dyser.gams", "r"))) {
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
