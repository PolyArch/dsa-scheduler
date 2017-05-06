#include "scheduler.h"

using namespace SB_CONFIG;
using namespace std;

#include <unordered_map>
#include <fstream>
#include <sstream>


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
bool HeuristicScheduler::assignVectorInputs(SbPDG* sbPDG, Schedule* sched) {
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  sbio_interface si =  subModel->io_interf();
  vector<pair<int,int>> sd;

  int n = sbPDG->num_vec_input();

  sbPDG->sort_vec_in();
  si.sort_in_vports(sd);

  for(int j = 0; j < n; ++j) {
    SbPDG_VecInput* vec_in = sbPDG->vec_in(j);
    //cout << "Assigning Vector Port:" << vec_in->gamsName() <<"\n";
    
    bool found_vector_port = false;

    //for(auto& i : subModel->io_interf().in_vports) {

    int curNum = progress_getCurNum(Input);

    for(auto& i : sd) {
      progress_updateCurNum(Input, curNum);
      int vport_num = i.first;
      auto vport_id = std::make_pair(true/*input*/,vport_num);
      vector<pair<int, vector<int>>>& vport_desc = si.getDesc_I(vport_num);

      //Check if the vetcor port is 1. big enough & 2. unassigned
      if(vec_in->num_inputs() <= vport_desc.size() && 
         sched->vportOf(vport_id) == NULL) {
              std::vector<bool> mask; 
              mask.resize(vport_desc.size());

              bool ports_okay_to_use=true;

              //Check if it's okay to assign to these ports
              for(unsigned m=0; m < vec_in->num_inputs(); ++m) {
                //Get the sbnode corresponding to mask[m]
                int cgra_port_num = vport_desc[m].first;
                sbinput* cgra_in_port = subModel->get_input(cgra_port_num);

                if(sched->pdgNodeOf(cgra_in_port) != NULL) {
                  ports_okay_to_use=false;
                  break;
                } 
                progress_incCurNum(Input);
              }
              progress_saveBestNum(Input);
              if(!ports_okay_to_use) {
                //cout << "skipping this port assignment\n";
                continue; //don't assign these ports
              }
              // Assign Individual Elements
              for(unsigned m=0; m < vec_in->num_inputs(); ++m) {
                mask[m]=true;
               
                //Get the sbnode corresponding to mask[m]
                int cgra_port_num = vport_desc[m].first;
                sbinput* cgra_in_port = subModel->get_input(cgra_port_num);

                //Get the input pdgnode corresponding to m
                SbPDG_Node* sbpdg_input = vec_in->getInput(m);
                sched->assign_node(sbpdg_input,cgra_in_port,0/*config*/);
              }
              //Perform the vector assignment
              sched->assign_vport(vec_in,vport_id,mask);
              found_vector_port=true;
      }
      if(found_vector_port) {
        break;
      }
    }
    if(!found_vector_port) {
      progress_saveBestNum(Input);  
      //cout << "Could not find Input hardware vector port\n";
      return false;
    }
    //progress_incCurNum(Input);
  }
  progress_saveBestNum(Input);  
  return true;
}


bool HeuristicScheduler::assignVectorOutputs(SbPDG* sbPDG, Schedule* sched) {
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  CandidateRouting candRouting;  
  sbio_interface si =  subModel->io_interf();
  vector<pair<int,int>> sd;

  int n = sbPDG->num_vec_output();
  

  sbPDG->sort_vec_out();
  si.sort_out_vports(sd);

  for(int j = 0; j < n; ++j) {
    SbPDG_VecOutput* vec_out = sbPDG->vec_out(j);
    //cout << "Assigning Vector Port:" << vec_out->gamsName() <<"\n";
    
    bool found_vector_port = false;

    int curNum = progress_getCurNum(Output);

    for(auto& i : sd) {
      progress_updateCurNum(Output, curNum);
      int vport_num = i.first;
      auto vport_id = std::make_pair(true/*input*/,vport_num);
      vector<pair<int, vector<int>>>& vport_desc = si.getDesc_I(vport_num);

      //Check if the vetcor port is 1. big enough & 2. unassigned
      if(vec_out->num_outputs() <= vport_desc.size() && 
         sched->vportOf(vport_id) == NULL) {
              std::vector<bool> mask; 
              mask.resize(vport_desc.size());
            
              bool ports_okay_to_use=true;
              candRouting.clear();
              //Check if it's okay to assign to these ports
              for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
                //Get the sbnode corresponding to mask[m]
                int cgra_port_num = vport_desc[m].first;
                sboutput* cgra_out_port = subModel->get_output(cgra_port_num);
                //Get the input pdgnode corresponding to m
                SbPDG_Node* sbpdg_output = vec_out->getOutput(m);
                if(sched->pdgNodeOf(cgra_out_port) != NULL) {
                  ports_okay_to_use=false;
                  break;
                } 
                std::pair<int,int> fscore = make_pair(0, MAX_ROUTE); /*BUG: 0 should change to MAX_ROUTE for MLG, a better way to fix this is to define fscore for each subclass of HeursisticScheduler*/
                std::pair<int,int> curScore = scheduleHere(sched, sbpdg_output, cgra_out_port, 0, candRouting, fscore); 
                if(curScore>=fscore) { //?????
                  ports_okay_to_use=false;
                  break;
                }
                progress_incCurNum(Output);
              }
              progress_saveBestNum(Output);
              if(!ports_okay_to_use) {
                cout << "skipping this port assignment\n";
                continue; //don't assign these ports
              }
              cout<<"Succeeded!"<<endl;
              applyRouting(sched, 0/*config*/, &candRouting); //Commit the routing
              // Assign Individual Elements
              for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
                mask[m]=true;

                //Get the sbnode corresponding to mask[m]
                int cgra_port_num = vport_desc[m].first;
                sboutput* cgra_out_port = subModel->get_output(cgra_port_num);

                //Get the input pdgnode corresponding to m
                SbPDG_Node* sbpdg_output = vec_out->getOutput(m);
                sched->assign_node(sbpdg_output,cgra_out_port,0/*config*/);
              }
              //Perform the vector assignment
              sched->assign_vport(vec_out,vport_id,mask);
              found_vector_port=true;
      }
      if (found_vector_port) {
        break;
      }
    }
    if(!found_vector_port) {

      progress_saveBestNum(Output);  
      //cout << "Could not find output hardware vector port\n";
      return false;
    }
    //progress_incCurNum(Output);
  }

  progress_saveBestNum(Output);  
  return true;
}

void HeuristicScheduler::applyRouting(Schedule* sched, int config,
                             CandidateRouting* candRouting) {

  std::map< std::pair<SB_CONFIG::sblink*,int>,SbPDG_Edge* >::iterator I,E;
  for(I= candRouting->routing.begin(), E=candRouting->routing.end();I!=E;++I) {
    
    sched->assign_link(I->second->def(),I->first.first, I->first.second);
    sched->updateLinkCount(I->first.first);
  }
  //TODO: Apply forwarding

}

void HeuristicScheduler::applyRouting(Schedule* sched, SbPDG_Node* pdgnode,
                             sbnode* here, int config, CandidateRouting* candRouting){
  
  //cout << "pdgnode: " << pdgnode->name()  << " sbnode: " << here->name() 
  //<< " nlinks: " << candRouting->routing.size() << "\n";  
  sched->assign_node(pdgnode,here,config);
  applyRouting(sched,config,candRouting);
}

void HeuristicScheduler::fillInputSpots(Schedule* sched,SbPDG_Input* pdginst,
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

void HeuristicScheduler::fillOutputSpots(Schedule* sched,SbPDG_Output* pdginst,
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

void HeuristicScheduler::fillInstSpots(Schedule* sched,SbPDG_Inst* pdginst,
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

pair<int,int> HeuristicScheduler::route_minimizeDistance(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest, int config, CandidateRouting& candRouting, pair<int,int> scoreLeft) {
  
  pair<int,int> fscore = make_pair(MAX_ROUTE,MAX_ROUTE);
  list<sbnode*> openset;
  unordered_map<sbnode*,int> node_dist;
  unordered_map<sbnode*,sblink*> came_from;
  bool found_dest = false;
  
  openset.push_back(source);
  
  SbPDG_Node* pdgnode = pdgedge->def();
  
  //fill up worklist with nodes reachable 
  Schedule::link_iterator I,E;
  for(I=sched->links_begin(pdgnode),E=sched->links_end(pdgnode);I!=E;++I) {
    if(I->second==config) {
      sbnode* dest = I->first->dest();
      if(!dynamic_cast<sbfu*>(dest)) {//don't route through fu
        openset.push_back(dest);
      }
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
  if(!found_dest) return fscore;  //routing failed, no routes exist!
  
  pair<int,int> score;
  score = make_pair(0,node_dist[dest]);
  
  sbnode* x = dest;
  while(came_from.count(x)!=0) {
    sblink* link = came_from[x];
    candRouting.routing[make_pair(link,config)]=pdgedge;
    x=link->orig();
  }
  
  return score;
}

//routes only inside a configuration
pair<int,int> HeuristicScheduler::route_minimizeOverlapping(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest,
                     int config, CandidateRouting& candRouting, pair<int,int> scoreLeft) {
  
  pair<int,int> fscore = make_pair(MAX_ROUTE,MAX_ROUTE);
  list<sbnode*> openset;
  map<sbnode*,int> node_dist;
  map<sbnode*,int> node_overlap;
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
      //if(sched_exist_pdg!=NULL && sched_exist_pdg!=pdgnode) continue;
      
      std::pair<sblink*,int> p = make_pair(link,config);
      SbPDG_Node* cand_exist_pdg = candRouting.routing.count(p)==0 ? NULL :
                                  candRouting.routing[p]->def();
      //if(cand_exist_pdg!=NULL && cand_exist_pdg!=pdgnode) continue;
      
      sbnode* next = link->dest();
      
   //   if(next==_sbModel->subModel()->cross_switch()) continue;
    //  if(next==_sbModel->subModel()->load_slice()  ) continue;
      if(node_dist.count(next)!=0) continue; //it has been looked at, or will be looked at
      
      found_dest=(next==dest);

      if(dynamic_cast<sbfu*>(next) && !found_dest) continue;  //don't route through fu

      came_from[next] = link;
      node_dist[next] = node_dist[node]+1;
      if ((sched_exist_pdg != NULL && sched_exist_pdg != pdgnode) 
        || (cand_exist_pdg != NULL && cand_exist_pdg != pdgnode)) {
        node_overlap[next]=node_overlap[node]+1;
      }
 
      if(found_dest) break; 
      
      openset.push_back(next);
    }
    if(found_dest) break;
  }
    
  if(!found_dest) return fscore;  //routing failed, no routes exist!
  
  int distScore;
  distScore = node_dist[dest];

  int overlapScore;
  overlapScore = node_overlap[dest];
 
  std::pair<int,int> score = make_pair(overlapScore,distScore);
   
  sbnode* x = dest;
  while(came_from.count(x)!=0) {
    sblink* link = came_from[x];
    candRouting.routing[make_pair(link,config)]=pdgedge;
    x=link->orig();
  }
  
  return score;
}

//routes only inside a configuration
int HeuristicScheduler::route_to_output(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source,
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


