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
                sched->assign_node(sbpdg_input,cgra_in_port);
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
                std::pair<int,int> curScore = scheduleHere(sched, sbpdg_output, cgra_out_port, candRouting, fscore); 
                if(curScore>=fscore) { //?????
                  ports_okay_to_use=false;
                  break;
                }
                progress_incCurNum(Output);
              }
              progress_saveBestNum(Output);
              if(!ports_okay_to_use) {
                //cout << "skipping this port assignment\n";
                continue; //don't assign these ports
              }
              applyRouting(sched, &candRouting); //Commit the routing
              // Assign Individual Elements
              for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
                mask[m]=true;

                //Get the sbnode corresponding to mask[m]
                int cgra_port_num = vport_desc[m].first;
                sboutput* cgra_out_port = subModel->get_output(cgra_port_num);

                //Get the input pdgnode corresponding to m
                SbPDG_Node* sbpdg_output = vec_out->getOutput(m);
                sched->assign_node(sbpdg_output,cgra_out_port);
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

void HeuristicScheduler::applyRouting(Schedule* sched, 
                             CandidateRouting* candRouting) {

  std::unordered_map<SB_CONFIG::sblink*,SbPDG_Edge* >::iterator I,E;
  for(I= candRouting->routing.begin(), E=candRouting->routing.end();I!=E;++I) {
    sblink* link = I->first;
    sbnode* dest = link->dest();
    if(sbfu* fu = dynamic_cast<sbfu*>(dest)) {
      SbPDG_Node* dest_pdgnode = sched->pdgNodeOf(fu);
      if(!dest_pdgnode) {
        sched->add_passthrough_node(dest);
        //sched->assign_edge_pt(I->second,dest);
      }
    }
    //cout<<"pdgnode: "<< I->second->def()->name()<<" sblink: "<<link->name()<<endl;  
    sched->assign_link(I->second->def(),I->first);
    sched->updateLinkCount(I->first);
  }
  //TODO: Apply forwarding

}

void HeuristicScheduler::applyRouting(Schedule* sched, SbPDG_Node* pdgnode,
                             sbnode* here, CandidateRouting* candRouting){
  
  //cout << "pdgnode: " << pdgnode->name()  << " sbnode: " << here->name() 
  //<< " nlinks: " << candRouting->routing.size() << "\n";  
  assert(pdgnode);

  int min_node_lat, max_node_lat;
  candRouting->fill_lat(pdgnode,sched,min_node_lat,max_node_lat);

  if(SbPDG_Inst* inst = dynamic_cast<SbPDG_Inst*>(pdgnode)) {
     int i = inst_lat(inst->inst());
     min_node_lat += i;
     max_node_lat += i;
  }

  if(min_node_lat > max_node_lat) {
    sched->add_violation(min_node_lat-max_node_lat);
    max_node_lat = min_node_lat;
  }

  sched->assign_lat_bounds(pdgnode,min_node_lat,max_node_lat);

  sched->assign_node(pdgnode,here);
  applyRouting(sched,candRouting);
}

void HeuristicScheduler::fillInputSpots(Schedule* sched,SbPDG_Input* pdginst,
                              vector<sbnode*>& spots) {
  spots.clear();
  
  SubModel::const_input_iterator I,E;
  for(I=_sbModel->subModel()->input_begin(),
      E=_sbModel->subModel()->input_end(); I!=E; ++I) {
     sbinput* cand_input = const_cast<sbinput*>(&(*I));
    
    if(sched->pdgNodeOf(cand_input)==NULL) {
       spots.push_back(cand_input);
    }
  }
}

void HeuristicScheduler::fillOutputSpots(Schedule* sched,SbPDG_Output* pdginst,
                              vector<sbnode*>& spots) {
  spots.clear();
  
  SubModel::const_output_iterator I,E;
  for(I=_sbModel->subModel()->output_begin(),
      E=_sbModel->subModel()->output_end(); I!=E; ++I) {
     sboutput* cand_output = const_cast<sboutput*>(&(*I));
    
    if(sched->pdgNodeOf(cand_output)==NULL) {
       spots.push_back(cand_output);
    }

  }
}

void HeuristicScheduler::fillInstSpots(Schedule* sched,SbPDG_Inst* pdginst,
                              vector<sbnode*>& spots) {
  spots.clear();
  
  for(int i = 2; i < _sbModel->subModel()->sizex(); ++i) {
    sbfu* cand_fu = _sbModel->subModel()->fuAt(i,0);
    
    if((cand_fu->fu_def()==NULL || cand_fu->fu_def()->is_cap(pdginst->inst()))
       && sched->pdgNodeOf(cand_fu)==NULL) {
       spots.push_back(cand_fu);
    }
  }
  
  for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
      sbfu* cand_fu = _sbModel->subModel()->fuAt(i,j);
      
      if((cand_fu->fu_def()==NULL||cand_fu->fu_def()->is_cap(pdginst->inst()))
         && sched->pdgNodeOf(cand_fu)==NULL) {
         spots.push_back(cand_fu);
      }
    }
  }
}

//I was too lazy to implement dijkstra's algorithm (also i don't like priority queues),
//so instead i made one fast list (openset) and one slow list (openset_long)
//Why, you ask is dijkstra's necesary -- well, we want to penalize passthrough nodes.
pair<int,int> HeuristicScheduler::route_minimizeDistance(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest, CandidateRouting& candRouting, pair<int,int> scoreLeft) {
  list<sbnode*> openset;
  list<sbnode*> openset_long;

  unordered_map<sbnode*,int> node_dist;
  unordered_map<sbnode*,sblink*> came_from;
  unordered_map<sbnode*,sblink*> prior_came_from;
  bool found_dest = false;
  
  openset.push_back(source);
  
  SbPDG_Node* pdgnode = pdgedge->def();
  
  //fill up worklist with nodes reachable 
  Schedule::link_iterator I,E;
  for(I=sched->links_begin(pdgnode),E=sched->links_end(pdgnode);I!=E;++I) {
    sblink* link = *I;
    sbnode* dest = link->dest();
    sbfu* fu = dynamic_cast<sbfu*>(dest);
    if(!fu || sched->pdgNodeOf(fu) == NULL) { // don't start at some other instruction's fu
      node_dist[dest]=0;
      openset.push_back(dest);
      prior_came_from[dest]=link;
    }
  }

  while(!openset.empty() || !openset_long.empty()) {
    if(openset.empty()) {
      openset = openset_long;
      openset_long.clear();
    }

    sbnode* node = openset.front();
    openset.pop_front();

    int cur_dist = node_dist[node];
    if(cur_dist >= scoreLeft.second) {
      continue;
    }
    
    for(auto I = node->obegin(), E = node->oend(); I!=E; ++I) {
      sblink* link = *I;
      
      //check if connection is closed..
      SbPDG_Node* sched_exist_pdg = sched->pdgNodeOf(link);
      if(sched_exist_pdg!=NULL && sched_exist_pdg!=pdgnode) continue;
      
      sblink* p = link;
      SbPDG_Node* cand_exist_pdg = candRouting.routing.count(p)==0 ? NULL :
                                  candRouting.routing[p]->def();
      if(cand_exist_pdg!=NULL && cand_exist_pdg!=pdgnode) continue;
     
      sbnode* next = link->dest();
      
      if(node_dist.count(next)!=0) continue; //has or will be looked at

      found_dest=(next==dest);
      bool passthrough = (dynamic_cast<sbfu*>(next) && !found_dest);
      int new_dist = cur_dist+1+passthrough*1000;

      came_from[next] = link;
      node_dist[next] = new_dist;

      if(found_dest) break; 
      
//      if(passthrough&&((((uint64_t)next)&0x70)!=0))  dumb idea
      if(passthrough) {
        openset_long.push_back(next);
      } else {
        openset.push_back(next);
      }

    }
    if(found_dest) break;
  } 
  if(!found_dest) return fscore;  //routing failed, no routes exist!
  
  pair<int,int> score;
  score = make_pair(0,node_dist[dest]);
  
  sbnode* x = dest;
  while(came_from.count(x)!=0) {
    sblink* link = came_from[x];
    candRouting.routing[link]=pdgedge;
    x=link->orig();
  }

  int count = 0; 
  while(prior_came_from.count(x)!=0) {
    sblink* link = prior_came_from[x];
    count++;
    x=link->orig();
  }

  int tot_lat = node_dist[dest] + count -1;
  int pts=0;
  while(tot_lat > 1000) {
    tot_lat -= 1000;
    pts++;
  }

  candRouting.edge_prop[pdgedge]=make_pair(tot_lat,pts);
  return score;
}

//routes only inside a configuration
pair<int,int> HeuristicScheduler::route_minimizeOverlapping(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest,
                     CandidateRouting& candRouting, pair<int,int> scoreLeft) {
  
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
    openset.push_back((*I)->dest());
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
      SbPDG_Node* sched_exist_pdg = sched->pdgNodeOf(link);
      //if(sched_exist_pdg!=NULL && sched_exist_pdg!=pdgnode) continue;
      
      sblink* p = link;
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
    candRouting.routing[link]=pdgedge;
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


