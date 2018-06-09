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


void System(const char* command) {
  int ret = system(command);
  if(ret) {
    std::cout << "Command: \"" << command 
              << "\" failed with return value: " << ret << "\n";
  }
}


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

  for(auto I = candRouting->routing.begin(), 
           E = candRouting->routing.end(); I!=E; ++I) {
    sblink* link = I->first;
    sbnode* dest = link->dest();
    if(sbfu* fu = dynamic_cast<sbfu*>(dest)) {
      SbPDG_Node* dest_pdgnode = sched->pdgNodeOf(fu);
      if(!dest_pdgnode) {
        //sched->add_passthrough_node(dest);
        sched->assign_edge_pt(I->second,dest);
      }
    }
    //cout<<"pdgnode: "<< I->second->def()->name()<<" sblink: "<<link->name()<<endl;  
    sched->assign_edgelink(I->second,I->first);
  }

  for(auto I : candRouting->edge_prop) {
    SbPDG_Edge* edge = I.first;
    int links = I.second.num_links;
    sched->set_num_links(links,edge);  
  } 
}

void HeuristicScheduler::applyRouting(Schedule* sched, SbPDG_Node* pdgnode,
                             sbnode* here, CandidateRouting* candRouting){
  
  //cout << "pdgnode: " << pdgnode->name()  << " sbnode: " << here->name() 
  //<< " nlinks: " << candRouting->routing.size() << "\n";  
  assert(pdgnode);

  int min_node_lat, max_node_lat;
  candRouting->fill_lat(sched,min_node_lat,max_node_lat);

  if(SbPDG_Inst* inst = dynamic_cast<SbPDG_Inst*>(pdgnode)) {
     int i = inst_lat(inst->inst());
     min_node_lat += i;
     max_node_lat += i;
  }

  //cout << pdgnode->name() << " " << min_node_lat << " " << max_node_lat << "\n";

  if(min_node_lat > max_node_lat) {
    sched->add_violation(min_node_lat-max_node_lat);
    sched->record_violation(pdgnode,min_node_lat-max_node_lat);
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
  
  for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
      sbfu* cand_fu = _sbModel->subModel()->fuAt(i,j);
      
      if((cand_fu->fu_def()==NULL||cand_fu->fu_def()->is_cap(pdginst->inst()))
       && !sched->pdgNodeOf(cand_fu) && !sched->isPassthrough(cand_fu)) {
         spots.push_back(cand_fu);
      }
    }
  }
}

void Scheduler::unroute(Schedule* sched, SbPDG_Edge* pdgedge, 
                        SB_CONFIG::sbnode* source) {
 
  std::queue<sbnode*> openset;
  openset.push(source);

  _sbModel->subModel()->clear_all_runtime_vals();
  
  SbPDG_Node* pdgnode = pdgedge->def();

  while(!openset.empty()) {
    sbnode* node = openset.front();
    openset.pop();
    node->set_done(1);

    for(auto I = node->obegin(), E = node->oend(); I!=E; ++I) {
      sblink* link = *I;
      sbnode* next = link->dest();
   
      if(sched->pdgNodeOf(link) == pdgnode) {
        sched->unassign_link(pdgnode,link);
        if(next->done()!=1) {
          openset.push(next);
        }
      } 
    }
  }
}

struct mycomparison {
  bool operator() (std::pair<sbnode*,int> lhs, std::pair<sbnode*,int> rhs) const {
    return lhs.second > rhs.second;
  }
};

//Dijkstra's Algorithm
pair<int,int> HeuristicScheduler::route_minimizeDistance(Schedule* sched, SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest, CandidateRouting& candRouting, pair<int,int> scoreLeft) {
  priority_queue<std::pair<sbnode*,int>,vector<std::pair<sbnode*,int>>,mycomparison> openset;

  _sbModel->subModel()->clear_all_runtime_vals();

  source->set_node_dist(0);
  openset.push(make_pair(source,0));
  
  SbPDG_Node* pdgnode = pdgedge->def();

  //cout << "*** ROUTING Problem; source: " << source->name() << " dest: " << dest->name() 
  //     << " pdgedge: " << pdgedge->name() << " *** \n";

  while(!openset.empty()) {
    sbnode* node = openset.top().first;
    //cout << "pop node: " << node->name() << " dist: " << node->node_dist() << " " << openset.top().second << "\n";
    openset.pop();
    if(node->done()) continue;

    node->set_done(1);

    if(node == dest) break;
    
    int cur_dist = node->node_dist();
    if(cur_dist >= scoreLeft.first) continue;

    for(auto I = node->obegin(), E = node->oend(); I!=E; ++I) {
      sblink* link = *I;
 
      sbnode* next = link->dest();
    
      //check if next is done or connection is closed..
      if(next->done()) continue;

      //check if connection is closed..
      SbPDG_Node* sched_node = sched->pdgNodeOf(link);
      if(sched_node!=NULL && sched_node!=pdgnode) continue;
      
      SbPDG_Node* cand_node = candRouting.routing.count(link)==0 ? NULL :
                                  candRouting.routing[link]->def();
      if(cand_node!=NULL && cand_node!=pdgnode) continue;

      int new_dist=0;
      bool is_dest=(next==dest);

      sbfu* fu = dynamic_cast<sbfu*>(next);
      if(fu && sched->pdgNodeOf(fu) && !is_dest) continue;  //stop if run into fu

      if(sched_node!=pdgnode && cand_node!=pdgnode) { //if not free link

        if(fu && sched->isPassthrough(fu)) continue; //someone else's pass through
    
        bool passthrough = (fu && !is_dest);
        new_dist =  cur_dist+1+passthrough*1000;
      } else {
        assert(cur_dist==0);
      }
     
      int next_dist = next->node_dist();
      if(next_dist==-1 || new_dist < next_dist) {
         next->set_came_from(link);
         next->set_node_dist(new_dist);
         openset.emplace(next,new_dist);
         //cout << "push node at " << link->name() << " with " << new_dist << "\n";
      }
    }
  }
  if(!dest->done()) return fscore;  //routing failed, no routes exist!
  
  pair<int,int> score;
  score = make_pair(0,dest->node_dist());
 
  sbnode* x = dest;
  while(x->node_dist()!=0) {
    sblink* link = x->came_from();
    candRouting.routing[link]=pdgedge;
    x=link->orig();
  }

  int count = 0;  //count how many are already routed
  while(x != source) {
    sblink* link = x->came_from();
    count++;
    candRouting.routing[link]=pdgedge;
    x=link->orig();
  }

  int tot_lat = dest->node_dist() + count -1;
  int pts=0;
  while(tot_lat > 1000) {
    tot_lat -= 1000;
    pts++;
  }

  auto& prop = candRouting.edge_prop[pdgedge];
  prop.num_links=tot_lat;
  prop.num_passthroughs=pts;
  return score;
}

bool Scheduler::check_res(SbPDG* sbPDG, SbModel* sbmodel) {
  int ninsts = sbPDG->inst_end() - sbPDG->inst_begin();

  int nfus = sbmodel->subModel()->sizex() * sbmodel->subModel()->sizey();

  if(ninsts > nfus) {
    cerr << "\n\nError: Too many instructions in SbPDG for given SBCONFIG\n\n";
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


