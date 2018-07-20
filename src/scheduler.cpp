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

//floyd's unique-number sampling algorithm (ordered)
//CACM Programming Pearls, 1987
void HeuristicScheduler::rand_n_choose_k(int n, int k, std::vector<int>& indices) {
  std::vector<bool> mask; 
  mask.resize(n);
  indices.clear();
  
  for(int j = n-k; j <n; ++j) {
    int t = rand_bt(0,j+1);
    if(mask[t]==false) {
      mask[t]=true;
    } else {
      mask[j]=true;
    }
  }
  for(int i = 0; i < n; ++i) {
    if(mask[i]) {
      indices.push_back(i);
    }
  }
}

void HeuristicScheduler::random_order(int n, std::vector<int>& order) {
  order.clear();
  for (int i=0; i<n; ++i) order.push_back(i); 
  std::random_shuffle ( order.begin(), order.end() ); 
}


//could make this a template really!
vector<bool> HeuristicScheduler::rand_node_choose_k(int k, 
    std::vector<sbnode*>& poss_nodes, std::vector<sbnode*>& chosen_nodes) {
  std::vector<bool> mask; 
  int n = poss_nodes.size();
  mask.resize(n);
  chosen_nodes.clear();
  
  for(int j = n-k; j <n; ++j) {
    int t = rand_bt(0,j+1);
    if(mask[t]==false) {
      mask[t]=true;
    } else {
      mask[j]=true;
    }
  }

  for(int i = 0; i < n; ++i) {
    if(mask[i]) {
      chosen_nodes.push_back(poss_nodes[i]);
    }
  }
  return mask;
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

    for(auto& i : sd) {
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
              }
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
      //cout << "Could not find Input hardware vector port\n";
      return false;
    }
  }
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

    for(auto& i : sd) {
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
              }
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
      //cout << "Could not find output hardware vector port\n";
      return false;
    }
  }
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
        for(auto& e : I->second) { 
          sched->assign_edge_pt(e,dest);
        }
      }
    }
    //cout<<"pdgnode: "<< I->second->def()->name()<<" sblink: "<<link->name()<<endl; 
    for(auto& e : I->second)  {
      sched->assign_edgelink(e,I->first);
    }
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
 
  //For Dedicated-required Instructions
  for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
    for(int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
      sbfu* cand_fu = _sbModel->subModel()->fuAt(i,j);

      if(!(cand_fu->fu_def()==NULL) &&
        !cand_fu->fu_def()->is_cap(pdginst->inst())) {
        continue;
      }
      
      if(!pdginst->is_temporal()) {
        //Normal Dedidated Instructions
        if(!sched->pdgNodeOf(cand_fu) && !sched->isPassthrough(cand_fu)) {
           spots.push_back(cand_fu);
        }
      } else {
        //For temporaly-shared instructions
        //For now the approach is to *not* consume dedicated resources, although
        //this can be changed later if that's helpful.
        if(cand_fu->max_util() > 1) {
          if(sched->thingsAssigned(cand_fu) +1 < cand_fu->max_util()) {
            spots.push_back(cand_fu);
          } 
        }
      }
    }
  }
  if(pdginst->is_temporal() && spots.size()==0) {
    cout << "Warning, no spots for" << pdginst->name() << "\n";
  }
}

//void Scheduler::unroute(Schedule* sched, SbPDG_Edge* pdgedge, 
//                        SB_CONFIG::sbnode* source) {
// 
//  std::queue<sbnode*> openset;
//  openset.push(source);
//
//  _sbModel->subModel()->clear_all_runtime_vals();
//  
//  SbPDG_Node* pdgnode = pdgedge->def();
//
//  while(!openset.empty()) {
//    sbnode* node = openset.front();
//    openset.pop();
//    node->set_done(1);
//
//    for(auto I = node->obegin(), E = node->oend(); I!=E; ++I) {
//      sblink* link = *I;
//      sbnode* next = link->dest();
//   
//      if(sched->pdgNodeOf(link) == pdgnode) {
//        sched->unassign_link(pdgnode,link);
//        if(next->done()!=1) {
//          openset.push(next);
//        }
//      } 
//    }
//  }
//}

struct mycomparison {
  bool operator() (std::pair<sbnode*,int> lhs, std::pair<sbnode*,int> rhs) const {
    return lhs.second > rhs.second;
  }
};

int HeuristicScheduler::routing_cost(SbPDG_Edge* edge, sblink* link, 
      Schedule* sched, CandidateRouting& candRouting, sbnode* dest) {

    SbPDG_Node* pdgnode = edge->def();
    sbnode* next = link->dest();

    //check if connection is closed..
    SbPDG_Node* sched_node = sched->pdgNodeOf(link);
    if(sched_node!=NULL && sched_node!=pdgnode) return -1;
    
    SbPDG_Node* cand_node = candRouting.routing.count(link)==0 ? NULL :
                            (*candRouting.routing[link].begin())->def();
    if(cand_node!=NULL && cand_node!=pdgnode) return -1;

    bool is_dest=(next==dest);

    sbfu* fu = dynamic_cast<sbfu*>(next);
    if(fu && sched->pdgNodeOf(fu) && !is_dest) return -1;  //stop if run into fu

    if(sched_node==pdgnode || cand_node==pdgnode) { 
      return 0; //free link because schedule or candidate routing already maps
    } else {
      if(fu && sched->isPassthrough(fu)) return -1; //someone else's pass through
    
      bool passthrough = (fu && !is_dest);
      return 1+passthrough*1000;
    }
}

//Dijkstra's Algorithm
pair<int,int> HeuristicScheduler::route_minimizeDistance(Schedule* sched, 
    SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest, 
    CandidateRouting& candRouting, pair<int,int> scoreLeft) {

  _route_times++;

  if(sched->link_count(pdgedge)!=0) {
     cerr << "Edge: " << pdgedge->name() << " is already routed!\n"; 
     assert(0);
  }
  
  priority_queue<std::pair<sbnode*,int>, 
                 vector<std::pair<sbnode*,int>>, mycomparison> openset;

  _sbModel->subModel()->clear_all_runtime_vals();

  source->set_node_dist(0);
  openset.push(make_pair(source,0));
  
  //cout << "*** ROUTING Problem; source: " 
  //     << source->name() << " dest: " << dest->name() 
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
      if(next->done()) continue;

      int route_cost = routing_cost(pdgedge,link,sched,candRouting,dest);
      if(route_cost ==-1) continue; // didn't work
       
      int new_dist = cur_dist + route_cost;

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

  int num_links=0, pts=0;

  x = dest;
  while(x->node_dist()!=0) {
    sblink* link = x->came_from();
    candRouting.routing[link].insert(pdgedge);
    x=link->orig();
    sbnode* next = link->dest();
    if(dynamic_cast<sbfu*>(next) && num_links!=0) {
      pts++;
    }
    num_links++;
  }

  while(x != source) {
    sblink* link = x->came_from();
    num_links++;
    candRouting.routing[link].insert(pdgedge);
    x=link->orig();
  }
  num_links--;

  auto& prop = candRouting.edge_prop[pdgedge];
  prop.num_links=num_links;
  prop.num_passthroughs=pts;
  return score;
}

bool Scheduler::check_res(SbPDG* sbPDG, SbModel* sbmodel) {
  int dedicated_insts=0;
  int temporal_insts=0;
  for(auto I = sbPDG->inst_begin(), E=sbPDG->inst_end(); I!=E; ++I) {
    SbPDG_Inst* i = *I;
    if(!i->is_temporal()) {
      dedicated_insts++;
    } else {
      temporal_insts++;
    }
  }

  int temporal_fus=0;
  int temporal_inst_slots=0;
  auto fu_list = _sbModel->subModel()->fu_list();
  for(sbfu* inst : fu_list) {
    if(inst->max_util() > 1) {
      temporal_inst_slots+=inst->max_util();
      temporal_fus += 1;
    }
  }

  int nfus = sbmodel->subModel()->sizex() * sbmodel->subModel()->sizey();

  if(dedicated_insts > nfus) {
    cerr << "\n\nError: Too many dedicated instructions ("
         << dedicated_insts << ") in SbPDG for given SBCONFIG (has " 
         << nfus << " fus)\n\n";
    exit(1);
  }
  if(temporal_insts > temporal_inst_slots) {
     cerr << "\n\nError: Too many temporal instructions ("
         << temporal_insts << ") in SbPDG for given SBCONFIG (has " 
         << temporal_inst_slots << " temporal slots)\n\n";
    exit(1);
  }

  bool failed_count_check=false;

  std::map<sb_inst_t,int> dedicated_count_types, temporal_count_types;
  for(auto Ii=sbPDG->inst_begin(), Ei=sbPDG->inst_end(); Ii!=Ei; ++Ii) {
    if(!(*Ii)->is_temporal()) {
      dedicated_count_types[(*Ii)->inst()]++;
    } else {
      temporal_count_types[(*Ii)->inst()]++;
    }
  }

  for(auto& pair : dedicated_count_types) {
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
           << " dedicated insts, but only " << fu_count 
           << " dedicated fus to support them\n";
    }
  }

  for(auto& pair : temporal_count_types) {
    sb_inst_t sb_inst = pair.first;
    int pdg_count = pair.second;

    int fu_count =0;
    for(int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
      for(int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
        sbfu* cand_fu = _sbModel->subModel()->fuAt(i,j);
        if(cand_fu->max_util() > 1 && cand_fu->fu_def()->is_cap(sb_inst)) {
          fu_count+=cand_fu->max_util();
        }
      }
    }
    if(fu_count < pdg_count) {
      failed_count_check=true;
      cerr << "Error: PDG has " << pdg_count << " " << name_of_inst(sb_inst) 
           << " temporal insts, but only " 
           << fu_count << " temporal fu slots to support them\n";
    }
  }



  if(failed_count_check) {
    cerr << "\n\nError: FAILED Basic FU Count Check\n\n";
    return false;
  }
  //TODO: add code from printPortcompatibility here
  
  return true;
}


