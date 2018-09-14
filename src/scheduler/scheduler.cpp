#include "scheduler.h"

using namespace SB_CONFIG;
using namespace std;

#include <unordered_map>
#include <fstream>
#include <sstream>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
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
  SB_CONFIG::SubModel *subModel = _sbModel->subModel();
  sbio_interface si = subModel->io_interf();
  vector<pair<int, int>> sd;

  int n = sbPDG->num_vec_input();

  sbPDG->sort_vec_in();
  si.sort_in_vports(sd);

  for (int j = 0; j < n; ++j) {
    SbPDG_VecInput *vec_in = sbPDG->vec_in(j);
    //cout << "Assigning Vector Port:" << vec_in->gamsName() <<"\n";

    bool found_vector_port = false;

    for (auto &i : sd) {
      int vport_num = i.first;
      auto vport_id = std::make_pair(true/*input*/, vport_num);
      const vector<int> &vport_desc = si.getDesc_I(vport_num)->port_vec();

      //Check if the vetcor port is 1. big enough & 2. unassigned
      if (vec_in->inputs().size() <= vport_desc.size() && sched->vportOf(vport_id) == nullptr) {
        std::vector<bool> mask;
        mask.resize(vport_desc.size());

        bool ports_okay_to_use = true;

        //Check if it's okay to assign to these ports
        for (unsigned m = 0; m < vec_in->inputs().size(); ++m) {
          //Get the sbnode corresponding to mask[m]
          int cgra_port_num = vport_desc[m];
          sbinput *cgra_in_port = subModel->inputs()[cgra_port_num];

          if (sched->pdgNodeOf(cgra_in_port) != nullptr) {
            ports_okay_to_use = false;
            break;
          }
        }
        if (!ports_okay_to_use) {
          //cout << "skipping this port assignment\n";
          continue; //don't assign these ports
        }
        // Assign Individual Elements
        for (unsigned m = 0; m < vec_in->inputs().size(); ++m) {
          mask[m] = true;

          //Get the sbnode corresponding to mask[m]
          int cgra_port_num = vport_desc[m];
          sbinput *cgra_in_port = subModel->inputs()[cgra_port_num];

          //Get the input pdgnode corresponding to m
          SbPDG_Node *sbpdg_input = vec_in->inputs()[m];
          sched->assign_node(sbpdg_input, make_pair(0, cgra_in_port));
        }
        //Perform the vector assignment
        sched->assign_vport(vec_in, vport_id, mask);
        found_vector_port = true;
      }
      if (found_vector_port) {
        break;
      }
    }
    if (!found_vector_port) {
      //cout << "Could not find Input hardware vector port\n";
      return false;
    }
  }
  return true;
}


bool HeuristicScheduler::assignVectorOutputs(SbPDG* sbPDG, Schedule* sched) {
  SB_CONFIG::SubModel *subModel = _sbModel->subModel();
  CandidateRouting candRouting;
  sbio_interface si = subModel->io_interf();
  vector<pair<int, int>> sd;

  int n = sbPDG->num_vec_output();


  sbPDG->sort_vec_out();
  si.sort_out_vports(sd);

  for (int j = 0; j < n; ++j) {
    SbPDG_VecOutput *vec_out = sbPDG->vec_out(j);
    //cout << "Assigning Vector Port:" << vec_out->gamsName() <<"\n";

    bool found_vector_port = false;

    for (auto &i : sd) {
      int vport_num = i.first;
      auto vport_id = std::make_pair(true/*input*/, vport_num);
      const vector<int> &vport_desc = si.getDesc_I(vport_num)->port_vec();

      //Check if the vetcor port is 1. big enough & 2. unassigned
      if (vec_out->outputs().size() <= vport_desc.size() && sched->vportOf(vport_id) == nullptr) {
        std::vector<bool> mask;
        mask.resize(vport_desc.size());

        bool ports_okay_to_use = true;
        candRouting.clear();
        //Check if it's okay to assign to these ports
        for (unsigned m = 0; m < vec_out->outputs().size(); ++m) {
          //Get the sbnode corresponding to mask[m]
          int cgra_port_num = vport_desc[m];
          sboutput *cgra_out_port = subModel->outputs()[cgra_port_num];
          //Get the input pdgnode corresponding to m
          SbPDG_Node *sbpdg_output = vec_out->outputs()[m];
          if (sched->pdgNodeOf(cgra_out_port) != nullptr) {
            ports_okay_to_use = false;
            break;
          }
          std::pair<int, int> curScore = scheduleHere(sched, sbpdg_output, make_pair(0, cgra_out_port), candRouting);
          if (curScore >= fscore) { //?????
            ports_okay_to_use = false;
            break;
          }
        }
        if (!ports_okay_to_use) {
          //cout << "skipping this port assignment\n";
          continue; //don't assign these ports
        }
        apply_routing(sched, &candRouting); //Commit the routing
        // Assign Individual Elements
        for (unsigned m = 0; m < vec_out->outputs().size(); ++m) {
          mask[m] = true;

          //Get the sbnode corresponding to mask[m]
          int cgra_port_num = vport_desc[m];
          sboutput *cgra_out_port = subModel->outputs()[cgra_port_num];

          //Get the input pdgnode corresponding to m
          SbPDG_Node *sbpdg_output = vec_out->outputs()[m];
          sched->assign_node(sbpdg_output, make_pair(0, cgra_out_port));
        }
        //Perform the vector assignment
        sched->assign_vport(vec_out, vport_id, mask);
        found_vector_port = true;
      }
      if (found_vector_port) {
        break;
      }
    }
    if (!found_vector_port) {
      //cout << "Could not find output hardware vector port\n";
      return false;
    }
  }
  return true;
}

void HeuristicScheduler::apply_routing(Schedule *sched, CandidateRouting *candRouting) {

  for (auto elem : candRouting->routing) {
    sblink *link = elem.first.second;
    sbnode *dest = link->dest();
    if (sbfu *fu = dynamic_cast<sbfu *>(dest)) {
      SbPDG_Node *dest_pdgnode = sched->pdgNodeOf(fu);
      if (!dest_pdgnode) {
        //sched->add_passthrough_node(dest);
        for (auto &e : elem.second) {
          //FIXME: make sure it is correct
          sched->assign_edge_pt(e, make_pair(elem.first.first, dest));
        }
      }
    }
    //cout<<"pdgnode: "<< I->second->def()->name()<<" sblink: "<<link->name()<<endl; 
    for (auto &e : elem.second) {
      sched->assign_edgelink(e, elem.first.first, elem.first.second);
    }
  }
}

void HeuristicScheduler::apply_routing(Schedule *sched, SbPDG_Node *pdgnode,
                                       pair<int, sbnode *> here, CandidateRouting *candRouting){
  
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
  apply_routing(sched, candRouting);
}

vector<pair<int, sbnode*>> HeuristicScheduler::fill_input_spots(Schedule* sched, SbPDG_Input* pdginst) {
  vector<pair<int, sbnode*>> spots;

  SubModel::const_input_iterator I, E;
  for (auto &elem : _sbModel->subModel()->inputs()) {
    sbinput *cand_input = elem;

    if (sched->pdgNodeOf(cand_input) == nullptr) {
      spots.emplace_back(make_pair(0, cand_input));
    }
  }
  return spots;
}

vector<pair<int, sbnode*>> HeuristicScheduler::fill_output_spots(Schedule* sched,SbPDG_Output* pdginst) {
  vector<pair<int, sbnode*>> spots;

  SubModel::const_output_iterator I, E;
  for (auto &elem : _sbModel->subModel()->outputs()) {
    sboutput *cand_output = elem;

    if (sched->pdgNodeOf(cand_output) == nullptr) {
      spots.emplace_back(make_pair(0, cand_output));
    }

  }
  return spots;
}

vector<pair<int, sbnode*>> HeuristicScheduler::fill_inst_spots(Schedule *sched, SbPDG_Inst *pdginst) {
  vector<pair<int, sbnode*>> spots;

  //For Dedicated-required Instructions
  for (sbfu *cand_fu : _sbModel->subModel()->fu_list()) {
    if (cand_fu->fu_def() != nullptr && !cand_fu->fu_def()->is_cap(pdginst->inst())) {
      continue;
    }

    if (!pdginst->is_temporal()) {
      if (sched->isPassthrough(cand_fu))
        continue;
      //Normal Dedidated Instructions
      
      //integrate this later!
      //int util=sched->thingsAssigned(cand_fu);
      //if(util==0 || (rand_bt(0,3+util*util) ==0) ) {
      //   spots.push_back(cand_fu);
      auto status = sched->pdg_nodes_of(cand_fu);
      vector<bool> occupied(8, false);
      for (auto elem : status) {
        for (int k = 0; k < elem.second->bitwidth() / 8; ++k)
          occupied[k + elem.first] = true;
      }
      for (int k = 0; k < 8; k += pdginst->bitwidth() / 8) {
        auto begin = occupied.begin() + k;
        auto end = occupied.begin() + k + pdginst->bitwidth() / 8;
        int util = accumulate(begin, end, false, [](bool a, bool b) -> int { return (int) a + (int) b; });
        if (util == 0 || rand_bt(0, 3 + util * util) == 0) {
          spots.emplace_back(make_pair(k, cand_fu));
        }
      }

    } else {
      //For temporaly-shared instructions
      //For now the approach is to *not* consume dedicated resources, although
      //this can be changed later if that's helpful.
      if ((int)sched->thingsAssigned(cand_fu) + 1 < cand_fu->max_util()) {
        spots.emplace_back(make_pair(0, cand_fu));
      }
    }
    
    
  }

  if (pdginst->is_temporal() && spots.empty()) {
    cout << "Warning, no spots for" << pdginst->name() << "\n";
  }
  return spots;
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

int HeuristicScheduler::routing_cost(SbPDG_Edge* edge, int from_slot, int next_slot, sblink* link,
      Schedule* sched, CandidateRouting& candRouting, const pair<int, sbnode*> &dest) {

  SbPDG_Node *pdgnode = edge->def();
  sbnode *next = link->dest();

  //check if connection is closed..
  SbPDG_Node *sched_node = sched->pdgNodeOf(next_slot, link);
  if (sched_node != nullptr && sched_node != pdgnode)
    return -1;

  SbPDG_Node *cand_node = candRouting.routing.count(make_pair(next_slot, link)) == 0 ? nullptr :
                          (*candRouting.routing[make_pair(next_slot, link)].begin())->def();
  if (cand_node != nullptr && cand_node != pdgnode)
    return -1;

  bool is_dest = (next == dest.second);

  sbfu *fu = dynamic_cast<sbfu *>(next);
  if (fu && sched->pdgNodeOf(fu) && !is_dest) {
    return -1;  //stop if run into fu
  }

  if (sched_node == pdgnode || cand_node == pdgnode) {
    return 0; //free link because schedule or candidate routing already maps
  } else {
    if (fu && sched->isPassthrough(fu)) {
      return -1; //someone else's pass through
    }

    bool passthrough = (fu && !is_dest);
    return 1 + passthrough * 1000;
  }
}

//Dijkstra's Algorithm
pair<int,int>
HeuristicScheduler::route_minimize_distance(Schedule *sched, SbPDG_Edge *pdgedge,
        pair<int, sbnode*> source, pair<int, sbnode*> dest, CandidateRouting &candRouting) {

  int bitwidth = std::min(pdgedge->def()->bitwidth(), pdgedge->use()->bitwidth());

  _route_times++;

  if (sched->link_count(pdgedge) != 0) {
    cerr << "Edge: " << pdgedge->name() << " is already routed!\n";
    assert(0);
  }

  set<std::tuple<int, int, sbnode*>> openset;

  _sbModel->subModel()->clear_all_runtime_vals();

  source.second->update_dist(source.first, 0, 0, nullptr);
  openset.emplace(0, source.first, source.second);

  while (!openset.empty()) {
    int cur_dist = std::get<0>(*openset.begin());
    int slot = std::get<1>(*openset.begin());
    sbnode *node = std::get<2>(*openset.begin());

    //std::cout << "Finalize distance of " << node->name() << ", " << slot << ": " << cur_dist << std::endl;

    openset.erase(openset.begin());

    if (slot == dest.first && node == dest.second)
      break;

    for (auto link : node->out_links()) {
      sbnode *next = link->dest();

      for (int delta  = 0; delta <= 1; ++delta) {
        int next_slot = slot + delta * bitwidth / 8;
        next_slot = (next_slot + 8) % 8;
        int route_cost = routing_cost(pdgedge, slot, next_slot, link, sched, candRouting, dest);

        if (route_cost == -1)
          continue;

        int new_dist = cur_dist + route_cost;

        int next_dist = next->node_dist(next_slot);
        if (next_dist == -1 || next_dist > new_dist) {
          if (next_dist != -1) {
            auto iter = openset.find(std::make_tuple(next_dist, next_slot, next));
            assert(iter != openset.end());
            openset.erase(iter);
          }
          openset.emplace(new_dist, next_slot, next);
          next->update_dist(next_slot, new_dist, slot, link);
          //std::cout << "Update " << next->name() << ", " << next_slot << " dist to " << new_dist << std::endl;
        }
      }
    }
    //std::cout << std::endl;
  }

  if (dest.second->node_dist(dest.first) == -1)
    return fscore;  //routing failed, no routes exist!

  pair<int, int> score;
  score = make_pair(0, dest.second->node_dist(dest.first));
  pair<int, sbnode*> x = dest;

  int num_links = 0, pts = 0;

  vector<pair<int, sbnode*>> path;
  path.emplace_back(x);
  while (x != source) {
    pair<int, sblink*> link = x.second->came_from(x.first);
    candRouting.routing[link].insert(pdgedge);
    x = make_pair(link.first, link.second->orig());
    sbnode *next = link.second->dest();
    if (dynamic_cast<sbfu *>(next) && num_links != 0) {
      pts++;
    }
    num_links++;
    path.emplace_back(x);
  }

  /*std::cout << "*** " << pdgedge->name() << " Path ***\n";
  for (auto iter = path.rbegin(); iter != path.rend(); ++iter) {
    std::cout << iter->second->name() << ", " << iter->first << ": " << iter->second->node_dist(iter->first) << std::endl;
  }
  std::cout << "\n";*/

  /*while (x != source) {
    sblink *link = x->came_from();
    num_links++;
    candRouting.routing[link].insert(pdgedge);
    x = link->orig();
  }*/

  auto &prop = candRouting.edge_prop[pdgedge];
  prop.num_links = num_links - 1;
  prop.num_passthroughs = pts;
  return score;
}

bool Scheduler::check_res(SbPDG* sbPDG, SbModel* sbmodel) {
  int dedicated_insts = 0;
  int temporal_insts = 0;
  for (auto I = sbPDG->inst_begin(), E = sbPDG->inst_end(); I != E; ++I) {
    SbPDG_Inst *i = *I;
    if (!i->is_temporal()) {
      dedicated_insts++;
    } else {
      temporal_insts++;
    }
  }

  int temporal_fus = 0;
  int temporal_inst_slots = 0;
  for (auto elem: _sbModel->subModel()->fu_list()) {
    temporal_inst_slots += elem->max_util();
    temporal_fus += 1;
  }

  //int nfus = sbmodel->subModel()->sizex() * sbmodel->subModel()->sizey();
  //if (dedicated_insts > nfus) {
  //  cerr << "\n\nError: Too many dedicated instructions ("
  //       << dedicated_insts << ") in SbPDG for given SBCONFIG (has "
  //       << nfus << " fus)\n\n";
  //  return false;
  //}
  if (temporal_insts > temporal_inst_slots) {
    cerr << "\n\nError: Too many temporal instructions ("
         << temporal_insts << ") in SbPDG for given SBCONFIG (has "
         << temporal_inst_slots << " temporal slots)\n\n";
    return false;
  }

  bool failed_count_check = false;

  std::map<sb_inst_t, int> dedicated_count_types, temporal_count_types;
  for (auto Ii = sbPDG->inst_begin(), Ei = sbPDG->inst_end(); Ii != Ei; ++Ii) {
    if (!(*Ii)->is_temporal()) {
      dedicated_count_types[(*Ii)->inst()]++;
    } else {
      temporal_count_types[(*Ii)->inst()]++;
    }
  }

  for (auto &pair : dedicated_count_types) {
    sb_inst_t sb_inst = pair.first;
    int pdg_count = pair.second;

    int fu_count = 0;
    for (sbfu* cand_fu : _sbModel->subModel()->fu_list()) {
      if (cand_fu->fu_def()->is_cap(sb_inst)) {
        fu_count += 64 / SB_CONFIG::bitwidth[sb_inst];
      }
    }
    if (fu_count < pdg_count) {
      failed_count_check = true;
      cerr << "Error: PDG has " << pdg_count << " " << name_of_inst(sb_inst)
           << " dedicated insts, but only " << fu_count
           << " dedicated fus to support them\n";
    }
  }

  for (auto &pair : temporal_count_types) {
    sb_inst_t sb_inst = pair.first;
    int pdg_count = pair.second;

    int fu_count = 0;
    for (int i = 0; i < _sbModel->subModel()->sizex(); ++i) {
      for (int j = 0; j < _sbModel->subModel()->sizey(); ++j) {
        sbfu *cand_fu = _sbModel->subModel()->fus()[i][j];
        if (cand_fu->max_util() > 1 && cand_fu->fu_def()->is_cap(sb_inst)) {
          fu_count += cand_fu->max_util(sb_inst);
        }
      }
    }
    if (fu_count < pdg_count) {
      failed_count_check = true;
      cerr << "Error: PDG has " << pdg_count << " " << name_of_inst(sb_inst)
           << " temporal insts, but only "
           << fu_count << " temporal fu slots to support them\n";
    }
  }


  if (failed_count_check) {
    cerr << "\n\nError: FAILED Basic FU Count Check\n\n";
    return false;
  }
  //TODO: add code from printPortcompatibility here

  return true;
}


