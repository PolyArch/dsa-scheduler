#include "scheduler.h"

using namespace SB_CONFIG;
using namespace std;

#include <unordered_map>
#include <fstream>
#include <list>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>


/*bool Scheduler::assignVectorInputs(SbPDG* sbPDG, Schedule* sched) {
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();

  int n = sbPDG->num_vec_input();

  for(int i = 0; i < n; ++i) {
    SbPDG_VecInput* vec_in = sbPDG->vec_in(i);
    cout << "Assigning Vector Port:" << vec_in->gamsName() <<"\n";
    
    bool found_vector_port = false;

    for(auto& i : subModel->io_interf().in_vports) {
      int vport_num = i.first;
      auto vport_id = std::make_pair(true,vport_num);
      vector<pair<int, vector<int>>>& vport_desc = i.second;

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
          cout << "skipping this port assignment\n";
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
          sched->assign_node(sbpdg_input,cgra_in_port,0);
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
      cout << "Could not find hardware vector port\n";
      return false;
    }
  }
  return true;
}


bool Scheduler::assignVectorOutputs(SbPDG* sbPDG, Schedule* sched) {
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  CandidateRouting candRouting;  

  int n = sbPDG->num_vec_output();

  for(int i = 0; i < n; ++i) {
    SbPDG_VecOutput* vec_out = sbPDG->vec_out(i);
    cout << "Assigning Vector Port:" << vec_out->gamsName() <<"\n";
    
    bool found_vector_port = false;

    for(auto& i : subModel->io_interf().out_vports) {
      int vport_num = i.first;
      auto vport_id = std::make_pair(false,vport_num);
      vector<pair<int, vector<int>>>& vport_desc = i.second;

      //Check if the vetcor port is 1. big enough & 2. unassigned
      if(vec_out->num_outputs() <= vport_desc.size() && 
         sched->vportOf(vport_id) == NULL) {
        std::vector<bool> mask; 
        mask.resize(vport_desc.size());

        candRouting.clear();
        bool ports_okay_to_use=true;

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

          int curScore = scheduleHere(sched, sbpdg_output, cgra_out_port, 
                                      0,candRouting,MAX_ROUTE);
          if(curScore>=MAX_ROUTE) {
            ports_okay_to_use=false;
            break;
          }
        }
        if(!ports_okay_to_use) {
          cout << "skipping this port assignment\n";
          continue; //don't assign these ports
        }

        applyRouting(sched, 0, &candRouting); //Commit the routing

        // Assign Individual Elements
        for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
          mask[m]=true;
         
          //Get the sbnode corresponding to mask[m]
          int cgra_port_num = vport_desc[m].first;
          sboutput* cgra_out_port = subModel->get_output(cgra_port_num);

          //Get the input pdgnode corresponding to m
          SbPDG_Node* sbpdg_output = vec_out->getOutput(m);
          
          sched->assign_node(sbpdg_output,cgra_out_port,0);
        }
        //Perform the vector assignment
        sched->assign_vport(vec_out,vport_id,mask);
        found_vector_port=true;
      }
      if(found_vector_port) {
        break;
      }
    }
    if(!found_vector_port) {
      cout << "Could not find hardware vector port\n";
      return false;
    }

  }
  return true;
}
*/

bool Scheduler::scheduleBacktracking(SbPDG* sbPDG, Schedule*& sched) {
  sched = new Schedule(_sbModel,sbPDG);
  sched->setNConfigs(1);

  bool vec_in_assigned = assignVectorInputs(sbPDG,sched);
  if(!vec_in_assigned) {
    return false;
  }

  map<SbPDG_Inst*,bool> seen;
  bool schedule_okay=true; 
 
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
      schedule_okay&=scheduleNodeBacktracking(sched,n);
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

  bool vec_out_assigned = assignVectorOutputs(sbPDG,sched);
  if(!vec_out_assigned) {
    return false;
  }
   
  return schedule_okay;
}


bool Scheduler::scheduleNodeBacktracking(Schedule* sched, SbPDG_Node* pdgnode) {
 /* 
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
        return true;
      }//apply routing step
    
    }//for loop -- check for all sbnode spots
  }
  
  
  //TODO: If not scheduled, then increase the numConfigs, and try again
  
  if(bestScore < MAX_ROUTE) {
    applyRouting(sched,pdgnode,bestspot,bestconfig,bestRouting);
  } else {
    cout << "no route found for pdgnode: " << pdgnode->name() << "\n";
    return false; 
  }
 */   
  return true;
}
