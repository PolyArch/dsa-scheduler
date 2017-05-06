#include "scheduler_sg.h"

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
#include <time.h>       /* time */

bool SchedulerStochasticGreedy::schedule(SbPDG* sbPDG, Schedule*& sched) {
  sched = new Schedule(getSBModel(),sbPDG);

  bool vec_in_assigned = assignVectorInputs(sbPDG,sched);
  if(!vec_in_assigned) {
    return false;
  }

  unordered_map<SbPDG_Inst*,bool> seen;
  bool schedule_okay=true; 

  list<SbPDG_Inst* > openset;
  SbPDG::const_input_iterator I,E;

  //pdg input nodes
  for(I=sbPDG->input_begin(),E=sbPDG->input_end();I!=E;++I) {
    SbPDG_Input* n = *I;

    SbPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(use_pdginst && seen.count(use_pdginst)==0) {
        seen[use_pdginst]=true;
        openset.push_back(use_pdginst);
      }
    }
  }

  //populate the schedule object
  while(!openset.empty()) {
    SbPDG_Inst* n = openset.front(); 
    openset.pop_front();

    schedule_okay&=scheduleNode(sched,n);
    if (schedule_okay) {
      progress_incCurNum(FA);
    } else {
      progress_saveBestNum(FA);
      return false;
    }
    SbPDG_Node::const_edge_iterator I,E;
    for(I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(use_pdginst && seen.count(use_pdginst)==0) {
        seen[use_pdginst]=true;
        openset.push_back(use_pdginst);
      }
    }
  }

  progress_saveBestNum(FA);

  bool vec_out_assigned = assignVectorOutputs(sbPDG,sched);
  if(!vec_out_assigned) {
    return false;
  }

  return schedule_okay;
}


bool SchedulerStochasticGreedy::scheduleNode(Schedule* sched, SbPDG_Node* pdgnode) {

  std::pair<int,int> bestScore = make_pair(0,MAX_ROUTE);
  std::pair<int,int> fscore = make_pair(0,MAX_ROUTE);
  CandidateRouting* bestRouting = new CandidateRouting();
  sbnode* bestspot=NULL;

  CandidateRouting* curRouting = new CandidateRouting();  

  std::vector<sbnode*> spots;

  if(SbPDG_Inst* pdginst= dynamic_cast<SbPDG_Inst*>(pdgnode))  { 
    fillInstSpots(sched, pdginst, spots);             //all possible candidates based on FU capability 
  } else if(SbPDG_Input* pdg_in = dynamic_cast<SbPDG_Input*>(pdgnode)) {
    fillInputSpots(sched,pdg_in,spots); 
  } else if(SbPDG_Output* pdg_out = dynamic_cast<SbPDG_Output*>(pdgnode)) {
    fillOutputSpots(sched,pdg_out,spots); 
  }

  //populate a scheduling score for each of canidate sbspot
  int r1 = rand() % 100;
  if ( r1 % 2 == 1) {
    bool succ = false;
    unsigned int attempt = 0;
    while (!succ && attempt < spots.size()) {
      int r2 = rand() % spots.size();
      sbnode* cand_spot = spots[r2];

      bestspot = cand_spot;;
      curRouting->routing.clear();
      curRouting->forwarding.clear();

      bestScore = scheduleHere(sched, pdgnode, cand_spot,*bestRouting,bestScore);

      if(bestScore < fscore) {
        succ = true;
      } else {
        attempt++;
      }
    }
  }
  else { 
    for(unsigned i=0; i < spots.size(); i++) {
      sbnode* cand_spot = spots[i];

      curRouting->routing.clear();
      curRouting->forwarding.clear();

      pair<int,int> curScore = scheduleHere(sched, pdgnode, cand_spot, *curRouting,bestScore);

      if(curScore < bestScore) {
        bestScore=curScore;
        bestspot=cand_spot;
        std::swap(bestRouting,curRouting);
      }

      if(bestScore<=make_pair(0,1))  {
        applyRouting(sched,pdgnode,bestspot,bestRouting);
        return true;
      }//apply routing step

    }//for loop -- check for all sbnode spots
  }


  if(bestScore < fscore) {
    applyRouting(sched,pdgnode,bestspot,bestRouting);
  } else {
    //cout << "no route found for pdgnode: " << pdgnode->name() << "\n";
    return false;
  }  
  return true;
}

std::pair<int,int> SchedulerStochasticGreedy::scheduleHere(Schedule* sched, 
    SbPDG_Node* n, SB_CONFIG::sbnode* here,
    CandidateRouting& candRouting, 
    std::pair<int,int> bestScore) {

  pair<int,int> score=make_pair(0,0);
  pair<int,int> fscore = make_pair(0,MAX_ROUTE);

  SbPDG_Node::const_edge_iterator I,E;

  for(I=n->ops_begin(), E=n->ops_end();I!=E;++I) {
    if(*I == NULL) { continue; } //could be immediate
    SbPDG_Edge* source_pdgegde = (*I);
    SbPDG_Node* source_pdgnode = source_pdgegde->def();     //could be input node also

    //route edge if source pdgnode is scheduled
    if(sched->isScheduled(source_pdgnode)) {
      sbnode* source_loc = sched->locationOf(source_pdgnode); //scheduled location

      //route using source node, sbnode
      pair<int,int> tempScore = route(sched, source_pdgegde, source_loc, here,candRouting,bestScore-score);
      score = score + tempScore;
      //cout << n->name() << " " << here->name() << " " << score << "\n";
      if(score>bestScore) return fscore;
    }
  }

  SbPDG_Node::const_edge_iterator Iu,Eu;
  for(Iu=n->uses_begin(), Eu=n->uses_end();Iu!=Eu;++Iu) {
    SbPDG_Edge* use_pdgedge = (*Iu);
    SbPDG_Node* use_pdgnode = use_pdgedge->use();

    //route edge if source pdgnode is scheduled
    if(sched->isScheduled(use_pdgnode)) {
      sbnode* use_loc = sched->locationOf(use_pdgnode);

      pair<int,int> tempScore = route(sched, use_pdgedge, here, use_loc,candRouting,bestScore-score);
      score = score + tempScore;
      //cout << n->name() << " " << here->name() << " " << score << "\n";
      if(score>bestScore) return score;
    }
  }

  return score;
}

pair<int,int> SchedulerStochasticGreedy::route(Schedule* sched, SbPDG_Edge* pdgedge,
    sbnode* source, sbnode* dest, CandidateRouting& candRouting, pair<int,int> scoreLeft) {
  pair<int,int> score = route_minimizeDistance(sched, pdgedge, source, dest, candRouting, scoreLeft);
  return score;
}

bool SchedulerStochasticGreedy::assignVectorInputs(SbPDG* sbPDG, Schedule* sched) {
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

    unsigned int index = 0;
    findFirstIndex(sd, si, vec_in->num_inputs(), index, Input);
    unsigned int attempt = 0;
    int curNum = progress_getCurNum(Input);

    do {
      progress_updateCurNum(Input, curNum);
      pair<bool, int> vport_id;
      vector<pair<int, vector<int>>> vport_desc;
      genRandomIndexBW(vport_id, vport_desc, sd, si, sd.size(), index, sched, Input);

      //Check if the vetcor port is 2. unassigned
      if (sched->vportOf(vport_id) != NULL) {
        progress_saveBestNum(Input);
        return false;
      }

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
      if (!ports_okay_to_use) {
        continue;
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
    } while ((attempt++ < sd.size() - index -1) && !found_vector_port);

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

void SchedulerStochasticGreedy::findFirstIndex(vector<pair<int,int>>& sd, sbio_interface& si, 
    unsigned int numIO, unsigned int& index, StatType s) {
  for (; index < sd.size(); index++) {
    pair<int,int> j = sd[index];
    int vn = j.first; 
    assert(s==Output || s==Input);
    vector<pair<int, vector<int>>>& vdesc = (s == Output ? si.getDesc_O(vn) : si.getDesc_I(vn));
    //Check if the vetcor port is 1. big enough
    if (numIO <= vdesc.size()) { break;}
  }
}

void SchedulerStochasticGreedy::genRandomIndexBW(pair<bool, int>& vport_id, vector<pair<int, vector<int>>>& vport_desc,  vector<pair<int,int>>& sd, sbio_interface& si, unsigned int size, unsigned int index, Schedule*& sched, StatType s) {
  unsigned int k;
  pair<int, int> p;
  int vport_num;
  unsigned int rep = 0;

  assert(s==Output || s==Input);

  do {
    k = rand() % (size - index) + index;
    p = sd[k];
    vport_num = p.first;
    vport_id = (s == Output ? std::make_pair(false/*output*/,vport_num) : std::make_pair(true/*input*/,vport_num));

    vport_desc = (s == Output ? si.getDesc_O(vport_num) : si.getDesc_I(vport_num));

  } while (sched->vportOf(vport_id) != NULL && rep++ < size);
}



bool SchedulerStochasticGreedy::assignVectorOutputs(SbPDG* sbPDG, Schedule* sched) {
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  CandidateRouting candRouting;  
  sbio_interface si =  subModel->io_interf();
  vector<pair<int,int>> sd;

  int n = sbPDG->num_vec_output();


  sbPDG->sort_vec_out();
  si.sort_out_vports(sd);

  for(int i = 0; i < n; ++i) {
    SbPDG_VecOutput* vec_out = sbPDG->vec_out(i);
    //cout << "Assigning Vector Port:" << vec_out->gamsName() <<"\n";

    bool found_vector_port = false;


    //for (auto& j : sd)
    unsigned int index = 0;
    findFirstIndex(sd, si, vec_out->num_outputs(), index, Output);
    unsigned int attempt = 0;
    int curNum = progress_getCurNum(Output);

    do {
      progress_updateCurNum(Output, curNum);
      pair<bool, int> vport_id;
      vector<pair<int, vector<int>>> vport_desc;
      genRandomIndexBW(vport_id, vport_desc, sd, si, sd.size(), index, sched, Output);

      //Check if the vetcor port is 2. unassigned
      if (sched->vportOf(vport_id) != NULL) { 
        progress_saveBestNum(Output);
        return false;
      }
      //cout<<"Considering port "<<vport_num<<endl;
      //else if (sched->vportOf(vport_id) == NULL) 
      std::vector<bool> mask; 
      mask.resize(vport_desc.size());
      candRouting.clear();
      bool ports_okay_to_use=true;

      unsigned int num = 0;
      std::map<int,int> hw2pdg;
      for (unsigned m=0; m < vport_desc.size() && num < vec_out->num_outputs(); m++) {
        int cgra_port_num = vport_desc[m].first;
        sboutput* cgra_out_port = subModel->get_output(cgra_port_num);
        //Get the input pdgnode corresponding to m
        SbPDG_Node* sbpdg_output = vec_out->getOutput(num);
        if(sched->pdgNodeOf(cgra_out_port) != NULL) {
          ports_okay_to_use=false;
          continue;
        } 
        std::pair<int,int> fscore = make_pair(0, MAX_ROUTE); //BUG: 0 should change to MAX_ROUTE for MLG, a better way to fix this is to define fscore for each subclass of HeursisticScheduler
        std::pair<int,int> curScore = scheduleHere(sched, sbpdg_output, cgra_out_port, candRouting, fscore); 
        if (curScore>=fscore) { //?????
          ports_okay_to_use=false;
          continue;
        }
        hw2pdg[m]=num;
        progress_incCurNum(Output);
        num++;
      }
      //Check if it's okay to assign to these ports
      /*for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
      //Get the sbnode corresponding to mask[m]
      int cgra_port_num = vport_desc[m].first;
      sboutput* cgra_out_port = subModel->get_output(cgra_port_num);
      //Get the input pdgnode corresponding to m
      SbPDG_Node* sbpdg_output = vec_out->getOutput(m);
      if(sched->pdgNodeOf(cgra_out_port) != NULL) {
      ports_okay_to_use=false;
      break;
      } 
      std::pair<int,int> fscore = make_pair(0, MAX_ROUTE); //BUG: 0 should change to MAX_ROUTE for MLG, a better way to fix this is to define fscore for each subclass of HeursisticScheduler
      std::pair<int,int> curScore = scheduleHere(sched, sbpdg_output, cgra_out_port, 0, candRouting, fscore); 
      if(curScore>=fscore) { //?????
      ports_okay_to_use=false;
      break;
      }
      progress_incCurNum(Output);
      }*/
      progress_saveBestNum(Output);
      if (!ports_okay_to_use) {
        //cout << "skipping this port assignment\n";
        continue; //don't assign these ports
      }
      cout<<"Succeeded!"<<endl;
      applyRouting(sched, &candRouting); //Commit the routing
      // Assign Individual Elements
      //for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
      for (auto i: hw2pdg) {
        int hwPort = i.first;
        int pdgPort = i.second;
        mask[hwPort]=true;
        //Get the sbnode corresponding to mask[m]
        int cgra_port_num = vport_desc[hwPort].first;
        sboutput* cgra_out_port = subModel->get_output(cgra_port_num);
        //Get the input pdgnode corresponding to m
        SbPDG_Node* sbpdg_output = vec_out->getOutput(pdgPort);
        sched->assign_node(sbpdg_output,cgra_out_port);
      }
      //Perform the vector assignment
      sched->assign_vport(vec_out,vport_id,mask);
      found_vector_port=true;
    } while (attempt++ < sd.size() - index - 1 && !found_vector_port);

      if (!found_vector_port) {
        progress_saveBestNum(Output);
        //cout << "Could not find output hardware vector port\n";
        return false;
      }
      //progress_incCurNum(Output);
    }
    progress_saveBestNum(Output);
    return true;
  }
