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

#define one2one 0
#define flexible 1
#define random 2
#define mapping random/*0: Randomized(random), 1: Flexible, 2: One-2-One(one2one)*/

bool SchedulerStochasticGreedy::schedule(SbPDG* sbPDG, Schedule*& sched)
{
  int upperbound = 50000;
  int max_iters_no_improvement = 100000000;
  srand(2);

  progress_initBestNums();

  Schedule* cur_sched = NULL;
  std::pair<int, int> best_score = make_pair(0, 0);
  bool best_succeeded = false;
  bool best_mapped = false;
  std::set<SbPDG_Output*> best_dummies;

  int last_improvement_iter = 0;

  int presize = sbPDG->num_insts();
  int postsize = presize;

  // An attempt to remap SbPDG
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  int hw_num_fu = subModel->sizex()*subModel->sizey();

  int remapNeeded = sbPDG->remappingNeeded(hw_num_fu); //setup remap structres 
  int iter = 0;
  while (iter < upperbound) {
    if(remapNeeded) { //remap every so often to try new possible dummy positions
      if( (iter & (16-1)) == 0) {
        sbPDG->remap(hw_num_fu);
        postsize = sbPDG->num_insts();
      }
    }

    progress_initCurNums();
    bool succeed_sched = schedule_internal(sbPDG, cur_sched);
    bool succeed_timing = false;

    int tot_mapped =
      progress_getBestNum(FA) + progress_getBestNum(Input) + progress_getBestNum(Output);
    int lat = -10000, latmis = 0;
    if (succeed_sched) { 
      succeed_timing = cur_sched->fixLatency(lat,latmis);
      tot_mapped+=succeed_timing; //add one to total_mapped if timing succeeded
      lat *= -1;
    }

    std::pair<int, int> score = make_pair(tot_mapped, lat);
    if (score > best_score) {
      if(remapNeeded) {
        sbPDG->printGraphviz("remap.dot");
        best_dummies = sbPDG->getDummiesOutputs();
      }

  
      if (verbose) {
        fprintf(stderr, "Iter: %4d, mapped: %3d, lat: %3d, ins: %d/%d, outs: %d/%d,"
                " insts: %d/%d,%d%s%s\n", iter, score.first - 1, -score.second,
                progress_getBestNum(Input), sbPDG->num_inputs(),
                progress_getBestNum(Output), sbPDG->num_outputs(),
                progress_getBestNum(FA), presize, postsize,
                succeed_sched ? ", all mapped" : "",
                succeed_timing ? ", timing met" : "");
      }
      best_score = score;
      if (sched) {
        delete sched;
      }
      sched = cur_sched;
      if (succeed_timing && !best_succeeded) {
        max_iters_no_improvement = std::max(1000, iter * 2);
      }

      best_mapped = succeed_sched;
      best_succeeded = succeed_timing;
      last_improvement_iter = iter;
    } else {
      delete cur_sched;
    }
    cur_sched = NULL;
    iter++;

    if (((iter - last_improvement_iter) > max_iters_no_improvement) && best_succeeded) {
      break;
    }
  }
  if(verbose) {
    cerr << "Breaking at Iter " << iter << "\n";
  }

  //Fix back up any dummies
  if(remapNeeded) {
    sbPDG->rememberDummies(best_dummies);
  }

  if(verbose) {
    if(best_mapped && !best_succeeded) {
      fprintf(stderr,"DFG Mapped, but Timing not met.  Simulation is still possible, but do not use generated schedule for hardware!");
    }
  }
  return best_succeeded;
}


bool can_go(SbPDG_Inst* n, unordered_map<SbPDG_Inst*,bool>& seen) {
  if(!n || seen[n]) {
    return false;
  }
  return true;
  for(auto II=n->ops_begin(), EE=n->ops_end(); II!=EE; ++II) {
    if(*II == 0) continue;
    if(SbPDG_Inst* inc_node = dynamic_cast<SbPDG_Inst*>((*II)->def())) {
      if(!seen[inc_node]) {
        return false;
      }
    }
  }
  return true;
}

bool SchedulerStochasticGreedy::schedule_internal(SbPDG* sbPDG, Schedule*& sched) {
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

    for(auto I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(can_go(use_pdginst,seen)) {
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
      if(!n->isDummy()) {
         progress_incCurNum(FA);
      }
    } else {
      progress_saveBestNum(FA);
      return false;
    }

    for(auto I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(can_go(use_pdginst,seen)) {
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

  std::pair<int,int> bestScore = fscore;
  CandidateRouting* bestRouting = new CandidateRouting();
  sbnode* bestspot = NULL;

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
  if ( (r1 % 8) == 1) { //Pick a totally random spot
    bool succ = false;
    unsigned int attempt = 0;
    while (!succ && attempt < spots.size()) {
      int r2 = rand() % spots.size();
      sbnode* cand_spot = spots[r2];

      bestspot = cand_spot;;
      bestRouting->routing.clear();
      bestRouting->forwarding.clear();

      bestScore = scheduleHere(sched, pdgnode, cand_spot,*bestRouting,bestScore);
      

      if(bestScore < fscore) {
        applyRouting(sched,pdgnode,bestspot,bestRouting);
        return true;
      } else {
        attempt++;
      }
    }
    return false; // couldn't find one

  } else {  
    for(unsigned i=0; i < spots.size(); i++) {
      sbnode* cand_spot = spots[i];

      curRouting->routing.clear();
      curRouting->forwarding.clear();

      pair<int,int> curScore = scheduleHere(sched, pdgnode, cand_spot,*curRouting,bestScore);
      curScore.first = curScore.second;
      if(curScore < fscore && true /*mode = timinig minimize*/ ) {
        int curNodeLat=0;
        int diff = 0;
        curRouting->fill_lat(pdgnode,sched, diff, curNodeLat);
        if(diff < 500 && diff >15) {
          curScore=fscore;
        }
        curScore.second = diff;
        //curScore.first+=diff/4;
      }

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

    if(bestScore < fscore) {
      applyRouting(sched,pdgnode,bestspot,bestRouting);
    } else {
      return false;
    }  
    return true;
  }
}

std::pair<int,int> SchedulerStochasticGreedy::scheduleHere(Schedule* sched, 
    SbPDG_Node* n, SB_CONFIG::sbnode* here,
    CandidateRouting& candRouting, 
    std::pair<int,int> bestScore) {

  pair<int,int> score=make_pair(0,0);

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
      #if mapping == random
      unsigned index = 0;
      std::unordered_map<int,int> num2pos;
      std::unordered_map<int,int> hw2pdg;

      for (unsigned m=0; m < vport_desc.size(); m++) {
        int cgra_port_num = vport_desc[m].first;
        sbinput* cgra_in_port = subModel->get_input(cgra_port_num);
        //cout << cgra_in_port->name() << "\n"; 
        if (sched->pdgNodeOf(cgra_in_port) == NULL) {
          num2pos[index++] = m;
        }
      }
      unsigned fromSize = vec_in->num_inputs();
      unsigned toSize = index;
      unsigned start = 0, end = 0;

      if (toSize < fromSize) { 
        ports_okay_to_use=false;
        continue;
      }


      for (unsigned m=0; m < fromSize ; ++m) {
        end = toSize - (fromSize-1-m);
        int pn;
        do {
          pn = rand() % (end-start) + start;
        } while (hw2pdg.count(pn) != 0);

        int hwPort = num2pos[pn];
        //cout<<start<<" "<<end<<" "<<pn<<" "<<hwPort<<endl;
        //cout<<start<<" "<<end<<" "<<pn<<" "<<hwPort<<" "<<fromSize<<" "<<toSize<<" "<<m<<endl;
        start = end;
        //Get the sbnode corresponding to mask[m]
        int cgra_port_num = vport_desc[hwPort].first;
        sbinput* cgra_in_port = subModel->get_input(cgra_port_num);
        //Get the input pdgnode corresponding to m
        assert(sched->pdgNodeOf(cgra_in_port) == NULL);
        //cout << "Assigned "<<cgra_in_port->name() << "\n"; 
        hw2pdg[pn]=m;
        progress_incCurNum(Input);
      }

      #elif mapping == one2one
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
      #endif

      progress_saveBestNum(Input);
      if (!ports_okay_to_use) {
        continue;
      }
#if mapping == one2one
      for(unsigned m=0; m < vec_in->num_inputs(); ++m) {
        int hwPort = m;
        int pdgPort = m;
#else
        for (auto i: hw2pdg) {
          int hwPort = num2pos[i.first];
          int pdgPort = i.second;
#endif
        mask[hwPort]=true;
        //Get the sbnode corresponding to mask[m]
        int cgra_port_num = vport_desc[hwPort].first;
        sbinput* cgra_in_port = subModel->get_input(cgra_port_num);

        //Get the input pdgnode corresponding to m
        SbPDG_Node* sbpdg_input = vec_in->getInput(pdgPort);
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

      //Check if it's okay to assign to these ports

      /*Randomized Mapping*/
#if mapping == random
      unsigned index = 0;
      std::unordered_map<int,int> num2pos;
      std::unordered_map<int,int> hw2pdg;
      for (unsigned m=0; m < vport_desc.size(); m++) {
        int cgra_port_num = vport_desc[m].first;
        sboutput* cgra_out_port = subModel->get_output(cgra_port_num);
        if (sched->pdgNodeOf(cgra_out_port) == NULL) {
          num2pos[index++] = m;
        }
      }
      if (index < vec_out->num_outputs()) { 
        ports_okay_to_use=false;
        continue;
      }
      unsigned fromSize = vec_out->num_outputs();
      unsigned toSize = index;
      unsigned start = 0, end = 0;
      for(unsigned m=0; m < fromSize ; ++m) {
        end = toSize - (fromSize-1-m);
        int pn;
        do {
          pn = rand() % (end-start) + start;
        } while (hw2pdg.count(pn)!=0);

        int hwPort = num2pos[pn];
        //cout<<start<<" "<<end<<" "<<pn<<" "<<hwPort<<" "<<fromSize<<" "<<toSize<<" "<<m<<endl;
        start = end;
        //Get the sbnode corresponding to mask[m]
        int cgra_port_num = vport_desc[hwPort].first;
        sboutput* cgra_out_port = subModel->get_output(cgra_port_num);
        //Get the input pdgnode corresponding to m
        SbPDG_Node* sbpdg_output = vec_out->getOutput(m);
        assert(sched->pdgNodeOf(cgra_out_port) == NULL);
        std::pair<int,int> curScore = scheduleHere(sched, sbpdg_output, cgra_out_port, candRouting, fscore); 
        if(curScore>=fscore) { //?????
          ports_okay_to_use=false;
          break;
        }
        hw2pdg[pn]=m;
        progress_incCurNum(Output);
      }
#elif mapping == flexible
      /*Flexible Mapping*/
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
        std::pair<int,int> curScore = scheduleHere(sched, sbpdg_output, cgra_out_port, candRouting, fscore); 
        if (curScore>=fscore) { //?????
          ports_okay_to_use=false;
          continue;
        }
        hw2pdg[m]=num;
        progress_incCurNum(Output);
        num++;
      }
#elif mapping == one2one
      /*One-to-One Mapping*/
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
        std::pair<int,int> curScore = scheduleHere(sched, sbpdg_output, cgra_out_port, 0, candRouting, fscore); 
        if(curScore>=fscore) { //?????
          ports_okay_to_use=false;
          break;
        }
        progress_incCurNum(Output);
      }
#endif
      progress_saveBestNum(Output);  
      if (!ports_okay_to_use) {
        //cout << "skipping this port assignment\n";
        continue; //don't assign these ports
      }
      //cout<<"Succeeded!"<<endl;
      // Assign Individual Elements
#if mapping == one2one
      for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
        int hwPort = m;
        int pdgPort = m;
#else
        for (auto i: hw2pdg) {
          int hwPort = num2pos[i.first];
          int pdgPort = i.second;
#endif
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
      applyRouting(sched, &candRouting); //Commit the routing

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
