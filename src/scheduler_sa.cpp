#include "scheduler_sa.h"

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

#define DEBUG_SCHED (false)

void SchedulerSimulatedAnnealing::initialize(SbPDG* sbPDG, Schedule*& sched) {
  //Sort the input ports once
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  sbio_interface& si =  subModel->io_interf();
  _sd_in.clear();
  si.sort_in_vports(_sd_in);
  si.sort_out_vports(_sd_out);

  sbPDG->sort_vec_in();
  sbPDG->sort_vec_out();

  sched = new Schedule(getSBModel(),sbPDG); //just a dummy one
}

std::pair<int, int> SchedulerSimulatedAnnealing::obj(
    Schedule*& sched, int& lat, int& latmis) {  
    int num_left = sched->num_left(); 
    bool succeed_sched = (num_left==0);

    bool succeed_timing = false;
    if (succeed_sched) { 
      succeed_timing = sched->fixLatency(lat,latmis);
    } else {
      latmis = MAX_ROUTE;
    }

    int violation = sched->violation();

    int obj = latmis*10000+violation*100+lat;

    return make_pair(succeed_sched + succeed_timing-num_left, -obj);
}

bool SchedulerSimulatedAnnealing::schedule(SbPDG* sbPDG, Schedule*& sched) {  
  initialize(sbPDG,sched);

  static int _srand_no=1; 

  int max_iters_no_improvement = 100000000;
  srand(++_srand_no);

  Schedule* cur_sched = new Schedule(getSBModel(),sbPDG);
  std::pair<int, int> best_score = make_pair(0, 0);
  bool best_succeeded = false;
  bool best_mapped = false;
  std::set<SbPDG_Output*> best_dummies;

  int last_improvement_iter = 0;

  _best_latmis=MAX_ROUTE;
  _best_lat=MAX_ROUTE;
  _best_violation=MAX_ROUTE;

  int presize = sbPDG->num_insts();
  int postsize = presize;

  // An attempt to remap SbPDG
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  int hw_num_fu = subModel->sizex()*subModel->sizey();

  int remapNeeded = false; //sbPDG->remappingNeeded(hw_num_fu); //setup remap structres 
  int iter = 0;
  while (iter < _max_iters) {
    if( (iter & (256-1)) == 0) {
      if( total_msec() > _reslim * 1000) {
        break;
      }
    }
    if( (iter & (1024-1)) == 0) {
      //Every so often, lets give up and start over from scratch
      delete cur_sched;
      cur_sched = new Schedule(getSBModel(),sbPDG);
     
      if(remapNeeded) { //remap every so often to try new possible dummy positions
        sbPDG->remap(hw_num_fu);
        postsize = sbPDG->num_insts();
        cur_sched->allocate_space();
      }
    }

    bool succeed_sched = schedule_internal(sbPDG, cur_sched);

    int lat=MAX_ROUTE,latmis=MAX_ROUTE;
    std::pair<int,int> score = obj(cur_sched,lat,latmis);

    int succeed_timing = (latmis ==0);
    if (score > best_score) {
      if(_integrate_timing && succeed_sched) { //set new best latmis to bound it
        _best_latmis=latmis;
        //_best_violation=violation;
      }

      if(remapNeeded) {
        sbPDG->printGraphviz("viz/remap.dot");
        best_dummies = sbPDG->getDummiesOutputs();
      }

      if (verbose) {
        fprintf(stdout, "Iter: %4d, time:%0.2f, rt:%d, left: %3d, " 
                "lat: %3d, vio %d, mis: %d, obj:%d, ins: %d/%d, outs: %d/%d,"
                " insts: %d/%d,%d, links:%d, edge-links:%d  %s%s\n", 
                iter, total_msec()/1000.f, _route_times,
                cur_sched->num_left(), lat, 
                cur_sched->violation(), latmis, -score.second,
                cur_sched->num_inputs_mapped(),  sbPDG->num_inputs(),
                cur_sched->num_outputs_mapped(), sbPDG->num_outputs(),
                cur_sched->num_insts_mapped(),  presize, postsize,
                cur_sched->num_links_mapped(),
                cur_sched->num_edge_links_mapped(),
                succeed_sched ? ", all mapped" : "",
                succeed_timing ? ", mismatch == 0" : "");
      }
      best_score = score;
      *sched = *cur_sched; //shallow copy of sched should work?
      if (succeed_timing && !best_succeeded) {
        max_iters_no_improvement = std::max(1000, iter * 2);
      }

      best_mapped = succeed_sched;
      best_succeeded = succeed_timing;
      last_improvement_iter = iter;
    }

    iter++;

    if (((iter - last_improvement_iter) > max_iters_no_improvement) 
        && best_succeeded) {
      break;
    }
    if(sched->violation() ==0 && iter > _max_iters_zero_vio) {
      break;
    }
  }
  if(verbose) {
    cout << "Breaking at Iter " << iter << "\n";
  }

  //Fix back up any dummies
  if(remapNeeded) {
    sbPDG->rememberDummies(best_dummies);
  }

  return best_mapped;
}

void SchedulerSimulatedAnnealing::findFirstIndex(vector<pair<int,int>>& sd, 
    sbio_interface& si, unsigned int numIO, unsigned int& index, bool is_input){
  for (; index < sd.size(); index++) {
    pair<int,int> j = sd[index];
    int vn = j.first; 
    vector<pair<int, vector<int>>>& vdesc = (!is_input ? si.getDesc_O(vn) : si.getDesc_I(vn));
    //Check if the vetcor port is 1. big enough
    if (numIO <= vdesc.size()) { break;}
  }
}

void SchedulerSimulatedAnnealing::genRandomIndexBW(pair<bool, int>& vport_id, vector<pair<int, vector<int>>>& vport_desc,  vector<pair<int,int>>& sd, sbio_interface& si, unsigned int size, unsigned int index, Schedule*& sched, bool is_input) {
  unsigned int k;
  pair<int, int> p;
  int vport_num;
  unsigned int rep = 0;

  do {
    k = rand() % (size - index) + index;
    p = sd[k];
    vport_num = p.first;
    vport_id = std::make_pair(is_input,vport_num);

    vport_desc = (!is_input ? si.getDesc_O(vport_num) : si.getDesc_I(vport_num));

  } while (sched->vportOf(vport_id) != NULL && rep++ < size);
}


bool SchedulerSimulatedAnnealing::schedule_input( SbPDG_VecInput*  vec_in, 
    SbPDG* sbPDG, Schedule* sched) {

  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  sbio_interface& si =  subModel->io_interf();
  bool found_vector_port = false;
  CandidateRouting candRouting;  


  unsigned int index = 0;
  findFirstIndex(_sd_in, si, vec_in->num_inputs(), index, true /*input*/);
  unsigned int attempt = 0;

  do { 
    pair<bool, int> vport_id;
    vector<pair<int, vector<int>>> vport_desc;
    genRandomIndexBW(vport_id, vport_desc, _sd_in, si, 
                     _sd_in.size(), index, sched, true /*input*/);

    //Check if the vetcor port is 2. unassigned
    if (sched->vportOf(vport_id) != NULL) {
      return false;
    }

    std::vector<bool> mask; 
    mask.resize(vport_desc.size());
    candRouting.clear();

    bool ports_okay_to_use=true;

    unsigned index = 0;
    std::unordered_map<int,int> num2pos;
    std::unordered_map<int,int> hw2pdg;

    for (unsigned m=0; m < vport_desc.size(); m++) {
      int cgra_port_num = vport_desc[m].first;
      sbinput* cgra_in_port = subModel->get_input(cgra_port_num);
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
      start = end;
      //Get the sbnode corresponding to mask[m]
      int cgra_port_num = vport_desc[hwPort].first;
      sbinput* cgra_in_port = subModel->get_input(cgra_port_num);
      //Get the input pdgnode corresponding to m
      assert(sched->pdgNodeOf(cgra_in_port) == NULL); 
      SbPDG_Node* sbpdg_input = vec_in->getInput(m);
      assert(!sched->isScheduled(sbpdg_input));
      std::pair<int,int> curScore = scheduleHere(sched, 
          sbpdg_input, cgra_in_port, candRouting, fscore); 
      if(curScore>=fscore) { //?????
        ports_okay_to_use=false;
        break;
      }

      hw2pdg[pn]=m;
    }

    if (!ports_okay_to_use) {
      continue;
    }

    for (auto i: hw2pdg) {
      int hwPort = num2pos[i.first];
      int pdgPort = i.second;
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
    applyRouting(sched, &candRouting); //Commit the routing

  } while ((attempt++ < _sd_in.size() - index -1) && !found_vector_port);

  return found_vector_port;
}

bool SchedulerSimulatedAnnealing::schedule_output(SbPDG_VecOutput* vec_out, 
    SbPDG* sbPDG, Schedule* sched) {
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  CandidateRouting candRouting;  
  sbio_interface& si =  subModel->io_interf();

  bool found_vector_port = false;

  unsigned int index = 0;
  findFirstIndex(_sd_out, si, vec_out->num_outputs(), index, false/*output*/);
  unsigned int attempt = 0;

  do {
    pair<bool, int> vport_id;
    vector<pair<int, vector<int>>> vport_desc;
    genRandomIndexBW(vport_id, vport_desc, _sd_out, si, 
                     _sd_out.size(), index, sched, false/*output*/);

    //Check if the vetcor port is 2. unassigned
    if (sched->vportOf(vport_id) != NULL) { 
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
      start = end;
      //Get the sbnode corresponding to mask[m]
      int cgra_port_num = vport_desc[hwPort].first;
      sboutput* cgra_out_port = subModel->get_output(cgra_port_num);
      //Get the input pdgnode corresponding to m
      SbPDG_Node* sbpdg_output = vec_out->getOutput(m);
      assert(sched->pdgNodeOf(cgra_out_port) == NULL);
      assert(!sched->isScheduled(sbpdg_output));
      std::pair<int,int> curScore = scheduleHere(sched, 
          sbpdg_output, cgra_out_port, candRouting, fscore); 
      if(curScore>=fscore) { //?????
        ports_okay_to_use=false;
        break;
      }
      hw2pdg[pn]=m;
    }
    if (!ports_okay_to_use) {
      //cout << "skipping this port assignment\n";
      continue; //don't assign these ports
    }
    //cout<<"Succeeded!"<<endl;
    // Assign Individual Elements
    for (auto i: hw2pdg) {
      int hwPort = num2pos[i.first];
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
    applyRouting(sched, &candRouting); //Commit the routing
  } while (attempt++ < _sd_out.size() - index - 1 && !found_vector_port);

  return found_vector_port;
}

bool SchedulerSimulatedAnnealing::timingIsStillGood(Schedule* sched) {
  int max_lat=0, max_lat_mis=0;
  sched->cheapCalcLatency(max_lat,max_lat_mis);
  return max_lat_mis < _sbModel->maxEdgeDelay();
}

bool SchedulerSimulatedAnnealing::map_one_input(SbPDG* sbPDG, Schedule* sched) {
  int n = sbPDG->num_vec_input();

  int p = rand_bt(0,n);
  for(int i = 0; i < n; ++i) {
    if(++p ==n) {p=0;}

    SbPDG_VecInput* vec_in = sbPDG->vec_in(p);
    if(sched->vecMapped(vec_in)) continue;    
    return schedule_input(vec_in,sbPDG,sched) && timingIsStillGood(sched);
  }
  return false;
}

bool SchedulerSimulatedAnnealing::map_one_output(SbPDG* sbPDG, Schedule* sched) {
   int n = sbPDG->num_vec_output();

  int p = rand_bt(0,n);
  for(int i = 0; i < n; ++i) {
    if(++p ==n) {p=0;}
    SbPDG_VecOutput* vec_out = sbPDG->vec_out(p);
    if(sched->vecMapped(vec_out)) continue;
    return schedule_output(vec_out,sbPDG,sched);
  }
  return false; 
}

bool SchedulerSimulatedAnnealing::map_one_inst(SbPDG* sbPDG, Schedule* sched) {
  std::vector<SbPDG_Inst*>& inst_vec = sbPDG->inst_vec();
  int n = inst_vec.size();

  int p = rand_bt(0,n);
  for(int i = 0; i < n; ++i) {
    if(++p ==n) {p=0;}
    SbPDG_Inst* inst = inst_vec[p];
    if(sched->isScheduled(inst)) continue;
    return scheduleNode(sched,inst) && timingIsStillGood(sched);
  }
  return false;
}

bool SchedulerSimulatedAnnealing::map_to_completion(SbPDG* sbPDG, Schedule* sched) {
  bool input_good=true;
  bool output_good=true;
  bool inst_good=true;

  if(DEBUG_SCHED) cout << "Map to completion! " << sched->num_mapped() << "\n";
  while(!sched->isComplete()) {
    int r = rand_bt(0,5); //upper limit defines ratio of input/output scheduling
    switch(r) {
      case 0: {
        if(!input_good) break;
        bool success = map_one_input(sbPDG,sched);
        if(!success) {
          return false;
        } else {
          input_good = !sched->inputs_complete();
        }
        break;
      }
      break;
      case 1: {
        if(!output_good) break;
        bool success = map_one_output(sbPDG,sched);
        if(!success) {
          return false;
        } else {
          output_good = !sched->outputs_complete();
        }
        break;
      }      
      default: {
        if(!inst_good) break;
        bool success = map_one_inst(sbPDG,sched);
        if(!success) {
          return false;
        } else {
          inst_good = !sched->insts_complete();
        }
        break;
      }      
    }
    if(!input_good && !output_good && !inst_good) break;
  }
  
  return sched->isComplete();
}

void SchedulerSimulatedAnnealing::unmap_one_input(SbPDG* sbPDG, Schedule* sched) {
  int n = sbPDG->num_vec_input();
  if(DEBUG_SCHED) cout << "unmap_one_input (" 
                       << sched->num_inputs_mapped() << " left)\n";

  int p = rand_bt(0,n);
  for(int i = 0; i < n; ++i) {
    if(++p ==n) {p=0;}

    SbPDG_VecInput* vec_in = sbPDG->vec_in(p);
    if(sched->vecMapped(vec_in)) {
      //cout << "I" << vec_in->name();
      sched->unassign_input_vec(vec_in);
      return;
    }
  }
  assert(0);
}

void SchedulerSimulatedAnnealing::unmap_one_output(SbPDG* sbPDG, Schedule* sched) {
  int n = sbPDG->num_vec_output();
  if(DEBUG_SCHED) cout << "unmap_one_output (" 
                       << sched->num_outputs_mapped() << " left)\n";

  int p = rand_bt(0,n);
  for(int i = 0; i < n; ++i) {
    if(++p ==n) {p=0;}
    SbPDG_VecOutput* vec_out = sbPDG->vec_out(p);
    if(sched->vecMapped(vec_out)) {
      //cout << "O" << vec_out->name();
      sched->unassign_output_vec(vec_out);
      return;
    }
  }
  assert(0);
}

void SchedulerSimulatedAnnealing::unmap_one_inst(SbPDG* sbPDG, Schedule* sched) {
  std::vector<SbPDG_Inst*>& inst_vec = sbPDG->inst_vec();
  int n = inst_vec.size();
  if(DEBUG_SCHED) cout << "unmap_one_inst(" 
                       << sched->num_insts_mapped() << " left)\n";

  int p = rand_bt(0,n);
  for(int i = 0; i < n; ++i) {
    if(++p ==n) {p=0;}
    SbPDG_Inst* inst = inst_vec[p];
    if(sched->isScheduled(inst)) {
      //cout << "V" << inst->name();
      sched->unassign_pdgnode(inst);

      //Error Checking : TODO: make function or remove these two loops
      SbPDG_Inst* n = inst;
      for(auto I=n->ops_begin(), E=n->ops_end();I!=E;++I) {
        if(*I == NULL) { continue; } //could be immediate
        SbPDG_Edge* source_pdgedge = (*I);
        if(sched->link_count(source_pdgedge)!=0) {
           cerr << "Edge: " << source_pdgedge->name() << " is already routed!\n"; 
           assert(0);
        }
      }

      for(auto Iu=n->uses_begin(), Eu=n->uses_end();Iu!=Eu;++Iu) {
        SbPDG_Edge* use_pdgedge = (*Iu);
        if(sched->link_count(use_pdgedge)!=0) {
           cerr << "Edge: " << use_pdgedge->name() << " is already routed!\n"; 
           assert(0);
        }
      }

      return;
    }
  }
  assert(0);
}

void SchedulerSimulatedAnnealing::unmap_some(SbPDG* sbPDG, Schedule* sched) {
  int num_to_unmap=3;

  while(num_to_unmap && sched->num_mapped()) {
    int r = rand_bt(0,5); //upper limit defines ratio of input/output scheduling
    switch(r) {
      case 0: if(sched->num_inputs_mapped() !=0) {
                unmap_one_input(sbPDG,sched); num_to_unmap--;
              }
      break;
      case 1: if(sched->num_outputs_mapped() !=0) {
                unmap_one_output(sbPDG,sched); num_to_unmap--;
              }
      break;
      default: if(sched->num_insts_mapped() !=0) {
                unmap_one_inst(sbPDG,sched); num_to_unmap--;
              }
    }
  }
  //cout << " ";
}



bool SchedulerSimulatedAnnealing::schedule_internal(SbPDG* sbPDG, Schedule*& sched) {
  int max_retries = 100;

  for(int t = 0; t < max_retries; ++t) {
    unmap_some(sbPDG,sched);

    map_to_completion(sbPDG,sched);
    if(sched->isComplete()) {
      //cout << "remppaed after " << t << " tries" << "\n";
      return true;
    } 
  } 
  //cout << "failed to remap\n";
  return false;
}

bool SchedulerSimulatedAnnealing::scheduleNode(Schedule* sched, 
    SbPDG_Node* pdgnode){
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

      curRouting->clear();
      pair<int,int> curScore = scheduleHere(sched, pdgnode, cand_spot,
                                            *curRouting, bestScore);

      if(curScore<fscore) {
        applyRouting(sched,pdgnode,cand_spot,curRouting);
        int lat=MAX_ROUTE,latmis=MAX_ROUTE; 
        curScore = obj(sched,lat,latmis);
        sched->unassign_pdgnode(pdgnode); //rip it up!
      }

      if(curScore < bestScore) {
        bestScore=curScore;
        bestspot=cand_spot;
        std::swap(bestRouting,curRouting);
      }

    }//for loop -- check for all sbnode spots

    if(bestScore < fscore) {
      applyRouting(sched,pdgnode,bestspot,bestRouting);
    } else {
      return false;
    }  

    return true;
  }
}

std::pair<int,int> SchedulerSimulatedAnnealing::scheduleHere(Schedule* sched, 
    SbPDG_Node* n, SB_CONFIG::sbnode* here,
    CandidateRouting& candRouting, std::pair<int,int> bestScore) {

  bestScore.first=MAX_ROUTE;
  bestScore.second=MAX_ROUTE;
  pair<int,int> score=make_pair(0,0);

  //cout << "Schedule Here "  << n->name() << " to here: " << here->name() << "\n";

  for(auto I=n->ops_begin(), E=n->ops_end();I!=E;++I) {
    if(*I == NULL) { continue; } //could be immediate
    SbPDG_Edge* source_pdgedge = (*I);
    SbPDG_Node* source_pdgnode = source_pdgedge->def();     //could be input node also

    if(sched->link_count(source_pdgedge)!=0) {
       cerr << "Edge: " << source_pdgedge->name() << " is already routed!\n"; 
       assert(0);
    }

    //cout << " try " << source_pdgedge->name() << " " 
    //     << source_pdgedge << " " << here->name() << " " 
    //     << score.first << "," << score.second << "\n";

    //route edge if source pdgnode is scheduled
    if(sched->isScheduled(source_pdgnode)) {
      sbnode* source_loc = sched->locationOf(source_pdgnode); //scheduled location

      //route using source node, sbnode
      pair<int,int> tempScore = route(sched, source_pdgedge, 
          source_loc, here,candRouting,bestScore-score);
      score = score + tempScore;

      //if(score>bestScore) return fscore;
    }
  }

  for(auto Iu=n->uses_begin(), Eu=n->uses_end();Iu!=Eu;++Iu) {
    SbPDG_Edge* use_pdgedge = (*Iu);
    SbPDG_Node* use_pdgnode = use_pdgedge->use();

    if(sched->link_count(use_pdgedge)!=0) {
       cerr << "Edge: " << use_pdgedge->name() << " is already routed!\n"; 
       assert(0);
    }

     //cout << " try " <<  n->name() << " " << here->name() << " " << score.first << " " << score.second << "\n";

    //route edge if source pdgnode is scheduled
    if(sched->isScheduled(use_pdgnode)) {
      sbnode* use_loc = sched->locationOf(use_pdgnode);

      //cout << "  routed\n";

      pair<int,int> tempScore = route(sched, use_pdgedge, here, use_loc,candRouting,bestScore-score);
      score = score + tempScore;
      //if(score>bestScore) return score;
    }
  }

  return score;
}

int SchedulerSimulatedAnnealing::routing_cost(SbPDG_Edge* edge, sblink* link, 
      Schedule* sched, CandidateRouting& candRouting, sbnode* dest) {

  SbPDG_Node* pdgnode = edge->def();
  sbnode* next = link->dest();

  //check if connection is closed..
  int t_cost = sched->temporal_cost(link, pdgnode);
  if(t_cost==1) { //empty
    if(candRouting.routing.count(link)) {
      bool found_match=false;
      for(SbPDG_Edge* edge : candRouting.routing[link]) {
        SbPDG_Node* cur_node = edge->def();
        if(cur_node == pdgnode) {
          found_match=true;
          break;
        }
      }
      if(found_match) t_cost=0;
      else t_cost=2;
    }
  }
  //if(t_cost==2) return -1;
  bool is_dest=(next==dest);

  sbfu* fu = dynamic_cast<sbfu*>(next);
  if(fu && sched->pdgNodeOf(fu) && !is_dest) return -1;  //stop if run into fu

  if(t_cost==0) { 
    return 0; //free link because schedule or candidate routing already maps
  } else {
    if(fu && sched->isPassthrough(fu)) return -1; //someone else's pass through
  
    bool passthrough = (fu && !is_dest);
//    int passthrough_cost = 100 / 10; 
    return 1+passthrough*1000;
  }
}


pair<int,int> SchedulerSimulatedAnnealing::route(Schedule* sched, 
    SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest, 
    CandidateRouting& candRouting, pair<int,int> scoreLeft) {

  pair<int,int> score = route_minimizeDistance(sched, pdgedge, source, dest, candRouting, scoreLeft);
    return score;
}



