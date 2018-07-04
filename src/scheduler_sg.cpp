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

void SchedulerStochasticGreedy::initialize(SbPDG* sbPDG, Schedule*& sched) {
  //Sort the input ports once
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  sbio_interface& si =  subModel->io_interf();
  _sd_in.clear();
  si.sort_in_vports(_sd_in);
  si.sort_out_vports(_sd_out);

  sbPDG->sort_vec_in();
  sbPDG->sort_vec_out();
}


bool SchedulerStochasticGreedy::schedule(SbPDG* sbPDG, Schedule*& sched) {  
  initialize(sbPDG,sched);

  static int _srand_no=1; 

  int max_iters_no_improvement = 100000000;
  srand(++_srand_no);

  Schedule* cur_sched = NULL;
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

  int remapNeeded = sbPDG->remappingNeeded(hw_num_fu); //setup remap structres 
  int iter = 0;
  while (iter < _max_iters) {
    if( (iter & (256-1)) == 0) {
      if( total_msec() > _reslim * 1000) {
        break;
      }
    }

    if(remapNeeded) { //remap every so often to try new possible dummy positions
      if( (iter & (16-1)) == 0) {
        sbPDG->remap(hw_num_fu);
        postsize = sbPDG->num_insts();
      }
    }

    bool succeed_sched = schedule_internal(sbPDG, cur_sched);
    int num_left = cur_sched->num_left(); 

    bool succeed_timing = false;
    int lat = MAX_ROUTE, latmis = MAX_ROUTE;
    if (succeed_sched) { 
      succeed_timing = cur_sched->fixLatency(lat,latmis);
    } else {
      latmis = MAX_ROUTE;
    }

    int violation = cur_sched->violation();

    int obj = lat;
    if(_integrate_timing) { 
      //obj = latmis*256+lat;
      //obj = violation*8192+lat*32+latmis;
      obj = latmis*10000+violation*100+lat;
    }

    std::pair<int, int> score = 
      make_pair(succeed_sched + succeed_timing-num_left, -obj);
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
                num_left, lat, violation, latmis, obj,
                cur_sched->num_inputs_mapped(),  sbPDG->num_inputs(),
                cur_sched->num_outputs_mapped(), sbPDG->num_outputs(),
                cur_sched->num_insts_mapped(),  presize, postsize,
                cur_sched->num_links_mapped(),
                cur_sched->num_edge_links_mapped(),
                succeed_sched ? ", all mapped" : "",
                succeed_timing ? ", mismatch == 0" : "");
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
    if(violation ==0 && iter > _max_iters_zero_vio) {
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

//  if(verbose) {
//    if(best_mapped && !best_succeeded) {
//      fprintf(stderr,"DFG Mapped, but Timing not met.  Simulation is still possible, but do not use generated schedule for hardware!");
//    }
//  }

//  return best_succeeded;
  return best_mapped;
}


bool can_go(SbPDG_Inst* n, unordered_map<SbPDG_Inst*,int>& state) {
  if(!n || state[n]==1) {
    return false;
  }
//  return true;
  for(auto II=n->ops_begin(), EE=n->ops_end(); II!=EE; ++II) {
    if(*II == 0) continue;
    if(SbPDG_Inst* inc_node = dynamic_cast<SbPDG_Inst*>((*II)->def())) {
      if(state[inc_node]!=2) {
        return false;
      }
    }
  }
  state[n]=1;
  return true;
}

bool SchedulerStochasticGreedy::schedule_internal(SbPDG* sbPDG, Schedule*& sched) {
  sched = new Schedule(getSBModel(),sbPDG);

  bool vec_in_assigned = assignVectorInputs(sbPDG,sched);
  if(!vec_in_assigned) {
    return false;
  }

  unordered_map<SbPDG_Inst*,int> state;
  bool schedule_okay=true; 

  list<SbPDG_Inst* > openset;
  SbPDG::const_input_iterator I,E;

  //pdg input nodes
  for(I=sbPDG->input_begin(),E=sbPDG->input_end();I!=E;++I) {
    SbPDG_Input* n = *I;

    for(auto I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(can_go(use_pdginst,state)) {
        openset.push_back(use_pdginst);
      }
    }
  }

  //populate the schedule object
  while(!openset.empty()) {
    int i = rand()%openset.size();
    list<SbPDG_Inst*>::iterator it = openset.begin();
    std::advance(it,i);
    
    SbPDG_Inst* n = *it; 
    openset.erase(it); 

    //SbPDG_Inst* n = openset.front(); 
    //openset.pop_front();

    schedule_okay&=scheduleNode(sched,n);
    if(!schedule_okay) return false;

    state[n]=2;

    if(_integrate_timing && sched->max_lat_mis() > _best_latmis) {
      return false;
    }

    /*if(_integrate_timing && sched->violation() > _best_violation) {
      return false;
    }*/

    for(auto I=n->uses_begin(), E=n->uses_end();I!=E;++I) {
      SbPDG_Inst* use_pdginst = dynamic_cast<SbPDG_Inst*>((*I)->use());
      if(can_go(use_pdginst,state)) {
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
      bestRouting->clear();

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
      curScore.first = curScore.second;

      if(curScore < fscore) { //if not failing score
        if(_integrate_timing) {

          int min_node_lat, max_node_lat;
          curRouting->fill_lat(sched, min_node_lat, max_node_lat);

          int violation = min_node_lat-max_node_lat;

          if(_integrate_timing && violation > _best_latmis) {
            curScore=fscore; // FAIL
          }


          //if(_strict_timing && violation > 0) {
          //  curScore=fscore; // FAIL
          //}
          curScore.second = -violation;
        }
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
    SbPDG_Node* n, SB_CONFIG::sbnode* here, CandidateRouting& candRouting, 
    std::pair<int,int> bestScore) {

  pair<int,int> score=make_pair(0,0);

  for(auto I=n->ops_begin(), E=n->ops_end();I!=E;++I) {
    if(*I == NULL) { continue; } //could be immediate
    SbPDG_Edge* source_pdgedge = (*I);
    SbPDG_Node* source_pdgnode = source_pdgedge->def();     //could be input node also

    //route edge if source pdgnode is scheduled
    if(sched->isScheduled(source_pdgnode)) {
      sbnode* source_loc = sched->locationOf(source_pdgnode); //scheduled location

      //route using source node, sbnode
      pair<int,int> tempScore = route(sched, source_pdgedge, source_loc, here,candRouting,bestScore-score);
      score = score + tempScore;
      //cout << source_pdgedge->name() << " " 
      //  << source_pdgedge << " " << here->name() << " " 
      //     << score.first << "," << score.second << "\n";
      if(score>bestScore) return fscore; //fail if score is worse (higher)
    }
  }

  for(auto Iu=n->uses_begin(), Eu=n->uses_end();Iu!=Eu;++Iu) {
    SbPDG_Edge* use_pdgedge = (*Iu);
    SbPDG_Node* use_pdgnode = use_pdgedge->use();

    //route edge if source pdgnode is scheduled
    if(sched->isScheduled(use_pdgnode)) {
      sbnode* use_loc = sched->locationOf(use_pdgnode);

      pair<int,int> tempScore = route(sched, use_pdgedge, here, use_loc,candRouting,bestScore-score);
      score = score + tempScore;
      //cout << n->name() << " " << here->name() << " " << score << "\n";
      if(score>bestScore) return score; //fail if score is worse (higher)
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
  sbio_interface& si =  subModel->io_interf();

  int n = sbPDG->num_vec_input();

  for(int j = 0; j < n; ++j) {
    SbPDG_VecInput* vec_in = sbPDG->vec_in(j);
    //cout << "Assigning Vector Port:" << vec_in->gamsName() <<"\n";

    bool found_vector_port = false;

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
      }
      #endif

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
    } while ((attempt++ < _sd_in.size() - index -1) && !found_vector_port);

    if(!found_vector_port) {
      //cout << "Could not find Input hardware vector port\n";
      return false;
    }
  }
  return true;
}

void SchedulerStochasticGreedy::findFirstIndex(vector<pair<int,int>>& sd, sbio_interface& si, 
    unsigned int numIO, unsigned int& index, bool is_input) {
  for (; index < sd.size(); index++) {
    pair<int,int> j = sd[index];
    int vn = j.first; 
    vector<pair<int, vector<int>>>& vdesc = (!is_input ? si.getDesc_O(vn) : si.getDesc_I(vn));
    //Check if the vetcor port is 1. big enough
    if (numIO <= vdesc.size()) { break;}
  }
}

void SchedulerStochasticGreedy::genRandomIndexBW(pair<bool, int>& vport_id, vector<pair<int, vector<int>>>& vport_desc,  vector<pair<int,int>>& sd, sbio_interface& si, unsigned int size, unsigned int index, Schedule*& sched, bool is_input) {
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



bool SchedulerStochasticGreedy::assignVectorOutputs(SbPDG* sbPDG, Schedule* sched) {
  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  CandidateRouting candRouting;  
  sbio_interface& si =  subModel->io_interf();

  for(int i = 0; i < sbPDG->num_vec_output(); ++i) {
    SbPDG_VecOutput* vec_out = sbPDG->vec_out(i);
    //cout << "Assigning Vector Port:" << vec_out->gamsName() <<"\n";

    bool found_vector_port = false;

    //for (auto& j : sd)
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
        num++;
      }
#endif
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
    } while (attempt++ < _sd_out.size() - index - 1 && !found_vector_port);
    applyRouting(sched, &candRouting); //Commit the routing

    if (!found_vector_port) {
      //cout << "Could not find output hardware vector port\n";
      return false;
    }

    //*** Accounting for timing
    int min_vec_lat, max_vec_lat;
    candRouting.fill_lat(sched,min_vec_lat,max_vec_lat);
   
    if(min_vec_lat > max_vec_lat) {
      sched->add_violation(min_vec_lat-max_vec_lat);
      //max_node_lat = min_node_lat;
    }
    for(unsigned m=0; m < vec_out->num_outputs(); ++m) {
      SbPDG_Output* pdgout = vec_out->getOutput(m);
      sched->assign_lat_bounds(pdgout,min_vec_lat,max_vec_lat);
      sched->record_violation(pdgout,min_vec_lat-max_vec_lat);
    }

  }
  return true;
}
