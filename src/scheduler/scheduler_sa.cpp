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

//Major things left to try that might improve the algorithm:
//1. Overprovisioning while routing/placing 
//    (maybe also the including actual simulated annealing part)
//2. Prioritizing nodes based on what their presumed effect on scheduling
//   quality might be.  This may be difficult to gauge though.

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

std::pair<int, int> SchedulerSimulatedAnnealing::obj( Schedule*& sched, 
    int& lat, int& latmis, int& ovr, int& agg_ovr, int& max_util) {  
    int num_left = sched->num_left(); 
    bool succeed_sched = (num_left==0);

    sched->get_overprov(ovr,agg_ovr,max_util);
     assert(agg_ovr>=0);

    bool succeed_ovr = (ovr ==0);

    bool succeed_timing = false;
    if (succeed_sched) { 
      succeed_timing = sched->fixLatency(lat,latmis);
    } else {
      latmis = 1000;
      lat = 1000;
    }

    int violation = sched->violation();

    int obj = agg_ovr*100000 + latmis*10000+violation*100+lat + max_util;

//        fprintf(stdout, "objective rt:%d, left: %3d, " 
//                "lat: %3d, vio %d, mis: %d, ovr: %d, util: %d, "
//                "obj:%d, ins: %d/%d, outs: %d/%d,"
//                " insts: %d, links:%d, edge-links:%d  %s%s\n", 
//                _route_times,
//                sched->num_left(), lat, 
//                sched->violation(), latmis, ovr, max_util, obj,
//                sched->num_inputs_mapped(),  sched->sbpdg()->num_inputs(),
//                sched->num_outputs_mapped(), sched->sbpdg()->num_outputs(),
//                sched->num_insts_mapped(),  
//                sched->num_links_mapped(),
//                sched->num_edge_links_mapped(),
//                succeed_sched ? ", all mapped" : "",
//                succeed_timing ? ", mismatch == 0" : "");

//    sched->printGraphviz("hi.gv");

    //succeed_timing + succeed_ovr
    return make_pair( succeed_sched  -num_left, -obj);
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
    if( (total_msec() > _reslim * 1000) || _should_stop ) {
      break;
    }

    bool print_stat=false;
    if( (iter & (256-1)) == 0) {
      print_stat=true;
    }

    if( (iter & (4096-1)) == 0) {
      //Every so often, lets give up and start over from scratch
      delete cur_sched;
      cur_sched = new Schedule(getSBModel(),sbPDG);
     
      if(remapNeeded) { //remap every so often to try new possible dummy positions
        sbPDG->remap(hw_num_fu);
        postsize = sbPDG->num_insts();
        cur_sched->allocate_space();
      }
    }

    // every so often you can reset?
    if( (iter & (512-1)) == 1) {
      *cur_sched = *sched; //shallow copy of sched should work?

    }


    bool succeed_sched = schedule_internal(sbPDG, cur_sched);

    int lat=INT_MAX,latmis=INT_MAX,agg_ovr=INT_MAX,ovr=INT_MAX,max_util=INT_MAX;
    std::pair<int,int> score = obj(cur_sched,lat,latmis,ovr,agg_ovr,max_util);

    int succeed_timing = (latmis ==0) && (ovr ==0);

    if (verbose && ((score > best_score) || print_stat)) {
      stringstream ss;
      ss << "viz/iter/" << iter << ".gv";
      cur_sched->printGraphviz(ss.str().c_str());

      for(int i = 0; i < sbPDG->num_vec_input(); ++i) {
        cout << cur_sched->vecPortOf(sbPDG->vec_in(i)).second << " ";
      }

      fprintf(stdout, "Iter: %4d, time:%0.2f, rt:%d, left: %3d, " 
              "lat: %3d, vio %d, mis: %d, ovr: %d, agg_ovr: %d, util: %d, "
              "obj:%d, ins: %d/%d, outs: %d/%d,"
              " insts: %d/%d,%d, links:%d, edge-links:%d  %s%s", 
              iter, total_msec()/1000.f, _route_times,
              cur_sched->num_left(), lat, 
              cur_sched->violation(), latmis, ovr, agg_ovr, 
              max_util, -score.second,
              cur_sched->num_inputs_mapped(),  sbPDG->num_inputs(),
              cur_sched->num_outputs_mapped(), sbPDG->num_outputs(),
              cur_sched->num_insts_mapped(),  presize, postsize,
              cur_sched->num_links_mapped(),
              cur_sched->num_edge_links_mapped(),
              succeed_sched ? ", all mapped" : "",
              succeed_timing ? ", mismatch == 0" : "");
      if(score > best_score) {
        cout << "\n";
      } else {
        cout.flush();
        cout << "\r";
      }
    }


    if (score > best_score) {
      sched->printGraphviz("viz/cur-best.gv");

      if(remapNeeded) {
        sbPDG->printGraphviz("viz/remap.dot");
        best_dummies = sbPDG->getDummiesOutputs();
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

  if(cur_sched) {
    delete cur_sched;
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

bool SchedulerSimulatedAnnealing::genRandomIndexBW(pair<bool, int>& vport_id, vector<pair<int, vector<int>>>& vport_desc,  vector<pair<int,int>>& sd, sbio_interface& si, unsigned int size, unsigned int index, Schedule*& sched, bool is_input) {
  unsigned int k;
  pair<int, int> p;
  int vport_num;
  unsigned int rep = 0;
  bool success=false;

  while (!success && rep++ < size*2) {
    k = rand() % (size - index) + index;
    p = sd[k];
    vport_num = p.first;
    vport_id = std::make_pair(is_input,vport_num);
    
    success = sched->vportOf(vport_id) == NULL;

    vport_desc = (!is_input ? si.getDesc_O(vport_num) : si.getDesc_I(vport_num));
  }

  return success;
}

bool SchedulerSimulatedAnnealing::schedule_input( SbPDG_VecInput*  vec_in, 
    SbPDG* sbPDG, Schedule* sched) {

  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  sbio_interface& si =  subModel->io_interf();
  int n_vertex = vec_in->num_inputs();
  int n_vertex_physical = n_vertex;

  if(vec_in->is_temporal()) {
    n_vertex_physical=1;  // temporal vectors are always scheduled to one node
  }

  unsigned int index = 0;
  int num_found=0;
  //use physical number of vertices to decide a port
  findFirstIndex(_sd_in, si, n_vertex_physical, index, true /*input*/);
  unsigned int attempt = 0;

  vector<int> order; //temp variable used for randomly iterating

  CandidateRouting r1,r2; //do this so that function scope de-allocates these
  CandidateRouting* bestRouting = &r1, * candRouting = &r2;  

  vector<bool> bestMask;
  pair<int,int> bestScore = std::make_pair(INT_MIN,INT_MIN);
  pair<bool, int> bestVportid;
  std::vector<sbnode*> bestInputs;

  while (num_found < 8 &&  (attempt++ < _sd_in.size())) {
    pair<bool, int> vport_id;
    vector<pair<int, vector<int>>> vport_desc;


    //TODO: put this code in schedule_output as well
    bool found=false;
    while(!found) {
      if(!genRandomIndexBW(vport_id, vport_desc, _sd_in, si, 
                       _sd_in.size(), index, sched, true /*input*/)) {
        unmap_one_input(sbPDG,sched);
      } else {
        found=true;
      } 
    }


    candRouting->clear();  
    std::vector<sbnode*> possInputs;

    for (unsigned m=0; m < vport_desc.size(); m++) {
      int cgra_port_num = vport_desc[m].first;
      sbinput* in = subModel->get_input(cgra_port_num);
      if (sched->pdgNodeOf(in) == NULL) {
        possInputs.push_back(in);
      }
    }


    if((int)possInputs.size() < n_vertex_physical) continue;

    vector<sbnode*> candInputs;
    rand_node_choose_k(n_vertex_physical, possInputs, candInputs);

    bool ports_okay_to_use=true;
    random_order(n_vertex,order);
    int num_links_used=0;
    for(int i : order) {
      //In a temporal region, just select the 0th input
      int cand_index = vec_in->is_temporal() ? 0 : i;
      sbnode* node = candInputs[cand_index];
      SbPDG_Node* vertex = vec_in->getInput(i);

      pair<int,int> n_score = scheduleHere(sched, vertex, node, 
                                           *candRouting, fscore);
      if(n_score>=fscore) {
        ports_okay_to_use=false;
        break;
      }

      num_links_used+=n_score.second;
    }

    if (!ports_okay_to_use) continue;
    num_found++;

    applyRouting(sched,candRouting);
    for(int i = 0; i < n_vertex; ++i) {
       int cand_index = vec_in->is_temporal() ? 0 : i;
       sbnode* node = candInputs[cand_index];
       sched->assign_node(vec_in->getInput(i),node);    
    }

    int lat=INT_MAX,latmis=INT_MAX,agg_ovr=INT_MAX,ovr=INT_MAX,max_util=INT_MAX;
    pair<int,int> candScore = obj(sched,lat,latmis,ovr,agg_ovr,max_util);
    candScore.second-=num_links_used;

    //TODO: does this help at all?
    int hw_port_size = vport_desc.size();
    int extra = hw_port_size - vec_in->num_inputs();
    candScore.second-=extra*10;

    sched->unassign_input_vec(vec_in);

    if(candScore > bestScore) {
      //cout << candScore.first << " " << candScore.second <<"\n";
      //cout << "lat: " << lat << " latmis: " << latmis 
      //    << " ovr " << ovr << " num links used " << num_links_used << "\n";
      bestScore=candScore;
      bestInputs=candInputs;
      bestVportid=vport_id;
      std::swap(bestRouting,candRouting);
      bestMask.clear();
      bestMask.resize(vport_desc.size());
      int num_cgra_ports=0; //for debugging
      for (int m=0; m < (int)vport_desc.size(); m++) {
        int cgra_port_num = vport_desc[m].first;
        for(int i = 0; i < (int)candInputs.size(); ++i) {
          if(static_cast<sbinput*>(candInputs[i])->port()==cgra_port_num) {
            assert(bestMask[m]==false);
            bestMask[m]=true;
            num_cgra_ports++;
            break;
          }
        }
      }
      //assert(n_vertex == num_cgra_ports);
    }
  }
  //cout << " -- \n";

  if(num_found > 0) {
    for(int i = 0; i < n_vertex; ++i) {
       int cand_index = vec_in->is_temporal() ? 0 : i;
       sbnode* node = bestInputs[cand_index];
       sched->assign_node(vec_in->getInput(i),node);    
    }
    sched->assign_vport(vec_in, bestVportid, bestMask);
    applyRouting(sched, bestRouting); //Commit the routing
  }
  return num_found>0;
}

bool SchedulerSimulatedAnnealing::schedule_output( SbPDG_VecOutput*  vec_out, 
    SbPDG* sbPDG, Schedule* sched) {

  SB_CONFIG::SubModel* subModel = _sbModel->subModel();
  sbio_interface& si =  subModel->io_interf();
  int n_vertex = vec_out->num_outputs();
  int n_vertex_physical = n_vertex;

  if(vec_out->is_temporal()) {
    n_vertex_physical=1;  // temporal vectors are always scheduled to one node
  }

  unsigned int index = 0;
  int num_found=0;
  findFirstIndex(_sd_out, si, n_vertex_physical, index, false /*output*/);
  unsigned int attempt = 0;

  vector<int> order; //temp variable used for randomly iterating

  CandidateRouting r1,r2;
  CandidateRouting* bestRouting = &r1, * candRouting = &r2;  

  vector<bool> bestMask;
  pair<int,int> bestScore = std::make_pair(INT_MIN,INT_MIN);
  pair<bool, int> bestVportid;
  std::vector<sbnode*> bestOutputs;


  while (num_found < 10 &&  (attempt++ < _sd_out.size())) {
    pair<bool, int> vport_id;
    vector<pair<int, vector<int>>> vport_desc;
    if(!genRandomIndexBW(vport_id, vport_desc, _sd_out, si, 
                     _sd_out.size(), index, sched, false /*output*/)) {
      return false;
    }

    candRouting->clear();  
    std::vector<sbnode*> possOutputs;

    for (unsigned m=0; m < vport_desc.size(); m++) {
      int cgra_port_num = vport_desc[m].first;
      sboutput* out = subModel->get_output(cgra_port_num);
      if (sched->pdgNodeOf(out) == NULL) {
        possOutputs.push_back(out);
      }
    }

    if((int)possOutputs.size() < n_vertex_physical) continue;

    vector<sbnode*> candOutputs;
    rand_node_choose_k(n_vertex_physical, possOutputs, candOutputs);

    bool ports_okay_to_use=true;
    random_order(n_vertex,order);
    int num_links_used=0;
    for(int i : order) {
      int cand_index = vec_out->is_temporal() ? 0 : i;
      sbnode* node = candOutputs[cand_index];
      SbPDG_Node* vertex = vec_out->getOutput(i);

      pair<int,int> n_score = scheduleHere(sched, vertex, node, 
                                           *candRouting, fscore);
      if(n_score>=fscore) {
        ports_okay_to_use=false;
        break;
      }

      num_links_used+=n_score.second;
    }

    if (!ports_okay_to_use) continue;
    num_found++;

    applyRouting(sched,candRouting);
    for(int i = 0; i < n_vertex; ++i) {
       int cand_index = vec_out->is_temporal() ? 0 : i;
       sbnode* node = candOutputs[cand_index];
       sched->assign_node(vec_out->getOutput(i),node);    
    }
    
    int lat=INT_MAX,latmis=INT_MAX,ovr=INT_MAX,agg_ovr=INT_MAX,max_util=INT_MAX;
    pair<int,int> candScore = obj(sched,lat,latmis,ovr,agg_ovr,max_util);
    candScore.second-=num_links_used;
    sched->unassign_output_vec(vec_out);

    if(candScore > bestScore) {
      //cout << candScore.first << " " << candScore.second <<"\n";
      //cout << "lat: " << lat << " latmis: " << latmis 
      //    << " ovr " << ovr << " num links used " << num_links_used << "\n";
      bestScore=candScore;
      bestOutputs=candOutputs;
      bestVportid=vport_id;
      std::swap(bestRouting,candRouting);
      bestMask.clear();
      bestMask.resize(vport_desc.size());
      int num_cgra_ports=0; //for debugging
      for (int m=0; m < (int)vport_desc.size(); m++) {
        int cgra_port_num = vport_desc[m].first;
        for(int i = 0; i < (int)candOutputs.size(); ++i) {
          if(static_cast<sboutput*>(candOutputs[i])->port()==cgra_port_num) {
            bestMask[m]=true;
            num_cgra_ports++;
            break;
          }
        }
      }
      //assert(n_vertex == num_cgra_ports);
    }
  }
  //cout << " -- \n";
  if(num_found > 0) {
    for(int i = 0; i < n_vertex; ++i) { 
       int cand_index = vec_out->is_temporal() ? 0 : i;
       sbnode* node = bestOutputs[cand_index];
       sched->assign_node(vec_out->getOutput(i),node);    
    }

    sched->assign_vport(vec_out,bestVportid,bestMask);
    applyRouting(sched, bestRouting); //Commit the routing
  }
  return num_found>0;
}

bool SchedulerSimulatedAnnealing::map_one_input(SbPDG* sbPDG, Schedule* sched) {
  if(DEBUG_SCHED) cout << "map_one_input (" 
                       << sched->num_inputs_mapped() << " mapped)\n";

  int n = sbPDG->num_vec_input();

  int p = rand_bt(0,n);
  for(int i = 0; i < n; ++i) {
    if(++p ==n) {p=0;}

    SbPDG_VecInput* vec_in = sbPDG->vec_in(p);
    if(sched->vecMapped(vec_in)) continue;    
    return schedule_input(vec_in,sbPDG,sched);
  }
  return false;
}

bool SchedulerSimulatedAnnealing::map_one_output(SbPDG* sbPDG, Schedule* sched) {
  if(DEBUG_SCHED) cout << "map_one_output (" 
                       << sched->num_outputs_mapped() << " mapped)\n";

   int n = sbPDG->num_vec_output();

  int p = rand_bt(0,n);
  for(int i = 0; i < n; ++i) {
    if(++p ==n) {p=0;}
    SbPDG_VecOutput* vec_out = sbPDG->vec_out(p);
    if(sched->vecMapped(vec_out)) continue;
    bool success = schedule_output(vec_out,sbPDG,sched);
    //if(!success) {
    //  cout << "failed with " << vec_out->name() << "\n";
    //}
    return success;
  }
  return false; 
}

bool SchedulerSimulatedAnnealing::map_one_inst(SbPDG* sbPDG, Schedule* sched) {
  if(DEBUG_SCHED) cout << "map_one_inst (" 
                       << sched->num_insts_mapped() << " mapped)\n";

  std::vector<SbPDG_Inst*>& inst_vec = sbPDG->inst_vec();
  int n = inst_vec.size();

  int p = rand_bt(0,n);
  for(int i = 0; i < n; ++i) {
    if(++p ==n) {p=0;}
    SbPDG_Inst* inst = inst_vec[p];
    if(sched->isScheduled(inst)) continue;

    //cout << "try to map this inst: " << inst->name() << "\n";

    bool success = scheduleNode(sched,inst);
    //if(!success) {
    //  cout << "success with " << inst->name() << "\n";
    //} else {
    //  cout << "failed with " << inst->name() << "\n";
    //}
    return success;
  }
  return false;
}

bool SchedulerSimulatedAnnealing::map_io_to_completion(SbPDG* sbPDG, 
    Schedule* sched) {

  while(!(sched->inputs_complete() && sched->outputs_complete())) {
    int r = rand_bt(0,2); //upper limit defines ratio of input/output scheduling
    switch(r) {
      case 0: {
        if(sched->inputs_complete()) break;
        bool success = map_one_input(sbPDG,sched);
        if(!success) return false;
        break;
      }
      case 1: {
        if(sched->outputs_complete()) break;
        bool success = map_one_output(sbPDG,sched);
        if(!success) return false;
        break;
      }      
      default: {assert(0); break;}      
    }
  }
  //cout << "Schedule Complete, mapped: " << sched->num_mapped() 
  //                          << "left:" << sched->num_left();
  
  return true;
}


bool SchedulerSimulatedAnnealing::map_to_completion(SbPDG* sbPDG, Schedule* sched) {

  if(DEBUG_SCHED) cout << "Map to completion! " << sched->num_mapped() << "\n";
  while(!sched->isComplete()) {
    int r = rand_bt(0,16); //upper limit defines ratio of input/output scheduling
    switch(r) {
      case 0: {
        if(sched->inputs_complete()) break;
        bool success = map_one_input(sbPDG,sched);
        //cout << "I";
        if(!success) {
          //cout << "IF";
          return false;
        }
        break;
      }
      case 1: {
        if(sched->outputs_complete()) break;
        bool success = map_one_output(sbPDG,sched);
        //cout << "O";
        if(!success) { 
          //cout << "OF";
          return false;
        }
        break;
      }      
      default: {
        if(sched->insts_complete()) break;
        bool success = map_one_inst(sbPDG,sched);
        //cout << "N";
        if(!success) { 
          //cout << "NF";
          return false;
        }
        break;
      }      
    }
  }
  //cout << "Schedule Complete, mapped: " << sched->num_mapped() 
  //                          << "left:" << sched->num_left();
  
  return true;
}

void SchedulerSimulatedAnnealing::unmap_one_input(SbPDG* sbPDG, Schedule* sched) {
  int n = sbPDG->num_vec_input();
  if(DEBUG_SCHED) cout << "unmap_one_input (" 
                       << sched->num_inputs_mapped() << " left)\n";


  int p = rand_bt(0,n);
  while(true) {
    for(int i = 0; i < n; ++i) {
      if(++p ==n) {p=0;}
  
      SbPDG_VecInput* vec_in = sbPDG->vec_in(p);
      if(sched->vecMapped(vec_in)) {
        //TODO: do this for outputs too, or just eliminate
        int hw_port_size = _sbModel->subModel()->io_interf().in_vports[sched->vecPortOf(vec_in).second].size();
        int extra = hw_port_size - vec_in->num_inputs();
        assert(extra>=0 && extra < 32); //don't expect this large of inputs
        int r = rand_bt(0,extra*extra+hw_port_size);
        //cout << "extra: " << extra << " for vport " << vec_in->name() << "\n";
        if(r<extra*extra+1) {
          sched->unassign_input_vec(vec_in);
          return;
        }
      }
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
      for(auto I=n->inc_e_begin(), E=n->inc_e_end();I!=E;++I) {
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

  //for(SbPDG_Edge* e : sbPDG->edges()) {
  //  if(sched->edge_links(e).size() > 8) {
  //    cout << e->name() << ":" <<  sched->edge_links(e).size() << " : ";
  //    for(auto* l : sched->edge_links(e)) {
  //      cout << l->name() << " ";
  //    }
  //    cout << "\n";
  //  }
  //}
  //cout << "\n";

  int num_to_unmap=1;
  int r = rand_bt(0,1000); //upper limit defines ratio of input/output scheduling
  if(r<5) { 
    num_to_unmap=10;
    //cout << "Z";
  } else if(r < 10) {
    num_to_unmap = 5;
    //cout << "X";
  } else if(r < 100) {
    //cout << "*";
    //clear all nodes with overprovisioning
    SB_CONFIG::SubModel* subModel = _sbModel->subModel();
    for(sbnode* n : subModel->node_list()) {
      if(sched->thingsAssigned(n) > 1) {
        auto vertices = sched->pdgNodesOf(n); //deliberately copy
        for(SbPDG_Node* v : vertices) {
          //cout << v->name() << "\n";
          //cout << v->group_id() << "\n";
          if(!v->is_temporal()) {
            sched->unassign_pdgnode(v);
          }
        }
      }
    }

    //for(auto I = sbPDG->nodes_begin(), E=sbPDG->nodes_end();I!=E;++I) {
    //  SbPDG_Node* v = *I;
    //  

    //}
  } //else {
    //cout << "u";
  //}

  while(num_to_unmap && sched->num_mapped()) {
    int r = rand_bt(0,8); //upper limit defines ratio of input/output scheduling
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

    if(_fake_it) {
      //Just map the i/o ports
      map_io_to_completion(sbPDG,sched);
      if(sched->inputs_complete() && sched->outputs_complete()) {
        return true;
      }
    } else {
      //THE REAL THING
      map_to_completion(sbPDG,sched);
      if(sched->isComplete()) {
        //cout << "remppaed after " << t << " tries" << "\n";
        return true;
      } 
    }
  } 
  //cout << "failed to remap\n";
  return false;
}

bool SchedulerSimulatedAnnealing::scheduleNode(Schedule* sched, 
    SbPDG_Node* pdgnode){
  std::vector<sbnode*> spots;

  CandidateRouting route1, route2;   
  CandidateRouting *bestRouting = &route1, *curRouting = &route2;


  if(SbPDG_Inst* pdginst= dynamic_cast<SbPDG_Inst*>(pdgnode))  { 
    fillInstSpots(sched, pdginst, spots);             //all possible candidates based on FU capability 
  } else if(SbPDG_Input* pdg_in = dynamic_cast<SbPDG_Input*>(pdgnode)) {
    fillInputSpots(sched,pdg_in,spots); 
  } else if(SbPDG_Output* pdg_out = dynamic_cast<SbPDG_Output*>(pdgnode)) {
    fillOutputSpots(sched,pdg_out,spots); 
  }

  //populate a scheduling score for each of canidate sbspot
  if ( (rand() % 1024) == 1) { //Pick a totally random spot
    //cout << "P";
    bool succ = false;
    unsigned int attempt = 0;
    while (!succ && attempt < spots.size()) {
      int r2 = rand() % spots.size();
      sbnode* cand_spot = spots[r2];

      std::pair<int,int> n_score = 
        scheduleHere(sched, pdgnode, cand_spot,*curRouting,fscore);
      bool failed = (n_score==fscore);

      if(!failed) {
        applyRouting(sched,pdgnode,cand_spot,curRouting);
        return true;
      } else {
        attempt++;
      }
    }
    return false; // couldn't find one

  } else {  
    std::pair<int,int> bestScore = make_pair(INT_MIN,INT_MIN);
    sbnode* bestspot = NULL;
    int num_found=0;

    for(unsigned i=0; i < spots.size(); i++) {
      sbnode* cand_spot = spots[i];

      curRouting->clear();
      std::pair<int,int> n_score = scheduleHere(sched, pdgnode, cand_spot,
                                                *curRouting, fscore);
 
      bool failed = (n_score==fscore);
      int routes = n_score.second;

      if(!failed) {
        num_found++;
        applyRouting(sched,pdgnode,cand_spot,curRouting);
    int lat=INT_MAX,latmis=INT_MAX,ovr=INT_MAX,agg_ovr=INT_MAX,max_util=INT_MAX;
        std::pair<int,int> curScore = obj(sched,lat,latmis,ovr,agg_ovr,max_util);
        curScore.second-=routes;
        sched->unassign_pdgnode(pdgnode); //rip it up!

        if(curScore > bestScore) {
          bestScore=curScore;
          bestspot=cand_spot;
          std::swap(bestRouting,curRouting);
        }
      }

    }

    if(num_found > 0) {
      applyRouting(sched,pdgnode,bestspot,bestRouting);
    }

    return (num_found > 0);
  }
}

std::pair<int,int> SchedulerSimulatedAnnealing::scheduleHere(Schedule* sched, 
    SbPDG_Node* n, SB_CONFIG::sbnode* here,
    CandidateRouting& candRouting, std::pair<int,int> bestScore) {

  bestScore.first=MAX_ROUTE;
  bestScore.second=MAX_ROUTE;
  pair<int,int> score=make_pair(0,0);

  //cout << "Schedule Here "  << n->name() << " to here: " << here->name() << "\n";

  //TODO: make work for decomp-CGRA
  for(auto I=n->inc_e_begin(), E=n->inc_e_end();I!=E;++I) {
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
      if(tempScore==fscore) {
        //cout << "failed to route def\n";
        return fscore;
      }

      score = score + tempScore;
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

      pair<int,int> tempScore = route(sched, use_pdgedge, here, use_loc,candRouting,bestScore-score);
      if(tempScore==fscore) {
        //cout << "failed to route use\n";
        return fscore;
      }
      score = score + tempScore;
    }
  }

  return score;
}

int SchedulerSimulatedAnnealing::routing_cost(SbPDG_Edge* edge, sblink* link, 
      Schedule* sched, CandidateRouting& candRouting, sbnode* dest) {

  SbPDG_Node* def_pdgnode = edge->def();
  SbPDG_Node* use_pdgnode = edge->use();

  bool is_in = def_pdgnode->type() == SbPDG_Node::V_INPUT;
  bool is_out = use_pdgnode->type() == SbPDG_Node::V_OUTPUT;
  bool is_io = is_in||is_out;

  bool is_temporal = def_pdgnode->is_temporal();
  bool is_temporal_inst = is_temporal && !is_io;
  bool is_temporal_in   = is_temporal && is_in;
  bool is_temporal_out  = is_temporal && is_out;


  //For now, links only route on their own network
  //ie. temporal_io and non-temporal route on dedicated network
  if(!is_temporal_inst && link->max_util()>1) return -1;
  if(is_temporal_inst && link->max_util()<=1) return -1;

  sbnode* next = link->dest();

  //check if connection is closed..
  //0: free
  //1: empty
  //2: already there
  int t_cost;
  if(is_temporal_in) {
    SbPDG_Input* in = static_cast<SbPDG_Input*>(def_pdgnode);
    t_cost = sched->temporal_cost_in(link, in->input_vec());
  } else if(is_temporal_out) {
    SbPDG_Output* out = static_cast<SbPDG_Output*>(use_pdgnode);
    t_cost = sched->temporal_cost_out(link, def_pdgnode,
                                      out->output_vec());
  } else { //NORMAL CASE!
    t_cost = sched->temporal_cost(link, def_pdgnode);
  }
  if(t_cost!=0) { //empty
    auto iter = candRouting.routing.find(link);
    if(iter != candRouting.routing.end()) {
      auto& edges = iter->second; 
      bool found_match=false;
      for(SbPDG_Edge* edge : edges) {
        SbPDG_Node* cur_node = edge->def();
        if(cur_node == def_pdgnode) {
          found_match=true;
          break;
        }
        //kind of horribly ugly
        if(is_temporal_in) {
          if(sched->input_matching_vector(cur_node,
             static_cast<SbPDG_Input*>(def_pdgnode)->input_vec()) ||
             sched->output_matching_vector(cur_node,
              static_cast<SbPDG_Output*>(use_pdgnode)->output_vec()) ){
              found_match=true;
              break;
          }
        }
        if(is_temporal_out) {

        }
      }
      if(found_match) t_cost=0;
      else if (t_cost==1) t_cost=1+edges.size(); //empty before
      else t_cost+=edges.size();
    }
  }

  if(t_cost>=2) {
    if(!is_temporal_inst) {
      return (t_cost)*(t_cost)*10;
    } 
  }
  bool is_dest=(next==dest);

  sbfu* fu = dynamic_cast<sbfu*>(next);
  if(fu && sched->pdgNodeOf(fu) && !is_dest) return -1;  //stop if run into fu

  if(t_cost==0) { 
    return 0; //free link because schedule or candidate routing already maps
  } else {
    if(fu && sched->isPassthrough(fu)) return -1; //someone else's pass through
  
    bool passthrough = (fu && !is_dest);
//    int passthrough_cost = 100 / 10; 
    return t_cost+passthrough*1000;
  }
}


pair<int,int> SchedulerSimulatedAnnealing::route(Schedule* sched, 
    SbPDG_Edge* pdgedge, sbnode* source, sbnode* dest, 
    CandidateRouting& candRouting, pair<int,int> scoreLeft) {

  if(source == dest) {
    sblink* link = source->getCycleLink();
    assert(link);
    candRouting.routing[link].insert(pdgedge);
    auto& prop = candRouting.edge_prop[pdgedge];
    prop.num_links=0;//1-1
    prop.num_passthroughs=0;
    return std::make_pair(0,2);
  }

  pair<int,int> score = route_minimizeDistance(sched, pdgedge, source, dest, candRouting, scoreLeft);
    return score;
}



