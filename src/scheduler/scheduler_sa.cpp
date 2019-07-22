#include "scheduler_sa.h"

using namespace SS_CONFIG;
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

void SchedulerSimulatedAnnealing::initialize(SSDfg* ssDFG, Schedule*& sched) {
  //Sort the input ports once
  SS_CONFIG::SubModel* subModel = _ssModel->subModel();
  ssio_interface& si =  subModel->io_interf();
  _sd_in.clear();
  _sd_out.clear();
  si.sort_in_vports(_sd_in);
  si.sort_out_vports(_sd_out);

  si.fill_vec();

  ssDFG->sort<SSDfgVecInput>();
  ssDFG->sort<SSDfgVecOutput>();

  sched = new Schedule(getSSModel(),ssDFG); //just a dummy one
}

std::pair<int, int> SchedulerSimulatedAnnealing::obj( Schedule*& sched, 
    int& lat, int& latmis, int& ovr, int& agg_ovr, int& max_util) {  
  int num_left = sched->num_left(); 
  bool succeed_sched = (num_left==0);

  sched->get_overprov(ovr,agg_ovr,max_util);
  bool succeed_timing = sched->fixLatency(lat,latmis);

  int violation = sched->violation();

//  int obj = agg_ovr*100000 + ovr * 50000 + latmis*10000+violation*100+lat + max_util;
    int obj = agg_ovr*10000 + violation*2000 +latmis*2000
      + lat + max_util*30000;


    if(false) {
      fprintf(stdout, "\n\nobjective rt:%d, left: %3d, " 
            "lat: %3d, vio %d, mis: %d, ovr: %d, util: %d, "
            "obj:%d, ins: %d/%zu, outs: %d/%zu,"
            " insts: %d, links:%d, edge-links:%d  %s%s\n", 
            _route_times,
            sched->num_left(), lat, 
            sched->violation(), latmis, ovr, max_util, obj,
            sched->num_mapped<SSDfgInput>(),  sched->ssdfg()->nodes<SSDfgInput*>().size(),
            sched->num_mapped<SSDfgOutput>(), sched->ssdfg()->nodes<SSDfgOutput*>().size(),
            sched->num_mapped<SSDfgInst>(),
            sched->num_links_mapped(),
            sched->num_edge_links_mapped(),
            succeed_sched ? ", all mapped" : "",
            succeed_timing ? ", mismatch == 0" : "");
    }

//    sched->printGraphviz("hi.gv");

  return make_pair( succeed_sched  -num_left, -obj);
}

bool SchedulerSimulatedAnnealing::schedule(SSDfg* ssDFG, Schedule*& sched) {  
  initialize(ssDFG,sched);

  int max_iters_no_improvement = 100000000;
  srand(++_srand);

  Schedule* cur_sched = new Schedule(getSSModel(),ssDFG);
  std::pair<int, int> best_score = make_pair(0, 0);
  bool best_succeeded = false;
  bool best_mapped = false;
  std::set<SSDfgOutput*> best_dummies;

  int last_improvement_iter = 0;

  _best_lat=MAX_ROUTE;
  _best_violation=MAX_ROUTE;

  int presize = ssDFG->inst_vec().size();
  int postsize = presize;

  // An attempt to remap SSDfg
  SS_CONFIG::SubModel* subModel = _ssModel->subModel();
  int hw_num_fu = subModel->sizex()*subModel->sizey();

  int remapNeeded = false; //ssDFG->remappingNeeded(); //setup remap structres
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
      cur_sched = new Schedule(getSSModel(),ssDFG);
     
      if(remapNeeded) { //remap every so often to try new possible dummy positions
        ssDFG->remap(hw_num_fu);
        postsize = ssDFG->inst_vec().size();
        cur_sched->allocate_space();
      }
    }

    // every so often you can reset?
    if(iter - last_improvement_iter > 1023) {
      *cur_sched = *sched; //shallow copy of sched should work?
    }


    bool succeed_sched = schedule_internal(ssDFG, cur_sched);

    int lat=INT_MAX,latmis=INT_MAX,agg_ovr=INT_MAX,ovr=INT_MAX,max_util=INT_MAX;
    std::pair<int,int> score = obj(cur_sched,lat,latmis,ovr,agg_ovr,max_util);

    int succeed_timing = (latmis ==0) && (ovr ==0);

    if (verbose && ((score > best_score) || print_stat)) {
      stringstream ss;
      ss << "viz/iter/" << iter << ".gv";
      cur_sched->printGraphviz(ss.str().c_str());

      for(int i = 0; i < ssDFG->num_vec_input(); ++i) {
        cout << cur_sched->vecPortOf(ssDFG->vec_in(i)).second << " ";
      }

      fprintf(stdout, "Iter: %4d, time:%0.2f, rt:%d, left: %3d, " 
              "lat: %3d, vio %d, mis: %d, ovr: %d, agg_ovr: %d, util: %d, "
              "obj:%d, ins: %d/%d, outs: %d/%d,"
              " insts: %d/%d,%d, links:%d, edge-links:%d  %s%s", 
              iter, total_msec()/1000.f, _route_times,
              cur_sched->num_left(), lat, 
              cur_sched->violation(), latmis, ovr, agg_ovr, 
              max_util, -score.second,
              cur_sched->num_mapped<SSDfgInput>(),  (int) ssDFG->nodes<SSDfgInput*>().size(),
              cur_sched->num_mapped<SSDfgOutput>(), (int) ssDFG->nodes<SSDfgOutput*>().size(),
              cur_sched->num_mapped<SSDfgInst>(),  presize, postsize,
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
        ssDFG->printGraphviz("viz/remap.dot");
        best_dummies = ssDFG->getDummiesOutputs();
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
    ssDFG->rememberDummies(best_dummies);
  }

  if(cur_sched) {
    delete cur_sched;
  }
  return best_mapped;
}

void SchedulerSimulatedAnnealing::findFirstIndex(
    ssio_interface& si, unsigned int numIO, unsigned int& index, bool is_input){
  for (size_t e = sd(is_input).size(); index < e; index++) {
    pair<int,int> j = sd(is_input)[index];
    int vn = j.first; 
    auto* vdesc = si.get(is_input, vn);
    //Check if the vetcor port is 1. big enough
    if (numIO <= vdesc->size()) { break;}
  }
}

bool SchedulerSimulatedAnnealing::genRandomIndexBW(
        pair<bool, int>& vport_id,
        vector<int>& vport_desc,
        ssio_interface& si,
        unsigned int index,
        Schedule*& sched,
        bool is_input) {
  unsigned int k;
  pair<int, int> p;
  int vport_num;
  unsigned int rep = 0;
  bool success=false;
  size_t _size = sd(is_input).size();

  while (!success && rep++ < _size*2) {
    k = rand() % (_size - index) + index;
    p = sd(is_input)[k];
    vport_num = p.first;
    vport_id = std::make_pair(is_input,vport_num);
    
    success = sched->vportOf(vport_id) == nullptr;

    vport_desc = si.get(is_input, vport_num)->port_vec();
  }

  return success;
}

bool SchedulerSimulatedAnnealing::map_io_to_completion(SSDfg* ssDFG, Schedule* sched) {

  while (!(sched->is_complete<SSDfgInput>() && sched->is_complete<SSDfgOutput>())) {
    int r = rand_bt(0, 2); //upper limit defines ratio of input/output scheduling
    switch (r) {
      case 0: {
        if (sched->is_complete<SSDfgInput>())
          break;
        bool success = map_one<SSDfgVecInput>(ssDFG, sched);
        if (!success) return false;
        break;
      }
      case 1: {
        if (sched->is_complete<SSDfgOutput>())
          break;
        bool success = map_one<SSDfgVecOutput>(ssDFG, sched);
        if (!success) return false;
        break;
      }
      default: {
        assert(0);
        break;
      }
    }
  }

  return true;
}


bool SchedulerSimulatedAnnealing::map_to_completion(SSDfg* ssDFG, Schedule* sched) {

  if(DEBUG_SCHED) cout << "Map to completion! " << sched->num_mapped<SSDfgNode>() << "\n";
  while(!sched->is_complete<SSDfgNode>()) {
    int r = rand_bt(0,16); //upper limit defines ratio of input/output scheduling
    switch(r) {
      case 0: {
        if(sched->is_complete<SSDfgInput>()) break;
        bool success = map_one<SSDfgVecInput>(ssDFG,sched);
        if(!success) {
          return false;
        }
        break;
      }
      case 1: {
        if(sched->is_complete<SSDfgOutput>()) break;
        bool success = map_one<SSDfgVecOutput>(ssDFG,sched);
        if(!success) { 
          return false;
        }
        break;
      }      
      default: {
        if(sched->is_complete<SSDfgInst>()) break;
        bool success = map_one<SSDfgInst>(ssDFG,sched);
        if(!success) { 
          return false;
        }
        break;
      }      
    }
  }
  
  return true;
}

void SchedulerSimulatedAnnealing::unmap_some(SSDfg* ssDFG, Schedule* sched) {

  int num_to_unmap = 1;
  int r = rand() % 1000; //upper limit defines ratio of input/output scheduling
  num_to_unmap = (r < 5) ? 10 : (r < 10 ? 5 : num_to_unmap);

  for (int i = 0; i < num_to_unmap && sched->num_mapped<SSDfgNode>(); ++i) {
    bool flag = sched->num_mapped<SSDfgNode>() != 0;
    while (flag) {
      r = rand() % 8;
      if (r == 0 && sched->num_mapped<SSDfgInput>()) {
        unmap_one<SSDfgVecInput>(ssDFG, sched);
        flag = false;
      } else if (r == 1 && sched->num_mapped<SSDfgOutput>()) {
        unmap_one<SSDfgVecOutput>(ssDFG, sched);
        flag = false;
      } else if (sched->num_mapped<SSDfgInst>()) {
        unmap_one<SSDfgInst>(ssDFG, sched);
        flag = false;
      }
    }
  }

}

bool SchedulerSimulatedAnnealing::schedule_internal(SSDfg* ssDFG, Schedule*& sched) {
  int max_retries = 100;

  for(int t = 0; t < max_retries; ++t) {
    unmap_some(ssDFG,sched);

    if(_fake_it) {
      //Just map the i/o ports
      map_io_to_completion(ssDFG,sched);
      if(sched->is_complete<SSDfgInput>() && sched->is_complete<SSDfgOutput>()) {
        return true;
      }
    } else {
      //THE REAL THING
      map_to_completion(ssDFG,sched);
      if(sched->is_complete<SSDfgNode>()) {
        //cout << "remppaed after " << t << " tries" << "\n";
        return true;
      } 
    }
  } 
  //cout << "failed to remap\n";
  return false;
}

bool SchedulerSimulatedAnnealing::scheduleNode(Schedule* sched, SSDfgInst* dfgnode) {
  std::vector<std::pair<int, ssnode*>> spots;

  CandidateRouting route1, route2;
  CandidateRouting *bestRouting = &route1, *curRouting = &route2;

  spots = dfgnode->candidates(sched, _ssModel);

  //populate a scheduling score for each of canidate ssspot
  if ( (rand() % 1024) == 1) { //Pick a totally random spot
    bool succ = false;
    unsigned int attempt = 0;
    while (!succ && attempt < spots.size()) {
      size_t r2 = rand() % spots.size();

      curRouting->clear();
      std::pair<int, int> n_score = scheduleHere(sched, dfgnode, spots[r2], *curRouting);
      bool failed = (n_score == fscore);

      if (!failed) {
        apply_routing(sched, dfgnode, spots[r2], curRouting);
        return true;
      } else {
        attempt++;
      }
    }
    return false; // couldn't find one

  } else {
    std::pair<int, int> bestScore = make_pair(INT_MIN, INT_MIN);
    std::pair<int, ssnode *> bestspot(-1, nullptr);
    int num_found = 0;

    for (unsigned i = 0; i < spots.size(); i++) {

      curRouting->clear();
      std::pair<int, int> n_score = scheduleHere(sched, dfgnode, spots[i], *curRouting);

      bool failed = (n_score == fscore);
      int routes = n_score.second;

      if (!failed) {
        num_found++;
        apply_routing(sched, dfgnode, spots[i], curRouting);
        int lat = INT_MAX, latmis = INT_MAX, ovr = INT_MAX, 
            agg_ovr = INT_MAX, max_util = INT_MAX;
        std::pair<int, int> curScore = obj(sched, lat, latmis, ovr, agg_ovr, max_util);
        curScore.second -= routes;
        sched->unassign_dfgnode(dfgnode); //rip it up!

        if (curScore > bestScore) {
          bestScore = curScore;
          bestspot = spots[i];
          std::swap(bestRouting, curRouting);
        }
      }

    }

    if (num_found > 0) {
      apply_routing(sched, dfgnode, bestspot, bestRouting);
    }

    return (num_found > 0);
  }
}

std::pair<int,int> SchedulerSimulatedAnnealing::scheduleHere(Schedule* sched,
    SSDfgNode* n, pair<int, SS_CONFIG::ssnode*> here, CandidateRouting& candRouting) {

  pair<int, int> score = make_pair(0, 0);

  //TODO: make work for decomp-CGRA
  for (auto source_dfgedge : n->in_edges()) {
    SSDfgNode *source_dfgnode = source_dfgedge->def();     //could be input node also

    if (sched->link_count(source_dfgedge) != 0) {
      cerr << "Edge: " << source_dfgedge->name() << " is already routed!\n";
      assert(0);
    }

    //route edge if source dfgnode is scheduled
    if (sched->is_scheduled(source_dfgnode)) {
      auto source_loc = sched->location_of(source_dfgnode); //scheduled location

      //route using source node, ssnode
      pair<int, int> tempScore = route(sched, source_dfgedge, source_loc, here, candRouting);
      if (tempScore == fscore) {
        //cout << "failed to route def\n";
        return fscore;
      }

      score = score + tempScore;
    }
  }

  for (auto use_dfgedge : n->uses()) {
    SSDfgNode *use_dfgnode = use_dfgedge->use();

    if (sched->link_count(use_dfgedge) != 0) {
      cerr << "Edge: " << use_dfgedge->name() << " is already routed!\n";
      assert(0);
    }

    //cout << " try " <<  n->name() << " " << here->name() << " " << score.first << " " << score.second << "\n";

    //route edge if source dfgnode is scheduled
    if (sched->is_scheduled(use_dfgnode)) {
      //ssnode *use_loc = sched->locationOf(use_dfgnode);
      auto use_loc = sched->location_of(use_dfgnode);

      pair<int, int> delta = route(sched, use_dfgedge, here, use_loc, candRouting);
      if (delta == fscore) {
        //cout << "failed to route use\n";
        return fscore;
      }
      score = score + delta;
    }
  }

  return score;
}

int SchedulerSimulatedAnnealing::routing_cost(SSDfgEdge* edge, int from_slot, int next_slot, sslink *link,
      Schedule* sched, CandidateRouting& candRouting, const pair<int, ssnode*> &dest) {

  SSDfgNode *def_dfgnode = edge->def();
  SSDfgNode *use_dfgnode = edge->use();

  bool is_in = def_dfgnode->type() == SSDfgNode::V_INPUT;
  bool is_out = use_dfgnode->type() == SSDfgNode::V_OUTPUT;
  bool is_io = is_in || is_out;

  bool is_temporal = def_dfgnode->is_temporal();
  bool is_temporal_inst = is_temporal && !is_io;
  bool is_temporal_in = is_temporal && is_in;
  bool is_temporal_out = is_temporal && is_out;

  int internet_dis = abs(from_slot - next_slot);
  internet_dis = min(internet_dis, 8 - internet_dis);

  //For now, links only route on their own network
  //ie. temporal_io and non-temporal route on dedicated network
  if ((!is_temporal_inst && link->max_util() > 1) || (is_temporal_inst && link->max_util() <= 1)) {
    return -1;
  }

  ssnode *next = link->dest();

  //check if connection is closed..
  //0: free
  //1: empty
  //2: already there
  int t_cost;
  if (is_temporal_in) {
    t_cost = sched->temporal_cost_in(link, dynamic_cast<SSDfgInput *>(def_dfgnode)->input_vec());
  } else if (is_temporal_out) {
    t_cost = sched->temporal_cost_out(make_pair(from_slot, link), def_dfgnode,
                                      dynamic_cast<SSDfgOutput *>(use_dfgnode)->output_vec());
  } else { //NORMAL CASE!
    t_cost = sched->temporal_cost(make_pair(from_slot, link), def_dfgnode);
    for (auto elem : candRouting.routing[make_pair(from_slot, link)]) {
      if (elem->def() == def_dfgnode) {
        t_cost = 0;
        break;
      }
    }
  }

  if (t_cost != 0) { //if we are not already riding on the same route

    if (candRouting.routing.count(make_pair(from_slot, link))) {
      auto& edges = candRouting.routing[make_pair(from_slot, link)];
      for (auto edge : edges) {
        auto *cur_node = edge->def();
        if (cur_node == def_dfgnode) {
          t_cost = 0;
          break;
        }
        //kind of horribly ugly
        if (is_temporal_in) {
          bool flag = false;
          if (auto input_vec = dynamic_cast<SSDfgInput *>(def_dfgnode)) {
            flag |= sched->input_matching_vector(cur_node, input_vec->input_vec());
          }
          if (auto output_vec = dynamic_cast<SSDfgOutput *>(use_dfgnode)) {
            flag |= sched->output_matching_vector(cur_node, output_vec->output_vec());
          }
          if (flag) {
            t_cost = 0;
            break;
          }
        }
        if (is_temporal_out) {

        }
      }
      if (t_cost!=0) t_cost+=edges.size();      
    }
  }

  if (t_cost >= 2) { //square law avoidance of existing routes
    if (!is_temporal_inst) {
      return (t_cost) * (t_cost) * 10;
    }
  }
  bool is_dest = (next == dest.second && next_slot == dest.first);

  ssfu *fu = dynamic_cast<ssfu *>(next);
  if (fu && sched->dfgNodeOf(fu) && !is_dest)
    return -1;  //stop if run into fu
  if (t_cost == 0) {
    return 0 + internet_dis; //free link because schedule or candidate routing already maps
  } else {
    if (fu && sched->isPassthrough(fu))
      return -1; //someone else's pass through
    bool passthrough = (fu && !is_dest);
    return t_cost + passthrough * 1000 + internet_dis;
  }
}


pair<int,int> SchedulerSimulatedAnnealing::route(Schedule* sched, SSDfgEdge* dfgedge,
        pair<int, ssnode*> source, pair<int, ssnode*> dest, CandidateRouting& candRouting) {

  if (source.second == dest.second) {
    sslink *link = source.second->get_cycle_link();
    assert(link);
    if (source.first < dest.first) {
      for (int i = source.first + 1; i <= dest.first; ++i)
        candRouting.routing[make_pair(i, link)].insert(dfgedge);
    } else if (source.first > dest.first) {
      for (int i = source.first - 1; i >= dest.first; --i)
        candRouting.routing[make_pair(i, link)].insert(dfgedge);
    } else {
      //I am not sure if this will happen
      candRouting.routing[make_pair(source.first, link)].insert(dfgedge);
    }
    auto &prop = candRouting.edge_prop[dfgedge];
    prop.num_links = 0;
    prop.num_passthroughs = 0;
    return std::make_pair(0, 2 + abs(source.first - dest.first));
  }

  pair<int, int> score = route_minimize_distance(sched, dfgedge, source, dest, candRouting);
  return score;
}
