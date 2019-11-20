#ifndef __SS_SCHEDULER_H__
#define __SS_SCHEDULER_H__

#include "model.h"
#include "schedule.h"
#include "ssdfg.h"

#include <stdlib.h>
#include <boost/functional.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>

#define MAX_ROUTE 100000000

using usec = std::chrono::microseconds;
using get_time = std::chrono::steady_clock;

int rand_bt(int s, int e);
int rand_bt_large(int s, int e);

template <typename T, typename U>
std::pair<T, U> operator+(const std::pair<T, U>& l, const std::pair<T, U>& r) {
  return {l.first + r.first, l.second + r.second};
}

template <typename T, typename U>
std::pair<T, U> operator-(const std::pair<T, U>& l, const std::pair<T, U>& r) {
  return {l.first - r.first, l.second - r.second};
}


// This class contains all info which you might want to remember about
class WorkloadSchedules {
public:
  std::vector<Schedule> sched_array; 
  
  float estimate_performance() {return 0;}
};

class SchedStats {
public:
  int lat = INT_MAX, latmis = INT_MAX;
  int agg_ovr = INT_MAX, ovr = INT_MAX, max_util = INT_MAX;
};

//Class which captures a codesign between hardware and software through
//scheduling.  Note here the the Model and all schedules are "owned" by
//the codesign instance, ie. they will be deleted when the class is deleted
//
// TODO: There are an infinite number of things to do here, starting with:
// 1. Explore full design space with randomness
// 2. 

class CodesignInstance {
  SSModel _ssModel;
  public:
  SSModel* ss_model() {return &_ssModel;}
  std::vector<WorkloadSchedules> workload_array;

  CodesignInstance(SSModel* model) : _ssModel(*model) {
    verify();
  }

  // Check that everything is okay
  void verify() {
    for_each_sched([&](Schedule& sched) { 
      assert(&_ssModel == sched.ssModel());
      assert(_ssModel.subModel()->node_list().size() >= sched.node_prop().size());
      assert(_ssModel.subModel()->link_list().size() >= sched.link_prop().size());

      for(auto& ep : sched.edge_prop()) {
        for(auto& p : ep.links) {
          assert(p.second->id() < _ssModel.subModel()->link_list().size());
          assert(p.second->id() < sched.link_prop().size());
          assert(_ssModel.subModel()->link_list()[p.second->id()] == p.second);
        }
      }


    });
    for(auto& node : _ssModel.subModel()->node_list()) {
      assert(node->subnet_table().size() == node->out_links().size());
    }
    for(int i = 0; i < _ssModel.subModel()->link_list().size(); ++i) {
      assert(_ssModel.subModel()->link_list()[i]->id() == i);
    }
  }

  //Verify, plus also check:
  //1. there are no hangers (elements with no purpose, usually after delete)
  void verify_strong() {
    verify();

    // I realize that hangers are kind of painful to ensure they are deleted
    // they will get cleaned up over time
    /*
    for(auto* node : _ssModel.subModel()->node_list()) {
      if(!(node->is_output() || node->out_links().size() > 0)) {
        std::cout << "problem with node, no outputs: " << node->name() << "\n";
        assert(0 && "bad node, no outputs");
      }
      if(!(node->is_input()  || node->in_links().size()  > 0)) {
        std::cout << "problem with node, no inputs: " << node->name() << "\n";
        assert(0 && "bad node, no inputs");
      }
    }*/
  }

  void add_random_edges_to_node(ssnode* n, int min_in, int max_in, 
                                           int min_out, int max_out) {
    auto* sub = _ssModel.subModel();

    int n_ins = rand_bt(min_in,max_in);
    int n_outs = rand_bt(min_out,max_out);
    for(int i = 0; i < n_ins; ++i) {
      int src_node_index = rand_bt(0,sub->node_list().size());
      ssnode* src = sub->node_list()[src_node_index];
      if(src->is_output() || src == n) {
        i--;
        continue;
      }
      sub->add_link(src,n);
    }
    for(int i = 0; i < n_outs; ++i) {
      int dst_node_index = rand_bt(0,sub->node_list().size());
      ssnode* dst = sub->node_list()[dst_node_index];
      if(dst->is_input() || dst == n) {
        i--;
        continue;
      }
      sub->add_link(n,dst);
    }
  }

  //Delete FUs, Switches, and Vports that can't possible be useful
  void delete_hangers() {
    auto* sub = _ssModel.subModel();

    for(ssfu* fu : sub->fu_list()) {
      if(fu->in_links().size() <=1 || fu->out_links().size() < 1) {
        delete_hw(fu);
      }
    }
    /*for(ssswitch* sw : sub->switch_list()) {
      if(sw->in_links().size() < 1 || sw->out_links().size() < 1) {
        delete_hw(sw);
      }
      if(sw->in_links().size() == 1 && sw->out_links().size() == 1) {
        sub->add_link(sw->in_links().front()->orig(),
                      sw->out_links().front()->dest());
        delete_hw(sw);
      }
    }*/
    for(ssvport* ivport : sub->input_list()) {
      if(ivport->out_links().size() < 1) {
        delete_hw(ivport);
      }
    }
    for(ssvport* ovport : sub->output_list()) {
      if(ovport->in_links().size() < 1) {
        delete_hw(ovport);
      }
    }

  }

  void make_random_modification() {
    auto* sub = _ssModel.subModel();

    verify_strong();
    //Choose a set of Items to remove
    int n_items = rand_bt(1,8);
    for(int i = 0; i < n_items; ++i) {
      int item_class = rand_bt(0,100);
      if(item_class < 65) {
        //delete a link
        if(sub->link_list().size()==0) continue;
        int index = rand_bt(0,sub->link_list().size());
        sslink* l = sub->link_list()[index];
        if(delete_linkp_list.count(l)) continue; //don't double delete
        delete_link(l);
      } else if(item_class < 80) {
        //delete a switch
        if(sub->switch_list().size()==0) continue;
        int index = rand_bt(0,sub->switch_list().size());
        ssswitch* sw = sub->switch_list()[index];
        if(delete_nodep_list.count(sw)) continue; //don't double delete
        delete_hw(sw);
      } else if (item_class < 90) {
        //delete an FU
        if(sub->fu_list().size()==0) continue;
        int index = rand_bt(0,sub->fu_list().size());
        ssfu* fu = sub->fu_list()[index];
        if(delete_nodep_list.count(fu)) continue; //don't double delete
        delete_hw(fu);
      } else if (item_class < 95) {
        //delete an VPort
        if(sub->input_list().size()==0) continue;
        int index = rand_bt(0,sub->input_list().size());
        ssvport* vport = sub->input_list()[index];
        if(delete_nodep_list.count(vport)) continue; //don't double delete
        delete_hw(vport);
      } else { // (item_class < 100) 
        //delete an VPort
        if(sub->output_list().size()==0) continue;
        int index = rand_bt(0,sub->output_list().size());
        ssvport* vport = sub->output_list()[index];
        if(delete_nodep_list.count(vport)) continue; //don't double delete
        delete_hw(vport);
      }
    }

    // TODO: delete some entries in routing tables?

    // Lets finalize the delete here so that the datastructre is consistent
    // again when we are adding things -- simpler
    finalize_delete();

    delete_hangers();

    finalize_delete(); // TODO part of finalize delete isredundant now

    verify_strong();

    //Items to add
    n_items = rand_bt(0,8);
    for(int i = 0; i < n_items; ++i) {
      int item_class = rand_bt(0,100);      
      if(item_class < 65) {
        // Add a random link -- really? really
        int src_node_index = rand_bt(0,sub->node_list().size());
        int dst_node_index = rand_bt(0,sub->node_list().size());
        ssnode* src = sub->node_list()[src_node_index];
        ssnode* dst = sub->node_list()[dst_node_index];
        if(src->is_output() || dst->is_input() || src == dst) continue;

        sslink* link = sub->add_link(src,dst);
        //std::cout << "adding link: " << link->name() << "\n"; 
      } else if(item_class < 80) {
        // Add a random switch
        ssswitch* sw = sub->add_switch();       
        add_random_edges_to_node(sw,1,9,1,9); 

        //std::cout << "adding switch" << sw->name() << " ins/outs:" 
        //          << sw->in_links().size() << "/" << sw->out_links().size() << "\n";
      } else if (item_class < 90) {
         ssfu* fu = sub->add_fu();
         //std::cout << "adding fu" << fu->id() << " ----------------------------\n";

         //Randomly pick an FU type from the set
         auto& fu_defs = _ssModel.fuModel()->fu_defs();
         int fu_def_index = rand_bt(0,fu_defs.size());
         func_unit_def* def = &fu_defs[fu_def_index];
         fu->setFUDef(def);

         add_random_edges_to_node(fu,1,9,1,9); 
       
      } else if (item_class < 95) {
        // Add a random input vport
         ssvport* vport = sub->add_vport(true);
         add_random_edges_to_node(vport,0,1,5,12); 
         //std::cout << "adding input vport: " << vport->name() << "\n";
      } else { // (item_class < 100) 
        // Add a random output vport
         ssvport* vport = sub->add_vport(false);
         add_random_edges_to_node(vport,5,12,0,1); 
         //std::cout << "adding output vport: " << vport->name() << "\n";
      } 
    }

    verify_strong();

    for_each_sched([&](Schedule& sched) { 
      sched.allocate_space();
    });


    verify_strong();

    // TODO: add some entries in routing tables?

    //End this by making sure all the routing tables are generated
    for(auto* node : sub->node_list()) {
      node->setup_routing_memo();
    }

    verify_strong();
  }

  void for_each_sched(const std::function<void (Schedule&)>& f) {
    for(auto& ws : workload_array) {
      for(Schedule& sched : ws.sched_array) {
        f(sched);
      }
    }
  }

  //Overide copy constructor to enable deep copies
  CodesignInstance(const CodesignInstance& c) : _ssModel(c._ssModel) {
   
    //SSModel* copy_model = (SSModel*)&c._ssModel;
    //auto* copy_sub = copy_model->subModel();
    //std::cout << "copy from:" << copy_sub << " copy to:" << _ssModel.subModel() << "\n";
    //assert(_ssModel.subModel() != copy_sub);

    for(auto& node : _ssModel.subModel()->node_list()) {
      assert(node->subnet_table().size() == node->out_links().size());
    }

    workload_array = c.workload_array;

    for_each_sched([&](Schedule& sched){ 
      //replace this ssmodel with the copy 
      sched.swap_model(_ssModel.subModel());
      sched.set_model(&_ssModel);
    });

    for(auto& node : _ssModel.subModel()->node_list()) {
      assert(node->subnet_table().size() == node->out_links().size());
    }

  }

  //Delete link on every schedule
  void delete_link(sslink* link) {
    delete_link_list.push_back(link->id());
    delete_linkp_list.insert(link);

    // remove it from every schedule
    for_each_sched([&](Schedule& sched){ 
      for(int slot = 0; slot < sched.num_slots(link); ++slot) {
        for(auto& p : sched.dfg_edges_of(slot,link)) {
          //TODO: consider just deleteting the edge, and having the scheduler
          //try to repair the edge schedule -- this might save some time
          //sched.unassign_edge(p.first);
          sched.unassign_dfgnode(p.first->def());
          sched.unassign_dfgnode(p.first->use());
        }
      }
    });
  }

  //Delete fu on every schedule
  void delete_hw(ssfu* fu) {
    delete_fu_list.push_back(fu->id());
    delete_node(fu);
  }

  //Delete switch on every schedule
  void delete_hw(ssswitch* sw) {
    //std::cout << "Deleting Switch:" << sw->name() << "    ptr:" << sw << "\n";
    delete_sw_list.push_back(sw->id());
    delete_node(sw);
  }

  //Delete vector port on every schedule
  void delete_hw(ssvport* vport) {
   delete_vport_list.push_back(vport->id());
   delete_node(vport);
  }

  // This makes the delete consistent across model and schedules
  void finalize_delete() {
    //Grab a copy copy of all nodes
    auto* sub = _ssModel.subModel();
    std::vector<ssnode*> n_copy = sub->node_list(); //I hope this copies the list? 
    std::vector<sslink*> l_copy = sub->link_list(); //I hope this copies the list?

    verify(); 

    // Remove the elements from these lists
    sub->delete_fus(delete_fu_list);
    sub->delete_switches(delete_sw_list);
    sub->delete_vports(delete_vport_list);
    sub->delete_nodes(delete_node_list); //these happen after above, b/c above uses id
    sub->delete_links(delete_link_list);

    // got to reorder all the links
    for_each_sched([&](Schedule& sched) { 
      sched.reorder_node_link(n_copy,l_copy);
    });

    verify(); 

    //finally, we just deleted a bunch of nodes/links, and we should
    //probably free the memory somehow?
    //that's why we tracked these datastructures
    for(auto* link : delete_linkp_list) {
      // we also need to tell the model to delete the link from its little lists
      link->orig()->unlink_outgoing(link);
      link->dest()->unlink_incomming(link);
      delete link;
    }
    for(auto* node : delete_nodep_list) delete node;

    verify();

    //finally finally, clear all datastructres used for deleting
    delete_nodep_list.clear();
    delete_linkp_list.clear();
  
    delete_node_list.clear();
    delete_link_list.clear();
    delete_fu_list.clear();
    delete_sw_list.clear();
    delete_vport_list.clear();

    verify(); 

  }

  std::pair<int,int> dse_sched_obj(Schedule* sched) {
    //YES, I KNOW THIS IS A COPY OF SCHED< JUST A TEST FOR NOW
    SchedStats s;
    int num_left = sched->num_left();
    bool succeed_sched = (num_left == 0);

    sched->get_overprov(s.ovr, s.agg_ovr, s.max_util);
    sched->fixLatency(s.lat, s.latmis);
  
    int violation = sched->violation();
  
    int obj = s.agg_ovr * 1000 + violation * 200 + s.latmis * 200 + s.lat +
              (s.max_util - 1) * 3000 + sched->num_passthroughs();
    obj = obj * 100 + sched->num_links_mapped();
 
    int succeed_timing = succeed_sched && (s.latmis == 0) && (s.ovr == 0);
  
    return std::make_pair(succeed_timing - num_left, -obj);
  }

  float dse_obj() {
    std::pair<int, int> total_score = std::make_pair(0,0);

    for(auto& ws : workload_array) {
      SchedStats s;
      std::pair<int, int> score = std::make_pair(INT_MIN,INT_MIN);
      for(Schedule& sched : ws.sched_array) {
        std::pair<int,int> new_score = dse_sched_obj(&sched);
        if(new_score > score) {
          score=new_score;
        }
      }

      //total_score.first += score.first;
      //total_score.second += score.second;
      //
      total_score.first += (score.first ==1);

    }

    std::cout << "obj first: " << total_score.first << "\n";

    float obj = total_score.first / _ssModel.subModel()->get_overall_area();
    return obj;
  }

  private:
  //When we delete a hardware element, we need to:
  //1. deschedule anything that was assigned to that element
  //2. remove the concept of that element from the schedule (consistency)
  //3. remove the element from the hardware description
  void delete_node(ssnode* n) {
    delete_node_list.push_back(n->id());
    delete_nodep_list.insert(n);

    for_each_sched([&](Schedule& sched){ 
      for(int slot = 0; slot < sched.num_slots(n); ++slot) {
        for(auto& p : sched.dfg_nodes_of(slot,n)) {
          sched.unassign_dfgnode(p.first);
        }
      }
    });
    
    for(auto& l : n->in_links()) {
      delete_link(l);
    }
    for(auto& l : n->out_links()) {
      delete_link(l);
    }
  }

  std::unordered_set<ssnode*> delete_nodep_list;
  std::unordered_set<sslink*> delete_linkp_list;

  std::vector<int> delete_node_list;
  std::vector<int> delete_link_list;
  std::vector<int> delete_fu_list;
  std::vector<int> delete_sw_list;
  std::vector<int> delete_vport_list;


};

class Scheduler {
 public:
  Scheduler(SS_CONFIG::SSModel* ssModel)
      : _ssModel(ssModel), _optcr(0.1f), _optca(0.0f), _reslim(100000.0f) {}

  bool check_feasible(SSDfg* ssDFG, SSModel* ssmodel, bool verbose);

  bool vport_feasible(SSDfg* ssDFG, SSModel* ssmodel, bool verbose);

  virtual bool schedule(SSDfg* ssDFG, Schedule*& schedule) = 0;

  virtual bool incrementalSchedule(CodesignInstance& incr_table) {
    assert(0 && "not supported");
  }

  bool verbose;
  bool suppress_timing_print = false;

  void set_max_iters(int i) { _max_iters = i; }

  std::string str_subalg;

  std::string AUX(int x) { return (x == -1 ? "-" : std::to_string(x)); }

  double total_msec() {
    auto end = get_time::now();
    auto diff = end - _start;
    return ((double)std::chrono::duration_cast<usec>(diff).count()) / 1000.0;
  }

  void set_start_time() {
    _start = get_time::now();
  }

  virtual bool schedule_timed(SSDfg* ssDFG, Schedule*& sched) {
    set_start_time();

    bool succeed_sched = schedule(ssDFG, sched);

    if (verbose && !suppress_timing_print) {
      printf("sched_time: %0.4f seconds\n", total_msec() / 1000.0);
    }

    return succeed_sched;
  }

  void setGap(float relative, float absolute = 1.0f) {
    _optcr = relative;
    _optca = absolute;
  }

  void setTimeout(float timeout) { _reslim = timeout; }

  bool running() { return !_should_stop; }
  void stop() { _should_stop = true; }

  void set_srand(int i) { _srand = i; }

 protected:
  SS_CONFIG::SSModel* getSSModel() { return _ssModel; }

  SS_CONFIG::SSModel* _ssModel;

  int _max_iters = 20000;
  bool _should_stop = false;
  int _srand = 0;

  std::vector<WorkloadSchedules*> _incr_schedules;

  float _optcr, _optca, _reslim;
  std::chrono::time_point<std::chrono::steady_clock> _start;
};

class HeuristicScheduler : public Scheduler {
 public:
  HeuristicScheduler(SS_CONFIG::SSModel* ssModel)
      : Scheduler(ssModel), fscore(std::make_pair(MAX_ROUTE, MAX_ROUTE)) {}

 protected:
  const std::pair<int, int> fscore;

  void random_order(int n, std::vector<int>& order);

  std::vector<bool> rand_node_choose_k(int k, std::vector<ssnode*>& input_nodes,
                                       std::vector<ssnode*>& output_nodes);

  void rand_n_choose_k(int n, int m, std::vector<int>& indices);

  int _route_times = 0;
};

#endif
