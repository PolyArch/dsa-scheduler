#pragma once

#include <string>

#include "dsa/arch/estimation.h"
#include "dsa/arch/model.h"
#include "schedule.h"
#include "scheduler.h"

// This class contains all info which you might want to remember about
class WorkloadSchedules {
 public:
  std::vector<Schedule> sched_array;

  float estimate_performance() { return 0; }
};

static unsigned int log2 (unsigned int val) {
    if (val == 0) return -1;
    if (val == 1) return 0;
    unsigned int ret = 0;
    while (val > 1) {
        val >>= 1;
        ret++;
    }
    return ret;
};

class SchedStats {
 public:
  int lat = INT_MAX, latmis = INT_MAX;
  int agg_ovr = INT_MAX, ovr = INT_MAX, max_util = INT_MAX;
};

// Class which captures a codesign between hardware and software through
// scheduling.  Note here the the Model and all schedules are "owned" by
// the codesign instance, ie. they will be deleted when the class is deleted
//
// TODO: There are an infinite number of things to do here, starting with:
// 1. Explore full design space with randomness
// 2.

template <typename T>
inline int non_uniform_random(const std::vector<T>& nodes, const std::vector<bool>& vec) {
  for (int res = rand() % nodes.size();; res = rand() % nodes.size()) {
    if (vec[nodes[res]->id()]) {
      return res;
    }
    if (rand() % 2 == 0) {
      return res;
    }
  }
}

class CodesignInstance {
  SSModel _ssModel;

 public:
  SSModel* ss_model() { return &_ssModel; }
  ~CodesignInstance() {}
  std::vector<WorkloadSchedules> workload_array;
  std::vector<int> weight;

  std::vector<Schedule*> res;

  /*
    Vector of unused nodes and links.
    The nodes and links are in the order of the original ADG.
    False means the node or link is used
    True means the node or link is unused
  */
  std::vector<bool> unused_nodes;
  std::vector<bool> unused_links;

  bool sanity_check{false};
  std::string dse_change = "none";
  std::vector<std::string> dse_changes_log;
  double performance = 0;
  double links_score = 0;
  std::vector<double> workload_performances;
  std::vector<std::vector<std::pair<std::string, double>>> dfg_performances;
  std::vector<double> workload_weights;
  double normalized_resources = 0;

  CodesignInstance(SSModel* model);

  // Check that everything is okay
  void verify() {
    if (!sanity_check) return;
    
    DSA_CHECK(workload_array.size() > 0);
    for (auto& work : workload_array) {
      DSA_CHECK(work.sched_array.size() > 0);
    }
    
    // Check Schedules
    for_each_sched([&](Schedule& sched) {
      sched.validate();

      DSA_CHECK(&_ssModel == sched.ssModel()) << &_ssModel << " " << sched.ssModel();
      DSA_CHECK(_ssModel.subModel()->node_list().size() >= sched.node_prop().size());
      DSA_CHECK(_ssModel.subModel()->link_list().size() >= sched.link_prop().size());

      for (auto& ep : sched.edge_prop()) {
        for (auto& p : ep.links) {
          DSA_CHECK(p.second->id() < (int)_ssModel.subModel()->link_list().size());
          DSA_CHECK(p.second->id() < (int)sched.link_prop().size());
          DSA_CHECK(_ssModel.subModel()->link_list()[p.second->id()] == p.second);
        }
      }
    });
    // Check Node
    for (auto& node : _ssModel.subModel()->node_list()) {
      if (auto fu = dynamic_cast<ssfu*>(node)) {
        DSA_CHECK(!fu->fu_type_.capability.empty());
      } else if (auto vport  = dynamic_cast<ssvport*>(node)) {
        if (vport->vp_stated())
          DSA_CHECK(vport->out_links().size() + vport->in_links().size() > 1);
      }
      //DSA_CHECK(node->parent == _ssModel.subModel());
      DSA_CHECK(node == _ssModel.subModel()->node_list()[node->id()]);
    }
    
    for (auto& node : _ssModel.subModel()->node_list()) {
      for (sslink* link : node->in_links()) {
        DSA_CHECK(link->id() < (int)_ssModel.subModel()->link_list().size());
        DSA_CHECK(_ssModel.subModel()->link_list()[link->id()] == link);
      }
      for (sslink* link : node->out_links()) {
        DSA_CHECK(link->id() < (int)_ssModel.subModel()->link_list().size());
        DSA_CHECK(_ssModel.subModel()->link_list()[link->id()] == link);
      }
    }

    for (unsigned i = 0; i < _ssModel.subModel()->link_list().size(); ++i) {
      DSA_CHECK(_ssModel.subModel()->link_list()[i]->id() == (int)i);
    }
  }

  /*
    Gets a log of the changes that happened during this iteration of the dse.
    The boolean cout relates to formating for either the log file or output.
  */
  std::string get_changes_log(bool cout = false) {
    std::ostringstream s;
    for(auto it = dse_changes_log.begin(); it != dse_changes_log.end(); ++it) {
      s << *it;
      if (cout)
        s << std::endl;
      else if (std::next(it) != dse_changes_log.end())
        s << ",";
    }
    return s.str();
  }

  std::string get_workload_weights(bool cout=false) {
    std::ostringstream s;
    for(auto it = workload_weights.begin(); it != workload_weights.end(); ++it) {
      s << *it;
      if (cout)
        s << std::endl;
      else if (std::next(it) != workload_weights.end())
        s << ",";
    }
    return s.str();
  }

  std::string get_workload_performances(bool cout=false) {
    std::ostringstream s;
    for(auto it = workload_performances.begin(); it != workload_performances.end(); ++it) {
      s << *it;
      if (cout)
        s << std::endl;
      else if (std::next(it) != workload_performances.end())
        s << ",";
    }
    return s.str();
  }

  std::string get_dfg_performances(bool cout=false) {
    std::ostringstream s;
    for(auto it = dfg_performances.begin(); it != dfg_performances.end(); ++it) {
      for(auto next = it->begin(); next != it->end(); ++next) {
        s << next->first << ": " << next->second;
        if (cout)
          s << std::endl;
        else if (std::next(it) != dfg_performances.end() || std::next(next) != it->end())
          s << ",";
      }
    }
    return s.str();
  }

  // Verify, plus also check:
  // 1. there are no hangers (elements with no purpose, usually after delete)
  void verify_strong() { verify(); }

  void add_random_edges_to_node(ssnode* n, int min_in, int max_in, int min_out,
                                int max_out) {
    auto* sub = _ssModel.subModel();

    if (sub->node_list().empty()) return;

    int n_ins = rand() % (max_in - min_in) + min_in;
    for (int i = 0, j = 0; i < n_ins && j < n_ins * 10 && n->in_links().size() <= 4;
         ++i, ++j) {
      int src_node_index = rand() % sub->nodes<ssnode*>().size();
      ssnode* src = sub->node_list()[src_node_index];
      if ((dynamic_cast<ssvport*>(src) && src->out_links().empty()) || src == n) {
        i--;
        continue;
      }

      if (src == n) continue;
      sub->add_link(src, n);
      check_stated(src);
      

    }
    int n_outs = rand() % (max_out - min_out) + min_out;
    for (int i = 0, j = 0; i < n_outs && j < n_outs * 10 && n->out_links().size() <= 4;
         ++i, ++j) {
      int dst_node_index = rand() % sub->node_list().size();
      ssnode* dst = sub->node_list()[dst_node_index];
      if ((dynamic_cast<ssvport*>(dst) && dst->in_links().empty()) || dst == n) {
        i--;
        continue;
      }

      if (n == dst) continue;

      sub->add_link(n, dst);
      check_stated(dst);
      
    }
  }

  template <typename T>
  bool delete_nodes(std::function<bool(T*)> f) {
    bool res = false;
    auto* sub = _ssModel.subModel();
    std::for_each(sub->nodes<T*>().begin(), sub->nodes<T*>().end(),
                  [this, &res, f](T* n) {
                    if (f(n)) {
                      this->delete_node(n);
                      res = true;
                    }
                  });
    return res;
  }

  /**
   * @brief Deletes Links and Nodes that can't be useful
   * 
   * @return true if something is deleted
   * @return false otherwise
   */
  bool delete_hangers() {
    bool deleted_something = false;

    //TODO: Convert from set once memory is added
    std::unordered_set<ssnode*> nodes_to_delete;

    auto* sub = _ssModel.subModel();

    for (ssfu* fu : sub->fu_list()) {
      if (fu->in_links().size() <= 1 || fu->out_links().size() < 1) {
        nodes_to_delete.insert(fu);
        deleted_something = true;
      }
      if (fu->fu_type_.capability.empty()) {
        nodes_to_delete.insert(fu);
        deleted_something = true;
      }
    }
    for (ssswitch* sw : sub->switch_list()) {
      if (sw->out_links().size() == 0 || sw->in_links().size() == 0) {
        nodes_to_delete.insert(sw);
        deleted_something = true;
      }
    }

    // TODO: ovport == ivport if input_size & ouput_size = 0
    for (ssvport* ivport : sub->input_list()) {
      if (ivport->out_links().size() < 1 && ivport->out_links().size() < 1) {
        nodes_to_delete.insert(ivport);
        deleted_something = true;
      }
    }
    for (ssvport* ovport : sub->output_list()) {
      if (ovport->in_links().size() < 1) {
        nodes_to_delete.insert(ovport);
        deleted_something = true;
      }
    }

    for (auto& node : nodes_to_delete) {
      delete_node(node);
    }

    return deleted_something;
  }

  void prune_all_unused() {
    // First Prune all current Hangers
    while (delete_hangers()) {  }

    // Now get the utilization
    auto util = utilization();
    
    // If there is no utilization, then we shouldn't be pruning
    if (std::get<2>(util) == 0) {
      DSA_LOG("No utilization, not pruning");
      return;
    }

    auto* sub = _ssModel.subModel();

    std::vector<ssnode*> nodes_to_delete;
    std::vector<sslink*> links_to_delete;
    
    // Add all links that are unused to a vector then delete them
    // Note: Links must be deleted before nodes
    for (int i = 0; i < unused_links.size(); i++) {
      if (unused_links[i])
        links_to_delete.push_back(sub->link_list()[i]);
    }

    for (auto link : links_to_delete) {
      delete_link(link);
    }

    // Add all nodes that are unused to a vector then delete them
    for (int i = 0; i < unused_nodes.size(); i++) {
      if (unused_nodes[i]) 
        nodes_to_delete.push_back(sub->node_list()[i]);
    }

    for (auto node : nodes_to_delete) {
      delete_node(node);
    }
    
    while (delete_hangers()) {  }
  }

  void make_random_modification(double temperature) {
    int iteration = rand() % 3;
    int add_iterations, remove_iterations, modify_iterations = 0;
    for (int i = 0; i < temperature; ++i) {
      int item_class = rand() % 100; // choose random number between 0 and 99
      bool change = false;
      switch (iteration) {
        case 0: {
          change = add_something(item_class);
          add_iterations++;
          break;
        }
        case 1: {
          change = remove_something(item_class);
          remove_iterations++;
          break;
        }
        case 2: {
          change = modify_something(item_class);
          modify_iterations++;
          break;
        }
      }
    }
    if (add_iterations > remove_iterations && add_iterations > modify_iterations) {
      dse_change = "add";
      if (rand() % 100 <= temperature * temperature)
        _ssModel.io_ports += rand() % (4 - _ssModel.io_ports + 1);
    } else if (remove_iterations > add_iterations && remove_iterations > modify_iterations) {
      dse_change = "remove";
      if (rand() % 100 <= temperature * temperature)
        _ssModel.io_ports = rand() % _ssModel.io_ports + 1;
    } else {
      dse_change = "change";
      if (rand() % 100 <= temperature)
        _ssModel.io_ports = rand() % 4 + 1;
    }
  }

  /**
   * @brief Adds some hardware to the ADG
   * 
   * 65% chance to add a random link
   * 15% chance to add a random switch
   * 10% chance to add a random FU
   * 5% chance to add a random input vport
   * 5% chance to add a random output vport
   * 
   * @param item_class A random number with the item to modified
   * @return bool whether something was succesfully added in this call.
   */
  bool add_something(int item_class) {
    std::ostringstream s;
    auto* sub = _ssModel.subModel();
    if (item_class < 65) { // 65% to add a random link
      if (sub->node_list().empty()) return false;
      int src_node_index = rand() % sub->node_list().size();
      int dst_node_index = rand() % sub->node_list().size();
      ssnode* src = sub->node_list()[src_node_index];
      ssnode* dst = sub->node_list()[dst_node_index];
      if (dynamic_cast<ssvport*>(src) && src->out_links().empty()) {
        return false;
      }
      if (dynamic_cast<ssvport*>(dst) && dst->in_links().empty()) {
        return false;
      }
      if (src == dst) return false;

      
      sub->add_link(src, dst);
      check_stated(src);
      check_stated(dst);

      s << "add link from " << src->name() << " to " << dst->name();
    } else if (item_class < 80) { // 15% to add a random switch
      ssswitch* sw = sub->add_switch();
      add_random_edges_to_node(sw, 1, 5, 1, 5);
      s << "add switch " <<  sw->name() << " ins/outs: " << sw->in_links().size() << "/" << sw->out_links().size();
    } else if (item_class < 90) { // 10% to add a random FU
      // Randomly pick an FU type from the set
      auto& fu_defs = _ssModel.fu_types;
      if (fu_defs.empty()) return false;
      ssfu* fu = sub->add_fu();
      int fu_def_index = rand() % fu_defs.size();
      Capability* def = fu_defs[fu_def_index];
      fu->fu_type_ = *def;
      s << "add function unit " << fu->name();
    } else if (item_class < 95) { //  5% to add a random input vector port
      ssvport* vport = sub->add_vport(true);
      add_random_edges_to_node(vport, 0, 1, 5, 12);
      s << "adding input vport " << vport->name();
    } else { // 5% to add a random output vector port
      ssvport* vport = sub->add_vport(false);
      add_random_edges_to_node(vport, 5, 12, 0, 1);
      s << "adding output vport " << vport->name();
    }
    dse_changes_log.push_back(s.str());
    for_each_sched([&](Schedule& sched) { sched.allocate_space(); });
    verify_strong();
    return true;
  }

  /**
   * @brief Removes something from the ADG
   * 
   * 60% Chance to remove a random link
   * 15% Chance to remove a random switch
   * 15% Chance to remove a random FU
   * 5% Chance to remove a random input vector port
   * 5% Chance to remove a random output vector port
   * 
   * @param item_class A random number with the item to remove
   * @return bool whether something was succesfully removed in this call.
   */
  bool remove_something(int item_class) {
    std::ostringstream s;
    auto* sub = _ssModel.subModel();
    if (item_class < 60) { // 60% to remove a random link
      if (sub->link_list().empty()) return false;
      int index = non_uniform_random(sub->link_list(), unused_links);
      sslink* l = sub->link_list()[index];
      s << "remove link " << l->name();
      dse_changes_log.push_back(s.str());
      delete_link(l);
    } else if (item_class < 75) { // 15% to remove a random switch
      if (sub->switch_list().empty()) return false;
      int index = non_uniform_random(sub->switch_list(), unused_nodes);
      ssswitch* sw = sub->switch_list()[index];
      s << "remove switch " << sw->name();
      dse_changes_log.push_back(s.str());
      delete_node(sw, true, false, true);
    } else if (item_class < 90) { // 15% to remove a random FU
      if (sub->fu_list().empty()) return false;
      int index = non_uniform_random(sub->fu_list(), unused_nodes);
      ssfu* fu = sub->fu_list()[index];
      s << "remove function unit " << fu->name();
      dse_changes_log.push_back(s.str());
      delete_node(fu, true, false, false);
    } else if (item_class < 95) { // 5% to remove a random input vector port
      if (sub->input_list().size() <= 1) return false;
      int index = non_uniform_random(sub->input_list(), unused_nodes);
      ssvport* vport = sub->input_list()[index];
      s << "remove input vport "<< vport->name();
      dse_changes_log.push_back(s.str());
      delete_node(vport, false, false, false);
    } else { // 5% to remove a random output vector port
      if (sub->output_list().size() <= 1) return false;
      int index = non_uniform_random(sub->output_list(), unused_nodes);
      ssvport* vport = sub->output_list()[index];
      s << "remove output vport "<< vport->name();
      dse_changes_log.push_back(s.str());
      delete_node(vport, false, false, false);
    }
    verify_strong();
    while (delete_hangers()) {  }
    return true;
  }

  /**
   * @brief Modifies a node in the ADG
   * 
   * 15% to change flow control
   * 5% to change the utilization of a functional unit
   * 30% chance to change the fifo depth
   * 20% chance to change a functional units fu type
   * 20% chance to change a nodes granularity
   * 5% to change input vport stated
   * 5% to change output vport stated
   * 
   * @param item_class A random number with the item to modified
   * @return bool whether something was succesfully modified in this call.
   */
  bool modify_something(int item_class) {
    std::ostringstream s;
    auto* sub = _ssModel.subModel();
    if (item_class < 15) { // 15% to change flow control
      if (sub->node_list().empty()) return false;
      int node_index = rand() % sub->node_list().size();
      ssnode* node = sub->node_list()[node_index];
      if (dynamic_cast<ssvport*>(node)) return false;
      s << "change Node " << node->name() << " flow control from " << node->flow_control() << " to " << !node->flow_control();

      node->flow_control(!node->flow_control());
      
      if (!node->flow_control()) {
        for_each_sched([&](Schedule& sched) {
          for (int slot = 0; slot < sched.num_slots(node); ++slot) {
            for (auto& p : sched.dfg_nodes_of(slot, node)) {
              if (sched.needs_dynamic[p.first->id()]) {
                sched.unassign_dfgnode(p.first);
              }
            }
          }
        });
      }
    } else if (item_class < 20) { // 5% to change the name of a node
      return false;

      if (sub->fu_list().empty()) return false;
      // Modify FU utilization
      int diff = rand() % 16 - 8;
      if (diff == 0) return false;
      int fu_index = rand() % sub->fu_list().size();
      ssfu* fu = sub->fu_list()[fu_index];
      int old_util = fu->max_util();
      int new_util = std::max(1, old_util + diff);
      if (diff < -4) new_util = 1;

      if (old_util == 1 && new_util > 1) {
        fu->flow_control(true);
      }

      fu->max_util(new_util);
      s << "change FU " << fu->name() << " util from " << old_util << " to " << new_util;

      // if we are constraining the problem, then lets re-assign anything
      // mapped to this FU
      if (new_util < old_util) {
        unassign_node(fu);
      }

    } else if (item_class < 50) { // 30% to change fu fifo depth
      if (sub->fu_list().empty()) return false;
      int diff = -(rand() % 3 + 1);
      if (rand() & 1) diff *= -1;
      int fu_index = rand() % sub->fu_list().size();
      ssfu* fu = sub->fu_list()[fu_index];
      int old_delay_fifo_depth =  fu->delay_fifo_depth();
      // Constrain delay fifo depth between 1 and 32
      int new_delay_fifo_depth = std::min(std::max(1, old_delay_fifo_depth + diff), 32);
      fu->max_delay(new_delay_fifo_depth);

      s << "change FU " << fu->name() << " fifo depth from " << old_delay_fifo_depth << " to " << new_delay_fifo_depth;
      // if we are constraining the problem, then lets re-assign anything
      // mapped to this FU
      for_each_sched([&](Schedule& sched) {
        for (int slot = 0; slot < sched.num_slots(fu); ++slot) {
          for (auto& p : sched.dfg_nodes_of(slot, fu)) {
            for (auto& op : p.first->ops()) {
              for (auto eid : op.edges) {
                auto* elem = &sched.ssdfg()->edges[eid];
                if (sched.edge_delay(elem) > fu->delay_fifo_depth()) {
                  sched.unassign_dfgnode(p.first);
                }
              }
            }
          }
        }
      });
    } else if (item_class < 70) { // 20% to change a node's fu_type
      if (sub->fu_list().empty()) return false;
      int index = rand() % sub->fu_list().size();
      auto fu = sub->fu_list()[index];

      if (rand() & 1) {
        s << "add FU " << fu->name() << " operation";
        dse_changes_log.push_back(s.str());

        fu->fu_type_ = *ss_model()->fu_types[rand() % ss_model()->fu_types.size()];
      } else if (fu->fu_type_.capability.size() > 1) {
        int j = rand() % fu->fu_type_.capability.size();
        s << "remove FU " << fu->name() << " operation " << dsa::name_of_inst(fu->fu_type_.capability.at(j).op);
        dse_changes_log.push_back(s.str());
        fu->fu_type_.Erase(j);
      }

      for_each_sched([&](Schedule& sched) {
        for (int slot = 0; slot < sched.num_slots(fu); ++slot) {
          for (auto& p : sched.dfg_nodes_of(slot, fu)) {
            if (auto inst = dynamic_cast<dsa::dfg::Instruction*>(p.first)) {
              if (!fu->fu_type_.Capable(inst->inst())) {
                sched.unassign_dfgnode(p.first);
              }
            }
          }
        }
      });
    } else if (item_class < 90) { // 20% to change node granularity
      return false;
      int index = rand() % sub->node_list().size();
      auto fu = sub->node_list()[index];
      int new_one = (1 << (rand() % 4)) * 8;
      while (new_one == fu->granularity() || new_one > fu->datawidth()) {
        new_one = (1 << (rand() % 4)) * 8;
      }
      for_each_sched([fu, this, new_one](Schedule& sched) {
        auto &np = sched.node_prop()[fu->id()];
        for (auto &slot : np.slots) {
          auto passthrus = slot.passthrus;
          for (auto edge : passthrus) {
            sched.unassign_edge(edge);
          }
          auto vertices = slot.vertices;
          for (auto v : vertices) {
            sched.unassign_dfgnode(v.first);
          }
        }
        np.slots.resize(fu->datawidth() / new_one);
        for (auto &link : fu->out_links()) {
          unassign_link(link);
          auto &lp = sched.link_prop()[link->id()];
          for (auto &slot : sched.link_prop()[link->id()].slots) {
            DSA_CHECK(slot.edges.empty());
          }
          lp.slots.resize(link->source()->datawidth() / new_one);
          link->subnet.resize(link->bitwidth() / new_one);
          if (link->subnet.size() >= 2) {
            link->subnet[1] = ~0ull >> (64 - link->bitwidth());
          }
        }
      });

      s << "change FU " << fu->name() << " granularity from " << fu->granularity() << " to " << new_one;
      fu->granularity(new_one);
    } else if (item_class < 95) { // 5% to change a input Vports stated
      int index = rand() % sub->input_list().size();
      auto vport = sub->input_list()[index];

      if (vport->out_links().size() < 2) return false;
      
      vport->vp_stated(!vport->vp_stated());
      unassign_node(vport);
      s << "change Input vport " << vport->name() << " state from " << !vport->vp_stated() << " to " << vport->vp_stated();
    } else { // 5% to change a output Vports stated
      int index = rand() % sub->output_list().size();
      auto vport = sub->output_list()[index];

      if (vport->in_links().size() < 2) return false;
      
      vport->vp_stated(!vport->vp_stated());
      unassign_node(vport);
      s << "change Output vport " << vport->name() << " state from " << !vport->vp_stated() << " to " << vport->vp_stated();
    }
    dse_changes_log.push_back(s.str());
    return true;
  }


  /**
   * @brief Lambda function to loop through all schedules
   * 
   * @param f 
   */
  void for_each_sched(const std::function<void(Schedule&)>& f) {
    for (auto& ws : workload_array) {
      for (Schedule& sched : ws.sched_array) {
        f(sched);
      }
    }
  }

  /**
   * @brief Construct a new Codesign Instance object
   * 
   * @param c other codesign instance
   * @param from_scratch whether to restart from scratch
   */
  CodesignInstance(const CodesignInstance& c, bool from_scratch) : _ssModel(c._ssModel){
    weight = c.weight;
    
    if (from_scratch) {
      for (auto& work : c.workload_array) {
        workload_array.emplace_back();
        for (auto& elem : work.sched_array) {
          workload_array.back().sched_array.emplace_back(ss_model(), elem.ssdfg());
        }
      }
    } else {
      workload_array = c.workload_array;
      for_each_sched([&](Schedule& sched) {
        // replace this ssmodel with the copy
        sched.swap_model(_ssModel.subModel());
        sched.set_model(&_ssModel);
      });
    }

    unused_nodes = c.unused_nodes;
    unused_links = c.unused_links;

    dse_obj();
  }

  /**
   * @brief Unassigns a link from all schedules
   * 
   * @param link the link to unassign
   */
  void unassign_link(sslink* link) {
    for_each_sched([&](Schedule& sched) {
      int slots = link->source()->datawidth() / link->source()->granularity();
      DSA_LOG(UNASSIGN) << "Unassign link " << link->id() << " with " << slots << " slots";
      for (int slot = 0; slot < slots; ++slot) {
        for (auto& p : sched.dfg_edges_of(slot, link)) {
          // TODO: consider just deleteting the edge, and having the scheduler
          // try to repair the edge schedule -- this might save some time
          // sched.unassign_edge(p.first);
          sched.unassign_dfgnode(sched.ssdfg()->edges[p.eid].def());
          sched.unassign_dfgnode(sched.ssdfg()->edges[p.eid].use());
        }
      }
    });
  }

  /**
   * @brief Unassigns a node from all schedules
   * 
   * @param node the node to unassign 
   */
  void unassign_node(ssnode* node) {
    for_each_sched([&](Schedule& sched) {
      for (int slot = 0; slot < sched.num_slots(node); ++slot) {
        for (auto& p : sched.dfg_nodes_of(slot, node)) {
          if (p.first)
            sched.unassign_dfgnode(p.first); 
        }
      }
    });
  }

  /**
   * @brief Checks whether vector port needs to change its stated value
   * 
   * @param node the node to check
   * @param deleting whether a link will be deleted from the schedule
   */
  void check_stated(ssnode* node, bool deleting=false) {
    if (auto vport = dynamic_cast<ssvport*>(node)) {
      bool prev_stated = vport->vp_stated();
      if (vport->in_links().size() + vport->out_links().size() > (1 + deleting)) {
        vport->vp_stated(true);
      } else {
        vport->vp_stated(false);
      }

      if (deleting)
        unassign_node(vport);
        
      else if (prev_stated != vport->vp_stated())
        unassign_node(vport);
    }
  }

  /**
   * @brief Delete a link on every schedule
   * 
   * @param link link to delete
   */
  void delete_link(sslink* link) {
    verify();
    
    auto* sub = _ssModel.subModel();
    check_stated(link->source(), true);
    check_stated(link->sink(), true);

    // remove it from every schedule
    unassign_link(link);
    sub->delete_link(link->id());
    for_each_sched([&](Schedule& sched) { 
      sched.remove_link(link->id());
    });

    delete link;
    verify();
  }

  double dse_sched_obj(Schedule* sched) {
    if (!sched) return 0.1;
    // YES, I KNOW THIS IS A COPY OF SCHED< JUST A TEST FOR NOW
    SchedStats s;
    int num_left = sched->num_left();
    bool succeed_sched = (num_left == 0);
    auto &delay_violation = sched->violation_penalty;
    delay_violation = std::make_pair(0, 0);

    sched->get_overprov(s.ovr, s.agg_ovr, s.max_util);
    sched->fixLatency(s.lat, s.latmis, delay_violation);

    int violation = sched->violation();
    double performance = sched->estimated_performance();
    
    if (succeed_sched) {
      double eval = performance;
      if (delay_violation.second + delay_violation.first != 0)
        eval *= ((double)delay_violation.second  / (delay_violation.second + delay_violation.first));

      eval /= std::max(1.0 + s.ovr, sqrt(s.agg_ovr));

      return eval;
    }

    return -num_left;
  }

  float dse_obj() {
    double total_score = 1.0;
    res.resize(workload_array.size());

    bool meaningful = false;
    double failure = 0;
    std::vector<std::vector<std::pair<std::string, double>>> _dfg_performances;
    std::vector<double> _workload_performances;
    std::vector<double> _workload_weights;

    for (int i = 0; i < workload_array.size(); ++i) {
      auto& ws = workload_array[i];
      double score = (double)1e-3;
      res[i] = nullptr;
      std::vector<std::pair<std::string, double>> performances;      
      for (Schedule& sched : ws.sched_array) {
        double new_score = dse_sched_obj(&sched);

        if (new_score >= score) {
          score = new_score;
          res[i] = &sched;
          meaningful = true;
        }
        if (new_score < 0.0) {
         
          failure = std::min(failure, new_score);
        }
        performances.push_back(std::make_pair(sched.ssdfg()->filename, new_score));
      }
      _dfg_performances.push_back(performances);
      _workload_performances.push_back(score);
      _workload_weights.push_back(weight[i]);
      total_score *= score * weight[i];
    }

    dfg_performances = _dfg_performances;
    workload_weights = _workload_weights;
    workload_performances = _workload_performances;

    if (!meaningful) {
      return exp(failure);
    }

    total_score = pow(total_score, (1.0 / workload_array.size()));

    auto estimated = dsa::adg::estimation::EstimatePowerAera(&_ssModel).sum();
    estimated->normalize();

    performance = total_score;
    normalized_resources = estimated->constrained_resource(0);

    double final_score = 0.0;
    double multiplier = 1.0;
    
    // Only Search Logic Lut, Ram Lut, FF (all other resources are fixed)
    for (int i = 0; i < 3; ++i) {
      final_score += (total_score / estimated->constrained_resource(i)) * multiplier;
      multiplier *= 0.1;
    }

    return final_score;
  }

  float weight_obj() { return dse_obj(); }


  /**
   * @brief Gets the utilization ratio of the current codesign instance
   * 
   * @return std::tuple<float, float, float>, the overall, node, and
   * link utilization ratio, respectively of current codesign instance
   */
  std::tuple<float, float, float> utilization() {
    // If DSE_OBJ is horrible return nothing
    if (abs(dse_obj()) < (1.0 + 1e-3)) {
      return {0, 0, 0};
    }
    
    // Check to see if there is a working schedule
    bool meaningful = false;
    for (auto sched : res) {
      if (sched != nullptr) {
        meaningful = true;
        break;
      }
    }

    // If there is no schedules, return nothing
    if (!meaningful) {
      return {0, 0, 0};
    }

    auto sub = _ssModel.subModel();
    

    unused_nodes = std::vector<bool>(sub->node_list().size(), true);
    unused_links = std::vector<bool>(sub->link_list().size(), true);

    for (size_t i = 0; i < res.size(); ++i) {
      auto sched = res[i];
      if (sched != nullptr) {
        for (auto& ep : sched->edge_prop()) {
          for(auto link_iter = ep.links.begin(); link_iter < ep.links.end(); link_iter++) {
            auto link = *link_iter;
            unused_links[link.second->id()] = false;
            unused_nodes[link.second->sink()->id()] = false;
            unused_nodes[link.second->source()->id()] = false;
          }
        }
      }
    }

    float node_count = unused_nodes.size();
    float link_count = unused_links.size();
    float unused_node_count = 0;
    float unused_link_count = 0;

    // Go through and count how many are true
    for (int i = 0; i < unused_nodes.size(); i++) {
      if (!unused_nodes[i]) unused_node_count++;
    }

    for (int i = 0; i < unused_links.size(); i++) {
      if (!unused_links[i]) unused_link_count++;
    }

    // do calculations to determine unitilization
    float overall_usage = (unused_node_count + unused_link_count) / (node_count + link_count);
    float nodes_usage = unused_node_count / node_count;
    float links_usage = unused_link_count / link_count;
    return {overall_usage, nodes_usage, links_usage};
  }

  void dump_breakdown(bool verbose) {
    auto estimated = dsa::adg::estimation::EstimatePowerAera(ss_model());
    estimated.Dump(std::cout);
    if (verbose) {
      for (int i = 0, n = res.size(); i < n; ++i) {
        if (!res[i]) {
          std::cout << "[skip]" << std::endl;
          continue;
        }
        auto performance = dse_sched_obj(res[i]);
        std::cout << res[i]->ssdfg()->filename << ": " << performance << std::endl;
      }
    }
  }

  /**
   * @brief Checks to see if a flip-flop can be removed without causing
   * a cycle in the hardware graph
   * 
   * @param n the node to check 
   * @return true if there will be no cycle created by removing the node
   * @return false if there will be a cycle created by removing the node
   */
  bool check_remove_flipflop(ssnode* n) {
    std::vector<bool> visited(ss_model()->node_list().size(), false);

    // Check upstream for cycles
    if (!check_remove_helper(n, visited, true))
      return false;
    
    // Check downstream for cycles
    if (!check_remove_helper(n, visited, false))
      return false;
    
    // No cycles found, so removing flipflow is fine
    return true;
  }

 private:
  /**
  * @brief Helper for check_remove_flipflop. Will recursively check all nodes
  * to see if there is a cycle
  * 
  * @param n the node to check
  * @param visited the set of nodes that have been visited
  * @param down whether to check the upstream or downstream
  * @return true if there is no cycle from removing this node
  * @return false if there is a cycle from removing this node
  */
  bool check_remove_helper(ssnode* n, std::vector<bool>& visited, bool down) {
    auto links = n->in_links();
    if (down) {
      links = n->out_links();
    }

    for (auto link : links) {
      auto node = link->source();
      if (down) {
        node = link->sink();
      }
      
      // If there is a delay then we can just continue
      if (node->max_delay() > 0) 
        continue;

      // If we have already visited this node then we have a cycle
      if (visited[node->id()])
        return false;
      visited[node->id()] = true;

      // recursively check all the nodes connected to this node
      if (!check_remove_helper(node, visited, down))
        return false;
    }
    // No cycle found
    return true;
  }

  /**
   * @brief Collapses the links through a node, of a given edge
   * 
   * @param n node to collapse
   * @param edge edge to collapse from
   * @param sched currently used schedule
   */
  void collapse_edge_links(ssnode* n, dsa::dfg::Edge edge, Schedule& sched) {
    auto* sub = _ssModel.subModel();
    auto& links = sched.links_of(&edge);
    for (auto it = links.begin(); it != links.end(); ++it) {
      //Check if link is the node to collapse
      if (it->second->sink()->id() == n->id() && std::next(it) != links.end()) {
        auto src = it->second->source();
        auto dst = std::next(it)->second->sink();

        sub->add_link(src, dst);
        check_stated(src);
        check_stated(dst);
        return;
      }
      
    }
  }

  /**
   * @brief Adds a fifo depth to all nodes after a given node in the given edge
   * 
   * TODO: Only Check the last node in an endge
   * 
   * @param fifos set of fu's that have already have their fifo depth added
   * @param n node to collapse
   * @param edge edge to check for functional units
   * @param sched current schedule to use
   */
  void add_fifo_edge(std::unordered_set<ssfu*>& fifos, ssnode* n, dsa::dfg::Edge edge, Schedule& sched) {
    bool found_node = false;
    for (auto link : sched.links_of(&edge)) {
      if (found_node) {
        if (auto fu = dynamic_cast<ssfu*>(link.second->sink())) {
          // Check if node is already in the set or if it has too large a delay fifo
          if (fu->delay_fifo_depth() < 32 && fifos.find(fu) == fifos.end()) {
            fu->max_delay(fu->delay_fifo_depth() + 1);
            fifos.insert(fu);
            // We don't need to reassign these to the schedule

            return;
          }
        }
      } else if (link.second->sink()->id() == n->id()) {
        found_node = true;
      }
    }
  }

  /**
   * @brief Collapses a vport
   * TODO Fix this
   * @param vport vport to collapse
   */
  void collapse_vport(ssvport* vport) {
    // Get Links used in schedules
    std::unordered_set<sslink*> links = useful_vport_links(vport);
    // If no links are used in any schedule, we can just remove vport
    if (links.size() == 0) return;

    auto* sub = _ssModel.subModel();
    ssvport* other_vport;
    
    if (vport->out_links().size() == 0) {
      // Find another random VPort to collapse this one into
      while (other_vport == nullptr || other_vport->id() == vport->id()) {
        int index = non_uniform_random(sub->output_list(), unused_nodes);
        other_vport = sub->output_list()[index];
      }
      // Check if width will go up by more than power of two
      if (log2(other_vport->in_links().size()) == log2(links.size() + other_vport->in_links().size())) {
        // Simply add links to this vport
        for (auto& link : links) {
          sub->add_link(link->source(), other_vport);
        }
      } else {
        // Add Switches so VPort retains current power of two width
        if (other_vport->in_links().size() > links.size()) {
          for (auto& link : other_vport->in_links()) {
            if (links.size() > 0) {
              auto link_to_add = links.begin();
              ssswitch* sw = sub->add_switch();
              sub->add_link(sw, other_vport);
              sub->add_link(link->source(), sw);
              sub->add_link((* link_to_add)->source(), sw);
              delete_link(link);
              links.erase(link_to_add);
            }
          }
        } else {
          // First add
          for (auto& link : other_vport->in_links()) {
            ssswitch* sw = sub->add_switch();
            sub->add_link(sw, other_vport);
            sub->add_link(link->source(), sw);
            delete_link(link);
          }
          // Now add to the switches
          int link_index = 0;
          for (auto& link : links) {
            sub->add_link(link->source(), other_vport->in_links()[link_index]->source());
            link_index++;
            if (link_index == other_vport->in_links().size()) {
              link_index = 0;
            }
          }
        }
      }
    } else {
      // Find another random VPort to collapse this one into
      while (other_vport == nullptr || other_vport->id() == vport->id()) {
        int index = non_uniform_random(sub->input_list(), unused_nodes);
        other_vport = sub->input_list()[index];
      }
      // Check if width will go up by more than power of two
      if (log2(other_vport->out_links().size()) == log2(links.size() + other_vport->out_links().size())) {
        // Simply add links to this vport
        for (auto& link : links) {
          sub->add_link(other_vport, link->sink());
        }
      } else {
        // Add Switches so VPort retains current power of two width
        if (other_vport->out_links().size() > links.size()) {
          for (auto& link : other_vport->out_links()) {
            if (links.size() > 0) {
              auto link_to_add = links.begin();
              ssswitch* sw = sub->add_switch();
              sub->add_link(other_vport, sw);
              sub->add_link(sw, link->sink());
              sub->add_link(sw, (* link_to_add)->sink());
              delete_link(link);
              links.erase(link_to_add);
            }
          }
        } else {
          // First add
          for (auto& link : other_vport->out_links()) {
            ssswitch* sw = sub->add_switch();
            sub->add_link(sw, other_vport);
            sub->add_link(sw, link->sink());
            delete_link(link);
          }
          // Now add to the switches
          int link_index = 0;
          for (auto& link : links) {
            sub->add_link(other_vport->out_links()[link_index]->sink(), link->sink());
            link_index++;
            if (link_index == other_vport->out_links().size()) {
              link_index = 0;
            }
          }
        }
      }
    }
    std::ostringstream s;
    s << dse_changes_log.back() << " collapsed vport into " << other_vport->name();
    dse_changes_log.back() = s.str();
  }

  std::unordered_set<sslink*> useful_vport_links(ssvport* n) {
    std::unordered_set<sslink*> links;
    for_each_sched([&](Schedule& sched) {
      auto link_prop = sched.link_prop(); 
      //Output Vector Port
      if (n->out_links().size() == 0) {
        for (auto& link : n->in_links()) {
          auto link_slice = link_prop[link->id()];
          if (link_slice.slots.size() > 0) {
            links.insert(link);
          }
        }
      } else {
        for (auto& link : n->out_links()) {
          auto link_slice = link_prop[link->id()];
          if (link_slice.slots.size() > 0) {
            links.insert(link);
          }
        }
      }
    });
    return links;
  }
  
  /**
   * @brief Deletes a node
   * 
   * When we delete a hardware element, we need to:
   *  1. deschedule anything that was assigned to that element
   *  2. remove the concept of that element from the schedule (consistency)
   *  3. remove the element from the hardware description
   * 
   * @param n node to delete
   * @param collapse_links whether to collapse the links of node
   * @param forward_vport whether to forward the links of vport
   * @param add_fifo whether to add fifo depths to all nodes after
   */
  void delete_node(ssnode* n, bool collapse_links=false, bool forward_vport=false, bool add_fifo=false) {

    auto* sub = _ssModel.subModel();
    
    std::unordered_set<ssfu*> fifos;
    int links_collapsed = 0;
    
    for_each_sched([&](Schedule& sched) {
      auto link_prop = sched.link_prop(); 
      for (auto& link : n->out_links()) {
        auto link_slice = link_prop[link->id()];
        for (auto &slot : link_slice.slots) {
          auto edges = slot.edges;
          for (auto edge_slice : edges) {
            auto& edge = sched.ssdfg()->edges[edge_slice.eid];
            if (add_fifo && sched.edge_delay(&edge) <= 0) {
              add_fifo_edge(fifos, n, edge, sched);
            }
            if (collapse_links) {
              collapse_edge_links(n, edge, sched);
              links_collapsed++;
            }
          }
        }
      }
    });

    if (collapse_links && dynamic_cast<ssvport*>(n)) {
      //collapse_vport(dynamic_cast<ssvport*>(n));
    }

    if (links_collapsed > 0) {
      std::ostringstream s;
      s << dse_changes_log.back() << " collapsed " << links_collapsed << " links";
      dse_changes_log.back() = s.str();
    }

    if (fifos.size() > 0) {
      std::ostringstream s;
      s << dse_changes_log.back() << " fifo depth added for ";
      for(auto it = fifos.begin(); it != fifos.end(); ++it) {
        s << "FU_" << (*it)->id();
        if (std::next(it) != fifos.end())
          s << ",";
      }
      dse_changes_log.back() = s.str();
    }

    for_each_sched([&](Schedule& sched) { sched.allocate_space(); });

    verify();

    while (n->in_links().size() > 0) {
      delete_link(n->in_links()[0]);
    }

    while (n->out_links().size() > 0) {
      delete_link(n->out_links()[0]);
    }
    
    for_each_sched([&](Schedule& sched) {
      for (int slot = 0; slot < sched.num_slots(n); ++slot) {
        for (auto& p : sched.dfg_nodes_of(slot, n)) {
          sched.unassign_dfgnode(p.first);
        }
      }
    });

    sub->delete_node(n->id());
    for_each_sched([&](Schedule& sched) { sched.remove_node(n->id()); });

    delete n;
    verify();
  }
};

namespace dsa {

void DesignSpaceExploration(SSModel &ssmodel, const std::string &pdg_filename);
} // namespace dsa
