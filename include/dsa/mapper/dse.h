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
    if (!cout)
      s << "\"";
    for(auto it = dse_changes_log.begin(); it != dse_changes_log.end(); ++it) {
      s << *it;
      if (cout)
        s << std::endl;
      else if (std::next(it) != dse_changes_log.end())
        s << ",";
    }
    if (!cout)
      s << "\"";
    return s.str();
  }

  std::string get_workload_weights(bool cout=false) {
    std::ostringstream s;
    if (!cout)
      s << "\"";
    for(auto it = workload_weights.begin(); it != workload_weights.end(); ++it) {
      s << *it;
      if (cout)
        s << std::endl;
      else if (std::next(it) != workload_weights.end())
        s << ",";
    }
    if (!cout)
      s << "\"";
    return s.str();
  }

  std::string get_workload_performances(bool cout=false) {
    std::ostringstream s;
    if (!cout)
      s << "\"";
    for(auto it = workload_performances.begin(); it != workload_performances.end(); ++it) {
      s << *it;
      if (cout)
        s << std::endl;
      else if (std::next(it) != workload_performances.end())
        s << ",";
    }
    if (!cout)
      s << "\"";
    return s.str();
  }

  std::string get_dfg_performances(bool cout=false) {
    std::ostringstream s;
    if (!cout)
      s << "\"";
    for(auto it = dfg_performances.begin(); it != dfg_performances.end(); ++it) {
      for(auto next = it->begin(); next != it->end(); ++next) {
        s << next->first << ": " << next->second;
        if (cout)
          s << std::endl;
        else if (std::next(it) != dfg_performances.end() || std::next(next) != it->end())
          s << ",";
      }
    }
    if (!cout)
      s << "\"";
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

  // Delete FUs, Switches, and Vports that can't possible be useful
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
    utilization();
    while (delete_hangers()) {  }

    //TODO: Convert from set once memory is added
    std::unordered_set<sslink*> links_to_delete;
    std::unordered_set<ssnode*> nodes_to_delete;

    auto* sub = _ssModel.subModel();

    for (int j = 0, n = sub->link_list().size(); j < n; ++j) {
      if (unused_links[j])
        links_to_delete.insert(sub->link_list()[j]);
    }
    for (int j = 0, n = sub->switch_list().size(); j < n; ++j) {
      if (unused_nodes[sub->switch_list()[j]->id()]) 
        nodes_to_delete.insert(sub->switch_list()[j]);
    }
    for (int j = 0, n = sub->fu_list().size(); j < n; ++j) {
      if (unused_nodes[sub->fu_list()[j]->id()]) 
        nodes_to_delete.insert(sub->fu_list()[j]);
    }
    for (int j = 0, n = sub->input_list().size(); j < n; ++j) {
      if (unused_nodes[sub->input_list()[j]->id()]) 
        nodes_to_delete.insert(sub->input_list()[j]);
    }
    for (int j = 0, n = sub->output_list().size(); j < n; ++j) {
      if (unused_nodes[sub->output_list()[j]->id()]) 
        nodes_to_delete.insert(sub->output_list()[j]);
    }

    for (auto& link : links_to_delete) {
      delete_link(link);
    }
    for (auto& node : nodes_to_delete) {
      delete_node(node);
    }

    // delete all hangers arising from situation
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

  /*
  * Adds a new node to the ADG.
  * Returns true if the node was added, false otherwise.
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

  /*
  * Deletes a node from the ADG.
  * Returns true if a node was deleted, false otherwise.
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
      if (sub->input_list().size() == 0) return false;
      int index = non_uniform_random(sub->input_list(), unused_nodes);
      ssvport* vport = sub->input_list()[index];
      s << "remove input vport "<< vport->name();
      dse_changes_log.push_back(s.str());
      delete_node(vport, false, false, false);
    } else { // 5% to remove a random output vector port
      if (sub->output_list().size() == 0) return false;
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

  /*
  * Modifies a node in the ADG.
  * Returns true if a node was modified, false otherwise.
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
    } else if (item_class < 30) { // 15% to change the name of a node
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
        for_each_sched([&](Schedule& sched) {
          for (int slot = 0; slot < sched.num_slots(fu); ++slot) {
            for (auto& p : sched.dfg_nodes_of(slot, fu)) {
              sched.unassign_dfgnode(p.first);
            }
          }
        });
      }

    } else if (item_class < 60) { // 30% to change fu fifo depth
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

    } else if (item_class < 80) { // 20% to change a node's fu_type
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
    } else { // 20% to change node granularity
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
    }
    dse_changes_log.push_back(s.str());
    return true;
  }

  void for_each_sched(const std::function<void(Schedule&)>& f) {
    for (auto& ws : workload_array) {
      for (Schedule& sched : ws.sched_array) {
        f(sched);
      }
    }
  }

  // Overide copy constructor to enable deep copies
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
    
   /*
    for (auto& work : c.workload_array) {
      workload_array.emplace_back();
      for (auto& elem : work.sched_array) {
        workload_array.back().sched_array.emplace_back(elem, &_ssModel);
      }
    }
    */

    unused_nodes = c.unused_nodes;
    unused_links = c.unused_links;
    dse_obj();
  }

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

  // Delete link on every schedule
  void delete_link(sslink* link) {
    verify();
    
    auto* sub = _ssModel.subModel();
    // remove it from every schedule
    unassign_link(link);
    sub->delete_link(link->id());
    for_each_sched([&](Schedule& sched) { sched.remove_link(link->id()); });
    delete link;
    verify();
  }

  double dse_sched_obj(Schedule* sched) {
    if (!sched) return 0.1;
    // YES, I KNOW THIS IS A COPY OF SCHED< JUST A TEST FOR NOW
    SchedStats s;
    int num_left = sched->num_left();
    bool succeed_sched = (num_left == 0);
    std::pair<int, int> delay_violation = std::make_pair(0, 0);

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

  std::tuple<float, float, float> utilization() {
    if (abs(dse_obj()) < (1.0 + 1e-3) || !res[0]) {
      return {0, 0, 0};
    }
    unused_nodes =
        std::vector<bool>(res[0]->ssModel()->subModel()->node_list().size(), true);
    unused_links =
        std::vector<bool>(res[0]->ssModel()->subModel()->link_list().size(), true);
    for (size_t i = 0; i < res.size(); ++i) {
      if (!res[i]) {
        return {0, 0, 0};
      }
      for (size_t j = 0; j < res[i]->node_prop().size(); ++j) {
        if (!unused_nodes[j]) {
          continue;
        }
        for (int k = 0; k < 8; ++k) {
          if (!res[i]->node_prop()[j].slots[k].vertices.empty() ||
              res[i]->node_prop()[j].slots[k].passthrus.size()) {
            unused_nodes[j] = false;
          }
        }
      }
      for (size_t j = 0; j < res[i]->link_prop().size(); ++j) {
        for (int k = 0; k < 8; ++k) {
          if (!res[i]->link_prop()[j].slots[k].edges.empty()) {
            unused_nodes[res[i]->ssModel()->subModel()->link_list()[j]->source()->id()] =
                false;
            unused_nodes[res[i]->ssModel()->subModel()->link_list()[j]->sink()->id()] =
                false;
            unused_links[j] = false;
          }
        }
      }
    }
    int cnt_nodes = 0, cnt_links = 0;
    for (auto elem : unused_nodes) cnt_nodes += elem;
    for (auto elem : unused_links) cnt_links += elem;
    float overall =
        (float)(cnt_nodes + cnt_links) / (unused_nodes.size() + unused_links.size());
    float nodes_ratio = (float)(cnt_nodes) / (unused_nodes.size());
    float links_ratio = (float)(cnt_links) / (unused_links.size());
    return {overall, nodes_ratio, links_ratio};
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

 private:

  void collapse_edge_links(ssnode* n, dsa::dfg::Edge edge, Schedule& sched) {
    auto* sub = _ssModel.subModel();
    auto& links = sched.links_of(&edge);
    for (auto it = links.begin(); it != links.end(); ++it) {
      //Check if link is the node to collapse
      if (it->second->sink()->id() == n->id() && std::next(it) != links.end()) {
        sub->add_link(it->second->source(), std::next(it)->second->sink());
        return;
      }
      
    }
  }

  void add_fifo_edge(std::unordered_set<ssfu*>& fifos, ssnode* n, dsa::dfg::Edge edge, Schedule& sched) {
    bool found_node = false;
    for (auto link : sched.links_of(&edge)) {
      if (found_node) {
        if (auto fu = dynamic_cast<ssfu*>(link.second->sink())) {
          if (fu->delay_fifo_depth() < 32 && fifos.find(fu) == fifos.end()) {
            fu->max_delay(fu->delay_fifo_depth() + 1);
            fifos.insert(fu);
          }
        }
      } else if (link.second->sink()->id() == n->id()) {
        found_node = true;
      }
    }
  }

  /*
  void add_vport(std::unordered_set<ssnode*> nodes_forwarded, ssnode* n, dsa::dfg::Edge edge, Schedule& sched) {
    for (auto link : sched.links_of(&edge)) {
    }
  }
  */



  // When we delete a hardware element, we need to:
  // 1. deschedule anything that was assigned to that element
  // 2. remove the concept of that element from the schedule (consistency)
  // 3. remove the element from the hardware description
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
std::string  dump_log(const double& time, const int& iteration, const double& temp, CodesignInstance* curr_ci, CodesignInstance* best_ci);
} // namespace dsa