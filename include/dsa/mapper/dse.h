#pragma once

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
  std::vector<WorkloadSchedules> workload_array;
  std::vector<int> weight;

  std::vector<Schedule*> res;
  std::vector<bool> unused_nodes;
  std::vector<bool> unused_links;

  bool sanity_check{false};

  CodesignInstance(SSModel* model);

  // Check that everything is okay
  void verify() {
    if (!sanity_check) return;

    for_each_sched([&](Schedule& sched) {
      sched.validate();

      assert(&_ssModel == sched.ssModel());
      assert(_ssModel.subModel()->node_list().size() >= sched.node_prop().size());
      assert(_ssModel.subModel()->link_list().size() >= sched.link_prop().size());

      for (auto& ep : sched.edge_prop()) {
        for (auto& p : ep.links) {
          assert(p.second->id() < (int)_ssModel.subModel()->link_list().size());
          assert(p.second->id() < (int)sched.link_prop().size());
          assert(_ssModel.subModel()->link_list()[p.second->id()] == p.second);
        }
      }
    });
    for (auto& node : _ssModel.subModel()->node_list()) {
      if (auto fu = dynamic_cast<ssfu*>(node)) {
        assert(!fu->fu_type_.capability.empty());
      }
    }
    for (unsigned i = 0; i < _ssModel.subModel()->link_list().size(); ++i) {
      assert(_ssModel.subModel()->link_list()[i]->id() == (int)i);
    }
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
                      this->delete_hw(n);
                      res = true;
                    }
                  });
    return res;
  }

  // Delete FUs, Switches, and Vports that can't possible be useful
  bool delete_hangers() {
    bool deleted_something = false;

    auto* sub = _ssModel.subModel();

    for (ssfu* fu : sub->fu_list()) {
      if (fu->in_links().size() <= 1 || fu->out_links().size() < 1) {
        delete_hw(fu);
        deleted_something = true;
      }
    }
    for (ssvport* ivport : sub->input_list()) {
      if (ivport->out_links().size() < 1) {
        delete_hw(ivport);
        deleted_something = true;
      }
    }
    for (ssvport* ovport : sub->output_list()) {
      if (ovport->in_links().size() < 1) {
        delete_hw(ovport);
        deleted_something = true;
      }
    }

    return deleted_something;
  }

  void prune_all_unused() {
    utilization();
    auto* sub = _ssModel.subModel();

    for (int j = 0, n = sub->link_list().size(); j < n; ++j) {
      if (unused_links[j]) delete_link(sub->link_list()[j]);
    }
    for (int j = 0, n = sub->switch_list().size(); j < n; ++j) {
      if (unused_nodes[sub->switch_list()[j]->id()]) delete_hw(sub->switch_list()[j]);
    }
    for (int j = 0, n = sub->fu_list().size(); j < n; ++j) {
      if (unused_nodes[sub->fu_list()[j]->id()]) delete_hw(sub->fu_list()[j]);
    }
    for (int j = 0, n = sub->input_list().size(); j < n; ++j) {
      if (unused_nodes[sub->input_list()[j]->id()]) delete_hw(sub->input_list()[j]);
    }
    for (int j = 0, n = sub->output_list().size(); j < n; ++j) {
      if (unused_nodes[sub->output_list()[j]->id()]) delete_hw(sub->output_list()[j]);
    }
    finalize_delete();

    while (delete_hangers()) {
      finalize_delete();  // TODO part of finalize delete isredundant now
    }
  }

  void make_random_modification(double temperature) {
    switch (rand() % 3) {
      case 0: add_something(temperature); break;
      case 1: remove_something(temperature); break;
      case 2: change_parameters_of_nodes(temperature);
    }
  }

  void add_something(int cnt) {
    if (rand() % 100 <= cnt * cnt)
      _ssModel.io_ports += rand() % (4 - _ssModel.io_ports + 1);

    auto* sub = _ssModel.subModel();

    verify_strong();

    // Items to add
    for (int i = 0; i < cnt; ++i) {
      int item_class = rand() % 100;
      if (item_class < 65) {
        // Add a random link -- really? really
        if (sub->node_list().empty()) continue;
        int src_node_index = rand() % sub->node_list().size();
        int dst_node_index = rand() % sub->node_list().size();
        ssnode* src = sub->node_list()[src_node_index];
        ssnode* dst = sub->node_list()[dst_node_index];
        if (dynamic_cast<ssvport*>(src) && src->out_links().empty()) {
          continue;
        }
        if (dynamic_cast<ssvport*>(dst) && dst->in_links().empty()) {
          continue;
        }
        if (src == dst) continue;

        // sslink* link =
        sub->add_link(src, dst);
        // std::cout << "adding link: " << link->name() << "\n";
      } else if (item_class < 80) {
        // Add a random switch
        ssswitch* sw = sub->add_switch();
        add_random_edges_to_node(sw, 1, 5, 1, 5);

        // std::cout << "adding switch" << sw->name() << " ins/outs:"
        //          << sw->in_links().size() << "/" << sw->out_links().size() << "\n";
      } else if (item_class < 90) {
        // Randomly pick an FU type from the set
        auto& fu_defs = _ssModel.fu_types;
        if (fu_defs.empty()) continue;
        ssfu* fu = sub->add_fu();
        int fu_def_index = rand() % fu_defs.size();
        Capability* def = fu_defs[fu_def_index];
        fu->fu_type_ = *def;

        add_random_edges_to_node(fu, 1, 5, 1, 5);

      } else if (item_class < 95) {
        // Add a random input vport
        ssvport* vport = sub->add_vport(true);
        add_random_edges_to_node(vport, 0, 1, 5, 12);
        // std::cout << "adding input vport: " << vport->name() << "\n";
      } else {  // (item_class < 100)
                // Add a random output vport
        ssvport* vport = sub->add_vport(false);
        add_random_edges_to_node(vport, 5, 12, 0, 1);
        // std::cout << "adding output vport: " << vport->name() << "\n";
      }
    }

    verify_strong();

    for_each_sched([&](Schedule& sched) { sched.allocate_space(); });

    verify_strong();

    // TODO: add some entries in routing tables?

    // End this by making sure all the routing tables are generated
    verify_strong();
  }

  void remove_something(int cnt) {
    if (rand() % 100 <= cnt * cnt) {
      _ssModel.io_ports = rand() % _ssModel.io_ports + 1;
    }
    auto* sub = _ssModel.subModel();
    // Choose a set of Items to remove
    for (int i = 0; i < cnt; ++i) {
      int item_class = rand() % 100;
      if (item_class < 60) {
        // delete a link
        if (sub->link_list().empty()) continue;
        int index = non_uniform_random(sub->link_list(), unused_links);
        sslink* l = sub->link_list()[index];
        if (delete_linkp_list.count(l)) continue;  // don't double delete
        std::cout << "remove: " << l->name() << std::endl;
        delete_link(l);
      } else if (item_class < 75) {
        // delete a switch
        if (sub->switch_list().empty()) continue;
        int index = non_uniform_random(sub->switch_list(), unused_nodes);
        ssswitch* sw = sub->switch_list()[index];
        if (delete_nodep_list.count(sw)) continue;  // don't double delete
        delete_hw(sw);
      } else if (item_class < 90) {
        // delete an FU
        if (sub->fu_list().empty()) continue;
        int index = non_uniform_random(sub->fu_list(), unused_nodes);
        ssfu* fu = sub->fu_list()[index];
        if (delete_nodep_list.count(fu)) continue;  // don't double delete
        delete_hw(fu);
      } else if (item_class < 95) {
        // delete an VPort
        if (sub->input_list().size() == 0) continue;
        int index = non_uniform_random(sub->input_list(), unused_nodes);
        ssvport* vport = sub->input_list()[index];
        if (delete_nodep_list.count(vport)) continue;  // don't double delete
        delete_hw(vport);
      } else {
        // delete an VPort
        if (sub->output_list().size() == 0) continue;
        int index = non_uniform_random(sub->output_list(), unused_nodes);
        ssvport* vport = sub->output_list()[index];
        if (delete_nodep_list.count(vport)) continue;  // don't double delete
        delete_hw(vport);
      }
    }

    // TODO: delete some entries in routing tables?

    // Lets finalize the delete here so that the datastructre is consistent
    // again when we are adding things -- simpler
    finalize_delete();

    while (delete_hangers()) {
      finalize_delete();  // TODO part of finalize delete isredundant now
    }

    verify_strong();
  }

  void change_parameters_of_nodes(int cnt) {
    auto* sub = _ssModel.subModel();

    if (rand() % 100 <= cnt) {
      _ssModel.io_ports = rand() % 4 + 1;
    }

    // Modifiers
    for (int i = 0; i < cnt; ++i) {
      int item_class = rand() % 100;
      if (item_class < 15) {
        if (sub->node_list().empty()) continue;
        int node_index = rand() % sub->node_list().size();
        ssnode* node = sub->node_list()[node_index];
        if (dynamic_cast<ssvport*>(node)) continue;

        node->set_flow_control(!node->flow_control());
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

      } else if (item_class < 30) {
        // TODO: Skip it for now
        --i;
        continue;

        if (sub->fu_list().empty()) continue;
        // Modify FU utilization
        int diff = rand() % 16 - 8;
        if (diff == 0) continue;
        int fu_index = rand() % sub->fu_list().size();
        ssfu* fu = sub->fu_list()[fu_index];
        int old_util = fu->max_util();
        int new_util = std::max(1, old_util + diff);
        if (diff < -4) new_util = 1;

        if (old_util == 1 && new_util > 1) {
          fu->set_flow_control(true);
        }

        fu->set_max_util(new_util);

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

      } else if (item_class < 60) {
        if (sub->fu_list().empty()) continue;
        // Modify FU delay-fifo depth
        int diff = -(rand() % 3 + 1);
        int fu_index = rand() % sub->fu_list().size();
        ssfu* fu = sub->fu_list()[fu_index];
        int new_delay_fifo_depth = std::max(1, fu->delay_fifo_depth() + diff);
        fu->set_delay_fifo_depth(new_delay_fifo_depth);

        // if we are constraining the problem, then lets re-assign anything
        // mapped to this FU
        for_each_sched([&](Schedule& sched) {
          for (int slot = 0; slot < sched.num_slots(fu); ++slot) {
            for (auto& p : sched.dfg_nodes_of(slot, fu)) {
              for (auto& op : p.first->ops()) {
                for (auto eid : op.edges) {
                  auto* elem = &sched.ssdfg()->edges[eid];
                  if (sched.edge_delay(elem) > fu->delay_fifo_depth()) {
                    std::cout << sched.edge_delay(elem) << " > " << fu->delay_fifo_depth()
                              << ", unassign " << p.first->name() << std::endl;
                    sched.unassign_dfgnode(p.first);
                  }
                }
              }
            }
          }
        });

      } else if (item_class < 80) {
        // change fu-type
        int index = rand() % sub->fu_list().size();
        auto fu = sub->fu_list()[index];

        if (rand() & 1) {
          fu->fu_type_ = *ss_model()->fu_types[rand() % ss_model()->fu_types.size()];
        } else if (fu->fu_type_.capability.size() > 1) {
          int j = rand() % fu->fu_type_.capability.size();
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
      } else if (item_class < 100) {
        // change decomposer
        int index = rand() % sub->node_list().size();
        auto fu = sub->node_list()[index];
        static const int candidates[] = {1, 2, 4, 8};
        int new_one = candidates[rand() % 4];
        while (new_one == fu->decomposer) {
          new_one = candidates[rand() % 4];
        }
        if (new_one < fu->decomposer) {
          for_each_sched([&](Schedule& sched) {
            for (int slot = 0; slot < sched.num_slots(fu); ++slot) {
              for (auto& p : sched.dfg_nodes_of(slot, fu)) {
                if (p.first->bitwidth() < new_one * 8) sched.unassign_dfgnode(p.first);
              }
            }
          });
        }
        std::cout << "decomposer changed from " << fu->decomposer << " to " << new_one
                  << std::endl;
        fu->decomposer = new_one;
        assert(fu->decomposer);
        fu->granularity = fu->mf_decomposer = fu->bitwidth() / fu->decomposer;
      }
    }
  }

  void for_each_sched(const std::function<void(Schedule&)>& f) {
    for (auto& ws : workload_array) {
      for (Schedule& sched : ws.sched_array) {
        f(sched);
      }
    }
  }

  // Overide copy constructor to enable deep copies
  CodesignInstance(const CodesignInstance& c, bool from_scratch) : _ssModel(c._ssModel) {
    // SSModel* copy_model = (SSModel*)&c._ssModel;
    // auto* copy_sub = copy_model->subModel();
    // std::cout << "copy from:" << copy_sub << " copy to:" << _ssModel.subModel() <<
    // "\n"; assert(_ssModel.subModel() != copy_sub);

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
  }

  // Delete link on every schedule
  void delete_link(sslink* link) {
    delete_link_list.push_back(link->id());
    delete_linkp_list.insert(link);

    // remove it from every schedule
    for_each_sched([&](Schedule& sched) {
      for (int slot = 0; slot < sched.num_slots(link); ++slot) {
        for (auto& p : sched.dfg_edges_of(slot, link)) {
          // TODO: consider just deleteting the edge, and having the scheduler
          // try to repair the edge schedule -- this might save some time
          // sched.unassign_edge(p.first);
          sched.unassign_dfgnode(p.first->def());
          sched.unassign_dfgnode(p.first->use());
        }
      }
    });
  }

  // Delete fu on every schedule
  void delete_hw(ssfu* fu) {
    delete_fu_list.push_back(fu->id());
    delete_node(fu);
  }

  // Delete switch on every schedule
  void delete_hw(ssswitch* sw) {
    // std::cout << "Deleting Switch:" << sw->name() << "    ptr:" << sw << "\n";
    delete_sw_list.push_back(sw->id());
    delete_node(sw);
  }

  // Delete vector port on every schedule
  void delete_hw(ssvport* vport) {
    delete_vport_list.push_back(vport->id());
    delete_node(vport);
  }

  // This makes the delete consistent across model and schedules
  void finalize_delete() {
    // Grab a copy copy of all nodes
    auto* sub = _ssModel.subModel();
    std::vector<ssnode*> n_copy = sub->node_list();  // I hope this copies the list?
    std::vector<sslink*> l_copy = sub->link_list();  // I hope this copies the list?

    verify();

    // Remove the elements from these lists
    sub->delete_nodes(delete_node_list);  // these happen after above, b/c above uses id
    sub->delete_links(delete_link_list);

    // got to reorder all the links
    for_each_sched([&](Schedule& sched) { sched.reorder_node_link(n_copy, l_copy); });

    verify();

    // finally, we just deleted a bunch of nodes/links, and we should
    // probably free the memory somehow?
    // that's why we tracked these datastructures
    for (auto* link : delete_linkp_list) {
      // we also need to tell the model to delete the link from its little lists
      // The sslink destructor unlinks it from the connected nodes' references
      delete link;
    }
    for (auto* node : delete_nodep_list) {
      delete node;
    }

    verify();

    // finally finally, clear all datastructres used for deleting
    delete_nodep_list.clear();
    delete_linkp_list.clear();

    delete_node_list.clear();
    delete_link_list.clear();
    delete_fu_list.clear();
    delete_sw_list.clear();
    delete_vport_list.clear();

    verify();
  }

  std::pair<double, int> dse_sched_obj(Schedule* sched) {
    if (!sched) return {0.1, INT_MIN};
    // YES, I KNOW THIS IS A COPY OF SCHED< JUST A TEST FOR NOW
    SchedStats s;
    int num_left = sched->num_left();
    bool succeed_sched = (num_left == 0);

    sched->get_overprov(s.ovr, s.agg_ovr, s.max_util);
    sched->fixLatency(s.lat, s.latmis);

    int violation = sched->violation();

    int obj = s.agg_ovr * 1000 + violation * 200 + s.latmis * 200 + s.lat +
              (s.max_util - 1) * 3000 + sched->total_passthrough;
    obj = obj * 100 + sched->num_links_mapped();

    int max_delay = sched->ssModel()->subModel()->fu_list()[0]->delay_fifo_depth();
    double performance = sched->estimated_performance();
    if (succeed_sched) {
      double eval = performance * ((double)max_delay / (max_delay + s.latmis));
      eval /= std::max(1.0 + s.ovr, sqrt(s.agg_ovr));
      return {eval, -obj};
    }

    return {-num_left, -obj};
  }

  float dse_obj() {
    std::pair<double, int> total_score = std::make_pair((double)1.0, 0);
    res.resize(workload_array.size());

    bool meaningful = true;
    double failure = 0;
    for (auto& ws : workload_array) {
      res[&ws - &workload_array[0]] = nullptr;
      std::pair<double, int> score = std::make_pair((double)1e-3, INT_MIN);
      bool yes = false;
      for (Schedule& sched : ws.sched_array) {
        std::pair<double, int> new_score = dse_sched_obj(&sched);
        if (new_score > score) {
          score = new_score;
          res[&ws - &workload_array[0]] = &sched;
          yes = true;
        }
        if (new_score.first < 0.0) {
          failure = std::min(failure, new_score.first);
        }
      }

      if (!yes) meaningful = false;
      total_score.first *= score.first * weight[&ws - &workload_array[0]];
    }

    if (!meaningful) {
      return exp(failure);
    }

    total_score.first = pow(total_score.first, (1.0 / workload_array.size()));

    auto estimated = dsa::adg::estimation::EstimatePowerAera(&_ssModel);

    float area = (estimated.Total<dsa::adg::estimation::Metric::Area>());
    // float obj = total_score.first * 1e6 / area;
    float obj = total_score.first * total_score.first * 1e6 / area;

    return obj;
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
            unused_nodes[res[i]->ssModel()->subModel()->link_list()[j]->orig()->id()] =
                false;
            unused_nodes[res[i]->ssModel()->subModel()->link_list()[j]->dest()->id()] =
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
        std::cout << res[i]->ssdfg()->filename << ": " << performance.first << std::endl;
      }
    }
  }

 private:
  // When we delete a hardware element, we need to:
  // 1. deschedule anything that was assigned to that element
  // 2. remove the concept of that element from the schedule (consistency)
  // 3. remove the element from the hardware description
  void delete_node(ssnode* n) {
    delete_node_list.push_back(n->id());
    delete_nodep_list.insert(n);

    for_each_sched([&](Schedule& sched) {
      for (int slot = 0; slot < sched.num_slots(n); ++slot) {
        for (auto& p : sched.dfg_nodes_of(slot, n)) {
          sched.unassign_dfgnode(p.first);
        }
      }
    });

    for (auto& l : n->in_links()) {
      delete_link(l);
    }
    for (auto& l : n->out_links()) {
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
