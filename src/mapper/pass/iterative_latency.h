#pragma once

#include <map>
#include <tuple>
#include <vector>

#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/utils.h"

namespace dsa {
namespace dfg {
namespace pass {

inline void inject_noop_impl(SSDfg* dfg, int eid, SSDfgInst* inst) {
  /* If inst is null:
   * before: def -[eid]-> use
   * after:  def -[new edge]-> noop -[eid]-> use
   *
   * If inst is not null:
   * before: def -[eid]-> use
   *          |
   *          +---------> inst -> ...
   *
   * after:  def          use
   *          |            ^
   *          |            |
   *          |          [eid]
   *          |            |
   *          +---------> inst -> ...
   */
  CHECK(eid < dfg->edges.size());
  if (inst) {
    auto& e0 = dfg->edges[eid];
    // If we already have a noop, we need sanity check if it is a noop
    CHECK(inst->inst() == dsa::SS_NONE && inst->ops().size() == 1)
        << "The injected instruction should be a noop.";
    auto& e1 = dfg->edges[inst->ops()[0].edges[0]];
    // CHECK(e0.sid == e1.sid && e0.vid == e1.vid && e0.l == e1.l && e0.r == e1.r) <<
    //   e0.name() << " " << e1.name();
    CHECK(inst->values.size() == 1);
  } else {
    // Create a noop instruction.
    dfg->emplace_back<SSDfgInst>(dfg, SS_NONE);
    inst = &dfg->instructions.back();
    // Create a edge from source to noop
    dfg->edges.emplace_back(dfg, dfg->edges[eid].sid, dfg->edges[eid].vid, inst->id(),
                            dfg->edges[eid].l, dfg->edges[eid].r);
    std::vector<int> es{dfg->edges.back().id};
    inst->ops().emplace_back(dfg, es, OperandType::data);
  }
  // Add this edge to inst's use.
  inst->values[0].uses.push_back(eid);
  // In both cases, we need remove the edge usage in the source.
  bool found = false;
  for (auto& value : dfg->edges[eid].def()->values) {
    auto iter = std::find(value.uses.begin(), value.uses.end(), eid);
    if (iter != value.uses.end()) {
      value.uses.erase(iter);
      found = true;
    }
  }
  CHECK(found) << "The value used by this edge not found!";
  // Change the edge source to the noop.
  dfg->edges[eid].sid = inst->id();
  dfg->edges[eid].vid = 0;
}

inline void inject_passthrus(SSDfg* dfg, Schedule* sched, std::vector<int>& edge_length,
                             std::vector<std::pair<int, int>>& mapping,
                             std::vector<std::vector<int>>& edge_groups) {
  edge_length.resize(sched->edge_prop().size(), 0);
  edge_groups.resize(dfg->edges.size());
  mapping.resize(dfg->nodes.size(), {-1, -1});
  using PassThruKey = std::tuple<int, int, int, int>;
  std::map<PassThruKey, int> replace;
  for (int i = 0, n = sched->edge_prop().size(); i < n; ++i) {
    auto& ep = sched->edge_prop()[i];
    auto f = [sched, &mapping, &i](SSDfgNode* node) {
      auto loc = sched->location_of(node);
      if (loc.second) {
        CHECK(node->id() < mapping.size());
        mapping[node->id()] = {loc.first, loc.second->id()};
      }
    };
    f(dfg->edges[i].def());
    f(dfg->edges[i].use());
    int distance = 1;
    for (int j = 1, m = ep.links.size(); j < m; ++j) {
      ++distance;
      if (auto pass = dynamic_cast<ssfu*>(ep.links[j].second->orig())) {
        PassThruKey key{ep.links[j].first, pass->id(), sched->ssdfg()->edges[i].sid,
                        sched->ssdfg()->edges[i].vid};
        auto iter = replace.find(key);
        if (iter == replace.end()) {
          inject_noop_impl(dfg, i, nullptr);
          int iid = dfg->instructions.back().id();
          replace[key] = iid;
          if (iid >= mapping.size()) {
            mapping.resize(iid + 1);
            mapping[iid] = {ep.links[j].first, pass->id()};
          }
        } else {
          auto inst = dynamic_cast<SSDfgInst*>(dfg->nodes[iter->second]);
          CHECK(inst);
          inject_noop_impl(dfg, i, inst);
        }
        LOG(EDGES) << dfg->edges.back().name() << " " << dfg->edges.back().id;
        if (dfg->edges.back().id >= edge_length.size())
          edge_length.resize(dfg->edges.back().id + 1);
        edge_length[dfg->edges.back().id] = distance;
        edge_groups[i].push_back(dfg->edges.back().id);
        distance = 1;
      }
    }
    edge_length[i] = distance;
    edge_groups[i].push_back(i);
  }
}

inline void dfg_impl(SSDfgNode* node, std::vector<bool>& visited,
                     std::vector<SSDfgNode*>& order) {
  if (visited[node->id()]) {
    return;
  }

  visited[node->id()] = true;

  for (auto& value : node->values) {
    for (auto eid : value.uses) {
      auto edge = &node->ssdfg()->edges[eid];
      dfg_impl(edge->use(), visited, order);
    }
  }

  order.push_back(node);
}

/* \brief Return the reversed topological order of the dataflow graph */
inline std::vector<SSDfgNode*> reversed_topology(SSDfg* dfg) {
  struct Rooter : Visitor {
    Rooter(int n) : visited(n, false) { res.reserve(n); }
    std::vector<bool> visited;
    std::vector<SSDfgNode*> res;
    void Visit(SSDfgVecInput* input) override { dfg_impl(input, visited, res); }
  };
  Rooter rooter(dfg->nodes.size());
  dfg->Apply(&rooter);
  return rooter.res;
}

struct Bounds {
  int min{0};
  int max{INT_MAX - 10000};
  std::string ToString() {
    std::ostringstream oss;
    oss << "[" << min << ", " << max << "]";
    return oss.str();
  }
};

struct ResetBoundVisitor : Visitor {
  ResetBoundVisitor(std::vector<Bounds>& b) : bounds(b) {}
  void Visit(SSDfgInst* inst) {
    bounds[inst->id()].min = 0;
    bounds[inst->id()].max = INT_MAX - 10000;
  }
  void Visit(SSDfgVecOutput* output) {
    bounds[output->id()].min = 0;
    bounds[output->id()].max = INT_MAX - 10000;
  }
  void Visit(SSDfgVecInput* input) {
    bounds[input->id()].min = bounds[input->id()].max = 0;
  }
  std::vector<Bounds>& bounds;
};

const int min_expect = 2;
const int max_expect = 8;

void iterative_bounds(SSDfg* dfg, std::vector<SSDfgNode*>& non_temp,
                      std::vector<int>& edge_length,
                      std::vector<std::pair<int, int>>& mapping, SSModel* model,
                      std::vector<Bounds>& bounds) {
  bool changed = true;
  bool overflow = false;
  int iters = 0;
  int max_mis = 0;
  bounds.resize(dfg->nodes.size());
  ResetBoundVisitor rbv(bounds);
  dfg->Apply(&rbv);

  while (changed || overflow) {
    changed = false;

    iters++;
    LOG(LAT_PASS) << "=================== " << iters << " ===================";
    if (overflow) {
      overflow = false;
      dfg->Apply(&rbv);
      max_mis++;
    }

    // FORWARD PASS
    for (int i = non_temp.size() - 1; i >= 0; --i) {
      SSDfgNode* node = non_temp[i];
      auto& vp = bounds[node->id()];
      int new_min = bounds[node->id()].min;
      int new_max = bounds[node->id()].max;

      for (auto& op : node->ops()) {
        for (auto& eid : op.edges) {
          auto edge = &dfg->edges[eid];
          SSDfgNode* origNode = edge->def();
          auto& orig_vp = bounds[origNode->id()];

          int routing_latency = edge_length[eid];
          int edge_lat = origNode->lat_of_inst() + routing_latency - 1;

          int fu_idx = mapping[edge->uid].second;
          auto max_ed = fu_idx != -1
                            ? model->subModel()->node_list()[fu_idx]->delay_fifo_depth()
                            : 0;

          // cout << " -----------------" <<  edge->name() << ": " << edge_lat << "\n";

          // This edge is routed
          if (routing_latency != 0) {
            // LOG(LAT_PASS) << edge->name();
            // LOG(LAT_PASS) << orig_vp.min << ", " << orig_vp.max;
            // LOG(LAT_PASS) << new_min << ", " << new_max;
            // LOG(LAT_PASS) << edge_lat << " " << max_ed << " " << max_mis;
            new_min = std::max(new_min, orig_vp.min + edge_lat);
            new_max = std::min(new_max, orig_vp.max + edge_lat + max_ed + max_mis);
          } else {
            // This edge is not routed, so give worst case upper bound
            new_min = std::max(new_min, orig_vp.min + edge_lat + min_expect);
            new_max =
                std::min(new_max, orig_vp.max + edge_lat + max_ed + max_mis + max_expect);
          }
        }
      }
      changed |= new_min != vp.min;
      changed |= new_max != vp.max;
      vp.min = new_min;
      vp.max = new_max;

      if (new_min > new_max) {
        overflow = true;
        LOG(LAT_PASS) << "forward overflow!";
        break;
      }
    }

    if (overflow) continue;

    // BACKWARDS PASS
    for (int i = 0, n = non_temp.size(); i < n; ++i) {
      SSDfgNode* node = non_temp[i];
      auto& vp = bounds[node->id()];
      int new_min = vp.min;
      int new_max = vp.max;

      for (auto& value : node->values) {
        for (auto eid : value.uses) {
          auto edge = &dfg->edges[eid];
          SSDfgNode* useNode = edge->use();
          auto& use_vp = bounds[useNode->id()];

          int routing_latency = edge_length[eid];
          int edge_lat = routing_latency - 1 + node->lat_of_inst();

          int fu_idx = mapping[edge->uid].second;
          auto max_ed = fu_idx != -1
                            ? model->subModel()->node_list()[fu_idx]->delay_fifo_depth()
                            : 0;

          if (routing_latency != 0) {
            // LOG(LAT_PASS) << edge->name();
            // LOG(LAT_PASS) << use_vp.min << ", " << use_vp.max;
            // LOG(LAT_PASS) << new_min << ", " << new_max;
            // LOG(LAT_PASS) << edge_lat << " " << max_ed << " " << max_mis;
            new_min = std::max(new_min, use_vp.min - edge_lat - max_ed - max_mis);
            new_max = std::min(new_max, use_vp.max - edge_lat);
          } else {
            new_min =
                std::max(new_min, use_vp.min - edge_lat - max_ed - max_mis - max_expect);
            new_max = std::min(new_max, use_vp.max - edge_lat - min_expect);
          }
        }
      }
      changed |= new_min != vp.min;
      changed |= new_max != vp.max;
      vp.min = new_min;
      vp.max = new_max;

      if (new_min > new_max) {
        LOG(LAT_PASS) << "backward overflow!";
        overflow = true;
        break;
      }
    }
  }
}

void assign_latency(SSDfg* dfg, SSModel* model, std::vector<SSDfgNode*>& non_temp,
                    std::vector<int>& edge_length,
                    std::vector<std::pair<int, int>>& mapping,
                    std::vector<Bounds>& bounds, std::vector<int>& latency,
                    std::vector<int>& edge_violation, std::vector<int>& edge_delay) {
  latency.resize(dfg->nodes.size(), 0);
  edge_violation.resize(dfg->edges.size(), 0);
  edge_delay.resize(dfg->edges.size(), 0);
  for (int i = non_temp.size() - 1; i >= 0; --i) {
    auto* node = non_temp[i];
    auto& vp = bounds[node->id()];
    int target = vp.min;
    LOG(LAT_PASS) << "process: " << node->name();

    int max = 0;
    // int mis = 0;
    for (auto& op : node->ops()) {
      for (auto eid : op.edges) {
        auto edge = &dfg->edges[eid];
        SSDfgNode* origNode = edge->def();

        int routing_latency = edge_length[eid];
        // int max_edge_delay = _ssModel->maxEdgeDelay();
        int fu_idx = mapping[edge->uid].second;
        auto max_ed =
            fu_idx != -1 ? model->subModel()->node_list()[fu_idx]->delay_fifo_depth() : 0;

        if (routing_latency == 0) {  // if its not scheduled yet, be more liberal
          routing_latency = min_expect;
          max_ed += max_expect;
        }

        int lat = latency[origNode->id()] + routing_latency - 1;

        int diff = std::max(std::min(max_ed, target - lat), 0);
        // mis = std::max(mis,(target- lat) - diff);
        edge_delay[eid] = diff;

        int vio = std::max(0, (target - lat) - max_ed);
        edge_violation[eid] = vio;
        LOG(LAT_PASS) << edge->name();
        LOG(LAT_PASS) << "src lat: " << latency[origNode->id()] << ", "
                      << "edge lat: " << edge_length[eid] << ", "
                      << "bounds: " << bounds[node->id()].ToString();
        LOG(LAT_PASS) << "delay: " << diff << "  vio: " << vio;

        max = std::max(max, lat + diff);
      }
    }
    latency[node->id()] = node->lat_of_inst() + max;
  }
}

inline void calc_mis_vio(SSDfg* dfg, std::vector<SSDfgNode*>& non_temp,
                         std::vector<int>& edge_latency, std::vector<int>& edge_delay,
                         std::vector<int>& latency, int& max_lat, int& max_lat_mis,
                         int& total_vio, std::vector<int>& group_mismatch,
                         std::vector<int>& node_violation) {
  max_lat = max_lat_mis = total_vio = 0;
  group_mismatch.resize(dfg->num_groups());
  node_violation.resize(dfg->nodes.size());
  std::fill(group_mismatch.begin(), group_mismatch.end(), 0);
  std::fill(node_violation.begin(), node_violation.end(), 0);

  for (int i = non_temp.size() - 1; i >= 0; --i) {
    int low_lat = MAX_SCHED_LAT, up_lat = 0;
    auto node = non_temp[i];
    for (auto& op : node->ops()) {
      for (auto eid : op.edges) {
        auto edge = &dfg->edges[eid];
        SSDfgNode* origNode = edge->def();

        // If routing latency is 0, then its okay to assume minimum
        CHECK(eid >= 0 && eid < edge_latency.size()) << eid << " " << edge_latency.size();
        int routing_latency = edge_latency[eid];
        if (routing_latency == 0) {
          routing_latency = min_expect;
        }

        if (origNode != nullptr) {
          int edge_lat = edge_delay[eid] + routing_latency - 1;
          CHECK(edge_lat >= 0);
          int lat = latency[origNode->id()] + edge_lat;

          if (lat > up_lat) up_lat = lat;
          if (lat < low_lat) low_lat = lat;
        }
      }
    }
    int diff = up_lat - low_lat;  // - _ssModel->maxEdgeDelay();

    if (!node->is_temporal()) {
      if (diff > max_lat_mis) {
        max_lat_mis = diff;
      }
      if (diff > group_mismatch[node->group_id()]) {
        group_mismatch[node->group_id()] = diff;
      }
      total_vio += std::max(0, diff);
      node_violation[node->id()] = diff;
    }

    int new_lat = node->lat_of_inst() + up_lat;
    latency[node->id()] = new_lat;

    if (max_lat < new_lat) max_lat = new_lat;
  }
}

inline SSDfg* IterativeLatency(Schedule* sched, int& max_lat, int& max_lat_mis,
                               int& total_vio, std::vector<int>& group_mismatch,
                               bool is_export) {
  SSDfg dfg_(*sched->ssdfg());
  // Inject passthrough noops into the DFG.
  std::vector<std::vector<int>> edge_groups;
  std::vector<int> edge_length;
  std::vector<std::pair<int, int>> mapping;
  inject_passthrus(&dfg_, sched, edge_length, mapping, edge_groups);
  for (auto& edge : sched->ssdfg()->edges) {
    LOG(LAT_PASS) << edge.name();
    for (auto& lp : sched->edge_prop()[edge.id].links) {
      LOG(LAT_PASS) << lp.first << " " << lp.second->name();
    }
  }
  CHECK(edge_length.size() == dfg_.edges.size())
      << edge_length.size() << " " << dfg_.edges.size();
  // Sort the new DFG nodes in topological order.
  auto ordered = reversed_topology(&dfg_);
  std::vector<SSDfgNode*> non_temp;
  std::copy_if(ordered.begin(), ordered.end(), std::back_inserter(non_temp),
               [](SSDfgNode* node) { return !node->is_temporal(); });
  for (auto elem : non_temp) {
    LOG(LAT_PASS) << "topo: " << elem->name();
  }
  std::vector<Bounds> bounds;
  // Migrate legacy iterative bound here.
  iterative_bounds(&dfg_, non_temp, edge_length, mapping, sched->ssModel(), bounds);

  for (auto elem : non_temp) {
    LOG(LAT_PASS) << elem->name() << "[" << bounds[elem->id()].min << ", "
                  << bounds[elem->id()].max << "]";
  }

  // Migrate legacy latency pass here.
  std::vector<int> latency, edge_delay, edge_violation;
  assign_latency(&dfg_, sched->ssModel(), non_temp, edge_length, mapping, bounds, latency,
                 edge_violation, edge_delay);

  for (auto node : dfg_.nodes) {
    LOG(LAT_PASS) << node->name() << ": " << latency[node->id()];
  }

  // Migrate legacy violation calculation here.
  std::vector<int> node_vio;
  calc_mis_vio(&dfg_, non_temp, edge_length, edge_delay, latency, max_lat, max_lat_mis,
               total_vio, group_mismatch, node_vio);
  // Commit results to the schedule.
  for (auto elem : sched->ssdfg()->nodes) {
    int id = elem->id();
    sched->vex_prop()[id].min_lat = bounds[id].min;
    sched->vex_prop()[id].max_lat = bounds[id].min;
    sched->vex_prop()[id].lat = latency[id];
    sched->vex_prop()[id].vio = node_vio[id];
  }
  for (auto& elem : sched->ssdfg()->edges) {
    sched->edge_prop()[elem.id].extra_lat = sched->edge_prop()[elem.id].vio = 0;
    LOG(LAT_PASS) << "edge: " << elem.name();
    for (auto id : edge_groups[elem.id]) {
      LOG(LAT_PASS) << "member: " << dfg_.edges[id].name();
      sched->edge_prop()[elem.id].extra_lat += edge_delay[id];
      sched->edge_prop()[elem.id].vio += edge_violation[id];
    }
    LOG(LAT_PASS) << "vio: " << sched->vioOf(&elem);
  }
  LOG(LAT_PASS) << "total vio: " << total_vio << " "
                << "latency: " << max_lat << " "
                << "mis: " << max_lat_mis;
  if (is_export) {
    SSDfg* res = new SSDfg(dfg_);
    for (auto& elem : res->edges) {
      elem.delay = edge_delay[elem.id];
    }
    return res;
  }
  return nullptr;
}

}  // namespace pass
}  // namespace dfg
}  // namespace dsa