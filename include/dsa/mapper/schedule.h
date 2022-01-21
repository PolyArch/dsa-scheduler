#pragma once

#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <map>
#include <unordered_set>

#include "dsa/arch/model.h"
#include "dsa/arch/sub_model.h"
#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/visitor.h"
#include "dsa/mapper/config_defs.h"

using namespace dsa;

#define MAX_SCHED_LAT 1000000

namespace dsa {
namespace mapper {

/*!
 * \brief The range silcing struct. Left close, right close.
 *        Use const for left and right because I do not expect users change it.
 */
struct Range {
  /*!
   * \brief The left point of slicing.
   */
  const int l;
  /*!
   * \brief The right point of slicing.
   */
  const int r;

  Range(int l_, int r_) : l(l_), r(r_) {}

  bool operator==(const Range&);
};

/*!
 * \brief The struct of decomposable mapping.
 */
template <typename T>
struct Slot {
  /*!
   * \brief The number of lane.
   */
  int lane_no;
  /*!
   * \brief The component to be decomposed.
   */
  T ref;

  Slot(int l, T r) : lane_no(l), ref(r) {}
};

/*!
 * \brief The struct of slicing DFG components.
 */
template<typename T>
struct DFGSlice {
  /*!
   * \brief The range of slicing.
   */
  int l, r;
  /*!
   * \brief The DFG node to be sliced.
   */
  T ref;
};

/*!
 * \brief The spot of candidate mapping.
 */
struct MapSpot {
  /*!
   * \brief If we want to routing along with this lane.
   */
  bool routing_along;
  /*!
   * \brief The candidate slot.
   */
  Slot<ssnode*> slot;

  MapSpot(bool r, const Slot<ssnode*> &s) :
    routing_along(r), slot(s.lane_no, s.ref) {}
};

/*!
 * \brif The properties of DFG node mapping.
 */
struct VertexProp {
  /*!
   * \brief The bound of data timing.
   */
  int min_lat{INT_MAX};
  int max_lat{INT_MIN};
  /*!
   * \brief The latency assigned to this node.
   */
  int lat{0};
  /*!
   * \brief The timing violation of this node.
   */
  int vio{0};
  /*!
   * \brief The ADG decomposable slot mapped to.
   */
  mapper::Slot<ssnode*> slot{-1, nullptr};
  /*!
   * \brief The helper of accessing the ADG node.
   */
  ssnode* node() const { return slot.ref; }
  /*!
   * \brief The helper of accessing the lane in the ADG node.
   */
  int lane() const { return slot.lane_no; }
};

// struct NodeProp {
//   /*! \brief To support the decomposability, we break a whole ssnode into slots. */
//   struct NodeSlot {
//     /*! \brief Edges pass through this node slot to route. */
//     std::vector<dsa::dfg::Edge*> passthrus;
//     /*! \brief Byte slot of the dfg node (inst/vec) that is mapped to this node slot. */
//     std::vector<std::pair<dsa::dfg::Node*, int>> vertices;
//   };
//   /*! \brief The mapping information of each lane hardware lane. */
//   std::vector<NodeSlot> slots;
// };

} // namespace mapper
} // namespace dsa

struct EdgeSlice {
  /*! \brief The identifier of the edge to be sliced. */
  int eid;

  // TODO(@were): Do I want to refactor this to left close right open?
  /*! \brief The **absolute** slicing of the edge. By **absolute**,
             we mean if a edge is a.1[16:31], and we want to extract
             [16:23], the value of l,and r should be 16,23 respectively,
             instead of implicitly adding the 16 offset. */
  int l, r;

  EdgeSlice(int eid, int l, int r) : eid(eid), l(l), r(r) {}

  bool operator==(const EdgeSlice& b) { return eid == b.eid && l == b.l && r == b.r; }
};

class Schedule {
 public:
  /*!
   * \brief Constructor for the hardware/software pair
   * \param model The hardware.
   * \param dfg The software DFG.
   */
  Schedule(SSModel* model, SSDfg* dfg);

  /*! \brief Deep copy a schedule. */
  Schedule(const Schedule& c, bool dup_dfg);

  Schedule(const Schedule& c, SSModel* model);

  //~Schedule() {delete _ssDFG;}

  constexpr static const float gvsf = 4.0f;

  // TODO(@were): Will it be better to move all these tools to
  //              a separate python script.
  // @{
  void printGraphviz(const char* name);

  void printNodeGraphviz(std::ofstream& ofs, ssnode* fu);

  void printMvnGraphviz(std::ofstream& ofs, ssnode* node);

  void printMelGraphviz(std::ofstream& ofs, ssnode* node);

  void printSwitchGraphviz(std::ofstream& ofs, ssswitch* sw);

  void DumpMappingInJson(const std::string& mapping_filename);

  void LoadMappingInJson(const std::string& mapping_filename);

  void printConfigHeader(std::ostream& os, std::string cfg_name, bool cheat = true);

  void printConfigCheat(std::ostream& os, std::string cfg_name);

  void printConfigVerif(std::ostream& os);

  void printCondensedVector(std::vector<int>& vec, std::ostream& os);
  // @}

  /*!
   * \brief The software to be mapped.
   */
  SSDfg* ssdfg() const { return _ssDFG; }

  /*!
   * \brief Assign latency to the given node.
   * \param node The node to be assigned.
   * \param lat The latency to assign.
   */
  void assign_lat(dsa::dfg::Node* node, int lat) {
    _vertexProp[node->id()].lat = lat;
  }

  /*!
   * \brief Return the latency of the given node in DFG.
   * \param n The DFG node to query.
   */
  int latOf(dsa::dfg::Node* n) { return _vertexProp[n->id()].lat; }

  /*!
   * \brief Assign the bound of DFG latency for the purpose of timing.
   * \param n The DFG node to update.
   * \param min The lower bound.
   * \param max The upper bound.
   */
  void assign_lat_bounds(dsa::dfg::Node* n, int min, int max) {
    auto& vertex_prop = _vertexProp[n->id()];
    vertex_prop.min_lat = min;
    vertex_prop.max_lat = max;
  }

  /*!
   * \brief Return the latency bound of the given DFG node.
   * \param n The node to query.
   */
  std::pair<int, int> lat_bounds(dsa::dfg::Node* n) {
    auto& vertex_prop = _vertexProp[n->id()];
    return std::make_pair(vertex_prop.min_lat, vertex_prop.max_lat);
  }

  bool isPassthrough(int slot, ssnode* node) {
    return !_nodeProp[node->id()].slots[slot].passthrus.empty();
  }

  void assign_edge_pt(dsa::dfg::Edge* edge, std::pair<int, ssnode*> pt) {
    if ((int)_edgeProp.size() <= edge->id) {
      _edgeProp.resize(edge->id + 1);
    }
    _nodeProp[pt.second->id()].slots[pt.first].passthrus.push_back(edge);
    ++total_passthrough;
    _edgeProp[edge->id].passthroughs.push_back(pt);
  }

  int groupMismatch(int g) { return _groupMismatch[g]; }

  int violation() { return _totalViolation; }

  void add_violation(int violation) {
    _totalViolation += violation;
    _max_lat_mis = std::max(_max_lat_mis, violation);
  }

  int vioOf(dsa::dfg::Node* n) { return _vertexProp[n->id()].vio; }

  void record_violation(dsa::dfg::Node* n, int violation) {
    _vertexProp[n->id()].vio = violation;
  }

  int vioOf(dsa::dfg::Edge* e) { return _edgeProp[e->id].vio; }

  void record_violation(dsa::dfg::Edge* e, int violation) {
    _edgeProp[e->id].vio = violation;
  }

  void check_all_links_consistency() {
    struct Checker : dfg::Visitor {
      Checker(Schedule* self) : self(self) {}
      void Visit(dsa::dfg::Node* node) {
        for (auto& operand : node->ops()) {
          for (auto eid : operand.edges) {
            self->check_links_consistency(&self->ssdfg()->edges[eid]);
          }
        }
      }
      Schedule* self;
    };
    Checker checker(this);
    ssdfg()->Apply(&checker);
  }

  void check_links_consistency(dsa::dfg::Edge* edge) {
    // TODO(@were): fix this.
    return;
    // sslink* prev_link = nullptr;
    // bool fail = false;
    // int prev_num_edges = 10000;

    // for (auto link : links_of(edge)) {
    //   if (prev_link != NULL && prev_link->sink() != link.second->source()) {
    //     std::cout << edge->name() << " has Failed Link Order Check\n";
    //     fail = true;
    //   }

    //   std::unordered_set<dsa::dfg::Edge*> edges;
    //   for (auto& i : edge_list(link.first, link.second)) {
    //     dsa::dfg::Edge* alt_edge = i.first;
    //     // Only relevant edges are 1. same value, 2. same slice, 3. same slot
    //     if (alt_edge->val() == edge->val() && alt_edge->l == edge->l &&
    //         link.first == i.second) {
    //       edges.insert(edge);
    //     }
    //   }
    //   int new_num_edges = edges.size();

    //   if (new_num_edges > prev_num_edges) {
    //     std::cout << "\n" << edge->name() << " has Failed Edge Consistency Check\n";
    //     fail = true;  // BAD!
    //     break;
    //   }
    //   prev_num_edges = new_num_edges;
    //   prev_link = link.second;
    // }
    // if (fail) {
    //   for (auto link : links_of(edge)) {
    //     std::cout << link.second->name() << " #edges";

    //     std::unordered_set<dsa::dfg::Edge*> edges;
    //     for (auto& i : edge_list(link.first, link.second)) {
    //       dsa::dfg::Edge* alt_edge = &ssdfg()->edges[i.eid];
    //       if (alt_edge->val() == edge->val() &&
    //           alt_edge->l == edge->l &&
    //           link.first == i.second) {
    //         edges.insert(edge);
    //       }
    //     }
    //     int new_num_edges = edges.size();

    //     std::cout << new_num_edges << " -- edges: ";
    //     for (auto i : edges) {
    //       std::cout << i->name() << " ";
    //     }
    //     std::cout << "\n";
    //   }
    //   printGraphviz("viz/sched-fail-incon-links.gv");
    //   CHECK(0) << "inconsistent links";
    // }
  }

  // this should be same as both input and output ports should give same locationOf
  // locationOf is the same ssnode assigned...
  int vecPortOf(dsa::dfg::VectorPort* vec) {
    auto mi = locationOf(vec);
    if (auto vport = dynamic_cast<ssvport*>(mi.node())) {
      DSA_CHECK(vport) << vec->name() << " " << mi.node()->name();
      return vport->port();
    } else {
      return -1;
    }
  }


  /*!
   * \brief Assign the given DFG node to the hardware resource.
   * \param dfgnode The DFG node.
   * \param assigned The hardware resource.
   */
  void assign_node(dsa::dfg::Node* dfgnode, std::pair<int, ssnode*> assigned) {
    int vid = dfgnode->id();
    DSA_CHECK(vid >= 0 && vid < _vertexProp.size());

    int orig_slot = assigned.first;
    auto snode = assigned.second;

    DSA_CHECK(_vertexProp[vid].node() == nullptr || _vertexProp[vid].node() == snode);
    if (_vertexProp[vid].slot.ref == snode) return;

    _vertexProp[vid].slot = {orig_slot, snode};

    DSA_CHECK(dfgnode->type() < dsa::dfg::Node::V_NUM_TYPES);
    _num_mapped[dfgnode->type()]++;

    DSA_CHECK(dfgnode);
    DSA_CHECK(snode->id() < (int)_nodeProp.size()) << snode->id() << "<" << (int)_nodeProp.size();

    // here dfg node is assigned to the hardware. Is it the second one? What is the slot?
    int num_slots = dfgnode->bitwidth() / snode->granularity();
    for (int i = 0; i < num_slots; ++i) {
      int slot = orig_slot + i;
      _nodeProp[snode->id()].slots[slot].vertices.emplace_back(dfgnode, orig_slot);
    }
  }

  void unassign_edge(dsa::dfg::Edge* edge) {
    auto& ep = _edgeProp[edge->id];

    _edge_links_mapped -= ep.links.size();

    // Remove all the edges for each of links
    for (auto& link : ep.links) {
      auto& lp = _linkProp[link.second->id()];

      int granularity = link.second->source()->granularity();
      int last_slot = edge->bitwidth() / granularity;
      for (int i = 0; i < last_slot; ++i) {
        int slot_index = (link.first + i) % link.second->sink()->lanes();
        DSA_CHECK(slot_index >= 0 && slot_index < (int) lp.slots.size());
        auto& slot = lp.slots[slot_index];

        auto& edges = slot.edges;
        EdgeSlice es(edge->id, edge->l + i * granularity, edge->l + (i + 1) * granularity - 1);
        auto it = std::find(edges.begin(), edges.end(), es);
        DSA_LOG(ASSIGN)
          << edges.size() << " mapped on " << link.first << "(" << slot_index << ")" << ","
          << link.second->name() << " [lp " << &lp << "], [lp.slot " << &lp.slots
          << "], [lp.slot[idx].edges " << &slot.edges << "]";
        DSA_CHECK(it != edges.end())
          << ssdfg()->edges[es.eid].name() << "[" << es.l << "," << es.r << "] not found on link "
          << link.second->name() << ", " << link.first;
        DSA_LOG(ASSIGN)
          << "unassign " << edge->name() << " [" << es.l << ", " << es.r << "] "
          << slot_index;
        edges.erase(it);
        if (slot.edges.empty()) {
          _links_mapped--;
          DSA_CHECK(_links_mapped >= 0);
        }
      }
    }

    // Remove all passthroughs associated with this edge
    for (auto& pt : ep.passthroughs) {
      auto& np = _nodeProp[pt.second->id()];
      auto& passthrus = np.slots[pt.first].passthrus;
      bool erased = false;
      for (auto iter = passthrus.begin(), end = passthrus.end(); iter != end; ++iter) {
        if (*iter == edge) {
          passthrus.erase(iter);
          --total_passthrough;
          erased = true;
          break;
        }
      }
      DSA_CHECK(erased);
    }

    _edgeProp[edge->id].reset();
  }

  // Delete all scheduling data associated with dfgnode, including its
  // mapped locations, and mapping information and metadata for edges
  void unassign_dfgnode(dsa::dfg::Node* dfgnode) {
    for (auto& op : dfgnode->ops()) {
      for (auto eid : op.edges) {
        auto* edge = &ssdfg()->edges[eid];
        unassign_edge(edge);
      }
    }
    for (auto& value : dfgnode->values) {
      for (auto eid : value.uses) {
        auto* edge = &ssdfg()->edges[eid];
        unassign_edge(edge);
      }
    }

    auto& vp = _vertexProp[dfgnode->id()];
    ssnode* node = vp.slot.ref;

    if (node) {
      int orig_slot = vp.slot.lane_no;
      _num_mapped[dfgnode->type()]--;
      int num_slots = dfgnode->bitwidth() / node->granularity();
      for (int i = 0; i < num_slots; ++i) {
        int slot = orig_slot + i;
        auto& vertices = _nodeProp[node->id()].slots[slot].vertices;
        auto it = std::find(vertices.begin(), vertices.end(), std::make_pair(dfgnode, orig_slot));
        DSA_CHECK(it != vertices.end());
        vertices.erase(it);
      }
      vp.slot = {-1, nullptr};
    }
  }

  std::vector<EdgeSlice>& edge_list(int slot, sslink* link) {
    return _linkProp[link->id()].slots[slot].edges;
  }

  void print_all_mapped() {
    std::cout << "Vertices: ";
    // TODO: can't get vertex/edge from id, need to modify dfg to maintain
    for (unsigned i = 0; i < _vertexProp.size(); ++i) {
      auto& v = _vertexProp[i];
      if (v.slot.ref) {
        std::cout << i << "->" << v.slot.ref->name() << " ";
      }
    }
    std::cout << "\nEdges: ";
    for (unsigned i = 0; i < _edgeProp.size(); ++i) {
      auto& e = _edgeProp[i];
      if (e.links.size()) {
        std::cout << i << " ";
      }
    }
    std::cout << "\n";
  }

  void assign_link_to_edge(dsa::dfg::Edge* dfgedge, int slot_index, sslink* slink) {
    DSA_CHECK(slink);
    DSA_CHECK(dfgedge);

    _edge_links_mapped++;
    auto& lp = _linkProp[slink->id()];

    int granularity = slink->source()->granularity();
    int lanes = slink->sink()->lanes();

    for (int i = 0; i < dfgedge->bitwidth() / granularity; ++i) {
      int cur_slot_index = (slot_index + i) % lanes;
      auto& slot = lp.slots[cur_slot_index];
      if (slot.edges.empty()) _links_mapped++;
      int l = dfgedge->l + granularity * i;
      int r = dfgedge->l + granularity * (i + 1) - 1;
      slot.edges.emplace_back(dfgedge->id, l, r);
      DSA_LOG(ASSIGN)
        << "assign " << dfgedge->name() << " [" << l << ", " << r << "]"
        << " to " << cur_slot_index << "," << slink->name();
    }

    if ((int)_edgeProp.size() <= dfgedge->id) {
      _edgeProp.resize(dfgedge->id + 1);
    }
  }

  // pdg edge to sslink
  void assign_edgelink(dsa::dfg::Edge* dfgedge, int slot, sslink* slink,
                       std::vector<std::pair<int, sslink*>>::iterator it) {
    assign_link_to_edge(dfgedge, slot, slink);
    int idx = it - _edgeProp[dfgedge->id].links.begin();
    DSA_CHECK(idx >= 0 && idx <= (int)_edgeProp[dfgedge->id].links.size()) << idx;
    // TODO: Performance
    _edgeProp[dfgedge->id].links.insert(it, std::make_pair(slot, slink));
  }

  // pdg edge to sslink
  void assign_edgelink(dsa::dfg::Edge* dfgedge, int slot, sslink* slink) {
    assign_link_to_edge(dfgedge, slot, slink);
    _edgeProp[dfgedge->id].links.push_back(std::make_pair(slot, slink));
  }

  // void print_links(dsa::dfg::Edge* dfgedge) {
  //  for(auto& i : _assignLinkEdge[dfgedge]) {
  //    cout << i->name() << " ";
  //  }
  //  cout << "\n";
  //}

  int link_count(dsa::dfg::Edge* dfgedge) { return _edgeProp[dfgedge->id].links.size(); }

  int edge_latency(dsa::dfg::Edge* pdgedge) {
    return _edgeProp[pdgedge->id].links.size();
  }

  // Calculate the total possible delay through delay fifos
  int max_edge_delay(dsa::dfg::Edge* pdgedge) {
    auto& ep = _edgeProp[pdgedge->id];
    int total_delay = 0;
    for (auto& link : ep.links) {
      total_delay += link.second->sink()->delay_fifo_depth();
    }
    return total_delay;
  }

  std::vector<std::pair<int, sslink*>>& links_of(dsa::dfg::Edge* edge) {
    auto& ep = _edgeProp[edge->id];
    return ep.links;
  }

  std::vector<std::pair<int, ssnode*>>& thrus_of(dsa::dfg::Edge* edge) {
    auto& ep = _edgeProp[edge->id];
    return ep.passthroughs;
  }

  void setLatOfLink(std::pair<int, sslink*> link, int l) {
    _linkProp[link.second->id()].slots[link.first].lat = l;
  }

  int latOfLink(std::pair<int, sslink*> link) {
    return _linkProp[link.second->id()].slots[link.first].lat;
  }

  bool linkAssigned(int slot, sslink* link) {
    return !_linkProp[link->id()].slots[slot].edges.empty();
  }

  // Return an alternate link for an edge
  dsa::dfg::Edge* alt_edge_for_link(std::pair<int, sslink*> link, dsa::dfg::Edge* e) {
    DSA_CHECK(link.second != nullptr) << "Alternate Edge for Link is null";
    auto& slots = _linkProp[link.second->id()].slots;
    int granularity = link.second->source()->granularity();
    for (auto it : slots[link.first].edges) {
      dsa::dfg::Edge* alt_e = &ssdfg()->edges[it.eid];
      // Only relevant edges are 1. same value, 2. same slice, 3. same slot
      if (alt_e->val() == e->val() && link.first * granularity == it.l &&
          (link.first + 1) * granularity - 1 == it.r) {
        return alt_e;
      }
    }
    return nullptr;
  }

  // return cost_to_route
  // 0: free
  // 1: empty
  //>2: already there
  int routing_cost(std::pair<int, sslink*> link, dsa::dfg::Edge* edge) {
    DSA_CHECK(link.second);

    if (needs_dynamic[edge->uid] && !link.second->flow_control()) {
      return -1;
    }

    auto& slots = _linkProp[link.second->id()].slots;
    // Check all slots will be occupied empty.
    bool num_edges = 0;
    int last_slot = link.first + edge->bitwidth() / link.second->source()->granularity();
    for (int s = link.first; s < last_slot; ++s) {
      int slot = s;
      num_edges = slots[slot].edges.size();
      if (num_edges != 0) break;
    }
    if (num_edges == 0) return 1;
    if (alt_edge_for_link(link, edge)) return 0;
    return num_edges + 1;
  }

  // Routing cost for inputs, but based on nodes instead of values
  int routing_cost_temporal_in(sslink* link, dsa::dfg::InputPort* in_v) {
    DSA_CHECK(link);
    auto& vec = _linkProp[link->id()].slots[0].edges;
    if (vec.empty()) return 1;
    for (auto elem : vec) {
      dsa::dfg::Edge* edge = &ssdfg()->edges[elem.eid];
      if (edge->def() == in_v) return 0;
    }
    return 2;
  }

  // Routing cost for outputs, but based on nodes instead of values
  int routing_cost_temporal_out(std::pair<int, sslink*> link, dsa::dfg::Node* node,
                                dsa::dfg::OutputPort* out_v) {
    DSA_CHECK(link.second);
    auto& vec = _linkProp[link.second->id()].slots[link.first].edges;
    if (vec.empty()) return 1;
    // It's free if the node is the same, or one of the use vectors is the same.
    for (auto elem : vec) {
      dsa::dfg::Edge* edge = &ssdfg()->edges[elem.eid];
      if (edge->def() == node) return 0;
      if (edge->use() == out_v) return 0;
    }
    return 2;
  }

  // find first node for
  dsa::dfg::Node* dfgNodeOf(int slot, sslink* link) {
    DSA_CHECK(link);
    auto& vec = _linkProp[link->id()].slots[slot].edges;
    return vec.empty() ? nullptr : ssdfg()->edges[vec[0].eid].def();
  }

  // find first node for
  dsa::dfg::Node* dfgNodeOf(sslink* link) { return dfgNodeOf(0, link); }
  
  // find first node for
  dsa::dfg::Node* dfgNodeOf(int slot, ssnode* node) {
    auto& vec = _nodeProp[node->id()].slots[slot].vertices;
    if (!vec.empty()) {
      return vec.front().first;
    }
    return nullptr;
  }
  // find first node for
  dsa::dfg::Node* dfgNodeOf(ssnode* node) { return dfgNodeOf(0, node); }

  std::vector<std::pair<dsa::dfg::Node*, int>>& dfg_nodes_of(int slot, ssnode* node) {
    return _nodeProp[node->id()].slots[slot].vertices;
  }

  /**
   * @brief Gets all the edges for a particular slot/link pair
   * 
   * @param slot the given slot
   * @param link the given link 
   * @return std::vector<EdgeSlice>& the set of edges for the given slot/link
   * pair
   */
  std::vector<EdgeSlice>& dfg_edges_of(int slot, sslink* link) {
    return _linkProp[link->id()].slots[slot].edges;
  }

  /**
   * @brief Get the Number of Slots for a Node
   * 
   * @param node the node to get the slots of
   * @return int the slots for the given node
   */
  int num_slots(ssnode* node) {
    return _nodeProp[node->id()].slots.size();
  }

  /**
   * @brief Get the Number of Slots for a given Link
   * 
   * @param link the link to get the slots of
   * @return int the slots for the given link
   */
  int num_slots(sslink* link) {
    return num_slots(link->source());
  }

  // we should depricate this?
  const mapper::VertexProp &locationOf(dsa::dfg::Node* dfgnode) {
    return _vertexProp[dfgnode->id()];
  }

  /**
   * @brief Gets if a dfgnode is currently schedule
   * 
   * @param dfgnode the dfgnode to check
   * @return bool if the given dfgnode is scheduled
   */
  bool is_scheduled(dsa::dfg::Node* dfgnode) {
    return _vertexProp[dfgnode->id()].node() != nullptr;
  }

  void stat_printOutputLatency();

  void set_model(SSModel* model) { _ssModel = model; }
  SSModel* ssModel() { return _ssModel; }

  // Assert error if problem with consistency of schedule
  void validate();

  bool fixLatency(int& lat, int& latmis, std::pair<int, int>& delay_violation);

  void clearAll() {
    _totalViolation = 0;
    _vertexProp.clear();
    _edgeProp.clear();
    _nodeProp.clear();
    _linkProp.clear();

    allocate_space();
  }

  // NOTE/WARN: interpretConfigBits creates a dfg object that should
  // be cleaned up later by the schedule object
  std::map<dsa::OpCode, int> interpretConfigBits(int size, uint64_t* bits);

  std::map<dsa::OpCode, int> interpretConfigBitsCheat(char* s);

  // TODO: Implement it to support MGRA
  // std::map<dsa::OpCode,int> interpretConfigBitsDedicated();

  void print_bit_loc() {
    std::cout << "Primary Config\n";
    std::cout << "Row: " << ROW_LOC << ":" << ROW_LOC + ROW_BITS - 1 << "\n";
    std::cout << "Switches: " << SWITCH_LOC << ":" << SWITCH_LOC + SWITCH_BITS - 1
              << "\n";
    std::cout << "FU Dir: " << FU_DIR_LOC << ":" << FU_DIR_LOC + FU_DIR_BITS - 1 << "\n";
    std::cout << "FU Pred Inv: " << FU_PRED_INV_LOC << ":"
              << FU_PRED_INV_LOC + FU_PRED_INV_BITS - 1 << "\n";
    std::cout << "Opcode: " << OPCODE_LOC << ":" << OPCODE_LOC + OPCODE_BITS - 1 << "\n";
    std::cout << "In Del.: " << IN_DELAY_LOC << ":" << IN_DELAY_LOC + IN_DELAY_BITS - 1
              << "\n";
  }

  void set_edge_delay(int i, dsa::dfg::Edge* e) { _edgeProp[e->id].extra_lat = i; }

  int edge_delay(dsa::dfg::Edge* e) { return _edgeProp[e->id].extra_lat; }

  void set_link_order(int slot, sslink* l, int i) {
    _linkProp[l->id()].slots[slot].order = i;
  }

  int link_order(std::pair<int, sslink*> l) {
    return _linkProp[l.second->id()].slots[l.first].order;
  }

  struct LinkProp;

  std::vector<LinkProp>& link_prop() { return _linkProp; }

  struct EdgeProp;

  std::vector<EdgeProp>& edge_prop() { return _edgeProp; }

  size_t num_passthroughs(dsa::dfg::Edge* e) {
    return _edgeProp[e->id].passthroughs.size();
  }

  int max_lat() {
    DSA_CHECK(_max_lat != -1);
    return _max_lat;
  }

  int max_lat_mis() { return _max_lat_mis; }

  void reset_lat_bounds() {
    for (auto& elem : _ssDFG->type_filter<dsa::dfg::InputPort>()) {
      auto& vp = _vertexProp[elem.id()];
      vp.min_lat = 0;
      vp.max_lat = 0;
    }
    for (auto& elem : _ssDFG->type_filter<dsa::dfg::Instruction>()) {
      auto& vp = _vertexProp[elem.id()];
      vp.min_lat = 0;
      vp.max_lat = INT_MAX - 1000;
    }
    for (auto& elem : _ssDFG->type_filter<dsa::dfg::OutputPort>()) {
      auto& vecp = _vertexProp[elem.id()];
      vecp.min_lat = 0;
      vecp.max_lat = INT_MAX - 1000;
    }
  }

 public:
  template <typename T>
  inline bool is_complete();

  template <typename T>
  inline int num_mapped() {
    return _num_mapped[T::KindValue];
  }

  unsigned num_links_mapped() { return _links_mapped; }

  unsigned num_edge_links_mapped() { return _edge_links_mapped; }

  inline unsigned num_left();

  void allocate_space() {
    if (_ssDFG) {
      _vertexProp.resize(_ssDFG->nodes.size());
      _edgeProp.resize(_ssDFG->edges.size());
      _groupMismatch.resize(_ssDFG->meta.size(), 0);
    }
    if (_ssModel) {
      _nodeProp.resize((size_t)_ssModel->subModel()->node_list().size());
      for (int i = 0, n = node_prop().size(); i < n; ++i) {
        _nodeProp[i].slots.resize(_ssModel->subModel()->node_list()[i]->lanes());
      }
      _linkProp.resize((size_t)_ssModel->subModel()->link_list().size());
      for (int i = 0, n = link_prop().size(); i < n; ++i) {
        link_prop()[i].slots.resize(
            (size_t)_ssModel->subModel()->link_list()[i]->source()->lanes());
      }
    }
  }

  int colorOf(dsa::dfg::Value* v);

  void get_overprov(int& ovr, int& agg_ovr, int& max_util);
  void get_link_overprov(sslink* link, int& ovr, int& agg_ovr, int& max_util);

  // Swaps the nodes from one schedule to another
  void swap_model(SpatialFabric* copy_sub) {
    for (auto& vp : _vertexProp) {
      if (vp.slot.ref) {
        vp.slot.ref = copy_sub->node_list()[vp.slot.ref->id()];  // bo ya
      }
    }
    for (auto& ep : _edgeProp) {
      for (auto& p : ep.links) {
        if (p.second) {
          p.second = copy_sub->link_list()[p.second->id()];
        }
      }
      for (auto& p : ep.passthroughs) {
        if (p.second) {
          p.second = copy_sub->node_list()[p.second->id()];
        }
      }
    }
  }


  void remove_link(int link_deleted_id) {
    _linkProp.erase(_linkProp.begin() + link_deleted_id);
  }
  
  void remove_node(int node_deleted_id) {
    _nodeProp.erase(_nodeProp.begin() + node_deleted_id);
  }

  struct EdgeProp {
    int num_links = 0;
    int extra_lat = 0;
    int vio = 0;  // temporary variable

    /*!
     * \brief Links used to route this edge, from source to destination.
     */
    std::vector<std::pair<int, sslink*>> links;

    std::vector<std::pair<int, ssnode*>> passthroughs;

    void reset() {
      num_links = 0;
      extra_lat = 0;
      links.clear();
      passthroughs.clear();
    }
  };

  struct NodeProp {
    /*! \brief To support the decomposability, we break a whole ssnode into slots. */
    struct NodeSlot {
      /*! \brief Edges pass through this node slot to route. */
      std::vector<dsa::dfg::Edge*> passthrus;
      /*! \brief Byte slot of the dfg node (inst/vec) that is mapped to this node slot. */
      std::vector<std::pair<dsa::dfg::Node*, int>> vertices;
    };
    /*! \brief The mapping information of each lane hardware lane. */
    std::vector<NodeSlot> slots;
  };

  struct LinkProp {
    struct LinkSlot {
      int lat = 0, order = -1;
      std::vector<EdgeSlice> edges;
    };
    /*! \brief The mapping information of each lane hardware lane. */
    std::vector<LinkSlot> slots;
  };

  std::vector<NodeProp>& node_prop() { return _nodeProp; }
  std::vector<mapper::VertexProp>& vex_prop() { return _vertexProp; }

  /*!
   * \brief Estimate the performance of this mapping.
   * \return The estimated instruction level parallelism on the fabric.
   */
  double estimated_performance();

 public:
  /*! \brief If each nodes in the DFG requires dynamic control in the hardware. */
  std::vector<bool> needs_dynamic;
  /*! \brief DFG is always a DAG. It stores its reversed topological order. */
  std::vector<dsa::dfg::Node*> reversed_topo;
  /*! \brief The gathered redundant operand edges of each node in the DFG. */
  std::vector<std::vector<dsa::dfg::Edge*>> operands;
  /*! \brief The gathered redundant user edges of each node in the DFG. */
  std::vector<std::vector<dsa::dfg::Edge*>> users;
  /*! \brief The distances among the nodes in the spatial hardware. */
  std::vector<std::vector<int>> distances;
  /*! \brief The data issue throughput of each sub-DFG. Used by simulation. */
  std::vector<int> group_throughput;
  /*! \brief The number of candidate spots of each DFG nodes. */
  std::vector<int> candidate_cnt;
  /*! \brief The total number of pass-through routings. */
  // TODO(@were): Make it `const' to prevent accidental modification.
  int total_passthrough{0};
  /*! \brief It normalizes the results of passes, and sync the DFG with the schedule. */
  void normalize();
  /*!
   * \brief The throughput penalty of violation.
   */
  std::pair<int, int> violation_penalty;

 private:
  /*! \brief The pointer to the spatial architecture. */
  SSModel* _ssModel;
  /*! \brief The pointer to the DFG. */
  SSDfg* _ssDFG;

  /*! \brief The gatherd sum of timing mismatch. */
  int _totalViolation = 0;
  /*! \brief The max latency of the DFG, and the max timing mismatch. */
  int _max_lat = -1, _max_lat_mis = -1;
  // TODO(@were): For simulation purpose, compute the mis of each group.

  /*! \brief The number mapped nodes in each data type. */
  unsigned _num_mapped[dsa::dfg::Node::V_NUM_TYPES] = {0};  // init all to zero
  /*! \brief Links occupied by edges. The edges mapped onto links. */
  int _links_mapped = 0, _edge_links_mapped = 0;

  /*! \brief The timing mismatch of each sub-DFG. */
  std::vector<int> _groupMismatch;
  /*! \brief The hardware information of each mapped DFG node. */
  std::vector<mapper::VertexProp> _vertexProp;
  /*! \brief The hardware information of each mapped DFG edge. */
  std::vector<EdgeProp> _edgeProp;
  /*! \brief The software information of each occupied spatial hardware node. */
  std::vector<NodeProp> _nodeProp;
  /*! \brief The software information of each occupied spatial hardware link. */
  std::vector<LinkProp> _linkProp;


  /*! \brief Expected edge latency used by timing. */
  // TODO(@were): Move these two to some constants.
  int _min_expected_route_latency = 2;
  int _max_expected_route_latency = 6;
};

template <>
inline int Schedule::num_mapped<dsa::dfg::Node*>() {
  return _num_mapped[dsa::dfg::Node::V_INST] + _num_mapped[dsa::dfg::Node::V_INPUT] +
         _num_mapped[dsa::dfg::Node::V_OUTPUT];
}

template <typename T>
inline bool Schedule::is_complete() {
  return (int)_ssDFG->type_filter<T>().size() == num_mapped<T>();
}

inline unsigned Schedule::num_left() {
  int num = _ssDFG->nodes.size() - num_mapped<dsa::dfg::Node*>();
  DSA_CHECK(num >= 0);
  return num;
}
