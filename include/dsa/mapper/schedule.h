#ifndef __SS__SCHEDULE_H__
#define __SS__SCHEDULE_H__

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <unordered_set>

#include <climits>

#include "dsa/mapper/config_defs.h"
#include "dsa/arch/model.h"
#include "dsa/arch/sub_model.h"
#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/visitor.h"

using namespace dsa;

#define MAX_SCHED_LAT 1000000

class Schedule {
  struct LastExecution {
    std::vector<std::pair<std::pair<int, ssnode*>, uint64_t>> kv;

    uint64_t* Find(std::pair<int, ssnode*> k) {
      static uint64_t res;
      for (int i = 0, n = kv.size(); i < n; ++i) {
        if (kv[i].first == k) {
          res = kv[i].second;
          return &res;
        }
      }
      return nullptr;
    }

    void Set(std::pair<int, ssnode*> k, uint64_t v) {
      for (int i = 0, n = kv.size(); i < n; ++i) {
        if (kv[i].first == k) {
          kv[i].second = v;
          return;
        }
      }
      kv.emplace_back(k, v);
    }
  };

 public:
  LastExecution lastExecutionKv;

  Schedule() {}

  // Read in schedule (both ssmodel, ssdfg, and schedule from file)
  Schedule(SSModel* model, SSDfg* dfg);


  constexpr static const float gvsf = 4.0f;

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

  // Rest of Stuff
  SSDfg* ssdfg() const { return _ssDFG; }

  void assign_lat(SSDfgNode* dfgnode, int lat) { _vertexProp[dfgnode->id()].lat = lat; }

  int latOf(SSDfgNode* dfgnode) { return _vertexProp[dfgnode->id()].lat; }

  void assign_lat_bounds(SSDfgNode* dfgnode, int min, int max) {
    auto& vertex_prop = _vertexProp[dfgnode->id()];
    vertex_prop.min_lat = min;
    vertex_prop.max_lat = max;
  }

  std::pair<int, int> lat_bounds(SSDfgNode* dfgnode) {
    auto& vertex_prop = _vertexProp[dfgnode->id()];
    return std::make_pair(vertex_prop.min_lat, vertex_prop.max_lat);
  }

  bool isPassthrough(int slot, ssnode* node) {
    return _nodeProp[node->id()].slots[slot].num_passthroughs > 0;
  }

  void assign_edge_pt(SSDfgEdge* edge, std::pair<int, ssnode*> pt) {
    if ((int)_edgeProp.size() <= edge->id()) {
      _edgeProp.resize(edge->id() + 1);
    }
    add_passthrough_node(pt.first, pt.second);
    _edgeProp[edge->id()].passthroughs.push_back(pt);
  }

  int groupMismatch(int g) { return _groupMismatch[g]; }

  int violation() { return _totalViolation; }

  void add_violation(int violation) {
    _totalViolation += violation;
    _max_lat_mis = std::max(_max_lat_mis, violation);
  }

  int vioOf(SSDfgNode* n) { return _vertexProp[n->id()].vio; }

  void record_violation(SSDfgNode* n, int violation) {
    _vertexProp[n->id()].vio = violation;
  }

  int vioOf(SSDfgEdge* e) { return _edgeProp[e->id()].vio; }
  void record_violation(SSDfgEdge* e, int violation) {
    _edgeProp[e->id()].vio = violation;
  }

  void check_all_links_consistency() {
    struct Checker : dfg::Visitor {
      Checker(Schedule *self) : self(self) {}
      void Visit(SSDfgNode *node) {
        for (auto &operand : node->ops()) {
          for (auto elem : operand.edges) {
            self->check_links_consistency(elem);
          }
        }
      }
      Schedule *self;
    };
    Checker checker(this);
    ssdfg()->Apply(&checker);
  }

  void check_links_consistency(SSDfgEdge* edge) {
    sslink* prev_link = nullptr;
    bool fail = false;
    int prev_num_edges = 10000;

    for (auto link : links_of(edge)) {
      if (prev_link != NULL && prev_link->dest() != link.second->orig()) {
        std::cout << edge->name() << " has Failed Link Order Check\n";
        fail = true;
      }

      std::unordered_set<SSDfgEdge*> edges;
      for (auto& i : edge_list(link.first, link.second)) {
        SSDfgEdge* alt_edge = i.first;
        // Only relevant edges are 1. same value, 2. same slice, 3. same slot
        if (alt_edge->val() == edge->val() && alt_edge->l() == edge->l() &&
            link.first == i.second) {
          edges.insert(edge);
        }
      }
      int new_num_edges = edges.size();

      if (new_num_edges > prev_num_edges) {
        std::cout << "\n" << edge->name() << " has Failed Edge Consistency Check\n";
        fail = true;  // BAD!
        break;
      }
      prev_num_edges = new_num_edges;
      prev_link = link.second;
    }
    if (fail) {
      for (auto link : links_of(edge)) {
        std::cout << link.second->name() << " #edges";

        std::unordered_set<SSDfgEdge*> edges;
        for (auto& i : edge_list(link.first, link.second)) {
          SSDfgEdge* alt_edge = i.first;
          if (alt_edge->val() == edge->val() && alt_edge->l() == edge->l() &&
              link.first == i.second) {
            edges.insert(edge);
          }
        }
        int new_num_edges = edges.size();

        std::cout << new_num_edges << " -- edges: ";
        for (auto i : edges) {
          std::cout << i->name() << " ";
        }
        std::cout << "\n";
      }
      printGraphviz("viz/sched-fail-incon-links.gv");
      assert(0 && "inconsistent links");
    }
  }

  int vecPortOf(SSDfgVec* vec) {
    ssnode* n = locationOf(vec);
    if (n) {
      auto vport = dynamic_cast<ssvport*>(n);
      if (!vport) {
        std::cerr << vec->name() << " " << n->name() << std::endl;
        assert(false);
      }
      return vport->port();
    } else {
      return -1;
    }
  }

  // Assign the ssnode to dfgnode and vice verse
  void assign_node(SSDfgNode* dfgnode, std::pair<int, ssnode*> assigned) {
    int vid = dfgnode->id();
    if (vid >= (int)_vertexProp.size()) {
      _vertexProp.resize(vid + 1);
    }
    int orig_slot = assigned.first;
    auto snode = assigned.second;

    assert(_vertexProp[vid].node == nullptr || _vertexProp[vid].node == snode);
    if (_vertexProp[vid].node == snode) return;

    _vertexProp[vid].node = snode;
    _vertexProp[vid].idx = orig_slot;
    _vertexProp[vid].width = dfgnode->bitwidth();

    assert(dfgnode->type() < SSDfgNode::V_NUM_TYPES);
    _num_mapped[dfgnode->type()]++;

    assert(dfgnode);

    assert(snode->id() < (int)_nodeProp.size());

    int num_slots = dfgnode->bitwidth() / 8;
    for (int i = 0; i < num_slots; ++i) {
      int slot = orig_slot + i;
      _nodeProp[snode->id()].slots[slot].vertices.emplace_back(dfgnode, orig_slot);
    }
  }

  void unassign_edge(SSDfgEdge* edge) {
    auto& ep = _edgeProp[edge->id()];

    _edge_links_mapped -= ep.links.size();

    // Remove all the edges for each of links
    for (auto& link : ep.links) {
      auto& lp = _linkProp[link.second->id()];

      int last_slot = edge->bitwidth() / 8;
      for (int i = 0; i < last_slot; ++i) {
        int slot_index = (link.first + i) % 8;
        auto& slot = lp.slots[slot_index];

        auto& edges = slot.edges;
        auto it =
            std::remove(edges.begin(), edges.end(), std::make_pair(edge, link.first));
        assert(it != edges.end());
        edges.erase(it, edges.end());
        if (slot.edges.empty()) {
          _links_mapped--;
          assert(_links_mapped >= 0);
        }
      }
    }

    // Remove all passthroughs associated with this edge
    for (auto& pt : ep.passthroughs) {
      auto& np = _nodeProp[pt.second->id()];
      np.slots[pt.first].num_passthroughs -= 1;  // take one passthrough edge away
      _num_passthroughs -= 1;
      assert(np.slots[pt.first].num_passthroughs >= 0);
    }

    _edgeProp[edge->id()].reset();
  }

  // Delete all scheduling data associated with dfgnode, including its
  // mapped locations, and mapping information and metadata for edges
  void unassign_dfgnode(SSDfgNode* dfgnode) {
    for (auto edge : dfgnode->in_edges()) {
      unassign_edge(edge);
    }
    for (auto edge : dfgnode->uses()) {
      unassign_edge(edge);
    }

    auto& vp = _vertexProp[dfgnode->id()];
    ssnode* node = vp.node;

    if (node) {
      int orig_slot = vp.idx;

      _num_mapped[dfgnode->type()]--;
      vp.node = nullptr;

      int num_slots = dfgnode->bitwidth() / 8;
      for (int i = 0; i < num_slots; ++i) {
        int slot = orig_slot + i;
        auto& vertices = _nodeProp[node->id()].slots[slot].vertices;

        auto it = std::remove(vertices.begin(), vertices.end(),
                              std::make_pair(dfgnode, orig_slot));
        assert(it != vertices.end());
        vertices.erase(it, vertices.end());
      }
    }
  }

  std::vector<std::pair<SSDfgEdge*, int>>& edge_list(int slot, sslink* link) {
    return _linkProp[link->id()].slots[slot].edges;
  }

  void print_all_mapped() {
    std::cout << "Vertices: ";
    // TODO: can't get vertex/edge from id, need to modify dfg to maintain
    for (unsigned i = 0; i < _vertexProp.size(); ++i) {
      auto& v = _vertexProp[i];
      if (v.node) {
        std::cout << i << "->" << v.node->name() << " ";
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

  void assign_link_to_edge(SSDfgEdge* dfgedge, int slot_index, sslink* slink) {
    assert(slink);
    assert(dfgedge);
    // assert(_assignLink[slink]==nullptr || _assignLink[slink] == dfgedge->def());
    // std::cout << dfgedge->name() << " assigned to "
    //          << slink->name() << "\n";

    _edge_links_mapped++;
    auto& lp = _linkProp[slink->id()];

    for (int i = 0; i < dfgedge->bitwidth() / 8; ++i) {
      int cur_slot_index = (slot_index + i) % 8;

      auto& slot = lp.slots[cur_slot_index];
      if (slot.edges.empty()) _links_mapped++;

      slot.edges.emplace_back(dfgedge, slot_index);
    }

    if ((int)_edgeProp.size() <= dfgedge->id()) {
      _edgeProp.resize(dfgedge->id() + 1);
    }
  }

  // pdg edge to sslink
  void assign_edgelink(SSDfgEdge* dfgedge, int slot, sslink* slink,
                       std::list<std::pair<int, sslink*>>::iterator it) {
    assign_link_to_edge(dfgedge, slot, slink);
    _edgeProp[dfgedge->id()].links.insert(it, std::make_pair(slot, slink));
  }

  // pdg edge to sslink
  void assign_edgelink(SSDfgEdge* dfgedge, int slot, sslink* slink) {
    assign_link_to_edge(dfgedge, slot, slink);
    _edgeProp[dfgedge->id()].links.push_back(std::make_pair(slot, slink));
  }

  // void print_links(SSDfgEdge* dfgedge) {
  //  for(auto& i : _assignLinkEdge[dfgedge]) {
  //    cout << i->name() << " ";
  //  }
  //  cout << "\n";
  //}

  int link_count(SSDfgEdge* dfgedge) { return _edgeProp[dfgedge->id()].links.size(); }

  int edge_latency(SSDfgEdge* pdgedge) {
    return _edgeProp[pdgedge->id()].links.size();
  }

  // Calculate the total possible delay through delay fifos
  int max_edge_delay(SSDfgEdge* pdgedge) {
    auto& ep = _edgeProp[pdgedge->id()];
    int total_delay = 0;
    for (auto& link : ep.links) {
      total_delay += link.second->dest()->delay_fifo_depth();
    }
    return total_delay;
  }

  std::list<std::pair<int, sslink*>>& links_of(SSDfgEdge* edge) {
    auto& ep = _edgeProp[edge->id()];
    return ep.links;
  }

  std::list<std::pair<int, ssnode*>>& thrus_of(SSDfgEdge* edge) {
    auto& ep = _edgeProp[edge->id()];
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
  SSDfgEdge* alt_edge_for_link(std::pair<int, sslink*> link, SSDfgEdge* e) {
    auto& slots = _linkProp[link.second->id()].slots;
    for (auto it : slots[link.first].edges) {
      SSDfgEdge* alt_e = it.first;
      // Only relevant edges are 1. same value, 2. same slice, 3. same slot
      if (alt_e->val() == e->val() && e->l() == alt_e->l() && link.first == it.second) {
        return alt_e;
      }
    }
    return nullptr;
  }

  // return cost_to_route
  // 0: free
  // 1: empty
  //>2: already there
  int routing_cost(std::pair<int, sslink*> link, SSDfgEdge* edge) {
    assert(link.second);

    if (edge->use()->needs_ctrl_dep() && !link.second->flow_control()) {
      return -1;
    }

    auto& slots = _linkProp[link.second->id()].slots;
    // Check all slots will be occupied empty.
    bool num_edges = 0;
    int last_slot = link.first + edge->bitwidth() / 8;
    for (int s = link.first; s < last_slot; ++s) {
      int slot = s % 8;
      num_edges = slots[slot].edges.size();
      if (num_edges != 0) break;
    }
    if (num_edges == 0) return 1;
    if (alt_edge_for_link(link, edge)) return 0;
    return num_edges + 1;
  }

  // Routing cost for inputs, but based on nodes instead of values
  int routing_cost_temporal_in(sslink* link, SSDfgVecInput* in_v) {
    assert(link);
    auto& vec = _linkProp[link->id()].slots[0].edges;
    if (vec.empty()) return 1;
    for (auto elem : vec) {
      SSDfgEdge* edge = elem.first;
      if (edge->def() == in_v) return 0;
    }
    return 2;
  }

  // Routing cost for outputs, but based on nodes instead of values
  int routing_cost_temporal_out(std::pair<int, sslink*> link, SSDfgNode* node,
                                SSDfgVecOutput* out_v) {
    assert(link.second);
    auto& vec = _linkProp[link.second->id()].slots[link.first].edges;
    if (vec.empty()) return 1;
    // It's free if the node is the same, or one of the use vectors is the same.
    for (auto elem : vec) {
      SSDfgEdge* edge = elem.first;
      if (edge->def() == node) return 0;
      if (edge->use() == out_v) return 0;
    }
    return 2;
  }

  // find first node for
  SSDfgNode* dfgNodeOf(int slot, sslink* link) {
    assert(link);
    auto& vec = _linkProp[link->id()].slots[slot].edges;
    return vec.empty() ? nullptr : (*vec.begin()).first->def();
  }

  // find first node for
  SSDfgNode* dfgNodeOf(sslink* link) { return dfgNodeOf(0, link); }

  // find first node for
  SSDfgNode* dfgNodeOf(int slot, ssnode* node) {
    auto& vec = _nodeProp[node->id()].slots[slot].vertices;
    if (!vec.empty()) {
      return vec.front().first;
    }
    return nullptr;
  }

  // find first node for
  SSDfgNode* dfgNodeOf(ssnode* node) { return dfgNodeOf(0, node); }

  std::vector<std::pair<SSDfgNode*, int>>& dfg_nodes_of(int slot, ssnode* node) {
    return _nodeProp[node->id()].slots[slot].vertices;
  }

  std::vector<std::pair<SSDfgEdge*, int>>& dfg_edges_of(int slot, sslink* link) {
    return _linkProp[link->id()].slots[slot].edges;
  }

  // probably eventually we will need to change slots per node ...
  int num_slots(ssnode* node) { return 8; }
  int num_slots(sslink* link) { return 8; }

  // we should depricate this?
  ssnode* locationOf(SSDfgNode* dfgnode) { return _vertexProp[dfgnode->id()].node; }

  std::pair<int, ssnode*> location_of(SSDfgNode* dfgnode) {
    return std::make_pair(_vertexProp[dfgnode->id()].idx,
                          _vertexProp[dfgnode->id()].node);
  }

  bool is_scheduled(SSDfgNode* dfgnode) {
    return _vertexProp[dfgnode->id()].node != nullptr;
  }

  void stat_printOutputLatency();

  void set_model(SSModel* model) { _ssModel = model; }
  SSModel* ssModel() { return _ssModel; }

  void iterativeFixLatency();

  // Assert error if problem with consistency of schedule
  void validate();

  void calcLatency(int& lat, int& latmis, bool warnMismatch = false);

  void cheapCalcLatency(int& lat, int& latmis);

  void calcNodeLatency(SSDfgNode*, int& lat, int& latmis);

  bool fixLatency(int& lat, int& latmis);

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

  void clear_ssdfg();

  void reset_simulation_state();

  void add_passthrough_node(int slot, ssnode* n) {
    _nodeProp[n->id()].slots[slot].num_passthroughs += 1;
    _num_passthroughs += 1;
  }

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

  void set_edge_delay(int i, SSDfgEdge* e) { _edgeProp[e->id()].extra_lat = i; }

  int edge_delay(SSDfgEdge* e) { return _edgeProp[e->id()].extra_lat; }

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

  size_t num_passthroughs(SSDfgEdge* e) { return _edgeProp[e->id()].passthroughs.size(); }

  int max_lat() {
    assert(_max_lat != -1);
    return _max_lat;
  }

  int max_lat_mis() { return _max_lat_mis; }

  int decode_lat_mis() { return _decode_lat_mis; }

  void set_decode_lat_mis(int i) { _decode_lat_mis = i; }

  void reset_lat_bounds() {
    for (auto elem : _ssDFG->nodes<SSDfgVecInput*>()) {
      auto& vp = _vertexProp[elem->id()];
      vp.min_lat = 0;
      vp.max_lat = 0;
    }
    for (auto elem : _ssDFG->nodes<SSDfgInst*>()) {
      auto& vp = _vertexProp[elem->id()];
      vp.min_lat = 0;
      vp.max_lat = INT_MAX - 1000;
    }
    for (auto elem : _ssDFG->nodes<SSDfgVecOutput*>()) {
      auto& vecp = _vertexProp[elem->id()];
      vecp.min_lat = 0;
      vecp.max_lat = INT_MAX - 1000;
    }
  }

  void set_name(std::string s) { _name = s; }

  std::string name() { return _name; }

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
      _vertexProp.resize((size_t)_ssDFG->num_node_ids());
      _edgeProp.resize((size_t)_ssDFG->num_edge_ids());
    }
    if (_ssModel) {
      _nodeProp.resize((size_t)_ssModel->subModel()->node_list().size());
      _linkProp.resize((size_t)_ssModel->subModel()->link_list().size());
    }
  }

  int colorOf(SSDfgValue* v);

  void get_overprov(int& ovr, int& agg_ovr, int& max_util);
  void get_link_overprov(sslink* link, int& ovr, int& agg_ovr, int& max_util);

  int num_passthroughs() { return _num_passthroughs; }

  // Swaps the nodes from one schedule to another
  void swap_model(SpatialFabric* copy_sub) {
    for (auto& vp : _vertexProp) {
      if (vp.node) {
        vp.node = copy_sub->node_list()[vp.node->id()];  // bo ya
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

  // Shuffle node and link properties post-delete
  void reorder_node_link(std::vector<ssnode*>& old_n, std::vector<sslink*>& old_l) {
    // first bulk copy node and link properties, b/c we're about to blow
    // everything away and shuffle
    auto copy_nodeProp = _nodeProp;
    auto copy_linkProp = _linkProp;

    _nodeProp.clear();
    _linkProp.clear();

    auto& new_node_list = _ssModel->subModel()->node_list();
    auto& new_link_list = _ssModel->subModel()->link_list();

    _nodeProp.resize(new_node_list.size());
    _linkProp.resize(new_link_list.size());

    // std::cout << _nodeProp.size() << "just resized\n";

    for (unsigned i = 0; i < copy_nodeProp.size(); ++i) {
      ssnode* n = old_n[i];
      // at this point, we don't know if this node has been deleted...
      // so to check, we are going to look up if its still there
      if (n->id() < (int)new_node_list.size() && new_node_list[n->id()] == n) {
        // ok, this node is still there, so perform the move
        _nodeProp[n->id()] = copy_nodeProp[i];
      }
    }
    for (unsigned i = 0; i < copy_linkProp.size(); ++i) {
      sslink* l = old_l[i];
      // at this point, we don't know if this link has been deleted...
      // so to check, we are going to look up if its still there
      if (l->id() < (int)new_link_list.size() && new_link_list[l->id()] == l) {
        // ok, this link is still there, so perform the move
        _linkProp[l->id()] = copy_linkProp[i];
      }
    }
    // std::cout << _nodeProp.size() << "just resized\n";
  }

  struct VertexProp {
    int min_lat = 0, max_lat = 0, lat = 0, vio = 0;
    ssnode* node = nullptr;
    int width = -1, idx = -1;
  };

  struct EdgeProp {
    int num_links = 0;
    int extra_lat = 0;
    int vio = 0;  // temporary variable
    std::list<std::pair<int, sslink*>> links;
    std::list<std::pair<int, ssnode*>> passthroughs;

    // For serialization only
    std::list<std::pair<int, int>> links_ser;
    std::list<std::pair<int, int>> passthroughs_ser;

    // std::vector<uint64_t> sched_index; //Temporary for debugging -- indicates the order

    void reset() {
      num_links = 0;
      extra_lat = 0;
      links.clear();
      passthroughs.clear();
    }
  };

  struct NodeProp {
    struct NodeSlot {
      int num_passthroughs = 0;
      // Integer here indicates the position to which the node was assigned
      std::vector<std::pair<SSDfgNode*, int>> vertices;
    };

    NodeSlot slots[8];
  };

  struct LinkProp {
    struct LinkSlot {
      int lat = 0, order = -1;
      // Integer here indicates the position to which the edge was assigned
      std::vector<std::pair<SSDfgEdge*, int>> edges;
    };

    LinkSlot slots[8];
  };

  std::vector<NodeProp>& node_prop() { return _nodeProp; }

 public:
  std::vector<bool> needs_dynamic;
  std::vector<SSDfgNode*> reversed_topo;
  std::vector<std::vector<SSDfgEdge*>> operands;
  std::vector<std::vector<SSDfgEdge*>> users;
  std::vector<std::vector<int>> distances;

 private:

  std::string _name;

  // Private Data
  SSModel* _ssModel;
  SSDfg* _ssDFG;

  int _totalViolation = 0;
  int _max_lat = -1, _max_lat_mis = -1;

  unsigned _num_mapped[SSDfgNode::V_NUM_TYPES] = {0};  // init all to zero
  int _links_mapped = 0, _edge_links_mapped = 0;

  std::map<int, int> _groupMismatch;
  std::vector<VertexProp> _vertexProp;
  std::vector<EdgeProp> _edgeProp;
  std::vector<NodeProp> _nodeProp;
  std::vector<LinkProp> _linkProp;

  int _decode_lat_mis = 0;

  // Can consider adjusting these as averages, or making edge-specific
  int _min_expected_route_latency = 2;
  int _max_expected_route_latency = 6;

  int _num_passthroughs = 0;

};

template <>
inline int Schedule::num_mapped<SSDfgNode>() {
  return _num_mapped[SSDfgNode::V_INST] + _num_mapped[SSDfgNode::V_INPUT] +
         _num_mapped[SSDfgNode::V_OUTPUT];
}

template <typename T>
inline bool Schedule::is_complete() {
  return (int)_ssDFG->nodes<T*>().size() == num_mapped<T>();
}

inline unsigned Schedule::num_left() {
  int num = _ssDFG->nodes<SSDfgNode*>().size() - num_mapped<SSDfgNode>();
  assert(num >= 0);
  return num;
}

#endif
