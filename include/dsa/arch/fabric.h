#pragma once

#include <iostream>
#include "dsa/arch/sub_model.h"
#include "dsa/core/singleton.h"

namespace dsa {

class SpatialFabric {
 public:
  // Port type of the substrate nodes
  // opensp -- dyser opensplyser N + N -1 ips
  // three ins -- Softbrain 3 x N
  // everywitch -- all switches has ops and ips
  enum class PortType { opensp, everysw, threein, threetwo };

  SpatialFabric() {}

  SpatialFabric(const SpatialFabric& c);

  SpatialFabric(std::istream& istream, const std::vector<Capability*>&);

  SpatialFabric(int x, int y, PortType pt = PortType::opensp, int ips = 2, int ops = 2);

  void Apply(adg::Visitor*);

  void PrintGraphviz(const std::string& name);

  void DumpHwInJson(const char* name);

  int sizex() { return _sizex; }

  int sizey() { return _sizey; }

  template <typename T>
  inline std::vector<T> nodes();

  template <typename T>
  T* random(std::function<bool(T*)> condition) {
    if (nodes<T*>().empty()) return nullptr;
    T* res = nodes<T*>()[rand() % nodes<T*>().size()];
    return f(res) ? res : nullptr;
  }

  template <typename T>
  std::vector<T> node_filter() {
    std::vector<T> res;
    for (auto elem : _node_list) {
      if (auto node = dynamic_cast<T>(elem)) {
        res.push_back(node);
      }
    }
    return res;
  }

  std::vector<ssfu*> fu_list() { return node_filter<ssfu*>(); }

  std::vector<ssswitch*> switch_list() { return node_filter<ssswitch*>(); }

  size_t num_fu() { return fu_list().size(); }

  void parse_io(std::istream& istream);

  ssio_interface& io_interf() { return _ssio_interf; }

  const std::vector<sslink*>& link_list() { return _link_list; }

  const std::vector<ssnode*>& node_list() { return _node_list; }

  std::vector<SyncNode*> vport_list() { return node_filter<SyncNode*>(); }

  std::vector<DataNode*> data_list() { return node_filter<DataNode*>(); }

  std::vector<ssgenerate*> gen_list() { return node_filter<ssgenerate*>(); }

  std::vector<ssregister*> reg_list() { return node_filter<ssregister*>(); }

  std::vector<ssscratchpad*> scratch_list() { return node_filter<ssscratchpad*>(); }

  std::vector<ssrecurrence*> recur_list() { return node_filter<ssrecurrence*>(); }

  std::vector<ssdma*> dma_list() { return node_filter<ssdma*>(); }

  std::vector<ssivport*> input_list() { return node_filter<ssivport*>(); }
  std::vector<ssovport*> output_list() { return node_filter<ssovport*>(); }

  void add_input(int i, ssnode* n) { _io_map[true][i] = n; }
  void add_output(int i, ssnode* n) { _io_map[false][i] = n; }

  ssfu* add_fu() {
    auto* fu = new ssfu();
    add_node(fu);  // id and stuff
    return fu;
  }

  ssfu* add_fu(int x, int y) {
    auto* fu = add_fu();
    fu->x(x);
    fu->y(y);
    return fu;
  }

  ssscratchpad* add_scratchpad() {
    auto* sp = new ssscratchpad();
    add_node(sp);
    return sp;
  }

  ssswitch* add_switch() {
    auto* sw = new ssswitch();
    add_node(sw);  // id and stuff
    return sw;
  }

  ssswitch* add_switch(int x, int y) {
    auto* sw = add_switch();
    sw->x(x);
    sw->y(y);
    return sw;
  }

  ssivport* add_input_vport() {
    auto* vp = new ssivport();
    add_node(vp);  // id and stuff
    return vp;
  }

  ssovport* add_output_vport() {
    auto* vp = new ssovport();
    add_node(vp);  // id and stuff
    return vp;
  }

  SyncNode* add_vport(bool is_input) {
    if (is_input) {
      return add_input_vport();
    } else {
      return add_output_vport();
    }
  }

  SyncNode* add_vport(bool is_input, int port_num) {
    SyncNode* vport = add_vport(is_input);
    vport->port(port_num);
    DSA_CHECK(!_ssio_interf.vports_map[is_input].count(port_num))
        << "Error: Multiple " << (is_input ? "input" : "output")
        << " ports with port number " << port_num << "created\n\n";
    _ssio_interf.vports_map[is_input][port_num] = vport;
    return vport;
  }

  // Creates a copy of the datastructre which gaurantees ordering
  // within the *_list datastructures (so they can be used for matching)
  SpatialFabric* copy() {
    SpatialFabric* copy_sub = new SpatialFabric();

    copy_sub->_sizex = _sizex;
    copy_sub->_sizey = _sizey;
    copy_sub->_ssio_interf = _ssio_interf;

    copy_sub->_node_list.resize(_node_list.size());
    copy_sub->_link_list.resize(_link_list.size());

    for (int i = 0, n = _node_list.size(); i < n; ++i) {
      auto node = _node_list[i];
      copy_sub->_node_list[i] = node->copy();
      node->parent = this;
    }

    for (unsigned i = 0; i < _link_list.size(); ++i) {
      auto* link = _link_list[i];
      auto* copy_link = new sslink();
      *copy_link = *link;
      copy_sub->_link_list[i] = copy_link;

      // make the links point to the new nodes
      copy_link->source_ = copy_sub->_node_list[link->source()->id()];
      copy_link->sink_ = copy_sub->_node_list[link->sink()->id()];
    }

    // make the nodes point to the new links
    for (unsigned I = 0; I < _node_list.size(); ++I) {
      auto* node = _node_list[I];
      auto* copy_node = copy_sub->_node_list[I];
      for (int j = 0; j < 2; ++j) {
        for (unsigned i = 0; i < node->links_[j].size(); ++i) {
          sslink* link = node->links_[j][i];
          copy_node->links_[j][i] = copy_sub->_link_list[link->id()];
        }
      }
    }

    return copy_sub;
  }

  // Efficient bulk delete from vector based on indices (O(n))
  template <typename T>
  void bulk_vec_delete(std::vector<T>& vec, std::vector<int>& indices) {
    indices.push_back(INT_MAX);
    std::sort(indices.begin(), indices.end(), [](int x, int y) { return x < y; });

    int ind = 0;
    unsigned nindex = indices[ind];
    int diff = 0;

    for (unsigned i = 0; i < vec.size(); ++i) {
      if (diff > 0) {
        vec[i - diff] = vec[i];
      }
      if (i > nindex) {
        nindex = indices[++ind];
        diff++;
      }
    }

    vec.resize(vec.size() - diff);
  }

  void delete_node(int node_index) {
    delete_by_id(_node_list, node_index);
    fix_id(_node_list);

    // Fix Compute and Sync Ids
    fix_local_id(fu_list());
    fix_local_id(switch_list());
    fix_local_id(input_list());
    fix_local_id(output_list());

    // Fix Id for memory nodes
    fix_local_id(scratch_list());
    fix_local_id(recur_list());
    fix_local_id(dma_list());
    fix_local_id(gen_list());
  }

  void delete_link(int link_index) {
    delete_by_id(_link_list, link_index);
    fix_id(_link_list);
  }

  /**
   * @brief Adds a link to the spatial fabric. Adds the link from the source
   * node to the sink node.
   * 
   * @param src node that represents the source
   * @param dst node that represents the sink
   * @param source_position position to add the link to source node, 
   * by default -1
   * @param sink_position position to add the link to sink node, by default -1
   * @return sslink* the created link between the source and destination nodes
   */
  sslink* add_link(ssnode* src, ssnode* dst, int source_position=-1, int sink_position=-1) {
    // Check if a self-link
    if (src->id() == dst->id()) 
      return nullptr;
    
    // Disable connections breaking dfg
    if (dynamic_cast<SyncNode*>(src) && dynamic_cast<SyncNode*>(dst))
      return nullptr;
    if (dynamic_cast<DataNode*>(src) && dynamic_cast<DataNode*>(dst))
      return nullptr;
    if (dynamic_cast<DataNode*>(src) && dynamic_cast<SpatialNode*>(dst))
      return nullptr;
    if (dynamic_cast<SpatialNode*>(src) && dynamic_cast<DataNode*>(dst))
      return nullptr;
    if (dynamic_cast<SpatialNode*>(src) && dynamic_cast<ssivport*>(dst))
      return nullptr;
    if (dynamic_cast<ssovport*>(src) && dynamic_cast<SpatialNode*>(dst))
      return nullptr;

    if (dynamic_cast<DataNode*>(src) && dynamic_cast<ssovport*>(dst))
      return nullptr;
    if (dynamic_cast<ssivport*>(src) && dynamic_cast<DataNode*>(dst))
      return nullptr;

    // Check if a link already exists
    for (auto link : src->out_links())
      if (link->sink()->id() == dst->id()) 
        return nullptr;

    sslink* link = src->add_link(dst, source_position, sink_position);
    link->id(_link_list.size());
    _link_list.push_back(link);

    src->setRoutingTableSize();
    dst->setRoutingTableSize();
      
    link->source()->addLinkToRoutingTable(link);
    link->sink()->addLinkToRoutingTable(link);

    return link;
  }

  // add node
  void add_node(ssnode* n) {
    // Set ID
    n->id(_node_list.size());

    
    // Set Local ID
    if (n->localId() == -1) {
      if (auto fu = dynamic_cast<ssfu*>(n)) {
        fu->localId(fu_list().size());
      } else if (auto sw = dynamic_cast<ssswitch*>(n)) {
        sw->localId(switch_list().size());
      } else if (auto vport = dynamic_cast<ssivport*>(n)) {
        vport->localId(input_list().size());
      } else if (auto vport = dynamic_cast<ssovport*>(n)) {
        vport->localId(output_list().size());
      } else if (auto scratchpad = dynamic_cast<ssscratchpad*>(n)) {
        scratchpad->localId(scratch_list().size());
      } else if (auto recur = dynamic_cast<ssrecurrence*>(n)) {
        recur->localId(recur_list().size());
      } else if (auto dma = dynamic_cast<ssdma*>(n)) {
        dma->localId(dma_list().size());
      } else if (auto gen = dynamic_cast<ssgenerate*>(n)) {
        gen->localId(gen_list().size());
      } else if (auto reg = dynamic_cast<ssregister*>(n)) {
        reg->localId(reg_list().size());
      } else {
        DSA_CHECK(false) << "Unknown node type: " << n->name();
      }
    }

    // Set Parent
    n->parent = this;

    // Push into node list
    _node_list.push_back(n);
  }

  virtual ~SpatialFabric() {
    for (sslink* l : _link_list) {
      delete l;
    }
    for (ssnode* n : _node_list) {
      delete n;
    }
  }

  void post_process();

 private:
  // These are only valid after regroup_vecs()
  std::vector<ssnode*> _node_list;
  std::vector<sslink*> _link_list;

  // TODO(@were): Deprecate these data structure after moving to JSON DSL.
  void build_substrate(int x, int y);
  void connect_substrate(int x, int y, PortType pt, int ips, int ops, int temp_x,
                         int temp_y, int temp_width, int temp_height);
  int _sizex, _sizey;  // size of SS cgra
  std::map<int, ssnode*> _io_map[2];
  ssio_interface _ssio_interf;
};

template <>
inline std::vector<ssfu*> SpatialFabric::nodes() {
  return fu_list();
}
template <>
inline std::vector<ssnode*> SpatialFabric::nodes() {
  return _node_list;
}

}  // namespace dsa
