#include <iostream>

#include "dsa/arch/sub_model.h"

namespace dsa {

class SpatialFabric {
 public:
  // Port type of the substrate nodes
  // opensp -- dyser opensplyser N + N -1 ips
  // three ins -- Softbrain 3 x N
  // everywitch -- all switches has ops and ips
  enum class PortType { opensp, everysw, threein, threetwo };

  SpatialFabric() {}

  SpatialFabric(std::istream& istream, const std::vector<Capability*> &);

  SpatialFabric(int x, int y, PortType pt = PortType::opensp, int ips = 2, int ops = 2);

  void Apply(adg::Visitor *);

  void PrintGraphviz(std::ostream& os);

  void DumpHwInJson(const char* name) {
    ofstream os(name);
    std::cout << "Hardware JSON file: " << name << std::endl;
    if (!os.good()) {
      return;
    }

    os << "{\n";  // Start of the JSON file
    // Instruction Set
    int start_enc = 3;
    std::set<OpCode> ss_inst_set;
    os << "\"Instruction Set\" : {\n";
    for (ssnode* node : node_list()) {
      ssfu* fu_node = dynamic_cast<ssfu*>(node);
      if (fu_node != nullptr) {
        for (auto &elem: fu_node->fu_type_.capability) {
          ss_inst_set.insert(elem.op);
        }
      }
    }
    int num_total_inst = ss_inst_set.size();
    int idx_inst = 0;
    for (OpCode inst : ss_inst_set) {
      os << "\"" << dsa::name_of_inst(inst) << "\" : " << start_enc + (idx_inst++);
      if (idx_inst < num_total_inst) {
        os << ",";
      }
      os << "\n";
    }
    os << "},\n";

    // Links
    os << "\"links\" : [\n";  // The Start of Links
    int idx_link = 0;
    int size_links = link_list().size();
    for (auto link : link_list()) {
      os << "{\n";
      os << "\"source\":";
      link->orig()->dumpIdentifier(os);
      os << ",\n";
      os << "\"sink\":";
      link->dest()->dumpIdentifier(os);
      os << "}";
      if (idx_link < size_links - 1) {
        idx_link++;
        os << ",\n";  // Seperate the links
      }
    }
    os << "],\n";  // The End of Links

    // Nodes
    os << "\"nodes\" : [\n";  // The Start of Nodes
    int idx_node = 0;
    int size_nodes = node_list().size();
    for (auto node : node_list()) {
      node->dumpFeatures(os);
      if (idx_node < size_nodes - 1) {
        idx_node++;
        os << ",\n";
      }
    }
    os << "]\n";  // The End of Nodes

    os << "}\n";  // End of the JSON file
  }

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

  void clear_all_runtime_vals();

  const std::vector<sslink*>& link_list() { return _link_list; }

  const std::vector<ssnode*>& node_list() { return _node_list; }

  std::vector<ssvport*> vport_list() { return node_filter<ssvport*>(); }

  std::vector<ssvport*> vlist_impl(bool is_input) {
    std::vector<ssvport*> res;
    auto vports = vport_list();
    for (auto elem : vports) {
      if (elem->links[is_input].empty()) res.push_back(elem);
    }
    return res;
  }
  std::vector<ssvport*> input_list() { return vlist_impl(true); }
  std::vector<ssvport*> output_list() { return vlist_impl(false); }

  void add_input(int i, ssnode* n) { _io_map[true][i] = n; }
  void add_output(int i, ssnode* n) { _io_map[false][i] = n; }

  ssfu* add_fu() {
    auto* fu = new ssfu();
    add_node(fu);  // id and stuff
    return fu;
  }

  ssfu* add_fu(int x, int y) {
    auto* fu = add_fu();
    fu->setXY(x, y);
    return fu;
  }

  ssswitch* add_switch() {
    auto* sw = new ssswitch();
    add_node(sw);  // id and stuff
    return sw;
  }

  ssswitch* add_switch(int x, int y) {
    auto* sw = add_switch();
    sw->setXY(x, y);
    return sw;
  }

  ssvport* add_vport(bool is_input) {
    auto vport = new ssvport();
    add_node(vport);
    return vport;
  }

  ssvport* add_vport(bool is_input, int port_num) {
    ssvport* vport = add_vport(is_input);
    if (_ssio_interf.vports_map[is_input].count(port_num)) {
      std::cerr << "Error: Multiple " << (is_input ? "input" : "output")
                << " ports with port number " << port_num << "created\n\n";
      assert(false && "port duplication error");
    }
    _ssio_interf.vports_map[is_input][port_num] = vport;
    vport->set_port(port_num);
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
    }

    for (unsigned i = 0; i < _link_list.size(); ++i) {
      auto* link = _link_list[i];
      auto* copy_link = new sslink();
      *copy_link = *link;
      copy_sub->_link_list[i] = copy_link;

      // make the links point to the new nodes
      copy_link->_orig = copy_sub->_node_list[link->orig()->id()];
      copy_link->_dest = copy_sub->_node_list[link->dest()->id()];
    }

    // make the nodes point to the new links
    for (unsigned I = 0; I < _node_list.size(); ++I) {
      auto* node = _node_list[I];
      auto* copy_node = copy_sub->_node_list[I];
      for (int j = 0; j < 2; ++j) {
        for (unsigned i = 0; i < node->links[j].size(); ++i) {
          sslink* link = node->links[j][i];
          copy_node->links[j][i] = copy_sub->_link_list[link->id()];
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

  // These we can use the faster bulk vec delete
  void delete_nodes(std::vector<int> v) {
    vec_delete_by_id(_node_list, v);
    fix_id(_node_list);
  }
  void delete_links(std::vector<int> v) {
    vec_delete_by_id(_link_list, v);
    fix_id(_link_list);
  }

  // External add link -- used by arch. search
  sslink* add_link(ssnode* src, ssnode* dst) {
    if (auto out = dynamic_cast<ssvport*>(src)) {
      CHECK(!out->out_links().empty());
    }
    if (auto in = dynamic_cast<ssvport*>(dst)) {
      CHECK(!in->in_links().empty());
    }

    sslink* link = src->add_link(dst);
    link->set_id(_link_list.size());
    _link_list.push_back(link);

    return link;
  }

  // add node
  void add_node(ssnode* n) {
    n->set_id(_node_list.size());
    n->parent = this;
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
  void parse_json_without_boost(std::string filename);
  void post_process();

 private:
  void build_substrate(int x, int y);

  void connect_substrate(int x, int y, PortType pt, int ips, int ops,
                         int temp_x, int temp_y, int temp_width, int temp_height);

  // These are only valid after regroup_vecs()
  std::vector<ssnode*> _node_list;
  std::vector<ssfu*> _fu_list;
  std::vector<sslink*> _link_list;

  // Temporary Datastructures, only for constructing the mapping
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
template <>
inline std::vector<ssvport*> SpatialFabric::nodes() {
  return vport_list();
}

}