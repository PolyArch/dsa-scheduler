#pragma once

#include <algorithm>
#include <bitset>
#include <climits>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dsa/debug.h"
#include "fu_model.h"
#include "json/visitor.h"
#include "predict.h"

#define DEF_ATTR(attr)                         \
  decltype(attr##_) attr() { return attr##_; } \
  void attr(const decltype(attr##_) & new_value) { attr##_ = new_value; }

namespace dsa {

namespace adg {

class Visitor;

}

class SpatialFabric;

using std::ofstream;
using std::ostream;
using std::to_string;

const int MAX_SUBNETS = 8;

class ssnode;
class ssvport;

template <typename T>
T get_prop_attr(plain::Object& prop, const std::string& name, T dft) {
  auto iter = prop.find(name);
  if (iter != prop.end())
    return *iter->second->As<T>();
  else
    return dft;
}

template <typename T>
void fix_id(std::vector<T>& vec) {
  for (int i = 0, n = vec.size(); i < n; ++i) {
    vec[i]->id(i);
  }
}

template <typename T>
void vec_delete_by_id(std::vector<T>& vec, std::vector<int>& indices) {
  auto new_end = std::remove_if(vec.begin(), vec.end(), [&](T fu) {
    for (int index : indices) {
      if (fu->id() == index) return true;
    }
    return false;
  });
  vec.erase(new_end, vec.end());
}

// TODO: Should we delete this class?
class ssio_interface {
 public:
  std::map<int, ssvport*> vports_map[2];

  using EntryType = std::pair<int, ssvport*>;
  std::vector<EntryType> vports_vec[2];

  std::map<int, ssvport*>& vports(bool is_input) { return vports_map[is_input]; }

  ssvport* get(bool is_input, int id) {
    auto& ports = vports(is_input);
    auto iter = ports.find(id);
    CHECK(iter != ports.end()) << is_input << " " << id;
    return iter->second;
  }

  void fill_vec();
};

class sslink {
 public:
  sslink() {}

  ~sslink();

  /*!
   * \brief Construct a link with a pair of source and sink.
   */
  sslink(ssnode* source, ssnode* sink) : source_(source), sink_(sink) {}

  /*!
   * \brief The text format of this link for the purpose of logging.
   */
  std::string name() const;

  /*!
   * \brief If the timing of this link is dynamic.
   */
  bool flow_control();

  /*!
   * \brief The bitwidth of this connection.
   */
  int bitwidth();

  /*!
   * \brief The connectivity of the sub-network lanes. Refer the method
   *        below for more details.
   */
  // TODO(@were): Extend this to bitset for wider decomposability.
  std::vector<int64_t> subnet;

  /*!
   * \brief Giving a starting lane, and the required width, return the
   *        connected lanes.
   * \param slot The starting lane.
   * \param width The width of lanes on the subnetwork.
   * \return int The bitmask of feasible lanes.
   */
  int slots(int slot, int width);

 protected:
  int id_{-1};
  int max_util_{1};
  ssnode* source_{nullptr};
  ssnode* sink_{nullptr};

  friend class SpatialFabric;

 public:
  DEF_ATTR(id)
  DEF_ATTR(max_util)
  DEF_ATTR(source)
  DEF_ATTR(sink)
};

class ssnode {
 public:
  enum NodeType { FU, Switch, InPort, OutPort, Unknown };

  ssnode() {}

  ssnode(int datawidth, int granularity, int util, bool dynamic_timing, int fifo)
      : datawidth_(datawidth),
        granularity_(granularity),
        max_util_(util),
        flow_control_(dynamic_timing),
        max_delay_(fifo) {}

  // TODO(@were): Deprecate this in the visitor pattern.
  virtual ssnode* copy() = 0;

  /*!
   * \brief The entrance for visitor pattern.
   */
  virtual void Accept(adg::Visitor* visitor) = 0;

  /*!
   * \brief Connect this and the given node with a link.
   */
  sslink* add_link(ssnode* node);

  /*!
   * \brief The textformat of this node for the purpose of logging.
   */
  virtual std::string name() const = 0;

  /*!
   * \brief The size of the timing delay buffer.
   */
  int delay_fifo_depth() { return max_delay_; }

  /*!
   * \brief In degrees of this node.
   */
  std::vector<sslink*>& in_links() { return links_[1]; }

  /*!
   * \brief Out degrees of this node.
   */
  std::vector<sslink*>& out_links() { return links_[0]; }

  /*!
   * \brief The method of checking hanger node in the ADG used by DSE.
   */
  virtual bool is_hanger() { return false; }

  /*!
   * \brief The number of lanes available in this node.
   */
  int lanes() { return datawidth() / granularity(); }

  // TODO(@were): Can we move these to visitor pattern?
  virtual void dumpIdentifier(ostream& os) = 0;
  virtual void dumpFeatures(ostream& os) = 0;

  /*!
   * \brief Scheduling runtime, for the purpose of routing.
   */
  int node_dist(int slot) { return _node_dist[slot]; }
  std::pair<int, sslink*> came_from(int slot) { return _came_from[slot]; }
  int done(int slot) { return _done[slot]; }
  void set_done(int slot, int n) { _done[slot] = n; }
  void update_dist_only(int slot, int dist) { _node_dist[slot] = dist; }
  void update_dist(int slot, int dist, int from_slot, sslink* from) {
    _node_dist[slot] = dist;
    _came_from[slot] = std::make_pair(from_slot, from);
  }
  void reset_runtime_vals() {
    memset(_node_dist, -1, sizeof _node_dist);
    memset(_came_from, 0, sizeof _came_from);
    memset(_done, 0, sizeof _done);
  }

  bool is_shared() { return max_util_ > 1; }

  virtual ~ssnode() {}

  /*!
   * \brief The spatial fabric to which this node belongs to.
   */
  SpatialFabric* parent{nullptr};

 protected:
  /*!
   * \brief The identifier of this node.
   */
  int id_{-1};
  /*!
   * \brief The coordination of this node in a mesh. If the topology is
   *        irregular, these two numbers are set to -1.
   */
  int x_{-1};
  int y_{-1};
  /*!
   * \brief The width of the datapath of this node.
   */
  int datawidth_{64};
  /*!
   * \brief The bit width of the decomposability.
   */
  int granularity_{8};
  /*!
   * \brief =1 indicates dedicated, and >1 means shared.
   */
  int max_util_{1};
  /*!
   * \brief If the timing of execution is determined when compilation.
   */
  bool flow_control_{true};
  /*!
   * \brief The size of the local FIFO buffer to delay the timing.
   */
  int max_delay_{15};
  /*!
   * \brief The output, and input of this node.
   */
  std::vector<sslink*> links_[2];  // {output, input}

  // TODO(@were): Separate this out.
  int _node_dist[8];
  int _done[8];
  std::pair<int, sslink*> _came_from[8];

  friend class SpatialFabric;
  friend class sslink;
  friend class CodesignInstance;

 public:
  /*!
   * \brief Get set/attributes
   */
  DEF_ATTR(id)
  DEF_ATTR(x)
  DEF_ATTR(y)
  DEF_ATTR(datawidth)
  DEF_ATTR(granularity)
  DEF_ATTR(max_util)
  DEF_ATTR(flow_control)
  DEF_ATTR(max_delay)
  const std::vector<sslink*> &links(int x) { return links_[x]; }
};

class ssswitch : public ssnode {
 public:
  ssswitch() : ssnode() {}

  ssswitch(int datawidth, int granularity, int util, bool dynamic_timing, int fifo)
      : ssnode(datawidth, granularity, util, dynamic_timing, fifo) {}

  void Accept(adg::Visitor* visitor) override;

  ssnode* copy() override {
    auto res = new ssswitch();
    *res = *this;
    return res;
  }

  virtual std::string name() const override {
    std::stringstream ss;
    ss << "SW_";
    if (x_ != -1 && y_ != -1) {
      ss << x_ << "_" << y_;
    } else {
      ss << id_;
    }
    return ss.str();
  }
  void dumpIdentifier(ostream& os) override {
    os << "[" + to_string(id_) + ",\"switch\"" + "]";
  }
  void dumpFeatures(ostream& os) override {
    os << "{\n";
    // ID
    os << "\"id\" : " << id() << ",\n";
    // NodeType
    os << "\"nodeType\" : "
       << "\"switch\""
       << ",\n";
    // data width
    os << "\"data_width\" : " << datawidth() << ",\n";
    // granularity
    os << "\"granularity\" : " << granularity_ << ",\n";
    // number of input
    int num_input = in_links().size();
    os << "\"num_input\" : " << num_input << ",\n";
    // number of output
    int num_output = out_links().size();
    os << "\"num_output\" : " << num_output << ",\n";
    // flow control
    os << "\"flow_control\" : " << (flow_control() ? "true" : "false") << ",\n";
    // max util
    os << "\"max_util\" : " << max_util() << ",\n";
    // input nodes
    os << "\"input_nodes\" : [";
    int idx_link = 0;
    for (auto in_link : in_links()) {
      in_link->source()->dumpIdentifier(os);
      if (idx_link < num_input - 1) {
        idx_link++;
        os << ", ";
      }
    }
    os << "],\n";
    // output nodes
    os << "\"output_nodes\" : [";
    idx_link = 0;
    for (auto out_link : out_links()) {
      out_link->sink()->dumpIdentifier(os);
      if (idx_link < num_output - 1) {
        idx_link++;
        os << ", ";
      }
    }
    os << "]";

    os << "}\n";
  }

  void collect_features() {
    features[0] = max_util_ > 1 ? 0.0 : 1.0;
    features[1] = max_util_ > 1 ? 1.0 : 0.0;

    CHECK(features[0] || features[1]);
    features[2] = flow_control_ ? 0.0 : 1.0;
    features[3] = flow_control_ ? 1.0 : 0.0;
    CHECK(features[2] || features[3]) << "Either Data(Static) or DataValidReady(Dynamic)";
    features[4] = lanes();
    features[5] = max_delay();
    features[6] = links_[1].size();
    features[7] = links_[0].size();
    features[8] = max_util_;
  }

  virtual ~ssswitch() {}

  void print_features() {
    std::cout << "------ Features : >>>>>> ";
    std::cout << "Not Shared ? " << features[0] << ", "
              << "Shared ? " << features[1] << ", "
              << "Not Flow Control ? " << features[2] << ", "
              << "Flow Control ? " << features[3] << ", "
              << "decomposer = " << features[4] << ", "
              << "max fifo depth = " << features[5] << ", "
              << "# input links = " << features[6] << ", "
              << "# output links = " << features[7] << ", "
              << "max util = " << features[8] << ", ";
    std::cout << " ------ Feature Ends <<<<<<\n";
  }

  void dump_features() {
    for (int i = 0; i < 9; ++i) {
      std::cout << features[i] << " ";
    }
    std::cout << "\n";
  }

 protected:
  double features[9];
  // Sihao: routing_lookup = {0->3, 1->4, 3->1}
  // means output0 receive input3
  //       output1 receive input4
  //       output3 receive input1
  // for those output port not mapped, they are connect to ground
};

class ssfu : public ssnode {
 public:
  ssfu() : ssnode() {}

  ssfu(int datawidth, int granularity, int util, bool dynamic_timing, int fifo,
       const Capability& fu_type)
      : ssnode(datawidth, granularity, util, dynamic_timing, fifo), fu_type_(fu_type) {}

  ssnode* copy() override {
    auto res = new ssfu();
    *res = *this;
    return res;
  }

  void Accept(adg::Visitor* visitor);

  void dumpIdentifier(ostream& os) override {
    os << "[" + to_string(id_) + ",\"function unit\"" + "]";
  }

  void dumpFeatures(ostream& os) override {
    os << "{\n";
    // ID
    os << "\"id\" : " << id() << ",\n";
    // NodeType
    os << "\"nodeType\" : "
       << "\"function unit\""
       << ",\n";
    // data width
    os << "\"data_width\" : " << datawidth() << ",\n";
    // granularity
    os << "\"granularity\" : " << granularity_ << ",\n";
    // number of input
    int num_input = in_links().size();
    os << "\"num_input\" : " << num_input << ",\n";
    // number of output
    int num_output = out_links().size();
    os << "\"num_output\" : " << num_output << ",\n";
    // flow control
    os << "\"flow_control\" : " << (flow_control() ? "true" : "false") << ",\n";
    // max util
    os << "\"max_util\" : " << max_util() << ",\n";
    // max delay fifo depth
    os << "\"max_delay_fifo_depth\" : " << max_delay_ << ",\n";
    // number of register
    os << "\"num_register\" : " << register_file_size << ",\n";
    // Instructions
    os << "\"instructions\" : [";
    int idx_inst = 0;
    int num_inst = fu_type_.capability.size();
    for (auto& elem : fu_type_.capability) {
      os << "\"" << dsa::name_of_inst(elem.op) << "\"";
      if (idx_inst < num_inst - 1) {
        os << ", ";
        idx_inst++;
      }
    }
    os << "],\n";
    // input nodes
    os << "\"input_nodes\" : [";
    int idx_link = 0;
    for (auto in_link : in_links()) {
      in_link->source()->dumpIdentifier(os);
      if (idx_link < num_input - 1) {
        idx_link++;
        os << ", ";
      }
    }
    os << "],\n";
    // output nodes
    os << "\"output_nodes\" : [";
    idx_link = 0;
    for (auto out_link : out_links()) {
      out_link->sink()->dumpIdentifier(os);
      if (idx_link < num_output - 1) {
        idx_link++;
        os << ", ";
      }
    }
    os << "]";
    os << "}\n";
  }

  std::string name() const override {
    std::stringstream ss;
    if (x_ != -1 && y_ != -1) {
      ss << "FU" << x_ << "_" << y_;
    } else {
      ss << "FU" << id_;
    }
    return ss.str();
  }

  double* collect_features() {
    features[0] = max_util_ > 1 ? 0.0 : 1.0;
    features[1] = max_util_ > 1 ? 1.0 : 0.0;

    CHECK(features[0] || features[1]);
    features[2] = !flow_control_ ? 1.0 : 0.0;
    features[3] = flow_control_ ? 1.0 : 0.0;
    CHECK(features[2] || features[3]) << "Either Data(Static) or DataValidReady(Dynamic)";
    features[6] = lanes();
    features[7] = max_delay_;
    features[8] = links_[1].size();
    features[9] = links_[0].size();
    features[10] = register_file_size;
    features[11] = max_util_;

    // print_features();

    return features;
  }

  virtual ~ssfu() {}

  void print_features() {
    std::cout << " ------ Features : >>>>>> ";
    std::cout << "Not Shared ? " << features[0] << ", "
              << "Shared ? " << features[1] << ", "
              << "Not Flow Control ? " << features[2] << ", "
              << "Flow Control ? " << features[3] << ", "
              << "Output Mode Individual ? " << features[4] << ", "
              << "Output Mode Universal ? " << features[5] << ", "
              << "decomposer = " << features[6] << ", "
              << "max delay fifo depth = " << features[7] << ", "
              << "# input links = " << features[8] << ", "
              << "# output links = " << features[9] << ", "
              << "# register = " << features[10] << ", "
              << "max util = " << features[11] << ", ";
    std::cout << " ------ Feature Ends <<<<<<\n";
  }

  void dump_features() {
    for (int i = 0; i < 12; ++i) {
      std::cout << features[i] << " ";
    }
    std::cout << "\n";
  }

  bool is_hanger() override { return in_links().size() <= 1 || out_links().size() < 1; }

  Capability fu_type_;

 protected:
  double features[12];
  int register_file_size = 4;

 private:
  friend class SpatialFabric;
};

// This should be improved later
class ssvport : public ssnode {
 public:
  ssnode* copy() override {
    auto res = new ssvport();
    *res = *this;
    return res;
  }

  ssvport() {}

  ssvport(int datawidth, int granularity, int util, bool dynamic_timing, int fifo)
      : ssnode(datawidth, granularity, util, dynamic_timing, fifo) {}

  void Accept(adg::Visitor* vistor);

  std::vector<int>& port_vec() { return _port_vec; }
  void set_port_vec(std::vector<int> p) { _port_vec = p; }
  size_t size() { return _port_vec.size(); }
  std::string name() const override {
    std::stringstream ss;
    ss << "OI"[links_[0].size() > 0];
    if (port_ != -1) {
      ss << "P" << port_;
    } else {
      ss << id_;
    }
    return ss.str();
  }

  void dumpIdentifier(ostream& os) override {
    os << "[" + std::to_string(id_) + ",\"vector port\"" + "]";
  }
  void dumpFeatures(ostream& os) override {
    os << "{\n";
    // ID
    os << "\"id\" : " << id() << ",\n";
    // # Port
    os << "\"port\" : " << port() << ",\n";
    // NodeType
    os << "\"nodeType\" : "
       << "\"vector port\""
       << ",\n";
    // data width
    os << "\"data_width\" : " << datawidth() << ",\n";
    // granularity
    os << "\"granularity\" : " << granularity_ << ",\n";
    // number of input
    int num_input = in_links().size();
    os << "\"num_input\" : " << num_input << ",\n";
    // number of output
    int num_output = out_links().size();
    os << "\"num_output\" : " << num_output << ",\n";
    // flow control
    os << "\"flow_control\" : " << (flow_control() ? "true" : "false") << ",\n";
    // max util
    os << "\"max_util\" : " << max_util() << ",\n";
    // input nodes
    os << "\"input_nodes\" : [";
    int idx_link = 0;
    for (auto in_link : in_links()) {
      in_link->source()->dumpIdentifier(os);
      if (idx_link < num_input - 1) {
        idx_link++;
        os << ", ";
      }
    }
    os << "],\n";
    // output nodes
    os << "\"output_nodes\" : [";
    idx_link = 0;
    for (auto out_link : out_links()) {
      out_link->sink()->dumpIdentifier(os);
      if (idx_link < num_output - 1) {
        idx_link++;
        os << ", ";
      }
    }
    os << "]";
    os << "}\n";
  }

  int bitwidth_capability() {
    int res = 0;
    CHECK((int)links_[0].empty() + (int)links_[1].empty() == 1);
    for (auto& elem : links_) {
      for (auto link : elem) {
        res += link->bitwidth();
      }
    }
    return res;
  }

  void set_port2node(std::string portname, ssnode* node) { port2node[portname] = node; }
  ssnode* convert_port2node(std::string portname) { return port2node[portname]; }

  virtual ~ssvport(){};

  bool is_hanger() override { return in_links().empty() && out_links().empty(); }

 private:
  int port_ = -1;
  std::vector<int> _port_vec;
  std::string io_type;
  int channel_buffer;
  std::map<std::string, ssnode*> port2node;

 public:
  DEF_ATTR(port)
};

}  // namespace dsa

#undef DEF_ATTR