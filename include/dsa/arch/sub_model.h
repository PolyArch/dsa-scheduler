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
#include "dsa/core/singleton.h"
#include "dsa/debug.h"
#include "fu_model.h"
#include "json/value.h"


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
class SyncNode;

template <typename T>
T get_prop_attr(Json::Value& prop, const std::string& name, T dft) {
  return prop.get("name", dft);
}

template <typename T>
void fix_id(std::vector<T>& vec) {
  for (int i = 0, n = vec.size(); i < n; ++i) {
    vec[i]->id(i);
  }
}

template <typename T>
void fix_local_id(std::vector<T> vec) {
  for (int i = 0, n = vec.size(); i < n; ++i) {
    vec[i]->localId(i);
  }
}

template <typename T>
void delete_by_id(std::vector<T>& vec, int index) {
  vec.erase(vec.begin() + index);
}

// TODO: Should we delete this class?
class ssio_interface {
 public:
  std::map<int, SyncNode*> vports_map[2];

  using EntryType = std::pair<int, SyncNode*>;
  std::vector<EntryType> vports_vec[2];

  std::map<int, SyncNode*>& vports(bool is_input) { return vports_map[is_input]; }

  SyncNode* get(bool is_input, int id) {
    auto& ports = vports(is_input);
    auto iter = ports.find(id);
    DSA_CHECK(iter != ports.end()) << is_input << " " << id;
    return iter->second;
  }

  void fill_vec();
};

/*!
 * \brief The struct of component connection.
 *        NOTE: This is not actually a hardware component. Just a set of helpers.
 */
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
   * \brief The bandwith of this connection.
   */
  int bitwidth();

  /*!
   * \brief The connectivity of the sub-network lanes. Refer the method
   *        below for more details.
   */
  // TODO(@were): Extend this to bitset for wider decomposability.
  std::vector<uint64_t> subnet;

  /*!
   * \brief Giving a starting lane, and the required width, return the
   *        connected lanes.
   * \param slot The starting lane.
   * \param width The width of lanes on the subnetwork.
   * \return int The bitmask of feasible lanes.
   */
  int slots(int slot, int width);

  void resetSubnet();

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
  enum NodeType { FunctionUnit, Switch, InputVectorPort, OutputVectorPort, Scratchpad, DirectMemoryAccess, Register, Generate, Recurrance, None };

  ssnode() {}

  ssnode(NodeType type)
      : type_(type) {}

  // TODO(@were): Deprecate this in the visitor pattern.
  virtual ssnode* copy() = 0;

  /*!
   * \brief Overload Equality operators
   */
  bool operator==(const ssnode& other) {
    return this->id_ == other.id_;
  }

  bool operator!=(const ssnode& other) {
    return this->id_ != other.id_;
  }

  bool spatial() {
    if (type_ == FunctionUnit || type_ == Switch) {
      return true;
    } else {
      return false;
    }
  }

  bool sync() {
    if (type_ == InputVectorPort || type_ == OutputVectorPort) {
      return true;
    } else {
      return false;
    }
  }

  bool data() {
    if (type_ == Scratchpad || type_ == DirectMemoryAccess || type_ == Register || type_ == Generate || type_ == Recurrance) {
      return true;
    } else {
      return false;
    }
  }

  /*!
   * \brief The entrance for visitor pattern.
   */
  virtual void Accept(adg::Visitor* visitor) = 0;

  /*!
   * \brief Connect this and the given node with a link.
   */
  sslink* add_link(ssnode* node, int source_position=-1, int sink_position=-1, bool insert=true);

  /*!
   * \brief Add Empty Link to fill link slot. Used during ADG Construction
   */
  void add_empty_link(ssnode* node);

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

  void shuffle_links(bool in_links) {
    std::random_shuffle(links_[in_links].begin(), links_[in_links].end());
  }

  /*!
   * \brief Index of a link
   */
  int link_index(sslink* link, bool in_link) {
    auto it = std::find(links_[in_link].begin(), links_[in_link].end(), link);
    if (it != links_[in_link].end()) {
      return std::distance(links_[in_link].begin(), it);
    } else {
      return -1;
    }
  }

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
  /**
   * @brief Local Node Id, ID per node type
   * 
   */
  int localId_{-1};
  /*!
   * \brief The coordination of this node in a mesh. If the topology is
   *        irregular, these two numbers are set to -1.
   */
  // TODO(@were): Move this to fabric class.
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

  NodeType type_{None};

  friend class SpatialFabric;
  friend class sslink;
  friend class CodesignInstance;

 public:
  /*!
   * \brief Get set/attributes
   */
  DEF_ATTR(id)
  DEF_ATTR(localId)
  DEF_ATTR(x)
  DEF_ATTR(y)
  DEF_ATTR(type);
  DEF_ATTR(datawidth)
  DEF_ATTR(granularity)
  DEF_ATTR(max_util)
  DEF_ATTR(flow_control)
  DEF_ATTR(max_delay)
  const std::vector<sslink*> &links(int x) { return links_[x]; }
};

////////////////////////////////////////////////////////////////
//                                                            //
//                     substructures                          //
//                                                            //
////////////////////////////////////////////////////////////////

class SpatialNode : public ssnode {
 public:
  SpatialNode(NodeType type)
      : ssnode(type) {}

  SpatialNode() 
      : ssnode() {}

  virtual std::string name() const = 0;
  virtual void Accept(adg::Visitor* visitor) = 0;
  virtual void dumpIdentifier(ostream& os) = 0;
  virtual void dumpFeatures(ostream& os) = 0;
  virtual ~SpatialNode() {}
};

class SyncNode : public ssnode {
 public:
  SyncNode(NodeType type)
      : ssnode(type) {}

  SyncNode() 
      : ssnode() {}

  virtual std::string name() const = 0;
  virtual void Accept(adg::Visitor* visitor) = 0;
  virtual void dumpIdentifier(ostream& os) = 0;
  virtual void dumpFeatures(ostream& os) = 0;
  virtual bool isInputPort() = 0;
  virtual bool isOutputPort() = 0;
  virtual int bitwidth_capability() = 0;
  virtual bool isHanger() = 0;
  virtual int busWidth() = 0;
  virtual bool padded() = 0;
  virtual ~SyncNode() {}

  std::vector<int>& port_vec() { return _port_vec; }
  void set_port_vec(std::vector<int> p) { _port_vec = p; }
  void set_port2node(std::string portname, ssnode* node) { port2node[portname] = node; }
  ssnode* convert_port2node(std::string portname) { return port2node[portname]; }
  size_t size() { return _port_vec.size(); }

 protected:
  int port_ = -1;
  std::vector<int> _port_vec;
  std::string io_type;
  int channel_buffer;
  std::map<std::string, ssnode*> port2node;

  // Vector Port Implementation (Both input and output vector port)
  // Example: A 8-wide vector port is used as a 4-wide vector port
  // 0 : Full XBar Implementation: Any-to-any connection
  //     Memory A[3:0] -> Compute [ A[1], [X], A[3], [X], [X], [X], A[2], A[0] ]
  // 1 : Limited XBar Implementation: To any port, but ordered
  //     Memory A[3:0] -> Compute [ A[3], [X], A[2], [X], [X], [X], A[1], A[0] ]
  // 2 : Non-XBar Implementation: Only go to lowest ports
  //     Memory A[3:0] -> Compute [ [X], [X], [X], [X], A[3], A[2], A[1], A[0] ]
  int vp_impl_{2};

  // Stream Stated Vector Port (Both input and output vector port)
  // Whether this vector port support state predication (for control),
  //  and padding (input vector port only) for no-balanced unrolling, which depends on state
  bool vp_stated_{true};

 public:
  DEF_ATTR(port);         // Both
  DEF_ATTR(vp_impl);       // IVP & OVP
  DEF_ATTR(vp_stated);     // IVP & OVP
};

class DataNode : public ssnode {
 public:
  DataNode(NodeType type)
      : ssnode(type) {}

  DataNode() 
      : ssnode() {}

  virtual std::string name() const = 0;
  virtual void Accept(adg::Visitor* visitor) = 0;
  virtual void dumpIdentifier(ostream& os) = 0;
  virtual void dumpFeatures(ostream& os) = 0;
  virtual ~DataNode() {}

 protected:

  int numWrite_{1};
  int memUnitBits_{8};
  int numRead_{1};
  int maxLength1D_{2147483646};
  int maxLength3D_{2147483646};
  int64_t capacity_{1024};
  bool linearLength1DStream_{true};
  int numGenDataType_{1};
  bool linearPadding_{true};
  int maxAbsStretch3D2D_{1073741822};
  int numPendingRequest_{16};
  int numLength1DUnitBitsExp_{4};
  int maxAbsStride3D_{1073741822};
  int maxAbsStride1D_{1073741824};
  bool indirectStride2DStream_{true};
  int numIdxUnitBitsExp_{4};
  int maxAbsDeltaStride2D_{1073741822};
  bool linearStride2DStream_{true};
  int maxLength2D_{2147483646};
  int maxAbsStretch2D_{1073741822};
  int numMemUnitBitsExp_{4};
  int maxAbsStretch3D1D_{1073741822};
  bool indirectIndexStream_{true};
  int numStride2DUnitBitsExp_{4};
  int writeWidth_{32};
  int maxAbsStride2D_{1073741822};
  int readWidth_{32};
  bool streamStated_{true};
  int numSpmBank_{4};
  bool indirectLength1DStream_{true};
  int maxAbsDeltaStretch2D_{1073741822};

  std::vector<std::string> atomicOperations_{};

public:
  DEF_ATTR(numWrite);
  DEF_ATTR(memUnitBits);
  DEF_ATTR(numRead);
  DEF_ATTR(maxLength1D);
  DEF_ATTR(maxLength3D);
  DEF_ATTR(capacity);
  DEF_ATTR(linearLength1DStream);
  DEF_ATTR(numGenDataType);
  DEF_ATTR(linearPadding);
  DEF_ATTR(maxAbsStretch3D2D);
  DEF_ATTR(numPendingRequest);
  DEF_ATTR(numLength1DUnitBitsExp);
  DEF_ATTR(maxAbsStride3D);
  DEF_ATTR(maxAbsStride1D);
  DEF_ATTR(indirectStride2DStream);
  DEF_ATTR(numIdxUnitBitsExp);
  DEF_ATTR(maxAbsDeltaStride2D);
  DEF_ATTR(linearStride2DStream);
  DEF_ATTR(maxLength2D);
  DEF_ATTR(maxAbsStretch2D);
  DEF_ATTR(numMemUnitBitsExp);
  DEF_ATTR(maxAbsStretch3D1D);
  DEF_ATTR(indirectIndexStream);
  DEF_ATTR(numStride2DUnitBitsExp);
  DEF_ATTR(writeWidth);
  DEF_ATTR(maxAbsStride2D);
  DEF_ATTR(readWidth);
  DEF_ATTR(streamStated);
  DEF_ATTR(numSpmBank);
  DEF_ATTR(indirectLength1DStream);
  DEF_ATTR(maxAbsDeltaStretch2D);
  DEF_ATTR(atomicOperations);
};

////////////////////////////////////////////////////////////////
//                                                            //
//                       hardware                             //
//                                                            //
////////////////////////////////////////////////////////////////

class ssswitch : public SpatialNode {
 public:

  ssswitch()
      : SpatialNode(NodeType::Switch) {}

  void Accept(adg::Visitor* visitor) override;

  ssnode* copy() override {
    auto *res = new ssswitch();
    *res = *this;
    return res;
  }

  virtual std::string name() const override {
    std::stringstream ss;
    ss << "SW" << localId_;
    return ss.str();
  }
  void dumpIdentifier(ostream& os) override {
    os << "[" + to_string(localId_) + ",\"switch\"" + "]";
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

    DSA_CHECK(features[0] || features[1]);
    features[2] = flow_control_ ? 0.0 : 1.0;
    features[3] = flow_control_ ? 1.0 : 0.0;
    DSA_CHECK(features[2] || features[3]) << "Either Data(Static) or DataValidReady(Dynamic)";
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
};

class ssfu : public SpatialNode {
 public:
  ssfu() : SpatialNode(NodeType::FunctionUnit) {}

  SpatialNode* copy() override {
    auto res = new ssfu();
    *res = *this;
    return res;
  }

  void Accept(adg::Visitor* visitor) override;

  void dumpIdentifier(ostream& os) override {
    os << "[" + to_string(localId_) + ",\"function unit\"" + "]";
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
    os << "\"num_register\" : " << regFileSize() << ",\n";
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
    ss << "FU" << localId_;
    return ss.str();
  }

  double* collect_features() {
    features[0] = max_util_ > 1 ? 0.0 : 1.0;
    features[1] = max_util_ > 1 ? 1.0 : 0.0;

    DSA_CHECK(features[0] || features[1]);
    features[2] = !flow_control_ ? 1.0 : 0.0;
    features[3] = flow_control_ ? 1.0 : 0.0;
    DSA_CHECK(features[2] || features[3]) << "Either Data(Static) or DataValidReady(Dynamic)";
    features[6] = lanes();
    features[7] = max_delay_;
    features[8] = links_[1].size();
    features[9] = links_[0].size();
    features[10] = regFileSize();
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


 protected:
  Capability fu_type_;

  // Function Unit related parameter
  int maxOpRepeat_{1};
  int definedLatency_{1};
  int instSlotSize_{1};
  // bool fuOpDynamic = dynamic_timing // use ssnode parameter for this
  // int instDelayFifoDepth_{} // use ssnode delay fifo depth


  // Register File related parameter
  int regFileSize_{0};
  bool asyncReg_{false};
  bool updReg_{false};
  std::vector<int> resettableRegIdx_{0};
  
  // Control related parameter
  bool inputCtrl_{false};
  bool outputCtrl_{false};
  int ctrlLUTSize_{0};
  bool operandReuse_{false};
  bool resultDiscard_{false};
  bool registerReset_{false};
  bool abstain_{false};

  // Feature Collection
  double features[12];

  private:
    friend class SpatialFabric;

  public:
    // Parameters
    DEF_ATTR(fu_type);

    // Function Unit Parameter Get/Set
    DEF_ATTR(maxOpRepeat);
    DEF_ATTR(definedLatency);
    DEF_ATTR(instSlotSize);

    // Register Parameter Get/Set
    DEF_ATTR(regFileSize);
    DEF_ATTR(asyncReg);
    DEF_ATTR(updReg);
    DEF_ATTR(resettableRegIdx);
    
    // Control Parameter Get/Set
    DEF_ATTR(inputCtrl);
    DEF_ATTR(outputCtrl);
    DEF_ATTR(ctrlLUTSize);
    DEF_ATTR(operandReuse);
    DEF_ATTR(resultDiscard);
    DEF_ATTR(registerReset);
    DEF_ATTR(abstain);

};

class ssconverge : public SpatialNode {
 public:

  ssconverge()
      : SpatialNode(NodeType::Switch) {}

  void Accept(adg::Visitor* visitor) override;

  ssnode* copy() override {
    auto *res = new ssconverge();
    *res = *this;
    return res;
  }

  virtual std::string name() const override {
    std::stringstream ss;
    ss << "CONV" << localId_;
    return ss.str();
  }
  void dumpIdentifier(ostream& os) override {
  }
  void dumpFeatures(ostream& os) override {
  }

  virtual ~ssconverge() {}
};

class ssdiverge : public SpatialNode {
 public:

  ssdiverge()
      : SpatialNode(NodeType::Switch) {}

  void Accept(adg::Visitor* visitor) override;

  ssnode* copy() override {
    auto *res = new ssdiverge();
    *res = *this;
    return res;
  }

  virtual std::string name() const override {
    std::stringstream ss;
    ss << "CONV" << localId_;
    return ss.str();
  }
  void dumpIdentifier(ostream& os) override {
  }
  void dumpFeatures(ostream& os) override {
  }

  virtual ~ssdiverge() {}
};


class ssivport : public SyncNode {
 public:

  ssnode* copy() override {
    auto res = new ssivport();
    *res = *this;
    return res;
  }

  ssivport()
      : SyncNode(NodeType::InputVectorPort) {}

  void Accept(adg::Visitor* vistor) override;
  
  std::string name() const override {
    std::stringstream ss;
    ss << "I";
    /*
    if (port_ != -1) {
      ss << "P" << port_ << "_";
    } */
    ss << localId_;
    
    return ss.str();
  }

  // Check the direction of vector port, whether or not is input vector port
  bool isInputPort() override { return true; }

  // Check whether it is output vector port
  bool isOutputPort() override { return false; }

  int busWidth() override {
    int busWidth = 0; 
    for (auto link : in_links()) {
      if (auto memory = dynamic_cast<DataNode*>(link->source())) {
        busWidth = std::max(busWidth, memory->readWidth());
      }
    }
    busWidth = std::min(busWidth, (int) (bitwidth_capability() / 8));
    busWidth = std::min(busWidth, 8);
    return busWidth; 
  }

  bool padded() override {
    bool padded = false;
    for (auto link : in_links()) {
      if (auto memory = dynamic_cast<DataNode*>(link->source())) {
        if (memory->linearPadding())
          padded = true;
      }
    }
    return padded;
  }

  void dumpIdentifier(ostream& os) override {
    os << "[" + std::to_string(localId_) + ",\"input vector port\"" + "]";
  }

  void dumpFeatures(ostream& os) override {
    os << "{\n";
    // ID
    os << "\"id\" : " << id() << ",\n";
    // # Port
    os << "\"port\" : " << port_ << ",\n";
    // NodeType
    os << "\"nodeType\" : "
       << "\"vector port\""
       << ",\n";
    // data width
    os << "\"data_width\" : " << datawidth() << ",\n";
    // granularity
    os << "\"granularity\" : " << granularity() << ",\n";
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

  int bitwidth_capability() override {
    int result = 0;
    for (int i = 0; i < (int) out_links().size(); ++i) {
      // If this vectorport is stated, then the first link should be reserved
      // for the stated edge and not count toward capability
      if (i == 0 && vp_stated())
        continue;
      result += out_links()[i]->bitwidth();
    }
    return result;
  }

  virtual ~ssivport(){};

  bool isHanger() override { return out_links().empty();}

 protected:

  // Repeative Input Vector Port (Input Vector Port only)
  // Whether this input vector port is able to issue the vector in periodic way
  bool repeatIVP_{false};

  // Broadcast Input Vector Port (Input Vector Port only)
  // Whether this input vector port is able to copy the stream response for the other port
  bool broadcastIVP_{false};

 public:
  DEF_ATTR(repeatIVP);     // IVP
  DEF_ATTR(broadcastIVP);  // IVP
};

class ssovport : public SyncNode {
 public:
  ssnode* copy() override {
    auto res = new ssovport();
    *res = *this;
    return res;
  }

  ssovport()
      : SyncNode(NodeType::OutputVectorPort) {}

  void Accept(adg::Visitor* vistor) override;
  
  std::string name() const override {
    std::stringstream ss;
    ss << "O";
    /*
    if (port_ != -1) {
      ss << "P" << port_ << "_";
    }*/
    ss << localId_;
    return ss.str();
  }

  // Check the direction of vector port, whether or not is input vector port
  bool isInputPort() override { return false; }

  // Check whether it is output vector port
  bool isOutputPort() override {return true;}
  
  int busWidth() override {
    int busWidth = 0; 
    for (auto link : out_links()) {
      if (auto memory = dynamic_cast<DataNode*>(link->sink())) {
        busWidth = std::max(busWidth, memory->readWidth());
      }
    }
    busWidth = std::min(busWidth, (int) (bitwidth_capability() / 8));
    busWidth = std::min(busWidth, 8);
    return busWidth; 
  }

  bool padded() override {
    bool padded = false;
    for (auto link : out_links()) {
      if (auto memory = dynamic_cast<DataNode*>(link->sink())) {
        if (memory->linearPadding())
          padded = true;
      }
    }
    return padded;
  }

  void dumpIdentifier(ostream& os) override {
    os << "[" + std::to_string(localId_) + ",\"vector port\"" + "]";
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

  int bitwidth_capability() override {
    int result = 0;
    for (int i = 0; i < (int) in_links().size(); ++i) {
      // If this vectorport is stated, then the first link should be reserved
      // for the stated edge and not count toward capability
      if (i == 0 && vp_stated())
        continue;
      result += in_links()[i]->bitwidth();
    }
    return result;
  }

  virtual ~ssovport(){};

  bool isHanger() override { return in_links().empty(); }

 protected:
  // Taskflow Output Vector Port (Output Vector Port only)
  bool taskOVP_{false};

  // Discardable Output Vector Port (Output Vector Port only)
  bool discardOVP_{false};

 public:
  DEF_ATTR(taskOVP);       // OVP
  DEF_ATTR(discardOVP);    // OVP
};

class ssscratchpad : public DataNode {
 public:

  ssscratchpad()
      : DataNode(NodeType::Scratchpad) {}

  void Accept(adg::Visitor* visitor) override;

  ssnode* copy() override {
    auto *res = new ssscratchpad();
    *res = *this;
    return res;
  }

  virtual std::string name() const override {
    std::stringstream ss;
    ss << "SPM" << localId_;
    return ss.str();
  }

  void dumpIdentifier(ostream& os) override {
    os << "[" + std::to_string(localId_) + ",\" scratchpad \"" + "]";
  }

  void dumpFeatures(ostream& os) override {
    os << "{\n";
    // ID
    os << "\"id\" : " << id() << ",\n";
    // NodeType
    os << "\"nodeType\" : "
       << "\"scratchpad\""
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

  virtual ~ssscratchpad() {}

  bool is_hanger() override { 
    return false;
  }

  bool indirect() {
    return indirectLength1DStream() || indirectIndexStream() || indirectStride2DStream();
  }

}; 

class ssdma : public DataNode {
 public:
  ssnode* copy() override {
    auto res = new ssdma();
    *res = *this;
    return res;
  }

  ssdma()
      : DataNode(NodeType::DirectMemoryAccess) {}

  void Accept(adg::Visitor* vistor) override;

  std::string name() const override {
    std::stringstream ss;
    ss << "DMA" << localId_;
    return ss.str();
  }

  void dumpIdentifier(ostream& os) override {
    os << "[" + std::to_string(localId_) + ",\" dma \"" + "]";
  }

  void dumpFeatures(ostream& os) override {
    os << "{\n";
    // ID
    os << "\"id\" : " << id() << ",\n";
    // NodeType
    os << "\"nodeType\" : "
       << "\"dma\""
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

  virtual ~ssdma() {}

  bool is_hanger() override { 
    return false;
  }

};

class ssrecurrence : public DataNode {
 public:
  ssnode* copy() override {
    auto res = new ssrecurrence();
    *res = *this;
    return res;
  }

  ssrecurrence()
      : DataNode(NodeType::Recurrance) {}

  void Accept(adg::Visitor* vistor) override;

  std::string name() const override {
    std::stringstream ss;
    ss << "REC" << localId_;
    return ss.str();
  }

  void dumpIdentifier(ostream& os) override {
    os << "[" + std::to_string(localId_) + ",\" recurrence \"" + "]";
  }

  void dumpFeatures(ostream& os) override {
    os << "{\n";
    // ID
    os << "\"id\" : " << id() << ",\n";
    // NodeType
    os << "\"nodeType\" : "
       << "\"recurrence\""
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

  virtual ~ssrecurrence() {}
  bool is_hanger() override { 
    return false;
  }
};

class ssgenerate : public DataNode {
 public:
  ssnode* copy() override {
    auto res = new ssgenerate();
    *res = *this;
    return res;
  }

  ssgenerate()
      : DataNode(NodeType::Generate) {}

  void Accept(adg::Visitor* vistor) override;

  std::string name() const override {
    std::stringstream ss;
    ss << "GEN" << localId_;
    return ss.str();
  }

  void dumpIdentifier(ostream& os) override {
    os << "[" + std::to_string(localId_) + ",\" generate \"" + "]";
  }

  void dumpFeatures(ostream& os) override {
    os << "{\n";
    // ID
    os << "\"id\" : " << id() << ",\n";
    // NodeType
    os << "\"nodeType\" : "
       << "\"generate\""
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

  virtual ~ssgenerate() {}

  bool is_hanger() override { 
    return false;
  }
};


class ssregister : public DataNode {
 public:
  ssnode* copy() override {
    auto res = new ssregister();
    *res = *this;
    return res;
  }

  ssregister()
      : DataNode(NodeType::Register) {}

  void Accept(adg::Visitor* vistor) override;

  std::string name() const override {
    std::stringstream ss;
    ss << "REG" << localId_;
    return ss.str();
  }

  void dumpIdentifier(ostream& os) override {
    os << "[" + std::to_string(localId_) + ",\" register \"" + "]";
  }

  void dumpFeatures(ostream& os) override {
    os << "{\n";
    // ID
    os << "\"id\" : " << id() << ",\n";
    // NodeType
    os << "\"nodeType\" : "
       << "\"register\""
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

  virtual ~ssregister() {}

  bool is_hanger() override { 
    return false;
  }
};

}  // namespace dsa

#undef DEF_ATTR
