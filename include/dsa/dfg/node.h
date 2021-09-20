#pragma once

#include <queue>

#include "dsa/debug.h"
#include "dsa/dfg/metadata.h"
#include "dsa/dfg/visitor.h"
#include "dsa/simulation/data.h"

class SSDfg;

namespace dsa {
namespace dfg {

class Node;

OperandType Str2Flag(const std::string& s);

/*! \brief The data structure of the values produced by a node. */
struct Value {
  /*!  \brief The default constructor. */
  Value() {}

  /*!
   * \brief Construct a value instance.
   * \param parent The DFG it belongs to.
   * \param nid The IR node it belongs to.
   * \brief The value index in the node to which this value belongs.
   */
  Value(SSDfg* parent, int nid, int index) : parent(parent), nid(nid), index(index) {}

  /*!
   * \brief The helper function translates the node id to node pointer.
   *        We adopt this design for the purpose of shallow copy.
   */
  Node* node() const;

  // @{
  std::queue<dsa::sim::SpatialPacket> fifo;
  void push(uint64_t value, bool valid, int delay);
  bool forward(bool attempt);
  // @}

  /*! \brief The text representative of this node for logging. */
  std::string name();

  // TODO(@were): Do we want to over engineer it to make all these const?
  /*! \brief The DFG this value belongs to. */
  SSDfg* parent;
  /*! \brief The IR node this value belongs to. */
  int nid{-1};
  /*! \brief The value index in the node to which this value belongs. */
  int index{0};
  /*! \brief The id's of the out going edges. */
  std::vector<int> uses;
  /*! \brief The register index this value should be written to. If no, -1. */
  int reg{-1};
  /*! \brief Symbol name in DFG format. */
  std::string symbol;
};

struct Operand {
  /*! \brief Connect this operand to an list of edges and concatenate their values. */
  Operand(SSDfg* parent, const std::vector<int>& es, OperandType type);

  /*! \brief Construct a register operand. */
  Operand(SSDfg* parent, int dtype, int idx);

  /*! \brief The constant constructor. */
  Operand(SSDfg* parent, uint64_t);

  /*! \brief Constructor for imm or register. */
  Operand(SSDfg* parent, OperandType type, uint64_t);

  /*! \brief If this operand is a constant. */
  bool isImm();

  /*! \brief If this operand is a register. */
  bool isReg();

  int regIdx() { 
    CHECK(isReg());
    return imm & (~((uint32_t) 0));
  }

  int regDtype() {
    CHECK(isReg());
    return (imm >> 32) & (~((uint32_t) 0));
  }

  /*! \brief The DFG this edge belongs to. */
  SSDfg* parent{nullptr};
  /*! \brief The edges from upstream values. */
  std::vector<int> edges;
  /*! \brief The constant operand. */
  uint64_t imm = 0;
  /*! \brief The type of this operand. */
  OperandType type;

  // TODO(@were): Move these to simulation.
  // @{
  // bool valid();

  std::vector<std::queue<sim::SpatialPacket>> fifos;

  bool ready();

  uint64_t poll();

  bool predicate();

  void pop();
  // @}
};

struct Edge {
  /*! \brief A default constructor for serialized load. */
  Edge() {}

  /*!
   * \brief Construct a new edge object
   * \param sid The id of the source node.
   * \param vid The value index of the source node.
   * \param uid The id of the consumer node.
   * \param parent The DFG this belongs to.
   * \param l The left slicing.
   * \param r The right slicing.
   */
  Edge(SSDfg* parent, int sid, int vid, int uid, int l = 0, int r = 63);

  /*! \brief Helper function to get the source node. */
  Node* def() const;
  /*! \brief Helper function to get the source value. */
  Value* val() const;
  /*! \brief Helper function to get the consumer node. */
  Node* use() const;
  /*! \brief Helper function to get either source or consumer. */
  Node* get(int) const;
  /*! \brief For debug. The text representative of this edge. */
  std::string name() const;
  /*! \brief The bitwidth of this data path. */
  int bitwidth() { return r - l + 1; }
  
  /*! \brief The index of the edge in the parent's edge list. */
  int id{-1};
  /*! \brief The DFG this edge belongs to. */
  SSDfg* parent{nullptr};
  /*! \brief The node produces the source value. */
  int sid{-1}, vid{-1};
  /*! \brief The consumer of this edge. */
  int uid{-1};
  /*! \brief The slicing applied on the source value. */
  int l{-1}, r{-1};
  /*! \brief Size of the FIFO buffer. */
  int buf_len{9};
  /*! \brief The delay of the FIFO buffer. */
  int delay{0};
};

/*! \brief The abstract class for all DFG nodes. */
class Node {
 public:
  virtual ~Node() {}

  virtual void Accept(dsa::dfg::Visitor*);

  Node() {}

  enum V_TYPE { V_INVALID, V_INPUT, V_OUTPUT, V_INST, V_NUM_TYPES };

  virtual int slot_for_use(dsa::dfg::Edge* edge, int node_slot);

  virtual int slot_for_op(dsa::dfg::Edge* edge, int node_slot) { return node_slot; }

  Node(SSDfg* ssdfg, V_TYPE v, const std::string& name = "");

  virtual int lat_of_inst() { return 0; }

  virtual std::string name() = 0;  // pure func

  dsa::dfg::Edge* getLinkTowards(Node* to);

  bool has_name() { return !_name.empty(); }

  void set_name(std::string name) { _name = name; }

  std::vector<dsa::dfg::Operand>& ops() { return _ops; }

  int id() { return _ID; }

  virtual void forward() = 0;

  //--------------------------------------------

  bool is_temporal();

  virtual int bitwidth() = 0;
  //---------------------------------------------------------------------------

  V_TYPE type() {
    CHECK(_vtype != V_INVALID);
    return _vtype;
  }

  int group_id() { return _group_id; }

  void set_group_id(int id) { _group_id = id; }

  SSDfg*& ssdfg() { return _ssdfg; }

  /*! \brief The values produced by this node. */
  std::vector<dsa::dfg::Value> values;

 protected:
  SSDfg* _ssdfg = 0;  // sometimes this is just nice to have : )

  // Static Stuff
  int _ID;
  std::string _name;
  std::vector<dsa::dfg::Operand> _ops;  // in edges

  int _min_lat = 0;
  int _max_thr = 0;
  int _group_id = 0;  // which group do I belong to

  V_TYPE _vtype;
};

}  // namespace dfg
}  // namespace dsa
