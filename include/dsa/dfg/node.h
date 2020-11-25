#pragma once

#include <queue>

#include "dsa/debug.h"
#include "dsa/dfg/metadata.h"
#include "dsa/simulation/data.h"

class SSDfg;
class SSDfgNode;
class SSDfgValue;

namespace dsa {
namespace dfg {

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
   * \param bitwidth The bitwidth of this value.
   */
  Value(SSDfg* parent, int nid, int index) : parent(parent), nid(nid), index(index) {}

  /*!
   * \brief The helper function translates the node id to node pointer.
   *        We adopt this design for the purpose of shallow copy.
   */
  SSDfgNode* node() const;

  // @{
  std::queue<dsa::simulation::Data> fifo;
  void push(uint64_t value, bool valid, int delay);
  bool forward(bool attempt);
  // @}

  /*! \brief The text representative of this node. */
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
};

struct Operand {
  /*! \brief Connect this operand to an list of edges and concatenate their values. */
  Operand(SSDfg* parent, const std::vector<int>& es, OperandType type);

  /*! \brief The constant constructor. */
  Operand(uint64_t);

  /*! \brief If this operand is a constant. */
  bool is_imm();

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
  bool valid();

  std::vector<std::queue<simulation::Data>> fifos;

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
  SSDfgNode* def() const;
  /*! \brief Helper function to get the source value. */
  Value* val() const;
  /*! \brief Helper function to get the consumer node. */
  SSDfgNode* use() const;
  /*! \brief Helper function to get either source or consumer. */
  SSDfgNode* get(int) const;
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

}  // namespace dfg
}  // namespace dsa