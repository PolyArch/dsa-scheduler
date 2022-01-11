#pragma once

#include <climits>

#include "dsa/dfg/metadata.h"
#include "dsa/dfg/node.h"

class SSDfg;

namespace dsa {
namespace dfg {

class VectorPort;

struct CompileMeta : MetaPort {
  VectorPort *parent, *destination;
  CompileMeta(const MetaPort&, VectorPort*);
  CompileMeta(){};
};

/*!
 * \brief The base class of an input/output in DFG.
 */
class VectorPort : public Node {
 public:
  friend class SSDfg;

  VectorPort() {}

  VectorPort(V_TYPE v, int len, int bitwidth_, const std::string& name, SSDfg* ssdfg,
             const MetaPort& meta);

  virtual void Accept(Visitor*) override;

  /*!
   * \brief The vector lanes of this i/o.
   */
  virtual int vectorLanes() = 0;

  /*! \brief If this port is an indirect port. */
  bool indirect() { return _indirect; }

  /*! \brief The text format name of this port. */
  std::string name() override { return _name; }

  /*! \brief The scalar bitwidth. */
  int bitwidth() override { return bitwidth_; }

  /*! \brief The bandwidth of this port. */
  int bandwidth() { return is_temporal() ? bitwidth() : bitwidth() * vectorLanes(); }

 protected:
  int bitwidth_{INT_MAX / 8};  // element bitwidth
  bool _indirect=false;

 public:
  CompileMeta meta;
};

class InputPort : public VectorPort {
 public:
  bool stated;

  static std::string Suffix() { return ""; }
  static bool IsInput() { return true; }

  static const int KindValue = V_INPUT;

  void Accept(Visitor*) final;

  InputPort() {}

  InputPort(int len, int width, const std::string& name, SSDfg* ssdfg, const MetaPort& meta,
            bool stated);

  /*!
   * \brief The vector lanes of this i/o.
   */
  int vectorLanes() override { return values.size() - stated; }

  // @{
  // Simulation stuff.
  int current_{0};
  void forward() override;
  bool can_push();
  // @}
};

class OutputPort : public VectorPort {
 public:
  /*!
   * \brief If this port should penetrate the state.
   */
  int penetrated_state{-1};

  static std::string Suffix() { return "_out"; }
  static bool IsInput() { return false; }

  static const int KindValue = V_OUTPUT;

  OutputPort() {}

  OutputPort(int len, int width, const std::string& name, SSDfg* ssdfg,
             const MetaPort& meta, int sid)
      : VectorPort(V_OUTPUT, len, width, name, ssdfg, meta), penetrated_state(sid) {}

  void Accept(Visitor*) override;

  /*!
   * \brief The vector lanes of this i/o.
   */
  int vectorLanes() override { return _ops.size() - (penetrated_state != -1); }

  virtual int slot_for_op(Edge* edge, int node_slot) override;

  // @{
  // Simulation stuff.
  void forward() override {}
  bool can_pop();
  void pop(std::vector<uint64_t>& data, std::vector<bool>& data_valid);
  // @}
};

}  // namespace dfg
}  // namespace dsa
