#pragma once

#include "dsa/arch/sub_model.h"

namespace dsa {
namespace adg {

/*! \brief A linear vistor for ADG nodes. */
// TODO(@were): I am not sure if we need to have a unifyed visitor for both sw and hw.
struct Visitor {
  virtual void Visit(ssnode* node);
  virtual void Visit(ssfu* node);
  virtual void Visit(ssswitch* node);
  virtual void Visit(ssivport* node);
  virtual void Visit(ssovport* node);
  virtual void Visit(ssdma* node);
  virtual void Visit(ssscratchpad* node);
  virtual void Visit(ssrecurrence* node);
  virtual void Visit(ssgenerate* node);
  virtual void Visit(ssregister* node);
};

/*! \brief A vistor for ADG nodes, which traverses all the nodes by DFS. */
struct GraphVisitor : Visitor {
  std::vector<bool> visited;
  GraphVisitor(SpatialFabric* fabric);
  virtual void Visit(ssfu* node);
  virtual void Visit(ssswitch* node);
  virtual void Visit(ssivport* node);
  virtual void Visit(ssovport* node);
  virtual void Visit(ssdma* node);
  virtual void Visit(ssscratchpad* node);
  virtual void Visit(ssrecurrence* node);
  virtual void Visit(ssgenerate* node);
  virtual void Visit(ssregister* node);

 private:
  void Visit(ssnode* node);
};

}  // namespace adg
}  // namespace dsa
