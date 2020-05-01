#include "dsa/arch/sub_model.h"

namespace dsa {
namespace adg {

/*! \brief A linear vistor for ADG nodes. */
// TODO(@were): I am not sure if we need to have a unifyed visitor for both sw and hw.
struct Visitor {
  virtual void Visit(ssnode *node);
  virtual void Visit(ssfu *node);
  virtual void Visit(ssswitch *node);
  virtual void Visit(ssvport *node);
};

/*! \brief A vistor for ADG nodes, which traverses all the nodes by DFS. */
struct GraphVisitor : Visitor {
  std::vector<bool> visited;
  GraphVisitor(SubModel *fabric);
  virtual void Visit(ssfu *node);
  virtual void Visit(ssswitch *node);
  virtual void Visit(ssvport *node);
 private:
  void Visit(ssnode *node);
};

}
}