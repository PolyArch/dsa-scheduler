#include "dsa/arch/visitor.h"

#include "dsa/arch/fabric.h"

namespace dsa {
namespace adg {

GraphVisitor::GraphVisitor(SpatialFabric* fabric)
    : visited(fabric->node_list().size(), false) {}

void Visitor::Visit(ssnode* node) {}
void Visitor::Visit(ssfu* node) { Visit(static_cast<ssnode*>(node)); }
void Visitor::Visit(ssvport* node) { Visit(static_cast<ssnode*>(node)); }
void Visitor::Visit(ssswitch* node) { Visit(static_cast<ssnode*>(node)); }

void GraphVisitor::Visit(ssnode* node) {
  visited[node->id()] = true;
  for (auto elem : node->out_links()) {
    if (!visited[elem->sink()->id()]) {
      elem->sink()->Accept(this);
    }
  }
}
void GraphVisitor::Visit(ssfu* node) { Visit(static_cast<ssnode*>(node)); }
void GraphVisitor::Visit(ssvport* node) { Visit(static_cast<ssnode*>(node)); }
void GraphVisitor::Visit(ssswitch* node) { Visit(static_cast<ssnode*>(node)); }

}  // namespace adg

#define DEFINE_ACCEPT(TYPE) \
  void TYPE::Accept(adg::Visitor* vistor) { vistor->Visit(this); }

DEFINE_ACCEPT(ssswitch);
DEFINE_ACCEPT(ssvport);
DEFINE_ACCEPT(ssfu);

}  // namespace dsa