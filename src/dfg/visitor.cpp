#include "dsa/dfg/visitor.h"

#include "dsa/dfg/instruction.h"
#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {

void Visitor::Visit(SSDfgNode* node) {}
void Visitor::Visit(Instruction* node) { Visit(static_cast<SSDfgNode*>(node)); }
void Visitor::Visit(VectorPort* node) { Visit(static_cast<SSDfgNode*>(node)); }
void Visitor::Visit(InputPort* node) { Visit(static_cast<VectorPort*>(node)); }
void Visitor::Visit(OutputPort* node) { Visit(static_cast<VectorPort*>(node)); }

#define DEFINE_VISITOR(TYPE) \
  void TYPE::Accept(Visitor* visitor) { visitor->Visit(this); }

DEFINE_VISITOR(Instruction)
DEFINE_VISITOR(VectorPort)
DEFINE_VISITOR(InputPort)
DEFINE_VISITOR(OutputPort)

#undef DEFINE_VISITOR

}  // namespace dfg
}  // namespace dsa

#define DEFINE_VISITOR(TYPE) \
  void TYPE::Accept(dsa::dfg::Visitor* visitor) { visitor->Visit(this); }

DEFINE_VISITOR(SSDfgNode)