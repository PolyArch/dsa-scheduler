#include "dsa/dfg/visitor.h"

#include "dsa/dfg/instruction.h"
#include "dsa/dfg/array.h"
#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {

void Visitor::Visit(Node* node) {}
void Visitor::Visit(Instruction* node) { Visit(static_cast<Node*>(node)); }
void Visitor::Visit(Operation* node) { Visit(static_cast<Node*>(node)); }
void Visitor::Visit(Array* node) { Visit(static_cast<Node*>(node)); }
void Visitor::Visit(DMA* node) { Visit(static_cast<Array*>(node)); }
void Visitor::Visit(Scratchpad* node) { Visit(static_cast<Array*>(node)); }
void Visitor::Visit(Recurrance* node) { Visit(static_cast<Array*>(node)); }
void Visitor::Visit(Register* node) { Visit(static_cast<Array*>(node)); }
void Visitor::Visit(Generate* node) { Visit(static_cast<Array*>(node)); }
void Visitor::Visit(VectorPort* node) { Visit(static_cast<Node*>(node)); }
void Visitor::Visit(InputPort* node) { Visit(static_cast<VectorPort*>(node)); }
void Visitor::Visit(OutputPort* node) { Visit(static_cast<VectorPort*>(node)); }

#define DEFINE_VISITOR(TYPE) \
  void TYPE::Accept(Visitor* visitor) { visitor->Visit(this); }

DEFINE_VISITOR(Instruction)
DEFINE_VISITOR(Operation)
DEFINE_VISITOR(VectorPort)
DEFINE_VISITOR(InputPort)
DEFINE_VISITOR(OutputPort)
DEFINE_VISITOR(Array)
DEFINE_VISITOR(DMA)
DEFINE_VISITOR(Scratchpad)
DEFINE_VISITOR(Recurrance)
DEFINE_VISITOR(Register)
DEFINE_VISITOR(Generate)
DEFINE_VISITOR(Node)

#undef DEFINE_VISITOR

}  // namespace dfg
}  // namespace dsa
