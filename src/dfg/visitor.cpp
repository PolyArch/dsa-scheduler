#include "dsa/dfg/visitor.h"
#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {

void Visitor::Visit(SSDfgNode *node) {}
void Visitor::Visit(SSDfgInst *node) { Visit(static_cast<SSDfgNode*>(node)); }
void Visitor::Visit(SSDfgVec *node) { Visit(static_cast<SSDfgNode*>(node)); }
void Visitor::Visit(SSDfgVecInput *node) { Visit(static_cast<SSDfgVec*>(node)); }
void Visitor::Visit(SSDfgVecOutput *node) { Visit(static_cast<SSDfgVec*>(node)); }

}
}

#define DEFINE_VISITOR(TYPE)                      \
  void TYPE::Accept(dsa::dfg::Visitor *visitor) { \
    visitor->Visit(this);                         \
  }

DEFINE_VISITOR(SSDfgNode)
DEFINE_VISITOR(SSDfgInst)
DEFINE_VISITOR(SSDfgVec)
DEFINE_VISITOR(SSDfgVecInput)
DEFINE_VISITOR(SSDfgVecOutput)