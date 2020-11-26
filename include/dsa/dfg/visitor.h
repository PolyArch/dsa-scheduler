#pragma once

class SSDfgNode;

namespace dsa {
namespace dfg {

class Instruction;
class VectorPort;
class InputPort;
class OutputPort;

struct Visitor {
  virtual void Visit(SSDfgNode*);
  virtual void Visit(Instruction*);
  virtual void Visit(VectorPort*);
  virtual void Visit(InputPort*);
  virtual void Visit(OutputPort*);
};

}  // namespace dfg
}  // namespace dsa