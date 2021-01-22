#pragma once

namespace dsa {
namespace dfg {

class Node;
class Instruction;
class Operation;
class VectorPort;
class InputPort;
class OutputPort;

struct Visitor {
  virtual void Visit(Node*);
  virtual void Visit(Instruction*);
  virtual void Visit(Operation*);
  virtual void Visit(VectorPort*);
  virtual void Visit(InputPort*);
  virtual void Visit(OutputPort*);
};

}  // namespace dfg
}  // namespace dsa