#pragma once

namespace dsa {
namespace dfg {

class Node;
class Instruction;
class Operation;
class Array;
class DMA;
class Scratchpad;
class Recurrance;
class Register;
class Generate;
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
  virtual void Visit(Array*);
  virtual void Visit(DMA*);
  virtual void Visit(Scratchpad*);
  virtual void Visit(Recurrance*);
  virtual void Visit(Register*);
  virtual void Visit(Generate*);
};

}  // namespace dfg
}  // namespace dsa