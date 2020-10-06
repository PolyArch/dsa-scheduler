#pragma once

class SSDfgNode;
class SSDfgInst;
class SSDfgVec;
class SSDfgVecInput;
class SSDfgVecOutput;

namespace dsa {
namespace dfg {

struct Visitor {
  virtual void Visit(SSDfgNode *);
  virtual void Visit(SSDfgInst *);
  virtual void Visit(SSDfgVec *);
  virtual void Visit(SSDfgVecInput *);
  virtual void Visit(SSDfgVecOutput *);
};

}
}