#pragma once

#include <climits>

#include "dsa/dfg/metadata.h"
#include "dsa/dfg/node.h"

class SSDfg;

namespace dsa {
namespace dfg {

class Array;



/*!
 * \brief The base class of an input/output in DFG.
 */
class Array : public Node {
 public:
  friend class SSDfg;

  Array() {}

  Array(int size, const std::string& name,
                       SSDfg* ssdfg);

  virtual void Accept(Visitor*) override;

  /*!
   * \brief The consumption rate of this array.
   */
  std::vector<double> Consumption(bool checkInput, bool repeat=false, bool reuse=false);

  /*! \brief The text format name of this array. */
  std::string name() override { return _name; }

  /*! \brief The scalar bitwidth. */
  int bitwidth() override { return bitwidth_; }
  
  virtual int vectorLanes() = 0;

  /*! \brief The size of this array. */
  int size() { return size_; }

 protected:
  int bitwidth_{INT_MAX / 8};  // element bitwidth
  int size_{INT_MAX};
};

class DMA : public Array {
 public:
  DMA() {}

  DMA(int size, const std::string& name, SSDfg* ssdfg);

  void Accept(Visitor*) override;

  int vectorLanes() override { return 1; }

  bool recurrant();

  double reuse();
  
  // @{
  // Simulation stuff.
  int current_{0};
  void forward() override {};
  // @}
};

class Scratchpad : public Array {
 public:
  Scratchpad() {}

  Scratchpad(int size, const std::string& name, SSDfg* ssdfg);

  void Accept(Visitor*) override;

  int vectorLanes() override { return 1; }

  // @{
  // Simulation stuff.
  int current_{0};
  void forward() override {};
  // @}

};


class Recurrance : public Array {
 public:
  Recurrance() {}

  Recurrance(int size, const std::string& name, SSDfg* ssdfg);

  void Accept(Visitor*) override;

  int vectorLanes() override { return 1; }

  // @{
  // Simulation stuff.
  int current_{0};
  void forward() override {};
  // @}

};

class Register : public Array {
 public:
  Register() {}

  Register(int size, const std::string& name, SSDfg* ssdfg);

  void Accept(Visitor*) override;

  int vectorLanes() override { return 1; }

  // @{
  // Simulation stuff.
  int current_{0};
  void forward() override {};
  // @}

};

class Generate : public Array {
 public:
  Generate() {}

  Generate(int size, const std::string& name, SSDfg* ssdfg);

  void Accept(Visitor*) override;

  int vectorLanes() override { return 1; }

  // @{
  // Simulation stuff.
  int current_{0};
  void forward() override {};
  // @}

};

}  // namespace dfg
}  // namespace dsa
