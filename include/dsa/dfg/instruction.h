#pragma once

#include <map>

#include "dsa/dfg/node.h"
#include "dsa/dfg/visitor.h"

class SSDfg;

namespace dsa {
namespace dfg {

/*!
 * \brief The control predication LUT. In the MICRO architecture,
 *        it is a \lfloor 64 / Total \rfloor-LUT, each the result
 *        is a Total-bit vector.
 */
struct CtrlBits {
  /*! \brief The predicated behaviors. */
  enum Control {
    B1,       // Backpressure on the first operand FIFO.
    B2,       // Backpressure on the second operand FIFO.
    Discard,  // Predication off the produced value.
    Reset,    // Reset the register file to all zeros.
    Abstain,  // Avoid instruction execution.
    Write,    // Write to the accumulate register
    Total     // Placeholder for the last behavior. Used by declaration.
  };

  /*!
   * \brief The behavior of predicated execution.
   */
  struct Behavior {
    /*! \brief Back pressure the operand. */
    std::vector<bool> backpressure;
    /*! \brief Discard the output. */
    bool discard{false};
    /*! \brief Reset the register file. */
    bool reset{false};
    /*! \brief Execute the instruction. */
    bool exec{true};
    /*! \brief Write the result to register. */
    bool write{false};

    Behavior(int n) : backpressure(n, false) {}
  };

  /*!
   * \brief Construct a new CtrlBits with a parsed raw map.
   * \param raw The parsed raw map from the control list.
   */
  CtrlBits(const std::map<int, std::vector<std::string>>& raw);

  /*!
   * \brief Construct a new CtrlBits object with an existing LUT buffer.
   * \param mask_ The LUT buffer.
   */
  CtrlBits(uint64_t mask_) : mask(mask_) {}

  /*! \brief Construct an empty CtrlBits object. */
  CtrlBits() : mask(0) {}

  void set(uint64_t val, Control b);
  bool test(uint64_t val, Control b);
  void test(uint64_t val, Behavior &b);
  CtrlBits& operator=(const CtrlBits& b) {
    mask = b.mask;
    const_cast<bool&>(is_dynamic) = b.is_dynamic;
    return *this;
  }

  uint64_t bits() { return mask; }

  bool needs_ctrl_dep() { return is_dynamic; }

  const bool is_dynamic{false};

 private:
  uint64_t mask{0};

  static Control str_to_enum(const std::string& s) {
    if (s == "b1") return B1;
    if (s == "b2") return B2;
    if (s == "d") return Discard;
    if (s == "r") return Reset;
    if (s == "a") return Abstain;
    if (s == "w") return Write;
    CHECK(false) << "Not a valid command";
    abort();
  }
};

/*! \brief IR node for the instructions in the DFG. */
class Instruction : public Node {
 public:
  static const int KindValue = V_INST;

  /*! \brief The entrance function for the visitor pattern. */
  void Accept(dsa::dfg::Visitor*) final;

  /*! \brief The default constructor. */
  Instruction() {}

  /*!
   * \brief The constructor with instruction opcode.
   * \param ssdfg The DFG this instruciton belongs to.
   * \param inst The instruction opcode.
   */
  Instruction(SSDfg* ssdfg, dsa::OpCode inst = dsa::SS_NONE);

  /*!
   * \brief The latency of the instruction execution.
   * \return The latency of the instruction execution.
   */
  int lat_of_inst() override { return inst_lat(inst()); }

  /*!
   * \brief The instruction opcode.
   * \return The instruction opcode.
   */
  dsa::OpCode inst() { return opcode; }

  /*! \brief The name of this instruction. */
  // TODO(@were): Do we want to rename this to ToString?
  std::string name() override;

  /*! \brief The predication affected by an upstream operand. */
  CtrlBits predicate;
  /*! \brief The predication affected by itself. */
  CtrlBits self_predicate;

  int bitwidth() override;

  // TODO(@were): Move these to simulation.
  // @{
  int last_execution{-1};
  void forward() override;
  uint64_t do_compute(bool& discard, std::vector<bool> &backpressure);
  // @}
 private:
  // TODO(@were): These are data structures for simulation. Move them out later.
  std::vector<uint64_t> _input_vals;
  std::vector<uint64_t> _output_vals;
  std::vector<uint64_t> _reg;

  dsa::OpCode opcode{dsa::OpCode::SS_NONE};
};

/*!
 * \brief IR node for the instructions occupies multiple compute resources.
 */
class Operation : public Node {
 public:
  static const int KindValue = V_INST;

  /*! \brief The entrance function for the visitor pattern. */
  void Accept(dsa::dfg::Visitor*) final;

  /*! \brief The default constructor. */
  Operation() {}

  /*!
   * \brief Construct a new Operation object.
   * \param ssdfg The parent dfg.
   * \param ops The compute resources required.
   * \param cnt The number of each compute resource.
   */
  Operation(SSDfg* ssdfg, const std::vector<OpCode>& ops_, const std::vector<int>& cnt_)
      : Node(ssdfg, V_INST), opcodes(ops_), cnt(cnt_) {}

  /*! \brief The name of this instruction. */
  // TODO(@were): Do we want to rename this to ToString?
  std::string name() override;

  // TODO(@were): Do we want any dynamic timing or predications?
  // TODO(@were): Do we want any decomposability?

  int bitwidth() override;

  /*! \brief Since it is no longer a single instruction, we allow users to customize the
   * latency. */
  int latency{-1};

  /*! \brief The compute resources required. */
  std::vector<OpCode> opcodes;

  /*! \brief The number of compute resources required. */
  std::vector<int> cnt;

  void forward() override { CHECK(false) << "Operation node is for scheduling only!"; }
};

}  // namespace dfg
}  // namespace dsa
