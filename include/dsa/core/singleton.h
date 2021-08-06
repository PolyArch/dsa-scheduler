#pragma once

#include <cstdint>

#include "cxxopts.hpp"

namespace dsa {

struct ContextFlags {
  /*!
   * \brief If this execution instance is verbose on logs.
   */
  bool verbose{false};
  /*!
   * \brief The max number of iterations of the instruction.
   */
  int max_iters{20000};
  /*!
   * \brief The max time cutoff of scheduling.
   */
  int timeout{86400};
  /*!
   * \brief If we want to dump the binary.
   */
  bool bitstream{false};
  /*!
   * \brief If we only want to schedule the ports to emulate the simulation.
   */
  bool dummy{false};
  /*!
   * \brief If true, do not throw an error when there are unused values in the DFG.
   */
  bool tolerate_unuse{false};

  ContextFlags();

  static ContextFlags &Global();

  void Load(const cxxopts::ParseResult &pr);

};

}
