#pragma once

#include <cstdint>

#include "cxxopts.hpp"

#include "dsa/arch/estimation.h"

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
   * \brief The max time cutoff of design space exploration.
   */
  int dse_timeout{-1};
  /*!
  * \brief The number of workers on the schedule function.
  */
  int num_schedule_workers{1};
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
  /*!
   * \brief The probability of routing along.
   */
  double route_along{0.0};
  /*!
   * \brief The target of design space exploration.
   */
  adg::estimation::Hardware dse_target{adg::estimation::Hardware::ASIC};
  /*!
   * \brief The budget of design space exploration.
   */
  adg::estimation::Resource *budget{nullptr};

  ContextFlags();

  static ContextFlags &Global();

  void Load(const cxxopts::ParseResult &pr);

};

}
