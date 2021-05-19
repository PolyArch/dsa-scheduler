#pragma once

#include <cstdint>

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

  ContextFlags();

  static ContextFlags &Global();
};

}