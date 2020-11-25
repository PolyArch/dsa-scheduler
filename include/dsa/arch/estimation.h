#pragma once

#include <iostream>

#include "dsa/arch/model.h"

namespace dsa {
namespace adg {
namespace estimation {

enum class Metric { Power = 0, Area = 1, Total = 2 };
enum class Breakdown { FU = 0, Network = 1, Sync = 2, Memory = 3, Total = 4 };

/*! \brief The result of power/area estimation */
struct Result {
  Result();
  double& operator()(Metric x, Breakdown);

  /*! \brief Print the breakdowns */
  void Dump(std::ostream&);

  /*! \brief Sum up the breakdowns */
  template <Metric metric>
  double Total() {
    double ret(0);
    for (int i = 0; i < (int)Breakdown::Total; ++i) {
      ret += operator()(metric, Breakdown(i));
    }
    return ret;
  }

 private:
  double result[(int)Metric::Total][(int)Breakdown::Total];
};

/*! \brief Estimating the power/area breakdown of the given hardware */
Result EstimatePowerAera(SSModel*);

}  // namespace estimation
}  // namespace adg
}  // namespace dsa