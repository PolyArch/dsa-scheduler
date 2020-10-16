#include <cstring>

#include "dsa/debug.h"
#include "dsa/arch/estimation.h"
#include "dsa/arch/visitor.h"

namespace dsa {
namespace adg {
namespace estimation {

const double DECOMP_COEF = 1.1;
const double CTRL_COEF[2] = {1.0, 1.7};
const double RADIX_COEF[(int) Metric::Total] = { 0.006, 24 };
const double FIFO_COEF[(int) Metric::Total] = { 0.2, 291 };

const std::map<std::pair<int, int>, std::pair<double, double>> MEMORY_DATA = {
  {{4096, 1}, {34125, 2.84}},
  {{4096, 2}, {94423, 3.81}},
  {{4096, 3}, {181106, 4.97}},
  {{4096, 4}, {296494, 6.34}},
  {{8192, 1}, {39643, 5.33}},
  {{8192, 2}, {105418, 6.63}},
  {{8192, 3}, {199273, 8.12}},
  {{8192, 4}, {323736, 9.82}},
  {{16384, 1}, {50650, 10.31}},
  {{16384, 2}, {127253, 12.24}},
  {{16384, 3}, {235226, 14.34}},
  {{16384, 4}, {377642, 16.73}},
  {{32768, 1}, {72545, 20.16}},
  {{32768, 2}, {170684, 23.42}},
  {{32768, 3}, {306771, 26.87}},
  {{32768, 4}, {486511, 30.53}},
};

double DecomposeCoef(int x) {
  assert(x == (x & -x));
  if (x == 1 || x == 2) {
    x -= 1;
  } else if (x == 4) {
    x = 2;
  } else if (x == 8) {
    x = 3;
  } else {
    assert(false && "unsupported yet!");
  }
  //std::cout << x << ": " << pow(1.1, x) << std::endl;
  return pow(DECOMP_COEF, x);
}

double RadixEst(Metric metric, int in, int out, int decompose, bool ctrl) {
  return in * out * DecomposeCoef(decompose) * CTRL_COEF[ctrl] * RADIX_COEF[(int) metric];
}

double FIFOEst(Metric metric, int depth) {
  return FIFO_COEF[(int) metric] * depth;
}

struct Estimator : Visitor {
  Estimator(SSModel *arch_) : arch(arch_) {
    auto iter = MEMORY_DATA.find({arch->memory_size, arch->io_ports});
    CHECK(iter != MEMORY_DATA.end());
    // TODO(@were): Figure out each portion of the constants.
    res(Metric::Area, Breakdown::Memory) += iter->second.first + (arch->indirect() == 2) * 88800 + 5200;
    res(Metric::Power, Breakdown::Memory) += iter->second.second + (arch->indirect() == 2) * 18.1 + 9.3;
  }

  void Visit(ssswitch *sw) {
    for (int i = 0; i < 2; ++i) {
      res((Metric) i, Breakdown::Network) += RadixEst((Metric) i, sw->in_links().size(),
                                                      sw->out_links().size(),
                                                      sw->decomposer, sw->flow_control());
      res((Metric) i, Breakdown::Sync) += FIFOEst((Metric) i, sw->delay_fifo_depth());
    }
  }

  void Visit(ssvport *vp) {
    for (int i = 0; i < 2; ++i) {
      res((Metric) i, Breakdown::Network) += RadixEst((Metric) i,
                                                      std::max((int) 1, (int) vp->in_links().size()),
                                                      std::max((int) 1, (int) vp->out_links().size()),
                                                      vp->decomposer, false);
      res((Metric) i, Breakdown::Sync) += FIFOEst((Metric) i, 2);
    }
  }

  void Visit(ssfu *fu) {
    res(Metric::Area, Breakdown::FU) += fu->fu_type_.area();
    res(Metric::Power, Breakdown::FU) += fu->fu_type_.power();
    for (int i = 0; i < 2; ++i) {
      res((Metric) i, Breakdown::Network) += RadixEst((Metric) i, 2, fu->in_links().size(),
                                                      fu->decomposer, fu->flow_control());
      res((Metric) i, Breakdown::Network) += RadixEst((Metric) i, 1, fu->out_links().size(),
                                                      fu->decomposer, fu->flow_control());
      res((Metric) i, Breakdown::Sync) += FIFOEst((Metric) i, fu->delay_fifo_depth());
    }
  }

  SSModel *arch;
  Result res;
};

Result EstimatePowerAera(SSModel *arch) {
  Estimator estimator(arch);
  arch->subModel()->Apply(&estimator);
  return estimator.res;
}

Result::Result() {
  memset(result, 0, sizeof result);
}

double &Result::operator()(Metric x, Breakdown y) {
  return result[(int) x][(int) y];
}

const char *BRKD_NAME[] = {
  "FU",
  "Network",
  "Sync",
  "Memory"
};

void Result::Dump(std::ostream &os) {
  for (int i = 0; i < 4; ++i) {
    os << BRKD_NAME[i] << ": " << operator()(Metric::Area, (Breakdown) i) << "um2 " 
                       << operator()(Metric::Power, (Breakdown) i) << "mw" << std::endl;
  }
}

}
}
}
