#include "dsa/arch/estimation.h"

#include <cstring>
#include <iomanip>
#include <vector>

#include "dsa/core/singleton.h"
#include "dsa/arch/visitor.h"
#include "dsa/arch/predict.h"
#include "dsa/debug.h"

namespace dsa {
namespace adg {
namespace estimation {


double ASICResource::normalize() {
  auto resource = ContextFlags::Global().budget;
  if (auto ar = dynamic_cast<ASICResource*>(resource)) {
    if (std::abs(ar->area) < 1e-6) {
      return area;
    }
    return area < ar->area ? area / ar->area : -1;
  }
  CHECK(false);
  return 0;
}

double FPGAResource::normalize() {
  auto resource = ContextFlags::Global().budget;
  if (auto fr = dynamic_cast<FPGAResource*>(resource)) {
    auto res = std::max(total_lut / fr->total_lut, ff / fr->ff);
    return res < 1 ? res : -1;
  }
  CHECK(false);
  return 0;
}

std::string ASICResource::dump() {
  std::ostringstream oss;
  oss << "power: " << power << ", area: " << area;
  return oss.str();
}

std::string FPGAResource::dump() {
  std::ostringstream oss;
  oss << "total lut: " << total_lut
    << ", logic lut: " << logic_lut
    << ", ram lut: " << ram_lut
    << ", ff: " << ff;
  return oss.str();
}

const double DECOMP_COEF = 1.1;
const double CTRL_COEF[2] = {1.0, 1.7};
const double RADIX_COEF[(int)Metric::Total] = {0.006, 24};
const double FIFO_COEF[(int)Metric::Total] = {0.2, 291};

const std::map<std::pair<int, int>, std::pair<double, double>> MEMORY_DATA = {
    {{4096, 1}, {34125, 2.84}},    {{4096, 2}, {94423, 3.81}},
    {{4096, 3}, {181106, 4.97}},   {{4096, 4}, {296494, 6.34}},
    {{8192, 1}, {39643, 5.33}},    {{8192, 2}, {105418, 6.63}},
    {{8192, 3}, {199273, 8.12}},   {{8192, 4}, {323736, 9.82}},
    {{16384, 1}, {50650, 10.31}},  {{16384, 2}, {127253, 12.24}},
    {{16384, 3}, {235226, 14.34}}, {{16384, 4}, {377642, 16.73}},
    {{32768, 1}, {72545, 20.16}},  {{32768, 2}, {170684, 23.42}},
    {{32768, 3}, {306771, 26.87}}, {{32768, 4}, {486511, 30.53}},
};

double DecomposeCoef(int x) {
  CHECK(x == (x & -x)) << "Only power of 2 supported!";
  if (x == 1 || x == 2) {
    x -= 1;
  } else if (x == 4) {
    x = 2;
  } else if (x == 8) {
    x = 3;
  } else {
    CHECK(false) << "Unsupported yet!";
  }
  // std::cout << x << ": " << pow(1.1, x) << std::endl;
  return pow(DECOMP_COEF, x);
}

std::pair<double, double> RadixEst(int in, int out, int decompose, bool ctrl) {
  double coef = in * out * DecomposeCoef(decompose) * CTRL_COEF[ctrl];
  return {coef * RADIX_COEF[(int) Metric::Power], coef * RADIX_COEF[(int) Metric::Area]};
}

std::pair<double, double> FIFOEst(int depth) {
  return {FIFO_COEF[(int) Metric::Power] * depth, FIFO_COEF[(int) Metric::Area] * depth};
}

struct Estimator : Visitor {
  Estimator(SSModel* arch_) : arch(arch_) {
    auto iter = MEMORY_DATA.find({arch->memory_size, arch->io_ports});
    CHECK(iter != MEMORY_DATA.end());
    double power = iter->second.second + (arch->indirect() == 2) * 18.1 + 9.3;
    double area = iter->second.first + (arch->indirect() == 2) * 88800 + 5200;
    switch (hw) {
    case Hardware::ASIC:
      res.add(Breakdown::Memory, power, area);
      break;
    case Hardware::FPGA:
      // TODO(@were): Include this!
      break;
    }
  }

  void Visit(ssswitch* sw) {
    switch (hw) {
    case Hardware::ASIC: {
      auto nw = RadixEst(sw->in_links().size(), sw->out_links().size(),
                         sw->lanes(), sw->flow_control());
      res.add(Breakdown::Network, nw.first, nw.second);
      auto sync = FIFOEst(sw->delay_fifo_depth());
      res.add(Breakdown::Sync, sync.first, sync.second);
      break;
    }
    case Hardware::FPGA: {
      std::vector<float> model_inputs{
        (float) sw->max_delay(), 
        (float) sw->lanes(), 
        (float) sw->in_links().size(), 
        (float) sw->max_util() > 1 ? (float) 1 : (float) 0, 
        (float) sw->out_links().size(), 
        (float) sw->flow_control() ? (float) 1 : (float) 0};
      auto of = router_area_predict_fpga(model_inputs);
      std::vector<double> od{(double) of[0], (double) of[1], (double) of[2], (double) of[3]};
      res.add(Breakdown::Network, od);
    }
      break;
    }
  }

  void Visit(ssvport* vp) {
    switch (hw) {
    case Hardware::FPGA: {
      std::vector<float> model_inputs{
        (float) vp->in_links().size(), 
        (float) vp->out_links().size(), 
        (float) vp->datawidth(), 
        (float) vp->delay_fifo_depth() };
      auto of = vport_area_predict_fpga(model_inputs);
      std::vector<double> od{(double) of[0], (double) of[1], (double) of[2], (double) of[3]};
      res.add(Breakdown::Sync, od);
      break;
    }
    case Hardware::ASIC: {
      auto nw =
          RadixEst(std::max((int)1, (int)vp->in_links().size()),
                   std::max((int)1, (int)vp->out_links().size()), vp->lanes(), false);
      res.add(Breakdown::Network, nw.first, nw.second);
      auto sync = FIFOEst(2);
      res.add(Breakdown::Sync, sync.first, sync.second);
      break;
    }
    }

  }

  void Visit(ssfu* fu) {
    switch (hw) {
    case Hardware::FPGA: {
      std::vector<float> model_inputs{
        (float) fu->lanes(),
        (float) fu->max_delay(), 
        (float) fu->in_links().size(),
        (float) fu->max_util() > 1 ? (float) 1 : (float) 0,
        (float) fu->out_links().size(), 
        (float) 0 /*features[4]*/, 
        (float) fu->flow_control() ? (float) 1 : (float) 0, 
        (float) fu->delay_fifo_depth()};
      auto of = pe_area_predict_fpga(model_inputs);
      std::vector<double> od{(double) of[0], (double) of[1], (double) of[2], (double) of[3]};
      res.add(Breakdown::FU, od);
      break;
    }
    case Hardware::ASIC: {
      auto nw_in =
        RadixEst(2, fu->in_links().size(), fu->lanes(), fu->flow_control());
      auto nw_out =
        RadixEst(1, fu->out_links().size(), fu->lanes(), fu->flow_control());
      auto sync = FIFOEst(fu->delay_fifo_depth());
      res.add(Breakdown::Network, nw_in.first, nw_in.second);
      res.add(Breakdown::Network, nw_out.first, nw_out.second);
      res.add(Breakdown::Sync, sync.first, sync.second);
      std::pair<double, double> fupa(fu->fu_type_.power(), fu->fu_type_.area());
      res.add(Breakdown::FU, fupa.first, fupa.second);
      break;
    }
    }
  }

  SSModel* arch;
  Result res;
  Hardware hw;
};

Result EstimatePowerAera(SSModel* arch) {
  Estimator estimator(arch);
  estimator.hw = ContextFlags::Global().dse_target;
  arch->subModel()->Apply(&estimator);
  return estimator.res;
}


const char* BRKD_NAME[] = {"FU", "Network", "Sync", "Memory"};

void Result::Dump(std::ostream& os) {
  for (int i = 0; i < (int) Breakdown::Total; ++i) {
    os << BRKD_NAME[i] << ": " << brkd[i]->dump() << std::endl;
  }
}

std::map<Hardware, std::function<Resource*()>> Result::RESOURCE_CONSTRUCTOR = {
  {Hardware::FPGA, []() -> Resource* { return new FPGAResource(); }},
  {Hardware::ASIC, []() -> Resource* { return new ASICResource(); }},
};

Result::Result() {
  brkd.resize((int) Breakdown::Total, nullptr);
  for (int i = 0; i < (int) brkd.size(); ++i) {
    brkd[i] = RESOURCE_CONSTRUCTOR[ContextFlags::Global().dse_target]();
  }
}

void Result::add(Breakdown k, double power, double area) {
  auto asic = dynamic_cast<ASICResource*>(brkd[(int) k]);
  CHECK(asic);
  asic->power += power;
  asic->area += area;
}

void Result::add(Breakdown k, const std::vector<double> &v) {
  auto fpga = dynamic_cast<FPGAResource*>(brkd[(int) k]);
  CHECK(fpga);
  fpga->total_lut += v[0];
  fpga->logic_lut += v[1];
  fpga->ram_lut += v[2];
  fpga->ff += v[3];
}

Resource *Result::sum() {
  switch (ContextFlags::Global().dse_target) {
  case Hardware::ASIC: {
    ASICResource *res = new ASICResource();
    for (int i = 0; i < (int) brkd.size(); ++i) {
      auto *asic = dynamic_cast<ASICResource*>(brkd[i]);
      res->power += asic->power;
      res->area += asic->area;
    }
    res->area /= 1e6;
    return res;
  }
  case Hardware::FPGA: {
    FPGAResource *res = new FPGAResource();
    for (int i = 0; i < (int) this->brkd.size(); ++i) {
      auto *fpga = dynamic_cast<FPGAResource*>(brkd[i]);
      res->total_lut += fpga->total_lut;
      res->logic_lut += fpga->logic_lut;
      res->ram_lut += fpga->ram_lut;
      res->ff += fpga->ff;
    }
    return res;
  }
  }
  CHECK(false);
  return nullptr;
}

}  // namespace estimation
}  // namespace adg
}  // namespace dsa
