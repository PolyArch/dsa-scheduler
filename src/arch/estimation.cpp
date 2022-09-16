#include "dsa/arch/estimation.h"

#include <cstring>
#include <iomanip>
#include <algorithm>
#include <vector>

#include "dsa/core/singleton.h"
#include "dsa/arch/model.h"
#include "../mapper/pass/resource_estimator.h"
#include "dsa/arch/visitor.h"
#include "dsa/arch/predict.h"
#include "dsa/debug.h"

namespace dsa {
namespace adg {
namespace estimation {

#if defined(__clang__) || defined (__GNUC__)
# define ATTRIBUTE_NO_SANITIZE_ADDRESS __attribute__((no_sanitize_address))
#else
# define ATTRIBUTE_NO_SANITIZE_ADDRESS
#endif

void ASICResource::normalize() {
  auto resource = ContextFlags::Global().budget;
  if (auto ar = dynamic_cast<ASICResource*>(resource))
    return;
  DSA_CHECK(false);
  return;
}

void FPGAResource::normalize() {
  auto budget = ContextFlags::Global().budget;
  if (auto fpga_budget = dynamic_cast<FPGAResource*>(budget)) {
    total_lut = total_lut / fpga_budget->total_lut;
    logic_lut = logic_lut / fpga_budget->logic_lut;
    ram_lut = ram_lut / fpga_budget->ram_lut;
    srl = srl/ fpga_budget->srl;
    ff = ff / fpga_budget->ff;
    ramb36 = ramb36 / fpga_budget->ramb36;
    ramb18 = ramb18 / fpga_budget->ramb18;
    uram = uram / fpga_budget->uram;
    dsp =  dsp / fpga_budget->dsp;
    return;
  }
  DSA_CHECK(false);
  return;
}

FPGAResource::FPGAResource(const std::vector<double> &v) {
  DSA_CHECK(v.size() == 9);
  total_lut = v[0];
  logic_lut = v[1];
  ram_lut = v[2];
  srl = v[3];
  ff = v[4];
  ramb36 = v[5];
  ramb18 = v[6];
  uram = v[7];
  dsp = v[8];
}

Resource* FPGAResource::clone() const {
  FPGAResource* r = new FPGAResource();
  r->total_lut = total_lut;
  r->logic_lut = logic_lut;
  r->ram_lut = ram_lut;
  r->srl = srl;
  r->ff = ff;
  r->ramb36 = ramb36;
  r->ramb18 = ramb18;
  r->uram = uram;
  r->dsp = dsp;

  return r;
}

Resource* ASICResource::clone() const {
  ASICResource* r = new ASICResource();
  r->power = power;
  r->area = area;

  return r;
}

void FPGAResource::scale_cores(int numcores) {
  total_lut = total_lut * numcores;
  logic_lut = logic_lut * numcores;
  ram_lut = ram_lut * numcores;
  srl = srl * numcores;
  ff = ff * numcores;
  ramb36 = ramb36 * numcores;
  ramb18 = ramb18 * numcores;
  uram = uram * numcores;
  dsp = dsp * numcores;
}

void ASICResource::scale_cores(int numcores) {
  power = power * numcores;
  area = area * numcores;
}

std::vector<double> ASICResource::to_vector() {
  return {power, area};
}

std::vector<double> FPGAResource::to_vector() {
  return {total_lut, logic_lut, ram_lut, srl, ff, ramb36, ramb18, uram, dsp};
}

// gets the nth largest resource
double FPGAResource::constrained_resource(int n) {
  auto budget = ContextFlags::Global().budget;
  if (auto fpga_budget = dynamic_cast<FPGAResource*>(budget)) {
    std::vector<double> resources = {
      logic_lut, ram_lut, srl, ff, ramb36, ramb18, uram, dsp
    };
    if (resources.size() <= n)
      return 0;
    std::sort(resources.begin(), resources.end(), std::greater<double>());
    return resources[n];
  }
  DSA_CHECK(false);
  return 0;
}

// Returns Area
double ASICResource::constrained_resource(int n) {
  auto resource = ContextFlags::Global().budget;
  if (auto ar = dynamic_cast<ASICResource*>(resource)) {
    if (std::abs(ar->area) < 1e-6) {
      return area;
    }
    return area < ar->area ? area / ar->area : -1;
  }
  DSA_CHECK(false);
  return 0;
}

std::string ASICResource::constrained_resource_name(int n) {
  auto resource = ContextFlags::Global().budget;
  if (auto ar = dynamic_cast<ASICResource*>(resource)) {
    return "area";
  }
  DSA_CHECK(false);
  return "";
}

// gets the name of the nth largest resource
std::string FPGAResource::constrained_resource_name(int n) {
  std::vector<double> resources = {
    logic_lut, ram_lut, srl, ff, ramb36, ramb18, uram, dsp
  };
  DSA_CHECK(resources.size() > n);
  std::vector<int> indices(resources.size());
  std::vector<std::string> names = {
    "total_lut", "logic lut", "ram_lut", "srl", "ff", "ramb32", "ramb18", "uram", "dsp"
  };
  std::sort(indices.begin(), indices.end(), [&](int lhs, int rhs) {
      return resources[lhs] < resources[rhs];
  });
  std::vector<std::string> res(indices.size());
  for (std::size_t i = 0; i != indices.size(); ++i) {
      res[indices[i]] = names[i];
  }
  return res[n];
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
    << ", srl: " << srl
    << ", ram lut: " << ram_lut
    << ", ff: " << ff
    << ", ramb36: " << ramb36
    << ", ramb18: " << ramb18
    << ", uram: " << uram
    << ", dsp: " << dsp;

  return oss.str();
}
/*
const double DECOMP_COEF = 1.1;
const double CTRL_COEF[2] = {1.0, 1.7};
const double RADIX_COEF[(int)Metric::Total] = {0.006, 24};
const double FIFO_COEF[(int)Metric::Total] = {0.2, 291};

const std::map<std::pair<int, int>, std::pair<double, double>> MEMORY_DATA = {
    {{4096, 1}, {34125, 2.84}},    {{4096, 2}, {94423, 3.81}},
    {{4096, 3}, {181106, 4.97}},   {{4096, 4}, {296494, 6.34}},
    {{8192, 1}, {39643, 5.33}},    {{8192, 2}, {105418, 6.63}},
    {{8192, 3}, {199273, 8.12}},   {{8192, 4}, {363736, 9.82}},
    {{16384, 1}, {50650, 10.31}},  {{16384, 2}, {127253, 12.24}},
    {{16384, 3}, {235226, 14.34}}, {{16384, 4}, {377642, 16.73}},
    {{36768, 1}, {72545, 20.16}},  {{36768, 2}, {170684, 23.42}},
    {{32768, 3}, {306771, 26.87}}, {{32768, 4}, {486511, 30.53}},
};

double DecomposeCoef(int x) {
  DSA_CHECK(x == (x & -x)) << "Only power of 2 supported!";
  if (x == 1 || x == 2) {
    x -= 1;
  } else if (x == 4) {
    x = 2;
  } else if (x == 8) {
    x = 3;
  } else {
    DSA_CHECK(false) << "Unsupported yet!";
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
  Estimator(SSModel* arch_, Hardware hw_) : arch(arch_), hw(hw_){
    switch (hw) {
      case Hardware::ASIC: {
        auto iter = MEMORY_DATA.find({arch->memory_size, arch->io_ports});
        DSA_CHECK(iter != MEMORY_DATA.end());
        double power = iter->second.second + (arch->indirect() == 2) * 18.1 + 9.3;
        double area = iter->second.first + (arch->indirect() == 2) * 88800 + 5200;
        res.add(Breakdown::Memory, power, area);
        break;
      }
      case Hardware::FPGA: {
        // TODO(@were): Include this!
        break;
      }
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
        std::vector<float> model_inputs {
          (float) sw->in_links().size(),
          (float) sw->out_links().size(),
          (float) sw->delay_fifo_depth(),
          (float) !sw->flow_control()
        };
        
        auto predicted = switch_area_predict_fpga(model_inputs);
        std::vector<double> output(predicted.begin(), predicted.end());
        res.add(Breakdown::Network, output);
        break;
      }
    }
  }
  void Visit(ssovport* vp) {
    switch (hw) {
      case Hardware::FPGA: {
        float model_input = (float)vp->in_links().size();
        auto predicted = output_vport_area_predict_fpga(model_input);
        std::vector<double> output(predicted.begin(), predicted.end());
        res.add(Breakdown::Sync, output);
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

  void Visit(ssivport* vp) {
    switch (hw) {
      case Hardware::FPGA: {
        float model_input = (float) vp->out_links().size();
        auto predicted = input_vport_area_predict_fpga(model_input);
        std::vector<double> output(predicted.begin(), predicted.end());
        res.add(Breakdown::Sync, output);
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
        Input: [Inputs, Outputs, OutputBuffer Depth, Static, Fifo Depth, Registers]
        Output: [TotalLUTs, LogicLUTs, LUTRAMs, FFs]
        std::vector<float> model_inputs{
          (float) fu->in_links().size(),
          (float) fu->out_links().size(),
          0,
          (float) fu->flow_control() ? 1.0f : 0.0f,
          (float) fu->delay_fifo_depth(),
          (float) fu->regFileSize(),
          (float) fu->ctrlLUTSize() > 0 ? 1.0f : 0.0f
        };
        auto predicted = pe_area_predict_fpga(model_inputs);

        std::vector<double> output(predicted.begin(), predicted.end());
        
        //Total Lut
        output[0] += fu->fu_type().LogicLut() * 2;
        // Logic Lut
        output[1] += fu->fu_type().LogicLut();
        // Lut Ram
        // TODO: Make more pernament. Just a constraint to make the DSE work.
        output[2] += fu->fu_type().LogicLut();

        // Flip Flop
        output[4] += fu->fu_type().FlipFlop();

        res.add(Breakdown::FU, output);
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
        std::pair<double, double> fupa(fu->fu_type().power(), fu->fu_type().area());
        res.add(Breakdown::FU, fupa.first, fupa.second);
        if (fu->max_util() != 1) {
          res.add(Breakdown::FU, 14.1, 16581.0);
        }
        break;
      }
    }
  }

  SSModel* arch;
  Result res;
  Hardware hw;
};
*/

ATTRIBUTE_NO_SANITIZE_ADDRESS
Result EstimatePowerAera(SSModel* arch) {
  ResourceEstimator est(arch, ContextFlags::Global().dse_target);
  //Estimator estimator(arch, ContextFlags::Global().dse_target);
  arch->subModel()->Apply(&est);
  return est.res;
}


const char* BRKD_NAME[] = {"FU", "Switch", "IVPort", "OVPort", "Scratchpad", "DMA", "Recurrance", "Generate", "Register", "Core", "System Bus"};

void Result::Dump(std::ostream& os) {
  for (int i = 0; i < (int) Breakdown::Total; ++i) {
    os << BRKD_NAME[i] << ": " << brkd[i]->dump() << std::endl;
  }
}

void Result::Dump_all_resources(std::ostream& os) {
  os << this->sum()->dump();
}

void Result::scale_cores(int numcores) {
  for (int i = 0; i < (int) Breakdown::Total; ++i) {
    brkd[i]->scale_cores(numcores);
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

Result::~Result() {
  for (int i = 0; i < (int) brkd.size(); ++i) {
    delete brkd[i];
  }
}

Result::Result(const Result& other) {
  brkd.resize((int) Breakdown::Total, nullptr);
  for (int i = 0; i < (int) Breakdown::Total; ++i) {
    brkd[i] = other.brkd[i]->clone();
  }
}

void Result::add(Breakdown k, double power, double area) {
  auto asic = dynamic_cast<ASICResource*>(brkd[(int) k]);
  DSA_CHECK(asic);
  asic->power += power;
  asic->area += area;
}

void Result::add(Breakdown k, const std::vector<double> &v) {
  auto fpga = dynamic_cast<FPGAResource*>(brkd[(int) k]);
  if (!fpga) {
    return;
  }
  DSA_CHECK(fpga) << "FPGA resource expected for " << BRKD_NAME[(int) k];
  fpga->total_lut += v[0];
  fpga->logic_lut += v[1];
  fpga->ram_lut += v[2];
  fpga->srl += v[3];
  fpga->ff += v[4];
  fpga->ramb36 += v[5];
  fpga->ramb18 += v[6];
  fpga->uram += v[7];
  fpga->dsp += v[8];
}

void Result::add_core_overhead() {
  auto core = ContextFlags::Global().core_resources;
  if (auto fpga_core = dynamic_cast<FPGAResource*>(core)) {
    std::vector<double> core_resources = core->to_vector();
    this->add(Breakdown::Core, core_resources);
  } else {
    std::vector<double> core_resources = core->to_vector();
    this->add(Breakdown::Core, core_resources);
  }
}

void Result::add_system_bus_overhead(int num_cores, int banks, int system_bus_width) {
  auto core = ContextFlags::Global().core_resources;
  if (auto fpga_core = dynamic_cast<FPGAResource*>(core)) {
    auto output = system_bus_area_predict_fpga({(double) num_cores, (double) banks, (double) system_bus_width});
    this->add(Breakdown::System_Bus, output);
  } else {
    // Do nothing
  }
}

void Result::add_dma_overhead(int links, int system_bus_width) {
  auto core = ContextFlags::Global().core_resources;
  if (auto fpga_core = dynamic_cast<FPGAResource*>(core)) {
    auto output = dma_area_predict_fpga({(double) links, (double) system_bus_width});
    this->add(Breakdown::DMA, output);
  } else {
    // Do nothing
  }
}


Resource *Result::resource_bd(int breakdown) {
  return brkd[breakdown];
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
        // total_lut, logic_lut, ram_lut, srl, ff, ramb32, ramb18, uram, dsp;
        res->total_lut += fpga->total_lut;
        res->logic_lut += fpga->logic_lut;
        res->ram_lut += fpga->ram_lut;
        res->srl += fpga->srl;
        res->ff += fpga->ff;
        res->ramb36 += fpga->ramb36;
        res->ramb18 += fpga->ramb18;
        res->uram += fpga->uram;
        res->dsp += fpga->dsp;
      }
      return res;
    }
  }
  DSA_CHECK(false);
  return nullptr;
}

}  // namespace estimation
}  // namespace adg
}  // namespace dsa
