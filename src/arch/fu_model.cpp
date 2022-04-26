#include "dsa/arch/fu_model.h"

#include <cassert>
#include <sstream>

#include "../utils/model_parsing.h"
#include "../utils/pe_utils.h"
#include "dsa/arch/ssinst.h"
#include "dsa/debug.h"

using namespace std;

namespace dsa {

// Power
double pa_impl(const Capability& cap, std::function<double(OpCode)> f) {
  double res = 0.0;
  std::map<std::pair<adg::pe_utils::Operation, bool>, double> sumup;
  for (const auto& elem : cap.capability) {
    auto info = adg::pe_utils::InstInfo(elem.op);
    std::pair<adg::pe_utils::Operation, bool> key{info.opcode, info.is_float};
    auto func = adg::pe_utils::COMBINER.find(key);
    if (func != adg::pe_utils::COMBINER.end()) {
      sumup[key] = func->second(sumup[key], std::max(0.0, f(elem.op)));
    } else {
      sumup[key] += std::max(0.0, f(elem.op));
    }
  }
  for (auto& elem : sumup) {
    DSA_LOG(PA) << (int)elem.first.first << ", " << elem.first.second << ", " << elem.second;
    res += elem.second;
  }
  return res;
}

// Area Model
double Capability::area() { return pa_impl(*this, inst_area); }

double Capability::power() { return pa_impl(*this, inst_power); }

double Capability::TotalLut() { return pa_impl(*this, inst_total_lut); }

double Capability::LogicLut() { return pa_impl(*this, inst_logic_lut); }

double Capability::RamLut() { return pa_impl(*this, inst_ram_lut); }

double Capability::SRL() { return pa_impl(*this, inst_srl); }

double Capability::FlipFlop() { return pa_impl(*this, inst_flip_flop); }

double Capability::RamB36() { return pa_impl(*this, inst_ram_b36); }

double Capability::RamB18() { return pa_impl(*this, inst_ram_b18); }

double Capability::URam() { return pa_impl(*this, inst_u_ram); }

double Capability::DSP() { return pa_impl(*this, inst_dsp); }

}  // namespace dsa