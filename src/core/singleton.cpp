#include <cstdint>

#include "dsa/debug.h"
#include "dsa/core/singleton.h"

namespace dsa {

ContextFlags::ContextFlags() {
  DSA_LOG(CF) << "am i constructed?";
}

ContextFlags &ContextFlags::Global() {
  static ContextFlags *instance_ = new ContextFlags();
  return *instance_;
}

void ContextFlags::Load(const cxxopts::ParseResult &parsed) {
  this->dummy = parsed.count("dummy");
  this->verbose = parsed.count("verbose");
  this->timeout = parsed["timeout"].as<int>();
  this->dse_timeout = parsed["dse-timeout"].as<int>();
  this->num_schedule_workers = parsed["sched-workers"].as<int>();
  this->bitstream = parsed["print-bitstream"].as<bool>();
  this->max_iters = parsed["max-iters"].as<int>();
  this->tolerate_unuse = parsed["tolerate-unuse"].as<bool>();
  this->route_along = parsed["route-along"].as<double>();
  using adg::estimation::Hardware;
  using adg::estimation::Result;
  this->dse_target = parsed.count("fpga") ? Hardware::FPGA : Hardware::ASIC;
  this->budget = Result::RESOURCE_CONSTRUCTOR[dse_target]();
  if (dse_target == Hardware::FPGA) {
    auto *fpga = dynamic_cast<adg::estimation::FPGAResource*>(budget);
    fpga->total_lut = 1182240;
    fpga->ff = 2364480;
  }

}
}
