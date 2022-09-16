#include <cstdint>

#include "cxxopts.hpp"

#include "dsa/debug.h"
#include "dsa/core/singleton.h"

#define TORCH_MODEL_PREFIX REPO_PREFIX "/estimation-models/"
namespace dsa {

ContextFlags::ContextFlags() {
  DSA_LOG(CF) << "am i constructed?";
}

ContextFlags &ContextFlags::Global() {
  static ContextFlags *instance_ = new ContextFlags();
  return *instance_;
}

EnvCache &ContextFlags::CachedEnv() {
  static EnvCache *instance_ = new EnvCache();
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
  this->core_resources = Result::RESOURCE_CONSTRUCTOR[dse_target]();
  this->adg_compat = parsed["compat-adg"].as<bool>();
  if (dse_target == Hardware::FPGA) {
    auto *budget = dynamic_cast<adg::estimation::FPGAResource*>(this->budget);
    auto *core_resources = dynamic_cast<adg::estimation::FPGAResource*>(this->core_resources);
    budget->total_lut = 1182240;
    core_resources->total_lut = 9446;
    
    budget->logic_lut = 1123206;
    core_resources->logic_lut = 7894;
    
    budget->ram_lut = 59034;
    core_resources->ram_lut = 1552;
    
    budget->srl = 592813;
    core_resources->srl = 0;
    
    budget->ff = 2364480;
    core_resources->ff = 3140;
    
    budget->ramb36 = 2160;
    core_resources->ramb36 = 0;
    
    budget->ramb18 = 4322;
    core_resources->ramb18 = 0;
    
    budget->uram = 960;
    core_resources->uram = 0;

    budget->dsp = 6840;
    core_resources->dsp = 0;
  }
  this->bitstream = parsed["print-bitstream"].as<bool>();
  this->max_iters = parsed["max-iters"].as<int>();
  this->tolerate_unuse = parsed["tolerate-unuse"].as<bool>();

}

} // Namespace DSA
