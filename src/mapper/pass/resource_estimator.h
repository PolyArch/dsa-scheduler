#pragma once

#include <torch/script.h>

#include "dsa/arch/estimation.h"
#include "dsa/arch/model.h"
#include "dsa/arch/predict.h"
#include "dsa/arch/visitor.h"
#include "dsa/core/singleton.h"
#include "dsa/debug.h"

#define TORCH_MODEL_PREFIX REPO_PREFIX "/estimation-models/"

namespace dsa {
namespace adg {
namespace estimation {

struct WrappedModule {
  torch::jit::script::Module m;

  WrappedModule(const std::string& path) {
    try {
      m = torch::jit::load(path);
    } catch (const c10::Error& e) {
      DSA_CHECK(false) << "Error Loading FPGA Model: " << path;
    }
  }
};

#define DEFINE_WRAPPED_MODEL(fn, mn)          \
  WrappedModule* fn() {                       \
    static auto* res = new WrappedModule(mn); \
    return res;                               \
  }

DEFINE_WRAPPED_MODEL(switch_model, TORCH_MODEL_PREFIX "switch/switch_model.pt");
DEFINE_WRAPPED_MODEL(pe_model,
                     TORCH_MODEL_PREFIX "processing_element/processing_element_model.pt");
DEFINE_WRAPPED_MODEL(ovp_model,
                     TORCH_MODEL_PREFIX "output_vector_port/output_vector_port_model.pt");
DEFINE_WRAPPED_MODEL(ivp_model,
                     TORCH_MODEL_PREFIX "input_vector_port/input_vector_port_model.pt");

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
  return {coef * RADIX_COEF[(int)Metric::Power], coef * RADIX_COEF[(int)Metric::Area]};
}

std::pair<double, double> FIFOEst(int depth) {
  return {FIFO_COEF[(int)Metric::Power] * depth, FIFO_COEF[(int)Metric::Area] * depth};
}

struct ResourceEstimator : Visitor {
  ResourceEstimator(SSModel* arch_, Hardware hw_) : arch(arch_), hw(hw_) {
    switch (hw) {
      case Hardware::ASIC: {
        auto iter = MEMORY_DATA.find({arch->memory_size, arch->io_ports});
        DSA_CHECK(iter != MEMORY_DATA.end());
        double power = iter->second.second + (arch->indirect() == 2) * 18.1 + 9.3;
        double area = iter->second.first + (arch->indirect() == 2) * 88800 + 5200;
        res.add(Breakdown::Scratchpad, power, area);
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
        auto nw = RadixEst(sw->in_links().size(), sw->out_links().size(), sw->lanes(),
                           sw->flow_control());
        res.add(Breakdown::Switch, nw.first, nw.second);
        auto sync = FIFOEst(sw->delay_fifo_depth());
        res.add(Breakdown::IVPort, sync.first, sync.second);
        break;
      }
      case Hardware::FPGA: {
        // Create Vector of NN Model Input Parameters
        std::vector<float> model_inputs{
            (float)sw->in_links().size(), 
            (float)sw->out_links().size(),
            (float)sw->delay_fifo_depth(), 
            (float)!sw->flow_control()
        };

        // Convert to Torch Tensor
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(torch::tensor(model_inputs));

        // Run NN Model
        at::Tensor switch_model_output = dsa::adg::estimation::switch_model()->m.forward(inputs).toTensor();

        // Extract Outputs
        std::vector<double> output = {
          switch_model_output[0].item<double>(), 
          switch_model_output[1].item<double>(), 
          switch_model_output[2].item<double>(), 
          0, 
          switch_model_output[3].item<double>(), 
          0, 0, 0, 0
        };

        // Add Outputs
        res.add(Breakdown::Switch, output);
        break;
      }
    }
  }
  void Visit(ssovport* vp) {
    switch (hw) {
      case Hardware::FPGA: {
        // Model Inputs
        std::vector<float> model_inputs{
            (float)vp->in_links().size(), 
            (float)vp->out_links().size(),
            (float)vp->delay_fifo_depth(),
            (float)vp->vp_stated(),
            (float)vp->discardOVP(),
            (float)vp->busWidth(),
            (float)vp->padded(),
        };
        
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(torch::tensor(model_inputs));

        // Run NN Model
        at::Tensor ovp_model_output = dsa::adg::estimation::ovp_model()->m.forward(inputs).toTensor();

        // Extract Outputs
        std::vector<double> output = {
          ovp_model_output[0].item<double>(), 
          ovp_model_output[1].item<double>(), 
          ovp_model_output[2].item<double>(), 
          0, 
          ovp_model_output[3].item<double>(), 
          0, 0, 0, 0
        };

        if (vp->busWidth() == 64) {
          output[0] = output[0] / 2;
          output[1] = output[1] / 2;
          output[2] = output[2] / 2;
          output[3] = output[3] / 2;
          output[4] = output[4] / 2;
          output[5] = output[5] / 2;
          output[6] = output[6] / 2;
          output[7] = output[7] / 2;
        }

        // Add Outputs
        res.add(Breakdown::OVPort, output);

        /*
        float model_input = (float)vp->in_links().size();
        auto predicted = output_vport_area_predict_fpga(model_input);
        std::vector<double> output(predicted.begin(), predicted.end());
        res.add(Breakdown::Sync, output);
        */
        break;
      }
      case Hardware::ASIC: {
        auto nw =
            RadixEst(std::max((int)1, (int)vp->in_links().size()),
                     std::max((int)1, (int)vp->out_links().size()), vp->lanes(), false);
        res.add(Breakdown::Switch, nw.first, nw.second);
        auto sync = FIFOEst(2);
        res.add(Breakdown::OVPort, sync.first, sync.second);
        break;
      }
    }
  }

  void Visit(ssivport* vp) {
    switch (hw) {
      case Hardware::FPGA: {
        // Model Inputs
        std::vector<float> model_inputs{
            (float)vp->in_links().size(), 
            (float)vp->out_links().size(),
            (float)vp->delay_fifo_depth(),
            (float)vp->vp_stated(),
            (float)vp->repeatIVP(),
            (float)vp->broadcastIVP(),
            (float)vp->busWidth(),
            (float)vp->padded(),
        };
        
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(torch::tensor(model_inputs));

        // Run NN Model
        at::Tensor ivp_model_output = dsa::adg::estimation::ivp_model()->m.forward(inputs).toTensor();

        // Extract Outputs
        std::vector<double> output = {
          ivp_model_output[0].item<double>(), 
          ivp_model_output[1].item<double>(), 
          ivp_model_output[2].item<double>(), 
          0, 
          ivp_model_output[3].item<double>(), 
          0, 0, 0, 0
        };

        // Add Outputs
        res.add(Breakdown::IVPort, output);
        /*
        float model_input = (float)vp->out_links().size();
        auto predicted = input_vport_area_predict_fpga(model_input);
        std::vector<double> output(predicted.begin(), predicted.end());
        res.add(Breakdown::Sync, output);
        */
        break;
      }
      case Hardware::ASIC: {
        auto nw =
            RadixEst(std::max((int)1, (int)vp->in_links().size()),
                     std::max((int)1, (int)vp->out_links().size()), vp->lanes(), false);
        res.add(Breakdown::Switch, nw.first, nw.second);
        auto sync = FIFOEst(2);
        res.add(Breakdown::IVPort, sync.first, sync.second);
        break;
      }
    }
  }

  void Visit(ssfu* fu) {
    switch (hw) {
      case Hardware::FPGA: {
        std::vector<float> model_inputs{
            (float)fu->in_links().size(),
            (float)fu->out_links().size(),
            0,
            (float)fu->flow_control() ? 1.0f : 0.0f,
            (float)fu->delay_fifo_depth(),
            (float)fu->regFileSize(),
            (float)fu->ctrlLUTSize() > 0 ? 1.0f : 0.0f
        };
    

        // Convert to Torch Tensor
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(torch::tensor(model_inputs));

        // Run NN Model
        at::Tensor pe_model_output = dsa::adg::estimation::pe_model()->m.forward(inputs).toTensor();

        // Extract Outputs
        std::vector<double> output = {
          pe_model_output[0].item<double>() + fu->fu_type().TotalLut(), 
          pe_model_output[1].item<double>() + fu->fu_type().LogicLut(), 
          pe_model_output[2].item<double>() + fu->fu_type().RamLut(), 
          fu->fu_type().SRL(), 
          pe_model_output[3].item<double>() + fu->fu_type().FlipFlop(), 
          fu->fu_type().RamB36(), fu->fu_type().RamB18(), fu->fu_type().URam(), fu->fu_type().DSP()
        };

        res.add(Breakdown::FU, output);
        break;
      }
      case Hardware::ASIC: {
        auto nw_in = RadixEst(2, fu->in_links().size(), fu->lanes(), fu->flow_control());
        auto nw_out =
            RadixEst(1, fu->out_links().size(), fu->lanes(), fu->flow_control());
        auto sync = FIFOEst(fu->delay_fifo_depth());
        res.add(Breakdown::Switch, nw_in.first, nw_in.second);
        res.add(Breakdown::Switch, nw_out.first, nw_out.second);
        res.add(Breakdown::IVPort, sync.first, sync.second);
        std::pair<double, double> fupa(fu->fu_type().power(), fu->fu_type().area());
        res.add(Breakdown::FU, fupa.first, fupa.second);
        if (fu->max_util() != 1) {
          res.add(Breakdown::FU, 14.1, 16581.0);
        }
        break;
      }
    }
  }

  void Visit(ssscratchpad* spm) {
    switch (hw) {
      case Hardware::FPGA: {
        std::vector<double> inputs = {
          (double)spm->in_links().size() + (double)spm->out_links().size(),
          (double)spm->readWidth(),
          (double)spm->capacity(),
          spm->indirect() ? 1.0 : 0.0,
        };

        auto output = spm_area_predict_fpga(inputs);

        res.add(Breakdown::Scratchpad, output);
        break;
      }
      case Hardware::ASIC: {
        // Do Nothing
        break;
      }
    }
  }

  void Visit(ssdma* dma) {
    switch (hw) {
      case Hardware::FPGA: {
        break;
        std::vector<double> input = {
          (double) dma->in_links().size() + dma->out_links().size(),
          (double) dma->readWidth(),
        };

        auto output = dma_area_predict_fpga(input);

        res.add(Breakdown::DMA, output);
        break;
      }
      case Hardware::ASIC: {
        // Do Nothing
        break;
      }
    }
  }

  void Visit(ssgenerate* gen) {
    switch (hw) {
      case Hardware::FPGA: {
        std::vector<double> baseline = {
          10023, 9965, 58, 0, 5200, 0, 0, 0, 0
        };
        std::vector<double> output = {
          (baseline[0] / 7) * gen->out_links().size(),
          (baseline[1] / 7) * gen->out_links().size(),
          (baseline[2] / 7) * gen->out_links().size(),
          (baseline[3] / 7) * gen->out_links().size(),
          (baseline[4] / 7) * gen->out_links().size(),
          (baseline[5] / 7) * gen->out_links().size(),
          (baseline[6] / 7) * gen->out_links().size(),
          (baseline[7] / 7) * gen->out_links().size(),
          (baseline[8] / 7) * gen->out_links().size()
        };

        res.add(Breakdown::Generate, output);
        break;
      }
      case Hardware::ASIC: {
        // Do Nothing
        break;
      }
    }
  }
  
  void Visit(ssrecurrence* rec) {
    switch (hw) {
      case Hardware::FPGA: {
        std::vector<double> baseline = {
          4976, 4857, 119, 0, 194, 0, 0, 0, 0
        };
        std::vector<double> output = {
          (baseline[0] / 5) * rec->in_links().size(),
          (baseline[1] / 5) * rec->in_links().size(),
          (baseline[2] / 5) * rec->in_links().size(),
          (baseline[3] / 5) * rec->in_links().size(),
          (baseline[4] / 5) * rec->in_links().size(),
          (baseline[5] / 5) * rec->in_links().size(),
          (baseline[6] / 5) * rec->in_links().size(),
          (baseline[7] / 5) * rec->in_links().size(),
          (baseline[8] / 5) * rec->in_links().size()
        };
        res.add(Breakdown::Recurrance, output);
        break;
      }
      case Hardware::ASIC: {
        // Do Nothing
        break;
      }
    }
  }

  void Visit(ssregister* reg) {
    switch (hw) {
      case Hardware::FPGA: {
        std::vector<double> baseline = {
          332, 332, 0, 0, 333, 0, 0, 0, 0
        };
        std::vector<double> output = {
          (baseline[0] / 5) * reg->in_links().size(),
          (baseline[1] / 5) * reg->in_links().size(),
          (baseline[2] / 5) * reg->in_links().size(),
          (baseline[3] / 5) * reg->in_links().size(),
          (baseline[4] / 5) * reg->in_links().size(),
          (baseline[5] / 5) * reg->in_links().size(),
          (baseline[6] / 5) * reg->in_links().size(),
          (baseline[7] / 5) * reg->in_links().size(),
          (baseline[8] / 5) * reg->in_links().size()
        };

        res.add(Breakdown::Register, output);
        break;
      }
      case Hardware::ASIC: {
        // Do Nothing
        break;
      }
    }
  }

  SSModel* arch;
  Result res;
  Hardware hw;
};
}  // namespace estimation
}  // namespace adg
}  // namespace dsa