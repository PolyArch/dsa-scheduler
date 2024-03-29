#include "dsa/arch/predict.h"

#include <torch/script.h>
#include <iostream>

#include "dsa/core/singleton.h"
#include "dsa/debug.h"

#define TORCH_MODEL_PREFIX REPO_PREFIX "/estimation-models/"

namespace dsa {

struct WrappedModule {
  torch::jit::script::Module m;

  WrappedModule(const std::string &path) {
    try {
      m = torch::jit::load(path);
    }
    catch (const c10::Error& e) {
     DSA_CHECK(false)
       << "Error Loading FPGA Model: " << path;
   }
  }

};

#define DEFINE_WRAPPED_MODEL(fn, mn)          \
  WrappedModule *fn() {                       \
    static auto *res = new WrappedModule(mn); \
    return res;                               \
  }

DEFINE_WRAPPED_MODEL(pe_total_lut, TORCH_MODEL_PREFIX "processing_element/pe_model_total_lut.pt")
DEFINE_WRAPPED_MODEL(pe_logic_lut, TORCH_MODEL_PREFIX "processing_element/pe_model_logic_lut.pt")
DEFINE_WRAPPED_MODEL(pe_ram_lut, TORCH_MODEL_PREFIX "processing_element/pe_model_ram_lut.pt")
DEFINE_WRAPPED_MODEL(pe_flip_flop, TORCH_MODEL_PREFIX "processing_element/pe_model_ff.pt")

DEFINE_WRAPPED_MODEL(sw_total_lut, TORCH_MODEL_PREFIX "switch/switch_model_total_lut.pt")
DEFINE_WRAPPED_MODEL(sw_logic_lut, TORCH_MODEL_PREFIX "switch/switch_model_logic_lut.pt")
DEFINE_WRAPPED_MODEL(sw_ram_lut, TORCH_MODEL_PREFIX "switch/switch_model_ram_lut.pt")
DEFINE_WRAPPED_MODEL(sw_flip_flop, TORCH_MODEL_PREFIX "switch/switch_model_ff.pt")

DEFINE_WRAPPED_MODEL(switch_model, TORCH_MODEL_PREFIX "switch/switch_model.pt");
DEFINE_WRAPPED_MODEL(pe_model, TORCH_MODEL_PREFIX "processing_element/processing_element_model.pt");

}

/*

Input: [Inputs, Outputs, OutputBuffer Depth, Static, Fifo Depth, Registers, Meta]
Output: [TotalLUTs, LogicLUTs, LUTRAMs, FFs]
*/
std::vector<float> pe_area_predict_fpga(std::vector<float> parameters) {
  auto pe_model = dsa::pe_model();

  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(torch::tensor(parameters));

  at::Tensor pe_model_output = pe_model->m.forward(inputs).toTensor();
  std::vector<float> output = {
          pe_model_output[0].item<float>(), 
          pe_model_output[1].item<float>(), 
          pe_model_output[2].item<float>(), 
          0, 
          pe_model_output[3].item<float>(), 
          0, 0, 0, 0
  };
  return output;
}

std::vector<float>  switch_area_predict_fpga(const  std::vector<float> parameters) {
  auto *switch_model = dsa::switch_model();
  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(torch::tensor(parameters));

  at::Tensor switch_model_output = switch_model->m.forward(inputs).toTensor();
  std::vector<float> output = {
          switch_model_output[0].item<float>(), 
          switch_model_output[1].item<float>(), 
          switch_model_output[2].item<float>(), 
          0, 
          switch_model_output[3].item<float>(), 
          0, 0, 0, 0
  };
  return output;
}

/* Input Parameters {back_pressure_fifo_depth decomposer num_input_ports isShared num_output_ports protocol} */
/* Output {TotalLUTs LogicLUTs LUTRAMs SRLs FFs RAMB36 RAMB18 URAM DSPBlocks} */
std::vector<float>  router_area_predict_fpga(const  std::vector<float> parameters){
  auto *total_lut = dsa::sw_total_lut();
  auto *logic_lut = dsa::sw_logic_lut();
  auto *lut_ram = dsa::sw_ram_lut();
  auto *ff = dsa::sw_flip_flop();

  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(torch::tensor(parameters));

  at::Tensor total_lut_output = total_lut->m.forward(inputs).toTensor();
  at::Tensor logic_lut_output = logic_lut->m.forward(inputs).toTensor();
  at::Tensor lut_ram_output = lut_ram->m.forward(inputs).toTensor();
  at::Tensor ff_output = ff->m.forward(inputs).toTensor();

  std::vector<float> output = {
          total_lut_output[0].item<float>(), 
          logic_lut_output[0].item<float>(), 
          lut_ram_output[0].item<float>(), 
          0, 
          ff_output[0].item<float>(), 
          0, 0, 0, 0
  };

  // Temporary Hack for Switch Models until updated ML Models are available
  if (output[1] < 100) {
    output[0] = output[0] - output[1] + 100.0;
    output[1] = 100.0;
  }
  if (output[2] < 100) {
    output[0] = output[0] - output[2] + 100.0;
    output[2] = 100.0;
  }

  return output;
}

std::vector<float> input_vport_area_predict_fpga(const float parameter) {
  int next_power_of_two =  pow(2, ceil(log(parameter)/log(2)));
  switch(next_power_of_two) {
      case 1:
        return {(int) (3014 / 4), (int) (2886 / 4), (int) (128 / 4), 0, (int) (584 / 4), 0, 0, 0, 0};
      case 2:
        return {(int) (3221 / 4), (int) (3013 / 4), (int) (208 / 4), 0, (int) (461 / 4), 0, 0, 0, 0};
      case 4:
        return {(int) (5027 / 4), (int) (4739 / 4), (int) (288 / 4), 0, (int) (473 / 4), 0, 0, 0, 0};
      case 8:
        return {(int) (7028 / 4), (int) (6900 / 4), (int) (128 / 4), 0, (int) ( 1017 / 4), 0, 0, 0, 0};
      default: {
        DSA_CHECK(false) << "Input Vector Port Too big. Size: " << parameter;
        return {0, 0, 0, 0, 0, 0, 0, 0, 0};
      }
    }
}


std::vector<int> stream_table = {15730, 15730, 0, 0, 4504, 0, 0, 0, 0};
std::vector<int> AGU = {2602, 2602, 0, 0, 0, 0, 0, 0, 0};
std::vector<int> ROB = {29801, 29801, 0, 0, 1344, 4, 0, 0, 0};
std::vector<int> RequestXBar = {972, 500, 472, 0, 52, 0, 0, 0, 0};
std::vector<int> ResponseXBar = {936, 528, 408, 0, 52, 0, 0, 0, 0};
std::vector<int> RobScheduler = {246, 246, 0, 0, 0, 0, 0, 0, 0};
std::vector<int> Banks = {548, 548, 0, 0, 152, 256, 0, 0, 0};

// Parameters = [#VP, SPM Width, SPM Capacity, Indirect]
std::vector<double> spm_area_predict_fpga(const std::vector<double> parameters) {
  std::vector<double> baseline = {11, 32, 1048576, 1};

  std::vector<double> output = {
    stream_table[0] / baseline[0] * parameters[0]
    + AGU[0] / baseline[1] * parameters[1]
    + ROB[0] / baseline[1] * parameters[1] * parameters[3]
    + (RequestXBar[0] + ResponseXBar[0]) / baseline[1] * parameters[1] * parameters[3]
    + RobScheduler[0] * parameters[3]
    + Banks[0] / baseline[2] * parameters[2],
    stream_table[1] / baseline[0] * parameters[0]
    + AGU[1] / baseline[1] * parameters[1]
    + ROB[1] / baseline[1] * parameters[1] * parameters[3]
    + (RequestXBar[1] + ResponseXBar[1]) / baseline[1] * parameters[1] * parameters[3]
    + RobScheduler[1] * parameters[3]
    + Banks[1] / baseline[2] * parameters[2],
    stream_table[2] / baseline[0] * parameters[0]
    + AGU[2] / baseline[1] * parameters[1]
    + ROB[2] / baseline[1] * parameters[1] * parameters[3]
    + (RequestXBar[2] + ResponseXBar[2]) / baseline[1] * parameters[1] * parameters[3]
    + RobScheduler[2] * parameters[3]
    + Banks[2] / baseline[2] * parameters[2],
    stream_table[3] / baseline[0] * parameters[0]
    + AGU[3] / baseline[1] * parameters[1]
    + ROB[3] / baseline[1] * parameters[1] * parameters[3]
    + (RequestXBar[3] + ResponseXBar[3]) / baseline[1] * parameters[1] * parameters[3]
    + RobScheduler[3] * parameters[3]
    + Banks[3] / baseline[2] * parameters[2],
    stream_table[4] / baseline[0] * parameters[0]
    + AGU[4] / baseline[1] * parameters[1]
    + ROB[4] / baseline[1] * parameters[1] * parameters[3]
    + (RequestXBar[4] + ResponseXBar[4]) / baseline[1] * parameters[1] * parameters[3]
    + RobScheduler[4] * parameters[3]
    + Banks[4] / baseline[2] * parameters[2],
    stream_table[5] / baseline[0] * parameters[0]
    + AGU[5] / baseline[1] * parameters[1]
    + ROB[5] / baseline[1] * parameters[1] * parameters[3]
    + (RequestXBar[5] + ResponseXBar[5]) / baseline[1] * parameters[1] * parameters[3]
    + RobScheduler[5] * parameters[3]
    + Banks[5] / baseline[2] * parameters[2],
    stream_table[6] / baseline[0] * parameters[0]
    + AGU[6] / baseline[1] * parameters[1]
    + ROB[6] / baseline[1] * parameters[1] * parameters[3]
    + (RequestXBar[6] + ResponseXBar[6]) / baseline[1] * parameters[1] * parameters[3]
    + RobScheduler[6] * parameters[3]
    + Banks[6] / baseline[2] * parameters[2],
    stream_table[7] / baseline[0] * parameters[0]
    + AGU[7] / baseline[1] * parameters[1]
    + ROB[7] / baseline[1] * parameters[1] * parameters[3]
    + (RequestXBar[7] + ResponseXBar[7]) / baseline[1] * parameters[1] * parameters[3]
    + RobScheduler[7] * parameters[3]
    + Banks[7] / baseline[2] * parameters[2],
    stream_table[8] / baseline[0] * parameters[0]
    + AGU[8] / baseline[1] * parameters[1]
    + ROB[8] / baseline[1] * parameters[1] * parameters[3]
    + (RequestXBar[8] + ResponseXBar[8]) / baseline[1] * parameters[1] * parameters[3]
    + RobScheduler[8] * parameters[3]
    + Banks[8] / baseline[2] * parameters[2]
  };
  return output;
}


std::vector<int> xBar = {182, 182, 0, 0, 11, 0, 0, 0, 0};
std::vector<int> TLB = {98, 98, 0, 0, 185, 0, 0, 0, 0};
std::vector<int> DMARobScheduler = {246, 246, 0, 0, 0, 0, 0, 0, 0};
std::vector<int> DMAStreamTable = {246, 246, 0, 0, 0, 0, 0, 0, 0};
std::vector<int> DMARob = {2314, 2314, 0, 0, 848, 4, 0, 0, 0};
std::vector<int> DMAAgu = {2593, 2593, 0, 0, 0, 0, 0, 0, 0};
std::vector<int> DMAReader = {376, 376, 0, 0, 1020, 0, 0, 0, 0};
std::vector<int> DMAWriter = {65, 37, 28, 0, 149, 0, 0, 0, 0};

std::vector<double> dma_area_predict_fpga(const std::vector<double> parameters) {
  std::vector<double> baseline = {11, 32};
  std::vector<double> output = {
    (xBar[0] + TLB[0] + DMARobScheduler[0])
    + DMAStreamTable[0] / baseline[0] * parameters[0]
    + (DMARob[0] + DMAAgu[0] + DMAReader[0] + DMAWriter[0]) / baseline[1] * parameters[1],
    (xBar[1] + TLB[1] + DMARobScheduler[1])
    + DMAStreamTable[1] / baseline[0] * parameters[0]
    + (DMARob[1] + DMAAgu[1] + DMAReader[1] + DMAWriter[1]) / baseline[1] * parameters[1],
    (xBar[2] + TLB[2] + DMARobScheduler[2])
    + DMAStreamTable[2] / baseline[0] * parameters[0]
    + (DMARob[2] + DMAAgu[2] + DMAReader[2] + DMAWriter[2]) / baseline[1] * parameters[1],
    (xBar[3] + TLB[3] + DMARobScheduler[3])
    + DMAStreamTable[3] / baseline[0] * parameters[0]
    + (DMARob[3] + DMAAgu[3] + DMAReader[3] + DMAWriter[3]) / baseline[1] * parameters[1],
    (xBar[4] + TLB[4] + DMARobScheduler[4])
    + DMAStreamTable[4] / baseline[0] * parameters[0]
    + (DMARob[4] + DMAAgu[4] + DMAReader[4] + DMAWriter[4]) / baseline[1] * parameters[1],
    (xBar[5] + TLB[5] + DMARobScheduler[5])
    + DMAStreamTable[5] / baseline[0] * parameters[0]
    + (DMARob[5] + DMAAgu[5] + DMAReader[5] + DMAWriter[5]) / baseline[1] * parameters[1],
    (xBar[6] + TLB[6] + DMARobScheduler[6])
    + DMAStreamTable[6] / baseline[0] * parameters[0]
    + (DMARob[6] + DMAAgu[6] + DMAReader[6] + DMAWriter[6]) / baseline[1] * parameters[1],
    (xBar[7] + TLB[7] + DMARobScheduler[7])
    + DMAStreamTable[7] / baseline[0] * parameters[0]
    + (DMARob[7] + DMAAgu[7] + DMAReader[7] + DMAWriter[7]) / baseline[1] * parameters[1],
    (xBar[8] + TLB[8] + DMARobScheduler[8])
    + DMAStreamTable[8] / baseline[0] * parameters[0]
    + (DMARob[8] + DMAAgu[8] + DMAReader[8] + DMAWriter[8]) / baseline[1] * parameters[1]
  };
  return output;
}

std::vector<float> output_vport_area_predict_fpga(const float parameter) {
  int next_power_of_two =  pow(2, ceil(log(parameter)/log(2)));
  switch(next_power_of_two) {
      case 1:
        return {(int) (3014 / 4), (int) (2886 / 4), (int) (128 / 4), 0, (int) (584 / 4), 0, 0, 0, 0};
      case 2:
        return {(int) (3221 / 4), (int) (3013 / 4), (int) (208 / 4), 0, (int) (461 / 4), 0, 0, 0, 0};
      case 4:
        return {(int) (5027 / 4), (int) (4739 / 4), (int) (288 / 4), 0, (int) (473 / 4), 0, 0, 0, 0};
      case 8:
        return {(int) (7028 / 4), (int) (6900 / 4), (int) (128 / 4), 0, (int) ( 1017 / 4), 0, 0, 0, 0};
      default: {
        DSA_CHECK(false) << "Output Vector Port Too big. Size: " << parameter;
        return {0, 0, 0, 0, 0, 0, 0, 0, 0};
      }
    }
}

std::vector<double> system_bus = {10024, 10024, 0, 0, 833, 0, 0, 0, 0};
std::vector<double> system_bus_area_predict_fpga(const std::vector<double> parameters) {
  std::vector<double> baseline = {4, 4, 32};
  std::vector<double> output = {
    system_bus[0] / (baseline[0] * baseline[1] * baseline[2]) * (parameters[0] * parameters[1] * parameters[2]),
    system_bus[1] / (baseline[0] * baseline[1] * baseline[2]) * (parameters[0] * parameters[1] * parameters[2]),
    system_bus[2] / (baseline[0] * baseline[1] * baseline[2]) * (parameters[0] * parameters[1] * parameters[2]),
    system_bus[3] / (baseline[0] * baseline[1] * baseline[2]) * (parameters[0] * parameters[1] * parameters[2]),
    system_bus[4] / (baseline[0] * baseline[1] * baseline[2]) * (parameters[0] * parameters[1] * parameters[2]),
    system_bus[5] / (baseline[0] * baseline[1] * baseline[2]) * (parameters[0] * parameters[1] * parameters[2]),
    system_bus[6] / (baseline[0] * baseline[1] * baseline[2]) * (parameters[0] * parameters[1] * parameters[2]),
    system_bus[7] / (baseline[0] * baseline[1] * baseline[2]) * (parameters[0] * parameters[1] * parameters[2]),
    system_bus[8] / (baseline[0] * baseline[1] * baseline[2]) * (parameters[0] * parameters[1] * parameters[2])
  };
  return output;
}

/* Input Parameters {num_input num_output} */
/* Output {TotalLUTs LogicLUTs LUTRAMs SRLs FFs RAMB36 RAMB18 URAM DSPBlocks} */
std::vector<float>  vport_area_predict_fpga(const  std::vector<float> parameters){
  if (parameters[0] == 0) {
    // Output Vector-Port
    int next_power_of_two =  pow(2, ceil(log(parameters[1])/log(2)));
    switch(next_power_of_two) {
      case 1:
        return {(int) (3014 / 4), (int) (2886 / 4), (int) (128 / 4), 0, (int) (584 / 4), 0, 0, 0, 0};
      case 2:
        return {(int) (3221 / 4), (int) (3013 / 4), (int) (208 / 4), 0, (int) (461 / 4), 0, 0, 0, 0};
      case 4:
        return {(int) (5027 / 4), (int) (4739 / 4), (int) (288 / 4), 0, (int) (473 / 4), 0, 0, 0, 0};
      case 8:
        return {(int) (7028 / 4), (int) (6900 / 4), (int) (128 / 4), 0, (int) ( 1017 / 4), 0, 0, 0, 0};
      default: {
        DSA_CHECK(false) << "Vector Port Too big. Size" << parameters[1];
        return {0, 0, 0, 0, 0, 0, 0, 0, 0};
      }
    }
  } else {
    // Input Vector-Port
    int next_power_of_two =  pow(2, ceil(log(parameters[0])/log(2)));
    switch(next_power_of_two) {
      case 1:
        return {(int) (3014 / 4), (int) (2886 / 4), (int) (128 / 4), 0, (int) (584 / 4), 0, 0, 0, 0};
      case 2:
        return {(int) (3221 / 4), (int) (3013 / 4), (int) (208 / 4), 0, (int) (461 / 4), 0, 0, 0, 0};
      case 4:
        return {(int) (5027 / 4), (int) (4739 / 4), (int) (288 / 4), 0, (int) (473 / 4), 0, 0, 0, 0};
      case 8:
        return {(int) (7028 / 4), (int) (6900 / 4), (int) (128 / 4), 0, (int) ( 1017 / 4), 0, 0, 0, 0};
      default: {
        DSA_CHECK(false) << "Vector Port Too big. Size" << parameters[0];
        return {0, 0, 0, 0, 0, 0, 0, 0, 0};
      }
    }
  }
}

double pred_vport_power(const double x1[3]) {
  double xp1_idx_0;
  double xp1_idx_1;
  double xp1_idx_2;
  double a;
  int k;
  static const double b_a[10] = {-0.33609787219468312, -0.49625494362153244,
                                 1.1094301467183465,   -0.14358487664730943,
                                 -1.1097330522698432,  0.46964184688449068,
                                 -0.32873862999759468, 0.8903508829783513,
                                 1.5188577092363085,   -1.1895080458219138};

  static const double c_a[10] = {-0.00883445088691465,   0.61150322081390918,
                                 0.82368363211707929,    -0.011711997303386254,
                                 -0.13730409373614236,   -0.0924128855974144,
                                 -0.0019092101022698364, -0.69353097373900385,
                                 1.7862826518845292,     0.87089503694710679};

  static const double d_a[30] = {
      -0.2249730018228093,    0.11267920379643222,   -0.29193775713945036,
      -0.0039176357141472353, -0.21855246085132746,  0.65499681188758585,
      0.21989075035619152,    -0.34429773055594237,  0.48171548116340479,
      -0.63482214263815906,   0.48260129117800826,   0.055388318646772849,
      -0.34377781362293791,   -1.470802024703032,    -0.30142167080042137,
      -0.03226800685958419,   0.22874724383867073,   -0.044667156150092362,
      0.56478619160431121,    -0.79207463452825067,  0.58283050789071189,
      -1.0511191082986375,    0.6381428735028809,    -0.06823778548293212,
      -0.71118000017843852,   -0.036944769255492137, 1.4056371166568615,
      -0.31694830353124276,   -0.3747937313285038,   -0.28913862318317357};

  //  ===== NEURAL NETWORK CONSTANTS =====
  //  Input 1
  //  Layer 1
  //  Layer 2
  //  Output 1
  //  ===== SIMULATION ========
  //  Dimensions
  //  samples
  //  Input 1
  //  ===== MODULE FUNCTIONS ========
  //  Map Minimum and Maximum Input Processing Function
  xp1_idx_0 = (x1[0] - 1.0) * 0.25 + -1.0;
  xp1_idx_1 = (x1[1] - 1.0) * 0.25 + -1.0;
  xp1_idx_2 = (x1[2] - 1.0) * 0.2 + -1.0;

  //  Layer 1
  //  Sigmoid Symmetric Transfer Function
  //  Layer 2
  //  Output 1
  //  Map Minimum and Maximum Output Reverse-Processing Function
  a = 0.0;
  for (k = 0; k < 10; k++) {
    a += b_a[k] *
         (2.0 /
              (std::exp(-2.0 * (c_a[k] + ((d_a[k] * xp1_idx_0 + d_a[k + 10] * xp1_idx_1) +
                                          d_a[k + 20] * xp1_idx_2))) +
               1.0) -
          1.0);
  }

  return ((a + -0.61184071419455976) - -1.0) / 0.142431881952456 + 0.1597;
}

double pred_vport_area(const double x1[3]) {
  double xp1_idx_0;
  double xp1_idx_1;
  double xp1_idx_2;
  double a;
  int k;
  static const double b_a[10] = {0.42586209957851195,  -0.49529854151114633,
                                 -0.65620882987429452, -0.98926148318253737,
                                 -1.7378056130468968,  0.70556067341231377,
                                 0.862650596672862,    -0.93764422134700276,
                                 0.6730761449343039,   0.87953759557981337};

  static const double c_a[10] = {-0.025240892594136692, -0.65236130269330384,
                                 -0.27582947397582669,  -0.78807116615760675,
                                 -1.5503422476296522,   -0.81809289045653488,
                                 -1.3302834692744177,   0.85464320944095906,
                                 -0.33656296616501835,  0.41393288391340805};

  static const double d_a[30] = {
      -0.580215418492399,    -0.65170839413727388, -0.3855905495436599,
      -0.084747445398591276, -0.2781810916397276,  0.325116319642029,
      0.79148562603794848,   0.053162424002346088, 0.12697933916348914,
      -0.26287652601386963,  0.30370913361567059,  -0.38982569691729035,
      -0.150645764701956,    -0.10600538245496603, -0.25067081989662,
      0.24179902072274875,   0.64639585769652119,  0.21035076813883202,
      0.1106299414379998,    -0.16908827010518743, -0.45239031492415616,
      -0.073628610293940272, -0.84441579666354538, -1.4374538863256627,
      0.69216221298935976,   1.2586824193661335,   0.58205615873715555,
      0.74655213889373984,   -1.589787664240887,   0.58347964793144813};

  //  ===== NEURAL NETWORK CONSTANTS =====
  //  Input 1
  //  Layer 1
  //  Layer 2
  //  Output 1
  //  ===== SIMULATION ========
  //  Dimensions
  //  samples
  //  Input 1
  //  ===== MODULE FUNCTIONS ========
  //  Map Minimum and Maximum Input Processing Function
  xp1_idx_0 = (x1[0] - 1.0) * 0.25 + -1.0;
  xp1_idx_1 = (x1[1] - 1.0) * 0.25 + -1.0;
  xp1_idx_2 = (x1[2] - 1.0) * 0.2 + -1.0;

  //  Layer 1
  //  Sigmoid Symmetric Transfer Function
  //  Layer 2
  //  Output 1
  //  Map Minimum and Maximum Output Reverse-Processing Function
  a = 0.0;
  for (k = 0; k < 10; k++) {
    a += b_a[k] *
         (2.0 /
              (std::exp(-2.0 * (c_a[k] + ((d_a[k] * xp1_idx_0 + d_a[k + 10] * xp1_idx_1) +
                                          d_a[k + 20] * xp1_idx_2))) +
               1.0) -
          1.0);
  }

  return ((a + -0.79628577721831928) - -1.0) / 0.00022396773043197 + 95.941998;
}

double pred_dedi_router_power(const double x1[5]) {
  int k;
  double xp1[5];
  static const signed char iv[5] = {2, 2, 1, 0, 0};

  static const double dv[5] = {0.25, 0.25, 0.285714285714286, 2.0, 2.0};

  double a;
  double d;
  int i;
  static const double b_a[10] = {0.052285807006767486,  -0.071836724521543882,
                                 0.2008635987928401,    0.10889784232688243,
                                 -0.037554635079893296, -1.4668470462137169,
                                 0.16463908629610788,   0.000550285035474616,
                                 0.04836339360436577,   0.0027711243897650039};

  static const double c_a[10] = {
      3.8833476025056766,    1.7777155231230961, -1.4410118423296459, 0.33856275518747025,
      0.0038782229983715943, 1.8917178478128913, 2.3540446176310326,  0.14894567291235411,
      2.3792676071617604,    1.9013070264742096};

  static const double d_a[50] = {
      0.82064868164349425,  -0.002990236049874001, 0.32336656867057539,
      -0.17425263891770965, -1.2571753163428452,   -0.010949608622847837,
      -0.36889395399192126, -0.015453005583982639, 1.8414992936276644,
      2.435976064116125,    1.2556465889552622,    -0.2361664391681608,
      0.61186006066518162,  -0.18183829618480363,  -1.2995680560174017,
      -0.41386379096282272, 1.8581708334225906,    -1.5119104149278555,
      -1.0631997634720904,  0.82894325973521987,   -1.2931775614716217,
      -1.5534738060297557,  0.39049531414621946,   0.18301004701879453,
      0.806642181007831,    -0.12099345713841086,  -0.79503186116104219,
      -2.001248408087,      -0.75535407620381856,  -0.65166607194569048,
      1.8125317017509819,   1.7293902286249776,    1.0872210813292289,
      -0.97375300696654254, 0.56644513158887333,   -0.6525937786770738,
      -0.6061022617634807,  0.03965686203087438,   -0.44728444387551808,
      0.066550409736951255, 0.078564806235120313,  0.053082540249658146,
      1.0610206811276086,   -1.1229100901401059,   0.91188501039277625,
      0.8699265440185564,   0.55308979689547022,   -0.97521177740089526,
      -0.79986194133640565, 0.024969705600209769};

  //  ===== NEURAL NETWORK CONSTANTS =====
  //  Input 1
  //  Layer 1
  //  Layer 2
  //  Output 1
  //  ===== SIMULATION ========
  //  Dimensions
  //  samples
  //  Input 1
  //  ===== MODULE FUNCTIONS ========
  //  Map Minimum and Maximum Input Processing Function
  for (k = 0; k < 5; k++) {
    xp1[k] = (x1[k] - static_cast<double>(iv[k])) * dv[k] + -1.0;
  }

  //  Layer 1
  //  Sigmoid Symmetric Transfer Function
  //  Layer 2
  //  Output 1
  //  Map Minimum and Maximum Output Reverse-Processing Function
  a = 0.0;
  for (k = 0; k < 10; k++) {
    d = 0.0;
    for (i = 0; i < 5; i++) {
      d += d_a[k + 10 * i] * xp1[i];
    }

    a += b_a[k] * (2.0 / (std::exp(-2.0 * (c_a[k] + d)) + 1.0) - 1.0);
  }

  return ((a + 0.476412816219808) - -1.0) / 0.239971875296215 + 0.02279;
}

double pred_dedi_router_area(const double x1[5]) {
  int k;
  double xp1[5];
  static const signed char iv[5] = {2, 2, 1, 0, 0};

  static const double dv[5] = {0.25, 0.25, 0.285714285714286, 2.0, 2.0};

  double a;
  double d;
  int i;
  static const double b_a[10] = {0.092138772791314716,  -0.011539889665360056,
                                 -0.062450197533940079, 0.47620544202671056,
                                 1.3030451716770628,    1.0538085169330644,
                                 -0.42142340402042278,  0.00902359491808406,
                                 -0.17669729407392915,  0.021644527812220732};

  static const double c_a[10] = {
      -2.3255277926398259, 84.478952801785169,  -1.6673578874332293, 0.00862214432003456,
      0.61468970718651084, -1.1952819217572836, 0.98408625223085322, -5.8453353114093227,
      -1.2580874866777876, -0.13461261885681874};

  static const double d_a[50] = {
      0.0662934417403558,   2.4439551723884794,    0.58687312818239079,
      0.67228441558324759,  0.16421889071130769,   0.29257640606266488,
      0.52778988104822611,  -3.0478913628711934,   -0.86043374854157562,
      -1.3272803468184278,  -1.7926527551890361,   3.8628032087170827,
      0.13516979089479547,  0.32949727748339031,   0.4555301583386186,
      0.42137133304999769,  0.55887876108933754,   -1.0910498580480843,
      1.022064433002327,    1.3893166612942469,    0.31745784562764851,
      0.56647129174939,     2.4234588017776253,    0.10578460426501761,
      0.0907919390826639,   0.3116535627946706,    0.33972158963999588,
      1.3371352660948785,   -0.042402984691485492, -1.1040530265080561,
      0.6004833971055098,   40.9280805325977,      -1.3039426081611813,
      1.085579677200583,    -0.99708607144962635,  -0.13347127795050134,
      -0.4057578313955657,  -1.1013190625907869,   1.2321034557763288,
      -1.6729074535205033,  1.8083359856277967,    -41.088949897746787,
      -0.59899726793400176, -1.7242256972027545,   -0.5526791607263859,
      -0.32434171288937141, 0.25741414982021149,   -0.39512496477513803,
      1.3488235688958836,   0.96963887361358969};

  //  ===== NEURAL NETWORK CONSTANTS =====
  //  Input 1
  //  Layer 1
  //  Layer 2
  //  Output 1
  //  ===== SIMULATION ========
  //  Dimensions
  //  samples
  //  Input 1
  //  ===== MODULE FUNCTIONS ========
  //  Map Minimum and Maximum Input Processing Function
  for (k = 0; k < 5; k++) {
    xp1[k] = (x1[k] - static_cast<double>(iv[k])) * dv[k] + -1.0;
  }

  //  Layer 1
  //  Sigmoid Symmetric Transfer Function
  //  Layer 2
  //  Output 1
  //  Map Minimum and Maximum Output Reverse-Processing Function
  a = 0.0;
  for (k = 0; k < 10; k++) {
    d = 0.0;
    for (i = 0; i < 5; i++) {
      d += d_a[k + 10 * i] * xp1[i];
    }

    a += b_a[k] * (2.0 / (std::exp(-2.0 * (c_a[k] + d)) + 1.0) - 1.0);
  }

  return ((a + -0.1036229666020429) - -1.0) / 0.000215696913863699 + 96.921999;
}

double router_power_predict(const double temp_x1[9]) {
  int k;
  double x1[9];
  for (int i = 0; i < 9; i++) {
    x1[i] = temp_x1[i];
  }
  if (x1[8] == 1.0) {
    double dedi_router_features[5];
    dedi_router_features[0] = x1[6];  // #in
    dedi_router_features[1] = x1[7];  // #out
    dedi_router_features[2] = x1[4];  // decomposer
    dedi_router_features[3] = x1[3];  // flow control
    dedi_router_features[4] = x1[2];  // not flow control
    double num_in = x1[6];
    double num_out = x1[7];
    if (num_in == 1.0 || num_out == 1.0) {
      dedi_router_features[0] = 2.0;
      dedi_router_features[1] = 2.0;
      double x4power = pred_dedi_router_power(dedi_router_features) /
                       ((2.0 / num_in) * (2.0 / num_out));
      dedi_router_features[0] = 6.0;
      dedi_router_features[1] = 6.0;
      double x6power = pred_dedi_router_power(dedi_router_features) /
                       ((6.0 / num_in) * (6.0 / num_out));
      return (x4power + x6power) / 2;
    }
    return pred_dedi_router_power(dedi_router_features);
  }

  // Scale the result when # input / output is one
  double scale_ratio = 1.0;
  double min_num_port = 3.0;
  if (x1[6] <= min_num_port) {
    scale_ratio = scale_ratio * (min_num_port / x1[6]);
    x1[6] = min_num_port;
  }
  if (x1[7] <= min_num_port) {
    scale_ratio = scale_ratio * (min_num_port / x1[7]);
    x1[7] = min_num_port;
  }

  double xp1[9];
  static const signed char iv0[9] = {0, 0, 0, 0, 1, 0, 2, 2, 2};

  static const double dv0[9] = {2.0,
                                2.0,
                                2.0,
                                2.0,
                                0.285714285714286,
                                0.25,
                                0.333333333333333,
                                0.333333333333333,
                                0.0666666666666667};

  double a;
  double d0;
  int i0;
  static const double b_a[20] = {
      1.2427122310309535,   0.45512992985216827, 0.75197690514116078,
      0.12741719074083643,  0.98226932041356774, 1.5941212365880275,
      -1.1260214227875707,  1.2705742179003572,  -0.8383893747116361,
      -1.3447519303611146,  0.9699406742563057,  0.99652212320658751,
      -1.2904184837375812,  1.4080626661911686,  0.68978702938813774,
      -1.1550998544127813,  0.34240110571092919, -1.5603537754963621,
      -0.72341436024622008, 0.88181157318608183};

  static const double c_a[20] = {
      -0.45636838524086537,  0.026946926605502158,  0.87624766977337687,
      -1.554794011518672,    0.13601829538915047,   0.0073808880551955924,
      -0.25854518045642111,  0.20276440618096198,   0.59866425477604712,
      -0.42146124173239441,  0.18267410677897858,   0.23678622367271482,
      -0.096818110690631634, -0.024742873218350659, -0.26717404488844143,
      -0.32682135972051612,  -0.047661869846627042, 0.24684769151872077,
      -0.056871980155201229, -0.27310892420231947};

  static const double d_a[180] = {
      0.48993236929935885,     0.19460990356373861,    -0.64540921214966,
      1.5677986375004387,      -0.098506370463236417,  0.15631377879564889,
      0.30913016383215691,     -0.1115974948367276,    -0.88459491974605908,
      0.39237331038190343,     -0.023809110453218575,  -0.16713219975049964,
      0.0057397521638931012,   -0.43829310693198631,   0.27424620680389661,
      0.22982586842153119,     -0.39982313085971644,   -0.2186250739975962,
      -0.0010250387576821393,  0.373045065745445,      -0.48993236930622042,
      -0.19460990356343685,    0.64540921215449254,    -1.5677986375024815,
      0.0985063704746865,      -0.15631377878681266,   -0.309130163832912,
      0.111597494824181,       0.88459491973827986,    -0.39237331038358386,
      0.023809110468309136,    0.16713219975014079,    -0.0057397521647584047,
      0.43829310693748963,     -0.27424620680591388,   -0.22982586842929484,
      0.39982313085540971,     0.21862507400596928,    0.0010250387485450797,
      -0.37304506574481638,    -0.0014530274485732334, -0.25862009657845847,
      0.018033997333833606,    0.053356646440423997,   -0.0041477776936936278,
      -0.0028420585567268831,  0.00031983099703566757, 0.00042766061351004236,
      -0.22948698710115847,    -0.003793317516735153,  0.005887544361673369,
      -0.0064577094122392443,  -0.001198215595584976,  0.78816749799372265,
      -0.0058331704593464831,  -0.064874863939170233,  1.3183973203262735,
      1.4312696395782736,      -0.010221661226594006,  -0.014379734538274241,
      0.0014530274488299083,   0.25862009657103779,    -0.018033997332706726,
      -0.053356646432368038,   0.0041477776941517552,  0.0028420585523315492,
      -0.00031983099733258203, -0.0004276606143435178, 0.2294869871077608,
      0.0037933175220023333,   -0.0058875443624576609, 0.00645770941403591,
      0.0011982155916287188,   -0.78816749798568175,   0.0058331704569679881,
      0.064874863935647911,    -1.3183973203265393,    -1.4312696395793481,
      0.010221661225250138,    0.014379734538356052,   0.13774562574907942,
      0.05070649572235255,     0.31908222943431636,    -1.155905172209992,
      0.0064106630540300552,   0.12788517173242195,    0.29148613375793814,
      0.11833382367984407,     -0.53665854803736934,   0.25468799913800244,
      0.19660381030162541,     0.100430363311559,      0.19924525770314835,
      0.045737514657796616,    0.313554364451222,      -0.011690979325186545,
      0.66047276538962185,     0.071203946217895234,   -0.11070517387248273,
      -0.33158007409467627,    0.00025112899416320362, 0.30023193433396855,
      0.047764194234752679,    -0.073392644079173466,  -0.004608390330270014,
      -0.0021605617466436276,  0.018429456171653234,   -0.00039727139187637845,
      -0.0092161289063835573,  -0.0043016265457941991, 0.012155059655307129,
      -0.01201396982521701,    -0.0056257464278892284, 0.30062658752297,
      -0.010579953047239984,   0.17041291033693129,    -0.01942078051951875,
      -0.700339547103069,      -0.02377719023091401,   -0.0072035782741649177,
      -2.7761856166761141,     0.0081997156178159041,  1.0045057085211864,
      -0.84219383875645359,    -1.1133698036665791,    0.10990430701448128,
      -0.35621722249434606,    -0.46757054918527424,   -0.34566429919502767,
      -3.0153231415727437,     0.42165457558401154,    0.4250637114646324,
      -0.23987962199365404,    0.0046138499304967679,  -2.4993505993353273,
      0.0045696343613757752,   0.4439617975432662,     0.0058971206102767975,
      -0.84330060996574541,    -0.17009499912407425,   0.12885820636143025,
      -0.40332589096177146,    0.253407235174373,      -1.4951182539530845,
      0.060863299785391643,    0.14488766078142809,    0.14175293993061477,
      -0.18139836782885654,    -0.46919306569255065,   0.2290466767912997,
      -0.10566134458470784,    -0.15147517166963131,   -0.16724609408453625,
      0.31300875048135351,     0.28366419503553386,    -0.38904881074220093,
      0.53324894024408653,     0.32376407387651868,    -0.16108258540250278,
      -0.42073153393238816,    0.26016011777945,       0.00908525855058285,
      0.7351897480986741,      -2.2338563826992881,    -0.045201124210140477,
      -0.395503079726311,      -0.17982039064073885,   0.23523882991088085,
      -0.98132083739967679,    0.48697499809725,       0.53641268409143472,
      -0.63254376979300453,    -0.32437746897802705,   0.019940554976885464,
      0.58361243276650132,     -0.023801752397354915,  1.1839036829230194,
      -0.0083718590141281385,  -0.15167641264507403,   -0.6523693262534962};

  //  ===== NEURAL NETWORK CONSTANTS =====
  //  Input 1
  //  Layer 1
  //  Layer 2
  //  Output 1
  //  ===== SIMULATION ========
  //  Dimensions
  //  samples
  //  Input 1
  //  ===== MODULE FUNCTIONS ========
  //  Map Minimum and Maximum Input Processing Function
  for (k = 0; k < 9; k++) {
    xp1[k] = (x1[k] - static_cast<double>(iv0[k])) * dv0[k] + -1.0;
  }

  //  Layer 1
  //  Sigmoid Symmetric Transfer Function
  //  Layer 2
  //  Output 1
  //  Map Minimum and Maximum Output Reverse-Processing Function
  a = 0.0;
  for (k = 0; k < 20; k++) {
    d0 = 0.0;
    for (i0 = 0; i0 < 9; i0++) {
      d0 += d_a[k + 20 * i0] * xp1[i0];
    }

    a += b_a[k] * (2.0 / (1.0 + std::exp(-2.0 * (c_a[k] + d0))) - 1.0);
  }

  return (((-2.0097572839566706 + a) - -1.0) / 0.0461763660123476 + -2.0) / scale_ratio;
}

double router_area_predict(const double temp_x1[9]) {
  int k;
  double x1[9];
  for (int i = 0; i < 9; i++) {
    x1[i] = temp_x1[i];
  }

  if (x1[8] == 1.0) {
    double dedi_router_features[5];
    dedi_router_features[0] = x1[6];  // #in
    dedi_router_features[1] = x1[7];  // #out
    dedi_router_features[2] = x1[4];  // decomposer
    dedi_router_features[3] = x1[3];  // flow control
    dedi_router_features[4] = x1[2];  // not flow control
    double num_in = x1[6];
    double num_out = x1[7];
    double decomposer = x1[4];
    if (num_in == 1.0 || num_out == 1.0 || decomposer == 8.0) {
      dedi_router_features[0] = 2.0;
      dedi_router_features[1] = 2.0;
      dedi_router_features[2] = 4.0;
      double x4area = pred_dedi_router_area(dedi_router_features) /
                      ((2.0 / num_in) * (2.0 / num_out) * (4.0 / decomposer));
      dedi_router_features[0] = 2.5;
      dedi_router_features[1] = 2.5;
      dedi_router_features[2] = 2.0;
      double x6area = pred_dedi_router_area(dedi_router_features) /
                      ((2.5 / num_in) * (2.5 / num_out) * (2.0 / decomposer));
      return (x4area + x6area) / 2;
    }
    return pred_dedi_router_area(dedi_router_features);
  }

  double scale_ratio = 1.0;
  double min_num_port = 10.0;
  if (x1[6] < min_num_port) {
    scale_ratio = scale_ratio * (min_num_port / x1[6]);
    x1[6] = min_num_port;
  }
  if (x1[7] < min_num_port) {
    scale_ratio = scale_ratio * (min_num_port / x1[7]);
    x1[7] = min_num_port;
  }

  double xp1[9];
  static const signed char iv0[9] = {0, 0, 0, 0, 1, 0, 2, 2, 2};

  static const double dv0[9] = {2.0,
                                2.0,
                                2.0,
                                2.0,
                                0.285714285714286,
                                0.25,
                                0.333333333333333,
                                0.333333333333333,
                                0.0666666666666667};

  double a;
  double d0;
  int i0;
  static const double b_a[15] = {
      2.6763474441996773,  -0.52053499710516171, 1.8575906242927878,
      0.92537779492579775, -1.6544073955246821,  1.7527383417246281,
      0.48042326864370677, -0.77908871124343559, 2.5378829790413375,
      2.0530803141305776,  -1.0526070393266171,  -1.7373658306309481,
      5.9597792438025,     -0.443328325695748,   4.1845005659225727};

  static const double c_a[15] = {
      0.1353117768596262,   0.51603120159911386,   0.46099018538109626,
      -0.12416501268600572, -0.062514579493247346, 0.22421011560789381,
      -0.10476491701070588, -0.0528863610884062,   0.61836347953227488,
      0.075991856140100039, -0.43412553665100295,  -0.031001339936404311,
      -0.21210213905238107, 0.046428427126867039,  -0.1501581153220827};

  static const double d_a[135] = {
      0.12414086358104942,    -0.61867955892759752,    0.13222732547645213,
      0.624504313913629,      -0.31999290982425022,    -0.3429678552378212,
      0.20266329630473331,    0.3241966898550821,      -0.51899334868785507,
      -0.097781060020816685,  0.52538772150575719,     -0.074797697094568447,
      0.05292995206400132,    0.047652510137592773,    0.095202376549566842,
      -0.12414086364471681,   0.61867955890766113,     -0.13222732547189311,
      -0.62450431390254124,   0.31999290981425077,     0.342967855231209,
      -0.20266329634153296,   -0.32419668984767352,    0.51899334866439606,
      0.097781060032551645,   -0.52538772150909641,    0.07479769712743263,
      -0.052929952079130482,  -0.047652510163583316,   -0.095202376543218142,
      0.0024314533181297323,  0.014169574625624893,    0.47993004457527438,
      -0.46702976060037354,   0.82489417308287427,     0.0010324912282622446,
      0.00816530125333516,    -0.0059232847466737888,  0.28499893233873674,
      0.00026607037417237449, -0.00094133093623215115, 0.00036299357226963574,
      -0.0031941956716063338, 0.0049881674388348451,   -0.00014816394181007232,
      -0.0024314533340158919, -0.014169574623868374,   -0.47993004456936977,
      0.46702976060355511,    -0.824894173094525,      -0.0010324912119590074,
      -0.0081653012756441028, 0.0059232847410404443,   -0.28499893232367718,
      -0.0002660703603319775, 0.00094133093303824041,  -0.00036299353589987438,
      0.0031941957051648305,  -0.004988167423502895,   0.00014816394418386305,
      -0.18790558657088882,   0.19221514075891397,     -0.00036700580374355332,
      0.12727755619463704,    0.0521214724982292,      -0.28640962528791514,
      0.34391832521538346,    -0.24987451645099268,    -0.0396803397902072,
      0.094315781226212717,   -0.12617043310382536,    -0.12238634748593266,
      0.1495231581275068,     0.40914293469666357,     -0.0773304642012931,
      0.0028627789264376829,  0.015585222707812428,    0.00829357806796351,
      0.25651928798894441,    0.11285884292957758,     0.0012720310317536122,
      -0.0046351230507452245, -0.0075461395601524438,  0.18520161451472836,
      -0.0040257247231232059, 0.0060042357654442767,   0.00585722722626185,
      -0.0068967350245333382, -0.010361013877422996,   0.0038742769457353822,
      -0.17137589261030947,   0.19623472518683452,     -0.00906815702240463,
      0.021464478611151635,   0.013233983618712131,    0.38418196160418977,
      0.33546546645206848,    -0.4912159255281644,     0.0039913265012905411,
      5.0459214712079818,     -3.5000390585569088,     -1.411900403174535,
      -0.1268704302542929,    -0.26596409850443226,    -2.7043509117334863,
      -0.32535294021139238,   1.1508876418315324,      0.535745017238543,
      -0.13278266656885745,   -0.662295063613748,      -0.017055751585135216,
      0.21122665366151538,    0.04564041313039912,     -0.417120414739455,
      0.068363423366641834,   -0.092858905821468163,   -0.10104756485876708,
      0.10408888407817656,    -0.20636915914591181,    -0.058233339078218711,
      -0.27460718860506189,   0.33411490214706951,     -0.034129613441487676,
      0.05912932916831317,    0.055206310989988017,    0.12598314459000604,
      -0.63378975559893969,   0.69322212802501981,     0.0048296370450634785,
      0.18474918397445808,    -0.2458631083774562,     -0.24232056861826576,
      0.20024827705553203,    -0.50550569452580618,    -0.1528501451732773};

  //  ===== NEURAL NETWORK CONSTANTS =====
  //  Input 1
  //  Layer 1
  //  Layer 2
  //  Output 1
  //  ===== SIMULATION ========
  //  Dimensions
  //  samples
  //  Input 1
  //  ===== MODULE FUNCTIONS ========
  //  Map Minimum and Maximum Input Processing Function
  for (k = 0; k < 9; k++) {
    xp1[k] = (x1[k] - static_cast<double>(iv0[k])) * dv0[k] + -1.0;
  }

  //  Layer 1
  //  Sigmoid Symmetric Transfer Function
  //  Layer 2
  //  Output 1
  //  Map Minimum and Maximum Output Reverse-Processing Function
  a = 0.0;
  for (k = 0; k < 15; k++) {
    d0 = 0.0;
    for (i0 = 0; i0 < 9; i0++) {
      d0 += d_a[k + 15 * i0] * xp1[i0];
    }

    a += b_a[k] * (2.0 / (1.0 + std::exp(-2.0 * (c_a[k] + d0))) - 1.0);
  }

  return (((-0.92890835966077534 + a) - -1.0) / 7.42757860780107E-5 + 112.601999) /
         scale_ratio;
}

double pe_power_predict(const double temp_x1[12]) {
  int k;
  double x1[12];
  x1[0] = 0.0;
  x1[1] = 1.0;
  for (int i = 2; i < 12; i++) {
    x1[i] = temp_x1[i];
  }
  double xp1[12];
  static const signed char iv0[12] = {0, 0, 0, 0, 0, 0, 1, 0, 2, 1, 2, 2};

  static const double dv0[12] = {2.0,
                                 2.0,
                                 2.0,
                                 2.0,
                                 2.0,
                                 2.0,
                                 0.285714285714286,
                                 0.2,
                                 0.666666666666667,
                                 1.0,
                                 0.0666666666666667,
                                 0.0666666666666667};

  double a;
  double d0;
  int i0;
  static const double b_a[20] = {
      0.1653129239084053,     -0.12584333069264217,   0.674231993490786,
      -1.6804553605072226,    0.15217444744695216,    1.5784342270781075,
      -0.061770736169562024,  0.00087008330289396058, 0.00063247128448299392,
      -0.81631958744511512,   0.000933563154276984,   -0.00032944762472614003,
      -0.0011302280340805811, -1.0409766979623944,    -0.0025958292954601906,
      0.13884352550949286,    0.16681146708848352,    -0.11423269312873904,
      -2.1419756161797494,    -1.359097786386769};

  static const double c_a[20] = {
      -1.8311106376686044,  -2.0478166679340295,  1.8157672894251551,
      -0.91539631475458216, -0.78634042334418242, 0.67290701764203231,
      0.493904563926242,    -0.83556931225060993, -1.097636054099548,
      -0.57766971413342183, 0.48530701252136604,  -0.074662056823540865,
      0.24694632889467724,  -1.5039826965022995,  1.6402152170764157,
      -0.5184414102362418,  -2.079332082822543,   1.1156940713910795,
      1.5278913378957961,   1.8984709271077094};

  static const double d_a[240] = {
      0.54758733541325011,     0.30931583536656121,     -0.668416963303646,
      0.49115579741589788,     -0.044193884130985156,   0.24409074111398407,
      -0.28817033709428108,    0.60281343575139612,     1.0977948578597925,
      1.0900057894504407,      -0.64223623289733522,    0.71059968805469131,
      1.0395174066028969,      -0.92396956516550188,    -0.31780495052391378,
      -1.1345044738699157,     0.25072915384728361,     1.1318626903030076,
      1.0721217021742553,      0.805055861383129,       0.433254579780154,
      -0.40837058378223823,    0.62241931186956345,     1.2425208846618057,
      0.58201933008260909,     -0.45858840180987753,    0.33073832460371527,
      -0.72643635073544155,    0.16103427966371717,     -0.85215358884139314,
      0.37940803740066575,     -0.70700523617360311,    -0.57893751307540775,
      0.16933747302462854,     0.16334473779344474,     1.0727711494760717,
      0.0081078596908720473,   -0.69138723343228881,    0.29869189258038847,
      -0.81650304687652708,    0.91994368825404083,     -0.30378988666137946,
      0.25937073066316874,     0.51136074118673447,     -1.9979122439777621,
      0.33863413263916264,     -0.015148591192031364,   0.17505385090899633,
      0.84313704256753907,     0.10798286863891224,     -0.83776569842624216,
      0.9659495865565344,      0.46982577705570638,     -0.47845317676513621,
      -0.76243819240030908,    -0.14127745294767571,    1.4541142610379751,
      -0.40137715967617221,    0.18038092573355247,     -0.285169042066191,
      -0.52380070737573259,    -0.3114821401729353,     0.263457226573604,
      0.49990560454024852,     2.1323725070973025,      0.32693924752162923,
      0.00098279799158944,     -0.61167761426073441,    -1.1333961103960113,
      -0.086666773576389688,   1.0628132990076378,      -1.0637750755693274,
      0.45370088181090723,     -0.47820275075035462,    0.835102339988049,
      -0.1288644691762349,     -1.1070312632996939,     -0.37147751120658939,
      0.19132016790869955,     -0.28685890643999351,    -0.50222989433269161,
      1.3181926149807275,      0.31560070670611223,     0.081544901386804536,
      0.066302616944460019,    -0.040211100621519828,   1.2355664804211983,
      0.738190336856921,       -0.89058592955004934,    0.2731858542178307,
      -1.008231939997021,      -0.52045507505756294,    0.71485373355019655,
      0.56835685097015332,     0.17178732430940993,     -1.1281841798386805,
      0.11595762571886446,     0.41128070289531471,     -0.89081218496196213,
      -0.18532162855131903,    -0.51133104601142931,    -0.25965834250511305,
      -0.51497020529108739,    0.078564655740742673,    0.07691676270396812,
      -0.12605638594850038,    -1.1958700647967375,     0.089749024424626178,
      1.0336408455905979,      0.279118432092503,       0.095251478769393591,
      -0.75725060949152789,    -1.1875055203793872,     0.547200595951559,
      -0.0417170431144281,     1.0493127551853747,      0.12195793078873973,
      0.46939655778545336,     -0.78925872115919393,    -0.23052637138828955,
      -0.33841730571699841,    1.0250986011254686,      -0.56922513258131757,
      0.36786553666807076,     0.43961002576661556,     0.22252748894207622,
      -0.94927923637735467,    -0.086659517751054072,   -0.44995526233541044,
      0.42195734174382615,     -0.011456816934018382,   1.8273258283347709,
      -0.34496306853774006,    -0.50779027722908632,    -0.93203440198668186,
      -0.57374353005615586,    0.28450063980292351,     0.53336901551270632,
      -0.49124462263778579,    0.0651833231845823,      3.7567088706232292,
      -0.019342872030644519,   0.025102652401166051,    -0.086779429664616742,
      -4.1699772802917208,     0.022531011405869144,    -0.033271104779922983,
      -1.0255285215639698,     -0.65364985908025364,    -1.7168156171687645,
      -0.021623637377631212,   -0.92971286069111947,    2.2218821493003755,
      -0.036973948171972665,   -0.47535887602246657,    -0.049136176649553859,
      -3.7665234741310578,     0.043508596291281891,    -0.0356541266247535,
      -0.0092103413467264014,  -0.0022341734417041207,  -0.12219002539582129,
      0.097884083086743248,    -0.0012485935476519213,  0.00410967521358271,
      0.018906146297923684,    -0.008334070398015295,   1.7928321754858776,
      0.6321632429312638,      0.00441132776939286,     0.024826842015381413,
      -0.56306336365671239,    0.19970925819041954,     -0.0097692643178313981,
      0.23698043155359022,     -0.0092862913675619135,  -0.0011158369123370389,
      -0.055824904704763458,   0.0049058583825746195,   0.02993849213289123,
      0.005493291255772817,    -0.045248101863852162,   0.0014848118724174679,
      -0.00087511318363510263, -0.0046345408000002782,  0.00023577593342006766,
      -0.0012127321749799474,  0.125978001709813,       0.57866641754665138,
      -0.00076261983269826816, 1.2176823440431344,      0.89201095958136245,
      -0.0087387544238853969,  -0.00045598019500710273, 0.13959803021450953,
      -0.004071077966366965,   -0.0047975604726165581,  -0.0018483014145565019,
      -3.41903226216512E-5,    0.0015295337560106937,   -0.0080313430508819547,
      -1.4593270169017525,     1.312166883754657,       -0.033054390505417919,
      0.0045628787362126527,   0.1597408809498265,      0.097912050574029449,
      0.052036642889206,       0.73573428680740571,     0.0042337559145805715,
      -0.76120023939789938,    0.476070674163483,       0.54493297459412615,
      -0.11618051742795812,    1.9579348700243542,      -0.20693812142175519,
      0.0088707395072320262,   -0.46073090844367037,    -0.048375360991719886,
      0.041691832613342507,    -0.28720725097011018,    1.0315422736525282,
      -0.58977036078226974,    -0.33638266814096324,    0.53841250365844628,
      0.15199769399459259,     -1.5690881855037848,     -1.1414065507879765,
      -0.75756418663956415,    0.460052919530493,       0.29875698369323617,
      -0.78270588845422517,    0.75059298943398989,     0.44108879296799208,
      -0.31671210897604074,    -0.1089776591267872,     0.15590813047135024,
      -0.88087522541317154,    -0.4940411981118184,     0.30236202236016063};

  //  ===== NEURAL NETWORK CONSTANTS =====
  //  Input 1
  //  Layer 1
  //  Layer 2
  //  Output 1
  //  ===== SIMULATION ========
  //  Dimensions
  //  samples
  //  Input 1
  //  ===== MODULE FUNCTIONS ========
  //  Map Minimum and Maximum Input Processing Function
  for (k = 0; k < 12; k++) {
    xp1[k] = (x1[k] - static_cast<double>(iv0[k])) * dv0[k] + -1.0;
  }

  //  Layer 1
  //  Sigmoid Symmetric Transfer Function
  //  Layer 2
  //  Output 1
  //  Map Minimum and Maximum Output Reverse-Processing Function
  a = 0.0;
  for (k = 0; k < 20; k++) {
    d0 = 0.0;
    for (i0 = 0; i0 < 12; i0++) {
      d0 += d_a[k + 20 * i0] * xp1[i0];
    }

    a += b_a[k] * (2.0 / (1.0 + std::exp(-2.0 * (c_a[k] + d0))) - 1.0);
  }

  return ((0.005000789365658005 + a) - -1.0) / 0.0697800184917049;
}

double pe_area_predict(const double temp_x1[12]) {
  int k;
  double x1[12];
  x1[0] = 0.0;
  x1[1] = 1.0;
  for (int i = 2; i < 12; i++) {
    x1[i] = temp_x1[i];
  }
  double xp1[12];
  static const signed char iv0[12] = {0, 0, 0, 0, 0, 0, 1, 0, 2, 1, 2, 2};

  static const double dv0[12] = {2.0,
                                 2.0,
                                 2.0,
                                 2.0,
                                 2.0,
                                 2.0,
                                 0.285714285714286,
                                 0.2,
                                 0.666666666666667,
                                 1.0,
                                 0.0666666666666667,
                                 0.0666666666666667};

  double a;
  double d0;
  int i0;
  static const double b_a[20] = {
      0.32246179149699328,     -0.00038783611863895637, 0.3159345317035897,
      -0.003777606609729782,   -8.263140267234786E-5,   -0.41328735001980882,
      -0.00612511490185159,    -1.1077055590210123,     0.84862936390691668,
      -0.00017459462893358902, -0.12130528154082355,    -0.028504707650152894,
      -0.0055769749527727654,  -0.015656824411140025,   -1.6388403022522962,
      1.79264296511659,        0.017761758149188598,    -0.71860318679980251,
      -0.3266325902497505,     -0.0018685996513895348};

  static const double c_a[20] = {
      -2.12428386661362,    1.6389902905763911,  1.172053502493783,
      1.6124599228447061,   1.8488690747283889,  -0.46623745914489884,
      0.63303035462212465,  0.23458706141070343, 0.28863648034384626,
      -0.20501555668927857, -0.2763000332101912, -0.4909977837804349,
      -0.4413726522147044,  1.441590087453634,   1.7648872548631966,
      0.19423454388276859,  1.2194533662891873,  -0.937463881340698,
      -1.3889360731349296,  -1.8306916177550241};

  static const double d_a[240] = {
      0.41211696344852566,    -0.93083332058716339,   0.14768926103701002,
      -0.030194823223166602,  -1.2295033192689759,    -0.0888062827383438,
      -0.45108555803507822,   0.68165406194367606,    -0.72763565703166422,
      0.0092701615801453318,  -0.69321536535664918,   -0.16371869921000046,
      -0.45899387494495181,   -0.036802339407171855,  0.74466970914350228,
      0.39675868012395432,    0.51971885553152375,    -0.56230897385191592,
      -0.78937261748257948,   -0.15103052861832439,   0.416807369578924,
      -0.82060553406329806,   -0.32056556713862622,   -1.0489044414730573,
      -0.063798743434621108,  0.61954619983129522,    0.60645061054695482,
      0.82013140542644813,    0.61403581453273515,    0.32836067735635033,
      -0.18701444843095436,   -0.98316759034724932,   0.069475858272344759,
      0.86996828462746367,    -0.34553547576020782,   0.727286441171247,
      -0.52957363431533844,   0.64739650512724034,    1.1479981601399665,
      -0.294619470589758,     -0.50220173553017622,   -0.16686364393331587,
      0.19508943588554237,    1.0246416525169748,     1.1436249144745956,
      -0.75779827851119252,   0.19365000045758113,    0.0023865161961881738,
      0.011182457864714317,   -0.7408212739551433,    0.37009903218042539,
      0.16830044060192878,    -0.35033631154121891,   0.40118013107388917,
      -0.30019976795352515,   0.13874331552241767,    0.088177547793163835,
      -0.68175752282612734,   -0.19268753461682081,   0.84073875527224584,
      -0.50892640631812724,   0.72568264885777223,    0.21406348576987241,
      -0.65311660155216456,   -0.026975924944608438,  -0.76733576815094917,
      -0.607007115724079,     -0.0021885533751020043, 0.0023239779180303758,
      0.67597736252537277,    0.38494049165283079,    0.15752085138892358,
      0.78181269887934923,    0.55314711371717085,    -0.30174231124657463,
      0.13681053803051163,    0.38691038960207219,    -0.68065222857292262,
      -0.18905947622563521,   0.011038786311022307,   -1.3664970703446333,
      -0.077813337172148764,  0.43162971902279551,    0.4778612363165784,
      0.77352747028401669,    0.81851127122673306,    -0.12233010398369659,
      0.067756956088673909,   0.37771264008145622,    -0.20571684394047726,
      -0.084749198884988816,  0.48853304438083017,    -0.53882741125503208,
      -0.48631300668610355,   -0.48135647535289028,   -0.104547492978951,
      -1.0285468455109581,    0.4230329679844273,     -0.8057065114286196,
      -0.14523377930082798,   0.54341681937789132,    0.18563678049016488,
      0.24805506278086581,    -1.122969284500642,     -0.72272354447195586,
      -0.80287140476129515,   0.21738679849710449,    0.099341490489256945,
      0.35144999860361786,    0.057904973789040949,   -0.084046398524638147,
      -0.52994057867618838,   -0.28939630394056526,   -0.37961464326533589,
      -0.39557068821553026,   -0.13694384661443107,   0.69854563268073777,
      0.48211148067019594,    0.624548260079763,      0.027953854124771339,
      0.54582035025241049,    -0.088852960746160989,  -1.7903425887476785,
      -0.031552330624948589,  1.9628713933689683,     -0.13696875685365067,
      -0.31013225851961174,   0.8695925661767111,     -0.39055874985303218,
      -0.4306800930964545,    -1.8359997715048697,    0.7433684437998912,
      0.17996691034463752,    1.0595182355281856,     -0.35860168073953491,
      0.51317405038107677,    0.10416094271414392,    -0.75328150279443318,
      0.43972809319420103,    -0.57157408459604186,   -0.76599310831222256,
      0.55439474742935646,    -0.14925462204115586,   -0.229430085503047,
      -0.80182353994727973,   -1.3939117561475687,    -0.3337253141827114,
      -0.0336268638489884,    -0.010400897435220997,  -0.35250095364329209,
      -1.1175395375218078,    0.0684313582693242,     -0.43772259062197028,
      -0.960481269323489,     -0.12907406614840239,   0.05107774924596397,
      -0.654447198398461,     -0.10777465039831433,   1.258120108417909,
      0.17062718275665092,    0.012756888306373898,   -0.39414912215490422,
      0.0038938550209976948,  0.89908737242448256,    -1.0064575887213441,
      0.0045134660033683723,  -0.87933241188445144,   -0.00091225084144185161,
      0.057691988229381021,   -0.32529003879793783,   -0.0014914017884647295,
      -0.07822978270469319,   -0.21975372307585114,   -0.13471655055716869,
      0.0092696636378600244,  0.0011184870360072697,  0.34831042334825368,
      -0.01375957491630779,   -0.0065274730595439971, -0.3222031110039148,
      0.00047975047058338707, -0.85335036833000388,   -0.0027464848130858178,
      0.088814291536646911,   0.14191347819815547,    0.0015149461110351688,
      0.3128477515892531,     0.0006295788116283355,  0.0030691100583427424,
      -0.82752474696204148,   -0.007489170303833362,  -0.021274507454319677,
      -0.15984820787040541,   0.0055302289205027113,  -0.00067588730285296061,
      0.00085617076700589235, 0.069874529690234527,   -0.0019906687504489739,
      -0.0037144183170245352, 0.60023723000569329,    -0.044316533886233557,
      0.038976433779532964,   0.20837581192784815,    0.40140900966739551,
      -1.7143087975513764,    0.0016572858328216585,  -0.92875054307296478,
      -0.067343450899589624,  0.90026644616223284,    -0.66302781073239681,
      0.0084870154517928814,  -1.0012722211065013,    -0.37572508142078059,
      -0.044776566498546014,  0.17046568570104145,    0.14881877654613915,
      -0.1250239129275261,    -0.36213468062869558,   -0.031982124229420442,
      -0.82758889908401845,   0.31080932895187413,    0.12896215160921293,
      0.61873448931996256,    -0.20512669265031105,   0.27235338255367392,
      -0.022642494075418091,  -0.48803295733591989,   -0.33226009491445557,
      -0.568321419720934,     -0.80230113911770529,   0.13310772795256584,
      -0.0914428962681094,    0.501431504270891,      0.74083590463122562,
      -0.39084717363414978,   -0.24835406383308176,   -0.40197829018865661,
      -0.23702432883622582,   0.20233678728238966,    0.87941403576019916};

  //  ===== NEURAL NETWORK CONSTANTS =====
  //  Input 1
  //  Layer 1
  //  Layer 2
  //  Output 1
  //  ===== SIMULATION ========
  //  Dimensions
  //  samples
  //  Input 1
  //  ===== MODULE FUNCTIONS ========
  //  Map Minimum and Maximum Input Processing Function
  for (k = 0; k < 12; k++) {
    xp1[k] = (x1[k] - static_cast<double>(iv0[k])) * dv0[k] + -1.0;
  }

  //  Layer 1
  //  Sigmoid Symmetric Transfer Function
  //  Layer 2
  //  Output 1
  //  Map Minimum and Maximum Output Reverse-Processing Function
  a = 0.0;
  for (k = 0; k < 20; k++) {
    d0 = 0.0;
    for (i0 = 0; i0 < 12; i0++) {
      d0 += d_a[k + 20 * i0] * xp1[i0];
    }

    a += b_a[k] * (2.0 / (1.0 + std::exp(-2.0 * (c_a[k] + d0))) - 1.0);
  }

  return ((0.39072252779946631 + a) - -1.0) / 0.00011258931034665;
}
