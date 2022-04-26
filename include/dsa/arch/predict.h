#ifndef PREDICT_H
#define PREDICT_H

// Include Files
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <vector>

// Function Declarations
double pe_area_predict(const double x1[12]);
std::vector<float>  pe_area_predict_fpga(const  std::vector<float> parameters);
double pe_power_predict(const double x1[12]);
double router_area_predict(const double x1[9]);

std::vector<float> switch_area_predict_fpga(const std::vector<float> parameters);
std::vector<float>  router_area_predict_fpga(const  std::vector<float> parameters);
double router_power_predict(const double x1[9]);
double pred_dedi_router_area(const double x1[5]);
double pred_dedi_router_power(const double x1[5]);

std::vector<float> input_vport_area_predict_fpga(const float in_links);
std::vector<float> output_vport_area_predict_fpga(const float in_links);
std::vector<float> vport_area_predict_fpga(const  std::vector<float> parameters);

std::vector<double> spm_area_predict_fpga(const std::vector<double> parameters);
std::vector<double> dma_area_predict_fpga(const std::vector<double> parameters);
std::vector<double> system_bus_area_predict_fpga(const std::vector<double> parameters);

double pred_vport_area(const double x1[3]);
double pred_vport_power(const double x1[3]);

#endif
