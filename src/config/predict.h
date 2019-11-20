#ifndef PREDICT_H
#define PREDICT_H

// Include Files
#include <cmath>
#include <cstddef>
#include <cstdlib>

// Function Declarations
double pe_area_predict(const double x1[12]);
double pe_power_predict(const double x1[12]);
double router_area_predict(const double x1[9]);
double router_power_predict(const double x1[9]);
double pred_dedi_router_area(const double x1[5]);
double pred_dedi_router_power(const double x1[5]);
double pred_vport_area(const double x1[3]);
double pred_vport_power(const double x1[3]);
#endif
