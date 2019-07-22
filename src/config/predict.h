#ifndef PREDICT_H
#define PREDICT_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include <cmath>

// Function Declarations
double pe_area_predict(const double x1[12]);
double pe_power_predict(const double x1[12]);
double router_area_predict(const double x1[9]);
double router_power_predict(const double x1[9]);

#endif
