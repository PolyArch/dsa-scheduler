#include <cassert>
#include <iostream>
#include <cmath>

#include "dsa/arch/pa_model.h"

double decompose_coef(int x) {
  assert(x == (x & -x));
  if (x == 1 || x == 2) {
    x -= 1;
  } else if (x == 4) {
    x = 2;
  } else if (x == 8) {
    x = 3;
  } else {
    assert(false && "unsupported yet!");
  }
  //std::cout << x << ": " << pow(1.1, x) << std::endl;
  return pow(1.1, x);
}

double radix_power(int in, int out, int decompose, bool ctrl) {
  return in * out * decompose_coef(decompose) * (ctrl ? 1.7 : 1.0) * 0.006;
}

double radix_area(int in, int out, int decompose, bool ctrl) {
  return in * out * decompose_coef(decompose) * (ctrl ? 1.7 : 1.0) * 24;
}

double fifo_area(int depth) {
  return 291 * depth;
}

double fifo_power(int depth) {
  return 0.2 * depth;
}
