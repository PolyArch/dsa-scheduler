#pragma once

#include <iostream>
#include <vector>

#include "dsa/arch/model.h"

namespace dsa {
namespace adg {
namespace estimation {

enum class Hardware { ASIC = 0, FPGA = 1 };
enum class Metric { Power = 0, Area = 1, Total = 2 };
enum class Breakdown { FU = 0, Network = 1, Sync = 2, Memory = 3, Total = 4 };

struct Resource {
  virtual ~Resource() {}
  virtual double normalize() = 0;
  virtual std::string dump() = 0;
};

struct ASICResource : Resource {
  /*! \brief The estimated power. */
  double power;
  /*! \brief The estimated area. */
  double area;

  double normalize() override;

  std::string dump() override;

  ASICResource(double p = 0, double o = 0) : power(p), area(o) {}
};

struct FPGAResource : Resource {
  /*! \brief The estimated resource. */
  double total_lut, logic_lut, ram_lut, ff;

  FPGAResource(const std::vector<double> &v = {0, 0, 0, 0}) {
    CHECK(v.size() == 4);
    total_lut = v[0];
    logic_lut = v[1];
    ram_lut = v[2];
    ff = v[3];
  }

  double normalize() override;

  std::string dump() override;
};

/*! \brief The result of power/area estimation */
struct Result {
  static std::map<Hardware, std::function<Resource*()>> RESOURCE_CONSTRUCTOR;

  Result();

  /*! \brief Print the breakdowns */
  void Dump(std::ostream&);

  Resource *sum();

  void add(Breakdown k, double power, double area);

  void add(Breakdown k, const std::vector<double> &v);

 private:
  std::vector<Resource*> brkd;
};

/*! \brief Estimating the power/area breakdown of the given hardware */
Result EstimatePowerAera(SSModel*);

}  // namespace estimation
}  // namespace adg
}  // namespace dsa
