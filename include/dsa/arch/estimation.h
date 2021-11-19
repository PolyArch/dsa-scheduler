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
  virtual void normalize() = 0;
  virtual double constrained_resource(int n) = 0;
  virtual std::string constrained_resource_name(int n) = 0;
  virtual std::string dump() = 0;
};

struct ASICResource : Resource {
  /*! \brief The estimated power. */
  double power;
  /*! \brief The estimated area. */
  double area;

  void normalize() override;
  double constrained_resource(int n) override;
  std::string constrained_resource_name(int n) override;
  std::string dump() override;

  ASICResource(double p = 0, double o = 0) : power(p), area(o) {}
};

struct FPGAResource : Resource {
  /*! \brief The estimated resource. */
  double total_lut, logic_lut, ram_lut, srl, ff, ramb32, ramb18, uram, dsp;

  FPGAResource(const std::vector<double> &v = {0, 0, 0, 0, 0, 0, 0, 0, 0}) {
    DSA_CHECK(v.size() == 9);
    total_lut = v[0];
    logic_lut = v[1];
    ram_lut = v[2];
    srl = v[3];
    ff = v[4];
    ramb32 = v[5];
    ramb18 = v[6];
    uram = v[7];
    dsp = v[8];
  }

  void normalize() override;
  double constrained_resource(int n) override;
  std::string constrained_resource_name(int n) override;

  std::string dump() override;
};

/*! \brief The result of power/area estimation */
struct Result {
  static std::map<Hardware, std::function<Resource*()>> RESOURCE_CONSTRUCTOR;

  Result();

  /*! \brief Print the breakdowns */
  void Dump(std::ostream&);
  void Dump_all_resources(std::ostream& os);

  Resource *sum();
  Resource *resource_bd(int breakdown);

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
