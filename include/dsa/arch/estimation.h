#pragma once

#include <functional>
#include <iostream>
#include <vector>
#include <map>

namespace dsa {

class SSModel;

namespace adg {
namespace estimation {

enum class Hardware { ASIC = 0, FPGA = 1 };
enum class Metric { Power = 0, Area = 1, Total = 2 };
enum class Breakdown { FU = 0, Switch = 1, IVPort = 2, OVPort = 3, Scratchpad = 4, DMA = 5, Recurrance = 6, Generate = 7, Register = 8, Core = 9, System_Bus = 10, Total = 11 };

struct Resource {
  virtual ~Resource() {}
  virtual Resource *clone() const = 0;
  virtual void normalize() = 0;
  virtual void scale_cores(int numcores) = 0;
  virtual double constrained_resource(int n) = 0;
  virtual std::string constrained_resource_name(int n) = 0;
  virtual std::vector<double> to_vector() = 0;
  virtual std::string dump() = 0;
};

struct ASICResource : Resource {
  /*! \brief The estimated power. */
  double power;
  /*! \brief The estimated area. */
  double area;

  void normalize() override;
  void scale_cores(int numcores) override;
  double constrained_resource(int n) override;
  std::string constrained_resource_name(int n) override;
  std::vector<double> to_vector() override;
  std::string dump() override;
  Resource *clone() const override;

  ASICResource(double p = 0, double o = 0) : power(p), area(o) {}
};

struct FPGAResource : Resource {
  /*! \brief The estimated resource. */
  double total_lut, logic_lut, ram_lut, srl, ff, ramb36, ramb18, uram, dsp;

  FPGAResource(const std::vector<double> &v = {0, 0, 0, 0, 0, 0, 0, 0, 0});

  void scale_cores(int numcores) override;
  void normalize() override;
  double constrained_resource(int n) override;
  Resource *clone() const override;
  std::string constrained_resource_name(int n) override;
  std::vector<double> to_vector() override;

  std::string dump() override;
};

/*! \brief The result of power/area estimation */
struct Result {
  static std::map<Hardware, std::function<Resource*()>> RESOURCE_CONSTRUCTOR;

  Result();
  Result(const Result &other);

  ~Result();

  /*! \brief Print the breakdowns */
  void Dump(std::ostream&);
  void Dump_all_resources(std::ostream& os);

  void scale_cores(int numcores);

  Resource *sum();
  Resource *resource_bd(int breakdown);

  void add(Breakdown k, double power, double area);
  void add(Breakdown k, const std::vector<double> &v);
  void add_core_overhead();
  void add_system_bus_overhead(int num_cores, int banks, int system_bus_width);
  void add_dma_overhead(int links, int system_bus_width);

 private:
  std::vector<Resource*> brkd;
};

/*! \brief Estimating the power/area breakdown of the given hardware */
Result EstimatePowerAera(SSModel*);

}  // namespace estimation
}  // namespace adg
}  // namespace dsa
