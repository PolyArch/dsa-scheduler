#pragma once

#include "dsa/dfg/metadata.h"
#include "dsa/dfg/node.h"

class SSDfg;

namespace dsa {
namespace dfg {

class VectorPort;

struct CompileMeta : MetaPort {
  VectorPort *parent, *destination;
  CompileMeta(const MetaPort&, VectorPort*);
  CompileMeta(){};
};

class VectorPort : public Node {
 public:
  friend class SSDfg;

  VectorPort() {}

  VectorPort(V_TYPE v, int len, int bitwidth, const std::string& name, SSDfg* ssdfg,
             const MetaPort& meta);

  virtual void Accept(Visitor*);

  void set_port_width(int n) { _port_width = n; }

  int get_port_width() { return _port_width; }

  void set_vp_len(int n) { _vp_len = n; }

  int get_vp_len() { return _vp_len; }

  int logical_len() { return _vp_len; }

  int length() { return _ops.size(); }

  bool indirect() { return _indirect; }

  virtual std::string name() override { return _name; }

  virtual int bitwidth() override { return _bitwidth; }

  int phys_bitwidth() { return is_temporal() ? 64 : (values.size() * bitwidth()); }

 protected:
  int _bitwidth;  // element bitwidth
  int _port_width;
  int _vp_len;
  bool _indirect=false;

 public:
  CompileMeta meta;
};

class InputPort : public VectorPort {
 public:
  static std::string Suffix() { return ""; }
  static bool IsInput() { return true; }

  static const int KindValue = V_INPUT;

  void Accept(Visitor*) final;

  InputPort() {}

  InputPort(int len, int width, const std::string& name, SSDfg* ssdfg,
            const MetaPort& meta);

  int current_{0};
  void forward() override;
  bool can_push();
};

class OutputPort : public VectorPort {
 public:
  static std::string Suffix() { return "_out"; }
  static bool IsInput() { return false; }

  static const int KindValue = V_OUTPUT;

  OutputPort() {}

  OutputPort(int len, int width, const std::string& name, SSDfg* ssdfg,
             const MetaPort& meta)
      : VectorPort(V_OUTPUT, len, width, name, ssdfg, meta) {}

  void Accept(Visitor*) override;

  virtual int slot_for_op(Edge* edge, int node_slot) override;

  void forward() override {}
  bool can_pop();
  void pop(std::vector<uint64_t>& data, std::vector<bool>& data_valid);
};

}  // namespace dfg
}  // namespace dsa
