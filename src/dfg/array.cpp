#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/array.h"

namespace dsa {
namespace dfg {

Array::Array(int size, const std::string& name,
                       SSDfg* ssdfg)
    : Node(ssdfg, V_ARRAY, name), bitwidth_(64), size_(size) {}

std::vector<double> Array::Consumption(bool checkInput, bool repeat, bool reuse) {
    std::vector<double> consumption(ssdfg()->meta.size(), 0);
    for (dfg::Edge edge : ssdfg()->edges) {
        // If this edge uses this memory node, then add the bandwidth of associated vector ports
        if (edge.use()->id() == id()) {
          if (auto port = dynamic_cast<dfg::InputPort*>(edge.def())) {
            if (checkInput) {
              auto portBandwidth = port->bandwidth();
              if (repeat) {
                DSA_CHECK(port->meta.repeat >= 1) << "Repeat must be >= 1 " << port->meta.repeat << " given for " << port->name();
              }
              if (reuse) {
                portBandwidth *= (1 - port->meta.reuse);
              }
              consumption[port->group_id()] += portBandwidth;
            }
          } else if (auto port = dynamic_cast<dfg::OutputPort*>(edge.def())) {
            if (!checkInput) {
              auto portBandwidth = port->bandwidth();
              if (repeat) {
                DSA_CHECK(port->meta.repeat >= 1) << "Repeat must be >= 1 " << port->meta.repeat << " given for " << port->name();
                portBandwidth /= port->meta.repeat;
              }
              if (reuse) {
                portBandwidth *= (1 - port->meta.reuse);
              }
              consumption[port->group_id()] += portBandwidth;
            }
          } else {
            DSA_CHECK(false) << "Edge Connected to Array " << name() << " is not a VectorPort";
          }
        }
        if (edge.def()->id() == id()) {
          if (auto port = dynamic_cast<dfg::InputPort*>(edge.use())) {
            if (checkInput) {
              auto portBandwidth = port->bandwidth();
              if (repeat) {
                DSA_CHECK(port->meta.repeat >= 1) << "Repeat must be >= 1 " << port->meta.repeat << " given for " << port->name();
              }
              if (reuse) {
                portBandwidth *= (1 - port->meta.reuse);
              }
              consumption[port->group_id()] += portBandwidth;
            }
          } else if (auto port = dynamic_cast<dfg::OutputPort*>(edge.use())) {
            if (!checkInput) {
              auto portBandwidth = port->bandwidth();
              if (repeat) {
                DSA_CHECK(port->meta.repeat >= 1) << "Repeat must be >= 1 " << port->meta.repeat << " given for " << port->name();
                portBandwidth /= port->meta.repeat;
              }
              if (reuse) {
                portBandwidth *= (1 - port->meta.reuse);
              }
              consumption[port->group_id()] += portBandwidth;
            }
          } else {
            DSA_CHECK(false) << "Edge Connected to Array " << name() << " is not a VectorPort";
          }
        }
    }
    return consumption;
}

DMA::DMA(int size, const std::string& name,
                       SSDfg* ssdfg)
    : Array(size, name, ssdfg) { _ntype = NodeType::DMA; }


bool DMA::recurrant() {
  int input_port = -1;
  int output_port = -1;
  bool conc = false;
  for (dfg::Edge edge : ssdfg()->edges) {
    // If this edge uses this memory node, then add the bandwidth of associated vector ports
    if (edge.use()->id() == id()) {
      if (auto port = dynamic_cast<dfg::OutputPort*>(edge.def())) {
        input_port = port->id();
        if (port->meta.conc > 0)
          conc = true;
      }
    }
    if (edge.def()->id() == id()) {
      if (auto port = dynamic_cast<dfg::InputPort*>(edge.use())) {
        output_port = port->id();
        if (port->meta.conc > 0)
          conc = true;
      }
    }
  }
  return conc && input_port != -1 && output_port != -1;
}

double DMA::reuse() {
  for (dfg::Edge edge : ssdfg()->edges) {
    // If this edge uses this memory node, then add the bandwidth of associated vector ports
    if (edge.use()->id() == id()) {
      if (auto port = dynamic_cast<dfg::OutputPort*>(edge.def())) {
        if (port->meta.reuse > 0)
          return port->meta.reuse;
      }
    }
    if (edge.def()->id() == id()) {
      if (auto port = dynamic_cast<dfg::InputPort*>(edge.use())) {
        if (port->meta.reuse > 0)
          return port->meta.reuse;
      }
    }
  }
  return 0;
}

Scratchpad::Scratchpad(int size, const std::string& name,
                       SSDfg* ssdfg)
    : Array(size, name, ssdfg) { _ntype = NodeType::SPAD; }

Recurrance::Recurrance(int size, const std::string& name,
                       SSDfg* ssdfg)
    : Array(size, name, ssdfg) { _ntype = NodeType::REC; }

Register::Register(int size, const std::string& name,
                       SSDfg* ssdfg)
    : Array(size, name, ssdfg) { _ntype = NodeType::REG; }

Generate::Generate(int size, const std::string& name,
                       SSDfg* ssdfg)
    : Array(size, name, ssdfg) { _ntype = NodeType::GEN; }

}  // namespace dfg
}  // namespace dsa
