#include <ostream>

#include "dsa/arch/visitor.h"
#include "dsa/arch/fabric.h"

namespace dsa {
namespace mapper {
namespace pass {


struct ADG_GVNode : adg::Visitor {
  ADG_GVNode(std::ostream& os_) : os(os_) {}
  ~ADG_GVNode() {}

  void Visit(ssswitch* sw) override {
    os << sw->name() << " [shape=\"circle\"]" << std::endl;
  }
  
  void Visit(ssfu* fu) override {
    os << fu->name() << " [shape=\"box\"]" << std::endl;
  }

  void Visit(ssivport* ivp) override {
    os << ivp->name() << " [shape=\"circle\"]" << std::endl;
  }

  void Visit(ssovport* ovp) override {
    os << ovp->name() << " [shape=\"circle\"]" << std::endl;
  }

  void Visit(ssdma* dma) override {
    os << dma->name() << " [shape=\"circle\"]" << std::endl;
  }

  void Visit(ssscratchpad* spm) override {
    os << spm->name() << " [shape=\"circle\"]" << std::endl;
  }

  void Visit(ssrecurrence* rec) override {
    os << rec->name() << " [shape=\"circle\"]" << std::endl;
  }

  void Visit(ssgenerate* gen) override {
    os << gen->name() << " [shape=\"circle\"]" << std::endl;
  }

  void Visit(ssregister* reg) override {
    os << reg->name() << " [shape=\"circle\"]" << std::endl;
  }

  std::ostream& os;
};

struct GPLink : adg::Visitor {
  GPLink(std::ostream& os_) : os(os_) {}
  ~GPLink() {}

  void Visit(ssnode* node) override {
    for (auto& link : node->out_links()) {
      os << link->source()->name() << " -> " << link->sink()->name() << std::endl;
    }
  }

  std::ostream& os;
};

inline void adg_graphviz(const std::string& name, SpatialFabric* fabric) {
  DSA_INFO << "Generating Graphvis at " << name;
  ofstream ofs(name);
  DSA_CHECK(ofs.good());
  ofs << "Digraph G {" << std::endl;
  ADG_GVNode gpn(ofs);
  GPLink gpl(ofs);

  ofs << "rankdir=LR;" << std::endl;
  ofs << "packMode=\"clust\";" << std::endl;
  ofs << "splines=true;" << std::endl;
  ofs << "subgraph cluster_spatial {" << std::endl;
  ofs << "style=invis" << std::endl;
  for (auto* sw : fabric->switch_list()) {
    sw->Accept(&gpn);
  }
  for (auto* fu : fabric->fu_list()) {
    fu->Accept(&gpn);
  }
  ofs << "}" << std::endl;

  ofs << "subgraph cluster_input_ports {" << std::endl;
  ofs << "style=invis" << std::endl;
  for (auto* ivp : fabric->input_list()) {
    ivp->Accept(&gpn);
  }
  ofs << "}" << std::endl;

  ofs << "subgraph cluster_output_ports {" << std::endl;
  ofs << "style=invis" << std::endl;
  // OVPort
  for (auto* ovp : fabric->output_list()) {
    ovp->Accept(&gpn);
  }
  ofs << "}" << std::endl;

  ofs << "subgraph cluster_data {" << std::endl;
  ofs << "style=invis" << std::endl;
  for (auto* dma : fabric->dma_list()) {
    dma->Accept(&gpn);
  }
  for (auto* rec : fabric->recur_list()) {
    rec->Accept(&gpn);
  }
  for (auto* gen : fabric->gen_list()) {
    gen->Accept(&gpn);
  }
  for (auto* spm : fabric->scratch_list()) {
    spm->Accept(&gpn);
  }
  for (auto* reg : fabric->reg_list()) {
    reg->Accept(&gpn);
  }
  ofs << "}" << std::endl;

  for (auto link : fabric->link_list()) {
    ofs << link->source()->name() << "->" << link->sink()->name() << std::endl;
  }

  ofs << "}";
}

}  // namespace pass
}  // namespace mapper
}  // namespace dsa
