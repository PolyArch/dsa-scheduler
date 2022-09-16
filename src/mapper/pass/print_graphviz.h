#include <ostream>

#include "dsa/dfg/visitor.h"
#include "dsa/mapper/schedule.h"

namespace dsa {
namespace mapper {
namespace pass {

struct GPNode : dfg::Visitor {
  GPNode(Schedule* sched_, std::ostream& os_) : sched(sched_), os(os_) {}
  ~GPNode() {}

  void printNode(dfg::Node* node, std::string color) {
    os << "N" << node->id() << " [ label = \"" << node->name();
    
    if (node->is_temporal()) {
      os << ":TMP";
    }
    
    if (sched) {
      if (sched->needs_dynamic[node->id()]) {
        os << ":C";
      }
      os << "\\n lat=" << sched->latOf(node) << " ";
      auto p = sched->lat_bounds(node);
      os << "\\n bounds=" << p.first << " to " << p.second;
      os << "\\n vio=" << sched->vioOf(node);
    }

    os << "\", style = \"filled\", color= \"" << color << "\"]; ";
    os << "\n";

    // print edges
    for (auto& v : node->values) {
      for (auto eid : v.uses) {
        auto e = &node->ssdfg()->edges[eid];
        std::string ncolor = "black";

        auto* n = e->use();
        os << "N" << node->id() << " -> N" << n->id() << "[ color=";
        os << ncolor;
        os << " label = \"";
        if (v.index != 0) {
          os << "v" << v.index << " ";
        }
        if (sched) {
          os << "l:" << sched->link_count(e) << "\\nex:" << sched->edge_delay(e)
             << "\\npt:" << sched->num_passthroughs(e);
        }
        os << e->l << ":" << e->r;
        os << "\"];\n";
      }
    }
    os << "\n";
  }

  void Visit(dfg::DMA* port) override {
    std::string ncolor = "#00B7EB";
    printNode(port, ncolor);
  }

  void Visit(dfg::Scratchpad* port) override {
    std::string ncolor = "#89CFF0";
    printNode(port, ncolor);
  }

  void Visit(dfg::Register* port) override {
    std::string ncolor = "#0F3D92";
    printNode(port, ncolor);
  }

  void Visit(dfg::Recurrance* port) override {
    std::string ncolor = "#1e9ae0";
    printNode(port, ncolor);
  }

  void Visit(dfg::Generate* port) override {
    std::string ncolor = "#0067A5";
    printNode(port, ncolor);
  }


  void Visit(dfg::InputPort* port) override {
    std::string ncolor = "#c895f6";
    printNode(port, ncolor);
  }

  void Visit(dfg::OutputPort* port) override {
    std::string ncolor = "#571a8e";
    printNode(port, ncolor);
  }

  void Visit(dfg::Instruction* inst) override {
    std::string ncolor = "#f46049";
    printNode(inst, ncolor);
  }

  void Visit(dfg::Operation* inst) override {
    std::string ncolor = "#f46049";
    printNode(inst, ncolor);
  }

  void Visit(dfg::Node* node) override {
    std::string ncolor = "black";
    printNode(node, ncolor);
  }
  Schedule* sched;
  std::ostream& os;
};

template <typename T>
struct GPVec : dfg::Visitor {
  GPVec(Schedule* sched_, std::ostream& os_) : sched(sched_), os(os_) {
    os << "\t{ rank = same; ";
  }
  ~GPVec() { os << "}\n"; }
  void Visit(T* node) override { os << "N" << node->id() << " "; }
  Schedule* sched;
  std::ostream& os;
};

inline void print_graphviz(const std::string& name, SSDfg* dfg, Schedule* sched = nullptr) {
  ofstream ofs(name);
  ofs << "Digraph G { \nnewrank=true;\n overlap = false;\n";
  GPNode gpn(sched, ofs);
  dfg->Apply(&gpn);
  GPVec<dfg::InputPort> gpvi(sched, ofs);
  dfg->Apply(&gpvi);
  GPVec<dfg::OutputPort> gpvo(sched, ofs);
  dfg->Apply(&gpvo);
  ofs << "}";
}

}  // namespace pass
}  // namespace mapper
}  // namespace dsa