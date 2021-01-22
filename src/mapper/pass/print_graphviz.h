#include <ostream>

#include "dsa/dfg/visitor.h"
#include "dsa/mapper/schedule.h"

namespace dsa {
namespace mapper {
namespace pass {

struct GPNode : dfg::Visitor {
  GPNode(Schedule* sched_, std::ostream& os_) : sched(sched_), os(os_) {}
  ~GPNode() {}
  void Visit(dfg::Node* node) override {
    std::string ncolor = "black";
    os << "N" << node->id() << " [ label = \"" << node->name();
    if (node->is_temporal()) {
      os << ":TMP";
    }
    if (sched) {
      if (sched->needs_dynamic[node->id()]) {
        os << ":C";
      }
    }

    if (sched) {
      os << "\\n lat=" << sched->latOf(node) << " ";
    }

    if (sched) {
      auto p = sched->lat_bounds(node);
      os << "\\n bounds=" << p.first << " to " << p.second;
      os << "\\n vio=" << sched->vioOf(node);
    }

    os << "\", color= \"" << ncolor << "\"]; ";

    os << "\n";

    // print edges
    for (auto& v : node->values) {
      for (auto eid : v.uses) {
        auto e = &node->ssdfg()->edges[eid];
        ncolor = "black";

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

void print_graphviz(const std::string& name, SSDfg* dfg, Schedule* sched = nullptr) {
  ofstream ofs(name);
  ofs << "Digraph G { \nnewrank=true;\n ";
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