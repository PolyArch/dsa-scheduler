#include <iostream>
#include <iomanip>

#include "dsa/arch/visitor.h"
#include "dsa/arch/fabric.h"
#include "dsa/mapper/schedule.h"

namespace dsa {
namespace mapper {
namespace pass {


struct Sched_GVNode : adg::Visitor {
  Sched_GVNode(std::ostream& os_, Schedule* sched_) : os(os_), sched(sched_) {}
  ~Sched_GVNode() {}

  void Visit(ssswitch* sw) override {
    os << sw->name() << " [shape=circle, pin=true];" << std::endl;
  }
  
  void Visit(ssfu* fu) override {
    os << fu->name() << "[shape=plaintext, ";
    os << "label = <<table border=\"0\" cellspacing=\"0\">";
    
    auto& np = sched->node_prop()[fu->id()];
    for (int i = 0; i < (int) np.slots.size(); ++i) {
      std::vector<dsa::dfg::Node*> vertices;
      // Populate list of vertices for this node
      for (auto elem : np.slots[i].vertices) {
        if (elem.second == i) {
          vertices.push_back(elem.first);
        }
      }
      if (vertices.size() == 0) {
        os << "<tr><td border=\"1\"> " << fu->name() << " </td></tr>";
      } else {
        for (auto v : vertices) {
          if (!v->values.empty()) {
            os << "<tr><td port=\"" << v->name() << "\" border=\"1\" bgcolor=\"#"
               << std::hex << std::setfill('0') << std::setw(6)
               << sched->colorOf(&v->values[0]) << std::dec << "\">" << v->name()
               << "</td></tr>";
          }
        }
      }
    }

    os << "\n</table>>, pin=true];\n";
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
  Schedule* sched;
};

struct Sched_GPLink : adg::Visitor {
  Sched_GPLink(std::ostream& os_, Schedule* sched_) : os(os_), sched(sched_) {}
  ~Sched_GPLink() {}

  void Visit(ssnode* node) override {
    for (auto link : node->out_links()) {
      int64_t ovr = 0, agg_ovr = 0, max_util = 0;
      sched->get_link_overprov(link, ovr, agg_ovr, max_util);

      std::vector<int> empty_slots;

      // show unique values and slices:  Value, l(), r()
      std::set<std::tuple<dsa::dfg::Value*, int, int>> seen_values;

      auto& lp = sched->link_prop()[link->id()];

      // First Get all the Empty Slots
      for (int slot = 0; slot < (int) lp.slots.size(); ++slot) {
        if (lp.slots[slot].edges.size() == 0) empty_slots.push_back(slot);
      }

      for (int slot = 0; slot < (int) lp.slots.size(); ++slot) {
        for (auto it : lp.slots[slot].edges) {
          dsa::dfg::Edge* e = &sched->ssdfg()->edges[it.eid];
          auto print_tuple = std::make_tuple(e->val(), e->l, e->r);
          if (!seen_values.count(print_tuple)) {
            seen_values.insert(print_tuple);

            // First Print the Name of Link Source Node and Sink Node
            os << link->source()->name() << "->" << link->sink()->name();
            
            // Print the Color of the link
            os << " [color=\"#" << std::hex << std::setfill('0') 
                << std::setw(6) << sched->colorOf(e->val()) << std::dec << "\" ";
            
            // If the link is overutilized, make it dotted
            if (link->max_util() > 1) {
              os << "style=dotted";
            }
            
            // Print the label of the link
            os << " label=\"";
            if (slot != 0) {
              os << "s:" << slot << "-" << slot + e->bitwidth() / 8 - 1;
            }
            if (agg_ovr != 0) {
              os << "OVR:" << agg_ovr << " ";
            }
            os << "D:" << sched->edge_delay(e) << "/" << sched->max_edge_delay(e) << " ";
            if (empty_slots.size() > 0 && empty_slots.size() != 8) {
              os << "S:";
              int prev_i = -100;
              bool printed_dash = false;
              for (unsigned ind = 0; ind < empty_slots.size(); ind++) {
                int i = empty_slots[ind];
                if (ind == 0) {
                  os << i;
                } else if (ind == empty_slots.size() - 1) {
                  if (printed_dash)
                    os << i;
                  else
                    os << "," << i;
                } else if (prev_i + 1 == i) {
                  if (!printed_dash) {
                    os << "-";
                    printed_dash = true;
                  }
                } else {
                  if (printed_dash) {
                    os << prev_i << "," << i;
                  } else {
                    os << "," << i;
                  }
                  printed_dash = false;
                }
                prev_i = i;
              }
            }

            os << "\"];\n";
          }
        }
      }
      
      if (seen_values.size() == 0) {
        os << link->source()->name() << "->" << link->sink()->name()
            << " [color=gray style=dotted, label=\"";
        if (empty_slots.size() != 8) {
          int prev_i = -100;
          bool printed_dash = false;
          for (unsigned ind = 0; ind < empty_slots.size(); ind++) {
            int i = empty_slots[ind];
            if (ind == 0) {
              os << i;
            } else if (ind == empty_slots.size() - 1) {
              if (printed_dash)
                os << i;
              else
                os << "," << i;
            } else if (prev_i + 1 == i) {
              if (!printed_dash) {
                os << "-";
                printed_dash = true;
              }
            } else {
              if (printed_dash) {
                os << prev_i << "," << i;
              } else {
                os << "," << i;
              }
              printed_dash = false;
            }
            prev_i = i;
          }
        }
        os << "\" fontcolor=gray]" << std::endl;
      }
    }
  }

  std::ostream& os;
  Schedule* sched;
};

inline void sched_graphviz(const std::string& name, SpatialFabric* fabric, Schedule* sched) {
  DSA_INFO << "Generating Schedule Graphvis at " << name;
  ofstream ofs(name);
  DSA_CHECK(ofs.good());
  ofs << "Digraph G {" << std::endl;
  Sched_GVNode gpn(ofs, sched);
  Sched_GPLink gpl(ofs, sched);

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

  for (auto node : fabric->node_list()) {
    node->Accept(&gpl);
  }

  ofs << "}";
}

}  // namespace pass
}  // namespace mapper
}  // namespace dsa
