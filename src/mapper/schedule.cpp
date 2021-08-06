
#include "dsa/mapper/schedule.h"

#include <assert.h>

#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <set>
#include <unordered_set>

#include "../utils/color_mapper.h"
#include "../utils/model_parsing.h"
#include "../utils/vector_utils.h"
#include "dsa/arch/ssinst.h"
#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/utils.h"
#include "dsa/dfg/visitor.h"
#include "dsa/mapper/dse.h"
#include "json/json.h"
#include "pass/bitstream.h"

using namespace std;
using namespace dsa;

int Schedule::colorOf(dsa::dfg::Value* v) { return cm::ColorOf(v); }

std::map<dsa::OpCode, int> Schedule::interpretConfigBits(int size, uint64_t* bits) {
  // Figure out if this configuration is real or not
  // NOTE: the first 9 characters of the configuration must spell filename
  // for this hack to work!
  CHECK(strncmp((char*)bits, "filename:", 9) == 0)
      << "Hardware configuration not supported yet!";
  char* c_bits = ((char*)bits) + 9;
  return interpretConfigBitsCheat(c_bits);
}

std::map<dsa::OpCode, int> Schedule::interpretConfigBitsCheat(char* s) {
  auto filename = std::string(".sched/") + s;
  _ssDFG = dsa::dfg::Import(filename);
  struct Counter : dfg::Visitor {
    void Visit(dsa::dfg::Instruction* inst) { ++inst_histo[inst->inst()]; }
    std::map<dsa::OpCode, int> inst_histo;
  } counter;
  _ssDFG->Apply(&counter);
  return counter.inst_histo;
}

void Schedule::LoadMappingInJson(const std::string& mapping_filename) {
  Json::CharReaderBuilder crb;
  std::ifstream ifs(mapping_filename);
  Json::Value *json = new Json::Value;
  std::string errs;
  Json::parseFromStream(crb, ifs, json, &errs);
  auto &instructions = *json;

  SSDfg* dfg = ssdfg();
  SpatialFabric* fabric = ssModel()->subModel();
  for (int i = 0, n = instructions.size(); i < n; ++i) {
    auto& obj = instructions[i];
    auto op = obj["op"].asString();
    if (op == "assign_node") {
      auto dfgnode = obj["dfgnode"].asInt();
      auto adgnode = obj["adgnode"].asInt();
      auto adgslot = obj["adgslot"].asInt();
      this->assign_node(dfg->type_filter<dsa::dfg::Node*>()[dfgnode],
                        {adgslot, fabric->node_list()[adgnode]});
    } else if (op == "assign_link") {
      auto dfgedge = obj["dfgedge"].asInt();
      auto adglink = obj["adglink"].asInt();
      auto adgslot = obj["adgslot"].asInt();
      this->assign_edgelink(&dfg->edges[dfgedge], adgslot, fabric->link_list()[adglink]);
    } else if (op == "assign_delay") {
      auto dfgedge = obj["dfgedge"].asInt();
      auto delay = obj["delay"].asInt();
      this->set_edge_delay(delay, &dfg->edges[dfgedge]);
    } else if (op == "max_lat_mis") {
      this->_max_lat_mis = obj["value"].asInt();
    }
  }
}

void Schedule::DumpMappingInJson(const std::string& mapping_filename) {

  SSDfg* ssDFG = ssdfg();
  auto& nodes = ssDFG->nodes;
  auto& edges = ssDFG->edges;
  Json::Value instructions;

  for (int i = 0, n = nodes.size(); i < n; ++i) {
    if (!is_scheduled(nodes[i])) {
      continue;
    }
    auto loc = location_of(nodes[i]);
    Json::Value mapping;
    mapping["op"] = "assign_node";
    mapping["dfgnode"] = nodes[i]->id();
    mapping["adgnode"] = loc.second->id();
    mapping["adgslot"] = loc.first;
    instructions.append(mapping);
  }

  for (int i = 0, n = edges.size(); i < n; ++i) {
    auto edge = &edges[i];
    auto& links = links_of(edge);
    for (auto link : links) {
      Json::Value mapping;
      mapping["op"] = "assign_link";
      mapping["dfgedge"] = edges[i].id;
      mapping["adglink"] = link.second->id();
      mapping["adgslot"] = link.first;
      instructions.append(mapping);
      Json::Value latency;
      latency["op"] = "assign_delay";
      latency["dfgedge"] = edges[i].id;
      latency["delay"] = edge_delay(&edges[i]);
      instructions.append(latency);
    }
  }

  Json::Value mis;
  mis["op"] = "max_lat_mis";
  mis["value"] = this->_max_lat_mis;
  instructions.append(mis);

  std::ofstream ofs(mapping_filename);
  ofs << instructions;
  CHECK(ofs.good()) << "Cannot open " << mapping_filename;
}

// Write to a header file
void Schedule::printConfigHeader(ostream& os, std::string cfg_name, bool use_cheat) {
  // Step 1: Write the vector port mapping
  // TODO(@Sihao): print out the real config bit stream
  os << "#pragma once\n";

  for (auto& pv : _ssDFG->type_filter<dsa::dfg::InputPort>()) {
    int pn = vecPortOf(&pv);
    if(pv.indirect()) {
      os << "#define P_" << cfg_name << "_" << pv.name() << "_in" << " " << (pn+NUM_IN_PORTS) << "\n";
    } else {
      os << "#define P_" << cfg_name << "_" << pv.name() << " " << pn << "\n";
    }
  }
  os << "\n";
  for (auto& pv : _ssDFG->type_filter<dsa::dfg::OutputPort>()) {
    int pn = vecPortOf(&pv);
    os << "#define P_" << cfg_name << "_" << pv.name() << " " << pn << "\n";
  }
  os << "\n";

  if (use_cheat) {
    printConfigCheat(os, cfg_name);
  } else {
    // For each edge, find out the passthrough node
    int edge_idx = 0;
    SSDfg* ssDFG = ssdfg();
    auto edge_list = ssDFG->edges;
    auto vertex_list = ssDFG->nodes;
    std::vector<std::map<int, dsa::adg::bitstream::NodeInfo>> info(
        this->ssModel()->subModel()->node_list().size());
    std::vector<int> opcodes(this->ssModel()->subModel()->node_list().size());
    for (auto& ep : _edgeProp) {
      dsa::dfg::Edge* edge = &edge_list[edge_idx];
      os << "// -------- EDGE:" << edge_idx << ", extra_lat = " << ep.extra_lat
         << " -------- " << endl;
      // loop for every link
      auto link_iter = ep.links.begin();
      while ((++link_iter) != ep.links.end()) {
        sslink* in_link = (--link_iter)->second;
        sslink* out_link = (++link_iter)->second;
        ssswitch* switch_node = dynamic_cast<ssswitch*>(in_link->sink());
        ssfu* fu_node = dynamic_cast<ssfu*>(out_link->sink());
        // config the switch
        CHECK(switch_node) << "edge can only be routed by switch";
        // TODO(@sihao): Support passthru.
        {
          os << "//   config " << switch_node->name() << endl;
          int in_idx = dsa::vector_utils::indexing(in_link, switch_node->in_links());
          int out_idx = dsa::vector_utils::indexing(out_link, switch_node->out_links());
          info[switch_node->id()][in_idx].route = out_idx;
          // os << "input size = " << switch_node -> in_links().size()
          //   << ", output size = " << switch_node -> out_links().size()<< endl;
          os << "//     route input port " << in_idx << " to output port " << out_idx
             << endl;
        }
        // config the fu
        if (fu_node != nullptr) {
          // the final destination is function unit
          int fu_id = fu_node->id();
          // TODO: Sihao no decomposability supported, so I just take first slot
          // TODO: and first vertex
          auto* vertex = _nodeProp[fu_id].slots[0].vertices[0].first;
          // int vertex_idx = vertex_pair.second;
          int edge_of_vertex_idx = vector_utils::indexing(edge, operands[vertex->id()]);
          // which input port does this edge used
          int input_port_idx = dsa::vector_utils::indexing(out_link, fu_node->in_links());
          CHECK(input_port_idx >= 0) << "not found input port";
          CHECK(edge_of_vertex_idx >= 0) << "This edge's destination is fu but not used?";
          {
            os << "//   config " << fu_node->name() << endl
               << "//     add extra delay " << ep.extra_lat << " for operand "
               << edge_of_vertex_idx << endl
               << "//     route input port " << input_port_idx << " to operand "
               << edge_of_vertex_idx << endl;
            info[fu_node->id()][edge_of_vertex_idx].delay = ep.extra_lat;
            info[fu_node->id()][edge_of_vertex_idx].operand = input_port_idx;
          }
          auto* inst_node = dynamic_cast<dsa::dfg::Instruction*>(vertex);
          CHECK(inst_node) << "why a non-instruction node will be mapped to fu";
          int local_opcode = fu_node->fu_type_.get_encoding(inst_node->inst());
          opcodes[fu_node->id()] = local_opcode;
          os << "//     set current opcode to " << local_opcode << " means "
             << name_of_inst(inst_node->inst()) << endl;
        }
      }
      edge_idx++;
    }

    // print out the config bits for every ssnode
    for (auto& node : ssModel()->subModel()->node_list()) {
      dsa::adg::bitstream::BitstreamWriter bw(info[node->id()], opcodes[node->id()]);
      node->Accept(&bw);
      uint64_t config_bits = 0;
      std::bitset<64> b_config_bit(config_bits);
      os << node->name() << " " << config_bits << " " << b_config_bit << endl;
    }

    /*
    int vertex_idx = 0;
    for(auto & vp : _vertexProp){
      os << "// vertex:" <<vertex_idx++<<endl;
      auto node = vp.node;
      os << "//   "<< node->name()
         << ", min_lat = " <<vp.min_lat
         << ", max_lat = " <<vp.max_lat
         << ", lat = " << vp.lat
         << ", vio = " <<vp.vio
         << endl;
    }

    int node_idx = 0;
    for(auto & np : _nodeProp){
      os <<"//  node:"<<node_idx++<<endl;
      for(int slot_idx = 0; slot_idx <8;slot_idx++){
        auto & vertices = np.slots[slot_idx].vertices;
        for(auto & vertex : vertices){
          os<< "//    slot:" << slot_idx
            << "  " << vertex.first->name()<<endl;
        }
      }
    }
    */
  }

}

void Schedule::printConfigCheat(ostream& os, std::string cfg_name) {
  std::string dfg_fname = ".sched/" + cfg_name + ".dfg.json";
  // TODO(@were): Dump the DFG with noop injected.
  dsa::dfg::Export(ssdfg(), dfg_fname);
  std::string sched_fname = ".sched/" + cfg_name + ".sched.json";
  DumpMappingInJson(sched_fname);

  os << "// CAUTION: This is a serialization-based version\n"
     << "// of the schedule.  (ie. cheating)  It is for simulation only.\n"
     << "// corresponding dfg is in: " << cfg_name << ".*.json\n\n";

  // Approximate number of config words, good enough for now
  int config_words = _ssModel->subModel()->node_list().size();

  config_words = std::max((int)cfg_name.size() + 9, config_words);
  // Negative size indicates funny thing
  os << "#define " << cfg_name << "_size " << config_words << "\n\n";

  // NOTE: Filename is necessary here! it is the indicator that we
  // are cheating and not giving the real config bits
  os << "char " << cfg_name << "_config[" << config_words << "] = \"";
  os << "filename:" << cfg_name << "\";\n\n";
}

void Schedule::printConfigVerif(ostream& os) {}

void Schedule::printMvnGraphviz(std::ofstream& ofs, ssnode* node) {
  auto& np = _nodeProp[node->id()];

  for (int i = 0; i < (int) np.slots.size(); ++i) {
    std::vector<dsa::dfg::Node*> vertices;
    for (auto elem : np.slots[i].vertices) {
      if (elem.second == i) {
        vertices.push_back(elem.first);
      }
    }

    if (vertices.size() == 0) {
      ofs << "<tr><td border=\"1\"> " << node->name() << " </td></tr>";
    } else {
      for (auto v : vertices) {
        if (!v->values.empty()) {
          ofs << "<tr><td port=\"" << v->name() << "\" border=\"1\" bgcolor=\"#"
              << std::hex << colorOf(&v->values[0]) << std::dec << "\">" << v->name()
              << "</td></tr>";
        }
      }
    }
  }
}

void Schedule::printMelGraphviz(std::ofstream& ofs, ssnode* node) {
  for (auto link : node->out_links()) {
    int ovr = 0, agg_ovr = 0, max_util = 0;
    get_link_overprov(link, ovr, agg_ovr, max_util);

    std::vector<int> empty_slots;

    // show unique values and slices:  Value, l(), r()
    std::set<std::tuple<dsa::dfg::Value*, int, int>> seen_values;

    auto& lp = _linkProp[link->id()];
    for (int slot = 0; slot < (int) lp.slots.size(); ++slot) {
      if (lp.slots[slot].edges.size() == 0) empty_slots.push_back(slot);

      for (auto it : lp.slots[slot].edges) {
        dsa::dfg::Edge* e = &ssdfg()->edges[it.eid];
        auto print_tuple = make_tuple(e->val(), e->l, e->r);
        if (!seen_values.count(print_tuple)) {
          seen_values.insert(print_tuple);

          ofs << link->source()->name() << "->" << link->sink()->name() << " [color=\"#"
              << std::hex << colorOf(e->val()) << std::dec << "\" "
              << (link->max_util() > 1 ? " style=dotted " : "") << " label=\"";
          if (slot != 0) {
            ofs << "s:" << slot << "-" << slot + e->bitwidth() / 8 - 1;
          }
          if (agg_ovr != 0) {
            ofs << "OVR:" << agg_ovr << " ";
          }
          ofs << "D:" << edge_delay(e) << "/" << max_edge_delay(e) << " ";

          ofs << "\"];\n";
        }
      }
    }
    if (empty_slots.size()) {
      ofs << link->source()->name() << "->" << link->sink()->name()
          << " [color=gray style=dotted, label=\"";
      if (empty_slots.size() != 8) {
        printCondensedVector(empty_slots, ofs);
      }
      ofs << "\" fontcolor=gray]"
          << "\n";
    }
  }
}

void Schedule::printCondensedVector(std::vector<int>& vec, std::ostream& os) {
  int prev_i = -100;
  bool printed_dash = false;
  for (unsigned ind = 0; ind < vec.size(); ind++) {
    int i = vec[ind];
    if (ind == 0) {
      os << i;
    } else if (ind == vec.size() - 1) {
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

void Schedule::printNodeGraphviz(std::ofstream& ofs, ssnode* node) {
  int sy = _ssModel->subModel()->sizey();

  if (dynamic_cast<ssvport*>(node)) {
    auto& np = _nodeProp[node->id()];
    if (np.slots[0].vertices.size() == 0) return;
  }

  ofs << node->name() << "[shape=plaintext, ";
  ofs << "label = <<table border=\"0\" cellspacing=\"0\">";

  printMvnGraphviz(ofs, node);

  ofs << "\n</table>>, pos = \"" << gvsf * node->x() + gvsf / 2.0 << ","
      << sy - gvsf * node->y() - 1 - gvsf / 2.0 << "!\"";
  ofs << ", pin=true];\n";
}

void Schedule::printSwitchGraphviz(std::ofstream& ofs, ssswitch* sw) {
  int sy = _ssModel->subModel()->sizey();

  ofs << sw->name() << " [shape=diamond, ";
  ofs << "pos = \"" << gvsf * sw->x() << "," << sy - gvsf * sw->y() - 1 << "!\"";
  ofs << ", pin=true];\n";
}

void Schedule::printGraphviz(const char* name) {
  ofstream ofs(name);
  if (!ofs.good()) {
    std::cerr << name << " not opened!" << std::endl;
  }

  dsa::SpatialFabric* sub = _ssModel->subModel();

  ofs << "digraph sched {\n";

  for (auto* elem : sub->input_list()) printNodeGraphviz(ofs, elem);

  for (auto* elem : sub->output_list()) printNodeGraphviz(ofs, elem);

  for (auto* elem : sub->switch_list()) printSwitchGraphviz(ofs, elem);

  for (auto* elem : sub->fu_list()) printNodeGraphviz(ofs, elem);

  for (ssnode* node : sub->node_list()) printMelGraphviz(ofs, node);

  ofs << "}\n\n";
}

void Schedule::stat_printOutputLatency() {
  int n = _ssDFG->type_filter<dsa::dfg::OutputPort>().size();
  cout << "** Output Vector Latencies **\n";
  for (int i = 0; i < n; i++) {
    auto* vec_out = &_ssDFG->type_filter<dsa::dfg::OutputPort>()[i];
    auto loc = location_of(vec_out);
    ssvport* vport = dynamic_cast<ssvport*>(loc.second);
    CHECK(vport) << this;
    cout << vec_out->name() << " to " << vport->name() << " sz" << vport->size() << ": ";
    for (auto inc_edge : operands[vec_out->id()]) {
      int routing_latency = edge_latency(inc_edge);
      int edge_lat = edge_delay(inc_edge) + routing_latency - 1;
      cout << latOf(inc_edge->def()) + edge_lat << " ";
    }
    cout << endl;
  }
}
#include "./pass/iterative_latency.h"

bool Schedule::fixLatency(int& max_lat, int& max_lat_mis) {
  for (auto& i : _edgeProp) {
    i.extra_lat = 0;
  }

  max_lat = 0;
  max_lat_mis = 0;
  dsa::dfg::pass::IterativeLatency(this, max_lat, max_lat_mis, _totalViolation,
                                   _groupMismatch, false);
  this->_max_lat = max_lat;
  this->_max_lat_mis = max_lat_mis;

  return max_lat_mis == 0;
}

void Schedule::validate() {
  // Invariant: All paths should start at the source, and end at the
  // destination
  for (dsa::dfg::Edge& edge : _ssDFG->edges) {
    auto& links = _edgeProp[edge.id].links;
    ssnode* def_node = locationOf(edge.def());
    ssnode* use_node = locationOf(edge.use());

    if (links.size() == 0) continue;  // maybe a partial schedule

    int i = 0;
    sslink* prev_link = nullptr;
    for (auto& linkp : links) {
      sslink* link = linkp.second;
      if (i == 0) {
        CHECK(link->source() == def_node);
      }
      if (i > 0) {
        CHECK(prev_link->sink() == link->source());
      }
      if (i + 1 < (int)links.size()) {
        CHECK(dynamic_cast<ssvport*>(link->sink()) == 0);
      }
      ++i;
      prev_link = link;
    }
    CHECK(prev_link);
    CHECK(prev_link->sink() == use_node);
  }
}

void Schedule::get_overprov(int& ovr, int& agg_ovr, int& max_util) {
  ovr = 0;
  agg_ovr = 0;
  max_util = 0;

  for (auto v : _vertexProp) {
    if (v.node) {
      const auto& np = _nodeProp[v.node->id()];

      // Calculate aggregate overage
      for (int i = 0, m = np.slots.size(); i < m; ++i) {
        auto& slot = np.slots[i];
        int cnt = 0;

        vector<dsa::dfg::Node*> io;
        vector<dsa::dfg::Node*> other;
        vector<dsa::dfg::Operation*> ops;
        for (auto elem : slot.vertices) {
          auto* v = elem.first;
          if (v->is_temporal()) {
            if (v->type() == dsa::dfg::Node::V_INPUT) io.push_back(v);
            if (v->type() == dsa::dfg::Node::V_OUTPUT) io.push_back(v);
          } else if (auto op = dynamic_cast<dsa::dfg::Operation*>(v)) {
            ops.push_back(op);
          } else {
            cnt++;
            other.push_back(elem.first);
          }
        }
        int unique_io = vector_utils::count_unique(io);

        int cur_util = cnt + slot.passthrus.size() + unique_io + (ops.size() != 0);
        int cur_ovr = cur_util - v.node->max_util();
        if (cur_ovr > 0) {
          LOG(OVERPROV) << v.node->name() << ": "
            << cnt << " + " << slot.passthrus.size() << " + "
            << unique_io << " + " << (ops.size() != 0) << " > " << v.node->max_util();
          for (auto elem: io) {
            LOG(OVERPROV) << elem->name();
          }
          for (auto elem: other) {
            LOG(OVERPROV) << elem->name();
          }
        }
        agg_ovr += std::max(cur_ovr, 0);
        ovr = max(ovr, cur_ovr);
        max_util = std::max(cur_util, max_util);
      }
    }
  }

  for (auto& n : _ssModel->subModel()->node_list()) {
    for (auto& elem : n->out_links()) {
      get_link_overprov(elem, ovr, agg_ovr, max_util);
    }
  }
}

void Schedule::get_link_overprov(sslink* link, int& ovr, int& agg_ovr, int& max_util) {
  int n = link->source()->datawidth() / link->source()->granularity();
  for (int slot = 0; slot < n; ++slot) {
    auto& lp = _linkProp[link->id()];
    int util = 0;

    std::vector<dsa::dfg::VectorPort*> vecs;
    std::vector<std::pair<dsa::dfg::Value*, int>> values;

    for (auto& it : lp.slots[slot].edges) {
      dsa::dfg::Edge* edge = &ssdfg()->edges[it.eid];
      auto v = edge->def();
      auto d = edge->use();
      if (v->is_temporal() || d->is_temporal()) {
        if (auto input = dynamic_cast<dsa::dfg::OutputPort*>(v)) {
          vecs.push_back(input);
          continue;
        }
        if (auto* out = dynamic_cast<dsa::dfg::OutputPort*>(d)) {
          vecs.push_back(out);
          continue;
        }
      } else {
        values.push_back(make_pair(edge->val(), edge->l));
      }
    }
    util = vector_utils::count_unique(values) + vector_utils::count_unique(vecs);
    int cur_ovr = util - link->max_util();
    if (cur_ovr > 0) {
      LOG(OVERPROV) << link->name() << ": " << values.size()
                    << " + " << vecs.size() << " > " << link->max_util();
      for (auto &value : values) {
        LOG(OVERPROV) << value.second << " " << value.first->name();
      }
      for (auto &vec : vecs) {
        LOG(OVERPROV) << vec->name();
      }
    }
    ovr = std::max(cur_ovr, ovr);
    agg_ovr += std::max(cur_ovr, 0);
    max_util = std::max(util, max_util);
  }
}

#include "./pass/candidates.h"
#include "./pass/collect_redundancy.h"
#include "./pass/propagate_control.h"
#include "./pass/reversed_topology.h"
#include "./pass/shortest_path.h"
#include "./pass/slice_edges.h"
#include "./pass/throughput.h"

Schedule::Schedule(SSModel* model, SSDfg* dfg) : _ssModel(model), _ssDFG(dfg) {
  allocate_space();
  normalize();
}

Schedule::Schedule(const Schedule& s, bool dup_)
    : _ssModel(s._ssModel),
      _ssDFG(s._ssDFG),
      _totalViolation(s._totalViolation),
      _max_lat(s._max_lat),
      _max_lat_mis(s._max_lat_mis),
      _links_mapped(s._links_mapped),
      _edge_links_mapped(s._edge_links_mapped),
      _groupMismatch(s._groupMismatch),
      _vertexProp(s._vertexProp),
      _edgeProp(s._edgeProp),
      _nodeProp(s._nodeProp),
      _linkProp(s._linkProp),
      _min_expected_route_latency(s._min_expected_route_latency),
      _max_expected_route_latency(s._max_expected_route_latency) {
  if (dup_) {
    // _ssDFG = new SSDfg(*s.ssdfg());
    // TODO(@were): Does it mean all the nodes are actually refered by id, so
    //              it is actually ok not to migrate all the loc redundant references
    //              to its actual node in the new DFG?
    // auto model = ssModel();
    // for (auto node : _ssDFG->nodes) {
    //   auto loc = location_of(node);
    //   if (loc.second) {
    //     for (auto &elem : _nodeProp[loc.second->id()].slots[loc.first].passthrus) {
    //       elem = &_ssDFG->edges[elem->id];
    //     }
    //     for (auto &elem : _nodeProp[loc.second->id()].slots[loc.first].vertices) {
    //       elem.first = _ssDFG->nodes[elem.first->id()];
    //     }
    //   }
    // }
    // for (auto edge :_ssDFG->edges) {
    //   for (auto &link : _edgeProp[edge.id].links) {
    //     for (auto &elem : _linkProp[link.second->id()].slots[link.first].edges) {
    //       elem.first = &_ssDFG->edges[elem.first->id];
    //     }
    //   }
    // }
  }
  for (int i = 0; i < dsa::dfg::Node::V_NUM_TYPES; ++i) {
    _num_mapped[i] = s._num_mapped[i];
  }
  normalize();
}

void Schedule::normalize() {
  auto dfg = _ssDFG;
  auto model = _ssModel;
  dsa::dfg::pass::SliceOverlappedEdges(dfg);
  // Reallocate the space after slicing edges.
  allocate_space();
  reversed_topo = dsa::dfg::pass::ReversedTopology(dfg);
  needs_dynamic = dsa::dfg::pass::PropagateControl(reversed_topo);
  auto redundancy = dsa::dfg::pass::CollectRedundancy(dfg);
  distances = dsa::arch::pass::ShortestPaths(model->subModel());
  operands = std::get<0>(redundancy);
  users = std::get<1>(redundancy);
  group_throughput = dsa::dfg::pass::GroupThroughput(dfg, reversed_topo);
  dsa::mapper::CandidateSpotVisitor cpv(this, 50);
  dfg->Apply(&cpv);
  candidate_cnt = cpv.cnt;
}

double Schedule::estimated_performance() {
  auto dfg = this->ssdfg();
  std::vector<std::vector<double>> bw(dfg->meta.size(), std::vector<double>(2, 0));
  std::vector<double> coef(dfg->meta.size(), (double)2.0);

  for (auto& elem : dfg->type_filter<dsa::dfg::InputPort>()) {
    if ((elem.meta.op >> (int)dsa::dfg::MetaPort::Operation::Read & 1) &&
        elem.meta.source != dsa::dfg::MetaPort::Data::Unknown) {
      bw[elem.group_id()][elem.meta.source == dsa::dfg::MetaPort::Data::SPad] +=
          (double) elem.get_vp_len() * elem.get_port_width() / 8;
    } else if ((elem.meta.op >> (int)dsa::dfg::MetaPort::Operation::IndRead & 1) ||
               (elem.meta.op >> (int)dsa::dfg::MetaPort::Operation::IndWrite) & 1) {
      if (this->ssModel()->indirect() < 1) {
        coef[elem.group_id()] = 0.1;
      }
    } else if (elem.meta.op >> (int)dsa::dfg::MetaPort::Operation::Atomic & 1) {
      if (this->ssModel()->indirect() < 2) {
        coef[elem.group_id()] = 0.1;
      }
    }
    coef[elem.group_id()] *= elem.meta.cmd;
  }

  std::vector<int> inst_cnt(dfg->meta.size(), 0);
  for (auto& elem : dfg->type_filter<dsa::dfg::Instruction>()) {
    ++inst_cnt[elem.group_id()];
  }

  double memory_bw = 64 * this->ssModel()->io_ports;
  std::vector<double> bw_coef(dfg->meta.size(), 1.0);
  for (int i = 0; i < dfg->meta.size(); ++i) {
    for (int j = 0; j < 2; ++j) {
      // std::cout << "memory bandwidth: " << memory_bw << " ? " << bw[i][j] << std::endl;
      if (bw[i][j] > memory_bw) {
        bw_coef[i] = std::min(bw_coef[i], memory_bw / bw[i][j]);
      }
    }
  }

  std::vector<double> nmlz_freq;
  for (int i = 0; i < dfg->meta.size(); ++i) {
    nmlz_freq.push_back(dfg->meta[i].frequency);
  }
  double nmlz = *std::max_element(nmlz_freq.begin(), nmlz_freq.end());
  for (int i = 0; i < dfg->meta.size(); ++i) {
    nmlz_freq[i] /= nmlz;
  }

  std::vector<double> rec_lat(dfg->meta.size(), 0.0);
  std::vector<double> rec_hide(dfg->meta.size(), 0.0);
  for (auto& elem : dfg->type_filter<dsa::dfg::OutputPort>()) {
    if (elem.meta.dest == dsa::dfg::MetaPort::Data::LocalPort) {
      double lat = this->latOf(&elem);
      double hide = (double) elem.meta.conc / dfg->meta[elem.group_id()].unroll;
      if (lat > hide) {
        rec_lat[elem.group_id()] = lat;
        rec_hide[elem.group_id()] = hide;
      }
    }
  }

  double overall = 0.0;

  for (int i = 0; i < dfg->meta.size(); ++i) {
    double v =
        std::min(bw_coef[i], rec_hide[i] / rec_lat[i]) * inst_cnt[i] * nmlz_freq[i];
    LOG(ESTIMATION) << "[Group " << i << "] Freq: " << dfg->meta[i].frequency
                    << ", #Insts:" << inst_cnt[i] << ", Memory: " << bw[i][0]
                    << ", SPad: " << bw[i][1] << ", Rec: " << rec_hide[i] << "/"
                    << rec_lat[i] << ", Overall: " << v
                    << ", Performance Coef: " << coef[i];
    overall += v * coef[i];
  }

  return overall;
}
