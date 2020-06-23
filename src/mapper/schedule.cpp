
#include <assert.h>

#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <set>
#include <unordered_set>

#include "dsa/mapper/schedule.h"
#include "dsa/arch/ssinst.h"
#include "dsa/dfg/ssdfg.h"
#include "dsa/dfg/visitor.h"
#include "dsa/dfg/utils.h"
#include "dsa/mapper/dse.h"
#include "json.tab.h"
#include "json.lex.h"
#include "../utils/model_parsing.h"
#include "../utils/color_mapper.h"
#include "../utils/vector_utils.h"

using namespace std;
using namespace dsa;

// Scheduling Interface
extern "C" void libssscheduler_is_present() {}

void Schedule::clear_ssdfg() {
  if (_ssDFG) {
    delete _ssDFG;
    _ssDFG = nullptr;
  }
}

void Schedule::reset_simulation_state() {
}

int Schedule::colorOf(SSDfgValue *v) {
 return cm::ColorOf(v);
}

std::map<dsa::OpCode, int> Schedule::interpretConfigBits(int size, uint64_t* bits) {
  // Figure out if this configuration is real or not
  // NOTE: the first 9 characters of the configuration must spell filename
  // for this hack to work!
  CHECK(strncmp((char*)bits, "filename:", 9) == 0) << "Hardware configuration not supported yet!";
  char* c_bits = ((char*) bits) + 9;
  return interpretConfigBitsCheat(c_bits);
}

std::map<dsa::OpCode, int> Schedule::interpretConfigBitsCheat(char* s) {
  auto filename = std::string("sched/") + s;
  _ssDFG = dsa::dfg::Import(filename);
  struct Counter : dfg::Visitor {
    void Visit(SSDfgInst *inst) {
      ++inst_histo[inst->inst()];
    }
    std::map<dsa::OpCode, int> inst_histo;
  } counter;
  _ssDFG->Apply(&counter);
  return counter.inst_histo;
}

void Schedule::LoadMappingInJson(const std::string& mapping_filename){
  FILE *fjson = fopen(mapping_filename.c_str(), "r");
  CHECK(fjson) << "Open " << mapping_filename << " failed";
  JSONrestart(fjson);
  struct params p;
  JSONparse(&p);

  auto &json = p.data;
  auto &instructions = *json->As<plain::Array>();

  SSDfg* dfg = ssdfg();
  SpatialFabric* fabric = ssModel()->subModel();
  for (int i = 0, n = instructions.size(); i < n; ++i) {
    auto &obj = *instructions[i]->As<plain::Object>();
    auto &op = *obj["op"]->As<std::string>();
    if (op == "assign_node") {
      auto dfgnode = *obj["dfgnode"]->As<int64_t>();
      auto adgnode = *obj["adgnode"]->As<int64_t>();
      auto adgslot = *obj["adgslot"]->As<int64_t>();
      this->assign_node(dfg->nodes<SSDfgNode*>()[dfgnode],
                        {adgslot, fabric->node_list()[adgnode]});
    } else if (op == "assign_link") {
      auto dfgedge = *obj["dfgedge"]->As<int64_t>();
      auto adglink = *obj["adglink"]->As<int64_t>();
      auto adgslot = *obj["adgslot"]->As<int64_t>();
      this->assign_edgelink(dfg->edges()[dfgedge],
                            adgslot, fabric->link_list()[adglink]);
    } else if (op == "assign_delay") {
      auto dfgedge = *obj["dfgedge"]->As<int64_t>();
      auto delay = *obj["delay"]->As<int64_t>();
      this->set_edge_delay(delay, dfg->edges()[dfgedge]);
    }
  }


}

void Schedule::DumpMappingInJson(const std::string& mapping_filename){
  ofstream os(mapping_filename);
  assert(os.good());

  SSDfg * ssDFG = ssdfg();
  std::vector<SSDfgNode *> nodes = ssDFG->nodes<SSDfgNode *>();
  std::vector<SSDfgEdge *> edges = ssDFG->edges();
  plain::Array instructions;

  for (int i = 0, n = nodes.size(); i < n; ++i) {
    auto loc = location_of(nodes[i]);
    plain::Object mapping;
    mapping["op"] = new json::String("assign_node");
    mapping["dfgnode"] = new json::Int(nodes[i]->id());
    mapping["adgnode"] = new json::Int(loc.second->id());
    mapping["adgslot"] = new json::Int(loc.first);
    instructions.push_back(new json::Object(mapping));
  }

  for (int i = 0, n = edges.size(); i < n; ++i) {
    auto links = links_of(edges[i]);
    for (auto link : links) {
      plain::Object mapping;
      mapping["op"] = new json::String("assign_link");
      mapping["dfgedge"] = new json::Int(edges[i]->id());
      mapping["adglink"] = new json::Int(link.second->id());
      mapping["adgslot"] = new json::Int(link.first);
      instructions.push_back(new json::Object(mapping));
      plain::Object latency;
      latency["op"] = new json::String("assign_delay");
      latency["dfgedge"] = new json::Int(edges[i]->id());
      latency["delay"] = new json::Int(edge_delay(edges[i]));
      instructions.push_back(new json::Object(latency));
    }
  }

  json::Array array(instructions);
  json::JSONPrinter printer(os);
  array.Accept(&printer);
}

// Write to a header file
void Schedule::printConfigHeader(ostream& os, std::string cfg_name, bool use_cheat) {
  // Step 1: Write the vector port mapping
  // TODO(@Sihao): print out the real config bit stream
  os << "#ifndef "
     << "__" << cfg_name << "_H__\n";
  os << "#define "
     << "__" << cfg_name << "_H__\n";

  for (auto& pv : _ssDFG->nodes<SSDfgVecInput*>()) {
    int pn = vecPortOf(pv);
    os << "#define P_" << cfg_name << "_" << pv->name() << " " << pn << "\n";
  }
  os << "\n";
  for (auto& pv : _ssDFG->nodes<SSDfgVecOutput*>()) {
    int pn = vecPortOf(pv);
    os << "#define P_" << cfg_name << "_" << pv->name() << " " << pn << "\n";
  }
  os << "\n";

  if (use_cheat) {
    printConfigCheat(os, cfg_name);
  } else {

    // For each edge, find out the passthrough node
    int edge_idx = 0;
    SSDfg * ssDFG = ssdfg();
    std::vector<SSDfgEdge *> edge_list = ssDFG -> edges();
    std::vector<SSDfgNode *> vertex_list = ssDFG -> nodes<SSDfgNode *>();
    for(auto & ep : _edgeProp){
      SSDfgEdge * edge = edge_list[edge_idx];
      os  << "// -------- EDGE:" << edge_idx
          << ", extra_lat = "<< ep.extra_lat
          << " -------- "<<endl;
      // loop for every link
      auto link_iter = ep.links.begin();
      while((++link_iter) != ep.links.end()){
        sslink * in_link = (--link_iter) -> second;
        sslink * out_link = (++link_iter) -> second;
        ssswitch * switch_node = dynamic_cast<ssswitch*>(in_link -> dest());
        ssfu * fu_node = dynamic_cast<ssfu*>(out_link -> dest());
        // config the switch
        if(switch_node != nullptr){
          os << "//   config " << switch_node -> name() << endl;
          int in_idx = dsa::vector_utils::indexing(in_link, switch_node->in_links());
          int out_idx = dsa::vector_utils::indexing(out_link, switch_node -> out_links());
          switch_node->route_io(in_idx, out_idx);
          //os << "input size = " << switch_node -> in_links().size()
          //   << ", output size = " << switch_node -> out_links().size()<< endl;
          os << "//     route input port "<< in_idx 
             <<" to output port "<< out_idx << endl;
        }else{
          assert(false && "edge can only be routed by switch");
        }
        // config the fu
        if(fu_node != nullptr){
          // the final destination is function unit
          int fu_id =fu_node -> id();
          // TODO: Sihao no decomposability supported, so I just take first slot
          // TODO: and first vertex
          SSDfgNode * vertex = _nodeProp[fu_id].slots[0].vertices[0].first;
          //int vertex_idx = vertex_pair.second;
          int edge_of_vertex_idx = vector_utils::indexing(edge, vertex->in_edges());
          // which input port does this edge used
          int input_port_idx = dsa::vector_utils::indexing(out_link, fu_node -> in_links());
          assert(input_port_idx >=0 && "not found input port ?");
          if(edge_of_vertex_idx >= 0){
            os << "//   config " << fu_node -> name()<<endl
               << "//     add extra delay " << ep.extra_lat
               << " for operand " << edge_of_vertex_idx << endl
               << "//     route input port " << input_port_idx
               << " to operand " << edge_of_vertex_idx <<endl;
            fu_node -> add_delay(edge_of_vertex_idx, ep.extra_lat);
            fu_node -> add_operand_sel(edge_of_vertex_idx, input_port_idx);
          }else{
            assert(false && "This edge's destination is fu but not used?");
          }
          SSDfgInst * inst_node = dynamic_cast<SSDfgInst *>(vertex);
          if(inst_node != nullptr){
            int local_opcode = fu_node ->fu_type_.get_encoding(inst_node -> inst());
            fu_node -> set_curr_opcode(local_opcode);
            os << "//     set current opcode to " << local_opcode
               << " means " << name_of_inst(inst_node -> inst()) << endl;
          }else{
            assert(false && "why a non-instruction node will be mapped to fu");
          }
        }
      }
      edge_idx++;
    }

    // print out the config bits for every ssnode
    for(auto & node : ssModel() ->subModel()->node_list()){

      uint64_t config_bits = node->get_config_bits();
      std::bitset<64> b_config_bit(config_bits);

      os << node->name() << " "
         << config_bits << " "
         << b_config_bit 
         << endl;
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

  os << "#endif //" << cfg_name << "_H\n";

}

void Schedule::printConfigCheat(ostream& os, std::string cfg_name) {
  // First, print the config to the file

  std::string dfg_fname = "sched/" + cfg_name + ".dfg.json";
  dsa::dfg::Export(ssdfg(), dfg_fname);
  std::string sched_fname = "sched/" + cfg_name + ".sched.json";
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

  for (int i = 0; i < 8; ++i) {
    std::vector<SSDfgNode*> vertices;
    for (auto elem : np.slots[i].vertices) {
      if (elem.second == i) {
        vertices.push_back(elem.first);
      }
    }

    if (vertices.size() == 0) {
      ofs << "<tr><td border=\"1\"> " << node->name() << " </td></tr>";
    } else {
      for (auto v : vertices) {
        if (!v->values().empty()) {
          ofs << "<tr><td port=\"" << v->name() << "\" border=\"1\" bgcolor=\"#" << std::hex
              << colorOf(v->values()[0]) << std::dec
              << "\">"
              << v->name() << "</td></tr>";
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
    std::set<std::tuple<SSDfgValue*, int, int>> seen_values;

    auto& lp = _linkProp[link->id()];
    for (int slot = 0; slot < 8; ++slot) {
      if (lp.slots[slot].edges.size() == 0) empty_slots.push_back(slot);

      for (auto it : lp.slots[slot].edges) {
        SSDfgEdge* e = it.first;
        auto print_tuple = make_tuple(e->val(), e->l(), e->r());
        if (!seen_values.count(print_tuple)) {
          seen_values.insert(print_tuple);

          ofs << link->orig()->name() << "->" << link->dest()->name() << " [color=\"#"
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
      ofs << link->orig()->name() << "->" << link->dest()->name()
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
  int n = _ssDFG->nodes<SSDfgVecOutput*>().size();
  cout << "** Output Vector Latencies **\n";
  for (int i = 0; i < n; i++) {
    SSDfgVecOutput* vec_out = _ssDFG->nodes<SSDfgVecOutput*>()[i];
    auto loc = location_of(vec_out);
    ssvport* vport = dynamic_cast<ssvport*>(loc.second);
    cout << vec_out->name() << " to " << vport->name() << " sz" << vport->size() << ": ";
    for (auto inc_edge : vec_out->in_edges()) {
      int routing_latency = edge_latency(inc_edge);
      int edge_lat = edge_delay(inc_edge) + routing_latency - 1;
      cout << latOf(inc_edge->def()) + edge_lat << " ";
    }
    cout << endl;
  }
}

bool Schedule::fixLatency(int& max_lat, int& max_lat_mis) {
  for (auto& i : _edgeProp) {
    i.extra_lat = 0;
  }

  iterativeFixLatency();

  max_lat = 0;
  max_lat_mis = 0;
  cheapCalcLatency(max_lat, max_lat_mis);

  // int max_lat2=0;
  // int max_lat_mis2=0;
  // calcLatency(max_lat2, max_lat_mis2);

  // if(max_lat != max_lat2|| max_lat_mis!=max_lat_mis2) {
  //  cout << "max_lat: " << max_lat << " mis:" << max_lat_mis << "\n";
  //  cout << "max_lat2: " << max_lat2 << " mis2:" << max_lat_mis2 << "\n";
  //}

  //_ssDFG->printGraphviz("viz/remap-fail.dot");
  // printGraphviz("viz/sched.gv");
  // cout << "-------------------------------------------------------------------\n";
  // cout << "-------------------------------------------------------------------\n";
  // exit(1);

  // printGraphviz("viz/sched.gv");

  return max_lat_mis == 0;
}

void Schedule::iterativeFixLatency() {
  bool changed = true;
  reset_lat_bounds();

  // int max_ed = _ssModel->maxEdgeDelay();
  int iters = 0;

  bool overflow = false;
  int max_mis = 0;

  int _max_expected_route_latency = 8;

  std::vector<SSDfgNode*> ordered_non_temp;
  std::copy_if(reversed_topo.begin(), reversed_topo.end(), std::back_inserter(ordered_non_temp),
               [](SSDfgNode* node) { return !node->is_temporal(); });

  while (changed || overflow) {
    changed = false;

    iters++;
    if (overflow) {
      overflow = false;
      reset_lat_bounds();
      max_mis++;
    }

    // FORWARD PASS
    for (SSDfgNode* node : ordered_non_temp) {
      auto& vp = _vertexProp[node->id()];
      int new_min = vp.min_lat;
      int new_max = vp.max_lat;

      for (auto edge : node->in_edges()) {
        SSDfgNode* origNode = edge->def();
        auto& orig_vp = _vertexProp[origNode->id()];

        int routing_latency = edge_latency(edge);
        int edge_lat = origNode->lat_of_inst() + routing_latency - 1;

        int max_ed = max_edge_delay(edge);

        // cout << " -----------------" <<  edge->name() << ": " << edge_lat << "\n";

        // This edge is routed
        if (routing_latency != 0) {
          new_min = std::max(new_min, orig_vp.min_lat + edge_lat);
          new_max = std::min(new_max, orig_vp.max_lat + edge_lat + max_ed + max_mis);
        } else {
          // This edge is not routed, so give worst case upper bound
          new_min =
              std::max(new_min, orig_vp.min_lat + edge_lat + _min_expected_route_latency);
          new_max = std::min(new_max, orig_vp.max_lat + edge_lat + max_ed + max_mis +
                                          _max_expected_route_latency);
        }
      }
      changed |= new_min != vp.min_lat;
      changed |= new_max != vp.max_lat;
      vp.min_lat = new_min;
      vp.max_lat = new_max;

      // cout << node->name() << "  min_lat:" << vp.min_lat
      //                     << " max_lat:"<< vp.max_lat
      //                     << " max_mis:" << max_mis << "\n";

      if (new_min > new_max) {
        overflow = true;
        break;
      }
    }

    if (overflow) continue;

    // BACKWARDS PASS
    for (int i = ordered_non_temp.size() - 1; i >= 0; i--) {
      SSDfgNode* node = ordered_non_temp[i];
      auto& vp = _vertexProp[node->id()];
      int new_min = vp.min_lat;
      int new_max = vp.max_lat;

      for (auto edge : node->uses()) {
        if (edge == nullptr) continue;
        SSDfgNode* useNode = edge->use();
        auto& use_vp = _vertexProp[useNode->id()];

        int routing_latency = edge_latency(edge);
        int edge_lat = routing_latency - 1 + node->lat_of_inst();

        // int my_max_ed = max_ed;
        // if (dynamic_cast<SSDfgVecOutput*>(useNode)) {
        //  my_max_ed = 0;
        //}
        int max_ed = max_edge_delay(edge);

        if (routing_latency != 0) {
          new_min = std::max(new_min, use_vp.min_lat - edge_lat - max_ed - max_mis);
          new_max = std::min(new_max, use_vp.max_lat - edge_lat);
        } else {
          new_min = std::max(new_min, use_vp.min_lat - edge_lat - max_ed - max_mis -
                                          _max_expected_route_latency);
          new_max =
              std::min(new_max, use_vp.max_lat - edge_lat - _min_expected_route_latency);
        }
      }
      changed |= new_min != vp.min_lat;
      changed |= new_max != vp.max_lat;
      vp.min_lat = new_min;
      vp.max_lat = new_max;

      // cout << node->name() << "  min_lat-b:" << vp.min_lat
      //                     << " max_lat-b:"<< vp.max_lat << "\n";

      if (new_min > new_max) {
        overflow = true;
        break;
      }
    }
  }

  // cout << "iters until converge: " << iters << ", mismatch: " << max_mis << "\n";
  // NOW SET THE LATENCY!

  // TODO: need to check how this allows delays or not on vector outputs
  for (SSDfgNode* node : ordered_non_temp) {
    auto& vp = _vertexProp[node->id()];
    // cout << inst->name() << "  min_lat:" << vp.min_lat << " max_lat:"
    //                                    << vp.max_lat << "\n";
    int target = vp.min_lat;
    // < vp.max_lat ? vp.min_lat :
    //              (vp.min_lat + vp.max_lat) / 2;
    // cout << "target : " << target << "\n";

    int max = 0;
    // int mis = 0;
    for (auto edge : node->in_edges()) {
      if (edge == nullptr) continue;
      SSDfgNode* origNode = edge->def();

      int routing_latency = edge_latency(edge);
      // int max_edge_delay = _ssModel->maxEdgeDelay();
      int max_ed = max_edge_delay(edge);

      if (routing_latency == 0) {  // if its not scheduled yet, be more liberal
        routing_latency = _min_expected_route_latency;
        max_ed += _max_expected_route_latency;
      }

      int lat = latOf(origNode) + routing_latency - 1;

      int diff = std::max(std::min(max_ed, target - lat), 0);
      // mis = std::max(mis,(target- lat) - diff);
      set_edge_delay(diff, edge);

      int vio = std::max(0, (target - lat) - max_ed);
      if (vio > 0) {
        record_violation(edge, vio);
      }

      max = std::max(max, lat + diff);
      // cout << " -- " << origNode->name() << "diff"  << diff
      //                         << "links:" << link_count(edge)-1 << "\n";
    }
    // cout << " * mis: " << mis << "\n";
    assign_lat(node, node->lat_of_inst() + max);
  }
}

void Schedule::cheapCalcLatency(int& max_lat, int& max_lat_mis) {
  _totalViolation = 0;
  max_lat_mis = 0;
  max_lat = 0;
  _groupMismatch.clear();

  for (SSDfgNode* node : reversed_topo) {
    calcNodeLatency(node, max_lat, max_lat_mis);
  }
}

void Schedule::calcNodeLatency(SSDfgNode* node, int& max_lat, int& max_lat_mis) {
  int low_lat = MAX_SCHED_LAT, up_lat = 0;

  for (auto edge : node->in_edges()) {
    SSDfgNode* origNode = edge->def();

    // If routing latency is 0, then its okay to assume minimum
    int routing_latency = edge_latency(edge);
    if (routing_latency == 0) {
      routing_latency = _min_expected_route_latency;
    }

    if (origNode != nullptr) {
      int edge_lat = edge_delay(edge) + routing_latency - 1;
      assert(edge_lat >= 0);
      int lat = latOf(origNode) + edge_lat;

      if (lat > up_lat) up_lat = lat;
      if (lat < low_lat) low_lat = lat;
    }
  }

  assign_lat_bounds(node, low_lat, up_lat);  // FIXME: turn off, just for debug

  int diff = up_lat - low_lat;  // - _ssModel->maxEdgeDelay();

  if (!node->is_temporal()) {
    if (diff > max_lat_mis) {
      max_lat_mis = diff;
    }
    if (diff > _groupMismatch[node->group_id()]) {
      _groupMismatch[node->group_id()] = diff;
    }
    if (diff > 0) {
      add_violation(diff);
      record_violation(node, diff);
    } else {
      record_violation(node, 0);
    }
  }

  int new_lat = node->lat_of_inst() + up_lat;
  assign_lat(node, new_lat);

  // ssnode* n = locationOf(inst);
  // cout << "C " << inst->name() << " node: " << n->name()
  //  << " low_lat: " << low_lat << " up_lat:" << up_lat << " latmis:"
  //  << max_lat_mis << "diff: " << diff << "\n";

  if (max_lat < new_lat) max_lat = new_lat;
}

void Schedule::validate() {
  // Invariant: All paths should start at the source, and end at the
  // destination
  for (SSDfgEdge* edge : _ssDFG->edges()) {
    auto& links = _edgeProp[edge->id()].links;
    ssnode* def_node = locationOf(edge->def());
    ssnode* use_node = locationOf(edge->use());

    if (links.size() == 0) continue;  // maybe a partial schedule

    int i = 0;
    sslink* prev_link = nullptr;
    for (auto& linkp : links) {
      sslink* link = linkp.second;
      if (i == 0) {
        assert(link->orig() == def_node);
      }
      if (i > 0) {
        assert(prev_link->dest() == link->orig());
      }
      if (i + 1 < (int) links.size()) {
        assert(dynamic_cast<ssvport*>(link->dest()) == 0);
      }
      ++i;
      prev_link = link;
    }
    assert(prev_link);
    assert(prev_link->dest() == use_node);
  }
}

// Calculate the exact latency by traversing the schedule
// -- Note that this function is performance non-critical, as its
// purpose is to verify the schedule's latency calcuated cheaply
// TODO: This function should be udpated for link slots
void Schedule::calcLatency(int& max_lat, int& max_lat_mis, bool warnMismatch) {
  queue<sslink*> openset;

  unordered_map<sslink*, int> lat_edge;

  max_lat = 0;
  max_lat_mis = 0;

  for (auto elem : _ssModel->subModel()->nodes<ssvport*>()) {
    for (auto link : elem->out_links()) {
      if (SSDfgNode* dfgnode = dfgNodeOf(0, link)) {
        openset.push(link);
        lat_edge[link] = latOf(dfgnode);
      }
    }
  }

  // Outlinks of all the inputs
  while (!openset.empty()) {
    sslink* inc_link = openset.front();
    openset.pop();
    // cout << inc_link->name() << "\n";

    // dest node
    ssnode* node = inc_link->dest();

    if (ssnode* next_node = dynamic_cast<ssfu*>(node)) {
      sslink* new_link = next_node->out_links()[0];
      if (lat_edge.count(new_link)) {
        continue;  // skip if we've done it already
      }

      SSDfgNode* next_dfgnode = dfgNodeOf(0, node);
      if (!next_dfgnode && !isPassthrough(0, node)) {
        assert(false && "problem with latency calculation!\n");
        max_lat = -1;
        max_lat_mis = -1;
        return;
      }

      SSDfgInst* next_dfginst = dynamic_cast<SSDfgInst*>(next_dfgnode);
      assert(next_dfginst || isPassthrough(0, node));

      bool everyone_is_here = true;

      for (auto& inlink : next_node->in_links()) {
        for (int slot = 0; slot < 8; ++slot) {
          if (dfgNodeOf(slot, inlink) != nullptr) {
            if (!lat_edge.count(inlink)) {
              everyone_is_here = false;
              break;
            }
          }
        }
      }

      int max_latency = 0;
      int low_latency = 100000000;  // magic number, forgive me

      if (everyone_is_here) {
        // cout << "----------------------------------- DONE WITH "
        //     <<  next_fu->name() << "\n";
        // Latency should be the same across all the incoming edges
        for (auto& inlink : next_node->in_links()) {
          for (int slot = 0; slot < 8; ++slot) {
            SSDfgNode* origNode = dfgNodeOf(slot, inlink);
            if (origNode != nullptr) {
              int curLat = lat_edge[inlink];
              // cout << "reading: " << inlink->name() << "\n";

              if (!isPassthrough(0, node)) {
                SSDfgEdge* edge = origNode->getLinkTowards(next_dfgnode);
                CHECK(edge) << "Edge: " << origNode->name() << " has no edge towards "
                            << next_dfgnode->name() << ", for link:" << inlink->name() << "\n";
                if (edge_delay(edge)) {
                  curLat += edge_delay(edge);
                }
              }

              if (curLat > max_latency) {
                max_latency = curLat;
              }
              if (curLat < low_latency) {
                low_latency = curLat;
              }

              if (warnMismatch && max_latency != low_latency) {
                cout << "Mismatch, min_lat:" << low_latency << ", max_lat:" << max_latency
                     << ", link:" << inlink->name() << "\n";
                if (!isPassthrough(0, node)) {
                  SSDfgEdge* edge = origNode->getLinkTowards(next_dfgnode);
                  cout << "(calcLat) Edge " << edge->name()
                       << "  lat_edge: " << lat_edge[inlink]
                       << "  extralat:" << edge_delay(edge) << "\n";
                } else {
                  cout << "passthrough\n";
                }
              }
            }
          }
        }
      }

      if (everyone_is_here) {
        // Update latency of outgoing edge
        if (isPassthrough(0, node)) {
          lat_edge[new_link] = max_latency + 1;  // TODO: Check this
        } else {                                 // regular inst
          int l = max_latency + inst_lat(next_dfginst->inst());
          lat_edge[new_link] = l;

          // if(next_dfginst) {
          //  cout << "L " << next_dfginst->name() << " lat:" << l
          //       << " low:" << low_latency << " up:" << max_latency << " - "
          //       << " lat:" << latOf(next_dfgnode)
          //       << " low:" << lat_bounds(next_dfgnode).first << " up:"
          //       << lat_bounds(next_dfgnode).second << "\n";
          //}

          assign_lat(next_dfgnode, l);
        }

        // if(next_dfginst) {
        //  cout << "L " << next_dfginst->name() << " " << latOf(next_dfgnode)
        //       << " up:" << max_latency << " low:" << low_latency << "\n";
        //}

        openset.push(new_link);

        // cout << "lat of " << next_dfgnode->name()
        //     << ", old:" << _latOf[next_dfgnode]
        //     << ", new:" << max_latency << " " << lat_edge[new_link] << "\n";

        int diff = max_latency - low_latency;
        if (diff > max_lat_mis) {
          max_lat_mis = diff;
        }
      }
    } else {
      for (auto& out_link : node->out_links()) {
        for (int slot = 0; slot < 8; ++slot) {
          // We'll need to check to make sure if there is ambiguity between links
          if (dfgNodeOf(slot, out_link) != dfgNodeOf(0, inc_link)) {
            continue;
          }
          lat_edge[out_link] = lat_edge[inc_link] + 1;
          openset.push(out_link);
        }
      }
    }
  }

  for (auto& i : lat_edge) {
    sslink* link = i.first;
    int lat = i.second;
    set_link_order(0, link, lat);
  }

  _max_lat = max_lat;
  _max_lat_mis = max_lat_mis;
}

void Schedule::get_overprov(int& ovr, int& agg_ovr, int& max_util) {
  ovr = 0;
  agg_ovr = 0;
  max_util = 0;

  for (auto v : _vertexProp) {
    if (v.node) {
      const auto& np = _nodeProp[v.node->id()];

      // Calculate aggregate overage
      for (int i = 0; i < 8; ++i) {
        auto& slot = np.slots[i];
        int cnt = 0;

        vector<SSDfgNode*> io;
        for (auto elem : slot.vertices) {
          SSDfgNode* v = elem.first;
          if (v->is_temporal()) {
            if (v->type() == SSDfgNode::V_INPUT) io.push_back(v);
            if (v->type() == SSDfgNode::V_OUTPUT) io.push_back(v);
          } else {
            cnt++;
          }
        }
        int unique_io = vector_utils::count_unique(io);

        int cur_util = cnt + slot.num_passthroughs + unique_io;
        int cur_ovr = cur_util - v.node->max_util();
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
  for (int slot = 0; slot < 8; ++slot) {
    auto& lp = _linkProp[link->id()];
    int util = 0;

    std::vector<SSDfgVec*> vecs;
    std::vector<std::pair<SSDfgValue*, int>> values;

    for (auto it : lp.slots[slot].edges) {
      SSDfgEdge* edge = it.first;
      // std::cout << edge->name() << "\n";
      auto v = edge->def();
      auto d = edge->use();
      if (v->is_temporal() || d->is_temporal()) {
        if (auto input = dynamic_cast<SSDfgVecInput*>(v)) {
          vecs.push_back(input);
          continue;
        }
        if (auto* out = dynamic_cast<SSDfgVecOutput*>(d)) {
          vecs.push_back(out);
          continue;
        }
      } else {
        values.push_back(make_pair(edge->val(), edge->l()));
        // cout << edge->name() << " " << edge->val()->index() << "\n";
      }
    }
    // if(values.size() > 0) {
    //  cout << link->name() << " " << slot
    //    << " has " << lp.slots[slot].edges.size() << " edges and "
    //    << count_unique(values) << " values " << "\n";
    //}
    // if(count_unique(vecs) > 1) {
    //  cout << "slot:" << slot << " link:" << link->name() << "has " <<
    //  count_unique(vecs) << "vecs \n"; for(auto & i : vecs) {
    //    cout << i->name() << "\n";
    //  }
    //}
    util = vector_utils::count_unique(values) + vector_utils::count_unique(vecs);
    int cur_ovr = util - link->max_util();
    ovr = std::max(cur_ovr, ovr);
    agg_ovr += std::max(cur_ovr, 0);
    max_util = std::max(util, max_util);
  }
}

#include "./pass/reversed_topology.h"
#include "./pass/collect_redundancy.h"
#include "./pass/propagate_control.h"

Schedule::Schedule(SSModel* model, SSDfg* dfg) : _ssModel(model), _ssDFG(dfg) {
  allocate_space();
  reversed_topo = dsa::dfg::pass::ReversedTopology(dfg);
  needs_dynamic = dsa::dfg::pass::PropagateControl(reversed_topo);
  auto redundancy = dsa::dfg::pass::CollectRedundancy(dfg);
  operands = std::get<0>(redundancy);
  users = std::get<1>(redundancy);
}
