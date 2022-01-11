
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

namespace dsa {
namespace mapper {

bool Range::operator==(const Range &b) {
  return l == b.l && r == b.r;
}

}
}



int Schedule::colorOf(dsa::dfg::Value* v) { return cm::ColorOf(v); }

std::map<dsa::OpCode, int> Schedule::interpretConfigBits(int size, uint64_t* bits) {
  // Figure out if this configuration is real or not
  // NOTE: the first 9 characters of the configuration must spell filename
  // for this hack to work!
  DSA_CHECK(strncmp((char*)bits, "filename:", 9) == 0)
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
    auto loc = locationOf(nodes[i]);
    Json::Value mapping;
    mapping["op"] = "assign_node";
    mapping["dfgnode"] = nodes[i]->id();
    mapping["adgnode"] = loc.node()->id();
    mapping["adgslot"] = loc.lane();
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
  DSA_CHECK(ofs.good()) << "Cannot open " << mapping_filename;
}

// Write to a header file
void Schedule::printConfigHeader(ostream& os, std::string cfg_name, bool use_cheat) {
  DSA_INFO << cfg_name << ": bitstream is being generated";
  os << "#pragma once" << std::endl;
  // Header file for data type
  os <<"// Header File for Data Type" <<std::endl<< "#include <cstdint>" 
      << std::endl << std::endl;
  os << "// Input Vector Ports" <<std::endl ;
  for (auto& pv : _ssDFG->type_filter<dsa::dfg::InputPort>()) {
    int pn = vecPortOf(&pv);
    if (pv.name()[0] == '$') {
      continue;
    }
    if(pv.indirect()) {
      os << "#define P_" << cfg_name << "_" << pv.name() << "_in" << " " << (pn+NUM_IN_PORTS) << "\n";
    } else {
      os << "#define P_" << cfg_name << "_" << pv.name() << " " << pn << std::endl;
    }
  }

  os << "// Output Vector Ports" <<std::endl ;
  for (auto& pv : _ssDFG->type_filter<dsa::dfg::OutputPort>()) {
    int pn = vecPortOf(&pv);
    if (pv.name()[0] == '$') {
      continue;
    }
    os << "#define P_" << cfg_name << "_" << pv.name() << " " << pn << std::endl;
  }
  os << std::endl;

  if (!ContextFlags::Global().bitstream) {
    printConfigCheat(os, cfg_name);
  } else {
    // For each edge, find out the passthrough node
    int edge_idx = 0;
    SSDfg* ssDFG = ssdfg();
    auto &edge_list = ssDFG->edges;
    auto &vertex_list = ssDFG->nodes;

    // This is the collection of node configuration, size of #node, global node id needed
    std::vector<dsa::adg::bitstream::NodeInfo> info(this->ssModel()->subModel()->node_list().size());
    
    /////////////////////////////////////////////
    //////////// Loop over all edges ////////////
    /////////////////////////////////////////////

    for (auto& ep : _edgeProp) {

      // Traverse all edges to gather all node reconfiguration
      dsa::dfg::Edge* edge = &edge_list[edge_idx];
      os << "// -------- EDGE:" << edge_idx << ", extra_lat = " << ep.extra_lat
         << " -------- " << endl;

      // Loop over all physical links on this edge
      for(auto link_iter = ep.links.begin(); link_iter < ep.links.end(); link_iter++){
        // Get the current link and next link
        sslink* currLink = link_iter->second;
        sslink* nextLink;
        if(link_iter+1 != ep.links.end()){
          nextLink = (link_iter+1)->second;
        }

        // Get the source of current link, source of one link can be SW, FU and IVP
        ssswitch* sourceSwNode =  dynamic_cast<ssswitch*> (currLink->source());
        ssfu*     sourceFuNode =  dynamic_cast<ssfu*>     (currLink->source());
        ssvport*  sourceIVPNode = dynamic_cast<ssvport*>  (currLink->source());

        // Get the sink of current link, sink of one link can be SW, FU and OVP
        ssswitch* sinkSwNode =    dynamic_cast<ssswitch*>(currLink->sink());
        ssfu*     sinkFuNode =    dynamic_cast<ssfu*>(currLink->sink());
        ssvport*  sinkOVPNode =   dynamic_cast<ssvport*>(currLink->sink());
        
        ///////////////////////////////////////////////
        //// Source Input Vector Port --> CurrLink ////
        ///////////////////////////////////////////////
        
        // Set the physical port to order in stream routing
        
        if(sourceIVPNode != nullptr){
          // Get the order in stream
          int vid = edge->vid;
          // Get the physcial compute port index for this input vector port
          int pid = dsa::vector_utils::indexing(currLink,sourceIVPNode->out_links());
          // Check the pid and vid
          DSA_CHECK(vid >= 0) << "Vector Index cannot be negative";
          DSA_CHECK(pid >= 0) << "Physical Port Index cannot be negative";
          // Record the mapping
          // Please be attention: for IVP, this mapping is physical port to order in stream mapping
          // + 1:           zero maps to null
          // + vp_stated(): if vector port is stated, Lowest port is left for state stream
          info[sourceIVPNode->id()].vectorRoute[pid] = vid + 1 + sourceIVPNode->vp_stated();
          // Sanity check, stated edge should maps to stated vector port
          if(edge->sourceStated()){
            DSA_CHECK(sourceIVPNode->vp_stated()) << "Stated edge comes from non-stated vector port";
          }
          // Write comment to header file
          os << "//\tconfig " << sourceIVPNode->name() << endl
               << "//\t\troute value[" << vid << "] to vport[" << pid << "]" << endl;
        }

        ////////////////////////////////////
        //// Source Switch --> CurrLink ////
        ////////////////////////////////////
        
        // Nothing to be done: handled at sink switch 
        if(sourceSwNode != nullptr){}
        
        ///////////////////////////////////////////
        //// Source Function Unit --> CurrLink ////
        ///////////////////////////////////////////
        
        // Set the output routing for source FU

        // if current link points from fu, we should encode output routing info
        if(sourceFuNode != nullptr){
          // Get the fu id
          int fu_id = sourceFuNode->id();
          // Get the output port of result
          int output_port_idx = dsa::vector_utils::indexing(currLink, sourceFuNode->out_links());
          // Since we only produce one output, let use zero for now.
          info[fu_id].resultOutRoute[0] = output_port_idx;
          // Write comment to header file
          os << "//\tconfig " << sourceFuNode->name() << endl
               << "//\t\troute result 0 to output port " << output_port_idx << endl;
        }

        ///////////////////////////////////////////////
        //// CurrLink --> Sink Switch --> NextLink ////
        ///////////////////////////////////////////////
        
        // Set the routing configuration for Sink Switch
        
        // if current link points to switch
        if(sinkSwNode != nullptr){
          int sw_id = sinkSwNode->id();
          os << "//\tconfig " << sinkSwNode->name() << endl;
          int in_idx = dsa::vector_utils::indexing(currLink, sinkSwNode->in_links());
          DSA_CHECK(nextLink) << "Switch cannot be the end node of one edge, only OVP or PE can";
          int out_idx = dsa::vector_utils::indexing(nextLink, sinkSwNode->out_links());
          DSA_CHECK(in_idx >= 0);
          DSA_CHECK(out_idx >= 0);
          info[sw_id].outputRoute[out_idx] = (in_idx + 1); // + 1 since 0 means ground
          // os << "input size = " << sinkSwNode -> in_links().size()
          //   << ", output size = " << sinkSwNode -> out_links().size()<< endl;
          os << "//\t\troute input port " << in_idx << " to output port " << out_idx
             << endl;
        }
        
        /////////////////////////////////////////////
        //// --> CurrLink --> Sink Function Unit ////
        /////////////////////////////////////////////

        // Set operand selection
        // Set operand delay
        // Set function unit opcode
        // Set control entry
        
        // if current link points to fu, we should encode every info except
        // output routing here
        if (sinkFuNode != nullptr) {
          // the final destination is function unit
          int fu_id = sinkFuNode->id();
          // TODO: Sihao: no decomposability supported, so I just take first slot and first vertex
          auto* vertex = _nodeProp[fu_id].slots[0].vertices[0].first;
          // Found the index of this edge (the index of operand)
          int operandIdx = vector_utils::indexing(edge, operands[vertex->id()]);
          // Found the index of physical port that this edge maps to
          int input_port_idx = dsa::vector_utils::indexing(currLink, sinkFuNode->in_links());
          // Get Encode for Operands Routing
          DSA_CHECK(input_port_idx >= 0) << "not found input port";
          DSA_CHECK(operandIdx >= 0) << "This edge's destination is fu but not used?";
          {
            os << "//\tconfig " << sinkFuNode->name() << endl
               << "//\t\tadd extra delay " << ep.extra_lat << " for operand "
               << operandIdx << endl
               << "//\t\troute input port " << input_port_idx << " to operand "
               << operandIdx << endl;
            if(operandIdx < 2){
              info[fu_id].operandDelay[operandIdx] = ep.extra_lat;
              info[fu_id].operandRoute[operandIdx] = input_port_idx + 1; // + 1 since 0 means grounded
            }else{
              // Encode input control selection, if this is the third operand
              DSA_CHECK(operandIdx == 2);
              info[fu_id].inputCtrlRoute = input_port_idx + 1;
            }
          }
          // Encode the Opcode for PE
          auto* inst = dynamic_cast<dsa::dfg::Instruction*>(vertex);
          DSA_CHECK(inst) << "why a non-instruction node will be mapped to fu";
          int local_opcode = sinkFuNode->fu_type_.get_encoding(inst->inst());
          os << "//\t\tset current opcode to " << local_opcode << " means "
             << name_of_inst(inst->inst()) << endl;
          info[fu_id].opcode = local_opcode;
          // Encode the control mode
          int ctrlMode = 0;
          // DSA_INFO << "Resize the Control LUT for " << sinkFuNode->name() 
          // << " to " << sinkFuNode->ctrlLUTSize();
          info[fu_id].ctrlLUT.resize(sinkFuNode->ctrlLUTSize());
          DSA_CHECK(fu_id >= 0 && fu_id < info.size()) << "FU_ID = " << fu_id << " access out of range";
          if(!inst->predicate.encode().empty()){
            // Predication comes from other nodes, which means one of the input ports is used as control signal, which is input ctrl
            ctrlMode = 1;
            os << "//\t\tset the control mode to input-controlled mode" << endl;
            os << "//\t\tset bitmask of control bits of interested (BMSS) = " << inst->predicate.bmss << endl;
            info[fu_id].bmss = inst->predicate.bmss;
            // Encode control entry
            for(auto ctrlEntry = inst->predicate.lut.begin(); ctrlEntry != inst->predicate.lut.end(); ctrlEntry++){
              // Get the entry index that is not scrashed
              int entryIdx = ctrlEntry->first;
              // Convery entry index to LUT index
              int lutIdx = inst->predicate.entryIdx2lutIdx(entryIdx);
              DSA_CHECK(lutIdx >= 0 && lutIdx < info[fu_id].ctrlLUT.size()) << "LUT Index = " 
                << lutIdx << " access out of range";
              info[fu_id].ctrlLUT[lutIdx].valid = true;
              os << "//\t\tenable input control for " << lutIdx << "(th) control entry" << endl;
              for(auto ctrlBit : ctrlEntry->second){
                if(ctrlBit == 0){
                  os << "//\t\tenable the first operand reuse for control entry " << lutIdx <<endl;
                  info[fu_id].ctrlLUT[lutIdx].operand0Reuse = true;
                }else if(ctrlBit == 1){
                  os << "//\t\tenable the second operand reuse for control entry " << lutIdx <<endl;
                  info[fu_id].ctrlLUT[lutIdx].operand1Reuse = true;
                }else if(ctrlBit == 2){
                  os << "//\t\tenable the result discard for control entry " << lutIdx <<endl;
                  info[fu_id].ctrlLUT[lutIdx].resultDiscard = true;
                }else if(ctrlBit == 3){
                  os << "//\t\tenable the register (acc) reset for control entry " << lutIdx <<endl;
                  info[fu_id].ctrlLUT[lutIdx].registerReset = true;
                }else if(ctrlBit == 4){
                  os << "//\t\tenable operation abstain for control entry " << lutIdx <<endl;
                  info[fu_id].ctrlLUT[lutIdx].abstain = true;
                }
              }
            }
          }else if(!inst->self_predicate.encode().empty()){
            // Self Predication means the control signal is produced by itself, which means output (arithmetic output) controlled
            ctrlMode = 2;
            os << "//\t\tset the control mode to output-controlled mode" << endl;
            os << "//\t\tset bitmask of control bits of interested (BMSS) = " << inst->self_predicate.bmss << endl;
            info[fu_id].bmss = inst->self_predicate.bmss;
            // Encode control entry            
            for(auto ctrlEntry = inst->self_predicate.lut.begin(); ctrlEntry != inst->self_predicate.lut.end(); ctrlEntry++){
              // Control Entry Index is being used as key index for accessing the control LUT
              int entryIdx = ctrlEntry->first;
              // Convert entry index to LUT index by combining with BMSS
              int lutIdx = inst->self_predicate.entryIdx2lutIdx(entryIdx);
              DSA_CHECK(lutIdx >= 0 && lutIdx < info[fu_id].ctrlLUT.size()) << "LUT Index = " 
                << lutIdx << " access out of range";
              // Enable this control LUT
              os << "//\t\tenable output control for " << lutIdx << "(th) control entry" << endl;
              info[fu_id].ctrlLUT[lutIdx].valid = true;
              // The control signal type is enumeration
              for(auto ctrlBit : ctrlEntry->second){
                if(ctrlBit == 0){   
                  os << "//\t\tenable the first operand reuse for control entry " << lutIdx <<endl;
                  info[fu_id].ctrlLUT[lutIdx].operand0Reuse = true;
                }else if(ctrlBit == 1){
                  os << "//\t\tenable the second operand reuse for control entry " << lutIdx <<endl;
                  info[fu_id].ctrlLUT[lutIdx].operand1Reuse = true;
                }else if(ctrlBit == 2){
                  os << "//\t\tenable the result discard for control entry " << lutIdx <<endl;
                  info[fu_id].ctrlLUT[lutIdx].resultDiscard = true;
                }else if(ctrlBit == 3){
                  os << "//\t\tenable the register (acc) reset for control entry " << lutIdx <<endl;
                  info[fu_id].ctrlLUT[lutIdx].registerReset = true;
                }else if(ctrlBit == 4){
                  os << "//\t\tenable operation abstain for control entry " << lutIdx <<endl;
                  info[fu_id].ctrlLUT[lutIdx].abstain = true;
                }
              }
            }
          }
          // Set the control mode for this PE
          info[fu_id].ctrlMode = ctrlMode;
          // Encode the Destination Register
          for(int resultIdx = 0; resultIdx < inst->values.size(); resultIdx++){
            if(inst->values[resultIdx].reg != -1){
              int regIdx = inst->values[resultIdx].reg;
              os << "//\t\twrite "<< resultIdx << "th result to register " << regIdx <<endl;
              info[fu_id].resultRegRoute[resultIdx] = regIdx;
            }
          }
        }// End of encoding sink Function Unit

        //////////////////////////////////////////////
        //// CurrLink --> Sink Output Vector Port ////
        //////////////////////////////////////////////

        // Set the order in stream to physical port routing
        if(sinkOVPNode != nullptr){
          // Get the physical port index
          int pid = dsa::vector_utils::indexing(currLink,sinkOVPNode->in_links());
          // Get the output of DFG: an edge whose last link points to OVP means such edge must points to OutputPort (DFG)
          dsa::dfg::OutputPort* outNdoe = dynamic_cast<dsa::dfg::OutputPort*> (edge->use());
          // Get the source index of this edge
          int sourceIdx = outNdoe->sourceEdgeIdx(edge);
          // Make sure that each edge has a non-negative sourceIdx
          DSA_CHECK(sourceIdx >= 0) << "The order in stream for edge connected to OVP cannot be negative";
          DSA_CHECK(pid >= 0) << "The index of physical port cannot be negative";
          // Record the mapping, + 1 since zero means connected to ground
          // Please be attention: for OVP, this is order in stream to physical mapping
          info[sinkOVPNode->id()].vectorRoute[sourceIdx] = pid + 1;
          // Sanity check: stated edge
          if(edge->sourceStated()){
            // Stated edge should goes to a output port (software) which is penetrated
            DSA_CHECK(edge->sinkStated()) << "Source stated edge goes to output port, but not stated to output port";
            // Stated edge should goes to lowest position
            DSA_CHECK(sourceIdx == 0) << "Stated edge goes to output vector port, but stream position is not zero";
          }
          // Write comment to header file
          os << "//\tconfig " << sinkOVPNode->name() << endl
               << "//\t\troute vport[" << pid << "] to value[" << sourceIdx << "]" << endl;
        }
      }// Loop end for all links on this edge
      edge_idx++;
    }

    /////////////////////////////////////////////
    ////////////  Loop end all edges ////////////
    /////////////////////////////////////////////

    // Debug Print
    // DSA_INFO << "Number of node = " << ssModel()->subModel()->node_list().size();
    // DSA_INFO << "Start Counting the Configuration Words";

    // Count the configuration word
    int configWords = 0;
    for (auto& node : ssModel()->subModel()->node_list()) {
      dsa::adg::bitstream::BitstreamWriter bw(info[node->id()]);
      node->Accept(&bw);
      configWords += bw.configBitsVec.size();
    }
    os << endl;
    
    // Debug Print
    // DSA_INFO << "Finish Counting the Configuration Words";

    // Print the header file
    // TODO: this part can be merged together with above, we should not do bitstream generation twice
    // and the first one is just for counting the number of configuration word
    os << "#define " << cfg_name << "_size " << configWords << std::endl;
    os << "//\tTyp|Node_ID|G|Idx|-------------- Configuration Bits --------------" << std::endl;
    os << "uint64_t " << cfg_name << "_config[" << cfg_name << "_size] = {" << std::endl;
    for (auto& node : ssModel()->subModel()->node_list()) {
      dsa::adg::bitstream::BitstreamWriter bw(info[node->id()]);
      node->Accept(&bw);
      for(int cfgIdx = 0; cfgIdx < bw.configBitsVec.size(); cfgIdx ++){
        uint64_t config_bits = bw.configBitsVec[cfgIdx];
        std::bitset<64> b_config_bit(config_bits);
        os << "\t0b" << b_config_bit // Print the binary of config bitstream
          << ", //" << node->name() << " " << config_bits << std::endl;
      }
    }
    os << "};";
  }
  DSA_INFO << cfg_name << ": bitstream generated";
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
  std::ofstream ofs = ofstream(name);
  DSA_CHECK(ofs.good()) << name << " not opened!";

  dsa::SpatialFabric* sub = _ssModel->subModel();

  ofs << "digraph sched {\n";

  for (auto* elem : sub->input_list()) {
    printNodeGraphviz(ofs, elem);
  }

  for (auto* elem : sub->output_list()) printNodeGraphviz(ofs, elem);

  for (auto* elem : sub->switch_list()) printSwitchGraphviz(ofs, elem);

  for (auto* elem : sub->fu_list()) printNodeGraphviz(ofs, elem);

  for (ssnode* node : sub->node_list()) printMelGraphviz(ofs, node);

  ofs << "}\n\n";
  ofs.close();
}

void Schedule::stat_printOutputLatency() {
  int n = _ssDFG->type_filter<dsa::dfg::OutputPort>().size();
  cout << "** Output Vector Latencies **\n";
  for (int i = 0; i < n; i++) {
    auto* vec_out = &_ssDFG->type_filter<dsa::dfg::OutputPort>()[i];
    auto loc = locationOf(vec_out);
    ssvport* vport = dynamic_cast<ssvport*>(loc.node());
    DSA_CHECK(vport) << loc.node()->name();
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

bool Schedule::fixLatency(int& max_lat, int& max_lat_mis, std::pair<int, int>& delay_violation) {
  for (auto& i : _edgeProp) {
    i.extra_lat = 0;
  }

  max_lat = 0;
  max_lat_mis = 0;
  dsa::dfg::pass::IterativeLatency(this, max_lat, max_lat_mis, _totalViolation,
                                   _groupMismatch, false, delay_violation);
  this->_max_lat = max_lat;
  this->_max_lat_mis = max_lat_mis;

  return max_lat_mis == 0;
}

void Schedule::validate() {
  // Invariant: All paths should start at the source, and end at the
  // destination
  for (dsa::dfg::Edge& edge : _ssDFG->edges) {
    auto& links = _edgeProp[edge.id].links;
    auto def_node = locationOf(edge.def());
    auto use_node = locationOf(edge.use());

    if (links.size() == 0) continue;  // maybe a partial schedule

    int i = 0;
    sslink* prev_link = nullptr;
    for (auto& linkp : links) {
      sslink* link = linkp.second;
      if (i == 0) {
        DSA_CHECK(link->source() == def_node.node());
      }
      if (i > 0) {
        DSA_CHECK(prev_link->sink() == link->source());
      }
      if (i + 1 < (int)links.size()) {
        DSA_CHECK(dynamic_cast<ssvport*>(link->sink()) == 0);
      }
      ++i;
      prev_link = link;
    }
    DSA_CHECK(prev_link);
    DSA_CHECK(prev_link->sink() == use_node.node());
  }
}

void Schedule::get_overprov(int& ovr, int& agg_ovr, int& max_util) {
  ovr = 0;
  agg_ovr = 0;
  max_util = 0;

  for (auto v : _vertexProp) {
    if (v.node() != nullptr) {
      const auto& np = _nodeProp[v.node()->id()];

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
        int cur_ovr = cur_util - v.slot.ref->max_util();
        if (cur_ovr > 0) {
          DSA_LOG(OVERPROV) << v.slot.ref->name() << ": "
            << cnt << " + " << slot.passthrus.size() << " + "
            << unique_io << " + " << (ops.size() != 0) << " > " << v.slot.ref->max_util();
          for (auto elem: io) {
            DSA_LOG(OVERPROV) << elem->name();
          }
          for (auto elem: other) {
            DSA_LOG(OVERPROV) << elem->name();
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
      DSA_LOG(OVERPROV) << link->name() << ": " << values.size()
                    << " + " << vecs.size() << " > " << link->max_util();
      for (auto &value : values) {
        DSA_LOG(OVERPROV) << value.second << " " << value.first->name();
      }
      for (auto &vec : vecs) {
        DSA_LOG(OVERPROV) << vec->name();
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


Schedule::Schedule(const Schedule& s, SSModel* _model)
    : _ssModel(_model),
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
      _max_expected_route_latency(s._max_expected_route_latency),
      reversed_topo(s.reversed_topo),
      needs_dynamic(s.needs_dynamic),
      distances(s.distances),
      operands(s.operands),
      users(s.users),
      group_throughput(s.group_throughput),
      candidate_cnt(s.candidate_cnt) {
  // Shallow Copy DFG
  _ssDFG = s._ssDFG;
  for (int i = 0; i < dsa::dfg::Node::V_NUM_TYPES; ++i) {
    _num_mapped[i] = s._num_mapped[i];
  }
  swap_model(_model->subModel());
  set_model(_model);
  // We can just copy everything over and not have to normalize
  //normalize();
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
  std::vector<double> coef(dfg->meta.size(), (double)1.0);

  for (auto& elem : dfg->type_filter<dsa::dfg::InputPort>()) {
    if ((elem.meta.op >> (int)dsa::dfg::MetaPort::Operation::Read & 1) &&
        elem.meta.source != dsa::dfg::MetaPort::Data::Unknown) {
      bw[elem.group_id()][elem.meta.source == dsa::dfg::MetaPort::Data::SPad] +=
          (double) elem.vectorLanes() * elem.bitwidth() / 8;
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
  for (int i = 0; (long unsigned int) i < dfg->meta.size(); ++i) {
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
  for (int i = 0; (long unsigned int) i < dfg->meta.size(); ++i) {
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

  int fifo = violation_penalty.second;
  for (int i = 0; (long unsigned int) i < dfg->meta.size(); ++i) {
    double v =
        std::min(bw_coef[i], rec_hide[i] / rec_lat[i]) * inst_cnt[i] * nmlz_freq[i];
    DSA_LOG(ESTIMATION) << "[Group " << i << "] Freq: " << dfg->meta[i].frequency
                    << ", #Insts:" << inst_cnt[i] << ", Memory: " << bw[i][0]
                    << ", SPad: " << bw[i][1] << ", Rec: " << rec_hide[i] << "/"
                    << rec_lat[i] << ", Overall: " << v
                    << ", Performance Coef: " << coef[i];
    overall += v * coef[i] * ((double)fifo / (_groupMismatch[i] + fifo));
  }

  return overall;
}
