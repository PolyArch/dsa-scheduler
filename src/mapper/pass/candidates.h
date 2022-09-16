#pragma once
#include "dsa/arch/fu_model.h"
#include "dsa/debug.h"
#include "dsa/mapper/schedule.h"
#include <sstream>

namespace dsa {
namespace mapper {

using std::ostringstream;

inline bool inputCreep(int &currentBit, int &currentLink, dfg::InputPort* input, ssivport* cand) {
  // Return false if there isn't anymore links
  if (currentLink >= cand->out_links().size())
    return false;

  // Get the current link
  sslink* link = cand->out_links()[currentLink];

  // The number of lanes needed to map
  int lanes = input->vectorLanes();
  // The bitwidth of each lane
  int bitwidth = input->bitwidth();

  // Add the first stated edge
  if (cand->vp_stated()) {
    if (link->bitwidth() < 8) {
      return false;
    }
    currentLink++;
    currentBit = 0;
    if (currentLink >= cand->out_links().size()) {
      return false;
    }
    link = cand->out_links()[currentLink];
  }

  // Creep through all the lanes
  for (int currentLane = 0; currentLane < lanes; currentLane++) {
    while (link->bitwidth() < currentBit + bitwidth) {
      currentLink++;
      currentBit = 0;
      if (currentLink >= cand->out_links().size()) {
        return false;
      }
      link = cand->out_links()[currentLink];
    }
    currentBit += bitwidth;
  }
  // Now we are at the end and it can map
  return true;
}

inline bool outputCreep(int &currentBit, int &currentLink, dfg::OutputPort* output, ssovport* cand) {
  bool stated = output->penetrated_state >= 0;
  // Return false if there isn't anymore links
  if (currentLink >= cand->in_links().size())
    return false;

  // Get the current link
  sslink* link = cand->in_links()[currentLink];
  // The number of lanes needed to map
  int lanes = output->vectorLanes();
  // The bitwidth of each lane
  int bitwidth = output->bitwidth();

  // First add the stated edges if needed
  if (cand->vp_stated()) {
    if (link->bitwidth() < 8) {
      return false;
    }
    currentLink++;
    if (currentLink >= cand->in_links().size()) {
      return false;
    }
    link = cand->in_links()[currentLink];
  }


  // Creep through all the lanes
  for (int currentLane = 0; currentLane < lanes; currentLane++) {
    while (link->bitwidth() < currentBit + bitwidth) {
      currentLink++;
      currentBit = 0;
      if (currentLink >= cand->in_links().size()) {
        return false;
      }
      link = cand->in_links()[currentLink];
    }
    currentBit += bitwidth;
  }
  // Now we are at the end and it can map
  return true;
}



struct CandidateSpotVisitor : dfg::Visitor {

  void Visit(dfg::Instruction* inst) override {
    auto fabric = sched->ssModel()->subModel();
    auto* model = fabric;

    std::vector<MapSpot> spots;
    std::vector<MapSpot> secondary_spots;

    std::vector<ssfu*> fus = model->nodes<dsa::ssfu*>();
    // For Dedicated-required Instructions
    for (size_t i = 0; i < fus.size(); ++i) {
      ssfu* cand_fu = fus[i];

      if (!cand_fu->fu_type().Capable(inst->inst())) {
        ostringstream os;
        os <<  "Not capable!" << inst->name() << " " << cand_fu->name() << " [";

        int idx_inst = 0;
        int num_inst = cand_fu->fu_type().capability.size();
        for (auto& elem : cand_fu->fu_type().capability) {
          os << "\"" << dsa::name_of_inst(elem.op) << "\"";
          if (idx_inst < num_inst - 1) {
            os << ", ";
            idx_inst++;
          }
        }
        os << "]";
        //DSA_LOG(CAND) << os.str();

        continue;
      }

      if (!inst->is_temporal()) {
        if (sched->isPassthrough(0, cand_fu))  {// FIXME -- this can't be right
          DSA_LOG(CAND) << "Used as Passthrough";
          continue;
        }
        // Normal Dedidated Instructions

        if (cand_fu->is_shared() && !spots.empty()) {
          DSA_LOG(CAND) << "Shared FU: " << cand_fu->name() << " " << inst->name();
          continue;
        }

        if (inst->bitwidth() < cand_fu->granularity()) {
          DSA_LOG(CAND) << "Granularity greater than instruction bitwidth " << cand_fu->name() << " " << inst->name();
          continue;
        }

        if (inst->bitwidth() > cand_fu->datawidth()) {
          DSA_LOG(CAND) << "Instruction bitwidth grater than fu datawidth " << cand_fu->name() << " " << inst->name();
          continue;
        }

        bool routing_along = rand() & 1;

        for (int k = 0; k < cand_fu->datawidth(); k += cand_fu->granularity()) {
          if (k % inst->bitwidth() != 0) {
            continue;
          }

          if (k + inst->bitwidth() > cand_fu->datawidth()) {
            continue;
          }

          if (routing_along) {
            if (inst->lane() < 0) {
              routing_along = false;
            } else if (k / inst->bitwidth() != inst->lane() % cand_fu->lanes()) {
              continue;
            }
          }

          int cnt = 1;
          for (int sub_slot = 0; sub_slot < inst->bitwidth(); sub_slot += cand_fu->granularity()) {
            cnt += sched->dfg_nodes_of((sub_slot + k) / cand_fu->granularity(), cand_fu).size();
          }

          if (cnt == 1) {
            spots.emplace_back(routing_along, Slot<ssnode*>(k / cand_fu->granularity(), fus[i]));
          } else {
            secondary_spots.emplace_back(routing_along, Slot<ssnode*>(k / cand_fu->granularity(), fus[i]));
          }
        }

      } else if (cand_fu->is_shared()) {
        // For temporaly-shared instructions
        // For now the approach is to *not* consume dedicated resources, although
        // this can be changed later if that's helpful.
        if ((int)sched->dfg_nodes_of(0, cand_fu).size() + 1 < cand_fu->max_util()) {
          spots.emplace_back(0, Slot<ssnode*>(0, fus[i]));
        } else {
          secondary_spots.emplace_back(0, Slot<ssnode*>(0, fus[i]));
        }
      }
    }

    // If we couldn't find any good spots, we can just pick a bad spot for now
    if (spots.empty()) {
      spots = secondary_spots;
    }

    std::random_shuffle(spots.begin(), spots.end());

    int n = spots.size();
    //if (n > max_candidates) n = max_candidates;

    cnt[inst->id()] = n;
    candidates[inst->id()] = spots;
  }

  void Visit(dfg::Operation* op) override {
    auto fabric = sched->ssModel()->subModel();
    for (int i = 0, n = fabric->fu_list().size(); i < n; ++i) {
      auto* fu = fabric->fu_list()[i];
      auto fu_type = fu->fu_type();
      auto& capability = fu_type.capability;
      std::vector<int> cnt(fu->fu_type().capability.size(), 0);
      for (int j = 0, m = capability.size(); j < m; ++j) {
        cnt[j] = capability[j].count;
      }
      // TODO(@were): For now, I assume that decomposability cannot happen on Operation
      // mapping.
      //              We actually need better data structure representation on
      //              decomposability, but I do not have a clear plan in my brain.
      // TODO(@were): For now, I assume all the instruction on this node should be
      // spatially shared.
      //              We later need a attribute on the adg node to indicate the sharing
      //              strategy.
      // TODO(@were): For the purpose of performance, we move this cnt to a schedule
      // redundant data structure.
      for (auto elem : sched->dfg_nodes_of(0, fabric->fu_list()[i])) {
        if (auto tmp = dynamic_cast<dfg::Operation*>(elem.first)) {
          for (int j = 0, m = tmp->opcodes.size(); j < m; ++j) {
            auto opcode = tmp->opcodes[j];
            auto iter = std::find_if(
                capability.begin(), capability.end(),
                [opcode](const Capability::Entry& entry) { return entry.op == opcode; });
            DSA_CHECK(iter != capability.end());
            cnt[iter - capability.begin()] -= tmp->cnt[j];
          }
        }
      }
      bool ok = true;
      for (int j = 0, m = op->opcodes.size(); j < m; ++j) {
        auto opcode = op->opcodes[j];
        auto iter = std::find_if(
            capability.begin(), capability.end(),
            [opcode](const Capability::Entry& entry) { return entry.op == opcode; });
        if (iter == capability.end()) {
          ok = false;
          break;
        }
        if (cnt[iter - capability.begin()] < op->cnt[j]) {
          ok = false;
          break;
        }
      }
      
      fu->fu_type(fu_type);
      if (ok) {
        candidates[op->id()].emplace_back(0, Slot<ssnode*>(0, static_cast<ssnode*>(fu)));
        ++this->cnt[op->id()];
      }
    }
  }
  
  /**
   * @brief Finds Candidate Hardware Vector Ports for a given Input Port
   * 
   * Restrictions:
   * A Stated Input Port must map to a stated Hardware Vector Port
   * The Hardware Vector Port must be capable of handling the Input Port
   * 
   * @param input the given software input port
   */
  void Visit(dfg::InputPort* input) override {
    auto fabric = sched->ssModel()->subModel();
    auto vports = fabric->input_list();
    // Lets write size in units of bits
    std::vector<MapSpot>& spots = candidates[input->id()];
    std::vector<MapSpot> bad;
    spots.clear();
    for (size_t i = 0; i < vports.size(); ++i) {
      auto cand = vports[i];
      // Multiple inputs can't map to the same hardware vector port
      if (!sched->node_prop()[cand->id()].slots[0].vertices.empty())
        continue;
      // Stated inputs must map to stated hardware vector ports
      if (input->stated && !cand->vp_stated())
        continue;

      int currentBit  = 0;
      int currentLink = 0;
      
      if (inputCreep(currentBit, currentLink, input, cand)) {
        spots.emplace_back(0, Slot<ssnode*>(0, static_cast<ssnode*>(cand)));
      }
    }
    
    if (spots.empty()) {
      spots = bad;
      if (input->stated) {
        DSA_LOG(WARNING) << input->bandwidth() << "-wide stated input port insufficient!";
      } else {
        DSA_LOG(WARNING) << input->bandwidth() << "-wide input port insufficient!";
      }
    }

    cnt[input->id()] = spots.size();
  }

  /**
   * @brief Finds Candidate Hardware Vector Ports for a given Output Port
   * 
   * Restrictions:
   * A Stated Output Port must map to a stated Hardware Vector Port
   * The Hardware Vector Port must be capable of handling the Output Port
   * 
   * @param output the given software output port
   */
  void Visit(dfg::OutputPort* output) override {
    auto fabric = sched->ssModel()->subModel();
    auto vports = fabric->output_list();
    // Lets write size in units of bits
    std::vector<MapSpot>& spots = candidates[output->id()];
    std::vector<MapSpot> bad;
    spots.clear();
    for (size_t i = 0; i < vports.size(); ++i) {
      auto cand = vports[i];
      // We can't have multiple outputs on the same vport
      if (!sched->node_prop()[cand->id()].slots[0].vertices.empty())
        continue;
      // Stated output must be mapped to a stated vport
      if ((output->penetrated_state >= 0) && !cand->vp_stated())
        continue;
      
      int currentBit  = 0;
      int currentLink = 0;
      if (outputCreep(currentBit, currentLink, output, cand)) {
        spots.emplace_back(0, Slot<ssnode*>(0, static_cast<ssnode*>(cand)));
      }
    }
    
    if (spots.empty()) {
      spots = bad;
      if (output->penetrated_state >= 0) {
        DSA_LOG(WARNING) << output->bandwidth() << "-wide stated input port insufficient!";
      } else {
        DSA_LOG(WARNING) << output->bandwidth() << "-wide input port insufficient!";
      }
    }

    cnt[output->id()] = spots.size();
  }

  void Visit(dfg::DMA* dma) override {
    auto fabric = sched->ssModel()->subModel();
    auto dma_nodes = fabric->dma_list();
    // Lets write size in units of bits
    std::vector<MapSpot>& spots = candidates[dma->id()];
    std::vector<MapSpot> bad;
    spots.clear();
    for (size_t i = 0; i < dma_nodes.size(); ++i) {
      auto cand = dma_nodes[i];
      if (cand->capacity() >= dma->size()) {
        spots.emplace_back(0, Slot<ssnode*>(0, cand));
      }
    }

    if (spots.empty()) {
      spots = bad;
      DSA_LOG(WARNING) << dma->size() << "KB DMA insufficient!";
    }

    cnt[dma->id()] = spots.size();
  }

  void Visit(dfg::Scratchpad* spm) override {
    auto fabric = sched->ssModel()->subModel();
    auto spm_nodes = fabric->scratch_list();
    // Lets write size in units of bits
    std::vector<MapSpot>& spots = candidates[spm->id()];
    std::vector<MapSpot> bad;
    spots.clear();
    for (size_t i = 0; i < spm_nodes.size(); ++i) {
      auto cand = spm_nodes[i];
      if (cand->capacity() >= spm->size()) {
        DSA_LOG(CAND) << "Adding candidate " << cand->name() << " to spm " << spm->name();
        spots.emplace_back(0, Slot<ssnode*>(0, cand));
      }
    }

    if (spots.empty()) {
      spots = bad;
      DSA_LOG(WARNING) << spm->size() << "KB ScratchPad insufficient!";
    }

    cnt[spm->id()] = spots.size();
  }

  void Visit(dfg::Register* reg) override {
    auto fabric = sched->ssModel()->subModel();
    auto reg_nodes = fabric->reg_list();
    // Lets write size in units of bits
    std::vector<MapSpot>& spots = candidates[reg->id()];
    std::vector<MapSpot> bad;
    spots.clear();
    for (size_t i = 0; i < reg_nodes.size(); ++i) {
      auto cand = reg_nodes[i];
      if (cand->capacity() >= reg->size()) {
        DSA_LOG(CAND) << "Adding candidate " << cand->name() << " to reg " << reg->name();
        spots.emplace_back(0, Slot<ssnode*>(0, cand));
      }
    }

    if (spots.empty()) {
      spots = bad;
      DSA_LOG(WARNING) << reg->size() << "KB Register Node insufficient!";
    }

    cnt[reg->id()] = spots.size();
  }

  void Visit(dfg::Recurrance* rec) override {
    auto fabric = sched->ssModel()->subModel();
    auto rec_nodes = fabric->recur_list();
    // Lets write size in units of bits
    std::vector<MapSpot>& spots = candidates[rec->id()];
    std::vector<MapSpot> bad;
    spots.clear();
    for (size_t i = 0; i < rec_nodes.size(); ++i) {
      auto cand = rec_nodes[i];
      if (cand->capacity() >= rec->size()) {
        DSA_LOG(CAND) << "Adding candidate " << cand->name() << " to rec " << rec->name();
        spots.emplace_back(0, Slot<ssnode*>(0, cand));
      }
    }

    if (spots.empty()) {
      spots = bad;
      DSA_LOG(WARNING) << rec->size() << "KB Recurrance Node insufficient!";
    }

    cnt[rec->id()] = spots.size();
  }

  void Visit(dfg::Generate* gen) override {
    auto fabric = sched->ssModel()->subModel();
    auto gen_nodes = fabric->gen_list();
    // Lets write size in units of bits
    std::vector<MapSpot>& spots = candidates[gen->id()];
    std::vector<MapSpot> bad;
    spots.clear();
    for (size_t i = 0; i < gen_nodes.size(); ++i) {
      auto cand = gen_nodes[i];
      if (cand->capacity() >= gen->size()) {
        DSA_LOG(CAND) << "Adding candidate " << cand->name() << " to gen " << gen->name();
        spots.emplace_back(0, Slot<ssnode*>(0, cand));
      }
    }

    if (spots.empty()) {
      spots = bad;
      DSA_LOG(WARNING) << gen->size() << "KB Generate Node insufficient!";
    }

    cnt[gen->id()] = spots.size();
  }

  CandidateSpotVisitor(Schedule* sched_, int max_candidates_)
      : sched(sched_),
        max_candidates(max_candidates_),
        cnt(sched_->ssdfg()->nodes.size()),
        candidates(sched_->ssdfg()->nodes.size()) {}

  Schedule* sched{nullptr};
  int max_candidates;
  std::vector<int> cnt;
  std::vector<std::vector<MapSpot>> candidates;
};

struct IsCandidateVisitor : dfg::Visitor {

  void Visit(dfg::Instruction* inst) override {
    if (spot.node()->type() == ssnode::NodeType::FunctionUnit) {
      ssfu* fu = dynamic_cast<ssfu*>(spot.node());
      if (!sched->node_prop()[fu->id()].slots[0].vertices.empty()) 
        return;

      // Check to make sure instruction works
      if (!fu->fu_type().Capable(inst->inst())) {
        return;
      }

      if (sched->isPassthrough(0, fu))  {
        return;
      }

      if (inst->bitwidth() < fu->granularity()) {
        return;
      }

      if (inst->bitwidth() > fu->datawidth()) {
        return;
      }

      if (spot.lane() % inst->bitwidth() != 0) {
        return;
      }

      if (spot.lane() + inst->bitwidth() > fu->datawidth()) {
        return;
      }

      is_candidate = true;
    }
  }

  void Visit(dfg::Operation* op) override {
    if (spot.node()->type() == ssnode::NodeType::FunctionUnit) {
      is_candidate = true;
    } 
  }
  
  void Visit(dfg::InputPort* input) override {
    if (spot.node()->type() == ssnode::NodeType::InputVectorPort) {
      ssivport* ivport = dynamic_cast<ssivport*>(spot.node());
      
      // We can't have multiple inputs on the same vport
      if (!sched->node_prop()[ivport->id()].slots[0].vertices.empty())
        return;
      
      // Stated input must be mapped to a stated vport
      if (input->stated && !ivport->vp_stated())
        return;
      
      int currentBit  = 0;
      int currentLink = 0;
      if (inputCreep(currentBit, currentLink, input, ivport)) {
        is_candidate = true;
      }
    } 
  }

  void Visit(dfg::OutputPort* output) override {
    if (spot.node()->type() == ssnode::NodeType::OutputVectorPort) {
      ssovport* ovport = dynamic_cast<ssovport*>(spot.node());
      
      // We can't have multiple outputs on the same vport
      if (!sched->node_prop()[ovport->id()].slots[0].vertices.empty())
        return;
      
      // Stated output must be mapped to a stated vport
      if ((output->penetrated_state >= 0) && !ovport->vp_stated())
        return;
      
      int currentBit  = 0;
      int currentLink = 0;
      if (outputCreep(currentBit, currentLink, output, ovport)) {
        is_candidate = true;
      }
    } 
  }

  void Visit(dfg::DMA* dma) override {
    if (spot.node()->type() == ssnode::NodeType::DirectMemoryAccess) {
      ssdma* dma_node = dynamic_cast<ssdma*>(spot.node());
      if (dma_node->capacity() >= dma->size()) {
        is_candidate = true;
      }
    } 
  }

  void Visit(dfg::Scratchpad* spm) override {
    if (spot.node()->type() == ssnode::NodeType::Scratchpad) {
      ssscratchpad* spm_node = dynamic_cast<ssscratchpad*>(spot.node());
      if (spm_node->capacity() >= spm->size()) {
        is_candidate = true;
      }
    }
  }

  void Visit(dfg::Register* reg) override {
    if (spot.node()->type() == ssnode::NodeType::Register) {
      ssregister* reg_node = dynamic_cast<ssregister*>(spot.node());
      if (reg_node->capacity() >= reg->size()) {
        is_candidate = true;
      }
    }
  }

  void Visit(dfg::Recurrance* rec) override {
    if (spot.node()->type() == ssnode::NodeType::Recurrance) {
      ssrecurrence* rec_node = dynamic_cast<ssrecurrence*>(spot.node());
      if (rec_node->capacity() >= rec->size()) {
        is_candidate = true;
      }
    }
  }

  void Visit(dfg::Generate* gen) override {
    if (spot.node()->type() == ssnode::NodeType::Generate) {
      ssgenerate* gen_node = dynamic_cast<ssgenerate*>(spot.node());
      if (gen_node->capacity() >= gen->size()) {
        is_candidate = true;
      }
    }
  }

  IsCandidateVisitor(Schedule* sched_, MapSpot spot_)
      : sched(sched_),
        spot(spot_) {}

  Schedule* sched{nullptr};
  MapSpot spot;
  bool is_candidate{false};
};

inline bool is_candidate(Schedule* sched, dfg::Node* node, MapSpot spot) {
  IsCandidateVisitor visitor(sched, spot);
  node->Accept(&visitor);
  return visitor.is_candidate;
}

}  // namespace mapper
}  // namespace dsa
