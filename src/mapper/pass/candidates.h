#pragma once
#include "dsa/arch/fu_model.h"
#include "dsa/debug.h"
#include "dsa/mapper/schedule.h"
#include <sstream>

namespace dsa {
namespace mapper {

using std::ostringstream;

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

      if (!cand_fu->fu_type_.Capable(inst->inst())) {
        ostringstream os;
        os <<  "Not capable!" << inst->name() << " " << cand_fu->name() << " [";

        int idx_inst = 0;
        int num_inst = cand_fu->fu_type_.capability.size();
        for (auto& elem : cand_fu->fu_type_.capability) {
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

        bool routing_along = rand() & 1;

        for (int k = 0; k < cand_fu->datawidth(); k += cand_fu->granularity()) {
          if (k % inst->bitwidth() != 0) {
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
    if (n > max_candidates) n = max_candidates;

    cnt[inst->id()] = n;
    candidates[inst->id()] = spots;
  }

  void Visit(dfg::Operation* op) override {
    auto fabric = sched->ssModel()->subModel();
    for (int i = 0, n = fabric->fu_list().size(); i < n; ++i) {
      auto* fu = fabric->fu_list()[i];
      auto& capability = fu->fu_type_.capability;
      std::vector<int> cnt(fu->fu_type_.capability.size(), 0);
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

      if (cand->bitwidth_capability() >= input->bandwidth()) {
        // Ensure that we are not mapping a stated software port to a non-stated hardware port
        if (!input->stated || cand->vp_stated()) {
          if (sched->node_prop()[cand->id()].slots[0].vertices.empty()) {
            spots.emplace_back(0, Slot<ssnode*>(0, cand));
          } else {
            spots.emplace_back(0, Slot<ssnode*>(0, cand));
          }
        }
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

      if (cand->bitwidth_capability() >= output->bandwidth()) {
        // Get whether this output-port is stated
        bool stated = output->penetrated_state >= 0;

        // Make sure we are not mapping a stated software port to a non-stated hardware vport
        if (!stated || cand->vp_stated()) {
          if (sched->node_prop()[cand->id()].slots[0].vertices.empty()) {
            spots.emplace_back(0, Slot<ssnode*>(0, cand));
          } else {
            bad.emplace_back(0, Slot<ssnode*>(0, cand));
          }
        }
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

}  // namespace mapper
}  // namespace dsa
