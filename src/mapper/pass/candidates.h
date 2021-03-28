#pragma once
#include "dsa/mapper/schedule.h"

namespace dsa{
namespace mapper{

struct CandidateSpotVisitor : dfg::Visitor {

  void Visit(SSDfgInst *inst) override {
    auto fabric = sched->ssModel()->subModel();
    auto *model = fabric;
    std::vector<std::pair<int, ssnode*>> spots;
    std::vector<std::pair<int, ssnode*>> not_chosen_spots;

    std::vector<ssfu*> fus = model->nodes<dsa::ssfu*>();
    // For Dedicated-required Instructions
    for (size_t i = 0; i < fus.size(); ++i) {
      ssfu* cand_fu = fus[i];

      if (!cand_fu->fu_type_.Capable(inst->inst()) ||
          (cand_fu->out_links().size() < inst->values.size())) {
        continue;
      }

      if (!inst->is_temporal()) {
        if (sched->isPassthrough(0, cand_fu))  // FIXME -- this can't be right
          continue;
        // Normal Dedidated Instructions

        if (cand_fu->is_shared() && !spots.empty()) {
          continue;
        }

        for (int k = 0; k < 8; k += inst->bitwidth() / 8) {
          int cnt = 0;
          for (int sub_slot = k; sub_slot < k + inst->bitwidth() / 8; ++sub_slot) {
            cnt += sched->dfg_nodes_of(sub_slot, cand_fu).size();
          }
          cnt = cnt / 8 + 1;

          if (rand() % (cnt * cnt) == 0) {
            spots.emplace_back(k, fus[i]);
          } else {
            not_chosen_spots.emplace_back(k, fus[i]);
          }
        }

      } else if (cand_fu->is_shared()) {
        // For temporaly-shared instructions
        // For now the approach is to *not* consume dedicated resources, although
        // this can be changed later if that's helpful.
        if ((int)sched->dfg_nodes_of(0, cand_fu).size() + 1 < cand_fu->max_util()) {
          spots.emplace_back(0, fus[i]);
        } else {
          not_chosen_spots.emplace_back(0, fus[i]);
        }
      }
    }

    // If we couldn't find any good spots, we can just pick a bad spot for now
    if (spots.size() == 0) {
      spots = not_chosen_spots;
    }

    std::random_shuffle(spots.begin(), spots.end());  

    int n = spots.size();
    if (n > max_candidates)
      n = max_candidates;

    cnt[inst->id()] = n;
    candidates[inst->id()] = std::vector<std::pair<int, ssnode*>>(spots.begin(), spots.begin() + n);
  }

  // This is finding the candidates for input vector ports...
  // possible numbers...condition for non-inclusiveness is somewhere else that uses these candidates...
  void Visit(SSDfgVecInput *input) override {
    auto fabric = sched->ssModel()->subModel();
    auto vports = fabric->input_list();
    // Lets write size in units of bits
    std::vector<std::pair<int, ssnode*>> &spots = candidates[input->id()];
    spots.clear();
    for (size_t i = 0; i < vports.size(); ++i) {
      auto cand = vports[i];
      if ((int)cand->bitwidth_capability() >= input->phys_bitwidth()) {
        spots.push_back(make_pair(0, cand));
      }
    }
    cnt[input->id()] = spots.size();
  }

  // This is finding the candidates for output vector ports...
  void Visit(SSDfgVecOutput *output) override {
    auto fabric = sched->ssModel()->subModel();
    auto vports = fabric->output_list();
    // Lets write size in units of bits
    std::vector<std::pair<int, ssnode*>> &spots = candidates[output->id()];
    spots.clear();
    for (size_t i = 0; i < vports.size(); ++i) {
      auto cand = vports[i];
      if ((int)cand->bitwidth_capability() >= output->phys_bitwidth()) {
        spots.push_back(make_pair(0, cand));
      }
    }
    cnt[output->id()] = spots.size();
  }

  CandidateSpotVisitor(Schedule *sched_, int max_candidates_) :
    sched(sched_), max_candidates(max_candidates_), cnt(sched_->ssdfg()->nodes.size()),
    candidates(sched_->ssdfg()->nodes.size()) {}

  Schedule *sched{nullptr};
  int max_candidates;
  std::vector<int> cnt;
  std::vector<std::vector<std::pair<int, ssnode*>>> candidates;
};


}
}
