#include "dsa/mapper/scheduler.h"

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <fstream>
#include <list>
#include <sstream>
#include <unordered_map>

#include "./pass/print_graphviz.h"
#include "./pass/print_json.h"
#include "dsa/arch/visitor.h"
#include "dsa/core/singleton.h"
#include "dsa/debug.h"
#include "dsa/dfg/instruction.h"
#include "dsa/dfg/visitor.h"
#include "dsa/mapper/scheduler_sa.h"

using namespace dsa;
using namespace std;

bool Scheduler::check_feasible(SSDfg* ssDFG, SSModel* ssmodel) {
  struct DFGCounter : dsa::dfg::Visitor {
    std::map<std::pair<OpCode, int>, int> inst_required;
    std::vector<int> inst_bitwidths;
    std::vector<int> ports[2];
    std::vector<int> spms;
    std::vector<int> dmas;
    std::vector<int> recs;
    std::vector<int> regs;
    std::vector<int> gens;

    void Visit(dfg::Instruction* node) {
      inst_required[{node->inst(), (int)node->is_temporal()}]++;
      for (int i = 0; i < node->ops().size(); ++i) {
        auto &operand = node->ops()[i];
        if (node->ops()[i].type != dfg::OperandType::data) {
          continue;
        }
        if (!operand.edges.empty()) {
          int operand_width = 0;
          for (auto eid : operand.edges) {
            operand_width += node->ssdfg()->edges[eid].bitwidth();
          }
          DSA_CHECK(operand_width == node->bitwidth())
            << node->name() << "'s operand " << i << " dtype mismatch! "
            << operand_width << " != " << node->bitwidth();
        }
      }
      inst_bitwidths.push_back(node->bitwidth());
    }
    void Visit(dfg::InputPort* vec) {
      ports[1].push_back(vec->bandwidth());
    }
    void Visit(dfg::OutputPort* vec) {
      ports[0].push_back(vec->bandwidth());
    }
    void Visit(dfg::Scratchpad* spm) {
      spms.push_back(spm->size());
    }
    void Visit(dfg::DMA* dma) {
      dmas.push_back(dma->size());
    }
    void Visit(dfg::Recurrance* rec) {
      recs.push_back(rec->size());
    }
    void Visit(dfg::Register* reg) {
      regs.push_back(reg->size());
    }
    void Visit(dfg::Generate* gen) {
      gens.push_back(gen->size());
    }
  };

  struct SpatialCounter : dsa::adg::Visitor {
    std::map<std::pair<OpCode, int>, int> inst_exist;
    std::vector<int> fu_granularities;
    std::vector<int> ports[2];
    std::vector<int64_t> spms;
    std::vector<int64_t> dmas;
    std::vector<int> recs;
    std::vector<int> regs;
    std::vector<int> gens;
    void Visit(ssfu* fu) override {
      for (auto elem : fu->fu_type().capability) {
        if (fu->max_util() == 1) {
          inst_exist[{elem.op, 0}] += 64 / dsa::bitwidth[elem.op];
        } else {
          inst_exist[{elem.op, 1}] += fu->max_util();
        }
      }
      fu_granularities.push_back(fu->granularity());
    }
    void Visit(ssivport* vp) override {
      ports[1].push_back(vp->bitwidth_capability());
    }

    void Visit(ssovport* vp) override {
      ports[0].push_back(vp->bitwidth_capability());
    }

    void Visit(ssscratchpad* spm) override {
      spms.push_back(spm->capacity());
    }
    
    void Visit(ssdma* dma) override {
      dmas.push_back(dma->capacity());
    }
    
    void Visit(ssregister* reg) override {
      regs.push_back(reg->capacity());
    }

    void Visit(ssgenerate* gen) override {
      gens.push_back(gen->capacity());
    }

    void Visit(ssrecurrence* rec) override {
      recs.push_back(rec->capacity());
    }

  };

  DFGCounter dc;
  SpatialCounter sc;
  ssDFG->Apply(&dc);
  _ssModel->subModel()->Apply(&sc);

  for (auto elem : dc.inst_required) {
    if (sc.inst_exist[elem.first] < elem.second) {
      DSA_LOG(COUNT) << elem.second << " " << name_of_inst(elem.first.first)
                 << " FU(s) are required in "
                 << (elem.first.second ? "temporal" : "dedicated") << " tiles, but only "
                 << sc.inst_exist[elem.first] << " found";
      return false;
    }
  }

  if (!dc.inst_bitwidths.empty()) {
    sort(dc.inst_bitwidths.begin(), dc.inst_bitwidths.end());
    sort(sc.fu_granularities.begin(), sc.fu_granularities.end(), std::greater<int>());
    if (dc.inst_bitwidths[0] < sc.fu_granularities[0]) {
      DSA_LOG(COUNT) << "The smallest instruction bitwidth is " << dc.inst_bitwidths[0]
                     << " bits, but the smallest FU granularity is "
                     << sc.fu_granularities[0] << " bits";
      return false;
    }
  }

  sort(dc.ports[0].begin(), dc.ports[0].end(), std::greater<int>());
  sort(sc.ports[0].begin(), sc.ports[0].end(), std::greater<int>());

  if (dc.ports[0].size() > sc.ports[0].size()) {
    DSA_LOG(COUNT) << "In total, " << dc.ports[0].size()
                << " output port(s) are required, but only have " << sc.ports[0].size();
    return false;
  }

  sort(dc.ports[0].begin(), dc.ports[0].end(), std::greater<int>());
  sort(sc.ports[0].begin(), sc.ports[0].end(), std::greater<int>());
  for (int j = 0, n = dc.ports[0].size(); j < n; ++j) {
    if (dc.ports[0][j] > sc.ports[0][j]) {
      DSA_LOG(COUNT) << "A " << dc.ports[0][j] << "-wide output port is required, but only have "
                  << sc.ports[0][j];
      return false;
    }
  }

  if (dc.ports[1].size() > sc.ports[1].size()) {
    DSA_LOG(COUNT) << "In total, " << dc.ports[1].size()
                << " input port(s) are required, but only have " << sc.ports[1].size();
    return false;
  }

  sort(dc.ports[1].begin(), dc.ports[1].end(), std::greater<int>());
  sort(sc.ports[1].begin(), sc.ports[1].end(), std::greater<int>());
  for (int j = 0, n = dc.ports[1].size(); j < n; ++j) {
    if (dc.ports[1][j] > sc.ports[1][j]) {
      DSA_LOG(COUNT) << "A " << dc.ports[1][j] << "-wide input port is required, but only have "
                  << sc.ports[1][j];
      return false;
    }
  }
  

  sort(dc.spms.begin(), dc.spms.end(), std::greater<int>());
  sort(sc.spms.begin(), sc.spms.end(), std::greater<int64_t>());

  int total_sw_spm = accumulate(dc.spms.begin(), dc.spms.end(), 0);
  int total_hw_spm = accumulate(sc.spms.begin(), sc.spms.end(), 0);
  
  if (total_sw_spm > total_hw_spm) {
    DSA_LOG(COUNT) << "In total, " << total_sw_spm
                << " kbs in scratchpad are required, but only have " << total_hw_spm;
    return false;
  }
  
  if (dc.spms.size() > 0) {
    if (sc.spms.size() > 0) {
      if (dc.spms[0] > sc.spms[0]) {
        DSA_LOG(COUNT) << "A " << dc.spms[0] << "-wide SPM is required for scratchpad with " << sc.spms[0] << " capacity";
        return false;
      }
    } else {
      DSA_LOG(COUNT) << "A " << dc.spms[0] << "-wide SPM is required for scratchpad";
      return false;
    }
  }
  

  int64_t total_sw_dma = accumulate(dc.dmas.begin(), dc.dmas.end(), 0);
  int64_t total_hw_dma = accumulate(sc.dmas.begin(), sc.dmas.end(), (int64_t) 0);
  
  if (total_sw_dma > total_hw_dma) {
    DSA_LOG(COUNT) << "In total, " << total_sw_dma
                << " kbs in DMA are required, but only have " << total_hw_dma;
    return false;
  }

  if (dc.dmas.size() > 0) {
    if (sc.dmas.size() > 0) {
      if (dc.dmas[0] > sc.dmas[0]) {
        DSA_LOG(COUNT) << "A " << dc.dmas[0] << "-wide DMA is required, but only have " << sc.dmas[0];
        return false;
      }
    } else {
      DSA_LOG(COUNT) << "A " << dc.dmas[0] << "-wide DMA is required for dma";
      return false;
    }
  }

  sort(dc.recs.begin(), dc.recs.end(), std::greater<int>());
  sort(sc.recs.begin(), sc.recs.end(), std::greater<int>());

  int total_sw_rec = accumulate(dc.recs.begin(), dc.recs.end(), 0);
  int total_hw_rec = accumulate(sc.recs.begin(), sc.recs.end(), 0);

  if (total_sw_rec > total_hw_rec) {
    DSA_LOG(COUNT) << "In total, " << total_sw_rec
                << " kbs in Recurrance are required, but only have " << total_hw_rec;
    return false;
  }

  if (dc.recs.size() > 0) {
    if (sc.recs.size() > 0) {
      if (dc.recs[0] > sc.recs[0]) {
        DSA_LOG(COUNT) << "A " << dc.recs[0] << "-wide Recurrance Node is required, but only have " << sc.recs[0];
        return false;
      }
    } else {
      DSA_LOG(COUNT) << "A " << dc.recs[0] << "-wide Recurrance Node is required for recurrance";
      return false;
    }
  }

  sort(dc.regs.begin(), dc.regs.end(), std::greater<int>());
  sort(sc.regs.begin(), sc.regs.end(), std::greater<int>());

  int total_sw_reg = accumulate(dc.regs.begin(), dc.regs.end(), 0);
  int total_hw_reg = accumulate(sc.regs.begin(), sc.regs.end(), 0);
  
  if (total_sw_reg > total_hw_reg) {
    DSA_LOG(COUNT) << "In total, " << total_sw_reg
                << " kbs in Registers are required, but only have " << total_hw_reg;
    return false;
  }

  if (dc.regs.size() > 0) {
    if (sc.regs.size() > 0) {   
      if (dc.regs[0] > sc.regs[0]) {
        DSA_LOG(COUNT) << "A " << dc.regs[0] << "-wide Register Node is required, but only have " << sc.regs[0];
        return false;
      }
    } else {
      DSA_LOG(COUNT) << "A " << dc.regs[0] << "-wide Register Node is required for register";
      return false;
    }
  }

  sort(dc.gens.begin(), dc.gens.end(), std::greater<int>());
  sort(sc.gens.begin(), sc.gens.end(), std::greater<int>());

  int total_sw_gen = accumulate(dc.gens.begin(), dc.gens.end(), 0);
  int total_hw_gen = accumulate(sc.gens.begin(), sc.gens.end(), 0);

  if (total_sw_gen > total_hw_gen) {
    DSA_LOG(COUNT) << "In total, " << total_sw_gen
                << " kbs in Generate are required, but only have " << total_hw_gen;
    return false;
  }

  if (dc.gens.size() > 0) {
    if (sc.gens.size() > 0) {   
      if (dc.gens[0] > sc.gens[0]) {
        DSA_LOG(COUNT) << "A " << dc.gens[0] << "-wide Generate Node is required, but only have " << sc.gens[0];
        return false;
      }
    } else {
      DSA_LOG(COUNT) << "A " << dc.gens[0] << "-wide Register Node is required for generate";
      return false;
    }
  }

  return true;
}

std::string basename(const std::string& filename) {
  size_t lastindex = filename.find_last_of(".");
  string res = filename.substr(0, lastindex);

  lastindex = filename.find_last_of("\\/");
  if (lastindex != string::npos) {
    res = res.substr(lastindex + 1);
  }
  return res;
}

std::string basedir(const std::string& filename) {
  size_t lastindex = filename.find_last_of("\\/");
  if (lastindex == string::npos) {
    return std::string("./");
  }
  return filename.substr(0, lastindex);
}

int Scheduler::invoke(SSModel* model, SSDfg* dfg) {
  bool succeed_sched = false;
  Schedule* sched = nullptr;

  // the name without preceeding dirs or file extension
  bool verbose = dsa::ContextFlags::Global().verbose;
  string dfg_base = basename(dfg->filename);
  string pdg_dir = basedir(dfg->filename);  // preceeding directories only
  if (pdg_dir[pdg_dir.length() - 1] != '\\' || pdg_dir[pdg_dir.length() - 1] != '/') {
    pdg_dir += "/";
  }
  string viz_dir = pdg_dir + "viz/";
  string iter_dir = pdg_dir + "viz/iters/";
  string verif_dir = pdg_dir + "verif/";

  ENFORCED_SYSTEM(("mkdir -p " + viz_dir).c_str());
  ENFORCED_SYSTEM(("mkdir -p " + iter_dir).c_str());
  ENFORCED_SYSTEM(("mkdir -p " + verif_dir).c_str());

  std::string model_filename = model->filename;
  int lastindex = model_filename.find_last_of(".");
  string model_rawname = model_filename.substr(0, lastindex);
  string model_base =
      model_rawname.substr(model_rawname.find_last_of("\\/") + 1, model_rawname.size());
    
  int64_t ovr = 0, agg_ovr = 0, max_util = 0;

  if (check_feasible(dfg, model)) {
    auto sigint_handler = [](int) { exit(1); };
    if (verbose) {
      DSA_INFO << "DFG=" << dfg_base << ", ADG=" << model_base;
      DSA_INFO << "Feasibility checked, scheduling ... ";
    }

    // Setup signal so we can stop if we need to
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = sigint_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    int lat = 0, latmis = 0;
    succeed_sched = schedule_timed(dfg, sched);
    sched->get_overprov(ovr, agg_ovr, max_util);
    int violation = sched->violation();

    if (dsa::ContextFlags::Global().verbose) {

      if (succeed_sched) {
        // Also check final latency
        DSA_INFO << "latency: " << lat;
        DSA_INFO << "lat-mismatch-max: " << latmis;
        DSA_INFO << "lat-mismatch-sum: " << violation;
        DSA_INFO << "overprov-max: " << ovr;
        DSA_INFO << "overprov-sum: " << agg_ovr;
        sched->stat_printOutputLatency();
      } else {
        DSA_INFO << "latency: " << -1;
        DSA_INFO << "latency mismatch: " << -1;
        DSA_INFO << "Scheduling Failed!\n";
      }
      dsa::mapper::pass::print_graphviz("viz/final.dot", dfg, sched);
      dsa::adg::print_sched_json("viz/final-sched_adg.json", model->subModel(), sched);
    }

    dsa::mapper::pass::print_graphviz(viz_dir + "/" + dfg_base + ".dot", dfg);
    DSA_INFO << "Printed " << viz_dir << "/" << dfg_base << ".dot";

    std::string sched_viz = viz_dir + dfg_base + "." + model_base + ".gv";
    ostringstream os;
    os << "viz/" << model_base << "-hw.json";
    model->subModel()->DumpHwInJson(os.str().c_str());
    sched->printGraphviz(sched_viz.c_str());
    DSA_INFO << "Printing Edges:";
    sched->printEdge();

    std::string verif_header = verif_dir + dfg_base + ".configbits";
    std::ofstream vsh(verif_header);
    DSA_CHECK(vsh.good());
    sched->printConfigVerif(vsh);
  }

  lastindex = dfg->filename.find_last_of(".");
  string pdg_rawname = dfg->filename.substr(0, lastindex);

  if (!succeed_sched || sched == nullptr) {
    DSA_CHECK(false) << "Cannot be scheduled, try a smaller DFG!";
  } else if (ovr > 0 && ContextFlags().bitstream) {
    DSA_CHECK(false) << "Overprovision detected, try a smaller DFG!";
  } else {
    if (ovr > 0) DSA_INFO << "Overprovision detected! Ovr-Max: " << ovr << " Ovr-Sum: " << agg_ovr;
    if(verbose) DSA_INFO << "Spatial scheduling finished!";
  }

  // Scheduling Finished
  std::string config_header = pdg_rawname + ".dfg.h";
  std::ofstream osh(config_header);
  DSA_CHECK(osh.good());
  sched->printConfigHeader(osh, dfg_base);
  if (dsa::ContextFlags::Global().verbose) {
    std::string spm_performance = "";
    std::string l2_performance = "";
    std::string dram_performance = "";
    DSA_INFO << "Performance: " << sched->estimated_performance(spm_performance, l2_performance, dram_performance);
    DSA_INFO << "  SPM Bandwidth: " << spm_performance;
    DSA_INFO << "  L2 Bandwidth: " << l2_performance;
    DSA_INFO << "  DRAM Bandwidth: " << dram_performance;
  }

  if (dsa::ContextFlags::Global().bitstream) {
    std::string config_header_bits = pdg_rawname + ".dfg.bits.h";
    std::ofstream oshb(config_header_bits);
    DSA_CHECK(oshb.good());
    sched->printConfigHeader(oshb, dfg_base, true);
  }

  return succeed_sched ? 0 : 2;  // just to calm HEAPCHECK
}
