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
    std::vector<int> ports[2];
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
    }
    void Visit(dfg::InputPort* vec) {
      ports[1].push_back(vec->bandwidth());
    }
    void Visit(dfg::OutputPort* vec) {
      ports[0].push_back(vec->bandwidth());
    }
  };

  struct SpatialCounter : dsa::adg::Visitor {
    std::map<std::pair<OpCode, int>, int> inst_exist;
    std::vector<int> ports[2];
    void Visit(ssfu* fu) override {
      for (auto elem : fu->fu_type_.capability) {
        if (fu->max_util() == 1) {
          inst_exist[{elem.op, 0}] += 64 / dsa::bitwidth[elem.op];
        } else {
          inst_exist[{elem.op, 1}] += fu->max_util();
        }
      }
    }
    void Visit(ssvport* vp) override {
      ports[vp->isInputPort()].push_back(vp->bitwidth_capability());
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

  for (int i = 0; i < 2; ++i) {
    if (dc.ports[i].size() > sc.ports[i].size()) {
      DSA_LOG(COUNT) << "In total, " << dc.ports[i].size()
                 << " port(s) are required, but only have " << sc.ports[i].size();
      return false;
    }
    sort(dc.ports[i].begin(), dc.ports[i].end(), std::greater<int>());
    sort(sc.ports[i].begin(), sc.ports[i].end(), std::greater<int>());
    for (int j = 0, n = dc.ports[i].size(); j < n; ++j) {
      if (dc.ports[i][j] > sc.ports[i][j]) {
        DSA_LOG(COUNT) << "A " << dc.ports[i][j] << "-wide port is required, but only have "
                   << sc.ports[i][j];
        return false;
      }
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
  string iter_dir = pdg_dir + "viz/iter/";
  string verif_dir = pdg_dir + "verif/";

  ENFORCED_SYSTEM(("mkdir -p " + viz_dir).c_str());
  ENFORCED_SYSTEM(("mkdir -p " + iter_dir).c_str());
  ENFORCED_SYSTEM(("mkdir -p " + verif_dir).c_str());

  std::string model_filename = model->filename;
  int lastindex = model_filename.find_last_of(".");
  string model_rawname = model_filename.substr(0, lastindex);
  string model_base =
      model_rawname.substr(model_rawname.find_last_of("\\/") + 1, model_rawname.size());
  int ovr = 0, agg_ovr = 0, max_util = 0;

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

    succeed_sched = schedule_timed(dfg, sched);

    int lat = 0, latmis = 0;

    if (dsa::ContextFlags::Global().verbose) {
      ovr = 0; agg_ovr = 0; max_util = 0;
      sched->get_overprov(ovr, agg_ovr, max_util);
      int violation = sched->violation();

      if (succeed_sched) {
        // Also check final latency
        cout << "latency: " << lat << "\n";
        cout << "lat-mismatch-max: " << latmis << "\n";
        cout << "lat-mismatch-sum: " << violation << "\n";
        cout << "overprov-max: " << ovr << "\n";
        cout << "overprov-sum: " << agg_ovr << "\n";
      } else {
        cout << "latency: " << -1 << "\n";
        cout << "latency mismatch: " << -1 << "\n";
        cout << "Scheduling Failed!\n";
      }
      sched->stat_printOutputLatency();
      dsa::mapper::pass::print_graphviz("viz/final.dot", dfg, sched);
    }

    dsa::mapper::pass::print_graphviz(viz_dir + dfg_base + ".dot", dfg);

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
  } else if (ovr > 0) {
    DSA_CHECK(false) << "Overprovision detected, try a smaller DFG!";
  } else {
    if(verbose) DSA_INFO << "Spatial scheduling finished!";
  }

  // Scheduling Finished
  std::string config_header = pdg_rawname + ".dfg.h";
  std::ofstream osh(config_header);
  DSA_CHECK(osh.good());
  sched->printConfigHeader(osh, dfg_base);
  if (dsa::ContextFlags::Global().verbose) {
    std::cout << "Performance: " << sched->estimated_performance() << std::endl;
  }

  if (dsa::ContextFlags::Global().bitstream) {
    std::string config_header_bits = pdg_rawname + ".dfg.bits.h";
    std::ofstream oshb(config_header_bits);
    DSA_CHECK(oshb.good());
    sched->printConfigHeader(oshb, dfg_base, true);
  }

  return succeed_sched ? 0 : 2;  // just to calm HEAPCHECK
}
