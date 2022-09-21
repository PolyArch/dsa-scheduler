#include "dsa/arch/model.h"

#include <assert.h>
#include <math.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

#include "../utils/model_parsing.h"
#include "../utils/string_utils.h"
#include "dsa/arch/ssinst.h"
#include "dsa/debug.h"
#include "json/json.h"

using namespace std;
using namespace dsa;

SSModel::SSModel(SpatialFabric* subModel) {
  DSA_CHECK(subModel);
  _subModel = new SpatialFabric(*subModel);
}

void SSModel::setMaxEdgeDelay(int d) {
  for (auto* fu : _subModel->fu_list()) {
    fu->max_delay(d);
  }
}

void SSModel::parse_exec(std::istream& istream) {
  string param, value;
  while (istream.good()) {
    if (istream.peek() == '[') break;  // break out if done

    ModelParsing::ReadPair(istream, param, value);

    ModelParsing::trim(param);
    ModelParsing::trim(value);

    if (param.length() == 0) {
      continue;
    }

    if (param == string("CMD_DISPATCH")) {
      DSA_CHECK(value == "INORDER" || value == "OOO");
      set_dispatch_inorder(value == "INORDER");
    } else if (param == string("CMD_DISPATCH_WIDTH")) {
      istringstream(value) >> _dispatch_width;
    }
  }
}

// Sihao @ Jian: I found this function is never used, can we delete it?
ssnode* if_isVector(ssnode* n, std::string portname) {
  // In case source is a vector port, get I/O node instead
  SyncNode* s_vp = dynamic_cast<SyncNode*>(n);
  if (s_vp != nullptr)
    return s_vp->convert_port2node(portname);
  else {
    return n;
  }
}

void ParseCapabilities(Capability& fu, string& cap_string) {
  stringstream ss(cap_string);
  string cur_cap;

  while (getline(ss, cur_cap, ',')) {
    stringstream pss(cur_cap);
    string cap;
    string enc_str;

    getline(pss, cap, ':');

    ModelParsing::trim(cap);

    if (cap.empty()) {
      return;
    }

    if (ModelParsing::stricmp(cap, "ALL")) {
      for (int i = 0; i < SS_NUM_TYPES; ++i) {
        fu.Add((OpCode)i, 1);
      }
      return;
    }

    OpCode ss_inst = inst_from_string(cap.c_str());

    if (ss_inst == SS_NONE || ss_inst == SS_ERR) {
      continue;
    }

    DSA_CHECK(pss.good()) << "Opcode with no encoding!";
    unsigned encoding;
    pss >> encoding;
    fu.Add(ss_inst, encoding);
  }
}

std::vector<Capability*> ParseFuType(std::istream& istream) {
  std::vector<Capability*> res;

  string param, value;

  while (istream.good()) {
    if (istream.peek() == '[') break;  // break out if done

    // string line;
    ModelParsing::ReadPair(istream, param, value);

    if (param[0] == '#' || value[0] == '#') continue;  // Not a comment

    if (ModelParsing::StartsWith(param, "FU_TYPE")) {
      // defining an fu and capabilitty

      string newtype;

      std::stringstream ss(param);

      getline(ss, param, ' ');
      getline(ss, newtype);

      res.push_back(new Capability(newtype));
      ParseCapabilities(*res.back(), value);

    } else if (ModelParsing::StartsWith(param, "SWITCH_TYPE")) {
      DSA_CHECK(false);
    }
  }

  return res;
}

// File constructor
SSModel::SSModel(const char* filename_) : filename(filename_) {
  ifstream ifs(filename, ios::in);
  string param, value;
  DSA_CHECK(ifs.good()) << "Could Not Open: " << filename << "\n";

  // Parse the JSON-format IR
  if (string_utils::String(filename).EndsWith(".json")) {
    _subModel = dsa::adg::Import(filename_);
    for (auto fu : subModel()->fu_list()) {
      fu_types.push_back(new Capability(fu->fu_type()));
    }

    // TODO(@were): Deprecate this shit.
    {
      Json::CharReaderBuilder crb;
      std::ifstream ifs(filename);
      std::string errs;
      Json::Value* cgra = new Json::Value();
      Json::parseFromStream(crb, ifs, cgra, &errs);
      indirect((*cgra)["indirect"].asInt());
    }
    return;
  }

  string_utils::String line;

  while (ifs.good()) {
    std::getline(ifs, line.operator std::string&());

    if (line.StartsWith("[exec-model]")) {
      parse_exec(ifs);
    }

    if (line.StartsWith("[fu-model]")) {
      this->fu_types = ParseFuType(ifs);
    }

    if (line.StartsWith("[sub-model]")) {
      if (fu_types.empty()) {
        cerr << "No Fu Model Specified\n";
        exit(1);
      }
      _subModel = new SpatialFabric(ifs, fu_types);
    }

    if (line.StartsWith("[io-model]")) {
      if (_subModel == nullptr) {
        cerr << "No Sub Model Specified\n";
        exit(1);
      }

      _subModel->parse_io(ifs);
    }
  }
  _subModel->post_process();
  
}

void SSModel::extract_llvm_flags() {

  dsa::SSModel &Model = *this;

  {
    bool RecBusFound = false;
    bool IndirectFound = false;
    bool TemporalFound = false;
    int MostFineGrainFU = 64;
    for (auto *Elem : Model.subModel()->node_list()) {
      switch (Elem->type()) {
      case dsa::ssnode::NodeType::Recurrance: {
        RecBusFound = true;
        break;
      }
      case dsa::ssnode::NodeType::DirectMemoryAccess: {
        auto *DMA = dynamic_cast<dsa::ssdma*>(Elem);
        IndirectFound = IndirectFound || (DMA->indirectLength1DStream() || DMA->indirectIndexStream() || DMA->indirectStride2DStream());
        break;
      }
      case dsa::ssnode::NodeType::Scratchpad: {
        auto *SPM = dynamic_cast<dsa::ssscratchpad*>(Elem);
        IndirectFound = IndirectFound || (SPM->indirectIndexStream() || SPM->indirectLength1DStream() || SPM->indirectStride2DStream());
        break;
      }
      case dsa::ssnode::NodeType::FunctionUnit: {
        auto *FU = dynamic_cast<dsa::ssfu*>(Elem);
        TemporalFound = TemporalFound || FU->is_shared();
        MostFineGrainFU = std::min(MostFineGrainFU, FU->granularity());
        break;
      }
      default:
        break;
      }
    }
    std::ofstream ofs(".extracted-llvm-flags");

    // MC.REC = MC.REC && RecBusFound;
    ofs << "REC " << RecBusFound << std::endl;
    if (!RecBusFound) {
      DSA_WARNING << "Recurrence bus capability not found!";
    }
    // MC.IND = MC.IND && IndirectFound;
    ofs << "IND " << IndirectFound << std::endl;
    if (!IndirectFound) {
      DSA_WARNING << "Indirect memory capability not found!";
    }
    // MC.GRANULARITY = MostFineGrainFU;
    ofs << "FU-GRAN " << MostFineGrainFU << std::endl;
    DSA_WARNING << "ADG finest granularity is " << MostFineGrainFU;
  }
}
