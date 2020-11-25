
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
#include "json.lex.h"
#include "json.tab.h"
#include "json/visitor.h"

using namespace std;
using namespace dsa;

SSModel::SSModel(SpatialFabric* subModel) {
  CHECK(subModel);
  _subModel = subModel;
}

void SSModel::setMaxEdgeDelay(int d) {
  for (auto* fu : _subModel->fu_list()) {
    fu->set_delay_fifo_depth(d);
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
      CHECK(value == "INORDER" || value == "OOO");
      set_dispatch_inorder(value == "INORDER");
    } else if (param == string("CMD_DISPATCH_WIDTH")) {
      istringstream(value) >> _dispatch_width;
    }
  }
}

ssnode* if_isVector(ssnode* n, std::string portname) {
  // In case source is a vector port, get I/O node instead
  ssvport* s_vp = dynamic_cast<ssvport*>(n);
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
        fu.Add((OpCode)i, i);
      }
      return;
    }

    OpCode ss_inst = inst_from_string(cap.c_str());

    if (ss_inst == SS_NONE || ss_inst == SS_ERR) {
      continue;
    }

    CHECK(pss.good()) << "Opcode with no encoding!";
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
      CHECK(false);
    }
  }

  return res;
}

// File constructor
SSModel::SSModel(const char* filename_) : filename(filename_) {
  ifstream ifs(filename, ios::in);
  string param, value;
  bool failed_read = ifs.fail();
  if (failed_read) {
    cerr << "Could Not Open: " << filename << "\n";
    return;
  }

  // Parse the JSON-format IR
  if (string_utils::String(filename).EndsWith(".json")) {
    _subModel = new SpatialFabric();
    _subModel->parse_json(filename);
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
