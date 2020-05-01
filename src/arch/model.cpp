
#include <assert.h>
#include <math.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

#include "../utils/model_parsing.h"
#include "dsa/arch/ssinst.h"
#include "dsa/arch/model.h"
#include "dsa/debug.h"

#include "json.lex.h"
#include "json.tab.h"
#include "json/visitor.h"


using namespace std;
using namespace dsa;

SSModel::SSModel(SubModel* subModel) {
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
      if (value == string("INORDER")) {
        set_dispatch_inorder(true);
      } else if (value == string("OOO")) {
        set_dispatch_inorder(false);
      } else {
        assert(0 && "Dispatch was not INORDER or OOO");
      }
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
  bool isJSON = ModelParsing::EndsWith(filename, ".json");
  if (isJSON) {
    //dsa::SubModel * _subModel = new SubModel();
    _subModel -> parse_json(filename);
    //parse_json(ifs);
    return;
  }

  char line[512];

  while (ifs.good()) {
    ifs.getline(line, 512);
    // string line;

    if (ModelParsing::StartsWith(line, "[exec-model]")) {
      parse_exec(ifs);
    }

    if (ModelParsing::StartsWith(line, "[fu-model]")) {
      this->fu_types = ParseFuType(ifs);
    }

    if (ModelParsing::StartsWith(line, "[sub-model]")) {
      if (fu_types.empty()) {
        cerr << "No Fu Model Specified\n";
        exit(1);
      }
      _subModel = new SubModel(ifs, fu_types);
    }

    if (ModelParsing::StartsWith(line, "[io-model]")) {
      if (_subModel == nullptr) {
        cerr << "No Sub Model Specified\n";
        exit(1);
      }

      _subModel->parse_io(ifs);
    }
  }

  _subModel->post_process();
}

extern "C" void libssconfig_is_present() {}
