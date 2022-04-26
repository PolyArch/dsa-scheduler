#pragma once

#include <cassert>
#include <iostream>
#include <sstream>

#include "dsa/debug.h"

namespace dsa {
namespace dfg {

enum class OperandType { data, ctrl, self, ctrl_true, ctrl_false, local_reg, unknown };

constexpr char const* OPERAND_TYPE[] = {"data", "ctrl", "self",
                                        "pred", "inv_pred", "register", "<unknown>"};

struct MetaPort {
  enum class Data {
    Memory,
    SPad,
    LocalPort,
    RemotePort,
    Unknown,
  };

  static const char* DataText[(int)Data::Unknown];

  enum class Operation {
    Read,
    Write,
    IndRead,
    IndWrite,
    Atomic,
    Unknown,
  };

  static const char* OperationText[(int)Operation::Unknown];

  Data source, dest;

  // Operation
  int op;

  // # of concurrent instances
  int conc;

  // The coef of memory command penalty,
  // and the coef of memory reuse reward.
  double cmd{1.0}, repeat{1.0}, reuse{0.0};

  std::string dest_port;

  MetaPort() { clear(); }

  void clear() {
    source = Data::Unknown;
    dest = Data::Unknown;
    op = 0;
    conc = 0;
    cmd = 1.0;
    repeat = 1.0;
    dest_port = "";
  }

  void set(const std::string& key, const std::string& val) {
    bool success = false;
    if (key == "src" || key == "dest") {
      auto& ref = key == "src" ? source : dest;
      for (int i = 0; i < (int)Data::Unknown; ++i) {
        if (val == DataText[i]) {
          ref = (Data)i;
          success = true;
          break;
        }
      }
      if (!success) {
        dest_port = val;
        success = true;
      }
    } else if (key == "op") {
      for (int i = 0; i < (int)Operation::Unknown; ++i) {
        if (val == OperationText[i]) {
          op |= 1 << i;
          success = true;
          break;
        }
      }
    } else if (key == "conc") {
      std::istringstream iss(val);
      DSA_CHECK(iss >> conc) << "Cannot read: " << val;
      success = true;
    } else if (key == "repeat" || key == "cmd" || key == "reuse") {
      std::istringstream iss(val);
      if (key == "repeat") {
        iss >> repeat;
      } else {
        if (key == "cmd") {
          iss >> cmd;
        } else if (key == "reuse") {
          iss >> reuse;
        }
      }
      success = true;
    }
    DSA_CHECK(success) << key << " " << val;
  }

  void to_pragma(std::ostream& os) const {
    if (source != Data::Unknown) {
      os << "#pragma src=" << DataText[(int)source] << "\n";
    }
    if (dest != Data::Unknown) {
      os << "#pragma dest=" << DataText[(int)dest] << "\n";
    }
    if (!dest_port.empty()) {
      os << "#pragma dest=" << dest_port << "\n";
    }
    for (int i = 0; i < (int)Operation::Unknown; ++i) {
      if (op >> i & 1) {
        os << "#pragma op=" << OperationText[i] << "\n";
      }
    }
    if (conc) {
      os << "#pragma conc=" << conc << "\n";
    }
    os << "#pragma cmd=" << cmd << "\n";
    os << "#pragma repeat=" << repeat << "\n";
    os << "#pragma reuse=" << reuse << "\n";
  }
};

}  // namespace dfg
}  // namespace dsa
