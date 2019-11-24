#ifndef __SS_METADATA_H__
#define __SS_METADATA_H__

#include <iostream>

namespace ssdfg {

struct MetaPort {

  enum class Data {
    Memory,
    SPad,
    LocalPort,
    RemotePort,
    Unknown,
  };

  static const char *DataText[(int) Data::Unknown];

  enum class Operation {
    Read,
    Write,
    IndRead,
    IndWrite,
    Atomic,
    Unknown,
  };

  static const char *OperationText[(int) Operation::Unknown];

  Data source, dest;

  // Operation
  int op;

  // # of instances
  int conc;

  // The coef of memory command penalty,
  // and the coef of memory reuse reward.
  double cmd{1.0}, repeat;

  std::string dest_port;

  MetaPort() { clear(); }

  void clear() {
    source = Data::Unknown;
    dest = Data::Unknown;
    op = 0;
    conc = 0;
    cmd = 1.0;
    repeat = 0.0;
    dest_port = "";
  }

  void set(const std::string &key, const std::string &val) {
    bool success = false;
    if (key == "src" || key == "dest") {
      auto &ref = key == "src" ? source : dest;
      for (int i = 0; i < (int) Data::Unknown; ++i) {
        if (val == DataText[i]) {
          ref = (Data) i;
          success = true;
          break;
        }
      }
      if (!success) {
        dest_port = val;
        success = true;
      }
    } else if (key == "op") {
      for (int i = 0; i < (int) Operation::Unknown; ++i) {
        if (val == OperationText[i]) {
          op |= 1 << i;
          success = true;
          break;
        }
      }
    } else if (key == "conc") {
      std::istringstream iss(val);
      assert(iss >> conc);
      success = true;
    } else if (key == "repeat" || key == "cmd") {
      std::istringstream iss(val);
      auto &ref = key == "repeat" ? repeat : cmd;
      iss >> ref;
      success = true;
    }
    if (!success) {
      std::cout << key << " " << val << std::endl;
      assert(false);
    }
  }

  void to_pragma(std::ostream &os) const {
    if (source != Data::Unknown) {
      os << "#pragma src=" << DataText[(int) source] << "\n";
    }
    if (dest != Data::Unknown) {
      os << "#pragma dest=" << DataText[(int) dest] << "\n";
    }
    if (!dest_port.empty()) {
      os << "#pragma dest=" << dest_port << "\n";
    }
    for (int i = 0; i < (int) Operation::Unknown; ++i) {
      if (op >> i & 1) {
        os << "#pragma op=" << OperationText[i] << "\n";
      }
    }
    if (conc) {
      os << "#pragma conc=" << conc << "\n";
    }
    if (repeat) {
      os << "#pragma repeat=" << repeat << "\n";
    }
  }

};

}

#endif // __SS_METADATA_H__
