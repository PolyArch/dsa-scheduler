#include <sstream>
#include <map>
#include <functional>

#include "dsa/arch/ssinst.h"
#include "../utils/string_utils.h"

namespace dsa {
namespace adg {
namespace pe_utils {

using string_utils::String;

enum class Operation { Add, Mul, Div, Cmp, Shift, Special, Unknown };

struct InstInfo {
  Operation opcode{Operation::Unknown};
  bool is_float{false};
  int bits{-1};
  int lanes{1};

  InstInfo(OpCode opcode_) {
    String opname(name_of_inst(opcode_));
    DEBUG(STR) << std::string(opname) << " " << opname.StartsWith("F") << " " << !opname.StartsWith("Fx");
    if (opname.StartsWith("F") && !opname.StartsWith("Fx")) {
      is_float = true;
    }
    auto f = [&opname, this](const String &s, Operation op) {
      if (opname.Index(s) != -1) {
        opcode = op;
      }
    };
    f("Add", Operation::Add);
    f("Sub", Operation::Add);
    f("Acc", Operation::Add);
    f("Div", Operation::Div);
    f("Mul", Operation::Mul);
    f("Cmp", Operation::Cmp);
    f("Shl", Operation::Shift);
    f("Shr", Operation::Shift);
    for (int i = 3; i <= 6; ++i) {
      // TODO(@were): Is it good to assume the highest bitwidth is 64?
      for (int j = 0; (1 << j) * (1 << i) <= 64; ++j) {
        std::ostringstream oss;
        oss << (1 << i);
        if (j) oss << "x" << (1 << j);
        if (opname.EndsWith(oss.str())) {
          lanes = j;
          bits = i;
          return;
        }
      }
    }
  }

};

auto MAX = [](double a, double b) { return std::max(a, b); };

std::map<std::pair<Operation, int>, std::function<double(double, double)>> COMBINER = {
  {{Operation::Add, false}, MAX},
  {{Operation::Mul, false}, MAX},
  {{Operation::Cmp, false}, MAX},
  {{Operation::Shift, false}, MAX},
};

}
}
}