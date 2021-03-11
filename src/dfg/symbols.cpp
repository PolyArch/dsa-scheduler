#include "dsa/dfg/symbols.h"
#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg{

OperandType Str2Flag(const std::string& s) {
  for (int i = 0; i < (int) OperandType::unknown; ++i) {
    if (OPERAND_TYPE[i] == s) {
      return static_cast<OperandType>(i);
    }
  }
  CHECK(false) << "Unknown Qualifier: " << s;
  throw;
}

ControlEntry::ControlEntry(const std::string& s, ParseResult* controller_)
    : controller(controller_), bits(0) {
  flag = Str2Flag(s);
}

ControlEntry::ControlEntry(const std::string& s,
            std::map<int, std::vector<std::string>>& bits_,
            ParseResult* controller_)
    : controller(controller_), bits(CtrlBits(bits_).bits()) {
  flag = Str2Flag(s);
}

TaskMapEntry::TaskMapEntry(ParseResult* controller_)
    : controller(controller_) {
  std::unordered_map<std::string, std::string> temp;
  port_map = temp;
}

TaskMapEntry::TaskMapEntry(
            std::unordered_map<std::string, std::string>& port_map_,
            ParseResult* controller_)
    : controller(controller_), port_map(TaskPortMap(port_map_).mapping()) {
}


}
}
