#include "dsa/arch/spad.h"

namespace dsa{
namespace adg {

ScratchMemory::ScratchMemory(int line_size_, int num_banks_, int num_bytes_):
  num_bytes(num_bytes_), num_banks(num_banks_), bank_width(line_size_) {}

}
}