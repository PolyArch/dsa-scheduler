#pragma once
#include "dsa/dfg/ssdfg.h"

namespace dsa {
namespace dfg {

void Export(SSDfg *dfg, const std::string &fname);
SSDfg* Import(const std::string &fname);

}
}
