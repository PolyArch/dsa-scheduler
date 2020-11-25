#pragma once

#include <string>

#include "dsa/dfg/ssdfg.h"
#include "dsa/mapper/schedule.h"

namespace dsa {
namespace dfg {

/*!
 * \brief Dump the DFG in json format for simulation.
 * \param dfg The DFG to dump.
 * \param fname The json filename.
 */
void Export(SSDfg* dfg, const std::string& fname);

/*!
 * \brief Load the json into DFG data structure.
 * \param fname The filename to load
 * \return The DFG loaded.
 */
SSDfg* Import(const std::string& fname);

}  // namespace dfg
}  // namespace dsa
