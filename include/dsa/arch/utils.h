#include <string>

#include "dsa/arch/fabric.h"

namespace dsa {
namespace adg {

/*!
 * \brief Load a JSON file and parse it to architecture description.
 * \param filename The name of the JSON file.
 * \return SpatialFabric* The parsed spatial architecture.
 */
SpatialFabric* Import(std::string filename);

}  // namespace adg
}  // namespace dsa