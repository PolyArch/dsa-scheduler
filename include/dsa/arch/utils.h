#pragma once

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

enum ADGKEY {
    #define MACRO(x, y) x,
    #include "./adg.ref"
    #undef MACRO  
};

/**
 * @brief This is the set for the name of the key in Architecture Description Graph
 * 
 */
const char* const ADGKEY_NAMES[]{
    #define MACRO(x, y) #y,
    #include "./adg.ref"
    #undef MACRO
};

}  // namespace adg
}  // namespace dsa