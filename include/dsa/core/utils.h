#include <string>

#include "json/json.h"

namespace dsa {
namespace core {
namespace utils {

/*!
 * \brief A helper wrapper for jsoncpp interfaces.
 * \param fname The file to be opened and loaded.
 */
Json::Value LoadJsonFromFile(const std::string &fname);

}
}
}
