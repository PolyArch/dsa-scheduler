#include <iostream>
#include <fstream>

#include "dsa/debug.h"
#include "dsa/core/utils.h"
#include "json/json.h"

namespace dsa {
namespace core {
namespace utils {

/*!
 * \brief A helper wrapper for jsoncpp interfaces.
 * \param fname The file to be opened and loaded.
 */
Json::Value LoadJsonFromFile(const std::string &fname) {
  Json::Value res;
  Json::CharReaderBuilder crb;
  std::ifstream ifs(fname);
  CHECK(ifs.good()) << "Cannot open " << fname;
  std::string errs;
  Json::parseFromStream(crb, ifs, &res, &errs);
  return res;
}

}
}
}

