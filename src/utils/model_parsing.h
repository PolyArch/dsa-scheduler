#pragma once

// TODO(@were): Rename this to string utils. Better to make it oo implicit to a std::string.

#include <iostream>
#include <cstring>
#include <string>
#include <algorithm>
#include <vector>


namespace dsa {
namespace ModelParsing {

using namespace std;

inline bool StartsWith(const std::string& text, const std::string& token) {
  if (text.length() < token.length()) return false;
  return (text.compare(0, token.length(), token) == 0);
}

inline bool EndsWith(const std::string& text, const char* token) {
  if (text.length() < strlen(token)) return false;
  int len = strlen(token);
  return (text.compare(text.size() - strlen(token), len, token) == 0);
}

inline void trim_comments(std::string& s) { s = s.substr(0, s.find("#")); }

// This function reads line from an ifstream, and gets a param and value,
// seperated by a ":"
inline bool ReadPair(istream& is, string& param, string& value) {
  // char line[512];
  // is.getline(line,512);

  string line;
  getline(is, line);

  if (is.fail()) {
    param = "";
    value = "";
    return false;
  }

  trim_comments(line);

  std::stringstream ss(line);
  getline(ss, param, ':');
  getline(ss, value);
  return true;
}

// trim from start
inline void ltrim(std::string& s) {
  s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                  std::not1(std::ptr_fun<int, int>(std::isspace))));
}

// trim from end
inline void rtrim(std::string& s) {
  s.erase(
      std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace)))
          .base(),
      s.end());
}

// trim from both ends
inline void trim(std::string& s) {
  rtrim(s);
  ltrim(s);
}

inline bool stricmp(const std::string& str1, const std::string& str2) {
  if (str1.size() != str2.size()) {
    return false;
  }
  for (std::string::const_iterator c1 = str1.begin(), c2 = str2.begin(); c1 != str1.end();
       ++c1, ++c2) {
    if (tolower(*c1) != tolower(*c2)) {
      return false;
    }
  }
  return true;
}

inline void split(const std::string& s, const char delim,
                         std::vector<std::string>& elems) {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
}

inline bool is_number(const std::string& s) {
  double val;
  return sscanf(s.c_str(), "%lf", &val) == 1;
}

}
}  // namespace dsa
