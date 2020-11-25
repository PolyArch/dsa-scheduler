#pragma once

#include <string>

namespace dsa {
namespace string_utils {

struct String {
  String(const std::string& s) : data(s) {}
  String(const char* s) : data(s) {}
  String() {}

  inline operator std::string&() { return data; }

  inline bool StartsWith(const String& s) const {
    return Guard(s) ? data.substr(0, s.Size()) == s.data : false;
  }

  inline int Size() const { return data.size(); }

  inline bool EndsWith(const String& s) const {
    return Guard(s) ? data.substr(data.size() - s.Size(), s.Size()) == s.data : false;
  }

  inline int Index(const String& s) const {
    if (!Guard(s)) return false;
    auto res = data.find(s.data);
    return res == std::string::npos ? -1 : (int)res;
  }

  inline std::vector<String> Split(char dlim) {
    std::vector<String> res;
    std::istringstream iss(data);
    std::string s;
    while (std::getline(iss, s, dlim)) {
      res.emplace_back(s);
    }
    return res;
  }

  inline String LStrip() const {
    std::string s(data);
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                    std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
  }

  inline String RStrip() const {
    std::string s(data);
    s.erase(std::find_if(s.rbegin(), s.rend(),
                         std::not1(std::ptr_fun<int, int>(std::isspace)))
                .base(),
            s.end());
    return s;
  }

  inline String Strip() const { return LStrip().RStrip(); }

 private:
  std::string data;

  inline bool Guard(const String& s) const { return s.Size() <= Size(); }
};

}  // namespace string_utils
}  // namespace dsa