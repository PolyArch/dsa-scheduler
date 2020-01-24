#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <cstdlib>
#include <string>
#include <iostream>
#include <fstream>
#include <utility>

class Debug {

  bool logging{false};

 public:
  Debug(std::string s) : logging(static_cast<bool>(getenv(s.c_str()))) {}

  // TODO(@were): It cannot use std::endl for now.
  template<typename T>
  Debug &operator<<(T &&x) {
    if (logging) {
      std::cerr << std::forward<T>(x);
    }
    return *this;
  }

};

#define DEBUG(TYPE) Debug(#TYPE)

#endif
