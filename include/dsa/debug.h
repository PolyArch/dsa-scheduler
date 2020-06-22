#pragma once

#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

class LOGGER {

  bool abort;

 public:
  LOGGER(std::string reason, std::string file, int lineno, bool abort) : abort(abort) {
    std::cerr << reason << " " << file << ":" << lineno << ": ";
  }

  ~LOGGER() noexcept(false) {
    std::cerr << std::endl;
    if (abort) {
      // TODO(@were): This is great for debugging backtrace but confuses user when reading the logs.
      throw;
    }
  }

  template <typename T>
  inline LOGGER& operator<<(T&& x) {
    std::cerr << std::forward<T>(x);
    return *this;
  }
};

#undef DEBUG
#undef CHECK
#undef ENFORCED_SYSTEM
#define DEBUG(S) if (getenv(#S)) LOGGER("[DEBUG]", __FILE__, __LINE__, false)
#define CHECK(COND) if (!(COND)) LOGGER("[CHECK FAIL]", __FILE__, __LINE__, true) << #COND
#define ENFORCED_SYSTEM(CMD)                                          \
  if (int ret = system(CMD))                                          \
    LOGGER("[SHELL]", __FILE__, __LINE__, true) << "Failed command: " \
                                                << (CMD) << ", code" << ret
