#pragma once

#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#ifndef REPO_PREFIX
#define REPO_PREFIX ""
#endif

class LOGGER {
  bool abort;

 public:
  LOGGER(std::string reason, std::string file, int lineno, bool abort_) : abort(abort_) {
    std::cerr << reason << " " << file.substr(std::string(REPO_PREFIX).size() + 1) << ":"
              << lineno << ": ";
  }

  ~LOGGER() noexcept(false) {
    std::cerr << std::endl;
    if (abort) {
      __builtin_unreachable();
    }
  }

  template <typename T>
  inline LOGGER& operator<<(T&& x) {
    std::cerr << std::forward<T>(x);
    return *this;
  }
};

#define CHECK(COND) \
  if (!(COND)) LOGGER("[CHECK FAIL]", __FILE__, __LINE__, true) << #COND << " "

#define WARNING LOGGER("[WARNING]", __FILE__, __LINE__, false)

#ifdef DEBUG_MODE
#define LOG(S) \
  if (getenv(#S)) LOGGER("[" #S "]", __FILE__, __LINE__, false)
#else
#define LOG(S) \
  if (false) LOGGER("[DEBUG]", __FILE__, __LINE__, false)
#endif

#define ENFORCED_SYSTEM(CMD)                  \
  if (int ret = system(CMD))                  \
  LOGGER("[SHELL]", __FILE__, __LINE__, true) \
      << "Failed command: " << (CMD) << ", code" << ret
