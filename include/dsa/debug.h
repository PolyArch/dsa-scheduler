#pragma once

#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#ifdef ENABLE_LLVM
#include "llvm/Support/raw_ostream.h"
#define OSTREAM llvm::errs()
#define NEWLINE "\n"
#else
#define OSTREAM std::cerr
#define NEWLINE std::endl
#endif

class LOGGER {
  bool abort_;

 public:
  LOGGER(std::string reason, std::string file, int lineno, bool abort_) : abort_(abort_) {
    int i = file.size() - 1;
    while (i >= 0 && file[i] != '/') {
      --i;
    }
    OSTREAM << reason << " " << file.substr(i + 1) << ":" << lineno << ": ";
  }

  ~LOGGER() noexcept(false) {
    OSTREAM << NEWLINE;
    if (abort_) {
      abort();
    }
  }

  template <typename T>
  inline LOGGER& operator<<(T&& x) {
    OSTREAM << std::forward<T>(x);
    return *this;
  }
};

#define CHECK(COND) \
  if (!(COND)) LOGGER("[CHECK FAIL]", __FILE__, __LINE__, true) << #COND << " "

#define WARNING LOGGER("[WARNING]", __FILE__, __LINE__, false)

#define INFO LOGGER("[INFO]", __FILE__, __LINE__, false)

#if defined(DEBUG_MODE) || defined(_DEBUG)
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
