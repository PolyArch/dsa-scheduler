#pragma once

#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#ifdef ENABLE_LLVM
#include "llvm/Support/raw_ostream.h"
#define DSA_OSTREAM llvm::errs()
#define DSA_NEWLINE "\n"
#else
#define DSA_OSTREAM std::cerr
#define DSA_NEWLINE std::endl
#endif

class DSA_LOGGER {
  bool abort_;

 public:
  DSA_LOGGER(std::string reason, std::string file, int lineno, bool abort_) : abort_(abort_) {
    int i = file.size() - 1;
    while (i >= 0 && file[i] != '/') {
      --i;
    }
    DSA_OSTREAM << reason << " " << file.substr(i + 1) << ":" << lineno << ": ";
  }

  ~DSA_LOGGER() noexcept(false) {
    DSA_OSTREAM << DSA_NEWLINE;
    if (abort_) {
      abort();
    }
  }

  template <typename T>
  inline DSA_LOGGER& operator<<(T&& x) {
    DSA_OSTREAM << std::forward<T>(x);
    return *this;
  }
};


#define CHECK(COND) \
  if (!(COND)) DSA_LOGGER("[CHECK FAIL]", __FILE__, __LINE__, true) << #COND << " "

#define DSA_WARNING DSA_LOGGER("[WARNING]", __FILE__, __LINE__, false)

#define DSA_INFO DSA_LOGGER("[INFO]", __FILE__, __LINE__, false)

#if defined(DEBUG_MODE) || defined(_DEBUG)
#define LOG(S) \
  if (getenv(#S)) DSA_LOGGER("[" #S "]", __FILE__, __LINE__, false)
#else
#define LOG(S) \
  if (false) DSA_LOGGER("[DEBUG]", __FILE__, __LINE__, false)
#endif

#define ENFORCED_SYSTEM(CMD)                        \
  if (int ret = system(CMD))                        \
    DSA_LOGGER("[SHELL]", __FILE__, __LINE__, true) \
      << "Failed command: " << (CMD) << ", code" << ret
