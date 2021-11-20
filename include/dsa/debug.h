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
namespace dsa {

class LOGGER {
  bool abort_;

 public:
  LOGGER(std::string reason, std::string file, int lineno, bool abort_) : abort_(abort_) {
    int i = file.size() - 1;
    while (i >= 0 && file[i] != '/') {
      --i;
    }
    DSA_OSTREAM << reason << file.substr(i + 1) << ":" << lineno << ": ";
  }

  ~LOGGER() noexcept(false) {
    DSA_OSTREAM << DSA_NEWLINE;
    if (abort_) {
      abort();
    }
  }

  template <typename T>
  inline LOGGER& operator<<(T&& x) {
    DSA_OSTREAM << std::forward<T>(x);
    return *this;
  }
};
}

#define DSA_CHECK(COND) \
  if (!(COND)) dsa::LOGGER("[CHECK FAIL]", __FILE__, __LINE__, true) << "(" << #COND << ") "

#define DSA_WARNING dsa::LOGGER("[WARNING]", __FILE__, __LINE__, false)

#define DSA_INFO dsa::LOGGER("[INFO]", __FILE__, __LINE__, false)

#define DSA_LOG(S) \
  if (getenv(#S)) dsa::LOGGER("[" #S "]", __FILE__, __LINE__, false)


#define ENFORCED_SYSTEM(CMD)                        \
  if (int ret = system(CMD))                        \
    dsa::LOGGER("[SHELL]", __FILE__, __LINE__, true) \
      << "Failed command: " << (CMD) << ", code" << ret
