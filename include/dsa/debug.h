#pragma once

#include <cassert>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <map>
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

class EnvCache {
  public:
    const bool &get_env(const std::string &key) {
      auto it = cache_entries.find(key);
      if(it == cache_entries.end()) {
        const char *ptr = getenv(key.c_str());
        it = cache_entries.insert({key, (bool) (ptr)}).first;
      }
      return it->second;
    }

    void clear() {
      cache_entries.clear();
    }

    ~EnvCache() {
      clear();
    }

  private:
      std::map<std::string, bool> cache_entries;
};

static EnvCache ENV_CACHE;

class LOGGER {
  bool abort_;

 public:
  LOGGER(std::string reason, std::string file, int lineno, bool abort_) : abort_(abort_)
#ifdef ENABLE_LLVM
    , message(s)
#endif
  {
    int i = file.size() - 1;
    while (i >= 0 && file[i] != '/') {
      --i;
    }
    message << reason << file.substr(i + 1) << ":" << lineno << ": ";
  }

  ~LOGGER() noexcept(false) {
    message << DSA_NEWLINE;
    DSA_OSTREAM << message.str();

    if (abort_) {
      abort();
    }
  }

  template <typename T>
  inline LOGGER& operator<<(T&& x) {
    message << std::forward<T>(x);
    return *this;
  }
 private:
#ifdef ENABLE_LLVM
  llvm::raw_string_ostream message;
  std::string s;
#else
  std::ostringstream message;
#endif
};
}

#define DSA_CHECK(COND) \
  if (!(COND)) dsa::LOGGER("[CHECK FAIL]", __FILE__, __LINE__, true) << "(" << #COND << ") "

#define DSA_WARNING dsa::LOGGER("[WARNING]", __FILE__, __LINE__, false)

#define DSA_INFO dsa::LOGGER("[INFO]", __FILE__, __LINE__, false)

#define DSA_LOG(S) \
  if (dsa::ENV_CACHE.get_env(#S))  \
    dsa::LOGGER("[" #S "]", __FILE__, __LINE__, false)


#define ENFORCED_SYSTEM(CMD)                        \
  if (int ret = system(CMD))                        \
    dsa::LOGGER("[SHELL]", __FILE__, __LINE__, true) \
      << "Failed command: " << (CMD) << ", code" << ret
