#pragma once

#include <cstdint>

namespace dsa {
namespace adg {

/* \brief The static memory declaration. */
struct ScratchMemory {

  /* \brief The capacity of the memory. */
  int num_bytes;
  /* \brief The number of memory banks. */
  int num_banks;
  /* \brief The most fine-grain data type that can be accessed. */
  int bank_width;

  ScratchMemory(int line_size_, int num_banks_, int num_bytes_);

};

}
}