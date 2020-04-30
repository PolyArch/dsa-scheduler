//This file generated from inst_model.cpp -- Do not edit.  Do not commit to repo.
#ifndef __SS_INST_H__
#define __SS_INST_H__

#include <string>
#include <string.h>
#include <cstring>
#include <assert.h>
#include <iostream>
#include <vector>
#include <complex>
#include <algorithm>
#include <xmmintrin.h>
#include <math.h>
#include "fixed_point.h"

using std::complex;

namespace SS_CONFIG {

float    as_float(std::uint32_t ui);
uint32_t as_uint32(float f);

double    as_double(std::uint64_t ui);
uint64_t as_uint64(double f);

std::complex<float> as_float_complex(uint64_t ui);
uint64_t as_uint64(const std::complex<float> &val);

enum OpCode {
SS_NONE=0,
SS_ERR,
  SS_Copy, 
  SS_LShf8x8, 
  SS_RShf8x8, 
  SS_AndR8x8, 
  SS_OrR8x8, 
  SS_XorR8x8, 
  SS_Cat8x8, 
  SS_Acc8x8, 
  SS_Add8x8, 
  SS_Sub8x8, 
  SS_Mul8x8, 
  SS_Div8x8, 
  SS_Max8x8, 
  SS_Min8x8, 
  SS_Mod8x8, 
  SS_And8x8, 
  SS_Nand8x8, 
  SS_Or8x8, 
  SS_Nor8x8, 
  SS_Xor8x8, 
  SS_Not8x8, 
  SS_CmpEQ8x8, 
  SS_CmpNE8x8, 
  SS_CmpGE8x8, 
  SS_CmpGT8x8, 
  SS_CmpLE8x8, 
  SS_CmpLT8x8, 
  SS_LShf16x4, 
  SS_RShf16x4, 
  SS_AndR16x4, 
  SS_OrR16x4, 
  SS_XorR16x4, 
  SS_Cat16x4, 
  SS_Acc16x4, 
  SS_Add16x4, 
  SS_Sub16x4, 
  SS_Mul16x4, 
  SS_Div16x4, 
  SS_Max16x4, 
  SS_Min16x4, 
  SS_Mod16x4, 
  SS_And16x4, 
  SS_Nand16x4, 
  SS_Or16x4, 
  SS_Nor16x4, 
  SS_Xor16x4, 
  SS_Not16x4, 
  SS_CmpEQ16x4, 
  SS_CmpNE16x4, 
  SS_CmpGE16x4, 
  SS_CmpGT16x4, 
  SS_CmpLE16x4, 
  SS_CmpLT16x4, 
  SS_LShf32x2, 
  SS_RShf32x2, 
  SS_AndR32x2, 
  SS_OrR32x2, 
  SS_XorR32x2, 
  SS_Cat32x2, 
  SS_Acc32x2, 
  SS_Add32x2, 
  SS_Sub32x2, 
  SS_Mul32x2, 
  SS_Div32x2, 
  SS_Max32x2, 
  SS_Min32x2, 
  SS_Mod32x2, 
  SS_And32x2, 
  SS_Nand32x2, 
  SS_Or32x2, 
  SS_Nor32x2, 
  SS_Xor32x2, 
  SS_Not32x2, 
  SS_CmpEQ32x2, 
  SS_CmpNE32x2, 
  SS_CmpGE32x2, 
  SS_CmpGT32x2, 
  SS_CmpLE32x2, 
  SS_CmpLT32x2, 
  SS_LShf64, 
  SS_RShf64, 
  SS_AndR64, 
  SS_OrR64, 
  SS_XorR64, 
  SS_Acc64, 
  SS_Add64, 
  SS_Sub64, 
  SS_Mul64, 
  SS_Div64, 
  SS_Max64, 
  SS_Min64, 
  SS_Mod64, 
  SS_And64, 
  SS_Nand64, 
  SS_Or64, 
  SS_Nor64, 
  SS_Xor64, 
  SS_Not64, 
  SS_CmpEQ64, 
  SS_CmpNE64, 
  SS_CmpGE64, 
  SS_CmpGT64, 
  SS_CmpLE64, 
  SS_CmpLT64, 
  SS_FAcc64, 
  SS_FAdd64, 
  SS_FSub64, 
  SS_FMul64, 
  SS_FDiv64, 
  SS_FSin64, 
  SS_FCos64, 
  SS_FSqrt64, 
  SS_NUM_TYPES
};

enum fu_type_t {
  FLOAT32_ALL, 
  FLOAT32_ALU, 
  FLOAT32_TRIG, 
  FLOAT64_ALL, 
  FLOAT64_ALU, 
  FLOAT64_TRIG, 
  FIXED8_ALL, 
  FIXED8_BIT, 
  FIXED8_ALU, 
  FIXED8_LOGIC, 
  FIXED8_CMP, 
  FIXED16_ALL, 
  FIXED16_BIT, 
  FIXED16_ALU, 
  FIXED16_LOGIC, 
  FIXED16_CMP, 
  FIXED32_ALL, 
  FIXED32_BIT, 
  FIXED32_ALU, 
  FIXED32_LOGIC, 
  FIXED32_CMP, 
  FIXED64_ALL, 
  FIXED64_BIT, 
  FIXED64_ALU, 
  FIXED64_LOGIC, 
  FIXED64_CMP, 
  NON_PREDEFINED_FU_TYPE
};

extern int num_ops[114];
extern int bitwidth[114];

// OpCode
OpCode inst_from_string(const char* str);
const char* name_of_inst(OpCode inst);
double inst_area(OpCode inst);
double inst_power(OpCode inst);
int inst_lat(OpCode inst);
int inst_thr(OpCode inst);
int num_values(OpCode inst);
// fu_type_t
fu_type_t fu_type_from_string(const char* str);
const char* name_of_fu_type(fu_type_t fu_type);
double fu_type_area(fu_type_t fu_type);
double fu_type_power(fu_type_t fu_type);
// execute 
uint64_t execute64(OpCode inst, std::vector<uint64_t>& ops, std::vector<uint64_t>& outs, uint64_t* reg, bool &discard, std::vector<bool>& back_array);
uint32_t execute32(OpCode inst, std::vector<uint32_t>& ops, std::vector<uint32_t>& outs, uint32_t* reg, bool &discard, std::vector<bool>& back_array);
uint16_t execute16(OpCode inst, std::vector<uint16_t>& ops, std::vector<uint16_t>& outs, uint16_t* reg, bool &discard, std::vector<bool>& back_array);
uint8_t execute8(OpCode inst, std::vector<uint8_t>& ops, std::vector<uint8_t>& outs, uint8_t* reg, bool &discard, std::vector<bool>& back_array);

}

#endif
