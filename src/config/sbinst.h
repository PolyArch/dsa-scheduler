//This file generated from inst_model.cpp -- Do not edit.  Do not commit to repo.
#ifndef __SB_INST_H__
#define __SB_INST_H__

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

namespace SB_CONFIG {

float    as_float(std::uint32_t ui);
uint32_t as_uint32(float f);

double    as_double(std::uint64_t ui);
uint64_t as_uint64(double f);

std::complex<float> as_float_complex(uint64_t ui);
uint64_t as_uint64(const std::complex<float> &val);

enum sb_inst_t {
SB_NONE=0,
SB_ERR,
SB_Add8, 
SB_Add16, 
SB_Mul16, 
SB_Keep16, 
SB_IndexMatch16, 
SB_FMul32, 
SB_FAdd32, 
SB_Add32, 
SB_Mul32, 
SB_Sig16, 
SB_Add16x4, 
SB_TAdd16x4, 
SB_HAdd16x4, 
SB_RShf16x4, 
SB_Sub16x4, 
SB_Mod16x4, 
SB_Abs16x4, 
SB_Acc16x4, 
SB_RShf2_16x4, 
SB_RShf4_16x4, 
SB_Mul16x4, 
SB_Div16x4, 
SB_Sig16x4, 
SB_Red16x4, 
SB_Max16x4, 
SB_Min16x4, 
SB_SMax16x4, 
SB_SMin16x4, 
SB_RedMax16x4, 
SB_RedMin16x4, 
SB_RedSMax16x4, 
SB_RedSMin16x4, 
SB_DelayFU, 
SB_Discard, 
SB_Keep, 
SB_Mul32x2, 
SB_Add32x2, 
SB_Red32x2, 
SB_RShf32x2, 
SB_Max32x2, 
SB_Min32x2, 
SB_RedMax32x2, 
SB_RedMin32x2, 
SB_BackMul64, 
SB_Mul64, 
SB_Sqr64, 
SB_Div64, 
SB_Add64, 
SB_Sub64, 
SB_RShf64, 
SB_LShf64, 
SB_Max64, 
SB_Min64, 
SB_Acc64, 
SB_FAdd32x2, 
SB_FSub32x2, 
SB_FRed32x2, 
SB_FAddSub32x2, 
SB_FSubAdd32x2, 
SB_FMul32x2, 
SB_FMulX32x2, 
SB_FAcc32x2, 
SB_FltCplxToFx, 
SB_CplxConj, 
SB_CplxMulCplx, 
SB_CplxMulConj, 
SB_NegCplxMulConj, 
SB_CplxRed32x2, 
SB_CplxSqrt, 
SB_CplxInv, 
SB_CplxInvConj, 
SB_CplxSqrtInv, 
SB_CplxNmlz, 
SB_CplxMulCons, 
SB_CplxDivCons, 
SB_CplxGivensCos, 
SB_CplxGivensSin, 
SB_CplxGivensRes, 
SB_RealSqrt, 
SB_RealInv, 
SB_RealSqrtInv, 
SB_Delay, 
SB_HouseHolder, 
SB_ImplicitQR, 
SB_IndexMatch32x2, 
SB_BackSub32x2, 
SB_BackNormRed32x2, 
SB_RLEDecoder16x4, 
SB_SpuMul16x4, 
SB_SpuAdd16x4, 
SB_MacRed16x2, 
SB_MatchIndex2, 
SB_Concatenate16To32, 
SB_Concatenate32To64, 
SB_NegFMul32x2, 
SB_NegCplxMulCons, 
SB_NegCplxDivCons, 
SB_FxRelu16x4, 
SB_FxSig16x4, 
SB_FxTanh16x4, 
SB_FxAdd16x4, 
SB_FxSub16x4, 
SB_FxRed16x4, 
SB_FxMul16x4, 
SB_FxExp16x4, 
SB_FxMulX16x4, 
SB_FxAcc16x4, 
SB_FxAddSub16x4, 
SB_FxSubAdd16x4, 
SB_FxRedCom16x4, 
SB_FxMul32x2, 
SB_FxAdd32x2, 
SB_FxRed32x2, 
SB_DupLow32, 
SB_DupHigh32, 
SB_ConcatLow32, 
SB_FAdd64, 
SB_FSub64, 
SB_FMul64, 
SB_FxExp64, 
SB_Select, 
SB_Merge, 
SB_MergeSentinal, 
SB_Index_match, 
SB_ICmp, 
SB_Hold, 
SB_Nor, 
SB_Phi, 
SB_And3, 
SB_And, 
SB_ICmpG, 
SB_Or, 
SB_Xor, 
SB_Not, 
SB_Copy, 
SB_ICmpEQ, 
SB_ICmpNE, 
SB_Switch, 
SB_Add, 
SB_Sub, 
SB_Mul, 
SB_UDiv, 
SB_SDiv, 
SB_URem, 
SB_SRem, 
SB_IMax, 
SB_IMin, 
SB_SMax, 
SB_SMin, 
SB_FAdd, 
SB_FSub, 
SB_FMul, 
SB_FDiv, 
SB_FRem, 
SB_Sqrt, 
SB_FSin, 
SB_FCos, 
SB_FMax, 
SB_FMin, 
SB_SExt, 
SB_Shl, 
SB_LShr, 
SB_AShr, 
SB_Ternary, 
SB_ICmpUGT, 
SB_ICmpUGE, 
SB_ICmpULT, 
SB_ICmpULE, 
SB_ICmpSGT, 
SB_ICmpSGE, 
SB_ICmpSLT, 
SB_ICmpSLE, 
SB_FCmpOEQ, 
SB_FCmpONE, 
SB_FCmpOGT, 
SB_FCmpOGE, 
SB_FCmpOLT, 
SB_FCmpOLE, 
SB_NUM_TYPES
};

extern int num_ops[180];
extern int bitwidth[180];

sb_inst_t inst_from_string(const char* str);
const char* name_of_inst(sb_inst_t inst);
int inst_lat(sb_inst_t inst);
int inst_thr(sb_inst_t inst);
void execute64(sb_inst_t inst, std::vector<uint64_t>& ops, std::vector<uint64_t>& outs, std::vector<uint64_t>& reg, uint64_t& discard, std::vector<bool>& back_array);
void execute32(sb_inst_t inst, std::vector<uint32_t>& ops, std::vector<uint32_t>& outs, std::vector<uint32_t>& reg, uint64_t& discard, std::vector<bool>& back_array);
void execute16(sb_inst_t inst, std::vector<uint16_t>& ops, std::vector<uint16_t>& outs, std::vector<uint16_t>& reg, uint64_t& discard, std::vector<bool>& back_array);
void execute8(sb_inst_t inst,  std::vector<uint8_t>& ops,  std::vector<uint8_t>& outs,  std::vector<uint8_t>& reg,  uint64_t& discard, std::vector<bool>& back_array);

};

#endif
