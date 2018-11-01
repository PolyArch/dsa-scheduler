//This file generated from inst_model.cpp -- Do not edit.  Do not commit to repo.
#include "sbinst.h"

float SB_CONFIG::as_float(std::uint32_t ui) {
  float f;
  std::memcpy(&f, &ui, sizeof(float));
  return f;
}

uint32_t SB_CONFIG::as_uint32(float f) {
  uint32_t ui;
  std::memcpy(&ui, &f, sizeof(float));
  return ui;
}

double SB_CONFIG::as_double(std::uint64_t ui) {
  double f;
  std::memcpy(&f, &ui, sizeof(double));
  return f;
}

uint64_t SB_CONFIG::as_uint64(double f) {
  uint64_t ui;
  std::memcpy(&ui, &f, sizeof(double));
  return ui;
}

std::complex<float> SB_CONFIG::as_float_complex(uint64_t val) {
  std::complex<float> res;
  std::memcpy(&res, &val, sizeof(val));
  return res;
}

uint64_t SB_CONFIG::as_uint64(const std::complex<float> &val) {
  uint64_t res;
  std::memcpy(&res, &val, sizeof(val));
  return res;
}

using namespace SB_CONFIG;

sb_inst_t SB_CONFIG::inst_from_string(const char* str) {
  if(strcmp(str,"NONE")==0) return SB_NONE;
  else if(strcmp(str,"Add8")==0) return SB_Add8;
  else if(strcmp(str,"Add16")==0) return SB_Add16;
  else if(strcmp(str,"Acc16")==0) return SB_Acc16;
  else if(strcmp(str,"Mul16")==0) return SB_Mul16;
  else if(strcmp(str,"Keep16")==0) return SB_Keep16;
  else if(strcmp(str,"Min16")==0) return SB_Min16;
  else if(strcmp(str,"IndexMatch16")==0) return SB_IndexMatch16;
  else if(strcmp(str,"Control16")==0) return SB_Control16;
  else if(strcmp(str,"ICmpEq16")==0) return SB_ICmpEq16;
  else if(strcmp(str,"ReLU16")==0) return SB_ReLU16;
  else if(strcmp(str,"ICmpNE16")==0) return SB_ICmpNE16;
  else if(strcmp(str,"Select16")==0) return SB_Select16;
  else if(strcmp(str,"FMul32")==0) return SB_FMul32;
  else if(strcmp(str,"FAdd32")==0) return SB_FAdd32;
  else if(strcmp(str,"FSub32")==0) return SB_FSub32;
  else if(strcmp(str,"Add32")==0) return SB_Add32;
  else if(strcmp(str,"Mul32")==0) return SB_Mul32;
  else if(strcmp(str,"Extract16")==0) return SB_Extract16;
  else if(strcmp(str,"Concat16")==0) return SB_Concat16;
  else if(strcmp(str,"BitsliceAcc64")==0) return SB_BitsliceAcc64;
  else if(strcmp(str,"Sig16")==0) return SB_Sig16;
  else if(strcmp(str,"Add16x4")==0) return SB_Add16x4;
  else if(strcmp(str,"TAdd16x4")==0) return SB_TAdd16x4;
  else if(strcmp(str,"HAdd16x4")==0) return SB_HAdd16x4;
  else if(strcmp(str,"RShf16x4")==0) return SB_RShf16x4;
  else if(strcmp(str,"Sub16x4")==0) return SB_Sub16x4;
  else if(strcmp(str,"Mod16x4")==0) return SB_Mod16x4;
  else if(strcmp(str,"Abs16x4")==0) return SB_Abs16x4;
  else if(strcmp(str,"Acc16x4")==0) return SB_Acc16x4;
  else if(strcmp(str,"RShf2_16x4")==0) return SB_RShf2_16x4;
  else if(strcmp(str,"RShf4_16x4")==0) return SB_RShf4_16x4;
  else if(strcmp(str,"Mul16x4")==0) return SB_Mul16x4;
  else if(strcmp(str,"Div16x4")==0) return SB_Div16x4;
  else if(strcmp(str,"Sig16x4")==0) return SB_Sig16x4;
  else if(strcmp(str,"Red16x4")==0) return SB_Red16x4;
  else if(strcmp(str,"Max16x4")==0) return SB_Max16x4;
  else if(strcmp(str,"Min16x4")==0) return SB_Min16x4;
  else if(strcmp(str,"SMax16x4")==0) return SB_SMax16x4;
  else if(strcmp(str,"SMin16x4")==0) return SB_SMin16x4;
  else if(strcmp(str,"RedMax16x4")==0) return SB_RedMax16x4;
  else if(strcmp(str,"RedMin16x4")==0) return SB_RedMin16x4;
  else if(strcmp(str,"RedSMax16x4")==0) return SB_RedSMax16x4;
  else if(strcmp(str,"RedSMin16x4")==0) return SB_RedSMin16x4;
  else if(strcmp(str,"DelayFU")==0) return SB_DelayFU;
  else if(strcmp(str,"Discard")==0) return SB_Discard;
  else if(strcmp(str,"Keep")==0) return SB_Keep;
  else if(strcmp(str,"Mul32x2")==0) return SB_Mul32x2;
  else if(strcmp(str,"Add32x2")==0) return SB_Add32x2;
  else if(strcmp(str,"Red32x2")==0) return SB_Red32x2;
  else if(strcmp(str,"RShf32x2")==0) return SB_RShf32x2;
  else if(strcmp(str,"Max32x2")==0) return SB_Max32x2;
  else if(strcmp(str,"Min32x2")==0) return SB_Min32x2;
  else if(strcmp(str,"RedMax32x2")==0) return SB_RedMax32x2;
  else if(strcmp(str,"RedMin32x2")==0) return SB_RedMin32x2;
  else if(strcmp(str,"BackMul64")==0) return SB_BackMul64;
  else if(strcmp(str,"Mul64")==0) return SB_Mul64;
  else if(strcmp(str,"Sqr64")==0) return SB_Sqr64;
  else if(strcmp(str,"Div64")==0) return SB_Div64;
  else if(strcmp(str,"Add64")==0) return SB_Add64;
  else if(strcmp(str,"Sub64")==0) return SB_Sub64;
  else if(strcmp(str,"RShf64")==0) return SB_RShf64;
  else if(strcmp(str,"LShf64")==0) return SB_LShf64;
  else if(strcmp(str,"Max64")==0) return SB_Max64;
  else if(strcmp(str,"Min64")==0) return SB_Min64;
  else if(strcmp(str,"Acc64")==0) return SB_Acc64;
  else if(strcmp(str,"FAdd32x2")==0) return SB_FAdd32x2;
  else if(strcmp(str,"FSub32x2")==0) return SB_FSub32x2;
  else if(strcmp(str,"FRed32x2")==0) return SB_FRed32x2;
  else if(strcmp(str,"FAddSub32x2")==0) return SB_FAddSub32x2;
  else if(strcmp(str,"FSubAdd32x2")==0) return SB_FSubAdd32x2;
  else if(strcmp(str,"FMul32x2")==0) return SB_FMul32x2;
  else if(strcmp(str,"FMulX32x2")==0) return SB_FMulX32x2;
  else if(strcmp(str,"FAcc32x2")==0) return SB_FAcc32x2;
  else if(strcmp(str,"FltCplxToFx")==0) return SB_FltCplxToFx;
  else if(strcmp(str,"CplxConj")==0) return SB_CplxConj;
  else if(strcmp(str,"CplxMulCplx")==0) return SB_CplxMulCplx;
  else if(strcmp(str,"CplxMulConj")==0) return SB_CplxMulConj;
  else if(strcmp(str,"NegCplxMulConj")==0) return SB_NegCplxMulConj;
  else if(strcmp(str,"CplxRed32x2")==0) return SB_CplxRed32x2;
  else if(strcmp(str,"CplxSqrt")==0) return SB_CplxSqrt;
  else if(strcmp(str,"CplxInv")==0) return SB_CplxInv;
  else if(strcmp(str,"CplxInvConj")==0) return SB_CplxInvConj;
  else if(strcmp(str,"CplxSqrtInv")==0) return SB_CplxSqrtInv;
  else if(strcmp(str,"CplxNmlz")==0) return SB_CplxNmlz;
  else if(strcmp(str,"CplxMulCons")==0) return SB_CplxMulCons;
  else if(strcmp(str,"CplxDivCons")==0) return SB_CplxDivCons;
  else if(strcmp(str,"CplxGivensCos")==0) return SB_CplxGivensCos;
  else if(strcmp(str,"CplxGivensSin")==0) return SB_CplxGivensSin;
  else if(strcmp(str,"CplxGivensRes")==0) return SB_CplxGivensRes;
  else if(strcmp(str,"RealSqrt")==0) return SB_RealSqrt;
  else if(strcmp(str,"RealInv")==0) return SB_RealInv;
  else if(strcmp(str,"RealSqrtInv")==0) return SB_RealSqrtInv;
  else if(strcmp(str,"Delay")==0) return SB_Delay;
  else if(strcmp(str,"HouseHolder")==0) return SB_HouseHolder;
  else if(strcmp(str,"ImplicitQR")==0) return SB_ImplicitQR;
  else if(strcmp(str,"IndexMatch32x2")==0) return SB_IndexMatch32x2;
  else if(strcmp(str,"BackSub32x2")==0) return SB_BackSub32x2;
  else if(strcmp(str,"BackNormRed32x2")==0) return SB_BackNormRed32x2;
  else if(strcmp(str,"RLEDecoder16x4")==0) return SB_RLEDecoder16x4;
  else if(strcmp(str,"SpuMul16x4")==0) return SB_SpuMul16x4;
  else if(strcmp(str,"SpuAdd16x4")==0) return SB_SpuAdd16x4;
  else if(strcmp(str,"MacRed16x2")==0) return SB_MacRed16x2;
  else if(strcmp(str,"MatchIndex2")==0) return SB_MatchIndex2;
  else if(strcmp(str,"Concatenate16To32")==0) return SB_Concatenate16To32;
  else if(strcmp(str,"Concatenate32To64")==0) return SB_Concatenate32To64;
  else if(strcmp(str,"NegFMul32x2")==0) return SB_NegFMul32x2;
  else if(strcmp(str,"NegCplxMulCons")==0) return SB_NegCplxMulCons;
  else if(strcmp(str,"NegCplxDivCons")==0) return SB_NegCplxDivCons;
  else if(strcmp(str,"FxRelu16x4")==0) return SB_FxRelu16x4;
  else if(strcmp(str,"FxSig16x4")==0) return SB_FxSig16x4;
  else if(strcmp(str,"FxTanh16x4")==0) return SB_FxTanh16x4;
  else if(strcmp(str,"FxAdd16x4")==0) return SB_FxAdd16x4;
  else if(strcmp(str,"FxSub16x4")==0) return SB_FxSub16x4;
  else if(strcmp(str,"FxRed16x4")==0) return SB_FxRed16x4;
  else if(strcmp(str,"FxMul16x4")==0) return SB_FxMul16x4;
  else if(strcmp(str,"FxExp16x4")==0) return SB_FxExp16x4;
  else if(strcmp(str,"FxMulX16x4")==0) return SB_FxMulX16x4;
  else if(strcmp(str,"FxAcc16x4")==0) return SB_FxAcc16x4;
  else if(strcmp(str,"FxAddSub16x4")==0) return SB_FxAddSub16x4;
  else if(strcmp(str,"FxSubAdd16x4")==0) return SB_FxSubAdd16x4;
  else if(strcmp(str,"FxRedCom16x4")==0) return SB_FxRedCom16x4;
  else if(strcmp(str,"FxMul32x2")==0) return SB_FxMul32x2;
  else if(strcmp(str,"FxAdd32x2")==0) return SB_FxAdd32x2;
  else if(strcmp(str,"FxRed32x2")==0) return SB_FxRed32x2;
  else if(strcmp(str,"DupLow32")==0) return SB_DupLow32;
  else if(strcmp(str,"DupHigh32")==0) return SB_DupHigh32;
  else if(strcmp(str,"ConcatLow32")==0) return SB_ConcatLow32;
  else if(strcmp(str,"FAdd64")==0) return SB_FAdd64;
  else if(strcmp(str,"FSub64")==0) return SB_FSub64;
  else if(strcmp(str,"FMul64")==0) return SB_FMul64;
  else if(strcmp(str,"FxExp64")==0) return SB_FxExp64;
  else if(strcmp(str,"Select")==0) return SB_Select;
  else if(strcmp(str,"Merge")==0) return SB_Merge;
  else if(strcmp(str,"MergeSentinal")==0) return SB_MergeSentinal;
  else if(strcmp(str,"Index_match")==0) return SB_Index_match;
  else if(strcmp(str,"ICmp")==0) return SB_ICmp;
  else if(strcmp(str,"Hold")==0) return SB_Hold;
  else if(strcmp(str,"Nor")==0) return SB_Nor;
  else if(strcmp(str,"Phi")==0) return SB_Phi;
  else if(strcmp(str,"And3")==0) return SB_And3;
  else if(strcmp(str,"And")==0) return SB_And;
  else if(strcmp(str,"ICmpG")==0) return SB_ICmpG;
  else if(strcmp(str,"Or")==0) return SB_Or;
  else if(strcmp(str,"Xor")==0) return SB_Xor;
  else if(strcmp(str,"Not")==0) return SB_Not;
  else if(strcmp(str,"Copy")==0) return SB_Copy;
  else if(strcmp(str,"ICmpEQ")==0) return SB_ICmpEQ;
  else if(strcmp(str,"ICmpNE")==0) return SB_ICmpNE;
  else if(strcmp(str,"Switch")==0) return SB_Switch;
  else if(strcmp(str,"Add")==0) return SB_Add;
  else if(strcmp(str,"Sub")==0) return SB_Sub;
  else if(strcmp(str,"Mul")==0) return SB_Mul;
  else if(strcmp(str,"UDiv")==0) return SB_UDiv;
  else if(strcmp(str,"SDiv")==0) return SB_SDiv;
  else if(strcmp(str,"URem")==0) return SB_URem;
  else if(strcmp(str,"SRem")==0) return SB_SRem;
  else if(strcmp(str,"IMax")==0) return SB_IMax;
  else if(strcmp(str,"IMin")==0) return SB_IMin;
  else if(strcmp(str,"SMax")==0) return SB_SMax;
  else if(strcmp(str,"SMin")==0) return SB_SMin;
  else if(strcmp(str,"FAdd")==0) return SB_FAdd;
  else if(strcmp(str,"FSub")==0) return SB_FSub;
  else if(strcmp(str,"FMul")==0) return SB_FMul;
  else if(strcmp(str,"FDiv")==0) return SB_FDiv;
  else if(strcmp(str,"FRem")==0) return SB_FRem;
  else if(strcmp(str,"Sqrt")==0) return SB_Sqrt;
  else if(strcmp(str,"FSin")==0) return SB_FSin;
  else if(strcmp(str,"FCos")==0) return SB_FCos;
  else if(strcmp(str,"FMax")==0) return SB_FMax;
  else if(strcmp(str,"FMin")==0) return SB_FMin;
  else if(strcmp(str,"SExt")==0) return SB_SExt;
  else if(strcmp(str,"Shl")==0) return SB_Shl;
  else if(strcmp(str,"LShr")==0) return SB_LShr;
  else if(strcmp(str,"AShr")==0) return SB_AShr;
  else if(strcmp(str,"Ternary")==0) return SB_Ternary;
  else if(strcmp(str,"ICmpUGT")==0) return SB_ICmpUGT;
  else if(strcmp(str,"ICmpUGE")==0) return SB_ICmpUGE;
  else if(strcmp(str,"ICmpULT")==0) return SB_ICmpULT;
  else if(strcmp(str,"ICmpULE")==0) return SB_ICmpULE;
  else if(strcmp(str,"ICmpSGT")==0) return SB_ICmpSGT;
  else if(strcmp(str,"ICmpSGE")==0) return SB_ICmpSGE;
  else if(strcmp(str,"ICmpSLT")==0) return SB_ICmpSLT;
  else if(strcmp(str,"ICmpSLE")==0) return SB_ICmpSLE;
  else if(strcmp(str,"FCmpOEQ")==0) return SB_FCmpOEQ;
  else if(strcmp(str,"FCmpONE")==0) return SB_FCmpONE;
  else if(strcmp(str,"FCmpOGT")==0) return SB_FCmpOGT;
  else if(strcmp(str,"FCmpOGE")==0) return SB_FCmpOGE;
  else if(strcmp(str,"FCmpOLT")==0) return SB_FCmpOLT;
  else if(strcmp(str,"FCmpOLE")==0) return SB_FCmpOLE;
  else { fprintf(stderr,"Config Library does not understand string\"%s\"\n",str); assert(0); return SB_ERR;}

}

const char* SB_CONFIG::name_of_inst(sb_inst_t inst) {
  switch(inst) {
    case SB_Add8: return "Add8";
    case SB_Add16: return "Add16";
    case SB_Acc16: return "Acc16";
    case SB_Mul16: return "Mul16";
    case SB_Keep16: return "Keep16";
    case SB_Min16: return "Min16";
    case SB_IndexMatch16: return "IndexMatch16";
    case SB_Control16: return "Control16";
    case SB_ICmpEq16: return "ICmpEq16";
    case SB_ReLU16: return "ReLU16";
    case SB_ICmpNE16: return "ICmpNE16";
    case SB_Select16: return "Select16";
    case SB_FMul32: return "FMul32";
    case SB_FAdd32: return "FAdd32";
    case SB_FSub32: return "FSub32";
    case SB_Add32: return "Add32";
    case SB_Mul32: return "Mul32";
    case SB_Extract16: return "Extract16";
    case SB_Concat16: return "Concat16";
    case SB_BitsliceAcc64: return "BitsliceAcc64";
    case SB_Sig16: return "Sig16";
    case SB_Add16x4: return "Add16x4";
    case SB_TAdd16x4: return "TAdd16x4";
    case SB_HAdd16x4: return "HAdd16x4";
    case SB_RShf16x4: return "RShf16x4";
    case SB_Sub16x4: return "Sub16x4";
    case SB_Mod16x4: return "Mod16x4";
    case SB_Abs16x4: return "Abs16x4";
    case SB_Acc16x4: return "Acc16x4";
    case SB_RShf2_16x4: return "RShf2_16x4";
    case SB_RShf4_16x4: return "RShf4_16x4";
    case SB_Mul16x4: return "Mul16x4";
    case SB_Div16x4: return "Div16x4";
    case SB_Sig16x4: return "Sig16x4";
    case SB_Red16x4: return "Red16x4";
    case SB_Max16x4: return "Max16x4";
    case SB_Min16x4: return "Min16x4";
    case SB_SMax16x4: return "SMax16x4";
    case SB_SMin16x4: return "SMin16x4";
    case SB_RedMax16x4: return "RedMax16x4";
    case SB_RedMin16x4: return "RedMin16x4";
    case SB_RedSMax16x4: return "RedSMax16x4";
    case SB_RedSMin16x4: return "RedSMin16x4";
    case SB_DelayFU: return "DelayFU";
    case SB_Discard: return "Discard";
    case SB_Keep: return "Keep";
    case SB_Mul32x2: return "Mul32x2";
    case SB_Add32x2: return "Add32x2";
    case SB_Red32x2: return "Red32x2";
    case SB_RShf32x2: return "RShf32x2";
    case SB_Max32x2: return "Max32x2";
    case SB_Min32x2: return "Min32x2";
    case SB_RedMax32x2: return "RedMax32x2";
    case SB_RedMin32x2: return "RedMin32x2";
    case SB_BackMul64: return "BackMul64";
    case SB_Mul64: return "Mul64";
    case SB_Sqr64: return "Sqr64";
    case SB_Div64: return "Div64";
    case SB_Add64: return "Add64";
    case SB_Sub64: return "Sub64";
    case SB_RShf64: return "RShf64";
    case SB_LShf64: return "LShf64";
    case SB_Max64: return "Max64";
    case SB_Min64: return "Min64";
    case SB_Acc64: return "Acc64";
    case SB_FAdd32x2: return "FAdd32x2";
    case SB_FSub32x2: return "FSub32x2";
    case SB_FRed32x2: return "FRed32x2";
    case SB_FAddSub32x2: return "FAddSub32x2";
    case SB_FSubAdd32x2: return "FSubAdd32x2";
    case SB_FMul32x2: return "FMul32x2";
    case SB_FMulX32x2: return "FMulX32x2";
    case SB_FAcc32x2: return "FAcc32x2";
    case SB_FltCplxToFx: return "FltCplxToFx";
    case SB_CplxConj: return "CplxConj";
    case SB_CplxMulCplx: return "CplxMulCplx";
    case SB_CplxMulConj: return "CplxMulConj";
    case SB_NegCplxMulConj: return "NegCplxMulConj";
    case SB_CplxRed32x2: return "CplxRed32x2";
    case SB_CplxSqrt: return "CplxSqrt";
    case SB_CplxInv: return "CplxInv";
    case SB_CplxInvConj: return "CplxInvConj";
    case SB_CplxSqrtInv: return "CplxSqrtInv";
    case SB_CplxNmlz: return "CplxNmlz";
    case SB_CplxMulCons: return "CplxMulCons";
    case SB_CplxDivCons: return "CplxDivCons";
    case SB_CplxGivensCos: return "CplxGivensCos";
    case SB_CplxGivensSin: return "CplxGivensSin";
    case SB_CplxGivensRes: return "CplxGivensRes";
    case SB_RealSqrt: return "RealSqrt";
    case SB_RealInv: return "RealInv";
    case SB_RealSqrtInv: return "RealSqrtInv";
    case SB_Delay: return "Delay";
    case SB_HouseHolder: return "HouseHolder";
    case SB_ImplicitQR: return "ImplicitQR";
    case SB_IndexMatch32x2: return "IndexMatch32x2";
    case SB_BackSub32x2: return "BackSub32x2";
    case SB_BackNormRed32x2: return "BackNormRed32x2";
    case SB_RLEDecoder16x4: return "RLEDecoder16x4";
    case SB_SpuMul16x4: return "SpuMul16x4";
    case SB_SpuAdd16x4: return "SpuAdd16x4";
    case SB_MacRed16x2: return "MacRed16x2";
    case SB_MatchIndex2: return "MatchIndex2";
    case SB_Concatenate16To32: return "Concatenate16To32";
    case SB_Concatenate32To64: return "Concatenate32To64";
    case SB_NegFMul32x2: return "NegFMul32x2";
    case SB_NegCplxMulCons: return "NegCplxMulCons";
    case SB_NegCplxDivCons: return "NegCplxDivCons";
    case SB_FxRelu16x4: return "FxRelu16x4";
    case SB_FxSig16x4: return "FxSig16x4";
    case SB_FxTanh16x4: return "FxTanh16x4";
    case SB_FxAdd16x4: return "FxAdd16x4";
    case SB_FxSub16x4: return "FxSub16x4";
    case SB_FxRed16x4: return "FxRed16x4";
    case SB_FxMul16x4: return "FxMul16x4";
    case SB_FxExp16x4: return "FxExp16x4";
    case SB_FxMulX16x4: return "FxMulX16x4";
    case SB_FxAcc16x4: return "FxAcc16x4";
    case SB_FxAddSub16x4: return "FxAddSub16x4";
    case SB_FxSubAdd16x4: return "FxSubAdd16x4";
    case SB_FxRedCom16x4: return "FxRedCom16x4";
    case SB_FxMul32x2: return "FxMul32x2";
    case SB_FxAdd32x2: return "FxAdd32x2";
    case SB_FxRed32x2: return "FxRed32x2";
    case SB_DupLow32: return "DupLow32";
    case SB_DupHigh32: return "DupHigh32";
    case SB_ConcatLow32: return "ConcatLow32";
    case SB_FAdd64: return "FAdd64";
    case SB_FSub64: return "FSub64";
    case SB_FMul64: return "FMul64";
    case SB_FxExp64: return "FxExp64";
    case SB_Select: return "Select";
    case SB_Merge: return "Merge";
    case SB_MergeSentinal: return "MergeSentinal";
    case SB_Index_match: return "Index_match";
    case SB_ICmp: return "ICmp";
    case SB_Hold: return "Hold";
    case SB_Nor: return "Nor";
    case SB_Phi: return "Phi";
    case SB_And3: return "And3";
    case SB_And: return "And";
    case SB_ICmpG: return "ICmpG";
    case SB_Or: return "Or";
    case SB_Xor: return "Xor";
    case SB_Not: return "Not";
    case SB_Copy: return "Copy";
    case SB_ICmpEQ: return "ICmpEQ";
    case SB_ICmpNE: return "ICmpNE";
    case SB_Switch: return "Switch";
    case SB_Add: return "Add";
    case SB_Sub: return "Sub";
    case SB_Mul: return "Mul";
    case SB_UDiv: return "UDiv";
    case SB_SDiv: return "SDiv";
    case SB_URem: return "URem";
    case SB_SRem: return "SRem";
    case SB_IMax: return "IMax";
    case SB_IMin: return "IMin";
    case SB_SMax: return "SMax";
    case SB_SMin: return "SMin";
    case SB_FAdd: return "FAdd";
    case SB_FSub: return "FSub";
    case SB_FMul: return "FMul";
    case SB_FDiv: return "FDiv";
    case SB_FRem: return "FRem";
    case SB_Sqrt: return "Sqrt";
    case SB_FSin: return "FSin";
    case SB_FCos: return "FCos";
    case SB_FMax: return "FMax";
    case SB_FMin: return "FMin";
    case SB_SExt: return "SExt";
    case SB_Shl: return "Shl";
    case SB_LShr: return "LShr";
    case SB_AShr: return "AShr";
    case SB_Ternary: return "Ternary";
    case SB_ICmpUGT: return "ICmpUGT";
    case SB_ICmpUGE: return "ICmpUGE";
    case SB_ICmpULT: return "ICmpULT";
    case SB_ICmpULE: return "ICmpULE";
    case SB_ICmpSGT: return "ICmpSGT";
    case SB_ICmpSGE: return "ICmpSGE";
    case SB_ICmpSLT: return "ICmpSLT";
    case SB_ICmpSLE: return "ICmpSLE";
    case SB_FCmpOEQ: return "FCmpOEQ";
    case SB_FCmpONE: return "FCmpONE";
    case SB_FCmpOGT: return "FCmpOGT";
    case SB_FCmpOGE: return "FCmpOGE";
    case SB_FCmpOLT: return "FCmpOLT";
    case SB_FCmpOLE: return "FCmpOLE";
case SB_NONE: return "NONE";
case SB_ERR:  assert(0); return "ERR";
case SB_NUM_TYPES:  assert(0); return "ERR";
    default: assert(0); return "DEFAULT";
  }

}

int SB_CONFIG::inst_lat(sb_inst_t inst) {
  switch(inst) {
    case SB_Add8: return 1;
    case SB_Add16: return 1;
    case SB_Acc16: return 1;
    case SB_Mul16: return 1;
    case SB_Keep16: return 1;
    case SB_Min16: return 1;
    case SB_IndexMatch16: return 1;
    case SB_Control16: return 1;
    case SB_ICmpEq16: return 1;
    case SB_ReLU16: return 1;
    case SB_ICmpNE16: return 1;
    case SB_Select16: return 1;
    case SB_FMul32: return 1;
    case SB_FAdd32: return 1;
    case SB_FSub32: return 1;
    case SB_Add32: return 1;
    case SB_Mul32: return 1;
    case SB_Extract16: return 1;
    case SB_Concat16: return 1;
    case SB_BitsliceAcc64: return 1;
    case SB_Sig16: return 3;
    case SB_Add16x4: return 1;
    case SB_TAdd16x4: return 1;
    case SB_HAdd16x4: return 1;
    case SB_RShf16x4: return 1;
    case SB_Sub16x4: return 1;
    case SB_Mod16x4: return 1;
    case SB_Abs16x4: return 1;
    case SB_Acc16x4: return 1;
    case SB_RShf2_16x4: return 1;
    case SB_RShf4_16x4: return 1;
    case SB_Mul16x4: return 1;
    case SB_Div16x4: return 3;
    case SB_Sig16x4: return 3;
    case SB_Red16x4: return 2;
    case SB_Max16x4: return 1;
    case SB_Min16x4: return 1;
    case SB_SMax16x4: return 1;
    case SB_SMin16x4: return 1;
    case SB_RedMax16x4: return 2;
    case SB_RedMin16x4: return 2;
    case SB_RedSMax16x4: return 2;
    case SB_RedSMin16x4: return 2;
    case SB_DelayFU: return 0;
    case SB_Discard: return 1;
    case SB_Keep: return 1;
    case SB_Mul32x2: return 1;
    case SB_Add32x2: return 1;
    case SB_Red32x2: return 2;
    case SB_RShf32x2: return 1;
    case SB_Max32x2: return 1;
    case SB_Min32x2: return 1;
    case SB_RedMax32x2: return 2;
    case SB_RedMin32x2: return 2;
    case SB_BackMul64: return 1;
    case SB_Mul64: return 1;
    case SB_Sqr64: return 1;
    case SB_Div64: return 1;
    case SB_Add64: return 1;
    case SB_Sub64: return 1;
    case SB_RShf64: return 1;
    case SB_LShf64: return 1;
    case SB_Max64: return 1;
    case SB_Min64: return 1;
    case SB_Acc64: return 1;
    case SB_FAdd32x2: return 1;
    case SB_FSub32x2: return 1;
    case SB_FRed32x2: return 1;
    case SB_FAddSub32x2: return 1;
    case SB_FSubAdd32x2: return 1;
    case SB_FMul32x2: return 2;
    case SB_FMulX32x2: return 2;
    case SB_FAcc32x2: return 1;
    case SB_FltCplxToFx: return 1;
    case SB_CplxConj: return 1;
    case SB_CplxMulCplx: return 4;
    case SB_CplxMulConj: return 4;
    case SB_NegCplxMulConj: return 4;
    case SB_CplxRed32x2: return 1;
    case SB_CplxSqrt: return 46;
    case SB_CplxInv: return 15;
    case SB_CplxInvConj: return 15;
    case SB_CplxSqrtInv: return 47;
    case SB_CplxNmlz: return 17;
    case SB_CplxMulCons: return 2;
    case SB_CplxDivCons: return 14;
    case SB_CplxGivensCos: return 12;
    case SB_CplxGivensSin: return 12;
    case SB_CplxGivensRes: return 12;
    case SB_RealSqrt: return 12;
    case SB_RealInv: return 12;
    case SB_RealSqrtInv: return 13;
    case SB_Delay: return 5;
    case SB_HouseHolder: return 20;
    case SB_ImplicitQR: return 20;
    case SB_IndexMatch32x2: return 1;
    case SB_BackSub32x2: return 1;
    case SB_BackNormRed32x2: return 1;
    case SB_RLEDecoder16x4: return 2;
    case SB_SpuMul16x4: return 2;
    case SB_SpuAdd16x4: return 2;
    case SB_MacRed16x2: return 1;
    case SB_MatchIndex2: return 1;
    case SB_Concatenate16To32: return 1;
    case SB_Concatenate32To64: return 1;
    case SB_NegFMul32x2: return 2;
    case SB_NegCplxMulCons: return 2;
    case SB_NegCplxDivCons: return 2;
    case SB_FxRelu16x4: return 1;
    case SB_FxSig16x4: return 3;
    case SB_FxTanh16x4: return 3;
    case SB_FxAdd16x4: return 1;
    case SB_FxSub16x4: return 1;
    case SB_FxRed16x4: return 2;
    case SB_FxMul16x4: return 1;
    case SB_FxExp16x4: return 3;
    case SB_FxMulX16x4: return 1;
    case SB_FxAcc16x4: return 1;
    case SB_FxAddSub16x4: return 1;
    case SB_FxSubAdd16x4: return 1;
    case SB_FxRedCom16x4: return 1;
    case SB_FxMul32x2: return 1;
    case SB_FxAdd32x2: return 1;
    case SB_FxRed32x2: return 2;
    case SB_DupLow32: return 1;
    case SB_DupHigh32: return 1;
    case SB_ConcatLow32: return 1;
    case SB_FAdd64: return 1;
    case SB_FSub64: return 1;
    case SB_FMul64: return 2;
    case SB_FxExp64: return 3;
    case SB_Select: return 1;
    case SB_Merge: return 1;
    case SB_MergeSentinal: return 1;
    case SB_Index_match: return 1;
    case SB_ICmp: return 1;
    case SB_Hold: return 1;
    case SB_Nor: return 1;
    case SB_Phi: return 1;
    case SB_And3: return 1;
    case SB_And: return 1;
    case SB_ICmpG: return 1;
    case SB_Or: return 1;
    case SB_Xor: return 1;
    case SB_Not: return 1;
    case SB_Copy: return 1;
    case SB_ICmpEQ: return 1;
    case SB_ICmpNE: return 1;
    case SB_Switch: return 1;
    case SB_Add: return 1;
    case SB_Sub: return 1;
    case SB_Mul: return 1;
    case SB_UDiv: return 1;
    case SB_SDiv: return 1;
    case SB_URem: return 1;
    case SB_SRem: return 1;
    case SB_IMax: return 1;
    case SB_IMin: return 1;
    case SB_SMax: return 1;
    case SB_SMin: return 1;
    case SB_FAdd: return 3;
    case SB_FSub: return 3;
    case SB_FMul: return 3;
    case SB_FDiv: return 12;
    case SB_FRem: return 12;
    case SB_Sqrt: return 12;
    case SB_FSin: return 24;
    case SB_FCos: return 24;
    case SB_FMax: return 3;
    case SB_FMin: return 3;
    case SB_SExt: return 1;
    case SB_Shl: return 1;
    case SB_LShr: return 1;
    case SB_AShr: return 1;
    case SB_Ternary: return 1;
    case SB_ICmpUGT: return 1;
    case SB_ICmpUGE: return 1;
    case SB_ICmpULT: return 1;
    case SB_ICmpULE: return 1;
    case SB_ICmpSGT: return 1;
    case SB_ICmpSGE: return 1;
    case SB_ICmpSLT: return 1;
    case SB_ICmpSLE: return 1;
    case SB_FCmpOEQ: return 3;
    case SB_FCmpONE: return 3;
    case SB_FCmpOGT: return 3;
    case SB_FCmpOGE: return 3;
    case SB_FCmpOLT: return 3;
    case SB_FCmpOLE: return 3;
    default: return 1;
  }

}

int SB_CONFIG::inst_thr(sb_inst_t inst) {
  switch(inst) {
    case SB_Add8: return 1;
    case SB_Add16: return 1;
    case SB_Acc16: return 1;
    case SB_Mul16: return 1;
    case SB_Keep16: return 1;
    case SB_Min16: return 1;
    case SB_IndexMatch16: return 1;
    case SB_Control16: return 1;
    case SB_ICmpEq16: return 1;
    case SB_ReLU16: return 1;
    case SB_ICmpNE16: return 1;
    case SB_Select16: return 1;
    case SB_FMul32: return 1;
    case SB_FAdd32: return 1;
    case SB_FSub32: return 1;
    case SB_Add32: return 1;
    case SB_Mul32: return 1;
    case SB_Extract16: return 1;
    case SB_Concat16: return 1;
    case SB_BitsliceAcc64: return 1;
    case SB_Sig16: return 1;
    case SB_Add16x4: return 1;
    case SB_TAdd16x4: return 1;
    case SB_HAdd16x4: return 1;
    case SB_RShf16x4: return 1;
    case SB_Sub16x4: return 1;
    case SB_Mod16x4: return 1;
    case SB_Abs16x4: return 1;
    case SB_Acc16x4: return 1;
    case SB_RShf2_16x4: return 1;
    case SB_RShf4_16x4: return 1;
    case SB_Mul16x4: return 1;
    case SB_Div16x4: return 1;
    case SB_Sig16x4: return 1;
    case SB_Red16x4: return 1;
    case SB_Max16x4: return 1;
    case SB_Min16x4: return 1;
    case SB_SMax16x4: return 1;
    case SB_SMin16x4: return 1;
    case SB_RedMax16x4: return 1;
    case SB_RedMin16x4: return 1;
    case SB_RedSMax16x4: return 1;
    case SB_RedSMin16x4: return 1;
    case SB_DelayFU: return 1;
    case SB_Discard: return 1;
    case SB_Keep: return 1;
    case SB_Mul32x2: return 1;
    case SB_Add32x2: return 1;
    case SB_Red32x2: return 1;
    case SB_RShf32x2: return 1;
    case SB_Max32x2: return 1;
    case SB_Min32x2: return 1;
    case SB_RedMax32x2: return 1;
    case SB_RedMin32x2: return 1;
    case SB_BackMul64: return 1;
    case SB_Mul64: return 1;
    case SB_Sqr64: return 1;
    case SB_Div64: return 1;
    case SB_Add64: return 1;
    case SB_Sub64: return 1;
    case SB_RShf64: return 1;
    case SB_LShf64: return 1;
    case SB_Max64: return 1;
    case SB_Min64: return 1;
    case SB_Acc64: return 1;
    case SB_FAdd32x2: return 1;
    case SB_FSub32x2: return 1;
    case SB_FRed32x2: return 1;
    case SB_FAddSub32x2: return 1;
    case SB_FSubAdd32x2: return 1;
    case SB_FMul32x2: return 1;
    case SB_FMulX32x2: return 1;
    case SB_FAcc32x2: return 1;
    case SB_FltCplxToFx: return 1;
    case SB_CplxConj: return 1;
    case SB_CplxMulCplx: return 1;
    case SB_CplxMulConj: return 1;
    case SB_NegCplxMulConj: return 1;
    case SB_CplxRed32x2: return 1;
    case SB_CplxSqrt: return 1;
    case SB_CplxInv: return 1;
    case SB_CplxInvConj: return 1;
    case SB_CplxSqrtInv: return 1;
    case SB_CplxNmlz: return 1;
    case SB_CplxMulCons: return 1;
    case SB_CplxDivCons: return 1;
    case SB_CplxGivensCos: return 7;
    case SB_CplxGivensSin: return 7;
    case SB_CplxGivensRes: return 7;
    case SB_RealSqrt: return 1;
    case SB_RealInv: return 1;
    case SB_RealSqrtInv: return 1;
    case SB_Delay: return 1;
    case SB_HouseHolder: return 20;
    case SB_ImplicitQR: return 20;
    case SB_IndexMatch32x2: return 1;
    case SB_BackSub32x2: return 1;
    case SB_BackNormRed32x2: return 1;
    case SB_RLEDecoder16x4: return 1;
    case SB_SpuMul16x4: return 1;
    case SB_SpuAdd16x4: return 1;
    case SB_MacRed16x2: return 1;
    case SB_MatchIndex2: return 1;
    case SB_Concatenate16To32: return 1;
    case SB_Concatenate32To64: return 1;
    case SB_NegFMul32x2: return 1;
    case SB_NegCplxMulCons: return 1;
    case SB_NegCplxDivCons: return 1;
    case SB_FxRelu16x4: return 1;
    case SB_FxSig16x4: return 1;
    case SB_FxTanh16x4: return 1;
    case SB_FxAdd16x4: return 1;
    case SB_FxSub16x4: return 1;
    case SB_FxRed16x4: return 1;
    case SB_FxMul16x4: return 1;
    case SB_FxExp16x4: return 1;
    case SB_FxMulX16x4: return 1;
    case SB_FxAcc16x4: return 1;
    case SB_FxAddSub16x4: return 1;
    case SB_FxSubAdd16x4: return 1;
    case SB_FxRedCom16x4: return 1;
    case SB_FxMul32x2: return 1;
    case SB_FxAdd32x2: return 1;
    case SB_FxRed32x2: return 1;
    case SB_DupLow32: return 1;
    case SB_DupHigh32: return 1;
    case SB_ConcatLow32: return 1;
    case SB_FAdd64: return 1;
    case SB_FSub64: return 1;
    case SB_FMul64: return 1;
    case SB_FxExp64: return 1;
    case SB_Select: return 1;
    case SB_Merge: return 1;
    case SB_MergeSentinal: return 1;
    case SB_Index_match: return 1;
    case SB_ICmp: return 1;
    case SB_Hold: return 1;
    case SB_Nor: return 1;
    case SB_Phi: return 1;
    case SB_And3: return 1;
    case SB_And: return 1;
    case SB_ICmpG: return 1;
    case SB_Or: return 1;
    case SB_Xor: return 1;
    case SB_Not: return 1;
    case SB_Copy: return 1;
    case SB_ICmpEQ: return 1;
    case SB_ICmpNE: return 1;
    case SB_Switch: return 1;
    case SB_Add: return 1;
    case SB_Sub: return 1;
    case SB_Mul: return 1;
    case SB_UDiv: return 1;
    case SB_SDiv: return 1;
    case SB_URem: return 1;
    case SB_SRem: return 1;
    case SB_IMax: return 1;
    case SB_IMin: return 1;
    case SB_SMax: return 1;
    case SB_SMin: return 1;
    case SB_FAdd: return 1;
    case SB_FSub: return 1;
    case SB_FMul: return 1;
    case SB_FDiv: return 1;
    case SB_FRem: return 1;
    case SB_Sqrt: return 1;
    case SB_FSin: return 1;
    case SB_FCos: return 1;
    case SB_FMax: return 1;
    case SB_FMin: return 1;
    case SB_SExt: return 1;
    case SB_Shl: return 1;
    case SB_LShr: return 1;
    case SB_AShr: return 1;
    case SB_Ternary: return 1;
    case SB_ICmpUGT: return 1;
    case SB_ICmpUGE: return 1;
    case SB_ICmpULT: return 1;
    case SB_ICmpULE: return 1;
    case SB_ICmpSGT: return 1;
    case SB_ICmpSGE: return 1;
    case SB_ICmpSLT: return 1;
    case SB_ICmpSLE: return 1;
    case SB_FCmpOEQ: return 1;
    case SB_FCmpONE: return 1;
    case SB_FCmpOGT: return 1;
    case SB_FCmpOGE: return 1;
    case SB_FCmpOLT: return 1;
    case SB_FCmpOLE: return 1;
    default: return 1;
  }

}

int SB_CONFIG::num_ops[191]={0, 0
														, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 2, 2, 2, 2
														, 2, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2
														, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2
														, 2, 2, 2, 2, 2, 2, 3, 2, 1, 2, 2, 2, 2, 2, 2, 2
														, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 2, 2, 2, 1, 1
														, 1, 1, 1, 1, 3, 3, 2, 2, 2, 1, 1, 1, 1, 3, 3, 2
														, 3, 2, 3, 3, 3, 1, 2, 2, 2, 2, 3, 3, 2, 2, 2, 2
														, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 2
														, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 3, 3, 2, 1, 2, 2
														, 1, 1, 2, 2, 8, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2
														, 2, 2, 2, 2, 2, 1, 1, 1, 2, 2, 1, 2, 2, 2, 3, 2
														, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};

int SB_CONFIG::bitwidth[191]={0, 0
														, 8, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 32, 32, 32, 32
														, 32, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
														, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
														, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
														, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
														, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
														, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
														, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
														, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
														, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
														, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
														, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64};

uint64_t SB_CONFIG::execute64(sb_inst_t inst, std::vector<uint64_t>& ops, std::vector<uint64_t>& reg, uint64_t& discard, std::vector<bool>& back_array) {
uint64_t& accum = reg[0]; 
  assert(ops.size() <= 4); 
  assert(ops.size() <=  (unsigned)(num_ops[inst]+1)); 
  if((ops.size() > (unsigned)num_ops[inst]) && (ops[ops.size()] == 0)) { 
    return ops[0];
  }
  switch(inst) {
    case SB_Extract16: {
      // Extract 16-bit slice in cyclic order
      
      uint16_t val;
      
      /*
      switch(reg[1]) {
        case 0: val = (ops[0]&0x000000000000FFFF)>>0;
      		  break;
        case 1: val = (ops[0]&0x00000000FFFF0000)>>16;
      		  break;
        case 2: val = (ops[0]&0x0000FFFF00000000)>>32;
      		  break;
        case 3: val = (ops[0]&0xFFFF000000000000)>>48;
      		  break;
        default: printf("wrong slice number");
      		   break;
      }
      */
      
      switch(reg[1]) {
        case 3: val = (ops[0]&0x000000000000FFFF)>>0;
      		  break;
        case 2: val = (ops[0]&0x00000000FFFF0000)>>16;
      		  break;
        case 1: val = (ops[0]&0x0000FFFF00000000)>>32;
      		  break;
        case 0: val = (ops[0]&0xFFFF000000000000)>>48;
      		  break;
        default: printf("wrong slice number");
      		   break;
      }
      
      uint64_t ret = 0x0000000000000000 | val;
      
      if(++reg[1]==4){
        reg[1]=0;
      }
      
      return ret;
    };
    case SB_Concat16: {
      // Concat 16-bit slice in cyclic order
      
      uint64_t ret = 0x0000000000000000;
      uint16_t val;
      
      /*
      switch(reg[1]) {
        case 0: val = (ops[0]&0x000000000000FFFF)>>0;
      		  break;
        case 1: val = (ops[0]&0x00000000FFFF0000)>>16;
      		  break;
        case 2: val = (ops[0]&0x0000FFFF00000000)>>32;
      		  break;
        case 3: val = (ops[0]&0xFFFF000000000000)>>48;
      		  break;
        default: printf("wrong slice number");
      		   break;
      }
      */
      
      switch(reg[1]) {
        case 3: val = (ops[0]&0x000000000000FFFF)>>0;
      		  break;
        case 2: val = (ops[0]&0x00000000FFFF0000)>>16;
      		  break;
        case 1: val = (ops[0]&0x0000FFFF00000000)>>32;
      		  break;
        case 0: val = (ops[0]&0xFFFF000000000000)>>48;
      		  break;
        default: printf("wrong slice number");
      		   break;
      }
      
      ret = ret | val;
      discard=true;
      
      if(++reg[1]==4){
        reg[1]=0;
        discard=false;
      }
      
      return ret;
    };
    case SB_BitsliceAcc64: {
      // Decode Control Logic
      bool do_reset   = ops[1] & (1 << 0);
      bool do_discard = ops[1] & (1 << 1);
      bool dont_accum = ops[1] & (1 << 2);
      // bool do_set_input   = ops[1] & (1 << 3);
      
      // I need to extract 16-bits from reg[1] location in ops[0] and accum
      // uint64_t b = (ops[0] << (reg[1]*16))&OxFFFFFFFFFFFFFFFF;
      uint64_t b = ops[0] << (reg[1]*16);
      
      
      if(!dont_accum) {
        accum+=b;
      }
      
      uint64_t ret = accum;
      
      if(do_discard) {
        discard=1;
      } 
      if(do_reset) {
        accum=0;
      }
      
      if(!do_discard){
       if(++reg[1]!=4){
         discard=true;
       } else {
         reg[1]=0;
       }
      }
      
      return ret; 
      
    };
    case SB_Sig16: {
      #define SIG (op*1024/(1024+op))
      //#define SIG op
      
      uint16_t op = (uint16_t)ops[0];
      
      if(ops.size() > 1) {
        if(ops[1]) {
          return (uint64_t) SIG; 
        } else {
          return ops[0];
        }
      }
      return (uint64_t) SIG;
    };
    case SB_Add16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      a0+=b0;
      a1+=b1;
      a2+=b2;
      a3+=b3;
      uint64_t c0 = (uint64_t)(a0)<<0;
      uint64_t c1 = (uint64_t)(a1)<<16;
      uint64_t c2 = (uint64_t)(a2)<<32;
      uint64_t c3 = (uint64_t)(a3)<<48;
      return c0 | c1 | c2 | c3;
      //return (uint64_t) _mm_adds_pu16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_TAdd16x4: {
      uint32_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint32_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint32_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint32_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint32_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint32_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint32_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint32_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      a0+=b0;
      a1+=b1;
      a2+=b2;
      a3+=b3;
      uint64_t c0 = (uint64_t)(a0&0x0000FFFF)<<0;
      uint64_t c1 = (uint64_t)(a1&0x0000FFFF)<<16;
      uint64_t c2 = (uint64_t)(a2&0x0000FFFF)<<32;
      uint64_t c3 = (uint64_t)(a3&0x0000FFFF)<<48;
      return c0 | c1 | c2 | c3;
      //return (uint64_t) _mm_adds_pu16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_HAdd16x4: {
      uint16_t a0 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t a1 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a2 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a3 = (ops[0]&0x000000000000FFFF)>>0;
      
      uint16_t b0 = (ops[1]&0xFFFF000000000000)>>48;
      //uint16_t b1 = (ops[1]&0x0000FFFF00000000)>>32;
      //uint16_t b2 = (ops[1]&0x00000000FFFF0000)>>16;
      //uint16_t b3 = (ops[1]&0x000000000000FFFF)>>0;
      
      uint64_t c0 = (uint64_t)(a0+a1)<<48;
      uint64_t c1 = (uint64_t)(a1+a2)<<32;
      uint64_t c2 = (uint64_t)(a2+a3)<<16;
      uint64_t c3 = (uint64_t)(a3+b0)<<0;
      
      return c0 | c1 | c2 | c3;
      //return (uint64_t) _mm_adds_pu16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_RShf16x4: {
      uint16_t a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint16_t a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      
      uint64_t b = ops[1];
      if(ops.size()==1) {
        b = 2;
      }
      uint64_t c0 = (uint64_t)(a0>>b)<<0;
      uint64_t c1 = (uint64_t)(a1>>b)<<32;
      return c0 | c1;
      //return (uint64_t) _mm_adds_pu16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_Sub16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      
      a0-=b0;
      a1-=b1;
      a2-=b2;
      a3-=b3;
      
      uint64_t c0 = (uint64_t)(a0 >= 0 ? a0 : (~(-a0)+1)&0x000000000000FFFF)<<0;
      uint64_t c1 = (uint64_t)(a1 >= 0 ? a1 : (~(-a1)+1)&0x000000000000FFFF)<<16;
      uint64_t c2 = (uint64_t)(a2 >= 0 ? a2 : (~(-a2)+1)&0x000000000000FFFF)<<32;
      uint64_t c3 = (uint64_t)(a3 >= 0 ? a3 : (~(-a3)+1)&0x000000000000FFFF)<<48;
      
      return c0 | c1 | c2 | c3;
      //return (uint64_t) _mm_adds_pu16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_Mod16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      a0%=b0;
      a1%=b1;
      a2%=b2;
      a3%=b3;
      uint64_t c0 = (uint64_t)(a0)<<0;
      uint64_t c1 = (uint64_t)(a1)<<16;
      uint64_t c2 = (uint64_t)(a2)<<32;
      uint64_t c3 = (uint64_t)(a3)<<48;
      return c0 | c1 | c2 | c3;
    };
    case SB_Abs16x4: {
      int16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      a0 = a0 >= 0 ? a0 : -a0;
      a1 = a1 >= 0 ? a1 : -a1;
      a2 = a2 >= 0 ? a2 : -a2;
      a3 = a3 >= 0 ? a3 : -a3;
      uint64_t c0 = (uint64_t)(a0)<<0;
      uint64_t c1 = (uint64_t)(a1)<<16;
      uint64_t c2 = (uint64_t)(a2)<<32;
      uint64_t c3 = (uint64_t)(a3)<<48;
      return c0 | c1 | c2 | c3;
    };
    case SB_Acc16x4: {
      // Decode control logic
      bool do_reset   = ops[1] & (1 << 0);
      bool do_discard = ops[1] & (1 << 1);
      bool dont_accum = ops[1] & (1 << 2);
      
      if(!dont_accum) {
        uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
        uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
        uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
        uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
        uint16_t b0 = (accum&0x000000000000FFFF)>>0;
        uint16_t b1 = (accum&0x00000000FFFF0000)>>16;
        uint16_t b2 = (accum&0x0000FFFF00000000)>>32;
        uint16_t b3 = (accum&0xFFFF000000000000)>>48;
      
        a0+=b0;
        a1+=b1;
        a2+=b2;
        a3+=b3;
        uint64_t c0 = (uint64_t)(a0)<<0;
        uint64_t c1 = (uint64_t)(a1)<<16;
        uint64_t c2 = (uint64_t)(a2)<<32;
        uint64_t c3 = (uint64_t)(a3)<<48;
      
        accum = c0 | c1 | c2 | c3;
      }
      
      uint64_t ret = accum;
      
      if(do_discard) {
        discard=1;
      } 
      if(do_reset) {
        accum=0;
      }
      
      return ret; 
    };
    case SB_RShf2_16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      
      uint64_t b = ops[1];
      if(ops.size()==1) {
        b = 2;
      }
      uint64_t c0 = (uint64_t)(a0>>b)<<0;
      uint64_t c1 = (uint64_t)(a1>>b)<<16;
      uint64_t c2 = (uint64_t)(a2>>b)<<32;
      uint64_t c3 = (uint64_t)(a3>>b)<<48;
      return c0 | c1 | c2 | c3;
      //return (uint64_t) _mm_adds_pu16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_RShf4_16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      
      uint64_t b = ops[1];
      if(ops.size()==1) {
        b = 4;
      }
      uint64_t c0 = (uint64_t)(a0>>b)<<0;
      uint64_t c1 = (uint64_t)(a1>>b)<<16;
      uint64_t c2 = (uint64_t)(a2>>b)<<32;
      uint64_t c3 = (uint64_t)(a3>>b)<<48;
      return c0 | c1 | c2 | c3;
      //return (uint64_t) _mm_adds_pu16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_Mul16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      a0*=b0;
      a1*=b1;
      a2*=b2;
      a3*=b3;
      uint64_t c0 = (uint64_t)(a0)<<0;
      uint64_t c1 = (uint64_t)(a1)<<16;
      uint64_t c2 = (uint64_t)(a2)<<32;
      uint64_t c3 = (uint64_t)(a3)<<48;
      return c0 | c1 | c2 | c3;
      //return (uint64_t) _mm_mullo_pi16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_Div16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      a0/=b0;
      a1/=b1;
      a2/=b2;
      a3/=b3;
      uint64_t c0 = (uint64_t)(a0)<<0;
      uint64_t c1 = (uint64_t)(a1)<<16;
      uint64_t c2 = (uint64_t)(a2)<<32;
      uint64_t c3 = (uint64_t)(a3)<<48;
      return c0 | c1 | c2 | c3;
    };
    case SB_Sig16x4: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_Red16x4: {
      uint16_t r0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t r1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t r2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t r3 = (ops[0]&0xFFFF000000000000)>>48;
      
      if(ops.size() > 1) { //additional op is acc
        return (r0+r1+r2+r3+((uint16_t)ops[1]));
      } 
      return (r0+r1+r2+r3);
      
    };
    case SB_Max16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      uint16_t t0 = a0 >= b0 ? a0 : b0;
      uint16_t t1 = a1 >= b1 ? a1 : b1;
      uint16_t t2 = a2 >= b2 ? a2 : b2;
      uint16_t t3 = a3 >= b3 ? a3 : b3;
      uint64_t c0 = (uint64_t)(t0)<<0;
      uint64_t c1 = (uint64_t)(t1)<<16;
      uint64_t c2 = (uint64_t)(t2)<<32;
      uint64_t c3 = (uint64_t)(t3)<<48;
      return c0 | c1 | c2 | c3;
    };
    case SB_Min16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      uint16_t t0 = a0 <= b0 ? a0 : b0;
      uint16_t t1 = a1 <= b1 ? a1 : b1;
      uint16_t t2 = a2 <= b2 ? a2 : b2;
      uint16_t t3 = a3 <= b3 ? a3 : b3;
      uint64_t c0 = (uint64_t)(t0)<<0;
      uint64_t c1 = (uint64_t)(t1)<<16;
      uint64_t c2 = (uint64_t)(t2)<<32;
      uint64_t c3 = (uint64_t)(t3)<<48;
      return c0 | c1 | c2 | c3;
    };
    case SB_SMax16x4: {
      int16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      int16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      int16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      int16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      int16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      int16_t t0 = a0 >= b0 ? a0 : b0;
      int16_t t1 = a1 >= b1 ? a1 : b1;
      int16_t t2 = a2 >= b2 ? a2 : b2;
      int16_t t3 = a3 >= b3 ? a3 : b3;
      uint64_t c0 = (uint64_t)(t0)<<0;
      uint64_t c1 = (uint64_t)(t1)<<16;
      uint64_t c2 = (uint64_t)(t2)<<32;
      uint64_t c3 = (uint64_t)(t3)<<48;
      return c0 | c1 | c2 | c3;
    };
    case SB_SMin16x4: {
      int16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      int16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      int16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      int16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      int16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      int16_t t0 = a0 <= b0 ? a0 : b0;
      int16_t t1 = a1 <= b1 ? a1 : b1;
      int16_t t2 = a2 <= b2 ? a2 : b2;
      int16_t t3 = a3 <= b3 ? a3 : b3;
      uint64_t c0 = (uint64_t)(t0)<<0;
      uint64_t c1 = (uint64_t)(t1)<<16;
      uint64_t c2 = (uint64_t)(t2)<<32;
      uint64_t c3 = (uint64_t)(t3)<<48;
      return c0 | c1 | c2 | c3;
    };
    case SB_RedMax16x4: {
      uint16_t r0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t r1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t r2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t r3 = (ops[0]&0xFFFF000000000000)>>48;
      
      uint16_t x = r0;
      if(r1 > x) {x=r1;}
      if(r2 > x) {x=r2;}
      if(r3 > x) {x=r3;}
      
      if(ops.size() > 1) { //additional op is acc
        uint16_t b = (uint16_t)ops[1];
        if(b > x) {x=b;}
      } 
      return x;
      
    };
    case SB_RedMin16x4: {
      uint16_t r0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t r1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t r2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t r3 = (ops[0]&0xFFFF000000000000)>>48;
      
      uint16_t x = r0;
      if(r1 < x) {x=r1;}
      if(r2 < x) {x=r2;}
      if(r3 < x) {x=r3;}
      
      if(ops.size() > 1) { //additional op is acc
        uint16_t b = (uint16_t)ops[1];
        if(b < x) {x=b;}
      } 
      return x;
      
    };
    case SB_RedSMax16x4: {
      int16_t r0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t r1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t r2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t r3 = (ops[0]&0xFFFF000000000000)>>48;
      
      int16_t x = r0;
      if(r1 > x) {x=r1;}
      if(r2 > x) {x=r2;}
      if(r3 > x) {x=r3;}
      
      if(ops.size() > 1) { //additional op is acc
        int16_t b = (int16_t)ops[1];
        if(b > x) {x=b;}
      } 
      return (uint64_t)x;
      
    };
    case SB_RedSMin16x4: {
      int16_t r0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t r1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t r2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t r3 = (ops[0]&0xFFFF000000000000)>>48;
      
      int16_t x = r0;
      if(r1 < x) {x=r1;}
      if(r2 < x) {x=r2;}
      if(r3 < x) {x=r3;}
      
      if(ops.size() > 1) { //additional op is acc
        int16_t b = (int16_t)ops[1];
        if(b < x) {x=b;}
      } 
      return (uint64_t)x;
      
    };
    case SB_DelayFU: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_Discard: {
      
      if (ops[1]) {
        discard = 1;
      }
      
      return ops[0];
      
    };
    case SB_Keep: {
      if (!ops[1]) {
        discard = 1;
      }
      
      return ops[0];
      
    };
    case SB_Mul32x2: {
      uint32_t a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      a0*=b0;
      a1*=b1;
      
      uint64_t c0 = (uint64_t)(a0)<<0;
      uint64_t c1 = (uint64_t)(a1)<<32;
      return c0 | c1;
      
      //return (uint64_t) _mm_mullo_pi32((__m64)ops[0], (__m64)ops[1]);  -- mullo_pi32 doesnt exisit in mm intrinsics
    };
    case SB_Add32x2: {
      uint32_t a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      a0+=b0;
      a1+=b1;
      uint64_t c0 = (uint64_t)(a0)<<0;
      uint64_t c1 = (uint64_t)(a1)<<32;
      return c0 | c1;
      //return (uint64_t) _mm_adds_pu16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_Red32x2: {
      uint32_t r0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t r1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      
      if(ops.size() > 1) { //additional op is acc
        return (r0+r1+((uint32_t)ops[1]));
      } 
      return (r0+r1);
      
    };
    case SB_RShf32x2: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      
      uint64_t b = ops[1];
      if(ops.size()==1) {
        b = 2;
      }
      uint64_t c0 = (uint64_t)(a0>>b)<<0;
      uint64_t c1 = (uint64_t)(a1>>b)<<16;
      uint64_t c2 = (uint64_t)(a2>>b)<<32;
      uint64_t c3 = (uint64_t)(a3>>b)<<48;
      return c0 | c1 | c2 | c3;
      //return (uint64_t) _mm_adds_pu16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_Max32x2: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_Min32x2: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_RedMax32x2: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_RedMin32x2: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_BackMul64: {
      back_array[0] = (ops[2] >> 3) & 1;
      back_array[1] = (ops[2] >> 4) & 1;
      back_array[2] = 0;
      
      double a = as_double(ops[0]);
      double b = as_double(ops[1]);
      double c = a*b;
      return as_uint64(c); 
    };
    case SB_Mul64: {
      return ops[0] * ops[1]; 
    };
    case SB_Sqr64: {
      return ops[0]*ops[0];
    };
    case SB_Div64: {
      if(ops[1]==0)
          return 0;
      
      return ops[0]/ops[1];
    };
    case SB_Add64: {
      return ops[0] + ops[1]; 
    };
    case SB_Sub64: {
      return ops[0] - ops[1]; 
    };
    case SB_RShf64: {
      if(ops[1]==64) {
        return 0;
      }
      return ops[0] >> ops[1];
    };
    case SB_LShf64: {
      if(ops[1]==64) {
        return 0;
      }
      return ops[0] << ops[1];
    };
    case SB_Max64: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_Min64: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_Acc64: {
      // Decode Control Logic
      bool do_reset   = ops[1] & (1 << 0);
      bool do_discard = ops[1] & (1 << 1);
      bool dont_accum = ops[1] & (1 << 2);
      // bool do_set_input   = ops[1] & (1 << 3);
      
      if(!dont_accum) {
        accum+=ops[0];
      }
      
      uint64_t ret = accum;
      
      if(do_discard) {
        discard=1;
      } 
      if(do_reset) {
        accum=0;
      }
      return ret; 
    };
    case SB_FAdd32x2: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t t_b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      float b1=as_float(t_b1);
      
      a0+=b0;
      a1+=b1;
      
      uint64_t c0 = (uint64_t)(as_uint32(a0))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(a1))<<32;
      return c0 | c1;
      //return (uint64_t) _mm_adds_pu16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_FSub32x2: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t t_b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      float b1=as_float(t_b1);
      
      a0-=b0;
      a1-=b1;
      
      uint64_t c0 = (uint64_t)(as_uint32(a0))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(a1))<<32;
      return c0 | c1;
      //return (uint64_t) _mm_subs_pu16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_FRed32x2: {
      uint32_t t_r0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_r1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      
      float r0=as_float(t_r0);
      float r1=as_float(t_r1);
      
      float result;
      if(ops.size() > 1) { //additional op is acc
        std::cout << as_float((uint32_t)ops[1]) << std::endl;
        result = r0 + r1 + as_float((uint32_t)ops[1]);
      } else {
        result = r0 + r1;
      }
      
      return (uint64_t)(as_uint32(result));
      
    };
    case SB_FAddSub32x2: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t t_b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      float b1=as_float(t_b1);
      
      a0+=a1;
      b0-=b1;
      
      uint64_t c0 = (uint64_t)(as_uint32(a0))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(b0))<<32;
      return c0 | c1;
      
      // Reduce the value respectively:
      // A: A0    A1
      // B: B0    B1
      // C: A0+A1 B0-B1
    };
    case SB_FSubAdd32x2: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t t_b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      float b1=as_float(t_b1);
      
      a0-=a1;
      b0+=b1;
      
      uint64_t c0 = (uint64_t)(as_uint32(a0))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(b0))<<32;
      return c0 | c1;
      
      // Reduce the value respectively:
      // A: A0    A1
      // B: B0    B1
      // C: A0+A1 B0-B1
    };
    case SB_FMul32x2: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t t_b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      float b1=as_float(t_b1);
      
      a0*=b0;
      a1*=b1;
      
      uint64_t c0 = (uint64_t)(as_uint32(a0))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(a1))<<32;
      return c0 | c1;
      
      //return (uint64_t) _mm_mullo_pi32((__m64)ops[0], (__m64)ops[1]);  -- mullo_pi32 doesnt exisit in mm intrinsics
    };
    case SB_FMulX32x2: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t t_b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      float b1=as_float(t_b1);
      
      a0*=b1;
      a1*=b0;
      
      uint64_t c0 = (uint64_t)(as_uint32(a0))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(a1))<<32;
      return c0 | c1;
      
      // Almost the same as FMul32x2, but multiply the values crossingly
      // A: A0    A1
      // B: B0    B1
      // C: A0*B1 A1*B0
      
    };
    case SB_FAcc32x2: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (accum&0x00000000FFFFFFFF)>>0;
      uint32_t t_b1 = (accum&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      float b1=as_float(t_b1);
      
      a0+=b0;
      a1+=b1;
      
      uint64_t c0 = (uint64_t)(as_uint32(a0))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(a1))<<32;
      accum = c0 | c1;
      
      uint64_t ret = accum;
      
      if(ops[1]==2 || ops[1]==3) {
        discard=1;
      } 
      if(ops[1]!=0 && ops[1]!=2) {
        accum=0;
      }
      
      
      return ret; 
    };
    case SB_FltCplxToFx: {
      complex<float> val(as_float_complex(ops[0]));
      uint16_t real = val.real() * (1 << FRAC_BITS);
      uint16_t imag = val.imag() * (1 << FRAC_BITS);
      
      uint16_t a0(real);
      uint16_t a1(imag);
      uint16_t a2(real);
      uint16_t a3(imag);
      
      uint64_t c0 = (uint64_t)(a0 >= 0 ? a0 : (~(-a0)+1)&0x000000000000FFFF)<<0;
      uint64_t c1 = (uint64_t)(a1 >= 0 ? a1 : (~(-a1)+1)&0x000000000000FFFF)<<16;
      uint64_t c2 = (uint64_t)(a2 >= 0 ? a2 : (~(-a2)+1)&0x000000000000FFFF)<<32;
      uint64_t c3 = (uint64_t)(a3 >= 0 ? a3 : (~(-a3)+1)&0x000000000000FFFF)<<48;
      
      return c0 | c1 | c2 | c3;
    };
    case SB_CplxConj: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      
      std::complex<float> a(a0, a1);
      std::complex<float> c = std::conj(a);
      
      uint64_t c0 = (uint64_t)(as_uint32(c.real()))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(c.imag()))<<32;
      
      return c0 | c1;
      
    };
    case SB_CplxMulCplx: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t t_b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      float b1=as_float(t_b1);
      
      std::complex<float> a(a0, a1);
      std::complex<float> b(b0, b1);
      std::complex<float> c = a * b;
      
      uint64_t c0 = (uint64_t)(as_uint32(c.real()))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(c.imag()))<<32;
      
      return c0 | c1;
      
    };
    case SB_CplxMulConj: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t t_b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      float b1=as_float(t_b1);
      
      std::complex<float> a(a0, a1);
      std::complex<float> b(b0, b1);
      std::complex<float> c = a * std::conj(b);
      
      uint64_t c0 = (uint64_t)(as_uint32(c.real()))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(c.imag()))<<32;
      
      return c0 | c1;
      
    };
    case SB_NegCplxMulConj: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t t_b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      float b1=as_float(t_b1);
      
      std::complex<float> a(a0, a1);
      std::complex<float> b(b0, b1);
      std::complex<float> c = -a * std::conj(b);
      
      uint64_t c0 = (uint64_t)(as_uint32(c.real()))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(c.imag()))<<32;
      
      return c0 | c1;
      
    };
    case SB_CplxRed32x2: {
      uint32_t _a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t _a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(_a0);
      float a1=as_float(_a1);
      
      std::complex<float> res(a0 + a1, 0.0f);
      
      uint64_t c0 = (uint64_t)(as_uint32(res.real()))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(res.imag()))<<32;
      
      return c0 | c1;
    };
    case SB_CplxSqrt: {
      uint32_t _a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t _a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(_a0);
      float a1=as_float(_a1);
      
      std::complex<float> v(a0, a1);
      
      std::complex<float> res = std::sqrt(v);
      
      uint64_t c0 = (uint64_t)(as_uint32(res.real()))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(res.imag()))<<32;
      
      return c0 | c1;
    };
    case SB_CplxInv: {
      uint32_t _a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t _a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(_a0);
      float a1=as_float(_a1);
      
      std::complex<float> v(a0, a1);
      
      std::complex<float> res = std::complex<float>(1., 0) / v;
      
      uint64_t c0 = (uint64_t)(as_uint32(res.real()))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(res.imag()))<<32;
      
      return c0 | c1;
    };
    case SB_CplxInvConj: {
      uint32_t _a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t _a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(_a0);
      float a1=as_float(_a1);
      
      std::complex<float> v(a0, a1);
      
      std::complex<float> res = std::complex<float>(1., 0) / v;
      
      res = std::conj(res);
      
      uint64_t c0 = (uint64_t)(as_uint32(res.real()))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(res.imag()))<<32;
      
      return c0 | c1;
    };
    case SB_CplxSqrtInv: {
      uint32_t _a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t _a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(_a0);
      float a1=as_float(_a1);
      
      std::complex<float> v(a0, a1);
      
      std::complex<float> res = 1.0f / std::sqrt(v);
      
      uint64_t c0 = (uint64_t)(as_uint32(res.real()))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(res.imag()))<<32;
      
      return c0 | c1;
    };
    case SB_CplxNmlz: {
      uint32_t _a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t _a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(_a0);
      float a1=as_float(_a1);
      
      std::complex<float> v(a0, a1);
      
      std::complex<float> norm = std::sqrt(v * std::conj(v));
      
      uint64_t c0 = (uint64_t)(as_uint32(v.real() / norm.real()))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(v.imag() / norm.real()))<<32;
      
      return c0 | c1;
    };
    case SB_CplxMulCons: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      
      a0 *= b0;
      a1 *= b0;
      
      uint64_t c0 = (uint64_t)(as_uint32(a0))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(a1))<<32;
      
      return c0 | c1;
      
    };
    case SB_CplxDivCons: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      
      a0 /= b0;
      a1 /= b0;
      
      uint64_t c0 = (uint64_t)(as_uint32(a0))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(a1))<<32;
      
      return c0 | c1;
      
      //return (uint64_t) _mm_mullo_pi32((__m64)ops[0], (__m64)ops[1]);  -- mullo_pi32 doesnt exisit in mm intrinsics
    };
    case SB_CplxGivensCos: {
      
      complex<float> a(as_float_complex(ops[0])), b(as_float_complex(ops[1])), c(0.);
      
      c = sqrt(
          a.real() * a.real() + a.imag() * a.imag() +
          b.real() * b.real() + b.imag() * b.imag()
      );
      
      c = std::conj(a) / c;
      
      uint64_t res;
      
      memcpy(&res, &c, sizeof res);
      
      return res;
    };
    case SB_CplxGivensSin: {
      
      complex<float> a(as_float_complex(ops[0])), b(as_float_complex(ops[1])), c(0.);
      
      c = sqrt(
          a.real() * a.real() + a.imag() * a.imag() +
          b.real() * b.real() + b.imag() * b.imag()
      );
      
      c = std::conj(b) / c;
      
      uint64_t res;
      
      memcpy(&res, &c, sizeof res);
      
      return res;
    };
    case SB_CplxGivensRes: {
      
      complex<float> a(as_float_complex(ops[0])), b(as_float_complex(ops[1])), c(0.);
      
      c = sqrt(
          a.real() * a.real() + a.imag() * a.imag() +
          b.real() * b.real() + b.imag() * b.imag()
      );
      
      uint64_t res;
      
      memcpy(&res, &c, sizeof res);
      
      return res;
    };
    case SB_RealSqrt: {
      uint32_t _a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      
      float a0=as_float(_a0);
      
      complex<float> val = sqrt(a0);
      
      uint64_t res;
      
      memcpy(&res, &val, sizeof res);
      
      return res;
    };
    case SB_RealInv: {
      uint32_t _a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      
      float a0=as_float(_a0);
      
      float res = 1.0 / a0;
      
      uint64_t c0 = ((uint64_t)(as_uint32(res)))<<0;
      uint64_t c1 = ((uint64_t)(as_uint32(res)))<<32;
      
      return c0 | c1;
    };
    case SB_RealSqrtInv: {
      uint32_t _a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      
      float a0=as_float(_a0);
      
      float res = 1.0 / sqrt(a0);
      
      uint64_t c0 = ((uint64_t)(as_uint32(res)))<<0;
      uint64_t c1 = ((uint64_t)(as_uint32(res)))<<32;
      
      return c0 | c1;
    };
    case SB_Delay: {
      return ops[0];
    };
    case SB_HouseHolder: {
      //Dummy instruction for HouseHolder vectors intermediate variables
      
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t t_b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      uint64_t inst = ops[2];
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      float b1=as_float(t_b1);
      
      float normx(a0 + a1);
      std::complex<float> head(b0, b1);
      normx += head.real() * head.real();
      normx += head.imag() * head.imag();
      normx = sqrt(normx);
      
      //printf("normx: %.5f\n", normx);
      
      std::complex<float> s = -head / ((float)sqrt(head.real() * head.real() + head.imag() * head.imag()));
      std::complex<float> alpha = s * normx;
      
      //printf("s: (%.5f, %.5f)\n", s.real(), s.imag());
      
      if (inst == 0) {
        uint64_t c0 = (uint64_t)(as_uint32(alpha.real()))<<0;
        uint64_t c1 = (uint64_t)(as_uint32(alpha.imag()))<<32;
        return c0 | c1;
      }
      
      std::complex<float> u1 = 1.0f / (head - s * normx);
      
      if (inst == 1) {
        u1 = std::conj(u1);
        uint64_t c0 = (uint64_t)(as_uint32(u1.real()))<<0;
        uint64_t c1 = (uint64_t)(as_uint32(u1.imag()))<<32;
        return c0 | c1;
      }
      
      std::complex<float> tau = -std::conj(s) / u1 / normx;
      
      if (inst == 2) {
        uint64_t c0 = (uint64_t)(as_uint32(tau.real()))<<0;
        uint64_t c1 = (uint64_t)(as_uint32(tau.imag()))<<32;
        return c0 | c1;
      }
      
    };
    case SB_ImplicitQR: {
      //Dummy instruction for Implicit QR factorization on a bidiagonal matrix
      
      #define FINALIZE   1
      #define INITIALIZE 2
      #define FURTHER    3
      #define ITERATION  4
      #define GRABSIN    5
      #define GRABCOS    6
      
      //ops[0]: D
      //ops[1]: F
      //ops[2]: Inst
      
      uint64_t inst = ops[2];
      
      //bool same_as_last = reg[5] == inst;
      uint64_t last_inst = reg[5];
      
      reg[5] = inst;
      
      if (inst == FINALIZE) {
        if (last_inst != FINALIZE && last_inst != ITERATION && last_inst != INITIALIZE) {
          std::cerr << "LAST INSTRUCTION" << last_inst << "\n";
          assert(false && "NOT VALID STATE FOR FINALIZE");
        }
        if (last_inst != FINALIZE) {
          //std::cout << "[FIN] f[i]" << as_float_complex(reg[2]) << "\n";
          return reg[2];
        } else {
          //std::cout << "[FIN] d[i+1]" << as_float_complex(reg[3]) << "\n\n";
          return reg[3];
        }
      }
      
      auto givens = [] (complex<float> a, complex<float> b) -> float {
        return sqrt((std::conj(a) * a).real() + (std::conj(b) * b).real());
      };
      
      if (inst == INITIALIZE) {
        if (last_inst == FINALIZE || last_inst == 0) {
          reg[0] = ops[0];
          reg[1] = ops[1];
          reg[4] = 1;
          discard = 1;
          //std::cout << "d[0]: " << as_float_complex(reg[0]) << "\n";
          //std::cout << "f[0]: " << as_float_complex(reg[1]) << "\n\n";
        } else if (last_inst == INITIALIZE) {
          ++reg[4];
          complex<float> d0 = as_float_complex(reg[0]);
          complex<float> f0 = as_float_complex(reg[1]);
          complex<float> d1 = as_float_complex(ops[0]);
          complex<float> dn1= as_float_complex(ops[1]);
      
          //if (reg[4] == 2) {
          //  std::cout << "d[1]: " << d1 << "\n";
          //  std::cout << "d[n-1]: " << dn1 << "\n";
          //}
      
          float mu = (dn1 * std::conj(dn1)).real();
          complex<float> a = std::conj(d0) * d0 - mu;
          complex<float> b = std::conj(f0) * d0;
          float r  = givens(a, b);
          complex<float> c = std::conj(a) / r;
          if (reg[4] == 2) {
            //std::cout << "[INIT] cos:" << c << "\n";
            return as_uint64(c);
          }
          complex<float> s = std::conj(b) / r;
          if (reg[4] == 3) {
            //std::cout << "[INIT] sin:" << s << "\n";
            return as_uint64(s);
          }
          a = d0 * c + f0 * std::conj(s);
          f0 = d0 * s - f0 * std::conj(c);
          b = d1 * std::conj(s);
          d1 *= -std::conj(c);
          r = givens(a, b);
          c = std::conj(a) / r;
          s = std::conj(b) / r;
          reg[0] = as_uint64(c);
          reg[1] = as_uint64(s);
          reg[2] = as_uint64(f0 * c + d1 * s);
          reg[3] = as_uint64(f0 * std::conj(s) - d1 * std::conj(c));
          //std::cout << "cos:" << c << "\n";
          //std::cout << "sin:" << s << "\n";
          //std::cout << "f[0]:" << as_float_complex(reg[2]) << "\n";
          //std::cout << "d[1]:" << as_float_complex(reg[3]) << "\n";
          //std::cout << "[INIT] [FIN] d[0]:" << r << "\n\n";
          return as_uint32(r);
        } else {
          std::cerr << "LAST INSTRUCTION" << last_inst << "\n";
          assert(false && "NOT VALID STATE FOR INITALIZATION");
        }
      }
      
      if (inst == FURTHER) {
        if (last_inst != INITIALIZE && last_inst != ITERATION) {
          std::cerr << "LAST INSTRUCTION" << last_inst << "\n";
          assert(false && "NOT VALID STATE FOR PREPARING FOR NEXT ITERATION");
        }
        complex<float> c = as_float_complex(reg[0]);
        complex<float> s = as_float_complex(reg[1]);
        complex<float> fi1 = as_float_complex(ops[1]);
        reg[0] = reg[2];
        reg[1] = as_uint64(fi1 * s);
        reg[2] = reg[3];
        reg[3] = as_uint64(-fi1 * std::conj(c));
        //std::cout << "f[i]:" << as_float_complex(reg[0]) << "\n"
        //  << "extra:" << as_float_complex(reg[1]) << "\n"
        //  << "d[i+1]:" << as_float_complex(reg[2]) << "\n"
        //  << "f[i+1]:" << as_float_complex(reg[3]) << "\n\n";
        discard = 1;
      }
      
      if (inst == ITERATION) {
        if (last_inst != FURTHER) {
          if (last_inst != ITERATION) {
            std::cerr << "LAST INSTRUCTION" << last_inst << "\n";
            assert(false && "NOT PREPARED FOR ITERATION");
          } else {
            ++reg[4];
          }
        } else {
          reg[4] = 1;
        }
        complex<float> a = as_float_complex(reg[0]);
        complex<float> b = as_float_complex(reg[1]);
        float r = givens(a, b);
        if (last_inst == FURTHER) {
          //Finalize f[i-1]
          //std::cout << "[FIN] f[i-1]" << r << "\n";
          assert(reg[4] == 1);
          return as_uint32(r);
        }
        complex<float> c  = std::conj(a) / r;
        if (reg[4] == 2) {
          return as_uint64(c);
        }
        complex<float> s  = std::conj(b) / r;
        if (reg[4] == 3) {
          return as_uint64(s);
        }
        complex<float> di = as_float_complex(reg[2]);
        complex<float> fi = as_float_complex(reg[3]);
        complex<float> di1 = as_float_complex(ops[0]);
        a   = di * c + fi * s;
        fi  = di * std::conj(s) - fi * std::conj(c);
        b   = di1 * s;
        di1*= -std::conj(c);
        r = givens(a, b);
        if (reg[4] == 4) {
          //std::cout << "[FIN] d[i]:" << r << "\n\n";
          return as_uint32(r);
        }
        c = std::conj(a) / r;
        s = std::conj(b) / r;
        reg[0] = as_uint64(c);
        reg[1] = as_uint64(s);
        reg[2] = as_uint64(c * fi + s * di1);
        reg[3] = as_uint64(std::conj(s) * fi - std::conj(c) * di1);
      
        //std::cout << "cos:" << c << "\n";
        //std::cout << "sin:" << s << "\n";
        //std::cout << "f[i]:" << as_float_complex(reg[2]) << "\n";
        //std::cout << "d[i+1]:" << as_float_complex(reg[3]) << "\n\n";
      
        discard = 1;
      }
      
      //magic number!
      return -1;
      
      #undef FINALIZE
      #undef FURTHER
      #undef ITERATION
      #undef INITIALIZE
    };
    case SB_IndexMatch32x2: {
      // Control Signal Definitions
      int do_reset   = 1 << 0;
      int do_discard = 1 << 1;
      int dont_accum = 1 << 2;
      int bp_op1     = 1 << 3;
      int bp_op2     = 1 << 4;
      
      #define SENTINAL32 ( ((uint32_t)1)<<31)
      
      uint32_t a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      int fm1=0;
      int fm2=0;
      
      if(a0==b0){
          fm1=1;
      } else if(a0==b1){
          fm1=2;
      }
      
      if(a1==b0){
          fm2=1;
      } else if(a1==b1){
          fm2=2;
      }
      
      int mul_sel = (fm1 << 5) | (fm1 << 7);
      
      if(a1==SENTINAL32 && b1==SENTINAL32) {
          back_array[0]=0;
          back_array[1]=0;
          return dont_accum | do_reset | mul_sel;
      } 
      else if(a1==b1){
          back_array[0]=0;
          back_array[1]=0;
          return do_discard | mul_sel;
      }
      else if(a1<b1 || b1==SENTINAL32){
          back_array[0]=0;
          back_array[1]=1;
          return dont_accum | do_discard | bp_op2 | mul_sel;
      }
      else if(a1>b1 || a1==SENTINAL32){
          back_array[0]=1;
          back_array[1]=0;
          return dont_accum | do_discard | bp_op1 | mul_sel;
      } else {
          assert(0 && "not possible");
      }
    };
    case SB_BackSub32x2: {
      back_array[0] = (ops[2] >> 3) & 1;
      back_array[1] = (ops[2] >> 4) & 1;
      back_array[2] = 0;
      
      uint32_t a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      int fm1 = (ops[2] >> 5) & 3;
      int fm2 = (ops[2] >> 7) & 3;
      
      uint64_t c0 = 0;
      uint64_t c1 = 0;
      
      // could have been fix minus
      switch(fm1){
          case 1: c0 = (uint64_t)(abs((int)(a0-b0)))<<0;
                  break;
          case 2: c0 = (uint64_t)(abs((int)(a0-b1)))<<0;
                  break;
          default: c0 = (uint64_t)(0)<<0;
                   break;
      }
      
      switch(fm2){
          case 1: c1 = (uint64_t)(abs((int)(a1-b0)))<<32;
                  break;
          case 2: c1 = (uint64_t)(abs((int)(a1-b1)))<<32;
                  break;
          default: c1 = (uint64_t)(0)<<32;
                   break;
      }
      
      return c0 | c1;
    };
    case SB_BackNormRed32x2: {
      // Decode Control Logic
      bool do_reset   = ops[1] & (1 << 0);
      bool do_discard = ops[1] & (1 << 1);
      bool dont_accum = ops[1] & (1 << 2);
      
      if(!dont_accum) {
        uint32_t a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
        uint32_t a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
        accum+=((a0*a0)+(a1*a1));
      }
      
      uint64_t ret = accum;
      
      if(do_discard) {
        discard=1;
      } 
      if(do_reset) {
        accum=0;
      }
      return ret; 
    };
    case SB_RLEDecoder16x4: {
      // ops[0] is sind1, ops[1] is Tx, ops[2] is Kx
      
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      uint16_t c0 = (ops[2]&0x000000000000FFFF)>>0;
      uint16_t c1 = (ops[2]&0x00000000FFFF0000)>>16;
      uint16_t c2 = (ops[2]&0x0000FFFF00000000)>>32;
      uint16_t c3 = (ops[2]&0xFFFF000000000000)>>48;
      
      a0 = (uint16_t)(a0/c0) + (uint16_t)(b0*(a0%c0));
      a1 = (uint16_t)(a1/c1) + (uint16_t)(b1*(a1%c1));
      a2 = (uint16_t)(a2/c2) + (uint16_t)(b2*(a2%c2));
      a3 = (uint16_t)(a3/c3) + (uint16_t)(b3*(a3%c3));
      
      uint64_t d0 = (uint64_t)(a0)<<0;
      uint64_t d1 = (uint64_t)(a1)<<16;
      uint64_t d2 = (uint64_t)(a2)<<32;
      uint64_t d3 = (uint64_t)(a3)<<48;
      return d0 | d1 | d2 | d3;
    };
    case SB_SpuMul16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      
      if(ops[2]==0){
        a0*=b0;
        a1*=b0;
        a2*=b0;
        a3*=b0;
      } else if(ops[2]==1){
        a0*=b1;
        a1*=b1;
        a2*=b1;
        a3*=b1;
      } else if(ops[2]==2){
        a0*=b2;
        a1*=b2;
        a2*=b2;
        a3*=b2;
      } else {
        a0*=b3;
        a1*=b3;
        a2*=b3;
        a3*=b3;
      }
      uint64_t c0 = (uint64_t)(a0)<<0;
      uint64_t c1 = (uint64_t)(a1)<<16;
      uint64_t c2 = (uint64_t)(a2)<<32;
      uint64_t c3 = (uint64_t)(a3)<<48;
      return c0 | c1 | c2 | c3;
      //return (uint64_t) _mm_mullo_pi16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_SpuAdd16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      
      if(ops[2]==0){
        a0+=b0;
        a1+=b0;
        a2+=b0;
        a3+=b0;
      } else if(ops[2]==1){
        a0+=b1;
        a1+=b1;
        a2+=b1;
        a3+=b1;
      } else if(ops[2]==2){
        a0+=b2;
        a1+=b2;
        a2+=b2;
        a3+=b2;
      } else {
        a0+=b3;
        a1+=b3;
        a2+=b3;
        a3+=b3;
      }
      uint64_t c0 = (uint64_t)(a0)<<0;
      uint64_t c1 = (uint64_t)(a1)<<16;
      uint64_t c2 = (uint64_t)(a2)<<32;
      uint64_t c3 = (uint64_t)(a3)<<48;
      return c0 | c1 | c2 | c3;
      //return (uint64_t) _mm_mullo_pi16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_MacRed16x2: {
      #define FxPnt 8
      #define fx_to_flt(x) ((float)(x) / (1<<FxPnt))
      #define flt_to_fx(x) ((int)(x) * (1<<FxPnt))
      
      int16_t a = (ops[0] >> 0)  & 65535;
      int16_t b = (ops[0] >> 16) & 65535;
      int16_t c = (ops[0] >> 32) & 65535;
      int16_t d = (ops[0] >> 48) & 65535;
      
      
      int64_t _accum = accum;
      
      _accum += ((int) a * (int) b) + ((int) c * (int) d);
      
      //printf("%f * %f = %f\n", fx_to_flt(a), fx_to_flt(b), fx_to_flt(a) * fx_to_flt(b));
      //printf("%f * %f = %f\n", fx_to_flt(c), fx_to_flt(d), fx_to_flt(c) * fx_to_flt(d));
      
      accum = _accum;
      
      
      if (ops[0] != 0) {
        discard = 1;
      } else {
        accum = 0;
        //printf("Accum: %f\n", fx_to_flt(_accum) / (1 << FxPnt));
      }
      
      return _accum;
      
      #undef FxPnt
      #undef fx_to_flt
      #undef flt_to_fx
    };
    case SB_MatchIndex2: {
      const uint64_t TokenEnd = ~0ull;
      
      if (ops[0] == TokenEnd && ops[1] != TokenEnd) {
        back_array[0] = 1;
        back_array[1] = 0;
        discard = 1;
        //puts("end of ops[0]");
      } else if (ops[1] == TokenEnd && ops[0] != TokenEnd) {
        back_array[0] = 0;
        back_array[1] = 1;
        discard = 1;
        //puts("end of ops[1]");
      } else if (ops[0] == ops[1] && ops[0] == TokenEnd) {
        back_array[0] = 0;
        back_array[1] = 0;
        accum = 0;
        return 0;
        //puts("end of both");
      } else {
        union {
          int16_t a[4];
          uint64_t val;
        } ret;
        ret.a[0] = ret.a[1] = ret.a[2] = ret.a[3] = 0;
      
        int a_idx0 = (ops[0] >> 0)  & 65535;
        int a_idx1 = (ops[0] >> 32) & 65535;
        int16_t a_val0 = (ops[0] >> 16) & 65535;
        int16_t a_val1 = (ops[0] >> 48) & 65535;
      
        int b_idx0 = (ops[1] >> 0)  & 65535;
        int b_idx1 = (ops[1] >> 32) & 65535;
        int16_t b_val0 = (ops[1] >> 16) & 65535;
        int16_t b_val1 = (ops[1] >> 48) & 65535;
        
        int index_a = (accum >> 0) & 65535;
        int index_b = (accum >> 16)& 65535;
      
        //printf("ops[0]: %d %d, %d %d\n", a_idx0, a_val0, a_idx1, a_val1);
        //printf("ops[1]: %d %d, %d %d\n", b_idx0, b_val0, b_idx1, b_val1);
        //printf("index_a': %d v.s. index_b': %d\n", index_a, index_b);
      
        index_a += a_idx0;
        index_b += b_idx0;
      
        if (index_a + a_idx1 == index_b) {
          ret.a[0] = a_val1;
          ret.a[1] = b_val0;
        } else if (index_b + b_idx1 == index_a) {
          ret.a[0] = a_val0;
          ret.a[1] = b_val1;
        } else if (index_a == index_b) {
          ret.a[0] = a_val0;
          ret.a[1] = b_val0;
          if (index_a + a_idx1 == index_b + b_idx1) {
            ret.a[2] = a_val1;
            ret.a[3] = b_val1;
          }
        }
      
        index_a += a_idx1;
        index_b += b_idx1;
        
        if (index_a < index_b) {
          back_array[0] = 0;
          back_array[1] = 1;
          index_b -= b_idx0 + b_idx1;
        } else if (index_a > index_b) {
          back_array[0] = 1;
          back_array[1] = 0;
          index_a -= a_idx0 + a_idx1;
        } else {
          back_array[0] = 0;
          back_array[0] = 0;
        }
      
        //printf("index_a: %d v.s. index_b: %d\n", index_a, index_b);
      
        accum = index_a | (index_b << 16);
      
        if (ret.val == 0)
          discard = 1;
        else {
          //printf("%d %d %d %d\n", (int16_t) (ret.val & 65535), (int16_t) (ret.val >> 16 & 65535),
             //(int16_t) (ret.val >> 32 & 65535), (int16_t) (ret.val >> 48 & 65535));
        }
      
        return ret.val;
      
      }
      
      return -1;
    };
    case SB_Concatenate16To32: {
      union {
        uint16_t a[2];
        uint32_t val;
      } res;
      
      res.a[0] = ops[0] & 65535;
      res.a[1] = ops[1] & 65535;
      
      return res.val;
    };
    case SB_Concatenate32To64: {
      union {
        uint32_t a[2];
        uint64_t val;
      } res;
      
      res.a[0] = ops[0];
      res.a[1] = ops[1];
      
      return res.val;
    };
    case SB_NegFMul32x2: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t t_b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      float b1=as_float(t_b1);
      
      a0*=-b0;
      a1*=-b1;
      
      uint64_t c0 = (uint64_t)(as_uint32(a0))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(a1))<<32;
      return c0 | c1;
      
      //return (uint64_t) _mm_mullo_pi32((__m64)ops[0], (__m64)ops[1]);  -- mullo_pi32 doesnt exisit in mm intrinsics
    };
    case SB_NegCplxMulCons: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      
      a0 *= -b0;
      a1 *= -b0;
      
      uint64_t c0 = (uint64_t)(as_uint32(a0))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(a1))<<32;
      return c0 | c1;
      
    };
    case SB_NegCplxDivCons: {
      uint32_t t_a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t t_a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t t_b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      
      float a0=as_float(t_a0);
      float a1=as_float(t_a1);
      float b0=as_float(t_b0);
      
      a0 /= -b0;
      a1 /= -b0;
      
      uint64_t c0 = (uint64_t)(as_uint32(a0))<<0;
      uint64_t c1 = (uint64_t)(as_uint32(a1))<<32;
      return c0 | c1;
      
      //return (uint64_t) _mm_mullo_pi32((__m64)ops[0], (__m64)ops[1]);  -- mullo_pi32 doesnt exisit in mm intrinsics
    };
    case SB_FxRelu16x4: {
      uint16_t i1 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t i2 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t i3 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t i4 = (ops[0]&0xFFFF000000000000)>>48;
      
      if ((ops.size() > 1) && (ops[1] == 0))
          return ops[0];
      
      if (i1 & 0x8000)
        i1 = 0;
      
      if (i2 & 0x8000)
        i2 = 0;
      
      if (i3 & 0x8000)
        i3 = 0;
      
      if (i4 & 0x8000)
        i4 = 0;
      
      uint64_t o1 = (uint64_t)(i1)<<0;
      uint64_t o2 = (uint64_t)(i2)<<16;
      uint64_t o3 = (uint64_t)(i3)<<32;
      uint64_t o4 = (uint64_t)(i4)<<48;
      
      return o1 | o2 | o3 | o4;
    };
    case SB_FxSig16x4: {
      int16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      
      if ((ops.size() > 1) && (ops[1] == 0))
          return ops[0];
      
      double d0 = FIX_TO_DOUBLE(a0);
      double d1 = FIX_TO_DOUBLE(a1);
      double d2 = FIX_TO_DOUBLE(a2);
      double d3 = FIX_TO_DOUBLE(a3);
      
      d0 = 1 / (1 + exp(-d0));
      d1 = 1 / (1 + exp(-d1));
      d2 = 1 / (1 + exp(-d2));
      d3 = 1 / (1 + exp(-d3));
      
      int16_t b0 = DOUBLE_TO_FIX(d0);
      int16_t b1 = DOUBLE_TO_FIX(d1);
      int16_t b2 = DOUBLE_TO_FIX(d2);
      int16_t b3 = DOUBLE_TO_FIX(d3);
      
      uint64_t c0 = ((uint64_t)(b0)<<0)&0x000000000000FFFF;
      uint64_t c1 = ((uint64_t)(b1)<<16)&0x00000000FFFF0000;
      uint64_t c2 = ((uint64_t)(b2)<<32)&0x0000FFFF00000000;
      uint64_t c3 = ((uint64_t)(b3)<<48)&0xFFFF000000000000;
      
      return c0 | c1 | c2 | c3;
    };
    case SB_FxTanh16x4: {
      int16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      
      if ((ops.size() > 1) && (ops[1] == 0))
          return ops[0];
      
      double d0 = FIX_TO_DOUBLE(a0);
      double d1 = FIX_TO_DOUBLE(a1);
      double d2 = FIX_TO_DOUBLE(a2);
      double d3 = FIX_TO_DOUBLE(a3);
      
      d0 = tanh(d0);
      d1 = tanh(d1);
      d2 = tanh(d2);
      d3 = tanh(d3);
      
      int16_t b0 = DOUBLE_TO_FIX(d0);
      int16_t b1 = DOUBLE_TO_FIX(d1);
      int16_t b2 = DOUBLE_TO_FIX(d2);
      int16_t b3 = DOUBLE_TO_FIX(d3);
      
      uint64_t c0 = ((uint64_t)(b0)<<0)&0x000000000000FFFF;
      uint64_t c1 = ((uint64_t)(b1)<<16)&0x00000000FFFF0000;
      uint64_t c2 = ((uint64_t)(b2)<<32)&0x0000FFFF00000000;
      uint64_t c3 = ((uint64_t)(b3)<<48)&0xFFFF000000000000;
      
      return c0 | c1 | c2 | c3;
    };
    case SB_FxAdd16x4: {
      int16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      int16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      int16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      int16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      int16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      /*
      printf("FxAdd16x4 A: %f %f %f %f\n", FIX_TO_DOUBLE(a0), FIX_TO_DOUBLE(a1), FIX_TO_DOUBLE(a2), FIX_TO_DOUBLE(a3));
      for (int i = 0; i < 64; ++i) putchar(ops[0] >> i & 1 ? '0' : '1'); puts("");
      printf("FxAdd16x4 B: %f %f %f %f\n", FIX_TO_DOUBLE(b0), FIX_TO_DOUBLE(b1), FIX_TO_DOUBLE(b2), FIX_TO_DOUBLE(b3));
      for (int i = 0; i < 64; ++i) putchar(ops[1] >> i & 1 ? '0' : '1'); puts("");
      */
      a0 = FIX_ADD(a0, b0);
      a1 = FIX_ADD(a1, b1);
      a2 = FIX_ADD(a2, b2);
      a3 = FIX_ADD(a3, b3);
      
      uint64_t c0 = (uint64_t)(a0 >= 0 ? a0 : (~(-a0)+1)&0x000000000000FFFF)<<0;
      uint64_t c1 = (uint64_t)(a1 >= 0 ? a1 : (~(-a1)+1)&0x000000000000FFFF)<<16;
      uint64_t c2 = (uint64_t)(a2 >= 0 ? a2 : (~(-a2)+1)&0x000000000000FFFF)<<32;
      uint64_t c3 = (uint64_t)(a3 >= 0 ? a3 : (~(-a3)+1)&0x000000000000FFFF)<<48;
      
      uint64_t res = c0 | c1 | c2 | c3;
      
      /*
      a0 = (res&0x000000000000FFFF)>>0;
      a1 = (res&0x00000000FFFF0000)>>16;
      a2 = (res&0x0000FFFF00000000)>>32;
      a3 = (res&0xFFFF000000000000)>>48;
      printf("FxAdd16x4 res: %f %f %f %f\n", FIX_TO_DOUBLE(a0), FIX_TO_DOUBLE(a1), FIX_TO_DOUBLE(a2), FIX_TO_DOUBLE(a3));
      for (int i = 0; i < 64; ++i) putchar(res >> i & 1 ? '0' : '1'); puts("");
      */
      
      return res;
    };
    case SB_FxSub16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      
      a0= FIX_MINUS(a0, b0);
      a1= FIX_MINUS(a1, b1);
      a2= FIX_MINUS(a2, b2);
      a3= FIX_MINUS(a3, b3);
      
      uint64_t c0 = (uint64_t)(a0 >= 0 ? a0 : (~(-a0)+1)&0x000000000000FFFF)<<0;
      uint64_t c1 = (uint64_t)(a1 >= 0 ? a1 : (~(-a1)+1)&0x000000000000FFFF)<<16;
      uint64_t c2 = (uint64_t)(a2 >= 0 ? a2 : (~(-a2)+1)&0x000000000000FFFF)<<32;
      uint64_t c3 = (uint64_t)(a3 >= 0 ? a3 : (~(-a3)+1)&0x000000000000FFFF)<<48;
      
      return c0 | c1 | c2 | c3;
      //return (uint64_t) _mm_adds_pu16((__m64)ops[0], (__m64)ops[1]);
    };
    case SB_FxRed16x4: {
      uint16_t r0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t r1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t r2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t r3 = (ops[0]&0xFFFF000000000000)>>48;
      
      uint16_t sum;
      
      uint16_t sum0 = r0 + r1;
      if (!((r0 ^ r1) & 0x8000) && ((r0 ^ sum0) & 0x8000) && !(r0 & 0x8000))
        sum0 = 0x7FFF;
      else if (!((r0 ^ r1) & 0x8000) && ((r0 ^ sum0) & 0x8000) && (r0 & 0x8000))
        sum0 = 0x8001;
      
      uint16_t sum1 = r2 + r3;
      if (!((r2 ^ r3) & 0x8000) && ((r2 ^ sum1) & 0x8000) && !(r2 & 0x8000))
        sum1 = 0x7FFF;
      else if (!((r2 ^ r3) & 0x8000) && ((r2 ^ sum1) & 0x8000) && (r2 & 0x8000))
        sum1 = 0x8001;
      
      uint16_t sum2 = sum0 + sum1;
      if (!((sum0 ^ sum1) & 0x8000) && ((sum0 ^ sum2) & 0x8000) && !(sum0 & 0x8000))
        sum2 = 0x7FFF;
      else if (!((sum0 ^ sum1) & 0x8000) && ((sum0 ^ sum2) & 0x8000) && (sum0 & 0x8000))
        sum2 = 0x8001;
      
      if(ops.size() > 1) { //additional op is acc
        sum = sum2 + (uint16_t)ops[1];
        if (!((sum2 ^ (uint16_t)ops[1]) & 0x8000) && ((sum2 ^ sum) & 0x8000) && !(sum2 & 0x8000))
          sum = 0x7FFF;
        else if (!((sum2 ^ (uint16_t)ops[1]) & 0x8000) && ((sum2 ^ sum) & 0x8000) && (sum2 & 0x8000))
          sum = 0x8001;
      } else {
        sum = sum2;
      }
      
      return sum;
    };
    case SB_FxMul16x4: {
      int16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      int16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      int16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      int16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      int16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      
      a0 = FIX_MUL(a0, b0);
      a1 = FIX_MUL(a1, b1);
      a2 = FIX_MUL(a2, b2);
      a3 = FIX_MUL(a3, b3);
      
      uint64_t c0 = (uint64_t)(a0 >= 0 ? a0 : (~(-a0)+1)&0x000000000000FFFF)<<0;
      uint64_t c1 = (uint64_t)(a1 >= 0 ? a1 : (~(-a1)+1)&0x000000000000FFFF)<<16;
      uint64_t c2 = (uint64_t)(a2 >= 0 ? a2 : (~(-a2)+1)&0x000000000000FFFF)<<32;
      uint64_t c3 = (uint64_t)(a3 >= 0 ? a3 : (~(-a3)+1)&0x000000000000FFFF)<<48;
      
      return c0 | c1 | c2 | c3;
    };
    case SB_FxExp16x4: {
      int16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      
      if ((ops.size() > 1) && (ops[1] == 0))
          return ops[0];
      
      double d0 = FIX_TO_DOUBLE(a0);
      double d1 = FIX_TO_DOUBLE(a1);
      double d2 = FIX_TO_DOUBLE(a2);
      double d3 = FIX_TO_DOUBLE(a3);
      
      d0 = exp(d0);
      d1 = exp(d1);
      d2 = exp(d2);
      d3 = exp(d3);
      
      int16_t b0 = DOUBLE_TO_FIX(d0);
      int16_t b1 = DOUBLE_TO_FIX(d1);
      int16_t b2 = DOUBLE_TO_FIX(d2);
      int16_t b3 = DOUBLE_TO_FIX(d3);
      
      uint64_t c0 = ((uint64_t)(b0)<<0)&0x000000000000FFFF;
      uint64_t c1 = ((uint64_t)(b1)<<16)&0x00000000FFFF0000;
      uint64_t c2 = ((uint64_t)(b2)<<32)&0x0000FFFF00000000;
      uint64_t c3 = ((uint64_t)(b3)<<48)&0xFFFF000000000000;
      
      return c0 | c1 | c2 | c3;
    };
    case SB_FxMulX16x4: {
      int16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      int16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      int16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      int16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      int16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      
      a0 = FIX_MUL(a0, b1);
      a1 = FIX_MUL(a1, b0);
      a2 = FIX_MUL(a2, b3);
      a3 = FIX_MUL(a3, b2);
      
      uint64_t c0 = (uint64_t)(a0 >= 0 ? a0 : (~(-a0)+1)&0x000000000000FFFF)<<0;
      uint64_t c1 = (uint64_t)(a1 >= 0 ? a1 : (~(-a1)+1)&0x000000000000FFFF)<<16;
      uint64_t c2 = (uint64_t)(a2 >= 0 ? a2 : (~(-a2)+1)&0x000000000000FFFF)<<32;
      uint64_t c3 = (uint64_t)(a3 >= 0 ? a3 : (~(-a3)+1)&0x000000000000FFFF)<<48;
      
      return c0 | c1 | c2 | c3;
    };
    case SB_FxAcc16x4: {
      int16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      int16_t b0 = (accum&0x000000000000FFFF)>>0;
      int16_t b1 = (accum&0x00000000FFFF0000)>>16;
      int16_t b2 = (accum&0x0000FFFF00000000)>>32;
      int16_t b3 = (accum&0xFFFF000000000000)>>48;
      
      a0 = FIX_ADD(a0, b0);
      a1 = FIX_ADD(a1, b1);
      a2 = FIX_ADD(a2, b2);
      a3 = FIX_ADD(a3, b3);
      
      uint64_t c0 = (uint64_t)(a0 >= 0 ? a0 : (~(-a0)+1)&0x000000000000FFFF)<<0;
      uint64_t c1 = (uint64_t)(a1 >= 0 ? a1 : (~(-a1)+1)&0x000000000000FFFF)<<16;
      uint64_t c2 = (uint64_t)(a2 >= 0 ? a2 : (~(-a2)+1)&0x000000000000FFFF)<<32;
      uint64_t c3 = (uint64_t)(a3 >= 0 ? a3 : (~(-a3)+1)&0x000000000000FFFF)<<48;
      
      accum = c0 | c1 | c2 | c3;
      
      uint64_t res = accum;
      
      if(ops[1]==2 || ops[1]==3) {
        discard=1;
      } 
      if(ops[1]!=0 && ops[1]!=2) {
        accum=0;
      }
      return res;
    };
    case SB_FxAddSub16x4: {
      uint16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      uint16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      uint16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      uint16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      uint16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      uint16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      uint16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      uint16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      
      a0 = FIX_ADD(a0, a1);
      a1 = FIX_MINUS(b0, b1);
      a2 = FIX_ADD(a2, a3);
      a3 = FIX_MINUS(b2, b3);
      
      uint64_t c0 = (uint64_t)(a0 >= 0 ? a0 : (~(-a0)+1)&0x000000000000FFFF)<<0;
      uint64_t c1 = (uint64_t)(a1 >= 0 ? a1 : (~(-a1)+1)&0x000000000000FFFF)<<16;
      uint64_t c2 = (uint64_t)(a2 >= 0 ? a2 : (~(-a2)+1)&0x000000000000FFFF)<<32;
      uint64_t c3 = (uint64_t)(a3 >= 0 ? a3 : (~(-a3)+1)&0x000000000000FFFF)<<48;
      
      return c0 | c1 | c2 | c3;
    };
    case SB_FxSubAdd16x4: {
      int16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      int16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      int16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      int16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      int16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      
      a0 = (int16_t) (FIX_MINUS(a0, a1));
      a1 = (int16_t) (FIX_ADD(b0, b1));
      a2 = (int16_t) (FIX_MINUS(a2, a3));
      a3 = (int16_t) (FIX_ADD(b2, b3));
      
      uint64_t c0 = (uint64_t)(a0 >= 0 ? a0 : (~(-a0)+1)&0x000000000000FFFF)<<0;
      uint64_t c1 = (uint64_t)(a1 >= 0 ? a1 : (~(-a1)+1)&0x000000000000FFFF)<<16;
      uint64_t c2 = (uint64_t)(a2 >= 0 ? a2 : (~(-a2)+1)&0x000000000000FFFF)<<32;
      uint64_t c3 = (uint64_t)(a3 >= 0 ? a3 : (~(-a3)+1)&0x000000000000FFFF)<<48;
      
      uint64_t res =  c0 | c1 | c2 | c3;
      
      return res;
    };
    case SB_FxRedCom16x4: {
      int16_t a0 = (ops[0]&0x000000000000FFFF)>>0;
      int16_t a1 = (ops[0]&0x00000000FFFF0000)>>16;
      int16_t a2 = (ops[0]&0x0000FFFF00000000)>>32;
      int16_t a3 = (ops[0]&0xFFFF000000000000)>>48;
      int16_t b0 = (ops[1]&0x000000000000FFFF)>>0;
      int16_t b1 = (ops[1]&0x00000000FFFF0000)>>16;
      int16_t b2 = (ops[1]&0x0000FFFF00000000)>>32;
      int16_t b3 = (ops[1]&0xFFFF000000000000)>>48;
      
      a0 = FIX_ADD(a0, a2);
      a1 = FIX_ADD(a1, a3);
      a2 = FIX_ADD(b0, b2);
      a3 = FIX_ADD(b1, b3);
      
      uint64_t c0 = (uint64_t)(a0 >= 0 ? a0 : (~(-a0)+1)&0x000000000000FFFF)<<0;
      uint64_t c1 = (uint64_t)(a1 >= 0 ? a1 : (~(-a1)+1)&0x000000000000FFFF)<<16;
      uint64_t c2 = (uint64_t)(a2 >= 0 ? a2 : (~(-a2)+1)&0x000000000000FFFF)<<32;
      uint64_t c3 = (uint64_t)(a3 >= 0 ? a3 : (~(-a3)+1)&0x000000000000FFFF)<<48;
      
      uint64_t res = c0 | c1 | c2 | c3;
      
      return res;
    };
    case SB_FxMul32x2: {
      int32_t a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      int32_t a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      int32_t b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      int32_t b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      int64_t im0 = ((int64_t)a0 * (int64_t)b0) >> 14; // 14 fractional bits 
      int32_t m0 = im0 > (int64_t)0x000000007FFFFFFF ? (int32_t)0x7FFFFFFF : (im0 < (int64_t)0xFFFFFFFF80000001 ? (int32_t)0x80000001 : im0);
      
      int64_t im1 = ((int64_t)a1 * (int64_t)b1) >> 14;
      int32_t m1 = im1 > (int64_t)0x000000007FFFFFFF ? (int32_t)0x7FFFFFFF : (im1 < (int64_t)0xFFFFFFFF80000001 ? (int32_t)0x80000001 : im1);
      
      uint64_t c0 = ((uint64_t)(m0)<<0)&0x00000000FFFFFFFF;
      uint64_t c1 = ((uint64_t)(m1)<<32)&0xFFFFFFFF00000000;
      
      return c0 | c1;
    };
    case SB_FxAdd32x2: {
      uint32_t a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t a1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      uint32_t b0 = (ops[1]&0x00000000FFFFFFFF)>>0;
      uint32_t b1 = (ops[1]&0xFFFFFFFF00000000)>>32;
      
      uint32_t sum;
      
      sum = a0 + b0;
      if (!((a0 ^ b0) & 0x80000000) && ((a0 ^ sum) & 0x80000000) && !(a0 & 0x80000000))
        a0 = 0x7FFFFFFF;
      else if (!((a0 ^ b0) & 0x80000000) && ((a0 ^ sum) & 0x80000000) && (a0 & 0x80000000))
        a0 = 0x80000001;
      else
        a0 = sum;
      
      sum = a1 + b1;
      if (!((a1 ^ b1) & 0x80000000) && ((a1 ^ sum) & 0x80000000) && !(a1 & 0x80000000))
        a1 = 0x7FFFFFFF;
      else if (!((a1 ^ b1) & 0x80000000) && ((a1 ^ sum) & 0x80000000) && (a1 & 0x80000000))
        a1 = 0x80000001;
      else
        a1 = sum;
      
      uint64_t c0 = ((uint64_t)(a0))<<0 & 0x00000000FFFFFFFF;
      uint64_t c1 = ((uint64_t)(a1))<<32 & 0xFFFFFFFF00000000;
      return c0 | c1;
    };
    case SB_FxRed32x2: {
      uint32_t r0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t r1 = (ops[0]&0xFFFFFFFF00000000)>>32;
      
      uint32_t sum;
      
      uint32_t sum0 = r0 + r1;
      if (!((r0 ^ r1) & 0x80000000) && ((r0 ^ sum0) & 0x80000000) && !(r0 & 0x80000000))
        sum0 = 0x7FFFFFFF;
      else if (!((r0 ^ r1) & 0x80000000) && ((r0 ^ sum0) & 0x80000000) && (r0 & 0x80000000))
        sum0 = 0x80000001;
      
      if(ops.size() > 1) { //additional op is acc
        sum = sum0 + (uint32_t)ops[1];
        if (!((sum0 ^ (uint32_t)ops[1]) & 0x80000000) && ((sum0 ^ sum) & 0x80000000) && !(sum0 & 0x80000000))
          sum = 0x7FFFFFFF;
        else if (!((sum0 ^ (uint32_t)ops[1]) & 0x80000000) && ((sum0 ^ sum) & 0x80000000) && (sum0 & 0x80000000))
          sum = 0x80000001;
      } else {
        sum = sum0;
      }
      
      return sum;
    };
    case SB_DupLow32: {
      uint32_t a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      
      uint64_t c0 = (uint64_t)(a0)<<0;
      uint64_t c1 = (uint64_t)(a0 & 0x00000000FFFFFFFF)<<32;
      
      return c0 | c1;
    };
    case SB_DupHigh32: {
      uint32_t a0 = (ops[0]&0xFFFFFFFF00000000)>>32;
      
      uint64_t c0 = (uint64_t)(a0)<<0;
      uint64_t c1 = (uint64_t)(a0 & 0x00000000FFFFFFFF)<<32;
      
      return c0 | c1;
    };
    case SB_ConcatLow32: {
      uint32_t a0 = (ops[0]&0x00000000FFFFFFFF)>>0;
      uint32_t a1 = (ops[1]&0x00000000FFFFFFFF)>>0;
      
      uint64_t c0 = (uint64_t)(a0)<<32;
      uint64_t c1 = (uint64_t)(a1)<<0;
      
      return c0 | c1;
      
    };
    case SB_FAdd64: {
      double a = as_double(ops[0]);
      double b = as_double(ops[1]);
      double c = a+b;
      return as_uint64(c); 
    };
    case SB_FSub64: {
      double a = as_double(ops[0]);
      double b = as_double(ops[1]);
      double c = a - b;
      return as_uint64(c); 
    };
    case SB_FMul64: {
      double a = as_double(ops[0]);
      double b = as_double(ops[1]);
      double c = a*b;
      return as_uint64(c); 
    };
    case SB_FxExp64: {
      if ((ops.size() > 1) && (ops[1] == 0))
          return ops[0];
      
      double d = FIX_TO_DOUBLE(ops[0]);
      
      d = exp(d);
      
      uint64_t b = DOUBLE_TO_FIX(d);
      
      
      return b;
    };
    case SB_Select: {
      return ops[2]==0 ? ops[0] : ops[1];
    };
    case SB_Merge: {
      if(ops[1] > ops[0]) {
      back_array[1] = true;
      back_array[0] = false;
      return ops[0];
      }
      
      else {
      back_array[1] = false;
      back_array[0] = true;
      return ops[1];
      }
      
      
      //return back_array[1]; //Vignesh - when you want this instruction to backpressurize on an edge, I think we should put backpressure on the unpropogated edge... but?
    };
    case SB_MergeSentinal: {
      #define SENTINAL ( ((uint64_t)1)<<63)
      
      if(ops[0]==SENTINAL && ops[1]==SENTINAL) {
          back_array[0]=0;
          back_array[1]=0;
          discard=true;
          return -1; // don't generate anything 
      } 
      else if(ops[0]<=ops[1] || ops[1]==SENTINAL){
          back_array[0]=0;
          back_array[1]=1;
          return ops[0];
      }
      else if(ops[1]<ops[0] || ops[0]==SENTINAL){
          back_array[0]=1;
          back_array[1]=0;
          return ops[1];
      } else {
          assert(0 && "not possible");
      }
    };
    case SB_Index_match: {
      // Control Signal Definitions
      int do_reset   = 1 << 0;
      int do_discard = 1 << 1;
      int dont_accum = 1 << 2;
      int bp_op1     = 1 << 3;
      int bp_op2     = 1 << 4;
      
      #define SENTINAL ( ((uint64_t)1)<<63)
      
      if(ops[0]==SENTINAL && ops[1]==SENTINAL) {
          back_array[0]=0;
          back_array[1]=0;
          return dont_accum | do_reset;
      } 
      else if(ops[0]==ops[1]){
          back_array[0]=0;
          back_array[1]=0;
          return do_discard;
      }
      else if(ops[0]<ops[1] || ops[1]==SENTINAL){
          back_array[0]=0;
          back_array[1]=1;
          return dont_accum | do_discard | bp_op2;
      }
      else if(ops[1]<ops[0] || ops[0]==SENTINAL){
          back_array[0]=1;
          back_array[1]=0;
          return dont_accum | do_discard | bp_op1;
      } else {
          assert(0 && "not possible");
      }
    };
    case SB_ICmp: {
      return ops[0] > ops[1];
    };
    case SB_Hold: {
      bool done = ops[1] & (1 << 0);
      bool do_update = ops[1] & (1 << 1);
      
      uint64_t ret = reg[0];
      // should I use accum?
      if(do_update) {
          reg[0] = ops[0];
          ret = reg[0];
      }
      if(done) {
          reg[0]=0;
      } else {
          discard=true;
      }
      
      return ret;
    };
    case SB_Nor: {
      return !(ops[0] | ops[1]);
    };
    case SB_Phi: {
    };
    case SB_And3: {
      return ops[0] & ops[1] & ops[2]; 
    };
    case SB_And: {
      return ops[0] & ops[1]; 
    };
    case SB_ICmpG: {
      if (ops[0] > reg[0]) {
          reg[0] = ops[0];
          return 1;
      } else {
          return 0;
      }
    };
    case SB_Or: {
      return ops[0] | ops[1]; 
    };
    case SB_Xor: {
      return ops[0] ^ ops[1]; 
    };
    case SB_Not: {
      return !ops[0]; 
    };
    case SB_Copy: {
      return ops[0];
    };
    case SB_ICmpEQ: {
      return ops[0] == ops[1]; 
    };
    case SB_ICmpNE: {
      return ops[0] != ops[1]; 
    };
    case SB_Switch: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_Add: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_Sub: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_Mul: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_UDiv: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_SDiv: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_URem: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_SRem: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_IMax: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_IMin: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_SMax: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_SMin: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FAdd: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FSub: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FMul: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FDiv: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FRem: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_Sqrt: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FSin: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FCos: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FMax: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FMin: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_SExt: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_Shl: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_LShr: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_AShr: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_Ternary: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_ICmpUGT: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_ICmpUGE: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_ICmpULT: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_ICmpULE: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_ICmpSGT: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_ICmpSGE: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_ICmpSLT: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_ICmpSLE: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FCmpOEQ: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FCmpONE: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FCmpOGT: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FCmpOGE: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FCmpOLT: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_FCmpOLE: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    default: assert(0 && "Instruction not defined (for this bitwidth)"); return 1;
  }

}

uint32_t SB_CONFIG::execute32(sb_inst_t inst, std::vector<uint32_t>& ops, std::vector<uint32_t>& reg, uint64_t& discard, std::vector<bool>& back_array) {
uint32_t& accum = reg[0]; 
  assert(ops.size() <= 4); 
  assert(ops.size() <=  (unsigned)(num_ops[inst]+1)); 
  if((ops.size() > (unsigned)num_ops[inst]) && (ops[ops.size()] == 0)) { 
    return ops[0];
  }
  switch(inst) {
    case SB_FMul32: {
      float a = as_float(ops[0]);
      float b = as_float(ops[1]);
      
      union {
        float val;
        uint32_t res;
      } ri;
      ri.val = a * b;
      
      return ri.res;
    };
    case SB_FAdd32: {
      float a = as_float(ops[0]);
      float b = as_float(ops[1]);
      
      union {
        float val;
        uint32_t res;
      } ri;
      ri.val = a + b;
      
      return ri.res;
    };
    case SB_FSub32: {
      float a = as_float(ops[0]);
      float b = as_float(ops[1]);
      
      union {
        float val;
        uint32_t res;
      } ri;
      ri.val = a - b;
      
      return ri.res;
    };
    case SB_Add32: {
      return ops[0] + ops[1];
    };
    case SB_Mul32: {
      return ops[0] * ops[1];
    };
    default: assert(0 && "Instruction not defined (for this bitwidth)"); return 1;
  }

}

uint16_t SB_CONFIG::execute16(sb_inst_t inst, std::vector<uint16_t>& ops, std::vector<uint16_t>& reg, uint64_t& discard, std::vector<bool>& back_array) {
uint16_t& accum = reg[0]; 
  assert(ops.size() <= 4); 
  assert(ops.size() <=  (unsigned)(num_ops[inst]+1)); 
  if((ops.size() > (unsigned)num_ops[inst]) && (ops[ops.size()] == 0)) { 
    return ops[0];
  }
  switch(inst) {
    case SB_Add16: {
      return ops[0] + ops[1];
    };
    case SB_Acc16: {
      // Decode Control Logic
      bool do_reset   = ops[1] & (1 << 0);
      bool do_discard = ops[1] & (1 << 1);
      bool dont_accum = ops[1] & (1 << 2);
      // bool do_set_input   = ops[1] & (1 << 3);
      
      if(!dont_accum) {
        accum+=ops[0];
      }
      
      uint64_t ret = accum;
      
      if(do_discard) {
        discard=1;
      } 
      if(do_reset) {
        accum=0;
      }
      return ret; 
    };
    case SB_Mul16: {
      return ops[0] * ops[1];
    };
    case SB_Keep16: {
      if (!ops[1]) {
        discard = 1;
      }
      
      return ops[0];
      
    };
    case SB_Min16: {
      if(ops[0]<ops[1]){
        return ops[0];
      } else {
        return ops[1];
      }
    };
    case SB_IndexMatch16: {
      #define SENTINAL16 ( ((uint16_t)1)<<15)
      
      if(ops[0]==SENTINAL16 && ops[1]==SENTINAL16) {
          back_array[0]=0;
          back_array[1]=0;
          // return 0; // should not generate any backpressure
          return 3;
      } 
      else if(ops[0]==ops[1]){
          back_array[0]=0;
          back_array[1]=0;
          return 0;
      }
      else if(ops[0]<ops[1] || ops[1]==SENTINAL16){
          back_array[0]=0;
          back_array[1]=1;
          return 1;
      }
      else if(ops[1]<ops[0] || ops[0]==SENTINAL16){
          back_array[0]=1;
          back_array[1]=0;
          return 2;
      } else {
          assert(0 && "not possible");
      }
    };
    case SB_Control16: {
      uint16_t ret;
      
      if(ops[0]==0) {
        back_array[0]=0;
        back_array[1]=0;
      } else if(ops[1]==0) {
        back_array[0]=0;
        back_array[1]=0;
      } else {
        if(ops[0]<ops[1]){
      	back_array[0]=0;
      	back_array[1]=1;
        } else {
      	back_array[0]=1;
      	back_array[1]=0;
        }
      }
      
      if(ops[0]==SENTINAL16 && ops[1]==SENTINAL16){
        back_array[0]=0;
        back_array[1]=0;
      }
      
      ret = back_array[0] << 1 | back_array[1] << 2;
      
      if(ops[0]<ops[1]){
        ret = ret | 1;
      } else if(ops[0]==SENTINAL16 && ops[1]==SENTINAL16){
        ret = 3; // 0,1,1(this will always be 0)
      }
        
      
      return ret;
    };
    case SB_ICmpEq16: {
      return ops[0] == ops[1]; 
    };
    case SB_ReLU16: {
      if(ops[0]>0)
        return ops[0];
      else
        return 0;
    };
    case SB_ICmpNE16: {assert(0 && "Instruction Not Implemented, add it to insts folder");};
    case SB_Select16: {
      return ((ops[2]&1)==0) ? ops[0] : ops[1];
    };
    default: assert(0 && "Instruction not defined (for this bitwidth)"); return 1;
  }

}

uint8_t SB_CONFIG::execute8(sb_inst_t inst, std::vector<uint8_t>& ops, std::vector<uint8_t>& reg, uint64_t& discard, std::vector<bool>& back_array) {
uint8_t& accum = reg[0]; 
  assert(ops.size() <= 4); 
  assert(ops.size() <=  (unsigned)(num_ops[inst]+1)); 
  if((ops.size() > (unsigned)num_ops[inst]) && (ops[ops.size()] == 0)) { 
    return ops[0];
  }
  switch(inst) {
    case SB_Add8: {
      return ops[0] + ops[1];
    };
    default: assert(0 && "Instruction not defined (for this bitwidth)"); return 1;
  }

}

