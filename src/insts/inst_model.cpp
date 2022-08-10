#include "inst_model.h"

#include <stdlib.h>
#include <string.h>

#include <map>
#include <set>
#include <sstream>

#include "../utils/model_parsing.h"
#include "dsa/debug.h"

using namespace dsa;
using namespace std;

// constructor based on input stream
InstModel::InstModel(char* filename) : filename_(filename) {
  ifstream ifs(filename, ios::in);

  base_folder = filename_;
  size_t r = base_folder.rfind('/');
  if (r == std::string::npos) {
    base_folder = "./";
  } else {
    base_folder = base_folder.substr(0, r) + "/";
  }

  if (ifs.fail()) {
    cerr << "Could Not Open: " << filename << "\n";
    return;
  }

  std::set<string> existing_insts;

  char line[512];
  while (ifs.good()) {
    // string line;
    ifs.getline(line, 512);

    string str_line = string(line);

    ModelParsing::trim(str_line);

    // Empty line or the first line
    if (str_line[0] == '#' || str_line.empty()) continue;

    char* token;
    token = strtok(line, " ");
    string str_name(token);

    if (existing_insts.count(str_name)) continue;
    existing_insts.insert(str_name);

    ConfigInst inst_;
    ConfigInst *inst = &inst_;
    inst->name(str_name);

    token = strtok(nullptr, " ");
    inst->bitwidth(atoi(token));

    token = strtok(nullptr, " ");
    inst->num_ops(atoi(token));

    token = strtok(nullptr, " ");
    inst->num_values(atoi(token));

    token = strtok(nullptr, " ");
    inst->latency(atoi(token));

    token = strtok(nullptr, " ");
    inst->throughput(atoi(token));

    _instList.push_back(inst_);
  }
}

void InstModel::readModel(char* filename) {
  // Open the CSV Power & Area model
  ifstream ifs(filename, ios::in);

  if (ifs.fail()) {
    cerr << "Could Not Open: " << filename << "\n";
    return;
  }

  std::map<std::string, std::vector<double>> inst_to_power_area;

  // Parsing the instructions
  char line[1024];
  while (ifs.good()) {
    // reading
    DSA_LOG(PA_MODEL) << "reading and parse the csv file\n";
    // fetch line
    ifs.getline(line, 1024);

    // string line
    std::string str_line = string(line);

    // trim
    ModelParsing::trim(str_line);

    // Empty line or the first line
    if (str_line[0] == '#' || str_line.empty()) continue;

    // splited line
    std::vector<std::string> tokens;
    ModelParsing::split(line, ',', tokens);

    if (ModelParsing::is_number(tokens[2])) {
      char* end = 0;
      std::string operation = tokens[0];
      std::string dtype = tokens[1];
      int bitwidth = 64;
      int opcode = tokens[2].empty() ? -1 : strtol(tokens[2].c_str(), &end, 10);
      int num_operands = tokens[3].empty() ? -1 : strtol(tokens[3].c_str(), &end, 10);
      int num_result = tokens[4].empty() ? -1 : strtol(tokens[4].c_str(), &end, 10);
      int latency = tokens[5].empty() ? -1 : strtol(tokens[5].c_str(), &end, 10);
      int throughput = tokens[6].empty() ? -1 : strtol(tokens[6].c_str(), &end, 10);
      double power = strtod(tokens[7].c_str(), &end);
      double area = strtod(tokens[8].c_str(), &end);
      double totalLut = strtod(tokens[9].c_str(), &end);
      double logicLut = strtod(tokens[10].c_str(), &end);
      double ramLut = strtod(tokens[11].c_str(), &end);
      double srl = strtod(tokens[12].c_str(), &end);
      double flipFlop = strtod(tokens[13].c_str(), &end);
      double ramB36 = strtod(tokens[14].c_str(), &end);
      double ramB18 = strtod(tokens[15].c_str(), &end);
      double uRam = strtod(tokens[16].c_str(), &end);
      double dsp = strtod(tokens[17].c_str(), &end);

      std::string inst_name = operation;

      if (!ModelParsing::StartsWith(dtype, "bitwise")) {
        inst_name += "_" + dtype;
        std::string bitwidth_str = dtype;
        bitwidth_str.erase(0, 1); // Remove First Character
        if (bitwidth_str.find("x") != std::string::npos) {
          std::vector<std::string> bitwidth_tokens;
          ModelParsing::split(bitwidth_str, 'x', bitwidth_tokens);

          bitwidth = strtod(bitwidth_tokens[0].c_str(), &end) * strtod(bitwidth_tokens[1].c_str(), &end);
        } else {
          bitwidth = strtod(bitwidth_str.c_str(), &end);
        }
      }

      bitwidth = std::max(1, bitwidth);

      ConfigInst inst_;
      ConfigInst* inst = &inst_;
      inst->name(inst_name);
      inst->dtype(dtype);
      inst->bitwidth(bitwidth);
      inst->opcode(opcode);
      inst->num_ops(num_operands);
      inst->num_values(num_result);
      inst->latency(latency);
      inst->throughput(throughput);
      inst->power(power);
      inst->area(area);
      inst->total_lut(totalLut);
      inst->logic_lut(logicLut);
      inst->ram_lut(ramLut);
      inst->srl(srl);
      inst->flip_flop(flipFlop);
      inst->ram_b36(ramB36);
      inst->ram_b18(ramB18);
      inst->u_ram(uRam);
      inst->dsp(dsp);
      instList_.push_back(inst_);
    }
  }
}

// constructor based on the input csv file
void InstModel::PowerAreaModel(char* filename) {
  // Open the CSV Power & Area model
  ifstream ifs(filename, ios::in);

  if (ifs.fail()) {
    cerr << "Could Not Open: " << filename << "\n";
    return;
  }

  std::map<std::string, std::vector<double>> inst_to_power_area;

  // Parsing the instructions
  char line[1024];
  while (ifs.good()) {
    // reading
    DSA_LOG(PA_MODEL) << "reading and parse the csv file\n";
    // fetch line
    ifs.getline(line, 1024);

    // string line
    std::string str_line = string(line);

    // trim
    ModelParsing::trim(str_line);

    // Empty line or the first line
    if (str_line[0] == '#' || str_line.empty()) continue;

    // splited line
    std::vector<std::string> tokens;
    ModelParsing::split(line, ',', tokens);

    if (ModelParsing::is_number(tokens[5]) && ModelParsing::is_number(tokens[6])) {
      char* end = 0;
      
      std::string inst_name = tokens[4];
      double area = strtod(tokens[5].c_str(), &end);
      double power = strtod(tokens[6].c_str(), &end);
      double totalLut = strtod(tokens[7].c_str(), &end);
      double logicLut = strtod(tokens[7].c_str(), &end);
      double flipFlop = strtod(tokens[8].c_str(), &end);
    
      if (ModelParsing::StartsWith(inst_name, "FLOAT") ||
          ModelParsing::StartsWith(inst_name, "FIXED")) {
        FuType* fuType = new FuType();
        fuType->name(inst_name);
        fuType->area(area);
        fuType->power(power);
        fuType->logic_lut(logicLut);
        fuType->flip_flop(flipFlop);

        _fuTypeList.push_back(fuType);
        continue;  // skip parsing the float/fixed function unit
      }

      inst_to_power_area.insert(pair<std::string, vector<double>>(
          inst_name, std::vector<double>{power, area, logicLut, flipFlop}));
    } else {
      continue;
    }
  }

  // add power and area to decomposed insts, unsigned/signed/fixed

  string prefix_fixed_str[6] = {"L", "H", "I", "S", "U", "Fx"};
  // High, Low, Int, Signed, Unsigned, Fixed are assumed to be the same
  auto fixed_size_map = inst_to_power_area;
  for (auto inst_pa : fixed_size_map) {
    string inst_name = inst_pa.first;
    string last_two_str = inst_name.substr(inst_name.length() - 2);
    DSA_LOG(PA_MODEL) << "the outer loop -- inst name = " << inst_name << "\n";
    if (inst_name[0] == 'F') {  // Floating Unit
      // Add floating-point 32x2 here
      if (last_two_str == "32") {
       std::vector<double> power_area = inst_to_power_area[inst_name];
       std::vector<double> power_areax2 =
            std::vector<double>{power_area[0] * 2, power_area[1] * 2, power_area[2] * 2, power_area[3] * 2};
        inst_to_power_area.insert(
            pair<string, std::vector<double>>(inst_name + "x2", power_areax2));
      }
      DSA_LOG(PA_MODEL) << "deal with the floating point\n";
      continue;  // Skip the rest floating-point inst
    } else {     // Fixed Instruction
      // Add sign prefix to every fixed inst
      std::vector<double> power_area = inst_to_power_area[inst_name];
      for (auto prefix : prefix_fixed_str) {
        stringstream sign_inst_ss;
        sign_inst_ss << prefix << inst_name;
        inst_to_power_area.insert(
            pair<std::string, std::vector<double>>(sign_inst_ss.str(), power_area));
        DSA_LOG(PA_MODEL) << "length of map = " << inst_to_power_area.size() << " -- "
                      << prefix << " : deal with the add sign bit to fixed point\n";
      }
      // Add power area to decomposed instruction
      char last_char = inst_name.back();
      if (last_char == '8' || last_two_str == "16" || last_two_str == "32") {
        DSA_LOG(PA_MODEL) << "length of map = " << inst_to_power_area.size() << " -- "
                      << "deal with the decomposed instructions\n";
        if (last_char == '8') {
          string func_name = inst_name.substr(0, inst_name.length() - 1);
          int decomposers[3] = {2, 4, 8};
          for (const int& decomposer : decomposers) {
            stringstream decomp_ss;
            decomp_ss << inst_name << "x" << decomposer;
            string decomp_inst_name = decomp_ss.str();
            std::vector<double> power_area_x_decomposer = std::vector<double>{
                power_area[0] * decomposer, power_area[1] * decomposer, power_area[2] * decomposer, power_area[3] * decomposer};
            inst_to_power_area.insert(pair<std::string, std::vector<double>>(
                decomp_inst_name, power_area_x_decomposer));
            for (auto prefix : prefix_fixed_str) {
              stringstream sign_decomp_ss;
              sign_decomp_ss << prefix << decomp_inst_name;
              DSA_LOG(PA_MODEL) << "signed decoup inst = " << sign_decomp_ss.str() << "\n";
              inst_to_power_area.insert(pair<std::string,  std::vector<double>>(
                  sign_decomp_ss.str(), power_area_x_decomposer));
            }
          }
          continue;
        }
        if (last_two_str == "16") {
          string func_name = inst_name.substr(0, inst_name.length() - 2);
          int decomposers[2] = {2, 4};
          for (const int& decomposer : decomposers) {
            stringstream decomp_ss;
            decomp_ss << inst_name << "x" << decomposer;
            string decomp_inst_name = decomp_ss.str();
            std::vector<double> power_area_x_decomposer = std::vector<double>{
                power_area[0] * decomposer, power_area[1] * decomposer, power_area[2] * decomposer, power_area[3] * decomposer};
            inst_to_power_area.insert(pair<std::string, std::vector<double>>(
                decomp_inst_name, power_area_x_decomposer));
            for (auto prefix : prefix_fixed_str) {
              stringstream sign_decomp_ss;
              sign_decomp_ss << prefix << decomp_inst_name;
              DSA_LOG(PA_MODEL) << "signed decoup inst = " << sign_decomp_ss.str() << "\n";
              inst_to_power_area.insert(pair<std::string, std::vector<double>>(
                  sign_decomp_ss.str(), power_area_x_decomposer));
            }
          }
          continue;
        }
        if (last_two_str == "32") {
          string func_name = inst_name.substr(0, inst_name.length() - 2);
          int decomposer = 2;
          stringstream decomp_ss;
          decomp_ss << inst_name << "x" << decomposer;
          string decomp_inst_name = decomp_ss.str();
          std::vector<double> power_area_x_decomposer = std::vector<double>{
                power_area[0] * decomposer, power_area[1] * decomposer, power_area[2] * decomposer, power_area[3] * decomposer};
          inst_to_power_area.insert(pair<std::string, std::vector<double>>(
              decomp_inst_name, power_area_x_decomposer));
          for (auto prefix : prefix_fixed_str) {
            stringstream sign_decomp_ss;
            sign_decomp_ss << prefix << decomp_inst_name;
            DSA_LOG(PA_MODEL) << "signed decoup inst = " << sign_decomp_ss.str() << "\n";
            inst_to_power_area.insert(pair<std::string, std::vector<double>>(
                sign_decomp_ss.str(), power_area_x_decomposer));
          }
          continue;
        }

      } else {
        continue;
      }
    }
  }

  // Adding the default instruction if ending with 64
  fixed_size_map = inst_to_power_area;
  for (auto inst_pa : fixed_size_map) {
    string inst_name = inst_pa.first;
    string last_two_str = inst_name.substr(inst_name.length() - 2);
    if (last_two_str == "64") {
      string func_name = inst_name.substr(0, inst_name.length() - 2);
      std::vector<double> power_area = inst_to_power_area[inst_name];
      inst_to_power_area.insert(
          pair<string, std::vector<double>>(func_name, power_area));
    }
    DSA_LOG(PA_MODEL) << "deal with instruction with out bitwidth\n";
  }

  // Add the power / area to inst list
  for (auto inst : _instList) {
    string inst_name = inst.name();
    if (inst_to_power_area.count(inst_name) > 0) {
      auto power_area = inst_to_power_area[inst_name];
      inst.area(power_area[1]);
      inst.power(power_area[0]);
      inst.logic_lut(power_area[2]);
      inst.flip_flop(power_area[3]);
    }
  }
}


void InstModel::newPrintCFiles(char* header_file, char* cpp_file) {
  int max_opcode = -1;
  for (auto inst : instList_) {
    if (inst.opcode() > max_opcode) {
      max_opcode = inst.opcode();
    }
  }

  // -------------------------print header file -----------------------------
  ofstream ofs(header_file, ios::out);
  ofs << "//This file generated from inst_model.cpp -- Do not edit.  Do not commit to "
         "repo.\n"
         "#ifndef __SS_INST_H__\n"
         "#define __SS_INST_H__\n"
         "\n"
         "#include <string>\n"
         "#include <string.h>\n"
         "#include <cstring>\n"
         "#include <assert.h>\n"
         "#include <iostream>\n"
         "#include <vector>\n"
         "#include <complex>\n"
         "#include <algorithm>\n"
         "#include <math.h>\n"
         "\n"
         "#include \"dsa/debug.h\"\n"

         "#define FIX_MAX ((1 << 15) - 1)\n"
         "#define FIX_MIN (-FIX_MAX)\n"
         "#define FIX_TRUNC(x) (x > FIX_MAX ? FIX_MAX : (x < FIX_MIN ? FIX_MIN : x))\n"
         "\n"
         "#define FRAC_BITS 11\n"
         "#define DELTA (((double)1.0) / (1 << FRAC_BITS))\n"
         "#define FLOAT_MAX (FIX_MAX * DELTA)\n"
         "#define FLOAT_MIN (FIX_MIN * DELTA)\n"
         "#define FLOAT_TRUNC(x) (x > FLOAT_MAX ? FLOAT_MAX : (x < FLOAT_MIN ? FLOAT_MIN "
         ": x))\n"
         "\n"
         "#define DOUBLE_TO_FIX(x) ((int)(FLOAT_TRUNC(x) / DELTA))\n"
         "#define FIX_TO_DOUBLE(x) (x * DELTA)\n"
         "\n"
         "#define FIX_ADD(a, b) (FIX_TRUNC((int)a + (int)b))\n"
         "#define FIX_MINUS(a, b) (FIX_ADD(a, -b))\n"
         "#define FIX_MUL(a, b) (FIX_TRUNC(((int)a * (int)b) * DELTA))\n"
         "#define FIX_TAN_H(x) (DOUBLE_TO_FIX(tanh(FIX_TO_DOUBLE(x))))\n"
         "\n"
         "using std::complex;\n"
         "\n"
         "namespace dsa {\n"
         "\n"

         "float    as_float(std::uint32_t ui);\n"
         "uint32_t as_uint32(float f);\n"
         "\n"
         "double    as_double(std::uint64_t ui);\n"
         "uint64_t as_uint64(double f);\n"
         "\n"

         "std::complex<float> as_float_complex(uint64_t ui);\n"
         "uint64_t as_uint64(const std::complex<float> &val);\n"
         "\n"

         "enum OpCode {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "  SS_" << instList_[i].name() << "=" << instList_[i].opcode() << ", \n";
  };
  ofs << "  SS_ERR,\n";
  ofs << "  SS_NUM_TYPES\n};\n\n";

  // Print out the pre-defined Function unit
  ofs << "enum fu_type_t {\n";
  for (unsigned i = 0; i < _fuTypeList.size(); ++i) {
    ofs << "  " << _fuTypeList[i]->name() << ", \n";
  };
  ofs << "  NON_PREDEFINED_FU_TYPE\n};\n";

  ofs << "\n";
  
  
  ofs << "extern int num_ops[" << max_opcode + 3 << "];\n";
  ofs << "extern int bitwidth[" << max_opcode + 3 << "];\n";

  ofs << "\n"
         "// OpCode\n"
         "OpCode inst_from_string(const char* str);\n"
         "const char* name_of_inst(OpCode inst);\n"
         "const char* inst_dtype(OpCode inst);\n"
         "double inst_area(OpCode inst);\n"
         "double inst_power(OpCode inst);\n"
         "double inst_total_lut(OpCode inst);\n"
         "double inst_logic_lut(OpCode inst);\n"
         "double inst_ram_lut(OpCode inst);\n"
         "double inst_srl(OpCode inst);\n"
         "double inst_flip_flop(OpCode inst);\n"
         "double inst_ram_b36(OpCode inst);\n"
         "double inst_ram_b18(OpCode inst);\n"
         "double inst_u_ram(OpCode inst);\n"
         "double inst_dsp(OpCode inst);\n"
         "int inst_lat(OpCode inst);\n"
         "int inst_thr(OpCode inst);\n"
         "int num_values(OpCode inst);\n";
         /*
         "// fu_type_t\n"
         "fu_type_t fu_type_from_string(const char* str);\n"
         "const char* name_of_fu_type(fu_type_t fu_type);\n"
         "double fu_type_area(fu_type_t fu_type);\n"
         "double fu_type_power(fu_type_t fu_type);\n";
         */
        
  // Generate an execute function for all bitwidths
  int bitwidths[4] = {64, 32, 16, 8};
  string types[4] = {"uint64_t", "uint32_t", "uint16_t", "uint8_t"};
  string suffixes[4] = {"64", "32", "16", "8"};

  ofs << "// execute \n";
  for (int i = 0; i < 4; ++i) {
    string dtype = types[i];
    string suffix = suffixes[i];
    ofs << dtype << " execute" << suffix << "(OpCode inst, "
        << "std::vector<" << dtype << ">& ops, "
        << "std::vector<" << dtype << ">& outs, " << dtype << "* reg, "
        << "bool &discard, std::vector<bool>& back_array);\n";
  }

  ofs << "\n"
         "}\n\n"
         "#endif\n";

  ofs.close();

  // -------------------------print cpp file --------------------------------

  ofs.open(cpp_file, ios::out);

  // inst_from_string
  ofs << "//This file generated from inst_model.cpp -- Do not edit.  Do not commit to "
         "repo.\n"
         "#include \""
      << header_file
      << "\"\n\n"

         "float dsa::as_float(std::uint32_t ui) {\n"
         "  float f;\n"
         "  std::memcpy(&f, &ui, sizeof(float));\n"
         "  return f;\n"
         "}\n"
         "\n"

         "uint32_t dsa::as_uint32(float f) {\n"
         "  uint32_t ui;\n"
         "  std::memcpy(&ui, &f, sizeof(float));\n"
         "  return ui;\n"
         "}\n"
         "\n"

         "double dsa::as_double(std::uint64_t ui) {\n"
         "  double f;\n"
         "  std::memcpy(&f, &ui, sizeof(double));\n"
         "  return f;\n"
         "}\n"
         "\n"

         "uint64_t dsa::as_uint64(double f) {\n"
         "  uint64_t ui;\n"
         "  std::memcpy(&ui, &f, sizeof(double));\n"
         "  return ui;\n"
         "}\n"
         "\n"

         "std::complex<float> dsa::as_float_complex(uint64_t val) {\n"
         "  std::complex<float> res;\n"
         "  std::memcpy(&res, &val, sizeof(val));\n"
         "  return res;\n"
         "}\n"
         "\n"

         "uint64_t dsa::as_uint64(const std::complex<float> &val) {\n"
         "  uint64_t res;\n"
         "  std::memcpy(&res, &val, sizeof(val));\n"
         "  return res;\n"
         "}\n"
         "\n"

         "using namespace dsa;\n\n"
         "OpCode dsa::inst_from_string(const char* str) {\n";

  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "  if(strcmp(str,\"" << instList_[i].name() << "\")==0) return SS_"
        << instList_[i].name() << ";\n";
  }
  /*
  ofs << "  else { fprintf(stderr,\"Config Library does not understand "
         "string\\\"%s\\\"\\n\",str); "
         "return SS_ERR;}\n";
         */
  ofs << "  else {return SS_ERR;}\n";
  ofs << "}\n\n";

  // Properties of Instructions

  // name_of_inst
  ofs << "const char* dsa::name_of_inst(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return \"" << instList_[i].name()
        << "\";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n";
  ofs << "}\n\n";

  // Inst dtype
  ofs << "const char* dsa::inst_dtype(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return \"" << instList_[i].dtype()
        << "\";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n";
  ofs << "}\n\n";

  // FUNCTION: inst_lat
  ofs << "int dsa::inst_lat(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].latency()
        << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // FUNCTION: inst_thr
  ofs << "int dsa::inst_thr(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].throughput()
        << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // area of the ssinst
  ofs << "double dsa::inst_area(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].area() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // power of the ssinst
  ofs << "double dsa::inst_power(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].power() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // TotalLut of the ssinst
  ofs << "double dsa::inst_total_lut(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].total_lut() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // LogicLut of the ssinst
  ofs << "double dsa::inst_logic_lut(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].logic_lut() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // RamLut of the ssinst
  ofs << "double dsa::inst_ram_lut(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].ram_lut() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // SRLs of the ssinst
  ofs << "double dsa::inst_srl(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].srl() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // FlipFlop of the ssinst
  ofs << "double dsa::inst_flip_flop(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].flip_flop() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // RamB36 of the ssinst
  ofs << "double dsa::inst_ram_b36(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].ram_b36() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // RamB18 of the ssinst
  ofs << "double dsa::inst_ram_b18(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].ram_b18() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // URam of the ssinst
  ofs << "double dsa::inst_u_ram(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].u_ram() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // DSP of the ssinst
  ofs << "double dsa::inst_dsp(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].dsp() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // FUNCTION: num_values
  ofs << "int dsa::num_values(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i].name() << ": return " << instList_[i].num_values()
        << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  /*
  // Instruction Bitwidth
  ofs << "int dsa::inst_bitwidth(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i]->name() << ": return " << instList_[i]->bitwidth() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\"; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // Instruction Num Ops
  ofs << "int dsa::inst_num_ops(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < instList_.size(); ++i) {
    ofs << "    case "
        << "SS_" << instList_[i]->name() << ": return " << instList_[i]->num_ops() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\"; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";
  */

  // num_ops_array
  ofs << "int dsa::num_ops[" << max_opcode + 3 << "]={\n";
  for (unsigned i = 0; i <= max_opcode; ++i) {
    auto instruction = getInst(i);
    int num_ops = 0;
    if (instruction != nullptr)
      num_ops = instruction->num_ops();

    ofs << num_ops << ", ";
    if (i % 16 == 15) {
      ofs << "\n";
    }
  }
  ofs << "0, 0};\n\n";

  // bitwidth_array
  ofs << "int dsa::bitwidth[" << max_opcode + 3 << "]={\n";
  for (unsigned i = 0; i <= max_opcode; ++i) {
    auto instruction = getInst(i);
    int bitwidth = 0;
    if (instruction != nullptr) {
      bitwidth = instruction->bitwidth();
    }
    ofs << bitwidth << ", ";
    if (i % 16 == 15) {
      ofs << "\n";
    }
  }
  ofs << "0, 0};\n\n";

  // FUNCTION: execute()
  for (int i = 0; i < 4; ++i) {
    int bitwidth = bitwidths[i];
    string dtype = types[i];
    string suffix = suffixes[i];

    ofs << dtype << " dsa::execute" << suffix << "(OpCode inst, "
        << "std::vector<" << dtype << ">& ops, "
        << "std::vector<" << dtype << ">& outs, " << dtype << " *reg, "
        << "bool &discard, std::vector<bool>& back_array) {\n";

    // somwhere below is an implementation of pass through, is it though? (tony, 2018)

    ofs << "  " << dtype
        << "& accum = reg[0];\n"
           "  (void) accum;\n"
           "  DSA_CHECK(ops.size() <= 4);\n"
           "  DSA_CHECK(ops.size() <=  (unsigned) (num_ops[inst]+1));\n"
           "  switch(inst) {\n";
    for (unsigned i = 0; i < instList_.size(); ++i) {
      ConfigInst* inst = &instList_[i];

      if (inst->bitwidth() != bitwidth)
        continue;  // TODO: later implement autovectorization

      ofs << "    case "
          << "SS_" << inst->name() << ": {";

      string inst_code_name = base_folder + "insts" + suffix + "/" + inst->name() + ".h";
      ifstream f(inst_code_name.c_str());

      if (f.good()) {
        std::string line;
        ofs << "\n";
        while (std::getline(f, line)) {
          ofs << "      " << line << "\n";
        }
        ofs << "    };\n";
      } else {
        ofs << "DSA_CHECK(false) << name_of_inst(inst) << \" not implemented, add it to insts folder\";";
        ofs << "};\n";
      }
    }
    ofs << "    default: DSA_CHECK(false) << \"Instruction not defined (for this bitwidth)\";"
           "return 1;\n";
    ofs << "  }\n\n";
    ofs << "}\n\n";
  }
  ofs.close();
}

ConfigInst* InstModel::getInst(int opcode) {
  for (unsigned i = 0; i < instList_.size(); ++i) {
    if (instList_[i].opcode() == opcode) {
      return &instList_[i];
    }
  }
  return nullptr;
}


void InstModel::printCFiles(char* header_file, char* cpp_file) {
  // -------------------------print header file -----------------------------
  ofstream ofs(header_file, ios::out);
  ofs << "//This file generated from inst_model.cpp -- Do not edit.  Do not commit to "
         "repo.\n"
         "#ifndef __SS_INST_H__\n"
         "#define __SS_INST_H__\n"
         "\n"
         "#include <string>\n"
         "#include <string.h>\n"
         "#include <cstring>\n"
         "#include <assert.h>\n"
         "#include <iostream>\n"
         "#include <vector>\n"
         "#include <complex>\n"
         "#include <algorithm>\n"
         "#include <math.h>\n"
         "\n"
         "#include \"dsa/debug.h\"\n"

         "#define FIX_MAX ((1 << 15) - 1)\n"
         "#define FIX_MIN (-FIX_MAX)\n"
         "#define FIX_TRUNC(x) (x > FIX_MAX ? FIX_MAX : (x < FIX_MIN ? FIX_MIN : x))\n"
         "\n"
         "#define FRAC_BITS 11\n"
         "#define DELTA (((double)1.0) / (1 << FRAC_BITS))\n"
         "#define FLOAT_MAX (FIX_MAX * DELTA)\n"
         "#define FLOAT_MIN (FIX_MIN * DELTA)\n"
         "#define FLOAT_TRUNC(x) (x > FLOAT_MAX ? FLOAT_MAX : (x < FLOAT_MIN ? FLOAT_MIN "
         ": x))\n"
         "\n"
         "#define DOUBLE_TO_FIX(x) ((int)(FLOAT_TRUNC(x) / DELTA))\n"
         "#define FIX_TO_DOUBLE(x) (x * DELTA)\n"
         "\n"
         "#define FIX_ADD(a, b) (FIX_TRUNC((int)a + (int)b))\n"
         "#define FIX_MINUS(a, b) (FIX_ADD(a, -b))\n"
         "#define FIX_MUL(a, b) (FIX_TRUNC(((int)a * (int)b) * DELTA))\n"
         "#define FIX_TAN_H(x) (DOUBLE_TO_FIX(tanh(FIX_TO_DOUBLE(x))))\n"
         "\n"
         "using std::complex;\n"
         "\n"
         "namespace dsa {\n"
         "\n"

         "float    as_float(std::uint32_t ui);\n"
         "uint32_t as_uint32(float f);\n"
         "\n"
         "double    as_double(std::uint64_t ui);\n"
         "uint64_t as_uint64(double f);\n"
         "\n"

         "std::complex<float> as_float_complex(uint64_t ui);\n"
         "uint64_t as_uint64(const std::complex<float> &val);\n"
         "\n"

         "enum OpCode {\n"
         "SS_ERR,\n";

  for (unsigned i = 0; i < _instList.size(); ++i) {
    ofs << "  SS_" << _instList[i].name() << ", \n";
  };

  ofs << "  SS_NUM_TYPES\n};\n\n";

  // Print out the pre-defined Function unit
  ofs << "enum fu_type_t {\n";
  for (unsigned i = 0; i < _fuTypeList.size(); ++i) {
    ofs << "  " << _fuTypeList[i]->name() << ", \n";
  };
  ofs << "  NON_PREDEFINED_FU_TYPE\n};\n";

  ofs << "\n";
  ofs << "extern int num_ops[" << _instList.size() + 2 << "];\n";
  ofs << "extern int bitwidth[" << _instList.size() + 2 << "];\n";

  ofs << "\n"
         "// OpCode\n"
         "OpCode inst_from_string(const char* str);\n"
         "const char* name_of_inst(OpCode inst);\n"
         "double inst_area(OpCode inst);\n"
         "double inst_power(OpCode inst);\n"
         "double inst_logic_lut(OpCode inst);\n"
         "double inst_flip_flop(OpCode inst);\n"
         "int inst_lat(OpCode inst);\n"
         "int inst_thr(OpCode inst);\n"
         "int num_values(OpCode inst);\n"
         "// fu_type_t\n"
         "fu_type_t fu_type_from_string(const char* str);\n"
         "const char* name_of_fu_type(fu_type_t fu_type);\n"
         "double fu_type_area(fu_type_t fu_type);\n"
         "double fu_type_power(fu_type_t fu_type);\n";

  // Generate an execute function for all bitwidths
  int bitwidths[4] = {64, 32, 16, 8};
  string types[4] = {"uint64_t", "uint32_t", "uint16_t", "uint8_t"};
  string suffixes[4] = {"64", "32", "16", "8"};

  ofs << "// execute \n";
  for (int i = 0; i < 4; ++i) {
    string dtype = types[i];
    string suffix = suffixes[i];
    ofs << dtype << " execute" << suffix << "(OpCode inst, "
        << "std::vector<" << dtype << ">& ops, "
        << "std::vector<" << dtype << ">& outs, " << dtype << "* reg, "
        << "bool &discard, std::vector<bool>& back_array);\n";
  }

  ofs << "\n"
         "}\n\n"
         "#endif\n";

  ofs.close();

  // -------------------------print cpp file --------------------------------

  ofs.open(cpp_file, ios::out);

  // inst_from_string
  ofs << "//This file generated from inst_model.cpp -- Do not edit.  Do not commit to "
         "repo.\n"
         "#include \""
      << header_file
      << "\"\n\n"

         "float dsa::as_float(std::uint32_t ui) {\n"
         "  float f;\n"
         "  std::memcpy(&f, &ui, sizeof(float));\n"
         "  return f;\n"
         "}\n"
         "\n"

         "uint32_t dsa::as_uint32(float f) {\n"
         "  uint32_t ui;\n"
         "  std::memcpy(&ui, &f, sizeof(float));\n"
         "  return ui;\n"
         "}\n"
         "\n"

         "double dsa::as_double(std::uint64_t ui) {\n"
         "  double f;\n"
         "  std::memcpy(&f, &ui, sizeof(double));\n"
         "  return f;\n"
         "}\n"
         "\n"

         "uint64_t dsa::as_uint64(double f) {\n"
         "  uint64_t ui;\n"
         "  std::memcpy(&ui, &f, sizeof(double));\n"
         "  return ui;\n"
         "}\n"
         "\n"

         "std::complex<float> dsa::as_float_complex(uint64_t val) {\n"
         "  std::complex<float> res;\n"
         "  std::memcpy(&res, &val, sizeof(val));\n"
         "  return res;\n"
         "}\n"
         "\n"

         "uint64_t dsa::as_uint64(const std::complex<float> &val) {\n"
         "  uint64_t res;\n"
         "  std::memcpy(&res, &val, sizeof(val));\n"
         "  return res;\n"
         "}\n"
         "\n"

         "using namespace dsa;\n\n"
         "OpCode dsa::inst_from_string(const char* str) {\n";

  for (unsigned i = 0; i < _instList.size(); ++i) {
    ofs << "  else if(strcmp(str,\"" << _instList[i].name() << "\")==0) return SS_"
        << _instList[i].name() << ";\n";
  }
  /*
  ofs << "  else { fprintf(stderr,\"Config Library does not understand "
         "string\\\"%s\\\"\\n\",str); "
         "return SS_ERR;}\n";
         */
  ofs << "  else {\n"
      << " DSA_WARNING << str << \" not found!\";\n"
      << " return SS_ERR;}\n";
  ofs << "}\n\n";

  // Properties of Instructions

  // name_of_inst
  ofs << "const char* dsa::name_of_inst(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < _instList.size(); ++i) {
    ofs << "    case "
        << "SS_" << _instList[i].name() << ": return \"" << _instList[i].name()
        << "\";\n";
  }
  ofs << "  case SS_ERR:  DSA_CHECK(false) << \"Error opcode!\"; throw;\n";
  ofs << "  case SS_NUM_TYPES:  DSA_CHECK(false) << \"Opcode exceeds.\";\n";
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n";
  ofs << "}\n\n";

  // FUNCTION: inst_lat
  ofs << "int dsa::inst_lat(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < _instList.size(); ++i) {
    ofs << "    case "
        << "SS_" << _instList[i].name() << ": return " << _instList[i].latency()
        << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // FUNCTION: inst_thr
  ofs << "int dsa::inst_thr(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < _instList.size(); ++i) {
    ofs << "    case "
        << "SS_" << _instList[i].name() << ": return " << _instList[i].throughput()
        << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // area of the ssinst
  ofs << "double dsa::inst_area(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < _instList.size(); ++i) {
    ofs << "    case "
        << "SS_" << _instList[i].name() << ": return " << _instList[i].area() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // power of the ssinst
  ofs << "double dsa::inst_power(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < _instList.size(); ++i) {
    ofs << "    case "
        << "SS_" << _instList[i].name() << ": return " << _instList[i].power() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // LogicLut of the ssinst
  ofs << "double dsa::inst_logic_lut(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < _instList.size(); ++i) {
    ofs << "    case "
        << "SS_" << _instList[i].name() << ": return " << _instList[i].logic_lut() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";


  // flip-flop of the ssinst
  ofs << "double dsa::inst_flip_flop(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < _instList.size(); ++i) {
    ofs << "    case "
        << "SS_" << _instList[i].name() << ": return " << _instList[i].flip_flop() << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";
  

  // Pre-defined Function Unit Type
  ofs << "fu_type_t dsa::fu_type_from_string(const char* str) {\n"
         "  if(strcmp(str, \"NON_PREDEFINED_FU_TYPE\") == 0) return "
         "NON_PREDEFINED_FU_TYPE;\n";
  for (unsigned i = 0; i < _fuTypeList.size(); ++i) {
    ofs << "  else if(strcmp(str,\"" << _fuTypeList[i]->name() << "\")==0) return "
        << _fuTypeList[i]->name() << ";\n";
  }
  ofs << "  else { return NON_PREDEFINED_FU_TYPE; }\n";
  ofs << "}\n\n";

  // name of fu type
  ofs << "const char* dsa::name_of_fu_type(fu_type_t fu_type) {\n"
         "  switch(fu_type) {\n";
  for (unsigned i = 0; i < _fuTypeList.size(); ++i) {
    ofs << "    case " << _fuTypeList[i]->name() << ": return \""
        << _fuTypeList[i]->name() << "\";\n";
  }
  ofs << "    case NON_PREDEFINED_FU_TYPE: return \"NON_PREDEFINED_FU_TYPE\";\n";
  ofs << "    default: return \"NON_PREDEFINED_FU_TYPE\";\n";
  ofs << "  }\n";
  ofs << "}\n\n";

  // area of the fu_type
  ofs << "double dsa::fu_type_area(fu_type_t fu_type) {\n"
         "  switch(fu_type) {\n";
  for (unsigned i = 0; i < _fuTypeList.size(); ++i) {
    ofs << "    case " << _fuTypeList[i]->name() << ": return " << _fuTypeList[i]->area()
        << ";\n";
  }
  ofs << "    default:  return -1.0;\n";
  ofs << "  }\n";
  ofs << "}\n\n";

  // power of the fu_type
  ofs << "double dsa::fu_type_power(fu_type_t fu_type) {\n"
         "  switch(fu_type) {\n";
  for (unsigned i = 0; i < _fuTypeList.size(); ++i) {
    ofs << "    case " << _fuTypeList[i]->name() << ": return " << _fuTypeList[i]->power()
        << ";\n";
  }
  ofs << "    default:  return -1.0;\n";
  ofs << "  }\n";
  ofs << "}\n\n";

  // FUNCTION: num_values
  ofs << "int dsa::num_values(OpCode inst) {\n"
         "  switch(inst) {\n";
  for (unsigned i = 0; i < _instList.size(); ++i) {
    ofs << "    case "
        << "SS_" << _instList[i].name() << ": return " << _instList[i].num_values()
        << ";\n";
  }
  ofs << "    default: DSA_CHECK(false) << \"Unknown inst\" << inst; throw;\n";
  ofs << "  }\n\n";
  ofs << "}\n\n";

  // num_ops_array
  ofs << "int dsa::num_ops[" << _instList.size() + 2 << "]={\n";
  ofs << "\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
  for (unsigned i = 0; i < _instList.size(); ++i) {
    ofs << ", " << _instList[i].num_ops();
    if (i % 16 == 15) {
      ofs << "\n";
      ofs << "\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
    }
  }
  ofs << "};\n\n";

  // bitwidth_array
  ofs << "int dsa::bitwidth[" << _instList.size() + 2 << "]={\n";
  ofs << "\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
  for (unsigned i = 0; i < _instList.size(); ++i) {
    ofs << ", " << _instList[i].bitwidth();
    if (i % 16 == 15) {
      ofs << "\n";
      ofs << "\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
    }
  }
  ofs << "};\n\n";

  // FUNCTION: execute()
  for (int i = 0; i < 4; ++i) {
    int bitwidth = bitwidths[i];
    string dtype = types[i];
    string suffix = suffixes[i];

    ofs << dtype << " dsa::execute" << suffix << "(OpCode inst, "
        << "std::vector<" << dtype << ">& ops, "
        << "std::vector<" << dtype << ">& outs, " << dtype << " *reg, "
        << "bool &discard, std::vector<bool>& back_array) {\n";

    // somwhere below is an implementation of pass through, is it though? (tony, 2018)

    ofs << "  " << dtype
        << "& accum = reg[0];\n"
           "  (void) accum;\n"
           "  DSA_CHECK(ops.size() <= 4);\n"
           "  DSA_CHECK(ops.size() <=  (unsigned)(num_ops[inst]+1));\n"
           "  switch(inst) {\n"
        << "    case SS_NONE: { return ops[0]; }\n";
    for (unsigned i = 0; i < _instList.size(); ++i) {
      ConfigInst* inst = &_instList[i];

      if (inst->bitwidth() != bitwidth)
        continue;  // TODO: later implement autovectorization

      ofs << "    case "
          << "SS_" << inst->name() << ": {";

      string inst_code_name = base_folder + "insts" + suffix + "/" + inst->name() + ".h";
      ifstream f(inst_code_name.c_str());

      if (f.good()) {
        std::string line;
        ofs << "\n";
        while (std::getline(f, line)) {
          ofs << "      " << line << "\n";
        }
        ofs << "    };\n";
      } else {
        ofs << "DSA_CHECK(false) << name_of_inst(inst) << \" not implemented, add it to insts folder\";";
        ofs << "};\n";
      }
    }
    ofs << "    default: DSA_CHECK(false) << \"Instruction not defined (for this bitwidth)\";"
           "return 1;\n";
    ofs << "  }\n\n";
    ofs << "}\n\n";
  }
  ofs.close();
}

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "Usage:\n inst_model [input file] [power&area file] [header file] [cpp "
                 "file]\n";
    return 1;
  }

  InstModel instModel(argv[1]);
  DSA_INFO << "InstModel created";
  DSA_INFO << argv[2];
  instModel.readModel(argv[2]);
  //instModel->PowerAreaModel(argv[2]);
  DSA_INFO << "PowerAreaModel created";
  DSA_INFO << "Printing C Files";
  instModel.newPrintCFiles(argv[3], argv[4]);
  return 0;
}
