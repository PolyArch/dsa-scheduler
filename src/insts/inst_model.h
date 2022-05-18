#ifndef __INST_MODEL_H__
#define __INST_MODEL_H__

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace dsa {

#define DEF_ATTR(attr)                         \
  decltype(attr##_) attr() { return attr##_; } \
  void attr(const decltype(attr##_) & new_value) { attr##_ = new_value; }

// Instruction Class
// Stores attributes like it's name, latency, etc...
class ConfigInst { 
 private:
  std::string name_;
  std::string dtype_;
  int opcode_;
  int latency_;
  int throughput_;  // technically 1/throughput in cycles
  int num_values_;
  int num_ops_;
  int bitwidth_;
  double area_ = -1.0;
  double power_ = -1.0;
  double total_lut_ = 5.0;
  double logic_lut_ = 5.0;
  double ram_lut_ = -1.0;
  double srl_ = -1.0;
  double flip_flop_ = 5.0;
  double ram_b36_ = -1.0;
  double ram_b18_ = -1.0;
  double u_ram_ = -1.0;
  double dsp_ = -1.0;
 public:
  DEF_ATTR(name);
  DEF_ATTR(dtype);
  DEF_ATTR(opcode);
  DEF_ATTR(latency);
  DEF_ATTR(throughput);
  DEF_ATTR(num_values);
  DEF_ATTR(num_ops);
  DEF_ATTR(bitwidth);
  DEF_ATTR(area);
  DEF_ATTR(power);
  DEF_ATTR(total_lut);
  DEF_ATTR(logic_lut);
  DEF_ATTR(ram_lut);
  DEF_ATTR(srl);
  DEF_ATTR(flip_flop);
  DEF_ATTR(ram_b36);
  DEF_ATTR(ram_b18);
  DEF_ATTR(u_ram);
  DEF_ATTR(dsp);
};

class FuType {
 private:
  std::string name_;
  double area_ = -1.0;
  double power_ = -1.0;
  double total_lut_ = 5.0;
  double logic_lut_ = 5.0;
  double ram_lut_ = -1.0;
  double flip_flop_ = 5.0;
  double dsp_ = -1.0;
 public:
  
  DEF_ATTR(name);
  DEF_ATTR(area);
  DEF_ATTR(power);
  DEF_ATTR(total_lut);
  DEF_ATTR(logic_lut);
  DEF_ATTR(ram_lut);
  DEF_ATTR(flip_flop);
  DEF_ATTR(dsp);
};

class InstModel {
 public:
  InstModel(char* filename);  // read the file and populate the instructions

  void readModel(char* filename);
  void PowerAreaModel(char* filename);

  void newPrintCFiles(char* header_file, char* cpp_file);
  void printCFiles(char* header, char* cpp);

  ConfigInst* getInst(int opcode);

 private:
  std::vector<ConfigInst> instList_;
  std::vector<ConfigInst> _instList;
  std::vector<FuType*> _fuTypeList;
  std::string filename_;
  std::string base_folder;
};

}  // namespace dsa

#endif
