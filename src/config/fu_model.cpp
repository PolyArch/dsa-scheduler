#include <sstream>

#include "assert.h"
#include "fu_model.h"
#include "model_parsing.h"
#include "ssinst.h"

using namespace SS_CONFIG;
using namespace std;

// FU_type(func_unit_def) capabilities
// FU_ADD: Add16x4:1

FuModel::FuModel(std::istream& istream) {
  // char line[512];
  string param, value;

  while (istream.good()) {
    if (istream.peek() == '[') break;  // break out if done

    // string line;
    ModelParsing::ReadPair(istream, param, value);

    if (param[0] == '#' || value[0] == '#') continue;  // Not a comment

    if (ModelParsing::StartsWith(param, "FU_TYPE")) {
      // defining an fu and capabilitty

      string newtype;

      std::stringstream ss(param);

      getline(ss, param, ' ');
      getline(ss, newtype);

      func_defs.push_back(func_unit_def(newtype));
      AddCapabilities(func_defs[func_defs.size() - 1], value);

    } else if (ModelParsing::StartsWith(param, "SWITCH_TYPE")) {
      // AddCapabilities(*GetFU("SWITCH"), value);
      assert(0);
    }
  }
}

func_unit_def* FuModel::GetFUDef(char* fu_cstr) {
  string s(fu_cstr);
  return GetFUDef(s);
}

// Get a functional unit based upon the description string (the name)
func_unit_def* FuModel::GetFUDef(string& fu_string) {
  for (unsigned i = 0; i < func_defs.size(); ++i) {
    if (func_defs[i].name().compare(fu_string) == 0) {
      return &func_defs[i];
    }
  }
  return nullptr;  // if no fu, return null
}

// Find all supported instructions by current instruction
std::set<ss_inst_t> func_unit_def::find_all_insts_covered(ss_inst_t source_inst){
  // initialize the covered instructions
  std::set<ss_inst_t> covered_inst;
  // parse the current instruction
  std::string inst_name = SS_CONFIG::name_of_inst(source_inst);
  //std::cout << "parsing " << inst_name << "\n";
  // the properties that required to be parsed
  bool is_fixed_point = true; // assuming it is fixed point instruction
  int bitwidth = 64; // assuming it is 64-bit
  int decomposer = 1; // assuming the decomposer = 1
  std::string func_name; // required function name

  // if start with 'F', and second char is in cap, then it is floating point
  if(inst_name[0]=='F' && inst_name[1]<='Z' && inst_name[1]>='A'){
    is_fixed_point = false;
  }

  // if the last char is not number, then it is a 64-bit instruction
  // Extract the bitwidth and decomposer
  char last_char = inst_name.back();
  if(!(last_char >='0' && last_char <= '9')){
    bitwidth = 64;
    decomposer = 1;
  }else{
    // when the last char is number
    // if ends with '8' could be 
    // 1. Sss8 (bitwidth = 8, decomposer = 1)
    // or 2. SssBBx8 (bitwidth BB, decomposer = 8)
    char last_2_char = inst_name[inst_name.length() - 2];
    char last_3_char = inst_name[inst_name.length() - 3];
    if(last_char == '8'){
      if((last_3_char >= '0' && last_3_char <= '9') && last_2_char == 'x'){
        // circumstance 2
        decomposer = 8;
        switch(last_3_char){
          case '8': bitwidth = 8;break;
          case '6': bitwidth = 16;break;
          case '2': bitwidth = 32;break;
          case '4': bitwidth = 64;break;
          default: std::cout<<"cannot parse "<<inst_name<<"\n";
          assert(0 && "why?what else?");break;
        }
      }else{
        // circumstance 1
        bitwidth = 8;decomposer = 1;
      }
    }else{
      // the last char is not '8', then it could be :
      // 1. SssDD (bitwidth = DD, decomposer = 1)
      // or 2. SssBBxD (bitwidth = BB, decomposer = D)
      if((last_3_char >= '0'&&last_3_char <= '9')&&(last_2_char == 'x')){
        // circumstance 2
        switch(last_char){
          case '1': decomposer = 1;break;
          case '2': decomposer = 2;break;
          case '4': decomposer = 4;break;
          default: std::cout<<"cannot parse "<<inst_name<<"\n";
          assert(0 && "why?what else?");break;
        }
        switch(last_3_char){
          case '8': bitwidth = 8;break;
          case '6': bitwidth = 16;break;
          case '2': bitwidth = 32;break;
          case '4': bitwidth = 64;break;
          default: std::cout<<"cannot parse "<<inst_name<<"\n";
          assert(0 && "why?what else?");break;
        }
      }else{
        // circumstance 1
        decomposer = 1;
        switch(last_char){
          case '6': bitwidth = 16;break;
          case '2': bitwidth = 32;break;
          case '4': bitwidth = 64;break;
          default: std::cout<<"cannot parse "<<inst_name<<"\n";
          assert(0 && "why?what else?");break;
        }
      }
    }
  }

  // Extract the function name of the instruction
  // find the location of first number character in string


  if (inst_name.find('_') != std::string::npos){
    func_name = inst_name.substr(0, inst_name.find('_'));
  }else{
    int num_loc = 0;
    while((num_loc < inst_name.length())&&
      (inst_name[num_loc] < '0' || inst_name[num_loc] > '9'))
      {num_loc++;}
    func_name = inst_name.substr(0,num_loc);
  }

  // if it is a fixed-point instruction, it is possible that it starts with "S", "I", "U", "Fx"
  std::string single_prefix_str = "LIUSH"; // wow, it is my name (by Sihao) :)
  if(is_fixed_point){
    if(ModelParsing::StartsWith(func_name, "Fx")){
      func_name = func_name.substr(2, func_name.length()-2);
    }else{
      char first_char = func_name[0];
      char second_char = func_name[1];
      // If first character is in prefix, and second character is in cap
      if( (single_prefix_str.find(first_char) != std::string::npos)
      &&(second_char >= 'A' && second_char <='Z')){
        func_name = func_name.substr(1, func_name.length()-1);
      }
    }
  }

  // Print out the current instruction properties
  //std::cout << (is_fixed_point ? "fixed-point " : "floating-point ")
  //          << bitwidth <<"-bit " << func_name << " x " << decomposer << "\n";
  
  // Generate the covering instruction
  int bitwidth_range[4] = {8,16,32,64};
  int decomposer_range[4] = {1,2,4,8};
  std::string fixed_prefixs[5] = {"", "S", "U", "I", "Fx"};
  std::set<std::string> cover_possible_insts;
  if(_inst_cover_group_map.count(func_name) > 0){ // this inst can cover more that one kind inst
    std::set<std::string> covering_func_names = _inst_cover_group_map[func_name];
    if(is_fixed_point){ // is fixed point
      // loop all covering function name
      for(const std::string cand_func_name : covering_func_names){
        // loop possible prefix
        for(const std::string fixed_prefix : fixed_prefixs){
          // attempting all bitwidth
          for(const int attem_bitwidth : bitwidth_range){
            // attempting all decomposer
            for(const int attem_decomposer : decomposer_range){
              // judge whether it is supported
              if(attem_bitwidth <= bitwidth && (attem_bitwidth * attem_decomposer) <= (bitwidth * decomposer)){
                // supported
                if(attem_decomposer == 1){
                  // non-decomposed
                  if(attem_bitwidth == 64){
                    cover_possible_insts.insert(fixed_prefix + cand_func_name);
                    cover_possible_insts.insert(fixed_prefix + cand_func_name + "64");
                  }else{
                    cover_possible_insts.insert(fixed_prefix + cand_func_name + std::to_string(attem_bitwidth));
                  }
                }else{
                  // decomposable
                  cover_possible_insts.insert(fixed_prefix + cand_func_name 
                  + std::to_string(attem_bitwidth) + "x" + std::to_string(attem_decomposer));
                }
              }else{
                // not supported
                continue;
              }
            }// end of decomposer
          }// end of bitwidth
        }// end of prefix
      }// end of candidate function unit name
    }else{ // floating point
      for(const std::string cand_func_name : covering_func_names){
        cover_possible_insts.insert(cand_func_name + std::to_string(bitwidth));
        if(decomposer == 2){
          cover_possible_insts.insert(cand_func_name + std::to_string(bitwidth) + "x2");
        }
      }
    }
  }else{// this inst can only support the same kind of inst
    if(is_fixed_point){ // is fixed point
      // loop possible prefix
      for(const std::string fixed_prefix : fixed_prefixs){
        // attempting all bitwidth
        for(const int attem_bitwidth : bitwidth_range){
          // attempting all decomposer
          for(const int attem_decomposer : decomposer_range){
            // judge whether it is supported
            if(attem_bitwidth <= bitwidth && (attem_bitwidth * attem_decomposer) <= (bitwidth * decomposer)){
              // supported
              if(attem_decomposer == 1){
                // non-decomposed
                if(attem_bitwidth == 64){
                  cover_possible_insts.insert(fixed_prefix + func_name);
                  cover_possible_insts.insert(fixed_prefix + func_name + "64");
                }else{
                  cover_possible_insts.insert(fixed_prefix + func_name + std::to_string(attem_bitwidth));
                }
              }else{
                // decomposable
                cover_possible_insts.insert(fixed_prefix + func_name 
                + std::to_string(attem_bitwidth) + "x" + std::to_string(attem_decomposer));
              }
            }else{
              // not supported
              continue;
            }
          }// end of decomposer
        }// end of bitwidth
      }// end of prefix
    }else{ // is floating point
      cover_possible_insts.insert(func_name + std::to_string(bitwidth));
      if(decomposer == 2){
        cover_possible_insts.insert(func_name + std::to_string(bitwidth) + "x2");
      }
    }
  }

  // convert inst string to ss_inst
  for(std::string cand_inst_name : cover_possible_insts){
    ss_inst_t curr_ssinst_t = SS_CONFIG::inst_from_string(cand_inst_name.c_str());
    if(curr_ssinst_t != SS_CONFIG::SS_ERR){
      covered_inst.insert(curr_ssinst_t);
    }
  }

  // return the result
  return covered_inst;
}

// Determine the instruction accountability
void func_unit_def::determine_inst_accountability(){
  // reset the accountability map (assume all instructions are accountable for hardware)
  for(auto inst_acc : _account_for_hw){
    inst_acc.second = true;
  }

  // loop for every instruction
  for(auto & inst_acc : _account_for_hw){
    ss_inst_t inst = inst_acc.first;
    bool account = inst_acc.second;
    // if this instruction is valid for hw
    if(account){
      // find all instructions that covered by this instruction
      std::set<ss_inst_t> all_insts_covered_by_this = 
        find_all_insts_covered(inst);
      // check all instructions
      for (auto & other_inst_acc : _account_for_hw){
        if(!other_inst_acc.second){
          continue; // either has already been supported by other instruction
        }else{
          // skip itself
          if(other_inst_acc.first == inst){continue;}
          // check whether cover other insts
          if(all_insts_covered_by_this.count(other_inst_acc.first)>0){
            //std::cout << SS_CONFIG::name_of_inst(other_inst_acc.first) 
            //          << " covered by " << SS_CONFIG::name_of_inst(inst) << "\n";
            // or covered by this instruction
            other_inst_acc.second = false;
          }else{
            // otherwise this instruction is accountable for hw (power and area)
            other_inst_acc.second = true;
          }
        }
      }
    }else{
      continue;
    }
  }

  // print out accountabiliy
  //for(const auto & inst_acc : _account_for_hw){
  //  std::cout << SS_CONFIG::name_of_inst(inst_acc.first) << ": " << (inst_acc.second ? "used ":"not ") << " | ";
  //}
  //std::cout << "\n";

  _accountability_dirty_bit = false;
}

double func_unit_def::get_area_from_inst_set(){
  // check if the accountability is not determined
  if(_accountability_dirty_bit){
    determine_inst_accountability();
  }

  // add the area of instruction that accountable
  double area = 0.0;
  for(ss_inst_t inst : cap()){
    if(!is_accountable_for_fw(inst)){
      continue;
    }
    double curr_area = SS_CONFIG::inst_area(inst);
    if(curr_area < 0.0){
      //std::cout << SS_CONFIG::name_of_inst(inst) << " is not support for area estimation"
      //          << std::endl;
      // to turn off this assertion you can assign a default area to it and change false to true
      // assert(false && "this instruction is not support for area estimation");
    }else{
      area += curr_area;
    }
  }
  return area;
}

double func_unit_def::get_power_from_inst_set(){
  // check if the accountability is not determined
  if(_accountability_dirty_bit){
    determine_inst_accountability();
  }

  // add the power of instruction that accountable
  double power = 0.0;
  for(ss_inst_t inst : cap()){
    if(!is_accountable_for_fw(inst)){
      continue;
    }
    double curr_power = SS_CONFIG::inst_power(inst);
    if(curr_power < 0.0){
      //std::cout << SS_CONFIG::name_of_inst(inst) << " is not support for power estimation"
      //          << std::endl;
      // to turn off this assertion you can assign a default power to it and change false to true
      power += 1.0; // accum 1mW if area is not found for this inst
      // assert(false && "this instruction is not support for power estimation");
    }else{
      power += curr_power;
    }
  }
  return power;
}

// Power
double func_unit_def::power(){
  // if no new cap is added, just return the old power
  if(!_power_dirty_bit) {
    //std::cout << "use the old power, no need to recalculate\n";
    return _power;
  }
  
  // recalculate the power
  double _power = 0.0;
  std::string fu_name = name();

  if (fu_name.find('-') != std::string::npos){
  // when finding '-', means that this function definition can be a combined fu
    std::vector<std::string> primitive_fu_names;
    ModelParsing::split(fu_name,'-',primitive_fu_names);
    std::set<SS_CONFIG::fu_type_t> primitive_fu_types;
    bool all_predefined = true; // assume all fu types are pre-defined
    // check if all fu types are pre-defined
    for(auto primitive_fu_name : primitive_fu_names){
      fu_type_t primitive_fu_type = SS_CONFIG::fu_type_from_string(primitive_fu_name.c_str());
      if(primitive_fu_type == SS_CONFIG::fu_type_t::NON_PREDEFINED_FU_TYPE){
        all_predefined = false; break;
      }else{
        primitive_fu_types.insert(primitive_fu_type);
      }
    }
    // I think it would be rare that all fu types are pre-defined
    if(all_predefined){
      // all fu types are pre-defined, then look up it power (quick)
      for(fu_type_t predefined_fu_type : primitive_fu_types){
        _power += _power + SS_CONFIG::fu_type_power(predefined_fu_type);
      }
    }else{
      _power = get_power_from_inst_set();
    }
  }else{
    _power = get_power_from_inst_set();
  }

  // This FU type is not a combined fu, then either a pre-defined (look up power)
  // or get power from instruction set
  fu_type_t fu_type = fu_type_from_string(name().c_str());
  if(fu_type == SS_CONFIG::fu_type_t::NON_PREDEFINED_FU_TYPE){
    _power = get_power_from_inst_set();
  }else{
    _power = SS_CONFIG::fu_type_power(fu_type);
  }

  _power_dirty_bit = false;
  return _power;
}

// Area Model
double func_unit_def::area(){
  // if no new cap is added, just return the old area
  if(!_area_dirty_bit) {
    //std::cout << "use the old area, no need to recalculate\n";
    return _area;
  }
  
  // recalculate the area
  double _area = 0.0;
  std::string fu_name = name();

  if (fu_name.find('-') != std::string::npos){
  // when finding '-', means that this function definition can be a combined fu
    std::vector<std::string> primitive_fu_names;
    ModelParsing::split(fu_name,'-',primitive_fu_names);
    std::set<SS_CONFIG::fu_type_t> primitive_fu_types;
    bool all_predefined = true; // assume all fu types are pre-defined
    // check if all fu types are pre-defined
    for(auto primitive_fu_name : primitive_fu_names){
      fu_type_t primitive_fu_type = SS_CONFIG::fu_type_from_string(primitive_fu_name.c_str());
      if(primitive_fu_type == SS_CONFIG::fu_type_t::NON_PREDEFINED_FU_TYPE){
        all_predefined = false; break;
      }else{
        primitive_fu_types.insert(primitive_fu_type);
      }
    }
    // I think it would be rare that all fu types are pre-defined
    if(all_predefined){
      // all fu types are pre-defined, then look up it area (quick)
      for(fu_type_t predefined_fu_type : primitive_fu_types){
        _area += _area + SS_CONFIG::fu_type_area(predefined_fu_type);
      }
    }else{
      _area = get_area_from_inst_set();
    }
  }else{
    _area = get_area_from_inst_set();
  }

  // This FU type is not a combined fu, then either a pre-defined (look up area)
  // or get area from instruction set
  fu_type_t fu_type = fu_type_from_string(name().c_str());
  if(fu_type == SS_CONFIG::fu_type_t::NON_PREDEFINED_FU_TYPE){
    _area = get_area_from_inst_set();
  }else{
    _area = SS_CONFIG::fu_type_area(fu_type);
  }

  _area_dirty_bit = false;
  return _area;
}

// This function reads line from an ifstream, and gets a param and value,
// seperated by a ":"
void FuModel::AddCapabilities(func_unit_def& fu, string& cap_string) {
  stringstream ss(cap_string);
  string cur_cap;

  while (getline(ss, cur_cap, ',')) {
    stringstream pss(cur_cap);
    string cap;
    string enc_str;

    getline(pss, cap, ':');

    ModelParsing::trim(cap);

    if (cap.empty()) {
      return;
    }

    if (ModelParsing::stricmp(cap, "ALL")) {
      for (int i = 0; i < SS_NUM_TYPES; ++i) {
        fu.add_cap((ss_inst_t)i);
      }
      return;
    }

    ss_inst_t ss_inst = inst_from_string(cap.c_str());

    if (ss_inst == SS_NONE || ss_inst == SS_ERR) {
      continue;
    }

    fu.add_cap(ss_inst);

    if (pss.good()) {  // then there must be an encoding string
      unsigned encoding;
      pss >> encoding;

      fu.set_encoding(ss_inst, encoding);
    }
  }
}
