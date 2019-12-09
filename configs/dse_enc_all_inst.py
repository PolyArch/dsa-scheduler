import re
import random

# set sbmodel file that contains all ssinst
ssinst_file = 'full.ssinst.sbmodel'

# Set the amount of combined function unit
max_combined_fu = 1000

# Set Min / Max number of Instruction of new fucntion unit
max_random_fu = 1000
min_num_inst = 2
max_num_inst = 10

# set the start encoding of instruction
start_enc = 2 # 0 is for NONE, 1 is for COPY

# Open the file that encoding will be written to
encoded_ssinst_sbmodel = 'enc.' + ssinst_file
enc_ssinst_file = open(encoded_ssinst_sbmodel,'w')

def write_inst_list_to_file(file, inst_set):
    inst_list = list(inst_set)
    for inst in inst_list:
        file.write(inst)
        if inst is inst_list[-1]:
            file.write('\n') # change line at the last inst
        else:
            file.write(', ') # seperate inst by comma


def enc_all_inst(set_inst):
    list_inst = list(set_inst)
    for i in range(0,len(list_inst)):
        list_inst[i] = list_inst[i] + ':' + str(start_enc + i)
    return list_inst

# FU_TYPE dictionary and instruction list
dict_fu = {}
inst_set = set()

# Mix two function unit type
def mix_fu_type(fu_names):
    fu_name_0 = fu_names[0]
    fu_name_1 = fu_names[1]
    fu0_insts = dict_fu[fu_name_0]
    fu1_insts = dict_fu[fu_name_1]
    num_mixed_inst = len(fu0_insts.union(fu1_insts))
    if not (num_mixed_inst > 64 or fu0_insts.issubset(fu1_insts) or fu1_insts.issubset(fu0_insts)):
        return fu_name_0+'-'+fu_name_1, fu0_insts.union(fu1_insts), True
    else:
        return '', set(), False


# Read the original ssinst sbmodel
with open(ssinst_file,'r') as ssinst_file:
    lines = ssinst_file.readlines()
    for line in lines:
        if 'FU_TYPE' in line:
            # Split the FU definition
            split = re.split(' |, |: |\n', line)
            # write FU_TYPE
            enc_ssinst_file.write('FU_TYPE ')
            # write FU name
            fu_name = split[1]
            enc_ssinst_file.write(fu_name+ ': ')
            # encode all instrutions
            curr_inst_list = split[2:-1]
            # add instructions to instruction set
            inst_set.update(curr_inst_list)
            # add fu_name -> curr_inst_list
            dict_fu[fu_name] = set(curr_inst_list)
            enc_list_inst = enc_all_inst(curr_inst_list)
            # write all encoded instruction
            write_inst_list_to_file(enc_ssinst_file,enc_list_inst)
        else:
            enc_ssinst_file.write(line)
            pass

num_inst = len(inst_set)
num_fu_type = len(dict_fu)
print('there are total ' + str(num_fu_type) + ' fu types, ' + str(num_inst) + ' instructions in original model')
enc_ssinst_file.close()

# Generate sbmodel file for DSE
dse_ssinst_sbmodel_file = 'dse.' + encoded_ssinst_sbmodel

with open(dse_ssinst_sbmodel_file,'w') as dse_sbmodel:
    # Copy Encoded Sbmodel to DSE sbmodel
    with open(encoded_ssinst_sbmodel, 'r') as enc_sbmodel:
        for line in enc_sbmodel.readlines():
            dse_sbmodel.write(line)
    dse_sbmodel.write('\n # ------------ Mixed Function Unit Types for DSE ------------\n')

    # Combine Existing Function Unit Type
    num_mixed_fu = 0
    while num_mixed_fu < max_combined_fu:
        fu_names_to_be_mix = random.sample(dict_fu.keys(), 2)
        new_fu_name, new_insts, mixed = mix_fu_type(fu_names_to_be_mix)
        if mixed:
            dict_fu[new_fu_name] = new_insts # Add the combined FU into dictionary
            dse_sbmodel.write('FU_TYPE ' + new_fu_name + ': ') # write fu name to file
            write_inst_list_to_file(dse_sbmodel, enc_all_inst(new_insts)) # write insts to file
            num_mixed_fu += 1

    # Create New Function Unit Type by combining two different instruction
    dse_sbmodel.write('\n # ------------ Random Function Unit Types for DSE ------------\n')
    num_random_fu = 0
    while num_random_fu < max_random_fu:
        num_inst = random.randint(min_num_inst, max_num_inst)
        random_inst = random.sample(inst_set, num_inst)
        if not (random_inst in dict_fu.values()):
            random_fu_name = 'RANDOM_FU_' + str(num_random_fu)
            dict_fu[random_fu_name] = random_inst
            dse_sbmodel.write('FU_TYPE ' + random_fu_name + ': ')
            write_inst_list_to_file(dse_sbmodel, enc_all_inst(random_inst))
            num_random_fu += 1

num_fu_type = len(dict_fu)
print('there are total ' + str(num_fu_type) + ' fu types in dse model')
