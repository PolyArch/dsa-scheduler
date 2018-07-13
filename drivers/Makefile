include $(SS_STACK)/msg.mk

level=../
include ../make.config

SYS = $(shell sys)
CXX = g++

CXXFLAGS := $(OPT) -Wall -g -std=c++11

LIB_PATH=$(SS_TOOLS)/lib 
INC_SBMODEL_PATH=$(SS_TOOLS)/include/ss-config
INC_SBSCHED_PATH=$(SS_TOOLS)/include/ss-scheduler

CXXFLAGS += -I$(INC_SBMODEL_PATH) -I$(INC_SBSCHED_PATH) -Wl,-rpath,$(LIB_PATH) 

LIBS +=-ly -lfl -lsbscheduler -lsbconfig

all: sb_sched sb_dfg_emu

sb_sched : sb_sched.cpp $(INC_SBMODEL_PATH)/*.h
	$(CXX) $(CXXFLAGS) -o $@ $< -L$(LIB_PATH) $(LIBS)  -Wl,-rpath,${SS_TOOLS}/lib

sb_dfg_emu : sb_dfg_emu.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -L$(LIB_PATH) $(LIBS)  -Wl,-rpath,${SS_TOOLS}/lib


clean:
	rm -rf *.o  sb_dfg_emu sb_sched *.d

include ../make.rules

