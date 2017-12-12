include $(SS_STACK)/msg.mk

level=../
include ../make.config

SYS = $(shell sys)
CXX = g++

CXXFLAGS :=  -Wall -g -std=c++11 

LIB_PATH=$(SS_TOOLS)/lib
INC_SBMODEL_PATH=$(SS_TOOLS)/include/ss-config
INC_SBSCHED_PATH=$(SS_TOOLS)/include/ss-scheduler

CXXFLAGS += -I$(INC_SBMODEL_PATH) -I$(INC_SBSCHED_PATH) -Wl,-rpath,$(LIB_PATH) 

#all: reschedule stat-config

all: sb_sched sb_dfg_emu
#reschedul


reschedule : reschedule.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_PATH) -lsbscheduler -lsbconfig

stat-config : stat-config.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_PATH) -lsbscheduler -lsbconfig

sb_sched : sb_sched.cpp $(INC_SBMODEL_PATH)/*.h
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_PATH) -lsbscheduler -lsbconfig -Wl,-rpath,${SS_TOOLS}/lib

sb_dfg_emu : sb_dfg_emu.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_PATH) -lsbscheduler -lsbconfig -Wl,-rpath,${SS_TOOLS}/lib


clean:
	rm -rf *.o  sb_dfg_emu sb_sched stat-config reschedule *.d

include ../make.rules

