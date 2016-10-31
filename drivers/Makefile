level=../
sb_config=../../
include ../make.config

SYS = $(shell sys)
CXX = g++

CXXFLAGS :=  -Wall -g -std=c++11

LIB_SBMODEL_PATH=$(sb_config)softbrain-config/build/lib
LIB_SBSCHED_PATH=$(level)build/lib
INC_SBMODEL_PATH=$(sb_config)softbrain-config/src
INC_SBSCHED_PATH=$(level)src

CXXFLAGS += -I$(INC_SBMODEL_PATH) -I$(INC_SBSCHED_PATH) -Wl,-rpath,$(LIB_SBSCHED_PATH)

#all: reschedule stat-config

all: sb_sched sb_dfg_emu reschedule stat-config


reschedule : reschedule.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_SBMODEL_PATH) -L$(LIB_SBSCHED_PATH) -lsbconfig -lsbscheduler -Wl,-rpath,${level}/lib

stat-config : stat-config.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_SBMODEL_PATH) -L$(LIB_SBSCHED_PATH) -lsbconfig -lsbscheduler -Wl,-rpath,${level}/lib

sb_sched : sb_sched.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_SBMODEL_PATH) -L$(LIB_SBSCHED_PATH) -lsbconfig -lsbscheduler -Wl,-rpath,${level}/lib

sb_dfg_emu : sb_dfg_emu.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_SBMODEL_PATH) -L$(LIB_SBSCHED_PATH) -lsbconfig -lsbscheduler -Wl,-rpath,${level}/lib


clean:
	rm -rf *.o  sb_sched stat-config reschedule *.d

include ../make.rules

