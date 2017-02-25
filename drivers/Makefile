level=../
include ../make.config

SYS = $(shell sys)
CXX = /s/gcc-5.1.0/@sys/bin/g++

CXXFLAGS :=  -Wall -g -std=c++11

LIB_PATH=$(prefix)/lib
INC_SBMODEL_PATH=$(prefix)/include/softbrain-config
INC_SBSCHED_PATH=$(prefix)/include/softbrain-scheduler

CXXFLAGS += -I$(INC_SBMODEL_PATH) -I$(INC_SBSCHED_PATH) -Wl,-rpath,$(LIB_PATH) 

#all: reschedule stat-config

all: sb_sched reschedule stat-config sb_dfg_emu


reschedule : reschedule.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_PATH) -lsbconfig -lsbscheduler

stat-config : stat-config.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_PATH) -lsbconfig -lsbscheduler

sb_sched : sb_sched.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_PATH) -lsbconfig -lsbscheduler -Wl,-rpath,${prefix}/lib

sb_dfg_emu : sb_dfg_emu.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_PATH) -lsbscheduler -lsbconfig -Wl,-rpath,${prefix}/lib


clean:
	rm -rf *.o  sb_dfg_emu sb_sched stat-config reschedule *.d

include ../make.rules

