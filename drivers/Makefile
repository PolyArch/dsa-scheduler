SYS = $(shell sys)
ifeq ($(SYS),amd64_rhel6)
    CXX = /s/gcc-4.6.2/@sys/bin/g++
else
    CXX = /s/gcc-4.6.1/@sys/bin/g++
endif


CXXFLAGS :=  -Wall -g -std=c++0x -lboost_regex

SRC_ROOT=../..
LIB_DYMODEL_PATH=$(SRC_ROOT)/softbrain-config/lib
INC_DYMODEL_PATH=$(SRC_ROOT)/softbrain-config/src
LIB_DYSCHED_PATH=$(SRC_ROOT)/softbrain-scheduler/lib
INC_DYSCHED_PATH=$(SRC_ROOT)/softbrain-scheduler/src

CXXFLAGS += -I$(INC_DYMODEL_PATH) -I$(INC_DYSCHED_PATH) -Wl,-rpath,$(shell pwd)/../lib -Wl,-rpath,$(shell pwd)/../../sb_config/lib

#all: reschedule stat-config

all: sb_sched reschedule stat-config


reschedule : reschedule.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -L$(LIB_DYMODEL_PATH) -L$(LIB_DYSCHED_PATH) -lsbconfig -lsbscheduler

stat-config : stat-config.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -L$(LIB_DYMODEL_PATH) -L$(LIB_DYSCHED_PATH) -lsbconfig -lsbscheduler

sb_sched : sb_sched.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -L$(LIB_DYMODEL_PATH) -L$(LIB_DYSCHED_PATH) -lsbconfig -lsbscheduler


clean:
	rm -rf *.o  sb_sched stat-config reschedule
