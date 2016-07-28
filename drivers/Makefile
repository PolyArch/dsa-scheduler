SYS = $(shell sys)
CXX = /s/gcc-5.1.0/@sys/bin/g++

CXXFLAGS :=  -Wall -g -std=c++11

SRC_ROOT=$(shell pwd)/../..

prefix?=$(SRC_ROOT)
INSTALL_LIB_SBMODEL_PATH=$(SRC_ROOT)/lib
INSTALL_INC_SBMODEL_PATH=$(SRC_ROOT)/include/softbrain-config
INSTALL_LIB_SBSCHED_PATH=$(SRC_ROOT)/lib
INSTALL_INC_SBSCHED_PATH=$(SRC_ROOT)/include/softbrain-scheduler

LIB_SBMODEL_PATH=../../softbrain-config/lib
INC_SBMODEL_PATH=../../softbrain-config/src
LIB_SBSCHED_PATH=../lib
INC_SBSCHED_PATH=../src


CXXFLAGS += -I$(INC_SBMODEL_PATH) -I$(INC_SBSCHED_PATH) -Wl,-rpath,$(LIB_SBMODEL_PATH) -Wl,-rpath,$(LIB_SBSCHED_PATH)  -Wl,-rpath,$(INSTALL_LIB_SBMODEL_PATH) -Wl,-rpath,$(INSTALL_LIB_SBSCHED_PATH) 


#all: reschedule stat-config

all: sb_sched reschedule stat-config


reschedule : reschedule.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_SBMODEL_PATH) -L$(LIB_SBSCHED_PATH) -lsbconfig -lsbscheduler

stat-config : stat-config.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_SBMODEL_PATH) -L$(LIB_SBSCHED_PATH) -lsbconfig -lsbscheduler

sb_sched : sb_sched.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_SBMODEL_PATH) -L$(LIB_SBSCHED_PATH) -lsbconfig -lsbscheduler -Wl,-rpath,${prefix}/lib


clean:
	rm -rf *.o  sb_sched stat-config reschedule *.d
