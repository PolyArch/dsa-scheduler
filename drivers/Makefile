SYS = $(shell sys)
CXX = /s/gcc-5.1.0/@sys/bin/g++

CXXFLAGS :=  -Wall -g -std=c++11

SRC_ROOT=$(shell pwd)/../..

prefix?=$(SRC_ROOT)
INSTALL_LIB_DYMODEL_PATH=$(SRC_ROOT)/lib
INSTALL_INC_DYMODEL_PATH=$(SRC_ROOT)/include/softbrain-config
INSTALL_LIB_DYSCHED_PATH=$(SRC_ROOT)/lib
INSTALL_INC_DYSCHED_PATH=$(SRC_ROOT)/include/softbrain-scheduler

LIB_DYMODEL_PATH=../../softbrain-config/lib
INC_DYMODEL_PATH=../../softbrain-config/src
LIB_DYSCHED_PATH=../lib
INC_DYSCHED_PATH=../src


CXXFLAGS += -I$(INC_DYMODEL_PATH) -I$(INC_DYSCHED_PATH) -Wl,-rpath,$(LIB_DYMODEL_PATH) -Wl,-rpath,$(LIB_DYSCHED_PATH)  -Wl,-rpath,$(INSTALL_LIB_DYMODEL_PATH) -Wl,-rpath,$(INSTALL_LIB_DYSCHED_PATH) 


#all: reschedule stat-config

all: sb_sched reschedule stat-config


reschedule : reschedule.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_DYMODEL_PATH) -L$(LIB_DYSCHED_PATH) -lsbconfig -lsbscheduler

stat-config : stat-config.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_DYMODEL_PATH) -L$(LIB_DYSCHED_PATH) -lsbconfig -lsbscheduler

sb_sched : sb_sched.cpp
	$(CXX) $(CXXFLAGS) -MD -o $@ $< -L$(LIB_DYMODEL_PATH) -L$(LIB_DYSCHED_PATH) -lsbconfig -lsbscheduler


clean:
	rm -rf *.o  sb_sched stat-config reschedule
