SYS = $(shell sys)
ifeq ($(SYS),amd64_rhel6)
    CXX = /s/gcc-4.6.2/@sys/bin/g++
else
    CXX = /s/gcc-4.6.1/@sys/bin/g++
endif


CXXFLAGS :=  -Wall -g -std=c++0x -lboost_regex

SRC_ROOT=../..
LIB_DYMODEL_PATH=$(SRC_ROOT)/dymodel/lib
INC_DYMODEL_PATH=$(SRC_ROOT)/dymodel/inc
LIB_DYSCHED_PATH=$(SRC_ROOT)/scheduler/lib
INC_DYSCHED_PATH=$(SRC_ROOT)/scheduler/inc

CXXFLAGS += -I$(INC_DYMODEL_PATH) -I$(INC_DYSCHED_PATH)

all: resched stat-config

resched : resched.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -L$(LIB_DYMODEL_PATH) -L$(LIB_DYSCHED_PATH) -ldymodel -ldyscheduler

stat-config : stat-config.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -L$(LIB_DYMODEL_PATH) -L$(LIB_DYSCHED_PATH) -ldymodel -ldyscheduler

clean:
	rm -rf *.o  resched stat-config
