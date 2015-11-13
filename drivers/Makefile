SYS = $(shell sys)
ifeq ($(SYS),amd64_rhel6)
    CXX = /s/gcc-4.6.2/@sys/bin/g++
else
    CXX = /s/gcc-4.6.1/@sys/bin/g++
endif


CXXFLAGS :=  -Wall -g -std=c++0x -lboost_regex

SRC_ROOT=../..
LIB_DYMODEL_PATH=$(SRC_ROOT)/sb_config/lib
INC_DYMODEL_PATH=$(SRC_ROOT)/sb_config/src
LIB_DYSCHED_PATH=$(SRC_ROOT)/sb_sched/lib
INC_DYSCHED_PATH=$(SRC_ROOT)/sb_sched/src

CXXFLAGS += -I$(INC_DYMODEL_PATH) -I$(INC_DYSCHED_PATH) -Wl,-rpath,$(shell pwd)/../lib -Wl,-rpath,$(shell pwd)/../../sb_config/lib

#all: reschedule stat-config

all: sb_sched reschedule stat-config


reschedule : reschedule.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -L$(LIB_DYMODEL_PATH) -L$(LIB_DYSCHED_PATH) -ldymodel -ldyscheduler

stat-config : stat-config.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -L$(LIB_DYMODEL_PATH) -L$(LIB_DYSCHED_PATH) -ldymodel -ldyscheduler

sb_sched : sb_sched.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -L$(LIB_DYMODEL_PATH) -L$(LIB_DYSCHED_PATH) -ldymodel -ldyscheduler


clean:
	rm -rf *.o  sb_sched stat-config reschedule
