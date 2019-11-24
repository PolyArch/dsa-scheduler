include $(SS_STACK)/msg.mk

level=../
include ../make.config

SYS = $(shell sys)
CXX = g++

CXXFLAGS := $(OPT) -Wall -g -std=c++11

LIB_PATH=../build/lib 

CXXFLAGS += -I../src/ -Wl,-rpath,$(LIB_PATH) 

LOCAL_LIBS = -ly -lfl -lssscheduler -lssconfig $(LIBS)

all: ss_sched 

ss_sched : ss_sched.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< -L$(LIB_PATH) $(LOCAL_LIBS)  -Wl,-rpath,${SS_TOOLS}/lib

clean:
	rm -rf *.o ss_sched *.d

include ../make.rules

