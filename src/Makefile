SYS = $(shell sys)
ifeq ($(SYS),amd64_rhel6)
    CXX = /s/gcc-4.6.2/@sys/bin/g++
else
    CXX = /s/gcc-4.6.1/@sys/bin/g++
endif


CXXFLAGS :=  -Wall -g -std=c++0x -lm $(FLAGS)
SOURCES= dypdg.cpp schedule.cpp scheduler.cpp color_mapper.cpp

INCLUDE_DEST=../inc
LIB_DEST=../lib
OBJ_DEST=../obj

PRE_OBJECTS=$(SOURCES:.cpp=.o)
OBJECTS = $(patsubst %,$(OBJ_DEST)/%,$(PRE_OBJECTS))

SRC_ROOT=../..
LIB_DYMODEL_PATH=$(SRC_ROOT)/dymodel/lib
INC_DYMODEL_PATH=$(SRC_ROOT)/dymodel/inc

all: $(LIB_DEST)/libdyscheduler.a $(LIB_DEST)/libdyscheduler.so

CXXFLAGS += -I$(INC_DYMODEL_PATH)

CXXFLAGS += -I$(INCLUDE_DEST) -fPIC -lboost_regex

$(LIB_DEST)/libdyscheduler.a: $(OBJECTS)
	ar crs $@ $^ 
    
$(LIB_DEST)/libdyscheduler.so: $(OBJECTS)
	$(CXX) $(CXXFLAGS) -shared -o $@ $^     
    
$(OBJ_DEST)/%.o: %.cpp $(INCLUDE_DEST)/%.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

.phony: clean

clean:
	-rm -Rf $(LIB_DEST)/*.so $(LIB_DEST)/*.a *.o $(OBJ_DEST)/*.o
