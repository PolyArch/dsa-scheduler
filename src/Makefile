SYS = $(shell sys)
CXX = /s/gcc-5.1.0/@sys/bin/g++

CXXFLAGS :=  -Wall -g -std=c++11 -O3 -lm $(FLAGS)
SOURCES= sbpdg.cpp schedule.cpp scheduler.cpp color_mapper.cpp

INCLUDE_DEST=../src
LIB_DEST=../lib
OBJ_DEST=../obj

PRE_OBJECTS=$(SOURCES:.cpp=.o)
OBJECTS = $(patsubst %,$(OBJ_DEST)/%,$(PRE_OBJECTS))

SRC_ROOT=../..
LIB_SBMODEL_PATH=$(SRC_ROOT)/softbrain-config/lib
INC_SBMODEL_PATH=$(SRC_ROOT)/softbrain-config/src

all: $(LIB_DEST)/libsbscheduler.a $(LIB_DEST)/libsbscheduler.so

CXXFLAGS += -I$(INC_SBMODEL_PATH)

CXXFLAGS += -I$(INCLUDE_DEST) -fPIC -lboost_regex

$(LIB_DEST)/libsbscheduler.a: $(OBJECTS)
	ar crs $@ $^ 
    
$(LIB_DEST)/libsbscheduler.so: $(OBJECTS)
	$(CXX) $(CXXFLAGS) -MD -shared -o $@ $^     

$(OBJ_DEST)/scheduler.o: scheduler.cpp multi_model.cpp spill_model.cpp softbrain_gams.cpp
	$(CXX) $(CXXFLAGS) -MD -c -o $@ $<

    
$(OBJ_DEST)/%.o: %.cpp $(INCLUDE_DEST)/%.h
	$(CXX) $(CXXFLAGS) -MD -c -o $@ $<

.phony: clean

clean:
	-rm -Rf $(LIB_DEST)/*.so $(LIB_DEST)/*.a *.o $(OBJ_DEST)/*.o  *.d  $(OBJ_DEST)/*.d

