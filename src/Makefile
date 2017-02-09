level=../
include ../make.config


SYS = $(shell sys)
CXX = /s/gcc-5.1.0/@sys/bin/g++

CXXFLAGS :=  -Wall -g -std=c++11 -O3 -lm $(FLAGS)
SOURCES= sbpdg.cpp schedule.cpp scheduler.cpp color_mapper.cpp

EXTRA_GAMS=softbrain_gams.cpp timing_model.cpp multi_model.cpp spill_model.cpp 

INCLUDE_DEST=../src
LIB_DEST=${build}/lib
OBJ_DEST=${build}/obj

PRE_OBJECTS=$(SOURCES:.cpp=.o)
OBJECTS = $(patsubst %,$(OBJ_DEST)/%,$(PRE_OBJECTS))

BOOST_PATH=${boost}
INC_SBMODEL_PATH=${prefix}/include/softbrain-config/

all: $(LIB_DEST)/libsbscheduler.a $(LIB_DEST)/libsbscheduler.so

CXXFLAGS += -I$(INC_SBMODEL_PATH)

CXXFLAGS += -I$(INCLUDE_DEST) -L$(BOOST_PATH) -fPIC -lboost_regex

$(LIB_DEST)/libsbscheduler.a: $(OBJECTS)
	ar crs $@ $^ 
    
$(LIB_DEST)/libsbscheduler.so: $(OBJECTS)
	$(CXX) $(CXXFLAGS) -MD -shared -o $@ $^     

$(OBJ_DEST)/scheduler.o: scheduler.cpp $(EXTRA_GAMS)
	$(CXX) $(CXXFLAGS) -MD -c -o $@ $<

    
$(OBJ_DEST)/%.o: %.cpp $(INCLUDE_DEST)/%.h
	$(CXX) $(CXXFLAGS) -MD -c -o $@ $<

.phony: clean

clean:
	-rm -Rf $(LIB_DEST)/*.so $(LIB_DEST)/*.a *.o $(OBJ_DEST)/*.o  *.d  $(OBJ_DEST)/*.d

include ../make.rules

