include $(SS_STACK)/msg.mk
level=../
include ../make.config


SYS = $(shell sys)
CXX = g++

OPT ?= -O3
#OPT = -Og
#OPT = -O0

LEX = lex
YACC = yacc -d

CXXFLAGS :=  -Wall -g -std=c++11 $(OPT) -ggdb -gdwarf-3 -ly -ll -lm -ltcmalloc $(FLAGS)
SOURCES= sbpdg.cpp schedule.cpp scheduler.cpp color_mapper.cpp scheduler_bkt.cpp scheduler_greedy.cpp scheduler_mlg.cpp scheduler_sg.cpp scheduler_sa.cpp scheduler_gams.cpp 

GAMS_DIR=gams_models
GAMS_FILES=$(wildcard gams_models/*.gms)
GAMS_INC=$(GAMS_FILES:.gms=.h)

INCLUDE_DEST=../src
LIB_DEST=${build}/lib
OBJ_DEST=${build}/obj


PRE_OBJECTS=$(SOURCES:.cpp=.o)
OBJECTS = $(patsubst %,$(OBJ_DEST)/%,$(PRE_OBJECTS)) dfg-parser.tab.o dfg-lex.yy.o

BOOST_PATH=${boost}
INC_SBMODEL_PATH=${SS_TOOLS}/include/ss-config/

all: $(LIB_DEST)/libsbscheduler.a $(LIB_DEST)/libsbscheduler.so

CXXFLAGS += -I$(INC_SBMODEL_PATH)

CXXFLAGS += -I$(INCLUDE_DEST) -L$(BOOST_PATH) -fPIC -lboost_regex

$(LIB_DEST)/libsbscheduler.a: $(OBJECTS)
	ar crs $@ $^ 
    
$(LIB_DEST)/libsbscheduler.so: $(OBJECTS)
	$(CXX) $(CXXFLAGS) -MD -shared -o $@ $^ $(LIBS)

$(OBJ_DEST)/scheduler_gams.o: scheduler_gams.cpp $(GAMS_INC) dfg-parser.tab.h
	$(CXX) $(CXXFLAGS) -MD -c -o $@ $<

$(OBJ_DEST)/%.o: %.cpp $(INCLUDE_DEST)/%.h dfg-parser.tab.h
	$(CXX) $(CXXFLAGS) -MD -c -o $@ $<

$(GAMS_DIR)/%.h: $(GAMS_DIR)/%.gms
	xxd -i $< > $@


#-------- stuff to do with dfg-parser/lexer -----------
dfg-parser.tab.o: dfg-parser.tab.c sbpdg.h
	$(CXX) $(filter-out -Wall, $(CXXFLAGS)) -c -o $@ $<

dfg-lex.yy.o: dfg-lex.yy.c
	$(CXX) $(filter-out -Wall, $(CXXFLAGS)) -c -o $@ $<

dfg-parser.tab.c dfg-parser.tab.h: dfg-parser.y
	$(YACC) -o dfg-parser.tab.c -v $^

dfg-lex.yy.c: dfg-parser.l dfg-parser.tab.h
	$(LEX) -o dfg-lex.yy.c $<
#----------------------------------------


.phony: clean

clean:
	-rm -Rf $(LIB_DEST)/*.so $(LIB_DEST)/*.a *.o $(OBJ_DEST)/*.o  *.d  $(OBJ_DEST)/*.d  $(GAMS_INC) dfg-lex.yy.c dfg-lex.yy.o *.tab.*  dfg-parser *.output

include ../make.rules

