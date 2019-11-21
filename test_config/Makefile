prefix?=$(SS_TOOLS)/lib

SYS = $(shell sys)
CXX = g++

client.out : client.cpp
	$(CXX) -g -std=c++11  -MD -o $@ $< -I${prefix}/src  -L${prefix}/lib -lsbconfig -Wl,-rpath,${prefix}/lib

clean:
	rm -rf *.o  *.out *.d

test_area_power.out: %.out: %.cpp
	$(CXX) -g -std=c++11  -MD -o $@ $< -I${prefix}/src ${prefix}/libssconfig.a -lm -lz
