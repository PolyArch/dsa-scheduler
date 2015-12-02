MKDIR_P = mkdir -p

prefix ?= $(shell pwd)/../

.PHONY:  directories program

all: directories program

program:
	+make -C src
	+make -C drivers

install: program
	${MKDIR_P} ${prefix}/lib
	cp lib/* ${prefix}/lib
	${MKDIR_P} ${prefix}/include/softbrain-scheduler
	cp src/*.h ${prefix}/include/softbrain-scheduler/
	${MKDIR_P} ${prefix}/bin
	cp drivers/sb_sched ${prefix}/bin

directories:
	${MKDIR_P} obj
	${MKDIR_P} lib

clean:
	make -C src clean
	make -C drivers clean
