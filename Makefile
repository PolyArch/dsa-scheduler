ifndef prefix
$(error Install directory "prefix" is undefined)
endif

level=./
include make.config

.PHONY:  directories program

all: directories program

program:
	+make -C src
	+make -C drivers

install: program
	${MKDIR_P} ${prefix}/lib
	cp ${build}/lib/* ${prefix}/lib
	${MKDIR_P} ${prefix}/include/softbrain-scheduler
	cp src/*.h ${prefix}/include/softbrain-scheduler/
	${MKDIR_P} ${prefix}/bin
	cp drivers/sb_sched ${prefix}/bin

clean:
	make -C src clean
	make -C drivers clean

include make.rules

