level=./
include make.config

.PHONY:  directories program make_drivers

all: directories program make_drivers

program:
	+make -C src

make_drivers: install
	+make -C drivers

install: install_program install_drivers 
	

install_drivers: make_drivers
	${MKDIR_P} ${prefix}/bin
	cp drivers/sb_sched ${prefix}/bin


install_program: program
ifndef prefix
$(error Install directory "prefix" is undefined)
endif
	${MKDIR_P} ${prefix}/lib
	cp ${build}/lib/* ${prefix}/lib
	${MKDIR_P} ${prefix}/include/softbrain-scheduler
	cp src/*.h ${prefix}/include/softbrain-scheduler/

clean:
	make -C src clean
	make -C drivers clean

include make.rules

