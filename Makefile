level=./
include make.config

all: directories program make_drivers

include make.rules


program:
	+make -C src

make_drivers: program
	make -C drivers

install: directories install_program install_drivers 
	

install_drivers: make_drivers
	${MKDIR_P} ${prefix}/bin
	cp drivers/sb_sched ${prefix}/bin
	cp drivers/sb_dfg_emu ${prefix}/bin

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


