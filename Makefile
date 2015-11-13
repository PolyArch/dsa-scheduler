MKDIR_P = mkdir -p

.PHONY:  directories program

all: directories program

directories: obj lib

obj:
	${MKDIR_P} $@

lib:
	${MKDIR_P} $@

program:
	+make -C src

