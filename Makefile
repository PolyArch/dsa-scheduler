all:
	mkdir -p build && cd build && cp ../config.cmake . && cmake .. && make -j

install: all
	make -C build install

clean:
	rm -rf build
	find include | grep "\.h" | xargs -I {} rm $(SS_TOOLS)/{}
	rm -f $(SS_TOOLS)/lib/libdsa.so $(SS_TOOLS)/lib/libjson-parser.so
