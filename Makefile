all: json dsa

json:
	mkdir -p 3rd-party/jsoncpp/build && cd 3rd-party/jsoncpp/build && \
	cmake .. -DCMAKE_INSTALL_PREFIX=$(SS_TOOLS) && make install -j

dsa: json
	mkdir -p build && cd build && cp ../config.cmake . && cmake .. && make install -j

clean:
	rm -rf build

uninstall:
	find include | grep "\.h" | xargs -I {} rm -f $(SS_TOOLS)/{}
	rm -rf $(SS_TOOLS)/include/json
	rm -f $(SS_TOOLS)/lib/libdsa.so
	# TODO(@were): Uninstall json libs
