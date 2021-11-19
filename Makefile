CMAKE_BUILD_TYPE ?= Release
all: 3rd-party/libtorch json dsa

3rd-party/libtorch:
	wget -O $@.zip https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.9.0%2Bcpu.zip && \
	unzip $@.zip
	mv libtorch $@

json:
	mkdir -p 3rd-party/jsoncpp/build && cd 3rd-party/jsoncpp/build && \
	cmake .. -DCMAKE_INSTALL_PREFIX=$(SS_TOOLS)  && make install -j

dsa: json
	mkdir -p build && cd build && \
	cmake -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) \
	      -DTorch_DIR="$(shell git rev-parse --show-toplevel)/3rd-party/libtorch/share/cmake/Torch" .. && \
	make install -j

clean:
	rm -rf build

uninstall:
	find include | grep "\.h" | xargs -I {} rm -f $(SS_TOOLS)/{}
	rm -rf $(SS_TOOLS)/include/json
	rm -f $(SS_TOOLS)/lib/libdsa.so
	# TODO(@were): Uninstall json libs