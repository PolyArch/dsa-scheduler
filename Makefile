CMAKE_BUILD_TYPE ?= Release

ifeq ($(OS),Windows_NT)
    TORCH_URL := https://download.pytorch.org/libtorch/cpu/libtorch-win-shared-with-deps-1.11.0%2Bcpu.zip
	wget -O $@ $(TORCH_URL)
else
    UNAME_S := $(shell uname -s)
    UNAME_P := $(shell uname -p)
    ifeq ($(UNAME_S),Linux)
        TORCH_URL := https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.9.0%2Bcpu.zip
    endif
    ifeq ($(UNAME_S),Darwin)
        TORCH_URL := https://download.pytorch.org/libtorch/cpu/libtorch-macos-1.11.0.zip
    endif
endif

all: 3rd-party/libtorch json dsa

# Get the download zip URL of libtorch
libtorch.zip:
	wget -O $@ $(TORCH_URL)

# Install LibTorch
3rd-party/libtorch: libtorch.zip
ifeq ($(UNAME_P),arm)
    ifeq ($(UNAME_S),Darwin)
	if [ ! -d "pytorch" ]; then \
		git clone -b master --recurse-submodule https://github.com/pytorch/pytorch.git; \
	fi
	mkdir -p pytorch/build && mkdir -p pytorch/install
	cd pytorch/build && \
	cmake -DBUILD_SHARED_LIBS:BOOL=ON -DCMAKE_BUILD_TYPE:STRING=Release -DPYTHON_EXECUTABLE:PATH=`which python3` -DCMAKE_INSTALL_PREFIX:PATH=../install .. && \
	cmake --build . --target install -j && cd ../..
	mv pytorch/install $@
    endif
else
	unzip $^
	mv libtorch $@
endif

json:
	mkdir -p 3rd-party/jsoncpp/build && cd 3rd-party/jsoncpp/build && \
	cmake .. -DCMAKE_INSTALL_PREFIX=$(SS_TOOLS)  && make install -j

dsa: json 3rd-party/libtorch
	mkdir -p build && cd build && \
	cmake -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) \
	      -DTorch_DIR="$(shell git rev-parse --show-toplevel)/3rd-party/libtorch/share/cmake/Torch" .. && \
	make install -j

clean:
	rm -rf build

ultraclean: clean
	rm -rf 3rd-party/libtorch

uninstall:
	find include | grep "\.h" | xargs -I {} rm -f $(SS_TOOLS)/{}
	rm -rf $(SS_TOOLS)/include/json
	rm -f $(SS_TOOLS)/lib/libdsa.so
	# TODO(@were): Uninstall json libs
