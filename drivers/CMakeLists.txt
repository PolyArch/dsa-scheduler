add_executable(ss_sched ss_sched.cpp)
target_include_directories(ss_sched PRIVATE
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/3rd-party/cxxopts/include
  ${CMAKE_SOURCE_DIR}/3rd-party/libtorch/include/torch/csrc/api/include
  ${CMAKE_INSTALL_PREFIX}/include)
target_link_directories(ss_sched PRIVATE ${CMAKE_INSTALL_PREFIX}/lib ${CMAKE_INSTALL_PREFIX}/lib64)
target_link_libraries(ss_sched PRIVATE dsa jsoncpp ${TORCH_LIBRARIES})

install(TARGETS ss_sched)
