add_executable(ss_sched ss_sched.cpp)
target_include_directories(ss_sched PRIVATE
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/3rd-party/cxxopts/include
  ${CMAKE_SOURCE_DIR}/json-parser/include
  ${CMAKE_INSTALL_PREFIX}/include)
target_link_libraries(ss_sched PRIVATE dsa json)

add_executable(ss_dse ss_dse.cpp)
target_include_directories(ss_dse PRIVATE
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/3rd-party/cxxopts/include
  ${CMAKE_SOURCE_DIR}/json-parser/include
  ${CMAKE_INSTALL_PREFIX}/include)
target_link_libraries(ss_dse PRIVATE dsa json)

install(TARGETS ss_sched)
install(TARGETS ss_dse)
