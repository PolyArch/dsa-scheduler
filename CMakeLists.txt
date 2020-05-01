cmake_minimum_required(VERSION 3.12)

project(SpatialScheduler)

# Use config.cmake file
if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/config.cmake)
  include(${CMAKE_CURRENT_BINARY_DIR}/config.cmake)
  message(STATUS "Using ${CMAKE_CURRENT_BINARY_DIR}/config.cmake")
else()
  message(STATUS "config.cmake not found, use defined configs")
endif()

add_subdirectory(${CMAKE_SOURCE_DIR}/json-parser)

# Build assumptions
set(BUILD_SHARED_LIBS ON)
if (DEFINED ENV{SS_TOOLS})
  set(CMAKE_INSTALL_PREFIX $ENV{SS_TOOLS})
else()
  message(FATAL "Please set SS_TOOLS to build!")
endif()

# Debug/Release
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3")
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions("DEBUG_MODE")
endif()

# Find dependencies: Boost and Parser
find_package(Boost REQUIRED system serialization)
find_package(FLEX REQUIRED)
find_package(BISON REQUIRED)


FLEX_TARGET(DFG_LEXER ${CMAKE_SOURCE_DIR}/src/ir/dfg-parser.l
  ${CMAKE_SOURCE_DIR}/src/ir/dfg-parser.yy.cc
  DEFINES_FILE ${CMAKE_SOURCE_DIR}/src/ir/dfg-parser.h)

BISON_TARGET(DFG_PARSER ${CMAKE_SOURCE_DIR}/src/ir/dfg-parser.y
  ${CMAKE_SOURCE_DIR}/src/ir/dfg-parser.tab.cc
  DEFINES_FILE ${CMAKE_SOURCE_DIR}/src/ir/dfg-parser.tab.h)

# Generate ssinsts.{cpp/h}
add_executable(inst_model ${CMAKE_SOURCE_DIR}/src/insts/inst_model.cpp)
target_include_directories(inst_model PRIVATE ${CMAKE_SOURCE_DIR}/include)

set(SS_INSTS_H ${CMAKE_SOURCE_DIR}/include/dsa/arch/ssinst.h)
set(SS_INSTS_CXX ${CMAKE_SOURCE_DIR}/src/insts/ssinst.cpp)
set(INST_MODEL ${CMAKE_SOURCE_DIR}/src/insts/"full.ssinst")
set(PA_MODEL ${CMAKE_SOURCE_DIR}/src/insts/"full.ssinst.power_area.csv")

add_custom_command(
  OUTPUT ${SS_INSTS_H} ${SS_INSTS_CXX}
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/inst_model ${INST_MODEL} ${PA_MODEL} ${SS_INSTS_H} ${SS_INSTS_CXX}
  DEPENDS inst_model
)

# Build the library
file(GLOB_RECURSE SRCS ${CMAKE_SOURCE_DIR}/src/*.cpp)

add_library(dsa
  ${SRCS}
  ${SS_INSTS_CXX}
  ${FLEX_DFG_LEXER_OUTPUTS}
  ${BISON_DFG_PARSER_OUTPUTS})

target_include_directories(dsa PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/json-parser/include
  ${Boost_INCLUDE_DIRS})

target_link_libraries(dsa PRIVATE json ${Boost_LIBRARIES})

# Install the header files and library
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)
install(TARGETS dsa DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/)

add_subdirectory(${CMAKE_SOURCE_DIR}/drivers)
