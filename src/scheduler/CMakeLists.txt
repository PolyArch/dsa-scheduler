# The lexer and parser
find_package(FLEX)

FLEX_TARGET(DFG_LEXER dfg-parser.l ${CMAKE_CURRENT_SOURCE_DIR}/dfg-parser.yy.cc
  DEFINES_FILE ${CMAKE_CURRENT_SOURCE_DIR}/dfg-parser.h)

find_package(BISON)

BISON_TARGET(DFG_PARSER dfg-parser.y
  ${CMAKE_CURRENT_SOURCE_DIR}/dfg-parser.tab.cc
  DEFINES_FILE ${CMAKE_CURRENT_SOURCE_DIR}/dfg-parser.tab.h)

# Build the library
file(GLOB_RECURSE SCHEDULER_SRCS *.cpp)

add_library(ssscheduler
  ${SCHEDULER_SRCS}
  ${FLEX_DFG_LEXER_OUTPUTS}
  ${BISON_DFG_PARSER_OUTPUTS})
target_include_directories(ssscheduler PRIVATE ${CMAKE_SOURCE_DIR}/src ${Boost_INCLUDE_DIRS})
target_link_libraries(ssscheduler PRIVATE ssconfig ${Boost_LIBRARIES})

file(GLOB SCHEDULER_HEADERS *.h)
set_target_properties(ssscheduler PROPERTIES PUBLIC_HEADER "${SCHEDULER_HEADERS}")

# Install the header files and library
install(TARGETS ssscheduler
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ss-scheduler)
