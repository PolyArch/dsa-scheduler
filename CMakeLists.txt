cmake_minimum_required(VERSION 3.12)

# Use config.cmake file
if(EXISTS ${CMAKE_CURRENT_BINARY_DIR}/config.cmake)
  include(${CMAKE_CURRENT_BINARY_DIR}/config.cmake)
  message(STATUS "Using ${CMAKE_CURRENT_BINARY_DIR}/config.cmake")
else()
  message(STATUS "config.cmake not found, use defined configs")
endif()

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

# Add Subprojects
find_package(Boost REQUIRED system serialization)

add_subdirectory(src/ss-config)
add_subdirectory(src/ss-scheduler)
add_subdirectory(drivers)
