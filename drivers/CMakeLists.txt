file(GLOB_RECURSE DRIVER_SRCS *.cpp)

add_executable(ss_sched ${DRIVER_SRCS})
target_include_directories(ss_sched PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(ss_sched PRIVATE ssconfig ssscheduler ${Boost_LIBRARIES})

install(TARGETS ss_sched)
