add_executable(ss_sched ss_sched.cpp)
target_include_directories(ss_sched PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(ss_sched PRIVATE ssconfig ssscheduler ${Boost_LIBRARIES})

add_executable(ss_dse ss_dse.cpp)
target_include_directories(ss_dse PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(ss_dse PRIVATE ssconfig ssscheduler ${Boost_LIBRARIES})

install(TARGETS ss_sched)
install(TARGETS ss_dse)
