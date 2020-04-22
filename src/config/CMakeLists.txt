# Build the generator of ssinsts.{cpp/h}
add_executable(inst_model
  ${CMAKE_CURRENT_SOURCE_DIR}/inst_model.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/model_parsing.cpp)

# Generate ssinsts.{cpp/h}
set(SS_INSTS_H ${CMAKE_CURRENT_SOURCE_DIR}/ssinst.h)
set(SS_INSTS_CXX ${CMAKE_CURRENT_SOURCE_DIR}/ssinst.cpp)
set(INST_MODEL ${CMAKE_CURRENT_SOURCE_DIR}/"full.ssinst")
set(PA_MODEL ${CMAKE_CURRENT_SOURCE_DIR}/"full.ssinst.power_area.csv")

add_custom_command(
  OUTPUT ${SS_INSTS_H} ${SS_INSTS_CXX}
  COMMAND ${CMAKE_CURRENT_BINARY_DIR}/inst_model ${INST_MODEL} ${PA_MODEL} ${SS_INSTS_H} ${SS_INSTS_CXX}
  DEPENDS inst_model
)

# Build the libs for ss-config
file(GLOB_RECURSE CONFIG_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_library(ssconfig ${CONFIG_SRC} ${SS_INSTS_CXX})

target_include_directories(ssconfig PRIVATE ${CMAKE_SOURCE_DIR}/src ${Boost_INCLUDE_DIRS})
target_link_libraries(ssconfig PRIVATE ${Boost_LIBRARIES})

file(GLOB CONFIG_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
set_target_properties(ssconfig PROPERTIES PUBLIC_HEADER "${CONFIG_HEADERS}")

# Install the header files and libraries
install(TARGETS ssconfig
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ss-config)
