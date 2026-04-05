cmake_minimum_required(VERSION 3.16)
project(robot_control_manager CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 出力先を自分のbuildディレクトリに固定
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ARROW REQUIRED arrow)
find_package(Eigen3 REQUIRED)

# Dora node api (built from ~/dora)
set(DORA_ROOT "$ENV{HOME}/dora")
set(DORA_NODE_API_INC "${DORA_ROOT}/target/cxxbridge/dora-node-api-cxx/install")
set(DORA_NODE_API_LIB "${DORA_ROOT}/target/release/libdora_node_api_cxx.a")

add_executable(robot_control_manager
  main.cpp
  robot_control_manager.cpp
  ../../controller/angle_pid_controller.cpp
  ../../controller/lqr_controller.cpp
  ../../controller/body_state_ekf.cpp
  ../../lib/robot_config.cpp
  ${DORA_NODE_API_INC}/dora-node-api.cc
)

target_include_directories(robot_control_manager PRIVATE
  ${DORA_NODE_API_INC}
  ${ARROW_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/vendor/OpenKF/src
)

target_link_libraries(robot_control_manager PRIVATE
  ${DORA_NODE_API_LIB}
  ${ARROW_LIBRARIES}
  Eigen3::Eigen
  pthread
  dl
)
