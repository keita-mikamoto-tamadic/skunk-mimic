cmake_minimum_required(VERSION 3.16)
project(dcm_frontend CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ARROW REQUIRED arrow)

set(DORA_ROOT "$ENV{HOME}/dora")
set(DORA_NODE_API_INC "${DORA_ROOT}/target/cxxbridge/dora-node-api-cxx/install")
set(DORA_NODE_API_LIB "${DORA_ROOT}/target/release/libdora_node_api_cxx.a")

add_executable(dcm_frontend
  main.cpp
  ../../lib/robot_config.cpp
  ${DORA_NODE_API_INC}/dora-node-api.cc
)

target_include_directories(dcm_frontend PRIVATE
  ${DORA_NODE_API_INC}
  ${ARROW_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_link_libraries(dcm_frontend PRIVATE
  ${DORA_NODE_API_LIB}
  ${ARROW_LIBRARIES}
  pthread
  dl
)
