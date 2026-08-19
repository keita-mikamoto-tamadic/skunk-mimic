cmake_minimum_required(VERSION 3.16)
project(stabilizer CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 出力先を自分のbuildディレクトリに固定
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ARROW REQUIRED arrow)
find_package(Eigen3 REQUIRED)

# Pinocchio (posture_ik: MJCF を読んで重心 IK)。pip の `pin` (cmeel ビルド) が
# ~/.local/lib/python3.*/site-packages/cmeel.prefix に C++ ライブラリ・ヘッダ・CMake config を
# 丸ごと置くので、システムに無ければそこを探す (robotpkg / ソースビルドなら通常の探索で見つかる)。
# 別の場所なら -DCMAKE_PREFIX_PATH=<prefix> で指定。
file(GLOB _CMEEL_PREFIXES "$ENV{HOME}/.local/lib/python3*/site-packages/cmeel.prefix")
list(APPEND CMAKE_PREFIX_PATH ${_CMEEL_PREFIXES})
find_package(pinocchio REQUIRED)
message(STATUS "pinocchio ${pinocchio_VERSION} from ${pinocchio_DIR}")

# Dora node api (built from ~/dora)
set(DORA_ROOT "$ENV{HOME}/dora")
set(DORA_NODE_API_INC "${DORA_ROOT}/target/cxxbridge/dora-node-api-cxx/install")
set(DORA_NODE_API_LIB "${DORA_ROOT}/target/release/libdora_node_api_cxx.a")

add_executable(stabilizer
  main.cpp
  angle_pid_controller.cpp
  lqr_controller.cpp
  body_state_ekf.cpp
  posture_ik.cpp
  ../../lib/robot_config.cpp
  ${DORA_NODE_API_INC}/dora-node-api.cc
)

target_include_directories(stabilizer PRIVATE
  ${DORA_NODE_API_INC}
  ${ARROW_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/vendor/OpenKF/src

)

target_link_libraries(stabilizer PRIVATE
  ${DORA_NODE_API_LIB}
  ${ARROW_LIBRARIES}
  Eigen3::Eigen
  pinocchio::pinocchio
  pthread
  dl
)
