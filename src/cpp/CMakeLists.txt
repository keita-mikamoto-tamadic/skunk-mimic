cmake_minimum_required(VERSION 3.16)
project(dora_cpp_node CXX)

# --- データフォーマット コード生成 ---
# src/data_format/axis_data.json を正本に C++/Python の定義を生成する。
# JSON か生成器を変更すると configure が再実行される。
set(_DATA_FORMAT_JSON ${CMAKE_CURRENT_SOURCE_DIR}/../data_format/axis_data.json)
set(_DATA_FORMAT_GEN  ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/gen_data_format.py)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${_DATA_FORMAT_JSON} ${_DATA_FORMAT_GEN})
execute_process(
  COMMAND python3 ${_DATA_FORMAT_GEN}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../..
  RESULT_VARIABLE _DATA_FORMAT_RESULT
)
if(NOT _DATA_FORMAT_RESULT EQUAL 0)
  message(FATAL_ERROR "data_format codegen failed (tools/gen_data_format.py)")
endif()

# Add subdirectories
add_subdirectory(node/dummy)
add_subdirectory(node/dummy2)
add_subdirectory(node/robot_control_manager)
add_subdirectory(node/device_control_manager)
add_subdirectory(node/imu_node)
add_subdirectory(node/stabilizer)