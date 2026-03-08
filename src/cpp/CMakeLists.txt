cmake_minimum_required(VERSION 3.16)
project(dora_cpp_node CXX)

# Add subdirectories
add_subdirectory(node/dummy)
add_subdirectory(node/dummy2)
add_subdirectory(node/robot_control_manager)
add_subdirectory(node/device_control_manager)
add_subdirectory(node/imu_node)