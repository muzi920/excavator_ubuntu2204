cmake_minimum_required(VERSION 3.8)
project(dsvt_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 纯 Python ROS2 包 (ament_python 通过 setup.py 处理)
# 此 CMakeLists.txt 用于 colcon build 兼容性
find_package(ament_cmake REQUIRED)

ament_package()
