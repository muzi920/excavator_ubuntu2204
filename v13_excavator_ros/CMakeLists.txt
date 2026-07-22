cmake_minimum_required(VERSION 3.8)
project(v13_excavator_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControllerCommand.msg"
  "msg/Inclinometer.msg"
  "msg/InclinometerGroup.msg"
  "msg/LidarImu.msg"
  "msg/RobotJointState.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

include_directories(include)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(v13_common
  src/protocol_helpers.cpp
  src/serial_port.cpp
  src/joint_fusion_helpers.cpp
)
target_compile_features(v13_common PUBLIC cxx_std_17)
target_include_directories(v13_common PUBLIC include)
ament_target_dependencies(v13_common
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(inclinometer_reader src/inclinometer_reader.cpp)
target_compile_features(inclinometer_reader PUBLIC cxx_std_17)
target_include_directories(inclinometer_reader PUBLIC include)
target_link_libraries(inclinometer_reader v13_common "${cpp_typesupport_target}")
ament_target_dependencies(inclinometer_reader rclcpp std_msgs)

add_executable(lidar_reader src/lidar_reader.cpp)
target_compile_features(lidar_reader PUBLIC cxx_std_17)
target_include_directories(lidar_reader PUBLIC include)
target_link_libraries(lidar_reader v13_common "${cpp_typesupport_target}")
ament_target_dependencies(lidar_reader rclcpp sensor_msgs std_msgs)

add_executable(lidar_imu_reader src/lidar_imu_reader.cpp)
target_compile_features(lidar_imu_reader PUBLIC cxx_std_17)
target_include_directories(lidar_imu_reader PUBLIC include)
target_link_libraries(lidar_imu_reader v13_common "${cpp_typesupport_target}")
ament_target_dependencies(lidar_imu_reader rclcpp std_msgs)

add_executable(controller_node src/controller_node.cpp)
target_compile_features(controller_node PUBLIC cxx_std_17)
target_include_directories(controller_node PUBLIC include)
target_link_libraries(controller_node v13_common "${cpp_typesupport_target}")
ament_target_dependencies(controller_node rclcpp std_msgs)

add_executable(v13_main_node src/v13_main_node.cpp)
target_compile_features(v13_main_node PUBLIC cxx_std_17)
target_include_directories(v13_main_node PUBLIC include)
target_link_libraries(v13_main_node v13_common "${cpp_typesupport_target}")
ament_target_dependencies(v13_main_node rclcpp std_msgs)

if(BUILD_TESTING)
  ament_add_gtest(test_protocol_helpers test/test_protocol_helpers.cpp)
  ament_add_gtest(test_joint_fusion test/test_joint_fusion.cpp)

  if(TARGET test_protocol_helpers)
    target_compile_features(test_protocol_helpers PUBLIC cxx_std_17)
    target_include_directories(test_protocol_helpers PUBLIC include)
    target_link_libraries(test_protocol_helpers v13_common)
  endif()

  if(TARGET test_joint_fusion)
    target_compile_features(test_joint_fusion PUBLIC cxx_std_17)
    target_include_directories(test_joint_fusion PUBLIC include)
    target_link_libraries(test_joint_fusion v13_common)
  endif()
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/v13_python_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  v13_common
  inclinometer_reader
  lidar_reader
  lidar_imu_reader
  controller_node
  v13_main_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
