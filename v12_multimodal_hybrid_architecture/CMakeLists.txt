cmake_minimum_required(VERSION 3.8)
project(v12_multimodal_hybrid_architecture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui videoio imgproc)

# 1. RTSP Camera Driver Node
add_executable(rtsp_camera_node src/rtsp_camera_node.cpp)
target_include_directories(rtsp_camera_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(rtsp_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(rtsp_camera_node ${OpenCV_LIBRARIES})

# 2. IMU & Sensor Node with TiltCompensator
add_executable(imu_sensor_node src/imu_sensor_node.cpp src/tilt_compensator.cpp)
ament_target_dependencies(imu_sensor_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)
find_package(Eigen3 REQUIRED)
target_include_directories(imu_sensor_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

# 3. Lidar Processor Node (Transform & Filter)
add_executable(lidar_processor_node src/lidar_processor_node.cpp src/tilt_compensator.cpp)
ament_target_dependencies(lidar_processor_node
  rclcpp
  sensor_msgs
)
find_package(Eigen3 REQUIRED)
target_include_directories(lidar_processor_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

# Install rules
install(TARGETS
  rtsp_camera_node
  imu_sensor_node
  lidar_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/v12_hybrid_gui.py
  scripts/inclinometer_sensor_bridge.py
  scripts/control_executor_node.py
  scripts/v12_launch_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
