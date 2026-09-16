cmake_minimum_required(VERSION 3.5)  # ROS2要求最低3.5
project(describe_60FED)

# 使用ament_cmake替代catkin
find_package(ament_cmake REQUIRED)

# 安装目录 - 使用固定路径格式
foreach(dir meshes urdf launch config)
    install(
        DIRECTORY ${dir}/
        DESTINATION share/${PROJECT_NAME}/${dir}
    )
endforeach()

# 替换catkin_package()
ament_package()
