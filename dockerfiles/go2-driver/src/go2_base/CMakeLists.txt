cmake_minimum_required(VERSION 3.5)
project(go2_base)
add_compile_options(-std=c++17)

set(AMENT_DEPENDENCIES rclcpp 
                       rclpy
                       geometry_msgs
                       sensor_msgs
                       nav_msgs
                       tf2
                       tf2_ros
                       std_srvs 
                       std_msgs
                       go2_srvs
                       rosbag2_cpp)

set (DEPENDENCY_LIST unitree_go
                     unitree_api
                     rclcpp
                     std_msgs
                     std_srvs
                     go2_srvs
                     rosbag2_cpp
)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(go2_srvs REQUIRED)

# Include Cpp "include" directory
include_directories(
    include /usr/include/eigen3
    include/common 
    include/nlohmann
)

link_directories(src)

add_executable(go2_highroscontrol src/go2_highroscontrol.cpp 
                                  src/common/ros2_sport_client.cpp)

ament_target_dependencies(go2_highroscontrol ${DEPENDENCY_LIST})

install(PROGRAMS  go2_base/go2_statepublisher
                  go2_base/go2_scanpublisher
                  go2_base/go2_camerapublisher
                  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
        go2_highroscontrol
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
ament_package()
