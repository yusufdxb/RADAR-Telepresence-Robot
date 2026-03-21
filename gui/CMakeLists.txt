cmake_minimum_required(VERSION 3.16)
project(radar_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(radar_gui main.cpp)

target_link_libraries(radar_gui
    Qt6::Core
    Qt6::Widgets
)

ament_target_dependencies(radar_gui
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
)

# Build instructions:
#
#   mkdir build && cd build
#   cmake .. -DCMAKE_PREFIX_PATH=/opt/ros/jazzy
#   make -j$(nproc)
#   ./radar_gui
