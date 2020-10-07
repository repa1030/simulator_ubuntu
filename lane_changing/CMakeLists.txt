cmake_minimum_required(VERSION 2.8.3)
project(lane_changing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    dynamic_reconfigure
    hska_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/LaneChanger.cfg
)

## Declare a catkin package
catkin_package()

## Build nodes
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
### Lane Changer
add_executable(
    lane_changer
    src/lane_changer/lane_changer.cpp
    src/lane_changer/lane_changer_node.cpp
)
add_dependencies(
    lane_changer
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
    lane_changer
    ${catkin_LIBRARIES}
)
