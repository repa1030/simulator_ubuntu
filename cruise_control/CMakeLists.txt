cmake_minimum_required(VERSION 2.8.3)
project(cruise_control)

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
    cfg/CruiseController.cfg
)

## Declare a catkin package
catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
add_executable(
    cruise_controller
    src/cruise_controller/cruise_controller.cpp
    src/cruise_controller/cruise_controller_node.cpp
)
add_dependencies(
    cruise_controller
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
    cruise_controller
    ${catkin_LIBRARIES}
)


