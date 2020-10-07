cmake_minimum_required(VERSION 2.8.3)
project(localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    tf
    dynamic_reconfigure
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/FakeLocalizer.cfg
)

## Declare a catkin package
catkin_package()

## Build nodes
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
add_executable(
    fake_localizer
    src/fake_localizer/fake_localizer.cpp
    src/fake_localizer/fake_localizer_node.cpp
)
add_dependencies(
    fake_localizer
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
    fake_localizer
    ${catkin_LIBRARIES}
)
