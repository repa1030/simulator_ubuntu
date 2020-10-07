cmake_minimum_required(VERSION 2.8.3)
project(hska_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    SteeringCommand.msg
    PedalCommand.msg
    LaneBoundary.msg
    LaneBoundaryArray.msg
    ADASCommand.msg
    WheelRotationalSpeed.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
