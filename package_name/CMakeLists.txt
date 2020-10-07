# TODO change "package_name" with the name of your package
# TODO change "node#_name" with the name of your nodes
cmake_minimum_required(VERSION 2.8.3)
project(package_name)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
# TODO adding used package (message, dynamic reconfigure)
# for example
    geometry_msgs
    dynamic_reconfigure
# or specially created messages
    hska_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
# TODO Adding all used cfg-files from cfg-folder
    cfg/Node1Name.cfg
    cfg/Node2Name.cfg
)

## Declare a catkin package
catkin_package() # catkin-provided CMake macro

## Build nodes
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
## Node1
add_executable(
    node1_name
# TODO adding cpp-files
    src/node1_name/node1_name.cpp
    src/node1_name/node1_name_node.cpp
)
add_dependencies(
    node1_name
# TODO adding special dependencies
# for example:
    hska_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
    node1_name
    ${catkin_LIBRARIES}
)
## Node2
add_executable(
    node2_name
# TODO adding cpp-files
    src/node2_name/node2_name.cpp
    src/node2_name/node2_name_node.cpp
)
add_dependencies(
    node2_name
# TODO adding special dependencies
# for example:
    hska_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
    node2_name
    ${catkin_LIBRARIES}
)

## For help: http://wiki.ros.org/catkin/CMakeLists.txt
