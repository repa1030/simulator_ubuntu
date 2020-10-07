cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

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
    cfg/PurePursuit.cfg
    cfg/PurePursuitVisualizer.cfg
)

## Declare a catkin package
catkin_package()

## Build nodes
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
### Pure Pursuit
add_executable(
    pure_pursuit
    src/pure_pursuit/pure_pursuit.cpp
    src/pure_pursuit/pure_pursuit_node.cpp
)
add_dependencies(
    pure_pursuit
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
    pure_pursuit
    ${catkin_LIBRARIES}
)
### Pure Pursuit Visualizer
add_executable(
    pure_pursuit_visualizer
    src/pure_pursuit_visualizer/pure_pursuit_visualizer.cpp
    src/pure_pursuit_visualizer/pure_pursuit_visualizer_node.cpp
)
add_dependencies(
    pure_pursuit_visualizer
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
    pure_pursuit_visualizer
    ${catkin_LIBRARIES}
)
