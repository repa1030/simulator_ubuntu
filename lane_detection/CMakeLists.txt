cmake_minimum_required(VERSION 2.8.3)
project(lane_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    cv_bridge
    image_transport
    dynamic_reconfigure
    hska_msgs
)

## System dependencies
find_package(OpenCV REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/LaneDetector.cfg
    cfg/LaneVisualizer.cfg
)

## Declare a catkin package
catkin_package()

## Build nodes
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
### Lane detector
add_executable(
    lane_detector
    src/lane_detector/lane_detector.cpp
    src/lane_detector/lane_detector_node.cpp
    src/poly_fit.cpp
    src/peak_finder.cpp
)
add_dependencies(
    lane_detector
    hska_msgs_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
    lane_detector
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)
### Lane visualizer
add_executable(
    lane_visualizer
    src/lane_visualizer/lane_visualizer.cpp
    src/lane_visualizer/lane_visualizer_node.cpp
)
add_dependencies(
    lane_visualizer
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
    lane_visualizer
    ${catkin_LIBRARIES}
)
