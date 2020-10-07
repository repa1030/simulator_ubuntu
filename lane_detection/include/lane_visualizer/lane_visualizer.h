// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#ifndef LANE_VISUALIZER_H
#define LANE_VISUALIZER_H

// ROS
#include <ros/ros.h>
#include <hska_msgs/LaneBoundaryArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <lane_detection/LaneVisualizerConfig.h>

// C++
#include <cmath>

class LaneVisualizer {

    public:

        LaneVisualizer();
        ~LaneVisualizer(); 

        // Main loop of node
        void run();

    private:

        // ROS specific
        ros::NodeHandle m_nh;
        ros::Rate m_loop_rate;
        ros::Publisher m_lanes_rviz_pub;
        ros::Publisher m_waypoint_rviz_pub;
        ros::Publisher m_param_update_pub;
        ros::Subscriber m_lane_lines_sub;
        ros::Subscriber m_waypoint_sub;

        // Messages
        std_msgs::ColorRGBA m_color_left;
        std_msgs::ColorRGBA m_color_right;
        std_msgs::ColorRGBA m_color_others;
        geometry_msgs::Vector3 m_scale_left;
        geometry_msgs::Vector3 m_scale_right;
        geometry_msgs::Vector3 m_scale_others;
        geometry_msgs::PointStamped m_current_waypoint;
        std_msgs::Bool m_param_update;
        hska_msgs::LaneBoundaryArray m_lane_lines;
        visualization_msgs::MarkerArray m_markers;
        visualization_msgs::Marker m_waypoint_marker;
        visualization_msgs::Marker m_line_marker;
		
        // Member variables;	
       	bool m_lane_lines_set;
        bool m_current_waypoint_set;
        bool m_init;
        unsigned int m_last_max_id;
       	double m_frequency;

        // Dynamic reconfigure
        dynamic_reconfigure::Server<lane_detection::LaneVisualizerConfig> server;
        dynamic_reconfigure::Server<lane_detection::LaneVisualizerConfig>::CallbackType f;

        // Member Functions
        void init();
        // Visualize the waypoint in rviz
        void visualizeWaypoint();
        // Fill the lane line marker array
        bool fillMarkerArray();

        // Callbacks
        void callbackCurrentLaneLines(const hska_msgs::LaneBoundaryArrayConstPtr& msg);
        void callbackCurrentWaypoint(const geometry_msgs::PointStampedConstPtr& msg);
        void callbackParameters(lane_detection::LaneVisualizerConfig &config, uint32_t level);

};

#endif
