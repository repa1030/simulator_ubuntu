// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#include "lane_visualizer/lane_visualizer.h"

// Constructor
LaneVisualizer::LaneVisualizer() : 
  m_loop_rate(30.0),
  m_current_waypoint_set(false),
  m_lane_lines_set(false),
  m_init (false) {

    init();

}

// Destructor
LaneVisualizer::~LaneVisualizer() {
}

// Initialization
void LaneVisualizer::init() {

    ROS_INFO("***[lane_visualizer]: Node Initializing.");

    // Dynamic reconfigure
    f = boost::bind(&LaneVisualizer::callbackParameters, this, _1, _2);
    server.setCallback(f);

    // Publisher
    m_lanes_rviz_pub = m_nh.advertise<visualization_msgs::MarkerArray>("/visualization/lane_lines", 1);
    m_waypoint_rviz_pub = m_nh.advertise<visualization_msgs::Marker>("/visualization/waypoint", 1);
    m_param_update_pub = m_nh.advertise<std_msgs::Bool>("/lane_visualizer/param_update_msg", 1);

    // Subscriber
    m_lane_lines_sub = m_nh.subscribe("/lanes/lines", 1, &LaneVisualizer::callbackCurrentLaneLines, this);
    m_waypoint_sub = m_nh.subscribe("/lanes/next_waypoint", 1, &LaneVisualizer::callbackCurrentWaypoint, this);

    // Parameters from parameter server
    m_nh.param("frequency", m_frequency, 30.0);
    
    m_nh.param("scale_x_l", m_scale_left.x, 0.1);
    m_nh.param("scale_y_l", m_scale_left.y, 0.1);
    m_nh.param("scale_z_l", m_scale_left.z, 0.1);

    m_nh.param("scale_x_r", m_scale_right.x, 0.1);
    m_nh.param("scale_y_r", m_scale_right.y, 0.1);
    m_nh.param("scale_z_r", m_scale_right.z, 0.1);

    m_nh.param("scale_x_o", m_scale_others.x, 0.1);
    m_nh.param("scale_y_o", m_scale_others.y, 0.1);
    m_nh.param("scale_z_o", m_scale_others.z, 0.1);

    m_nh.param("scale_x_wp", m_waypoint_marker.scale.x, 0.3);
    m_nh.param("scale_y_wp", m_waypoint_marker.scale.y, 0.3);
    m_nh.param("scale_z_wp", m_waypoint_marker.scale.z, 0.3);

    int buf[3];
    double alpha;   

    m_nh.param("color_r_l", buf[0], 0);
    m_nh.param("color_g_l", buf[1], 0);
    m_nh.param("color_b_l", buf[2], 255);
    m_nh.param("color_a_l", alpha, 1.0);
    m_color_left.r = (double)buf[0]/255.0; m_color_left.g = (double)buf[1]/255.0;
    m_color_left.b = (double)buf[2]/255.0; m_color_left.a = alpha;
    
    m_nh.param("color_r_r", buf[0], 0);
    m_nh.param("color_g_r", buf[1], 255);
    m_nh.param("color_b_r", buf[2], 0);
    m_nh.param("color_a_r", alpha, 1.0);
    m_color_right.r = (double)buf[0]/255.0; m_color_right.g = (double)buf[1]/255.0;
    m_color_right.b = (double)buf[2]/255.0; m_color_right.a = alpha;

    m_nh.param("color_r_o", buf[0], 255);
    m_nh.param("color_g_o", buf[1], 0);
    m_nh.param("color_b_o", buf[2], 0);
    m_nh.param("color_a_o", alpha, 1.0);
    m_color_others.r = (double)buf[0]/255.0; m_color_others.g = (double)buf[1]/255.0;
    m_color_others.b = (double)buf[2]/255.0; m_color_others.a = alpha;

    m_nh.param("color_r_wp", buf[0], 255);
    m_nh.param("color_g_wp", buf[1], 255);
    m_nh.param("color_b_wp", buf[2], 0);
    m_nh.param("color_a_wp", alpha, 1.0);
    m_waypoint_marker.color.r = (double)buf[0]/255.0; m_waypoint_marker.color.g = (double)buf[1]/255.0;
    m_waypoint_marker.color.b = (double)buf[2]/255.0; m_waypoint_marker.color.a = alpha;

    m_line_marker.pose.orientation.x = 0.0;
    m_line_marker.pose.orientation.y = 0.0;
    m_line_marker.pose.orientation.z = 0.0;
    m_line_marker.pose.orientation.w = 1.0;
    m_line_marker.ns = "lane_lines";
    m_line_marker.action = visualization_msgs::Marker::ADD;
    m_line_marker.lifetime = ros::Duration(0.2);
    m_waypoint_marker.frame_locked = true;

    m_waypoint_marker.pose.orientation = m_line_marker.pose.orientation;
    m_waypoint_marker.ns = "waypoint";
    m_waypoint_marker.action = visualization_msgs::Marker::ADD;
    m_waypoint_marker.type = visualization_msgs::Marker::SPHERE;
    m_waypoint_marker.lifetime = ros::Duration(0.2);
    m_waypoint_marker.frame_locked = true;

    m_param_update.data = true;
    m_init = true;

}

// Main loop of the node
void LaneVisualizer::run() {

    m_loop_rate = ros::Rate(m_frequency);

    // Trigger parameter update initialy
    m_param_update_pub.publish(m_param_update);

    ROS_INFO("***[lane_visualizer]: Node is Running.");

    while (ros::ok()) {
        ros::spinOnce();
        if (m_lane_lines_set) {
            if (fillMarkerArray()) {
                m_lanes_rviz_pub.publish(m_markers);
                m_lane_lines_set = false;
            }
        }
        if (m_current_waypoint_set) {
            visualizeWaypoint();
            m_waypoint_rviz_pub.publish(m_waypoint_marker);
            m_current_waypoint_set = false;
        }
        m_loop_rate.sleep();
    }
}

void LaneVisualizer::visualizeWaypoint() {

    m_waypoint_marker.header = m_current_waypoint.header;
    m_waypoint_marker.pose.position = m_current_waypoint.point;

}

bool LaneVisualizer::fillMarkerArray() {

    int id = 2;
    bool success = true;
    m_markers.markers.clear();
    m_line_marker.header = m_lane_lines.header;
    for (int i = 0; i < m_lane_lines.boundaries.size(); i++) {
        // Line position
        if (m_lane_lines.boundaries.at(i).line_pos == hska_msgs::LaneBoundary::DRIVING_LEFT) {
            m_line_marker.color = m_color_left;
            m_line_marker.scale = m_scale_left;
            m_line_marker.id = 0;
        }
        else if (m_lane_lines.boundaries.at(i).line_pos == hska_msgs::LaneBoundary::DRIVING_RIGHT) {
            m_line_marker.color = m_color_right;
            m_line_marker.scale = m_scale_right;
            m_line_marker.id = 1;
        }        
        else {
            m_line_marker.color = m_color_others;
            m_line_marker.scale = m_scale_others;
            m_line_marker.id = id;
            id++;
        }
        // Line type
        if (m_lane_lines.boundaries.at(i).style == hska_msgs::LaneBoundary::DASHED) {
            m_line_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        }
        else {
            m_line_marker.scale.y = 0;
            m_line_marker.scale.z = 0;
            m_line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        }
        // Get line points
        geometry_msgs::Point pt;
        pt.z = 0.0;
        for (int j = m_lane_lines.img_height; j > 0; j -= 30) {
            pt.x = m_lane_lines.min_look_ahead + (m_lane_lines.img_height - j) /  m_lane_lines.pixel_to_m;
            pt.y = 0.0;
            for (int a = 0; a < m_lane_lines.boundaries.at(i).polynom_coeffs.size(); a++)
                pt.y += m_lane_lines.boundaries.at(i).polynom_coeffs.at(a)*pow(j,a);
            pt.y = (m_lane_lines.img_width/2 - pt.y) / m_lane_lines.pixel_to_m;
            if (isnan(pt.x) || isnan(pt.y)) {
                break;
                success = false;
            }
            if (!success) break;
            m_line_marker.points.push_back(pt);
        }
        if (m_line_marker.points.size() < 2) {
            m_line_marker.points.clear();
        }
        else {
            m_markers.markers.push_back(m_line_marker);
            m_line_marker.points.clear();
        }
    }
    return success;

}
    
// Callback of lane lines
void LaneVisualizer::callbackCurrentLaneLines(const hska_msgs::LaneBoundaryArrayConstPtr& msg) {

    m_lane_lines = *msg;
    m_lane_lines_set = true;
	
}

// Callback of current waypoint
void LaneVisualizer::callbackCurrentWaypoint(const geometry_msgs::PointStampedConstPtr& msg) {

    m_current_waypoint = *msg;
    m_current_waypoint_set = true;

}

// Callback of reconfigurable parameters
void LaneVisualizer::callbackParameters(lane_detection::LaneVisualizerConfig &config, uint32_t level) {

    if (m_init)
        m_param_update_pub.publish(m_param_update);

    m_frequency = config.frequency;
    m_loop_rate = ros::Rate(m_frequency);

    m_scale_left.x = config.scale_x_l;
    m_scale_left.y = config.scale_y_l;
    m_scale_left.z = config.scale_z_l;
    m_color_left.r = (double)config.color_r_l / 255.0;
    m_color_left.g = (double)config.color_g_l / 255.0;
    m_color_left.b = (double)config.color_b_l / 255.0;
    m_color_left.a = config.color_a_l;

    m_scale_right.x = config.scale_x_r;
    m_scale_right.y = config.scale_y_r;
    m_scale_right.z = config.scale_z_r;
    m_color_right.r = (double)config.color_r_r / 255.0;
    m_color_right.g = (double)config.color_g_r / 255.0;
    m_color_right.b = (double)config.color_b_r / 255.0;
    m_color_right.a = config.color_a_r;

    m_scale_others.x = config.scale_x_o;
    m_scale_others.y = config.scale_y_o;
    m_scale_others.z = config.scale_z_o;
    m_color_others.r = (double)config.color_r_o / 255.0;
    m_color_others.g = (double)config.color_g_o / 255.0;
    m_color_others.b = (double)config.color_b_o / 255.0;
    m_color_others.a = config.color_a_o;

}
