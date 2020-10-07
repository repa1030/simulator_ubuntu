// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#include "pure_pursuit_visualizer/pure_pursuit_visualizer.h"

// Constructor
PurePursuitVisualizer::PurePursuitVisualizer() : 
  m_loop_rate(30.0),
  m_next_wp_set(false) {

    init();

}

// Destructor
PurePursuitVisualizer::~PurePursuitVisualizer() {
}

// Initialization
void PurePursuitVisualizer::init() {

    ROS_INFO("***[pure_pursuit_visualizer]: Node Initializing.");

    double color[4];
    double scale;

    // Dynamic reconfigure
    f = boost::bind(&PurePursuitVisualizer::callbackParameters, this, _1, _2);
    server.setCallback(f);

    // Publisher 
    m_traj_circle_pub = m_nh.advertise<visualization_msgs::Marker>("/visualization/pure_pursuit", 1);

    // Subscriber
    m_next_wp_sub = m_nh.subscribe("/lanes/next_waypoint", 1, &PurePursuitVisualizer::callbackCurrentWaypoint, this);

    // Parameters from parameter server
    m_nh.param("frequency", m_frequency, 30.0);

    m_nh.param("color_r", color[0], 1.0);
    m_nh.param("color_g", color[1], 1.0);
    m_nh.param("color_b", color[2], 1.0);
    m_nh.param("color_a", color[3], 1.0);
    
    m_nh.param("scale", scale, 0.1);

    // Setting up visualization markers
    m_traj_circle_mark.header.frame_id = "base_link";
    m_traj_circle_mark.ns = "trajectory_marker";
    m_traj_circle_mark.id = 0;
    m_traj_circle_mark.pose.orientation.x = 0.0;
    m_traj_circle_mark.pose.orientation.y = 0.0;
    m_traj_circle_mark.pose.orientation.z = 0.0;
    m_traj_circle_mark.pose.orientation.w = 1.0;
    m_traj_circle_mark.type = visualization_msgs::Marker::LINE_STRIP;
    m_traj_circle_mark.action = visualization_msgs::Marker::ADD;
    m_traj_circle_mark.scale.x = scale;
    m_traj_circle_mark.color.r = color[0];
    m_traj_circle_mark.color.g = color[1];
    m_traj_circle_mark.color.b = color[2];
    m_traj_circle_mark.color.a = color[3];
    m_traj_circle_mark.frame_locked = true;

}

// Main loop of the node
void PurePursuitVisualizer::run() {

    m_loop_rate = ros::Rate(m_frequency);

    ROS_INFO("***[pure_pursuit_visualizer]: Node is Running.");

    while(ros::ok()) {
        ros::spinOnce();
      	if(m_next_wp_set) {
            calcRadius();
            generateTrajectoryCircle();
            displayTrajectoryCircle();
            m_traj_circle_pub.publish(m_traj_circle_mark);
            m_traj_circle_mark.points.clear();
            m_traj_circle_mark.colors.clear();
            m_traj_circle_arr.clear();
            m_next_wp_set = false;
        }
        m_loop_rate.sleep();
    }
	
}

// Calculate Pure Pursuit Radius
void PurePursuitVisualizer::calcRadius() {

    double dist = sqrt(m_next_wp.point.x * m_next_wp.point.x + 
                        m_next_wp.point.y * m_next_wp.point.y);
    double denominator = 2 * m_next_wp.point.y;
    double numerator = pow(dist, 2);

    if(denominator != 0)
        m_radius = numerator / denominator;
    else
        m_radius = 0;

}


// Generate the locus of pure pursuit
void PurePursuitVisualizer::generateTrajectoryCircle() {

    double range = M_PI / 8;
    double increment = 0.01;

    for(double i = 0; i < range; i += increment) {
        // calc a point of circumference
        geometry_msgs::Point p;
        p.x = m_radius * cos(i);
        p.y = m_radius * sin(i);

        // transform to (radius,0)
        geometry_msgs::Point relative_p;
        relative_p.x = p.x - m_radius;
        relative_p.y = p.y;

        // rotate -90°
        geometry_msgs::Point rotate_p = rotatePoint(relative_p, -90);

        m_traj_circle_arr.push_back(rotate_p);
    }

    // reverse vector
    std::reverse(m_traj_circle_arr.begin(), m_traj_circle_arr.end());

    for(double i = 0; i > (-1) * range; i -= increment) {
        // calc a point of circumference
        geometry_msgs::Point p;
        p.x = m_radius * cos(i);
        p.y = m_radius * sin(i);

        // transform to (radius,0)
        geometry_msgs::Point relative_p;
        relative_p.x = p.x - m_radius;
        relative_p.y = p.y;

        // rotate -90°
        geometry_msgs::Point rotate_p = rotatePoint(relative_p, -90);

        m_traj_circle_arr.push_back(rotate_p);
    }

}

// Display the locus of pure pursuit by markers.
void PurePursuitVisualizer::displayTrajectoryCircle() {

    m_traj_circle_mark.header.stamp = m_next_wp.header.stamp;

    std_msgs::ColorRGBA color;
    color.a = m_traj_circle_mark.color.a;
    color.b = m_traj_circle_mark.color.b;
    color.r = m_traj_circle_mark.color.g;
    color.g = m_traj_circle_mark.color.r;

    for(auto el : m_traj_circle_arr) {
        for(std::vector<geometry_msgs::Point>::iterator it = m_traj_circle_arr.begin(); 
            it != m_traj_circle_arr.end();
            it++) {

            m_traj_circle_mark.points.push_back(el);
            m_traj_circle_mark.colors.push_back(color);

        }
    }

}

// Rotate Point
geometry_msgs::Point PurePursuitVisualizer::rotatePoint(geometry_msgs::Point point, double degree) {

    geometry_msgs::Point rotate;
    rotate.x = cos(deg2rad(degree)) * point.x - sin(deg2rad(degree)) * point.y;
    rotate.y = sin(deg2rad(degree)) * point.x + cos(deg2rad(degree)) * point.y;

    return rotate;

}

// Callback of current waypoint
void PurePursuitVisualizer::callbackCurrentWaypoint (const geometry_msgs::PointStampedConstPtr& msg) {

    m_next_wp = *msg;
    m_next_wp_set = true;
	
}

// Callback of reconfigurable parameters
void PurePursuitVisualizer::callbackParameters(pure_pursuit::PurePursuitVisualizerConfig &config, uint32_t level) {

    m_loop_rate = ros::Rate(config.frequency);
    m_frequency = config.frequency;

    m_traj_circle_mark.color.r = config.color_r;
    m_traj_circle_mark.color.g = config.color_g;
    m_traj_circle_mark.color.b = config.color_b;
    m_traj_circle_mark.color.a = config.color_a;
	
    m_traj_circle_mark.scale.x = config.scale;

}
