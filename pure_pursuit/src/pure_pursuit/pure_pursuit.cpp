// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#include "pure_pursuit/pure_pursuit.h"

// Constructor
PurePursuit::PurePursuit() : 
  MAX_RADIUS(9e10),
  m_loop_rate(30.0),
  m_next_wp_set(false) {

    init();

}

// Destructor
PurePursuit::~PurePursuit() {
}

// Initialization
void PurePursuit::init() {

    ROS_INFO("***[pure_pursuit]: Node Initializing.");

    // Set ADAS command status for lkas to false
    m_adas_cmd.lkas = false;

    // Dynamic reconfigure
    f = boost::bind(&PurePursuit::callbackParameters, this, _1, _2);
    server.setCallback(f);

    // Publisher
    m_steer_pub = m_nh.advertise<hska_msgs::SteeringCommand>("/ctrl_cmd/steering_angle", 1);

    // Subscriber
    m_next_wp_sub = m_nh.subscribe("/lanes/next_waypoint", 1, &PurePursuit::callbackCurrentWaypoint, this);
    m_adas_cmd_sub = m_nh.subscribe("/adas_cmd", 1, &PurePursuit::callbackADASCmd, this);

    // Parameters from parameter server
    m_nh.param("frequency", m_frequency, 30.0);
    m_nh.param("display_information", m_display_information, false);
    m_nh.param("wheel_base", m_wheel_base, 2.69);

}

// Main loop of the node
void PurePursuit::run() {

    m_loop_rate = ros::Rate(m_frequency);

    ROS_INFO("***[pure_pursuit]: Node is Running.");

    while(ros::ok()) {
        ros::spinOnce();
      	if(m_next_wp_set && m_adas_cmd.lkas) {
            calcSteeringAngle();
            m_steer_pub.publish(m_steer);
            m_next_wp_set = false;
            if(m_display_information) {
                ROS_INFO_THROTTLE(5, "***[pure_pursuit]: Current Steering Angle: %.2f rad", m_steer.steering_angle);       
            }
        }
        m_loop_rate.sleep();
    }
	
}

// Calculation of steering angle
void PurePursuit::calcSteeringAngle() {

    double dst;
    double kappa = 0.0;

    // Calculating distance to waypoint
    dst = sqrt(m_next_wp.point.x * m_next_wp.point.x + 
               m_next_wp.point.y * m_next_wp.point.y);

    // Calculating curvature to reach waypoint
    kappa = calcCurvature(dst, m_next_wp.point.y);

    // Converting curvature into steering angle in radian
    m_steer.steering_angle = -atan(kappa*m_wheel_base);
    m_steer.active = true;

}

// Calculation of the curvature with the Pure Pursuit algorithm
double PurePursuit::calcCurvature(double dist, double y) {

    double curvature = 0;
    // No steering if offset == 0 or distance == 0
    if(y == 0 || dist == 0) {
        curvature = (1/MAX_RADIUS);
    }
    // Calculating the required curvature with the 
    // Pure Pursuit algorithm
    else {
        curvature = 2*y/(dist*dist);
        // Apply maximum radius if curvature is very small
        if((abs(curvature) < (1/MAX_RADIUS))) {
            curvature = 1/MAX_RADIUS;
        }
    }
    return curvature;

}

// Callback of current waypoint
void PurePursuit::callbackCurrentWaypoint (const geometry_msgs::PointStampedConstPtr& msg) {

    m_next_wp = *msg;
    m_next_wp_set = true;
	
}

// Callback of ADAS command status
void PurePursuit::callbackADASCmd (const hska_msgs::ADASCommandConstPtr& msg) {

    m_adas_cmd = *msg;
	
}

// Callback of reconfigurable parameters
void PurePursuit::callbackParameters(pure_pursuit::PurePursuitConfig &config, uint32_t level) {

    m_loop_rate = ros::Rate(config.frequency);
    m_wheel_base = config.wheel_base;
    m_frequency = config.frequency;
    m_display_information = config.display_information;
	
}
