// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#include "lane_changer/lane_changer.h"

// Constructor
LaneChanger::LaneChanger() : 
  m_loop_rate(30.0),
  m_next_wp_set(false),
  m_steer_raw_set(false),
  m_lane_lines_set(false),
  m_adas_cmd_set(false),
  m_change_to_right(false),
  m_change_to_left(false),
  m_changing(false) {

    init();

}

// Destructor
LaneChanger::~LaneChanger() {
}

// Initialization
void LaneChanger::init() {

    ROS_INFO("***[lane_changer]: Node Initializing.");

    m_adas_cmd.lkas = false;

    // Dynamic reconfigure
    f = boost::bind(&LaneChanger::callbackParameters, this, _1, _2);
    server.setCallback(f);

    // Publisher
    m_steer_pub = m_nh.advertise<hska_msgs::SteeringCommand>("/ctrl_cmd/steering_angle", 1);

    // Subscriber
    m_steer_raw_sub = m_nh.subscribe("/pure_pursuit/steering_raw", 1, &LaneChanger::callbackCurrentSteerCmd, this);
    m_next_wp_sub = m_nh.subscribe("/lanes/next_waypoint", 1, &LaneChanger::callbackCurrentWaypoint, this);    
    m_adas_cmd_sub = m_nh.subscribe("/adas_cmd", 1, &LaneChanger::callbackADASCommand, this);
    m_lane_lines_sub = m_nh.subscribe("/lanes/lines", 1, &LaneChanger::callbackLaneLines, this);

    // Parameters from parameter server
    m_nh.param("frequency", m_frequency, 30.0);
    m_nh.param("display_information", m_display_information, true);
    m_nh.param("steer_change", m_steer_change, 0.5);

}

// Main loop of the node
void LaneChanger::run() {

    m_loop_rate = ros::Rate(m_frequency);

    ROS_INFO("***[lane_changer]: Node is Running.");

    while(ros::ok()) {
        ros::spinOnce();
        if(m_adas_cmd_set && m_lane_lines_set && m_adas_cmd.lkas && !m_changing) {
            checkLaneChangePermission();
            m_lane_lines_set = false;
            m_adas_cmd_set = false;
        }
        // Check if adas is active and waypoint and steer is set
      	if(m_adas_cmd.lkas && m_next_wp_set && m_steer_raw_set) {
            // Calculate the new steering angle
            calcSteerForLaneChange();
            // Track the waypoint to see when lane change is done
            trackWaypoint();
            // Publish steer and reset check variables
            m_steer_pub.publish(m_steer);
            m_next_wp_set = false;
            m_steer_raw_set = false;
            m_steer.reset_blinker_state = false;
        }
        m_loop_rate.sleep();
    }
	
}

// Calculation of steering angle
void LaneChanger::calcSteerForLaneChange() {

    static double steer;
    
    // Add the steer offset to the current steering angle
    // Make sure to do this only once in the lane change
    if(m_change_to_right && !m_changing) {
        m_changing = true;
        steer = m_steer.steering_angle + m_steer_change*M_PI/180.0;
    }
    else if(m_change_to_left && !m_changing) {
        m_changing = true;
        steer = m_steer.steering_angle - m_steer_change*M_PI/180.0;
    }
    else if((!m_change_to_left && !m_change_to_right)) {
        m_changing = false;
        steer = m_steer.steering_angle;    
    }

    m_steer.steering_angle = steer;

}

// Tracking position of waypoint
void LaneChanger::trackWaypoint() {

    static geometry_msgs::PointStamped wp;
    static bool is_saved = false;

    // Save the waypoint before starting lane change
    if((m_change_to_right || m_change_to_left) && !is_saved) {
        wp = m_next_wp;
        is_saved = true;
    }
    else if(!m_change_to_right && !m_change_to_left) {
        is_saved = false;
    }

    // Check if the waypoint is on the new lane
    // If so then the lane change is complete
    if(m_change_to_right && (wp.point.y > m_next_wp.point.y + 0.1)) {
        if(m_display_information)
            ROS_INFO("***[lane_changer]: Vehicle Arrived in New Lane.");
        // Reset the current blinker state
        m_steer.reset_blinker_state = true;
        // Lane change is done
        m_change_to_right = false;
        is_saved = false;
    }
    else if(m_change_to_left && (wp.point.y < m_next_wp.point.y - 0.1)) {
        if(m_display_information)
            ROS_INFO("***[lane_changer]: Vehicle Arrived in New Lane.");
        // Reset the current blinker state
        m_steer.reset_blinker_state = true;
        // Lane change is done
        m_change_to_left = false;
        is_saved = false;
    }

}

// Check permission of lane changing
void LaneChanger::checkLaneChangePermission() {

    // Initialize the lane lines with type UNKNOWN
    unsigned char right = hska_msgs::LaneBoundary::UNKNOWN;
    unsigned char left = hska_msgs::LaneBoundary::UNKNOWN;

    // Return if there is no lane change request
    if(m_adas_cmd.lane_change_req == hska_msgs::ADASCommand::NONE)
        return;

    // Get types of driving lane lines
    for(int i = 0; i < m_lane_lines.boundaries.size(); i++) {
        if(m_lane_lines.boundaries[i].line_pos == hska_msgs::LaneBoundary::DRIVING_LEFT) {
            left = m_lane_lines.boundaries[i].style;
        }
        else if(m_lane_lines.boundaries[i].line_pos == hska_msgs::LaneBoundary::DRIVING_RIGHT) {
            right = m_lane_lines.boundaries[i].style;
        }
    }

    // Check if right lane line is dashed (if requested right lane change)
    if(m_adas_cmd.lane_change_req == hska_msgs::ADASCommand::RIGHT 
       && right == hska_msgs::LaneBoundary::DASHED) {
        if(m_display_information)
            ROS_INFO("***[lane_changer]: Starting Lane Change to Right.");
        m_change_to_right = true;
    }
    // Check if left lane line is dashed (if requested left lane change)
    else if(m_adas_cmd.lane_change_req == hska_msgs::ADASCommand::LEFT 
            && left == hska_msgs::LaneBoundary::DASHED) {
        if(m_display_information)
            ROS_INFO("***[lane_changer]: Starting Lane Change to Left.");
        m_change_to_left = true;
    }
    // If the line is solid, no lane change is permitted (throw warning)
    else {
        if(m_adas_cmd.lane_change_req == hska_msgs::ADASCommand::RIGHT && m_display_information)
            ROS_WARN_THROTTLE(3, "***[lane_changer]: Lane Changing to Right is Currently not Permitted.");
        else if(m_adas_cmd.lane_change_req == hska_msgs::ADASCommand::LEFT && m_display_information)
            ROS_WARN_THROTTLE(3, "***[lane_changer]: Lane Changing to Left is Currently not Permitted.");
    }

}

// Callback of current raw steering command
void LaneChanger::callbackCurrentSteerCmd(const hska_msgs::SteeringCommandConstPtr& msg) {

    m_steer.header = msg->header;
    m_steer.steering_angle = msg->steering_angle;
    m_steer.active = msg->active;
    m_steer_raw_set = true;
	
}

// Callback of current waypoint
void LaneChanger::callbackCurrentWaypoint(const geometry_msgs::PointStampedConstPtr& msg) {

    m_next_wp = *msg;
    m_next_wp_set = true;
	
}

// Callback of ADAS command status
void LaneChanger::callbackADASCommand(const hska_msgs::ADASCommandConstPtr& msg) {

    m_adas_cmd = *msg;
    m_adas_cmd_set = true;
	
}

// Callback of lane lines
void LaneChanger::callbackLaneLines(const hska_msgs::LaneBoundaryArrayConstPtr& msg) {

    m_lane_lines = *msg;
    m_lane_lines_set = true;
	
}

// Callback of reconfigurable parameters
void LaneChanger::callbackParameters(lane_changing::LaneChangerConfig &config, uint32_t level) {

    m_frequency = config.frequency;
    m_loop_rate = ros::Rate(m_frequency);
    m_display_information = config.display_information;
    m_steer_change = config.steer_change;
	
}
