// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#ifndef LANE_CHANGER_H
#define LANE_CHANGER_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

// HSKA messages
#include <hska_msgs/SteeringCommand.h>
#include <hska_msgs/LaneBoundaryArray.h>
#include <hska_msgs/ADASCommand.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <lane_changing/LaneChangerConfig.h>

class LaneChanger {

    public:

        LaneChanger();
        ~LaneChanger(); 

        // Main loop of node
        void run();

    private:

        // ROS specific
        ros::NodeHandle m_nh;
        ros::Rate m_loop_rate;
        ros::Publisher m_steer_pub;
        ros::Subscriber m_steer_raw_sub;
        ros::Subscriber m_next_wp_sub;
        ros::Subscriber m_adas_cmd_sub;
        ros::Subscriber m_lane_lines_sub;

        // Messages
        hska_msgs::SteeringCommand m_steer;
        hska_msgs::ADASCommand m_adas_cmd;
        hska_msgs::LaneBoundaryArray m_lane_lines;
        geometry_msgs::PointStamped m_next_wp;
		
        // Member variables;	
       	bool m_next_wp_set;
        bool m_steer_raw_set;
        bool m_lane_lines_set;
        bool m_adas_cmd_set;
       	double m_frequency;
        double m_steer_change;
        bool m_changing;
        bool m_change_to_right;
        bool m_change_to_left;
        bool m_display_information;

        // Dynamic reconfigure
        dynamic_reconfigure::Server<lane_changing::LaneChangerConfig> server;
        dynamic_reconfigure::Server<lane_changing::LaneChangerConfig>::CallbackType f;

        // Member Functions
        void init();
        // Calculate the steering offset for the lane change
        void calcSteerForLaneChange();
        // Track the calculated waypoint
        void trackWaypoint();
        // Check the lane line style
        void checkLaneChangePermission();

        // Callbacks
        void callbackCurrentSteerCmd(const hska_msgs::SteeringCommandConstPtr& msg);
        void callbackCurrentWaypoint(const geometry_msgs::PointStampedConstPtr& msg);
        void callbackADASCommand(const hska_msgs::ADASCommandConstPtr& msg);
        void callbackLaneLines(const hska_msgs::LaneBoundaryArrayConstPtr& msg);
        void callbackParameters(lane_changing::LaneChangerConfig &config, uint32_t level);

};

#endif
