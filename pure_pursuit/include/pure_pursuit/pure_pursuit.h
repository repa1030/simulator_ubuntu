// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

// HSKA messages
#include <hska_msgs/SteeringCommand.h>
#include <hska_msgs/ADASCommand.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <pure_pursuit/PurePursuitConfig.h>

// C++
#include <math.h>

class PurePursuit {

    public:

        PurePursuit();
        ~PurePursuit(); 

        // Main loop of node
        void run();

    private:

        // Constants
        const double MAX_RADIUS;

        // ROS specific
        ros::NodeHandle m_nh;
        ros::Rate m_loop_rate;
        ros::Publisher m_steer_pub;
        ros::Subscriber m_next_wp_sub;
        ros::Subscriber m_adas_cmd_sub;

        // Messages
        hska_msgs::SteeringCommand m_steer;
        hska_msgs::ADASCommand m_adas_cmd;
        geometry_msgs::PointStamped m_next_wp;
		
        // Member variables;	
       	bool m_next_wp_set;
        bool m_display_information;
       	double m_frequency;
       	double m_wheel_base;

        // Dynamic reconfigure
        dynamic_reconfigure::Server<pure_pursuit::PurePursuitConfig> server;
        dynamic_reconfigure::Server<pure_pursuit::PurePursuitConfig>::CallbackType f;

        // Member Functions
        void init();
        void calcSteeringAngle();  
        double calcCurvature(double dist, double y);

        // Callbacks
        void callbackCurrentWaypoint(const geometry_msgs::PointStampedConstPtr& msg);
        void callbackADASCmd(const hska_msgs::ADASCommandConstPtr& msg);
        void callbackParameters(pure_pursuit::PurePursuitConfig &config, uint32_t level);

};

#endif
