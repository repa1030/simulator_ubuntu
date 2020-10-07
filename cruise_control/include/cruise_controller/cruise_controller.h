// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#ifndef CRUISE_CONTROL_H
#define CRUISE_CONTROL_H

// Maths
#include <cmath>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>

//HSKA messages
#include <hska_msgs/PedalCommand.h>
#include <hska_msgs/ADASCommand.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <cruise_control/CruiseControllerConfig.h>


class CruiseController {

    public:
        CruiseController();
        ~CruiseController();

        //Main loop of node
        void run();

    private:

        // ROS specific
        ros::NodeHandle m_nh;
        ros::Rate m_loop_rate;
        ros::Publisher m_pedal_pub;
        ros::Subscriber m_current_velocity_sub;
        ros::Subscriber m_adas_cmd_sub;
        ros::Subscriber m_waypoint_sub;

        // Messages
        hska_msgs::PedalCommand m_pedal;
        hska_msgs::ADASCommand m_adas_cmd;
        geometry_msgs::PointStamped m_waypoint;

        //Member variables
        double m_frequency;
        bool m_adaptive;
        bool m_waypoint_set;
        bool m_current_velocity_set;
        bool m_ctrl_gas_enable;
        bool m_ctrl_brake_enable;
        double m_max_accel;
        double m_delta_accel;
        double m_delta_velocity; 
        double m_max_force;
        double m_target_velocity; // m/s!!
        double m_current_velocity;
        // Controller variables
        double m_ctrl_diff;
        double m_ctrl_diff_old_gas;
        double m_ctrl_diff_old_2_gas;
        double m_ctrl_diff_old_brake;
        double m_ctrl_diff_old_2_brake;
        double m_dt;
        double m_integral_gas;
        double m_integral_brake;
        double m_cc_gas_Kp;
        double m_cc_gas_Ki;
        double m_cc_gas_Kd;
        double m_cc_brake_Kp;
        double m_cc_brake_Ki;
        double m_cc_brake_Kd;
        double m_max;
        double m_min;
        
        // Dynamic reconfigure
        dynamic_reconfigure::Server<cruise_control::CruiseControllerConfig> server;
        dynamic_reconfigure::Server<cruise_control::CruiseControllerConfig>::CallbackType f;

        //Member functions
        void init();
        void calcPedalPosition();
        void hysteresis();
        void setTargetVelocity();
        double ctrlGaspedal();
        double ctrlBreakpedal();

        //Callbacks
        void callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg);
        void callbackWaypoint(const geometry_msgs::PointStampedConstPtr& msg);
        void callbackADASCmd(const hska_msgs::ADASCommandConstPtr& msg);
        void callbackParameters(cruise_control::CruiseControllerConfig &config, uint32_t level);
        
};

#endif
