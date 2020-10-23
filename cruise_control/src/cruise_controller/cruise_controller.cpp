// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#include "cruise_controller/cruise_controller.h"

// Constructor
CruiseController::CruiseController():
  m_loop_rate(30.0),
  m_ctrl_gas_enable(false),
  m_ctrl_brake_enable(false),
  m_adaptive(false),
  m_waypoint_set(false),
  m_current_velocity_set(false),
  m_current_velocity(0.0),
  m_ctrl_diff(0.0),
  m_ctrl_diff_old_gas(0.0),
  m_ctrl_diff_old_2_gas(0.0),
  m_ctrl_diff_old_brake(0.0),
  m_ctrl_diff_old_2_brake(0.0),
  m_dt(0.0),
  m_integral_gas(0.0),
  m_integral_brake(0.0),
  m_max(100.0),
  m_min(0.0) {

    init();
}

// Destructor
CruiseController::~CruiseController() {
}

// Intitialization
void CruiseController::init() {
   
    ROS_INFO("***[cruise_controller]: Node Initializing.");

    // Dynamic reconfigure
    f = boost::bind(&CruiseController::callbackParameters, this, _1, _2);
    server.setCallback(f);

    // Publisher
    m_pedal_pub = m_nh.advertise<hska_msgs::PedalCommand>("/ctrl_cmd/speed_pedals", 1);
    
    // Subscriber
    m_current_velocity_sub = m_nh.subscribe("/vehicle/current_velocity", 1, &CruiseController::callbackCurrentVelocity, this);
    m_adas_cmd_sub = m_nh.subscribe("/adas_cmd", 1, &CruiseController::callbackADASCmd, this);
    m_waypoint_sub = m_nh.subscribe("/lanes/next_waypoint", 1, &CruiseController::callbackWaypoint, this);

    // Parameters from parameter server
    m_nh.param("frequency", m_frequency, 30.0);
    m_nh.param("adaptive_velocity", m_adaptive, false);
    m_nh.param("max_acceleration", m_max_accel, 5.0);
    m_nh.param("delta_acceleration", m_delta_accel, 0.5);
    m_nh.param("delta_velocity", m_delta_velocity, 2.0); // for hysteresis
    m_nh.param("cc_gas_Kp", m_cc_gas_Kp, 69.75);
    m_nh.param("cc_gas_Ki", m_cc_gas_Ki, 11.24);
    m_nh.param("cc_gas_Kd", m_cc_gas_Kd, 0.0);
    m_nh.param("cc_brake_Kp", m_cc_brake_Kp, 10.0);
    m_nh.param("cc_brake_Ki", m_cc_brake_Ki, 0.0);
    m_nh.param("cc_brake_Kd", m_cc_brake_Kd, 0.0);

}

// Main loop of the node
void CruiseController::run() {

    double time_begin = 0.0;
    double time_now = 0.0;
    bool begin_flag = true;
    
    m_loop_rate = ros::Rate(m_frequency);
    ROS_INFO("***[cruise_controller]: Node is Running.");

    while(ros::ok()) {
        ros::spinOnce();
      	if(m_current_velocity_set && m_adas_cmd.cc) {
            // Set target velocity
            setTargetVelocity();
            // Call of functions
            hysteresis();
            // Calc dt
            if(begin_flag) { 
                time_begin = ros::Time::now().toSec();
                m_dt = 0.0;
                begin_flag= false;
            }
            else {
                time_now = ros::Time::now().toSec();
                m_dt = time_now - time_begin;
                time_begin = time_now;
            }
            // Calc pedal pos
            calcPedalPosition();
            // Publishing of m_pedal
            m_pedal.header.stamp = ros::Time::now();
            m_pedal_pub.publish(m_pedal);
            m_current_velocity_set = false;
        }
        // Reset controllers
        else if(!m_adas_cmd.cc) {
            m_ctrl_diff= 0.0;
            m_ctrl_diff_old_gas = 0.0;
            m_ctrl_diff_old_2_gas = 0.0;
            m_ctrl_diff_old_brake = 0.0;
            m_ctrl_diff_old_2_brake = 0.0;
        }
        m_loop_rate.sleep();
        
    }
	
}

// Calculate hysteresis for activating of the different PID-Controller
void CruiseController::hysteresis() {
    
    if((m_current_velocity + m_delta_velocity)< m_target_velocity) {
        m_ctrl_gas_enable = true;
        m_ctrl_brake_enable = false;
    }
    else if((m_current_velocity - m_delta_velocity)> m_target_velocity) {
        m_ctrl_brake_enable = true;
        m_ctrl_gas_enable = false;
    }

}

// Calculate break- and gaspedal position
void CruiseController::calcPedalPosition() {
        
   if(m_ctrl_gas_enable) {
        m_ctrl_diff = m_target_velocity - m_current_velocity;
        m_pedal.gas_pedal = ctrlGaspedal();
        m_pedal.brake_pedal = 0.0;
        m_pedal.active = true;
        m_ctrl_diff_old_brake = 0.0;
        m_ctrl_diff_old_2_brake = 0.0;
   }
   else if(m_ctrl_brake_enable) {
        m_ctrl_diff = m_current_velocity - m_target_velocity;
        m_pedal.brake_pedal = ctrlBreakpedal();
        m_pedal.gas_pedal = 0.0;
        m_pedal.active = true;
        m_ctrl_diff_old_gas = 0.0;
        m_ctrl_diff_old_2_gas = 0.0;
    }
    
}

// PID-Contoller for Gaspedal
double CruiseController::ctrlGaspedal() {

    double C_0 = m_cc_gas_Kp + m_cc_gas_Ki * m_dt + m_cc_gas_Kd * (1/m_dt);
    double C_1 = m_cc_gas_Kp + 2 * m_cc_gas_Kd * (1/m_dt);
    double C_2 = m_cc_gas_Kd * (1/m_dt);

    double output_gas = (C_0 * m_ctrl_diff) + (C_1 * m_ctrl_diff_old_gas) + (C_2 * m_ctrl_diff_old_2_gas);

    m_ctrl_diff_old_2_gas = m_ctrl_diff_old_gas;
    m_ctrl_diff_old_gas = m_ctrl_diff;
    
    // Restrict to max/min
    if(output_gas > m_max){
        output_gas = m_max;
    }
    else if(output_gas < m_min){
        output_gas = m_min;
    }

    return (output_gas/100);

}

// PID-Controller for Brakepedal
double CruiseController::ctrlBreakpedal() {

    double C_0 = m_cc_brake_Kp + m_cc_brake_Ki* m_dt + m_cc_brake_Kd * (1/m_dt);
    double C_1 = m_cc_brake_Kp + 2 * m_cc_brake_Kd * (1/m_dt);
    double C_2 = m_cc_brake_Kd * (1/m_dt);

    double output_brake = (C_2 * m_ctrl_diff_old_2_brake) + (C_1 * m_ctrl_diff_old_brake) + (C_0 * m_ctrl_diff);

    m_ctrl_diff_old_2_brake = m_ctrl_diff_old_brake;
    m_ctrl_diff_old_brake = m_ctrl_diff; 

    // Restrict to max/min
    if(output_brake > m_max){
        output_brake = m_max;
    }
    else if(output_brake < m_min){
        output_brake = m_min;
    }

    return (output_brake/100);

}

void CruiseController::setTargetVelocity() {

    if(m_adaptive && m_waypoint_set) {

        // Radius Calculation
        double dist = sqrt(pow(m_waypoint.point.x, 2) + 
                            pow(m_waypoint.point.y, 2));
        // Calculate the trajectory radius
        double r;
        if(m_waypoint.point.y == 0 || dist == 0) {
            r = -1;
        }
        else {
            r = pow(dist, 2) / abs((2 * m_waypoint.point.y));
        }
        
        // Adapting target velocity
        if(r < 0) {
            m_target_velocity = m_adas_cmd.cmd_velocity;
        }
        else {
            // Acceleration if target velocity
            double a = (m_adas_cmd.cmd_velocity * m_adas_cmd.cmd_velocity)/r;
            // Current acceleration
            double a_curr = (m_current_velocity * m_current_velocity)/r;
            // Hysteresis target acceleration
            if(a - m_delta_accel > m_max_accel) {
                // Hysteresis current acceleration
                if(a_curr - m_delta_accel > m_max_accel)
                    m_target_velocity = sqrt(m_max_accel*r);
            }
            else if(a + m_delta_accel < m_max_accel) {
                if(a_curr + m_delta_accel < m_max_accel)
                    m_target_velocity = m_adas_cmd.cmd_velocity;
            }
        }
    }
    // No adaptive target velocity
    else if(!m_adaptive) {
        m_target_velocity = m_adas_cmd.cmd_velocity;
    }

}

// Callback of current velocity
void CruiseController::callbackCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg) {

    m_current_velocity = msg->twist.linear.x;
    m_current_velocity_set = true;

}

// Callback of ADAS command status
void CruiseController::callbackADASCmd(const hska_msgs::ADASCommandConstPtr& msg) {

    m_adas_cmd = *msg;

}

void CruiseController::callbackWaypoint(const geometry_msgs::PointStampedConstPtr& msg) {

    m_waypoint.point = msg->point;
    m_waypoint_set = true;

}

// Callback of reconfigurable parameters
void CruiseController::callbackParameters(cruise_control::CruiseControllerConfig &config, uint32_t level) {

    m_frequency = config.frequency;
    m_loop_rate = ros::Rate(m_frequency);
    m_delta_accel = config.delta_acceleration;
    m_delta_velocity = config.delta_velocity;
    m_adaptive = config.adaptive_velocity;
    m_max_accel = config.max_acceleration;
    m_cc_gas_Kp = config.cc_gas_Kp;
    m_cc_gas_Ki = config.cc_gas_Ki;
    m_cc_gas_Kd = config.cc_gas_Kd;
    m_cc_brake_Kp = config.cc_brake_Kp;
    m_cc_brake_Ki = config.cc_brake_Ki;
    m_cc_brake_Kd = config.cc_brake_Kd;
    
}
