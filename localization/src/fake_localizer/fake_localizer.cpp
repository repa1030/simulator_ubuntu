// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#include "fake_localizer/fake_localizer.h"

// Constructor
FakeLocalizer::FakeLocalizer() : 
  m_loop_rate(30.0),
  m_current_pose_set(false) {

    init();

}

// Destructor
FakeLocalizer::~FakeLocalizer() {
}

// Initialization
void FakeLocalizer::init() {

    ROS_INFO("***[fake_localizer]: Node Initializing.");

    // Dynamic reconfigure
    f = boost::bind(&FakeLocalizer::callbackParameters, this, _1, _2);
    server.setCallback(f);

    // Subscriber
    m_pose_sub = m_nh.subscribe("/vehicle/ground_truth/current_pose", 1, &FakeLocalizer::callbackCurrentPose, this);

    // Parameters from parameter server
    m_nh.param("frequency", m_frequency, 30.0);

}

// Main loop of the node
void FakeLocalizer::run() {

    m_loop_rate = ros::Rate(m_frequency);

    ROS_INFO("***[fake_localizer]: Node is Running.");

    while(ros::ok()) {
        ros::spinOnce();
      	if(m_current_pose_set) {
            getVehiclePosition();
            publishPosition();
            m_current_pose_set = false;
        }
        m_loop_rate.sleep();
    }
	
}

// Get position of vehicle and save in transform
void FakeLocalizer::getVehiclePosition() {

    m_tf.setOrigin(tf::Vector3(m_current_pose.pose.position.x,
                                m_current_pose.pose.position.y,
                                m_current_pose.pose.position.z));
    m_tf.setRotation(tf::Quaternion(m_current_pose.pose.orientation.x,
                                    m_current_pose.pose.orientation.y,
                                    m_current_pose.pose.orientation.z,
                                    m_current_pose.pose.orientation.w));

}

// Publish position of vehicle as tf transform
void FakeLocalizer::publishPosition() {

    m_br.sendTransform(tf::StampedTransform(m_tf, m_current_pose.header.stamp, "/world", "/base_link"));

}

// Callback of current waypoint
void FakeLocalizer::callbackCurrentPose (const geometry_msgs::PoseStampedConstPtr& msg) {

    m_current_pose = *msg;
    m_current_pose_set = true;
	
}

// Callback of reconfigurable parameters
void FakeLocalizer::callbackParameters(localization::FakeLocalizerConfig &config, uint32_t level) {

    m_frequency = config.frequency;
    m_loop_rate = ros::Rate(m_frequency);
	
}
