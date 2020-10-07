// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: TODO Adding authors
// =================================================================
// TODO change "package_name" with the name of your package
// TODO change "node1_name" or "Node1Name" with the name of your node

#include "node1_name/node1_name.h"

// Constructor with list init
Node1Name::Node1Name() : 
  m_loop_rate (10.0),
  m_dummy_rqt (20.0),
  m_dummy_set (false),
  m_next_wp_set (false) {
    // call of init-function
    init();

}

// Destructor
Node1Name::~Node1Name() {
}

// Initialization
void Node1Name::init() {

    // Dynamic reconfigure
    f = boost::bind(&Node1Name::callbackParameters, this, _1, _2);
    server.setCallback(f);

    // Publisher
    // TODO define puplisher
    // For help: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
    // Syntax: 
    // m_dummy_pub = m_nh.advertise<message_package::message_type>("/topic", queue_size);
    // Example:
    m_steer_pub = m_nh.advertise<hska_msgs::SteeringCommand>("/node1/steer", 1);

    // Subscriber
    // TODO define subscriber
    // For help: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29 */
    // Syntax: 
    // m_dummy_sub = m_nh.subscribe("/topic", queue_size, &Node1Name::callbackfunction, this);
    // Example:
    m_next_wp_sub = m_nh.subscribe("/node1/wp", 1, &Node1Name::callbackCurrentWaypoint, this);

    // Parameters from parameter server
    // TODO Adding dynamic reconfigure parameters
    // Syntax: m_nh.param( "parameter_name_in_rqt", parameter_name_in_c++, default_value);
    //Examples:
    m_nh.param("frequency", m_frequency, 10.0);
    m_nh.param("dummy_parameter", m_dummy_rqt, 20.0);

}

// Main loop of the node
void Node1Name::run() {

    m_loop_rate = ros::Rate(m_frequency);

    while (ros::ok()) {
        ros::spinOnce();
      	if (m_dummy_set) {
            // TODO Adding functions, which should be called if the subscriber_var is updated
            function_dummy();
            // ...
            // m_dummy_pub.publish(m_dummy); 
            // example: 
            m_steer_pub.publish(m_steer);
            m_dummy_set = false; // reset set-var
        }
        m_loop_rate.sleep();
    }
	
}

// Function description
void Node1Name::function_dummy() {

  // TODO Place here your code which is called in the run()-function

}



/* Callback of dummy variable
void Node1Name::callbackDummy (const message_package::message_typedConstPtr& msg) {

    m_dummy = *msg;
    m_dummy_set = true; 
	
} */

// Example: 
// Callback of current waypoint
void Node1Name::callbackCurrentWaypoint (const geometry_msgs::PointStampedConstPtr& msg) {

    m_next_wp = *msg;
    m_next_wp_set = true;
	
}

// Callback of reconfigurable parameters
void Node1Name::callbackParameters(package_name::Node1NameConfig &config, uint32_t level) {
    // TODO Adding reconfigurable parameter to callback*/
    // Syntax:parameter_name_in_c++ = config.parameter_name_in_rqt;
    // Basics
    m_loop_rate = ros::Rate(config.frequency);
    m_frequency = config.frequency;
    // Own Parameters
    m_dummy_rqt = config.dummy_parameter;
    
	
}
