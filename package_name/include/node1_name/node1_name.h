// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: TODO Adding authors
// =================================================================

// TODO change "package_name" with the name of your package
// TODO change "node1_name" or "Node1Name" with the name of your node

#ifndef NODE1_NAME_H
#define NODE1_NAME_H

// TODO Adding ROS specific header
// ROS 
#include <ros/ros.h>

// TODO Adding header for used messages
// Syntax: #include <message_package/message_type.h> 
// packages: std_msgs (http://wiki.ros.org/std_msgs) or 
// common_msgs: geometry_msgs, sensor_msgs,... (http://wiki.ros.org/common_msgs)
// for example:(for messages from line 54)
#include <geometry_msgs/PointStamped.h>
// specific hska message (please refer folder hska_message)
#include <hska_msgs/SteeringCommand.h> 
#include <hska_msgs/ADASCommand.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <package_name/Node1NameConfig.h>


class Node1Name {

    public:

        Node1Name();
        ~Node1Name(); 

        // Main loop of node
        void run();

    private:

        // ROS specific
        // TODO Adding variables for NodeHandler
        ros::NodeHandle m_nh;
        // Rate of loop
        ros::Rate m_loop_rate;
        // Publisher and Subscriber for ROS node (as many as you need)
        // Syntax
        // ros::Publisher m_dummy_pub;
        // ros::Subscriber m_dummy_sub;
        // Example:
        ros::Publisher m_steer_pub;
        ros::Subscriber m_next_wp_sub;


        // Messages
        // TODO Adding variables for used messages
        // Syntax: 
        // message_package::message_type m_var;
        // examples:
        hska_msgs::SteeringCommand m_steer;
        geometry_msgs::PointStamped m_next_wp;
		
        // Member variables
        // TODO Adding member variables for your node
        bool m_dummy_set;
        bool m_next_wp_set;
        double m_frequency;
 
        // own variables
        double m_dummy_rqt; // reconfigurable parameter
        //...

        // Dynamic reconfigure
        dynamic_reconfigure::Server<package_name::Node1NameConfig> server;
        dynamic_reconfigure::Server<package_name::Node1NameConfig>::CallbackType f;

        // Member Functions
        // TODO Declare function of the node
        void init();
        void function_dummy();
        //...

        // Callbacks
        // TODO Adding callback function for subscriber and dynamic reconfigure
        // For help: http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers*/
        // Syntax: 
        // void callbackName(const message_package::message_typeConstPtr& msg);
        // Example for Waypoint:
        void callbackCurrentWaypoint(const geometry_msgs::PointStampedConstPtr& msg);

        //Callback for dynamic reconfigure
        void callbackParameters(package_name::Node1NameConfig &config, uint32_t level);

};

#endif
