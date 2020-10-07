// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#ifndef FAKE_LOCALIZER_H
#define FAKE_LOCALIZER_H

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <localization/FakeLocalizerConfig.h>

class FakeLocalizer {

    public:

        FakeLocalizer();
        ~FakeLocalizer(); 

        // Main loop of node
        void run();

    private:

        // ROS specific
        ros::NodeHandle m_nh;
        ros::Rate m_loop_rate;
        ros::Subscriber m_pose_sub;

        // Messages
        geometry_msgs::PoseStamped m_current_pose;
		
        // Member variables;	
       	bool m_current_pose_set;
       	double m_frequency;
        tf::TransformBroadcaster m_br;
        tf::Transform m_tf;

        // Dynamic reconfigure
        dynamic_reconfigure::Server<localization::FakeLocalizerConfig> server;
        dynamic_reconfigure::Server<localization::FakeLocalizerConfig>::CallbackType f;

        // Member Functions
        void init();
        void getVehiclePosition();
        void publishPosition();

        // Callbacks
        void callbackCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
        void callbackParameters(localization::FakeLocalizerConfig &config, uint32_t level);

};

#endif
