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
#include <visualization_msgs/Marker.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <pure_pursuit/PurePursuitVisualizerConfig.h>

// C++
#define _USE_MATH_DEFINES
#include <math.h>

class PurePursuitVisualizer {

    public:

        PurePursuitVisualizer();
        ~PurePursuitVisualizer(); 

        // Main loop of node
        void run();

    private:

        // ROS specific
        ros::NodeHandle m_nh;
        ros::Rate m_loop_rate;
        ros::Publisher m_traj_circle_pub;
        ros::Subscriber m_next_wp_sub;
		
        // Member variables;	
       	bool m_next_wp_set;
        std::vector<geometry_msgs::Point> m_traj_circle_arr;
        visualization_msgs::Marker m_traj_circle_mark;
        geometry_msgs::PointStamped m_next_wp;
       	double m_frequency;
        double m_radius;

        // Dynamic reconfigure
        dynamic_reconfigure::Server<pure_pursuit::PurePursuitVisualizerConfig> server;
        dynamic_reconfigure::Server<pure_pursuit::PurePursuitVisualizerConfig>::CallbackType f;

        // Member Functions
        void init();
        void calcRadius();
        void generateTrajectoryCircle();
        void displayTrajectoryCircle();
        geometry_msgs::Point rotatePoint(geometry_msgs::Point point, double degree);

        // Callbacks
        void callbackCurrentWaypoint(const geometry_msgs::PointStampedConstPtr& msg);
        void callbackParameters(pure_pursuit::PurePursuitVisualizerConfig &config, uint32_t level);

        // Inline Functions
        inline double deg2rad(double deg) {
            return deg * M_PI / 180;
        }

};

#endif
