// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#ifndef LANE_DETECTION_H
#define LANE_DETECTION_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// HSKA messages
#include <hska_msgs/LaneBoundary.h>
#include <hska_msgs/LaneBoundaryArray.h>
#include <hska_msgs/ADASCommand.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <lane_detection/LaneDetectorConfig.h>
#include <lane_detection/LaneVisualizerConfig.h>

// C++
#include <string.h>
#include <vector>
#include <opencv2/opencv.hpp>

// Custom
#include "poly_fit.h"
#include "peak_finder.h"

class LaneDetector {

    public:

        LaneDetector();
        ~LaneDetector();

        // Main loop of node
        void run();

    private:

        // Constants
        const double PI;

        // ROS specific
        ros::NodeHandle m_nh;
        ros::Rate m_loop_rate;
        ros::Publisher m_waypoint_pub;
        ros::Publisher m_lane_lines_pub;
        ros::Publisher m_output_img_pub;
        ros::Subscriber m_current_velocity_sub;
        ros::Subscriber m_vis_param_sub;
        image_transport::ImageTransport m_it;
        image_transport::Publisher m_detect_image_pub;
        image_transport::Publisher m_warped_image_pub;
        image_transport::Publisher m_hist_image_pub;
        image_transport::Subscriber m_orig_image_sub;

        // Peak finder and interpolation
        PeakFinder m_peak_finder;
        PolyFit m_poly_right;
        PolyFit m_poly_left;
        bool m_poly_left_set;
        bool m_poly_right_set;

        // Member variables
        double m_frequency;
        bool m_display_information;
        
        double m_look_ahead_min;
        double m_look_ahead_max;
        double m_look_ahead_ratio;
        
        bool m_multi_lane_detection;
        double m_pixel_to_m;
        int m_sw[2];
        int m_hist_limit;

        bool m_publish_img_debug;
        bool m_publish_img_detection;
        bool m_draw_lane_lines;
        bool m_draw_waypoint;
        int m_draw_thickness;
        int m_waypoint_radius;
        cv::Scalar m_color_right_line;
        cv::Scalar m_color_left_line;
        cv::Scalar m_color_other_line;
        cv::Scalar m_color_waypoint;
        
        ros::Time m_img_input_time;
        bool m_is_image_set;
        bool m_is_velocity_set;
        bool m_save_img;
        std::string m_calib_path;
        int m_polynom_degree;
        double m_current_velocity;
        double m_last_lane_width;
        double m_baselink_to_img;
        int m_color_thresh;
        geometry_msgs::PointStamped m_next_waypoint;
        hska_msgs::LaneBoundaryArray m_lane_lines;
        cv::Mat m_img_input;
        cv::Mat m_img_warped;
        cv::Mat m_img_hist;
        cv::Mat m_warp_mat;
        cv::Mat m_warp_mat_inv;
        cv::Size m_warp_size;

        // Dynamic reconfigure
        dynamic_reconfigure::Server<lane_detection::LaneDetectorConfig> m_server;
        dynamic_reconfigure::Server<lane_detection::LaneDetectorConfig>::CallbackType m_f;

        // Member functions
        void init();
        // Lane detection pipeline
        void detectLaneLines();
        // Sliding window method to find lane line points
        std::vector<cv::Point2f> slidingWindow(cv::Mat image, cv::Rect window, hska_msgs::LaneBoundary &bound);
        // calculate next waypoint in base_link coordinates
        void calculateWaypoint();
        // publish messages
        void publishMessages(bool is_wp_set);

        // Callbacks
        void callbackVelocity(const geometry_msgs::TwistStampedConstPtr& msg);
        void callbackImage(const sensor_msgs::ImageConstPtr& msg);
        void callbackADASCmd(const hska_msgs::ADASCommandConstPtr& msg);
        void callbackParameters(lane_detection::LaneDetectorConfig &config, uint32_t level);
        void callbackVisualParameters(const std_msgs::BoolConstPtr& msg);

};

#endif

