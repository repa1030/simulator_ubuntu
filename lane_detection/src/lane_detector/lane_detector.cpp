// =================================================================
// Copyright (C) 2020 Hochschule Karlsruhe - Technik und Wirtschaft
// This program and the accompanying materials
// are made available under the terms of the MIT license.
// =================================================================
// Authors: Anna-Lena Marmein, Magdalena Kugler, Yannick Rauch,
//          Markus Zimmermann, Patrick Rebling
// =================================================================

#include "lane_detector/lane_detector.h"

// Contructor
LaneDetector::LaneDetector() :
  PI(3.14159265),
  m_it(m_nh),
  m_loop_rate(30.0),
  m_is_image_set(false),
  m_is_velocity_set(false),
  m_last_lane_width(-1),
  m_poly_right(PolyFit()),
  m_poly_left(PolyFit()) {

    init();

}

// Destructor
LaneDetector::~LaneDetector() {

}

// Initialization
void LaneDetector::init() {

    ROS_INFO("***[lane_detector]: Node Initializing.");

    double dst_plane[2];
    double padding;
    double cam_rot, cam_fov, cam_h, base_to_cam;
    int color_right[3], color_left[3], color_other[3], color_wp[3];
    cv::Point2f src_p[4];
    cv::Point2f dst_p[4];

    // Dynamic reconfigure
    m_f = boost::bind(&LaneDetector::callbackParameters, this, _1, _2);
    m_server.setCallback(m_f);

    // Publishers
    m_waypoint_pub = m_nh.advertise<geometry_msgs::PointStamped>("/lanes/next_waypoint", 1);
    m_lane_lines_pub = m_nh.advertise<hska_msgs::LaneBoundaryArray>("/lanes/lines", 1);
    m_detect_image_pub = m_it.advertise("/lane_detector/detected_image", 1);
    m_warped_image_pub = m_it.advertise("/lane_detector/warped_image", 1);
    m_hist_image_pub = m_it.advertise("/lane_detector/histogramm_image", 1);

    // Subscribers
    m_vis_param_sub = m_nh.subscribe("/lane_visualizer/param_update_msg", 1, &LaneDetector::callbackVisualParameters, this);
    m_orig_image_sub = m_it.subscribe("/vehicle/cameras/front_center", 1, &LaneDetector::callbackImage, this, image_transport::TransportHints("compressed"));
    m_current_velocity_sub = m_nh.subscribe("/vehicle/current_velocity", 1, &LaneDetector::callbackVelocity, this);

    // Parameters from parameter server
    m_nh.param("frequency", m_frequency, 30.0);
    m_nh.param("display_information", m_display_information, false);

    m_nh.param("look_ahead_min_distance", m_look_ahead_min, 8.0);
    m_nh.param("look_ahead_max_distance", m_look_ahead_max, 20.0);
    m_nh.param("look_ahead_ratio", m_look_ahead_ratio, 1.0);

    m_nh.param("multi_lane_detection", m_multi_lane_detection, true);
    m_nh.param("polynom_degree", m_polynom_degree, 2);
    m_nh.param("pixel_to_m", m_pixel_to_m, 50.0);
    m_nh.param("padding_width", padding, 7.0);
    m_nh.param("sliding_window_width", m_sw[0], 75);
    m_nh.param("sliding_window_height", m_sw[1], 50);
    m_nh.param("histogramm_limit", m_hist_limit, 360);
    m_nh.param("color_threshold", m_color_thresh, 100);

    m_nh.param("calibration_image_path", m_calib_path, std::string("/home/ros/Pictures"));
    m_nh.param("save_calibration_image", m_save_img, false);
    m_nh.param("camera_height", cam_h, 1.33);
    m_nh.param("camera_fov_vertical", cam_fov, 36.0);
    m_nh.param("camera_rotation_vertical", cam_rot, -3.0);
    m_nh.param("baselink_camera_distance", base_to_cam, 0.42);
    m_nh.param("src_pt1_x", src_p[0].x, 264.0F);
    m_nh.param("src_pt1_y", src_p[0].y, 719.0F);
    m_nh.param("src_pt2_x", src_p[1].x, 953.0F);
    m_nh.param("src_pt2_y", src_p[1].y, 719.0F);
    m_nh.param("src_pt3_x", src_p[2].x, 741.0F);
    m_nh.param("src_pt3_y", src_p[2].y, 514.0F);
    m_nh.param("src_pt4_x", src_p[3].x, 517.0F);
    m_nh.param("src_pt4_y", src_p[3].y, 514.0F);
    m_nh.param("plane_width", dst_plane[0], 3.00);
    m_nh.param("plane_length", dst_plane[1], 10.00);

    // Get visualization data from lane_visualizer config    
    m_nh.param("/lane_visualizer/publish_debug_images", m_publish_img_debug, true);
    m_nh.param("/lane_visualizer/publish_detection_image", m_publish_img_detection, true);
    m_nh.param("/lane_visualizer/draw_lane_lines", m_draw_lane_lines, true);
    m_nh.param("/lane_visualizer/draw_waypoint", m_draw_waypoint, true);
    m_nh.param("/lane_visualizer/line_thickness", m_draw_thickness, 4);
    m_nh.param("/lane_visualizer/waypoint_radius", m_waypoint_radius, 6);
    m_nh.param("/lane_visualizer/color_r_r", color_right[0], 0);
    m_nh.param("/lane_visualizer/color_g_r", color_right[1], 255);
    m_nh.param("/lane_visualizer/color_b_r", color_right[2], 0);
    m_nh.param("/lane_visualizer/color_r_l", color_left[0], 0);
    m_nh.param("/lane_visualizer/color_g_l", color_left[1], 0);
    m_nh.param("/lane_visualizer/color_b_l", color_left[2], 255);
    m_nh.param("/lane_visualizer/color_r_o", color_other[0], 255);
    m_nh.param("/lane_visualizer/color_g_o", color_other[1], 0);
    m_nh.param("/lane_visualizer/color_b_o", color_other[2], 0);
    m_nh.param("/lane_visualizer/color_r_wp", color_wp[0], 255);
    m_nh.param("/lane_visualizer/color_g_wp", color_wp[1], 140);
    m_nh.param("/lane_visualizer/color_b_wp", color_wp[2], 0);
    m_color_right_line = cv::Scalar(color_right[2], color_right[1], color_right[0]);
    m_color_left_line = cv::Scalar(color_left[2], color_left[1], color_left[0]);
    m_color_other_line =  cv::Scalar(color_other[2], color_other[1], color_other[0]);
    m_color_waypoint = cv::Scalar(color_wp[2], color_wp[1], color_wp[0]);

    // Preprocessing CV
    double y_1, y_2;
    // Check for look ahead distance, resize if too short
    if (m_look_ahead_max < dst_plane[1]) {
        y_1 = dst_plane[1];
        y_2 = 0.0;
    }
    else {
        y_1 = m_look_ahead_max;
        y_2 = m_look_ahead_max - dst_plane[1];
    }
    // Calculate the distance from base_link to image bottom
    m_baselink_to_img = base_to_cam + (cam_h / tan((cam_fov/2 + cam_rot) * PI / 180.0));
    // Calculcate destination points for perspective transform
    dst_p[0].x = padding * m_pixel_to_m;
    dst_p[0].y = y_1 * m_pixel_to_m;
    dst_p[1].x = (dst_plane[0] + padding) * m_pixel_to_m;
    dst_p[1].y = dst_p[0].y;
    dst_p[2].x = dst_p[1].x;
    dst_p[2].y = y_2 * m_pixel_to_m;
    dst_p[3].x = dst_p[0].x;
    dst_p[3].y = dst_p[2].y;
    // Get the transform matrix and also the one for inverse transform
    m_warp_mat = cv::getPerspectiveTransform(src_p, dst_p);
    m_warp_mat_inv = cv::getPerspectiveTransform(dst_p, src_p);
    // Save the size that the transformed image should have
    m_warp_size = cv::Size(
        (dst_plane[0] + 2 * padding) * m_pixel_to_m, 
        y_1 * m_pixel_to_m
    );
    // Save information in the lane line message for visualization
    m_lane_lines.pixel_to_m = m_pixel_to_m;
    m_lane_lines.min_look_ahead = m_baselink_to_img;
    m_lane_lines.max_look_ahead = m_look_ahead_max;
    m_lane_lines.img_width = m_warp_size.width;
    m_lane_lines.img_height = m_warp_size.height;

    // Initialize ROS messages
    m_next_waypoint.header.frame_id = "base_link";
    m_lane_lines.header.frame_id = "base_link";

}

// Main loop of the node
void LaneDetector::run() {

    m_loop_rate = ros::Rate(m_frequency);

    ROS_INFO("***[lane_detector]: Node is Running.");

    while(ros::ok()) {
        ros::spinOnce();
        // Check if image is set and velocity is set
        if(m_is_image_set && m_is_velocity_set) {
            // Execute lane detection pipeline
            detectLaneLines();
            // Check if there are both polynoms
            if(m_poly_right_set && m_poly_left_set) {
                // Calculate the next waypoint
                calculateWaypoint();
                m_poly_right_set = false;
                m_poly_left_set = false;
                publishMessages(true);
            }
            else {
                // publish the messages without waypoint
                publishMessages(false);
            }
            // Reset the check variables
            m_is_image_set = false;
            m_is_velocity_set = false;
            // Display output if desired
            if (m_display_information) {
                ROS_INFO_THROTTLE(5, "***[lane_detector]: Last Known Driving Lane Width: %.2fm", m_last_lane_width);       
            }        
        }      
        m_loop_rate.sleep();
    }

}

// Detection pipline
void LaneDetector::detectLaneLines() {

    if (m_save_img) {
        auto img_path = m_calib_path;
        img_path += "/calib_img_";
        img_path += std::to_string((int)ros::Time::now().toSec());
        img_path += ".png";
        cv::imwrite(img_path, m_img_input);
        m_save_img = false;
        ROS_INFO("***[lane_detector]: Calibration image saved to %s.", m_calib_path.c_str());
    }

    // Destination for warped image
    cv::Mat dst(m_warp_size, CV_8UC3);

    // Working image
    cv::Mat img;

    // Warp and convert to gray
    cv::warpPerspective(m_img_input, dst, m_warp_mat, dst.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::cvtColor(dst, img, cv::COLOR_RGB2GRAY);
    
    // Extract yellow and white info
    cv::Mat maskYellow, maskWhite;
    
    cv::inRange(img, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), maskYellow);
    cv::inRange(img, cv::Scalar(150, 150, 150), cv::Scalar(255, 255, 255), maskWhite);
    
    cv::Mat mask, processed;
    cv::bitwise_or(maskYellow, maskWhite, mask); //Combine the two masks
    cv::bitwise_and(img, mask, processed); // Extract
    
    // Blur the image a bit so that gaps are smoother
    const cv::Size kernelSize = cv::Size(9, 9);
    GaussianBlur(processed, processed, kernelSize, 0);

    // Try to fill the gaps
    cv::Mat kernel = cv::Mat::ones(15, 15, CV_8U);
    cv::dilate(processed, processed, kernel);
    cv::erode(processed, processed, kernel);
    cv::morphologyEx(processed, processed, cv::MORPH_CLOSE, kernel);
    
    // Keep only what's above m_color_thresh value, other is then black 
    cv::threshold(processed, processed, m_color_thresh, 255, cv::THRESH_BINARY);

    // Save warped image for output and drawing
    m_img_warped = cv::Mat(m_warp_size, CV_8UC3);
    cv::cvtColor(processed, m_img_warped, cv::COLOR_GRAY2RGB);

    // Generate histogramm of input image
    std::vector<int> values;
    int count = 0;
    // Make sure that total image height is maximum of hist limit
    if(m_hist_limit > processed.rows)
        m_hist_limit = processed.rows;
    for(int i = 0; i < processed.cols; i++) {
        for(int j = processed.rows; j > processed.rows-m_hist_limit; j--) {
            if(processed.at<uchar>(j,i) == 255) count++;
        }
        values.push_back(count);
        count = 0;
    }

    // Search peaks in histogramm
    std::vector<int> peak_out;
    std::vector<int> peaks;
    int dr, dl, i_dr, i_dl;
    // Mutli lane detection
    if(m_multi_lane_detection) {
        // Apply peak finder
        m_peak_finder.findPeaks(values, peak_out);
        if(peak_out.empty()) return;
        // Remove peaks that are near each other to avoid overlaying lane lines
        for(int i = 1; i < peak_out.size(); i++) {
            if (peak_out[i] - peak_out[i-1] > m_sw[0])
                peaks.push_back(peak_out[i-1]);
        }
        // Check also the end point
        if (peak_out[peak_out.size()-1] - peak_out[peak_out.size()-2] > m_sw[0])
            peaks.push_back(peak_out[peak_out.size()-1]);
        peak_out = peaks;
        float vehicle_pos = m_img_warped.cols/2;
        dr = peak_out[0];
        dl = peak_out[0];
        // Search for left and right driving lane
        for(int i = 0; i < peak_out.size(); i++) {
            if((vehicle_pos - dl > vehicle_pos - peak_out[i] && vehicle_pos - peak_out[i] >= 0)
                    || (vehicle_pos - dl < vehicle_pos - peak_out[i] && vehicle_pos - dl < 0))
                dl = peak_out[i];
                i_dl = i;
            if((vehicle_pos - dr < vehicle_pos - peak_out[i] && vehicle_pos - peak_out[i] <= 0)
                    || (vehicle_pos - dr > vehicle_pos - peak_out[i] && vehicle_pos - dr > 0))
                dr = peak_out[i];
                i_dr = i;
        }
    }
    // Single lane detection (useful at night scenes)
    else {
        // Getting highest value in histogramm 
        std::vector<int>::iterator left_highest, right_highest;
        std::size_t half_size = values.size() / 2;
        left_highest = std::max_element(values.begin(), values.begin() + half_size-1);
        right_highest = std::max_element(values.begin() + half_size, values.end());
        dr = std::distance(values.begin(), right_highest);
        dl = std::distance(values.begin(), left_highest);
        peak_out.push_back(dr);
        peak_out.push_back(dl);
    }

    // Plotting histogramm
    if(m_publish_img_debug) {

        std::vector<int>::iterator highest = std::max_element(values.begin(), values.end());
        m_img_hist = cv::Mat(*highest, values.size(), CV_8UC3, cv::Scalar(0, 0, 0));

        for(int i = 1; i < values.size(); i++) {
            cv::line(m_img_hist, cv::Point(i-1, *highest - values.at(i-1)), 
                    cv::Point(i, *highest - values.at(i)), cv::Scalar(255, 255, 255), m_draw_thickness);
        }
        for(int i = 0; i < peak_out.size(); i++) {
            if(peak_out[i] == dl) {
                cv::circle(m_img_warped, cv::Point(peak_out[i], m_img_warped.rows-10), 5, cv::Scalar(255,0,0), 5);
                cv::circle(m_img_hist, cv::Point(peak_out[i], m_img_hist.rows-10), 5, cv::Scalar(255,0,0), 5);
            }
            else if(peak_out[i] == dr) {
                cv::circle(m_img_warped, cv::Point(peak_out[i], m_img_warped.rows-10), 5, cv::Scalar(0,255,0), 5);
                cv::circle(m_img_hist, cv::Point(peak_out[i], m_img_hist.rows-10), 5, cv::Scalar(0,255,0), 5);
            }
            else {
                cv::circle(m_img_warped, cv::Point(peak_out[i], m_img_warped.rows-10), 5, cv::Scalar(0,0,255), 5);
                cv::circle(m_img_hist, cv::Point(peak_out[i], m_img_hist.rows-10), 5, cv::Scalar(0,0,255), 5);
            }
        }
    }

    // Applying sliding windows to peaks and polynom fitting
    // Points in Line
    std::vector<cv::Point2f> pts_lane;
    // Polynom
    std::vector<double> poly_coeffs;
    std::vector<PolyFit> interpol;
    // Visualization on warped image
    std::vector<cv::Point2f> line;
    cv::Scalar color;
    // RVIZ
    hska_msgs::LaneBoundary bound;
    bound.color = hska_msgs::LaneBoundary::UNKNOWN;
    m_lane_lines.boundaries.clear();

    // Apply detection to all found peaks
    for(int i = 0; i < peak_out.size(); i++) {
        // Apply sliding window
        pts_lane = slidingWindow(processed, cv::Rect(peak_out[i] - m_sw[0]/2, processed.rows - m_sw[1], m_sw[0], m_sw[1]), bound);
        // Interpolate
        PolyFit poly(pts_lane, true, m_polynom_degree);
        // Save coefficients in lane line message
        bound.polynom_coeffs = poly.getPolynomCoefficients();

        // What kind of lane line? (driving_right, driving_left, other)
        // fill the corresponding data into lane line information
        if(peak_out[i] == dl) {
            color = m_color_left_line;
            m_poly_left = poly;
            m_poly_left_set = true;
            bound.line_pos = hska_msgs::LaneBoundary::DRIVING_LEFT;
        }        
        else if(peak_out[i] == dr) {
            color = m_color_right_line;
            m_poly_right = poly;
            m_poly_right_set = true;
            bound.line_pos = hska_msgs::LaneBoundary::DRIVING_RIGHT;
        }
        else {
            color = m_color_other_line;
            bound.line_pos = hska_msgs::LaneBoundary::OTHER;
        }
        m_lane_lines.boundaries.push_back(bound);

        // Drawing on images
        if(m_draw_lane_lines && (m_publish_img_detection || m_publish_img_debug)) {
            std::vector<cv::Point2f> points_tf;
            int mod_fac = 100;
            for (int y = 1; y < processed.rows; y++) {
                // Get points (warped and original)
                cv::Point2f point1(cv::Point2f(poly.getValueAt(y - 1), y - 1));
                cv::Point2f point2(cv::Point2f(poly.getValueAt(y), y));
                points_tf.push_back(point1);
                points_tf.push_back(point2);
                cv::perspectiveTransform(points_tf, points_tf, m_warp_mat_inv);
                if(bound.style == hska_msgs::LaneBoundary::SOLID) {
                    cv::line(m_img_warped, point1, point2, color, m_draw_thickness);
                    cv::line(m_img_input, points_tf[0], points_tf[1], color, m_draw_thickness);
                }
                else if(bound.style == hska_msgs::LaneBoundary::DASHED && (y % mod_fac == 0)) {
                    cv::circle(m_img_warped, point1, m_draw_thickness, color, -1);
                    cv::circle(m_img_input, points_tf[0], m_draw_thickness, color, -1);
                    mod_fac--;
                }
                points_tf.clear();
            }             
        }
    }

}

// Sliding window for lane line detection
std::vector<cv::Point2f> LaneDetector::slidingWindow(cv::Mat image, cv::Rect window, hska_msgs::LaneBoundary &bound) {

    std::vector<cv::Point2f> points;
    const cv::Size imgSize = image.size();
    bool should_break = false;
    bool first_line_pixels_found = false;
    bool line_interruption = false;

    // Check if image is to small for sliding window size
    // Resize the sliwind window if its too small
    if(imgSize.width <= window.width) {
        double hist_pt = window.x + window.width/2.0;
        window.width = imgSize.width/4.0;
        window.x = hist_pt - window.width/2.0;
    }
    if(imgSize.height <= window.height) {
        window.height = imgSize.height/10.0;
        window.y = imgSize.height - window.height;
    }

    // Set lane boundary type to solid until interruption is found
    bound.style = hska_msgs::LaneBoundary::SOLID;

    // Initial sliding window check:
    // Make sure the window doesn't overflow, we get an error if we try to get data outside the matrix
    window.x = window.x < 0 ? 0 : window.x;
    window.x = (window.x + window.width >= imgSize.width) ? imgSize.width - window.width - 1 : window.x;

    // Sliding window
    while(true) {
        float currentX = window.x + window.width * 0.5f;
        cv::Mat roi = image(window);
        std::vector<cv::Point> locations;
        cv::findNonZero(roi, locations); // Get all non-black pixels. All are white in our case      
        float avgX = 0.0f;
        for (int i = 0; i < locations.size(); ++i) // Calculate average X position
        {
            float x = locations[i].x;
            avgX += window.x + x;
        }
        avgX = locations.empty() ? currentX : avgX / locations.size();
        cv::Point point(avgX, window.y + window.height * 0.5f);
        // Draw sliding windows on debug image
        if(locations.empty() && m_publish_img_debug) {
            cv::rectangle(m_img_warped, window, cv::Scalar(0,0,255), 3);
        }
        else if(m_publish_img_debug) {
            cv::rectangle(m_img_warped, window, cv::Scalar(0,255,0), 3);
        }
        // Add point and check line style
        if(!locations.empty()) {
            // First white pixels found -> beginn of line
            first_line_pixels_found = true;
            points.push_back(point);
            // Was the line already interrupted?
            if (line_interruption) {
                // Therefore the line is dashed
                bound.style = hska_msgs::LaneBoundary::DASHED;
            }
        }
        // Lane line found but now there is an interruption
        else if(first_line_pixels_found) {
            line_interruption = true;
        }

        // Move the window up
        window.y -= window.height;
        
        // For the uppermost position
        if(window.y < 0) {
            window.y = 0;
            should_break = true;
        }
        
        // Move x position
        window.x += (point.x - currentX);
        
        // Make sure the window doesn't overflow, we get an error if we try to get data outside the matrix
        window.x = window.x < 0 ? 0 : window.x;
        window.x = (window.x + window.width >= imgSize.width) ? imgSize.width - window.width - 1 : window.x;
        
        if(should_break)
            break;   
    }
    
    return points;

}

// Calculate waypoint
void LaneDetector::calculateWaypoint() {

    double la, la_tmp;
    // Error avoidance
    // Look ahead distance should not be smaller than distance from baselink to image 
    if(m_look_ahead_max < m_baselink_to_img)
        m_look_ahead_max = m_baselink_to_img;
    // Calculate look ahead distance
    la_tmp = m_look_ahead_ratio * m_current_velocity;
    // Check if look ahead distance is bigger than maximum or smaller than minimum
    la_tmp = (la_tmp < m_baselink_to_img) ? 0 : la_tmp - m_baselink_to_img;
    la = (la_tmp < m_look_ahead_min - m_baselink_to_img) ? 
            m_look_ahead_min - m_baselink_to_img : (la_tmp > m_look_ahead_max - m_baselink_to_img) ? 
                m_look_ahead_max - m_baselink_to_img : la_tmp;
    // Calculate look ahead distance in image coordinates
    la = m_img_warped.rows - (la * m_pixel_to_m);
    // Next waypoint in pixel (image coordinates)
    geometry_msgs::Point wp_tmp;
    // Waypoint is desired to be in the middle of the driving lane
    wp_tmp.x = (m_poly_right.getValueAt(la) + m_poly_left.getValueAt(la)) / 2;
    wp_tmp.y = la;
    // Write waypoint to vector cause perspectiveTransform() required a vector
    std::vector<cv::Point2f> wp_img;
    wp_img.push_back(cv::Point(wp_tmp.x, wp_tmp.y));
    // Draw waypoint if desired
    if(m_draw_waypoint) {
        #if CV_MAJOR_VERSION == 3
        cv::circle(m_img_warped, wp_img[0], m_waypoint_radius, m_color_waypoint, CV_FILLED);
        cv::perspectiveTransform(wp_img, wp_img, m_warp_mat_inv);
        cv::circle(m_img_input, wp_img[0], m_waypoint_radius, m_color_waypoint, CV_FILLED);
        #else
        cv::circle(m_img_warped, wp_img[0], m_waypoint_radius, m_color_waypoint, cv::FILLED);
        cv::perspectiveTransform(wp_img, wp_img, m_warp_mat_inv);
        cv::circle(m_img_input, wp_img[0], m_waypoint_radius, m_color_waypoint, cv::FILLED);
        #endif
    }
    // Convert to m and to base_link frame
    m_next_waypoint.header.stamp = m_img_input_time;
    m_next_waypoint.point.x = m_baselink_to_img + (static_cast<float>(m_img_warped.rows) - wp_tmp.y) / m_pixel_to_m;
    m_next_waypoint.point.y = (static_cast<float>(m_img_warped.cols) / 2.0 - wp_tmp.x) / m_pixel_to_m;

    // Check for NaN
    m_next_waypoint.point.x = isnan(m_next_waypoint.point.x) ? 0.0 : m_next_waypoint.point.x;
    m_next_waypoint.point.y = isnan(m_next_waypoint.point.y) ? 0.0 : m_next_waypoint.point.y;

    // Save last known lane width
    m_last_lane_width = (m_poly_right.getValueAt(la) - m_poly_left.getValueAt(la)) / m_pixel_to_m;

}

// Publish messages
void LaneDetector::publishMessages(bool is_wp_set) {

    // Publish waypoint
    if(is_wp_set)
        m_waypoint_pub.publish(m_next_waypoint);
    // Publish detection result
    if(m_publish_img_detection) {
        cv_bridge::CvImagePtr cv_ptr_d(new cv_bridge::CvImage);
        cv_ptr_d->encoding = "bgr8";
        cv_ptr_d->header.stamp = m_img_input_time;
        cv_ptr_d->header.frame_id = "front_center_camera";
        cv_ptr_d->image = m_img_input;
        m_detect_image_pub.publish(cv_ptr_d->toImageMsg());
    }
    // Publish debug image
    if(m_publish_img_debug) {
        cv_bridge::CvImagePtr cv_ptr_debug(new cv_bridge::CvImage);
        cv_ptr_debug->encoding = "bgr8";
        cv_ptr_debug->header.stamp = m_img_input_time;
        cv_ptr_debug->header.frame_id = "front_center_camera";
        cv_ptr_debug->image = m_img_hist;
        m_hist_image_pub.publish(cv_ptr_debug->toImageMsg());
        cv_ptr_debug->image = m_img_warped;
        m_warped_image_pub.publish(cv_ptr_debug->toImageMsg());
    }
    // Publish lane lines for visualization
    m_lane_lines.header.stamp = m_img_input_time;
    m_lane_lines_pub.publish(m_lane_lines);

}

// Callback current velocity
void LaneDetector::callbackVelocity(const geometry_msgs::TwistStampedConstPtr& msg) {

    m_current_velocity = msg->twist.linear.x;
    m_is_velocity_set = true;

}

// Callback image
void LaneDetector::callbackImage(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cvImg = cv_bridge::toCvCopy(msg, "bgr8");
    m_img_input = cvImg->image;
    m_img_input_time = msg->header.stamp;
    m_is_image_set = true;

}

// Callback dynamic reconfigure
void LaneDetector::callbackParameters(lane_detection::LaneDetectorConfig &config, uint32_t level) {

    cv::Point2f src_p[4];
    cv::Point2f dst_p[4];

    // Generall settings
    m_frequency = config.frequency;
    m_loop_rate = ros::Rate(m_frequency);
    m_display_information = config.display_information;

    m_look_ahead_min = config.look_ahead_min_distance;
    m_look_ahead_max = config.look_ahead_max_distance;
    m_look_ahead_ratio = config.look_ahead_ratio;
    m_multi_lane_detection = config.multi_lane_detection;
    m_polynom_degree = config.polynom_degree;
    m_pixel_to_m = config.pixel_to_m;
    m_sw[0] = config.sliding_window_width;
    m_sw[1] = config.sliding_window_height;
    m_hist_limit = config.histogramm_limit;
    m_color_thresh = config.color_threshold;

    // CV transform settings
    m_calib_path = config.calibration_image_path;
    m_save_img = config.save_calibration_image;
    config.save_calibration_image = false;
    m_baselink_to_img = config.baselink_camera_distance + 
                        (config.camera_height / tan((config.camera_fov_vertical/2 + config.camera_rotation_vertical) 
                        * PI / 180.0));

    // Source points    
    src_p[0] = cv::Point2f(config.src_pt1_x, config.src_pt1_y);
    src_p[1] = cv::Point2f(config.src_pt2_x, config.src_pt2_y);
    src_p[2] = cv::Point2f(config.src_pt3_x, config.src_pt3_y);
    src_p[3] = cv::Point2f(config.src_pt4_x, config.src_pt4_y);    
    // Destination points
    double y_1, y_2;
    if(m_look_ahead_max < config.plane_length) {
        y_1 = config.plane_length;
        y_2 = 0.0;
    }
    else {
        y_1 = m_look_ahead_max;
        y_2 = m_look_ahead_max - config.plane_length;
    }
    dst_p[0].x = config.padding_width * m_pixel_to_m;
    dst_p[0].y = y_1 * m_pixel_to_m;
    dst_p[1].x = (config.plane_width + config.padding_width) * m_pixel_to_m;
    dst_p[1].y = dst_p[0].y;
    dst_p[2].x = dst_p[1].x;
    dst_p[2].y = y_2 * m_pixel_to_m;
    dst_p[3].x = dst_p[0].x;
    dst_p[3].y = dst_p[2].y;
    // Warp data
    m_warp_mat = cv::getPerspectiveTransform(src_p, dst_p);
    m_warp_mat_inv = cv::getPerspectiveTransform(dst_p, src_p);
    m_warp_size = cv::Size(
        (config.plane_width + 2 * config.padding_width) * m_pixel_to_m, 
        y_1 * m_pixel_to_m
    );
    m_lane_lines.pixel_to_m = m_pixel_to_m;
    m_lane_lines.min_look_ahead = m_baselink_to_img;
    m_lane_lines.max_look_ahead = m_look_ahead_max;
    m_lane_lines.img_width = m_warp_size.width;
    m_lane_lines.img_height = m_warp_size.height;

}

// Callback visual dynamic reconfigure parameters from lane_visualizer
void LaneDetector::callbackVisualParameters(const std_msgs::BoolConstPtr& msg) {
    
    int color_right[3], color_left[3], color_other[3], color_wp[3];

    m_nh.getParam("/lane_visualizer/publish_debug_images", m_publish_img_debug);
    m_nh.getParam("/lane_visualizer/publish_detection_image", m_publish_img_detection);
    m_nh.getParam("/lane_visualizer/draw_lane_lines", m_draw_lane_lines);
    m_nh.getParam("/lane_visualizer/draw_waypoint", m_draw_waypoint);
    m_nh.getParam("/lane_visualizer/line_thickness", m_draw_thickness);
    m_nh.getParam("/lane_visualizer/waypoint_radius", m_waypoint_radius);

    m_nh.getParam("/lane_visualizer/color_r_r", color_right[0]);
    m_nh.getParam("/lane_visualizer/color_g_r", color_right[1]);
    m_nh.getParam("/lane_visualizer/color_b_r", color_right[2]);
    m_nh.getParam("/lane_visualizer/color_r_l", color_left[0]);
    m_nh.getParam("/lane_visualizer/color_g_l", color_left[1]);
    m_nh.getParam("/lane_visualizer/color_b_l", color_left[2]);
    m_nh.getParam("/lane_visualizer/color_r_o", color_other[0]);
    m_nh.getParam("/lane_visualizer/color_g_o", color_other[1]);
    m_nh.getParam("/lane_visualizer/color_b_o", color_other[2]);
    m_nh.getParam("/lane_visualizer/color_r_wp", color_wp[0]);
    m_nh.getParam("/lane_visualizer/color_g_wp", color_wp[1]);
    m_nh.getParam("/lane_visualizer/color_b_wp", color_wp[2]);

    m_color_right_line = cv::Scalar(color_right[2], color_right[1], color_right[0]);
    m_color_left_line = cv::Scalar(color_left[2], color_left[1], color_left[0]);
    m_color_other_line =  cv::Scalar(color_other[2], color_other[1], color_other[0]);
    m_color_waypoint = cv::Scalar(color_wp[2], color_wp[1], color_wp[0]);

}
