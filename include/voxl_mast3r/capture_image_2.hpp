/**
 * @file capture_image_2.hpp
 * @brief Header file for the CaptureImage2 class.
 * This class is responsible for capturing images based on drone trajectory from a camera and saving them to a specified directory.
 * @author FNU Koustubh koustubh@umd.edu
 * @author Keyur Borad kborad@umd.edu 
 * @date 2025-04-27
 */

#ifndef CAPTURE_IMAGE_2_HPP
#define CAPTURE_IMAGE_2_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class CaptureImage2 : public rclcpp::Node
{
public:
    CaptureImage2();
    ~CaptureImage2();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void saveImage(const cv::Mat &image, const std::string &filename);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber_;
    std::string save_directory_;
    int image_count_;
    bool odometry_received_;
    px4_msgs::msg::VehicleOdometry last_odometry_;
    std::string camera_topic_;
};

#endif