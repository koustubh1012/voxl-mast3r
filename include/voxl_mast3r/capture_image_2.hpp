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

/**
 * @brief Class to capture images from a camera and save them based on drone trajectory.
 * This class subscribes to the camera topic and the vehicle odometry topic.
 * It saves the images to a specified directory with a filename based on the image count.
 */
class CaptureImage2 : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the CaptureImage2 class.
     * Initializes the node, parameters, and subscribers.
     */
    CaptureImage2();
    /**
     * @brief Destructor for the CaptureImage2 class.
     * Cleans up resources and stops the node.
     */
    ~CaptureImage2();

private:
    /**
     * @brief Callback function for the camera image topic.
     * This function is called when a new image is received.
     * It converts the ROS image message to OpenCV format and saves it to the specified directory.
     * @param msg The received image message.
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief Callback function for the vehicle odometry topic.
     * This function is called when a new odometry message is received.
     * @param msg The received odometry message.
     */
    void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    /**
     * @brief Saves the image to the specified directory.
     * @param image The image to be saved.
     * @param filename The filename for the saved image.
     */
    void saveImage(const cv::Mat &image, const std::string &filename);

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber_;
    std::string save_directory_;
    int image_count_;
    bool odometry_received_;
    px4_msgs::msg::VehicleOdometry last_odometry_;
    std::string camera_topic_;
};

#endif