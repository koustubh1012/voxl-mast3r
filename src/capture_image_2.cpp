#include "capture_image_2.hpp"

CaptureImage2::CaptureImage2() 
    : Node("capture_image_2"), image_count_(0)
{
    // Initialize parameters
    this->declare_parameter<std::string>("save_directory", "/home/koustubh/test");
    this->declare_parameter<std::string>("camera_topic", "/hires_small_color");

    this->get_parameter("save_directory", save_directory_);
    this->get_parameter("camera_topic", camera_topic_);

    // Create subscribers
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_, 10, std::bind(&CaptureImage2::imageCallback, this, std::placeholders::_1));
    odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", 10, std::bind(&CaptureImage2::odometryCallback, this, std::placeholders::_1));
}

CaptureImage2::~CaptureImage2()
{
    // Destructor
}

void CaptureImage2::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Convert ROS image message to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    // Save the image with the current timestamp and odometry data
    std::string filename = save_directory_ + "/image_" + std::to_string(image_count_) + ".jpg";
    saveImage(cv_ptr->image, filename);
    image_count_++;
}

void CaptureImage2::odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    // Store the last odometry message
    last_odometry_ = *msg;
    odometry_received_ = true;
}

void CaptureImage2::saveImage(const cv::Mat &image, const std::string &filename)
{
    // Save the image to the specified directory
    if (!cv::imwrite(filename, image)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save image: %s", filename.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Saved image: %s", filename.c_str());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CaptureImage2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}