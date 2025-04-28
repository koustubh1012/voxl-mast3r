#include "capture_image_2.hpp"

CaptureImage2::CaptureImage2() 
    : Node("capture_image_2"), image_count_(0)
{
    // Initialize parameters
    this->declare_parameter<std::string>("save_directory", "/home/koustubh/test");
    this->declare_parameter<std::string>("camera_topic", "/hires_small_color");

    this->get_parameter("save_directory", save_directory_);
    this->get_parameter("camera_topic", camera_topic_);

    // Set qos
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    qos.history(rclcpp::HistoryPolicy::KeepLast);

    // Create subscribers
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_, 10, std::bind(&CaptureImage2::imageCallback, this, std::placeholders::_1));
    odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos, std::bind(&CaptureImage2::odometryCallback, this, std::placeholders::_1));

    // Initialize odometry data
    odometry_received_ = false;
}

CaptureImage2::~CaptureImage2()
{
    // Destructor
}

void CaptureImage2::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}

void CaptureImage2::odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    // Store the first odometry message
    if(!odometry_received_) {
        last_odometry_ = *msg;
        odometry_received_ = true;
        RCLCPP_INFO(this->get_logger(), "First odometry data received: %f, %f, %f", 
                    msg->position[0], msg->position[1], msg->position[2]);
    }
    // Check if the drone has moved more than the distance threshold
    distance = sqrt(pow(msg->position[0] - last_odometry_.position[0], 2) +
                     pow(msg->position[1] - last_odometry_.position[1], 2) +
                     pow(msg->position[2] - last_odometry_.position[2], 2));
    // RCLCPP_INFO(this->get_logger(), "Distance: %f", distance);
    if (distance > DISTANCE_THRESHOLD) {
        // Save the image with the current timestamp and odometry data
        std::string filename = save_directory_ + "/image_" + std::to_string(image_count_) + ".jpg";
        saveImage(cv_ptr->image, filename);
        last_odometry_ = *msg; // Update the last odometry data
        image_count_++;
    }
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