#pragma once

#include "../include/monobehaviour.hpp"
#include <rcl/publisher.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>

namespace engineering_match::image {

class ImageProcesser : public base::IMonoBehaviour {
public:
    ImageProcesser()
        : base::IMonoBehaviour("ImageProcesser") {};

    int count = 0;
    void start() override
    {
        // Initialization code for image processing
        // image_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        //     "/unity/imu_data", 10);
        RCLCPP_INFO(rclcpp::get_logger("ImageProcesser"), "ImageProcesser started.");
    };

    void update() override
    {
        count++;
        // Code to process images
        // std_msgs::msg::Float32MultiArray msg;
        // msg.data = { count };
        // image_publisher_->publish(msg);
    };

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr image_publisher_;
};
}