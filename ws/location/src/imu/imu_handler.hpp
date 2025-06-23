#pragma once

#include "../include/monobehaviour.hpp"
#include <opencv2/core/matx.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <opencv2/opencv.hpp>
#include <vector>

namespace engineering_match::imu {

class IMUHandler : public base::IMonoBehaviour {
public:
    IMUHandler()
        : base::IMonoBehaviour("IMUHandler") {};

    void imu_data_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("IMUHandler"), "Received IMU data: [%f, %f, %f]",
            msg->data[0], msg->data[1], msg->data[2]);
        // Process the IMU data and update current position
        current_position_ = std::vector<float> { msg->data[0], msg->data[1] };
    };

    void target_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("IMUHandler"), "Received target position: [%f, %f]",
            msg->data[0], msg->data[1]);
        // Update the target position
        target_position_ = std::vector<float> { msg->data[0], msg->data[1] };
    };

    void start() override
    {
        // Initialization code for IMU handling
        RCLCPP_INFO(rclcpp::get_logger("IMUHandler"), "IMUHandler started.");
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/unity/velocity_data", 10);
    };

    void update() override
    {
        std_msgs::msg::Float32MultiArray msg;
        std::vector<float> velocity = { (target_position_[0] - current_position_[0]),
            (target_position_[1] - current_position_[1]) };

        msg.data = { velocity[0], velocity[1] };
        velocity_publisher_->publish(msg);
        RCLCPP_INFO(rclcpp::get_logger("IMUHandler"), "Published velocity: [%f, %f]", velocity[0], velocity[1]);
    };

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_publisher_;
    std::vector<float> current_position_ = { 0.0f, 0.0f };
    std::vector<float> target_position_ = { 0.0f, 0.0f };
};

}