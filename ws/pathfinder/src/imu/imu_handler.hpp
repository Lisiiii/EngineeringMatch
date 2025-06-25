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
    IMUHandler(const std::string& name, float update_rate)
        : base::IMonoBehaviour(name, update_rate)
    {
        start();
    };

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
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/unity/imu_data", 10,
            std::bind(&IMUHandler::imu_data_callback, this, std::placeholders::_1));
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/unity/target_position", 10,
            std::bind(&IMUHandler::target_position_callback, this, std::placeholders::_1));
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/unity/velocity_data", 10);

        RCLCPP_INFO(this->get_logger(), "started.");
    };

    void update() override
    {
        std_msgs::msg::Float32MultiArray msg;
        std::vector<float> velocity = { (target_position_[0] - current_position_[0]),
            (target_position_[1] - current_position_[1]) };

        msg.data = { velocity[0], velocity[1] };
        velocity_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published velocity: [%f, %f]", velocity[0], velocity[1]);
    };

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_publisher_;
    std::vector<float> current_position_ = { 0.0f, 0.0f };
    std::vector<float> target_position_ = { 0.0f, 0.0f };
};

}