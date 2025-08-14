#pragma once

#include "../include/monobehaviour.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace engineering_match::message {

class MessageHandler : public base::IMonoBehaviour {
public:
    MessageHandler(const std::string& name, float update_rate)
        : base::IMonoBehaviour(name, update_rate) {};
    void start() override
    {
        imu_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/unity/imu_data", 10,
            std::bind(&MessageHandler::imu_data_callback, this, std::placeholders::_1));
        position_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/respi/position", 10);
        rotation_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/respi/rotation", 10);

        RCLCPP_INFO(this->get_logger(), "started.");
    };

    void update() override
    {
        std_msgs::msg::Float32MultiArray position_msg;
        position_msg.data = current_position_;
        position_publisher_->publish(position_msg);

        std_msgs::msg::Float32MultiArray rotation_msg;
        rotation_msg.data = { rotation_ };
        rotation_publisher_->publish(rotation_msg);
    };

private:
    void imu_data_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // RCLCPP_INFO(rclcpp::get_logger("IMUHandler"), "Received IMU data: [%f, %f, %f]",
        //     msg->data[0], msg->data[1], msg->data[2]);
        // Process the IMU data and update current position
        current_position_ = std::vector<float> { msg->data[0], msg->data[1] };
        rotation_ = msg->data[2];
    };

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rotation_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr imu_subscription_;

    std::vector<float> current_position_ = { 0.0f, 0.0f };
    std::vector<float> target_position_ = { 0.0f, 0.0f };
    float rotation_ = 0.0f;
};
}