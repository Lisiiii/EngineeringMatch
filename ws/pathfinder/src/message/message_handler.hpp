#pragma once

#include "../include/monobehaviour.hpp"
#include <memory>
#include <opencv2/core/matx.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <opencv2/opencv.hpp>
#include <vector>

namespace engineering_match::data::message {

class MessageHandler : public base::IMonoBehaviour {
public:
    MessageHandler(const std::string& name, float update_rate)
        : base::IMonoBehaviour(name, update_rate) {};

    enum class MsgType {
        IMU_DATA,
        TARGET_POSITION,
        VELOCITY_DATA,
    };

    std::vector<float> current_position_ = { 0.0f, 0.0f };
    std::vector<float> target_position_ = { 0.0f, 0.0f };

    void start() override
    {
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/unity/imu_data", 10,
            std::bind(&MessageHandler::imu_data_callback, this, std::placeholders::_1));
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/unity/target_position", 10,
            std::bind(&MessageHandler::target_position_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/unity/velocity_data", 10);

        RCLCPP_INFO(this->get_logger(), "started.");
    };

    bool publish_to_topic(MsgType type, const std::vector<float>& data)
    {
        switch (type) {
        case MsgType::IMU_DATA: {
            std_msgs::msg::Float32MultiArray msg;
            msg.data = data;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published IMU data: [%f, %f, %f]", data[0], data[1], data[2]);
            return true;
        }
        case MsgType::TARGET_POSITION: {
            std_msgs::msg::Float32MultiArray msg;
            msg.data = data;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published target position: [%f, %f]", data[0], data[1]);
            return true;
        }
        case MsgType::VELOCITY_DATA:
            std_msgs::msg::Float32MultiArray velocity_msg;
            velocity_msg.data = data;
            publisher_->publish(velocity_msg);
            return true;
        }
    }

    bool publish_to_serial(const std::vector<float>& velocity_data)
    {
        // TODO
        return false;
    };

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    void imu_data_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("IMUHandler"), "Received IMU data: [%f, %f, %f]",
            msg->data[0], msg->data[1], msg->data[2]);
        // Process the IMU data and update current position
        current_position_ = std::vector<float> { msg->data[0], msg->data[1] };
    };

    void target_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // RCLCPP_INFO(rclcpp::get_logger("IMUHandler"), "Received target position: [%f, %f]",
        // msg->data[0], msg->data[1]);
        // Update the target position
        // target_position_ = std::vector<float> { msg->data[0], msg->data[1] };
    };
};
}