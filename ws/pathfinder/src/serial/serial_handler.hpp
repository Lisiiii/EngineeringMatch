#pragma once

#include "../include/monobehaviour.hpp"
#include "serial_defination.hpp"
#include <memory>

#include <rclcpp/subscription.hpp>
#include <vector>

#include <serial/serial.h>

#define DEBUG_MODE

namespace engineering_match::SerialHandler {

class SerialHandler : public base::IMonoBehaviour {
public:
    SerialHandler(const std::string& name, float update_rate)
        : base::IMonoBehaviour(name, update_rate) {};

    void start() override
    {
        for (int i = 0; i < 5; i++) {
            try {
                serial_instance_ = std::make_unique<serial::Serial>(
                    "/dev/ttyUSB0", 115200,
                    serial::Timeout::simpleTimeout(50U),
                    serial::eightbits,
                    serial::parity_none,
                    serial::stopbits_one,
                    serial::flowcontrol_none);
                if (serial_instance_->isOpen()) {
                    RCLCPP_INFO(get_logger(), "[+] Serial port opened successfully!");
                }
                break;
            } catch (const serial::IOException& se) {
                RCLCPP_ERROR(get_logger(), "[x] Can't open serial port!");
                RCLCPP_ERROR(get_logger(), "%s", se.what());
            }
            sleep(1);
        }

        velocity_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/respi/velocity", 10,
            std::bind(&SerialHandler::on_velocity_received, this, std::placeholders::_1));
        rotation_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/respi/palstance", 10,
            std::bind(&SerialHandler::on_rotation_received, this, std::placeholders::_1));
        command_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/respi/command", 10,
            std::bind(&SerialHandler::on_command_received, this, std::placeholders::_1));
        response_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/respi/response", 10);
        RCLCPP_INFO(this->get_logger(), "started.");
    };

    void update() override
    {
        std::vector<float> data_to_send = { command_id_, current_pos_rot_[0], current_pos_rot_[1], current_pos_rot_[2] };
        publish_to_serial(data_to_send);
        serial_package::PackageToReceive package;
        if (read_from_serial(package)) {
            response_id_ = static_cast<float>(package.command_id);
        }
    };

private:
    bool publish_to_serial(const std::vector<float>& data)
    {
        if (serial_instance_ == nullptr) {
            RCLCPP_ERROR(get_logger(), "[x] Serial instance is not initialized!");
            return false;
        }
        if (!serial_instance_->isOpen()) {
            RCLCPP_ERROR(get_logger(), "[x] Serial port is not open!");
            return false;
        }

        serial_package::PackageToSend package;
        package.command_id = data[0];
        package.move_data[0] = data[1];
        package.move_data[1] = data[2];
        package.move_data[2] = data[3];

        try {
            serial_instance_->write(
                reinterpret_cast<const uint8_t*>(&package),
                sizeof(package));

            return true;
        } catch (const serial::IOException& se) {
            RCLCPP_ERROR(get_logger(), "[x] Can't write to serial port!");
            RCLCPP_ERROR(get_logger(), "%s", se.what());
            return false;
        }
    };

    bool read_from_serial(serial_package::PackageToReceive& package)
    {
        if (serial_instance_ == nullptr) {
            RCLCPP_ERROR(get_logger(), "[x] Serial instance is not initialized!");
            return false;
        }
        if (!serial_instance_->isOpen()) {
            RCLCPP_ERROR(get_logger(), "[x] Serial port is not open!");
            return false;
        }

        try {
            if (serial_instance_->available() < sizeof(package)) {
                return false;
            }
            serial_instance_->read(
                reinterpret_cast<uint8_t*>(&package),
                sizeof(package));

            return true;
        } catch (const serial::IOException& se) {
            RCLCPP_ERROR(get_logger(), "[x] Can't read from serial port!");
            RCLCPP_ERROR(get_logger(), "%s", se.what());
            return false;
        }
    };
    void on_velocity_received(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        current_pos_rot_[0] = msg->data[0];
        current_pos_rot_[1] = msg->data[1];
        RCLCPP_INFO(this->get_logger(), "Velocity received: [%f, %f]", current_pos_rot_[0], current_pos_rot_[1]);
    }
    void on_rotation_received(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        current_pos_rot_[2] = msg->data[0];
        RCLCPP_INFO(this->get_logger(), "Rotation received: %f", current_pos_rot_[2]);
    }
    void on_command_received(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        command_id_ = msg->data[0];
    }

    std::shared_ptr<serial::Serial> serial_instance_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rotation_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr command_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr response_publisher_;
    float current_pos_rot_[3] = { 0.0f, 0.0f, 0.0f };
    float command_id_ = 0.0f;
    float response_id_ = 0.0f;
};
}