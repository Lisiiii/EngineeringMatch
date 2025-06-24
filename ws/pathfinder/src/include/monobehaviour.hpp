#pragma once

#include <rclcpp/executors.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>

namespace engineering_match::base {

class IMonoBehaviour : public rclcpp::Node {
public:
    IMonoBehaviour(const std::string& node_name)
        : rclcpp::Node(node_name) {};
    virtual ~IMonoBehaviour() = default;
    virtual void start() {};
    virtual void update() {};

    rclcpp::TimerBase::SharedPtr update_timer_;
};
}