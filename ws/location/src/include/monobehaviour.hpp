#pragma once

#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <vector>

namespace engineering_match::base {

class IMonoBehaviour : public rclcpp::Node {
public:
    IMonoBehaviour(const std::string& node_name)
        : rclcpp::Node(node_name)
    {
    }
    virtual ~IMonoBehaviour() = default;
    virtual void start() { }
    virtual void update() { }
};

class ObjectManager : public rclcpp::Node {
private:
    std::vector<std::shared_ptr<IMonoBehaviour>> managed_objects_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    float update_rate_hz_ = 20.0f;

    void start_all()
    {
        RCLCPP_INFO(this->get_logger(), "- [Starting]Starting all managed objects...");
        for (auto& obj : managed_objects_) {
            obj->start();
        }

        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / update_rate_hz_)),
            std::bind(&ObjectManager::update_all, this));
    }
    void update_all()
    {
        for (auto& obj : managed_objects_) {
            obj->update();
        }
    }
    void clear()
    {
        managed_objects_.clear();
    }

public:
    ObjectManager(const std::string& node_name = "ObjectManager", const float update_rate_hz = 20.0f)
        : rclcpp::Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "- ObjectManager initialized with update rate: %f Hz", update_rate_hz);
        update_rate_hz_ = update_rate_hz;
    }

    void start()
    {
        start_all();
    }

    ~ObjectManager()
    {
        clear();
    }

    template <typename T, typename... Args>
    void instantiate(Args&&... args)
    {
        static_assert(std::is_base_of_v<IMonoBehaviour, T>,
            "T must inherit from IManagedObject");

        auto obj = std::make_shared<T>(std::forward<Args>(args)...);
        managed_objects_.push_back(obj);

        RCLCPP_INFO(this->get_logger(), "-- [Instantiated] Object of type: %s",
            typeid(T).name());
    }

    template <typename MsgType, typename CallbackClass>
    void subscribe(const std::string& topic_name,
        void (CallbackClass::*callback)(const typename MsgType::SharedPtr))
    {
        static_assert(std::is_base_of_v<rclcpp::Node, CallbackClass>,
            "CallbackClass must inherit from rclcpp::Node");
        // get the class instance in managed_objects_
        CallbackClass* callback_class = nullptr;
        for (const auto& obj : managed_objects_) {
            if (auto derived = dynamic_cast<CallbackClass*>(obj.get())) {
                callback_class = derived;
                break;
            }
        }
        if (!callback_class) {
            RCLCPP_ERROR(this->get_logger(), "Callback class not found in managed objects.");
            return;
        }
        auto subscription = this->create_subscription<MsgType>(
            topic_name, 10,
            std::bind(callback, callback_class, std::placeholders::_1));
        subscriptions_.push_back(subscription);
        RCLCPP_INFO(this->get_logger(), "-- [Subscribed] to topic: %s", topic_name.c_str());
    }
};
} // namespace engineering_match::base