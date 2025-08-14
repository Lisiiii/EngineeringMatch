#pragma once

#include "../image_process/image_processer.hpp"
#include "../include/monobehaviour.hpp"

#include "state_defines.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <functional>
#include <memory>
#include <numbers>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription_base.hpp>
#include <unordered_map>
#include <vector>

namespace engineering_match::state_machine {

class StateMachine : public base::IMonoBehaviour {
public:
    StateMachine(const std::string& name, float update_rate, std::shared_ptr<image::ImageProcesser> image_processer)
        : base::IMonoBehaviour(name, update_rate)
        , image_processer_(image_processer) {};

    void start() override
    {
        initialize();

        position_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/respi/position", 10,
            std::bind(&StateMachine::on_position_received, this, std::placeholders::_1));
        rotation_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/respi/rotation", 10,
            std::bind(&StateMachine::on_rotation_received, this, std::placeholders::_1));
        response_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/respi/response", 10,
            std::bind(&StateMachine::on_response_received, this, std::placeholders::_1));
        velocity_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/respi/velocity", 10);
        palstance_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/respi/palstance", 10);
        command_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/respi/command", 10);

        forward_clamped_colors_ = {};
        transitionTo(StageType::STARTUP);
    }

    void update() override
    {
        update_environment();
        if (stages_[current_stage_].update == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "No update function defined for stage: %d", static_cast<int>(current_stage_));
            return;
        }
        stages_[current_stage_].update();
    }

private:
    void initialize();

    void transitionTo(StageType new_stage)
    {
        RCLCPP_INFO(this->get_logger(), "Transitioning to stage: %d", static_cast<int>(current_stage_));
        if (stages_[current_stage_].exit == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "No exit function defined for stage: %d", static_cast<int>(current_stage_));
            return;
        }
        stages_[current_stage_].exit();
        current_stage_ = new_stage;
        if (stages_[current_stage_].enter == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "No enter function defined for stage: %d", static_cast<int>(current_stage_));
            return;
        }
        stages_[current_stage_].enter();
    }

    void update_environment()
    {
        std_msgs::msg::Float32MultiArray command_msg;
        std_msgs::msg::Float32MultiArray velocity_msg;
        std_msgs::msg::Float32MultiArray palstance_msg;
        command_msg.data = { command_id_ };
        velocity_msg.data = { velocity_.x, velocity_.y };
        palstance_msg.data = { palstance_ };
        command_publisher_->publish(command_msg);
        velocity_publisher_->publish(velocity_msg);
        palstance_publisher_->publish(palstance_msg);

        RCLCPP_INFO(this->get_logger(), "[Updating environment][stage-%d]", static_cast<int>(current_stage_));
    }

    float distance(const cv::Point2f& point1, const cv::Point2f& point2)
    {
        return cv::norm(point1 - point2);
    }

    void on_position_received(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        current_location_ = cv::Point2f(msg->data[0], msg->data[1]);
        RCLCPP_INFO(this->get_logger(), "Current location updated to: (%f, %f)", current_location_.x, current_location_.y);
    }

    void on_rotation_received(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        current_rotation_angle_ = msg->data[0];
    }

    void on_response_received(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        response_id_ = msg->data[0];
        RCLCPP_INFO(this->get_logger(), "Response ID updated to: %f", response_id_);
    }

private:
    StageType current_stage_ = StageType::STARTUP;
    cv::Point2f current_location_ = { 0.0f, 0.0f };
    cv::Point2f velocity_ = { 0.0f, 0.0f };
    float palstance_ = 0;
    float current_rotation_angle_ = 0.0f;
    float command_id_ = 0.0f;
    float response_id_ = 0.0f;

    std::deque<image::Colors::ColorType> forward_clamped_colors_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rotation_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr response_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr palstance_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_publisher_;

    std::shared_ptr<image::ImageProcesser> image_processer_;

    enum WorkingState {
        NOTSTART,
        WORKING,
        COMPLETED,
    };

    WorkingState place_state_
        = NOTSTART;
    WorkingState clamp_state_
        = NOTSTART;
    WorkingState rotate_state_
        = NOTSTART;

    struct Stage {
        int current_stage = 0;
        std::vector<int> execute_step;
        std::vector<std::function<bool()>> stages;
        std::function<void()> enter;
        std::function<void()> update;
        std::function<void()> exit;
    };

    struct Behavior {
        std::function<bool(DirectionType clamp_direction)> clamping;
        std::function<bool(DirectionType move_direction, cv::Point2f target)> moving;
        std::function<bool(DirectionType place_direction)> placing;
        std::function<bool(DeviceType device)> rotating;
        std::function<void()> idle;
    };

    Behavior behaviors_;
    std::unordered_map<StageType, Stage> stages_;
};

void inline StateMachine::initialize()
{
    // initialize behaviors
    behaviors_ = {
        [this](DirectionType clamp_direction) {
            switch (clamp_state_) {
            case NOTSTART: {
                RCLCPP_INFO(this->get_logger(), "Clamping started.");
                // clamping action
                if (clamp_direction == DirectionType::FORWARD) {
                    image::Colors::ColorType front_color = image::Colors::ColorType::CANTIDTIFY;
                    while (front_color == image::Colors::ColorType::CANTIDTIFY) {
                        front_color = image_processer_->read_and_get_color();
                    }
                    forward_clamped_colors_.push_back(front_color);
                }
                command_id_ = static_cast<float>((clamp_direction == DirectionType::FORWARD) ? CommandType::FCLAMP : CommandType::BCLAMP);
                clamp_state_ = WORKING;
                return false;
            }
            case WORKING:
                if (response_id_ == static_cast<float>((clamp_direction == DirectionType::FORWARD) ? CommandType::FCLAMP : CommandType::BCLAMP)) {
                    clamp_state_ = COMPLETED;
                    command_id_ = 0.0f;
                }
                return false;
            case COMPLETED:
                RCLCPP_INFO(this->get_logger(), "Already clamped, no action needed.");
                clamp_state_ = NOTSTART; // Reset state after completion
                return true;
            }
            return false;
        },
        [this](DirectionType move_direction, cv::Point2f target) {
            RCLCPP_INFO(this->get_logger(), "Moving in direction: %s to target: %f, %f",
                move_direction == DirectionType::FORWARD ? "FORWARD" : "BACKWARD",
                target.x, target.y);

            cv::Point2f direction = target - current_location_;
            float cur_rot_rad = (current_rotation_angle_ + 90.0f) / 180.0f * std::numbers::pi;
            cv::Point2f current_dir = { std::cos(cur_rot_rad),
                std::sin(cur_rot_rad) };

            float rotate = current_dir.x * direction.y - direction.x * current_dir.y;
            if (rotate > 0.01)
                palstance_ = 1;
            else if (rotate < -0.01)
                palstance_ = -1;
            else
                palstance_ = 0;

            if (distance(current_location_, target) < 0.1f) {
                RCLCPP_INFO(this->get_logger(), "Already at target location.");
                velocity_ = { 0.0f, 0.0f };
                return true;
            } else if (palstance_ != 0) {
                velocity_ = { 0.0f, 0.0f };
                return false;
            } else {
                velocity_ = (target - current_location_) * 0.1f;
            }
            return false;
        },
        [this](DirectionType place_direction) {
            switch (place_state_) {
            case NOTSTART:
                RCLCPP_INFO(this->get_logger(), "Placing started.");
                // placing action
                command_id_ = static_cast<float>((place_direction == DirectionType::FORWARD) ? CommandType::FPLACE : CommandType::BPLACE);
                place_state_ = WORKING;
                return false;
            case WORKING:
                if (response_id_ == static_cast<float>((place_direction == DirectionType::FORWARD) ? CommandType::FPLACE : CommandType::BPLACE)) {
                    place_state_ = COMPLETED;
                    command_id_ = 0.0f;
                    if (place_direction == DirectionType::FORWARD) {
                        forward_clamped_colors_.pop_front();
                    }
                }
                return false;
            case COMPLETED:
                RCLCPP_INFO(this->get_logger(), "Already placed, no action needed.");
                place_state_ = NOTSTART; // Reset state after completion
                return true;
            }
            return false;
        },
        [this](DeviceType device) {
            RCLCPP_INFO(this->get_logger(), "Rotating device: %s",
                device == DeviceType::CAMERA ? "CAMERA" : "CAR");
            switch (rotate_state_) {
            case NOTSTART:
                RCLCPP_INFO(this->get_logger(), "Rotation started.");
                // rotation action
                command_id_ = static_cast<float>((device == DeviceType::CAMERA) ? CommandType::ROTCAM : CommandType::ROTCAR);
                rotate_state_ = WORKING;
                return false;
            case WORKING:
                if (response_id_ == static_cast<float>((device == DeviceType::CAMERA) ? CommandType::ROTCAM : CommandType::ROTCAR)) {
                    rotate_state_ = COMPLETED;
                    command_id_ = 0.0f;
                }
                return false;
            case COMPLETED:
                RCLCPP_INFO(this->get_logger(), "Already rotated, no action needed.");
                rotate_state_ = NOTSTART; // Reset state after completion
                return true;
            }
            return false;
        },
        [this]() {
            RCLCPP_INFO(this->get_logger(), "Idle state.");
        }
    };

    // initialize stages
    stages_[StageType::STARTUP] = {
        0,
        {},
        {},
        [] {
        },
        [this] {
            behaviors_.idle();
            if (1)
                transitionTo(StageType::GET_A_C_E);
        },
        [] {

        }
    };
    stages_[StageType::GET_A_C_E] = {
        0,
        { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 },
        {
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.CENTERTARGET);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.APOINT);
            },
            [this] {
                return behaviors_.clamping(DirectionType::FORWARD);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.CENTERTARGET);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.CPOINT);
            },
            [this] {
                return behaviors_.clamping(DirectionType::FORWARD);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.CENTERTARGET);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.EPOINT);
            },
            [this] {
                return behaviors_.clamping(DirectionType::FORWARD);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.CENTERTARGET);
            },
        },

        [this] {
            stages_[StageType::GET_A_C_E].current_stage = 0;
        },
        [this] {
            auto& stages = stages_[StageType::GET_A_C_E].stages;
            auto& current_stage = stages_[StageType::GET_A_C_E].current_stage;
            auto& execute_step = stages_[StageType::GET_A_C_E].execute_step;

            auto result = static_cast<int>(stages[execute_step[current_stage]]());
            RCLCPP_INFO(this->get_logger(), "[Stage %d]Substage %d result: %d", static_cast<int>(current_stage_), current_stage, result);
            current_stage += result;
            if (current_stage >= static_cast<int>(execute_step.size())) {
                transitionTo(StageType::PLACE_A_C_E_POINT);
            }
        },
        [] {} // exit
    };
    stages_[StageType::PLACE_A_C_E_POINT] = {
        0,
        { 0, 1, 0, 1, 0, 1 },
        {
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, color_to_point_[forward_clamped_colors_.front()]);
            },
            [this] {
                return behaviors_.placing(DirectionType::FORWARD);
            },
        },

        [] {

        },
        [this] {
            auto& stages = stages_[StageType::PLACE_A_C_E_POINT].stages;
            auto& current_stage = stages_[StageType::PLACE_A_C_E_POINT].current_stage;
            auto& execute_step = stages_[StageType::PLACE_A_C_E_POINT].execute_step;

            auto result = static_cast<int>(stages[execute_step[current_stage]]());
            RCLCPP_INFO(this->get_logger(), "[Stage %d]Substage %d result: %d", static_cast<int>(current_stage_), current_stage, result);
            current_stage += result;
            if (current_stage >= static_cast<int>(execute_step.size())) {
                transitionTo(StageType::GET_AND_TEMPLY_PLACE_F_POINT);
            }
        },
        [] {}, // exit
    };
    stages_[StageType::GET_AND_TEMPLY_PLACE_F_POINT] = {
        0,

        // clang-format off
        { 0, 1, 2, 
            3, 1, 4, 6,5,
            3, 1, 4,6,5,
            3,1,4,6,5,
            3,1,4,6,5,
            3,1,4,6,5, },
        // clang-format on
        {

            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.FPOINT);
            },
            [this] {
                return behaviors_.rotating(DeviceType::CAR);
            },
            [this] {
                return behaviors_.clamping(DirectionType::BACKWARD);
            },
            [this] {
                return behaviors_.placing(DirectionType::BACKWARD);
            },
            [this] {
                return behaviors_.clamping(DirectionType::FORWARD);
            },
            [this] {
                return behaviors_.placing(DirectionType::FORWARD);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, color_to_point_[forward_clamped_colors_.front()]);
            },

        },

        [] {

        },
        [this] {
            auto& stages = stages_[StageType::GET_AND_TEMPLY_PLACE_F_POINT].stages;
            auto& current_stage = stages_[StageType::GET_AND_TEMPLY_PLACE_F_POINT].current_stage;
            auto& execute_step = stages_[StageType::GET_AND_TEMPLY_PLACE_F_POINT].execute_step;

            auto result = static_cast<int>(stages[execute_step[current_stage]]());
            RCLCPP_INFO(this->get_logger(), "[Stage %d]Substage %d result: %d", static_cast<int>(current_stage_), current_stage, result);
            current_stage += result;
            if (current_stage >= static_cast<int>(execute_step.size())) {
                transitionTo(StageType::GET_AND_TEMPLY_PLACE_F_POINT);
            }
        },
        [] {}, // exit
    };
    stages_[StageType::GET_AND_TEMPLY_PLACE_G_POINT] = {
        0,

        // clang-format off
        { 0, 1, 2, 
            3, 1, 4, 6,5,
            3, 1, 4,6,5,
            3,1,4,6,5,
            3,1,4,6,5,
            3,1,4,6,5, },
        // clang-format on
        {

            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.GPOINT);
            },
            [this] {
                return behaviors_.rotating(DeviceType::CAR);
            },
            [this] {
                return behaviors_.clamping(DirectionType::BACKWARD);
            },
            [this] {
                return behaviors_.placing(DirectionType::BACKWARD);
            },
            [this] {
                return behaviors_.clamping(DirectionType::FORWARD);
            },
            [this] {
                return behaviors_.placing(DirectionType::FORWARD);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, color_to_point_[forward_clamped_colors_.front()]);
            },

        },

        [] {

        },
        [this] {
            auto& stages = stages_[StageType::GET_AND_TEMPLY_PLACE_G_POINT].stages;
            auto& current_stage = stages_[StageType::GET_AND_TEMPLY_PLACE_G_POINT].current_stage;
            auto& execute_step = stages_[StageType::GET_AND_TEMPLY_PLACE_G_POINT].execute_step;

            auto result = static_cast<int>(stages[execute_step[current_stage]]());
            RCLCPP_INFO(this->get_logger(), "[Stage %d]Substage %d result: %d", static_cast<int>(current_stage_), current_stage, result);
            current_stage += result;
            if (current_stage >= static_cast<int>(execute_step.size())) {
                transitionTo(StageType::COMPLETE_POINT);
            }
        },
        [] {}, // exit
    };
    stages_[StageType::COMPLETE_POINT] = {
        0,

        // clang-format off
        { 2, 0, 0,0,7, 1,
            3, 0, 0,0,8, 1,
            4, 0, 0,0,9, 1,
            5, 0, 0,0,10, 1,
             6, 0, 0,0,11, 1,},
        // clang-format on
        {

            [this] {
                return behaviors_.clamping(DirectionType::FORWARD);
            },
            [this] {
                return behaviors_.placing(DirectionType::FORWARD);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.APOINT);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.BPOINT);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.CPOINT);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.DPOINT);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.EPOINT);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.GREENTARGET);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.WHITETARGET);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.REDTARTGET);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.BLACKTARGET);
            },
            [this] {
                return behaviors_.moving(DirectionType::FORWARD, Locations.BLUETARGET);
            },

        },

        [] {

        },
        [this] {
            auto& stages = stages_[StageType::COMPLETE_POINT].stages;
            auto& current_stage = stages_[StageType::COMPLETE_POINT].current_stage;
            auto& execute_step = stages_[StageType::COMPLETE_POINT].execute_step;

            auto result = static_cast<int>(stages[execute_step[current_stage]]());
            RCLCPP_INFO(this->get_logger(), "[Stage %d]Substage %d result: %d", static_cast<int>(current_stage_), current_stage, result);
            current_stage += result;
            if (current_stage >= static_cast<int>(execute_step.size())) {
                transitionTo(StageType::IDLE);
            }
        },
        [] {}, // exit
    };
    stages_[StageType::IDLE] = {
        0,
        {},
        {},
        [] {
        },
        [this] {
            behaviors_.idle();
        },
        [] {
        }
    };
};
}
