#pragma once

#include "../include/monobehaviour.hpp"
#include <functional>
#include <rclcpp/logging.hpp>
#include <unordered_map>
#include <vector>

namespace engineering_match::pathfinder {

class StateMachine;

enum class LocationType {
    STARTUPPOINT,
    CENTERTARGET,
    APOINT,
    BPOINT,
    CPOINT,
    DPOINT,
    EPOINT,
    FPOINT,
    GPOINT,
    REDTARTGET,
    GREENTARGET,
    BLUETARGET,
    BLACKTARGET,
    WHITETARGET,
    SOLIDDISTANCE,
};

enum class DirectionType {
    FORWARD,
    BACKWARD,
};

enum class DeviceType {
    CAMERA,
    CAR
};

enum class ColorType {
    RED,
    GREEN,
    BLUE,
    WHITE,
    BLACK,
};

enum class StageType {
    STARTUP,
    GET_AND_PLACE_A_POINT,
    GET_AND_PLACE_C_POINT,
    GET_AND_PLACE_E_POINT,
    GET_AND_TEMPLY_PLACE_F_POINT,
    GET_AND_TEMPLY_PLACE_G_POINT,
    COMPLETE_A,
    COMPLETE_B,
    COMPLETE_C,
    COMPLETE_D,
    COMPLETE_E,
};

class StateMachine : public base::IMonoBehaviour {
public:
    StateMachine(const std::string& name, float update_rate)
        : base::IMonoBehaviour(name, update_rate) {};

    void start() override
    {
        initialize();

        current_location_ = LocationType::STARTUPPOINT;
        forward_clamped_colors_ = {};
        backward_clamped_colors_ = {};
        if_barrier_ = false;
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
    StageType current_stage_;
    LocationType current_location_;
    std::vector<ColorType> forward_clamped_colors_;
    std::vector<ColorType> backward_clamped_colors_;
    std::unordered_map<ColorType, LocationType> color_to_target_ = {
        { ColorType::RED, LocationType::REDTARTGET },
        { ColorType::GREEN, LocationType::GREENTARGET },
        { ColorType::BLUE, LocationType::BLUETARGET },
        { ColorType::WHITE, LocationType::WHITETARGET },
        { ColorType::BLACK, LocationType::BLACKTARGET }
    };
    std::unordered_map<ColorType, LocationType> color_to_point_ = {
        { ColorType::RED, LocationType::APOINT },
        { ColorType::GREEN, LocationType::BPOINT },
        { ColorType::BLUE, LocationType::CPOINT },
        { ColorType::WHITE, LocationType::DPOINT },
        { ColorType::BLACK, LocationType::EPOINT }
    };
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
    bool if_barrier_;

    struct Stage {
        int current_stage = 0;
        std::vector<std::function<int()>> stages;
        std::function<void()> enter;
        std::function<void()> update;
        std::function<void()> exit;
    };

    struct Behavior {
        std::function<bool(DirectionType clamp_direction, bool is_multiple)> clamping;
        std::function<bool(DirectionType move_direction, LocationType target)> moving;
        std::function<bool(DirectionType place_direction, int placing_count)> placing;
        std::function<bool(DeviceType device)> rotating;
        std::function<void()> idle;
    };

    Behavior behaviors_;
    std::unordered_map<StageType, Stage> stages_;

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
        // This function can be used to update the environment or state
        // before processing the current stage.
        clamp_state_ = COMPLETED;
        place_state_ = COMPLETED;
        rotate_state_ = COMPLETED;
        if_barrier_ = true; // Reset barrier state if needed
        // get a random location
        current_location_ = static_cast<LocationType>(rand() % static_cast<int>(LocationType::SOLIDDISTANCE) + 1);

        if (forward_clamped_colors_.empty()) {
            forward_clamped_colors_.emplace_back(ColorType::RED);
        }
        if (backward_clamped_colors_.empty()) {
            backward_clamped_colors_.emplace_back(ColorType::GREEN);
        }

        RCLCPP_INFO(this->get_logger(), "[Updating environment][stage-%d]", static_cast<int>(current_stage_));
    }
};

void inline StateMachine::initialize()
{
    // initialize behaviors
    behaviors_ = {
        [this](DirectionType clamp_direction, bool is_multiple) {
            RCLCPP_INFO(this->get_logger(), "Clamping in direction: %s, multiple: %s",
                clamp_direction == DirectionType::FORWARD ? "FORWARD" : "BACKWARD",
                is_multiple ? "true" : "false");
            switch (clamp_state_) {
            case NOTSTART:
                RCLCPP_INFO(this->get_logger(), "Clamping started.");
                // Simulate clamping action
            case WORKING:
                return false;
            case COMPLETED:
                RCLCPP_INFO(this->get_logger(), "Already clamped, no action needed.");
                clamp_state_ = NOTSTART; // Reset state after completion
                return true;
            }
            return false;
        },
        [this](DirectionType move_direction, LocationType target) {
            RCLCPP_INFO(this->get_logger(), "Moving in direction: %s to target: %d",
                move_direction == DirectionType::FORWARD ? "FORWARD" : "BACKWARD",
                static_cast<int>(target));
            if (current_location_ == target) {
                RCLCPP_INFO(this->get_logger(), "Already at target location.");
                return true;
            } else {
                // Simulate moving to the target location
                return false;
            }
            return false;
        },
        [this](DirectionType place_direction, int placing_count) {
            RCLCPP_INFO(this->get_logger(), "Placing in direction: %s, count: %d",
                place_direction == DirectionType::FORWARD ? "FORWARD" : "BACKWARD",
                placing_count);
            switch (place_state_) {
            case NOTSTART:
                RCLCPP_INFO(this->get_logger(), "Placing started.");
                // Simulate placing action
            case WORKING:
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
                // Simulate rotation action
            case WORKING:
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
        [this] {
            if (true) { // Replace with actual condition
                transitionTo(StageType::GET_AND_PLACE_A_POINT);
            }
        },
        [] {
        },
        [] {
        }
    };
    stages_[StageType::GET_AND_PLACE_A_POINT] = {
        0,
        {
            [this] { return static_cast<int>(behaviors_.moving(DirectionType::FORWARD, LocationType::APOINT)); },
            [this] {
                bool clamped = behaviors_.clamping(DirectionType::FORWARD, false);
                bool has_colors = !forward_clamped_colors_.empty();
                return if_barrier_ ? static_cast<int>(clamped && has_colors) : static_cast<int>(clamped && has_colors) * 5;
            },
            [this] { return static_cast<int>(behaviors_.rotating(DeviceType::CAR)); },
            [this] { return static_cast<int>(behaviors_.moving(DirectionType::BACKWARD, color_to_point_[forward_clamped_colors_.front()])); },
            [this] { return static_cast<int>(behaviors_.clamping(DirectionType::BACKWARD, false)); },
            [this] { return static_cast<int>(behaviors_.rotating(DeviceType::CAR)); },
            [this] { return static_cast<int>(behaviors_.moving(DirectionType::FORWARD, color_to_target_[forward_clamped_colors_.front()])); },
            [this] {
                if (behaviors_.placing(DirectionType::FORWARD, false)) {
                    forward_clamped_colors_.erase(forward_clamped_colors_.begin());
                    return if_barrier_ ? 1 : -100;
                } else
                    return 0;
            },
            [this] { return static_cast<int>(behaviors_.moving(DirectionType::BACKWARD, LocationType::SOLIDDISTANCE)); },
            [this] { return static_cast<int>(behaviors_.placing(DirectionType::BACKWARD, false)); },
            [this] { return static_cast<int>(behaviors_.moving(DirectionType::FORWARD, LocationType::SOLIDDISTANCE)); },
            [this] { return static_cast<int>(behaviors_.rotating(DeviceType::CAR)); },
            [this] {
                if (behaviors_.clamping(DirectionType::FORWARD, false)) {
                    if_barrier_ = false;
                    return 1;
                } else
                    return 0;
            },
        },

        [this] {
            stages_[StageType::GET_AND_PLACE_A_POINT].current_stage = 0;
        },
        [this] {
            auto stages = stages_[StageType::GET_AND_PLACE_A_POINT].stages;
            auto& current_stage = stages_[StageType::GET_AND_PLACE_A_POINT].current_stage;

            if (current_stage >= static_cast<int>(stages.size()) || current_stage < 0) {
                transitionTo(StageType::GET_AND_PLACE_C_POINT);
                return;
            }
            auto result = stages[current_stage]();
            RCLCPP_INFO(this->get_logger(), "[Stage %d]Substage %d result: %d", static_cast<int>(current_stage_), current_stage, result);
            current_stage += result;
        },
        [] {} // exit
    };
};
}
