#pragma once

#include "../include/monobehaviour.hpp"
#include <functional>
#include <unordered_map>

namespace engineering_match::pathfinder {

class StateMachine;

enum class StateType {
    IDLE,
    MOVE,
    CLAMP,
    PLACE,
    ROTATE,
};

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
};

enum class DirectionType {
    FORWARD,
    BACKWARD,
};

class StateMachine : base::IMonoBehaviour {
public:
    StateMachine()
        : base::IMonoBehaviour("StateMachine") {};

    void start() override
    {
        RCLCPP_INFO(rclcpp::get_logger("StateMachine"), "StateMachine started.");

        states_[StateType::IDLE] = {
            [this] {
                transitionTo(StateType::MOVE);
            },
            [] {},
            [] {},
        };
        states_[StateType::MOVE] = {
            [this] {
                transitionTo(StateType::CLAMP);
            },
            [] {},
            [] {},
        };
        states_[StateType::CLAMP] = {
            [this] {
                RCLCPP_INFO(rclcpp::get_logger("StateMachine"), "Clamping the object.");
                transitionTo(StateType::PLACE);
            },
            [] {},
            [] {},
        };
        states_[StateType::PLACE] = {
            [this] {
                RCLCPP_INFO(rclcpp::get_logger("StateMachine"), "Placing the object.");
                // Logic for placing the object
                transitionTo(StateType::ROTATE);
            },
            [] {},
            [] {},
        };
        states_[StateType::ROTATE] = {
            [this] {
                RCLCPP_INFO(rclcpp::get_logger("StateMachine"), "Rotating the object.");
                // Logic for rotating the object
                transitionTo(StateType::IDLE);
            },
            [] {},
            [] {},
        };

        transitionTo(StateType::IDLE);
        current_location_ = LocationType::STARTUPPOINT;
    }

    void update() override
    {
        states_[current_state_].update();
    }

    void transitionTo(StateType new_state)
    {
        states_[current_state_].exit();
        current_state_ = new_state;
        states_[current_state_].enter();
    }

private:
    StateType current_state_;
    LocationType current_location_;
    struct StateBehaviour {
        std::function<void()> enter;
        std::function<void()> update;
        std::function<void()> exit;
    };

    std::unordered_map<StateType, StateBehaviour> states_;
};

}