#include "image_process/image_processer.hpp"
#include "imu/imu_handler.hpp"
#include "pathfinder/state_machine.hpp"
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
using namespace engineering_match;
int main()
{
    rclcpp::init(0, nullptr);

    rclcpp::executors::SingleThreadedExecutor rcl_executor;
    // auto image_processer = std::make_shared<image::ImageProcesser>("ImageProcesser", 10);
    // auto imu_handler = std::make_shared<imu::IMUHandler>("IMUHandler", 5);
    auto state_machine = std::make_shared<pathfinder::StateMachine>("StateMachine", 20);
    state_machine->start();

    // rcl_executor.add_node(image_processer);
    // rcl_executor.add_node(imu_handler);
    rcl_executor.add_node(state_machine);

    rcl_executor.spin();

    rclcpp::shutdown();

    return 0;
}