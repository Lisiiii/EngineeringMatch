#include "image_process/image_processer.hpp"
#include "include/monobehaviour.hpp"
#include "message/message_handler.hpp"
#include "serial/serial_handler.hpp"
#include "state_machine/state_machine.hpp"
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
using namespace engineering_match;
int main()
{
    rclcpp::init(0, nullptr);

    rclcpp::executors::SingleThreadedExecutor rcl_executor;

    auto image_processer = std::make_shared<image::ImageProcesser>("ImageProcesser", 30);
    image_processer->start();

    // auto message_handler = std::make_shared<message::MessageHandler>("MessageHandler", 1);
    // message_handler->start();
    auto serial_handler = std::make_shared<SerialHandler::SerialHandler>("SerialHandler", 30);
    serial_handler->start();

    auto state_machine = std::make_shared<state_machine::StateMachine>("StateMachine", 30, image_processer);
    state_machine->start();

    // rcl_executor.add_node(message_handler);
    rcl_executor.add_node(image_processer);
    rcl_executor.add_node(state_machine);
    rcl_executor.add_node(serial_handler);

    rcl_executor.spin();

    rclcpp::shutdown();

    return 0;
}