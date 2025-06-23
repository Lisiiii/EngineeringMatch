#include "image_process/image_processer.hpp"
#include "imu/imu_handler.hpp"
#include "include/monobehaviour.hpp"
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
using namespace engineering_match;
int main()
{
    rclcpp::init(0, nullptr);
    auto object_manager = std::make_shared<base::ObjectManager>("ObjectManager", 20.0f);
    object_manager->instantiate<imu::IMUHandler>();
    object_manager->subscribe<std_msgs::msg::Float32MultiArray>(
        "/unity/imu_data", &imu::IMUHandler::imu_data_callback);
    object_manager->subscribe<std_msgs::msg::Float32MultiArray>(
        "/unity/target_position", &imu::IMUHandler::target_position_callback);
    object_manager->instantiate<image::ImageProcesser>();
    object_manager->start();

    rclcpp::spin(object_manager);
    rclcpp::shutdown();

    return 0;
}