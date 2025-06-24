#include "image_process/image_processer.hpp"
#include "imu/imu_handler.hpp"
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
using namespace engineering_match;
int main()
{
    rclcpp::init(0, nullptr);

    rclcpp::executors::SingleThreadedExecutor rcl_executor;
    auto image_processer = std::make_shared<image::ImageProcesser>("ImageProcesser", 30);
    auto imu_handler = std::make_shared<imu::IMUHandler>("IMUHandler", 30);

    rclcpp::spin(image_processer);

    rclcpp::shutdown();

    return 0;
}