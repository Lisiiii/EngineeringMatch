cmake_minimum_required(VERSION 3.8)
project(pathfinder)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB FUNCTIONS_SOURCE src/*.cpp)

# BUILD EXE
add_executable(
  ${PROJECT_NAME}
  ${FUNCTIONS_SOURCE}
)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()