cmake_minimum_required(VERSION 3.5)
project(ros2bot_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(ros2bot_base_node src/ros2bot_base_node.cpp)
ament_target_dependencies(ros2bot_base_node rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)

# install targets
install(TARGETS
  ros2bot_base_node
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
