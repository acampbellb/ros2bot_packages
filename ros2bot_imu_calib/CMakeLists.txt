cmake_minimum_required(VERSION 3.5)
project(ros2bot_imu_calib)

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
find_package(sensor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS}
                            ${YAML_INCLUDEDIR}
)

add_definitions(${EIGEN3_DEFINITIONS})

# accel_calib library
set(BUILD_SHARED_LIBS ON)
add_library(accel_calib SHARED src/accel_calib/accel_calib.cpp)
ament_target_dependencies(accel_calib rclcpp yaml-cpp)
target_link_libraries(accel_calib yaml-cpp)

# do calib node
add_executable(do_calib src/do_calib_node.cpp src/do_calib.cpp)
ament_target_dependencies(do_calib rclcpp sensor_msgs Eigen3 ament_index_cpp)
target_link_libraries(do_calib accel_calib)

# apply calib node
add_executable(apply_calib src/apply_calib_node.cpp src/apply_calib.cpp)
ament_target_dependencies(apply_calib rclcpp sensor_msgs Eigen3 ament_index_cpp)
target_link_libraries(apply_calib accel_calib)

# install targets
install(TARGETS 
  accel_calib do_calib apply_calib
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY 
  config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
