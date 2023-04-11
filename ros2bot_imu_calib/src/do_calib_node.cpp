// node performs accelerometer calibration and writes parameters to data file

#include "ros2bot_imu_calib/do_calib.hpp"

/// @brief Entrypoint
/// @param argc Argument count
/// @param argv Array of arguments
/// @return 0
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);  
  auto node = std::make_shared<DoCalib>();
  
  while (rclcpp::ok() && node->running())
  {
      rclcpp::spin_some(node);
  }

  return 0;
}
