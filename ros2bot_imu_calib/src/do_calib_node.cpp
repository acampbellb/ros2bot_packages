// node performs accelerometer calibration and writes parameters to data file

#include "ros2bot_imu_calib/do_calib.h"

/// @brief Entrypoint
/// @param argc Argument count
/// @param argv Array of arguments
/// @return 0
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);  
  auto node = std::make_shared<imu_calib::DoCalib>();
  
  while (rclcpp::ok() && node.running())
  {
      rclcpp::spin_once(node);
  }

  return 0;
}
