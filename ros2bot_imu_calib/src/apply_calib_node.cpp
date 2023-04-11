// node applies a previosly computed calibration to imu data

#include "ros2bot_imu_calib/apply_calib.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApplyCalib>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
