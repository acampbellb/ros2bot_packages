// class for performing IMU calibration

#include <string>
#include <vector>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "ros2bot_imu_calib/accel_calib.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

class DoCalib : public rclcpp::Node
{
public:
  DoCalib();

  bool running();

private:
  enum DoCalibState
  {
    START,
    SWITCHING,
    RECEIVING,
    COMPUTING,
    DONE
  };

  AccelCalib calib_;
  DoCalibState state_;

  int measurements_per_orientation_;
  int measurements_received_;

  double reference_acceleration_;
  std::string output_file_;

  std::queue<AccelCalib::Orientation> orientations_;
  AccelCalib::Orientation current_orientation_;

  std::string orientation_labels_[6];

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  void imu_cb(sensor_msgs::msg::Imu::SharedPtr imu);
};