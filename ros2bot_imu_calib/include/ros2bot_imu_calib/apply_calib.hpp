// class for applying a previously computed calibration to IMU data

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "ros2bot_imu_calib/accel_calib.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

class ApplyCalib : public rclcpp::Node
{
public:
  ApplyCalib();

private:
  AccelCalib calib_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr corrected_pub_;

  void imu_cb(sensor_msgs::msg::Imu::SharedPtr raw);

  bool calibrate_gyros_;
  int gyro_calib_samples_;
  int gyro_sample_count_;

  double gyro_bias_x_;
  double gyro_bias_y_;
  double gyro_bias_z_;
};
