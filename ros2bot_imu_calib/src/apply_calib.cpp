// class for applying a previously computed calibration to IMU data

#include "ros2bot_imu_calib/apply_calib.hpp"

using std::placeholders::_1;

ApplyCalib::ApplyCalib() : Node("apply_calib_node")
{
  int queue_size;

  gyro_sample_count_ = 0;
  gyro_bias_x_ = 0.0;
  gyro_bias_y_ = 0.0;
  gyro_bias_z_ = 0.0;

  std::string calib_file;
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("ros2bot_imu_calib");
  std::string imu_calib_path = package_share_directory + "/config/imu_calib.yaml";

  // get parameters
  this->declare_parameter("calib_file", imu_calib_path);
  rclcpp::Parameter calib_file_param = this->get_parameter("calib_file");
  calib_file = calib_file_param.as_string();

  this->declare_parameter("calibrate_gyros", true);
  rclcpp::Parameter calibrate_gyros_param = this->get_parameter("calibrate_gyros");
  calibrate_gyros_ = calibrate_gyros_param.as_bool();

  this->declare_parameter("gyro_calib_samples", 100);
  rclcpp::Parameter gyro_calib_samples_param = this->get_parameter("gyro_calib_samples");
  gyro_calib_samples_ = gyro_calib_samples_param.as_int();

  this->declare_parameter("queue_size", 5);
  rclcpp::Parameter queue_size_param = this->get_parameter("queue_size");
  queue_size = queue_size_param.as_int();  

  if (!calib_.loadCalib(calib_file) || !calib_.calibReady())
  {
    RCLCPP_FATAL(this->get_logger(), "Calibration could not be loaded");
    rclcpp::shutdown();
  }

  // create subscription to raw imu
  raw_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("raw", queue_size, std::bind(&ApplyCalib::imu_cb, this, _1));

  // create publisher for corrected imu
  corrected_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("corrected", queue_size);
}

void ApplyCalib::imu_cb(sensor_msgs::msg::Imu::SharedPtr raw)
{
  if (calibrate_gyros_)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "Calibrating gyros; do not move the IMU");

    // recursively compute mean gyro measurements
    gyro_sample_count_++;
    gyro_bias_x_ = ((gyro_sample_count_ - 1) * gyro_bias_x_ + raw->angular_velocity.x) / gyro_sample_count_;
    gyro_bias_y_ = ((gyro_sample_count_ - 1) * gyro_bias_y_ + raw->angular_velocity.y) / gyro_sample_count_;
    gyro_bias_z_ = ((gyro_sample_count_ - 1) * gyro_bias_z_ + raw->angular_velocity.z) / gyro_sample_count_;

    if (gyro_sample_count_ >= gyro_calib_samples_)
    {
      RCLCPP_INFO(this->get_logger(), "Gyro calibration complete! (bias = [%.3f, %.3f, %.3f])", gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
      calibrate_gyros_ = false;
    }

    return;
  }

  sensor_msgs::msg::Imu corrected = *raw;

  calib_.applyCalib(raw->linear_acceleration.x, raw->linear_acceleration.y, raw->linear_acceleration.z,
                    &corrected.linear_acceleration.x, &corrected.linear_acceleration.y, &corrected.linear_acceleration.z);

  corrected.angular_velocity.x -= gyro_bias_x_;
  corrected.angular_velocity.y -= gyro_bias_y_;
  corrected.angular_velocity.z -= gyro_bias_z_;

  corrected_pub_->publish(corrected);
}
