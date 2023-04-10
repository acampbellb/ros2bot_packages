// class for applying a previously computed calibration to IMU data

#include "ros2bot_imu_calib/apply_calib.h"

namespace imu_calib
{
  ApplyCalib::ApplyCalib() : gyro_sample_count_(0),
                             gyro_bias_x_(0.0),
                             gyro_bias_y_(0.0),
                             gyro_bias_z_(0.0)
  {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("imu_calib");
    std::string imu_calib_path = package_share_directory + "/config/imu_calib.yaml";

    // get parameters
    this->declare_parameter("calib_file", imu_calib_path);
    rclcpp::Parameter calib_file_param = this->get_parameter("calib_file");
    calib_file_ = calib_file_param.as_string();

    if (!calib_.loadCalib(calib_file) || !calib_.calibReady())
    {
      RCLCPP_FATAL(this->get_logger(), "Calibration could not be loaded");
      ros::shutdown();
    }

    nh_private.param<bool>("calibrate_gyros", calibrate_gyros_, true);
    nh_private.param<int>("gyro_calib_samples", gyro_calib_samples_, 100);

    int queue_size;
    nh_private.param<int>("queue_size", queue_size, 5);

    raw_sub_ = nh.subscribe("raw", queue_size, &ApplyCalib::rawImuCallback, this);
    corrected_pub_ = nh.advertise<sensor_msgs::msg::Imu>("corrected", queue_size);
  }

  void ApplyCalib::rawImuCallback(sensor_msgs::msg::Imu::SharedPtr raw)
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

    corrected_pub_.publish(corrected);
  }
} 
