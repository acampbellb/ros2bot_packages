// class for applying a previously computed calibration to IMU data

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/Imu.h>
#include <imu_calib/accel_calib.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace imu_calib : public rclcpp::Node
{
  class ApplyCalib
  {
  public:
    ApplyCalib();

  private:
    AccelCalib calib_;

    //ros::Subscriber raw_sub_;
    //ros::Publisher corrected_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr corrected_pub_;

    void rawImuCallback(sensor_msgs::msg::Imu::SharedPtr raw);

    bool calibrate_gyros_;
    int gyro_calib_samples_;
    int gyro_sample_count_;

    double gyro_bias_x_;
    double gyro_bias_y_;
    double gyro_bias_z_;
  };
} 
