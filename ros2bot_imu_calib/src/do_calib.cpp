// class for performing IMU calibration

#include "ros2bot_imu_calib/do_calib.hpp"

using std::placeholders::_1;

DoCalib::DoCalib() : Node("do_calib_node")
{
  state_ = START;

  std::string package_share_directory = ament_index_cpp::get_package_share_directory("ros2bot_imu_calib");
  std::string imu_calib_path = package_share_directory + "/config/imu_calib.yaml";

  // create velocity / geometry twist message subscription
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1, std::bind(&DoCalib::imu_cb, this, _1));

  // declare parameters w/ defaults
  this->declare_parameter("measurements", 500);
  rclcpp::Parameter measurements_param = this->get_parameter("measurements");
  measurements_per_orientation_ = measurements_param.as_int();

  this->declare_parameter("reference_acceleration", 9.80665);
  rclcpp::Parameter acceleration_param = this->get_parameter("reference_acceleration");
  reference_acceleration_ = acceleration_param.as_double();

  this->declare_parameter("output_file", imu_calib_path);
  rclcpp::Parameter output_file_param = this->get_parameter("output_file");
  output_file_ = output_file_param.as_string();

  orientations_.push(AccelCalib::XPOS);
  orientations_.push(AccelCalib::XNEG);
  orientations_.push(AccelCalib::YPOS);
  orientations_.push(AccelCalib::YNEG);
  orientations_.push(AccelCalib::ZPOS);
  orientations_.push(AccelCalib::ZNEG);

  orientation_labels_[AccelCalib::XPOS] = "X+";
  orientation_labels_[AccelCalib::XNEG] = "X-";
  orientation_labels_[AccelCalib::YPOS] = "Y+";
  orientation_labels_[AccelCalib::YNEG] = "Y-";
  orientation_labels_[AccelCalib::ZPOS] = "Z+";
  orientation_labels_[AccelCalib::ZNEG] = "Z-";
}

bool DoCalib::running()
{
  return state_ != DONE;
}

void DoCalib::imu_cb(sensor_msgs::msg::Imu::SharedPtr imu)
{
  bool accepted;

  switch (state_)
  {
  case START:
    calib_.beginCalib(6 * measurements_per_orientation_, reference_acceleration_);
    state_ = SWITCHING;
    break;

  case SWITCHING:
    if (orientations_.empty())
    {
      state_ = COMPUTING;
    }
    else
    {
      current_orientation_ = orientations_.front();

      orientations_.pop();
      measurements_received_ = 0;

      std::cout << "orient IMU with " << orientation_labels_[current_orientation_] << " axis up and press <enter>";
      std::cin.get();
      std::cout << "recording measurements...";

      state_ = RECEIVING;
    }

    break;

  case RECEIVING:
    accepted = calib_.addMeasurement(current_orientation_,
                                     imu->linear_acceleration.x,
                                     imu->linear_acceleration.y,
                                     imu->linear_acceleration.z);

    measurements_received_ += accepted ? 1 : 0;

    if (measurements_received_ >= measurements_per_orientation_)
    {
      std::cout << " done." << std::endl;
      state_ = SWITCHING;
    }

    break;

  case COMPUTING:
    std::cout << "computing calibration parameters...";

    if (calib_.computeCalib())
    {
      std::cout << " success!" << std::endl;
      std::cout << "saving calibration file...";

      if (calib_.saveCalib(output_file_))
      {
        std::cout << " success!" << std::endl;
      }
      else
      {
        std::cout << " failed." << std::endl;
      }
    }
    else
    {
      std::cout << " failed.";
      RCLCPP_ERROR(this->get_logger(), "[DoCalibNode]: calibration failed");
    }

    state_ = DONE;
    break;

  case DONE:
    break;
  }
}
