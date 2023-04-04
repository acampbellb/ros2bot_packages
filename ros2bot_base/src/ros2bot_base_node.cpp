#include <string.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::placeholders::_1;

/*
 * Node parameters
 * -------------------------------
 * - linear_scale_x (double)
 * - linear_scale_y (double)
 * - odom_frame (string)
 * - base_footprint_frame (string)
 */

/// @brief ros2bot robot base node
/// Serves as the publisher of odometry messages. It subscribes to velocity messages
/// in the form of geometry twist messages, and produces odometry messages from them.
/// This is a heavy-duty cycle node, so it's implemented as a Cpp node.
class Ros2botBaseNode : public rclcpp::Node
{

public:
    /// @brief Constructor
    Ros2botBaseNode() : Node("ros2bot_base_node")
    {
        RCLCPP_INFO(this->get_logger(), "[Ros2botBaseNode]: initializing");

        // declare parameters w/ defaults
        this->declare_parameter("linear_scale_x", 1.0);
        this->declare_parameter("linear_scale_y", 1.0);
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_footprint_frame", "base_footprint");

        // get parameters
        rclcpp::Parameter linear_scale_x_param = this->get_parameter("linear_scale_x");
        rclcpp::Parameter linear_scale_y_param = this->get_parameter("linear_scale_y");
        rclcpp::Parameter odom_frame_param = this->get_parameter("odom_frame");
        rclcpp::Parameter base_footprint_frame_param = this->get_parameter("base_footprint_frame");

        // convert parameters to field values
        linear_scale_x_ = linear_scale_x_param.as_double();
        linear_scale_y_ = linear_scale_y_param.as_double();
        odom_frame_ = odom_frame_param.as_string();
        base_footprint_frame_ = base_footprint_frame_param.as_string();

        // initialize remaining fields
        linear_velocity_x_ = 0.0;
        linear_velocity_y_ = 0.0;
        angular_velocity_z_ = 0.0;
        last_velocity_time_ = rclcpp::Time();
        velocity_dt_ = 0.0;
        x_pos_ = 0.0;
        y_pos_ = 0.0;
        heading_ = 0.0;

        // create velocity / geometry twist message subscription
        velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/sub_vel", 50, std::bind(&Ros2botBaseNode::velocity_subscriber_cb, this, _1));

        // create odometry publisher
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/pub_odom", 50);

        RCLCPP_INFO(this->get_logger(), "[Ros2botBaseNode]: initialized");
    }

private:
    // fields

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Time last_velocity_time_;
    std::string odom_frame_;
    std::string base_footprint_frame_;
    double linear_scale_x_;
    double linear_scale_y_;
    double velocity_dt_;
    double x_pos_;
    double y_pos_;
    double heading_;
    double linear_velocity_x_;
    double linear_velocity_y_;
    double angular_velocity_z_;

    // methods

    /// @brief Callback function that receives geometry twist messages
    /// @param msg 
    void velocity_subscriber_cb(const geometry_msgs::msg::Twist::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "ODOM PUBLISH %.2f,%.2f,%.2f,%.2f,%.2f,%.2f", 
            msg->linear.x, msg->linear.y, msg->linear.z, 
            msg->angular.x, msg->angular.y, msg->angular.z);            
        RCLCPP_INFO_STREAM(this->get_logger(), "[Ros2botBaseNode]: received twist message '" << msg << "'");

        rclcpp::Time current_time = now();
        linear_velocity_x_ = msg->linear.x * linear_scale_x_;
        linear_velocity_y_ = msg->linear.y * linear_scale_y_;
        angular_velocity_z_ = msg->angular.z;
        velocity_dt_ = (current_time - last_velocity_time_).seconds();
        last_velocity_time_ = current_time;

        // compute odometry given the velocities of the robot
        double delta_heading = angular_velocity_z_ * velocity_dt_; // radians
        double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * velocity_dt_;
        double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * velocity_dt_;

        // calculate current position of robot
        x_pos_ += delta_x;
        y_pos_ += delta_y;
        heading_ += delta_heading;

        // calculate robot's heading in quaternion angle
        geometry_msgs::msg::Quaternion odometry_quaternion = create_quaternion_msg_from_yaw(heading_);
        nav_msgs::msg::Odometry odometry_msg;

        odometry_msg.header.stamp = current_time;
        odometry_msg.header.frame_id = odom_frame_;
        odometry_msg.child_frame_id = base_footprint_frame_;

        // robot position as x, y, z
        odometry_msg.pose.pose.position.x = x_pos_;
        odometry_msg.pose.pose.position.y = y_pos_;
        odometry_msg.pose.pose.position.z = 0.0;

        // robot heading as quaternion
        odometry_msg.pose.pose.orientation = odometry_quaternion;
        odometry_msg.pose.covariance[0] = 0.001;
        odometry_msg.pose.covariance[7] = 0.001;
        odometry_msg.pose.covariance[35] = 0.001;

        // linear speed from encoders
        odometry_msg.twist.twist.linear.x = linear_velocity_x_;
        odometry_msg.twist.twist.linear.y = linear_velocity_y_;
        odometry_msg.twist.twist.linear.z = 0.0;
        odometry_msg.twist.twist.angular.x = 0.0;
        odometry_msg.twist.twist.angular.y = 0.0;

        // angular speed from encoders
        odometry_msg.twist.twist.angular.z = angular_velocity_z_;
        odometry_msg.twist.covariance[0] = 0.0001;
        odometry_msg.twist.covariance[7] = 0.0001;
        odometry_msg.twist.covariance[35] = 0.0001;

        // odometry publish
        RCLCPP_INFO(this->get_logger(), "ODOMETRY MESSAGE PUBLISHED");
        odometry_publisher_->publish(odometry_msg);
    }

    /// @brief Function create a quaternion message from a yaw value
    /// @param yaw 
    /// @return Quaternion
    geometry_msgs::msg::Quaternion create_quaternion_msg_from_yaw(double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    }
};

/// @brief Entrypoint
/// @param argc Argument count
/// @param argv Array of arguments
/// @return 0
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ros2botBaseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}