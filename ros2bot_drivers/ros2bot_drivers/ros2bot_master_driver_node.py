#!/usr/bin/env python3

import math
import random
import rclpy

from sensor_msgs.msg import Imu, MagneticField, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool
from ros2bot_master_lib import Ros2botMasterDriver
from rclpy.node import Node
from rclpy.clock import Clock

class Ros2botMasterDriverNode(Node):
    def __init__(self):
        super().__init__("ros2bot_master_driver_node")
        self.get_logger().info("Hello from ros2bot driver node")

        # initialize state
        self.RA2DE = 180 / math.pi
        self.master = Ros2botMasterDriver()
        self.master.set_bot_type(1)
        self.timer = None

        # declare parameters
        self.declare_parameter('imu_link', 'imu_link')
        self.declare_parameter('prefix', '')
        self.declare_parameter('xlinear_limit', 1.0)
        self.declare_parameter('ylinear_limit', 1.0)
        self.declare_parameter('angular_limit', 5.0)
        self.declare_parameter('process_cmd_freq', 0.2)

        # get parameters
        self.imu_link = self.get_parameter('imu_link')
        self.prefix = self.get_parameter('prefix')
        self.xlinear_limit = self.get_parameter('xlinear_limit')
        self.ylinear_limit = self.get_parameter('ylinear_limit')
        self.angular_limit = self.get_parameter('angular_limit')
        self.process_cmd_freq = self.get_parameter('process_cmd_freq')

        # create subscriptions 
        self.sub_cmd_velocity = self.create_subscription(Twist, 'cmd_vel', self.cmd_velocity_cb, 1)
        self.sub_rgb_light = self.create_subscription(Int32, 'rgb_light', self.rgb_light_cb, 100)
        self.sub_buzzer = self.create_subscription(Bool, 'buzzer', self.buzzer_cb, 100)

        # create publishers w/ relative named topics
        self.edition_pub = self.create_publisher(Float32, 'edition', 100)
        self.battery_voltage_pub = self.create_publisher(Float32, 'voltage', 100)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 100)

        # create publishers w/ absolute named topics
        self.velocity_pub = self.create_publisher(Twist, 'vel_raw', 50)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 100)
        self.magnetic_field_pub = self.create_publisher(MagneticField, '/imu/mag', 100)

        # start dirver library's receiving thread
        self.master.create_receive_thread()

    def start_timer(self):
        self.timer = self.create_timer(self.process_cmd_freq, self.process_cmd_cb)

    def cancel_timer(self):
        if not self.timer.is_canceled:
            self.timer.cancel()

    # car motion control, subscriber callback function
    def cmd_velocity_cb(self, msg):
        self.get_logger().info('ros2bot_master_driver node heard velocity msg: "%s"' % msg.data)
        # robot motion control, subscriber callback function
        if not isinstance(msg, Twist): 
            return

        # issue linear and angular velocity
        vx = msg.linear.x * 1.0
        vy = msg.linear.y * 1.0
        angular = msg.angular.z * 1.0
        # velocity: ±1, angular: ±5
        # trolley motion control, velocity=[-1, 1], angular=[-5, 5]
        self.master.set_bot_motion(vx, vy, angular)

    def rgb_light_cb(self, msg):
        self.get_logger().info('ros2bot_master_driver node heard rgb light msg: "%s"' % msg.data)

        if not isinstance(msg, Int32): 
            return

        for i in range(3): 
            self.master.set_colorful_effect(msg.data, 6, parm=1)

    def buzzer_cb(self, msg):
        self.get_logger().info('ros2bot_master_driver node heard buzzer msg: "%s"' % msg.data)

        if not isinstance(msg, Bool): 
            return

        if msg.data:
            for i in range(3): 
                self.master.set_beep(1)
        else:
            for i in range(3): 
                self.master.set_beep(0)

    # Publish the speed of the robot, gyroscope data, and battery voltage
    def process_cmd_cb(self):
        if rclpy.ok():
            time_stamp = Clock().now()

            # get master driver data
            ax, ay, az = self.master.get_accelerometer_data()
            gx, gy, gz = self.master.get_gyroscope_data()
            mx, my, mz = self.master.get_magnetometer_data()
            vx, vy, angular = self.master.get_motion_data()

            mx = mx * 1.0
            my = my * 1.0
            mz = mz * 1.0

            # initialize imu gyroscope
            imu = Imu()
            imu.header.stamp = time_stamp.to_msg()
            imu.header.frame_id = self.imu_link
            imu.linear_acceleration.x = ax * 1.0
            imu.linear_acceleration.y = ay * 1.0
            imu.linear_acceleration.z = az * 1.0
            imu.angular_velocity.x = gx * 1.0
            imu.angular_velocity.y = gy * 1.0
            imu.angular_velocity.z = gz * 1.0

            # initialize magnetic gyrocope
            magnetic_field = MagneticField()
            magnetic_field.header.stamp = time_stamp.to_msg()
            magnetic_field.header.frame_id = self.imu_link
            magnetic_field.magnetic_field.x = mx * 1.0
            magnetic_field.magnetic_field.y = my * 1.0
            magnetic_field.magnetic_field.z = mz * 1.0

            # initialize twist
            twist = Twist()
            twist.linear.x = vx * 1.0
            twist.linear.y = vy * 1.0
            twist.angular.z = angular * 1.0

            # initialize voltage
            battery_voltage = Float32()
            battery_voltage.data = self.master.get_battery_voltage() * 1.0

            # initialize edition
            edition = Float32()
            edition.data = self.master.get_version() * 1.0

            # initialize joint state
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now()
            joint_state.header.frame_id = "joint_states"
            if len(self.prefix) == 0:
                joint_state.name = ["front_right_joint", "front_left_joint",
                                    "back_right_joint", "back_left_joint"]
            else:
                joint_state.name = [self.prefix+"/front_right_joint", self.prefix+"/front_left_joint",
                                    self.prefix+"/back_right_joint", self.prefix+"/back_left_joint"]
            joint_state.position = [0, 0, 0, 0]
            if not vx == vy == angular == 0:
                i = random.uniform(-3.14, 3.14)
                joint_state.position = [i, i, i, i]

            # publish the current linear velocity and angular velocity of the master driver
            self.velocity_pub.publish(twist)

            # publish gyroscope data
            self.imu_pub.publish(imu)
            self.magnetic_field_pub.publish(magnetic_field)

            # publish voltage data
            self.battery_voltage_pub.publish(battery_voltage)

            # publish edition
            self.edition_pub.publish(edition)

            # publish joint state data (Disabled)
            # self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = Ros2botMasterDriverNode()
    
    try:
        node.start_timer()
        rclpy.spin(node)
    except Exception as ex:
        print(ex)
        print("ros2bot master driver node shutting down")
    finally:
        node.cancel_timer()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

