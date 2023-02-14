#!/usr/bin/env python3

import rclpy

import math
import numpy as np

from pid import SinglePID
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class Ros2botLidarWarningNode(Node):
    def __init__(self):
        super().__init__("ros2bot_lidar_warning_node")
        self.get_logger().info("Hello from ros2bot lidar warning node")

        # initialize state
        self.RA2DE = 180 / math.pi
        self.moving = False
        self.buzzer_state = False
        self.joy_active = False

        # declare parameters
        self.declare_parameter('switch', False)
        self.declare_parameter('laser_angle', 70)
        self.declare_parameter('response_dist', 0.5)
        self.declare_parameter('angle_kp', 3.0)
        self.declare_parameter('angle_ki', 0.0)
        self.declare_parameter('angle_kd', 5.0)

        # get parameters
        self.switch = self.get_parameter('switch')
        self.laser_angle = self.get_parameter('laser_angle')
        self.response_dist = self.get_parameter('response_dist')
        self.angle_kp = self.get_parameter('angle_kp')
        self.angle_ki = self.get_parameter('angle_ki')
        self.angle_kd = self.get_parameter('angle_kd')

        # initialize pid
        self.angle_pid = SinglePID(self.angle_kp, self.angle_ki, self.angle_kd)

        # create publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 3)
        self.buzzer_pub = self.create_publisher(Bool, '/buzzer', 1)

        # create subscriptions
        self.joy_state_sub = self.create_subscription(Bool, '/joy_state', self.joy_state_cb, 1)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_scan_cb, 1)

    def joy_state_cb(self, msg):
        if not isinstance(msg, Bool): 
            return
        self.joy_active = msg.data

    def laser_scan_cb(self, msg):
        if not isinstance(msg, LaserScan): 
            return

        # record the laser scan and publish the position of the nearest object (or point to a point)
        ranges = np.array(msg.ranges)

        # create distance list, put the effective distance within the detection range into the list
        min_dist_list = []

        # create a serial number and place the ID corresponding to the valid distance in the list
        min_dist_id_list = []

        # if we already have a last scan to compare to:
        for i in range(len(ranges)):
            angle = (msg.angle_min + msg.angle_increment * i) * self.RA2DE
            # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, msg.ranges[i])
            if abs(angle) > (180 - self.laser_angle):
                min_dist_list.append(ranges[i])
                min_dist_id_list.append(angle)

        if len(min_dist_list) == 0: 
            return

        # find the minimum distance
        min_dist = min(min_dist_list)

        # find the ID corresponding to the minimum distance
        min_dist_id = min_dist_id_list[min_dist_list.index(min_dist)]

        if self.joy_ctrl.joy_active or self.switch == True:
            if self.moving == True:
                self.vel_pub.publish(Twist())
                self.moving = not self.moving
            return

        self.moving = True

        if min_dist <= self.response_dist:
            if self.buzzer_state == False:
                b = Bool()
                b.data = True
                self.buzzer_pub.publish(b)
                self.buzzer_state = True
        else:
            if self.buzzer_state == True:
                self.buzzer_pub.publish(Bool())
                self.buzzer_state = False

        velocity = Twist()

        # The PID algorithm is used to make the car move to the corresponding position steadily
        angle_pid_compute = self.angle_pid.pid_compute((180 - abs(min_dist_id)) / 36, 0)

        if min_dist_id > 0: 
            velocity.angular.z = -angle_pid_compute
        else: 
            velocity.angular.z = angle_pid_compute

        if angle_pid_compute < 0.02: 
            velocity.angular.z = 0

        self.vel_pub.publish(velocity)

def main(args=None):
    rclpy.init(args=args)
    node = Ros2botLidarWarningNode()
    
    try:
        rclpy.spin(node)
    except Exception as ex:
        print(ex)
        print("ros2bot lidar warning node shutting down")
    finally:
        node.vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()