#!/usr/bin/env python3

import rclpy

import math
import numpy as np

from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class Ros2botLidarAvoidanceNode(Node):
    def __init__(self):
        super().__init__("ros2bot_lidar_avoidance_node")
        self.get_logger().info("Hello from ros2bot lidar avoidance node")

        # initialize state
        self.RA2DE = 180 / math.pi
        self.joy_active = False
        self.moving = False
        self.right_warning = 0
        self.left_warning = 0
        self.front_warning = 0

        # declare parameters
        self.declare_parameter('switch', False)
        self.declare_parameter('linear', 0.5)
        self.declare_parameter('angular', 1.0)
        self.declare_parameter('laser_angle', 40.0) # 10-180
        self.declare_parameter('response_dist', 0.55)

        # get parameters
        self.switch = self.get_parameter('switch')
        self.linear = self.get_parameter('linear')
        self.angular = self.get_parameter('angular')
        self.laser_angle = self.get_parameter('laser_angle')
        self.response_dist = self.get_parameter('response_dist')

        # create publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

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

        # Record the laser scan and publish the position of the nearest object (or point to a point)
        ranges = np.array(msg.ranges)
        self.right_warning = 0
        self.left_warning = 0
        self.front_warning = 0

        # if we already have a last scan to compare to
        for i in range(len(ranges)):
            angle = (msg.angle_min + msg.angle_increment * i) * self.RA2DE
            # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, scan_data.ranges[i])
            if 160 > angle > 180 - self.laser_angle:
                if ranges[i] < self.response_dist * 1.5: self.right_warning += 1
            if - 160 < angle < self.laser_angle - 180:
                if ranges[i] < self.response_dist * 1.5: self.left_warning += 1
            if abs(angle) > 160:
                if ranges[i] <= self.response_dist * 1.5: self.front_warning += 1

        # print (self.left_warning, self.front_warning, self.right_warning)
        if self.joy_ctrl.joy_active or self.switch == True:
            if self.moving == True:
                self.vel_pub.publish(Twist())
                self.moving = not self.moving
            return
        self.Moving = True
        twist = Twist()

        # Left positive and right negative
        if self.front_warning > 10 and self.left_warning > 10 and self.right_warning > 10:
            print ('1, there are obstacles in the left and right, turn right')
            twist.linear.x = self.linear
            twist.angular.z = -self.angular
            self.vel_pub.publish(twist)
            sleep(0.2)
        elif self.front_warning > 10 and self.left_warning <= 10 and self.right_warning > 10:
            print ('2, there is an obstacle in the middle right, turn left')
            twist.linear.x = 0.0
            twist.angular.z = self.angular
            self.vel_pub.publish(twist)
            sleep(0.2)
            if self.left_warning > 10 and self.right_warning <= 10:
                print ('3, there is an obstacle on the left, turn right')
                twist.linear.x = 0.0
                twist.angular.z = -self.angular
                self.vel_pub.publish(twist)
                sleep(0.5)
        elif self.front_warning > 10 and self.left_warning > 10 and self.right_warning <= 10:
            print ('4. There is an obstacle in the middle left, turn right')
            twist.linear.x = 0.0
            twist.angular.z = -self.angular
            self.vel_pub.publish(twist)
            sleep(0.2)
            if self.left_warning <= 10 and self.right_warning > 10:
                print ('5, there is an obstacle on the left, turn left')
                twist.linear.x = 0.0
                twist.angular.z = self.angular
                self.vel_pub.publish(twist)
                sleep(0.5)
        elif self.front_warning > 10 and self.left_warning < 10 and self.right_warning < 10:
            print ('6, there is an obstacle in the middle, turn left')
            twist.linear.x = 0.0
            twist.angular.z = self.angular
            self.vel_pub.publish(twist)
            sleep(0.2)
        elif self.front_warning < 10 and self.left_warning > 10 and self.right_warning > 10:
            print ('7. There are obstacles on the left and right, turn right')
            twist.linear.x = 0.0
            twist.angular.z = -self.angular
            self.vel_pub.publish(twist)
            sleep(0.4)
        elif self.front_warning < 10 and self.left_warning > 10 and self.right_warning <= 10:
            print ('8, there is an obstacle on the left, turn right')
            twist.linear.x = 0.0
            twist.angular.z = -self.angular
            self.vel_pub.publish(twist)
            sleep(0.2)
        elif self.front_warning < 10 and self.left_warning <= 10 and self.right_warning > 10:
            print ('9, there is an obstacle on the right, turn left')
            twist.linear.x = 0.0
            twist.angular.z = self.angular
            self.vel_pub.publish(twist)
            sleep(0.2)
        elif self.front_warning <= 10 and self.left_warning <= 10 and self.right_warning <= 10:
            print ('10, no obstacles, go forward')
            twist.linear.x = self.linear
            twist.angular.z = 0.0
            self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Ros2botLidarAvoidanceNode()
    
    try:
        rclpy.spin(node)
    except Exception as ex:
        print(ex)
        print("ros2bot lidar avoidance node shutting down")
    finally:
        node.vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
