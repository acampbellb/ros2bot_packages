#!/usr/bin/env python3

import rclpy

import math
import numpy as np

from pid import SinglePID
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.node import Node

class Ros2botLidarTrackerNode(Node):
    def __init__(self):
        super().__init__("ros2bot_lidar_tracker_node")
        self.get_logger().info("Hello from ros2bot lidar tracker node")

        # initialize state
        self.RA2DE = 180 / math.pi
        self.moving = False
        self.joy_active = False
        
        # declare parameters
        self.declare_parameter('switch', False)
        self.declare_parameter('laser_angle', 40.0)
        self.declare_parameter('priority_angle', 30.0)
        self.declare_parameter('response_dist', 0.55)
        self.declare_parameter('linear_kp', 2.0)
        self.declare_parameter('linear_ki', 0.0)
        self.declare_parameter('linear_kd', 2.0)
        self.declare_parameter('angle_kp', 3.0)
        self.declare_parameter('angle_ki', 0.0)
        self.declare_parameter('angle_kd', 5.0)

        # get parameters
        self.switch = self.get_parameter('switch')
        self.laser_angle = self.get_parameter('laser_angle')
        self.priority_angle = self.get_parameter('priority_angle')
        self.response_dist = self.get_parameter('response_dist')
        self.linear_kp = self.get_parameter('linear_kp')
        self.linear_ki = self.get_parameter('linear_ki')
        self.linear_kd = self.get_parameter('linear_kd')
        self.angle_kp = self.get_parameter('angle_kp')
        self.angle_ki = self.get_parameter('angle_ki')
        self.angle_kd = self.get_parameter('angle_kd')

        # initialize pids
        self.linear_pid = SinglePID(self.linear_kp, self.linear_ki, self.linear_kd)
        self.angle_pid = SinglePID(self.angle_kp, self.angle_ki, self.angle_kd)

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

        # record the laser scan and publish the position of the nearest object (or point to a point)
        ranges = np.array(msg.ranges)
        offset = 0.5
        front_dist_list = []
        front_dist_id_list = []
        min_dist_list = []
        min_dist_id_list = []
        
        # if we already have a last scan to compare to:
        for i in range(len(ranges)):
            angle = (msg.angle_min + msg.angle_increment * i) * self.RA2DE
            # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, msg.ranges[i])
            if abs(angle) > (180 - self.priority_angle):
                if ranges[i] < (self.response_dist + offset):
                    front_dist_list.append(ranges[i])
                    front_dist_id_list.append(angle)
            elif (180 - self.laser_angle) < angle < (180 - self.priority_angle):
                min_dist_list.append(ranges[i])
                min_dist_id_list.append(angle)
            elif (self.priority_angle - 180) < angle < (self.laser_angle - 180):
                min_dist_list.append(ranges[i])
                min_dist_id_list.append(angle)

        # find the minimum distance and the ID corresponding to the minimum distance
        if len(front_dist_id_list) != 0:
            min_dist = min(front_dist_list)
            min_dist_id = front_dist_id_list[front_dist_list.index(min_dist)]
        else:
            min_dist = min(min_dist_list)
            min_dist_id = min_dist_id_list[min_dist_list.index(min_dist)]

        # rospy.loginfo('minDist: {}, minDistID: {}'.format(minDist, minDistID))
        if self.joy_ctrl.joy_active or self.switch == True:
            if self.moving == True:
                self.vel_pub.publish(Twist())
                self.moving = not self.moving
            return

        self.moving = True
        velocity = Twist()

        if abs(min_dist - self.response_dist) < 0.1: 
            min_dist = self.response_dist

        velocity.linear.x = -self.linear_pid.pid_compute(self.response_dist, min_dist)
        angle_pid_compute = self.angle_pid.pid_compute((180 - abs(min_dist_id)) / 72, 0)

        if min_dist_id > 0: 
            velocity.angular.z = -angle_pid_compute
        else: 
            velocity.angular.z = angle_pid_compute

        if angle_pid_compute < 0.02: 
            velocity.angular.z = 0.0

        self.vel_pub.publish(velocity)

def main(args=None):
    rclpy.init(args=args)
    node = Ros2botLidarTrackerNode()
    
    try:
        rclpy.spin(node)
    except Exception as ex:
        print(ex)
        print("ros2bot lidar tracker node shutting down")
    finally:
        node.vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()