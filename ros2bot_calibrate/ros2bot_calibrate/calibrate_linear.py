#!/usr/bin/env python3

# Calibrate Linear
# Usage: Move the robot 1.0 meter to check on the PID parameters of the base controller.

import rclpy
import tf2_ros

from time import time
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node

class CalibrateLinear(Node):

    def __init__(self):
        super().__init__("calibrate_linear_node")
        self.declare_parameters(
                    namespace='',
                    parameters=[
                        ('rate', None),
                        ('direction', None),
                        ('test_distance', None),
                        ('speed', None),
                        ('tolerance', None),
                        ('odom_linear_scale_correction', None),
                        ('start_test', None),
                        ('base_frame', None),
                        ('odom_frame', None)                        
                    ])   
        self.rate = self.get_parameter('rate')        
        self.direction = self.get_parameter('direction')
        self.test_distance = self.get_parameter('test_distance')
        self.speed = self.get_parameter('speed')
        self.tolerance = self.get_parameter('tolerance')
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction')
        self.start_test = self.get_parameter('start_test')
        self.base_frame = self.get_parameter('base_frame')
        self.odom_frame = self.get_parameter('odom_frame')    
        # create publisher to control robot speed
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 5)  
        # create transform listener
        self.tfbuf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfbuf)                      

def main(args=None):
    rclpy.init(args=args)
    node = CalibrateLinear()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()             