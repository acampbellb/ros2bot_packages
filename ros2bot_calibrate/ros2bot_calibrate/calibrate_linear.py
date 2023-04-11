#!/usr/bin/env python3

# Calibrate Linear
# Usage: Move the robot 1.0 meter to check on the PID parameters of the base controller.

import rclpy

from time import time
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry

class CalibrateLinear():

    def __init__(self):
        super().__init__("calibrate_linear_node")
        self.declare_parameters(
                    namespace='',
                    parameters=[
                        ('direction', None),
                        ('test_distance', None),
                        ('speed', None),
                        ('tolerance', None),
                        ('odom_linear_scale_correction', None),
                        ('start_test', None)
                    ])   

def main(args=None):
    rclpy.init(args=args)
    node = CalibrateLinear()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()             