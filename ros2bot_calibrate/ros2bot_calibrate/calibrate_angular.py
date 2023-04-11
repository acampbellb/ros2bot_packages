#!/usr/bin/env python3

# Calbrate Angular
# Usage: Rotate the robot 360 degrees to check the odometry parameters of the base controller.

import rclpy

class CalibrateAngular():

    def __init__(self):
        super().__init__("calibrate_angular_node")
        self.declare_parameters(
                    namespace='',
                    parameters=[
                        ('test_angle', None),
                        ('speed', None),
                        ('tolerance', None),
                        ('odom_angular_scale_correction', None),
                        ('start_test', None)
                    ])   

def main(args=None):
    rclpy.init(args=args)
    node = CalibrateAngular()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()          