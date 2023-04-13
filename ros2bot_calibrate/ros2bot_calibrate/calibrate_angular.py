#!/usr/bin/env python3

# Calbrate Angular
# Usage: Rotate the robot 360 degrees to check the odometry parameters of the base controller.

import rclpy
import tf2_ros

from time import time
from math import radians, copysign
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from transform_utils import quat_to_angle, normalize_angle
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformException

class CalibrateAngular(Node):

    def __init__(self):
        super().__init__("calibrate_angular_node")
        self.declare_parameters(
                    namespace='',
                    parameters=[
                        ('rate', None),
                        ('test_angle', None),
                        ('speed', None),
                        ('tolerance', None),
                        ('odom_angular_scale_correction', None),
                        ('start_test', None),
                        ('base_frame', None),
                        ('odom_frame', None)
                    ])   
        self.rate = self.get_parameter('rate')
        self.test_angle = self.get_parameter('test_angle')
        self.speed = self.get_parameter('speed')
        self.tolerance = self.get_parameter('tolerance')
        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction')
        self.start_test = self.get_parameter('start_test')
        self.base_frame = self.get_parameter('base_frame')
        self.odom_frame = self.get_parameter('odom_frame')
        # create publisher to control robot speed
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 5)   
        # create transform listener
        self.tfbuf = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfbuf)     
        rclpy.spin_once(self)
        self.tfbuf.can_transform(self.base_frame, self.odom_frame, rclpy.time.Time(), Duration(seconds=60))
        reverse = 1
        while rclpy.ok():
            if self.start_test:
                # get the current rotation angle from tf
                self.odom_angle = self.get_odom_angle()
                last_angle = self.odom_angle
                turn_angle = 0
                self.test_angle *= reverse
                error = self.test_angle - turn_angle     
                # alternate directions between tests
                reverse = -reverse      
                while abs(error) > self.tolerance and self.start_test:
                    start = time()                     
                    if not rclpy.ok() : return
                    # rotate the robot to reduce the error
                    move_cmd = Twist()
                    move_cmd.angular.z = copysign(self.speed, error)
                    self.cmd_vel.publish(move_cmd)
                    rclpy.spin_once(self)
                    # get the current rotation angle from tf
                    self.odom_angle = self.get_odom_angle()                                        
                    # compute how far we have gone since the last measurement
                    delta_angle = self.odom_angular_scale_correction * normalize_angle(self.odom_angle - last_angle)                    
                    # add to our total angle so far
                    turn_angle += delta_angle
                    # compute the new error
                    error = self.test_angle - turn_angle
                    # ztore the current angle for the next comparison
                    last_angle = self.odom_angle
                    end = time()                    
                    self.get_logger().info(f'Time:{start-end}, test angle:{self.test_angle}, turn_angle:{turn_angle}')
                # update status flag
                self.start_test = False
                # TODO: dynamic parameter
                # params = {'start_test': False}
                # dyn_client.update_configuration(params)                
                # stop robot
                self.cmd_vel.publish(Twist())
            rclpy.spin_once(self)
        # stop robot
        self.cmd_vel.publish(Twist())

    def get_odom_angle(self):
        # get the current transform between the odom and base frames
        try:
            t = self.tfbuf.lookup_transform(self.base_frame, self.odom_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.odom_frame} to {self.base_frame}: {ex}')            
            return
        quat = t.transform.rotation
        return quat_to_angle(quat)        

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CalibrateAngular()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('stopping robot...')
        node.cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()          