#!/usr/bin/env python3

# Calibrate Linear
# Usage: Move the robot 1.0 meter to check on the PID parameters of the base controller.

import rclpy
import tf2_ros

from time import time
from math import copysign, sqrt, pow
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformException

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
        rclpy.spin_once(self)
        self.tfbuf.can_transform(self.base_frame, self.odom_frame, rclpy.time.Time(), Duration(seconds=60))    
        self.get_logger().info('linear calibration starting ...')    
        # get the starting position from the tf2 transform between the odom and base frames
        self.position = Point()
        self.position = self.get_position()
        x_start = self.position.x
        y_start = self.position.y   
        # start calibration
        while rclpy.ok():
            # stop the robot by default
            move_cmd = Twist()            
            if self.start_test:
                start = time()
                # get the current position from the tf transform between the odom and base frames
                self.position = self.get_position()                
                # compute the euclidean distance from the target point
                distance = sqrt(pow((self.position.x - x_start), 2) +
                                pow((self.position.y - y_start), 2))   
                # correct the estimated distance by the correction factor
                distance *= self.odom_linear_scale_correction                             
                # how close are we?
                error = distance - self.test_distance    
                # are we close enough?
                if not self.start_test or abs(error) < self.tolerance:
                    self.start_test = False
                    # TODO: dynamic parameters
                    # params = {'start_test': False}
                    # dyn_client.update_configuration(params)
                else:
                    # if not, move in the appropriate direction
                    if self.direction:
                        move_cmd.linear.x = copysign(self.speed, -1 * error)
                    else:
                        move_cmd.linear.y = copysign(self.speed, -1 * error)   
                # DEBUG OUTPUT
                # print("error: ", error)
                # print("self.speed: ", self.speed)
                # print("x: ", move_cmd.linear.x)  
                self.cmd_pub.publish(move_cmd)
                end = time()
                # print(f'time: {start-end}, distance: {distance}, test_distance: {self.test_distance}, position: {self.position}')
            else:
                self.position = self.get_position()
                x_start = self.position.x
                y_start = self.position.y
                self.cmd_pub.publish(Twist())     
            rclpy.spin_once(self)
        # stop robot
        self.cmd_pub.publish(Twist())           

    def get_position(self):
        # get the current transform between the odom and base frames               
        try:
            t = self.tfbuf.lookup_transform(self.base_frame, self.odom_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'transform exception {ex}')  
            return
        vector = t.transform.translation
        return Point(*vector)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = CalibrateLinear()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('stopping robot...')
        node.cmd_vel.publish(Twist())  
        rclpy.spin_once(node)      
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()             