#!/usr/bin/env python3

import rclpy

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool
from ros2bot_master_lib import Ros2botMasterDriver
from ros2bot_speach_lib import Ros2botSpeachDriver
from rclpy.node import Node

class Ros2botSpeachDriverNode(Node):
    def __init__(self):
        super().__init__("ros2bot_speach_driver_node")
        self.get_logger().info("Hello from ros2bot speach driver node")

        # load libraries
        self.master = Ros2botMasterDriver()
        self.speach = Ros2botSpeachDriver()

        # declare parameters
        self.declare_parameter('process_cmd_freq', 0.3)

        # get parameters
        self.process_cmd_freq = self.get_parameter('process_cmd_freq')

        # create subscriptions
        self.sub_cmd_velocity = self.create_subscription(Twist, 'cmd_vel', self.cmd_velocity_cb, 1)
        self.sub_joy_state = self.create_subscription(Bool, '/joy_state', self.joy_state_cb)

        # create publishers
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.joy_active = False

    def start_timer(self):
        self.timer = self.create_timer(self.process_cmd_freq, self.process_cmd_cb)

    def cancel_timer(self):
        if not self.timer.is_canceled:
            self.timer.cancel()

    def cmd_velocity_cb(self, msg):
        self.get_logger().info('ros2bot_speach_driver node heard velocity msg: "%s"' % msg.data)

		# car motion control, subscriber callback function
        if not isinstance(msg, Twist): 
            return

		# issue linear and angular velocity
        vx = msg.linear.x
        vy = msg.linear.y
        angular = msg.angular.z

		# linear: ±1, angular: ±5
		# trolley motion control,vesl=[-1, 1], angular=[-5, 5]
        self.master.set_car_motion(vx, vy, angular)

    def joy_state_cb(self, msg):
        self.get_logger().info('ros2bot_speach_driver node heard joy state msg: "%s"' % msg.data)

        if not isinstance(msg, Bool): 
            return
            
        self.joy_active = msg.data
        self.velocity_pub.publish(Twist())

    def process_cmd_cb(self):
        if rclpy.ok():
            speech_r = self.speach.speech_read()

            if speech_r == 2:
                vx = 0.0
                vy = 0.0
                angular = 0
                self.master.set_car_motion(vx, vy, angular)
                self.speach.void_write(speech_r)
            elif speech_r == 4:
                vx = 0.5
                vy = 0.0
                angular = 0
                self.master.set_car_motion(vx, vy, angular)
                self.speach.void_write(speech_r)
            elif speech_r == 5:
                vx = -0.5
                vy = 0.0
                angular = 0
                self.master.set_car_motion(vx, vy, angular)
                self.speach.void_write(speech_r)
            elif speech_r == 6:
                vx = 0.2
                vy = 0.0
                angular = 0.5
                self.master.set_car_motion(vx, vy, angular)
                self.speach.void_write(speech_r)
            elif speech_r == 7:
                vx = 0.2
                vy = 0.0
                angular = -0.5
                self.master.set_car_motion(vx, vy, angular) 
                self.speach.void_write(speech_r)
            elif speech_r == 8:
                vx = 0.0
                vy = 0.0
                angular = 0.5
                self.master.set_car_motion(vx, vy, angular)
                self.speach.void_write(speech_r)
            elif speech_r == 9:
                vx = 0.0
                vy = 0.0
                angular = -0.5
                self.master.set_car_motion(vx, vy, angular)
                self.speach.void_write(speech_r)
            elif speech_r == 11:
                self.master.set_colorful_lamps(0xFF,255,0,0)
                self.speach.void_write(speech_r)
            elif speech_r == 17:
                self.master.set_colorful_effect(3, 6, parm=1)
                self.speach.void_write(speech_r)
            elif speech_r == 10:
                self.master.set_colorful_effect(0, 6, parm=1)
                self.speach.void_write(speech_r)
            elif speech_r == 12:
                self.master.set_colorful_lamps(0xFF,0,255,0)
                self.speach.void_write(speech_r)
            elif speech_r == 13:
                self.master.set_colorful_lamps(0xFF,0,0,255)
                self.speach.void_write(speech_r)
            elif speech_r == 14:
                self.master.set_colorful_lamps(0xFF,255,255,0)
                self.speach.void_write(speech_r)
            elif speech_r == 15:
                self.master.set_colorful_effect(1, 6, parm=1)
                self.speach.void_write(speech_r)
            elif speech_r == 16:
                self.master.set_colorful_effect(4, 6, parm=1)
                self.speach.void_write(speech_r)
            elif speech_r == 17:
                self.master.set_colorful_effect(3, 6, parm=1)
                self.speach.void_write(speech_r)
            elif speech_r == 18:
                self.master.set_colorful_effect(1, 6, parm=1)
                self.speach.void_write(speech_r)
            elif speech_r == 32:
                self.speach.void_write(speech_r)
            elif speech_r == 33:
                self.speach.void_write(speech_r)
            elif speech_r == 34:
                self.speach.void_write(speech_r)
            elif speech_r == 35:
                self.speach.void_write(speech_r)
            elif speech_r == 36:
                self.speach.void_write(speech_r)
            elif speech_r == 37:
                self.speach.void_write(speech_r)
  
def main(args=None):
    rclpy.init(args=args)
    node = Ros2botSpeachDriverNode()
    
    try:
        node.start_timer()
        rclpy.spin(node)
    except Exception as ex:
        print(ex)
        print("ros2bot speach driver node shutting down")
    finally:
        node.cancel_timer()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
