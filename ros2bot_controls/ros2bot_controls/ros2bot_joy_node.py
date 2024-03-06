#!/usr/bin/env python3

import time
import getpass
import rclpy

from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from rclpy.node import Node

class Ros2botJoyTelopNode(Node):
    def __init__(self):
        super().__init__("ros2bot_joy_node")
        self.get_logger().info("Hello from ros2bot joy node")

        # initialize state
        self.joy_active = False
        self.buzzer_active = False
        self.rgb_light_index = 0
        self.cancel_time = time.time()
        self.user_name = getpass.getuser()
        self.linear_gear = 1
        self.angular_gear = 1

        # declare parameters
        self.declare_parameter('xspeed_limit', 1.0)
        self.declare_parameter('yspeed_limit', 1.0)
        self.declare_parameter('angular_speed_limit', 5.0)

        # get parameters
        self.xspeed_limit = self.get_parameter('xspeed_limit')
        self.yspeed_limit = self.get_parameter('yspeed_limit')
        self.angular_speed_limit = self.get_parameter('angular_speed_limit')

        # create publishers
        self.goal_pub = self.create_publisher(GoalID, 'move_base/cancel', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.buzzer_pub = self.create_publisher(Bool, 'buzzer', 1)
        self.joy_state_pub = self.create_publisher(Bool, 'joy_state', 10)
        self.rgb_light_pub = self.create_publisher(Int32, 'rgb_light', 10)

        # create subscriptions
        self.sub_joy = self.create_subscription(Joy, 'joy', self.button_cb, 10)

    # callback function for joy subscription
    def button_cb(self, msg):
        if not isinstance(msg, Joy): 
            return

        if self.user_name == "jetson": # -or- 'root'
            self.user_jetson(msg)
        else: 
            self.user_pc(msg)

    def user_jetson(self, msg):
        # cancel
        if msg.axes[4] == -1: 
            self.cancel_nav()

        # rgb light
        if msg.buttons[7] == 1:
            if self.rgb_light_index < 6:
                for i in range(3): 
                    self.rgb_light_pub.publish(self.rgb_light_index)
            else: 
                self.rgb_light_index = 0
            self.rgb_light_index += 1

        # buzzer
        if msg.buttons[11] == 1:
            self.buzzer_active = not self.buzzer_active
            for i in range(3): 
                self.buzzer_pub.publish(self.buzzer_active)

        # linear gear control
        if msg.buttons[13] == 1:
            if self.linear_gear == 1.0: 
                self.linear_gear = 1.0 / 3
            elif self.linear_gear == 1.0 / 3: 
                self.linear_gear = 2.0 / 3
            elif self.linear_gear == 2.0 / 3: 
                self.linear_gear = 1
                
        # angular gear control
        if msg.buttons[14] == 1:
            if self.angular_gear == 1.0: 
                self.angular_gear = 1.0 / 4
            elif self.angular_gear == 1.0 / 4: 
                self.angular_gear = 1.0 / 2
            elif self.angular_gear == 1.0 / 2: 
                self.angular_gear = 3.0 / 4
            elif self.angular_gear == 3.0 / 4: 
                self.angular_gear = 1.0

        xlinear_speed = self.filter_data(msg.axes[1]) * self.xspeed_limit * self.linear_gear
        ylinear_speed = self.filter_data(msg.axes[0]) * self.yspeed_limit * self.linear_gear
        angular_speed = self.filter_data(msg.axes[2]) * self.angular_speed_limit * self.angular_gear

        if xlinear_speed > self.xspeed_limit: 
            xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit: 
            xlinear_speed = -self.xspeed_limit

        if ylinear_speed > self.yspeed_limit: 
            ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit: 
            ylinear_speed = -self.yspeed_limit

        if angular_speed > self.angular_speed_limit: 
            angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit: 
            angular_speed = -self.angular_speed_limit

        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed

        for i in range(3): 
            self.cmd_vel_pub.publish(twist)

    def user_pc(self, msg):
        if msg.axes[5] == -1: 
            self.cancel_nav()

        if msg.buttons[5] == 1:
            if self.rgb_light_index < 6:
                self.rgb_light_pub.publish(self.rgb_light_index)
            else: 
                self.rgb_light_index = 0
            self.rgb_light_index += 1

        if msg.buttons[7] == 1:
            self.buzzer_active = not self.buzzer_active
            self.buzzer_pub.publish(self.buzzer_active)

        # gear control
        if msg.buttons[9] == 1:
            if self.linear_gear == 1.0: 
                self.linear_gear = 1.0 / 3
            elif self.linear_gear == 1.0 / 3: 
                self.linear_gear = 2.0 / 3
            elif self.linear_gear == 2.0 / 3: 
                self.linear_gear = 1

        if msg.buttons[10] == 1:
            if self.angular_gear == 1.0: 
                self.angular_gear = 1.0 / 4
            elif self.angular_gear == 1.0 / 4: 
                self.angular_gear = 1.0 / 2
            elif self.angular_gear == 1.0 / 2: 
                self.angular_gear = 3.0 / 4
            elif self.angular_gear == 3.0 / 4: 
                self.angular_gear = 1.0

        xlinear_speed = self.filter_data(msg.axes[1]) * self.xspeed_limit * self.linear_gear
        ylinear_speed = self.filter_data(msg.axes[0]) * self.yspeed_limit * self.linear_gear
        angular_speed = self.filter_data(msg.axes[3]) * self.angular_speed_limit * self.angular_gear

        if xlinear_speed > self.xspeed_limit: 
            xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit: 
            xlinear_speed = -self.xspeed_limit

        if ylinear_speed > self.yspeed_limit: 
            ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit: 
            ylinear_speed = -self.yspeed_limit

        if angular_speed > self.angular_speed_limit: 
            angular_speed = self.angular_speed_limit            
        elif angular_speed < -self.angular_speed_limit: 
            angular_speed = -self.angular_speed_limit

        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed

        for i in range(3): 
            self.cmd_vel_pub.publish(twist)

    def filter_data(self, value):
        if abs(value) < 0.2: 
            value = 0
        return value

    def cancel_nav(self):
        # move_base, issue the move_base cancel command
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            self.joy_active = not self.joy_active
            for i in range(3):
                self.joy_state_pub.publish(Bool(self.joy_active))
                self.goal_pub.publish(GoalID())
                self.cmd_vel_pub.publish(Twist())
            self.cancel_time = now_time
  
def main(args=None):
    rclpy.init(args=args)
    node = Ros2botJoyTelopNode()
    
    try:
        rclpy.spin(node)
    except Exception as ex:
        print(ex)
        print("ros2bot joy node shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
