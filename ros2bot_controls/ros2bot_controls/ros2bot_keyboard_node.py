#!/usr/bin/env python3

import sys
import select
import termios
import tty
import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node

class Ros2botKeyboardNode(Node):
    def __init__(self):
        super().__init__("ros2bot_keyboard_node")
        self.get_logger().info("Hello from ros2bot keyboard node")

        self.banner = """
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        | ROS2BOT KEYBOARD CONTROLLER |
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        MOVEMENT CONTROLS:
        -----------------

          Q    W    E
           \   |   /
          A ---+--- D
           /   |   \
          Z    X    C

        SPEED CONTROLS
        --------------

        U/J    : +/- MAX SPEED BY 10%
        I/K    : +/- LINEAR SPEED 10%
        O/L    : +/- ANGULAR SPEED BY 10%
        G      : X & Y SPEED SWITCH
        
        STOP CONTROLS
        -------------

        S      : SLOW STOP
        SPACE  : FORCE STOP       
        OTHER  : STOP CONTROL
        CTRL-C : EXIT
        """        
        
        # initialize move keys
        self.moveKeys = {
            'q': (1, 1),
            'Q': (1, 1),                     
            'w': (1, 0),
            'W': (1, 0),            
            'e': (1, -1),
            'E': (1, -1),            
            'a': (0, 1),
            'A': (0, 1),            
            'd': (0, -1),
            'D': (0, -1),
            'z': (-1, -1),  
            'Z': (-1, -1),                      
            'x': (-1, 0),
            'X': (-1, 0),
            'c': (-1, 1),
            'C': (-1, 1)            
        }

        # initialize speed keys
        self.speedKeys = {
            'U': (1.1, 1.1),
            'u': (1.1, 1.1),
            'J': (.9, .9),
            'j': (.9, .9),            
            'I': (1.1, 1),
            'i': (1.1, 1),
            'K': (.9, 1),
            'k': (.9, 1),
            'O': (1, 1.1),
            'o': (1, 1.1),
            'L': (1, .9),
            'l': (1, .9),
        }

        # initialize state
        self.xspeed_switch = True
        self.speed = 0.2
        self.turn = 1.0
        self.x = 0
        self.th = 0
        self.status = 0
        self.stop = False
        self.count = 0
        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = None

        # declare parameters
        self.declare_parameter('linear_limit', 1.0)
        self.declare_parameter('angular_limit', 5.0)
        self.declare_parameter('process_key_freq', 0.3)

        # get parameters
        self.linear_limit = self.get_parameter('linear_limit')
        self.angular_limit = self.get_parameter('angular_limit')
        self.process_key_freq = self.get_parameter('process_key_freq')

        # create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

    def start_timer(self):
        self.timer = self.create_timer(self.process_key_freq, self.process_key_cb)

    def cancel_timer(self):
        if not self.timer.is_canceled:
            self.timer.cancel()

    def getKey(self):
        # tty.setraw():raw；fileno():(fd)
        tty.setraw(sys.stdin.fileno())
        # select():fileno()
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist: 
            key = sys.stdin.read(1)
        else: 
            key = ''
        # tcsetattr tty
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self):
        return "currently:\tspeed %s\tturn %s " % (self.speed, self.turn)

    def drain_key_buffer(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def process_key_cb(self):
        if rclpy.ok:
            # read key
            key = self.getKey()

            # x & y speed switch
            if key == "g" or key == "G":
                self.xspeed_switch = not self.xspeed_switch
            # smooth stop
            elif key == 's' or key == 'S':
                self.count = self.count + 1
                if self.count > 4:
                    (self.x, self.th) = (0, 0)      
            # force stop
            elif key == ' ':
                (self.x, self.th) = (0, 0)                              

            # movement control
            if key in self.moveKeys.keys():
                self.x = self.moveKeys[key][0]
                self.th = self.moveKeys[key][1]
                self.count = 0
            # speed control
            elif key in self.speedKeys.keys():
                self.speed = self.speed * self.speedKeys[key][0]
                self.turn = self.turn * self.speedKeys[key][1]
                self.count = 0
                if self.speed > self.linear_limit:
                    self.speed = self.linear_limit
                if self.turn > self.angular_limit:
                    self.turn = self.angular_limit
                print(self.vels())
                if (self.status == 14):
                    print(self.banner)
                self.status = (self.status + 1) % 15
            # stop keyboard control
            else:
                print("keyboard control stopped: {}".format(not self.stop))
                self.stop = not self.stop
                # if exit key, cancel reocurring timer event and exit
                if (key == '\x03'):
                    self.cancel_timer()
                    return 

            # create and publish twist message
            twist = Twist()

            if self.xspeed_switch:
                twist.linear.x = self.speed * self.x
            else:
                twist.linear.y = self.speed * self.x

            twist.angular.z = self.turn * self.th

            if not self.stop:
                self.cmd_vel_pub.publish(twist)
  
def main(args=None):
    rclpy.init(args=args)
    node = Ros2botKeyboardNode()
    
    try:
        print(node.banner)
        print(node.vels())
        node.start_timer()        
        rclpy.spin(node)
    except Exception as ex:
        print(ex)
        print("ros2bot keyboard node shutting down")
    finally:
        node.cancel_timer()
        node.drain_key_buffer()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
