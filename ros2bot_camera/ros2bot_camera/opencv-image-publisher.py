#!/usr/bin/env python3

import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisher(Node):

    def __init__(self):
        # initiate the node class's constructor and give it a name
        super().__init__('image_publisher')   
        # create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        # publish a message every 0.1 seconds
        timer_period = 0.1  
        # create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # create a VideoCapture object, the argument '0' gets the default webcam
        self.cap = cv2.VideoCapture(0)     
        # used to convert between ROS and OpenCV images
        self.br = CvBridge()    

    def timer_callback(self):
        # capture frame-by-frame, this method returns True/False as well as the video frame.
        ret, frame = self.cap.read()        
        if ret == True:
           # publish the image, the 'cv2_to_imgmsg' method converts an OpenCV, image to a ROS 2 image message
           self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

        # display the message on the console
        self.get_logger().info('Publishing video frame')
                       
def main(args=None):
    # initialize the rclpy library
    rclpy.init(args=args)    
    # create the node
    image_publisher = ImagePublisher()   
    # spin the node so the callback function is called.
    rclpy.spin(image_publisher)   
    # destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    image_publisher.destroy_node()     
    # shutdown the ROS client library for Python
    rclpy.shutdown()    

if __name__ == '__main__':
    main()    

              