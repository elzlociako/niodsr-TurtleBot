#!/usr/bin/env python3

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from geometry_msgs.msg import Twist
import numpy as np
from cv_bridge import CvBridge # ROS2 package to convert between ROS and OpenCV Images
import cv2 # Python OpenCV library

def default_view():
    img = np.zeros((300,600,3), np.uint8)
    cv2.line(img,(200,0),(200,600),(255,0,0),4)
    cv2.line(img,(400,0),(400,600),(255,0,0),4)
    cv2.putText(img, 'Backwards', (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(img, 'Stop', (250,50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1, cv2.LINE_AA)
    cv2.putText(img, 'Forward', (450,50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1, cv2.LINE_AA)

    return img

default_img = default_view()
set_direction=0.0
class Publisher(Node):
    def __init__(self):
        super().__init__('pub')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def draw_circle(self, event, x, y, flags, param):
        global default_img, set_direction

        if event == cv2.EVENT_LBUTTONDOWN: 
            default_img = default_view()
            cv2.circle(default_img, (x, y), 25, (100,100,100), -1) 
            if x < 200:
                set_direction = -1.0
            if x > 400:
                set_direction = 1.0
            if x >= 200 and x <= 400:
                set_direction = 0.0

    def timer_callback(self):
        global default_img, set_direction

        pure_fun = Twist()
        pure_fun.linear.x = set_direction
        pure_fun.linear.y = 0.0
        pure_fun.linear.z = 0.0
        
        pure_fun.angular.x = 0.0  
        pure_fun.angular.y = 0.0  
        pure_fun.angular.z = 0.0    
        self.publisher_.publish(pure_fun)

        print(set_direction)

        # cv2.namedWindow("NiODSR")
        # cv2.setMouseCallback('NiODSR', self.draw_circle)
        cv2.imshow('NiODSR', default_img)
        cv2.setMouseCallback('NiODSR', self.draw_circle)

        self.get_logger().info(f'ACTUAL SPEED: {set_direction}')
        self.i += 1

        cv2.waitKey(1)       


def main(args=None):
    rclpy.init(args=args)

    pub = Publisher()
    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
