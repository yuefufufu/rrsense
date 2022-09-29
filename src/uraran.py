#! /usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

red_val: float = 1
yellow_val: float = 1


def callback_uragon(uragon):
        pub = rospy.Publisher("uraran", Twist, queue_size=50)
        uraran = Twist()
        #uraran.linear.x = uragon.linear.x*yellow_val/red_val
        #uraran.linear.y = uragon.linear.y
        #uraran.linear.z = uragon.linear.z
        #uraran.angular.x = uragon.angular.x
        #uraran.angular.y = uragon.angular.y
        #uraran.angular.z = uragon.angular.z*yellow_val/red_val        
        pub.publish(uraran)        
        print(uragon.linear.x,uragon.angular.z)
        
        
        


def subscriber():
    rospy.init_node("uraran")
    rospy.Subscriber("uragon", Twist, callback_uragon)
    #rospy.Subscriber("red", Float64, callback_red)
    #rospy.Subscriber("yellow", Float64, callback_yellow)

    rospy.spin()


if __name__ == "__main__":
    subscriber()
