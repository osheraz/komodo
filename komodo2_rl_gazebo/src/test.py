#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float32

from geometry_msgs.msg import Twist
import numpy as np

def callback(msg):
    rospy.loginfo("lidar: " + str(msg.ranges[270]))  # len is 540

def update_force(msg):
    rospy.loginfo("force: " + str(msg.data))  # len is 540

rospy.init_node('data')
sub = rospy.Subscriber('/scan',LaserScan, callback)
force_subscriber = rospy.Subscriber('/arm/calibrated_force', Float32, update_force)

rospy.spin()