#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

def callback(msg):
    rospy.loginfo(msg.ranges[270])  # len is 540

rospy.init_node('distance')
sub = rospy.Subscriber('/scan',LaserScan, callback)
rospy.spin()