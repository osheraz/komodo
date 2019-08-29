#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
    rospy.loginfo(msg.ranges[360])

rospy.init_node('distance')
sub = rospy.Subscriber('/scan',LaserScan, callback)
rospy.spin()