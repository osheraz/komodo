#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Float32
from sensor_msgs.msg import Imu, LaserScan

from geometry_msgs.msg import Twist
import numpy as np


class checks:
    def __init__(self):
        rospy.init_node('data')

        # sub = rospy.Subscriber('/scan',LaserScan, self.callback)
        # force_subscriber = rospy.Subscriber('/arm/calibrated_force', Float32, self.update_force)
        self.imu_subscriber = rospy.Subscriber('/IMU', Imu, self.imu_subscriber_callback)

        self.ori_arr = np.zeros([4])
        self.ang_arr = np.zeros([3])
        self.lin_arr = np.zeros([3])
        rospy.spin()

    def callback(self,msg):
        rospy.loginfo("lidar: " + str(msg.ranges[270]))  # len is 540

    def update_force(self,msg):
        rospy.loginfo("force: " + str(msg.data))  # len is 540

    def imu_subscriber_callback(self,imu):
        orientation = np.array([imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w])
        rospy.loginfo("orien: "  + str(np.round(orientation,3)))
        angular_vel = np.array([imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z])
        rospy.loginfo("angular: "  + str(np.round(angular_vel,3)))
        linear_acc = np.array([imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z])
        rospy.loginfo("linear: "  + str(np.round(linear_acc,3)))
        self.ori_arr = np.vstack((self.ori_arr,  orientation))
        self.ang_arr = np.vstack((self.ang_arr,  angular_vel))
        self.lin_arr = np.vstack((self.lin_arr,  linear_acc))
        if self.lin_arr.shape[0] == 100:
            np.save('lin',self.lin_arr)
            np.save('ori',self.ang_arr)
            np.save('ang',self.ang_arr)


def plottt():
    lin = np.load('lin.npy')
    ori = np.load('ori.npy')
    ang = np.load('ang.npy')

    lin_sim = np.load('lin_sim.npy')
    ori_sim = np.load('ori_sim.npy')
    ang_sim = np.load('ang_sim.npy')

    import matplotlib.pyplot as plt
    plt.figure(1)
    plt.plot(lin[60:,2])
    plt.plot(-lin_sim[:,2])
    plt.show()
    # plt.figure(2)
    # plt.plot(ori[:][:])
    # plt.plot(ori_sim[:][:])
    # plt.show()
    # plt.figure(3)
    # plt.plot(ang[:][:])
    # plt.plot(ang_sim[:][:])
    # plt.show()

if __name__ == '__main__':
    try:
        # checks()
        plottt()

    except rospy.ROSInterruptException:
        pass