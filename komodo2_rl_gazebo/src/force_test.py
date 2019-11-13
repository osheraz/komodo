#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
import numpy as np

from scipy.ndimage.filters import gaussian_filter1d


def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth

class force_listener:
    def __init__(self):
        # start the node
        #rospy.init_node("listener")
        # subscribe to the '/sr_tactile/touch/ff' topic
        self.sub = rospy.Subscriber("/bucket_torque_sensor", WrenchStamped, self.callback)
        self.arr = []
        # keep python from exiting until this node is stopped
        #rospy.spin()

    def callback(self,msg):
        # log the message data to the terminal
        torque = msg.wrench.torque.y
        #rospy.loginfo(torque)
        self.arr.append(torque)

    def force_plot(self):
        self.sub.unregister()
        import matplotlib.pyplot as plt
        plt.plot(self.arr, 'o')
        plt.plot(smooth(self.arr, 3), 'r-', lw=2)
        plt.plot(smooth(self.arr, 19), 'g-', lw=2)
        plt.show()


if __name__ == '__main__':
    force_listener()
